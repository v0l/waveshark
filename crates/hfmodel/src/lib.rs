//! Fetching a model off Hugging Face, once, with somewhere to report to.
//!
//! Both speech models want the same three or four files and the same
//! progress: a config, a tokenizer, and weights that are either one
//! safetensors file or an index and the shards it names. What differs is
//! which files and what is done with them afterwards, so this hands over a
//! [`Fetch`] that knows the repository and the directory, and the caller asks
//! it for the files it needs.
//!
//! The files are fetched once, or placed by hand, and after that the model is
//! as offline as demodulation is.

use common::{Error, Result};
use std::path::{Path, PathBuf};

/// How far a download has got, reported as it goes.
///
/// A model is between 74 MB and several gigabytes over somebody's home
/// connection, and without this an interface can only say "downloading" for
/// as long as it takes, which is indistinguishable from a fetch that has
/// hung.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fetching {
    /// The file being fetched now.
    pub file: String,
    /// Bytes of it that have arrived, and what it is altogether. The total
    /// is zero until the hub answers with a length.
    pub done: u64,
    pub total: u64,
    /// Files finished before this one, and how many there are to do. The
    /// count is only known as the fetch walks the repository, so it grows.
    pub files_done: usize,
    pub files: usize,
}

/// Somewhere to report progress to. A closure rather than a trait, because
/// the callers keep it behind a mutex an interface reads.
pub type OnProgress<'a> = &'a mut dyn FnMut(&Fetching);

const HUB: &str = "https://huggingface.co";
const CONNECT: std::time::Duration = std::time::Duration::from_secs(30);

/// One repository, one directory, and a running report.
pub struct Fetch<'a> {
    client: httpc::BlockingClient,
    base: String,
    dir: PathBuf,
    seen: std::cell::RefCell<Fetching>,
    on: std::cell::RefCell<OnProgress<'a>>,
}

impl<'a> Fetch<'a> {
    pub fn new(
        repo: &str,
        revision: &str,
        dir: impl AsRef<Path>,
        on: OnProgress<'a>,
    ) -> Result<Self> {
        Self::at(format!("{HUB}/{repo}/resolve/{revision}"), dir, on)
    }

    /// The same, for a repository the hub files under datasets rather than
    /// models: a pronunciation dictionary is data, and is published as such.
    pub fn dataset(
        repo: &str,
        revision: &str,
        dir: impl AsRef<Path>,
        on: OnProgress<'a>,
    ) -> Result<Self> {
        Self::at(format!("{HUB}/datasets/{repo}/resolve/{revision}"), dir, on)
    }

    fn at(base: String, dir: impl AsRef<Path>, on: OnProgress<'a>) -> Result<Self> {
        let dir = dir.as_ref().to_path_buf();
        std::fs::create_dir_all(&dir)?;
        let client =
            httpc::blocking_download(CONNECT).map_err(|e| Error::other(format!("hub: {e}")))?;
        Ok(Self {
            client,
            base,
            dir,
            seen: std::cell::RefCell::new(Fetching::default()),
            on: std::cell::RefCell::new(on),
        })
    }

    fn report(&self, change: impl FnOnce(&mut Fetching)) {
        change(&mut self.seen.borrow_mut());
        (self.on.borrow_mut())(&self.seen.borrow());
    }

    /// One file, into the directory. Already there and it is not fetched
    /// again.
    pub fn get(&self, name: &str) -> Result<PathBuf> {
        let dst = self.dir.join(name);
        if dst.is_file() {
            return Ok(dst);
        }
        if let Some(parent) = dst.parent() {
            std::fs::create_dir_all(parent)?;
        }
        self.report(|f| {
            f.file = name.to_string();
            f.done = 0;
            f.total = 0;
            f.files = f.files.max(f.files_done + 1);
        });
        let url = format!("{}/{name}", self.base);
        let fail = |e: String| Error::other(format!("hub {name}: {e}"));
        let mut resp = self
            .client
            .get(&url)
            .send()
            .and_then(|r| r.error_for_status())
            .map_err(|e| fail(e.to_string()))?;
        let total = resp.content_length().unwrap_or(0);
        self.report(|f| f.total = total);
        let part = dst.with_file_name(format!(
            "{}.part",
            dst.file_name().and_then(|n| n.to_str()).unwrap_or("download")
        ));
        let mut out = std::io::BufWriter::new(std::fs::File::create(&part)?);
        let mut buf = vec![0u8; 256 * 1024];
        loop {
            let n = std::io::Read::read(&mut resp, &mut buf).map_err(|e| fail(e.to_string()))?;
            if n == 0 {
                break;
            }
            std::io::Write::write_all(&mut out, &buf[..n])?;
            self.report(|f| f.done += n as u64);
        }
        std::io::Write::flush(&mut out)?;
        drop(out);
        std::fs::rename(&part, &dst)?;
        self.report(|f| {
            f.files_done += 1;
            f.total = f.total.max(f.done);
            f.done = f.total;
        });
        Ok(dst)
    }

    /// The weights, however the repository publishes them: one safetensors
    /// file, or an index and every shard it names. Answers with the file a
    /// loader is given, which for a sharded model is the index.
    ///
    /// Asking the hub which rather than reading the listing: a 404 on the
    /// index is the answer.
    pub fn weights(&self) -> Result<PathBuf> {
        match self.get("model.safetensors.index.json") {
            Ok(index) => {
                for shard in shards(&index) {
                    self.get(&shard)?;
                }
                Ok(index)
            }
            Err(_) => self.get("model.safetensors"),
        }
    }
}

/// The shard files an index names, or nothing for a file that is not one.
pub fn shards(index: &Path) -> Vec<String> {
    if index.file_name().and_then(|n| n.to_str()) != Some("model.safetensors.index.json") {
        return Vec::new();
    }
    let Ok(text) = std::fs::read_to_string(index) else {
        return Vec::new();
    };
    let Ok(v) = serde_json::from_str::<serde_json::Value>(&text) else {
        return Vec::new();
    };
    let mut out: Vec<String> = v
        .get("weight_map")
        .and_then(|m| m.as_object())
        .map(|m| m.values().filter_map(|v| v.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    out.sort();
    out.dedup();
    out
}

/// What a model takes on disc: the files given, plus the shards an index
/// among them names.
pub fn bytes_of(files: &[PathBuf]) -> u64 {
    let mut all = files.to_vec();
    for f in files {
        if let Some(dir) = f.parent() {
            all.extend(shards(f).into_iter().map(|s| dir.join(s)));
        }
    }
    all.into_iter().filter_map(|p| std::fs::metadata(p).ok()).map(|m| m.len()).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_index_names_its_shards_once_and_in_order() {
        let dir = std::env::temp_dir().join(format!("hfmodel-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory");
        let index = dir.join("model.safetensors.index.json");
        std::fs::write(
            &index,
            r#"{"weight_map":{"b":"model-00002-of-00002.safetensors",
                              "a":"model-00001-of-00002.safetensors",
                              "c":"model-00002-of-00002.safetensors"}}"#,
        )
        .expect("an index");
        assert_eq!(
            shards(&index),
            ["model-00001-of-00002.safetensors", "model-00002-of-00002.safetensors"]
        );
        // Anything else is not an index, whatever it holds.
        let other = dir.join("config.json");
        std::fs::write(&other, "{}").expect("a config");
        assert!(shards(&other).is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn hub(
        files: &'static [(&'static str, &'static str)],
    ) -> (String, std::sync::mpsc::Receiver<String>) {
        use std::io::{BufRead, BufReader, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let base = format!("http://{}/org/model/resolve/main", listener.local_addr().unwrap());
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for sock in listener.incoming().flatten() {
                let mut reader = BufReader::new(sock.try_clone().unwrap());
                let mut first = String::new();
                reader.read_line(&mut first).unwrap();
                loop {
                    let mut line = String::new();
                    if reader.read_line(&mut line).unwrap_or(0) == 0 || line == "\r\n" {
                        break;
                    }
                }
                let path = first.split(' ').nth(1).unwrap_or_default().to_string();
                let _ = tx.send(path.clone());
                let found = files.iter().find(|(name, _)| path.ends_with(&format!("/{name}")));
                let answer = match found {
                    Some((_, body)) => format!(
                        "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                        body.len()
                    ),
                    None => {
                        "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
                            .to_string()
                    }
                };
                let _ = reader.into_inner().write_all(answer.as_bytes());
            }
        });
        (base, rx)
    }

    #[test]
    fn weights_without_an_index_are_the_one_file_and_a_file_held_is_not_fetched_again() {
        let (base, asked) = hub(&[
            ("config.json", "{}"),
            ("model.safetensors", "0123456789"),
            ("voices/af.bin", "v"),
        ]);
        let dir = std::env::temp_dir().join(format!("hfmodel-fetch-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut reports = Vec::new();
        let mut on = |f: &Fetching| reports.push(f.clone());
        let fetch = Fetch::at(base, &dir, &mut on).unwrap();
        let weights = fetch.weights().unwrap();
        assert_eq!(weights, dir.join("model.safetensors"));
        assert_eq!(std::fs::read(&weights).unwrap(), b"0123456789");
        assert_eq!(fetch.get("voices/af.bin").unwrap(), dir.join("voices/af.bin"));
        assert_eq!(fetch.get("config.json").unwrap(), dir.join("config.json"));
        assert_eq!(fetch.get("config.json").unwrap(), dir.join("config.json"));
        assert!(fetch.get("missing.json").is_err());
        drop(fetch);
        let paths: Vec<String> = asked.try_iter().collect();
        assert_eq!(
            paths,
            [
                "/org/model/resolve/main/model.safetensors.index.json",
                "/org/model/resolve/main/model.safetensors",
                "/org/model/resolve/main/voices/af.bin",
                "/org/model/resolve/main/config.json",
                "/org/model/resolve/main/missing.json",
            ],
            "the second config.json came off the disc"
        );
        let last = reports.iter().rfind(|f| f.file == "model.safetensors").unwrap();
        assert_eq!((last.done, last.total), (10, 10));
        assert_eq!(reports.last().map(|f| f.files_done), Some(3), "three files arrived");
        assert!(!dir.join("model.safetensors.index.json").exists());
        assert!(!dir.join("missing.json.part").exists());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
