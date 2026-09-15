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

/// What the hub calls as bytes arrive, folded into a [`Fetching`] and handed
/// on. Held by reference so one report survives every file of a fetch.
struct Report<'a, 'b> {
    seen: &'a std::cell::RefCell<Fetching>,
    on: &'a std::cell::RefCell<OnProgress<'b>>,
}

impl hf_hub::api::Progress for Report<'_, '_> {
    fn init(&mut self, size: usize, filename: &str) {
        let mut f = self.seen.borrow_mut();
        f.file = filename.to_string();
        f.done = 0;
        f.total = size as u64;
        (self.on.borrow_mut())(&f);
    }

    fn update(&mut self, size: usize) {
        let mut f = self.seen.borrow_mut();
        f.done += size as u64;
        (self.on.borrow_mut())(&f);
    }

    fn finish(&mut self) {
        let mut f = self.seen.borrow_mut();
        f.done = f.total;
        (self.on.borrow_mut())(&f);
    }
}

/// One repository, one directory, and a running report.
pub struct Fetch<'a> {
    api: hf_hub::api::sync::ApiRepo,
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
        use hf_hub::api::sync::ApiBuilder;
        let dir = dir.as_ref().to_path_buf();
        std::fs::create_dir_all(&dir)?;
        let api = ApiBuilder::new().build().map_err(|e| Error::other(format!("hub: {e}")))?.repo(
            hf_hub::Repo::with_revision(
                repo.to_string(),
                hf_hub::RepoType::Model,
                revision.to_string(),
            ),
        );
        Ok(Self {
            api,
            dir,
            seen: std::cell::RefCell::new(Fetching::default()),
            on: std::cell::RefCell::new(on),
        })
    }

    /// One file, into the directory. Already there and it is not fetched
    /// again.
    pub fn get(&self, name: &str) -> Result<PathBuf> {
        {
            let mut f = self.seen.borrow_mut();
            f.file = name.to_string();
            f.done = 0;
            f.total = 0;
            f.files = f.files.max(f.files_done + 1);
        }
        let src = self
            .api
            .download_with_progress(name, Report { seen: &self.seen, on: &self.on })
            .map_err(|e| Error::other(format!("hub {name}: {e}")))?;
        {
            let mut f = self.seen.borrow_mut();
            f.files_done += 1;
            f.done = f.total;
        }
        (self.on.borrow_mut())(&self.seen.borrow());
        let dst = self.dir.join(name);
        if !dst.exists() {
            std::fs::copy(&src, &dst)?;
        }
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
}
