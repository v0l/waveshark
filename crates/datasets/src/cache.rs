//! A file cache: whole files from somewhere else, kept on disk, refreshed
//! when the far end says they changed.
//!
//! Reference data is not signal: an airport, a repeater and a DMR ID are the
//! same on every receiver and change a few times a month. Embedding a slice
//! of one in the binary makes the release the only way to update it and the
//! author the only person who can choose the slice, so instead every dataset
//! is a [`Source`] the cache fetches once and then revalidates with the
//! headers the server already publishes.
//!
//! A source is not necessarily a URL. [`Fetch`] is the whole interface, and
//! [`Http`] and [`File`] implement it; anything else that can produce bytes
//! and say whether they changed can be a source without the datasets above
//! knowing where they come from.
//!
//! Freshness is the HTTP model and nothing more: an entity tag or a
//! modification date is stored beside the file, sent back as
//! `If-None-Match` / `If-Modified-Since`, and a 304 costs one round trip and
//! no bytes. [`Source::max_age`] is only a floor on how often that round trip
//! happens, so a run that restarts every minute does not ask an unchanging
//! 85 MB file about itself every minute.
//!
//! Everything here works in paths rather than buffers. The DMR user dump is
//! 85 MB of JSON, and holding the download and the parse of it at once is a
//! quarter of a gigabyte for a callsign lookup, so a fetch streams to the
//! file and a reader is what the parse gets.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// What the far end said about the version we now hold, in the form it wants
/// back to decide whether that version is still current.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Seen {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub etag: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub last_modified: Option<String>,
}

impl Seen {
    fn is_empty(&self) -> bool {
        self.etag.is_none() && self.last_modified.is_none()
    }
}

/// Where a cached file comes from. Implementors are shared across threads
/// because the refresh runs off the UI thread.
pub trait Fetch: Send + Sync {
    /// A human-readable name for logs and errors, usually the URL.
    fn origin(&self) -> String;

    /// Write the current bytes to `to`, or return `None` without writing
    /// anything if what `have` describes is still current.
    fn fetch(&self, have: &Seen, to: &mut dyn Write) -> Result<Option<Seen>, Error>;
}

/// A cached dataset file: what to call it on disk, where it comes from, and
/// how often it is worth asking whether it changed.
#[derive(Clone)]
pub struct Source {
    /// File name under the cache directory. Also the metadata key, so it must
    /// be unique and a valid file name.
    pub name: &'static str,
    pub from: Arc<dyn Fetch>,
    pub max_age: Duration,
}

impl Source {
    pub fn http(name: &'static str, url: &'static str, max_age: Duration) -> Self {
        Self { name, from: Arc::new(Http { url }), max_age }
    }
}

/// An HTTP source, revalidated with entity tags and modification dates.
pub struct Http {
    pub url: &'static str,
}

/// Identifies the client to servers with a usage policy, the same way the
/// tile fetcher does.
const AGENT: &str = concat!(
    "WaveShark/",
    env!("CARGO_PKG_VERSION"),
    " (https://github.com/v0l/waveshark)"
);

/// A dataset arrives in one response. The DMR user dump is 85 MB, and a limit
/// an order of magnitude above that guards against a redirect to something
/// else entirely rather than bounding anything real.
const MAX_BYTES: u64 = 1 << 30;

impl Fetch for Http {
    fn origin(&self) -> String {
        self.url.to_string()
    }

    fn fetch(&self, have: &Seen, to: &mut dyn Write) -> Result<Option<Seen>, Error> {
        let agent: ureq::Agent = ureq::Agent::config_builder()
            .user_agent(AGENT)
            // 304 is the answer we are hoping for, not a failure.
            .http_status_as_error(false)
            .timeout_global(Some(Duration::from_secs(600)))
            .build()
            .into();
        let mut req = agent.get(self.url);
        if let Some(e) = &have.etag {
            req = req.header("If-None-Match", e);
        }
        if let Some(m) = &have.last_modified {
            req = req.header("If-Modified-Since", m);
        }
        let fail = |e: String| Error::Fetch(self.url.to_string(), e);
        let mut resp = req.call().map_err(|e| fail(e.to_string()))?;
        let code = resp.status().as_u16();
        if code == 304 {
            return Ok(None);
        }
        if code != 200 {
            return Err(Error::Status(self.url.into(), code));
        }
        let header = |k: &str| {
            resp.headers().get(k).and_then(|v| v.to_str().ok()).map(str::to_string)
        };
        let seen = Seen { etag: header("etag"), last_modified: header("last-modified") };
        let mut body = resp.body_mut().with_config().limit(MAX_BYTES).reader();
        std::io::copy(&mut body, to).map_err(|e| fail(e.to_string()))?;
        Ok(Some(seen))
    }
}

/// A file somewhere else on this machine, revalidated by its modification
/// time. Useful for a dataset kept up to date by something other than this
/// program, and for tests, which must not touch the network.
pub struct File {
    pub path: PathBuf,
}

impl Fetch for File {
    fn origin(&self) -> String {
        self.path.display().to_string()
    }

    fn fetch(&self, have: &Seen, to: &mut dyn Write) -> Result<Option<Seen>, Error> {
        let io = |e: std::io::Error| Error::Io(self.origin(), e);
        let meta = std::fs::metadata(&self.path).map_err(io)?;
        let stamp = meta
            .modified()
            .ok()
            .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
            .map(|d| format!("{}.{:09}", d.as_secs(), d.subsec_nanos()));
        if stamp.is_some() && stamp == have.last_modified {
            return Ok(None);
        }
        let mut f = std::fs::File::open(&self.path).map_err(io)?;
        std::io::copy(&mut f, to).map_err(io)?;
        Ok(Some(Seen { etag: None, last_modified: stamp }))
    }
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}: {1}")]
    Io(String, #[source] std::io::Error),
    #[error("{0}: {1}")]
    Fetch(String, String),
    #[error("{0}: HTTP {1}")]
    Status(String, u16),
    #[error("no cache directory")]
    NoDir,
    #[error("{0}: {1}")]
    Parse(String, String),
}

/// What is recorded beside a cached file. The length is checked against the
/// file on load: a download killed halfway leaves a short file, and a short
/// file with valid metadata beside it would be revalidated as current
/// forever.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
struct Meta {
    #[serde(flatten)]
    seen: Seen,
    len: u64,
    /// Unix seconds at the last successful revalidation, whether or not it
    /// produced new bytes. This is what [`Source::max_age`] is measured
    /// against.
    checked: u64,
}

/// Whether a refresh is allowed to skip the round trip.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum When {
    /// Only if the last check is older than [`Source::max_age`]. What a
    /// startup does.
    IfDue,
    /// Ask now regardless. What a person pressing refresh means.
    Now,
}

/// What the cache holds for one source, for a view that reports it.
#[derive(Clone, Debug, Default)]
pub struct Status {
    pub origin: String,
    /// Absent when nothing complete is cached.
    pub bytes: Option<u64>,
    /// Unix seconds at the last successful check, new bytes or not.
    pub checked: Option<u64>,
    /// What the far end called this version, for a view that wants to show
    /// which one is held.
    pub last_modified: Option<String>,
}

/// The cached files themselves, under one directory.
#[derive(Clone)]
pub struct Cache {
    dir: PathBuf,
}

impl Cache {
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self { dir: dir.into() }
    }

    /// `$XDG_CACHE_HOME/waveshark/data`, or `~/.cache` when unset. Beside the
    /// map tiles, because both are copies of something published elsewhere
    /// and both can be deleted without losing anything of the operator's.
    pub fn default_dir() -> Result<PathBuf, Error> {
        let base = std::env::var_os("XDG_CACHE_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache")))
            .ok_or(Error::NoDir)?;
        Ok(base.join("waveshark").join("data"))
    }

    pub fn at_default_dir() -> Result<Self, Error> {
        let dir = Self::default_dir()?;
        std::fs::create_dir_all(&dir).map_err(|e| Error::Io(dir.display().to_string(), e))?;
        Ok(Self::new(dir))
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    fn file(&self, src: &Source) -> PathBuf {
        self.dir.join(src.name)
    }

    fn meta_file(&self, src: &Source) -> PathBuf {
        self.dir.join(format!("{}.meta.json", src.name))
    }

    fn meta(&self, src: &Source) -> Option<Meta> {
        let raw = std::fs::read(self.meta_file(src)).ok()?;
        let meta: Meta = serde_json::from_slice(&raw).ok()?;
        (std::fs::metadata(self.file(src)).ok()?.len() == meta.len).then_some(meta)
    }

    /// The path of a complete cached copy, if there is one.
    pub fn cached(&self, src: &Source) -> Option<PathBuf> {
        self.meta(src)?;
        Some(self.file(src))
    }

    /// The cached file, downloading it if it is not there yet. This is the
    /// call on the path to first use: it blocks on the network, so it belongs
    /// off the UI thread.
    pub fn get(&self, src: &Source) -> Result<PathBuf, Error> {
        if let Some(p) = self.cached(src) {
            return Ok(p);
        }
        match self.fetch(src, &Seen::default())? {
            Some(p) => Ok(p),
            // Nothing is cached, so a source claiming nothing changed has
            // nothing to compare against and is answering the wrong question.
            None => Err(Error::Fetch(
                src.from.origin(),
                "not modified, but nothing is cached".into(),
            )),
        }
    }

    /// The whole file, for datasets small enough to hold at once.
    pub fn read(&self, src: &Source) -> Result<Vec<u8>, Error> {
        let p = self.get(src)?;
        std::fs::read(&p).map_err(|e| Error::Io(p.display().to_string(), e))
    }

    /// What is held for this source, and when it was last checked.
    pub fn status(&self, src: &Source) -> Status {
        let meta = self.meta(src);
        Status {
            origin: src.from.origin(),
            bytes: meta.as_ref().map(|m| m.len),
            checked: meta.as_ref().map(|m| m.checked),
            last_modified: meta.and_then(|m| m.seen.last_modified),
        }
    }

    /// Ask whether the file changed, and return its path if it did.
    ///
    /// `None` means there is nothing new to do: either the far end said so,
    /// or the check was not due and `when` allowed skipping it.
    pub fn refresh(&self, src: &Source, when: When) -> Result<Option<PathBuf>, Error> {
        let meta = self.meta(src);
        if let Some(m) = &meta {
            let due = now().saturating_sub(m.checked) >= src.max_age.as_secs();
            if when == When::IfDue && !due && !m.seen.is_empty() {
                return Ok(None);
            }
        }
        let seen = meta.map(|m| m.seen).unwrap_or_default();
        let got = self.fetch(src, &seen)?;
        if got.is_none() {
            self.touch(src);
        }
        Ok(got)
    }

    /// Fetch into a temporary beside the target and rename it into place, so
    /// a run killed mid-download leaves the previous copy rather than a
    /// truncated file that parses to nonsense.
    fn fetch(&self, src: &Source, have: &Seen) -> Result<Option<PathBuf>, Error> {
        std::fs::create_dir_all(&self.dir)
            .map_err(|e| Error::Io(self.dir.display().to_string(), e))?;
        let path = self.file(src);
        let tmp = path.with_extension("part");
        let io = |p: &Path| {
            let p = p.display().to_string();
            move |e: std::io::Error| Error::Io(p.clone(), e)
        };
        let f = std::fs::File::create(&tmp).map_err(io(&tmp))?;
        let mut out = std::io::BufWriter::new(f);
        let seen = match src.from.fetch(have, &mut out) {
            Ok(Some(seen)) => seen,
            other => {
                drop(out);
                let _ = std::fs::remove_file(&tmp);
                return other.map(|_| None);
            }
        };
        out.flush().map_err(io(&tmp))?;
        let len = out.get_ref().metadata().map_err(io(&tmp))?.len();
        drop(out);
        std::fs::rename(&tmp, &path).map_err(io(&path))?;
        let meta = Meta { seen, len, checked: now() };
        let raw = serde_json::to_vec(&meta).unwrap_or_default();
        let mpath = self.meta_file(src);
        std::fs::write(&mpath, raw).map_err(io(&mpath))?;
        tracing::info!(dataset = src.name, bytes = len, from = %src.from.origin(), "dataset downloaded");
        Ok(Some(path))
    }

    /// Record that the file was revalidated and is still current, so the next
    /// run does not ask again inside `max_age`.
    fn touch(&self, src: &Source) {
        let Some(mut meta) = self.meta(src) else { return };
        meta.checked = now();
        if let Ok(raw) = serde_json::to_vec(&meta) {
            let _ = std::fs::write(self.meta_file(src), raw);
        }
    }
}

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    /// A source that answers from memory and counts how often it was asked,
    /// which is what the freshness rules are actually about.
    struct Counted {
        body: parking_lot::Mutex<Vec<u8>>,
        tag: parking_lot::Mutex<String>,
        fetches: AtomicUsize,
    }

    impl Fetch for Counted {
        fn origin(&self) -> String {
            "counted".into()
        }
        fn fetch(&self, have: &Seen, to: &mut dyn Write) -> Result<Option<Seen>, Error> {
            self.fetches.fetch_add(1, Ordering::Relaxed);
            let tag = self.tag.lock().clone();
            if have.etag.as_deref() == Some(tag.as_str()) {
                return Ok(None);
            }
            to.write_all(&self.body.lock()).unwrap();
            Ok(Some(Seen { etag: Some(tag), last_modified: None }))
        }
    }

    fn tmpdir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("waveshark-cache-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    fn counted(dir: &Path, max_age: Duration) -> (Cache, Source, Arc<Counted>) {
        let from = Arc::new(Counted {
            body: parking_lot::Mutex::new(b"one".to_vec()),
            tag: parking_lot::Mutex::new("v1".into()),
            fetches: AtomicUsize::new(0),
        });
        let src = Source { name: "thing.txt", from: from.clone(), max_age };
        (Cache::new(dir), src, from)
    }

    #[test]
    fn the_second_run_reads_the_disk_instead_of_the_network() {
        let dir = tmpdir("second-run");
        let (cache, src, from) = counted(&dir, Duration::from_secs(3600));
        assert_eq!(cache.read(&src).unwrap(), b"one");
        assert_eq!(cache.read(&src).unwrap(), b"one");
        assert_eq!(from.fetches.load(Ordering::Relaxed), 1);
        // And inside max_age a refresh does not even revalidate.
        assert!(cache.refresh(&src, When::IfDue).unwrap().is_none());
        assert_eq!(from.fetches.load(Ordering::Relaxed), 1);
        // Unless somebody asked for it, which is what the button does.
        assert!(cache.refresh(&src, When::Now).unwrap().is_none());
        assert_eq!(from.fetches.load(Ordering::Relaxed), 2, "a forced check must go out");
    }

    #[test]
    fn a_status_says_what_is_held_and_when_it_was_checked() {
        let dir = tmpdir("status");
        let (cache, src, _) = counted(&dir, Duration::from_secs(3600));
        assert_eq!(cache.status(&src).bytes, None, "nothing cached yet");
        cache.read(&src).unwrap();
        let s = cache.status(&src);
        assert_eq!(s.bytes, Some(3));
        assert!(s.checked.is_some_and(|c| c > 0));
    }

    #[test]
    fn a_changed_source_arrives_on_refresh_and_an_unchanged_one_does_not() {
        let dir = tmpdir("changed");
        let (cache, src, from) = counted(&dir, Duration::ZERO);
        assert_eq!(cache.read(&src).unwrap(), b"one");
        // Same tag: revalidated, no new bytes.
        assert!(cache.refresh(&src, When::IfDue).unwrap().is_none());
        *from.body.lock() = b"two".to_vec();
        *from.tag.lock() = "v2".into();
        assert!(cache.refresh(&src, When::IfDue).unwrap().is_some());
        assert_eq!(cache.read(&src).unwrap(), b"two");
    }

    #[test]
    fn a_truncated_file_is_not_served_from_the_cache() {
        let dir = tmpdir("truncated");
        let (cache, src, _) = counted(&dir, Duration::from_secs(3600));
        cache.read(&src).unwrap();
        std::fs::write(dir.join("thing.txt"), b"o").unwrap();
        assert!(cache.cached(&src).is_none(), "short file served as complete");
    }

    #[test]
    fn a_failed_fetch_leaves_the_previous_copy_alone() {
        struct Broken;
        impl Fetch for Broken {
            fn origin(&self) -> String {
                "broken".into()
            }
            fn fetch(&self, _: &Seen, to: &mut dyn Write) -> Result<Option<Seen>, Error> {
                to.write_all(b"half a fi").unwrap();
                Err(Error::Status("broken".into(), 500))
            }
        }
        let dir = tmpdir("failed");
        let (cache, src, _) = counted(&dir, Duration::ZERO);
        cache.read(&src).unwrap();
        let broken = Source { name: src.name, from: Arc::new(Broken), max_age: Duration::ZERO };
        assert!(cache.refresh(&broken, When::Now).is_err());
        assert_eq!(cache.read(&src).unwrap(), b"one");
    }

    #[test]
    fn a_local_file_source_revalidates_on_its_mtime() {
        let dir = tmpdir("local");
        let src_path = dir.join("input.txt");
        std::fs::write(&src_path, b"hello").unwrap();
        let cache = Cache::new(dir.join("cache"));
        let src = Source {
            name: "input.txt",
            from: Arc::new(File { path: src_path.clone() }),
            max_age: Duration::ZERO,
        };
        assert_eq!(cache.read(&src).unwrap(), b"hello");
        assert!(cache.refresh(&src, When::Now).unwrap().is_none());
        // Rewriting moves the mtime, which is this source's validator.
        std::thread::sleep(Duration::from_millis(20));
        std::fs::write(&src_path, b"goodbye").unwrap();
        assert!(cache.refresh(&src, When::Now).unwrap().is_some());
        assert_eq!(cache.read(&src).unwrap(), b"goodbye");
    }

    /// radioid.net and pistar both serve large files under a usage policy,
    /// and a client that identifies itself as nothing is the one they block
    /// first.
    #[test]
    fn the_agent_names_the_product_and_where_it_came_from() {
        assert!(AGENT.starts_with("WaveShark/"), "{AGENT}");
        assert!(AGENT.contains(env!("CARGO_PKG_VERSION")), "{AGENT}");
        assert!(AGENT.contains("github.com/v0l/waveshark"), "{AGENT}");
    }
}
