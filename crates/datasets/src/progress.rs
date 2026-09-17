//! How far a download has got, readable while it is running.
//!
//! A dataset is fetched on a thread of its own and can take minutes: the
//! cell export is 85 MB and a script repository two gigabytes. What the
//! fetch knows is bytes written and, where the far end said so, bytes
//! expected; what the settings pane needs is those two numbers without
//! holding a handle to the thread.
//!
//! So the numbers are kept beside the dataset's name rather than passed
//! back: the name is what both ends already agree on (`Source::name`, or a
//! repository's cache directory), and one of these outlives any particular
//! download of it.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Mutex, OnceLock};

/// Bytes written and, where it was declared, bytes expected.
#[derive(Debug, Default)]
pub struct Progress {
    done: AtomicU64,
    /// Zero when the far end sent no length, which is the honest answer for
    /// a chunked response: a bar with no end is better than a made-up one.
    total: AtomicU64,
    running: AtomicBool,
}

impl Progress {
    /// A download is starting: whatever the last one got to is not this one.
    pub fn start(&self) {
        self.done.store(0, Ordering::Release);
        self.total.store(0, Ordering::Release);
        self.running.store(true, Ordering::Release);
    }

    /// What the far end said it was sending.
    pub fn expect(&self, bytes: u64) {
        self.total.store(bytes, Ordering::Release);
    }

    /// Another `n` bytes landed.
    pub fn wrote(&self, n: u64) {
        self.done.fetch_add(n, Ordering::AcqRel);
    }

    /// Stopped, whether it finished or failed.
    pub fn stop(&self) {
        self.running.store(false, Ordering::Release);
    }

    /// Bytes so far, bytes expected, and whether it is still going.
    pub fn read(&self) -> Reading {
        Reading {
            done: self.done.load(Ordering::Acquire),
            total: match self.total.load(Ordering::Acquire) {
                0 => None,
                n => Some(n),
            },
            running: self.running.load(Ordering::Acquire),
        }
    }
}

/// What a download has got to, for the row that draws it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Reading {
    pub done: u64,
    /// `None` where the far end declared no length.
    pub total: Option<u64>,
    pub running: bool,
}

impl Reading {
    /// How far through, 0 to 1, or `None` without a length to be through.
    pub fn fraction(&self) -> Option<f32> {
        let total = self.total?;
        (total > 0).then(|| (self.done as f32 / total as f32).clamp(0.0, 1.0))
    }
}

/// The progress of whatever is being fetched under this name.
///
/// Interned and leaked: there are a few dozen dataset names in a run, each
/// one a `&'static str` already, and a reference that stays valid is what
/// lets a fetch deep in the stack report without carrying a handle.
pub fn of(name: &str) -> &'static Progress {
    static ALL: OnceLock<Mutex<HashMap<String, &'static Progress>>> = OnceLock::new();
    let all = ALL.get_or_init(|| Mutex::new(HashMap::new()));
    let mut all = all.lock().unwrap_or_else(|e| e.into_inner());
    if let Some(p) = all.get(name) {
        return p;
    }
    let p: &'static Progress = Box::leak(Box::new(Progress::default()));
    all.insert(name.to_string(), p);
    p
}

/// A writer that counts what goes through it.
pub struct Counted<'a, W: std::io::Write> {
    pub inner: W,
    pub progress: &'a Progress,
}

impl<W: std::io::Write> std::io::Write for Counted<'_, W> {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let n = self.inner.write(buf)?;
        self.progress.wrote(n as u64);
        Ok(n)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.inner.flush()
    }
}

/// A reader that counts what comes out of it, for a fetch that unpacks as
/// it reads rather than writing a file.
pub struct Tapped<'a, R: std::io::Read> {
    pub inner: R,
    pub progress: &'a Progress,
}

impl<R: std::io::Read> std::io::Read for Tapped<'_, R> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let n = self.inner.read(buf)?;
        self.progress.wrote(n as u64);
        Ok(n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read as _, Write as _};

    #[test]
    fn a_name_keeps_its_own_count() {
        let a = of("one");
        let b = of("two");
        a.start();
        a.expect(200);
        a.wrote(50);
        assert_eq!(a.read(), Reading { done: 50, total: Some(200), running: true });
        assert_eq!(a.read().fraction(), Some(0.25));
        // Asking again is asking the same one.
        assert_eq!(of("one").read().done, 50);
        assert_eq!(b.read(), Reading::default());
        a.stop();
        assert!(!of("one").read().running);
    }

    #[test]
    fn a_download_with_no_length_has_no_fraction() {
        let p = of("lengthless");
        p.start();
        p.wrote(10);
        assert_eq!(p.read().total, None);
        assert_eq!(p.read().fraction(), None);
    }

    #[test]
    fn a_second_download_counts_from_nothing() {
        let p = of("twice");
        p.start();
        p.wrote(90);
        p.stop();
        p.start();
        assert_eq!(p.read().done, 0);
    }

    #[test]
    fn the_wrappers_count_what_passes_through_them() {
        let p = of("wrapped");
        p.start();
        let mut out = Counted { inner: Vec::new(), progress: p };
        out.write_all(b"twelve bytes").unwrap();
        assert_eq!(p.read().done, 12);

        let q = of("unwrapped");
        q.start();
        let mut buf = Vec::new();
        Tapped { inner: &b"four"[..], progress: q }.read_to_end(&mut buf).unwrap();
        assert_eq!(q.read().done, 4);
    }
}
