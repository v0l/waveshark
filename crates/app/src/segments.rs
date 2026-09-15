//! A folder of dated, capped, append-only files, for the logs that keep what
//! the receiver heard.
//!
//! The packet log and the call log differ in what a record is and in nothing
//! else: both roll a file per day, start a new segment when one grows too
//! large, buffer their writes so a busy band is not a syscall per record,
//! delete the oldest segments to stay under a limit on the folder, and have
//! to survive being killed mid-write. That is this file, and a third log
//! wanting the same behaviour writes its records and nothing more.
//!
//! Errors are swallowed on purpose. A full disk or a read-only home must not
//! take the receiver down or fill the fault line: a log is a convenience, and
//! losing it is not worth losing what is on screen.

use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

/// What one log's files are: their extension, the header each carries, and
/// how large a segment grows before the next is started.
pub struct Format {
    /// Extension, which is also which files in the folder the log owns and
    /// may delete.
    pub ext: &'static str,
    pub magic: &'static [u8],
    pub version: u16,
    /// The log is trimmed by deleting whole files, so this is how coarsely it
    /// can be trimmed and how much of the recent past a trim takes with it.
    pub segment_bytes: u64,
    pub buf_bytes: usize,
    /// How long a record may sit in the buffer before it reaches the disk,
    /// which is what a receiver killed at 4am loses.
    pub flush_every: Duration,
}

pub struct Segments {
    dir: PathBuf,
    fmt: Format,
    /// The day currently open, as `YYYY-MM-DD`, and its writer.
    open: Option<(String, std::io::BufWriter<std::fs::File>)>,
    /// The segment being written, as `YYYY-MM-DD.NNN`, which is the one file
    /// in the folder a trim may not delete.
    segment: Option<String>,
    /// Bytes in the open segment.
    bytes: u64,
    /// Bytes in every other segment, so the folder's total is this plus the
    /// open one and no directory has to be walked per record.
    older: u64,
    full: bool,
    written: u64,
    cap: Option<u64>,
    dirty: bool,
    last_flush: Instant,
    /// When the folder was last added up, for the reading in the interface.
    measured: Instant,
}

impl Segments {
    /// Measured here rather than at the first record: the reading is about
    /// the folder, and a receiver that has heard nothing yet still has
    /// whatever last night wrote sitting on the disk.
    pub fn new(dir: PathBuf, fmt: Format) -> Self {
        let older = measure(&dir, fmt.ext, None);
        Self {
            dir,
            fmt,
            open: None,
            segment: None,
            bytes: 0,
            older,
            full: false,
            written: 0,
            cap: None,
            dirty: false,
            last_flush: Instant::now(),
            measured: Instant::now(),
        }
    }

    /// Change the folder's limit. `None` lifts it.
    pub fn set_cap(&mut self, cap: Option<u64>) {
        self.cap = cap;
        // Raising the cap on a log that stopped should start it again, or the
        // setting would only take effect on the next restart.
        if self.cap.is_none_or(|c| self.total() < c) {
            self.full = false;
        }
    }

    /// What the folder holds: the segment being written and every one kept.
    pub fn total(&self) -> u64 {
        self.older + self.bytes
    }

    /// Records appended since the receiver started.
    pub fn written(&self) -> u64 {
        self.written
    }

    /// Whether it has stopped accepting anything. A log that quietly stopped
    /// writing is worse than one that never started, so the interface has to
    /// be able to say which it is.
    pub fn full(&self) -> bool {
        self.full
    }

    /// Add up the folder again, at most every couple of seconds.
    ///
    /// Called from whatever publishes the status rather than from a write:
    /// the open segment is counted as it grows, so this is only for what
    /// changed underneath, and a directory listing per record is a syscall
    /// per record.
    pub fn refresh_folder(&mut self) {
        const EVERY: Duration = Duration::from_secs(2);
        if self.measured.elapsed() < EVERY {
            return;
        }
        self.measured = Instant::now();
        self.older = measure(&self.dir, self.fmt.ext, self.segment.as_deref());
    }

    /// Add a record, in the file for the day it happened on.
    pub fn append(&mut self, at_us: u64, rec: &[u8]) {
        let Some(w) = self.writer(at_us) else { return };
        if w.write_all(rec).is_err() {
            self.full = true;
            return;
        }
        self.dirty = true;
        self.bytes += rec.len() as u64;
        self.written += 1;
        if self.cap.is_some_and(|c| self.total() >= c) {
            let open = self.segment.clone().unwrap_or_default();
            self.full = !self.make_room(&open);
        }
        self.flush_due();
    }

    /// Push the buffer to the disk if it has been waiting long enough.
    ///
    /// A per-record flush turns every record into a write syscall, and on a
    /// band that produces thousands a second that is the receiver's time
    /// spent on a convenience. Batching by deadline keeps the syscall rate
    /// bounded by the clock rather than by the traffic.
    pub fn flush_due(&mut self) {
        if self.dirty && self.last_flush.elapsed() >= self.fmt.flush_every {
            self.flush();
        }
    }

    /// Put everything buffered on the disk now.
    pub fn flush(&mut self) {
        self.last_flush = Instant::now();
        if !self.dirty {
            return;
        }
        self.dirty = false;
        if let Some((_, w)) = self.open.as_mut()
            && w.flush().is_err()
        {
            self.full = true;
        }
    }

    /// Pretend the folder was last measured long ago, so the next refresh
    /// actually looks. For tests of what the interface reads.
    #[cfg(test)]
    pub fn forget_measurement(&mut self) {
        self.measured -= Duration::from_secs(60);
    }

    /// Delete whole segments, oldest first, until the folder is back under
    /// its limit. The segment being written is never a candidate: it holds
    /// the records somebody is watching arrive.
    ///
    /// Returns whether there is now room. When there is not, the only file
    /// left is the open segment and it is over the limit on its own, so
    /// appending stops rather than the log eating itself; a cap under one
    /// segment is the only way to reach that.
    fn make_room(&mut self, open: &str) -> bool {
        let Some(cap) = self.cap else { return true };
        if self.total() < cap {
            return true;
        }
        let Ok(entries) = std::fs::read_dir(&self.dir) else {
            return false;
        };
        // The name is the date and a sequence, so alphabetical order is
        // chronological.
        let mut days: Vec<PathBuf> = entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.extension().is_some_and(|x| x == self.fmt.ext)
                    && p.file_stem().is_some_and(|s| s != open)
            })
            .collect();
        days.sort();
        for path in days {
            let size = std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0);
            if std::fs::remove_file(&path).is_ok() {
                self.older = self.older.saturating_sub(size);
            }
            if self.total() < cap {
                return true;
            }
        }
        false
    }

    /// Open or roll the day's file, returning nothing once logging has
    /// stopped.
    fn writer(&mut self, at_us: u64) -> Option<&mut std::io::BufWriter<std::fs::File>> {
        if self.full {
            return None;
        }
        let day = day_of(at_us);
        let roll = self.bytes >= self.fmt.segment_bytes;
        if roll || self.open.as_ref().is_none_or(|(d, _)| *d != day) {
            // The old segment's buffer goes out before its writer does, or a
            // roll silently truncates the file it just closed.
            self.flush();
            if std::fs::create_dir_all(&self.dir).is_err() {
                self.full = true;
                return None;
            }
            let (name, path) = self.next_segment(&day);
            let fresh = !path.exists();
            let Ok(f) = std::fs::OpenOptions::new().create(true).append(true).open(&path) else {
                self.full = true;
                return None;
            };
            let mut w = std::io::BufWriter::with_capacity(self.fmt.buf_bytes, f);
            self.bytes = w.get_ref().metadata().map(|m| m.len()).unwrap_or(0);
            self.older = measure(&self.dir, self.fmt.ext, Some(&name));
            self.measured = Instant::now();
            // A receiver started against a folder already over its limit
            // makes room before it writes, rather than on the record that
            // happens to cross the line.
            if !self.make_room(&name) {
                self.full = true;
                return None;
            }
            if fresh || self.bytes == 0 {
                let head = w.write_all(self.fmt.magic).is_ok()
                    && w.write_all(&self.fmt.version.to_le_bytes()).is_ok();
                if !head {
                    self.full = true;
                    return None;
                }
                self.bytes += self.fmt.magic.len() as u64 + 2;
            }
            self.open = Some((day, w));
            self.segment = Some(name);
        }
        self.open.as_mut().map(|(_, w)| w)
    }

    /// The next segment for a day: the highest sequence already there, or a
    /// new one when that segment is full.
    ///
    /// A restart continues the last segment rather than starting another, so
    /// a receiver stopped and started ten times leaves ten minutes of log in
    /// one file rather than ten files of a minute.
    fn next_segment(&self, day: &str) -> (String, PathBuf) {
        let mut last: Option<(u32, PathBuf)> = None;
        if let Ok(entries) = std::fs::read_dir(&self.dir) {
            for e in entries.flatten() {
                let p = e.path();
                if p.extension().is_none_or(|x| x != self.fmt.ext) {
                    continue;
                }
                let Some(stem) = p.file_stem().and_then(|s| s.to_str()) else {
                    continue;
                };
                let Some((d, seq)) = stem.rsplit_once('.') else {
                    continue;
                };
                if d != day {
                    continue;
                }
                let Ok(seq) = seq.parse::<u32>() else {
                    continue;
                };
                if last.as_ref().is_none_or(|(n, _)| seq > *n) {
                    last = Some((seq, p));
                }
            }
        }
        let next = match last {
            Some((seq, ref p)) => {
                let size = std::fs::metadata(p).map(|m| m.len()).unwrap_or(0);
                if size >= self.fmt.segment_bytes { seq + 1 } else { seq }
            }
            None => 0,
        };
        let name = format!("{day}.{next:03}");
        let path = self.dir.join(format!("{name}.{}", self.fmt.ext));
        (name, path)
    }
}

/// A closed receiver keeps its last records. `BufWriter` drops silently, and
/// silently is exactly how the tail of an overnight session goes missing.
impl Drop for Segments {
    fn drop(&mut self) {
        self.flush();
    }
}

/// What a log folder holds, for a receiver with no log open to ask.
pub fn folder_bytes(dir: &Path, ext: &str) -> u64 {
    measure(dir, ext, None)
}

/// Add up the segments on the disk, other than the one named.
fn measure(dir: &Path, ext: &str, except: Option<&str>) -> u64 {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return 0;
    };
    entries
        .flatten()
        .filter(|e| {
            let p = e.path();
            p.extension().is_some_and(|x| x == ext)
                && except.is_none_or(|e| p.file_stem().is_some_and(|s| s != e))
        })
        .filter_map(|e| e.metadata().ok())
        .map(|m| m.len())
        .sum()
}

/// UTC date as `YYYY-MM-DD`, by civil-from-days rather than a calendar crate.
pub fn day_of(at_us: u64) -> String {
    let days = (at_us / 1_000_000) as i64 / 86_400;
    // Howard Hinnant's civil_from_days, which is exact and fits in a function.
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02}")
}

/// `YYYY-MM-DDTHH:MM:SSZ`, for a log a person reads rather than a program.
pub fn iso_of(at_us: u64) -> String {
    let secs = at_us / 1_000_000;
    let rest = secs % 86_400;
    format!("{}T{:02}:{:02}:{:02}Z", day_of(at_us), rest / 3600, rest % 3600 / 60, rest % 60)
}

/// The header a reader checks before it parses anything, and where the first
/// record starts. `None` when the file is not one of ours.
pub fn after_magic(buf: &[u8], magic: &[u8]) -> Option<usize> {
    if buf.len() < magic.len() + 2 || &buf[..magic.len()] != magic {
        return None;
    }
    Some(magic.len() + 2)
}
