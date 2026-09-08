//! Every burst the receiver hears, written to disk as it arrives.
//!
//! On by default and with no switch in the interface, because the value of a
//! log like this is entirely in having it already: the interesting
//! transmission is always the one that happened before anyone thought to
//! press record. A band left running overnight is a test corpus, and a corpus
//! of real bursts is the only honest way to tell whether a change to a
//! decoder helped.
//!
//! # What is stored
//!
//! What the demodulator produced, and nothing else: the mark and gap timings
//! of each burst, or the frame bytes where the demodulator makes bytes rather
//! than timings. The parsed frame is deliberately absent. A parse is a
//! conclusion, and a conclusion stored without the evidence cannot be checked
//! later, corrected by a better decoder, or used to prove that a decoder was
//! wrong. Timings can be decoded again next year; a field map cannot be
//! un-decoded.
//!
//! Undecoded bursts are written too, and they are the ones that matter most.
//! A burst that no protocol claimed leaves no other trace at all, and it is
//! the raw material for adding the protocol that would have claimed it.
//!
//! # The format
//!
//! A little-endian binary stream, one file per day, appended:
//!
//! ```text
//! file   := "WSPKT\0" u16 version
//! record := u32 body_len, u8 kind, u8 flags, u16 pulses_or_bytes,
//!           u64 at_us, u64 center_hz, u32 bandwidth_hz,
//!           f32 rssi_dbfs, f32 snr_db, body, [iq]
//! body   := kind 1: [u32 mark_us, u32 gap_us] * n
//!           kind 2: [u8] * n
//!           kind 3: measure, [u32 mark_us, u32 gap_us] * n
//! iq     := f64 rate, u64 center_hz, u32 n, [i16 i, i16 q] * n
//!        |  f64 rate, u64 center_hz, u32 (n | 0x8000_0000), u32 len,
//!           [u8] * len          -- the same samples, zstd compressed
//! ```
//!
//! The high bit of the sample count says the block is compressed, which is a
//! compatible change rather than a version bump: a reader that does not know
//! it asks for a count of two billion samples, does not find them, and
//! reports the record without its burst rather than misreading one.
//!
//! The samples come after the body, from version 2. The body's length is
//! fixed by the count in the header, so a reader of version 1 files takes
//! the body and stops, and a reader of version 2 files takes what is left as
//! the burst. Sixteen bits a component because that is more than any
//! converter here has, and floats would double a file that is already mostly
//! samples.
//!
//! Samples are kept because timings and bytes are what one demodulator made
//! of a burst, and a different demodulator, or the same one fixed, wants the
//! burst. A row without them can be read again but not re-read.
//!
//! Binary rather than the line-delimited JSON this replaces, because the
//! content changed: a burst is a few hundred timings, and a hundred bytes of
//! JSON per pulse turns an overnight capture into gigabytes of quoting. The
//! length prefix means an unknown `kind` can be skipped rather than
//! misparsed, and a receiver killed mid-write costs the last record rather
//! than the file, since a short tail cannot be mistaken for a complete
//! record.
//!
//! Written by hand rather than through a serialisation crate: the record is
//! nine scalars and an array, and it is not worth a dependency in the crate
//! that has none.

use std::io::{Read, Write};
use std::path::PathBuf;

use common::{Packet, PacketBody, Pulse};

/// How much of the disk the whole log folder may take.
///
/// A limit per day answered the wrong question. Nobody wants to know what one
/// file may reach; they want to know what the receiver may take, and a cap of
/// half a gigabyte a day is a cap of fifteen gigabytes a month with nothing
/// standing in its way. This is the number a disk has, so this is the number
/// the setting asks for, and the oldest days go to keep the folder under it.
pub const DEFAULT_MAX_BYTES: u64 = 2 << 30;

/// What a segment file is called after its date and sequence, and so which
/// files in the folder the log owns and may delete.
const EXT: &str = "wspkt";

/// How large one segment grows before the next is started.
///
/// The log is trimmed by deleting whole files, so the segment size is how
/// coarsely it can be trimmed and how much of the recent past a trim can
/// take with it. A day was the unit until a busy 2.4 GHz session put 122 GB
/// in one file: the cap could then only stop the log, because the one file
/// over it was the one being written. At a quarter of a gigabyte a trim
/// costs the oldest few minutes of a busy band, or the oldest week of a
/// quiet one.
const SEGMENT_BYTES: u64 = 256 << 20;

const MAGIC: &[u8; 6] = b"WSPKT\0";
const VERSION: u16 = 2;

/// Write buffer per open day file. A pulse record is a few hundred bytes and
/// 1090 MHz can produce thousands of frames a second, so this is sized to
/// hold a busy second rather than a single burst.
const BUF_BYTES: usize = 256 << 10;

/// How long a record may sit in the buffer before it reaches the disk.
///
/// The buffer is what makes a high packet rate cheap, and the deadline is
/// what stops it costing an evening of captures when the receiver is killed:
/// at most this much is ever in flight, no matter how quiet or how busy the
/// band is.
const FLUSH_EVERY: std::time::Duration = std::time::Duration::from_millis(250);

/// Timings from a front end that detects bursts.
pub const KIND_PULSES: u8 = 1;
/// Bytes from a demodulator that produces frames directly, such as Mode S.
pub const KIND_BYTES: u8 = 2;
/// A burst with what it was measured to be in front of its timings, of
/// which there may be none: a chirp or a carrier no front end reads is a
/// measurement and nothing else.
pub const KIND_MEASURED: u8 = 3;

/// How the burst was keyed, in the record's second byte.
///
/// That byte was written as zero and never read, which is what makes this a
/// compatible change rather than a version bump: an older file says nothing
/// about the keying, and nothing is exactly what it knew.
fn keying_code(m: Option<&'static str>) -> u8 {
    match m {
        Some("OOK") => 1,
        Some("ASK") => 2,
        Some("FSK") => 3,
        Some("4-FSK") => 4,
        Some("MSK") => 5,
        _ => 0,
    }
}

fn keying_from_code(c: u8) -> Option<&'static str> {
    match c {
        1 => Some("OOK"),
        2 => Some("ASK"),
        3 => Some("FSK"),
        4 => Some("4-FSK"),
        5 => Some("MSK"),
        _ => None,
    }
}

/// Bytes before the body of a record: kind, keying, count, time, frequency,
/// bandwidth, level and noise.
const HEAD_LEN: usize = 1 + 1 + 2 + 8 + 8 + 4 + 4 + 4;

/// Rate, centre and count in front of the samples.
const IQ_HEAD_LEN: usize = 8 + 8 + 4;

/// Samples a record may carry, which is the end of the burst when there are
/// more.
///
/// Four million was "well past anything a burst can be", and on a 2.4 GHz
/// source read at a couple of megasamples that is exactly what a burst was:
/// sixteen megabytes a record, and a day's log of 122 GB. A quarter of a
/// million samples is a megabyte before compression and a tenth of a second
/// at 2.4 MS/s, which holds any packet this receiver decodes; a LoRa symbol
/// at the highest spreading factor is the only thing that comes close.
const IQ_MAX_SAMPLES: usize = 1 << 18;

/// Compression level for the samples. Level 1: the samples are noise-like
/// and do not compress far whatever is spent on them, and this runs at
/// gigabytes a second, which a packet sink on the radio's thread needs.
const IQ_ZSTD_LEVEL: i32 = 1;

/// Marks a compressed sample block in the count field.
const IQ_COMPRESSED: u32 = 0x8000_0000;

pub struct PacketLog {
    dir: PathBuf,
    /// The day currently open, as `YYYY-MM-DD`, and its writer.
    open: Option<(String, std::io::BufWriter<std::fs::File>)>,
    /// The segment being written, as `YYYY-MM-DD.NNN`, which is the one file
    /// in the folder a trim may not delete.
    segment: Option<String>,
    /// Bytes in the day's file.
    bytes: u64,
    /// Bytes in every other day's file, so the folder's total is this plus
    /// the open one and no directory has to be walked per record.
    older: u64,
    full: bool,
    /// Packets appended since the receiver started.
    written: u64,
    /// Size the whole folder may reach, or `None` for no limit.
    cap: Option<u64>,
    /// Records written into the buffer since the last flush.
    dirty: bool,
    last_flush: std::time::Instant,
    /// When the folder was last added up, for the reading in the interface.
    measured: std::time::Instant,
}

impl PacketLog {
    /// `$XDG_DATA_HOME/waveshark/packets`, or `~/.local/share` when unset.
    pub fn default_dir() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_DATA_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))?;
        Some(base.join("waveshark").join("packets"))
    }

    /// `$XDG_DATA_HOME/waveshark/survey.sqlite`, beside the packet log.
    ///
    /// One file rather than one a day, which is what the log does: a survey
    /// is a table of things rather than a stream of events, and a device
    /// heard yesterday and again today is one row in one file.
    pub fn default_survey_path() -> Option<PathBuf> {
        Self::default_dir().and_then(|d| d.parent().map(|p| p.join("survey.sqlite")))
    }

    pub fn new(dir: PathBuf) -> Self {
        // Measured here rather than at the first record: the reading is about
        // the folder, and a receiver that has heard nothing yet, or is tuned
        // where no front end produces packets, still has whatever last night
        // wrote sitting on the disk. It read 0 B until something arrived.
        let older = measure(&dir, None);
        Self {
            dir,
            open: None,
            segment: None,
            bytes: 0,
            older,
            full: false,
            written: 0,
            cap: Some(DEFAULT_MAX_BYTES),
            dirty: false,
            last_flush: std::time::Instant::now(),
            measured: std::time::Instant::now(),
        }
    }

    /// Change the folder's limit. `None` lifts it, which is what a receiver
    /// left running on 1090 MHz for a week wants.
    pub fn with_cap(mut self, cap: Option<u64>) -> Self {
        self.cap = cap;
        // Raising the cap on a log that stopped should start it again, or the
        // setting would only take effect on the next restart.
        if self.cap.is_none_or(|c| self.total() < c) {
            self.full = false;
        }
        self
    }

    /// What the folder holds: the day being written and every day kept.
    pub fn total(&self) -> u64 {
        self.older + self.bytes
    }

    /// Add up the folder again, at most this often.
    ///
    /// Called from whatever publishes the status rather than from a write:
    /// the segment being written is counted as it grows, so this is only for
    /// what changed underneath, and a directory listing per record is a
    /// syscall per burst.
    pub fn refresh_folder(&mut self) {
        const EVERY: std::time::Duration = std::time::Duration::from_secs(2);
        if self.measured.elapsed() < EVERY {
            return;
        }
        self.measured = std::time::Instant::now();
        self.older = measure(&self.dir, self.segment.as_deref());
    }

    /// Delete whole segments, oldest first, until the folder is back under
    /// its limit. The segment being written is never a candidate: it is the
    /// one with the packets somebody is watching arrive.
    ///
    /// Returns whether there is now room. When there is not, the only file
    /// left is the open segment and it is over the limit on its own, so
    /// appending stops rather than the log eating itself; a cap under
    /// [`SEGMENT_BYTES`] is the only way to reach that.
    fn make_room(&mut self, open: &str) -> bool {
        let Some(cap) = self.cap else { return true };
        if self.total() < cap {
            return true;
        }
        let Ok(entries) = std::fs::read_dir(&self.dir) else { return false };
        // The name is the date and a sequence, so alphabetical order is
        // chronological.
        let mut days: Vec<std::path::PathBuf> = entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                p.extension().is_some_and(|x| x == EXT)
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

    /// Open or roll the day's file, returning false once logging has stopped.
    ///
    /// Errors are swallowed on purpose. A full disk or a read-only home must
    /// not take the receiver down or spam the fault line: the log is a
    /// convenience, and losing it is not worth losing the packets on screen.
    fn writer(&mut self, at_us: u64) -> Option<&mut std::io::BufWriter<std::fs::File>> {
        if self.full {
            return None;
        }
        let day = day_of(at_us);
        let roll = self.bytes >= SEGMENT_BYTES;
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
            let mut w = std::io::BufWriter::with_capacity(BUF_BYTES, f);
            self.bytes = w.get_ref().metadata().map(|m| m.len()).unwrap_or(0);
            self.older = measure(&self.dir, Some(&name));
            self.measured = std::time::Instant::now();
            // A receiver started against a folder already over its limit
            // makes room before it writes, rather than on the record that
            // happens to cross the line.
            if !self.make_room(&name) {
                self.full = true;
                return None;
            }
            if fresh || self.bytes == 0 {
                if w.write_all(MAGIC).is_err() || w.write_all(&VERSION.to_le_bytes()).is_err() {
                    self.full = true;
                    return None;
                }
                self.bytes += MAGIC.len() as u64 + 2;
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
                if p.extension().is_none_or(|x| x != EXT) {
                    continue;
                }
                let Some(stem) = p.file_stem().and_then(|s| s.to_str()) else { continue };
                let Some((d, seq)) = stem.rsplit_once('.') else { continue };
                if d != day {
                    continue;
                }
                let Ok(seq) = seq.parse::<u32>() else { continue };
                if last.as_ref().is_none_or(|(n, _)| seq > *n) {
                    last = Some((seq, p));
                }
            }
        }
        let next = match last {
            Some((seq, ref p)) => {
                let size = std::fs::metadata(p).map(|m| m.len()).unwrap_or(0);
                if size >= SEGMENT_BYTES {
                    seq + 1
                } else {
                    seq
                }
            }
            None => 0,
        };
        let name = format!("{day}.{next:03}");
        let path = self.dir.join(format!("{name}.{EXT}"));
        (name, path)
    }

    fn append(&mut self, at_us: u64, rec: &[u8]) {
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
    /// A per-record flush turns every burst into a write syscall, and on a
    /// band that produces thousands a second that is the receiver's time
    /// spent on a convenience. Batching by deadline keeps the syscall rate
    /// bounded by the clock rather than by the traffic.
    fn flush_due(&mut self) {
        if self.dirty && self.last_flush.elapsed() >= FLUSH_EVERY {
            self.flush();
        }
    }

    /// Put everything buffered on the disk now.
    pub fn flush(&mut self) {
        self.last_flush = std::time::Instant::now();
        if !self.dirty {
            return;
        }
        self.dirty = false;
        if let Some((_, w)) = self.open.as_mut() {
            if w.flush().is_err() {
                self.full = true;
            }
        }
    }
}

/// A closed receiver keeps its last bursts. `BufWriter` drops silently, and
/// silently is exactly how the tail of an overnight capture goes missing.
impl Drop for PacketLog {
    fn drop(&mut self) {
        self.flush();
    }
}

impl nodes::PacketSink for PacketLog {
    fn bytes(&self) -> u64 {
        self.total()
    }

    fn full(&self) -> bool {
        self.full
    }

    fn write(&mut self, p: &Packet) {
        // The log holds evidence, and speech is not evidence of itself: an
        // over from a voice channel is on the bus so it is heard, listed as a
        // call and read by the transcriber, and its body is an empty frame
        // with a megabyte of audio hanging off it. Writing that put a row
        // with nothing in it into the log for every transmission, and
        // replaying it produced a packet no decoder could ever say anything
        // about.
        if !carries_evidence(p) {
            return;
        }
        let rec = match &p.body {
            PacketBody::Pulses(package) => {
                let pulses = &package.pulses;
                // A burst longer than this is not a packet; the count is
                // capped rather than the record refused, so whatever it was
                // is still on record with its level and frequency.
                let n = pulses.len().min(u16::MAX as usize);
                let measure = p.measure.as_ref().map(put_measure).unwrap_or_default();
                let kind = if p.measure.is_some() { KIND_MEASURED } else { KIND_PULSES };
                let mut rec = Vec::with_capacity(4 + HEAD_LEN + measure.len() + n * 8);
                put_head(&mut rec, kind, n as u16, measure.len() + n * 8, p);
                rec.extend_from_slice(&measure);
                for pulse in &pulses[..n] {
                    rec.extend_from_slice(&pulse.mark.to_le_bytes());
                    rec.extend_from_slice(&pulse.gap.to_le_bytes());
                }
                rec
            }
            PacketBody::Frame(frame) => {
                let n = frame.bytes.len().min(u16::MAX as usize);
                let mut rec = Vec::with_capacity(4 + HEAD_LEN + n);
                put_head(&mut rec, KIND_BYTES, n as u16, n, p);
                rec.extend_from_slice(&frame.bytes[..n]);
                rec
            }
        };
        let rec = match p.samples().map(|q| q.as_ref()) {
            Some(q) if !q.samples.is_empty() => put_iq(rec, q),
            _ => rec,
        };
        self.append(p.at_us, &rec);
    }

    fn written(&self) -> u64 {
        self.written
    }

    /// Called between blocks whether or not anything arrived, so a band that
    /// went quiet still gets its last burst on the disk.
    fn flush(&mut self) {
        self.flush_due();
        self.refresh_folder();
    }
}

/// What a log folder holds, for a receiver with no log open to ask.
pub fn folder_bytes(dir: &std::path::Path) -> u64 {
    measure(dir, None)
}

/// Add up the segments on the disk, other than the one named.
fn measure(dir: &std::path::Path, except: Option<&str>) -> u64 {
    let Ok(entries) = std::fs::read_dir(dir) else { return 0 };
    entries
        .flatten()
        .filter(|e| {
            let p = e.path();
            p.extension().is_some_and(|x| x == EXT)
                && except.is_none_or(|e| p.file_stem().is_some_and(|s| s != e))
        })
        .filter_map(|e| e.metadata().ok())
        .map(|m| m.len())
        .sum()
}

/// Whether there is anything in this packet a decoder could read.
///
/// Timings, bytes, samples or a measurement. A packet with none of them is a
/// record of something having happened, which the call list and the survey
/// already keep, in a file whose whole purpose is to be decoded again later.
fn carries_evidence(p: &Packet) -> bool {
    let body = match &p.body {
        PacketBody::Pulses(package) => !package.pulses.is_empty(),
        PacketBody::Frame(frame) => !frame.bytes.is_empty(),
    };
    // A measurement counts. A chirp has no timings and no bytes, and what the
    // classifier made of it is the whole record of a LoRa transmission
    // nothing decoded.
    body || p.measure.is_some() || p.samples().is_some_and(|q| !q.samples.is_empty())
}

/// A measurement: three length-prefixed strings, then the numbers.
fn put_measure(m: &common::Measure) -> Vec<u8> {
    let mut out = Vec::new();
    let put_str = |out: &mut Vec<u8>, s: &str| {
        let b = &s.as_bytes()[..s.len().min(u16::MAX as usize)];
        out.extend_from_slice(&(b.len() as u16).to_le_bytes());
        out.extend_from_slice(b);
    };
    put_str(&mut out, m.modulation);
    put_str(&mut out, m.front_end);
    put_str(&mut out, m.mode.as_deref().unwrap_or(""));
    out.extend_from_slice(&m.confidence.to_le_bytes());
    out.extend_from_slice(&m.duration_us.to_le_bytes());
    for v in [m.bandwidth_hz, m.baud, m.separation_hz, m.sweep_hz_s, m.symbol_period_us] {
        out.extend_from_slice(&v.to_le_bytes());
    }
    out
}

/// The measurement at the front of a body, and the rest of the body.
fn take_measure(body: &[u8]) -> Option<(common::Measure, &[u8])> {
    let mut at = 0usize;
    let take_str = |at: &mut usize| -> Option<String> {
        let n = u16::from_le_bytes(body.get(*at..*at + 2)?.try_into().ok()?) as usize;
        *at += 2;
        let s = String::from_utf8_lossy(body.get(*at..*at + n)?).into_owned();
        *at += n;
        Some(s)
    };
    let modulation = take_str(&mut at)?;
    let front = take_str(&mut at)?;
    let mode = take_str(&mut at)?;
    let getf = |o: usize| -> Option<f32> { Some(f32::from_le_bytes(body.get(o..o + 4)?.try_into().ok()?)) };
    let confidence = getf(at)?;
    let duration_us = u32::from_le_bytes(body.get(at + 4..at + 8)?.try_into().ok()?);
    let nums: Vec<f32> = (0..5).map(|k| getf(at + 8 + k * 4)).collect::<Option<_>>()?;
    let rest = body.get(at + 28..)?;
    Some((
        common::Measure {
            modulation: common::Measure::label(&modulation),
            confidence,
            front_end: common::Measure::front(&front),
            mode: (!mode.is_empty()).then_some(mode),
            duration_us,
            bandwidth_hz: nums[0],
            baud: nums[1],
            separation_hz: nums[2],
            sweep_hz_s: nums[3],
            symbol_period_us: nums[4],
        },
        rest,
    ))
}

/// The samples after the body, and the length prefix grown to cover them.
///
/// The tail of the burst rather than its head when there are too many: a
/// packet ends where the front end stopped reading, so the last samples are
/// the ones with the signal in them and the first are the silence before it.
fn put_iq(mut rec: Vec<u8>, q: &common::IqBurst) -> Vec<u8> {
    let from = q.samples.len().saturating_sub(IQ_MAX_SAMPLES);
    let samples = &q.samples[from..];
    let mut raw = Vec::with_capacity(samples.len() * 4);
    for s in samples {
        for v in [s.re, s.im] {
            raw.extend_from_slice(&((v * 32767.0).round().clamp(-32768.0, 32767.0) as i16).to_le_bytes());
        }
    }
    rec.extend_from_slice(&q.rate.to_le_bytes());
    rec.extend_from_slice(&q.center_hz.to_le_bytes());
    // Compressed only when it helps. A burst of full-scale noise does not
    // compress, and a block that grew would cost the disk and the reader
    // both.
    match zstd::encode_all(&raw[..], IQ_ZSTD_LEVEL) {
        Ok(z) if z.len() + 4 < raw.len() => {
            rec.extend_from_slice(&((samples.len() as u32) | IQ_COMPRESSED).to_le_bytes());
            rec.extend_from_slice(&(z.len() as u32).to_le_bytes());
            rec.extend_from_slice(&z);
        }
        _ => {
            rec.extend_from_slice(&(samples.len() as u32).to_le_bytes());
            rec.extend_from_slice(&raw);
        }
    }
    let len = (rec.len() - 4) as u32;
    rec[..4].copy_from_slice(&len.to_le_bytes());
    rec
}

/// The samples at the tail of a record, when it has them.
fn take_iq(tail: &[u8]) -> Option<std::sync::Arc<common::IqBurst>> {
    if tail.len() < IQ_HEAD_LEN {
        return None;
    }
    let rate = f64::from_le_bytes(tail[0..8].try_into().ok()?);
    let center_hz = u64::from_le_bytes(tail[8..16].try_into().ok()?);
    let count = u32::from_le_bytes(tail[16..20].try_into().ok()?);
    let n = (count & !IQ_COMPRESSED) as usize;
    let raw: std::borrow::Cow<'_, [u8]> = if count & IQ_COMPRESSED != 0 {
        let len = u32::from_le_bytes(tail.get(20..24)?.try_into().ok()?) as usize;
        let z = tail.get(24..24 + len)?;
        zstd::decode_all(z).ok()?.into()
    } else {
        tail.get(IQ_HEAD_LEN..IQ_HEAD_LEN + n * 4)?.into()
    };
    let samples = raw
        .chunks_exact(4)
        .take(n)
        .map(|c| {
            let i = i16::from_le_bytes([c[0], c[1]]) as f32 / 32767.0;
            let q = i16::from_le_bytes([c[2], c[3]]) as f32 / 32767.0;
            common::C32::new(i, q)
        })
        .collect();
    Some(std::sync::Arc::new(common::IqBurst { rate, center_hz, samples }))
}

fn put_head(out: &mut Vec<u8>, kind: u8, count: u16, body_len: usize, p: &Packet) {
    out.extend_from_slice(&((HEAD_LEN + body_len) as u32).to_le_bytes());
    out.push(kind);
    out.push(keying_code(p.modulation()));
    out.extend_from_slice(&count.to_le_bytes());
    out.extend_from_slice(&p.at_us.to_le_bytes());
    out.extend_from_slice(&p.center_hz().to_le_bytes());
    out.extend_from_slice(&p.bandwidth_hz.to_le_bytes());
    out.extend_from_slice(&p.rssi_dbfs().to_le_bytes());
    out.extend_from_slice(&p.snr_db().to_le_bytes());
}

/// Read a log back.
///
/// A log nothing can read is a log nobody keeps, so the reader ships with the
/// writer and is tested against it. A truncated final record, which is what a
/// receiver killed mid-write leaves, ends the iteration rather than failing:
/// every complete record before it is still good.
pub fn read(path: impl AsRef<std::path::Path>) -> std::io::Result<Vec<Packet>> {
    let mut f = std::fs::File::open(path)?;
    let mut buf = Vec::new();
    f.read_to_end(&mut buf)?;
    Ok(parse(&buf))
}

pub fn parse(buf: &[u8]) -> Vec<Packet> {
    let mut out = Vec::new();
    if buf.len() < MAGIC.len() + 2 || &buf[..MAGIC.len()] != MAGIC {
        return out;
    }
    let mut at = MAGIC.len() + 2;
    while at + 4 <= buf.len() {
        let len = u32::from_le_bytes(buf[at..at + 4].try_into().unwrap()) as usize;
        at += 4;
        if len < HEAD_LEN || at + len > buf.len() {
            break;
        }
        let r = &buf[at..at + len];
        at += len;
        let kind = r[0];
        let count = u16::from_le_bytes(r[2..4].try_into().unwrap()) as usize;
        let get64 = |o: usize| u64::from_le_bytes(r[o..o + 8].try_into().unwrap());
        let get32 = |o: usize| u32::from_le_bytes(r[o..o + 4].try_into().unwrap());
        let getf = |o: usize| f32::from_le_bytes(r[o..o + 4].try_into().unwrap());
        let body = &r[HEAD_LEN..];
        let mut measure = None;
        let (packet_body, tail) = match kind {
            KIND_PULSES | KIND_MEASURED => {
                let mut body = body;
                if kind == KIND_MEASURED {
                    let Some((m, rest)) = take_measure(body) else { continue };
                    measure = Some(m);
                    body = rest;
                }
                let n = count.min(body.len() / 8);
                let mut pulses = Vec::with_capacity(n);
                for k in 0..n {
                    let o = k * 8;
                    pulses.push(Pulse {
                        mark: u32::from_le_bytes(body[o..o + 4].try_into().unwrap()),
                        gap: u32::from_le_bytes(body[o + 4..o + 8].try_into().unwrap()),
                    });
                }
                (
                    PacketBody::Pulses(common::Package {
                        pulses,
                        snr_db: getf(28),
                        rssi_dbfs: getf(24),
                        start_sample: 0,
                        center_hz: get64(12),
                        modulation: keying_from_code(r[1]),
                    }),
                    &body[n * 8..],
                )
            }
            KIND_BYTES => {
                let n = count.min(body.len());
                (
                    PacketBody::Frame(common::Frame {
                        bytes: body[..n].to_vec(),
                        center_hz: get64(12),
                        rssi_dbfs: getf(24),
                        snr_db: getf(28),
                        iq: None,
                    }),
                    &body[n..],
                )
            }
            // An unknown kind is skipped by its length rather than guessed
            // at, which is the whole reason the length comes first.
            _ => continue,
        };
        out.push(Packet {
            at_us: get64(4),
            bandwidth_hz: get32(20),
            body: packet_body,
            measure,
            // The log keeps evidence, not conclusions: a packet read back is
            // decoded again by whatever decoders exist now, which is the
            // reason a protocol written in October can read a burst recorded
            // in September.
            decodes: Vec::new(),
            iq: take_iq(tail),
            // The log is a record of what was on the air, in timings and
            // bytes. Speech is not written to it: an hour of a busy repeater
            // is gigabytes, and the decision to keep audio belongs to
            // whoever asked for a recording.
            audio: None,
        });
    }
    out
}

/// UTC date as `YYYY-MM-DD`, by civil-from-days rather than a calendar crate.
fn day_of(at_us: u64) -> String {
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

#[cfg(test)]
mod tests {
    use super::*;
    use nodes::PacketSink;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-pktlog-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn burst(center: u64) -> Packet {
        Packet::of_pulses(
            AT,
            31_250,
            common::Package {
                pulses: vec![
                    Pulse { mark: 500, gap: 1500 },
                    Pulse { mark: 1500, gap: 500 },
                    Pulse { mark: 500, gap: 9000 },
                ],
                snr_db: 18.5,
                rssi_dbfs: -21.25,
                start_sample: 0,
                center_hz: center,
                modulation: Some("OOK"),
            },
        )
    }

    #[test]
    fn a_measured_burst_round_trips_with_its_measurement() {
        let d = std::env::temp_dir().join(format!("sr-wspkt-measure-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        let mut log = PacketLog::new(d.clone());
        let mut p = burst(869_500_000);
        if let PacketBody::Pulses(pkg) = &mut p.body {
            pkg.pulses.clear();
        }
        p.measure = Some(common::Measure {
            modulation: "chirp",
            confidence: 0.83,
            front_end: "none",
            mode: Some("LoRa SF9 BW125 (EU868)".into()),
            duration_us: 183_000,
            bandwidth_hz: 125_000.0,
            baud: 0.0,
            separation_hz: 0.0,
            sweep_hz_s: 30_500_000.0,
            symbol_period_us: 0.0,
        });
        let mut q = burst(433_920_000);
        q.measure = Some(common::Measure {
            modulation: "OOK",
            confidence: 0.91,
            front_end: "ook",
            mode: None,
            duration_us: 184_000,
            bandwidth_hz: 11_700.0,
            baud: 1_500.0,
            separation_hz: 0.0,
            sweep_hz_s: 0.0,
            symbol_period_us: 0.0,
        });
        nodes::PacketSink::write(&mut log, &p);
        nodes::PacketSink::write(&mut log, &q);
        log.flush();
        let got = read(d.join(format!("{}.000.wspkt", day_of(AT)))).unwrap();
        assert_eq!(got, vec![p, q]);
        let _ = std::fs::remove_dir_all(&d);
    }

    fn frame(at_us: u64, bytes: &[u8]) -> Packet {
        Packet::of_frame(
            at_us,
            2_000_000,
            common::Frame::unmeasured(bytes.to_vec()).at(1_090_000_000),
        )
    }

    /// 2026-08-31T12:00:00Z, in microseconds.
    const AT: u64 = 1_788_177_600_000_000;

    #[test]
    fn a_burst_comes_back_exactly_as_it_was_detected() {
        // The whole point of storing the demodulator's output: what comes
        // back has to be good enough to decode again.
        let d = dir("roundtrip");
        let mut log = PacketLog::new(d.clone());
        let p = burst(433_920_000);
        log.write(&p);
        log.flush();

        let got = read(d.join("2026-08-31.000.wspkt")).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0], p, "what came back is not what was heard");
        // And it is a package again, ready for a decoder that did not exist
        // when it was written.
        assert_eq!(got[0].package().map(|p| p.pulses.len()), Some(3));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_samples_are_compressed_and_bounded() {
        // Two things a day's log depends on: a record carries the end of the
        // burst rather than every sample a front end had, and what it does
        // carry is compressed. A 2.4 GHz session wrote 122 GB in a day
        // without either.
        let d = dir("iqsize");
        let mut log = PacketLog::new(d.clone());
        let mut p = burst(868_300_000);
        // A quiet channel with a burst at the end of it, which is the shape
        // the ring behind a front end has.
        let mut samples = vec![common::C32::new(0.0, 0.0); 400_000];
        for (i, s) in samples.iter_mut().enumerate().skip(390_000) {
            let ph = i as f32 * 0.7;
            *s = common::C32::new(ph.cos() * 0.5, ph.sin() * 0.5);
        }
        p.iq = Some(std::sync::Arc::new(common::IqBurst {
            rate: 1_000_000.0,
            center_hz: 868_300_000,
            samples,
        }));
        log.write(&p);
        log.flush();
        let path = d.join(format!("{}.000.wspkt", day_of(AT)));
        let size = std::fs::metadata(&path).unwrap().len();
        assert!(size < 200_000, "a record of {size} bytes for one burst");
        let got = read(&path).unwrap();
        let iq = got[0].iq.as_ref().expect("the samples came back");
        assert_eq!(iq.samples.len(), IQ_MAX_SAMPLES, "kept {} samples", iq.samples.len());
        // The end of the burst, which is where the signal was.
        let last = iq.samples.last().unwrap();
        assert!(last.norm() > 0.4, "the tail is silence: {last:?}");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_burst_comes_back_with_its_samples() {
        let d = dir("iq");
        let mut log = PacketLog::new(d.clone());
        let mut p = frame(AT, &[0x4c, 0x6f, 0x52, 0x61]);
        p.iq = Some(std::sync::Arc::new(common::IqBurst {
            rate: 62_500.0,
            center_hz: 869_618_000,
            samples: (0..1000)
                .map(|i| common::C32::new((i as f32 * 0.01).sin() * 0.5, (i as f32 * 0.01).cos() * 0.5))
                .collect(),
        }));
        let mut q = burst(433_920_000);
        q.iq = Some(std::sync::Arc::new(common::IqBurst {
            rate: 250_000.0,
            center_hz: 433_920_000,
            samples: vec![common::C32::new(1.0, -1.0); 10],
        }));
        log.write(&p);
        log.write(&q);
        log.flush();
        let got = read(d.join("2026-08-31.000.wspkt")).unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].frame(), p.frame());
        let (a, b) = (got[0].iq.as_ref().unwrap(), p.iq.as_ref().unwrap());
        assert_eq!((a.rate, a.center_hz, a.samples.len()), (b.rate, b.center_hz, b.samples.len()));
        let err = a.samples.iter().zip(&b.samples).map(|(x, y)| (x - y).norm()).fold(0.0f32, f32::max);
        assert!(err < 1e-4, "samples moved by {err}");
        assert_eq!(got[1].iq.as_ref().unwrap().samples[0], common::C32::new(1.0, -1.0));
        assert_eq!(got[1].package().map(|p| p.pulses.len()), Some(3));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_byte_frame_is_stored_as_bytes() {
        // Mode S demodulates to frames rather than timings, and storing those
        // is storing the demodulator's output just the same.
        let d = dir("bytes");
        let mut log = PacketLog::new(d.clone());
        let bytes = [0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3];
        log.write(&frame(AT, &bytes));
        log.flush();

        let got = read(d.join("2026-08-31.000.wspkt")).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].frame(), Some(&bytes[..]));
        assert_eq!(got[0].center_hz(), 1_090_000_000);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_receiver_killed_mid_write_costs_one_record() {
        // How a log actually ends: not closed, but killed. Every complete
        // record before the tear has to survive.
        let d = dir("torn");
        let mut log = PacketLog::new(d.clone());
        for _ in 0..3 {
            log.write(&burst(868_300_000));
        }
        log.flush();
        let path = d.join("2026-08-31.000.wspkt");
        let mut raw = std::fs::read(&path).unwrap();
        raw.truncate(raw.len() - 9);
        assert_eq!(parse(&raw).len(), 2, "a torn tail took a good record with it");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_day_rolls_over_into_a_second_file() {
        let d = dir("roll");
        let mut log = PacketLog::new(d.clone());
        log.write(&burst(433_920_000));
        let mut tomorrow = burst(433_920_000);
        tomorrow.at_us += 86_400_000_000;
        log.write(&tomorrow);
        log.flush();
        assert!(d.join("2026-08-31.000.wspkt").exists());
        assert!(d.join("2026-09-01.000.wspkt").exists());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_days_roll_carries_the_buffer_with_it() {
        // Buffered records belong to the file they were written for, and the
        // roll is where a forgotten flush loses a whole day.
        let d = dir("roll-flush");
        let mut log = PacketLog::new(d.clone());
        log.write(&burst(433_920_000));
        let mut tomorrow = burst(433_920_000);
        tomorrow.at_us += 86_400_000_000;
        log.write(&tomorrow);
        assert_eq!(read(d.join("2026-08-31.000.wspkt")).unwrap().len(), 1, "yesterday was lost");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_dropped_log_still_has_its_last_burst() {
        // How a receiver ends: the log goes out of scope with records still
        // in the buffer.
        let d = dir("drop");
        let mut log = PacketLog::new(d.clone());
        log.write(&burst(433_920_000));
        drop(log);
        assert_eq!(read(d.join("2026-08-31.000.wspkt")).unwrap().len(), 1);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_burst_of_packets_costs_one_write() {
        // The reason for the buffer: a thousand packets arriving inside the
        // flush deadline must not be a thousand syscalls.
        let d = dir("batch");
        let mut log = PacketLog::new(d.clone());
        for _ in 0..1000 {
            log.write(&burst(868_300_000));
        }
        // Nothing has reached the disk yet beyond what overflowed the buffer,
        // and the count on screen is still honest about what was accepted.
        assert_eq!(log.written(), 1000);
        log.flush();
        assert_eq!(read(d.join("2026-08-31.000.wspkt")).unwrap().len(), 1000);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// The limit is on the folder, so it is the folder that has to come back
    /// under it. Deleting the oldest day is what a rolling log does; stopping
    /// dead with a week of disk still free is what the per-day limit did.
    #[test]
    fn the_oldest_days_go_to_keep_the_folder_under_its_limit() {
        let d = dir("prune");
        std::fs::create_dir_all(&d).unwrap();
        for day in ["2026-08-28", "2026-08-29", "2026-08-30"] {
            std::fs::write(d.join(format!("{day}.{EXT}")), vec![0u8; 40_000]).unwrap();
        }
        // Room for the three kept days and a little of today's.
        let mut log = PacketLog::new(d.clone()).with_cap(Some(100_000));
        for _ in 0..100 {
            log.write(&burst(868_300_000));
        }
        log.flush();
        assert!(!d.join("2026-08-28.wspkt").exists(), "the oldest segment was kept");
        assert!(d.join("2026-08-30.wspkt").exists(), "a recent segment was thrown away");
        assert!(d.join("2026-08-31.000.wspkt").exists(), "today was not written");
        assert!(!log.full(), "logging stopped although there was room to make");
        assert!(log.total() < 100_000, "the folder is {} bytes", log.total());
        let _ = std::fs::remove_dir_all(&d);
    }

    /// A voice over is on the bus so it can be heard, listed and read, and
    /// its body is empty: there is nothing in it for a decoder to find later,
    /// which is what this file is for.
    #[test]
    fn speech_is_not_evidence_and_is_not_logged() {
        let d = dir("voice");
        let mut log = PacketLog::new(d.clone());
        let mut over = Packet::of_frame(AT, 12_500, common::Frame::unmeasured(Vec::new()));
        over.audio = Some(std::sync::Arc::new(common::Speech {
            pcm: vec![0.1; 48_000],
            rate: 48_000.0,
        }));
        log.write(&over);
        log.write(&burst(868_300_000));
        log.flush();
        let got = read(d.join(format!("{}.000.wspkt", day_of(AT)))).expect("the log reads back");
        assert_eq!(got.len(), 1, "the over went into the log");
        assert!(matches!(got[0].body, PacketBody::Pulses(_)));
        let _ = std::fs::remove_dir_all(&d);
    }

    /// What the interface shows is what the folder holds, and a log that has
    /// not written anything yet holds whatever last night wrote. It read 0 B
    /// until the first burst arrived, which on a quiet band is never.
    #[test]
    fn a_log_that_has_written_nothing_reports_the_folder() {
        let d = dir("holds");
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join(format!("2026-08-30.000.{EXT}")), vec![0u8; 40_000]).unwrap();
        let mut log = PacketLog::new(d.clone());
        assert_eq!(log.total(), 40_000, "the folder was not measured");
        // And it follows the folder afterwards, whoever emptied it.
        std::fs::remove_file(d.join(format!("2026-08-30.000.{EXT}"))).unwrap();
        log.measured -= std::time::Duration::from_secs(5);
        log.refresh_folder();
        assert_eq!(log.total(), 0);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_file_that_is_not_a_log_reads_as_empty() {
        assert!(parse(b"this is not a packet log at all").is_empty());
        assert!(parse(b"").is_empty());
    }

    #[test]
    fn dates_are_utc_civil_days() {
        assert_eq!(day_of(AT), "2026-08-31");
        assert_eq!(day_of(0), "1970-01-01");
        // A leap day, which is where a hand-rolled calendar goes wrong.
        assert_eq!(day_of(1_709_208_000_000_000), "2024-02-29");
    }
}
