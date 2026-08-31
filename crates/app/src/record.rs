//! Writing bursts to disk as they are received.
//!
//! Adding a protocol means looking at the same signal over and over: slice it
//! this way, try that timing, check whether the CRC lands. Doing that against
//! a live radio means waiting for the device to transmit again between every
//! attempt, and it means the thing you are debugging is never twice the same.
//! A recorded burst is a fixture: it decodes identically every run, it goes in
//! a test, and it can be handed to someone else along with a bug report.
//!
//! The captures are ordinary rtl_433 style files, `<name>_<freq>_<rate>.cu8`,
//! so `sources::FileSource` reads them back without being told anything and
//! rtl_433 itself can be pointed at them for comparison. That mattered more
//! than saving a few bytes with a private format.

use crate::radio::DecodeRecord;
use common::{Hz, C32};
use dsp::fir::FirDecim;
use dsp::mixer::Mixer;
use std::io::Write;
use std::path::{Path, PathBuf};

/// Seconds of signal kept before the block that reported a decode.
///
/// A packet is reported long after it was transmitted. The transmission
/// itself takes time (a Fine Offset frame runs about 90 ms), a pulse detector
/// only closes a burst once it has seen the silence after it, the channelizer
/// and the per-channel filters add their own latency, and the whole lot is
/// quantised to the block the radio happened to deliver.
///
/// Measured on the Fine Offset capture at 250 kS/s in 16384 sample blocks,
/// which is the worst case because a block is 65 ms at that rate: 0.25 s of
/// history loses the packet, 0.3 s catches it. This is set well above that
/// because a lost burst is not recoverable and the memory is cheap: even at
/// 20 MS/s the ring is 120 MB, and at the usual 2.4 MS/s it is 14 MB.
const PRE_ROLL: f64 = 0.75;

/// Bandwidth kept around the burst, and so the rate the file is written at.
///
/// Wide enough for any of these protocols, including FSK with a deviation of
/// tens of kilohertz, and wide enough that a neighbouring transmission is
/// still in the file rather than filtered into silence, which matters when the
/// thing being debugged is why two signals were confused. It is also the rate
/// most rtl_433 captures use, so the files look like the ones already in
/// `testdata`.
const TARGET_RATE: f64 = 250_000.0;

/// Stop after this much has been written.
///
/// A busy 868 MHz band produces about one burst a second, and an unattended
/// receiver left running overnight would otherwise fill a disk. Reaching the
/// limit is reported once and then recording simply stops, because the
/// alternative is a receiver that dies at 4am for a reason nobody sees.
const DEFAULT_BUDGET: u64 = 256 << 20;

/// A window of recent samples, kept so a burst can be written out after the
/// decoder has finished with it.
struct Ring {
    buf: Vec<C32>,
    /// Where the next sample goes, and how many have ever been pushed. The
    /// count is the clock: a burst is identified by the sample index of the
    /// block it came from, which survives the ring wrapping around.
    write: usize,
    total: u64,
}

impl Ring {
    fn new(len: usize) -> Self {
        Self { buf: vec![C32::new(0.0, 0.0); len.max(1)], write: 0, total: 0 }
    }

    fn push(&mut self, iq: &[C32]) {
        // A block longer than the whole ring can only leave its tail, which is
        // the part nearest the decode anyway.
        let iq = if iq.len() > self.buf.len() { &iq[iq.len() - self.buf.len()..] } else { iq };
        let n = self.buf.len();
        for &s in iq {
            self.buf[self.write] = s;
            self.write = (self.write + 1) % n;
        }
        self.total += iq.len() as u64;
    }

    /// Copy `len` samples starting at absolute index `from`, or as much of
    /// that range as the ring still holds.
    fn copy(&self, from: u64, len: usize) -> Vec<C32> {
        let oldest = self.total.saturating_sub(self.buf.len() as u64);
        let from = from.max(oldest);
        let end = (from + len as u64).min(self.total);
        if end <= from {
            return Vec::new();
        }
        let n = self.buf.len();
        // `write` holds the position of the newest sample plus one, which is
        // also where the oldest one lives once the ring has wrapped.
        let start = (self.write + n - (self.total - from) as usize % n) % n;
        let count = (end - from) as usize;
        (0..count).map(|i| self.buf[(start + i) % n]).collect()
    }
}

/// Writes every reported burst to a directory of captures.
pub struct Recorder {
    dir: PathBuf,
    rate: f64,
    center: f64,
    ring: Ring,
    pre_roll: f64,
    seq: u32,
    written: u64,
    budget: u64,
    full: bool,
    /// Sample index of the block currently being processed, so a decode found
    /// in it can be located in the ring.
    block_start: u64,
    block_len: usize,
}

impl Recorder {
    pub fn new(dir: impl AsRef<Path>, rate: f64, center: Hz) -> std::io::Result<Self> {
        let dir = dir.as_ref().to_path_buf();
        std::fs::create_dir_all(&dir)?;
        Ok(Self {
            dir,
            rate,
            center: center.as_f64(),
            ring: Ring::new((rate * PRE_ROLL) as usize + 1),
            pre_roll: PRE_ROLL,
            seq: 0,
            written: 0,
            budget: DEFAULT_BUDGET,
            full: false,
            block_start: 0,
            block_len: 0,
        })
    }

    /// Shorten the history, for tests that measure how much is needed.
    #[cfg(test)]
    fn with_pre_roll(mut self, secs: f64) -> Self {
        self.pre_roll = secs;
        self.ring = Ring::new((self.rate * secs) as usize + 1);
        self
    }

    /// Change how much may be written before recording stops.
    pub fn with_budget(mut self, bytes: u64) -> Self {
        self.budget = bytes;
        self
    }

    /// Retuning or resampling invalidates the history: the samples still in
    /// the ring came from a different part of the spectrum.
    pub fn retune(&mut self, rate: f64, center: Hz) {
        if rate != self.rate {
            self.ring = Ring::new((rate * self.pre_roll) as usize + 1);
        }
        self.rate = rate;
        self.center = center.as_f64();
        self.ring.total = 0;
        self.ring.write = 0;
    }

    /// Take a block of samples, before the scanner is run on it.
    pub fn push(&mut self, iq: &[C32]) {
        self.block_start = self.ring.total;
        self.block_len = iq.len();
        self.ring.push(iq);
    }

    /// Write out the burst a decode came from.
    ///
    /// Returns the file written, or `None` when the burst has already fallen
    /// out of the ring or the disk budget is spent.
    pub fn capture(&mut self, r: &DecodeRecord) -> Option<PathBuf> {
        if self.full {
            return None;
        }
        let pre = (self.rate * self.pre_roll) as u64;
        let from = self.block_start.saturating_sub(pre);
        let len = (self.block_start - from) as usize + self.block_len;
        let raw = self.ring.copy(from, len);
        if raw.is_empty() {
            return None;
        }

        let factor = ((self.rate / TARGET_RATE).floor() as usize).max(1);
        let out_rate = self.rate / factor as f64;
        let mut shifted = Vec::with_capacity(raw.len());
        // Mix the burst to DC before filtering, so what survives the decimator
        // is the band around the signal rather than the band around wherever
        // the receiver happened to be tuned.
        Mixer::new(self.center - r.freq, self.rate).process(&raw, &mut shifted);
        let iq = if factor > 1 {
            let mut out = Vec::with_capacity(shifted.len() / factor + 1);
            FirDecim::design_hz(self.rate, factor, out_rate * 0.4, 60.0).process(&shifted, &mut out);
            out
        } else {
            shifted
        };

        self.seq += 1;
        // The sequence number is prefixed with a letter because the filename
        // is also the metadata: `sources::parse_filename` reads the frequency
        // and rate out of it by looking for numeric tokens, and a bare `0001`
        // is a perfectly good number. It was read as a sample rate of 1 Hz,
        // and the capture replayed as silence.
        let name = format!(
            "g{:04}_{}_{}_{:.4}M_{:.0}k.cu8",
            self.seq,
            sanitise(&r.model),
            r.modulation.to_ascii_lowercase(),
            r.freq / 1e6,
            out_rate / 1e3,
        );
        let path = self.dir.join(&name);
        let bytes = to_cu8(&iq);
        if std::fs::write(&path, &bytes).is_err() {
            return None;
        }
        self.written += bytes.len() as u64;
        self.full = self.written >= self.budget;
        self.append_index(&name, r, out_rate, iq.len());
        Some(path)
    }

    /// Whether the budget has been reached, so the caller can say so once.
    pub fn is_full(&self) -> bool {
        self.full
    }

    pub fn written(&self) -> u64 {
        self.written
    }

    /// One JSON object per capture, appended.
    ///
    /// Written by hand rather than through a serialiser because the shape is
    /// flat and the file's whole purpose is to be read by something else,
    /// `jq` or a script or a person, without this program being involved.
    fn append_index(&self, name: &str, r: &DecodeRecord, rate: f64, samples: usize) {
        let Ok(mut f) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(self.dir.join("index.jsonl"))
        else {
            return;
        };
        let hex: String = r.bytes.iter().map(|b| format!("{b:02x}")).collect();
        let crc = match r.crc {
            Some(true) => "\"ok\"",
            Some(false) => "\"bad\"",
            None => "null",
        };
        let _ = writeln!(
            f,
            concat!(
                r#"{{"file":"{}","freq_hz":{:.0},"rate_hz":{:.0},"samples":{},"#,
                r#""protocol":"{}","modulation":"{}","rssi_dbfs":{:.1},"snr_db":{:.1},"#,
                r#""crc":{},"bytes":"{}","detail":"{}"}}"#
            ),
            esc(name),
            r.freq,
            rate,
            samples,
            esc(&r.model),
            esc(r.modulation),
            r.rssi_dbfs,
            r.snr_db,
            crc,
            hex,
            esc(&r.detail),
        );
    }
}

/// Quantise to the eight bits the radios actually deliver.
///
/// Both an RTL-SDR and a HackRF hand over eight bit samples, so writing
/// anything wider stores precision the capture never had. Clipped rather than
/// scaled: a burst loud enough to clip was clipping in the receiver too, and
/// rescaling here would hide that from whoever debugs the file later.
fn to_cu8(iq: &[C32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(iq.len() * 2);
    for s in iq {
        for v in [s.re, s.im] {
            out.push((v * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8);
        }
    }
    out
}

fn sanitise(s: &str) -> String {
    let s: String = s
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' { c } else { '-' })
        .collect();
    if s.is_empty() { "unknown".into() } else { s }
}

fn esc(s: &str) -> String {
    s.chars()
        .flat_map(|c| match c {
            '"' => vec!['\\', '"'],
            '\\' => vec!['\\', '\\'],
            c if (c as u32) < 0x20 => vec![' '],
            c => vec![c],
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ramp(n: usize, start: f32) -> Vec<C32> {
        (0..n).map(|i| C32::new(start + i as f32, 0.0)).collect()
    }

    #[test]
    fn the_ring_returns_the_samples_it_was_given() {
        let mut r = Ring::new(100);
        r.push(&ramp(30, 0.0));
        let got = r.copy(0, 30);
        assert_eq!(got.len(), 30);
        assert_eq!(got[0].re, 0.0);
        assert_eq!(got[29].re, 29.0);
    }

    #[test]
    fn the_ring_keeps_reading_correctly_after_it_wraps() {
        // The whole point of the ring is that it survives hours of running,
        // so wrapping has to be as ordinary as the first block.
        let mut r = Ring::new(100);
        for k in 0..7 {
            r.push(&ramp(30, k as f32 * 30.0));
        }
        let got = r.copy(180, 30);
        assert_eq!(got.iter().map(|s| s.re).collect::<Vec<_>>(), (180..210).map(|v| v as f32).collect::<Vec<_>>());
    }

    #[test]
    fn a_burst_older_than_the_ring_is_dropped_rather_than_guessed_at() {
        let mut r = Ring::new(100);
        r.push(&ramp(300, 0.0));
        // Asking for something long gone gives back what is still held, not
        // silence dressed up as signal.
        let got = r.copy(0, 50);
        assert!(got.iter().all(|s| s.re >= 200.0), "stale samples were served");
    }

    /// Record a real transmission, then decode the file that was written.
    ///
    /// This is the whole promise of the recorder: what comes back off the disk
    /// has to be the same packet, or every fixture built with it is a lie.
    #[test]
    fn a_recorded_burst_decodes_the_same_way_on_replay() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
        if !p.exists() {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        }
        let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
        let dir = std::env::temp_dir().join(format!("sr-record-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);

        let rec = Recorder::new(&dir, buf.rate.as_f64(), buf.center).unwrap();
        let (live, _rec) = crate::radio::scan_with_recorder(&buf, rec);
        assert!(
            live.iter().any(|r| r.model.contains("Fineoffset")),
            "the fixture did not decode live, so replay proves nothing"
        );

        let files: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("cu8"))
            .collect();
        assert!(!files.is_empty(), "nothing was written to {}", dir.display());

        let mut found = false;
        for f in &files {
            for r in crate::radio::replay(f).unwrap() {
                if r.model.contains("Fineoffset") {
                    found = true;
                    assert_eq!(
                        r.fields.iter().find(|(k, _)| k == "temperature_c").map(|(_, v)| v.as_f64()),
                        Some(Some(16.2)),
                        "the same packet came back with a different reading"
                    );
                }
            }
        }
        assert!(found, "{} capture(s) written and none decoded", files.len());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_quarter_second_of_history_is_not_enough() {
        // The reason PRE_ROLL is what it is. A packet takes time to send, the
        // detector waits for silence after it, and the filters add latency, so
        // by the time a decode is reported the burst is long past. If this
        // ever starts passing, the pipeline got faster and the constant can
        // come down; if the round trip test starts failing, look here first.
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
        if !p.exists() {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        }
        let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
        let dir = std::env::temp_dir().join(format!("sr-short-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let rec = Recorder::new(&dir, buf.rate.as_f64(), buf.center).unwrap().with_pre_roll(0.25);
        crate::radio::scan_with_recorder(&buf, rec);

        let decoded = std::fs::read_dir(&dir)
            .unwrap()
            .filter_map(|e| e.ok().map(|e| e.path()))
            .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("cu8"))
            .any(|f| {
                crate::radio::replay(f).unwrap().iter().any(|r| r.model.contains("Fineoffset"))
            });
        assert!(!decoded, "0.25 s now catches the burst; PRE_ROLL can be reduced");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn a_capture_carries_its_tuning_in_its_name() {
        // The filename is the only metadata a replay reads, so it has to
        // survive the parser that reads it. It did not, once.
        let dir = std::env::temp_dir().join(format!("sr-name-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut r = Recorder::new(&dir, 250_000.0, Hz::mhz(434)).unwrap();
        r.push(&ramp(4096, 0.0));
        let rec = crate::radio::DecodeRecord::for_test(433_920_000.0, "Fineoffset-WHx080");
        let path = r.capture(&rec).unwrap();

        let meta = sources::parse_filename(&path);
        assert_eq!(meta.center, Some(Hz(433_920_000)));
        assert_eq!(meta.rate, Some(common::Sps(250_000)));
        assert_eq!(meta.format, Some(common::SampleFormat::Cu8));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn the_index_names_every_capture_that_was_written() {
        let dir = std::env::temp_dir().join(format!("sr-index-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut r = Recorder::new(&dir, 250_000.0, Hz::mhz(434)).unwrap();
        r.push(&ramp(4096, 0.0));
        let mut rec = crate::radio::DecodeRecord::for_test(433_920_000.0, "Fineoffset-WHx080");
        rec.detail = "temperature_c=16.2".into();
        let path = r.capture(&rec).expect("a burst still in the ring must be written");
        let name = path.file_name().unwrap().to_str().unwrap();

        let index = std::fs::read_to_string(dir.join("index.jsonl")).unwrap();
        assert!(index.contains(name), "the capture is not in the index");
        assert!(index.contains("\"freq_hz\":433920000"), "{index}");
        assert!(index.contains("temperature_c=16.2"), "{index}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn full_scale_stays_inside_eight_bits() {
        let b = to_cu8(&[C32::new(1.0, -1.0), C32::new(2.0, -2.0), C32::new(0.0, 0.0)]);
        assert_eq!(b[0], 255);
        assert_eq!(b[1], 0);
        assert_eq!(b[2], 255, "a sample past full scale must clip, not wrap");
        assert_eq!(b[4], 128);
    }
}

