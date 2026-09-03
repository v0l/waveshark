//! Writing the span itself to disk, exactly as it arrived.
//!
//! The burst recorder in the application writes what a decoder already
//! understood: it is triggered by a decode, and what it saves is a slice
//! around one. That is the wrong tool for a signal nothing decodes, which is
//! the only interesting kind. When the receiver shows a transmission and
//! reads nothing from it, the evidence needed is the raw span over the whole
//! transmission, with no gate, no trigger and no protocol involved.
//!
//! So this is a tap that writes every sample it is given to one file, named
//! the way `sources::FileSource` reads it back, `<name>_<freq>M_<rate>k.cu8`.
//! Replaying that file puts the same samples through the same graph, and a
//! decoder can then be changed and tried again against a signal that is
//! identical every run.
//!
//! A capture is large: 2.4 MS/s as unsigned bytes is 4.8 MB a second, and as
//! 16-bit pairs twice that. The budget is therefore part of the node rather
//! than something the caller is trusted to watch, and reaching it stops the
//! writing rather than the receiver.

use common::{Error, Hz, Result, SampleFormat, C32};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::io::Write;
use std::path::{Path, PathBuf};

/// Bytes written before a capture stops on its own.
///
/// A gigabyte is three and a half minutes of a 2.4 MS/s span as bytes, which
/// is long enough for any transmission somebody is standing there waiting
/// for, and small enough that forgetting to stop costs a gigabyte rather
/// than a filesystem.
pub const DEFAULT_BUDGET: u64 = 1 << 30;

/// A file being written, and what is known about it.
struct Sink {
    path: PathBuf,
    file: std::io::BufWriter<std::fs::File>,
    bytes: u64,
    samples: u64,
}

/// The raw IQ capture: everything that passes, to a file.
pub struct IqCaptureNode {
    dir: PathBuf,
    name: String,
    format: SampleFormat,
    budget: u64,
    enabled: bool,
    rate: f64,
    center: Hz,
    sink: Option<Sink>,
    /// Set once the budget is spent, so the file is closed and the state is
    /// reportable rather than the node silently doing nothing.
    full: bool,
    /// The last thing that went wrong, reported once through an event and
    /// kept for a status line.
    error: Option<String>,
    reported: bool,
    buf: Vec<u8>,
}

impl IqCaptureNode {
    pub fn new(dir: impl AsRef<Path>) -> Self {
        Self {
            dir: dir.as_ref().to_path_buf(),
            name: "capture".into(),
            format: SampleFormat::Cu8,
            budget: DEFAULT_BUDGET,
            enabled: true,
            rate: 0.0,
            center: Hz(0),
            sink: None,
            full: false,
            error: None,
            reported: false,
            buf: Vec::new(),
        }
    }

    /// What the file is called before the frequency and rate are appended.
    pub fn with_name(mut self, name: impl Into<String>) -> Self {
        let name = sanitise(&name.into());
        self.name = name;
        self
    }

    pub fn with_format(mut self, f: SampleFormat) -> Self {
        self.format = f;
        self
    }

    /// How much may be written before the capture stops, in bytes.
    pub fn with_budget(mut self, bytes: u64) -> Self {
        self.budget = bytes;
        self
    }

    pub fn with_enabled(mut self, on: bool) -> Self {
        self.enabled = on;
        self
    }

    /// The file being written, once there is one.
    pub fn path(&self) -> Option<&Path> {
        self.sink.as_ref().map(|s| s.path.as_path())
    }

    pub fn bytes(&self) -> u64 {
        self.sink.as_ref().map(|s| s.bytes).unwrap_or(0)
    }

    /// Seconds of signal written, which is what somebody watching wants to
    /// know: bytes are an implementation detail of the format.
    pub fn seconds(&self) -> f64 {
        match (&self.sink, self.rate) {
            (Some(s), r) if r > 0.0 => s.samples as f64 / r,
            _ => 0.0,
        }
    }

    pub fn budget(&self) -> u64 {
        self.budget
    }

    pub fn is_full(&self) -> bool {
        self.full
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn error(&self) -> Option<&str> {
        self.error.as_deref()
    }

    pub fn format(&self) -> SampleFormat {
        self.format
    }

    /// Stop or start writing. Stopping closes the file, so what is on disk is
    /// complete and replayable the moment the button comes back up; starting
    /// again opens a new one rather than appending to a file whose name says
    /// it was recorded somewhere else.
    ///
    /// Switching on a capture that stopped at its budget starts it again,
    /// because that is what pressing the button after reading why it stopped
    /// is asking for.
    pub fn set_enabled(&mut self, on: bool) {
        if on == self.enabled && !(on && (self.full || self.error.is_some())) {
            return;
        }
        self.enabled = on;
        self.close();
        if on {
            self.full = false;
            self.error = None;
            self.reported = false;
        }
    }

    fn close(&mut self) {
        if let Some(s) = &mut self.sink {
            let _ = s.file.flush();
        }
        self.sink = None;
    }

    /// Open the file for the tuning in force, named so that replaying it
    /// needs no arguments.
    fn open(&mut self, at_us: u64) -> Result<()> {
        std::fs::create_dir_all(&self.dir)?;
        let name = format!(
            "{}_{}_{:.4}M_{:.0}k.{}",
            self.name,
            stamp(at_us),
            self.center.as_f64() / 1e6,
            self.rate / 1e3,
            self.format.extension(),
        );
        let path = self.dir.join(name);
        let file = std::fs::File::create(&path)?;
        self.sink = Some(Sink {
            path,
            // A megabyte at a time: at 2.4 MS/s the graph delivers a block
            // every few milliseconds, and one write syscall each would be
            // thousands a second for no reason.
            file: std::io::BufWriter::with_capacity(1 << 20, file),
            bytes: 0,
            samples: 0,
        });
        Ok(())
    }

    fn write(&mut self, iq: &[C32]) -> Result<()> {
        self.buf.clear();
        self.format.encode(iq, &mut self.buf);
        let Some(s) = &mut self.sink else { return Ok(()) };
        s.file.write_all(&self.buf)?;
        s.bytes += self.buf.len() as u64;
        s.samples += iq.len() as u64;
        if self.budget > 0 && s.bytes >= self.budget {
            self.full = true;
            self.close();
        }
        Ok(())
    }
}

impl Simple for IqCaptureNode {
    fn name(&self) -> &str {
        "iq_capture"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(Error::other("iq_capture needs IQ"));
        }
        // The name carries the frequency and the rate, so a change to either
        // ends the file: one capture is one tuning, and a replay that trusts
        // the name has to be right about every sample in it.
        if i.spec.rate != self.rate || i.spec.center != self.center {
            self.close();
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let iq = i.as_iq().unwrap_or(&[]);
        if !self.enabled || self.full || iq.is_empty() {
            return Ok(());
        }
        // Opened on the first block rather than at negotiation, so a graph
        // that is built and thrown away leaves no empty file behind.
        if self.sink.is_none() {
            if let Err(e) = self.open(now_us()) {
                self.fail(format!("cannot open a capture in {}: {e}", self.dir.display()), c);
                return Ok(());
            }
        }
        if let Err(e) = self.write(iq) {
            let path = self.path().map(|p| p.display().to_string()).unwrap_or_default();
            self.fail(format!("cannot write {path}: {e}"), c);
            self.close();
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.close();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("enabled", self.enabled).label("Write the span to disk"),
            Param::float("budget_mb", self.budget as f64 / (1 << 20) as f64, 16.0..=65_536.0)
                .unit("MB")
                .label("Stop after")
                .log(),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => {
                self.set_enabled(v.as_bool().unwrap_or(true));
                Ok(())
            }
            "budget_mb" => {
                self.budget = (v.as_f64().unwrap_or(0.0).max(0.0) * (1 << 20) as f64) as u64;
                Ok(())
            }
            _ => Err(Error::other(format!("iq_capture: unknown parameter {name:?}"))),
        }
    }
}

impl IqCaptureNode {
    /// Report a problem once and stop trying, so a full disk does not fill
    /// the event log at the rate blocks arrive.
    fn fail(&mut self, message: String, c: &mut NodeCtx<'_>) {
        if !self.reported {
            self.reported = true;
            c.emit(pipeline::event::Event::Warning { stage: "iq_capture".into(), message: message.clone() });
        }
        self.error = Some(message);
        self.full = true;
    }
}

/// UTC as `YYYYmmdd-HHMMSS`.
///
/// The hyphen is not decoration: `sources::parse_filename` reads the
/// frequency and rate out of a name by looking for numeric tokens, and a bare
/// run of digits is a perfectly good number. One was read as a sample rate of
/// 1 Hz once already, in the burst recorder.
fn stamp(at_us: u64) -> String {
    let secs = (at_us / 1_000_000) as i64;
    let (days, rem) = (secs.div_euclid(86_400), secs.rem_euclid(86_400));
    // Howard Hinnant's civil_from_days, which is exact and fits here.
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
    format!("{y:04}{m:02}{d:02}-{:02}{:02}{:02}", rem / 3600, rem / 60 % 60, rem % 60)
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// Anything that would confuse the name back into metadata, or a shell.
fn sanitise(s: &str) -> String {
    let s: String = s
        .chars()
        .filter(|c| c.is_ascii_alphanumeric() || *c == '-')
        .collect();
    if s.is_empty() { "capture".into() } else { s }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-iqcap-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn spec(rate: f64, center: Hz) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, center), latency: 0 }
    }

    fn feed(n: &mut IqCaptureNode, iq: &[C32], ins: &[PortSpec]) {
        let mut out = Payload::Iq(Vec::new());
        let (mut events, mut tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, ins, &[], &mut events, &mut tags);
        Simple::process(n, &Payload::Iq(iq.to_vec()), &mut out, &mut ctx).unwrap();
    }

    fn tone(n: usize) -> Vec<C32> {
        (0..n)
            .map(|k| {
                let p = std::f32::consts::TAU * 0.05 * k as f32;
                C32::new(p.cos() * 0.5, p.sin() * 0.5)
            })
            .collect()
    }

    #[test]
    fn the_capture_replays_as_what_was_written() {
        // The whole point of the node: what comes off the disk is what went
        // in, at the rate and frequency it was received on, without anybody
        // having to say so.
        let d = dir("roundtrip");
        let rate = 250_000.0;
        let center = Hz(433_920_000);
        let mut n = IqCaptureNode::new(&d).with_name("m17");
        let ins = [spec(rate, center)];
        Node::negotiate(&mut n, &ins).unwrap();
        let iq = tone(4096);
        feed(&mut n, &iq, &ins);
        let path = n.path().unwrap().to_path_buf();
        Simple::reset(&mut n);

        let buf = sources::FileSource::open(&path).unwrap().read_all().unwrap();
        assert_eq!(buf.rate.0, rate as u64);
        assert_eq!(buf.center.0, center.0);
        assert_eq!(buf.samples.len(), iq.len());
        // Eight bits of quantisation, so the samples come back near enough
        // rather than exactly.
        for (a, b) in buf.samples.iter().zip(&iq) {
            assert!((a - b).norm() < 0.02, "{a} against {b}");
        }
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_budget_stops_it_rather_than_the_disk() {
        let d = dir("budget");
        let ins = [spec(250_000.0, Hz(433_920_000))];
        let mut n = IqCaptureNode::new(&d).with_budget(4_000);
        Node::negotiate(&mut n, &ins).unwrap();
        for _ in 0..4 {
            feed(&mut n, &tone(1_000), &ins);
        }
        assert!(n.is_full(), "the budget was not enforced");
        // Two bytes a sample, so the first block of a thousand is 2000 and
        // the second reaches the budget exactly.
        let written: u64 = std::fs::read_dir(&d)
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.metadata().map(|m| m.len()).unwrap_or(0))
            .sum();
        assert_eq!(written, 4_000);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_graph_that_never_ran_leaves_no_file() {
        let d = dir("empty");
        let mut n = IqCaptureNode::new(&d);
        Node::negotiate(&mut n, &[spec(250_000.0, Hz(433_920_000))]).unwrap();
        assert!(n.path().is_none());
        assert!(!d.exists(), "an unused capture made a directory");
    }

    #[test]
    fn retuning_ends_the_file() {
        // The name carries the frequency, so samples from a new one cannot
        // go in the old file.
        let d = dir("retune");
        let ins = [spec(250_000.0, Hz(433_920_000))];
        let mut n = IqCaptureNode::new(&d);
        Node::negotiate(&mut n, &ins).unwrap();
        feed(&mut n, &tone(256), &ins);
        let first = n.path().unwrap().to_path_buf();
        let ins = [spec(250_000.0, Hz(868_300_000))];
        Node::negotiate(&mut n, &ins).unwrap();
        feed(&mut n, &tone(256), &ins);
        let second = n.path().unwrap().to_path_buf();
        assert_ne!(first, second);
        assert!(second.to_string_lossy().contains("868.3000M"));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_stamp_is_not_read_back_as_a_frequency() {
        // 2024-05-01 12:34:56 UTC.
        let s = stamp(1_714_566_896_000_000);
        assert_eq!(s, "20240501-123456");
        let name = format!("capture_{s}_433.4750M_2400k.cu8");
        let meta = sources::parse_filename(Path::new(&name));
        assert_eq!(meta.center, Some(Hz(433_475_000)));
        assert_eq!(meta.rate, Some(common::Sps(2_400_000)));
    }
}
