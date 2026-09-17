//! Replay a recorded span on the air: a capture file as a transmit source.
//!
//! The other half of [`crate::capture_nodes`], which writes the span. What
//! was recorded is already IQ, so there is nothing to modulate: the samples
//! are resampled from the rate they were taken at to the rate the radio is
//! transmitting at and handed straight down the chain. Where the file's
//! centre and the dial disagree, the shift is the mixer's behind this stage,
//! the same as everywhere else, so a capture made at one dial position can
//! go out at another.
//!
//! The file is read here rather than handed in whole the way a `.sub` is,
//! because a span is gigabytes where a pulse list is kilobytes: the stage
//! holds an open file and a block of it at a time.

use common::{C32, Result, SampleFormat};
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::io::{BufReader, Read, Seek, SeekFrom};

const PATH: &str = "path";
const RATE_SPS: &str = "rate_sps";
const LEVEL: &str = "level";
const REPEAT: &str = "repeat";

/// How much of full scale a replayed capture is sent at.
///
/// Below one because a recording is not a modulator's output: its peaks are
/// whatever the receiving radio's gain made of them, and a file recorded into
/// a clipping front end would otherwise be transmitted clipped as well.
const DEFAULT_LEVEL: f32 = 0.8;

/// Complex samples read from disc per refill.
///
/// About 8 ms at 2 MS/s and 800 us at 20, so a block of graph time takes one
/// or two reads of the file rather than one read per sample.
const CHUNK: usize = 16_384;

/// A capture being replayed: the open file and where it is in it.
struct Reading {
    file: BufReader<std::fs::File>,
    format: SampleFormat,
    /// Samples handed to the chain since this pass of the file started, which
    /// is what says how far through it the transmission is. Not what has been
    /// read: a refill runs ahead of the block being filled, and on a short
    /// file it reads the lot before the first sample goes out.
    sent: u64,
    total: u64,
    /// Read but not yet asked for, at the output rate.
    pending: std::collections::VecDeque<C32>,
    resample: Option<dsp::resample::Rational>,
    /// Set once the file has run out and nothing is repeating it.
    finished: bool,
}

/// Replay a capture file into a transmit chain.
pub struct IqTxNode {
    path: String,
    /// The rate the file was recorded at, which the name carries and the
    /// interface reads; zero until one is set, and a stage with none is
    /// silent rather than guessing. A guessed rate rescales every symbol in
    /// the recording and puts a signal of the wrong width on the air.
    file_rate: f64,
    level: f32,
    repeat: bool,
    rate: f64,
    open: Option<Reading>,
    /// Why the file will not play, for the stage to report rather than fail
    /// the whole graph: a transmitter that refuses to build is a receiver
    /// that stops.
    fault: Option<String>,
}

impl Default for IqTxNode {
    fn default() -> Self {
        Self {
            path: String::new(),
            file_rate: 0.0,
            level: DEFAULT_LEVEL,
            repeat: true,
            rate: 0.0,
            open: None,
            fault: None,
        }
    }
}

impl IqTxNode {
    pub fn new(path: &str, file_rate: f64) -> Self {
        Self { path: path.to_string(), file_rate, ..Self::default() }
    }

    /// Whether a file is open and has samples left to send.
    pub fn is_playing(&self) -> bool {
        self.open.as_ref().is_some_and(|r| !r.finished)
    }

    pub fn fault(&self) -> Option<&str> {
        self.fault.as_deref()
    }

    /// How far through the file the replay is, 0 to 1, and 0 with none open.
    pub fn progress(&self) -> f64 {
        let (Some(r), true) = (&self.open, self.rate > 0.0 && self.file_rate > 0.0) else {
            return 0.0;
        };
        if r.total == 0 {
            return 0.0;
        }
        let read = r.sent as f64 * self.file_rate / self.rate;
        ((read % r.total as f64) / r.total as f64).clamp(0.0, 1.0)
    }

    /// Open the file, or record why it cannot be.
    fn load(&mut self) {
        self.open = None;
        self.fault = None;
        if self.path.is_empty() {
            return;
        }
        let path = std::path::Path::new(&self.path);
        let Some(format) =
            path.extension().and_then(|e| e.to_str()).and_then(SampleFormat::from_extension)
        else {
            self.fault = Some(format!(
                "{}: cannot tell its sample format from the name; expected cu8, cs8, cs16 or cf32",
                self.path
            ));
            return;
        };
        if self.file_rate <= 0.0 {
            self.fault = Some(format!("{}: nothing says what rate it was recorded at", self.path));
            return;
        }
        let file = match std::fs::File::open(path) {
            Ok(f) => f,
            Err(e) => {
                self.fault = Some(format!("{}: {e}", self.path));
                return;
            }
        };
        let total =
            file.metadata().map(|m| m.len()).unwrap_or(0) / format.bytes_per_sample() as u64;
        self.open = Some(Reading {
            file: BufReader::with_capacity(1 << 20, file),
            format,
            sent: 0,
            total,
            pending: std::collections::VecDeque::new(),
            resample: None,
            finished: false,
        });
        self.fit_rate();
    }

    /// Put the resampler between the file's rate and the graph's.
    ///
    /// A capture recorded at the rate the radio is transmitting at needs
    /// none, which is the case worth keeping free: every other one costs a
    /// polyphase filter per sample.
    fn fit_rate(&mut self) {
        let (from, to) = (self.file_rate, self.rate);
        let Some(r) = self.open.as_mut() else { return };
        r.resample = match from > 0.0 && to > 0.0 && (from - to).abs() > 1.0 {
            true => {
                tracing::warn!(
                    "iq_tx: recorded at {from} S/s and transmitting at {to} S/s; resampling"
                );
                dsp::resample::Rational::new(from, to, 512)
            }
            false => None,
        };
    }

    /// Fill `pending` until it holds `want` samples at the output rate, or
    /// the file has run out.
    fn refill(&mut self, want: usize) {
        let Some(r) = self.open.as_mut() else { return };
        let bps = r.format.bytes_per_sample();
        let mut raw = vec![0u8; CHUNK * bps];
        let mut read = Vec::with_capacity(CHUNK);
        let mut out = Vec::with_capacity(CHUNK * 2);
        while r.pending.len() < want && !r.finished {
            let mut filled = 0usize;
            while filled < raw.len() {
                match r.file.read(&mut raw[filled..]) {
                    Ok(0) => break,
                    Ok(n) => filled += n,
                    Err(_) => break,
                }
            }
            // A short read at the end is normal; a partial sample means the
            // file is truncated, and keeping the odd bytes would swap I and Q
            // for everything after it.
            let usable = filled - (filled % bps);
            if usable == 0 {
                match self.repeat && r.total > 0 {
                    true => {
                        let _ = r.file.seek(SeekFrom::Start(0));
                        // A join is a discontinuity whatever is done about
                        // it, and the resampler's history is of the end of
                        // the file rather than the start.
                        if let Some(rs) = r.resample.as_mut() {
                            rs.reset();
                        }
                        continue;
                    }
                    false => {
                        r.finished = true;
                        break;
                    }
                }
            }
            read.clear();
            r.format.convert(&raw[..usable], &mut read);
            out.clear();
            match r.resample.as_mut() {
                Some(rs) => rs.process(&read, &mut out),
                None => out.extend_from_slice(&read),
            }
            r.pending.extend(out.iter().copied());
        }
    }
}

impl Simple for IqTxNode {
    fn name(&self) -> &str {
        "iq_tx"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("iq_tx needs the rate it should transmit at"));
        }
        self.rate = input.spec.rate;
        if self.open.is_none() {
            self.load();
        } else {
            self.fit_rate();
        }
        Ok(StreamSpec {
            kind: PortKind::Iq,
            rate: self.rate,
            center: input.spec.center,
            // The whole span: a recording is of everything the receiver could
            // hear, and nothing here narrows it.
            bandwidth: self.rate,
            channels: 1,
            flow: Flow::Tx,
            domain: Domain::Baseband,
        })
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let want = input.len();
        let out = output.iq_mut();
        if want == 0 {
            return Ok(());
        }
        // Nothing open is silence rather than a refusal: the transmit chain
        // is drawn whether or not a file has been chosen, and a stage that
        // failed here would take the receiver's graph with it.
        self.refill(want);
        let level = self.level;
        let Some(r) = self.open.as_mut() else {
            out.resize(want, C32::default());
            return Ok(());
        };
        out.reserve(want);
        for _ in 0..want {
            match r.pending.pop_front() {
                Some(s) => {
                    out.push(s * level);
                    r.sent += 1;
                }
                None => out.push(C32::default()),
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.load();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::text(PATH, self.path.clone()).label("Capture"),
            Param::float(RATE_SPS, self.file_rate, 0.0..=100e6).label("Recorded at").unit("S/s"),
            Param::float(LEVEL, self.level as f64, 0.0..=1.0).label("Level"),
            Param::bool(REPEAT, self.repeat).label("Loop"),
        ]
    }

    fn configure(&mut self, s: &Settings) {
        self.path = s.str_or(PATH, "").to_string();
        self.file_rate = s.f64_or(RATE_SPS, 0.0);
        self.level = s.f64_or(LEVEL, DEFAULT_LEVEL as f64).clamp(0.0, 1.0) as f32;
        self.repeat = s.bool_or(REPEAT, true);
        self.load();
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            // Opening the file here is a read on the thread the graph runs
            // on, which is what the operator asked for by choosing one: the
            // alternative is a stage that says nothing until the next block.
            PATH => {
                self.path = value.as_str().unwrap_or_default().to_string();
                self.load();
            }
            RATE_SPS => {
                self.file_rate = value.as_f64().unwrap_or(0.0).max(0.0);
                self.load();
            }
            LEVEL => self.level = value.as_f64().unwrap_or(0.0).clamp(0.0, 1.0) as f32,
            REPEAT => self.repeat = value.as_bool().unwrap_or(true),
            _ => return Err(common::Error::other(format!("iq_tx: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

pub const IQ_TX: StageDesc = StageDesc {
    name: "iq_tx",
    summary: "Replay a recorded span, ready for a transmitter",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_iq_tx(s: &Settings) -> Result<Box<dyn Node>> {
    let mut n = IqTxNode::default();
    Simple::configure(&mut n, s);
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec { rate, ..Default::default() }, latency: 0 }
    }

    /// A ramp of `n` samples as an 8-bit unsigned capture, named so the
    /// format can be read off it.
    fn capture(name: &str, n: usize) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join("sr_iq_tx");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        let bytes: Vec<u8> = (0..n).flat_map(|i| [(i % 256) as u8, 128u8]).collect();
        std::fs::write(&path, bytes).unwrap();
        path
    }

    fn run(n: &mut IqTxNode, rate: f64, block: usize) -> Vec<C32> {
        let input = Payload::Real(vec![0.0; block]);
        let mut out = Payload::empty_of(PortKind::Iq);
        let mut ev = Vec::new();
        let mut tg = Vec::new();
        let ins = [spec(rate)];
        let ctx = &mut NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(n, &input, &mut out, ctx).unwrap();
        out.as_iq().unwrap().to_vec()
    }

    #[test]
    fn a_capture_at_the_transmit_rate_goes_out_sample_for_sample() {
        let path = capture("even_433.92M_250k.cu8", 1024);
        let mut n = IqTxNode::new(&path.display().to_string(), 250_000.0);
        n.repeat = false;
        let spec = Simple::negotiate(&mut n, &spec(250_000.0)).unwrap();
        assert_eq!(spec.kind, PortKind::Iq);
        assert_eq!(spec.flow, Flow::Tx);
        assert!(n.fault().is_none(), "{:?}", n.fault());

        let out = run(&mut n, 250_000.0, 256);
        assert_eq!(out.len(), 256);
        // The first sample of the file: I = 0 of 255 in offset binary, which
        // is full scale negative, and Q = 128, which is nothing.
        assert!((out[0].re - -1.0 * DEFAULT_LEVEL).abs() < 0.01, "{:?}", out[0]);
        assert!(out[0].im.abs() < 0.01, "{:?}", out[0]);
        // A quarter of a 1024 sample file in one block of 256.
        assert!((n.progress() - 0.25).abs() < 0.05, "played {}", n.progress());
    }

    #[test]
    fn a_capture_at_half_the_rate_comes_out_at_twice_the_length() {
        let path = capture("ramp_433.92M_125k.cu8", 4096);
        let mut n = IqTxNode::new(&path.display().to_string(), 125_000.0);
        n.repeat = false;
        Simple::negotiate(&mut n, &spec(250_000.0)).unwrap();
        let out = run(&mut n, 250_000.0, 1000);
        assert_eq!(out.len(), 1000);
        // 1000 samples out of a resampler running at two for one is 500 of
        // the file's own samples.
        let played = n.progress() * 4096.0;
        assert!((495.0..=505.0).contains(&played), "read {played} samples for 1000 out");
    }

    #[test]
    fn a_file_that_has_run_out_goes_quiet_and_a_looped_one_does_not() {
        let path = capture("loop_433.92M_250k.cu8", 512);
        let mut once = IqTxNode::new(&path.display().to_string(), 250_000.0);
        once.repeat = false;
        Simple::negotiate(&mut once, &spec(250_000.0)).unwrap();
        let out = run(&mut once, 250_000.0, 1024);
        assert_eq!(out.len(), 1024);
        let quiet = out[512..].iter().filter(|s| s.norm() < 1e-6).count();
        assert_eq!(quiet, 512, "the half past the end of the file is silence");
        assert!(!once.is_playing());

        let mut looped = IqTxNode::new(&path.display().to_string(), 250_000.0);
        Simple::negotiate(&mut looped, &spec(250_000.0)).unwrap();
        let out = run(&mut looped, 250_000.0, 1024);
        assert_eq!(out.iter().filter(|s| s.norm() < 1e-6).count(), 0, "a loop never runs out");
        assert!(looped.is_playing());
        // Sample 512 is sample 0 again: the file starts over rather than
        // running on into whatever is after it on disc.
        assert!((out[512].re - out[0].re).abs() < 1e-6);
    }

    #[test]
    fn a_missing_rate_or_an_unknown_extension_is_a_fault_not_a_refusal() {
        let path = capture("mystery.bin", 64);
        let mut n = IqTxNode::new(&path.display().to_string(), 250_000.0);
        Simple::negotiate(&mut n, &spec(250_000.0)).unwrap();
        assert!(n.fault().unwrap().contains("sample format"), "{:?}", n.fault());

        let known = capture("norate_433.92M_250k.cu8", 64);
        let mut n = IqTxNode::new(&known.display().to_string(), 0.0);
        Simple::negotiate(&mut n, &spec(250_000.0)).unwrap();
        assert!(n.fault().unwrap().contains("rate"), "{:?}", n.fault());
        // And it transmits nothing rather than transmitting a guess.
        assert_eq!(run(&mut n, 250_000.0, 128).iter().filter(|s| s.norm() > 0.0).count(), 0);
    }

    #[test]
    fn nothing_chosen_is_silence() {
        let mut n = IqTxNode::default();
        Simple::negotiate(&mut n, &spec(2_400_000.0)).unwrap();
        let out = run(&mut n, 2_400_000.0, 512);
        assert_eq!(out.len(), 512);
        assert_eq!(out.iter().filter(|s| s.norm() > 0.0).count(), 0);
        assert!(n.fault().is_none(), "no file chosen is not a fault");
    }
}
