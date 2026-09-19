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

use common::{C32, Error, Hz, Result, SampleFormat};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::collections::VecDeque;
use std::io::Write;
use std::path::{Path, PathBuf};

/// What opens a file.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Trigger {
    /// The operator's switch: every sample from the moment it goes on, to one
    /// file.
    #[default]
    Switch,
    /// Power in the span: one file per burst, holding the pre-roll before it
    /// and the tail after it.
    Energy,
}

impl Trigger {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "switch" => Some(Self::Switch),
            "energy" => Some(Self::Energy),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Switch => "switch",
            Self::Energy => "energy",
        }
    }
}

/// What the trigger's threshold is measured against.
///
/// Both, because neither answers on its own: a level above the floor survives
/// a gain change and follows a band as it gets busier, and a level in dBFS is
/// the number an operator can reason about when the floor itself is what
/// moved.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Reference {
    #[default]
    Floor,
    Absolute,
}

impl Reference {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "floor" => Some(Self::Floor),
            "absolute" => Some(Self::Absolute),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Floor => "floor",
            Self::Absolute => "absolute",
        }
    }
}

/// How long a frame of the trigger's power measurement is.
///
/// Two milliseconds, because the measurement has to be short against the
/// shortest thing worth catching and long enough to average a modulated
/// carrier: a 20 ms POCSAG codeword covers ten frames, and a frame of 20 ms
/// measured a 5 ms burst 6 dB low in the test below.
const FRAME_MS: f64 = 2.0;

/// Frames per sub-window and sub-windows held by the floor tracker. The
/// product is its memory, so at 2 ms a frame this is eight seconds, which has
/// to stay longer than the longest transmission or an over is learned as
/// noise (see `dsp::detect::NoiseFloor`).
const FLOOR_SUB_LEN: usize = 32;
const FLOOR_SUB_COUNT: usize = 128;

/// How much signal the pre-roll may hold, whatever is asked for. Five seconds
/// of a 2.4 MS/s span is 96 MB of `C32` in memory, which is already more than
/// a trigger needs to keep the head of a burst.
const MAX_PRE_MS: f64 = 5_000.0;

/// How much of the disk the capture folder may take.
///
/// The limit used to be on one file, which answered the wrong question: what
/// fills a disk is not one capture but the twenty left behind from last
/// week. Four gigabytes is a quarter of an hour of a 2.4 MS/s span as bytes,
/// spread over as many captures as somebody made, and a capture that would
/// take the folder past it does not start.
///
/// Nothing here deletes a capture. A recording is evidence of a signal that
/// may not come again, and a receiver that quietly threw last night's away to
/// make room for tonight's would be worse than one that stops.
pub const DEFAULT_BUDGET: u64 = 4 << 30;

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
    /// What the whole folder may reach.
    budget: u64,
    /// What the captures already in it come to, measured when a file is
    /// opened rather than per block.
    older: u64,
    /// The size of the file being written, kept beside the sink so a capture
    /// that stopped at the limit can still say how large it got.
    last: u64,
    /// When the folder was last added up, so a reader is not charged a
    /// directory listing per block.
    measured: Option<std::time::Instant>,
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
    trigger: Trigger,
    reference: Reference,
    /// dB above the tracked floor, or dBFS, depending on the reference.
    threshold_db: f32,
    /// The band the trigger measures: how wide, and how far from the middle
    /// of the span. Zero width is the whole span.
    band_hz: f64,
    band_offset_hz: f64,
    /// The transform behind a band measurement, built for the frame length in
    /// force. Absent while the whole span is measured, which costs nothing.
    band: Option<dsp::detect::BandPower>,
    pre_ms: f64,
    hang_ms: f64,
    /// The trigger's own floor tracker, fed a frame's power at a time.
    /// Separate from the detector's: it follows whatever band the trigger was
    /// set to, which is not a channel the detector opened.
    floor: dsp::detect::NoiseFloor,
    /// Samples held back so the head of a burst is in its file.
    pre_roll: VecDeque<C32>,
    /// Samples of a frame not yet complete, so the measurement is always over
    /// the same length whatever the block size is.
    pending: Vec<C32>,
    /// Samples since the last frame above the threshold, against the hang.
    quiet: u64,
    /// The last frame's power and the floor under it, both dBFS, for the card.
    level_db: f32,
    floor_db: f32,
    /// Files opened by the trigger since the graph was built.
    bursts: u64,
}

impl IqCaptureNode {
    pub fn new(dir: impl AsRef<Path>) -> Self {
        Self {
            dir: dir.as_ref().to_path_buf(),
            name: "capture".into(),
            format: SampleFormat::Cu8,
            budget: DEFAULT_BUDGET,
            older: 0,
            last: 0,
            measured: None,
            enabled: true,
            rate: 0.0,
            center: Hz(0),
            sink: None,
            full: false,
            error: None,
            reported: false,
            buf: Vec::new(),
            trigger: Trigger::Switch,
            reference: Reference::Floor,
            threshold_db: 10.0,
            band_hz: 0.0,
            band_offset_hz: 0.0,
            band: None,
            pre_ms: 500.0,
            hang_ms: 1_000.0,
            floor: dsp::detect::NoiseFloor::new(FLOOR_SUB_LEN, FLOOR_SUB_COUNT),
            pre_roll: VecDeque::new(),
            pending: Vec::new(),
            quiet: 0,
            level_db: f32::NEG_INFINITY,
            floor_db: f32::NEG_INFINITY,
            bursts: 0,
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

    /// How large the capture folder may get before writing stops, in bytes.
    pub fn with_budget(mut self, bytes: u64) -> Self {
        self.budget = bytes;
        self
    }

    pub fn with_enabled(mut self, on: bool) -> Self {
        self.enabled = on;
        self
    }

    /// What opens a file, and on what terms when that is energy.
    pub fn with_trigger(mut self, t: Trigger) -> Self {
        self.trigger = t;
        self
    }

    pub fn with_threshold(mut self, reference: Reference, db: f32) -> Self {
        self.reference = reference;
        self.threshold_db = db;
        self
    }

    /// Measure `width_hz` centred `offset_hz` from the middle of the span
    /// rather than the span itself. Zero width is the span.
    pub fn with_band(mut self, width_hz: f64, offset_hz: f64) -> Self {
        self.set_band(width_hz, offset_hz);
        self
    }

    fn set_band(&mut self, width_hz: f64, offset_hz: f64) {
        if width_hz == self.band_hz && offset_hz == self.band_offset_hz {
            return;
        }
        self.band_hz = width_hz.max(0.0);
        self.band_offset_hz = offset_hz;
        // The floor was learned from a different measurement, so it says
        // nothing about this one.
        self.floor.reset();
        self.band = None;
    }

    /// The band the trigger measures, in Hz, and how far it sits from the
    /// middle of the span. Zero width is the whole span.
    pub fn band(&self) -> (f64, f64) {
        (self.band_hz, self.band_offset_hz)
    }

    /// How much of the signal before the trigger goes in the file, and how
    /// long the power may stay under the threshold before it is closed.
    pub fn with_window(mut self, pre_ms: f64, hang_ms: f64) -> Self {
        self.pre_ms = pre_ms.clamp(0.0, MAX_PRE_MS);
        self.hang_ms = hang_ms.max(0.0);
        self
    }

    pub fn trigger(&self) -> Trigger {
        self.trigger
    }

    /// Waiting for a signal: armed, switched on, and not writing.
    pub fn is_armed(&self) -> bool {
        self.trigger == Trigger::Energy && self.enabled && !self.full && self.sink.is_none()
    }

    /// Writing a file right now, however it was started.
    pub fn is_recording(&self) -> bool {
        self.sink.is_some()
    }

    /// Files the trigger has opened, which is what says whether it is set too
    /// low: a threshold under the floor makes one long file, and one over
    /// every signal makes none.
    pub fn bursts(&self) -> u64 {
        self.bursts
    }

    /// The last frame's power in dBFS, and the floor under it.
    pub fn level_db(&self) -> f32 {
        self.level_db
    }

    pub fn floor_db(&self) -> f32 {
        self.floor_db
    }

    /// What the threshold comes to right now in dBFS, which is the number the
    /// card shows: a level above the floor means nothing until the floor is
    /// known, so this is `None` until the tracker has seen its whole window.
    pub fn threshold_dbfs(&self) -> Option<f32> {
        match self.reference {
            Reference::Absolute => Some(self.threshold_db),
            Reference::Floor => self.floor.is_ready().then_some(self.floor_db + self.threshold_db),
        }
    }

    /// The file being written, once there is one.
    pub fn path(&self) -> Option<&Path> {
        self.sink.as_ref().map(|s| s.path.as_path())
    }

    pub fn bytes(&self) -> u64 {
        self.last
    }

    /// What the folder holds: this capture and every one kept beside it.
    pub fn folder_bytes(&self) -> u64 {
        self.older + self.last
    }

    /// Add the folder up again, at most this often.
    ///
    /// Called from whatever publishes the status rather than from `process`:
    /// a directory listing per block is a syscall every few milliseconds, and
    /// this number only has to be right for somebody reading it. While a file
    /// is open the total is already exact, since the folder was measured when
    /// it was created and the writing is counted.
    pub fn refresh_folder(&mut self) {
        const EVERY: std::time::Duration = std::time::Duration::from_secs(2);
        if self.sink.is_some() || self.measured.is_some_and(|t| t.elapsed() < EVERY) {
            return;
        }
        self.older = self.measure();
        self.last = 0;
        self.measured = Some(std::time::Instant::now());
    }

    /// Add up the captures already on the disk. Every file is counted, not
    /// only the ones this node named: what the setting promises is a limit on
    /// the folder, and a `.cs16` from last month takes the same disk.
    fn measure(&self) -> u64 {
        let Ok(entries) = std::fs::read_dir(&self.dir) else {
            return 0;
        };
        entries
            .flatten()
            .filter(|e| e.path().is_file())
            .filter_map(|e| e.metadata().ok())
            .map(|m| m.len())
            .sum()
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
        self.rearm();
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
        let name = |stamp: &str| {
            format!(
                "{}_{stamp}_{:.4}M_{:.0}k.{}",
                self.name,
                self.center.as_f64() / 1e6,
                self.rate / 1e3,
                self.format.extension(),
            )
        };
        let at = stamp(at_us);
        let mut path = self.dir.join(name(&at));
        // An armed capture writes a file per burst and two bursts can fall in
        // the same millisecond, which named them the same thing and lost the
        // first. The counter goes inside the time token, where
        // `sources::parse_filename` will not read it as a rate.
        for n in 2.. {
            if !path.exists() {
                break;
            }
            path = self.dir.join(name(&format!("{at}-{n}")));
        }
        self.older = self.measure();
        self.measured = Some(std::time::Instant::now());
        if self.budget > 0 && self.older >= self.budget {
            self.full = true;
            return Err(Error::other(format!(
                "{} already holds {:.1} GB, which is the limit",
                self.dir.display(),
                self.older as f64 / (1u64 << 30) as f64
            )));
        }
        let file = std::fs::File::create(&path)?;
        self.last = 0;
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

    /// Samples in one measured frame at the rate in force.
    fn frame_len(&self) -> usize {
        ((self.rate * FRAME_MS / 1e3) as usize).max(64)
    }

    fn pre_samples(&self) -> usize {
        (self.rate * self.pre_ms.clamp(0.0, MAX_PRE_MS) / 1e3) as usize
    }

    fn hang_samples(&self) -> u64 {
        (self.rate * self.hang_ms.max(0.0) / 1e3) as u64
    }

    /// One measured frame of the span: what it came to, whether that is a
    /// signal, and where the samples go.
    fn armed_frame(&mut self, iq: &[C32], c: &mut NodeCtx<'_>) {
        let power = self.frame_power(iq);
        let floor = self.floor.update(power.max(f32::MIN_POSITIVE));
        self.level_db = 10.0 * power.max(1e-20).log10();
        self.floor_db = 10.0 * floor.max(1e-20).log10();
        let over = match self.reference {
            Reference::Absolute => self.level_db >= self.threshold_db,
            // Nothing triggers until the tracker has seen its whole window:
            // the first estimate rests on a handful of frames and sits far
            // too low, so an armed capture would open a file every time a
            // stream started.
            Reference::Floor => {
                self.floor.is_ready() && self.level_db - self.floor_db >= self.threshold_db
            }
        };
        if over {
            self.quiet = 0;
        } else {
            self.quiet += iq.len() as u64;
        }
        if self.sink.is_none() {
            if !over {
                let keep = self.pre_samples();
                self.pre_roll.extend(iq.iter().copied());
                while self.pre_roll.len() > keep {
                    self.pre_roll.pop_front();
                }
                return;
            }
            if let Err(e) = self.open(now_us()) {
                self.fail(format!("cannot open a capture in {}: {e}", self.dir.display()), c);
                return;
            }
            self.bursts += 1;
            let roll: Vec<C32> = self.pre_roll.drain(..).collect();
            if !roll.is_empty() {
                self.write_or_fail(&roll, c);
            }
        }
        self.write_or_fail(iq, c);
        // The hang is written rather than trimmed: the tail of a burst is
        // where a decoder's trailing bits are, and a file that stops at the
        // last loud frame has lost them.
        if !over && self.quiet >= self.hang_samples() {
            self.close();
            self.pre_roll.clear();
        }
    }

    /// What a frame came to, over the band the trigger was set to or over the
    /// whole span when it was not.
    ///
    /// The transform costs a 2 ms frame's worth of work per 2 ms of signal,
    /// which at 2.4 MS/s is 500 transforms of 4,800 points a second, and only
    /// when a band is set on an armed capture.
    fn frame_power(&mut self, iq: &[C32]) -> f32 {
        if self.band_hz <= 0.0 || self.rate <= 0.0 {
            return iq.iter().map(|s| s.norm_sqr()).sum::<f32>() / iq.len() as f32;
        }
        let band = match &mut self.band {
            Some(b) if b.len() == iq.len() => b,
            _ => self.band.insert(dsp::detect::BandPower::new(iq.len())),
        };
        band.measure(iq, self.rate, self.band_offset_hz, self.band_hz)
    }

    fn write_or_fail(&mut self, iq: &[C32], c: &mut NodeCtx<'_>) {
        if let Err(e) = self.write(iq) {
            let path = self.path().map(|p| p.display().to_string()).unwrap_or_default();
            self.fail(format!("cannot write {path}: {e}"), c);
            self.close();
        }
    }

    fn write(&mut self, iq: &[C32]) -> Result<()> {
        self.buf.clear();
        self.format.encode(iq, &mut self.buf);
        let Some(s) = &mut self.sink else {
            return Ok(());
        };
        s.file.write_all(&self.buf)?;
        s.bytes += self.buf.len() as u64;
        s.samples += iq.len() as u64;
        self.last = s.bytes;
        if self.budget > 0 && self.older + s.bytes >= self.budget {
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
            // A new rate is a new frame length, so the tracked floor and the
            // pre-roll are measurements of something else.
            self.rearm();
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
        if self.trigger == Trigger::Energy {
            // Measured a frame at a time whatever the block size is, so the
            // threshold means the same thing on every source. Samples that do
            // not fill a frame wait for the next block rather than being
            // measured over a shorter window, and stay in order behind it.
            let mut pending = std::mem::take(&mut self.pending);
            pending.extend_from_slice(iq);
            let n = self.frame_len();
            let mut at = 0;
            while at + n <= pending.len() && !self.full {
                self.armed_frame(&pending[at..at + n], c);
                at += n;
            }
            pending.drain(..at);
            self.pending = pending;
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
        self.write_or_fail(iq, c);
        Ok(())
    }

    fn reset(&mut self) {
        self.close();
        self.rearm();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool(ENABLED, self.enabled).label("Write the span to disk"),
            Param::float(BUDGET_MB, self.budget as f64 / (1 << 20) as f64, 16.0..=65_536.0)
                .unit("MB")
                .label("Stop when the folder reaches")
                .log(),
            Param::choice(
                TRIGGER,
                match self.trigger {
                    Trigger::Switch => 0,
                    Trigger::Energy => 1,
                },
                vec!["switch".into(), "energy".into()],
            )
            .label("Start a file on"),
            Param::choice(
                REFERENCE,
                match self.reference {
                    Reference::Floor => 0,
                    Reference::Absolute => 1,
                },
                vec!["floor".into(), "absolute".into()],
            )
            .label("Threshold is"),
            Param::float(THRESHOLD_DB, self.threshold_db as f64, -120.0..=60.0)
                .unit("dB")
                .label("Trigger at"),
            Param::float(BAND_HZ, self.band_hz, 0.0..=100e6).unit("Hz").label("Measure a band of"),
            Param::float(BAND_OFFSET_HZ, self.band_offset_hz, -50e6..=50e6)
                .unit("Hz")
                .label("Band sits from centre"),
            Param::float(PRE_MS, self.pre_ms, 0.0..=MAX_PRE_MS).unit("ms").label("Keep before"),
            Param::float(HANG_MS, self.hang_ms, 0.0..=30_000.0).unit("ms").label("Hold after"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            ENABLED => {
                self.set_enabled(v.as_bool().unwrap_or(true));
                Ok(())
            }
            BUDGET_MB => {
                self.budget = (v.as_f64().unwrap_or(0.0).max(0.0) * (1 << 20) as f64) as u64;
                Ok(())
            }
            TRIGGER => {
                let was = self.trigger;
                self.trigger = match &v {
                    ParamValue::Int(0) => Trigger::Switch,
                    ParamValue::Int(_) => Trigger::Energy,
                    _ => v.as_str().and_then(Trigger::parse).unwrap_or(self.trigger),
                };
                if self.trigger != was {
                    // The file open under one trigger was started on terms
                    // the other does not hold, so it is finished here rather
                    // than grown by a rule it was not opened under.
                    self.close();
                    self.rearm();
                }
                Ok(())
            }
            REFERENCE => {
                self.reference = match &v {
                    ParamValue::Int(0) => Reference::Floor,
                    ParamValue::Int(_) => Reference::Absolute,
                    _ => v.as_str().and_then(Reference::parse).unwrap_or(self.reference),
                };
                Ok(())
            }
            THRESHOLD_DB => {
                self.threshold_db = v.as_f64().unwrap_or(self.threshold_db as f64) as f32;
                Ok(())
            }
            BAND_HZ => {
                self.set_band(v.as_f64().unwrap_or(self.band_hz).max(0.0), self.band_offset_hz);
                Ok(())
            }
            BAND_OFFSET_HZ => {
                self.set_band(self.band_hz, v.as_f64().unwrap_or(self.band_offset_hz));
                Ok(())
            }
            PRE_MS => {
                self.pre_ms = v.as_f64().unwrap_or(self.pre_ms).clamp(0.0, MAX_PRE_MS);
                Ok(())
            }
            HANG_MS => {
                self.hang_ms = v.as_f64().unwrap_or(self.hang_ms).max(0.0);
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
            c.warn(message.clone());
        }
        self.error = Some(message);
        self.full = true;
    }

    /// Forget what the trigger measured, without touching what is on disk.
    fn rearm(&mut self) {
        self.floor.reset();
        self.band = None;
        self.pre_roll.clear();
        self.pending.clear();
        self.quiet = 0;
        self.level_db = f32::NEG_INFINITY;
        self.floor_db = f32::NEG_INFINITY;
    }
}

/// UTC as `YYYYmmdd-HHMMSS-mmm`.
///
/// The hyphen is not decoration: `sources::parse_filename` reads the
/// frequency and rate out of a name by looking for numeric tokens, and a bare
/// run of digits is a perfectly good number. One was read as a sample rate of
/// 1 Hz once already, in the burst recorder.
///
/// The milliseconds are not decoration either: an armed capture writes a file
/// per burst, and two bursts in the same second gave the same name, so the
/// second one silently replaced the first.
fn stamp(at_us: u64) -> String {
    let secs = (at_us / 1_000_000) as i64;
    let nanos = (at_us % 1_000_000) as u32 * 1_000;
    chrono::DateTime::from_timestamp(secs, nanos)
        .unwrap_or(chrono::DateTime::UNIX_EPOCH)
        .format("%Y%m%d-%H%M%S-%3f")
        .to_string()
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// Anything that would confuse the name back into metadata, or a shell.
fn sanitise(s: &str) -> String {
    let s: String = s.chars().filter(|c| c.is_ascii_alphanumeric() || *c == '-').collect();
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

    /// The limit is on the folder, so what is already in it counts. A limit
    /// per file let a night of pressing the button fill a disk one gigabyte
    /// at a time, with every file inside its budget.
    #[test]
    fn captures_already_on_the_disk_count_against_the_budget() {
        let d = dir("folder-budget");
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join("last-week_433.9200M_250k.cu8"), vec![0u8; 3_000]).unwrap();
        let ins = [spec(250_000.0, Hz(433_920_000))];
        let mut n = IqCaptureNode::new(&d).with_budget(4_000);
        Node::negotiate(&mut n, &ins).unwrap();
        for _ in 0..4 {
            feed(&mut n, &tone(1_000), &ins);
        }
        assert!(n.is_full(), "the folder went past its limit");
        assert!(n.folder_bytes() >= 4_000, "{} bytes", n.folder_bytes());
        assert!(
            d.join("last-week_433.9200M_250k.cu8").exists(),
            "a capture was deleted to make room, which loses evidence"
        );
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

    /// Noise at about -40 dBFS a sample, which is what the floor tracker has
    /// to settle on before anything can trigger.
    fn noise(n: usize, seed: &mut u32) -> Vec<C32> {
        (0..n)
            .map(|_| {
                let mut r = || {
                    *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (*seed >> 8) as f32 / (1 << 23) as f32 - 1.0
                };
                C32::new(r() * 0.01, r() * 0.01)
            })
            .collect()
    }

    /// Enough noise for `dsp::detect::NoiseFloor` to have its whole window,
    /// which at 2 ms a frame is 8.2 s: 32 frames a sub-window, 128 of them.
    fn settle(n: &mut IqCaptureNode, rate: f64, ins: &[PortSpec], seed: &mut u32) {
        let frames = FLOOR_SUB_LEN * FLOOR_SUB_COUNT + 1;
        let block = (rate * FRAME_MS / 1e3) as usize;
        for _ in 0..frames {
            feed(n, &noise(block, seed), ins);
        }
    }

    fn armed(dir: &Path, rate: f64) -> IqCaptureNode {
        IqCaptureNode::new(dir)
            .with_trigger(Trigger::Energy)
            .with_threshold(Reference::Floor, 10.0)
            .with_window(100.0, 40.0)
            .with_budget(1 << 30)
            .with_name(&format!("armed{}", rate as u64))
    }

    fn files(d: &Path) -> Vec<PathBuf> {
        let mut v: Vec<_> = std::fs::read_dir(d)
            .map(|r| r.flatten().map(|e| e.path()).collect())
            .unwrap_or_default();
        v.sort();
        v
    }

    /// A burst either side of a quiet gap is two files, each holding the
    /// signal and the pre-roll in front of it and nothing like the whole run.
    #[test]
    fn a_burst_makes_a_file_and_the_silence_between_makes_none() {
        let d = dir("armed-bursts");
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let mut n = armed(&d, rate);
        Node::negotiate(&mut n, &ins).unwrap();
        let mut seed = 1;
        settle(&mut n, rate, &ins, &mut seed);
        assert!(n.is_armed(), "nothing was written by the noise");
        assert_eq!(files(&d).len(), 0);

        // Two 20 ms tones, 200 ms of noise apart. The hang is 40 ms, so the
        // gap closes the first file well before the second tone arrives.
        let burst = (rate * 0.020) as usize;
        for pass in 0..2 {
            feed(&mut n, &tone(burst), &ins);
            assert!(n.is_recording(), "the tone did not open a file on pass {pass}");
            for _ in 0..10 {
                feed(&mut n, &noise((rate * 0.020) as usize, &mut seed), &ins);
            }
            assert!(!n.is_recording(), "the file stayed open through the silence");
        }
        assert_eq!(n.bursts(), 2, "one file per burst");
        let files = files(&d);
        assert_eq!(files.len(), 2);
        for f in &files {
            let buf = sources::FileSource::open(f).unwrap().read_all().unwrap();
            assert_eq!(buf.rate.0, rate as u64);
            // The burst, the 100 ms of pre-roll in front of it and the 40 ms
            // hang behind it, to a frame: 16,000 samples, against the 84,000
            // the whole run put through the node.
            assert!(
                (14_000..18_000).contains(&buf.samples.len()),
                "{} holds {} samples",
                f.display(),
                buf.samples.len()
            );
        }
        let _ = std::fs::remove_dir_all(&d);
    }

    /// The point of the pre-roll: the first sample of the burst is not the
    /// first sample of the file, so the head of a frame is not lost to the
    /// time the trigger took to decide.
    #[test]
    fn the_file_starts_before_the_signal_did() {
        let d = dir("armed-preroll");
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let mut n = armed(&d, rate).with_window(100.0, 40.0);
        Node::negotiate(&mut n, &ins).unwrap();
        let mut seed = 7;
        settle(&mut n, rate, &ins, &mut seed);
        feed(&mut n, &tone((rate * 0.020) as usize), &ins);
        Simple::reset(&mut n);
        let f = files(&d).remove(0);
        let buf = sources::FileSource::open(&f).unwrap().read_all().unwrap();
        // 100 ms of pre-roll at 100 kS/s, held to the frame the trigger
        // measures in: 10,000 samples, and never more than one frame short.
        let pre = (rate * 0.100) as usize;
        assert!(buf.samples.len() > pre, "{} samples, pre-roll {pre}", buf.samples.len());
        let head = &buf.samples[..pre - 200];
        let loud = head.iter().filter(|s| s.norm_sqr() > 0.01).count();
        assert_eq!(loud, 0, "the pre-roll is not noise, so it is not the pre-roll");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// Minutes of noise and nothing else. The threshold is relative, so a
    /// receiver left armed on an empty channel must write nothing at all
    /// rather than one file the size of the night.
    #[test]
    fn noise_alone_writes_nothing() {
        let d = dir("armed-noise");
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let mut n = armed(&d, rate);
        Node::negotiate(&mut n, &ins).unwrap();
        let mut seed = 99;
        // Five minutes at 100 kS/s.
        for _ in 0..1_500 {
            feed(&mut n, &noise((rate * 0.200) as usize, &mut seed), &ins);
        }
        assert_eq!(n.bursts(), 0);
        assert_eq!(files(&d).len(), 0, "noise triggered the capture");
        assert!(n.is_armed());
        let _ = std::fs::remove_dir_all(&d);
    }

    /// An absolute threshold does not wait for the floor, and says what it
    /// resolved to straight away. The tone is 0.5 in each component, so its
    /// power is 0.25 and its level -6.02 dBFS: -10 dBFS triggers on it and
    /// 0 dBFS does not.
    #[test]
    fn an_absolute_threshold_triggers_without_a_learned_floor() {
        let d = dir("armed-absolute");
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let mut n = armed(&d, rate).with_threshold(Reference::Absolute, -10.0);
        Node::negotiate(&mut n, &ins).unwrap();
        assert_eq!(n.threshold_dbfs(), Some(-10.0));
        feed(&mut n, &tone((rate * 0.010) as usize), &ins);
        assert!(n.is_recording());
        assert_eq!(n.bursts(), 1);
        assert!((n.level_db() - -6.02).abs() < 0.1, "{} dBFS", n.level_db());

        let mut seed = 3;
        let mut over = armed(&d, rate).with_threshold(Reference::Absolute, 0.0);
        Node::negotiate(&mut over, &ins).unwrap();
        feed(&mut over, &tone((rate * 0.010) as usize), &ins);
        feed(&mut over, &noise((rate * 0.010) as usize, &mut seed), &ins);
        assert_eq!(over.bursts(), 0, "a threshold above the signal still triggered");
        Simple::reset(&mut n);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// The budget is the budget however the file was started, and a full
    /// folder stops an armed capture rather than letting the trigger open
    /// another file.
    #[test]
    fn the_budget_still_stops_an_armed_capture() {
        let d = dir("armed-budget");
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let mut n = armed(&d, rate).with_budget(4_000).with_threshold(Reference::Absolute, -10.0);
        Node::negotiate(&mut n, &ins).unwrap();
        for _ in 0..4 {
            feed(&mut n, &tone(2_000), &ins);
        }
        assert!(n.is_full());
        assert!(!n.is_armed(), "a full capture still reports itself as waiting");
        let written: u64 =
            files(&d).iter().filter_map(|p| p.metadata().ok()).map(|m| m.len()).sum();
        assert_eq!(written, 4_000);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// A block is not a frame: the same signal must trigger the same way
    /// whether the source delivers it in one block or in twenty.
    #[test]
    fn the_block_size_does_not_change_what_triggers() {
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let burst = (rate * 0.020) as usize;
        let mut written = Vec::new();
        for (name, block) in [("armed-block-one", burst), ("armed-block-many", 137)] {
            let d = dir(name);
            let mut n = armed(&d, rate).with_threshold(Reference::Absolute, -10.0);
            Node::negotiate(&mut n, &ins).unwrap();
            let iq = tone(burst);
            for chunk in iq.chunks(block) {
                feed(&mut n, chunk, &ins);
            }
            // Silence long enough for the hang to close the file.
            let mut seed = 5;
            for _ in 0..10 {
                feed(&mut n, &noise((rate * 0.020) as usize, &mut seed), &ins);
            }
            assert_eq!(n.bursts(), 1, "{name}");
            let f = files(&d).remove(0);
            written.push(sources::FileSource::open(&f).unwrap().read_all().unwrap().samples.len());
            let _ = std::fs::remove_dir_all(&d);
        }
        // Within one frame of each other: a block that does not divide into
        // frames leaves its remainder for the next block.
        let frame = (rate * FRAME_MS / 1e3) as usize;
        assert!(
            written[0].abs_diff(written[1]) <= frame,
            "{} against {} samples",
            written[0],
            written[1]
        );
    }

    /// An NFM transmission 20 dB over the noise in its own channel, sitting
    /// 300 kHz up a 2.4 MS/s span.
    fn nfm_burst(n: usize, rate: f64, offset_hz: f64, noise_power: f32) -> Vec<C32> {
        // 12.5 kHz of a 2.4 MHz span carries 100 * (12.5 / 2400) = 0.52 times
        // the span's noise power, so the whole span rises 1.8 dB.
        let amp = (100.0 * noise_power * NARROW_HZ as f32 / rate as f32).sqrt();
        let mut phase = 0.0f32;
        (0..n)
            .map(|k| {
                // 2.5 kHz deviation at 1 kHz, which fills about 7 kHz.
                let m = (std::f32::consts::TAU * 1_000.0 / rate as f32 * k as f32).sin();
                phase += std::f32::consts::TAU * (offset_hz as f32 + 2_500.0 * m) / rate as f32;
                C32::new(phase.cos(), phase.sin()) * amp
            })
            .collect()
    }

    const NARROW_HZ: f64 = 12_500.0;

    /// The whole point of measuring a band: a narrow signal on a wide span
    /// lifts the span by 1.8 dB and its own channel by 20, so the trigger has
    /// to be looking at the channel or it never fires.
    #[test]
    fn a_narrow_burst_trips_a_band_and_not_the_span() {
        let rate = 2_400_000.0;
        let offset = 300_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let frame = (rate * FRAME_MS / 1e3) as usize;
        let mut opened = Vec::new();
        for (name, width) in [("armed-span", 0.0), ("armed-band", NARROW_HZ)] {
            let d = dir(name);
            let mut n = armed(&d, rate).with_band(width, offset).with_name(name);
            Node::negotiate(&mut n, &ins).unwrap();
            let mut seed = 2_024;
            settle(&mut n, rate, &ins, &mut seed);
            assert!(n.is_armed(), "{name}: the noise alone opened a file");
            let noise_power = {
                let b = noise(frame, &mut seed.clone());
                b.iter().map(|s| s.norm_sqr()).sum::<f32>() / frame as f32
            };
            // 100 ms of it, which is a short over.
            for _ in 0..50 {
                let bg = noise(frame, &mut seed);
                let sig = nfm_burst(frame, rate, offset, noise_power);
                let block: Vec<C32> = bg.iter().zip(&sig).map(|(a, b)| a + b).collect();
                feed(&mut n, &block, &ins);
            }
            opened.push((n.bursts(), files(&d).len()));
            let _ = std::fs::remove_dir_all(&d);
        }
        assert_eq!(opened[0], (0, 0), "the span measurement caught a 12.5 kHz signal");
        assert_eq!(opened[1], (1, 1), "the band measurement missed the transmission");
    }

    /// A band as wide as the span is the span, so the threshold an operator
    /// set before there were bands still means what it did.
    #[test]
    fn a_band_wider_than_the_span_triggers_like_the_span() {
        let d = dir("armed-wide-band");
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let mut n = armed(&d, rate).with_band(rate, 0.0).with_threshold(Reference::Absolute, -10.0);
        Node::negotiate(&mut n, &ins).unwrap();
        feed(&mut n, &tone((rate * 0.010) as usize), &ins);
        assert_eq!(n.bursts(), 1);
        // The same -6.02 dBFS the span measurement reads for this tone.
        assert!((n.level_db() - -6.02).abs() < 0.3, "{} dBFS", n.level_db());
        Simple::reset(&mut n);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// Minutes of noise with a band set, and nothing written: a narrower
    /// measurement is a lower floor, not a trigger that fires on its own.
    #[test]
    fn noise_in_a_band_writes_nothing() {
        let d = dir("armed-band-noise");
        let rate = 100_000.0;
        let ins = [spec(rate, Hz(433_920_000))];
        let mut n = armed(&d, rate).with_band(12_500.0, 20_000.0);
        Node::negotiate(&mut n, &ins).unwrap();
        let mut seed = 41;
        // Five minutes at 100 kS/s.
        for _ in 0..1_500 {
            feed(&mut n, &noise((rate * 0.200) as usize, &mut seed), &ins);
        }
        assert_eq!(n.bursts(), 0);
        assert_eq!(files(&d).len(), 0, "noise in a band triggered the capture");
        assert!(n.is_armed());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_stamp_is_not_read_back_as_a_frequency() {
        // 2024-05-01 12:34:56 UTC.
        let s = stamp(1_714_566_896_250_000);
        assert_eq!(s, "20240501-123456-250");
        let name = format!("capture_{s}_433.4750M_2400k.cu8");
        let meta = sources::parse_filename(Path::new(&name));
        assert_eq!(meta.center, Some(Hz(433_475_000)));
        assert_eq!(meta.rate, Some(common::Sps(2_400_000)));
    }
}

/// The setting names this stage reads.
const DIR: &str = "dir";
const NAME: &str = "name";
const FORMAT: &str = "format";
const BUDGET_MB: &str = "budget_mb";
const ENABLED: &str = "enabled";
const TRIGGER: &str = "trigger";
const REFERENCE: &str = "reference";
const THRESHOLD_DB: &str = "threshold_db";
pub const BAND_HZ: &str = "band_hz";
pub const BAND_OFFSET_HZ: &str = "band_offset_hz";
const PRE_MS: &str = "pre_ms";
const HANG_MS: &str = "hang_ms";

pub const DESC: StageDesc = StageDesc {
    name: "iq_capture",
    summary: "Write the span to a file as it arrives, or a file per burst \
              when armed on energy, so a signal nothing decodes can be \
              worked on off the air",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let default = SampleFormat::Cu8;
    let format =
        SampleFormat::from_extension(s.str_or(FORMAT, default.extension())).unwrap_or(default);
    let mb = s.f64_or(BUDGET_MB, 0.0);
    let budget = if mb > 0.0 { (mb * (1u64 << 20) as f64) as u64 } else { DEFAULT_BUDGET };
    let trigger = Trigger::parse(s.str_or(TRIGGER, "switch")).unwrap_or_default();
    let reference = Reference::parse(s.str_or(REFERENCE, "floor")).unwrap_or_default();
    Ok(Box::new(
        IqCaptureNode::new(s.str_or(DIR, "."))
            .with_name(s.str_or(NAME, "capture"))
            .with_format(format)
            .with_budget(budget)
            .with_enabled(s.bool_or(ENABLED, true))
            .with_trigger(trigger)
            .with_threshold(reference, s.f64_or(THRESHOLD_DB, 10.0) as f32)
            .with_band(s.f64_or(BAND_HZ, 0.0), s.f64_or(BAND_OFFSET_HZ, 0.0))
            .with_window(s.f64_or(PRE_MS, 500.0), s.f64_or(HANG_MS, 1_000.0)),
    ))
}
