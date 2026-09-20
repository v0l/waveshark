//! Sinks: nodes that consume a stream and produce no stream.
//!
//! A spectrum display and a recorder are not stages in a chain, they are
//! places a stream ends. That makes them look like a poor fit for a graph,
//! and they are not: what they consume is IQ from the same source as
//! everything else, they need the same negotiated rate and centre frequency
//! to interpret it, and if they sit outside the graph then the graph is no
//! longer a description of what the receiver is doing.
//!
//! What they produce is read back by downcasting rather than through events.
//! An event per block carrying a spectrum frame would clone a few thousand
//! floats for a display that only ever draws the most recent one, and the
//! recorder's ring is far larger than that. Events are for things that
//! happened; these are things that are.

use common::{C32, Error, Result};
use dsp::Spectrum;
use dsp::spectrum::{self, Detector};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// How the converter is being driven, measured on the samples of the last
/// spectrum frame.
///
/// Two failures look like a quiet band from the display and are nothing of
/// the kind. Too little gain leaves the noise under half a step of the
/// converter and the samples take only the two or three values around zero:
/// there is no floor to detect against, and the first real signal drops the
/// quantisation noise everywhere else by ten decibels, so the detector
/// opens on the whole band. Too much gain puts the samples on the rails and
/// every level and shape is a lie. Both are the operator's to fix and both
/// are invisible until something says so.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AdcHealth {
    /// Distinct values the in-phase samples took, capped at [`Self::LEVELS_ENOUGH`].
    pub levels: u16,
    /// Share of samples at either rail, 0 to 1.
    pub clipped: f32,
    /// Peak sample magnitude, in dBFS.
    pub peak_db: f32,
}

impl AdcHealth {
    /// Distinct levels past which the converter is counted as driven. An
    /// eight bit converter with its noise at a step and a half shows a
    /// dozen; a starved one shows two or three.
    pub const LEVELS_ENOUGH: u16 = 8;
    /// Samples within this of full scale are on the rail. Eight bit sources
    /// land at 127/127.5 rather than exactly one.
    const RAIL: f32 = 0.98;
    /// Share of samples on the rail that counts as clipping. A burst of
    /// packet clips its own peaks and nobody wants a warning for that; a
    /// tenth of the frame is a gain that is too high.
    const CLIP_FRAC: f32 = 0.05;

    pub fn measure(iq: &[C32]) -> Self {
        if iq.is_empty() {
            return Self::default();
        }
        let mut peak = 0.0f32;
        let mut rails = 0usize;
        for s in iq {
            let m = s.re.abs().max(s.im.abs());
            peak = peak.max(m);
            if m >= Self::RAIL {
                rails += 1;
            }
        }
        // Distinct values are counted on a bounded sample: a frame is a few
        // thousand samples and the answer wanted is "two" or "plenty".
        let mut re: Vec<f32> = iq.iter().take(1024).map(|s| s.re).collect();
        re.sort_by(f32::total_cmp);
        re.dedup();
        Self {
            levels: re.len().min(Self::LEVELS_ENOUGH as usize) as u16,
            clipped: rails as f32 / iq.len() as f32,
            peak_db: 20.0 * peak.max(1e-6).log10(),
        }
    }

    pub fn starved(&self) -> bool {
        self.levels > 0 && self.levels < Self::LEVELS_ENOUGH
    }

    pub fn clipping(&self) -> bool {
        self.clipped >= Self::CLIP_FRAC
    }
}

/// The FFT behind the spectrum and the waterfall.
pub struct SpectrumNode {
    spec: Spectrum,
    adc: AdcHealth,
    rate: f64,
    center: common::Hz,
    /// Whether the last block completed a frame, so the host knows when there
    /// is something new to draw rather than redrawing the same bins.
    fresh: bool,
    /// Frames per second worth producing.
    refresh_hz: f32,
    /// Samples still to go before the next frame is published.
    ///
    /// Only the publishing is gated. Every sample is transformed and folded
    /// into the frame being built, so a burst between two published frames
    /// is in the next one rather than lost.
    debt: f64,
    /// What the trace shows out of each frame, and what the waterfall does.
    ///
    /// Two settings because the two are read for different things: a trace
    /// is watched to judge a level, a waterfall to notice that something
    /// happened. The same split a spectrum analyser makes between its
    /// detector and its trace mode.
    trace: Detector,
    wf: Detector,
    /// The waterfall's own reading of the last frame, which the host takes
    /// beside the trace.
    wf_db: Vec<f32>,
}

/// The detectors, as a picker's options. The percentile is named for the
/// number it is set to, so the option reads `p80` rather than `percentile`.
fn detectors(current: Detector) -> Vec<String> {
    Detector::options(current).iter().map(|d| d.label()).collect()
}

fn index_of(d: Detector) -> usize {
    Detector::ALL.iter().position(|x| x.at_percent(0) == d.at_percent(0)).unwrap_or(0)
}

/// Whichever detector a setting names, by index or by name: a patch holds
/// the index, and a saved record or an agent says the word.
///
/// An index keeps the percent already set, because the picker and the number
/// beside it are two settings for one value.
fn detector_at(v: &ParamValue, current: Detector) -> Detector {
    if let Some(name) = v.as_str() {
        return Detector::parse(name);
    }
    let i = v.as_i64().unwrap_or(0).max(0) as usize;
    let d = Detector::ALL.get(i).copied().unwrap_or_default();
    d.at_percent(current.percent().unwrap_or(spectrum::DEFAULT_PERCENT))
}

/// A detector moved to the percent a setting names, which a detector that
/// takes no percentile ignores.
fn at_percent(d: Detector, v: &ParamValue) -> Detector {
    let p = v.as_f64().unwrap_or(f64::from(spectrum::DEFAULT_PERCENT));
    d.at_percent(p.clamp(1.0, 99.0) as u8)
}

/// The transform size a spectrum stage's description asks for, rounded down
/// to the power of two the transform takes.
///
/// One reading of the setting, so the size a stage asks for and the size a
/// node already has are compared on the same terms.
pub fn spectrum_size(settings: &Settings) -> usize {
    let size = settings.i64_or(SIZE, DEFAULT_SPECTRUM_SIZE).clamp(64, 32_768) as usize;
    1usize << (usize::BITS - 1 - size.leading_zeros()) as usize
}

impl SpectrumNode {
    pub fn new(size: usize) -> Self {
        Self {
            spec: Spectrum::new(size),
            rate: 0.0,
            center: common::Hz(0),
            adc: AdcHealth::default(),
            fresh: false,
            refresh_hz: 30.0,
            debt: 0.0,
            trace: Detector::Average,
            wf: Detector::Peak,
            wf_db: Vec::new(),
        }
    }

    /// What the trace takes out of a frame.
    pub fn set_trace(&mut self, d: Detector) {
        self.trace = d;
    }

    pub fn trace(&self) -> Detector {
        self.trace
    }

    /// The percent a detector is set to, which a plain reading reports as
    /// the default rather than as nothing: the control is always on the
    /// card, and reads what it would take if it were chosen.
    fn percent(&self, d: Detector) -> u8 {
        d.percent().unwrap_or(spectrum::DEFAULT_PERCENT)
    }

    /// What the waterfall takes out of the same frame.
    pub fn set_waterfall(&mut self, d: Detector) {
        self.wf = d;
    }

    pub fn waterfall(&self) -> Detector {
        self.wf
    }

    /// The waterfall's reading of the last published frame.
    pub fn waterfall_db(&self) -> &[f32] {
        &self.wf_db
    }

    /// How often a frame is worth producing.
    pub fn set_refresh(&mut self, hz: f32) {
        self.refresh_hz = hz.clamp(1.0, 240.0);
    }

    pub fn refresh(&self) -> f32 {
        self.refresh_hz
    }

    pub fn size(&self) -> usize {
        self.spec.size()
    }

    /// Whether a new frame completed in the last block.
    pub fn is_fresh(&self) -> bool {
        self.fresh
    }

    /// The averaged power spectrum, in dBFS.
    pub fn power_db(&mut self) -> &[f32] {
        self.spec.power_db()
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// How the converter was driven in the last frame.
    pub fn adc(&self) -> AdcHealth {
        self.adc
    }

    pub fn center(&self) -> common::Hz {
        self.center
    }

    pub fn set_smoothing(&mut self, v: f32) {
        self.spec.smoothing = v.clamp(0.0, 1.0);
    }

    /// How much of the last frame the next one keeps.
    pub fn smoothing(&self) -> f32 {
        self.spec.smoothing
    }
}

impl Simple for SpectrumNode {
    fn name(&self) -> &str {
        "spectrum"
    }

    /// Not a sink any more: the frames it computes go out on a port, so a
    /// recorder reads the transform the display is already running rather
    /// than running a second one over the same samples.
    fn is_sink(&self) -> bool {
        false
    }

    /// The transform cannot be resized, and one holding an average of
    /// another band is worse than one starting empty.
    fn survives_rebuild(&self, retuned: bool, settings: &Settings) -> bool {
        !retuned && self.size() == spectrum_size(settings)
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(Error::other("spectrum needs IQ"));
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        // Frames rather than samples: the rate a reader wants from this
        // port is how many frames a second arrive, not the sample rate the
        // transform ate.
        let mut out = i.spec.with_kind(PortKind::Spectrum);
        out.rate = f64::from(self.refresh_hz);
        Ok(out)
    }

    /// Every sample is transformed, and a frame is published at the display
    /// rate holding everything since the last one.
    ///
    /// The transform used to run on one block in thirty and the rest were
    /// thrown away, which is fine for a display and wrong for anything that
    /// records: a burst is milliseconds long and would be seen only if it
    /// landed in the sampled block. What is published is the loudest each
    /// bin reached over the interval, and the mean beside it; the display's
    /// own smoothing is applied per published frame instead of per
    /// transform, so the picture reads the same at any sample rate.
    ///
    /// Measured at 1.1% of a core at 2.4 MS/s and 1024 bins, and 1.3% at
    /// 16384, by `dsp::spectrum::tests::what_full_coverage_costs`. The old
    /// gate saved that and cost every burst between frames.
    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let iq = i.as_iq().unwrap_or(&[]);
        self.fresh = false;
        if let Some(out) = o.spectrum_mut() {
            out.clear();
        }
        if !iq.is_empty() {
            self.spec.process(iq);
            self.adc = AdcHealth::measure(iq);
        }
        self.debt -= iq.len() as f64;
        if self.debt > 0.0 || self.spec.pending_frames() == 0 {
            return Ok(());
        }
        self.debt = self.rate / self.refresh_hz.max(1.0) as f64;
        let frame = self.spec.take(&[self.trace, self.wf]);
        self.spec.fold(self.trace.of(&frame));
        self.wf_db.clear();
        self.wf_db.extend_from_slice(self.wf.of(&frame));
        self.fresh = true;
        if let Some(out) = o.spectrum_mut() {
            out.push(common::SpectrumFrame {
                at_us: common::packet::now_us(),
                center_hz: self.center.as_f64(),
                span_hz: self.rate,
                db: std::sync::Arc::new(frame.peak),
                mean_db: std::sync::Arc::new(frame.mean),
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.spec.reset();
        self.fresh = false;
        self.debt = 0.0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("smoothing", self.spec.smoothing as f64, 0.0..=0.99)
                .label("Frame to frame averaging"),
            Param::float("refresh", self.refresh_hz as f64, 1.0..=120.0)
                .unit("Hz")
                .label("Frames a second"),
            Param::choice("trace", index_of(self.trace), detectors(self.trace))
                .label("What the trace shows"),
            Param::float("trace_percent", f64::from(self.percent(self.trace)), 1.0..=99.0)
                .unit("%")
                .label("Which percentile the trace takes"),
            Param::choice("waterfall", index_of(self.wf), detectors(self.wf))
                .label("What the waterfall shows"),
            Param::float("waterfall_percent", f64::from(self.percent(self.wf)), 1.0..=99.0)
                .unit("%")
                .label("Which percentile the waterfall takes"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "smoothing" => {
                self.set_smoothing(v.as_f64().unwrap_or_default() as f32);
                Ok(())
            }
            "refresh" => {
                self.set_refresh(v.as_f64().unwrap_or(30.0) as f32);
                Ok(())
            }
            "trace" => {
                self.trace = detector_at(&v, self.trace);
                Ok(())
            }
            "waterfall" => {
                self.wf = detector_at(&v, self.wf);
                Ok(())
            }
            "trace_percent" => {
                self.trace = at_percent(self.trace, &v);
                Ok(())
            }
            "waterfall_percent" => {
                self.wf = at_percent(self.wf, &v);
                Ok(())
            }
            _ => Err(Error::other(format!("spectrum: unknown parameter {name:?}"))),
        }
    }
}

/// A tap that keeps the recent past, so a transmission can be saved after the
/// fact.
///
/// The node owns the ring and nothing else does. What to keep is a decision
/// made from decoded events, which arrive after this node has already run, so
/// the host makes that call between blocks through [`RingNode::ring_mut`]
/// rather than the node trying to read the future.
pub struct RingNode<T: Ring> {
    ring: T,
    rate: f64,
}

/// Whatever the host wants filled with raw samples.
///
/// A trait rather than a concrete recorder so `nodes` does not have to know
/// how a file is written, which is an application's business.
pub trait Ring: Send + 'static {
    fn push(&mut self, iq: &[C32]);
    fn reset(&mut self) {}
}

impl<T: Ring> RingNode<T> {
    pub fn new(ring: T) -> Self {
        Self { ring, rate: 0.0 }
    }

    pub fn ring(&self) -> &T {
        &self.ring
    }

    pub fn ring_mut(&mut self) -> &mut T {
        &mut self.ring
    }

    pub fn into_ring(self) -> T {
        self.ring
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }
}

impl<T: Ring> Simple for RingNode<T> {
    fn name(&self) -> &str {
        "record"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(Error::other("record needs IQ"));
        }
        self.rate = i.spec.rate;
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.ring.push(i.as_iq().unwrap_or(&[]));
        Ok(())
    }

    fn reset(&mut self) {
        self.ring.reset();
    }
}

/// Somewhere to put every packet a receiver hears.
///
/// A trait rather than a file, so `nodes` does not have to know where the
/// data goes or what it is written as, which is an application's business.
pub trait PacketSink: Send + 'static {
    fn write(&mut self, p: &common::packet::Packet);
    /// How many have been written, for a status line.
    fn written(&self) -> u64 {
        0
    }

    /// How large the sink has grown, and whether it has stopped accepting
    /// anything. A log that quietly stopped writing is worse than one that
    /// never started, so the interface has to be able to say which it is.
    fn bytes(&self) -> u64 {
        0
    }

    fn full(&self) -> bool {
        false
    }

    /// A chance to put buffered work somewhere durable, called once per
    /// block whether or not any packets arrived.
    ///
    /// A sink that batches needs a tick from outside it: the last burst
    /// before a band goes quiet has nothing following it to push it out.
    fn flush(&mut self) {}
}

/// The packet bus: where everything that produces packets meets everything
/// that consumes them.
///
/// It has one input per source, because a receiver hears packets from several
/// places at once: the OOK bank, the FSK bank, and on 1090 MHz a demodulator
/// that produces frames rather than timings. They arrive in different shapes
/// and leave in one, so a consumer never has to care which front end was
/// involved.
///
/// Consumers hang off its output rather than off the demodulators, and that
/// is the point of it: a flight list, a packet list, a map and a chart all
/// want the same stream, and attaching each of them to the demodulator it
/// happens to care about would rebuild the same fan-out four times and make
/// every one of them a special case.
///
/// What travels is what the demodulator produced and nothing else. The parsed
/// frame is deliberately absent: a parse is a conclusion, and a conclusion
/// travelling in place of its evidence cannot be checked later, corrected by
/// a better decoder, or shown to have been wrong. Timings can be decoded
/// again next year; a field map cannot be un-decoded.
///
/// The file is optional and the bus is not. Turning the log off should stop
/// writing to disk, not disconnect every view from the traffic.
pub struct PacketBusNode {
    sink: Option<Box<dyn PacketSink>>,
    inputs: usize,
    /// Which inputs have already been reported for putting a packet on the
    /// bus with no level. Once each, because the offender is a front end and
    /// it will do it for every packet it produces.
    unmeasured: std::collections::HashSet<usize>,
}

impl PacketBusNode {
    pub fn new(inputs: usize) -> Self {
        Self { sink: None, inputs: inputs.max(1), unmeasured: std::collections::HashSet::new() }
    }

    pub fn with_sink(mut self, sink: Option<Box<dyn PacketSink>>) -> Self {
        self.sink = sink;
        self
    }

    pub fn set_sink(&mut self, sink: Option<Box<dyn PacketSink>>) {
        self.sink = sink;
    }

    pub fn has_sink(&self) -> bool {
        self.sink.is_some()
    }

    /// Change how many sources feed it.
    ///
    /// The count is fixed at construction because the graph asks a node how
    /// many inputs it has before wiring it, and it changes whenever the set
    /// of front ends does: tuning from 433 MHz to 1090 replaces two channel
    /// banks with one Mode S demodulator. Without this the bus carried into
    /// the new graph still claims the old count and the build fails with a
    /// port that nothing connected.
    pub fn set_inputs(&mut self, n: usize) {
        self.inputs = n.max(1);
        self.unmeasured.clear();
    }

    /// Say so, once, when a packet arrives without a finite level.
    ///
    /// The one place every packet passes through, so a front end that has not
    /// been taught to measure is named here rather than showing up as a blank
    /// column in the list and a row nobody can sort, judge or compare. A feed
    /// from another receiver is the honest exception: the far end reports
    /// what it measured, and the AVR format reports nothing.
    fn check_measured(&mut self, k: usize, p: &common::packet::Packet, ctx: &mut NodeCtx<'_>) {
        if p.carrier.rssi_dbfs.is_finite() && p.carrier.snr_db.is_finite() {
            return;
        }
        if !self.unmeasured.insert(k) {
            return;
        }
        ctx.warn(format!(
            "input {k} put a packet on the bus at {:.4} MHz with no level: \
             rssi {}, snr {}",
            p.carrier.center_hz as f64 / 1e6,
            p.carrier.rssi_dbfs,
            p.carrier.snr_db
        ));
    }

    /// Packets written to the sink, or zero when there is none.
    pub fn written(&self) -> u64 {
        self.sink.as_ref().map(|s| s.written()).unwrap_or(0)
    }

    /// Size of what the sink has written, and whether it has given up.
    pub fn sink_bytes(&self) -> u64 {
        self.sink.as_ref().map(|s| s.bytes()).unwrap_or(0)
    }

    pub fn sink_full(&self) -> bool {
        self.sink.as_ref().is_some_and(|s| s.full())
    }
}

/// Wall clock, in microseconds since the epoch.
///
/// Each block is stamped as it is processed, which on a live receiver is
/// within a block of when the burst arrived: the same accuracy the packet
/// list has always shown. Replaying a file stamps the packets with the time
/// of the replay, because that is when the receiver heard them; the file's
/// own timeline belongs to the file.
impl pipeline::node::Node for PacketBusNode {
    fn name(&self) -> &str {
        "packet_bus"
    }

    /// How many inputs the bus has is how many things feed it, which changes
    /// with every retune. It is carried across rebuilds because it holds the
    /// open log file, so it has to be told.
    fn configure(&mut self, settings: &Settings) {
        self.set_inputs(settings.i64_or(INPUTS, 1).max(1) as usize);
    }

    fn num_inputs(&self) -> usize {
        self.inputs
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        for i in inputs {
            if !matches!(i.spec.kind, PortKind::Pulses | PortKind::Packets) {
                return Err(Error::other("the packet bus takes detected bursts or packets"));
            }
        }
        let first = inputs.first().map(|i| i.spec).unwrap_or(StreamSpec::iq(0.0, common::Hz(0)));
        let mut out = first.with_kind(PortKind::Packets);
        // Packets are events in time, not a sampled stream.
        out.rate = 0.0;
        Ok(vec![out])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let at_us = common::packet::now_us();
        let out = outputs[0].packets_mut();
        for (k, payload) in inputs.iter().enumerate() {
            let spec = ctx.inputs.get(k).map(|p| p.spec);
            let from = out.len();
            match payload {
                Payload::Pulses(found) => {
                    let bandwidth_hz = spec.map(|s| s.bandwidth as u32).unwrap_or(0);
                    let center_hz = spec.map(|s| s.center.0).unwrap_or(0);
                    for d in found.iter() {
                        // Assembled here, where the stream the detector was
                        // reading is known: a detection is timings and a
                        // level, and the stream is what places it.
                        let carrier = common::packet::Carrier::heard(
                            at_us,
                            center_hz,
                            bandwidth_hz,
                            d.rssi_dbfs,
                            d.snr_db,
                            common::SourceId(0),
                        )
                        .lasting(d.duration_us);
                        out.push(common::packet::Packet::heard(carrier).keyed(d.keying.clone()));
                    }
                }
                // A feed from another receiver arrives already stamped: it
                // knows when it heard the frame, on what frequency and how
                // strongly, and none of that should be replaced with ours.
                Payload::Packets(ps) => out.extend(ps.iter().cloned()),
                _ => {}
            }
            // Checked as it goes on, so what is judged is what the bus will
            // carry: a packet built here from pulses or from a frame.
            if let Some(p) = out[from..]
                .iter()
                .find(|p| !(p.carrier.rssi_dbfs.is_finite() && p.carrier.snr_db.is_finite()))
            {
                self.check_measured(k, p, ctx);
            }
        }
        if let Some(sink) = self.sink.as_mut() {
            for p in out.iter() {
                sink.write(p);
            }
            sink.flush();
        }
        Ok(())
    }
}

/// The DC notch at the head of the chain.
///
/// A direct-conversion front end puts local oscillator leakage and the ADC's
/// offset at exactly the tuned frequency, which reads as a very strong carrier
/// that is not there. It is removed here, before anything else sees it, so the
/// spectrum, the detectors and the audio all agree about what arrived.
pub struct DcBlockNode {
    dc: Option<dsp::DcBlock>,
    rate: f64,
    enabled: bool,
}

impl Default for DcBlockNode {
    fn default() -> Self {
        Self::new()
    }
}

impl DcBlockNode {
    pub fn new() -> Self {
        Self { dc: None, rate: 0.0, enabled: true }
    }

    pub fn set_enabled(&mut self, on: bool) {
        if on != self.enabled {
            self.enabled = on;
            // The estimate belongs to a particular tuning and gain setting;
            // switching back on later should measure again rather than
            // subtract whatever was true minutes ago.
            self.dc = None;
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Forget the measured offset. Every retune and gain change moves it, and
    /// a stale estimate shows up as a spur that was not there a moment ago.
    pub fn remeasure(&mut self) {
        self.dc = None;
    }

    pub fn offset(&self) -> C32 {
        self.dc.as_ref().map(|d| d.offset()).unwrap_or_default()
    }
}

impl Simple for DcBlockNode {
    fn name(&self) -> &str {
        "dc_block"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(Error::other("dc block needs IQ"));
        }
        if self.rate != i.spec.rate {
            self.rate = i.spec.rate;
            // The notch width is set from the rate.
            self.dc = None;
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let iq = i.as_iq().unwrap_or(&[]);
        let out = o.iq_mut();
        out.extend_from_slice(iq);
        if !self.enabled {
            return Ok(());
        }
        let rate = self.rate;
        let dc = self.dc.get_or_insert_with(|| {
            let mut d = dsp::DcBlock::new(rate);
            // Primed from the first block it sees, so the notch does not have
            // to settle while the display is already showing the result.
            d.prime(iq);
            d
        });
        dc.process(out);
        Ok(())
    }

    fn reset(&mut self) {
        self.dc = None;
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::bool("enabled", self.enabled).label("Remove the centre spur")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => {
                self.set_enabled(v.as_bool().unwrap_or(true));
                Ok(())
            }
            _ => Err(Error::other(format!("dc_block: unknown parameter {name:?}"))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(433_920_000)), latency: 0 }
    }

    fn tone(n: usize) -> Vec<C32> {
        (0..n)
            .map(|k| {
                let p = std::f32::consts::TAU * 0.1 * k as f32;
                C32::new(p.cos(), p.sin())
            })
            .collect()
    }

    #[test]
    fn the_spectrum_reports_a_frame_only_when_one_completed() {
        let mut s = SpectrumNode::new(256);
        Simple::negotiate(&mut s, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let mut tags = Vec::new();
        let ins = [spec(2_400_000.0)];

        let mut run = |n: usize, s: &mut SpectrumNode| {
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Simple::process(s, &Payload::Iq(tone(n)), &mut out, &mut ctx).unwrap();
            s.is_fresh()
        };

        assert!(!run(64, &mut s), "a quarter of a frame is not a frame");
        assert!(run(256, &mut s), "past a full frame there is something to draw");
        // At 2.4 MS/s and 30 frames a second, the next 80k samples are not
        // worth transforming.
        assert!(!run(4096, &mut s), "the gate holds off the next frame");
        tags.clear();
    }

    #[test]
    fn the_spectrum_finds_the_tone_where_it_is() {
        let mut s = SpectrumNode::new(256);
        Simple::negotiate(&mut s, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let tags = Vec::new();
        let ins = [spec(2_400_000.0)];
        let mut new_tags = Vec::new();
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(&mut s, &Payload::Iq(tone(4096)), &mut out, &mut ctx).unwrap();

        // 0.1 cycles per sample, so bin 0.1 * 256 from the centre.
        let db = s.power_db().to_vec();
        let peak = db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(peak, 128 + 26, "peak at bin {peak} of {}", db.len());
    }

    #[test]
    fn a_sink_writes_no_stream() {
        // The graph gives every node an output slot; a sink leaves it empty
        // rather than copying its input into a buffer nobody reads.
        let mut s = SpectrumNode::new(256);
        Simple::negotiate(&mut s, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let tags = Vec::new();
        let ins = [spec(2_400_000.0)];
        let mut new_tags = Vec::new();
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(&mut s, &Payload::Iq(tone(512)), &mut out, &mut ctx).unwrap();
        assert_eq!(out.len(), 0);
    }

    struct Counting(usize);
    impl Ring for Counting {
        fn push(&mut self, iq: &[C32]) {
            self.0 += iq.len();
        }
    }

    #[test]
    fn the_ring_sees_every_sample_the_graph_was_fed() {
        let mut r = RingNode::new(Counting(0));
        Simple::negotiate(&mut r, &spec(2_400_000.0)).unwrap();
        let mut out = Payload::Iq(Vec::new());
        let mut events = Vec::new();
        let tags = Vec::new();
        let ins = [spec(2_400_000.0)];
        for _ in 0..3 {
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Simple::process(&mut r, &Payload::Iq(tone(100)), &mut out, &mut ctx).unwrap();
        }
        assert_eq!(r.ring().0, 300);
    }
}

#[cfg(test)]
mod adc_tests {
    use super::AdcHealth;
    use common::C32;

    /// The capture that showed the need: an RTL at 27 dB on 868 MHz whose
    /// every sample was 127 or 128, so the whole band was one bit of
    /// quantisation noise and the first packet to arrive dropped it by
    /// eleven decibels everywhere else.
    #[test]
    fn two_values_is_starved_and_a_driven_converter_is_not() {
        let starved: Vec<C32> = (0..4096)
            .map(|i| {
                let v = |k: u32| {
                    if (i * 7 + k).is_multiple_of(3) { -0.5 / 127.5 } else { 0.5 / 127.5 }
                };
                C32::new(v(0), v(1))
            })
            .collect();
        let h = AdcHealth::measure(&starved);
        assert_eq!(h.levels, 2);
        assert!(h.starved() && !h.clipping());

        let mut seed = 12345u32;
        let mut noise = || {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            ((seed >> 16) as i32 % 21 - 10) as f32 / 127.5
        };
        let driven: Vec<C32> = (0..4096).map(|_| C32::new(noise(), noise())).collect();
        let h = AdcHealth::measure(&driven);
        assert!(!h.starved() && !h.clipping(), "{h:?}");

        let railed: Vec<C32> = (0..4096)
            .map(|i| if i % 4 == 0 { C32::new(1.0, -1.0) } else { C32::new(0.1, 0.2) })
            .collect();
        assert!(AdcHealth::measure(&railed).clipping());
    }
}

/// The setting names these stages read.
const INPUTS: &str = "inputs";
const SIZE: &str = "size";

/// Bins in the display transform when nothing has asked for more.
const DEFAULT_SPECTRUM_SIZE: i64 = 1_024;

pub const PACKET_BUS: StageDesc = StageDesc {
    name: "packet_bus",
    summary: "Gather bursts, frames and packets from every front end \
              into one stream, and write them to the log",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build_packet_bus(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(PacketBusNode::new(s.i64_or(INPUTS, 1).max(1) as usize)))
}

pub const DC_BLOCK: StageDesc = StageDesc {
    name: "dc_block",
    summary: "Remove the centre spur a direct-conversion receiver \
              produces, by measuring it rather than notching it out",
    category: Category::Filter,
    feeds_bus: false,
};

pub fn build_dc_block(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DcBlockNode::new()))
}

pub const SPECTRUM: StageDesc = StageDesc {
    name: "spectrum",
    summary: "An FFT display of whatever is wired into it, drawn \
              alongside the receiver's own",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build_spectrum(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(SpectrumNode::new(spectrum_size(s))))
}
