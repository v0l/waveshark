//! Transmit stages: text to timings, timings to IQ.
//!
//! The same shape as the receive side read backwards. A keyer turns bytes
//! into a [`PortKind::Pulses`] burst the way a slicer turns a burst into
//! bytes, and a modulator turns that burst into [`PortKind::Iq`] the way a
//! detector turns IQ into a burst. Both are nodes, so a transmission is
//! visible in the chain view, tappable, and parameterised like everything
//! else the receiver does.
//!
//! A chain does not usually wire the two by hand. [`MorseTxNode`] is one node
//! taking bytes and producing IQ, holding both inside and showing them
//! through [`pipeline::node::Node::subgraphs`], which is the transmit mirror
//! of the front ends that take IQ and produce packets. The modulator is
//! shared: it keys whatever timings arrive, so every protocol with a timing
//! table adds an encoder and reuses this carrier.

use crate::mod_nodes::OokModNode;
use common::{Result, C32};
use pipeline::graph::Topology;
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec};
use pipeline::Graph;
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Bytes of text in, Morse timings out.
pub struct MorseKeyNode {
    wpm: f32,
}

impl Default for MorseKeyNode {
    fn default() -> Self {
        Self { wpm: 20.0 }
    }
}

impl MorseKeyNode {
    pub fn new(wpm: f32) -> Self {
        Self {
            wpm: wpm.clamp(1.0, 60.0),
        }
    }
}

impl Simple for MorseKeyNode {
    fn name(&self) -> &str {
        "morse_key"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Bytes {
            return Err(common::Error::other("morse_key takes text as bytes"));
        }
        Ok(StreamSpec {
            kind: PortKind::Pulses,
            // Timings are microseconds, so the port has no sample rate of its
            // own; the rate it carries is the one the modulator will key at,
            // passed through so a chain stays rate-consistent.
            rate: input.spec.rate,
            center: input.spec.center,
            bandwidth: input.spec.bandwidth,
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
        let Some(bytes) = input.as_bytes() else {
            return Ok(());
        };
        if bytes.is_empty() {
            return Ok(());
        }
        let text = String::from_utf8_lossy(bytes);
        let pkg = decode::morse::encode(&text, self.wpm);
        if !pkg.pulses.is_empty() {
            output.pulses_mut().push(pkg);
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(WPM, self.wpm as f64, 1.0..=60.0)
            .label("Speed")
            .unit("wpm")]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            WPM => {
                self.wpm = value.as_f64().unwrap_or(20.0).clamp(1.0, 60.0) as f32;
                Ok(())
            }
            _ => Err(common::Error::other(format!(
                "morse_key: unknown parameter {name:?}"
            ))),
        }
    }
}

/// Morse as one stage: text in, keyed carrier out.
///
/// The transmit mirror of a front end like [`crate::M17Node`], which takes IQ
/// and produces packets with its stages inside it. What it holds is an
/// ordinary graph of the two stages above, so the timings between them are a
/// real edge that the chain view draws and a tap can read, and the modulator
/// is the same one every other keyed protocol will use.
pub struct MorseTxNode {
    inner: Graph,
}

impl Default for MorseTxNode {
    fn default() -> Self {
        Self::new(20.0, 0.0)
    }
}

impl MorseTxNode {
    pub fn new(wpm: f32, offset_hz: f64) -> Self {
        // A placeholder rate, because a graph negotiates as it is built and
        // the real rate is not known until the graph around this node says
        // what the radio is running at. `negotiate` replaces it.
        let input = StreamSpec {
            kind: PortKind::Bytes,
            rate: 1.0,
            flow: Flow::Tx,
            ..StreamSpec::default()
        };
        let mut b = Graph::builder(input);
        let key = b.add_labeled("morse_key", Box::new(MorseKeyNode::new(wpm)));
        let modu = b.add_labeled("ook_mod", Box::new(OokModNode::new(offset_hz, 0.25, 500.0)));
        b.source(key.i());
        b.link(key, modu);
        b.output(modu.o());
        Self {
            inner: b.build().expect("morse_tx graph is fixed and acyclic"),
        }
    }
}

impl Node for MorseTxNode {
    fn name(&self) -> &str {
        "morse_tx"
    }

    fn subgraphs(&self) -> Vec<Topology> {
        vec![self.inner.topology()]
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Bytes {
            return Err(common::Error::other("morse_tx takes text as bytes"));
        }
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other(
                "morse_tx needs the rate it should key at",
            ));
        }
        *self.inner.input_buf() = Payload::Bytes(Vec::new());
        self.inner.set_input_spec(StreamSpec {
            kind: PortKind::Bytes,
            rate: i.spec.rate,
            center: i.spec.center,
            bandwidth: i.spec.bandwidth,
            channels: 1,
            flow: Flow::Tx,
            domain: Domain::Baseband,
        })?;
        Ok(vec![self.inner.output_spec()])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(bytes) = inputs[0].as_bytes() else {
            return Ok(());
        };
        if bytes.is_empty() {
            return Ok(());
        }
        {
            let buf = self.inner.input_buf();
            buf.clear();
            buf.bytes_mut().extend_from_slice(bytes);
        }
        self.inner.run()?;
        if let Some(iq) = self.inner.output().as_iq() {
            outputs[0].iq_mut().extend_from_slice(iq);
        }
        for t in self.inner.output_tags() {
            ctx.tag(t.clone());
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn params(&self) -> Vec<Param> {
        let mut out = Vec::new();
        for (id, _) in self.inner.order() {
            if let Some(n) = self.inner.node(id) {
                out.extend(n.params());
            }
        }
        out
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let ids: Vec<_> = self.inner.order().map(|(id, _)| id).collect();
        for id in ids {
            let Some(n) = self.inner.node_mut(id) else {
                continue;
            };
            if n.params().iter().any(|p| p.name == name) {
                return n.set_param(name, value);
            }
        }
        Err(common::Error::other(format!(
            "morse_tx: unknown parameter {name:?}"
        )))
    }
}

/// A tone, added to whatever is already on the stream.
///
/// The simplest thing to modulate and the standard way to check a
/// transmitter: a steady tone on an NFM carrier is what a service monitor
/// measures deviation from, and what tells you the audio chain is alive
/// before speech is involved. Added rather than substituted, so it can also
/// be laid over real audio as a test tone.
pub struct ToneNode {
    hz: f64,
    level: f32,
    rate: f64,
    phase: f64,
}

impl Default for ToneNode {
    fn default() -> Self {
        Self {
            hz: 1_000.0,
            level: 0.5,
            rate: 0.0,
            phase: 0.0,
        }
    }
}

impl ToneNode {
    pub fn new(hz: f64, level: f32) -> Self {
        Self {
            hz,
            level: level.clamp(0.0, 1.0),
            ..Self::default()
        }
    }
}

impl Simple for ToneNode {
    fn name(&self) -> &str {
        "tone"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Real {
            return Err(common::Error::other("tone runs on a real audio stream"));
        }
        self.rate = input.spec.rate;
        Ok(input.spec)
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(audio) = input.as_real() else {
            return Ok(());
        };
        let step = std::f64::consts::TAU * self.hz / self.rate.max(1.0);
        let out = output.real_mut();
        out.reserve(audio.len());
        for &a in audio {
            out.push(a + self.level * self.phase.sin() as f32);
            self.phase += step;
            if self.phase > std::f64::consts::TAU {
                self.phase -= std::f64::consts::TAU;
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.phase = 0.0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(HZ, self.hz, 20.0..=20_000.0)
                .label("Tone")
                .unit("Hz"),
            Param::float(LEVEL, self.level as f64, 0.0..=1.0).label("Level"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let v = value.as_f64().unwrap_or(0.0);
        match name {
            HZ => self.hz = v.max(0.0),
            LEVEL => self.level = v.clamp(0.0, 1.0) as f32,
            _ => {
                return Err(common::Error::other(format!(
                    "tone: unknown parameter {name:?}"
                )))
            }
        }
        Ok(())
    }
}

/// The end of a transmit chain: IQ in, samples out of the antenna, and what
/// went out on its own port for [`TxMonitorNode`] to draw.
///
/// The mirror of the radio at the head of the receive graph, and the node
/// that makes a transmission visible as something the graph does rather than
/// something done to it. It is also what paces the whole chain: the device's
/// write blocks once the radio has enough queued, so a graph ending here runs
/// at the sample rate without anything timing it.
///
/// Refuses a receive stream. A demodulator's output wired into a transmitter
/// is the mistake this exists to make impossible.
pub struct TxSinkNode {
    /// The radio, while a channel is keyed. Absent the rest of the time: the
    /// stage is in the graph whether or not anything is transmitting, so the
    /// chain can be seen and set up before the key is pressed, and so keying
    /// does not rebuild the graph and throw away the spectrum's averaging.
    stream: Option<Box<dyn common::TxStream>>,
    rate: common::Sps,
    center: common::Hz,
    written: u64,
    /// Blocks the device could not take, because the radio went away.
    failed: u64,
}

impl TxSinkNode {
    /// A transmitter with no radio yet: in the graph, and off.
    pub fn idle() -> Self {
        Self {
            stream: None,
            rate: common::Sps(0),
            center: common::Hz(0),
            written: 0,
            failed: 0,
        }
    }

    /// Hand it a radio: the key going down.
    pub fn attach(&mut self, stream: Box<dyn common::TxStream>) {
        self.finish(std::time::Duration::from_millis(200));
        self.written = 0;
        self.failed = 0;
        self.stream = Some(stream);
    }

    /// Whether it is transmitting now.
    pub fn keyed(&self) -> bool {
        self.stream.is_some()
    }

    pub fn new(stream: Box<dyn common::TxStream>) -> Self {
        Self {
            stream: Some(stream),
            rate: common::Sps(0),
            center: common::Hz(0),
            written: 0,
            failed: 0,
        }
    }

    /// Complex samples handed to the radio since the node was built.
    pub fn written(&self) -> u64 {
        self.written
    }

    /// Transfers the radio sent as silence because nothing was queued in
    /// time. Non-zero means the transmission has holes in it.
    pub fn underruns(&self) -> u64 {
        self.stream.as_ref().map(|s| s.underruns()).unwrap_or(0)
    }

    pub fn failed_blocks(&self) -> u64 {
        self.failed
    }

    /// Let everything written reach the radio, then stop transmitting.
    ///
    /// The stage stays in the graph: what it loses is the radio, which is
    /// also what hands a half duplex one back to the receiver.
    pub fn finish(&mut self, timeout: std::time::Duration) {
        if let Some(s) = &mut self.stream {
            s.drain(timeout);
            s.stop();
        }
        self.stream = None;
    }
}

impl Simple for TxSinkNode {
    fn name(&self) -> &str {
        "radio_tx"
    }

    /// Not a sink: what went to the antenna leaves here as well, for
    /// [`TxMonitorNode`] to put back on the receiver's own spectrum. A half
    /// duplex radio hears nothing while it transmits, and the only thing
    /// holding the samples that did go out is this stage.
    fn is_sink(&self) -> bool {
        false
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Iq {
            return Err(common::Error::other("radio_tx needs IQ"));
        }
        if !input.spec.is_tx() {
            return Err(common::Error::other(
                "radio_tx was given a receive stream; a modulator has to be in front of it",
            ));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("radio_tx needs a sample rate"));
        }
        self.rate = common::Sps(input.spec.rate.round() as u64);
        self.center = input.spec.center;
        Ok(input.spec)
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = input.as_iq() else {
            return Ok(());
        };
        if iq.is_empty() {
            return Ok(());
        }
        // Not keyed: the chain runs and produces, and nothing leaves the
        // antenna. That is what makes the stages worth having in the graph
        // when the key is up, since the modulator's output can be tapped and
        // the levels set before anything is radiated. Nothing leaves this
        // port either, so the monitor draws nothing.
        let Some(s) = &mut self.stream else {
            return Ok(());
        };
        output.iq_mut().extend_from_slice(iq);
        let buf = common::IqBuf::new(iq.to_vec(), self.center, self.rate, self.written);
        match s.write(&buf) {
            Ok(()) => self.written += iq.len() as u64,
            // The radio going away must not take the graph down with it: a
            // receiver that keeps running is more useful than one that exits
            // because a transmission could not finish. Said once, because a
            // transmitter that has stopped taking samples will refuse every
            // block from here on and the first refusal is the news.
            Err(e) => {
                if self.failed == 0 {
                    tracing::warn!("the radio stopped taking samples mid-transmission: {e}");
                }
                self.failed += 1;
            }
        }
        Ok(())
    }
}

/// Put what is going out into what the receiver sees.
///
/// A half duplex radio hears nothing while it transmits, so the driver hands
/// its receive stream a noise floor and the spectrum is flat for the length
/// of the over. That is honest and useless: an operator wants to see their
/// own signal, and it is the only way to check without a second radio that
/// the transmission is where it was meant to be, is the width it should be,
/// and is being modulated at all.
///
/// So the transmitter's own samples arrive on the second input, shifted by
/// the difference between where it is transmitting and where the receiver is
/// tuned, and are summed into the span exactly as a real signal on that
/// frequency would arrive. The level is what the modulator produced, which is
/// not calibrated against anything: this is a monitor, not a measurement, and
/// a transmission on the waterfall is drawn in the same place a receiver
/// across the room would see it and not at the strength it would see it.
///
/// Only while the radio is deaf, which is what `enabled` says. A full duplex
/// radio hears its own transmission for real, and mirroring on top of that
/// would draw it twice.
pub struct TxMonitorNode {
    /// Where the transmitter is against the receiver's own centre.
    shift_hz: f64,
    enabled: bool,
    mixer: dsp::Mixer,
    rate: f64,
    scratch: Vec<C32>,
}

impl Default for TxMonitorNode {
    fn default() -> Self {
        Self {
            shift_hz: 0.0,
            enabled: false,
            mixer: dsp::Mixer::new(0.0, 1.0),
            rate: 0.0,
            scratch: Vec::new(),
        }
    }
}

impl TxMonitorNode {
    /// Whether what is being transmitted is drawn on the receiver's span.
    /// Set from the radio thread, which is the only thing that knows whether
    /// the receive stream has gone deaf for the over.
    pub fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

impl Node for TxMonitorNode {
    fn name(&self) -> &str {
        "tx_monitor"
    }



    fn num_inputs(&self) -> usize {
        2
    }

    /// The transmitter's input is allowed to be empty, and usually is: a
    /// receiver with no transmit chain still has a head, and this stage sits
    /// in front of it. Refusing the wire would drop the stage, and with it
    /// everything downstream of the head.
    fn optional_inputs(&self) -> bool {
        true
    }

    /// The whole point of this stage: what went out of the antenna, back onto
    /// what came in.
    fn joins_flows(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let span = inputs
            .first()
            .ok_or_else(|| common::Error::other("tx_monitor needs the span on its first input"))?;
        if span.spec.kind != PortKind::Iq {
            return Err(common::Error::other("tx_monitor needs IQ on its first input"));
        }
        self.rate = span.spec.rate;
        self.mixer.set_shift(self.shift_hz, self.rate.max(1.0));
        Ok(vec![span.spec])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(span) = inputs.first().and_then(|p| p.as_iq()) else {
            return Ok(());
        };
        let out = outputs[0].iq_mut();
        out.extend_from_slice(span);
        let sent = inputs.get(1).and_then(|p| p.as_iq()).unwrap_or(&[]);
        if !self.enabled || sent.is_empty() {
            return Ok(());
        }
        self.scratch.clear();
        self.mixer.process(sent, &mut self.scratch);
        for (dst, src) in out.iter_mut().zip(self.scratch.iter()) {
            *dst += *src;
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.scratch.clear();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(SHIFT_HZ, self.shift_hz, -30e6..=30e6)
                .label("Transmitting from centre")
                .unit("Hz"),
            Param::bool(ENABLED, self.enabled).label("Draw the transmission"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            SHIFT_HZ => {
                self.shift_hz = value.as_f64().unwrap_or(0.0);
                self.mixer.set_shift(self.shift_hz, self.rate.max(1.0));
            }
            ENABLED => self.enabled = value.as_bool().unwrap_or(false),
            _ => {
                return Err(common::Error::other(format!(
                    "tx_monitor: unknown parameter {name:?}"
                )))
            }
        }
        Ok(())
    }
}

/// Speech, resampled to whatever rate the radio is running at.
///
/// The microphone is not read by the radio thread and handed to the graph: it
/// is a node, so what goes on air is visible in the chain and can be tapped
/// between the microphone and the modulator. What it holds is an
/// [`audio::AudioSource`], which is the microphone in normal use and a known
/// waveform in a test.
///
/// The resampling is Catmull-Rom over the source's samples. A radio's rate is
/// hundreds of times the microphone's, so this is heavy interpolation rather
/// than a small ratio change: at 48 kHz in and 2 MS/s out, the images of a
/// 3 kHz tone land near 45 kHz, which cubic interpolation leaves about 70 dB
/// down and linear leaves about 48 dB down. A polyphase filter would do
/// better and costs a multiply-accumulate per output sample, which at 2 MS/s
/// is the expensive place to spend one.
/// Most the microphone can be driven into the limiter by, as a voltage
/// ratio: 20 dB. A radio's microphone amplifier has that much in hand
/// because the limiter is meant to be reached on every syllable, and a
/// UV-5R measured off air was deviating twice what the app did at a gain of
/// 1.7 on the same voice, with the limiter never touched.
pub const MIC_GAIN_MAX: f32 = 10.0;

pub struct MicNode {
    src: std::sync::Arc<dyn audio::AudioSource>,
    level: f32,
    /// Peak of the last block, before the gain, for a meter beside the key.
    peak: f32,
    /// The last block arrived already flat-topped: runs of samples sitting
    /// on one value at the block's own peak, which is a converter or a
    /// capture chain clipping before anything here ran. Nothing downstream
    /// can undo it and it sounds like a voice with the body taken out, so
    /// it is worth a warning where the meter is. An evening went on a
    /// microphone boost set 6 dB too high before this existed.
    clipped: bool,
    /// The band the audio is limited to, in hertz, and the filter that does
    /// it.
    ///
    /// Filtered here, at the microphone's rate, and not further down the
    /// chain: the modulator runs at the radio's rate, where a hundred taps
    /// cannot make a three kilohertz cutoff at all. A filter designed there
    /// rolls off from a few hundred hertz and speech through it sounds like
    /// a blanket over the microphone.
    band: (f64, f64),
    filter: Option<crate::RealFir>,
    /// Pre-emphasis time constant in microseconds, or zero for flat.
    ///
    /// Every FM voice receiver de-emphasises: it rolls the audio off at
    /// 6 dB an octave above a few hundred hertz, because its transmitter
    /// boosted it by the same before modulating, and the pair puts the
    /// FM noise triangle where the ear minds it least. Flat audio through
    /// that receiver comes out with its consonants 15 to 20 dB down, which
    /// is speech with the top taken off: a voice you can hear and not
    /// understand. 750 us is the land mobile and amateur figure.
    emphasis_us: f64,
    emphasis: Emphasis,
    /// The rate the filter was designed at, so a parameter change can build a
    /// new one without waiting for the graph to negotiate again.
    src_rate: f64,
    /// Output rate, from negotiation.
    rate: f64,
    /// Position between source samples, in source samples.
    phase: f64,
    /// The four samples the interpolator reads, oldest first.
    window: [f32; 4],
    /// Source samples not yet consumed.
    pending: std::collections::VecDeque<f32>,
    /// Output samples produced with nothing to produce them from, which is
    /// the microphone not keeping up.
    starved: u64,
}

impl MicNode {
    /// Speech, limited to `band` and levelled if `agc`.
    ///
    /// The band is the transmission's, not a preference: what leaves the
    /// modulator is as wide as the deviation plus the highest note it was
    /// given, so the audio limit is what keeps a transmission inside its
    /// channel.
    pub fn with_band(
        src: std::sync::Arc<dyn audio::AudioSource>,
        level: f32,
        band: (f64, f64),
    ) -> Self {
        Self {
            band,
            ..Self::new(src, level)
        }
    }

    /// Speech, at a level the operator sets against the meter.
    ///
    /// There is no levelling here on purpose. An AGC on a transmitter has
    /// nothing to level against between words, so it winds all the way up and
    /// puts the room on air at full deviation: what a listener hears is a
    /// gate opening onto hiss every time the talker pauses. A radio's
    /// microphone amplifier limits rather than levels, and until there is a
    /// limiter the honest control is the fader and the meter beside it.
    pub fn new(src: std::sync::Arc<dyn audio::AudioSource>, level: f32) -> Self {
        Self {
            src,
            level: level.clamp(0.0, MIC_GAIN_MAX),
            peak: 0.0,
            clipped: false,
            // Communications speech: from 400 Hz, which is where a handheld's
            // own microphone chain starts and lower sounds muddy beside it,
            // out to 3.4 kHz, which is where intelligibility lives.
            band: (400.0, 3_400.0),
            filter: None,
            emphasis_us: 750.0,
            emphasis: Emphasis::default(),
            src_rate: 0.0,
            rate: 0.0,
            phase: 0.0,
            window: [0.0; 4],
            pending: std::collections::VecDeque::new(),
            starved: 0,
        }
    }

    /// Output samples that had no speech behind them.
    pub fn starved(&self) -> u64 {
        self.starved
    }

    /// What the microphone is putting in, before the gain: the number a meter
    /// on the strip shows, so an operator can see whether they are being
    /// heard without asking anybody.
    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// Whether the microphone's own signal is arriving clipped.
    pub fn input_clipped(&self) -> bool {
        self.clipped
    }

    /// Flat tops: the block's highest or lowest value held by a run of
    /// samples in a row, more than once. Speech never sits on one value for
    /// a quarter of a millisecond; a rail does. Each rail on its own, since
    /// a microphone overloads one side first.
    fn flat_topped(v: &[f32], peak: f32) -> bool {
        if peak < 0.1 || v.is_empty() {
            return false;
        }
        let hi = v.iter().copied().fold(f32::NEG_INFINITY, f32::max);
        let lo = v.iter().copied().fold(f32::INFINITY, f32::min);
        let tol = peak * 0.002;
        let runs_at = |rail: f32| {
            let (mut run, mut runs) = (0usize, 0usize);
            for &x in v {
                if (x - rail).abs() <= tol {
                    run += 1;
                    if run == 12 {
                        runs += 1;
                    }
                } else {
                    run = 0;
                }
            }
            runs
        };
        runs_at(hi) >= 2 || runs_at(lo) >= 2
    }

    /// Build the speech filter for the band and rate now set.
    ///
    /// Designed at the source's rate, which is what makes a sharp cutoff
    /// affordable: 255 taps at 48 kHz put the transition inside 200 Hz. The
    /// same design at the radio's rate cannot make a 3 kHz cutoff at all.
    fn design(&mut self) {
        let rate = self.src_rate;
        if rate <= 0.0 {
            return;
        }
        self.emphasis = Emphasis::new(rate, self.emphasis_us);
        let (lo, hi) = self.band;
        let hi = hi.min(rate * 0.45);
        let lo = lo.clamp(0.0, hi - 100.0);
        // A band open at both ends is no filter at all, and saying so with a
        // filter that passes everything costs 255 taps a sample.
        if lo <= 1.0 && hi >= rate * 0.44 {
            self.filter = None;
            return;
        }
        let taps = dsp::filter::design(
            dsp::filter::Response::Bandpass,
            255,
            rate,
            (lo + hi) / 2.0,
            hi - lo,
            60.0,
        );
        self.filter = Some(crate::RealFir::new(taps));
    }

    fn next_source(&mut self) -> bool {
        match self.pending.pop_front() {
            Some(v) => {
                self.window = [self.window[1], self.window[2], self.window[3], v];
                true
            }
            None => {
                // Silence rather than the last sample held: a held sample is
                // a DC offset, and on an FM carrier that is a steady
                // deviation for as long as the microphone is behind.
                self.window = [self.window[1], self.window[2], self.window[3], 0.0];
                self.starved += 1;
                false
            }
        }
    }
}

/// First-order pre-emphasis: a shelf rising from `1 / (2 pi tau)` and
/// levelling off 12 dB up.
///
/// `(1 + s tau) / (1 + s tau / 4)` through the bilinear transform, unity at
/// DC. At 750 us the corner is 212 Hz and the shelf turns over at 850 Hz,
/// so 3 kHz sits about 6 dB over 500 Hz.
///
/// Less than the textbook 6 dB an octave all the way to 3 kHz, and measured
/// against a UV-5R rather than the textbook. Twenty decibels of boost in
/// front of the limiter turned every glottal pulse into a click that hit
/// the clipper while the body of the vowel between them sat twenty
/// decibels down: off air, the app's voice was a train of clipped spikes
/// with silence between, and the radio's was a dense waveform. The clipper
/// then took the energy the receiver's de-emphasis would have turned back
/// into the vowel, which is hollow, buzzing speech. The handheld's own
/// transmitted spectrum falls about 6 dB from 600 Hz to 3 kHz on speech;
/// with this shelf the app's falls about the same.
#[derive(Clone, Copy, Debug, Default)]
struct Emphasis {
    b0: f32,
    b1: f32,
    a1: f32,
    x1: f32,
    y1: f32,
    on: bool,
}

impl Emphasis {
    fn new(rate: f64, tau_us: f64) -> Self {
        if tau_us <= 0.0 || rate <= 0.0 {
            return Self::default();
        }
        let k1 = 2.0 * rate * tau_us * 1e-6;
        let k2 = k1 / 4.0;
        let a0 = 1.0 + k2;
        Self {
            b0: ((1.0 + k1) / a0) as f32,
            b1: ((1.0 - k1) / a0) as f32,
            a1: ((1.0 - k2) / a0) as f32,
            x1: 0.0,
            y1: 0.0,
            on: true,
        }
    }

    fn process(&mut self, v: &mut [f32]) {
        if !self.on {
            return;
        }
        for x in v {
            let y = self.b0 * *x + self.b1 * self.x1 - self.a1 * self.y1;
            self.x1 = *x;
            self.y1 = y;
            *x = y;
        }
    }

    fn reset(&mut self) {
        self.x1 = 0.0;
        self.y1 = 0.0;
    }
}

/// Catmull-Rom through four points, at `t` in [0, 1] between the middle two.
fn cubic(p: [f32; 4], t: f32) -> f32 {
    let (a, b, c, d) = (p[0], p[1], p[2], p[3]);
    0.5 * ((2.0 * b)
        + (-a + c) * t
        + (2.0 * a - 5.0 * b + 4.0 * c - d) * t * t
        + (-a + 3.0 * b - 3.0 * c + d) * t * t * t)
}

impl Simple for MicNode {
    fn name(&self) -> &str {
        "mic"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Real {
            return Err(common::Error::other("mic runs on a real audio stream"));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other(
                "mic needs the rate it should produce at",
            ));
        }
        if self.src.rate() <= 0.0 {
            return Err(common::Error::other(
                "the microphone reports no sample rate",
            ));
        }
        self.rate = input.spec.rate;
        self.src_rate = self.src.rate();
        self.design();
        Ok(input.spec)
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(audio) = input.as_real() else {
            return Ok(());
        };
        let n = audio.len();
        if n == 0 {
            return Ok(());
        }
        let step = self.src.rate() / self.rate;
        // What this block needs, plus the interpolator's own lookahead.
        let want = (n as f64 * step).ceil() as usize + 4;
        self.pending.reserve(want);
        let mut got = Vec::with_capacity(want);
        self.src
            .take(&mut got, want.saturating_sub(self.pending.len()));
        // Measured before anything is done to it, so the meter shows what the
        // microphone heard rather than what the limiter made of it.
        self.peak = got.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        if !got.is_empty() {
            self.clipped = Self::flat_topped(&got, self.peak);
        }
        // The order a radio's microphone amplifier does it in: emphasis,
        // gain, the limiter, then the filter that takes the limiter's
        // harmonics out before they widen the transmission. The limiter is
        // a clipper at full deviation. Speech peaks stand ten decibels or
        // more over its average, so audio scaled to keep the peaks legal
        // deviates the carrier a third of the way on the average syllable
        // and the transmission is quiet; clipped, the average comes up and
        // the peaks stay where they were, which is what every voice radio
        // does and what one sounds like on the other end.
        self.emphasis.process(&mut got);
        for v in &mut got {
            *v = (*v * self.level).clamp(-1.0, 1.0);
        }
        if let Some(f) = &mut self.filter {
            f.process(&mut got);
        }
        self.pending.extend(got);

        let out = output.real_mut();
        out.reserve(n);
        for &passthrough in audio {
            while self.phase >= 1.0 {
                self.next_source();
                self.phase -= 1.0;
            }
            let v = cubic(self.window, self.phase as f32);
            out.push(passthrough + v.clamp(-1.0, 1.0));
            self.phase += step;
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.phase = 0.0;
        self.window = [0.0; 4];
        self.pending.clear();
        self.starved = 0;
        self.peak = 0.0;
        self.emphasis.reset();
        if let Some(f) = &mut self.filter {
            f.reset();
        }
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("level", self.level as f64, 0.0..=MIC_GAIN_MAX as f64)
                .label("Mic gain")
                .unit("x"),
            Param::float("emphasis_us", self.emphasis_us, 0.0..=1_000.0)
                .label("Pre-emphasis")
                .unit("us"),
            // The band is here rather than fixed because what sounds right
            // depends on the microphone, the voice and what is listening: a
            // telephone band is the safe default and not the only answer.
            Param::float("low_hz", self.band.0, 0.0..=1_000.0)
                .label("Mic low cut")
                .unit("Hz"),
            Param::float("high_hz", self.band.1, 1_000.0..=20_000.0)
                .label("Mic high cut")
                .unit("Hz"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            "level" => {
                self.level = value
                    .as_f64()
                    .unwrap_or(1.0)
                    .clamp(0.0, MIC_GAIN_MAX as f64) as f32;
                Ok(())
            }
            "low_hz" => {
                self.band.0 = value.as_f64().unwrap_or(0.0).max(0.0);
                self.design();
                Ok(())
            }
            "high_hz" => {
                self.band.1 = value.as_f64().unwrap_or(3_400.0).max(500.0);
                self.design();
                Ok(())
            }
            "emphasis_us" => {
                self.emphasis_us = value.as_f64().unwrap_or(0.0).clamp(0.0, 1_000.0);
                self.design();
                Ok(())
            }
            _ => Err(common::Error::other(format!(
                "mic: unknown parameter {name:?}"
            ))),
        }
    }
}

#[cfg(test)]
mod mic_tests {
    use super::*;
    use std::sync::Arc;

    fn spec(rate: f64) -> StreamSpec {
        StreamSpec {
            kind: PortKind::Real,
            rate,
            center: common::Hz(145_500_000),
            bandwidth: 6_000.0,
            flow: Flow::Tx,
            ..Default::default()
        }
    }

    fn run(node: &mut MicNode, rate: f64, n: usize) -> Vec<f32> {
        let s = spec(rate);
        Simple::negotiate(
            node,
            &PortSpec {
                spec: s,
                latency: 0,
            },
        )
        .unwrap();
        let input = Payload::Real(vec![0.0; n]);
        let mut out = Payload::Real(Vec::new());
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let ins = [PortSpec {
            spec: s,
            latency: 0,
        }];
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(node, &input, &mut out, &mut ctx).unwrap();
        match out {
            Payload::Real(v) => v,
            _ => unreachable!(),
        }
    }

    #[test]
    fn speech_comes_out_at_the_rate_the_radio_wants_and_the_pitch_it_went_in_at() {
        // 48 kHz in, 480 kHz out: ten times, which is the shape of the real
        // ratio without the test taking a second to run.
        let mic_rate = 48_000.0;
        let out_rate = 480_000.0;
        let tone: Vec<f32> = (0..48_000)
            .map(|i| (std::f32::consts::TAU * 1_000.0 * i as f32 / mic_rate as f32).sin())
            .collect();
        let src = Arc::new(audio::Canned::new(tone, mic_rate, true));
        let mut node = MicNode::new(src, 1.0);
        let out = run(&mut node, out_rate, 48_000);

        assert_eq!(out.len(), 48_000);
        assert_eq!(
            node.starved(),
            0,
            "the source had plenty and was read short"
        );
        // Pitch, by zero crossings past the interpolator's first few samples.
        let seg = &out[100..];
        let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
        // The block is a tenth of a second, so one crossing either way is
        // 10 Hz: this asserts the pitch is unchanged, not that the estimator
        // is precise.
        let hz = crossings as f64 * out_rate / seg.len() as f64;
        assert!((hz - 1_000.0).abs() < 15.0, "1 kHz came out at {hz:.0} Hz");
        let peak = seg.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(
            (peak - 1.0).abs() < 0.05,
            "level changed on the way through: {peak}"
        );
    }

    fn tone_peak(hz: f32, amp: f32, gain: f32, emphasis_us: f64) -> f32 {
        let tone: Vec<f32> = (0..48_000)
            .map(|i| amp * (std::f32::consts::TAU * hz * i as f32 / 48_000.0).sin())
            .collect();
        let src = Arc::new(audio::Canned::new(tone, 48_000.0, true));
        let mut node = MicNode::new(src, gain);
        node.emphasis_us = emphasis_us;
        let out = run(&mut node, 48_000.0, 8_000);
        out[4_000..].iter().fold(0.0f32, |m, v| m.max(v.abs()))
    }

    /// The top of the speech band goes out a few decibels over the bottom,
    /// which is what a receiver's de-emphasis takes back out; flat, it would
    /// arrive with the consonants missing, and boosted by the full twenty
    /// the limiter ate the vowels.
    #[test]
    fn speech_is_pre_emphasised_the_way_a_receiver_expects() {
        let low = tone_peak(800.0, 0.01, 1.0, 750.0);
        let high = tone_peak(2_500.0, 0.01, 1.0, 750.0);
        let db = 20.0 * (high / low).log10();
        assert!(
            (1.5..=6.0).contains(&db),
            "2.5 kHz sits {db:.1} dB over 800 Hz"
        );
        // And the octave under 630 Hz is well down, the way a handheld's is.
        let bass = tone_peak(300.0, 0.01, 1.0, 750.0);
        let bass_db = 20.0 * (bass / low).log10();
        assert!(
            bass_db < -8.0,
            "300 Hz is only {bass_db:.1} dB under 800 Hz"
        );
        let flat_low = tone_peak(800.0, 0.01, 1.0, 0.0);
        let flat_high = tone_peak(2_500.0, 0.01, 1.0, 0.0);
        let flat = 20.0 * (flat_high / flat_low).log10();
        assert!(
            flat.abs() < 1.5,
            "with emphasis off the band tilts {flat:.1} dB"
        );
    }

    /// Driven hard, the audio limits at full deviation and the limiter's
    /// harmonics stay inside the band: what leaves is a clipped tone, not a
    /// square wave.
    #[test]
    fn the_limiter_holds_full_deviation_and_the_filter_cleans_up_after_it() {
        let peak = tone_peak(1_000.0, 0.5, 3.0, 0.0);
        assert!(
            peak <= 1.05 && peak > 0.9,
            "overdriven audio came out at {peak}"
        );
        // A hard-clipped 1 kHz tone has its third harmonic 10 dB down; the
        // 3.4 kHz filter leaves a fundamental with a little third in it, so
        // the waveform's zero crossings are still 2000 a second.
        let tone: Vec<f32> = (0..48_000)
            .map(|i| 0.5 * (std::f32::consts::TAU * 1_000.0 * i as f32 / 48_000.0).sin())
            .collect();
        let src = Arc::new(audio::Canned::new(tone, 48_000.0, true));
        let mut node = MicNode::new(src, 3.0);
        node.emphasis_us = 0.0;
        let out = run(&mut node, 48_000.0, 48_000);
        let seg = &out[4_000..];
        let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count() as f64;
        let hz = crossings * 48_000.0 / seg.len() as f64;
        assert!(
            (hz - 1_000.0).abs() < 15.0,
            "clipping put the tone at {hz:.0} Hz"
        );
        // Above the band there is nothing: the fifth harmonic at 5 kHz is
        // what the clipper made and the filter took away.
        let bin = |f: f64| {
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (i, v) in seg.iter().enumerate() {
                let p = std::f64::consts::TAU * f * i as f64 / 48_000.0;
                re += *v as f64 * p.cos();
                im += *v as f64 * p.sin();
            }
            (re * re + im * im).sqrt() / seg.len() as f64
        };
        let fifth_db = 20.0 * (bin(5_000.0) / bin(1_000.0)).log10();
        assert!(
            fifth_db < -40.0,
            "the fifth harmonic is only {fifth_db:.0} dB down"
        );
    }

    /// A capture already clipping is reported as such; clean speech is not.
    #[test]
    fn a_flat_topped_input_is_called_clipped() {
        let clean: Vec<f32> = (0..4800)
            .map(|i| 0.4 * (std::f32::consts::TAU * 150.0 * i as f32 / 48_000.0).sin())
            .collect();
        let clipped: Vec<f32> = clean.iter().map(|v| v.clamp(-0.25, 0.4)).collect();
        assert!(!MicNode::flat_topped(&clean, 0.4));
        assert!(MicNode::flat_topped(&clipped, 0.4));
        let src = Arc::new(audio::Canned::new(clipped, 48_000.0, true));
        let mut node = MicNode::new(src, 1.0);
        let _ = run(&mut node, 48_000.0, 4_000);
        assert!(node.input_clipped());
    }

    #[test]
    fn a_microphone_that_falls_behind_transmits_silence_not_a_held_sample() {
        // A held sample is a DC offset, and on an FM carrier that is a steady
        // deviation for as long as the microphone is behind: a tone off
        // frequency rather than a gap.
        let src = Arc::new(audio::Canned::new(vec![1.0; 100], 48_000.0, false));
        let mut node = MicNode::new(src, 1.0);
        let out = run(&mut node, 48_000.0, 4_000);
        assert!(node.starved() > 0, "the source ran out and nothing noticed");
        let tail = &out[out.len() - 500..];
        assert!(
            tail.iter().all(|v| v.abs() < 1e-6),
            "the tail is not silent"
        );
    }

    #[test]
    fn the_level_control_scales_what_goes_on_air() {
        // A tone inside the speech band rather than a constant: the stage
        // filters what it passes, and a constant is exactly what a
        // transmitter must not put on a carrier.
        let tone: Vec<f32> = (0..48_000)
            .map(|i| 0.5 * (std::f32::consts::TAU * 1_000.0 * i as f32 / 48_000.0).sin())
            .collect();
        let src = Arc::new(audio::Canned::new(tone, 48_000.0, true));
        let mut node = MicNode::new(src, 2.0);
        let out = run(&mut node, 48_000.0, 4_000);
        let peak = out[1_000..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(
            (peak - 1.0).abs() < 0.05,
            "0.5 at twice gain came out at {peak}"
        );
    }
}

/// The head of a transmit chain: a block of time, from a block of samples.
///
/// A transmitter has nothing upstream of it, but a graph node has an input,
/// and the thing the transmit side actually needs from the receiver is its
/// clock: one block of received time is one block of transmitted time, and
/// that is what keeps the two halves in step without either of them owning a
/// timer. So this takes the receiver's stream and emits the same number of
/// silent audio samples for the modulator's chain to fill.
pub struct TxClockNode {
    rate: f64,
}

impl Default for TxClockNode {
    fn default() -> Self {
        Self { rate: 0.0 }
    }
}

impl Simple for TxClockNode {
    fn name(&self) -> &str {
        "tx_clock"
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other(
                "tx_clock needs a stream to take its clock from",
            ));
        }
        self.rate = input.spec.rate;
        Ok(StreamSpec {
            kind: PortKind::Real,
            rate: self.rate,
            center: input.spec.center,
            // What a voice occupies, which is what the modulator checks its
            // deviation against. The stages between here and it narrow this
            // no further, so it is stated once.
            bandwidth: 6_000.0,
            channels: 1,
            flow: Flow::Tx,
            domain: pipeline::port::Domain::Baseband,
        })
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let n = input.len();
        output.real_mut().resize(n, 0.0);
        Ok(())
    }
}

/// The setting names these stages read, and the speed a keyer runs at when
/// nothing has said otherwise.
const WPM: &str = "wpm";
const HZ: &str = "hz";
const LEVEL: &str = "level";
const OFFSET_HZ: &str = "offset_hz";
const SHIFT_HZ: &str = "shift_hz";
const ENABLED: &str = "enabled";

/// A comfortable hand speed, and what a keyer starts at.
pub const DEFAULT_WPM: f32 = 20.0;

pub const TX_CLOCK: StageDesc = StageDesc {
    name: "tx_clock",
    summary: "Take the receiver's clock and give a transmit chain a \
              block of time to fill",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_tx_clock(_s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(TxClockNode::default()))
}

pub const TX_MONITOR: StageDesc = StageDesc {
    name: "tx_monitor",
    summary: "Draw what is being transmitted on the receiver's own span, \
              for the length of an over a half duplex radio cannot hear",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_tx_monitor(s: &Settings) -> Result<Box<dyn Node>> {
    let mut n = TxMonitorNode::default();
    Node::set_param(&mut n, SHIFT_HZ, ParamValue::Float(s.f64_or(SHIFT_HZ, 0.0)))?;
    n.set_enabled(s.bool_or(ENABLED, false));
    Ok(Box::new(n))
}

pub const TONE: StageDesc = StageDesc {
    name: "tone",
    summary: "A test tone, added to whatever is on the stream",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_tone(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(ToneNode::new(
        s.f64_or(HZ, 1_000.0),
        s.f64_or(LEVEL, 0.8) as f32,
    )))
}

pub const MORSE_TX: StageDesc = StageDesc {
    name: "morse_tx",
    summary: "Key text as Morse on a carrier, ready for a transmitter",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_morse_tx(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(MorseTxNode::new(
        s.f64_or(WPM, DEFAULT_WPM as f64) as f32,
        s.f64_or(OFFSET_HZ, 0.0),
    )))
}

pub const MORSE_KEY: StageDesc = StageDesc {
    name: "morse_key",
    summary: "Text to Morse mark and gap timings",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_morse_key(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(MorseKeyNode::new(
        s.f64_or(WPM, DEFAULT_WPM as f64) as f32,
    )))
}
