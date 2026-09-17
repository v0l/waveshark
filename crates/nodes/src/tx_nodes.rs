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
use common::pulse::Package;
use common::{C32, Result};
use pipeline::Graph;
use pipeline::graph::Topology;
use pipeline::node::{Node, NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec};
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
        Self { wpm: wpm.clamp(1.0, 60.0) }
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
        vec![Param::float(WPM, self.wpm as f64, 1.0..=60.0).label("Speed").unit("wpm")]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            WPM => {
                self.wpm = value.as_f64().unwrap_or(20.0).clamp(1.0, 60.0) as f32;
                Ok(())
            }
            _ => Err(common::Error::other(format!("morse_key: unknown parameter {name:?}"))),
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
        Self { inner: b.build().expect("morse_tx graph is fixed and acyclic") }
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
            return Err(common::Error::other("morse_tx needs the rate it should key at"));
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
        Err(common::Error::other(format!("morse_tx: unknown parameter {name:?}")))
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
        Self { hz: 1_000.0, level: 0.5, rate: 0.0, phase: 0.0 }
    }
}

impl ToneNode {
    pub fn new(hz: f64, level: f32) -> Self {
        Self { hz, level: level.clamp(0.0, 1.0), ..Self::default() }
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
            Param::float(HZ, self.hz, 20.0..=20_000.0).label("Tone").unit("Hz"),
            Param::float(LEVEL, self.level as f64, 0.0..=1.0).label("Level"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let v = value.as_f64().unwrap_or(0.0);
        match name {
            HZ => self.hz = v.max(0.0),
            LEVEL => self.level = v.clamp(0.0, 1.0) as f32,
            _ => return Err(common::Error::other(format!("tone: unknown parameter {name:?}"))),
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
    /// Where what went out goes to be drawn, which is the receiver's own
    /// span. Handed over rather than wired, so the transmitter can move to a
    /// thread of its own without the monitor losing sight of it.
    sent: Option<Sent>,
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
            sent: None,
        }
    }

    /// Where to leave what went to the antenna, for the monitor to draw.
    pub fn send_to(&mut self, sent: Sent) {
        self.sent = Some(sent);
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
            sent: None,
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

    /// Take the radio back without ending the over, for a chain being
    /// replaced by another that will go on transmitting.
    pub fn detach(&mut self) -> Option<Box<dyn common::TxStream>> {
        self.stream.take()
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

    /// What an operator has to know about a transmission in progress: how
    /// much has gone out, and how much of it was silence the radio sent
    /// because nothing was queued in time. A count that climbs is a
    /// transmission with holes in it, which nothing else on the screen
    /// would show.
    fn readings(&self) -> Vec<(String, String)> {
        let sent = self.written as f64 / self.rate.as_f64().max(1.0);
        let mut out = vec![("on air".into(), format!("{sent:.1} s"))];
        let idle = self.underruns();
        if idle > 0 {
            out.push(("gaps".into(), idle.to_string()));
        }
        if self.failed > 0 {
            out.push(("refused".into(), self.failed.to_string()));
        }
        out
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
        if let Some(sent) = &self.sent
            && let Ok(mut q) = sent.lock()
        {
            q.extend(iq.iter().copied());
        }
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
    /// What was taken off the queue this block.
    taken: Vec<C32>,
    /// What went to the antenna, waiting to be drawn. Handed over rather
    /// than wired, because the transmitter runs on a thread of its own and a
    /// wire cannot cross one.
    sent: Sent,
}

/// How much of the transmission may wait to be drawn. The transmitter holds
/// itself a fifth of a second ahead of the radio, so the queue sits at about
/// that much in the steady state; this is the point at which the receiver is
/// so far behind that catching up matters more than continuity.
const BACKLOG_S: f64 = 1.0;

/// Samples on their way from the transmitter to the receiver's own span.
pub type Sent = std::sync::Arc<std::sync::Mutex<std::collections::VecDeque<C32>>>;

impl Default for TxMonitorNode {
    fn default() -> Self {
        Self {
            shift_hz: 0.0,
            enabled: false,
            mixer: dsp::Mixer::new(0.0, 1.0),
            rate: 0.0,
            scratch: Vec::new(),
            taken: Vec::new(),
            sent: Sent::default(),
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

    /// Where the transmitter is against the receiver's centre.
    pub fn set_shift(&mut self, hz: f64) {
        self.shift_hz = hz;
        self.mixer.set_shift(hz, self.rate.max(1.0));
    }

    /// Where to put what went out, for whoever is transmitting.
    pub fn sent(&self) -> Sent {
        self.sent.clone()
    }
}

impl Node for TxMonitorNode {
    fn name(&self) -> &str {
        "tx_monitor"
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
        self.taken.clear();
        if let Ok(mut sent) = self.sent.lock() {
            if !self.enabled {
                // Nothing is going out, so nothing is waiting to be drawn.
                sent.clear();
            } else {
                // Whole blocks only, and a backlog rather than a trim. On a
                // half duplex radio this is not a picture of the
                // transmission, it is the only thing the receiver is given,
                // and a decoder reads it: every sample thrown away and every
                // part filled block is a splice in the middle of a symbol,
                // which costs the lock and takes a super frame to get back.
                // The transmitter runs a fifth of a second ahead of the
                // radio to keep the stream fed, so a queue trimmed to a
                // block or two is trimmed on every single block.
                let keep = ((self.rate * BACKLOG_S) as usize).max(4 * span.len());
                let over = sent.len().saturating_sub(keep);
                sent.drain(..over);
                if sent.len() >= span.len() {
                    self.taken.extend(sent.drain(..span.len()));
                }
            }
        }
        if !self.enabled || self.taken.is_empty() {
            return Ok(());
        }
        self.scratch.clear();
        self.mixer.process(&self.taken, &mut self.scratch);
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
                )));
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
        Self { band, ..Self::new(src, level) }
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
            return Err(common::Error::other("mic needs the rate it should produce at"));
        }
        if self.src.rate() <= 0.0 {
            return Err(common::Error::other("the microphone reports no sample rate"));
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
        self.src.take(&mut got, want.saturating_sub(self.pending.len()));
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
            Param::float("low_hz", self.band.0, 0.0..=1_000.0).label("Mic low cut").unit("Hz"),
            Param::float("high_hz", self.band.1, 1_000.0..=20_000.0)
                .label("Mic high cut")
                .unit("Hz"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            "level" => {
                self.level = value.as_f64().unwrap_or(1.0).clamp(0.0, MIC_GAIN_MAX as f64) as f32;
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
            _ => Err(common::Error::other(format!("mic: unknown parameter {name:?}"))),
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
        Simple::negotiate(node, &PortSpec { spec: s, latency: 0 }).unwrap();
        let input = Payload::Real(vec![0.0; n]);
        let mut out = Payload::Real(Vec::new());
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let ins = [PortSpec { spec: s, latency: 0 }];
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
        assert_eq!(node.starved(), 0, "the source had plenty and was read short");
        // Pitch, by zero crossings past the interpolator's first few samples.
        let seg = &out[100..];
        let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
        // The block is a tenth of a second, so one crossing either way is
        // 10 Hz: this asserts the pitch is unchanged, not that the estimator
        // is precise.
        let hz = crossings as f64 * out_rate / seg.len() as f64;
        assert!((hz - 1_000.0).abs() < 15.0, "1 kHz came out at {hz:.0} Hz");
        let peak = seg.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!((peak - 1.0).abs() < 0.05, "level changed on the way through: {peak}");
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
        assert!((1.5..=6.0).contains(&db), "2.5 kHz sits {db:.1} dB over 800 Hz");
        // And the octave under 630 Hz is well down, the way a handheld's is.
        let bass = tone_peak(300.0, 0.01, 1.0, 750.0);
        let bass_db = 20.0 * (bass / low).log10();
        assert!(bass_db < -8.0, "300 Hz is only {bass_db:.1} dB under 800 Hz");
        let flat_low = tone_peak(800.0, 0.01, 1.0, 0.0);
        let flat_high = tone_peak(2_500.0, 0.01, 1.0, 0.0);
        let flat = 20.0 * (flat_high / flat_low).log10();
        assert!(flat.abs() < 1.5, "with emphasis off the band tilts {flat:.1} dB");
    }

    /// Driven hard, the audio limits at full deviation and the limiter's
    /// harmonics stay inside the band: what leaves is a clipped tone, not a
    /// square wave.
    #[test]
    fn the_limiter_holds_full_deviation_and_the_filter_cleans_up_after_it() {
        let peak = tone_peak(1_000.0, 0.5, 3.0, 0.0);
        assert!(peak <= 1.05 && peak > 0.9, "overdriven audio came out at {peak}");
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
        assert!((hz - 1_000.0).abs() < 15.0, "clipping put the tone at {hz:.0} Hz");
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
        assert!(fifth_db < -40.0, "the fifth harmonic is only {fifth_db:.0} dB down");
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
        assert!(tail.iter().all(|v| v.abs() < 1e-6), "the tail is not silent");
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
        assert!((peak - 1.0).abs() < 0.05, "0.5 at twice gain came out at {peak}");
    }
}

/// A transmission's bits, handed out at the rate they are keyed at.
///
/// Every data source in a transmit chain has the same problem: it holds a
/// whole transmission, and the chain asks it for one block of time at a
/// time. A POCSAG batch at 1200 baud is most of a second, and a source that
/// answered the first block with all of it would hand the radio a second of
/// samples every few milliseconds. So the block's length in seconds is what
/// decides how many bits leave, exactly as [`crate::dvbt_nodes::TsSourceNode`]
/// paces a multiplex against its bit rate.
///
/// Held here rather than in each protocol's node because the pacing is the
/// same for POCSAG, RTTY and BLE, and the only thing that differs is what
/// encoded the bits.
pub struct Keyer {
    baud: f64,
    bits: Vec<bool>,
    at: usize,
    /// Bits owed from the blocks already clocked, as a fraction so a baud
    /// that does not divide a block still comes out at the right rate.
    owed: f64,
    /// Bit periods of idle left before the transmission starts again.
    rest: f64,
    /// Idle between one pass and the next, in bit periods. A key held down
    /// sends again rather than falling silent, which is what a pager
    /// transmitter and a beacon both do.
    gap_bits: f64,
    /// Whether the rest between passes is no carrier rather than the lower
    /// tone held. A pager and a teleprinter idle on a tone and a receiver
    /// times itself on it; a short burst protocol stops, and a burst
    /// detector that never sees the channel go quiet never cuts a burst out
    /// of it.
    silent_rest: bool,
    passes: u64,
}

impl Keyer {
    pub fn new(baud: f64, gap_bits: f64) -> Self {
        Self {
            baud: baud.max(1.0),
            bits: Vec::new(),
            at: 0,
            owed: 0.0,
            rest: 0.0,
            gap_bits: gap_bits.max(0.0),
            silent_rest: false,
            passes: 0,
        }
    }

    /// Rest on no carrier rather than on the lower tone.
    pub fn resting_silent(mut self) -> Self {
        self.silent_rest = true;
        self
    }

    /// What to key from now on, from the top.
    pub fn load(&mut self, bits: Vec<bool>) {
        self.bits = bits;
        self.at = 0;
        self.owed = 0.0;
        self.rest = 0.0;
    }

    pub fn set_baud(&mut self, baud: f64) {
        self.baud = baud.max(1.0);
    }

    pub fn baud(&self) -> f64 {
        self.baud
    }

    /// Times the transmission has been sent whole, which is what says a key
    /// held down is repeating rather than stalled.
    pub fn passes(&self) -> u64 {
        self.passes
    }

    pub fn is_loaded(&self) -> bool {
        !self.bits.is_empty()
    }

    /// The timings for one block: `samples` of clock at `rate`.
    pub fn take(&mut self, samples: usize, rate: f64) -> Package {
        if self.bits.is_empty() || rate <= 0.0 {
            return Package::default();
        }
        self.owed += samples as f64 / rate * self.baud;
        let mut out: Vec<bool> = Vec::new();
        while self.owed >= 1.0 {
            self.owed -= 1.0;
            if self.rest > 0.0 {
                self.rest -= 1.0;
                // A silent rest is bit times that go by without being keyed
                // at all, so the modulator produces nothing for them and the
                // channel really is empty between bursts.
                if !self.silent_rest {
                    out.push(false);
                }
                continue;
            }
            out.push(self.bits[self.at]);
            self.at += 1;
            if self.at >= self.bits.len() {
                self.at = 0;
                self.passes += 1;
                self.rest = self.gap_bits;
            }
        }
        dsp::pulse::keyed(&out, self.baud)
    }
}

/// Run a protocol's declared transmit chain for `seconds` and collect what
/// would go to the antenna.
///
/// Built through the registry from [`crate::protocol::Protocol::transmit`],
/// so a round trip test proves the stage names and settings a protocol
/// declares are the ones that exist and negotiate, not a chain the test
/// assembled for itself.
#[cfg(test)]
pub(crate) fn transmit_for(
    p: &dyn crate::protocol::Protocol,
    rate: f64,
    center: common::Hz,
    seconds: f64,
    set: &[(&str, ParamValue)],
) -> Vec<C32> {
    let mut tx = p.transmit().unwrap_or_else(|| panic!("{} does not transmit", p.id()));
    // What an operator types into the source's card before keying.
    for (k, v) in set {
        tx.source.settings.insert((*k).into(), v.clone());
    }
    let clock = StreamSpec {
        kind: PortKind::Real,
        rate,
        center,
        channels: 1,
        flow: Flow::Tx,
        domain: Domain::Baseband,
        ..Default::default()
    };
    let mut g = crate::build_chain(clock, &[tx.source, tx.modulator], &crate::registry())
        .unwrap_or_else(|e| panic!("{}: {e}", p.id()));
    let block = 4_096;
    let blocks = (seconds * rate / block as f64).ceil() as usize;
    let mut out = Vec::with_capacity(blocks * block);
    for _ in 0..blocks {
        {
            let buf = g.input_buf();
            buf.clear();
            buf.real_mut().resize(block, 0.0);
        }
        g.run().expect("the transmit chain runs");
        let block_out = g.output().as_iq().unwrap_or(&[]).to_vec();
        // A block that keyed nothing at all is the stage resting on no
        // carrier, and on the air that is a block of time with no signal in
        // it. Filling it keeps the bursts where the clock put them, which is
        // what a detector at the other end cuts them out of. A block that
        // keyed something short is left as it is: padding inside a keyed run
        // would put a hole in the carrier.
        match block_out.is_empty() {
            true => out.resize(out.len() + block, C32::new(0.0, 0.0)),
            false => out.extend_from_slice(&block_out),
        }
    }
    out
}

/// What the speaker is playing, in dBFS, for whatever has to decide about
/// the microphone.
///
/// Shared rather than wired for the same reason the monitor's queue is: the
/// speaker is a stage in the receiver's graph and this is read on the
/// transmitter's thread, and a wire cannot cross one.
pub type Heard = std::sync::Arc<std::sync::atomic::AtomicU32>;

/// Key the transmitter from the level of the voice.
///
/// The decision is [`dsp::vox::Vox`]; what this adds is where it sits. The
/// stage is between the microphone and the modulator, so the level it tests
/// is the audio that would go on air, gain, limiter and speech filter
/// included: a threshold set against the meter is set against the same
/// number the decision is made on.
///
/// It passes the audio through untouched and keys nothing itself. What the
/// key does is the radio's business, and this says only whether it should be
/// down; see `crate::transmit` and `Radio::vox`.
pub struct VoxNode {
    vox: dsp::vox::Vox,
    threshold: f32,
    tail_ms: f64,
    anti_trip: bool,
    /// Length and pitch of the courtesy tone sent at the end of an over, or
    /// zero length for none. On a channel with no squelch tail the far end
    /// has nothing else to tell it the over finished.
    roger_ms: f64,
    roger_hz: f64,
    rate: f64,
    heard: Heard,
    open: bool,
    /// Samples of roger beep still to send. The key is held down for them,
    /// since a beep sent after the carrier drops is not sent at all.
    beep: usize,
    /// Whether this block carried any of the beep, which is what holds the
    /// key down over the block that finishes it: the samples are queued
    /// before anybody asks again, and unkeying on the same block cuts the
    /// tone off in the radio's own buffer.
    beeping: bool,
    phase: f64,
}

impl Default for VoxNode {
    fn default() -> Self {
        Self::new(DEFAULT_VOX_THRESHOLD, DEFAULT_VOX_TAIL_MS)
    }
}

impl VoxNode {
    pub fn new(threshold: f32, tail_ms: f64) -> Self {
        Self {
            // Rebuilt at negotiation, where the rate the tail is counted in
            // is known.
            vox: dsp::vox::Vox::new(1.0, threshold, tail_ms),
            threshold,
            tail_ms,
            anti_trip: true,
            roger_ms: 0.0,
            roger_hz: 1_000.0,
            rate: 0.0,
            heard: Heard::default(),
            open: false,
            beep: 0,
            beeping: false,
            phase: 0.0,
        }
    }

    /// Where to read what the speaker is playing, for anti-trip.
    pub fn watch(&mut self, heard: Heard) {
        self.heard = heard;
    }

    pub fn heard(&self) -> Heard {
        self.heard.clone()
    }

    /// Whether the key should be down: the voice, or the beep that ends it.
    pub fn is_open(&self) -> bool {
        self.open || self.beeping
    }

    /// The level the decision is being made on, as an amplitude in 0..1, so
    /// the meter beside the threshold shows the number being compared.
    pub fn level(&self) -> f32 {
        db_amplitude(self.vox.level_db())
    }

    /// Whether the key is being held up because the receiver is playing
    /// something.
    pub fn held_off(&self) -> bool {
        self.vox.is_held()
    }

    fn heard_db(&self) -> f32 {
        let a = f32::from_bits(self.heard.load(std::sync::atomic::Ordering::Relaxed));
        match a > 0.0 {
            true => 20.0 * a.log10(),
            false => f32::NEG_INFINITY,
        }
    }
}

fn db_amplitude(db: f32) -> f32 {
    match db.is_finite() {
        true => 10f32.powf(db / 20.0).clamp(0.0, 1.0),
        false => 0.0,
    }
}

impl Simple for VoxNode {
    fn name(&self) -> &str {
        "vox"
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            (
                "key".into(),
                match (self.is_open(), self.held_off()) {
                    (true, _) => "down".into(),
                    (false, true) => "held off".to_string(),
                    (false, false) => "up".into(),
                },
            ),
            ("level".into(), format!("{:.0} dB", self.vox.level_db())),
        ]
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Real {
            return Err(common::Error::other("vox runs on a real audio stream"));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("vox needs the rate its tail is counted in"));
        }
        self.rate = input.spec.rate;
        self.vox = dsp::vox::Vox::new(self.rate, self.threshold, self.tail_ms);
        self.vox.set_anti_trip(self.anti_trip);
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
        if audio.is_empty() {
            return Ok(());
        }
        let was = self.open;
        self.open = self.vox.update(audio, self.heard_db(), audio.len());
        if was && !self.open && self.roger_ms > 0.0 {
            self.beep = (self.rate * self.roger_ms / 1000.0) as usize;
            self.phase = 0.0;
        }
        let out = output.real_mut();
        out.extend_from_slice(audio);
        self.beeping = self.beep > 0;
        if self.beep == 0 {
            return Ok(());
        }
        // The beep replaces the audio rather than adding to it: what it is
        // laid over is the tail of an over that has already finished, and a
        // courtesy tone mixed with the last syllable is neither.
        let n = self.beep.min(out.len());
        let step = std::f64::consts::TAU * self.roger_hz / self.rate.max(1.0);
        for s in out.iter_mut().take(n) {
            *s = 0.5 * self.phase.sin() as f32;
            self.phase += step;
        }
        self.beep -= n;
        Ok(())
    }

    fn reset(&mut self) {
        self.vox.reset();
        self.open = false;
        self.beep = 0;
        self.beeping = false;
        self.phase = 0.0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float(THRESHOLD, self.threshold as f64, 0.0..=1.0).label("Vox threshold"),
            Param::float(TAIL_MS, self.tail_ms, 0.0..=5_000.0).label("Vox tail").unit("ms"),
            Param::bool(ANTI_TRIP, self.anti_trip).label("Ignore the speaker"),
            Param::float(ROGER_MS, self.roger_ms, 0.0..=1_000.0).label("Roger beep").unit("ms"),
            Param::float(ROGER_HZ, self.roger_hz, 300.0..=3_000.0)
                .label("Roger beep pitch")
                .unit("Hz"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            THRESHOLD => {
                self.threshold = value.as_f64().unwrap_or(0.1).clamp(0.0, 1.0) as f32;
                self.vox.set_threshold(self.threshold);
            }
            TAIL_MS => {
                self.tail_ms = value.as_f64().unwrap_or(0.0).clamp(0.0, 5_000.0);
                self.vox.set_tail_ms(self.rate.max(1.0), self.tail_ms);
            }
            ANTI_TRIP => {
                self.anti_trip = value.as_bool().unwrap_or(true);
                self.vox.set_anti_trip(self.anti_trip);
            }
            ROGER_MS => self.roger_ms = value.as_f64().unwrap_or(0.0).clamp(0.0, 1_000.0),
            ROGER_HZ => self.roger_hz = value.as_f64().unwrap_or(1_000.0).clamp(300.0, 3_000.0),
            _ => return Err(common::Error::other(format!("vox: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

#[cfg(test)]
mod vox_tests {
    use super::*;

    const RATE: f64 = 48_000.0;
    const BLOCK: usize = 960;

    fn run(node: &mut VoxNode, amp: f32, blocks: usize) -> (usize, Vec<f32>) {
        let spec = StreamSpec {
            kind: PortKind::Real,
            rate: RATE,
            flow: Flow::Tx,
            bandwidth: 6_000.0,
            ..Default::default()
        };
        if node.rate <= 0.0 {
            Simple::negotiate(node, &PortSpec { spec, latency: 0 }).unwrap();
        }
        let block: Vec<f32> = (0..BLOCK)
            .map(|i| amp * (std::f32::consts::TAU * 400.0 * i as f32 / RATE as f32).sin())
            .collect();
        let mut down = 0;
        let mut last = Vec::new();
        for _ in 0..blocks {
            let input = Payload::Real(block.clone());
            let mut out = Payload::Real(Vec::new());
            let (mut ev, mut tg) = (Vec::new(), Vec::new());
            let ins = [PortSpec { spec, latency: 0 }];
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(node, &input, &mut out, &mut ctx).unwrap();
            if node.is_open() {
                down += 1;
            }
            let Payload::Real(v) = out else { unreachable!() };
            last = v;
        }
        (down, last)
    }

    /// Speech keys it, the audio goes through unchanged, and silence lets it
    /// up after the tail.
    #[test]
    fn a_voice_keys_the_stage_and_the_audio_passes_through() {
        let mut node = VoxNode::new(0.1, 200.0);
        let (down, out) = run(&mut node, 0.5, 10);
        assert_eq!(down, 10, "the key was not down for every block of speech");
        assert_eq!(out.len(), BLOCK);
        let peak = out.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!((peak - 0.5).abs() < 0.01, "the audio came through at {peak}");
        // 200 ms of tail at 20 ms a block, less the block it fell in.
        let (down, _) = run(&mut node, 0.0, 40);
        assert_eq!(down, 9, "200 ms of tail, in blocks of 20 ms");
    }

    /// What the speaker is playing raises the threshold, so the station being
    /// listened to does not key the transmitter.
    #[test]
    fn the_speaker_holds_the_key_up() {
        let mut node = VoxNode::new(0.02, 200.0);
        // The speaker at half scale; the microphone hears it ten decibels
        // down, which is what `amp` is here.
        node.heard().store(0.5f32.to_bits(), std::sync::atomic::Ordering::Relaxed);
        let (down, _) = run(&mut node, 0.16, 20);
        assert_eq!(down, 0, "the receiver's own audio keyed the transmitter");
        // The speaker stops and the same level at the microphone keys it.
        node.heard().store(0.0f32.to_bits(), std::sync::atomic::Ordering::Relaxed);
        let (down, _) = run(&mut node, 0.16, 20);
        assert_eq!(down, 20, "a voice in a quiet room did not key");
    }

    /// The over ends with a courtesy tone, and the key stays down for it.
    #[test]
    fn the_roger_beep_is_sent_before_the_key_comes_up() {
        let mut node = VoxNode::new(0.1, 0.0);
        Node::set_param(&mut node, ROGER_MS, ParamValue::Float(100.0)).unwrap();
        let (down, _) = run(&mut node, 0.5, 5);
        assert_eq!(down, 5);
        // No tail, so the key comes up on the first silent block and the beep
        // holds it down for five more: 100 ms at 20 ms a block.
        let (down, out) = run(&mut node, 0.0, 20);
        assert_eq!(down, 5, "100 ms of roger beep, in blocks of 20 ms");
        assert_eq!(out.len(), BLOCK, "the beep changed the block's length");
        assert!(!node.is_open(), "the key stayed down after the beep");

        // And with no beep asked for, the key comes up at once.
        let mut bare = VoxNode::new(0.1, 0.0);
        let (down, _) = run(&mut bare, 0.5, 5);
        assert_eq!(down, 5);
        let (down, _) = run(&mut bare, 0.0, 20);
        assert_eq!(down, 0, "the key hung on with no beep to send");
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
            return Err(common::Error::other("tx_clock needs a stream to take its clock from"));
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
        output.real_mut().resize(input.len(), 0.0);
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
const THRESHOLD: &str = "threshold";
const TAIL_MS: &str = "tail_ms";
const ANTI_TRIP: &str = "anti_trip";
const ROGER_MS: &str = "roger_ms";
const ROGER_HZ: &str = "roger_hz";

/// Where a vox starts: a voice at a hand's width from the microphone reads
/// about a tenth of full scale through the speech filter, and a room with
/// nobody in it a hundredth.
pub const DEFAULT_VOX_THRESHOLD: f32 = 0.05;

/// And how long it holds after a voice stops. Long enough to cross the gap
/// between two sentences, short enough that the over does not end with a
/// second of the room.
pub const DEFAULT_VOX_TAIL_MS: f64 = 700.0;

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
    Ok(Box::new(ToneNode::new(s.f64_or(HZ, 1_000.0), s.f64_or(LEVEL, 0.8) as f32)))
}

pub const VOX: StageDesc = StageDesc {
    name: "vox",
    summary: "Key the transmitter while somebody is talking, ignoring what \
              the speaker is playing",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_vox(s: &Settings) -> Result<Box<dyn Node>> {
    let mut n = VoxNode::new(
        s.f64_or(THRESHOLD, DEFAULT_VOX_THRESHOLD as f64) as f32,
        s.f64_or(TAIL_MS, DEFAULT_VOX_TAIL_MS),
    );
    Node::set_param(&mut n, ANTI_TRIP, ParamValue::Bool(s.bool_or(ANTI_TRIP, true)))?;
    Node::set_param(&mut n, ROGER_MS, ParamValue::Float(s.f64_or(ROGER_MS, 0.0)))?;
    Node::set_param(&mut n, ROGER_HZ, ParamValue::Float(s.f64_or(ROGER_HZ, 1_000.0)))?;
    Ok(Box::new(n))
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
    Ok(Box::new(MorseKeyNode::new(s.f64_or(WPM, DEFAULT_WPM as f64) as f32)))
}

#[cfg(test)]
mod monitor_tests {
    use super::*;

    /// On a half duplex radio the loopback is not a picture of the
    /// transmission, it is the whole of what the receiver is given, and a
    /// decoder reads it. So it has to come back unbroken: one sample thrown
    /// away in the middle of an OFDM symbol costs the lock, and the picture
    /// comes and goes.
    ///
    /// The transmitter holds itself ahead of the radio, so the queue always
    /// carries a lead. Trimming that lead every block, which is what a queue
    /// kept to a block or two does, drops samples continuously while the
    /// spectrum still looks exactly right.
    #[test]
    fn the_loopback_comes_back_unbroken() {
        const BLOCK: usize = 4096;
        const RATE: f64 = 9_142_857.0;
        let mut node = TxMonitorNode::default();
        let spec = StreamSpec {
            kind: PortKind::Iq,
            rate: RATE,
            center: common::Hz(474_000_000),
            ..Default::default()
        };
        Node::negotiate(&mut node, &[PortSpec { spec, latency: 0 }]).unwrap();
        node.set_enabled(true);
        let sent = node.sent();

        // A counted stream, so a hole in what comes back is a jump in the
        // numbers rather than something to be eyeballed.
        let mut wrote = 0u32;
        let mut read: Vec<u32> = Vec::new();
        // The transmitter's lead: a fifth of a second before the first block.
        let lead = (RATE * 0.2) as u32;
        for _ in 0..lead {
            sent.lock().unwrap().push_back(C32::new(wrote as f32, 0.0));
            wrote += 1;
        }
        for _ in 0..64 {
            for _ in 0..BLOCK {
                sent.lock().unwrap().push_back(C32::new(wrote as f32, 0.0));
                wrote += 1;
            }
            let input = Payload::Iq(vec![C32::new(0.0, 0.0); BLOCK]);
            let mut out = Payload::Iq(Vec::new());
            let (mut ev, mut tg) = (Vec::new(), Vec::new());
            let ins = [PortSpec { spec, latency: 0 }];
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Node::process(&mut node, &[&input], std::slice::from_mut(&mut out), &mut ctx).unwrap();
            let Payload::Iq(v) = out else { unreachable!() };
            assert_eq!(v.len(), BLOCK, "the span is passed through whole");
            read.extend(v.iter().map(|s| s.re.round() as u32));
        }

        assert_eq!(read.len(), 64 * BLOCK, "every block carried the transmission");
        assert_eq!(read[0], 0, "the transmission is drawn from its start");
        for (i, v) in read.iter().enumerate() {
            assert_eq!(*v, i as u32, "a hole in the loopback at sample {i}");
        }
    }

    /// With the key up there is nothing going out, and what was queued
    /// behind it is stale: drawn later it would be a splice of an old over
    /// onto a new one.
    #[test]
    fn what_was_queued_behind_an_over_does_not_outlive_it() {
        let mut node = TxMonitorNode::default();
        let spec = StreamSpec { kind: PortKind::Iq, rate: 1_000_000.0, ..Default::default() };
        Node::negotiate(&mut node, &[PortSpec { spec, latency: 0 }]).unwrap();
        let sent = node.sent();
        sent.lock().unwrap().extend((0..4096).map(|i| C32::new(i as f32, 0.0)));
        let input = Payload::Iq(vec![C32::new(0.0, 0.0); 1024]);
        let mut out = Payload::Iq(Vec::new());
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let ins = [PortSpec { spec, latency: 0 }];
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Node::process(&mut node, &[&input], std::slice::from_mut(&mut out), &mut ctx).unwrap();
        assert!(sent.lock().unwrap().is_empty(), "the queue outlived the over");
        let Payload::Iq(v) = out else { unreachable!() };
        assert!(v.iter().all(|s| s.norm() == 0.0), "a stale over was drawn on the span");
    }
}
