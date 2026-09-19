//! MDC-1200 as a graph node: an FM voice channel in, the unit id out.
//!
//! Two layers of modulation, the same shape as APRS. The channel is ordinary
//! narrowband FM, so the node mixes the channel down, filters it and
//! discriminates it; the burst is then in the *audio* as fast FSK at 1200
//! baud, which [`dsp::afsk`] reads and [`decode::mdc1200`] frames.
//!
//! What reaches the bus is the seven information bytes, which carry their
//! own CRC: a row is published only where that CRC passed, so a unit id on
//! the packet list is a radio that really transmitted and not a fit to
//! noise.
//!
//! The identity is `radio-unit`, which is the namespace `ident` publishes a
//! DTMF PTT-ID under. They are the same fact about the same fleet sent two
//! ways, and a device heard both ways should be one device.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::mdc1200;
pub use decode::mdc1200::decoded;
use dsp::afsk::{AfskBits, AfskConfig, FFSK1200};
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::mdc::CHANNEL_WIDTH_HZ;
pub use identify::mdc::DEFAULT_HZ;
pub use identify::mdc::Mdc;
pub use identify::mdc::{AUDIO_HZ, DEVIATION_HZ};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

const CHANNEL_HZ: &str = "channel_hz";

pub struct MdcNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    bits: AfskBits,
    framer: mdc1200::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    symbols: Vec<dsp::afsk::Symbol>,
    meter: crate::FrameMeter,
    read: u64,
}

impl Default for MdcNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl MdcNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            bits: AfskBits::with_tones(AUDIO_HZ, FFSK1200, AfskConfig::default()),
            framer: mdc1200::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            symbols: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            read: 0,
        }
    }

    /// Bursts whose CRC passed since the node was built.
    pub fn read(&self) -> u64 {
        self.read
    }

    /// Bursts that framed and then failed their CRC: a channel with MDC on
    /// it that never reads is a different fault from a quiet channel.
    pub fn refused(&self) -> u64 {
        self.framer.refused()
    }
}

impl Simple for MdcNode {
    fn name(&self) -> &str {
        "mdc1200"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("mdc1200 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("mdc1200 needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        if audio_rate < 4.0 * FFSK1200.space_hz {
            return Err(common::Error::other("mdc1200 needs room for the 1800 Hz tone"));
        }
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.bits = AfskBits::with_tones(audio_rate, FFSK1200, AfskConfig::default());
        self.framer.reset();
        // Measured on the channel rather than on the span: a level taken
        // before the mixer is a level of the band.
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0);

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(CHANNEL_HZ, self.channel_hz, 1e5..=1e10).unit("Hz").label("Channel")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => {
                self.channel_hz = v.as_f64().unwrap_or(self.channel_hz);
                Ok(())
            }
            _ => Err(common::Error::other(format!("mdc1200: unknown parameter {name:?}"))),
        }
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = vec![("read".into(), self.read.to_string())];
        if self.framer.refused() > 0 {
            out.push(("refused".into(), self.framer.refused().to_string()));
        }
        out
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);

        let mut symbols = std::mem::take(&mut self.symbols);
        symbols.clear();
        let audio = std::mem::take(&mut self.audio);
        self.bits.process(&audio, &mut symbols);
        self.audio = audio;
        for sym in &symbols {
            // A quiet channel still produces symbols, and clocking those
            // into the framer is how a sync word gets invented.
            if sym.quiet {
                continue;
            }
            // The tone says whether the data bit changed: mark for no
            // change, space for a change.
            if let Some(info) = self.framer.push(!sym.mark) {
                self.read += 1;
                o.frames_mut().push(self.meter.frame(info.to_vec()));
            }
        }
        self.symbols = symbols;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.bits.reset();
        self.framer.reset();
        self.meter.reset();
    }
}

impl Protocol for Mdc {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }

    /// The seven bytes carry a CRC over four of them, and nothing else on
    /// the bus is seven bytes that pass it: the frame identifies itself
    /// without being told where it was heard, which matters because MDC
    /// rides any FM channel anybody points it at.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} MDC", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "MDC".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "mdc1200",
    summary: "One FM channel: the data burst a Motorola radio sends its unit id in",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(MdcNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    const RATE: f64 = 96_000.0;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A burst keyed onto the channel: the tones, then FM modulated, with
    /// `offset` hertz of mistuning and `noise` of added noise.
    fn keyed(op: u8, arg: u8, unit: u16, offset: f64, noise: f32) -> Vec<C32> {
        let tones = mdc1200::encode_tones(op, arg, unit, 0x00, 3);
        let audio_rate = 24_000.0;
        let audio = dsp::afsk::modulate(&tones, audio_rate, FFSK1200);
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let per = (RATE / audio_rate) as usize;
        let mut iq = Vec::with_capacity(audio.len() * per);
        let mut phase = 0.0f64;
        for &a in &audio {
            for _ in 0..per {
                phase += std::f64::consts::TAU * (f64::from(a) * DEVIATION_HZ + offset) / RATE;
                iq.push(C32::new(
                    phase.cos() as f32 + noise * rng(),
                    phase.sin() as f32 + noise * rng(),
                ));
            }
        }
        iq
    }

    fn run(node: &mut MdcNode, iq: &[C32], center: f64) -> Vec<Vec<u8>> {
        let ins = [spec(RATE, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in iq.chunks(4096) {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes));
            }
        }
        frames
    }

    fn node(center: f64) -> MdcNode {
        let mut n = MdcNode::new(center);
        n.negotiate(&spec(RATE, center)).unwrap();
        n
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = MdcNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 160_000_000.0)).is_err());
        assert!(n.negotiate(&spec(8_000.0, DEFAULT_HZ)).is_err());
    }

    /// The whole path on synthetic RF: a burst keyed onto an FM channel,
    /// into the node, out as the radio that sent it.
    #[test]
    fn a_keyed_burst_is_read_back_as_the_unit_that_sent_it() {
        let mut n = node(DEFAULT_HZ);
        let frames = run(&mut n, &keyed(0x01, 0x80, 0x1234, 0.0, 0.0), DEFAULT_HZ);
        assert_eq!(frames.len(), 1, "{} bursts off the air", frames.len());
        assert_eq!(n.read(), 1);
        assert_eq!(n.refused(), 0);

        let d = decoded(&frames[0], Hz(DEFAULT_HZ as u64)).expect("a decode");
        assert_eq!(d.protocol, "MDC-1200");
        assert_eq!(d.field("unit"), Some(&common::Value::Text("1234".into())));
        assert_eq!(d.field("operation"), Some(&common::Value::Text("PTT-ID".into())));
        assert_eq!(d.crc_ok, Some(true));
        assert!(!d.written, "a radio emitted it, nobody wrote it");
        assert_eq!(d.identity.as_ref().map(|i| i.id.as_str()), Some("1234"));
    }

    /// A call alert names the radio being paged, not the one sending, and
    /// the row says which.
    #[test]
    fn a_call_alert_names_its_target() {
        let mut n = node(DEFAULT_HZ);
        let frames = run(&mut n, &keyed(0x63, 0x85, 0xABCD, 0.0, 0.0), DEFAULT_HZ);
        assert_eq!(frames.len(), 1);
        let d = decoded(&frames[0], Hz(DEFAULT_HZ as u64)).expect("a decode");
        assert_eq!(d.field("target"), Some(&common::Value::Text("ABCD".into())));
        assert_eq!(d.field("unit"), None, "a call alert is not the sender's id");
    }

    /// Nobody is tuned exactly, and a burst a little off the dial still
    /// reads. Measured on this synthetic burst: to 1 kHz out it reads whole,
    /// and at 1.5 kHz, which is more than half the narrowband deviation, it
    /// is gone.
    #[test]
    fn a_mistuned_burst_still_reads() {
        for offset in [-1_000.0, -500.0, 0.0, 500.0, 1_000.0] {
            let mut n = node(DEFAULT_HZ);
            let frames = run(&mut n, &keyed(0x01, 0x80, 0x0042, offset, 0.0), DEFAULT_HZ);
            assert_eq!(frames.len(), 1, "{offset} Hz off: {} bursts", frames.len());
            assert_eq!(mdc1200::parse(&frames[0]).unwrap().unit, 0x0042);
        }
    }

    /// A burst under noise still reads, and the sync word is never invented
    /// out of the noise alone. Measured on this synthetic burst: noise of
    /// 1.5 times the carrier amplitude across the 96 kS/s span still reads,
    /// and twice it reads nothing rather than reading a wrong unit id.
    #[test]
    fn a_burst_under_noise_reads_and_a_worse_one_reads_nothing() {
        let read = |noise| {
            let mut n = node(DEFAULT_HZ);
            let frames = run(&mut n, &keyed(0x01, 0x80, 0x0042, 0.0, noise), DEFAULT_HZ);
            (frames.len(), n.refused(), frames.first().and_then(|b| mdc1200::parse(b)))
        };
        let (count, refused, msg) = read(1.5);
        assert_eq!((count, refused), (1, 0));
        assert_eq!(msg.expect("a burst").unit, 0x0042);
        assert_eq!(read(2.0).0, 0, "a burst was invented at twice the noise");
    }

    /// Two minutes of noise produces nothing. The sync word and the CRC are
    /// what stand between a busy channel and invented unit ids.
    #[test]
    fn noise_produces_no_bursts() {
        let mut seed = 0xdead_beef_cafe_f00du64;
        let mut rng = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> =
            (0..(RATE * 120.0) as usize).map(|_| C32::new(rng() * 0.3, rng() * 0.3)).collect();
        let mut n = node(DEFAULT_HZ);
        let frames = run(&mut n, &iq, DEFAULT_HZ);
        assert_eq!(frames.len(), 0, "noise made {} bursts", frames.len());
        assert_eq!(n.read(), 0);
    }
}
