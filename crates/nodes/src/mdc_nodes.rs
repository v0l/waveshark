//! MDC-1200 as a graph node: an FM voice channel in, the unit id out.
//!
//! Two layers of modulation, the same shape as APRS. The channel is ordinary
//! narrowband FM, so the node mixes the channel down, filters it and
//! discriminates it; the burst is then in the *audio* as fast FSK at 1200
//! baud, which is MSK, so [`dsp::msk`] reads the data off the phase and
//! [`decode::mdc1200`] frames it.
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
pub use decode::mdc1200::read;
use dsp::msk::{MskConfig, MskDemod};
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::mdc::CHANNEL_WIDTH_HZ;
pub use identify::mdc::DEFAULT_HZ;
pub use identify::mdc::Mdc;
pub use identify::mdc::{AUDIO_HZ, DEVIATION_HZ};
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
    msk: MskDemod,
    framer: mdc1200::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    bits: Vec<bool>,
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
            msk: MskDemod::new(AUDIO_HZ, MskConfig::FFSK1200),
            framer: mdc1200::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            bits: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0)
                .keyed_as(common::Modulation::Msk),
            read: 0,
        }
    }

    /// Bursts whose CRC passed since the node was built.
    pub fn read(&self) -> u64 {
        self.read
    }

    /// Bursts that framed and then failed their CRC, the parity bytes
    /// having failed to repair them: a channel with MDC on it that never
    /// reads is a different fault from a quiet channel.
    pub fn refused(&self) -> u64 {
        self.framer.refused()
    }

    /// Bursts the parity bytes took back, which the CRC refused as they
    /// arrived: how much of what is heard is arriving damaged.
    pub fn repaired(&self) -> u64 {
        self.framer.repaired()
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
        if audio_rate < 4.0 * (MskConfig::FFSK1200.carrier_hz + MskConfig::FFSK1200.baud / 4.0) {
            return Err(common::Error::other("mdc1200 needs room for the 1800 Hz tone"));
        }
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.msk = MskDemod::new(audio_rate, MskConfig::FFSK1200);
        self.framer.reset();
        // Measured on the channel rather than on the span: a level taken
        // before the mixer is a level of the band.
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0)
            .keyed_as(common::Modulation::Msk);

        let mut out = i.spec.with_kind(PortKind::Packets);
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

        let mut bits = std::mem::take(&mut self.bits);
        bits.clear();
        let audio = std::mem::take(&mut self.audio);
        self.msk.process(&audio, &mut bits);
        self.audio = audio;
        for &bit in &bits {
            if let Some(info) = self.framer.push(bit) {
                self.read += 1;
                o.packets_mut().push(self.meter.packet_now(info.to_vec()));
            }
        }
        self.bits = bits;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.msk.reset();
        self.framer.reset();
        self.meter.reset();
    }
}

impl Protocol for Mdc {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
    }

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
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        read(bytes).map(|d| vec![d])
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
        let audio = dsp::afsk::modulate(&tones, audio_rate, dsp::afsk::FFSK1200);
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
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes().to_vec()));
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

        let d = read(&frames[0]).expect("a decode");
        assert_eq!((d.id, d.kind), ("mdc1200", "ptt_id"));
        // The radio transmitting names itself, which is what a device list
        // rows on; a radio emitted it, so nobody wrote anything.
        assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("1234"));
        assert_eq!(d.parties().0, Some("1234"));
        assert!(d.wrote().is_none());
    }

    /// A call alert names the radio being paged, not the one sending, and
    /// the row says which.
    #[test]
    fn a_call_alert_names_its_target() {
        let mut n = node(DEFAULT_HZ);
        let frames = run(&mut n, &keyed(0x63, 0x85, 0xABCD, 0.0, 0.0), DEFAULT_HZ);
        assert_eq!(frames.len(), 1);
        let d = read(&frames[0]).expect("a decode");
        // A call alert names the radio being paged, so it is the party
        // called and never the sender.
        assert_eq!((d.id, d.kind), ("mdc1200", "call_alert"));
        assert_eq!(d.parties(), (None, Some("ABCD")));
        assert!(d.subject.is_none(), "a call alert is not the sender's id");
    }

    /// Nobody is tuned exactly, and a burst off the dial still reads.
    /// Measured on this synthetic burst: every offset out to 6 kHz, the edge
    /// of the 12.5 kHz channel, reads whole, because a mistuned FM channel
    /// is a DC offset in the audio and the matched filter sits 1500 Hz away
    /// from it. The tone correlator this replaced was gone by 1.5 kHz.
    #[test]
    fn a_mistuned_burst_still_reads() {
        for offset in [-6_000.0, -2_500.0, -1_000.0, 0.0, 1_000.0, 2_500.0, 6_000.0] {
            let mut n = node(DEFAULT_HZ);
            let frames = run(&mut n, &keyed(0x01, 0x80, 0x0042, offset, 0.0), DEFAULT_HZ);
            assert_eq!(frames.len(), 1, "{offset} Hz off: {} bursts", frames.len());
            assert_eq!(mdc1200::parse(&frames[0]).unwrap().unit, 0x0042);
        }
    }

    /// A burst under noise still reads, and the sync word is never invented
    /// out of the noise alone. Measured on this synthetic burst: noise of
    /// twice the carrier amplitude across the 96 kS/s span still reads, and
    /// at two and a half times the block is refused rather than read as a
    /// wrong unit id. The tone correlator this replaced lost the burst at
    /// 1.5.
    #[test]
    fn a_burst_under_noise_reads_and_a_worse_one_reads_nothing() {
        let read = |noise| {
            let mut n = node(DEFAULT_HZ);
            let frames = run(&mut n, &keyed(0x01, 0x80, 0x0042, 0.0, noise), DEFAULT_HZ);
            (frames.len(), n.refused(), frames.first().and_then(|b| mdc1200::parse(b)))
        };
        let (count, refused, msg) = read(2.0);
        assert_eq!((count, refused), (1, 0));
        assert_eq!(msg.expect("a burst").unit, 0x0042);
        assert_eq!(read(2.5).0, 0, "a burst was invented at two and a half times the noise");
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
        assert_eq!(n.repaired(), 0, "the code voted a burst out of noise");
    }

    /// The carrier dropping out mid-burst costs the bits it covered and no
    /// more, which is what the interleaver and the parity bytes were put
    /// there for. Measured over dead-carrier gaps of 2, 4, 6, 8 and 10 ms at
    /// three places in the burst: all 15 read as the radio that sent them,
    /// 4 of them on the code's vote. Reading the tones and differencing them
    /// instead, which is what a dropout turns into an inversion of the rest
    /// of the block, read none of the 15.
    #[test]
    fn a_dead_carrier_mid_burst_still_reads_the_unit() {
        let (mut refused, mut repaired, mut read) = (0, 0, 0);
        let mut units = Vec::new();
        for ms in [2.0f64, 4.0, 6.0, 8.0, 10.0] {
            for at in [0.35f64, 0.5, 0.65] {
                let mut iq = keyed(0x01, 0x80, 0x0042, 0.0, 0.0);
                let start = (iq.len() as f64 * at) as usize;
                let end = (start + (RATE * ms / 1000.0) as usize).min(iq.len());
                for s in &mut iq[start..end] {
                    *s = C32::new(0.0, 0.0);
                }
                let mut n = node(DEFAULT_HZ);
                let frames = run(&mut n, &iq, DEFAULT_HZ);
                read += frames.len();
                units.extend(frames.iter().filter_map(|b| mdc1200::parse(b)).map(|m| m.unit));
                refused += n.refused();
                repaired += n.repaired();
            }
        }
        assert_eq!((read, refused, repaired), (15, 0, 4));
        assert_eq!(units.iter().filter(|&&u| u == 0x0042).count(), 15, "{units:?}");
    }
}
