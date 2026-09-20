//! Inmarsat STD-C as a graph node: one TDM channel in, packets out.
//!
//! The waveform is [`dsp::bpsk`] and the frames are [`decode::inmarsat::stdc`];
//! what is here is the wiring. A channel is mixed down, filtered to the few
//! kilohertz the 1200 symbol carrier occupies, demodulated, and handed to the
//! framer, which finds the unique word, undoes the interleaver, decodes the
//! code and unscrambles what is left.
//!
//! What reaches the bus is one packet at a time rather than the 640 byte
//! frame, because a frame is a queue of unrelated things: a bulletin board,
//! a channel assignment for one ship, a SafetyNET warning for everybody.
//! Only a packet whose own check bytes agree is sent on, so a row here is
//! something that was received rather than something that was guessed at.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::inmarsat::read;
use decode::inmarsat::{BAND_HZ, stdc};
use dsp::bpsk::{BpskConfig, BpskDemod};
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::stdc::CHANNEL_WIDTH_HZ;
pub use identify::stdc::DEFAULT_HZ;
pub use identify::stdc::FEED_HZ;
pub use identify::stdc::Stdc;
pub use identify::stdc::WORK_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub struct StdcNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    demod: BpskDemod,
    framer: stdc::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    soft: Vec<f32>,
    frames: Vec<stdc::Frame>,
    meter: crate::FrameMeter,
    packets: u64,
}

impl Default for StdcNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl StdcNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(WORK_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            demod: BpskDemod::new(WORK_HZ, BpskConfig::INMARSAT_C),
            framer: stdc::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            soft: Vec::new(),
            frames: Vec::new(),
            meter: crate::FrameMeter::new(WORK_HZ, channel_hz as u64, 10.0)
                .keyed_as(common::Modulation::Psk2),
            packets: 0,
        }
    }

    /// Packets whose check bytes agreed since the node was built.
    pub fn packets(&self) -> u64 {
        self.packets
    }
}

impl Simple for StdcNode {
    fn name(&self) -> &str {
        "stdc"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("stdc reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("stdc needs its channel inside the span"));
        }
        let factor = (rate / WORK_HZ).round().max(1.0) as usize;
        let work = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.demod = BpskDemod::new(work, BpskConfig::INMARSAT_C);
        self.framer.reset();
        // On the channel, not on the span: a 6 kHz channel in a megahertz of
        // band is a thousandth of the power.
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 10.0)
            .keyed_as(common::Modulation::Psk2);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);

        self.soft.clear();
        let mut soft = std::mem::take(&mut self.soft);
        self.demod.process(&self.narrow, &mut soft);
        self.frames.clear();
        let mut frames = std::mem::take(&mut self.frames);
        self.framer.process(&soft, &mut frames);
        self.soft = soft;

        let out = o.packets_mut();
        for f in &frames {
            for p in stdc::packets(&f.bytes) {
                if !p.check_ok {
                    continue;
                }
                self.packets += 1;
                out.push(self.meter.packet_now(p.bytes));
            }
        }
        self.frames = frames;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.framer.reset();
        self.meter.reset();
    }
}

impl Protocol for Stdc {
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

    /// The L-band downlinks to mobiles. Which channel a network control
    /// station is on depends on the satellite in view, so the band is the
    /// claim and the scanner table names the carriers inside it.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} STD-C", hz / 1e6)
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND_HZ.1 - BAND_HZ.0) as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(BAND_HZ.0..BAND_HZ.1).contains(&hz) || bytes.len() < 3 {
            return None;
        }
        // A signal unit is an Aero frame and is exactly twelve bytes; an
        // STD-C packet says its own length in its first byte or two.
        if bytes.len() == decode::inmarsat::aero::SU_BYTES {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "stdc",
    summary: "One Inmarsat STD-C TDM: 1200 baud BPSK, EGC and SafetyNET",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(StdcNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::media;
    use common::{C32, Hz};

    /// Run a channel through the node and collect what reached the bus.
    fn heard(n: &mut StdcNode, rate: f64, iq: &[C32]) -> Vec<Vec<u8>> {
        let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(DEFAULT_HZ as u64)), latency: 0 };
        n.negotiate(&spec).expect("a channel");
        let ins = [spec];
        let tags = Vec::new();
        let mut got = Vec::new();
        for chunk in iq.chunks(16_384) {
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            n.process(&Payload::Iq(chunk.to_vec()), &mut out, &mut ctx).expect("read");
            if let Payload::Packets(f) = out {
                got.extend(f.into_iter().map(|x| x.bytes().to_vec()));
            }
        }
        got
    }

    #[test]
    fn the_channel_has_to_be_inside_the_span() {
        let mut n = StdcNode::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(200_000.0, Hz(1_530_000_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let near = PortSpec { spec: StreamSpec::iq(200_000.0, Hz(1_541_400_000)), latency: 0 };
        let out = n.negotiate(&near).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Packets);
        assert_eq!(out.center, Hz(DEFAULT_HZ as u64));
    }

    /// One frame of packets, keyed as BPSK at the offset a tuner leaves and
    /// read back off the air by the node: the whole path, symbols included.
    #[test]
    fn a_keyed_frame_is_read_off_the_air() {
        let mut frame = vec![0u8; stdc::FRAME_BYTES];
        let mut board: Vec<u8> = vec![0x7D, 0x01, 0x12, 0x34, 0x01, 0x02, 0x03, 0x04];
        board.resize(14, 0);
        let check = stdc::check(&board);
        board[12..].copy_from_slice(&check);
        frame[..board.len()].copy_from_slice(&board);

        let symbols = stdc::encode_frame(&frame);
        // Two frames, because the framer is looking for a word it has not
        // seen before and the first symbols of a stream are its worst.
        let mut keyed: Vec<u8> = Vec::new();
        for _ in 0..2 {
            keyed.extend(symbols.iter().map(|s| u8::from(*s < 0.0)));
        }
        let rate = 38_400.0;
        let iq: Vec<C32> = dsp::bpsk::modulate(&keyed, rate, BpskConfig::INMARSAT_C, 300.0, 0.5);

        let mut n = StdcNode::new(DEFAULT_HZ);
        let frames = heard(&mut n, rate, &iq);
        assert_eq!(frames.len(), 1, "one packet out of two frames");
        assert_eq!(n.packets(), 1);
        let d = read(&frames[0]).expect("a row");
        assert_eq!((d.id, d.kind), ("inmarsat-c", "bulletin_board"));
    }

    /// An EGC broadcast is a machine addressing an area, so it carries text
    /// and fields and nobody wrote it.
    #[test]
    fn a_safetynet_warning_is_text_that_nobody_wrote() {
        let text = b"NAVAREA I 123/25 NORTH SEA UNLIT BUOY ADRIFT";
        let mut egc: Vec<u8> = vec![0xB1, 0, 0x31, 0x40, 0x00, 0x07, 0x01, 0x00];
        egc.extend([0x01, 0x02, 0x03, 0x04]);
        egc.extend(text.iter().copied());
        egc.extend([0, 0]);
        egc[1] = (egc.len() - 2) as u8;
        let check = stdc::check(&egc);
        let n = egc.len();
        egc[n - 2..].copy_from_slice(&check);

        let d = read(&egc).expect("a row");
        // A coast station warning an area: nobody wrote it, and it is
        // something whoever is at sea has to be told about.
        assert_eq!((d.id, d.kind), ("inmarsat-c", "egc"));
        assert!(d.wrote().is_none(), "a coast station's computer did not write it");
        let common::packet::Fact::Alert(a) = &d.facts[0] else {
            panic!("an alert, got {:?}", d.facts);
        };
        assert_eq!(a.kind, common::packet::AlertKind::Weather);
        assert_eq!(a.severity, common::packet::Severity::Warning);
        assert_eq!(a.text.as_deref(), Some("NAVAREA I 123/25 NORTH SEA UNLIT BUOY ADRIFT"));
    }

    /// Ten minutes of noise on the channel, and nothing reaches the bus.
    #[test]
    fn noise_produces_no_packets() {
        let rate = 38_400.0;
        let mut s = 5u64;
        let iq: Vec<C32> = (0..(rate as usize * 600))
            .map(|_| {
                let mut next = || {
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (s >> 33) as f32 / (1u64 << 30) as f32 - 1.0
                };
                C32::new(next(), next())
            })
            .collect();
        let mut n = StdcNode::new(DEFAULT_HZ);
        assert_eq!(heard(&mut n, rate, &iq).len(), 0);
        assert_eq!(n.packets(), 0);
    }
}
