//! Z-Wave on the 868 and 908 MHz channels as a graph node: one channel in,
//! MAC frames out.
//!
//! Wiring only, like `nrf24_nodes`. The waveform is [`dsp::fsk::BitSync`],
//! which reads any two-level FSK at a told baud, and the frame is
//! [`decode::zwave`], which holds the preamble, the layout and the two
//! checks. Neither knows about the other or about the graph.
//!
//! What a listener gets is the network and its traffic: which home id is
//! transmitting, which node addressed which, and what command class was
//! invoked. A secured network encrypts the payload above that, so a row is
//! who talked to whom and not what was said.
//!
//! # Three data rates at once
//!
//! A network runs at 9.6, 40 or 100 kbit/s and says which nowhere on the
//! air, and a controller drops to a slower rate to reach a distant node, so
//! one house can use two of them in a minute. All three clocks therefore run
//! over the same samples and the frame check decides which was right, as the
//! two nRF24 rates do. 9.6 kbit/s is Manchester coded, so its clock runs at
//! 19.2 kchip/s and the chips are folded to bits at both phases; 40 and
//! 100 kbit/s are sent as they are.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::zwave;
pub use decode::zwave::read;
pub use decode::zwave::{KEEP_BITS, MAX_FRAME_BITS, RATES};
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::zwave::CHANNEL_WIDTH_HZ;
pub use identify::zwave::CHANNELS;
pub use identify::zwave::WORK_HZ;
pub use identify::zwave::ZWave;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct ZWaveNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    readers: Vec<zwave::Reader>,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    meter: crate::FrameMeter,
    accepted: u64,
}

impl Default for ZWaveNode {
    fn default() -> Self {
        Self::new(868_420_000.0)
    }
}

impl ZWaveNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(WORK_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            readers: RATES
                .iter()
                .map(|(baud, bw, man)| zwave::Reader::new(WORK_HZ, *baud, *bw, *man))
                .collect(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            meter: crate::FrameMeter::new(WORK_HZ, 868_420_000, 0.05),
            accepted: 0,
        }
    }

    /// Frames that passed their check since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for ZWaveNode {
    fn name(&self) -> &str {
        "zwave"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("zwave reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if self.channel_hz <= 0.0 {
            self.channel_hz = center;
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("zwave needs its channel inside the span"));
        }
        let mut factor = 1usize;
        while rate / (factor * 2) as f64 >= WORK_HZ {
            factor *= 2;
        }
        let work = rate / factor as f64;
        if work < 4.0 * RATES[2].0 {
            return Err(common::Error::other(
                "zwave needs at least 400 kS/s: four samples a symbol at 100 kbit/s",
            ));
        }
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.readers = RATES
            .iter()
            .map(|(baud, bw, man)| zwave::Reader::new(work, *baud, *bw, *man))
            .collect();
        // Fifty milliseconds: the longest frame at 9.6 kbit/s is about
        // 60 ms of preamble and payload, and a shorter ring would hand a
        // slow frame samples that are not its own.
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 0.05);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);

        let mut found: Vec<zwave::Frame> = Vec::new();
        let narrow = std::mem::take(&mut self.narrow);
        for r in &mut self.readers {
            r.read(&narrow, &mut found);
        }
        self.narrow = narrow;

        let out = o.packets_mut();
        for f in &found {
            self.accepted += 1;
            out.push(self.meter.packet_now(f.bytes.clone()));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.meter.reset();
        for r in &mut self.readers {
            r.reset();
        }
    }
}

impl Protocol for ZWave {
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

    /// The European channel, which is where most of the world's Z-Wave is.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: CHANNEL_WIDTH_HZ as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !CHANNELS.iter().any(|c| (c - hz).abs() <= CHANNEL_WIDTH_HZ / 2.0) {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "Z-WAVE".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "zwave",
    summary: "One Z-Wave channel: G.9959 frames at 9.6, 40 and 100 kbit/s",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(ZWaveNode::new(s.f64_or(CHANNEL_HZ, 868_420_000.0))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A transmission as a node sends it: the frame keyed at one of the
    /// three rates, with silence either side. `manchester` doubles every bit
    /// into a chip pair, which is what 9.6 kbit/s does.
    fn burst(frame: &[u8], rate: f64, baud: f64, deviation_hz: f64, man: bool) -> Vec<C32> {
        let bits = zwave::keyed(frame, 20);
        let symbols: Vec<bool> =
            if man { bits.iter().flat_map(|b| [*b, !*b]).collect() } else { bits };
        let iq = dsp::fsk::modulate(&symbols, rate, baud, deviation_hz, 0.5);
        let quiet = vec![C32::new(0.0, 0.0); (rate * 0.005) as usize];
        [&quiet[..], &iq[..], &quiet[..]].concat()
    }

    fn run(node: &mut ZWaveNode, iq: &[C32], rate: f64, center: f64) -> Vec<Vec<u8>> {
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in iq.chunks(8192) {
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

    /// The whole path on synthetic RF at each of the three rates, with the
    /// transmitter off the centre of the span: nothing on the air says which
    /// rate is in use, so the check is what decides.
    #[test]
    fn a_keyed_frame_becomes_a_row_at_every_rate() {
        let (rate, center) = (2_000_000.0, 868_300_000.0);
        let channel = 868_420_000.0;
        for (baud, deviation, man, fcs) in [
            (19_200.0, 20_000.0, true, zwave::Fcs::Xor),
            (40_000.0, 20_000.0, false, zwave::Fcs::Xor),
            (100_000.0, 29_000.0, false, zwave::Fcs::Crc16),
        ] {
            let frame = zwave::encode(
                fcs,
                0xd6b2_6208,
                1,
                7,
                zwave::singlecast_control(3, true),
                &[0x25, 0x01, 0xff],
            );
            let iq = burst(&frame, rate, baud, deviation, man);
            let mut ph = 0.0f64;
            let iq: Vec<C32> = iq
                .iter()
                .map(|s| {
                    ph += std::f64::consts::TAU * (channel - center) / rate;
                    s * C32::new(ph.cos() as f32, ph.sin() as f32)
                })
                .collect();

            let mut node = ZWaveNode::new(channel);
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            assert_eq!(frames.len(), 1, "{baud} baud: {} frames", frames.len());
            assert_eq!(frames[0], frame, "{baud} baud: the bytes came back changed");

            let d = read(&frames[0]).expect("a decode");
            assert_eq!((d.id, d.kind), ("zwave", "singlecast"));
            assert!(d.wrote().is_none(), "a plug being switched is a machine talking");
            // The home identifier is part of who a node is: two networks
            // in a street both have a node 1.
            assert_eq!(d.parties(), (Some("d6b26208:1"), Some("d6b26208:7")));
            assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("d6b26208:1"));
        }
    }

    /// A command and the acknowledgement that answers it, a few
    /// milliseconds apart as a real exchange is, are two rows and not one.
    #[test]
    fn both_halves_of_an_exchange_are_read() {
        let (rate, center) = (1_000_000.0, 868_420_000.0);
        let command = zwave::encode(
            zwave::Fcs::Xor,
            0x0161_f498,
            1,
            7,
            zwave::singlecast_control(3, true),
            &[0x25, 0x01, 0x00],
        );
        let ack = zwave::encode(zwave::Fcs::Xor, 0x0161_f498, 7, 1, [0x03, 0x03], &[]);
        let mut iq = burst(&command, rate, 40_000.0, 20_000.0, false);
        iq.extend(burst(&ack, rate, 40_000.0, 20_000.0, false));

        let mut node = ZWaveNode::new(center);
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 2, "{} frames of two transmissions", frames.len());
        let pairs: Vec<(u8, u8)> = frames
            .iter()
            .map(|f| {
                let p = zwave::parse(f).expect("a frame");
                (p.source, p.dest)
            })
            .collect();
        assert_eq!(pairs, vec![(1, 7), (7, 1)], "the exchange came back out of order");
        let d = read(&frames[1]).expect("a decode");
        assert_eq!(d.kind, "ack");
        let (from, to) = d.parties();
        assert!(from.is_some_and(|f| f.ends_with(":7")), "{from:?}");
        assert!(to.is_some_and(|t| t.ends_with(":1")), "{to:?}");
    }

    /// Ten seconds of noise produces nothing at any of the three rates.
    #[test]
    fn noise_produces_no_frames() {
        let (rate, center) = (1_000_000.0, 868_420_000.0);
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> = (0..rate as usize * 10).map(|_| C32::new(rng(), rng())).collect();
        let mut node = ZWaveNode::new(center);
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 0, "{} frames out of ten seconds of noise", frames.len());
    }

    #[test]
    fn the_node_refuses_a_span_it_cannot_read() {
        let mut n = ZWaveNode::new(868_420_000.0);
        assert!(n.negotiate(&spec(2_000_000.0, 868_420_000.0)).is_ok());
        assert!(n.negotiate(&spec(500_000.0, 868_420_000.0)).is_ok());
        assert!(n.negotiate(&spec(200_000.0, 868_420_000.0)).is_err());
        assert!(n.negotiate(&spec(1_000_000.0, 869_500_000.0)).is_err());
    }
}
