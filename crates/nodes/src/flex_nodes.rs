//! FLEX as a graph node.
//!
//! The same shape as POCSAG and for the same reason: the channel is ordinary
//! narrowband FM, so the node mixes the pager channel down, filters it,
//! discriminates it, and hands the audio to `dsp::flex`, which recovers the
//! symbols, finds the sync word and collects the frame's phases. The words
//! are read by `decode::flex`. Neither of those knows about pipelines.
//!
//! What reaches the bus is one frame: the sync code, the frame information
//! word and every phase's words as they were received, so a reader gets the
//! evidence rather than a rendering of it.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::flex::read;
use dsp::flex::{CHANNEL_WIDTH_HZ, DEVIATION_HZ, FlexConfig, FlexDemod, Frame};
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::flex::AUDIO_HZ;
pub use identify::flex::Flex;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// A UK FLEX paging channel, and only the default the node is built with
/// before the scanner table tells it where to listen.
pub const DEFAULT_HZ: f64 = 153_275_000.0;

pub struct FlexNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    demod: FlexDemod,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    meter: crate::FrameMeter,
    frames: Vec<Frame>,
    accepted: u64,
}

impl Default for FlexNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl FlexNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            demod: FlexDemod::new(AUDIO_HZ, FlexConfig::default()),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    /// Frames accepted since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for FlexNode {
    fn name(&self) -> &str {
        "flex"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("flex reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("flex needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.demod = FlexDemod::new(audio_rate, FlexConfig::default());
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0);

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
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);

        self.meter.feed(&self.narrow);
        self.frames.clear();
        let audio = std::mem::take(&mut self.audio);
        self.demod.process(&audio, &mut self.frames);
        self.audio = audio;

        let out = o.packets_mut();
        for f in &self.frames {
            self.accepted += 1;
            out.push(self.meter.packet_now(f.to_bytes()));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.demod.reset();
    }
}

impl Protocol for Flex {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
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

    /// The same paging allocations POCSAG watches, claimed one hertz
    /// narrower so that a frame is offered here first: a FLEX frame carries
    /// its sync code in its first two bytes and is refused below if it is
    /// not one, where POCSAG reads any bytes at all as codewords.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 64_999_999 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !dsp::pocsag::is_pager_band(p.center_hz() as f64) {
            return None;
        }
        let decoded = read(bytes);
        (!decoded.is_empty()).then_some(decoded)
    }

    /// A Dutch national FLEX channel: the one most likely to be carrying
    /// traffic anywhere a European operator points the receiver.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} FLEX", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "FLEX".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "flex",
    summary: "One FLEX paging channel: narrowband FM at 1600 or 3200 baud, two or four level",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(FlexNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::flex::Body;
    use decode::flex::Fiw;
    use dsp::flex::{Mode, encode_symbols};

    /// One frame as it reaches the packet bus, off a VHF pager channel.
    fn packet(bytes: Vec<u8>) -> common::packet::Packet {
        crate::measured(153_350_000, CHANNEL_WIDTH_HZ as u32, bytes, -40.0, 20.0)
    }

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = FlexNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 160_000_000.0)).is_err());
        assert!(n.negotiate(&spec(50_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(10_000.0, DEFAULT_HZ)).is_err());
    }

    /// FM at 2.4 MS/s carrying one FLEX frame, through the node and out as
    /// addressed pages.
    ///
    /// The point is that the three layers agree: the symbols `dsp::flex`
    /// recovers are the words `decode::flex` reads, in the interleave the
    /// addresses depend on. Each layer is tested alone and each could be
    /// self-consistently wrong.
    fn a_frame_through_the_node(mode: Mode) -> Vec<common::packet::Proto> {
        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let pages: Vec<Vec<(u32, Body)>> = vec![
            vec![
                (1_234_567, Body::Alpha("MOVE TO CHANNEL 2".into())),
                (98_765, Body::Numeric("0123456789".into())),
            ],
            vec![(4_242, Body::Alpha("SECOND PHASE".into()))],
            vec![(7, Body::Tone)],
            vec![(1_000_000, Body::Alpha("FOURTH PHASE".into()))],
        ];
        let phases: Vec<Vec<u32>> =
            (0..mode.phases()).map(|p| decode::flex::encode(&pages[p])).collect();
        let fiw = Fiw { cycle: 3, frame: 42 }.encode();
        let symbols = encode_symbols(mode, fiw, &phases);

        // Keyed FSK: the levels a FLEX transmitter sends, at the deviation it
        // sends them at. The speed is announced by the sync word alone, so
        // the node has to read it out of the signal.
        let head = 32 + 64 + 48;
        let mut iq = Vec::with_capacity(symbols.len() * 1_500);
        let mut phase = 0.0f64;
        for (i, &s) in symbols.iter().enumerate() {
            let baud = if i < head { 1600.0 } else { f64::from(mode.baud) };
            let sps = (rate / baud) as usize;
            let f = (f64::from(s) - 1.5) / 1.5 * DEVIATION_HZ;
            for _ in 0..sps {
                phase += std::f64::consts::TAU * f / rate;
                iq.push(common::C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }

        let mut node = FlexNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); 200_000];
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes().to_vec()));
            }
        }
        assert_eq!(frames.len(), 1, "expected one frame off the air");
        read(&frames[0])
    }

    #[test]
    fn a_modulated_frame_becomes_pages() {
        let decodes = a_frame_through_the_node(Mode { baud: 1600, levels: 2 });
        assert_eq!(decodes.len(), 2, "one phase carries two pages");
        // A page is written to whoever carries the pager, and the capcode is
        // the pager called: the network transmitted it.
        assert_eq!(decodes[0].kind, "FLEX-Alpha");
        assert_eq!(decodes[0].wrote(), Some("MOVE TO CHANNEL 2"));
        assert_eq!(decodes[0].parties(), (None, Some("1234567")));
        assert!(decodes[0].subject.is_none());
        assert_eq!(decodes[1].kind, "FLEX-Numeric");
        assert_eq!(decodes[1].wrote(), Some("0123456789"));
    }

    /// The fastest mode carries four phases at once, and all four are read.
    #[test]
    fn four_phases_are_read_from_one_frame() {
        let decodes = a_frame_through_the_node(dsp::flex::Mode { baud: 3200, levels: 4 });
        assert_eq!(decodes.len(), 5, "four phases, five pages");
        let capcodes: Vec<String> =
            decodes.iter().filter_map(|d| d.parties().1.map(str::to_string)).collect();
        assert_eq!(capcodes, ["1234567", "98765", "4242", "7", "1000000"]);
        // A tone page is the beep and nothing else; the phases it came off
        // are in the frame.
        assert_eq!(decodes[3].kind, "FLEX-Tone");
        assert_eq!(decodes[3].wrote(), None);
        assert_eq!(decodes[4].wrote(), Some("FOURTH PHASE"));
    }

    /// FLEX is offered a pager-band frame before POCSAG is, so it has to
    /// refuse one that is not a FLEX frame rather than read it.
    #[test]
    fn a_pocsag_transmission_is_left_for_pocsag() {
        let words: Vec<u32> = decode::pocsag::encode(1_234_568, 3, &decode::pocsag::Body::Tone)
            .into_iter()
            .map(dsp::pocsag::encode_codeword)
            .collect();
        let t = dsp::pocsag::Transmission { codewords: words, baud: 1200, corrected: 0, lost: 0 };
        let bytes = t.to_bytes();
        let p = packet(bytes.clone());
        assert_eq!(Flex.stated(&p), None, "a pager frame was claimed as FLEX");
        assert_eq!(
            crate::pocsag_nodes::Pocsag.stated(&p).map(|r| r.len()),
            Some(1),
            "POCSAG should still read its own"
        );
        // And the other way round: FLEX bytes are not offered to POCSAG,
        // because FLEX is asked first and answers.
        let frame = Frame {
            mode: dsp::flex::Mode { baud: 1600, levels: 2 },
            fiw: Fiw { cycle: 1, frame: 2 }.encode(),
            phases: vec![decode::flex::encode(&[(1_234_567, Body::Alpha("HELLO".into()))])],
        };
        let bytes = frame.to_bytes();
        let p = packet(bytes.clone());
        assert_eq!(Flex.stated(&p).map(|r| r.len()), Some(1));
    }

    /// Minutes of noise on the channel produce no rows.
    #[test]
    fn noise_is_not_a_page() {
        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let mut node = FlexNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut seed = 0x2468_ace0u32;
        let mut next = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed >> 8) as f32 / 8_388_608.0 - 1.0
        };
        let mut rows = 0;
        // Sixty seconds of noise, in one-second blocks.
        for _ in 0..60 {
            let block: Vec<common::C32> =
                (0..rate as usize).map(|_| common::C32::new(next(), next())).collect();
            let input = Payload::Iq(block);
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                for frame in f {
                    rows += read(frame.bytes()).len();
                }
            }
        }
        assert_eq!(rows, 0, "noise was read as pages");
    }
}
