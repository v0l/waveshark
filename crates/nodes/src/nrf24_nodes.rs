//! nRF24L01 and XN297 bursts as a graph node: a channel of the 2.4 GHz band
//! in, frames out.
//!
//! Wiring only. The waveform is [`dsp::fsk::BitSync`], which reads any
//! two-level FSK at a told baud, and the frame is [`decode::nrf24`], which
//! holds the XN297 preamble, the scrambling table and the CRC. Neither knows
//! about the other or about the graph.
//!
//! # Why only the XN297 half of the family is readable
//!
//! A plain ShockBurst frame opens with one byte of preamble and then the
//! address the receiver was told in advance, so a listener who was not told
//! it has eight known bits to find a packet by, which is not enough. The
//! XN297 clone in every cheap remote sends 28 bits of its own preamble first,
//! which is a sync word a stranger can lock onto, and scrambles with a
//! published table. So this reads the clone and refuses to invent the
//! original.
//!
//! # Both bit rates at once
//!
//! The chip keys 250 kbit/s or 1 Mbit/s and says which nowhere on the air, so
//! both are demodulated in parallel over the same samples and the CRC decides
//! which was right. A bit clock is cheap beside the mixer and the filter in
//! front of it, and the alternative is asking an operator to know what a toy
//! they can hear is keyed at.
//!
//! What this cannot do is find a burst nothing opened a source around. A
//! remote hops every packet and each packet is about 200 us, so it depends on
//! the detector opening something on the hop: where it does, this reads it.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
use decode::nrf24;
use dsp::fsk::BitSync;
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Where the chip can tune: a megahertz a step from 2400 MHz, 126 channels.
pub const BAND: (f64, f64) = (2_400_000_000.0, 2_526_000_000.0);

/// The channel one burst occupies. A 1 Mbit/s link keys about 320 kHz of
/// deviation, which is a megahertz by Carson, and the channels are spaced a
/// megahertz apart.
pub const CHANNEL_WIDTH_HZ: f64 = 1_000_000.0;

/// The two bit rates an XN297 keys. The chip supports no others.
const BAUDS: [f64; 2] = [250_000.0, 1_000_000.0];

/// Rate the channel is cut down to before the bit clocks read it: four
/// samples a symbol at the faster rate, which is where [`BitSync`] stops.
const WORK_HZ: f64 = 4_000_000.0;

/// The longest frame the chip sends: five address bytes, thirty-two of
/// payload and the check, behind the preamble.
const MAX_FRAME_BITS: usize = nrf24::PREAMBLE_BITS + (5 + 32 + 2) * 8;

/// Bits kept behind the search so a frame split across two blocks is still
/// whole when the second arrives.
const KEEP_BITS: usize = MAX_FRAME_BITS * 2;

pub struct Nrf24Node {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    readers: Vec<Reader>,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    meter: crate::FrameMeter,
    accepted: u64,
}

/// One bit rate's clock and the bits it has produced but not yet read a
/// frame out of.
struct Reader {
    sync: BitSync,
    bits: Vec<bool>,
    /// Bits dropped off the front, so a frame's position stays a position in
    /// the stream rather than in what is left of it.
    dropped: u64,
    /// Where the search has reached, counted in the same stream positions.
    /// The tail is kept for a frame that is still arriving, so without this
    /// the frame at the end of one block is read again out of the next.
    read_from: u64,
}

impl Reader {
    fn new(rate: f64, baud: f64) -> Self {
        // A GFSK link at modulation index 0.64 occupies about 1.6 times its
        // baud, and the filter in the bit clock is what keeps the rest of
        // the channel's noise out of the discriminator.
        Self {
            sync: BitSync::with_bandwidth(rate, baud, 1.6 * baud),
            bits: Vec::new(),
            dropped: 0,
            read_from: 0,
        }
    }

    /// Demodulate a block and hand back every frame that closed inside it,
    /// each with the bit it started at.
    fn read(&mut self, iq: &[common::C32], out: &mut Vec<(u64, nrf24::Packet)>) {
        if !self.sync.usable() {
            return;
        }
        self.sync.process(iq, &mut self.bits);
        let mut from = (self.read_from - self.dropped) as usize;
        while let Some(at) = nrf24::find_preamble(&self.bits, from) {
            // A preamble too near the end may be a frame still arriving, so
            // leave it for the next block rather than deciding on half of it.
            if self.bits.len() - at < MAX_FRAME_BITS {
                from = at;
                break;
            }
            match nrf24::decode(&self.bits, at) {
                Some(p) => {
                    from = at + p.bits();
                    out.push((self.dropped + at as u64, p));
                }
                None => from = at + 1,
            }
        }
        self.read_from = self.dropped + from as u64;
        let keep = self.bits.len().min(KEEP_BITS);
        let cut = self.bits.len() - keep;
        if cut > 0 {
            self.bits.drain(..cut);
            self.dropped += cut as u64;
            self.read_from = self.read_from.max(self.dropped);
        }
    }

    fn reset(&mut self) {
        self.sync.reset();
        self.bits.clear();
        self.dropped = 0;
        self.read_from = 0;
    }
}

impl Default for Nrf24Node {
    fn default() -> Self {
        Self::new(2_441_000_000.0)
    }
}

impl Nrf24Node {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(WORK_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            readers: BAUDS.iter().map(|b| Reader::new(WORK_HZ, *b)).collect(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            meter: crate::FrameMeter::new(WORK_HZ, 2_441_000_000, 0.01),
            accepted: 0,
        }
    }

    /// Frames that passed the CRC since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for Nrf24Node {
    fn name(&self) -> &str {
        "nrf24"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("nrf24 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if !(BAND.0..BAND.1).contains(&self.channel_hz) {
            self.channel_hz = center;
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("nrf24 needs its channel inside the span"));
        }
        // Down to about four megasamples, which is four a symbol at
        // 1 Mbit/s: below that the faster of the two clocks refuses, and
        // above it every extra sample is spent proving the same bits.
        let mut factor = 1usize;
        while rate / (factor * 2) as f64 >= WORK_HZ {
            factor *= 2;
        }
        let work = rate / factor as f64;
        if work < 1_000_000.0 {
            return Err(common::Error::other("nrf24 needs at least 1 MS/s"));
        }
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.readers = BAUDS.iter().map(|b| Reader::new(work, *b)).collect();
        if self.readers.iter().all(|r| !r.sync.usable()) {
            return Err(common::Error::other("nrf24 needs four samples a symbol"));
        }
        // Ten milliseconds: a burst is about 200 us and a remote sends one
        // every few, so a frame's own samples are still there when it is
        // read out.
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 0.01);

        let mut out = i.spec.with_kind(PortKind::Frames);
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

        let mut found: Vec<(u64, nrf24::Packet)> = Vec::new();
        let narrow = std::mem::take(&mut self.narrow);
        for r in &mut self.readers {
            r.read(&narrow, &mut found);
        }
        self.narrow = narrow;

        let out = o.frames_mut();
        for (_at, p) in &found {
            self.accepted += 1;
            out.push(self.meter.frame(p.on_air()));
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

/// The channel index a centre names, as the chip's own register value.
pub fn channel_of(center_hz: f64) -> Option<u8> {
    let ch = ((center_hz - BAND.0) / 1e6).round();
    (0.0..=125.0).contains(&ch).then_some(ch as u8)
}

/// The row a frame off the bus becomes.
pub fn nrf24_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let p = nrf24::from_on_air(bytes)?;
    let mut fields = nrf24::fields(&p);
    if let Some(ch) = channel_of(center.as_f64()) {
        fields.insert(0, ("channel".into(), Value::Int(i64::from(ch))));
    }
    let address =
        fields.iter().find(|(k, _)| k == "address").map(|(_, v)| v.to_string()).unwrap_or_default();
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    Some(
        Decoded::bytes("XN297", center, 0.0, bytes.to_vec())
            .by(common::Identity::new("nrf24", address.clone()))
            .with_link(pipeline::event::Link {
                from: Some(pipeline::event::Party::unit(address)),
                to: None,
            })
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation(common::Modulation::Gfsk)
            // The CRC-16 was checked again here, on the bytes in the row,
            // rather than taken on trust from whatever put them on the bus.
            .with_crc(Some(true)),
    )
}

pub struct Nrf24;

impl Protocol for Nrf24 {
    fn id(&self) -> &'static str {
        "nrf24"
    }
    fn label(&self) -> &'static str {
        "nrf24"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["xn297", "shockburst"]
    }
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }
    /// The middle of the band, which is where a remote's hop set is centred
    /// even though no particular packet is sent there.
    fn default_hz(&self) -> f64 {
        2_441_000_000.0
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 1_000_000.0,
            feed_rate_hz: WORK_HZ,
            span_wide: false,
            families: &[],
        }
    }
    /// A remote hops on every packet, so the channel one was heard on says
    /// nothing about where the next one will be.
    fn stickiness(&self) -> Stickiness {
        Stickiness::Forget
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) {
            return None;
        }
        Some(nrf24_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn stage_label(&self, hz: f64) -> String {
        match channel_of(hz) {
            Some(ch) => format!("ch{ch} nRF24"),
            None => "nRF24".into(),
        }
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "nRF24".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "nrf24",
    summary: "One 2.4 GHz nRF24 channel: XN297 bursts at 250 kbit/s and 1 Mbit/s",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Nrf24Node::new(s.f64_or(CHANNEL_HZ, 2_441_000_000.0))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A burst as the chip sends it: a lead-in of alternating bits for the
    /// listener's clock, the frame, and silence either side.
    fn burst(address: &[u8], payload: &[u8], rate: f64, baud: f64) -> Vec<C32> {
        let mut bits: Vec<bool> = (0..64).map(|k| k % 2 == 0).collect();
        bits.extend(nrf24::encode(address, payload, true));
        // Modulation index 0.64, which is what the chip keys.
        let iq = dsp::fsk::modulate(&bits, rate, baud, 0.32 * baud, 0.5);
        let quiet = vec![C32::new(0.0, 0.0); (rate * 0.002) as usize];
        [&quiet[..], &iq[..], &quiet[..]].concat()
    }

    fn run(node: &mut Nrf24Node, iq: &[C32], rate: f64, center: f64) -> Vec<Vec<u8>> {
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in iq.chunks(8192) {
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

    /// The whole path on synthetic RF, at both bit rates and with the
    /// transmitter off the centre of the span: nothing on the air says which
    /// rate is in use, so the CRC is what decides.
    #[test]
    fn a_keyed_burst_becomes_a_row_at_either_bit_rate() {
        let address = [0xa4, 0x03, 0x55, 0x11, 0x22];
        let payload: Vec<u8> = (0..15).map(|i| i * 17 + 3).collect();
        for baud in [250_000.0, 1_000_000.0] {
            let (rate, center) = (8_000_000.0, 2_440_000_000.0);
            let channel = center + 2_000_000.0;
            let iq = burst(&address, &payload, rate, baud);
            let mut ph = 0.0f64;
            let iq: Vec<C32> = iq
                .iter()
                .map(|s| {
                    ph += std::f64::consts::TAU * 2_000_000.0 / rate;
                    s * C32::new(ph.cos() as f32, ph.sin() as f32)
                })
                .collect();

            let mut node = Nrf24Node::new(channel);
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            assert_eq!(frames.len(), 1, "{baud} baud: {} frames", frames.len());

            let d = nrf24_decoded(&frames[0], Hz(channel as u64)).expect("a decode");
            assert_eq!(d.protocol, "XN297");
            assert_eq!(d.crc_ok, Some(true));
            assert!(!d.written, "a remote is a machine talking about itself");
            let detail = d.detail.as_deref().unwrap();
            assert!(detail.contains("address=a403551122"), "{detail}");
            assert!(detail.contains("payload_len=15"), "{detail}");
            assert!(detail.contains("channel=42"), "{detail}");
        }
    }

    /// Two bursts a few milliseconds apart, which is what a remote sends,
    /// are two rows and not one.
    #[test]
    fn every_burst_of_a_run_is_read() {
        let (rate, center) = (4_000_000.0, 2_450_000_000.0);
        let address = [0xcc, 0xcc, 0xcc, 0xcc, 0xcc];
        let mut iq = Vec::new();
        for n in 0..4u8 {
            iq.extend(burst(&address, &[n, 2, 3, 4], rate, 1_000_000.0));
        }
        let mut node = Nrf24Node::new(center);
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 4, "{} frames of four bursts", frames.len());
        let payloads: Vec<u8> =
            frames.iter().map(|f| nrf24::from_on_air(f).expect("a packet").payload[0]).collect();
        assert_eq!(payloads, vec![0, 1, 2, 3], "the bursts came back out of order");
    }

    /// Two seconds of noise produces nothing. The preamble is 28 known bits
    /// and the CRC is sixteen more, and the two together are the whole of
    /// what keeps an empty band empty.
    #[test]
    fn noise_produces_no_frames() {
        let (rate, center) = (4_000_000.0, 2_450_000_000.0);
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> = (0..rate as usize * 2).map(|_| C32::new(rng(), rng())).collect();
        let mut node = Nrf24Node::new(center);
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 0, "{} frames out of two seconds of noise", frames.len());
    }

    #[test]
    fn the_node_refuses_a_span_it_cannot_read() {
        let mut n = Nrf24Node::new(2_450_000_000.0);
        assert!(n.negotiate(&spec(8_000_000.0, 2_450_000_000.0)).is_ok());
        assert!(n.negotiate(&spec(2_000_000.0, 2_450_000_000.0)).is_ok());
        assert!(n.negotiate(&spec(500_000.0, 2_450_000_000.0)).is_err());
        assert!(n.negotiate(&spec(8_000_000.0, 2_470_000_000.0)).is_err());
    }

    #[test]
    fn a_centre_names_the_chip_s_own_channel() {
        assert_eq!(channel_of(2_400_000_000.0), Some(0));
        assert_eq!(channel_of(2_476_000_000.0), Some(76));
        assert_eq!(channel_of(2_600_000_000.0), None);
    }
}
