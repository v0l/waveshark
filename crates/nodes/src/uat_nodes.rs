//! UAT at 978 MHz as a graph node.
//!
//! The wiring only: the waveform is `dsp::fsk`'s sync-word detector, the
//! frames and the FIS-B products are `decode::uat`, and neither knows about
//! the other or about pipelines.
//!
//! One node rather than two, for the reason Mode S is one: a frame the
//! demodulator believes blanks the air it occupied, and the only thing that
//! can tell a sync word off noise from a real one is the Reed-Solomon
//! decode. So the acceptance test runs inside the search, as the closure the
//! detector is given, and a rejected match blanks nothing.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::uat::MAX_SYNC_ERRORS;
pub use decode::uat::adsb_read;
pub use decode::uat::product_read;
pub use decode::uat::read;
pub use decode::uat::round1;
pub use decode::uat::round5;
pub use decode::uat::uplink_read;
use decode::uat::{self};
pub use decode::uat::{ADSB, UPLINK, patterns};
pub use decode::uat::{correct, pack};
use dsp::fsk::{SyncBurst, SyncDetector};
use identify::Signal;
pub use identify::uat::CHANNEL_WIDTH_HZ;
pub use identify::uat::Uat;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct UatNode {
    det: SyncDetector,
    meter: crate::FrameMeter,
    bursts: Vec<SyncBurst>,
    accepted: u64,
}

impl Default for UatNode {
    fn default() -> Self {
        Self::new()
    }
}

impl UatNode {
    pub fn new() -> Self {
        Self {
            // Replaced at negotiation, when the real rate is known.
            det: SyncDetector::new(2_400_000.0, uat::BAUD, patterns()),
            // An uplink frame is 4.4 ms of air; ten milliseconds holds one
            // whole with room either side.
            meter: crate::FrameMeter::new(2_400_000.0, uat::CHANNEL_HZ as u64, 0.01)
                .keyed_as(common::Modulation::Fsk2),
            bursts: Vec::new(),
            accepted: 0,
        }
    }

    /// Frames that corrected since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for UatNode {
    fn name(&self) -> &str {
        "uat"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("uat reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < 2.0 * uat::BAUD {
            return Err(common::Error::other(
                "uat needs 2.084 MS/s or more: its bits are 0.96 us wide",
            ));
        }
        // The channel and its skirts have to be inside the span. Read
        // through the anti-alias filter's edge it is silence.
        if (center - uat::CHANNEL_HZ).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("uat needs 978 MHz inside the span"));
        }
        self.det = SyncDetector::new(rate, uat::BAUD, patterns());
        self.meter = crate::FrameMeter::new(rate, uat::CHANNEL_HZ as u64, 0.01)
            .keyed_as(common::Modulation::Fsk2);
        // Frames rather than bytes: an 18-byte basic message and a 34-byte
        // long one written into one buffer cannot be told apart afterwards,
        // and the length is what says which it was.
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(uat::CHANNEL_HZ as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.meter.feed(iq);
        self.bursts.clear();
        // The code is the acceptance test, so only a frame that corrected
        // blanks the air behind it.
        self.det.process_valid(iq, &mut self.bursts, &|b: &SyncBurst| correct(b).is_some());
        let out = o.packets_mut();
        for b in &self.bursts {
            let Some(c) = correct(b) else { continue };
            self.accepted += 1;
            let snr = self.meter.snr_db_at(b.at_sample, b.len_samples);
            out.push(self.meter.packet_measured(c.data, b.at_sample, b.len_samples, snr));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.det.reset();
    }
}

impl Protocol for Uat {
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

    /// An aircraft sends its own position, and a ground station its site.
    fn reports_position(&self) -> bool {
        true
    }

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 2_000_000 }
    }
    /// A UAT payload and any other frame are both bytes; where it was
    /// received is what tells them apart, and then its length says which of
    /// the three forms it is.
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !uat::is_uat_band(p.center_hz() as f64) {
            return None;
        }
        Some(uat::parse(bytes).map(|f| read(&f)).unwrap_or_default())
    }

    fn stage_label(&self, _hz: f64) -> String {
        "978 UAT".into()
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "UAT".into() }]
    }
    /// Every sample gets a discriminator and two correlations, so what it is
    /// handed is what it costs: 2.4 MS/s is plenty for a 1 Mbit/s link and
    /// the 20 MS/s a receiver may be running is eight times the work for the
    /// same frames.
    fn narrow_span(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("uat")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "uat",
    summary: "978 MHz UAT: sync word search, FSK bits and Reed-Solomon",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(UatNode::default()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_rate_its_bits_cannot_survive() {
        let mut n = UatNode::default();
        assert!(n.negotiate(&spec(2_000_000.0, uat::CHANNEL_HZ)).is_err());
        assert!(n.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).is_ok());
        // Tuned elsewhere: the channel is not in the span.
        assert!(n.negotiate(&spec(2_400_000.0, 1_090_000_000.0)).is_err());
    }

    /// Bits keyed at the UAT rate and deviation, sync word first.
    fn on_air(sync: u64, payload: &[u8]) -> Vec<C32> {
        let mut bits: Vec<bool> = (0..uat::SYNC_BITS).rev().map(|k| (sync >> k) & 1 == 1).collect();
        for b in payload {
            for k in (0..8).rev() {
                bits.push((b >> k) & 1 == 1);
            }
        }
        dsp::fsk::modulate(&bits, 2_400_000.0, uat::BAUD, uat::DEVIATION_HZ, 1.0)
    }

    fn noise(n: usize, seed: u64, amp: f32) -> Vec<C32> {
        let mut s = seed;
        let mut rng = move || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((s >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * amp
        };
        (0..n).map(|_| C32::new(rng(), rng())).collect()
    }

    /// Run blocks through the node and collect what reached the bus.
    fn run(node: &mut UatNode, blocks: &[Vec<C32>]) -> Vec<common::packet::Packet> {
        let ins = [spec(2_400_000.0, uat::CHANNEL_HZ)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in blocks {
            let input = Payload::Iq(block.clone());
            let mut out = Payload::Packets(Vec::new());
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                frames.extend(f);
            }
        }
        frames
    }

    /// A long codeword keyed on the air, read back as the aircraft that sent
    /// it. The point is that the three layers agree about bit order: each
    /// could be self-consistently wrong alone.
    #[test]
    fn a_keyed_frame_becomes_an_aircraft_at_the_right_place() {
        let payload = long_payload();
        let mut coded = payload.clone();
        coded.extend_from_slice(
            &decode::rs::ReedSolomon::new(8, 0x187, 120, 1, 14, 207).encode(&payload),
        );
        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        let frames = run(
            &mut node,
            &[noise(16_384, 1, 0.02), on_air(uat::ADSB_SYNC, &coded), noise(16_384, 2, 0.02)],
        );

        assert_eq!(frames.len(), 1, "one frame off the air");
        assert_eq!(frames[0].bytes(), payload, "the 34 bytes the aircraft sent");
        assert!(frames[0].carrier.rssi_dbfs.is_finite() && frames[0].carrier.snr_db.is_finite());
        assert!(frames[0].carrier.iq.is_some(), "the samples it was read from");
        assert_eq!(node.accepted(), 1);

        let rows = Uat.stated(&frames[0]).expect("a frame from the UAT channel");
        assert_eq!(rows.len(), 1);
        let d = &rows[0];
        assert_eq!((d.id, d.kind), ("uat", "position"));
        assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("a0dead"));
        assert_eq!(d.subject.as_ref().and_then(|e| e.name.clone()), Some("N172SP".into()));
        let p = d.placed().expect("a position");
        assert!((p.lat - 40.0).abs() < 0.001 && (p.lon + 105.0).abs() < 0.001);
        // Height is a reading: an aircraft sends one in frames that say
        // nothing about where it is.
        assert!(d.facts.iter().any(|f| matches!(
            f,
            common::packet::Fact::Sensed(r)
                if r.quantity == common::packet::Quantity::Altitude
                    && (r.value - 9_500.0 * 0.3048).abs() < 1.0
        )));
    }

    /// Minutes of noise, and nothing reaches the bus: a sync word that
    /// matches noise has no codeword behind it.
    #[test]
    fn noise_decodes_to_nothing() {
        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        // Two minutes of air at 2.4 MS/s, in blocks of a hundredth of a
        // second.
        let blocks: Vec<Vec<C32>> = (0..600).map(|i| noise(24_000, 1000 + i, 0.3)).collect();
        let frames = run(&mut node, &blocks);
        assert_eq!(frames.len(), 0, "{} frames out of noise", frames.len());
        assert_eq!(node.accepted(), 0);
    }

    /// One wrong byte in the air is inside the code, and the row that comes
    /// out is the one the aircraft sent.
    #[test]
    fn a_frame_with_a_broken_byte_still_corrects() {
        let payload = long_payload();
        let mut coded = payload.clone();
        coded.extend_from_slice(
            &decode::rs::ReedSolomon::new(8, 0x187, 120, 1, 14, 207).encode(&payload),
        );
        coded[5] ^= 0xff;
        coded[20] ^= 0x0f;
        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        let frames = run(
            &mut node,
            &[noise(16_384, 3, 0.02), on_air(uat::ADSB_SYNC, &coded), noise(16_384, 4, 0.02)],
        );
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].bytes(), payload);
    }

    /// A ground station's uplink: 552 bytes of interleaved codewords, read
    /// back as the station and the one product it sent.
    #[test]
    fn a_keyed_uplink_becomes_a_station_and_a_product() {
        let mut data = vec![0u8; uat::UPLINK_DATA_BYTES];
        let lat = (39.861_67f64 * 16_777_216.0 / 360.0).round() as u32;
        let lon = ((360.0f64 - 104.673) * 16_777_216.0 / 360.0).round() as u32;
        data[0] = (lat >> 15) as u8;
        data[1] = (lat >> 7) as u8;
        data[2] = ((lat << 1) as u8) | ((lon >> 23) as u8 & 1);
        data[3] = (lon >> 15) as u8;
        data[4] = (lon >> 7) as u8;
        data[5] = ((lon << 1) as u8) | 1;
        data[6] = 0x80 | 0x20 | 9;
        data[7] = 2 << 4;
        // Product 51, national NEXRAD: a picture, listed rather than drawn.
        let body = [(51u16 >> 6) as u8, ((51 & 0x3f) << 2) as u8, 16 << 2, 53 << 2, 0xde, 0xad];
        data[8] = (body.len() >> 1) as u8;
        data[9] = (body.len() << 7) as u8;
        data[10..10 + body.len()].copy_from_slice(&body);

        let rs = decode::rs::ReedSolomon::new(8, 0x187, 120, 1, 20, 163);
        let mut coded = vec![0u8; uat::UPLINK_BYTES];
        for block in 0..uat::UPLINK_BLOCKS {
            let from = block * uat::UPLINK_BLOCK_DATA_BYTES;
            let chunk = &data[from..from + uat::UPLINK_BLOCK_DATA_BYTES];
            let mut word = chunk.to_vec();
            word.extend_from_slice(&rs.encode(chunk));
            for (i, b) in word.iter().enumerate() {
                coded[i * uat::UPLINK_BLOCKS + block] = *b;
            }
        }

        let mut node = UatNode::default();
        node.negotiate(&spec(2_400_000.0, uat::CHANNEL_HZ)).unwrap();
        let frames = run(
            &mut node,
            &[noise(16_384, 5, 0.02), on_air(uat::UPLINK_SYNC, &coded), noise(16_384, 6, 0.02)],
        );
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].bytes().len(), uat::UPLINK_DATA_BYTES);

        let rows = Uat.stated(&frames[0]).expect("an uplink");
        assert_eq!(rows.len(), 2, "the station, and the product it sent");
        assert_eq!((rows[0].id, rows[0].kind), ("uat", "uplink"));
        let site = rows[0].placed().expect("a site position");
        assert!((site.lat - 39.861_67).abs() < 0.001 && (site.lon + 104.673).abs() < 0.001);
        // A weather product a machine composed and broadcast to everybody in
        // range: an advisory, and never a message anybody wrote.
        assert_eq!(rows[1].id, "fisb");
        assert!(rows[1].wrote().is_none());
        assert!(rows[1].facts.iter().any(|f| matches!(
            f,
            common::packet::Fact::Alert(a) if a.kind == common::packet::AlertKind::Weather
        )));
    }

    /// The aircraft of the decode crate's own test: 40 N, 105 W at 9500
    /// feet, calling itself N172SP.
    fn long_payload() -> Vec<u8> {
        let mut f = vec![0u8; uat::LONG_DATA_BYTES];
        f[0] = 1 << 3;
        f[1] = 0xa0;
        f[2] = 0xde;
        f[3] = 0xad;
        let lat = (40.0f64 * 16_777_216.0 / 360.0).round() as u32;
        let lon = ((360.0f64 - 105.0) * 16_777_216.0 / 360.0).round() as u32;
        f[4] = (lat >> 15) as u8;
        f[5] = (lat >> 7) as u8;
        f[6] = ((lat << 1) as u8) | ((lon >> 23) as u8 & 1);
        f[7] = (lon >> 15) as u8;
        f[8] = (lon >> 7) as u8;
        f[9] = (lon << 1) as u8;
        let alt = ((9_500 + 1000) / 25 + 1) as u16;
        f[10] = (alt >> 4) as u8;
        f[11] = ((alt << 4) as u8) | 8;
        let (ns, ew) = (101u32, 101u32);
        f[12] = (ns >> 6) as u8;
        f[13] = ((ns << 2) as u8) | (ew >> 9) as u8;
        f[14] = (ew >> 1) as u8;
        f[15] = (ew << 7) as u8;
        let vv = 640 / 64 + 1;
        f[15] |= (vv >> 4) as u8;
        f[16] = (vv << 4) as u8;
        // "N172SP", three characters to every two bytes, base 40, the first
        // of them the emitter category.
        for (at, v) in
            [(17, 1600 + 23 * 40 + 1), (19, 7 * 1600 + 2 * 40 + 28), (21, 25 * 1600 + 36 * 40 + 36)]
        {
            f[at] = (v >> 8) as u8;
            f[at + 1] = v as u8;
        }
        f[26] = 0x02;
        f
    }
}
