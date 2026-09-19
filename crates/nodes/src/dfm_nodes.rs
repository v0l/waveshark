//! Graw DFM radiosondes as a stage: a source's stream in, gathered records
//! out.
//!
//! A DFM keys 2500 chips a second, Manchester coded, so the data rate is
//! 1250 bits a second and a 280-bit frame takes 224 ms of air. The waveform
//! is [`dsp::fsk::BitSync`] at the chip rate, the frame is
//! [`decode::dfm`], and this is the wire between them: find the header in
//! the chips, take the Manchester coding off, read the frame, and hand it to
//! the gatherer that turns nine of them into one position.
//!
//! Polarity is not fixed. A DFM-06 and a DFM-09 key the same coding the
//! other way up, and which way a receiver sees it also depends on the
//! receiver, so the header is looked for both ways round and whichever
//! matches says how to read the rest.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::dfm;
pub use decode::dfm::decoded;
use dsp::fsk::BitSync;
use identify::Signal;
pub use identify::dfm::BAND;
pub use identify::dfm::BAUD;
pub use identify::dfm::CHANNEL_WIDTH_HZ;
pub use identify::dfm::Dfm;
pub use identify::dfm::OCCUPIED_HZ;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct DfmNode {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    framer: dfm::Framer,
}

impl Default for DfmNode {
    fn default() -> Self {
        Self::new()
    }
}

impl DfmNode {
    pub fn new() -> Self {
        Self { sync: None, meter: crate::FrameMeter::new(1.0, 0, 0.6), framer: dfm::Framer::new() }
    }

    /// Frames whose every nibble came through the Hamming code.
    pub fn frames(&self) -> u64 {
        self.framer.frames()
    }

    /// Records gathered, which is what reaches the bus.
    pub fn records(&self) -> u64 {
        self.framer.records()
    }
}

impl Simple for DfmNode {
    fn name(&self) -> &str {
        "dfm"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dfm reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "dfm needs at least {} S/s for its {BAUD} chips a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 2.5);
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(s)) = (i.as_iq(), self.sync.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        s.process(iq, self.framer.sink());
        for record in self.framer.take() {
            o.frames_mut().push(self.meter.frame(record));
        }
        self.framer.trim();
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.framer.reset();
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

impl Protocol for Dfm {
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

    /// The meteorological aids allocation, as for the RS41: 2500 baud FSK in
    /// a 12.5 kHz channel is a common enough shape, and outside this band
    /// none of it is a sonde.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || bytes.len() != dfm::RECORD {
            return None;
        }
        Some(decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("dfm")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "dfm",
    summary: "Graw DFM radiosonde frames, 2500 baud Manchester FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DfmNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame's chips: the header, then every bit of the frame as a
    /// transition.
    fn chips(bits: &[bool]) -> Vec<bool> {
        bits.iter().flat_map(|&b| [!b, b]).collect()
    }

    /// One data block's nibbles, Hamming coded and interleaved, as the frame
    /// carries them. The same arrangement the decoder undoes.
    fn put(bits: &mut [bool], at: usize, nibs: &[u8]) {
        let rows: [u8; 4] = [0b0111_0000, 0b1011_0000, 0b1101_0000, 0b1110_0000];
        let l = nibs.len();
        for (i, nib) in nibs.iter().enumerate() {
            let mut code = nib << 4;
            for (r, row) in rows.iter().enumerate() {
                code |= ((row & code).count_ones() as u8 & 1) << (3 - r);
            }
            for j in 0..8 {
                bits[at + l * j + i] = code >> (7 - j) & 1 != 0;
            }
        }
    }

    /// A frame carrying two data blocks and one configuration channel.
    fn frame(conf: [u8; 7], a: [u8; 13], b: [u8; 13]) -> Vec<bool> {
        let mut bits = vec![false; dfm::FRAME_BITS];
        for (k, bit) in bits.iter_mut().take(16).enumerate() {
            *bit = dfm::HEADER >> (15 - k) & 1 != 0;
        }
        put(&mut bits, 16, &conf);
        put(&mut bits, 16 + 56, &a);
        put(&mut bits, 16 + 160, &b);
        bits
    }

    fn block(id: u8, payload: u64) -> [u8; 13] {
        let mut nibs = [0u8; 13];
        for (i, nib) in nibs.iter_mut().enumerate().take(12) {
            *nib = (payload >> (4 * (11 - i))) as u8 & 0xF;
        }
        nibs[12] = id;
        nibs
    }

    fn a_flight() -> Vec<bool> {
        let lat = 533_500_000u32 as u64;
        let lon = (-50_000_000i32) as u32 as u64;
        let alt = 471_222u64;
        let blocks = [
            block(0, 2 << 24 | 0x71 << 16),
            block(1, 40_000),
            block(2, lat << 16 | 900),
            block(3, lon << 16 | 20_000),
            block(4, alt << 16 | 500),
            block(5, 0),
            block(6, 0),
            block(7, 0),
            block(8, 0x7E9 << 36 | 3 << 32 | 13 << 27 | 5 << 22 | 42 << 16 | 11 << 8),
        ];
        // Channel 0xB with both halves of the serial, which names it a
        // DFM-17.
        let conf = |half: u8, v: u16| {
            let mut c = [0u8; 7];
            c[0] = 0xB;
            c[1] = 0xC;
            for (i, n) in (0..4).map(|i| (v >> (4 * (3 - i))) as u8 & 0xF).enumerate() {
                c[2 + i] = n;
            }
            c[6] = half;
            c
        };
        let mut bits = Vec::new();
        for (i, pair) in blocks.chunks(2).enumerate() {
            let b = pair.get(1).copied().unwrap_or(pair[0]);
            let c = match i {
                0 => conf(0, 0x0163),
                _ => conf(1, 0x4A21),
            };
            bits.extend(frame(c, pair[0], b));
        }
        bits
    }

    /// The whole chain this file is, on samples: the chip clock, the header
    /// search, the Manchester decode, and the gathering that turns nine
    /// blocks into one fix.
    #[test]
    fn a_keyed_flight_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 48_000.0;
            let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
            wire.extend(chips(&a_flight()).into_iter().map(|c| c != inverted));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(&wire, rate, BAUD, 2_400.0, 0.5);

            let mut n = DfmNode::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                n.sync.as_mut().unwrap().process(block, n.framer.sink());
                got.extend(n.framer.take());
            }
            assert_eq!(got.len(), 1, "{} records, inverted {inverted}", got.len());
            assert_eq!(n.frames(), 5, "{} frames read", n.frames());

            let d = decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
            assert_eq!(d.field("model").map(|v| v.to_string()).as_deref(), Some("DFM-17"));
            assert_eq!(
                d.field("serial").map(|v| v.to_string()),
                Some(format!("{}", 0x0163_4A21u32))
            );
            let p = d.position.expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
            assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.01, "{:?}", p.altitude_m);
        }
    }

    /// Twenty seconds of noise produces no records. The header is 32 chips
    /// with four of slack, so candidates turn up regularly and the Hamming
    /// code is what refuses them.
    #[test]
    fn noise_produces_no_records() {
        let rate = 48_000.0;
        let mut seed = 0x2468_ace0_1357_9bdfu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = DfmNode::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut records = 0;
        for block in iq.chunks(4096) {
            n.sync.as_mut().unwrap().process(block, n.framer.sink());
            records += n.framer.take().len();
            n.framer.trim();
        }
        assert_eq!(records, 0, "{records} records out of twenty seconds of noise");
    }
}
