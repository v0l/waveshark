//! Meisei iMS-100 radiosondes as a stage: a source's stream in, gathered
//! records out.
//!
//! 2400 chips a second, biphase coded, so the data rate is 1200 bits a
//! second and each half-frame of 300 bits takes a quarter of a second. The
//! waveform is [`dsp::fsk::BitSync`] at the chip rate, the frame is
//! [`decode::meisei`], and this is the wire between them: find one of the two
//! half-frame headers in the chips, take the coding off, read the six
//! codewords, and hand the half to the gatherer that pairs them.
//!
//! The coding carries no polarity of its own: a bit is whether the two chips
//! of a pair are the same, which does not change when the receiver turns the
//! signal over. Only the header search has to look both ways, and only to
//! find the pairs.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::meisei;
pub use decode::meisei::read;
pub use decode::meisei::this_year;
use dsp::fsk::BitSync;
use identify::Signal;
pub use identify::meisei::BAND;
pub use identify::meisei::BAUD;
pub use identify::meisei::CHANNEL_WIDTH_HZ;
pub use identify::meisei::Meisei;
pub use identify::meisei::OCCUPIED_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct MeiseiNode {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    framer: meisei::Framer,
}

impl Default for MeiseiNode {
    fn default() -> Self {
        Self::new()
    }
}

impl MeiseiNode {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            framer: meisei::Framer::new(),
        }
    }

    /// Half-frames whose codewords all came through the BCH.
    pub fn halves(&self) -> u64 {
        self.framer.halves()
    }

    /// Records gathered, which is what reaches the bus.
    pub fn records(&self) -> u64 {
        self.framer.records()
    }
}

impl Simple for MeiseiNode {
    fn name(&self) -> &str {
        "ims100"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ims100 reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "ims100 needs at least {} S/s for its {BAUD} chips a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 1.0);
        let mut out = i.spec.with_kind(PortKind::Packets);
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
            o.packets_mut().push(self.meter.packet_now(record));
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

impl Protocol for Meisei {
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

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || bytes.len() != meisei::RECORD {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("ims100")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "ims100",
    summary: "Meisei iMS-100 radiosonde frames, 2400 baud biphase FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(MeiseiNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bits as chips: the level turns over at every bit, and again in the
    /// middle of a zero.
    fn keyed(bits: &[bool]) -> Vec<bool> {
        let mut chips = Vec::with_capacity(bits.len() * 2);
        let mut level = false;
        for &bit in bits {
            level = !level;
            chips.push(level);
            if !bit {
                level = !level;
            }
            chips.push(level);
        }
        chips
    }

    /// One half-frame's bits, built the way the decoder's own test does.
    fn half(kind: meisei::Half, words: &[u16; meisei::WORDS]) -> Vec<bool> {
        const GEN: u64 = 0b1_0101_0011_1001;
        let header = match kind {
            meisei::Half::First => meisei::HEADER_A,
            meisei::Half::Second => meisei::HEADER_B,
        };
        let mut bits: Vec<bool> = (0..24).map(|i| header >> (23 - i) & 1 != 0).collect();
        for block in 0..6 {
            let mut data: Vec<bool> = Vec::with_capacity(34);
            for w in 0..2 {
                let value = words[block * 2 + w];
                data.extend((0..16).map(|i| value >> (15 - i) & 1 != 0));
                data.push(value.count_ones().is_multiple_of(2));
            }
            let mut acc = 0u64;
            for bit in &data {
                acc = acc << 1 | u64::from(*bit);
                if acc >> 12 & 1 != 0 {
                    acc ^= GEN;
                }
            }
            for _ in 0..12 {
                acc <<= 1;
                if acc >> 12 & 1 != 0 {
                    acc ^= GEN;
                }
            }
            bits.extend(data);
            bits.extend((0..12).map(|i| acc >> (11 - i) & 1 != 0));
        }
        bits
    }

    fn a_flight() -> ([u16; meisei::WORDS], [u16; meisei::WORDS]) {
        let mut a = [0u16; meisei::WORDS];
        a[0] = 1_746;
        a[10] = 20_500;
        a[11] = 5 << 8 | 42;
        let mut b = [0u16; meisei::WORDS];
        b[0] = 13 * 1000 + 3 * 10 + 5;
        let (lat, lon) = (53_210_000u32, 5_000_000u32);
        b[1] = (lat >> 16) as u16;
        b[2] = lat as u16;
        b[3] = (lon >> 16) as u16;
        b[4] = lon as u16;
        let alt = 471_222u32;
        b[5] = (alt >> 8) as u16;
        b[6] = (alt as u16) << 8;
        b[9] = 20_000;
        b[10] = 1_749;
        b[11] = a[10]
            .wrapping_add(a[11])
            .wrapping_add(b[..11].iter().fold(0u16, |s, w| s.wrapping_add(*w)));
        (a, b)
    }

    /// The whole chain this file is, on samples, and both ways up: the chip
    /// clock, the header search, the biphase decode, the BCH and the pairing
    /// of two half-frames into one fix.
    #[test]
    fn a_keyed_pair_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 48_000.0;
            let (a, b) = a_flight();
            let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
            wire.extend(keyed(&half(meisei::Half::First, &a)));
            wire.extend(keyed(&half(meisei::Half::Second, &b)));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(
                &wire.iter().map(|c| c != &inverted).collect::<Vec<_>>(),
                rate,
                BAUD,
                1_200.0,
                0.5,
            );

            let mut n = MeiseiNode::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for block in iq.chunks(2048) {
                n.sync.as_mut().unwrap().process(block, n.framer.sink());
                got.extend(n.framer.take());
            }
            assert_eq!(got.len(), 1, "{} records, inverted {inverted}", got.len());
            assert_eq!(n.halves(), 2, "{} half-frames read", n.halves());

            let d = read(&got[0]).expect("a decode");
            assert_eq!(d.id, "meisei");
            let p = d.placed().expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon - 5.0).abs() < 1e-6, "{}", p.lon);
        }
    }

    /// Twenty seconds of noise produces no records.
    #[test]
    fn noise_produces_no_records() {
        let rate = 48_000.0;
        let mut seed = 0x0bad_c0de_dead_10ccu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = MeiseiNode::new();
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
