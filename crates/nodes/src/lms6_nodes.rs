//! Lockheed Martin LMS6 radiosondes as a stage: a source's stream in,
//! frames out.
//!
//! The most protected sonde in the band, and so the deepest stack: 4800
//! coded bits a second, rate a half convolutional with the second of each
//! pair inverted on the air, a 260-byte block behind that, and a CCSDS
//! Reed-Solomon codeword inside the block holding one 223-byte frame. Only
//! the last of those belongs to the protocol, so the waveform is here, the
//! block code and the frame are in [`decode::lms6`], and the trellis is
//! `dsp::conv` parameterised with this sonde's own polynomials.
//!
//! The block sync is looked for in the coded bits rather than in the decoded
//! ones: it is what fixes both the pair alignment the trellis needs and the
//! polarity the receiver may have turned over.
//!
//! Waveform, polynomials, block layout and the reversed codeword order are
//! from zilog80's `rs1729/RS`, `demod/mod/lms6Xmod.c`.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::lms6;
pub use decode::lms6::read;
use dsp::fsk::BitSync;
use identify::Signal;
pub use identify::lms6::BAND;
pub use identify::lms6::BAUD;
pub use identify::lms6::CHANNEL_WIDTH_HZ;
pub use identify::lms6::Lms6;
pub use identify::lms6::OCCUPIED_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct Lms6Node {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    framer: lms6::Framer,
    frames: u64,
}

impl Default for Lms6Node {
    fn default() -> Self {
        Self::new()
    }
}

impl Lms6Node {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6).keyed_as(common::Modulation::Fsk2),
            framer: lms6::Framer::new(),
            frames: 0,
        }
    }

    /// Frames whose own check held.
    pub fn frames(&self) -> u64 {
        self.frames
    }
}

impl Simple for Lms6Node {
    fn name(&self) -> &str {
        "lms6"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("lms6 reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "lms6 needs at least {} S/s for its {BAUD} coded bits a second",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 1.0)
            .keyed_as(common::Modulation::Fsk2);
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
        for frame in self.framer.take() {
            o.packets_mut().push(self.meter.packet_now(frame));
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

impl Protocol for Lms6 {
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

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || bytes.len() != lms6::FRAME {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("lms6")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "lms6",
    summary: "Lockheed Martin LMS6 radiosonde frames, 4800 baud coded FSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Lms6Node::new()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use decode::lms6::CODE;

    /// A block on the air: the sync and the codeword through the trellis,
    /// with the second of each coded pair inverted.
    fn keyed(block: &[u8]) -> Vec<bool> {
        let mut enc = dsp::conv::Encoder::new(CODE);
        let mut coded = Vec::new();
        for byte in block.iter().chain(std::iter::once(&0u8)) {
            for k in (0..8).rev() {
                enc.push(byte >> k & 1, &mut coded);
            }
        }
        coded.iter().enumerate().map(|(i, b)| (*b == 1) != (i % 2 == 1)).collect()
    }

    /// A block carrying one frame: the sync, the frame, and the parity that
    /// makes the 255 bytes a codeword.
    fn a_block() -> Vec<u8> {
        const B60B60: f64 = (1u32 << 30) as f64 / 90.0;
        let deg = |d: f64| ((d * B60B60) as i32).to_be_bytes().to_vec();
        let vel = |ms: f64| ((ms * 1000.0) as i32).to_be_bytes()[1..].to_vec();
        let mut frame = vec![0u8; lms6::FRAME];
        frame[..4].copy_from_slice(&lms6::SYNC);
        let put = |f: &mut Vec<u8>, at: usize, b: Vec<u8>| {
            f[4 + at..4 + at + b.len()].copy_from_slice(&b)
        };
        put(&mut frame, 0x00, 0x00_A1_B2_C3u32.to_be_bytes().to_vec());
        put(&mut frame, 0x04, 4_321u16.to_be_bytes().to_vec());
        put(&mut frame, 0x06, 452_540_000u32.to_be_bytes().to_vec());
        put(&mut frame, 0x0E, deg(53.35));
        put(&mut frame, 0x12, deg(-5.0));
        put(&mut frame, 0x16, 4_712_220i32.to_be_bytes().to_vec());
        put(&mut frame, 0x1A, vel(9.0));
        put(&mut frame, 0x20, vel(5.0));
        let cs = decode::bits::crc16(&frame[..221], 0x1021, 0x0000).to_be_bytes();
        frame[221..223].copy_from_slice(&cs);

        let mut codeword = vec![0u8; lms6::CODEWORD];
        codeword[..lms6::FRAME].copy_from_slice(&frame);
        // The sonde sends the codeword highest symbol first, so the parity
        // is computed on the reversed word and put back the same way.
        codeword.reverse();
        let parity = lms6::code().encode(&codeword[lms6::CODEWORD - lms6::MESSAGE..]);
        codeword[..lms6::CODEWORD - lms6::MESSAGE].copy_from_slice(&parity);
        codeword.reverse();

        let mut block = lms6::BLOCK_SYNC.to_vec();
        block.extend(codeword);
        block
    }

    /// The whole chain this file is, on samples and both ways up: the bit
    /// clock, the sync search, the trellis, the block code and the frame's
    /// own check.
    #[test]
    fn a_keyed_block_is_read_off_the_samples() {
        for inverted in [false, true] {
            let rate = 48_000.0;
            let block = a_block();
            let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
            wire.extend(keyed(&block).into_iter().map(|b| b != inverted));
            wire.extend((0..40).map(|i| i % 2 == 0));
            let iq = dsp::fsk::modulate(&wire, rate, BAUD, 2_400.0, 0.5);

            let mut n = Lms6Node::new();
            n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
            let mut got = Vec::new();
            for chunk in iq.chunks(2048) {
                n.sync.as_mut().unwrap().process(chunk, n.framer.sink());
                got.extend(n.framer.take());
            }
            assert_eq!(got.len(), 1, "{} frames, inverted {inverted}", got.len());
            assert_eq!(got[0][..], block[5..5 + lms6::FRAME], "not the bytes that were keyed");

            let d = read(&got[0]).expect("a decode");
            assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("10597059"));
            let p = d.placed().expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
            assert!(
                d.facts.iter().any(|f| matches!(
                    f,
                    common::packet::Fact::Sensed(r)
                        if r.quantity == common::packet::Quantity::Altitude
                            && (r.value - 4_712.22).abs() < 0.01
                )),
                "{d:?}"
            );
        }
    }

    /// Twenty seconds of noise produces no frames.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 48_000.0;
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = Lms6Node::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut frames = 0;
        for chunk in iq.chunks(4096) {
            n.sync.as_mut().unwrap().process(chunk, n.framer.sink());
            frames += n.framer.take().len();
            n.framer.trim();
        }
        assert_eq!(frames, 0, "{frames} frames out of twenty seconds of noise");
    }
}
