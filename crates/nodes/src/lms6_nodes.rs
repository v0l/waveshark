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
use dsp::conv::{Code, Ends, Viterbi};
use dsp::fsk::BitSync;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Coded bits a second. Half of them survive the trellis, so the frame rate
/// is one a second.
pub const BAUD: f64 = 4_800.0;

/// The channel an LMS6 is tuned to.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// What the signal occupies.
pub const OCCUPIED_HZ: f64 = 12_000.0;

/// The meteorological aids band.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// This sonde's own rate 1/2 code: constraint seven, and neither of the two
/// polynomials every other standard here uses.
const CODE: Code = Code { constraint: 7, polys: &[0x4F, 0x6D] };

/// Bytes in a block: the sync and the Reed-Solomon codeword.
const BLOCK: usize = lms6::BLOCK_SYNC.len() + lms6::CODEWORD;

/// Bits in a block once the trellis has run, and coded bits on the air. One
/// byte of tail is read with it, as the decoder it came from does, so the
/// last bits of the block have something behind them in the trellis.
const BLOCK_BITS: usize = (BLOCK + 1) * 8;
const CODED_BITS: usize = BLOCK_BITS * 2;

/// Coded bits held while looking for a block: two blocks and a bit.
const MAX_BITS: usize = CODED_BITS * 2;

/// Coded bits of the sync allowed to be wrong. The Reed-Solomon behind it
/// refuses what a false sync produces, and a sonde at the edge of reception
/// loses a few.
const SYNC_SLACK: u32 = 8;

pub struct Lms6Node {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    bits: Vec<bool>,
    /// Coded bits already searched and known not to start a block.
    scanned: usize,
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
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            bits: Vec::new(),
            scanned: 0,
            frames: 0,
        }
    }

    /// Frames whose own check held.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    fn search(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let sync = sync_bits();
        let mut at = self.scanned;
        while at + sync.len() <= self.bits.len() {
            let Some(inverted) = matches(&self.bits, at, &sync) else {
                at += 1;
                continue;
            };
            if at + CODED_BITS > self.bits.len() {
                self.scanned = at;
                return out;
            }
            match self.read_block(at, inverted) {
                Some(frame) => {
                    self.frames += 1;
                    out.push(frame);
                    self.bits.drain(..at + CODED_BITS);
                    at = 0;
                    self.scanned = 0;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }

    /// The block starting at coded bit `at`: the trellis, the block code,
    /// and the frame inside it.
    fn read_block(&self, at: usize, inverted: bool) -> Option<Vec<u8>> {
        // The trellis reads soft values, and what a hard slicer produces is
        // the same thing with every value at full confidence.
        let soft: Vec<f32> = self.bits[at..at + CODED_BITS]
            .iter()
            .enumerate()
            .map(|(i, &b)| {
                // The second of each coded pair goes out inverted, and the
                // whole stream may be over as well.
                let bit = b != inverted && i % 2 == 0 || b == inverted && i % 2 == 1;
                match bit {
                    true => -1.0,
                    false => 1.0,
                }
            })
            .collect();
        let decoded =
            Viterbi::decode_block(CODE, &soft, dsp::conv::P_1_2, BLOCK_BITS, Ends::Anywhere);
        let bytes: Vec<u8> =
            decoded.chunks(8).map(|b| b.iter().fold(0u8, |v, bit| v << 1 | (*bit & 1))).collect();
        if bytes.len() < BLOCK || bytes[..lms6::BLOCK_SYNC.len()] != lms6::BLOCK_SYNC {
            return None;
        }
        let mut block = bytes[lms6::BLOCK_SYNC.len()..BLOCK].to_vec();
        lms6::correct(&mut block)?;
        let frame = block[..lms6::FRAME].to_vec();
        lms6::parse(&frame).is_some().then_some(frame)
    }
}

/// The block sync as it goes out: through the same code the data does, with
/// the second of each pair inverted.
fn sync_bits() -> Vec<bool> {
    let mut enc = dsp::conv::Encoder::new(CODE);
    let mut coded = Vec::new();
    for byte in lms6::BLOCK_SYNC {
        for k in (0..8).rev() {
            enc.push(byte >> k & 1, &mut coded);
        }
    }
    coded.iter().enumerate().map(|(i, b)| (*b == 1) != (i % 2 == 1)).collect()
}

/// Whether the sync sits at `at`, and which way up.
fn matches(bits: &[bool], at: usize, sync: &[bool]) -> Option<bool> {
    let mut wrong = [0u32; 2];
    for (k, &want) in sync.iter().enumerate() {
        wrong[usize::from(bits[at + k] == want)] += 1;
    }
    match (wrong[0] <= SYNC_SLACK, wrong[1] <= SYNC_SLACK) {
        (true, _) => Some(false),
        (_, true) => Some(true),
        _ => None,
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
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 1.0);
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(i.spec.rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(s)) = (i.as_iq(), self.sync.as_mut()) else {
            return Ok(());
        };
        self.meter.feed(iq);
        s.process(iq, &mut self.bits);
        for frame in self.search() {
            o.frames_mut().push(self.meter.frame(frame));
        }
        if self.bits.len() > MAX_BITS {
            let drop = self.bits.len() - MAX_BITS;
            self.bits.drain(..drop);
            self.scanned = self.scanned.saturating_sub(drop);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.meter.reset();
        self.bits.clear();
        self.scanned = 0;
        if let Some(s) = &mut self.sync {
            s.reset();
        }
    }
}

/// What the protocols node makes of an LMS6 frame.
pub fn lms6_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let r = lms6::parse(bytes)?;
    let (h, m, s) = r.utc;
    let serial = format!("{}", r.serial);
    let fields: Vec<(String, common::Value)> = vec![
        ("model".into(), common::Value::Text("LMS6".into())),
        ("serial".into(), common::Value::Text(serial.clone())),
        ("frame".into(), common::Value::Int(r.frame_no as i64)),
        ("altitude_m".into(), common::Value::Float(r.altitude_m)),
        ("climb_ms".into(), common::Value::Float(r.climb_ms)),
        ("speed_kt".into(), common::Value::Float(r.speed_kt)),
        ("course_deg".into(), common::Value::Float(r.course_deg)),
        ("utc".into(), common::Value::Text(format!("{h:02}:{m:02}:{s:06.3}"))),
    ];

    let mut d = Decoded::bytes("lms6", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(true))
        .with_text(r.summary())
        .with_detail(format!("LMS6, frame {}", r.frame_no))
        .with_fields(fields)
        .by(common::Identity::new("lms6", serial).made_by("Lockheed Martin"));
    if r.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: r.altitude_m,
                climb_ms: r.climb_ms,
                // The frame carries no battery voltage.
                battery_v: f32::NAN,
                satellites: 0,
                descending: r.climb_ms < -1.0,
                sensors: None,
            })
            .at_position(common::Position {
                lat: r.lat_deg,
                lon: r.lon_deg,
                altitude_m: Some(r.altitude_m),
                speed_kt: Some(r.speed_kt),
                course_deg: Some(r.course_deg),
            });
    }
    Some(d)
}

pub struct Lms6;

impl Protocol for Lms6 {
    fn id(&self) -> &'static str {
        "lms6"
    }
    fn label(&self) -> &'static str {
        "lms6"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["lms6-403", "lockheed"]
    }
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }
    fn default_hz(&self) -> f64 {
        403_000_000.0
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 4.0 * BAUD,
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) || bytes.len() != lms6::FRAME {
            return None;
        }
        Some(lms6_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
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
                let mut bits = Vec::new();
                n.sync.as_mut().unwrap().process(chunk, &mut bits);
                n.bits.extend(bits);
                got.extend(n.search());
            }
            assert_eq!(got.len(), 1, "{} frames, inverted {inverted}", got.len());
            assert_eq!(got[0][..], block[5..5 + lms6::FRAME], "not the bytes that were keyed");

            let d = lms6_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
            assert_eq!(d.field("serial").map(|v| v.to_string()).as_deref(), Some("10597059"));
            let p = d.position.expect("a position");
            assert!((p.lat - 53.35).abs() < 1e-6, "{}", p.lat);
            assert!((p.lon + 5.0).abs() < 1e-6, "{}", p.lon);
            assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.01, "{:?}", p.altitude_m);
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
            let mut bits = Vec::new();
            n.sync.as_mut().unwrap().process(chunk, &mut bits);
            n.bits.extend(bits);
            frames += n.search().len();
            if n.bits.len() > MAX_BITS {
                let drop = n.bits.len() - MAX_BITS;
                n.bits.drain(..drop);
                n.scanned = n.scanned.saturating_sub(drop);
            }
        }
        assert_eq!(frames, 0, "{frames} frames out of twenty seconds of noise");
    }
}
