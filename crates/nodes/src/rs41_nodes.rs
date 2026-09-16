//! Vaisala RS41 radiosondes as a stage: a source's stream in, sonde frames
//! out.
//!
//! A sonde is a balloon-borne transmitter keying 4800 baud GFSK on a channel
//! between 400 and 406 MHz, one 320 or 518 byte frame a second for the two
//! hours it takes to reach 35 km and burst. The waveform is
//! [`dsp::fsk::BitSync`] parameterised at that baud, the frame is
//! [`decode::rs41`], and this is the wire between them: find the header in
//! the bit stream, take the scrambler off, let the Reed-Solomon code repair
//! what it can, and put the frame on the packet bus.
//!
//! The bits arrive least significant first, and the scrambler covers the
//! header as well as the body, so there are two header constants and using
//! the wrong one finds nothing: `rs41::HEADER_AIR` is what is correlated in
//! the bit stream and `rs41::HEADER` is what that descrambles to.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::rs41;
use dsp::fsk::BitSync;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Symbols a second.
pub const BAUD: f64 = 4_800.0;

/// The channel a sonde is tuned to, which is the 10 kHz raster Vaisala
/// steps through the band on. The signal itself is 9.6 kHz, so the channel
/// is barely wider than what is in it and the two neighbours are clear.
pub const CHANNEL_WIDTH_HZ: f64 = 10_000.0;

/// What the signal itself occupies: 4800 baud keyed 2.4 kHz either way is
/// 9.6 kHz by Carson's rule, which is also what Vaisala quotes.
pub const OCCUPIED_HZ: f64 = 9_600.0;

/// The band sondes are launched into (ITU meteorological aids, region 1 and
/// beyond). Vaisala tunes an RS41 anywhere in it in 10 kHz steps.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Bits held while looking for a header. Two long frames and the gap
/// between them: enough that a frame straddling two blocks is never lost,
/// and bounded so a channel with nothing on it cannot grow.
const MAX_BITS: usize = rs41::FRAME_AUX * 8 * 3;

/// Header bit errors tolerated. The header is 64 bits and a sonde at the
/// edge of reception loses a few; more than this and it is not a header.
/// Four leaves a false alarm rate of about one in a million bit positions,
/// which at 4800 baud is one spurious search every three minutes and costs
/// nothing, because the Reed-Solomon code then refuses it.
const HEADER_SLACK: u32 = 4;

pub struct Rs41Node {
    sync: Option<BitSync>,
    meter: crate::FrameMeter,
    bits: Vec<bool>,
    /// Bits already searched and known not to start a header. Only appended
    /// to, so what was rejected stays rejected.
    scanned: usize,
    frames: u64,
}

impl Default for Rs41Node {
    fn default() -> Self {
        Self::new()
    }
}

impl Rs41Node {
    pub fn new() -> Self {
        Self {
            sync: None,
            meter: crate::FrameMeter::new(1.0, 0, 0.6),
            bits: Vec::new(),
            scanned: 0,
            frames: 0,
        }
    }

    /// Frames whose Reed-Solomon codewords both decoded, since the node was
    /// made.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Look for headers in the bits held, returning every frame behind one.
    fn search(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let header = header_bits();
        let mut at = self.scanned;
        while at + header.len() <= self.bits.len() {
            let mut wrong = 0u32;
            for (k, &want) in header.iter().enumerate() {
                if self.bits[at + k] != want {
                    wrong += 1;
                    if wrong > HEADER_SLACK {
                        break;
                    }
                }
            }
            if wrong > HEADER_SLACK {
                at += 1;
                continue;
            }
            match self.read_frame(at) {
                // A header with a frame behind it: take both out of the
                // buffer so the search does not walk back into them.
                Some(Some(frame)) => {
                    let used = frame.len() * 8;
                    out.push(frame);
                    self.bits.drain(..at + used);
                    at = 0;
                    self.scanned = 0;
                }
                // A header whose frame has not all arrived: wait here.
                Some(None) => {
                    self.scanned = at;
                    return out;
                }
                None => at += 1,
            }
        }
        self.scanned = at;
        out
    }

    /// Read the frame starting at bit `at`. `None` where those bits are not
    /// a frame, `Some(None)` where not enough of them have arrived.
    fn read_frame(&self, at: usize) -> Option<Option<Vec<u8>>> {
        // Not enough bits yet is not the same answer as not a frame: the
        // search resumes where it stopped, so treating a short buffer as a
        // rejection walks the cursor past the header and loses the frame
        // that was about to arrive.
        let Some(mut probe) = pack(&self.bits, at, rs41::FRAME_STD) else { return Some(None) };
        rs41::descramble(&mut probe);
        // The length marker sits in the data, so it has to be read before
        // the code has passed on it; a wrong bit here costs one frame.
        let len = rs41::frame_len(probe[rs41::DATA_AT])?;
        let mut frame = if len == rs41::FRAME_STD {
            probe
        } else {
            let Some(mut long) = pack(&self.bits, at, len) else { return Some(None) };
            rs41::descramble(&mut long);
            long
        };
        rs41::correct(&mut frame)?;
        Some(Some(frame))
    }
}

/// The header as it arrives: eight bytes, least significant bit first.
fn header_bits() -> Vec<bool> {
    rs41::HEADER_AIR.iter().flat_map(|b| (0..8).map(move |k| b >> k & 1 != 0)).collect()
}

/// `len` bytes from bit `at`, least significant bit first, or `None` where
/// the bits are not all there.
fn pack(bits: &[bool], at: usize, len: usize) -> Option<Vec<u8>> {
    if at + len * 8 > bits.len() {
        return None;
    }
    Some(
        bits[at..at + len * 8]
            .chunks(8)
            .map(|c| c.iter().enumerate().fold(0u8, |b, (k, &s)| b | (s as u8) << k))
            .collect(),
    )
}

impl Simple for Rs41Node {
    fn name(&self) -> &str {
        "rs41"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("rs41 reads complex baseband"));
        }
        let s = BitSync::with_bandwidth(i.spec.rate, BAUD, OCCUPIED_HZ);
        if !s.usable() {
            return Err(common::Error::other(format!(
                "rs41 needs at least {} S/s for its {BAUD} baud",
                4.0 * BAUD
            )));
        }
        self.sync = Some(s);
        // A frame is 534 ms of air, so the ring has to be long enough to
        // give one back once it has decoded.
        self.meter = crate::FrameMeter::new(i.spec.rate, i.spec.center.0, 0.6);
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
            self.frames += 1;
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

/// What the protocols node makes of a sonde frame: which balloon it is,
/// where, and how it is flying.
pub fn rs41_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let f = rs41::parse(bytes)?;
    let mut fields: Vec<(String, common::Value)> = vec![
        ("serial".into(), common::Value::Text(f.serial.clone())),
        ("frame".into(), common::Value::Int(f.frame_no as i64)),
        ("battery_v".into(), common::Value::Float(f.battery_v as f64)),
        ("state".into(), common::Value::Text(f.flight.label().into())),
    ];
    if f.has_position() {
        fields.push(("altitude_m".into(), common::Value::Float(f.altitude_m)));
        fields.push(("climb_ms".into(), common::Value::Float(f.climb_ms)));
        fields.push(("speed_kt".into(), common::Value::Float(f.speed_kt)));
        fields.push(("course_deg".into(), common::Value::Float(f.course_deg)));
        fields.push(("satellites".into(), common::Value::Int(f.satellites as i64)));
    }
    if let (Some(w), Some(t)) = (f.gps_week, f.gps_tow_ms) {
        fields.push(("gps_week".into(), common::Value::Int(w as i64)));
        fields.push(("gps_tow_ms".into(), common::Value::Int(t as i64)));
    }
    fields.push(("pcb_temp_c".into(), common::Value::Int(f.pcb_temp_c as i64)));
    if f.bad_blocks > 0 {
        fields.push(("bad_blocks".into(), common::Value::Int(f.bad_blocks as i64)));
    }

    let mut d = Decoded::bytes("rs41", center, 0.0, bytes.to_vec())
        .with_modulation(common::Modulation::Fsk2)
        .with_crc(Some(f.bad_blocks == 0))
        .with_text(f.summary())
        .with_detail(format!("frame {}, {:.1} V, {}", f.frame_no, f.battery_v, f.flight.label()))
        .with_fields(fields)
        .by(common::Identity::new("vaisala", f.serial.clone()).made_by("Vaisala"));
    if f.has_position() {
        d = d
            .reporting(common::ReportDetail::Sonde {
                altitude_m: f.altitude_m,
                climb_ms: f.climb_ms,
                battery_v: f.battery_v,
                satellites: f.satellites,
                descending: f.flight == rs41::Flight::Descent,
                sensors: f.meas.map(|meas| common::SondeSensors { meas, calibration: f.subframe }),
            })
            .at_position(common::Position {
                lat: f.lat_deg,
                lon: f.lon_deg,
                altitude_m: Some(f.altitude_m),
                speed_kt: Some(f.speed_kt),
                course_deg: Some(f.course_deg),
            });
    }
    Some(d)
}

pub struct Rs41;

impl Protocol for Rs41 {
    fn id(&self) -> &'static str {
        "rs41"
    }
    fn label(&self) -> &'static str {
        "rs41"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["radiosonde", "sonde", "vaisala"]
    }
    /// The meteorological aids allocation. Placed by band rather than
    /// anywhere: 4800 baud FSK in a 15 kHz channel is a shape a great many
    /// things have, and outside this band none of them is a sonde.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }
    /// The middle of the band. Europe launches mostly between 402 and 405
    /// MHz, and a sonde is found by scanning rather than by being known.
    fn default_hz(&self) -> f64 {
        403_000_000.0
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            // Four samples a symbol is the demodulator's floor.
            min_rate_hz: 4.0 * BAUD,
            // Ten, which leaves the timing loop room to interpolate.
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }
    /// The six megahertz of the sonde band, which nothing else here claims.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) {
            return None;
        }
        Some(rs41_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("rs41")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "rs41",
    summary: "Vaisala RS41 radiosonde frames, 4800 baud GFSK at 400 to 406 MHz",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Rs41Node::new()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A frame made here, keyed at 4800 baud and read back off the samples,
    /// which is the whole chain this file is: bit clock, header search,
    /// descramble, Reed-Solomon, parse.
    pub(super) fn a_frame(serial: &[u8; 8], frame_no: u16) -> Vec<u8> {
        let mut f = vec![0u8; rs41::FRAME_STD];
        f[..8].copy_from_slice(&rs41::HEADER);
        f[56] = 0x0F;
        let mut body = vec![0u8; 0x28];
        body[..2].copy_from_slice(&frame_no.to_le_bytes());
        body[2..10].copy_from_slice(serial);
        body[0x0A] = 27;
        body[0x0D] = 0x01;
        let mut at = 57;
        let put = |id: u8, body: &[u8], f: &mut Vec<u8>, at: &mut usize| {
            f[*at] = id;
            f[*at + 1] = body.len() as u8;
            f[*at + 2..*at + 2 + body.len()].copy_from_slice(body);
            let crc = rs41::block_crc(body).to_le_bytes();
            f[*at + 2 + body.len()..*at + 4 + body.len()].copy_from_slice(&crc);
            *at += 4 + body.len();
        };
        put(0x79, &body, &mut f, &mut at);
        // A position block placing it over the Irish Sea at 4712 m above
        // the ellipsoid, which is where these earth-centred metres are.
        let mut pos = vec![0u8; 0x15];
        let (x, y, z) = (3_800_000.0f64, -340_000.0f64, 5_100_000.0f64);
        pos[0..4].copy_from_slice(&((x * 100.0) as i32).to_le_bytes());
        pos[4..8].copy_from_slice(&((y * 100.0) as i32).to_le_bytes());
        pos[8..12].copy_from_slice(&((z * 100.0) as i32).to_le_bytes());
        pos[0x12] = 11;
        put(0x7B, &pos, &mut f, &mut at);
        let pad = vec![0u8; rs41::FRAME_STD - at - 4];
        put(0x76, &pad, &mut f, &mut at);
        assert_eq!(at, rs41::FRAME_STD);
        rs41::protect(&mut f);
        f
    }

    /// Frame bytes back onto the air: scrambled from the parity on, then
    /// keyed least significant bit first.
    pub(super) fn key(frame: &[u8], rate: f64) -> Vec<common::C32> {
        let mut wire = frame.to_vec();
        rs41::descramble(&mut wire);
        assert_eq!(wire[..8], rs41::HEADER_AIR);
        let bits: Vec<bool> =
            wire.iter().flat_map(|b| (0..8).map(move |k| b >> k & 1 != 0)).collect();
        // A run of alternating symbols first, so the timing loop has
        // something to lock to before the header arrives, as a sonde's own
        // preamble gives it.
        let mut wire: Vec<bool> = (0..200).map(|i| i % 2 == 0).collect();
        wire.extend(bits);
        // And a tail, because the last bit is only decided once the symbol
        // after it has gone through the filter.
        wire.extend((0..40).map(|i| i % 2 == 0));
        dsp::fsk::modulate(&wire, rate, BAUD, 2_400.0, 0.5)
    }

    #[test]
    fn a_keyed_frame_is_read_off_the_samples() {
        let rate = 48_000.0;
        let frame = a_frame(b"W1234567", 900);
        let iq = key(&frame, rate);
        let mut n = Rs41Node::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut got = Vec::new();
        for block in iq.chunks(2048) {
            let mut bits = Vec::new();
            n.sync.as_mut().unwrap().process(block, &mut bits);
            n.bits.extend(bits);
            got.extend(n.search());
        }
        assert_eq!(got.len(), 1, "{} frames off one transmission", got.len());
        assert_eq!(got[0], frame, "the bytes are not the ones that were keyed");
        let d = rs41_decoded(&got[0], common::Hz(403_000_000)).expect("a decode");
        assert_eq!(d.field("serial").map(|v| v.to_string()).as_deref(), Some("W1234567"));
        assert_eq!(d.crc_ok, Some(true));
        let p = d.position.expect("a position");
        assert!((p.lat - 53.385_054).abs() < 1e-5, "{}", p.lat);
        assert!((p.lon + 5.112_850).abs() < 1e-5, "{}", p.lon);
        assert!((p.altitude_m.unwrap() - 4_712.22).abs() < 0.01, "{:?}", p.altitude_m);
    }

    /// Noise on the channel produces no frames at all. A header search with
    /// four bits of slack finds candidates in noise regularly, and the only
    /// thing that stops one being reported is the code refusing it.
    #[test]
    fn noise_produces_no_frames() {
        let rate = 48_000.0;
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<common::C32> =
            (0..rate as usize * 20).map(|_| common::C32::new(rng(), rng())).collect();
        let mut n = Rs41Node::new();
        n.sync = Some(BitSync::with_bandwidth(rate, BAUD, OCCUPIED_HZ));
        let mut frames = 0;
        for block in iq.chunks(4096) {
            let mut bits = Vec::new();
            n.sync.as_mut().unwrap().process(block, &mut bits);
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
