//! Oregon Scientific v2.1 and v3 sensors.
//!
//! 433.92 MHz, Manchester at 1024 baud, and the first user of the Manchester
//! slicer, which until now had no protocol behind it. v3 is the THGR810, the
//! THN802 and the WGR800 wind meter; v2.1 is the THGR122N, THN132N and
//! RTGN318 families, which carry the same payload layout through an extra
//! layer of coding.
//!
//! Three things make this family awkward, all of them from the wire format
//! rather than from the radio:
//!
//! - The Manchester convention is not fixed. Which half of a symbol carries
//!   the bit depends on where the slicer started, so the sync word is searched
//!   for in both polarities and the payload inverted if it was the second one.
//! - Every nibble arrives with its bits in the opposite order, so the frame
//!   has to be reflected before anything in it means what the layout says.
//! - Values are BCD, including the temperature, which is why 21.7 C is
//!   `0x71 0x02` and not `0xd9`.
//!
//! After reflection:
//!
//! ```text
//! SSSS CDDB TTTT TT?? HH?? KKKK
//! ```
//!
//! - `SSSS` 16 bit sensor id, naming the model
//! - `C`    channel, `DD` rolling device id, `B` battery low in bit 2
//! - `TTTT` temperature, BCD tenths, with a sign bit and a hundreds field
//! - `HH`   humidity, BCD percent
//! - `KKKK` sum of every nibble before it, itself stored with its own two
//!   nibbles swapped, and starting at a nibble that differs per model
//!
//! The checksum is eight bits over a dozen or so nibbles, which is weak, so a
//! match is only accepted where the sensor id is one this decoder knows and
//! the BCD digits are digits.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing};

/// Timings shared by both versions: half a symbol at 1024 baud. Pulses run
/// shorter than pauses on these sensors, which the slicer's rounding absorbs.
fn oregon_timing() -> Timing {
    Timing {
        coding: Coding::Manchester,
        short_us: 488,
        long_us: 976,
        sync_us: 0,
        tolerance_us: 0,
        reset_us: 2400,
    }
}

// ---------------------------------------------------------------------------
// Shared frame handling
// ---------------------------------------------------------------------------

/// Reverse the bits within each nibble, which is the order these sensors send
/// them in. rtl_433 calls this `reflect_nibbles`, and every layout below is
/// written for the reflected frame.
fn reflect_nibbles(b: u8) -> u8 {
    let r = |n: u8| ((n & 1) << 3) | ((n & 2) << 1) | ((n & 4) >> 1) | ((n & 8) >> 3);
    (r(b >> 4) << 4) | r(b & 0x0f)
}

/// What the nibbles before the checksum sum to, and what the frame says they
/// should. The checksum byte is stored with its two nibbles swapped, and on
/// half these layouts it starts at an odd nibble and straddles two bytes.
fn checksum(msg: &[u8], nibble: usize) -> (u8, u8) {
    let whole: u16 = msg[..nibble / 2].iter().map(|b| (b >> 4) as u16 + (b & 0x0f) as u16).sum();
    if nibble % 2 == 1 {
        let sum = ((whole + (msg[nibble / 2] >> 4) as u16) & 0xff) as u8;
        (sum, (msg[nibble / 2] & 0x0f) | (msg[nibble / 2 + 1] & 0xf0))
    } else {
        ((whole & 0xff) as u8, msg[nibble / 2].rotate_left(4))
    }
}

/// Temperature in Celsius: BCD tenths, with a hundreds field and a sign bit.
///
/// Every digit is BCD, so a nibble above nine is a frame that passed an eight
/// bit checksum by luck rather than a reading.
fn temperature_c(msg: &[u8]) -> Result<f64, DecodeError> {
    if [msg[4] & 0x0f, msg[4] >> 4, msg[5] >> 4].iter().any(|n| *n > 9) {
        return Err(DecodeError::Implausible("temperature is not BCD"));
    }
    let mut t = ((msg[5] >> 4) as f64 * 100.0
        + (msg[4] & 0x0f) as f64 * 10.0
        + (msg[4] >> 4) as f64)
        / 10.0
        + (msg[5] & 0x07) as f64 * 100.0;
    if msg[5] & 0x08 != 0 {
        t = -t;
    }
    if !(-50.0..=70.0).contains(&t) {
        return Err(DecodeError::Implausible("temperature out of range"));
    }
    Ok(t)
}

fn humidity_pct(msg: &[u8]) -> Result<u8, DecodeError> {
    if [msg[6] & 0x0f, msg[6] >> 4].iter().any(|n| *n > 9) {
        return Err(DecodeError::Implausible("humidity is not BCD"));
    }
    let h = (msg[6] & 0x0f) * 10 + (msg[6] >> 4);
    if h > 100 {
        return Err(DecodeError::Implausible("humidity above 100%"));
    }
    Ok(h)
}

/// Wind in m/s and degrees, from the WGR800's BCD digits.
fn wind(msg: &[u8]) -> Result<(f64, f64, f64), DecodeError> {
    if [msg[5] & 0x0f, msg[6] >> 4, msg[6] & 0x0f, msg[7] >> 4, msg[7] & 0x0f, msg[8] >> 4]
        .iter()
        .any(|n| *n > 9)
    {
        return Err(DecodeError::Implausible("wind is not BCD"));
    }
    let gust = (msg[5] & 0x0f) as f64 / 10.0 + (msg[6] >> 4) as f64 + (msg[6] & 0x0f) as f64 * 10.0;
    let avg = (msg[7] >> 4) as f64 / 10.0 + (msg[7] & 0x0f) as f64 + (msg[8] >> 4) as f64 * 10.0;
    // The sensor tops out well below this; anything faster is a bad frame.
    if gust > 56.0 || avg > 56.0 {
        return Err(DecodeError::Implausible("wind speed out of range"));
    }
    Ok((gust, avg, (msg[4] >> 4) as f64 * 22.5))
}

/// The fields every one of these frames carries in its first four bytes.
fn common_fields(model: &'static str, msg: &[u8]) -> Report {
    let mut r = Report::new(model);
    r.crc_valid = Some(true);
    r.raw = msg.to_vec();
    r.int("id", ((msg[2] & 0x0f) | (msg[3] & 0xf0)) as i64)
        .int("channel", (msg[2] >> 4) as i64)
        .bool("battery_ok", msg[3] >> 2 & 1 == 0)
}

/// Every bit offset where a 16 bit sync pattern sits.
///
/// Every offset rather than the first, because a burst holds several repeats
/// and the earliest is as likely as any to be the one the detector clipped.
fn sync_offsets(bits: &BitBuffer, sync: u32) -> Vec<usize> {
    (0..bits.len().saturating_sub(16)).filter(|i| bits.extract(*i, 16) == Some(sync)).collect()
}

// ---------------------------------------------------------------------------
// v3
// ---------------------------------------------------------------------------

pub struct OregonV3;

/// Sync word ending the preamble, in the polarity the layout is written in,
/// and the same sync inverted, which is how it arrives when the slicer locks
/// onto the other half of the symbol.
const V3_SYNC: [(u32, bool); 2] = [(0x0005, false), (0xfffa, true)];
/// Longest v3 frame worth reading here, in bytes.
const V3_MAX_BYTES: usize = 12;
/// Nibble the THGR810's checksum starts at.
const CHECKSUM_NIBBLE: usize = 15;
/// Payload bytes a temperature and humidity frame carries, used by the tests
/// that build one.
#[cfg(test)]
const MSG_BYTES: usize = 9;

/// One v3 layout: which model, where its checksum starts, and what it reports.
struct V3Model {
    model: &'static str,
    checksum_nibble: usize,
    kind: Kind,
}

enum Kind {
    TempHumidity,
    Temperature,
    Wind,
}

/// Known sensor ids. The THGR810 rolled its id several times across rebrands
/// (Newentor, Unni, Liorque), all differing only in the second nibble.
fn v3_model_of(id: u16) -> Option<V3Model> {
    let m = |model, checksum_nibble, kind| Some(V3Model { model, checksum_nibble, kind });
    match id {
        0xf824 | 0xf024 | 0xf224 | 0xfa24 | 0xf8b4 => {
            m("Oregon-THGR810", CHECKSUM_NIBBLE, Kind::TempHumidity)
        }
        0xc844 => m("Oregon-THN802", 12, Kind::Temperature),
        0x1984 | 0x1994 => m("Oregon-WGR800", 17, Kind::Wind),
        _ => None,
    }
}

impl Protocol for OregonV3 {
    fn name(&self) -> &'static str {
        "Oregon-v3"
    }

    fn timing(&self) -> Timing {
        oregon_timing()
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let mut best = Err(DecodeError::NotThisProtocol);
        for (sync, invert) in V3_SYNC {
            for at in sync_offsets(bits, sync) {
                match v3_frame(bits, at + 16, invert) {
                    Ok(r) => return Ok(r),
                    Err(e) => best = keep_worse(best, e),
                }
            }
        }
        best
    }
}

fn v3_frame(bits: &BitBuffer, start: usize, invert: bool) -> Result<Report, DecodeError> {
    let take = (bits.len() - start).min(V3_MAX_BYTES * 8);
    let frame = bits.slice(start, take);
    let frame = if invert { frame.inverted() } else { frame };
    let msg: Vec<u8> = frame.as_padded_bytes().iter().map(|b| reflect_nibbles(*b)).collect();
    if msg.len() < 7 {
        return Err(DecodeError::WrongLength { got: take, want: 7 * 8 });
    }

    let m = v3_model_of(((msg[0] as u16) << 8) | msg[1] as u16)
        .ok_or(DecodeError::NotThisProtocol)?;
    if msg.len() < m.checksum_nibble / 2 + 2 {
        return Err(DecodeError::WrongLength { got: take, want: m.checksum_nibble * 4 + 8 });
    }
    let (sum, stored) = checksum(&msg, m.checksum_nibble);
    if sum != stored {
        return Err(DecodeError::CrcFailed);
    }

    let mut r = common_fields(m.model, &msg);
    match m.kind {
        Kind::TempHumidity => {
            r = r
                .float("temperature_c", round1(temperature_c(&msg)?))
                .int("humidity_pct", humidity_pct(&msg)? as i64);
        }
        Kind::Temperature => r = r.float("temperature_c", round1(temperature_c(&msg)?)),
        Kind::Wind => {
            let (gust, avg, direction) = wind(&msg)?;
            r = r
                .float("wind_gust_ms", gust)
                .float("wind_avg_ms", avg)
                .float("wind_direction_deg", direction);
        }
    }
    Ok(r)
}

// ---------------------------------------------------------------------------
// v2.1
// ---------------------------------------------------------------------------

/// Oregon Scientific v2.1: the THGR122N, THN132N and RTGN318 families.
///
/// Same payload layout as v3 once the bits are recovered, and a different way
/// of carrying them. v2.1 sends every bit twice, inverted the second time, on
/// top of the Manchester coding the slicer already undid, so the sliced stream
/// holds complementary pairs and the payload is one bit per pair. A pair whose
/// halves agree is where the transmission ended, which is also how the frame
/// length is measured, and the length is what tells a THN132N from a THR228N:
/// they share a sensor id and differ only in how much they send.
///
/// The preamble reads as a run of 0x55 or 0xaa depending on where the slicer
/// locked, and the sync byte that follows reads as 0x99 either way, so both
/// 16 bit patterns rtl_433 searches for are searched for here too, in the
/// buffer and in its inverse.
pub struct OregonV2;

/// Preamble tail plus sync, in each polarity the slicer can produce.
const V2_SYNC: [u32; 2] = [0x5599, 0xaa99];
/// rtl_433 stops unpacking at 173 bits, the longest v2.1 message there is.
const V2_MAX_BITS: usize = 176;

/// One v2.1 layout: sensor id, how many payload bits it sends, where its
/// checksum starts and whether it has a humidity element.
struct V2Model {
    model: &'static str,
    bits: usize,
    checksum_nibble: usize,
    humidity: bool,
}

/// The layouts sharing this decoder, keyed the way rtl_433 keys them: some ids
/// are exact, and the rebranded RTGN and RTHN sensors vary in their top
/// nibble, so those match on the low twelve bits.
fn v2_model_of(id: u16, bits: usize) -> Option<V2Model> {
    let m = |model, bits, checksum_nibble, humidity| {
        Some(V2Model { model, bits, checksum_nibble, humidity })
    };
    match (id, id & 0x0fff, bits) {
        (0x1d20, _, _) => m("Oregon-THGR122N", 76, 15, true),
        (0x1d30, _, _) => m("Oregon-THGR968", 76, 15, true),
        // The THR228N shares this id with the THN132N and differs only in
        // message length, which is not measurable here: a burst runs one copy
        // straight into the preamble of the next, and that preamble unpacks as
        // valid pairs, so the frame never ends where the transmitter stopped.
        // Both sensors report the same fields, so the more common one is named.
        (0xec40, _, _) => m("Oregon-THN132N", 64, 12, false),
        (0xec41, _, _) => m("Oregon-AWR129", 76, 12, false),
        (_, 0x0cc3, 80) => m("Oregon-RTGN129", 80, 15, true),
        (_, 0x0cc3, _) => m("Oregon-RTGN318", 76, 15, true),
        (0xcc43, _, _) => m("Oregon-THN129", 68, 12, false),
        (_, 0x0cd3, _) => m("Oregon-RTHN129", 68, 12, false),
        _ => None,
    }
}

impl Protocol for OregonV2 {
    fn name(&self) -> &'static str {
        "Oregon-v2.1"
    }

    fn timing(&self) -> Timing {
        oregon_timing()
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let inverted = bits.inverted();
        let mut best = Err(DecodeError::NotThisProtocol);
        for buf in [bits, &inverted] {
            for sync in V2_SYNC {
                for at in sync_offsets(buf, sync) {
                    match v2_frame(buf, at + 16) {
                        Ok(r) => return Ok(r),
                        Err(e) => best = keep_worse(best, e),
                    }
                }
            }
        }
        best
    }
}

/// Unpack, reflect and parse one candidate frame starting at `start`.
fn v2_frame(bits: &BitBuffer, start: usize) -> Result<Report, DecodeError> {
    let payload = unpack_doubled(bits, start);
    let msg: Vec<u8> = payload.as_padded_bytes().iter().map(|b| reflect_nibbles(*b)).collect();
    if msg.len() < 7 {
        return Err(DecodeError::WrongLength { got: payload.len(), want: 64 });
    }
    let id = ((msg[0] as u16) << 8) | msg[1] as u16;
    let m = v2_model_of(id, payload.len()).ok_or(DecodeError::NotThisProtocol)?;
    if payload.len() < m.bits || msg.len() < m.checksum_nibble / 2 + 2 {
        return Err(DecodeError::WrongLength { got: payload.len(), want: m.bits });
    }
    let (sum, stored) = checksum(&msg, m.checksum_nibble);
    if sum != stored {
        return Err(DecodeError::CrcFailed);
    }

    let temperature = temperature_c(&msg)?;
    let channel = msg[2] >> 4;
    // Valid channels are 1, 2 and 4 on the three-channel sensors and 1 to 5 on
    // the RTGN ones. Zero is a frame that found its checksum by luck.
    if channel == 0 || channel > 5 {
        return Err(DecodeError::Implausible("channel out of range"));
    }

    let mut r = common_fields(m.model, &msg).float("temperature_c", round1(temperature));
    if m.humidity {
        r = r.int("humidity_pct", humidity_pct(&msg)? as i64);
    }
    Ok(r)
}

/// Undo v2.1's doubling: each payload bit arrives as a complementary pair, and
/// a pair whose halves agree ends the message.
fn unpack_doubled(bits: &BitBuffer, start: usize) -> BitBuffer {
    let mut out = BitBuffer::new();
    let mut i = start;
    while i + 1 < bits.len() && out.len() < V2_MAX_BITS {
        let (a, b) = (bits.get(i).unwrap(), bits.get(i + 1).unwrap());
        if a == b {
            break;
        }
        out.push(b);
        i += 2;
    }
    out
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

/// Keep the most informative failure across the candidates tried.
///
/// A burst offers many sync matches and most of them are noise, so
/// "not this protocol" from the last one would hide the checksum failure that
/// says the signal really is an Oregon sensor received badly.
fn keep_worse(
    best: Result<Report, DecodeError>,
    e: DecodeError,
) -> Result<Report, DecodeError> {
    let rank = |e: &DecodeError| match e {
        DecodeError::NotThisProtocol => 0,
        DecodeError::WrongLength { .. } => 1,
        DecodeError::Implausible(_) => 2,
        DecodeError::CrcFailed => 3,
    };
    match &best {
        Err(prev) if rank(prev) < rank(&e) => Err(e),
        _ => best,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// Build a v3 frame the way the sensor sends it: preamble, sync, then the
    /// payload with every nibble reflected.
    fn frame(id: u16, channel: u8, device: u8, temp_c: f64, humidity: u8, low: bool) -> BitBuffer {
        let nibble = v3_model_of(id).map_or(CHECKSUM_NIBBLE, |m| m.checksum_nibble);
        let msg = payload(id, channel, device, temp_c, humidity, low, nibble);
        let mut out = BitBuffer::new();
        for _ in 0..24 {
            out.push(false);
        }
        // The sync nibble, completing the 0x0005 pattern the decoder looks
        // for: the preamble above supplies the leading zeros.
        for bit in [false, true, false, true] {
            out.push(bit);
        }
        for b in msg {
            let wire = reflect_nibbles(b);
            for i in 0..8 {
                out.push(wire & (0x80 >> i) != 0);
            }
        }
        out
    }

    /// The same payload as a v2.1 transmission: preamble of 0x55, the 0x99
    /// sync, then every bit sent twice with the second copy inverted.
    fn frame_v2(
        id: u16,
        channel: u8,
        device: u8,
        temp_c: f64,
        humidity: u8,
        low: bool,
        layout: (usize, usize),
    ) -> BitBuffer {
        let (bits, checksum_nibble) = layout;
        let msg = payload(id, channel, device, temp_c, humidity, low, checksum_nibble);
        let mut out = BitBuffer::new();
        for _ in 0..16 {
            for bit in [false, true] {
                out.push(bit);
            }
        }
        for bit in [true, false, false, true, true, false, false, true] {
            out.push(bit);
        }
        for i in 0..bits {
            let bit = reflect_nibbles(msg[i / 8]) & (0x80 >> (i % 8)) != 0;
            out.push(!bit);
            out.push(bit);
        }
        // Silence after the frame reads as a pair whose halves agree, which is
        // what ends the message.
        out.push(false);
        out.push(false);
        out
    }

    fn payload(
        id: u16,
        channel: u8,
        device: u8,
        temp_c: f64,
        humidity: u8,
        low: bool,
        checksum_nibble: usize,
    ) -> [u8; MSG_BYTES + 2] {
        let mut msg = [0u8; MSG_BYTES + 2];
        msg[0] = (id >> 8) as u8;
        msg[1] = id as u8;
        msg[2] = (channel << 4) | (device & 0x0f);
        msg[3] = (device & 0xf0) | if low { 0x04 } else { 0 };

        let t = (temp_c.abs() * 10.0).round() as u32;
        msg[4] = (((t % 10) as u8) << 4) | ((t / 10 % 10) as u8);
        msg[5] = (((t / 100 % 10) as u8) << 4)
            | if temp_c < 0.0 { 0x08 } else { 0 }
            | (t / 1000) as u8;
        msg[6] = ((humidity % 10) << 4) | (humidity / 10);

        let sum = checksum(&msg, checksum_nibble).0;
        let at = checksum_nibble / 2;
        if checksum_nibble % 2 == 1 {
            msg[at] = (msg[at] & 0xf0) | (sum & 0x0f);
            msg[at + 1] = sum & 0xf0;
        } else {
            msg[at] = sum.rotate_left(4);
        }
        msg
    }

    #[test]
    fn decodes_a_thgr810_frame() {
        let r = OregonV3.decode(&frame(0xf824, 1, 0x3a, 21.7, 48, false)).unwrap();
        assert_eq!(r.model, "Oregon-THGR810");
        assert_eq!(r.get("channel"), Some(&Value::Int(1)));
        assert_eq!(r.get("id"), Some(&Value::Int(0x3a)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.7)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(48)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn the_other_manchester_polarity_decodes_the_same() {
        // Which half of the symbol carries the bit depends on where the
        // slicer started, and both happen in practice.
        let f = frame(0xf824, 1, 0x3a, 21.7, 48, false);
        let a = OregonV3.decode(&f).unwrap();
        let b = OregonV3.decode(&f.inverted()).unwrap();
        assert_eq!(a.fields, b.fields);
    }

    #[test]
    fn a_frost_reading_carries_its_sign_bit() {
        let r = OregonV3.decode(&frame(0xf824, 2, 0x3a, -6.3, 91, true)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-6.3)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_temperature_only_sensor_reports_no_humidity() {
        let r = OregonV3.decode(&frame(0xc844, 1, 0x11, 19.0, 0, false)).unwrap();
        assert_eq!(r.model, "Oregon-THN802");
        assert!(r.get("humidity_pct").is_none());
    }

    #[test]
    fn an_unknown_sensor_id_is_not_claimed() {
        // The checksum is eight bits over fifteen nibbles, so the id table is
        // doing most of the work of not claiming other people's frames.
        assert_eq!(
            OregonV3.decode(&frame(0x1234, 1, 0x3a, 21.7, 48, false)),
            Err(DecodeError::NotThisProtocol)
        );
    }

    #[test]
    fn a_corrupt_frame_fails_its_checksum() {
        let f = frame(0xf824, 1, 0x3a, 21.7, 48, false);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 60 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert_eq!(OregonV3.decode(&broken), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn decodes_a_thgr122n_frame() {
        let f = frame_v2(0x1d20, 1, 0x3a, 21.7, 48, false, (76, 15));
        let r = OregonV2.decode(&f).unwrap();
        assert_eq!(r.model, "Oregon-THGR122N");
        assert_eq!(r.get("id"), Some(&Value::Int(0x3a)));
        assert_eq!(r.get("channel"), Some(&Value::Int(1)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.7)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(48)));
    }

    #[test]
    fn a_temperature_only_v2_sensor_reports_no_humidity() {
        let f = frame_v2(0xec40, 1, 0x7d, 20.2, 0, false, (64, 12));
        let r = OregonV2.decode(&f).unwrap();
        assert_eq!(r.model, "Oregon-THN132N");
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(20.2)));
        assert!(r.get("humidity_pct").is_none());
    }

    #[test]
    fn a_v2_frame_decodes_in_either_polarity() {
        let f = frame_v2(0x1d20, 2, 0x3a, -5.5, 30, false, (76, 15));
        let a = OregonV2.decode(&f).unwrap();
        let b = OregonV2.decode(&f.inverted()).unwrap();
        assert_eq!(a.fields, b.fields);
        assert_eq!(a.get("temperature_c"), Some(&Value::Float(-5.5)));
    }

    #[test]
    fn a_corrupt_v2_frame_fails_its_checksum() {
        // Both halves of a pair, so the frame stays well formed and only the
        // value it carries is wrong.
        let f = frame_v2(0x1d20, 1, 0x3a, 21.7, 48, false, (76, 15));
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 90 || i == 91 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert!(matches!(
            OregonV2.decode(&broken),
            Err(DecodeError::CrcFailed | DecodeError::Implausible(_))
        ));
    }
}
