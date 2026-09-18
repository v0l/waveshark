//! Tyre pressure sensors.
//!
//! A TPMS sensor sits inside a wheel and reports its own identity, the
//! pressure and the temperature every minute or two while the car is moving.
//! That makes them the most useful thing on 315 and 433 MHz for identifying a
//! vehicle: the id is fixed for the life of the sensor, four of them travel
//! together, and nothing about the transmission is authenticated.
//!
//! They are also the most demanding thing on the band for a receiver. A frame
//! is a few milliseconds long, it arrives once per wheel per minute, and it is
//! gone. Nothing here can be asked to repeat itself.

use crate::bits::{BitBuffer, crc8};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing, differential_manchester_decode, manchester_decode};

pub struct ToyotaTpms;

/// The tail of the sync, as rtl_433 searches for it: twelve bits, with the
/// last of them handed to the differential decoder so it can find its phase.
const TOYOTA_SYNC: u32 = 0xa9e;
/// Payload bits, before the three bit trailer.
const TOYOTA_BITS: usize = 72;

impl Protocol for ToyotaTpms {
    fn name(&self) -> &'static str {
        "Toyota"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: 52,
            long_us: 52,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 150,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // Which tone the discriminator calls high depends on which side of the
        // carrier the channel sat on, so both polarities are tried.
        let inverted = bits.inverted();
        let mut best = Err(DecodeError::NotThisProtocol);
        for buf in [bits, &inverted] {
            for at in 0..buf.len().saturating_sub(12) {
                if buf.extract(at, 12) != Some(TOYOTA_SYNC) {
                    continue;
                }
                match toyota_frame(buf, at + 11) {
                    Ok(r) => return Ok(r),
                    Err(e) => best = Err(e),
                }
            }
        }
        best
    }
}

fn toyota_frame(bits: &BitBuffer, start: usize) -> Result<Report, DecodeError> {
    let payload = differential_manchester_decode(bits, start, 80);
    if payload.len() < TOYOTA_BITS {
        return Err(DecodeError::WrongLength { got: payload.len(), want: TOYOTA_BITS });
    }
    let b = payload.as_padded_bytes();
    if b[8] != crc8(&b[..8], 0x07, 0x80) {
        return Err(DecodeError::CrcFailed);
    }

    let pressure = ((b[4] & 0x7f) as u16) << 1 | (b[5] >> 7) as u16;
    // The frame carries the pressure twice, the second time inverted.
    if pressure != (b[7] ^ 0xff) as u16 {
        return Err(DecodeError::Implausible("the two pressure fields disagree"));
    }
    let temperature = (((b[5] & 0x7f) as i16) << 1 | (b[6] >> 7) as i16) - 40;

    let id = (b[0] as u32) << 24 | (b[1] as u32) << 16 | (b[2] as u32) << 8 | b[3] as u32;
    let mut r = Report::new("Toyota");
    r.crc_valid = Some(true);
    r.raw = b[..9].to_vec();
    Ok(r.text("id", format!("{id:08x}"))
        .int("status", ((b[4] & 0x80) | (b[6] & 0x7f)) as i64)
        .float("pressure_psi", pressure as f64 * 0.25 - 7.0)
        .float("temperature_c", temperature as f64))
}

/// The preamble Ford, Renault and Citroen sensors all send, as the half
/// symbols reach the decoder: `55 55 55 56` with the tone assignment the
/// discriminator happened to give them.
const VDO_PREAMBLE: u32 = 0xaaa9;
const VDO_PREAMBLE_BITS: usize = 16;

/// Every Manchester payload a `55 55 55 56` preamble introduces, in both
/// polarities.
///
/// Which tone the discriminator calls high depends on which side of the
/// carrier the channel sat on, and nothing in the frame says which way round
/// it was: rtl_433 inverts the buffer before searching, which is the same
/// thing said once rather than twice.
fn vdo_frames(bits: &BitBuffer, want_bits: usize) -> impl Iterator<Item = Vec<u8>> {
    let inverted = bits.inverted();
    let mut out = Vec::new();
    for buf in [bits.clone(), inverted] {
        for at in 0..buf.len().saturating_sub(VDO_PREAMBLE_BITS) {
            if buf.extract(at, VDO_PREAMBLE_BITS) != Some(VDO_PREAMBLE) {
                continue;
            }
            let payload = manchester_decode(&buf, at + VDO_PREAMBLE_BITS);
            if payload.len() >= want_bits {
                out.push(payload.as_padded_bytes().to_vec());
            }
        }
    }
    out.into_iter()
}

/// The sensor Ford fits to the Fiesta, Focus, Kuga, Escape and Transit, built
/// by Continental and sold as a VDO part.
///
/// 433.92 MHz here and 315 MHz in the United States, FSK at 52 us a symbol,
/// Manchester over a `55 55 55 56` preamble. Eight bytes:
///
/// ```text
/// II II II II PP TT FF CC
/// ```
///
/// - `I` 32 bit sensor id
/// - `PP` pressure, quarter PSI a count, with a ninth bit in the flags
/// - `TT` temperature in Celsius offset by 56, valid only while the top bit
///   is clear: with it set the byte carries something else that is not a
///   measurement
/// - `FF` flags: moving, at rest or learning
/// - `CC` the sum of the seven bytes before it
pub struct FordTpms;

const FORD_BYTES: usize = 8;

impl Protocol for FordTpms {
    fn name(&self) -> &'static str {
        "Ford"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: 52,
            long_us: 52,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 150,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let mut best = Err(DecodeError::NotThisProtocol);
        for b in vdo_frames(bits, FORD_BYTES * 8) {
            match ford_frame(&b) {
                Ok(r) => return Ok(r),
                Err(e) => best = Err(e),
            }
        }
        best
    }
}

fn ford_frame(b: &[u8]) -> Result<Report, DecodeError> {
    let sum = b[..7].iter().fold(0u8, |a, v| a.wrapping_add(*v));
    if sum != b[7] {
        return Err(DecodeError::CrcFailed);
    }

    // Three bits say what the sensor is doing, and only three combinations of
    // them mean anything. A sum over seven bytes is a weak check on its own,
    // so a frame claiming a state no sensor sends is refused rather than
    // reported with the flags for the reader to puzzle over.
    let (moving, learn) = match b[6] & 0x4c {
        0x08 => (false, true),
        0x04 => (false, false),
        0x44 => (true, false),
        _ => {
            return Err(DecodeError::Implausible("flags say neither moving, at rest nor learning"));
        }
    };
    if b[6] & 0x90 != 0 {
        return Err(DecodeError::Implausible("a flag bit no sensor sets"));
    }

    let id = (b[0] as u32) << 24 | (b[1] as u32) << 16 | (b[2] as u32) << 8 | b[3] as u32;
    let code = (b[4] as u32) << 16 | (b[5] as u32) << 8 | b[6] as u32;
    // The ninth bit of the pressure lives in the flags, which a Transit at
    // lorry pressures needs and nothing else sets.
    let pressure = (((b[6] & 0x20) as u16) << 3 | b[4] as u16) as f64 * 0.25;

    let mut r = Report::new("Ford");
    r.crc_valid = Some(true);
    r.raw = b[..FORD_BYTES].to_vec();
    r = r
        .text("id", format!("{id:08x}"))
        .float("pressure_psi", pressure)
        .bool("moving", moving)
        .bool("learn", learn)
        .text("code", format!("{code:06x}"));
    // The top bit of the temperature byte marks the byte as something else,
    // so there is no reading to report rather than a reading to distrust.
    if b[5] & 0x80 == 0 {
        r = r.float("temperature_c", (b[5] & 0x7f) as f64 - 56.0);
    }
    Ok(r)
}

/// The sensor on the Renault Clio, Captur and Zoe, and on the Dacia Sandero.
///
/// The same waveform as the Ford sensor, nine bytes rather than eight and a
/// CRC rather than a sum:
///
/// ```text
/// FP PT TI II II ?? ?? CC
/// ```
///
/// - `F` six bits of flags, then ten bits of pressure at 0.75 kPa a count
/// - `T` temperature in Celsius, offset by 30
/// - `I` 24 bit sensor id, least significant byte first
/// - `?` two bytes nobody has explained, usually 0xffff
/// - `CC` CRC8, polynomial 0x07 from zero, over the eight bytes before it
pub struct RenaultTpms;

const RENAULT_BYTES: usize = 9;

impl Protocol for RenaultTpms {
    fn name(&self) -> &'static str {
        "Renault"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: 52,
            long_us: 52,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 150,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let mut best = Err(DecodeError::NotThisProtocol);
        for b in vdo_frames(bits, RENAULT_BYTES * 8) {
            match renault_frame(&b) {
                Ok(r) => return Ok(r),
                Err(e) => best = Err(e),
            }
        }
        best
    }
}

fn renault_frame(b: &[u8]) -> Result<Report, DecodeError> {
    if b[8] != crc8(&b[..8], 0x07, 0x00) {
        return Err(DecodeError::CrcFailed);
    }
    let temperature = b[2] as i32 - 30;
    if !(-40..=100).contains(&temperature) {
        return Err(DecodeError::Implausible("temperature out of range"));
    }
    let id = (b[5] as u32) << 16 | (b[4] as u32) << 8 | b[3] as u32;

    let mut r = Report::new("Renault");
    r.crc_valid = Some(true);
    r.raw = b[..RENAULT_BYTES].to_vec();
    Ok(r.text("id", format!("{id:06x}"))
        .text("flags", format!("{:02x}", b[0] >> 2))
        .float("pressure_kpa", (((b[0] & 0x03) as u16) << 8 | b[1] as u16) as f64 * 0.75)
        .float("temperature_c", temperature as f64))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A Toyota frame as it goes out: sync, then each bit as two half symbols,
    /// the pair equal for a 1 and unequal for a 0, with the level inverted at
    /// every symbol boundary because that boundary transition is the clock.
    fn toyota_burst(frame: &[u8; 9]) -> BitBuffer {
        let mut b = BitBuffer::new();
        // The alternating sync, ending in the twelve bits the decoder searches
        // for. The last of them is a sync bit rather than data: the
        // differential decoder needs the level before the frame to know which
        // way the first symbol boundary went.
        for bit in [0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0] {
            b.push(bit == 1);
        }
        let mut level = false;
        for byte in frame {
            for i in 0..8 {
                let one = byte & (0x80 >> i) != 0;
                level = !level;
                b.push(level);
                if !one {
                    level = !level;
                }
                b.push(level);
            }
        }
        b
    }

    /// rtl_433's own recording `Toyota_TPMS/gfile006`, as this decoder reads
    /// it: id fb0a43e7, 36.75 PSI, 29 C.
    fn toyota_frame() -> [u8; 9] {
        let mut f = [0xfb, 0x0a, 0x43, 0xe7, 0xae, 0x8a, 0x80, 0x00, 0x00];
        let pressure = 175u16; // (36.75 + 7) * 4
        let temperature = 69u16; // 29 + 40
        f[4] = 0x80 | (pressure >> 1) as u8;
        f[5] = ((pressure as u8 & 1) << 7) | (temperature >> 1) as u8;
        f[6] = (temperature as u8 & 1) << 7;
        f[7] = !(pressure as u8);
        f[8] = crc8(&f[..8], 0x07, 0x80);
        f
    }

    #[test]
    fn decodes_a_toyota_frame() {
        let r = ToyotaTpms.decode(&toyota_burst(&toyota_frame())).unwrap();
        assert_eq!(r.model, "Toyota");
        assert_eq!(r.get("id"), Some(&Value::Text("fb0a43e7".into())));
        assert_eq!(r.get("pressure_psi"), Some(&Value::Float(36.75)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(29.0)));
        assert_eq!(r.get("status"), Some(&Value::Int(128)));
    }

    #[test]
    fn a_toyota_frame_decodes_in_either_polarity() {
        let f = toyota_burst(&toyota_frame());
        let a = ToyotaTpms.decode(&f).unwrap();
        let b = ToyotaTpms.decode(&f.inverted()).unwrap();
        assert_eq!(a.fields, b.fields);
    }

    #[test]
    fn a_toyota_frame_whose_pressure_fields_disagree_is_refused() {
        let mut f = toyota_frame();
        f[7] ^= 0x01;
        f[8] = crc8(&f[..8], 0x07, 0x80);
        assert_eq!(
            ToyotaTpms.decode(&toyota_burst(&f)),
            Err(DecodeError::Implausible("the two pressure fields disagree"))
        );
    }

    #[test]
    fn a_corrupt_toyota_frame_fails_its_crc() {
        let mut f = toyota_frame();
        f[2] ^= 0x08;
        assert_eq!(ToyotaTpms.decode(&toyota_burst(&f)), Err(DecodeError::CrcFailed));
    }

    /// A Ford or Renault frame as it goes out: the `55 55 55 56` preamble,
    /// then each bit as a pair of half symbols, the bit itself second.
    fn vdo_burst(frame: &[u8]) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..32 {
            b.push(0x5555_5556u32 & (0x8000_0000 >> i) != 0);
        }
        for byte in frame {
            for i in 0..8 {
                let one = byte & (0x80 >> i) != 0;
                b.push(one);
                b.push(!one);
            }
        }
        b
    }

    /// rtl_433's recording `Ford_TPMS/gfile059`: id 45bb320f, 26.5 PSI, a
    /// moving wheel, and a temperature byte carrying something else.
    fn ford_frame_bytes() -> [u8; 8] {
        let mut f = [0x45, 0xbb, 0x32, 0x0f, 0x6a, 0xd4, 0x46, 0x00];
        f[7] = f[..7].iter().fold(0u8, |a, v| a.wrapping_add(*v));
        f
    }

    #[test]
    fn decodes_a_ford_frame() {
        let r = FordTpms.decode(&vdo_burst(&ford_frame_bytes())).unwrap();
        assert_eq!(r.model, "Ford");
        assert_eq!(r.get("id"), Some(&Value::Text("45bb320f".into())));
        assert_eq!(r.get("code"), Some(&Value::Text("6ad446".into())));
        assert_eq!(r.get("pressure_psi"), Some(&Value::Float(26.5)));
        assert_eq!(r.get("moving"), Some(&Value::Bool(true)));
        assert_eq!(r.get("learn"), Some(&Value::Bool(false)));
        // The top bit of the temperature byte is set, so there is no reading.
        assert_eq!(r.get("temperature_c"), None);
    }

    #[test]
    fn a_ford_frame_reads_its_temperature_when_the_byte_holds_one() {
        let mut f = ford_frame_bytes();
        f[5] = 56 + 21;
        f[7] = f[..7].iter().fold(0u8, |a, v| a.wrapping_add(*v));
        let r = FordTpms.decode(&vdo_burst(&f)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.0)));
    }

    #[test]
    fn a_ford_frame_with_flags_no_sensor_sends_is_refused() {
        let mut f = ford_frame_bytes();
        f[6] = 0x40;
        f[7] = f[..7].iter().fold(0u8, |a, v| a.wrapping_add(*v));
        assert_eq!(
            FordTpms.decode(&vdo_burst(&f)),
            Err(DecodeError::Implausible("flags say neither moving, at rest nor learning"))
        );
    }

    #[test]
    fn a_corrupt_ford_frame_fails_its_sum() {
        let mut f = ford_frame_bytes();
        f[1] ^= 0x04;
        assert_eq!(FordTpms.decode(&vdo_burst(&f)), Err(DecodeError::CrcFailed));
    }

    /// rtl_433's recording `Renault_TPMS/gfile070`: id 87f293, flags 34,
    /// 202.5 kPa at 25 C.
    fn renault_frame_bytes() -> [u8; 9] {
        // 270 counts of 0.75 kPa, the top two bits of it sharing a byte with
        // the flags.
        let mut f = [0xd1, 0x0e, 0x37, 0x93, 0xf2, 0x87, 0xff, 0xff, 0x00];
        f[8] = crc8(&f[..8], 0x07, 0x00);
        f
    }

    #[test]
    fn decodes_a_renault_frame() {
        let r = RenaultTpms.decode(&vdo_burst(&renault_frame_bytes())).unwrap();
        assert_eq!(r.model, "Renault");
        assert_eq!(r.get("id"), Some(&Value::Text("87f293".into())));
        assert_eq!(r.get("flags"), Some(&Value::Text("34".into())));
        assert_eq!(r.get("pressure_kpa"), Some(&Value::Float(202.5)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(25.0)));
    }

    #[test]
    fn a_corrupt_renault_frame_fails_its_crc() {
        let mut f = renault_frame_bytes();
        f[4] ^= 0x20;
        assert_eq!(RenaultTpms.decode(&vdo_burst(&f)), Err(DecodeError::CrcFailed));
    }
}
