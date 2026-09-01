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

use crate::bits::{crc8, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::find_frame;
use crate::slicer::{differential_manchester_decode, Coding, Timing};

/// Schrader Electronics MRXGG4, the sensor fitted to a large share of European
/// and American cars.
///
/// 433.92 MHz, OOK Manchester at 120 us a half symbol. A sync nibble then
/// eight bytes:
///
/// ```text
/// PF FI II II II SS TT CC
/// ```
///
/// - `P` a constant 0xf preamble nibble, `F` twelve bits of flags
/// - `I` 28 bit sensor id, which is what identifies the wheel and the car
/// - `SS` pressure, 25 mbar a count
/// - `TT` temperature in Celsius, offset by 50 so it can go below freezing
/// - `CC` CRC8, polynomial 0x07 with an initial value of 0xf0, over the seven
///   bytes before it
pub struct SchraderTpms;

const FRAME_BYTES: usize = 8;

impl Protocol for SchraderTpms {
    fn name(&self) -> &'static str {
        "Schrader"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Manchester,
            short_us: 120,
            long_us: 240,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 480,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let b = find_frame(bits, FRAME_BYTES, |b| {
            // The preamble nibble is checked as well as the CRC. Eight bits of
            // CRC over a burst offering hundreds of windows is not on its own
            // enough to keep a phantom wheel out of the log.
            b[0] >> 4 == 0x0f && b[7] == crc8(&b[..7], 0x07, 0xf0)
        })
        .ok_or(match bits.len() {
            n if n < FRAME_BYTES * 8 => {
                DecodeError::WrongLength { got: n, want: FRAME_BYTES * 8 }
            }
            _ => DecodeError::CrcFailed,
        })?;

        let id = ((b[1] as u32 & 0x0f) << 24) | (b[2] as u32) << 16 | (b[3] as u32) << 8 | b[4] as u32;
        let flags = ((b[0] & 0x0f) << 4) | (b[1] >> 4);
        let temperature = b[6] as i32 - 50;
        if !(-50..=100).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        Ok(r
            .text("id", format!("{id:07X}"))
            .text("flags", format!("{flags:02x}"))
            // 25 mbar a count, which is 2.5 kPa.
            .float("pressure_kpa", b[5] as f64 * 2.5)
            .float("temperature_c", temperature as f64))
    }
}

/// Pacific Industrial PMV-C210, the sensor Toyota fits and TRW builds under
/// licence for several other makes.
///
/// 433.92 MHz, FSK at 52 us a half symbol, differential Manchester rather than
/// plain: the bit is whether the symbol begins with a transition, and the
/// transition between symbols is the clock. Fourteen bits of sync, then nine
/// bytes:
///
/// ```text
/// II II II II SP PT TS QQ CC
/// ```
///
/// - `I` 32 bit sensor id
/// - `S` a status bit and seven more status bits three bytes later
/// - `P` pressure, quarter PSI a count, offset by 7 PSI
/// - `T` temperature in Celsius, offset by 40
/// - `QQ` the pressure again, inverted, which is a second check on the
///   quantity most likely to be read wrong
/// - `CC` CRC8, polynomial 0x07 with an initial value of 0x80
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
    Ok(r
        .text("id", format!("{id:08x}"))
        .int("status", ((b[4] & 0x80) | (b[6] & 0x7f)) as i64)
        .float("pressure_psi", pressure as f64 * 0.25 - 7.0)
        .float("temperature_c", temperature as f64))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// The frame from rtl_433's own description of the protocol, which is a
    /// real capture: id 03A38B2, no pressure, 23 C.
    const FRAME: [u8; 8] = [0xf6, 0x70, 0x3a, 0x38, 0xb2, 0x00, 0x49, 0x49];

    fn burst(frame: &[u8; 8], repeats: usize) -> BitBuffer {
        let mut b = BitBuffer::new();
        for _ in 0..repeats {
            b.mark_row();
            for byte in frame {
                for i in 0..8 {
                    b.push(byte & (0x80 >> i) != 0);
                }
            }
        }
        b
    }

    #[test]
    fn decodes_the_documented_frame() {
        let r = SchraderTpms.decode(&burst(&FRAME, 3)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Text("03A38B2".into())));
        assert_eq!(r.get("flags"), Some(&Value::Text("67".into())));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(23.0)));
        assert_eq!(r.get("pressure_kpa"), Some(&Value::Float(0.0)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_corrupt_frame_fails_its_crc() {
        let mut frame = FRAME;
        frame[3] ^= 0x10;
        assert_eq!(
            SchraderTpms.decode(&burst(&frame, 3)),
            Err(DecodeError::CrcFailed)
        );
    }

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

    #[test]
    fn a_frame_without_the_preamble_nibble_is_not_claimed() {
        // The CRC still passes here, so only the preamble check refuses it.
        let mut frame = FRAME;
        frame[0] = 0x26;
        frame[7] = crc8(&frame[..7], 0x07, 0xf0);
        assert_eq!(
            SchraderTpms.decode(&burst(&frame, 3)),
            Err(DecodeError::CrcFailed)
        );
    }
}
