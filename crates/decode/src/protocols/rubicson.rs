//! Rubicson, TFA 30.3197 and inFactory PT-310 temperature sensors.
//!
//! 433.92 MHz, 36 bits of PPM, twelve repeats per transmission. The layout is
//! all but identical to Nexus, which is why [`super::NexusTh`] refuses any
//! frame that satisfies the CRC below: without this decoder those frames were
//! being thrown away, and with a Nexus decoder but no Rubicson one they would
//! be reported as the wrong device with the wrong temperature.
//!
//! ```text
//! [id0] [id1] [B0CC] [temp0] [temp1] [temp2] [f] [crc0] [crc1]
//! ```
//!
//! - `id`   8 bits, redrawn when the batteries are changed
//! - `B`    battery ok
//! - `CC`   channel, 0 to 2
//! - `temp` 12 bit signed, 0.1 C steps
//! - `f`    always 1111
//! - `crc`  CRC-8, polynomial 0x31, init 0x6c, over the five bytes formed by
//!   the first seven nibbles right-padded with a zero nibble, then the CRC
//!   itself. Not a plain CRC over the frame: the padding matters

use crate::bits::{crc8, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::find_frame_bits;
use crate::slicer::Timing;

pub struct Rubicson;

const FRAME_BITS: usize = 36;

impl Protocol for Rubicson {
    fn name(&self) -> &'static str {
        "Rubicson-Temperature"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(1000, 2000, 4800)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let b = find_frame_bits(bits, FRAME_BITS, crc_ok).ok_or(match bits.len() {
            n if n < FRAME_BITS => DecodeError::WrongLength { got: n, want: FRAME_BITS },
            _ => DecodeError::CrcFailed,
        })?;

        let raw = (((b[1] as u16 & 0x0f) << 8) | b[2] as u16) as i16;
        let temperature = ((raw << 4) >> 4) as f64 * 0.1;
        if !(-40.0..=70.0).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        Ok(r.int("id", b[0] as i64)
            .int("channel", ((b[1] >> 4) & 0x03) as i64 + 1)
            .float("temperature_c", (temperature * 10.0).round() / 10.0)
            .bool("battery_ok", b[1] & 0x80 != 0))
    }
}

fn crc_ok(b: &[u8]) -> bool {
    if b[3] & 0xf0 != 0xf0 {
        return false;
    }
    crc8(&crc_input(b), 0x31, 0x6c) == 0
}

/// The five bytes the CRC covers: seven data nibbles, a zero nibble, then the
/// two CRC nibbles, which straddle a byte boundary in the frame.
fn crc_input(b: &[u8]) -> [u8; 5] {
    [b[0], b[1], b[2], b[3] & 0xf0, ((b[3] & 0x0f) << 4) | (b[4] >> 4)]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::crc8 as crc;
    use crate::protocol::Value;

    fn frame(id: u8, channel: u8, temp_c: f64, battery_ok: bool) -> BitBuffer {
        let raw = ((temp_c * 10.0).round() as i16 & 0x0fff) as u16;
        let mut b = [0u8; 5];
        b[0] = id;
        b[1] = (if battery_ok { 0x80 } else { 0 }) | ((channel - 1) << 4) | (raw >> 8) as u8;
        b[2] = raw as u8;
        b[3] = 0xf0;
        // The check is that the CRC over all five bytes comes to zero, and
        // the step for the last byte is invertible, so the byte that makes it
        // zero is simply the CRC of the four before it.
        let c = crc(&[b[0], b[1], b[2], 0xf0], 0x31, 0x6c);
        b[3] |= c >> 4;
        b[4] = (c & 0x0f) << 4;

        let mut out = BitBuffer::new();
        for i in 0..FRAME_BITS {
            out.push(b[i / 8] & (0x80 >> (i % 8)) != 0);
        }
        out
    }

    #[test]
    fn decodes_a_rubicson_frame() {
        let r = Rubicson.decode(&frame(0x74, 1, 14.9, true)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x74)));
        assert_eq!(r.get("channel"), Some(&Value::Int(1)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(14.9)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_frost_reading_survives_the_sign_extension() {
        let r = Rubicson.decode(&frame(0x74, 3, -3.4, false)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-3.4)));
        assert_eq!(r.get("channel"), Some(&Value::Int(3)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_corrupt_frame_fails_its_crc() {
        let f = frame(0x74, 1, 14.9, true);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 20 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert_eq!(Rubicson.decode(&broken), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn nexus_does_not_also_claim_a_rubicson_frame() {
        // The two layouts differ only in what the last byte means, so without
        // the CRC test in the Nexus decoder this frame would be reported twice
        // and once wrongly.
        let f = frame(0x74, 1, 14.9, true);
        assert!(Rubicson.decode(&f).is_ok());
        assert!(super::super::NexusTh.decode(&f).is_err());
    }
}
