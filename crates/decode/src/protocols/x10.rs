//! X10 RF: the wireless side of the oldest home automation system still in
//! service. Remotes, wall switches, motion sensors and door contacts.
//!
//! 310 MHz in North America, 433.92 MHz in Europe. A 9 ms sync mark and a
//! 4.5 ms gap, then 32 PPM bits, repeated five times with a 40 ms gap. The
//! framing is close enough to the NEC infrared protocol to have been borrowed
//! from it, complement bytes included.
//!
//! ```text
//! HHHH xUxx  ~~~~ ~~~~  EUUS UUUx  ~~~~ ~~~~
//! ```
//!
//! - byte 1 is the complement of byte 0, and byte 3 of byte 2. That is the
//!   whole integrity check, and at sixteen bits it is a better one than most
//!   of the sensors on these bands manage
//! - `H` house code A to P, in a scrambled bit order
//! - `U` unit number 1 to 16, spread across both halves of the frame
//! - `S` state, on or off
//! - `E` marks a house-wide command (dim, bright, all lights on, all off)
//!   rather than a command to one unit

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::find_frame;
use crate::slicer::Timing;

pub struct X10Rf;

const FRAME_BYTES: usize = 4;

/// Bits that are the same in every frame, and what they have to be. Cheap, and
/// they cut the false positive rate by another four bits on top of the
/// complement check.
const FIXED: [(u8, u8); 4] = [(0x0b, 0x00), (0x0b, 0x0b), (0x07, 0x00), (0x07, 0x07)];

impl Protocol for X10Rf {
    fn name(&self) -> &'static str {
        "X10-RF"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(562, 1687, 6000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let b = find_frame(bits, FRAME_BYTES, |b| {
            b[0] ^ b[1] == 0xff
                && b[2] ^ b[3] == 0xff
                && FIXED.iter().zip(b).all(|((mask, want), v)| v & mask == *want)
        })
        .ok_or(match bits.len() {
            n if n < FRAME_BYTES * 8 => DecodeError::WrongLength { got: n, want: FRAME_BYTES * 8 },
            _ => DecodeError::CrcFailed,
        })?;

        // The house code is Gray-ish rather than binary: the bits are a
        // scrambled function of each other, which is how the original
        // hardware's rotary switch was wired.
        let h: Vec<u8> = (4..8).map(|i| (b[0] >> (7 - (i - 4))) & 1).collect();
        let house = ((!(h[0] ^ h[1]) & 1) << 3) | ((!h[1] & 1) << 2) | ((h[1] ^ h[2]) << 1) | h[3];
        let mut unit = ((b[0] & 0x04) << 1)
            | ((b[2] & 0x40) >> 4)
            | ((b[2] & 0x08) >> 2)
            | ((b[2] & 0x10) >> 4);
        unit += 1;

        let mut r = Report::new(self.name());
        // Two complement bytes are a real check, if a weak one: they catch
        // every single-bit error and most bursts.
        r.crc_valid = Some(true);
        r.raw = b.clone();
        let state = if b[2] & 0x80 != 0 {
            // A house-wide command names no unit.
            unit = 0;
            match b[2] {
                0x98 => "DIM",
                0x88 => "BRIGHT",
                0x90 => "ALL LIGHTS ON",
                0x80 => "ALL OFF",
                _ => return Err(DecodeError::Implausible("unknown house command")),
            }
        } else if b[2] & 0x20 == 0 {
            "ON"
        } else {
            "OFF"
        };
        Ok(r.text("channel", ((b'A' + house) as char).to_string())
            .int("unit", unit as i64)
            .text("state", state))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A frame from its two data bytes, with the complements filled in.
    fn frame(b0: u8, b2: u8) -> BitBuffer {
        BitBuffer::from_bytes(&[b0, !b0, b2, !b2])
    }

    #[test]
    fn decodes_a_unit_switched_on() {
        // House A, unit 1, on: the canonical frame from the W800 protocol
        // notes.
        let r = X10Rf.decode(&frame(0x60, 0x00)).unwrap();
        assert_eq!(r.get("channel"), Some(&Value::Text("A".into())));
        assert_eq!(r.get("unit"), Some(&Value::Int(1)));
        assert_eq!(r.get("state"), Some(&Value::Text("ON".into())));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn the_state_bit_reads_off() {
        let r = X10Rf.decode(&frame(0x60, 0x20)).unwrap();
        assert_eq!(r.get("state"), Some(&Value::Text("OFF".into())));
    }

    #[test]
    fn a_house_wide_command_names_no_unit() {
        let r = X10Rf.decode(&frame(0x60, 0x80)).unwrap();
        assert_eq!(r.get("state"), Some(&Value::Text("ALL OFF".into())));
        assert_eq!(r.get("unit"), Some(&Value::Int(0)));
    }

    #[test]
    fn a_frame_whose_complement_does_not_match_is_refused() {
        let mut b = BitBuffer::from_bytes(&[0x60, !0x60u8 ^ 0x02, 0x00, 0xff]);
        b.push(false);
        assert_eq!(X10Rf.decode(&b), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn the_constant_bits_are_enforced() {
        // 0x62 sets a bit that is zero in every real frame. Without this check
        // the complement pair alone would accept it.
        assert_eq!(X10Rf.decode(&frame(0x62, 0x00)), Err(DecodeError::CrcFailed));
    }
}
