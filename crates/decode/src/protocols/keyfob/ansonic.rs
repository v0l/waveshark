//! Ansonic: a 12-bit fixed-code remote whose short mark is a `1` and long
//! mark a `0`, the same polarity super-radio's slicer already produces, so
//! (unlike most keyfobs) the frame is found without inversion.
//!
//! ```text
//! 10101010 1 01 0  k
//! ```
//! The low 11 bits are the DIP-switch address, with the button in bits 1-2:
//! `cnt = code & 0xfff`, `btn = (code >> 1) & 0x3`.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, plausible, pwm};
use crate::slicer::Timing;

pub struct Ansonic;

const FRAME_BITS: usize = 12;

impl Protocol for Ansonic {
    fn name(&self) -> &'static str {
        "Ansonic"
    }

    fn timing(&self) -> Timing {
        pwm(555, 1111, 2500)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // Note: no inversion. Ansonic sends short-mark 1.
        find_and_parse(bits, FRAME_BITS, false, |b| {
            let code = ((b[0] as u16) << 4) | (b[1] as u16 >> 4);
            if !plausible(code as u64, FRAME_BITS as u32) {
                return None;
            }
            let mut r = Report::new("Ansonic");
            r.crc_valid = None;
            r.raw = b[..2].to_vec();
            Some(
                r.int("code", code as i64)
                    .int("cnt", (code & 0xfff) as i64)
                    .int("btn", ((code >> 1) & 0x3) as i64),
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::BitBuffer;
    use crate::protocol::Value;

    /// No inversion: decode input is the on-air frame as-is.
    fn input(code: u16) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..FRAME_BITS {
            b.push(code & (1 << (FRAME_BITS - 1 - i)) != 0);
        }
        b
    }

    #[test]
    fn decodes_a_key_press() {
        // btn = bits 1-2 of 0x123.
        let r = Ansonic.decode(&input(0x123)).unwrap();
        assert_eq!(r.get("cnt"), Some(&Value::Int(0x123)));
        assert_eq!(r.get("btn"), Some(&Value::Int(0x123 >> 1 & 0x3)));
    }
}
