//! CAME: the Spanish gate-and-remote maker's fixed-code fobs, in 12-bit and
//! 24-bit forms (plus, on the same timing, AirForce and PRASTEL which the
//! Flipper detects from the frame length; only the two CAME lengths are
//! registered here).
//!
//! A short mark is a `0` and a long mark a `1`, so frames are found on the
//! inverted buffer. There is no checksum: the frame length and its repeats
//! are the only integrity check. One decoder struct, two registered
//! instances differing only in frame length, so the 24-bit frame is not also
//! matched as two adjacent 12-bit halves.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, plausible, pwm};
use crate::slicer::Timing;

pub struct Came {
    name: &'static str,
    frame_bits: usize,
}

/// CAME 12-bit fixed code.
pub fn came12_bit() -> Came {
    Came { name: "CAME-12bit", frame_bits: 12 }
}

/// CAME 24-bit fixed code.
pub fn came24_bit() -> Came {
    Came { name: "CAME-24bit", frame_bits: 24 }
}

impl Protocol for Came {
    fn name(&self) -> &'static str {
        self.name
    }

    fn timing(&self) -> Timing {
        pwm(320, 640, 2500)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let fb = self.frame_bits;
        find_and_parse(bits, fb, true, |b| {
            let code = if fb == 12 {
                ((b[0] as u32) << 4) | (b[1] as u32 >> 4)
            } else {
                (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32
            };
            if !plausible(code as u64, fb as u32) {
                return None;
            }
            let mut r = Report::new(self.name);
            r.crc_valid = None;
            r.raw = b.to_vec();
            Some(r.int("code", code as i64))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::BitBuffer;
    use crate::protocol::Value;

    fn input(fb: usize, code: u32) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..fb {
            b.push(code & (1 << (fb - 1 - i)) != 0);
        }
        b.inverted()
    }

    #[test]
    fn decodes_a_12bit_frame() {
        let r = came12_bit().decode(&input(12, 0xabc)).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0xabc)));
        assert_eq!(r.crc_valid, None);
    }

    #[test]
    fn decodes_a_24bit_frame() {
        let r = came24_bit().decode(&input(24, 0xabc_def)).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0xabc_def)));
    }

    #[test]
    fn a_12bit_decoder_does_not_swallow_a_24bit_frame() {
        // Two 12-bit halves both decode, but the 24-bit frame does not match
        // the 12-bit frame length with corroboration.
        assert_eq!(came12_bit().decode(&input(24, 0xabc_def)), Err(DecodeError::NotThisProtocol));
    }
}
