//! Bett: an 18-bit fixed-code gate remote (BETT, and the Italian gate makers
//! that share its chipset).
//!
//! A short mark (340 us) is a `0` and a long mark (2000 us) is a `1`, so the
//! frame is found on the inverted buffer. There is no checksum, no serial and
//! no button split: the whole 18-bit code is a fixed address set in the fob,
//! and the DIP-switch pattern on the receiver is derived from it. The burst
//! detector splits the long guard gap between frames, so each package is one
//! frame.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, plausible, pwm};
use crate::slicer::Timing;

pub struct Bett;

const FRAME_BITS: usize = 18;

impl Protocol for Bett {
    fn name(&self) -> &'static str {
        "Bett"
    }

    fn timing(&self) -> Timing {
        pwm(340, 2000, 15000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        find_and_parse(bits, FRAME_BITS, true, |b| {
            let code = (b[0] as u32) << 10 | (b[1] as u32) << 2 | (b[2] as u32 >> 6);
            if !plausible(code as u64, FRAME_BITS as u32) {
                return None;
            }
            let mut r = Report::new("Bett");
            r.crc_valid = None;
            r.raw = b[..3].to_vec();
            Some(r.int("code", code as i64))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::BitBuffer;
    use crate::protocol::Value;

    fn input(code: u32) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..FRAME_BITS {
            b.push(code & (1 << (FRAME_BITS - 1 - i)) != 0);
        }
        b.inverted()
    }

    #[test]
    fn decodes_a_gate_code() {
        let r = Bett.decode(&input(0x3abcd)).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0x3abcd)));
        assert_eq!(r.crc_valid, None);
    }
}
