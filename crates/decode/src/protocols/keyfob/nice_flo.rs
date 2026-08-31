//! Nice FLO: an Italian gate remote's 12-bit fixed code (the newer FLO2/FLOR
//! rolling codes live under their own names).
//!
//! A short mark is a `0` and a long mark a `1`, so the frame is found on the
//! inverted buffer. No checksum; the frame length and repeats corroborate.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, pwm};
use crate::slicer::Timing;

pub struct NiceFlo;

const FRAME_BITS: usize = 12;

impl Protocol for NiceFlo {
    fn name(&self) -> &'static str {
        "Nice-Flo"
    }

    fn timing(&self) -> Timing {
        pwm(700, 1400, 3000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        find_and_parse(bits, FRAME_BITS, true, |b| {
            let code = ((b[0] as u16) << 4) | (b[1] as u16 >> 4);
            if code == 0 {
                return None;
            }
            let mut r = Report::new("Nice-Flo");
            r.crc_valid = None;
            r.raw = b[..2].to_vec();
            Some(r.int("code", code as i64))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::BitBuffer;
    use crate::protocol::Value;

    fn input(code: u16) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..FRAME_BITS {
            b.push(code & (1 << (FRAME_BITS - 1 - i)) != 0);
        }
        b.inverted()
    }

    #[test]
    fn decodes_a_gate_code() {
        let r = NiceFlo.decode(&input(0xabc)).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0xabc)));
        assert_eq!(r.crc_valid, None);
    }
}
