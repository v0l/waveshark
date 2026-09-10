//! Holtek HT12x (HT12D/HT12E): 12-bit fixed-code remotes, the smallest fob
//! format the Flipper supports.
//!
//! A short mark is a 0 and a long mark is a 1 (inverted from the slicer). The
//! 12-bit code splits as an 8-bit serial/address and a 4-bit button:
//!
//! ```text
//! AAAA AAAA BBBB
//! ```

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, plausible, pwm};
use crate::slicer::Timing;

pub struct HoltekHt12x;

const FRAME_BITS: usize = 12;

impl Protocol for HoltekHt12x {
    fn name(&self) -> &'static str {
        "Holtek-HT12x"
    }

    fn timing(&self) -> Timing {
        pwm(320, 640, 2500)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        find_and_parse(bits, FRAME_BITS, true, |b| {
            let code = ((b[0] as u16) << 4) | (b[1] as u16 >> 4);
            if !plausible(code as u64, FRAME_BITS as u32) {
                return None;
            }
            let mut r = Report::new("Holtek-HT12x");
            r.crc_valid = None;
            r.raw = b[..2].to_vec();
            Some(
                r.int("code", code as i64)
                    .int("serial", ((code >> 4) & 0xff) as i64)
                    .int("btn", (code & 0xf) as i64),
            )
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
    fn decodes_a_key_press() {
        let r = HoltekHt12x.decode(&input(0xabc)).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0xabc)));
        assert_eq!(r.get("serial"), Some(&Value::Int(0xab)));
        assert_eq!(r.get("btn"), Some(&Value::Int(0xc)));
        assert_eq!(r.crc_valid, None);
    }

    #[test]
    fn a_zero_code_is_rejected() {
        assert_eq!(HoltekHt12x.decode(&input(0)), Err(DecodeError::NotThisProtocol));
    }
}
