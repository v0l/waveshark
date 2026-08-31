//! Linear: a 10-bit fixed-code remote (Linear 3089 and the DIP-switch
//! remotes that share its chip).
//!
//! A short mark is a `0` and a long mark a `1`, so the frame is found on the
//! inverted buffer. The Flipper notes the decoder collects the code
//! inverted relative to the label, and its display flips it back: the value
//! that matches a receiver's DIP switches is `!data & 0x3ff`, which is what
//! this reports as `code`.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, plausible, pwm};
use crate::slicer::Timing;

pub struct Linear;

const FRAME_BITS: usize = 10;

impl Protocol for Linear {
    fn name(&self) -> &'static str {
        "Linear"
    }

    fn timing(&self) -> Timing {
        pwm(500, 1500, 2500)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        find_and_parse(bits, FRAME_BITS, true, |b| {
            let data = ((b[0] as u16) << 2) | (b[1] as u16 >> 6);
            if !plausible(data as u64, FRAME_BITS as u32) {
                return None;
            }
            // The collected bits are inverted relative to the printed key.
            let code = !data & 0x3ff;
            let mut r = Report::new("Linear");
            r.crc_valid = None;
            r.raw = b[..2].to_vec();
            Some(r.int("code", code as i64).int("raw", data as i64))
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
    fn decodes_a_dip_switch_code() {
        // On-air data 0x155 inverts to the printed key 0x2aa.
        let r = Linear.decode(&input(0x155)).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0x2aa)));
        assert_eq!(r.get("raw"), Some(&Value::Int(0x155)));
        assert_eq!(r.crc_valid, None);
    }
}
