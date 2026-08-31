//! Linear Delta3: an 8-bit rolling-code remote whose short mark is a `1` and
//! long mark a `0`, the same polarity as the slicer, so the frame is found
//! without inversion (the opposite of the fixed-code Linear above).
//!
//! The 8-bit code is a per-press counter the receiver compares against a
//! window; this decoder surfaces the counter, which is all a receiver without
//! the learned key can do with a rolling code.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, plausible, pwm};
use crate::slicer::Timing;

pub struct LinearDelta3;

const FRAME_BITS: usize = 8;

impl Protocol for LinearDelta3 {
    fn name(&self) -> &'static str {
        "Linear-Delta3"
    }

    fn timing(&self) -> Timing {
        pwm(500, 2000, 4000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // No inversion: Delta3 sends short-mark 1.
        find_and_parse(bits, FRAME_BITS, false, |b| {
            let code = b[0];
            if !plausible(code as u64, FRAME_BITS as u32) {
                return None;
            }
            let mut r = Report::new("Linear-Delta3");
            r.crc_valid = None;
            r.raw = b[..1].to_vec();
            Some(r.int("cnt", code as i64))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::BitBuffer;
    use crate::protocol::Value;

    fn input(code: u8) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..FRAME_BITS {
            b.push(code & (1 << (FRAME_BITS - 1 - i)) != 0);
        }
        b
    }

    #[test]
    fn decodes_a_counter_value() {
        let r = LinearDelta3.decode(&input(0x5a)).unwrap();
        assert_eq!(r.get("cnt"), Some(&Value::Int(0x5a)));
        assert_eq!(r.crc_valid, None);
    }
}
