//! Princeton (also the 24-bit "Generic" fixed-code remote many clone it
//! from): the commonest garage-door keyfob on 433.92 MHz.
//!
//! A short mark is a 0 and a long mark is a 1, transmitted as a burst of
//! identical 24-bit frames behind a ~14 ms preamble. There is no checksum of
//! any kind, so the decoder reports `crc_valid: None` and relies on the
//! frame's repeat count (the corroboration in `find_and_parse`) to tell a
//! real remote from a burst that happens to decode. The 24-bit code splits as
//! 20-bit serial and 4-bit button for the common remotes; a few use one of
//! four 8-bit button codes in the low byte, which the Flipper detects and
//! reports as an 8-bit button. The serial/button split is shown here the same
//! way the Flipper's `check_remote_controller` does.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, pwm};
use crate::slicer::Timing;

/// A Princeton 24-bit fixed-code remote.
pub struct Princeton;

const FRAME_BITS: usize = 24;

impl Protocol for Princeton {
    fn name(&self) -> &'static str {
        "Princeton"
    }

    fn timing(&self) -> Timing {
        pwm(390, 1170, 3000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        find_and_parse(bits, FRAME_BITS, true, |b| {
            // 24-bit code, MSB first, short-mark-0 on the air.
            let code = (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32;
            if code == 0 {
                return None;
            }
            let (serial, btn) = match code & 0xff {
                // Second encoding: an 8-bit button code in the low byte.
                0x30 | 0xc0 => (code >> 8, code & 0xff),
                // Button codes 0x03/0x0c read as zero-leading; fix them up.
                0x03 | 0x0c => (code >> 8, (code & 0xff) | 0xf0),
                _ => (code >> 4, code & 0xf),
            };
            let mut r = Report::new("Princeton");
            r.crc_valid = None;
            r.raw = b.to_vec();
            Some(
                r.int("code", code as i64)
                    .int("serial", serial as i64)
                    .int("btn", btn as i64),
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::BitBuffer;
    use crate::protocol::Value;

    /// Build decode input for an on-air 24-bit code: the slicer inverts, so
    /// the frame goes in inverted.
    fn input(code: u32) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..FRAME_BITS {
            b.push(code & (1 << (FRAME_BITS - 1 - i)) != 0);
        }
        b.inverted()
    }

    #[test]
    fn decodes_a_remote_press() {
        let r = Princeton.decode(&input(0xa1_3f_08)).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0xa13f08)));
        assert_eq!(r.get("serial"), Some(&Value::Int(0xa13f0)));
        assert_eq!(r.get("btn"), Some(&Value::Int(0x8)));
        assert_eq!(r.crc_valid, None);
    }

    #[test]
    fn finds_the_frame_among_repeats() {
        // Three identical frames back to back, as a real burst sends.
        let mut b = BitBuffer::new();
        for _ in 0..3 {
            for i in 0..FRAME_BITS {
                b.push(0xa1_3f_08 & (1 << (FRAME_BITS - 1 - i)) != 0);
            }
        }
        let r = Princeton.decode(&b.inverted()).unwrap();
        assert_eq!(r.get("code"), Some(&Value::Int(0xa13f08)));
    }

    #[test]
    fn an_8bit_button_code_is_reported_as_such() {
        let r = Princeton.decode(&input(0x1234_c0)).unwrap();
        assert_eq!(r.get("serial"), Some(&Value::Int(0x1234)));
        assert_eq!(r.get("btn"), Some(&Value::Int(0xc0)));
    }

    #[test]
    fn a_zero_code_is_not_this_protocol() {
        assert_eq!(Princeton.decode(&input(0)), Err(DecodeError::NotThisProtocol));
    }
}
