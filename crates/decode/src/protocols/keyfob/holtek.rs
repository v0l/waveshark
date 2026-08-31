//! Holtek (HT6P20/HT6P30B family): 40-bit fixed-code remotes, one of the most
//! common doorbell and gate fobs in Asia and Europe.
//!
//! A short mark is a 0 and a long mark is a 1 (inverted from the slicer), so
//! the frame is found on the inverted buffer. The 40 bits are:
//!
//! ```text
//! HHHH SSSS SSSS SSSS SSSS BBBB BBBB BBBB BBBB
//! ```
//!
//! - `H` 4-bit header, always `0x5`, which is what pins the frame's alignment
//! - `S` 20-bit serial, transmitted least-significant-bit first
//! - `B` 16 bits as four 4-bit button nibbles, every one `0xA` except the
//!   nibble belonging to the pressed button
//!
//! There is no checksum. The header and the repeat corroboration are the only
//! integrity check, which is what the Flipper relies on too.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::keyfob::shared::{find_and_parse, pwm, reverse_key};
use crate::slicer::Timing;

pub struct Holtek;

const FRAME_BITS: usize = 40;
const HEADER_MASK: u64 = 0xf000_0000_00;
const HEADER: u64 = 0x5000_0000_00;

impl Protocol for Holtek {
    fn name(&self) -> &'static str {
        "Holtek"
    }

    fn timing(&self) -> Timing {
        pwm(430, 870, 4000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        find_and_parse(bits, FRAME_BITS, true, |b| {
            let data = (b[0] as u64) << 32
                | (b[1] as u64) << 24
                | (b[2] as u64) << 16
                | (b[3] as u64) << 8
                | b[4] as u64;
            if data & HEADER_MASK != HEADER {
                return None;
            }
            // Serial is the 20 bits under the header, sent LSB first.
            let serial = reverse_key((data >> 16) & 0xfffff, 20);
            // Four 4-bit button nibbles in the low 16 bits; all read 0xA
            // except the pressed one, whose index is the button number.
            let btn = match (data & 0xffff) as u16 {
                n if n & 0xf != 0xa => 1,
                n if (n >> 4) & 0xf != 0xa => 2,
                n if (n >> 8) & 0xf != 0xa => 3,
                n if (n >> 12) & 0xf != 0xa => 4,
                _ => return None,
            };
            let mut r = Report::new("Holtek");
            r.crc_valid = None;
            r.raw = b.to_vec();
            Some(r.int("code", data as i64).int("serial", serial as i64).int("btn", btn))
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::BitBuffer;
    use crate::protocol::Value;

    fn on_air(header: u8, serial: u64, pressed_nibble: u8) -> u64 {
        // Four nibbles all 0xa except the pressed one.
        let nibbles: [u8; 4] = [pressed_nibble, 0xa, 0xa, 0xa];
        let mut btn_field = 0u64;
        for (i, n) in nibbles.iter().enumerate() {
            btn_field |= (*n as u64) << (i * 4);
        }
        // Serial is transmitted LSB first, so it goes on the wire reversed
        // and the decoder reverses it back to `serial`.
        let data = (header as u64) << 36 | reverse_key(serial, 20) << 16 | btn_field;
        data
    }

    /// Build decode input: invert the on-air frame.
    fn input(data: u64) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..FRAME_BITS {
            b.push(data & (1 << (FRAME_BITS - 1 - i)) != 0);
        }
        b.inverted()
    }

    #[test]
    fn decodes_a_key_press() {
        // serial 0x12345 transmitted LSB first -> reverse_key makes it back.
        let data = on_air(0x5, 0x12345, 0x1);
        let r = Holtek.decode(&input(data)).unwrap();
        assert_eq!(r.get("serial"), Some(&Value::Int(0x12345)));
        assert_eq!(r.get("btn"), Some(&Value::Int(1)));
        assert_eq!(r.crc_valid, None);
    }

    #[test]
    fn a_wrong_header_is_not_this_protocol() {
        assert_eq!(Holtek.decode(&input(on_air(0x7, 0x12345, 0x1))), Err(DecodeError::NotThisProtocol));
    }

    #[test]
    fn no_button_nibble_means_reject() {
        // All four nibbles 0xa: no button pressed, so no valid frame.
        let data = 0x5000_0000_00 | (0x12345u64 << 16) | 0xaaaa;
        assert_eq!(Holtek.decode(&input(data)), Err(DecodeError::NotThisProtocol));
    }
}
