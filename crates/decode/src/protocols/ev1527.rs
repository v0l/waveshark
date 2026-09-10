//! EV1527, PT2260/PT2262 and SC2260/SC2262 fixed-code remotes.
//!
//! Doorbells, garage and gate remotes, PIR sensors and door contacts: the most
//! common thing on 433.92 and 315 MHz, and the least verifiable. A frame is 24
//! data bits and a sync mark, with no checksum of any kind, so this decoder
//! reports `crc_valid: None` and requires the burst to be exactly one frame
//! long. Even then it will claim the occasional burst belonging to something
//! else. That is inherent to the protocol, not a shortcut taken here.
//!
//! ```text
//! IIII IIII IIII IIII CCCC CCCC S
//! ```
//!
//! - `I` 16 bit address, fixed in the transmitter at the factory or by
//!   solder bridges
//! - `C` 8 bit command, one bit per button on most remotes
//! - `S` the sync mark, which is always a short pulse
//!
//! The `tristate` field is the PT226x view of the same bits, where each pair
//! is one of the three states a pin can be strapped to. It is what a remote's
//! DIP switches read as, so it is the form worth writing down when cloning a
//! sensor's address into a receiver.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::Timing;

pub struct Ev1527;

const FRAME_BITS: usize = 25;

impl Protocol for Ev1527 {
    fn name(&self) -> &'static str {
        "Generic-Remote"
    }

    fn timing(&self) -> Timing {
        Timing::pwm(464, 1404, 1800).with_tolerance(200)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // One frame per burst, nothing before or after it. With no integrity
        // check whatsoever, the frame's own length is the only evidence that
        // this is what it claims to be.
        if bits.len() < FRAME_BITS || bits.len() > FRAME_BITS + 1 {
            return Err(DecodeError::WrongLength { got: bits.len(), want: FRAME_BITS });
        }
        // The 25th bit is the sync mark, always short and so always a 1. The
        // data bits are documented with the opposite polarity, which is why
        // only the first three bytes are inverted.
        if bits.get(24) != Some(true) {
            return Err(DecodeError::NotThisProtocol);
        }
        let raw = bits.slice(0, 24);
        let b: Vec<u8> = raw.as_bytes().iter().map(|v| !v).collect();

        // An all-zero address or command is what an empty buffer decodes to,
        // and no real remote ships with either.
        if (b[0] == 0 && b[1] == 0) || b[2] == 0 {
            return Err(DecodeError::Implausible("address or command is zero"));
        }

        let full = (b[0] as u32) << 16 | (b[1] as u32) << 8 | b[2] as u32;
        let mut r = Report::new(self.name());
        r.crc_valid = None;
        r.raw = b.clone();
        Ok(r.int("id", ((b[0] as i64) << 8) | b[1] as i64)
            .int("cmd", b[2] as i64)
            .text("tristate", tristate(full)))
    }
}

/// The PT226x tri-state reading of 24 bits: each pair is one pin.
fn tristate(full: u32) -> String {
    (0..12)
        .rev()
        .map(|i| match (full >> (i * 2)) & 0x03 {
            0b00 => '0',
            0b01 => 'Z', // floating
            0b10 => 'X', // invalid on an SC226x, legal on an EV1527
            _ => '1',
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A frame as the slicer produces it: data inverted, sync bit set.
    fn frame(id: u16, cmd: u8) -> BitBuffer {
        let mut b = BitBuffer::new();
        for byte in [!(id >> 8) as u8, !(id as u8), !cmd] {
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        b.push(true);
        b
    }

    #[test]
    fn decodes_a_remote_press() {
        let r = Ev1527.decode(&frame(0xa13f, 0x08)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0xa13f)));
        assert_eq!(r.get("cmd"), Some(&Value::Int(8)));
        assert_eq!(r.crc_valid, None, "there is no check to pass");
    }

    #[test]
    fn tristate_reads_pin_pairs() {
        // Pairs of 0b11 read as 1 and 0b00 as 0, so 0xff00f0 is twelve pins:
        // four ones, four zeros, two ones, two zeros.
        let r = Ev1527.decode(&frame(0xff00, 0xf0)).unwrap();
        assert_eq!(r.get("tristate"), Some(&Value::Text("111100001100".into())));
    }

    #[test]
    fn a_missing_sync_bit_is_not_this_protocol() {
        let f = frame(0xa13f, 0x08);
        let mut no_sync = f.slice(0, 24);
        no_sync.push(false);
        assert_eq!(Ev1527.decode(&no_sync), Err(DecodeError::NotThisProtocol));
    }

    #[test]
    fn a_burst_longer_than_one_frame_is_refused() {
        // Without this the decoder would find a "remote" in any long burst,
        // because there is nothing in the frame to disagree with.
        let f = frame(0xa13f, 0x08);
        let mut long = BitBuffer::new();
        for _ in 0..3 {
            for i in 0..f.len() {
                long.push(f.get(i).unwrap());
            }
        }
        assert!(matches!(Ev1527.decode(&long), Err(DecodeError::WrongLength { .. })));
    }

    #[test]
    fn a_zero_address_is_refused() {
        assert_eq!(
            Ev1527.decode(&frame(0x0000, 0x08)),
            Err(DecodeError::Implausible("address or command is zero"))
        );
    }
}
