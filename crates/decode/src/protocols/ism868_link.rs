//! A two-way 868 MHz link with a "GO" sync, vendor unknown.
//!
//! Heard continuously in Ireland on 868.100 and 868.500 MHz: 2-FSK at
//! 19.6 kbaud NRZ, an 80-bit alternating preamble, the sync `47 4F` ("GO"),
//! a 16-bit id whose low two bits cycle, and 17 to 19 bytes that are
//! block-encrypted (byte entropy 7.98 bits, no counter, no field a
//! de-whitening finds). Two ids relay one another's bodies with only the
//! outer bytes changed, each sends every 1 to 15 seconds all day, and there
//! is no integrity check to find because the check is inside the cipher.
//!
//! Two identities, not a houseful: a hub and a repeater keeping a link up,
//! rather than sensors reporting. Repeated 8-byte blocks recur at a fixed
//! offset under the same slot from both identities, which is a 64-bit block
//! cipher in ECB relaying one payload verbatim.
//!
//! Ajax Jeweller fits the band, the hopping and the "block encryption" its
//! literature claims, but so does every other 868 MHz alarm sold here, and
//! nobody has published this sync word. So the name says what was measured
//! and not whose it is. What is read is the framing; nothing inside is.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing};

pub struct Ism868Link;

/// "GO".
const SYNC: u32 = 0x474f;
const SYNC_BITS: usize = 16;
/// Alternating bits that must precede the sync. The preamble on the air is
/// eighty; a receiver that opened late has fewer, and noise has none.
const PREAMBLE_MIN: usize = 16;
/// Id and the least body seen, in bytes; a shorter match is the sync
/// occurring inside something else.
const MIN_BYTES: usize = 2 + 12;
const MAX_BYTES: usize = 2 + 24;

impl Protocol for Ism868Link {
    fn name(&self) -> &'static str {
        "ISM868-Link"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: 51,
            long_us: 51,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 400,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let need = PREAMBLE_MIN + SYNC_BITS + MIN_BYTES * 8;
        if bits.len() < need {
            return Err(DecodeError::WrongLength { got: bits.len(), want: need });
        }
        for at in PREAMBLE_MIN..bits.len() - SYNC_BITS - MIN_BYTES * 8 {
            if bits.extract(at, SYNC_BITS) != Some(SYNC) {
                continue;
            }
            let alternating = (1..=PREAMBLE_MIN).all(|k| bits.get(at - k) != bits.get(at - k + 1))
                || (at >= PREAMBLE_MIN + 1 && (1..=PREAMBLE_MIN).all(|k| bits.get(at - k - 1) != bits.get(at - k)));
            if !alternating {
                continue;
            }
            let n = ((bits.len() - at - SYNC_BITS) / 8).min(MAX_BYTES);
            let mut body = Vec::with_capacity(n);
            for i in 0..n {
                let Some(v) = bits.extract(at + SYNC_BITS + i * 8, 8) else { break };
                body.push(v as u8);
            }
            // The air frame ends on a bit boundary rather than a byte one, so
            // the slicer's tail is padding; strip the zero bytes it becomes.
            while body.len() > MIN_BYTES && *body.last().unwrap() == 0 {
                body.pop();
            }
            if body.len() < MIN_BYTES {
                return Err(DecodeError::WrongLength { got: body.len(), want: MIN_BYTES });
            }
            let id = u16::from_be_bytes([body[0], body[1]]);
            let mut r = Report::new(self.name());
            r.crc_valid = None;
            r.raw = body.clone();
            return Ok(r
                .text("node", format!("{:04x}", id >> 2))
                .int("slot", i64::from(id & 3))
                .int("length", body.len() as i64 - 2)
                .text("body", body[2..].iter().map(|b| format!("{b:02x}")).collect::<String>())
                .bool("encrypted", true));
        }
        Err(DecodeError::NotThisProtocol)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn bits_of(preamble: usize, bytes: &[u8]) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..preamble {
            b.push(i % 2 == 0);
        }
        for byte in bytes {
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        b
    }

    /// A frame as logged on 868.1 MHz on 2026-09-04, node 0x2efd slot 3.
    #[test]
    fn a_logged_frame_reads_its_node_and_slot() {
        let frame: Vec<u8> = (0.."474fbbf729ad1fe85e2e03f8c34ba9fc5f9a21987ffa".len() / 2)
            .map(|i| u8::from_str_radix(&"474fbbf729ad1fe85e2e03f8c34ba9fc5f9a21987ffa"[i * 2..i * 2 + 2], 16).unwrap())
            .collect();
        let r = Ism868Link.decode(&bits_of(83, &frame)).expect("a frame");
        assert_eq!(r.fields["node"], Value::Text("2efd".into()));
        assert_eq!(r.fields["slot"], Value::Int(3));
        assert_eq!(r.fields["length"], Value::Int(18));
        assert_eq!(r.crc_valid, None);
    }

    /// The sync inside random bits, with no preamble ahead of it, is not a
    /// frame: without a check inside, the preamble is the only corroboration.
    #[test]
    fn the_sync_alone_is_not_enough() {
        let mut junk = vec![0x3c, 0x91, 0x5a, 0x47, 0x4f];
        junk.extend_from_slice(&[0x11; 20]);
        let mut b = BitBuffer::new();
        for byte in &junk {
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        assert!(Ism868Link.decode(&b).is_err());
    }
}
