//! Alarm system sensors.
//!
//! Door contacts, motion detectors and glass break sensors report to a panel
//! over a one-way radio link with no encryption and no rolling code. Anything
//! within range can read which door in which house just opened, which is worth
//! knowing about a technology sold as security.

use crate::bits::{crc16, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{manchester_decode, Coding, Timing};

/// Honeywell (Ademco) door and window sensors: the 5811 and 5816, 2Gig's DW10
/// and DW11, the RE208 repeater and the 2GIG-GB1 glass break detector.
///
/// 345 MHz, OOK, Manchester at 136 us a half symbol. The frame is a preamble
/// and eight bytes:
///
/// ```text
/// CI II IE SS
/// ```
///
/// - `C` channel, which also says whose CRC polynomial the frame uses
/// - `I` 20 bit device serial, the number engraved on the sensor
/// - `E` event bits: contact, tamper, reed switch, alarm, low battery and
///   heartbeat
/// - `SS` CRC16 over the four bytes before it
///
/// The preamble is searched for in the half-symbol stream rather than paired
/// from bit zero, because it is a run of alternating halves ending in one that
/// does not alternate, and a receiver whose AGC was still settling will have
/// produced noise in front of it. Every match is tried, not the first: inside
/// that run a 24 bit window matches at the wrong alignment too, and only the
/// CRC can tell them apart.
pub struct HoneywellSecurity;

/// The preamble as it appears before Manchester decoding: `0xfffe` encoded.
const PREAMBLE: u32 = 0x55_5556;
const PREAMBLE_BITS: usize = 24;
/// Bytes the decoder reads out of a frame, of the eight it carries.
const MSG_BYTES: usize = 6;

impl Protocol for HoneywellSecurity {
    fn name(&self) -> &'static str {
        "Honeywell-Security"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: 136,
            long_us: 136,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 408,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        if bits.len() < 120 {
            return Err(DecodeError::WrongLength { got: bits.len(), want: 120 });
        }
        let mut best = Err(DecodeError::NotThisProtocol);
        for at in 0..bits.len() - PREAMBLE_BITS {
            if bits.extract(at, PREAMBLE_BITS) != Some(PREAMBLE) {
                continue;
            }
            match frame(&manchester_decode(bits, at + PREAMBLE_BITS)) {
                Ok(r) => return Ok(r),
                Err(e) => best = Err(e),
            }
        }
        best
    }
}

fn frame(decoded: &BitBuffer) -> Result<Report, DecodeError> {
    if decoded.len() < MSG_BYTES * 8 {
        return Err(DecodeError::WrongLength { got: decoded.len(), want: MSG_BYTES * 8 });
    }
    let b = decoded.as_padded_bytes();
    let channel = b[0] >> 4;
    let id = ((b[0] as u32 & 0x0f) << 16) | (b[1] as u32) << 8 | b[2] as u32;
    let crc = ((b[4] as u16) << 8) | b[5] as u16;
    if id == 0 && crc == 0 {
        return Err(DecodeError::Implausible("empty frame"));
    }
    // 2Gig's sensors use one polynomial and Honeywell's own another, and the
    // channel is what says which. An unknown channel is refused rather than
    // guessed at, since guessing halves the strength of the only check here.
    let poly = match channel {
        0x2 | 0x4 | 0x9 | 0xa | 0xc => 0x8050,
        0x8 => 0x8005,
        _ => return Err(DecodeError::NotThisProtocol),
    };
    if crc != crc16(&b[..4], poly, 0) {
        return Err(DecodeError::CrcFailed);
    }

    let event = b[3];
    let mut r = Report::new("Honeywell-Security");
    r.crc_valid = Some(true);
    r.raw = b[..MSG_BYTES].to_vec();
    Ok(r
        .int("id", id as i64)
        .int("channel", channel as i64)
        .int("event", event as i64)
        .text("state", if event & 0x80 != 0 { "open" } else { "closed" })
        .bool("contact_open", event & 0x80 != 0)
        .bool("reed_open", event & 0x20 != 0)
        .bool("alarm", event & 0x10 != 0)
        .bool("tamper", event & 0x40 != 0)
        .bool("battery_ok", event & 0x08 == 0)
        .bool("heartbeat", event & 0x04 != 0))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A frame as the sensor sends it: preamble, then each bit as a
    /// low-then-high half pair for a 1.
    fn burst(msg: &[u8; MSG_BYTES]) -> BitBuffer {
        let mut b = BitBuffer::new();
        // A little noise in front, which is what the preamble search is for.
        for bit in [true, true, false, true, true] {
            b.push(bit);
        }
        for i in 0..PREAMBLE_BITS {
            b.push(PREAMBLE & (1 << (PREAMBLE_BITS - 1 - i)) != 0);
        }
        for byte in msg {
            for i in 0..8 {
                let one = byte & (0x80 >> i) != 0;
                b.push(!one);
                b.push(one);
            }
        }
        b
    }

    /// rtl_433's `honeywell_5816/g001` capture: id 231303 on channel 8, an
    /// open contact.
    fn frame_5816() -> [u8; MSG_BYTES] {
        let mut m = [0x83, 0x87, 0x87, 0xa0, 0x00, 0x00];
        let crc = crc16(&m[..4], 0x8005, 0);
        m[4] = (crc >> 8) as u8;
        m[5] = crc as u8;
        m
    }

    #[test]
    fn decodes_a_door_sensor_opening() {
        let r = HoneywellSecurity.decode(&burst(&frame_5816())).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x38787)));
        assert_eq!(r.get("channel"), Some(&Value::Int(8)));
        assert_eq!(r.get("state"), Some(&Value::Text("open".into())));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.get("heartbeat"), Some(&Value::Bool(false)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_2gig_channel_uses_the_other_polynomial() {
        let mut m = [0xa1, 0x20, 0x56, 0xa0, 0x00, 0x00];
        let crc = crc16(&m[..4], 0x8050, 0);
        m[4] = (crc >> 8) as u8;
        m[5] = crc as u8;
        let r = HoneywellSecurity.decode(&burst(&m)).unwrap();
        assert_eq!(r.get("channel"), Some(&Value::Int(0xa)));
        assert_eq!(r.get("id"), Some(&Value::Int(0x12056)));
    }

    #[test]
    fn a_corrupt_frame_fails_its_crc() {
        let mut m = frame_5816();
        m[2] ^= 0x08;
        assert_eq!(HoneywellSecurity.decode(&burst(&m)), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn an_unknown_channel_is_not_claimed() {
        let mut m = frame_5816();
        m[0] = 0x53;
        let crc = crc16(&m[..4], 0x8005, 0);
        m[4] = (crc >> 8) as u8;
        m[5] = crc as u8;
        assert_eq!(
            HoneywellSecurity.decode(&burst(&m)),
            Err(DecodeError::NotThisProtocol)
        );
    }
}
