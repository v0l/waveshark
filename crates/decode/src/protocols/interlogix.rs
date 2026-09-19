//! Interlogix, GE and UTC security sensors, and the ELK-319DWM and Alula
//! RE101 modules that speak the same protocol.
//!
//! 319.5 MHz, PPM with a 122 us short gap and a 244 us long one. The frame is
//! US patent 5761206: a front porch, twelve sync zeros, a start bit, then 46
//! bits of sensor.
//!
//! ```text
//! IIIII DDDD TTT B LLLLLLLLLL PP
//! ```
//!
//! - `I` 20 bit serial, the number engraved on the sensor
//! - `D` 4 bit device type: contact, keyfob, motion, heat or glass
//! - `T` 3 bit trigger count, which stops a recorded packet being replayed
//! - `B` low battery
//! - `L` a latch bit and a debounced level for each of five inputs, which is
//!   what says the door is open, the case has been opened or the detector has
//!   fired
//! - `P` even parity over the odd bits, odd parity over the even ones
//!
//! Two parity bits are all the integrity this frame has, and one window in
//! four satisfies them, so the report says it has no integrity check and the
//! decoder leans on the rest of the frame's shape instead: the row is 57 to 64
//! bits, the preamble is where it should be, and neither the serial nor the
//! state is all zeros or all ones. Those are rtl_433's rules, and they are
//! what stops a security sensor being invented out of a burst from something
//! else.
//!
//! A keyfob reports its buttons through the trigger count rather than through
//! the latches, which is the manufacturer's own team misreading the patent,
//! and the layout has to be read differently because of it.

use crate::bits::{BitBuffer, reflect8};
use crate::protocol::{DecodeError, Proof, Protocol, Report};
use crate::protocols::rows_within;
use crate::slicer::Timing;

pub struct InterlogixSecurity;

/// The bottom eight bits of the thirteen bit preamble, ending in the start
/// bit. Everything before it is sync and carries nothing.
const PREAMBLE: u32 = 0x01;
const PREAMBLE_BITS: usize = 8;
const MESSAGE_BITS: usize = 46;

impl Protocol for InterlogixSecurity {
    fn name(&self) -> &'static str {
        "Interlogix-Security"
    }

    fn timing(&self) -> Timing {
        // The tolerance is what keeps a row whole: rtl_433 ends a row only at
        // the 500 us reset, and a narrower row break would cut the frame at
        // its own long gaps.
        Timing::ppm(122, 244, 500).with_tolerance(128)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        if !rows_within(bits, 57..=64) {
            return Err(DecodeError::NotThisProtocol);
        }
        let last = bits.len().saturating_sub(PREAMBLE_BITS + MESSAGE_BITS);
        for at in 0..=last {
            if bits.extract(at, PREAMBLE_BITS) != Some(PREAMBLE) {
                continue;
            }
            let m = bits.slice(at + PREAMBLE_BITS, MESSAGE_BITS).as_padded_bytes().to_vec();
            if !plausible(&m) || !parity_ok(&m) {
                continue;
            }
            return Ok(report(self.name(), &m));
        }
        Err(DecodeError::NotThisProtocol)
    }
}

/// rtl_433's sanity rules: neither the serial nor the state may be all zeros
/// or all ones, both of which a dead receiver produces and no sensor sends.
fn plausible(m: &[u8]) -> bool {
    !(m[..3] == [0x00; 3] || m[..3] == [0xff; 3] || m[3..6] == [0x00; 3] || m[3..6] == [0xff; 3])
}

/// Even parity over the odd data bits and odd parity over the even ones, both
/// of which land in the top two bits of the last byte.
fn parity_ok(m: &[u8]) -> bool {
    let folded = m[0] ^ m[1] ^ m[2] ^ m[3] ^ m[4];
    let folded = (folded >> 4) ^ (folded & 0x0f);
    let folded = (folded >> 2) ^ (folded & 0x03);
    folded ^ (m[5] >> 6) == 0x03
}

fn report(name: &'static str, m: &[u8]) -> Report {
    let kind = reflect8(m[2]) >> 4;
    let keyfob = kind == 0xf;
    let latches = if keyfob {
        // A keyfob sends the button in the trigger count, so each of the five
        // is one value of that field rather than a bit of its own.
        let button = m[3] & 0x0e;
        [button == 0x04, button == 0x08, button == 0x0c, button == 0x02, button == 0x0a]
            .map(|pressed| !pressed)
    } else {
        [m[3] & 0x04, m[3] & 0x01, m[4] & 0x40, m[4] & 0x10, m[4] & 0x04].map(|bit| bit != 0)
    };

    let mut r = Report::new(name);
    // Two parity bits pass on one window in four. Whatever else that is, it is
    // not something to show beside a CRC.
    r.proof = Proof::None;
    r.raw = m.to_vec();
    r = r
        .text("subtype", device_type(kind))
        .text("id", format!("{:02x}{:02x}{:02x}", reflect8(m[2]), reflect8(m[1]), reflect8(m[0])))
        .bool("battery_ok", keyfob || m[3] & 0x10 == 0)
        .text("raw_message", format!("{:02x}{:02x}{:02x}", m[3], m[4], m[5]));
    for (i, open) in latches.iter().enumerate() {
        r = r.text(&format!("switch{}", i + 1), if *open { "OPEN" } else { "CLOSED" });
    }
    r
}

fn device_type(kind: u8) -> &'static str {
    match kind {
        0xa => "contact",
        0xf => "keyfob",
        0x4 => "motion",
        0x6 => "heat",
        0x9 => "glass",
        _ => "unknown",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A frame as the corpus holds it: sync zeros, the start bit, then the 46
    /// bit message. `serial` and `state` are the bytes rtl_433 prints as the
    /// device serial and the raw message.
    fn frame(serial: [u8; 3], state: [u8; 3]) -> BitBuffer {
        let m = [
            reflect8(serial[2]),
            reflect8(serial[1]),
            reflect8(serial[0]),
            state[0],
            state[1],
            state[2],
        ];
        let mut bits = BitBuffer::new();
        bits.extend(false, 12);
        bits.push(true);
        for i in 0..MESSAGE_BITS {
            bits.push(m[i / 8] >> (7 - i % 8) & 1 != 0);
        }
        bits
    }

    #[test]
    fn reads_a_motion_sensor() {
        // rtl_433's own reading of `interlogix/06/motion_319.5M_250k.cu8`.
        let r = InterlogixSecurity.decode(&frame([0x46, 0x8d, 0x19], [0xe9, 0x15, 0x28])).unwrap();
        assert_eq!(r.get("subtype"), Some(&Value::Text("motion".into())));
        assert_eq!(r.get("id"), Some(&Value::Text("468d19".into())));
        assert_eq!(r.get("raw_message"), Some(&Value::Text("e91528".into())));
        assert_eq!(r.get("switch1"), Some(&Value::Text("CLOSED".into())));
        assert_eq!(r.get("switch2"), Some(&Value::Text("OPEN".into())));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
    }

    #[test]
    fn reads_a_door_contact() {
        // rtl_433's reading of `interlogix/04/gfile001.cu8`, which is not in
        // the corpus here: both its transmissions land while the receiver's
        // noise floor is still settling, so the auto node reads a fragment of
        // the first burst and nothing of the second. The frame is kept as a
        // vector because it is the only contact sensor to hand, and `contact`
        // is the device type most of these are.
        let r = InterlogixSecurity.decode(&frame([0xa7, 0x10, 0x6d], [0xcd, 0x15, 0xb8])).unwrap();
        assert_eq!(r.get("subtype"), Some(&Value::Text("contact".into())));
        assert_eq!(r.get("id"), Some(&Value::Text("a7106d".into())));
        assert_eq!(r.get("switch1"), Some(&Value::Text("OPEN".into())));
        assert_eq!(r.get("switch3"), Some(&Value::Text("CLOSED".into())));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
    }

    #[test]
    fn a_parity_error_is_not_a_sensor() {
        let mut state = [0xe9, 0x15, 0x28];
        state[1] ^= 0x02;
        assert_eq!(
            InterlogixSecurity.decode(&frame([0x46, 0x8d, 0x19], state)),
            Err(DecodeError::NotThisProtocol)
        );
    }

    #[test]
    fn a_reading_is_never_presented_as_verified() {
        let r = InterlogixSecurity.decode(&frame([0x46, 0x8d, 0x19], [0xe9, 0x15, 0x28])).unwrap();
        assert_eq!(r.proof, Proof::None);
    }

    #[test]
    fn an_empty_frame_is_refused_however_its_parity_falls() {
        let empty = frame([0, 0, 0], [0, 0, 0]);
        assert_eq!(InterlogixSecurity.decode(&empty), Err(DecodeError::NotThisProtocol));
    }
}
