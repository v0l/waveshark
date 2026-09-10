//! Nexus and the many rebadges of it: FreeTec NC-7345, infactory NX-3980,
//! Solight TE82S, TFA 30.3209.02.
//!
//! 36 bits, PPM, 433.92 MHz, sent twelve times per transmission.
//!
//! ```text
//! [id0] [id1] [flags] [temp0] [temp1] [temp2] [const] [humi0] [humi1]
//! ```
//!
//! - `id`     8 bits, changes when the batteries are replaced
//! - `flags`  `B T C C`: battery ok, test mode, then a two bit channel
//! - `temp`   12 bit signed, 0.1 C steps
//! - `const`  always 1111
//! - `humi`   8 bits, percent
//!
//! There is no checksum, only that constant nibble, so this decoder reports
//! `crc_valid: None` and leans on rtl_433's sanity rules to keep the false
//! positive rate down. On a busy 433 band it will still occasionally claim a
//! burst that belongs to something else, which is the nature of a protocol
//! with four check bits.

use crate::bits::{crc8, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::Timing;

pub struct NexusTh;

const FRAME_BITS: usize = 36;

impl Protocol for NexusTh {
    fn name(&self) -> &'static str {
        "Nexus-TH"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(1000, 2000, 5000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        if bits.len() < FRAME_BITS {
            return Err(DecodeError::WrongLength { got: bits.len(), want: FRAME_BITS });
        }
        // A protocol with no checksum needs corroboration from somewhere. A
        // buffer holding exactly one frame is that: the burst began and ended
        // where a Nexus frame does. Otherwise the frame has to appear twice,
        // which is what rtl_433 requires of it.
        let exact = bits.len() <= FRAME_BITS + 1;
        let mut found = None;
        for start in 0..=(bits.len() - FRAME_BITS) {
            let frame = bits.slice(start, FRAME_BITS);
            let b = frame.as_padded_bytes();
            if !plausible(b) {
                continue;
            }
            let repeated = start + 2 * FRAME_BITS <= bits.len()
                && bits.slice(start + FRAME_BITS, FRAME_BITS) == frame;
            if exact || repeated {
                found = Some(b.to_vec());
                break;
            }
        }
        let b = found.ok_or(DecodeError::NotThisProtocol)?;

        let temp_raw = (((b[1] as u16 & 0x0f) << 8) | b[2] as u16) as i16;
        let temperature = ((temp_raw << 4) >> 4) as f64 * 0.1;
        if !(-40.0..=70.0).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }
        let humidity = ((b[3] & 0x0f) << 4) | (b[4] >> 4);
        if humidity > 100 {
            return Err(DecodeError::Implausible("humidity above 100%"));
        }

        let mut r = Report::new(self.name());
        // Four constant bits are not an integrity check and must not be
        // presented as one.
        r.crc_valid = None;
        r.raw = b.clone();
        r = r
            .int("id", b[0] as i64)
            .int("channel", ((b[1] >> 4) & 0x03) as i64 + 1)
            .float("temperature_c", (temperature * 10.0).round() / 10.0)
            .bool("battery_ok", b[1] & 0x80 != 0);
        if b[1] & 0x40 != 0 {
            r = r.bool("test", true);
        }
        // Zero means the sensor has no humidity element, not zero percent.
        if humidity != 0 {
            r = r.int("humidity_pct", humidity as i64);
        }
        Ok(r)
    }
}

/// rtl_433's sanity rules, which are all the protection this frame layout has.
fn plausible(b: &[u8]) -> bool {
    if b[3] & 0xf0 != 0xf0 {
        return false; // the constant nibble
    }
    if (b[0] == 0 && b[2] == 0 && b[3] == 0) || (b[0] == 0xff && b[2] == 0xff && b[3] == 0xff) {
        return false;
    }
    if b[1] & 0x30 == 0x30 {
        return false; // channel outside 1-3
    }
    // The Rubicson/Solight-TE44/EMOS family has an all but identical layout
    // whose last byte is a real CRC rather than humidity. A frame satisfying
    // that CRC is theirs, not ours, so hand it over rather than claim it.
    let crc_in = [b[0], b[1], b[2], b[3] & 0xf0, ((b[3] & 0x0f) << 4) | (b[4] >> 4)];
    crc8(&crc_in, 0x31, 0x6c) != 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn frame(id: u8, channel: u8, temp_c: f64, humidity: u8, battery_ok: bool) -> BitBuffer {
        let raw = ((temp_c * 10.0).round() as i16 & 0x0fff) as u16;
        let mut b = BitBuffer::new();
        let mut push = |v: u32, n: usize| {
            for i in (0..n).rev() {
                b.push(v >> i & 1 != 0);
            }
        };
        push(id as u32, 8);
        push(battery_ok as u32, 1);
        push(0, 1);
        push(channel as u32 - 1, 2);
        push(raw as u32, 12);
        push(0x0f, 4);
        push(humidity as u32, 8);
        b
    }

    #[test]
    fn decodes_a_nexus_frame() {
        let r = NexusTh.decode(&frame(0x5c, 2, 19.4, 62, true)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x5c)));
        assert_eq!(r.get("channel"), Some(&Value::Int(2)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(19.4)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(62)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
    }

    #[test]
    fn a_nexus_decode_is_never_presented_as_verified() {
        // Four constant bits are not a CRC. A map or a chart downstream has to
        // be able to tell the difference.
        let r = NexusTh.decode(&frame(0x5c, 1, 19.4, 62, true)).unwrap();
        assert_eq!(r.crc_valid, None);
    }

    #[test]
    fn temperatures_below_zero_survive_the_sign_extension() {
        let r = NexusTh.decode(&frame(0x5c, 3, -8.7, 55, false)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-8.7)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_sensor_without_a_humidity_element_reports_none() {
        let r = NexusTh.decode(&frame(0x5c, 1, 19.4, 0, true)).unwrap();
        assert!(r.get("humidity_pct").is_none());
    }

    #[test]
    fn a_missing_constant_nibble_is_not_this_protocol() {
        let f = frame(0x5c, 1, 19.4, 62, true);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            // Clear one bit of the constant nibble.
            broken.push(if i == 24 { false } else { f.get(i).unwrap() });
        }
        assert_eq!(NexusTh.decode(&broken), Err(DecodeError::NotThisProtocol));
    }

    #[test]
    fn an_unrepeated_frame_inside_a_longer_burst_is_not_claimed() {
        // With no checksum, a 36 bit window that happens to look right inside
        // a long burst from something else is a phantom sensor. Corroboration
        // is either an exactly frame-sized burst or a repeat.
        let f = frame(0x5c, 1, 19.4, 62, true);
        let mut long = BitBuffer::new();
        for _ in 0..20 {
            long.push(false);
        }
        for i in 0..f.len() {
            long.push(f.get(i).unwrap());
        }
        for _ in 0..20 {
            long.push(true);
        }
        assert_eq!(NexusTh.decode(&long), Err(DecodeError::NotThisProtocol));

        // The same frame twice is corroboration, and decodes.
        let mut twice = BitBuffer::new();
        for _ in 0..2 {
            for i in 0..f.len() {
                twice.push(f.get(i).unwrap());
            }
        }
        assert!(NexusTh.decode(&twice).is_ok());
    }
}
