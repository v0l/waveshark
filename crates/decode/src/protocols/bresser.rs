//! Bresser Thermo-/Hygro-Sensor 3CH, also sold as Renkforce DM-7511.
//!
//! 433.92 MHz, 40 bits of PWM behind four 750 us sync marks, fifteen repeats
//! every minute.
//!
//! ```text
//! [id] [id] [flags] [temp] [temp] [temp] [humi] [humi] [chk] [chk]
//! ```
//!
//! - `id`    8 bits, redrawn at power up
//! - flags   battery low, test button, then a two bit channel
//! - `temp`  12 bits, degrees Fahrenheit, offset 90, 0.1 F steps
//! - `humi`  8 bits, percent
//! - `chk`   the first four bytes added together
//!
//! The sensor is the only one here that measures in Fahrenheit. It is reported
//! in Celsius anyway, because every other decoder in this crate does and a
//! chart that has to ask which unit a reading is in is not a chart.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::find_frame;
use crate::slicer::Timing;

pub struct Bresser3Ch;

const FRAME_BYTES: usize = 5;

impl Protocol for Bresser3Ch {
    fn name(&self) -> &'static str {
        "Bresser-3CH"
    }

    fn timing(&self) -> Timing {
        Timing::pwm_sync(250, 500, 750, 1250)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let bits = bits.inverted();
        let b = find_frame(&bits, FRAME_BYTES, |b| {
            // Subtractive rather than a comparison so it reads the way the
            // sensor computes it, and rejects the all-zero frame for free.
            b[..4].iter().any(|v| *v != 0)
                && b[0].wrapping_add(b[1]).wrapping_add(b[2]).wrapping_add(b[3]).wrapping_sub(b[4])
                    == 0
        })
        .ok_or(match bits.len() {
            n if n < FRAME_BYTES * 8 => DecodeError::WrongLength { got: n, want: FRAME_BYTES * 8 },
            _ => DecodeError::CrcFailed,
        })?;

        let channel = (b[1] >> 4) & 0x03;
        if channel == 0 {
            return Err(DecodeError::Implausible("channel zero"));
        }
        let fahrenheit = ((((b[1] as u16 & 0x0f) << 8) | b[2] as u16) as f64 - 900.0) * 0.1;
        if !(-20.0..=160.0).contains(&fahrenheit) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }
        let humidity = b[3];
        if humidity > 100 {
            return Err(DecodeError::Implausible("humidity above 100%"));
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        Ok(r.int("id", b[0] as i64)
            .int("channel", channel as i64)
            .float("temperature_c", (((fahrenheit - 32.0) / 1.8) * 10.0).round() / 10.0)
            .int("humidity_pct", humidity as i64)
            .bool("battery_ok", b[1] & 0x80 == 0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn frame(id: u8, channel: u8, temp_f: f64, humidity: u8, battery_low: bool) -> BitBuffer {
        let raw = ((temp_f * 10.0).round() as i32 + 900) as u16;
        let mut b = [0u8; FRAME_BYTES];
        b[0] = id;
        b[1] = (if battery_low { 0x80 } else { 0 }) | (channel << 4) | (raw >> 8) as u8;
        b[2] = raw as u8;
        b[3] = humidity;
        b[4] = b[0].wrapping_add(b[1]).wrapping_add(b[2]).wrapping_add(b[3]);
        // The frame travels inverted.
        BitBuffer::from_bytes(&b).inverted()
    }

    #[test]
    fn decodes_a_bresser_frame_and_converts_to_celsius() {
        // 68.0 F is 20.0 C exactly.
        let r = Bresser3Ch.decode(&frame(0x3d, 2, 68.0, 51, false)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x3d)));
        assert_eq!(r.get("channel"), Some(&Value::Int(2)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(20.0)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(51)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_reading_below_freezing_decodes() {
        // 14.0 F is -10.0 C.
        let r = Bresser3Ch.decode(&frame(0x3d, 1, 14.0, 88, true)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-10.0)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_corrupt_frame_fails_its_checksum() {
        let f = frame(0x3d, 2, 68.0, 51, false);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 30 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert_eq!(Bresser3Ch.decode(&broken), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn channel_zero_is_not_a_channel_any_sensor_has() {
        assert_eq!(
            Bresser3Ch.decode(&frame(0x3d, 0, 68.0, 51, false)),
            Err(DecodeError::Implausible("channel zero"))
        );
    }
}
