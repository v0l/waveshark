//! Acurite 5-in-1 and 3-in-1 weather stations: the one frame of the family
//! a description cannot say, because wind speed is zero for no rotations and
//! a linear function of them otherwise. The tower, 609TXC, 606TX and 986
//! are descriptions under `crates/decode/protocols/weather`.
//!
//! Frame layout transcribed from rtl_433's `acurite.c`: eight bytes, PWM
//! behind a sync mark, inverted, with an additive checksum and even parity
//! over the bytes between the id and the sum.

use crate::bits::{BitBuffer, checksum8, even_parity};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::find_frame;
use crate::slicer::Timing;

pub struct AcuriteWind;

const WIND_BYTES: usize = 8;
const MSG_5N1_WIND_RAIN: u8 = 0x31;
const MSG_5N1_WIND_TH: u8 = 0x38;
const MSG_3N1_WIND_TH: u8 = 0x20;

/// The direction each index means, in sixteenths of a turn. The sensor's own
/// order, which is neither clockwise nor Gray coded: it is the order the vane
/// switches happen to be wired in.
const WIND_DIRECTIONS: [u8; 16] = [14, 11, 13, 12, 15, 10, 0, 9, 3, 6, 4, 5, 2, 7, 1, 8];

impl Protocol for AcuriteWind {
    fn name(&self) -> &'static str {
        "Acurite-5n1"
    }

    fn timing(&self) -> Timing {
        Timing::pwm_sync(220, 408, 620, 4000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let bits = bits.inverted();
        let b = find_frame(&bits, WIND_BYTES, |b| {
            matches!(b[2] & 0x3f, MSG_5N1_WIND_RAIN | MSG_5N1_WIND_TH | MSG_3N1_WIND_TH)
                && checksum8(&b[..WIND_BYTES - 1]) == b[WIND_BYTES - 1]
                && even_parity(&b[2..WIND_BYTES - 1])
        })
        .ok_or(match bits.len() {
            n if n < WIND_BYTES * 8 => DecodeError::WrongLength { got: n, want: WIND_BYTES * 8 },
            _ => DecodeError::CrcFailed,
        })?;

        let channel = match b[0] >> 6 {
            0b00 => "C",
            0b10 => "B",
            0b11 => "A",
            _ => return Err(DecodeError::Implausible("invalid channel")),
        };
        let message_type = b[2] & 0x3f;
        let three_in_one = message_type == MSG_3N1_WIND_TH;
        let model = if three_in_one { "Acurite-3n1" } else { "Acurite-5n1" };
        // The 3n1 spends two more bits of byte zero on the id, where the 5n1
        // keeps them for the sequence number. rtl_433 reads the sequence
        // number from both anyway, so the 3n1's is two bits of its own id.
        let id = if three_in_one {
            ((b[0] as i64 & 0x3f) << 8) | b[1] as i64
        } else {
            ((b[0] as i64 & 0x0f) << 8) | b[1] as i64
        };

        let mut r = Report::new(model);
        r.crc_valid = Some(true);
        r.raw = b.clone();
        r = r
            .int("message_type", message_type as i64)
            .int("id", id)
            .text("channel", channel)
            .int("sequence_num", ((b[0] >> 4) & 0x03) as i64)
            .bool("battery_ok", b[2] & 0x40 != 0);

        if three_in_one {
            // The 3n1 sends wind in whole miles an hour and puts one more bit
            // into the temperature, with a different offset again.
            let raw = ((b[4] as i32 & 0x1f) << 7) | (b[5] as i32 & 0x7f);
            let temperature = fahrenheit_to_c((raw - 1480) as f64 * 0.1)?;
            let humidity = b[3] & 0x7f;
            if humidity > 100 {
                return Err(DecodeError::Implausible("humidity above 100%"));
            }
            return Ok(r
                .float("wind_avg_ms", round2((b[6] & 0x7f) as f64 * 0.44704))
                .float("temperature_c", round1(temperature))
                .int("humidity_pct", humidity as i64));
        }

        // Cup rotations per four seconds, with a fixed offset that only
        // applies once the cups are turning at all.
        let rotations = ((b[3] as i32 & 0x1f) << 3) | ((b[4] as i32 & 0x70) >> 4);
        let wind_kmh = if rotations > 0 { rotations as f64 * 0.8278 + 1.0 } else { 0.0 };
        r = r.float("wind_avg_ms", round2(wind_kmh / 3.6));

        if message_type == MSG_5N1_WIND_RAIN {
            let rain = ((b[5] as i32 & 0x7f) << 7) | (b[6] as i32 & 0x7f);
            Ok(r.float("wind_direction_deg", WIND_DIRECTIONS[(b[4] & 0x0f) as usize] as f64 * 22.5)
                .float("rain_total_mm", round2(rain as f64 * 0.254)))
        } else {
            let raw = ((b[4] as i32 & 0x0f) << 7) | (b[5] as i32 & 0x7f);
            let temperature = fahrenheit_to_c((raw - 400) as f64 * 0.1)?;
            let humidity = b[6] & 0x7f;
            if humidity > 100 {
                return Err(DecodeError::Implausible("humidity above 100%"));
            }
            Ok(r.float("temperature_c", round1(temperature)).int("humidity_pct", humidity as i64))
        }
    }
}

/// The 606TX, sold as the Technoline TX960 as well: temperature only, and the
/// cheapest thing Acurite make.
///
/// PPM at 2 and 4 ms, four bytes, and an LFSR digest rather than the sum its
/// siblings use:
///
/// ```text
/// IIII IIII  BUCC TTTT  TTTT TTTT  KKKK KKKK
/// ```
///
/// - `I` id, redrawn at random whenever the batteries come out
/// - `B` battery good, `U` the pairing button, `C` channel
/// - `T` temperature, 12 bit signed, tenths of a degree
/// - `K` LFSR digest over the first three bytes
fn fahrenheit_to_c(f: f64) -> Result<f64, DecodeError> {
    if !(-40.0..=158.0).contains(&f) {
        return Err(DecodeError::Implausible("temperature out of range"));
    }
    Ok((f - 32.0) / 1.8)
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}

fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn seal(b: &mut [u8]) {
        for byte in b[2..WIND_BYTES - 1].iter_mut() {
            if byte.count_ones() % 2 == 1 {
                *byte |= 0x80;
            }
        }
        b[WIND_BYTES - 1] = checksum8(&b[..WIND_BYTES - 1]);
    }

    fn wind_rain(id: u16, sequence: u8, rotations: u16, direction: u8, rain: u16) -> Vec<u8> {
        let mut b = vec![0u8; WIND_BYTES];
        b[0] = 0xc0 | (sequence << 4) | ((id >> 8) as u8 & 0x0f);
        b[1] = id as u8;
        b[2] = 0x40 | MSG_5N1_WIND_RAIN;
        b[3] = (rotations >> 3) as u8 & 0x1f;
        b[4] = ((rotations as u8 & 0x07) << 4) | (direction & 0x0f);
        b[5] = (rain >> 7) as u8 & 0x7f;
        b[6] = rain as u8 & 0x7f;
        seal(&mut b);
        b
    }

    /// A burst of three, numbered as the station numbers them, with the row
    /// marks its sync gaps leave behind.
    fn wind_bits(frames: &[Vec<u8>]) -> BitBuffer {
        let mut b = BitBuffer::new();
        for f in frames {
            b.mark_row();
            for byte in f {
                for i in 0..8 {
                    b.push(byte & (0x80 >> i) != 0);
                }
            }
        }
        b.inverted()
    }

    #[test]
    fn decodes_a_5n1_wind_and_rain_frame() {
        let frames: Vec<Vec<u8>> = (0..3).map(|seq| wind_rain(0x347, seq, 4, 0x04, 66)).collect();
        let r = AcuriteWind.decode(&wind_bits(&frames)).unwrap();
        assert_eq!(r.model, "Acurite-5n1");
        assert_eq!(r.get("id"), Some(&Value::Int(0x347)));
        assert_eq!(r.get("channel"), Some(&Value::Text("A".into())));
        // Four cup rotations in four seconds, which is 4.3 km/h.
        assert_eq!(r.get("wind_avg_ms"), Some(&Value::Float(1.2)));
        assert_eq!(r.get("wind_direction_deg"), Some(&Value::Float(337.5)));
        assert_eq!(r.get("rain_total_mm"), Some(&Value::Float(16.76)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_burst_of_numbered_repeats_still_corroborates() {
        // No two copies are identical, because each carries its own sequence
        // number, so the only evidence left is that the rows repeat at the
        // frame's own period.
        let frames: Vec<Vec<u8>> = (0..3).map(|seq| wind_rain(0x347, seq, 4, 0x04, 66)).collect();
        let bits = wind_bits(&frames);
        assert_eq!(AcuriteWind.decode(&bits).unwrap().get("sequence_num"), Some(&Value::Int(0)));
    }

    #[test]
    fn decodes_a_3n1_frame() {
        let temp_raw = 1480 + 302; // 30.2 F, as the 3n1 encodes it
        let mut b = vec![0u8; WIND_BYTES];
        b[0] = 0xc0 | 0x1f;
        b[1] = 0x38;
        b[2] = 0x40 | MSG_3N1_WIND_TH;
        b[3] = 43; // humidity
        b[4] = (temp_raw >> 7) as u8 & 0x1f;
        b[5] = temp_raw as u8 & 0x7f;
        b[6] = 5; // miles an hour
        seal(&mut b);
        let r = AcuriteWind.decode(&wind_bits(&[b.clone(), b])).unwrap();
        assert_eq!(r.model, "Acurite-3n1");
        assert_eq!(r.get("id"), Some(&Value::Int(7992)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(43)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-1.0)));
        assert_eq!(r.get("wind_avg_ms"), Some(&Value::Float(2.24)));
    }
}
