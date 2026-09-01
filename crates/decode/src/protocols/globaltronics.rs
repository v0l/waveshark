//! Globaltronics GT-WT-02 and GT-WT-03, the thermo-hygrometers sold with Aldi
//! and Lidl weather stations across Europe.
//!
//! Same manufacturer, unrelated frames. The 02 is PPM with millisecond symbols
//! and a nibble-sum checksum; the 03 is PWM with a sync mark, inverted, and a
//! rolling-key checksum that is neither a CRC nor a sum.
//!
//! GT-WT-02, 37 bits:
//!
//! ```text
//! IIIIIIII BMCCTTTT TTTTTTTT HHHHHHHX XXXXX
//! ```
//!
//! GT-WT-03, 41 bits, the last a stop bit:
//!
//! ```text
//! IIIIIIII HHHHHHHH BMCCTTTT TTTTTTTT XXXXXXXX 1
//! ```
//!
//! - `I` id, redrawn when the batteries are changed
//! - `B` battery low, `M` manual send button
//! - `C` channel, 0 to 2
//! - `T` temperature, 12 bit two's complement, 0.1 C steps
//! - `H` humidity percent, with 10 and 110 as the "LL" and "HH" sentinels the
//!   display shows outside the sensor's working range
//! - `X` checksum

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::find_frame_bits;
use crate::slicer::Timing;

pub struct GtWt02;

const WT02_BITS: usize = 37;

impl Protocol for GtWt02 {
    fn name(&self) -> &'static str {
        "GT-WT02"
    }

    fn timing(&self) -> Timing {
        // Millisecond symbols: slow even by 433 standards, and the reason this
        // one needs a 12 ms reset where most protocols want two or three.
        Timing::ppm(2500, 5000, 12_000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let b = find_frame_bits(bits, WT02_BITS, |b| {
            // rtl_433 refuses an all-zero frame; a zero id is refused here as
            // well, because a six bit checksum passes on one window in
            // sixty-four and this decoder gets offered far more windows than
            // rtl_433's does. Observed claiming a Schrader tyre sensor's burst
            // as a sensor with no id reading 0.1 C.
            b[0] != 0 && nibble_sum(b) == ((b[3] & 1) << 5) + (b[4] >> 3)
        })
        .ok_or(match bits.len() {
            n if n < WT02_BITS => DecodeError::WrongLength { got: n, want: WT02_BITS },
            _ => DecodeError::CrcFailed,
        })?;

        let channel = (b[1] >> 4) & 0x03;
        if channel > 2 {
            return Err(DecodeError::Implausible("invalid channel"));
        }
        let temperature = signed12(b[1], b[2]);
        if !(-20.0..=60.0).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }
        let humidity = humidity_pct(b[3] >> 1, 20..=90)?;

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        r = r
            .int("id", b[0] as i64)
            .int("channel", channel as i64 + 1)
            .float("temperature_c", temperature)
            .int("humidity_pct", humidity as i64)
            .bool("battery_ok", b[1] & 0x80 == 0);
        if b[1] & 0x40 != 0 {
            r = r.bool("button", true);
        }
        Ok(r)
    }
}

/// Eight nibbles added together, modulo 64. The last nibble is only three bits
/// wide because the fourth is the top bit of the checksum itself.
fn nibble_sum(b: &[u8]) -> u8 {
    let s: u16 = [b[0] >> 4, b[0] & 0x0f, b[1] >> 4, b[1] & 0x0f, b[2] >> 4, b[2] & 0x0f, b[3] >> 4]
        .iter()
        .map(|v| *v as u16)
        .sum::<u16>()
        + (b[3] & 0x0e) as u16;
    (s & 0x3f) as u8
}

pub struct GtWt03;

/// Forty data bits plus the stop bit, which has to be counted: the frame
/// length is also the spacing between repeats.
const WT03_BITS: usize = 41;

impl Protocol for GtWt03 {
    fn name(&self) -> &'static str {
        "GT-WT03"
    }

    fn timing(&self) -> Timing {
        Timing::pwm_sync(256, 625, 855, 3000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let bits = bits.inverted();
        let b = find_frame_bits(&bits, WT03_BITS, |b| {
            b[..5].iter().any(|v| *v != 0) && roll_byte(&b[..4], 0x3100) ^ b[4] ^ 0x2d == 0
        })
        .ok_or(match bits.len() {
            n if n < WT03_BITS => DecodeError::WrongLength { got: n, want: WT03_BITS },
            _ => DecodeError::CrcFailed,
        })?;

        let channel = (b[2] >> 4) & 0x03;
        let temperature = signed12(b[2], b[3]);
        // -50.1 and 70.1 are the sensor's own out-of-range markers, so the
        // window has to be a shade wider than its specified range.
        if !(-50.1..=70.1).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }
        let humidity = humidity_pct(b[1], 20..=95)?;

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        r = r
            .int("id", b[0] as i64)
            .int("channel", channel as i64 + 1)
            .float("temperature_c", temperature)
            .int("humidity_pct", humidity as i64)
            .bool("battery_ok", b[2] & 0x80 == 0);
        if b[2] & 0x40 != 0 {
            r = r.bool("button", true);
        }
        Ok(r)
    }
}

/// Per byte, XOR a key into the sum for every set bit, the key rolling right
/// from `gen` as the bits are walked MSB first and resetting at each byte.
///
/// The low byte of a Galois LFSR-16 seeded per byte, in other words, which is
/// why neither a CRC nor a sum reproduces it.
fn roll_byte(data: &[u8], gen: u16) -> u8 {
    let mut sum = 0u8;
    for &byte in data {
        let mut key = gen;
        for i in (0..8).rev() {
            if byte >> i & 1 != 0 {
                sum ^= key as u8;
            }
            key >>= 1;
        }
    }
    sum
}

/// A 12 bit two's complement temperature split across two bytes, in tenths.
fn signed12(hi: u8, lo: u8) -> f64 {
    let raw = (((hi as u16 & 0x0f) << 8) | lo as u16) as i16;
    (((raw << 4) >> 4) as f64) / 10.0
}

/// Humidity, with the display's out-of-range sentinels mapped to the ends of
/// the scale and anything else outside the working range refused.
fn humidity_pct(raw: u8, working: std::ops::RangeInclusive<u8>) -> Result<u8, DecodeError> {
    match raw {
        10 => Ok(0),
        110 => Ok(100),
        v if working.contains(&v) => Ok(v),
        _ => Err(DecodeError::Implausible("humidity outside the sensor's range")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn bits_of(b: &[u8], n: usize) -> BitBuffer {
        let mut out = BitBuffer::new();
        for i in 0..n {
            out.push(b[i / 8] & (0x80 >> (i % 8)) != 0);
        }
        out
    }

    fn wt02(id: u8, channel: u8, temp_c: f64, humidity: u8, battery_low: bool) -> BitBuffer {
        let raw = ((temp_c * 10.0).round() as i16 & 0x0fff) as u16;
        let mut b = [0u8; 5];
        b[0] = id;
        b[1] = (if battery_low { 0x80 } else { 0 }) | (channel << 4) | (raw >> 8) as u8;
        b[2] = raw as u8;
        b[3] = humidity << 1;
        let sum = nibble_sum(&b);
        b[3] |= sum >> 5;
        b[4] = (sum & 0x1f) << 3;
        bits_of(&b, WT02_BITS)
    }

    #[test]
    fn decodes_a_gt_wt_02_frame() {
        let r = GtWt02.decode(&wt02(0x34, 0, 23.7, 35, false)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x34)));
        assert_eq!(r.get("channel"), Some(&Value::Int(1)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(23.7)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(35)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_gt_wt_02_frame_below_zero_decodes() {
        let r = GtWt02.decode(&wt02(0x34, 1, -12.1, 40, true)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-12.1)));
        assert_eq!(r.get("channel"), Some(&Value::Int(2)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn the_display_sentinels_become_the_ends_of_the_scale() {
        // "LL" and "HH" are what the sensor shows outside its working range,
        // and it transmits them as 10 and 110. Reporting 110% would be worse
        // than reporting saturation.
        assert_eq!(
            GtWt02.decode(&wt02(0x34, 0, 23.7, 110, false)).unwrap().get("humidity_pct"),
            Some(&Value::Int(100))
        );
        assert_eq!(
            GtWt02.decode(&wt02(0x34, 0, 23.7, 10, false)).unwrap().get("humidity_pct"),
            Some(&Value::Int(0))
        );
    }

    #[test]
    fn a_corrupt_gt_wt_02_frame_fails_its_checksum() {
        let f = wt02(0x34, 0, 23.7, 35, false);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 18 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert_eq!(GtWt02.decode(&broken), Err(DecodeError::CrcFailed));
    }

    fn wt03(id: u8, channel: u8, temp_c: f64, humidity: u8, battery_low: bool) -> BitBuffer {
        let raw = ((temp_c * 10.0).round() as i16 & 0x0fff) as u16;
        let mut b = [0u8; 6];
        b[0] = id;
        b[1] = humidity;
        b[2] = (if battery_low { 0x80 } else { 0 }) | (channel << 4) | (raw >> 8) as u8;
        b[3] = raw as u8;
        b[4] = roll_byte(&b[..4], 0x3100) ^ 0x2d;
        b[5] = 0x80; // the stop bit
        bits_of(&b, WT03_BITS).inverted()
    }

    #[test]
    fn decodes_a_gt_wt_03_frame() {
        let r = GtWt03.decode(&wt03(0x17, 0, 26.1, 48, false)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x17)));
        assert_eq!(r.get("channel"), Some(&Value::Int(1)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(26.1)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(48)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_gt_wt_03_frame_below_zero_decodes() {
        let r = GtWt03.decode(&wt03(0x01, 2, -4.4, 55, true)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-4.4)));
        assert_eq!(r.get("channel"), Some(&Value::Int(3)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_corrupt_gt_wt_03_frame_fails_its_checksum() {
        let f = wt03(0x17, 0, 26.1, 48, false);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 12 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert_eq!(GtWt03.decode(&broken), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn the_rolling_checksum_is_not_a_sum_or_a_crc() {
        // Worth pinning: it looks like both and is neither, so a future
        // refactor reaching for crc8 here would break it silently.
        assert_eq!(roll_byte(&[0x00, 0x00, 0x00, 0x00], 0x3100), 0x00);
        assert_eq!(roll_byte(&[0x80, 0x00, 0x00, 0x00], 0x3100), 0x00);
        assert_eq!(roll_byte(&[0x01, 0x00, 0x00, 0x00], 0x3100), 0x62);
        assert_eq!(roll_byte(&[0xff, 0x00, 0x00, 0x00], 0x3100), 0x62 ^ 0xc4 ^ 0x88 ^ 0x10 ^ 0x20 ^ 0x40 ^ 0x80);
    }
}
