//! Acurite 609TXC and the 592TXR "tower" family.
//!
//! Both are 433.92 MHz, both are sold in large numbers, and they share nothing
//! but a manufacturer: the 609 is PPM with an additive checksum, the tower is
//! PWM with a sync mark, inverted bits, a checksum *and* per-byte parity.
//!
//! Frame layouts, transcribed from rtl_433's `acurite.c`.
//!
//! 609TXC, 5 bytes:
//!
//! ```text
//! II ST TT HH CC
//! ```
//!
//! - `II` id, changes at power up
//! - `S`  status, bit 3 is battery low
//! - `TTT` temperature, 12 bit signed, 0.1 C steps
//! - `HH` humidity, percent
//! - `CC` sum of the first four bytes
//!
//! 592TXR tower sensor, 7 bytes, message type 0x04:
//!
//! ```text
//! CCII IIII  IIII IIII  pB00 0100  pHHH HHHH  p??T TTTT  pTTT TTTT  KKKK KKKK
//! ```
//!
//! - `C` channel, 00 is C, 10 is B, 11 is A, 01 is invalid
//! - `I` 14 bit id
//! - `B` battery, 1 is good
//! - `H` humidity, percent
//! - `T` temperature, offset 1000, 0.1 C steps
//! - `K` sum of the preceding six bytes
//! - `p` even parity over bytes 2 to 5

use crate::bits::{checksum8, crc8le, even_parity, lfsr_digest8, BitBuffer};
use crate::protocols::{find_frame, rows_of};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::Timing;

pub struct Acurite609Txc;

const TXC_BYTES: usize = 5;

impl Protocol for Acurite609Txc {
    fn name(&self) -> &'static str {
        "Acurite-609TXC"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(1000, 2000, 10_000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // A byte-wide sum over four bytes is weak enough that a burst of
        // something else passes it every few hundred windows, so the sanity
        // rules carry as much weight as the checksum. A zero id is one: the
        // sensor draws it at random when the batteries go in and never reports
        // zero. Seen on rtl_433's own X10 recording, which this decoder claimed
        // as a sensor reading 14.3 C at 0% humidity.
        let b = find_frame(bits, TXC_BYTES, |b| {
            checksum8(&b[..4]) == b[4] && b[0] != 0 && b[..4] != [0; 4]
        })
        .ok_or(match bits.len() {
            n if n < TXC_BYTES * 8 => DecodeError::WrongLength { got: n, want: TXC_BYTES * 8 },
            _ => DecodeError::CrcFailed,
        })?;

        // Sign extend from 12 bits: the sensor reports below zero as a two's
        // complement value in the low nibble of byte 1 plus byte 2.
        let raw = (((b[1] as u16 & 0x0f) << 8) | b[2] as u16) as i16;
        let temperature = ((raw << 4) >> 4) as f64 * 0.1;
        if !(-40.0..=70.0).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }
        let humidity = b[3];
        if humidity > 100 {
            return Err(DecodeError::Implausible("humidity above 100%"));
        }
        let status = b[1] >> 4;

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        Ok(r
            .int("id", b[0] as i64)
            .float("temperature_c", (temperature * 10.0).round() / 10.0)
            .int("humidity_pct", humidity as i64)
            .bool("battery_ok", status & 0x8 == 0))
    }
}

/// The 592TXR tower sensor, and the 592TX without humidity.
pub struct AcuriteTower;

const TOWER_BYTES: usize = 7;
const MSG_TOWER: u8 = 0x04;

impl Protocol for AcuriteTower {
    fn name(&self) -> &'static str {
        "Acurite-Tower"
    }

    fn timing(&self) -> Timing {
        Timing::pwm_sync(220, 408, 620, 4000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // Documented with the opposite polarity to what the slicer produces,
        // exactly as rtl_433 does with its `bitbuffer_invert`.
        let bits = bits.inverted();
        let b = find_frame(&bits, TOWER_BYTES, |b| {
            b[2] & 0x3f == MSG_TOWER
                && checksum8(&b[..TOWER_BYTES - 1]) == b[TOWER_BYTES - 1]
                // Parity covers the message type, humidity and temperature
                // bytes only; the id bytes and the checksum are full width.
                && even_parity(&b[2..TOWER_BYTES - 1])
        })
        .ok_or(match bits.len() {
            n if n < TOWER_BYTES * 8 => DecodeError::WrongLength { got: n, want: TOWER_BYTES * 8 },
            _ => DecodeError::CrcFailed,
        })?;

        // 01 is not a channel any sensor can be set to, so it means the frame
        // is corrupt in a way the checksum happened not to catch.
        let channel = match b[0] >> 6 {
            0b00 => "C",
            0b10 => "B",
            0b11 => "A",
            _ => return Err(DecodeError::Implausible("invalid channel")),
        };
        let humidity = b[3] & 0x7f;
        if humidity > 100 && humidity != 127 {
            return Err(DecodeError::Implausible("humidity above 100%"));
        }
        let raw = ((b[4] as u16 & 0x7f) << 7) | (b[5] as u16 & 0x7f);
        let temperature = (raw as f64 - 1000.0) * 0.1;
        if !(-40.0..=70.0).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        r = r
            .int("id", (((b[0] as i64) & 0x3f) << 8) | b[1] as i64)
            .text("channel", channel)
            .float("temperature_c", (temperature * 10.0).round() / 10.0)
            .bool("battery_ok", b[2] & 0x40 != 0);
        // 127 is what a 592TX without a humidity sensor sends. Reporting it as
        // 127% would be worse than not reporting it.
        if humidity != 127 {
            r = r.int("humidity_pct", humidity as i64);
        }
        Ok(r)
    }
}

/// The Iris 5-in-1 and Notos 3-in-1 weather stations.
///
/// Same wire format as the tower sensor and one byte longer, with the message
/// type saying which set of readings the frame carries. The 5n1 alternates
/// between two of them, wind with direction and rain, then wind with
/// temperature and humidity, so a full picture takes two transmissions.
///
/// ```text
/// CCSS IIII  IIII IIII  pB11 0001  p??W WWWW  pWWW DDDD  pRRR RRRR  pRRR RRRR  KKKK KKKK
/// CCSS IIII  IIII IIII  pB11 1000  p??W WWWW  pWWW TTTT  pTTT TTTT  pHHH HHHH  KKKK KKKK
/// ```
///
/// - `C` channel, `S` a sequence number, `I` the station id
/// - `W` wind speed, in cup rotations per four seconds
/// - `D` wind direction, an index into a scrambled sixteen point table
/// - `R` rain, counted in hundredths of an inch since the batteries went in
/// - `T` temperature in Fahrenheit, offset by 40
/// - `H` humidity, percent
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
            Ok(r
                .float(
                    "wind_direction_deg",
                    WIND_DIRECTIONS[(b[4] & 0x0f) as usize] as f64 * 22.5,
                )
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
pub struct Acurite606Tx;

const TX606_BYTES: usize = 4;

impl Protocol for Acurite606Tx {
    fn name(&self) -> &'static str {
        "Acurite-606TX"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(2000, 4000, 10_000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let b = find_frame(bits, TX606_BYTES, |b| {
            b[..3] != [0; 3] && lfsr_digest8(&b[..3], 0x98, 0xf1) == b[3]
        })
        .ok_or(match bits.len() {
            n if n < TX606_BYTES * 8 => {
                DecodeError::WrongLength { got: n, want: TX606_BYTES * 8 }
            }
            _ => DecodeError::CrcFailed,
        })?;

        let raw = (((b[1] as u16 & 0x0f) << 8) | b[2] as u16) as i16;
        let temperature = ((raw << 4) >> 4) as f64 * 0.1;
        if !(-40.0..=70.0).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        Ok(r
            .int("id", b[0] as i64)
            .int("channel", ((b[1] >> 4) & 0x03) as i64 + 1)
            .float("temperature_c", round1(temperature))
            .bool("battery_ok", b[1] & 0x80 != 0)
            .bool("button", b[1] & 0x40 != 0))
    }
}

/// The 986 refrigerator and freezer thermometer: two probes, one display, and
/// the only Acurite here that sends its bits least significant first.
///
/// PPM at 520 and 880 us, five bytes once each is reversed:
///
/// ```text
/// TTTT TTTT  IIII IIII  IIII IIII  SSSS SSSN  KKKK KKKK
/// ```
///
/// - `T` temperature in Fahrenheit, sign and magnitude rather than two's
///   complement, which is why -5 F is 0x85
/// - `I` sensor id, `N` which of the two probes sent this, `S` status with the
///   low battery flag in its bottom bit
/// - `K` CRC8 computed least significant bit first
pub struct Acurite986;

const A986_BYTES: usize = 5;

impl Protocol for Acurite986 {
    fn name(&self) -> &'static str {
        "Acurite-986"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(520, 880, 4000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // Only frames the slicer's own row breaks bracket are considered, and
        // only where the row is the length rtl_433 expects. Eight bits of CRC
        // over every offset of a long buffer is not a filter at all: without
        // the row test this decoder claimed bursts from three other sensors in
        // the corpus, all of them reading 0 F.
        let f = rows_of(bits, A986_BYTES * 8, 39..=43)
            .find(|f| {
                if f[..3] == [0; 3] || f[..3] == [0xff; 3] {
                    return false;
                }
                crc8le(&f[..4], 0x07, 0) == f[4]
            })
            .ok_or(match bits.len() {
                n if n < A986_BYTES * 8 => {
                    DecodeError::WrongLength { got: n, want: A986_BYTES * 8 }
                }
                _ => DecodeError::CrcFailed,
            })?;

        let magnitude = (f[0] & 0x7f) as f64;
        let fahrenheit = if f[0] & 0x80 != 0 { -magnitude } else { magnitude };
        // A fridge or a freezer, so the useful range is small and anything
        // outside it is a frame that found the CRC by luck.
        if !(-40.0..=90.0).contains(&fahrenheit) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }
        let probe = f[3] & 0x01;
        let status = f[3] >> 1;

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = f.clone();
        Ok(r
            .int("id", ((f[1] as i64) << 8) | f[2] as i64)
            // The display calls probe one the fridge and probe two the
            // freezer, and labels them that way, so the label is what a user
            // will be looking for.
            .text("channel", if probe == 1 { "2F" } else { "1R" })
            .float("temperature_c", round1((fahrenheit - 32.0) / 1.8))
            .int("status", status as i64)
            .bool("battery_ok", status & 1 == 0))
    }
}

/// The sensor's range in Fahrenheit, refused outside it, converted to Celsius.
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
    use crate::bits::reflect8;
    use crate::protocol::Value;

    fn txc(id: u8, temp_c: f64, humidity: u8, battery_low: bool) -> Vec<u8> {
        let raw = (temp_c * 10.0).round() as i16 & 0x0fff;
        let mut b = vec![0u8; TXC_BYTES];
        b[0] = id;
        b[1] = (if battery_low { 0x80 } else { 0x20 }) | (raw >> 8) as u8;
        b[2] = raw as u8;
        b[3] = humidity;
        b[4] = checksum8(&b[..4]);
        b
    }

    #[test]
    fn decodes_a_609txc_frame() {
        let r = Acurite609Txc.decode(&BitBuffer::from_bytes(&txc(0xb2, 21.7, 48, false))).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0xb2)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.7)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(48)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_609txc_reads_below_zero() {
        let r = Acurite609Txc.decode(&BitBuffer::from_bytes(&txc(0x11, -12.3, 61, true))).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-12.3)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_609txc_frame_is_found_wherever_the_slicer_started() {
        let f = txc(0xb2, 21.7, 48, false);
        let mut b = BitBuffer::new();
        for bit in [true, false, true] {
            b.push(bit);
        }
        for byte in f.iter().chain(f.iter()) {
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        let r = Acurite609Txc.decode(&b).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.7)));
    }

    #[test]
    fn a_corrupt_609txc_frame_fails_its_checksum() {
        let mut f = txc(0xb2, 21.7, 48, false);
        f[3] ^= 0x10;
        assert_eq!(
            Acurite609Txc.decode(&BitBuffer::from_bytes(&f)),
            Err(DecodeError::CrcFailed)
        );
    }

    /// Build a tower frame, in the polarity the decoder sees before inverting.
    fn tower(id: u16, channel: u8, temp_c: f64, humidity: u8, battery_ok: bool) -> Vec<u8> {
        let raw = ((temp_c * 10.0).round() as i32 + 1000) as u16;
        let mut b = vec![0u8; TOWER_BYTES];
        b[0] = (channel << 6) | ((id >> 8) as u8 & 0x3f);
        b[1] = id as u8;
        b[2] = if battery_ok { 0x40 } else { 0x00 } | MSG_TOWER;
        b[3] = humidity & 0x7f;
        b[4] = (raw >> 7) as u8 & 0x7f;
        b[5] = raw as u8 & 0x7f;
        // An odd byte gets its parity bit set, which is where the 0x80 in a
        // real frame comes from.
        for byte in b[2..6].iter_mut() {
            if byte.count_ones() % 2 == 1 {
                *byte |= 0x80;
            }
        }
        b[6] = checksum8(&b[..6]);
        b
    }

    fn tower_bits(frame: &[u8]) -> BitBuffer {
        // The decoder inverts, so the test feeds it inverted bits.
        BitBuffer::from_bytes(frame).inverted()
    }

    #[test]
    fn decodes_a_592txr_tower_frame() {
        let f = tower(0x1234, 0b11, 18.4, 55, true);
        let r = AcuriteTower.decode(&tower_bits(&f)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x1234)));
        assert_eq!(r.get("channel"), Some(&Value::Text("A".into())));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(18.4)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(55)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
    }

    #[test]
    fn a_tower_frame_below_zero_and_on_battery_low() {
        let f = tower(0x0abc, 0b00, -7.5, 92, false);
        let r = AcuriteTower.decode(&tower_bits(&f)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-7.5)));
        assert_eq!(r.get("channel"), Some(&Value::Text("C".into())));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_592tx_without_a_humidity_sensor_reports_no_humidity() {
        let f = tower(0x0abc, 0b10, 20.0, 127, true);
        let r = AcuriteTower.decode(&tower_bits(&f)).unwrap();
        assert!(r.get("humidity_pct").is_none(), "127 is 'no sensor', not 127%");
    }

    #[test]
    fn a_tower_frame_with_bad_parity_is_refused() {
        // Parity is the check that catches the single-bit errors the additive
        // checksum lets through, so it has to be enforced, not just computed.
        let mut f = tower(0x1234, 0b11, 18.4, 55, true);
        f[4] ^= 0x80;
        f[6] = checksum8(&f[..6]);
        assert_eq!(AcuriteTower.decode(&tower_bits(&f)), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn another_manufacturers_message_type_is_not_claimed() {
        let mut f = tower(0x1234, 0b11, 18.4, 55, true);
        f[2] = (f[2] & 0xc0) | 0x31;
        f[6] = checksum8(&f[..6]);
        assert_eq!(AcuriteTower.decode(&tower_bits(&f)), Err(DecodeError::CrcFailed));
    }

    /// Finish a weather station frame: parity bits, then the checksum.
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
        let frames: Vec<Vec<u8>> =
            (0..3).map(|seq| wind_rain(0x347, seq, 4, 0x04, 66)).collect();
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
        let frames: Vec<Vec<u8>> =
            (0..3).map(|seq| wind_rain(0x347, seq, 4, 0x04, 66)).collect();
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

    fn tx606(id: u8, channel: u8, temp_c: f64, battery_ok: bool, button: bool) -> Vec<u8> {
        let raw = ((temp_c * 10.0).round() as i16) & 0x0fff;
        let mut b = vec![0u8; TX606_BYTES];
        b[0] = id;
        b[1] = (if battery_ok { 0x80 } else { 0 })
            | (if button { 0x40 } else { 0 })
            | ((channel - 1) << 4)
            | (raw >> 8) as u8;
        b[2] = raw as u8;
        b[3] = lfsr_digest8(&b[..3], 0x98, 0xf1);
        b
    }

    #[test]
    fn decodes_a_606tx_frame() {
        let f = tx606(163, 1, 10.1, true, false);
        let r = Acurite606Tx.decode(&BitBuffer::from_bytes(&f)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(163)));
        assert_eq!(r.get("channel"), Some(&Value::Int(1)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(10.1)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_606tx_reads_below_zero() {
        let f = tx606(163, 1, -0.3, true, false);
        let r = Acurite606Tx.decode(&BitBuffer::from_bytes(&f)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-0.3)));
    }

    #[test]
    fn a_corrupt_606tx_frame_fails_its_digest() {
        let mut f = tx606(163, 1, 10.1, true, false);
        f[2] ^= 0x04;
        assert_eq!(
            Acurite606Tx.decode(&BitBuffer::from_bytes(&f)),
            Err(DecodeError::CrcFailed)
        );
    }

    /// A 986 transmission: two rows, each 40 bits, sent least significant bit
    /// first, which is what the decoder's row test and its reflection expect.
    fn a986_bits(fahrenheit: i8, id: u16, probe: u8, battery_low: bool) -> BitBuffer {
        let mut f = [0u8; A986_BYTES];
        f[0] = if fahrenheit < 0 { 0x80 | (-fahrenheit) as u8 } else { fahrenheit as u8 };
        f[1] = (id >> 8) as u8;
        f[2] = id as u8;
        f[3] = ((if battery_low { 1 } else { 0 }) << 1) | probe;
        f[4] = crc8le(&f[..4], 0x07, 0);

        let mut b = BitBuffer::new();
        for _ in 0..2 {
            b.mark_row();
            for byte in f {
                let wire = reflect8(byte);
                for i in 0..8 {
                    b.push(wire & (0x80 >> i) != 0);
                }
            }
        }
        b
    }

    #[test]
    fn decodes_a_986_freezer_reading() {
        let r = Acurite986.decode(&a986_bits(-5, 0x10ac, 0, true)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x10ac)));
        assert_eq!(r.get("channel"), Some(&Value::Text("1R".into())));
        // -5 F, which the display shows as -20.6 C.
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-20.6)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_986_frame_outside_a_row_is_not_claimed() {
        // The row length is most of this decoder's integrity check, so a
        // buffer with no row structure must yield nothing at all.
        let bits = a986_bits(51, 0x427d, 0, false);
        let flat = bits.slice(0, bits.len());
        assert_eq!(Acurite986.decode(&flat), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn a_corrupt_986_frame_fails_its_crc() {
        // The same bit in both copies: one good repeat is enough to decode, so
        // corrupting only one would prove nothing.
        let mut b = BitBuffer::new();
        let good = a986_bits(51, 0x427d, 0, false);
        for i in 0..good.len() {
            if good.rows().contains(&i) {
                b.mark_row();
            }
            let flip = i % 40 == 12;
            b.push(if flip { !good.get(i).unwrap() } else { good.get(i).unwrap() });
        }
        assert_eq!(Acurite986.decode(&b), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn a_5n1_frame_with_a_bad_checksum_is_refused() {
        let mut frames: Vec<Vec<u8>> =
            (0..3).map(|seq| wind_rain(0x347, seq, 4, 0x04, 66)).collect();
        for f in frames.iter_mut() {
            f[5] ^= 0x01;
        }
        assert_eq!(AcuriteWind.decode(&wind_bits(&frames)), Err(DecodeError::CrcFailed));
    }
}
