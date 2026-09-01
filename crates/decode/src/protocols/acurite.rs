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

use crate::bits::{checksum8, even_parity, BitBuffer};
use crate::protocols::find_frame;
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

#[cfg(test)]
mod tests {
    use super::*;
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
}
