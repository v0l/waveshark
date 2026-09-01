//! LaCrosse sensors: the OOK TX141TH-Bv2 and the FSK "IT+" TX29/TX35.
//!
//! TX141TH-Bv2, also sold as TFA 30.3221.02 and 30.3249.02. Four 833 us sync
//! marks then 40 PWM bits, twelve times per burst:
//!
//! ```text
//! [id] [id] [flags] [temp] [temp] [temp] [humi] [humi] [chk] [chk]
//! ```
//!
//! - `id`    8 bits, redrawn at power up
//! - flags   battery low, test button, then a two bit channel
//! - `temp`  12 bits, offset 500, 0.1 C steps
//! - `humi`  8 bits, percent
//! - `chk`   LFSR digest, gen 0x31, key 0xf4, reflected, over the first four
//!   bytes. Not a CRC, and not computable with one
//!
//! TX29-IT and TX35DTH-IT are the same payload sent as FSK NRZ at 868.24 MHz,
//! 55 us per bit for the TX29 and 105 us for the TX35, behind a `2dd4` sync
//! word:
//!
//! ```text
//! 9 II IIII B U TTTT TTTT TTTT W HHHHHHH CCCCCCCC
//! ```
//!
//! - `9`     model nibble, always 9
//! - `I`     6 bit id
//! - `B`     new battery, `U` unused, `W` weak battery
//! - `T`     temperature in BCD, tenths, offset 40 C
//! - `H`     humidity percent, 0x6a meaning the sensor has none
//! - `C`     CRC-8, polynomial 0x31, init 0x00, over the preceding four bytes

use crate::bits::{crc8, lfsr_digest8_reflect, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::protocols::find_frame;
use crate::slicer::{Coding, Timing};

pub struct LacrosseTx141thBv2;

const TX141TH_BYTES: usize = 5;

impl Protocol for LacrosseTx141thBv2 {
    fn name(&self) -> &'static str {
        "LaCrosse-TX141THBv2"
    }

    fn timing(&self) -> Timing {
        Timing::pwm_sync(208, 417, 833, 1700)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // A long mark is a 1 on the air, which is the opposite of what the
        // slicer calls it, so the whole buffer is inverted first.
        let bits = bits.inverted();
        let b = find_frame(&bits, TX141TH_BYTES, |b| {
            b[0] != 0 && lfsr_digest8_reflect(&b[..4], 0x31, 0xf4) == b[4]
        })
        .ok_or(match bits.len() {
            n if n < TX141TH_BYTES * 8 => {
                DecodeError::WrongLength { got: n, want: TX141TH_BYTES * 8 }
            }
            _ => DecodeError::CrcFailed,
        })?;

        let temperature = ((((b[1] as u16 & 0x0f) << 8) | b[2] as u16) as f64 - 500.0) * 0.1;
        if !(-40.0..=60.0).contains(&temperature) {
            return Err(DecodeError::Implausible("temperature out of range"));
        }
        let humidity = b[3];
        if humidity == 0 || humidity > 100 {
            return Err(DecodeError::Implausible("humidity outside 1-100%"));
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.clone();
        r = r
            .int("id", b[0] as i64)
            .int("channel", ((b[1] >> 4) & 0x03) as i64)
            .float("temperature_c", (temperature * 10.0).round() / 10.0)
            .int("humidity_pct", humidity as i64)
            .bool("battery_ok", b[1] & 0x80 == 0);
        if b[1] & 0x40 != 0 {
            r = r.bool("test", true);
        }
        Ok(r)
    }
}

/// LaCrosse "IT+" over FSK: TX29-IT and TX35DTH-IT.
///
/// One decoder, two instances: the payload is identical and only the bit
/// period differs, so registering both costs a struct field rather than a
/// second parser.
pub struct LacrosseIt {
    name: &'static str,
    bit_us: u32,
}

impl LacrosseIt {
    pub fn tx29() -> Self {
        Self { name: "LaCrosse-TX29IT", bit_us: 55 }
    }

    pub fn tx35() -> Self {
        Self { name: "LaCrosse-TX35DTHIT", bit_us: 105 }
    }
}

/// Preamble plus the `2dd4` sync word and the model nibble, 24 bits.
const IT_SYNC: [u8; 3] = [0xa2, 0xdd, 0x49];
/// The payload starts at the model nibble, four bits before the sync ends.
const IT_SYNC_LEAD: usize = 20;
const IT_BYTES: usize = 5;
/// Humidity value meaning the sensor has no humidity element.
const IT_NO_HUMIDITY: u8 = 0x6a;
/// Humidity value meaning the reading is from the second temperature probe.
const IT_PROBE: u8 = 0x7d;

impl Protocol for LacrosseIt {
    fn name(&self) -> &'static str {
        self.name
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: self.bit_us,
            long_us: self.bit_us,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 4000,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let at = bits.find(&IT_SYNC, 24).ok_or(DecodeError::NotThisProtocol)?;
        let start = at + IT_SYNC_LEAD;
        // A frame whose last bits are zeros ends with the carrier already off,
        // and a detector cannot see how long silence was meant to last: the
        // final gap it reports is its own reset timeout. So the tail is padded
        // with the zeros silence stands for, at most one byte of them, and the
        // CRC still has to hold across the padding. Without this a TX29 frame
        // ending in two zero bits is thrown away, which is what rtl_433's own
        // recording of one does.
        let have = bits.len().saturating_sub(start);
        if have + 8 < IT_BYTES * 8 {
            return Err(DecodeError::WrongLength { got: have, want: IT_BYTES * 8 });
        }
        let frame = bits.slice(start, IT_BYTES * 8);
        let b = frame.as_padded_bytes();
        if crc8(&b[..4], 0x31, 0x00) != b[4] {
            return Err(DecodeError::CrcFailed);
        }

        // Temperature is BCD, which makes a corrupt-but-CRC-valid frame easy
        // to spot: a nibble above 9 cannot have been transmitted.
        let (tens, ones, tenths) = (b[1] & 0x0f, b[2] >> 4, b[2] & 0x0f);
        if tens > 9 || ones > 9 || tenths > 9 {
            return Err(DecodeError::Implausible("temperature is not BCD"));
        }
        let temperature = tens as f64 * 10.0 + ones as f64 + tenths as f64 * 0.1 - 40.0;
        let humidity = b[3] & 0x7f;
        let mut id = ((b[0] as i64 & 0x0f) << 2) | (b[1] >> 6) as i64;
        if humidity == IT_PROBE {
            // The probe channel is reported as its own sensor rather than as a
            // second reading from this one, matching rtl_433.
            id += 0x40;
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.to_vec();
        r = r
            .int("id", id)
            .float("temperature_c", (temperature * 10.0).round() / 10.0)
            .bool("battery_ok", b[3] & 0x80 == 0)
            .bool("battery_new", b[1] >> 5 & 1 != 0);
        if humidity != IT_NO_HUMIDITY && humidity != IT_PROBE {
            if humidity > 100 {
                return Err(DecodeError::Implausible("humidity above 100%"));
            }
            r = r.int("humidity_pct", humidity as i64);
        }
        Ok(r)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    fn tx141th(id: u8, channel: u8, temp_c: f64, humidity: u8, battery_low: bool) -> Vec<u8> {
        let raw = ((temp_c * 10.0).round() as i32 + 500) as u16;
        let mut b = vec![0u8; TX141TH_BYTES];
        b[0] = id;
        b[1] = (if battery_low { 0x80 } else { 0 }) | ((channel & 0x03) << 4) | (raw >> 8) as u8;
        b[2] = raw as u8;
        b[3] = humidity;
        b[4] = lfsr_digest8_reflect(&b[..4], 0x31, 0xf4);
        b
    }

    /// The decoder inverts, so a test frame goes in inverted.
    fn on_air(frame: &[u8]) -> BitBuffer {
        BitBuffer::from_bytes(frame).inverted()
    }

    #[test]
    fn decodes_a_tx141th_frame() {
        let r = LacrosseTx141thBv2.decode(&on_air(&tx141th(0x9c, 1, 23.6, 44, false))).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x9c)));
        assert_eq!(r.get("channel"), Some(&Value::Int(1)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(23.6)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(44)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_tx141th_frame_below_zero_decodes() {
        let r = LacrosseTx141thBv2.decode(&on_air(&tx141th(0x9c, 0, -15.2, 88, true))).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-15.2)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn a_tx141th_frame_is_found_among_its_repeats() {
        // A burst holds twelve copies and the slicer starts mid-frame.
        let f = tx141th(0x9c, 1, 23.6, 44, false);
        let mut b = BitBuffer::new();
        for bit in [true, false, false, true, true, true, false] {
            b.push(bit);
        }
        for byte in f.iter().chain(f.iter()).chain(f.iter()) {
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        let r = LacrosseTx141thBv2.decode(&b.inverted()).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(23.6)));
    }

    #[test]
    fn a_corrupt_tx141th_frame_fails_its_digest() {
        let mut f = tx141th(0x9c, 1, 23.6, 44, false);
        f[2] ^= 0x08;
        assert_eq!(
            LacrosseTx141thBv2.decode(&on_air(&f)),
            Err(DecodeError::CrcFailed)
        );
    }

    /// An IT+ frame: preamble, sync word, then the five payload bytes.
    fn it_frame(id: u8, temp_c: f64, humidity: u8, battery_low: bool, new: bool) -> BitBuffer {
        let t = ((temp_c + 40.0) * 10.0).round() as u32;
        let (tens, ones, tenths) = (t / 100, (t / 10) % 10, t % 10);
        let mut b = [0u8; IT_BYTES];
        b[0] = 0x90 | ((id >> 2) & 0x0f);
        b[1] = ((id & 0x03) << 6) | (if new { 0x20 } else { 0 }) | tens as u8;
        b[2] = (ones as u8) << 4 | tenths as u8;
        b[3] = (if battery_low { 0x80 } else { 0 }) | (humidity & 0x7f);
        b[4] = crc8(&b[..4], 0x31, 0x00);

        let mut out = BitBuffer::new();
        // Long preamble, as most of these sensors send.
        for _ in 0..8 {
            out.push(true);
            out.push(false);
        }
        for byte in [0x2d, 0xd4] {
            for i in 0..8 {
                out.push(byte & (0x80 >> i) != 0);
            }
        }
        for byte in b {
            for i in 0..8 {
                out.push(byte & (0x80 >> i) != 0);
            }
        }
        out
    }

    #[test]
    fn decodes_a_tx29_frame_behind_its_sync_word() {
        let r = LacrosseIt::tx29().decode(&it_frame(0x25, 21.3, 57, false, false)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x25)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(21.3)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(57)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn tx29_and_tx35_differ_only_in_bit_period() {
        let f = it_frame(0x25, 21.3, 57, false, false);
        let a = LacrosseIt::tx29().decode(&f).unwrap();
        let b = LacrosseIt::tx35().decode(&f).unwrap();
        assert_eq!(a.fields, b.fields);
        assert_eq!(LacrosseIt::tx29().timing().short_us, 55);
        assert_eq!(LacrosseIt::tx35().timing().short_us, 105);
    }

    #[test]
    fn a_temperature_only_sensor_reports_no_humidity() {
        let r = LacrosseIt::tx29()
            .decode(&it_frame(0x25, -4.5, IT_NO_HUMIDITY, true, true))
            .unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-4.5)));
        assert!(r.get("humidity_pct").is_none());
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
        assert_eq!(r.get("battery_new"), Some(&Value::Bool(true)));
    }

    #[test]
    fn the_second_probe_channel_gets_its_own_id() {
        let r = LacrosseIt::tx29().decode(&it_frame(0x25, 45.0, IT_PROBE, false, false)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(0x25 + 0x40)));
        assert!(r.get("humidity_pct").is_none());
    }

    #[test]
    fn a_corrupt_it_frame_fails_its_crc() {
        let f = it_frame(0x25, 21.3, 57, false, false);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 40 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert_eq!(LacrosseIt::tx29().decode(&broken), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn a_burst_without_the_sync_word_is_not_this_protocol() {
        let b = BitBuffer::from_bytes(&[0x55; 12]);
        assert_eq!(LacrosseIt::tx29().decode(&b), Err(DecodeError::NotThisProtocol));
    }
}
