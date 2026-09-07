//! Fine Offset WH1080/WH3080 weather station, OOK variant.
//!
//! Also sold as Watson W-8681, Digitech XC0348, PCE-FWS 20, Elecsa AstroTouch
//! 6975 and Froggit WH1080. Transmits every 48 seconds on 433.92, 868.3 or
//! 915 MHz depending on region.
//!
//! Frame layout, 11 bytes, MSB first:
//!
//! ```text
//! ff FI IT TT HH SS GG ?R RR BD CC
//! ```
//!
//! - `ff` preamble, eight 1 bits
//! - `F`  4-bit message format: 0xa weather, 0xb datetime, 0x7 UV/light
//! - `I`  8-bit device id
//! - `T`  temperature, offset 400, 0.1 C steps (only the low 10 bits are used)
//! - `H`  humidity, percent
//! - `S`  wind speed, 0.34 m/s steps
//! - `G`  gust speed, 0.34 m/s steps
//! - `R`  12-bit rain counter, 0.3 mm steps
//! - `B`  4-bit flags, bit 0 is battery low
//! - `D`  4-bit wind direction index
//! - `CC` CRC-8, polynomial 0x31, init 0xff, over all 11 bytes

use crate::bits::{checksum8, crc8, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing};

/// Wind direction index to degrees, 22.5 degree steps starting at north.
const WIND_DIR: [u16; 16] = [
    0, 23, 45, 68, 90, 113, 135, 158, 180, 203, 225, 248, 270, 293, 315, 338,
];

const FRAME_BYTES: usize = 11;
const FRAME_BITS: usize = FRAME_BYTES * 8;

pub struct FineOffsetWh1080;

impl Protocol for FineOffsetWh1080 {
    fn name(&self) -> &'static str {
        "Fineoffset-WHx080"
    }

    fn timing(&self) -> Timing {
        // rtl_433's published figures. A real envelope detector measures both
        // widths roughly 55 us short, which midpoint classification absorbs.
        Timing::pwm(544, 1524, 2800)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        if bits.len() < FRAME_BITS {
            return Err(DecodeError::WrongLength {
                got: bits.len(),
                want: FRAME_BITS,
            });
        }

        // Sync on the 0xff preamble rather than assuming the frame starts at
        // bit zero. The slicer begins wherever the detector first triggered,
        // which in practice is a bit or two early or late.
        let start = bits.find(&[0xff], 8).ok_or(DecodeError::NotThisProtocol)?;
        if start + FRAME_BITS > bits.len() {
            return Err(DecodeError::WrongLength {
                got: bits.len() - start,
                want: FRAME_BITS,
            });
        }
        let frame = bits.slice(start, FRAME_BITS);
        let b = frame.as_bytes();

        if crc8(b, 0x31, 0xff) != 0 {
            return Err(DecodeError::CrcFailed);
        }

        let msg_format = b[1] >> 4;
        let device_id = ((b[1] << 4) & 0xf0) | (b[2] >> 4);

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.to_vec();
        r = r.int("station_id", device_id as i64);

        match msg_format {
            0x0a => {
                // Only the low 10 bits are temperature; the upper 2 are a sign
                // convention this variant does not use.
                let temp_raw = (((b[2] & 0x03) as u16) << 8) | b[3] as u16;
                let temperature = (temp_raw as f64 - 400.0) * 0.1;
                if !(-50.0..=80.0).contains(&temperature) {
                    return Err(DecodeError::Implausible("temperature out of range"));
                }
                let humidity = b[4];
                if humidity > 100 {
                    return Err(DecodeError::Implausible("humidity above 100%"));
                }
                let rain_raw = (((b[7] & 0x0f) as u16) << 8) | b[8] as u16;

                r = r
                    .int("msg_type", 0)
                    .float("temperature_c", (temperature * 10.0).round() / 10.0)
                    .int("humidity_pct", humidity as i64)
                    .float("wind_avg_ms", round2(b[5] as f64 * 0.34))
                    .float("wind_gust_ms", round2(b[6] as f64 * 0.34))
                    .int(
                        "wind_direction_deg",
                        WIND_DIR[(b[9] & 0x0f) as usize] as i64,
                    )
                    .float("rain_total_mm", round2(rain_raw as f64 * 0.3))
                    .bool("battery_ok", (b[9] >> 4) != 1);
                Ok(r)
            }
            0x0b => {
                // Around minute 59 of even hours the sensor stops sending
                // weather data and transmits a DCF77/WWVB time signal instead.
                let hours = ((b[3] & 0x30) >> 4) * 10 + (b[3] & 0x0f);
                let minutes = ((b[4] & 0xf0) >> 4) * 10 + (b[4] & 0x0f);
                let seconds = ((b[5] & 0xf0) >> 4) * 10 + (b[5] & 0x0f);
                if hours > 23 || minutes > 59 || seconds > 59 {
                    return Err(DecodeError::Implausible("impossible clock value"));
                }
                r = r
                    .int("msg_type", 1)
                    .text("time", format!("{hours:02}:{minutes:02}:{seconds:02}"));
                Ok(r)
            }
            _ => Err(DecodeError::NotThisProtocol),
        }
    }
}

fn round2(v: f64) -> f64 {
    (v * 100.0).round() / 100.0
}

/// Fine Offset / Ecowitt WH51 soil moisture probe, also sold as the Ecowitt
/// WN31, MISOL/1 and SwitchDoc Labs SM23.
///
/// FSK rather than OOK, 58 us a bit, at 433.92, 868 or 915 MHz depending on
/// the region. Fourteen bytes behind an `aa 2d d4` preamble:
///
/// ```text
/// FF II II II TB YY MM ZA AA XX XX XX CC SS
/// ```
///
/// - `FF` family code, always 0x51
/// - `II` 24 bit id
/// - `T`  transmission boost: set to 7 when the reading changes and counted
///   down, which is what makes the sensor report every 10 s instead of 70 s
/// - `B`  battery voltage in tenths of a volt
/// - `MM` moisture percent, as the sensor computes it
/// - `AAA` 9 bit raw ADC reading, which is the number to calibrate against
///   when the percentage looks wrong at the ends of the scale
/// - `CC` CRC-8, polynomial 0x31, init 0x00, over the first twelve bytes
/// - `SS` sum of the first thirteen
///
/// Two independent checks over the same bytes, so a decode here is about as
/// trustworthy as anything on these bands gets.
pub struct FineOffsetWh51;

const WH51_BYTES: usize = 14;
const WH51_PREAMBLE: [u8; 3] = [0xaa, 0x2d, 0xd4];
const WH51_FAMILY: u8 = 0x51;

impl Protocol for FineOffsetWh51 {
    fn name(&self) -> &'static str {
        "Fineoffset-WH51"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Nrz,
            short_us: 58,
            long_us: 58,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 5000,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        let at = bits
            .find(&WH51_PREAMBLE, 24)
            .ok_or(DecodeError::NotThisProtocol)?;
        let start = at + 24;
        if start + WH51_BYTES * 8 > bits.len() {
            return Err(DecodeError::WrongLength {
                got: bits.len() - start,
                want: WH51_BYTES * 8,
            });
        }
        let frame = bits.slice(start, WH51_BYTES * 8);
        let b = frame.as_bytes();
        if b[0] != WH51_FAMILY {
            return Err(DecodeError::NotThisProtocol);
        }
        if crc8(&b[..12], 0x31, 0x00) != b[12] || checksum8(&b[..13]) != b[13] {
            return Err(DecodeError::CrcFailed);
        }

        let moisture = b[6];
        if moisture > 100 {
            return Err(DecodeError::Implausible("moisture above 100%"));
        }

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.to_vec();
        Ok(
            r.text("id", format!("{:02x}{:02x}{:02x}", b[1], b[2], b[3]))
                .int("moisture_pct", moisture as i64)
                .int("ad_raw", (((b[7] as i64) & 0x01) << 8) | b[8] as i64)
                .int("battery_mv", (b[4] & 0x1f) as i64 * 100)
                .int("boost", (b[4] >> 5) as i64)
                // A single alkaline cell: 1.6 V is fresh, 1.2 V is about to take
                // the readings with it. Reported as millivolts rather than a
                // fraction so the threshold stays the reader's to choose.
                .bool("battery_ok", b[4] & 0x1f >= 13),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// Build a valid weather frame with a correct CRC.
    fn frame(
        id: u8,
        temp_c: f64,
        humidity: u8,
        wind: u8,
        gust: u8,
        rain_raw: u16,
        dir: u8,
        battery_low: bool,
    ) -> Vec<u8> {
        let temp_raw = ((temp_c * 10.0) + 400.0).round() as u16;
        let mut b = vec![0u8; FRAME_BYTES];
        b[0] = 0xff;
        b[1] = 0xa0 | (id >> 4);
        b[2] = ((id & 0x0f) << 4) | ((temp_raw >> 8) as u8 & 0x03);
        b[3] = temp_raw as u8;
        b[4] = humidity;
        b[5] = wind;
        b[6] = gust;
        b[7] = (rain_raw >> 8) as u8 & 0x0f;
        b[8] = rain_raw as u8;
        b[9] = (if battery_low { 0x10 } else { 0x00 }) | (dir & 0x0f);
        b[10] = crc8(&b[..10], 0x31, 0xff);
        b
    }

    #[test]
    fn decodes_a_synthetic_weather_frame() {
        let f = frame(196, 16.2, 89, 0, 0, 281, 8, false);
        let r = FineOffsetWh1080.decode(&BitBuffer::from_bytes(&f)).unwrap();
        assert_eq!(r.get("station_id"), Some(&Value::Int(196)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(16.2)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(89)));
        assert_eq!(r.get("wind_direction_deg"), Some(&Value::Int(180)));
        assert_eq!(r.get("rain_total_mm"), Some(&Value::Float(84.3)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn finds_a_frame_that_does_not_start_at_bit_zero() {
        let f = frame(196, 16.2, 89, 0, 0, 281, 8, false);
        let mut b = BitBuffer::new();
        // Five junk bits before the preamble, as a real slicer would produce.
        for bit in [false, true, true, false, false] {
            b.push(bit);
        }
        for byte in &f {
            for i in 0..8 {
                b.push(byte & (0x80 >> i) != 0);
            }
        }
        let r = FineOffsetWh1080.decode(&b).unwrap();
        assert_eq!(r.get("station_id"), Some(&Value::Int(196)));
    }

    #[test]
    fn a_corrupted_frame_fails_crc_rather_than_decoding_wrongly() {
        let mut f = frame(196, 16.2, 89, 0, 0, 281, 8, false);
        f[4] ^= 0x20; // flip a humidity bit
        assert_eq!(
            FineOffsetWh1080.decode(&BitBuffer::from_bytes(&f)),
            Err(DecodeError::CrcFailed)
        );
    }

    #[test]
    fn implausible_humidity_is_rejected_even_with_a_valid_crc() {
        // A valid CRC over nonsense must still be refused: CRC-8 lets roughly
        // one corrupt frame in 256 through, and at hundreds of packets an hour
        // that is a phantom reading every few minutes.
        let mut b = vec![0u8; FRAME_BYTES];
        b[0] = 0xff;
        b[1] = 0xac;
        b[2] = 0x40;
        b[3] = 0xa2;
        b[4] = 200; // impossible humidity
        b[10] = crc8(&b[..10], 0x31, 0xff);
        assert_eq!(
            FineOffsetWh1080.decode(&BitBuffer::from_bytes(&b)),
            Err(DecodeError::Implausible("humidity above 100%"))
        );
    }

    #[test]
    fn battery_low_flag_is_read() {
        let f = frame(196, 16.2, 89, 0, 0, 281, 8, true);
        let r = FineOffsetWh1080.decode(&BitBuffer::from_bytes(&f)).unwrap();
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
    }

    #[test]
    fn wind_speed_scales_by_034_metres_per_second() {
        let f = frame(196, 16.2, 89, 10, 20, 0, 0, false);
        let r = FineOffsetWh1080.decode(&BitBuffer::from_bytes(&f)).unwrap();
        assert_eq!(r.get("wind_avg_ms"), Some(&Value::Float(3.4)));
        assert_eq!(r.get("wind_gust_ms"), Some(&Value::Float(6.8)));
    }

    #[test]
    fn a_short_buffer_is_rejected() {
        let b = BitBuffer::from_bytes(&[0xff, 0xa0, 0x00]);
        assert!(matches!(
            FineOffsetWh1080.decode(&b),
            Err(DecodeError::WrongLength { .. })
        ));
    }
}
