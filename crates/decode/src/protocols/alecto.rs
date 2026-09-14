//! Alecto V1 weather sensors: the WS3500 and WS4500, Ventus W155 and W044,
//! Auriol and Silvercrest rebadges of the same transmitter.
//!
//! 433.92 MHz, PPM at 2 and 4 ms, the same 36 bits and the same timings as
//! Prologue, and told apart from it by the checksum in the last nibble. Bytes
//! arrive least significant bit first, so every field below is read out of a
//! bit-reversed byte.
//!
//! ```text
//! IIIIIIII BMMP TTTT TTTT TTTT HHHHHHHH CCCC
//! ```
//!
//! - `I` 8 bit id, two bits of which are the channel, redrawn at power up
//! - `B` battery low, `M` message type, `P` set when the button was pressed
//! - `T` 12 bit temperature, tenths of a degree, two's complement
//! - `H` humidity, BCD percent
//! - `C` the nibbles of the first four bytes summed, biased by whether the
//!   frame is a rain message, and stored bit-reversed
//!
//! Message type 3 is the rain gauge, whose reading replaces the temperature
//! and humidity, and a wind message when the low nibble says so. Wind is not
//! read here: it takes its speed from one row of a burst and its gust and
//! direction from another five rows later, which the detector's own framing
//! does not reliably deliver, and rtl_433 marks its own version untested.
//!
//! Four bits of checksum is one frame in sixteen passing by luck, so the same
//! corroboration Prologue needs applies: the frame appears on two rows, or the
//! package is one frame and nothing else.

use crate::bits::{BitBuffer, reflect8};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::Timing;

pub struct AlectoV1;

const FRAME_BITS: usize = 36;
const COPIES: usize = 2;

impl Protocol for AlectoV1 {
    fn name(&self) -> &'static str {
        // Overwritten per message below, since rtl_433 names the rain gauge
        // and the thermo-hygrometer separately.
        "AlectoV1-Temperature"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(2000, 4000, 10_000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        if bits.len() < FRAME_BITS {
            return Err(DecodeError::WrongLength { got: bits.len(), want: FRAME_BITS });
        }
        let b = frame(bits).ok_or(DecodeError::NotThisProtocol)?;

        let id = reflect8(b[0]);
        let channel = ((b[0] & 0x0c) >> 2) as i64;
        let battery_ok = b[1] & 0x80 == 0;
        let message = (b[1] & 0x60) >> 5;
        let rain = b[1] & 0x0f == 0x0c;

        let mut r = Report::new(self.name());
        r.crc_valid = Some(true);
        r.raw = b.to_vec();
        r = r.int("id", id as i64).int("channel", channel).bool("battery_ok", battery_ok);

        if message == 0x3 {
            if !rain {
                // A wind message, which needs rows this decoder does not read.
                return Err(DecodeError::NotThisProtocol);
            }
            r.model = "AlectoV1-Rain";
            let raw = (reflect8(b[3]) as u32) << 8 | reflect8(b[2]) as u32;
            return Ok(r.float("rain_total_mm", raw as f64 * 0.25));
        }

        let temp_raw = ((reflect8(b[1]) as u16 & 0xf0) | (reflect8(b[2]) as u16) << 8) as i16;
        let temperature = (temp_raw >> 4) as f64 * 0.1;
        let humidity = bcd(reflect8(b[3]));
        // rtl_433's own guard against reading a Prologue frame as this one.
        if humidity > 100 {
            return Err(DecodeError::Implausible("humidity above 100%"));
        }
        Ok(r.float("temperature_c", (temperature * 10.0).round() / 10.0)
            .int("humidity_pct", humidity as i64))
    }
}

/// The 36 bit row this burst sent twice, or the one row a package holding a
/// single copy is.
fn frame(bits: &BitBuffer) -> Option<[u8; 5]> {
    let mut starts = vec![0];
    starts.extend(bits.rows().iter().copied().filter(|s| *s != 0));
    let ends = starts.iter().skip(1).copied().chain(std::iter::once(bits.len()));
    let rows: Vec<BitBuffer> = starts
        .iter()
        .copied()
        .zip(ends)
        .filter(|(start, end)| (FRAME_BITS..=FRAME_BITS + 1).contains(&(end - start)))
        .map(|(start, _)| bits.slice(start, FRAME_BITS))
        .collect();
    let alone = rows.len() == 1 && bits.len() <= FRAME_BITS + 1;
    let row = rows.iter().find(|r| {
        let mut b = [0u8; 5];
        b.copy_from_slice(&r.as_padded_bytes()[..5]);
        checksum_ok(&b) && (alone || rows.iter().filter(|o| o == r).count() >= COPIES)
    })?;
    let mut b = [0u8; 5];
    b.copy_from_slice(&row.as_padded_bytes()[..5]);
    Some(b)
}

/// The nibbles of the first four bytes summed with every byte bit-reversed,
/// biased by whether the frame is a rain message, against the top nibble of
/// the fifth byte.
pub(crate) fn checksum_ok(b: &[u8; 5]) -> bool {
    let sum: u32 =
        b[..4].iter().map(|x| reflect8(*x)).map(|x| (x & 0x0f) as u32 + (x >> 4) as u32).sum();
    let sum = if b[1] & 0x7f == 0x6c { sum + 7 } else { 0x0f_u32.wrapping_sub(sum) };
    reflect8(((sum & 0x0f) << 4) as u8) == b[4] >> 4
}

fn bcd(x: u8) -> u8 {
    (x >> 4) * 10 + (x & 0x0f)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A burst as the sensor sends it, from the five bytes on the air.
    fn burst(b: [u8; 5], copies: usize) -> BitBuffer {
        let mut bits = BitBuffer::new();
        for copy in 0..copies {
            if copy > 0 {
                bits.mark_row();
            }
            for i in 0..FRAME_BITS {
                bits.push(b[i / 8] >> (7 - i % 8) & 1 != 0);
            }
        }
        bits
    }

    /// The five bytes of a thermo-hygrometer message, built from the fields.
    ///
    /// Every byte is written reflected and then reversed, because that is the
    /// order the sensor sends them in and the order the layout is stated in.
    fn thermo(id: u8, temp_c: f64, humidity: u8, battery_ok: bool) -> [u8; 5] {
        let v = ((temp_c * 10.0).round() as i16 & 0x0fff) as u16;
        let mut b = [0u8; 5];
        b[0] = reflect8(id);
        // The temperature's low nibble shares a byte with the flags: message
        // type zero, no button, and the battery bit at the bottom.
        b[1] = reflect8(((v & 0x0f) << 4) as u8 | !battery_ok as u8);
        b[2] = reflect8((v >> 4) as u8);
        b[3] = reflect8((humidity / 10) << 4 | (humidity % 10));
        let sum: u32 =
            b[..4].iter().map(|x| reflect8(*x)).map(|x| (x & 0x0f) as u32 + (x >> 4) as u32).sum();
        let sum = if b[1] & 0x7f == 0x6c { sum + 7 } else { 0x0f_u32.wrapping_sub(sum) };
        b[4] = reflect8(((sum & 0x0f) << 4) as u8) << 4;
        b
    }

    #[test]
    fn decodes_a_thermo_hygrometer() {
        let b = thermo(44, 28.8, 36, true);
        let r = AlectoV1.decode(&burst(b, 7)).unwrap();
        assert_eq!(r.model, "AlectoV1-Temperature");
        assert_eq!(r.get("id"), Some(&Value::Int(44)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(28.8)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(36)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn a_temperature_below_zero_survives_the_sign_extension() {
        let r = AlectoV1.decode(&burst(thermo(247, -5.1, 62, true), 7)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-5.1)));
    }

    #[test]
    fn a_broken_checksum_is_not_this_protocol() {
        let mut b = thermo(44, 28.8, 36, true);
        b[4] ^= 0x10;
        assert_eq!(AlectoV1.decode(&burst(b, 7)), Err(DecodeError::NotThisProtocol));
    }

    #[test]
    fn one_copy_inside_a_longer_burst_is_not_claimed() {
        // Four bits of checksum passes on one window in sixteen, so a single
        // copy is not evidence unless the package is that copy.
        let b = thermo(44, 28.8, 36, true);
        let mut once = burst(b, 1);
        once.mark_row();
        once.extend(false, 20);
        assert_eq!(AlectoV1.decode(&once), Err(DecodeError::NotThisProtocol));
        assert!(AlectoV1.decode(&burst(b, 1)).is_ok());
    }
}
