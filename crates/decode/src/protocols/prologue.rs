//! Prologue thermo-hygrometers, and the sensors sold as FreeTec NC-7104,
//! Pearl NC-7159-675, ThermoPro TX2 and TFA's 30.3240.10 pool thermometer.
//!
//! 433.92 MHz, PPM with a 500 us mark, a 2000 us gap for a zero and a 4000 us
//! gap for a one, 36 bits sent seven times with a 9000 us gap between copies.
//!
//! ```text
//! [type] [id0] [id1] [flags] [temp0] [temp1] [temp2] [humi0] [humi1]
//! ```
//!
//! - `type` 4 bits, 9 or 5, which is the only fixed field in the frame
//! - `id`   8 bits, redrawn when the batteries go in
//! - flags  battery ok, button pressed, then a two bit channel
//! - `temp` 12 bit signed, tenths of a degree Celsius
//! - `humi` 8 bits percent, 0xcc when the sensor has no humidity element
//!
//! There is no checksum, so the corroboration is the transmission's own shape:
//! a frame is claimed only where two of the burst's rows are 36 bits long and
//! identical, or where a package is one such row and nothing else. Nothing
//! weaker will do, because a four bit constant lets one window in sixteen
//! through. rtl_433 asks for four copies of the seven the sensor sends, which
//! it can because its reset limit holds a whole transmission in one buffer;
//! this receiver's detector cuts a package at the 9 ms gap between copies, so
//! asking for four here reads none of these sensors at all.
//!
//! The Alecto V1 family (Auriol, Unitec W186-F) sends the same 36 bits at the
//! same timings with a real checksum in the last nibble, and rtl_433 resolves
//! the collision by decoder priority. Here the frame is handed over instead: a
//! frame satisfying that checksum is theirs.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::Timing;

pub struct PrologueTh;

const FRAME_BITS: usize = 36;
/// Copies of the frame a burst must carry before it is read without a
/// checksum, or one where the package holds a single frame and nothing else.
const COPIES: usize = 2;

impl Protocol for PrologueTh {
    fn name(&self) -> &'static str {
        "Prologue-TH"
    }

    fn timing(&self) -> Timing {
        Timing::ppm(2000, 4000, 10_000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        if bits.len() < FRAME_BITS {
            return Err(DecodeError::WrongLength { got: bits.len(), want: FRAME_BITS });
        }
        let b = repeated_row(bits).ok_or(DecodeError::NotThisProtocol)?;

        if b[0] & 0xf0 != 0x90 && b[0] & 0xf0 != 0x50 {
            return Err(DecodeError::NotThisProtocol);
        }
        if super::alecto::checksum_ok(&b) {
            return Err(DecodeError::NotThisProtocol);
        }

        let temp_raw = (((b[2] as u16) << 8 | (b[3] & 0xf0) as u16) as i16) >> 4;
        let humidity = (b[3] & 0x0f) << 4 | b[4] >> 4;

        let mut r = Report::new(self.name());
        // Four bits of fixed type are not an integrity check, and a reading
        // taken on that alone must not be shown as a verified one.
        r.crc_valid = None;
        r.raw = b.to_vec();
        r = r
            .int("subtype", (b[0] >> 4) as i64)
            .int("id", ((b[0] & 0x0f) << 4 | b[1] >> 4) as i64)
            .int("channel", (b[1] & 0x03) as i64 + 1)
            .bool("battery_ok", b[1] & 0x08 != 0)
            .bool("button", b[1] & 0x04 != 0)
            .float("temperature_c", (temp_raw as f64 * 0.1 * 10.0).round() / 10.0);
        // 0xcc is the sensor saying it has no humidity element. Any other
        // value is a reading, zero included.
        if humidity != 0xcc {
            r = r.int("humidity_pct", humidity as i64);
        }
        Ok(r)
    }
}

/// The 36 bit row this burst sent at least [`COPIES`] times, or the one row a
/// package holding a single copy is.
///
/// Rows rather than bit offsets, because the 9000 us gap between copies is
/// where the slicer cut and a frame starts there or nowhere. A row longer than
/// 37 bits is not this protocol: rtl_433 allows the one trailing zero a
/// detector adds and nothing beyond it.
///
/// A package that is one frame long and nothing else is the detector agreeing
/// with the frame's own boundaries, and is accepted on that alone, as the
/// Nexus decoder accepts it.
fn repeated_row(bits: &BitBuffer) -> Option<[u8; 5]> {
    // The slicer marks a row where it cut, which leaves the first copy's start
    // unmarked: it is where the buffer begins.
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
    let row = rows.iter().find(|r| alone || rows.iter().filter(|o| o == r).count() >= COPIES)?;
    let mut b = [0u8; 5];
    b.copy_from_slice(&row.as_padded_bytes()[..5]);
    Some(b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A burst as the sensor sends it: the frame on `copies` rows of its own.
    fn burst(
        subtype: u8,
        id: u8,
        channel: u8,
        tenths_c: i16,
        humidity: u8,
        button: bool,
        copies: usize,
    ) -> BitBuffer {
        let raw = (tenths_c & 0x0fff) as u16;
        let b = [
            subtype << 4 | id >> 4,
            id << 4 | 0x08 | (button as u8) << 2 | (channel - 1),
            (raw >> 4) as u8,
            (raw << 4) as u8 | humidity >> 4,
            humidity << 4,
        ];
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

    #[test]
    fn decodes_a_sensor_reading() {
        let r = PrologueTh.decode(&burst(5, 167, 3, 146, 90, true, 7)).unwrap();
        assert_eq!(r.get("subtype"), Some(&Value::Int(5)));
        assert_eq!(r.get("id"), Some(&Value::Int(167)));
        assert_eq!(r.get("channel"), Some(&Value::Int(3)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(14.6)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(90)));
        assert_eq!(r.get("button"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, None);
    }

    #[test]
    fn a_temperature_below_zero_survives_the_sign_extension() {
        let r = PrologueTh.decode(&burst(5, 242, 1, -76, 40, false, 7)).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-7.6)));
    }

    #[test]
    fn a_sensor_without_a_humidity_element_reports_none() {
        let r = PrologueTh.decode(&burst(9, 78, 1, 10, 0xcc, false, 7)).unwrap();
        assert!(r.get("humidity_pct").is_none());
        // Zero is a reading, though, and rtl_433 prints it.
        let r = PrologueTh.decode(&burst(9, 213, 2, 237, 0, false, 7)).unwrap();
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(0)));
    }

    #[test]
    fn a_frame_inside_a_longer_burst_needs_a_copy_of_itself() {
        // One copy in a package holding other rows is a window that happened
        // to have the type nibble in the right place, and there are four bits
        // of that.
        let mut once = burst(5, 167, 3, 146, 90, false, 1);
        once.mark_row();
        once.extend(false, 20);
        assert_eq!(PrologueTh.decode(&once), Err(DecodeError::NotThisProtocol));
        assert!(PrologueTh.decode(&burst(5, 167, 3, 146, 90, false, 2)).is_ok());
    }

    #[test]
    fn a_package_holding_one_frame_and_nothing_else_is_read() {
        // What the detector hands over for these sensors: it cuts on the 9 ms
        // gap, so each of the seven copies arrives on its own.
        assert!(PrologueTh.decode(&burst(9, 78, 1, 10, 0xcc, false, 1)).is_ok());
    }
}
