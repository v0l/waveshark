//! Hideki sensors, sold as Cresta, TFA Nexus, Irox, Mebus and the Bresser 5CH:
//! a thermo-hygrometer, a temperature-only sensor, an anemometer and a rain
//! gauge, all on one frame.
//!
//! 433.92 MHz, differential Manchester at 1040 us a bit, sent on air inverted.
//! Each byte carries an even parity bit after it, so nine bits on the wire
//! carry eight of payload, and the payload ends with a byte-wise XOR and a
//! CRC-8. Only after both checks are the bytes reflected into the order the
//! layout is written in.
//!
//! ```text
//! [sync] [rc chan] [len] [seq type] ...payload... [xor] [crc]
//! ```
//!
//! How many bytes there are is what says which sensor sent it: seven payload
//! bytes is the temperature-only sensor, eight the rain gauge, nine the
//! thermo-hygrometer and thirteen the anemometer. The length byte says the
//! same thing and is checked against it.
//!
//! The temperature is BCD in tenths with its sign in a bit of its own, the
//! humidity is BCD percent, the rain gauge counts 0.7 mm a tip, and the wind
//! direction is a four bit code through a table that is not in order, because
//! it is the output of a reed switch ring rather than a number.

use crate::bits::{BitBuffer, crc8, xor8};
use crate::protocol::{DecodeError, Proof, Protocol, Report};
use crate::slicer::{Coding, Timing, differential_manchester_decode, slice_manchester_half};
use common::Pulse;

pub struct Hideki;

/// The last of the sync, not inverted: `00000110 1`.
const SYNC: u32 = 0x0d;
const SYNC_BITS: usize = 9;

impl Protocol for Hideki {
    fn name(&self) -> &'static str {
        "Hideki-TS04"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Manchester,
            short_us: 520,
            long_us: 1040,
            sync_us: 0,
            tolerance_us: 240,
            reset_us: 4000,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // The differential decoder's output, which is what `decode_burst`
        // hands over once it has found the sync.
        read(bits)
    }

    fn decode_burst(&self, pulses: &[Pulse]) -> Result<Report, DecodeError> {
        let raw = slice_manchester_half(pulses, &self.timing())
            .map_err(|_| DecodeError::NotThisProtocol)?;
        // Differential decoding is blind to polarity, so both ways round are
        // one stream here. The sync is written the way the specification has
        // it and the payload is sent inverted, which is why the search and the
        // read are on opposite polarities of the same bits.
        let decoded = differential_manchester_decode(&raw, 0, 14 * 9 + 16);
        let flipped = decoded.inverted();
        let mut best = Err(DecodeError::NotThisProtocol);
        for (find, take) in [(&flipped, &decoded), (&decoded, &flipped)] {
            for at in 0..find.len().saturating_sub(SYNC_BITS) {
                if find.extract(at, SYNC_BITS) != Some(SYNC) {
                    continue;
                }
                let body = take.slice(at + SYNC_BITS, take.len() - at - SYNC_BITS);
                match read(&body) {
                    Ok(r) => return Ok(r),
                    Err(e) => best = Err(e),
                }
            }
        }
        best
    }
}

/// Read a frame from the bits behind the sync, parity bits still in place.
fn read(bits: &BitBuffer) -> Result<Report, DecodeError> {
    // Up to four bits may be missing off the end, as rtl_433 allows.
    let bytes = (bits.len() + 4) / 9;
    let kind = Kind::of(bytes).ok_or(DecodeError::WrongLength { got: bits.len(), want: 9 * 9 })?;

    // A transmission can end up to four bits short of its last parity bit,
    // which reads as a zero the way it does out of rtl_433's own zero-padded
    // row; the XOR and the CRC are what say the frame is whole.
    let bit = |i: usize| bits.get(i).unwrap_or(false);
    let mut packet = Vec::with_capacity(bytes);
    for i in 0..bytes {
        let at = i * 9;
        let mut byte = 0u8;
        for k in 0..8 {
            byte = byte << 1 | bit(at + k) as u8;
        }
        let parity = bit(at + 8);
        if parity != (byte.count_ones() % 2 == 1) {
            return Err(DecodeError::CrcFailed);
        }
        packet.push(byte);
    }
    if xor8(&packet[..bytes - 1]) != 0 {
        return Err(DecodeError::CrcFailed);
    }
    if crc8(&packet, 0x07, 0x00) != 0 {
        return Err(DecodeError::CrcFailed);
    }
    // Only now are the bytes in the order the layout is written in.
    let b: Vec<u8> = packet.iter().map(|x| crate::bits::reflect8(*x)).collect();
    if ((b[1] >> 1) & 0x1f) as usize + 2 != bytes {
        return Err(DecodeError::Implausible("the length byte disagrees with the frame"));
    }

    let mut channel = ((b[0] >> 5) & 0x0f) as i64;
    // Channel 5 is reported as 4 by the station and by rtl_433: the fifth
    // switch position skips a code.
    if channel >= 5 {
        channel -= 1;
    }
    let mut r = Report::new(kind.model());
    r.proof = Proof::Checked(8);
    r.raw = b.clone();
    r = r.int("id", (b[0] & 0x0f) as i64).int("channel", channel);

    if kind == Kind::Rain {
        let tips = (b[4] as u32) << 8 | b[3] as u32;
        return Ok(r
            .bool("battery_ok", b[1] & 0x40 != 0)
            .float("rain_total_mm", (tips as f64 * 0.7 * 10.0).round() / 10.0));
    }

    let tenths = (b[4] & 0x0f) as i32 * 100 + (b[3] >> 4) as i32 * 10 + (b[3] & 0x0f) as i32;
    let tenths = if b[4] & 0x80 == 0 { -tenths } else { tenths };
    r = r
        .bool("battery_ok", b[4] & 0x40 != 0)
        .float("temperature_c", (tenths as f64 * 0.1 * 10.0).round() / 10.0);

    Ok(match kind {
        Kind::Ts04 => r.int("humidity_pct", ((b[5] >> 4) * 10 + (b[5] & 0x0f)) as i64),
        Kind::Wind => {
            let speed =
                (b[8] & 0x0f) as f64 * 100.0 + (b[7] >> 4) as f64 * 10.0 + (b[7] & 0x0f) as f64;
            let gust =
                (b[9] >> 4) as f64 * 100.0 + (b[9] & 0x0f) as f64 * 10.0 + (b[8] >> 4) as f64;
            r.float("wind_avg_ms", mph_to_ms(speed * 0.1))
                .float("wind_gust_ms", mph_to_ms(gust * 0.1))
                .float("wind_direction_deg", DIRECTION[(b[10] >> 4) as usize] as f64 * 22.5)
                .int("wind_approach", APPROACH[((b[10] >> 2) & 0x03) as usize])
        }
        Kind::Temperature | Kind::Rain => r,
    })
}

fn mph_to_ms(mph: f64) -> f64 {
    (mph * 0.44704 * 100.0).round() / 100.0
}

/// The wind vane's codes in the order the ring of reed switches produces them,
/// which is not the order of the compass.
const DIRECTION: [u8; 16] = [0, 15, 13, 14, 9, 10, 12, 11, 1, 2, 4, 3, 8, 7, 5, 6];
/// Still, turning clockwise, turning anticlockwise, and a fourth code no
/// sensor is known to send.
const APPROACH: [i64; 4] = [0, 1, -1, 2];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Temperature,
    Rain,
    Ts04,
    Wind,
}

impl Kind {
    /// Which sensor a frame of this many bytes came from, the sync byte
    /// included, as rtl_433 decides it.
    fn of(bytes: usize) -> Option<Self> {
        Some(match bytes {
            7 => Self::Temperature,
            8 => Self::Rain,
            9 => Self::Ts04,
            13 => Self::Wind,
            _ => return None,
        })
    }

    fn model(self) -> &'static str {
        match self {
            Self::Temperature => "Hideki-Temperature",
            Self::Rain => "Hideki-Rain",
            Self::Ts04 => "Hideki-TS04",
            Self::Wind => "Hideki-Wind",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bits::reflect8;
    use crate::protocol::Value;

    /// The bits behind the sync for a payload given in the reflected order the
    /// layout uses: the XOR and the CRC appended, every byte reflected back,
    /// and a parity bit after each.
    fn frame(payload: &[u8]) -> BitBuffer {
        let mut packet: Vec<u8> = payload.iter().map(|b| reflect8(*b)).collect();
        packet.push(xor8(&packet));
        let crc = crc8(&packet, 0x07, 0x00);
        packet.push(crc);
        let mut bits = BitBuffer::new();
        for byte in packet {
            for i in (0..8).rev() {
                bits.push(byte >> i & 1 != 0);
            }
            bits.push(byte.count_ones() % 2 == 1);
        }
        bits
    }

    /// A thermo-hygrometer message, in the reflected order: id and channel,
    /// length, sequence and type, then the readings.
    fn ts04(channel: u8, rc: u8, tenths: i32, humidity: u8) -> Vec<u8> {
        let (sign, t) = if tenths < 0 { (0u8, -tenths) } else { (0x80, tenths) };
        vec![
            channel << 5 | rc,
            7 << 1,
            0x1e,
            ((t / 10 % 10) as u8) << 4 | (t % 10) as u8,
            sign | 0x40 | (t / 100) as u8,
            ((humidity / 10) << 4) | (humidity % 10),
            0x00,
        ]
    }

    #[test]
    fn decodes_a_thermo_hygrometer() {
        let r = read(&frame(&ts04(3, 8, 251, 69))).unwrap();
        assert_eq!(r.model, "Hideki-TS04");
        assert_eq!(r.get("id"), Some(&Value::Int(8)));
        assert_eq!(r.get("channel"), Some(&Value::Int(3)));
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(25.1)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(69)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert!(r.proof.passed());
    }

    #[test]
    fn a_temperature_below_zero_keeps_its_sign_bit() {
        let r = read(&frame(&ts04(1, 3, -44, 38))).unwrap();
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(-4.4)));
    }

    #[test]
    fn a_flipped_bit_fails_a_check_rather_than_reporting() {
        let good = frame(&ts04(3, 8, 251, 69));
        let mut broken = BitBuffer::new();
        for i in 0..good.len() {
            broken.push(if i == 30 { !good.get(i).unwrap() } else { good.get(i).unwrap() });
        }
        assert_eq!(read(&broken), Err(DecodeError::CrcFailed));
    }

    #[test]
    fn a_length_byte_that_disagrees_with_the_frame_is_refused() {
        let mut p = ts04(3, 8, 251, 69);
        p[1] = 9 << 1;
        assert!(matches!(read(&frame(&p)), Err(DecodeError::Implausible(_))));
    }
}
