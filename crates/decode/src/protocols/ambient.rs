//! Ambient Weather F007TH, and the sensors that share its frame: the F012TH,
//! the SwitchDoc F016TH and TFA's 30.3208.02 sender.
//!
//! 433.92 or 868 MHz, Manchester at 1 kbaud, three copies back to back behind
//! a preamble of zeros. After the preamble the frame is six bytes:
//!
//! ```text
//! xxxxMMMM IIIIIIII BCCCTTTT TTTTTTTT HHHHHHHH DDDDDDDD
//! ```
//!
//! - `M`  model nibble, 5 on every sensor known to send this frame
//! - `I`  8 bit id, redrawn when the batteries go in
//! - `B`  battery low, `C` a three bit channel set by a switch
//! - `T`  12 bit temperature, tenths of a degree Fahrenheit offset by 400
//! - `H`  humidity percent
//! - `D`  LFSR digest, gen 0x98, key 0x3e, over the five bytes before it,
//!   exclusive-ored with 0x64
//!
//! The digest is eight bits over five bytes, which is weak enough that
//! rtl_433 puts sanity rules behind it, and those are here too: a humidity
//! above 100% or a temperature outside the manual's -40 to 140 F is a frame
//! that passed by luck.
//!
//! What the frame does not have is a length or a terminator, so the decoder
//! finds the preamble rather than assuming the burst starts at one. Both
//! polarities are searched, because which half of a Manchester symbol carries
//! the bit depends on where the detector triggered.

use crate::bits::{BitBuffer, lfsr_digest8};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{Coding, Timing};

pub struct AmbientF007th;

/// The last twelve bits before the frame: eight of the preamble's zeros and
/// the `0100` the model byte opens with. Searched twelve bits long and the
/// frame taken from eight in, so those four bits are both the end of the
/// preamble and the start of the payload, which is how rtl_433 reads it.
const PREAMBLE: u32 = 0x014;
const PREAMBLE_BITS: usize = 12;
const FRAME_BYTES: usize = 6;

impl Protocol for AmbientF007th {
    fn name(&self) -> &'static str {
        "Ambientweather-F007TH"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Manchester,
            short_us: 500,
            long_us: 1000,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 2400,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        if bits.len() < PREAMBLE_BITS + FRAME_BYTES * 8 {
            return Err(DecodeError::WrongLength {
                got: bits.len(),
                want: PREAMBLE_BITS + FRAME_BYTES * 8,
            });
        }
        let mut passed_digest = false;
        for stream in [bits.slice(0, bits.len()), bits.inverted()] {
            let last = stream.len() - (8 + FRAME_BYTES * 8);
            for at in 0..=last {
                if stream.extract(at, PREAMBLE_BITS) != Some(PREAMBLE) {
                    continue;
                }
                let b = stream.slice(at + 8, FRAME_BYTES * 8).as_padded_bytes().to_vec();
                if lfsr_digest8(&b[..5], 0x98, 0x3e) ^ 0x64 != b[5] {
                    continue;
                }
                passed_digest = true;
                if let Ok(r) = report(self.name(), &b) {
                    return Ok(r);
                }
            }
        }
        Err(if passed_digest {
            DecodeError::Implausible("reading outside the sensor's range")
        } else {
            DecodeError::NotThisProtocol
        })
    }
}

fn report(name: &'static str, b: &[u8]) -> Result<Report, DecodeError> {
    let humidity = b[4];
    if humidity > 100 {
        return Err(DecodeError::Implausible("humidity above 100%"));
    }
    let tenths_f = (((b[2] & 0x0f) as i32) << 8 | b[3] as i32) - 400;
    let fahrenheit = tenths_f as f64 * 0.1;
    if !(-40.0..=140.0).contains(&fahrenheit) {
        return Err(DecodeError::Implausible("temperature outside -40 to 140 F"));
    }

    let mut r = Report::new(name);
    r.crc_valid = Some(true);
    r.raw = b.to_vec();
    Ok(r.int("id", b[1] as i64)
        .int("channel", (((b[2] & 0x70) >> 4) + 1) as i64)
        .bool("battery_ok", b[2] & 0x80 == 0)
        .float("temperature_c", (((fahrenheit - 32.0) / 1.8) * 10.0).round() / 10.0)
        .int("humidity_pct", humidity as i64))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;

    /// A frame as the sensor sends it: zeros, the single one bit that ends the
    /// preamble, then the six bytes with their digest.
    fn frame(id: u8, channel: u8, tenths_f: i32, humidity: u8, battery_ok: bool) -> BitBuffer {
        let raw = (tenths_f + 400) as u16;
        let mut b = [0u8; 6];
        b[0] = 0x45;
        b[1] = id;
        b[2] = (!battery_ok as u8) << 7 | (channel - 1) << 4 | (raw >> 8) as u8 & 0x0f;
        b[3] = raw as u8;
        b[4] = humidity;
        b[5] = lfsr_digest8(&b[..5], 0x98, 0x3e) ^ 0x64;
        let mut bits = BitBuffer::new();
        bits.extend(false, 15);
        bits.push(true);
        for byte in b {
            for i in (0..8).rev() {
                bits.push(byte >> i & 1 != 0);
            }
        }
        bits
    }

    #[test]
    fn decodes_a_sensor_reading() {
        let r = AmbientF007th.decode(&frame(3, 4, 721, 15, true)).unwrap();
        assert_eq!(r.get("id"), Some(&Value::Int(3)));
        assert_eq!(r.get("channel"), Some(&Value::Int(4)));
        assert_eq!(r.get("humidity_pct"), Some(&Value::Int(15)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        // 72.1 F, as rtl_433 reports it for the third capture in the corpus.
        assert_eq!(r.get("temperature_c"), Some(&Value::Float(22.3)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn reads_the_stream_whichever_way_round_it_arrived() {
        let f = frame(169, 1, -46, 19, true);
        let want = AmbientF007th.decode(&f).unwrap();
        assert_eq!(AmbientF007th.decode(&f.inverted()).unwrap(), want);
        assert_eq!(want.get("temperature_c"), Some(&Value::Float(-20.3)));
    }

    #[test]
    fn a_broken_digest_is_not_this_protocol() {
        let f = frame(37, 5, 678, 35, false);
        let mut broken = BitBuffer::new();
        for i in 0..f.len() {
            broken.push(if i == 60 { !f.get(i).unwrap() } else { f.get(i).unwrap() });
        }
        assert_eq!(AmbientF007th.decode(&broken), Err(DecodeError::NotThisProtocol));
    }

    #[test]
    fn an_impossible_humidity_is_refused_even_though_the_digest_passes() {
        // Eight bits over five bytes lets one frame in 256 through, so the
        // sanity rules are load bearing rather than decoration.
        let r = AmbientF007th.decode(&frame(3, 4, 721, 200, true));
        assert!(matches!(r, Err(DecodeError::Implausible(_))), "got {r:?}");
    }
}
