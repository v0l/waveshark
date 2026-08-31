//! Somfy RTS: rolling-code blinds, awnings and garage remotes.
//!
//! 433.42 MHz, plain IEEE Manchester at a 604 us half-symbol. The on-air frame
//! is a preamble (hardware + software sync) followed by 56 Manchester bits.
//! The slicer pairs from the package's first mark, which the odd-length
//! preamble breaks, so this decoder slices the raw half-symbol stream, finds
//! the sync word in it (rtl_433 does the same, in its OOK_PCM step), and only
//! then Manchester-decodes the 56-bit payload from an aligned offset.
//!
//! The 56 bits are 7 bytes scrambled by XORing each with the previous
//! scrambled byte. Descrambled, reading big-endian:
//!
//! ```text
//! S CCCC RR RR AA AA AA
//! ```
//!
//! - `S`    seed (scrambler key, effectively random)
//! - `C`    control in the high nibble, nibble-XOR checksum in the low
//! - `RR`   replay counter, the part that rolls
//! - `AAA`  remote address
//!
//! The checksum is the XOR of every nibble folded onto one nibble; a valid
//! frame descrambles to one whose folded nibble-sum is zero.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::{manchester_decode, slice_manchester_half, Coding, Timing};
use dsp::pulse::Package;

pub struct SomfyRts;

/// Half-symbol width in microseconds; one Manchester bit is two of these.
const TE: u32 = 604;

/// Sync words in the half-symbol stream (mark=1, gap=0), from rtl_433: the
/// first-frame, retransmission, and slightly-short variants. Each is directly
/// followed by the 56-bit Manchester payload.
const PREAMBLES: &[(&[u8], usize)] = &[
    (&[0xf0, 0xf0, 0xff, 0x00], 25),
    (&[0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xff, 0x00], 49),
    (&[0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xfe, 0x00], 48),
];

const DATA_BITS: usize = 56;
const DATA_BYTES: usize = 7;

fn control_name(control: u8, seed: u8) -> &'static str {
    match control {
        1 => "My",
        2 => "Up",
        3 => "My + Up",
        4 => "Down",
        5 => "My + Down",
        6 => "Up + Down",
        7 => "My + Up + Down",
        8 => "Prog",
        9 => "Sun + Flag",
        10 => "Flag",
        // A quirk of some TEL-FIX / clone remotes: the command is fixed at 0xf
        // and the actual button lives in the seed's low nibble.
        0xf => match seed & 0xf {
            5 => "Stop",
            6 => "Up",
            8 => "Down",
            12 => "Prog",
            _ => "? (0xf)",
        },
        _ => "? (unknown)",
    }
}

impl Protocol for SomfyRts {
    fn name(&self) -> &'static str {
        "Somfy-RTS"
    }

    fn timing(&self) -> Timing {
        Timing {
            coding: Coding::Manchester,
            short_us: TE,
            long_us: TE * 2,
            sync_us: 0,
            tolerance_us: 0,
            reset_us: 4000,
        }
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // Only reachable if a caller hands over bits it paired itself. The
        // real entry point is decode_package, which needs the raw half-symbol
        // stream to find the sync; here the bits are already Manchester
        // symbols, so all that is left is to read a frame off the front.
        parse(bits, 0).ok_or(DecodeError::NotThisProtocol)
    }

    fn decode_package(&self, pkg: &Package) -> Result<Report, DecodeError> {
        let raw =
            slice_manchester_half(pkg, &self.timing()).map_err(|_| DecodeError::NotThisProtocol)?;
        // A detector may hand the level stream inverted; try both polarities.
        for inverted in [false, true] {
            let r = if inverted {
                raw.inverted()
            } else {
                raw.clone()
            };
            for (pat, bits) in PREAMBLES {
                if let Some(pos) = r.find(pat, *bits) {
                    let data_start = pos + *bits;
                    // The sync's trailing half can land on either side of the
                    // symbol boundary; try a small phase window.
                    for phase in 0..4 {
                        if let Some(rep) = scan(&r, data_start + phase) {
                            return Ok(rep);
                        }
                    }
                }
            }
        }
        Err(DecodeError::NotThisProtocol)
    }
}

/// Manchester-decode from `start` of the half-symbol stream and read a frame.
fn scan(raw: &BitBuffer, start: usize) -> Option<Report> {
    // 56 Manchester symbols need 112 half-symbols to be present at all.
    if start + DATA_BITS * 2 > raw.len() {
        return None;
    }
    parse(&manchester_decode(raw, start), 0)
}

/// Descramble and validate the 56 Manchester symbols at `start`, returning a
/// report only if the nibble checksum holds.
fn parse(dec: &BitBuffer, start: usize) -> Option<Report> {
    if start + DATA_BITS > dec.len() {
        return None;
    }
    let mut b = [0u8; DATA_BYTES];
    for (i, byte) in b.iter_mut().enumerate() {
        *byte = dec.extract(start + i * 8, 8)? as u8;
    }
    // Unscramble: each byte XORs with the previous scrambled byte.
    for i in (1..DATA_BYTES).rev() {
        b[i] ^= b[i - 1];
    }
    // Checksum is the XOR of every nibble, folded onto one nibble.
    let sum = b.iter().fold(0u8, |acc, &x| acc ^ x ^ (x >> 4)) & 0x0f;
    if sum != 0 {
        return None;
    }
    let seed = b[0];
    let control = b[1] >> 4;
    let counter = ((b[2] as u16) << 8) | b[3] as u16;
    let address = ((b[6] as u32) << 16) | ((b[5] as u32) << 8) | b[4] as u32;

    let mut r = Report::new("Somfy-RTS");
    r.crc_valid = Some(true);
    r.raw = b.to_vec();
    r = r
        .int("id", address as i64)
        .text("control", control_name(control, seed))
        .int("counter", counter as i64)
        .int("seed", seed as i64);
    Some(r)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::slicer::slice_manchester_half;

    /// Build the on-air frame: sync word then IEEE Manchester (bit 1 =
    /// low-then-high, bit 0 = high-then-low) over the scrambled 56-bit data.
    fn frame(
        seed: u8,
        control: u8,
        counter: u16,
        address: u32,
        sync: &[u8],
        sync_bits: usize,
    ) -> Package {
        let mut f = [0u8; DATA_BYTES];
        f[0] = seed;
        f[1] = control << 4;
        f[2] = (counter >> 8) as u8;
        f[3] = counter as u8;
        f[4] = address as u8;
        f[5] = (address >> 8) as u8;
        f[6] = (address >> 16) as u8;
        // Checksum: the XOR of every nibble must come to zero, so set the
        // low nibble of byte 1 to the current nibble-XOR.
        let sum: u8 = f
            .iter()
            .map(|&x| (x ^ (x >> 4)) & 0x0f)
            .fold(0, |a, b| a ^ b);
        f[1] = (control << 4) | (sum & 0x0f);
        // Scramble: each byte XORs with the previous scrambled byte.
        for i in 1..DATA_BYTES {
            f[i] ^= f[i - 1];
        }
        // half-symbol level stream: sync then data as IEEE Manchester
        let mut lv: Vec<bool> = Vec::new();
        for i in 0..sync_bits {
            lv.push(sync[i / 8] & (0x80 >> (i % 8)) != 0);
        }
        for b in f {
            for i in 0..8 {
                let bit = b & (0x80 >> i) != 0;
                lv.push(!bit); // first half
                lv.push(bit); // second half
            }
        }
        // to mark/gap pulses, starting on a mark
        let mut runs: Vec<(bool, u32)> = Vec::new();
        for &level in &lv {
            if let Some(last) = runs.last_mut() {
                if last.0 == level {
                    last.1 += 1;
                    continue;
                }
            }
            runs.push((level, 1));
        }
        let mut pulses: Vec<(u32, u32)> = Vec::new();
        // A package starts on a mark; drop a leading gap if the stream opens low.
        let mut idx = if runs[0].0 { 0 } else { 1 };
        loop {
            let mut mw = 0;
            while idx < runs.len() && runs[idx].0 {
                mw += runs[idx].1;
                idx += 1;
            }
            let mut gw = 0;
            while idx < runs.len() && !runs[idx].0 {
                gw += runs[idx].1;
                idx += 1;
            }
            if mw == 0 {
                break;
            }
            pulses.push((mw * TE, gw * TE));
            if idx >= runs.len() {
                break;
            }
        }
        // The slicer drops the final pulse's gap as inter-package silence, so
        // always close with a terminator; otherwise a frame whose last half is
        // low would lose it.
        pulses.push((0, TE));
        Package {
            pulses: pulses
                .into_iter()
                .map(|(m, g)| dsp::pulse::Pulse { mark: m, gap: g })
                .collect(),
            snr_db: 20.0,
            rssi_dbfs: -12.0,
            center_hz: 0,
            start_sample: 0,
        }
    }

    #[test]
    fn decodes_a_retransmission_frame() {
        // From a real capture: seed 0x5b, control Up, counter 0x1fe, addr 0x123456.
        let p = frame(
            0x5b,
            2,
            0x01fe,
            0x123456,
            &[0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xff, 0x00],
            49,
        );
        let raw = slice_manchester_half(&p, &SomfyRts.timing()).unwrap();
        // sanity: raw is a half-symbol stream, the sync search should find it
        assert!(raw
            .find(&[0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xff, 0x00], 49)
            .is_some());
        let r = SomfyRts.decode_package(&p).unwrap();
        assert_eq!(r.model, "Somfy-RTS");
        assert_eq!(
            r.get("control"),
            Some(&crate::protocol::Value::Text("Up".into()))
        );
        assert_eq!(r.get("counter"), Some(&crate::protocol::Value::Int(0x01fe)));
        assert_eq!(r.get("id"), Some(&crate::protocol::Value::Int(0x123456)));
        assert_eq!(r.crc_valid, Some(true));
    }

    #[test]
    fn rejects_a_frame_whose_checksum_fails() {
        let mut p = frame(
            0x5b,
            2,
            0x01fe,
            0x123456,
            &[0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xff, 0x00],
            49,
        );
        // Flip one data half-symbol by stretching a mark past its neighbour,
        // which corrupts a bit without disturbing the sync word.
        let n = p.pulses.len();
        p.pulses[n - 4].mark += TE;
        p.pulses[n - 4].gap -= TE;
        assert!(SomfyRts.decode_package(&p).is_err());
    }

    #[test]
    fn rejects_noise_with_no_sync_word() {
        let pulses: Vec<_> = (0..40)
            .map(|i| dsp::pulse::Pulse {
                mark: TE * (1 + i % 3),
                gap: TE * (1 + (i + 1) % 3),
            })
            .collect();
        let p = Package {
            pulses,
            snr_db: 20.0,
            rssi_dbfs: -12.0,
            center_hz: 0,
            start_sample: 0,
        };
        assert!(SomfyRts.decode_package(&p).is_err());
    }

    #[test]
    fn decodes_a_first_frame() {
        let p = frame(0xa7, 8, 0x0001, 0x0000aa, &[0xf0, 0xf0, 0xff, 0x00], 25);
        let r = SomfyRts.decode_package(&p).unwrap();
        assert_eq!(
            r.get("control"),
            Some(&crate::protocol::Value::Text("Prog".into()))
        );
        assert_eq!(r.get("counter"), Some(&crate::protocol::Value::Int(1)));
        assert_eq!(r.get("id"), Some(&crate::protocol::Value::Int(0x0000aa)));
    }
}
