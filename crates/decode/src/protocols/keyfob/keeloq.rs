//! Microchip KeeLoq: the rolling-code encoder inside most gate, garage and
//! car remotes that are not fixed-code.
//!
//! An HCS200 or HCS301 sends twelve preamble pulses of TE high and TE low,
//! a header of ten TE low, then 66 bits at three TE each, a 1 being TE high
//! and 2 TE low and a 0 the other way round, least significant bit first: a
//! 32-bit hopping code, which is the counter and discriminator encrypted
//! under the manufacturer's key and changes every press; a 28-bit serial
//! number; four button bits; a low-battery flag; and a repeat flag, set on
//! every frame after the first while the button is held. TE is nominally
//! 400 us.
//!
//! Nothing here can be checked. The hopping code is ciphertext, and without
//! the key it is 32 bits that change every burst, which is what a rolling
//! code is for. What makes a match believable is the frame's shape rather
//! than its contents: exactly 66 bits in a row of their own, after a row of
//! a dozen ones that the header gap separates from them. A burst of noise or
//! another protocol does not fall into that shape, so the report stands
//! with no integrity field rather than none at all.

use crate::bits::{reflect8, BitBuffer};
use crate::protocol::{DecodeError, Protocol, Report};
use crate::slicer::Timing;

pub struct KeeLoq;

/// Nominal element time, in microseconds.
const TE: u32 = 400;
const FRAME_BITS: usize = 66;

impl Protocol for KeeLoq {
    fn name(&self) -> &'static str {
        "KeeLoq"
    }

    fn timing(&self) -> Timing {
        // The opening mark, longer than any symbol, is declared as a sync so
        // the slicer starts a row on it rather than refusing the burst.
        Timing::pwm_sync(TE, 2 * TE, 4 * TE, 10_000)
    }

    fn decode(&self, bits: &BitBuffer) -> Result<Report, DecodeError> {
        // Rows as the slicer marked them, with the first starting at zero
        // whether or not anything marked it.
        let mut starts: Vec<usize> = bits.rows().to_vec();
        if starts.first() != Some(&0) {
            starts.insert(0, 0);
        }
        for (k, &start) in starts.iter().enumerate() {
            let end = starts.get(k + 1).copied().unwrap_or(bits.len());
            // The frame, and up to two bits of trailing partial symbol.
            if !(FRAME_BITS..=FRAME_BITS + 2).contains(&(end - start)) {
                continue;
            }
            // Behind it, the preamble: a dozen ones on their own row.
            let Some(&pre) = k.checked_sub(1).map(|j| &starts[j]) else { continue };
            let pre_len = start - pre;
            if !(10..=14).contains(&pre_len) || (pre..start).any(|i| bits.get(i) != Some(true)) {
                continue;
            }
            let bytes: Vec<u8> = bits
                .slice(start, FRAME_BITS)
                .as_padded_bytes()
                .iter()
                .map(|b| reflect8(*b))
                .collect();
            let hop = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            let low = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
            let serial = low & 0x0fff_ffff;
            let button = low >> 28;
            let vlow = bytes[8] & 1 != 0;
            let rpt = bytes[8] & 2 != 0;
            if serial == 0 || hop == 0 {
                continue;
            }
            let mut r = Report::new("KeeLoq");
            r.crc_valid = None;
            r.raw = bytes;
            return Ok(r
                .int("serial", serial as i64)
                .int("btn", button as i64)
                .int("hop", hop as i64)
                .bool("battery_ok", !vlow)
                .bool("repeat", rpt));
        }
        Err(DecodeError::NotThisProtocol)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Value;
    use dsp::pulse::Package;
    use dsp::Pulse;

    /// The burst as it came off the air, one of a remote pressed every few
    /// seconds on 433.889 MHz: the preamble, the header gap, 66 bits.
    const OFF_AIR: [(u32, u32); 79] = [
        (1650, 150), (375, 413), (375, 413), (375, 450), (375, 413), (375, 413), (375, 413), (413, 413),
        (375, 413), (375, 413), (413, 413), (413, 375), (413, 3938), (450, 788), (413, 788), (788, 450),
        (788, 375), (413, 788), (413, 788), (413, 788), (825, 375), (413, 788), (788, 450), (375, 788),
        (413, 788), (825, 375), (825, 375), (413, 788), (825, 375), (413, 788), (413, 788), (825, 375),
        (413, 788), (413, 788), (413, 788), (413, 788), (788, 413), (413, 788), (825, 375), (825, 413),
        (375, 788), (788, 450), (375, 788), (413, 788), (825, 375), (413, 788), (788, 413), (825, 375),
        (413, 788), (413, 788), (413, 788), (825, 375), (825, 375), (788, 450), (375, 788), (788, 413),
        (413, 788), (825, 375), (825, 413), (375, 788), (788, 413), (825, 375), (788, 413), (375, 825),
        (413, 788), (413, 788), (825, 375), (788, 413), (825, 375), (825, 375), (825, 413), (788, 375),
        (788, 450), (788, 375), (413, 788), (825, 375), (825, 375), (413, 788), (413, 10000),
    ];

    fn package(pulses: &[(u32, u32)]) -> Package {
        Package {
            pulses: pulses.iter().map(|&(mark, gap)| Pulse { mark, gap }).collect(),
            ..Default::default()
        }
    }

    #[test]
    fn a_remote_off_the_air_decodes() {
        let r = KeeLoq.decode_package(&package(&OFF_AIR)).expect("a KeeLoq frame");
        assert_eq!(r.get("serial"), Some(&Value::Int(0x01c4a39)));
        assert_eq!(r.get("btn"), Some(&Value::Int(2)));
        assert_eq!(r.get("hop"), Some(&Value::Int(0x697b4d73)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(false)));
        assert_eq!(r.get("repeat"), Some(&Value::Bool(true)));
        assert_eq!(r.crc_valid, None);
    }

    /// Encode a frame the way the chip sends it.
    fn frame(hop: u32, serial: u32, button: u32, vlow: bool, rpt: bool) -> Vec<(u32, u32)> {
        let mut p = vec![(1650, 150)];
        p.extend(std::iter::repeat((TE, TE)).take(11));
        p.push((TE, 10 * TE));
        let word: u128 = hop as u128
            | (serial as u128) << 32
            | (button as u128) << 60
            | (vlow as u128) << 64
            | (rpt as u128) << 65;
        for i in 0..66 {
            let bit = (word >> i) & 1 == 1;
            p.push(if bit { (TE, 2 * TE) } else { (2 * TE, TE) });
        }
        p.last_mut().unwrap().1 = 10_000;
        p
    }

    #[test]
    fn every_field_comes_back_where_it_was_put() {
        let r = KeeLoq.decode_package(&package(&frame(0x1234_5678, 0x0abc_def, 0x4, false, false))).unwrap();
        assert_eq!(r.get("hop"), Some(&Value::Int(0x1234_5678)));
        assert_eq!(r.get("serial"), Some(&Value::Int(0x0abc_def)));
        assert_eq!(r.get("btn"), Some(&Value::Int(4)));
        assert_eq!(r.get("battery_ok"), Some(&Value::Bool(true)));
        assert_eq!(r.get("repeat"), Some(&Value::Bool(false)));
    }

    #[test]
    fn the_frame_has_to_have_its_shape() {
        // The same bits without the header gap are one long row, and a
        // frame that is not on a row of its own behind the preamble is not
        // claimed however well its bits fit.
        let mut p = frame(0x1234_5678, 0x0abc_def, 0x4, false, false);
        p[12].1 = TE;
        assert_eq!(KeeLoq.decode_package(&package(&p)), Err(DecodeError::NotThisProtocol));
        // And a frame cut short is not one either.
        let short: Vec<(u32, u32)> = frame(0x1234_5678, 0x0abc_def, 0x4, false, false)[..40].to_vec();
        assert_eq!(KeeLoq.decode_package(&package(&short)), Err(DecodeError::NotThisProtocol));
    }
}
