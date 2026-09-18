//! Shared encoders for the keyfob remotes: the write side of the timing
//! tables the decoders read.
//!
//! Each function turns a code into a [`Package`] of PWM timings shaped the
//! way the Flipper's own encoders (`lib/subghz/protocols/*.c`) shape them,
//! including the header gap, start bit and guard gap a real remote sends.
//! A package here round-trips: decoding it with the protocol beside this
//! function recovers the code it was built from, which is the test each one
//! carries.

use crate::bits::BitBuffer;
use crate::slicer::{Coding, Timing};
use common::pulse::{Package, Pulse};
use std::time::Duration;

/// Inter-frame silence, so a burst of repeats splits into packages the way
/// the receiver's burst detector expects and `find_and_parse` corroborates
/// the frame across.
pub const INTER_FRAME_GAP_US: u32 = 10_000;

/// A PWM table to encode with. The decoders take a `Timing` with a coding;
/// an encoder wants the widths and the reset only.
pub fn frame(t: Timing, bits: &BitBuffer, repeats: usize) -> Package {
    let Coding::Pwm = t.coding else {
        panic!("keyfob encoders encode PWM tables only");
    };
    let mut pkg = Package::default();
    for r in 0..repeats {
        for i in 0..bits.len() {
            // Short mark is a 1, long mark a 0: the on-air convention the
            // decoders read through `find_and_parse`, and the one every
            // Flipper test recording confirms.
            let one = bits.get(i) == Some(true);
            let mark = if one { t.short_us } else { t.long_us };
            let gap = if one { t.long_us } else { t.short_us };
            pkg.pulses.push(Pulse { mark, gap });
        }
        let last = pkg.pulses.last_mut().expect("a frame has bits");
        last.gap = if r + 1 == repeats { t.reset_us } else { INTER_FRAME_GAP_US };
    }
    pkg
}

/// Silence a `Duration` long, as one pulse with no carrier.
pub fn silence(d: Duration) -> Pulse {
    Pulse { mark: 0, gap: d.as_micros().min(u32::MAX as u128) as u32 }
}

/// Repeat a package `n` times, separated by `gap`.
pub fn repeated(pkg: &Package, n: usize, gap: Duration) -> Package {
    let mut out = Package::default();
    for r in 0..n {
        out.pulses.extend(pkg.pulses.iter().copied());
        if r + 1 < n {
            if let Some(last) = out.pulses.last_mut() {
                last.gap = last.gap.max(gap.as_micros() as u32);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{Protocol, Value};
    use crate::slicer::slice;

    fn bits_of(s: &str) -> BitBuffer {
        let mut b = BitBuffer::with_capacity(s.len());
        for c in s.chars() {
            b.push(c == '1');
        }
        b
    }

    #[test]
    fn a_frame_round_trips_through_the_slicer() {
        let t = Timing::pwm(320, 640, 2500);
        let pkg = frame(t, &bits_of("101100101111"), 3);
        assert_eq!(pkg.pulses.len(), 36, "twelve bits, three repeats");
        // Last gap is the reset, middle ones the inter-frame gap.
        assert_eq!(pkg.pulses.last().unwrap().gap, 2500);
        let sliced = slice(&pkg, &t).expect("our own frame must slice");
        // Three identical frames tile the buffer end to end, which is what
        // `find_and_parse` corroborates a checksum-free protocol with.
        // 12 bits pads to two bytes in the hex.
        for r in 0..3 {
            assert_eq!(sliced.slice(r * 12, 12).to_hex(), "b2 f0");
        }
    }

    #[test]
    fn repeat_joins_are_gap_not_carrier() {
        let t = Timing::pwm(500, 1500, 2500);
        let pkg = frame(t, &bits_of("10"), 2);
        assert_eq!(pkg.pulses[1].gap, INTER_FRAME_GAP_US);
        assert_eq!(pkg.pulses[3].gap, t.reset_us);
    }

    #[test]
    fn short_mark_is_one() {
        // Bit order and polarity in one: 0b10 is a short mark then a long
        // mark, per the convention every Flipper corpus recording confirms.
        let t = Timing::pwm(400, 1200, 3000);
        let pkg = frame(t, &bits_of("10"), 1);
        assert_eq!(pkg.pulses[0].mark, 400);
        assert_eq!(pkg.pulses[0].gap, 1200);
        assert_eq!(pkg.pulses[1].mark, 1200);
        assert_eq!(pkg.pulses[1].gap, 3000);
    }

    /// The princeton key from the Flipper's own unit tests
    /// (`assets/unit_tests/subghz/princeton.sub`: TE 400, key 0x95d5d4),
    /// encoded and decoded back. What this encoder must produce is the
    /// shape a real remote sent, and the corpus recording is that shape.
    #[test]
    fn the_flipper_corpus_princeton_key_round_trips() {
        let p = crate::script::named("Princeton").unwrap();
        let mut bits = BitBuffer::with_capacity(24);
        for i in 0..24 {
            bits.push(0x95_d5_d4 & (1 << (23 - i)) != 0);
        }
        let pkg = frame(p.timing(), &bits, 3);
        let r = p.decode_package(&pkg).expect("the corpus key round-trips");
        // The decoder inverts the buffer before parsing (short mark 0 on
        // the air), so the code it reports is the key's complement.
        assert_eq!(r.get("code"), Some(&Value::Int(0x6a_2a_2b)));
        assert_eq!(r.get("serial"), Some(&Value::Int(0x6a_2a_2)));
        assert_eq!(r.get("btn"), Some(&Value::Int(0xb)));
    }
}
