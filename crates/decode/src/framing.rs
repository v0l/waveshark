//! Finding where a frame starts in a burst nobody claims.
//!
//! A slicer starts at whatever edge the detector triggered on, so the bits it
//! produces are at an arbitrary phase. For an unknown FSK device that is the
//! single biggest obstacle to reading anything: two receptions of the same
//! transmitter come out at different bit offsets, so their hex dumps share no
//! byte, and the sync word that would identify the device is smeared across
//! byte boundaries differently every time. The dump opens with ten bytes of
//! `55` and the interesting part begins mid-byte.
//!
//! Nearly every packet radio on the ISM bands opens with an alternating
//! preamble, because that is what a receiver needs to recover its clock and
//! set its slicing threshold. That preamble is the phase reference the burst
//! carries with it: cut at the end of the alternating run and the bits that
//! follow are byte-aligned to the transmitter's own framing, not to the
//! detector's trigger.
//!
//! The cut is deterministic rather than exact. Alternation does not stop the
//! instant the preamble does; it runs on into the sync word for as many bits
//! as the sync happens to keep alternating, up to three or four in practice.
//! That costs a fixed offset, the *same* fixed offset for every reception of
//! the same device, which is all that matters: two frames from one transmitter
//! now align with each other, and a shared sync word is visible as identical
//! leading bytes instead of having to be found by hand at eight bit offsets.

use crate::bits::BitBuffer;

/// Where the preamble was and what follows it.
#[derive(Clone, Debug, PartialEq)]
pub struct Framing {
    /// Bit offset of the first bit of the alternating run.
    pub start: usize,
    /// Length of the alternating run, in bits.
    pub preamble_bits: usize,
    /// The bits after the run, byte-aligned to the transmitter's framing.
    pub frame: BitBuffer,
    /// The same bits with [`ROLLBACK`] more in front, for a check that has its
    /// own way of telling whether it is aligned. The cut cannot see where the
    /// preamble stopped and an alternating sync word started, so a frame check
    /// with a CRC behind it should try each offset from here and let the CRC
    /// pick; a human reading the dump wants [`frame`](Self::frame).
    pub rolled_back: BitBuffer,
    /// Further frames in the same burst, each behind a preamble of its own.
    ///
    /// A transmitter that sends its message twice with a few tens of symbols
    /// between the copies produces one burst, not two, because the carrier
    /// never drops for long enough to close it. Reporting only the first copy
    /// throws away the second, and the second is the more interesting one: it
    /// is the same device a few milliseconds later, so whatever differs
    /// between the copies is a counter or a nonce and everything that does not
    /// is the header.
    pub repeats: Vec<BitBuffer>,
}

impl Framing {
    /// Bytes of the frame with the trailing padding removed.
    ///
    /// The slicer runs on past the end of the transmission and turns the
    /// silence into zeros, capped at the reset gap, so a 22 byte frame is
    /// reported as 47 bytes of which 25 are nothing. The real length is a
    /// device fingerprint and worth having; the padding is not, and reading a
    /// dump means counting past it.
    pub fn content_bytes(&self) -> usize {
        let b = self.frame.as_bytes();
        b.iter().rposition(|v| *v != 0).map_or(0, |i| i + 1)
    }

    /// The first four bytes after the preamble, the candidate sync word, as
    /// hex. Two receptions of one device agree here; two devices do not.
    pub fn sync_hex(&self) -> String {
        self.frame
            .as_bytes()
            .iter()
            .take(4)
            .map(|b| format!("{b:02x}"))
            .collect()
    }
}

/// Shortest alternating run worth calling a preamble.
///
/// Twelve bits is a byte and a half of `55`. Shorter runs happen by chance in
/// random payload data often enough to move the cut somewhere meaningless, and
/// no real preamble is that short: the shortest in common use is four bytes,
/// and CC1101-class parts default to four or eight.
pub const MIN_PREAMBLE_BITS: usize = 12;

/// Find the longest alternating run of at least `min_bits` and align to its
/// end.
///
/// `None` when the burst has no such run, which is the honest answer for a
/// coding whose preamble is not alternating (Manchester carries its own clock
/// and often opens on a constant level) and for a burst that is noise.
pub fn frame_from_preamble(bits: &BitBuffer, min_bits: usize) -> Option<Framing> {
    let (start, len) = longest_alternating_run(bits)?;
    if len < min_bits.max(2) {
        return None;
    }
    let after = start + len;
    if after >= bits.len() {
        return None;
    }
    // A repeat has to be a preamble, not a run of alternating payload. Half
    // the first preamble is the test: a transmitter sends the same preamble
    // before every copy, and payload that alternates for twenty or forty bits
    // does not happen in data that has been whitened or encrypted.
    let repeat_min = min_bits.max(len / 2);
    // Every other copy, before or after the one that was cut on: the longest
    // preamble is the primary, and there is no reason the transmitter's first
    // copy has to be the one whose preamble the detector caught most of.
    let repeats = runs(bits)
        .filter(|(at, n)| *at != start && *n >= repeat_min && at + n < bits.len())
        .map(|(at, n)| bits.slice(at + n, bits.len() - at - n))
        .collect();
    Some(Framing {
        start,
        preamble_bits: len,
        frame: bits.slice(after, bits.len() - after),
        rolled_back: bits.slice(after.saturating_sub(ROLLBACK), bits.len() - after + ROLLBACK.min(after)),
        repeats,
    })
}

/// How far before the cut the real frame might start.
///
/// The cut lands at the last bit that alternates, and the sync word carries on
/// alternating for as many of its leading bits as happen to. Three is the most
/// that is worth allowing for: a sync word alternating for four or more bits is
/// indistinguishable from more preamble, and every extra bit offered to a CRC
/// check is another chance for it to pass on nothing.
pub const ROLLBACK: usize = 3;

/// Offset and length of the longest run of bits that alternate with their
/// neighbour. Polarity does not matter: `0101` and `1010` are the same
/// preamble seen through opposite slicer polarity, and which one a burst
/// yields depends on nothing more than which tone the detector called the
/// mark.
fn longest_alternating_run(bits: &BitBuffer) -> Option<(usize, usize)> {
    let best = runs(bits).max_by_key(|(_, n)| *n)?;
    (best.1 >= 2).then_some(best)
}

/// Every maximal alternating run as (offset, length).
///
/// The longest is the preamble rather than the first, which was checked
/// against a day of 868 MHz bursts: taking the first run long enough moved the
/// cut onto a short alternation in the noise at the leading edge on five
/// bursts out of nine hundred and improved none of them.
fn runs(bits: &BitBuffer) -> impl Iterator<Item = (usize, usize)> + '_ {
    let mut run_start = 0usize;
    let mut i = 1usize;
    std::iter::from_fn(move || {
        while i < bits.len() {
            let boundary = bits.get(i) == bits.get(i - 1);
            i += 1;
            if boundary {
                let out = (run_start, i - 1 - run_start);
                run_start = i - 1;
                return Some(out);
            }
        }
        if run_start < bits.len() {
            let out = (run_start, bits.len() - run_start);
            run_start = bits.len();
            return Some(out);
        }
        None
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn buffer(bits: &[u8]) -> BitBuffer {
        let mut b = BitBuffer::new();
        for &v in bits {
            b.push(v != 0);
        }
        b
    }

    /// Bits of `bytes`, MSB first, offset by `phase` junk bits in front.
    fn shifted(phase: usize, bytes: &[u8]) -> BitBuffer {
        let mut b = BitBuffer::new();
        for i in 0..phase {
            b.push(i % 3 == 0);
        }
        for &byte in bytes {
            for i in (0..8).rev() {
                b.push(byte >> i & 1 != 0);
            }
        }
        b
    }

    #[test]
    fn the_same_frame_at_different_phases_aligns_the_same_way() {
        // The observation this exists for: one transmitter received twice, the
        // detector triggering at a different edge each time. Before alignment
        // the two dumps share no byte.
        let frame = [0x55u8, 0x55, 0x55, 0x55, 0x55, 0x48, 0xe9, 0xf7, 0x12, 0x34, 0x9a];
        let a = frame_from_preamble(&shifted(0, &frame), MIN_PREAMBLE_BITS).expect("a");
        let b = frame_from_preamble(&shifted(3, &frame), MIN_PREAMBLE_BITS).expect("b");
        assert_eq!(a.frame.as_bytes(), b.frame.as_bytes(), "phase changed the frame");
        assert_eq!(a.sync_hex(), b.sync_hex());
        // The cut runs into the sync for as many bits as it keeps
        // alternating, so what comes out is a fixed rotation of the sync, not
        // necessarily 48e9f7 itself. It has to be the *same* rotation both
        // times, and it has to be stable enough to compare devices by.
        assert!(!a.sync_hex().is_empty());
    }

    #[test]
    fn opposite_slicer_polarity_finds_the_same_preamble() {
        let frame = [0x55u8, 0x55, 0x55, 0x55, 0x48, 0xe9, 0xf7, 0x12];
        let up = shifted(0, &frame);
        let down = up.inverted();
        let a = frame_from_preamble(&up, MIN_PREAMBLE_BITS).expect("a");
        let b = frame_from_preamble(&down, MIN_PREAMBLE_BITS).expect("b");
        assert_eq!(a.preamble_bits, b.preamble_bits);
        assert_eq!(a.start, b.start);
    }

    #[test]
    fn a_short_chance_alternation_is_not_a_preamble() {
        // 0xa3 0x0f: six alternating bits at the front and nothing else. Far
        // too short to be a preamble, and cutting there would be worse than
        // not cutting at all.
        let b = shifted(0, &[0xa3, 0x0f, 0x00, 0xff]);
        assert_eq!(frame_from_preamble(&b, MIN_PREAMBLE_BITS), None);
    }

    #[test]
    fn a_preamble_with_nothing_after_it_yields_no_frame() {
        let b = shifted(0, &[0x55, 0x55, 0x55, 0x55]);
        assert_eq!(frame_from_preamble(&b, MIN_PREAMBLE_BITS), None);
    }

    #[test]
    fn a_burst_holding_two_copies_reports_both() {
        // A transmitter that repeats its message with a few tens of symbols
        // between the copies produces one burst, because the carrier never
        // drops long enough to close it. Both copies were seen this way on
        // 868.49 MHz, and the second carried a counter one higher than the
        // first, which is the whole reason it is worth keeping.
        let mut air = vec![0x55u8, 0x55, 0x55, 0x55, 0x55, 0x48, 0xe9, 0xf7, 0x12, 0x15];
        air.extend_from_slice(&[0x55, 0x55, 0x55, 0x55, 0x55, 0x48, 0xe9, 0xf7, 0x12, 0x16, 0, 0]);
        let f = frame_from_preamble(&shifted(0, &air), MIN_PREAMBLE_BITS).expect("framing");
        assert_eq!(f.repeats.len(), 1, "the second copy was dropped");
        let (a, b) = (f.frame.as_bytes(), f.repeats[0].as_bytes());
        assert_eq!(a[..4], b[..4], "the two copies disagree about the header");
        assert_ne!(a[4], b[4], "the counter that differs between copies was lost");
    }

    #[test]
    fn payload_that_alternates_for_a_while_is_not_a_second_copy() {
        // 0x55 in the middle of a payload is eight alternating bits and
        // nothing more. Calling that a preamble would report a second frame
        // made of the rest of the first one.
        let air = [0x55u8, 0x55, 0x55, 0x55, 0x55, 0x55, 0x48, 0xe9, 0x55, 0x12, 0x9a, 0x33];
        let f = frame_from_preamble(&shifted(0, &air), MIN_PREAMBLE_BITS).expect("framing");
        assert!(f.repeats.is_empty(), "invented {} repeats", f.repeats.len());
    }

    #[test]
    fn trailing_padding_does_not_count_towards_the_frame_length() {
        // The slicer turns the silence after the burst into zeros, capped at
        // the reset gap. A 22 byte frame came out as 47 bytes that way.
        let mut air = vec![0x55u8, 0x55, 0x55, 0x55, 0x48, 0xe9, 0xf7, 0x12];
        air.extend_from_slice(&[0; 12]);
        let f = frame_from_preamble(&shifted(0, &air), MIN_PREAMBLE_BITS).expect("framing");
        assert_eq!(f.content_bytes(), 4, "padding counted as content");
        assert!(f.frame.as_bytes().len() > 12, "the padding was thrown away, not just excluded");
    }

    #[test]
    fn the_longest_run_wins_over_an_earlier_short_one() {
        let mut bits = buffer(&[0, 1, 0, 1, 0, 0]);
        for _ in 0..20 {
            bits.push(true);
            bits.push(false);
        }
        for _ in 0..8 {
            bits.push(true);
        }
        let f = frame_from_preamble(&bits, MIN_PREAMBLE_BITS).expect("framing");
        assert_eq!(f.start, 5);
        assert!(f.preamble_bits >= 40, "found only {} bits", f.preamble_bits);
    }
}
