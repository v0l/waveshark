//! Shared machinery for keyfob remotes ported from the Flipper Zero firmware.
//!
//! Almost every fixed-code keyfob remote is PWM with a short mark for one
//! symbol and a long mark for the other, transmitted as a short burst of
//! identical frames behind a long preamble. The Flipper's streaming decoders
//! each re-implement the same edge walker; here that is the slicer, and a
//! protocol is a timing table plus a frame parser, exactly as for the sensor
//! protocols. What the keyfobs add is polarity: the Flipper reads a short
//! mark as `0` and a long mark as `1`, which is the opposite of what
//! [`crate::slicer`] produces, so the whole buffer is inverted before parsing.

use crate::bits::BitBuffer;
use crate::protocol::{DecodeError, Report};
use crate::slicer::{Coding, Timing};

/// How many bits of a package may sit outside the frames that tile it.
///
/// A real reception is one frame, or a train of identical frames, plus the
/// start bit the Flipper's streaming decoders count outside the frame and the
/// slicer cannot know to drop. That is one bit, and this allows two.
///
/// It has to stay this tight. At eight bits a 24-bit frame could be claimed
/// by a 12-bit protocol reading the middle of it, with six spare bits at each
/// end, which is the same mistake in miniature as reading a 12-bit code out
/// of a 64-bit KeeLoq frame.
const SLOP: usize = 2;

/// Build a PWM timing table from a Flipper `SubGhzBlockConst`.
///
/// `te_short`/`te_long` carry over unchanged. `reset_us` is the silence that
/// separates one frame from the next; on the air it is the remote's guard
/// time, and it is what lets the burst detector split a repeat train into
/// frame-aligned packages.
pub fn pwm(short_us: u32, long_us: u32, reset_us: u32) -> Timing {
    Timing { coding: Coding::Pwm, short_us, long_us, sync_us: 0, tolerance_us: 0, reset_us }
}

/// Find a `frame_bits`-wide frame that `parse` accepts, then return what
/// `parse` produced for it.
///
/// The slicer emits a short mark as `1` and a long mark as `0`. A Flipper
/// protocol either reads the same way (Ansonic, Linear Delta3) or the
/// opposite way (Princeton, Holtek, CAME, the rest), which is what `invert`
/// is for: pass `true` when the on-air convention is short-mark `0` /
/// long-mark `1`. Either way, `parse` sees the on-air convention.
///
/// A checksum-free protocol has no integrity check to fall back on, so the
/// package itself has to corroborate the match: the frame must *tile* it, end
/// to end, which is what a remote transmits. One frame is a package the
/// length of a frame; a repeat burst is a package that is a whole number of
/// identical frames.
///
/// Requiring only that the frame appear twice somewhere is not enough, and
/// the difference is not theoretical. Measured on a Flipper Zero sending a
/// 64-bit KeeLoq frame, six fixed-code protocols claimed it at once by
/// matching a 12 or 24-bit window somewhere inside the hop code. Tiling
/// refuses all of them: a 64-bit package is not a whole number of 12-bit
/// frames, and its windows are not identical.
///
/// Tiling says nothing about a frame of one repeated symbol, since a constant
/// run tiles anything. That is what [`plausible`] is for, and between them
/// they are as close as a protocol with no integrity field gets to one.
///
/// `parse` is called once per candidate window with the zero-padded frame
/// bytes and returns the report on success, `None` on any mismatch.
pub fn find_and_parse(
    bits: &BitBuffer,
    frame_bits: usize,
    invert: bool,
    mut parse: impl FnMut(&[u8]) -> Option<Report>,
) -> Result<Report, DecodeError> {
    let bits = if invert { bits.inverted() } else { bits.clone() };
    let want = frame_bits;
    if bits.len() < want {
        return Err(DecodeError::NotThisProtocol);
    }
    for start in 0..=(bits.len() - want) {
        let frame = bits.slice(start, want);
        if !tiles(&bits, start, &frame) {
            continue;
        }
        if let Some(r) = parse(frame.as_padded_bytes()) {
            return Ok(r);
        }
    }
    Err(DecodeError::NotThisProtocol)
}

/// Whether `frame`, found at `start`, repeats end to end across the package.
///
/// Anything left over at either end has to be inside [`SLOP`], which covers a
/// start bit the slicer could not know to drop and a trailing partial symbol.
fn tiles(bits: &BitBuffer, start: usize, frame: &BitBuffer) -> bool {
    let want = frame.len();
    // Whole frames before this one, and after it.
    let mut lead = start;
    while lead >= want && bits.slice(lead - want, want) == *frame {
        lead -= want;
    }
    let mut tail = start + want;
    while tail + want <= bits.len() && bits.slice(tail, want) == *frame {
        tail += want;
    }
    lead <= SLOP && bits.len() - tail <= SLOP
}

/// Reverse the order of the low `bits` of `value`, like the Flipper's
/// `subghz_protocol_blocks_reverse_key`. Several remotes transmit their serial
/// least-significant-bit first; this is what puts it back in the order a
/// printed label would use.
pub fn reverse_key(value: u64, bits: u32) -> u64 {
    value.reverse_bits() >> (64 - bits)
}

/// The fewest symbol changes a frame must contain to be believed.
///
/// Measured on live 433 MHz traffic: a strong continuous transmission read
/// through a keyfob's PWM timing slices into a run of one symbol with a
/// single flip where the burst started, and three protocols claimed the same
/// burst at once with codes of `0x800`, `0x1FF` and `0x7F`. Every one of
/// those has one or two transitions. A DIP-switch address that reads as a
/// single run is possible, and without a checksum it is indistinguishable
/// from that carrier, so it is refused: a wrong decode presented as a gate
/// remote is worse than a missed one.
const MIN_TRANSITIONS: u32 = 3;

/// True when `code` could be a real fixed-code frame rather than one symbol
/// repeated.
///
/// Rejecting degenerate frames is the closest a checksum-free protocol comes
/// to an integrity check, and it is what keeps it from claiming every
/// monotonic burst on the band. An all-zero or all-one frame is the obvious
/// case; the one that actually bit was a frame that is *nearly* constant,
/// which the corroboration in [`find_and_parse`] cannot catch either, because
/// a run of one symbol trivially equals itself a frame later.
pub fn plausible(code: u64, bits: u32) -> bool {
    let mask = if bits >= 64 { u64::MAX } else { (1u64 << bits) - 1 };
    let code = code & mask;
    if code == 0 || code == mask {
        return false;
    }
    // Adjacent bits that differ, counted across the frame only.
    let changes = ((code ^ (code >> 1)) & (mask >> 1)).count_ones();
    changes >= MIN_TRANSITIONS
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Report;

    #[test]
    fn a_frame_that_is_one_symbol_repeated_is_not_a_remote() {
        // The four readings a continuous 433.87 MHz carrier produced on the
        // air, each claimed by a different manufacturer's protocol within the
        // same millisecond: Nice-Flo 0x800, Linear 0x1FF, Linear-Delta3 0x7F
        // and Ansonic 0xBFF. Every one is a run of one symbol with the flip
        // where the burst began.
        assert!(!plausible(0x800, 12), "Nice-Flo code=2048");
        assert!(!plausible(0x1ff, 10), "Linear code=511");
        assert!(!plausible(0x7f, 8), "Linear-Delta3 cnt=127");
        assert!(!plausible(0xbff, 12), "Ansonic code=3071");
    }

    /// A 64-bit KeeLoq frame as the slicer delivers it: a 32-bit encrypted
    /// hop code, a 28-bit serial and a 4-bit button. The hop code is the
    /// output of a block cipher, so it is the arbitrary bit pattern that
    /// makes a short fixed-code frame match somewhere inside it.
    fn keeloq(hop: u32, serial: u32, btn: u8) -> BitBuffer {
        let data = ((btn as u64 & 0xf) << 60)
            | ((serial as u64 & 0x0fff_ffff) << 32)
            | hop as u64;
        let mut b = BitBuffer::new();
        for i in 0..64 {
            b.push(data & (1 << (63 - i)) != 0);
        }
        b
    }

    #[test]
    fn a_long_frame_is_not_carved_up_by_the_short_fixed_code_protocols() {
        // Measured on a Flipper Zero transmitting KeeLoq on 433.87 MHz: six
        // protocols claimed the same burst, reporting codes that were windows
        // into the hop code. None of them can tile a 64-bit package, and that
        // is what refuses them.
        let bits = keeloq(0x5c1d_2f83, 0x0a5b_c31, 0x2);
        for frame_bits in [8usize, 10, 12, 18, 24] {
            let got = find_and_parse(&bits, frame_bits, false, |b| {
                let mut v = 0u64;
                for (i, byte) in b.iter().enumerate() {
                    v |= (*byte as u64) << (8 * (b.len() - 1 - i));
                }
                v >>= (8 * b.len()) as u32 - frame_bits as u32;
                plausible(v, frame_bits as u32).then(|| Report::new("test"))
            });
            assert!(
                got.is_err(),
                "a {frame_bits}-bit protocol claimed part of a 64-bit frame"
            );
        }
    }

    #[test]
    fn a_repeat_train_still_decodes() {
        // The other half of the same rule: a remote sending its frame three
        // times must not be refused for being longer than one frame.
        let mut bits = BitBuffer::new();
        for _ in 0..3 {
            for b in [true, false, true, false, true, true, false, false, true, true, false, true]
            {
                bits.push(b);
            }
        }
        let got = find_and_parse(&bits, 12, false, |_| Some(Report::new("test")));
        assert!(got.is_ok(), "three copies of one frame is exactly what a remote sends");
    }

    #[test]
    fn the_obvious_degenerate_frames_are_still_refused() {
        for bits in [8u32, 12, 24] {
            assert!(!plausible(0, bits));
            assert!(!plausible((1u64 << bits) - 1, bits));
        }
    }

    #[test]
    fn a_real_dip_switch_address_still_decodes() {
        // Addresses off real remotes, which is what the guard must not cost:
        // an alternating pattern, a mixed one, and a 24-bit Princeton code
        // with its button nibble.
        assert!(plausible(0xabc, 12));
        assert!(plausible(0x155, 10));
        assert!(plausible(0x5a, 8));
        assert!(plausible(0xa1_3f_08, 24));
    }

    #[test]
    fn a_single_run_of_ones_is_refused_however_long_the_frame() {
        // Two transitions, which is what a carrier chopped at both ends of a
        // package looks like. A DIP address that reads this way is possible
        // and cannot be told from the carrier without a checksum, so the
        // reading is given up rather than guessed.
        assert!(!plausible(0b0000_1111_0000, 12));
        assert!(!plausible(0b0011_1111_1100, 12));
        assert!(plausible(0b0000_1111_0100, 12), "three transitions is enough");
    }
}
