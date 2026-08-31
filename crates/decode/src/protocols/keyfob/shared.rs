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

/// How many bits a single-frame package may exceed the frame by before it is
/// no longer treated as one frame and must show a repeat instead.
///
/// A real reception is one frame plus maybe a leading start bit and a little
/// slicer slop. The Flipper's streaming decoders count the start bit out of
/// the frame, but the slicer does not know it is a start bit, so it becomes a
/// bit here. Allowing a few bits of slop covers that; a burst twice as long
/// as the frame (which is what a mis-sliced other-protocol signal looks
/// like) is refused until it shows the same frame twice.
const ALONE_SLOP: usize = 8;

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
/// A checksum-free protocol has no integrity check to fall back on, so a
/// match must be corroborated: either the identical frame appears again one
/// frame later (what a real remote's repeat burst provides), or the package
/// is short enough that it is plainly one frame rather than some other
/// protocol's burst sliced at the wrong rate. That is the closest such a
/// protocol comes to not claiming every burst on the band.
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
    let alone = bits.len() <= want + ALONE_SLOP;
    for start in 0..=(bits.len() - want) {
        let frame = bits.slice(start, want);
        if !alone {
            let repeated = |at: usize| bits.slice(at, want) == frame;
            let corroborated = (start + 2 * want <= bits.len() && repeated(start + want))
                || (start >= want && repeated(start - want));
            if !corroborated {
                continue;
            }
        }
        if let Some(r) = parse(frame.as_padded_bytes()) {
            return Ok(r);
        }
    }
    Err(DecodeError::NotThisProtocol)
}

/// Reverse the order of the low `bits` of `value`, like the Flipper's
/// `subghz_protocol_blocks_reverse_key`. Several remotes transmit their serial
/// least-significant-bit first; this is what puts it back in the order a
/// printed label would use.
pub fn reverse_key(value: u64, bits: u32) -> u64 {
    value.reverse_bits() >> (64 - bits)
}

/// True when `code` could be a real fixed-code frame rather than a burst of
/// every bit the same.
///
/// A remote's DIP address is never constant, so an all-zero or all-one frame
/// is how a no-checksum protocol misreads some other signal sliced at the
/// wrong rate (a PPM weather sensor run through a keyfob's PWM timing comes
/// out as a run of one symbol) or a piece of noise. Rejecting degenerate
/// frames is the closest a checksum-free protocol comes to an integrity
/// check, and it is what keeps it from claiming every monotonic burst on the
/// band.
pub fn plausible(code: u64, bits: u32) -> bool {
    code != 0 && code != (1u64 << bits) - 1
}
