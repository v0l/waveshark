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
use crate::protocols::find_frame_bits;
use crate::slicer::{Coding, Timing};

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
/// protocol either reads the same way (Ansonic, CAME) or the opposite way
/// (Princeton, Holtek, the rolling codes), which is what `invert` is for:
/// pass `true` when the on-air convention is short-mark `0` / long-mark `1`.
/// Either way, `parse` sees the on-air convention.
///
/// Corroboration (the identical frame again one frame later, or a short
/// enough buffer that the detector's own framing agrees) is what stops a
/// checksum-free protocol claiming every burst on the band; see
/// [`find_frame_bits`].
///
/// `parse` is called once per candidate window with the zero-padded frame
/// bytes and returns the report on success, `None` on any mismatch. The frame
/// bytes are taken from the exact window `find_frame_bits` matched, which is
/// why the search predicate and the parse share one closure.
pub fn find_and_parse(
    bits: &BitBuffer,
    frame_bits: usize,
    invert: bool,
    mut parse: impl FnMut(&[u8]) -> Option<Report>,
) -> Result<Report, DecodeError> {
    let bits = if invert { bits.inverted() } else { bits.clone() };
    let b = find_frame_bits(&bits, frame_bits, |b| parse(b).is_some())
        .ok_or(DecodeError::NotThisProtocol)?;
    parse(&b).ok_or(DecodeError::NotThisProtocol)
}

/// Reverse the order of the low `bits` of `value`, like the Flipper's
/// `subghz_protocol_blocks_reverse_key`. Several remotes transmit their serial
/// least-significant-bit first; this is what puts it back in the order a
/// printed label would use.
pub fn reverse_key(value: u64, bits: u32) -> u64 {
    value.reverse_bits() >> (64 - bits)
}
