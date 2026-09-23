//! Device protocols.
//!
//! Each module is one family of devices. They share the slicers and the pulse
//! detector entirely, so the marginal cost of a protocol is its frame layout
//! and integrity check, not another DSP chain.
//!
//! Frame layouts and timings are transcribed from rtl_433, which is the only
//! description most of these devices have. Where its decoder applies a sanity
//! rule, that rule is here too: the rules are not cosmetic, they are what
//! stops a checksum-free protocol claiming every burst on the band.

mod ert;
mod esl;
mod globaltronics;
mod hanshow;
mod hideki;
mod interlogix;
pub mod keyfob;
mod oregon;
mod somfy_rts;

pub use ert::{ErtIdm, ErtScm, ErtScmPlus};
pub use esl::Esl;
pub use globaltronics::GtWt02;
pub use hanshow::Hanshow;
pub use hideki::Hideki;
pub use interlogix::InterlogixSecurity;
pub use keyfob::KeeLoq;
pub use oregon::OregonV2;
pub use somfy_rts::SomfyRts;

use crate::bits::BitBuffer;

/// Could this buffer hold a frame whose row is `row_bits` long?
///
/// A row is where the slicer cut, which is where the transmitter stopped, so a
/// burst whose every row is a different length is a different protocol however
/// well a window inside it checksums. Several rtl_433 decoders test the row
/// length before anything else for exactly that reason.
///
/// This asks whether any row is that long, which is the cheap question. The
/// row a frame was actually taken from is the one that has to be that long,
/// and [`row_len_at`] answers that where the caller knows the offset.
///
/// A buffer with no cut in it is one row, which is what the detector says it
/// is: a burst it saw begin and end. That is the common case here, since the
/// gap between copies is usually long enough to end the package rather than
/// only the row.
pub(crate) fn rows_within(bits: &BitBuffer, row_bits: std::ops::RangeInclusive<usize>) -> bool {
    row_lengths(bits).any(|(_, len)| row_bits.contains(&len))
}

/// The length of the row bit `at` falls in, for a decoder that searches
/// offsets rather than taking a row whole.
///
/// rtl_433 hands a decoder one row and it checks that row's length:
/// tpms_gm.c wants `num_rows == 1` and 130 bits before it looks at the
/// preamble. A search that accepts because some *other* row in the package
/// was the right length is not the same test, and that is how the
/// GM-Aftermarket description read a window of zeros out of an Oregon
/// RTGN318 burst.
pub(crate) fn row_len_at(bits: &BitBuffer, at: usize) -> usize {
    row_lengths(bits)
        .find(|(start, len)| at >= *start && at < start + len)
        .map(|(_, len)| len)
        .unwrap_or(0)
}

// A boundary list need not begin at zero: a burst sliced as one row can come
// back with a single mark at its end, and taking those marks as row starts
// then measures the empty tail after the last one and misses the row itself.
fn row_lengths(bits: &BitBuffer) -> impl Iterator<Item = (usize, usize)> + '_ {
    let mut starts: Vec<usize> = bits.rows().to_vec();
    if starts.first() != Some(&0) {
        starts.insert(0, 0);
    }
    let ends: Vec<usize> =
        starts.iter().skip(1).copied().chain(std::iter::once(bits.len())).collect();
    starts.into_iter().zip(ends).map(|(start, end)| (start, end.saturating_sub(start)))
}

/// [`find_frame`] for a frame whose length is not a whole number of bytes,
/// which is most of them: 36, 37 and 41 bit frames are all common. The bytes
/// handed to `ok` are zero padded on the right, as rtl_433's rows are.
///
/// Where the slicer found row boundaries those are tried first, because a row
/// starts where the transmitter stopped, and that is real evidence about
/// alignment rather than a guess. It matters more than it sounds: in a burst of
/// twelve copies every bit offset repeats at the row period, so a misaligned
/// window is corroborated exactly as well as the right one and only the
/// checksum stands between a six bit sum and an invented reading. Observed on
/// rtl_433's own GT-WT02 recording, which decoded as a different sensor at a
/// different temperature until the row starts were kept.
///
/// The scan over every offset stays as a fallback, for the packages a detector
/// hands over with no gap long enough to cut on, and there the repeat must sit
/// exactly one frame away. Loosening that to a copy anywhere in the buffer was
/// tried and reverted: a frame that is mostly zeros repeats at every offset,
/// and rtl_433's Nexus recording promptly decoded as an Acurite sensor
/// reading 0.0 C.
pub(crate) fn find_frame_bits(
    bits: &BitBuffer,
    want: usize,
    mut ok: impl FnMut(&[u8]) -> bool,
) -> Option<Vec<u8>> {
    if bits.len() < want {
        return None;
    }
    // A buffer barely longer than the frame is the detector agreeing with the
    // frame's own boundaries, and that is corroboration in itself. The margin
    // is a quarter of a frame rather than a whole one: at a whole frame's slack
    // a 52 bit buffer counts as holding one 37 bit frame alone, and rtl_433's
    // Acurite 606TX recording duly decoded as a Globaltronics sensor, the two
    // protocols being close enough in timing to slice the same way.
    let alone = bits.len() < want + want / 4;

    let rows: Vec<usize> = bits
        .rows()
        .iter()
        .copied()
        .chain(std::iter::once(0))
        .filter(|s| s + want <= bits.len())
        .collect();
    for &start in &rows {
        let frame = bits.slice(start, want);
        if !ok(frame.as_padded_bytes()) {
            continue;
        }
        // Another copy at another row start, which noise does not produce and
        // a misread row cannot fake.
        let corroborated = rows.iter().any(|&at| at != start && bits.slice(at, want) == frame);
        // Or a second row a frame's length away that reads as well, which is
        // the same evidence without needing the copies to be identical.
        // Acurite's weather stations number their repeats, so no two copies in
        // a burst are ever the same and the test above cannot see a
        // transmission that is plainly periodic. The slack is for the sync
        // mark between repeats, which leaves the copies a bit further apart
        // than the frame is long. The second row has to pass the check too:
        // band noise cuts rows at every spacing, and a bare period against a
        // byte-wide sum read a doorbell recording as an Acurite 609TXC.
        let mut periodic = false;
        for &at in &rows {
            if at != start
                && at.abs_diff(start).abs_diff(want) <= 2
                && ok(bits.slice(at, want).as_padded_bytes())
            {
                periodic = true;
                break;
            }
        }
        if alone || corroborated || periodic {
            return Some(frame.as_padded_bytes().to_vec());
        }
    }

    for start in 0..=(bits.len() - want) {
        let frame = bits.slice(start, want);
        if !ok(frame.as_padded_bytes()) {
            continue;
        }
        let repeated = |at: usize| bits.slice(at, want) == frame;
        if alone
            || (start + 2 * want <= bits.len() && repeated(start + want))
            || (start >= want && repeated(start - want))
        {
            return Some(frame.as_padded_bytes().to_vec());
        }
    }
    None
}
