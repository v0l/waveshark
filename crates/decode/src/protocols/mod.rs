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

mod alecto;
mod ert;
mod esl;
mod globaltronics;
mod hanshow;
mod hideki;
mod interlogix;
mod ism868_link;
pub mod keyfob;
mod oregon;
mod security;
mod somfy_rts;

pub use alecto::AlectoV1;
pub use ert::{ErtIdm, ErtScm, ErtScmPlus};
pub use esl::Esl;
pub use globaltronics::{GtWt02, GtWt03};
pub use hanshow::Hanshow;
pub use hideki::Hideki;
pub use interlogix::InterlogixSecurity;
pub use ism868_link::Ism868Link;
pub use keyfob::KeeLoq;
pub use oregon::{OregonV2, OregonV3};
pub use security::HoneywellSecurity;
pub use somfy_rts::SomfyRts;

use crate::bits::BitBuffer;

/// Could this buffer hold a frame whose row is `row_bits` long?
///
/// A row is where the slicer cut, which is where the transmitter stopped, so a
/// burst whose every row is a different length is a different protocol however
/// well a window inside it checksums. Several rtl_433 decoders test the row
/// length before anything else for exactly that reason.
///
/// A buffer with no cut in it is one row, which is what the detector says it
/// is: a burst it saw begin and end. That is the common case here, since the
/// gap between copies is usually long enough to end the package rather than
/// only the row.
pub(crate) fn rows_within(bits: &BitBuffer, row_bits: std::ops::RangeInclusive<usize>) -> bool {
    let starts: Vec<usize> = if bits.rows().is_empty() { vec![0] } else { bits.rows().to_vec() };
    let starts = &starts[..];
    let ends = starts.iter().skip(1).copied().chain(std::iter::once(bits.len()));
    starts.iter().copied().zip(ends).any(|(start, end)| row_bits.contains(&(end - start)))
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
        // Or the rows themselves repeating at this frame's own period, which
        // is the same evidence without needing the copies to be identical.
        // Acurite's weather stations number their repeats, so no two copies in
        // a burst are ever the same and the test above cannot see a
        // transmission that is plainly periodic. The slack is for the sync
        // mark between repeats, which leaves the copies a bit further apart
        // than the frame is long.
        let periodic = rows.iter().any(|&at| at != start && at.abs_diff(start).abs_diff(want) <= 2);
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
