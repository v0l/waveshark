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

mod acurite;
mod bresser;
mod ev1527;
mod fineoffset;
mod globaltronics;
mod keyfob;
mod lacrosse;
mod nexus;
mod oregon;
mod rubicson;
mod security;
mod somfy_rts;
mod tpms;
mod x10;

pub use acurite::{Acurite609Txc, AcuriteTower, AcuriteWind};
pub use bresser::Bresser3Ch;
pub use ev1527::Ev1527;
pub use fineoffset::{FineOffsetWh1080, FineOffsetWh51};
pub use globaltronics::{GtWt02, GtWt03};
pub use keyfob::{Ansonic, Bett, came12_bit, came24_bit, Holtek, HoltekHt12x, Linear, LinearDelta3, NiceFlo, Princeton};
pub use lacrosse::{LacrosseIt, LacrosseTx141thBv2};
pub use nexus::NexusTh;
pub use oregon::{OregonV2, OregonV3};
pub use rubicson::Rubicson;
pub use security::HoneywellSecurity;
pub use somfy_rts::SomfyRts;
pub use tpms::{SchraderTpms, ToyotaTpms};
pub use x10::X10Rf;

use crate::bits::BitBuffer;

/// Find a frame satisfying `ok`, at any bit offset, with corroboration.
///
/// A burst holds the same frame many times over and the slicer starts wherever
/// the detector triggered, so the frame is neither at bit zero nor byte
/// aligned and has to be searched for. That search is also how a decoder
/// invents devices: an 8 bit checksum passes on one window in 256, and a five
/// hundred bit burst of noise offers five hundred windows. Two of them will
/// pass. Observed in the field as a LaCrosse sensor reporting 43.6 C at 5%
/// humidity in a British winter.
///
/// So a match must be corroborated, in one of the two ways a real burst
/// provides. Either the buffer holds nothing but this frame, meaning the
/// detector's own framing agrees with it, or the identical frame appears again
/// one frame later, which is what these sensors transmit: the same packet
/// three to twelve times back to back. Noise does neither.
pub(crate) fn find_frame(
    bits: &BitBuffer,
    bytes: usize,
    ok: impl FnMut(&[u8]) -> bool,
) -> Option<Vec<u8>> {
    find_frame_bits(bits, bytes * 8, ok)
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
    // Room for a whole second copy is what makes a repeat check meaningful;
    // below that the buffer is one frame plus slicer slop.
    let alone = bits.len() < want * 2;

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
        let corroborated = rows
            .iter()
            .any(|&at| at != start && bits.slice(at, want) == frame);
        // Or the rows themselves repeating at this frame's own period, which
        // is the same evidence without needing the copies to be identical.
        // Acurite's weather stations number their repeats, so no two copies in
        // a burst are ever the same and the test above cannot see a
        // transmission that is plainly periodic. The slack is for the sync
        // mark between repeats, which leaves the copies a bit further apart
        // than the frame is long.
        let periodic = rows
            .iter()
            .any(|&at| at != start && at.abs_diff(start).abs_diff(want) <= 2);
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
