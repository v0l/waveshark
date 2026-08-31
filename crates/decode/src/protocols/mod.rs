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
mod x10;

pub use acurite::{Acurite609Txc, AcuriteTower};
pub use bresser::Bresser3Ch;
pub use ev1527::Ev1527;
pub use fineoffset::{FineOffsetWh1080, FineOffsetWh51};
pub use globaltronics::{GtWt02, GtWt03};
pub use keyfob::{Ansonic, Bett, came12_bit, came24_bit, Holtek, HoltekHt12x, Linear, NiceFlo, Princeton};
pub use lacrosse::{LacrosseIt, LacrosseTx141thBv2};
pub use nexus::NexusTh;
pub use oregon::OregonV3;
pub use rubicson::Rubicson;
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
/// The frame length is also the repeat spacing, so a trailing stop bit has to
/// be counted in it or the repeat check looks one bit off and finds nothing.
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
