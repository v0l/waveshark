//! RTL2832U and R82xx over USB, with no system library.
//!
//! A port of librtlsdr's register path: the demodulator in [`rtl2832`], the
//! Rafael Micro tuner in [`r82xx`]. The GPL of that code carries over, so this
//! crate is GPL-2.0-or-later like the rest of the tree.
//!
//! Controls stay usable while a [`rtl2832::Reader`] runs, as with librtlsdr:
//! register writes go to endpoint 0 and samples come off the bulk endpoint.

mod error;
mod gains;
mod r82xx;
mod rtl2832;
mod transport;

pub use error::{Error, Result};
pub use r82xx::{Board, Chip};
pub use rtl2832::{DEF_RTL_XTAL, DirectSampling, Enumerated, Reader, RtlSdr, Tuner};
