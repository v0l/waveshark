//! Protocol decoding, layered the way rtl_433 layers it.
//!
//! ```text
//!   IQ -> envelope    -> OokDetector -> Package (mark/gap timings)
//!   IQ -> discriminator -> FskDetector -^
//!                                       |
//!                                    slicer -> BitBuffer
//!                                       |
//!                                   Protocol::decode -> Report
//! ```
//!
//! The expensive DSP happens once per channel. Everything protocol-specific
//! operates on integers and costs almost nothing, which is what makes running
//! every known protocol against every detected burst affordable.

pub mod adsb;
pub mod ais;
pub mod analyze;
pub mod bits;
pub mod protocol;
pub mod protocols;
pub mod slicer;

pub use analyze::{analyze, Analysis};
pub use bits::BitBuffer;
pub use protocol::{DecodeError, Protocol, Protocols, Report, Value};
pub use slicer::{slice, Coding, SliceError, Timing};
