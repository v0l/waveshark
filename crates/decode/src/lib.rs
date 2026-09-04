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
pub mod aprs;
pub mod ax25;
pub mod analyze;
pub mod bds;
pub mod bits;
pub mod framing;
pub mod m17;
#[cfg(feature = "tea")]
pub mod gpu;
#[cfg(feature = "tea")]
pub mod keystream;
pub mod lora;
#[cfg(feature = "tea")]
pub mod recover;
#[cfg(feature = "tea")]
pub mod ta61;
#[cfg(feature = "tea")]
pub mod tea;
pub mod vocoder;
pub mod voice;
pub mod pocsag;
pub mod protocol;
pub mod protocols;
pub mod slicer;
pub mod tetra;
pub mod whiten;
pub mod wmbus;

pub use analyze::{analyze, Analysis};
pub use bits::BitBuffer;
pub use framing::Framing;
pub use protocol::{DecodeError, Protocol, Protocols, Report, Value};
pub use slicer::{slice, Coding, SliceError, Timing};
