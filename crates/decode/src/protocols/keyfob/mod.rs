//! Keyfob remote-control protocols ported from the Flipper Zero firmware
//! (Momentum-Firmware, `lib/subghz/protocols`).
//!
//! These are the remotes you find on garage doors, gates and car alarm
//! fobs: short OOK PWM frames on 315/433/868 MHz with no checksum (so the
//! frame's own repeat count is the integrity check), or a rolling code whose
//! counter rolls on every press. They differ from the sensor protocols in
//! `..` only in being keyed by a button rather than a measurement.
//!
//! `shared` holds the helpers: a Flipper-style timing table constructor, the
//! bit-reversal the Flipper applies to several serial numbers, and
//! `find_and_parse`, the invert-and-search used by every fixed-code remote.
//!
//! Frame layouts and timings are transcribed from the Flipper's own
//! `SubGhzBlockConst` and decoder state machines; where a decoder reads the
//! opposite bit polarity from waveshark's slicer, `find_and_parse` is told
//! to invert.

pub mod shared;

mod ansonic;
mod bett;
mod came;
mod holtek;
mod holtek_ht12x;
mod keeloq;
mod linear;
mod linear_delta3;
mod nice_flo;
mod princeton;

pub use ansonic::Ansonic;
pub use bett::Bett;
pub use came::{came12_bit, came24_bit};
pub use holtek::Holtek;
pub use holtek_ht12x::HoltekHt12x;
pub use keeloq::KeeLoq;
pub use linear::Linear;
pub use linear_delta3::LinearDelta3;
pub use nice_flo::NiceFlo;
pub use princeton::Princeton;
