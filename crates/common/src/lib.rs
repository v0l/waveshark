//! Core vocabulary shared by every layer of super-radio.
//!
//! Named `common` rather than `core` because a crate called `core` shadows the
//! Rust sysroot crate. Nothing in here does DSP or I/O: it defines sample
//! buffers, the device abstraction, tuning units, and errors.

pub mod device;
pub mod error;
pub mod iq;
pub mod pulse;
pub mod value;
pub mod units;

pub use device::{
    Device, DeviceInfo, DriverKind, GainMode, GainStage, RxStream, Toggle, TunerRange,
};
pub use error::{Error, Result};
pub use iq::{IqBuf, SampleFormat, C32};
pub use pulse::{Package, Packet, PacketBody, Pulse};
pub use value::Value;
pub use units::{Hz, Sps};
