//! Core vocabulary shared by every layer of waveshark.
//!
//! Named `common` rather than `core` because a crate called `core` shadows the
//! Rust sysroot crate. Nothing in here does DSP or I/O: it defines sample
//! buffers, the device abstraction, tuning units, and errors.

pub mod decode;
pub mod device;
pub mod error;
pub mod iq;
pub mod modulation;
pub mod pulse;
pub mod source;
pub mod value;
pub mod units;

pub use device::{
    Choice, Device, DeviceInfo, DriverKind, GainMode, GainStage, RxStream, Toggle, TunerRange,
    TxInfo, TxStream,
};
pub use error::{Error, Result};
pub use iq::{IqBuf, SampleFormat, C32};
pub use decode::{
    media, Airtime, Cpr, Decoded, Identity, Link, Party, PartyKind, Position, ReportDetail,
};
pub use modulation::Modulation;
pub use pulse::{
    Frame, FrontEnd, IqBurst, Measure, Package, Packet, PacketBody, Pixels, Pulse, Speech, VideoFrame, Voice,
};
pub use source::{SourceBlock, SourceId, SourceState};
pub use value::Value;
pub use units::{Hz, Sps};
