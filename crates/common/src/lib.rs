//! Core vocabulary shared by every layer of waveshark.
//!
//! Named `common` rather than `core` because a crate called `core` shadows the
//! Rust sysroot crate. Nothing in here does DSP or I/O: it defines sample
//! buffers, the device abstraction, tuning units, and errors.

pub mod addr;
pub mod bands;
pub mod decode;
pub mod demod;
pub mod device;
pub mod error;
pub mod geohash;
pub mod iq;
pub mod modulation;
pub mod packet;
pub mod pulse;
pub mod rtl;
pub mod source;
pub mod units;
pub mod value;

pub use decode::{
    Airtime, CONTROL_CHANNELS, ChannelPlan, ChannelUse, Cpr, Decoded, Identity, Link, Party,
    PartyKind, Position, ReportDetail, Secrecy, SondeSensors, media,
};
pub use demod::{Demod, Passband};
pub use device::{
    Choice, Device, DeviceInfo, DriverKind, GainMode, GainStage, Number, RxStream, Toggle,
    TunerRange, Tuning, TxInfo, TxStream,
};
pub use error::{Error, Result};
pub use iq::{C32, IqBuf, SampleFormat};
pub use modulation::Modulation;
pub use pulse::{
    ANALOGUE, CHANNEL_MATCH_HZ, Cadence, Chroma, ConversationKey, Decoder, Frame, FrontEnd,
    IqBurst, Matrix, Measure, Over, Package, Packet, PacketBody, Phase, Pixels, Pulse, Range,
    SpectrumFrame, Speech, Update, VideoFrame, Voice, Yuv,
};
pub use source::{SourceBlock, SourceId, SourceState};
pub use units::{Hz, Sps};
pub use value::{Data, FieldType, Unit, Value};
