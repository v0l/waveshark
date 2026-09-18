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

pub mod acars;
pub mod adsb;
pub mod ais;
pub mod analyze;
pub mod aprs;
pub mod apt;
pub mod ax25;
pub mod bds;
pub mod bits;
pub mod ble;
pub mod ccsds;
pub mod channel_keys;
pub(crate) mod crypto;
pub mod dab;
pub mod dfm;
pub mod dmr;
pub mod dmr_bp;
pub mod droneid;
pub mod dtmf;
pub mod dvbt;
pub mod eas;
pub mod elrs;
pub mod epirb;
pub mod flex;
pub mod flysky;
pub mod framing;
pub mod frsky;
pub mod geo;
#[cfg(feature = "tea")]
pub mod gpu;
pub mod gsm;
pub mod ieee802154;
pub mod imet;
pub mod inmarsat;
pub mod jpeg;
#[cfg(feature = "tea")]
pub mod keystream;
pub mod linescan;
pub mod lms6;
pub mod lora;
pub mod lora_li;
pub mod lorawan;
pub mod lrpt;
pub mod m10;
pub mod m17;
pub mod mdc1200;
#[cfg(feature = "ffmpeg")]
pub mod media;
pub mod meisei;
pub mod meshcore;
pub mod meshtastic;
pub mod morse;
pub mod mpegts;
pub mod mrz;
pub mod nrf24;
pub mod odid;
pub mod p25;
pub mod pocsag;
pub mod protocol;
pub mod protocols;
#[cfg(feature = "tea")]
pub mod recover;
pub mod rs;
pub mod rs41;
pub mod rtty;
pub mod slicer;
pub mod sstv;
pub mod subghz;
#[cfg(feature = "tea")]
pub mod ta61;
#[cfg(feature = "tea")]
pub mod tea;
pub mod tetra;
#[cfg(feature = "ffmpeg")]
pub mod transcode;
pub mod twotone;
pub mod uat;
pub mod vdl2;
pub mod video_channels;
pub mod vocoder;
pub mod voice;
pub mod wefax;
pub mod whiten;
pub mod wifi;
pub mod wmbus;
pub mod zwave;

pub use analyze::{Analysis, analyze};
pub use bits::BitBuffer;
pub use framing::Framing;
pub use protocol::{DecodeError, Protocol, Protocols, Report, Value};
pub use slicer::{Coding, SliceError, Timing, slice};
