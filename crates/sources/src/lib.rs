//! Sample sources: files, synthetic signals, and (via the driver crates) live
//! hardware. Also the sinks that take samples the other way, so a transmit
//! path can be exercised into a capture rather than into an antenna.

pub mod bench;
pub mod file;
pub mod sink;

pub use bench::FileRadio;
pub use file::{FileMeta, FileSource, parse_filename};
pub use sink::FileSink;
