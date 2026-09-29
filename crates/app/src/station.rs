//! Where the receiver is, read for as long as the program runs.
//!
//! The GPS is not a property of the radio. It was owned by the radio thread,
//! which meant no fixes before a device was chosen, none while one was being
//! swapped, and a settings pane that said "no radio running" where the
//! position should be, on a machine with a gpsd sitting there answering.
//!
//! So it lives here: one reader, started the first time anything asks, and
//! replaced only when an operator names a different GPS. Both the interface
//! and the radio thread read the same fix from it, and the survey and the
//! tracker are told the station position by whoever is holding them.

#[cfg(feature = "gps")]
#[path = "station/receiver.rs"]
mod imp;
#[cfg(not(feature = "gps"))]
#[path = "station/fixed.rs"]
mod imp;

pub use imp::{connected, fix, fixes, set_source, sky};
