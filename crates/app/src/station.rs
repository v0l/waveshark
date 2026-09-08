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

use parking_lot::RwLock;
use std::sync::OnceLock;

fn cell() -> &'static RwLock<gps::Source> {
    static GPS: OnceLock<RwLock<gps::Source>> = OnceLock::new();
    GPS.get_or_init(|| RwLock::new(gps::Source::start(gps::Config::new(gps::Transport::default()))))
}

/// Read from a named GPS, or `None` for the local gpsd the reader looks for
/// on its own. Restarting is what stops two readers fighting over one serial
/// port, so a transport that has not changed is left alone.
pub fn set_source(transport: Option<gps::Transport>) {
    let want = transport.unwrap_or_default();
    let mut g = cell().write();
    if *g.transport() != want {
        *g = gps::Source::start(gps::Config::new(want));
    }
}

/// The current position, or `None` when there has never been one or the last
/// one has gone stale.
pub fn fix() -> Option<gps::Fix> {
    cell().read().fix()
}

/// Whether the link is up, which is not the same as having a fix: a GPS
/// indoors is connected and lost.
pub fn connected() -> bool {
    cell().read().connected()
}

pub fn fixes() -> u64 {
    cell().read().fixes()
}

/// Satellites used and in view, which is the one thing worth showing while
/// there is no fix.
pub fn sky() -> Option<gps::Sky> {
    cell().read().sky()
}
