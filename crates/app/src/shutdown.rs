//! Ctrl-C and `kill` close the window instead of killing the process.
//!
//! A receiver is usually stopped from the terminal it was started in, and the
//! default action for SIGINT and SIGTERM tears the process down where it
//! stands: the session file keeps whatever it had two seconds ago, the packet
//! log loses whatever was still buffered, and the radio thread never gets to
//! release its USB claim, which is what leaves the next process unable to
//! open the device at all.
//!
//! So a signal is turned into the same request the window's close button
//! makes. The shutdown path that is already tested is then the only one, and
//! `Drop` runs for the radio and the log.
//!
//! A second signal is left to the default action. Waiting on a hung shutdown
//! with no way out is worse than losing the last of the log, and by then the
//! caller has said twice what they want.

use std::sync::atomic::{AtomicBool, Ordering};

#[cfg(unix)]
#[path = "shutdown/unix.rs"]
mod platform;
#[cfg(not(unix))]
#[path = "shutdown/other.rs"]
mod platform;

pub use platform::install;

static ASKED: AtomicBool = AtomicBool::new(false);

/// Has a signal asked the receiver to stop?
pub fn asked() -> bool {
    ASKED.load(Ordering::Relaxed)
}
