mod status;

pub use status::Status;

#[cfg(feature = "kiwisdr")]
mod client;
#[cfg(not(feature = "kiwisdr"))]
#[path = "kiwisdr/offline.rs"]
mod client;

pub use client::*;
