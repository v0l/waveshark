use std::fmt;

/// What went wrong talking to a dongle
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("no RTL2832U device at that index")]
    NoDevice,
    #[error("the device is open elsewhere")]
    Busy,
    #[error("no permission to open the device: install the udev rules")]
    Permission,
    #[error("USB transfer failed: {0}")]
    Usb(String),
    #[error("{0}")]
    Tuner(String),
    #[error("{0}")]
    Unsupported(String),
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    pub(crate) fn usb(what: &str, e: impl fmt::Display) -> Self {
        Self::Usb(format!("{what}: {e}"))
    }
}
