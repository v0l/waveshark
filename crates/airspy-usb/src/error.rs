use std::fmt;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("no Airspy at that index")]
    NoDevice,
    #[error("the Airspy is open elsewhere")]
    Busy,
    #[error("no permission to open the Airspy: install the udev rules")]
    Permission,
    #[error("USB transfer failed: {0}")]
    Usb(String),
    #[error("the Airspy stream was stopped")]
    Stopped,
}

pub type Result<T> = std::result::Result<T, Error>;

impl Error {
    pub(crate) fn usb(what: &str, e: impl fmt::Display) -> Self {
        Self::Usb(format!("{what}: {e}"))
    }
}
