use crate::units::{Hz, Sps};

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("no device matched the requested selector")]
    NoDevice,

    #[error("device is already in use by another process")]
    Busy,

    #[error("permission denied opening device (missing udev rule?)")]
    Permission,

    #[error("frequency {req} outside tuner range {lo}..={hi}")]
    FreqOutOfRange { req: Hz, lo: Hz, hi: Hz },

    #[error("sample rate {req} not supported by this device")]
    RateUnsupported { req: Sps },

    #[error("device disconnected mid-stream")]
    Disconnected,

    #[error("stream overrun: {dropped} samples dropped")]
    Overrun { dropped: u64 },

    #[error("this device cannot transmit")]
    TxUnsupported,

    #[error("cannot transmit while receiving on a half duplex radio")]
    HalfDuplexBusy,

    #[error("unsupported tuner: {0}")]
    UnsupportedTuner(String),

    #[error("usb transport: {0}")]
    Usb(String),

    #[error(transparent)]
    Io(#[from] std::io::Error),

    #[error("{0}")]
    Other(String),
}

impl Error {
    pub fn other(msg: impl Into<String>) -> Self {
        Self::Other(msg.into())
    }

    /// Whether retrying the same operation could plausibly succeed.
    pub fn is_transient(&self) -> bool {
        matches!(self, Self::Busy | Self::Overrun { .. })
    }
}
