//! Error types for HackRF operations.

use thiserror::Error;

/// Result type for HackRF operations.
pub type Result<T> = std::result::Result<T, Error>;

/// Error codes matching libhackrf enum values (for cross-compatibility).
///
/// Reference: hackrf.h `enum hackrf_error`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum HackRfErrorCode {
    /// Operation successful.
    Success = 0,
    /// True (for boolean returns).
    True = 1,
    /// Parameter invalid.
    InvalidParam = -2,
    /// No HackRF device found.
    NotFound = -5,
    /// Device is busy.
    Busy = -6,
    /// Out of memory.
    NoMem = -11,
    /// LibUSB error.
    Libusb = -1000,
    /// Thread error.
    Thread = -1001,
    /// Streaming thread error.
    StreamingThreadErr = -1002,
    /// Streaming stopped.
    StreamingStopped = -1003,
    /// Streaming exit called.
    StreamingExitCalled = -1004,
    /// USB API version not supported.
    UsbApiVersion = -1005,
    /// Other error.
    Other = -9999,
}

impl HackRfErrorCode {
    /// Get a human-readable name for the error code.
    pub fn name(self) -> &'static str {
        match self {
            HackRfErrorCode::Success => "HACKRF_SUCCESS",
            HackRfErrorCode::True => "HACKRF_TRUE",
            HackRfErrorCode::InvalidParam => "HACKRF_ERROR_INVALID_PARAM",
            HackRfErrorCode::NotFound => "HACKRF_ERROR_NOT_FOUND",
            HackRfErrorCode::Busy => "HACKRF_ERROR_BUSY",
            HackRfErrorCode::NoMem => "HACKRF_ERROR_NO_MEM",
            HackRfErrorCode::Libusb => "HACKRF_ERROR_LIBUSB",
            HackRfErrorCode::Thread => "HACKRF_ERROR_THREAD",
            HackRfErrorCode::StreamingThreadErr => "HACKRF_ERROR_STREAMING_THREAD_ERR",
            HackRfErrorCode::StreamingStopped => "HACKRF_ERROR_STREAMING_STOPPED",
            HackRfErrorCode::StreamingExitCalled => "HACKRF_ERROR_STREAMING_EXIT_CALLED",
            HackRfErrorCode::UsbApiVersion => "HACKRF_ERROR_USB_API_VERSION",
            HackRfErrorCode::Other => "HACKRF_ERROR_OTHER",
        }
    }
}

/// Errors that can occur during HackRF operations.
#[derive(Debug, Error)]
pub enum Error {
    /// Failed to open USB device.
    #[error("failed to open USB device: {0}")]
    OpenFailed(#[source] nusb::Error),

    /// Failed to claim USB interface.
    #[error("failed to claim USB interface: {0}")]
    ClaimFailed(#[source] nusb::Error),

    /// Control transfer failed.
    #[error("control transfer failed: {0}")]
    ControlTransfer(#[source] nusb::transfer::TransferError),

    /// Bulk transfer failed.
    #[error("bulk transfer failed: {0}")]
    BulkTransfer(#[source] nusb::transfer::TransferError),

    /// No HackRF device found.
    #[error("no HackRF device found")]
    DeviceNotFound,

    /// Device configuration failed.
    #[error("configuration failed: {0}")]
    ConfigFailed(String),

    /// Invalid response from device.
    #[error("invalid device response: {0}")]
    InvalidResponse(String),

    /// Streaming error.
    #[error("streaming error: {0}")]
    StreamingError(String),

    /// Timeout waiting for device response.
    #[error("device timeout")]
    Timeout,
}
