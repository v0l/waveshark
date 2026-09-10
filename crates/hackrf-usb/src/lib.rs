//! Pure-Rust USB transport for HackRF One, receive and transmit.
//!
//! Vendored from `rs-hackrf` 0.4.2 by Xavier Olive (MIT), which covers
//! discovery, control transfers and RX streaming. What is added here is the
//! transmit half: TXVGA gain, `TRANSCEIVER_MODE_TRANSMIT`, the bulk OUT
//! endpoint and a writer that keeps transfers queued ahead of the radio.
//!
//! It lives in the workspace rather than as a dependency because half duplex
//! switching needs both halves to share one interface claim: a HackRF that is
//! still claimed for RX cannot be reopened for TX, and releasing the claim on
//! every mode switch is where the races are.
//!
//! # Sample format
//!
//! Interleaved 8-bit signed I/Q in both directions: `[I, Q]`, each `i8`.

pub mod error;
pub mod transport;

pub use error::{Error, HackRfErrorCode, Result};
pub use transport::{
    AsyncReadControlHandle, AsyncReadHandle, AsyncWriteHandle, HackRf, RECOMMENDED_BUFFER_SIZE,
    RecvState, TRANSFER_BUFFER_SIZE, TXVGA_MAX_DB,
};

/// HackRF USB Vendor ID (OpenMoko Inc, shared VID).
pub const HACKRF_VID: u16 = 0x1d50;

/// HackRF One USB Product ID.
pub const HACKRF_ONE_PID: u16 = 0x6089;

/// HackRF Jawbreaker USB Product ID.
pub const HACKRF_JAWBREAKER_PID: u16 = 0x604b;

/// rad1o USB Product ID.
pub const RAD1O_PID: u16 = 0xcc15;
