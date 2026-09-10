//! USB transport layer for HackRF devices.
//!
//! This module handles all USB communication with HackRF hardware:
//! device discovery, control transfers, and bulk streaming.

use crate::error::{Error, Result};
use crate::{HACKRF_JAWBREAKER_PID, HACKRF_ONE_PID, HACKRF_VID, RAD1O_PID};
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient};
use nusb::{Endpoint, MaybeFuture};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, mpsc};
use std::thread;
use std::time::Duration;

// ─── USB Constants ────────────────────────────────────────────────────────────

/// USB control transfer timeout (500ms, matches rs-spy convention).
/// Note: libhackrf uses timeout=0 (infinite), but bounded timeout is safer.
const USB_TIMEOUT: Duration = Duration::from_millis(500);

/// Bulk IN endpoint for RX data (endpoint 1, IN direction).
/// Reference: hackrf.c `RX_ENDPOINT_ADDRESS`
const RX_ENDPOINT_ADDRESS: u8 = 0x81;

/// Bulk OUT endpoint for TX data (endpoint 2, OUT direction).
/// Reference: hackrf.c `TX_ENDPOINT_ADDRESS`
const TX_ENDPOINT_ADDRESS: u8 = 0x02;

/// Number of concurrent USB transfers for streaming.
pub const TRANSFER_COUNT: usize = 4;

/// Size of each transfer buffer (256 KiB, matches libhackrf TRANSFER_BUFFER_SIZE).
pub const TRANSFER_BUFFER_SIZE: usize = 262_144;

/// Recommended buffer size for user reads (same as TRANSFER_BUFFER_SIZE).
pub const RECOMMENDED_BUFFER_SIZE: usize = TRANSFER_BUFFER_SIZE;

/// Bulk read timeout for streaming (1 second).
const BULK_TIMEOUT: Duration = Duration::from_millis(1000);

/// Default number of in-flight transfers for multi-transfer streaming.
pub const DEFAULT_NUM_TRANSFERS: usize = 4;

// ─── Multi-Transfer Streaming Types ───────────────────────────────────────────

/// Control commands for the multi-transfer streaming thread.
pub enum StreamControl {
    /// Retune to center frequency (Hz)
    Tune(u64),
    /// Set LNA gain (dB, 0-40 in 8 dB steps)
    SetLnaGain(u32),
    /// Set VGA gain (dB, 0-62 in 2 dB steps)
    SetVgaGain(u32),
    /// Enable/disable RF amplifier
    SetAmpEnable(bool),
    /// Set TX VGA (IF) gain (dB, 0-47 in 1 dB steps)
    SetTxvgaGain(u32),
}

/// Handle for receiving streaming data from a HackRF device.
///
/// Obtained from [`HackRf::into_streaming_reader()`]. The streaming thread
/// keeps multiple USB bulk transfers in-flight simultaneously, eliminating
/// the inter-transfer gap that causes FIFO overflow and sample loss.
pub struct AsyncReadHandle {
    rx: mpsc::Receiver<Result<Vec<u8>>>,
    ctrl_tx: mpsc::Sender<StreamControl>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
    thread: Option<thread::JoinHandle<()>>,
}

/// What a non-blocking read of a stream found.
pub enum RecvState {
    Data(Vec<u8>),
    /// Nothing yet; the next transfer has not landed.
    Empty,
    /// The transfer failed.
    Failed(Error),
    /// The streaming thread has exited: the radio is gone, or was stopped.
    Closed,
}

/// Clonable handle for sending control commands to an active streaming session.
///
/// Obtained from [`AsyncReadHandle::control_handle()`].
#[derive(Clone)]
pub struct AsyncReadControlHandle {
    ctrl_tx: mpsc::Sender<StreamControl>,
    stop: Arc<AtomicBool>,
    dropped: Arc<AtomicU64>,
}

impl AsyncReadHandle {
    /// Receive the next chunk of raw IQ bytes (blocking).
    ///
    /// Returns `None` when the streaming thread has exited (device disconnected,
    /// stop requested, or handle dropped).
    pub fn recv(&self) -> Option<Result<Vec<u8>>> {
        self.rx.recv().ok()
    }

    /// Try to receive the next chunk without blocking.
    pub fn try_recv(&self) -> Option<Result<Vec<u8>>> {
        self.rx.try_recv().ok()
    }

    /// The same, but saying which kind of nothing it got.
    ///
    /// A caller polling for samples has to tell "the next transfer has not
    /// landed yet" from "the streaming thread has gone", and
    /// [`Self::try_recv`] returns `None` for both. A receiver that cannot
    /// tell them apart polls a dead radio forever.
    pub fn recv_state(&self) -> RecvState {
        match self.rx.try_recv() {
            Ok(Ok(chunk)) => RecvState::Data(chunk),
            Ok(Err(e)) => RecvState::Failed(e),
            Err(mpsc::TryRecvError::Empty) => RecvState::Empty,
            Err(mpsc::TryRecvError::Disconnected) => RecvState::Closed,
        }
    }

    /// Get a clonable control handle for sending commands to the streaming thread.
    pub fn control_handle(&self) -> AsyncReadControlHandle {
        AsyncReadControlHandle {
            ctrl_tx: self.ctrl_tx.clone(),
            stop: self.stop.clone(),
            dropped: self.dropped.clone(),
        }
    }

    /// Stop streaming.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Drop for AsyncReadHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

impl AsyncReadControlHandle {
    /// Number of chunks dropped due to channel overflow (should always be 0
    /// since we use blocking sends; provided for diagnostics).
    pub fn dropped_chunks(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Request the streaming thread to stop.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Retune to a new center frequency (Hz).
    pub fn tune(&self, freq_hz: u64) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::Tune(freq_hz))
            .map_err(|_| Error::StreamingError("control channel closed".to_string()))
    }

    /// Set LNA gain (0-40 dB).
    pub fn set_lna_gain(&self, gain_db: u32) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::SetLnaGain(gain_db))
            .map_err(|_| Error::StreamingError("control channel closed".to_string()))
    }

    /// Set VGA gain (0-62 dB).
    pub fn set_vga_gain(&self, gain_db: u32) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::SetVgaGain(gain_db))
            .map_err(|_| Error::StreamingError("control channel closed".to_string()))
    }

    /// Enable or disable RF amplifier.
    pub fn set_amp_enable(&self, enable: bool) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::SetAmpEnable(enable))
            .map_err(|_| Error::StreamingError("control channel closed".to_string()))
    }

    /// Set TX VGA (IF) gain (0-47 dB).
    pub fn set_txvga_gain(&self, gain_db: u32) -> Result<()> {
        self.ctrl_tx
            .send(StreamControl::SetTxvgaGain(gain_db))
            .map_err(|_| Error::StreamingError("control channel closed".to_string()))
    }
}

// ─── Vendor Request IDs ───────────────────────────────────────────────────────
// Reference: hackrf.c lines 59-118

/// USB vendor request command IDs.
///
/// These map to `hackrf_vendor_request` enum in the C library.
/// Only the commands needed for RX operation are included.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
enum VendorRequest {
    /// Set transceiver mode (OFF/RX/TX/SS/CPLD_UPDATE/RX_SWEEP).
    SetTransceiverMode = 1,
    /// Set sample rate (8 bytes: freq_hz u32 + divider u32, LE).
    SampleRateSet = 6,
    /// Set baseband filter bandwidth (wValue=low16, wIndex=high16).
    BasebandFilterBandwidthSet = 7,
    /// Read board ID (returns 1 byte).
    BoardIdRead = 14,
    /// Read firmware version string.
    VersionStringRead = 15,
    /// Set center frequency (8 bytes: freq_mhz u32 + freq_hz u32, LE).
    SetFreq = 16,
    /// Enable/disable RF amplifier (wValue = 0 or 1).
    AmpEnable = 17,
    /// Read MCU part ID + serial number (24 bytes).
    BoardPartidSerialnoRead = 18,
    /// Set LNA gain (wIndex=value, reads 1 byte ACK).
    SetLnaGain = 19,
    /// Set VGA/baseband gain (wIndex=value, reads 1 byte ACK).
    SetVgaGain = 20,
    /// Set TX IF/VGA gain (wIndex=value, reads 1 byte ACK).
    SetTxvgaGain = 21,
    /// Enable/disable bias tee / antenna power (wValue = 0 or 1).
    AntennaEnable = 23,
    /// Reset the device. It re-enumerates afterwards, so everything holding
    /// it has to reopen.
    Reset = 30,
    /// Read board hardware revision (1 byte). Requires USB API >= 0x0106.
    BoardRevRead = 45,
    /// Read supported platform bitfield (4 bytes, big-endian). Requires USB API >= 0x0106.
    SupportedPlatformRead = 46,
}

// ─── Transceiver Modes ────────────────────────────────────────────────────────

/// Transceiver operating mode.
/// Reference: hackrf.c `hackrf_transceiver_mode` enum
const TRANSCEIVER_MODE_OFF: u16 = 0;
const TRANSCEIVER_MODE_RECEIVE: u16 = 1;
const TRANSCEIVER_MODE_TRANSMIT: u16 = 2;

/// Full scale of the transmit IF gain, in dB.
pub const TXVGA_MAX_DB: u32 = 47;

// ─── Board IDs ────────────────────────────────────────────────────────────────

/// Board ID to human-readable name.
/// Reference: hackrf.h `enum hackrf_board_id`
pub fn board_id_name(id: u8) -> &'static str {
    match id {
        0 => "Jellybean",
        1 => "Jawbreaker",
        2 => "HackRF One",
        3 => "rad1o",
        4 => "HackRF One", // r9+
        5 => "HackRF Pro", // Praline
        0xFE => "Unrecognized",
        0xFF => "Undetected",
        _ => "Unknown",
    }
}

/// Board revision to human-readable name.
///
/// The GSG (Great Scott Gadgets) flag is in bit 7.
/// Reference: hackrf.h `enum hackrf_board_rev`
pub fn board_rev_name(rev: u8) -> &'static str {
    // Strip the GSG manufacturer flag for the name
    match rev & !BOARD_REV_GSG {
        0 => "Older than r6",
        1 => "r6",
        2 => "r7",
        3 => "r8",
        4 => "r9",
        5 => "r10",
        _ => {
            if rev == 0xFE {
                "Unrecognized"
            } else if rev == 0xFF {
                "Undetected"
            } else {
                "Unknown"
            }
        }
    }
}

// ─── Baseband Filter Bandwidth Table ──────────────────────────────────────────
// Reference: hackrf.c `max2837_ft` table (valid MAX2837 filter bandwidths)

/// Valid baseband filter bandwidth values (Hz) for the MAX2837 transceiver.
static MAX2837_BANDWIDTHS: &[u32] = &[
    1_750_000, 2_500_000, 3_500_000, 5_000_000, 5_500_000, 6_000_000, 7_000_000, 8_000_000,
    9_000_000, 10_000_000, 12_000_000, 14_000_000, 15_000_000, 20_000_000, 24_000_000, 28_000_000,
];

/// Compute the best baseband filter bandwidth for a given target bandwidth.
///
/// Returns the largest valid MAX2837 bandwidth that is <= the target, except
/// below the minimum supported bandwidth where the minimum is returned. This
/// matches libhackrf's `hackrf_compute_baseband_filter_bw()` behavior.
pub fn compute_baseband_filter_bw(bandwidth_hz: u32) -> u32 {
    let mut previous = MAX2837_BANDWIDTHS[0];
    for &bw in MAX2837_BANDWIDTHS {
        if bw >= bandwidth_hz {
            return if bw == bandwidth_hz { bw } else { previous };
        }
        previous = bw;
    }
    // If requested bandwidth exceeds all valid values, return the largest.
    previous
}

// ─── Platform Flags ───────────────────────────────────────────────────────────

/// Platform bitfield flags.
/// Reference: hackrf.h `enum hackrf_platform`
pub const PLATFORM_JAWBREAKER: u32 = 1;
pub const PLATFORM_HACKRF1_OG: u32 = 2;
pub const PLATFORM_RAD1O: u32 = 4;
pub const PLATFORM_HACKRF1_R9: u32 = 8;

/// GSG manufacturer flag in board revision byte.
pub const BOARD_REV_GSG: u8 = 0x80;

// ─── HackRf Device ───────────────────────────────────────────────────────────

/// Main HackRF device handle.
///
/// Wraps a nusb USB interface and provides methods for device discovery,
/// configuration, and streaming. The nusb `Interface` is internally
/// Arc-backed, so cloning is cheap.
pub struct HackRf {
    iface: nusb::Interface,
    /// Keep the Device alive so USB isn't released prematurely.
    #[allow(dead_code)]
    usb_device: nusb::Device,
    /// USB API version (from bcdDevice descriptor field).
    usb_api_version: u16,
    /// Bulk IN endpoint for RX streaming (opened on start_rx, closed on stop_rx).
    rx_endpoint: Option<Endpoint<Bulk, In>>,
}

/// Handle for feeding samples to a transmitting HackRF.
///
/// Obtained from [`HackRf::into_streaming_writer()`]. The writing thread keeps
/// several USB bulk transfers queued so the radio always has bytes ahead of
/// it; a chunk that arrives too late leaves a gap, and a gap in transmission
/// is a splatter rather than a dropout, so the count is reported.
pub struct AsyncWriteHandle {
    tx: mpsc::SyncSender<Vec<u8>>,
    ctrl_tx: mpsc::Sender<StreamControl>,
    stop: Arc<AtomicBool>,
    idle: Arc<AtomicU64>,
    /// Chunks accepted but not yet handed to the USB queue.
    queued: Arc<AtomicU64>,
    thread: Option<thread::JoinHandle<()>>,
}

impl AsyncWriteHandle {
    /// Queue one chunk of interleaved `i8` I/Q for transmission.
    ///
    /// Blocks while the queue is full, which is the backpressure that keeps a
    /// producer running at the sample rate rather than ahead of it.
    pub fn send(&self, chunk: Vec<u8>) -> Result<()> {
        self.queued.fetch_add(1, Ordering::Relaxed);
        self.tx.send(chunk).map_err(|_| {
            self.queued.fetch_sub(1, Ordering::Relaxed);
            Error::StreamingError("transmit thread has exited".to_string())
        })
    }

    /// Queue a chunk, giving up after `timeout` rather than blocking.
    pub fn send_timeout(&self, chunk: Vec<u8>, timeout: Duration) -> Result<()> {
        let deadline = std::time::Instant::now() + timeout;
        self.queued.fetch_add(1, Ordering::Relaxed);
        let mut chunk = chunk;
        loop {
            match self.tx.try_send(chunk) {
                Ok(()) => return Ok(()),
                Err(mpsc::TrySendError::Full(c)) => {
                    if std::time::Instant::now() >= deadline {
                        self.queued.fetch_sub(1, Ordering::Relaxed);
                        return Err(Error::Timeout);
                    }
                    chunk = c;
                    thread::sleep(Duration::from_millis(1));
                }
                Err(mpsc::TrySendError::Disconnected(_)) => {
                    self.queued.fetch_sub(1, Ordering::Relaxed);
                    return Err(Error::StreamingError("transmit thread has exited".to_string()));
                }
            }
        }
    }

    pub fn control_handle(&self) -> AsyncReadControlHandle {
        AsyncReadControlHandle {
            ctrl_tx: self.ctrl_tx.clone(),
            stop: self.stop.clone(),
            dropped: self.idle.clone(),
        }
    }

    /// Transfers the thread filled with zeros because no chunk was ready.
    pub fn idle_transfers(&self) -> u64 {
        self.idle.load(Ordering::Relaxed)
    }

    /// Stop transmitting. Queued chunks are dropped rather than played out;
    /// call [`Self::drain`] first to send what is already queued.
    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Wait until every queued chunk has been handed to the device, or until
    /// `timeout` expires. Returns whether the queue emptied.
    ///
    /// The queue emptying means the last chunk was submitted, not that it has
    /// left the antenna: the device buffers a few transfers of its own, so a
    /// caller that stops immediately afterwards truncates the tail.
    pub fn drain(&self, timeout: Duration) -> bool {
        let deadline = std::time::Instant::now() + timeout;
        while std::time::Instant::now() < deadline {
            if self.queued.load(Ordering::Relaxed) == 0 {
                return true;
            }
            if self.stop.load(Ordering::Relaxed) {
                return false;
            }
            thread::sleep(Duration::from_millis(2));
        }
        false
    }
}

impl Drop for AsyncWriteHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

impl HackRf {
    // ─── Device Discovery ─────────────────────────────────────────────────

    /// List all connected HackRF devices by serial number string.
    pub fn list_devices() -> Result<Vec<String>> {
        let mut serials = Vec::new();

        for dev_info in nusb::list_devices()
            .wait()
            .map_err(Error::OpenFailed)?
            .filter(|d| d.vendor_id() == HACKRF_VID && is_hackrf_pid(d.product_id()))
        {
            let serial = dev_info.serial_number().unwrap_or("(unknown)").to_string();
            serials.push(serial);
        }

        Ok(serials)
    }

    /// Open the first available HackRF device.
    pub fn open_first() -> Result<Self> {
        Self::open_by_index(0)
    }

    /// Open a HackRF device by index in the enumerated list.
    pub fn open_by_index(index: usize) -> Result<Self> {
        let devices: Vec<_> = nusb::list_devices()
            .wait()
            .map_err(Error::OpenFailed)?
            .filter(|d| d.vendor_id() == HACKRF_VID && is_hackrf_pid(d.product_id()))
            .collect();

        let dev_info = devices.into_iter().nth(index).ok_or(Error::DeviceNotFound)?;

        // Extract USB API version from bcdDevice descriptor field
        let usb_api_version = dev_info.device_version();

        let usb_device = dev_info.open().wait().map_err(Error::OpenFailed)?;

        // Linux: detach kernel driver if attached (ignore errors)
        #[cfg(target_os = "linux")]
        {
            let _ = usb_device.detach_kernel_driver(0);
        }

        let iface = usb_device.claim_interface(0).wait().map_err(Error::ClaimFailed)?;

        tracing::info!("HackRF device opened successfully");

        Ok(HackRf { iface, usb_device, usb_api_version, rx_endpoint: None })
    }

    // ─── Device Info Queries ──────────────────────────────────────────────

    /// Read the board ID.
    ///
    /// Returns a numeric board ID. Use [`board_id_name()`] to get a
    /// human-readable string.
    ///
    /// Reference: hackrf.c `hackrf_board_id_read()` - vendor request 14
    pub fn board_id(&self) -> Result<u8> {
        let data = self.control_in(VendorRequest::BoardIdRead, 0, 0, 1)?;
        if data.is_empty() {
            return Err(Error::InvalidResponse("board_id: no data returned".into()));
        }
        Ok(data[0])
    }

    /// Read the firmware version string.
    ///
    /// Reference: hackrf.c `hackrf_version_string_read()` - vendor request 15
    pub fn version(&self) -> Result<String> {
        let data = self.control_in(VendorRequest::VersionStringRead, 0, 0, 255)?;
        // Null-terminate and convert to string
        let end = data.iter().position(|&b| b == 0).unwrap_or(data.len());
        Ok(String::from_utf8_lossy(&data[..end]).to_string())
    }

    /// Read the USB API version from the device descriptor.
    ///
    /// This is NOT a vendor request - it comes from the bcdDevice field
    /// of the USB device descriptor, which is read during open.
    ///
    /// Reference: hackrf.c `hackrf_usb_api_version_read()`
    pub fn usb_api_version(&self) -> u16 {
        self.usb_api_version
    }

    /// Read board part ID and serial number.
    ///
    /// Returns (part_id_0, part_id_1, serial_number_string).
    /// The serial number is formatted as a 32-character hex string
    /// matching hackrf_info output.
    ///
    /// Reference: hackrf.c `hackrf_board_partid_serialno_read()` - vendor request 18
    pub fn board_partid_serialno(&self) -> Result<(u32, u32, String)> {
        let data = self.control_in(VendorRequest::BoardPartidSerialnoRead, 0, 0, 24)?;
        if data.len() < 24 {
            return Err(Error::InvalidResponse(format!(
                "board_partid_serialno: expected 24 bytes, got {}",
                data.len()
            )));
        }

        // All fields are little-endian u32
        let part_id_0 = u32::from_le_bytes(data[0..4].try_into().unwrap());
        let part_id_1 = u32::from_le_bytes(data[4..8].try_into().unwrap());
        let serial_0 = u32::from_le_bytes(data[8..12].try_into().unwrap());
        let serial_1 = u32::from_le_bytes(data[12..16].try_into().unwrap());
        let serial_2 = u32::from_le_bytes(data[16..20].try_into().unwrap());
        let serial_3 = u32::from_le_bytes(data[20..24].try_into().unwrap());

        // Format serial as 32-char hex string (matching hackrf_info output)
        let serial_str =
            format!("{:08x}{:08x}{:08x}{:08x}", serial_0, serial_1, serial_2, serial_3);

        Ok((part_id_0, part_id_1, serial_str))
    }

    /// Read board hardware revision.
    ///
    /// Requires USB API version >= 0x0106.
    ///
    /// Reference: hackrf.c `hackrf_board_rev_read()` - vendor request 45
    pub fn board_rev(&self) -> Result<u8> {
        if self.usb_api_version < 0x0106 {
            return Err(Error::ConfigFailed("board_rev requires USB API >= 0x0106".into()));
        }
        let data = self.control_in(VendorRequest::BoardRevRead, 0, 0, 1)?;
        if data.is_empty() {
            return Err(Error::InvalidResponse("board_rev: no data returned".into()));
        }
        Ok(data[0])
    }

    /// Read supported platform bitfield.
    ///
    /// Requires USB API version >= 0x0106.
    /// Returns a bitfield where each bit represents a supported platform.
    ///
    /// Reference: hackrf.c `hackrf_supported_platform_read()` - vendor request 46
    /// Note: This value is big-endian (unlike most other HackRF responses).
    pub fn supported_platform(&self) -> Result<u32> {
        if self.usb_api_version < 0x0106 {
            return Err(Error::ConfigFailed(
                "supported_platform requires USB API >= 0x0106".into(),
            ));
        }
        let data = self.control_in(VendorRequest::SupportedPlatformRead, 0, 0, 4)?;
        if data.len() < 4 {
            return Err(Error::InvalidResponse(format!(
                "supported_platform: expected 4 bytes, got {}",
                data.len()
            )));
        }
        // Big-endian assembly (unique among HackRF commands)
        let platform = ((data[0] as u32) << 24)
            | ((data[1] as u32) << 16)
            | ((data[2] as u32) << 8)
            | (data[3] as u32);
        Ok(platform)
    }

    // ─── Configuration Commands ───────────────────────────────────────────

    /// Set the center frequency in Hz.
    ///
    /// The frequency is split into MHz and Hz remainder parts and sent as
    /// an 8-byte little-endian struct.
    ///
    /// HackRF One frequency range: ~1 MHz to 6 GHz.
    ///
    /// Reference: hackrf.c `hackrf_set_freq()` - vendor request 16
    pub fn set_freq(&self, freq_hz: u64) -> Result<()> {
        let freq_mhz = (freq_hz / 1_000_000) as u32;
        let freq_remainder = (freq_hz % 1_000_000) as u32;

        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&freq_mhz.to_le_bytes());
        data[4..8].copy_from_slice(&freq_remainder.to_le_bytes());

        self.control_out(VendorRequest::SetFreq, 0, 0, &data)?;

        tracing::debug!("Set frequency to {} Hz ({}.{:06} MHz)", freq_hz, freq_mhz, freq_remainder);
        Ok(())
    }

    /// Set the sample rate using manual freq_hz / divider parameters.
    ///
    /// After setting the sample rate, automatically sets the baseband
    /// filter bandwidth to 75% of the effective rate (matching libhackrf).
    ///
    /// Common rates: 2M, 4M, 8M, 10M, 12.5M, 16M, 20M (max)
    ///
    /// Reference: hackrf.c `hackrf_set_sample_rate_manual()` - vendor request 6
    pub fn set_sample_rate_manual(&self, freq_hz: u32, divider: u32) -> Result<()> {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&freq_hz.to_le_bytes());
        data[4..8].copy_from_slice(&divider.to_le_bytes());

        self.control_out(VendorRequest::SampleRateSet, 0, 0, &data)?;

        // Automatically set baseband filter to 75% of effective rate
        // Reference: hackrf.c line 1857-1860
        let effective_rate = freq_hz / divider;
        let bw = compute_baseband_filter_bw((0.75 * effective_rate as f64) as u32);
        self.set_baseband_filter_bandwidth(bw)?;

        tracing::debug!(
            "Set sample rate to {} Hz (divider={}), baseband BW={} Hz",
            freq_hz,
            divider,
            bw
        );
        Ok(())
    }

    /// Set the sample rate in Hz.
    ///
    /// This is the simple API - computes optimal freq/divider pair automatically.
    /// For most cases, using divider=1 with the target rate works fine.
    ///
    /// Reference: hackrf.c `hackrf_set_sample_rate()`
    pub fn set_sample_rate(&self, sample_rate_hz: u32) -> Result<()> {
        // Simple approach: use divider=1 for common rates
        // This works for all standard rates (2M, 4M, 8M, 10M, 16M, 20M)
        self.set_sample_rate_manual(sample_rate_hz, 1)
    }

    /// Set baseband filter bandwidth directly.
    ///
    /// Valid values are from the MAX2837 filter table. Use
    /// [`compute_baseband_filter_bw()`] to find the nearest valid value.
    ///
    /// Reference: hackrf.c `hackrf_set_baseband_filter_bandwidth()` - vendor request 7
    pub fn set_baseband_filter_bandwidth(&self, bandwidth_hz: u32) -> Result<()> {
        let w_value = (bandwidth_hz & 0xFFFF) as u16;
        let w_index = (bandwidth_hz >> 16) as u16;

        self.control_out(VendorRequest::BasebandFilterBandwidthSet, w_value, w_index, &[])?;

        tracing::debug!("Set baseband filter bandwidth to {} Hz", bandwidth_hz);
        Ok(())
    }

    /// Set LNA (low noise amplifier) gain.
    ///
    /// Range: 0-40 dB in 8 dB steps. Value is rounded down to nearest 8 dB.
    ///
    /// Reference: hackrf.c `hackrf_set_lna_gain()` - vendor request 19
    pub fn set_lna_gain(&self, gain_db: u32) -> Result<()> {
        if gain_db > 40 {
            return Err(Error::ConfigFailed(format!("LNA gain must be 0-40, got {gain_db}")));
        }
        // Round down to 8 dB steps (mask off lower 3 bits)
        let value = gain_db & !0x07;

        let retval = self.control_in(VendorRequest::SetLnaGain, 0, value as u16, 1)?;

        if retval.is_empty() || retval[0] == 0 {
            return Err(Error::InvalidResponse(format!(
                "set_lna_gain({value}): device rejected value"
            )));
        }

        tracing::debug!("Set LNA gain to {} dB (requested {} dB)", value, gain_db);
        Ok(())
    }

    /// Set VGA (variable gain amplifier / baseband) gain.
    ///
    /// Range: 0-62 dB in 2 dB steps. Value is rounded down to nearest 2 dB.
    ///
    /// Reference: hackrf.c `hackrf_set_vga_gain()` - vendor request 20
    pub fn set_vga_gain(&self, gain_db: u32) -> Result<()> {
        if gain_db > 62 {
            return Err(Error::ConfigFailed(format!("VGA gain must be 0-62, got {gain_db}")));
        }
        // Round down to 2 dB steps (mask off LSB)
        let value = gain_db & !0x01;

        let retval = self.control_in(VendorRequest::SetVgaGain, 0, value as u16, 1)?;

        if retval.is_empty() || retval[0] == 0 {
            return Err(Error::InvalidResponse(format!(
                "set_vga_gain({value}): device rejected value"
            )));
        }

        tracing::debug!("Set VGA gain to {} dB (requested {} dB)", value, gain_db);
        Ok(())
    }

    /// Enable or disable the RF amplifier (14 dB, before LNA).
    ///
    /// Reference: hackrf.c `hackrf_set_amp_enable()` - vendor request 17
    pub fn set_amp_enable(&self, enable: bool) -> Result<()> {
        let value = if enable { 1u16 } else { 0u16 };
        self.control_out(VendorRequest::AmpEnable, value, 0, &[])?;
        tracing::debug!("RF amplifier {}", if enable { "enabled" } else { "disabled" });
        Ok(())
    }

    /// Set the transmit IF (TXVGA) gain.
    ///
    /// Range: 0-47 dB in 1 dB steps. This is the only transmit gain that is
    /// continuous; the front end amp adds its fixed step on top.
    ///
    /// Reference: hackrf.c `hackrf_set_txvga_gain()` - vendor request 21
    pub fn set_txvga_gain(&self, gain_db: u32) -> Result<()> {
        if gain_db > TXVGA_MAX_DB {
            return Err(Error::ConfigFailed(format!(
                "TXVGA gain must be 0-{TXVGA_MAX_DB}, got {gain_db}"
            )));
        }

        let retval = self.control_in(VendorRequest::SetTxvgaGain, 0, gain_db as u16, 1)?;

        if retval.is_empty() || retval[0] == 0 {
            return Err(Error::InvalidResponse(format!(
                "set_txvga_gain({gain_db}): device rejected value"
            )));
        }

        tracing::debug!("Set TXVGA gain to {} dB", gain_db);
        Ok(())
    }

    /// Enable or disable the antenna port bias tee (DC power on antenna).
    ///
    /// WARNING: Only enable this if your antenna or LNA requires DC power!
    ///
    /// Reference: hackrf.c `hackrf_set_antenna_enable()` - vendor request 23
    pub fn set_antenna_enable(&self, enable: bool) -> Result<()> {
        let value = if enable { 1u16 } else { 0u16 };
        self.control_out(VendorRequest::AntennaEnable, value, 0, &[])?;
        tracing::debug!("Bias tee {}", if enable { "enabled" } else { "disabled" });
        Ok(())
    }

    // ─── Streaming ────────────────────────────────────────────────────────

    /// Reset the board, as `hackrf_reset()` does.
    ///
    /// The last resort for a radio that has stopped producing samples: it
    /// drops off the bus and comes back a second or two later, so every
    /// handle to it is dead afterwards and the caller has to reopen. The
    /// device usually does not acknowledge the request before it goes, so a
    /// transfer error here is the normal case rather than a failure.
    pub fn reset(&self) -> Result<()> {
        let r = self.control_out(VendorRequest::Reset, 0, 0, &[]);
        tracing::info!("HackRF reset requested");
        match r {
            Ok(()) => Ok(()),
            // It went before it could answer, which is what was asked for.
            Err(Error::ControlTransfer(_)) => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Set the transceiver mode.
    ///
    /// Reference: hackrf.c `hackrf_set_transceiver_mode()` - vendor request 1
    fn set_transceiver_mode(&self, mode: u16) -> Result<()> {
        self.control_out(VendorRequest::SetTransceiverMode, mode, 0, &[])?;
        Ok(())
    }

    /// Start RX streaming.
    ///
    /// Opens the bulk IN endpoint and sets transceiver to receive mode.
    /// After calling this, use [`read_sync()`](Self::read_sync) to read sample data.
    ///
    /// Reference: hackrf.c `hackrf_start_rx()`
    pub fn start_rx(&mut self) -> Result<()> {
        // Open the bulk IN endpoint for RX
        let ep_in = self
            .iface
            .endpoint::<Bulk, In>(RX_ENDPOINT_ADDRESS)
            .map_err(|e| Error::ConfigFailed(format!("failed to open RX endpoint: {e}")))?;
        self.rx_endpoint = Some(ep_in);

        self.set_transceiver_mode(TRANSCEIVER_MODE_RECEIVE)?;
        tracing::info!("RX streaming started");
        Ok(())
    }

    /// Stop RX streaming.
    ///
    /// Sets transceiver back to OFF mode and closes the bulk endpoint.
    ///
    /// Reference: hackrf.c `hackrf_stop_rx()`
    pub fn stop_rx(&mut self) -> Result<()> {
        self.set_transceiver_mode(TRANSCEIVER_MODE_OFF)?;

        // Cancel any pending transfers and drop the endpoint
        if let Some(mut ep) = self.rx_endpoint.take() {
            ep.cancel_all();
            while ep.pending() > 0 {
                let _ = ep.wait_next_complete(Duration::from_millis(100));
            }
        }

        tracing::info!("RX streaming stopped");
        Ok(())
    }

    /// Start multi-transfer RX streaming and return an async read handle.
    ///
    /// This consumes the `HackRf` device and moves it into a dedicated
    /// streaming thread that keeps multiple USB bulk transfers in-flight
    /// simultaneously. This eliminates the inter-transfer gap that causes
    /// FIFO overflow and sample loss with single-transfer `read_sync()`.
    ///
    /// Architecture:
    /// ```text
    ///   streaming thread (nusb endpoint queue)  ──sync_channel──▶  consumer
    ///     └── control commands via mpsc channel   (tune / gain)
    /// ```
    ///
    /// Use [`AsyncReadHandle::recv()`] to read chunks and
    /// [`AsyncReadHandle::control_handle()`] for runtime control.
    ///
    /// # Arguments
    ///
    /// * `num_transfers` - Number of concurrent USB transfers (0 for default of 4)
    /// * `transfer_size` - Size of each transfer buffer in bytes (0 for default of 256 KB)
    pub fn into_streaming_reader(
        self,
        num_transfers: usize,
        transfer_size: usize,
    ) -> Result<AsyncReadHandle> {
        let num_transfers = if num_transfers == 0 { DEFAULT_NUM_TRANSFERS } else { num_transfers };
        let transfer_size = if transfer_size == 0 { TRANSFER_BUFFER_SIZE } else { transfer_size };

        if !transfer_size.is_multiple_of(512) {
            return Err(Error::StreamingError(format!(
                "Invalid transfer size {} (must be multiple of 512)",
                transfer_size
            )));
        }

        // The streaming thread opens the endpoint and queues transfers before
        // enabling RX. Starting RX before any USB reads are pending can overflow
        // the HackRF FIFO and drop initial samples.

        // Bounded channel for sample delivery (backpressure-aware)
        let (tx, rx) = mpsc::sync_channel::<Result<Vec<u8>>>(num_transfers * 4);
        // Unbounded channel for control commands
        let (ctrl_tx, ctrl_rx) = mpsc::channel::<StreamControl>();
        let stop = Arc::new(AtomicBool::new(false));
        let dropped = Arc::new(AtomicU64::new(0));

        let iface = self.iface.clone();
        let stop_thread = Arc::clone(&stop);

        let thread = thread::Builder::new()
            .name("hackrf-stream".into())
            .spawn(move || {
                streaming_thread(
                    self,
                    iface,
                    tx,
                    stop_thread,
                    ctrl_rx,
                    transfer_size,
                    num_transfers,
                );
            })
            .map_err(|e| Error::StreamingError(format!("failed to spawn streaming thread: {e}")))?;

        Ok(AsyncReadHandle { rx, ctrl_tx, stop, dropped, thread: Some(thread) })
    }

    /// Start transmitting and return a handle to feed samples into.
    ///
    /// Mirrors [`Self::into_streaming_reader()`]: the device moves onto a
    /// dedicated thread that keeps several bulk OUT transfers in flight, so
    /// there is never a moment where the radio has nothing queued. A transfer
    /// the producer failed to fill in time is sent as zeros and counted; the
    /// alternative is a stalled endpoint, which the hardware treats as an
    /// underrun and which sounds like a click on air.
    ///
    /// The transmit gain is not set here. A HackRF that comes up transmitting
    /// at whatever TXVGA it was left at is a hazard, so the caller sets the
    /// gain deliberately, after checking what is connected to the antenna.
    pub fn into_streaming_writer(
        self,
        num_transfers: usize,
        transfer_size: usize,
    ) -> Result<AsyncWriteHandle> {
        let num_transfers = if num_transfers == 0 { DEFAULT_NUM_TRANSFERS } else { num_transfers };
        let transfer_size = if transfer_size == 0 { TRANSFER_BUFFER_SIZE } else { transfer_size };

        if !transfer_size.is_multiple_of(512) {
            return Err(Error::StreamingError(format!(
                "Invalid transfer size {transfer_size} (must be multiple of 512)"
            )));
        }

        let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(num_transfers * 4);
        let (ctrl_tx, ctrl_rx) = mpsc::channel::<StreamControl>();
        let stop = Arc::new(AtomicBool::new(false));
        let idle = Arc::new(AtomicU64::new(0));
        let queued = Arc::new(AtomicU64::new(0));

        let iface = self.iface.clone();
        let stop_thread = Arc::clone(&stop);
        let idle_thread = Arc::clone(&idle);
        let queued_thread = Arc::clone(&queued);

        let thread = thread::Builder::new()
            .name("hackrf-tx".into())
            .spawn(move || {
                transmitting_thread(
                    self,
                    iface,
                    rx,
                    stop_thread,
                    idle_thread,
                    queued_thread,
                    ctrl_rx,
                    transfer_size,
                    num_transfers,
                );
            })
            .map_err(|e| Error::StreamingError(format!("failed to spawn transmit thread: {e}")))?;

        Ok(AsyncWriteHandle { tx, ctrl_tx, stop, idle, queued, thread: Some(thread) })
    }

    /// Read a chunk of RX data synchronously.
    ///
    /// Returns the number of bytes actually read. The data is interleaved
    /// 8-bit signed I/Q samples: `[I0, Q0, I1, Q1, ...]`.
    ///
    /// Use a buffer of [`RECOMMENDED_BUFFER_SIZE`] (256 KiB) for best performance.
    pub fn read_sync(&mut self, buf: &mut [u8]) -> Result<usize> {
        self.read_sync_timeout(buf, BULK_TIMEOUT)
    }

    /// Read a chunk of RX data with a custom timeout.
    pub fn read_sync_timeout(&mut self, buf: &mut [u8], timeout: Duration) -> Result<usize> {
        let ep = self
            .rx_endpoint
            .as_mut()
            .ok_or_else(|| Error::StreamingError("RX not started (call start_rx first)".into()))?;

        let transfer_buf = Buffer::new(buf.len());
        let completion = ep.transfer_blocking(transfer_buf, timeout);
        completion.status.map_err(Error::BulkTransfer)?;

        let n = completion.actual_len.min(buf.len());
        buf[..n].copy_from_slice(&completion.buffer[..n]);
        Ok(n)
    }

    // ─── Low-Level USB Helpers ────────────────────────────────────────────

    /// Send a vendor control IN transfer (device -> host).
    ///
    /// Returns the received data bytes.
    fn control_in(
        &self,
        request: VendorRequest,
        w_value: u16,
        w_index: u16,
        length: u16,
    ) -> Result<Vec<u8>> {
        let data = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: request as u8,
                    value: w_value,
                    index: w_index,
                    length,
                },
                USB_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)?;

        tracing::trace!(
            "control_in: req={:?} wValue={} wIndex={} -> {} bytes",
            request,
            w_value,
            w_index,
            data.len()
        );
        Ok(data)
    }

    /// Send a vendor control OUT transfer (host -> device).
    fn control_out(
        &self,
        request: VendorRequest,
        w_value: u16,
        w_index: u16,
        data: &[u8],
    ) -> Result<()> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: request as u8,
                    value: w_value,
                    index: w_index,
                    data,
                },
                USB_TIMEOUT,
            )
            .wait()
            .map_err(Error::ControlTransfer)?;

        tracing::trace!(
            "control_out: req={:?} wValue={} wIndex={} data={} bytes",
            request,
            w_value,
            w_index,
            data.len()
        );
        Ok(())
    }
}

impl Drop for HackRf {
    fn drop(&mut self) {
        // Stop transceiver
        let _ = self.set_transceiver_mode(TRANSCEIVER_MODE_OFF);

        // Clean up endpoint if still open
        if let Some(mut ep) = self.rx_endpoint.take() {
            ep.cancel_all();
            while ep.pending() > 0 {
                let _ = ep.wait_next_complete(Duration::from_millis(100));
            }
        }

        // Note: nusb releases the interface automatically when all
        // Interface clones are dropped. No explicit release needed.
        tracing::debug!("HackRf device closed");
    }
}

/// Check if a USB product ID matches a known HackRF device.
fn is_hackrf_pid(pid: u16) -> bool {
    pid == HACKRF_ONE_PID || pid == HACKRF_JAWBREAKER_PID || pid == RAD1O_PID
}

// ─── Multi-Transfer Streaming Thread ──────────────────────────────────────────

/// The streaming thread function.
///
/// Uses nusb's Endpoint queue to keep multiple USB bulk transfers in-flight
/// simultaneously. When one transfer completes, a new one is immediately
/// re-submitted, so there is **never a gap** where the HackRF has no pending
/// USB read — preventing internal FIFO overflow and sample loss.
///
/// The thread loops:
/// 1. Process any pending control commands (tune, gain) — non-blocking
/// 2. Wait for the next completed transfer
/// 3. Send the data to the consumer via the bounded channel (blocking for backpressure)
/// 4. Re-submit a new buffer to keep the queue full
fn streaming_thread(
    mut dev: HackRf,
    iface: nusb::Interface,
    tx: mpsc::SyncSender<Result<Vec<u8>>>,
    stop: Arc<AtomicBool>,
    ctrl_rx: mpsc::Receiver<StreamControl>,
    transfer_size: usize,
    num_transfers: usize,
) {
    tracing::debug!(
        "HackRF streaming thread started: transfer_size={}, num_transfers={}",
        transfer_size,
        num_transfers
    );

    // Open the bulk IN endpoint
    let Ok(mut ep_in) = iface.endpoint::<Bulk, In>(RX_ENDPOINT_ADDRESS) else {
        tracing::warn!("failed to open bulk endpoint 0x{:02x}", RX_ENDPOINT_ADDRESS);
        return;
    };

    // Pre-fill the queue with N transfers before enabling RX so the device has
    // host-side reads ready as soon as samples start flowing.
    for _ in 0..num_transfers {
        ep_in.submit(Buffer::new(transfer_size));
    }

    tracing::debug!("submitted {} initial transfers", num_transfers);

    if let Err(e) = dev.set_transceiver_mode(TRANSCEIVER_MODE_RECEIVE) {
        tracing::warn!("failed to enable HackRF RX mode: {}", e);
        ep_in.cancel_all();
        while ep_in.pending() > 0 {
            let _ = ep_in.wait_next_complete(Duration::from_millis(100));
        }
        return;
    }
    tracing::info!("HackRF RX mode enabled for streaming");

    // Consecutive transfer error counter for disconnect detection
    const MAX_CONSECUTIVE_ERRORS: u32 = 5;
    let mut consecutive_errors: u32 = 0;

    // Main streaming loop
    while !stop.load(Ordering::Relaxed) {
        // 1. Process any pending control commands (non-blocking)
        while let Ok(cmd) = ctrl_rx.try_recv() {
            match cmd {
                StreamControl::Tune(freq) => {
                    tracing::debug!("streaming thread: tuning to {} Hz", freq);
                    if let Err(e) = dev.set_freq(freq) {
                        tracing::warn!("HackRF retune to {} Hz failed: {}", freq, e);
                    }
                }
                StreamControl::SetLnaGain(gain) => {
                    if let Err(e) = dev.set_lna_gain(gain) {
                        tracing::warn!("HackRF set LNA gain to {} dB failed: {}", gain, e);
                    }
                }
                StreamControl::SetVgaGain(gain) => {
                    if let Err(e) = dev.set_vga_gain(gain) {
                        tracing::warn!("HackRF set VGA gain to {} dB failed: {}", gain, e);
                    }
                }
                StreamControl::SetAmpEnable(enable) => {
                    if let Err(e) = dev.set_amp_enable(enable) {
                        tracing::warn!("HackRF set amp enable={} failed: {}", enable, e);
                    }
                }
                StreamControl::SetTxvgaGain(gain) => {
                    tracing::warn!("transmit gain {} dB requested while receiving", gain);
                }
            }
        }

        if stop.load(Ordering::Relaxed) {
            break;
        }

        // 2. Wait for the next completed transfer
        let completion = match ep_in.wait_next_complete(Duration::from_millis(500)) {
            Some(c) => c,
            None => {
                // Timeout — check stop flag and retry
                tracing::trace!("streaming: transfer timeout, retrying");
                continue;
            }
        };

        // 3. Check for transfer errors
        if let Err(ref e) = completion.status {
            consecutive_errors += 1;
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                tracing::warn!(
                    "HackRF disconnected ({} consecutive transfer errors: {})",
                    consecutive_errors,
                    e
                );
                stop.store(true, Ordering::Relaxed);
                break;
            }
            tracing::warn!(
                "bulk transfer error ({}/{}): {}",
                consecutive_errors,
                MAX_CONSECUTIVE_ERRORS,
                e
            );
            // Re-submit and continue (transient errors are common)
            ep_in.submit(Buffer::new(transfer_size));
            continue;
        }

        // Successful transfer — reset error counter
        consecutive_errors = 0;

        // Extract the data
        let data = completion.buffer[..completion.actual_len].to_vec();

        if data.is_empty() {
            ep_in.submit(Buffer::new(transfer_size));
            continue;
        }

        // Re-submit a new transfer before handing the completed buffer to the
        // consumer. This keeps the USB queue full even if downstream DSP blocks
        // briefly while receiving this chunk.
        ep_in.submit(Buffer::new(transfer_size));

        // 4. Send data to the consumer (blocking — provides backpressure).
        // IMPORTANT: We use blocking send here to avoid silently dropping chunks;
        // dropped samples corrupt protocol framing. The transfer has already been
        // re-submitted above, so short consumer stalls do not immediately drain
        // the HackRF USB queue.
        match tx.send(Ok(data)) {
            Ok(()) => {}
            Err(_) => {
                // Channel closed — consumer is gone
                tracing::debug!("streaming: consumer disconnected");
                break;
            }
        }
    }

    // Stop the receiver
    let _ = dev.stop_rx();

    // Cancel all pending transfers
    ep_in.cancel_all();

    // Drain remaining completions
    while ep_in.pending() > 0 {
        let _ = ep_in.wait_next_complete(Duration::from_millis(100));
    }

    tracing::debug!("HackRF streaming thread exited");
}

// ─── Transmit Thread ──────────────────────────────────────────────────────────

/// How long the transmit thread waits for the producer before filling a
/// transfer with zeros. Long enough to cover a scheduling hiccup, short
/// enough that the queue never empties: the remaining in-flight transfers
/// hold about one queue depth of samples, which at 2 MS/s is 500 ms.
const TX_FILL_WAIT: Duration = Duration::from_millis(20);

/// Cuts the producer's chunks into exactly the transfer size the endpoint
/// wants, keeping whatever is left over for the next transfer.
struct TxFiller {
    pend: Vec<u8>,
}

impl TxFiller {
    fn next(
        &mut self,
        rx: &mpsc::Receiver<Vec<u8>>,
        queued: &AtomicU64,
        idle: &AtomicU64,
        size: usize,
    ) -> Vec<u8> {
        let deadline = std::time::Instant::now() + TX_FILL_WAIT;
        while self.pend.len() < size {
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            if left.is_zero() {
                break;
            }
            match rx.recv_timeout(left) {
                Ok(chunk) => {
                    queued.fetch_sub(1, Ordering::Relaxed);
                    self.pend.extend_from_slice(&chunk);
                }
                Err(_) => break,
            }
        }

        if self.pend.len() >= size {
            return self.pend.drain(..size).collect();
        }

        // Nothing to send, or not enough. Zero is no carrier rather than a
        // held sample, so a gap is silence and not a tone at the last value.
        idle.fetch_add(1, Ordering::Relaxed);
        let mut out = std::mem::take(&mut self.pend);
        out.resize(size, 0);
        out
    }
}

/// The transmit thread: the mirror of [`streaming_thread`].
///
/// Transfers are queued before the transceiver is put into transmit mode, so
/// the radio has bytes waiting the moment it starts asking for them. Turning
/// transmit on first and filling afterwards puts whatever the FIFO held on
/// air, which is the previous transmission.
#[allow(clippy::too_many_arguments)]
fn transmitting_thread(
    dev: HackRf,
    iface: nusb::Interface,
    rx: mpsc::Receiver<Vec<u8>>,
    stop: Arc<AtomicBool>,
    idle: Arc<AtomicU64>,
    queued: Arc<AtomicU64>,
    ctrl_rx: mpsc::Receiver<StreamControl>,
    transfer_size: usize,
    num_transfers: usize,
) {
    let Ok(mut ep_out) = iface.endpoint::<Bulk, Out>(TX_ENDPOINT_ADDRESS) else {
        tracing::warn!("failed to open bulk endpoint 0x{:02x}", TX_ENDPOINT_ADDRESS);
        return;
    };

    let mut filler = TxFiller { pend: Vec::new() };

    for _ in 0..num_transfers {
        let chunk = filler.next(&rx, &queued, &idle, transfer_size);
        ep_out.submit(Buffer::from(chunk));
    }

    if let Err(e) = dev.set_transceiver_mode(TRANSCEIVER_MODE_TRANSMIT) {
        tracing::warn!("failed to enable HackRF TX mode: {}", e);
        ep_out.cancel_all();
        while ep_out.pending() > 0 {
            let _ = ep_out.wait_next_complete(Duration::from_millis(100));
        }
        return;
    }
    tracing::info!("HackRF TX mode enabled");

    const MAX_CONSECUTIVE_ERRORS: u32 = 5;
    let mut consecutive_errors: u32 = 0;

    while !stop.load(Ordering::Relaxed) {
        while let Ok(cmd) = ctrl_rx.try_recv() {
            match cmd {
                StreamControl::Tune(freq) => {
                    if let Err(e) = dev.set_freq(freq) {
                        tracing::warn!("HackRF retune to {} Hz failed: {}", freq, e);
                    }
                }
                StreamControl::SetTxvgaGain(gain) => {
                    if let Err(e) = dev.set_txvga_gain(gain) {
                        tracing::warn!("HackRF set TXVGA gain to {} dB failed: {}", gain, e);
                    }
                }
                StreamControl::SetAmpEnable(enable) => {
                    if let Err(e) = dev.set_amp_enable(enable) {
                        tracing::warn!("HackRF set amp enable={} failed: {}", enable, e);
                    }
                }
                // Receive gains have no meaning while transmitting.
                StreamControl::SetLnaGain(_) | StreamControl::SetVgaGain(_) => {}
            }
        }

        if stop.load(Ordering::Relaxed) {
            break;
        }

        let completion = match ep_out.wait_next_complete(Duration::from_millis(500)) {
            Some(c) => c,
            None => continue,
        };

        if let Err(ref e) = completion.status {
            consecutive_errors += 1;
            if consecutive_errors >= MAX_CONSECUTIVE_ERRORS {
                tracing::warn!(
                    "HackRF disconnected ({} consecutive transmit errors: {})",
                    consecutive_errors,
                    e
                );
                stop.store(true, Ordering::Relaxed);
                break;
            }
            tracing::warn!(
                "bulk transmit error ({}/{}): {}",
                consecutive_errors,
                MAX_CONSECUTIVE_ERRORS,
                e
            );
        } else {
            consecutive_errors = 0;
        }

        let chunk = filler.next(&rx, &queued, &idle, transfer_size);
        ep_out.submit(Buffer::from(chunk));
    }

    // Off before the transfers are cancelled: cancelling first leaves the
    // device transmitting whatever the FIFO still holds.
    let _ = dev.set_transceiver_mode(TRANSCEIVER_MODE_OFF);
    ep_out.cancel_all();
    while ep_out.pending() > 0 {
        let _ = ep_out.wait_next_complete(Duration::from_millis(100));
    }

    tracing::debug!(
        "HackRF transmit thread exited, {} idle transfers",
        idle.load(Ordering::Relaxed)
    );
}

#[cfg(test)]
mod tests {
    use super::compute_baseband_filter_bw;

    #[test]
    fn baseband_filter_bandwidth_matches_libhackrf_round_down() {
        // Below the minimum table entry, libhackrf returns the minimum.
        assert_eq!(compute_baseband_filter_bw(1_536_000), 1_750_000);

        // Otherwise libhackrf rounds down to avoid exceeding 75% of Fs.
        assert_eq!(compute_baseband_filter_bw(3_000_000), 2_500_000);
        assert_eq!(compute_baseband_filter_bw(3_072_000), 2_500_000);
        assert_eq!(compute_baseband_filter_bw(7_500_000), 7_000_000);
        assert_eq!(compute_baseband_filter_bw(7_680_000), 7_000_000);
    }

    #[test]
    fn baseband_filter_bandwidth_keeps_exact_matches() {
        assert_eq!(compute_baseband_filter_bw(1_750_000), 1_750_000);
        assert_eq!(compute_baseband_filter_bw(8_000_000), 8_000_000);
        assert_eq!(compute_baseband_filter_bw(28_000_000), 28_000_000);
    }

    #[test]
    fn baseband_filter_bandwidth_caps_above_maximum() {
        assert_eq!(compute_baseband_filter_bw(99_000_000), 28_000_000);
    }
}

#[cfg(test)]
mod tx_tests {
    use super::*;

    fn filler() -> (TxFiller, AtomicU64, AtomicU64) {
        (TxFiller { pend: Vec::new() }, AtomicU64::new(0), AtomicU64::new(0))
    }

    #[test]
    fn chunks_are_recut_to_the_transfer_size_without_losing_samples() {
        let (mut f, queued, idle) = filler();
        let (tx, rx) = mpsc::sync_channel::<Vec<u8>>(8);
        // Three chunks of 6 bytes, asked for in transfers of 4.
        for i in 0..3u8 {
            queued.fetch_add(1, Ordering::Relaxed);
            tx.send(vec![i * 10, i * 10 + 1, i * 10 + 2, i * 10 + 3, i * 10 + 4, i * 10 + 5])
                .unwrap();
        }
        let mut out = Vec::new();
        for _ in 0..5 {
            out.extend(f.next(&rx, &queued, &idle, 4));
        }
        assert_eq!(&out[..18], &[0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15, 20, 21, 22, 23, 24, 25]);
        // The fourth transfer runs past the data, so it is padded and counted.
        assert_eq!(idle.load(Ordering::Relaxed), 1);
        assert_eq!(&out[18..], &[0, 0], "the tail is padded, not repeated");
        assert_eq!(queued.load(Ordering::Relaxed), 0, "every chunk accounted for");
    }

    #[test]
    fn a_starved_transfer_is_zeros_rather_than_the_last_sample() {
        let (mut f, queued, idle) = filler();
        let (_tx, rx) = mpsc::sync_channel::<Vec<u8>>(1);
        let chunk = f.next(&rx, &queued, &idle, 512);
        assert_eq!(chunk.len(), 512);
        assert!(chunk.iter().all(|&b| b == 0), "a gap must be no carrier");
        assert_eq!(idle.load(Ordering::Relaxed), 1);
    }
}
