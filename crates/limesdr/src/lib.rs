//! LimeSDR support through LimeSuite's LMS API.
//!
//! # Threading model
//!
//! LimeSuite runs its own USB thread behind a FIFO, so there is no callback to
//! service and no reader thread of our own: [`LimeStream::read`] calls
//! `LMS_RecvStream` and blocks in the library. Control calls come from the UI
//! thread while that read is in flight, which LimeSuite allows, but its device
//! state is not internally locked, so every control call is serialised through
//! [`Handle::ctl`] the same way the RTL-SDR driver does it.
//!
//! # Sample handling
//!
//! The stream is configured for `LMS_FMT_F32` over a 12-bit link. LimeSuite
//! then hands back interleaved f32 already scaled to roughly [-1, 1], which is
//! the format the rest of waveshark wants, so samples are read straight into a
//! `Vec<C32>` with no conversion pass. The link stays 12-bit because that is
//! what the ADC produces: asking for 16-bit doubles the USB load for four bits
//! of zeros, and this board is often on a USB 2.0 port where that is the
//! difference between streaming and not.

use common::device::{Device, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange};
use common::{C32, Error, Hz, IqBuf, Result, SampleFormat, Sps, TxStream};
use limesdr_sys as ffi;
use std::ffi::CStr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// Datasheet tuning range for the LimeSDR-USB and Mini.
const FREQ_MIN: u64 = 100_000;
const FREQ_MAX: u64 = 3_800_000_000;

/// The LMS7002M will go lower, but below 1 MS/s the decimation chain needs
/// oversampling ratios the FPGA does not always have, and the whole point of
/// this receiver is wide spans. Narrower views come from the span zoom.
const RATE_MIN: u64 = 1_000_000;
/// Ceiling on a USB 3.0 link, and the board's own maximum: 61.44 MS/s is
/// twice the 30.72 MHz reference. At 12 bits that is 184 MB/s, which is
/// inside what the FX3 carries at SuperSpeed. Whether the host keeps up with
/// the channelizer at that rate is the host's problem and shows in the
/// dropped count, so the driver does not decide it in advance.
const RATE_MAX_USB3: u64 = 61_440_000;
/// Ceiling on a USB 2.0 link, where the same board enumerates at 480 Mb/s and
/// carries an eighth of the samples. Measured clean at 4 MS/s with no drops.
const RATE_MAX_USB2: u64 = 4_000_000;

/// The analogue LPF is set above the sample rate, so the usable span is
/// limited by the digital decimation filter rather than by the LPF.
const USABLE_RATIO: f32 = 0.85;

/// Combined RX gain range LimeSuite distributes across LNA, TIA and PGA.
const GAIN_MAX_DB: f32 = 73.0;

/// Full scale of the transmit gain LimeSuite distributes across the TX chain.
/// Less than the receive range because there are fewer stages to spread it
/// over: no LNA, and the PA driver is the last of them.
const TX_GAIN_MAX_DB: f32 = 64.0;

/// RX channel opened by default. The board has two, each with its own set of
/// RF connectors, but streaming both doubles the USB load for an antenna most
/// setups do not have, so the second one is a choice rather than a default.
const DEFAULT_CHAN: usize = 0;

/// The antenna choice that means "pick the matched port for the frequency".
const AUTO_ANTENNA: &str = "Auto";

/// Milliseconds of signal in one `LMS_RecvStream` call.
///
/// Sized in time rather than samples because the rates span a factor of
/// sixty: 32k samples is 8 ms at 4 MS/s and half a millisecond at 61.44, and
/// at the top end that is a syscall per USB frame for no benefit.
const CHUNK_MS: u64 = 10;
/// Depth of the FIFO LimeSuite fills from its USB thread, in milliseconds.
///
/// This is how long the host may be busy elsewhere before samples are lost. A
/// fixed sample count gets this backwards: it is deepest at the low rates that
/// never needed it and shallowest at the high rates that cannot survive a
/// scheduling gap.
const FIFO_MS: u64 = 200;
/// Bounds on the read size, in complex samples.
const CHUNK_MIN: usize = 16 * 1024;
const CHUNK_MAX: usize = 512 * 1024;

fn chunk_samples(rate: Sps) -> usize {
    ((rate.0 * CHUNK_MS / 1000) as usize).clamp(CHUNK_MIN, CHUNK_MAX)
}
/// How long a single read waits before it is treated as a stall.
const RECV_TIMEOUT_MS: u32 = 1000;

/// How long a send waits for room in the transmit FIFO. Longer than a FIFO
/// takes to drain at any supported rate, so a timeout means the radio has
/// stopped consuming rather than that it is briefly behind.
const SEND_TIMEOUT_MS: u32 = 1000;
/// Samples in one link packet at the 12-bit link format, which is what
/// LimeSuite's dropped and overrun counters are counted in.
const SAMPLES_PER_PACKET: u64 = 1360;
/// Consecutive timeouts tolerated before the stream is declared dead. The
/// radio produces samples continuously, so a full second of nothing means the
/// link is gone rather than that the band is quiet.
const MAX_TIMEOUTS: u32 = 5;

/// The last error LimeSuite recorded, which is the only detail its integer
/// return codes carry.
fn last_error() -> String {
    unsafe {
        let p = ffi::LMS_GetLastErrorMessage();
        if p.is_null() { String::new() } else { CStr::from_ptr(p).to_string_lossy().into_owned() }
    }
}

fn check(rc: i32, what: &'static str) -> Result<()> {
    if rc == ffi::LMS_SUCCESS {
        return Ok(());
    }
    let msg = last_error();
    let low = msg.to_ascii_lowercase();
    if low.contains("permission") || low.contains("access denied") {
        Err(Error::Permission)
    } else if low.contains("busy") || low.contains("in use") {
        Err(Error::Busy)
    } else if msg.is_empty() {
        Err(Error::other(format!("{what} failed (LimeSuite rc={rc})")))
    } else {
        Err(Error::other(format!("{what} failed: {msg}")))
    }
}

fn cstr(buf: &[std::os::raw::c_char]) -> String {
    unsafe { CStr::from_ptr(buf.as_ptr()) }.to_string_lossy().into_owned()
}

/// One LimeSDR as the library describes it, before it is opened.
///
/// The `info` string is LimeSuite's own device selector, so it is both the
/// stable identity of the unit and the argument that reopens it.
#[derive(Clone, Debug)]
pub struct Enumerated {
    pub index: usize,
    pub info: String,
    pub name: String,
    pub serial: String,
    /// True when the board is on a USB 2.0 port, which caps the sample rate.
    pub usb2: bool,
}

impl Enumerated {
    fn parse(index: usize, info: String) -> Self {
        // LimeSuite formats the string as comma-separated `key=value` pairs
        // after a leading board name:
        //   LimeSDR-USB, media=USB 2.0, module=FX3, addr=1d50:6108, serial=...
        let mut name = String::new();
        let mut serial = String::new();
        let mut usb2 = false;
        for (i, part) in info.split(',').map(str::trim).enumerate() {
            match part.split_once('=') {
                Some(("serial", v)) => serial = v.to_string(),
                Some(("media", v)) => usb2 = v.contains("2.0"),
                _ if i == 0 => name = part.to_string(),
                _ => {}
            }
        }
        Self { index, info, name, serial, usb2 }
    }

    /// Highest sample rate this board's link will carry.
    pub fn rate_max(&self) -> Sps {
        Sps(if self.usb2 { RATE_MAX_USB2 } else { RATE_MAX_USB3 })
    }

    pub fn label(&self) -> String {
        let name = if self.name.is_empty() { "LimeSDR" } else { &self.name };
        let tail = short_serial(&self.serial);
        if tail.is_empty() { name.to_string() } else { format!("{name} {tail}") }
    }
}

/// Every LimeSDR attached to the system.
pub fn enumerate() -> Vec<Enumerated> {
    if !ffi::available() {
        tracing::debug!("LimeSuite is not installed, so no LimeSDR can be opened");
        return Vec::new();
    }
    // Two calls: the first for the count, the second to fill the buffer.
    let n = unsafe { ffi::LMS_GetDeviceList(std::ptr::null_mut()) };
    if n <= 0 {
        return Vec::new();
    }
    let mut list = vec![[0 as std::os::raw::c_char; 256]; n as usize];
    let n = unsafe { ffi::LMS_GetDeviceList(list.as_mut_ptr()) };
    if n <= 0 {
        return Vec::new();
    }
    list.iter().take(n as usize).enumerate().map(|(i, s)| Enumerated::parse(i, cstr(s))).collect()
}

/// Serial tails identify a unit; the leading zeros do not.
fn short_serial(s: &str) -> String {
    let t = s.trim_start_matches('0');
    if t.len() > 8 { t[t.len() - 8..].to_string() } else { t.to_string() }
}

/// Raw device pointer. LimeSuite has no thread affinity requirement, only a
/// data-race one, which [`Handle::ctl`] resolves.
struct Raw(*mut ffi::lms_device_t);
unsafe impl Send for Raw {}
unsafe impl Sync for Raw {}

struct Handle {
    raw: Raw,
    ctl: Mutex<()>,
}

impl Handle {
    fn ptr(&self) -> *mut ffi::lms_device_t {
        self.raw.0
    }
}

impl Drop for Handle {
    fn drop(&mut self) {
        unsafe { ffi::LMS_Close(self.raw.0) };
    }
}

/// An RX antenna port, as LimeSuite indexes them.
#[derive(Clone, Debug)]
struct Antenna {
    index: usize,
    /// The receiver this path belongs to, since the list covers both.
    chan: usize,
    name: String,
    /// The connector this path comes in on, as the board is marked.
    label: String,
}

/// A TX antenna port and the band LimeSuite recommends it for.
///
/// The band is asked for rather than assumed, because the two names mean
/// opposite switch positions on the two boards: BAND1 is 30 MHz - 1.9 GHz on
/// a LimeSDR-USB and 2 - 2.6 GHz on a Mini.
#[derive(Clone, Debug)]
struct TxPort {
    index: usize,
    /// The transmitter this path belongs to, since the list covers both.
    chan: usize,
    name: String,
    /// The connector this switch position drives, as the board is marked.
    label: String,
    band: std::ops::RangeInclusive<f64>,
}

/// What a signal path is called on the board, where the board marks its
/// connectors one per path.
///
/// A LimeSDR-USB and a PCIe card bring each channel's paths out to their own
/// SMA: RX1_H, RX1_L and RX1_W for the three receive inputs of the first
/// channel, TX1_1 and TX1_2 for its two transmit outputs, and the same again
/// as RX2 and TX2 for the second. A Mini has one connector each way for all
/// of them, so there is no connector name to give and LimeSuite's own stands.
fn connector(board: &str, chan: usize, port: &str) -> String {
    let board = board.to_ascii_lowercase();
    if !(board.contains("usb") || board.contains("pcie")) {
        return port.to_string();
    }
    match port {
        "BAND1" => format!("TX{}_1", chan + 1),
        "BAND2" => format!("TX{}_2", chan + 1),
        "LNAH" => format!("RX{}_H", chan + 1),
        "LNAL" => format!("RX{}_L", chan + 1),
        "LNAW" => format!("RX{}_W", chan + 1),
        _ => port.to_string(),
    }
}

pub struct LimeSdr {
    handle: Arc<Handle>,
    info: DeviceInfo,
    /// The converter on the cable and the reference correction, which the
    /// `Device` trait does the arithmetic with.
    tuning: common::Tuning,
    center: Hz,
    rate: Sps,
    gain_db: f32,
    antennas: Vec<Antenna>,
    /// LimeSuite's name for the port to force, or `AUTO_ANTENNA` to follow
    /// the frequency. Kept as the path rather than the connector so that a
    /// forced choice survives a change of channel.
    ///
    /// Which connector the cable is in is not something a driver can work out:
    /// a board wired to RX2_H alone hears nothing on the port the frequency
    /// says it should use.
    antenna: String,
    /// RX channel in use. Each has its own H, L and W connectors.
    chan: usize,
    /// Whether to run the RX calibration after retuning.
    calibrate: bool,
    /// Whether the chip's internal test tone replaces the antenna.
    test_signal: bool,
    streaming: Arc<AtomicBool>,
    /// Where the transmitter is tuned, which is its own synthesiser and not
    /// the receiver's: this radio can listen on a repeater's output while
    /// transmitting on its input.
    tx_center: Hz,
    /// Transmit gain, starting at nothing. A transmitter that comes up at
    /// whatever the last session left behind puts power into whatever is on
    /// the port before the operator has looked at it.
    tx_gain_db: f32,
    /// TX channel in use. Each has its own connectors and its own
    /// synthesiser; the receive channel is a separate choice.
    tx_chan: usize,
    /// The transmit ports of the channel in use, with their bands.
    tx_ports: Vec<TxPort>,
    /// LimeSuite's name for the port to force, or `AUTO_ANTENNA` to follow
    /// the frequency. Kept as the switch position rather than the connector
    /// so that a forced choice survives a change of channel.
    tx_antenna: String,
    transmitting: Arc<AtomicBool>,
}

impl LimeSdr {
    /// Open by enumeration index.
    pub fn open(index: usize) -> Result<Self> {
        let found = enumerate();
        let e = found.get(index).cloned().ok_or(Error::NoDevice)?;
        Self::open_enumerated(&e)
    }

    pub fn open_first() -> Result<Self> {
        Self::open(0)
    }

    /// Open the unit whose serial or LimeSuite info string matches, else by
    /// index if `id` parses as a number.
    pub fn open_by_id(id: &str) -> Result<Self> {
        let found = enumerate();
        if let Some(e) = found.iter().find(|e| e.serial == id || e.info == id) {
            return Self::open_enumerated(e);
        }
        if let Ok(i) = id.parse::<usize>() {
            return Self::open(i);
        }
        Err(Error::NoDevice)
    }

    fn open_enumerated(e: &Enumerated) -> Result<Self> {
        let mut dev: *mut ffi::lms_device_t = std::ptr::null_mut();
        let c = std::ffi::CString::new(e.info.as_str())
            .map_err(|_| Error::other("device info string contains a NUL"))?;
        let rc = unsafe { ffi::LMS_Open(&mut dev, c.as_ptr(), std::ptr::null_mut()) };
        if rc != ffi::LMS_SUCCESS || dev.is_null() {
            let msg = last_error();
            let low = msg.to_ascii_lowercase();
            return Err(if low.contains("permission") || low.contains("access") {
                Error::Permission
            } else if low.contains("busy") || low.contains("in use") {
                Error::Busy
            } else if msg.is_empty() {
                Error::NoDevice
            } else {
                Error::other(format!("LMS_Open failed: {msg}"))
            });
        }
        let handle = Arc::new(Handle { raw: Raw(dev), ctl: Mutex::new(()) });

        // LMS_Init loads the operating configuration. Without it the chip is
        // in its datasheet reset state, which does not stream.
        check(unsafe { ffi::LMS_Init(handle.ptr()) }, "LMS_Init")?;
        check(
            unsafe { ffi::LMS_EnableChannel(handle.ptr(), ffi::LMS_CH_RX, DEFAULT_CHAN, true) },
            "LMS_EnableChannel",
        )?;
        // TX stays off. It draws current and puts the PA in a state we have no
        // reason to be in on a receive-only tool.
        let _ =
            unsafe { ffi::LMS_EnableChannel(handle.ptr(), ffi::LMS_CH_TX, DEFAULT_CHAN, false) };

        let firmware = unsafe {
            let p = ffi::LMS_GetDeviceInfo(handle.ptr());
            if p.is_null() {
                String::new()
            } else {
                let d = &*p;
                format!("fw {} hw {}", cstr(&d.firmwareVersion), cstr(&d.hardwareVersion))
            }
        };

        let channels =
            unsafe { ffi::LMS_GetNumChannels(handle.ptr(), ffi::LMS_CH_RX) }.max(1) as usize;
        // A LimeSDR-USB is 2x2 and a Mini is 1x1, so this is asked rather
        // than assumed: a board with one transmitter told it had two would
        // fail at the second stream rather than at the setting that asked
        // for it.
        let tx_channels =
            unsafe { ffi::LMS_GetNumChannels(handle.ptr(), ffi::LMS_CH_TX) }.max(1) as usize;
        let antennas = rx_antennas(&handle, channels, &e.name);
        let tx_ports = tx_ports(&handle, tx_channels, &e.name);

        let rate_max = e.rate_max().0.min(reported_rate_max(&handle).unwrap_or(u64::MAX));
        let info = DeviceInfo {
            kind: DriverKind::LimeSdr,
            id: if e.serial.is_empty() { format!("limesdr:{}", e.index) } else { e.serial.clone() },
            label: e.label(),
            tuner: if firmware.is_empty() {
                "LMS7002M".to_string()
            } else {
                format!("LMS7002M ({firmware})")
            },
            ranges: vec![TunerRange {
                range: Hz(FREQ_MIN)..=Hz(FREQ_MAX),
                label: "100 kHz - 3.8 GHz",
            }],
            rates: Vec::new(),
            rate_range: Sps(RATE_MIN)..=Sps(rate_max.max(RATE_MIN)),
            // LimeSuite's own gain control is one number that it distributes
            // across the LNA, TIA and PGA by a table in the library. Splitting
            // it here would mean reimplementing that distribution through
            // register writes, and getting it wrong shows up as a noise figure
            // nobody can account for.
            gain_stages: vec![common::GainStage {
                name: "gain".into(),
                label: "RX gain (LNA + TIA + PGA)".into(),
                range: 0.0..=GAIN_MAX_DB,
                values: Vec::new(),
                step: 1.0,
                auto: false,
            }],
            native_format: SampleFormat::Cf32,
            usable_bandwidth_ratio: USABLE_RATIO,
            tunable: true,
            tx: Some(common::TxInfo {
                ranges: vec![TunerRange {
                    range: Hz(FREQ_MIN)..=Hz(FREQ_MAX),
                    label: "100 kHz - 3.8 GHz",
                }],
                rate_range: Sps(RATE_MIN)..=Sps(rate_max.max(RATE_MIN)),
                gain_stages: vec![common::GainStage {
                    name: "gain".into(),
                    label: "TX gain (IAMP + PAD)".into(),
                    range: 0.0..=TX_GAIN_MAX_DB,
                    values: Vec::new(),
                    step: 1.0,
                    auto: false,
                }],
                native_format: SampleFormat::Cf32,
                // The whole point of this radio on a transmit path: separate
                // receive and transmit chains with their own synthesisers, so
                // an over is not a gap in the waterfall and a repeater split
                // is two frequencies at once rather than a retune.
                half_duplex: false,
                channels: tx_channels,
            }),
        };

        let mut me = Self {
            tuning: Default::default(),
            handle,
            info,
            center: Hz::mhz(100),
            rate: Sps(RATE_MIN.max(2_000_000).min(rate_max)),
            gain_db: 40.0,
            antennas,
            antenna: AUTO_ANTENNA.to_string(),
            chan: DEFAULT_CHAN,
            calibrate: false,
            test_signal: false,
            streaming: Arc::new(AtomicBool::new(false)),
            tx_center: Hz::mhz(100),
            tx_gain_db: 0.0,
            tx_chan: DEFAULT_CHAN,
            tx_ports,
            tx_antenna: AUTO_ANTENNA.to_string(),
            transmitting: Arc::new(AtomicBool::new(false)),
        };
        let rate = me.rate;
        me.set_rate(rate)?;
        me.set_center(Hz::mhz(100))?;
        me.set_gain("gain", GainMode::Manual(40.0))?;
        Ok(me)
    }

    /// The RX port to use at a given frequency.
    ///
    /// Under `Auto`, LNAL is matched for the low band and LNAH for the high
    /// one, with the crossover where LimeSuite's own Soapy driver puts it.
    /// That is only right if the antenna is on the port the frequency picks,
    /// which is why the choice can be pinned instead.
    fn antenna_for(&self, f: Hz) -> Option<&Antenna> {
        if self.antenna != AUTO_ANTENNA {
            if let Some(a) =
                self.antennas.iter().find(|a| a.name == self.antenna || a.label == self.antenna)
            {
                return Some(a);
            }
        }
        let want = if f.get() >= 1_500_000_000 { "LNAH" } else { "LNAL" };
        let on = |a: &&Antenna| a.chan == self.chan;
        self.antennas
            .iter()
            .filter(on)
            .find(|a| a.name == want)
            .or_else(|| self.antennas.iter().filter(on).find(|a| a.name == "LNAW"))
            .or_else(|| self.antennas.iter().find(on))
    }

    /// The port named, whichever channel it is on.
    fn port_named(&self, value: &str) -> Option<&Antenna> {
        self.antennas.iter().find(|a| a.label == value || a.name == value)
    }

    /// The transmit port named, whichever channel it is on.
    fn tx_port_named(&self, value: &str) -> Option<&TxPort> {
        self.tx_ports.iter().find(|p| p.label == value || p.name == value)
    }

    /// Put a freshly selected channel into the state the old one was in.
    ///
    /// Every one of these is per channel in the LMS API, so a channel change
    /// that only enabled the new one would come up at whatever gain and
    /// frequency `LMS_Init` left it at.
    fn configure_channel(&mut self) -> Result<()> {
        check(
            unsafe { ffi::LMS_EnableChannel(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, true) },
            "LMS_EnableChannel",
        )?;
        self.apply_antenna(self.center)?;
        check(
            unsafe {
                ffi::LMS_SetLOFrequency(
                    self.handle.ptr(),
                    ffi::LMS_CH_RX,
                    self.chan,
                    self.center.0 as f64,
                )
            },
            "LMS_SetLOFrequency",
        )?;
        self.apply_lpf(self.rate)?;
        check(
            unsafe {
                ffi::LMS_SetGaindB(
                    self.handle.ptr(),
                    ffi::LMS_CH_RX,
                    self.chan,
                    self.gain_db.round() as u32,
                )
            },
            "LMS_SetGaindB",
        )
    }

    fn apply_antenna(&self, f: Hz) -> Result<()> {
        let Some(a) = self.antenna_for(f) else { return Ok(()) };
        check(
            unsafe { ffi::LMS_SetAntenna(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, a.index) },
            "LMS_SetAntenna",
        )
    }

    /// The TX port to use at a given frequency.
    fn tx_port_for(&self, f: Hz) -> Option<&TxPort> {
        pick_tx_port(&self.tx_ports, &self.tx_antenna, f, self.tx_chan)
    }

    /// Put the transmit chain of the channel now in use where the settings
    /// say, so a key pressed later finds it tuned and matched.
    fn configure_tx_channel(&self) -> Result<()> {
        check(
            unsafe {
                ffi::LMS_SetLOFrequency(
                    self.handle.ptr(),
                    ffi::LMS_CH_TX,
                    self.tx_chan,
                    self.tx_center.0 as f64,
                )
            },
            "LMS_SetLOFrequency",
        )?;
        self.apply_tx_antenna(self.tx_center)?;
        check(
            unsafe {
                ffi::LMS_SetGaindB(
                    self.handle.ptr(),
                    ffi::LMS_CH_TX,
                    self.tx_chan,
                    self.tx_gain_db.round() as u32,
                )
            },
            "LMS_SetGaindB",
        )
    }

    /// Point the transmit chain at a port.
    ///
    /// Nothing else does. `LMS_Init` leaves the chip on BAND1 and never
    /// touches the board's RF switch, which on a LimeSDR-USB is FPGA register
    /// 0x17 and comes up selecting the other output: the PA drives a pin the
    /// switch has disconnected, so the transmitter runs and the connector is
    /// dead. The Mini picks a port itself on every retune, so this only
    /// repeats what it already did.
    fn apply_tx_antenna(&self, f: Hz) -> Result<()> {
        let Some(p) = self.tx_port_for(f) else { return Ok(()) };
        check(
            unsafe {
                ffi::LMS_SetAntenna(self.handle.ptr(), ffi::LMS_CH_TX, self.tx_chan, p.index)
            },
            "LMS_SetAntenna",
        )
    }

    /// Set the analogue low-pass filter to sit just outside the sample rate.
    ///
    /// Placing it at the rate rather than inside it keeps the band edges from
    /// rolling off within the span we display, and the digital decimation
    /// filter is what actually rejects the aliases.
    fn apply_lpf(&self, rate: Sps) -> Result<()> {
        let mut range = ffi::lms_range_t { min: 0.0, max: 0.0, step: 0.0 };
        let rc = unsafe { ffi::LMS_GetLPFBWRange(self.handle.ptr(), ffi::LMS_CH_RX, &mut range) };
        let want = rate.0 as f64 * 1.3;
        let bw = if rc == ffi::LMS_SUCCESS && range.max > range.min {
            want.clamp(range.min, range.max)
        } else {
            want
        };
        check(
            unsafe { ffi::LMS_SetLPFBW(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, bw) },
            "LMS_SetLPFBW",
        )
    }

    /// Run the RX calibration, which corrects DC offset and IQ imbalance.
    ///
    /// Best effort on purpose. It takes the best part of a second, it needs
    /// the current LO and bandwidth, and it fails routinely near the ends of
    /// the tuning range. A failed calibration leaves a centre spur and an
    /// image, not a broken receiver, so it must never fail a retune.
    fn try_calibrate(&self) {
        if !self.calibrate || self.streaming.load(Ordering::SeqCst) {
            return;
        }
        let bw = (self.rate.0 as f64 * 1.3).max(2.5e6);
        let rc = unsafe { ffi::LMS_Calibrate(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, bw, 0) };
        if rc != ffi::LMS_SUCCESS {
            tracing::warn!(error = %last_error(), "LimeSDR RX calibration failed");
        }
    }

    /// Frequency the LO actually landed on after the PLL rounded.
    pub fn actual_center(&self) -> Hz {
        let mut f = 0.0f64;
        let rc = unsafe {
            ffi::LMS_GetLOFrequency(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, &mut f)
        };
        if rc == ffi::LMS_SUCCESS { Hz(f.round() as u64) } else { self.center }
    }

    pub fn actual_rate(&self) -> Sps {
        let (mut host, mut rf) = (0.0f64, 0.0f64);
        let rc = unsafe {
            ffi::LMS_GetSampleRate(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, &mut host, &mut rf)
        };
        if rc == ffi::LMS_SUCCESS { Sps(host.round() as u64) } else { self.rate }
    }

    /// Temperature of the LMS7002M die in degrees Celsius.
    pub fn chip_temperature(&self) -> Option<f64> {
        let mut t = 0.0f64;
        let rc = unsafe { ffi::LMS_GetChipTemperature(self.handle.ptr(), 0, &mut t) };
        (rc == ffi::LMS_SUCCESS).then_some(t)
    }
}

/// Every receive connector the board has, across its channels.
fn rx_antennas(handle: &Handle, channels: usize, board: &str) -> Vec<Antenna> {
    (0..channels).flat_map(|c| rx_antennas_on(handle, c, board)).collect()
}

fn rx_antennas_on(handle: &Handle, chan: usize, board: &str) -> Vec<Antenna> {
    let n = unsafe {
        ffi::LMS_GetAntennaList(handle.ptr(), ffi::LMS_CH_RX, chan, std::ptr::null_mut())
    };
    if n <= 0 {
        return Vec::new();
    }
    let mut list = vec![[0 as std::os::raw::c_char; 16]; n as usize];
    let n =
        unsafe { ffi::LMS_GetAntennaList(handle.ptr(), ffi::LMS_CH_RX, chan, list.as_mut_ptr()) };
    list.iter()
        .take(n.max(0) as usize)
        .enumerate()
        .map(|(i, s)| {
            let name = cstr(s);
            let label = connector(board, chan, &name);
            Antenna { index: i, chan, name, label }
        })
        // Index 0 is "NONE", which disconnects the receiver from every port,
        // and LB1/LB2 are the loopback taps from the transmitter rather than
        // connectors. Offering either is offering a way to hear nothing.
        .filter(|a| a.name.starts_with("LNA"))
        .collect()
}

/// Every transmit connector the board has, across its channels.
fn tx_ports(handle: &Handle, channels: usize, board: &str) -> Vec<TxPort> {
    (0..channels).flat_map(|c| tx_ports_on(handle, c, board)).collect()
}

fn tx_ports_on(handle: &Handle, chan: usize, board: &str) -> Vec<TxPort> {
    let n = unsafe {
        ffi::LMS_GetAntennaList(handle.ptr(), ffi::LMS_CH_TX, chan, std::ptr::null_mut())
    };
    if n <= 0 {
        return Vec::new();
    }
    let mut list = vec![[0 as std::os::raw::c_char; 16]; n as usize];
    let n =
        unsafe { ffi::LMS_GetAntennaList(handle.ptr(), ffi::LMS_CH_TX, chan, list.as_mut_ptr()) };
    list.iter()
        .take(n.max(0) as usize)
        .enumerate()
        // "NONE" disconnects the transmitter, and the Mini lists an "Auto"
        // that is the library doing what `tx_port_for` does.
        .filter(|(_, s)| cstr(*s).starts_with("BAND"))
        .map(|(i, s)| {
            let mut range = ffi::lms_range_t { min: 0.0, max: 0.0, step: 0.0 };
            let rc =
                unsafe { ffi::LMS_GetAntennaBW(handle.ptr(), ffi::LMS_CH_TX, chan, i, &mut range) };
            let band = if rc == ffi::LMS_SUCCESS && range.max > range.min {
                range.min..=range.max
            } else {
                0.0..=0.0
            };
            let name = cstr(s);
            let label = connector(board, chan, &name);
            TxPort { index: i, chan, name, label, band }
        })
        .collect()
}

/// The port a frequency should leave by, or the one pinned by name.
///
/// `chan` is the transmitter in use, which is what `Auto` chooses within: a
/// pinned port names its own channel and moving to it moves the chain.
fn pick_tx_port<'a>(ports: &'a [TxPort], forced: &str, f: Hz, chan: usize) -> Option<&'a TxPort> {
    if forced != AUTO_ANTENNA {
        if let Some(p) = ports.iter().find(|p| p.name == forced || p.label == forced) {
            return Some(p);
        }
    }
    let hz = f.0 as f64;
    let on = |p: &&TxPort| p.chan == chan;
    ports
        .iter()
        .filter(on)
        .find(|p| p.band.contains(&hz))
        // Off the end of both bands (below 30 MHz, or in the gap between them)
        // the nearest one still radiates, where no band at all does not.
        .or_else(|| {
            ports
                .iter()
                .filter(on)
                .min_by(|a, b| band_distance(&a.band, hz).total_cmp(&band_distance(&b.band, hz)))
        })
}

/// How far a frequency is from a band, and zero inside it.
fn band_distance(band: &std::ops::RangeInclusive<f64>, hz: f64) -> f64 {
    if hz < *band.start() {
        *band.start() - hz
    } else if hz > *band.end() {
        hz - *band.end()
    } else {
        0.0
    }
}

fn reported_rate_max(handle: &Handle) -> Option<u64> {
    let mut range = ffi::lms_range_t { min: 0.0, max: 0.0, step: 0.0 };
    let rc = unsafe { ffi::LMS_GetSampleRateRange(handle.ptr(), ffi::LMS_CH_RX, &mut range) };
    (rc == ffi::LMS_SUCCESS && range.max > 0.0).then_some(range.max as u64)
}

impl Device for LimeSdr {
    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }

    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }

    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    /// A LimeSDR retune recalibrates the VCO inside `LMS_SetLOFrequency`,
    /// which takes tens of milliseconds, and the stream goes on delivering
    /// what the board collected through it. Sixty covers that with room:
    /// three waterfall rows at the default rate, against a bright stripe
    /// across every row otherwise.
    fn settle(&self) -> std::time::Duration {
        std::time::Duration::from_millis(60)
    }

    fn set_center(&mut self, f: Hz) -> Result<()> {
        if !self.info.covers(f) {
            return Err(Error::FreqOutOfRange { req: f, lo: Hz(FREQ_MIN), hi: Hz(FREQ_MAX) });
        }
        let _g = self.handle.ctl.lock().unwrap();
        let prev = self.center;
        self.center = f;
        // The port has to follow the frequency, or tuning across 1.5 GHz
        // leaves the receiver on a port that is not matched there and the
        // signal quietly drops 20 dB.
        if self.antenna_for(prev).map(|a| a.index) != self.antenna_for(f).map(|a| a.index) {
            self.apply_antenna(f)?;
        }
        check(
            unsafe {
                ffi::LMS_SetLOFrequency(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, f.0 as f64)
            },
            "LMS_SetLOFrequency",
        )
        .inspect_err(|_| self.center = prev)?;
        drop(_g);
        self.try_calibrate();
        Ok(())
    }

    fn center(&self) -> Hz {
        self.center
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        if !self.info.rate_range.contains(&r) {
            return Err(Error::RateUnsupported { req: r });
        }
        let _g = self.handle.ctl.lock().unwrap();
        // Oversampling 0 lets LimeSuite pick the highest ratio the decimation
        // chain supports at this rate, which is what keeps the ADC noise
        // spread out rather than folded into the span.
        check(
            unsafe { ffi::LMS_SetSampleRate(self.handle.ptr(), r.0 as f64, 0) },
            "LMS_SetSampleRate",
        )?;
        self.rate = r;
        self.apply_lpf(r)?;
        drop(_g);
        self.try_calibrate();
        Ok(())
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        // "tuner" is what a caller that wants one number for the whole front
        // end asks for, and on this radio that is the only number there is.
        if stage != "gain" && stage != "tuner" {
            return Err(Error::other(format!("no gain stage named {stage:?}")));
        }
        // The LMS7002M has no AGC of its own that LimeSuite exposes, so "auto"
        // is a fixed operating point with headroom rather than a no-op: about
        // half scale, which is where the front end is linear.
        let db = match mode {
            GainMode::Auto => 40.0,
            GainMode::Manual(db) => db,
        }
        .clamp(0.0, GAIN_MAX_DB);
        let _g = self.handle.ctl.lock().unwrap();
        check(
            unsafe {
                ffi::LMS_SetGaindB(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, db.round() as u32)
            },
            "LMS_SetGaindB",
        )?;
        self.gain_db = db.round();
        Ok(())
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        // Read back rather than trust: LimeSuite quantises to whole dB and to
        // what the three stages can actually add up to.
        let mut db = 0u32;
        let rc =
            unsafe { ffi::LMS_GetGaindB(self.handle.ptr(), ffi::LMS_CH_RX, self.chan, &mut db) };
        let v = if rc == ffi::LMS_SUCCESS { db as f32 } else { self.gain_db };
        vec![("gain".to_string(), GainMode::Manual(v))]
    }

    fn toggles(&self) -> Vec<common::Toggle> {
        vec![common::Toggle {
            name: "calibrate".into(),
            label: "Calibrate on retune".into(),
            help: "Runs the RX DC and IQ imbalance calibration after every tune or rate change. It removes the centre spur and the mirror image, and it costs the best part of a second each time, so a scan that steps frequency will crawl with it on."
                .into(),
            on: self.calibrate,
        },
        common::Toggle {
            name: "test_signal".into(),
            label: "Internal test tone".into(),
            help: "Replaces the antenna with the chip's own NCO tone at an eighth of the sample rate. It answers one question and it is the right question when a radio hears nothing: a tone that appears proves the converter and the whole path back to here, so what is left is the front end or the cable."
                .into(),
            on: self.test_signal,
        }]
    }

    fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
        match name {
            "calibrate" => {
                self.calibrate = on;
                self.try_calibrate();
            }
            "test_signal" => {
                let sig = if on {
                    ffi::lms_testsig_t_LMS_TESTSIG_NCODIV8
                } else {
                    ffi::lms_testsig_t_LMS_TESTSIG_NONE
                };
                let _g = self.handle.ctl.lock().unwrap();
                check(
                    unsafe {
                        ffi::LMS_SetTestSignal(
                            self.handle.ptr(),
                            ffi::LMS_CH_RX,
                            self.chan,
                            sig,
                            0,
                            0,
                        )
                    },
                    "LMS_SetTestSignal",
                )?;
                self.test_signal = on;
            }
            _ => return Err(Error::other(format!("no setting named {name:?}"))),
        }
        Ok(())
    }

    fn choices(&self) -> Vec<common::Choice> {
        let mut v = Vec::new();
        if !self.antennas.is_empty() {
            let mut options = vec![AUTO_ANTENNA.to_string()];
            options.extend(self.antennas.iter().map(|a| a.label.clone()));
            v.push(common::Choice {
                name: "antenna".into(),
                label: "Receive port".into(),
                help: "Which socket the aerial is in, by the name printed beside it. The board has three inputs per receiver and they are separately matched: H is the high band, L the low one and W broadband, at several dB of noise figure. Picking a port on the other receiver moves the receiver, which stops and restarts the stream. Auto follows the frequency on the receiver in use, taking L below 1.5 GHz and H above, which is only right if the aerial is on that port."
                    .into(),
                options,
                selected: self.antenna.clone(),
            });
        }
        if !self.tx_ports.is_empty() {
            let mut options = vec![AUTO_ANTENNA.to_string()];
            options.extend(self.tx_ports.iter().map(|p| p.label.clone()));
            v.push(common::Choice {
                name: "tx_antenna".into(),
                label: "Transmit port".into(),
                help: "Which socket the transmitter drives, by the name printed beside it. The board has two outputs per transmitter, matched for the low and the high band, and a port that does not match the frequency radiates a fraction of the power. Picking a port on the other transmitter moves the transmitter, which is refused while a key is down. Auto takes the port LimeSuite says is matched at the transmit frequency."
                    .into(),
                options,
                selected: self.tx_antenna.clone(),
            });
        }
        v
    }

    fn set_choice(&mut self, name: &str, value: &str) -> Result<()> {
        match name {
            "antenna" => {
                let chan = match self.port_named(value) {
                    Some(a) => a.chan,
                    None if value == AUTO_ANTENNA => self.chan,
                    None => {
                        return Err(Error::other(format!("no antenna port named {value:?}")));
                    }
                };
                let handle = self.handle.clone();
                let _g = handle.ctl.lock().unwrap();
                let was = std::mem::replace(&mut self.antenna, value.to_string());
                if chan == self.chan {
                    return self.apply_antenna(self.center);
                }
                let old = self.chan;
                self.chan = chan;
                if let Err(e) = self.configure_channel() {
                    self.chan = old;
                    self.antenna = was;
                    return Err(e);
                }
                // Only after the new one is up, so a failure leaves a working
                // receiver rather than none at all.
                let _ = unsafe {
                    ffi::LMS_EnableChannel(self.handle.ptr(), ffi::LMS_CH_RX, old, false)
                };
                Ok(())
            }
            "tx_antenna" => {
                let chan = match self.tx_port_named(value) {
                    Some(p) => p.chan,
                    None if value == AUTO_ANTENNA => self.tx_chan,
                    None => {
                        return Err(Error::other(format!("no transmit port named {value:?}")));
                    }
                };
                // The channel is what the transmit stream was set up on, and
                // moving it under a key that is down puts the samples on a
                // chain that is no longer tuned or matched.
                if chan != self.tx_chan && self.transmitting.load(Ordering::SeqCst) {
                    return Err(Error::Busy);
                }
                let handle = self.handle.clone();
                let _g = handle.ctl.lock().unwrap();
                let was = std::mem::replace(&mut self.tx_antenna, value.to_string());
                if chan == self.tx_chan {
                    return self.apply_tx_antenna(self.tx_center);
                }
                let old = self.tx_chan;
                self.tx_chan = chan;
                if let Err(e) = self.configure_tx_channel() {
                    self.tx_chan = old;
                    self.tx_antenna = was;
                    return Err(e);
                }
                let _ = unsafe {
                    ffi::LMS_EnableChannel(self.handle.ptr(), ffi::LMS_CH_TX, old, false)
                };
                Ok(())
            }
            _ => Err(Error::other(format!("no setting named {name:?}"))),
        }
    }

    fn choice_needs_restart(&self, name: &str, value: &str) -> bool {
        // The channel is a property of the stream LimeSuite set up, not a
        // setting on it, so a port on the other receiver means tearing the
        // stream down around the change. The transmit stream is a separate
        // one the receiver is not holding, and a port that moves it is
        // refused while a key is down instead.
        name == "antenna" && self.port_named(value).is_some_and(|a| a.chan != self.chan)
    }

    fn rate_needs_restart(&self) -> bool {
        // LMS_SetSampleRate reconfigures the decimation chain the running
        // stream is fed by, so the stream has to go down around it.
        true
    }

    fn set_tx_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        if stage != "gain" && !stage.is_empty() {
            return Err(Error::other(format!("no transmit gain stage named {stage}")));
        }
        // No transmit AGC, and guessing an operating point for a transmitter
        // means guessing how much power to radiate: auto is the bottom.
        let db = match mode {
            GainMode::Auto => 0.0,
            GainMode::Manual(db) => db,
        }
        .clamp(0.0, TX_GAIN_MAX_DB);
        let _g = self.handle.ctl.lock().unwrap();
        check(
            unsafe {
                ffi::LMS_SetGaindB(
                    self.handle.ptr(),
                    ffi::LMS_CH_TX,
                    self.tx_chan,
                    db.round() as u32,
                )
            },
            "LMS_SetGaindB",
        )?;
        self.tx_gain_db = db;
        Ok(())
    }

    fn tx_gains(&self) -> Vec<(String, GainMode)> {
        let mut db = 0u32;
        let rc =
            unsafe { ffi::LMS_GetGaindB(self.handle.ptr(), ffi::LMS_CH_TX, self.tx_chan, &mut db) };
        let v = if rc == ffi::LMS_SUCCESS { db as f32 } else { self.tx_gain_db };
        vec![("gain".into(), GainMode::Manual(v))]
    }

    fn set_tx_center(&mut self, f: Hz) -> Result<()> {
        if f.0 < FREQ_MIN || f.0 > FREQ_MAX {
            return Err(Error::FreqOutOfRange { req: f, lo: Hz(FREQ_MIN), hi: Hz(FREQ_MAX) });
        }
        let _g = self.handle.ctl.lock().unwrap();
        check(
            unsafe {
                ffi::LMS_SetLOFrequency(self.handle.ptr(), ffi::LMS_CH_TX, self.tx_chan, f.0 as f64)
            },
            "LMS_SetLOFrequency",
        )?;
        self.tx_center = f;
        self.apply_tx_antenna(f)?;
        Ok(())
    }

    fn tx_center(&self) -> Hz {
        self.tx_center
    }

    fn start_tx(&mut self) -> Result<Box<dyn TxStream>> {
        if self.transmitting.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let start = || -> Result<ffi::lms_stream_t> {
            let g = self.handle.ctl.lock().unwrap();
            check(
                unsafe {
                    ffi::LMS_EnableChannel(self.handle.ptr(), ffi::LMS_CH_TX, self.tx_chan, true)
                },
                "LMS_EnableChannel",
            )?;
            // The transmit chain comes up wherever LMS_Init left it, and its
            // synthesiser is not the receiver's, so it is tuned here even
            // when both ends are on the same frequency.
            check(
                unsafe {
                    ffi::LMS_SetLOFrequency(
                        self.handle.ptr(),
                        ffi::LMS_CH_TX,
                        self.tx_chan,
                        self.tx_center.0 as f64,
                    )
                },
                "LMS_SetLOFrequency",
            )?;
            self.apply_tx_antenna(self.tx_center)?;
            check(
                unsafe {
                    ffi::LMS_SetGaindB(
                        self.handle.ptr(),
                        ffi::LMS_CH_TX,
                        self.tx_chan,
                        self.tx_gain_db.round() as u32,
                    )
                },
                "LMS_SetGaindB",
            )?;
            drop(g);

            let chunk = chunk_samples(self.rate);
            let mut stream = ffi::lms_stream_t {
                handle: 0,
                isTx: true,
                channel: self.tx_chan as u32,
                fifoSize: (self.rate.0 * FIFO_MS / 1000).max(chunk as u64 * 4) as u32,
                throughputVsLatency: if self.rate.0 > 20_000_000 { 1.0 } else { 0.5 },
                dataFmt: ffi::LMS_FMT_F32,
                linkFmt: ffi::LMS_LINK_FMT_DEFAULT,
            };
            let g = self.handle.ctl.lock().unwrap();
            check(
                unsafe { ffi::LMS_SetupStream(self.handle.ptr(), &mut stream) },
                "LMS_SetupStream",
            )?;
            drop(g);
            if let Err(e) = check(unsafe { ffi::LMS_StartStream(&mut stream) }, "LMS_StartStream") {
                unsafe { ffi::LMS_DestroyStream(self.handle.ptr(), &mut stream) };
                return Err(e);
            }
            Ok(stream)
        };

        match start() {
            Ok(stream) => Ok(Box::new(LimeTxStream {
                handle: self.handle.clone(),
                stream: StreamPtr(stream),
                rate: self.rate,
                chan: self.tx_chan,
                underruns: AtomicU64::new(0),
                stopped: false,
                transmitting: self.transmitting.clone(),
            })),
            Err(e) => {
                let _ = unsafe {
                    ffi::LMS_EnableChannel(self.handle.ptr(), ffi::LMS_CH_TX, self.tx_chan, false)
                };
                self.transmitting.store(false, Ordering::SeqCst);
                Err(e)
            }
        }
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }

        let chunk = chunk_samples(self.rate);
        let mut stream = ffi::lms_stream_t {
            handle: 0,
            isTx: false,
            channel: self.chan as u32,
            fifoSize: (self.rate.0 * FIFO_MS / 1000).max(chunk as u64 * 4) as u32,
            // Bias towards throughput once the bus is carrying tens of
            // megasamples: the transfers have to be large enough that the
            // per-transfer cost stops mattering, and the added latency is a
            // few milliseconds of waterfall lag.
            throughputVsLatency: if self.rate.0 > 20_000_000 { 1.0 } else { 0.5 },
            dataFmt: ffi::LMS_FMT_F32,
            linkFmt: ffi::LMS_LINK_FMT_DEFAULT,
        };

        let g = self.handle.ctl.lock().unwrap();
        let setup = check(
            unsafe { ffi::LMS_SetupStream(self.handle.ptr(), &mut stream) },
            "LMS_SetupStream",
        );
        drop(g);
        if let Err(e) = setup {
            self.streaming.store(false, Ordering::SeqCst);
            return Err(e);
        }
        if let Err(e) = check(unsafe { ffi::LMS_StartStream(&mut stream) }, "LMS_StartStream") {
            unsafe { ffi::LMS_DestroyStream(self.handle.ptr(), &mut stream) };
            self.streaming.store(false, Ordering::SeqCst);
            return Err(e);
        }

        Ok(Box::new(LimeStream {
            handle: self.handle.clone(),
            stream: StreamPtr(stream),
            center: self.center,
            rate: self.rate,
            chunk,
            seq: 0,
            dropped: AtomicU64::new(0),
            stopped: false,
            streaming: self.streaming.clone(),
        }))
    }
}

/// LimeSuite's stream descriptor is a plain struct holding an opaque handle;
/// the library's own locking is what makes concurrent use of it safe.
struct StreamPtr(ffi::lms_stream_t);
unsafe impl Send for StreamPtr {}

pub struct LimeStream {
    /// Keeps the device open for as long as the stream exists.
    handle: Arc<Handle>,
    stream: StreamPtr,
    center: Hz,
    rate: Sps,
    /// Samples asked for per read, fixed for the life of the stream because
    /// the rate cannot change under it.
    chunk: usize,
    seq: u64,
    dropped: AtomicU64,
    stopped: bool,
    streaming: Arc<AtomicBool>,
}

impl LimeStream {
    /// Fold LimeSuite's since-last-call counters into the running total.
    ///
    /// `LMS_GetStreamStatus` resets `overrun` and `droppedPackets` on every
    /// read, so the only way to have a total is to accumulate it, and the only
    /// caller that may read the status is this one.
    ///
    /// Both counters are in packets rather than samples, so they are scaled to
    /// what a packet holds. The trait's contract is samples, and a figure a
    /// thousand times smaller than every other driver's is worse than none.
    fn poll_status(&self) {
        let mut st = ffi::lms_stream_status_t {
            active: false,
            fifoFilledCount: 0,
            fifoSize: 0,
            underrun: 0,
            overrun: 0,
            droppedPackets: 0,
            sampleRate: 0.0,
            linkRate: 0.0,
            timestamp: 0,
        };
        let p = &self.stream.0 as *const ffi::lms_stream_t as *mut ffi::lms_stream_t;
        if unsafe { ffi::LMS_GetStreamStatus(p, &mut st) } == ffi::LMS_SUCCESS {
            let lost = (st.overrun as u64 + st.droppedPackets as u64) * SAMPLES_PER_PACKET;
            if lost > 0 {
                self.dropped.fetch_add(lost, Ordering::Relaxed);
            }
        }
    }
}

impl RxStream for LimeStream {
    fn read(&mut self) -> Result<IqBuf> {
        let mut samples: Vec<C32> = Vec::with_capacity(self.chunk);
        for _ in 0..MAX_TIMEOUTS {
            if self.stopped {
                return Err(Error::Disconnected);
            }
            // `C32` is `#[repr(C)]` over two f32, which is exactly the
            // interleaved layout LMS_FMT_F32 writes, so the FIFO is copied
            // straight into the buffer the rest of the graph consumes.
            let n = unsafe {
                ffi::LMS_RecvStream(
                    &mut self.stream.0,
                    samples.as_mut_ptr().cast(),
                    self.chunk,
                    std::ptr::null_mut(),
                    RECV_TIMEOUT_MS,
                )
            };
            if n < 0 {
                return Err(Error::other(format!("LMS_RecvStream failed: {}", last_error())));
            }
            if n == 0 {
                continue;
            }
            // Safety: LimeSuite wrote `n` complex samples into the buffer, and
            // `n` never exceeds the capacity reserved above.
            unsafe { samples.set_len(n as usize) };
            self.poll_status();
            let count = samples.len() as u64;
            let buf = IqBuf::new(samples, self.center, self.rate, self.seq);
            self.seq += count;
            return Ok(buf);
        }
        Err(Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        if !self.stopped {
            self.stopped = true;
            unsafe { ffi::LMS_StopStream(&mut self.stream.0) };
        }
    }
}

impl Drop for LimeStream {
    fn drop(&mut self) {
        self.stop();
        unsafe { ffi::LMS_DestroyStream(self.handle.ptr(), &mut self.stream.0) };
        self.streaming.store(false, Ordering::SeqCst);
    }
}

/// The transmit side of a full duplex radio.
///
/// Runs alongside [`LimeStream`] rather than instead of it: the two are
/// separate chains on the chip with their own synthesisers, so an over does
/// not interrupt reception and the two ends of a repeater pair can be tuned
/// independently.
pub struct LimeTxStream {
    handle: Arc<Handle>,
    stream: StreamPtr,
    rate: Sps,
    chan: usize,
    underruns: AtomicU64,
    stopped: bool,
    transmitting: Arc<AtomicBool>,
}

impl LimeTxStream {
    /// Samples the FIFO still holds, which is what a drain waits on.
    fn queued(&self) -> u64 {
        let mut st = ffi::lms_stream_status_t {
            active: false,
            fifoFilledCount: 0,
            fifoSize: 0,
            underrun: 0,
            overrun: 0,
            droppedPackets: 0,
            sampleRate: 0.0,
            linkRate: 0.0,
            timestamp: 0,
        };
        let p = &self.stream.0 as *const ffi::lms_stream_t as *mut ffi::lms_stream_t;
        if unsafe { ffi::LMS_GetStreamStatus(p, &mut st) } != ffi::LMS_SUCCESS {
            return 0;
        }
        // The counter resets on every read, so the total has to accumulate
        // here, exactly as the receive side's does.
        if st.underrun > 0 {
            self.underruns.fetch_add(st.underrun as u64, Ordering::Relaxed);
        }
        st.fifoFilledCount as u64
    }
}

impl TxStream for LimeTxStream {
    fn write(&mut self, buf: &IqBuf) -> Result<()> {
        if self.stopped {
            return Err(Error::Disconnected);
        }
        // A block at another rate would go out at this one, stretched or
        // compressed: the wrong bandwidth on the wrong frequency.
        if buf.rate != self.rate {
            return Err(Error::RateUnsupported { req: buf.rate });
        }
        if buf.samples.is_empty() {
            return Ok(());
        }
        let mut sent = 0usize;
        while sent < buf.samples.len() {
            // `C32` is two f32, which is the interleaved layout LMS_FMT_F32
            // reads, so the graph's own buffer goes straight to the radio.
            let n = unsafe {
                ffi::LMS_SendStream(
                    &mut self.stream.0,
                    buf.samples[sent..].as_ptr().cast(),
                    buf.samples.len() - sent,
                    std::ptr::null(),
                    SEND_TIMEOUT_MS,
                )
            };
            if n < 0 {
                return Err(Error::other(format!("LMS_SendStream failed: {}", last_error())));
            }
            if n == 0 {
                // The FIFO is full and stayed full for the timeout, which is
                // the radio not consuming: it has gone away rather than being
                // busy, since a running transmitter drains at the sample rate.
                return Err(Error::Disconnected);
            }
            sent += n as usize;
        }
        self.queued();
        Ok(())
    }

    fn underruns(&self) -> u64 {
        self.queued();
        self.underruns.load(Ordering::Relaxed)
    }

    fn drain(&mut self, timeout: std::time::Duration) -> bool {
        let deadline = std::time::Instant::now() + timeout;
        while std::time::Instant::now() < deadline {
            if self.queued() == 0 {
                return true;
            }
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        false
    }

    fn stop(&mut self) {
        if self.stopped {
            return;
        }
        self.stopped = true;
        unsafe { ffi::LMS_StopStream(&mut self.stream.0) };
        // Off, not idle: an enabled transmit chain draws current and leaves
        // the driver stage biased with nothing to send.
        let _g = self.handle.ctl.lock().unwrap();
        let _ =
            unsafe { ffi::LMS_EnableChannel(self.handle.ptr(), ffi::LMS_CH_TX, self.chan, false) };
    }
}

impl Drop for LimeTxStream {
    fn drop(&mut self) {
        // The tail first: dropping with the FIFO full cuts the end of the
        // transmission off mid-word.
        self.drain(std::time::Duration::from_millis(500));
        self.stop();
        unsafe { ffi::LMS_DestroyStream(self.handle.ptr(), &mut self.stream.0) };
        self.transmitting.store(false, Ordering::SeqCst);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_info_string_yields_a_serial_and_the_link_speed() {
        let e = Enumerated::parse(
            0,
            "LimeSDR-USB, media=USB 2.0, module=FX3, addr=1d50:6108, serial=00090726074D181E"
                .into(),
        );
        assert_eq!(e.name, "LimeSDR-USB");
        assert_eq!(e.serial, "00090726074D181E");
        assert!(e.usb2);
        assert_eq!(e.rate_max(), Sps(RATE_MAX_USB2));
        assert_eq!(e.label(), "LimeSDR-USB 074D181E");
    }

    #[test]
    fn a_usb3_board_is_allowed_the_higher_rate() {
        let e = Enumerated::parse(0, "LimeSDR-USB, media=USB 3.0, serial=0001".into());
        assert!(!e.usb2);
        assert_eq!(e.rate_max(), Sps(RATE_MAX_USB3));
    }

    #[test]
    fn an_info_string_without_a_serial_still_gives_a_usable_label() {
        let e = Enumerated::parse(2, "LimeSDR-Mini, media=USB 3.0".into());
        assert_eq!(e.label(), "LimeSDR-Mini");
        assert_eq!(e.index, 2);
    }

    #[test]
    fn a_complex_sample_is_two_floats_with_no_padding() {
        // The read path writes LimeSuite's interleaved f32 straight into a
        // Vec<C32>. If that ever stops being the same layout, every sample
        // after the first is garbage.
        assert_eq!(std::mem::size_of::<C32>(), 2 * std::mem::size_of::<f32>());
        assert_eq!(std::mem::align_of::<C32>(), std::mem::align_of::<f32>());
    }

    /// The two boards give the same two names to opposite switch positions,
    /// so the choice is made from the bands LimeSuite reports rather than
    /// from the names: `LMS7_Device::GetTxPathBand` for the USB board against
    /// `LMS7_LimeSDR_mini`'s override of it.
    #[test]
    fn a_transmit_port_is_chosen_by_its_band_and_not_its_name() {
        let port = |i, chan, n: &str, lo: f64, hi: f64| TxPort {
            index: i,
            chan,
            name: n.into(),
            label: connector("LimeSDR-USB", chan, n),
            band: lo..=hi,
        };
        let usb = [port(1, 0, "BAND1", 30e6, 1.9e9), port(2, 0, "BAND2", 2e9, 2.6e9)];
        let mini = [port(1, 0, "BAND1", 2e9, 2.6e9), port(2, 0, "BAND2", 30e6, 1.9e9)];

        let at = |ports: &[TxPort], hz: u64| {
            pick_tx_port(ports, AUTO_ANTENNA, Hz(hz), 0).unwrap().name.clone()
        };
        assert_eq!(at(&usb, 145_500_000), "BAND1");
        assert_eq!(at(&mini, 145_500_000), "BAND2");
        assert_eq!(at(&usb, 2_400_000_000), "BAND2");
        assert_eq!(at(&mini, 2_400_000_000), "BAND1");

        // In the gap between the bands, and below the bottom of both, the
        // nearest port is still a port.
        assert_eq!(at(&usb, 1_950_000_000), "BAND1");
        assert_eq!(at(&usb, 10_000_000), "BAND1");

        // A pinned port wins at any frequency; an unknown name falls back.
        assert_eq!(pick_tx_port(&usb, "BAND2", Hz::mhz(145), 0).unwrap().index, 2);
        assert_eq!(pick_tx_port(&usb, "BAND9", Hz::mhz(145), 0).unwrap().index, 1);
        assert!(pick_tx_port(&[], AUTO_ANTENNA, Hz::mhz(145), 0).is_none());

        // And the connector the board is marked with names the same port as
        // the switch position behind it, so a setting written either way
        // reaches it.
        assert_eq!(pick_tx_port(&usb, "TX1_2", Hz::mhz(145), 0).unwrap().index, 2);
    }

    /// A pinned connector names its own transmitter; Auto stays on the one
    /// in use rather than wandering to the other board half.
    #[test]
    fn a_transmit_port_carries_the_channel_it_is_on() {
        let port = |i, chan, n: &str, lo: f64, hi: f64| TxPort {
            index: i,
            chan,
            name: n.into(),
            label: connector("LimeSDR-USB", chan, n),
            band: lo..=hi,
        };
        let board = [
            port(1, 0, "BAND1", 30e6, 1.9e9),
            port(2, 0, "BAND2", 2e9, 2.6e9),
            port(1, 1, "BAND1", 30e6, 1.9e9),
            port(2, 1, "BAND2", 2e9, 2.6e9),
        ];
        let labels: Vec<&str> = board.iter().map(|p| p.label.as_str()).collect();
        assert_eq!(labels, ["TX1_1", "TX1_2", "TX2_1", "TX2_2"]);

        let pinned = pick_tx_port(&board, "TX2_2", Hz::mhz(145), 0).expect("a port");
        assert_eq!((pinned.chan, pinned.index), (1, 2), "a pinned port names its transmitter");

        for chan in [0, 1] {
            let auto = pick_tx_port(&board, AUTO_ANTENNA, Hz::mhz(145), chan).expect("a port");
            assert_eq!((auto.chan, auto.name.as_str()), (chan, "BAND1"), "auto on TX{}", chan + 1);
        }
    }

    /// What the operator reads on the socket, per board and per channel.
    ///
    /// A LimeSDR-USB is marked RX1_H, RX1_L, RX1_W and TX1_1, TX1_2, with
    /// the same again for the second channel; a Mini brings every path out
    /// on one socket each way, so there is nothing to rename and
    /// LimeSuite's own name stands.
    #[test]
    fn a_port_is_offered_under_the_name_printed_on_the_board() {
        assert_eq!(connector("LimeSDR-USB", 0, "BAND1"), "TX1_1");
        assert_eq!(connector("LimeSDR-USB", 0, "BAND2"), "TX1_2");
        assert_eq!(connector("LimeSDR-USB", 1, "BAND1"), "TX2_1");
        assert_eq!(connector("LimeSDR-USB", 1, "BAND2"), "TX2_2");
        assert_eq!(connector("LimeSDR-USB", 0, "LNAH"), "RX1_H");
        assert_eq!(connector("LimeSDR-USB", 0, "LNAL"), "RX1_L");
        assert_eq!(connector("LimeSDR-USB", 0, "LNAW"), "RX1_W");
        assert_eq!(connector("LimeSDR-USB", 1, "LNAH"), "RX2_H");
        assert_eq!(connector("LimeSDR-PCIe", 1, "BAND2"), "TX2_2");
        assert_eq!(connector("LimeSDR-Mini", 0, "BAND1"), "BAND1");
        assert_eq!(connector("LimeSDR-Mini", 0, "LNAW"), "LNAW");
        // Whatever else the list holds is passed through untouched: NONE and
        // the Mini's own Auto are not connectors.
        assert_eq!(connector("LimeSDR-USB", 0, "NONE"), "NONE");
    }

    #[test]
    fn serials_shorten_to_the_identifying_tail() {
        assert_eq!(short_serial("0000000000000000457863dc3579c1df"), "3579c1df");
        assert_eq!(short_serial(""), "");
    }
}
