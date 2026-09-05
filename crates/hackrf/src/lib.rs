//! HackRF One support, adapting `hackrf-usb` to the workspace `Device` trait.
//!
//! The USB protocol work is `hackrf-usb`'s; this crate is the translation
//! layer: capability reporting, Cs8 conversion in both directions, and the
//! gain policy.

pub mod gain;

use common::{
    Device, DeviceInfo, DriverKind, Error, GainMode, Hz, IqBuf, Result, RxStream, SampleFormat,
    Sps, TunerRange, TxInfo, TxStream, C32,
};
use hackrf_usb::{AsyncReadControlHandle, AsyncReadHandle, AsyncWriteHandle, HackRf};
use std::time::Duration;

/// Datasheet tuning range.
const FREQ_MIN: u64 = 1_000_000;
const FREQ_MAX: u64 = 6_000_000_000;
/// Below 2 MS/s the USB stream starves; above 20 the host usually cannot keep up.
const RATE_MIN: u64 = 2_000_000;
const RATE_MAX: u64 = 20_000_000;

/// The analogue filter is set to three quarters of the sample rate, so the
/// outer eighth at each edge is roll-off rather than usable span.
const USABLE_RATIO: f32 = 0.75;

fn map_err(e: hackrf_usb::Error) -> Error {
    let s = e.to_string();
    if s.contains("Access") || s.contains("permission") || s.contains("Permission") {
        Error::Permission
    } else if s.contains("busy") || s.contains("Busy") {
        Error::Busy
    } else if s.contains("No HackRF") || s.contains("not found") {
        Error::NoDevice
    } else {
        Error::other(s)
    }
}

/// Serial numbers of every attached HackRF.
pub fn enumerate() -> Vec<String> {
    HackRf::list_devices().unwrap_or_default()
}

/// What the radio is doing with its USB claim, and the silence a receiver
/// hears while it is transmitting.
///
/// A HackRF is half duplex, so an over stops reception. Tearing the receive
/// stream down to say so makes every consumer of it deal with the radio
/// vanishing and coming back, and the graph above it loses the state it took
/// minutes to build: the spectrum's averaging, every channel's squelch, every
/// decoder mid-frame. The driver keeps the stream instead and feeds it a
/// noise floor at the same rate and centre, so the receiver runs through an
/// over and the waterfall shows the gap rather than stopping.
///
/// A full duplex radio needs none of this, which is why it is here rather
/// than in the caller: what a device does when it is asked to transmit is the
/// device's own business. `DeviceInfo::tx`'s `half_duplex` says which kind it
/// is.
pub struct Shared {
    /// The reader, while there is one. Taken out for the duration of an over.
    rx: parking_lot::Mutex<Option<AsyncReadHandle>>,
    /// The transmitter's control channel, while an over is running.
    ///
    /// Held here rather than on the device because the device is not told
    /// when an over ends: the transmit stream is what ends it, and a stored
    /// handle to a thread that has exited refuses every command with
    /// "control channel closed", which is what stopped the first key-up from
    /// working at all.
    tx: parking_lot::Mutex<Option<AsyncReadControlHandle>>,
    /// Whether reads should produce silence rather than samples.
    silent: std::sync::atomic::AtomicBool,
    /// Which unit to reopen: the same one, by the index it was opened at.
    index: usize,
    /// What the receiver should be set to, for whatever reopens the radio:
    /// the transmit stream when an over ends, and the watchdog when the
    /// hardware stops producing samples.
    want: parking_lot::Mutex<Wanted>,
}

/// The receive settings to restore on a reopen.
#[derive(Clone, Copy, Debug)]
struct Wanted {
    center: Hz,
    rate: Sps,
    stages: gain::Stages,
}

/// Open the unit again and put it back to work receiving.
fn reopen_rx(index: usize, want: Wanted) -> Result<AsyncReadHandle> {
    let d = HackRf::open_by_index(index).map_err(map_err)?;
    d.set_sample_rate(want.rate.0 as u32).map_err(map_err)?;
    let bw = hackrf_usb::transport::compute_baseband_filter_bw((want.rate.0 as u32) * 3 / 4);
    d.set_baseband_filter_bandwidth(bw).map_err(map_err)?;
    d.set_freq(want.center.0).map_err(map_err)?;
    let handle = d.into_streaming_reader(0, 0).map_err(map_err)?;
    // Entering receive mode re-initialises the front end, so tuning and gain
    // are set again through the running stream rather than before it.
    let c = handle.control_handle();
    c.tune(want.center.0).map_err(map_err)?;
    c.set_amp_enable(want.stages.amp).map_err(map_err)?;
    c.set_lna_gain(want.stages.lna).map_err(map_err)?;
    c.set_vga_gain(want.stages.vga).map_err(map_err)?;
    Ok(handle)
}

impl Shared {
    fn is_silent(&self) -> bool {
        self.silent.load(std::sync::atomic::Ordering::Relaxed)
    }
}

pub struct HackRfDevice {
    dev: Option<HackRf>,
    info: DeviceInfo,
    center: Hz,
    rate: Sps,
    stages: gain::Stages,
    tx_stages: gain::TxStages,

    shared: std::sync::Arc<Shared>,
    /// The index this unit was opened at, for reopening it after an over.
    index: usize,
}

impl HackRfDevice {
    pub fn open(index: usize) -> Result<Self> {
        let dev = HackRf::open_by_index(index).map_err(map_err)?;
        let serial = dev
            .board_partid_serialno()
            .map(|(_, _, s)| s)
            .unwrap_or_else(|_| format!("index{index}"));
        let version = dev.version().unwrap_or_else(|_| "unknown".into());
        let board = dev
            .board_id()
            .map(hackrf_usb::transport::board_id_name)
            .unwrap_or("HackRF");

        let info = DeviceInfo {
            kind: DriverKind::HackRf,
            id: serial.clone(),
            label: format!("{board} {}", short_serial(&serial)),
            tuner: format!("MAX2837 / RFFC5072 (fw {version})"),
            ranges: vec![TunerRange {
                range: Hz(FREQ_MIN)..=Hz(FREQ_MAX),
                label: "1 MHz - 6 GHz",
            }],
            rates: Vec::new(),
            rate_range: Sps(RATE_MIN)..=Sps(RATE_MAX),
            // The three real stages, in signal path order. A caller that
            // wants one number still has "tuner", which distributes across
            // them, but a control panel should show what the hardware has.
            gain_stages: vec![
                common::GainStage {
                    name: "amp".into(),
                    label: "Front end amp".into(),
                    range: 0.0..=gain::AMP_DB,
                    values: vec![0.0, gain::AMP_DB],
                    step: gain::AMP_DB,
                    auto: false,
                },
                common::GainStage {
                    name: "lna".into(),
                    label: "LNA (sets noise figure)".into(),
                    range: 0.0..=40.0,
                    values: Vec::new(),
                    step: 8.0,
                    auto: false,
                },
                common::GainStage {
                    name: "vga".into(),
                    label: "Baseband VGA (drives the ADC)".into(),
                    range: 0.0..=62.0,
                    values: Vec::new(),
                    step: 2.0,
                    auto: false,
                },
            ],
            native_format: SampleFormat::Cs8,
            usable_bandwidth_ratio: USABLE_RATIO,
            tx: Some(TxInfo {
                ranges: vec![TunerRange {
                    range: Hz(FREQ_MIN)..=Hz(FREQ_MAX),
                    label: "1 MHz - 6 GHz",
                }],
                rate_range: Sps(RATE_MIN)..=Sps(RATE_MAX),
                // Two stages on transmit, and neither is the receive path's:
                // the LNA and baseband VGA are not in circuit at all.
                gain_stages: vec![
                    common::GainStage {
                        name: "amp".into(),
                        label: "Front end amp".into(),
                        range: 0.0..=gain::AMP_DB,
                        values: vec![0.0, gain::AMP_DB],
                        step: gain::AMP_DB,
                        auto: false,
                    },
                    common::GainStage {
                        name: "txvga".into(),
                        label: "Transmit IF gain".into(),
                        range: 0.0..=gain::TXVGA_MAX_DB,
                        values: Vec::new(),
                        step: 1.0,
                        auto: false,
                    },
                ],
                native_format: SampleFormat::Cs8,
                half_duplex: true,
                channels: 1,
            }),
        };

        let mut d = Self {
            dev: Some(dev),
            info,
            center: Hz(100_000_000),
            rate: Sps(8_000_000),
            stages: gain::Stages::from_total(32.0),
            tx_stages: gain::TxStages::default(),
            shared: std::sync::Arc::new(Shared {
                rx: parking_lot::Mutex::new(None),
                tx: parking_lot::Mutex::new(None),
                silent: std::sync::atomic::AtomicBool::new(false),
                index,
                want: parking_lot::Mutex::new(Wanted {
                    center: Hz(100_000_000),
                    rate: Sps(8_000_000),
                    stages: gain::Stages::from_total(32.0),
                }),
            }),
            index,
        };
        d.set_rate(Sps(8_000_000))?;
        d.set_center(Hz(100_000_000))?;
        d.apply_gain()?;
        Ok(d)
    }

    pub fn open_first() -> Result<Self> {
        Self::open(0)
    }

    fn hw(&self) -> Result<&HackRf> {
        self.dev.as_ref().ok_or(Error::Disconnected)
    }

    /// The control channel of whichever stream is running now.
    ///
    /// Asked for on every call rather than stored, because which stream is
    /// running changes underneath this: an over replaces the reader with a
    /// writer and puts the reader back when it ends.
    fn ctl(&self) -> Option<AsyncReadControlHandle> {
        if let Some(c) = self.shared.tx.lock().as_ref() {
            return Some(c.clone());
        }
        self.shared.rx.lock().as_ref().map(|h| h.control_handle())
    }

    fn transmitting(&self) -> bool {
        self.shared.tx.lock().is_some()
    }

    fn apply_tx_gain(&self) -> Result<()> {
        let gain::TxStages { amp, txvga } = self.tx_stages;
        if let Some(c) = self.ctl() {
            c.set_amp_enable(amp).map_err(map_err)?;
            c.set_txvga_gain(txvga).map_err(map_err)?;
        } else {
            let d = self.hw()?;
            d.set_amp_enable(amp).map_err(map_err)?;
            d.set_txvga_gain(txvga).map_err(map_err)?;
        }
        Ok(())
    }

    fn apply_gain(&self) -> Result<()> {
        self.shared.want.lock().stages = self.stages;
        let gain::Stages { amp, lna, vga } = self.stages;
        if let Some(c) = self.ctl() {
            c.set_amp_enable(amp).map_err(map_err)?;
            c.set_lna_gain(lna).map_err(map_err)?;
            c.set_vga_gain(vga).map_err(map_err)?;
        } else {
            let d = self.hw()?;
            d.set_amp_enable(amp).map_err(map_err)?;
            d.set_lna_gain(lna).map_err(map_err)?;
            d.set_vga_gain(vga).map_err(map_err)?;
        }
        Ok(())
    }
}

fn short_serial(s: &str) -> String {
    // Serials are 32 hex digits and mostly leading zeros; the tail identifies
    // the unit and is what is printed on comparison tools.
    let t = s.trim_start_matches('0');
    if t.len() > 8 { t[t.len() - 8..].to_string() } else { t.to_string() }
}

impl Device for HackRfDevice {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn set_center(&mut self, f: Hz) -> Result<()> {
        if f.0 < FREQ_MIN || f.0 > FREQ_MAX {
            return Err(Error::FreqOutOfRange { req: f, lo: Hz(FREQ_MIN), hi: Hz(FREQ_MAX) });
        }
        // Retuning through the control handle keeps the stream running; going
        // via the device while streaming would need RX stopped and restarted.
        match self.ctl() {
            Some(c) => c.tune(f.0).map_err(map_err)?,
            None => self.hw()?.set_freq(f.0).map_err(map_err)?,
        }
        self.center = f;
        self.shared.want.lock().center = f;
        Ok(())
    }

    fn center(&self) -> Hz {
        self.center
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        if r.0 < RATE_MIN || r.0 > RATE_MAX {
            return Err(Error::RateUnsupported { req: r });
        }
        let d = self.hw()?;
        d.set_sample_rate(r.0 as u32).map_err(map_err)?;
        // set_sample_rate picks a filter, but be explicit: the default must
        // stay below the rate or out-of-band energy folds into the span.
        let bw = hackrf_usb::transport::compute_baseband_filter_bw((r.0 as u32) * 3 / 4);
        d.set_baseband_filter_bandwidth(bw).map_err(map_err)?;
        self.rate = r;
        self.shared.want.lock().rate = r;
        Ok(())
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        // No AGC in this hardware, so "auto" is a sensible fixed operating
        // point rather than a silent no-op.
        let db = match mode {
            GainMode::Auto => 32.0,
            GainMode::Manual(db) => db,
        };
        if self.transmitting() {
            return Err(Error::HalfDuplexBusy);
        }
        if !self.stages.set(stage, db) {
            return Err(Error::other(format!("no gain stage named {stage}")));
        }
        self.apply_gain()
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        vec![
            (
                "amp".into(),
                GainMode::Manual(if self.stages.amp { gain::AMP_DB } else { 0.0 }),
            ),
            ("lna".into(), GainMode::Manual(self.stages.lna as f32)),
            ("vga".into(), GainMode::Manual(self.stages.vga as f32)),
        ]
    }

    fn rate_needs_restart(&self) -> bool {
        true
    }

    fn set_tx_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        // There is no transmit AGC either, and guessing an operating point
        // for a transmitter means guessing how much power to radiate. Auto
        // means the bottom of the range.
        let db = match mode {
            GainMode::Auto => 0.0,
            GainMode::Manual(db) => db,
        };
        if !self.tx_stages.set(stage, db) {
            return Err(Error::other(format!("no transmit gain stage named {stage}")));
        }
        match self.transmitting() {
            true => self.apply_tx_gain(),
            // Applying it now would switch the front end amp on while the
            // receiver is running, which is a receive setting with the same
            // name. It goes on the hardware when transmit starts.
            false => Ok(()),
        }
    }

    fn tx_gains(&self) -> Vec<(String, GainMode)> {
        vec![
            (
                "amp".into(),
                GainMode::Manual(if self.tx_stages.amp { gain::AMP_DB } else { 0.0 }),
            ),
            ("txvga".into(), GainMode::Manual(self.tx_stages.txvga as f32)),
        ]
    }

    fn start_tx(&mut self) -> Result<Box<dyn TxStream>> {
        // Half duplex, so the radio has to stop receiving. The receive stream
        // is not torn down for it: it goes silent for the over and comes back
        // by itself, so nothing above the driver has to deal with the radio
        // disappearing. See [`Shared`].
        let was_rx = {
            let mut slot = self.shared.rx.lock();
            match slot.take() {
                Some(handle) => {
                    self.shared.silent.store(true, std::sync::atomic::Ordering::Relaxed);
                    handle.stop();
                    // Drained before it is dropped, and this is not optional:
                    // dropping joins the USB thread while still holding the
                    // receiving end of the bounded channel that thread sends
                    // into. A thread mid-send then waits for a read that will
                    // never come and the join waits for the thread, so the
                    // whole receiver stops dead the first time anybody keys
                    // up. The stop flag is only looked at between transfers,
                    // so the thread has to be let through its current send.
                    while handle.recv().is_some() {}
                    drop(handle);
                    true
                }
                None => false,
            }
        };
        let dev = match self.dev.take() {
            Some(d) => d,
            None => {
                // The reader owned it, so the unit is reopened rather than
                // handed over. It needs a moment to release the USB claim;
                // reopening immediately gets "already in use".
                std::thread::sleep(Duration::from_millis(150));
                let d = HackRf::open_by_index(self.index).map_err(map_err)?;
                d.set_sample_rate(self.rate.0 as u32).map_err(map_err)?;
                d.set_freq(self.center.0).map_err(map_err)?;
                d
            }
        };
        let handle = dev.into_streaming_writer(0, 0).map_err(map_err)?;
        *self.shared.tx.lock() = Some(handle.control_handle());
        // As with receive, entering the mode re-initialises the front end,
        // so the tuning and the gain have to be set again after it.
        if let Some(c) = self.ctl() {
            c.tune(self.center.0).map_err(map_err)?;
        }
        self.apply_tx_gain()?;
        Ok(Box::new(HackRfTxStream {
            handle: Some(handle),
            rate: self.rate,
            center: self.center,
            bytes: Vec::new(),
            shared: self.shared.clone(),
            restore_rx: was_rx,
            stalled: false,
        }))
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        let dev = self.dev.take().ok_or(Error::Disconnected)?;
        let handle = dev.into_streaming_reader(0, 0).map_err(map_err)?;
        // Entering receive mode re-initialises the front end, so the tune and
        // the gains set before streaming do not survive it: at 95.8 MHz the
        // floor reads 3 LSB rms without this and 23 with it, the difference
        // between hearing the FM band and hearing the converter.
        let ctl = handle.control_handle();
        *self.shared.rx.lock() = Some(handle);
        ctl.tune(self.center.0).map_err(map_err)?;
        self.apply_gain()?;
        self.shared.silent.store(false, std::sync::atomic::Ordering::Relaxed);
        Ok(Box::new(HackRfStream {
            shared: self.shared.clone(),
            center: self.center,
            rate: self.rate,
            seq: 0,
            last_dropped: 0,
            samples: Vec::new(),
            noise: 0x9E37_79B9_7F4A_7C15,
            silence_at: None,
            last_real: std::time::Instant::now(),
            attempts: 0,
        }))
    }
}

pub struct HackRfTxStream {
    /// Taken when the over ends, so the writer is joined and the USB claim
    /// released before the radio is reopened for receive.
    handle: Option<AsyncWriteHandle>,
    rate: Sps,
    center: Hz,
    /// Reused so a block at the sample rate does not allocate per call.
    bytes: Vec<u8>,
    shared: std::sync::Arc<Shared>,
    /// Whether a receive stream was running when this one started, and so
    /// whether one has to be running again when it finishes.
    restore_rx: bool,
    /// The writer stopped taking samples: a send timed out. Nothing more is
    /// offered to it, and the over ends without waiting on it.
    stalled: bool,
}

impl HackRfTxStream {
    /// End the over and give the radio back to the receiver.
    ///
    /// Called from `stop` and again from `drop`, and safe to call twice: the
    /// handle is taken the first time.
    fn hand_back(&mut self) {
        let Some(handle) = self.handle.take() else { return };
        // Before anything else: a command sent to a writer that has stopped
        // fails, and the device has to fall back to the reader's channel.
        *self.shared.tx.lock() = None;
        if !self.stalled {
            handle.drain(Duration::from_millis(500));
        }
        handle.stop();
        drop(handle);
        if !self.restore_rx {
            self.shared.silent.store(false, std::sync::atomic::Ordering::Relaxed);
            return;
        }
        // The writer's thread has to have let go of the device before the
        // same unit can be opened again.
        std::thread::sleep(Duration::from_millis(150));
        let want = *self.shared.want.lock();
        match reopen_rx(self.shared.index, want) {
            Ok(handle) => {
                *self.shared.rx.lock() = Some(handle);
                self.shared.silent.store(false, std::sync::atomic::Ordering::Relaxed);
            }
            // The radio did not come back here. The read side is told it is
            // no longer an over, so its watchdog sees an empty stream and
            // reopens or resets the board; left silent, it read the over
            // as still running and produced a noise floor for ever, which
            // is what a radio that "stopped working" after a key-up was.
            Err(e) => {
                tracing::warn!("HackRF did not return to receive: {e}");
                self.shared.silent.store(false, std::sync::atomic::Ordering::Relaxed);
            }
        }
    }
}

impl TxStream for HackRfTxStream {
    fn write(&mut self, buf: &IqBuf) -> Result<()> {
        // A block at another rate would go out at this one, stretched or
        // compressed: the wrong bandwidth on the wrong frequency. Resampling
        // belongs upstream where the graph can see it.
        if buf.rate != self.rate {
            return Err(Error::RateUnsupported { req: buf.rate });
        }
        if buf.samples.is_empty() {
            return Ok(());
        }
        let Some(h) = &self.handle else { return Err(Error::Disconnected) };
        if self.stalled {
            return Err(Error::Disconnected);
        }
        self.bytes.clear();
        SampleFormat::Cs8.encode(&buf.samples, &mut self.bytes);
        // Bounded, because this runs on the radio thread. A writer whose
        // transfers have stopped completing fills its queue and a blocking
        // send then never returns: the whole receiver stood still, not
        // transmitting and not receiving, until the process was killed.
        // A few blocks' worth is more than a healthy queue ever needs.
        let block = Duration::from_secs_f64(buf.samples.len() as f64 / self.rate.as_f64());
        let patience = (block * 4).max(Duration::from_millis(250));
        match h.send_timeout(std::mem::take(&mut self.bytes), patience) {
            Ok(()) => Ok(()),
            Err(e) => {
                self.stalled = true;
                tracing::warn!("HackRF transmitter stopped taking samples: {e}");
                Err(map_err(e))
            }
        }
    }

    fn underruns(&self) -> u64 {
        self.handle.as_ref().map(|h| h.idle_transfers()).unwrap_or(0)
    }

    fn drain(&mut self, timeout: Duration) -> bool {
        self.handle.as_ref().map(|h| h.drain(timeout)).unwrap_or(true)
    }

    fn stop(&mut self) {
        self.hand_back();
    }
}

/// Let the tail out, and put the receiver back, before this is torn down.
///
/// Dropping the write handle cancels transfers that have not gone out yet, so
/// a caller that drops immediately after its last `write` truncates the end
/// of the transmission.
impl Drop for HackRfTxStream {
    fn drop(&mut self) {
        self.hand_back();
    }
}

/// Level the driver feeds a receiver while the radio is transmitting.
///
/// Not zero, and not arbitrarily small either. A silent stream is not
/// something a receiver ever sees, and the code downstream is built for what
/// one does see: the spectrum would draw a floor at minus infinity, the level
/// gates would have nothing to measure a threshold against, and an AGC would
/// wind itself all the way up in the couple of seconds an over lasts.
///
/// Three least significant bits of the eight bit converter, which is what
/// this radio's own floor measures at 95.8 MHz with the front end running.
/// A floor far below that is not a quiet band but a broken converter, and
/// the ADC health check says so: at 90 dB down every sample lands on the
/// same value and the interface reports a starved converter for the length
/// of every over.
const SILENCE_RMS: f32 = 3.0 / 128.0;

pub struct HackRfStream {
    shared: std::sync::Arc<Shared>,
    center: Hz,
    rate: Sps,
    seq: u64,
    last_dropped: u64,
    samples: Vec<C32>,
    /// State of the noise the silence is made of.
    noise: u64,
    /// When the current run of silence started producing, so it is paced to
    /// the sample rate rather than run as fast as the caller asks.
    silence_at: Option<std::time::Instant>,
    /// When a real block last arrived, which is what the watchdog measures
    /// against.
    last_real: std::time::Instant,
    /// Restarts tried since the last real block.
    attempts: u32,
}

impl HackRfStream {
    /// Whether what is being read is the radio or the driver standing in for
    /// it, which is what a receiver shows as an over rather than as a dead
    /// band.
    pub fn is_silent(&self) -> bool {
        self.shared.is_silent()
    }

    /// A block of noise floor, paced to real time so the graph above runs at
    /// the rate it would have.
    fn silence(&mut self) -> IqBuf {
        let n = (self.rate.as_f64() * 0.02) as usize;
        let start = *self.silence_at.get_or_insert_with(std::time::Instant::now);
        self.samples.clear();
        self.samples.reserve(n);
        for _ in 0..n {
            // xorshift, so the floor is noise rather than a tone: a constant
            // would show as a carrier at DC through every detector below.
            self.noise ^= self.noise << 13;
            self.noise ^= self.noise >> 7;
            self.noise ^= self.noise << 17;
            let i = ((self.noise >> 40) & 0xFF_FFFF) as f32 / 16_777_216.0 - 0.5;
            let q = ((self.noise >> 16) & 0xFF_FFFF) as f32 / 16_777_216.0 - 0.5;
            self.samples.push(C32::new(i * SILENCE_RMS, q * SILENCE_RMS));
        }
        let want = std::time::Duration::from_secs_f64(n as f64 / self.rate.as_f64());
        let elapsed = start.elapsed();
        if want > elapsed {
            std::thread::sleep(want - elapsed);
        }
        self.silence_at = Some(std::time::Instant::now());
        let buf =
            IqBuf::new(std::mem::take(&mut self.samples), self.center, self.rate, self.seq);
        self.seq += n as u64;
        buf
    }
}

/// Signed 8-bit two's complement to unit-scaled complex float.
fn decode(bytes: &[u8], out: &mut Vec<C32>) {
    out.clear();
    out.reserve(bytes.len() / 2);
    for p in bytes.chunks_exact(2) {
        out.push(C32::new(
            p[0] as i8 as f32 * (1.0 / 128.0),
            p[1] as i8 as f32 * (1.0 / 128.0),
        ));
    }
}

/// How long a receiver may produce nothing, or nothing but zeros, before the
/// driver decides the radio has stopped and restarts it.
///
/// A HackRF that has just transmitted sometimes comes back streaming zeros:
/// the USB transfers keep arriving and every sample in them is the same
/// value, which is a converter that is not running. Nothing above the driver
/// can tell that from a very quiet band, and it is the driver that knows the
/// difference between a floor and a flatline.
const DEAD_AFTER: std::time::Duration = std::time::Duration::from_millis(600);

/// Reopens tried before the board is reset over USB.
///
/// A reset makes the radio drop off the bus and come back a second or two
/// later, which is disruptive enough to be a last resort rather than a first
/// response.
const REOPENS_BEFORE_RESET: u32 = 2;

impl HackRfStream {
    /// Whether a block is a real one or the converter standing still.
    fn is_flatline(samples: &[C32]) -> bool {
        match samples.first() {
            None => true,
            Some(first) => samples.iter().all(|c| c == first),
        }
    }

    /// Put the radio back to work after it has stopped producing.
    ///
    /// Reopening is enough for a board that is merely wedged in the wrong
    /// transceiver mode. A board that will not come back that way is reset
    /// over USB, which re-enumerates it, and then reopened once it is back.
    fn revive(&mut self, why: &str) {
        self.attempts += 1;
        tracing::warn!("HackRF {why}; restarting the receiver (attempt {})", self.attempts);
        if let Some(h) = self.shared.rx.lock().take() {
            h.stop();
            // Drained before dropping, or the join waits on a send nobody
            // will read. See the note on this stream's own Drop.
            while h.recv().is_some() {}
        }
        std::thread::sleep(std::time::Duration::from_millis(200));

        if self.attempts > REOPENS_BEFORE_RESET {
            if let Ok(d) = HackRf::open_by_index(self.shared.index) {
                let _ = d.reset();
                drop(d);
            }
            // It has to finish enumerating before it can be opened again.
            std::thread::sleep(std::time::Duration::from_secs(2));
        }

        let want = *self.shared.want.lock();
        match reopen_rx(self.shared.index, want) {
            Ok(h) => {
                *self.shared.rx.lock() = Some(h);
                self.last_real = std::time::Instant::now();
                tracing::info!("HackRF receiving again at {}", want.center);
            }
            Err(e) => tracing::warn!("HackRF did not reopen: {e}"),
        }
    }
}

impl RxStream for HackRfStream {
    fn read(&mut self) -> Result<IqBuf> {
        loop {
            if self.shared.is_silent() {
                // Transmitting: the radio is deaf by design, and the clock
                // for the watchdog starts again when it comes back.
                self.last_real = std::time::Instant::now();
                self.attempts = 0;
                return Ok(self.silence());
            }
            self.silence_at = None;
            // Polled rather than blocked on, because the handle has to be
            // available to whoever starts an over: a read holding the lock
            // until the next transfer arrives would stall keying by a whole
            // buffer, which at 2 MS/s is 65 ms.
            let got = {
                let slot = self.shared.rx.lock();
                match slot.as_ref() {
                    Some(h) => h.recv_state(),
                    None => hackrf_usb::RecvState::Empty,
                }
            };
            match got {
                hackrf_usb::RecvState::Data(chunk) => {
                    decode(&chunk, &mut self.samples);
                    // A transfer of identical samples is a converter that has
                    // stopped, which is what a HackRF does after some overs.
                    if Self::is_flatline(&self.samples) {
                        if self.last_real.elapsed() > DEAD_AFTER {
                            self.revive("is delivering nothing but flat samples");
                        }
                        continue;
                    }
                    self.last_real = std::time::Instant::now();
                    self.attempts = 0;
                    let n = self.samples.len() as u64;
                    let buf = IqBuf::new(
                        std::mem::take(&mut self.samples),
                        self.center,
                        self.rate,
                        self.seq,
                    );
                    self.seq += n;
                    return Ok(buf);
                }
                hackrf_usb::RecvState::Failed(e) => return Err(map_err(e)),
                // The streaming thread has gone. That is the radio being
                // unplugged or reset by hand, and it is worth one attempt to
                // get it back before the receiver above is told.
                hackrf_usb::RecvState::Closed => {
                    if self.attempts >= REOPENS_BEFORE_RESET + 1 {
                        return Err(Error::Disconnected);
                    }
                    self.revive("stopped streaming");
                }
                hackrf_usb::RecvState::Empty => {
                    if self.last_real.elapsed() > DEAD_AFTER {
                        if self.attempts >= REOPENS_BEFORE_RESET + 1 {
                            return Err(Error::Disconnected);
                        }
                        self.revive("has sent nothing");
                        continue;
                    }
                    std::thread::sleep(std::time::Duration::from_millis(1));
                }
            }
        }
    }

    fn dropped(&self) -> u64 {
        // Chunks, not samples, so scale by what a chunk holds.
        let c = self
            .shared
            .rx
            .lock()
            .as_ref()
            .map(|h| h.control_handle().dropped_chunks())
            .unwrap_or(0);
        c.saturating_mul((hackrf_usb::TRANSFER_BUFFER_SIZE / 2) as u64)
    }

    fn silent(&self) -> bool {
        self.shared.is_silent()
    }

    fn stop(&mut self) {
        if let Some(h) = self.shared.rx.lock().as_ref() {
            h.stop();
        }
    }
}

/// Let the streaming thread finish before its handle joins it.
///
/// `AsyncReadHandle::drop` joins the USB thread while still holding the
/// receiving end of the channel that thread sends into. The send is blocking
/// and the channel is bounded, so a thread that is mid-send when the receiver
/// stops reading waits for a read that will never come, and the join waits
/// for the thread: the process hangs on exit with the window already gone,
/// which is what makes a receiver look like it ignores SIGTERM.
///
/// Draining until the sender is gone breaks the cycle. The stop flag is only
/// looked at between transfers, so the thread has to be let through its
/// current send before it can notice it.
impl Drop for HackRfStream {
    fn drop(&mut self) {
        let handle = self.shared.rx.lock().take();
        if let Some(h) = handle {
            h.stop();
            while h.recv().is_some() {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_bytes_decode_to_the_right_sign_and_scale() {
        let mut out = Vec::new();
        // 0x7f is +127, 0x81 is -127, 0x00 is zero.
        decode(&[0x7f, 0x81, 0x00, 0x40], &mut out);
        assert_eq!(out.len(), 2);
        assert!((out[0].re - 127.0 / 128.0).abs() < 1e-6);
        assert!((out[0].im + 127.0 / 128.0).abs() < 1e-6);
        assert_eq!(out[1].re, 0.0);
        assert!((out[1].im - 0.5).abs() < 1e-6);
    }

    #[test]
    fn decoded_samples_stay_inside_the_unit_circle_ish() {
        let mut out = Vec::new();
        let bytes: Vec<u8> = (0..=255u8).flat_map(|b| [b, b]).collect();
        decode(&bytes, &mut out);
        for c in &out {
            assert!(c.re.abs() <= 1.0 && c.im.abs() <= 1.0, "{c} outside unit scale");
        }
    }

    #[test]
    fn an_odd_trailing_byte_is_ignored_rather_than_panicking() {
        let mut out = Vec::new();
        decode(&[1, 2, 3], &mut out);
        assert_eq!(out.len(), 1, "a half sample must not be invented");
    }

    #[test]
    fn decode_reuses_the_buffer() {
        let mut out = Vec::with_capacity(1024);
        let cap = out.capacity();
        for _ in 0..10 {
            decode(&[1, 2, 3, 4], &mut out);
        }
        assert_eq!(out.len(), 2);
        assert_eq!(out.capacity(), cap, "buffer was reallocated every call");
    }

    #[test]
    fn silence_is_a_quiet_band_and_not_an_empty_one() {
        // What a receiver reads while the radio is transmitting. Zero would
        // put the spectrum at minus infinity and leave every level gate with
        // nothing to measure against, so the floor is a real one: noise, at
        // about 90 dB down.
        let shared = std::sync::Arc::new(Shared {
            rx: parking_lot::Mutex::new(None),
            tx: parking_lot::Mutex::new(None),
            silent: std::sync::atomic::AtomicBool::new(true),
            index: 0,
            want: parking_lot::Mutex::new(Wanted {
                center: Hz(433_920_000),
                rate: Sps(2_000_000),
                stages: gain::Stages::from_total(32.0),
            }),
        });
        let mut st = HackRfStream {
            shared,
            center: Hz(433_920_000),
            rate: Sps(2_000_000),
            seq: 0,
            last_dropped: 0,
            samples: Vec::new(),
            noise: 0x9E37_79B9_7F4A_7C15,
            silence_at: None,
            last_real: std::time::Instant::now(),
            attempts: 0,
        };
        assert!(st.is_silent());
        let a = st.silence();
        let b = st.silence();
        assert_eq!(a.len(), 40_000, "20 ms at 2 MS/s");
        assert_eq!(a.center, Hz(433_920_000), "silence is still the tuned band");
        assert_eq!(b.seq, a.seq + a.len() as u64, "the sample count has to stay continuous");

        let rms =
            (a.samples.iter().map(|c| c.norm_sqr() as f64).sum::<f64>() / a.len() as f64).sqrt();
        let db = 20.0 * rms.log10();
        // A quiet band on an eight bit converter, not an empty one: below
        // about 50 dB down every sample lands on the same code and the ADC
        // health check calls the converter starved.
        assert!((-45.0..-25.0).contains(&db), "silence reads {db:.0} dBFS");
        // Noise, not a constant: a steady value is a carrier at DC to
        // everything downstream.
        let first = a.samples[0];
        assert!(a.samples.iter().any(|c| (c - first).norm() > 1e-7));
    }

    #[test]
    fn silence_is_paced_to_the_sample_rate() {
        // The graph above runs on the blocks it is handed, so silence that
        // arrived as fast as it could be generated would run the receiver at
        // a hundred times real time for the length of an over.
        let shared = std::sync::Arc::new(Shared {
            rx: parking_lot::Mutex::new(None),
            tx: parking_lot::Mutex::new(None),
            silent: std::sync::atomic::AtomicBool::new(true),
            index: 0,
            want: parking_lot::Mutex::new(Wanted {
                center: Hz(433_920_000),
                rate: Sps(2_000_000),
                stages: gain::Stages::from_total(32.0),
            }),
        });
        let mut st = HackRfStream {
            shared,
            center: Hz(100_000_000),
            rate: Sps(2_000_000),
            seq: 0,
            last_dropped: 0,
            samples: Vec::new(),
            noise: 1,
            silence_at: None,
            last_real: std::time::Instant::now(),
            attempts: 0,
        };
        let t = std::time::Instant::now();
        for _ in 0..5 {
            st.silence();
        }
        let el = t.elapsed().as_secs_f64();
        // Five blocks of 20 ms, less the first which starts the clock.
        assert!((0.06..0.16).contains(&el), "five blocks took {el:.3} s");
    }

    #[test]
    fn serials_shorten_to_the_part_that_identifies_the_unit() {
        assert_eq!(short_serial("0000000000000000457863dc3579c1df"), "3579c1df");
        assert_eq!(short_serial("abc"), "abc");
    }
}

#[cfg(test)]
mod watchdog_tests {
    use super::*;

    #[test]
    fn a_transfer_of_identical_samples_is_a_stopped_converter() {
        // What a HackRF hands over after some transmissions: the USB keeps
        // delivering and every sample in the block is the same value. A very
        // quiet band is not that, because noise is never one number.
        let flat = vec![C32::new(0.0, 0.0); 1024];
        assert!(HackRfStream::is_flatline(&flat));
        let stuck = vec![C32::new(0.5, -0.25); 1024];
        assert!(HackRfStream::is_flatline(&stuck), "a held value is a flatline too");

        let mut noise = flat.clone();
        noise[512] = C32::new(1.0 / 128.0, 0.0);
        assert!(!HackRfStream::is_flatline(&noise), "one bit of movement is a live converter");
        assert!(HackRfStream::is_flatline(&[]), "an empty block carries no evidence of life");
    }
}
