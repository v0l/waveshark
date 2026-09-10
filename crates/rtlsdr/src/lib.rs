//! Safe wrapper over librtlsdr.
//!
//! # Threading model
//!
//! librtlsdr's `rtlsdr_read_async` blocks for the whole life of a stream, so
//! it gets its own thread. Control calls (retune, gain) then necessarily come
//! from a different thread than the one inside `read_async`. This is what
//! `rtl_tcp` does and libusb is thread-safe for concurrent transfers, but
//! librtlsdr itself keeps mutable state per device, so every control call is
//! serialised through [`Handle::ctl`].
//!
//! # Sample handling
//!
//! Conversion from the RTL2832U's native offset-binary u8 to normalised f32
//! happens inside the USB callback thread. That sounds wasteful but it is the
//! cheapest place to do it: the bytes are already hot in L1 from the transfer,
//! and doing it here means the channel carries ready-to-use buffers instead of
//! forcing every downstream consumer to know about `cu8`.

use common::device::{Device, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use rtlsdr_sys as ffi;
use std::ffi::{c_void, CStr};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

/// USB transfer size. Must be a multiple of 512; 16 KiB is librtlsdr's default
/// and keeps the per-transfer overhead negligible at 2.4 MS/s.
const XFER_BYTES: u32 = 16 * 1024;
/// Number of transfers librtlsdr keeps in flight.
const XFER_COUNT: u32 = 15;
/// Depth of the buffer queue handed to the consumer. At 2.4 MS/s each buffer
/// is ~3.4 ms, so 64 is roughly 220 ms of slack before we start dropping.
const QUEUE_DEPTH: usize = 64;

/// Raw device pointer. librtlsdr has no thread affinity requirement, only a
/// data-race one, which [`Handle`] resolves with a mutex.
struct Raw(*mut ffi::rtlsdr_dev_t);
unsafe impl Send for Raw {}
unsafe impl Sync for Raw {}

struct Handle {
    raw: Raw,
    /// Serialises control transfers against each other. Deliberately *not*
    /// held during `read_async`, which runs for the stream's whole lifetime.
    ctl: Mutex<()>,
}

impl Handle {
    fn ptr(&self) -> *mut ffi::rtlsdr_dev_t {
        self.raw.0
    }
}

impl Drop for Handle {
    fn drop(&mut self) {
        unsafe { ffi::rtlsdr_close(self.raw.0) };
    }
}

fn check(rc: i32, what: &'static str) -> Result<()> {
    if rc == 0 {
        Ok(())
    } else {
        Err(Error::Other(format!("{what} failed (librtlsdr rc={rc})")))
    }
}

/// One enumerated dongle, before it is opened.
#[derive(Clone, Debug)]
pub struct Enumerated {
    pub index: u32,
    pub name: String,
    pub manufacturer: String,
    pub product: String,
    pub serial: String,
}

/// List every RTL-SDR attached to the system.
pub fn enumerate() -> Vec<Enumerated> {
    let n = unsafe { ffi::rtlsdr_get_device_count() };
    (0..n)
        .filter_map(|i| {
            let name = unsafe {
                let p = ffi::rtlsdr_get_device_name(i);
                if p.is_null() {
                    return None;
                }
                CStr::from_ptr(p).to_string_lossy().into_owned()
            };
            let (mut m, mut p, mut s) = ([0i8; 256], [0i8; 256], [0i8; 256]);
            let rc = unsafe {
                ffi::rtlsdr_get_device_usb_strings(
                    i,
                    m.as_mut_ptr().cast(),
                    p.as_mut_ptr().cast(),
                    s.as_mut_ptr().cast(),
                )
            };
            let cstr = |b: &[i8; 256]| unsafe {
                CStr::from_ptr(b.as_ptr().cast()).to_string_lossy().into_owned()
            };
            Some(Enumerated {
                index: i,
                name,
                manufacturer: if rc == 0 { cstr(&m) } else { String::new() },
                product: if rc == 0 { cstr(&p) } else { String::new() },
                serial: if rc == 0 { cstr(&s) } else { String::new() },
            })
        })
        .collect()
}

fn tuner_name(t: ffi::rtlsdr_tuner) -> &'static str {
    match t.0 {
        1 => "E4000",
        2 => "FC0012",
        3 => "FC0013",
        4 => "FC2580",
        5 => "R820T",
        6 => "R828D",
        _ => "unknown",
    }
}

/// Tunable span per tuner, in hertz. These are the manufacturer figures that
/// librtlsdr will actually accept, not the optimistic datasheet ones.
fn tuner_ranges(t: ffi::rtlsdr_tuner) -> Vec<TunerRange> {
    match t.0 {
        // E4000 has a genuine hole around the 1100-1250 MHz IF region.
        1 => vec![
            TunerRange { range: Hz::mhz(52)..=Hz::mhz(1100), label: "low" },
            TunerRange { range: Hz::mhz(1250)..=Hz::mhz(2200), label: "high" },
        ],
        5 | 6 => vec![TunerRange { range: Hz::mhz(24)..=Hz::mhz(1766), label: "main" }],
        _ => vec![TunerRange { range: Hz::mhz(22)..=Hz::mhz(1100), label: "main" }],
    }
}

pub struct RtlSdr {
    handle: Arc<Handle>,
    info: DeviceInfo,
    center: Hz,
    rate: Sps,
    /// Supported tuner gains in dB, ascending, as reported by the tuner driver.
    gains: Vec<f32>,
    streaming: Arc<AtomicBool>,
    /// Settings the driver cannot read back.
    ///
    /// librtlsdr has no getter for any of these, so the only way a control can
    /// show what is switched on is for the driver to remember what it set.
    tuner_gain: GainMode,
    rtl_agc: bool,
    bias_tee: bool,
    direct_sampling: bool,
    ppm: f64,
}

impl RtlSdr {
    /// Open by enumeration index.
    pub fn open(index: u32) -> Result<Self> {
        let mut dev: *mut ffi::rtlsdr_dev_t = std::ptr::null_mut();
        let rc = unsafe { ffi::rtlsdr_open(&mut dev, index) };
        if rc != 0 || dev.is_null() {
            // librtlsdr collapses everything to negative rc; map the two that
            // users actually hit to actionable errors.
            return Err(match rc {
                -6 => Error::Busy,
                -3 => Error::Permission,
                _ => Error::NoDevice,
            });
        }

        let handle = Arc::new(Handle { raw: Raw(dev), ctl: Mutex::new(()) });
        let tuner = unsafe { ffi::rtlsdr_get_tuner_type(handle.ptr()) };

        // Query the gain table. Returning 0 entries means the tuner driver has
        // no gain control, which for our purposes is a hard failure: blind
        // detection needs a usable dynamic range.
        let n = unsafe { ffi::rtlsdr_get_tuner_gains(handle.ptr(), std::ptr::null_mut()) };
        if n <= 0 {
            return Err(Error::UnsupportedTuner(format!(
                "{} reports no gain steps",
                tuner_name(tuner)
            )));
        }
        let mut raw_gains = vec![0i32; n as usize];
        unsafe { ffi::rtlsdr_get_tuner_gains(handle.ptr(), raw_gains.as_mut_ptr()) };
        // librtlsdr reports gain in tenths of a dB.
        let gains: Vec<f32> = raw_gains.iter().map(|g| *g as f32 / 10.0).collect();

        let e = enumerate().into_iter().find(|e| e.index == index);
        let serial = e.as_ref().map(|e| e.serial.clone()).unwrap_or_default();
        let label = e
            .as_ref()
            .map(|e| format!("{} {}", e.manufacturer, e.product))
            .unwrap_or_else(|| "RTL-SDR".into());

        let info = DeviceInfo {
            kind: DriverKind::RtlSdr,
            id: if serial.is_empty() { format!("rtlsdr:{index}") } else { serial },
            label: label.trim().to_string(),
            tuner: tuner_name(tuner).to_string(),
            ranges: tuner_ranges(tuner),
            // The RTL2832U accepts 225001-300000 and 900001-3200000 S/s, but
            // above 2.4 MS/s most USB 2.0 host controllers cannot sustain the
            // bulk rate and you get silent sample loss. These are the rates
            // worth offering.
            rates: [
                240_000, 960_000, 1_024_000, 1_200_000, 2_048_000, 2_400_000, 2_560_000, 3_200_000,
            ]
            .into_iter()
            .map(Sps)
            .collect(),
            rate_range: Sps(225_001)..=Sps(3_200_000),
            gain_stages: vec![common::GainStage {
                name: "tuner".to_string(),
                label: "Tuner RF".to_string(),
                range: gains.first().copied().unwrap_or(0.0)..=gains.last().copied().unwrap_or(0.0),
                // The tuner accepts these exact values and nothing between
                // them, so the control should offer exactly these.
                values: gains.clone(),
                step: 0.0,
                auto: true,
            }],
            native_format: SampleFormat::Cu8,
            // The RTL2832U has no analogue anti-alias filter worth the name;
            // the outer ~20% of the span is contaminated by the decimation
            // filter's transition and by the DC spur's skirt.
            usable_bandwidth_ratio: 0.80,
            tx: None,
        };

        let mut me = Self {
            handle,
            info,
            center: Hz::mhz(100),
            rate: Sps(2_048_000),
            gains,
            streaming: Arc::new(AtomicBool::new(false)),
            tuner_gain: GainMode::Manual(0.0),
            rtl_agc: false,
            bias_tee: false,
            direct_sampling: false,
            ppm: 0.0,
        };

        // Sane defaults: manual tuner gain (AGC hunting ruins wideband
        // detection because the noise floor moves under you) and RTL digital
        // AGC off.
        me.set_rate(Sps(2_048_000))?;
        me.set_center(Hz::mhz(100))?;
        me.set_gain("tuner", GainMode::Auto)?;
        me.set_rtl_agc(false)?;
        Ok(me)
    }

    /// Open the first device whose serial matches, else by index if `id` parses
    /// as a number.
    pub fn open_by_id(id: &str) -> Result<Self> {
        if let Some(e) = enumerate().into_iter().find(|e| e.serial == id) {
            return Self::open(e.index);
        }
        if let Ok(i) = id.parse::<u32>() {
            return Self::open(i);
        }
        Err(Error::NoDevice)
    }

    /// Snap a requested dB value to the nearest step the tuner actually has.
    fn nearest_gain(&self, db: f32) -> i32 {
        let g = self
            .gains
            .iter()
            .copied()
            .min_by(|a, b| (a - db).abs().total_cmp(&(b - db).abs()))
            .unwrap_or(0.0);
        (g * 10.0).round() as i32
    }

    pub fn supported_gains(&self) -> &[f32] {
        &self.gains
    }

    /// RTL2832U digital AGC, which is separate from the tuner's own gain.
    pub fn set_rtl_agc(&mut self, on: bool) -> Result<()> {
        let _g = self.handle.ctl.lock().unwrap();
        check(unsafe { ffi::rtlsdr_set_agc_mode(self.handle.ptr(), on as i32) }, "set_agc_mode")
    }

    /// Bias tee power on the antenna port. Off by default, and worth leaving
    /// off unless an LNA is actually attached.
    pub fn set_bias_tee(&mut self, on: bool) -> Result<()> {
        let _g = self.handle.ctl.lock().unwrap();
        check(unsafe { ffi::rtlsdr_set_bias_tee(self.handle.ptr(), on as i32) }, "set_bias_tee")
    }

    /// Direct sampling mode: 0 off, 1 I branch, 2 Q branch. Q branch on an
    /// RTL-SDR Blog v3 gives usable HF coverage below the tuner's 24 MHz floor.
    pub fn set_direct_sampling(&mut self, mode: i32) -> Result<()> {
        let _g = self.handle.ctl.lock().unwrap();
        check(
            unsafe { ffi::rtlsdr_set_direct_sampling(self.handle.ptr(), mode) },
            "set_direct_sampling",
        )
    }

    /// Exact tuned frequency after the PLL rounds to its step size. Worth
    /// reading back: the R820T's step is around 1-2 Hz but the error compounds
    /// with the ppm correction.
    pub fn actual_center(&self) -> Hz {
        Hz(unsafe { ffi::rtlsdr_get_center_freq(self.handle.ptr()) } as u64)
    }

    pub fn actual_rate(&self) -> Sps {
        Sps(unsafe { ffi::rtlsdr_get_sample_rate(self.handle.ptr()) } as u64)
    }
}

impl Device for RtlSdr {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn set_center(&mut self, f: Hz) -> Result<()> {
        if !self.info.covers(f) {
            let r = &self.info.ranges[0].range;
            return Err(Error::FreqOutOfRange { req: f, lo: *r.start(), hi: *r.end() });
        }
        let _g = self.handle.ctl.lock().unwrap();
        check(
            unsafe { ffi::rtlsdr_set_center_freq(self.handle.ptr(), f.get() as u32) },
            "set_center_freq",
        )?;
        drop(_g);
        self.center = f;
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
        check(
            unsafe { ffi::rtlsdr_set_sample_rate(self.handle.ptr(), r.get() as u32) },
            "set_sample_rate",
        )?;
        drop(_g);
        self.rate = r;
        Ok(())
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        if stage != "tuner" {
            return Err(Error::other(format!("no gain stage named {stage:?}")));
        }
        self.tuner_gain = mode;
        let _g = self.handle.ctl.lock().unwrap();
        match mode {
            GainMode::Auto => check(
                unsafe { ffi::rtlsdr_set_tuner_gain_mode(self.handle.ptr(), 0) },
                "set_tuner_gain_mode(auto)",
            ),
            GainMode::Manual(db) => {
                check(
                    unsafe { ffi::rtlsdr_set_tuner_gain_mode(self.handle.ptr(), 1) },
                    "set_tuner_gain_mode(manual)",
                )?;
                let tenths = self.nearest_gain(db);
                check(
                    unsafe { ffi::rtlsdr_set_tuner_gain(self.handle.ptr(), tenths) },
                    "set_tuner_gain",
                )
            }
        }
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        vec![("tuner".to_string(), self.tuner_gain)]
    }

    fn toggles(&self) -> Vec<common::Toggle> {
        vec![
            common::Toggle {
                name: "rtl_agc".into(),
                label: "RTL2832U digital AGC".into(),
                help: "Gain control in the demodulator chip, after the tuner. Recovers a weak signal on a quiet band, and ruins wideband work: the noise floor moves under you and every level measurement moves with it."
                    .into(),
                on: self.rtl_agc,
            },
            common::Toggle {
                name: "bias_tee".into(),
                label: "Bias tee".into(),
                help: "Puts 4.5 V on the antenna socket to power a mast head amplifier. Leave it off unless you know what is on the other end of the cable, because a shorted or DC coupled antenna takes the current."
                    .into(),
                on: self.bias_tee,
            },
            common::Toggle {
                name: "direct_sampling".into(),
                label: "Direct sampling (HF)".into(),
                help: "Bypasses the tuner and samples the Q branch directly, which reaches below the tuner's 24 MHz floor on a v3 dongle. Everything above about 14 MHz aliases, and the tuner gain does nothing while it is on."
                    .into(),
                on: self.direct_sampling,
            },
        ]
    }

    fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
        match name {
            "rtl_agc" => {
                self.set_rtl_agc(on)?;
                self.rtl_agc = on;
            }
            "bias_tee" => {
                self.set_bias_tee(on)?;
                self.bias_tee = on;
            }
            "direct_sampling" => {
                self.set_direct_sampling(if on { 2 } else { 0 })?;
                self.direct_sampling = on;
            }
            _ => return Err(Error::other(format!("no setting named {name:?}"))),
        }
        Ok(())
    }

    fn ppm(&self) -> f64 {
        self.ppm
    }

    fn set_ppm(&mut self, ppm: f64) -> Result<()> {
        let _g = self.handle.ctl.lock().unwrap();
        let rc = unsafe { ffi::rtlsdr_set_freq_correction(self.handle.ptr(), ppm.round() as i32) };
        // -2 means "already set to this value", which is not an error.
        if rc == 0 || rc == -2 {
            self.ppm = ppm;
            Ok(())
        } else {
            check(rc, "set_freq_correction")
        }
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }

        let _g = self.handle.ctl.lock().unwrap();
        check(unsafe { ffi::rtlsdr_reset_buffer(self.handle.ptr()) }, "reset_buffer")?;
        drop(_g);

        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));

        // The context is built here but the raw pointer is only taken inside
        // the thread. Capturing a `*mut` in the closure would make it non-Send,
        // and would also be a lie: the pointer is only valid once the box has
        // been moved to its final home.
        let ctx = Box::new(CbCtx {
            tx,
            dropped: dropped.clone(),
            center: self.center,
            rate: self.rate,
            seq: 0,
        });

        let handle = self.handle.clone();
        let streaming = self.streaming.clone();
        let join = std::thread::Builder::new()
            .name("rtlsdr-rx".into())
            .spawn(move || {
                // `ctx` lives for the whole of read_async, which is the only
                // thing that dereferences this pointer.
                let mut ctx = ctx;
                let ctx_ptr: *mut CbCtx = &mut *ctx;
                let rc = unsafe {
                    ffi::rtlsdr_read_async(
                        handle.ptr(),
                        Some(rtlsdr_cb),
                        ctx_ptr.cast::<c_void>(),
                        XFER_COUNT,
                        XFER_BYTES,
                    )
                };
                if rc != 0 {
                    tracing::warn!(rc, "rtlsdr_read_async exited with error");
                }
                streaming.store(false, Ordering::SeqCst);
                drop(ctx);
            })
            .map_err(|e| Error::other(format!("spawn rx thread: {e}")))?;

        Ok(Box::new(RtlStream {
            rx,
            dropped,
            handle: self.handle.clone(),
            join: Some(join),
            stopped: false,
        }))
    }
}

struct CbCtx {
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    center: Hz,
    rate: Sps,
    seq: u64,
}

/// Called by librtlsdr on its own thread for each completed USB transfer.
///
/// # Safety
/// `ctx` must point at a live `CbCtx` for the duration of `rtlsdr_read_async`.
unsafe extern "C" fn rtlsdr_cb(buf: *mut u8, len: u32, ctx: *mut c_void) {
    if ctx.is_null() || buf.is_null() || len == 0 {
        return;
    }
    let ctx = &mut *ctx.cast::<CbCtx>();
    let raw = std::slice::from_raw_parts(buf, len as usize);

    let mut samples = Vec::with_capacity(len as usize / 2);
    SampleFormat::Cu8.convert(raw, &mut samples);
    let n = samples.len() as u64;

    let buf = IqBuf::new(samples, ctx.center, ctx.rate, ctx.seq);
    ctx.seq += n;

    // Never block the USB callback. Blocking here stalls the transfer queue
    // and causes librtlsdr to drop transfers wholesale, which is worse than
    // dropping one buffer deliberately.
    match ctx.tx.try_send(buf) {
        Ok(()) => {}
        Err(TrySendError::Full(_)) => {
            ctx.dropped.fetch_add(n, Ordering::Relaxed);
        }
        Err(TrySendError::Disconnected(_)) => {}
    }
}

struct RtlStream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    handle: Arc<Handle>,
    join: Option<std::thread::JoinHandle<()>>,
    stopped: bool,
}

impl RxStream for RtlStream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        if !self.stopped {
            self.stopped = true;
            unsafe { ffi::rtlsdr_cancel_async(self.handle.ptr()) };
        }
    }
}

impl Drop for RtlStream {
    fn drop(&mut self) {
        self.stop();
        // Drain so the callback's try_send never blocks the cancel path.
        while self.rx.try_recv().is_ok() {}
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}
