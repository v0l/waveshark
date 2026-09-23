//! RTL-SDR over USB, driven by `rtlsdr-usb` with no system library.
//!
//! The dongle answers control transfers on endpoint 0 and streams on the bulk
//! endpoint, so tuning and gain changes work while the stream runs: control
//! calls take the state lock and the reader never does. Sample conversion from
//! offset-binary u8 happens in the reader thread, which keeps the channel
//! carrying ready-to-use buffers the way it did over librtlsdr.

use common::device::{Device, DeviceInfo, DriverKind, GainMode, RxStream};
use common::rtl::{self, Tuner};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use rtlsdr_usb as usb;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use usb::DirectSampling;

/// Depth of the buffer queue handed to the consumer. At 2.4 MS/s a 16 KiB
/// transfer is ~3.4 ms, so 64 is roughly 220 ms of slack before dropping.
const QUEUE_DEPTH: usize = 64;

pub use usb::Enumerated;

/// List every RTL-SDR attached to the system.
pub fn enumerate() -> Vec<Enumerated> {
    usb::RtlSdr::list()
}

pub struct RtlSdr {
    dev: Arc<usb::RtlSdr>,
    info: DeviceInfo,
    tuning: common::Tuning,
    center: Hz,
    rate: Sps,
    gains: Vec<f32>,
    streaming: Arc<AtomicBool>,
    tuner_gain: GainMode,
    switches: rtl::Switches,
    ppm: f64,
}

/// What this driver can drive. Offset tuning is an E4000 register write that
/// `rtlsdr-usb` does not make, so it is not offered here however the dongle
/// answers.
const OFFERED: [rtl::Switch; 3] =
    [rtl::Switch::RtlAgc, rtl::Switch::BiasTee, rtl::Switch::DirectSampling];

impl RtlSdr {
    pub fn open(index: u32) -> Result<Self> {
        let dev = Arc::new(usb::RtlSdr::open(index as usize).map_err(map_err)?);
        Self::from_usb(dev)
    }

    pub fn open_by_id(id: &str) -> Result<Self> {
        let dev = Arc::new(usb::RtlSdr::open_by_id(id).map_err(map_err)?);
        Self::from_usb(dev)
    }

    fn from_usb(dev: Arc<usb::RtlSdr>) -> Result<Self> {
        let code = dev.tuner().code();
        let tuner = Tuner::from_code(code as u32);
        let gains: Vec<f32> = dev.tuner_gains().iter().map(|g| *g as f32 / 10.0).collect();
        if gains.is_empty() {
            return Err(Error::UnsupportedTuner(format!("{} reports no gain steps", tuner.name())));
        }

        let serial = dev.serial().to_string();
        let label = if dev.manufacturer().is_empty() && dev.product().is_empty() {
            "RTL-SDR".to_string()
        } else {
            format!("{} {}", dev.manufacturer(), dev.product()).trim().to_string()
        };

        let info = DeviceInfo {
            kind: DriverKind::RtlSdr,
            id: if serial.is_empty() { "rtlsdr".into() } else { serial },
            label,
            tuner: tuner.name().to_string(),
            ranges: tuner.ranges(),
            rates: rtl::RATES.to_vec(),
            rate_range: rtl::RATE_RANGE,
            gain_stages: vec![common::GainStage {
                name: "tuner".to_string(),
                label: "Tuner RF".to_string(),
                range: *gains.first().unwrap()..=*gains.last().unwrap(),
                // The tuner accepts these exact values and nothing between
                // them, so the control should offer exactly these.
                values: gains.clone(),
                step: 0.0,
                auto: true,
            }],
            native_format: SampleFormat::Cu8,
            usable_bandwidth_ratio: rtl::USABLE_BANDWIDTH_RATIO,
            tunable: true,
            tx: None,
        };

        let mut me = Self {
            tuning: Default::default(),
            dev,
            info,
            center: Hz::mhz(100),
            rate: Sps(2_048_000),
            gains,
            streaming: Arc::new(AtomicBool::new(false)),
            tuner_gain: GainMode::Manual(0.0),
            switches: rtl::Switches::default(),
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

    pub fn supported_gains(&self) -> &[f32] {
        &self.gains
    }

    pub fn set_rtl_agc(&mut self, on: bool) -> Result<()> {
        self.dev.set_rtl_agc(on).map_err(map_err)?;
        self.switches.set(rtl::Switch::RtlAgc, on);
        Ok(())
    }

    pub fn set_bias_tee(&mut self, on: bool) -> Result<()> {
        self.dev.set_bias_tee(on).map_err(map_err)?;
        self.switches.set(rtl::Switch::BiasTee, on);
        Ok(())
    }

    pub fn set_direct_sampling(&mut self, on: bool) -> Result<()> {
        let mode = if on { DirectSampling::Q } else { DirectSampling::Off };
        self.dev.set_direct_sampling_mode(mode).map_err(map_err)?;
        self.switches.set(rtl::Switch::DirectSampling, on);
        if !on {
            self.set_rate(self.rate)?;
            self.set_gain("tuner", self.tuner_gain)?;
        }
        Ok(())
    }

    pub fn actual_center(&self) -> Hz {
        Hz(self.dev.frequency() as u64)
    }

    pub fn actual_rate(&self) -> Sps {
        Sps(self.dev.sample_rate() as u64)
    }
}

fn map_err(e: usb::Error) -> Error {
    match e {
        usb::Error::NoDevice => Error::NoDevice,
        usb::Error::Busy => Error::Busy,
        usb::Error::Permission => Error::Permission,
        usb::Error::Unsupported(m) => Error::UnsupportedTuner(m),
        other => Error::other(other.to_string()),
    }
}

fn nearest_gain(gains: &[f32], db: f32) -> f32 {
    gains.iter().copied().min_by(|a, b| (a - db).abs().total_cmp(&(b - db).abs())).unwrap_or(0.0)
}

impl Device for RtlSdr {
    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }

    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }

    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn set_center(&mut self, f: Hz) -> Result<()> {
        if !self.info.covers(f) {
            let r = &self.info.ranges[0].range;
            return Err(Error::FreqOutOfRange { req: f, lo: *r.start(), hi: *r.end() });
        }
        self.dev.set_frequency(f.get() as u32).map_err(map_err)?;
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
        self.dev.set_sample_rate(r.get() as u32).map_err(map_err)?;
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
        match mode {
            GainMode::Auto => self.dev.set_tuner_gain(false, 0).map_err(map_err),
            GainMode::Manual(db) => {
                let tenths = (nearest_gain(&self.gains, db) * 10.0).round() as i32;
                self.dev.set_tuner_gain(true, tenths).map_err(map_err)
            }
        }
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        vec![("tuner".to_string(), self.tuner_gain)]
    }

    fn toggles(&self) -> Vec<common::Toggle> {
        self.switches.toggles(&OFFERED)
    }

    fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
        match rtl::Switch::from_name(name).filter(|s| OFFERED.contains(s)) {
            Some(rtl::Switch::RtlAgc) => self.set_rtl_agc(on),
            Some(rtl::Switch::BiasTee) => self.set_bias_tee(on),
            Some(rtl::Switch::DirectSampling) => self.set_direct_sampling(on),
            Some(rtl::Switch::OffsetTuning) | None => {
                Err(Error::other(format!("no setting named {name:?}")))
            }
        }
    }

    fn ppm(&self) -> f64 {
        self.ppm
    }

    fn set_ppm(&mut self, ppm: f64) -> Result<()> {
        self.dev.set_ppm(ppm.round() as i32).map_err(map_err)?;
        self.ppm = ppm;
        Ok(())
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }

        if let Err(e) = self.dev.reset_buffer().map_err(map_err) {
            self.streaming.store(false, Ordering::SeqCst);
            return Err(e);
        }

        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));

        let reader = match self.dev.start_rx().map_err(map_err) {
            Ok(r) => r,
            Err(e) => {
                self.streaming.store(false, Ordering::SeqCst);
                return Err(e);
            }
        };

        let reader = Arc::new(Mutex::new(reader));

        let ctx =
            Ctx { tx, dropped: dropped.clone(), center: self.center, rate: self.rate, seq: 0 };

        let streaming = self.streaming.clone();
        let stream_flag = streaming.clone();
        let loop_reader = reader.clone();
        std::thread::Builder::new()
            .name("rtlsdr-rx".into())
            .spawn(move || convert_loop(loop_reader, ctx, streaming))?;
        Ok(Box::new(RtlStream { rx, dropped, streaming: stream_flag, reader }))
    }
}

struct Ctx {
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    center: Hz,
    rate: Sps,
    seq: u64,
}

fn convert_loop(reader: Arc<Mutex<usb::Reader>>, mut ctx: Ctx, streaming: Arc<AtomicBool>) {
    loop {
        let chunk = {
            let Ok(mut r) = reader.lock() else { break };
            r.read()
        };
        match chunk {
            Ok(bytes) => {
                let mut samples = Vec::with_capacity(bytes.len() / 2);
                SampleFormat::Cu8.convert(&bytes, &mut samples);
                let n = samples.len() as u64;
                let buf = IqBuf::new(samples, ctx.center, ctx.rate, ctx.seq);
                ctx.seq += n;
                // Never block the reader. Blocking here stalls the endpoint
                // queue and drops transfers wholesale, which is worse than
                // dropping one buffer deliberately.
                match ctx.tx.try_send(buf) {
                    Ok(()) => {}
                    Err(TrySendError::Full(_)) => {
                        ctx.dropped.fetch_add(n, Ordering::Relaxed);
                    }
                    Err(TrySendError::Disconnected(_)) => break,
                }
            }
            Err(e) => {
                if !matches!(e, usb::Error::Usb(ref m) if m == "stopped") {
                    tracing::warn!("rtlsdr stream ended: {e}");
                }
                break;
            }
        }
    }
    streaming.store(false, Ordering::SeqCst);
}

struct RtlStream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    streaming: Arc<AtomicBool>,
    reader: Arc<Mutex<usb::Reader>>,
}

impl RxStream for RtlStream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        // Cancelling the endpoint queue wakes the convert thread's read.
        if let Ok(mut r) = self.reader.lock() {
            r.stop();
        }
    }
}

impl Drop for RtlStream {
    fn drop(&mut self) {
        self.streaming.store(false, Ordering::SeqCst);
    }
}
