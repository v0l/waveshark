pub mod gain;
pub mod hf;

use airspy_usb as usb;
use common::device::{Choice, Device, DeviceInfo, DriverKind, GainMode, GainStage, RxStream};
use common::{C32, Error, Hz, IqBuf, Result, SampleFormat, Sps, Toggle, TunerRange};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use dsp::real_iq::RealToIq;
use gain::{Stages, Table};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

const FREQ_MIN: u64 = 24_000_000;
const FREQ_MAX: u64 = 1_800_000_000;
const QUEUE_DEPTH: usize = 32;
const FALLBACK_RATES: [u32; 2] = [10_000_000, 2_500_000];
const GAIN_TABLE: &str = "gain_table";
const BIAS_TEE: &str = "bias_tee";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Model {
    R2,
    Mini,
    Unknown,
}

impl Model {
    fn read(version: &str, rates: &[u32]) -> Self {
        if version.to_ascii_uppercase().contains("MINI") {
            return Self::Mini;
        }
        match rates.iter().max() {
            Some(&top) if top <= 6_000_000 => Self::Mini,
            Some(_) => Self::R2,
            None => Self::Unknown,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::R2 => "Airspy R2",
            Self::Mini => "Airspy Mini",
            Self::Unknown => "Airspy",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    pub index: usize,
    pub serial: String,
    pub model: Model,
    pub rates: Vec<u32>,
}

impl Found {
    pub fn label(&self) -> String {
        let tail = self.serial.trim_start_matches('0');
        let tail = &tail[tail.len().saturating_sub(8)..];
        match tail.is_empty() {
            true => self.model.name().to_string(),
            false => format!("{} {tail}", self.model.name()),
        }
    }
}

type Known = Mutex<HashMap<String, (Model, Vec<u32>)>>;

fn known() -> &'static Known {
    static KNOWN: OnceLock<Known> = OnceLock::new();
    KNOWN.get_or_init(Default::default)
}

fn remember(serial: &str, model: Model, rates: &[u32]) {
    if let Ok(mut k) = known().lock() {
        k.insert(serial.to_string(), (model, rates.to_vec()));
    }
}

fn identify(dev: &usb::Airspy) -> (Model, Vec<u32>) {
    let rates = dev.sample_rates().unwrap_or_else(|_| FALLBACK_RATES.to_vec());
    let version = dev.version().unwrap_or_default();
    (Model::read(&version, &rates), rates)
}

pub fn enumerate() -> Vec<Found> {
    usb::enumerate()
        .into_iter()
        .map(|e| {
            let cached = known().lock().ok().and_then(|k| k.get(&e.serial).cloned());
            let (model, rates) = match cached {
                Some(c) => c,
                None => match usb::Airspy::open_enumerated(&e) {
                    Ok(dev) => {
                        let (model, rates) = identify(&dev);
                        remember(&e.serial, model, &rates);
                        (model, rates)
                    }
                    Err(_) => (Model::Unknown, FALLBACK_RATES.to_vec()),
                },
            };
            Found { index: e.index, serial: e.serial, model, rates }
        })
        .collect()
}

pub(crate) fn map_err(e: usb::Error) -> Error {
    match e {
        usb::Error::NoDevice => Error::NoDevice,
        usb::Error::Busy => Error::Busy,
        usb::Error::Permission => Error::Permission,
        usb::Error::Stopped => Error::Disconnected,
        usb::Error::Usb(m) => Error::other(m),
    }
}

pub struct Airspy {
    dev: Arc<usb::Airspy>,
    info: DeviceInfo,
    tuning: common::Tuning,
    center: Hz,
    rate: Sps,
    rates: Vec<u32>,
    stages: Stages,
    table: Table,
    total: Option<f32>,
    bias_tee: bool,
}

impl Airspy {
    pub fn open(index: usize) -> Result<Self> {
        let found = usb::enumerate().into_iter().nth(index).ok_or(Error::NoDevice)?;
        let dev = usb::Airspy::open_enumerated(&found).map_err(map_err)?;
        let (model, rates) = identify(&dev);
        remember(&found.serial, model, &rates);
        let version = dev.version().unwrap_or_default();
        let found = Found { index, serial: found.serial, model, rates: rates.clone() };
        let info = DeviceInfo {
            kind: DriverKind::Airspy,
            id: match found.serial.is_empty() {
                true => format!("airspy{index}"),
                false => found.serial.clone(),
            },
            label: found.label(),
            tuner: format!("R820T2 (fw {version})"),
            ranges: vec![TunerRange {
                range: Hz(FREQ_MIN)..=Hz(FREQ_MAX),
                label: "24 MHz - 1.8 GHz",
            }],
            rates: sorted(&rates).into_iter().map(|r| Sps(r as u64)).collect(),
            rate_range: rate_range(&rates),
            gain_stages: vec![
                GainStage {
                    name: "lna".into(),
                    label: "LNA".into(),
                    range: 0.0..=gain::LNA_DB[gain::LNA_DB.len() - 1],
                    values: gain::LNA_DB.to_vec(),
                    step: 0.0,
                    auto: true,
                },
                GainStage {
                    name: "mixer".into(),
                    label: "Mixer".into(),
                    range: 0.0..=16.1,
                    values: gain::MIXER_DB.to_vec(),
                    step: 0.0,
                    auto: true,
                },
                GainStage {
                    name: "vga".into(),
                    label: "IF VGA (drives the ADC)".into(),
                    range: gain::vga_db(0)..=gain::vga_db(usb::VGA_MAX),
                    values: gain::vga_values(),
                    step: 3.5,
                    auto: false,
                },
            ],
            native_format: SampleFormat::Cs16,
            tunable: true,
            centre_spur: false,
            tx: None,
        };
        let mut me = Self {
            dev: Arc::new(dev),
            info,
            tuning: Default::default(),
            center: Hz::mhz(100),
            rate: Sps(0),
            rates,
            stages: Stages::auto(),
            table: Table::Linearity,
            total: None,
            bias_tee: false,
        };
        let slowest = *me.info.rate_range.start();
        me.set_rate(slowest)?;
        me.set_center(Hz::mhz(100))?;
        me.apply_gain()?;
        me.dev.set_bias_tee(false).map_err(map_err)?;
        Ok(me)
    }

    fn apply_gain(&self) -> Result<()> {
        let s = self.stages;
        self.dev.set_lna_agc(s.lna_agc).map_err(map_err)?;
        self.dev.set_mixer_agc(s.mixer_agc).map_err(map_err)?;
        if !s.lna_agc {
            self.dev.set_lna_gain(s.lna).map_err(map_err)?;
        }
        if !s.mixer_agc {
            self.dev.set_mixer_gain(s.mixer).map_err(map_err)?;
        }
        self.dev.set_vga_gain(s.vga).map_err(map_err)
    }
}

fn sorted(rates: &[u32]) -> Vec<u32> {
    let mut r = rates.to_vec();
    r.sort_unstable();
    r.dedup();
    r
}

pub fn rate_range(rates: &[u32]) -> std::ops::RangeInclusive<Sps> {
    let r = sorted(rates);
    let lo = r.first().copied().unwrap_or(FALLBACK_RATES[1]);
    let hi = r.last().copied().unwrap_or(FALLBACK_RATES[0]);
    Sps(lo as u64)..=Sps(hi as u64)
}

impl Device for Airspy {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }

    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }

    fn set_center(&mut self, f: Hz) -> Result<()> {
        if !self.info.covers(f) {
            return Err(Error::FreqOutOfRange { req: f, lo: Hz(FREQ_MIN), hi: Hz(FREQ_MAX) });
        }
        self.dev.set_frequency(f.0 as u32).map_err(map_err)?;
        self.center = f;
        Ok(())
    }

    fn center(&self) -> Hz {
        self.center
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        let index = self
            .rates
            .iter()
            .position(|&x| x as u64 == r.0)
            .ok_or(Error::RateUnsupported { req: r })?;
        self.dev.set_sample_rate_index(index as u16).map_err(map_err)?;
        self.rate = r;
        Ok(())
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn rate_needs_restart(&self) -> bool {
        true
    }

    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        let mut s = self.stages;
        match (stage, mode) {
            ("tuner" | "", GainMode::Auto) => {
                s = Stages::auto();
                self.total = None;
            }
            ("tuner" | "", GainMode::Manual(db)) => {
                s = self.table.nearest(db);
                self.total = Some(db);
            }
            ("lna", GainMode::Auto) => s.lna_agc = true,
            ("lna", GainMode::Manual(db)) => {
                s.lna_agc = false;
                s.lna = gain::nearest_step(&gain::LNA_DB, db);
            }
            ("mixer", GainMode::Auto) => s.mixer_agc = true,
            ("mixer", GainMode::Manual(db)) => {
                s.mixer_agc = false;
                s.mixer = gain::nearest_step(&gain::MIXER_DB, db);
            }
            ("vga", GainMode::Auto) => s.vga = gain::AUTO_VGA,
            ("vga", GainMode::Manual(db)) => s.vga = gain::nearest_step(&gain::vga_values(), db),
            _ => return Err(Error::other(format!("no gain stage named {stage:?}"))),
        }
        if !matches!(stage, "tuner" | "") {
            self.total = None;
        }
        self.stages = s;
        self.apply_gain()
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        let s = self.stages;
        let level = |auto: bool, db: f32| match auto {
            true => GainMode::Auto,
            false => GainMode::Manual(db),
        };
        vec![
            ("lna".into(), level(s.lna_agc, gain::LNA_DB[s.lna as usize])),
            ("mixer".into(), level(s.mixer_agc, gain::MIXER_DB[s.mixer as usize])),
            ("vga".into(), GainMode::Manual(gain::vga_db(s.vga))),
        ]
    }

    fn toggles(&self) -> Vec<Toggle> {
        vec![Toggle {
            name: BIAS_TEE.into(),
            label: "Bias tee".into(),
            help: "Puts 4.5 V at 50 mA on the antenna socket to power a mast head amplifier. Leave it off unless you know what is on the other end of the cable, because a shorted or DC coupled antenna takes the current.".into(),
            on: self.bias_tee,
        }]
    }

    fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
        match name {
            BIAS_TEE => {
                self.dev.set_bias_tee(on).map_err(map_err)?;
                self.bias_tee = on;
                Ok(())
            }
            _ => Err(Error::other(format!("no setting named {name:?}"))),
        }
    }

    fn choices(&self) -> Vec<Choice> {
        vec![Choice {
            name: GAIN_TABLE.into(),
            label: "Gain table".into(),
            help: "How one gain figure is split across the LNA, mixer and VGA. Linearity keeps the front end low and holds up next to strong signals; sensitivity puts the gain in the LNA for the lowest noise on a quiet band.".into(),
            options: Table::ALL.iter().map(|t| t.name().to_string()).collect(),
            selected: self.table.name().into(),
        }]
    }

    fn set_choice(&mut self, name: &str, value: &str) -> Result<()> {
        if name != GAIN_TABLE {
            return Err(Error::other(format!("no setting named {name:?}")));
        }
        self.table = Table::from_name(value)
            .ok_or_else(|| Error::other(format!("no gain table {value:?}")))?;
        match self.total {
            Some(db) => self.set_gain("tuner", GainMode::Manual(db)),
            None => Ok(()),
        }
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        let reader = self.dev.start_rx().map_err(map_err)?;
        self.dev.set_frequency(self.center.0 as u32).map_err(map_err)?;
        self.apply_gain()?;
        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        let stopper = reader.stopper();
        let ctx = Ctx {
            tx,
            dropped: dropped.clone(),
            center: self.center,
            rate: self.rate,
            seq: 0,
            convert: RealToIq::new(),
        };
        std::thread::Builder::new()
            .name("airspy-rx".into())
            .spawn(move || convert_loop(reader, ctx))?;
        Ok(Box::new(AirspyStream { rx, dropped, stopper }))
    }
}

struct Ctx {
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    center: Hz,
    rate: Sps,
    seq: u64,
    convert: RealToIq,
}

pub fn real_samples(bytes: &[u8], out: &mut Vec<f32>) {
    out.clear();
    out.extend(bytes.chunks_exact(2).map(|w| {
        let v = u16::from_le_bytes([w[0], w[1]]) & 0x0fff;
        (v as f32 - 2048.0) / 2048.0
    }));
}

fn convert_loop(mut reader: usb::Reader, mut ctx: Ctx) {
    let mut real = Vec::new();
    loop {
        let bytes = match reader.read() {
            Ok(b) => b,
            Err(usb::Error::Stopped) => break,
            Err(e) => {
                tracing::warn!("airspy stream ended: {e}");
                break;
            }
        };
        real_samples(&bytes, &mut real);
        let mut samples: Vec<C32> = Vec::with_capacity(real.len() / 2 + 1);
        ctx.convert.process(&real, &mut samples);
        let n = samples.len() as u64;
        let buf = IqBuf::new(samples, ctx.center, ctx.rate, ctx.seq);
        ctx.seq += n;
        match ctx.tx.try_send(buf) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                ctx.dropped.fetch_add(n, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => break,
        }
    }
}

pub(crate) struct AirspyStream {
    pub(crate) rx: Receiver<IqBuf>,
    pub(crate) dropped: Arc<AtomicU64>,
    pub(crate) stopper: usb::Stopper,
}

impl RxStream for AirspyStream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        self.stopper.stop();
    }
}

impl Drop for AirspyStream {
    fn drop(&mut self) {
        self.stopper.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn twelve_bit_offset_binary_is_centred_and_scaled_to_one() {
        let bytes = [0x00, 0x08, 0x00, 0x00, 0xff, 0x0f, 0x00, 0x0c, 0x00, 0xf8];
        let mut out = Vec::new();
        real_samples(&bytes, &mut out);
        assert_eq!(out, vec![0.0, -1.0, 2047.0 / 2048.0, 0.5, 0.0]);
    }

    #[test]
    fn the_model_is_read_off_the_firmware_and_then_its_rates() {
        let r2 = [10_000_000, 2_500_000];
        let mini = [6_000_000, 3_000_000];
        assert_eq!(Model::read("AirSpy NOS v1.0.0-rc10-6-g4008185 2020-05-08", &r2), Model::R2);
        assert_eq!(Model::read("AirSpy MINI v1.0.0-rc10-6-g4008185 2020-05-08", &r2), Model::Mini);
        assert_eq!(Model::read("", &mini), Model::Mini);
        assert_eq!(Model::read("", &[]), Model::Unknown);
    }

    #[test]
    fn the_label_carries_the_serial_tail() {
        let f = |serial: &str, model| {
            Found { index: 0, serial: serial.into(), model, rates: Vec::new() }.label()
        };
        assert_eq!(f("A74068C82F531693", Model::R2), "Airspy R2 2F531693");
        assert_eq!(f("0000000000000042", Model::Mini), "Airspy Mini 42");
        assert_eq!(f("", Model::Unknown), "Airspy");
    }

    #[test]
    fn the_rates_the_firmware_lists_are_the_range_in_either_order() {
        assert_eq!(rate_range(&[10_000_000, 2_500_000]), Sps(2_500_000)..=Sps(10_000_000));
        assert_eq!(rate_range(&[3_000_000, 6_000_000]), Sps(3_000_000)..=Sps(6_000_000));
        assert_eq!(rate_range(&[]), Sps(2_500_000)..=Sps(10_000_000));
    }
}
