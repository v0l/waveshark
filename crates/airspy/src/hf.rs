use crate::map_err;
use airspy_usb::hf as usb;
use common::device::{Choice, Device, DeviceInfo, DriverKind, GainMode, GainStage, RxStream};
use common::{C32, Error, Hz, IqBuf, Result, SampleFormat, Sps, Toggle, TunerRange};
use crossbeam_channel::{Sender, TrySendError, bounded};
use dsp::iq_balance::IqBalance;
use dsp::{DcBlock, Mixer};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

const HF: std::ops::RangeInclusive<u64> = 500..=31_000_000;
const VHF: std::ops::RangeInclusive<u64> = 60_000_000..=260_000_000;
const ZERO_IF_SHIFT_HZ: f64 = 5_000.0;
const ZERO_IF_LO_MIN_KHZ: u32 = 180;
const LOW_IF_LO_MIN_KHZ: u32 = 84;
const FALLBACK_RATE: u32 = 768_000;
const QUEUE_DEPTH: usize = 256;
const SPUR_CUTOFF_HZ: f64 = 20.0;
const PREAMP: &str = "preamp";
const BIAS_TEE: &str = "bias_tee";
const AGC_THRESHOLD: &str = "agc_threshold";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Found {
    pub index: usize,
    pub serial: String,
    pub product: String,
    pub rates: Vec<u32>,
}

impl Found {
    pub fn label(&self) -> String {
        let name = model(&self.product);
        let tail = self.serial.trim_start_matches('0');
        let tail = &tail[tail.len().saturating_sub(8)..];
        match tail.is_empty() {
            true => name.to_string(),
            false => format!("{name} {tail}"),
        }
    }
}

fn model(product: &str) -> &'static str {
    match product.to_ascii_uppercase().contains("DISCOVERY") {
        true => "Airspy HF+ Discovery",
        false => "Airspy HF+",
    }
}

type Known = Mutex<HashMap<String, Vec<u32>>>;

fn known() -> &'static Known {
    static KNOWN: OnceLock<Known> = OnceLock::new();
    KNOWN.get_or_init(Default::default)
}

fn rates_of(dev: &usb::AirspyHf) -> Vec<u32> {
    let rates = dev.sample_rates().unwrap_or_else(|_| vec![FALLBACK_RATE]);
    if let Ok(mut k) = known().lock() {
        k.insert(dev.serial().to_string(), rates.clone());
    }
    rates
}

pub fn enumerate() -> Vec<Found> {
    usb::enumerate()
        .into_iter()
        .map(|e| {
            let cached = known().lock().ok().and_then(|k| k.get(&e.serial).cloned());
            let rates = match cached {
                Some(r) => r,
                None => usb::AirspyHf::open_enumerated(&e)
                    .map(|d| rates_of(&d))
                    .unwrap_or_else(|_| vec![FALLBACK_RATE]),
            };
            Found { index: e.index, serial: e.serial, product: e.product, rates }
        })
        .collect()
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plan {
    pub lo_khz: u32,
    pub adjusted_hz: f64,
}

pub fn plan(f: Hz, ppb: i32, ppm: f64, low_if: bool) -> Plan {
    let adjusted_hz = f.0 as f64 * (1.0 + ppb as f64 * 1e-9 + ppm * 1e-6);
    let (lift, floor) = match low_if {
        true => (0.0, LOW_IF_LO_MIN_KHZ),
        false => (ZERO_IF_SHIFT_HZ, ZERO_IF_LO_MIN_KHZ),
    };
    let lo_khz = (((adjusted_hz + lift) * 1e-3).round().max(0.0) as u32).max(floor);
    Plan { lo_khz, adjusted_hz }
}

impl Plan {
    pub fn shift_hz(&self, delta_hz: f64) -> f64 {
        self.adjusted_hz - self.lo_khz as f64 * 1e3 + delta_hz
    }
}

pub struct AirspyHf {
    dev: Arc<usb::AirspyHf>,
    info: DeviceInfo,
    tuning: common::Tuning,
    center: Hz,
    rate: Sps,
    rates: Vec<u32>,
    low_if: Vec<bool>,
    att_steps: Vec<f32>,
    filter_gain: f32,
    lo_khz: Option<u32>,
    delta_hz: f64,
    ppb: i32,
    ppm: f64,
    shift: Arc<AtomicU64>,
    agc: bool,
    att: usize,
    agc_high: bool,
    preamp: bool,
    bias_tee: bool,
    bias_tees: u32,
}

impl AirspyHf {
    pub fn open(index: usize) -> Result<Self> {
        let found = usb::enumerate().into_iter().nth(index).ok_or(Error::NoDevice)?;
        let dev = usb::AirspyHf::open_enumerated(&found).map_err(map_err)?;
        let rates = rates_of(&dev);
        let low_if = dev.low_if(rates.len()).unwrap_or_else(|_| vec![false; rates.len()]);
        let mut att_steps =
            dev.att_steps().unwrap_or_else(|_| (0..=8).map(|i| i as f32 * 6.0).collect());
        att_steps.sort_by(f32::total_cmp);
        let bias_tees = dev.bias_tees();
        let calibration = dev.calibration().ok().flatten();
        if let Some(c) = calibration {
            dev.set_vctcxo(c.vctcxo).map_err(map_err)?;
            dev.set_frontend_options(c.frontend).map_err(map_err)?;
        }
        let version = dev.version().unwrap_or_default();
        let found =
            Found { index, serial: found.serial, product: found.product, rates: rates.clone() };
        let mut attenuation: Vec<f32> = att_steps.iter().map(|a| -a).collect();
        attenuation.sort_by(f32::total_cmp);
        let info = DeviceInfo {
            kind: DriverKind::AirspyHf,
            id: match found.serial.is_empty() {
                true => format!("airspyhf{index}"),
                false => found.serial.clone(),
            },
            label: found.label(),
            tuner: format!("{} (fw {version})", model(&found.product)),
            ranges: vec![
                TunerRange { range: Hz(*HF.start())..=Hz(*HF.end()), label: "HF 0.5 kHz - 31 MHz" },
                TunerRange { range: Hz(*VHF.start())..=Hz(*VHF.end()), label: "VHF 60 - 260 MHz" },
            ],
            rates: sorted(&rates).into_iter().map(|r| Sps(r as u64)).collect(),
            rate_range: rate_range(&rates),
            gain_stages: vec![GainStage {
                name: "att".into(),
                label: "RF attenuator".into(),
                range: attenuation.first().copied().unwrap_or(-48.0)..=0.0,
                values: attenuation,
                step: 0.0,
                auto: true,
            }],
            native_format: SampleFormat::Cs16,
            tunable: true,
            centre_spur: false,
            tx: None,
        };
        let mut me = Self {
            dev: Arc::new(dev),
            info,
            tuning: Default::default(),
            center: Hz(7_100_000),
            rate: Sps(0),
            rates,
            low_if,
            att_steps,
            filter_gain: 1.0,
            lo_khz: None,
            delta_hz: 0.0,
            ppb: calibration.map(|c| c.ppb).unwrap_or(0),
            ppm: 0.0,
            shift: Arc::new(AtomicU64::new(0f64.to_bits())),
            agc: true,
            att: 0,
            agc_high: false,
            preamp: false,
            bias_tee: false,
            bias_tees,
        };
        let start = match me.rates.contains(&FALLBACK_RATE) {
            true => FALLBACK_RATE,
            false => me.rates.iter().copied().max().unwrap_or(FALLBACK_RATE),
        };
        me.set_rate(Sps(start as u64))?;
        me.apply_gain()?;
        me.dev.set_lna(false).map_err(map_err)?;
        Ok(me)
    }

    fn index(&self) -> Option<usize> {
        self.rates.iter().position(|&x| x as u64 == self.rate.0)
    }

    fn is_low_if(&self) -> bool {
        self.index().and_then(|i| self.low_if.get(i).copied()).unwrap_or(false)
    }

    fn retune(&mut self, f: Hz) -> Result<()> {
        let p = plan(f, self.ppb, self.ppm, self.is_low_if());
        if self.lo_khz != Some(p.lo_khz) {
            self.dev.set_frequency_khz(p.lo_khz).map_err(map_err)?;
            self.lo_khz = Some(p.lo_khz);
            self.delta_hz = self.dev.freq_delta_hz().unwrap_or(0.0);
        }
        self.shift.store(p.shift_hz(self.delta_hz).to_bits(), Ordering::Relaxed);
        self.center = f;
        Ok(())
    }

    fn apply_gain(&self) -> Result<()> {
        self.dev.set_agc(self.agc).map_err(map_err)?;
        self.dev.set_agc_threshold_high(self.agc_high).map_err(map_err)?;
        if !self.agc {
            self.dev.set_att(self.att as u16).map_err(map_err)?;
        }
        Ok(())
    }

    fn att_db(&self) -> f32 {
        self.att_steps.get(self.att).copied().unwrap_or(0.0)
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
    let lo = r.first().copied().unwrap_or(FALLBACK_RATE);
    let hi = r.last().copied().unwrap_or(FALLBACK_RATE);
    Sps(lo as u64)..=Sps(hi as u64)
}

impl Device for AirspyHf {
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
            return Err(Error::FreqOutOfRange { req: f, lo: Hz(*HF.start()), hi: Hz(*VHF.end()) });
        }
        self.retune(f)
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
        let low_if = self.low_if.get(index).copied().unwrap_or(false);
        if !low_if && self.lo_khz.is_some_and(|k| k < ZERO_IF_LO_MIN_KHZ) {
            self.dev.set_frequency_khz(ZERO_IF_LO_MIN_KHZ).map_err(map_err)?;
            self.lo_khz = Some(ZERO_IF_LO_MIN_KHZ);
        }
        self.dev.set_sample_rate_index(index as u16).map_err(map_err)?;
        self.filter_gain =
            self.dev.filter_gain_db().map(|db| 10f32.powf(-0.05 * db as f32)).unwrap_or(1.0);
        self.rate = r;
        self.lo_khz = None;
        self.retune(self.center)
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn rate_needs_restart(&self) -> bool {
        true
    }

    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        if !matches!(stage, "att" | "tuner" | "") {
            return Err(Error::other(format!("no gain stage named {stage:?}")));
        }
        match mode {
            GainMode::Auto => self.agc = true,
            GainMode::Manual(db) => {
                let want = (-db).max(0.0);
                self.agc = false;
                self.att = self
                    .att_steps
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| (*a - want).abs().total_cmp(&(*b - want).abs()))
                    .map(|(i, _)| i)
                    .unwrap_or(0);
            }
        }
        self.apply_gain()
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        let mode = match self.agc {
            true => GainMode::Auto,
            false => GainMode::Manual(-self.att_db()),
        };
        vec![("att".into(), mode)]
    }

    fn toggles(&self) -> Vec<Toggle> {
        let mut v = vec![Toggle {
            name: PREAMP.into(),
            label: "Preamp".into(),
            help: "Six decibels of gain ahead of the mixer, taken back out after the converter, so it lowers the noise figure without moving the level. Leave it off next to strong broadcast stations.".into(),
            on: self.preamp,
        }];
        if self.bias_tees > 0 {
            v.push(Toggle {
                name: BIAS_TEE.into(),
                label: "Bias tee".into(),
                help: "Puts power on the antenna socket for an active antenna or a mast head amplifier. Leave it off unless you know what is on the other end of the cable, because a shorted or DC coupled antenna takes the current.".into(),
                on: self.bias_tee,
            });
        }
        v
    }

    fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
        match name {
            PREAMP => {
                self.dev.set_lna(on).map_err(map_err)?;
                self.preamp = on;
            }
            BIAS_TEE if self.bias_tees > 0 => {
                self.dev.set_bias_tee(on).map_err(map_err)?;
                self.bias_tee = on;
            }
            _ => return Err(Error::other(format!("no setting named {name:?}"))),
        }
        Ok(())
    }

    fn choices(&self) -> Vec<Choice> {
        vec![Choice {
            name: AGC_THRESHOLD.into(),
            label: "AGC threshold".into(),
            help: "Where the automatic attenuator starts to act. Low keeps strong stations further from overload; high holds the gain up for weak ones on a quiet band.".into(),
            options: vec!["Low".into(), "High".into()],
            selected: if self.agc_high { "High" } else { "Low" }.into(),
        }]
    }

    fn set_choice(&mut self, name: &str, value: &str) -> Result<()> {
        if name != AGC_THRESHOLD {
            return Err(Error::other(format!("no setting named {name:?}")));
        }
        self.agc_high = match value {
            "Low" => false,
            "High" => true,
            other => return Err(Error::other(format!("no AGC threshold {other:?}"))),
        };
        self.apply_gain()
    }

    fn set_ppm(&mut self, ppm: f64) -> Result<()> {
        self.ppm = ppm;
        self.lo_khz = None;
        self.retune(self.center)
    }

    fn ppm(&self) -> f64 {
        self.ppm
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        let reader = self.dev.start_rx().map_err(map_err)?;
        let stopper = reader.stopper();
        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        let convert = Convert::new(
            self.rate.as_f64(),
            self.filter_gain,
            self.is_low_if(),
            self.shift.clone(),
        );
        let ctx =
            Ctx { tx, dropped: dropped.clone(), center: self.center, rate: self.rate, convert };
        std::thread::Builder::new()
            .name("airspyhf-rx".into())
            .spawn(move || convert_loop(reader, ctx))?;
        Ok(Box::new(crate::AirspyStream { rx, dropped, stopper }))
    }
}

pub struct Convert {
    scale: f32,
    rate: f64,
    zero_if: Option<(DcBlock, IqBalance)>,
    mixer: Mixer,
    shift: Arc<AtomicU64>,
    applied: f64,
}

impl Convert {
    pub fn new(rate: f64, filter_gain: f32, low_if: bool, shift: Arc<AtomicU64>) -> Self {
        let zero_if =
            (!low_if).then(|| (DcBlock::with_cutoff(rate, SPUR_CUTOFF_HZ), IqBalance::new(rate)));
        Self {
            scale: filter_gain / 32768.0,
            rate,
            zero_if,
            mixer: Mixer::new(0.0, rate),
            shift,
            applied: 0.0,
        }
    }

    pub fn process(&mut self, bytes: &[u8]) -> Vec<C32> {
        let mut out: Vec<C32> = bytes
            .chunks_exact(4)
            .map(|w| {
                let im = i16::from_le_bytes([w[0], w[1]]) as f32;
                let re = i16::from_le_bytes([w[2], w[3]]) as f32;
                C32::new(re * self.scale, im * self.scale)
            })
            .collect();
        if let Some((dc, balance)) = &mut self.zero_if {
            dc.process(&mut out);
            balance.process(&mut out);
        }
        let shift = f64::from_bits(self.shift.load(Ordering::Relaxed));
        if shift != self.applied {
            self.mixer.set_shift(-shift, self.rate);
            self.applied = shift;
        }
        self.mixer.process_in_place(&mut out);
        out
    }
}

struct Ctx {
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    center: Hz,
    rate: Sps,
    convert: Convert,
}

fn convert_loop(mut reader: airspy_usb::Reader, mut ctx: Ctx) {
    let mut seq = 0u64;
    loop {
        let bytes = match reader.read() {
            Ok(b) => b,
            Err(airspy_usb::Error::Stopped) => break,
            Err(e) => {
                tracing::warn!("airspy hf+ stream ended: {e}");
                break;
            }
        };
        let samples = ctx.convert.process(&bytes);
        let n = samples.len() as u64;
        let buf = IqBuf::new(samples, ctx.center, ctx.rate, seq);
        seq += n;
        match ctx.tx.try_send(buf) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                ctx.dropped.fetch_add(n, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => break,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_zero_if_rate_puts_the_oscillator_five_kilohertz_up_and_shifts_back_down() {
        let p = plan(Hz(7_074_000), 0, 0.0, false);
        assert_eq!(p.lo_khz, 7_079);
        assert_eq!(p.shift_hz(0.0), -5_000.0);
        assert_eq!(p.shift_hz(12.5), -4_987.5, "the synthesiser's own error is added back");
    }

    #[test]
    fn a_low_if_rate_puts_the_oscillator_on_the_dial() {
        let p = plan(Hz(7_074_300), 0, 0.0, true);
        assert_eq!(p.lo_khz, 7_074);
        assert_eq!(p.shift_hz(0.0), 300.0);
    }

    #[test]
    fn below_the_oscillators_floor_the_rest_is_a_shift_inside_the_span() {
        let zero = plan(Hz(77_500), 0, 0.0, false);
        assert_eq!((zero.lo_khz, zero.shift_hz(0.0)), (180, -102_500.0), "DCF77 at zero IF");
        let low = plan(Hz(60_000), 0, 0.0, true);
        assert_eq!((low.lo_khz, low.shift_hz(0.0)), (84, -24_000.0), "MSF at low IF");
    }

    #[test]
    fn the_flash_calibration_and_the_operators_correction_both_move_the_oscillator() {
        let p = plan(Hz(10_000_000), 1_000, 0.0, false);
        assert_eq!(p.lo_khz, 10_005);
        assert!((p.shift_hz(0.0) + 4_990.0).abs() < 1e-6);
        let q = plan(Hz(10_000_000), 1_000, -1.0, false);
        assert!((q.adjusted_hz - 10_000_000.0).abs() < 1e-6);
    }

    #[test]
    fn the_labels_follow_the_product_string_and_the_serial() {
        let f = |product: &str| {
            Found {
                index: 0,
                serial: "3952C3DA2A3C0B35".into(),
                product: product.into(),
                rates: vec![],
            }
            .label()
        };
        assert_eq!(f("AIRSPY HF+ Discovery"), "Airspy HF+ Discovery 2A3C0B35");
        assert_eq!(f("AIRSPY HF+"), "Airspy HF+ 2A3C0B35");
        assert_eq!(f(""), "Airspy HF+ 2A3C0B35");
    }

    const RATE: f64 = 768_000.0;

    fn raw(n: usize, tones: &[(f64, f64)], dc: (f64, f64), gain: f64, phase_deg: f64) -> Vec<u8> {
        let (sp, cp) = phase_deg.to_radians().sin_cos();
        let mut out = Vec::with_capacity(n * 4);
        for k in 0..n {
            let (mut i, mut q) = dc;
            for (hz, a) in tones {
                let t = std::f64::consts::TAU * hz * k as f64 / RATE;
                i += a * t.cos();
                q += a * t.sin();
            }
            let q = gain * (q * cp - i * sp);
            out.extend(((q * 32767.0).round() as i16).to_le_bytes());
            out.extend(((i * 32767.0).round() as i16).to_le_bytes());
        }
        out
    }

    fn at_db(x: &[C32], hz: f64) -> f64 {
        let step = -std::f64::consts::TAU * hz / RATE;
        let n = x.len() as f64;
        let (mut re, mut im, mut sum) = (0.0f64, 0.0f64, 0.0f64);
        for (k, s) in x.iter().enumerate() {
            let w = 0.5 - 0.5 * (std::f64::consts::TAU * k as f64 / n).cos();
            let (sn, cs) = (step * k as f64).sin_cos();
            re += w * (s.re as f64 * cs - s.im as f64 * sn);
            im += w * (s.re as f64 * sn + s.im as f64 * cs);
            sum += w;
        }
        20.0 * ((re * re + im * im).sqrt() / sum).log10()
    }

    fn run(bytes: &[u8], low_if: bool, shift: f64) -> Vec<C32> {
        let mut c = Convert::new(RATE, 1.0, low_if, Arc::new(AtomicU64::new(shift.to_bits())));
        bytes.chunks(usb::TRANSFER_BYTES).flat_map(|b| c.process(b)).collect()
    }

    #[test]
    fn the_quadrature_word_comes_first_in_each_sample() {
        let mut bytes = Vec::new();
        bytes.extend(16384i16.to_le_bytes());
        bytes.extend((-8192i16).to_le_bytes());
        let mut c = Convert::new(RATE, 2.0, true, Arc::new(AtomicU64::new(0f64.to_bits())));
        assert_eq!(c.process(&bytes), vec![C32::new(-0.5, 1.0)]);
    }

    #[test]
    fn an_ft8_signal_at_zero_if_lands_on_the_dial_with_its_image_100_db_and_the_spur_120_db_down() {
        let p = plan(Hz(7_074_000), 0, 0.0, false);
        let offset = 7_075_500.0 - p.lo_khz as f64 * 1e3;
        let bytes = raw(RATE as usize, &[(offset, 0.3)], (0.02, -0.01), 1.03, 2.0);
        let out = run(&bytes, false, p.shift_hz(0.0));
        let tail = &out[out.len() - 65_536..];
        let wanted = at_db(tail, 1_500.0);
        let image = at_db(tail, 8_500.0) - wanted;
        let spur = at_db(tail, 5_000.0) - wanted;
        assert!((wanted - 20.0 * 0.3f64.log10()).abs() < 0.1, "the signal read {wanted:.2} dB");
        assert!(image < -100.0, "its image is {image:.1} dB below it");
        assert!(spur < -120.0, "the oscillator's spur is {spur:.1} dB below it");
    }

    #[test]
    fn a_low_if_stream_is_shifted_and_its_33_db_image_left_to_the_firmware() {
        let bytes = raw(200_000, &[(20_300.0, 0.3)], (0.0, 0.0), 1.03, 2.0);
        let out = run(&bytes, true, 300.0);
        let tail = &out[out.len() - 65_536..];
        let wanted = at_db(tail, 20_000.0);
        let image = at_db(tail, -20_600.0) - wanted;
        assert!((wanted - 20.0 * 0.3f64.log10()).abs() < 0.2, "the signal read {wanted:.2} dB");
        assert!(
            (-34.0..-32.0).contains(&image),
            "the firmware's image is left alone at {image:.1} dB"
        );
    }
}
