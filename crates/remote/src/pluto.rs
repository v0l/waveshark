use crate::gaps::Gaps;
use crate::iiod::{Attr, Client, Context, Format};
use crate::{Probe, Proto, QUEUE_DEPTH};
use common::device::{
    Choice, Device as DeviceTrait, DeviceInfo, DriverKind, GainMode, GainStage, RxStream,
    TunerRange, TxInfo, TxStream,
};
use common::time::Duration;
use common::{C32, Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use std::ops::RangeInclusive;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

const PHY: &str = "ad9361-phy";
const ADC: &str = "cf-ad9361-lpc";
const DAC: &str = "cf-ad9361-dds-core-lpc";

pub const USB_VID: u16 = 0x0456;
pub const USB_PID: u16 = 0xb673;
pub const USB_ADDR: &str = "192.168.2.1:30431";

const DIRECT_MIN: u64 = 2_083_334;
const DECIMATION: u64 = 8;
const LINK_MAX: u64 = 4_000_000;
pub const RATES: RangeInclusive<Sps> = Sps(DIRECT_MIN.div_ceil(DECIMATION))..=Sps(LINK_MAX);

const SHIPPED: RangeInclusive<Hz> = Hz(325_000_000)..=Hz(3_800_000_000);
const TX_ATTENUATION: f32 = 89.75;
const RX_GAIN: RangeInclusive<f32> = -3.0..=71.0;
const BANDWIDTH: RangeInclusive<u64> = 200_000..=56_000_000;
const KERNEL_BUFFERS: usize = 4;
const AHEAD: usize = 1;
const READ_TIMEOUT: Duration = Duration::from_secs(5);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Agc {
    Slow,
    Fast,
}

impl Agc {
    const ALL: [Agc; 2] = [Agc::Slow, Agc::Fast];

    fn name(self) -> &'static str {
        match self {
            Self::Slow => "slow",
            Self::Fast => "fast",
        }
    }

    fn mode(self) -> &'static str {
        match self {
            Self::Slow => "slow_attack",
            Self::Fast => "fast_attack",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|a| a.name() == s || a.mode() == s)
    }
}

fn available(s: &str) -> Option<(f64, f64, f64)> {
    let mut it = s.trim().trim_start_matches('[').trim_end_matches(']').split_whitespace();
    let lo = it.next()?.parse().ok()?;
    let step = it.next()?.parse().ok()?;
    let hi = it.next()?.parse().ok()?;
    Some((lo, step, hi))
}

fn leading_number(s: &str) -> Option<f64> {
    s.split_whitespace().next()?.parse().ok()
}

fn range_label(r: &RangeInclusive<Hz>) -> &'static str {
    match (r.start().0, r.end().0) {
        (325_000_000, 3_800_000_000) => "325 MHz - 3.8 GHz",
        (70_000_000, 6_000_000_000) => "70 MHz - 6 GHz",
        (46_875_001, 6_000_000_000) => "47 MHz - 6 GHz",
        _ => "AD936x",
    }
}

fn block_samples(rate: Sps) -> usize {
    ((rate.as_f64() / 50.0) as usize).clamp(4096, 1 << 18)
}

fn block_time(rate: Sps) -> Duration {
    Duration::from_secs_f64(block_samples(rate) as f64 / rate.as_f64().max(1.0))
}

struct Found {
    label: String,
    id: String,
    tuner: String,
    rx_range: RangeInclusive<Hz>,
    tx_range: RangeInclusive<Hz>,
    gain: (f32, f32, f32),
}

fn lo_range(c: &mut Client, chan: &str) -> RangeInclusive<Hz> {
    let at = Attr::Channel { dev: PHY, chan, output: true, attr: "frequency_available" };
    match c.read(at).ok().as_deref().and_then(available) {
        Some((lo, _, hi)) if hi > lo => Hz(lo as u64)..=Hz(hi as u64),
        _ => SHIPPED,
    }
}

fn describe(c: &mut Client, ctx: &Context, addr: &str) -> Result<Found> {
    if ctx.device(PHY).is_none() || ctx.device(ADC).is_none() {
        return Err(Error::other(format!("{addr} is iiod but has no AD936x on it")));
    }
    let serial = ctx.attr("hw_serial").unwrap_or("");
    let tail = &serial[serial.len().saturating_sub(8)..];
    let model = ctx.attr("hw_model").unwrap_or("");
    let board = match model.contains("Pluto") {
        true => "ADALM-PLUTO",
        false => model.split(" (").next().filter(|m| !m.is_empty()).unwrap_or("AD936x"),
    };
    let label = match tail.is_empty() {
        true => format!("{board} {addr}"),
        false => format!("{board} {tail}"),
    };
    let id = match serial.is_empty() {
        true => format!("pluto:{addr}"),
        false => format!("pluto:{serial}"),
    };
    let tuner = ctx.attr("ad9361-phy,model").map(str::to_uppercase).unwrap_or("AD936x".into());
    let gain_at =
        Attr::Channel { dev: PHY, chan: "voltage0", output: false, attr: "hardwaregain_available" };
    let gain = match c.read(gain_at).ok().as_deref().and_then(available) {
        Some((lo, step, hi)) if hi > lo => (lo as f32, step as f32, hi as f32),
        _ => (*RX_GAIN.start(), 1.0, *RX_GAIN.end()),
    };
    Ok(Found {
        label,
        id,
        tuner,
        rx_range: lo_range(c, "altvoltage0"),
        tx_range: lo_range(c, "altvoltage1"),
        gain,
    })
}

pub fn probe(addr: &str) -> Result<Probe> {
    let addr = Proto::Pluto.parse_addr(addr)?;
    let mut c = Client::connect(&addr)?;
    let ctx = c.context()?;
    let found = describe(&mut c, &ctx, &addr)?;
    Ok(Probe {
        proto: Proto::Pluto,
        addr,
        center: None,
        rate: None,
        rates: Vec::new(),
        rate_range: Some(RATES),
        gain_db: None,
        name: String::new(),
        settings: Vec::new(),
        tunable: true,
        tune_range: Some(found.rx_range),
        tuner: found.tuner,
    })
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Attached {
    pub serial: String,
}

impl Attached {
    pub fn label(&self) -> String {
        let tail = &self.serial[self.serial.len().saturating_sub(8)..];
        match tail.is_empty() {
            true => "ADALM-PLUTO".to_string(),
            false => format!("ADALM-PLUTO {tail}"),
        }
    }
}

#[cfg(feature = "usb")]
pub fn attached() -> Vec<Attached> {
    use nusb::MaybeFuture;
    let Ok(devices) = nusb::list_devices().wait() else { return Vec::new() };
    devices
        .filter(|d| d.vendor_id() == USB_VID && d.product_id() == USB_PID)
        .map(|d| Attached { serial: d.serial_number().unwrap_or("").to_string() })
        .collect()
}

pub struct Pluto {
    addr: String,
    ctl: Client,
    info: DeviceInfo,
    tuning: common::Tuning,
    center: Hz,
    rate: Sps,
    decimator: bool,
    dac_rate: bool,
    rx_mask: String,
    rx_format: [Format; 2],
    tx: Option<(String, [Format; 2])>,
    gain: GainMode,
    agc: Agc,
    xo: Option<f64>,
    ppm: f64,
    tx_center: Hz,
    tx_level: f32,
    streaming: Arc<AtomicBool>,
    transmitting: Arc<AtomicBool>,
}

fn stream_formats(ctx: &Context, dev: &str, output: bool) -> Result<(String, [Format; 2])> {
    let d = ctx.device(dev).ok_or_else(|| Error::other(format!("no {dev}")))?;
    let mask = d.mask(&[("voltage0", output), ("voltage1", output)])?;
    let format = |id: &str| {
        d.channel(id, output)
            .and_then(|c| c.scan)
            .map(|s| s.format)
            .filter(|f| f.storage == 16 && f.repeat == 1)
            .ok_or_else(|| Error::other(format!("{dev} {id} is not a 16-bit stream")))
    };
    Ok((mask, [format("voltage0")?, format("voltage1")?]))
}

impl Pluto {
    pub fn open(addr: &str, kind: DriverKind) -> Result<Self> {
        let addr = Proto::Pluto.parse_addr(addr)?;
        let mut ctl = Client::connect(&addr)?;
        let ctx = ctl.context()?;
        let found = describe(&mut ctl, &ctx, &addr)?;
        let (rx_mask, rx_format) = stream_formats(&ctx, ADC, false)?;
        let tx = stream_formats(&ctx, DAC, true).ok();
        let has = |dev: &str, chan: &str, output: bool, attr: &str| {
            ctx.device(dev)
                .and_then(|d| d.channel(chan, output))
                .is_some_and(|c| c.attrs.iter().any(|a| a == attr))
        };
        let decimator = has(ADC, "voltage0", false, "sampling_frequency");
        let dac_rate = has(DAC, "voltage0", true, "sampling_frequency");
        let xo = ctl
            .read(Attr::Device { dev: PHY, attr: "xo_correction" })
            .ok()
            .and_then(|v| leading_number(&v))
            .filter(|v| *v > 0.0);
        let low = match decimator {
            true => *RATES.start(),
            false => Sps(DIRECT_MIN),
        };
        let rate_range = low..=*RATES.end();
        let (glo, gstep, ghi) = found.gain;
        let info = DeviceInfo {
            kind,
            id: found.id,
            label: found.label,
            tuner: found.tuner,
            ranges: vec![TunerRange {
                label: range_label(&found.rx_range),
                range: found.rx_range.clone(),
            }],
            rates: Vec::new(),
            rate_range: rate_range.clone(),
            gain_stages: vec![GainStage {
                name: "rf".into(),
                label: "RF gain (AD936x gain table)".into(),
                range: glo..=ghi,
                values: Vec::new(),
                step: gstep,
                auto: true,
            }],
            native_format: SampleFormat::Cs16,
            tunable: true,
            centre_spur: true,
            tx: tx.as_ref().map(|_| TxInfo {
                ranges: vec![TunerRange {
                    label: range_label(&found.tx_range),
                    range: found.tx_range.clone(),
                }],
                rate_range,
                gain_stages: vec![GainStage {
                    name: "gain".into(),
                    label: "TX level (above full attenuation)".into(),
                    range: 0.0..=TX_ATTENUATION,
                    values: Vec::new(),
                    step: 0.25,
                    auto: false,
                }],
                native_format: SampleFormat::Cs16,
                half_duplex: false,
                channels: 1,
            }),
        };
        let start = Hz(433_920_000).clamp(*found.rx_range.start(), *found.rx_range.end());
        let tx_start = start.clamp(*found.tx_range.start(), *found.tx_range.end());
        let mut me = Self {
            addr,
            ctl,
            info,
            tuning: Default::default(),
            center: start,
            rate: Sps(2_400_000),
            decimator,
            dac_rate,
            rx_mask,
            rx_format,
            tx,
            gain: GainMode::Auto,
            agc: Agc::Slow,
            xo,
            ppm: 0.0,
            tx_center: tx_start,
            tx_level: 0.0,
            streaming: Arc::new(AtomicBool::new(false)),
            transmitting: Arc::new(AtomicBool::new(false)),
        };
        if me.tx.is_some() {
            me.write_tx_level(0.0)?;
            me.ctl.write(tx_lo("powerdown"), "1")?;
        }
        me.set_rate(Sps(2_400_000))?;
        me.set_center(start)?;
        me.set_gain("rf", GainMode::Auto)?;
        Ok(me)
    }

    pub fn address(&self) -> &str {
        &self.addr
    }

    fn write_tx_level(&mut self, level: f32) -> Result<()> {
        let at = Attr::Channel { dev: PHY, chan: "voltage0", output: true, attr: "hardwaregain" };
        self.ctl.write(at, &format!("{:.2}", level - TX_ATTENUATION))
    }
}

fn rx_path(attr: &str) -> Attr<'_> {
    Attr::Channel { dev: PHY, chan: "voltage0", output: false, attr }
}

fn tx_lo(attr: &str) -> Attr<'_> {
    Attr::Channel { dev: PHY, chan: "altvoltage1", output: true, attr }
}

impl DeviceTrait for Pluto {
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
        let r = &self.info.ranges[0].range;
        if !r.contains(&f) {
            return Err(Error::FreqOutOfRange { req: f, lo: *r.start(), hi: *r.end() });
        }
        let at = Attr::Channel { dev: PHY, chan: "altvoltage0", output: true, attr: "frequency" };
        self.ctl.write(at, &f.0.to_string())?;
        self.center = f;
        Ok(())
    }

    fn center(&self) -> Hz {
        self.center
    }

    fn settle(&self) -> Duration {
        block_time(self.rate) * (AHEAD as u32 + 2)
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        if !self.info.rate_range.contains(&r) {
            return Err(Error::RateUnsupported { req: r });
        }
        let decimate = r.0 < DIRECT_MIN;
        let chip = if decimate { r.0 * DECIMATION } else { r.0 };
        self.ctl.write(rx_path("sampling_frequency"), &chip.to_string())?;
        if self.decimator {
            let at = Attr::Channel {
                dev: ADC,
                chan: "voltage0",
                output: false,
                attr: "sampling_frequency",
            };
            self.ctl.write(at, &r.0.to_string())?;
        }
        if self.dac_rate && self.tx.is_some() {
            let at = Attr::Channel {
                dev: DAC,
                chan: "voltage0",
                output: true,
                attr: "sampling_frequency",
            };
            self.ctl.write(at, &r.0.to_string())?;
        }
        let bw = r.0.clamp(*BANDWIDTH.start(), *BANDWIDTH.end()).to_string();
        self.ctl.write(rx_path("rf_bandwidth"), &bw)?;
        if self.tx.is_some() {
            let at =
                Attr::Channel { dev: PHY, chan: "voltage0", output: true, attr: "rf_bandwidth" };
            self.ctl.write(at, &bw)?;
        }
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
        if stage != "rf" {
            return Err(Error::other(format!("{} has no {stage} gain", self.info.label)));
        }
        match mode {
            GainMode::Auto => self.ctl.write(rx_path("gain_control_mode"), self.agc.mode())?,
            GainMode::Manual(db) => {
                let db = self.info.gain_stages[0].quantise(db);
                self.ctl.write(rx_path("gain_control_mode"), "manual")?;
                self.ctl.write(rx_path("hardwaregain"), &format!("{db}"))?;
            }
        }
        self.gain = mode;
        Ok(())
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        vec![("rf".into(), self.gain)]
    }

    fn choices(&self) -> Vec<Choice> {
        vec![Choice {
            name: "agc".into(),
            label: "AGC".into(),
            help: "How the AD936x moves its own gain while RF gain is on automatic. Slow \
                   follows the noise floor and holds through a burst; fast drops the gain \
                   at the start of each burst and suits packet traffic."
                .into(),
            options: Agc::ALL.iter().map(|a| a.name().to_string()).collect(),
            selected: self.agc.name().to_string(),
        }]
    }

    fn set_choice(&mut self, name: &str, value: &str) -> Result<()> {
        if name != "agc" {
            return Err(Error::other(format!("no setting named {name:?}")));
        }
        let agc =
            Agc::parse(value).ok_or_else(|| Error::other(format!("no AGC mode {value:?}")))?;
        self.agc = agc;
        if self.gain == GainMode::Auto {
            self.ctl.write(rx_path("gain_control_mode"), agc.mode())?;
        }
        Ok(())
    }

    fn set_ppm(&mut self, ppm: f64) -> Result<()> {
        let base = self.xo.ok_or_else(|| Error::other("this firmware has no xo_correction"))?;
        let hz = (base * (1.0 + ppm * 1e-6)).round() as u64;
        self.ctl.write(Attr::Device { dev: PHY, attr: "xo_correction" }, &hz.to_string())?;
        self.ppm = ppm;
        Ok(())
    }

    fn ppm(&self) -> f64 {
        self.ppm
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let started = (|| {
            let block = block_samples(self.rate);
            let mut c = Client::connect(&self.addr)?;
            let _ = c.set_buffers(ADC, KERNEL_BUFFERS);
            c.open(ADC, block, &self.rx_mask)?;
            c.set_read_timeout(Some(READ_TIMEOUT))?;
            Ok::<_, Error>((c, block))
        })();
        let (client, block) = match started {
            Ok(s) => s,
            Err(e) => {
                self.streaming.store(false, Ordering::SeqCst);
                return Err(e);
            }
        };
        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let sock = client.shutdown_handle();
        let pump = Pump {
            client,
            block,
            format: self.rx_format,
            center: self.center,
            rate: self.rate,
            tx,
            dropped: dropped.clone(),
            stop: stop.clone(),
        };
        let streaming = self.streaming.clone();
        let addr = self.addr.clone();
        let join = common::thread::Builder::new()
            .name("pluto-rx".into())
            .spawn(move || {
                if let Err(e) = pump.run() {
                    tracing::warn!("pluto {addr}: {e}");
                }
                streaming.store(false, Ordering::SeqCst);
            })
            .map_err(|e| Error::other(format!("spawn rx thread: {e}")))?;
        Ok(Box::new(Stream { rx, dropped, stop, sock, join: Some(join) }))
    }

    fn set_tx_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        if self.tx.is_none() {
            return Err(Error::TxUnsupported);
        }
        if stage != "gain" && !stage.is_empty() {
            return Err(Error::other(format!("no transmit gain stage named {stage}")));
        }
        let level = match mode {
            GainMode::Auto => 0.0,
            GainMode::Manual(db) => ((db * 4.0).round() / 4.0).clamp(0.0, TX_ATTENUATION),
        };
        self.write_tx_level(level)?;
        self.tx_level = level;
        Ok(())
    }

    fn tx_gains(&self) -> Vec<(String, GainMode)> {
        match self.tx {
            Some(_) => vec![("gain".into(), GainMode::Manual(self.tx_level))],
            None => Vec::new(),
        }
    }

    fn set_tx_center(&mut self, f: Hz) -> Result<()> {
        let Some(t) = &self.info.tx else { return Err(Error::TxUnsupported) };
        let r = &t.ranges[0].range;
        if !r.contains(&f) {
            return Err(Error::FreqOutOfRange { req: f, lo: *r.start(), hi: *r.end() });
        }
        self.ctl.write(tx_lo("frequency"), &f.0.to_string())?;
        self.tx_center = f;
        Ok(())
    }

    fn tx_center(&self) -> Hz {
        self.tx_center
    }

    fn start_tx(&mut self) -> Result<Box<dyn TxStream>> {
        let Some((mask, format)) = self.tx.clone() else { return Err(Error::TxUnsupported) };
        if self.transmitting.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let block = block_samples(self.rate);
        let started = (|| {
            self.ctl.write(tx_lo("frequency"), &self.tx_center.0.to_string())?;
            self.write_tx_level(self.tx_level)?;
            self.ctl.write(tx_lo("powerdown"), "0")?;
            let mut c = Client::connect(&self.addr)?;
            let _ = c.set_buffers(DAC, KERNEL_BUFFERS);
            c.open(DAC, block, &mask)?;
            c.set_read_timeout(Some(READ_TIMEOUT))?;
            Ok::<_, Error>(c)
        })();
        match started {
            Ok(client) => Ok(Box::new(Transmit {
                client,
                bytes: block * 4,
                pending: Vec::with_capacity(block * 4),
                format,
                rate: self.rate,
                stopped: false,
                transmitting: self.transmitting.clone(),
            })),
            Err(e) => {
                let _ = self.ctl.write(tx_lo("powerdown"), "1");
                self.transmitting.store(false, Ordering::SeqCst);
                Err(e)
            }
        }
    }
}

struct Pump {
    client: Client,
    block: usize,
    format: [Format; 2],
    center: Hz,
    rate: Sps,
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
}

impl Pump {
    fn run(mut self) -> Result<()> {
        let bytes = self.block * 4;
        for _ in 0..=AHEAD {
            self.client.request(ADC, bytes)?;
        }
        let mut raw = vec![0u8; bytes];
        let mut gaps = Gaps::new(self.rate);
        while !self.stop.load(Ordering::Relaxed) {
            match self.client.receive(&mut raw) {
                Ok(()) => {}
                Err(_) if self.stop.load(Ordering::Relaxed) => break,
                Err(Error::Disconnected) => break,
                Err(e) => return Err(e),
            }
            self.client.request(ADC, bytes)?;
            let [i, q] = self.format;
            let samples: Vec<C32> = raw
                .chunks_exact(4)
                .map(|w| C32::new(i.sample([w[0], w[1]]), q.sample([w[2], w[3]])))
                .collect();
            let n = samples.len() as u64;
            let lost = gaps.arrived(n);
            if lost > 0 {
                self.dropped.fetch_add(lost, Ordering::Relaxed);
                tracing::warn!(
                    "pluto: {lost} samples ({:.0} ms) never arrived",
                    lost as f64 * 1000.0 / self.rate.as_f64()
                );
            }
            let buf = IqBuf::new(samples, self.center, self.rate, gaps.counted() - n);
            match self.tx.try_send(buf) {
                Ok(()) => {}
                Err(TrySendError::Full(buf)) => {
                    self.dropped.fetch_add(buf.len() as u64, Ordering::Relaxed);
                }
                Err(TrySendError::Disconnected(_)) => break,
            }
        }
        Ok(())
    }
}

struct Stream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    sock: Option<std::net::TcpStream>,
    join: Option<common::thread::JoinHandle<()>>,
}

impl RxStream for Stream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(s) = &self.sock {
            let _ = s.shutdown(std::net::Shutdown::Both);
        }
    }
}

impl Drop for Stream {
    fn drop(&mut self) {
        self.stop();
        while self.rx.try_recv().is_ok() {}
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

struct Transmit {
    client: Client,
    bytes: usize,
    pending: Vec<u8>,
    format: [Format; 2],
    rate: Sps,
    stopped: bool,
    transmitting: Arc<AtomicBool>,
}

impl Transmit {
    fn flush(&mut self) -> Result<()> {
        if self.pending.is_empty() {
            return Ok(());
        }
        let out = std::mem::take(&mut self.pending);
        let sent = self.client.send_buffer(DAC, &out);
        self.pending = out;
        self.pending.clear();
        sent
    }
}

impl TxStream for Transmit {
    fn write(&mut self, buf: &IqBuf) -> Result<()> {
        if self.stopped {
            return Err(Error::Disconnected);
        }
        if buf.rate != self.rate {
            return Err(Error::RateUnsupported { req: buf.rate });
        }
        let [i, q] = self.format;
        for s in &buf.samples {
            self.pending.extend(i.word(s.re));
            self.pending.extend(q.word(s.im));
            if self.pending.len() >= self.bytes {
                self.flush()?;
            }
        }
        Ok(())
    }

    fn underruns(&self) -> u64 {
        0
    }

    fn drain(&mut self, _timeout: Duration) -> bool {
        self.flush().is_ok()
    }

    fn stop(&mut self) {
        if self.stopped {
            return;
        }
        self.stopped = true;
        let _ = self.client.close(DAC);
        let _ = self.client.write(tx_lo("powerdown"), "1");
        self.client.shutdown();
        self.transmitting.store(false, Ordering::SeqCst);
    }
}

impl Drop for Transmit {
    fn drop(&mut self) {
        if !self.stopped {
            let _ = self.flush();
        }
        self.stop();
    }
}

pub(crate) struct Remote;

impl crate::Protocol for Remote {
    fn proto(&self) -> Proto {
        Proto::Pluto
    }

    fn probe(&self, addr: &str) -> Result<Probe> {
        probe(addr)
    }

    fn open(&self, addr: &str) -> Result<Box<dyn common::Device>> {
        Ok(Box::new(Pluto::open(addr, common::device::DriverKind::Network)?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::iiod::tests::{Heard, PLUTO_XML, fake};

    fn pluto() -> (String, Heard) {
        fake(
            PLUTO_XML,
            &[
                ("ad9361-phy OUTPUT altvoltage0 frequency_available", "[325000000 1 3800000000]"),
                ("ad9361-phy OUTPUT altvoltage1 frequency_available", "[46875001 1 6000000000]"),
                ("ad9361-phy INPUT voltage0 hardwaregain_available", "[-3 1 71]"),
                ("ad9361-phy xo_correction", "40000159"),
            ],
        )
    }

    fn written(heard: &Heard) -> Vec<String> {
        heard.lock().unwrap().iter().filter(|l| l.contains(" = ")).cloned().collect()
    }

    #[test]
    fn a_probe_names_the_board_and_its_tuning_range() {
        let (addr, _) = pluto();
        let p = probe(&addr).unwrap();
        assert_eq!(p.proto, Proto::Pluto);
        assert_eq!(p.name, "");
        assert_eq!(p.tuner, "AD9363A");
        assert_eq!(p.tune_range, Some(Hz(325_000_000)..=Hz(3_800_000_000)));
        assert_eq!(p.rate_range, Some(Sps(260_417)..=Sps(4_000_000)));
        assert!(p.tunable);
    }

    #[test]
    fn a_context_with_no_ad936x_is_not_a_pluto() {
        const XADC: &str =
            r#"<context name="local"><device id="iio:device0" name="xadc"></device></context>"#;
        let (addr, _) = fake(XADC, &[]);
        let e = probe(&addr).unwrap_err().to_string();
        assert!(e.contains("no AD936x"), "{e}");
    }

    #[test]
    fn opening_leaves_the_transmitter_off_and_tunes_the_receiver() {
        let (addr, heard) = pluto();
        let d = Pluto::open(&addr, DriverKind::Pluto).unwrap();
        let info = d.info();
        assert_eq!(info.kind, DriverKind::Pluto);
        assert_eq!(info.id, "pluto:1044734c960500111e002e0041984fc267");
        assert_eq!(info.label, "ADALM-PLUTO 984fc267");
        assert_eq!(info.ranges[0].label, "325 MHz - 3.8 GHz");
        assert_eq!(info.rate_range, Sps(260_417)..=Sps(4_000_000));
        assert_eq!(info.gain_stages[0].range, -3.0..=71.0);
        let tx = info.tx.as_ref().unwrap();
        assert!(!tx.half_duplex);
        assert_eq!(tx.ranges[0].label, "47 MHz - 6 GHz");
        assert_eq!(tx.gain_stages[0].range, 0.0..=89.75);
        assert_eq!(d.center(), Hz(433_920_000));
        assert_eq!(
            written(&heard),
            [
                "ad9361-phy OUTPUT voltage0 hardwaregain = -89.75",
                "ad9361-phy OUTPUT altvoltage1 powerdown = 1",
                "ad9361-phy INPUT voltage0 sampling_frequency = 2400000",
                "cf-ad9361-lpc INPUT voltage0 sampling_frequency = 2400000",
                "cf-ad9361-dds-core-lpc OUTPUT voltage0 sampling_frequency = 2400000",
                "ad9361-phy INPUT voltage0 rf_bandwidth = 2400000",
                "ad9361-phy OUTPUT voltage0 rf_bandwidth = 2400000",
                "ad9361-phy OUTPUT altvoltage0 frequency = 433920000",
                "ad9361-phy INPUT voltage0 gain_control_mode = slow_attack",
            ]
        );
    }

    #[test]
    fn a_rate_below_the_chips_floor_runs_the_chip_eight_times_faster() {
        let (addr, heard) = pluto();
        let mut d = Pluto::open(&addr, DriverKind::Network).unwrap();
        heard.lock().unwrap().clear();
        d.set_rate(Sps(1_024_000)).unwrap();
        assert_eq!(
            written(&heard),
            [
                "ad9361-phy INPUT voltage0 sampling_frequency = 8192000",
                "cf-ad9361-lpc INPUT voltage0 sampling_frequency = 1024000",
                "cf-ad9361-dds-core-lpc OUTPUT voltage0 sampling_frequency = 1024000",
                "ad9361-phy INPUT voltage0 rf_bandwidth = 1024000",
                "ad9361-phy OUTPUT voltage0 rf_bandwidth = 1024000",
            ]
        );
        assert_eq!(d.rate(), Sps(1_024_000));
        assert!(matches!(d.set_rate(Sps(250_000)), Err(Error::RateUnsupported { .. })));
        assert!(matches!(d.set_rate(Sps(10_000_000)), Err(Error::RateUnsupported { .. })));
        assert!(matches!(d.set_center(Hz(100_000_000)), Err(Error::FreqOutOfRange { .. })));
    }

    #[test]
    fn gain_agc_and_crystal_go_to_the_phy() {
        let (addr, heard) = pluto();
        let mut d = Pluto::open(&addr, DriverKind::Pluto).unwrap();
        heard.lock().unwrap().clear();
        d.set_gain("rf", GainMode::Manual(40.4)).unwrap();
        d.set_choice("agc", "fast").unwrap();
        d.set_gain("rf", GainMode::Auto).unwrap();
        d.set_ppm(-2.5).unwrap();
        d.set_tx_gain("gain", GainMode::Manual(79.8)).unwrap();
        d.set_tx_center(Hz(5_800_000_000)).unwrap();
        assert!(d.set_gain("lna", GainMode::Auto).is_err());
        assert!(d.set_choice("agc", "hybrid").is_err());
        assert_eq!(
            written(&heard),
            [
                "ad9361-phy INPUT voltage0 gain_control_mode = manual",
                "ad9361-phy INPUT voltage0 hardwaregain = 40",
                "ad9361-phy INPUT voltage0 gain_control_mode = fast_attack",
                "ad9361-phy xo_correction = 40000059",
                "ad9361-phy OUTPUT voltage0 hardwaregain = -10.00",
                "ad9361-phy OUTPUT altvoltage1 frequency = 5800000000",
            ]
        );
        assert_eq!(d.ppm(), -2.5);
        assert_eq!(d.choices()[0].selected, "fast");
        assert_eq!(d.tx_gains(), [("gain".to_string(), GainMode::Manual(79.75))]);
    }

    #[test]
    fn a_stream_reads_twelve_bit_iq_off_the_adc() {
        let (addr, heard) = pluto();
        let mut d = Pluto::open(&addr, DriverKind::Pluto).unwrap();
        d.set_rate(Sps(2_400_000)).unwrap();
        let mut s = d.start_rx().unwrap();
        assert!(matches!(d.start_rx(), Err(Error::Busy)));
        let a = s.read().unwrap();
        let b = s.read().unwrap();
        assert_eq!(a.len(), 48_000);
        assert_eq!(a.rate, Sps(2_400_000));
        assert_eq!(a.center, Hz(433_920_000));
        assert_eq!(b.seq, 48_000);
        assert_eq!(a.samples[0], C32::new(-1.0, -2047.0 / 2048.0));
        assert_eq!(a.samples[1], C32::new(-2046.0 / 2048.0, -2045.0 / 2048.0));
        assert_eq!(s.dropped(), 0);
        s.stop();
        drop(s);
        let heard = heard.lock().unwrap();
        assert!(heard.contains(&"SET cf-ad9361-lpc BUFFERS_COUNT 4".to_string()));
        assert!(heard.contains(&"OPEN cf-ad9361-lpc 48000 00000003".to_string()));
    }

    #[test]
    fn transmit_powers_the_lo_up_for_the_over_and_down_after() {
        let (addr, heard) = pluto();
        let mut d = Pluto::open(&addr, DriverKind::Pluto).unwrap();
        d.set_rate(Sps(2_400_000)).unwrap();
        heard.lock().unwrap().clear();
        let mut t = d.start_tx().unwrap();
        assert!(matches!(d.start_tx(), Err(Error::Busy)));
        let tone: Vec<C32> = (0..60_000).map(|_| C32::new(1.0, -0.5)).collect();
        t.write(&IqBuf::new(tone, Hz(0), Sps(2_400_000), 0)).unwrap();
        assert!(t.write(&IqBuf::new(vec![C32::new(0.0, 0.0)], Hz(0), Sps(1_000_000), 0)).is_err());
        assert!(t.drain(Duration::from_secs(1)));
        t.stop();
        let heard = heard.lock().unwrap().clone();
        let full = 32767i16.to_le_bytes();
        let half = (-16384i16).to_le_bytes();
        let first = [full[0], full[1], half[0], half[1], full[0], full[1], half[0], half[1]];
        assert_eq!(
            heard,
            [
                "ad9361-phy OUTPUT altvoltage1 frequency = 433920000".to_string(),
                "ad9361-phy OUTPUT voltage0 hardwaregain = -89.75".to_string(),
                "ad9361-phy OUTPUT altvoltage1 powerdown = 0".to_string(),
                "SET cf-ad9361-dds-core-lpc BUFFERS_COUNT 4".to_string(),
                "OPEN cf-ad9361-dds-core-lpc 48000 00000003".to_string(),
                format!("WRITEBUF 192000 {first:?}"),
                format!("WRITEBUF 48000 {first:?}"),
                "CLOSE cf-ad9361-dds-core-lpc".to_string(),
                "ad9361-phy OUTPUT altvoltage1 powerdown = 1".to_string(),
            ]
        );
        drop(t);
        assert!(d.start_tx().is_ok(), "the transmitter is free again once the over ends");
    }
}
