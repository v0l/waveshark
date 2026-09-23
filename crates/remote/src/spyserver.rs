use crate::{CONNECT_TIMEOUT, Probe, Proto, QUEUE_DEPTH};
use common::device::{
    Device as DeviceTrait, DeviceInfo, DriverKind, GainMode, Number, RxStream, TunerRange,
};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

pub const DEFAULT_PORT: u16 = 5555;

const PROTOCOL_VERSION: u32 = (2 << 24) | 1700;

const COMMAND_HEADER: usize = 8;
const MESSAGE_HEADER: usize = 20;

const DEVICE_INFO_LEN: usize = 48;

const CLIENT_SYNC_LEN: usize = 36;

const MAX_BODY: usize = 1 << 20;

const READ_TIMEOUT: Duration = Duration::from_secs(5);

const GAIN: &str = "gain";

#[derive(Clone, Copy)]
#[repr(u32)]
enum Cmd {
    Hello = 0,
    SetSetting = 2,
}

#[derive(Clone, Copy)]
#[repr(u32)]
enum Set {
    StreamingMode = 0,
    StreamingEnabled = 1,
    Gain = 2,
    IqFormat = 100,
    IqFrequency = 101,
    IqDecimation = 102,
    IqDigitalGain = 103,
}

const STREAM_MODE_IQ: u32 = 1;
const FORMAT_UINT8: u32 = 1;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DeviceType {
    #[default]
    Invalid,
    AirspyOne,
    AirspyHf,
    RtlSdr,
    Other(u32),
}

impl DeviceType {
    fn from_code(code: u32) -> Self {
        match code {
            0 => Self::Invalid,
            1 => Self::AirspyOne,
            2 => Self::AirspyHf,
            3 => Self::RtlSdr,
            other => Self::Other(other),
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Invalid => "no device",
            Self::AirspyOne => "Airspy",
            Self::AirspyHf => "Airspy HF+",
            Self::RtlSdr => "RTL-SDR",
            Self::Other(_) => "unknown",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Msg {
    DeviceInfo,
    ClientSync,
    Pong,
    ReadSetting,
    Uint8Iq,
    Other(u16),
}

impl Msg {
    fn from_code(code: u16) -> Self {
        match code {
            0 => Self::DeviceInfo,
            1 => Self::ClientSync,
            2 => Self::Pong,
            3 => Self::ReadSetting,
            100 => Self::Uint8Iq,
            other => Self::Other(other),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Header {
    kind: Msg,
    gain_db: u16,
    seq: u32,
    body: usize,
}

impl Header {
    fn parse(buf: &[u8; MESSAGE_HEADER]) -> Result<Self> {
        let word = |at: usize| u32::from_le_bytes(buf[at..at + 4].try_into().unwrap());
        let kind = word(4);
        let body = word(16) as usize;
        if body > MAX_BODY {
            return Err(Error::other(format!("spyserver: {body} byte message")));
        }
        Ok(Header {
            kind: Msg::from_code(kind as u16),
            gain_db: (kind >> 16) as u16,
            seq: word(12),
            body,
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Info {
    pub device: DeviceType,
    pub serial: u32,
    pub max_rate: u32,
    pub max_bandwidth: u32,
    pub decimation_stages: u32,
    pub max_gain_index: u32,
    pub min_hz: u32,
    pub max_hz: u32,
    pub resolution: u32,
    pub min_decimation: u32,
}

impl Info {
    fn parse(body: &[u8]) -> Result<Self> {
        if body.len() < DEVICE_INFO_LEN {
            return Err(Error::other(format!("spyserver: {} byte device info", body.len())));
        }
        let word = |i: usize| u32::from_le_bytes(body[i * 4..i * 4 + 4].try_into().unwrap());
        Ok(Info {
            device: DeviceType::from_code(word(0)),
            serial: word(1),
            max_rate: word(2),
            max_bandwidth: word(3),
            decimation_stages: word(4),
            max_gain_index: word(6),
            min_hz: word(7),
            max_hz: word(8),
            resolution: word(9),
            min_decimation: word(10),
        })
    }

    pub fn rates(&self) -> Vec<Sps> {
        let mut out: Vec<Sps> = (self.min_decimation..=self.decimation_stages)
            .map(|i| Sps((self.max_rate >> i.min(31)) as u64))
            .filter(|r| r.0 > 0)
            .collect();
        out.reverse();
        out
    }

    fn stage_rate(&self, decimation: u32) -> u64 {
        (self.max_rate >> decimation.min(31)) as u64
    }

    fn decimation(&self, rate: Sps) -> Option<u32> {
        let stages =
            || (self.min_decimation..=self.decimation_stages).filter(|i| self.stage_rate(*i) > 0);
        stages().filter(|i| self.stage_rate(*i) >= rate.0).max().or_else(|| stages().min())
    }

    fn digital_gain(&self, gain_index: u32, decimation: u32) -> u32 {
        let stages = (decimation as f32 * 3.01) as u32;
        match self.device {
            DeviceType::AirspyOne => self.max_gain_index.saturating_sub(gain_index) + stages,
            _ => stages,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Sync {
    pub can_control: bool,
    pub gain: u32,
    pub device_center: u32,
    pub iq_center: u32,
    pub min_iq_center: u32,
    pub max_iq_center: u32,
}

impl Sync {
    fn parse(body: &[u8]) -> Result<Self> {
        if body.len() < CLIENT_SYNC_LEN {
            return Err(Error::other(format!("spyserver: {} byte sync", body.len())));
        }
        let word = |i: usize| u32::from_le_bytes(body[i * 4..i * 4 + 4].try_into().unwrap());
        Ok(Sync {
            can_control: word(0) != 0,
            gain: word(1),
            device_center: word(2),
            iq_center: word(3),
            min_iq_center: word(5),
            max_iq_center: word(6),
        })
    }
}

#[derive(Default)]
struct Landed {
    center: AtomicU64,
    min_center: AtomicU32,
    max_center: AtomicU32,
    can_control: AtomicBool,
}

impl Landed {
    fn take(&self, s: &Sync) {
        self.center.store(s.iq_center as u64, Ordering::Relaxed);
        self.min_center.store(s.min_iq_center, Ordering::Relaxed);
        self.max_center.store(s.max_iq_center, Ordering::Relaxed);
        self.can_control.store(s.can_control, Ordering::Relaxed);
    }

    fn reach(&self, info: &Info) -> std::ops::RangeInclusive<Hz> {
        match self.can_control.load(Ordering::Relaxed) {
            true => Hz(info.min_hz as u64)..=Hz(info.max_hz as u64),
            false => {
                let lo = self.min_center.load(Ordering::Relaxed) as u64;
                let hi = self.max_center.load(Ordering::Relaxed) as u64;
                Hz(lo)..=Hz(hi.max(lo))
            }
        }
    }
}

fn other(e: impl std::fmt::Display) -> Error {
    Error::other(format!("spyserver: {e}"))
}

fn fill(sock: &mut TcpStream, buf: &mut [u8], until: Instant) -> Result<()> {
    let mut at = 0;
    while at < buf.len() {
        if Instant::now() >= until {
            return Err(Error::other("spyserver: the server stopped sending"));
        }
        match sock.read(&mut buf[at..]) {
            Ok(0) => return Err(Error::Disconnected),
            Ok(n) => at += n,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
            Err(e) => return Err(other(e)),
        }
    }
    Ok(())
}

fn message(sock: &mut TcpStream, body: &mut Vec<u8>, until: Instant) -> Result<Header> {
    let mut head = [0u8; MESSAGE_HEADER];
    fill(sock, &mut head, until)?;
    let header = Header::parse(&head)?;
    body.clear();
    body.resize(header.body, 0);
    fill(sock, body, until)?;
    Ok(header)
}

fn command(sock: &mut TcpStream, cmd: Cmd, body: &[u8]) -> Result<()> {
    let mut out = Vec::with_capacity(COMMAND_HEADER + body.len());
    out.extend_from_slice(&(cmd as u32).to_le_bytes());
    out.extend_from_slice(&(body.len() as u32).to_le_bytes());
    out.extend_from_slice(body);
    sock.write_all(&out).map_err(other)
}

fn setting(sock: &mut TcpStream, which: Set, value: u32) -> Result<()> {
    let mut body = [0u8; 8];
    body[..4].copy_from_slice(&(which as u32).to_le_bytes());
    body[4..].copy_from_slice(&value.to_le_bytes());
    command(sock, Cmd::SetSetting, &body)
}

fn connect(addr: &str) -> Result<(TcpStream, Info, Sync)> {
    let resolved = addr
        .to_socket_addrs()
        .map_err(|e| Error::other(format!("{addr}: {e}")))?
        .next()
        .ok_or_else(|| Error::other(format!("{addr} resolves to nothing")))?;
    let mut sock = TcpStream::connect_timeout(&resolved, CONNECT_TIMEOUT)
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;
    sock.set_read_timeout(Some(CONNECT_TIMEOUT)).map_err(other)?;
    let _ = sock.set_nodelay(true);

    let mut hello = Vec::new();
    hello.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    hello.extend_from_slice(b"waveshark");
    command(&mut sock, Cmd::Hello, &hello)?;

    let until = Instant::now() + CONNECT_TIMEOUT * 2;
    let mut body = Vec::new();
    let (mut info, mut sync) = (None, None);
    while info.is_none() || sync.is_none() {
        let head = message(&mut sock, &mut body, until)?;
        match head.kind {
            Msg::DeviceInfo => info = Some(Info::parse(&body)?),
            Msg::ClientSync => sync = Some(Sync::parse(&body)?),
            _ => continue,
        }
    }
    let (info, sync) = (info.unwrap_or_default(), sync.unwrap_or_default());
    if info.device == DeviceType::Invalid || info.max_rate == 0 {
        return Err(Error::other(format!("{addr} has no radio on it")));
    }
    Ok((sock, info, sync))
}

pub fn probe(addr: &str) -> Result<Probe> {
    let addr = Proto::SpyServer.parse_addr(addr).ok_or(Error::NoDevice)?;
    let (sock, info, sync) = connect(&addr)?;
    drop(sock);
    tracing::debug!(
        "spyserver {addr}: {} at {} MS/s, {} decimation stages",
        info.device.name(),
        info.max_rate,
        info.decimation_stages
    );
    let landed = Landed::default();
    landed.take(&sync);
    let reach = landed.reach(&info);
    Ok(Probe {
        proto: Proto::SpyServer,
        addr,
        center: Some(Hz(sync.iq_center as u64)),
        rate: None,
        rates: info.rates(),
        gain_db: None,
        name: match sync.can_control {
            true => info.device.name().to_string(),
            false => format!("{} (shared)", info.device.name()),
        },
        settings: Vec::new(),
        tunable: sync.can_control,
        tune_range: Some(reach),
        tuner: info.device.name().to_string(),
    })
}

pub struct Device {
    addr: String,
    info: DeviceInfo,
    server: Info,
    tuning: common::Tuning,
    sock: TcpStream,
    rate: Sps,
    gain_index: u32,
    streaming: Arc<AtomicBool>,
    landed: Arc<Landed>,
}

impl Device {
    pub fn open(addr: &str) -> Result<Self> {
        let addr = Proto::SpyServer.parse_addr(addr).ok_or(Error::NoDevice)?;
        let (sock, server, sync) = connect(&addr)?;
        let landed = Arc::new(Landed::default());
        landed.take(&sync);
        let rates = server.rates();
        let rate = *rates.last().unwrap_or(&Sps(server.max_rate as u64));
        let reach = landed.reach(&server);
        let info = DeviceInfo {
            kind: DriverKind::Network,
            id: format!("spyserver:{addr}"),
            label: format!("SpyServer {addr} ({})", server.device.name()),
            tuner: server.device.name().to_string(),
            ranges: vec![TunerRange { range: reach.clone(), label: "rx" }],
            rate_range: *rates.first().unwrap_or(&rate)..=rate,
            rates,
            gain_stages: Vec::new(),
            native_format: SampleFormat::Cu8,
            usable_bandwidth_ratio: (server.max_bandwidth as f32 / server.max_rate.max(1) as f32)
                .clamp(0.1, 1.0),
            tunable: sync.can_control,
            tx: None,
        };
        tracing::debug!(
            "spyserver {addr}: {} serial {:08X}, {} bit",
            server.device.name(),
            server.serial,
            server.resolution
        );
        Ok(Self {
            addr,
            info,
            server,
            tuning: Default::default(),
            sock,
            rate,
            gain_index: sync.gain.min(server.max_gain_index),
            streaming: Arc::new(AtomicBool::new(false)),
            landed,
        })
    }

    pub fn address(&self) -> &str {
        &self.addr
    }

    pub fn server(&self) -> &Info {
        &self.server
    }

    fn tell_gain(&mut self) -> Result<()> {
        let decimation = self.server.decimation(self.rate).unwrap_or(0);
        let digital = self.server.digital_gain(self.gain_index, decimation);
        setting(&mut self.sock, Set::Gain, self.gain_index)?;
        setting(&mut self.sock, Set::IqDigitalGain, digital)
    }

    fn refresh_reach(&mut self) {
        let reach = self.landed.reach(&self.server);
        self.info.tunable = self.landed.can_control.load(Ordering::Relaxed);
        self.info.ranges = vec![TunerRange { range: reach, label: "rx" }];
    }
}

impl DeviceTrait for Device {
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
        setting(&mut self.sock, Set::IqFrequency, f.0 as u32)?;
        self.landed.center.store(f.0, Ordering::Relaxed);
        self.refresh_reach();
        Ok(())
    }

    fn center(&self) -> Hz {
        Hz(self.landed.center.load(Ordering::Relaxed))
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        let decimation = self.server.decimation(r).ok_or(Error::RateUnsupported { req: r })?;
        setting(&mut self.sock, Set::IqDecimation, decimation)?;
        self.rate = Sps(self.server.stage_rate(decimation));
        self.tell_gain()?;
        self.refresh_reach();
        Ok(())
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn rate_needs_restart(&self) -> bool {
        true
    }

    fn set_gain(&mut self, _stage: &str, _mode: GainMode) -> Result<()> {
        Ok(())
    }

    fn numbers(&self) -> Vec<Number> {
        if self.server.max_gain_index == 0 {
            return Vec::new();
        }
        vec![Number {
            name: GAIN.into(),
            label: "Gain step".into(),
            help: format!(
                "The far end's own gain index, 0 to {}, which is not decibels: \
                 what each step is worth is the {}'s business.",
                self.server.max_gain_index,
                self.server.device.name()
            ),
            range: 0.0..=self.server.max_gain_index as f64,
            step: 1.0,
            unit: "step".into(),
            value: self.gain_index as f64,
        }]
    }

    fn set_number(&mut self, name: &str, value: f64) -> Result<()> {
        if name != GAIN {
            return Err(Error::other(format!("no setting named {name:?}")));
        }
        self.gain_index = (value.round().max(0.0) as u32).min(self.server.max_gain_index);
        self.tell_gain()
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let decimation = self.server.decimation(self.rate).unwrap_or(0);
        let center = self.center().0 as u32;
        let start = (|| -> Result<()> {
            setting(&mut self.sock, Set::IqFormat, FORMAT_UINT8)?;
            setting(&mut self.sock, Set::IqDecimation, decimation)?;
            setting(&mut self.sock, Set::IqFrequency, center)?;
            setting(&mut self.sock, Set::StreamingMode, STREAM_MODE_IQ)?;
            self.tell_gain()?;
            setting(&mut self.sock, Set::StreamingEnabled, 1)
        })();
        if let Err(e) = start {
            self.streaming.store(false, Ordering::SeqCst);
            return Err(e);
        }

        let sock = self.sock.try_clone().map_err(other)?;
        sock.set_read_timeout(Some(READ_TIMEOUT)).map_err(other)?;

        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));
        let rate = self.rate;
        let landed = self.landed.clone();
        let streaming = self.streaming.clone();
        let counted = dropped.clone();
        let halt = stop.clone();
        let addr = self.addr.clone();
        let join = std::thread::Builder::new()
            .name("spyserver-rx".into())
            .spawn(move || {
                if let Err(e) = pump(sock, rate, landed, tx, counted, halt) {
                    tracing::warn!("spyserver {addr}: {e}");
                }
                streaming.store(false, Ordering::SeqCst);
            })
            .map_err(|e| Error::other(format!("spawn rx thread: {e}")))?;

        Ok(Box::new(NetStream {
            rx,
            dropped,
            stop,
            sock: self.sock.try_clone().ok(),
            join: Some(join),
        }))
    }
}

fn pump(
    mut sock: TcpStream,
    rate: Sps,
    landed: Arc<Landed>,
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    let mut body = Vec::new();
    let mut samples = Vec::new();
    let mut counted: u64 = 0;
    let mut last_seq: Option<u32> = None;
    while !stop.load(Ordering::Relaxed) {
        let until = Instant::now() + READ_TIMEOUT * 4;
        let head = match message(&mut sock, &mut body, until) {
            Ok(h) => h,
            Err(Error::Disconnected) => return Ok(()),
            Err(e) => return Err(e),
        };
        match head.kind {
            Msg::ClientSync => {
                if let Ok(s) = Sync::parse(&body) {
                    landed.take(&s);
                }
                continue;
            }
            Msg::Uint8Iq => {}
            _ => continue,
        }
        samples.clear();
        SampleFormat::Cu8.convert(&body, &mut samples);
        if head.gain_db > 0 {
            let scale = 10f32.powf(-(head.gain_db as f32) / 20.0);
            for s in &mut samples {
                *s *= scale;
            }
        }
        let n = samples.len() as u64;
        if let Some(prev) = last_seq {
            let missed = head.seq.wrapping_sub(prev).saturating_sub(1) as u64;
            if missed > 0 {
                let lost = missed * n;
                counted += lost;
                dropped.fetch_add(lost, Ordering::Relaxed);
                tracing::warn!("spyserver: {missed} blocks ({lost} samples) never arrived");
            }
        }
        last_seq = Some(head.seq);
        let center = Hz(landed.center.load(Ordering::Relaxed));
        let buf = IqBuf::new(std::mem::take(&mut samples), center, rate, counted);
        counted += n;
        samples = Vec::with_capacity(n as usize);
        match tx.try_send(buf) {
            Ok(()) => {}
            Err(TrySendError::Full(buf)) => {
                dropped.fetch_add(buf.len() as u64, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => break,
        }
    }
    Ok(())
}

struct NetStream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
    sock: Option<TcpStream>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl RxStream for NetStream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(s) = &self.sock {
            if let Ok(mut w) = s.try_clone() {
                let _ = setting(&mut w, Set::StreamingEnabled, 0);
            }
            let _ = s.shutdown(std::net::Shutdown::Read);
        }
    }
}

impl Drop for NetStream {
    fn drop(&mut self) {
        self.stop();
        while self.rx.try_recv().is_ok() {}
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    fn airspy_info() -> Vec<u8> {
        words(&[
            1,
            0x35484D63,
            3_000_000,
            2_400_000,
            10,
            0,
            21,
            24_000_000,
            1_800_000_000,
            12,
            0,
            0,
        ])
    }

    fn words(v: &[u32]) -> Vec<u8> {
        v.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    fn sync_body(can_control: u32, center: u32, lo: u32, hi: u32) -> Vec<u8> {
        let mut b = words(&[can_control, 0, center, center, center, lo, hi, center, center]);
        b.extend_from_slice(&[0, 0, 0, 0]);
        b
    }

    fn message_bytes(kind: u32, seq: u32, body: &[u8]) -> Vec<u8> {
        let mut out = words(&[(2 << 24) | 1921, kind, 0, seq, body.len() as u32]);
        out.extend_from_slice(body);
        out
    }

    fn fake(
        info: Vec<u8>,
        can_control: u32,
        blocks: u32,
        skip: Option<u32>,
    ) -> (String, std::sync::mpsc::Receiver<(u32, u32)>) {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let (mut sock, _) = l.accept().unwrap();
            let mut head = [0u8; COMMAND_HEADER];
            sock.read_exact(&mut head).unwrap();
            let len = u32::from_le_bytes(head[4..8].try_into().unwrap()) as usize;
            let mut hello = vec![0u8; len];
            sock.read_exact(&mut hello).unwrap();
            let _ = tx.send((
                u32::from_le_bytes(head[..4].try_into().unwrap()),
                u32::from_le_bytes(hello[..4].try_into().unwrap()),
            ));
            sock.write_all(&message_bytes(0, 0, &info)).unwrap();
            sock.write_all(&message_bytes(
                1,
                0,
                &sync_body(can_control, 95_500_000, 94_300_000, 96_700_000),
            ))
            .unwrap();

            let mut writer = sock.try_clone().unwrap();
            let mut body = [0u8; 8];
            while sock.read_exact(&mut body).is_ok() {
                let cmd = u32::from_le_bytes(body[..4].try_into().unwrap());
                let len = u32::from_le_bytes(body[4..8].try_into().unwrap()) as usize;
                let mut rest = vec![0u8; len];
                if sock.read_exact(&mut rest).is_err() {
                    break;
                }
                if cmd != Cmd::SetSetting as u32 {
                    continue;
                }
                let which = u32::from_le_bytes(rest[..4].try_into().unwrap());
                let value = u32::from_le_bytes(rest[4..8].try_into().unwrap());
                if tx.send((which, value)).is_err() {
                    break;
                }
                if which == Set::IqFrequency as u32 {
                    let _ = writer.write_all(&message_bytes(
                        1,
                        0,
                        &sync_body(can_control, value, value - 100_000, value + 100_000),
                    ));
                }
                if which == Set::StreamingEnabled as u32 && value == 1 {
                    let block: Vec<u8> = (0..16u8).map(|i| 120 + i).collect();
                    for seq in 0..blocks {
                        if Some(seq) == skip {
                            continue;
                        }
                        let kind = 100 | (6 << 16);
                        if writer.write_all(&message_bytes(kind, seq, &block)).is_err() {
                            return;
                        }
                    }
                }
            }
        });
        (addr, rx)
    }

    #[test]
    fn a_probe_reads_the_device_and_the_window_it_is_offering() {
        let (addr, _cmds) = fake(airspy_info(), 0, 0, None);
        let p = probe(&addr).unwrap();
        assert_eq!(p.proto, Proto::SpyServer);
        assert_eq!(p.tuner, "Airspy");
        assert_eq!(p.name, "Airspy (shared)");
        assert_eq!(p.center, Some(Hz(95_500_000)));
        assert_eq!(p.rate, None, "the rate is ours out of the stages offered");
        assert!(
            !p.tunable,
            "a server granting no control is a window to look through, not a dial: \
             it closes altogether once the whole of it is being streamed"
        );
        assert_eq!(p.tune_range, Some(Hz(94_300_000)..=Hz(96_700_000)));
    }

    #[test]
    fn a_server_that_grants_control_offers_the_whole_radio() {
        let (addr, _cmds) = fake(airspy_info(), 1, 0, None);
        let p = probe(&addr).unwrap();
        assert_eq!(p.name, "Airspy");
        assert!(p.tunable);
        assert_eq!(p.tune_range, Some(Hz(24_000_000)..=Hz(1_800_000_000)));
    }

    #[test]
    fn a_probe_offers_no_span_wider_than_the_server_streams() {
        let mut info = airspy_info();
        info[40..44].copy_from_slice(&2u32.to_le_bytes());
        let (addr, _cmds) = fake(info, 1, 0, None);
        let p = probe(&addr).unwrap();
        assert_eq!(p.rates.len(), 9, "stages two to ten");
        assert_eq!(p.rates.first(), Some(&Sps(2929)));
        assert_eq!(p.rates.last(), Some(&Sps(750_000)), "3 MS/s decimated twice");
    }

    #[test]
    fn the_rates_are_the_stages_the_device_offers() {
        let info = Info::parse(&airspy_info()).unwrap();
        assert_eq!(info.device, DeviceType::AirspyOne);
        assert_eq!(info.serial, 0x35484D63);
        assert_eq!(info.max_gain_index, 21);
        assert_eq!(info.resolution, 12);
        let rates = info.rates();
        assert_eq!(rates.len(), 11, "eleven stages, zero to ten");
        assert_eq!(rates.first(), Some(&Sps(2929)));
        assert_eq!(rates.last(), Some(&Sps(3_000_000)));
        assert_eq!(info.decimation(Sps(3_000_000)), Some(0));
        assert_eq!(info.decimation(Sps(375_000)), Some(3));
        assert_eq!(
            info.decimation(Sps(2_400_000)),
            Some(0),
            "a span is never quietly narrowed: a rate no stage has takes the next one up"
        );
        assert_eq!(info.decimation(Sps(1_000_000)), Some(1), "1.5 MS/s, not 750 kS/s");
        assert_eq!(info.decimation(Sps(1)), Some(10), "the slowest stage there is");
        assert_eq!(info.decimation(Sps(9_000_000)), Some(0), "and nothing goes faster than it");
        assert_eq!(info.digital_gain(0, 0), 21);
        assert_eq!(info.digital_gain(21, 3), 9);
        let rtl = Info { device: DeviceType::RtlSdr, ..info };
        assert_eq!(rtl.digital_gain(0, 0), 0);
        assert_eq!(rtl.digital_gain(0, 3), 9);
    }

    #[test]
    fn a_sync_longer_than_the_published_structure_still_reads() {
        let s = Sync::parse(&sync_body(1, 95_500_000, 94_300_000, 96_700_000)).unwrap();
        assert!(s.can_control);
        assert_eq!(s.iq_center, 95_500_000);
        assert_eq!(s.min_iq_center, 94_300_000);
        assert_eq!(s.max_iq_center, 96_700_000);
        let short = Sync::parse(&[0u8; 20]).unwrap_err().to_string();
        assert!(short.contains("20 byte sync"), "{short}");
    }

    #[test]
    fn opening_reads_the_device_and_a_retune_is_one_setting() {
        let (addr, cmds) = fake(airspy_info(), 1, 0, None);
        let mut d = Device::open(&addr).unwrap();
        let (cmd, version) = cmds.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(cmd, Cmd::Hello as u32);
        assert_eq!(version, (2 << 24) | 1700, "2.0.1700, what every client announces");

        assert_eq!(d.info().tuner, "Airspy");
        assert_eq!(d.info().rates.len(), 11);
        assert_eq!(d.rate(), Sps(3_000_000), "the fastest stage until told otherwise");
        assert_eq!(d.info().usable_bandwidth_ratio, 0.8);

        d.set_center(Hz::mhz(145)).unwrap();
        assert_eq!(cmds.recv_timeout(Duration::from_secs(2)).unwrap(), (101, 145_000_000));
        assert_eq!(d.center(), Hz::mhz(145));

        d.set_rate(Sps(375_000)).unwrap();
        assert_eq!(cmds.recv_timeout(Duration::from_secs(2)).unwrap(), (102, 3));
        assert_eq!(cmds.recv_timeout(Duration::from_secs(2)).unwrap(), (2, 0), "gain index");
        assert_eq!(
            cmds.recv_timeout(Duration::from_secs(2)).unwrap(),
            (103, 30),
            "21 dB of unused Airspy gain and 3 dB a decimation stage"
        );
        d.set_rate(Sps(2_304_000)).unwrap();
        assert_eq!(cmds.recv_timeout(Duration::from_secs(2)).unwrap(), (102, 0));
        assert_eq!(d.rate(), Sps(3_000_000), "a span the receiver offers is not a stage here");
        assert!(d.rate_needs_restart());
    }

    #[test]
    fn the_gain_is_offered_as_the_index_it_is() {
        let (addr, cmds) = fake(airspy_info(), 1, 0, None);
        let mut d = Device::open(&addr).unwrap();
        let _ = cmds.recv_timeout(Duration::from_secs(2));
        let n = d.numbers();
        assert_eq!(n.len(), 1);
        assert_eq!(n[0].name, "gain");
        assert_eq!(*n[0].range.end(), 21.0);
        assert_eq!(n[0].step, 1.0);
        assert_eq!(n[0].unit, "step");
        assert_eq!(n[0].value, 0.0);

        d.set_number("gain", 14.0).unwrap();
        assert_eq!(cmds.recv_timeout(Duration::from_secs(2)).unwrap(), (2, 14));
        assert_eq!(cmds.recv_timeout(Duration::from_secs(2)).unwrap(), (103, 7), "21 - 14");
        assert_eq!(d.numbers()[0].value, 14.0);
        d.set_number("gain", 99.0).unwrap();
        assert_eq!(cmds.recv_timeout(Duration::from_secs(2)).unwrap(), (2, 21));
        assert!(d.set_number("bias_tee", 1.0).is_err());
        d.set_gain("tuner", GainMode::Manual(20.0)).unwrap();
        assert_eq!(
            d.numbers()[0].value,
            21.0,
            "a gain in dB is not carried by this protocol, and the receiver sets one on \
             every start, so it is taken and ignored rather than stopping the radio"
        );

        let mut hf = airspy_info();
        hf[24..28].copy_from_slice(&0u32.to_le_bytes());
        hf[..4].copy_from_slice(&2u32.to_le_bytes());
        let (addr, _cmds) = fake(hf, 1, 0, None);
        let d = Device::open(&addr).unwrap();
        assert_eq!(d.info().tuner, "Airspy HF+");
        assert!(d.numbers().is_empty());
    }

    #[test]
    fn a_stream_asks_for_bytes_and_delivers_what_arrives() {
        let (addr, cmds) = fake(airspy_info(), 1, 4, None);
        let mut d = Device::open(&addr).unwrap();
        let _ = cmds.recv_timeout(Duration::from_secs(2));
        d.set_center(Hz::mhz(145)).unwrap();
        let mut s = d.start_rx().unwrap();

        let mut sent = Vec::new();
        while sent.len() < 8 {
            sent.push(cmds.recv_timeout(Duration::from_secs(2)).unwrap());
        }
        assert_eq!(sent[0], (101, 145_000_000), "the retune before the stream");
        assert_eq!(sent[1], (100, 1), "unsigned bytes");
        assert_eq!(sent[2], (102, 0));
        assert_eq!(sent[3], (101, 145_000_000));
        assert_eq!(sent[4], (0, 1), "samples only, no spectrum");
        assert_eq!(sent[5], (2, 0));
        assert_eq!(sent[6], (103, 21));
        assert_eq!(sent[7], (1, 1), "streaming on");

        let b = s.read().unwrap();
        assert_eq!(b.len(), 8);
        assert_eq!(b.rate, Sps(3_000_000));
        assert_eq!(b.center, Hz::mhz(145));
        assert_eq!(b.seq, 0);
        let scale = 10f32.powf(-6.0 / 20.0);
        assert!((b.samples[0].re - (120.0 - 127.5) / 127.5 * scale).abs() < 1e-6);
        assert!((b.samples[7].im - (135.0 - 127.5) / 127.5 * scale).abs() < 1e-6);
        let second = s.read().unwrap();
        assert_eq!(second.seq, 8, "the numbering counts samples, not blocks");
        assert_eq!(s.dropped(), 0);
        s.stop();
    }

    #[test]
    fn a_block_the_server_dropped_is_counted_as_lost_samples() {
        let (addr, cmds) = fake(airspy_info(), 1, 4, Some(2));
        let mut d = Device::open(&addr).unwrap();
        let _ = cmds.recv_timeout(Duration::from_secs(2));
        let mut s = d.start_rx().unwrap();
        let seqs: Vec<u64> = (0..3).map(|_| s.read().unwrap().seq).collect();
        assert_eq!(seqs, vec![0, 8, 24], "the missing block's eight samples are counted");
        assert_eq!(s.dropped(), 8);
        s.stop();
    }

    #[test]
    fn a_server_with_no_radio_on_it_is_refused() {
        let (addr, _cmds) = fake(words(&[0; 12]), 0, 0, None);
        let e = probe(&addr).unwrap_err().to_string();
        assert!(e.contains("no radio on it"), "{e}");
    }

    #[test]
    fn a_second_reader_is_refused_while_one_is_running() {
        let (addr, _cmds) = fake(airspy_info(), 1, 1000, None);
        let mut d = Device::open(&addr).unwrap();
        let _s = d.start_rx().unwrap();
        assert!(matches!(d.start_rx(), Err(Error::Busy)));
    }
}
