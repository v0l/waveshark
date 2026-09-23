use crate::{CONNECT_TIMEOUT, Probe, Proto, QUEUE_DEPTH};
use common::device::{
    Device as DeviceTrait, DeviceInfo, DriverKind, GainMode, GainStage, RxStream, TunerRange,
};
use common::{C32, Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{Receiver, Sender, TryRecvError, TrySendError, bounded, unbounded};
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::ops::RangeInclusive;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant};
use tungstenite::client::IntoClientRequest;
use tungstenite::{Message, WebSocket};

pub const DEFAULT_PORT: u16 = 8073;

const TUNER: &str = "KiwiSDR";

const GAIN: &str = "tuner";

const MAX_GAIN_DB: f32 = 120.0;

const USABLE_RATIO: f32 = 0.85;

const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(10);

const POLL: Duration = Duration::from_millis(50);

const KEEPALIVE: Duration = Duration::from_secs(1);

const MAX_STATUS: usize = 64 * 1024;

const RATE_TOLERANCE: f64 = 0.01;

const FLAG_STEREO: u8 = 0x08;
const FLAG_COMPRESSED: u8 = 0x10;

const SND_HEADER: usize = 7;
const GPS_HEADER: usize = 10;

const NARROW_RATE: Sps = Sps(12_000);
const WIDE_RATE: Sps = Sps(20_250);

type Ws = WebSocket<TcpStream>;

#[derive(Clone, Debug, PartialEq)]
pub struct Status {
    pub name: String,
    pub bands: RangeInclusive<Hz>,
    pub users: u32,
    pub users_max: u32,
    pub offline: bool,
    pub rate: Sps,
}

impl Status {
    pub fn parse(text: &str) -> Result<Self> {
        let mut fields = std::collections::HashMap::new();
        for line in text.lines() {
            if let Some((k, v)) = line.split_once('=') {
                fields.insert(k.trim(), v.trim());
            }
        }
        if !fields.contains_key("status") {
            return Err(Error::other("not a KiwiSDR status page"));
        }
        let number = |k: &str| fields.get(k).and_then(|v| v.parse::<u32>().ok()).unwrap_or(0);
        let bands = fields
            .get("bands")
            .and_then(|v| v.split_once('-'))
            .and_then(|(lo, hi)| Some(Hz(lo.parse().ok()?)..=Hz(hi.parse().ok()?)))
            .unwrap_or(Hz(0)..=Hz::mhz(30));
        let rate = match fields.get("mode").is_some_and(|m| m.starts_with("rx3")) {
            true => WIDE_RATE,
            false => NARROW_RATE,
        };
        Ok(Self {
            name: fields.get("name").map(|s| s.to_string()).unwrap_or_default(),
            bands,
            users: number("users"),
            users_max: number("users_max"),
            offline: fields.get("offline").is_some_and(|v| *v == "yes"),
            rate,
        })
    }
}

fn dial(addr: &str) -> Result<(TcpStream, Duration)> {
    let resolved = addr
        .to_socket_addrs()
        .map_err(|e| Error::other(format!("{addr}: {e}")))?
        .next()
        .ok_or_else(|| Error::other(format!("{addr} resolves to nothing")))?;
    let started = Instant::now();
    let sock = TcpStream::connect_timeout(&resolved, CONNECT_TIMEOUT)
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;
    let rtt = started.elapsed();
    sock.set_read_timeout(Some(CONNECT_TIMEOUT))
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;
    let _ = sock.set_nodelay(true);
    Ok((sock, rtt))
}

pub fn fetch_status(addr: &str) -> Result<Status> {
    let (mut sock, _) = dial(addr)?;
    let request = format!(
        "GET /status HTTP/1.0\r\nHost: {addr}\r\nUser-Agent: {}\r\nConnection: close\r\n\r\n",
        httpc::USER_AGENT
    );
    sock.write_all(request.as_bytes()).map_err(|e| Error::other(format!("{addr}: {e}")))?;
    let mut raw = Vec::new();
    sock.take(MAX_STATUS as u64)
        .read_to_end(&mut raw)
        .map_err(|e| Error::other(format!("{addr} sent no status: {e}")))?;
    let text = String::from_utf8_lossy(&raw);
    let (head, body) = text
        .split_once("\r\n\r\n")
        .ok_or_else(|| Error::other(format!("{addr} is not a KiwiSDR")))?;
    let answered = head.lines().next().and_then(|l| l.split_whitespace().nth(1));
    if answered != Some("200") {
        return Err(Error::other(format!("{addr} is not a KiwiSDR")));
    }
    Status::parse(body).map_err(|_| Error::other(format!("{addr} is not a KiwiSDR")))
}

pub fn probe(addr: &str) -> Result<Probe> {
    let addr = Proto::KiwiSdr.parse_addr(addr).ok_or(Error::NoDevice)?;
    let status = fetch_status(&addr)?;
    if status.offline {
        return Err(Error::other(format!("{addr} is offline")));
    }
    tracing::debug!(
        "KiwiSDR {addr}: {} users of {}, {:?}",
        status.users,
        status.users_max,
        status.bands
    );
    Ok(Probe {
        proto: Proto::KiwiSdr,
        addr,
        center: None,
        rate: Some(status.rate),
        rates: Vec::new(),
        gain_db: None,
        name: String::new(),
        settings: Vec::new(),
        tunable: true,
        tune_range: Some(status.bands),
        tuner: TUNER.to_string(),
    })
}

#[derive(Clone, Debug, PartialEq)]
enum Said {
    SampleRate(f64),
    AudioRate(u32),
    FreqOffsetKhz(f64),
    Admitted,
    Refused(String),
}

fn unquote(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        let hex = b.get(i + 1..i + 3).and_then(|h| std::str::from_utf8(h).ok());
        match (b[i], hex.and_then(|h| u8::from_str_radix(h, 16).ok())) {
            (b'%', Some(v)) => {
                out.push(v);
                i += 3;
            }
            (c, _) => {
                out.push(c);
                i += 1;
            }
        }
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn read_msg(body: &str) -> Vec<Said> {
    body.split(' ')
        .filter_map(|pair| {
            let (name, value) = pair.split_once('=').unwrap_or((pair, ""));
            Some(match name {
                "sample_rate" => Said::SampleRate(value.parse().ok()?),
                "audio_rate" => Said::AudioRate(value.parse().ok()?),
                "freq_offset" => Said::FreqOffsetKhz(value.parse().ok()?),
                "badp" => match value {
                    "0" => Said::Admitted,
                    "1" => Said::Refused("wants a password, or every open channel is taken".into()),
                    "5" => Said::Refused("takes one connection per address".into()),
                    other => Said::Refused(format!("refused the connection (badp={other})")),
                },
                "too_busy" => Said::Refused(format!("has all {value} channels taken")),
                "down" => Said::Refused("is down".into()),
                "redirect" => Said::Refused(format!("redirects to {}", unquote(value))),
                _ => return None,
            })
        })
        .collect()
}

#[derive(Debug, PartialEq)]
enum Frame<'a> {
    Msg(&'a str),
    Snd(&'a [u8]),
    Other,
}

fn frame(raw: &[u8]) -> Frame<'_> {
    match raw.get(..3) {
        Some(b"MSG") => Frame::Msg(std::str::from_utf8(&raw[3..]).unwrap_or("").trim_start()),
        Some(b"SND") => Frame::Snd(&raw[3..]),
        _ => Frame::Other,
    }
}

fn read_snd(body: &[u8], out: &mut Vec<C32>) -> Result<u32> {
    if body.len() < SND_HEADER {
        return Err(Error::other("KiwiSDR sent a short SND frame"));
    }
    let flags = body[0];
    let seq = u32::from_le_bytes(body[1..5].try_into().unwrap());
    if flags & FLAG_STEREO == 0 || flags & FLAG_COMPRESSED != 0 {
        return Err(Error::other(format!(
            "KiwiSDR sent audio rather than IQ (flags {flags:#04x})"
        )));
    }
    let data = body.get(SND_HEADER + GPS_HEADER..).unwrap_or(&[]);
    out.extend(data.chunks_exact(4).map(|s| {
        let i = i16::from_be_bytes([s[0], s[1]]) as f32 / 32768.0;
        let q = i16::from_be_bytes([s[2], s[3]]) as f32 / 32768.0;
        C32::new(i, q)
    }));
    Ok(seq)
}

fn tune_command(center: Hz, rate: Sps, offset_khz: f64) -> String {
    let half = (rate.as_f64() / 2.0).round() as i64;
    let khz = center.as_f64() / 1e3 - offset_khz;
    format!("SET mod=iq low_cut={} high_cut={half} freq={khz:.3}", -half)
}

fn gain_command(mode: GainMode) -> String {
    match mode {
        GainMode::Auto => "SET agc=1 hang=0 thresh=-100 slope=6 decay=1000 manGain=50".into(),
        GainMode::Manual(db) => format!(
            "SET agc=0 hang=0 thresh=-100 slope=6 decay=1000 manGain={}",
            db.clamp(0.0, MAX_GAIN_DB).round() as i32
        ),
    }
}

fn say(ws: &mut Ws, text: impl Into<String>) -> Result<()> {
    ws.send(Message::text(text.into())).map_err(|e| Error::other(format!("KiwiSDR: {e}")))
}

fn timed_out(e: &tungstenite::Error) -> bool {
    matches!(
        e,
        tungstenite::Error::Io(io)
            if matches!(io.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut)
    )
}

fn raw(m: &Message) -> Option<&[u8]> {
    match m {
        Message::Binary(b) => Some(b),
        Message::Text(t) => Some(t.as_bytes()),
        _ => None,
    }
}

struct Admission {
    rate: f64,
    offset_khz: f64,
}

fn admit(ws: &mut Ws, addr: &str) -> Result<Admission> {
    say(ws, "SET auth t=kiwi p=")?;
    say(ws, "SET ident_user=WaveShark")?;
    let deadline = Instant::now() + HANDSHAKE_TIMEOUT;
    let (mut rate, mut admitted, mut answered, mut offset_khz) = (None, false, false, 0.0);
    while rate.is_none() || !admitted || !answered {
        if Instant::now() > deadline {
            return Err(Error::other(format!("{addr} never said its sample rate")));
        }
        let m = match ws.read() {
            Ok(m) => m,
            Err(e) if timed_out(&e) => continue,
            Err(e) => return Err(Error::other(format!("{addr}: {e}"))),
        };
        let Some(Frame::Msg(body)) = raw(&m).map(frame) else { continue };
        for said in read_msg(body) {
            match said {
                Said::SampleRate(r) => rate = Some(r),
                Said::AudioRate(r) => {
                    say(ws, format!("SET AR OK in={r} out=44100"))?;
                    answered = true;
                }
                Said::FreqOffsetKhz(k) => offset_khz = k,
                Said::Admitted => admitted = true,
                Said::Refused(why) => return Err(Error::other(format!("{addr} {why}"))),
            }
        }
    }
    Ok(Admission { rate: rate.unwrap_or_default(), offset_khz })
}

fn connect(addr: &str) -> Result<(Ws, Duration)> {
    let (sock, rtt) = dial(addr)?;
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or_default();
    let mut req = format!("ws://{addr}/{stamp}/SND")
        .into_client_request()
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;
    req.headers_mut().insert(
        tungstenite::http::header::USER_AGENT,
        tungstenite::http::HeaderValue::from_static(httpc::USER_AGENT),
    );
    let (ws, _) = tungstenite::client(req, sock)
        .map_err(|e| Error::other(format!("{addr} is not a KiwiSDR: {e}")))?;
    Ok((ws, rtt))
}

enum Ctl {
    Say(String),
    Start(Sender<IqBuf>, Arc<AtomicU64>),
    Stop,
}

struct Session {
    ws: Ws,
    ctl: Receiver<Ctl>,
    center: Arc<AtomicU64>,
    rate: Sps,
    streaming: Arc<AtomicBool>,
}

struct Out {
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    last: Option<u32>,
    counted: u64,
}

impl Out {
    fn deliver(&mut self, seq: u32, samples: Vec<C32>, center: Hz, rate: Sps) -> bool {
        let n = samples.len() as u64;
        if let Some(prev) = self.last {
            let lost = seq.wrapping_sub(prev).wrapping_sub(1) as u64 * n;
            if lost > 0 {
                self.counted += lost;
                self.dropped.fetch_add(lost, Ordering::Relaxed);
                tracing::warn!(
                    "KiwiSDR: {lost} samples ({:.0} ms) never arrived",
                    lost as f64 * 1000.0 / rate.as_f64()
                );
            }
        }
        self.last = Some(seq);
        let buf = IqBuf::new(samples, center, rate, self.counted);
        self.counted += n;
        match self.tx.try_send(buf) {
            Ok(()) => true,
            Err(TrySendError::Full(buf)) => {
                self.dropped.fetch_add(buf.len() as u64, Ordering::Relaxed);
                true
            }
            Err(TrySendError::Disconnected(_)) => false,
        }
    }
}

impl Session {
    fn heard(&mut self, body: &str) -> Result<()> {
        for said in read_msg(body) {
            match said {
                Said::AudioRate(r) => say(&mut self.ws, format!("SET AR OK in={r} out=44100"))?,
                Said::Refused(why) => return Err(Error::other(format!("KiwiSDR {why}"))),
                _ => {}
            }
        }
        Ok(())
    }

    fn run(mut self) {
        if let Err(e) = self.ws.get_mut().set_read_timeout(Some(POLL)) {
            tracing::warn!("KiwiSDR: {e}");
            return;
        }
        let mut out: Option<Out> = None;
        let mut alive = Instant::now();
        loop {
            loop {
                match self.ctl.try_recv() {
                    Ok(Ctl::Say(s)) => {
                        if let Err(e) = say(&mut self.ws, s) {
                            tracing::warn!("{e}");
                        }
                    }
                    Ok(Ctl::Start(tx, dropped)) => {
                        out = Some(Out { tx, dropped, last: None, counted: 0 })
                    }
                    Ok(Ctl::Stop) => out = None,
                    Err(TryRecvError::Empty) => break,
                    Err(TryRecvError::Disconnected) => {
                        let _ = self.ws.close(None);
                        let _ = self.ws.flush();
                        return;
                    }
                }
            }
            if alive.elapsed() >= KEEPALIVE {
                alive = Instant::now();
                if say(&mut self.ws, "SET keepalive").is_err() {
                    break;
                }
            }
            let m = match self.ws.read() {
                Ok(m) => m,
                Err(e) if timed_out(&e) => continue,
                Err(e) => {
                    tracing::warn!("KiwiSDR: {e}");
                    break;
                }
            };
            if let Message::Close(_) = m {
                break;
            }
            match raw(&m).map(frame) {
                Some(Frame::Snd(body)) => {
                    let Some(o) = out.as_mut() else { continue };
                    let mut samples = Vec::new();
                    match read_snd(body, &mut samples) {
                        Ok(seq) => {
                            let center = Hz(self.center.load(Ordering::Relaxed));
                            if !o.deliver(seq, samples, center, self.rate) {
                                out = None;
                            }
                        }
                        Err(e) => tracing::warn!("{e}"),
                    }
                }
                Some(Frame::Msg(body)) => {
                    if let Err(e) = self.heard(body) {
                        tracing::warn!("{e}");
                        break;
                    }
                }
                _ => {}
            }
        }
        self.streaming.store(false, Ordering::SeqCst);
    }
}

pub struct Device {
    addr: String,
    info: DeviceInfo,
    tuning: common::Tuning,
    ctl: Sender<Ctl>,
    center: Arc<AtomicU64>,
    rate: Sps,
    offset_khz: f64,
    gain: GainMode,
    settle: Duration,
    streaming: Arc<AtomicBool>,
}

impl Device {
    pub fn open(addr: &str) -> Result<Self> {
        let addr = Proto::KiwiSdr.parse_addr(addr).ok_or(Error::NoDevice)?;
        let status = fetch_status(&addr)?;
        if status.offline {
            return Err(Error::other(format!("{addr} is offline")));
        }
        let (mut ws, rtt) = connect(&addr)?;
        let admitted = admit(&mut ws, &addr)?;
        let rate = Sps(admitted.rate.round() as u64);
        if rate.0 == 0 {
            return Err(Error::other(format!("{addr} said a sample rate of zero")));
        }
        let center = status.bands.start().0.max(Hz::mhz(10).0).min(status.bands.end().0);
        let gain = GainMode::Auto;
        for s in [
            "SET squelch=0 max=0".to_string(),
            "SET genattn=0".to_string(),
            "SET gen=0 mix=-1".to_string(),
            tune_command(Hz(center), rate, admitted.offset_khz),
            gain_command(gain),
            "SET compression=0".to_string(),
            "SET keepalive".to_string(),
        ] {
            say(&mut ws, s)?;
        }
        tracing::debug!("KiwiSDR {addr}: {rate}, {rtt:?} away");

        let info = DeviceInfo {
            kind: DriverKind::Network,
            id: format!("kiwisdr:{addr}"),
            label: format!("KiwiSDR {addr}"),
            tuner: TUNER.to_string(),
            ranges: vec![TunerRange { range: status.bands.clone(), label: "HF" }],
            rates: vec![rate],
            rate_range: rate..=rate,
            gain_stages: vec![GainStage {
                name: GAIN.to_string(),
                label: "KiwiSDR gain".to_string(),
                range: 0.0..=MAX_GAIN_DB,
                values: Vec::new(),
                step: 1.0,
                auto: true,
            }],
            native_format: SampleFormat::Cs16,
            usable_bandwidth_ratio: USABLE_RATIO,
            tunable: true,
            tx: None,
        };

        let (ctl, rx) = unbounded();
        let center = Arc::new(AtomicU64::new(center));
        let streaming = Arc::new(AtomicBool::new(false));
        let session =
            Session { ws, ctl: rx, center: center.clone(), rate, streaming: streaming.clone() };
        std::thread::Builder::new()
            .name("kiwisdr".into())
            .spawn(move || session.run())
            .map_err(|e| Error::other(format!("spawn KiwiSDR thread: {e}")))?;

        Ok(Self {
            addr,
            info,
            tuning: Default::default(),
            ctl,
            center,
            rate,
            offset_khz: admitted.offset_khz,
            gain,
            settle: rtt + Duration::from_secs_f64(2.0 * 512.0 / rate.as_f64()),
            streaming,
        })
    }

    pub fn address(&self) -> &str {
        &self.addr
    }

    fn tell(&self, s: String) -> Result<()> {
        self.ctl.send(Ctl::Say(s)).map_err(|_| Error::Disconnected)
    }
}

impl DeviceTrait for Device {
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
            let (lo, hi) = (*self.info.ranges[0].range.start(), *self.info.ranges[0].range.end());
            return Err(Error::FreqOutOfRange { req: f, lo, hi });
        }
        self.tell(tune_command(f, self.rate, self.offset_khz))?;
        self.center.store(f.0, Ordering::Relaxed);
        Ok(())
    }

    fn center(&self) -> Hz {
        Hz(self.center.load(Ordering::Relaxed))
    }

    fn settle(&self) -> Duration {
        self.settle
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        match (r.as_f64() - self.rate.as_f64()).abs() <= self.rate.as_f64() * RATE_TOLERANCE {
            true => Ok(()),
            false => Err(Error::RateUnsupported { req: r }),
        }
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        if stage != GAIN {
            return Err(Error::other(format!("KiwiSDR has no {stage} gain")));
        }
        self.tell(gain_command(mode))?;
        self.gain = mode;
        Ok(())
    }

    fn gains(&self) -> Vec<(String, GainMode)> {
        vec![(GAIN.to_string(), self.gain)]
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        if self.ctl.send(Ctl::Start(tx, dropped.clone())).is_err() {
            self.streaming.store(false, Ordering::SeqCst);
            return Err(Error::Disconnected);
        }
        Ok(Box::new(NetStream {
            rx,
            dropped,
            ctl: self.ctl.clone(),
            streaming: self.streaming.clone(),
        }))
    }
}

struct NetStream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    ctl: Sender<Ctl>,
    streaming: Arc<AtomicBool>,
}

impl RxStream for NetStream {
    fn read(&mut self) -> Result<IqBuf> {
        self.rx.recv().map_err(|_| Error::Disconnected)
    }

    fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    fn stop(&mut self) {
        let _ = self.ctl.send(Ctl::Stop);
        self.streaming.store(false, Ordering::SeqCst);
    }
}

impl Drop for NetStream {
    fn drop(&mut self) {
        self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::TcpListener;

    const TARLEE: &str = "status=active\noffline=no\nname=2-30MHZ SDR #1, VK5ARG Remote Receiver Site | Near Tarlee, South Australia\nbands=1800000-30000000\nfreq_offset=0.000\nmode=rx8.wf3\nusers=6\nusers_max=8\nsw_version=KiwiSDR_v1.902\n";

    const IRONSTONE: &str =
        "status=active\noffline=no\nbands=0-30000000\nusers=1\nusers_max=8\nmode=rx8.wf3\n";

    const IRONSTONE_HANDSHAKE: &[&str] = &[
        "MSG sample_rate=11998.884049",
        "MSG client_public_ip=83.71.105.199",
        "MSG rx_chans=8 firmware_sel=5",
        "MSG chan_no_pwd=3",
        "MSG chan_no_pwd_true=0",
        "MSG is_local=0,0,0",
        "MSG max_camp=4",
        "MSG badp=0",
        "MSG version_maj=1 version_min=902 debian_ver=8 model=1 platform=0 hw=1 ext_clk=0 freq_offset=0.000 abyy=A26 dx_db_name=dx has_attn=0",
        "MSG cfg_loaded",
        "MSG center_freq=15000000 bandwidth=30000000 adc_clk_nom=66666600",
        "MSG audio_init=0 audio_rate=12000",
        "MSG last_community_download=Downloads%20enabled.%20Last%20checked%3a%20Wed%20Sep%2023%2002%3a43%3a09%202026",
        "MSG max_thr=90",
        "MSG rf_attn=0.0",
    ];

    const IRONSTONE_SND_6: &str =
        "534e44080600000002b8fc00a695040092815c2bf065f602f082f651f067f668efedf6f7";

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn the_status_page_says_the_band_and_how_full_it_is() {
        let s = Status::parse(TARLEE).unwrap();
        assert_eq!(
            s.name,
            "2-30MHZ SDR #1, VK5ARG Remote Receiver Site | Near Tarlee, South Australia"
        );
        assert_eq!(s.bands, Hz(1_800_000)..=Hz::mhz(30));
        assert_eq!((s.users, s.users_max), (6, 8));
        assert!(!s.offline);
        assert_eq!(s.rate, Sps(12_000));

        let s = Status::parse(IRONSTONE).unwrap();
        assert_eq!(s.bands, Hz(0)..=Hz::mhz(30));
        assert_eq!((s.users, s.users_max), (1, 8));

        let three = Status::parse("status=active\nmode=rx3.wf3\n").unwrap();
        assert_eq!(
            three.rate,
            Sps(20_250),
            "rx3 firmware said 20250.84 to 20251.06 S/s on four public receivers"
        );
        assert_eq!(three.bands, Hz(0)..=Hz::mhz(30), "no bands line reads as the whole of HF");

        assert!(Status::parse("<html>not found</html>").is_err());
    }

    #[test]
    fn the_handshake_says_the_rate_and_lets_us_in() {
        let said: Vec<Said> = IRONSTONE_HANDSHAKE
            .iter()
            .flat_map(|m| match frame(m.as_bytes()) {
                Frame::Msg(body) => read_msg(body),
                other => panic!("{other:?}"),
            })
            .collect();
        assert_eq!(
            said,
            vec![
                Said::SampleRate(11998.884049),
                Said::Admitted,
                Said::FreqOffsetKhz(0.0),
                Said::AudioRate(12000),
            ]
        );
    }

    #[test]
    fn a_refusal_says_why() {
        let why = |m: &str| match read_msg(m).pop() {
            Some(Said::Refused(why)) => why,
            other => panic!("{other:?}"),
        };
        assert_eq!(why("badp=1"), "wants a password, or every open channel is taken");
        assert_eq!(why("too_busy=8"), "has all 8 channels taken");
        assert_eq!(why("badp=5"), "takes one connection per address");
        assert_eq!(
            why("redirect=http%3a%2f%2fother.test%3a8073"),
            "redirects to http://other.test:8073"
        );
        assert_eq!(why("down"), "is down");
    }

    #[test]
    fn an_iq_frame_is_big_endian_pairs_after_the_gps_stamp() {
        let raw = hex(IRONSTONE_SND_6);
        let Frame::Snd(body) = frame(&raw) else { panic!("not SND") };
        let mut out = Vec::new();
        assert_eq!(read_snd(body, &mut out).unwrap(), 6);
        let want = [(-3995, -2558), (-3966, -2479), (-3993, -2456), (-4115, -2313)];
        assert_eq!(out.len(), 4);
        for (c, (i, q)) in out.iter().zip(want) {
            assert_eq!((c.re, c.im), (i as f32 / 32768.0, q as f32 / 32768.0));
        }

        let mut audio = raw.clone();
        audio[3] = 0x10;
        let Frame::Snd(body) = frame(&audio) else { panic!("not SND") };
        assert!(read_snd(body, &mut Vec::new()).is_err(), "compressed mono is not IQ");
    }

    #[test]
    fn a_tune_asks_for_the_whole_rate_in_kilohertz() {
        assert_eq!(
            tune_command(Hz(7_100_000), Sps(11_999), 0.0),
            "SET mod=iq low_cut=-6000 high_cut=6000 freq=7100.000"
        );
        assert_eq!(
            tune_command(Hz(144_300_000), Sps(12_000), 116_000.0),
            "SET mod=iq low_cut=-6000 high_cut=6000 freq=28300.000",
            "a KiwiSDR behind a converter is tuned below its dial"
        );
        assert_eq!(
            gain_command(GainMode::Manual(250.0)),
            "SET agc=0 hang=0 thresh=-100 slope=6 decay=1000 manGain=120"
        );
    }

    fn snd(seq: u32) -> Vec<u8> {
        let mut f = b"SND".to_vec();
        f.push(FLAG_STEREO);
        f.extend(seq.to_le_bytes());
        f.extend([0x02, 0xb8]);
        f.extend([0u8; GPS_HEADER]);
        for k in 0..512i16 {
            f.extend((seq as i16 * 100).to_be_bytes());
            f.extend((-k).to_be_bytes());
        }
        f
    }

    fn fake(
        status: &'static str,
        greeting: &'static [&'static str],
        seqs: &'static [u32],
    ) -> (String, std::sync::mpsc::Receiver<String>) {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = l.local_addr().unwrap().to_string();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            for sock in l.incoming() {
                let mut sock = sock.unwrap();
                let mut peek = [0u8; 12];
                while sock.peek(&mut peek).unwrap() < peek.len() {
                    std::thread::sleep(Duration::from_millis(1));
                }
                if &peek == b"GET /status " {
                    let mut req = Vec::new();
                    let mut byte = [0u8; 1];
                    while !req.ends_with(b"\r\n\r\n") && sock.read(&mut byte).unwrap() == 1 {
                        req.push(byte[0]);
                    }
                    let _ =
                        write!(sock, "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n{status}");
                    continue;
                }
                let tx = tx.clone();
                std::thread::spawn(move || {
                    let mut ws = tungstenite::accept(sock).unwrap();
                    for m in greeting {
                        ws.send(Message::text(*m)).unwrap();
                    }
                    ws.get_mut().set_read_timeout(Some(Duration::from_millis(20))).unwrap();
                    let mut tunes = 0;
                    loop {
                        match ws.read() {
                            Ok(Message::Text(t)) => {
                                tunes += t.starts_with("SET mod=") as usize;
                                if tx.send(t.to_string()).is_err() {
                                    return;
                                }
                                if tunes == 2 {
                                    break;
                                }
                            }
                            Ok(_) => {}
                            Err(e) if timed_out(&e) => {}
                            Err(_) => return,
                        }
                    }
                    for s in seqs {
                        ws.send(Message::binary(snd(*s))).unwrap();
                    }
                    let _ = ws.close(None);
                    let _ = ws.flush();
                });
            }
        });
        (addr, rx)
    }

    #[test]
    fn a_probe_reads_the_status_page_and_takes_no_channel() {
        let (addr, said) = fake(TARLEE, &[], &[]);
        let p = probe(&format!("http://{addr}/")).unwrap();
        assert_eq!(p.proto, Proto::KiwiSdr);
        assert_eq!(p.addr, addr);
        assert_eq!(p.rate, Some(Sps(12_000)));
        assert_eq!(p.tune_range, Some(Hz(1_800_000)..=Hz::mhz(30)));
        assert!(p.tunable);
        assert_eq!(p.center, None);
        assert!(said.recv_timeout(Duration::from_millis(200)).is_err(), "no socket was opened");
    }

    #[test]
    fn an_offline_receiver_is_not_offered() {
        let (addr, _) = fake("status=active\noffline=yes\nbands=0-30000000\n", &[], &[]);
        let e = probe(&addr).unwrap_err().to_string();
        assert!(e.ends_with("is offline"), "{e}");
    }

    #[test]
    fn a_receiver_wanting_a_password_says_so() {
        let (addr, _) = fake(IRONSTONE, &["MSG sample_rate=11998.884049", "MSG badp=1"], &[]);
        let e = Device::open(&addr).err().unwrap().to_string();
        assert_eq!(e, format!("{addr} wants a password, or every open channel is taken"));
    }

    #[test]
    fn a_stream_is_numbered_by_the_frames_the_receiver_sent() {
        let (addr, said) = fake(IRONSTONE, IRONSTONE_HANDSHAKE, &[1, 2, 3, 5, 6]);
        let mut d = Device::open(&addr).unwrap();
        assert_eq!(d.rate(), Sps(11_999));
        assert_eq!(d.info().rate_range, Sps(11_999)..=Sps(11_999));
        assert!(d.set_rate(Sps(12_000)).is_ok(), "the probe's nominal rate is this one");
        assert!(d.set_rate(Sps(20_250)).is_err());
        let mut s = d.start_rx().unwrap();
        d.set_center(Hz(7_100_000)).unwrap();
        assert!(d.set_center(Hz::mhz(31)).is_err());

        let mut got = Vec::new();
        while let Ok(b) = s.read() {
            got.push(b);
        }
        let seqs: Vec<u64> = got.iter().map(|b| b.seq).collect();
        assert_eq!(seqs, vec![0, 512, 1024, 2048, 2560], "frame 4 never came");
        assert_eq!(s.dropped(), 512);
        assert!(got.iter().all(|b| b.len() == 512 && b.rate == Sps(11_999)));
        assert!(got.iter().all(|b| b.center == Hz(7_100_000)));
        let firsts: Vec<f32> = got.iter().map(|b| b.samples[0].re * 32768.0).collect();
        assert_eq!(firsts, vec![100.0, 200.0, 300.0, 500.0, 600.0]);
        assert_eq!(got[0].samples[511].im * 32768.0, -511.0);

        let commands: Vec<String> = said.try_iter().filter(|c| c != "SET keepalive").collect();
        assert_eq!(
            commands,
            vec![
                "SET auth t=kiwi p=",
                "SET ident_user=WaveShark",
                "SET AR OK in=12000 out=44100",
                "SET squelch=0 max=0",
                "SET genattn=0",
                "SET gen=0 mix=-1",
                "SET mod=iq low_cut=-6000 high_cut=6000 freq=10000.000",
                "SET agc=1 hang=0 thresh=-100 slope=6 decay=1000 manGain=50",
                "SET compression=0",
                "SET mod=iq low_cut=-6000 high_cut=6000 freq=7100.000",
            ]
        );
    }

    #[test]
    fn a_second_reader_is_refused_while_one_is_running() {
        let (addr, _said) = fake(IRONSTONE, IRONSTONE_HANDSHAKE, &[]);
        let mut d = Device::open(&addr).unwrap();
        let mut s = d.start_rx().unwrap();
        assert!(matches!(d.start_rx(), Err(Error::Busy)));
        s.stop();
        assert!(d.start_rx().is_ok());
    }
}
