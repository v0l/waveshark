use super::{
    COMMAND_HEADER, Cmd, DeviceType, FORMAT_UINT8, Info, PROTOCOL_VERSION, STREAM_MODE_IQ, Set,
    Sync,
};
use crate::cut::Cut;
use common::C32;
use dsp::spectrum::{Detector, Spectrum};
use iqstream::server::{Stream, Tapped};
use iqstream::{SettingKind, SettingValue};
use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

const MAX_COMMAND_BODY: usize = 256;
const HELLO_WAIT: Duration = Duration::from_secs(5);
const IDLE: Duration = Duration::from_millis(100);
const LOWEST_RATE: u32 = 8_000;
const MOST_STAGES: u32 = 15;
const CHUNK_SAMPLES: usize = 32_768;
const FFT_FPS: f64 = 15.0;
const FFT_MIN: usize = 128;
const FFT_MAX: usize = 8_192;
const STREAM_STATUS: u32 = 0;
const STREAM_IQ: u32 = 1;
const STREAM_FFT: u32 = 4;
const MSG_DEVICE_INFO: u32 = 0;
const MSG_CLIENT_SYNC: u32 = 1;
const MSG_PONG: u32 = 2;
const MSG_UINT8_FFT: u32 = 301;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Format {
    Uint8,
    Int16,
    Int24,
    Float,
}

impl Format {
    fn from_code(code: u32) -> Option<Self> {
        match code {
            1 => Some(Self::Uint8),
            2 => Some(Self::Int16),
            3 => Some(Self::Int24),
            4 => Some(Self::Float),
            _ => None,
        }
    }

    fn message(self) -> u32 {
        match self {
            Self::Uint8 => 100,
            Self::Int16 => 101,
            Self::Int24 => 102,
            Self::Float => 103,
        }
    }

    fn full_scale(self) -> f32 {
        match self {
            Self::Uint8 => 128.0,
            Self::Int16 => 32_768.0,
            Self::Int24 => 8_388_608.0,
            Self::Float => 1.0,
        }
    }

    fn encode(self, iq: &[C32], gain: f32, out: &mut Vec<u8>) {
        let scale = self.full_scale() * gain;
        let top = self.full_scale() - 1.0;
        let bottom = -self.full_scale();
        for v in iq.iter().flat_map(|s| [s.re, s.im]) {
            let x = v * scale;
            match self {
                Self::Uint8 => out.push((x.round().clamp(bottom, top) + 128.0) as u8),
                Self::Int16 => {
                    out.extend_from_slice(&(x.round().clamp(bottom, top) as i16).to_le_bytes())
                }
                Self::Int24 => {
                    out.extend_from_slice(&(x.round().clamp(bottom, top) as i32).to_le_bytes()[..3])
                }
                Self::Float => out.extend_from_slice(&x.to_le_bytes()),
            }
        }
    }
}

#[derive(Clone, Debug)]
struct Asked {
    streaming: bool,
    mode: u32,
    format: Format,
    iq_hz: u32,
    iq_decimation: u32,
    digital_gain_db: u32,
    fft_uint8: bool,
    fft_hz: u32,
    fft_decimation: u32,
    fft_db_offset: i32,
    fft_db_range: u32,
    fft_pixels: u32,
    gain_index: u32,
    sync_due: bool,
    pongs: Vec<Vec<u8>>,
}

impl Asked {
    fn new(center_hz: u32) -> Self {
        Self {
            streaming: false,
            mode: STREAM_MODE_IQ,
            format: Format::Uint8,
            iq_hz: center_hz,
            iq_decimation: 0,
            digital_gain_db: 0,
            fft_uint8: true,
            fft_hz: center_hz,
            fft_decimation: 0,
            fft_db_offset: 0,
            fft_db_range: 127,
            fft_pixels: 1_024,
            gain_index: 0,
            sync_due: false,
            pongs: Vec::new(),
        }
    }

    fn wants_iq(&self) -> bool {
        self.streaming && self.mode & STREAM_IQ != 0
    }

    fn wants_fft(&self) -> bool {
        self.streaming && self.mode & STREAM_FFT != 0 && self.fft_uint8
    }
}

fn device_type(hardware: &str) -> DeviceType {
    match hardware {
        "airspy" => DeviceType::AirspyOne,
        "airspyhf" => DeviceType::AirspyHf,
        _ => DeviceType::RtlSdr,
    }
}

fn gains(stream: &Stream) -> Option<(String, Vec<f32>)> {
    stream
        .settings()
        .into_iter()
        .find(|s| s.kind == SettingKind::Gain && !s.gains_db.is_empty())
        .map(|s| (s.name, s.gains_db))
}

fn stages(rate: u32) -> u32 {
    (0..=MOST_STAGES).take_while(|n| rate >> n >= LOWEST_RATE).last().unwrap_or(0)
}

fn reach_hz(stream: &Stream) -> (u32, u32) {
    let (c, half) = (stream.center_hz(), stream.sample_rate() as u64 / 2);
    match (stream.tunable(), stream.tune_range_hz()) {
        (true, Some((lo, hi))) => (lo as u32, hi as u32),
        _ => (c.saturating_sub(half) as u32, (c + half) as u32),
    }
}

fn info(stream: &Stream) -> Info {
    let rate = stream.sample_rate();
    let (min_hz, max_hz) = reach_hz(stream);
    Info {
        device: device_type(&stream.hardware()),
        serial: 0,
        max_rate: rate,
        max_bandwidth: rate,
        decimation_stages: stages(rate),
        gain_stages: 0,
        max_gain_index: gains(stream).map_or(0, |(_, g)| g.len().saturating_sub(1) as u32),
        min_hz,
        max_hz,
        resolution: 8,
        min_decimation: 0,
        forced_format: 0,
    }
}

fn window(stream: &Stream, decimation: u32) -> (u32, u32) {
    if stream.tunable() {
        return reach_hz(stream);
    }
    let rate = stream.sample_rate() as f64;
    let reach = Cut::reach(rate, rate / (1u64 << decimation) as f64);
    let c = stream.center_hz() as f64;
    ((c - reach) as u32, (c + reach) as u32)
}

fn sync(stream: &Stream, asked: &Asked) -> Sync {
    let (min_iq, max_iq) = window(stream, asked.iq_decimation);
    let (min_fft, max_fft) = window(stream, asked.fft_decimation);
    Sync {
        can_control: stream.tunable(),
        gain: asked.gain_index,
        device_center: stream.center_hz() as u32,
        iq_center: asked.iq_hz.clamp(min_iq, max_iq.max(min_iq)),
        fft_center: asked.fft_hz.clamp(min_fft, max_fft.max(min_fft)),
        min_iq_center: min_iq,
        max_iq_center: max_iq,
        min_fft_center: min_fft,
        max_fft_center: max_fft,
    }
}

struct Wire {
    sock: TcpStream,
    seq: u32,
    head: Vec<u8>,
}

impl Wire {
    fn send(&mut self, kind: u32, flags: u32, stream: u32, body: &[u8]) -> std::io::Result<()> {
        self.head.clear();
        for w in [PROTOCOL_VERSION, kind | (flags << 16), stream, self.seq, body.len() as u32] {
            self.head.extend_from_slice(&w.to_le_bytes());
        }
        self.seq = self.seq.wrapping_add(1);
        self.sock.write_all(&self.head)?;
        self.sock.write_all(body)
    }
}

fn hello(sock: &mut TcpStream) -> std::io::Result<String> {
    sock.set_read_timeout(Some(HELLO_WAIT))?;
    let (cmd, body) = command(sock)?;
    sock.set_read_timeout(None)?;
    if cmd != Cmd::Hello as u32 || body.len() < 4 {
        return Err(std::io::Error::other("the first command was not a hello"));
    }
    let version = u32::from_le_bytes([body[0], body[1], body[2], body[3]]);
    if version >> 24 != PROTOCOL_VERSION >> 24 {
        return Err(std::io::Error::other(format!("protocol {}", version >> 24)));
    }
    Ok(String::from_utf8_lossy(&body[4..]).into_owned())
}

fn command(sock: &mut TcpStream) -> std::io::Result<(u32, Vec<u8>)> {
    let mut head = [0u8; COMMAND_HEADER];
    sock.read_exact(&mut head)?;
    let cmd = u32::from_le_bytes([head[0], head[1], head[2], head[3]]);
    let len = u32::from_le_bytes([head[4], head[5], head[6], head[7]]) as usize;
    if len > MAX_COMMAND_BODY {
        return Err(std::io::Error::other(format!("a {len} byte command")));
    }
    let mut body = vec![0u8; len];
    sock.read_exact(&mut body)?;
    Ok((cmd, body))
}

pub fn serve(mut sock: TcpStream, peer: SocketAddr, stream: Arc<Stream>) {
    let who = match hello(&mut sock) {
        Ok(name) => format!("{name} at {peer}"),
        Err(e) => {
            tracing::debug!("spyserver: {peer}: {e}");
            return;
        }
    };
    let asked = Arc::new(Mutex::new(Asked::new(stream.center_hz() as u32)));
    let gone = Arc::new(AtomicBool::new(false));
    let Ok(commands) = sock.try_clone() else { return };
    let (a, g, s) = (asked.clone(), gone.clone(), stream.clone());
    if std::thread::Builder::new()
        .name("spyserver-cmd".into())
        .spawn(move || listen(commands, &s, &a, &g))
        .is_err()
    {
        return;
    }
    tracing::info!("spyserver: {who} reading {}", stream.name());
    let _ = sock.set_nodelay(true);
    let wire = Wire { sock, seq: 0, head: Vec::with_capacity(20) };
    if let Err(e) = send(wire, &stream, &asked, &gone) {
        tracing::debug!("spyserver: {who}: {e}");
    }
    gone.store(true, Ordering::SeqCst);
    tracing::info!("spyserver: {who} left");
}

fn listen(mut sock: TcpStream, stream: &Stream, asked: &Mutex<Asked>, gone: &AtomicBool) {
    while let Ok((cmd, body)) = command(&mut sock) {
        let Ok(mut a) = asked.lock() else { break };
        if cmd == Cmd::Ping as u32 {
            a.pongs.push(body);
            continue;
        }
        if cmd != Cmd::SetSetting as u32 || body.len() < 8 {
            continue;
        }
        let word = |i: usize| u32::from_le_bytes([body[i], body[i + 1], body[i + 2], body[i + 3]]);
        let (which, value) = (word(0), word(4));
        let Some(which) = Set::from_code(which) else { continue };
        match which {
            Set::StreamingMode => a.mode = value,
            Set::StreamingEnabled => a.streaming = value != 0,
            Set::Gain => {
                if let Some((name, steps)) = gains(stream) {
                    let i = (value as usize).min(steps.len() - 1);
                    if stream.ask_setting(&name, SettingValue::Gain(steps[i])) {
                        a.gain_index = i as u32;
                    }
                }
                a.sync_due = true;
            }
            Set::IqFormat => {
                if let Some(f) = Format::from_code(value) {
                    a.format = f;
                }
            }
            Set::IqFrequency => {
                a.iq_hz = value;
                stream.ask(value as u64);
                a.sync_due = true;
            }
            Set::IqDecimation => {
                a.iq_decimation = value.min(stages(stream.sample_rate()));
                a.sync_due = true;
            }
            Set::IqDigitalGain => a.digital_gain_db = value,
            Set::FftFormat => a.fft_uint8 = value == FORMAT_UINT8,
            Set::FftFrequency => {
                a.fft_hz = value;
                a.sync_due = true;
            }
            Set::FftDecimation => {
                a.fft_decimation = value.min(stages(stream.sample_rate()));
                a.sync_due = true;
            }
            Set::FftDbOffset => a.fft_db_offset = value as i32,
            Set::FftDbRange => a.fft_db_range = value.max(1),
            Set::FftDisplayPixels => a.fft_pixels = value.max(1),
        }
    }
    gone.store(true, Ordering::SeqCst);
}

struct Fft {
    spectrum: Spectrum,
    pixels: usize,
    rate: f64,
    fed: f64,
}

impl Fft {
    fn new(pixels: u32, rate: f64) -> Self {
        let size = (pixels as usize).next_power_of_two().clamp(FFT_MIN, FFT_MAX);
        Self { spectrum: Spectrum::new(size), pixels: pixels as usize, rate, fed: 0.0 }
    }

    fn feed(&mut self, iq: &[C32]) -> bool {
        self.spectrum.process(iq);
        self.fed += iq.len() as f64;
        if self.fed < self.rate / FFT_FPS {
            return false;
        }
        self.fed = 0.0;
        true
    }

    fn frame(&mut self, db_offset: i32, db_range: u32, out: &mut Vec<u8>) {
        let frame = self.spectrum.take(&[Detector::Average]);
        let bins = Detector::Average.of(&frame);
        let range = db_range as f32;
        out.clear();
        for p in 0..self.pixels {
            let lo = p * bins.len() / self.pixels;
            let hi = ((p + 1) * bins.len() / self.pixels).max(lo + 1).min(bins.len());
            let db = bins[lo..hi].iter().copied().fold(f32::MIN, f32::max);
            let level = (db + db_offset as f32 + range) / range;
            out.push((level * 255.0).round().clamp(0.0, 255.0) as u8);
        }
    }
}

fn send(
    mut wire: Wire,
    stream: &Arc<Stream>,
    asked: &Mutex<Asked>,
    gone: &AtomicBool,
) -> std::io::Result<()> {
    let rate = stream.sample_rate();
    wire.send(MSG_DEVICE_INFO, 0, STREAM_STATUS, &info(stream).encode())?;
    let first =
        asked.lock().map(|a| sync(stream, &a)).map_err(|_| std::io::Error::other("lock"))?;
    wire.send(MSG_CLIENT_SYNC, 0, STREAM_STATUS, &first.encode())?;
    let mut tap = stream.tap();
    let (mut iq_cut, mut fft_cut) = (Cut::new(), Cut::new());
    let (mut iq, mut body) = (Vec::new(), Vec::new());
    let mut fft: Option<Fft> = None;
    let mut last_sync = first;
    while !gone.load(Ordering::SeqCst) && stream.sample_rate() == rate {
        let tapped = tap.next(IDLE);
        let a = match asked.lock() {
            Ok(mut a) => {
                let now = a.clone();
                a.sync_due = false;
                a.pongs.clear();
                now
            }
            Err(_) => return Ok(()),
        };
        for pong in &a.pongs {
            wire.send(MSG_PONG, 0, STREAM_STATUS, pong)?;
        }
        let now = sync(stream, &a);
        if a.sync_due || now != last_sync {
            wire.send(MSG_CLIENT_SYNC, 0, STREAM_STATUS, &now.encode())?;
            last_sync = now;
        }
        let block = match tapped {
            Tapped::Block(b) => b,
            Tapped::Closed => return Ok(()),
            Tapped::Retuned(_) | Tapped::Idle => continue,
        };
        let span = (stream.center_hz() as f64, stream.sample_rate() as f64);
        if a.wants_iq() {
            let rate = span.1 / (1u64 << a.iq_decimation) as f64;
            iq.clear();
            iq_cut.run(&block, span, (now.iq_center as f64, rate), &mut iq);
            let (gain, flags) = match a.format {
                Format::Float => (1.0, 0),
                _ => (10f32.powf(a.digital_gain_db as f32 / 20.0), a.digital_gain_db & 0xffff),
            };
            for chunk in iq.chunks(CHUNK_SAMPLES) {
                body.clear();
                a.format.encode(chunk, gain, &mut body);
                wire.send(a.format.message(), flags, STREAM_IQ, &body)?;
            }
        }
        if a.wants_fft() {
            let rate = span.1 / (1u64 << a.fft_decimation) as f64;
            let stale =
                fft.as_ref().is_none_or(|f| f.pixels != a.fft_pixels as usize || f.rate != rate);
            if stale {
                fft = Some(Fft::new(a.fft_pixels, rate));
            }
            iq.clear();
            fft_cut.run(&block, span, (now.fft_center as f64, rate), &mut iq);
            if let Some(f) = fft.as_mut()
                && f.feed(&iq)
            {
                f.frame(a.fft_db_offset, a.fft_db_range, &mut body);
                wire.send(MSG_UINT8_FFT, 0, STREAM_FFT, &body)?;
            }
        }
    }
    Ok(())
}
