//! A radio on the other end of a network socket, spoken to with iqstream.
//!
//! An iqstream server owns one tuner and fans its samples out to many readers:
//! control over TCP, samples over UDP, each subscriber choosing its own bit
//! depth and compression. That makes a receiver on a mast, a Pi in a loft, or
//! a dongle already claimed by an ADS-B decoder usable from here.
//!
//! # What this device cannot do
//!
//! The tuner is pinned by whoever feeds the server, so the centre frequency
//! and the sample rate are readings rather than settings. [`Device::set_center`]
//! and [`Device::set_gain`] accept and ignore, because the caller retunes on
//! every drag of the dial and a hard error there stops the receiver dead. What
//! the stream is actually on is always what [`Device::center`] reports.
//!
//! # Threading
//!
//! The client is asynchronous and everything above the driver boundary here is
//! not, so the stream gets a thread with a single-threaded tokio runtime on it
//! and hands decoded buffers over a bounded channel, the same shape the USB
//! drivers use.

use common::device::{Device, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use iqstream::client::{ClientConfig, IqStream};
use iqstream::proto::Codec;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

/// The port `iqstreamd` listens on for control connections.
pub const DEFAULT_PORT: u16 = 1234;

/// How long to wait for a server to answer before calling it unreachable.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(3);

/// Blocks queued for the consumer. A block is tens of milliseconds, so this is
/// a couple of seconds of slack before the oldest are dropped.
const QUEUE_DEPTH: usize = 64;

/// Bits per I or Q value asked of the server.
///
/// Eight is the dongle's own resolution: fewer bits shrink the stream but cost
/// decodes, and the receiver here is doing more than counting ADS-B messages.
const BITS: u8 = 8;

/// Add the default port to a bare host, and reject what is not an address.
pub fn parse_addr(s: &str) -> Option<String> {
    let s = s.trim();
    if s.is_empty() || s.contains(char::is_whitespace) {
        return None;
    }
    // A bracketed IPv6 literal already carries its own colons.
    if s.starts_with('[') {
        return Some(if s.ends_with(']') { format!("{s}:{DEFAULT_PORT}") } else { s.to_string() });
    }
    // More than one colon and no brackets is a bare IPv6 address, whose last
    // group would otherwise read as a port.
    if s.matches(':').count() > 1 {
        return Some(format!("[{s}]:{DEFAULT_PORT}"));
    }
    match s.rsplit_once(':') {
        Some((host, port)) if !host.is_empty() && port.parse::<u16>().is_ok() => Some(s.to_string()),
        _ => Some(format!("{s}:{DEFAULT_PORT}")),
    }
}

/// What a server says about its stream, before anything subscribes for real.
#[derive(Clone, Debug, PartialEq)]
pub struct Probe {
    pub addr: String,
    pub center: Hz,
    pub rate: Sps,
    /// Gain the source was started with, when it was told.
    pub gain_db: Option<f32>,
    /// Whether the server will accept a retune. False for every server that
    /// takes its samples from another process's tuner, which is all of them.
    pub tunable: bool,
}

fn runtime() -> Result<tokio::runtime::Runtime> {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .map_err(|e| Error::other(format!("tokio runtime: {e}")))
}

fn config(name: &str) -> ClientConfig {
    ClientConfig {
        name: name.to_string(),
        bits: BITS,
        codec: Codec::Zstd,
        // Level 1 already reaches this data's entropy bound; higher only
        // spends the server's CPU.
        level: 1,
        // A gap in the samples is padded with mid scale rather than closed up,
        // so the timebase stays true and a loss shows as silence instead of
        // sliding everything after it.
        pad_gaps: true,
        ..Default::default()
    }
}

/// Ask a server what it is streaming, then disconnect.
///
/// Used to build the device list: the centre frequency and the sample rate
/// belong to the server, so they have to be read before anything can offer a
/// span list or draw a spectrum.
pub fn probe(addr: &str) -> Result<Probe> {
    let addr = parse_addr(addr).ok_or(Error::NoDevice)?;
    let rt = runtime()?;
    let a = addr.clone();
    rt.block_on(async move {
        let connect = IqStream::connect(a.as_str(), config("waveshark probe"));
        let mut stream = tokio::time::timeout(CONNECT_TIMEOUT, connect)
            .await
            .map_err(|_| Error::other(format!("{a} did not answer")))?
            .map_err(|e| Error::other(format!("{a}: {e}")))?;
        let info = *stream.info();
        let _ = stream.unsubscribe().await;
        if info.sample_rate == 0 {
            return Err(Error::other(format!("{a} did not say what rate it is running at")));
        }
        Ok(Probe {
            addr: a,
            center: Hz(info.center_hz),
            rate: Sps(info.sample_rate as u64),
            gain_db: info.gain_db,
            tunable: info.tunable,
        })
    })
}

pub struct IqNet {
    addr: String,
    info: DeviceInfo,
    center: Hz,
    rate: Sps,
    streaming: Arc<AtomicBool>,
}

impl IqNet {
    /// Connect once to learn what the server is streaming, and keep the
    /// address for the subscription the stream will open.
    pub fn open(addr: &str) -> Result<Self> {
        let p = probe(addr)?;
        Ok(Self::from_probe(&p))
    }

    pub fn from_probe(p: &Probe) -> Self {
        let label = match p.gain_db {
            Some(g) => format!("iqstream {} at {g:.1} dB", p.addr),
            None => format!("iqstream {}", p.addr),
        };
        let info = DeviceInfo {
            kind: DriverKind::IqStream,
            id: format!("iqstream:{}", p.addr),
            label,
            tuner: "remote".to_string(),
            // One point wide, and true: the frequency is the source process's
            // to choose. A range covering the band would let the dial move to
            // somewhere the samples do not come from.
            ranges: vec![TunerRange { range: p.center..=p.center, label: "pinned" }],
            rates: vec![p.rate],
            rate_range: p.rate..=p.rate,
            // Gain belongs to whoever owns the tuner. Offering a slider that
            // moves nothing would be worse than offering none.
            gain_stages: Vec::new(),
            native_format: SampleFormat::Cu8,
            // Unknown from here: the server does not say what is feeding it.
            // The usual answer is an RTL-SDR, so assume its filtering.
            usable_bandwidth_ratio: 0.80,
        };
        Self {
            addr: p.addr.clone(),
            info,
            center: p.center,
            rate: p.rate,
            streaming: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn address(&self) -> &str {
        &self.addr
    }
}

impl Device for IqNet {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }

    /// Accepted and ignored. The tuner is pinned by the process feeding the
    /// server, and the dial issues one of these per frame while it is dragged.
    fn set_center(&mut self, _f: Hz) -> Result<()> {
        Ok(())
    }

    fn center(&self) -> Hz {
        self.center
    }

    fn set_rate(&mut self, r: Sps) -> Result<()> {
        if r != self.rate {
            return Err(Error::RateUnsupported { req: r });
        }
        Ok(())
    }

    fn rate(&self) -> Sps {
        self.rate
    }

    /// Also accepted and ignored: the receiver sets a gain on startup and a
    /// refusal there would stop it before the first sample arrived.
    fn set_gain(&mut self, _stage: &str, _mode: GainMode) -> Result<()> {
        Ok(())
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        let (stop_tx, stop_rx) = tokio::sync::watch::channel(false);

        let addr = self.addr.clone();
        let (center, rate) = (self.center, self.rate);
        let streaming = self.streaming.clone();
        let counted = dropped.clone();
        let join = std::thread::Builder::new()
            .name("iqstream-rx".into())
            .spawn(move || {
                match runtime() {
                    Ok(rt) => {
                        if let Err(e) =
                            rt.block_on(pump(addr, center, rate, tx, counted, stop_rx))
                        {
                            tracing::warn!("iqstream: {e}");
                        }
                    }
                    Err(e) => tracing::error!("{e}"),
                }
                streaming.store(false, Ordering::SeqCst);
            })
            .map_err(|e| Error::other(format!("spawn rx thread: {e}")))?;

        Ok(Box::new(NetStream { rx, dropped, stop: stop_tx, join: Some(join) }))
    }
}

/// Subscribe, decode, and hand blocks over until told to stop.
async fn pump(
    addr: String,
    center: Hz,
    rate: Sps,
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    mut stop: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    let connect = IqStream::connect(addr.as_str(), config("waveshark"));
    let mut stream = tokio::time::timeout(CONNECT_TIMEOUT, connect)
        .await
        .map_err(|_| Error::other(format!("{addr} did not answer")))?
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;

    let mut samples = Vec::new();
    loop {
        let block = tokio::select! {
            // next_block is cancellation safe, so losing this branch to the
            // stop signal loses nothing that had arrived.
            _ = stop.changed() => break,
            b = stream.next_block() => match b {
                Ok(Some(b)) => b,
                // The server closed the control connection, which is how it
                // ends a subscription.
                Ok(None) => break,
                Err(e) => return Err(Error::other(format!("{addr}: {e}"))),
            },
        };

        samples.clear();
        SampleFormat::Cu8.convert(&block.samples, &mut samples);
        if block.padded_before > 0 {
            dropped.fetch_add(block.padded_before, Ordering::Relaxed);
        }
        // The block's index counts real samples, and padding for a loss was
        // put in front of them, so the buffer starts that much earlier.
        let seq = block.sample_index.saturating_sub(block.padded_before);
        let buf = IqBuf::new(std::mem::take(&mut samples), center, rate, seq);
        match tx.try_send(buf) {
            Ok(()) => {}
            // A consumer that cannot keep up loses the oldest samples rather
            // than stalling the socket, which would make the server drop them
            // for us and take the control connection's keepalive with it.
            Err(TrySendError::Full(buf)) => {
                dropped.fetch_add(buf.len() as u64, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => break,
        }
    }
    let _ = stream.unsubscribe().await;
    Ok(())
}

struct NetStream {
    rx: Receiver<IqBuf>,
    dropped: Arc<AtomicU64>,
    stop: tokio::sync::watch::Sender<bool>,
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
        let _ = self.stop.send(true);
    }
}

impl Drop for NetStream {
    fn drop(&mut self) {
        self.stop();
        // Drain so the pump's try_send cannot be holding a full channel while
        // the thread is being waited on.
        while self.rx.try_recv().is_ok() {}
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_bare_host_gets_the_default_port() {
        assert_eq!(parse_addr("radarpi").as_deref(), Some("radarpi:1234"));
        assert_eq!(parse_addr("10.0.0.5").as_deref(), Some("10.0.0.5:1234"));
        assert_eq!(parse_addr(" radarpi:9000 ").as_deref(), Some("radarpi:9000"));
        assert_eq!(parse_addr(""), None);
        assert_eq!(parse_addr("two words"), None);
    }

    #[test]
    fn an_ipv6_address_keeps_its_own_colons() {
        // Splitting on the last colon would read fd00::1 as host "fd00:" on
        // port ":1", which resolves to nothing and reports the wrong reason.
        assert_eq!(parse_addr("fd00::1").as_deref(), Some("[fd00::1]:1234"));
        assert_eq!(parse_addr("[fd00::1]:9000").as_deref(), Some("[fd00::1]:9000"));
        assert_eq!(parse_addr("[fd00::1]").as_deref(), Some("[fd00::1]:1234"));
    }

    #[test]
    fn a_pinned_stream_refuses_a_rate_it_is_not_running() {
        // The span list is built from the device's own rate, so anything else
        // arriving here is a stale setting from another radio and taking it
        // would label every frequency on screen wrongly.
        let mut d = IqNet::from_probe(&Probe {
            addr: "example:1234".into(),
            center: Hz::mhz(1090),
            rate: Sps(2_400_000),
            gain_db: Some(49.6),
            tunable: false,
        });
        assert!(d.set_rate(Sps(2_400_000)).is_ok());
        assert!(d.set_rate(Sps(2_048_000)).is_err());
        // A retune is accepted and does nothing, because the dial sends one
        // per frame and an error there kills the receiver's thread.
        assert!(d.set_center(Hz::mhz(433)).is_ok());
        assert_eq!(d.center(), Hz::mhz(1090));
    }
}
