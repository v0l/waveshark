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

use crate::{CONNECT_TIMEOUT, Probe, Proto, QUEUE_DEPTH};
use common::device::{
    Device as DeviceTrait, DeviceInfo, DriverKind, GainMode, RxStream, TunerRange,
};
use common::{Error, Hz, IqBuf, Result, SampleFormat, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use iqstream::client::{ClientConfig, IqStream};
use iqstream::proto::Codec;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

/// Bits per I or Q value asked of the server.
///
/// Eight is the dongle's own resolution: fewer bits shrink the stream but cost
/// decodes, and the receiver here is doing more than counting ADS-B messages.
const BITS: u8 = 8;

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
    let addr = Proto::IqStream.parse_addr(addr).ok_or(Error::NoDevice)?;
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
            proto: Proto::IqStream,
            addr: a,
            center: Some(Hz(info.center_hz)),
            rate: Some(Sps(info.sample_rate as u64)),
            gain_db: info.gain_db,
            // The server's own word, and not the same question as whether the
            // protocol can carry a retune: nothing serving a shared tuner
            // says yes.
            tunable: info.tunable,
            tuner: "remote".to_string(),
        })
    })
}

pub struct Device {
    addr: String,
    info: DeviceInfo,
    /// The converter on the cable and the reference correction, which the
    /// `Device` trait does the arithmetic with.
    tuning: common::Tuning,
    center: Hz,
    rate: Sps,
    streaming: Arc<AtomicBool>,
}

impl Device {
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
        // A server that did not say is still streaming something; the entry
        // has to carry a number, and the dial cannot move it either way.
        let (center, rate) = (p.center.unwrap_or(Hz(0)), p.rate.unwrap_or(Sps(1)));
        let info = DeviceInfo {
            kind: DriverKind::Network,
            id: format!("iqstream:{}", p.addr),
            label,
            tuner: "remote".to_string(),
            // One point wide, and true: the frequency is the source process's
            // to choose. A range covering the band would let the dial move to
            // somewhere the samples do not come from.
            ranges: vec![TunerRange { range: center..=center, label: "pinned" }],
            rates: vec![rate],
            rate_range: rate..=rate,
            // Gain belongs to whoever owns the tuner. Offering a slider that
            // moves nothing would be worse than offering none.
            gain_stages: Vec::new(),
            native_format: SampleFormat::Cu8,
            // Unknown from here: the server does not say what is feeding it.
            // The usual answer is an RTL-SDR, so assume its filtering.
            usable_bandwidth_ratio: 0.80,
            tunable: false,
            tx: None,
        };
        Self {
            addr: p.addr.clone(),
            info,
            center,
            rate,
            streaming: Arc::new(AtomicBool::new(false)),
            tuning: Default::default(),
        }
    }

    pub fn address(&self) -> &str {
        &self.addr
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
                        if let Err(e) = rt.block_on(pump(addr, center, rate, tx, counted, stop_rx))
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
    fn a_pinned_stream_refuses_a_rate_it_is_not_running() {
        // The span list is built from the device's own rate, so anything else
        // arriving here is a stale setting from another radio and taking it
        // would label every frequency on screen wrongly.
        let mut d = Device::from_probe(&Probe {
            proto: Proto::IqStream,
            addr: "example:1234".into(),
            center: Some(Hz::mhz(1090)),
            rate: Some(Sps(2_400_000)),
            gain_db: Some(49.6),
            tunable: false,
            tuner: "remote".into(),
        });
        assert!(d.set_rate(Sps(2_400_000)).is_ok());
        assert!(d.set_rate(Sps(2_048_000)).is_err());
        // A retune is accepted and does nothing, because the dial sends one
        // per frame and an error there kills the receiver's thread.
        assert!(d.set_center(Hz::mhz(433)).is_ok());
        assert_eq!(d.center(), Hz::mhz(1090));
        assert!(!d.info().tunable);
    }
}
