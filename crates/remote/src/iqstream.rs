//! A radio on the other end of a network socket, spoken to with iqstream.
//!
//! An iqstream server owns one tuner and fans its samples out to many readers:
//! control over TCP, samples over UDP, each subscriber choosing its own bit
//! depth and compression. That makes a receiver on a mast, a Pi in a loft, or
//! a dongle already claimed by an ADS-B decoder usable from here.
//!
//! # What this device can and cannot do
//!
//! The sample rate is always the server's: it is one stream fanned out, and no
//! reader may reshape it for the others. [`Device::set_gain`] accepts and
//! ignores, because gain belongs to whoever owns the tuner.
//!
//! The frequency depends on what the server said. A server sharing somebody
//! else's dongle pins its tuner and offers no range, so the dial is one point
//! wide and [`Device::set_center`] accepts and ignores: the caller retunes on
//! every drag and a hard error there would stop the receiver dead. A server
//! that offered its dial takes the range it named, and a retune goes out as an
//! [`iqstream::proto::msg::TUNE`].
//!
//! Either way [`Device::center`] reports where the stream actually is, which
//! after a tune is where the far end landed rather than what was asked for.
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

/// What a tunable server is asked for, and where it says it landed.
///
/// The dial is set on this thread and the socket lives on another, so the two
/// pass a frequency each way: `want` is written by [`Device::set_center`] and
/// read by the pump, `at` is written by the pump from every `TUNED` and read
/// back by [`Device::center`]. A drag writes `want` a frame at a time and only
/// the last matters, which is exactly what a watch channel keeps.
struct Dial {
    want: tokio::sync::watch::Sender<u64>,
    at: Arc<AtomicU64>,
}

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
            tune_range: info.tune_range_hz.map(|(lo, hi)| Hz(lo)..=Hz(hi)),
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
    /// Present only where the server offered its dial and said how far it
    /// reaches. None is a pinned tuner, and every retune is then ignored.
    dial: Option<Dial>,
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
        // A dial needs somewhere to go, so a server that takes a tune but
        // named no range is treated as pinned: it can still be asked over the
        // wire, but nothing here can offer a slider with no ends.
        let movable = p.tune_range.clone().filter(|_| p.tunable);
        let ranges = match &movable {
            Some(r) => vec![TunerRange { range: r.clone(), label: "remote" }],
            // One point wide, and true: the frequency is the source process's
            // to choose. A range covering the band would let the dial move to
            // somewhere the samples do not come from.
            None => vec![TunerRange { range: center..=center, label: "pinned" }],
        };
        let info = DeviceInfo {
            kind: DriverKind::Network,
            id: format!("iqstream:{}", p.addr),
            label,
            tuner: "remote".to_string(),
            ranges,
            rates: vec![rate],
            rate_range: rate..=rate,
            // Gain belongs to whoever owns the tuner. Offering a slider that
            // moves nothing would be worse than offering none.
            gain_stages: Vec::new(),
            native_format: SampleFormat::Cu8,
            // Unknown from here: the server does not say what is feeding it.
            // The usual answer is an RTL-SDR, so assume its filtering.
            usable_bandwidth_ratio: 0.80,
            tunable: movable.is_some(),
            tx: None,
        };
        let dial = movable.map(|_| Dial {
            want: tokio::sync::watch::channel(center.0).0,
            at: Arc::new(AtomicU64::new(center.0)),
        });
        Self {
            addr: p.addr.clone(),
            info,
            center,
            rate,
            streaming: Arc::new(AtomicBool::new(false)),
            tuning: Default::default(),
            dial,
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

    /// Asked of the far end where it offered its dial, and otherwise accepted
    /// and ignored: the tuner is pinned by the process feeding the server, and
    /// the dial issues one of these per frame while it is dragged.
    ///
    /// Even where it is asked, this returns before the tuner has moved. The
    /// answer arrives as a `TUNED` and shows up in [`Device::center`], because
    /// a far end steps in units of its own and blocking the dial on a network
    /// round trip would make it stutter.
    fn set_center(&mut self, f: Hz) -> Result<()> {
        if let Some(d) = &self.dial {
            let _ = d.want.send(f.0);
        }
        Ok(())
    }

    fn center(&self) -> Hz {
        match &self.dial {
            Some(d) => Hz(d.at.load(Ordering::Relaxed)),
            None => self.center,
        }
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
        let dial = self.dial.as_ref().map(|d| (d.want.subscribe(), d.at.clone()));
        let join = std::thread::Builder::new()
            .name("iqstream-rx".into())
            .spawn(move || {
                match runtime() {
                    Ok(rt) => {
                        let f = pump(addr, center, rate, tx, counted, stop_rx, dial);
                        if let Err(e) = rt.block_on(f) {
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
#[allow(clippy::too_many_arguments)]
async fn pump(
    addr: String,
    center: Hz,
    rate: Sps,
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    mut stop: tokio::sync::watch::Receiver<bool>,
    dial: Option<(tokio::sync::watch::Receiver<u64>, Arc<AtomicU64>)>,
) -> Result<()> {
    let connect = IqStream::connect(addr.as_str(), config("waveshark"));
    let mut stream = tokio::time::timeout(CONNECT_TIMEOUT, connect)
        .await
        .map_err(|_| Error::other(format!("{addr} did not answer")))?
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;

    let (mut want, at) = match dial {
        Some((w, at)) => (Some(w), Some(at)),
        None => (None, None),
    };
    let mut samples = Vec::new();
    loop {
        let block = tokio::select! {
            // next_block is cancellation safe, so losing this branch to the
            // stop signal loses nothing that had arrived.
            _ = stop.changed() => break,
            // Only ever Some where the far end offered its dial, and never
            // ready otherwise, so a pinned stream never reaches this arm.
            _ = async { want.as_mut().unwrap().changed().await }, if want.is_some() => {
                let hz = *want.as_mut().unwrap().borrow_and_update();
                // A refusal is logged and dropped rather than ending the
                // stream: the samples keep coming from where the tuner is.
                if let Err(e) = stream.tune(hz).await {
                    tracing::debug!("iqstream: {e}");
                }
                continue;
            }
            b = stream.next_block() => match b {
                Ok(Some(b)) => b,
                // The server closed the control connection, which is how it
                // ends a subscription.
                Ok(None) => break,
                Err(e) => return Err(Error::other(format!("{addr}: {e}"))),
            },
        };
        if let Some(at) = &at {
            at.store(block.center_hz, Ordering::Relaxed);
        }

        samples.clear();
        SampleFormat::Cu8.convert(&block.samples, &mut samples);
        if block.padded_before > 0 {
            dropped.fetch_add(block.padded_before, Ordering::Relaxed);
        }
        // The block's index counts real samples, and padding for a loss was
        // put in front of them, so the buffer starts that much earlier.
        let seq = block.sample_index.saturating_sub(block.padded_before);
        // The block's own centre, not the one subscribed at: after a retune
        // the two differ, and everything downstream labels its spectrum and
        // its packets from this.
        let heard = match at.is_some() {
            true => Hz(block.center_hz),
            false => center,
        };
        let buf = IqBuf::new(std::mem::take(&mut samples), heard, rate, seq);
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

    fn probe(tunable: bool, tune_range: Option<std::ops::RangeInclusive<Hz>>) -> Probe {
        Probe {
            proto: Proto::IqStream,
            addr: "example:1234".into(),
            center: Some(Hz::mhz(1090)),
            rate: Some(Sps(2_400_000)),
            gain_db: Some(49.6),
            tunable,
            tune_range,
            tuner: "remote".into(),
        }
    }

    #[test]
    fn a_pinned_stream_refuses_a_rate_it_is_not_running() {
        // The span list is built from the device's own rate, so anything else
        // arriving here is a stale setting from another radio and taking it
        // would label every frequency on screen wrongly.
        let mut d = Device::from_probe(&probe(false, None));
        assert!(d.set_rate(Sps(2_400_000)).is_ok());
        assert!(d.set_rate(Sps(2_048_000)).is_err());
        // A retune is accepted and does nothing, because the dial sends one
        // per frame and an error there kills the receiver's thread.
        assert!(d.set_center(Hz::mhz(433)).is_ok());
        assert_eq!(d.center(), Hz::mhz(1090));
        assert!(!d.info().tunable);
        assert_eq!(d.info().ranges.len(), 1);
        assert_eq!(*d.info().ranges[0].range.start(), Hz::mhz(1090));
        assert_eq!(*d.info().ranges[0].range.end(), Hz::mhz(1090));
        assert_eq!(d.info().ranges[0].label, "pinned");
    }

    /// A server that offered its dial gets a real range and a real retune. The
    /// reading does not move until the far end says where it landed, which is
    /// why the dial can only be asked and not set.
    #[test]
    fn an_offered_dial_takes_the_range_the_server_named() {
        let mut d = Device::from_probe(&probe(true, Some(Hz::mhz(24)..=Hz::mhz(1766))));
        assert!(d.info().tunable);
        assert_eq!(d.info().ranges.len(), 1);
        assert_eq!(*d.info().ranges[0].range.start(), Hz::mhz(24));
        assert_eq!(*d.info().ranges[0].range.end(), Hz::mhz(1766));
        assert_eq!(d.info().ranges[0].label, "remote");
        assert!(d.set_center(Hz::mhz(433)).is_ok());
        assert_eq!(d.center(), Hz::mhz(1090), "not until the far end answers");
    }

    /// Tunable with no range is treated as pinned. A dial with no ends cannot
    /// be drawn, and offering one that runs from zero to zero would let an
    /// operator ask for somewhere the samples never come from.
    #[test]
    fn a_server_that_named_no_range_is_pinned() {
        let d = Device::from_probe(&probe(true, None));
        assert!(!d.info().tunable);
        assert_eq!(d.info().ranges[0].label, "pinned");
    }
}
