//! A radio on the other end of a network socket, spoken to with iqstream.
//!
//! An iqstream server carries one tuner or several and fans each one's samples
//! out to many readers: control over TCP, samples over UDP, each subscriber
//! choosing its own bit depth and compression. That makes a receiver on a
//! mast, a Pi in a loft, or a dongle already claimed by an ADS-B decoder
//! usable from here.
//!
//! A server carrying several is several radios in the list, one per tuner,
//! addressed `host:port#2`. Each opens its own subscription, so two of them
//! can be read at once and each is tuned on its own.
//!
//! # What this device can and cannot do
//!
//! The sample rate is always the server's: it is one stream fanned out, and no
//! reader may reshape it for the others. The gain and the switches belong to
//! whoever owns the tuner: a server that offered its radio takes them from
//! here and says what they became, and one sharing a tuner somebody else is
//! listening to offers none, so there is no control to move.
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
use iqstream::{Setting, SettingKind, SettingValue};
use std::sync::Arc;
use std::sync::Mutex;
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

/// What the far end is set to, and the requests going the other way.
///
/// The controls are read on this thread and set on it, and the socket is on
/// another, so a request is queued rather than carried out: the answer is the
/// far end saying what the radio became, which lands in `now`.
struct Controls {
    /// What the far end last said it is set to. Written by the pump from
    /// every stream change, read by [`Device::gains`] and its neighbours.
    now: Arc<Mutex<Vec<Setting>>>,
    asks: tokio::sync::mpsc::UnboundedSender<(String, SettingValue)>,
    /// Taken by the stream when it starts. A request made before anything is
    /// streaming waits in the channel rather than being lost.
    queue: Mutex<Option<tokio::sync::mpsc::UnboundedReceiver<(String, SettingValue)>>>,
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

fn config(name: &str, stream: Option<u16>) -> ClientConfig {
    ClientConfig {
        name: name.to_string(),
        stream,
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
/// span list or draw a spectrum. An address naming a tuner describes that one;
/// one naming none describes the first, which is the whole of what a
/// single-tuner server has.
pub fn probe(addr: &str) -> Result<Probe> {
    let wanted = crate::split_stream(addr).1;
    let found = probe_all(addr)?;
    match wanted {
        Some(id) => found
            .into_iter()
            .find(|p| crate::split_stream(&p.addr).1 == Some(id))
            .ok_or_else(|| Error::other(format!("that server has no tuner {id}"))),
        None => found.into_iter().next().ok_or(Error::NoDevice),
    }
}

/// Every tuner a server is offering.
///
/// One connection answers for all of them, because the welcome lists them:
/// nothing subscribes here, so asking costs a handshake however many tuners
/// come back.
pub fn probe_all(addr: &str) -> Result<Vec<Probe>> {
    let addr = Proto::IqStream.parse_addr(addr).ok_or(Error::NoDevice)?;
    let (host, _) = crate::split_stream(&addr);
    let host = host.to_string();
    let rt = runtime()?;
    rt.block_on(async move {
        let listing = iqstream::list(host.as_str(), "waveshark probe");
        let streams = tokio::time::timeout(CONNECT_TIMEOUT, listing)
            .await
            .map_err(|_| Error::other(format!("{host} did not answer")))?
            .map_err(|e| Error::other(format!("{host}: {e}")))?;
        let named: Vec<Probe> = streams
            .into_iter()
            .filter(|s| s.sample_rate > 0)
            .map(|s| Probe {
                proto: Proto::IqStream,
                // The tuner travels with the address, so an entry opened from
                // the list reaches the one it was made from.
                addr: format!("{host}#{}", s.id),
                center: Some(Hz(s.center_hz)),
                rate: Some(Sps(s.sample_rate as u64)),
                gain_db: s.gain_db,
                name: s.name,
                settings: s.settings,
                // The server's own word, and not the same question as whether
                // the protocol can carry a retune: nothing serving a tuner
                // somebody else is listening to says yes.
                tunable: s.tunable,
                tune_range: s.tune_range_hz.map(|(lo, hi)| Hz(lo)..=Hz(hi)),
                tuner: "remote".to_string(),
            })
            .collect();
        match named.is_empty() {
            true => Err(Error::other(format!("{host} did not say what rate it is running at"))),
            false => Ok(named),
        }
    })
}

pub struct Device {
    addr: String,
    /// Which tuner of that server's, taken off the address.
    stream: Option<u16>,
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
    controls: Controls,
    /// Whether the far end takes a setting from here at all.
    settable: bool,
}

/// A gain stage out of what the far end said about one of its own.
///
/// Only a gain becomes a stage: a switch and an antenna port are the driver's
/// other two lists, and a stage made out of either would be a slider on a
/// thing that is not one.
fn gain_stage(s: &Setting) -> Option<common::device::GainStage> {
    if s.kind != SettingKind::Gain {
        return None;
    }
    let (lo, hi) = s.range_db.unwrap_or((0.0, 50.0));
    Some(common::device::GainStage {
        name: s.name.clone(),
        label: s.label.clone(),
        range: lo..=hi,
        // Unknown from here: the far end says how far a gain goes and not
        // which steps its hardware has, so it is asked for what was wanted
        // and reports back what it managed.
        values: Vec::new(),
        step: 0.0,
        auto: true,
    })
}

impl Device {
    /// Connect once to learn what the server is streaming, and keep the
    /// address for the subscription the stream will open.
    pub fn open(addr: &str) -> Result<Self> {
        let p = probe(addr)?;
        Ok(Self::from_probe(&p))
    }

    pub fn from_probe(p: &Probe) -> Self {
        let what = match p.name.is_empty() {
            true => p.addr.clone(),
            false => format!("{} on {}", p.name, p.addr),
        };
        let label = match p.gain_db {
            Some(g) => format!("iqstream {what} at {g:.1} dB"),
            None => format!("iqstream {what}"),
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
            // Gain belongs to whoever owns the tuner, so the stages are the
            // ones it offered, and none at all from a server sharing a radio
            // somebody else is listening to: a slider that moves nothing
            // would be worse than no slider.
            gain_stages: match p.tunable {
                true => p.settings.iter().filter_map(gain_stage).collect(),
                false => Vec::new(),
            },
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
        let (asks, queue) = tokio::sync::mpsc::unbounded_channel();
        Self {
            addr: p.addr.clone(),
            stream: crate::split_stream(&p.addr).1,
            info,
            center,
            rate,
            streaming: Arc::new(AtomicBool::new(false)),
            tuning: Default::default(),
            dial,
            controls: Controls {
                now: Arc::new(Mutex::new(p.settings.clone())),
                asks,
                queue: Mutex::new(Some(queue)),
            },
            // A radio somebody else is listening to takes nothing from here,
            // and the receiver sets a gain on startup: without this every
            // shared stream would open with a refusal.
            settable: p.tunable,
        }
    }

    /// What the far end says it is set to now.
    fn setting(&self, name: &str) -> Option<Setting> {
        let held = self.controls.now.lock().ok()?;
        held.iter().find(|s| s.name == name).cloned()
    }

    /// Queue a request for the pump to put on the wire, or refuse it where
    /// the far end is not offering that control.
    fn ask(&self, name: &str, value: SettingValue) -> Result<()> {
        if !self.settable {
            return Err(Error::other("the far end owns this radio's settings"));
        }
        if self.setting(name).is_none() {
            return Err(Error::other(format!("the far end has no {name} setting")));
        }
        self.controls.asks.send((name.to_string(), value)).map_err(|_| Error::Disconnected)
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

    /// Asked of the far end where it offered its radio, and otherwise
    /// accepted and ignored: the receiver sets a gain on startup and a
    /// refusal there would stop it before the first sample arrived.
    fn set_gain(&mut self, stage: &str, mode: GainMode) -> Result<()> {
        if !self.settable {
            return Ok(());
        }
        let value = match mode {
            GainMode::Auto => SettingValue::Auto,
            GainMode::Manual(db) => SettingValue::Gain(db),
        };
        // A stage the far end does not have is not a fault either: a receiver
        // opening on a stale setting would otherwise stop dead.
        let _ = self.ask(stage, value);
        Ok(())
    }

    /// What the far end last said, not what was asked for: a request takes a
    /// round trip and a driver snaps a gain to its own step.
    fn gains(&self) -> Vec<(String, GainMode)> {
        self.controls
            .now
            .lock()
            .map(|now| {
                now.iter()
                    .filter(|s| s.kind == SettingKind::Gain)
                    .map(|s| {
                        let mode = match s.value {
                            SettingValue::Gain(db) => GainMode::Manual(db),
                            _ => GainMode::Auto,
                        };
                        (s.name.clone(), mode)
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    fn toggles(&self) -> Vec<common::device::Toggle> {
        if !self.settable {
            return Vec::new();
        }
        self.controls
            .now
            .lock()
            .map(|now| {
                now.iter()
                    .filter_map(|s| match s.value {
                        SettingValue::Switch(on) => Some(common::device::Toggle {
                            name: s.name.clone(),
                            label: s.label.clone(),
                            help: "on the radio at the far end".into(),
                            on,
                        }),
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    fn set_toggle(&mut self, name: &str, on: bool) -> Result<()> {
        self.ask(name, SettingValue::Switch(on))
    }

    fn choices(&self) -> Vec<common::device::Choice> {
        if !self.settable {
            return Vec::new();
        }
        self.controls
            .now
            .lock()
            .map(|now| {
                now.iter()
                    .filter_map(|s| match &s.value {
                        SettingValue::Choice(selected) => Some(common::device::Choice {
                            name: s.name.clone(),
                            label: s.label.clone(),
                            help: "on the radio at the far end".into(),
                            options: s.options.clone(),
                            selected: selected.clone(),
                        }),
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    fn set_choice(&mut self, name: &str, value: &str) -> Result<()> {
        self.ask(name, SettingValue::Choice(value.to_string()))
    }

    fn numbers(&self) -> Vec<common::device::Number> {
        if !self.settable {
            return Vec::new();
        }
        self.controls
            .now
            .lock()
            .map(|now| {
                now.iter()
                    .filter_map(|s| match s.value {
                        SettingValue::Number(v) => Some(common::device::Number {
                            name: s.name.clone(),
                            label: s.label.clone(),
                            help: "on the radio at the far end".into(),
                            // A far end that named no ends is given the one
                            // it is already at, so a control cannot be drawn
                            // that asks for somewhere it will not go.
                            range: s.range.map(|(lo, hi)| lo..=hi).unwrap_or(v..=v),
                            step: s.step,
                            unit: s.unit.clone(),
                            value: v,
                        }),
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    fn set_number(&mut self, name: &str, value: f64) -> Result<()> {
        self.ask(name, SettingValue::Number(value))
    }

    fn start_rx(&mut self) -> Result<Box<dyn RxStream>> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let (tx, rx) = bounded::<IqBuf>(QUEUE_DEPTH);
        let dropped = Arc::new(AtomicU64::new(0));
        let (stop_tx, stop_rx) = tokio::sync::watch::channel(false);

        let addr = crate::split_stream(&self.addr).0.to_string();
        let stream = self.stream;
        let (center, rate) = (self.center, self.rate);
        let streaming = self.streaming.clone();
        let counted = dropped.clone();
        let dial = self.dial.as_ref().map(|d| (d.want.subscribe(), d.at.clone()));
        let asks = self.controls.queue.lock().ok().and_then(|mut q| q.take());
        let settings = self.controls.now.clone();
        let join = std::thread::Builder::new()
            .name("iqstream-rx".into())
            .spawn(move || {
                match runtime() {
                    Ok(rt) => {
                        let f = pump(
                            Wire { addr, stream, center, rate },
                            tx,
                            counted,
                            stop_rx,
                            dial,
                            asks,
                            settings,
                        );
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
/// Which tuner on which server, and what it was streaming when it was probed.
struct Wire {
    addr: String,
    stream: Option<u16>,
    center: Hz,
    rate: Sps,
}

async fn pump(
    wire: Wire,
    tx: Sender<IqBuf>,
    dropped: Arc<AtomicU64>,
    mut stop: tokio::sync::watch::Receiver<bool>,
    dial: Option<(tokio::sync::watch::Receiver<u64>, Arc<AtomicU64>)>,
    mut asks: Option<tokio::sync::mpsc::UnboundedReceiver<(String, SettingValue)>>,
    held: Arc<Mutex<Vec<Setting>>>,
) -> Result<()> {
    let Wire { addr, stream: which, center, rate } = wire;
    let connect = IqStream::connect(addr.as_str(), config("waveshark", which));
    let mut stream = tokio::time::timeout(CONNECT_TIMEOUT, connect)
        .await
        .map_err(|_| Error::other(format!("{addr} did not answer")))?
        .map_err(|e| Error::other(format!("{addr}: {e}")))?;

    let (mut want, at) = match dial {
        Some((w, at)) => (Some(w), Some(at)),
        None => (None, None),
    };
    let mut samples = Vec::new();
    // What the far end was set to when this subscription started. A change
    // under a running stream is worth a line in the log: nothing here can set
    // a remote gain, but a level that moved is why the noise floor did.
    let mut settings = stream.info().settings.clone();
    loop {
        let block = tokio::select! {
            // next_block is cancellation safe, so losing this branch to the
            // stop signal loses nothing that had arrived.
            _ = stop.changed() => break,
            // Only ever Some where the far end offered its dial, and never
            // ready otherwise, so a pinned stream never reaches this arm.
            // A gain, a switch or an antenna port asked of the far end. What
            // it becomes arrives back as a stream change, which is what the
            // controls above read.
            ask = async { asks.as_mut().unwrap().recv().await }, if asks.is_some() => {
                match ask {
                    Some((name, value)) => {
                        if let Err(e) = stream.set_setting(&name, value).await {
                            tracing::debug!("iqstream: {addr}: {e}");
                        }
                    }
                    // The device was dropped; the samples keep coming until
                    // whoever holds the stream stops it.
                    None => asks = None,
                }
                continue;
            }
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
        if stream.info().settings != settings {
            settings = stream.info().settings.clone();
            for s in &settings {
                tracing::info!("iqstream: {addr} {} is {}", s.label, describe(&s.value));
            }
            if let Ok(mut now) = held.lock() {
                *now = settings.clone();
            }
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

/// One setting's value, for a log line an operator reads.
fn describe(v: &iqstream::SettingValue) -> String {
    use iqstream::SettingValue as V;
    match v {
        V::Auto => "set by the far end".into(),
        V::Gain(db) => format!("{db:.1} dB"),
        V::Switch(on) => match on {
            true => "on".into(),
            false => "off".into(),
        },
        V::Choice(name) => name.clone(),
        V::Number(v) => format!("{v}"),
        V::Unknown(v) => format!("{v} (a setting this build does not know)"),
    }
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
            name: "loft".into(),
            settings: Vec::new(),
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

    fn gain(db: f32) -> Setting {
        Setting {
            name: "tuner".into(),
            label: "RF gain".into(),
            kind: SettingKind::Gain,
            value: SettingValue::Gain(db),
            range_db: Some((0.0, 49.6)),
            ..Default::default()
        }
    }

    fn bias(on: bool) -> Setting {
        Setting {
            name: "bias_t".into(),
            label: "Bias tee".into(),
            kind: SettingKind::Switch,
            value: SettingValue::Switch(on),
            ..Default::default()
        }
    }

    /// A server that offered its radio gives the receiver real controls: a
    /// gain stage with the range the far end named, and its switches.
    #[test]
    fn an_offered_radio_brings_its_controls_with_it() {
        let mut p = probe(true, Some(Hz::mhz(24)..=Hz::mhz(1766)));
        p.settings = vec![gain(32.8), bias(false)];
        let mut d = Device::from_probe(&p);

        assert_eq!(d.info().gain_stages.len(), 1, "one gain, and the bias tee is not one");
        assert_eq!(d.info().gain_stages[0].name, "tuner");
        assert_eq!(*d.info().gain_stages[0].range.end(), 49.6);
        assert_eq!(d.gains(), vec![("tuner".to_string(), GainMode::Manual(32.8))]);
        assert_eq!(d.toggles().len(), 1);
        assert!(!d.toggles()[0].on);

        // Asked for, and not yet true: the reading is what the far end last
        // said, which a request does not change.
        assert!(d.set_gain("tuner", GainMode::Manual(14.0)).is_ok());
        assert!(d.set_toggle("bias_t", true).is_ok());
        assert_eq!(d.gains(), vec![("tuner".to_string(), GainMode::Manual(32.8))]);
        assert!(!d.toggles()[0].on);

        // A control the far end has not got is refused rather than sent.
        assert!(d.set_toggle("offset_tuning", true).is_err());
        assert!(d.set_choice("antenna", "LNAW").is_err());
    }

    /// A server sharing a radio somebody else is listening to offers no
    /// controls at all, and the gain the receiver sets on startup is taken
    /// and dropped rather than refused.
    #[test]
    fn a_shared_radio_offers_nothing_to_move() {
        let mut p = probe(false, None);
        p.settings = vec![gain(32.8), bias(true)];
        let mut d = Device::from_probe(&p);
        assert!(d.info().gain_stages.is_empty());
        assert!(d.toggles().is_empty());
        assert!(d.choices().is_empty());
        assert!(d.set_gain("tuner", GainMode::Manual(14.0)).is_ok(), "accepted and ignored");
        assert!(d.set_toggle("bias_t", false).is_err(), "and a switch is refused outright");
    }
}
