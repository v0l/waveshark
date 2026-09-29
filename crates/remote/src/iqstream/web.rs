use super::{Hands, Landing, RATE_SETTLE, Wire, config};
use crate::CONNECT_TIMEOUT;
use common::device::RxStream;
use common::time::{Duration, Instant};
use common::{Error, IqBuf, Result, Sps};
use iqstream::web::{Dial, IqStream, Next};
use iqstream::{SettingValue, StreamDesc};
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, LazyLock, Mutex};

const POLL: Duration = Duration::from_millis(50);

static HEARD: LazyLock<Mutex<HashMap<String, Vec<StreamDesc>>>> = LazyLock::new(Default::default);
static ASKING: LazyLock<Mutex<HashSet<String>>> = LazyLock::new(Default::default);

const OFFER_WAIT: Duration = Duration::from_secs(5);
const SIGNAL_WAIT: Duration = Duration::from_secs(10);
const OPEN_WAIT: Duration = Duration::from_secs(12);

fn url(host: &str) -> String {
    match iqstream::ws::is_url(host) {
        true => host.to_string(),
        false => format!("ws://{host}/"),
    }
}

fn reach(host: &str) -> Result<Dial> {
    let Some(key) = iqstream::ws::webrtc_key(host) else { return Ok(Dial::Url(url(host))) };
    let server = nostr_directory::PublicKey::from_hex(key)
        .ok_or_else(|| Error::other(format!("{host} names no nostr key")))?;
    let (id, offer) =
        httpc::rtc::offer(iqstream::ws::SUBPROTOCOL, OFFER_WAIT).map_err(Error::other)?;
    let answer =
        nostr_directory::signal::ask(&server, &offer, &nostr_directory::RELAYS, SIGNAL_WAIT)
            .inspect_err(|_| httpc::rtc::forget(id))
            .map_err(|e| Error::other(format!("{host}: {e}")))?;
    let bridge = httpc::rtc::open(id, &answer, host, OPEN_WAIT).map_err(Error::other)?;
    Ok(Dial::Rtc(bridge))
}

fn remember(host: &str, found: &[StreamDesc]) {
    if let Ok(mut heard) = HEARD.lock() {
        heard.insert(host.to_string(), found.to_vec());
    }
}

pub(super) fn list(host: &str) -> Result<Vec<StreamDesc>> {
    if common::page::here() {
        return known(host);
    }
    let found = iqstream::web::list(reach(host)?, "waveshark probe", CONNECT_TIMEOUT)
        .map_err(|e| Error::other(format!("{host}: {e}")))?;
    remember(host, &found);
    Ok(found)
}

fn known(host: &str) -> Result<Vec<StreamDesc>> {
    if let Some(found) = HEARD.lock().ok().and_then(|h| h.get(host).cloned()) {
        return Ok(found);
    }
    if ASKING.lock().is_ok_and(|mut a| a.insert(host.to_string())) {
        let asked = host.to_string();
        common::thread::spawn(move || {
            match iqstream::web::list(reach(&asked)?, "waveshark probe", CONNECT_TIMEOUT) {
                Ok(found) => {
                    remember(&asked, &found);
                    crate::answered_now();
                }
                Err(e) => tracing::debug!("iqstream: {asked}: {e}"),
            }
            if let Ok(mut asking) = ASKING.lock() {
                asking.remove(&asked);
            }
            Ok::<(), Error>(())
        });
    }
    Err(Error::other(format!("{host} has not answered yet")))
}

pub(super) fn set_rate(host: &str, id: u16, r: Sps) -> Result<()> {
    let value = SettingValue::Choice(r.0.to_string());
    iqstream::web::set(
        reach(host)?,
        "waveshark",
        id,
        iqstream::RATE_SETTING,
        value,
        CONNECT_TIMEOUT,
    )
    .map_err(|e| Error::other(format!("{host}: {e}")))?;
    let started = Instant::now();
    loop {
        if let Ok(streams) = iqstream::web::list(reach(host)?, "waveshark probe", CONNECT_TIMEOUT)
            && streams.iter().any(|s| s.id == id && s.sample_rate as u64 == r.0)
        {
            return Ok(());
        }
        if started.elapsed() >= RATE_SETTLE {
            return Err(Error::other(format!("{host} did not move to {} S/s", r.0)));
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

pub(super) fn start(wire: Wire, hands: Hands) -> Result<Box<dyn RxStream>> {
    let Hands { dial, asks, held, streaming } = hands;
    let stream = reach(&wire.addr)
        .and_then(|d| IqStream::connect(d, config("waveshark", wire.stream), CONNECT_TIMEOUT));
    let stream = match stream {
        Ok(s) => s,
        Err(e) => {
            streaming.store(false, Ordering::SeqCst);
            return Err(Error::other(format!("{}: {e}", wire.addr)));
        }
    };
    let (want, at) = match dial {
        Some((w, at)) => (Some(w), Some(at)),
        None => (None, None),
    };
    let seen = stream.info().settings;
    let landing = Landing::new(&wire, at, seen, held, Arc::new(AtomicU64::new(0)));
    Ok(Box::new(NetStream { stream: Some(stream), landing, want, asks, streaming }))
}

struct NetStream {
    stream: Option<IqStream>,
    landing: Landing,
    want: Option<tokio::sync::watch::Receiver<u64>>,
    asks: Option<tokio::sync::mpsc::UnboundedReceiver<(String, SettingValue)>>,
    streaming: Arc<AtomicBool>,
}

impl NetStream {
    fn ask(&mut self, stream: &IqStream) {
        if let Some(want) = self.want.as_mut()
            && want.has_changed().unwrap_or(false)
        {
            let hz = *want.borrow_and_update();
            if let Err(e) = stream.tune(hz) {
                tracing::debug!("iqstream: {e}");
            }
        }
        while let Some(Ok((name, value))) = self.asks.as_mut().map(|a| a.try_recv()) {
            if let Err(e) = stream.set_setting(&name, value) {
                tracing::debug!("iqstream: {}: {e}", self.landing.addr);
            }
        }
    }
}

impl RxStream for NetStream {
    fn read(&mut self) -> Result<IqBuf> {
        loop {
            let stream = self.stream.take().ok_or(Error::Disconnected)?;
            self.ask(&stream);
            match stream.next_block(POLL) {
                Next::Block(block) => {
                    let settings = stream.info().settings;
                    self.stream = Some(stream);
                    return Ok(self.landing.land(block, &settings));
                }
                Next::Idle => self.stream = Some(stream),
                Next::Ended => {
                    self.stop();
                    return Err(Error::Disconnected);
                }
            }
        }
    }

    fn dropped(&self) -> u64 {
        let behind = self.stream.as_ref().map_or(0, IqStream::dropped);
        self.landing.dropped.load(Ordering::Relaxed) + behind
    }

    fn stop(&mut self) {
        self.stream = None;
        self.streaming.store(false, Ordering::SeqCst);
    }
}

impl Drop for NetStream {
    fn drop(&mut self) {
        self.stop();
    }
}
