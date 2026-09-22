//! Handing tuners out to network subscribers.
//!
//! A server is a port with a set of [`Stream`]s on it, one per tuner, and a
//! subscriber names the one it wants. The graph is synchronous and the socket
//! work is not, so a server owns a thread with a single-threaded runtime on it
//! and the caller only ever touches a stream: [`Stream::push`] to give it
//! samples, [`Stream::retuned`] to tell it the dial moved, [`Stream::wanted`]
//! to find out that a subscriber asked it to move.
//!
//! Streams come and go while the server runs, because a receiver that opens a
//! second dongle should not need a second port. Every control connection is
//! told when the set changes, so nobody is left subscribed to a name that no
//! longer means anything, and told again when one of them is set differently:
//! a gain turned down, a bias tee switched off, an antenna port moved. A
//! reader has to know what its samples were taken at, or the level it reports
//! is a level nothing was ever heard at.
//!
//! Every subscription gets its own task, because each chose its own bit depth
//! and codec and so each has to pack the same block differently. A stream's
//! subscriptions share one [`tokio::sync::broadcast`] of raw blocks, which
//! drops the oldest for a subscriber falling behind rather than stalling the
//! others. One control connection may hold a subscription to each stream at
//! once, so its writes go through a channel to a single writer task rather
//! than several tasks contending for the socket.
//!
//! # A tune is a request, not a setting
//!
//! Nothing here moves a tuner. A [`msg::TUNE`] is parked in [`Stream::wanted`]
//! for whoever owns that radio to pick up and act on, and the answer comes
//! back through [`Stream::retuned`] once the tuner has actually landed. The
//! two are deliberately separate: a subscriber asking for 433.92 MHz on a
//! dongle that steps in units of its own must be told where it really went,
//! and only the owner knows that.

use crate::proto::{
    BitDepth, Codec, DATA_HEADER_LEN, DataHeader, Frame, MAX_DATAGRAM_PAYLOAD, MAX_FRAME_PAYLOAD,
    PREAMBLE_LEN, PROBE_LADDER, SAFE_DATAGRAM_PAYLOAD, Setting, SettingValue, StreamDesc, Tlvs,
    Transport, VERSION_MAJOR, decode_preamble, decode_punch, encode_inline, encode_preamble,
    encode_probe, error_code, msg, now_ns, pack, put_streams, tag,
};
use common::{Error, Result};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream, UdpSocket};
use tokio::sync::{broadcast, mpsc};

fn other(e: impl std::fmt::Display) -> Error {
    Error::other(e.to_string())
}

/// Blocks held for a subscriber before the oldest is dropped.
///
/// A block is tens of milliseconds, so this is about a second of slack. Deeper
/// only buys a slow reader stale samples: what it wants is the live edge of
/// the stream, and it will never catch up by being given more history.
const FANOUT_DEPTH: usize = 32;

/// How long a subscriber may go without answering a keepalive.
const IDLE_TIMEOUT_S: u64 = 45;

/// How often the server pings a quiet subscriber.
const PING_INTERVAL_S: u64 = 15;

/// Blocks held for a subscriber reading its samples off the control
/// connection. A full queue drops the block rather than waiting, because the
/// keepalives and the unsubscribe share that socket and a stalled reader must
/// not be able to hold them up.
const INLINE_DEPTH: usize = 8;

/// How long the probe of each candidate size is repeated, so a single lost
/// datagram does not cost the path its real size.
const PROBE_TRIES: usize = 2;

/// One tuner, as the welcome describes it.
#[derive(Clone, Debug, Default)]
pub struct StreamConfig {
    /// What an operator picking a tuner sees: the radio's own label.
    pub name: String,
    pub center_hz: u64,
    pub sample_rate: u32,
    pub gain_db: Option<f32>,
    /// Whether a subscriber may move this dial. Off unless an operator asked
    /// for it: granting it on the receiver's own radio moves the local screen.
    pub tunable: bool,
    /// How far the tuner reaches, sent only when `tunable`. A subscriber that
    /// is not told this can ask for a frequency but cannot offer a dial,
    /// because it has no idea where the dial may go.
    pub tune_range_hz: Option<(u64, u64)>,
    /// What else the radio is set to: its gain stages, its switches, its
    /// antenna port. Kept up to date with [`Stream::set_settings`].
    pub settings: Vec<Setting>,
}

/// What the server is, and the tuners it starts with.
#[derive(Clone, Debug)]
pub struct ServerConfig {
    /// Reported to subscribers for logging.
    pub name: String,
    /// Tuners offered from the moment the port opens. More may be added with
    /// [`Server::add_stream`] while it runs.
    pub streams: Vec<StreamConfig>,
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig { name: "waveshark".into(), streams: Vec::new() }
    }
}

impl ServerConfig {
    /// One tuner and nothing else, which is what a 1.1 server was.
    pub fn single(name: &str, stream: StreamConfig) -> Self {
        ServerConfig { name: name.into(), streams: vec![stream] }
    }
}

/// A frequency a subscriber asked for, waiting to be acted on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tune {
    pub center_hz: u64,
}

/// A setting a subscriber asked for, waiting to be acted on.
///
/// Named rather than described: what a gain of 24 dB means is the driver's
/// business, and this crate does not know what a gain stage is.
#[derive(Clone, Debug, PartialEq)]
pub struct Ask {
    pub name: String,
    pub value: SettingValue,
}

/// One tuner on a server: what is pushed into it, and who is reading it.
pub struct Stream {
    id: u16,
    name: String,
    /// Changes when whoever owns the tuner reshapes it, so a rate is read
    /// rather than copied into every welcome as it is built.
    sample_rate: AtomicU32,
    /// The gain and everything else that is set rather than heard, which move
    /// under the readers and so are held together and announced together.
    state: Mutex<State>,
    tunable: bool,
    tune_range_hz: Option<(u64, u64)>,
    center_hz: AtomicU64,
    blocks: broadcast::Sender<Arc<Vec<u8>>>,
    /// Sent to every subscriber, so a retune reaches a reader that did not ask
    /// for one.
    retunes: broadcast::Sender<u64>,
    /// The last frequency a subscriber asked for, coalesced: a client dragging
    /// a dial sends one of these a frame and only the last is worth anything.
    wanted: Mutex<Option<Tune>>,
    /// Settings a subscriber asked for, one per name for the same reason:
    /// a gain slider dragged across its range is one request by the time
    /// anybody looks.
    asks: Mutex<Vec<Ask>>,
    subscribers: AtomicUsize,
    blocks_sent: AtomicU64,
    blocks_dropped: AtomicU64,
    /// The server's own, so a setting moving reaches every connection and not
    /// only this tuner's readers.
    changed: broadcast::Sender<Change>,
}

/// What a tuner is set to, beyond where it is pointed.
#[derive(Default)]
struct State {
    gain_db: Option<f32>,
    settings: Vec<Setting>,
}

impl Stream {
    fn new(id: u16, cfg: StreamConfig, changed: broadcast::Sender<Change>) -> Arc<Self> {
        Arc::new(Stream {
            id,
            name: cfg.name,
            sample_rate: AtomicU32::new(cfg.sample_rate),
            state: Mutex::new(State { gain_db: cfg.gain_db, settings: cfg.settings }),
            tunable: cfg.tunable,
            tune_range_hz: cfg.tune_range_hz,
            center_hz: AtomicU64::new(cfg.center_hz),
            blocks: broadcast::channel(FANOUT_DEPTH).0,
            retunes: broadcast::channel(8).0,
            wanted: Mutex::new(None),
            asks: Mutex::new(Vec::new()),
            subscribers: AtomicUsize::new(0),
            blocks_sent: AtomicU64::new(0),
            blocks_dropped: AtomicU64::new(0),
            changed,
        })
    }

    pub fn id(&self) -> u16 {
        self.id
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn sample_rate(&self) -> u32 {
        self.sample_rate.load(Ordering::Relaxed)
    }

    /// Say the tuner is running at another rate.
    ///
    /// Announced like any other change, though a subscriber already reading
    /// cannot act on it: the samples it is being handed are at the new rate
    /// whatever it does, and whoever changes a rate under its readers is
    /// expected to end them by dropping the stream.
    pub fn set_sample_rate(&self, rate: u32) {
        if self.sample_rate.swap(rate, Ordering::Relaxed) != rate {
            self.announce();
        }
    }

    pub fn gain_db(&self) -> Option<f32> {
        self.state.lock().ok().and_then(|s| s.gain_db)
    }

    /// Say what the whole front end is reading at, where a radio has one
    /// number for it. Announced when it moves.
    pub fn set_gain_db(&self, gain_db: Option<f32>) {
        let moved = match self.state.lock() {
            Ok(mut s) if s.gain_db != gain_db => {
                s.gain_db = gain_db;
                true
            }
            _ => false,
        };
        if moved {
            self.announce();
        }
    }

    pub fn settings(&self) -> Vec<Setting> {
        self.state.lock().map(|s| s.settings.clone()).unwrap_or_default()
    }

    /// Say what the radio's gain stages, switches and choices are set to.
    ///
    /// The whole set each time rather than one at a time, because that is
    /// what a caller reading them back off a driver has, and a set that is
    /// the same as the one before it is not announced.
    pub fn set_settings(&self, settings: Vec<Setting>) {
        let moved = match self.state.lock() {
            Ok(mut s) if s.settings != settings => {
                s.settings = settings;
                true
            }
            _ => false,
        };
        if moved {
            self.announce();
        }
    }

    /// Tell every connection what this tuner is now, subscribed to it or not.
    fn announce(&self) {
        let _ = self.changed.send(Change::Stream(self.id));
    }

    pub fn tunable(&self) -> bool {
        self.tunable
    }

    pub fn subscribers(&self) -> usize {
        self.subscribers.load(Ordering::Relaxed)
    }

    pub fn blocks_sent(&self) -> u64 {
        self.blocks_sent.load(Ordering::Relaxed)
    }

    /// Blocks thrown away because a subscriber was not taking them fast
    /// enough: it fell behind the fan-out, or it reads them off the control
    /// connection and its queue there was full. The alternative for the
    /// second is holding up its keepalives behind its own sample data.
    pub fn blocks_dropped(&self) -> u64 {
        self.blocks_dropped.load(Ordering::Relaxed)
    }

    pub fn center_hz(&self) -> u64 {
        self.center_hz.load(Ordering::Relaxed)
    }

    pub fn desc(&self) -> StreamDesc {
        let (gain_db, settings) = match self.state.lock() {
            Ok(s) => (s.gain_db, s.settings.clone()),
            Err(_) => (None, Vec::new()),
        };
        StreamDesc {
            id: self.id,
            name: self.name.clone(),
            center_hz: self.center_hz(),
            sample_rate: self.sample_rate(),
            gain_db,
            tunable: self.tunable,
            tune_range_hz: self.tune_range_hz.filter(|_| self.tunable),
            settings,
        }
    }

    /// Hand one block of interleaved UC8 to every subscriber of this tuner.
    ///
    /// Costs nothing with nobody listening, which is what lets the node stay
    /// in the graph whether or not anybody has connected.
    pub fn push(&self, uc8: &[u8]) {
        if self.blocks.receiver_count() == 0 || uc8.is_empty() {
            return;
        }
        let _ = self.blocks.send(Arc::new(uc8.to_vec()));
    }

    /// Park a frequency as a subscriber's [`msg::TUNE`] does, and refuse it on
    /// the same terms.
    ///
    /// The wire is not the only way in: the same request arrives this way from
    /// a test, and from anything local that wants to drive the server's own
    /// side of the split rather than the radio directly.
    pub fn ask(&self, center_hz: u64) -> bool {
        if !self.tunable {
            return false;
        }
        match self.wanted.lock() {
            Ok(mut w) => {
                *w = Some(Tune { center_hz });
                true
            }
            Err(_) => false,
        }
    }

    /// The frequency a subscriber last asked for, taken.
    ///
    /// Whoever owns the radio polls this and moves the dial. Nothing here can
    /// do it: this crate does not know what a tuner is.
    pub fn wanted(&self) -> Option<Tune> {
        self.wanted.lock().ok().and_then(|mut w| w.take())
    }

    /// Park a setting as a subscriber's [`msg::SET_SETTING`] does, refused on
    /// the same terms as a tune.
    pub fn ask_setting(&self, name: &str, value: SettingValue) -> bool {
        if !self.tunable {
            return false;
        }
        match self.asks.lock() {
            Ok(mut asks) => {
                let ask = Ask { name: name.to_string(), value };
                match asks.iter_mut().find(|a| a.name == ask.name) {
                    Some(held) => *held = ask,
                    None => asks.push(ask),
                }
                true
            }
            Err(_) => false,
        }
    }

    /// The settings subscribers asked for, taken.
    ///
    /// Whoever owns the radio applies them and then says what they really
    /// became with [`Stream::set_settings`], because a driver snaps a gain to
    /// its own step and refuses what the hardware will not do.
    pub fn asked(&self) -> Vec<Ask> {
        self.asks.lock().map(|mut a| std::mem::take(&mut *a)).unwrap_or_default()
    }

    /// Say where this tuner actually landed, and tell every subscriber.
    ///
    /// Called for any move, not only one a subscriber asked for: a reader
    /// whose stream slid out from under it because the local operator dragged
    /// the dial has to be told, or it labels the next block at the old
    /// frequency.
    pub fn retuned(&self, center_hz: u64) {
        if self.center_hz.swap(center_hz, Ordering::Relaxed) != center_hz {
            let _ = self.retunes.send(center_hz);
        }
    }
}

/// What a connection has to pass on: the set of tuners, or one of them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Change {
    /// A tuner was added or taken away.
    Set,
    /// One tuner is set differently than it was.
    Stream(u16),
}

/// The tuners, shared between the caller and every connection task.
struct Streams {
    name: String,
    streams: Mutex<Vec<Arc<Stream>>>,
    next_id: AtomicU32,
    /// Poked when a stream is added, removed or changed, so every connection
    /// can pass it on.
    changed: broadcast::Sender<Change>,
    /// The port punches arrive on and samples leave from, told to every
    /// client in the welcome. Zero where the port could not be had for UDP,
    /// which leaves a client naming its own port as a 1.3 one does.
    data_port: u16,
    /// Where each subscription's punch is expected, by the token it carries.
    /// A pump waits on its entry until a datagram arrives from the address
    /// the client's NAT gave it.
    punches: Mutex<HashMap<u64, tokio::sync::watch::Sender<Option<SocketAddr>>>>,
    /// Punches that arrived before the subscribe naming them, which is the
    /// order a client opening its hole first sends them in.
    early: Mutex<HashMap<u64, (SocketAddr, std::time::Instant)>>,
}

/// How long a punch is held for a subscribe that has not arrived yet. Long
/// enough for a connection that punched first, short enough that a token
/// nobody subscribes with is forgotten.
const EARLY_PUNCH_S: u64 = 30;

/// Holds a subscription's place in the punch table for as long as its pump
/// runs, and gives it up however the pump ends.
struct Punched {
    shared: Arc<Streams>,
    token: u64,
}

impl Drop for Punched {
    fn drop(&mut self) {
        if let Ok(mut p) = self.shared.punches.lock() {
            p.remove(&self.token);
        }
    }
}

/// One datagram off the server's data port: a punch, and nothing else is
/// expected there. Told to the subscription that named the token, which is
/// how a client behind NAT is reached at all.
async fn punch_loop(data: Arc<UdpSocket>, shared: Arc<Streams>) {
    let mut buf = [0u8; 2048];
    loop {
        let Ok((n, from)) = data.recv_from(&mut buf).await else {
            continue;
        };
        let Some(token) = decode_punch(&buf[..n]) else {
            continue;
        };
        let waiting = shared.punches.lock().ok().and_then(|p| p.get(&token).cloned());
        match waiting {
            Some(tx) => {
                tx.send_if_modified(|held| match *held == Some(from) {
                    true => false,
                    false => {
                        *held = Some(from);
                        true
                    }
                });
            }
            // The subscribe naming this token has not been read yet.
            None => {
                if let Ok(mut early) = shared.early.lock() {
                    let now = std::time::Instant::now();
                    early.retain(|_, (_, at)| at.elapsed().as_secs() < EARLY_PUNCH_S);
                    early.insert(token, (from, now));
                }
            }
        }
    }
}

impl Streams {
    fn all(&self) -> Vec<Arc<Stream>> {
        self.streams.lock().map(|s| s.clone()).unwrap_or_default()
    }

    fn descs(&self) -> Vec<StreamDesc> {
        self.all().iter().map(|s| s.desc()).collect()
    }

    /// The stream a message named, or the first one where it named none,
    /// which is what a 1.1 client does.
    fn pick(&self, id: Option<u16>) -> Option<Arc<Stream>> {
        let all = self.all();
        match id {
            Some(id) => all.into_iter().find(|s| s.id == id),
            None => all.into_iter().next(),
        }
    }
}

pub struct Server {
    addr: SocketAddr,
    inner: Arc<Streams>,
    stop: Arc<AtomicBool>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl Server {
    /// Listen on `addr`, or fail if the port is taken.
    ///
    /// A port of zero asks the kernel for a free one, which is what a test
    /// wants; [`Server::addr`] then says which it got.
    pub fn start(addr: SocketAddr, cfg: ServerConfig) -> Result<Arc<Self>> {
        // Bound with std so a port already in use is an error the caller sees,
        // rather than a log line from a thread it has already left. Doing it
        // through the runtime would need a block_on, which panics when start
        // is called from inside another runtime, as a test does.
        let listener = std::net::TcpListener::bind(addr).map_err(|e| match e.kind() {
            std::io::ErrorKind::AddrInUse => Error::Busy,
            _ => other(e),
        })?;
        listener.set_nonblocking(true).map_err(other)?;
        let bound = listener.local_addr().map_err(other)?;

        // The same port for the samples, so a client has one address to punch
        // to and the datagrams come back from the address it punched: a NAT
        // that mapped the control connection will pass those and nothing
        // else. A port that cannot be had for UDP leaves the server unable to
        // be punched, which is a server a client names its own port to.
        let data = std::net::UdpSocket::bind(bound).ok();
        let data_port = data.as_ref().and_then(|d| d.local_addr().ok()).map_or(0, |a| a.port());
        if let Some(d) = &data {
            d.set_nonblocking(true).map_err(other)?;
        }

        let inner = Arc::new(Streams {
            name: cfg.name,
            streams: Mutex::new(Vec::new()),
            next_id: AtomicU32::new(0),
            changed: broadcast::channel(8).0,
            data_port,
            punches: Mutex::new(HashMap::new()),
            early: Mutex::new(HashMap::new()),
        });
        for s in cfg.streams {
            add(&inner, s);
        }

        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|e| Error::other(format!("tokio runtime: {e}")))?;

        let stop = Arc::new(AtomicBool::new(false));
        let stopping = stop.clone();
        let shared = inner.clone();
        let join = std::thread::Builder::new()
            .name("iqstream-srv".into())
            .spawn(move || {
                // from_std registers with the reactor, so it has to happen
                // inside the runtime rather than on the way in.
                rt.block_on(async move {
                    let data = data.and_then(|d| UdpSocket::from_std(d).ok()).map(Arc::new);
                    if let Some(d) = data.clone() {
                        tokio::spawn(punch_loop(d, shared.clone()));
                    }
                    match TcpListener::from_std(listener) {
                        Ok(l) => accept_loop(l, shared, data, stopping).await,
                        Err(e) => tracing::error!("iqstream: {e}"),
                    }
                });
            })
            .map_err(|e| Error::other(format!("spawn server thread: {e}")))?;

        Ok(Arc::new(Server { addr: bound, inner, stop, join: Some(join) }))
    }

    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    pub fn name(&self) -> &str {
        &self.inner.name
    }

    /// Offer another tuner on this port, from now on.
    ///
    /// Every connected client is told, because a client that asked for the
    /// list once would otherwise never learn of a dongle plugged in since.
    pub fn add_stream(&self, cfg: StreamConfig) -> Arc<Stream> {
        let s = add(&self.inner, cfg);
        let _ = self.inner.changed.send(Change::Set);
        s
    }

    /// The tuner of this name, added if it is not there yet.
    ///
    /// What a stage does on every rebuild: the name is the identity, so the
    /// readers of a tuner survive the graph around it being built again.
    pub fn stream_named(&self, cfg: StreamConfig) -> Arc<Stream> {
        if let Some(s) = self.inner.all().into_iter().find(|s| s.name == cfg.name) {
            return s;
        }
        self.add_stream(cfg)
    }

    pub fn stream(&self, id: u16) -> Option<Arc<Stream>> {
        self.inner.pick(Some(id))
    }

    /// The first tuner, which is the one a client that named none is given.
    pub fn default_stream(&self) -> Option<Arc<Stream>> {
        self.inner.pick(None)
    }

    pub fn streams(&self) -> Vec<Arc<Stream>> {
        self.inner.all()
    }

    /// Stop offering a tuner. Its subscribers are dropped, since there is
    /// nothing left for them to read.
    pub fn remove_stream(&self, id: u16) {
        if let Ok(mut v) = self.inner.streams.lock() {
            v.retain(|s| s.id != id);
        }
        let _ = self.inner.changed.send(Change::Set);
    }
}

fn add(inner: &Arc<Streams>, cfg: StreamConfig) -> Arc<Stream> {
    let id = inner.next_id.fetch_add(1, Ordering::Relaxed) as u16;
    let s = Stream::new(id, cfg, inner.changed.clone());
    if let Ok(mut v) = inner.streams.lock() {
        v.push(s.clone());
    }
    s
}

impl Drop for Server {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        // Poke the acceptor so it wakes and sees the flag rather than sitting
        // in accept() until the next connection.
        let _ = std::net::TcpStream::connect(self.addr);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

async fn accept_loop(
    listener: TcpListener,
    shared: Arc<Streams>,
    data: Option<Arc<UdpSocket>>,
    stop: Arc<AtomicBool>,
) {
    loop {
        let Ok((sock, peer)) = listener.accept().await else {
            continue;
        };
        if stop.load(Ordering::SeqCst) {
            return;
        }
        let shared = shared.clone();
        let data = data.clone();
        tokio::spawn(async move {
            if let Err(e) = serve(sock, peer, shared, data).await {
                tracing::debug!("iqstream: {peer} left: {e}");
            }
        });
    }
}

/// One subscription on one connection: the task pumping it, the sizes the
/// client has said its path takes, and the switch that ends it.
struct Subscription {
    stop: tokio::sync::watch::Sender<bool>,
    /// The largest payload a probe got through with. Raised as the client
    /// answers, so the first blocks leave at the size any path takes and the
    /// rest at the size this one does.
    payload: tokio::sync::watch::Sender<usize>,
    task: tokio::task::JoinHandle<()>,
}

impl Subscription {
    async fn end(self) {
        let _ = self.stop.send(true);
        let _ = self.task.await;
    }
}

/// One client, from preamble to disconnection.
///
/// The connection reads frames and owns the subscriptions it opened; the
/// samples of each go out from a task of their own. Everything written to the
/// socket goes through `out`, because several tasks have something to say on
/// it and a half-written frame from two of them at once is an unreadable
/// stream.
async fn serve(
    sock: TcpStream,
    peer: SocketAddr,
    shared: Arc<Streams>,
    data: Option<Arc<UdpSocket>>,
) -> Result<()> {
    sock.set_nodelay(true).map_err(other)?;
    let (mut rd, mut wr) = sock.into_split();

    let mut preamble = [0u8; PREAMBLE_LEN];
    rd.read_exact(&mut preamble).await.map_err(other)?;
    let (major, _) = decode_preamble(&preamble)?;
    wr.write_all(&encode_preamble()).await.map_err(other)?;
    if major != VERSION_MAJOR {
        let mut t = Tlvs::new();
        t.u16(tag::ERROR_CODE, error_code::UNSUPPORTED_VERSION)
            .str(tag::ERROR_MESSAGE, &format!("this server speaks {VERSION_MAJOR}.x"));
        let _ = wr.write_all(&Frame::new(msg::ERROR, &t).encode()).await;
        return Err(Error::other(format!("{peer} speaks version {major}")));
    }

    let hello = read_frame(&mut rd).await?.ok_or(Error::Disconnected)?;
    if hello.msg_type != msg::HELLO {
        let mut t = Tlvs::new();
        t.u16(tag::ERROR_CODE, error_code::BAD_REQUEST).str(tag::ERROR_MESSAGE, "expected hello");
        let _ = wr.write_all(&Frame::new(msg::ERROR, &t).encode()).await;
        return Err(Error::other("no hello"));
    }
    let who = hello.tlvs()?.str(tag::CLIENT_NAME).unwrap_or_else(|| peer.to_string());

    // Samples on their own queue, read second and never waited on, so a
    // reader taking them off the control connection cannot hold up a ping or
    // an unsubscribe behind a megabyte of its own samples.
    let (out, mut outbox) = mpsc::channel::<Vec<u8>>(64);
    let (inline, mut inbox) = mpsc::channel::<Vec<u8>>(INLINE_DEPTH);
    let writer = tokio::spawn(async move {
        let mut inline_open = true;
        loop {
            let bytes = tokio::select! {
                biased;
                frame = outbox.recv() => match frame {
                    Some(b) => b,
                    None => return,
                },
                block = inbox.recv(), if inline_open => match block {
                    Some(b) => b,
                    None => {
                        inline_open = false;
                        continue;
                    }
                },
            };
            if wr.write_all(&bytes).await.is_err() {
                return;
            }
        }
    });

    let result = converse(&mut rd, &out, &inline, &shared, data, peer, &who).await;
    drop(out);
    drop(inline);
    let _ = writer.await;
    tracing::info!("iqstream: {who} left");
    result
}

#[allow(clippy::too_many_arguments)]
async fn converse(
    rd: &mut tokio::net::tcp::OwnedReadHalf,
    out: &mpsc::Sender<Vec<u8>>,
    inline: &mpsc::Sender<Vec<u8>>,
    shared: &Arc<Streams>,
    data: Option<Arc<UdpSocket>>,
    peer: SocketAddr,
    who: &str,
) -> Result<()> {
    send(out, welcome(shared)).await?;

    let mut subs: HashMap<u16, Subscription> = HashMap::new();
    let mut changed = shared.changed.subscribe();
    let mut ping = tokio::time::interval(std::time::Duration::from_secs(PING_INTERVAL_S));
    ping.tick().await;
    let mut last_seen = tokio::time::Instant::now();

    let ending = loop {
        tokio::select! {
            _ = ping.tick() => {
                if last_seen.elapsed().as_secs() > IDLE_TIMEOUT_S {
                    break Err(Error::other("keepalive timed out"));
                }
                let mut t = Tlvs::new();
                t.u64(tag::TIMESTAMP_NS, now_ns());
                send(out, Frame::new(msg::PING, &t)).await?;
            }
            // One tuner is set differently: a gain, a switch, an antenna
            // port, the rate. Passed on to this connection whether or not it
            // subscribed to that tuner, because a reader is entitled to know
            // what the others are doing before it picks one.
            change = changed.recv() => {
                if let Ok(Change::Stream(id)) = change {
                    if let Some(s) = shared.pick(Some(id)) {
                        let mut t = Tlvs::new();
                        put_streams(&mut t, &[s.desc()]);
                        send(out, Frame::new(msg::STREAM_CHANGED, &t)).await?;
                    }
                    continue;
                }
                // A tuner came or went. Everything connected is told, and a
                // subscription whose stream is gone is ended here rather than
                // left reading a channel nothing will ever push to.
                let live = shared.all();
                let lost: Vec<u16> = subs
                    .keys()
                    .copied()
                    .filter(|id| !live.iter().any(|s| s.id == *id))
                    .collect();
                for id in lost {
                    if let Some(sub) = subs.remove(&id) {
                        sub.end().await;
                    }
                    let mut t = Tlvs::new();
                    t.u16(tag::STREAM_ID, id);
                    send(out, Frame::new(msg::UNSUBSCRIBED, &t)).await?;
                }
                send(out, streams_frame(shared)).await?;
            }
            frame = read_frame(rd) => {
                let Some(frame) = frame? else { break Ok(()) };
                last_seen = tokio::time::Instant::now();
                let named = frame.tlvs()?.u16(tag::STREAM_ID);
                match frame.msg_type {
                    msg::SUBSCRIBE => {
                        let Some(stream) = shared.pick(named) else {
                            fault(out, error_code::NO_SUCH_STREAM, "no such tuner here").await?;
                            continue;
                        };
                        // A second subscribe to the same tuner replaces the
                        // first: the client has changed its mind about the
                        // bit depth, and two pumps would send it both.
                        if let Some(old) = subs.remove(&stream.id()) {
                            old.end().await;
                        }
                        let started = subscribe(
                            &frame, &stream, out, inline, shared, data.clone(), peer, who,
                        )
                        .await?;
                        match started {
                            Some(sub) => {
                                subs.insert(stream.id(), sub);
                            }
                            None => continue,
                        }
                    }
                    // Without a stream id, every subscription this connection
                    // holds, which is what a 1.1 client means by it.
                    msg::UNSUBSCRIBE => {
                        let ids: Vec<u16> = match named {
                            Some(id) => vec![id],
                            None => subs.keys().copied().collect(),
                        };
                        for id in ids {
                            if let Some(sub) = subs.remove(&id) {
                                sub.end().await;
                            }
                            let mut t = Tlvs::new();
                            t.u16(tag::STREAM_ID, id);
                            send(out, Frame::new(msg::UNSUBSCRIBED, &t)).await?;
                        }
                        if subs.is_empty() {
                            break Ok(());
                        }
                    }
                    msg::LIST_STREAMS => send(out, streams_frame(shared)).await?,
                    msg::PING => {
                        let mut t = Tlvs::new();
                        if let Some(ts) = frame.tlvs()?.u64(tag::TIMESTAMP_NS) {
                            t.u64(tag::TIMESTAMP_NS, ts);
                        }
                        send(out, Frame::new(msg::PONG, &t)).await?;
                    }
                    msg::PONG => {}
                    // A datagram of that size reached the client, so the
                    // path takes it: the pump reads this before every block
                    // and the stream widens as the sizes are confirmed.
                    msg::PROBED => {
                        let t = frame.tlvs()?;
                        let sub = shared.pick(named).and_then(|s| subs.get(&s.id()));
                        if let (Some(size), Some(sub)) = (t.u16(tag::PROBE_SIZE), sub) {
                            sub.payload.send_if_modified(|held| match size as usize > *held {
                                true => {
                                    *held = size as usize;
                                    true
                                }
                                false => false,
                            });
                        }
                    }
                    msg::TUNE => tune(&frame, shared, named, out).await?,
                    msg::SET_SETTING => set_setting(&frame, shared, named, out).await?,
                    _ => {}
                }
            }
        }
    };

    for (_, sub) in subs.drain() {
        sub.end().await;
    }
    ending
}

/// What a `TUNE` does, which is to be written down for whoever owns that
/// tuner, once it is clear there is one and it may be moved.
async fn tune(
    frame: &Frame,
    shared: &Arc<Streams>,
    named: Option<u16>,
    out: &mpsc::Sender<Vec<u8>>,
) -> Result<()> {
    let Some(stream) = shared.pick(named) else {
        return fault(out, error_code::NO_SUCH_STREAM, "no such tuner here").await;
    };
    let hz = frame.tlvs()?.u64(tag::CENTER_HZ);
    match (stream.tunable(), hz) {
        (false, _) => {
            fault(out, error_code::NOT_TUNABLE, "this receiver is not offering its dial").await
        }
        (true, None) => fault(out, error_code::BAD_REQUEST, "tune named no frequency").await,
        (true, Some(hz)) if stream.tune_range_hz.is_some_and(|(lo, hi)| hz < lo || hz > hi) => {
            fault(out, error_code::OUT_OF_RANGE, "outside this tuner's range").await
        }
        // Parked, not acted on: where it lands comes back as TUNED once the
        // radio has actually moved.
        (true, Some(hz)) => {
            if let Ok(mut wanted) = stream.wanted.lock() {
                *wanted = Some(Tune { center_hz: hz });
            }
            Ok(())
        }
    }
}

/// What a `SET_SETTING` does, which is to be written down for whoever owns
/// that radio once it is clear the tuner is offered and knows the setting.
async fn set_setting(
    frame: &Frame,
    shared: &Arc<Streams>,
    named: Option<u16>,
    out: &mpsc::Sender<Vec<u8>>,
) -> Result<()> {
    let Some(stream) = shared.pick(named) else {
        return fault(out, error_code::NO_SUCH_STREAM, "no such tuner here").await;
    };
    if !stream.tunable() {
        return fault(out, error_code::NOT_TUNABLE, "this radio is not offered for setting").await;
    }
    let t = frame.tlvs()?;
    let (Some(name), Some(text)) = (t.str(tag::SETTING_NAME), t.str(tag::SETTING_VALUE)) else {
        return fault(out, error_code::BAD_REQUEST, "name a setting and a value").await;
    };
    // Only what the tuner said it had, so a name the radio never offered is
    // refused here rather than swallowed by whoever polls the requests.
    let Some(known) = stream.settings().into_iter().find(|s| s.name == name) else {
        return fault(out, error_code::NO_SUCH_SETTING, "this tuner has no such setting").await;
    };
    stream.ask_setting(&name, SettingValue::parse(known.kind, &text));
    Ok(())
}

/// Take one subscribe and start the task that feeds it, or answer why not.
#[allow(clippy::too_many_arguments)]
async fn subscribe(
    frame: &Frame,
    stream: &Arc<Stream>,
    out: &mpsc::Sender<Vec<u8>>,
    inline: &mpsc::Sender<Vec<u8>>,
    shared: &Arc<Streams>,
    data: Option<Arc<UdpSocket>>,
    peer: SocketAddr,
    who: &str,
) -> Result<Option<Subscription>> {
    let s = frame.tlvs()?;
    let bits = match BitDepth::new(s.u8(tag::BIT_DEPTH).unwrap_or(8)) {
        Ok(b) => b,
        Err(e) => {
            fault(out, error_code::UNSUPPORTED_BIT_DEPTH, &e.to_string()).await?;
            return Ok(None);
        }
    };
    let codec = match Codec::from_code(s.u8(tag::CODEC).unwrap_or(0)) {
        Ok(c) => c,
        Err(e) => {
            fault(out, error_code::UNSUPPORTED_CODEC, &e.to_string()).await?;
            return Ok(None);
        }
    };
    let level = s.u8(tag::CODEC_LEVEL).unwrap_or(1) as i32;
    let transport = match Transport::from_code(s.u8(tag::TRANSPORT).unwrap_or(0)) {
        Ok(t) => t,
        Err(e) => {
            fault(out, error_code::UNSUPPORTED_TRANSPORT, &e.to_string()).await?;
            return Ok(None);
        }
    };
    let token = s.u64(tag::PUNCH_TOKEN);

    // Where the samples go over UDP. The port a client names is where a 1.3
    // one is served and where a punch that never arrives leaves this; a punch
    // replaces it with the address the client's NAT really gave it, which is
    // the only address that reaches a client behind one.
    let named_port = s.u16(tag::UDP_PORT).map(|port| {
        let mut dest = peer;
        dest.set_port(port);
        dest
    });
    let route = match transport {
        Transport::Tcp => Route::Tcp(inline.clone()),
        Transport::Udp => {
            let Some(data) = data else {
                fault(out, error_code::UNSUPPORTED_TRANSPORT, "this server has no data port")
                    .await?;
                return Ok(None);
            };
            if named_port.is_none() && token.is_none() {
                fault(out, error_code::BAD_REQUEST, "subscribe named no port and no token").await?;
                return Ok(None);
            }
            // A client that punched before it subscribed has already said
            // where it is, and waiting for it to say so again would cost the
            // subscription its first second of samples.
            let punched = token.and_then(|token| {
                let mut early = shared.early.lock().ok()?;
                let (addr, at) = early.remove(&token)?;
                (at.elapsed().as_secs() < EARLY_PUNCH_S).then_some(addr)
            });
            let (dest_tx, dest_rx) = tokio::sync::watch::channel(punched.or(named_port));
            let held = token.map(|token| {
                if let Ok(mut p) = shared.punches.lock() {
                    p.insert(token, dest_tx);
                }
                Punched { shared: shared.clone(), token }
            });
            Route::Udp { data, dest: dest_rx, token, _held: held }
        }
    };

    let mut r = Tlvs::new();
    r.u16(tag::STREAM_ID, stream.id())
        .u8(tag::BIT_DEPTH, bits.0)
        .u8(tag::CODEC, codec.code())
        .u8(tag::TRANSPORT, transport.code());
    send(out, Frame::new(msg::SUBSCRIBED, &r)).await?;
    tracing::info!(
        "iqstream: {who} subscribed to {} at {} bit {codec:?} over {transport:?}",
        stream.name(),
        bits.0
    );

    let (stop_tx, stop_rx) = tokio::sync::watch::channel(false);
    // A client that can be probed starts at the size any path takes and is
    // widened by what it answers; one that cannot be gets what every client
    // got before there were probes.
    let (payload_tx, payload_rx) = tokio::sync::watch::channel(match token {
        Some(_) => SAFE_DATAGRAM_PAYLOAD,
        None => MAX_DATAGRAM_PAYLOAD,
    });
    let task = tokio::spawn({
        let stream = stream.clone();
        let out = out.clone();
        async move {
            let feed = Feed { bits, codec, level, route, payload: payload_rx };
            if let Err(e) = pump(&stream, &out, feed, stop_rx).await {
                tracing::debug!("iqstream: {} stopped: {e}", stream.name());
            }
        }
    });
    Ok(Some(Subscription { stop: stop_tx, payload: payload_tx, task }))
}

fn welcome(shared: &Arc<Streams>) -> Frame {
    let mut w = Tlvs::new();
    w.str(tag::SERVER_NAME, &shared.name);
    let descs = shared.descs();
    // The first tuner in the flat tags as well, because a 1.0 or 1.1 client
    // reads nothing else and there is no version in which it can be told
    // there are others.
    if let Some(d) = descs.first() {
        w.u64(tag::CENTER_HZ, d.center_hz)
            .u32(tag::SAMPLE_RATE, d.sample_rate)
            .u8(tag::TUNABLE, d.tunable as u8);
        if let Some(g) = d.gain_db {
            w.i16(tag::GAIN_DDB, (g * 10.0) as i16);
        }
        if let Some((lo, hi)) = d.tune_range_hz {
            w.u64(tag::TUNE_MIN_HZ, lo).u64(tag::TUNE_MAX_HZ, hi);
        }
    }
    w.put(tag::SUPPORTED_BIT_DEPTHS, &BitDepth::SUPPORTED)
        .put(tag::SUPPORTED_CODECS, &[Codec::None.code(), Codec::Zstd.code()]);
    // Where to punch. A server that could not have the port for UDP says
    // nothing, and is then a server a client names its own port to.
    if shared.data_port != 0 {
        w.u16(tag::DATA_PORT, shared.data_port);
    }
    put_streams(&mut w, &descs);
    Frame::new(msg::WELCOME, &w)
}

fn streams_frame(shared: &Arc<Streams>) -> Frame {
    let mut t = Tlvs::new();
    put_streams(&mut t, &shared.descs());
    Frame::new(msg::STREAMS, &t)
}

async fn send(out: &mpsc::Sender<Vec<u8>>, frame: Frame) -> Result<()> {
    out.send(frame.encode()).await.map_err(|_| Error::Disconnected)
}

async fn fault(out: &mpsc::Sender<Vec<u8>>, code: u16, message: &str) -> Result<()> {
    let mut t = Tlvs::new();
    t.u16(tag::ERROR_CODE, code).str(tag::ERROR_MESSAGE, message);
    send(out, Frame::new(msg::ERROR, &t)).await
}

/// Where one subscription's samples go, and how big they may be when they
/// get there.
enum Route {
    Udp {
        data: Arc<UdpSocket>,
        /// Where the client really is, which is what its punch said and not
        /// what it thinks its own address is. None until it has punched, and
        /// a pump with nothing here has nowhere to send.
        dest: tokio::sync::watch::Receiver<Option<SocketAddr>>,
        token: Option<u64>,
        _held: Option<Punched>,
    },
    /// The samples on the control connection, for a client no datagram
    /// reached.
    Tcp(mpsc::Sender<Vec<u8>>),
}

/// What one subscription asked for.
struct Feed {
    bits: BitDepth,
    codec: Codec,
    level: i32,
    route: Route,
    payload: tokio::sync::watch::Receiver<usize>,
}

/// One subscription's samples, until it is ended or the tuner goes away.
async fn pump(
    stream: &Arc<Stream>,
    out: &mpsc::Sender<Vec<u8>>,
    mut feed: Feed,
    mut stop: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    let mut blocks = stream.blocks.subscribe();
    let mut retunes = stream.retunes.subscribe();
    stream.subscribers.fetch_add(1, Ordering::Relaxed);
    let _count = Counted(stream.clone());
    let (bits, codec, level) = (feed.bits, feed.codec, feed.level);

    // Nothing can be sent to a client that has not been heard from, so the
    // pump waits for the punch rather than sending to an address that is only
    // the client's guess at itself. A client that gave up on UDP meanwhile
    // subscribes again over TCP, which replaces this subscription.
    if let Route::Udp { data, dest, token, .. } = &mut feed.route {
        while dest.borrow_and_update().is_none() {
            tokio::select! {
                _ = stop.changed() => return Ok(()),
                changed = dest.changed() => if changed.is_err() { return Ok(()) },
            }
        }
        let to = *dest.borrow_and_update();
        if let (Some(to), Some(token)) = (to, *token) {
            let mut probe = Vec::new();
            for size in PROBE_LADDER {
                encode_probe(token, size as u16, &mut probe);
                for _ in 0..PROBE_TRIES {
                    let _ = data.send_to(&probe, to).await;
                }
            }
        }
    }

    let mut seq: u32 = 0;
    let mut sample_index: u64 = 0;
    let mut packed = Vec::new();
    let mut datagram = [0u8; DATA_HEADER_LEN + MAX_DATAGRAM_PAYLOAD];

    loop {
        tokio::select! {
            _ = stop.changed() => return Ok(()),
            hz = retunes.recv() => {
                if let Ok(hz) = hz {
                    let mut t = Tlvs::new();
                    t.u16(tag::STREAM_ID, stream.id()).u64(tag::CENTER_HZ, hz);
                    send(out, Frame::new(msg::TUNED, &t)).await?;
                }
            }
            block = blocks.recv() => {
                let block = match block {
                    Ok(b) => b,
                    // Behind by more than the fan-out holds. The samples are
                    // gone; the index still counts them, so the gap shows up
                    // at the far end as padding rather than as a jump.
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        tracing::debug!("iqstream: subscriber lagged {n} blocks");
                        stream.blocks_dropped.fetch_add(n, Ordering::Relaxed);
                        continue;
                    }
                    Err(broadcast::error::RecvError::Closed) => return Ok(()),
                };
                let samples = block.len() / 2;
                pack(&block, bits.0, &mut packed);
                let body = match codec {
                    Codec::None => std::mem::take(&mut packed),
                    Codec::Zstd => zstd::bulk::compress(&packed, level).map_err(other)?,
                };
                let head = |frag_index, frag_count| DataHeader {
                    version: VERSION_MAJOR as u8,
                    sample_index,
                    block_seq: seq,
                    frag_index,
                    frag_count,
                    bit_depth: bits.0,
                    codec,
                    decimation: 1,
                    block_samples: samples as u32,
                    stream_id: stream.id(),
                };
                match &feed.route {
                    Route::Udp { data, dest, .. } => {
                        let Some(to) = *dest.borrow() else { continue };
                        let cap = (*feed.payload.borrow_and_update()).min(MAX_DATAGRAM_PAYLOAD);
                        send_block(data, to, &mut datagram, &body, cap, head).await;
                    }
                    // Nothing is cut up here: one record carries the block
                    // whatever its size, because the stream under it is
                    // already in order and will not lose a piece of it.
                    Route::Tcp(inline) => {
                        let mut whole = [0u8; DATA_HEADER_LEN];
                        head(0, 1).encode(&mut whole);
                        let mut with_head = whole.to_vec();
                        with_head.extend_from_slice(&body);
                        let mut record = Vec::new();
                        encode_inline(&with_head, &mut record);
                        if inline.try_send(record).is_err() {
                            stream.blocks_dropped.fetch_add(1, Ordering::Relaxed);
                            continue;
                        }
                    }
                }
                seq = seq.wrapping_add(1);
                sample_index += samples as u64;
                stream.blocks_sent.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

/// Keeps a stream's reader count true however its pump ends.
struct Counted(Arc<Stream>);

impl Drop for Counted {
    fn drop(&mut self) {
        self.0.subscribers.fetch_sub(1, Ordering::Relaxed);
    }
}

/// One block as one or more datagrams, none larger than the path was measured
/// to take.
///
/// A block at a wide span is far larger than an MTU, so it is cut into
/// fragments that each carry the whole header: a receiver can then tell a lost
/// fragment from a lost block without keeping the two apart itself.
async fn send_block(
    udp: &UdpSocket,
    dest: SocketAddr,
    datagram: &mut [u8; DATA_HEADER_LEN + MAX_DATAGRAM_PAYLOAD],
    body: &[u8],
    cap: usize,
    head: impl Fn(u16, u16) -> DataHeader,
) {
    let frag_count = body.len().div_ceil(cap).max(1) as u16;
    for (i, chunk) in body.chunks(cap).enumerate() {
        let mut header = [0u8; DATA_HEADER_LEN];
        head(i as u16, frag_count).encode(&mut header);
        datagram[..DATA_HEADER_LEN].copy_from_slice(&header);
        datagram[DATA_HEADER_LEN..DATA_HEADER_LEN + chunk.len()].copy_from_slice(chunk);
        // A refused datagram is one lost block, not a dead subscriber: the
        // receiver pads the gap and carries on.
        let _ = udp.send_to(&datagram[..DATA_HEADER_LEN + chunk.len()], dest).await;
    }
}

async fn read_frame(sock: &mut tokio::net::tcp::OwnedReadHalf) -> Result<Option<Frame>> {
    let mut head = [0u8; 4];
    match sock.read_exact(&mut head).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(other(e)),
    }
    let len = u16::from_le_bytes([head[2], head[3]]) as usize;
    if len > MAX_FRAME_PAYLOAD {
        return Err(Error::other(format!("control frame too large: {len}")));
    }
    let mut payload = vec![0u8; len];
    sock.read_exact(&mut payload).await.map_err(other)?;
    Ok(Some(Frame { version: head[0], msg_type: head[1], payload }))
}
