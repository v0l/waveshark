//! Handing this receiver's span out to network subscribers.
//!
//! The graph is synchronous and the socket work is not, so a server owns a
//! thread with a single-threaded runtime on it and the caller only ever
//! touches [`Server`]: [`Server::push`] to give it samples, [`Server::retuned`]
//! to tell it the dial moved, [`Server::wanted`] to find out that a subscriber
//! asked it to move.
//!
//! Every subscriber gets its own task, because each chose its own bit depth
//! and codec and so each has to pack the same block differently. They share
//! one [`tokio::sync::broadcast`] of raw blocks, which drops the oldest for a
//! subscriber falling behind rather than stalling the others.
//!
//! # A tune is a request, not a setting
//!
//! Nothing here moves a tuner. A [`msg::TUNE`] is parked in [`Server::wanted`]
//! for whoever owns the radio to pick up and act on, and the answer comes back
//! through [`Server::retuned`] once the tuner has actually landed. The two are
//! deliberately separate: a subscriber asking for 433.92 MHz on a dongle that
//! steps in units of its own must be told where it really went, and only the
//! owner knows that.

use crate::proto::{
    BitDepth, Codec, DATA_HEADER_LEN, DataHeader, Frame, MAX_DATAGRAM_PAYLOAD, MAX_FRAME_PAYLOAD,
    PREAMBLE_LEN, Tlvs, VERSION_MAJOR, decode_preamble, encode_preamble, error_code, msg, now_ns,
    pack, tag,
};
use common::{Error, Result};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream, UdpSocket};
use tokio::sync::broadcast;

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

/// What the receiver is streaming, as the welcome describes it.
#[derive(Clone, Debug)]
pub struct ServerConfig {
    /// Reported to subscribers for logging.
    pub name: String,
    pub center_hz: u64,
    pub sample_rate: u32,
    pub gain_db: Option<f32>,
    /// Whether a subscriber may move the dial. Off unless an operator asked
    /// for it: the local screen follows a remote tune.
    pub tunable: bool,
    /// How far the tuner reaches, sent only when `tunable`. A subscriber that
    /// is not told this can ask for a frequency but cannot offer a dial,
    /// because it has no idea where the dial may go.
    pub tune_range_hz: Option<(u64, u64)>,
}

impl Default for ServerConfig {
    fn default() -> Self {
        ServerConfig {
            name: "waveshark".into(),
            center_hz: 0,
            sample_rate: 0,
            gain_db: None,
            tunable: false,
            tune_range_hz: None,
        }
    }
}

/// A frequency a subscriber asked for, waiting to be acted on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Tune {
    pub center_hz: u64,
}

pub struct Server {
    addr: SocketAddr,
    blocks: broadcast::Sender<Arc<Vec<u8>>>,
    /// Read by every subscriber task when it builds a welcome, and by the
    /// fan-out when it labels a block.
    center_hz: Arc<AtomicU64>,
    sample_rate: u32,
    tunable: bool,
    name: String,
    gain_ddb: Option<i16>,
    /// The last frequency a subscriber asked for, coalesced: a client dragging
    /// a dial sends one of these a frame and only the last is worth anything.
    wanted: Arc<Mutex<Option<Tune>>>,
    subscribers: Arc<AtomicUsize>,
    blocks_sent: Arc<AtomicU64>,
    /// Sent to every subscriber, so a retune reaches a reader that did not ask
    /// for one.
    retunes: broadcast::Sender<u64>,
    stop: Arc<AtomicBool>,
    join: Option<std::thread::JoinHandle<()>>,
}

impl Server {
    /// Listen on `addr`, or fail if the port is taken.
    ///
    /// A port of zero asks the kernel for a free one, which is what a test
    /// wants; [`Server::addr`] then says which it got.
    pub fn start(addr: SocketAddr, cfg: ServerConfig) -> Result<Arc<Self>> {
        let (blocks, _) = broadcast::channel(FANOUT_DEPTH);
        let (retunes, _) = broadcast::channel(8);
        let center_hz = Arc::new(AtomicU64::new(cfg.center_hz));
        let wanted = Arc::new(Mutex::new(None));
        let subscribers = Arc::new(AtomicUsize::new(0));
        let blocks_sent = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));

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

        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|e| Error::other(format!("tokio runtime: {e}")))?;

        let shared = Shared {
            cfg: cfg.clone(),
            center_hz: center_hz.clone(),
            blocks: blocks.clone(),
            retunes: retunes.clone(),
            wanted: wanted.clone(),
            subscribers: subscribers.clone(),
            blocks_sent: blocks_sent.clone(),
        };
        let stopping = stop.clone();
        let join = std::thread::Builder::new()
            .name("iqstream-srv".into())
            .spawn(move || {
                // from_std registers with the reactor, so it has to happen
                // inside the runtime rather than on the way in.
                rt.block_on(async move {
                    match TcpListener::from_std(listener) {
                        Ok(l) => accept_loop(l, shared, stopping).await,
                        Err(e) => tracing::error!("iqstream: {e}"),
                    }
                });
            })
            .map_err(|e| Error::other(format!("spawn server thread: {e}")))?;

        Ok(Arc::new(Server {
            addr: bound,
            blocks,
            center_hz,
            sample_rate: cfg.sample_rate,
            tunable: cfg.tunable,
            name: cfg.name,
            gain_ddb: cfg.gain_db.map(|g| (g * 10.0) as i16),
            wanted,
            subscribers,
            blocks_sent,
            retunes,
            stop,
            join: Some(join),
        }))
    }

    pub fn addr(&self) -> SocketAddr {
        self.addr
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn gain_ddb(&self) -> Option<i16> {
        self.gain_ddb
    }

    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
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

    pub fn center_hz(&self) -> u64 {
        self.center_hz.load(Ordering::Relaxed)
    }

    /// Hand one block of interleaved UC8 to every subscriber.
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

    /// Say where the tuner actually landed, and tell every subscriber.
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

#[derive(Clone)]
struct Shared {
    cfg: ServerConfig,
    center_hz: Arc<AtomicU64>,
    blocks: broadcast::Sender<Arc<Vec<u8>>>,
    retunes: broadcast::Sender<u64>,
    wanted: Arc<Mutex<Option<Tune>>>,
    subscribers: Arc<AtomicUsize>,
    blocks_sent: Arc<AtomicU64>,
}

async fn accept_loop(listener: TcpListener, shared: Shared, stop: Arc<AtomicBool>) {
    loop {
        let Ok((sock, peer)) = listener.accept().await else {
            continue;
        };
        if stop.load(Ordering::SeqCst) {
            return;
        }
        let shared = shared.clone();
        tokio::spawn(async move {
            if let Err(e) = serve(sock, peer, shared).await {
                tracing::debug!("iqstream: {peer} left: {e}");
            }
        });
    }
}

/// One subscriber, from preamble to disconnection.
async fn serve(sock: TcpStream, peer: SocketAddr, shared: Shared) -> Result<()> {
    sock.set_nodelay(true).map_err(other)?;
    let (mut rd, mut wr) = sock.into_split();

    let mut preamble = [0u8; PREAMBLE_LEN];
    rd.read_exact(&mut preamble).await.map_err(other)?;
    let (major, _) = decode_preamble(&preamble)?;
    wr.write_all(&encode_preamble()).await.map_err(other)?;
    if major != VERSION_MAJOR {
        let _ = send_error(
            &mut wr,
            error_code::UNSUPPORTED_VERSION,
            &format!("this server speaks {VERSION_MAJOR}.x"),
        )
        .await;
        return Err(Error::other(format!("{peer} speaks version {major}")));
    }

    let hello = read_frame(&mut rd).await?.ok_or(Error::Disconnected)?;
    if hello.msg_type != msg::HELLO {
        let _ = send_error(&mut wr, error_code::BAD_REQUEST, "expected hello").await;
        return Err(Error::other("no hello"));
    }
    let who = hello.tlvs()?.str(tag::CLIENT_NAME).unwrap_or_else(|| peer.to_string());

    let mut w = Tlvs::new();
    w.str(tag::SERVER_NAME, &shared.cfg.name)
        .u64(tag::CENTER_HZ, shared.center_hz.load(Ordering::Relaxed))
        .u32(tag::SAMPLE_RATE, shared.cfg.sample_rate)
        .u8(tag::TUNABLE, shared.cfg.tunable as u8)
        .put(tag::SUPPORTED_BIT_DEPTHS, &BitDepth::SUPPORTED)
        .put(tag::SUPPORTED_CODECS, &[Codec::None.code(), Codec::Zstd.code()]);
    if let Some(g) = shared.cfg.gain_db {
        w.i16(tag::GAIN_DDB, (g * 10.0) as i16);
    }
    if let (true, Some((lo, hi))) = (shared.cfg.tunable, shared.cfg.tune_range_hz) {
        w.u64(tag::TUNE_MIN_HZ, lo).u64(tag::TUNE_MAX_HZ, hi);
    }
    wr.write_all(&Frame::new(msg::WELCOME, &w).encode()).await.map_err(other)?;

    let sub = read_frame(&mut rd).await?.ok_or(Error::Disconnected)?;
    if sub.msg_type != msg::SUBSCRIBE {
        let _ = send_error(&mut wr, error_code::BAD_REQUEST, "expected subscribe").await;
        return Err(Error::other("no subscribe"));
    }
    let s = sub.tlvs()?;
    let bits = BitDepth::new(s.u8(tag::BIT_DEPTH).unwrap_or(8)).map_err(|e| {
        tracing::debug!("{peer}: {e}");
        e
    })?;
    let codec = Codec::from_code(s.u8(tag::CODEC).unwrap_or(0))?;
    let level = s.u8(tag::CODEC_LEVEL).unwrap_or(1) as i32;
    let udp_port = s.u16(tag::UDP_PORT).ok_or_else(|| Error::other("subscribe named no port"))?;

    // The samples go to the address the control connection came from, on the
    // port the subscriber named. Taking the port alone is what lets a client
    // behind a NAT be reached at all: it has no way to know its own address.
    let mut dest = peer;
    dest.set_port(udp_port);
    let udp = UdpSocket::bind(if dest.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" })
        .await
        .map_err(other)?;

    let mut r = Tlvs::new();
    r.u8(tag::BIT_DEPTH, bits.0).u8(tag::CODEC, codec.code());
    wr.write_all(&Frame::new(msg::SUBSCRIBED, &r).encode()).await.map_err(other)?;
    shared.subscribers.fetch_add(1, Ordering::Relaxed);
    tracing::info!("iqstream: {who} subscribed from {peer}, {} bit {codec:?}", bits.0);

    let result = pump(&mut rd, &mut wr, &udp, dest, bits, codec, level, &shared).await;
    shared.subscribers.fetch_sub(1, Ordering::Relaxed);
    tracing::info!("iqstream: {who} left");
    result
}

#[allow(clippy::too_many_arguments)]
async fn pump(
    rd: &mut tokio::net::tcp::OwnedReadHalf,
    wr: &mut tokio::net::tcp::OwnedWriteHalf,
    udp: &UdpSocket,
    dest: SocketAddr,
    bits: BitDepth,
    codec: Codec,
    level: i32,
    shared: &Shared,
) -> Result<()> {
    let mut blocks = shared.blocks.subscribe();
    let mut retunes = shared.retunes.subscribe();
    let mut ping = tokio::time::interval(std::time::Duration::from_secs(PING_INTERVAL_S));
    ping.tick().await;

    let mut seq: u32 = 0;
    let mut sample_index: u64 = 0;
    let mut packed = Vec::new();
    let mut last_seen = tokio::time::Instant::now();
    let mut datagram = [0u8; DATA_HEADER_LEN + MAX_DATAGRAM_PAYLOAD];

    loop {
        tokio::select! {
            _ = ping.tick() => {
                if last_seen.elapsed().as_secs() > IDLE_TIMEOUT_S {
                    return Err(Error::other("keepalive timed out"));
                }
                let mut t = Tlvs::new();
                t.u64(tag::TIMESTAMP_NS, now_ns());
                wr.write_all(&Frame::new(msg::PING, &t).encode()).await.map_err(other)?;
            }
            hz = retunes.recv() => {
                if let Ok(hz) = hz {
                    let mut t = Tlvs::new();
                    t.u64(tag::CENTER_HZ, hz);
                    wr.write_all(&Frame::new(msg::TUNED, &t).encode()).await.map_err(other)?;
                }
            }
            frame = read_frame(rd) => {
                let Some(frame) = frame? else { return Ok(()) };
                last_seen = tokio::time::Instant::now();
                match frame.msg_type {
                    msg::UNSUBSCRIBE => {
                        let _ = wr.write_all(&Frame::empty(msg::UNSUBSCRIBED).encode()).await;
                        return Ok(());
                    }
                    msg::PING => {
                        let mut t = Tlvs::new();
                        if let Some(ts) = frame.tlvs()?.u64(tag::TIMESTAMP_NS) {
                            t.u64(tag::TIMESTAMP_NS, ts);
                        }
                        wr.write_all(&Frame::new(msg::PONG, &t).encode()).await.map_err(other)?;
                    }
                    msg::PONG => {}
                    msg::TUNE => {
                        let hz = frame.tlvs()?.u64(tag::CENTER_HZ);
                        match (shared.cfg.tunable, hz) {
                            (false, _) => {
                                send_error(wr, error_code::NOT_TUNABLE,
                                    "this receiver is not offering its dial").await?;
                            }
                            (true, None) => {
                                send_error(wr, error_code::BAD_REQUEST,
                                    "tune named no frequency").await?;
                            }
                            (true, Some(hz))
                                if shared.cfg.tune_range_hz
                                    .is_some_and(|(lo, hi)| hz < lo || hz > hi) =>
                            {
                                send_error(wr, error_code::OUT_OF_RANGE,
                                    "outside this tuner's range").await?;
                            }
                            // Parked, not acted on: where it lands comes back
                            // as TUNED once the radio has actually moved.
                            (true, Some(hz)) => {
                                if let Ok(mut wanted) = shared.wanted.lock() {
                                    *wanted = Some(Tune { center_hz: hz });
                                }
                            }
                        }
                    }
                    _ => {}
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
                send_block(udp, dest, &mut datagram, &body, seq, sample_index,
                           samples as u32, bits, codec).await?;
                seq = seq.wrapping_add(1);
                sample_index += samples as u64;
                shared.blocks_sent.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

/// One block as one or more datagrams.
///
/// A block at a wide span is far larger than an MTU, so it is cut into
/// fragments that each carry the whole header: a receiver can then tell a lost
/// fragment from a lost block without keeping the two apart itself.
#[allow(clippy::too_many_arguments)]
async fn send_block(
    udp: &UdpSocket,
    dest: SocketAddr,
    datagram: &mut [u8; DATA_HEADER_LEN + MAX_DATAGRAM_PAYLOAD],
    body: &[u8],
    seq: u32,
    sample_index: u64,
    block_samples: u32,
    bits: BitDepth,
    codec: Codec,
) -> Result<()> {
    let frag_count = body.len().div_ceil(MAX_DATAGRAM_PAYLOAD).max(1) as u16;
    for (i, chunk) in body.chunks(MAX_DATAGRAM_PAYLOAD).enumerate() {
        let header = DataHeader {
            version: VERSION_MAJOR as u8,
            sample_index,
            block_seq: seq,
            frag_index: i as u16,
            frag_count,
            bit_depth: bits.0,
            codec,
            decimation: 1,
            block_samples,
        };
        let mut head = [0u8; DATA_HEADER_LEN];
        header.encode(&mut head);
        datagram[..DATA_HEADER_LEN].copy_from_slice(&head);
        datagram[DATA_HEADER_LEN..DATA_HEADER_LEN + chunk.len()].copy_from_slice(chunk);
        // A refused datagram is one lost block, not a dead subscriber: the
        // receiver pads the gap and carries on.
        let _ = udp.send_to(&datagram[..DATA_HEADER_LEN + chunk.len()], dest).await;
    }
    Ok(())
}

async fn send_error(
    wr: &mut tokio::net::tcp::OwnedWriteHalf,
    code: u16,
    message: &str,
) -> Result<()> {
    let mut t = Tlvs::new();
    t.u16(tag::ERROR_CODE, code).str(tag::ERROR_MESSAGE, message);
    wr.write_all(&Frame::new(msg::ERROR, &t).encode()).await.map_err(other)
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
