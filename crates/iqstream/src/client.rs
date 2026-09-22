//! Subscribing to somebody else's tuner.
//!
//! Keepalives are answered inside [`IqStream::next_block`], so a caller that
//! stops polling is dropped by the server within its keepalive window. That is
//! deliberate: the control connection is the subscription's lifetime.
//!
//! Samples arrive over UDP and a block larger than an MTU arrives in
//! fragments, so [`Assembler`] holds the pieces. A block still missing a
//! fragment when a newer block completes is abandoned rather than waited for:
//! a late block is worth nothing to a demodulator, and the gap it leaves is
//! reported in [`Block::padded_before`] so the timebase stays true.
//!
//! # Getting the samples through a NAT
//!
//! A client behind NAT has no idea what address the world reaches it on, so
//! it punches: a datagram to the server's data port, repeated until samples
//! arrive and then at the keepalive interval to hold the mapping open. The
//! server sends to the address that punch came from, which is the only
//! address that works. It then probes the path with datagrams of a few sizes
//! and only the ones that arrive are answered, so a path that will not carry
//! a 1500 byte datagram gets smaller ones rather than nothing.
//!
//! Where no datagram arrives at all, which is a symmetric NAT or a firewall
//! that drops UDP outright, the client asks again for the samples on the
//! control connection ([`crate::proto::Transport::Tcp`]) and reads them off
//! the socket it already has.

use crate::proto::{
    BitDepth, Codec, DATA_HEADER_LEN, DataHeader, Frame, INLINE_MAGIC, MAX_FRAME_PAYLOAD,
    MAX_INLINE_RECORD, PREAMBLE_LEN, Setting, SettingValue, StreamDesc, Tlvs, Transport,
    VERSION_MAJOR, VERSION_MINOR, decode_preamble, decode_probe, encode_preamble, encode_punch,
    msg, now_ns, read_streams, tag, unpack,
};
use common::{Error, Result};
use std::collections::BTreeMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::tcp::{OwnedReadHalf, OwnedWriteHalf};
use tokio::net::{TcpStream, ToSocketAddrs, UdpSocket};
use tokio::sync::mpsc;

fn other(e: impl std::fmt::Display) -> Error {
    Error::other(e.to_string())
}

#[derive(Debug, Clone)]
pub struct ClientConfig {
    /// Reported to the server for logging.
    pub name: String,
    /// Bits per I or Q value, 3 to 8. Below 8 is lossy.
    pub bits: u8,
    pub codec: Codec,
    /// zstd level. Above 1 costs server CPU for almost no ratio.
    pub level: i8,
    /// Which tuner to read, of the ones the server offers. None takes the
    /// first, which is the only one a 1.1 server has.
    pub stream: Option<u16>,
    /// Local UDP port. 0 picks a free one.
    pub local_port: u16,
    /// Report lost samples in [`Block::padded_before`] and fill them with mid
    /// scale, so the output keeps real time alignment.
    pub pad_gaps: bool,
    pub ping_interval: Duration,
    /// What the samples should travel on.
    pub transport: Prefer,
    /// How long [`Prefer::Auto`] waits for a datagram before asking for the
    /// samples on the control connection instead. A punch is one round trip
    /// and is sent six times inside this, so a path that carries datagrams
    /// at all has delivered one by then.
    pub udp_timeout: Duration,
}

/// What an operator wants the samples carried on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Prefer {
    /// UDP, and the control connection where no datagram arrives. What a
    /// client over the internet wants, since it cannot know in advance
    /// whether the path carries datagrams at all.
    #[default]
    Auto,
    /// UDP only. A stream that cannot be got through is no stream, which is
    /// what a reader wanting the live edge or nothing asks for.
    Udp,
    /// The control connection from the start, for a path already known to
    /// drop datagrams.
    Tcp,
}

impl Default for ClientConfig {
    fn default() -> Self {
        ClientConfig {
            name: "iqstream".into(),
            bits: 8,
            codec: Codec::None,
            level: 1,
            stream: None,
            local_port: 0,
            pad_gaps: true,
            ping_interval: Duration::from_secs(10),
            transport: Prefer::Auto,
            udp_timeout: Duration::from_secs(3),
        }
    }
}

#[derive(Debug, Clone)]
pub struct StreamInfo {
    /// Which tuner of the server's this is reading.
    pub id: u16,
    /// What the far end calls that tuner. Empty from a 1.1 server, which has
    /// one stream and never names it.
    pub name: String,
    pub center_hz: u64,
    pub sample_rate: u32,
    pub gain_db: Option<f32>,
    /// Whether the server will accept [`IqStream::tune`]. False for one
    /// sharing a tuner somebody else owns.
    pub tunable: bool,
    /// How far the far end's tuner reaches. None from a server that takes a
    /// tune but did not say where it may go, which is every 1.0 server: ask
    /// and find out is all that is left.
    pub tune_range_hz: Option<(u64, u64)>,
    /// What else the far end is set to: gain stages, switches, antenna port.
    /// Kept up to date from [`msg::STREAM_CHANGED`], so this is what the
    /// block last taken was heard at.
    pub settings: Vec<Setting>,
    pub bit_depth: u8,
    pub codec: Codec,
    pub block_samples: u32,
}

/// One decoded block of interleaved UC8 samples.
#[derive(Debug, Clone)]
pub struct Block {
    /// Index of the first complex sample since stream start.
    pub sample_index: u64,
    pub samples: Vec<u8>,
    /// Complex samples lost before this block. Already prepended as mid scale
    /// when `pad_gaps` is set.
    pub padded_before: u64,
    /// The centre the server was on when this block was sent, which is the
    /// last [`msg::TUNED`] seen. Every block carries it so a caller labelling
    /// a spectrum does not have to track the retunes itself.
    pub center_hz: u64,
    /// Which tuner these samples are of, which is the one subscribed to: a
    /// datagram of anything else is dropped before it reaches here.
    pub stream_id: u16,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Stats {
    pub datagrams: u64,
    pub blocks: u64,
    pub incomplete_blocks: u64,
    pub padded_samples: u64,
    pub rtt_ms: Option<f64>,
}

pub struct IqStream {
    /// None where UDP was never on the table: the port would not bind, or the
    /// samples were asked for on the control connection from the start.
    udp: Option<UdpSocket>,
    /// Where the punches go, which is the server's data port at the address
    /// the control connection reached. None from a server too old to be
    /// punched, which is served the port this client bound instead.
    punch_to: Option<SocketAddr>,
    /// What this subscription's punches and probes carry, so the far end can
    /// tell which subscription an address it learns belongs to.
    token: u64,
    transport: Transport,
    /// Set until the first block arrives, after which a stream that has not
    /// heard anything over UDP asks for it over TCP instead.
    fallback_at: Option<tokio::time::Instant>,
    /// Cleared until a block has arrived, which is what the punching is for
    /// and what stops it.
    heard: bool,
    punching: tokio::time::Interval,
    control: OwnedWriteHalf,
    frames: mpsc::Receiver<Frame>,
    /// Blocks carried on the control connection, for a subscription that gave
    /// up on datagrams.
    inline: mpsc::Receiver<Vec<u8>>,
    info: StreamInfo,
    /// Every tuner the server offered, kept so a caller can show the others
    /// without connecting again.
    available: Vec<StreamDesc>,
    config: ClientConfig,
    assembler: Assembler,
    /// Set when the server says this subscription is over, which is how a
    /// tuner taken off the server ends its readers: the control connection
    /// stays up, so nothing else here would notice.
    ended: bool,
    stats: Stats,
    ping: tokio::time::Interval,
    buf: Vec<u8>,
    decoded: Vec<u8>,
}

/// What a server said when it was greeted, before anything subscribed.
struct Greeting {
    read_half: OwnedReadHalf,
    write_half: OwnedWriteHalf,
    welcome: Frame,
    streams: Vec<StreamDesc>,
    /// Where to punch, and the sign that this server knows how to be: a
    /// server that named no data port is a 1.3 one, and is told a port.
    punch_to: Option<SocketAddr>,
}

/// A token no other subscription on this server will be using.
///
/// The clock and a counter, mixed, which is enough: it is not a secret and
/// nothing turns on guessing it, only on two subscriptions of one client not
/// colliding.
fn token() -> u64 {
    static COUNT: AtomicU64 = AtomicU64::new(0);
    let mut v =
        now_ns() ^ (COUNT.fetch_add(1, Ordering::Relaxed).wrapping_mul(0x9e37_79b9_7f4a_7c15));
    v ^= v >> 30;
    v = v.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    v ^= v >> 27;
    v.wrapping_mul(0x94d0_49bb_1331_11eb)
}

/// Connect, say hello, and take the welcome.
///
/// A 1.1 server lists no tuners and describes its one stream in the flat
/// tags, so one is made out of those: everything above here then reads a set
/// of tuners whatever the far end's version.
async fn greet<A: ToSocketAddrs + std::fmt::Debug>(server: A, name: &str) -> Result<Greeting> {
    let control = TcpStream::connect(&server).await.map_err(other)?;
    control.set_nodelay(true).map_err(other)?;
    let peer = control.peer_addr().map_err(other)?;
    let (mut read_half, mut write_half) = control.into_split();

    write_half.write_all(&encode_preamble()).await.map_err(other)?;
    let mut preamble = [0u8; PREAMBLE_LEN];
    read_half.read_exact(&mut preamble).await.map_err(other)?;
    let (major, minor) = decode_preamble(&preamble)?;
    if major != VERSION_MAJOR {
        return Err(Error::other(format!(
            "server speaks version {major}.{minor}, this speaks {VERSION_MAJOR}.{VERSION_MINOR}"
        )));
    }

    let mut hello = Tlvs::new();
    hello.str(tag::CLIENT_NAME, name);
    write_half.write_all(&Frame::new(msg::HELLO, &hello).encode()).await.map_err(other)?;
    let welcome = read_frame(&mut read_half)
        .await?
        .ok_or_else(|| Error::other("server closed before welcome"))?;
    if welcome.msg_type != msg::WELCOME {
        return Err(Error::other(describe_error(&welcome)));
    }
    let w = welcome.tlvs()?;
    let mut streams = read_streams(&w);
    if streams.is_empty() {
        streams.push(StreamDesc {
            id: 0,
            name: w.str(tag::SERVER_NAME).unwrap_or_default(),
            center_hz: w.u64(tag::CENTER_HZ).unwrap_or(0),
            sample_rate: w.u32(tag::SAMPLE_RATE).unwrap_or(0),
            gain_db: w.i16(tag::GAIN_DDB).map(|g| g as f32 / 10.0),
            tunable: w.u8(tag::TUNABLE).unwrap_or(0) != 0,
            tune_range_hz: w.u64(tag::TUNE_MIN_HZ).zip(w.u64(tag::TUNE_MAX_HZ)),
            settings: Vec::new(),
        });
    }
    let punch_to = w.u16(tag::DATA_PORT).map(|port| {
        let mut to = peer;
        to.set_port(port);
        to
    });
    drop(w);
    Ok(Greeting { read_half, write_half, welcome, streams, punch_to })
}

/// One subscribe, which says the same things whichever transport it asks for.
fn subscribe_frame(
    config: &ClientConfig,
    bits: BitDepth,
    stream_id: u16,
    transport: Transport,
    token: u64,
    udp_port: Option<u16>,
) -> Frame {
    let mut sub = Tlvs::new();
    sub.u16(tag::STREAM_ID, stream_id)
        .u8(tag::BIT_DEPTH, bits.0)
        .u8(tag::CODEC, config.codec.code())
        .u8(tag::CODEC_LEVEL, config.level as u8)
        .u16(tag::DECIMATION, 1)
        .u8(tag::TRANSPORT, transport.code());
    if transport == Transport::Udp {
        sub.u64(tag::PUNCH_TOKEN, token);
    }
    if let Some(port) = udp_port {
        sub.u16(tag::UDP_PORT, port);
    }
    Frame::new(msg::SUBSCRIBE, &sub)
}

/// What tuners a server has, without subscribing to any of them.
///
/// What builds a radio list: a server with three dongles on it is three
/// entries, and each says where it is and how far its dial goes.
pub async fn list<A: ToSocketAddrs + std::fmt::Debug>(
    server: A,
    name: &str,
) -> Result<Vec<StreamDesc>> {
    Ok(greet(server, name).await?.streams)
}

impl IqStream {
    pub async fn connect<A: ToSocketAddrs + std::fmt::Debug>(
        server: A,
        config: ClientConfig,
    ) -> Result<Self> {
        let bit_depth = BitDepth::new(config.bits)?;

        // A port that will not bind is a machine that is not going to carry
        // datagrams, so it is the same answer as a path that drops them: ask
        // for the samples on the connection that already works.
        let udp = match config.transport {
            Prefer::Tcp => None,
            _ => UdpSocket::bind(("0.0.0.0", config.local_port)).await.ok(),
        };

        let Greeting { mut read_half, mut write_half, welcome, streams, punch_to } =
            greet(server, &config.name).await?;
        let w = welcome.tlvs()?;
        if let Some(depths) = w.get(tag::SUPPORTED_BIT_DEPTHS)
            && !depths.contains(&bit_depth.0)
        {
            return Err(Error::other(format!("server does not offer {} bit samples", bit_depth.0)));
        }
        if let Some(codecs) = w.get(tag::SUPPORTED_CODECS)
            && !codecs.contains(&config.codec.code())
        {
            return Err(Error::other(format!("server does not offer codec {:?}", config.codec)));
        }
        let wanted = match config.stream {
            Some(id) => streams
                .iter()
                .find(|s| s.id == id)
                .ok_or_else(|| Error::other(format!("server has no tuner {id}")))?,
            None => streams.first().ok_or_else(|| Error::other("server offers no tuner"))?,
        }
        .clone();

        // A server that can be punched is told nothing about this client's
        // own port: behind a NAT it is not the port anything arrives on, and
        // a server sending there is sending at some other machine on the same
        // network. Where the punch does not get through, nothing does, and
        // that is what the fallback is for.
        let token = token();
        let transport = match udp.is_some() {
            true => Transport::Udp,
            false => Transport::Tcp,
        };
        let local_port = match (&udp, punch_to) {
            (Some(udp), None) => Some(udp.local_addr().map_err(other)?.port()),
            _ => None,
        };
        if transport == Transport::Tcp && punch_to.is_none() {
            return Err(Error::other("this server cannot carry samples on the control connection"));
        }
        let sub = subscribe_frame(&config, bit_depth, wanted.id, transport, token, local_port);
        write_half.write_all(&sub.encode()).await.map_err(other)?;
        let reply = read_frame(&mut read_half)
            .await?
            .ok_or_else(|| Error::other("server closed during subscribe"))?;
        if reply.msg_type != msg::SUBSCRIBED {
            return Err(Error::other(format!("subscribe refused: {}", describe_error(&reply))));
        }
        let r = reply.tlvs()?;

        let info = StreamInfo {
            id: wanted.id,
            name: wanted.name.clone(),
            center_hz: wanted.center_hz,
            sample_rate: wanted.sample_rate,
            gain_db: wanted.gain_db,
            tunable: wanted.tunable,
            tune_range_hz: wanted.tune_range_hz,
            settings: wanted.settings.clone(),
            bit_depth: r.u8(tag::BIT_DEPTH).unwrap_or(bit_depth.0),
            codec: Codec::from_code(r.u8(tag::CODEC).unwrap_or(config.codec.code()))?,
            block_samples: r.u32(tag::BLOCK_SAMPLES).unwrap_or(0),
        };

        // read_exact is not cancellation safe, so the socket is read by its
        // own task and what comes off it is handed over channels that select!
        // can poll safely. Samples travel on their own, because a block is
        // worth dropping where a control frame is not.
        let (frame_tx, frames) = mpsc::channel::<Frame>(8);
        let (data_tx, inline) = mpsc::channel::<Vec<u8>>(8);
        tokio::spawn(async move {
            while let Ok(Some(inbound)) = read_inbound(&mut read_half).await {
                let sent = match inbound {
                    Inbound::Control(frame) => frame_tx.send(frame).await.is_ok(),
                    Inbound::Data(bytes) => data_tx.send(bytes).await.is_ok(),
                };
                if !sent {
                    return;
                }
            }
        });

        let mut ping = tokio::time::interval(config.ping_interval);
        ping.tick().await;
        // Often enough that the whole ladder of punches is spent inside the
        // fallback timeout, and cheap: twelve bytes each.
        let mut punching =
            tokio::time::interval((config.udp_timeout / 6).max(Duration::from_millis(50)));
        punching.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        let fallback_at = match (transport, config.transport, punch_to) {
            (Transport::Udp, Prefer::Auto, Some(_)) => {
                Some(tokio::time::Instant::now() + config.udp_timeout)
            }
            _ => None,
        };

        let mut stream = IqStream {
            udp,
            punch_to,
            token,
            transport,
            fallback_at,
            heard: false,
            punching,
            control: write_half,
            frames,
            inline,
            info,
            available: streams,
            config,
            assembler: Assembler::default(),
            ended: false,
            stats: Stats::default(),
            ping,
            buf: vec![0u8; 65536],
            decoded: Vec::new(),
        };
        stream.punch().await;
        Ok(stream)
    }

    /// Open the hole, and keep it open. Costs twelve bytes and is what makes
    /// the server able to reach a client it cannot address.
    async fn punch(&mut self) {
        if let (Transport::Udp, Some(udp), Some(to)) = (self.transport, &self.udp, self.punch_to) {
            let _ = udp.send_to(&encode_punch(self.token), to).await;
        }
    }

    /// Ask for the samples on the control connection instead, because none
    /// arrived over UDP.
    ///
    /// A fresh subscribe rather than a message of its own: the server already
    /// replaces a subscription to the same tuner, and everything the first
    /// one asked for has to be said again anyway.
    async fn fall_back(&mut self) -> Result<()> {
        tracing::info!(
            "iqstream: nothing over UDP in {:?}, asking for the samples on the control connection",
            self.config.udp_timeout
        );
        self.transport = Transport::Tcp;
        self.fallback_at = None;
        self.udp = None;
        // The new subscription counts from zero, and what the old one was
        // waiting on will never arrive.
        self.assembler = Assembler::default();
        let bits = BitDepth::new(self.info.bit_depth)?;
        let sub =
            subscribe_frame(&self.config, bits, self.info.id, Transport::Tcp, self.token, None);
        self.control.write_all(&sub.encode()).await.map_err(other)
    }

    pub fn info(&self) -> &StreamInfo {
        &self.info
    }

    /// Every tuner this server offers, as its welcome listed them.
    pub fn available(&self) -> &[StreamDesc] {
        &self.available
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    pub fn local_port(&self) -> u16 {
        self.udp.as_ref().and_then(|u| u.local_addr().ok()).map(|a| a.port()).unwrap_or(0)
    }

    /// What the samples are travelling on now, which is not always what was
    /// asked for: a stream that heard nothing over UDP is on TCP.
    pub fn transport(&self) -> Transport {
        self.transport
    }

    /// Ask the server to move its tuner.
    ///
    /// Returns as soon as the request is on the wire. Where the tuner landed
    /// arrives later as a [`msg::TUNED`] and shows up in [`Block::center_hz`]
    /// and [`StreamInfo::center_hz`], because a dongle steps in units of its
    /// own and the answer is not always what was asked for.
    pub async fn tune(&mut self, center_hz: u64) -> Result<()> {
        if !self.info.tunable {
            return Err(Error::other("this server will not be tuned from here"));
        }
        if let Some((lo, hi)) = self.info.tune_range_hz
            && !(lo..=hi).contains(&center_hz)
        {
            return Err(Error::other(format!(
                "{:.4} MHz is outside the far end's {:.4} to {:.4} MHz",
                center_hz as f64 / 1e6,
                lo as f64 / 1e6,
                hi as f64 / 1e6
            )));
        }
        let mut t = Tlvs::new();
        t.u16(tag::STREAM_ID, self.info.id).u64(tag::CENTER_HZ, center_hz);
        self.control.write_all(&Frame::new(msg::TUNE, &t).encode()).await.map_err(other)
    }

    /// Ask the far end to set one of this tuner's settings.
    ///
    /// Returns as soon as the request is on the wire, like [`IqStream::tune`]
    /// and for the same reason: what the radio really became arrives later as
    /// a [`msg::STREAM_CHANGED`] and shows up in [`StreamInfo::settings`],
    /// because a driver snaps a gain to its own step.
    pub async fn set_setting(&mut self, name: &str, value: SettingValue) -> Result<()> {
        if !self.info.tunable {
            return Err(Error::other("this server will not be set from here"));
        }
        if !self.info.settings.iter().any(|s| s.name == name) {
            return Err(Error::other(format!("that tuner has no setting called {name:?}")));
        }
        let mut t = Tlvs::new();
        t.u16(tag::STREAM_ID, self.info.id)
            .str(tag::SETTING_NAME, name)
            .str(tag::SETTING_VALUE, &value.text());
        self.control.write_all(&Frame::new(msg::SET_SETTING, &t).encode()).await.map_err(other)
    }

    /// Next complete block, or `None` when the server closes the control
    /// connection. Cancellation safe: dropping the future loses nothing.
    pub async fn next_block(&mut self) -> Result<Option<Block>> {
        loop {
            if self.ended {
                return Ok(None);
            }
            tokio::select! {
                _ = self.ping.tick() => {
                    let mut t = Tlvs::new();
                    t.u64(tag::TIMESTAMP_NS, now_ns());
                    self.control.write_all(&Frame::new(msg::PING, &t).encode()).await.map_err(other)?;
                    // The mapping the punch opened closes on a NAT that has
                    // seen nothing go out of it for a minute or two.
                    self.punch().await;
                }
                // Only while nothing has arrived: the first punches are the
                // ones that matter, and after that the keepalive carries it.
                _ = self.punching.tick(), if !self.heard => self.punch().await,
                _ = sleep_until(self.fallback_at), if self.fallback_at.is_some() => {
                    self.fall_back().await?;
                }
                frame = self.frames.recv() => match frame {
                    None => return Ok(None),
                    Some(frame) => self.handle_control(frame).await?,
                },
                // A whole block off the control connection, which is where a
                // subscription that gave up on datagrams reads them.
                record = self.inline.recv() => {
                    let Some(bytes) = record else { return Ok(None) };
                    self.stats.datagrams += 1;
                    self.heard = true;
                    self.fallback_at = None;
                    if let Some(block) = take_datagram(
                        &bytes,
                        &self.info,
                        &self.config,
                        &mut self.assembler,
                        &mut self.stats,
                        &mut self.decoded,
                    )? {
                        return Ok(Some(block));
                    }
                }
                received = recv(self.udp.as_ref(), &mut self.buf), if self.udp.is_some() => {
                    let n = received.map_err(other)?;
                    // A probe is worth nothing except that it arrived: the
                    // server sizes its datagrams by which of them are
                    // answered, so this says so and reads on.
                    if let Some((token, size)) = decode_probe(&self.buf[..n]) {
                        if token == self.token {
                            let mut t = Tlvs::new();
                            t.u16(tag::STREAM_ID, self.info.id).u16(tag::PROBE_SIZE, size);
                            let probed = Frame::new(msg::PROBED, &t).encode();
                            self.control.write_all(&probed).await.map_err(other)?;
                        }
                        continue;
                    }
                    self.stats.datagrams += 1;
                    // The hole is open and the samples are coming through it.
                    self.heard = true;
                    self.fallback_at = None;
                    if let Some(block) = take_datagram(
                        &self.buf[..n],
                        &self.info,
                        &self.config,
                        &mut self.assembler,
                        &mut self.stats,
                        &mut self.decoded,
                    )? {
                        return Ok(Some(block));
                    }
                }
            }
        }
    }

    async fn handle_control(&mut self, frame: Frame) -> Result<()> {
        match frame.msg_type {
            // Answer the server's keepalive, otherwise it drops the stream.
            msg::PING => {
                let mut t = Tlvs::new();
                if let Some(ts) = frame.tlvs()?.u64(tag::TIMESTAMP_NS) {
                    t.u64(tag::TIMESTAMP_NS, ts);
                }
                self.control.write_all(&Frame::new(msg::PONG, &t).encode()).await.map_err(other)?;
            }
            msg::PONG => {
                if let Some(ts) = frame.tlvs()?.u64(tag::TIMESTAMP_NS) {
                    self.stats.rtt_ms = Some(now_ns().saturating_sub(ts) as f64 / 1e6);
                }
            }
            // Somebody moved the tuner, possibly not us. From here on the
            // samples are of somewhere else, so the reading changes before the
            // next block leaves rather than after.
            msg::TUNED => {
                let t = frame.tlvs()?;
                let mine = t.u16(tag::STREAM_ID).is_none_or(|id| id == self.info.id);
                if let (true, Some(hz)) = (mine, t.u64(tag::CENTER_HZ)) {
                    self.info.center_hz = hz;
                }
                for s in &mut self.available {
                    if t.u16(tag::STREAM_ID).is_none_or(|id| id == s.id)
                        && let Some(hz) = t.u64(tag::CENTER_HZ)
                    {
                        s.center_hz = hz;
                    }
                }
            }
            // The set of tuners changed: one was plugged in, or the receiver
            // at the far end stopped serving one.
            msg::STREAMS => self.available = read_streams(&frame.tlvs()?),
            // One tuner is set differently than it was: a gain moved, a bias
            // tee went off, the antenna port changed. What it says about the
            // one being read replaces what the welcome said, because from
            // here on the samples were taken at the new setting.
            msg::STREAM_CHANGED => {
                for desc in read_streams(&frame.tlvs()?) {
                    if desc.id == self.info.id {
                        self.info.center_hz = desc.center_hz;
                        self.info.sample_rate = desc.sample_rate;
                        self.info.gain_db = desc.gain_db;
                        self.info.tunable = desc.tunable;
                        self.info.tune_range_hz = desc.tune_range_hz;
                        self.info.settings = desc.settings.clone();
                    }
                    match self.available.iter_mut().find(|s| s.id == desc.id) {
                        Some(known) => *known = desc,
                        None => self.available.push(desc),
                    }
                }
            }
            // The subscription is over: asked for, or the tuner taken off
            // the server by whoever owns it. Either way no more samples of
            // it will arrive, and a reader waiting for them would wait for
            // ever, because the connection itself is still up.
            msg::UNSUBSCRIBED => {
                if frame.tlvs()?.u16(tag::STREAM_ID).is_none_or(|id| id == self.info.id) {
                    self.ended = true;
                }
            }
            // A refused tune is not a dead subscription: the samples keep
            // coming from wherever the tuner already was.
            msg::ERROR => tracing::warn!("iqstream: {}", describe_error(&frame)),
            _ => {}
        }
        Ok(())
    }

    /// Stop the stream cleanly. Dropping the client also works, but this tells
    /// the server immediately instead of leaving it to the keepalive.
    pub async fn unsubscribe(&mut self) -> Result<()> {
        let mut t = Tlvs::new();
        t.u16(tag::STREAM_ID, self.info.id);
        self.control.write_all(&Frame::new(msg::UNSUBSCRIBE, &t).encode()).await.map_err(other)
    }
}

/// A datagram, however it arrived: one block of samples once every fragment
/// of it is in, and nothing until then.
fn take_datagram(
    datagram: &[u8],
    info: &StreamInfo,
    config: &ClientConfig,
    assembler: &mut Assembler,
    stats: &mut Stats,
    decoded: &mut Vec<u8>,
) -> Result<Option<Block>> {
    let Ok((header, payload)) = DataHeader::decode(datagram) else {
        return Ok(None);
    };
    // Another tuner on the same server, reaching this port because a
    // subscription was replaced and the old pump had a datagram already in
    // flight.
    if header.stream_id != info.id {
        return Ok(None);
    }
    let Some(body) = assembler.push(&header, payload, stats) else {
        return Ok(None);
    };
    decode_block(&header, &body, decoded)?;
    stats.blocks += 1;

    let padded = assembler.take_pending_gap().unwrap_or(0);
    let pad = config.pad_gaps && padded > 0;
    let mut samples = Vec::with_capacity(decoded.len() + if pad { padded as usize * 2 } else { 0 });
    if pad {
        stats.padded_samples += padded;
        // 0x80 is mid scale for UC8: silence, not a full scale step that a
        // demodulator would see as a pulse.
        samples.resize(padded as usize * 2, 0x80);
    }
    samples.extend_from_slice(decoded);
    Ok(Some(Block {
        sample_index: header.sample_index,
        samples,
        padded_before: padded,
        center_hz: info.center_hz,
        stream_id: header.stream_id,
    }))
}

async fn recv(sock: Option<&UdpSocket>, buf: &mut [u8]) -> std::io::Result<usize> {
    match sock {
        Some(sock) => sock.recv(buf).await,
        None => std::future::pending().await,
    }
}

async fn sleep_until(at: Option<tokio::time::Instant>) {
    match at {
        Some(at) => tokio::time::sleep_until(at).await,
        None => std::future::pending().await,
    }
}

/// Reassembles fragmented blocks. A block is emitted once every fragment has
/// arrived; a block still missing fragments when a newer one completes is
/// abandoned, because a late block is worthless to a decoder.
#[derive(Default)]
struct Assembler {
    blocks: BTreeMap<u32, Partial>,
    next_sample: Option<u64>,
    pending_gap: Option<u64>,
}

struct Partial {
    fragments: BTreeMap<u16, Vec<u8>>,
    frag_count: u16,
    sample_index: u64,
    block_samples: u32,
}

impl Assembler {
    fn push(&mut self, header: &DataHeader, payload: &[u8], stats: &mut Stats) -> Option<Vec<u8>> {
        let entry = self.blocks.entry(header.block_seq).or_insert_with(|| Partial {
            fragments: BTreeMap::new(),
            frag_count: header.frag_count,
            sample_index: header.sample_index,
            block_samples: header.block_samples,
        });
        entry.fragments.insert(header.frag_index, payload.to_vec());
        if entry.fragments.len() < entry.frag_count as usize {
            return None;
        }

        let done = self.blocks.remove(&header.block_seq)?;
        // Anything older than the block just completed will never complete.
        let stale: Vec<u32> = self
            .blocks
            .keys()
            .copied()
            .filter(|seq| header.block_seq.wrapping_sub(*seq) < u32::MAX / 2)
            .collect();
        for seq in stale {
            self.blocks.remove(&seq);
            stats.incomplete_blocks += 1;
        }

        if let Some(expected) = self.next_sample
            && done.sample_index > expected
        {
            self.pending_gap = Some(done.sample_index - expected);
        }
        self.next_sample = Some(done.sample_index + done.block_samples as u64);

        let mut body = Vec::new();
        for (_, frag) in done.fragments {
            body.extend_from_slice(&frag);
        }
        Some(body)
    }

    fn take_pending_gap(&mut self) -> Option<u64> {
        self.pending_gap.take()
    }
}

fn decode_block(header: &DataHeader, body: &[u8], out: &mut Vec<u8>) -> Result<()> {
    let values = header.block_samples as usize * 2;
    let packed_len = BitDepth::new(header.bit_depth)?.packed_len(header.block_samples as usize);
    let decompressed;
    let packed = match header.codec {
        Codec::None => body,
        Codec::Zstd => {
            decompressed = zstd::bulk::decompress(body, packed_len + 1024).map_err(other)?;
            &decompressed
        }
    };
    unpack(packed, header.bit_depth, values, out);
    Ok(())
}

pub fn describe_error(frame: &Frame) -> String {
    if frame.msg_type != msg::ERROR {
        return format!("unexpected message type {:#04x}", frame.msg_type);
    }
    match frame.tlvs() {
        Ok(t) => format!(
            "server error {}: {}",
            t.u16(tag::ERROR_CODE).unwrap_or(0),
            t.str(tag::ERROR_MESSAGE).unwrap_or_default()
        ),
        Err(e) => format!("undecodable error frame: {e}"),
    }
}

/// What comes off the control connection: a frame, or a whole datagram where
/// the samples are travelling on it.
enum Inbound {
    Control(Frame),
    Data(Vec<u8>),
}

/// One frame or one inline record, told apart by the first four bytes: a
/// frame opens with a protocol version, which the magic cannot be.
async fn read_inbound(sock: &mut OwnedReadHalf) -> Result<Option<Inbound>> {
    let mut head = [0u8; 4];
    match sock.read_exact(&mut head).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(other(e)),
    }
    if head == INLINE_MAGIC {
        let mut len = [0u8; 4];
        sock.read_exact(&mut len).await.map_err(other)?;
        let len = u32::from_le_bytes(len) as usize;
        if !(DATA_HEADER_LEN..=MAX_INLINE_RECORD).contains(&len) {
            return Err(Error::other(format!("inline record of {len} bytes")));
        }
        let mut datagram = vec![0u8; len];
        sock.read_exact(&mut datagram).await.map_err(other)?;
        return Ok(Some(Inbound::Data(datagram)));
    }
    let len = u16::from_le_bytes([head[2], head[3]]) as usize;
    if len > MAX_FRAME_PAYLOAD {
        return Err(Error::other(format!("control frame too large: {len}")));
    }
    let mut payload = vec![0u8; len];
    sock.read_exact(&mut payload).await.map_err(other)?;
    Ok(Some(Inbound::Control(Frame { version: head[0], msg_type: head[1], payload })))
}

pub async fn read_frame(sock: &mut OwnedReadHalf) -> Result<Option<Frame>> {
    match read_inbound(sock).await? {
        Some(Inbound::Control(frame)) => Ok(Some(frame)),
        Some(Inbound::Data(_)) => Err(Error::other("samples before a subscription")),
        None => Ok(None),
    }
}
