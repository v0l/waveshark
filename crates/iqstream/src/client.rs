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
    BitDepth, Frame, PREAMBLE_LEN, SettingValue, StreamDesc, Tlvs, Transport, decode_probe,
    encode_preamble, encode_punch, msg, now_ns, tag,
};
use crate::subscribe::{
    Answer, Assembler, Head, Inbound, greeted, head, heard, hello, inline_len, offered, set_frame,
    setting_frame, subscribe_frame, subscribed, take_datagram, token, tune_frame,
    unsubscribe_frame, wanted, welcomed,
};
pub use crate::subscribe::{Block, ClientConfig, Prefer, Stats, StreamInfo, describe_error};
use crate::ws;
use common::time::Duration;
use common::{Error, Result};
use std::net::SocketAddr;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpStream, UdpSocket};
use tokio::sync::mpsc;

fn other(e: impl std::fmt::Display) -> Error {
    Error::other(e.to_string())
}

fn data_port(welcome: &Frame) -> Option<u16> {
    welcome.tlvs().ok()?.u16(tag::DATA_PORT)
}

fn ping_frame() -> Frame {
    let mut t = Tlvs::new();
    t.u64(tag::TIMESTAMP_NS, now_ns());
    Frame::new(msg::PING, &t)
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
    control: ws::Write,
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
    read_half: ws::Read,
    write_half: ws::Write,
    welcome: Frame,
    streams: Vec<StreamDesc>,
    /// Where to punch, and the sign that this server knows how to be: a
    /// server that named no data port is a 1.3 one, and is told a port.
    punch_to: Option<SocketAddr>,
    over_ws: bool,
}

enum Pipe<'a> {
    Named(&'a str),
    Open(ws::Read, ws::Write),
}

/// Connect, say hello, and take the welcome.
///
/// A 1.1 server lists no tuners and describes its one stream in the flat
/// tags, so one is made out of those: everything above here then reads a set
/// of tuners whatever the far end's version.
async fn greet(server: &str, name: &str) -> Result<Greeting> {
    let over_ws = ws::is_url(server);
    let (read_half, write_half, peer) = match over_ws {
        true => {
            let (r, w) = match ws::is_webtransport(server) {
                true => crate::wt::connect(server).await?,
                false => ws::connect(server).await?,
            };
            (r, w, None)
        }
        false => {
            let control = TcpStream::connect(server).await.map_err(other)?;
            control.set_nodelay(true).map_err(other)?;
            let peer = control.peer_addr().map_err(other)?;
            let (r, w) = control.into_split();
            (Box::new(r) as ws::Read, Box::new(w) as ws::Write, Some(peer))
        }
    };
    greet_on(read_half, write_half, peer, name).await
}

async fn greet_on(
    mut read_half: ws::Read,
    mut write_half: ws::Write,
    peer: Option<SocketAddr>,
    name: &str,
) -> Result<Greeting> {
    let over_ws = peer.is_none();
    write_half.write_all(&encode_preamble()).await.map_err(other)?;
    let mut preamble = [0u8; PREAMBLE_LEN];
    read_half.read_exact(&mut preamble).await.map_err(other)?;
    greeted(&preamble)?;
    write_half.write_all(&hello(name).encode()).await.map_err(other)?;
    let welcome = read_frame(&mut read_half)
        .await?
        .ok_or_else(|| Error::other("server closed before welcome"))?;
    let streams = welcomed(&welcome)?;
    let punch_to = peer.zip(data_port(&welcome)).map(|(mut to, port)| {
        to.set_port(port);
        to
    });
    Ok(Greeting { read_half, write_half, welcome, streams, punch_to, over_ws })
}

/// What tuners a server has, without subscribing to any of them.
///
/// What builds a radio list: a server with three dongles on it is three
/// entries, and each says where it is and how far its dial goes.
pub async fn set(
    server: &str,
    name: &str,
    stream: u16,
    setting: &str,
    value: SettingValue,
) -> Result<()> {
    let mut g = greet(server, name).await?;
    let set = set_frame(stream, setting, &value);
    g.write_half.write_all(&set.encode()).await.map_err(other)?;
    g.write_half.shutdown().await.map_err(other)?;
    let mut rest = Vec::new();
    let _ = g.read_half.read_to_end(&mut rest).await;
    Ok(())
}

pub async fn list(server: &str, name: &str) -> Result<Vec<StreamDesc>> {
    Ok(greet(server, name).await?.streams)
}

impl IqStream {
    pub async fn connect(server: &str, config: ClientConfig) -> Result<Self> {
        Self::subscribe(Pipe::Named(server), config).await
    }

    pub async fn connect_over(
        read: ws::Read,
        write: ws::Write,
        config: ClientConfig,
    ) -> Result<Self> {
        Self::subscribe(Pipe::Open(read, write), config).await
    }

    async fn subscribe(pipe: Pipe<'_>, mut config: ClientConfig) -> Result<Self> {
        let bit_depth = BitDepth::new(config.bits)?;
        if !matches!(pipe, Pipe::Named(server) if !ws::is_url(server)) {
            config.transport = Prefer::Tcp;
        }

        // A port that will not bind is a machine that is not going to carry
        // datagrams, so it is the same answer as a path that drops them: ask
        // for the samples on the connection that already works.
        let udp = match config.transport {
            Prefer::Tcp => None,
            _ => UdpSocket::bind(("0.0.0.0", config.local_port)).await.ok(),
        };

        let greeting = match pipe {
            Pipe::Named(server) => greet(server, &config.name).await?,
            Pipe::Open(read, write) => greet_on(read, write, None, &config.name).await?,
        };
        let Greeting { mut read_half, mut write_half, welcome, streams, punch_to, over_ws } =
            greeting;
        offered(&welcome, &config, bit_depth)?;
        let wanted = wanted(&streams, &config)?;

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
        if transport == Transport::Tcp && punch_to.is_none() && !over_ws {
            return Err(Error::other("this server cannot carry samples on the control connection"));
        }
        let sub = subscribe_frame(&config, bit_depth, wanted.id, transport, token, local_port);
        write_half.write_all(&sub.encode()).await.map_err(other)?;
        let reply = read_frame(&mut read_half)
            .await?
            .ok_or_else(|| Error::other("server closed during subscribe"))?;
        let info = subscribed(&reply, &wanted, &config, bit_depth)?;

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
        let tune = tune_frame(&self.info, center_hz)?;
        self.control.write_all(&tune.encode()).await.map_err(other)
    }

    /// Ask the far end to set one of this tuner's settings.
    ///
    /// Returns as soon as the request is on the wire, like [`IqStream::tune`]
    /// and for the same reason: what the radio really became arrives later as
    /// a [`msg::STREAM_CHANGED`] and shows up in [`StreamInfo::settings`],
    /// because a driver snaps a gain to its own step.
    pub async fn set_setting(&mut self, name: &str, value: SettingValue) -> Result<()> {
        let set = setting_frame(&self.info, name, &value)?;
        self.control.write_all(&set.encode()).await.map_err(other)
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
                    self.control.write_all(&ping_frame().encode()).await.map_err(other)?;
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
        match heard(&frame, &mut self.info, &mut self.available, &mut self.stats)? {
            Answer::Reply(reply) => self.control.write_all(&reply.encode()).await.map_err(other)?,
            Answer::Ended => self.ended = true,
            Answer::Nothing => {}
        }
        Ok(())
    }

    /// Stop the stream cleanly. Dropping the client also works, but this tells
    /// the server immediately instead of leaving it to the keepalive.
    pub async fn unsubscribe(&mut self) -> Result<()> {
        let unsubscribe = unsubscribe_frame(&self.info);
        self.control.write_all(&unsubscribe.encode()).await.map_err(other)
    }
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

async fn read_inbound(sock: &mut ws::Read) -> Result<Option<Inbound>> {
    let mut first = [0u8; 4];
    match sock.read_exact(&mut first).await {
        Ok(_) => {}
        Err(e) if e.kind() == std::io::ErrorKind::UnexpectedEof => return Ok(None),
        Err(e) => return Err(other(e)),
    }
    match head(first)? {
        Head::Inline => {
            let mut len = [0u8; 4];
            sock.read_exact(&mut len).await.map_err(other)?;
            let mut datagram = vec![0u8; inline_len(len)?];
            sock.read_exact(&mut datagram).await.map_err(other)?;
            Ok(Some(Inbound::Data(datagram)))
        }
        Head::Frame { version, msg_type, len } => {
            let mut payload = vec![0u8; len];
            sock.read_exact(&mut payload).await.map_err(other)?;
            Ok(Some(Inbound::Control(Frame { version, msg_type, payload })))
        }
    }
}

pub async fn read_frame(sock: &mut ws::Read) -> Result<Option<Frame>> {
    match read_inbound(sock).await? {
        Some(Inbound::Control(frame)) => Ok(Some(frame)),
        Some(Inbound::Data(_)) => Err(Error::other("samples before a subscription")),
        None => Ok(None),
    }
}
