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

use crate::proto::{
    BitDepth, Codec, DataHeader, Frame, MAX_FRAME_PAYLOAD, PREAMBLE_LEN, Tlvs, VERSION_MAJOR,
    VERSION_MINOR, decode_preamble, encode_preamble, msg, now_ns, tag, unpack,
};
use common::{Error, Result};
use std::collections::BTreeMap;
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
    /// Local UDP port. 0 picks a free one.
    pub local_port: u16,
    /// Report lost samples in [`Block::padded_before`] and fill them with mid
    /// scale, so the output keeps real time alignment.
    pub pad_gaps: bool,
    pub ping_interval: Duration,
}

impl Default for ClientConfig {
    fn default() -> Self {
        ClientConfig {
            name: "iqstream".into(),
            bits: 8,
            codec: Codec::None,
            level: 1,
            local_port: 0,
            pad_gaps: true,
            ping_interval: Duration::from_secs(10),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StreamInfo {
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
    udp: UdpSocket,
    control: OwnedWriteHalf,
    frames: mpsc::Receiver<Frame>,
    info: StreamInfo,
    config: ClientConfig,
    assembler: Assembler,
    stats: Stats,
    ping: tokio::time::Interval,
    buf: Vec<u8>,
    decoded: Vec<u8>,
}

impl IqStream {
    pub async fn connect<A: ToSocketAddrs + std::fmt::Debug>(
        server: A,
        config: ClientConfig,
    ) -> Result<Self> {
        let bit_depth = BitDepth::new(config.bits)?;

        let udp = UdpSocket::bind(("0.0.0.0", config.local_port)).await.map_err(other)?;
        let local_port = udp.local_addr().map_err(other)?.port();

        let control = TcpStream::connect(&server).await.map_err(other)?;
        control.set_nodelay(true).map_err(other)?;
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
        hello.str(tag::CLIENT_NAME, &config.name);
        write_half.write_all(&Frame::new(msg::HELLO, &hello).encode()).await.map_err(other)?;
        let welcome = read_frame(&mut read_half)
            .await?
            .ok_or_else(|| Error::other("server closed before welcome"))?;
        if welcome.msg_type != msg::WELCOME {
            return Err(Error::other(describe_error(&welcome)));
        }
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

        let mut sub = Tlvs::new();
        sub.u16(tag::UDP_PORT, local_port)
            .u8(tag::BIT_DEPTH, bit_depth.0)
            .u8(tag::CODEC, config.codec.code())
            .u8(tag::CODEC_LEVEL, config.level as u8)
            .u16(tag::DECIMATION, 1);
        write_half.write_all(&Frame::new(msg::SUBSCRIBE, &sub).encode()).await.map_err(other)?;
        let reply = read_frame(&mut read_half)
            .await?
            .ok_or_else(|| Error::other("server closed during subscribe"))?;
        if reply.msg_type != msg::SUBSCRIBED {
            return Err(Error::other(format!("subscribe refused: {}", describe_error(&reply))));
        }
        let r = reply.tlvs()?;

        let info = StreamInfo {
            center_hz: w.u64(tag::CENTER_HZ).unwrap_or(0),
            sample_rate: w.u32(tag::SAMPLE_RATE).unwrap_or(0),
            gain_db: w.i16(tag::GAIN_DDB).map(|g| g as f32 / 10.0),
            tunable: w.u8(tag::TUNABLE).unwrap_or(0) != 0,
            tune_range_hz: w.u64(tag::TUNE_MIN_HZ).zip(w.u64(tag::TUNE_MAX_HZ)),
            bit_depth: r.u8(tag::BIT_DEPTH).unwrap_or(bit_depth.0),
            codec: Codec::from_code(r.u8(tag::CODEC).unwrap_or(config.codec.code()))?,
            block_samples: r.u32(tag::BLOCK_SAMPLES).unwrap_or(0),
        };

        // read_exact is not cancellation safe, so frames are read by their own
        // task and handed over a channel that select! can poll safely.
        let (frame_tx, frames) = mpsc::channel::<Frame>(8);
        tokio::spawn(async move {
            while let Ok(Some(frame)) = read_frame(&mut read_half).await {
                if frame_tx.send(frame).await.is_err() {
                    return;
                }
            }
        });

        let mut ping = tokio::time::interval(config.ping_interval);
        ping.tick().await;

        Ok(IqStream {
            udp,
            control: write_half,
            frames,
            info,
            config,
            assembler: Assembler::default(),
            stats: Stats::default(),
            ping,
            buf: vec![0u8; 65536],
            decoded: Vec::new(),
        })
    }

    pub fn info(&self) -> &StreamInfo {
        &self.info
    }

    pub fn stats(&self) -> Stats {
        self.stats
    }

    pub fn local_port(&self) -> u16 {
        self.udp.local_addr().map(|a| a.port()).unwrap_or(0)
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
        t.u64(tag::CENTER_HZ, center_hz);
        self.control.write_all(&Frame::new(msg::TUNE, &t).encode()).await.map_err(other)
    }

    /// Next complete block, or `None` when the server closes the control
    /// connection. Cancellation safe: dropping the future loses nothing.
    pub async fn next_block(&mut self) -> Result<Option<Block>> {
        loop {
            tokio::select! {
                _ = self.ping.tick() => {
                    let mut t = Tlvs::new();
                    t.u64(tag::TIMESTAMP_NS, now_ns());
                    self.control.write_all(&Frame::new(msg::PING, &t).encode()).await.map_err(other)?;
                }
                frame = self.frames.recv() => match frame {
                    None => return Ok(None),
                    Some(frame) => self.handle_control(frame).await?,
                },
                received = self.udp.recv(&mut self.buf) => {
                    let n = received.map_err(other)?;
                    let Ok((header, payload)) = DataHeader::decode(&self.buf[..n]) else {
                        continue;
                    };
                    self.stats.datagrams += 1;
                    let Some(body) = self.assembler.push(&header, payload, &mut self.stats) else {
                        continue;
                    };
                    decode_block(&header, &body, &mut self.decoded)?;
                    self.stats.blocks += 1;

                    let padded = self.assembler.take_pending_gap().unwrap_or(0);
                    let pad = self.config.pad_gaps && padded > 0;
                    let mut samples = Vec::with_capacity(
                        self.decoded.len() + if pad { padded as usize * 2 } else { 0 },
                    );
                    if pad {
                        self.stats.padded_samples += padded;
                        // 0x80 is mid scale for UC8: silence, not a full scale
                        // step that a demodulator would see as a pulse.
                        samples.resize(padded as usize * 2, 0x80);
                    }
                    samples.extend_from_slice(&self.decoded);
                    return Ok(Some(Block {
                        sample_index: header.sample_index,
                        samples,
                        padded_before: padded,
                        center_hz: self.info.center_hz,
                    }));
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
                if let Some(hz) = frame.tlvs()?.u64(tag::CENTER_HZ) {
                    self.info.center_hz = hz;
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
        self.control.write_all(&Frame::empty(msg::UNSUBSCRIBE).encode()).await.map_err(other)
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

pub async fn read_frame(sock: &mut OwnedReadHalf) -> Result<Option<Frame>> {
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
