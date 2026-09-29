use crate::proto::{
    BitDepth, Codec, DATA_HEADER_LEN, DataHeader, Frame, INLINE_MAGIC, MAX_FRAME_PAYLOAD,
    MAX_INLINE_RECORD, PREAMBLE_LEN, Setting, SettingValue, StreamDesc, Tlvs, Transport,
    VERSION_MAJOR, VERSION_MINOR, decode_preamble, msg, now_ns, read_streams, tag, unpack,
};
use common::time::Duration;
use common::{Error, Result};
use std::collections::BTreeMap;
use std::sync::atomic::{AtomicU64, Ordering};

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
    /// Whether the server will accept a tune. False for one sharing a tuner
    /// somebody else owns.
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

/// A token no other subscription on this server will be using.
///
/// The clock and a counter, mixed, which is enough: it is not a secret and
/// nothing turns on guessing it, only on two subscriptions of one client not
/// colliding.
pub(crate) fn token() -> u64 {
    static COUNT: AtomicU64 = AtomicU64::new(0);
    let mut v =
        now_ns() ^ (COUNT.fetch_add(1, Ordering::Relaxed).wrapping_mul(0x9e37_79b9_7f4a_7c15));
    v ^= v >> 30;
    v = v.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    v ^= v >> 27;
    v.wrapping_mul(0x94d0_49bb_1331_11eb)
}

pub(crate) fn greeted(preamble: &[u8; PREAMBLE_LEN]) -> Result<()> {
    let (major, minor) = decode_preamble(preamble)?;
    match major == VERSION_MAJOR {
        true => Ok(()),
        false => Err(Error::other(format!(
            "server speaks version {major}.{minor}, this speaks {VERSION_MAJOR}.{VERSION_MINOR}"
        ))),
    }
}

pub(crate) fn hello(name: &str) -> Frame {
    let mut hello = Tlvs::new();
    hello.str(tag::CLIENT_NAME, name);
    Frame::new(msg::HELLO, &hello)
}

/// A 1.1 server lists no tuners and describes its one stream in the flat
/// tags, so one is made out of those: everything above here then reads a set
/// of tuners whatever the far end's version.
pub(crate) fn welcomed(welcome: &Frame) -> Result<Vec<StreamDesc>> {
    if welcome.msg_type != msg::WELCOME {
        return Err(Error::other(describe_error(welcome)));
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
    Ok(streams)
}

pub(crate) fn offered(welcome: &Frame, config: &ClientConfig, bits: BitDepth) -> Result<()> {
    let w = welcome.tlvs()?;
    if let Some(depths) = w.get(tag::SUPPORTED_BIT_DEPTHS)
        && !depths.contains(&bits.0)
    {
        return Err(Error::other(format!("server does not offer {} bit samples", bits.0)));
    }
    if let Some(codecs) = w.get(tag::SUPPORTED_CODECS)
        && !codecs.contains(&config.codec.code())
    {
        return Err(Error::other(format!("server does not offer codec {:?}", config.codec)));
    }
    Ok(())
}

pub(crate) fn wanted(streams: &[StreamDesc], config: &ClientConfig) -> Result<StreamDesc> {
    match config.stream {
        Some(id) => streams
            .iter()
            .find(|s| s.id == id)
            .ok_or_else(|| Error::other(format!("server has no tuner {id}"))),
        None => streams.first().ok_or_else(|| Error::other("server offers no tuner")),
    }
    .cloned()
}

/// One subscribe, which says the same things whichever transport it asks for.
pub(crate) fn subscribe_frame(
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

pub(crate) fn subscribed(
    reply: &Frame,
    wanted: &StreamDesc,
    config: &ClientConfig,
    bits: BitDepth,
) -> Result<StreamInfo> {
    if reply.msg_type != msg::SUBSCRIBED {
        return Err(Error::other(format!("subscribe refused: {}", describe_error(reply))));
    }
    let r = reply.tlvs()?;
    Ok(StreamInfo {
        id: wanted.id,
        name: wanted.name.clone(),
        center_hz: wanted.center_hz,
        sample_rate: wanted.sample_rate,
        gain_db: wanted.gain_db,
        tunable: wanted.tunable,
        tune_range_hz: wanted.tune_range_hz,
        settings: wanted.settings.clone(),
        bit_depth: r.u8(tag::BIT_DEPTH).unwrap_or(bits.0),
        codec: Codec::from_code(r.u8(tag::CODEC).unwrap_or(config.codec.code()))?,
        block_samples: r.u32(tag::BLOCK_SAMPLES).unwrap_or(0),
    })
}

pub(crate) fn set_frame(stream: u16, setting: &str, value: &SettingValue) -> Frame {
    let mut t = Tlvs::new();
    t.u16(tag::STREAM_ID, stream)
        .str(tag::SETTING_NAME, setting)
        .str(tag::SETTING_VALUE, &value.text());
    Frame::new(msg::SET_SETTING, &t)
}

pub(crate) fn tune_frame(info: &StreamInfo, center_hz: u64) -> Result<Frame> {
    if !info.tunable {
        return Err(Error::other("this server will not be tuned from here"));
    }
    if let Some((lo, hi)) = info.tune_range_hz
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
    t.u16(tag::STREAM_ID, info.id).u64(tag::CENTER_HZ, center_hz);
    Ok(Frame::new(msg::TUNE, &t))
}

pub(crate) fn setting_frame(info: &StreamInfo, name: &str, value: &SettingValue) -> Result<Frame> {
    if !info.tunable {
        return Err(Error::other("this server will not be set from here"));
    }
    if !info.settings.iter().any(|s| s.name == name) {
        return Err(Error::other(format!("that tuner has no setting called {name:?}")));
    }
    Ok(set_frame(info.id, name, value))
}

pub(crate) fn unsubscribe_frame(info: &StreamInfo) -> Frame {
    let mut t = Tlvs::new();
    t.u16(tag::STREAM_ID, info.id);
    Frame::new(msg::UNSUBSCRIBE, &t)
}

pub(crate) enum Answer {
    Nothing,
    Reply(Frame),
    Ended,
}

pub(crate) fn heard(
    frame: &Frame,
    info: &mut StreamInfo,
    available: &mut Vec<StreamDesc>,
    stats: &mut Stats,
) -> Result<Answer> {
    match frame.msg_type {
        // Answer the server's keepalive, otherwise it drops the stream.
        msg::PING => {
            let mut t = Tlvs::new();
            if let Some(ts) = frame.tlvs()?.u64(tag::TIMESTAMP_NS) {
                t.u64(tag::TIMESTAMP_NS, ts);
            }
            return Ok(Answer::Reply(Frame::new(msg::PONG, &t)));
        }
        msg::PONG => {
            if let Some(ts) = frame.tlvs()?.u64(tag::TIMESTAMP_NS) {
                stats.rtt_ms = Some(now_ns().saturating_sub(ts) as f64 / 1e6);
            }
        }
        // Somebody moved the tuner, possibly not us. From here on the
        // samples are of somewhere else, so the reading changes before the
        // next block leaves rather than after.
        msg::TUNED => {
            let t = frame.tlvs()?;
            let mine = t.u16(tag::STREAM_ID).is_none_or(|id| id == info.id);
            if let (true, Some(hz)) = (mine, t.u64(tag::CENTER_HZ)) {
                info.center_hz = hz;
            }
            for s in available.iter_mut() {
                if t.u16(tag::STREAM_ID).is_none_or(|id| id == s.id)
                    && let Some(hz) = t.u64(tag::CENTER_HZ)
                {
                    s.center_hz = hz;
                }
            }
        }
        // The set of tuners changed: one was plugged in, or the receiver
        // at the far end stopped serving one.
        msg::STREAMS => *available = read_streams(&frame.tlvs()?),
        // One tuner is set differently than it was: a gain moved, a bias
        // tee went off, the antenna port changed. What it says about the
        // one being read replaces what the welcome said, because from
        // here on the samples were taken at the new setting.
        msg::STREAM_CHANGED => {
            for desc in read_streams(&frame.tlvs()?) {
                if desc.id == info.id {
                    info.center_hz = desc.center_hz;
                    info.sample_rate = desc.sample_rate;
                    info.gain_db = desc.gain_db;
                    info.tunable = desc.tunable;
                    info.tune_range_hz = desc.tune_range_hz;
                    info.settings = desc.settings.clone();
                }
                match available.iter_mut().find(|s| s.id == desc.id) {
                    Some(known) => *known = desc,
                    None => available.push(desc),
                }
            }
        }
        // The subscription is over: asked for, or the tuner taken off
        // the server by whoever owns it. Either way no more samples of
        // it will arrive, and a reader waiting for them would wait for
        // ever, because the connection itself is still up.
        msg::UNSUBSCRIBED => {
            if frame.tlvs()?.u16(tag::STREAM_ID).is_none_or(|id| id == info.id) {
                return Ok(Answer::Ended);
            }
        }
        // A refused tune is not a dead subscription: the samples keep
        // coming from wherever the tuner already was.
        msg::ERROR => tracing::warn!("iqstream: {}", describe_error(frame)),
        _ => {}
    }
    Ok(Answer::Nothing)
}

/// A datagram, however it arrived: one block of samples once every fragment
/// of it is in, and nothing until then.
pub(crate) fn take_datagram(
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

/// Reassembles fragmented blocks. A block is emitted once every fragment has
/// arrived; a block still missing fragments when a newer one completes is
/// abandoned, because a late block is worthless to a decoder.
#[derive(Default)]
pub(crate) struct Assembler {
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
    let packed = match header.codec {
        Codec::None => body,
        Codec::Zstd => &unzstd(body, header)?,
    };
    unpack(packed, header.bit_depth, values, out);
    Ok(())
}

#[cfg(all(feature = "net", not(target_arch = "wasm32")))]
fn unzstd(body: &[u8], header: &DataHeader) -> Result<Vec<u8>> {
    let packed_len = BitDepth::new(header.bit_depth)?.packed_len(header.block_samples as usize);
    zstd::bulk::decompress(body, packed_len + 1024).map_err(|e| Error::other(e.to_string()))
}

#[cfg(all(feature = "net", target_arch = "wasm32"))]
fn unzstd(body: &[u8], header: &DataHeader) -> Result<Vec<u8>> {
    unzstd_in_rust(body, header)
}

#[cfg(any(test, all(feature = "net", target_arch = "wasm32")))]
fn unzstd_in_rust(body: &[u8], header: &DataHeader) -> Result<Vec<u8>> {
    let packed_len = BitDepth::new(header.bit_depth)?.packed_len(header.block_samples as usize);
    let mut out = Vec::with_capacity(packed_len + 1024);
    ruzstd::decoding::FrameDecoder::new()
        .decode_all_to_vec(body, &mut out)
        .map_err(|e| Error::other(e.to_string()))?;
    Ok(out)
}

#[cfg(not(feature = "net"))]
fn unzstd(_: &[u8], _: &DataHeader) -> Result<Vec<u8>> {
    Err(Error::other("this build reads no zstd blocks"))
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
pub(crate) enum Inbound {
    Control(Frame),
    Data(Vec<u8>),
}

pub(crate) enum Head {
    Frame { version: u8, msg_type: u8, len: usize },
    Inline,
}

/// One frame or one inline record, told apart by the first four bytes: a
/// frame opens with a protocol version, which the magic cannot be.
pub(crate) fn head(head: [u8; 4]) -> Result<Head> {
    if head == INLINE_MAGIC {
        return Ok(Head::Inline);
    }
    let len = u16::from_le_bytes([head[2], head[3]]) as usize;
    match len > MAX_FRAME_PAYLOAD {
        true => Err(Error::other(format!("control frame too large: {len}"))),
        false => Ok(Head::Frame { version: head[0], msg_type: head[1], len }),
    }
}

pub(crate) fn inline_len(len: [u8; 4]) -> Result<usize> {
    let len = u32::from_le_bytes(len) as usize;
    match (DATA_HEADER_LEN..=MAX_INLINE_RECORD).contains(&len) {
        true => Ok(len),
        false => Err(Error::other(format!("inline record of {len} bytes"))),
    }
}

#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
pub(crate) fn inbound(buf: &[u8]) -> Result<Option<(Inbound, usize)>> {
    let Some(h) = buf.first_chunk::<4>() else { return Ok(None) };
    match head(*h)? {
        Head::Frame { version, msg_type, len } => Ok(buf.get(4..4 + len).map(|payload| {
            (Inbound::Control(Frame { version, msg_type, payload: payload.to_vec() }), 4 + len)
        })),
        Head::Inline => {
            let Some(len) = buf.get(4..8) else { return Ok(None) };
            let len = inline_len(len.try_into().expect("four bytes"))?;
            Ok(buf.get(8..8 + len).map(|d| (Inbound::Data(d.to_vec()), 8 + len)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::encode_inline;

    fn wire() -> (Vec<u8>, Vec<u8>) {
        let mut record = vec![0u8; DATA_HEADER_LEN + 12];
        record.iter_mut().enumerate().for_each(|(i, b)| *b = i as u8);
        let mut inline = Vec::new();
        encode_inline(&record, &mut inline);
        let mut bytes = Frame::empty(msg::PING).encode();
        bytes.extend(&inline);
        bytes.extend(hello("browser").encode());
        (bytes, record)
    }

    fn read(bytes: &[u8], chunk: usize) -> Vec<Inbound> {
        let (mut held, mut out) = (Vec::new(), Vec::new());
        for piece in bytes.chunks(chunk) {
            held.extend_from_slice(piece);
            while let Some((got, used)) = inbound(&held).unwrap() {
                out.push(got);
                held.drain(..used);
            }
        }
        assert!(held.is_empty(), "{} bytes left over at chunks of {chunk}", held.len());
        out
    }

    #[test]
    fn a_stream_cut_anywhere_reads_as_the_same_frames_and_records() {
        let (bytes, record) = wire();
        for chunk in [1, 3, 4, 7, 8, 64, bytes.len()] {
            let got = read(&bytes, chunk);
            assert_eq!(got.len(), 3, "at chunks of {chunk}");
            assert!(
                matches!(&got[0], Inbound::Control(f) if f.msg_type == msg::PING && f.payload.is_empty())
            );
            assert!(matches!(&got[1], Inbound::Data(d) if *d == record));
            assert!(matches!(&got[2], Inbound::Control(f) if f.msg_type == msg::HELLO
                    && f.tlvs().unwrap().str(tag::CLIENT_NAME).as_deref() == Some("browser")));
        }
    }

    #[test]
    fn a_block_zstd_packed_by_the_server_reads_the_same_in_pure_rust() {
        let samples: Vec<u8> = (0..40_960u32)
            .map(|i| (128.0 + 40.0 * (i as f64 * 0.013).sin()) as u8 ^ (i % 3) as u8)
            .collect();
        let packed = zstd::bulk::compress(&samples, 1).unwrap();
        let header = DataHeader {
            version: 1,
            sample_index: 0,
            block_seq: 0,
            frag_index: 0,
            frag_count: 1,
            bit_depth: 8,
            codec: Codec::Zstd,
            decimation: 1,
            block_samples: samples.len() as u32 / 2,
            stream_id: 0,
        };
        assert!(packed.len() < samples.len(), "{} of {} bytes", packed.len(), samples.len());
        assert_eq!(unzstd_in_rust(&packed, &header).unwrap(), samples);
        assert_eq!(unzstd(&packed, &header).unwrap(), samples);
    }

    #[test]
    fn a_record_longer_than_the_protocol_allows_is_refused_before_it_is_read() {
        let mut bytes = INLINE_MAGIC.to_vec();
        bytes.extend(((MAX_INLINE_RECORD + 1) as u32).to_le_bytes());
        assert!(inbound(&bytes).is_err());
        assert!(inbound(&bytes[..6]).unwrap().is_none(), "a length not yet whole waits");
    }
}
