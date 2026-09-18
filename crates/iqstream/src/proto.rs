//! Binary wire format.
//!
//! Control runs over TCP as versioned frames carrying TLV payloads. Data runs
//! over UDP as self-describing datagrams. Everything is little-endian.
//!
//! Unknown TLV tags must be ignored, which is what makes a field additive: a
//! new server can talk to an old client as long as the major version matches.
//!
//! Vendored from <https://github.com/v0l/iqstream> so both ends of the
//! protocol can be spoken from here. 1.1 adds [`msg::TUNE`]; everything else
//! is 1.0 unchanged, and the two interoperate.

use common::{Error, Result};

macro_rules! bail {
    ($($t:tt)*) => {
        return Err(Error::other(format!($($t)*)))
    };
}

pub const CONTROL_MAGIC: [u8; 4] = *b"IQSC";
pub const DATA_MAGIC: [u8; 4] = *b"IQSD";

/// Bumped for an incompatible change. Both ends must agree.
pub const VERSION_MAJOR: u16 = 1;
/// Bumped when tags or message types are added. The lower of the two wins.
///
/// 1: [`msg::TUNE`] and [`msg::TUNED`], with [`tag::TUNE_MIN_HZ`] and
/// [`tag::TUNE_MAX_HZ`] in the welcome. A 1.0 server answers an unknown
/// message type with [`msg::ERROR`] and keeps the subscription, and ignores
/// tags it does not know, so the two versions interoperate in both
/// directions.
pub const VERSION_MINOR: u16 = 1;

pub const PREAMBLE_LEN: usize = 8;
pub const FRAME_HEADER_LEN: usize = 4;
pub const MAX_FRAME_PAYLOAD: usize = 8192;
pub const DATA_HEADER_LEN: usize = 32;

/// Fits inside a 1500 byte MTU alongside IPv6 and UDP headers.
pub const MAX_DATAGRAM_PAYLOAD: usize = 1400;

pub mod msg {
    pub const HELLO: u8 = 0x01;
    pub const WELCOME: u8 = 0x02;
    pub const SUBSCRIBE: u8 = 0x03;
    pub const SUBSCRIBED: u8 = 0x04;
    pub const UNSUBSCRIBE: u8 = 0x05;
    pub const UNSUBSCRIBED: u8 = 0x06;
    pub const PING: u8 = 0x07;
    pub const PONG: u8 = 0x08;
    pub const GET_STATS: u8 = 0x09;
    pub const STATS: u8 = 0x0a;
    pub const ERROR: u8 = 0x0b;
    /// Move the server's tuner, carrying [`super::tag::CENTER_HZ`]. Refused
    /// with [`super::error_code::NOT_TUNABLE`] unless the welcome said
    /// [`super::tag::TUNABLE`]. Since 1.1.
    pub const TUNE: u8 = 0x0c;
    /// Where the tuner actually landed, which is not always what was asked
    /// for: a dongle steps in units of its own. Sent to every subscriber, not
    /// only the one that asked, because a shared stream that moved under a
    /// reader who did not ask would otherwise be labelled at the old
    /// frequency. Since 1.1.
    pub const TUNED: u8 = 0x0d;
}

pub mod tag {
    // Identity and capability
    pub const CLIENT_NAME: u16 = 0x0001;
    pub const SERVER_NAME: u16 = 0x0002;
    // Stream description
    pub const CENTER_HZ: u16 = 0x0010;
    pub const SAMPLE_RATE: u16 = 0x0011;
    /// Tenths of a dB, signed.
    pub const GAIN_DDB: u16 = 0x0012;
    pub const TUNABLE: u16 = 0x0013;
    pub const SUPPORTED_BIT_DEPTHS: u16 = 0x0014;
    pub const SUPPORTED_CODECS: u16 = 0x0015;
    /// How far a tunable server will go, either way. Sent only with
    /// [`TUNABLE`] set, because a dial has to know where it may travel before
    /// it can offer to move: a server saying it is tunable and nothing else
    /// can be asked, but not driven. Since 1.1.
    pub const TUNE_MIN_HZ: u16 = 0x0016;
    pub const TUNE_MAX_HZ: u16 = 0x0017;
    // Subscription parameters
    pub const UDP_PORT: u16 = 0x0020;
    pub const BIT_DEPTH: u16 = 0x0021;
    pub const CODEC: u16 = 0x0022;
    pub const CODEC_LEVEL: u16 = 0x0023;
    pub const DECIMATION: u16 = 0x0024;
    pub const MAX_PAYLOAD: u16 = 0x0025;
    pub const BLOCK_SAMPLES: u16 = 0x0026;
    // Counters
    pub const BLOCKS_SENT: u16 = 0x0030;
    pub const BYTES_SENT: u16 = 0x0031;
    pub const BLOCKS_DROPPED: u16 = 0x0032;
    // Failure
    pub const ERROR_CODE: u16 = 0x0040;
    pub const ERROR_MESSAGE: u16 = 0x0041;
    /// Opaque to the receiver, echoed verbatim in the pong, so the sender can
    /// measure round trip time without keeping state.
    pub const TIMESTAMP_NS: u16 = 0x0050;
}

/// Monotonic-ish nanoseconds for ping timestamps. Only differences measured by
/// the same host are meaningful.
pub fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

pub mod error_code {
    pub const BAD_REQUEST: u16 = 1;
    pub const UNSUPPORTED_VERSION: u16 = 2;
    pub const UNSUPPORTED_BIT_DEPTH: u16 = 3;
    pub const UNSUPPORTED_CODEC: u16 = 4;
    pub const UNSUPPORTED_DECIMATION: u16 = 5;
    /// The server serves a tuner it will not let a subscriber move.
    pub const NOT_TUNABLE: u16 = 6;
    /// Asked for a frequency the tuner cannot reach.
    pub const OUT_OF_RANGE: u16 = 7;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Codec {
    None,
    Zstd,
}

impl Codec {
    pub fn code(self) -> u8 {
        match self {
            Codec::None => 0,
            Codec::Zstd => 1,
        }
    }

    pub fn from_code(code: u8) -> Result<Self> {
        match code {
            0 => Ok(Codec::None),
            1 => Ok(Codec::Zstd),
            other => bail!("unknown codec {other}"),
        }
    }

    pub fn parse(name: &str) -> Result<Self> {
        match name {
            "none" => Ok(Codec::None),
            "zstd" => Ok(Codec::Zstd),
            other => bail!("unknown codec {other:?}"),
        }
    }
}

/// Samples are unsigned, packed MSB first, `bits` per I or Q value. 8 bits is
/// the dongle's native UC8 and is passed through untouched.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BitDepth(pub u8);

impl BitDepth {
    pub const SUPPORTED: [u8; 6] = [3, 4, 5, 6, 7, 8];

    pub fn new(bits: u8) -> Result<Self> {
        if !Self::SUPPORTED.contains(&bits) {
            bail!("unsupported bit depth {bits}");
        }
        Ok(BitDepth(bits))
    }

    pub fn packed_len(&self, samples: usize) -> usize {
        // Two values per complex sample.
        (samples * 2 * self.0 as usize + 7) / 8
    }
}

// ---------------------------------------------------------------------------
// TLV encoding
// ---------------------------------------------------------------------------

#[derive(Default, Debug, Clone)]
pub struct Tlvs {
    buf: Vec<u8>,
}

impl Tlvs {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn bytes(&self) -> &[u8] {
        &self.buf
    }

    pub fn put(&mut self, tag: u16, value: &[u8]) -> &mut Self {
        assert!(value.len() <= u16::MAX as usize);
        self.buf.extend_from_slice(&tag.to_le_bytes());
        self.buf.extend_from_slice(&(value.len() as u16).to_le_bytes());
        self.buf.extend_from_slice(value);
        self
    }

    pub fn u8(&mut self, tag: u16, v: u8) -> &mut Self {
        self.put(tag, &[v])
    }
    pub fn u16(&mut self, tag: u16, v: u16) -> &mut Self {
        self.put(tag, &v.to_le_bytes())
    }
    pub fn i16(&mut self, tag: u16, v: i16) -> &mut Self {
        self.put(tag, &v.to_le_bytes())
    }
    pub fn u32(&mut self, tag: u16, v: u32) -> &mut Self {
        self.put(tag, &v.to_le_bytes())
    }
    pub fn u64(&mut self, tag: u16, v: u64) -> &mut Self {
        self.put(tag, &v.to_le_bytes())
    }
    pub fn str(&mut self, tag: u16, v: &str) -> &mut Self {
        self.put(tag, v.as_bytes())
    }
}

#[derive(Debug, Clone, Default)]
pub struct TlvMap<'a> {
    entries: Vec<(u16, &'a [u8])>,
}

impl<'a> TlvMap<'a> {
    pub fn parse(mut buf: &'a [u8]) -> Result<Self> {
        let mut entries = Vec::new();
        while !buf.is_empty() {
            if buf.len() < 4 {
                bail!("truncated TLV header");
            }
            let tag = u16::from_le_bytes([buf[0], buf[1]]);
            let len = u16::from_le_bytes([buf[2], buf[3]]) as usize;
            if buf.len() < 4 + len {
                bail!("truncated TLV value for tag {tag:#06x}");
            }
            entries.push((tag, &buf[4..4 + len]));
            buf = &buf[4 + len..];
        }
        Ok(TlvMap { entries })
    }

    pub fn get(&self, tag: u16) -> Option<&'a [u8]> {
        self.entries.iter().find(|(t, _)| *t == tag).map(|(_, v)| *v)
    }

    pub fn u8(&self, tag: u16) -> Option<u8> {
        self.get(tag).and_then(|v| v.first().copied())
    }
    pub fn u16(&self, tag: u16) -> Option<u16> {
        self.get(tag)
            .and_then(|v| v.get(..2))
            .map(|v| u16::from_le_bytes([v[0], v[1]]))
    }
    pub fn i16(&self, tag: u16) -> Option<i16> {
        self.u16(tag).map(|v| v as i16)
    }
    pub fn u32(&self, tag: u16) -> Option<u32> {
        self.get(tag)
            .and_then(|v| v.get(..4))
            .map(|v| u32::from_le_bytes(v.try_into().unwrap()))
    }
    pub fn u64(&self, tag: u16) -> Option<u64> {
        self.get(tag)
            .and_then(|v| v.get(..8))
            .map(|v| u64::from_le_bytes(v.try_into().unwrap()))
    }
    pub fn str(&self, tag: u16) -> Option<String> {
        self.get(tag)
            .map(|v| String::from_utf8_lossy(v).into_owned())
    }
}

// ---------------------------------------------------------------------------
// Control frames
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Frame {
    pub version: u8,
    pub msg_type: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    pub fn new(msg_type: u8, tlvs: &Tlvs) -> Self {
        Frame {
            version: VERSION_MAJOR as u8,
            msg_type,
            payload: tlvs.bytes().to_vec(),
        }
    }

    pub fn empty(msg_type: u8) -> Self {
        Frame::new(msg_type, &Tlvs::new())
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(FRAME_HEADER_LEN + self.payload.len());
        out.push(self.version);
        out.push(self.msg_type);
        out.extend_from_slice(&(self.payload.len() as u16).to_le_bytes());
        out.extend_from_slice(&self.payload);
        out
    }

    pub fn tlvs(&self) -> Result<TlvMap<'_>> {
        TlvMap::parse(&self.payload)
    }
}

pub fn encode_preamble() -> [u8; PREAMBLE_LEN] {
    let mut out = [0u8; PREAMBLE_LEN];
    out[0..4].copy_from_slice(&CONTROL_MAGIC);
    out[4..6].copy_from_slice(&VERSION_MAJOR.to_le_bytes());
    out[6..8].copy_from_slice(&VERSION_MINOR.to_le_bytes());
    out
}

/// Returns the peer's (major, minor) version.
pub fn decode_preamble(buf: &[u8; PREAMBLE_LEN]) -> Result<(u16, u16)> {
    if buf[0..4] != CONTROL_MAGIC {
        bail!("not an iqstream control connection");
    }
    Ok((
        u16::from_le_bytes([buf[4], buf[5]]),
        u16::from_le_bytes([buf[6], buf[7]]),
    ))
}

// ---------------------------------------------------------------------------
// Data datagrams
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DataHeader {
    pub version: u8,
    /// Index of the first complex sample of this block since stream start.
    pub sample_index: u64,
    pub block_seq: u32,
    pub frag_index: u16,
    pub frag_count: u16,
    pub bit_depth: u8,
    pub codec: Codec,
    pub decimation: u16,
    /// Complex samples in the block once decoded, so a receiver can size its
    /// buffer and pad correctly when a block is lost.
    pub block_samples: u32,
}

impl DataHeader {
    pub fn encode(&self, out: &mut [u8; DATA_HEADER_LEN]) {
        out[0..4].copy_from_slice(&DATA_MAGIC);
        out[4..6].copy_from_slice(&(DATA_HEADER_LEN as u16).to_le_bytes());
        out[6] = self.version;
        out[7] = 0;
        out[8..16].copy_from_slice(&self.sample_index.to_le_bytes());
        out[16..20].copy_from_slice(&self.block_seq.to_le_bytes());
        out[20..22].copy_from_slice(&self.frag_index.to_le_bytes());
        out[22..24].copy_from_slice(&self.frag_count.to_le_bytes());
        out[24] = self.bit_depth;
        out[25] = self.codec.code();
        out[26..28].copy_from_slice(&self.decimation.to_le_bytes());
        out[28..32].copy_from_slice(&self.block_samples.to_le_bytes());
    }

    pub fn decode(buf: &[u8]) -> Result<(Self, &[u8])> {
        if buf.len() < DATA_HEADER_LEN {
            bail!("short datagram: {} bytes", buf.len());
        }
        if buf[0..4] != DATA_MAGIC {
            bail!("bad magic");
        }
        let header_len = u16::from_le_bytes([buf[4], buf[5]]) as usize;
        if header_len < DATA_HEADER_LEN || header_len > buf.len() {
            bail!("bad header length {header_len}");
        }
        let header = DataHeader {
            version: buf[6],
            sample_index: u64::from_le_bytes(buf[8..16].try_into().unwrap()),
            block_seq: u32::from_le_bytes(buf[16..20].try_into().unwrap()),
            frag_index: u16::from_le_bytes(buf[20..22].try_into().unwrap()),
            frag_count: u16::from_le_bytes(buf[22..24].try_into().unwrap()),
            bit_depth: buf[24],
            codec: Codec::from_code(buf[25])?,
            decimation: u16::from_le_bytes([buf[26], buf[27]]),
            block_samples: u32::from_le_bytes(buf[28..32].try_into().unwrap()),
        };
        Ok((header, &buf[header_len..]))
    }
}

// ---------------------------------------------------------------------------
// Quantisation
// ---------------------------------------------------------------------------

/// Pack interleaved UC8 down to `bits` per value, MSB first. Dropping low bits
/// is the only lever that meaningfully shrinks this stream; see
/// docs/compression.md for what each depth costs in decoded messages.
pub fn pack(uc8: &[u8], bits: u8, out: &mut Vec<u8>) {
    out.clear();
    if bits == 8 {
        out.extend_from_slice(uc8);
        return;
    }
    out.reserve((uc8.len() * bits as usize + 7) / 8);
    let shift = 8 - bits;
    let mut acc: u32 = 0;
    let mut held = 0u32;
    for &byte in uc8 {
        acc = (acc << bits) | (byte >> shift) as u32;
        held += bits as u32;
        while held >= 8 {
            held -= 8;
            out.push((acc >> held) as u8);
        }
    }
    if held > 0 {
        out.push((acc << (8 - held)) as u8);
    }
}

/// Reverse of [`pack`], reconstructing at the middle of each quantisation step
/// rather than its floor, which halves the added noise.
pub fn unpack(packed: &[u8], bits: u8, values: usize, out: &mut Vec<u8>) {
    out.clear();
    if bits == 8 {
        out.extend_from_slice(&packed[..values.min(packed.len())]);
        return;
    }
    out.reserve(values);
    let shift = 8 - bits;
    let half = if shift > 0 { 1u8 << (shift - 1) } else { 0 };
    let mask = (1u32 << bits) - 1;
    let mut acc: u32 = 0;
    let mut held = 0u32;
    let mut produced = 0usize;
    for &byte in packed {
        acc = (acc << 8) | byte as u32;
        held += 8;
        while held >= bits as u32 && produced < values {
            held -= bits as u32;
            let v = ((acc >> held) & mask) as u8;
            out.push((v << shift) | half);
            produced += 1;
        }
    }
    while produced < values {
        out.push(0x80);
        produced += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_roundtrip_preserves_high_bits() {
        let src: Vec<u8> = (0..=255u8).collect();
        for bits in BitDepth::SUPPORTED {
            let mut packed = Vec::new();
            pack(&src, bits, &mut packed);
            assert_eq!(packed.len(), (src.len() * bits as usize + 7) / 8);
            let mut back = Vec::new();
            unpack(&packed, bits, src.len(), &mut back);
            assert_eq!(back.len(), src.len());
            for (a, b) in src.iter().zip(&back) {
                assert_eq!(a >> (8 - bits), b >> (8 - bits), "bits={bits}");
            }
        }
    }

    #[test]
    fn tlv_roundtrip() {
        let mut t = Tlvs::new();
        t.u64(tag::CENTER_HZ, 1_090_000_000)
            .u32(tag::SAMPLE_RATE, 2_400_000)
            .i16(tag::GAIN_DDB, 496)
            .str(tag::CLIENT_NAME, "probe");
        let m = TlvMap::parse(t.bytes()).unwrap();
        assert_eq!(m.u64(tag::CENTER_HZ), Some(1_090_000_000));
        assert_eq!(m.u32(tag::SAMPLE_RATE), Some(2_400_000));
        assert_eq!(m.i16(tag::GAIN_DDB), Some(496));
        assert_eq!(m.str(tag::CLIENT_NAME).as_deref(), Some("probe"));
        assert_eq!(m.u8(0xffff), None);
    }

    #[test]
    fn frame_roundtrip() {
        let mut t = Tlvs::new();
        t.u16(tag::UDP_PORT, 5000).u8(tag::BIT_DEPTH, 6);
        let bytes = Frame::new(msg::SUBSCRIBE, &t).encode();
        assert_eq!(bytes[1], msg::SUBSCRIBE);
        let len = u16::from_le_bytes([bytes[2], bytes[3]]) as usize;
        let m = TlvMap::parse(&bytes[FRAME_HEADER_LEN..FRAME_HEADER_LEN + len]).unwrap();
        assert_eq!(m.u16(tag::UDP_PORT), Some(5000));
        assert_eq!(m.u8(tag::BIT_DEPTH), Some(6));
    }
}
