//! Binary wire format.
//!
//! Control runs over TCP as versioned frames carrying TLV payloads. Data runs
//! over UDP as self-describing datagrams. Everything is little-endian.
//!
//! Unknown TLV tags must be ignored, which is what makes a field additive: a
//! new server can talk to an old client as long as the major version matches.
//!
//! Vendored from <https://github.com/v0l/iqstream> so both ends of the
//! protocol can be spoken from here. 1.1 adds [`msg::TUNE`], 1.2 adds streams;
//! everything else is 1.0 unchanged, and the versions interoperate.

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
///
/// 2: several tuners on one port. The welcome carries a [`tag::STREAM`] per
/// tuner, [`tag::STREAM_ID`] says which one a subscribe, a tune or a tuned is
/// about, and the data header carries it too. A 1.1 peer names no stream and
/// gets the first one, which is the only stream a 1.1 server has.
///
/// 3: a setting that is a plain number, as [`SettingKind::Number`] with
/// [`tag::SETTING_MIN`], [`tag::SETTING_MAX`], [`tag::SETTING_STEP`] and
/// [`tag::SETTING_UNIT`]. A 1.2 reader sees a kind it does not know and
/// leaves that one setting alone, keeping the rest of the tuner's list.
pub const VERSION_MINOR: u16 = 3;

pub const PREAMBLE_LEN: usize = 8;
pub const FRAME_HEADER_LEN: usize = 4;
pub const MAX_FRAME_PAYLOAD: usize = 8192;
pub const DATA_HEADER_LEN: usize = 36;

/// The 1.1 data header, which carried no stream id.
///
/// Still read, because the header says its own length and a 1.1 server sends
/// this: the fields up to here have not moved, and everything in such a
/// stream is stream zero.
pub const DATA_HEADER_LEN_1_1: usize = 32;

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
    /// Ask what tuners the server has now. The welcome already said, so this
    /// is for a client that has been connected a while. Since 1.2.
    pub const LIST_STREAMS: u8 = 0x0e;
    /// The tuners, as [`super::tag::STREAM`] entries. Sent in answer to
    /// [`LIST_STREAMS`], and unasked to every connection when a tuner is
    /// added or taken away. Since 1.2.
    pub const STREAMS: u8 = 0x0f;
    /// One tuner, as a [`super::tag::STREAM`] entry, because something about
    /// it moved: a gain, a switch, an antenna port, the rate it runs at.
    ///
    /// Sent to every connection, whether or not it subscribed to that tuner,
    /// for the reason [`TUNED`] is: a reader labelling what it hears with a
    /// gain that was turned down ten minutes ago is reporting a level that
    /// was never true. Since 1.2.
    pub const STREAM_CHANGED: u8 = 0x10;
    /// Ask a tuner to be set differently: a gain, a switch, an antenna port,
    /// named by [`super::tag::SETTING_NAME`] and carrying
    /// [`super::tag::SETTING_VALUE`].
    ///
    /// A request like [`TUNE`] and refused on the same terms, since a radio
    /// somebody here is listening to is not one a subscriber may reach into.
    /// What it was actually set to comes back as [`STREAM_CHANGED`], because
    /// a driver snaps a gain to its own step. Since 1.2.
    pub const SET_SETTING: u8 = 0x11;
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
    /// One tuner, as a nested TLV block: see [`super::StreamDesc`]. Repeated
    /// once per tuner in a welcome or a [`super::msg::STREAMS`]. Since 1.2.
    pub const STREAM: u16 = 0x0018;
    /// Which tuner a subscribe, an unsubscribe, a tune or a tuned is about.
    /// Absent means the first one. Since 1.2.
    pub const STREAM_ID: u16 = 0x0019;
    /// What a tuner is called, for an operator picking one. Since 1.2.
    pub const STREAM_NAME: u16 = 0x001a;
    // Settings of a tuner, numbered clear of the subscription parameters and
    // the counters below: a nested block has its own context, but a tag that
    // means two things in one protocol is a trap for whoever reads a capture.
    /// One setting of a tuner, as a nested TLV block: see [`super::Setting`].
    /// Repeated once per setting inside a [`STREAM`]. Since 1.2.
    pub const SETTING: u16 = 0x0060;
    pub const SETTING_NAME: u16 = 0x0061;
    /// What an operator reading it calls it, which is not what the driver
    /// calls it.
    pub const SETTING_LABEL: u16 = 0x0062;
    /// Which of [`super::SettingKind`] this is.
    pub const SETTING_KIND: u16 = 0x0063;
    /// `auto` or tenths of a dB for a gain, `on` or `off` for a switch, the
    /// option's own name for a choice, the figure itself for a number.
    pub const SETTING_VALUE: u16 = 0x0064;
    /// One option a choice offers, repeated.
    pub const SETTING_OPTION: u16 = 0x0065;
    /// How far a gain goes, in tenths of a dB.
    pub const SETTING_MIN_DDB: u16 = 0x0066;
    pub const SETTING_MAX_DDB: u16 = 0x0067;
    /// How far a number goes, written out in its own unit rather than in a
    /// fixed scale: a trim is hertz and a gain is decibels, and a tenth of a
    /// dB field cannot hold either. Since 1.3.
    pub const SETTING_MIN: u16 = 0x0068;
    pub const SETTING_MAX: u16 = 0x0069;
    /// The smallest change a number takes, and zero for a continuous one.
    /// Since 1.3.
    pub const SETTING_STEP: u16 = 0x006a;
    /// What a number is measured in, shown after it: "Hz", "dB". Since 1.3.
    pub const SETTING_UNIT: u16 = 0x006b;
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
    /// Named a stream this server does not have, or no longer has.
    pub const NO_SUCH_STREAM: u16 = 8;
    /// Named a setting this tuner does not offer.
    pub const NO_SUCH_SETTING: u16 = 9;
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

    /// Every value under one tag, in the order they were written, for a tag
    /// that repeats: one [`tag::STREAM`] per tuner.
    pub fn all(&self, tag: u16) -> Vec<&'a [u8]> {
        self.entries.iter().filter(|(t, _)| *t == tag).map(|(_, v)| *v).collect()
    }

    pub fn u8(&self, tag: u16) -> Option<u8> {
        self.get(tag).and_then(|v| v.first().copied())
    }
    pub fn u16(&self, tag: u16) -> Option<u16> {
        self.get(tag).and_then(|v| v.get(..2)).map(|v| u16::from_le_bytes([v[0], v[1]]))
    }
    pub fn i16(&self, tag: u16) -> Option<i16> {
        self.u16(tag).map(|v| v as i16)
    }
    pub fn u32(&self, tag: u16) -> Option<u32> {
        self.get(tag).and_then(|v| v.get(..4)).map(|v| u32::from_le_bytes(v.try_into().unwrap()))
    }
    pub fn u64(&self, tag: u16) -> Option<u64> {
        self.get(tag).and_then(|v| v.get(..8)).map(|v| u64::from_le_bytes(v.try_into().unwrap()))
    }
    pub fn str(&self, tag: u16) -> Option<String> {
        self.get(tag).map(|v| String::from_utf8_lossy(v).into_owned())
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
        Frame { version: VERSION_MAJOR as u8, msg_type, payload: tlvs.bytes().to_vec() }
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
    Ok((u16::from_le_bytes([buf[4], buf[5]]), u16::from_le_bytes([buf[6], buf[7]])))
}

// ---------------------------------------------------------------------------
// Data datagrams
// ---------------------------------------------------------------------------

/// What kind of thing a [`Setting`] is, which is what a reader draws it as.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettingKind {
    /// A gain in dB, or the driver picking it.
    Gain,
    /// A switch: a bias tee, an offset tuning, a direct sampling input.
    Switch,
    /// One of a named set: an antenna port, a receive channel.
    Choice,
    /// A plain figure in a unit of its own: a frequency trim in hertz, a
    /// correction in parts per million.
    Number,
    /// Something a later version of this protocol knows about and this build
    /// does not, kept so it can be shown rather than dropped.
    Unknown(u8),
}

impl SettingKind {
    pub fn code(self) -> u8 {
        match self {
            Self::Gain => 0,
            Self::Switch => 1,
            Self::Choice => 2,
            Self::Number => 3,
            Self::Unknown(c) => c,
        }
    }

    pub fn from_code(code: u8) -> Self {
        match code {
            0 => Self::Gain,
            1 => Self::Switch,
            2 => Self::Choice,
            3 => Self::Number,
            other => Self::Unknown(other),
        }
    }
}

/// What one setting of a tuner is set to.
#[derive(Debug, Clone, PartialEq)]
pub enum SettingValue {
    /// The driver or the hardware picking the gain for itself.
    Auto,
    Gain(f32),
    Switch(bool),
    /// The option's own name, as it appears in [`Setting::options`].
    Choice(String),
    /// A figure in the setting's own unit, between [`Setting::range`].
    Number(f64),
    /// A kind this build does not know, kept as it arrived so it can be shown
    /// and sent back unchanged rather than drawn as something it is not.
    Unknown(String),
}

impl SettingValue {
    pub fn text(&self) -> String {
        match self {
            Self::Auto => "auto".into(),
            Self::Gain(db) => format!("{:.1}", db),
            Self::Switch(on) => match on {
                true => "on".into(),
                false => "off".into(),
            },
            Self::Choice(v) => v.clone(),
            // Rust's shortest representation that reads back as the same
            // number, so a trim of one hertz survives the round trip.
            Self::Number(v) => format!("{v}"),
            Self::Unknown(v) => v.clone(),
        }
    }

    pub fn parse(kind: SettingKind, text: &str) -> Self {
        match kind {
            SettingKind::Switch => Self::Switch(matches!(text, "on" | "1" | "true")),
            SettingKind::Gain => match text {
                "auto" => Self::Auto,
                db => Self::Gain(db.parse().unwrap_or(0.0)),
            },
            SettingKind::Number => Self::Number(text.parse().unwrap_or(0.0)),
            SettingKind::Choice => Self::Choice(text.to_string()),
            // Not a choice: a reader drawing an unknown kind as one would
            // offer an empty list of options and set it to nothing.
            SettingKind::Unknown(_) => Self::Unknown(text.to_string()),
        }
    }
}

/// One thing about a tuner that is set rather than heard: a gain stage, a
/// switch, an antenna port.
///
/// Carried in the tuner's description and sent again whenever it moves, so a
/// reader knows what the samples it is being given were taken at. What the
/// far end does not offer it does not list, which is how a dongle and a
/// LimeSDR describe themselves through the same field.
#[derive(Debug, Clone, PartialEq)]
pub struct Setting {
    /// The driver's own name for it, which is what a request would name.
    pub name: String,
    /// What an operator reading it calls it.
    pub label: String,
    pub kind: SettingKind,
    pub value: SettingValue,
    /// What a choice offers, and empty for anything else.
    pub options: Vec<String>,
    /// How far a gain goes, in dB, where the far end said.
    pub range_db: Option<(f32, f32)>,
    /// How far a number goes, in its own unit.
    pub range: Option<(f64, f64)>,
    /// The smallest change a number takes, and zero for a continuous one.
    pub step: f64,
    /// What a number is measured in, and empty for anything else.
    pub unit: String,
}

impl Default for Setting {
    fn default() -> Self {
        Self {
            name: String::new(),
            label: String::new(),
            kind: SettingKind::Switch,
            value: SettingValue::Switch(false),
            options: Vec::new(),
            range_db: None,
            range: None,
            step: 0.0,
            unit: String::new(),
        }
    }
}

/// One figure off a tag that holds it as text, and None where the tag is
/// absent or is not a number.
fn number(m: &TlvMap<'_>, tag: u16) -> Option<f64> {
    m.str(tag).and_then(|s| s.parse().ok())
}

impl Setting {
    pub fn encode(&self) -> Vec<u8> {
        let mut t = Tlvs::new();
        t.str(tag::SETTING_NAME, &self.name)
            .str(tag::SETTING_LABEL, &self.label)
            .u8(tag::SETTING_KIND, self.kind.code())
            .str(tag::SETTING_VALUE, &self.value.text());
        for o in &self.options {
            t.str(tag::SETTING_OPTION, o);
        }
        if let Some((lo, hi)) = self.range_db {
            t.i16(tag::SETTING_MIN_DDB, (lo * 10.0) as i16)
                .i16(tag::SETTING_MAX_DDB, (hi * 10.0) as i16);
        }
        if let Some((lo, hi)) = self.range {
            t.str(tag::SETTING_MIN, &format!("{lo}")).str(tag::SETTING_MAX, &format!("{hi}"));
        }
        if self.step != 0.0 {
            t.str(tag::SETTING_STEP, &format!("{}", self.step));
        }
        if !self.unit.is_empty() {
            t.str(tag::SETTING_UNIT, &self.unit);
        }
        t.bytes().to_vec()
    }

    pub fn decode(buf: &[u8]) -> Result<Self> {
        let m = TlvMap::parse(buf)?;
        let kind = SettingKind::from_code(m.u8(tag::SETTING_KIND).unwrap_or(0));
        Ok(Setting {
            name: m.str(tag::SETTING_NAME).unwrap_or_default(),
            label: m.str(tag::SETTING_LABEL).unwrap_or_default(),
            kind,
            value: SettingValue::parse(kind, &m.str(tag::SETTING_VALUE).unwrap_or_default()),
            options: m
                .all(tag::SETTING_OPTION)
                .iter()
                .map(|v| String::from_utf8_lossy(v).into_owned())
                .collect(),
            range_db: m
                .i16(tag::SETTING_MIN_DDB)
                .zip(m.i16(tag::SETTING_MAX_DDB))
                .map(|(lo, hi)| (lo as f32 / 10.0, hi as f32 / 10.0)),
            range: number(&m, tag::SETTING_MIN).zip(number(&m, tag::SETTING_MAX)),
            step: number(&m, tag::SETTING_STEP).unwrap_or(0.0),
            unit: m.str(tag::SETTING_UNIT).unwrap_or_default(),
        })
    }
}

/// One tuner a server is offering, as the welcome describes it.
///
/// Carried as a nested TLV block so the set can repeat and so a field added
/// later is additive here exactly as it is at the top level.
#[derive(Debug, Clone, PartialEq)]
pub struct StreamDesc {
    pub id: u16,
    /// What an operator picking a tuner sees. Never empty on the wire from
    /// this implementation, but a server may leave it out.
    pub name: String,
    pub center_hz: u64,
    pub sample_rate: u32,
    pub gain_db: Option<f32>,
    pub tunable: bool,
    pub tune_range_hz: Option<(u64, u64)>,
    /// Everything else the far end is set to: its gain stages, its switches,
    /// its antenna port. Empty from a server that says nothing about them.
    pub settings: Vec<Setting>,
}

impl StreamDesc {
    pub fn encode(&self) -> Vec<u8> {
        let mut t = Tlvs::new();
        t.u16(tag::STREAM_ID, self.id)
            .str(tag::STREAM_NAME, &self.name)
            .u64(tag::CENTER_HZ, self.center_hz)
            .u32(tag::SAMPLE_RATE, self.sample_rate)
            .u8(tag::TUNABLE, self.tunable as u8);
        if let Some(g) = self.gain_db {
            t.i16(tag::GAIN_DDB, (g * 10.0) as i16);
        }
        if let Some((lo, hi)) = self.tune_range_hz {
            t.u64(tag::TUNE_MIN_HZ, lo).u64(tag::TUNE_MAX_HZ, hi);
        }
        for s in &self.settings {
            t.put(tag::SETTING, &s.encode());
        }
        t.bytes().to_vec()
    }

    pub fn decode(buf: &[u8]) -> Result<Self> {
        let m = TlvMap::parse(buf)?;
        Ok(StreamDesc {
            id: m.u16(tag::STREAM_ID).unwrap_or(0),
            name: m.str(tag::STREAM_NAME).unwrap_or_default(),
            center_hz: m.u64(tag::CENTER_HZ).unwrap_or(0),
            sample_rate: m.u32(tag::SAMPLE_RATE).unwrap_or(0),
            gain_db: m.i16(tag::GAIN_DDB).map(|g| g as f32 / 10.0),
            tunable: m.u8(tag::TUNABLE).unwrap_or(0) != 0,
            tune_range_hz: m.u64(tag::TUNE_MIN_HZ).zip(m.u64(tag::TUNE_MAX_HZ)),
            settings: m.all(tag::SETTING).iter().filter_map(|b| Setting::decode(b).ok()).collect(),
        })
    }
}

/// The tuners in a welcome or a [`msg::STREAMS`].
///
/// Empty from a 1.1 peer, which describes its one stream with the flat tags
/// instead; a caller reading a welcome makes a [`StreamDesc`] out of those.
pub fn read_streams(m: &TlvMap<'_>) -> Vec<StreamDesc> {
    m.all(tag::STREAM).iter().filter_map(|b| StreamDesc::decode(b).ok()).collect()
}

pub fn put_streams(t: &mut Tlvs, streams: &[StreamDesc]) {
    for s in streams {
        t.put(tag::STREAM, &s.encode());
    }
}

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
    /// Which tuner these samples are of. Zero from a 1.1 server, which has
    /// one. Since 1.2.
    pub stream_id: u16,
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
        out[32..34].copy_from_slice(&self.stream_id.to_le_bytes());
        out[34..36].copy_from_slice(&0u16.to_le_bytes());
    }

    pub fn decode(buf: &[u8]) -> Result<(Self, &[u8])> {
        if buf.len() < DATA_HEADER_LEN_1_1 {
            bail!("short datagram: {} bytes", buf.len());
        }
        if buf[0..4] != DATA_MAGIC {
            bail!("bad magic");
        }
        let header_len = u16::from_le_bytes([buf[4], buf[5]]) as usize;
        if header_len < DATA_HEADER_LEN_1_1 || header_len > buf.len() {
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
            stream_id: match header_len >= DATA_HEADER_LEN {
                true => u16::from_le_bytes([buf[32], buf[33]]),
                false => 0,
            },
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

    /// A number keeps its figure, its ends, its step and its unit across the
    /// wire, and a kind this build has never heard of comes back as it went
    /// rather than as an empty choice.
    #[test]
    fn a_number_and_an_unknown_kind_both_survive_a_round_trip() {
        let n = Setting {
            name: "trim1".into(),
            label: "Tuner 2 trim".into(),
            kind: SettingKind::Number,
            value: SettingValue::Number(-1234.5),
            range: Some((-960_000.0, 960_000.0)),
            step: 1.0,
            unit: "Hz".into(),
            ..Default::default()
        };
        let back = Setting::decode(&n.encode()).unwrap();
        assert_eq!(back, n);
        assert_eq!(back.value, SettingValue::Number(-1234.5));
        assert_eq!(back.range, Some((-960_000.0, 960_000.0)));
        assert_eq!(back.step, 1.0);
        assert_eq!(back.unit, "Hz");

        let future = Setting {
            name: "shape".into(),
            label: "Filter shape".into(),
            kind: SettingKind::Unknown(9),
            value: SettingValue::Unknown("raised".into()),
            ..Default::default()
        };
        let back = Setting::decode(&future.encode()).unwrap();
        assert_eq!(back, future);
        assert_eq!(back.kind.code(), 9);
        assert_eq!(SettingKind::Number.code(), 3);
        // A gain is untouched by any of it.
        let g = Setting {
            name: "tuner".into(),
            label: "RF gain".into(),
            kind: SettingKind::Gain,
            value: SettingValue::Gain(32.8),
            range_db: Some((0.0, 49.6)),
            ..Default::default()
        };
        let back = Setting::decode(&g.encode()).unwrap();
        assert_eq!(back, g);
        assert_eq!(back.range, None);
        assert_eq!(back.step, 0.0);
        assert!(back.unit.is_empty());
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

    /// A tuner survives the nested encoding whole, and several of them keep
    /// their order and stay apart.
    #[test]
    fn streams_roundtrip_in_a_welcome() {
        let streams = vec![
            StreamDesc {
                id: 0,
                name: "span".into(),
                center_hz: 1_090_000_000,
                sample_rate: 2_400_000,
                gain_db: Some(49.6),
                tunable: false,
                tune_range_hz: None,
                settings: Vec::new(),
            },
            StreamDesc {
                id: 3,
                name: "loft dongle".into(),
                center_hz: 433_920_000,
                sample_rate: 1_024_000,
                gain_db: None,
                tunable: true,
                tune_range_hz: Some((24_000_000, 1_766_000_000)),
                settings: vec![
                    Setting {
                        name: "tuner".into(),
                        label: "RF gain".into(),
                        kind: SettingKind::Gain,
                        value: SettingValue::Gain(32.8),
                        options: Vec::new(),
                        range_db: Some((0.0, 49.6)),
                        ..Default::default()
                    },
                    Setting {
                        name: "bias_t".into(),
                        label: "Bias tee".into(),
                        kind: SettingKind::Switch,
                        value: SettingValue::Switch(true),
                        ..Default::default()
                    },
                    Setting {
                        name: "antenna".into(),
                        label: "Antenna".into(),
                        kind: SettingKind::Choice,
                        value: SettingValue::Choice("LNAW".into()),
                        options: vec!["LNAH".into(), "LNAL".into(), "LNAW".into()],
                        ..Default::default()
                    },
                    Setting {
                        name: "trim1".into(),
                        label: "Tuner 2 trim".into(),
                        kind: SettingKind::Number,
                        value: SettingValue::Number(-1234.5),
                        range: Some((-960_000.0, 960_000.0)),
                        step: 1.0,
                        unit: "Hz".into(),
                        ..Default::default()
                    },
                ],
            },
        ];
        let mut t = Tlvs::new();
        t.str(tag::SERVER_NAME, "waveshark");
        put_streams(&mut t, &streams);
        let m = TlvMap::parse(t.bytes()).unwrap();
        assert_eq!(read_streams(&m), streams);
        assert_eq!(m.str(tag::SERVER_NAME).as_deref(), Some("waveshark"));
    }

    /// A 1.1 welcome carries no stream entries, and reading it says so rather
    /// than inventing one: the caller makes the single stream out of the flat
    /// tags it does carry.
    #[test]
    fn a_welcome_without_streams_lists_none() {
        let mut t = Tlvs::new();
        t.u64(tag::CENTER_HZ, 1_090_000_000).u32(tag::SAMPLE_RATE, 2_400_000);
        let m = TlvMap::parse(t.bytes()).unwrap();
        assert!(read_streams(&m).is_empty());
    }

    /// The header grew by four bytes at the end, so a 1.1 datagram still
    /// decodes: its own length says where its payload starts, and everything
    /// in a 1.1 stream is stream zero.
    #[test]
    fn a_data_header_says_which_stream_and_an_old_one_still_reads() {
        let header = DataHeader {
            version: 1,
            sample_index: 4096,
            block_seq: 7,
            frag_index: 1,
            frag_count: 2,
            bit_depth: 8,
            codec: Codec::None,
            decimation: 1,
            block_samples: 2048,
            stream_id: 5,
        };
        let mut buf = [0u8; DATA_HEADER_LEN + 4];
        let mut head = [0u8; DATA_HEADER_LEN];
        header.encode(&mut head);
        buf[..DATA_HEADER_LEN].copy_from_slice(&head);
        buf[DATA_HEADER_LEN..].copy_from_slice(&[1, 2, 3, 4]);
        let (back, body) = DataHeader::decode(&buf).unwrap();
        assert_eq!(back, header);
        assert_eq!(body, &[1, 2, 3, 4]);

        // The same bytes as a 1.1 server writes them: a 32 byte header, four
        // bytes of payload, and no stream id anywhere.
        let mut old = [0u8; DATA_HEADER_LEN_1_1 + 4];
        old[..DATA_HEADER_LEN_1_1].copy_from_slice(&head[..DATA_HEADER_LEN_1_1]);
        old[4..6].copy_from_slice(&(DATA_HEADER_LEN_1_1 as u16).to_le_bytes());
        old[DATA_HEADER_LEN_1_1..].copy_from_slice(&[1, 2, 3, 4]);
        let (back, body) = DataHeader::decode(&old).unwrap();
        assert_eq!(back.stream_id, 0, "a 1.1 stream is stream zero");
        assert_eq!(back.sample_index, 4096);
        assert_eq!(back.block_samples, 2048);
        assert_eq!(body, &[1, 2, 3, 4]);
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
