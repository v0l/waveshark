//! The fast information channel of a DAB ensemble, EN 300 401 clauses 5, 6
//! and 11: what turns the soft bits of the first few symbols of a frame into
//! the ensemble's own description of itself.
//!
//! Three things sit between a fast information block and the carriers. A rate
//! 1/4 convolutional code protects it, punctured back to rate 1/3 by a
//! pattern the standard writes out block by block; an energy dispersal
//! sequence keeps the multiplex's spectrum flat; and a CRC says whether the
//! block arrived. Every one of those is undone here, in that order backwards.
//!
//! What comes out is a stream of fast information groups, which are the
//! ensemble's tables: its identifier and name, the services it carries and
//! their names, where each service sits in the main service channel and what
//! it is protected by. A receiver that has read those knows what is on the
//! air without decoding a note of audio.
//!
//! Nothing here touches the main service channel, so no sound comes out of
//! it: this is the station list and the tuning table, not the radio.

use crate::bits::crc16;
use crate::whiten::Prbs9;
use dsp::conv;

/// Bits in one fast information block, its CRC included.
pub const FIB_BITS: usize = 256;
/// Bytes in a fast information block.
pub const FIB: usize = FIB_BITS / 8;
/// Fast information blocks that share one convolutional codeword.
pub const FIBS_PER_CODEWORD: usize = 3;
/// Bits a codeword carries after puncturing, which is what arrives off the
/// carriers.
pub const CODEWORD_BITS: usize = 2304;
/// Bits the codeword protects: three blocks.
pub const CODEWORD_DATA: usize = FIBS_PER_CODEWORD * FIB_BITS;

/// The DAB convolutional code, EN 300 401 clause 11.1.1: rate 1/4, constraint
/// length 7, generators 133, 171, 145 and 133 octal in that order.
pub const CODE: conv::Code = conv::Code { constraint: 7, polys: &[0o133, 0o171, 0o145, 0o133] };

/// Puncturing vectors PI 15, PI 16 and the tail's, EN 300 401 table 29. Each
/// is a mask over thirty-two mother bits.
const PI15: [u8; 32] = [
    1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0,
];
const PI16: [u8; 32] = [
    1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0,
];
const PI_TAIL: [u8; 32] = [
    1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
];

/// The puncturing of one fast information codeword, EN 300 401 clause 11.2:
/// the 3072 mother bits in twenty-four blocks of 128, the first twenty-one
/// punctured by PI 16 and the last three by PI 15, then the twenty-four bits
/// the register's tail produces punctured by the tail vector.
///
/// The mask is the whole codeword rather than a repeating period, because the
/// pattern does not repeat: 2016 bits survive the first stretch, 276 the
/// second and 12 the tail, which is the 2304 a frame's carriers hold.
pub fn puncture() -> Vec<u8> {
    let mut out = Vec::with_capacity(4 * (CODEWORD_DATA + 6));
    for block in 0..24 {
        let pi = if block < 21 { &PI16 } else { &PI15 };
        for i in 0..128 {
            out.push(pi[i % 32]);
        }
    }
    out.extend_from_slice(&PI_TAIL[..24]);
    out
}

/// How the fast information channel is faring.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Stats {
    /// Codewords decoded.
    pub codewords: u64,
    /// Blocks whose CRC agreed.
    pub good: u64,
    /// Blocks whose CRC did not.
    pub bad: u64,
}

impl Stats {
    /// The share of blocks that arrived intact, or `None` before any did.
    pub fn quality(&self) -> Option<f32> {
        let total = self.good + self.bad;
        (total > 0).then(|| self.good as f32 / total as f32)
    }
}

/// What a service component carries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Audio {
    /// MPEG-1 layer II, which is plain DAB.
    Mp2,
    /// HE-AAC v2 in a superframe, which is DAB+.
    AacPlus,
    /// A value the standard has not assigned.
    Other(u8),
}

impl Audio {
    /// EN 300 401 clause 6.3.1, the audio service component type.
    pub fn from_ascty(ascty: u8) -> Self {
        match ascty {
            0 => Audio::Mp2,
            63 => Audio::AacPlus,
            other => Audio::Other(other),
        }
    }

    pub fn label(self) -> String {
        match self {
            Audio::Mp2 => "DAB".into(),
            Audio::AacPlus => "DAB+".into(),
            Audio::Other(v) => format!("audio {v}"),
        }
    }
}

/// What a component of a service is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Component {
    /// A stream of audio in a subchannel.
    Stream { sub_channel: u8, audio: Audio },
    /// A stream of data in a subchannel: a slideshow, a traffic feed.
    Data { sub_channel: u8, kind: u8 },
    /// Data in packets, addressed by a service component identifier rather
    /// than by a subchannel of its own.
    Packet { id: u16 },
}

impl Component {
    pub fn sub_channel(&self) -> Option<u8> {
        match self {
            Component::Stream { sub_channel, .. } | Component::Data { sub_channel, .. } => {
                Some(*sub_channel)
            }
            Component::Packet { .. } => None,
        }
    }
}

/// The error protection a subchannel is carried under, EN 300 401 clause
/// 6.2.1. Unequal protection is the older profile, with its own table of
/// sizes; equal protection states a level and a size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protection {
    /// The short form: a table index, whose level runs 1 (strongest) to 5.
    Unequal { level: u8 },
    /// The long form, profile A.
    EqualA { level: u8 },
    /// The long form, profile B.
    EqualB { level: u8 },
}

impl Protection {
    pub fn label(self) -> String {
        match self {
            Protection::Unequal { level } => format!("UEP {level}"),
            Protection::EqualA { level } => format!("EEP {level}-A"),
            Protection::EqualB { level } => format!("EEP {level}-B"),
        }
    }
}

/// Where a subchannel sits in the main service channel and what it costs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SubChannel {
    pub id: u8,
    /// First capacity unit, of the 864 a frame has.
    pub start: u16,
    /// Capacity units it occupies.
    pub size: u16,
    pub bitrate_kbps: u16,
    pub protection: Protection,
}

/// The programme type a service announces, EN 300 401 table 12 with the names
/// TS 101 756 gives them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProgrammeType {
    None,
    News,
    CurrentAffairs,
    Information,
    Sport,
    Education,
    Drama,
    Culture,
    Science,
    Varied,
    Pop,
    Rock,
    EasyListening,
    LightClassical,
    SeriousClassical,
    OtherMusic,
    Weather,
    Finance,
    Children,
    SocialAffairs,
    Religion,
    PhoneIn,
    Travel,
    Leisure,
    Jazz,
    Country,
    National,
    Oldies,
    Folk,
    Documentary,
    AlarmTest,
    Alarm,
}

impl ProgrammeType {
    pub const ALL: [ProgrammeType; 32] = [
        ProgrammeType::None,
        ProgrammeType::News,
        ProgrammeType::CurrentAffairs,
        ProgrammeType::Information,
        ProgrammeType::Sport,
        ProgrammeType::Education,
        ProgrammeType::Drama,
        ProgrammeType::Culture,
        ProgrammeType::Science,
        ProgrammeType::Varied,
        ProgrammeType::Pop,
        ProgrammeType::Rock,
        ProgrammeType::EasyListening,
        ProgrammeType::LightClassical,
        ProgrammeType::SeriousClassical,
        ProgrammeType::OtherMusic,
        ProgrammeType::Weather,
        ProgrammeType::Finance,
        ProgrammeType::Children,
        ProgrammeType::SocialAffairs,
        ProgrammeType::Religion,
        ProgrammeType::PhoneIn,
        ProgrammeType::Travel,
        ProgrammeType::Leisure,
        ProgrammeType::Jazz,
        ProgrammeType::Country,
        ProgrammeType::National,
        ProgrammeType::Oldies,
        ProgrammeType::Folk,
        ProgrammeType::Documentary,
        ProgrammeType::AlarmTest,
        ProgrammeType::Alarm,
    ];

    /// The five bit code, which is closed: every value names a type.
    pub fn from_code(code: u8) -> ProgrammeType {
        ProgrammeType::ALL[(code & 31) as usize]
    }

    pub fn code(self) -> u8 {
        ProgrammeType::ALL.iter().position(|p| *p == self).unwrap_or(0) as u8
    }

    pub fn label(self) -> &'static str {
        match self {
            ProgrammeType::None => "none",
            ProgrammeType::News => "news",
            ProgrammeType::CurrentAffairs => "current affairs",
            ProgrammeType::Information => "information",
            ProgrammeType::Sport => "sport",
            ProgrammeType::Education => "education",
            ProgrammeType::Drama => "drama",
            ProgrammeType::Culture => "culture",
            ProgrammeType::Science => "science",
            ProgrammeType::Varied => "varied",
            ProgrammeType::Pop => "pop music",
            ProgrammeType::Rock => "rock music",
            ProgrammeType::EasyListening => "easy listening",
            ProgrammeType::LightClassical => "light classical",
            ProgrammeType::SeriousClassical => "serious classical",
            ProgrammeType::OtherMusic => "other music",
            ProgrammeType::Weather => "weather",
            ProgrammeType::Finance => "finance",
            ProgrammeType::Children => "children",
            ProgrammeType::SocialAffairs => "social affairs",
            ProgrammeType::Religion => "religion",
            ProgrammeType::PhoneIn => "phone in",
            ProgrammeType::Travel => "travel",
            ProgrammeType::Leisure => "leisure",
            ProgrammeType::Jazz => "jazz",
            ProgrammeType::Country => "country music",
            ProgrammeType::National => "national music",
            ProgrammeType::Oldies => "oldies",
            ProgrammeType::Folk => "folk music",
            ProgrammeType::Documentary => "documentary",
            ProgrammeType::AlarmTest => "alarm test",
            ProgrammeType::Alarm => "alarm",
        }
    }
}

/// One service of an ensemble: a station.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Service {
    pub id: u32,
    pub name: Option<String>,
    pub programme_type: Option<ProgrammeType>,
    /// The language code of EN 300 401 table 9, kept as the code: the table
    /// is long and nothing here reads it.
    pub language: Option<u8>,
    pub components: Vec<Component>,
}

impl Service {
    /// The subchannel the service's audio is in, which is the first audio
    /// component it declared.
    pub fn audio(&self) -> Option<(u8, Audio)> {
        self.components.iter().find_map(|c| match c {
            Component::Stream { sub_channel, audio } => Some((*sub_channel, *audio)),
            _ => None,
        })
    }
}

/// An ensemble, as its own tables describe it.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Ensemble {
    pub id: Option<u16>,
    pub name: Option<String>,
    pub services: Vec<Service>,
    pub sub_channels: Vec<SubChannel>,
}

impl Ensemble {
    pub fn service(&self, id: u32) -> Option<&Service> {
        self.services.iter().find(|s| s.id == id)
    }

    pub fn sub_channel(&self, id: u8) -> Option<&SubChannel> {
        self.sub_channels.iter().find(|s| s.id == id)
    }

    /// The services with a name and an audio component, which is what a
    /// station list holds.
    pub fn stations(&self) -> impl Iterator<Item = &Service> {
        self.services.iter().filter(|s| s.name.is_some() && s.audio().is_some())
    }

    fn service_mut(&mut self, id: u32) -> &mut Service {
        if let Some(at) = self.services.iter().position(|s| s.id == id) {
            return &mut self.services[at];
        }
        self.services.push(Service { id, ..Default::default() });
        self.services.last_mut().expect("just pushed")
    }
}

/// The fast information channel: soft bits in, the ensemble's tables out.
pub struct Fic {
    mask: Vec<u8>,
    prbs: Vec<u8>,
    pending: Vec<f32>,
    ensemble: Ensemble,
    pub stats: Stats,
}

impl Default for Fic {
    fn default() -> Self {
        Self::new()
    }
}

impl Fic {
    pub fn new() -> Self {
        Self {
            mask: puncture(),
            prbs: Prbs9::new().take(CODEWORD_DATA).collect(),
            pending: Vec::new(),
            ensemble: Ensemble::default(),
            stats: Stats::default(),
        }
    }

    pub fn ensemble(&self) -> &Ensemble {
        &self.ensemble
    }

    pub fn reset(&mut self) {
        self.pending.clear();
        self.ensemble = Ensemble::default();
        self.stats = Stats::default();
    }

    /// Read soft bits off the fast information symbols. Returns the number of
    /// blocks that passed their CRC.
    pub fn push(&mut self, soft: &[f32]) -> usize {
        self.pending.extend_from_slice(soft);
        let mut good = 0;
        while self.pending.len() >= CODEWORD_BITS {
            let block: Vec<f32> = self.pending.drain(..CODEWORD_BITS).collect();
            good += self.codeword(&block);
        }
        good
    }

    /// One codeword: the code off it, the dispersal out of it, and its three
    /// blocks read.
    fn codeword(&mut self, soft: &[f32]) -> usize {
        self.stats.codewords += 1;
        let bits = conv::Viterbi::decode_block(
            CODE,
            soft,
            &self.mask,
            CODEWORD_DATA + 6,
            conv::Ends::Zero,
        );
        let mut bytes = [0u8; FIBS_PER_CODEWORD * FIB];
        for (i, chunk) in bits[..CODEWORD_DATA].chunks(8).enumerate() {
            let mut byte = 0u8;
            for (j, &bit) in chunk.iter().enumerate() {
                byte |= (bit ^ self.prbs[i * 8 + j]) << (7 - j);
            }
            bytes[i] = byte;
        }
        let mut good = 0;
        for fib in bytes.chunks(FIB) {
            if !fib_ok(fib) {
                self.stats.bad += 1;
                continue;
            }
            self.stats.good += 1;
            good += 1;
            read_fib(fib, &mut self.ensemble);
        }
        good
    }
}

/// Whether a block's CRC agrees, EN 300 401 clause 5.2.2: the CCITT
/// polynomial over the thirty bytes of groups, started at all ones and
/// transmitted complemented.
pub fn fib_ok(fib: &[u8]) -> bool {
    if fib.len() != FIB {
        return false;
    }
    let want = crc16(&fib[..FIB - 2], 0x1021, 0xFFFF) ^ 0xFFFF;
    want == u16::from_be_bytes([fib[FIB - 2], fib[FIB - 1]])
}

/// The check a block is transmitted with.
pub fn fib_crc(groups: &[u8]) -> u16 {
    crc16(groups, 0x1021, 0xFFFF) ^ 0xFFFF
}

/// `count` bits of `data` starting at bit `at`, most significant first.
fn bits(data: &[u8], at: usize, count: usize) -> u32 {
    let mut out = 0u32;
    for i in 0..count {
        let b = at + i;
        let bit = match data.get(b / 8) {
            Some(byte) => (byte >> (7 - b % 8)) & 1,
            None => 0,
        };
        out = (out << 1) | bit as u32;
    }
    out
}

/// Read one block's fast information groups into the ensemble.
pub fn read_fib(fib: &[u8], ensemble: &mut Ensemble) {
    let mut at = 0usize;
    while at < FIB - 2 {
        let kind = fib[at] >> 5;
        let length = (fib[at] & 0x1F) as usize;
        if kind == 7 && length == 31 {
            return;
        }
        let end = at + 1 + length;
        if end > FIB - 2 {
            return;
        }
        let group = &fib[at..end];
        match kind {
            0 => read_fig0(group, ensemble),
            1 => read_fig1(group, ensemble),
            _ => {}
        }
        at = end;
    }
}

/// FIG type 0, the ensemble's structure.
fn read_fig0(g: &[u8], ensemble: &mut Ensemble) {
    if g.len() < 2 {
        return;
    }
    // A group about another ensemble says nothing about this one.
    let other = bits(g, 9, 1) == 1;
    let long_ids = bits(g, 10, 1) == 1;
    if other {
        return;
    }
    match bits(g, 11, 5) {
        0 => {
            if g.len() >= 4 {
                ensemble.id = Some(bits(g, 16, 16) as u16);
            }
        }
        1 => read_subchannels(g, ensemble),
        2 => read_services(g, ensemble, long_ids),
        17 => read_programme_types(g, ensemble),
        _ => {}
    }
}

/// FIG 0/1, subchannel organisation.
fn read_subchannels(g: &[u8], ensemble: &mut Ensemble) {
    let mut at = 16usize;
    let end = g.len() * 8;
    while at + 24 <= end {
        let id = bits(g, at, 6) as u8;
        let start = bits(g, at + 6, 10) as u16;
        let long = bits(g, at + 16, 1) == 1;
        let (size, protection, next) = match long {
            false => {
                let index = bits(g, at + 18, 6) as usize;
                let (size, level, _) = UEP[index.min(UEP.len() - 1)];
                (size, Protection::Unequal { level }, at + 24)
            }
            true => {
                if at + 32 > end {
                    return;
                }
                let option = bits(g, at + 17, 3);
                let level = bits(g, at + 20, 2) as u8 + 1;
                let size = bits(g, at + 22, 10) as u16;
                let protection = match option {
                    0 => Protection::EqualA { level },
                    _ => Protection::EqualB { level },
                };
                (size, protection, at + 32)
            }
        };
        let bitrate_kbps = bitrate_of(size, protection);
        let sub = SubChannel { id, start, size, bitrate_kbps, protection };
        match ensemble.sub_channels.iter_mut().find(|s| s.id == id) {
            Some(existing) => *existing = sub,
            None => ensemble.sub_channels.push(sub),
        }
        at = next;
    }
}

/// The bit rate a subchannel of `size` capacity units carries.
///
/// A capacity unit is 64 bits in a 24 ms frame, so eight of them are one
/// kilobit a second, and the protection says how many of those bits are the
/// code's. EN 300 401 tables 7 and 8 give the divisors.
fn bitrate_of(size: u16, protection: Protection) -> u16 {
    match protection {
        Protection::Unequal { level } => UEP
            .iter()
            .find(|(s, l, _)| *s == size && *l == level)
            .map(|(_, _, rate)| *rate)
            .unwrap_or(0),
        // Table 7: a profile A subchannel of rate R kbit/s takes R/8 times
        // 12, 8, 6 or 4 capacity units at levels 1 to 4.
        Protection::EqualA { level } => {
            let n = [12u16, 8, 6, 4][(level.clamp(1, 4) - 1) as usize];
            size / n * 8
        }
        // Table 8: the same for profile B, in units of 32 kbit/s.
        Protection::EqualB { level } => {
            let n = [27u16, 21, 18, 15][(level.clamp(1, 4) - 1) as usize];
            size / n * 32
        }
    }
}

/// EN 300 401 table 6, the unequal protection profiles: capacity units, the
/// protection level, and the bit rate each index stands for.
const UEP: [(u16, u8, u16); 64] = [
    (16, 5, 32),
    (21, 4, 32),
    (24, 3, 32),
    (29, 2, 32),
    (35, 1, 32),
    (24, 5, 48),
    (29, 4, 48),
    (35, 3, 48),
    (42, 2, 48),
    (52, 1, 48),
    (29, 5, 56),
    (35, 4, 56),
    (42, 3, 56),
    (52, 2, 56),
    (32, 5, 64),
    (42, 4, 64),
    (48, 3, 64),
    (58, 2, 64),
    (70, 1, 64),
    (40, 5, 80),
    (52, 4, 80),
    (58, 3, 80),
    (70, 2, 80),
    (84, 1, 80),
    (48, 5, 96),
    (58, 4, 96),
    (70, 3, 96),
    (84, 2, 96),
    (104, 1, 96),
    (58, 5, 112),
    (70, 4, 112),
    (84, 3, 112),
    (104, 2, 112),
    (64, 5, 128),
    (84, 4, 128),
    (96, 3, 128),
    (116, 2, 128),
    (140, 1, 128),
    (80, 5, 160),
    (104, 4, 160),
    (116, 3, 160),
    (140, 2, 160),
    (168, 1, 160),
    (96, 5, 192),
    (116, 4, 192),
    (140, 3, 192),
    (168, 2, 192),
    (208, 1, 192),
    (116, 5, 224),
    (140, 4, 224),
    (168, 3, 224),
    (208, 2, 224),
    (232, 1, 224),
    (128, 5, 256),
    (168, 4, 256),
    (192, 3, 256),
    (232, 2, 256),
    (280, 1, 256),
    (160, 5, 320),
    (208, 4, 320),
    (280, 2, 320),
    (192, 5, 384),
    (280, 3, 384),
    (416, 1, 384),
];

/// FIG 0/2, which services there are and what they are made of.
fn read_services(g: &[u8], ensemble: &mut Ensemble, long_ids: bool) {
    let mut at = 16usize;
    let end = g.len() * 8;
    while at + 24 <= end {
        let (id, after) = match long_ids {
            true => (bits(g, at, 32), at + 32),
            false => (bits(g, at, 16), at + 16),
        };
        let count = bits(g, after + 4, 4) as usize;
        let mut components = Vec::with_capacity(count);
        let mut c = after + 8;
        for _ in 0..count {
            if c + 16 > end {
                return;
            }
            let kind = bits(g, c, 2);
            let component = match kind {
                0 => Component::Stream {
                    sub_channel: bits(g, c + 8, 6) as u8,
                    audio: Audio::from_ascty(bits(g, c + 2, 6) as u8),
                },
                1 => Component::Data {
                    sub_channel: bits(g, c + 8, 6) as u8,
                    kind: bits(g, c + 2, 6) as u8,
                },
                _ => Component::Packet { id: bits(g, c + 2, 12) as u16 },
            };
            components.push(component);
            c += 16;
        }
        ensemble.service_mut(id).components = components;
        at = c;
    }
}

/// FIG 0/17, the programme type of each service.
fn read_programme_types(g: &[u8], ensemble: &mut Ensemble) {
    let mut at = 16usize;
    let end = g.len() * 8;
    while at + 32 <= end {
        let id = bits(g, at, 16);
        let language = bits(g, at + 18, 1) == 1;
        let complement = bits(g, at + 19, 1) == 1;
        let mut here = at;
        if language {
            if here + 40 > end {
                return;
            }
            ensemble.service_mut(id).language = Some(bits(g, here + 24, 8) as u8);
            here += 8;
        }
        let code = bits(g, here + 27, 5) as u8;
        ensemble.service_mut(id).programme_type = Some(ProgrammeType::from_code(code));
        at = here + if complement { 40 } else { 32 };
    }
}

/// FIG type 1, the labels.
fn read_fig1(g: &[u8], ensemble: &mut Ensemble) {
    if g.len() < 2 {
        return;
    }
    let charset = bits(g, 8, 4) as u8;
    match bits(g, 13, 3) {
        // The ensemble's own label, after its identifier.
        0 => {
            if let Some(name) = label(g, 32, charset) {
                ensemble.id = Some(bits(g, 16, 16) as u16);
                ensemble.name = Some(name);
            }
        }
        // A programme service label, by its sixteen bit identifier.
        1 => {
            if let Some(name) = label(g, 32, charset) {
                ensemble.service_mut(bits(g, 16, 16)).name = Some(name);
            }
        }
        // A data service label, by its thirty-two bit identifier.
        5 => {
            if let Some(name) = label(g, 48, charset) {
                ensemble.service_mut(bits(g, 16, 32)).name = Some(name);
            }
        }
        _ => {}
    }
}

/// The sixteen character label at bit `at`, trimmed.
fn label(g: &[u8], at: usize, charset: u8) -> Option<String> {
    if at / 8 + 16 > g.len() {
        return None;
    }
    // Character set 0 is the EBU Latin repertoire of TS 101 756 annex C, 4 is
    // ISO 8859-1 and 6 is UCS-2, neither of which is transmitted in practice
    // and neither of which is read here.
    if charset != 0 {
        return None;
    }
    let text: String =
        g[at / 8..at / 8 + 16].iter().map(|&c| EBU_LATIN[c as usize]).collect::<String>();
    let trimmed = text.trim().trim_end_matches('\0').trim().to_string();
    (!trimmed.is_empty()).then_some(trimmed)
}

/// The EBU Latin repertoire, TS 101 756 annex C, as Unicode. It is ASCII in
/// the middle and everything a European broadcaster spells its name with
/// either side.
pub const EBU_LATIN: [char; 256] = [
    '\0', 'Ę', 'Į', 'Ų', 'Ă', 'Ė', 'Ď', 'Ș', 'Ț', 'Ċ', '\n', '\u{0b}', 'Ġ', 'Ĺ', 'Ż', 'Ń', 'ą',
    'ę', 'į', 'ų', 'ă', 'ė', 'ď', 'ș', 'ț', 'ċ', 'Ň', 'Ě', 'ġ', 'ĺ', 'ż', '\u{82}', ' ', '!', '"',
    '#', 'ł', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3', '4', '5',
    '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
    'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '[',
    'Ů', ']', 'Ł', '_', 'Ą', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '«', 'ů', '»', 'Ľ', 'Ħ', 'á', 'à',
    'é', 'è', 'í', 'ì', 'ó', 'ò', 'ú', 'ù', 'Ñ', 'Ç', 'Ş', 'ß', '¡', 'Ÿ', 'â', 'ä', 'ê', 'ë', 'î',
    'ï', 'ô', 'ö', 'û', 'ü', 'ñ', 'ç', 'ş', 'ğ', 'ı', 'ÿ', 'Ķ', 'Ņ', '©', 'Ģ', 'Ğ', 'ě', 'ň', 'ő',
    'Ő', '€', '£', '$', 'Ā', 'Ē', 'Ī', 'Ū', 'ķ', 'ņ', 'Ļ', 'ģ', 'ļ', 'İ', 'ń', 'ű', 'Ű', '¿', 'ľ',
    '°', 'ā', 'ē', 'ī', 'ū', 'Á', 'À', 'É', 'È', 'Í', 'Ì', 'Ó', 'Ò', 'Ú', 'Ù', 'Ř', 'Č', 'Š', 'Ž',
    'Ð', 'Ŀ', 'Â', 'Ä', 'Ê', 'Ë', 'Î', 'Ï', 'Ô', 'Ö', 'Û', 'Ü', 'ř', 'č', 'š', 'ž', 'đ', 'ŀ', 'Ã',
    'Å', 'Æ', 'Œ', 'ŷ', 'Ý', 'Õ', 'Ø', 'Þ', 'Ŋ', 'Ŕ', 'Ć', 'Ś', 'Ź', 'Ť', 'ð', 'ã', 'å', 'æ', 'œ',
    'ŵ', 'ý', 'õ', 'ø', 'þ', 'ŋ', 'ŕ', 'ć', 'ś', 'ź', 'ť', 'ħ',
];

/// The transmit side: fast information groups into blocks, and blocks into
/// the bits a frame's carriers hold.
///
/// It is here so the receiver can be tested against an ensemble whose every
/// service is known.
#[derive(Default)]
pub struct FicTx {
    groups: Vec<Vec<u8>>,
}

impl FicTx {
    pub fn new() -> Self {
        Self::default()
    }

    /// FIG 0/0, the ensemble identifier.
    pub fn ensemble(&mut self, id: u16) -> &mut Self {
        let mut g = vec![0x00, 0x00];
        g.extend_from_slice(&id.to_be_bytes());
        // The change flag, the alarm flag, the CIF counter and the occurrence
        // change, none of which this says anything with.
        g.extend_from_slice(&[0, 0, 0]);
        self.group(0, g)
    }

    /// FIG 0/1, one subchannel in the long, equal protection form.
    pub fn sub_channel(&mut self, id: u8, start: u16, size: u16, level: u8) -> &mut Self {
        let mut g = vec![0x00, 0x01];
        g.push((id << 2) | (start >> 8) as u8);
        g.push(start as u8);
        // Long form, profile A, the level and the size.
        g.push(0x80 | ((level - 1) << 2) | (size >> 8) as u8);
        g.push(size as u8);
        self.group(0, g)
    }

    /// FIG 0/2, one service and the subchannel its audio is in.
    pub fn service(&mut self, id: u16, sub_channel: u8, audio: Audio) -> &mut Self {
        let ascty = match audio {
            Audio::Mp2 => 0u8,
            Audio::AacPlus => 63,
            Audio::Other(v) => v,
        };
        let mut g = vec![0x00, 0x02];
        g.extend_from_slice(&id.to_be_bytes());
        g.push(0x01);
        g.push(ascty);
        // The subchannel, primary, not conditional access.
        g.push(sub_channel << 2 | 0b10);
        self.group(0, g)
    }

    /// FIG 0/17, a service's programme type.
    pub fn programme_type(&mut self, id: u16, pty: ProgrammeType) -> &mut Self {
        let mut g = vec![0x00, 0x11];
        g.extend_from_slice(&id.to_be_bytes());
        g.push(0x00);
        g.push(pty.code());
        self.group(0, g)
    }

    /// FIG 1/0, the ensemble's label.
    pub fn ensemble_label(&mut self, id: u16, name: &str) -> &mut Self {
        let mut g = vec![0x20, 0x00];
        g.extend_from_slice(&id.to_be_bytes());
        g.extend_from_slice(&chars(name));
        g.extend_from_slice(&[0xFF, 0xFF]);
        self.group(1, g)
    }

    /// FIG 1/1, a service's label.
    pub fn service_label(&mut self, id: u16, name: &str) -> &mut Self {
        let mut g = vec![0x20, 0x01];
        g.extend_from_slice(&id.to_be_bytes());
        g.extend_from_slice(&chars(name));
        g.extend_from_slice(&[0xFF, 0xFF]);
        self.group(1, g)
    }

    fn group(&mut self, kind: u8, mut g: Vec<u8>) -> &mut Self {
        // The first byte is the type and the length of what follows it, which
        // is known only now the group is built.
        g[0] = (kind << 5) | (g.len() as u8 - 1);
        self.groups.push(g);
        self
    }

    /// The blocks the groups pack into, each with its CRC. Groups are not
    /// split across blocks, which is what the standard requires of a
    /// transmitter and what makes a block readable on its own.
    pub fn fibs(&self) -> Vec<[u8; FIB]> {
        let mut out: Vec<[u8; FIB]> = Vec::new();
        let mut fill: Vec<u8> = Vec::new();
        for g in &self.groups {
            if fill.len() + g.len() > FIB - 2 {
                out.push(seal(&fill));
                fill.clear();
            }
            fill.extend_from_slice(g);
        }
        if !fill.is_empty() {
            out.push(seal(&fill));
        }
        out
    }

    /// One frame's worth of blocks, repeating what there is until the frame
    /// is full, and the coded bits they become.
    pub fn frame_bits(&self, fibs_per_frame: usize) -> Vec<u8> {
        let fibs = self.fibs();
        let mut out = Vec::new();
        for i in 0..fibs_per_frame {
            let fib = fibs.get(i % fibs.len().max(1)).copied().unwrap_or(seal(&[]));
            out.extend_from_slice(&fib);
        }
        let mut bits = Vec::with_capacity(out.len() * 8);
        for byte in out {
            for i in (0..8).rev() {
                bits.push((byte >> i) & 1);
            }
        }
        bits
    }
}

/// A block from the groups in it: the end marker, the padding and the CRC.
fn seal(groups: &[u8]) -> [u8; FIB] {
    let mut fib = [0u8; FIB];
    fib[..groups.len()].copy_from_slice(groups);
    if groups.len() < FIB - 2 {
        fib[groups.len()] = 0xFF;
    }
    let crc = fib_crc(&fib[..FIB - 2]);
    fib[FIB - 2..].copy_from_slice(&crc.to_be_bytes());
    fib
}

/// A label as sixteen EBU Latin characters, padded with spaces.
fn chars(name: &str) -> [u8; 16] {
    let mut out = [b' '; 16];
    for (i, c) in name.chars().take(16).enumerate() {
        out[i] = EBU_LATIN.iter().position(|e| *e == c).unwrap_or(b' ' as usize) as u8;
    }
    out
}

/// The coded bits of a codeword: the convolutional code, the puncturing and
/// the energy dispersal, which is what a transmitter puts on the carriers.
pub fn encode(fibs: &[u8]) -> Vec<u8> {
    assert_eq!(fibs.len(), FIBS_PER_CODEWORD * FIB, "three blocks to a codeword");
    let prbs: Vec<u8> = Prbs9::new().take(CODEWORD_DATA).collect();
    let mut bits = Vec::with_capacity(CODEWORD_DATA + 6);
    for (i, byte) in fibs.iter().enumerate() {
        for j in 0..8 {
            bits.push(((byte >> (7 - j)) & 1) ^ prbs[i * 8 + j]);
        }
    }
    bits.extend_from_slice(&[0; 6]);
    let mut encoder = conv::Encoder::new(CODE);
    let mut coded = Vec::with_capacity(4 * bits.len());
    for bit in bits {
        encoder.push(bit, &mut coded);
    }
    let mask = puncture();
    coded.iter().zip(mask.iter()).filter(|&(_, &keep)| keep == 1).map(|(&bit, _)| bit).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// EN 300 401 clause 11.2: twenty-four blocks of 128 bits and a tail of
    /// twenty-four, of which 2304 bits are transmitted.
    #[test]
    fn the_puncturing_leaves_the_bits_a_frame_carries() {
        let mask = puncture();
        assert_eq!(mask.len(), 4 * (CODEWORD_DATA + 6));
        assert_eq!(mask.len(), 3096);
        assert_eq!(mask.iter().filter(|b| **b == 1).count(), CODEWORD_BITS);
        // The three stretches, counted apart: PI 16 keeps 24 bits of every
        // 32, PI 15 keeps 23, and the tail keeps 12 of its 24.
        assert_eq!(mask[..21 * 128].iter().filter(|b| **b == 1).count(), 2016);
        assert_eq!(mask[21 * 128..24 * 128].iter().filter(|b| **b == 1).count(), 276);
        assert_eq!(mask[24 * 128..].iter().filter(|b| **b == 1).count(), 12);
    }

    /// The energy dispersal is its own inverse and repeats every 511 bits,
    /// which is the period of a nine stage register.
    #[test]
    fn the_dispersal_sequence_is_the_standards() {
        let prbs: Vec<u8> = Prbs9::new().take(2 * 511).collect();
        assert_eq!(prbs[..511], prbs[511..]);
        // Half the period is ones, one short of it, as a maximal length
        // register always is.
        assert_eq!(prbs[..511].iter().filter(|b| **b == 1).count(), 256);
    }

    /// A block built here reads back with its CRC intact and its groups in
    /// place.
    #[test]
    fn a_block_round_trips_through_its_crc() {
        let mut tx = FicTx::new();
        tx.ensemble(0xC1AB).ensemble_label(0xC1AB, "WaveShark");
        let fibs = tx.fibs();
        assert_eq!(fibs.len(), 1);
        assert!(fib_ok(&fibs[0]));
        let mut ensemble = Ensemble::default();
        read_fib(&fibs[0], &mut ensemble);
        assert_eq!(ensemble.id, Some(0xC1AB));
        assert_eq!(ensemble.name.as_deref(), Some("WaveShark"));
        // One bit wrong and the block is refused.
        let mut bad = fibs[0];
        bad[4] ^= 0x01;
        assert!(!fib_ok(&bad));
    }

    /// A whole codeword through the code and back, with nothing in the way:
    /// three blocks in, three blocks out, and the tables they describe.
    #[test]
    fn a_codeword_survives_its_own_coding() {
        let mut tx = FicTx::new();
        tx.ensemble(0xC221)
            .ensemble_label(0xC221, "SUPER RADIO")
            .sub_channel(1, 0, 72, 3)
            .service(0xC7D1, 1, Audio::AacPlus)
            .service_label(0xC7D1, "Shark FM")
            .programme_type(0xC7D1, ProgrammeType::Pop);
        let fibs = tx.fibs();
        let mut packed = Vec::new();
        for i in 0..FIBS_PER_CODEWORD {
            packed.extend_from_slice(&fibs[i % fibs.len()]);
        }
        let coded = encode(&packed);
        assert_eq!(coded.len(), CODEWORD_BITS);
        let soft: Vec<f32> = coded.iter().map(|&b| if b == 0 { 1.0 } else { -1.0 }).collect();

        let mut fic = Fic::new();
        assert_eq!(fic.push(&soft), FIBS_PER_CODEWORD);
        assert_eq!(fic.stats.good, 3);
        assert_eq!(fic.stats.bad, 0);
        let e = fic.ensemble();
        assert_eq!(e.id, Some(0xC221));
        assert_eq!(e.name.as_deref(), Some("SUPER RADIO"));
        assert_eq!(e.services.len(), 1);
        let s = &e.services[0];
        assert_eq!(s.id, 0xC7D1);
        assert_eq!(s.name.as_deref(), Some("Shark FM"));
        assert_eq!(s.programme_type, Some(ProgrammeType::Pop));
        assert_eq!(s.audio(), Some((1, Audio::AacPlus)));
        let sub = e.sub_channel(1).expect("the subchannel it named");
        assert_eq!((sub.start, sub.size), (0, 72));
        assert_eq!(sub.protection, Protection::EqualA { level: 3 });
        // Table 7: 72 capacity units at level 3-A is 96 kbit/s.
        assert_eq!(sub.bitrate_kbps, 96);
    }

    /// Noise through the same decoder: the Viterbi always puts out bits, and
    /// the CRC is what stops them becoming an ensemble.
    #[test]
    fn noise_produces_no_blocks() {
        let mut fic = Fic::new();
        let mut state = 0x1234_5678u32;
        let mut soft = Vec::with_capacity(CODEWORD_BITS);
        for _ in 0..200 {
            soft.clear();
            for _ in 0..CODEWORD_BITS {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                soft.push((state >> 16) as i16 as f32 / 32768.0);
            }
            assert_eq!(fic.push(&soft), 0);
        }
        assert_eq!(fic.stats.good, 0);
        assert_eq!(fic.stats.bad, 600);
        assert_eq!(*fic.ensemble(), Ensemble::default());
    }
}
