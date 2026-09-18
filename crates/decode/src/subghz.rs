//! Flipper Zero `.sub` files: parse a saved capture or key into the pulses
//! that play it.
//!
//! The format is Flipper's flipper_format key/value text, documented at
//! <https://developer.flipper.net/flipperzero/doxygen/subghz_file_format.html>.
//! Two families matter:
//!
//! * RAW files (`Protocol: RAW`): `RAW_Data` lines of alternating mark/gap
//!   microseconds, positive mark first. This is what most traded captures
//!   are, and it needs no protocol knowledge: the timings are the signal.
//! * Key files (`Protocol: Princeton`, `CAME`, ...): a key, a bit count and
//!   a protocol's timing table. These are rebuilt as pulses with the
//!   encoders in `protocols::keyfob::encode`, so a saved key goes back on
//!   the air without the Flipper.
//!
//! Presets are parsed for the modulation they imply and nothing else: the
//! radio here has its own gain and bandwidth, and the CC1101 register dump
//! a custom preset carries says nothing a `Config` field cannot.

use crate::protocols::keyfob::encode::{self, INTER_FRAME_GAP_US};

use crate::slicer::Timing;
use common::pulse::{Package, Pulse};
use std::time::Duration;

/// Why a `.sub` file could not be read.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SubError {
    /// The file does not start with a Flipper SubGhz header.
    NotASubFile,
    /// A required key is missing or misread; carries the key name.
    BadField(&'static str),
    /// A protocol this build cannot encode. The message names it, because
    /// "cannot play this" without the word KeeLoq in it helps nobody.
    UnsupportedProtocol(String),
}

impl std::fmt::Display for SubError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotASubFile => write!(f, "not a Flipper SubGhz file"),
            Self::BadField(k) => write!(f, "{k} is missing or unreadable"),
            Self::UnsupportedProtocol(p) => {
                write!(f, "this build cannot encode the {p} protocol")
            }
        }
    }
}

impl std::error::Error for SubError {}

/// The modulation a preset names, reduced to what the transmitter does with
/// it. A preset's bandwidth and PA table are the Flipper's radio's business.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Preset {
    /// On/off keying, the async OOK presets at either bandwidth.
    Ook,
    /// Two-level frequency shift, with the deviation in Hz the preset name
    /// carries: 238 (2 kHz nominal, the name is the register value), 12 000
    /// or 47 600.
    Fsk(u32),
    /// GFSK, which the Flipper records as a custom preset with the GFSK
    /// registers; treated as FSK for replay.
    Gfsk,
    /// A `Custom_preset_data` dump, whose modulation this file cannot say.
    /// Nothing here reads CC1101 registers, so the file's own `Modulation`
    /// key is trusted when present and OOK is assumed otherwise, which is
    /// what the overwhelming majority of custom presets in circulation are.
    Custom,
}

impl Preset {
    fn parse(s: &str) -> Self {
        match s.trim() {
            "FuriHalSubGhzPresetOok270Async" | "FuriHalSubGhzPresetOok650Async" => Self::Ook,
            "FuriHalSubGhzPreset2FSKDev238Async" => Self::Fsk(2_000),
            "FuriHalSubGhzPreset2FSKDev12KAsync" => Self::Fsk(12_000),
            "FuriHalSubGhzPreset2FSKDev476Async" => Self::Fsk(47_600),
            "FuriHalSubGhzPresetCustom" => Self::Custom,
            "FuriHalSubGhzPresetGFSKDev9_6Async" | "FuriHalSubGhzPresetMSK29_3Async" => Self::Gfsk,
            _ => Self::Custom,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Ook => "OOK",
            Self::Fsk(_) => "FSK",
            Self::Gfsk => "GFSK",
            Self::Custom => "custom",
        }
    }

    /// The preset name to write. A deviation between the three the Flipper
    /// has goes to the nearest of them: the receiver that reads the file has
    /// only those registers, so a written 30 kHz is a file nothing can play.
    pub fn name(self) -> &'static str {
        match self {
            Self::Ook | Self::Custom => "FuriHalSubGhzPresetOok650Async",
            Self::Gfsk => "FuriHalSubGhzPresetGFSKDev9_6Async",
            Self::Fsk(dev) if dev <= 7_000 => "FuriHalSubGhzPreset2FSKDev238Async",
            Self::Fsk(dev) if dev <= 29_800 => "FuriHalSubGhzPreset2FSKDev12KAsync",
            Self::Fsk(_) => "FuriHalSubGhzPreset2FSKDev476Async",
        }
    }

    /// The preset for a burst keyed the way the receiver heard it.
    ///
    /// A classified burst says which family it was keyed on and not how far
    /// the tones were apart, so a frequency-keyed one is written at the
    /// middle of the three deviations the Flipper has. The keying is the
    /// part a reader cannot recover from the timings; the deviation it can
    /// be told to change.
    pub fn of_modulation(m: common::Modulation) -> Self {
        use common::Modulation as M;
        match m {
            M::Fsk2 | M::Fsk4 | M::Afsk => Self::Fsk(12_000),
            M::Gfsk | M::Gmsk | M::Msk => Self::Gfsk,
            _ => Self::Ook,
        }
    }
}

/// A `.sub` file to write out.
///
/// The write side of [`parse`], and the reason it is here rather than beside
/// the packet list: a file this produces has to be one this module reads
/// back, and the two conventions (which protocols the key is complemented
/// for, which timing table each uses) are stated once.
#[derive(Clone, Debug, PartialEq)]
pub struct Save {
    pub frequency: u64,
    pub preset: Preset,
    pub body: Body,
}

/// What the file says, which is either the timings or the key behind them.
#[derive(Clone, Debug, PartialEq)]
pub enum Body {
    /// The burst as heard. Anything can be written this way, and nothing is
    /// claimed about it beyond the widths.
    Raw(Package),
    /// A protocol's key, which a Flipper can replay, edit and put in a
    /// remote. Only written where a decoder recovered the code.
    Key {
        /// The Flipper's name for the protocol, not the decoder's.
        protocol: &'static str,
        bit: u32,
        key: u64,
        /// Written only where the protocol's timing is a multiple of it.
        te: Option<u32>,
    },
}

/// What a decoded remote is written as, or `None` for a protocol with no
/// key encoder, a rolling code, or a decode carrying no code.
///
/// `protocol` is the decoder's name and `fields` what it reported. The key
/// written is what [`key_package`] has to read to rebuild the code the
/// decoder found, which is the code itself where the decoder and the file
/// agree on polarity and its complement where they do not. Which is which
/// is not derivable from the parse side's own list, because two decoders
/// report the complement of the bits they sliced; the round-trip test below
/// keys every protocol here through its own decoder.
pub fn key_of_decode(protocol: &str, fields: &[(String, common::Value)]) -> Option<Body> {
    // Name, bits, TE where the timing is a multiple of one, and whether the
    // key file's value is the code as reported.
    let (name, bit, te, key_is_code) = match protocol {
        "Princeton" => ("Princeton", 24, Some(400), false),
        "CAME-12bit" => ("CAME", 12, None, false),
        "CAME-24bit" => ("CAME", 24, None, false),
        "Nice-Flo" => ("Nice FLO", 12, None, false),
        "Holtek" => ("Holtek", 40, None, false),
        "Holtek-HT12x" => ("Holtek_HT12X", 12, Some(320), false),
        "Bett" => ("BETT", 18, None, false),
        "Ansonic" => ("Ansonic", 12, None, false),
        // Linear is the one that reports the complement of what it sliced,
        // so its key file value is its code as printed.
        "Linear" => ("Linear", 10, None, true),
        "Linear-Delta3" => ("LinearDelta3", 8, None, false),
        _ => return None,
    };
    // `code` for most, `cnt` for the two that call the frame a counter.
    let code = ["code", "cnt"]
        .iter()
        .find_map(|k| fields.iter().find(|(n, _)| n == k).and_then(|(_, v)| v.as_i64()))?
        as u64;
    let n = bit as usize;
    let key = if key_is_code { code & mask(n) } else { !code & mask(n) };
    Some(Body::Key { protocol: name, bit, key, te })
}

impl Save {
    /// The file, as text ready to write to disk.
    pub fn text(&self) -> String {
        let mut s = String::new();
        let kind = match self.body {
            Body::Raw(_) => "RAW",
            Body::Key { .. } => "Key",
        };
        s.push_str(&format!("Filetype: Flipper SubGhz {kind} File\n"));
        s.push_str("Version: 1\n");
        s.push_str(&format!("Frequency: {}\n", self.frequency));
        s.push_str(&format!("Preset: {}\n", self.preset.name()));
        match &self.body {
            Body::Raw(pkg) => {
                s.push_str("Protocol: RAW\n");
                // Mark positive, gap negative, and the widths split across
                // lines the way the Flipper writes them: its own reader
                // takes a bounded line, and a single line of a long capture
                // is tens of kilobytes.
                let mut values = Vec::with_capacity(pkg.pulses.len() * 2);
                for p in &pkg.pulses {
                    if p.mark > 0 {
                        values.push(p.mark as i64);
                    }
                    if p.gap > 0 {
                        values.push(-(p.gap as i64));
                    }
                }
                for chunk in values.chunks(RAW_PER_LINE) {
                    let line: Vec<String> = chunk.iter().map(|v| v.to_string()).collect();
                    s.push_str(&format!("RAW_Data: {}\n", line.join(" ")));
                }
            }
            Body::Key { protocol, bit, key, te } => {
                s.push_str(&format!("Protocol: {protocol}\n"));
                s.push_str(&format!("Bit: {bit}\n"));
                let b = key.to_be_bytes();
                let hex: Vec<String> = b.iter().map(|v| format!("{v:02X}")).collect();
                s.push_str(&format!("Key: {}\n", hex.join(" ")));
                if let Some(te) = te {
                    s.push_str(&format!("TE: {te}\n"));
                }
            }
        }
        s
    }

    /// A name for the file, with no directory and no extension: what it is,
    /// where it was, and when, because a second press of the same button is
    /// a different file the operator still wants to keep.
    pub fn file_stem(&self, protocol: &str, at: std::time::SystemTime) -> String {
        let secs =
            at.duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or_default();
        let safe: String =
            protocol.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '_' }).collect();
        format!("{safe}_{:.2}MHz_{secs}", self.frequency as f64 / 1e6)
    }
}

/// How many timings go on one `RAW_Data` line. The Flipper writes 512 values
/// a line; anything of that order reads back the same.
const RAW_PER_LINE: usize = 512;

/// Protocols whose decoder inverts what it slices, so the key file's value
/// is what the decoder reports rather than its complement. The parse side
/// states the same list, and a test writes then reads back every protocol
/// here to keep the two from drifting apart.
fn complemented(protocol: &str) -> bool {
    matches!(
        protocol,
        "Princeton" | "CAME" | "Nice FLO" | "Holtek" | "Holtek_HT12X" | "BETT" | "Linear"
    )
}

/// A parsed `.sub` file: everything the transmitter needs.
#[derive(Clone, Debug, PartialEq)]
pub struct SubGhz {
    /// Carrier frequency, in Hz. A file without one is not playable: there
    /// is nothing sensible to guess from.
    pub frequency: u64,
    pub preset: Preset,
    /// The protocol the file names, as written. `RAW` for raw captures.
    pub protocol: String,
    /// The bursts to key, in microseconds. Raw files carry theirs as
    /// recorded; key files carry the encoding of their key. The final gap of
    /// the last burst is the tail silence, not a symbol.
    pub bursts: Vec<Package>,
}

impl SubGhz {
    /// The file's name is what a person knows it by; the timings are what
    /// the transmitter keys.
    pub fn label(&self) -> String {
        self.protocol.clone()
    }

    /// Total keyed duration, from the widths themselves.
    pub fn duration(&self) -> Duration {
        let us: u64 = self
            .bursts
            .iter()
            .flat_map(|b| b.pulses.iter())
            .map(|p| p.mark as u64 + p.gap as u64)
            .sum();
        Duration::from_micros(us)
    }
}

/// Parse a `.sub` file.
pub fn parse(text: &str) -> Result<SubGhz, SubError> {
    let mut fields: Vec<(String, String)> = Vec::new();
    let mut raw_data: Vec<i64> = Vec::new();
    let mut saw_header = false;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let Some((k, v)) = line.split_once(':') else {
            continue;
        };
        let (k, v) = (k.trim(), v.trim());
        if k == "Filetype" {
            if v.starts_with("Flipper SubGhz") {
                saw_header = true;
            } else {
                return Err(SubError::NotASubFile);
            }
            continue;
        }
        if k == "RAW_Data" {
            for tok in v.split_whitespace() {
                let n: i64 = tok.parse().map_err(|_| SubError::BadField("RAW_Data"))?;
                raw_data.push(n);
            }
        }
        fields.push((k.to_string(), v.to_string()));
    }
    if !saw_header {
        return Err(SubError::NotASubFile);
    }
    let get = |name: &str| fields.iter().find(|(k, _)| k == name).map(|(_, v)| v.clone());
    let frequency: u64 =
        get("Frequency").and_then(|v| v.parse().ok()).ok_or(SubError::BadField("Frequency"))?;
    let preset = Preset::parse(&get("Preset").unwrap_or_default());
    let protocol = get("Protocol").ok_or(SubError::BadField("Protocol"))?;

    let bursts = match protocol.as_str() {
        "RAW" => vec![raw_package(&raw_data)?],
        other => vec![key_package(other, &get)?],
    };
    Ok(SubGhz { frequency, preset, protocol, bursts })
}

/// RAW_Data values into one package. Positive is carrier on, negative off.
/// The Flipper's writer zeroes the sign pattern on the first value in
/// practice but the format says positive first; both are accepted, and a
/// leading gap is dropped rather than played as a pause before nothing.
fn raw_package(data: &[i64]) -> Result<Package, SubError> {
    if data.is_empty() {
        return Err(SubError::BadField("RAW_Data"));
    }
    let mut pulses = Vec::with_capacity(data.len() / 2 + 1);
    // Signs alternate; the absolute value is the width. A pair of same-sign
    // values is a malformed file, and is read as the widths it claims rather
    // than dropped, because a partial burst is still evidence.
    let mut i = 0;
    if data[0] < 0 {
        pulses.push(Pulse { mark: 0, gap: (-data[0]).min(u32::MAX as i64) as u32 });
        i = 1;
    }
    while i < data.len() {
        let mark = data[i].abs().min(u32::MAX as i64) as u32;
        let gap = data.get(i + 1).map_or(0, |v| v.abs().min(u32::MAX as i64) as u32);
        if mark > 0 || gap > 0 {
            pulses.push(Pulse { mark, gap });
        }
        i += 2;
    }
    Ok(Package { pulses, ..Default::default() })
}

/// A key file into pulses, via the encoder for the protocol it names.
///
/// `get` reads fields past the key/value list, which key files carry in
/// whatever order their writer chose.
fn key_package(protocol: &str, get: &dyn Fn(&str) -> Option<String>) -> Result<Package, SubError> {
    let bit: u32 = get("Bit").and_then(|v| v.parse().ok()).ok_or(SubError::BadField("Bit"))?;
    let key = get("Key")
        .map(|v| v.split_whitespace().collect::<String>())
        .and_then(|v| u64::from_str_radix(&v, 16).ok())
        .ok_or(SubError::BadField("Key"))?;
    let te: u32 = get("TE").and_then(|v| v.parse().ok()).unwrap_or(0);
    // Every encoder repeats the frame; a key file may say how many.
    let repeats: usize = get("Repeat").and_then(|v| v.parse().ok()).unwrap_or(10);

    // The frame as MSB-first bits: what `find_and_parse` inverts and reads,
    // so the encoder is fed exactly the buffer the decoder expects to see
    // inverted. `Key` holds the value printed in the file, which the
    // Flipper writes as the data the decoder decoded, complemented or not
    // per protocol below.
    let bits = |v: u64, n: u32| {
        let mut b = crate::bits::BitBuffer::with_capacity(n as usize);
        for i in (0..n).rev() {
            b.push(v >> i & 1 == 1);
        }
        b
    };

    // Which protocols read the wire complemented: for these the decoder
    // inverts what it slices, so the on-air bits are the key file's value
    // as written, and the code the decoder reports is its complement. The
    // two read conventions meet at the same place the decoder tests pin
    // them. Protocols listed here are fed the key unchanged; the rest are
    // fed the key's complement, because their decoder does not invert.
    let complement = complemented(protocol);

    let timing = match protocol {
        "Princeton" => {
            let te = if te == 0 { 400 } else { te };
            Timing::pwm(te, te * 3, te * 30)
        }
        "CAME" => {
            return came_package(bit, key, complement, repeats);
        }
        "Nice FLO" => Timing::pwm(700, 1400, 3000),
        "Holtek" => Timing::pwm(430, 870, 4000),
        "Holtek_HT12X" => {
            let te = if te == 0 { 320 } else { te };
            Timing::pwm(te, te * 2, te * 10)
        }
        "Linear" => Timing::pwm(500, 1500, 2500),
        "LinearDelta3" => Timing::pwm(500, 2000, 4000),
        "Ansonic" => Timing::pwm(555, 1111, 2500),
        "BETT" => Timing::pwm(340, 2000, 15_000),
        "GateTX" => Timing::pwm(350, 700, 2500),
        "SMC5326" => {
            let te = if te == 0 { 320 } else { te };
            Timing::pwm(te, te * 3, te * 25)
        }
        other => return Err(SubError::UnsupportedProtocol(other.to_string())),
    };

    let n = bit.min(64) as usize;
    let v = if complement { key & mask(n) } else { !key & mask(n) };
    Ok(encode::frame(timing, &bits(v, n as u32), repeats))
}

fn came_package(bit: u32, key: u64, complement: bool, repeats: usize) -> Result<Package, SubError> {
    // CAME's header is a long silence before a short start mark, sent
    // ahead of the frame on every repeat: 47 te_short for 12-bit, 76 for
    // 24-bit, per the Flipper's own encoder. The frame's bits then run
    // long-gap-first for a 1, so the frame here is built as gap/mark pairs
    // after the start mark rather than as mark/gap pairs.
    let (header_te, frame_bits) = match bit {
        12 => (47u32, 12usize),
        24 => (76, 24),
        _ => return Err(SubError::UnsupportedProtocol(format!("CAME/{bit} bit"))),
    };
    let v = if complement { key & mask(frame_bits) } else { !key & mask(frame_bits) };
    // A pulse is a mark and the gap after it, so the frame is paired that
    // way rather than as the gap-then-mark the protocol is described in:
    // the air is the same either way, but a pulse with a zero mark is a
    // symbol of no width to every slicer that reads one back.
    // No leading silence: a burst starts at its first mark, and a package
    // opening on a pulse of no width is one no slicer will read back. The
    // header gap is still there between repeats, as the gap of the pulse
    // before each.
    let mut pulses = Vec::with_capacity(frame_bits + 1);
    let mut mark = 320;
    for i in (0..frame_bits).rev() {
        let one = v >> i & 1 == 1;
        let (gap, next) = if one { (640, 320) } else { (320, 640) };
        pulses.push(Pulse { mark, gap });
        mark = next;
    }
    pulses.push(Pulse { mark, gap: INTER_FRAME_GAP_US });
    let one = Package { pulses, ..Default::default() };
    Ok(encode::repeated(&one, repeats, Duration::from_micros(320 * header_te as u64)))
}

fn mask(bits: usize) -> u64 {
    if bits >= 64 { u64::MAX } else { (1u64 << bits) - 1 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::{DecodeError, Protocol};

    const PRINCETON_RAW: &str = "\
Filetype: Flipper SubGhz RAW File
Version: 1
Frequency: 433920000
Preset: FuriHalSubGhzPresetOok650Async
Protocol: RAW
RAW_Data: 350 -350 350 -700 700 -350 350 -350 700 -700 350 -350 350 -350 700 -350 350 -700 700 -350 350 -350 350 -700 700 -350 350 -350 700 -700 350 -10000
";

    const PRINCETON_KEY: &str = "\
Filetype: Flipper SubGhz Key File
Version: 1
Frequency: 433920000
Preset: FuriHalSubGhzPresetOok650Async
Protocol: Princeton
Bit: 24
Key: 00 00 00 00 00 95 D5 D4
TE: 400
";

    /// The same key as `PRINCETON_KEY`, as this module writes it.
    const PRINCETON_KEY_OUT: &str = "\
Filetype: Flipper SubGhz Key File
Version: 1
Frequency: 433920000
Preset: FuriHalSubGhzPresetOok650Async
Protocol: Princeton
Bit: 24
Key: 00 00 00 00 00 95 D5 D4
TE: 400
";

    #[test]
    fn a_nullsec_raw_file_parses_to_its_timings() {
        let s = parse(PRINCETON_RAW).expect("the nullsec doorbell shape");
        assert_eq!(s.frequency, 433_920_000);
        assert_eq!(s.preset, Preset::Ook);
        assert_eq!(s.protocol, "RAW");
        assert_eq!(s.bursts.len(), 1);
        let p = &s.bursts[0];
        assert_eq!(p.pulses.len(), 16, "one pulse per mark/gap pair");
        assert_eq!(p.pulses[0], Pulse { mark: 350, gap: 350 });
        assert_eq!(p.pulses[1], Pulse { mark: 350, gap: 700 });
    }

    #[test]
    fn a_key_file_becomes_pulses_its_own_decoder_reads() {
        let s = parse(PRINCETON_KEY).expect("the Flipper corpus princeton key");
        assert_eq!(s.protocol, "Princeton");
        let pkg = &s.bursts[0];
        // Decoded back through the receiver's own Princeton decoder, the
        // key comes out complemented (short mark 0 on the air): the same
        // value the Flipper's decoder reports for this file.
        let p = crate::script::named("Princeton").unwrap();
        let r = p.decode_package(pkg).expect("the encoder's own pulses decode");
        assert_eq!(r.get("code"), Some(&crate::protocol::Value::Int(0x6a_2a_2b)));
    }

    #[test]
    fn a_raw_file_decodes_back_through_the_receiver() {
        // Sixteen mark/gap pairs, which is two thirds of a Princeton frame:
        // the timings are that protocol's but the frame is short, so the
        // decoder refuses it rather than reading a code out of a fragment.
        let s = parse(PRINCETON_RAW).unwrap();
        assert_eq!(s.bursts[0].pulses.len(), 16);
        let p = crate::script::named("Princeton").unwrap();
        assert_eq!(p.decode_package(&s.bursts[0]), Err(DecodeError::NotThisProtocol));
    }

    /// Every protocol [`key_of_decode`] writes, keyed through its own
    /// encoder and read back by its own decoder: the code that goes in is
    /// the code that comes out. This is what pins the key polarity, which
    /// differs between decoders that invert what they slice and decoders
    /// that report the complement of what they sliced.
    #[test]
    fn every_key_protocol_written_is_read_back_as_the_same_code() {
        use crate::protocol::Value;
        let holtek = 0x50_d2_aa_aa_a1_u64;
        let cases: [(&str, &str, u64); 8] = [
            ("Princeton", "code", 0xa1_3f_08),
            ("Nice-Flo", "code", 0xabc),
            ("Holtek", "code", holtek),
            ("Holtek-HT12x", "code", 0xabc),
            ("Bett", "code", 0x3_ab_cd),
            ("Ansonic", "code", 0xabc),
            ("Linear", "code", 0x2aa),
            ("Linear-Delta3", "cnt", 0x5a),
        ];
        for (name, field, code) in cases {
            let decoder = crate::script::named(name).unwrap();
            let fields = vec![(field.to_string(), Value::Int(code as i64))];
            let body = key_of_decode(name, &fields).unwrap_or_else(|| panic!("{name} has a key"));
            let save = Save { frequency: 433_920_000, preset: Preset::Ook, body };
            let text = save.text();
            let back =
                parse(&text).unwrap_or_else(|e| panic!("{name} writes a readable file: {e}"));
            let r = decoder
                .decode_package(&back.bursts[0])
                .unwrap_or_else(|e| panic!("{name} decodes its own key file: {e:?}"));
            assert_eq!(
                r.get(field),
                Some(&Value::Int(code as i64)),
                "{name} round-trips its code through a key file"
            );
        }
    }

    /// CAME is written and read back as the same key, but not decoded back:
    /// its symbol is a gap then a mark, so a package of mark/gap pulses
    /// carries the start mark in the first symbol and every decoded frame
    /// comes out one symbol shifted. The file is right; the receiver's own
    /// CAME decoder cannot yet be used to check it.
    #[test]
    fn came_is_written_as_a_key_the_encoder_reads_back() {
        use crate::protocol::Value;
        for (name, bit, code) in [("CAME-12bit", 12u32, 0xabc_u64), ("CAME-24bit", 24, 0xab_cd_ef)]
        {
            let fields = vec![("code".to_string(), Value::Int(code as i64))];
            let body = key_of_decode(name, &fields).unwrap();
            assert_eq!(
                body,
                Body::Key { protocol: "CAME", bit, key: !code & mask(bit as usize), te: None }
            );
            let save = Save { frequency: 433_920_000, preset: Preset::Ook, body };
            let back = parse(&save.text()).expect("CAME writes a readable file");
            assert_eq!(back.protocol, "CAME");
            // Ten repeats of the frame: twelve or twenty-four symbols and
            // the start mark, each time.
            assert_eq!(back.bursts[0].pulses.len(), 10 * (bit as usize + 1));
        }
    }

    #[test]
    fn a_key_file_is_written_the_way_the_flipper_writes_one() {
        use crate::protocol::Value;
        let fields = vec![("code".to_string(), Value::Int(0x6a_2a_2b))];
        let body = key_of_decode("Princeton", &fields).unwrap();
        let save = Save { frequency: 433_920_000, preset: Preset::Ook, body };
        assert_eq!(save.text(), PRINCETON_KEY_OUT);
        // And the file the corpus carries for the same code parses to the
        // same key, so what is written is what a Flipper would have.
        assert_eq!(parse(&save.text()).unwrap().bursts, parse(PRINCETON_KEY).unwrap().bursts);
    }

    #[test]
    fn a_raw_save_keeps_every_width() {
        let pkg = Package {
            pulses: vec![
                Pulse { mark: 350, gap: 350 },
                Pulse { mark: 350, gap: 700 },
                Pulse { mark: 700, gap: 10_000 },
            ],
            ..Default::default()
        };
        let save = Save {
            frequency: 315_000_000,
            preset: Preset::Fsk(12_000),
            body: Body::Raw(pkg.clone()),
        };
        let text = save.text();
        assert!(text.starts_with("Filetype: Flipper SubGhz RAW File\n"));
        assert!(text.contains("Preset: FuriHalSubGhzPreset2FSKDev12KAsync\n"));
        let back = parse(&text).unwrap();
        assert_eq!(back.frequency, 315_000_000);
        assert_eq!(back.bursts[0].pulses, pkg.pulses);
    }

    #[test]
    fn a_long_capture_is_written_over_several_lines() {
        let pulses: Vec<Pulse> = (0..600).map(|_| Pulse { mark: 350, gap: 350 }).collect();
        let save = Save {
            frequency: 433_920_000,
            preset: Preset::Ook,
            body: Body::Raw(Package { pulses: pulses.clone(), ..Default::default() }),
        };
        let text = save.text();
        assert_eq!(text.lines().filter(|l| l.starts_with("RAW_Data:")).count(), 3);
        assert_eq!(parse(&text).unwrap().bursts[0].pulses, pulses);
    }

    #[test]
    fn a_protocol_with_no_encoder_is_not_offered_a_key_file() {
        use crate::protocol::Value;
        let fields = vec![("code".to_string(), Value::Int(1))];
        assert_eq!(key_of_decode("KeeLoq", &fields), None);
        // And a decode carrying no code at all cannot be written as a key.
        assert_eq!(key_of_decode("Princeton", &[]), None);
    }

    #[test]
    fn not_a_sub_file_is_refused() {
        assert_eq!(parse("Filetype: WAV\nVersion: 1\n"), Err(SubError::NotASubFile));
        assert_eq!(parse("nothing here"), Err(SubError::NotASubFile));
    }

    #[test]
    fn a_missing_frequency_is_named() {
        let text =
            "Filetype: Flipper SubGhz RAW File\nVersion: 1\nProtocol: RAW\nRAW_Data: 100 -100\n";
        assert_eq!(parse(text), Err(SubError::BadField("Frequency")));
    }

    #[test]
    fn an_unsupported_protocol_names_itself() {
        let text = "\
Filetype: Flipper SubGhz Key File
Version: 1
Frequency: 433920000
Preset: FuriHalSubGhzPresetOok650Async
Protocol: KeeLoq
Bit: 64
Key: 48 50 F0 72 33 78 95 14
";
        match parse(text) {
            Err(SubError::UnsupportedProtocol(p)) => {
                assert_eq!(p, "KeeLoq");
            }
            other => panic!("expected UnsupportedProtocol, got {other:?}"),
        }
    }

    #[test]
    fn fsk_presets_carry_their_deviation() {
        assert_eq!(Preset::parse("FuriHalSubGhzPreset2FSKDev476Async"), Preset::Fsk(47_600));
        assert_eq!(Preset::parse("FuriHalSubGhzPresetCustom"), Preset::Custom);
    }

    #[test]
    fn comments_are_skipped_and_may_precede_anything() {
        let text = "\
# NullSec SubGHz - Doorbell 433MHz
Filetype: Flipper SubGhz RAW File
Version: 1
# For security research
Frequency: 433920000
Preset: FuriHalSubGhzPresetOok650Async
Protocol: RAW
RAW_Data: 100 -100
";
        let s = parse(text).expect("comments do not break the parse");
        assert_eq!(s.bursts[0].pulses.len(), 1);
    }
}
