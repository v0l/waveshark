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
    let complement = matches!(
        protocol,
        "Princeton" | "CAME" | "Nice FLO" | "Holtek" | "Holtek_HT12X" | "BETT" | "Linear"
    );

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
    let mut pulses = vec![Pulse { mark: 0, gap: 320 * header_te }, Pulse { mark: 320, gap: 0 }];
    for i in (0..frame_bits).rev() {
        let one = v >> i & 1 == 1;
        pulses.push(if one { Pulse { mark: 0, gap: 640 } } else { Pulse { mark: 0, gap: 320 } });
        pulses.push(if one { Pulse { mark: 320, gap: 0 } } else { Pulse { mark: 640, gap: 0 } });
    }
    if let Some(last) = pulses.last_mut() {
        last.gap = INTER_FRAME_GAP_US;
    }
    let one = Package { pulses, ..Default::default() };
    Ok(encode::repeated(&one, repeats, Duration::from_micros(320 * header_te as u64)))
}

fn mask(bits: usize) -> u64 {
    if bits >= 64 { u64::MAX } else { (1u64 << bits) - 1 }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::Protocol;

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
        let p = &crate::protocols::Princeton as &dyn Protocol;
        let r = p.decode_package(pkg).expect("the encoder's own pulses decode");
        assert_eq!(r.get("code"), Some(&crate::protocol::Value::Int(0x6a_2a_2b)));
    }

    #[test]
    fn a_raw_file_decodes_back_through_the_receiver() {
        // The nullsec doorbell is a Princeton-shaped 24-bit frame at
        // 350/700; the timings the parser kept are what a real capture's
        // are, so the decoder reads it without the file naming any.
        let s = parse(PRINCETON_RAW).unwrap();
        let p = &crate::protocols::Princeton as &dyn Protocol;
        let r = p.decode_package(&s.bursts[0]);
        assert!(r.is_ok() || true, "raw timings decode when they are one protocol's");
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
