//! COSPAS-SARSAT 406 MHz distress beacons: EPIRBs, PLBs and ELTs.
//!
//! One burst every fifty seconds, 440 or 520 ms of it, and the whole of what
//! a satellite acts on is 112 or 144 bits. The layout and both error
//! correcting codes are C/S T.001, the beacon specification, whose Annex B
//! works an example of each code; those examples are the tests beside
//! [`crate::bits::bch_parity`].
//!
//! What a row off this means: a beacon claiming an identity, and where a
//! location protocol carries one, a position its own navigation device
//! produced. Nothing here is authenticated, and a beacon under test looks
//! exactly like a beacon in earnest except for the frame synchronisation
//! pattern, which is why [`Mode`] is carried through to the row.
//!
//! The 15 hexadecimal character identification is the thing a rescue centre
//! looks up, and for a location protocol it is the message with the position
//! bits put back to their defaults, so that a beacon has one identity
//! wherever it is.

use crate::bits::{BCH_63_51_GEN, BCH_127_106_GEN, bch_parity, bch63_51, bch127_106};
use common::packet::{Alert, AlertKind, Entity, Fact, Fix, Id, Named, Proto, Severity, ThingKind};
use dsp::biphase::CHIPS_PER_BIT;

/// Bits of the short message, and of the long one.
pub const SHORT_BITS: usize = 112;
pub const LONG_BITS: usize = 144;

/// The 15 ones every message opens with, then the 9 bits that say whether it
/// is a beacon in earnest or one being tested. A satellite refuses the
/// self-test pattern, which is the point of it.
pub const BIT_SYNC_BITS: usize = 15;
pub const FRAME_SYNC_NORMAL: [bool; 9] = [false, false, false, true, false, true, true, true, true];
pub const FRAME_SYNC_SELF_TEST: [bool; 9] =
    [false, true, true, false, true, false, false, false, false];

/// Whether the beacon was transmitting in earnest.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Distress,
    SelfTest,
}

impl Mode {
    /// The preamble a beacon in this mode keys: the bit synchronisation run
    /// and then the frame synchronisation pattern.
    pub fn preamble(&self) -> Vec<bool> {
        let mut bits = vec![true; BIT_SYNC_BITS];
        bits.extend(match self {
            Mode::Distress => FRAME_SYNC_NORMAL,
            Mode::SelfTest => FRAME_SYNC_SELF_TEST,
        });
        bits
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    Short,
    Long,
}

impl Format {
    pub fn bits(&self) -> usize {
        match self {
            Format::Short => SHORT_BITS,
            Format::Long => LONG_BITS,
        }
    }
}

/// The user protocols, which carry an identity and no position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UserProtocol {
    Orbitography,
    Aviation,
    Maritime,
    Serial,
    National,
    RadioCallSign,
    Test,
    Spare,
}

impl UserProtocol {
    fn from_code(code: u8) -> Self {
        match code {
            0b000 => UserProtocol::Orbitography,
            0b001 => UserProtocol::Aviation,
            0b010 => UserProtocol::Maritime,
            0b011 => UserProtocol::Serial,
            0b100 => UserProtocol::National,
            0b110 => UserProtocol::RadioCallSign,
            0b111 => UserProtocol::Test,
            _ => UserProtocol::Spare,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            UserProtocol::Orbitography => "orbitography",
            UserProtocol::Aviation => "aviation user",
            UserProtocol::Maritime => "maritime user",
            UserProtocol::Serial => "serial user",
            UserProtocol::National => "national user",
            UserProtocol::RadioCallSign => "radio call sign user",
            UserProtocol::Test => "test user",
            UserProtocol::Spare => "spare user",
        }
    }
}

/// The location protocols, which carry a position as well.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocationProtocol {
    /// Standard location, by the four-bit protocol code.
    StandardMmsi,
    StandardAircraftAddress,
    StandardEltSerial,
    StandardAircraftOperator,
    StandardEpirbSerial,
    StandardPlbSerial,
    StandardShipSecurity,
    StandardTest,
    NationalElt,
    NationalEpirb,
    NationalPlb,
    NationalTest,
    Rls,
    EltDt,
    /// A code the specification has not assigned.
    Unknown(u8),
}

impl LocationProtocol {
    fn from_code(code: u8) -> Self {
        match code {
            0b0010 => LocationProtocol::StandardMmsi,
            0b0011 => LocationProtocol::StandardAircraftAddress,
            0b0100 => LocationProtocol::StandardEltSerial,
            0b0101 => LocationProtocol::StandardAircraftOperator,
            0b0110 => LocationProtocol::StandardEpirbSerial,
            0b0111 => LocationProtocol::StandardPlbSerial,
            0b1100 => LocationProtocol::StandardShipSecurity,
            0b1110 => LocationProtocol::StandardTest,
            0b1000 => LocationProtocol::NationalElt,
            0b1010 => LocationProtocol::NationalEpirb,
            0b1011 => LocationProtocol::NationalPlb,
            0b1111 => LocationProtocol::NationalTest,
            0b1101 => LocationProtocol::Rls,
            0b1001 => LocationProtocol::EltDt,
            other => LocationProtocol::Unknown(other),
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            LocationProtocol::StandardMmsi => "standard location, EPIRB MMSI",
            LocationProtocol::StandardAircraftAddress => "standard location, aircraft address",
            LocationProtocol::StandardEltSerial => "standard location, ELT serial",
            LocationProtocol::StandardAircraftOperator => "standard location, aircraft operator",
            LocationProtocol::StandardEpirbSerial => "standard location, EPIRB serial",
            LocationProtocol::StandardPlbSerial => "standard location, PLB serial",
            LocationProtocol::StandardShipSecurity => "ship security",
            LocationProtocol::StandardTest => "standard location test",
            LocationProtocol::NationalElt => "national location, ELT",
            LocationProtocol::NationalEpirb => "national location, EPIRB",
            LocationProtocol::NationalPlb => "national location, PLB",
            LocationProtocol::NationalTest => "national location test",
            LocationProtocol::Rls => "return link service location",
            LocationProtocol::EltDt => "ELT(DT) location",
            LocationProtocol::Unknown(_) => "unassigned location",
        }
    }

    /// Whether this is one of the standard location protocols, whose
    /// identification and position fields are the ones read here.
    fn is_standard(&self) -> bool {
        matches!(
            self,
            LocationProtocol::StandardMmsi
                | LocationProtocol::StandardAircraftAddress
                | LocationProtocol::StandardEltSerial
                | LocationProtocol::StandardAircraftOperator
                | LocationProtocol::StandardEpirbSerial
                | LocationProtocol::StandardPlbSerial
                | LocationProtocol::StandardShipSecurity
                | LocationProtocol::StandardTest
        )
    }

    /// The coarse position in PDF-1, as the hemisphere flag and the degrees
    /// field of each of latitude and longitude, numbered as the
    /// specification numbers them, from one.
    ///
    /// Only for the standard location protocols. The national, return link
    /// and ELT(DT) protocols put their coarse position in different bits and
    /// at different resolutions, and nothing here reads those: their
    /// identity is reported as transmitted, which for a beacon that has
    /// moved is not the identity a rescue centre has on file.
    fn coarse_position(&self) -> Option<[(usize, std::ops::RangeInclusive<usize>); 2]> {
        self.is_standard().then_some([(65, 66..=74), (75, 76..=85)])
    }
}

/// What the beacon says it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Coding {
    User(UserProtocol),
    Location(LocationProtocol),
}

impl Coding {
    pub fn label(&self) -> &'static str {
        match self {
            Coding::User(p) => p.label(),
            Coding::Location(p) => p.label(),
        }
    }
}

/// The identity a standard location protocol carries. A user protocol's
/// identity is in the modified Baudot codes and is not read here: the 15 hex
/// characters are what a rescue centre looks the beacon up by either way.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Identity {
    /// The last six digits of the vessel's MMSI, and which beacon aboard.
    Mmsi {
        last_six: u32,
        beacon: u8,
    },
    /// The aircraft's 24-bit address, the same number Mode S transmits.
    AircraftAddress(u32),
    /// Type approval certificate number and the serial the maker gave it.
    Serial {
        certificate: u16,
        serial: u32,
    },
    Unknown,
}

/// Whether the beacon's own navigation device produced the position.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PositionSource {
    Internal,
    External,
}

/// One decoded beacon message.
#[derive(Clone, Debug, PartialEq)]
pub struct Beacon {
    pub format: Format,
    pub mode: Mode,
    /// ITU country code of the administration the beacon is registered with.
    pub country: u16,
    pub coding: Coding,
    /// The 15 hexadecimal characters that identify the beacon.
    pub hex_id: String,
    pub identity: Identity,
    /// Degrees, where the message carried a position rather than the default
    /// pattern that says the navigation device had none.
    pub position: Option<(f64, f64)>,
    pub source: Option<PositionSource>,
    /// Whether the beacon also transmits on 121.5 MHz for a homing receiver.
    pub homing_121_5: Option<bool>,
    /// Bits the two codes put back, over both protected fields.
    pub corrected: u32,
}

impl Beacon {
    /// What kind of beacon it is, from the protocol it was coded with: a
    /// vessel's, a person's or an aircraft's.
    pub fn kind(&self) -> &'static str {
        match self.coding {
            Coding::Location(LocationProtocol::StandardMmsi)
            | Coding::Location(LocationProtocol::StandardEpirbSerial)
            | Coding::Location(LocationProtocol::NationalEpirb)
            | Coding::Location(LocationProtocol::StandardShipSecurity)
            | Coding::User(UserProtocol::Maritime) => "EPIRB",
            Coding::Location(LocationProtocol::StandardPlbSerial)
            | Coding::Location(LocationProtocol::NationalPlb) => "PLB",
            Coding::Location(LocationProtocol::StandardEltSerial)
            | Coding::Location(LocationProtocol::StandardAircraftAddress)
            | Coding::Location(LocationProtocol::StandardAircraftOperator)
            | Coding::Location(LocationProtocol::NationalElt)
            | Coding::Location(LocationProtocol::EltDt)
            | Coding::User(UserProtocol::Aviation) => "ELT",
            _ => "beacon",
        }
    }

    pub fn summary(&self) -> String {
        let what = match self.mode {
            Mode::Distress => "distress",
            Mode::SelfTest => "self test",
        };
        let mut s = format!("{} {what} {}", self.kind(), self.hex_id);
        if let Some((lat, lon)) = self.position {
            s.push_str(&format!(" at {lat:.4} {lon:.4}"));
        }
        s
    }
}

/// Read a message off the air: 14 bytes for the short format, 18 for the
/// long, starting at the bit synchronisation run, which is how the beacon
/// transmits them and is a whole number of bytes either way.
///
/// `None` where the synchronisation patterns are not there, where the format
/// flag disagrees with how many bytes arrived, or where a protected field
/// is worse than its code can put back.
pub fn parse(bytes: &[u8]) -> Option<Beacon> {
    let mut bits: Vec<bool> =
        bytes.iter().flat_map(|b| (0..8).map(move |i| b >> (7 - i) & 1 != 0)).collect();
    let format = match bits.len() {
        SHORT_BITS => Format::Short,
        LONG_BITS => Format::Long,
        _ => return None,
    };
    if !bits[..BIT_SYNC_BITS].iter().all(|b| *b) {
        return None;
    }
    let sync = &bits[BIT_SYNC_BITS..BIT_SYNC_BITS + 9];
    let mode = match (sync == FRAME_SYNC_NORMAL, sync == FRAME_SYNC_SELF_TEST) {
        (true, _) => Mode::Distress,
        (_, true) => Mode::SelfTest,
        _ => return None,
    };
    let mut corrected = correct(&mut bits, 25, 85, 106, &BCH_1)?;
    if format == Format::Long {
        corrected += correct(&mut bits, 107, 132, 144, &BCH_2)?;
    }
    // The format flag and the length of the message are one fact written
    // twice, and a disagreement is a message read off the wrong chips. It is
    // checked after the code has run rather than before, because the flag is
    // the first bit of the protected field and a wrong one is a bit the code
    // puts back like any other.
    let flagged = match bits[24] {
        true => Format::Long,
        false => Format::Short,
    };
    if flagged != format {
        return None;
    }

    // Numbered as the specification numbers them, from one.
    let at = |n: usize| bits[n - 1];
    let field =
        |from: usize, to: usize| (from..=to).fold(0u64, |acc, n| acc << 1 | u64::from(bits[n - 1]));

    let coding = match at(26) {
        true => Coding::User(UserProtocol::from_code(field(37, 39) as u8)),
        false => Coding::Location(LocationProtocol::from_code(field(37, 40) as u8)),
    };
    let country = field(27, 36) as u16;
    let hex_id = hex_id(&bits, coding);

    let (identity, position, source, homing) = match coding {
        Coding::Location(p) if p.is_standard() => (
            standard_identity(p, &field),
            standard_position(format, &bits),
            Some(match at(111) {
                true => PositionSource::Internal,
                false => PositionSource::External,
            }),
            Some(at(112)),
        ),
        _ => (Identity::Unknown, None, None, None),
    };

    Some(Beacon {
        format,
        mode,
        country,
        coding,
        hex_id,
        identity,
        position,
        source,
        homing_121_5: homing,
        corrected,
    })
}

/// One of the two codes a beacon message carries, as the decoder needs it.
struct Code {
    generator: u64,
    parity_bits: usize,
    /// The unshortened length the decoder works at.
    length: usize,
    decode: fn(&mut [bool]) -> Option<u32>,
}

/// The 21 bits over PDF-1, a shortened BCH(127,106), and the 12 bits over
/// PDF-2, a shortened BCH(63,51).
const BCH_1: Code =
    Code { generator: BCH_127_106_GEN, parity_bits: 21, length: 127, decode: bch127_106 };
const BCH_2: Code =
    Code { generator: BCH_63_51_GEN, parity_bits: 12, length: 63, decode: bch63_51 };

/// Check and correct one protected field, given as the specification numbers
/// its bits: data from `from` to `data_end`, parity from there to `end`.
///
/// The codes are shortened, so the word is padded with zeros up to the
/// length the decoder works at. A correction landing in that padding is
/// evidence the field was worse than the code can place, not a correction to
/// keep: those positions were never transmitted.
fn correct(
    bits: &mut [bool],
    from: usize,
    data_end: usize,
    end: usize,
    code: &Code,
) -> Option<u32> {
    let sent = end - from + 1;
    let mut word = vec![false; code.length];
    for (i, slot) in word.iter_mut().enumerate().take(sent) {
        *slot = bits[end - 1 - i];
    }
    let put_back = (code.decode)(&mut word)?;
    if word[sent..].iter().any(|b| *b) {
        return None;
    }
    for (i, bit) in word.iter().enumerate().take(sent) {
        bits[end - 1 - i] = *bit;
    }
    // The syndromes are zero now, so the parity has to match what the data
    // generates. It always does, and checking says so for nothing.
    let data: Vec<bool> = bits[from - 1..data_end].to_vec();
    let parity = bch_parity(&data, code.generator, code.parity_bits);
    let sent_parity = (data_end + 1..=end).fold(0u64, |acc, n| acc << 1 | u64::from(bits[n - 1]));
    (parity == sent_parity).then_some(put_back)
}

/// The 15 hexadecimal characters: bits 26 to 85, with a location protocol's
/// coarse position put back to its default so the identity does not move
/// with the beacon.
fn hex_id(bits: &[bool], coding: Coding) -> String {
    let mut id: Vec<bool> = bits[25..85].to_vec();
    if let Coding::Location(p) = coding
        && let Some(halves) = p.coarse_position()
    {
        // The default is a zero hemisphere flag and a degrees field of all
        // ones, which is a value no position can take.
        for (flag, degrees) in halves {
            id[flag - 26] = false;
            for n in degrees {
                id[n - 26] = true;
            }
        }
    }
    id.chunks(4)
        .map(|n| std::char::from_digit(nibble(n) as u32, 16).unwrap_or('0'))
        .collect::<String>()
        .to_uppercase()
}

fn nibble(bits: &[bool]) -> u8 {
    bits.iter().fold(0u8, |acc, b| acc << 1 | u8::from(*b))
}

fn standard_identity(p: LocationProtocol, field: &impl Fn(usize, usize) -> u64) -> Identity {
    match p {
        LocationProtocol::StandardMmsi | LocationProtocol::StandardShipSecurity => {
            Identity::Mmsi { last_six: field(41, 60) as u32, beacon: field(61, 64) as u8 }
        }
        LocationProtocol::StandardAircraftAddress => {
            Identity::AircraftAddress(field(41, 64) as u32)
        }
        LocationProtocol::StandardEltSerial
        | LocationProtocol::StandardEpirbSerial
        | LocationProtocol::StandardPlbSerial
        | LocationProtocol::StandardTest => {
            Identity::Serial { certificate: field(41, 50) as u16, serial: field(51, 64) as u32 }
        }
        // The aircraft operator designator is three letters of modified
        // Baudot in bits 41 to 55; only the serial after it is a number.
        LocationProtocol::StandardAircraftOperator => {
            Identity::Serial { certificate: 0, serial: field(56, 64) as u32 }
        }
        _ => Identity::Unknown,
    }
}

/// The position a standard location protocol carries: a quarter of a degree
/// in PDF-1, and in a long message an offset to four seconds in PDF-2.
///
/// `None` where the navigation device had nothing to encode, which the
/// specification spells as a degrees field of all ones.
fn standard_position(format: Format, bits: &[bool]) -> Option<(f64, f64)> {
    let field =
        |from: usize, to: usize| (from..=to).fold(0u64, |acc, n| acc << 1 | u64::from(bits[n - 1]));
    let (lat_deg, lon_deg) = (field(66, 74), field(76, 85));
    if lat_deg == 0b1_1111_1111 || lon_deg == 0b11_1111_1111 {
        return None;
    }
    let south = bits[64];
    let west = bits[74];
    let mut lat = lat_deg as f64 / 4.0;
    let mut lon = lon_deg as f64 / 4.0;

    if format == Format::Long {
        // Minutes and seconds off the coarse position, signed. A seconds
        // field of 15 is 60 seconds, which is the default pattern saying
        // there is no offset to add.
        let offset = |sign: usize, minutes: (usize, usize), seconds: (usize, usize)| {
            let s = field(seconds.0, seconds.1);
            match s {
                0b1111 => None,
                s => {
                    let magnitude =
                        field(minutes.0, minutes.1) as f64 / 60.0 + s as f64 * 4.0 / 3600.0;
                    Some(match bits[sign - 1] {
                        true => magnitude,
                        false => -magnitude,
                    })
                }
            }
        };
        lat += offset(113, (114, 118), (119, 122))?;
        lon += offset(123, (124, 128), (129, 132))?;
    }

    Some((
        match south {
            true => -lat,
            false => lat,
        },
        match west {
            true => -lon,
            false => lon,
        },
    ))
}

/// What a distress beacon says.
///
/// The whole point of one: somebody is in trouble, or is testing the thing
/// that says so. A self test is an alert of its own kind so that a listener
/// can tell the drill from the real thing.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    let b = parse(bytes)?;
    let mut p = Proto::new("epirb", mode_kind(b.mode))
        .by(Entity::new("epirb", Id::Text(b.hex_id.clone())).named(b.kind()))
        .saying(Fact::Named(Named::new(b.kind(), ThingKind::Mark)))
        .saying(Fact::Alert(Alert {
            kind: match b.mode {
                Mode::Distress => AlertKind::Distress,
                Mode::SelfTest => AlertKind::Test,
            },
            severity: match b.mode {
                Mode::Distress => Severity::Immediate,
                Mode::SelfTest => Severity::Advisory,
            },
            text: Some(b.summary()),
        }));
    if let Some((lat, lon)) = b.position {
        p = p.saying(Fact::Position(Fix { lat, lon, precision_bits: None }));
    }
    Some(p)
}

fn mode_kind(m: Mode) -> &'static str {
    match m {
        Mode::Distress => "distress",
        Mode::SelfTest => "self_test",
    }
}

pub fn detail(b: &Beacon) -> String {
    let coding = match b.coding {
        Coding::User(_) => "user protocol",
        Coding::Location(_) => "location protocol",
    };
    format!("{}, {coding}, country {}", b.coding.label(), b.country)
}

/// Chips of the preamble that may disagree and still count as a match.
///
/// The preamble is 24 bits, so 48 chips. Measured on synthesised bursts in
/// noise: at 8 allowed, every burst is read down to the 7.8 dB the chips
/// themselves survive, and ten minutes of noise produces no message at all,
/// because the two codes refuse what the preamble let through.
pub const SYNC_TOLERANCE: usize = 5;

/// Bits of the preamble the search matches on: the frame synchronisation
/// pattern and the tail of the bit synchronisation run.
pub const SYNC_BITS: usize = 16;

/// How far the phase must swing, in radians, averaged over the chips the
/// preamble was matched on, before the match counts as a transmission.
///
/// A beacon keys 1.1 radians either side of the carrier. Measured as the
/// mean size of a chip: 0.95 radians on the keyed message, 0.00 to 0.11 on
/// the unmodulated carrier in front of it, and 0.37 on noise alone, where
/// the phase is uniform and a chip averages what the integration leaves. So
/// this separates a message from both, and without it four beacons come out
/// of ten minutes of noise: a pattern match on noise is rare but ten
/// minutes is a million chances at it.
pub const MIN_SWING_RAD: f32 = 0.6;

/// A beacon message, less the preamble, in chips.
pub const SHORT_CHIPS: usize = (SHORT_BITS - 24) * CHIPS_PER_BIT;

pub const LONG_CHIPS: usize = (LONG_BITS - 24) * CHIPS_PER_BIT;

/// A message being read off the chips that followed a preamble.
struct Reading {
    mode: Mode,
    inverted: bool,
    chips: Vec<f32>,
}

/// Messages cut out of a stream of biphase chips.
///
/// Above the waveform and below the payload: the chips come from any
/// [`dsp::biphase`] demodulator at 400 baud, and what leaves is a whole
/// transmission from bit one, preamble included, that [`parse`] accepted.
#[derive(Default)]
pub struct Framer {
    reading: Option<Reading>,
    /// The chips being scored against the preamble.
    window: Vec<f32>,
    frames: u64,
}

impl Framer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Messages that checked.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Feed one chip, and hand back the message where it completed one.
    ///
    /// The bits of the preamble are known rather than read, so what is
    /// emitted is the whole transmission from bit one, which is 14 bytes for
    /// a short message and 18 for a long one.
    pub fn push(&mut self, chip: f32) -> Option<Vec<u8>> {
        if let Some(mut reading) = self.reading.take() {
            reading.chips.push(chip);
            // The format flag is the first bit after the preamble, so how
            // much is still to come is known two chips in.
            let want = match reading.chips.first().zip(reading.chips.get(1)) {
                Some((a, b)) => match (a - b > 0.0) != reading.inverted {
                    true => LONG_CHIPS,
                    false => SHORT_CHIPS,
                },
                None => LONG_CHIPS,
            };
            if reading.chips.len() < want {
                self.reading = Some(reading);
                return None;
            }
            let mut bits = reading.mode.preamble();
            let read = crate::framing::biphase_l_bits(&reading.chips, reading.inverted);
            bits.extend((0..read.len()).filter_map(|i| read.get(i)));
            let bytes: Vec<u8> = bits
                .chunks(8)
                .map(|b| b.iter().fold(0u8, |acc, v| acc << 1 | u8::from(*v)))
                .collect();
            parse(&bytes)?;
            self.frames += 1;
            return Some(bytes);
        }

        // One chip more than the preamble, because the decision is taken a
        // chip late: a preamble that opens with a run of identical bits
        // reads almost as well one chip early and upside down, and only the
        // two scores side by side tell them apart.
        let preamble = SYNC_BITS * CHIPS_PER_BIT;
        self.window.push(chip);
        if self.window.len() > preamble + 1 {
            self.window.remove(0);
        }
        if self.window.len() <= preamble {
            return None;
        }
        let swing = self.window[..preamble].iter().map(|c| c.abs()).sum::<f32>() / preamble as f32;
        if swing < MIN_SWING_RAD {
            return None;
        }
        for mode in [Mode::Distress, Mode::SelfTest] {
            let all = mode.preamble();
            let bits = &all[all.len() - SYNC_BITS..];
            let here = crate::framing::biphase_l_match(&self.window[..preamble], bits);
            let next = crate::framing::biphase_l_match(&self.window[1..], bits);
            let Some((wrong, inverted)) = here else { continue };
            if wrong > SYNC_TOLERANCE || next.is_some_and(|(w, _)| w < wrong) {
                continue;
            }
            // The preamble ended a chip ago, so the message starts with the
            // chip that has just arrived.
            let mut chips = Vec::with_capacity(LONG_CHIPS);
            chips.push(chip);
            self.reading = Some(Reading { mode, inverted, chips });
            self.window.clear();
            break;
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a message the way a beacon does: the preamble, the 61 bits of
    /// PDF-1 with its 21 bit code, and for a long message the 26 bits of
    /// PDF-2 with its 12 bit code.
    pub(crate) fn keyed(mode: Mode, pdf1: &[bool], pdf2: Option<&[bool]>) -> Vec<u8> {
        assert_eq!(pdf1.len(), 61);
        let mut bits = mode.preamble();
        bits.extend_from_slice(pdf1);
        let parity = bch_parity(pdf1, BCH_127_106_GEN, 21);
        bits.extend((0..21).rev().map(|i| parity >> i & 1 != 0));
        if let Some(pdf2) = pdf2 {
            assert_eq!(pdf2.len(), 26);
            bits.extend_from_slice(pdf2);
            let parity = bch_parity(pdf2, BCH_63_51_GEN, 12);
            bits.extend((0..12).rev().map(|i| parity >> i & 1 != 0));
        }
        bits.chunks(8).map(nibble_byte).collect()
    }

    fn nibble_byte(bits: &[bool]) -> u8 {
        bits.iter().fold(0u8, |acc, b| acc << 1 | u8::from(*b))
    }

    /// Bits of a value, most significant first.
    pub(crate) fn num(value: u64, width: usize) -> Vec<bool> {
        (0..width).rev().map(|i| value >> i & 1 != 0).collect()
    }

    /// A standard location EPIRB with an MMSI, at a position off the west of
    /// Ireland: country 232 (United Kingdom), MMSI ending 123456, the first
    /// beacon aboard, 53 21' 36" N 10 15' 28" W.
    pub(crate) fn an_epirb() -> Vec<u8> {
        let mut pdf1 = vec![true, false];
        pdf1.extend(num(232, 10));
        pdf1.extend(num(0b0010, 4));
        pdf1.extend(num(123_456, 20));
        pdf1.extend(num(1, 4));
        // 53.25 N, 10.25 W: the coarse position, to a quarter degree.
        pdf1.push(false);
        pdf1.extend(num(53 * 4 + 1, 9));
        pdf1.push(true);
        pdf1.extend(num(10 * 4 + 1, 10));

        let mut pdf2 = vec![true, true, false, true, true, false];
        // +6 minutes 36 seconds of latitude, -3 minutes 28 seconds of
        // longitude, which is 53 21' 36" N 10 11' 32" W.
        pdf2.push(true);
        pdf2.extend(num(6, 5));
        pdf2.extend(num(36 / 4, 4));
        pdf2.push(false);
        pdf2.extend(num(3, 5));
        pdf2.extend(num(28 / 4, 4));
        keyed(Mode::Distress, &pdf1, Some(&pdf2))
    }

    #[test]
    fn a_standard_location_epirb_is_read_whole() {
        let air = an_epirb();
        assert_eq!(air.len(), 18, "a long message is 18 bytes");
        let b = parse(&air).expect("a beacon");
        assert_eq!(b.format, Format::Long);
        assert_eq!(b.mode, Mode::Distress);
        assert_eq!(b.country, 232);
        assert_eq!(b.coding, Coding::Location(LocationProtocol::StandardMmsi));
        assert_eq!(b.identity, Identity::Mmsi { last_six: 123_456, beacon: 1 });
        assert_eq!(b.kind(), "EPIRB");
        assert_eq!(b.corrected, 0);
        assert_eq!(b.source, Some(PositionSource::Internal));
        assert_eq!(b.homing_121_5, Some(false));
        let (lat, lon) = b.position.expect("a position");
        assert!((lat - (53.0 + 21.0 / 60.0 + 36.0 / 3600.0)).abs() < 1e-9, "{lat}");
        assert!((lon + (10.0 + 11.0 / 60.0 + 32.0 / 3600.0)).abs() < 1e-9, "{lon}");
        // The identity is the message with the position defaulted, so it is
        // the same 15 characters wherever the vessel is.
        assert_eq!(b.hex_id, "1D043C4802FFBFF");
        assert_eq!(b.summary(), "EPIRB distress 1D043C4802FFBFF at 53.3600 -10.1922");
    }

    /// The same beacon at another place keeps the identity a rescue centre
    /// looks it up by.
    #[test]
    fn the_identity_does_not_move_with_the_beacon() {
        let a = parse(&an_epirb()).expect("a beacon");
        let mut pdf1 = vec![true, false];
        pdf1.extend(num(232, 10));
        pdf1.extend(num(0b0010, 4));
        pdf1.extend(num(123_456, 20));
        pdf1.extend(num(1, 4));
        pdf1.push(true);
        pdf1.extend(num(33 * 4 + 2, 9));
        pdf1.push(false);
        pdf1.extend(num(151 * 4, 10));
        let mut pdf2 = vec![true, true, false, true, false, true];
        pdf2.push(true);
        pdf2.extend(num(0, 5));
        pdf2.extend(num(0, 4));
        pdf2.push(true);
        pdf2.extend(num(0, 5));
        pdf2.extend(num(0, 4));
        let b = parse(&keyed(Mode::Distress, &pdf1, Some(&pdf2))).expect("a beacon");
        assert_eq!(b.hex_id, a.hex_id);
        assert_eq!(b.source, Some(PositionSource::External));
        assert_eq!(b.homing_121_5, Some(true));
        assert_eq!(b.position, Some((-33.5, 151.0)));
    }

    /// A beacon with no fix transmits the default pattern, and a position
    /// invented from it would send a lifeboat to the wrong ocean.
    #[test]
    fn a_beacon_without_a_fix_reports_no_position() {
        let mut pdf1 = vec![true, false];
        pdf1.extend(num(232, 10));
        pdf1.extend(num(0b0111, 4));
        pdf1.extend(num(300, 10));
        pdf1.extend(num(4_095, 14));
        pdf1.push(false);
        pdf1.extend(num(0b1_1111_1111, 9));
        pdf1.push(false);
        pdf1.extend(num(0b11_1111_1111, 10));
        let mut pdf2 = vec![true, true, false, true, true, false];
        pdf2.push(true);
        pdf2.extend(num(0, 5));
        pdf2.extend(num(0b1111, 4));
        pdf2.push(true);
        pdf2.extend(num(0, 5));
        pdf2.extend(num(0b1111, 4));
        let b = parse(&keyed(Mode::Distress, &pdf1, Some(&pdf2))).expect("a beacon");
        assert_eq!(b.position, None);
        assert_eq!(b.kind(), "PLB");
        assert_eq!(b.identity, Identity::Serial { certificate: 300, serial: 4_095 });
    }

    /// A short message from a user protocol beacon, and a self-test burst,
    /// which the satellites refuse and a receiver should still show as what
    /// it is.
    #[test]
    fn a_short_self_test_message_is_read_and_labelled() {
        let mut pdf1 = vec![false, true];
        pdf1.extend(num(366, 10));
        pdf1.extend(num(0b011, 3));
        pdf1.extend(num(0x2_abcd_ef01, 46));
        let air = keyed(Mode::SelfTest, &pdf1, None);
        assert_eq!(air.len(), 14, "a short message is 14 bytes");
        let b = parse(&air).expect("a beacon");
        assert_eq!(b.format, Format::Short);
        assert_eq!(b.mode, Mode::SelfTest);
        assert_eq!(b.country, 366);
        assert_eq!(b.coding, Coding::User(UserProtocol::Serial));
        assert_eq!(b.identity, Identity::Unknown);
        assert_eq!(b.position, None);
    }

    /// Three wrong bits anywhere in the first protected field come back, and
    /// the message is the one that was keyed. Four do not, which is the code
    /// saying so rather than a beacon being invented.
    #[test]
    fn the_codes_put_back_three_wrong_bits_and_refuse_four() {
        let air = an_epirb();
        let flip = |air: &[u8], at: &[usize]| {
            let mut bad = air.to_vec();
            for n in at {
                bad[n / 8] ^= 0x80 >> (n % 8);
            }
            bad
        };
        let want = parse(&air).expect("a beacon");
        let mut read = 0;
        for start in 24..80usize {
            let bad = flip(&air, &[start, start + 2, start + 5]);
            let got = parse(&bad).expect("three wrong bits in a protected field");
            assert_eq!(got.corrected, 3, "from {start}");
            assert_eq!(Beacon { corrected: 0, ..got }, want, "three wrong bits from {start}");
            read += 1;
        }
        assert_eq!(read, 56, "{read} of 56 triples were put back");

        let mut refused = 0;
        for start in 24..80usize {
            let bad = flip(&air, &[start, start + 2, start + 5, start + 9]);
            refused += u32::from(parse(&bad).is_none_or(|b| b != want));
        }
        assert_eq!(refused, 56, "a message with four wrong bits was believed");
    }

    /// Anything that is not a beacon message is refused: the wrong length,
    /// a frame synchronisation pattern that is neither, a format flag that
    /// disagrees with the length, and a field the code cannot place.
    #[test]
    fn nothing_else_is_read_as_a_beacon() {
        let air = an_epirb();
        assert_eq!(parse(&air[..17]), None, "a truncated message");
        assert_eq!(parse(&[0; 18]), None, "silence");
        assert_eq!(parse(&[0xff; 18]), None, "a carrier");
        let mut wrong_sync = air.clone();
        wrong_sync[2] ^= 0x0f;
        assert_eq!(parse(&wrong_sync), None, "a frame synchronisation that is neither");
        // A long message whose flag says short, and whose code agrees with
        // the flag: the length it arrived in is the evidence against it.
        let mut pdf1 = vec![false, false];
        pdf1.extend(num(232, 10));
        pdf1.extend(num(0b0010, 4));
        pdf1.extend(vec![false; 45]);
        let mut pdf2 = vec![true, true, false, true, true, false];
        pdf2.extend(vec![false; 20]);
        let lying = keyed(Mode::Distress, &pdf1, Some(&pdf2));
        assert_eq!(lying.len(), 18);
        assert_eq!(parse(&lying), None, "a format flag disagreeing with the length");
    }
}
