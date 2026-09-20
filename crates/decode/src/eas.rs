//! SAME headers: the bursts an EAS or NOAA Weather Radio alert opens with.
//!
//! Bits in, an alert out. The waveform above it is AFSK at 520.83 baud with
//! the tones inverted from the usual order ([`dsp::afsk::SAME`], mark at
//! 2083.3 Hz and space at 1562.5 Hz), and what arrives here is the bit
//! decisions: true for a mark.
//!
//! # The burst
//!
//! Sixteen bytes of 0xAB, then an ASCII header, sent three times with about
//! a second between the copies, and the same again as `NNNN` at the end of
//! the alert. Bytes go out least significant bit first with no start bit, no
//! stop bit and no parity, so the preamble is the only byte clock there is.
//! It self-aligns: the eight bit pattern of 0xAB has no shorter period, so
//! sixteen bits of it match at one phase only.
//!
//! # Correcting it
//!
//! There is no check sequence anywhere in SAME. What the standard gives
//! instead is the three copies: take the byte two of the three agree on
//! (47 CFR 11.31(c), which is why receivers are required to read all three).
//! [`Assembler`] does that, falls back to a copy that parses whole where the
//! vote does not, and refuses anything that is not a well formed header, so
//! the format itself is the check that keeps noise off the bus.
//!
//! The header is `ZCZC-ORG-EEE-PSSCCC-PSSCCC...+TTTT-JJJHHMM-LLLLLLLL-`:
//! who originated it, what it is about, the counties it covers as FIPS
//! codes, how long it runs, when it was issued and who sent it.

use common::packet::{AlertKind, Entity, Fact, Id, Proto, Severity};
/// The preamble byte, sixteen of which open every burst.
pub const PREAMBLE: u8 = 0xAB;
pub const PREAMBLE_BYTES: usize = 16;

/// The two headers a burst can carry.
pub const START: &[u8] = b"ZCZC";
pub const END: &[u8] = b"NNNN";

/// A header with 31 location codes, which is the most the standard allows,
/// is 268 characters. Nothing longer is a header.
const MAX_HEADER: usize = 268;

/// `NNNN` is the shortest thing a burst can legitimately carry.
const MIN_HEADER: usize = 4;

/// Who originated the alert.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Originator {
    /// A broadcast station or cable system.
    Broadcast,
    /// Civil authorities.
    Civil,
    /// The National Weather Service, which is everything on 162 MHz.
    WeatherService,
    /// The Primary Entry Point system, which is how a national alert enters.
    PrimaryEntryPoint,
    /// The Emergency Action Notification network of the older rules.
    EanNetwork,
    Other,
}

impl Originator {
    pub fn of(code: &str) -> Self {
        match code {
            "EAS" => Originator::Broadcast,
            "CIV" => Originator::Civil,
            "WXR" => Originator::WeatherService,
            "PEP" => Originator::PrimaryEntryPoint,
            "EAN" => Originator::EanNetwork,
            _ => Originator::Other,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Originator::Broadcast => "broadcast station",
            Originator::Civil => "civil authorities",
            Originator::WeatherService => "National Weather Service",
            Originator::PrimaryEntryPoint => "primary entry point",
            Originator::EanNetwork => "EAN network",
            Originator::Other => "unknown originator",
        }
    }
}

/// What the alert is about, from the three letter event code.
///
/// The FCC list (47 CFR 11.31) with the National Weather Service codes that
/// ride the same headers. A code nobody has published is still an alert, so
/// it keeps its letters and reads as itself.
pub fn event_label(code: &str) -> Option<&'static str> {
    Some(match code {
        "ADR" => "Administrative Message",
        "AVA" => "Avalanche Watch",
        "AVW" => "Avalanche Warning",
        "BLU" => "Blue Alert",
        "BZW" => "Blizzard Warning",
        "CAE" => "Child Abduction Emergency",
        "CDW" => "Civil Danger Warning",
        "CEM" => "Civil Emergency Message",
        "CFA" => "Coastal Flood Watch",
        "CFW" => "Coastal Flood Warning",
        "DMO" => "Practice Demonstration",
        "DSW" => "Dust Storm Warning",
        "EAN" => "Emergency Action Notification",
        "EAT" => "Emergency Action Termination",
        "EQW" => "Earthquake Warning",
        "EVI" => "Evacuation Immediate",
        "EWW" => "Extreme Wind Warning",
        "FFA" => "Flash Flood Watch",
        "FFS" => "Flash Flood Statement",
        "FFW" => "Flash Flood Warning",
        "FLA" => "Flood Watch",
        "FLS" => "Flood Statement",
        "FLW" => "Flood Warning",
        "FRW" => "Fire Warning",
        "FSW" => "Flash Freeze Warning",
        "FZW" => "Freeze Warning",
        "HLS" => "Hurricane Statement",
        "HMW" => "Hazardous Materials Warning",
        "HUA" => "Hurricane Watch",
        "HUW" => "Hurricane Warning",
        "HWA" => "High Wind Watch",
        "HWW" => "High Wind Warning",
        "LAE" => "Local Area Emergency",
        "LEW" => "Law Enforcement Warning",
        "NIC" => "National Information Center",
        "NMN" => "Network Message Notification",
        "NPT" => "National Periodic Test",
        "NST" => "National Silent Test",
        "NUW" => "Nuclear Power Plant Warning",
        "RHW" => "Radiological Hazard Warning",
        "RMT" => "Required Monthly Test",
        "RWT" => "Required Weekly Test",
        "SMW" => "Special Marine Warning",
        "SPS" => "Special Weather Statement",
        "SPW" => "Shelter In Place Warning",
        "SSA" => "Storm Surge Watch",
        "SSW" => "Storm Surge Warning",
        "SVA" => "Severe Thunderstorm Watch",
        "SVR" => "Severe Thunderstorm Warning",
        "SVS" => "Severe Weather Statement",
        "TOA" => "Tornado Watch",
        "TOE" => "Telephone Outage Emergency",
        "TOR" => "Tornado Warning",
        "TRA" => "Tropical Storm Watch",
        "TRW" => "Tropical Storm Warning",
        "TSA" => "Tsunami Watch",
        "TSW" => "Tsunami Warning",
        "VOW" => "Volcano Warning",
        "WSA" => "Winter Storm Watch",
        "WSW" => "Winter Storm Warning",
        _ => return None,
    })
}

/// Which part of a county the alert covers, the `P` of `PSSCCC`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Part {
    All,
    Northwest,
    North,
    Northeast,
    West,
    Central,
    East,
    Southwest,
    South,
    Southeast,
}

impl Part {
    fn of(digit: u8) -> Self {
        match digit {
            1 => Part::Northwest,
            2 => Part::North,
            3 => Part::Northeast,
            4 => Part::West,
            5 => Part::Central,
            6 => Part::East,
            7 => Part::Southwest,
            8 => Part::South,
            9 => Part::Southeast,
            _ => Part::All,
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Part::All => "all",
            Part::Northwest => "northwest",
            Part::North => "north",
            Part::Northeast => "northeast",
            Part::West => "west",
            Part::Central => "central",
            Part::East => "east",
            Part::Southwest => "southwest",
            Part::South => "south",
            Part::Southeast => "southeast",
        }
    }
}

/// One county or marine zone the alert covers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Location {
    pub part: Part,
    /// State FIPS code, or 75 for a marine zone.
    pub state: u8,
    /// County FIPS code within the state.
    pub county: u16,
}

impl Location {
    /// The six digits as they were sent.
    pub fn code(&self) -> String {
        let p = match self.part {
            Part::All => 0,
            Part::Northwest => 1,
            Part::North => 2,
            Part::Northeast => 3,
            Part::West => 4,
            Part::Central => 5,
            Part::East => 6,
            Part::Southwest => 7,
            Part::South => 8,
            Part::Southeast => 9,
        };
        format!("{p}{:02}{:03}", self.state, self.county)
    }

    /// The state the FIPS code names, where it names one.
    pub fn state_name(&self) -> Option<&'static str> {
        state_name(self.state)
    }
}

/// The state a FIPS code names. 75 is not a state: the National Weather
/// Service uses it for the coastal and offshore marine zones.
pub fn state_name(fips: u8) -> Option<&'static str> {
    Some(match fips {
        1 => "Alabama",
        2 => "Alaska",
        4 => "Arizona",
        5 => "Arkansas",
        6 => "California",
        8 => "Colorado",
        9 => "Connecticut",
        10 => "Delaware",
        11 => "District of Columbia",
        12 => "Florida",
        13 => "Georgia",
        15 => "Hawaii",
        16 => "Idaho",
        17 => "Illinois",
        18 => "Indiana",
        19 => "Iowa",
        20 => "Kansas",
        21 => "Kentucky",
        22 => "Louisiana",
        23 => "Maine",
        24 => "Maryland",
        25 => "Massachusetts",
        26 => "Michigan",
        27 => "Minnesota",
        28 => "Mississippi",
        29 => "Missouri",
        30 => "Montana",
        31 => "Nebraska",
        32 => "Nevada",
        33 => "New Hampshire",
        34 => "New Jersey",
        35 => "New Mexico",
        36 => "New York",
        37 => "North Carolina",
        38 => "North Dakota",
        39 => "Ohio",
        40 => "Oklahoma",
        41 => "Oregon",
        42 => "Pennsylvania",
        44 => "Rhode Island",
        45 => "South Carolina",
        46 => "South Dakota",
        47 => "Tennessee",
        48 => "Texas",
        49 => "Utah",
        50 => "Vermont",
        51 => "Virginia",
        53 => "Washington",
        54 => "West Virginia",
        55 => "Wisconsin",
        56 => "Wyoming",
        60 => "American Samoa",
        66 => "Guam",
        69 => "Northern Mariana Islands",
        72 => "Puerto Rico",
        75 => "marine zone",
        78 => "US Virgin Islands",
        _ => return None,
    })
}

/// When the alert was issued, as the header gives it: the day of the year
/// and the time, both UTC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Issued {
    pub day_of_year: u16,
    pub hour: u8,
    pub minute: u8,
}

impl Issued {
    pub fn label(&self) -> String {
        format!("day {} {:02}:{:02} UTC", self.day_of_year, self.hour, self.minute)
    }
}

/// One alert, as the header describes it.
#[derive(Clone, Debug, PartialEq)]
pub struct Alert {
    pub originator: Originator,
    /// The three letters as sent, kept beside the enum for display.
    pub originator_code: String,
    pub event_code: String,
    pub locations: Vec<Location>,
    /// How long the alert runs, in minutes, from `+TTTT`.
    pub valid_minutes: u32,
    pub issued: Issued,
    /// The eight character callsign of the station that sent it.
    pub station: String,
    /// The header exactly as it came off the air.
    pub header: String,
}

impl Alert {
    /// What the event code is called, or the code itself where nobody has
    /// published it.
    pub fn event(&self) -> &str {
        event_label(&self.event_code).unwrap_or(&self.event_code)
    }

    /// The counties, as a person would read them.
    pub fn where_label(&self) -> String {
        self.locations
            .iter()
            .map(|l| match (l.state_name(), l.part) {
                (Some(state), Part::All) => format!("{state} {:03}", l.county),
                (Some(state), part) => format!("{state} {:03} ({})", l.county, part.label()),
                (None, _) => l.code(),
            })
            .collect::<Vec<_>>()
            .join(", ")
    }

    /// One sentence for a person: what it is, who sent it, how long it runs.
    pub fn summary(&self) -> String {
        let (h, m) = (self.valid_minutes / 60, self.valid_minutes % 60);
        format!(
            "{} from {} for {}, valid {h}h{m:02}, issued {}",
            self.event(),
            self.station,
            self.where_label(),
            self.issued.label()
        )
    }
}

/// What a burst carried.
#[derive(Clone, Debug, PartialEq)]
pub enum Header {
    Alert(Alert),
    /// `NNNN`, which closes the alert the three headers opened.
    EndOfMessage,
}

/// Read a header burst. `None` where the bytes are not a well formed header,
/// which is the only check SAME offers.
pub fn parse(bytes: &[u8]) -> Option<Header> {
    let text = std::str::from_utf8(bytes).ok()?;
    if text.starts_with("NNNN") {
        return Some(Header::EndOfMessage);
    }
    let body = text.strip_prefix("ZCZC-")?;
    let (left, right) = body.split_once('+')?;
    let mut fields = left.split('-');
    let originator_code = three_letters(fields.next()?)?;
    let event_code = three_letters(fields.next()?)?;
    let mut locations = Vec::new();
    for f in fields {
        locations.push(location(f)?);
    }
    // A header with no county covers nothing, so it is not a header.
    if locations.is_empty() || locations.len() > 31 {
        return None;
    }

    let mut rest = right.split('-');
    let valid = digits(rest.next()?, 4)?;
    let valid_minutes = (valid / 100) * 60 + valid % 100;
    let stamp = digits(rest.next()?, 7)?;
    let issued = Issued {
        day_of_year: (stamp / 10_000) as u16,
        hour: ((stamp / 100) % 100) as u8,
        minute: (stamp % 100) as u8,
    };
    if issued.day_of_year == 0 || issued.day_of_year > 366 || issued.hour > 23 || issued.minute > 59
    {
        return None;
    }
    let station = rest.next()?.trim();
    if station.is_empty() || station.len() > 8 || !station.bytes().all(is_station_byte) {
        return None;
    }

    Some(Header::Alert(Alert {
        originator: Originator::of(&originator_code),
        originator_code,
        event_code,
        locations,
        valid_minutes,
        issued,
        station: station.to_string(),
        header: text.to_string(),
    }))
}

fn is_station_byte(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'/' || b == b' '
}

fn three_letters(f: &str) -> Option<String> {
    (f.len() == 3 && f.bytes().all(|b| b.is_ascii_uppercase() || b.is_ascii_digit()))
        .then(|| f.to_string())
}

fn digits(f: &str, n: usize) -> Option<u32> {
    (f.len() == n && f.bytes().all(|b| b.is_ascii_digit())).then(|| f.parse().ok())?
}

fn location(f: &str) -> Option<Location> {
    let v = digits(f, 6)?;
    Some(Location {
        part: Part::of((v / 100_000) as u8),
        state: ((v / 1_000) % 100) as u8,
        county: (v % 1_000) as u16,
    })
}

/// The preamble bits, in the order they go on the air: 0xAB least
/// significant bit first, so 1 1 0 1 0 1 0 1.
fn preamble_bits() -> [bool; 8] {
    let mut bits = [false; 8];
    for (i, b) in bits.iter_mut().enumerate() {
        *b = PREAMBLE >> i & 1 == 1;
    }
    bits
}

/// Two bytes of preamble as a 16 bit pattern, first bit in the top bit. Two
/// rather than one because one byte of an alternating pattern turns up in
/// noise every few hundred bits; sixteen bits of it does not, and a burst
/// carries 128 of them so there are 113 chances to catch it.
fn preamble_pattern() -> u16 {
    let mut p = 0u16;
    for _ in 0..2 {
        for b in preamble_bits() {
            p = p << 1 | u16::from(b);
        }
    }
    p
}

/// Bits to bursts: finds the preamble, byte-aligns to it, and collects the
/// printable ASCII that follows.
///
/// What comes out is whatever was on the air, whole or not. Refusing a copy
/// here because it does not parse would throw away the very copies the vote
/// exists to repair, so judging one is [`Assembler`]'s job.
pub struct Framer {
    hunt: u16,
    pattern: u16,
    /// `None` while hunting, else the bits of the byte being assembled.
    byte: Option<(u8, u32)>,
    burst: Vec<u8>,
    /// Bytes at the end of the burst that no text could contain.
    junk: usize,
}

impl Default for Framer {
    fn default() -> Self {
        Self::new()
    }
}

impl Framer {
    pub fn new() -> Self {
        Self { hunt: 0, pattern: preamble_pattern(), byte: None, burst: Vec::new(), junk: 0 }
    }

    pub fn reset(&mut self) {
        self.hunt = 0;
        self.byte = None;
        self.burst.clear();
        self.junk = 0;
    }

    /// The channel went quiet, so whatever was being collected ends here.
    pub fn quiet(&mut self) -> Option<Vec<u8>> {
        let out = self.finish();
        self.hunt = 0;
        out
    }

    /// One bit, true for a mark. `Some` when a burst ended.
    pub fn push(&mut self, bit: bool) -> Option<Vec<u8>> {
        self.hunt = self.hunt << 1 | u16::from(bit);
        let Some((byte, have)) = &mut self.byte else {
            if self.hunt == self.pattern {
                self.byte = Some((0, 0));
            }
            return None;
        };
        // Least significant bit first, with no start or stop bit: the
        // preamble is the whole of the byte clock.
        *byte |= u8::from(bit) << *have;
        *have += 1;
        if *have < 8 {
            return None;
        }
        let done = *byte;
        self.byte = Some((0, 0));
        // The rest of the preamble runs on past the sixteen bits that
        // aligned us, and it is not printable, so skipping it here is also
        // what starts the header at the right byte.
        if done == PREAMBLE && self.burst.is_empty() {
            return None;
        }
        // A byte nothing could have written is kept, not dropped: the copy
        // it damages is the copy the vote is there to repair, and dropping
        // it would move every byte after it out of position. Two in a row is
        // the pause after the header rather than an error inside it.
        self.burst.push(done);
        if !(done.is_ascii_graphic() || done == b' ') {
            self.junk += 1;
            if self.junk < 2 {
                return None;
            }
        } else {
            self.junk = 0;
            // A header ends at the dash after the station callsign, and that
            // is the only place a dash closes something that parses: the
            // dashes between the counties are followed by more header.
            // Without this the burst would run on into whatever the
            // discriminator made of the pause before the next copy.
            if done == b'-' {
                let ended = &self.burst[..self.burst.len() - 1];
                if parse(ended).is_some() {
                    let ended = ended.to_vec();
                    self.reset();
                    return Some(ended);
                }
            }
            if self.burst == END {
                self.reset();
                return Some(END.to_vec());
            }
            if self.burst.len() < MAX_HEADER {
                return None;
            }
        }
        let out = self.finish();
        self.hunt = 0;
        out
    }

    fn finish(&mut self) -> Option<Vec<u8>> {
        let mut burst = std::mem::take(&mut self.burst);
        self.byte = None;
        self.junk = 0;
        while burst.last().is_some_and(|b| !(b.is_ascii_graphic() || *b == b' ')) {
            burst.pop();
        }
        (burst.len() >= MIN_HEADER).then_some(burst)
    }
}

/// The three copies of a header, voted into one.
///
/// A copy takes up to 4.1 seconds on the air (268 bytes at 520.83 baud) and
/// the standard leaves about a second between them, so a fourth copy is
/// never coming after five and [`Assembler::WAIT_S`] seconds of quiet is
/// what ends the wait.
pub struct Assembler {
    copies: Vec<Vec<u8>>,
    waited_s: f64,
    refused: u64,
}

impl Default for Assembler {
    fn default() -> Self {
        Self::new()
    }
}

impl Assembler {
    /// How long after the last copy the vote is taken with what arrived.
    pub const WAIT_S: f64 = 6.0;

    pub fn new() -> Self {
        Self { copies: Vec::new(), waited_s: 0.0, refused: 0 }
    }

    pub fn reset(&mut self) {
        self.copies.clear();
        self.waited_s = 0.0;
    }

    /// Groups of bursts that framed and then were no header: a channel with
    /// something SAME shaped on it that never reads is a different fault
    /// from a quiet channel.
    pub fn refused(&self) -> u64 {
        self.refused
    }

    /// One copy off the air. `Some` once all three have arrived.
    pub fn push(&mut self, copy: Vec<u8>) -> Option<Vec<u8>> {
        self.waited_s = 0.0;
        self.copies.push(copy);
        (self.copies.len() >= 3).then(|| self.decide()).flatten()
    }

    /// Time passing with nothing heard. `Some` where copies were waiting and
    /// no more are coming.
    pub fn advance(&mut self, dt_s: f64) -> Option<Vec<u8>> {
        if self.copies.is_empty() {
            return None;
        }
        self.waited_s += dt_s;
        (self.waited_s >= Self::WAIT_S).then(|| self.decide()).flatten()
    }

    fn decide(&mut self) -> Option<Vec<u8>> {
        let copies = std::mem::take(&mut self.copies);
        self.waited_s = 0.0;
        let voted = vote(&copies);
        if parse(&voted).is_some() {
            return Some(voted);
        }
        // The vote is only a correction where two copies agree. Where it
        // fails, a copy that arrived whole still is one.
        let whole = copies.into_iter().find(|c| parse(c).is_some());
        if whole.is_none() {
            self.refused += 1;
        }
        whole
    }
}

/// The byte two of the copies agree on, at every position.
///
/// Where no two agree, the first copy's byte, which is what a receiver with
/// one copy would have had anyway. The length is the one at least two copies
/// share, or the first copy's.
pub fn vote(copies: &[Vec<u8>]) -> Vec<u8> {
    let Some(first) = copies.first() else { return Vec::new() };
    let len = copies
        .iter()
        .map(Vec::len)
        .find(|n| copies.iter().filter(|c| c.len() == *n).count() >= 2)
        .unwrap_or(first.len());
    (0..len)
        .map(|i| {
            let at = |c: &Vec<u8>| c.get(i).copied();
            let agreed = copies
                .iter()
                .filter_map(at)
                .find(|b| copies.iter().filter_map(at).filter(|o| o == b).count() >= 2);
            agreed.or_else(|| at(first)).unwrap_or(b' ')
        })
        .collect()
}

/// A header as bits on the air: the preamble, then the bytes least
/// significant bit first. For tests and for anything that wants to key one.
pub fn encode_bits(header: &str) -> Vec<bool> {
    let mut bits = Vec::with_capacity((PREAMBLE_BYTES + header.len()) * 8);
    for byte in std::iter::repeat_n(PREAMBLE, PREAMBLE_BYTES).chain(header.bytes()).chain([b'\0']) {
        for i in 0..8 {
            bits.push(byte >> i & 1 == 1);
        }
    }
    bits
}

/// What an emergency alert says.
///
/// This is the one protocol whose whole purpose is to interrupt somebody, so
/// it is an alert and nothing else: a warning a station broadcast, not a
/// message anybody wrote.
pub fn read(bytes: &[u8]) -> Option<Proto> {
    let alert = parse(bytes)?;
    Some(match &alert {
        Header::EndOfMessage => Proto::new("eas", "end_of_message"),
        Header::Alert(a) => Proto::new("eas", "alert")
            .by(Entity::new("eas-station", Id::Text(a.station.clone())))
            .saying(Fact::Alert(common::packet::Alert {
                kind: severity_of(a).0,
                severity: severity_of(a).1,
                text: Some(a.summary()),
            })),
    })
}

/// What the event code makes of it: a test is a drill and everything else
/// stands as the station sent it.
fn severity_of(a: &Alert) -> (AlertKind, Severity) {
    match a.event_code.as_str() {
        "RWT" | "RMT" | "NPT" | "DMO" => (AlertKind::Test, Severity::Advisory),
        "TOR" | "SVR" | "FFW" | "HUW" | "TSW" | "EWW" | "SQW" => {
            (AlertKind::Weather, Severity::Immediate)
        }
        "EAN" | "EAT" | "NUW" | "RHW" | "LEW" | "CDW" => (AlertKind::Civil, Severity::Immediate),
        _ => (AlertKind::Civil, Severity::Warning),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tornado warning for two Missouri counties, in the form the National
    /// Weather Service sends: the example header of NWSI 10-1712.
    const TOR: &str = "ZCZC-WXR-TOR-029095-029183+0030-1250100-KEAX/NWS-";

    #[test]
    fn a_header_reads_as_its_fields() {
        let Some(Header::Alert(a)) = parse(TOR.as_bytes()) else { panic!("not an alert") };
        assert_eq!(a.originator, Originator::WeatherService);
        assert_eq!(a.event_code, "TOR");
        assert_eq!(a.event(), "Tornado Warning");
        assert_eq!(a.locations.len(), 2);
        assert_eq!(a.locations[0], Location { part: Part::All, state: 29, county: 95 });
        assert_eq!(a.locations[1].county, 183);
        assert_eq!(a.locations[0].state_name(), Some("Missouri"));
        assert_eq!(a.valid_minutes, 30);
        assert_eq!(a.issued, Issued { day_of_year: 125, hour: 1, minute: 0 });
        assert_eq!(a.station, "KEAX/NWS");
        assert_eq!(a.where_label(), "Missouri 095, Missouri 183");
    }

    /// The part digit and the marine state code, which is what a special
    /// marine warning carries.
    #[test]
    fn a_part_county_and_a_marine_zone_read() {
        let h = "ZCZC-WXR-SMW-075530-509183+0100-2001200-KHOU/NWS-";
        let Some(Header::Alert(a)) = parse(h.as_bytes()) else { panic!("not an alert") };
        assert_eq!(a.locations[0], Location { part: Part::All, state: 75, county: 530 });
        assert_eq!(a.locations[0].state_name(), Some("marine zone"));
        assert_eq!(a.locations[1].part, Part::Central);
        assert_eq!(a.locations[1].code(), "509183");
        assert_eq!(a.valid_minutes, 60);
    }

    #[test]
    fn the_end_of_message_burst_reads() {
        assert_eq!(parse(b"NNNN"), Some(Header::EndOfMessage));
    }

    /// The format is the only check there is, so every way it can be wrong
    /// has to be refused.
    #[test]
    fn a_malformed_header_is_refused() {
        for bad in [
            "ZCZC-WXR-TOR-029095+0030-1250100-KEAX/NWS", // no station terminator is fine
            "ZCZC-WXR-TOR-02909-029183+0030-1250100-KEAX/NWS-", // five digit county
            "ZCZC-WXR-TOR+0030-1250100-KEAX/NWS-",       // no county at all
            "ZCZC-WXR-TOR-029095+0030-1259900-KEAX/NWS-", // hour 99
            "ZCZC-WXR-TOR-029095+0030-0000100-KEAX/NWS-", // day zero
            "ZCZC-WXR-TOR-029095+003-1250100-KEAX/NWS-", // three digit purge
            "XCZC-WXR-TOR-029095+0030-1250100-KEAX/NWS-", // not a header
            "ZCZC-WXR-TOR-029095+0030-1250100-KEAX/NWS/EXTRA-", // station too long
        ]
        .iter()
        .skip(1)
        {
            assert_eq!(parse(bad.as_bytes()), None, "{bad} was accepted");
        }
        // The first entry is well formed: a header whose trailing dash the
        // air cut off is still a header.
        assert!(matches!(
            parse(b"ZCZC-WXR-TOR-029095+0030-1250100-KEAX/NWS"),
            Some(Header::Alert(_))
        ));
    }

    /// The correction the standard asks for: one wrong byte in each of two
    /// copies, and the third settles both.
    #[test]
    fn two_of_three_agreeing_corrects_both_copies() {
        let mut a = TOR.as_bytes().to_vec();
        let mut b = TOR.as_bytes().to_vec();
        a[7] = b'#';
        b[20] = b'!';
        let c = TOR.as_bytes().to_vec();
        assert_eq!(parse(&a), None);
        assert_eq!(parse(&b), None);
        assert_eq!(vote(&[a, b, c]), TOR.as_bytes());
    }

    /// Two copies wrong in the same place is not correctable, and the
    /// assembler then takes the copy that arrived whole rather than nothing.
    #[test]
    fn a_whole_copy_is_used_where_the_vote_fails() {
        let mut bad = TOR.as_bytes().to_vec();
        bad[7] = b'#';
        let mut asm = Assembler::new();
        assert_eq!(asm.push(bad.clone()), None);
        assert_eq!(asm.push(bad), None);
        assert_eq!(asm.push(TOR.as_bytes().to_vec()), Some(TOR.as_bytes().to_vec()));
    }

    /// Two copies and then silence: the alert is published rather than held
    /// for a third that is not coming.
    #[test]
    fn two_copies_and_silence_still_publish() {
        let mut asm = Assembler::new();
        assert_eq!(asm.push(TOR.as_bytes().to_vec()), None);
        assert_eq!(asm.push(TOR.as_bytes().to_vec()), None);
        assert_eq!(asm.advance(Assembler::WAIT_S - 0.5), None);
        assert_eq!(asm.advance(1.0), Some(TOR.as_bytes().to_vec()));
        assert_eq!(asm.advance(60.0), None, "the alert was published twice");
    }

    /// The whole bit path: three keyed copies in, one voted header out.
    #[test]
    fn three_keyed_copies_frame_and_vote() {
        let mut framer = Framer::new();
        let mut asm = Assembler::new();
        let mut out = None;
        for copy in 0..3 {
            for (i, bit) in encode_bits(TOR).into_iter().enumerate() {
                // One byte of the second copy corrupted, to prove the vote is
                // what fixed it rather than luck.
                let bit = if copy == 1 && (152..160).contains(&i) { !bit } else { bit };
                if let Some(burst) = framer.push(bit) {
                    out = asm.push(burst).or(out);
                }
            }
            // The pause between copies, which is where a copy the framer
            // could not end on its terminating dash ends instead.
            for _ in 0..64 {
                if let Some(burst) = framer.push(false) {
                    out = asm.push(burst).or(out);
                }
            }
        }
        let voted = out.expect("three copies produced no alert");
        // The framer ends the header at its terminating dash, so that dash
        // is the one byte of it that never reaches the vote.
        assert_eq!(voted, TOR.trim_end_matches('-').as_bytes());
        let Some(Header::Alert(a)) = parse(&voted) else { panic!("the vote is not an alert") };
        assert_eq!((a.event(), a.station.as_str()), ("Tornado Warning", "KEAX/NWS"));
        assert_eq!(asm.refused(), 0);
    }

    /// Ten million bits of noise, which is five hours of air at 520.83 baud,
    /// and not one of them is a header.
    #[test]
    fn noise_produces_no_headers() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut framer = Framer::new();
        let mut asm = Assembler::new();
        let mut headers = 0;
        for _ in 0..10_000_000 {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            if let Some(b) = framer.push(seed & 1 == 1)
                && asm.push(b).is_some()
            {
                headers += 1;
            }
        }
        assert_eq!(headers, 0, "noise made {headers} alerts");
    }
}
