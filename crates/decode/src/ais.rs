//! AIS message frames, per ITU-R M.1371.
//!
//! This is the frame layer only: payload bytes in, vessel state out. The
//! 162 MHz demodulator that produces those bytes is `dsp::ais`, and the split
//! is the same one Mode S has for the same reason. Everything below the
//! payload is a link layer question (NRZI, HDLC flags, bit stuffing, the
//! frame check sequence), everything above it is a table of bit offsets, and
//! keeping them apart means the tables can be checked against published
//! sentences without a radio in the room.
//!
//! # Bit order
//!
//! A payload here is bytes packed **most significant bit first**, so bits 0 to
//! 5 are the message type. That is the convention the NMEA AIVDM armoured
//! payload unpacks to, and therefore the one every published example and
//! every reference decode is written in. It is *not* the order the bits go on
//! the air: HDLC sends each byte least significant bit first, and undoing
//! that is `dsp::ais`'s job, done once at the point where the link layer ends.
//! Choosing the documented convention here is what makes these tables
//! checkable against evidence somebody else produced.
//!
//! # Position
//!
//! Unlike ADS-B, a position here is absolute. There is no compact position
//! reporting, no pairing of frames and no reference needed: one message
//! carries a latitude and a longitude outright, in ten-thousandths of a
//! minute. A tracker therefore has far less to do with AIS than with ADS-B,
//! and none of the zone ambiguity that makes a single Mode S frame dangerous.

/// Longitude and latitude arrive as ten-thousandths of a minute, so a degree
/// is sixty minutes of ten thousand units.
const COORD_SCALE: f64 = 600_000.0;

/// A parsed AIS message.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub msg_type: u8,
    pub repeat: u8,
    /// Maritime Mobile Service Identity, the nine digit identity of the
    /// station. This is the field that turns a stream of messages into tracks.
    pub mmsi: u32,
    pub kind: Message,
}

/// What a message says, for the types worth parsing.
#[derive(Clone, Debug, PartialEq)]
pub enum Message {
    /// A position report: types 1, 2 and 3 from Class A transponders, and
    /// 18 and 19 from the smaller Class B ones.
    Position(Position),
    /// Type 4, a shore station reporting its own position and the time. Worth
    /// keeping: it is a fixed point whose coordinates are surveyed, which
    /// makes it the one thing on the map that can check the receiver.
    BaseStation { position: Option<(f64, f64)>, utc: Option<Utc> },
    /// Types 5 and 24, which carry the name a vessel is known by.
    Static(Static),
    /// Type 21, a buoy or beacon rather than a vessel.
    AidToNavigation {
        name: Option<String>,
        position: Option<(f64, f64)>,
        aid_type: u8,
    },
    /// A type this decoder does not read, named so the log still counts it.
    Unsupported { msg_type: u8 },
}

/// Where a station is and how it is moving.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Position {
    /// Absent when the transponder reports it has no fix, which it signals
    /// with coordinates outside the possible range rather than with a flag.
    pub position: Option<(f64, f64)>,
    /// Speed over ground, knots.
    pub sog_kt: Option<f64>,
    /// Course over ground, degrees true. Where it is going, which for a
    /// vessel in a current is not where it is pointing.
    pub cog_deg: Option<f64>,
    /// True heading, degrees. Where it is pointing.
    pub heading_deg: Option<f64>,
    pub nav_status: Option<u8>,
    /// Rate of turn, degrees per minute, positive to starboard.
    pub turn_deg_min: Option<f64>,
    /// A Class B transponder: smaller, lower powered, and usually leisure
    /// rather than commercial traffic.
    pub class_b: bool,
}

/// The identity a vessel broadcasts separately from its position.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Static {
    pub name: Option<String>,
    pub callsign: Option<String>,
    pub imo: Option<u32>,
    pub ship_type: Option<u8>,
    pub destination: Option<String>,
    pub draught_m: Option<f64>,
}

/// The time a base station reports, which is UTC to the second.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Utc {
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Fewer bits than the common header, so there is not even an identity.
    TooShort,
}

/// Bits of a payload, read most significant first and bounds checked.
///
/// Every read returns `None` past the end rather than panicking, because a
/// truncated message is a thing the air produces routinely and a decoder that
/// panics on one is a decoder that stops the receiver.
struct Bits<'a> {
    data: &'a [u8],
    len: usize,
}

impl<'a> Bits<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, len: data.len() * 8 }
    }

    fn u(&self, start: usize, len: usize) -> Option<u64> {
        if len > 64 || start + len > self.len {
            return None;
        }
        let mut v: u64 = 0;
        for i in start..start + len {
            let bit = (self.data[i / 8] >> (7 - i % 8)) & 1;
            v = (v << 1) | u64::from(bit);
        }
        Some(v)
    }

    /// Two's complement over `len` bits.
    fn i(&self, start: usize, len: usize) -> Option<i64> {
        let v = self.u(start, len)?;
        let sign = 1u64 << (len - 1);
        Some(if v & sign != 0 { v as i64 - (1i64 << len) } else { v as i64 })
    }

    /// Six bit ASCII, as AIS packs names and callsigns.
    ///
    /// Values under 32 are the upper case block starting at `@`, the rest are
    /// ASCII as-is. `@` is the pad, so the string ends at the first one.
    fn text(&self, start: usize, len: usize) -> Option<String> {
        let mut s = String::with_capacity(len / 6);
        for i in (start..start + len).step_by(6) {
            let Some(v) = self.u(i, 6) else { break };
            let c = if v < 32 { (v as u8) + 64 } else { v as u8 };
            if c == b'@' {
                break;
            }
            s.push(c as char);
        }
        let s = s.trim().to_string();
        (!s.is_empty()).then_some(s)
    }

    /// A coordinate pair, `None` when either is outside the possible range.
    ///
    /// The standard signals "no fix" with 181 degrees of longitude and 91 of
    /// latitude rather than with a flag, so the range check is the test: a
    /// receiver that skips it puts vessels in a neat line off the coast of
    /// Africa, which is what an unchecked sentinel looks like on a map.
    fn coord(&self, lon_at: usize, lat_at: usize) -> Option<(f64, f64)> {
        let lon = self.i(lon_at, 28)? as f64 / COORD_SCALE;
        let lat = self.i(lat_at, 27)? as f64 / COORD_SCALE;
        (lat.abs() <= 90.0 && lon.abs() <= 180.0).then_some((lat, lon))
    }
}

/// Parse a payload into a message.
///
/// `payload` is the message bytes, most significant bit first; see the module
/// note on bit order.
pub fn parse(payload: &[u8]) -> Result<Frame, ParseError> {
    let b = Bits::new(payload);
    // Type, repeat and MMSI are common to every message, so a payload without
    // them is not an AIS message at all.
    let msg_type = b.u(0, 6).ok_or(ParseError::TooShort)? as u8;
    let repeat = b.u(6, 2).ok_or(ParseError::TooShort)? as u8;
    let mmsi = b.u(8, 30).ok_or(ParseError::TooShort)? as u32;

    let kind = match msg_type {
        1..=3 => Message::Position(class_a(&b)),
        4 => Message::BaseStation {
            position: b.coord(79, 107),
            utc: utc(&b),
        },
        5 => Message::Static(Static {
            imo: b.u(40, 30).map(|v| v as u32).filter(|v| *v != 0),
            callsign: b.text(70, 42),
            name: b.text(112, 120),
            ship_type: b.u(232, 8).map(|v| v as u8),
            draught_m: b.u(294, 8).map(|v| v as f64 / 10.0).filter(|v| *v > 0.0),
            destination: b.text(302, 120),
        }),
        18 | 19 => Message::Position(class_b(&b)),
        21 => Message::AidToNavigation {
            aid_type: b.u(38, 5).unwrap_or(0) as u8,
            name: b.text(43, 120),
            position: b.coord(164, 192),
        },
        // Type 24 comes in two halves: part A is the name, part B the type and
        // callsign. Reported as whichever half arrived; a tracker merges them
        // by MMSI, which is what it does with every other message anyway.
        24 => Message::Static(match b.u(38, 2) {
            Some(1) => Static {
                ship_type: b.u(40, 8).map(|v| v as u8),
                callsign: b.text(90, 42),
                ..Static::default()
            },
            _ => Static { name: b.text(40, 120), ..Static::default() },
        }),
        _ => Message::Unsupported { msg_type },
    };
    Ok(Frame { msg_type, repeat, mmsi, kind })
}

/// Types 1, 2 and 3: the Class A position report.
fn class_a(b: &Bits) -> Position {
    Position {
        position: b.coord(61, 89),
        nav_status: b.u(38, 4).map(|v| v as u8).filter(|v| *v < 15),
        turn_deg_min: b.i(42, 8).and_then(turn),
        sog_kt: b.u(50, 10).filter(|v| *v != 1023).map(|v| v as f64 / 10.0),
        cog_deg: b.u(116, 12).filter(|v| *v != 3600).map(|v| v as f64 / 10.0),
        heading_deg: b.u(128, 9).filter(|v| *v != 511).map(|v| v as f64),
        class_b: false,
    }
}

/// Types 18 and 19: the Class B report, whose fields sit at their own offsets
/// because it has no navigation status or rate of turn to carry.
fn class_b(b: &Bits) -> Position {
    Position {
        position: b.coord(57, 85),
        sog_kt: b.u(46, 10).filter(|v| *v != 1023).map(|v| v as f64 / 10.0),
        cog_deg: b.u(112, 12).filter(|v| *v != 3600).map(|v| v as f64 / 10.0),
        heading_deg: b.u(124, 9).filter(|v| *v != 511).map(|v| v as f64),
        class_b: true,
        ..Position::default()
    }
}

/// Rate of turn is stored square-rooted, so that one byte covers a range no
/// vessel exceeds at a resolution that matters near zero. -128 means the
/// transponder is not reporting one.
fn turn(v: i64) -> Option<f64> {
    if v == -128 {
        return None;
    }
    let m = (v as f64 / 4.733).powi(2);
    Some(if v < 0 { -m } else { m })
}

fn utc(b: &Bits) -> Option<Utc> {
    let year = b.u(38, 14)? as u16;
    // Zero is the standard's "not available" for the year, and a base station
    // without a clock has nothing to say about the time.
    if year == 0 {
        return None;
    }
    Some(Utc {
        year,
        month: b.u(52, 4)? as u8,
        day: b.u(56, 5)? as u8,
        hour: b.u(61, 5)? as u8,
        minute: b.u(66, 6)? as u8,
        second: b.u(72, 6)? as u8,
    })
}

/// What a navigation status code means, for a display.
pub fn nav_status_name(v: u8) -> &'static str {
    match v {
        0 => "under way",
        1 => "at anchor",
        2 => "not under command",
        3 => "restricted manoeuvrability",
        4 => "constrained by draught",
        5 => "moored",
        6 => "aground",
        7 => "fishing",
        8 => "under sail",
        11 => "towing astern",
        12 => "pushing ahead",
        14 => "AIS-SART",
        _ => "unknown",
    }
}

/// The broad class of a ship type code. The first digit is the category and
/// the second a detail nobody reads at a glance, so only the category is
/// named here.
pub fn ship_type_name(v: u8) -> &'static str {
    match v {
        20..=29 => "wing in ground",
        30 => "fishing",
        31 | 32 => "towing",
        33 => "dredging",
        34 => "diving",
        35 => "military",
        36 => "sailing",
        37 => "pleasure craft",
        40..=49 => "high speed craft",
        50 => "pilot",
        51 => "search and rescue",
        52 => "tug",
        53 => "port tender",
        55 => "law enforcement",
        58 => "medical transport",
        60..=69 => "passenger",
        70..=79 => "cargo",
        80..=89 => "tanker",
        _ => "other",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Unpack an NMEA AIVDM armoured payload into the bytes [`parse`] takes.
    ///
    /// The test vectors below are published sentences, which are written in
    /// this armouring, so this is the bridge between the evidence and the
    /// decoder. It is test-only on purpose: nothing in the receiver ever sees
    /// an NMEA sentence, because the frames arrive off the air as bits.
    fn unarmor(p: &str) -> Vec<u8> {
        let mut bits: Vec<u8> = Vec::with_capacity(p.len() * 6);
        for ch in p.bytes() {
            let mut v = ch - 48;
            if v > 40 {
                v -= 8;
            }
            for i in (0..6).rev() {
                bits.push((v >> i) & 1);
            }
        }
        let mut out = vec![0u8; bits.len().div_ceil(8)];
        for (i, b) in bits.iter().enumerate() {
            out[i / 8] |= b << (7 - i % 8);
        }
        out
    }

    /// A Class A position report off Le Havre.
    ///
    /// Published sentence, and the position is the check that matters: the
    /// field offsets in this file are only right if the vessel lands in the
    /// English Channel rather than in the Atlantic or in a field.
    #[test]
    fn a_class_a_report_lands_where_the_published_one_does() {
        let f = parse(&unarmor("13HOI:0P0000VOHLCnHQKwvL05Ip")).unwrap();
        assert_eq!(f.msg_type, 1);
        assert_eq!(f.mmsi, 227_006_760, "a French MMSI, MID 227");
        let Message::Position(p) = f.kind else { panic!("{f:?}") };
        let (lat, lon) = p.position.expect("a fix");
        assert!((lat - 49.475_576).abs() < 1e-5, "latitude {lat}");
        assert!((lon - 0.131_38).abs() < 1e-5, "longitude {lon}");
        assert_eq!(p.sog_kt, Some(0.0));
        assert_eq!(p.cog_deg, Some(36.7));
        assert_eq!(p.nav_status, Some(0));
        // 511 is the standard's "not available" for heading, so it must not
        // be reported as a vessel pointing just west of north.
        assert_eq!(p.heading_deg, None);
        assert!(!p.class_b);
    }

    #[test]
    fn a_second_class_a_report_lands_in_san_francisco_bay() {
        let f = parse(&unarmor("15M67FC000G?ufbE`FepT@3n00Sa")).unwrap();
        assert_eq!(f.mmsi, 366_053_209, "a US MMSI, MID 366");
        let Message::Position(p) = f.kind else { panic!("{f:?}") };
        let (lat, lon) = p.position.expect("a fix");
        assert!((lat - 37.802_118).abs() < 1e-5, "latitude {lat}");
        assert!((lon - -122.341_618).abs() < 1e-5, "longitude {lon}");
        assert_eq!(p.nav_status, Some(3));
    }

    /// Class B sits at different offsets from Class A, which is the whole
    /// reason it is a separate function: read with the Class A table this
    /// message puts a vessel a long way from the Caspian.
    #[test]
    fn a_class_b_report_uses_its_own_offsets() {
        let f = parse(&unarmor("B6CdCm0t3`tba35f@V9faHi7kP06")).unwrap();
        assert_eq!(f.msg_type, 18);
        assert_eq!(f.mmsi, 423_302_100);
        let Message::Position(p) = f.kind else { panic!("{f:?}") };
        let (lat, lon) = p.position.expect("a fix");
        assert!((lat - 40.005_283).abs() < 1e-5, "latitude {lat}");
        assert!((lon - 53.010_996).abs() < 1e-5, "longitude {lon}");
        assert_eq!(p.sog_kt, Some(1.4));
        assert!(p.class_b, "a Class B transponder must be marked as one");
    }

    /// The message that gives a vessel a name, which is the only reason a
    /// static report is worth reading at all.
    #[test]
    fn a_static_report_carries_the_name_and_the_voyage() {
        let p = unarmor(
            "55?MbV02;H;s<HtKR20EHE:0@T4@Dn2222222216L961O5Gf0NSQEp6ClRp888888888888880",
        );
        let f = parse(&p).unwrap();
        assert_eq!(f.msg_type, 5);
        assert_eq!(f.mmsi, 351_759_000);
        let Message::Static(s) = f.kind else { panic!("{f:?}") };
        assert_eq!(s.name.as_deref(), Some("EVER DIADEM"));
        assert_eq!(s.callsign.as_deref(), Some("3FOF8"));
        assert_eq!(s.imo, Some(9_134_270));
        assert_eq!(s.destination.as_deref(), Some("NEW YORK"));
        assert_eq!(s.ship_type, Some(70));
        assert_eq!(ship_type_name(70), "cargo");
        assert_eq!(s.draught_m, Some(12.2));
    }

    /// Type 24 arrives in two halves, and each has to be read as the half it
    /// says it is: part B's ship type sits where part A's name does.
    #[test]
    fn both_halves_of_a_type_24_report_are_read() {
        let a = parse(&unarmor("H42O55i18tMET00000000000000")).unwrap();
        let Message::Static(s) = a.kind else { panic!() };
        assert_eq!(s.name.as_deref(), Some("PROGUY"));

        let b = parse(&unarmor("H3s0dP4WGSD?PB0000000000000")).unwrap();
        let Message::Static(s) = b.kind else { panic!() };
        assert_eq!(s.ship_type, Some(39));
        assert_eq!(s.name, None, "part B carries no name");
    }

    /// A shore station, whose position is surveyed and whose clock is the
    /// only absolute time anything on the band reports.
    #[test]
    fn a_base_station_reports_a_position_and_the_time() {
        let f = parse(&unarmor("403OviQuMGCqWrRO9>E6fE700@GO")).unwrap();
        assert_eq!(f.msg_type, 4);
        let Message::BaseStation { position, utc } = f.kind else { panic!("{f:?}") };
        let (lat, lon) = position.expect("a surveyed position");
        assert!((lat - 36.883_766).abs() < 1e-5, "latitude {lat}");
        assert!((lon - -76.352_361).abs() < 1e-5, "longitude {lon}");
        let utc = utc.expect("a clock");
        assert_eq!((utc.year, utc.month, utc.day), (2007, 5, 14));
        assert_eq!((utc.hour, utc.minute, utc.second), (19, 57, 39));
    }

    /// The sentinel is how a transponder says it has no fix, and it has to be
    /// dropped rather than plotted: 181 east and 91 north is not a place.
    #[test]
    fn a_report_with_no_fix_has_no_position() {
        // Type 1 with longitude and latitude set to their unavailable values.
        let mut bits = vec![0u8; 168];
        let put = |bits: &mut Vec<u8>, at: usize, len: usize, v: u64| {
            for i in 0..len {
                bits[at + i] = ((v >> (len - 1 - i)) & 1) as u8;
            }
        };
        put(&mut bits, 0, 6, 1);
        put(&mut bits, 8, 30, 123_456_789);
        // 181 degrees east and 91 north, as the standard defines them.
        put(&mut bits, 61, 28, (181 * 600_000i64) as u64);
        put(&mut bits, 89, 27, (91 * 600_000i64) as u64);
        let mut payload = vec![0u8; 21];
        for (i, b) in bits.iter().enumerate() {
            payload[i / 8] |= b << (7 - i % 8);
        }
        let f = parse(&payload).unwrap();
        let Message::Position(p) = f.kind else { panic!() };
        assert_eq!(p.position, None, "the no-fix sentinel must not be plotted");
        assert_eq!(f.mmsi, 123_456_789);
    }

    /// A truncated message must not panic. The air produces these constantly,
    /// and a decoder that falls over on one takes the receiver with it.
    #[test]
    fn a_short_payload_is_refused_rather_than_read_past() {
        assert_eq!(parse(&[]), Err(ParseError::TooShort));
        assert_eq!(parse(&[0x14]), Err(ParseError::TooShort));
        // Enough for the header and nothing else: readable, with no position.
        let f = parse(&[0x04, 0x00, 0x00, 0x00, 0x00]).unwrap();
        assert_eq!(f.msg_type, 1);
        let Message::Position(p) = f.kind else { panic!() };
        assert_eq!(p.position, None);
    }

    #[test]
    fn an_unparsed_type_still_reports_its_identity() {
        // Type 8, a binary broadcast this decoder does not read. It still
        // says a station is there, which is what the log wants from it.
        let mut payload = vec![0u8; 21];
        payload[0] = 8 << 2;
        let f = parse(&payload).unwrap();
        assert_eq!(f.kind, Message::Unsupported { msg_type: 8 });
    }

    /// Rate of turn is stored square-rooted and signed, so the decoding has
    /// to undo both. -128 is the transponder declining to say.
    #[test]
    fn rate_of_turn_is_unpacked_from_its_square_root() {
        assert_eq!(turn(-128), None);
        assert_eq!(turn(0), Some(0.0));
        let right = turn(47).expect("a turn to starboard");
        assert!((right - 98.7).abs() < 1.0, "{right}");
        assert!(turn(-47).unwrap() < 0.0, "a turn to port is negative");
    }
}
