//! APRS payloads: what an AX.25 information field means.
//!
//! Position arrives in three different encodings and they share almost
//! nothing, which is why this is longer than it looks like it should be.
//!
//! - **Uncompressed** is human readable: `4903.50N/07201.75W-`, degrees and
//!   decimal minutes, and you can read it off a packet by eye.
//! - **Compressed** packs the same thing into thirteen characters of base 91,
//!   which is smaller on air and unreadable to a person.
//! - **Mic-E** is the strange one. It splits the position between the
//!   information field and the *destination callsign*, abusing an address
//!   field to carry latitude digits, because that costs no extra bytes in a
//!   frame that has to have a destination anyway. It is what most vehicle
//!   trackers transmit, so a decoder without it misses most of what moves.
//!
//! Everything here is a pure function of bytes. Nothing knows about radios.

/// A position, however it was encoded.
#[derive(Clone, Debug, PartialEq)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    /// Course over ground in degrees true, when reported.
    pub course_deg: Option<f64>,
    /// Speed over ground in knots, when reported.
    pub speed_kt: Option<f64>,
    pub altitude_ft: Option<i32>,
    /// The two characters selecting the icon a map should draw: a table
    /// selector and a code. `/>` is a car, `/-` a house, `/O` a balloon.
    pub symbol_table: char,
    pub symbol_code: char,
}

impl Default for Position {
    fn default() -> Self {
        Self {
            lat: 0.0,
            lon: 0.0,
            course_deg: None,
            speed_kt: None,
            altitude_ft: None,
            symbol_table: '/',
            symbol_code: '.',
        }
    }
}

/// What a payload turned out to be.
#[derive(Clone, Debug, PartialEq)]
pub enum Report {
    Position { position: Position, comment: Option<String> },
    Status(String),
    Message { to: String, text: String },
    /// A type this decoder does not read, named by its data type identifier
    /// so the log still counts it.
    Other(char),
}

/// Parse an information field.
///
/// `destination` is the AX.25 destination callsign, needed because Mic-E
/// hides half its latitude in there. Everything else ignores it.
pub fn parse(info: &[u8], destination: &str) -> Option<Report> {
    let kind = *info.first()? as char;
    match kind {
        // Mic-E, current and old prototypes.
        '`' | '\'' | '\x1c' | '\x1d' => mic_e(info, destination),
        '!' | '=' | '/' | '@' => position(info, kind),
        '>' => Some(Report::Status(text(&info[1..]))),
        ':' => message(info),
        _ => Some(Report::Other(kind)),
    }
}

fn text(b: &[u8]) -> String {
    b.iter()
        .map(|&c| if (32..127).contains(&c) { c as char } else { '.' })
        .collect::<String>()
        .trim_end()
        .to_string()
}

/// `:ADDRESSEE:text`
fn message(info: &[u8]) -> Option<Report> {
    // Nine characters of addressee, then a colon.
    if info.len() < 11 || info[10] != b':' {
        return None;
    }
    Some(Report::Message {
        to: text(&info[1..10]),
        text: text(&info[11..]),
    })
}

/// A position report, compressed or not, with or without a timestamp.
fn position(info: &[u8], kind: char) -> Option<Report> {
    // The timestamped forms put seven characters of time first, which nothing
    // here reads: the packet's own arrival time is better evidence than a
    // clock somebody else set.
    let body = if kind == '/' || kind == '@' { info.get(8..)? } else { info.get(1..)? };
    // A digit here is degrees, so the report is uncompressed. Anything else
    // is a compressed symbol table selector.
    if body.first()?.is_ascii_digit() {
        uncompressed(body)
    } else {
        compressed(body)
    }
}

/// `4903.50N/07201.75W-comment`
fn uncompressed(b: &[u8]) -> Option<Report> {
    if b.len() < 19 {
        return None;
    }
    let s = std::str::from_utf8(b.get(..19)?).ok()?;
    let lat = degrees_minutes(&s[0..8], 2)?;
    let lat = match s.as_bytes()[7] {
        b'N' => lat,
        b'S' => -lat,
        _ => return None,
    };
    let symbol_table = s.as_bytes()[8] as char;
    let lon = degrees_minutes(&s[9..18], 3)?;
    let lon = match s.as_bytes()[17] {
        b'E' => lon,
        b'W' => -lon,
        _ => return None,
    };
    let symbol_code = s.as_bytes()[18] as char;

    let rest = &b[19..];
    let mut pos =
        Position { lat, lon, symbol_table, symbol_code, ..Position::default() };
    // `nnn/nnn` immediately after the symbol is course and speed in knots.
    let comment_at = if rest.len() >= 7 && rest[3] == b'/' {
        let cs = std::str::from_utf8(&rest[..7]).ok()?;
        if let (Ok(c), Ok(sp)) = (cs[0..3].parse::<f64>(), cs[4..7].parse::<f64>()) {
            pos.course_deg = (c > 0.0).then_some(c);
            pos.speed_kt = Some(sp);
        }
        7
    } else {
        0
    };
    let comment = text(&rest[comment_at.min(rest.len())..]);
    pos.altitude_ft = altitude(&comment);
    Some(Report::Position { position: pos, comment: (!comment.is_empty()).then_some(comment) })
}

/// `DDMM.hh` with `deg` leading degree digits, followed by the hemisphere.
fn degrees_minutes(s: &str, deg: usize) -> Option<f64> {
    let d: f64 = s.get(..deg)?.trim().parse().ok()?;
    // Ambiguity is expressed by replacing minute digits with spaces, which
    // parse as the middle of the remaining range closely enough.
    let m: f64 = s.get(deg..deg + 5)?.replace(' ', "0").parse().ok()?;
    Some(d + m / 60.0)
}

/// Thirteen characters of base 91: `/YYYYXXXX$cs T`.
fn compressed(b: &[u8]) -> Option<Report> {
    if b.len() < 13 {
        return None;
    }
    let symbol_table = b[0] as char;
    let y = base91(&b[1..5])?;
    let x = base91(&b[5..9])?;
    let symbol_code = b[9] as char;

    let mut pos = Position {
        // The two scale factors are fixed by the format.
        lat: 90.0 - y / 380_926.0,
        lon: -180.0 + x / 190_463.0,
        symbol_table,
        symbol_code,
        ..Position::default()
    };
    if !(-90.0..=90.0).contains(&pos.lat) || !(-180.0..=180.0).contains(&pos.lon) {
        return None;
    }
    // The two characters after the symbol are course and speed, altitude, or
    // nothing, depending on the compression type byte.
    let (c, s, t) = (b[10], b[11], b[12]);
    if c != b' ' {
        if (t >> 3) & 0b11 == 0b10 {
            // Altitude, as a base 91 power of 1.002.
            let e = (c - 33) as f64 * 91.0 + (s - 33) as f64;
            pos.altitude_ft = Some(1.002f64.powf(e) as i32);
        } else {
            pos.course_deg = Some((c - 33) as f64 * 4.0).filter(|v| *v > 0.0);
            pos.speed_kt = Some(1.08f64.powf((s - 33) as f64) - 1.0);
        }
    }
    let comment = text(&b[13..]);
    Some(Report::Position { position: pos, comment: (!comment.is_empty()).then_some(comment) })
}

fn base91(b: &[u8]) -> Option<f64> {
    let mut v = 0.0;
    for &c in b {
        if !(33..=124).contains(&c) {
            return None;
        }
        v = v * 91.0 + (c - 33) as f64;
    }
    Some(v)
}

/// Mic-E, whose latitude lives in the destination callsign.
///
/// Each destination character carries a latitude digit plus one bit of
/// something else, encoded by which of three character ranges it falls in:
/// `0-9` is a digit with the bit clear, `A-J` is a digit with the bit set and
/// the position ambiguous, `P-Y` is a digit with the bit set. The bits from
/// characters four, five and six are the north/south flag, a hundred degree
/// longitude offset, and the east/west flag.
fn mic_e(info: &[u8], destination: &str) -> Option<Report> {
    let d = destination.as_bytes();
    if d.len() < 6 || info.len() < 9 {
        return None;
    }
    let mut digits = [0u8; 6];
    let mut bits = [false; 6];
    for i in 0..6 {
        let c = d[i];
        let (v, b) = match c {
            b'0'..=b'9' => (c - b'0', false),
            b'A'..=b'J' => (c - b'A', true),
            b'P'..=b'Y' => (c - b'P', true),
            // Ambiguity characters: the digit is unknown, the bit is set for
            // the upper one.
            b'K' | b'L' => (0, c == b'L'),
            b'Z' => (0, true),
            _ => return None,
        };
        digits[i] = v;
        bits[i] = b;
    }

    let lat = f64::from(digits[0] * 10 + digits[1])
        + (f64::from(digits[2] * 10 + digits[3])
            + f64::from(digits[4] * 10 + digits[5]) / 100.0)
            / 60.0;
    let lat = if bits[3] { lat } else { -lat };

    // Longitude degrees come from the information field, offset by the bit
    // the destination carried.
    let mut deg = i32::from(info[1]) - 28;
    if bits[4] {
        deg += 100;
    }
    if (180..=189).contains(&deg) {
        deg -= 80;
    } else if (190..=199).contains(&deg) {
        deg -= 190;
    }
    let mut min = i32::from(info[2]) - 28;
    if min >= 60 {
        min -= 60;
    }
    let hun = i32::from(info[3]) - 28;
    let lon = f64::from(deg) + (f64::from(min) + f64::from(hun) / 100.0) / 60.0;
    // West is the set bit, and a west longitude is negative.
    let lon = if bits[5] { -lon } else { lon };
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return None;
    }

    // Speed and course are split across three bytes, which is the other
    // reason Mic-E is awkward: the middle byte carries the units digit of the
    // speed and the hundreds digit of the course at once.
    let sp = i32::from(info[4]) - 28;
    let dc = i32::from(info[5]) - 28;
    let se = i32::from(info[6]) - 28;
    let mut speed = sp * 10 + dc / 10;
    if speed >= 800 {
        speed -= 800;
    }
    let mut course = (dc % 10) * 100 + se;
    if course >= 400 {
        course -= 400;
    }

    let comment = text(&info[9..]);
    let pos = Position {
        lat,
        lon,
        speed_kt: Some(f64::from(speed)),
        course_deg: (course > 0).then(|| f64::from(course)),
        altitude_ft: altitude(&comment),
        symbol_code: info[7] as char,
        symbol_table: info[8] as char,
    };
    Some(Report::Position { position: pos, comment: (!comment.is_empty()).then_some(comment) })
}

/// `/A=001234` anywhere in a comment is altitude in feet.
fn altitude(comment: &str) -> Option<i32> {
    let at = comment.find("/A=")?;
    comment.get(at + 3..at + 9)?.trim().parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pos(r: &Report) -> &Position {
        match r {
            Report::Position { position, .. } => position,
            other => panic!("not a position: {other:?}"),
        }
    }

    /// The worked example from the APRS specification.
    #[test]
    fn an_uncompressed_position_decodes_to_the_published_coordinates() {
        let r = parse(b"!4903.50N/07201.75W-Test 001234", "APRS").unwrap();
        let p = pos(&r);
        // 49 degrees 3.50 minutes north, 72 degrees 1.75 minutes west.
        assert!((p.lat - 49.058_333).abs() < 1e-5, "latitude {}", p.lat);
        assert!((p.lon - -72.029_166).abs() < 1e-5, "longitude {}", p.lon);
        assert_eq!(p.symbol_table, '/');
        assert_eq!(p.symbol_code, '-', "a house");
    }

    /// Course and speed sit immediately after the symbol, and must not be
    /// swallowed into the comment.
    #[test]
    fn course_and_speed_are_read_when_present() {
        let r = parse(b"!4903.50N/07201.75W>088/036heading out", "APRS").unwrap();
        let p = pos(&r);
        assert_eq!(p.course_deg, Some(88.0));
        assert_eq!(p.speed_kt, Some(36.0));
        let Report::Position { comment, .. } = &r else { panic!() };
        assert_eq!(comment.as_deref(), Some("heading out"));
    }

    /// The other worked example from the specification, which is the same
    /// place written the compact way.
    #[test]
    fn a_compressed_position_decodes_to_the_published_coordinates() {
        let r = parse(b"!/5L!!<*e7> sT", "APRS").unwrap();
        let p = pos(&r);
        assert!((p.lat - 49.5).abs() < 1e-4, "latitude {}", p.lat);
        assert!((p.lon - -72.75).abs() < 1e-4, "longitude {}", p.lon);
        assert_eq!(p.symbol_code, '>', "a car");
    }

    /// Encode a Mic-E packet the way a transmitter does.
    ///
    /// A round trip rather than a published example, and that is weaker: it
    /// shares its assumptions with the decoder, so it proves the two halves
    /// agree rather than that either matches the standard. It is here because
    /// the destination encoding is the part with real traps in it (the
    /// hemisphere bits, the hundred degree offset, and the byte that carries
    /// half the speed and half the course at once), and a round trip does
    /// catch getting any of those inconsistent. The uncompressed and
    /// compressed formats above are checked against the specification's own
    /// worked examples, which is the stronger evidence.
    fn encode_mic_e(lat: f64, lon: f64, speed: i32, course: i32) -> (String, Vec<u8>) {
        let north = lat >= 0.0;
        let west = lon < 0.0;
        let (alat, alon) = (lat.abs(), lon.abs());
        let deg = alat.trunc() as i32;
        let minutes = (alat - alat.trunc()) * 60.0;
        let min = minutes.trunc() as i32;
        let hun = ((minutes - minutes.trunc()) * 100.0).round() as i32;
        let digits = [deg / 10, deg % 10, min / 10, min % 10, hun / 10, hun % 10];

        let lon_deg = alon.trunc() as i32;
        // Which encoding the degrees take decides the offset bit, including
        // the case that looks backwards: under ten degrees needs the offset
        // set, because the decoder's 190..199 remapping is what brings it
        // back down.
        let (lon_byte, offset) = match lon_deg {
            0..=9 => (lon_deg + 118, true),
            10..=99 => (lon_deg + 28, false),
            100..=109 => (lon_deg + 8, true),
            _ => (lon_deg - 72, true),
        };
        let bits = [false, false, false, north, offset, west];
        let dest: String = digits
            .iter()
            .zip(bits)
            .map(|(d, b)| if b { (b'P' + *d as u8) as char } else { (b'0' + *d as u8) as char })
            .collect();

        let lon_min = minutes_of(alon);
        let lon_hun = hundredths_of(alon);
        let mut info = vec![b'`'];
        info.push(lon_byte as u8);
        info.push(if lon_min < 10 { lon_min + 88 } else { lon_min + 28 } as u8);
        info.push((lon_hun + 28) as u8);
        info.push((speed / 10 + 28) as u8);
        info.push(((speed % 10) * 10 + course / 100 + 28) as u8);
        info.push((course % 100 + 28) as u8);
        info.push(b'>');
        info.push(b'/');
        (dest, info)
    }

    fn minutes_of(v: f64) -> i32 {
        ((v - v.trunc()) * 60.0).trunc() as i32
    }

    fn hundredths_of(v: f64) -> i32 {
        let m = (v - v.trunc()) * 60.0;
        ((m - m.trunc()) * 100.0).round() as i32
    }

    /// Mic-E splits the position between the payload and the destination
    /// callsign, so a decoder that ignores the destination puts the station in
    /// the wrong hemisphere or on the wrong continent.
    #[test]
    fn mic_e_round_trips_through_the_destination_callsign() {
        // Both hemispheres, both sides of the meridian, and longitudes either
        // side of the hundred degree offset that the encoding special cases.
        for (lat, lon, speed, course) in [
            (53.35, -6.26, 0, 0),
            (33.427, -112.129, 20, 251),
            (-33.86, 151.21, 45, 90),
            (51.5, 0.12, 5, 359),
            (-1.5, -5.0, 60, 180),
        ] {
            let (dest, info) = encode_mic_e(lat, lon, speed, course);
            let r = parse(&info, &dest).unwrap_or_else(|| panic!("{lat},{lon} did not parse"));
            let p = pos(&r);
            assert!((p.lat - lat).abs() < 0.001, "latitude {} wanted {lat}", p.lat);
            assert!((p.lon - lon).abs() < 0.001, "longitude {} wanted {lon}", p.lon);
            assert_eq!(p.speed_kt, Some(f64::from(speed)), "speed for {lat},{lon}");
            if course > 0 {
                assert_eq!(p.course_deg, Some(f64::from(course)), "course for {lat},{lon}");
            }
        }
    }

    /// The one bit that decides which half of the planet a station is on.
    #[test]
    fn the_destination_decides_the_hemisphere() {
        let (dest, info) = encode_mic_e(53.35, -6.26, 0, 0);
        let north = parse(&info, &dest).unwrap();
        assert!(pos(&north).lat > 0.0);
        // Clear the north bit by moving that character out of the upper range.
        let mut south: Vec<char> = dest.chars().collect();
        south[3] = (south[3] as u8 - b'P' + b'0') as char;
        let south: String = south.into_iter().collect();
        assert!(pos(&parse(&info, &south).unwrap()).lat < 0.0, "the bit was ignored");
    }

    #[test]
    fn altitude_is_read_out_of_a_comment() {
        let r = parse(b"!4903.50N/07201.75W-/A=001234", "APRS").unwrap();
        assert_eq!(pos(&r).altitude_ft, Some(1234));
    }

    #[test]
    fn a_status_and_a_message_are_not_positions() {
        assert_eq!(parse(b">on the air", "APRS"), Some(Report::Status("on the air".into())));
        let m = parse(b":EI2ABC   :hello", "APRS").unwrap();
        assert_eq!(m, Report::Message { to: "EI2ABC".into(), text: "hello".into() });
    }

    /// Nothing here may panic on a truncated payload.
    #[test]
    fn short_and_malformed_payloads_are_refused() {
        assert_eq!(parse(b"", "APRS"), None);
        assert!(parse(b"!490", "APRS").is_none());
        assert!(parse(b"!/5L", "APRS").is_none());
        assert!(parse(b"`(_", "S32UAT").is_none());
        // A destination that is not Mic-E at all.
        assert!(parse(b"`(_fn\"Oj/", "!!!!!!").is_none());
        // An unknown data type is reported rather than dropped.
        assert_eq!(parse(b"{xyz", "APRS"), Some(Report::Other('{')));
    }
}
