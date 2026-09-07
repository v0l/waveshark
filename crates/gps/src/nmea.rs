//! NMEA 0183, the two sentences a position comes from.
//!
//! A GPS emits a dozen sentence types and a survey needs two of them. `RMC`
//! is the recommended minimum: time, date, validity, position, speed and
//! track, which is everything except quality. `GGA` adds the quality:
//! satellites, horizontal dilution and altitude. Neither is a superset of the
//! other, so a fix is assembled from both as they arrive rather than taken
//! from whichever came last.
//!
//! Everything else is ignored by design. `GSV` lists satellites in view,
//! `GSA` the ones used, `VTG` repeats the course; none of them says where the
//! receiver is, and parsing them would be parsing for its own sake.
//!
//! # The talker prefix is not the constellation
//!
//! A sentence begins `$GPRMC`, `$GNRMC`, `$GLRMC` or `$GARMC` depending on
//! which constellations the receiver used, and a multi-constellation module
//! switches between them mid-session as satellites come and go. So the two
//! characters after the dollar are skipped rather than matched: a receiver
//! that only accepts `$GP` sentences goes blind the moment the module locks
//! onto Galileo as well.

/// A position from the GPS, as complete as the sentences that built it.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Fix {
    pub lat: f64,
    pub lon: f64,
    /// Metres above mean sea level, where the receiver reported it.
    pub alt_m: Option<f64>,
    /// Ground speed in metres per second.
    pub speed_ms: Option<f64>,
    /// Course over ground in degrees true.
    pub track_deg: Option<f64>,
    /// Satellites used in the solution.
    pub sats: Option<u8>,
    /// Horizontal dilution of precision: how much the satellite geometry
    /// multiplies the ranging error. Under 2 is good, over 5 is a fix worth
    /// distrusting.
    pub hdop: Option<f64>,
    /// UTC seconds since the epoch, from the sentence rather than from this
    /// machine's clock. A receiver logging a survey may have no other source
    /// of true time, and a laptop that came back from suspend has the wrong
    /// one.
    pub utc: Option<u64>,
}

/// What a line turned out to be.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sentence {
    /// Position and movement, valid flag included.
    Rmc(Fix),
    /// Position and fix quality.
    Gga(Fix),
    /// A sentence this does not read, or one whose fields were empty.
    Other,
}

/// Parse one NMEA line.
///
/// `None` when the line is not NMEA at all or its checksum fails. The
/// checksum is cheap and a serial line that has lost a byte produces
/// sentences that parse perfectly into the wrong place, so it is checked
/// rather than trusted: a single flipped bit in a longitude field is half a
/// degree, which is thirty miles of survey.
pub fn parse_sentence(line: &str) -> Option<Sentence> {
    let line = line.trim();
    let body = line.strip_prefix('$').or_else(|| line.strip_prefix('!'))?;
    let (body, checksum) = match body.split_once('*') {
        Some((b, c)) => (b, Some(c)),
        None => (body, None),
    };
    if let Some(want) = checksum.and_then(|c| u8::from_str_radix(c.trim(), 16).ok()) {
        let got = body.bytes().fold(0u8, |a, b| a ^ b);
        if got != want {
            return None;
        }
    }
    let mut f = body.split(',');
    let kind = f.next()?;
    // Skip the talker: a multi-constellation module changes it mid-session.
    let kind = kind.get(2..)?;
    let fields: Vec<&str> = f.collect();
    match kind {
        "RMC" => Some(rmc(&fields)),
        "GGA" => Some(gga(&fields)),
        _ => Some(Sentence::Other),
    }
}

/// `RMC`: time, validity, position, speed, track, date.
fn rmc(f: &[&str]) -> Sentence {
    // Field 1 is `A` for a valid fix and `V` for a warning, which means the
    // receiver is reporting its last idea of where it was rather than where
    // it is. Both come with plausible coordinates.
    if f.len() < 9 || f[1] != "A" {
        return Sentence::Other;
    }
    let (Some(lat), Some(lon)) = (coord(f[2], f[3]), coord(f[4], f[5])) else {
        return Sentence::Other;
    };
    Sentence::Rmc(Fix {
        lat,
        lon,
        // Knots on the wire, everywhere else in this project metres a second.
        speed_ms: f[6].parse::<f64>().ok().map(|kt| kt * 0.514_444),
        track_deg: f[7].parse().ok(),
        utc: utc(f[8], f[0]),
        ..Default::default()
    })
}

/// `GGA`: time, position, fix quality, satellites, HDOP, altitude.
fn gga(f: &[&str]) -> Sentence {
    // Quality 0 is no fix. 1 is GPS, 2 differential, and 6 is dead reckoning,
    // which is the receiver guessing from its last velocity: plausible,
    // moving, and not a measurement.
    if f.len() < 9 || !matches!(f[5], "1" | "2" | "4" | "5") {
        return Sentence::Other;
    }
    let (Some(lat), Some(lon)) = (coord(f[1], f[2]), coord(f[3], f[4])) else {
        return Sentence::Other;
    };
    Sentence::Gga(Fix {
        lat,
        lon,
        alt_m: f[8].parse().ok(),
        sats: f[6].parse().ok(),
        hdop: f[7].parse().ok(),
        ..Default::default()
    })
}

/// NMEA writes a coordinate as degrees and minutes run together, `4807.038`
/// meaning 48 degrees and 7.038 minutes, with the hemisphere in its own
/// field. Degrees are two digits for latitude and three for longitude, so the
/// split is by length rather than by a fixed offset.
fn coord(value: &str, hemisphere: &str) -> Option<f64> {
    if value.is_empty() {
        return None;
    }
    let dot = value.find('.').unwrap_or(value.len());
    let split = dot.checked_sub(2)?;
    let deg: f64 = value.get(..split)?.parse().ok()?;
    let min: f64 = value.get(split..)?.parse().ok()?;
    let v = deg + min / 60.0;
    match hemisphere {
        "N" | "E" => Some(v),
        "S" | "W" => Some(-v),
        _ => None,
    }
}

/// `hhmmss.sss` and `ddmmyy` into seconds since the epoch.
fn utc(date: &str, time: &str) -> Option<u64> {
    if date.len() < 6 || time.len() < 6 {
        return None;
    }
    // Two-digit years, with no century anywhere in NMEA 0183. Pivoted at 80
    // the way every other reader of two-digit years does, so the sentences in
    // the standard's own examples read as the 1990s they were written in and
    // a receiver on the air today reads as this century.
    let yy: u64 = date.get(4..6)?.parse().ok()?;
    utc_from_parts(
        if yy < 80 { 2000 + yy } else { 1900 + yy },
        date.get(2..4)?.parse().ok()?,
        date.get(0..2)?.parse().ok()?,
        time.get(0..2)?.parse().ok()?,
        time.get(2..4)?.parse().ok()?,
        time.get(4..6)?.parse().ok()?,
    )
}

/// A civil date and time in UTC as seconds since the epoch.
///
/// Written out rather than taken from a date library, because the whole of
/// the problem is six integers and a leap year rule, and this crate has no
/// other reason to have one. Shared with the gpsd path, whose timestamps are
/// ISO 8601 and the same six numbers.
pub fn utc_from_parts(
    year: u64,
    month: u64,
    day: u64,
    hour: u64,
    minute: u64,
    second: u64,
) -> Option<u64> {
    if !(1..=12).contains(&month) || day == 0 || year < 1970 {
        return None;
    }
    let leap = |y: u64| y.is_multiple_of(4) && (!y.is_multiple_of(100) || y.is_multiple_of(400));
    let mut days = 0u64;
    for y in 1970..year {
        days += if leap(y) { 366 } else { 365 };
    }
    const LENGTHS: [u64; 12] = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    for m in 1..month {
        days += LENGTHS[(m - 1) as usize] + u64::from(m == 2 && leap(year));
    }
    days += day - 1;
    Some(((days * 24 + hour) * 60 + minute) * 60 + second)
}

/// Assemble fixes from the sentences as they arrive.
///
/// `RMC` and `GGA` each carry half of what a survey wants and arrive in
/// either order, once a second. So the two are merged and a fix is only
/// published on the sentence that carries the position, which means one fix a
/// second rather than two half fixes.
#[derive(Default)]
pub struct Assembler {
    quality: Fix,
}

impl Assembler {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed one line. Returns a fix when the line completed one.
    pub fn push(&mut self, line: &str) -> Option<Fix> {
        match parse_sentence(line)? {
            Sentence::Gga(f) => {
                self.quality = f;
                Some(f)
            }
            Sentence::Rmc(mut f) => {
                f.alt_m = self.quality.alt_m;
                f.sats = self.quality.sats;
                f.hdop = self.quality.hdop;
                Some(f)
            }
            Sentence::Other => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real line from a u-blox module, checksum included.
    const RMC: &str = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A";
    const GGA: &str = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

    #[test]
    fn an_rmc_sentence_gives_a_position_and_a_speed() {
        let Some(Sentence::Rmc(f)) = parse_sentence(RMC) else { panic!("not parsed") };
        assert!((f.lat - 48.117_3).abs() < 1e-4, "lat {}", f.lat);
        assert!((f.lon - 11.516_6).abs() < 1e-4, "lon {}", f.lon);
        // 22.4 knots.
        assert!((f.speed_ms.unwrap() - 11.52).abs() < 0.01);
        assert_eq!(f.track_deg, Some(84.4));
        // 23 March 1994, 12:35:19 UTC.
        assert_eq!(f.utc, Some(764_426_119));
    }

    #[test]
    fn a_gga_sentence_gives_the_quality() {
        let Some(Sentence::Gga(f)) = parse_sentence(GGA) else { panic!("not parsed") };
        assert_eq!(f.sats, Some(8));
        assert_eq!(f.hdop, Some(0.9));
        assert_eq!(f.alt_m, Some(545.4));
    }

    /// The two halves are merged, so what a consumer gets has both the
    /// position and the reason to believe it.
    #[test]
    fn the_assembler_carries_quality_from_gga_onto_the_next_rmc() {
        let mut a = Assembler::new();
        a.push(GGA).expect("gga is a fix of its own");
        let f = a.push(RMC).expect("rmc completes one");
        assert_eq!(f.sats, Some(8));
        assert_eq!(f.hdop, Some(0.9));
        assert!(f.speed_ms.is_some(), "and keeps what only RMC has");
    }

    /// A receiver with no lock still talks, and its coordinates are zero or
    /// stale. Recording those is how a survey ends up in the Atlantic.
    #[test]
    fn a_sentence_without_a_fix_is_not_a_position() {
        let void = "$GPRMC,123519,V,4807.038,N,01131.000,E,,,230394,,*0A";
        assert_eq!(parse_sentence(void).map(|_| ()), Some(()));
        assert!(!matches!(parse_sentence(void), Some(Sentence::Rmc(_))));
        let nofix = "$GPGGA,123519,,,,,0,00,,,M,,M,,*6B";
        assert!(!matches!(parse_sentence(nofix), Some(Sentence::Gga(_))));
    }

    /// A corrupted line parses into a plausible wrong place, so the checksum
    /// decides rather than the fields.
    #[test]
    fn a_line_with_a_bad_checksum_is_refused() {
        let mut bad = RMC.to_string();
        bad.replace_range(20..21, "9");
        assert!(parse_sentence(&bad).is_none(), "a corrupted sentence was accepted");
    }

    /// The talker prefix changes with the constellation in use, mid-session.
    #[test]
    fn any_talker_is_read() {
        for line in [
            "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*74",
            "$GLRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*76",
        ] {
            assert!(matches!(parse_sentence(line), Some(Sentence::Rmc(_))), "{line}");
        }
    }

    #[test]
    fn the_southern_and_western_hemispheres_are_negative() {
        let s = "$GPRMC,123519,A,3352.000,S,15112.000,E,000.0,000.0,230394,,*03";
        let Some(Sentence::Rmc(f)) = parse_sentence(s) else { panic!("not parsed") };
        assert!(f.lat < 0.0, "Sydney is south: {}", f.lat);
        assert!(f.lon > 0.0, "and east: {}", f.lon);
    }
}
