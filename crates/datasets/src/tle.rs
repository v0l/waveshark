//! Orbital elements: where a satellite will be, as a row of mean elements.
//!
//! General perturbations data as CelesTrak publishes it. The elements are a
//! fit at an epoch, meaningful only through the model they were fitted for,
//! SGP4, and only near that epoch: a set a fortnight old puts a low
//! satellite kilometres from where it is. So what is downloaded here is not
//! a position, it is the input to one, and how old it is matters as much as
//! what it says.
//!
//! # Why this is CSV and not two lines
//!
//! The two-line format has five digits for a catalogue number and the
//! catalogue passed 99999 in July 2026: an object launched since cannot be
//! expressed in it at all, and CelesTrak does not publish one for it. Their
//! usage policy asks software to move to the CSV form, which carries the
//! same fields, is smaller, and has no such limit. `amateur` already holds
//! objects numbered 100000 and up, so this is not a future problem.
//!
//! # Their limits, kept
//!
//! CelesTrak serves hundreds of thousands of addresses a day and asks in
//! writing for three things: query only the documented URLs, download only
//! when the data is going to be used and only once per update, and stop
//! immediately on any answer that is not a 200 rather than retrying into
//! their firewall. GP data updates every two hours; [`MAX_AGE`] is longer
//! than that, nothing here polls, a group is fetched the first time
//! something asks for it, and a refusal halts the dataset until a person
//! presses refresh (`Cache::refresh`, `When::IfDue`).
//!
//! Nothing here propagates anything. This is the file and the numbers in it;
//! turning a set into a position over a place is `crates/orbit`'s job.

use crate::cache::{Cache, Error, Source, When};
use std::time::Duration;

/// Where a group's file comes from and what shape it arrives in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Feed {
    /// A CelesTrak group key, fetched as GP data in CSV.
    CelesTrak(&'static str),
    /// A published file of three-line sets: a name and the two lines under
    /// it. Read only where the publisher offers nothing else, because the
    /// format cannot express a catalogue number past five digits.
    ThreeLine(&'static str),
    /// A Space-Track query, fetched under the operator's own login and
    /// answered in the same three-line form.
    SpaceTrack(&'static str),
}

/// One published group of elements: what it is called and where it lives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Group {
    /// What the group is called, for the row and for a view that groups by
    /// it.
    pub name: &'static str,
    /// Where the file comes from and how it is written.
    pub feed: Feed,
    /// File name under the cache directory, and the metadata key.
    pub file: &'static str,
    /// What is in it, for the pane that offers to download it.
    pub about: &'static str,
    /// Who publishes it, and the page saying on what terms.
    pub publisher: &'static str,
    pub page: &'static str,
    /// The name the publisher asks to be credited by, the licence in brief,
    /// and the terms in a sentence. Drawn wherever the data is: a group is
    /// not always CelesTrak's, and crediting them for somebody else's file
    /// is a credit given to the wrong project.
    pub credit_name: &'static str,
    pub credit_licence: &'static str,
    pub terms: &'static str,
}

impl Group {
    /// The query this group is fetched with. CelesTrak's is the documented
    /// GP one, in the format they ask new software to use; anything else is
    /// a redirect or a 404 by their policy.
    pub fn url(&self) -> String {
        match self.feed {
            Feed::CelesTrak(key) => {
                format!("https://celestrak.org/NORAD/elements/gp.php?GROUP={key}&FORMAT=csv")
            }
            Feed::ThreeLine(url) | Feed::SpaceTrack(url) => url.to_string(),
        }
    }

    pub fn source(&self) -> Source {
        if let Feed::SpaceTrack(url) = self.feed {
            return Source {
                name: self.file,
                from: std::sync::Arc::new(crate::spacetrack::Query { url: url.to_string() }),
                max_age: crate::spacetrack::MAX_AGE,
                check: Some(|head| match three_line_head(head) {
                    true => Ok(()),
                    false => Err("Space-Track did not answer with elements".into()),
                }),
            };
        }
        let src = Source::http(self.file, self.url(), MAX_AGE);
        match self.feed {
            // A refusal, a redirect body or an unknown group comes back as
            // prose or HTML with a 200 in front of it. Storing that as the
            // dataset would replace elements that were fine.
            Feed::CelesTrak(_) => src.checked(|head| match head.starts_with(b"OBJECT_NAME,") {
                true => Ok(()),
                false => Err("CelesTrak did not answer with GP data".into()),
            }),
            _ => src.checked(|head| match three_line_head(head) {
                true => Ok(()),
                false => Err("the answer was not a file of three-line sets".into()),
            }),
        }
    }

    /// The file, as elements.
    pub fn parse(&self, raw: &[u8]) -> Result<Sats, Error> {
        match self.feed {
            Feed::CelesTrak(_) => parse(self.file, raw),
            _ => parse_three_line(self.file, raw),
        }
    }

    /// The credential this group cannot be fetched without, where it has
    /// one. Asked here so a view can say why a row is not offering a
    /// download rather than offering one that fails.
    pub fn needs_login(&self) -> bool {
        matches!(self.feed, Feed::SpaceTrack(_))
    }
}

/// Whether the head of a file looks like a name and a first line under it.
///
/// The name is `0 ISS (ZARYA)` in Space-Track's 3LE and `ISS (ZARYA)` in
/// CelesTrak's TLE, which is the whole difference between the two.
fn three_line_head(head: &[u8]) -> bool {
    let text = String::from_utf8_lossy(head);
    let mut lines = text.lines();
    let named = lines.next().is_some_and(|l| !l.trim().is_empty() && !l.starts_with('<'));
    named && lines.next().is_some_and(|l| l.starts_with("1 "))
}

/// How long a copy is treated as current.
///
/// CelesTrak rebuilds GP data every two hours and asks for one download per
/// update. Six is inside that and is as often as elements are worth
/// refetching for a receiver: a set six hours old moves a low satellite by
/// under a kilometre.
const MAX_AGE: Duration = Duration::from_secs(6 * 3600);

const CELESTRAK: &str = "celestrak.org";
const CELESTRAK_PAGE: &str = "https://celestrak.org/NORAD/elements/";
const CELESTRAK_TERMS: &str = "CelesTrak, credit Dr. T.S. Kelso and link celestrak.org";

pub static AMATEUR: Group = Group {
    name: "Amateur",
    feed: Feed::CelesTrak("amateur"),
    file: "celestrak-amateur.csv",
    about: "Amateur radio satellites: the linear transponders, the FM \
            repeaters and the packet digipeaters, which is the group this \
            receiver can actually hear.",
    publisher: CELESTRAK,
    page: CELESTRAK_PAGE,
    credit_name: "CelesTrak",
    credit_licence: "Dr. T.S. Kelso",
    terms: CELESTRAK_TERMS,
};

pub static WEATHER: Group = Group {
    name: "Weather",
    feed: Feed::CelesTrak("weather"),
    file: "celestrak-weather.csv",
    about: "Polar weather satellites, including the NOAA APT birds around \
            137 MHz and the Meteor LRPT ones beside them.",
    publisher: CELESTRAK,
    page: CELESTRAK_PAGE,
    credit_name: "CelesTrak",
    credit_licence: "Dr. T.S. Kelso",
    terms: CELESTRAK_TERMS,
};

pub static CUBESATS: Group = Group {
    name: "CubeSats",
    feed: Feed::CelesTrak("cubesat"),
    file: "celestrak-cubesat.csv",
    about: "CubeSats, which is where most new amateur and university \
            beacons appear before they are catalogued anywhere else.",
    publisher: CELESTRAK,
    page: CELESTRAK_PAGE,
    credit_name: "CelesTrak",
    credit_licence: "Dr. T.S. Kelso",
    terms: CELESTRAK_TERMS,
};

pub static STATIONS: Group = Group {
    name: "Space stations",
    feed: Feed::CelesTrak("stations"),
    file: "celestrak-stations.csv",
    about: "The ISS and the other crewed stations, which carry voice \
            repeaters and SSTV.",
    publisher: CELESTRAK,
    page: CELESTRAK_PAGE,
    credit_name: "CelesTrak",
    credit_licence: "Dr. T.S. Kelso",
    terms: CELESTRAK_TERMS,
};

pub static GNSS: Group = Group {
    name: "GNSS",
    feed: Feed::CelesTrak("gnss"),
    file: "celestrak-gnss.csv",
    about: "GPS, Galileo, GLONASS and BeiDou, for judging what a receiver's \
            own fix has to work with.",
    publisher: CELESTRAK,
    page: CELESTRAK_PAGE,
    credit_name: "CelesTrak",
    credit_licence: "Dr. T.S. Kelso",
    terms: CELESTRAK_TERMS,
};

/// The satellites the TinyGS network listens to.
///
/// A list rather than a category: these are the ones somebody is decoding
/// today, mostly LoRa and FSK cubesats at 400 and 900 MHz, and a new one
/// appears here when the network starts tracking it rather than when a
/// catalogue decides what it is. TinyGS publishes the same elements its own
/// stations point by, which is why the pass this quotes is the pass their
/// network is working.
///
/// What it does not carry is the modem settings. Those are on an endpoint
/// that answers browsers and not programs, so the spreading factor is still
/// found by trying (`crates/nodes/src/lora_nodes.rs`).
pub static TINYGS: Group = Group {
    name: "TinyGS",
    feed: Feed::ThreeLine("https://api.tinygs.com/v1/tinygs_supported.txt"),
    file: "tinygs-supported.txt",
    about: "The satellites the TinyGS network tracks: LoRa and FSK cubesats \
            around 400 and 900 MHz, which is where a receiver with a LoRa \
            decoder has something to hear.",
    publisher: "tinygs.com",
    page: "https://tinygs.com/",
    credit_name: "TinyGS",
    credit_licence: "open network",
    terms: "TinyGS, credit the network and link tinygs.com",
};

/// Everything in orbit whose elements were fitted in the last ten days.
///
/// The catalogue rather than a group of it, which is what Space-Track is
/// for: an object launched last week is here and is in no group anywhere
/// until somebody classifies it, and a beacon is worth hearing well before
/// that. The epoch bound is what keeps it usable, since a set older than
/// that puts a low satellite kilometres from where it is, and `DECAY_DATE
/// null-val` drops everything that has already come down.
///
/// About 30 MB, and it needs the operator's own login.
pub static SPACE_TRACK: Group = Group {
    name: "Space-Track",
    feed: Feed::SpaceTrack(
        "https://www.space-track.org/basicspacedata/query/class/gp/\
         EPOCH/%3Enow-10/DECAY_DATE/null-val/format/3le",
    ),
    file: "spacetrack-gp.3le",
    about: "Every catalogued object still in orbit whose elements were \
            fitted in the last ten days, from Space-Track itself. Needs a \
            free account of your own, and it is the only source here that \
            has a satellite before somebody has decided what group it \
            belongs to.",
    publisher: "space-track.org",
    page: "https://www.space-track.org/",
    credit_name: "Space-Track",
    credit_licence: "US Space Force, under their user agreement",
    terms: "Space-Track user agreement: your own account, no redistribution",
};

/// Every group that is offered, in the order a view lists them.
pub static GROUPS: &[&Group] =
    &[&AMATEUR, &WEATHER, &CUBESATS, &STATIONS, &GNSS, &TINYGS, &SPACE_TRACK];

pub fn group(name: &str) -> Option<&'static Group> {
    GROUPS.iter().copied().find(|g| g.name.eq_ignore_ascii_case(name))
}

/// One object's elements.
///
/// The field names are the CCSDS orbit mean-elements message ones, because
/// that is what the columns are called and what every other implementation
/// reading this data calls them. Angles are degrees, mean motion is
/// revolutions a day, and none of it means anything except through SGP4.
#[derive(Clone, Debug, PartialEq)]
pub struct Elements {
    /// `ISS (ZARYA)`, as published.
    pub name: String,
    /// Catalogue number, which is the identity: names are edited and
    /// repeated, numbers are not. Six digits and growing since July 2026.
    pub norad: u64,
    /// International designator, `1998-067A`.
    pub cospar: String,
    /// `U`, `C` or `S`, as published.
    pub classification: char,
    /// When the elements were fitted, as Unix seconds. What decides whether
    /// they are worth using.
    pub epoch_s: i64,
    pub mean_motion: f64,
    pub mean_motion_dot: f64,
    pub mean_motion_ddot: f64,
    pub eccentricity: f64,
    pub inclination_deg: f64,
    pub right_ascension_deg: f64,
    pub argument_of_perigee_deg: f64,
    pub mean_anomaly_deg: f64,
    /// Radiation pressure coefficient, in inverse earth radii.
    pub drag_term: f64,
    pub ephemeris_type: u8,
    pub element_set_number: u64,
    pub revolution_number: u64,
}

impl Elements {
    /// How old the fit is, in days, at a given time. Negative for a set
    /// published ahead of now, which happens: a fit can be propagated
    /// forward of the last observation.
    pub fn age_days(&self, now_s: i64) -> f64 {
        (now_s - self.epoch_s) as f64 / 86_400.0
    }

    /// Roughly how long an orbit takes, in minutes. Enough on its own to
    /// tell a low orbit from a geostationary one.
    pub fn period_min(&self) -> f64 {
        match self.mean_motion > 0.0 {
            true => 1440.0 / self.mean_motion,
            false => 0.0,
        }
    }
}

/// Every set in one group's file, in the order published.
#[derive(Clone, Debug, Default)]
pub struct Sats(Vec<Elements>);

impl Sats {
    /// One object by its catalogue number.
    pub fn get(&self, norad: u64) -> Option<&Elements> {
        self.0.iter().find(|e| e.norad == norad)
    }

    /// The first object whose name contains this, ignoring case. Names are
    /// what an operator reads on a satellite page, so a lookup by name is
    /// what a person will reach for; the catalogue number is what code
    /// should key on.
    pub fn find(&self, name: &str) -> Option<&Elements> {
        let want = name.to_ascii_uppercase();
        self.0.iter().find(|e| e.name.to_ascii_uppercase().contains(&want))
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Elements> {
        self.0.iter()
    }
}

/// One row as the columns are named. Deserialised by header rather than by
/// position: CelesTrak has added columns before and will again.
#[derive(serde::Deserialize)]
#[allow(non_snake_case)]
struct Row {
    OBJECT_NAME: String,
    OBJECT_ID: String,
    EPOCH: String,
    MEAN_MOTION: f64,
    ECCENTRICITY: f64,
    INCLINATION: f64,
    RA_OF_ASC_NODE: f64,
    ARG_OF_PERICENTER: f64,
    MEAN_ANOMALY: f64,
    #[serde(default)]
    EPHEMERIS_TYPE: u8,
    #[serde(default)]
    CLASSIFICATION_TYPE: String,
    NORAD_CAT_ID: u64,
    #[serde(default)]
    ELEMENT_SET_NO: u64,
    #[serde(default)]
    REV_AT_EPOCH: u64,
    #[serde(default)]
    BSTAR: f64,
    #[serde(default)]
    MEAN_MOTION_DOT: f64,
    #[serde(default)]
    MEAN_MOTION_DDOT: f64,
}

pub fn parse(name: &str, raw: &[u8]) -> Result<Sats, Error> {
    let mut rdr = csv::ReaderBuilder::new().flexible(true).from_reader(raw);
    let mut out = Vec::new();
    for row in rdr.deserialize::<Row>() {
        let Ok(r) = row else { continue };
        let Some(epoch_s) = epoch(&r.EPOCH) else {
            continue;
        };
        out.push(Elements {
            name: r.OBJECT_NAME.trim().to_string(),
            norad: r.NORAD_CAT_ID,
            cospar: r.OBJECT_ID.trim().to_string(),
            classification: r.CLASSIFICATION_TYPE.chars().next().unwrap_or('U'),
            epoch_s,
            mean_motion: r.MEAN_MOTION,
            mean_motion_dot: r.MEAN_MOTION_DOT,
            mean_motion_ddot: r.MEAN_MOTION_DDOT,
            eccentricity: r.ECCENTRICITY,
            inclination_deg: r.INCLINATION,
            right_ascension_deg: r.RA_OF_ASC_NODE,
            argument_of_perigee_deg: r.ARG_OF_PERICENTER,
            mean_anomaly_deg: r.MEAN_ANOMALY,
            drag_term: r.BSTAR,
            ephemeris_type: r.EPHEMERIS_TYPE,
            element_set_number: r.ELEMENT_SET_NO,
            revolution_number: r.REV_AT_EPOCH,
        });
    }
    if out.is_empty() {
        return Err(Error::Parse(name.into(), "no elements in the file".into()));
    }
    Ok(Sats(out))
}

/// `2026-09-07T20:08:10.802688`, which is ISO 8601 in UTC with no zone on
/// it. Parsed here rather than through a date library: the shape is fixed by
/// the format and this crate has no other use for one.
fn epoch(field: &str) -> Option<i64> {
    let (date, time) = field.trim().split_once('T')?;
    let mut d = date.split('-');
    let (y, mo, day): (i64, i64, i64) =
        (d.next()?.parse().ok()?, d.next()?.parse().ok()?, d.next()?.parse().ok()?);
    let mut t = time.split(':');
    let (h, mi): (i64, i64) = (t.next()?.parse().ok()?, t.next()?.parse().ok()?);
    let sec: f64 = t.next()?.parse().ok()?;
    let leap = |y: i64| y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
    let mut days = 0i64;
    for year in 1970..y {
        days += if leap(year) { 366 } else { 365 };
    }
    const LENGTHS: [i64; 12] = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    for m in 0..(mo - 1).clamp(0, 11) {
        days += LENGTHS[m as usize] + i64::from(m == 1 && leap(y));
    }
    days += day - 1;
    Some(days * 86_400 + h * 3600 + mi * 60 + sec as i64)
}

/// A file of three-line sets: a name, then the two lines of a TLE.
///
/// Checked against CelesTrak's CSV for the same object at the same epoch,
/// Norby (46494) on 2026-09-08: every field agrees to the digits the
/// two-line format keeps, which is seven for the eccentricity and five
/// significant for the drag term.
///
/// The columns are fixed by the format and are read by position, because
/// that is the only thing a TLE guarantees: fields run together, a sign can
/// sit where a space would, and splitting on whitespace reads a negative
/// exponent as its own field. A set whose lines are short or whose numbers
/// do not parse is skipped rather than failing the file, so one bad entry
/// does not cost the rest.
pub fn parse_three_line(name: &str, raw: &[u8]) -> Result<Sats, Error> {
    let text = String::from_utf8_lossy(raw);
    let lines: Vec<&str> = text.lines().map(str::trim_end).collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i + 2 < lines.len() {
        let (title, l1, l2) = (lines[i], lines[i + 1], lines[i + 2]);
        if !l1.starts_with("1 ") || !l2.starts_with("2 ") {
            i += 1;
            continue;
        }
        i += 3;
        // Space-Track writes the name as line zero of a 3LE, so it arrives
        // with `0 ` in front of it; CelesTrak's TLE files do not.
        let title = title.trim().strip_prefix("0 ").unwrap_or(title.trim());
        let Some(e) = three_line_set(title.trim(), l1, l2) else {
            continue;
        };
        out.push(e);
    }
    if out.is_empty() {
        return Err(Error::Parse(name.into(), "no elements in the file".into()));
    }
    Ok(Sats(out))
}

/// One set, by column. Columns are one-based in the specification and the
/// slices here are the same fields counted from zero.
fn three_line_set(name: &str, l1: &str, l2: &str) -> Option<Elements> {
    let (a, b) = (l1.as_bytes(), l2.as_bytes());
    if a.len() < 63 || b.len() < 63 {
        return None;
    }
    let f = |line: &str, from: usize, to: usize| -> Option<String> {
        line.get(from..to.min(line.len())).map(|s| s.trim().to_string())
    };
    let num = |line: &str, from: usize, to: usize| -> Option<f64> {
        let s = f(line, from, to)?;
        s.parse().ok()
    };
    Some(Elements {
        name: name.to_string(),
        norad: f(l1, 2, 7)?.parse().ok()?,
        cospar: cospar(&f(l1, 9, 17)?),
        classification: l1.chars().nth(7).unwrap_or('U'),
        epoch_s: tle_epoch(&f(l1, 18, 32)?)?,
        mean_motion: num(l2, 52, 63)?,
        // As published: the first derivative is already halved in this
        // format, and CelesTrak's CSV carries the same halved number under
        // the same name, so the two feeds agree.
        mean_motion_dot: num(l1, 33, 43).unwrap_or(0.0),
        mean_motion_ddot: assumed_decimal(&f(l1, 44, 52)?),
        eccentricity: format!("0.{}", f(l2, 26, 33)?).parse().ok()?,
        inclination_deg: num(l2, 8, 16)?,
        right_ascension_deg: num(l2, 17, 25)?,
        argument_of_perigee_deg: num(l2, 34, 42)?,
        mean_anomaly_deg: num(l2, 43, 51)?,
        drag_term: assumed_decimal(&f(l1, 53, 61)?),
        ephemeris_type: f(l1, 62, 63)?.parse().unwrap_or(0),
        element_set_number: f(l1, 64, 68).and_then(|s| s.parse().ok()).unwrap_or(0),
        revolution_number: f(l2, 63, 68).and_then(|s| s.parse().ok()).unwrap_or(0),
    })
}

/// `20068J` as the international designator everything else writes,
/// `2020-068J`. Two digits of year, so 57 and up are the twentieth century:
/// nothing was launched before Sputnik and the format has no room to say
/// otherwise.
fn cospar(field: &str) -> String {
    let field = field.trim();
    let Some(yy) = field.get(..2).and_then(|s| s.parse::<u32>().ok()) else {
        return field.to_string();
    };
    let year = if yy >= 57 { 1900 + yy } else { 2000 + yy };
    format!("{year}-{}", &field[2..])
}

/// `26251.29878944`: two digits of year, the day of the year, and a
/// fraction of a day.
fn tle_epoch(field: &str) -> Option<i64> {
    let yy: i64 = field.get(..2)?.parse().ok()?;
    let day: f64 = field.get(2..)?.parse().ok()?;
    let year = if yy >= 57 { 1900 + yy } else { 2000 + yy };
    let leap = |y: i64| y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
    let mut days = 0i64;
    for y in 1970..year {
        days += if leap(y) { 366 } else { 365 };
    }
    // Day one is the first of January, so the day number is one-based.
    Some(days * 86_400 + ((day - 1.0) * 86_400.0) as i64)
}

/// `42542-3` is 0.42542e-3, and ` 00000-0` is zero: the format leaves out
/// the decimal point and writes the exponent's sign where a letter would be.
fn assumed_decimal(field: &str) -> f64 {
    let s = field.trim();
    if s.is_empty() {
        return 0.0;
    }
    let (sign, rest) = match s.strip_prefix('-') {
        Some(r) => (-1.0, r),
        None => (1.0, s.strip_prefix('+').unwrap_or(s)),
    };
    let split = rest.rfind(['-', '+']);
    let (mantissa, exp) = match split {
        Some(at) if at > 0 => (&rest[..at], rest[at..].parse::<i32>().unwrap_or(0)),
        _ => (rest, 0),
    };
    let Ok(m) = format!("0.{}", mantissa.trim()).parse::<f64>() else {
        return 0.0;
    };
    sign * m * 10f64.powi(exp)
}

pub fn load(cache: &Cache, g: &'static Group) -> Result<Sats, Error> {
    let src = g.source();
    g.parse(&cache.read(&src)?)
}

pub fn refresh(cache: &Cache, g: &'static Group, when: When) -> Result<Option<Sats>, Error> {
    let src = g.source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    g.parse(&cache.read(&src)?).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rows as CelesTrak publishes them, including an object numbered past
    /// the five digits the two-line format allows: the reason this reads CSV
    /// at all.
    const FILE: &str = "OBJECT_NAME,OBJECT_ID,EPOCH,MEAN_MOTION,ECCENTRICITY,INCLINATION,\
RA_OF_ASC_NODE,ARG_OF_PERICENTER,MEAN_ANOMALY,EPHEMERIS_TYPE,CLASSIFICATION_TYPE,NORAD_CAT_ID,\
ELEMENT_SET_NO,REV_AT_EPOCH,BSTAR,MEAN_MOTION_DOT,MEAN_MOTION_DDOT\n\
ISS (ZARYA),1998-067A,2024-01-15T12:40:33.000000,15.49514029,.0006703,51.6416,247.4627,130.5360,\
325.0288,0,U,25544,999,43134,.30074E-3,.16717E-3,0\n\
OSCAR 7 (AO-7),1974-089B,2026-09-07T20:08:10.802688,12.53699421,.00121035,101.9922,264.9500,\
335.1722,201.7555,0,U,7530,999,37087,.8846638E-4,-.32E-6,0\n\
JAMX01 (JING'AN DREAM STAR),2026-195A,2026-09-07T18:37:33.559104,15.09314296,.0012594,97.5394,\
324.3518,204.8688,155.1935,0,U,100465,999,205,.18875655E-3,.2951E-4,0\n";

    fn sats() -> Sats {
        parse("test", FILE.as_bytes()).expect("three sets")
    }

    #[test]
    fn reads_a_name_a_number_and_the_elements() {
        let s = sats();
        assert_eq!(s.len(), 3);
        let iss = s.get(25544).expect("the station");
        assert_eq!(iss.name, "ISS (ZARYA)");
        assert_eq!(iss.cospar, "1998-067A");
        assert_eq!(iss.classification, 'U');
        assert!((iss.inclination_deg - 51.6416).abs() < 1e-4);
        assert!((iss.mean_motion - 15.49514029).abs() < 1e-8);
        assert!((iss.eccentricity - 0.0006703).abs() < 1e-9);
        assert!((iss.drag_term - 0.00030074).abs() < 1e-9);
        // Fifteen and a half orbits a day is about ninety-three minutes.
        assert!((iss.period_min() - 92.9).abs() < 0.5, "{}", iss.period_min());
    }

    /// The catalogue passed five digits in July 2026. An object numbered
    /// above it has no two-line form at all, which is why this reads CSV.
    #[test]
    fn a_six_digit_catalogue_number_is_an_ordinary_object() {
        assert!(sats().get(100_465).is_some());
        assert_eq!(sats().find("JAMX01").map(|e| e.norad), Some(100_465));
    }

    /// The epoch is what decides whether a set is worth propagating, so it
    /// has to be a real time and not a field copied about.
    #[test]
    fn the_epoch_reads_as_a_time() {
        let iss = sats().get(25544).unwrap().clone();
        // 2024-01-15T12:40:33Z.
        assert_eq!(iss.epoch_s, 1_705_322_433);
        assert!((iss.age_days(iss.epoch_s + 86_400) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn a_file_of_prose_is_an_error_rather_than_an_empty_list() {
        assert!(parse("test", b"Invalid query: unknown group\n").is_err());
        assert!(parse("test", b"OBJECT_NAME,OBJECT_ID,EPOCH\n").is_err());
    }

    #[test]
    fn a_name_is_a_way_in_for_a_person() {
        assert_eq!(sats().find("zarya").map(|e| e.norad), Some(25544));
        assert!(sats().find("voyager").is_none());
    }

    /// Every group has its own cache file, or refreshing one would overwrite
    /// another's and report its count. And every CelesTrak query is the
    /// documented one: they redirect or refuse anything else, and their
    /// policy asks software not to send it.
    #[test]
    fn every_group_asks_the_documented_query_for_a_file_of_its_own() {
        let mut seen: Vec<&str> = Vec::new();
        for g in GROUPS {
            assert!(!seen.contains(&g.file), "{} is used twice", g.file);
            if let Feed::CelesTrak(key) = g.feed {
                assert_eq!(
                    g.url(),
                    format!("https://celestrak.org/NORAD/elements/gp.php?GROUP={key}&FORMAT=csv")
                );
                assert_eq!(g.publisher, CELESTRAK);
            }
            assert!(g.url().starts_with("https://"), "{}", g.url());
            assert!(!g.publisher.is_empty() && g.page.starts_with("https://"));
            seen.push(g.file);
        }
        // Two hours is their update rate; asking oftener is what the policy
        // exists to stop.
        assert!(MAX_AGE >= Duration::from_secs(2 * 3600));
    }

    /// Two sets as TinyGS publishes them, copied from
    /// `api.tinygs.com/v1/tinygs_supported.txt`. The numbers are checked
    /// against the same objects in CelesTrak's CSV, which is what makes this
    /// a test of the parser rather than of itself.
    const SUPPORTED: &str = "Norby\n\
1 46494U 20068J   26251.29878944  .00025403  00000-0  42542-3 0  9994\n\
2 46494  97.8600 274.8030 0003926 324.8190  35.2801 15.51851027329086\n\
FossaSat-2E11\n\
1 58253U 23185C   26251.51119827  .00004452  00000-0  21174-3 0  9992\n\
2 58253  97.4381 305.7297 0010633  36.8891 323.2954 15.20015625 96321\n";

    #[test]
    fn a_three_line_file_reads_as_the_same_elements() {
        let s = parse_three_line("test", SUPPORTED.as_bytes()).expect("two sets");
        assert_eq!(s.len(), 2);
        let n = s.get(46_494).expect("Norby");
        assert_eq!(n.name, "Norby");
        assert_eq!(n.cospar, "2020-068J");
        assert_eq!(n.classification, 'U');
        assert!((n.mean_motion - 15.51851027).abs() < 1e-8, "{}", n.mean_motion);
        assert!((n.eccentricity - 0.0003926).abs() < 1e-9);
        assert!((n.inclination_deg - 97.86).abs() < 1e-4);
        assert!((n.right_ascension_deg - 274.803).abs() < 1e-4);
        assert!((n.argument_of_perigee_deg - 324.819).abs() < 1e-4);
        assert!((n.mean_anomaly_deg - 35.2801).abs() < 1e-4);
        assert!((n.mean_motion_dot - 0.00025403).abs() < 1e-11);
        assert!((n.drag_term - 0.42542e-3).abs() < 1e-12, "{}", n.drag_term);
        assert_eq!(n.mean_motion_ddot, 0.0);
        assert_eq!(n.element_set_number, 999);
        assert_eq!(n.revolution_number, 32908);
        assert_eq!(s.find("fossasat").map(|e| e.norad), Some(58_253));
    }

    /// The epoch is the field a stale set is judged by, so it has to come
    /// out as a time and not as a day number: 26251.29878944 is the 251st
    /// day of 2026, which is 2026-09-08T07:10:15Z.
    #[test]
    fn a_two_digit_year_and_a_day_number_read_as_a_time() {
        let s = parse_three_line("test", SUPPORTED.as_bytes()).unwrap();
        assert_eq!(s.get(46_494).unwrap().epoch_s, 1_788_851_415);
    }

    /// The exponent is written without its `e` and a mantissa without its
    /// point, and a sign can sit where a space would.
    #[test]
    fn an_assumed_decimal_is_a_number() {
        assert_eq!(assumed_decimal(" 00000-0"), 0.0);
        assert!((assumed_decimal(" 42542-3") - 0.42542e-3).abs() < 1e-12);
        assert!((assumed_decimal("-11606-4") + 0.11606e-4).abs() < 1e-12);
        assert!((assumed_decimal(" 12345+2") - 12.345).abs() < 1e-9);
        assert_eq!(assumed_decimal(""), 0.0);
    }

    /// A page of HTML with a 200 in front of it is the failure this guards:
    /// stored as the dataset it would replace elements that were fine.
    #[test]
    fn only_a_file_of_sets_is_accepted() {
        assert!(three_line_head(SUPPORTED.as_bytes()));
        assert!(!three_line_head(b"<!DOCTYPE html>\n<html>\n"));
        assert!(!three_line_head(b"\n1 46494U 20068J\n"));
        assert!(parse_three_line("test", b"nothing here\nor here\n").is_err());
    }
}
