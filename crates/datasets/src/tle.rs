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

/// One published group of elements: what it is called and where it lives.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Group {
    /// What the group is called, for the row and for a view that groups by
    /// it.
    pub name: &'static str,
    /// CelesTrak's own group key, which is what the URL asks for.
    pub key: &'static str,
    /// File name under the cache directory, and the metadata key.
    pub file: &'static str,
    /// What is in it, for the pane that offers to download it.
    pub about: &'static str,
}

impl Group {
    /// The documented GP query, in the format CelesTrak asks new software to
    /// use. Anything else is a redirect or a 404 by their policy.
    pub fn url(&self) -> String {
        format!("https://celestrak.org/NORAD/elements/gp.php?GROUP={}&FORMAT=csv", self.key)
    }

    pub fn source(&self) -> Source {
        Source::http(self.file, self.url(), MAX_AGE).checked(|head| {
            // A refusal, a redirect body or an unknown group comes back as
            // prose or HTML with a 200 in front of it. Storing that as the
            // dataset would replace elements that were fine.
            match head.starts_with(b"OBJECT_NAME,") {
                true => Ok(()),
                false => Err("CelesTrak did not answer with GP data".into()),
            }
        })
    }
}

/// How long a copy is treated as current.
///
/// CelesTrak rebuilds GP data every two hours and asks for one download per
/// update. Six is inside that and is as often as elements are worth
/// refetching for a receiver: a set six hours old moves a low satellite by
/// under a kilometre.
const MAX_AGE: Duration = Duration::from_secs(6 * 3600);

pub static AMATEUR: Group = Group {
    name: "Amateur",
    key: "amateur",
    file: "celestrak-amateur.csv",
    about: "Amateur radio satellites: the linear transponders, the FM \
            repeaters and the packet digipeaters, which is the group this \
            receiver can actually hear.",
};

pub static WEATHER: Group = Group {
    name: "Weather",
    key: "weather",
    file: "celestrak-weather.csv",
    about: "Polar weather satellites, including the NOAA APT birds around \
            137 MHz and the Meteor LRPT ones beside them.",
};

pub static CUBESATS: Group = Group {
    name: "CubeSats",
    key: "cubesat",
    file: "celestrak-cubesat.csv",
    about: "CubeSats, which is where most new amateur and university \
            beacons appear before they are catalogued anywhere else.",
};

pub static STATIONS: Group = Group {
    name: "Space stations",
    key: "stations",
    file: "celestrak-stations.csv",
    about: "The ISS and the other crewed stations, which carry voice \
            repeaters and SSTV.",
};

pub static GNSS: Group = Group {
    name: "GNSS",
    key: "gnss",
    file: "celestrak-gnss.csv",
    about: "GPS, Galileo, GLONASS and BeiDou, for judging what a receiver's \
            own fix has to work with.",
};

/// Every group that is offered, in the order a view lists them.
pub static GROUPS: &[&Group] = &[&AMATEUR, &WEATHER, &CUBESATS, &STATIONS, &GNSS];

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
        let Some(epoch_s) = epoch(&r.EPOCH) else { continue };
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

pub fn load(cache: &Cache, g: &'static Group) -> Result<Sats, Error> {
    let src = g.source();
    parse(src.name, &cache.read(&src)?)
}

pub fn refresh(cache: &Cache, g: &'static Group, when: When) -> Result<Option<Sats>, Error> {
    let src = g.source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse(src.name, &cache.read(&src)?).map(Some)
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
    /// another's and report its count. And every query is the documented
    /// one: CelesTrak redirects or refuses anything else, and their policy
    /// asks software not to send it.
    #[test]
    fn every_group_asks_the_documented_query_for_a_file_of_its_own() {
        let mut seen: Vec<&str> = Vec::new();
        for g in GROUPS {
            assert!(!seen.contains(&g.file), "{} is used twice", g.file);
            assert_eq!(
                g.url(),
                format!("https://celestrak.org/NORAD/elements/gp.php?GROUP={}&FORMAT=csv", g.key)
            );
            seen.push(g.file);
        }
        // Two hours is their update rate; asking oftener is what the policy
        // exists to stop.
        assert!(MAX_AGE >= Duration::from_secs(2 * 3600));
    }
}
