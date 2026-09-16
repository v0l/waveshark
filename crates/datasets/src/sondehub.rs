//! Radiosonde launch sites, from SondeHub.
//!
//! An upper-air station launches a balloon at a published hour, so a sonde
//! is the rare transmitter whose next appearance can be read off a table: a
//! site, the days and times it launches, and which instrument it flies. The
//! frequency is not in there and never is, because the station picks one off
//! the 10 kHz raster on the day, which is why the band is scanned rather
//! than a channel list watched.
//!
//! The list is crowd-sourced against what the SondeHub receivers actually
//! hear, so a site appears with its schedule some time after somebody starts
//! catching its flights, and 120 of the 900 have no schedule at all.

use crate::cache::{Cache, Error, Source, When};
use std::time::Duration;

/// Sites are added and corrected by hand, a few a week at most, so a check a
/// day apart is already far oftener than the file changes.
const MAX_AGE: Duration = Duration::from_secs(24 * 3600);

pub fn source() -> Source {
    Source::http("sondehub-sites.json", "https://api.v2.sondehub.org/sites", MAX_AGE).checked(
        |head| match head.starts_with(b"{") {
            true => Ok(()),
            false => Err("SondeHub did not answer with the launch site list".into()),
        },
    )
}

/// What a station flies, from the WMO radiosonde type in BUFR table 0 02 011.
///
/// The model, not the manufacturer: two codes are often the same instrument
/// registered twice (13 and 14 are both RS92, 23, 24, 41 and 42 are all
/// RS41), and what a listener wants to know is whether the thing in the air
/// is one this receiver can read.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Model {
    Rs41,
    Rs92,
    Rs92Ngp,
    Dfm06,
    Dfm09,
    Dfm17,
    M10,
    M20,
    Ps20,
    IMet1,
    IMet4,
    IMet54,
    Ims100,
    Lms6_403,
    Lms6_1680,
    MrzN1,
    Mrz3mk,
    Mts01,
    Rs11g,
    WxR301d,
    /// A code no decoder here could use: the Soviet and Chinese instruments,
    /// the retired Vaisalas, the rawinsonde targets that carry no
    /// transmitter at all, and the station that filed no type.
    /// [`Sonde::label`] still names most of them.
    Other,
}

/// The codes worth naming that no receiver can read. Names as the SondeHub
/// tracker shows them, with WMO's own wording for the three that mean there
/// was nothing to receive.
const UNREAD: &[(&str, &str)] = &[
    ("0", "no radiosonde"),
    ("1", "passive target"),
    ("2", "active target"),
    ("15", "PAZA-12M"),
    ("16", "PAZA-22"),
    ("20", "MK3"),
    ("21", "1524LA LORAN-C"),
    ("26", "SRS-C34"),
    ("27", "AVK-MRZ"),
    ("28", "AVK-AK2-02"),
    ("29", "MARZ2-2"),
    ("30", "RS2-80"),
    ("33", "GTS1-2/GFE(L)"),
    ("45", "CF-06"),
    ("58", "AVK-BAR"),
    ("59", "M2K2-R"),
    ("68", "AVK-RZM-2"),
    ("69", "MARL-A/Vektor-M-RZM-2"),
    ("73", "MARL-A"),
    ("78", "RS90"),
    ("80", "RS92"),
    ("88", "MARL-A/Vektor-M-MRZ"),
    ("89", "MARL-A/Vektor-M-BAR"),
    ("90", "type not filed"),
    ("97", "iMet-2"),
    ("99", "iMet-2"),
];

impl Model {
    /// The WMO code as SondeHub publishes it, which is a string because the
    /// low ones are written with a leading zero.
    pub fn from_code(code: &str) -> Model {
        match code.trim_start_matches('0') {
            "23" | "24" | "41" | "42" => Model::Rs41,
            "13" | "14" => Model::Rs92,
            "52" => Model::Rs92Ngp,
            "18" => Model::Dfm06,
            "17" => Model::Dfm09,
            "54" => Model::Dfm17,
            "77" => Model::M10,
            "63" => Model::M20,
            "64" => Model::Ps20,
            "7" => Model::IMet1,
            "34" => Model::IMet4,
            "84" => Model::IMet54,
            "35" => Model::Ims100,
            "11" => Model::Lms6_403,
            "82" => Model::Lms6_1680,
            "19" => Model::MrzN1,
            "62" => Model::Mrz3mk,
            "65" => Model::Mts01,
            "22" => Model::Rs11g,
            "38" => Model::WxR301d,
            _ => Model::Other,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Model::Rs41 => "RS41",
            Model::Rs92 => "RS92",
            Model::Rs92Ngp => "RS92-NGP",
            Model::Dfm06 => "DFM-06",
            Model::Dfm09 => "DFM-09",
            Model::Dfm17 => "DFM-17",
            Model::M10 => "M10",
            Model::M20 => "M20",
            Model::Ps20 => "PS-20",
            Model::IMet1 => "iMet-1",
            Model::IMet4 => "iMet-4",
            Model::IMet54 => "iMet-54",
            Model::Ims100 => "iMS-100",
            Model::Lms6_403 => "LMS6-403",
            Model::Lms6_1680 => "LMS6-1680",
            Model::MrzN1 => "MRZ-N1",
            Model::Mrz3mk => "MRZ-3MK",
            Model::Mts01 => "MTS01",
            Model::Rs11g => "RS-11G",
            Model::WxR301d => "WxR-301D",
            Model::Other => "other",
        }
    }

    /// The registry protocol that reads this instrument, where one does.
    /// Named rather than decided here: `datasets` knows nothing about the
    /// decoders, and a caller that has the registry can ask it.
    pub fn protocol(self) -> Option<&'static str> {
        match self {
            Model::Rs41 => Some("rs41"),
            _ => None,
        }
    }
}

/// One instrument a site flies: the model, the code it was published under,
/// and the frequency where a site is known to keep to one.
#[derive(Clone, Debug, PartialEq)]
pub struct Sonde {
    pub model: Model,
    pub code: String,
    /// Some sites publish a fixed channel beside the type. Most do not: the
    /// station picks one on the day.
    pub hz: Option<f64>,
}

impl Sonde {
    /// What to call this instrument on screen: the model where one is named,
    /// otherwise the legacy name behind the code, otherwise the code itself.
    pub fn label(&self) -> String {
        if self.model != Model::Other {
            return self.model.label().into();
        }
        let code = self.code.trim_start_matches('0');
        let code = if code.is_empty() { "0" } else { code };
        match UNREAD.iter().find(|(c, _)| *c == code) {
            Some((_, name)) => (*name).into(),
            None => format!("WMO type {code}"),
        }
    }
}

/// When a site launches. Times are UTC and synoptic, and a balloon can go up
/// as much as an hour before the hour it is filed under.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Launch {
    pub day: Day,
    pub hour: u8,
    pub minute: u8,
}

/// The day part of a launch time. SondeHub writes it as a leading field on
/// the time: zero for every day, otherwise the ISO weekday.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Day {
    Every,
    Monday,
    Tuesday,
    Wednesday,
    Thursday,
    Friday,
    Saturday,
    Sunday,
}

impl Day {
    fn from_field(n: u8) -> Option<Day> {
        Some(match n {
            0 => Day::Every,
            1 => Day::Monday,
            2 => Day::Tuesday,
            3 => Day::Wednesday,
            4 => Day::Thursday,
            5 => Day::Friday,
            6 => Day::Saturday,
            7 => Day::Sunday,
            _ => return None,
        })
    }

    pub fn label(self) -> &'static str {
        match self {
            Day::Every => "every day",
            Day::Monday => "Monday",
            Day::Tuesday => "Tuesday",
            Day::Wednesday => "Wednesday",
            Day::Thursday => "Thursday",
            Day::Friday => "Friday",
            Day::Saturday => "Saturday",
            Day::Sunday => "Sunday",
        }
    }

    /// The ISO weekday, or `None` for a daily launch.
    fn iso(self) -> Option<u32> {
        match self {
            Day::Every => None,
            d => Some(d as u32),
        }
    }
}

impl Launch {
    /// Written as SondeHub writes it: `0:12:00` is every day at 12:00 UTC,
    /// `3:18:00` is Wednesdays at 18:00 UTC. Not a duration, however much it
    /// looks like one.
    pub fn parse(s: &str) -> Option<Launch> {
        let mut f = s.split(':');
        let day = Day::from_field(f.next()?.trim().parse().ok()?)?;
        let hour: u8 = f.next()?.trim().parse().ok()?;
        let minute: u8 = f.next()?.trim().parse().ok()?;
        (hour < 24 && minute < 60).then_some(Launch { day, hour, minute })
    }

    /// How long until this launch, from `now`. Zero at the moment itself,
    /// and never more than a week.
    pub fn after(&self, now: chrono::DateTime<chrono::Utc>) -> Duration {
        use chrono::{Datelike, Timelike};
        let now_s = i64::from(now.hour() * 3600 + now.minute() * 60 + now.second());
        let at_s = i64::from(u32::from(self.hour) * 3600 + u32::from(self.minute) * 60);
        let days = match self.day.iso() {
            None => i64::from(at_s < now_s),
            Some(iso) => {
                let ahead = (iso as i64 - now.weekday().number_from_monday() as i64).rem_euclid(7);
                match ahead == 0 && at_s < now_s {
                    true => 7,
                    false => ahead,
                }
            }
        };
        Duration::from_secs((days * 86_400 + at_s - now_s) as u64)
    }
}

/// A place balloons go up from.
#[derive(Clone, Debug)]
pub struct Site {
    /// The WMO station number, or a negative number for a site that has no
    /// WMO registration. An identifier, so it stays a string.
    pub station: String,
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    pub alt_m: f64,
    pub sondes: Vec<Sonde>,
    /// Empty where nobody has worked out the schedule yet, which is true of
    /// about one site in seven.
    pub schedule: Vec<Launch>,
    /// What the flights from here have been measured to do, where enough of
    /// them have been tracked: metres a second up, metres a second down
    /// under the parachute, and the height the balloon bursts at.
    pub ascent_ms: Option<f64>,
    pub descent_ms: Option<f64>,
    pub burst_m: Option<f64>,
    pub notes: String,
}

impl Site {
    /// Whether this site flies something this receiver can read.
    pub fn flies(&self, model: Model) -> bool {
        self.sondes.iter().any(|s| s.model == model)
    }

    /// The soonest launch from here, as the time until it and the entry it
    /// came from. `None` where the schedule is not known.
    pub fn next_launch(&self, now: chrono::DateTime<chrono::Utc>) -> Option<(Duration, Launch)> {
        self.schedule.iter().map(|l| (l.after(now), *l)).min_by_key(|(d, _)| *d)
    }
}

mod wire {
    //! The published shape: a map of station number to site. Everything but
    //! the name, the position and the height is optional, and `rs_types`
    //! holds either a bare code or a code with a frequency beside it.

    #[derive(serde::Deserialize)]
    pub struct Site {
        #[serde(default)]
        pub station_name: String,
        /// Longitude first, as GeoJSON has it.
        #[serde(default)]
        pub position: Vec<f64>,
        #[serde(default)]
        pub alt: f64,
        #[serde(default)]
        pub rs_types: Vec<Type>,
        #[serde(default)]
        pub times: Vec<String>,
        #[serde(default)]
        pub ascent_rate: Option<f64>,
        #[serde(default)]
        pub descent_rate: Option<f64>,
        #[serde(default)]
        pub burst_altitude: Option<f64>,
        #[serde(default)]
        pub notes: Option<String>,
    }

    /// `"41"`, or `["41", "404.1"]` where the site keeps to a channel. Both
    /// the code and the megahertz are written as a number by some
    /// contributors and as a string by others, so a field is either.
    #[derive(serde::Deserialize)]
    #[serde(untagged)]
    pub enum Type {
        Code(Field),
        WithFrequency(Vec<Field>),
    }

    #[derive(serde::Deserialize)]
    #[serde(untagged)]
    pub enum Field {
        Text(String),
        Number(f64),
    }

    impl Field {
        pub fn text(&self) -> String {
            match self {
                Field::Text(s) => s.clone(),
                Field::Number(n) => n.to_string(),
            }
        }

        pub fn number(&self) -> Option<f64> {
            match self {
                Field::Text(s) => s.trim().parse().ok(),
                Field::Number(n) => Some(*n),
            }
        }
    }
}

pub fn parse(name: &str, json: &[u8]) -> Result<Vec<Site>, Error> {
    let doc: std::collections::HashMap<String, wire::Site> =
        serde_json::from_slice(json).map_err(|e| Error::Parse(name.into(), e.to_string()))?;
    let mut sites: Vec<Site> = doc.into_iter().filter_map(|(id, s)| site(id, s)).collect();
    if sites.is_empty() {
        return Err(Error::Parse(name.into(), "no launch sites in the file".into()));
    }
    // By station number, so a list of them reads the way the WMO blocks run
    // and two runs of the program show the same order: the file is a map,
    // and a map has none.
    sites.sort_by(|a, b| a.station.cmp(&b.station));
    Ok(sites)
}

fn site(station: String, s: wire::Site) -> Option<Site> {
    let (lon, lat) = match s.position.as_slice() {
        [lon, lat, ..] => (*lon, *lat),
        _ => return None,
    };
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return None;
    }
    Some(Site {
        station,
        name: s.station_name,
        lat,
        lon,
        alt_m: s.alt,
        sondes: s.rs_types.iter().map(sonde).collect(),
        // A time nobody can parse is dropped rather than shown as a guess:
        // a handful of sites write prose there ("Irregular", "mission
        // driven"), and those belong in the notes if anywhere.
        schedule: s.times.iter().filter_map(|t| Launch::parse(t)).collect(),
        ascent_ms: s.ascent_rate,
        descent_ms: s.descent_rate,
        burst_m: s.burst_altitude,
        notes: s.notes.unwrap_or_default(),
    })
}

fn sonde(t: &wire::Type) -> Sonde {
    let (code, hz) = match t {
        wire::Type::Code(c) => (c.text(), None),
        wire::Type::WithFrequency(v) => (
            v.first().map(|f| f.text()).unwrap_or_default(),
            v.get(1).and_then(|f| f.number()).map(|mhz| mhz * 1e6),
        ),
    };
    Sonde { model: Model::from_code(&code), code, hz }
}

pub fn load(cache: &Cache) -> Result<Vec<Site>, Error> {
    let src = source();
    parse(src.name, &cache.read(&src)?)
}

pub fn refresh(cache: &Cache, when: When) -> Result<Option<Vec<Site>>, Error> {
    let src = source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse(src.name, &cache.read(&src)?).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;

    /// Rows in the shape the API answers with: a WMO station launching twice
    /// a day, an unofficial site with a weekday schedule and measured
    /// flight numbers, a site whose type carries a frequency, and one whose
    /// schedule is a sentence.
    const FILE: &str = r#"{
      "03918": {"station_name":"Castor Bay (Ireland)","station":"03918","alt":26.0,
                "position":[-6.3339,54.4964],"rs_types":["41"],
                "times":["0:00:00","0:12:00"]},
      "-1":    {"station_name":"Melbourne BoM (Australia)","station":"-1","alt":119.0,
                "position":[144.947375,-37.689883],"rs_types":["41"],"times":["3:00:00"],
                "ascent_rate":5.4,"descent_rate":5.3,"burst_altitude":29500.0,
                "notes":"OzoneSonde Launches on Wednesdays"},
      "72265": {"station_name":"Del Rio (United States)","station":"72265","alt":313.0,
                "position":[-100.92,29.37],"rs_types":[["82","1680.0"],"20"],
                "times":["0:11:00","Irregular"]}
    }"#;

    fn sites() -> Vec<Site> {
        parse("test", FILE.as_bytes()).expect("the file parses")
    }

    #[test]
    fn a_site_carries_its_position_schedule_and_instrument() {
        let sites = sites();
        assert_eq!(sites.len(), 3, "{:?}", sites.iter().map(|s| &s.station).collect::<Vec<_>>());
        // Sorted by station number, so the unofficial site sorts first.
        assert_eq!(sites[0].station, "-1");
        let castor = sites.iter().find(|s| s.station == "03918").expect("the Irish site");
        assert!((castor.lat - 54.4964).abs() < 1e-6, "{}", castor.lat);
        assert!((castor.lon + 6.3339).abs() < 1e-6, "{}", castor.lon);
        assert_eq!(castor.alt_m, 26.0);
        assert_eq!(castor.sondes.len(), 1);
        assert_eq!(castor.sondes[0].model, Model::Rs41);
        assert_eq!(castor.sondes[0].hz, None);
        assert!(castor.flies(Model::Rs41));
        assert_eq!(castor.sondes[0].model.protocol(), Some("rs41"));
        assert_eq!(
            castor.schedule,
            [
                Launch { day: Day::Every, hour: 0, minute: 0 },
                Launch { day: Day::Every, hour: 12, minute: 0 }
            ]
        );
    }

    #[test]
    fn a_type_can_carry_a_frequency_and_prose_is_not_a_launch_time() {
        let sites = sites();
        let del_rio = sites.iter().find(|s| s.station == "72265").expect("the Texan site");
        assert_eq!(del_rio.sondes.len(), 2);
        assert_eq!(del_rio.sondes[0].model, Model::Lms6_1680);
        assert_eq!(del_rio.sondes[0].hz, Some(1_680_000_000.0));
        // Code 20 is an MK3, which nothing here reads.
        assert_eq!(del_rio.sondes[1].model, Model::Other);
        assert_eq!(del_rio.sondes[1].code, "20");
        assert_eq!(del_rio.sondes[1].model.protocol(), None);
        assert_eq!(del_rio.sondes[1].label(), "MK3");
        assert_eq!(del_rio.sondes[0].label(), "LMS6-1680");
        assert!(!del_rio.flies(Model::Rs41));
        // "Irregular" is dropped; the one real time is kept.
        assert_eq!(del_rio.schedule, [Launch { day: Day::Every, hour: 11, minute: 0 }]);
    }

    /// The measured flight numbers, and a weekday schedule read as a
    /// weekday rather than as three hours.
    #[test]
    fn a_weekday_schedule_names_its_day() {
        let sites = sites();
        let melb = sites.iter().find(|s| s.station == "-1").expect("the Australian site");
        assert_eq!(melb.schedule, [Launch { day: Day::Wednesday, hour: 0, minute: 0 }]);
        assert_eq!(melb.ascent_ms, Some(5.4));
        assert_eq!(melb.descent_ms, Some(5.3));
        assert_eq!(melb.burst_m, Some(29_500.0));
        assert_eq!(melb.notes, "OzoneSonde Launches on Wednesdays");
    }

    /// The next launch from a site, which is the whole point of holding the
    /// schedule: a daily site rolls over midnight, a weekday site waits for
    /// its day, and a launch happening now is now rather than a week away.
    #[test]
    fn the_next_launch_is_counted_from_now() {
        let at = |h, m| chrono::Utc.with_ymd_and_hms(2025, 3, 13, h, m, 0).unwrap();
        // 13 March 2025 is a Thursday.
        assert_eq!(at(9, 0).format("%A").to_string(), "Thursday");
        let sites = sites();
        let castor = sites.iter().find(|s| s.station == "03918").unwrap();
        let melb = sites.iter().find(|s| s.station == "-1").unwrap();

        let (in_, next) = castor.next_launch(at(9, 30)).expect("a next launch");
        assert_eq!(next, Launch { day: Day::Every, hour: 12, minute: 0 });
        assert_eq!(in_, Duration::from_secs(150 * 60));
        // Past the last launch of the day, the next is tomorrow's first.
        let (in_, next) = castor.next_launch(at(23, 0)).expect("a next launch");
        assert_eq!(next, Launch { day: Day::Every, hour: 0, minute: 0 });
        assert_eq!(in_, Duration::from_secs(3600));
        // Thursday morning to the following Wednesday midnight: six days
        // ahead on the calendar, less the nine hours already gone today.
        let (in_, _) = melb.next_launch(at(9, 0)).expect("a next launch");
        assert_eq!(in_, Duration::from_secs(5 * 86_400 + 15 * 3600));
        // And a launch at this very minute is not next week's.
        let wed = chrono::Utc.with_ymd_and_hms(2025, 3, 19, 0, 0, 0).unwrap();
        assert_eq!(melb.next_launch(wed).expect("a next launch").0, Duration::ZERO);
    }
}
