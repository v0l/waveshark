//! What a satellite transmits on, from the SatNOGS transmitter database.
//!
//! Elements say where a satellite will be; this says where to tune when it
//! gets there. A row is one transmitter: a downlink, sometimes an uplink,
//! the mode, the baud rate where it is digital, and whether anybody has
//! heard it lately. SatNOGS keeps it because their ground stations need the
//! same two facts a receiver here does, and the numbers are crowd-corrected
//! against what the network actually observes rather than copied off a
//! launch press release.
//!
//! Keyed by NORAD catalogue number, which is what joins it to the elements.
//! A satellite has several transmitters and most of them are dead: a beacon
//! that stopped in 2011 is still in the file, correctly, because the file is
//! a record. Reading it means choosing, so [`Transmitters::best`] states the
//! choice in one place rather than leaving every caller to invent one.

use crate::cache::{Cache, Error, Source, When};
use std::time::Duration;

/// The whole transmitter table, about four megabytes of JSON.
///
/// One request rather than a query per satellite: the file is small, a pass
/// list asks about a hundred objects at once, and a receiver on a hilltop
/// wants the answer without a network.
pub fn source() -> Source {
    Source::http(
        "satnogs-transmitters.json",
        "https://db.satnogs.org/api/transmitters/?format=json",
        Duration::from_secs(24 * 3600),
    )
    .checked(|head| match head.starts_with(b"[") {
        true => Ok(()),
        false => Err("SatNOGS did not answer with the transmitter list".into()),
    })
}

/// One transmitter, as SatNOGS holds it.
#[derive(Clone, Debug, PartialEq)]
pub struct Transmitter {
    /// The satellite it belongs to, which is what joins this to a set of
    /// elements.
    pub norad: u64,
    /// What it is called on the satellite page: `Mode V/U FM voice`.
    pub description: String,
    /// Hertz, as transmitted. Doppler is the receiver's problem and is not
    /// baked in here.
    pub downlink_hz: Option<u64>,
    pub uplink_hz: Option<u64>,
    /// `FM`, `USB`, `BPSK`, `AFSK`, as SatNOGS names modes. Empty where it
    /// names none.
    pub mode: String,
    /// Symbols a second, where it is digital.
    pub baud: Option<f64>,
    /// Whether SatNOGS believes it is still transmitting. Most rows are
    /// dead: a file that is a record keeps them.
    pub alive: bool,
    /// `Amateur`, `Weather`, `Unknown`. What licence it is under, roughly.
    pub service: String,
    /// A transponder inverts and a beacon does not, which decides which way
    /// an uplink moves.
    pub invert: bool,
}

impl Transmitter {
    /// A phrase for a card: the mode, the frequency and what it is.
    pub fn label(&self) -> String {
        let f = match self.downlink_hz {
            Some(hz) => format!("{:.4} MHz", hz as f64 / 1e6),
            None => "no downlink".into(),
        };
        match (self.mode.is_empty(), self.description.is_empty()) {
            (false, false) => format!("{f} {} ({})", self.mode, self.description),
            (false, true) => format!("{f} {}", self.mode),
            _ => f,
        }
    }
}

/// Every transmitter in the file, sorted by satellite.
#[derive(Clone, Debug, Default)]
pub struct Transmitters(Vec<Transmitter>);

impl Transmitters {
    /// Every transmitter of one satellite, alive or not.
    pub fn for_norad(&self, norad: u64) -> impl Iterator<Item = &Transmitter> {
        let start = self.0.partition_point(|t| t.norad < norad);
        self.0[start..].iter().take_while(move |t| t.norad == norad)
    }

    /// The one downlink to quote for a satellite, or `None` if it has none.
    ///
    /// Alive first, because a dead beacon is not what anybody is waiting
    /// for; then a real downlink frequency; then the lowest, which for a
    /// satellite with both a two-metre and a seventy-centimetre downlink
    /// picks the one a wideband receiver is more likely to be on and the one
    /// with less Doppler to chase. Stated here rather than at each call so
    /// the map, the pass list and the dial quote the same number.
    pub fn best(&self, norad: u64) -> Option<&Transmitter> {
        self.for_norad(norad)
            .filter(|t| t.downlink_hz.is_some())
            .min_by_key(|t| (!t.alive, t.downlink_hz.unwrap_or(u64::MAX)))
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Transmitter> {
        self.0.iter()
    }
}

/// One row of the API, with everything this does not use left out.
#[derive(serde::Deserialize)]
struct Row {
    #[serde(default)]
    description: Option<String>,
    #[serde(default)]
    alive: bool,
    #[serde(default)]
    uplink_low: Option<u64>,
    #[serde(default)]
    downlink_low: Option<u64>,
    #[serde(default)]
    mode: Option<String>,
    #[serde(default)]
    baud: Option<f64>,
    #[serde(default)]
    norad_cat_id: Option<u64>,
    #[serde(default)]
    service: Option<String>,
    #[serde(default)]
    invert: bool,
}

pub fn parse(name: &str, raw: &[u8]) -> Result<Transmitters, Error> {
    let rows: Vec<Row> = serde_json::from_slice(raw)
        .map_err(|e| Error::Parse(name.into(), e.to_string()))?;
    let mut out: Vec<Transmitter> = rows
        .into_iter()
        .filter_map(|r| {
            // A row with no satellite cannot be joined to an orbit, so it is
            // nothing this can use. They exist: an entry made before the
            // object was catalogued.
            let norad = r.norad_cat_id?;
            Some(Transmitter {
                norad,
                description: r.description.unwrap_or_default(),
                downlink_hz: r.downlink_low.filter(|hz| *hz > 0),
                uplink_hz: r.uplink_low.filter(|hz| *hz > 0),
                mode: r.mode.unwrap_or_default(),
                baud: r.baud,
                alive: r.alive,
                service: r.service.unwrap_or_default(),
                invert: r.invert,
            })
        })
        .collect();
    if out.is_empty() {
        return Err(Error::Parse(name.into(), "no transmitters in the file".into()));
    }
    // Sorted so a satellite's rows are one run, which is what makes a lookup
    // a partition point rather than a scan of five thousand.
    out.sort_by_key(|t| (t.norad, !t.alive, t.downlink_hz.unwrap_or(u64::MAX)));
    Ok(Transmitters(out))
}

pub fn load(cache: &Cache) -> Result<Transmitters, Error> {
    let src = source();
    parse(src.name, &cache.read(&src)?)
}

pub fn refresh(cache: &Cache, when: When) -> Result<Option<Transmitters>, Error> {
    let src = source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse(src.name, &cache.read(&src)?).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rows in the shape the API answers with, including the ones that make
    /// choosing hard: a dead beacon, a satellite with two downlinks, and an
    /// entry for an object that has no catalogue number.
    const FILE: &str = r#"[
      {"uuid":"a","description":"Mode U TLM","alive":false,"uplink_low":null,
       "downlink_low":437505000,"mode":"BPSK","baud":1200.0,"norad_cat_id":25544,
       "status":"inactive","service":"Amateur","invert":false},
      {"uuid":"b","description":"Voice repeater","alive":true,"uplink_low":145990000,
       "downlink_low":437800000,"mode":"FM","baud":null,"norad_cat_id":25544,
       "status":"active","service":"Amateur","invert":false},
      {"uuid":"c","description":"Mode V APRS","alive":true,"uplink_low":145825000,
       "downlink_low":145825000,"mode":"AFSK","baud":1200.0,"norad_cat_id":25544,
       "status":"active","service":"Amateur","invert":false},
      {"uuid":"d","description":"Beacon","alive":true,"uplink_low":null,
       "downlink_low":435300000,"mode":"CW","baud":null,"norad_cat_id":7530,
       "status":"active","service":"Amateur","invert":false},
      {"uuid":"e","description":"Uncatalogued","alive":true,"uplink_low":null,
       "downlink_low":401000000,"mode":"FSK","baud":9600.0,"norad_cat_id":null,
       "status":"active","service":"Unknown","invert":false}
    ]"#;

    fn db() -> Transmitters {
        parse("test", FILE.as_bytes()).expect("four transmitters")
    }

    #[test]
    fn a_row_with_no_satellite_is_not_a_transmitter_anything_can_use() {
        assert_eq!(db().len(), 4);
        assert!(db().iter().all(|t| t.norad != 0));
    }

    #[test]
    fn a_satellites_transmitters_are_found_by_its_catalogue_number() {
        let d = db();
        assert_eq!(d.for_norad(25544).count(), 3);
        assert_eq!(d.for_norad(7530).count(), 1);
        assert_eq!(d.for_norad(99999).count(), 0);
    }

    /// The alive one is picked over the dead one even though the dead one is
    /// lower, and the lowest is picked among the living.
    #[test]
    fn the_downlink_to_quote_is_alive_and_then_lowest() {
        let d = db();
        let best = d.best(25544).expect("a downlink");
        assert_eq!(best.downlink_hz, Some(145_825_000));
        assert!(best.alive);
        assert_eq!(d.best(7530).map(|t| t.mode.clone()), Some("CW".into()));
        assert!(d.best(99999).is_none());
    }

    #[test]
    fn a_label_says_the_frequency_the_mode_and_what_it_is() {
        let d = db();
        assert_eq!(d.best(25544).unwrap().label(), "145.8250 MHz AFSK (Mode V APRS)");
    }

    #[test]
    fn a_file_of_prose_is_an_error_rather_than_an_empty_table() {
        assert!(parse("test", b"<html>maintenance</html>").is_err());
        assert!(parse("test", b"[]").is_err());
    }
}
