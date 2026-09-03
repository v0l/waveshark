//! Airports and their air traffic frequencies, from OurAirports.
//!
//! The map draws the aircraft a rooftop hears over tiles it fetches itself,
//! and an airport is a fixed thing whose frequencies are published, so it
//! comes down the same way the tiles do. The two files stay separate because
//! the source does: airports are a list, frequencies are a list that
//! references it.
//!
//! The published files are the whole world, and the whole world is what is
//! kept. An earlier build shipped a filtered slice of northern Europe in the
//! binary, which made the release the only way to fix a wrong frequency and
//! made a receiver anywhere else stare at an empty map.

use crate::cache::{Cache, Error, Source};
use std::time::Duration;

/// OurAirports publishes a daily rebuild, so a check a day apart cannot miss
/// much and a check every launch would be noise.
const MAX_AGE: Duration = Duration::from_secs(24 * 3600);

pub fn airports_source() -> Source {
    Source::http(
        "ourairports-airports.csv",
        "https://davidmegginson.github.io/ourairports-data/airports.csv",
        MAX_AGE,
    )
}

pub fn frequencies_source() -> Source {
    Source::http(
        "ourairports-frequencies.csv",
        "https://davidmegginson.github.io/ourairports-data/airport-frequencies.csv",
        MAX_AGE,
    )
}

/// How an airport's frequencies are grouped in the tooltip, in the order a
/// pilot would read them. Everything left over sorts after Approach.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum FreqKind {
    Tower,
    Ground,
    Delivery,
    Atis,
    Approach,
    Other,
}

impl FreqKind {
    /// Short name for the tooltip row. The raw OurAirports `type` is a mess
    /// of case and abbreviation ("TWR", "Twr", "Tower", "APP", "APP vhf"),
    /// so it is bucketed into a small set and shown as the bucket.
    pub fn label(self) -> &'static str {
        match self {
            FreqKind::Tower => "Tower",
            FreqKind::Ground => "Ground",
            FreqKind::Delivery => "Clearance",
            FreqKind::Atis => "ATIS",
            FreqKind::Approach => "Approach",
            FreqKind::Other => "Other",
        }
    }
}

fn freq_kind(raw: &str) -> FreqKind {
    let u = raw.to_ascii_uppercase();
    if u.contains("TWR") || u == "TOWER" {
        FreqKind::Tower
    } else if u.contains("GND") || u.contains("GROUND") || u == "GRN" {
        FreqKind::Ground
    } else if u.contains("DEL") {
        FreqKind::Delivery
    } else if u.contains("ATIS") {
        FreqKind::Atis
    } else if u.contains("APP") {
        FreqKind::Approach
    } else {
        FreqKind::Other
    }
}

/// Size class, from OurAirports. Drives how the marker is drawn and how soon
/// the ident label appears.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Kind {
    Large,
    Medium,
    Small,
}

#[derive(Clone, Debug)]
pub struct Freq {
    pub kind: FreqKind,
    /// The role as OurAirports named it, kept because the bucket loses detail
    /// ("Clearance Delivery" rather than "Delivery").
    pub desc: String,
    pub mhz: f64,
}

#[derive(Clone, Debug)]
pub struct Airport {
    pub ident: String,
    pub name: String,
    pub kind: Kind,
    pub lat: f64,
    pub lon: f64,
    /// Above mean sea level. Missing in the source for a sixth of the rows
    /// (most of them small strips), so it is optional rather than a lie.
    pub elev_ft: Option<i32>,
    pub freqs: Vec<Freq>,
}

/// Read from the cache, downloading whichever file is not there yet.
pub fn load(cache: &Cache) -> Result<Vec<Airport>, Error> {
    let a = cache.read(&airports_source())?;
    let f = cache.read(&frequencies_source())?;
    parse(&a, &f)
}

/// Check both files and reparse if either changed. `None` means the cached
/// airports are still current.
pub fn refresh(cache: &Cache) -> Result<Option<Vec<Airport>>, Error> {
    let (asrc, fsrc) = (airports_source(), frequencies_source());
    // Both are asked before either result is used: a changed airport list
    // with last week's frequencies is a worse state than either alone.
    let changed = cache.refresh(&asrc)?.is_some() | cache.refresh(&fsrc)?.is_some();
    if !changed {
        return Ok(None);
    }
    parse(&cache.read(&asrc)?, &cache.read(&fsrc)?).map(Some)
}

fn parse(airports_csv: &[u8], freqs_csv: &[u8]) -> Result<Vec<Airport>, Error> {
    let a = std::str::from_utf8(airports_csv)
        .map_err(|e| Error::Parse("airports.csv".into(), e.to_string()))?;
    let f = std::str::from_utf8(freqs_csv)
        .map_err(|e| Error::Parse("airport-frequencies.csv".into(), e.to_string()))?;
    let mut out = parse_airports(a)?;
    attach_frequencies(&mut out, f)?;
    Ok(out)
}

fn parse_airports(text: &str) -> Result<Vec<Airport>, Error> {
    let mut rows = reader(text);
    let cols = columns(&mut rows, "airports.csv")?;
    let need = |n: &str| {
        cols.iter()
            .position(|c| c == n)
            .ok_or_else(|| Error::Parse("airports.csv".into(), format!("no {n} column")))
    };
    let (ci, ct, cn, cla, clo, ce) = (
        need("ident")?,
        need("type")?,
        need("name")?,
        need("latitude_deg")?,
        need("longitude_deg")?,
        need("elevation_ft")?,
    );
    let mut out = Vec::with_capacity(1 << 15);
    let mut row = csv::StringRecord::new();
    while next(&mut rows, &mut row)? {
        let Some(ty) = row.get(ct) else { continue };
        // Heliports, seaplane bases, balloonports and closed fields are two
        // thirds of the file and none of them is an airband facility. The
        // marker scheme has three sizes because the source does.
        let kind = match ty {
            "large_airport" => Kind::Large,
            "medium_airport" => Kind::Medium,
            "small_airport" => Kind::Small,
            _ => continue,
        };
        let (Some(ident), Some(name), Some(lat), Some(lon)) =
            (row.get(ci), row.get(cn), row.get(cla), row.get(clo))
        else {
            continue;
        };
        let (Ok(lat), Ok(lon)) = (lat.parse(), lon.parse()) else { continue };
        out.push(Airport {
            ident: ident.to_string(),
            name: name.to_string(),
            kind,
            lat,
            lon,
            elev_ft: row.get(ce).and_then(|e| e.trim().parse().ok()),
            freqs: Vec::new(),
        });
    }
    if out.is_empty() {
        return Err(Error::Parse("airports.csv".into(), "no airports in the file".into()));
    }
    out.sort_by(|a, b| a.ident.cmp(&b.ident));
    Ok(out)
}

fn attach_frequencies(airports: &mut [Airport], text: &str) -> Result<(), Error> {
    let file = "airport-frequencies.csv";
    let mut rows = reader(text);
    let cols = columns(&mut rows, file)?;
    let need = |n: &str| {
        cols.iter()
            .position(|c| c == n)
            .ok_or_else(|| Error::Parse(file.into(), format!("no {n} column")))
    };
    let (ci, ct, cd, cm) =
        (need("airport_ident")?, need("type")?, need("description")?, need("frequency_mhz")?);
    let mut row = csv::StringRecord::new();
    while next(&mut rows, &mut row)? {
        let (Some(ident), Some(ty), Some(mhz)) = (row.get(ci), row.get(ct), row.get(cm)) else {
            continue;
        };
        let Ok(mhz) = mhz.parse() else { continue };
        // Most frequencies in the file belong to airports that were filtered
        // out above, so the miss is the common case and has to be cheap.
        let Ok(at) = airports.binary_search_by(|a| a.ident.as_str().cmp(ident)) else { continue };
        let desc = row.get(cd).filter(|d| !d.is_empty()).unwrap_or(ty);
        airports[at].freqs.push(Freq { kind: freq_kind(ty), desc: desc.to_string(), mhz });
    }
    for a in airports.iter_mut() {
        // Primary traffic frequencies first, then everything else in the
        // order they appeared. A pilot reads Tower before Approach.
        a.freqs.sort_by_key(|x| x.kind);
        // Exact duplicates happen in the source (a frequency listed twice).
        a.freqs.dedup_by(|x, y| x.kind == y.kind && x.desc == y.desc && x.mhz == y.mhz);
    }
    Ok(())
}

/// The published files have a header row and rows that are occasionally
/// short, so the reader is told both.
fn reader(text: &str) -> csv::Reader<&[u8]> {
    csv::ReaderBuilder::new().flexible(true).from_reader(text.as_bytes())
}

/// Column names, read once. Everything below is looked up by name, so a new
/// column added to the front of a published dump shifts nothing here.
fn columns(rows: &mut csv::Reader<&[u8]>, file: &str) -> Result<Vec<String>, Error> {
    let head = rows.headers().map_err(|e| Error::Parse(file.into(), e.to_string()))?;
    Ok(head.iter().map(str::to_string).collect())
}

fn next(rows: &mut csv::Reader<&[u8]>, row: &mut csv::StringRecord) -> Result<bool, Error> {
    rows.read_record(row).map_err(|e| Error::Parse("csv".into(), e.to_string()))
}

/// Format a frequency the way an airband chart does: 118.6, not 118.600.
pub fn fmt_mhz(mhz: f64) -> String {
    let s = format!("{mhz:.3}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    const AIRPORTS: &str = "\"id\",\"ident\",\"type\",\"name\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\"\n\
1,\"EIDW\",\"large_airport\",\"Dublin Airport\",53.421299,-6.27007,242\n\
2,\"EGLL\",\"large_airport\",\"London Heathrow Airport\",51.4706,-0.461941,83\n\
3,\"EIWT\",\"small_airport\",\"Weston Airport\",53.3522,-6.48611,150\n\
4,\"XX-0001\",\"heliport\",\"Rooftop, somewhere\",1.0,2.0,\n\
5,\"XX-0002\",\"closed\",\"Old field\",3.0,4.0,10\n";

    const FREQS: &str = "\"id\",\"airport_ref\",\"airport_ident\",\"type\",\"description\",\"frequency_mhz\"\n\
1,1,\"EIDW\",\"APP\",\"Dublin Approach\",121.1\n\
2,1,\"EIDW\",\"TWR\",\"Dublin Tower\",118.6\n\
3,1,\"EIDW\",\"TWR\",\"Dublin Tower\",118.6\n\
4,1,\"EIDW\",\"ATIS\",\"\",124.525\n\
5,9,\"ZZZZ\",\"TWR\",\"Nowhere Tower\",123.0\n\
6,3,\"EIWT\",\"GND\",\"Weston Ground\",121.0\n";

    fn parsed() -> Vec<Airport> {
        parse(AIRPORTS.as_bytes(), FREQS.as_bytes()).unwrap()
    }

    #[test]
    fn only_airports_are_kept_not_heliports_or_closed_fields() {
        let a = parsed();
        let idents: Vec<&str> = a.iter().map(|x| x.ident.as_str()).collect();
        assert_eq!(idents, ["EGLL", "EIDW", "EIWT"]);
    }

    #[test]
    fn dublin_has_its_tower_ground_and_atis() {
        let a = parsed();
        let dublin = a.iter().find(|x| x.ident == "EIDW").expect("Dublin");
        assert_eq!(dublin.kind, Kind::Large);
        assert!((dublin.lat - 53.4213).abs() < 0.01);
        assert_eq!(dublin.elev_ft, Some(242));
        assert!(dublin.freqs.iter().any(|f| f.kind == FreqKind::Tower && f.mhz == 118.6));
        assert!(dublin.freqs.iter().any(|f| f.kind == FreqKind::Atis));
        // The duplicate tower row in the source is not shown twice.
        assert_eq!(dublin.freqs.iter().filter(|f| f.kind == FreqKind::Tower).count(), 1);
        // An empty description falls back to the raw type.
        let atis = dublin.freqs.iter().find(|f| f.kind == FreqKind::Atis).unwrap();
        assert_eq!(atis.desc, "ATIS");
    }

    #[test]
    fn frequencies_read_in_the_order_a_pilot_would() {
        let a = parsed();
        let dublin = a.iter().find(|x| x.ident == "EIDW").unwrap();
        let kinds: Vec<FreqKind> = dublin.freqs.iter().map(|f| f.kind).collect();
        let mut sorted = kinds.clone();
        sorted.sort();
        assert_eq!(kinds, sorted);
        assert_eq!(kinds.first(), Some(&FreqKind::Tower));
    }

    #[test]
    fn a_frequency_for_an_airport_that_was_filtered_out_is_dropped() {
        let a = parsed();
        assert!(a.iter().all(|x| x.freqs.iter().all(|f| f.desc != "Nowhere Tower")));
    }

    #[test]
    fn an_elevation_the_source_left_blank_is_absent_not_zero() {
        let missing = "\"id\",\"ident\",\"type\",\"name\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\"\n\
1,\"EIWT\",\"small_airport\",\"Weston\",53.3,-6.4,\n";
        let a = parse(missing.as_bytes(), FREQS.as_bytes()).unwrap();
        assert_eq!(a[0].elev_ft, None);
    }

    #[test]
    fn a_file_with_no_airports_is_an_error_rather_than_an_empty_map() {
        let empty = "\"id\",\"ident\",\"type\",\"name\",\"latitude_deg\",\"longitude_deg\",\"elevation_ft\"\n";
        assert!(parse(empty.as_bytes(), FREQS.as_bytes()).is_err());
    }

    #[test]
    fn the_ambiguous_type_is_bucketed_to_tower() {
        assert_eq!(freq_kind("TWR"), FreqKind::Tower);
        assert_eq!(freq_kind("Twr"), FreqKind::Tower);
        assert_eq!(freq_kind("Tower"), FreqKind::Tower);
        assert_eq!(freq_kind("GND"), FreqKind::Ground);
        assert_eq!(freq_kind("ground"), FreqKind::Ground);
        assert_eq!(freq_kind("APP vhf"), FreqKind::Approach);
        assert_eq!(freq_kind("MISC"), FreqKind::Other);
    }

    #[test]
    fn frequencies_format_without_trailing_zeros() {
        assert_eq!(fmt_mhz(118.6), "118.6");
        assert_eq!(fmt_mhz(124.525), "124.525");
        assert_eq!(fmt_mhz(121.0), "121");
    }
}
