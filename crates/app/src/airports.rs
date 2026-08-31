//! Airports and their air traffic frequencies, bundled from OurAirports.
//!
//! The map draws the aircraft a rooftop hears over tiles it fetches itself,
//! but an airport is a fixed thing and the frequencies of it are published
//! data, so neither has to arrive over the network. `data/airports.tsv` and
//! `data/frequencies.tsv` are a filtered slice of the public-domain OurAirports
//! dataset: the airports of Ireland, Britain, northern France, the Low
//! Countries, Germany, Scandinavia and Iceland, which is the reach a receiver
//! in the north Atlantic corridor is pointed at. They are read once at startup
//! and held, because a few thousand rows parsed at load is a millisecond, not
//! a cost to amortise.
//!
//! The two files stay separate because the source does: airports are a list,
//! frequencies are a list that references it. Joining them into one row with
//! inline frequencies would make the format harder to read and the parse no
//! simpler.

use std::sync::LazyLock;

/// The zoom at which airports first appear. Below this the map is wide enough
/// that every marker would be a blob under a handful of aircraft, and the
/// range rings already say where the interesting things are.
pub const SHOW_ZOOM: f64 = 9.0;

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

#[derive(Debug)]
pub struct Freq {
    pub kind: FreqKind,
    /// The role as OurAirports named it, kept because the bucket loses detail
    /// ("Clearance Delivery" rather than "Delivery").
    pub desc: String,
    pub mhz: f64,
}

#[derive(Debug)]
pub struct Airport {
    pub ident: String,
    pub name: String,
    pub kind: Kind,
    pub lat: f64,
    pub lon: f64,
    /// Above mean sea level. Missing in the source for a sixth of the slice
    /// (most of them small strips), so it is optional rather than a lie.
    pub elev_ft: Option<i32>,
    pub freqs: Vec<Freq>,
}

/// The bundled slice, parsed once. `all()` hands out references that outlive
/// any frame, so the view can borrow them for a tooltip without a clone.
pub static AIRPORTS: LazyLock<Vec<Airport>> = LazyLock::new(parse_bundle);

pub fn all() -> &'static [Airport] {
    &AIRPORTS
}

fn parse_bundle() -> Vec<Airport> {
    let mut by_ident: std::collections::HashMap<&str, (Kind, f64, f64, Option<i32>)> =
        std::collections::HashMap::new();
    let mut names: std::collections::HashMap<&str, &str> = std::collections::HashMap::new();

    // Header lines start with '#'. Every other line is ident-name-type-lat-lon-elev.
    for line in AIRPORT_ROWS.lines().filter(|l| !l.is_empty() && !l.starts_with('#')) {
        let mut it = line.splitn(6, '\t');
        let (ident, name, ty, lat, lon, elev) = match (it.next(), it.next(), it.next(), it.next(), it.next(), it.next()) {
            (Some(a), Some(n), Some(t), Some(la), Some(lo), Some(el)) => (a, n, t, la, lo, el),
            _ => continue,
        };
        let kind = match ty {
            "large_airport" => Kind::Large,
            "medium_airport" => Kind::Medium,
            _ => Kind::Small,
        };
        let (Ok(lat), Ok(lon)) = (lat.parse(), lon.parse()) else { continue };
        let elev = elev.trim().parse().ok();
        by_ident.insert(ident, (kind, lat, lon, elev));
        names.insert(ident, name);
    }

    let mut freqs: std::collections::HashMap<&str, Vec<Freq>> = std::collections::HashMap::new();
    for line in FREQ_ROWS.lines().filter(|l| !l.is_empty() && !l.starts_with('#')) {
        let mut it = line.splitn(4, '\t');
        let (ident, ty, desc, mhz) = match (it.next(), it.next(), it.next(), it.next()) {
            (Some(a), Some(t), Some(d), Some(m)) => (a, t, d, m),
            _ => continue,
        };
        let Ok(mhz) = mhz.parse() else { continue };
        let desc = if desc.is_empty() { ty } else { desc };
        freqs.entry(ident).or_default().push(Freq { kind: freq_kind(ty), desc: desc.to_string(), mhz });
    }

    let mut out: Vec<Airport> = Vec::new();
    for (ident, (kind, lat, lon, elev)) in by_ident {
        let mut f = freqs.remove(ident).unwrap_or_default();
        // Primary traffic frequencies first, then everything else in the order
        // they appeared. A pilot reads Tower before Approach.
        f.sort_by_key(|x| x.kind);
        // Exact duplicates happen in the source (a frequency listed twice).
        f.dedup_by(|a, b| a.kind == b.kind && a.desc == b.desc && a.mhz == b.mhz);
        out.push(Airport {
            ident: ident.to_string(),
            name: names.get(ident).copied().unwrap_or("").to_string(),
            kind,
            lat,
            lon,
            elev_ft: elev,
            freqs: f,
        });
    }
    out
}

/// Format a frequency the way an airband chart does: 118.6, not 118.600.
pub fn fmt_mhz(mhz: f64) -> String {
    let s = format!("{mhz:.3}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

const AIRPORT_ROWS: &str = include_str!("../data/airports.tsv");
const FREQ_ROWS: &str = include_str!("../data/frequencies.tsv");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_bundle_parses_and_dublin_is_in_it() {
        let aps = all();
        assert!(aps.len() > 3000, "bundle shrank: {}", aps.len());
        let dublin = aps.iter().find(|a| a.ident == "EIDW").expect("Dublin");
        assert_eq!(dublin.kind, Kind::Large);
        assert!((dublin.lat - 53.4287).abs() < 0.01);
        assert!((dublin.lon - -6.2621).abs() < 0.01);
        // Its tower, ground and ATIS, the frequencies a flight observer wants.
        assert!(dublin.freqs.iter().any(|f| f.kind == FreqKind::Tower && (f.mhz - 118.6).abs() < 1e-9));
        assert!(dublin.freqs.iter().any(|f| f.kind == FreqKind::Ground && (f.mhz - 121.8).abs() < 1e-9));
        assert!(dublin.freqs.iter().any(|f| f.kind == FreqKind::Atis));
    }

    #[test]
    fn frequencies_sort_with_the_primary_first() {
        let aps = all();
        let heathrow = aps.iter().find(|a| a.ident == "EGLL").expect("Heathrow");
        let kinds: Vec<FreqKind> = heathrow.freqs.iter().map(|f| f.kind).collect();
        let mut sorted = kinds.clone();
        sorted.sort();
        // The primary kinds already precede any Others, and Tower comes first.
        assert_eq!(kinds, sorted, "frequencies not in reading order");
        assert!(heathrow.freqs.iter().any(|f| f.kind == FreqKind::Tower));
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

    #[test]
    fn every_frequency_belongs_to_a_bundled_airport() {
        // A frequency for an airport outside the slice is data that will never
        // be shown, which means the slice and the frequencies have drifted.
        let aps = all();
        let idents: std::collections::HashSet<&str> =
            aps.iter().map(|a| a.ident.as_str()).collect();
        for f in FREQ_ROWS.lines().filter(|l| !l.is_empty() && !l.starts_with('#')) {
            let ident = f.split('\t').next().unwrap_or("");
            assert!(idents.contains(ident), "orphan frequency for {ident}");
        }
    }
}
