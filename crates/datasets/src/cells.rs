//! Cellular reference data: who an MCC/MNC belongs to, and where the cells
//! with a given identity have been heard.
//!
//! A decoded GSM beacon carries numbers and nothing else: a country code, a
//! network code, a location area and a cell. Two published datasets turn
//! those into something an operator can read, and they are kept apart
//! because they answer different questions at very different sizes. The
//! [`Operators`] table is a few hundred kilobytes and names the network, so
//! it is fetched for everyone. The OpenCelliD export puts the cell on the
//! map, needs a token of the operator's own, and is downloaded one country
//! at a time: the world file is hundreds of megabytes of cells on other
//! continents, and a receiver hears one country's.

use crate::cache::{Cache, Error, Source, When};
use std::io::Read;
use std::time::Duration;

/// Both are rebuilt daily at the far end.
const DAILY: Duration = Duration::from_secs(24 * 3600);

pub fn operators_source() -> Source {
    Source::http(
        "mcc-mnc-list.json",
        "https://raw.githubusercontent.com/pbakondy/mcc-mnc-list/master/mcc-mnc-list.json",
        DAILY,
    )
}

/// The country export for one MCC.
///
/// The token is the operator's, so the URL is built rather than static. The
/// cache file name is not: it is the MCC, so changing country downloads a
/// second file rather than overwriting the first, and a token that changes
/// does not invalidate what was already fetched.
pub fn towers_source(mcc: u16, token: &str) -> Source {
    let url =
        format!("https://opencellid.org/ocid/downloads?token={token}&type=mcc&file={mcc}.csv.gz");
    Source::http(mcc_file(mcc), url, DAILY).checked(|head| match token_error(head) {
        Some(msg) => Err(msg),
        None => Ok(()),
    })
}

/// Leaked because [`Source::name`] is the cache file name and must outlive
/// the fetch. One country per run in practice, and a handful over a long
/// session; the alternative is a lifetime on `Source` for a string of four
/// bytes.
fn mcc_file(mcc: u16) -> &'static str {
    Box::leak(format!("opencellid-{mcc}.csv.gz").into_boxed_str())
}

/// One network, as the MCC/MNC list publishes it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Operator {
    pub mcc: u16,
    /// Published as a string because leading zeros are significant: MNC 01
    /// and MNC 1 are the same network, but MNC 001 is not MNC 01 everywhere.
    pub mnc: String,
    /// What subscribers call it, which is the name worth showing.
    pub brand: String,
    /// The licence holder, which is often a different and longer name.
    pub operator: String,
    pub country: String,
    /// ISO 3166-1 alpha-2, lowercase as published, empty for the few rows
    /// that carry a subdivision code instead.
    pub iso: String,
    /// `Operational`, `Not operational`, `Reserved`. A reserved code with no
    /// transmitter is still worth naming when one turns up on the air.
    pub status: String,
    /// Free text, as published: `GSM 900 / LTE 1800`.
    pub bands: String,
}

/// Every published network, sorted for lookup by the pair in a beacon.
#[derive(Clone, Debug, Default)]
pub struct Operators(Vec<Operator>);

impl Operators {
    /// The network a decoded MCC/MNC belongs to.
    ///
    /// GSM sends a two or three digit MNC and the file publishes whichever
    /// the country uses, so `01` from the air must still find `1` where that
    /// is how it is listed.
    pub fn get(&self, mcc: u16, mnc: &str) -> Option<&Operator> {
        let want = mnc.trim_start_matches('0');
        self.0
            .iter()
            .find(|o| o.mcc == mcc && (o.mnc == mnc || o.mnc.trim_start_matches('0') == want))
    }

    /// The MCC used in a country, for picking which OpenCelliD export to
    /// fetch from the country already set in the receiver. A country with
    /// several MCCs gets the one most of its networks are under.
    pub fn mcc_for_country(&self, iso: &str) -> Option<u16> {
        let iso = iso.to_ascii_lowercase();
        let mut counts: Vec<(u16, usize)> = Vec::new();
        for o in self.0.iter().filter(|o| o.iso == iso) {
            match counts.iter_mut().find(|(m, _)| *m == o.mcc) {
                Some((_, n)) => *n += 1,
                None => counts.push((o.mcc, 1)),
            }
        }
        counts.sort_by_key(|(mcc, n)| (std::cmp::Reverse(*n), *mcc));
        counts.first().map(|(mcc, _)| *mcc)
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Operator> {
        self.0.iter()
    }
}

/// Every field is optional and every one of them is published as `null`
/// somewhere in the file: a reserved MCC with no operator, a row with no
/// brand, a country with no ISO code. A missing name is a row worth keeping
/// without a name, not a file that fails to parse.
#[derive(serde::Deserialize)]
struct RawOperator {
    #[serde(default)]
    mcc: Option<String>,
    #[serde(default)]
    mnc: Option<String>,
    #[serde(default)]
    brand: Option<String>,
    #[serde(default)]
    operator: Option<String>,
    #[serde(default, rename = "countryName")]
    country_name: Option<String>,
    #[serde(default, rename = "countryCode")]
    country_code: Option<String>,
    #[serde(default)]
    status: Option<String>,
    #[serde(default)]
    bands: Option<String>,
}

pub fn parse_operators(raw: &[u8]) -> Result<Operators, Error> {
    let rows: Vec<RawOperator> = serde_json::from_slice(raw)
        .map_err(|e| Error::Parse("mcc-mnc-list.json".into(), e.to_string()))?;
    let mut out: Vec<Operator> = rows
        .into_iter()
        .filter_map(|r| {
            let code = r.country_code.unwrap_or_default();
            Some(Operator {
                mcc: r.mcc?.parse().ok()?,
                mnc: r.mnc.unwrap_or_default(),
                brand: r.brand.unwrap_or_default(),
                operator: r.operator.unwrap_or_default(),
                country: r.country_name.unwrap_or_default(),
                // `GE-AB` and the like are a subdivision, not a country, and
                // matching one against a two-letter setting would be wrong.
                iso: match code.len() {
                    2 => code.to_ascii_lowercase(),
                    _ => String::new(),
                },
                status: r.status.unwrap_or_default(),
                bands: r.bands.unwrap_or_default(),
            })
        })
        .collect();
    out.sort_by(|a, b| (a.mcc, a.mnc.as_str()).cmp(&(b.mcc, b.mnc.as_str())));
    Ok(Operators(out))
}

pub fn load_operators(cache: &Cache) -> Result<Operators, Error> {
    parse_operators(&cache.read(&operators_source())?)
}

pub fn refresh_operators(cache: &Cache, when: When) -> Result<Option<Operators>, Error> {
    if cache.refresh(&operators_source(), when)?.is_none() {
        return Ok(None);
    }
    load_operators(cache).map(Some)
}

/// One cell as OpenCelliD holds it: an identity, a position averaged over
/// the reports, and how sure the crowd is of it.
#[derive(Clone, Debug, PartialEq)]
pub struct Cell {
    /// `GSM`, `UMTS`, `LTE`, `NR`, `CDMA`, as published.
    pub radio: String,
    pub mcc: u16,
    /// Network code. Held as published, for the same reason as [`Operator`].
    pub mnc: String,
    /// Location area, tracking area or system identity, depending on radio.
    pub area: u32,
    pub cell: u64,
    pub lat: f64,
    pub lon: f64,
    /// Metres the position is thought good to. Large where the only reports
    /// were from a car going past.
    pub range_m: u32,
    pub samples: u32,
    /// Unix seconds at the last report, so a cell nobody has heard for two
    /// years can be told from one heard this week.
    pub updated: u64,
}

/// Every cell in the export, sorted by identity.
#[derive(Clone, Debug, Default)]
pub struct Cells(Vec<Cell>);

impl Cells {
    /// Where a decoded cell is, if the crowd has heard it.
    pub fn get(&self, mcc: u16, mnc: &str, area: u32, cell: u64) -> Option<&Cell> {
        let want = mnc.trim_start_matches('0');
        self.0
            .binary_search_by(|c| (c.mcc, c.area, c.cell).cmp(&(mcc, area, cell)))
            .ok()
            .map(|i| &self.0[i])
            .filter(|c| c.mnc.trim_start_matches('0') == want)
    }

    /// Every cell of one network in the export, for drawing a network's
    /// footprint rather than answering about one beacon.
    pub fn in_network(&self, mcc: u16, mnc: &str) -> impl Iterator<Item = &Cell> {
        let want = mnc.trim_start_matches('0').to_string();
        self.0.iter().filter(move |c| c.mcc == mcc && c.mnc.trim_start_matches('0') == want)
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &Cell> {
        self.0.iter()
    }
}

/// The export is gzipped CSV with a header row:
/// `radio,mcc,net,area,cell,unit,lon,lat,range,samples,changeable,created,updated,averageSignal`.
pub fn parse_towers(name: &str, gz: &[u8]) -> Result<Cells, Error> {
    // A bad token answers 200 with a JSON error body, not a 4xx, so the
    // cache stores that body as the dataset and the failure surfaces here.
    // Say what it was rather than reporting a gzip error about it.
    if let Some(msg) = token_error(gz) {
        return Err(Error::Parse(name.into(), msg));
    }
    let mut csv_bytes = Vec::new();
    flate2::read::GzDecoder::new(gz)
        .read_to_end(&mut csv_bytes)
        .map_err(|e| Error::Parse(name.into(), e.to_string()))?;
    let mut rdr = csv::ReaderBuilder::new().flexible(true).from_reader(&csv_bytes[..]);
    let mut out = Vec::new();
    for rec in rdr.records().flatten() {
        let f = |i: usize| rec.get(i).unwrap_or("").trim();
        let Ok(mcc) = f(1).parse::<u16>() else {
            continue;
        };
        let (Ok(lon), Ok(lat)) = (f(6).parse::<f64>(), f(7).parse::<f64>()) else {
            continue;
        };
        out.push(Cell {
            radio: f(0).to_string(),
            mcc,
            mnc: f(2).to_string(),
            area: f(3).parse().unwrap_or(0),
            cell: f(4).parse().unwrap_or(0),
            lat,
            lon,
            range_m: f(8).parse().unwrap_or(0),
            samples: f(9).parse().unwrap_or(0),
            updated: f(12).parse().unwrap_or(0),
        });
    }
    out.sort_by_key(|c| (c.mcc, c.area, c.cell));
    Ok(Cells(out))
}

/// The message from an OpenCelliD error body, or `None` when the bytes are
/// something else. Checked before gunzipping because the error is served
/// with the same 200 as the file.
fn token_error(raw: &[u8]) -> Option<String> {
    let head = raw.get(..256.min(raw.len()))?;
    if !head.starts_with(b"{") {
        return None;
    }
    // The body is small enough to be whole in the head the cache checks, and
    // a real export is gzip and never reaches here.
    let v: serde_json::Value = serde_json::from_slice(raw).ok()?;
    let msg = v.get("message")?.as_str()?;
    Some(match msg {
        "INVALID_TOKEN" => "the OpenCelliD token was rejected".to_string(),
        "RATE_LIMITED" => "OpenCelliD allows two downloads of a file a day".to_string(),
        other => other.to_string(),
    })
}

pub fn load_towers(cache: &Cache, mcc: u16, token: &str) -> Result<Cells, Error> {
    let src = towers_source(mcc, token);
    parse_towers(src.name, &cache.read(&src)?)
}

pub fn refresh_towers(
    cache: &Cache,
    mcc: u16,
    token: &str,
    when: When,
) -> Result<Option<Cells>, Error> {
    let src = towers_source(mcc, token);
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse_towers(src.name, &cache.read(&src)?).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"[
      {"type":"National","countryName":"United Kingdom","countryCode":"GB","mcc":"234","mnc":"15","brand":"Vodafone","operator":"Vodafone Ltd","status":"Operational","bands":"GSM 900 / LTE 800"},
      {"type":"National","countryName":"United Kingdom","countryCode":"GB","mcc":"234","mnc":"30","brand":"EE","operator":"EE Ltd","status":"Operational","bands":"GSM 1800"},
      {"type":"National","countryName":"Abkhazia","countryCode":"GE-AB","mcc":"289","mnc":"67","brand":"Aquafon","operator":"Aquafon JSC","status":"Operational","bands":""}
    ]"#;

    #[test]
    fn reads_a_network_by_the_pair_a_beacon_sends() {
        let ops = parse_operators(SAMPLE.as_bytes()).unwrap();
        assert_eq!(ops.get(234, "15").map(|o| o.brand.as_str()), Some("Vodafone"));
        // A beacon sends two digits where the file publishes one, and often
        // the other way round.
        assert_eq!(ops.get(234, "015").map(|o| o.brand.as_str()), Some("Vodafone"));
        assert_eq!(ops.get(234, "99"), None);
    }

    #[test]
    fn a_row_with_nulls_in_it_is_kept_without_them() {
        let raw = r#"[
          {"countryName":null,"countryCode":null,"mcc":"901","mnc":"01","brand":null,"operator":"ICO","status":null,"bands":null},
          {"countryName":"Nowhere","countryCode":"XX","mcc":null,"mnc":"1","brand":"None","operator":null,"status":null,"bands":null}
        ]"#;
        let ops = parse_operators(raw.as_bytes()).unwrap();
        // The row without an MCC has no identity to be found by and is
        // dropped; the one with nulls for its names is kept.
        assert_eq!(ops.len(), 1);
        let o = ops.get(901, "1").expect("row");
        assert_eq!(o.operator, "ICO");
        assert!(o.brand.is_empty() && o.iso.is_empty());
    }

    #[test]
    fn a_subdivision_code_is_not_a_country() {
        let ops = parse_operators(SAMPLE.as_bytes()).unwrap();
        assert_eq!(ops.mcc_for_country("GB"), Some(234));
        assert_eq!(ops.mcc_for_country("ge"), None);
    }

    #[test]
    fn a_rejected_token_is_reported_as_one() {
        let body = br#"{"status":"error","message":"INVALID_TOKEN"}"#;
        let e = parse_towers("opencellid-234.csv.gz", body).unwrap_err();
        assert!(e.to_string().contains("token"), "{e}");
    }

    #[test]
    fn reads_the_export_a_row_at_a_time() {
        let csv = "radio,mcc,net,area,cell,unit,lon,lat,range,samples,changeable,created,updated,averageSignal\n\
                   GSM,234,15,1234,5678,0,-0.1,51.5,1000,42,1,1500000000,1700000000,0\n\
                   LTE,234,30,4321,8765,0,-0.2,51.6,500,7,1,1500000000,1700000000,0\n";
        let mut gz = Vec::new();
        {
            use std::io::Write;
            let mut enc = flate2::write::GzEncoder::new(&mut gz, flate2::Compression::fast());
            enc.write_all(csv.as_bytes()).unwrap();
            enc.finish().unwrap();
        }
        let cells = parse_towers("opencellid-234.csv.gz", &gz).unwrap();
        assert_eq!(cells.len(), 2);
        let c = cells.get(234, "15", 1234, 5678).expect("cell");
        assert_eq!((c.lat, c.lon), (51.5, -0.1));
        assert_eq!(c.range_m, 1000);
        // The identity is the key, and the network code has to agree: two
        // networks in a country can use the same area and cell number.
        assert_eq!(cells.get(234, "30", 1234, 5678), None);
        assert_eq!(cells.in_network(234, "030").count(), 1);
    }
}
