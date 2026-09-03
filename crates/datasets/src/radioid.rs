//! The radioid.net dumps: DMR repeaters, DMR user IDs and NXDN IDs.
//!
//! A digital voice frame carries a number, not a callsign. radioid.net is the
//! registry those numbers are issued from and publishes the whole registry,
//! so a decoded ID becomes a callsign, a name and a town with no lookup over
//! the network per transmission and nothing leaking about what is being
//! listened to.
//!
//! The user dump is 85 MB of JSON, which is why [`Users`] is a sorted array
//! of records searched by ID rather than a map: the map costs a hashed
//! allocation per entry for a table that is built once, read often and never
//! changed.

use crate::cache::{Cache, Error, Source, When};
use std::path::Path;
use std::time::Duration;

/// The dumps are rebuilt daily. Checking once a day costs one conditional
/// request per file and the file is unchanged nearly every time.
const MAX_AGE: Duration = Duration::from_secs(24 * 3600);

pub fn repeaters_source() -> Source {
    Source::http("radioid-rptrs.json", "https://radioid.net/static/rptrs.json", MAX_AGE)
}

pub fn users_source() -> Source {
    Source::http("radioid-users.json", "https://radioid.net/static/users.json", MAX_AGE)
}

pub fn nxdn_source() -> Source {
    Source::http("radioid-nxdn.csv", "https://radioid.net/static/nxdn.csv", MAX_AGE)
}

/// A registered radio: what the ID on the air belongs to.
#[derive(Clone, Debug, PartialEq)]
pub struct User {
    pub id: u32,
    pub callsign: String,
    pub name: String,
    pub city: String,
    pub state: String,
    pub country: String,
}

/// Every registered ID, sorted, for lookup by the number in a decoded frame.
#[derive(Clone, Debug, Default)]
pub struct Users(Vec<User>);

impl Users {
    pub fn get(&self, id: u32) -> Option<&User> {
        self.0.binary_search_by_key(&id, |u| u.id).ok().map(|i| &self.0[i])
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = &User> {
        self.0.iter()
    }

    fn from_rows(mut rows: Vec<User>) -> Self {
        rows.sort_by_key(|u| u.id);
        // An ID appears once in a sane dump, but a duplicate would make the
        // binary search return either of them, so the first wins explicitly.
        rows.dedup_by_key(|u| u.id);
        Self(rows)
    }
}

/// A DMR repeater, as the registry describes it.
#[derive(Clone, Debug, PartialEq)]
pub struct Repeater {
    pub id: u32,
    pub callsign: String,
    /// Output frequency in Hz. Published as a string in MHz, and occasionally
    /// blank, so a repeater without one is kept without a frequency rather
    /// than dropped.
    pub freq_hz: Option<f64>,
    /// Input offset in Hz, signed. `+5.000` is 5 MHz up.
    pub offset_hz: Option<f64>,
    /// Colour code, 0 to 15. The registry has rows outside that range (one
    /// says 293), which are a typed-in number rather than a colour code, so
    /// they are recorded as unknown rather than clamped into a wrong answer.
    pub color_code: Option<u8>,
    pub city: String,
    pub state: String,
    pub country: String,
    pub trustee: String,
    pub network: String,
    pub active: bool,
}

pub fn load_repeaters(cache: &Cache) -> Result<Vec<Repeater>, Error> {
    parse_repeaters(&cache.read(&repeaters_source())?)
}

pub fn refresh_repeaters(cache: &Cache, when: When) -> Result<Option<Vec<Repeater>>, Error> {
    let src = repeaters_source();
    match cache.refresh(&src, when)? {
        None => Ok(None),
        Some(_) => parse_repeaters(&cache.read(&src)?).map(Some),
    }
}

/// The user dump is read from the file rather than a buffer: 85 MB of JSON
/// and the records parsed out of it do not both need to be in memory.
pub fn load_users(cache: &Cache) -> Result<Users, Error> {
    parse_users(&cache.get(&users_source())?)
}

pub fn refresh_users(cache: &Cache, when: When) -> Result<Option<Users>, Error> {
    match cache.refresh(&users_source(), when)? {
        None => Ok(None),
        Some(path) => parse_users(&path).map(Some),
    }
}

pub fn load_nxdn(cache: &Cache) -> Result<Users, Error> {
    parse_nxdn(&cache.read(&nxdn_source())?)
}

pub fn refresh_nxdn(cache: &Cache, when: When) -> Result<Option<Users>, Error> {
    let src = nxdn_source();
    match cache.refresh(&src, when)? {
        None => Ok(None),
        Some(_) => parse_nxdn(&cache.read(&src)?).map(Some),
    }
}

mod wire {
    //! The published shapes, named as the dumps name them. Every field is
    //! optional or defaulted: the registry has rows with a missing town and
    //! rows where a number is quoted, and one of those must not cost the
    //! other 300 000 rows.

    #[derive(serde::Deserialize)]
    pub struct Users {
        #[serde(default)]
        pub users: Vec<User>,
    }

    #[derive(serde::Deserialize)]
    pub struct User {
        #[serde(default)]
        pub id: u32,
        #[serde(default)]
        pub callsign: String,
        #[serde(default)]
        pub fname: String,
        #[serde(default)]
        pub surname: String,
        #[serde(default)]
        pub city: String,
        #[serde(default)]
        pub state: String,
        #[serde(default)]
        pub country: String,
    }

    #[derive(serde::Deserialize)]
    pub struct Repeaters {
        #[serde(default)]
        pub rptrs: Vec<Repeater>,
    }

    #[derive(serde::Deserialize)]
    pub struct Repeater {
        #[serde(default)]
        pub id: u32,
        #[serde(default)]
        pub callsign: String,
        #[serde(default)]
        pub frequency: String,
        #[serde(default)]
        pub offset: String,
        /// Read as whatever it happens to be: the registry does not police
        /// this field, and a row holding 293 or a quoted digit must not cost
        /// the other 60 000 rows their parse.
        #[serde(default)]
        pub color_code: serde_json::Value,
        #[serde(default)]
        pub city: String,
        #[serde(default)]
        pub state: String,
        #[serde(default)]
        pub country: String,
        #[serde(default)]
        pub trustee: Vec<String>,
        #[serde(default)]
        pub ipsc_network: String,
        #[serde(default)]
        pub status: String,
    }
}

fn parse_users(path: &Path) -> Result<Users, Error> {
    let name = || path.display().to_string();
    let f = std::fs::File::open(path).map_err(|e| Error::Io(name(), e))?;
    let doc: wire::Users = serde_json::from_reader(std::io::BufReader::with_capacity(1 << 20, f))
        .map_err(|e| Error::Parse(name(), e.to_string()))?;
    if doc.users.is_empty() {
        return Err(Error::Parse(name(), "no users in the dump".into()));
    }
    Ok(Users::from_rows(doc.users.into_iter().map(user).collect()))
}

fn user(u: wire::User) -> User {
    let name = match (u.fname.trim(), u.surname.trim()) {
        (f, "") => f.to_string(),
        ("", s) => s.to_string(),
        (f, s) => format!("{f} {s}"),
    };
    User { id: u.id, callsign: u.callsign, name, city: u.city, state: u.state, country: u.country }
}

fn parse_repeaters(json: &[u8]) -> Result<Vec<Repeater>, Error> {
    let doc: wire::Repeaters = serde_json::from_slice(json)
        .map_err(|e| Error::Parse("rptrs.json".into(), e.to_string()))?;
    if doc.rptrs.is_empty() {
        return Err(Error::Parse("rptrs.json".into(), "no repeaters in the dump".into()));
    }
    Ok(doc
        .rptrs
        .into_iter()
        .map(|r| Repeater {
            id: r.id,
            callsign: r.callsign,
            freq_hz: mhz_to_hz(&r.frequency),
            offset_hz: mhz_to_hz(&r.offset),
            color_code: color_code(&r.color_code),
            city: r.city,
            state: r.state,
            country: r.country,
            trustee: r.trustee.first().cloned().unwrap_or_default(),
            network: r.ipsc_network,
            // Anything the registry has not marked ACTIVE is treated as not
            // on the air, which is the safer way round for a scanner.
            active: r.status.eq_ignore_ascii_case("active"),
        })
        .collect())
}

/// A DMR colour code is four bits. Anything else in the field is not one.
fn color_code(v: &serde_json::Value) -> Option<u8> {
    let n = v.as_u64().or_else(|| v.as_str()?.trim().parse().ok())?;
    (n <= 15).then_some(n as u8)
}

/// The dumps publish frequencies as decimal MHz in a string, sometimes empty
/// and sometimes signed. Hz is what the rest of the receiver speaks.
fn mhz_to_hz(s: &str) -> Option<f64> {
    let t = s.trim();
    if t.is_empty() {
        return None;
    }
    t.parse::<f64>().ok().map(|m| m * 1e6)
}

fn parse_nxdn(csv_bytes: &[u8]) -> Result<Users, Error> {
    let file = "nxdn.csv";
    let mut rows = csv::ReaderBuilder::new().flexible(true).from_reader(csv_bytes);
    let head = rows.headers().map_err(|e| Error::Parse(file.into(), e.to_string()))?.clone();
    let need = |n: &str| {
        head.iter()
            .position(|c| c.eq_ignore_ascii_case(n))
            .ok_or_else(|| Error::Parse(file.into(), format!("no {n} column")))
    };
    let (ci, cc, cf, cl, ct, cs, cn) = (
        need("RADIO_ID")?,
        need("CALLSIGN")?,
        need("FIRST_NAME")?,
        need("LAST_NAME")?,
        need("CITY")?,
        need("STATE")?,
        need("COUNTRY")?,
    );
    let mut out = Vec::new();
    let mut row = csv::StringRecord::new();
    let at = |row: &csv::StringRecord, i: usize| row.get(i).unwrap_or("").trim().to_string();
    while rows.read_record(&mut row).map_err(|e| Error::Parse(file.into(), e.to_string()))? {
        let Some(Ok(id)) = row.get(ci).map(|v| v.trim().parse()) else { continue };
        let name = match (at(&row, cf), at(&row, cl)) {
            (f, l) if l.is_empty() => f,
            (f, l) if f.is_empty() => l,
            (f, l) => format!("{f} {l}"),
        };
        out.push(User {
            id,
            callsign: at(&row, cc),
            name,
            city: at(&row, ct),
            state: at(&row, cs),
            country: at(&row, cn),
        });
    }
    if out.is_empty() {
        return Err(Error::Parse(file.into(), "no IDs in the dump".into()));
    }
    Ok(Users::from_rows(out))
}

#[cfg(test)]
mod tests {
    use super::*;

    const RPTRS: &str = r#"{"rptrs":[
        {"id":2723001,"callsign":"EI7MRD","city":"Dublin","state":"","country":"Ireland",
         "frequency":"438.62500","color_code":1,"offset":"-7.600","ipsc_network":"Brandmeister",
         "trustee":["EI2GYB"],"status":"ACTIVE"},
        {"id":2723002,"callsign":"EI0OFF","city":"Cork","state":"","country":"Ireland",
         "frequency":"","color_code":0,"offset":"","ipsc_network":"","trustee":[],
         "status":"Inactive"},
        {"id":2723003,"callsign":"EI9JUNK","frequency":"145.7875","color_code":293,
         "offset":"-0.600","status":"ACTIVE"}]}"#;

    const NXDN: &str = "RADIO_ID,CALLSIGN,FIRST_NAME,LAST_NAME,CITY,STATE,COUNTRY\n\
1,KB3AWQ,John,,Williamsport,Pennsylvania,United States\n\
2,W2FLY,Harry J,Smith,Mullica Hill,New Jersey,United States\n\
oops,BADROW,x,y,z,w,v\n";

    #[test]
    fn a_repeater_frequency_and_offset_come_out_in_hz() {
        let r = parse_repeaters(RPTRS.as_bytes()).unwrap();
        assert_eq!(r[0].callsign, "EI7MRD");
        assert_eq!(r[0].freq_hz, Some(438_625_000.0));
        assert_eq!(r[0].offset_hz, Some(-7_600_000.0));
        assert_eq!(r[0].trustee, "EI2GYB");
        assert_eq!(r[0].color_code, Some(1));
        assert!(r[0].active);
    }

    #[test]
    fn a_repeater_without_a_frequency_is_kept_without_one() {
        let r = parse_repeaters(RPTRS.as_bytes()).unwrap();
        assert_eq!(r[1].freq_hz, None);
        assert_eq!(r[1].offset_hz, None);
        assert!(!r[1].active, "anything but ACTIVE is off the air");
    }

    #[test]
    fn a_colour_code_outside_four_bits_is_unknown_and_not_fatal() {
        // The live dump has a row saying 293, which took the whole file with
        // it when the field was read as a byte.
        let r = parse_repeaters(RPTRS.as_bytes()).unwrap();
        assert_eq!(r.len(), 3);
        assert_eq!(r[2].color_code, None);
        assert_eq!(r[2].freq_hz, Some(145_787_500.0));
    }

    #[test]
    fn an_empty_dump_is_an_error_rather_than_an_empty_registry() {
        assert!(parse_repeaters(br#"{"rptrs":[]}"#).is_err());
        assert!(parse_nxdn(b"RADIO_ID,CALLSIGN,FIRST_NAME,LAST_NAME,CITY,STATE,COUNTRY\n").is_err());
    }

    #[test]
    fn an_nxdn_id_looks_up_to_its_callsign_and_name() {
        let u = parse_nxdn(NXDN.as_bytes()).unwrap();
        assert_eq!(u.len(), 2, "the unparseable row is skipped, not fatal");
        let one = u.get(1).expect("ID 1");
        assert_eq!(one.callsign, "KB3AWQ");
        assert_eq!(one.name, "John", "a missing surname leaves no trailing space");
        assert_eq!(u.get(2).unwrap().name, "Harry J Smith");
        assert!(u.get(99).is_none());
    }

    #[test]
    fn users_are_searchable_whatever_order_the_dump_was_in() {
        let u = Users::from_rows(vec![
            User { id: 30, callsign: "C".into(), ..blank() },
            User { id: 10, callsign: "A".into(), ..blank() },
            User { id: 20, callsign: "B".into(), ..blank() },
            User { id: 10, callsign: "duplicate".into(), ..blank() },
        ]);
        assert_eq!(u.len(), 3);
        assert_eq!(u.get(10).unwrap().callsign, "A");
        assert_eq!(u.get(30).unwrap().callsign, "C");
        assert!(u.get(15).is_none());
    }

    fn blank() -> User {
        User {
            id: 0,
            callsign: String::new(),
            name: String::new(),
            city: String::new(),
            state: String::new(),
            country: String::new(),
        }
    }
}
