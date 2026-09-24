use crate::cache::{Cache, Error, Source, When};
use std::collections::HashMap;
use std::io::BufRead;
use std::path::Path;
use std::time::Duration;

const MAX_AGE: Duration = Duration::from_secs(24 * 3600);

pub fn source() -> Source {
    Source::http(
        "tar1090-aircraft.csv.gz",
        "https://raw.githubusercontent.com/wiedehopf/tar1090-db/csv/aircraft.csv.gz",
        MAX_AGE,
    )
    .checked(gzipped)
}

pub fn types_source() -> Source {
    Source::http(
        "tar1090-aircraft-types.json.gz",
        "https://raw.githubusercontent.com/wiedehopf/tar1090-db/master/db/icao_aircraft_types2.js",
        MAX_AGE,
    )
    .checked(gzipped)
}

fn gzipped(head: &[u8]) -> Result<(), String> {
    match head.starts_with(&[0x1f, 0x8b]) {
        true => Ok(()),
        false => Err("not a gzip file".into()),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Aircraft<'a> {
    pub icao: u32,
    pub registration: &'a str,
    pub type_code: &'a str,
    pub description: &'a str,
    pub year: Option<u16>,
    pub owner: &'a str,
    pub military: bool,
    pub class: Option<Class>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Class {
    Heavy,
    Jet,
    Twin,
    Single,
    Rotorcraft,
    Glider,
    Balloon,
}

impl Class {
    pub fn from_icao(designator: &str, description: &str, wake: &str) -> Option<Self> {
        if designator == "GLID" {
            return Some(Class::Glider);
        }
        let mut d = description.chars();
        let (body, engines, power) = (d.next()?, d.next()?.to_digit(10)?, d.next()?);
        let heavy = matches!(wake, "H" | "J");
        match (body, power) {
            ('H' | 'G', _) => Some(Class::Rotorcraft),
            ('B', _) => Some(Class::Balloon),
            ('L' | 'S' | 'A' | 'R', 'J') if heavy => Some(Class::Heavy),
            ('L' | 'S' | 'A' | 'R', 'J') => Some(Class::Jet),
            ('R', _) => Some(Class::Twin),
            ('L' | 'S' | 'A', _) if engines >= 2 => Some(Class::Twin),
            ('L' | 'S' | 'A', _) => Some(Class::Single),
            _ => None,
        }
    }
}

impl Aircraft<'_> {
    pub fn summary(&self) -> String {
        [self.registration, self.type_code]
            .into_iter()
            .filter(|s| !s.is_empty())
            .collect::<Vec<_>>()
            .join(" ")
    }
}

#[derive(Clone, Debug, Default)]
pub struct Fleet {
    icao: Vec<u32>,
    start: Vec<u32>,
    text: String,
    types: HashMap<String, Class>,
}

impl Fleet {
    pub fn with_types(mut self, types: HashMap<String, Class>) -> Self {
        self.types = types;
        self
    }

    pub fn get(&self, icao: u32) -> Option<Aircraft<'_>> {
        let i = self.icao.binary_search(&icao).ok()?;
        let end = self.start.get(i + 1).map_or(self.text.len(), |e| *e as usize);
        let mut f = self.text[self.start[i] as usize..end].split(';');
        let mut next = || f.next().unwrap_or("");
        let (registration, type_code) = (next(), next());
        Some(Aircraft {
            icao,
            registration,
            type_code,
            military: next().starts_with('1'),
            description: next(),
            year: next().parse().ok(),
            owner: next(),
            class: self.types.get(type_code).copied(),
        })
    }

    pub fn len(&self) -> usize {
        self.icao.len()
    }

    pub fn is_empty(&self) -> bool {
        self.icao.is_empty()
    }
}

pub fn load(cache: &Cache) -> Result<Fleet, Error> {
    let fleet = parse(&cache.get(&source())?)?;
    Ok(fleet.with_types(parse_types(&cache.get(&types_source())?)?))
}

pub fn refresh(cache: &Cache, when: When) -> Result<Option<Fleet>, Error> {
    let changed =
        cache.refresh(&source(), when)?.is_some() | cache.refresh(&types_source(), when)?.is_some();
    match changed {
        false => Ok(None),
        true => load(cache).map(Some),
    }
}

fn parse_types(path: &Path) -> Result<HashMap<String, Class>, Error> {
    let name = path.display().to_string();
    let f = std::fs::File::open(path).map_err(|e| Error::Io(name.clone(), e))?;
    read_types(&name, flate2::read::GzDecoder::new(f))
}

pub fn read_types(name: &str, json: impl std::io::Read) -> Result<HashMap<String, Class>, Error> {
    let rows: HashMap<String, Vec<String>> = serde_json::from_reader(std::io::BufReader::new(json))
        .map_err(|e| Error::Parse(name.into(), e.to_string()))?;
    let types: HashMap<String, Class> = rows
        .into_iter()
        .filter_map(|(t, r)| {
            let field = |i: usize| r.get(i).map_or("", String::as_str);
            let class = Class::from_icao(&t, field(1), field(2))?;
            Some((t, class))
        })
        .collect();
    if types.is_empty() {
        return Err(Error::Parse(name.into(), "no aircraft types in the file".into()));
    }
    Ok(types)
}

fn parse(path: &Path) -> Result<Fleet, Error> {
    let name = path.display().to_string();
    let f = std::fs::File::open(path).map_err(|e| Error::Io(name.clone(), e))?;
    read(&name, flate2::read::GzDecoder::new(f))
}

pub fn read(name: &str, csv: impl std::io::Read) -> Result<Fleet, Error> {
    let mut rows: Vec<(u32, String)> = Vec::new();
    for line in std::io::BufReader::with_capacity(1 << 20, csv).lines() {
        let line = line.map_err(|e| Error::Parse(name.into(), e.to_string()))?;
        let Some((hex, rest)) = line.split_once(';') else { continue };
        let Ok(icao) = u32::from_str_radix(hex.trim(), 16) else { continue };
        let rest = rest.trim_end_matches(';');
        let f: Vec<&str> = rest.splitn(5, ';').collect();
        let blank = |i: usize| f.get(i).is_none_or(|s| s.trim().is_empty());
        if blank(0) && blank(1) && blank(3) {
            continue;
        }
        rows.push((icao, rest.to_string()));
    }
    if rows.is_empty() {
        return Err(Error::Parse(name.into(), "no aircraft in the file".into()));
    }
    rows.sort_by_key(|r| r.0);
    rows.dedup_by_key(|r| r.0);
    let mut fleet = Fleet {
        icao: Vec::with_capacity(rows.len()),
        start: Vec::with_capacity(rows.len()),
        text: String::with_capacity(rows.iter().map(|r| r.1.len()).sum()),
        types: HashMap::new(),
    };
    for (icao, rest) in rows {
        fleet.icao.push(icao);
        fleet.start.push(fleet.text.len() as u32);
        fleet.text.push_str(&rest);
    }
    Ok(fleet)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "\
000001;;;10;;;Miscode - VARIOUS;
4CA068;EI-CJX;B752;00;BOEING 757-200;;;
A00F5E;N1026T;BE35;00;BEECH 35 Bonanza;1976;EDENS ERIK;
43C6F3;ZZ177;C17;10;BOEING C-17 Globemaster III;;Royal Air Force;
45E28F;OY-XTO;SF25;00;SCHEIBE SF-25 Falke;;;
nothex;X;Y;00;;;;
4CA068;EI-DUP;A320;00;;;;
";

    const TYPES: &str = r#"{"B752":["BOEING 757-200","L2J","M"],"BE35":["BEECH 35 Bonanza","L1P","L"],"C17":["BOEING C-17","L4J","H"],"SF25":["SCHEIBE SF-25 Falke","L1P","L"],"ZZZZ":["","-0-","-"]}"#;

    fn fleet() -> Fleet {
        let types = read_types("types", TYPES.as_bytes()).unwrap();
        read("sample", SAMPLE.as_bytes()).unwrap().with_types(types)
    }

    fn class(designator: &str, description: &str, wake: &str) -> Option<Class> {
        Class::from_icao(designator, description, wake)
    }

    #[test]
    fn icao_type_descriptions_pick_the_shape_tar1090_icao_aircraft_types2_gives() {
        assert_eq!(class("R44", "H1P", "L"), Some(Class::Rotorcraft));
        assert_eq!(class("EC35", "H2T", "L"), Some(Class::Rotorcraft));
        assert_eq!(class("GYRO", "G0-", "-"), Some(Class::Rotorcraft));
        assert_eq!(class("C172", "L1P", "L"), Some(Class::Single));
        assert_eq!(class("PC12", "L1T", "L"), Some(Class::Single));
        assert_eq!(class("ULAC", "L0-", "-"), Some(Class::Single));
        assert_eq!(class("BE58", "L2P", "L"), Some(Class::Twin));
        assert_eq!(class("DH8D", "L2T", "M"), Some(Class::Twin));
        assert_eq!(class("V22", "R2T", "M"), Some(Class::Twin));
        assert_eq!(class("B738", "L2J", "M"), Some(Class::Jet));
        assert_eq!(class("C510", "L2J", "L"), Some(Class::Jet));
        assert_eq!(class("B77W", "L2J", "H"), Some(Class::Heavy));
        assert_eq!(class("A388", "L4J", "J"), Some(Class::Heavy));
        assert_eq!(class("GLID", "L0-", "-"), Some(Class::Glider));
        assert_eq!(class("BALL", "B0-", "-"), Some(Class::Balloon));
        assert_eq!(class("DRON", "D0-", "-"), None);
        assert_eq!(class("GND", "V0-", "-"), None);
        assert_eq!(class("ZZZZ", "-0-", "-"), None);
    }

    #[test]
    fn four_of_five_types_are_kept_and_joined_to_the_fleet() {
        assert_eq!(read_types("types", TYPES.as_bytes()).unwrap().len(), 4);
        let f = fleet();
        assert_eq!(f.get(0x4CA068).unwrap().class, Some(Class::Jet));
        assert_eq!(f.get(0x43C6F3).unwrap().class, Some(Class::Heavy));
        let bare = read("sample", SAMPLE.as_bytes()).unwrap();
        assert_eq!(bare.get(0x4CA068).unwrap().class, None);
    }

    #[test]
    fn four_of_seven_rows_are_kept() {
        let f = fleet();
        assert_eq!(f.len(), 4, "the empty row, the bad hex and the duplicate are dropped");
        assert!(f.get(0x000001).is_none());
    }

    #[test]
    fn an_address_reads_back_every_field() {
        let f = fleet();
        assert_eq!(
            f.get(0xA00F5E),
            Some(Aircraft {
                icao: 0xA00F5E,
                registration: "N1026T",
                type_code: "BE35",
                description: "BEECH 35 Bonanza",
                year: Some(1976),
                owner: "EDENS ERIK",
                military: false,
                class: Some(Class::Single),
            })
        );
        let c17 = f.get(0x43C6F3).unwrap();
        assert!(c17.military);
        assert_eq!(c17.owner, "Royal Air Force");
        assert_eq!(c17.year, None);
    }

    #[test]
    fn the_first_of_a_duplicated_address_wins() {
        assert_eq!(fleet().get(0x4CA068).unwrap().registration, "EI-CJX");
    }

    #[test]
    fn the_summary_is_registration_then_type() {
        let f = fleet();
        assert_eq!(f.get(0x4CA068).unwrap().summary(), "EI-CJX B752");
        assert_eq!(f.get(0x45E28F).unwrap().summary(), "OY-XTO SF25");
        assert!(f.get(0x123456).is_none());
    }

    #[test]
    fn a_file_with_no_aircraft_is_an_error() {
        assert!(read("empty", "000001;;;10;;;;\n".as_bytes()).is_err());
    }

    #[test]
    fn a_refusal_body_is_not_taken_for_the_file() {
        assert!(gzipped(b"404: Not Found").is_err());
        assert!(gzipped(&[0x1f, 0x8b, 8, 0]).is_ok());
    }
}
