use crate::model::{Dial, Entry, Hardware, Location, Station, Tuner, Version};
use common::geohash;
use nostr_sdk::prelude::Tag;

pub const SCHEME: &str = "iqstream://";
pub const GEOHASH_LADDER: usize = 5;
pub const HASHTAGS: [&str; 2] = ["sdr", "iqstream"];

pub fn encode(entry: &Entry) -> Vec<Tag> {
    let s = &entry.station;
    let mut tags = vec![Tag::custom("name", [s.name.as_str()])];
    if !s.description.is_empty() {
        tags.push(Tag::custom("description", [s.description.as_str()]));
    }
    tags.push(Tag::custom("r", [format!("{SCHEME}{}", entry.addr())]));
    if let Some(p) = entry.data_port {
        tags.push(Tag::custom("data_port", [p.to_string()]));
    }
    tags.push(Tag::custom("version", [format!("{}.{}", s.version.major, s.version.minor)]));
    tags.push(Tag::custom("clients", [s.clients.to_string()]));
    if let Some(max) = s.max_clients {
        tags.push(Tag::custom("max_clients", [max.to_string()]));
    }
    if let Some(secs) = s.session_limit_secs {
        tags.push(Tag::custom("session_limit", [secs.to_string()]));
    }
    if let Some(at) = s.location {
        let hash = geohash::encode(at.lat, at.lon, GEOHASH_LADDER);
        tags.extend((1..=hash.len()).map(|n| Tag::custom("g", [&hash[..n]])));
    }
    let mut hashtags: Vec<&str> = HASHTAGS.to_vec();
    for t in &s.tuners {
        let h = t.hardware.as_str();
        if !h.is_empty() && !hashtags.contains(&h) {
            hashtags.push(h);
        }
    }
    tags.extend(hashtags.into_iter().map(Tag::hashtag));
    tags.extend(s.tuners.iter().map(tuner));
    tags
}

fn tuner(t: &Tuner) -> Tag {
    let mut fields = vec![
        format!("id {}", t.id),
        format!("name {}", t.name),
        format!("type {}", t.hardware.as_str()),
    ];
    if !t.antenna.is_empty() {
        fields.push(format!("antenna {}", t.antenna));
    }
    fields.push(format!("center {}", t.center_hz));
    fields.push(format!("rate {}", t.sample_rate));
    match t.dial {
        Dial::Fixed => fields.push("tunable 0".into()),
        Dial::Tunable { min_hz, max_hz } => {
            fields.push("tunable 1".into());
            fields.extend(min_hz.map(|hz| format!("min {hz}")));
            fields.extend(max_hz.map(|hz| format!("max {hz}")));
        }
    }
    Tag::custom("tuner", fields)
}

pub fn decode<'a>(tags: impl Iterator<Item = &'a Tag> + Clone) -> Result<Entry, String> {
    let first =
        |key: &str| tags.clone().find(|t| t.kind() == key).and_then(|t| t.content()).map(str::trim);
    let number = |key: &str| -> Result<Option<u64>, String> {
        first(key)
            .map(|v| v.parse().map_err(|_| format!("{key} {v:?} is not a number")))
            .transpose()
    };
    let (host, port) = first("r")
        .and_then(|r| r.strip_prefix(SCHEME))
        .and_then(host_port)
        .ok_or("no iqstream:// address in an r tag")?;
    let version = first("version").and_then(version).ok_or("no protocol version")?;
    let location = tags
        .clone()
        .filter(|t| t.kind() == "g")
        .filter_map(|t| t.content())
        .max_by_key(|g| g.len())
        .and_then(geohash::decode)
        .map(|(lat, lon)| Location { lat, lon });
    let tuners = tags
        .clone()
        .filter(|t| t.kind() == "tuner")
        .map(|t| read_tuner(&t.as_slice()[1..]))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Entry {
        host,
        port,
        data_port: number("data_port")?.map(|p| p as u16),
        station: Station {
            name: first("name").unwrap_or_default().to_string(),
            description: first("description").unwrap_or_default().to_string(),
            location,
            version,
            clients: number("clients")?.unwrap_or(0) as u32,
            max_clients: number("max_clients")?.map(|m| m as u32),
            session_limit_secs: number("session_limit")?.map(|s| s as u32),
            tuners,
        },
    })
}

fn host_port(addr: &str) -> Option<(String, u16)> {
    let (host, port) = match addr.strip_prefix('[') {
        Some(v6) => v6.split_once("]:")?,
        None => addr.rsplit_once(':')?,
    };
    (!host.is_empty()).then_some(())?;
    Some((host.to_string(), port.parse().ok()?))
}

fn version(v: &str) -> Option<Version> {
    let (major, minor) = v.split_once('.')?;
    Some(Version { major: major.parse().ok()?, minor: minor.parse().ok()? })
}

fn read_tuner(fields: &[String]) -> Result<Tuner, String> {
    let get = |key: &str| {
        fields.iter().filter_map(|f| f.split_once(' ')).find(|(k, _)| *k == key).map(|(_, v)| v)
    };
    let num = |key: &str| -> Result<Option<u64>, String> {
        get(key)
            .map(|v| v.parse().map_err(|_| format!("tuner {key} {v:?} is not a number")))
            .transpose()
    };
    let need = |key: &str| num(key)?.ok_or(format!("tuner without {key}"));
    let dial = match get("tunable") {
        Some("1") => Dial::Tunable { min_hz: num("min")?, max_hz: num("max")? },
        _ => Dial::Fixed,
    };
    Ok(Tuner {
        id: need("id")? as u16,
        name: get("name").unwrap_or_default().to_string(),
        hardware: Hardware::from(get("type").ok_or("tuner without type")?),
        antenna: get("antenna").unwrap_or_default().to_string(),
        center_hz: need("center")?,
        sample_rate: need("rate")? as u32,
        dial,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::tests::{airband, entry, hf};

    fn wire(tags: &[Tag]) -> Vec<Vec<&str>> {
        tags.iter().map(|t| t.as_slice().iter().map(String::as_str).collect()).collect()
    }

    #[test]
    fn a_station_is_published_as_tags_with_one_tuner_tag_per_tuner() {
        let e =
            Entry { data_port: Some(40_001), ..entry("sdr.example.net", vec![airband(), hf()]) };
        let tags = encode(&e);
        let v = Version::OURS;
        let version = format!("{}.{}", v.major, v.minor);
        assert_eq!(
            wire(&tags),
            vec![
                vec!["name", "G0ABC"],
                vec!["description", "Loft, Reading"],
                vec!["r", "iqstream://sdr.example.net:5557"],
                vec!["data_port", "40001"],
                vec!["version", &version],
                vec!["clients", "1"],
                vec!["max_clients", "4"],
                vec!["g", "g"],
                vec!["g", "gc"],
                vec!["g", "gcp"],
                vec!["g", "gcpk"],
                vec!["g", "gcpk9"],
                vec!["t", "sdr"],
                vec!["t", "iqstream"],
                vec!["t", "rtlsdr"],
                vec!["t", "rx888"],
                vec![
                    "tuner",
                    "id 0",
                    "name Airband",
                    "type rtlsdr",
                    "antenna Discone",
                    "center 125000000",
                    "rate 2400000",
                    "tunable 0"
                ],
                vec![
                    "tuner",
                    "id 1",
                    "name HF",
                    "type rx888",
                    "center 7100000",
                    "rate 1000000",
                    "tunable 1",
                    "min 100000",
                    "max 30000000"
                ],
            ]
        );
    }

    #[test]
    fn the_tags_read_back_to_the_station_with_its_location_to_the_finest_geohash() {
        let e = Entry { data_port: Some(40_001), ..entry("2001:db8::7", vec![airband(), hf()]) };
        let back = decode(encode(&e).iter()).unwrap();
        let at = back.station.location.unwrap();
        assert!((at.lat - 51.45).abs() < 0.022 && (at.lon - -0.97).abs() < 0.022, "{at:?}");
        let exact =
            Entry { station: Station { location: e.station.location, ..back.station }, ..back };
        assert_eq!(exact, e);
        assert_eq!(exact.addr(), "[2001:db8::7]:5557");
    }

    fn parsed(tags: &[&[&str]]) -> Result<Entry, String> {
        let tags: Vec<Tag> = tags.iter().map(|t| Tag::parse(t.iter().copied()).unwrap()).collect();
        decode(tags.iter())
    }

    #[test]
    fn a_tuner_tag_ignores_fields_it_does_not_know_and_names_what_is_missing() {
        let base: [&[&str]; 2] = [&["r", "iqstream://a.example:5557"], &["version", "1.9"]];
        let with = |tuner: &'static [&'static str]| {
            let mut t = base.to_vec();
            t.push(tuner);
            parsed(&t)
        };
        let e =
            with(&["tuner", "id 3", "type hackrf", "center 1", "rate 2", "gain 30", "tunable 1"])
                .unwrap();
        let t = &e.station.tuners[0];
        assert_eq!((t.id, &t.hardware, t.name.as_str()), (3, &Hardware::HackRf, ""));
        assert_eq!(t.dial, Dial::Tunable { min_hz: None, max_hz: None });
        assert_eq!(
            with(&["tuner", "id 0", "type hackrf", "rate 2"]).unwrap_err(),
            "tuner without center"
        );
        assert_eq!(
            with(&["tuner", "id 0", "type hackrf", "center x", "rate 2"]).unwrap_err(),
            "tuner center \"x\" is not a number"
        );
    }

    #[test]
    fn a_station_without_an_address_or_a_version_is_not_one() {
        assert_eq!(
            parsed(&[&["version", "1.4"]]).unwrap_err(),
            "no iqstream:// address in an r tag"
        );
        assert!(parsed(&[&["r", "https://a.example:5557"], &["version", "1.4"]]).is_err());
        assert!(parsed(&[&["r", "iqstream://a.example"], &["version", "1.4"]]).is_err());
        assert_eq!(parsed(&[&["r", "iqstream://a.example:1"]]).unwrap_err(), "no protocol version");
        let bare = parsed(&[&["r", "iqstream://a.example:1"], &["version", "1.4"]]).unwrap();
        assert_eq!(
            (bare.station.tuners.len(), bare.station.location, bare.data_port),
            (0, None, None)
        );
    }
}
