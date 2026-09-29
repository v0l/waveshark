use common::geohash;
use sdr_directory::{
    Dial, Entry, Hardware, Location, Protocol, Station, Tuner, Version, WebTransport,
};

pub const SCHEME: &str = "iqstream://";
pub const GEOHASH_LADDER: usize = 5;
pub const HASHTAGS: [&str; 2] = ["sdr", "iqstream"];

pub type Tag = Vec<String>;

pub fn tag<S: Into<String>>(key: &str, values: impl IntoIterator<Item = S>) -> Tag {
    std::iter::once(key.to_string()).chain(values.into_iter().map(Into::into)).collect()
}

fn key(t: &Tag) -> &str {
    t.first().map_or("", String::as_str)
}

fn value(t: &Tag) -> Option<&str> {
    t.get(1).map(String::as_str)
}

pub fn encode(entry: &Entry) -> Vec<Tag> {
    let s = &entry.station;
    let mut tags = vec![tag("name", [s.name.as_str()])];
    if !s.description.is_empty() {
        tags.push(tag("description", [s.description.as_str()]));
    }
    tags.push(tag("r", [format!("{SCHEME}{}", entry.addr())]));
    tags.extend(entry.also.iter().map(|a| tag("r", [format!("{SCHEME}{a}")])));
    if let Some(p) = entry.data_port {
        tags.push(tag("data_port", [p.to_string()]));
    }
    if let Some(wt) = &entry.webtransport {
        let values = std::iter::once(wt.port.to_string()).chain(wt.hashes.iter().cloned());
        tags.push(tag("webtransport", values));
    }
    if entry.webrtc {
        tags.push(tag("webrtc", [crate::event::SIGNAL.to_string()]));
    }
    if let Protocol::IqStream(v) = s.protocol {
        tags.push(tag("version", [format!("{}.{}", v.major, v.minor)]));
    }
    tags.push(tag("clients", [s.clients.to_string()]));
    if let Some(max) = s.max_clients {
        tags.push(tag("max_clients", [max.to_string()]));
    }
    if let Some(secs) = s.session_limit_secs {
        tags.push(tag("session_limit", [secs.to_string()]));
    }
    if let Some(at) = s.location {
        let hash = geohash::encode(at.lat, at.lon, at.geohash_len);
        tags.extend((1..=hash.len()).map(|n| tag("g", [&hash[..n]])));
    }
    let mut hashtags: Vec<&str> = HASHTAGS.to_vec();
    for t in &s.tuners {
        let h = t.hardware.as_str();
        if !h.is_empty() && !hashtags.contains(&h) {
            hashtags.push(h);
        }
    }
    tags.extend(hashtags.into_iter().map(|h| tag("t", [h])));
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
    tag("tuner", fields)
}

pub fn decode<'a>(tags: impl Iterator<Item = &'a Tag> + Clone) -> Result<Entry, String> {
    let first =
        |key: &str| tags.clone().find(|t| self::key(t) == key).and_then(value).map(str::trim);
    let number = |key: &str| -> Result<Option<u64>, String> {
        first(key)
            .map(|v| v.parse().map_err(|_| format!("{key} {v:?} is not a number")))
            .transpose()
    };
    let mut addrs = tags
        .clone()
        .filter(|t| key(t) == "r")
        .filter_map(value)
        .filter_map(|r| r.trim().strip_prefix(SCHEME))
        .filter_map(host_port);
    let (host, port) = addrs.next().ok_or("no iqstream:// address in an r tag")?;
    let also: Vec<std::net::SocketAddr> =
        addrs.filter_map(|(h, p)| Some(std::net::SocketAddr::new(h.parse().ok()?, p))).collect();
    let version = first("version").and_then(version).ok_or("no protocol version")?;
    let location =
        tags.clone().filter(|t| key(t) == "g").filter_map(value).max_by_key(|g| g.len()).and_then(
            |g| {
                let (lat, lon) = geohash::decode(g)?;
                Some(Location { lat, lon, geohash_len: g.len() })
            },
        );
    let tuners = tags
        .clone()
        .filter(|t| key(t) == "tuner")
        .map(|t| read_tuner(&t[1..]))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Entry {
        host,
        port,
        data_port: number("data_port")?.map(|p| p as u16),
        also,
        webtransport: tags.clone().find(|t| key(t) == "webtransport").and_then(webtransport),
        webrtc: tags.clone().any(|t| key(t) == "webrtc"),
        station: Station {
            name: first("name").unwrap_or_default().to_string(),
            description: first("description").unwrap_or_default().to_string(),
            location,
            protocol: Protocol::IqStream(version),
            clients: number("clients")?.unwrap_or(0) as u32,
            max_clients: number("max_clients")?.map(|m| m as u32),
            session_limit_secs: number("session_limit")?.map(|s| s as u32),
            tuners,
        },
    })
}

fn webtransport(t: &Tag) -> Option<WebTransport> {
    let port = value(t)?.trim().parse().ok()?;
    let hashes: Vec<String> = t[2..]
        .iter()
        .map(|h| h.trim().to_ascii_lowercase())
        .filter(|h| h.len() == 64 && h.bytes().all(|b| b.is_ascii_hexdigit()))
        .collect();
    (!hashes.is_empty()).then_some(WebTransport { port, hashes })
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
    use sdr_directory::model::fixtures::{airband, entry, hf};

    fn wire(tags: &[Tag]) -> Vec<Vec<&str>> {
        tags.iter().map(|t| t.iter().map(String::as_str).collect()).collect()
    }

    #[test]
    fn a_station_is_published_as_tags_with_one_tuner_tag_per_tuner() {
        let e = Entry {
            data_port: Some(40_001),
            also: vec!["10.100.2.249:1234".parse().unwrap()],
            ..entry("sdr.example.net", vec![airband(), hf()])
        };
        let tags = encode(&e);
        let v = Version::OURS;
        let version = format!("{}.{}", v.major, v.minor);
        assert_eq!(
            wire(&tags),
            vec![
                vec!["name", "G0ABC"],
                vec!["description", "Loft, Reading"],
                vec!["r", "iqstream://sdr.example.net:5557"],
                vec!["r", "iqstream://10.100.2.249:1234"],
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
        let e = Entry {
            data_port: Some(40_001),
            also: vec!["[fd00::7]:1234".parse().unwrap(), "10.0.0.7:1234".parse().unwrap()],
            webtransport: Some(WebTransport {
                port: 40_002,
                hashes: vec!["ab".repeat(32), "cd".repeat(32)],
            }),
            webrtc: true,
            ..entry("2001:db8::7", vec![airband(), hf()])
        };
        let back = decode(encode(&e).iter()).unwrap();
        let at = back.station.location.unwrap();
        assert!((at.lat - 51.45).abs() < 0.022 && (at.lon - -0.97).abs() < 0.022, "{at:?}");
        let exact =
            Entry { station: Station { location: e.station.location, ..back.station }, ..back };
        assert_eq!(exact, e);
        assert_eq!(exact.addr(), "[2001:db8::7]:5557");
    }

    #[test]
    fn a_station_listed_to_forty_kilometres_carries_four_geohash_tags_and_reads_back_as_four() {
        let mut e = entry("sdr.example.net", vec![airband()]);
        e.station.location =
            Some(Location::within(51.45, -0.97, sdr_directory::Accuracy::District));
        let tags = encode(&e);
        let cells: Vec<&str> = tags.iter().filter(|t| key(t) == "g").filter_map(value).collect();
        assert_eq!(cells, ["g", "gc", "gcp", "gcpk"]);
        let at = decode(tags.iter()).unwrap().station.location.unwrap();
        assert_eq!(at.geohash_len, 4);
        assert!((at.lat - 51.45).abs() < 0.18 && (at.lon - -0.97).abs() < 0.18, "{at:?}");
    }

    fn parsed(tags: &[&[&str]]) -> Result<Entry, String> {
        let tags: Vec<Tag> =
            tags.iter().map(|t| t.iter().map(|s| s.to_string()).collect()).collect();
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
    fn the_first_address_is_the_station_and_the_rest_are_other_ways_to_it() {
        let e = parsed(&[
            &["r", "https://sdr.example.net"],
            &["r", "iqstream://83.71.105.199:53956"],
            &["version", "1.5"],
            &["r", "iqstream://10.100.2.249:1234"],
            &["r", "iqstream://sdr.lan:1234"],
            &["r", "iqstream://[fd00::7]:1234"],
        ])
        .unwrap();
        assert_eq!(e.addr(), "83.71.105.199:53956", "the first iqstream one, not the first r");
        assert_eq!(
            e.also,
            ["10.100.2.249:1234".parse().unwrap(), "[fd00::7]:1234".parse().unwrap()],
            "a name that is not an address is skipped rather than resolved"
        );
    }

    #[test]
    fn a_webtransport_tag_names_its_port_and_the_certificates_it_may_serve() {
        let base: [&[&str]; 2] = [&["r", "iqstream://a.example:5557"], &["version", "1.4"]];
        let with = |extra: &[&str]| {
            let mut tags: Vec<&[&str]> = base.to_vec();
            tags.push(extra);
            parsed(&tags).unwrap()
        };
        let (a, b) = ("AB".repeat(32), "cd".repeat(32));
        let wt = with(&["webtransport", "5558", &a, &b]);
        assert_eq!(
            wt.webtransport,
            Some(WebTransport { port: 5558, hashes: vec!["ab".repeat(32), b.clone()] })
        );
        assert_eq!(
            wt.webtransport_url().unwrap(),
            format!("https://a.example:5558/?cert={},{b}", "ab".repeat(32))
        );
        assert_eq!(with(&["webtransport", "5558", "short"]).webtransport, None);
        assert_eq!(with(&["webtransport", "port", &b]).webtransport, None);
        assert_eq!(parsed(&base).unwrap().webtransport, None);
        assert!(with(&["webrtc", "20690"]).webrtc);
        assert!(!parsed(&base).unwrap().webrtc);
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
