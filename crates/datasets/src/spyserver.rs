use crate::cache::{Cache, Error, Source, When};
use common::time::Duration;
use sdr_directory::{
    Accuracy, Author, Dial, Entry, Hardware, Listing, Location, Protocol, Station, Tuner,
};
use serde::Deserialize;

pub fn source() -> Source {
    Source::http(
        "spyservers.json",
        "https://airspy.com/directory/status.json",
        Duration::from_secs(10 * 60),
    )
    .checked(|head| match head.starts_with(b"{") {
        true => Ok(()),
        false => Err("airspy.com did not answer with the server directory".into()),
    })
}

#[derive(Deserialize)]
struct File {
    servers: Vec<Row>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Row {
    #[serde(default)]
    streaming_host: String,
    #[serde(default)]
    streaming_port: u16,
    #[serde(default)]
    general_description: String,
    #[serde(default)]
    owner_name: String,
    #[serde(default)]
    device_type: String,
    #[serde(default)]
    antenna_type: String,
    #[serde(default)]
    minimum_frequency: u64,
    #[serde(default)]
    maximum_frequency: u64,
    #[serde(default)]
    current_center_frequency: u64,
    #[serde(default)]
    maximum_streamed_bandwidth: u64,
    #[serde(default)]
    current_client_count: u32,
    #[serde(default)]
    max_clients: u32,
    #[serde(default)]
    full_control_allowed: bool,
    #[serde(default)]
    online: bool,
    #[serde(default)]
    max_session_duration: u32,
    #[serde(default)]
    antenna_location: Option<Antenna>,
}

#[derive(Deserialize)]
struct Antenna {
    #[serde(default)]
    lat: f64,
    #[serde(default)]
    long: f64,
}

fn hardware(device_type: &str) -> Hardware {
    match device_type {
        "AirspyOne" => Hardware::Airspy,
        "AirspyHF+" => Hardware::AirspyHf,
        "RTL-SDR" => Hardware::RtlSdr,
        other => Hardware::Other(other.to_string()),
    }
}

pub fn parse(name: &str, raw: &[u8], now: u64) -> Result<Vec<Listing>, Error> {
    let file: File =
        serde_json::from_slice(raw).map_err(|e| Error::Parse(name.into(), e.to_string()))?;
    let mut out: Vec<Listing> = file
        .servers
        .into_iter()
        .filter(|r| r.online && !r.streaming_host.trim().is_empty() && r.streaming_port != 0)
        .map(|r| {
            let written = r.general_description.trim();
            let description = match written.is_empty() || written == "no description" {
                true => r.owner_name.trim().to_string(),
                false => written.to_string(),
            };
            let host = r.streaming_host.trim().to_string();
            let dial = match r.full_control_allowed {
                true => Dial::Tunable {
                    min_hz: Some(r.minimum_frequency),
                    max_hz: Some(r.maximum_frequency),
                },
                false => Dial::Fixed,
            };
            let tuner = Tuner {
                id: 0,
                name: String::new(),
                hardware: hardware(&r.device_type),
                antenna: r.antenna_type.trim().to_string(),
                center_hz: r.current_center_frequency,
                sample_rate: r.maximum_streamed_bandwidth.min(u32::MAX as u64) as u32,
                dial,
            };
            let entry = Entry {
                host: host.clone(),
                port: r.streaming_port,
                data_port: None,
                also: Vec::new(),
                station: Station {
                    name: description,
                    description: String::new(),
                    location: r
                        .antenna_location
                        .filter(|l| l.lat != 0.0 || l.long != 0.0)
                        .map(|l| Location::within(l.lat, l.long, Accuracy::Street)),
                    protocol: Protocol::SpyServer,
                    clients: r.current_client_count,
                    max_clients: Some(r.max_clients),
                    session_limit_secs: (r.max_session_duration > 0)
                        .then_some(r.max_session_duration),
                    tuners: vec![tuner],
                },
            };
            Listing { author: Author(entry.addr()), seen: now, entry }
        })
        .collect();
    sdr_directory::distinct(&mut out);
    if out.is_empty() {
        return Err(Error::Parse(name.into(), "no servers in the directory".into()));
    }
    Ok(out)
}

pub fn load(cache: &Cache) -> Result<Vec<Listing>, Error> {
    let src = source();
    parse(src.name, &cache.read(&src)?, sdr_directory::now())
}

pub fn refresh(cache: &Cache, when: When) -> Result<Option<Vec<Listing>>, Error> {
    let src = source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse(src.name, &cache.read(&src)?, sdr_directory::now()).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;

    const FILE: &str = r#"{"servers":[
      {"serverVersion":"2.0.1922","maxClients":5,"currentClientCount":0,"currentClients":[],
       "maxSessionDuration":180,"streamingPort":5556,"ownerName":"Russ Gladden",
       "ownerEmail":"sdr@example.org","antennaType":"Discone",
       "antennaLocation":{"lat":45.2851739,"long":-75.731526},
       "generalDescription":"Airspy R2 - Ottawa, Canada","deviceType":"AirspyOne",
       "deviceSerial":"266B939B","deviceResolution":12,"fullControlAllowed":true,
       "currentCenterFrequency":93600000,"minimumFrequency":24000000,
       "maximumFrequency":1700000000,"maximumStreamedBandwidth":2000000,
       "maximumIQSampleRate":2500000,"streamingHost":"135.23.92.149","online":true,
       "registered":false,"lastSeen":10},
      {"maxClients":10,"currentClientCount":0,"maxSessionDuration":0,"streamingPort":5555,
       "ownerName":"John Doe L8ZEE","ownerEmail":"john@example.com","antennaType":"",
       "antennaLocation":{"lat":0,"long":0},"generalDescription":"no description",
       "deviceType":"AirspyOne","fullControlAllowed":false,
       "currentCenterFrequency":808712500,"minimumFrequency":24000000,
       "maximumFrequency":1750000000,"maximumStreamedBandwidth":8000000,
       "streamingHost":"209.248.90.17","online":true,"lastSeen":5},
      {"maxClients":1,"currentClientCount":1,"maxSessionDuration":0,"streamingPort":5555,
       "ownerName":"anonymous","ownerEmail":"","antennaType":"long wire",
       "generalDescription":"HF in Saigon","deviceType":"AirspyHF+",
       "fullControlAllowed":true,"currentCenterFrequency":9011000,"minimumFrequency":0,
       "maximumFrequency":31000000,"maximumStreamedBandwidth":62500,
       "streamingHost":"2001:db8::7","online":true,"lastSeen":3},
      {"maxClients":4,"currentClientCount":0,"maxSessionDuration":0,"streamingPort":5555,
       "ownerName":"somebody","generalDescription":"Switched off for the winter",
       "deviceType":"RTL-SDR","fullControlAllowed":true,"minimumFrequency":24000000,
       "maximumFrequency":1766000000,"streamingHost":"sdr.example.net","online":false,
       "lastSeen":86400},
      {"maxClients":4,"currentClientCount":0,"streamingPort":0,"streamingHost":"",
       "generalDescription":"nowhere to connect","online":true}
    ]}"#;

    const NOW: u64 = 1_750_000_000;

    fn servers() -> Vec<Listing> {
        parse("test", FILE.as_bytes(), NOW).unwrap()
    }

    fn named<'a>(v: &'a [Listing], name: &str) -> &'a Listing {
        v.iter().find(|l| l.entry.station.name == name).unwrap()
    }

    fn tuner<'a>(v: &'a [Listing], name: &str) -> &'a Tuner {
        &named(v, name).entry.station.tuners[0]
    }

    #[test]
    fn every_online_row_with_an_address_parses() {
        let v = servers();
        let names: Vec<&str> = v.iter().map(|l| l.entry.station.name.as_str()).collect();
        assert_eq!(names, ["Airspy R2 - Ottawa, Canada", "John Doe L8ZEE", "HF in Saigon"]);
        assert!(v.iter().all(|l| l.seen == NOW && l.entry.station.protocol == Protocol::SpyServer));
        assert_eq!(tuner(&v, "Airspy R2 - Ottawa, Canada").antenna, "Discone");
        assert_eq!(tuner(&v, "John Doe L8ZEE").antenna, "");
    }

    #[test]
    fn a_row_is_opened_at_host_colon_port() {
        let v = servers();
        assert_eq!(named(&v, "Airspy R2 - Ottawa, Canada").entry.addr(), "135.23.92.149:5556");
        assert_eq!(named(&v, "HF in Saigon").entry.addr(), "[2001:db8::7]:5555");
    }

    #[test]
    fn full_control_is_a_dial_across_the_range_and_none_is_a_fixed_one() {
        let v = servers();
        assert_eq!(
            tuner(&v, "Airspy R2 - Ottawa, Canada").dial,
            Dial::Tunable { min_hz: Some(24_000_000), max_hz: Some(1_700_000_000) }
        );
        assert_eq!(tuner(&v, "John Doe L8ZEE").dial, Dial::Fixed);
    }

    #[test]
    fn the_device_type_is_the_hardware() {
        let v = servers();
        let hw: Vec<&Hardware> = v.iter().map(|l| &l.entry.station.tuners[0].hardware).collect();
        assert_eq!(hw, [&Hardware::Airspy, &Hardware::Airspy, &Hardware::AirspyHf]);
    }

    #[test]
    fn a_blank_description_falls_back_to_the_owner_and_the_email_is_not_read() {
        let v = servers();
        assert_eq!(named(&v, "John Doe L8ZEE").entry.station.location, None, "0, 0 is nowhere");
        assert!(!format!("{v:?}").contains("example.com"));
        assert!(!format!("{v:?}").contains("sdr@example.org"));
    }

    #[test]
    fn the_readings_come_through_as_the_directory_gives_them() {
        let v = servers();
        let s = &named(&v, "Airspy R2 - Ottawa, Canada").entry.station;
        let t = &s.tuners[0];
        assert_eq!((t.center_hz, t.sample_rate), (93_600_000, 2_000_000));
        assert_eq!((s.clients, s.max_clients), (0, Some(5)));
        assert_eq!(s.session_limit_secs, Some(180));
        let at = s.location.unwrap();
        assert_eq!((at.lat, at.lon), (45.2851739, -75.731526));
        assert_eq!(named(&v, "John Doe L8ZEE").entry.station.session_limit_secs, None);
    }

    #[test]
    fn a_wanted_frequency_keeps_the_servers_whose_dial_reaches_it() {
        let v = servers();
        let kept = |hz: u64| -> Vec<&str> {
            let q = sdr_directory::Query { hz: Some(hz), ..Default::default() };
            v.iter().filter(|l| q.keeps(l, NOW)).map(|l| l.entry.station.name.as_str()).collect()
        };
        assert_eq!(kept(145_800_000), ["Airspy R2 - Ottawa, Canada"]);
        assert_eq!(kept(810_000_000), ["Airspy R2 - Ottawa, Canada", "John Doe L8ZEE"]);
        assert_eq!(kept(7_074_000), ["HF in Saigon"]);
    }

    #[test]
    fn a_directory_with_no_servers_is_an_error() {
        assert!(parse("test", br#"{"servers":[]}"#, NOW).is_err());
        assert!(parse("test", b"<html>", NOW).is_err());
    }
}
