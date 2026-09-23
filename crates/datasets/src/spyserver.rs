use crate::cache::{Cache, Error, Source, When};
use serde::Deserialize;
use std::time::Duration;

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

#[derive(Clone, Debug, PartialEq)]
pub struct Server {
    pub host: String,
    pub port: u16,
    pub description: String,
    pub device: String,
    pub antenna: String,
    pub min_hz: u64,
    pub max_hz: u64,
    pub center_hz: u64,
    pub bandwidth_hz: u64,
    pub clients: u32,
    pub max_clients: u32,
    pub full_control: bool,
    pub online: bool,
    pub session_limit: Option<u32>,
    pub location: Option<(f64, f64)>,
}

impl Server {
    pub fn addr(&self) -> String {
        match self.host.contains(':') {
            true => format!("[{}]:{}", self.host, self.port),
            false => format!("{}:{}", self.host, self.port),
        }
    }

    pub fn tunes(&self, hz: u64) -> bool {
        (self.min_hz..=self.max_hz).contains(&hz)
    }

    pub fn has_slot(&self) -> bool {
        self.clients < self.max_clients
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Filter {
    pub hz: Option<u64>,
    pub full_control: bool,
    pub free: bool,
}

impl Filter {
    pub fn keeps(&self, s: &Server) -> bool {
        s.online
            && self.hz.is_none_or(|hz| s.tunes(hz))
            && (!self.full_control || s.full_control)
            && (!self.free || s.has_slot())
    }
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
    antenna_location: Option<Location>,
}

#[derive(Deserialize)]
struct Location {
    #[serde(default)]
    lat: f64,
    #[serde(default)]
    long: f64,
}

pub fn parse(name: &str, raw: &[u8]) -> Result<Vec<Server>, Error> {
    let file: File =
        serde_json::from_slice(raw).map_err(|e| Error::Parse(name.into(), e.to_string()))?;
    let mut out: Vec<Server> = file
        .servers
        .into_iter()
        .filter(|r| !r.streaming_host.trim().is_empty() && r.streaming_port != 0)
        .map(|r| {
            let written = r.general_description.trim();
            let description = match written.is_empty() || written == "no description" {
                true => r.owner_name.trim().to_string(),
                false => written.to_string(),
            };
            Server {
                host: r.streaming_host.trim().to_string(),
                port: r.streaming_port,
                description,
                device: r.device_type,
                antenna: r.antenna_type.trim().to_string(),
                min_hz: r.minimum_frequency,
                max_hz: r.maximum_frequency,
                center_hz: r.current_center_frequency,
                bandwidth_hz: r.maximum_streamed_bandwidth,
                clients: r.current_client_count,
                max_clients: r.max_clients,
                full_control: r.full_control_allowed,
                online: r.online,
                session_limit: (r.max_session_duration > 0).then_some(r.max_session_duration),
                location: r
                    .antenna_location
                    .filter(|l| l.lat != 0.0 || l.long != 0.0)
                    .map(|l| (l.lat, l.long)),
            }
        })
        .collect();
    if out.is_empty() {
        return Err(Error::Parse(name.into(), "no servers in the directory".into()));
    }
    out.sort_by(|a, b| {
        (!a.has_slot(), !a.full_control, a.description.to_lowercase()).cmp(&(
            !b.has_slot(),
            !b.full_control,
            b.description.to_lowercase(),
        ))
    });
    Ok(out)
}

pub fn load(cache: &Cache) -> Result<Vec<Server>, Error> {
    let src = source();
    parse(src.name, &cache.read(&src)?)
}

pub fn refresh(cache: &Cache, when: When) -> Result<Option<Vec<Server>>, Error> {
    let src = source();
    if cache.refresh(&src, when)?.is_none() {
        return Ok(None);
    }
    parse(src.name, &cache.read(&src)?).map(Some)
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

    fn servers() -> Vec<Server> {
        parse("test", FILE.as_bytes()).unwrap()
    }

    fn named<'a>(v: &'a [Server], description: &str) -> &'a Server {
        v.iter().find(|s| s.description == description).unwrap()
    }

    #[test]
    fn every_row_with_an_address_parses_offline_and_unnamed_antenna_included() {
        let v = servers();
        assert_eq!(v.len(), 4, "the row with no host and port is the only one dropped");
        assert!(!named(&v, "Switched off for the winter").online);
        assert_eq!(named(&v, "Airspy R2 - Ottawa, Canada").antenna, "Discone");
        assert_eq!(named(&v, "John Doe L8ZEE").antenna, "");
    }

    #[test]
    fn a_row_is_opened_at_host_colon_port() {
        let v = servers();
        assert_eq!(named(&v, "Airspy R2 - Ottawa, Canada").addr(), "135.23.92.149:5556");
        assert_eq!(named(&v, "HF in Saigon").addr(), "[2001:db8::7]:5555");
        assert_eq!(named(&v, "Switched off for the winter").addr(), "sdr.example.net:5555");
    }

    #[test]
    fn full_control_is_carried_per_server() {
        let v = servers();
        assert!(named(&v, "Airspy R2 - Ottawa, Canada").full_control);
        assert!(!named(&v, "John Doe L8ZEE").full_control);
    }

    #[test]
    fn a_blank_description_falls_back_to_the_owner_and_the_email_is_not_read() {
        let v = servers();
        let s = named(&v, "John Doe L8ZEE");
        assert_eq!(s.location, None, "0, 0 is a location nobody gave");
        assert!(!format!("{v:?}").contains("example.com"));
        assert!(!format!("{v:?}").contains("sdr@example.org"));
    }

    #[test]
    fn the_readings_come_through_as_the_directory_gives_them() {
        let v = servers();
        let s = named(&v, "Airspy R2 - Ottawa, Canada");
        assert_eq!((s.min_hz, s.max_hz, s.center_hz), (24_000_000, 1_700_000_000, 93_600_000));
        assert_eq!(s.bandwidth_hz, 2_000_000);
        assert_eq!((s.clients, s.max_clients), (0, 5));
        assert_eq!(s.session_limit, Some(180));
        assert_eq!(s.location, Some((45.2851739, -75.731526)));
        assert_eq!(named(&v, "John Doe L8ZEE").session_limit, None);
    }

    #[test]
    fn a_server_with_a_slot_free_and_full_control_is_listed_first() {
        let order: Vec<String> = servers().into_iter().map(|s| s.description).collect();
        assert_eq!(
            order,
            [
                "Airspy R2 - Ottawa, Canada",
                "Switched off for the winter",
                "John Doe L8ZEE",
                "HF in Saigon"
            ]
        );
    }

    fn kept(f: Filter) -> Vec<String> {
        servers().into_iter().filter(|s| f.keeps(s)).map(|s| s.description).collect()
    }

    #[test]
    fn with_no_filter_every_online_server_is_kept() {
        assert_eq!(kept(Filter::default()).len(), 3, "the offline server is dropped");
    }

    #[test]
    fn a_wanted_frequency_keeps_the_servers_that_tune_it() {
        let f = Filter { hz: Some(145_800_000), ..Filter::default() };
        assert_eq!(kept(f), ["Airspy R2 - Ottawa, Canada", "John Doe L8ZEE"]);
        let f = Filter { hz: Some(7_074_000), ..Filter::default() };
        assert_eq!(kept(f), ["HF in Saigon"]);
        let f = Filter { hz: Some(1_720_000_000), ..Filter::default() };
        assert_eq!(kept(f), ["John Doe L8ZEE"], "past the R2's top, inside the other's");
    }

    #[test]
    fn full_control_and_a_free_slot_each_narrow_the_list() {
        let f = Filter { full_control: true, ..Filter::default() };
        assert_eq!(kept(f), ["Airspy R2 - Ottawa, Canada", "HF in Saigon"]);
        let f = Filter { free: true, ..Filter::default() };
        assert_eq!(kept(f), ["Airspy R2 - Ottawa, Canada", "John Doe L8ZEE"]);
        let f = Filter { full_control: true, free: true, ..Filter::default() };
        assert_eq!(kept(f), ["Airspy R2 - Ottawa, Canada"]);
    }

    #[test]
    fn a_directory_with_no_servers_is_an_error() {
        assert!(parse("test", br#"{"servers":[]}"#).is_err());
        assert!(parse("test", b"<html>").is_err());
    }
}
