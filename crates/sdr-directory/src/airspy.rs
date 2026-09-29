use crate::{Author, Dial, Entry, Error, Hardware, Listing, Published, SdrDirectory, Tuner};
use web_time::Duration;

pub const REGISTER: &str = "http://directory.api.airspy.com:8080/register";
pub const EVERY: Duration = Duration::from_secs(15);
pub const PROTOCOL: &str = "2.0.1700";
pub const MAX_CLIENTS: u32 = 8;
pub const DISPLAY_FPS: u32 = 15;
const CONTENT_TYPE: &str = "application/x-sdr-server-status";
const WAIT: Duration = Duration::from_secs(10);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Config {
    pub register: String,
    pub owner_email: String,
}

impl Default for Config {
    fn default() -> Self {
        Self { register: REGISTER.into(), owner_email: String::new() }
    }
}

pub struct AirspyDirectory {
    config: Config,
}

pub fn device_type(hardware: &Hardware) -> &'static str {
    match hardware {
        Hardware::Airspy => "AirspyOne",
        Hardware::AirspyHf => "AirspyHF+",
        _ => "RTL-SDR",
    }
}

fn reach(t: &Tuner) -> (u64, u64) {
    match t.dial {
        Dial::Tunable { min_hz: Some(lo), max_hz: Some(hi) } => (lo, hi),
        _ => t.span_hz(),
    }
}

pub fn status(entry: &Entry, config: &Config) -> Option<String> {
    let s = &entry.station;
    let t = s.tuners.first()?;
    let (lo, hi) = reach(t);
    let location = s.location.map_or(String::new(), |at| {
        let cell = common::geohash::encode(at.lat, at.lon, at.geohash_len);
        let (lat, lon) = common::geohash::decode(&cell).unwrap_or((at.lat, at.lon));
        format!("{lat:.4}, {lon:.4}")
    });
    let yes = |b: bool| if b { "Yes" } else { "No" };
    let one_line = |v: &str| v.replace(['\r', '\n'], " ");
    let fields: [(&str, String); 21] = [
        ("ServerVersion", PROTOCOL.into()),
        (
            "OperatingSystem",
            format!("waveshark {} on {}", env!("CARGO_PKG_VERSION"), std::env::consts::OS),
        ),
        ("MaxClients", s.max_clients.unwrap_or(MAX_CLIENTS).to_string()),
        ("CurrentClientCount", s.clients.to_string()),
        ("CurrentClients", String::new()),
        ("MaxSessionDuration", s.session_limit_secs.map_or(0, |v| v / 60).to_string()),
        ("StreamingPort", entry.port.to_string()),
        ("OwnerName", one_line(&s.name)),
        ("OwnerEmail", one_line(&config.owner_email)),
        ("AntennaType", one_line(&t.antenna)),
        ("AntennaLocation", location),
        ("GeneralDescription", one_line(&s.description)),
        ("DeviceType", device_type(&t.hardware).into()),
        ("DeviceSerial", "0".into()),
        ("DeviceResolution", "8".into()),
        ("FullControlAllowed", yes(t.tunable()).into()),
        ("CurrentCenterFrequency", t.center_hz.to_string()),
        ("MinimumFrequency", lo.to_string()),
        ("MaximumFrequency", hi.to_string()),
        ("MaximumDisplayedBandwidth", t.sample_rate.to_string()),
        ("DisplayFPS", DISPLAY_FPS.to_string()),
    ];
    let mut out: String = fields.iter().map(|(k, v)| format!("{k}={v}\r\n")).collect();
    out.push_str(&format!("MaximumStreamedBandwidth={}\r\n", t.sample_rate));
    out.push_str(&format!("MaximumIQSampleRate={}\r\n", t.sample_rate));
    Some(out)
}

impl SdrDirectory for AirspyDirectory {
    type Config = Config;

    fn open(config: &Config, _wait: Duration) -> Result<Self, Error> {
        Ok(Self { config: config.clone() })
    }

    fn author(&self) -> Option<Author> {
        None
    }

    fn every(&self) -> Duration {
        EVERY
    }

    fn announce(&self, entry: &Entry) -> Result<Published, Error> {
        let body = status(entry, &self.config).ok_or_else(|| {
            Error::Refused(vec![(self.config.register.clone(), "no tuner".into())])
        })?;
        let answer = httpc::post(&self.config.register)
            .timeout(WAIT)
            .header("Content-Type", CONTENT_TYPE)
            .body(body)
            .wait()
            .map_err(Error::Unreachable)?;
        match answer.is_success() {
            true => Ok(Published { accepted: 1, refused: Vec::new() }),
            false => Err(Error::Refused(vec![(
                self.config.register.clone(),
                format!("HTTP {}", answer.status),
            )])),
        }
    }

    fn withdraw(&self) -> Result<Published, Error> {
        Ok(Published { accepted: 0, refused: Vec::new() })
    }

    fn list(&self, _wait: Duration) -> Result<Vec<Listing>, Error> {
        Ok(Vec::new())
    }

    fn list_near(&self, _geohash: &str, _wait: Duration) -> Result<Vec<Listing>, Error> {
        Ok(Vec::new())
    }

    fn close(self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Accuracy, Location, Protocol, Station, Version};

    fn entry() -> Entry {
        Entry {
            host: "203.0.113.7".into(),
            port: 5555,
            data_port: Some(5555),
            also: Vec::new(),
            webtransport: None,
            webrtc: false,
            station: Station {
                name: "EI0ABC".into(),
                description: "Loft\nNavan".into(),
                location: Some(Location::within(53.637, -6.653, Accuracy::Town)),
                protocol: Protocol::IqStream(Version::OURS),
                clients: 2,
                max_clients: None,
                session_limit_secs: None,
                tuners: vec![Tuner {
                    id: 0,
                    name: "span".into(),
                    hardware: Hardware::RtlSdr,
                    antenna: "discone".into(),
                    center_hz: 433_920_000,
                    sample_rate: 2_400_000,
                    dial: Dial::Fixed,
                }],
            },
        }
    }

    #[test]
    fn the_status_is_the_fields_spyserver_2_0_1922_posts_in_its_order() {
        let body = status(&entry(), &Config { owner_email: "a@b.c".into(), ..Default::default() })
            .unwrap();
        let keys: Vec<&str> =
            body.lines().filter_map(|l| l.split_once('=')).map(|(k, _)| k).collect();
        assert_eq!(
            keys,
            [
                "ServerVersion",
                "OperatingSystem",
                "MaxClients",
                "CurrentClientCount",
                "CurrentClients",
                "MaxSessionDuration",
                "StreamingPort",
                "OwnerName",
                "OwnerEmail",
                "AntennaType",
                "AntennaLocation",
                "GeneralDescription",
                "DeviceType",
                "DeviceSerial",
                "DeviceResolution",
                "FullControlAllowed",
                "CurrentCenterFrequency",
                "MinimumFrequency",
                "MaximumFrequency",
                "MaximumDisplayedBandwidth",
                "DisplayFPS",
                "MaximumStreamedBandwidth",
                "MaximumIQSampleRate",
            ]
        );
        assert_eq!(body.matches("\r\n").count(), 23);
        for line in [
            "StreamingPort=5555",
            "OwnerName=EI0ABC",
            "OwnerEmail=a@b.c",
            "GeneralDescription=Loft Navan",
            "DeviceType=RTL-SDR",
            "FullControlAllowed=No",
            "CurrentCenterFrequency=433920000",
            "MinimumFrequency=432720000",
            "MaximumFrequency=435120000",
            "MaximumIQSampleRate=2400000",
            "CurrentClientCount=2",
            "MaxClients=8",
        ] {
            assert!(body.contains(&format!("{line}\r\n")), "{line} missing from {body}");
        }
    }

    #[test]
    fn an_announcement_is_one_post_of_the_status_to_the_register_path() {
        use std::io::{Read, Write};
        let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let register = format!("http://{}/register", l.local_addr().unwrap());
        let heard = common::thread::spawn(move || {
            let (mut sock, _) = l.accept().unwrap();
            let mut got = Vec::new();
            let mut buf = [0u8; 4096];
            while !String::from_utf8_lossy(&got).contains("MaximumIQSampleRate=2400000\r\n") {
                let n = sock.read(&mut buf).unwrap();
                got.extend_from_slice(&buf[..n]);
            }
            sock.write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n").unwrap();
            String::from_utf8(got).unwrap()
        });
        let dir =
            AirspyDirectory::open(&Config { register, owner_email: String::new() }, WAIT).unwrap();
        assert_eq!(dir.announce(&entry()).unwrap().accepted, 1);
        let request = heard.join().unwrap();
        assert!(request.starts_with("POST /register HTTP/1.1\r\n"), "{request}");
        assert!(
            request
                .to_ascii_lowercase()
                .contains("content-type: application/x-sdr-server-status\r\n")
        );
        assert!(request.ends_with(&status(&entry(), &Config::default()).unwrap()));
        assert_eq!(dir.every(), Duration::from_secs(15));
    }

    #[test]
    fn the_location_is_sent_no_finer_than_the_listing_allows() {
        let body = status(&entry(), &Config::default()).unwrap();
        let at = body.lines().find_map(|l| l.strip_prefix("AntennaLocation=")).unwrap();
        let hash = common::geohash::encode(53.637, -6.653, Accuracy::Town.geohash_len());
        let (lat, lon) = common::geohash::decode(&hash).unwrap();
        assert_eq!(at, format!("{lat:.4}, {lon:.4}"));
        assert_ne!(at, "53.6370, -6.6530");
    }
}
