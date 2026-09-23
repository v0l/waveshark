use crate::cache::{Cache, Error, Source, When};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Mutex;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

pub const WORKERS: usize = 10;

pub const ANSWERED_FOR: u64 = 60 * 60;

pub const SILENT_FOR: u64 = 4 * 60 * 60;

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

    pub fn as_heard(&self, heard: Option<Heard>) -> Server {
        let mut s = self.clone();
        if let Some(Heard::Answered { control, center_hz }) = heard {
            s.full_control = control;
            s.center_hz = center_hz;
        }
        s
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Heard {
    Answered { control: bool, center_hz: u64 },
    Silent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Probed {
    pub at: u64,
    pub heard: Heard,
}

impl Probed {
    pub fn stale(&self, now: u64) -> bool {
        let lasts = match self.heard {
            Heard::Answered { .. } => ANSWERED_FOR,
            Heard::Silent => SILENT_FOR,
        };
        now.saturating_sub(self.at) >= lasts
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Probes(HashMap<String, Probed>);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Tally {
    pub listed: usize,
    pub checked: usize,
    pub answering: usize,
}

impl Probes {
    pub fn path(cache: &Cache) -> PathBuf {
        cache.dir().join("spyservers-probed.json")
    }

    pub fn read(path: &Path) -> Self {
        std::fs::read(path)
            .ok()
            .and_then(|raw| serde_json::from_slice(&raw).ok())
            .unwrap_or_default()
    }

    pub fn write(&self, path: &Path) -> std::io::Result<()> {
        let raw = serde_json::to_vec(self).map_err(std::io::Error::other)?;
        let part = path.with_extension("json.part");
        std::fs::write(&part, raw)?;
        std::fs::rename(&part, path)
    }

    pub fn get(&self, s: &Server) -> Option<Probed> {
        self.0.get(&s.addr()).copied()
    }

    pub fn heard(&self, s: &Server) -> Option<Heard> {
        self.get(s).map(|p| p.heard)
    }

    pub fn insert(&mut self, s: &Server, probed: Probed) {
        self.0.insert(s.addr(), probed);
    }

    pub fn due<'a>(&self, servers: &'a [Server], now: u64) -> Vec<&'a Server> {
        servers.iter().filter(|s| s.online && self.get(s).is_none_or(|p| p.stale(now))).collect()
    }

    pub fn tally(&self, servers: &[Server]) -> Tally {
        servers.iter().filter(|s| s.online).fold(Tally::default(), |mut t, s| {
            t.listed += 1;
            match self.heard(s) {
                Some(Heard::Answered { .. }) => {
                    t.checked += 1;
                    t.answering += 1;
                }
                Some(Heard::Silent) => t.checked += 1,
                None => {}
            }
            t
        })
    }

    fn keep_listed(&mut self, servers: &[Server]) {
        self.0.retain(|addr, _| servers.iter().any(|s| s.addr() == *addr));
    }
}

pub fn sweep(
    probes: &Mutex<Probes>,
    servers: &[Server],
    now: u64,
    workers: usize,
    probe: impl Fn(&Server) -> Heard + Sync,
) -> usize {
    let due = {
        let mut held = probes.lock().unwrap_or_else(|e| e.into_inner());
        held.keep_listed(servers);
        held.due(servers, now)
    };
    let next = AtomicUsize::new(0);
    std::thread::scope(|scope| {
        for _ in 0..workers.min(due.len()) {
            scope.spawn(|| {
                while let Some(s) = due.get(next.fetch_add(1, Ordering::Relaxed)) {
                    let heard = probe(s);
                    probes
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .insert(s, Probed { at: now, heard });
                }
            });
        }
    });
    due.len()
}

#[derive(Clone, Debug, PartialEq)]
pub struct Listed {
    pub server: Server,
    pub heard: Option<Heard>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Filter {
    pub hz: Option<u64>,
    pub full_control: bool,
    pub free: bool,
}

impl Filter {
    pub fn keeps(&self, s: &Server, heard: Option<Heard>) -> bool {
        s.online
            && heard != Some(Heard::Silent)
            && self.hz.is_none_or(|hz| s.tunes(hz))
            && (!self.full_control || s.full_control)
            && (!self.free || s.has_slot())
    }

    pub fn list(&self, servers: &[Server], probes: &Probes) -> Vec<Listed> {
        let mut out: Vec<Listed> = servers
            .iter()
            .map(|s| {
                let heard = probes.heard(s);
                Listed { server: s.as_heard(heard), heard }
            })
            .filter(|l| self.keeps(&l.server, l.heard))
            .collect();
        out.sort_by_key(|l| l.heard.is_none());
        out
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
        servers().into_iter().filter(|s| f.keeps(s, None)).map(|s| s.description).collect()
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

    fn heard_by_name(s: &Server) -> Heard {
        match s.description.as_str() {
            "John Doe L8ZEE" => Heard::Answered { control: true, center_hz: 145_800_000 },
            _ => Heard::Silent,
        }
    }

    fn swept(now: u64) -> (Mutex<Probes>, Vec<Server>) {
        let (probes, v) = (Mutex::new(Probes::default()), servers());
        assert_eq!(
            sweep(&probes, &v, now, WORKERS, heard_by_name),
            3,
            "the offline one is not asked"
        );
        (probes, v)
    }

    #[test]
    fn of_one_answering_and_two_silent_exactly_the_one_that_answered_is_kept() {
        let (probes, v) = swept(1000);
        let listed = Filter::default().list(&v, &probes.lock().unwrap());
        let names: Vec<&str> = listed.iter().map(|l| l.server.description.as_str()).collect();
        assert_eq!(names, ["John Doe L8ZEE"]);
        assert_eq!(probes.lock().unwrap().tally(&v), Tally { listed: 3, checked: 3, answering: 1 });
    }

    #[test]
    fn what_the_server_said_overrides_what_the_directory_said() {
        let (probes, v) = swept(1000);
        let f = Filter { full_control: true, ..Filter::default() };
        let listed = f.list(&v, &probes.lock().unwrap());
        assert_eq!(listed.len(), 1, "the directory says John Doe grants no control");
        assert!(listed[0].server.full_control);
        assert_eq!(listed[0].server.center_hz, 145_800_000);
        assert_eq!(named(&v, "John Doe L8ZEE").center_hz, 808_712_500);
    }

    #[test]
    fn a_server_that_answered_is_listed_above_those_not_yet_asked() {
        let v = servers();
        let mut probes = Probes::default();
        let hf = named(&v, "HF in Saigon");
        probes.insert(hf, Probed { at: 0, heard: Heard::Answered { control: true, center_hz: 1 } });
        let names: Vec<String> =
            Filter::default().list(&v, &probes).into_iter().map(|l| l.server.description).collect();
        assert_eq!(names, ["HF in Saigon", "Airspy R2 - Ottawa, Canada", "John Doe L8ZEE"]);
        assert_eq!(probes.tally(&v), Tally { listed: 3, checked: 1, answering: 1 });
    }

    #[test]
    fn a_second_sweep_asks_again_only_what_has_gone_stale() {
        let (probes, v) = swept(1000);
        let asked = AtomicUsize::new(0);
        let count = |s: &Server| {
            asked.fetch_add(1, Ordering::Relaxed);
            heard_by_name(s)
        };
        assert_eq!(sweep(&probes, &v, 1000 + ANSWERED_FOR - 1, WORKERS, count), 0);
        assert_eq!(sweep(&probes, &v, 1000 + SILENT_FOR - 1, WORKERS, count), 1, "the live one");
        assert_eq!(sweep(&probes, &v, 1000 + SILENT_FOR, WORKERS, count), 2, "the silent two");
        assert_eq!(asked.load(Ordering::Relaxed), 3);
    }

    #[test]
    fn ten_servers_are_asked_at_a_time() {
        let v: Vec<Server> = (0..30u16)
            .map(|i| Server { port: 5000 + i, ..named(&servers(), "John Doe L8ZEE").clone() })
            .collect();
        let (now, peak) = (AtomicUsize::new(0), AtomicUsize::new(0));
        let started = std::time::Instant::now();
        let probes = Mutex::new(Probes::default());
        let asked = sweep(&probes, &v, 0, WORKERS, |_| {
            peak.fetch_max(now.fetch_add(1, Ordering::SeqCst) + 1, Ordering::SeqCst);
            std::thread::sleep(Duration::from_millis(100));
            now.fetch_sub(1, Ordering::SeqCst);
            Heard::Silent
        });
        assert_eq!((asked, peak.load(Ordering::SeqCst)), (30, 10));
        let took = started.elapsed();
        assert!(took >= Duration::from_millis(300), "floor, three rounds of ten: {took:?}");
        assert!(took < Duration::from_millis(900), "ceiling, not one at a time: {took:?}");
    }

    #[test]
    fn the_probes_are_kept_on_disk_and_a_torn_file_reads_as_none() {
        let dir = std::env::temp_dir().join(format!("spyprobe-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("spyservers-probed.json");
        let (probes, v) = swept(1000);
        probes.lock().unwrap().write(&path).unwrap();
        let back = Probes::read(&path);
        assert_eq!(back, *probes.lock().unwrap());
        assert_eq!(back.tally(&v), Tally { listed: 3, checked: 3, answering: 1 });
        std::fs::write(&path, b"{\"torn").unwrap();
        assert_eq!(Probes::read(&path), Probes::default());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn a_server_gone_from_the_directory_is_forgotten_at_the_next_sweep() {
        let (probes, mut v) = swept(1000);
        v.retain(|s| s.description != "HF in Saigon");
        assert_eq!(sweep(&probes, &v, 1000, WORKERS, heard_by_name), 0);
        assert_eq!(probes.lock().unwrap().0.len(), 2);
    }

    #[test]
    fn a_directory_with_no_servers_is_an_error() {
        assert!(parse("test", br#"{"servers":[]}"#).is_err());
        assert!(parse("test", b"<html>").is_err());
    }
}
