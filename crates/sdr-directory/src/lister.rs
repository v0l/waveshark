use crate::model::private;
use crate::portmap::PortMap;
use crate::{
    ANNOUNCE_EVERY_SECS, Entry, Hardware, Location, SdrDirectory, Station, Tuner, Version,
};
use std::net::SocketAddr;
use std::num::NonZeroU16;
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

const DIRECTORY_WAIT: Duration = Duration::from_secs(10);
const SERVER_POLL: Duration = Duration::from_secs(1);
const RETRY: Duration = Duration::from_secs(30);
const WATCH: Duration = Duration::from_secs(1);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pace {
    pub settle: Duration,
    pub apart: Duration,
}

impl Pace {
    pub const LIVE: Pace =
        Pace { settle: Duration::from_secs(3), apart: Duration::from_secs(3 * 60) };
}

#[derive(Clone, Debug, PartialEq)]
pub struct Offer<C> {
    pub name: String,
    pub description: String,
    pub antenna: String,
    pub location: Option<Location>,
    pub public_host: Option<String>,
    pub directory: C,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ListingState {
    Waiting,
    Mapping,
    Unreachable(String),
    Listed { at: String, servers: usize },
    Refused(String),
}

impl ListingState {
    pub fn describe(&self) -> String {
        match self {
            ListingState::Waiting => "waiting for the server".into(),
            ListingState::Mapping => "opening the port on the router".into(),
            ListingState::Unreachable(why) => format!("not listed: {why}"),
            ListingState::Listed { at, servers } => format!("listed as {at} on {servers} servers"),
            ListingState::Refused(why) => format!("not listed: {why}"),
        }
    }
}

struct Wanted<C> {
    now: Mutex<(u64, Option<Offer<C>>)>,
    moved: Condvar,
}

impl<C: Clone + PartialEq> Wanted<C> {
    fn new() -> Self {
        Wanted { now: Mutex::new((0, None)), moved: Condvar::new() }
    }

    fn current(&self) -> (u64, Option<Offer<C>>) {
        self.now.lock().map(|n| n.clone()).unwrap_or((0, None))
    }

    fn set(&self, listing: Option<Offer<C>>, only_if_moved: bool) {
        let Ok(mut n) = self.now.lock() else { return };
        if only_if_moved && n.1 == listing {
            return;
        }
        *n = (n.0 + 1, listing);
        self.moved.notify_all();
    }

    fn wait(&self, seen: u64, at_most: Duration) {
        let Ok(n) = self.now.lock() else { return };
        let _ = self.moved.wait_timeout_while(n, at_most, |n| n.0 == seen);
    }
}

pub struct Lister<D: SdrDirectory> {
    wanted: Arc<Wanted<D::Config>>,
    state: Arc<Mutex<ListingState>>,
    thread: std::thread::JoinHandle<()>,
}

impl<D: SdrDirectory + 'static> Lister<D> {
    pub fn start(
        find: impl Fn() -> Option<Arc<iqstream::Server>> + Send + 'static,
        listing: Offer<D::Config>,
    ) -> std::io::Result<Lister<D>> {
        Lister::paced(find, listing, Pace::LIVE)
    }

    pub fn paced(
        find: impl Fn() -> Option<Arc<iqstream::Server>> + Send + 'static,
        listing: Offer<D::Config>,
        pace: Pace,
    ) -> std::io::Result<Lister<D>> {
        let wanted = Arc::new(Wanted::new());
        wanted.set(Some(listing), false);
        let state = Arc::new(Mutex::new(ListingState::Waiting));
        let (shown, asked) = (state.clone(), wanted.clone());
        let thread = std::thread::Builder::new()
            .name("iqstream-list".into())
            .spawn(move || run::<D>(find, &asked, &shown, pace))?;
        Ok(Lister { wanted, state, thread })
    }

    pub fn update(&self, listing: Offer<D::Config>) {
        self.wanted.set(Some(listing), true);
    }

    pub fn offer(&self) -> Option<Offer<D::Config>> {
        self.wanted.current().1
    }

    pub fn state(&self) -> ListingState {
        self.state.lock().map(|s| s.clone()).unwrap_or(ListingState::Waiting)
    }

    pub fn stop(&self) {
        self.wanted.set(None, false);
    }

    pub fn finished(&self) -> bool {
        self.thread.is_finished()
    }

    pub fn withdraw(self, within: Duration) {
        withdraw_all(vec![self], within);
    }
}

impl<D: SdrDirectory> Drop for Lister<D> {
    fn drop(&mut self) {
        self.wanted.set(None, false);
    }
}

pub fn withdraw_all<D: SdrDirectory + 'static>(listers: Vec<Lister<D>>, within: Duration) {
    for l in &listers {
        l.stop();
    }
    let started = std::time::Instant::now();
    while listers.iter().any(|l| !l.finished()) && started.elapsed() < within {
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn set(state: &Mutex<ListingState>, now: ListingState) {
    if let Ok(mut s) = state.lock()
        && *s != now
    {
        tracing::info!("iqstream directory: {}", now.describe());
        *s = now;
    }
}

pub fn entry<C>(
    listing: &Offer<C>,
    host: &str,
    port: u16,
    data_port: Option<u16>,
    also: Vec<SocketAddr>,
    server: &iqstream::Server,
) -> Entry {
    let streams = server.streams();
    let tuners = streams
        .iter()
        .map(|s| {
            let hardware = s.hardware();
            let hardware = match hardware.is_empty() {
                true => Hardware::Other("unknown".into()),
                false => Hardware::from(hardware.as_str()),
            };
            Tuner::of(&s.desc(), hardware, &listing.antenna)
        })
        .collect();
    Entry {
        host: host.to_string(),
        port,
        data_port,
        also,
        station: Station {
            name: listing.name.clone(),
            description: listing.description.clone(),
            location: listing.location,
            version: Version::OURS,
            clients: streams.iter().map(|s| s.subscribers() as u32).sum(),
            max_clients: None,
            session_limit_secs: None,
            tuners,
        },
    }
}

struct Reach {
    host: String,
    port: u16,
    data_port: Option<u16>,
    also: Vec<SocketAddr>,
}

impl Reach {
    fn entry<C>(&self, listing: &Offer<C>, server: &iqstream::Server) -> Entry {
        entry(listing, &self.host, self.port, self.data_port, self.also.clone(), server)
    }
}

fn reach<C>(
    listing: &Offer<C>,
    server: &iqstream::Server,
    map: &mut Option<PortMap>,
    state: &Mutex<ListingState>,
) -> Result<Reach, String> {
    let local = server.addr().port();
    let (host, port, data_port) = match &listing.public_host {
        Some(host) => (host.clone(), local, Some(local)),
        None => mapped(map, local, state)?,
    };
    server.set_public(listing.public_host.is_none().then(|| iqstream::Public {
        addr: SocketAddr::new(host.parse().unwrap_or(server.addr().ip()), port),
        data_port,
    }));
    let also = match listing.public_host {
        Some(_) => Vec::new(),
        None => lan_addr(server.addr()).into_iter().collect(),
    };
    Ok(Reach { host, port, data_port, also })
}

fn run<D: SdrDirectory>(
    find: impl Fn() -> Option<Arc<iqstream::Server>>,
    wanted: &Wanted<D::Config>,
    state: &Mutex<ListingState>,
    pace: Pace,
) {
    let Some(server) = wait_for_server(&find, wanted) else { return };
    let mut map: Option<PortMap> = None;
    let mut dir: Option<(D::Config, D)> = None;
    let mut announced = false;
    let mut listed: Option<(Reach, Entry)> = None;
    let mut tried: Option<u64> = None;
    let mut due = Instant::now();
    let mut last = Instant::now();
    let mut moved: Option<(Entry, Instant)> = None;
    while let (seen, Some(listing)) = wanted.current() {
        let drifted = listed
            .as_ref()
            .map(|(r, was)| (r.entry(&listing, &server), was))
            .and_then(|(now, was)| (now != *was).then_some(now));
        moved = match (drifted, moved.take()) {
            (Some(now), Some((was, since))) if now == was => Some((was, since)),
            (Some(now), _) => Some((now, Instant::now())),
            (None, _) => None,
        };
        let settled = moved.as_ref().is_some_and(|(_, since)| since.elapsed() >= pace.settle);
        let drift = settled && last.elapsed() >= pace.apart;
        if tried != Some(seen) || drift || Instant::now() >= due {
            (tried, moved, last) = (Some(seen), None, Instant::now());
            let now = match reach(&listing, &server, &mut map, state) {
                Err(why) => {
                    listed = None;
                    ListingState::Unreachable(why)
                }
                Ok(r) => {
                    let e = r.entry(&listing, &server);
                    let now = announce(&listing, &e, &mut dir, &mut announced);
                    listed = matches!(now, ListingState::Listed { .. }).then_some((r, e));
                    now
                }
            };
            let mut next = match listed {
                Some(_) => Duration::from_secs(ANNOUNCE_EVERY_SECS),
                None => RETRY,
            };
            if let Some(m) = &map {
                next = next.min(m.renew_in());
            }
            due = Instant::now() + next;
            set(state, now);
        }
        let left = due.saturating_duration_since(Instant::now());
        wanted.wait(seen, if listed.is_some() { left.min(WATCH) } else { left });
    }
    if let Some((_, dir)) = dir {
        close(dir, announced);
    }
    if let Some(map) = &map {
        map.close();
    }
    server.set_public(None);
}

pub fn lan_addr(bound: SocketAddr) -> Option<SocketAddr> {
    let ip = match bound.ip().is_unspecified() {
        false => bound.ip(),
        true => {
            let probe = std::net::UdpSocket::bind("0.0.0.0:0").ok()?;
            probe.connect("192.0.2.1:9").ok()?;
            probe.local_addr().ok()?.ip()
        }
    };
    private(ip).then_some(SocketAddr::new(ip, bound.port()))
}

fn wait_for_server<C: Clone + PartialEq>(
    find: &impl Fn() -> Option<Arc<iqstream::Server>>,
    wanted: &Wanted<C>,
) -> Option<Arc<iqstream::Server>> {
    loop {
        let (seen, listing) = wanted.current();
        listing.as_ref()?;
        if let Some(s) = find() {
            return Some(s);
        }
        wanted.wait(seen, SERVER_POLL);
    }
}

fn mapped(
    map: &mut Option<PortMap>,
    local: u16,
    state: &Mutex<ListingState>,
) -> Result<(String, u16, Option<u16>), String> {
    let m = match map.as_mut() {
        None => {
            set(state, ListingState::Mapping);
            let port = NonZeroU16::new(local).ok_or("the server has no port")?;
            map.insert(PortMap::open(port)?).mapped()
        }
        Some(m) if m.renew_in().is_zero() => m.renew()?,
        Some(m) => m.mapped(),
    };
    let tcp = m.tcp.ok_or("the router did not open the port")?;
    Ok((tcp.ip().to_string(), tcp.port(), m.udp.map(|u| u.port())))
}

fn close<D: SdrDirectory>(dir: D, announced: bool) {
    if announced && let Err(e) = dir.withdraw() {
        tracing::warn!("iqstream listing: withdraw: {e}");
    }
    dir.close();
}

fn announce<D: SdrDirectory>(
    listing: &Offer<D::Config>,
    entry: &Entry,
    dir: &mut Option<(D::Config, D)>,
    announced: &mut bool,
) -> ListingState {
    if dir.as_ref().is_none_or(|(config, _)| *config != listing.directory) {
        if let Some((_, old)) = dir.take() {
            close(old, std::mem::take(announced));
        }
        match D::open(&listing.directory, DIRECTORY_WAIT) {
            Ok(d) => *dir = Some((listing.directory.clone(), d)),
            Err(e) => return ListingState::Refused(e.to_string()),
        }
    }
    let Some((_, d)) = dir.as_ref() else {
        return ListingState::Refused("no directory".into());
    };
    match d.announce(entry) {
        Ok(p) => {
            *announced = true;
            ListingState::Listed { at: entry.addr(), servers: p.accepted }
        }
        Err(e) => ListingState::Refused(e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_lan_address_is_only_ever_a_private_one() {
        let at = |a: &str| lan_addr(a.parse().unwrap());
        assert_eq!(at("10.1.2.3:1234"), Some("10.1.2.3:1234".parse().unwrap()));
        assert_eq!(at("203.0.113.9:1234"), None);
        assert_eq!(at("127.0.0.1:1234"), None);
        assert!(at("0.0.0.0:1234").is_none_or(|a| a.port() == 1234 && private(a.ip())));
    }

    type Heard = Arc<Mutex<Vec<(Instant, Entry)>>>;

    #[derive(Clone, Default)]
    struct Log(Heard);

    impl PartialEq for Log {
        fn eq(&self, other: &Log) -> bool {
            Arc::ptr_eq(&self.0, &other.0)
        }
    }

    impl std::fmt::Debug for Log {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            f.write_str("Log")
        }
    }

    struct Recorder(Heard);

    impl SdrDirectory for Recorder {
        type Config = Log;

        fn open(config: &Log, _: Duration) -> Result<Recorder, crate::Error> {
            Ok(Recorder(config.0.clone()))
        }

        fn author(&self) -> Option<crate::Author> {
            None
        }

        fn announce(&self, entry: &Entry) -> Result<crate::Published, crate::Error> {
            self.0.lock().unwrap().push((Instant::now(), entry.clone()));
            Ok(crate::Published { accepted: 1, refused: Vec::new() })
        }

        fn withdraw(&self) -> Result<crate::Published, crate::Error> {
            Ok(crate::Published { accepted: 1, refused: Vec::new() })
        }

        fn list(&self, _: Duration) -> Result<Vec<crate::Listing>, crate::Error> {
            Ok(Vec::new())
        }

        fn list_near(&self, _: &str, _: Duration) -> Result<Vec<crate::Listing>, crate::Error> {
            Ok(Vec::new())
        }

        fn close(self) {}
    }

    fn centres(log: &Log) -> Vec<(Instant, u64, bool)> {
        let heard = log.0.lock().unwrap();
        heard
            .iter()
            .map(|(at, e)| (*at, e.station.tuners[0].center_hz, e.station.location.is_some()))
            .collect()
    }

    fn until(log: &Log, n: usize) -> Vec<(Instant, u64, bool)> {
        let started = Instant::now();
        loop {
            let heard = centres(log);
            if heard.len() >= n {
                return heard;
            }
            assert!(started.elapsed() < Duration::from_secs(10), "never {n} announcements");
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    #[test]
    fn a_retune_is_listed_once_it_settles_and_no_sooner_than_the_pace_allows() {
        let server = iqstream::Server::start(
            "127.0.0.1:0".parse().unwrap(),
            iqstream::ServerConfig { name: "test".into(), streams: Vec::new() },
        )
        .unwrap();
        let stream = server.stream_named(iqstream::StreamConfig {
            name: "span".into(),
            center_hz: 433_920_000,
            sample_rate: 2_400_000,
            ..Default::default()
        });
        let log = Log::default();
        let offer = Offer {
            name: "G0ABC".into(),
            description: String::new(),
            antenna: String::new(),
            location: Some(Location::within(51.45, -0.97, crate::Accuracy::Town)),
            public_host: Some("198.51.100.7".into()),
            directory: log.clone(),
        };
        let pace = Pace { settle: Duration::from_millis(200), apart: Duration::from_secs(2) };
        let found = server.clone();
        let lister =
            Lister::<Recorder>::paced(move || Some(found.clone()), offer.clone(), pace).unwrap();
        let first = until(&log, 1)[0];

        stream.retuned(145_800_000);
        std::thread::sleep(Duration::from_millis(500));
        stream.retuned(145_825_000);
        let second = until(&log, 2)[1];
        assert_eq!(second.1, 145_825_000, "the dial where it came to rest, not where it passed");
        assert!(
            second.0 - first.0 >= pace.apart,
            "floor: a retune was listed {:?} after the last listing",
            second.0 - first.0
        );

        lister.update(Offer { location: None, ..offer });
        let third = until(&log, 3)[2];
        assert!(!third.2);
        assert!(
            third.0 - second.0 < pace.apart,
            "ceiling: the operator's change waited for the pace"
        );
        std::thread::sleep(pace.apart + Duration::from_millis(500));
        assert_eq!(centres(&log).len(), 3, "a tuner holding still is not listed again");
        lister.withdraw(Duration::from_secs(2));
    }
}
