use crate::model::private;
use crate::portmap::PortMap;
use crate::{
    ANNOUNCE_EVERY_SECS, Entry, Hardware, Location, Protocol, SdrDirectory, Station, Tuner, Version,
};
use std::net::SocketAddr;
use std::num::NonZeroU16;
use std::sync::{Arc, Condvar, Mutex};
use web_time::{Duration, Instant};

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
    thread: common::thread::JoinHandle<()>,
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
        Lister::mapping(find, listing, pace, PortMap::open)
    }

    fn mapping(
        find: impl Fn() -> Option<Arc<iqstream::Server>> + Send + 'static,
        listing: Offer<D::Config>,
        pace: Pace,
        open: impl Fn(NonZeroU16) -> Result<PortMap, String> + Send + 'static,
    ) -> std::io::Result<Lister<D>> {
        let wanted = Arc::new(Wanted::new());
        wanted.set(Some(listing), false);
        let state = Arc::new(Mutex::new(ListingState::Waiting));
        let (shown, asked) = (state.clone(), wanted.clone());
        let thread = common::thread::Builder::new()
            .name("iqstream-list".into())
            .spawn(move || run::<D>(find, &asked, &shown, pace, &open))?;
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
    let started = web_time::Instant::now();
    while listers.iter().any(|l| !l.finished()) && started.elapsed() < within {
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn set(state: &Mutex<ListingState>, now: ListingState) {
    if let Ok(mut s) = state.lock()
        && *s != now
    {
        match now {
            ListingState::Unreachable(_) | ListingState::Refused(_) => {
                tracing::warn!("iqstream directory: {}", now.describe())
            }
            _ => tracing::info!("iqstream directory: {}", now.describe()),
        }
        *s = now;
    }
}

pub fn entry<C>(
    listing: &Offer<C>,
    host: &str,
    port: u16,
    data_port: Option<u16>,
    also: Vec<SocketAddr>,
    webtransport: Option<u16>,
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
        webtransport: webtransport.zip(server.webtransport()).map(|(port, offered)| {
            let at = match host.contains(':') {
                true => format!("[{host}]:{port}"),
                false => format!("{host}:{port}"),
            };
            iqstream::ws::webtransport_url(&at, &offered.hex())
        }),
        webrtc: server.webrtc(),
        station: Station {
            name: listing.name.clone(),
            description: listing.description.clone(),
            location: listing.location,
            protocol: Protocol::IqStream(Version::OURS),
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
    webtransport: Option<u16>,
}

impl Reach {
    fn entry<C>(&self, listing: &Offer<C>, server: &iqstream::Server) -> Entry {
        let (host, port, data_port) = (&self.host, self.port, self.data_port);
        entry(listing, host, port, data_port, self.also.clone(), self.webtransport, server)
    }
}

fn reach<C>(
    listing: &Offer<C>,
    server: &iqstream::Server,
    maps: &mut Maps,
    state: &Mutex<ListingState>,
    open: &impl Fn(NonZeroU16) -> Result<PortMap, String>,
) -> Result<Reach, String> {
    let local = server.addr().port();
    let (host, port, data_port) = match &listing.public_host {
        Some(host) => (host.clone(), local, Some(local)),
        None => mapped(&mut maps.iq, local, state, open)?,
    };
    let webtransport = server.webtransport().and_then(|o| match &listing.public_host {
        Some(_) => Some(o.port),
        None => match mapped(&mut maps.webtransport, o.port, state, open) {
            Ok((_, _, udp)) => udp,
            Err(e) => {
                tracing::debug!("iqstream directory: webtransport port {}: {e}", o.port);
                None
            }
        },
    });
    server.set_public(listing.public_host.is_none().then(|| iqstream::Public {
        addr: SocketAddr::new(host.parse().unwrap_or(server.addr().ip()), port),
        data_port,
    }));
    let also = match listing.public_host {
        Some(_) => Vec::new(),
        None => lan_addr(server.addr()).into_iter().collect(),
    };
    Ok(Reach { host, port, data_port, also, webtransport })
}

fn run<D: SdrDirectory>(
    find: impl Fn() -> Option<Arc<iqstream::Server>>,
    wanted: &Wanted<D::Config>,
    state: &Mutex<ListingState>,
    pace: Pace,
    open: &impl Fn(NonZeroU16) -> Result<PortMap, String>,
) {
    let Some(server) = wait_for_server(&find, wanted) else { return };
    let mut maps = Maps::default();
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
            let now = match reach(&listing, &server, &mut maps, state, open) {
                Err(why) => {
                    listed = None;
                    if let Some((_, d)) = &dir {
                        withdraw(d, &mut announced);
                    }
                    ListingState::Unreachable(why)
                }
                Ok(r) => {
                    let e = r.entry(&listing, &server);
                    let now = announce(&listing, &e, &mut dir, &mut announced);
                    listed = matches!(now, ListingState::Listed { .. }).then_some((r, e));
                    now
                }
            };
            let mut next = match (&listed, &dir) {
                (Some(_), Some((_, d))) => d.every(),
                (Some(_), None) => Duration::from_secs(ANNOUNCE_EVERY_SECS),
                (None, _) => RETRY,
            };
            for renew in maps.renew_in() {
                next = next.min(renew);
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
    for map in [&maps.iq, &maps.webtransport].into_iter().flatten() {
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

#[derive(Default)]
struct Maps {
    iq: Option<PortMap>,
    webtransport: Option<PortMap>,
}

impl Maps {
    fn renew_in(&self) -> impl Iterator<Item = Duration> + '_ {
        [&self.iq, &self.webtransport].into_iter().flatten().filter_map(PortMap::renew_in)
    }
}

fn mapped(
    map: &mut Option<PortMap>,
    local: u16,
    state: &Mutex<ListingState>,
    open: &impl Fn(NonZeroU16) -> Result<PortMap, String>,
) -> Result<(String, u16, Option<u16>), String> {
    let m = match map.as_mut() {
        None => {
            set(state, ListingState::Mapping);
            let port = NonZeroU16::new(local).ok_or("the server has no port")?;
            map.insert(open(port)?).mapped()
        }
        Some(m) if m.renew_in().is_none_or(|d| d.is_zero()) => m.renew()?,
        Some(m) => m.mapped(),
    };
    let tcp = m.tcp.ok_or("the router did not open the port")?;
    Ok((tcp.ip().to_string(), tcp.port(), m.udp.map(|u| u.port())))
}

fn close<D: SdrDirectory>(dir: D, mut announced: bool) {
    withdraw(&dir, &mut announced);
    dir.close();
}

fn withdraw<D: SdrDirectory>(dir: &D, announced: &mut bool) {
    if std::mem::take(announced)
        && let Err(e) = dir.withdraw()
    {
        tracing::warn!("iqstream listing: withdraw: {e}");
    }
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

    type Heard = Arc<Mutex<Vec<(Instant, Option<Entry>)>>>;

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
            self.0.lock().unwrap().push((Instant::now(), Some(entry.clone())));
            Ok(crate::Published { accepted: 1, refused: Vec::new() })
        }

        fn withdraw(&self) -> Result<crate::Published, crate::Error> {
            self.0.lock().unwrap().push((Instant::now(), None));
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
            .filter_map(|(at, e)| Some((*at, e.as_ref()?)))
            .map(|(at, e)| (at, e.station.tuners[0].center_hz, e.station.location.is_some()))
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
            iqstream::ServerConfig { name: "test".into(), ..Default::default() },
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

    #[test]
    fn a_server_offering_webtransport_is_listed_with_its_port_and_certificates() {
        let server = iqstream::Server::start(
            "127.0.0.1:0".parse().unwrap(),
            iqstream::ServerConfig { webtransport: Some(0), ..Default::default() },
        )
        .unwrap();
        server.stream_named(iqstream::StreamConfig {
            name: "span".into(),
            center_hz: 1_090_000_000,
            sample_rate: 2_400_000,
            ..Default::default()
        });
        let offered = server.webtransport().unwrap();
        let log = Log::default();
        let offer = Offer {
            name: "G0ABC".into(),
            description: String::new(),
            antenna: String::new(),
            location: None,
            public_host: Some("198.51.100.7".into()),
            directory: log.clone(),
        };
        let found = server.clone();
        let lister =
            Lister::<Recorder>::paced(move || Some(found.clone()), offer, Pace::LIVE).unwrap();
        until(&log, 1);
        let listed = log.0.lock().unwrap()[0].1.clone().unwrap();
        assert_eq!(
            listed.webtransport_url().unwrap(),
            format!("https://198.51.100.7:{}/?cert={}", offered.port, offered.hex().join(","))
        );
        lister.withdraw(Duration::from_secs(2));
    }

    type Asked = Arc<Mutex<Vec<(Instant, u8, u16)>>>;

    fn fake_nat_pmp(lease_secs: u32, grants: Vec<Option<u16>>) -> (std::net::SocketAddrV4, Asked) {
        let sock = std::net::UdpSocket::bind("127.0.0.1:0").unwrap();
        let std::net::SocketAddr::V4(at) = sock.local_addr().unwrap() else { unreachable!() };
        let heard = Arc::new(Mutex::new(Vec::new()));
        let log = heard.clone();
        common::thread::spawn(move || {
            let mut buf = [0u8; 64];
            let mut tcp_asked = 0;
            while let Ok((n, from)) = sock.recv_from(&mut buf) {
                let r = &buf[..n];
                let mut answer = vec![0, 128 + r[1], 0, 0, 0, 0, 0, 1];
                match r[1] {
                    0 => answer.extend([203, 0, 113, 9]),
                    op => {
                        let asked = u16::from_be_bytes([r[6], r[7]]);
                        log.lock().unwrap().push((Instant::now(), op, asked));
                        if op == 2 {
                            tcp_asked += 1;
                        }
                        let grant = grants[(tcp_asked - 1).min(grants.len() - 1)];
                        let (code, port) = match grant {
                            Some(p) => (0u16, p + u16::from(op == 1)),
                            None => (3, 0),
                        };
                        answer[2..4].copy_from_slice(&code.to_be_bytes());
                        answer.extend_from_slice(&r[4..6]);
                        answer.extend_from_slice(&port.to_be_bytes());
                        answer.extend_from_slice(&lease_secs.to_be_bytes());
                    }
                }
                let _ = sock.send_to(&answer, from);
            }
        });
        (at, heard)
    }

    #[test]
    fn a_lease_is_renewed_before_it_lapses_a_moved_port_relisted_and_a_refusal_withdrawn() {
        let server = iqstream::Server::start(
            "127.0.0.1:0".parse().unwrap(),
            iqstream::ServerConfig { name: "test".into(), ..Default::default() },
        )
        .unwrap();
        server.stream_named(iqstream::StreamConfig {
            name: "span".into(),
            center_hz: 1_090_000_000,
            sample_rate: 2_400_000,
            ..Default::default()
        });
        let lease = Duration::from_secs(2);
        let (gateway, asked) = fake_nat_pmp(
            lease.as_secs() as u32,
            vec![Some(41_000), Some(41_000), Some(42_000), None],
        );
        let log = Log::default();
        let offer = Offer {
            name: "radarpi".into(),
            description: String::new(),
            antenna: String::new(),
            location: None,
            public_host: None,
            directory: log.clone(),
        };
        let found = server.clone();
        let lister = Lister::<Recorder>::mapping(
            move || Some(found.clone()),
            offer,
            Pace::LIVE,
            move |port| PortMap::nat_pmp_at(gateway, port),
        )
        .unwrap();
        let started = Instant::now();
        while !matches!(lister.state(), ListingState::Unreachable(_)) {
            assert!(started.elapsed() < Duration::from_secs(10), "the refusal was never shown");
            std::thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(
            lister.state().describe(),
            "not listed: the router did not open the port: NAT-PMP result 3"
        );
        lister.withdraw(Duration::from_secs(2));

        let heard: Vec<Option<(String, Option<u16>)>> = log
            .0
            .lock()
            .unwrap()
            .iter()
            .map(|(_, e)| e.as_ref().map(|e| (e.addr(), e.data_port)))
            .collect();
        assert_eq!(
            heard,
            [
                Some(("203.0.113.9:41000".to_string(), Some(41_001))),
                Some(("203.0.113.9:41000".to_string(), Some(41_001))),
                Some(("203.0.113.9:42000".to_string(), Some(42_001))),
                None,
            ],
            "listed, renewed, relisted on the new port, withdrawn when the router refused"
        );

        let asked = asked.lock().unwrap().clone();
        let tcp: Vec<(Instant, u16)> =
            asked.iter().filter(|(_, op, _)| *op == 2).map(|(at, _, port)| (*at, *port)).collect();
        assert_eq!(
            tcp.iter().map(|(_, port)| *port).collect::<Vec<_>>(),
            [server.addr().port(), 41_000, 41_000, 42_000, 0],
            "each renewal asks for the port it holds, then for any port once that is refused"
        );
        for pair in tcp[..4].windows(2) {
            let gap = pair[1].0 - pair[0].0;
            assert!(gap < lease, "ceiling: renewed {gap:?} after a {lease:?} lease was granted");
            assert!(gap >= lease / 2 - Duration::from_millis(50), "floor: renewed after {gap:?}");
        }
    }
}
