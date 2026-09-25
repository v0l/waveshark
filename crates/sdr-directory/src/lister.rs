use crate::model::private;
use crate::portmap::PortMap;
use crate::{
    ANNOUNCE_EVERY_SECS, Entry, Hardware, Location, SdrDirectory, Station, Tuner, Version,
};
use std::net::SocketAddr;
use std::num::NonZeroU16;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

const DIRECTORY_WAIT: Duration = Duration::from_secs(10);
const SERVER_POLL: Duration = Duration::from_secs(1);
const RETRY: Duration = Duration::from_secs(30);

#[derive(Clone, Debug, PartialEq)]
pub struct Offer<C> {
    pub name: String,
    pub description: String,
    pub antenna: String,
    pub location: Option<(f64, f64)>,
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
        let wanted = Arc::new(Wanted::new());
        wanted.set(Some(listing), false);
        let state = Arc::new(Mutex::new(ListingState::Waiting));
        let (shown, asked) = (state.clone(), wanted.clone());
        let thread = std::thread::Builder::new()
            .name("iqstream-list".into())
            .spawn(move || run::<D>(find, &asked, &shown))?;
        Ok(Lister { wanted, state, thread })
    }

    pub fn update(&self, listing: Offer<D::Config>) {
        self.wanted.set(Some(listing), true);
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
            location: listing.location.map(|(lat, lon)| Location { lat, lon }),
            version: Version::OURS,
            clients: streams.iter().map(|s| s.subscribers() as u32).sum(),
            max_clients: None,
            session_limit_secs: None,
            tuners,
        },
    }
}

fn run<D: SdrDirectory>(
    find: impl Fn() -> Option<Arc<iqstream::Server>>,
    wanted: &Wanted<D::Config>,
    state: &Mutex<ListingState>,
) {
    let Some(server) = wait_for_server(&find, wanted) else { return };
    let local = server.addr().port();
    let mut map: Option<PortMap> = None;
    let mut dir: Option<(D::Config, D)> = None;
    let mut announced = false;
    while let (seen, Some(listing)) = wanted.current() {
        let reach = match &listing.public_host {
            Some(host) => Ok((host.clone(), local, Some(local))),
            None => mapped(&mut map, local, state),
        };
        let now = match reach {
            Err(why) => ListingState::Unreachable(why),
            Ok((host, port, data_port)) => {
                server.set_public(listing.public_host.is_none().then(|| iqstream::Public {
                    addr: SocketAddr::new(host.parse().unwrap_or(server.addr().ip()), port),
                    data_port,
                }));
                let also: Vec<SocketAddr> = listing
                    .public_host
                    .is_none()
                    .then(|| lan_addr(server.addr()))
                    .flatten()
                    .into_iter()
                    .collect();
                announce(
                    &listing,
                    &entry(&listing, &host, port, data_port, also, &server),
                    &mut dir,
                    &mut announced,
                )
            }
        };
        let mut next = match now {
            ListingState::Listed { .. } => Duration::from_secs(ANNOUNCE_EVERY_SECS),
            _ => RETRY,
        };
        if let Some(m) = &map {
            next = next.min(m.renew_in());
        }
        set(state, now);
        wanted.wait(seen, next);
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
}
