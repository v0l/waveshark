use crate::event::ANNOUNCE_EVERY_SECS;
use crate::model::private;
use crate::portmap::PortMap;
use crate::{Directory, Entry, Hardware, Keys, Location, Station, Tuner, Version};
use std::net::SocketAddr;
use std::num::NonZeroU16;
use std::sync::{Arc, Mutex};
use std::time::Duration;
use tokio::sync::watch;

const MAPPING_WAIT: Duration = Duration::from_secs(20);
const RELAY_WAIT: Duration = Duration::from_secs(10);
const SERVER_POLL: Duration = Duration::from_secs(1);
const RETRY: Duration = Duration::from_secs(30);

#[derive(Clone, Debug, PartialEq)]
pub struct Listing {
    pub name: String,
    pub description: String,
    pub antenna: String,
    pub location: Option<(f64, f64)>,
    pub public_host: Option<String>,
    pub nsec: String,
    pub relays: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ListingState {
    Waiting,
    Mapping,
    Unreachable(String),
    Listed { at: String, relays: usize },
    Refused(String),
}

impl ListingState {
    pub fn describe(&self) -> String {
        match self {
            ListingState::Waiting => "waiting for the server".into(),
            ListingState::Mapping => "opening the port on the router".into(),
            ListingState::Unreachable(why) => format!("not listed: {why}"),
            ListingState::Listed { at, relays } => format!("listed as {at} on {relays} relays"),
            ListingState::Refused(why) => format!("not listed: {why}"),
        }
    }
}

pub struct Lister {
    wanted: watch::Sender<Option<Listing>>,
    state: Arc<Mutex<ListingState>>,
    thread: std::thread::JoinHandle<()>,
}

impl Lister {
    pub fn start(
        find: impl Fn() -> Option<Arc<iqstream::Server>> + Send + 'static,
        listing: Listing,
    ) -> std::io::Result<Lister> {
        let (wanted, rx) = watch::channel(Some(listing));
        let state = Arc::new(Mutex::new(ListingState::Waiting));
        let shown = state.clone();
        let thread = std::thread::Builder::new().name("iqstream-list".into()).spawn(move || {
            match tokio::runtime::Builder::new_current_thread().enable_all().build() {
                Ok(rt) => rt.block_on(run(find, rx, shown)),
                Err(e) => set(&shown, ListingState::Refused(format!("tokio runtime: {e}"))),
            }
        })?;
        Ok(Lister { wanted, state, thread })
    }

    pub fn update(&self, listing: Listing) {
        self.wanted.send_if_modified(|w| {
            let moved = w.as_ref() != Some(&listing);
            *w = Some(listing);
            moved
        });
    }

    pub fn state(&self) -> ListingState {
        self.state.lock().map(|s| s.clone()).unwrap_or(ListingState::Waiting)
    }

    pub fn stop(&self) {
        let _ = self.wanted.send(None);
    }

    pub fn finished(&self) -> bool {
        self.thread.is_finished()
    }

    pub fn withdraw(self, within: Duration) {
        withdraw_all(vec![self], within);
    }
}

pub fn withdraw_all(listers: Vec<Lister>, within: Duration) {
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

pub fn entry(
    listing: &Listing,
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

async fn run(
    find: impl Fn() -> Option<Arc<iqstream::Server>>,
    mut wanted: watch::Receiver<Option<Listing>>,
    state: Arc<Mutex<ListingState>>,
) {
    let Some(server) = wait_for_server(&find, &mut wanted).await else { return };
    let local = server.addr().port();
    let mut map: Option<PortMap> = None;
    let mut dir: Option<(Vec<String>, Directory)> = None;
    let mut announced: Option<Keys> = None;
    loop {
        let Some(listing) = wanted.borrow_and_update().clone() else { break };
        let reach = match &listing.public_host {
            Some(host) => Ok((host.clone(), local, Some(local))),
            None => mapped(&mut map, local, &state).await,
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
                .await
            }
        };
        let next = match now {
            ListingState::Listed { .. } => Duration::from_secs(ANNOUNCE_EVERY_SECS),
            _ => RETRY,
        };
        set(&state, now);
        tokio::select! {
            _ = tokio::time::sleep(next) => {}
            changed = wanted.changed() => if changed.is_err() { break },
        }
    }
    if let (Some((_, dir)), Some(keys)) = (&dir, &announced)
        && let Err(e) = dir.withdraw(keys).await
    {
        tracing::warn!("iqstream listing: withdraw: {e}");
    }
    if let Some(map) = &map {
        map.close();
    }
    server.set_public(None);
    if let Some((_, dir)) = dir {
        dir.shutdown().await;
    }
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

async fn wait_for_server(
    find: &impl Fn() -> Option<Arc<iqstream::Server>>,
    wanted: &mut watch::Receiver<Option<Listing>>,
) -> Option<Arc<iqstream::Server>> {
    loop {
        if wanted.borrow().is_none() {
            return None;
        }
        if let Some(s) = find() {
            return Some(s);
        }
        tokio::select! {
            _ = tokio::time::sleep(SERVER_POLL) => {}
            changed = wanted.changed() => changed.ok()?,
        }
    }
}

async fn mapped(
    map: &mut Option<PortMap>,
    local: u16,
    state: &Mutex<ListingState>,
) -> Result<(String, u16, Option<u16>), String> {
    if map.is_none() {
        set(state, ListingState::Mapping);
        let port = NonZeroU16::new(local).ok_or("the server has no port")?;
        *map = Some(PortMap::open(port).await?);
    }
    let map = map.as_ref().ok_or("no port map")?;
    let m = match map.mapped().tcp {
        Some(_) => map.mapped(),
        None => map.wait(MAPPING_WAIT).await,
    };
    let tcp = m.tcp.ok_or("the router did not open the port")?;
    Ok((tcp.ip().to_string(), tcp.port(), m.udp.map(|u| u.port())))
}

async fn announce(
    listing: &Listing,
    entry: &Entry,
    dir: &mut Option<(Vec<String>, Directory)>,
    announced: &mut Option<Keys>,
) -> ListingState {
    let Some(keys) = crate::identity(&listing.nsec) else {
        return ListingState::Refused("no key to sign the listing with".into());
    };
    if dir.as_ref().is_none_or(|(relays, _)| *relays != listing.relays) {
        if let Some((_, old)) = dir.take() {
            old.shutdown().await;
        }
        match Directory::connect(&listing.relays, RELAY_WAIT).await {
            Ok(d) => *dir = Some((listing.relays.clone(), d)),
            Err(e) => return ListingState::Refused(e.to_string()),
        }
    }
    let Some((_, d)) = dir.as_ref() else {
        return ListingState::Refused("no relays".into());
    };
    if let Some(was) = announced.as_ref().filter(|was| was.public_key() != keys.public_key()) {
        let _ = d.withdraw(was).await;
    }
    match d.announce(&keys, entry).await {
        Ok(p) => {
            *announced = Some(keys);
            ListingState::Listed { at: entry.addr(), relays: p.accepted }
        }
        Err(e) => ListingState::Refused(e.to_string()),
    }
}
