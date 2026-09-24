use iqdirectory::event::ANNOUNCE_EVERY_SECS;
use iqdirectory::portmap::PortMap;
use iqdirectory::{Directory, Entry, Hardware, Keys, Location, Station, Tuner, Version};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::num::NonZeroU16;
use std::sync::{Arc, Mutex, OnceLock};
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

struct Lister {
    wanted: watch::Sender<Option<Listing>>,
    state: Arc<Mutex<ListingState>>,
    thread: std::thread::JoinHandle<()>,
}

fn listers() -> &'static Mutex<HashMap<SocketAddr, Lister>> {
    static LISTERS: OnceLock<Mutex<HashMap<SocketAddr, Lister>>> = OnceLock::new();
    LISTERS.get_or_init(Default::default)
}

pub fn list(addr: SocketAddr, listing: Option<Listing>) {
    let Ok(mut table) = listers().lock() else { return };
    match (table.get(&addr), listing) {
        (Some(l), Some(listing)) => {
            l.wanted.send_if_modified(|w| {
                let moved = w.as_ref() != Some(&listing);
                *w = Some(listing);
                moved
            });
        }
        (Some(_), None) => {
            if let Some(l) = table.remove(&addr) {
                let _ = l.wanted.send(None);
            }
        }
        (None, Some(listing)) => {
            let (wanted, rx) = watch::channel(Some(listing));
            let state = Arc::new(Mutex::new(ListingState::Waiting));
            let shown = state.clone();
            let spawned =
                std::thread::Builder::new().name("iqstream-list".into()).spawn(move || {
                    match tokio::runtime::Builder::new_current_thread().enable_all().build() {
                        Ok(rt) => rt.block_on(run(addr, rx, shown)),
                        Err(e) => set(&shown, ListingState::Refused(format!("tokio runtime: {e}"))),
                    }
                });
            match spawned {
                Ok(thread) => {
                    table.insert(addr, Lister { wanted, state, thread });
                }
                Err(e) => tracing::warn!("iqstream listing: {e}"),
            }
        }
        (None, None) => {}
    }
}

pub fn withdraw_all(within: Duration) {
    let all: Vec<Lister> = match listers().lock() {
        Ok(mut table) => table.drain().map(|(_, l)| l).collect(),
        Err(_) => return,
    };
    for l in &all {
        let _ = l.wanted.send(None);
    }
    let started = std::time::Instant::now();
    while all.iter().any(|l| !l.thread.is_finished()) && started.elapsed() < within {
        std::thread::sleep(Duration::from_millis(20));
    }
}

pub fn state(addr: SocketAddr) -> Option<ListingState> {
    let table = listers().lock().ok()?;
    let l = table.get(&addr)?;
    l.state.lock().ok().map(|s| s.clone())
}

fn set(state: &Mutex<ListingState>, now: ListingState) {
    if let Ok(mut s) = state.lock() {
        *s = now;
    }
}

pub fn entry(
    listing: &Listing,
    host: &str,
    port: u16,
    data_port: Option<u16>,
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
    addr: SocketAddr,
    mut wanted: watch::Receiver<Option<Listing>>,
    state: Arc<Mutex<ListingState>>,
) {
    let Some(server) = wait_for_server(addr, &mut wanted).await else { return };
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
                    addr: SocketAddr::new(host.parse().unwrap_or(addr.ip()), port),
                    data_port,
                }));
                announce(
                    &listing,
                    &entry(&listing, &host, port, data_port, &server),
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

async fn wait_for_server(
    addr: SocketAddr,
    wanted: &mut watch::Receiver<Option<Listing>>,
) -> Option<Arc<iqstream::Server>> {
    loop {
        if wanted.borrow().is_none() {
            return None;
        }
        if let Some(s) = crate::iqstream_nodes::running(addr) {
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
    let Some(keys) = iqdirectory::identity(&listing.nsec) else {
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

#[cfg(test)]
mod tests {
    use super::*;
    use nostr_sdk::prelude::MockRelay;
    use std::time::Instant;

    fn until<T>(what: &str, mut f: impl FnMut() -> Option<T>) -> T {
        let started = Instant::now();
        loop {
            if let Some(v) = f() {
                return v;
            }
            assert!(started.elapsed() < Duration::from_secs(15), "never {what}");
            std::thread::sleep(Duration::from_millis(50));
        }
    }

    #[test]
    fn a_served_tuner_is_listed_with_its_hardware_and_withdrawn_when_serving_stops() {
        let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build().unwrap();
        let relay = rt.block_on(MockRelay::run()).unwrap();
        let url = rt.block_on(relay.url()).to_string();
        let addr = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap();
        let server = crate::iqstream_nodes::server(addr).unwrap();
        let stream = server.stream_named(iqstream::StreamConfig {
            name: "span".into(),
            center_hz: 433_920_000,
            sample_rate: 2_400_000,
            ..Default::default()
        });
        stream.set_hardware("hackrf");
        let (keys, nsec) = iqdirectory::new_identity();
        let listing = Listing {
            name: "G0ABC".into(),
            description: "Loft".into(),
            antenna: "whip".into(),
            location: Some((51.45, -0.97)),
            public_host: Some("198.51.100.7".into()),
            nsec,
            relays: vec![url.clone()],
        };
        list(addr, Some(listing.clone()));
        let at = format!("198.51.100.7:{}", addr.port());
        until("listed", || match state(addr) {
            Some(ListingState::Listed { at: listed, relays: 1 }) if listed == at => Some(()),
            _ => None,
        });
        assert_eq!(server.public(), None, "a host given outright is not behind a NAT");

        let reader = rt.block_on(Directory::connect(&[url.as_str()], RELAY_WAIT)).unwrap();
        let listed = rt.block_on(reader.list(RELAY_WAIT)).unwrap();
        assert_eq!(listed.len(), 1);
        assert_eq!(listed[0].author, keys.public_key());
        assert_eq!(listed[0].entry.addr(), at);
        let tuners = &listed[0].entry.station.tuners;
        assert_eq!(tuners.len(), 1);
        assert_eq!(
            (&tuners[0].name, &tuners[0].hardware, tuners[0].center_hz, tuners[0].antenna.as_str()),
            (&"span".to_string(), &Hardware::HackRf, 433_920_000, "whip")
        );

        list(addr, Some(listing.clone()));
        list(addr, None);
        assert_eq!(state(addr), None);
        until("withdrawn", || {
            rt.block_on(reader.list(RELAY_WAIT)).unwrap().is_empty().then_some(())
        });

        std::thread::sleep(Duration::from_millis(1_100));
        list(addr, Some(listing));
        until("listed again, a second after the deletion that covers its own second", || {
            matches!(state(addr), Some(ListingState::Listed { .. })).then_some(())
        });
        let started = Instant::now();
        withdraw_all(Duration::from_secs(5));
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "ceiling: withdrawing outlasted the wait"
        );
        assert_eq!(state(addr), None);
        assert!(
            rt.block_on(reader.list(RELAY_WAIT)).unwrap().is_empty(),
            "gone before exit returns"
        );
        relay.shutdown();
    }
}
