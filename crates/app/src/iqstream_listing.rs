use nostr_directory::{Config, NostrDirectory};
use sdr_directory::SdrDirectory;
use sdr_directory::airspy::{self, AirspyDirectory};
pub use sdr_directory::lister::ListingState;
use sdr_directory::lister::{Lister, Offer};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::{Mutex, OnceLock};
use std::time::Duration;

pub type Listing = Offer<Config>;
pub type AirspyListing = Offer<airspy::Config>;

struct Table<D: SdrDirectory>(Mutex<HashMap<SocketAddr, Lister<D>>>);

impl<D: SdrDirectory + 'static> Table<D> {
    fn new() -> Self {
        Self(Mutex::new(HashMap::new()))
    }

    fn list(&self, addr: SocketAddr, listing: Option<Offer<D::Config>>) {
        let Ok(mut table) = self.0.lock() else { return };
        match (table.get(&addr), listing) {
            (Some(l), Some(listing)) => l.update(listing),
            (Some(_), None) => {
                if let Some(l) = table.remove(&addr) {
                    l.stop();
                }
            }
            (None, Some(listing)) => {
                match Lister::start(move || nodes::iqstream_nodes::running(addr), listing) {
                    Ok(l) => {
                        table.insert(addr, l);
                    }
                    Err(e) => tracing::warn!("listing: {e}"),
                }
            }
            (None, None) => {}
        }
    }

    fn follow(
        &self,
        was: Option<(SocketAddr, Offer<D::Config>)>,
        now: Option<(SocketAddr, Offer<D::Config>)>,
    ) {
        if let Some((at, _)) = &was
            && now.as_ref().is_none_or(|(addr, _)| addr != at)
        {
            self.list(*at, None);
        }
        if let Some((addr, listing)) = now {
            self.list(addr, Some(listing));
        }
    }

    fn drain(&self) -> Vec<Lister<D>> {
        match self.0.lock() {
            Ok(mut table) => table.drain().map(|(_, l)| l).collect(),
            Err(_) => Vec::new(),
        }
    }

    fn state(&self, addr: SocketAddr) -> Option<ListingState> {
        self.0.lock().ok()?.get(&addr).map(Lister::state)
    }
}

fn nostr() -> &'static Table<NostrDirectory> {
    static TABLE: OnceLock<Table<NostrDirectory>> = OnceLock::new();
    TABLE.get_or_init(Table::new)
}

fn airspy() -> &'static Table<AirspyDirectory> {
    static TABLE: OnceLock<Table<AirspyDirectory>> = OnceLock::new();
    TABLE.get_or_init(Table::new)
}

#[cfg(test)]
pub fn list(addr: SocketAddr, listing: Option<Listing>) {
    nodes::iqstream_nodes::describe_listing_with(|addr| state(addr).map(|s| s.describe()));
    nostr().list(addr, listing);
}

pub fn follow(was: Option<(SocketAddr, Listing)>, now: Option<(SocketAddr, Listing)>) {
    nodes::iqstream_nodes::describe_listing_with(|addr| state(addr).map(|s| s.describe()));
    nostr().follow(was, now);
}

pub fn follow_airspy(
    was: Option<(SocketAddr, AirspyListing)>,
    now: Option<(SocketAddr, AirspyListing)>,
) {
    airspy().follow(was, now);
}

pub fn withdraw_all(within: Duration) {
    sdr_directory::lister::withdraw_all(airspy().drain(), Duration::ZERO);
    sdr_directory::lister::withdraw_all(nostr().drain(), within);
}

pub fn state(addr: SocketAddr) -> Option<ListingState> {
    nostr().state(addr)
}

pub fn airspy_state(addr: SocketAddr) -> Option<ListingState> {
    airspy().state(addr)
}

#[cfg(test)]
pub fn offered(addr: SocketAddr) -> Option<Listing> {
    nostr().0.lock().ok()?.get(&addr)?.offer()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nostr_directory::mock::MockRelay;
    use sdr_directory::{Author, Hardware, SdrDirectory};
    use std::time::Instant;

    const WAIT: Duration = Duration::from_secs(10);

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
        let relay = MockRelay::run().unwrap();
        let url = relay.url().to_string();
        let addr = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap();
        let server = nodes::iqstream_nodes::server(addr).unwrap();
        let stream = server.stream_named(iqstream::StreamConfig {
            name: "span".into(),
            center_hz: 433_920_000,
            sample_rate: 2_400_000,
            ..Default::default()
        });
        stream.set_hardware("hackrf");
        let (keys, nsec) = nostr_directory::new_identity();
        let listing = Listing {
            name: "G0ABC".into(),
            description: "Loft".into(),
            antenna: "whip".into(),
            location: Some(sdr_directory::Location::within(
                51.45,
                -0.97,
                sdr_directory::Accuracy::Town,
            )),
            public_host: Some("198.51.100.7".into()),
            directory: Config::publisher(&nsec, &[url.as_str()]),
        };
        list(addr, Some(listing.clone()));
        let at = format!("198.51.100.7:{}", addr.port());
        until("listed", || match state(addr) {
            Some(ListingState::Listed { at: listed, servers: 1 }) if listed == at => Some(()),
            _ => None,
        });
        assert_eq!(server.public(), None, "a host given outright is not behind a NAT");

        let reader = NostrDirectory::open(&Config::reader(&[url.as_str()]), WAIT).unwrap();
        let listed = reader.list(WAIT).unwrap();
        assert_eq!(listed.len(), 1);
        assert_eq!(listed[0].author, Author::from(keys.public_key()));
        assert_eq!(listed[0].entry.addr(), at);
        let tuners = &listed[0].entry.station.tuners;
        assert_eq!(tuners.len(), 1);
        assert_eq!(
            (&tuners[0].name, &tuners[0].hardware, tuners[0].center_hz, tuners[0].antenna.as_str()),
            (&"span".to_string(), &Hardware::HackRf, 433_920_000, "whip")
        );

        let unlocated = Listing { location: None, ..listing.clone() };
        list(addr, Some(unlocated.clone()));
        until("the location taken off the listing", || {
            reader.list(WAIT).unwrap()[0].entry.station.location.is_none().then_some(())
        });

        list(addr, Some(unlocated));
        list(addr, None);
        assert_eq!(state(addr), None);
        until("withdrawn", || reader.list(WAIT).unwrap().is_empty().then_some(()));

        std::thread::sleep(Duration::from_millis(2_100));
        list(addr, Some(listing));
        until("listed again, past a deletion dated a second ahead by the relisting", || {
            matches!(state(addr), Some(ListingState::Listed { .. })).then_some(())
        });
        let started = Instant::now();
        withdraw_all(Duration::from_secs(5));
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "ceiling: withdrawing outlasted the wait"
        );
        assert_eq!(state(addr), None);
        assert!(reader.list(WAIT).unwrap().is_empty(), "gone before exit returns");
        relay.shutdown();
    }
}
