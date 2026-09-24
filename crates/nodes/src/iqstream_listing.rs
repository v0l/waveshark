use iqdirectory::lister::Lister;
pub use iqdirectory::lister::{Listing, ListingState};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::{Mutex, OnceLock};
use std::time::Duration;

fn listers() -> &'static Mutex<HashMap<SocketAddr, Lister>> {
    static LISTERS: OnceLock<Mutex<HashMap<SocketAddr, Lister>>> = OnceLock::new();
    LISTERS.get_or_init(Default::default)
}

pub fn list(addr: SocketAddr, listing: Option<Listing>) {
    let Ok(mut table) = listers().lock() else { return };
    match (table.get(&addr), listing) {
        (Some(l), Some(listing)) => l.update(listing),
        (Some(_), None) => {
            if let Some(l) = table.remove(&addr) {
                l.stop();
            }
        }
        (None, Some(listing)) => {
            match Lister::start(move || crate::iqstream_nodes::running(addr), listing) {
                Ok(l) => {
                    table.insert(addr, l);
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
    iqdirectory::lister::withdraw_all(all, within);
}

pub fn state(addr: SocketAddr) -> Option<ListingState> {
    listers().lock().ok()?.get(&addr).map(Lister::state)
}

#[cfg(test)]
mod tests {
    use super::*;
    use iqdirectory::{Directory, Hardware};
    use nostr_sdk::prelude::MockRelay;
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

        let reader = rt.block_on(Directory::connect(&[url.as_str()], WAIT)).unwrap();
        let listed = rt.block_on(reader.list(WAIT)).unwrap();
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
        until("withdrawn", || rt.block_on(reader.list(WAIT)).unwrap().is_empty().then_some(()));

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
        assert!(rt.block_on(reader.list(WAIT)).unwrap().is_empty(), "gone before exit returns");
        relay.shutdown();
    }
}
