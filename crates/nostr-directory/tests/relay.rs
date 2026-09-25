use nostr_directory::mock::MockRelay;
use nostr_directory::{Config, NostrDirectory, new_identity};
use sdr_directory::{
    Accuracy, Author, Dial, Entry, Hardware, Location, Query, SdrDirectory, Station, Tuner,
    Version, now,
};
use std::time::Duration;

const WAIT: Duration = Duration::from_secs(5);

fn entry(host: &str, center_hz: u64) -> Entry {
    Entry {
        host: host.into(),
        port: 5557,
        data_port: None,
        also: Vec::new(),
        station: Station {
            name: host.into(),
            description: String::new(),
            location: Some(Location::within(51.45, -0.97, Accuracy::Town)),
            version: Version::OURS,
            clients: 0,
            max_clients: Some(4),
            session_limit_secs: None,
            tuners: vec![Tuner {
                id: 0,
                name: "rtl0".into(),
                hardware: Hardware::RtlSdr,
                antenna: String::new(),
                center_hz,
                sample_rate: 2_400_000,
                dial: Dial::Fixed,
            }],
        },
    }
}

fn hosts(l: &[sdr_directory::Listing]) -> Vec<&str> {
    let mut h: Vec<&str> = l.iter().map(|l| l.entry.host.as_str()).collect();
    h.sort();
    h
}

#[test]
fn two_stations_announce_one_moves_one_withdraws_and_a_reader_sees_each_step() {
    let relay = MockRelay::run().unwrap();
    let url = std::env::var("NOSTR_DIRECTORY_RELAY").unwrap_or_else(|_| relay.url().to_string());
    let relays = [url.as_str()];
    let (a, a_nsec) = new_identity();
    let (_, b_nsec) = new_identity();
    let as_a = NostrDirectory::open(&Config::publisher(&a_nsec, &relays), WAIT).unwrap();
    let as_b = NostrDirectory::open(&Config::publisher(&b_nsec, &relays), WAIT).unwrap();
    let reader = NostrDirectory::open(&Config::reader(&relays), WAIT).unwrap();
    assert_eq!(as_a.author(), Some(Author::from(a.public_key())));
    assert_eq!(reader.author(), None);
    assert!(reader.announce(&entry("r.example", 1)).is_err(), "a reader has no key to sign with");

    assert_eq!(as_a.announce(&entry("a.example", 125_000_000)).unwrap().accepted, 1);
    assert_eq!(as_b.announce(&entry("b.example", 433_920_000)).unwrap().accepted, 1);
    let listed = reader.list(WAIT).unwrap();
    assert_eq!(hosts(&listed), ["a.example", "b.example"]);
    let at_433 = Query { hz: Some(433_920_000), ..Query::default() };
    let kept: Vec<&str> =
        listed.iter().filter(|l| at_433.keeps(l, now())).map(|l| l.entry.host.as_str()).collect();
    assert_eq!(kept, ["b.example"]);

    std::thread::sleep(Duration::from_millis(1_100));
    as_a.announce(&entry("a.example", 1_090_000_000)).unwrap();
    let listed = reader.list(WAIT).unwrap();
    assert_eq!(listed.len(), 2, "one listing per station");
    let moved = listed.iter().find(|l| Some(&l.author) == as_a.author().as_ref()).unwrap();
    assert_eq!(moved.entry.station.tuners[0].center_hz, 1_090_000_000);

    as_b.withdraw().unwrap();
    let listed = reader.list(WAIT).unwrap();
    assert_eq!(hosts(&listed), ["a.example"]);

    let near = reader.list_near("gcpk9y", WAIT).unwrap();
    assert_eq!(hosts(&near), ["a.example"], "a relay finds the station by its geohash");
    assert_eq!(reader.list_near("gcpvj0", WAIT).unwrap().len(), 0, "London is not Reading");
    assert_eq!(reader.list_near("gcp", WAIT).unwrap().len(), 1, "a coarser cell holds it");

    as_a.withdraw().unwrap();
    for d in [as_a, as_b, reader] {
        d.close();
    }
}
