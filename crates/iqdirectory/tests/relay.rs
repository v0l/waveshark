use iqdirectory::model::{Dial, Hardware, Location, Station, Tuner, Version};
use iqdirectory::{Directory, Entry, Query, new_identity};
use nostr_sdk::prelude::{MockRelay, Timestamp};
use std::time::Duration;

const WAIT: Duration = Duration::from_secs(5);

fn entry(host: &str, center_hz: u64) -> Entry {
    Entry {
        host: host.into(),
        port: 5557,
        data_port: None,
        station: Station {
            name: host.into(),
            description: String::new(),
            location: Some(Location { lat: 51.45, lon: -0.97 }),
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

fn hosts(l: &[iqdirectory::Listing]) -> Vec<&str> {
    let mut h: Vec<&str> = l.iter().map(|l| l.entry.host.as_str()).collect();
    h.sort();
    h
}

#[tokio::test(flavor = "multi_thread")]
async fn two_stations_announce_one_moves_one_withdraws_and_a_reader_sees_each_step() {
    let relay = MockRelay::run().await.unwrap();
    let url = relay.url().await.to_string();
    let (a, _) = new_identity();
    let (b, _) = new_identity();
    let dir = Directory::connect(&[url.as_str()], WAIT).await.unwrap();

    assert_eq!(dir.announce(&a, &entry("a.example", 125_000_000)).await.unwrap().accepted, 1);
    assert_eq!(dir.announce(&b, &entry("b.example", 433_920_000)).await.unwrap().accepted, 1);
    let reader = Directory::connect(&[url.as_str()], WAIT).await.unwrap();
    let listed = reader.list(WAIT).await.unwrap();
    assert_eq!(hosts(&listed), ["a.example", "b.example"]);
    let now = Timestamp::now().as_secs();
    let at_433 = Query { hz: Some(433_920_000), ..Query::default() };
    let kept: Vec<&str> =
        listed.iter().filter(|l| at_433.keeps(l, now)).map(|l| l.entry.host.as_str()).collect();
    assert_eq!(kept, ["b.example"]);

    tokio::time::sleep(Duration::from_millis(1_100)).await;
    dir.announce(&a, &entry("a.example", 1_090_000_000)).await.unwrap();
    let listed = reader.list(WAIT).await.unwrap();
    assert_eq!(listed.len(), 2, "one listing per station");
    let moved = listed.iter().find(|l| l.author == a.public_key()).unwrap();
    assert_eq!(moved.entry.station.tuners[0].center_hz, 1_090_000_000);

    dir.withdraw(&b).await.unwrap();
    let listed = reader.list(WAIT).await.unwrap();
    assert_eq!(hosts(&listed), ["a.example"]);

    let near = reader.list_near("gcpk9y", WAIT).await.unwrap();
    assert_eq!(hosts(&near), ["a.example"], "a relay finds the station by its geohash");
    assert_eq!(reader.list_near("gcpvj0", WAIT).await.unwrap().len(), 0, "London is not Reading");
    assert_eq!(reader.list_near("gcp", WAIT).await.unwrap().len(), 1, "a coarser cell holds it");

    dir.shutdown().await;
    reader.shutdown().await;
    relay.shutdown();
}
