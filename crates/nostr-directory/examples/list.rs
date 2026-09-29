use common::time::{Duration, Instant};
use nostr_directory::{Config, NostrDirectory};
use sdr_directory::SdrDirectory;

fn main() {
    let relays: Vec<String> = std::env::args().skip(1).collect();
    let t = Instant::now();
    let dir = NostrDirectory::open(&Config::reader(&relays), Duration::from_secs(10)).unwrap();
    println!("opened in {:?}", t.elapsed());
    let listed = dir.list(Duration::from_secs(10)).unwrap();
    println!("{} listings in {:?}", listed.len(), t.elapsed());
    for l in &listed {
        println!(
            "{} {} {} {:?}",
            l.author,
            l.entry.addr(),
            l.entry.station.name,
            l.entry.webtransport
        );
    }
    dir.close();
}
