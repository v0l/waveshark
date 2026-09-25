use sdr_directory::portmap::PortMap;
use std::num::NonZeroU16;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let port: u16 = std::env::args().nth(1).and_then(|p| p.parse().ok()).unwrap_or(5557);
    let map = PortMap::open(NonZeroU16::new(port).expect("a port above zero")).unwrap();
    println!("via {}", map.via());
    println!("mapped {:?}, renewing in {:?}", map.mapped(), map.renew_in());
    map.close();
}
