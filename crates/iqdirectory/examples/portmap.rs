use iqdirectory::portmap::PortMap;
use std::num::NonZeroU16;
use std::time::Duration;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let port: u16 = std::env::args().nth(1).and_then(|p| p.parse().ok()).unwrap_or(5557);
    let map = PortMap::open(NonZeroU16::new(port).expect("a port above zero")).await.unwrap();
    println!("gateway {:?}", map.gateway());
    println!("mapped {:?}", map.wait(Duration::from_secs(15)).await);
    map.close();
    tokio::time::sleep(Duration::from_millis(500)).await;
}
