//! Look for a sub-ghz-modem, and watch the fixes one gives.
//!     modemscan                       every usb serial port
//!     modemscan /dev/ttyACM0 [secs]   one port, then its NMEA feed
//! A probe opens the port, so anything else reading it has to be closed
//! first.
use gps::{Config, Source, Transport};

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(path) = args.next() else {
        for f in gps::modem::discover() {
            println!("{}  {}", f.transport, f.info.summary());
        }
        return;
    };
    let secs: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(30);
    match gps::modem::probe(&path, gps::modem::BAUD, gps::modem::PROBE) {
        Ok(info) => println!("{path}: {} fw {}", info.summary(), info.fw),
        Err(e) => {
            println!("{path}: {e}");
            return;
        }
    }
    let src = Source::start(Config::new(Transport::Modem { path, baud: gps::modem::BAUD }));
    let until = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    while std::time::Instant::now() < until {
        std::thread::sleep(std::time::Duration::from_millis(500));
        match src.fix() {
            Some(f) => println!("{:.6}, {:.6}  sats {:?}  utc {:?}", f.lat, f.lon, f.sats, f.utc),
            None => println!("no fix (connected {}, {} so far)", src.connected(), src.fixes()),
        }
    }
}
