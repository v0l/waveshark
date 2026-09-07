//! Watch a GPS and print what it says.
//!     gpswatch [source] [seconds]
//! Source is a serial port (/dev/ttyACM0, /dev/ttyUSB0@4800) or a gpsd
//! address (gpsd:localhost); the default is gpsd on this machine.
use gps::{Config, Source, Transport};

fn main() {
    let arg = std::env::args().nth(1).unwrap_or_else(|| "gpsd:127.0.0.1:2947".into());
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(30);
    let Some(t) = Transport::parse(&arg) else {
        eprintln!("{arg:?} is not a serial port or a gpsd address");
        return;
    };
    println!("watching {t} for {secs}s");
    let src = Source::start(Config::new(t));
    let until = std::time::Instant::now() + std::time::Duration::from_secs(secs);
    let mut last = None;
    while std::time::Instant::now() < until {
        std::thread::sleep(std::time::Duration::from_millis(500));
        let fix = src.fix();
        if fix != last {
            match fix {
                Some(f) => println!(
                    "{:.6}, {:.6}  alt {:?}  speed {:?}  track {:?}  sats {:?}  hdop {:?}  utc {:?}",
                    f.lat, f.lon, f.alt_m, f.speed_ms, f.track_deg, f.sats, f.hdop, f.utc
                ),
                None => println!("no fix (connected {}, {} so far)", src.connected(), src.fixes()),
            }
            last = fix;
        }
    }
    println!("connected {}, {} fixes", src.connected(), src.fixes());
}
