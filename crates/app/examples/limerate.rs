//! What sample rate this link actually sustains.
use common::{Device, Hz, Sps};
fn main() {
    let mut d = limesdr::LimeSdr::open(0).unwrap();
    d.set_center(Hz(101_600_000)).unwrap();
    for mhz in [4.0f64, 10.0, 20.0, 30.72, 40.0, 50.0, 61.44] {
        let r = Sps((mhz * 1e6) as u64);
        // Bypass the driver's own ceiling: the point is to find where it is.
        if let Err(e) = d.set_rate(r) { println!("{mhz:>5.1} MS/s  refused: {e}"); continue; }
        let mut s = d.start_rx().unwrap();
        for _ in 0..10 { let _ = s.read(); }
        let t = std::time::Instant::now();
        let mut n = 0u64;
        while t.elapsed().as_secs_f64() < 3.0 { match s.read() { Ok(b) => n += b.samples.len() as u64, Err(_) => break } }
        let el = t.elapsed().as_secs_f64();
        println!("{mhz:>5.1} MS/s  got {:>6.3} MS/s  dropped {:>10}", n as f64 / el / 1e6, s.dropped());
    }
}
