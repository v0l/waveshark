//! Open the HackRF directly and check it streams.
use common::{Device, GainMode, Hz, Sps};
fn main() {
    let serials = hackrf::enumerate();
    println!("hackrf devices: {serials:?}");
    if serials.is_empty() {
        return;
    }
    let mut d = match hackrf::HackRfDevice::open(0) {
        Ok(d) => d,
        Err(e) => {
            println!("open failed: {e}");
            return;
        }
    };
    println!("label   : {}", d.info().label);
    println!("tuner   : {}", d.info().tuner);
    println!("range   : {:?}", d.info().ranges.iter().map(|r| r.label).collect::<Vec<_>>());
    println!("rates   : {:?}..={:?}", d.info().rate_range.start().0, d.info().rate_range.end().0);
    d.set_rate(Sps(8_000_000)).unwrap();
    d.set_center(Hz(95_800_000)).unwrap();
    let mut s = d.start_rx().unwrap();
    // Warm up, then sweep gain and check the level actually follows.
    let t = std::time::Instant::now();
    let (mut n, mut blocks) = (0u64, 0u64);
    while t.elapsed().as_secs_f64() < 3.0 {
        if let Ok(b) = s.read() {
            n += b.samples.len() as u64;
            blocks += 1;
        } else {
            break;
        }
    }
    let el = t.elapsed().as_secs_f64();
    println!(
        "\n{blocks} blocks, {n} samples in {el:.1}s = {:.3} MS/s (asked 8.000)",
        n as f64 / el / 1e6
    );
    println!("dropped {}\n", s.dropped());

    println!(
        "{:>6} {:>5} {:>5} {:>4}  {:>9} {:>8}",
        "ask dB", "amp", "lna", "vga", "rms dBFS", "peak"
    );
    for ask in [0.0f32, 16.0, 32.0, 48.0, 64.0, 80.0] {
        let (amp, lna, vga) = hackrf::gain::distribute(ask);
        d.set_gain("tuner", GainMode::Manual(ask)).unwrap();
        // Let the change settle through the buffers already in flight.
        for _ in 0..12 {
            let _ = s.read();
        }
        let (mut acc, mut cnt, mut peak) = (0.0f64, 0u64, 0.0f32);
        for _ in 0..6 {
            if let Ok(b) = s.read() {
                for c in b.samples.iter().step_by(16) {
                    acc += c.norm_sqr() as f64;
                    cnt += 1;
                    peak = peak.max(c.norm());
                }
            }
        }
        let rms = (acc / cnt.max(1) as f64).sqrt();
        println!(
            "{ask:>6.0} {:>5} {lna:>5} {vga:>4}  {:>9.1} {peak:>8.3}",
            if amp { "on" } else { "off" },
            20.0 * (rms + 1e-12).log10()
        );
    }
    s.stop();
}
