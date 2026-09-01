//! Open the LimeSDR directly and check it streams.
use common::{Device, GainMode, Hz, Sps};

fn main() {
    let found = limesdr::enumerate();
    println!("limesdr devices:");
    for e in &found {
        println!("  [{}] {} usb2={} max={} S/s", e.index, e.info, e.usb2, e.rate_max().0);
    }
    if found.is_empty() {
        return;
    }
    let mut d = match limesdr::LimeSdr::open(0) {
        Ok(d) => d,
        Err(e) => {
            println!("open failed: {e}");
            return;
        }
    };
    println!("label   : {}", d.info().label);
    println!("tuner   : {}", d.info().tuner);
    println!("range   : {:?}", d.info().ranges.iter().map(|r| r.label).collect::<Vec<_>>());
    println!("rates   : {}..={}", d.info().rate_range.start().0, d.info().rate_range.end().0);
    println!("toggles : {:?}", d.toggles().iter().map(|t| t.name.clone()).collect::<Vec<_>>());
    if let Some(t) = d.chip_temperature() {
        println!("temp    : {t:.1} C");
    }

    let rate = Sps(4_000_000).min(*d.info().rate_range.end());
    d.set_rate(rate).unwrap();
    d.set_center(Hz(95_800_000)).unwrap();
    println!("actual  : {} S/s at {} Hz", d.actual_rate().0, d.actual_center().0);

    let mut s = d.start_rx().unwrap();
    let t = std::time::Instant::now();
    let (mut n, mut blocks) = (0u64, 0u64);
    while t.elapsed().as_secs_f64() < 3.0 {
        match s.read() {
            Ok(b) => {
                n += b.samples.len() as u64;
                blocks += 1;
            }
            Err(e) => {
                println!("read failed: {e}");
                break;
            }
        }
    }
    let el = t.elapsed().as_secs_f64();
    println!(
        "\n{blocks} blocks, {n} samples in {el:.1}s = {:.3} MS/s (asked {:.3})",
        n as f64 / el / 1e6,
        rate.0 as f64 / 1e6
    );
    println!("dropped {}\n", s.dropped());

    println!("{:>6}  {:>9} {:>8}", "ask dB", "rms dBFS", "peak");
    for ask in [0.0f32, 20.0, 40.0, 60.0, 73.0] {
        d.set_gain("gain", GainMode::Manual(ask)).unwrap();
        for _ in 0..8 {
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
        println!("{ask:>6.0}  {:>9.1} {peak:>8.3}", 20.0 * rms.max(1e-9).log10());
    }
    println!("read back: {:?}", d.gains());
    println!("dropped {}", s.dropped());
}
