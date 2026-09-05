//! Watch one frequency on the LimeSDR while something transmits on it.
//!
//! Usage: `txcheck [freq_hz] [offset_hz] [seconds]`
//!
//! Prints the bin at `freq + offset` against the median bin of the span, once
//! every 100 ms. Comparing against the median rather than an absolute level
//! makes the reading a signal-to-noise figure, which does not depend on the
//! receiving gain being calibrated.

use common::{Device, GainMode, Hz, Sps};
use dsp::spectrum::Spectrum;

fn main() {
    let mut args = std::env::args().skip(1);
    let freq: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(868_500_000);
    let offset: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(100_000.0);
    let secs: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(10.0);

    let rate = Sps(4_000_000);
    let mut d = limesdr::LimeSdr::open(0).expect("open LimeSDR");
    d.set_rate(rate).unwrap();
    d.set_center(Hz(freq)).unwrap();
    d.set_gain("gain", GainMode::Manual(40.0)).unwrap();
    let actual = d.actual_rate().as_f64();

    const N: usize = 4096;
    let mut spec = Spectrum::new(N);
    spec.smoothing = 1.0;
    // Bins run lowest frequency first with DC in the middle.
    let bin = (N as i64 / 2 + (offset / actual * N as f64).round() as i64)
        .clamp(0, N as i64 - 1) as usize;

    let mut s = d.start_rx().unwrap();
    let t = std::time::Instant::now();
    let mut next = std::time::Duration::ZERO;
    println!(
        "watching {:.4} MHz at {:.3} MS/s, bin {bin} of {N}",
        (freq as f64 + offset) / 1e6,
        actual / 1e6
    );
    println!("{:>6}  {:>9}  {:>9}  {:>8}", "t", "tone dB", "floor dB", "snr dB");

    while t.elapsed().as_secs_f64() < secs {
        let Ok(b) = s.read() else { break };
        if !spec.process(&b.samples) || t.elapsed() < next {
            continue;
        }
        next = t.elapsed() + std::time::Duration::from_millis(100);

        let db = spec.power_db();
        let tone = db[bin];
        let mut sorted = db.to_vec();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted[N / 2];
        println!(
            "{:6.1}  {tone:9.1}  {floor:9.1}  {:8.1}",
            t.elapsed().as_secs_f64(),
            tone - floor
        );
    }
    s.stop();
}
