//! The bare detector on noise: how much of the auto node's cost is the
//! detector itself.
use common::C32;
use dsp::{SourceConfig, SourceDetector, SourceExtractor};
fn main() {
    let rate: f64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(20_000_000.0);
    let mut seed = 0x1234_5678_9abc_def1u64;
    let n = (rate * 3.0) as usize;
    let iq: Vec<C32> = (0..n).map(|_| {
        seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17;
        let a = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
        seed ^= seed << 13; seed ^= seed >> 7; seed ^= seed << 17;
        let b = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
        C32::new(a * 0.05, b * 0.05)
    }).collect();
    let transmitters: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let mut iq = iq;
    for k in 0..transmitters {
        let off = -rate * 0.4 + rate * 0.8 * (k as f64 + 0.5) / transmitters as f64;
        let period = (rate * 0.25) as usize;
        let burst = (rate * 0.04) as usize;
        let start = (period as f64 * k as f64 / transmitters as f64) as usize;
        let mut ph = 0.0f64;
        let mut t = start;
        while t + burst < iq.len() {
            for i in 0..burst {
                let on = (i / (rate as usize / 2000)) % 2 == 0;
                ph += std::f64::consts::TAU * off / rate;
                if on {
                    iq[t + i] += C32::new(0.2 * ph.cos() as f32, 0.2 * ph.sin() as f32);
                }
            }
            t += period;
        }
    }
    let cfg = SourceConfig::default();
    let mut d = SourceDetector::new(rate, rate, cfg);
    let mut e = SourceExtractor::new(rate, 868e6, d.latency_samples(), cfg);
    let block = 16_384;
    let t0 = std::time::Instant::now();
    for b in iq.chunks(block) { d.process(b); }
    let det = t0.elapsed().as_secs_f64();
    let mut d2 = SourceDetector::new(rate, rate, cfg);
    let mut out = Vec::new();
    let t1 = std::time::Instant::now();
    for b in iq.chunks(block) { let ev = d2.process(b).to_vec(); e.process(b, &ev, &mut out); }
    let both = t1.elapsed().as_secs_f64();
    let secs = n as f64 / rate;
    let opened = out.iter().filter(|b| b.state == common::SourceState::Opened).count();
    let widths: Vec<String> = out.iter().filter(|b| b.state == common::SourceState::Opened).take(6).map(|b| format!("{:.0}k@{:.0}k", b.bandwidth_hz / 1e3, b.rate / 1e3)).collect();
    println!("{:.1} MS/s, {transmitters} tx, fft {} bins, {} frames/s: detector {:.2}x real time, detector+extractor {:.2}x; {opened} sources opened, e.g. {}", rate / 1e6, d.fft_size(), d.frame_rate() as u64, secs / det, secs / both, widths.join(" "));
}
