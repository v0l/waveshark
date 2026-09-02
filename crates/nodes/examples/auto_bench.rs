//! How much span the auto node keeps up with.
//!
//! Noise at a rate, then noise with keyed transmitters in it, fed in
//! radio-sized blocks. Prints seconds of signal processed per second of
//! wall clock: above 1.0 keeps up with a live radio.
//!
//!     cargo run --release -p nodes --example auto_bench -- 4000000 8

use common::{Hz, C32};
use nodes::{build_chain, registry, NodeSpec};
use pipeline::StreamSpec;

fn noise(n: usize, amp: f32, seed: &mut u64) -> Vec<C32> {
    (0..n)
        .map(|_| {
            *seed ^= *seed << 13;
            *seed ^= *seed >> 7;
            *seed ^= *seed << 17;
            let u1 = ((*seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-12);
            *seed ^= *seed << 13;
            *seed ^= *seed >> 7;
            *seed ^= *seed << 17;
            let u2 = (*seed >> 11) as f64 / (1u64 << 53) as f64;
            let r = (-2.0 * u1.ln()).sqrt();
            let th = std::f64::consts::TAU * u2;
            C32::new((r * th.cos()) as f32 * amp, (r * th.sin()) as f32 * amp)
        })
        .collect()
}

fn main() {
    let rate: f64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(4_000_000.0);
    let transmitters: usize = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0);
    let seconds = 4.0;
    let mut seed = 0x1234_5678_9abc_def1u64;
    let mut iq = noise((rate * seconds) as usize, 0.02, &mut seed);
    // Each transmitter: 2 kbit/s OOK, 40 ms bursts every 250 ms, spread
    // across the span, so a few are open at any moment.
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
    let mut g = build_chain(StreamSpec::iq(rate, Hz::mhz(868)), &[NodeSpec::new("auto")], &registry()).unwrap();
    let block = 16_384;
    // Warm up on the first second, then time the rest.
    let warm = (rate as usize).min(iq.len());
    for b in iq[..warm].chunks(block) {
        g.feed_iq(b).unwrap();
    }
    let t0 = std::time::Instant::now();
    let mut packets = 0usize;
    for b in iq[warm..].chunks(block) {
        g.feed_iq(b).unwrap();
        packets += g.output().len();
    }
    let wall = t0.elapsed().as_secs_f64();
    let audio = (iq.len() - warm) as f64 / rate;
    println!(
        "{:.1} MS/s, {transmitters} transmitters: {:.2}x real time ({packets} packets)",
        rate / 1e6,
        audio / wall
    );
}
