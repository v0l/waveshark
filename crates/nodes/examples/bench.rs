//! Measure wideband throughput: how many channels can be decoded in real time.
//!
//! Reports throughput as a multiple of real time, which is the number that
//! matters: anything above 1.0x can keep up with a live radio at that rate.

use common::{Hz, C32};
use nodes::{registry, ChannelBank, Gating, NodeSpec};
use std::time::Instant;

fn chain() -> Vec<NodeSpec> {
    vec![
        NodeSpec::new("envelope"),
        NodeSpec::new("pulse_detect")
            .f("reset_us", 10_000.0)
            .i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ]
}

/// Noise plus a few OOK-ish bursts, so the detector has real work to do rather
/// than trivially rejecting silence.
fn signal(n: usize, rate: f64) -> Vec<C32> {
    let mut seed = 12345u64;
    let mut rng = move || {
        seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((seed >> 33) as f32 / (1u64 << 31) as f32) - 0.5
    };
    (0..n)
        .map(|i| {
            let t = i as f64 / rate;
            // A 1 kHz OOK burst pattern across part of the span.
            let on = ((t * 1000.0) as u64).is_multiple_of(3);
            let a = if on { 0.5 } else { 0.0 };
            let ph = (t * 40_000.0 * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
            C32::new(
                a * ph.cos() as f32 + rng() * 0.05,
                a * ph.sin() as f32 + rng() * 0.05,
            )
        })
        .collect()
}

fn main() {
    let rate: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(2_400_000.0);
    let secs = 2.0;
    let n = (rate * secs) as usize;
    println!("input: {:.1} MS/s, {secs}s, {n} samples", rate / 1e6);
    println!("threads: {}\n", rayon::current_num_threads());
    println!(
        "{:>6}  {:>10}  {:>9}  {:>10}  {:>8}",
        "chans", "ch rate", "ch BW", "wall", "x real"
    );

    let sig = signal(n, rate);
    for &chans in &[8usize, 16, 32, 64, 128, 256, 512] {
        let mut bank = ChannelBank::new(chans, 12, rate, Hz::mhz(433));
        bank.set_gating(if std::env::args().nth(2).is_some() {
            Gating::OnDetection
        } else {
            Gating::Always
        });
        if bank.set_all_chains(&chain(), &registry()).is_err() {
            continue;
        }
        // Warm up so allocation and first-touch page faults are not measured.
        let _ = bank.process(&sig[..n / 20]);
        bank.reset();

        let t = Instant::now();
        let _ = bank.process(&sig).expect("bank");
        let wall = t.elapsed().as_secs_f64();
        println!(
            "{chans:>6}  {:>8.1} kS/s  {:>6.1} kHz  {:>8.3} s  {:>7.2}x",
            bank.channel_rate() / 1e3,
            bank.channel_bandwidth() / 1e3,
            wall,
            secs / wall
        );
    }
}
