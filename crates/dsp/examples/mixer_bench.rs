//! The cost of shifting a 40 MS/s span, as the chain view showed it at 23%.
//!
//! `cargo run --release -p dsp --example mixer_bench`

use common::C32;
use dsp::mixer::Mixer;
use std::time::Instant;

fn main() {
    let rate = 40_000_000.0;
    let block = 400_000;
    let blocks = 100;
    let sig: Vec<C32> = (0..block)
        .map(|i| {
            let p = std::f32::consts::TAU * 0.0137 * i as f32;
            C32::new(p.cos() * 0.5, p.sin() * 0.5)
        })
        .collect();
    let mut m = Mixer::new(-1_234_567.0, rate);
    let mut out = Vec::with_capacity(block);
    m.process(&sig, &mut out);
    let t = Instant::now();
    let mut sink = 0.0f32;
    for _ in 0..blocks {
        out.clear();
        m.process(&sig, &mut out);
        sink += out[out.len() / 2].re;
    }
    let per_block = t.elapsed().as_secs_f64() / blocks as f64;
    let real = block as f64 / rate;
    println!(
        "{:.2} ms per {:.0} ms block, {:.0}% of real time ({sink:e})",
        per_block * 1e3,
        real * 1e3,
        100.0 * per_block / real
    );
}
