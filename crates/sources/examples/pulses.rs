//! Dump the pulse train from a capture file.
//! Usage: cargo run --release -p sources --example pulses -- <file.cu8>

use dsp::{OokDetector, PulseConfig};
use sources::FileSource;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).ok_or("usage: pulses <file>")?;
    let src = FileSource::open(&path)?;
    let buf = src.read_all()?;
    println!("{path}: {} samples @ {} ({:.3}s)", buf.len(), buf.rate, buf.duration().as_secs_f64());

    let env: Vec<f32> = buf.samples.iter().map(|c| c.norm()).collect();

    // Fine Offset uses long inter-symbol gaps, so the reset must be generous
    // or a single packet is reported as many.
    let cfg = PulseConfig { reset_us: 10_000, min_pulses: 20, ..Default::default() };
    let mut d = OokDetector::new(buf.rate.as_f64(), cfg);
    let mut pkgs = Vec::new();
    d.process(&env, &mut pkgs);

    println!("\n{} package(s)", pkgs.len());
    for (i, p) in pkgs.iter().enumerate() {
        println!(
            "\n--- package {i}: {} pulses, {:.1} ms, SNR {:.1} dB, at sample {}",
            p.pulses.len(),
            p.duration_us() as f64 / 1000.0,
            p.snr_db,
            p.start_sample
        );
        println!("  mark clusters: {:?}", p.mark_histogram(150));
        println!("  gap  clusters: {:?}", p.gap_histogram(150));
        let s: Vec<String> =
            p.pulses.iter().take(24).map(|x| format!("{}/{}", x.mark, x.gap)).collect();
        println!("  pulses: {}", s.join(" "));
    }
    Ok(())
}
