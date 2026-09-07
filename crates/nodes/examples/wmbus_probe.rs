//! Run a capture through the wM-Bus demodulator on its own, with the channel
//! cut out by hand instead of by the auto node:
//!     wmbus_probe <file.cu8> [offset_hz] [cutoff_hz]
use common::C32;
use dsp::wmbus::Demod;
use sources::FileSource;

fn main() {
    let path = std::env::args().nth(1).expect("file");
    let shift: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let cutoff: f64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(125_000.0);
    let buf = FileSource::open(std::path::Path::new(&path)).unwrap().read_all().unwrap();
    let rate = buf.rate.as_f64();
    let mut samples = buf.samples.clone();
    let mut ph = 0.0f64;
    for x in samples.iter_mut() {
        ph += std::f64::consts::TAU * -shift / rate;
        *x *= C32::new(ph.cos() as f32, ph.sin() as f32);
    }
    let taps = lowpass(cutoff / rate, 127);
    let mut filtered = Vec::with_capacity(samples.len());
    for i in 0..samples.len() {
        let mut acc = C32::new(0.0, 0.0);
        for (k, t) in taps.iter().enumerate() {
            if i >= k {
                acc += samples[i - k] * *t;
            }
        }
        filtered.push(acc);
    }
    let mut d = Demod::new(rate);
    let mut n = 0;
    for block in filtered.chunks(16_384) {
        for f in d.process(block) {
            n += 1;
            println!(
                "{:?} at {:.4}s {} bytes: {}",
                f.mode,
                f.at as f64 / rate,
                f.bytes.len(),
                f.bytes.iter().map(|b| format!("{b:02x}")).collect::<String>()
            );
        }
    }
    println!("{n} frame(s) at {rate} S/s, shifted {shift} Hz, cutoff {cutoff} Hz");
}

fn lowpass(fc: f64, n: usize) -> Vec<f32> {
    let mid = (n / 2) as f64;
    let mut t: Vec<f32> = (0..n)
        .map(|i| {
            let x = i as f64 - mid;
            let sinc = if x == 0.0 { 2.0 * fc } else { (std::f64::consts::TAU * fc * x).sin() / (std::f64::consts::PI * x) };
            let w = 0.54 - 0.46 * (std::f64::consts::TAU * i as f64 / (n - 1) as f64).cos();
            (sinc * w) as f32
        })
        .collect();
    let sum: f32 = t.iter().sum();
    for x in t.iter_mut() {
        *x /= sum;
    }
    t
}
