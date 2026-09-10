//! Read BLE advertising out of a capture and list what advertised.
//!     ble_file <file.cs8|file.cu8> <rate> <centre_hz>
use common::C32;
use dsp::{BleConfig, BleDetector};
use std::collections::BTreeMap;

fn mean_offset(frames: &[dsp::BleFrame]) -> f32 {
    if frames.is_empty() {
        return 0.0;
    }
    frames.iter().map(|f| f.freq_off_hz).sum::<f32>() / frames.len() as f32
}

fn main() {
    let path = std::env::args().nth(1).expect("file");
    let rate: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(16e6);
    let centre: f64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(2.43e9);
    let bytes = std::fs::read(&path).unwrap();
    let signed = path.ends_with(".cs8");
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| {
            if signed {
                C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0)
            } else {
                C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5)
            }
        })
        .collect();

    let mut det = BleDetector::new(rate, centre, BleConfig::default());
    println!("channels: {:?}", det.channels());
    let mut frames = Vec::new();
    let t0 = std::time::Instant::now();
    for c in iq.chunks(65_536) {
        det.process(c, &mut frames);
    }
    let secs = iq.len() as f64 / rate;
    println!(
        "{} frames from {secs:.1}s in {:.1}s ({:.1}x real time)",
        frames.len(),
        t0.elapsed().as_secs_f64(),
        secs / t0.elapsed().as_secs_f64()
    );
    let mut by: BTreeMap<String, usize> = BTreeMap::new();
    for f in &frames {
        let mac: Vec<String> = f.pdu[2..8].iter().rev().map(|b| format!("{b:02X}")).collect();
        *by.entry(mac.join(":")).or_default() += 1;
    }
    for (mac, n) in &by {
        println!("  {mac} {n}");
    }
    println!("mean frequency error {:.0} Hz", mean_offset(&frames));
}
