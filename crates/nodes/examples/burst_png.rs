//! Draw the spectrogram of the first decoded LoRa packet in a capture, the
//! way the inspector does, to a PPM for looking at.
//!     burst_png <file.cu8> <rate> <centre_hz> <out.ppm>
use common::C32;
use nodes::{build_chain, registry, NodeSpec};
use pipeline::StreamSpec;

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let rate: f64 = a[2].parse().unwrap();
    let centre: f64 = a[3].parse().unwrap();
    let bytes = std::fs::read(&a[1]).unwrap();
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
        .collect();
    let mut g = build_chain(StreamSpec::iq(rate, common::Hz(centre as u64)), &[NodeSpec::new("auto")], &registry()).unwrap();
    let mut burst = None;
    for b in iq.chunks(16_384) {
        g.feed_iq(b).unwrap();
        for p in g.output().as_packets().unwrap_or(&[]) {
            if let (common::PacketBody::Frame(f), Some(q)) = (&p.body, &p.iq) {
                if nodes::lora_nodes::lora_decoded(&f[..], common::Hz(p.center_hz)).is_some() && burst.is_none() {
                    burst = Some(q.clone());
                }
            }
        }
    }
    let q = burst.expect("a LoRa packet");
    let (cols, rows) = (1300usize, 256usize);
    let win = (q.samples.len() / 300).clamp(8, rows / 8);
    let img = dsp::spectrum::spectrogram(&q.samples, cols, rows, win);
    let n = img.len() / cols;
    let mut sorted: Vec<f32> = img.iter().copied().filter(|v| v.is_finite()).collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let floor = sorted[sorted.len() / 2];
    let span = (0.0 - floor).max(6.0);
    let mut out = format!("P6 {cols} {n} 255\n").into_bytes();
    for r in 0..n {
        for c in 0..cols {
            let v = ((img[(n - 1 - r) * cols + c] - floor) / span).clamp(0.0, 1.0);
            let g = (v * 255.0) as u8;
            out.extend_from_slice(&[(g as f32 * 0.9) as u8, (g as f32 * 0.8) as u8, (60.0 + (1.0 - v) * 40.0) as u8]);
        }
    }
    std::fs::write(&a[4], out).unwrap();
    eprintln!("{} samples, window {win}", q.samples.len());
}
