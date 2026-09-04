//! Replay a .cu8 through the auto node and report throughput.
//!     auto_file <file.cu8> <rate> <centre_hz>
use common::{Hz, C32};
use nodes::{build_chain, registry, NodeSpec};
use pipeline::StreamSpec;

fn main() {
    let path = std::env::args().nth(1).expect("file");
    let rate: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(2_048_000.0);
    let centre: f64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(869_200_000.0);
    let bytes = std::fs::read(&path).unwrap();
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
        .collect();
    let mut g = build_chain(StreamSpec::iq(rate, Hz(centre as i64 as u64)), &[NodeSpec::new("auto")], &registry()).unwrap();
    let block = 16_384;
    let t0 = std::time::Instant::now();
    let mut packets = 0usize;
    let mut worst = 0.0f64;
    let mut slow_blocks = 0usize;
    for (i, b) in iq.chunks(block).enumerate() {
        let t = std::time::Instant::now();
        let evs = g.feed_iq(b).unwrap();
        if std::env::var_os("EVENTS").is_some() {
            for e in evs {
                eprintln!("EV at {:.3}s: {e:?}", i as f64 * block as f64 / rate);
            }
        }
        for p in g.output().as_packets().unwrap_or(&[]) {
            packets += 1;
            if let common::PacketBody::Frame(b) = &p.body {
                if let Some(d) = nodes::lora_nodes::lora_decoded(&b[..], common::Hz(p.center_hz)) {
                    eprintln!("LORA at {:.2}s: {:?}", i as f64 * block as f64 / rate, d);
                }
            }
        }
        let dt = t.elapsed().as_secs_f64();
        let real = block as f64 / rate;
        if dt > real { slow_blocks += 1; }
        if dt > worst { worst = dt; eprintln!("block {i} at {:.2}s took {:.1} ms ({:.1}x block)", i as f64 * real, dt * 1e3, dt / real); }
    }
    let wall = t0.elapsed().as_secs_f64();
    let secs = iq.len() as f64 / rate;
    println!("{:.2}x real time, {packets} packets, {slow_blocks} of {} blocks slower than real time", secs / wall, iq.len() / block);
}
