//! Run the source detector over a .cu8 capture and list what it opens.
//! Usage: detect_file <file.cu8> <rate> [bandwidth]
use common::C32;
use dsp::{SourceConfig, SourceDetector, SourceEvent};

fn main() {
    let path = std::env::args().nth(1).expect("file");
    let rate: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(2_048_000.0);
    let bw: f64 = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let bytes = std::fs::read(&path).unwrap();
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
        .collect();
    let mut det = SourceDetector::new(rate, bw, SourceConfig::default());
    let block = 65536;
    let mut opened = 0usize;
    let mut open_now = 0usize;
    let mut max_open = 0usize;
    let mut hist = std::collections::BTreeMap::<i64, usize>::new();
    let t0 = std::time::Instant::now();
    for chunk in iq.chunks(block) {
        for ev in det.process(chunk) {
            match ev {
                SourceEvent::Opened(s) => {
                    opened += 1;
                    open_now += 1;
                    max_open = max_open.max(open_now);
                    *hist.entry((s.center_hz / 50e3).round() as i64).or_default() += 1;
                    println!(
                        "{:8.3}s open  {:+9.1} kHz  w {:6.1} kHz  snr {:5.1}  id {}",
                        s.start_sample as f64 / rate,
                        s.center_hz / 1e3,
                        s.bandwidth_hz() / 1e3,
                        s.peak_snr_db,
                        s.id.0
                    );
                }
                SourceEvent::Closed(s) | SourceEvent::Superseded(s) => {
                    open_now -= 1;
                    let end = s.end_sample.unwrap_or(s.start_sample);
                    println!(
                        "{:8.3}s close {:+9.1} kHz  w {:6.1} kHz  snr {:5.1}  id {}  dur {:.1} ms  frames {}",
                        end as f64 / rate,
                        s.center_hz / 1e3,
                        s.bandwidth_hz() / 1e3,
                        s.peak_snr_db,
                        s.id.0,
                        (end - s.start_sample) as f64 / rate * 1e3,
                        s.frames
                    );
                }
            }
        }
    }
    let secs = iq.len() as f64 / rate;
    eprintln!(
        "{} opened over {:.1}s, max concurrent {}, detector {:.2}x real time",
        opened,
        secs,
        max_open,
        secs / t0.elapsed().as_secs_f64()
    );
    eprintln!("by 50 kHz cell:");
    for (k, v) in hist {
        eprintln!("  {:+8.0} kHz: {}", k as f64 * 50.0, v);
    }
}
