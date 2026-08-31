//! Demodulate and decode a 1090 MHz IQ capture, printing what came out.
//!
//! Exists to be pointed at a recording alongside another decoder, which is the
//! only way to know whether the demodulator works: every synthetic test shares
//! its assumptions with the thing it tests.
//!
//! ```text
//! cargo run --release -p decode --example adsb_file -- capture.cu8 [rate]
//! ```

use decode::adsb;
use dsp::{ModeSConfig, ModeSDetector};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("usage: adsb_file <capture.cu8> [rate]");
    let rate: f64 = args.next().map(|s| s.parse().unwrap()).unwrap_or(2.4e6);

    let raw = std::fs::read(&path).expect("read capture");
    let iq: Vec<common::C32> = raw
        .chunks_exact(2)
        .map(|c| {
            common::C32::new(
                (c[0] as f32 - 127.5) / 127.5,
                (c[1] as f32 - 127.5) / 127.5,
            )
        })
        .collect();

    let cfg = ModeSConfig {
        preamble_ratio: std::env::var("MODES_RATIO").ok().and_then(|v| v.parse().ok()).unwrap_or(3.0),
        min_level: std::env::var("MODES_LEVEL").ok().and_then(|v| v.parse().ok()).unwrap_or(0.004),
    };
    let mut d = ModeSDetector::new(rate, cfg);
    let mut frames = Vec::new();
    // The address book both filters and steers: a frame it refuses does not
    // blank the microseconds it sits on, so a real frame underneath is still
    // found. It is behind a cell only because the validator is shared.
    let book = std::cell::RefCell::new(adsb::AddressBook::new());
    // In blocks, the way a radio delivers them, so the buffer boundary is
    // exercised rather than avoided.
    for block in iq.chunks(65_536) {
        d.process_valid(block, &mut frames, &|f: &dsp::ModeSFrame| {
            if std::env::var("MODES_CANDIDATES").is_ok() {
                let hex: String = f.bytes.iter().map(|x| format!("{x:02x}")).collect();
                eprintln!("cand {hex}");
            }
            book.borrow_mut().accept(&f.bytes, f.weak_bits == 0)
        });
    }

    let (mut ok, mut bad) = (0, 0);
    for f in &frames {
        // Error correction belongs here rather than in the demodulator: it is
        // arithmetic on the frame, not signal processing.
        let df = f.bytes[0] >> 3;
        let bytes = match df {
            17 | 18 => adsb::fix_single_bit(&f.bytes).unwrap_or_else(|| f.bytes.clone()),
            _ => f.bytes.clone(),
        };
        let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
        match adsb::parse(&bytes) {
            Ok(p) => {
                ok += 1;
                println!("{hex} df={} {:?}", p.df, p.kind);
            }
            Err(e) => {
                bad += 1;
                println!("{hex} rejected: {e}");
            }
        }
    }
    eprintln!(
        "{} frames demodulated, {ok} parsed, {bad} rejected, {} aircraft, from {:.1} s",
        frames.len(),
        book.borrow().len(),
        iq.len() as f64 / rate
    );
}
