//! What each span-wide front end costs on a band with nothing in it.
//!
//! The narrowband decoders only run on a source the detector opened, so what
//! they cost when nothing is transmitting is nothing. A span-wide front end
//! is handed every block for as long as the receiver runs, so this is the
//! floor under a receiver parked on its band.
//!
//! Through the auto node, so what each one is handed is what the receiver
//! hands it.
//!
//!     cargo run --release -p nodes --example front_cost -- 20000000
use common::{Hz, C32};

fn noise(n: usize, amp: f32, seed: &mut u64) -> Vec<C32> {
    (0..n)
        .map(|_| {
            let mut next = || {
                *seed ^= *seed << 13;
                *seed ^= *seed >> 7;
                *seed ^= *seed << 17;
                (*seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5
            };
            let (a, b) = (next(), next());
            C32::new(a * amp, b * amp)
        })
        .collect()
}

fn main() {
    let rate: f64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(20e6);
    let secs: f64 = 2.0;
    let mut seed = 0x1234_5678_9abc_def1u64;
    let iq = noise((rate * secs) as usize, 0.05, &mut seed);
    for p in nodes::protocol::all() {
        let shape = p.shape();
        if !shape.span_wide || rate < shape.min_rate_hz {
            continue;
        }
        let center = Hz(p.default_hz() as u64);
        let (factor, fed) = match p.narrow_span() {
            true => nodes::protocol::span_feed(rate, 0.0, &shape),
            false => (1, rate),
        };
        let mut chain = match factor {
            1 => Vec::new(),
            f => vec![nodes::NodeSpec::new("decimate")
                .i("factor", f as i64)
                .f("passband", 0.8)
                .f("atten_db", 60.0)],
        };
        let at = nodes::Placed {
            center_hz: p.default_hz(),
            width_hz: shape.widths[0],
            rate: fed,
            snr_db: f32::NAN,
        };
        chain.extend(p.chain(at));
        let mut g = match nodes::build_chain(
            pipeline::StreamSpec::iq(rate, center),
            &chain,
            &nodes::registry(),
        ) {
            Ok(g) => g,
            Err(e) => {
                println!("{:<8} refused {rate} S/s: {e}", p.id());
                continue;
            }
        };
        let t = std::time::Instant::now();
        for b in iq.chunks(131_072) {
            g.feed_iq(b).unwrap();
        }
        let el = t.elapsed().as_secs_f64();
        println!(
            "{:<8} fed {:>8.3} MS/s of {:.0}: {:.1}s of quiet band in {el:.3}s = {:>6.1}% of one core, watch {:?}",
            p.id(),
            fed / 1e6,
            rate / 1e6,
            secs,
            100.0 * el / secs,
            p.watch()
        );
    }
}
