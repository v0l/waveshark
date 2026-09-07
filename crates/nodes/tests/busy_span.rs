//! A busy ISM band on a wide span: many devices keying on and off at once
//! over 16 MHz, through the auto node, timed.
//!
//! What the receiver has to survive is not one strong signal but thirty
//! weak ones bursting at random, each of which opens a source, is cut out,
//! classified and offered to every front end that fits. This is the case
//! that took a 16 MS/s HackRF under real time on an eight core machine, and
//! the bar here is deliberately low so a slow CI runner passes: it is a
//! guard against a cost that has quietly gone linear in the source count,
//! not a performance target. Real numbers come from running it with
//! `--nocapture` on the machine in question.

use common::{Hz, C32};
use nodes::{build_chain, registry, NodeSpec};
use pipeline::StreamSpec;

/// `count` transmitters keying on and off at random across the span, each a
/// two-tone FSK burst 20 to 200 ms long with gaps of the same order.
fn busy_band(rate: f64, count: usize, secs: f64) -> Vec<C32> {
    let n = (rate * secs) as usize;
    let mut seed = 0x9E37_79B9_7F4A_7C15u64;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64
    };
    struct Tx {
        off: f64,
        dev: f64,
        ph: f64,
        on_until: usize,
        next_on: usize,
        bit: bool,
        bit_len: usize,
        bit_at: usize,
    }
    let mut txs: Vec<Tx> = (0..count)
        .map(|_| Tx {
            off: (rnd() - 0.5) * rate * 0.8,
            dev: 5_000.0 + rnd() * 40_000.0,
            ph: 0.0,
            on_until: 0,
            next_on: (rnd() * 0.3 * rate) as usize,
            bit: false,
            bit_len: (rate / (2_000.0 + rnd() * 30_000.0)) as usize,
            bit_at: 0,
        })
        .collect();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut s = C32::new((rnd() as f32 - 0.5) * 0.02, (rnd() as f32 - 0.5) * 0.02);
        for t in txs.iter_mut() {
            if i >= t.on_until {
                if i >= t.next_on {
                    t.on_until = i + ((0.02 + rnd() * 0.18) * rate) as usize;
                    t.next_on = t.on_until + ((0.05 + rnd() * 0.4) * rate) as usize;
                } else {
                    continue;
                }
            }
            if i >= t.bit_at {
                t.bit = rnd() > 0.5;
                t.bit_at = i + t.bit_len;
            }
            let f = t.off + if t.bit { t.dev } else { -t.dev };
            t.ph += std::f64::consts::TAU * f / rate;
            if t.ph > std::f64::consts::TAU {
                t.ph -= std::f64::consts::TAU;
            }
            s += C32::new(0.03 * t.ph.cos() as f32, 0.03 * t.ph.sin() as f32);
        }
        out.push(s);
    }
    out
}

#[test]
fn thirty_bursting_devices_over_sixteen_megahertz() {
    if cfg!(debug_assertions) {
        eprintln!("skipping: throughput is only measurable in a release build");
        return;
    }
    let rate = 16_000_000.0;
    let iq = busy_band(rate, 30, 2.0);
    let mut g = build_chain(
        StreamSpec::iq(rate, Hz(869_525_000)),
        &[NodeSpec::new("auto")],
        &registry(),
    )
    .unwrap();
    let t0 = std::time::Instant::now();
    let mut packets = 0;
    for b in iq.chunks(262_144) {
        g.feed_iq(b).unwrap();
        if let pipeline::Payload::Packets(p) = g.output() {
            packets += p.len();
        }
    }
    let x = (iq.len() as f64 / rate) / t0.elapsed().as_secs_f64();
    let topo = g.topology();
    for n in &topo.nodes {
        for (name, c) in &n.phases {
            eprintln!(
                "    {:<18} p95 {:>8} us  mean {:>8.0} us",
                name, c.p95_us, c.mean_us
            );
        }
    }
    eprintln!(
        "busy span: {x:.2}x real time on {} threads, {packets} packets",
        rayon::current_num_threads()
    );
    assert!(packets > 0, "nothing came out of a busy band");
    assert!(x > 0.5, "a busy 16 MS/s span ran at only {x:.2}x real time");
}
