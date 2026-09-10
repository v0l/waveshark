//! Decode RDS from a recorded capture and report how well it went.
//!
//! Hardware runs take the best part of a minute and are not repeatable. This
//! reads a `cu8` file so a change can be measured in seconds against exactly
//! the same signal.
//!
//! Usage: `cargo run -p dsp --release --example rds_offline -- <file.cu8> [rate]`

use common::C32;
use dsp::rds::{BlockSync, GroupDecoder, RdsDemod};
use dsp::{FirDecim, FmDemod, StereoDecoder};
use std::f64::consts::TAU;

fn goertzel(x: &[f32], f: f64, rate: f64) -> f64 {
    let k = TAU * f / rate;
    let c = 2.0 * k.cos();
    let (mut a, mut b) = (0.0f64, 0.0f64);
    for &v in x {
        let t = v as f64 + c * a - b;
        b = a;
        a = t;
    }
    (a * a + b * b - c * a * b).sqrt() / x.len() as f64
}

/// Mean power across a band, sampled at several bins.
///
/// A single bin at 57 kHz measures nothing useful: RDS is suppressed-carrier
/// double sideband, so the carrier frequency itself is a null. The question is
/// whether there is energy in the sidebands either side of it.
fn band(x: &[f32], lo: f64, hi: f64, rate: f64) -> f64 {
    let n = 12;
    let mut acc = 0.0;
    for k in 0..n {
        let f = lo + (hi - lo) * k as f64 / (n - 1) as f64;
        let v = goertzel(x, f, rate);
        acc += v * v;
    }
    (acc / n as f64).sqrt()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let path = args.get(1).cloned().unwrap_or_else(|| {
        eprintln!("usage: rds_offline <file.cu8> [rate]");
        std::process::exit(2);
    });
    let rate: f64 = args.get(2).and_then(|v| v.parse().ok()).unwrap_or(1_024_000.0);

    let raw = std::fs::read(&path).expect("read capture");
    let samples: Vec<C32> = raw
        .chunks_exact(2)
        .map(|p| C32::new((p[0] as f32 - 127.5) / 127.5, (p[1] as f32 - 127.5) / 127.5))
        .collect();
    println!("{path}: {:.1}s at {:.0} kS/s", samples.len() as f64 / rate, rate / 1000.0);

    // The occupied bandwidth is 264 kHz: Carson with RDS at 57 kHz as the
    // highest modulating frequency, not audio at 15 kHz.
    let dec = ((rate / 330_000.0).round() as usize).max(1);
    let if_rate = rate / dec as f64;
    let mut iff = FirDecim::design_hz(rate, dec, 132_000.0, 70.0);
    let mut fm = FmDemod::new(if_rate, 75_000.0);
    let mut st = StereoDecoder::new(if_rate);
    let mut rds = RdsDemod::new(if_rate);
    let mut sync = BlockSync::new();
    let mut groups = GroupDecoder::new();

    let (mut iq, mut disc) = (Vec::new(), Vec::new());
    let (mut l, mut r, mut bits) = (Vec::new(), Vec::new(), Vec::new());
    let mut total_bits = 0u64;

    for (n, chunk) in samples.chunks(262_144).enumerate() {
        iq.clear();
        iff.process(chunk, &mut iq);
        disc.clear();
        fm.process(&iq, &mut disc);
        st.process(&disc, &mut l, &mut r);
        bits.clear();
        rds.process(&disc, st.phases(), &mut bits);
        total_bits += bits.len() as u64;
        for b in &bits {
            if let Some(g) = sync.push(*b) {
                groups.push(&g);
            }
        }
        // An empty stretch above RDS and below the IF edge, as a noise floor.
        let noise = band(&disc, 64_000.0, 72_000.0, if_rate);
        let db = |v: f64| 20.0 * (v / noise.max(1e-15)).log10();
        println!(
            "{n:3} vs noise: pilot {:5.1} diff-band {:5.1} rds-band {:5.1} dB | lock {:.2} | \
             arm {} margin {:.2} lvl {:.4} | bits {total_bits} groups {} err {} sync {}",
            db(goertzel(&disc, 19_000.0, if_rate)),
            db(band(&disc, 24_000.0, 52_000.0, if_rate)),
            db(band(&disc, 54_800.0, 59_200.0, if_rate)),
            st.lock(),
            rds.timing().0,
            rds.timing().1,
            rds.level(),
            sync.groups,
            sync.errors,
            sync.is_synced(),
        );
    }

    let s = groups.station();
    let expected = total_bits / 104;
    println!(
        "\ngroups {} of about {expected} possible  errors {}  rejected {}  yield {:.1}%",
        sync.groups,
        sync.errors,
        sync.rejected,
        100.0 * sync.groups as f64 / expected.max(1) as f64
    );
    println!(
        "PI {}  name {:?}  pty {:?}",
        s.pi.map(|p| format!("{p:04X}")).unwrap_or_else(|| "-".into()),
        s.name,
        s.pty_name()
    );
    if let Some(rt) = &s.radiotext {
        println!("radiotext: {rt}");
    }
}
