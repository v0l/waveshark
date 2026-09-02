//! Record IQ off the LimeSDR into a file the corpus tools can read back.
//!
//! `cargo run --release -p app --example limerec -- <name> <MHz> <MS/s> <seconds> [antenna] [channel]`
//!
//! The filename carries the centre frequency and sample rate in the form
//! `sources::parse_filename` expects, because a capture whose rate has to be
//! guessed rescales every timing measured from it.
//!
//! Written as float rather than as the radio's native format: this is training
//! and evaluation material, and the eight bits an RTL-SDR would give are not
//! what a LimeSDR heard.

use common::{Device, GainMode, Hz, Sps};
use std::io::Write;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() < 4 {
        eprintln!(
            "usage: limerec <name> <MHz> <MS/s> <seconds> [antenna=LNAL] [channel=RX2] [gain=50]"
        );
        std::process::exit(2);
    }
    let name = a[0].clone();
    let mhz: f64 = a[1].parse().expect("centre frequency in MHz");
    let msps: f64 = a[2].parse().expect("sample rate in MS/s");
    let secs: f64 = a[3].parse().expect("duration in seconds");
    let ant = a.get(4).cloned().unwrap_or_else(|| "LNAL".into());
    let chan = a.get(5).cloned().unwrap_or_else(|| "RX2".into());
    let gain: f32 = a.get(6).map(|v| v.parse().unwrap()).unwrap_or(50.0);

    let mut d = limesdr::LimeSdr::open(0).expect("open LimeSDR");
    d.set_rate(Sps((msps * 1e6) as u64)).expect("set rate");
    d.set_choice("channel", &chan).expect("set channel");
    d.set_choice("antenna", &ant).expect("set antenna");
    d.set_gain("gain", GainMode::Manual(gain)).expect("set gain");
    d.set_center(Hz((mhz * 1e6) as u64)).expect("tune");

    let path = format!("{name}_{mhz}M_{}k.cf32", (msps * 1000.0) as u64);
    let mut out = std::io::BufWriter::with_capacity(
        1 << 22,
        std::fs::File::create(&path).expect("create output"),
    );

    let mut s = d.start_rx().expect("start rx");
    // The first blocks arrive while the front end is still settling, and an
    // AGC-free capture of that is a ramp rather than a signal.
    for _ in 0..20 {
        let _ = s.read();
    }

    let want = (msps * 1e6 * secs) as u64;
    let (mut got, mut peak) = (0u64, 0.0f32);
    while got < want {
        let Ok(b) = s.read() else { break };
        for c in &b.samples {
            peak = peak.max(c.re.abs()).max(c.im.abs());
            out.write_all(&c.re.to_le_bytes()).unwrap();
            out.write_all(&c.im.to_le_bytes()).unwrap();
        }
        got += b.samples.len() as u64;
    }
    drop(s);
    out.flush().unwrap();

    let full_scale_db = 20.0 * peak.max(1e-9).log10();
    println!(
        "{path}: {got} samples, {:.1} s, peak {full_scale_db:.1} dBFS",
        got as f64 / (msps * 1e6)
    );
    // Clipping destroys exactly the amplitude structure a modulation classifier
    // reads, and a peak this close to full scale means the gain was too high
    // for the strongest signal in the span.
    if full_scale_db > -1.0 {
        eprintln!("warning: clipping, lower the gain");
    } else if full_scale_db < -30.0 {
        eprintln!("warning: very quiet, raise the gain");
    }
}
