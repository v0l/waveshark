//! Record IQ off the HackRF into a file the corpus tools can read back.
//!
//! `cargo run --release -p app --example hrfrec -- <name> <MHz> <MS/s> <seconds> [gain=40]`
//!
//! Same output convention as `limerec`, and the same reason for it: the
//! filename carries the centre frequency and sample rate because a capture
//! whose rate has to be guessed rescales every timing measured from it.
//!
//! The HackRF exists here for the bands the LimeSDR on its low-band port
//! cannot reach. Everything above 1.5 GHz, which is where Wi-Fi, Bluetooth and
//! the mobile downlinks live, is HackRF territory.

use common::{Device, GainMode, Hz, Sps};
use std::io::Write;

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() < 4 {
        eprintln!("usage: hrfrec <name> <MHz> <MS/s> <seconds> [gain=40]");
        std::process::exit(2);
    }
    let name = a[0].clone();
    let mhz: f64 = a[1].parse().expect("centre frequency in MHz");
    let msps: f64 = a[2].parse().expect("sample rate in MS/s");
    let secs: f64 = a[3].parse().expect("duration in seconds");
    let gain: f32 = a.get(4).map(|v| v.parse().unwrap()).unwrap_or(40.0);

    let mut d = hackrf::HackRfDevice::open(0).expect("open HackRF");
    d.set_rate(Sps((msps * 1e6) as u64)).expect("set rate");
    d.set_center(Hz((mhz * 1e6) as u64)).expect("tune");
    d.set_gain("tuner", GainMode::Manual(gain)).expect("set gain");

    let path = format!("{name}_{mhz}M_{}k.cf32", (msps * 1000.0) as u64);
    let mut out = std::io::BufWriter::with_capacity(
        1 << 22,
        std::fs::File::create(&path).expect("create output"),
    );

    let mut s = d.start_rx().expect("start rx");
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

    let db = 20.0 * peak.max(1e-9).log10();
    println!("{path}: {got} samples, {:.1} s, peak {db:.1} dBFS", got as f64 / (msps * 1e6));
    // An eight-bit ADC has 48 dB to give and clipping costs the amplitude
    // structure a modulation classifier reads, so both ends matter more here
    // than they do on the LimeSDR.
    if db > -1.0 {
        eprintln!("warning: clipping, lower the gain");
    } else if db < -30.0 {
        eprintln!("warning: very quiet, raise the gain");
    }
}
