//! Sweep a band on the LimeSDR and print what stands above the floor.
use common::{Device, GainMode, Hz, Sps};
fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    let lo: f64 = a.first().map(|v| v.parse().unwrap()).unwrap_or(87.5);
    let hi: f64 = a.get(1).map(|v| v.parse().unwrap()).unwrap_or(108.0);
    let ant = a.get(2).cloned().unwrap_or_else(|| "LNAH".into());
    let chan = a.get(3).cloned().unwrap_or_else(|| "RX2".into());
    let mut d = limesdr::LimeSdr::open(0).unwrap();
    d.set_rate(Sps(4_000_000)).unwrap();
    d.set_choice("channel", &chan).unwrap();
    d.set_choice("antenna", &ant).unwrap();
    d.set_gain("gain", GainMode::Manual(60.0)).unwrap();

    let mut found: Vec<(f64, f32, f32)> = Vec::new();
    let mut f = lo * 1e6 + 1.5e6;
    while f < hi * 1e6 {
        d.set_center(Hz(f as u64)).unwrap();
        let mut s = d.start_rx().unwrap();
        let mut sp = dsp::spectrum::Spectrum::new(4096);
        for _ in 0..10 { let _ = s.read(); }
        for _ in 0..6 { let b = s.read().unwrap(); sp.process(&b.samples); }
        drop(s);
        let db = sp.power_db().to_vec();
        let n = db.len();
        let mut sorted = db.clone();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted[n / 2];
        for (i, v) in db.iter().enumerate() {
            // Middle is the DC spur, edges are the filter roll-off.
            if i.abs_diff(n / 2) < 24 || i < n / 8 || i > n * 7 / 8 { continue; }
            if *v - floor > 15.0 {
                found.push((f + (i as f64 - n as f64 / 2.0) * 4e6 / n as f64, *v, *v - floor));
            }
        }
        f += 3.0e6;
    }
    // Keep the strongest bin per 150 kHz so one station is one line.
    found.sort_by(|a, b| b.1.total_cmp(&a.1));
    let mut kept: Vec<(f64, f32, f32)> = Vec::new();
    for c in found {
        if kept.iter().all(|k| (k.0 - c.0).abs() > 150e3) { kept.push(c); }
    }
    kept.sort_by(|a, b| a.0.total_cmp(&b.0));
    println!("{ant} on {chan}, {lo}-{hi} MHz, 60 dB gain");
    for (hz, db, up) in kept {
        println!("  {:>9.3} MHz  {db:>7.1} dBFS  {up:>5.1} dB above floor", hz / 1e6);
    }
}
