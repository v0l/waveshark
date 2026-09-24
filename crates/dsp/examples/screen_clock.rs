//! The pixel clock as a carrier, against the pixel clock as a period.

use common::{C32, SampleFormat};
use std::io::Read;

fn main() {
    let path = std::env::args().nth(1).expect("a capture");
    let rate: f64 = std::env::args().nth(2).map_or(20e6, |s| s.parse().unwrap());
    let dial: f64 = std::env::args().nth(3).map_or(1485e6, |s| s.parse().unwrap());
    let nominal: f64 = std::env::args().nth(4).map_or(148.5e6, |s| s.parse().unwrap());
    let totals: f64 = std::env::args().nth(5).map_or(2200.0, |s| s.parse().unwrap());
    let fmt = match path.ends_with(".cs8") {
        true => SampleFormat::Cs8,
        false => SampleFormat::Cs16,
    };
    let mut f = std::fs::File::open(&path).expect("open");
    let mut raw = vec![0u8; (rate * 0.2) as usize * fmt.bytes_per_sample()];
    f.read_exact(&mut raw).expect("read");
    let mut iq: Vec<C32> = Vec::new();
    fmt.convert(&raw, &mut iq);

    let h = (dial / nominal).round();
    match dsp::raster::find_carrier(&iq, rate, (-400e3, 400e3)) {
        Some(c) => {
            let clock = (dial + c.offset_hz) / h;
            println!(
                "carrier {:+.1} Hz, {:.1} dB over median -> harmonic {h}, clock {:.3} Hz, line {:.4} Hz",
                c.offset_hz,
                c.over_db,
                clock,
                clock / totals
            );
        }
        None => println!("no carrier"),
    }
    let decim = (rate / 2e6).round().max(1.0) as usize;
    let env: Vec<f32> = iq
        .chunks(decim)
        .map(|c| c.iter().map(|s| s.norm()).sum::<f32>() / c.len() as f32)
        .collect();
    let srate = rate / decim as f64;
    let limits = dsp::raster::Limits { line_hz: (25_175.0, 250_000.0), ..Default::default() };
    match dsp::raster::find_line(&env, srate, limits) {
        Some((line, _)) => println!("ladder line {:.4} Hz", srate / line),
        None => println!("ladder: no line"),
    }
}
