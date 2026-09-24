//! What a capture of a screen's emissions says about its raster.

use common::{C32, SampleFormat};
use std::io::Read;

fn main() {
    let path = std::env::args().nth(1).expect("a capture");
    let rate: f64 = std::env::args().nth(2).map_or(20e6, |s| s.parse().unwrap());
    let seconds: f64 = std::env::args().nth(3).map_or(1.5, |s| s.parse().unwrap());
    let fmt = match path.ends_with(".cs8") {
        true => SampleFormat::Cs8,
        false => SampleFormat::Cs16,
    };
    let mut f = std::fs::File::open(&path).expect("open");
    let mut raw = vec![0u8; (rate * seconds) as usize * fmt.bytes_per_sample()];
    f.read_exact(&mut raw).expect("read");
    let mut iq: Vec<C32> = Vec::new();
    fmt.convert(&raw, &mut iq);
    let decim = (rate / 2e6).round().max(1.0) as usize;
    let env: Vec<f32> = iq
        .chunks(decim)
        .map(|c| c.iter().map(|s| s.norm()).sum::<f32>() / c.len() as f32)
        .collect();
    let srate = rate / decim as f64;
    println!("{} samples at {rate}, search stream {} at {srate}", iq.len(), env.len());
    let limits = dsp::raster::Limits { line_hz: (25_175.0, 162_000.0), ..Default::default() };
    match dsp::raster::find_line(&env, srate, limits) {
        Some((line, score)) => println!(
            "line: {:.3} kHz, score {score:.0}; at 1125 lines {:.4} Hz, at 806 {:.4} Hz",
            srate / line / 1e3,
            srate / (line * 1125.0),
            srate / (line * 806.0)
        ),
        None => println!("no line"),
    }
    match dsp::raster::find_periods(&env, srate, limits) {
        Some(p) => println!(
            "locked: {} lines, frame {:.4} Hz, line {:.3} kHz, score {:.0}",
            p.lines,
            p.frame_hz(srate),
            p.line_hz(srate) / 1e3,
            p.score
        ),
        None => println!("no lock"),
    }
}
