//! Look for DVB-T multiplexes in a capture: `dvbt_probe <file> [seconds]`.
//!
//! The centre and rate come from the file name. Every 8 MHz channel the span
//! covers is mixed down, resampled to the standard's rate and handed to the
//! receiver, which either locks and says what the transmission parameters are
//! or does not.

use common::C32;
use dsp::resample::Rational;
use nodes::dvbt_nodes::{DvbtReceiver, RATE_HZ};

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("usage: dvbt_probe <file> [seconds]");
    let want_s: f64 = args.next().map(|s| s.parse().expect("seconds")).unwrap_or(4.0);
    let meta = sources::file::parse_filename(std::path::Path::new(&path));
    let center = meta.center.expect("a centre in the name").as_f64();
    let rate = meta.rate.expect("a rate in the name").as_f64();
    let format = meta.format.expect("a format from the extension");
    println!("{path}: {:.3} MHz at {:.3} MS/s, {format:?}", center / 1e6, rate / 1e6);

    let raw = std::fs::read(&path).expect("capture");
    let want = (want_s * rate) as usize;
    let samples: Vec<C32> = match format {
        common::SampleFormat::Cs16 => raw
            .chunks_exact(4)
            .take(want)
            .map(|c| {
                let i = i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0;
                let q = i16::from_le_bytes([c[2], c[3]]) as f32 / 32_768.0;
                C32::new(i, q)
            })
            .collect(),
        _ => raw
            .chunks_exact(2)
            .take(want)
            .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
            .collect(),
    };
    println!("{:.2} s of samples", samples.len() as f64 / rate);

    // Every channel centre of the 8 MHz plan that the span holds whole.
    let half = rate / 2.0 - 4.0e6;
    let mut hz = ((center - half - 474.0e6) / 8.0e6).ceil() * 8.0e6 + 474.0e6;
    while hz <= center + half {
        probe(&samples, rate, center, hz);
        hz += 8.0e6;
    }
}

fn probe(samples: &[C32], rate: f64, center: f64, hz: f64) {
    let factor = (rate / RATE_HZ).floor().max(1.0) as usize;
    let mut mixer = dsp::mixer::Mixer::new(center - hz, rate);
    let mut decim = dsp::fir::FirDecim::design_hz(rate, factor, 4.0e6, 60.0);
    let mut resample = Rational::approx(rate / factor as f64, RATE_HZ, 4096);
    let mut rx = DvbtReceiver::new();

    let (mut mixed, mut narrow, mut at_rate, mut packets) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for block in samples.chunks(262_144) {
        mixed.clear();
        narrow.clear();
        at_rate.clear();
        mixer.process(block, &mut mixed);
        decim.process(&mixed, &mut narrow);
        resample.process(&narrow, &mut at_rate);
        rx.push(&at_rate, &mut packets);
    }
    let ch = ((hz - 474.0e6) / 8.0e6).round() as i64 + 21;
    match rx.params() {
        Some(p) => println!(
            "  {:.1} MHz (ch {ch}): {:?} {:?} {:?} guard {:?}, {} packets",
            hz / 1e6,
            p.mode,
            p.constellation,
            p.code_rate_hp,
            p.guard,
            packets.len()
        ),
        None => println!("  {:.1} MHz (ch {ch}): no lock", hz / 1e6),
    }
}
