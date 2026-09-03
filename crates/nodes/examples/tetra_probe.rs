//! Probe the TETRA fixture carrier by carrier, printing what each layer saw.
//!
//! cargo run --release -p nodes --example tetra_probe -- testdata/tetra_downlink_391.5M_2400k.cu8

use dsp::tetra::{TetraConfig, TetraDemod, TetraRx};
use dsp::{FirDecim, Mixer};

fn main() {
    let path = std::env::args().nth(1).expect("path to capture");
    let buf = sources::FileSource::open(std::path::Path::new(&path))
        .unwrap()
        .read_all()
        .unwrap();
    let rate = buf.rate.as_f64();
    let center = buf.center.as_f64();
    for hz in [391_181_000.0f64, 391_704_500.0] {
        for out_rate in [72_000.0, 25_000.0] {
            let factor = (rate / out_rate).round() as usize;
            let mut mixer = Mixer::new(center - hz, rate);
            let mut decim = FirDecim::design_hz(rate, factor, 12_150.0, 60.0);
            let mut demod = TetraDemod::new(rate / factor as f64, TetraConfig::default());
            let mut rx = TetraRx::new();
            let (mut mixed, mut narrow) = (Vec::new(), Vec::new());
            let mut quality = Vec::new();
            let mut blocks = Vec::new();
            let mut kinds = std::collections::BTreeMap::new();
            for chunk in buf.samples.chunks(65_536) {
                mixed.clear();
                mixer.process(chunk, &mut mixed);
                narrow.clear();
                decim.process(&mixed, &mut narrow);
                let mut got = Vec::new();
                demod.process(&narrow, &mut got);
                for b in &got {
                    *kinds.entry(format!("{:?}", b.kind)).or_insert(0u32) += 1;
                    rx.push(b, &mut blocks);
                }
                quality.extend(got.iter().map(|b| b.quality));
            }
            let mean_q = quality.iter().sum::<f32>() / quality.len().max(1) as f32;
            println!(
                "{:.4} MHz @ {:.0} S/s: {} bursts (q~{mean_q:.2}) {kinds:?}, stats {:?}, {} blocks ({} failed), cell {:?}",
                hz / 1e6,
                rate / factor as f64,
                quality.len(),
                demod.stats(),
                blocks.len(),
                rx.failed,
                rx.cell
            );
        }
    }
}
