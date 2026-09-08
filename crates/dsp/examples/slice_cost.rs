//! What slicing a wide span costs, both ways.
use common::C32;
use std::time::Instant;

fn noise(n: usize) -> Vec<C32> {
    let mut s = 0x1234_5678u32;
    (0..n)
        .map(|_| {
            let mut r = || {
                s ^= s << 13;
                s ^= s >> 17;
                s ^= s << 5;
                s as f32 / u32::MAX as f32 - 0.5
            };
            C32::new(r(), r())
        })
        .collect()
}

fn main() {
    let rate = 61_440_000.0;
    let secs = 1.0;
    let iq = noise((rate * secs) as usize);

    for chans in [1usize, 3, 6, 12] {
        let list: Vec<f64> = (0..chans).map(|i| 2_457e6 - 20e6 + i as f64 * 3e6).collect();
        let mut s = dsp::slice::Slicer::new(rate, 2_457e6, 20e6, &list, 3072).expect("slicer");
        let mut out = Vec::new();
        let t = Instant::now();
        for block in iq.chunks(65_536) {
            s.process(block, &mut out);
        }
        let el = t.elapsed().as_secs_f64();
        println!(
            "fast convolution, {chans:>2} channels: {:.3}s for {secs}s = {:.1}x real time",
            el,
            secs / el
        );
    }

    // The other way: a mixer and a decimator per channel, which is what
    // `dsp::source` does for a source too wide for the bank.
    for chans in [1usize, 3] {
        let mut rx: Vec<(dsp::Mixer, dsp::FirDecim)> = (0..chans)
            .map(|i| {
                (
                    dsp::Mixer::new(i as f64 * 5e6, rate),
                    dsp::FirDecim::design_hz(rate, 3, 8_300_000.0, 60.0),
                )
            })
            .collect();
        let t = Instant::now();
        let (mut mixed, mut out) = (Vec::new(), Vec::new());
        for block in iq.chunks(65_536) {
            for (m, d) in rx.iter_mut() {
                mixed.clear();
                m.process(block, &mut mixed);
                out.clear();
                d.process(&mixed, &mut out);
            }
        }
        let el = t.elapsed().as_secs_f64();
        println!(
            "mixer and decimator, {chans:>2} channels: {:.3}s for {secs}s = {:.1}x real time \
             (and lands on 20.48 MS/s, not 20)",
            el,
            secs / el
        );
    }
}
