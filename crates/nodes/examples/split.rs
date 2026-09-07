//! Where does the time go: channelizer, transpose, or the per-channel graphs?
use common::C32;
use dsp::Channelizer;
use std::time::Instant;

fn main() {
    let rate = 50e6;
    let secs = 1.0;
    let n = (rate * secs) as usize;
    let sig: Vec<C32> = (0..n)
        .map(|i| C32::new((i as f32 * 0.001).sin(), (i as f32 * 0.001).cos()))
        .collect();
    for &m in &[64usize, 512] {
        let mut ch = Channelizer::new(m, 12, 90.0);
        let mut frames: Vec<C32> = Vec::with_capacity(n * 2 / m * m);
        let t = Instant::now();
        ch.process(&sig, |f| frames.extend_from_slice(f.samples));
        let t_ch = t.elapsed().as_secs_f64();

        let nf = frames.len() / m;
        let mut lanes: Vec<Vec<C32>> = (0..m).map(|_| Vec::with_capacity(nf)).collect();
        let t = Instant::now();
        for (c, lane) in lanes.iter_mut().enumerate() {
            for f in 0..nf {
                lane.push(frames[f * m + c]);
            }
        }
        let t_tr = t.elapsed().as_secs_f64();
        println!("M={m:3}: channelize {t_ch:.3}s ({:.2}x real)  transpose {:.3}s  -> channelizer alone caps at {:.1} MS/s",
            secs/t_ch, t_tr, rate/1e6*(secs/t_ch));
    }
}
