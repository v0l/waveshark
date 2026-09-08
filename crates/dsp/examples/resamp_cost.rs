use common::C32;
fn main() {
    let n = 20_480_000usize;
    let iq: Vec<C32> = (0..n).map(|i| C32::new((i as f32 * 0.01).sin(), 0.0)).collect();
    let mut r = dsp::resample::Rational::new(20.48e6, 20e6, 4096).unwrap();
    let mut out = Vec::with_capacity(n);
    let t = std::time::Instant::now();
    for b in iq.chunks(65_536) {
        out.clear();
        r.process(b, &mut out);
    }
    let el = t.elapsed().as_secs_f64();
    println!("resample 20.48 -> 20 MS/s: 1.0 s of air in {el:.3} s = {:.0}% of a core", el * 100.0);
}
