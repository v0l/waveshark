//! Where the time in the trellis goes: the step itself, the step through
//! `push` with its puncturing, and the traceback.
fn main() {
    let n = 4_000_000usize;
    let soft: Vec<f32> = (0..2 * n).map(|k| if k % 3 == 0 { 0.9 } else { -0.8 }).collect();

    let mut v = dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST);
    let mut out = Vec::new();
    let t = std::time::Instant::now();
    for pair in soft.chunks_exact(2) {
        v.push_step(pair, &mut out);
    }
    let step = t.elapsed().as_secs_f64();
    println!(
        "push_step {:.1} Msteps/s ({:.1} ns a step), {} bits",
        n as f64 / step / 1e6,
        step / n as f64 * 1e9,
        out.len()
    );

    let mut v = dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST);
    let mut out = Vec::new();
    let t = std::time::Instant::now();
    v.push(&soft, dsp::conv::P_1_2, &mut out);
    let whole = t.elapsed().as_secs_f64();
    println!(
        "push {:.1} Msteps/s ({:.1} ns a step)",
        n as f64 / whole / 1e6,
        whole / n as f64 * 1e9
    );

    for (name, mut v) in [
        ("whole numbers, sixteen lanes", dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST)),
        ("floats, eight lanes", dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST).only_floats()),
        ("floats, one at a time", dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST).only_scalar()),
    ] {
        let mut out = Vec::new();
        let t = std::time::Instant::now();
        v.push(&soft, dsp::conv::P_1_2, &mut out);
        let took = t.elapsed().as_secs_f64();
        println!("  {name}: {:.1} Msteps/s", n as f64 / took / 1e6);
    }

    // The same through a puncturing mask, which is what DVB-T at 2/3 uses.
    let mask = [1u8, 1, 1, 0];
    let mut v = dsp::conv::Viterbi::new(dsp::conv::K7_X_FIRST);
    let mut out = Vec::new();
    let t = std::time::Instant::now();
    v.push(&soft, &mask, &mut out);
    let punctured = t.elapsed().as_secs_f64();
    println!("push punctured {:.1} Msteps/s", out.len() as f64 / punctured / 1e6);
}
