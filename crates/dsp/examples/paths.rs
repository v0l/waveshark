fn main() {
    for (name, code) in [
        ("K7", dsp::conv::K7_X_FIRST),
        ("M17", dsp::conv::M17),
        ("TETRA 1/3", dsp::conv::TETRA_1_3),
    ] {
        let v = dsp::conv::Viterbi::new(code);
        println!("{name}: narrow/wide/butterfly {:?}", v.paths());
    }
}
