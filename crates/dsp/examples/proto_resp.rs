fn main() {
    for (m, t, a) in
        [(32usize, 12usize, 100.0f64), (32, 12, 90.0), (32, 16, 100.0), (32, 24, 110.0)]
    {
        let h = dsp::fir::pfb_prototype(m, t, a);
        // evaluate |H(f)| at 2 channels away (f = 2/m) and find worst-case in stopband
        let ev = |f: f64| -> f64 {
            let (mut re, mut im) = (0.0, 0.0);
            for (n, &c) in h.iter().enumerate() {
                let p = -2.0 * std::f64::consts::PI * f * n as f64;
                re += c as f64 * p.cos();
                im += c as f64 * p.sin();
            }
            (re * re + im * im).sqrt()
        };
        let dc = ev(0.0);
        let at2 = 20.0 * (ev(2.0 / m as f64) / dc).log10();
        let mut worst: f64 = -300.0;
        let mut k = 0;
        while k < 20000 {
            let f = 1.5 / m as f64 + (k as f64 / 20000.0) * (0.5 - 1.5 / m as f64);
            let v = 20.0 * (ev(f) / dc).log10();
            if v > worst {
                worst = v;
            }
            k += 1;
        }
        println!("m={m} t={t} atten_req={a}: |H| at 2ch = {at2:.1} dB, worst stopband = {worst:.1} dB, taps={}", h.len());
    }
}
