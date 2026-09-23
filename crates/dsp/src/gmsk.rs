use common::C32;

pub fn gaussian_pulse(sps: usize, bt: f64) -> Vec<f32> {
    let span = 4;
    let n = span * sps;
    let sigma = (2f64.ln()).sqrt() / (std::f64::consts::TAU * bt);
    let mut p = vec![0.0f64; n];
    for (i, v) in p.iter_mut().enumerate() {
        let t = (i as f64 + 0.5) / sps as f64 - span as f64 / 2.0;
        let steps = 32;
        let mut acc = 0.0;
        for k in 0..steps {
            let u = t - 0.5 + (k as f64 + 0.5) / steps as f64;
            acc += (-u * u / (2.0 * sigma * sigma)).exp();
        }
        *v = acc / steps as f64;
    }
    let area: f64 = p.iter().sum();
    p.iter().map(|&v| (v / area) as f32).collect()
}

pub fn modulate(symbols: &[f32], sps: usize, bt: f64) -> Vec<C32> {
    let pulse = gaussian_pulse(sps, bt);
    let span = pulse.len() / sps;
    let mut freq = vec![0.0f32; (symbols.len() + span) * sps];
    for (i, &a) in symbols.iter().enumerate() {
        for (j, &p) in pulse.iter().enumerate() {
            freq[i * sps + j] += a * p;
        }
    }
    let mut phase = 0.0f64;
    freq.iter()
        .map(|&f| {
            phase += std::f64::consts::FRAC_PI_2 * f64::from(f);
            C32::new(phase.cos() as f32, phase.sin() as f32)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_symbol_turns_the_phase_a_quarter_turn_whatever_the_bt() {
        for bt in [0.3, 0.4, 0.5] {
            let sum: f32 = gaussian_pulse(8, bt).iter().sum();
            assert!((sum - 1.0).abs() < 1e-5, "BT {bt}: pulse area {sum}");
            let iq = modulate(&[1.0; 16], 8, bt);
            let turn = (iq[12 * 8] * iq[11 * 8].conj()).arg();
            assert!((turn - std::f32::consts::FRAC_PI_2).abs() < 1e-3, "BT {bt}: {turn}");
        }
    }

    #[test]
    fn a_lower_bt_spreads_the_pulse_wider() {
        let edge = |bt| gaussian_pulse(8, bt)[4];
        assert!(edge(0.3) > edge(0.4) && edge(0.4) > edge(0.5));
    }
}
