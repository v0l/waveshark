pub struct LinePredictor {
    weights: Vec<f32>,
    history: Vec<f32>,
    span: usize,
    pos: usize,
    mu: f32,
    keep: f32,
    level: f32,
    fall: f32,
}

impl LinePredictor {
    pub fn new(taps: usize, delay: usize, mu: f32, leak: f32) -> Self {
        let taps = taps.max(1);
        let span = taps + delay.max(1);
        Self {
            weights: vec![0.0; taps],
            history: vec![0.0; 2 * span],
            span,
            pos: 0,
            mu,
            keep: 1.0 - leak,
            level: 0.0,
            fall: 1.0 - 1.0 / span as f32,
        }
    }

    pub fn predict(&mut self, x: f32) -> f32 {
        let span = self.span;
        self.history[self.pos] = x;
        self.history[self.pos + span] = x;
        let start = self.pos + 1;
        let past = &self.history[start..start + self.weights.len()];
        let (mut y, mut power) = (0.0f32, 0.0f32);
        for (w, p) in self.weights.iter().zip(past) {
            y += w * p;
            power += p * p;
        }
        self.level = (self.level * self.fall).max(x * x);
        let g = self.mu * (x - y) / (power + self.level * past.len() as f32 + 1e-12);
        for (w, p) in self.weights.iter_mut().zip(past) {
            *w = *w * self.keep + g * p;
        }
        self.pos = if self.pos + 1 == span { 0 } else { self.pos + 1 };
        y
    }

    pub fn reset(&mut self) {
        self.weights.fill(0.0);
        self.history.fill(0.0);
        self.level = 0.0;
        self.pos = 0;
    }
}

pub struct AutoNotch {
    predictor: LinePredictor,
}

impl AutoNotch {
    pub fn new(rate: f64) -> Self {
        let per_ms = rate / 1000.0;
        let taps = (NOTCH_TAPS_MS * per_ms).round() as usize;
        let delay = (NOTCH_DELAY_MS * per_ms).round() as usize;
        Self { predictor: LinePredictor::new(taps, delay, NOTCH_MU, NOTCH_LEAK) }
    }

    pub fn sample(&mut self, x: f32) -> f32 {
        x - self.predictor.predict(x)
    }

    pub fn process(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.sample(*x);
        }
    }

    pub fn reset(&mut self) {
        self.predictor.reset();
    }
}

const NOTCH_TAPS_MS: f64 = 4.0;
const NOTCH_DELAY_MS: f64 = 2.0;
const NOTCH_MU: f32 = 0.01;
const NOTCH_LEAK: f32 = 1e-5;

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 48_000.0;

    fn tone(n: usize, hz: f64, amp: f32) -> Vec<f32> {
        (0..n)
            .map(|i| {
                amp * ((hz * i as f64 / RATE).rem_euclid(1.0) * std::f64::consts::TAU).sin() as f32
            })
            .collect()
    }

    fn noise(n: usize, amp: f32, seed: u64) -> Vec<f32> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (((s >> 33) as f32 / (1u64 << 31) as f32) - 0.5) * amp
            })
            .collect()
    }

    fn voice_band(n: usize, amp: f32, seed: u64) -> Vec<f32> {
        let taps = crate::filter::design(
            crate::filter::Response::Bandpass,
            255,
            RATE,
            1_500.0,
            2_400.0,
            60.0,
        );
        let raw = noise(n + taps.len(), 1.0, seed);
        let out: Vec<f32> =
            (0..n).map(|i| taps.iter().zip(&raw[i..]).map(|(t, x)| t * x).sum()).collect();
        let r = rms(&out);
        out.iter().map(|x| x * amp / r).collect()
    }

    fn syllables(n: usize, amp: f32) -> Vec<f32> {
        let mut phase = 0.0f64;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / RATE;
            let pitch = 140.0 + 30.0 * (t * 3.1).sin();
            phase += pitch / RATE;
            let on = (t * 4.0).rem_euclid(1.0) < 0.7;
            let v: f64 = (1..=18)
                .map(|h| {
                    let hz = pitch * h as f64;
                    let formant = (-((hz - 700.0) / 400.0).powi(2)).exp()
                        + 0.6 * (-((hz - 1_800.0) / 500.0).powi(2)).exp();
                    formant * (phase * h as f64 * std::f64::consts::TAU).sin()
                })
                .sum();
            out.push(if on { v as f32 } else { 0.0 });
        }
        let r = rms(&out);
        out.iter().map(|x| x * amp / r).collect()
    }

    fn rms(v: &[f32]) -> f32 {
        (v.iter().map(|x| x * x).sum::<f32>() / v.len().max(1) as f32).sqrt()
    }

    fn goertzel_amp(v: &[f32], hz: f64) -> f64 {
        let w = std::f64::consts::TAU * hz / RATE;
        let coeff = 2.0 * w.cos();
        let (mut s1, mut s2) = (0.0f64, 0.0f64);
        for x in v {
            let s0 = f64::from(*x) + coeff * s1 - s2;
            s2 = s1;
            s1 = s0;
        }
        2.0 * (s1 * s1 + s2 * s2 - coeff * s1 * s2).max(0.0).sqrt() / v.len() as f64
    }

    fn db(x: f64) -> f64 {
        20.0 * x.log10()
    }

    fn notched(x: &[f32]) -> Vec<f32> {
        let mut out = x.to_vec();
        AutoNotch::new(RATE).process(&mut out);
        out
    }

    fn settled(v: &[f32]) -> &[f32] {
        &v[v.len() / 2..]
    }

    fn add(a: &[f32], b: &[f32]) -> Vec<f32> {
        a.iter().zip(b).map(|(x, y)| x + y).collect()
    }

    const SECONDS: usize = 4;

    #[test]
    fn a_heterodyne_alone_goes_down_44_db() {
        let n = RATE as usize * SECONDS;
        let out = notched(&tone(n, 1_234.0, 0.3));
        let depth = db(goertzel_amp(settled(&out), 1_234.0) / 0.3);
        assert!((-47.0..=-42.0).contains(&depth), "notch depth {depth:.1} dB, measured -44.6");
    }

    #[test]
    fn under_speech_a_heterodyne_goes_down_24_db() {
        let n = RATE as usize * SECONDS;
        let mixed = add(&syllables(n, 0.2), &tone(n, 1_234.0, 0.3));
        let out = notched(&mixed);
        let depth = db(goertzel_amp(settled(&out), 1_234.0) / 0.3);
        assert!((-27.0..=-22.0).contains(&depth), "notch depth {depth:.1} dB, measured -24.8");
    }

    #[test]
    fn a_second_weaker_heterodyne_goes_down_11_db_beside_the_first() {
        let n = RATE as usize * SECONDS;
        let two = add(&tone(n, 1_234.0, 0.3), &tone(n, 2_100.0, 0.1));
        let out = notched(&add(&syllables(n, 0.2), &two));
        let first = db(goertzel_amp(settled(&out), 1_234.0) / 0.3);
        let second = db(goertzel_amp(settled(&out), 2_100.0) / 0.1);
        assert!((-27.0..=-22.0).contains(&first), "first {first:.1} dB, measured -24.6");
        assert!((-13.0..=-9.0).contains(&second), "second {second:.1} dB, measured -11.0");
    }

    #[test]
    fn voiced_speech_loses_a_quarter_db() {
        let n = RATE as usize * SECONDS;
        let speech = syllables(n, 0.2);
        let out = notched(&speech);
        let loss = db(rms(settled(&out)) as f64 / rms(settled(&speech)) as f64);
        assert!((-0.5..=0.0).contains(&loss), "speech changed by {loss:.2} dB, measured -0.22");
    }

    #[test]
    fn voice_band_noise_passes_within_a_tenth_of_a_db() {
        let n = RATE as usize * SECONDS;
        let hiss = voice_band(n, 0.2, 7);
        let out = notched(&hiss);
        let loss = db(rms(settled(&out)) as f64 / rms(settled(&hiss)) as f64);
        assert!((-0.1..=0.0).contains(&loss), "noise changed by {loss:.2} dB, measured -0.03");
    }

    #[test]
    fn a_tone_that_jumps_is_down_20_db_within_30_ms() {
        let second = RATE as usize;
        let jump: Vec<f32> =
            tone(second, 1_000.0, 0.3).into_iter().chain(tone(second, 1_700.0, 0.3)).collect();
        let out = notched(&jump);
        let ten_ms = second / 100;
        let quiet = 0.3 / 2f32.sqrt() * 0.1;
        let settle = out[second..].chunks(ten_ms).position(|c| rms(c) < quiet).map(|k| k * 10);
        assert_eq!(settle, Some(30), "milliseconds until the new tone was 20 dB down");
    }

    #[test]
    fn speech_after_silence_peaks_within_half_a_db_of_the_input() {
        let n = RATE as usize * SECONDS;
        let speech = syllables(n, 0.2);
        let out = notched(&speech);
        let peak = |v: &[f32]| v.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        let ratio = db((peak(&out) / peak(&speech)) as f64);
        assert!(ratio <= 0.5, "output peak {ratio:.2} dB over the input's, measured 0.04");
    }

    #[test]
    fn silence_stays_silent() {
        let out = notched(&vec![0.0; RATE as usize]);
        assert_eq!(out.iter().filter(|x| **x != 0.0).count(), 0);
    }

    #[test]
    fn reset_forgets_the_tone() {
        let n = RATE as usize;
        let mut notch = AutoNotch::new(RATE);
        let mut first = tone(n, 1_234.0, 0.3);
        notch.process(&mut first);
        notch.reset();
        let mut again = tone(n, 1_234.0, 0.3);
        notch.process(&mut again);
        assert_eq!(first, again);
    }
}
