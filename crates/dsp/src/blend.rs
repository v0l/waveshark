//! Noise-driven treble cut, the "high blend" every car radio does.
//!
//! FM noise is triangular: its power density rises with baseband frequency, so
//! on a weak signal most of the audible hiss sits in the top octave while
//! almost none of the programme content does. Progressively lowpassing the
//! audio as the signal weakens therefore removes most of the noise for very
//! little loss, and is why a distant station on a car radio sounds muffled
//! rather than hissy.
//!
//! Noise is estimated from the discriminator output above 60 kHz, which is
//! beyond every baseband component (audio 15 kHz, pilot 19 kHz, stereo
//! difference to 53 kHz, RDS to 59.4 kHz). Whatever is up there is noise.

/// Measures FM noise from the ultrasonic part of the discriminator output.
pub struct NoiseMeter {
    /// Three cascaded one-pole highpasses, 18 dB/octave.
    hp: [f32; 3],
    hp_a: f32,
    lp: f32,
    lp_a: f32,
    hf: f32,
    lf: f32,
    alpha: f32,
}

impl NoiseMeter {
    pub fn new(rate: f64) -> Self {
        let pole = |f: f64| (1.0 - (-std::f64::consts::TAU * f / rate).exp()) as f32;
        Self {
            hp: [0.0; 3],
            // Above 60 kHz there is no baseband content at all: audio ends at
            // 15 kHz, the pilot is 19 kHz, stereo difference reaches 53 kHz and
            // RDS 59.4 kHz. A plain first difference has no cutoff and counts
            // the pilot and subcarriers as noise, which makes a strong stereo
            // station read noisier than empty spectrum.
            hp_a: pole(65_000.0),
            lp: 0.0,
            lp_a: pole(15_000.0),
            hf: 0.0,
            lf: 0.0,
            // ~100 ms: fast enough to follow a fade, slow enough not to pump
            // on programme transients.
            alpha: (1.0 / (rate * 0.1)) as f32,
        }
    }

    /// Feed discriminator samples; returns high-frequency energy as a
    /// fraction of the total.
    ///
    /// A ratio rather than an absolute level, so the reading does not move
    /// with gain, station strength or volume. An absolute meter has to be
    /// recalibrated for every receiver setting, which in practice means it is
    /// always wrong.
    pub fn process(&mut self, disc: &[f32]) -> f32 {
        for &v in disc {
            let mut x = v;
            for s in self.hp.iter_mut() {
                *s += self.hp_a * (x - *s);
                x -= *s;
            }
            self.hf += self.alpha * (x.abs() - self.hf);

            self.lp += self.lp_a * (v - self.lp);
            self.lf += self.alpha * (self.lp.abs() - self.lf);
        }
        self.level()
    }

    /// Out-of-band noise relative to audio-band content. Low is clean.
    pub fn level(&self) -> f32 {
        if self.lf > 1e-9 { self.hf / self.lf } else { 1.0 }
    }

    pub fn reset(&mut self) {
        self.hp = [0.0; 3];
        self.lp = 0.0;
        self.hf = 0.0;
        self.lf = 0.0;
    }
}

/// One-pole lowpass whose cutoff can move every block.
pub struct VariableLowpass {
    rate: f64,
    alpha: f32,
    state: f32,
    cutoff: f64,
}

impl VariableLowpass {
    pub fn new(rate: f64, cutoff: f64) -> Self {
        let mut s = Self { rate, alpha: 0.0, state: 0.0, cutoff: 0.0 };
        s.set_cutoff(cutoff);
        s
    }

    pub fn set_cutoff(&mut self, cutoff: f64) {
        let c = cutoff.clamp(200.0, self.rate / 2.0 * 0.99);
        self.cutoff = c;
        self.alpha = (1.0 - (-std::f64::consts::TAU * c / self.rate).exp()) as f32;
    }

    pub fn cutoff(&self) -> f64 {
        self.cutoff
    }

    pub fn process(&mut self, buf: &mut [f32]) {
        for v in buf.iter_mut() {
            self.state += self.alpha * (*v - self.state);
            *v = self.state;
        }
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Maps a noise estimate to an audio cutoff, with hysteresis-free smoothing.
pub struct HighBlend {
    lp: VariableLowpass,
    /// Cutoff when the signal is clean.
    max_cutoff: f64,
    /// Cutoff when the signal is at its worst.
    min_cutoff: f64,
    /// Noise level mapping to `max_cutoff` and `min_cutoff`.
    clean: f32,
    noisy: f32,
    current: f64,
}

impl HighBlend {
    pub fn new(audio_rate: f64) -> Self {
        Self {
            lp: VariableLowpass::new(audio_rate, 15_000.0),
            max_cutoff: 15_000.0,
            min_cutoff: 3_000.0,
            clean: 0.05,
            noisy: 0.60,
            current: 15_000.0,
        }
    }

    /// Calibrate the noise thresholds. `clean` should be the meter reading on
    /// a strong station and `noisy` the reading on an unusable one.
    pub fn set_range(&mut self, clean: f32, noisy: f32) {
        self.clean = clean;
        self.noisy = noisy.max(clean * 1.01);
    }

    pub fn cutoff(&self) -> f64 {
        self.current
    }

    /// Update the cutoff from a noise estimate and filter the audio.
    pub fn process(&mut self, noise: f32, audio: &mut [f32]) {
        let t = ((noise - self.clean) / (self.noisy - self.clean)).clamp(0.0, 1.0) as f64;
        // Interpolate in log frequency: hearing is logarithmic, and a linear
        // sweep spends most of its range where it is inaudible.
        let target = self.max_cutoff * (self.min_cutoff / self.max_cutoff).powf(t);
        // Glide rather than jump, or the timbre audibly steps.
        self.current += (target - self.current) * 0.1;
        self.lp.set_cutoff(self.current);
        self.lp.process(audio);
    }

    pub fn reset(&mut self) {
        self.lp.reset();
        self.current = self.max_cutoff;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn noise(n: usize, amp: f32, seed: u64) -> Vec<f32> {
        let mut s = seed;
        (0..n)
            .map(|_| {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (((s >> 33) as f32 / (1u64 << 31) as f32) - 0.5) * amp
            })
            .collect()
    }

    fn tone(n: usize, hz: f64, rate: f64) -> Vec<f32> {
        (0..n)
            .map(|i| ((hz * i as f64 / rate).rem_euclid(1.0) * std::f64::consts::TAU).sin() as f32)
            .collect()
    }

    fn rms(v: &[f32]) -> f32 {
        (v.iter().map(|x| x * x).sum::<f32>() / v.len() as f32).sqrt()
    }

    /// Audio-band tone plus a controllable amount of out-of-band noise.
    fn signal_with_noise(n: usize, hiss: f32, rate: f64, seed: u64) -> Vec<f32> {
        let tone = tone(n, 1_000.0, rate);
        let hf = noise(n, hiss, seed);
        tone.iter().zip(&hf).map(|(a, b)| a + b).collect()
    }

    #[test]
    fn the_meter_ranks_noise_levels_correctly() {
        // The reading is a ratio, so it must track *relative* noise, not
        // absolute level: scaling the whole signal must not change it.
        let rate = 300_000.0;
        let mut a = NoiseMeter::new(rate);
        let mut b = NoiseMeter::new(rate);
        let clean = a.process(&signal_with_noise(300_000, 0.01, rate, 1));
        let dirty = b.process(&signal_with_noise(300_000, 0.50, rate, 2));
        assert!(dirty > clean * 3.0, "clean {clean:.4}, dirty {dirty:.4}");
    }

    #[test]
    fn the_meter_is_independent_of_absolute_level() {
        let rate = 300_000.0;
        let mut a = NoiseMeter::new(rate);
        let mut b = NoiseMeter::new(rate);
        let quiet: Vec<f32> = signal_with_noise(300_000, 0.1, rate, 5);
        let loud: Vec<f32> = quiet.iter().map(|v| v * 10.0).collect();
        let x = a.process(&quiet);
        let y = b.process(&loud);
        assert!((x - y).abs() < x * 0.05, "level changed the reading: {x:.4} vs {y:.4}");
    }

    #[test]
    fn the_meter_mostly_ignores_low_frequency_content() {
        // A loud bass tone must not be mistaken for noise, or the blend closes
        // down on exactly the strong signals it should leave alone.
        let mut m = NoiseMeter::new(300_000.0);
        let audio = m.process(&tone(200_000, 400.0, 300_000.0));
        let mut m2 = NoiseMeter::new(300_000.0);
        let hiss = m2.process(&noise(200_000, 0.3, 3));
        assert!(hiss > audio * 5.0, "bass read as {audio:.4}, noise as {hiss:.4}");
    }

    #[test]
    fn variable_lowpass_attenuates_above_its_cutoff() {
        let rate = 48_000.0;
        let mut lp = VariableLowpass::new(rate, 3_000.0);
        let mut hi = tone(48_000, 12_000.0, rate);
        lp.process(&mut hi);
        let mut lp2 = VariableLowpass::new(rate, 3_000.0);
        let mut lo = tone(48_000, 500.0, rate);
        lp2.process(&mut lo);
        let db = 20.0 * (rms(&hi[2000..]) / rms(&lo[2000..])).log10();
        assert!(db < -10.0, "12 kHz only {db:.1} dB below 500 Hz");
    }

    #[test]
    fn a_clean_signal_keeps_full_bandwidth() {
        let mut hb = HighBlend::new(48_000.0);
        let mut audio = tone(4_800, 1_000.0, 48_000.0);
        for _ in 0..50 {
            hb.process(0.01, &mut audio);
        }
        assert!(hb.cutoff() > 12_000.0, "clean signal cut to {:.0} Hz", hb.cutoff());
    }

    #[test]
    fn a_noisy_signal_closes_the_treble_down() {
        let mut hb = HighBlend::new(48_000.0);
        let mut audio = tone(4_800, 1_000.0, 48_000.0);
        for _ in 0..100 {
            hb.process(0.9, &mut audio);
        }
        assert!(hb.cutoff() < 4_000.0, "noisy signal left cutoff at {:.0} Hz", hb.cutoff());
    }

    #[test]
    fn the_cutoff_glides_rather_than_jumping() {
        let mut hb = HighBlend::new(48_000.0);
        let mut audio = tone(480, 1_000.0, 48_000.0);
        let before = hb.cutoff();
        hb.process(0.9, &mut audio);
        let after = hb.cutoff();
        assert!(after < before, "cutoff did not move");
        assert!(after > before * 0.5, "cutoff jumped from {before:.0} to {after:.0}");
    }
}
