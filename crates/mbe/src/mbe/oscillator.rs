//! Port of jmbe `codec.oscillator.Oscillator`, used by the AMBE tone
//! generator. Rotates a unit phasor at `frequency` per sample of
//! `sample_rate`.

#[derive(Clone, Debug)]
pub struct Oscillator {
    angle_per_sample: (f32, f32),
    current_angle: (f32, f32),
    frequency: f64,
    sample_rate: f64,
}

impl Oscillator {
    pub fn new(frequency: f64, sample_rate: f64) -> Self {
        let mut oscillator = Self {
            angle_per_sample: (0.0, 0.0),
            current_angle: (0.0, -1.0),
            frequency,
            sample_rate,
        };
        oscillator.update();
        oscillator
    }

    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    pub fn set_frequency(&mut self, frequency: f64) {
        self.frequency = frequency;
        self.update();
    }

    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
        self.update();
    }

    /// Generates `sample_count` real samples at `gain`, starting with one
    /// rotation as the Java generate() does.
    pub fn generate(&mut self, sample_count: usize, gain: f32) -> Vec<f32> {
        let mut samples = vec![0.0f32; sample_count];

        if self.frequency != 0.0 {
            for sample in samples.iter_mut() {
                self.rotate();
                *sample = self.quadrature() * gain;
            }
        }

        samples
    }

    pub fn rotate(&mut self) {
        let (a_re, a_im) = self.angle_per_sample;
        let (re, im) = self.current_angle;
        let new_re = re * a_re - im * a_im;
        let new_im = re * a_im + im * a_re;

        // fastNormalize
        let magnitude = (new_re as f64 * new_re as f64 + new_im as f64 * new_im as f64).sqrt() as f32;
        if magnitude > 0.0 {
            self.current_angle = (new_re / magnitude, new_im / magnitude);
        } else {
            self.current_angle = (new_re, new_im);
        }
    }

    pub fn inphase(&self) -> f32 {
        self.current_angle.0
    }

    pub fn quadrature(&self) -> f32 {
        self.current_angle.1
    }

    fn update(&mut self) {
        let angle_per_sample =
            (2.0 * std::f64::consts::PI * self.frequency / self.sample_rate) as f32;
        self.angle_per_sample = (angle_per_sample.cos(), angle_per_sample.sin());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generates_a_unit_amplitude_sine() {
        let mut oscillator = Oscillator::new(1000.0, 8000.0);
        let samples = oscillator.generate(160, 1.0);
        assert!(samples.iter().all(|s| s.abs() <= 1.0 + 1e-4));
        assert!(samples.iter().any(|s| s.abs() > 0.9));
    }

    #[test]
    fn zero_frequency_is_silent() {
        let mut oscillator = Oscillator::new(0.0, 8000.0);
        let samples = oscillator.generate(160, 1.0);
        assert!(samples.iter().all(|s| *s == 0.0));
    }
}
