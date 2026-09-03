//! Shared MBE synthesiser core, ported from jmbe `jmbe.codec`:
//! `MBESynthesizer`, `MBEModelParameters`, the noise generators and the
//! tone oscillator. The `ambe` and `imbe` modules hold everything codec
//! specific.

pub mod oscillator;

use crate::fft::RealFft256;
use crate::window::synthesis_window;

pub const SAMPLES_PER_FRAME: usize = 160;

const TWO_PI: f32 = std::f32::consts::TAU;
const TWO56_OVER_TWO_PI: f32 = 256.0 / std::f32::consts::TAU;
const AUDIO_SCALAR_16_BITS_SIGNED: f32 = 1.0 / 32767.0;
const MAXIMUM_AUDIO_AMPLITUDE: f32 = 0.95;
const WHITE_NOISE_SCALAR: f32 = std::f32::consts::TAU / 53125.0;

// Unvoiced scaling coefficient (yw) from synthesis window (ws) and pitch
// refinement window (wr), algorithm 121.
const UNVOICED_SCALING_COEFFICIENT: f32 = 146.17696;

/// MBE frame type, port of `jmbe.codec.FrameType`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameType {
    Voice,
    Erasure,
    Silence,
    Tone,
}

impl FrameType {
    /// Port of `FrameType.fromValue`, for the D-STAR/DMR frame type field.
    pub fn from_value(value: u32) -> Option<FrameType> {
        match value {
            0..=119 => Some(FrameType::Voice),
            120..=123 => Some(FrameType::Erasure),
            124 | 125 => Some(FrameType::Silence),
            126 | 127 => Some(FrameType::Tone),
            _ => None,
        }
    }
}

/// Multi-Band Excitation voice frame model parameters required to
/// synthesize one audio frame. Port of `jmbe.codec.MBEModelParameters`.
#[derive(Clone, Debug)]
pub struct ModelParameters {
    pub frame_type: FrameType,
    /// Fundamental frequency in the range 0.0 to 0.5.
    pub frequency: f32,
    /// Number of frequency bands to synthesize.
    pub l: usize,
    /// Voicing decision per band; index 0 is unused, matching the Java
    /// arrays that are indexed from 1.
    pub voicing: Vec<bool>,
    pub log2_spectral_amplitudes: Vec<f32>,
    pub spectral_amplitudes: Vec<f32>,
    pub enhanced_spectral_amplitudes: Vec<f32>,
    pub local_energy: f32,
    pub amplitude_threshold: i32,
    pub error_rate: f32,
    /// Total number of bit errors detected/corrected for the frame.
    pub error_count: i32,
    /// Bit error count for coset 4.
    pub error_count4: i32,
    pub repeat_count: i32,
}

impl ModelParameters {
    /// Defaults matching the Java field initializers: local energy 75000,
    /// amplitude threshold 20480.
    pub fn new() -> Self {
        Self {
            frame_type: FrameType::Voice,
            frequency: 0.0,
            l: 0,
            voicing: vec![false; 1],
            log2_spectral_amplitudes: vec![],
            spectral_amplitudes: vec![],
            enhanced_spectral_amplitudes: vec![],
            local_energy: 75000.0,
            amplitude_threshold: 20480,
            error_rate: 0.0,
            error_count: 0,
            error_count4: 0,
            repeat_count: 0,
        }
    }

    /// Indicates if any of the L frequency band harmonics are voiced.
    pub fn has_voiced_bands(&self) -> bool {
        self.voicing.iter().any(|v| *v)
    }

    /// Count of the unvoiced bands in this frame.
    pub fn unvoiced_band_count(&self) -> usize {
        self.voicing.iter().filter(|v| !**v).count()
    }

    /// Indicates if adaptive smoothing is required when the error rate
    /// threshold is exceeded.
    pub fn requires_adaptive_smoothing(&self) -> bool {
        self.error_rate > 0.0125 || self.error_count > 4
    }

    pub fn is_repeat_frame(&self) -> bool {
        self.repeat_count > 0
    }

    /// Indicates the frame repeat count has exceeded the muting threshold.
    pub fn is_max_frame_repeat(&self) -> bool {
        self.repeat_count >= 4
    }

    /// Port of `setSpectralAmplitudes`: stores the decoded amplitudes and
    /// runs the chapter 8 enhancement against the previous frame's energy.
    pub fn set_spectral_amplitudes(
        &mut self,
        spectral_amplitudes: Vec<f32>,
        previous_local_energy: f32,
        previous_amplitude_threshold: i32,
    ) {
        self.spectral_amplitudes = spectral_amplitudes;
        self.enhance_spectral_amplitudes(previous_local_energy, previous_amplitude_threshold);
    }

    /// Port of `enhanceSpectralAmplitudes` (algorithms 105 to 116).
    fn enhance_spectral_amplitudes(
        &mut self,
        previous_local_energy: f32,
        previous_amplitude_threshold: i32,
    ) {
        const PI_96: f64 = 0.96 * std::f64::consts::PI;

        let frequency = self.frequency as f64;
        let band_count = self.l;

        // RM0 and RM1 accumulate as f32 per step, matching the Java float
        // arrays that narrow the double cos term on each +=.
        let mut rm0 = 0.0f32;
        let mut rm1 = 0.0f32;

        for l in 1..=band_count {
            let amplitude_squared = self.spectral_amplitudes[l] * self.spectral_amplitudes[l];
            rm0 += amplitude_squared;
            rm1 += amplitude_squared * (frequency * l as f64).cos() as f32;
        }

        let mut enhanced = vec![0.0f32; band_count + 1];

        if rm0 == 0.0 {
            self.enhanced_spectral_amplitudes = enhanced;
            return;
        }

        let rm0_squared = rm0 * rm0;
        let rm1_squared = rm1 * rm1;

        // Algorithm 107 - calculate enhancement weights (W).
        let mut weights = vec![0.0f64; band_count + 1];
        for l in 1..=band_count {
            let temp = (PI_96 as f32
                * (rm0_squared + rm1_squared
                    - (2.0 * rm0 * rm1 * (frequency * l as f64).cos() as f32)))
                / (self.frequency * rm0 * (rm0_squared - rm1_squared));
            weights[l] = (self.spectral_amplitudes[l] as f64).sqrt() * (temp as f64).powf(0.25);
        }

        // Algorithm 108 - apply weights to produce enhanced amplitudes.
        for l in 1..=band_count {
            if 8 * l <= band_count {
                enhanced[l] = self.spectral_amplitudes[l];
            } else if weights[l] > 1.2 {
                enhanced[l] = self.spectral_amplitudes[l] * 1.2;
            } else if weights[l] < 0.5 {
                enhanced[l] = self.spectral_amplitudes[l] * 0.5;
            } else {
                enhanced[l] = self.spectral_amplitudes[l] * weights[l] as f32;
            }
        }

        // Algorithm 109 - remove energy differential of enhanced amplitudes.
        let mut denominator = 0.0f32;
        for l in 1..=band_count {
            denominator += enhanced[l] * enhanced[l];
        }

        let y = (rm0 / denominator).sqrt();

        // Algorithm 110 - scale enhanced amplitudes to remove energy
        // differential.
        for l in 1..=band_count {
            enhanced[l] *= y as f32;
        }

        // Algorithm 111 - calculate local energy.
        self.local_energy = 0.95 * previous_local_energy + 0.05 * rm0;
        if self.local_energy < 10000.0 {
            self.local_energy = 10000.0;
        }

        self.enhanced_spectral_amplitudes = enhanced;
        self.apply_adaptive_smoothing(previous_amplitude_threshold);
    }

    /// Port of `applyAdaptiveSmoothing` on enhanced spectral amplitudes and
    /// the voicing decisions when the error rate is above the threshold
    /// that causes audio distortions or discontinuities.
    fn apply_adaptive_smoothing(&mut self, previous_amplitude_threshold: i32) {
        let l = self.l;

        // Algorithm 112 - calculate adaptive threshold. With a low error
        // rate the threshold is effectively infinite and no voicing
        // decision is overridden.
        if self.error_rate <= 0.005 && self.error_count <= 4 {
        } else {
            let energy = (self.local_energy as f64).powf(0.375);
            let vm = if self.error_rate <= 0.0125 && self.error_count4 == 0 {
                (45.255 * energy) / (277.26 * self.error_rate as f64).exp()
            } else {
                1.414 * energy
            };

            // Voicing decisions only have to be smoothed in the presence
            // of errors.
            for l in 1..=l {
                let amplitude = self.enhanced_spectral_amplitudes[l];

                // Algorithm 113 - apply adaptive threshold to voice/no
                // voice decisions.
                if amplitude > vm as f32 {
                    self.voicing[l] = true;
                }
            }
        };

        // Algorithm 114 - calculate amplitude measure.
        let mut amplitude_measure = 0.0f64;
        for l in 1..=l {
            amplitude_measure += self.enhanced_spectral_amplitudes[l] as f64;
        }

        // Algorithm 115 - calculate amplitude threshold.
        let tm: i32 = if self.error_rate <= 0.005 && self.error_count <= 6 {
            20480
        } else {
            6000 - (300 * self.error_count) + previous_amplitude_threshold
        };

        self.amplitude_threshold = tm;

        // Algorithm 116 - scale enhanced spectral amplitudes if amplitude
        // measure is greater than amplitude threshold.
        if amplitude_measure > tm as f64 {
            let scale = tm as f64 / amplitude_measure;
            for l in 1..=l {
                self.enhanced_spectral_amplitudes[l] *= scale as f32;
            }
        }
    }
}

impl Default for ModelParameters {
    fn default() -> Self {
        Self::new()
    }
}

/// Lehmer sequence noise generator, port of
/// `jmbe.codec.MBENoiseSequenceGenerator`.
#[derive(Clone, Debug)]
pub struct MbeNoiseSequence {
    sample: f32,
    current_buffer: [f32; 256],
}

impl MbeNoiseSequence {
    pub fn new() -> Self {
        Self {
            sample: 3147.0,
            current_buffer: [0.0; 256],
        }
    }

    fn next(&mut self) -> f32 {
        let next = self.sample;
        self.sample = ((171.0 * next) + 11213.0) % 53125.0;
        next
    }

    /// Generates a 256 sample white noise buffer where each successive
    /// buffer overlaps the preceding one by 96 samples.
    pub fn next_buffer(&mut self) -> [f32; 256] {
        let copy = self.current_buffer;

        self.current_buffer.copy_within(160..256, 0);

        for x in 96..256 {
            self.current_buffer[x] = self.next();
        }

        copy
    }
}

impl Default for MbeNoiseSequence {
    fn default() -> Self {
        Self::new()
    }
}

/// White noise for frame muting, standing in for jmbe's
/// `WhiteNoiseGenerator` (which uses `java.util.Random` and is therefore
/// non-deterministic anyway). Lehmer sequence mapped onto -1.0 to 1.0.
pub struct WhiteNoise {
    u: u64,
}

impl WhiteNoise {
    pub fn new() -> Self {
        Self { u: 3147 }
    }

    fn next_unit(&mut self) -> f32 {
        self.u = (171 * self.u + 11213) % 53125;
        self.u as f32
    }

    /// `length` samples scaled by `gain`, matching
    /// `WhiteNoiseGenerator.getSamples(length, gain)`.
    pub fn samples(&mut self, length: usize, gain: f32) -> Vec<f32> {
        (0..length)
            .map(|_| (self.next_unit() / 53125.0 * 2.0 - 1.0) * gain)
            .collect()
    }
}

impl Default for WhiteNoise {
    fn default() -> Self {
        Self::new()
    }
}

/// Base multi-band excitation synthesiser, port of
/// `jmbe.codec.MBESynthesizer`. Holds the cross-frame phase and window
/// state; codec specific synthesizers own one of these plus their previous
/// frame's [`ModelParameters`].
pub struct MbeSynthesizer {
    fft: RealFft256,
    noise_sequence: MbeNoiseSequence,
    white_noise: WhiteNoise,
    previous_phase_o: [f32; 57],
    previous_phase_v: [f32; 57],
    previous_uw: [f32; 256],
}

impl MbeSynthesizer {
    pub fn new() -> Self {
        Self {
            fft: RealFft256::new(),
            noise_sequence: MbeNoiseSequence::new(),
            white_noise: WhiteNoise::new(),
            previous_phase_o: [0.0; 57],
            previous_phase_v: [0.0; 57],
            previous_uw: [0.0; 256],
        }
    }

    /// Calculates the minimum 256-point DFT index for each of the L
    /// frequency bands (algorithm 122).
    pub fn frequency_band_edge_minimums(voice_parameters: &ModelParameters) -> Vec<i32> {
        let mut a = vec![0; voice_parameters.l + 1];

        let multiplier = TWO56_OVER_TWO_PI * voice_parameters.frequency;

        for l in 1..=voice_parameters.l {
            a[l] = ((l as f32 - 0.5) * multiplier).ceil() as i32;
        }

        a
    }

    /// Calculates the maximum 256-point DFT index for each of the L
    /// frequency bands (algorithm 123).
    pub fn frequency_band_edge_maximums(voice_parameters: &ModelParameters) -> Vec<i32> {
        let mut b = vec![0; voice_parameters.l + 1];

        let multiplier = TWO56_OVER_TWO_PI * voice_parameters.frequency;

        for x in 1..=voice_parameters.l {
            b[x] = ((x as f32 + 0.5) * multiplier).ceil() as i32;
        }

        b
    }

    /// Generates 160 samples (20 ms) of voice audio using the model
    /// parameters, scaled to -1.0 to 1.0.
    pub fn get_voice(&mut self, parameters: &ModelParameters, previous: &ModelParameters) -> [f32; SAMPLES_PER_FRAME] {
        let u = self.noise_sequence.next_buffer();

        let unvoiced = self.get_unvoiced(parameters, &u);
        let voiced = self.get_voiced(parameters, previous, &u);

        let mut audio = [0.0f32; SAMPLES_PER_FRAME];

        // Algorithm 142 - combine voiced and unvoiced audio samples to form
        // the completed audio samples. The first frame after reset divides
        // zero noise energy by itself and yields NaN, exactly as jmbe does;
        // jmbe's callers cast NaN to short 0, so map non-finite samples to
        // silence here for the same audible result.
        for x in 0..SAMPLES_PER_FRAME {
            let sample = clip((voiced[x] + unvoiced[x]) * AUDIO_SCALAR_16_BITS_SIGNED);
            audio[x] = if sample.is_finite() { sample } else { 0.0 };
        }

        audio
    }

    /// Generates 160 samples of quiet white noise for frame muting.
    pub fn get_white_noise(&mut self) -> [f32; SAMPLES_PER_FRAME] {
        let samples = self.white_noise.samples(SAMPLES_PER_FRAME, 0.003);
        let mut out = [0.0f32; SAMPLES_PER_FRAME];
        out.copy_from_slice(&samples);
        out
    }

    /// Generates the unvoiced component of the audio signal using a white
    /// noise generator where the frequency components corresponding to the
    /// voiced harmonics are removed (algorithms 118 to 126).
    fn get_unvoiced(
        &mut self,
        parameters: &ModelParameters,
        white_noise_samples: &[f32; 256],
    ) -> [f32; SAMPLES_PER_FRAME] {
        let mut uw = [0.0f32; 256];

        for x in 0..256 {
            uw[x] = white_noise_samples[x] * synthesis_window(x as i32 - 128);
        }

        // Algorithms 122 and 123 - generate the 256 FFT bins to L frequency
        // band mapping from the fundamental frequency.
        let voiced_bands = &parameters.voicing;
        let m = &parameters.enhanced_spectral_amplitudes;
        let a_min = Self::frequency_band_edge_minimums(parameters);
        let b_max = Self::frequency_band_edge_maximums(parameters);

        // Algorithm 118 - perform 256-point DFT against the samples.
        self.fft.forward(&mut uw);

        // Algorithms 120 - determine band-level scaling value for each DFT
        // bin for unvoiced samples and zeroize all voiced and out-of-band
        // bins.
        let mut dft_bin_scalor = [0.0f32; 128];

        for l in 1..=parameters.l {
            if !voiced_bands[l] {
                let mut numerator = 0.0f32;

                for n in a_min[l]..b_max[l] {
                    if n < 128 {
                        numerator += uw[2 * n as usize] * uw[2 * n as usize];
                        numerator += uw[2 * n as usize + 1] * uw[2 * n as usize + 1];
                    }
                }

                let denominator = (b_max[l] - a_min[l]) as f32;

                let scalor = UNVOICED_SCALING_COEFFICIENT * m[l]
                    / (numerator / denominator).sqrt();

                for n in a_min[l]..b_max[l] {
                    if n < 128 {
                        dft_bin_scalor[n as usize] = scalor;
                    }
                }
            }
        }

        // Algorithms 119, 120 and 124 - scale the DFT bins in the a-b
        // min/max bin ranges. The scalor array starts at zero, which also
        // zeroizes the lowest and highest frequency DFT bins per algorithm
        // 124 that were not listed in the a-b ranges.
        for bin in 0..128 {
            uw[2 * bin] *= dft_bin_scalor[bin];
            uw[2 * bin + 1] *= dft_bin_scalor[bin];
        }

        // Algorithm 125 - calculate inverse DFT of the scaled DFT bins to
        // recreate the white noise, notched for voiced bands.
        self.fft.inverse(&mut uw);

        // Algorithm 126 - use the weighted overlap add algorithm to combine
        // the previous Uw and the current inverse DFT results to form the
        // final unvoiced set.
        let mut unvoiced = [0.0f32; SAMPLES_PER_FRAME];

        for n in 0..SAMPLES_PER_FRAME {
            let previous_window = synthesis_window(n as i32);
            let current_window = synthesis_window(n as i32 - SAMPLES_PER_FRAME as i32);

            let previous_uw = if n < 128 { self.previous_uw[n + 128] } else { 0.0 };
            let current_uw = if n >= 32 { uw[n - 32] } else { 0.0 };

            unvoiced[n] = ((previous_window * previous_uw) + (current_window * current_uw))
            / ((previous_window * previous_window)
                + (current_window * current_window));
        }

        self.previous_uw = uw;

        unvoiced
    }

    /// Reconstructs the voiced audio components using the model parameters
    /// from both the current and previous frames (algorithms 127 to 140).
    fn get_voiced(
        &mut self,
        current_frame: &ModelParameters,
        previous_frame: &ModelParameters,
        u: &[f32; 256],
    ) -> [f32; SAMPLES_PER_FRAME] {
        let current_frequency = current_frame.frequency;
        let previous_frequency = previous_frame.frequency;
        let average_frequency = (previous_frequency + current_frequency) / 2.0;
        let phase_offset_per_frame = average_frequency * SAMPLES_PER_FRAME as f32;

        // Algorithm 139 - calculate the current phase angle for each
        // harmonic.
        let mut current_phase_v = [0.0f32; 57];

        for l in 1..=56 {
            // Unwrap the previous phase before updating to avoid overflow.
            self.previous_phase_v[l] %= TWO_PI;
            current_phase_v[l] = self.previous_phase_v[l] + phase_offset_per_frame * l as f32;
        }

        // Short circuit if there are no voiced bands and return zeros.
        if !previous_frame.has_voiced_bands() && !current_frame.has_voiced_bands() {
            self.previous_phase_v = current_phase_v;
            return [0.0; SAMPLES_PER_FRAME];
        }

        let current_l = current_frame.l;
        let previous_l = previous_frame.l;
        let max_l = current_l.max(previous_l);

        let current_voicing = resized(current_frame.voicing.clone(), max_l + 1);
        let previous_voicing = resized(previous_frame.voicing.clone(), max_l + 1);

        // Algorithm 140 partial - number of unvoiced spectral amplitudes in
        // the current frame.
        let unvoiced_band_count = current_frame.unvoiced_band_count();

        let mut current_phase_o = [0.0f32; 57];
        let threshold = (current_l as f32 / 4.0).floor() as usize;

        for l in 1..=56 {
            if l <= threshold {
                current_phase_o[l] = current_phase_v[l];
            } else if l <= max_l {
                let pl = WHITE_NOISE_SCALAR * u[l] - std::f32::consts::PI;
                current_phase_o[l] =
                    current_phase_v[l] + (unvoiced_band_count as f32 * pl) / current_l as f32;
            }
        }

        let current_m = &current_frame.enhanced_spectral_amplitudes;
        let previous_m = &previous_frame.enhanced_spectral_amplitudes;
        let mut voiced = [0.0f32; SAMPLES_PER_FRAME];

        // Algorithm 127 - reconstruct 160 voice samples using each of the l
        // harmonics that are common between this frame and the previous
        // frame.
        let exceeds_threshold =
            (current_frequency - previous_frequency).abs() >= (0.1 * current_frequency);

        for n in 0..SAMPLES_PER_FRAME {
            for l in 1..=max_l {
                let l = l as i32;
                let nf = n as f32;

                if current_voicing[l as usize] && previous_voicing[l as usize] {
                    if l >= 8 || exceeds_threshold {
                        // Algorithm 133
                        let previous_phase = self.previous_phase_o[l as usize]
                            + (previous_frequency * nf * l as f32);
                        voiced[n] += 2.0
                            * (synthesis_window(n as i32) * previous_m[l as usize]
                                * cos64(previous_phase));

                        let current_phase = current_phase_o[l as usize]
                            + (current_frequency * (nf - SAMPLES_PER_FRAME as f32) * l as f32);
                        voiced[n] += 2.0
                            * (synthesis_window(n as i32 - SAMPLES_PER_FRAME as i32)
                                * current_m[l as usize]
                                * cos64(current_phase));
                    } else {
                        // Algorithm 135 - amplitude function: linear
                        // interpolation of the harmonic's amplitude from the
                        // previous frame to the current.
                        let amplitude = previous_m[l as usize]
                            + ((nf / SAMPLES_PER_FRAME as f32)
                                * (current_m[l as usize] - previous_m[l as usize]));

                        // Algorithm 137
                        let ol = current_phase_o[l as usize]
                            - self.previous_phase_o[l as usize]
                            - (phase_offset_per_frame * l as f32);

                        // Algorithm 138
                        let wl = (ol
                            - (TWO_PI * ((ol + std::f32::consts::PI) / TWO_PI).floor()))
                            / SAMPLES_PER_FRAME as f32;

                        // Algorithm 136 - phase function
                        let phase = self.previous_phase_o[l as usize]
                            + ((previous_frequency * l as f32 + wl) * nf)
                            + ((current_frequency - previous_frequency)
                                * ((l * n as i32 * n as i32) as f32 / 320.0));

                        // Algorithm 134
                        voiced[n] += 2.0 * (amplitude * cos64(phase));
                    }
                } else if !current_voicing[l as usize] && previous_voicing[l as usize] {
                    // Algorithm 131
                    voiced[n] += 2.0
                        * (synthesis_window(n as i32)
                            * previous_m[l as usize]
                            * cos64(
                                self.previous_phase_o[l as usize]
                                    + (previous_frequency * nf * l as f32),
                            ));
                } else if current_voicing[l as usize] && !previous_voicing[l as usize] {
                    // Algorithm 132
                    voiced[n] += 2.0
                        * (synthesis_window(n as i32 - SAMPLES_PER_FRAME as i32)
                            * current_m[l as usize]
                            * cos64(
                                current_phase_o[l as usize]
                                    + (current_frequency
                                        * (nf - SAMPLES_PER_FRAME as f32)
                                        * l as f32),
                            ));
                }

                // Algorithm 130 - harmonics that are unvoiced in both the
                // current and previous frames contribute nothing.
            }
        }

        self.previous_phase_v = current_phase_v;
        self.previous_phase_o = current_phase_o;

        voiced
    }
}

impl Default for MbeSynthesizer {
    fn default() -> Self {
        Self::new()
    }
}

/// Java `Math.cos` works in double precision; match that so the output
/// lines up with jmbe's.
fn cos64(x: f32) -> f32 {
    (x as f64).cos() as f32
}

fn clip(value: f32) -> f32 {
    value.clamp(-MAXIMUM_AUDIO_AMPLITUDE, MAXIMUM_AUDIO_AMPLITUDE)
}

/// Resizes the voicing decisions array, padding newly added indices with
/// false, matching `MBESynthesizer.resize`.
fn resized(mut voicing: Vec<bool>, size: usize) -> Vec<bool> {
    if voicing.len() != size {
        voicing.resize(size, false);
    }
    voicing
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn band_edges_bracket_the_fundamental() {
        let mut params = ModelParameters::new();
        params.frequency = 0.25;
        params.l = 2;

        let min = MbeSynthesizer::frequency_band_edge_minimums(&params);
        let max = MbeSynthesizer::frequency_band_edge_maximums(&params);

        // multiplier = 256/(2pi) * 0.25 = 10.185
        let m = (256.0f32 / std::f32::consts::TAU) * 0.25;
        assert_eq!(min[1], ((0.5f32) * m).ceil() as i32);
        assert_eq!(max[1], ((1.5f32) * m).ceil() as i32);
        assert_eq!(min[2], ((1.5f32) * m).ceil() as i32);
        assert_eq!(max[2], ((2.5f32) * m).ceil() as i32);
    }

    #[test]
    fn synthesis_window_is_unit_at_centre_and_zero_at_edges() {
        assert_eq!(synthesis_window(0), 1.0);
        assert_eq!(synthesis_window(104), 0.02);
        assert_eq!(synthesis_window(105), 0.0);
        assert_eq!(synthesis_window(106), 0.0);
        assert_eq!(synthesis_window(-105), 0.0);
        assert_eq!(synthesis_window(-106), 0.0);
    }

    #[test]
    fn noise_sequence_is_deterministic() {
        let mut a = MbeNoiseSequence::new();
        let mut b = MbeNoiseSequence::new();
        for _ in 0..1024 {
            assert_eq!(a.next_buffer(), b.next_buffer());
        }
    }

    #[test]
    fn unvoiced_frame_is_bounded_and_deterministic() {
        // Zero amplitudes hit the 0/0 in the band scaling, same as jmbe, so
        // exercise the unvoiced path with real amplitudes instead.
        let mut synth = MbeSynthesizer::new();
        let mut params = ModelParameters::new();
        params.l = 8;
        params.frequency = 0.05;
        params.voicing = vec![false; 9];
        params.enhanced_spectral_amplitudes = vec![100.0; 9];

        let previous = params.clone();
        let voice = synth.get_voice(&params, &previous);
        assert!(voice.iter().all(|s| s.abs() <= 0.95 + 1e-4));

        let mut synth2 = MbeSynthesizer::new();
        let voice2 = synth2.get_voice(&params, &previous);
        assert_eq!(voice, voice2);
    }
}
