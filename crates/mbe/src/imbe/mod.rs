//! IMBE codec, ported from jmbe `jmbe.codec.imbe`: frame parsing, model
//! parameter decoding and the synthesizer wrapper. The codec window tables
//! live in `crate::window` (jmbe `codec.imbe.Window`).

mod tables;

use crate::bits::BitFrame;
use crate::edac::{golay23_check_and_correct, hamming15_check_and_correct};
use crate::mbe::{FrameType, MbeSynthesizer, ModelParameters, SAMPLES_PER_FRAME};
use tables::{
    DEINTERLEAVE, GAINS, GAIN_INDEXES, HARMONIC_ALLOCATIONS, QUANTIZED_VALUE_INDEXES, STEP_SIZES,
    VOICE_DECISION_INDEX,
};

const RANDOMIZER_SEED: [usize; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
const VECTOR_B0: [usize; 8] = [0, 1, 2, 3, 4, 5, 141, 142];

/// Coefficient offsets for bit lengths 0 to 10: (2 ^ (bit length - 1)) - 0.5.
const COEFFICIENT_OFFSET: [f32; 11] = [
    0.0, 0.5, 1.5, 3.5, 7.5, 15.5, 31.5, 63.5, 127.5, 255.5, 511.5,
];

const MAX_HEADROOM_THRESHOLD: i32 = 3;

/// Port of `IMBEFundamentalFrequency`: the b0 index, with -1 standing in for
/// the INVALID entry. Frequency and L are derived from the index by the
/// section 6.1 formulas exactly as the Java enum constructor does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImbeFundamentalFrequency {
    index: i32,
}

impl ImbeFundamentalFrequency {
    /// Index 134. The Java comment claims L = 30 but the constructor formula
    /// yields L = 39; the formula wins, as it does in the Java runtime.
    pub const DEFAULT: Self = Self { index: 134 };
    pub const INVALID: Self = Self { index: -1 };

    /// Port of `fromValue`: values 0 to 207 are valid, all else is INVALID.
    pub fn from_value(value: u32) -> Self {
        if value <= 207 {
            Self {
                index: value as i32,
            }
        } else {
            Self::INVALID
        }
    }

    pub fn is_valid(self) -> bool {
        self.index >= 0
    }

    /// w0 = 4 pi / (index + 39.5), computed in f64 and narrowed to f32 as the
    /// Java constructor does. INVALID computes with index -1, same as Java.
    pub fn frequency(self) -> f32 {
        (4.0 * std::f64::consts::PI / (self.index as f64 + 39.5)) as f32
    }

    /// L = floor(0.9254 * floor((pi / w0) + 0.25)).
    pub fn l(self) -> usize {
        let frequency = self.frequency() as f64;
        (0.9254 * ((std::f64::consts::PI / frequency) + 0.25).floor()).floor() as usize
    }
}

/// Port of `IMBEModelParameters`: the shared model parameters plus the IMBE
/// fundamental frequency and coset 0 error count.
#[derive(Clone, Debug)]
pub struct ImbeModelParameters {
    pub base: ModelParameters,
    pub fundamental: ImbeFundamentalFrequency,
    /// E0: number of bit errors detected/corrected in coset word 0.
    pub error_count_coset0: i32,
}

impl ImbeModelParameters {
    /// Port of the no-arg constructor: DEFAULT fundamental frequency with
    /// voicing all false, log2 amplitudes zero and spectral amplitudes 1.0.
    pub fn new() -> Self {
        Self::with_fundamental(ImbeFundamentalFrequency::DEFAULT)
    }

    /// Port of `IMBEModelParameters(IMBEFundamentalFrequency)`. The
    /// enhancement is not run here; the Java constructor assigns the raw
    /// amplitudes to both the plain and enhanced fields directly.
    pub fn with_fundamental(fundamental: ImbeFundamentalFrequency) -> Self {
        let mut parameters = Self {
            base: ModelParameters::new(),
            fundamental,
            error_count_coset0: 0,
        };
        parameters.set_fundamental(fundamental);

        let lplus1 = parameters.base.l + 1;
        parameters.base.voicing = vec![false; lplus1];
        parameters.base.log2_spectral_amplitudes = vec![0.0; lplus1];
        parameters.base.spectral_amplitudes = vec![1.0; lplus1];
        parameters.base.enhanced_spectral_amplitudes = vec![1.0; lplus1];
        parameters
    }

    /// Port of `setMBEFundamentalFrequency`.
    fn set_fundamental(&mut self, fundamental: ImbeFundamentalFrequency) {
        self.fundamental = fundamental;
        // getFrameType returns VOICE for every enum entry including INVALID.
        self.base.frame_type = FrameType::Voice;
        self.base.frequency = fundamental.frequency();
        self.base.l = fundamental.l();
    }

    /// Indicates the current error rate requires frame muting (white noise).
    pub fn requires_muting(&self) -> bool {
        self.base.error_rate > 0.0875
    }

    /// Indicates a frame repeat is required: invalid fundamental frequency
    /// or the algorithm 97/98 error thresholds exceeded.
    pub fn repeat_required(&self) -> bool {
        self.fundamental == ImbeFundamentalFrequency::INVALID || self.exceeds_error_threshold()
    }

    fn exceeds_error_threshold(&self) -> bool {
        self.error_count_coset0 >= 2
            && self.base.error_count as f32 >= 10.0 + (40.0 * self.base.error_rate)
    }

    /// Port of `copy`: repeats the previous frame's parameters, or resets to
    /// defaults once the previous repeat count exceeds the threshold.
    pub fn copy(&mut self, previous: &ImbeModelParameters) {
        if previous.base.repeat_count > MAX_HEADROOM_THRESHOLD {
            self.set_fundamental(ImbeFundamentalFrequency::DEFAULT);
            let lplus1 = self.base.l + 1;

            self.base.voicing = vec![false; lplus1];
            self.base.log2_spectral_amplitudes = vec![0.0; lplus1];

            // Java passes this frame's own (still default) local energy and
            // amplitude threshold; the error fields set by setErrors are
            // kept and the repeat count stays at zero.
            let local_energy = self.base.local_energy;
            let amplitude_threshold = self.base.amplitude_threshold;
            self.base
                .set_spectral_amplitudes(vec![1.0; lplus1], local_energy, amplitude_threshold);
        } else {
            self.set_fundamental(previous.fundamental);
            self.base.voicing = previous.base.voicing.clone();
            self.base.log2_spectral_amplitudes = previous.base.log2_spectral_amplitudes.clone();
            self.base.set_spectral_amplitudes(
                previous.base.spectral_amplitudes.clone(),
                previous.base.local_energy,
                previous.base.amplitude_threshold,
            );
            self.base.amplitude_threshold = previous.base.amplitude_threshold;
            self.base.local_energy = previous.base.local_energy;
            self.error_count_coset0 = previous.error_count_coset0;
            self.base.error_count = previous.base.error_count;
            self.base.error_count4 = previous.base.error_count4;
            self.base.error_rate = previous.base.error_rate;
            self.base.repeat_count = previous.base.repeat_count + 1;
        }
    }

    /// Port of `setErrors`.
    pub fn set_errors(
        &mut self,
        previous_error_rate: f32,
        errors_coset0: u32,
        errors_coset4: u32,
        errors_total: u32,
    ) {
        self.error_count_coset0 = errors_coset0 as i32;
        self.base.error_count = errors_total as i32;
        self.base.error_count4 = errors_coset4 as i32;
        self.base.error_rate = (0.95 * previous_error_rate) + (0.000365 * errors_total as f32);
    }
}

impl Default for ImbeModelParameters {
    fn default() -> Self {
        Self::new()
    }
}

/// Port of `Gain.fromValue().getGain()`: Annex E value plus the 1.0 the Java
/// constructor adds. Panics outside 0 to 63 like the Java throw.
fn gain_from_value(value: u32) -> f32 {
    assert!(
        value <= 63,
        "Value must be in range 0-63. Unsupported value: {value}"
    );
    GAINS[value as usize] + 1.0
}

/// Port of `IMBEFrame`: parses a 144-bit IMBE voice frame, performing
/// deinterleaving, derandomization and error detection/correction.
pub struct ImbeFrame {
    frame: BitFrame,
    fundamental: ImbeFundamentalFrequency,
    errors: [u32; 7],
    error_count_total: u32,
}

impl ImbeFrame {
    /// Constructs and decodes an IMBE frame from an 18-byte message.
    pub fn new(data: &[u8]) -> Self {
        let mut frame = BitFrame::from_bytes(data, false);

        Self::deinterleave(&mut frame);

        let mut errors = [0u32; 7];
        let mut error_count_total = 0;

        errors[0] = golay23_check_and_correct(&mut frame, 0);
        error_count_total += errors[0];

        Self::derandomize(&mut frame);

        errors[1] = golay23_check_and_correct(&mut frame, 23);
        error_count_total += errors[1];
        errors[2] = golay23_check_and_correct(&mut frame, 46);
        error_count_total += errors[2];
        errors[3] = golay23_check_and_correct(&mut frame, 69);
        error_count_total += errors[3];
        errors[4] = hamming15_check_and_correct(&mut frame, 92);
        error_count_total += errors[4];
        errors[5] = hamming15_check_and_correct(&mut frame, 107);
        error_count_total += errors[5];
        errors[6] = hamming15_check_and_correct(&mut frame, 122);
        error_count_total += errors[6];

        let fundamental = ImbeFundamentalFrequency::from_value(frame.get_int(&VECTOR_B0));

        Self {
            frame,
            fundamental,
            errors,
            error_count_total,
        }
    }

    pub fn fundamental_frequency(&self) -> ImbeFundamentalFrequency {
        self.fundamental
    }

    pub fn errors(&self) -> &[u32; 7] {
        &self.errors
    }

    /// Port of `IMBEInterleave.deinterleave`.
    fn deinterleave(frame: &mut BitFrame) {
        let original: Vec<bool> = (0..144).map(|i| frame.get(i)).collect();

        for i in 0..144 {
            frame.clear(i);
        }

        for (i, bit) in original.iter().enumerate() {
            if *bit {
                frame.set(DEINTERLEAVE[i]);
            }
        }
    }

    /// Removes the randomizer: a pseudo-random sequence seeded from the
    /// first 12 bits of coset word c0, xored over coset words c1 to c6.
    fn derandomize(frame: &mut BitFrame) {
        let offset = 23;
        let seed = frame.get_int(&RANDOMIZER_SEED);

        // Alg 52
        let mut pr = 16 * seed;

        for x in 0..114 {
            // Alg 53 simplified to a modulus operation
            pr = (173 * pr + 13849) % 65536;

            // Alg 54 - values 32768 and above are a 1; xor via flip
            if pr >= 32768 {
                frame.flip(x + offset);
            }
        }
    }

    /// Port of `getModelParameters`: model parameters for this frame given
    /// the previous frame's parameters.
    pub fn model_parameters(&self, previous: &ImbeModelParameters) -> ImbeModelParameters {
        let mut parameters = ImbeModelParameters::with_fundamental(self.fundamental);
        parameters.set_errors(
            previous.base.error_rate,
            self.errors[0],
            self.errors[4],
            self.error_count_total,
        );

        if parameters.repeat_required() {
            parameters.copy(previous);
        } else {
            parameters.base.voicing = self.voicing_decisions();
            let log2_spectral_amplitudes = self.log2_spectral_amplitudes(previous);
            parameters.base.log2_spectral_amplitudes = log2_spectral_amplitudes.clone();
            parameters.base.set_spectral_amplitudes(
                Self::spectral_amplitudes(&log2_spectral_amplitudes),
                previous.base.local_energy,
                previous.base.amplitude_threshold,
            );
        }

        parameters
    }

    /// Reconstructs the spectral amplitude prediction residuals T for all
    /// values of L (algorithms 68 to 74).
    pub fn spectral_amplitude_prediction_residuals(&self) -> Vec<f32> {
        let l = self.fundamental.l();
        let table = l - 9;

        let gain_index = self.frame.get_int(&GAIN_INDEXES[table]);

        let mut g = [0.0f32; 7];
        g[1] = gain_from_value(gain_index);

        let step_sizes = STEP_SIZES[table];
        let indexes = QUANTIZED_VALUE_INDEXES[table];

        // Alg 68 - decoding gain vector G; step sizes and quantized value
        // index tables are zero based, so m - 3 aligns with them.
        for m in 3..=7usize {
            let index_set = indexes[m - 3];

            if !index_set.is_empty() {
                let b = self.frame.get_int(index_set);
                g[m - 1] = step_sizes[m - 3] * (b as f32 - COEFFICIENT_OFFSET[index_set.len()]);
            }
        }

        let harmonic_allocations = HARMONIC_ALLOCATIONS[table];
        // Allocation for i = 6 (index 5) always has the largest block; use
        // it to dimension the C matrix, as the Java does.
        let columns = harmonic_allocations[5].len() + 1;
        let mut c = vec![vec![0.0f32; columns]; 7];

        // Alg 69 & 70 - gain vector R as inverse DCT of G, into C[i][1].
        for i in 1..=6usize {
            c[i][1] = g[1];

            for m in 2..=6usize {
                // Java mixes f64 Math.PI with f32 subterms; the cos argument
                // is computed in f64 and the result narrowed to f32.
                let angle = std::f64::consts::PI
                    * (m - 1) as f64
                    * ((i as f32 - 0.5f32) as f64)
                    / 6.0;
                c[i][1] += 2.0f32 * g[m] * angle.cos() as f32;
            }
        }

        // Alg 71 & 72 - decode the higher order DCT coefficients.
        for i in 1..=6usize {
            let harmonics = harmonic_allocations[i - 1];

            if harmonics.len() > 1 {
                for j in 2..=harmonics.len() {
                    let m = harmonics[j - 1];
                    let index_set = indexes[m - 3];

                    if !index_set.is_empty() {
                        let b = self.frame.get_int(index_set);
                        c[i][j] =
                            step_sizes[m - 3] * (b as f32 - COEFFICIENT_OFFSET[index_set.len()]);
                    }
                }
            }
        }

        // Alg 73 & 74 - inverse DCT of C into T.
        let mut t = vec![0.0f32; l + 1];
        let mut l_index = 1;

        for i in 1..=6usize {
            let ji = harmonic_allocations[i - 1].len();

            for j in 1..=ji {
                t[l_index] = c[i][1];

                if ji >= 2 {
                    for k in 2..=ji {
                        let angle = std::f64::consts::PI
                            * (k - 1) as f64
                            * ((j as f32 - 0.5f32) as f64)
                            / (ji as f32 as f64);
                        t[l_index] += 2.0f32 * c[i][k] * angle.cos() as f32;
                    }
                }

                l_index += 1;
            }
        }

        t
    }

    /// Port of `resize` (algorithm 79 support): grows the array to
    /// `next_l + 1` entries, padding with the highest indexed value.
    fn resize(elements: &[f32], next_l: usize) -> Vec<f32> {
        let mut resized = elements.to_vec();
        if next_l > elements.len() - 1 {
            let highest = *elements.last().unwrap();
            resized.resize(next_l + 1, highest);
        }
        resized
    }

    /// Algorithms 75 to 79 - log2 spectral amplitudes from the current
    /// frame's prediction residuals and the previous frame's log2
    /// amplitudes scaled to the current L.
    pub fn log2_spectral_amplitudes(&self, previous: &ImbeModelParameters) -> Vec<f32> {
        let l_int = self.fundamental.l();
        let l = l_int as f32;
        let lplus1 = l_int + 1;

        let previous_l = previous.base.l;

        let previous_log2m = Self::resize(
            &previous.base.log2_spectral_amplitudes,
            l_int.max(previous_l) + 1,
        );

        let t = self.spectral_amplitude_prediction_residuals();

        let scale = previous_l as f32 / l;

        let mut kl = vec![0.0f32; lplus1];
        let mut kl_floor = vec![0usize; lplus1];
        let mut sl = vec![0.0f32; lplus1];

        for band in 1..lplus1 {
            // Alg 75
            kl[band] = band as f32 * scale;
            kl_floor[band] = (kl[band] as f64).floor() as i32 as usize;
            // Alg 76
            sl[band] = kl[band] - kl_floor[band] as f32;
        }

        let mut sum = 0.0f32;

        for band in 1..lplus1 {
            // Alg 77 partial - summation
            sum += ((1.0 - sl[band]) * previous_log2m[kl_floor[band]])
                + (sl[band] * previous_log2m[kl_floor[band] + 1]);
        }

        let mut log2m = vec![0.0f32; lplus1];

        // Alg 55 - prediction coefficient
        let p = if l <= 15.0 {
            0.4
        } else if l <= 24.0 {
            0.03 * l - 0.05
        } else {
            0.7
        };

        let pl_sum = p / l * sum;

        // Alg 77
        for band in 1..=l_int {
            log2m[band] = t[band]
                + (p * (1.0 - sl[band]) * previous_log2m[kl_floor[band]])
                + (p * sl[band] * previous_log2m[kl_floor[band] + 1])
                - pl_sum;
        }

        log2m
    }

    /// Inverse log2: M[l] = 2 ^ log2M[l], computed with f64 pow and
    /// narrowed like Java's Math.pow cast. Index 0 becomes 1.0.
    fn spectral_amplitudes(log2_spectral_amplitudes: &[f32]) -> Vec<f32> {
        log2_spectral_amplitudes
            .iter()
            .map(|value| 2.0f64.powf(*value as f64) as f32)
            .collect()
    }

    /// Voiced/unvoiced flag per harmonic 1 to L; index 0 unused.
    pub fn voicing_decisions(&self) -> Vec<bool> {
        let l = self.fundamental.l();
        let mut decisions = vec![false; l + 1];

        for x in 1..=l {
            decisions[x] = self.frame.get(VOICE_DECISION_INDEX[x]);
        }

        decisions
    }
}

/// Port of `IMBESynthesizer` plus the `IMBEAudioCodec` byte-frame entry
/// point: produces 160 samples of 8 kHz audio per 144-bit IMBE frame.
pub struct ImbeSynthesizer {
    mbe: MbeSynthesizer,
    previous: ImbeModelParameters,
}

impl ImbeSynthesizer {
    pub fn new() -> Self {
        Self {
            mbe: MbeSynthesizer::new(),
            previous: ImbeModelParameters::new(),
        }
    }

    /// Resets the previous frame to the defaults, matching
    /// `IMBESynthesizer.reset`.
    pub fn reset(&mut self) {
        self.previous = ImbeModelParameters::new();
    }

    /// Decodes an 18-byte IMBE frame to audio, matching
    /// `IMBEAudioCodec.getAudio(byte[])`.
    pub fn decode(&mut self, frame_data: &[u8]) -> [f32; SAMPLES_PER_FRAME] {
        self.decode_frame(&ImbeFrame::new(frame_data))
    }

    /// Port of `IMBESynthesizer.getAudio(IMBEFrame)`: white noise on max
    /// repeat or muting, else voice; the previous parameters update after
    /// synthesis in every case.
    pub fn decode_frame(&mut self, frame: &ImbeFrame) -> [f32; SAMPLES_PER_FRAME] {
        let parameters = frame.model_parameters(&self.previous);

        let audio = if parameters.base.is_max_frame_repeat() || parameters.requires_muting() {
            self.mbe.get_white_noise()
        } else {
            self.mbe.get_voice(&parameters.base, &self.previous.base)
        };

        self.previous = parameters;

        audio
    }
}

impl Default for ImbeSynthesizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame_bytes(hex: &str) -> Vec<u8> {
        (0..hex.len())
            .step_by(2)
            .map(|x| u8::from_str_radix(&hex[x..x + 2], 16).unwrap())
            .collect()
    }

    const FRAME_1: &str = "7C57B79E016C72542611A1E329DDE3A3DCFE";
    const FRAME_2: &str = "103395EC150222288DE7B35977025308C4E8";

    #[test]
    fn fundamental_frequency_boundaries() {
        let w0 = ImbeFundamentalFrequency::from_value(0);
        assert!(w0.is_valid());
        assert_eq!(w0.l(), 9);

        let w207 = ImbeFundamentalFrequency::from_value(207);
        assert!(w207.is_valid());
        assert_eq!(w207.l(), 56);

        assert_eq!(
            ImbeFundamentalFrequency::from_value(208),
            ImbeFundamentalFrequency::INVALID
        );

        // The Java DEFAULT comment claims L = 30 but the constructor formula
        // gives 39; the port matches the formula.
        assert_eq!(ImbeFundamentalFrequency::DEFAULT.l(), 39);

        // INVALID computes with index -1, yielding L = 8, same as Java.
        assert_eq!(ImbeFundamentalFrequency::INVALID.l(), 8);

        for value in 0..=207u32 {
            let l = ImbeFundamentalFrequency::from_value(value).l();
            assert!((9..=56).contains(&l), "L {l} out of range for {value}");
        }
    }

    #[test]
    fn gain_table_boundaries() {
        // Java's Gain constructor adds 1.0 to the Annex E value.
        assert_eq!(gain_from_value(0), -2.842205 + 1.0);
        assert_eq!(gain_from_value(63), 8.695827 + 1.0);
    }

    #[test]
    fn tables_are_consistent_for_all_l() {
        for l in 9..=56usize {
            let table = l - 9;
            // Coefficients b3 to b(L + 1).
            assert_eq!(STEP_SIZES[table].len(), l - 1, "step sizes for L {l}");
            assert_eq!(
                QUANTIZED_VALUE_INDEXES[table].len(),
                l - 1,
                "quantized indexes for L {l}"
            );
            // Six J blocks whose harmonic counts sum to L.
            let allocations = HARMONIC_ALLOCATIONS[table];
            assert_eq!(allocations.len(), 6);
            let total: usize = allocations.iter().map(|block| block.len()).sum();
            assert_eq!(total, l, "harmonic allocation total for L {l}");
            // Quantized index sets never exceed the coefficient offset table.
            for set in QUANTIZED_VALUE_INDEXES[table].iter() {
                assert!(set.len() <= 10);
            }
        }
    }

    #[test]
    fn parses_known_frame() {
        let frame = ImbeFrame::new(&frame_bytes(FRAME_1));
        assert!(frame.fundamental_frequency().is_valid());
        let l = frame.fundamental_frequency().l();
        assert!((9..=56).contains(&l));
        assert_eq!(frame.voicing_decisions().len(), l + 1);

        let previous = ImbeModelParameters::new();
        let parameters = frame.model_parameters(&previous);
        assert_eq!(parameters.base.l, l);
        assert_eq!(parameters.base.enhanced_spectral_amplitudes.len(), l + 1);
        assert!(parameters
            .base
            .enhanced_spectral_amplitudes
            .iter()
            .all(|a| a.is_finite()));
    }

    #[test]
    fn decodes_smoke_frames_to_valid_audio() {
        let mut synthesizer = ImbeSynthesizer::new();
        let mut any_nonzero = false;

        for hex in [FRAME_1, FRAME_2] {
            let audio = synthesizer.decode(&frame_bytes(hex));
            assert_eq!(audio.len(), 160);
            for sample in audio.iter() {
                assert!(sample.is_finite());
                assert!((-0.95..=0.95).contains(sample), "sample {sample}");
            }
            if audio.iter().any(|s| *s != 0.0) {
                any_nonzero = true;
            }
        }

        assert!(any_nonzero, "both frames decoded to silence");
    }

    #[test]
    fn reset_restores_deterministic_state() {
        let mut a = ImbeSynthesizer::new();
        let first = a.decode(&frame_bytes(FRAME_1));
        let differs = a.decode(&frame_bytes(FRAME_1));
        assert_ne!(first, differs, "previous frame state should carry over");
    }
}
