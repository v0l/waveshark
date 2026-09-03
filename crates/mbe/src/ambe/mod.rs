//! AMBE 3600x2450 codec, ported from jmbe `jmbe.codec.ambe`: frame parsing,
//! model parameter decoding, tone decoding and the synthesizer wrapper.

mod tables;

use crate::bits::BitFrame;
use crate::edac::{golay23_check_and_correct, golay24_check_and_correct};
use crate::mbe::oscillator::Oscillator;
use crate::mbe::{FrameType, MbeSynthesizer, ModelParameters, SAMPLES_PER_FRAME};
use tables::{
    DIFFERENTIAL_GAINS, FUNDAMENTAL_FREQUENCIES, HOCB5, HOCB6, HOCB7, HOCB8, LMPR_BLOCK_LENGTHS,
    PRBA24, PRBA58, TONES, VOICING_DECISIONS,
};

// Interleave maps from the 72-bit frame into the coset vectors.
const VECTOR_C0: [usize; 24] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 1, 5, 9, 13, 17, 21,
];
const VECTOR_C1: [usize; 23] = [
    25, 29, 33, 37, 41, 45, 49, 53, 57, 61, 65, 69, 2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42,
];
const VECTOR_C2: [usize; 11] = [46, 50, 54, 58, 62, 66, 70, 3, 7, 11, 15];
const VECTOR_C3: [usize; 14] = [19, 23, 27, 31, 35, 39, 43, 47, 51, 55, 59, 63, 67, 71];
const VECTOR_U0: [usize; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];
const VECTOR_U0_TONE_CHECK: [usize; 6] = [0, 1, 2, 3, 4, 5];
const VECTOR_U3_TONE_CHECK: [usize; 4] = [10, 11, 12, 13];
const VECTOR_U0_B0_HIGH: [usize; 4] = [0, 1, 2, 3];
const VECTOR_U0_B1_HIGH: [usize; 4] = [4, 5, 6, 7];
const VECTOR_U0_B2_HIGH: [usize; 4] = [8, 9, 10, 11];
const VECTOR_U1_B3_HIGH: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
const VECTOR_U1_HIGH_TONE_VERIFY: [usize; 4] = [0, 1, 2, 3];
const VECTOR_U1_B4_HIGH: [usize; 4] = [8, 9, 10, 11];
const VECTOR_U1_LOW_TONE_VERIFY: [usize; 4] = [8, 9, 10, 11];
const VECTOR_U2_B5_HIGH: [usize; 4] = [0, 1, 2, 3];
const VECTOR_U2_B6_HIGH: [usize; 3] = [4, 5, 6];
const VECTOR_U2_B7_HIGH: [usize; 3] = [7, 8, 9];
const VECTOR_U2_B8_HIGH: [usize; 1] = [10];
const VECTOR_U3_B1_LOW: [usize; 1] = [0];
const VECTOR_U3_B2_LOW: [usize; 1] = [1];
const VECTOR_U3_B0_LOW: [usize; 3] = [2, 3, 4];
const VECTOR_U3_B3_LOW: [usize; 1] = [5];
const VECTOR_U3_B4_LOW: [usize; 3] = [6, 7, 8];
const VECTOR_U3_B5_LOW: [usize; 1] = [9];
const VECTOR_U3_B6_LOW: [usize; 1] = [10];
const VECTOR_U3_B7_LOW: [usize; 1] = [11];
const VECTOR_U3_B8_LOW: [usize; 2] = [12, 13];
const VECTOR_U0_AD_HIGH: [usize; 6] = [6, 7, 8, 9, 10, 11];
const VECTOR_U3_AD_LOW: [usize; 1] = [8];
const VECTOR_U1_ID: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
const U0_TONE_FRAME_VALUE: u32 = 63;
const U3_TONE_FRAME_VALUE: u32 = 0;

/// Port of `AMBEFundamentalFrequency`: an index into the W0 to W127 table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FundamentalFrequency(pub usize);

pub const W120: FundamentalFrequency = FundamentalFrequency(120);
pub const W124: FundamentalFrequency = FundamentalFrequency(124);

impl FundamentalFrequency {
    /// Port of `fromValue`; panics outside 0 to 127 like the Java throw.
    pub fn from_value(value: u32) -> Self {
        assert!(
            value <= 127,
            "Fundamental frequency value must be in the range 0 - 127. Unrecognized: {value}"
        );
        Self(value as usize)
    }

    /// Fundamental frequency in radians per sample, narrowed to f32 after the
    /// f64 two pi scaling exactly as the Java constructor does.
    pub fn frequency(self) -> f32 {
        (FUNDAMENTAL_FREQUENCIES[self.0].0 * 2.0 * std::f64::consts::PI) as f32
    }

    pub fn l(self) -> usize {
        FUNDAMENTAL_FREQUENCIES[self.0].1
    }

    pub fn frame_type(self) -> FrameType {
        FUNDAMENTAL_FREQUENCIES[self.0].2
    }
}

/// Tone classification matching the metadata branches of
/// `AMBEAudioCodec.getAudioWithMetadata`, checked in that order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToneType {
    CallProgress,
    Discrete,
    Dtmf,
    Knox,
    Invalid,
}

/// Port of the `Tone` enumeration entry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tone {
    pub value: i32,
    pub label: &'static str,
    pub frequency1: f64,
    pub frequency2: f64,
}

pub const INVALID_TONE: Tone = Tone {
    value: -1,
    label: "INVALID",
    frequency1: 0.0,
    frequency2: 0.0,
};

impl Tone {
    /// Port of `Tone.fromValue`: unknown values map to the INVALID tone.
    pub fn from_value(value: u32) -> Tone {
        for &(tone_value, label, frequency1, frequency2) in TONES.iter() {
            if tone_value == value as i32 {
                return Tone {
                    value: tone_value,
                    label,
                    frequency1,
                    frequency2,
                };
            }
        }
        INVALID_TONE
    }

    pub fn is_valid(&self) -> bool {
        self.value != INVALID_TONE.value
    }

    pub fn has_frequency2(&self) -> bool {
        self.frequency2 > 0.0
    }

    pub fn tone_type(&self) -> ToneType {
        match self.value {
            160..=163 => ToneType::CallProgress,
            5..=122 => ToneType::Discrete,
            128..=143 => ToneType::Dtmf,
            144..=159 => ToneType::Knox,
            _ => ToneType::Invalid,
        }
    }
}

/// Port of `ToneParameters`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToneParameters {
    pub tone: Tone,
    pub amplitude: u32,
}

impl ToneParameters {
    pub fn is_valid_tone(&self) -> bool {
        self.tone.is_valid()
    }
}

/// Port of `ToneGenerator`: 160 samples per frame at 8 kHz from one or two
/// oscillators.
pub struct ToneGenerator {
    oscillator1: Oscillator,
    oscillator2: Oscillator,
}

const TWO_CHANNEL_GAIN_REDUCTION: f32 = 0.5;

impl ToneGenerator {
    pub fn new() -> Self {
        Self {
            oscillator1: Oscillator::new(0.0, 8000.0),
            oscillator2: Oscillator::new(0.0, 8000.0),
        }
    }

    /// Port of `generate`; panics on an invalid tone like the Java throw.
    pub fn generate(&mut self, tone_parameters: &ToneParameters) -> [f32; SAMPLES_PER_FRAME] {
        assert!(
            tone_parameters.is_valid_tone(),
            "Cannot generate tone audio - INVALID tone"
        );

        let tone = tone_parameters.tone;
        let mut gain = tone_parameters.amplitude as f32 / 127.0;
        let mut audio = [0.0f32; SAMPLES_PER_FRAME];

        if tone.has_frequency2() {
            gain *= TWO_CHANNEL_GAIN_REDUCTION;

            self.oscillator1.set_frequency(tone.frequency1);
            self.oscillator2.set_frequency(tone.frequency2);

            let samples = self.oscillator1.generate(SAMPLES_PER_FRAME, gain);
            let samples2 = self.oscillator2.generate(SAMPLES_PER_FRAME, gain);

            for x in 0..SAMPLES_PER_FRAME {
                audio[x] = samples[x] + samples2[x];
            }
        } else {
            self.oscillator1.set_frequency(tone.frequency1);
            audio.copy_from_slice(&self.oscillator1.generate(SAMPLES_PER_FRAME, gain));
        }

        audio
    }
}

impl Default for ToneGenerator {
    fn default() -> Self {
        Self::new()
    }
}

/// Port of `AMBEModelParameters`: the shared model parameters plus the AMBE
/// fundamental frequency entry and differential gain state.
#[derive(Clone, Debug)]
pub struct AmbeModelParameters {
    pub base: ModelParameters,
    pub fundamental: FundamentalFrequency,
    pub gain: f32,
}

impl AmbeModelParameters {
    /// Port of the no-arg constructor: W124 silence fundamental but frame
    /// type VOICE, unity spectral amplitudes, zero gain.
    pub fn new() -> Self {
        let mut parameters = Self {
            base: ModelParameters::new(),
            fundamental: W124,
            gain: 0.0,
        };
        parameters.set_fundamental(W124);
        parameters.set_defaults(FrameType::Voice);
        parameters
    }

    /// Port of the frame constructor for VOICE, SILENCE and ERASURE frames.
    pub fn from_frame(
        fundamental: FundamentalFrequency,
        b: &[u32; 9],
        errors: &[u32; 2],
        previous: &AmbeModelParameters,
    ) -> Self {
        let mut parameters = Self {
            base: ModelParameters::new(),
            fundamental,
            gain: 0.0,
        };
        parameters.set_fundamental(fundamental);

        // Alg 55 & 56
        parameters.base.error_count = (errors[0] + errors[1]) as i32;
        parameters.base.error_rate =
            (0.95 * previous.base.error_rate) + (0.001064 * parameters.base.error_count as f32);

        // Alg 57 & 58 determine if this should be a frame repeat due to
        // excessive errors or an ERASURE frame type.
        if fundamental.frame_type() == FrameType::Erasure {
            parameters.set_defaults(FrameType::Erasure);
        } else if errors[0] >= 4 || (errors[0] >= 2 && parameters.base.error_count >= 6) {
            // Alg 59 to 64
            parameters.base.repeat_count = previous.base.repeat_count + 1;
            parameters.set_fundamental(previous.fundamental);
            parameters.gain = previous.gain;
            parameters.base.voicing = previous.base.voicing.clone();
            parameters.base.log2_spectral_amplitudes =
                previous.base.log2_spectral_amplitudes.clone();
            parameters.base.set_spectral_amplitudes(
                previous.base.spectral_amplitudes.clone(),
                previous.base.local_energy,
                previous.base.amplitude_threshold,
            );
            parameters.base.local_energy = previous.base.local_energy;
        } else {
            if fundamental.frame_type() == FrameType::Voice {
                parameters.set_voicing_decisions(b[1]);
            } else {
                // Silence frame
                parameters.base.voicing = vec![false; parameters.base.l + 1];
            }

            parameters.set_gain(b[2], previous);
            parameters.decode_prba_vector(b[3], b[4], b[5], b[6], b[7], b[8], previous);
        }

        parameters
    }

    /// Port of `setMBEFundamentalFrequency`.
    fn set_fundamental(&mut self, fundamental: FundamentalFrequency) {
        self.fundamental = fundamental;
        self.base.frame_type = fundamental.frame_type();
        self.base.frequency = fundamental.frequency();
        self.base.l = fundamental.l();
    }

    /// Port of `setDefaults`.
    fn set_defaults(&mut self, frame_type: FrameType) {
        self.base.frame_type = frame_type;

        let size = self.base.l + 1;
        self.base.voicing = vec![false; size];
        self.base.log2_spectral_amplitudes = vec![0.0; size];
        self.base.spectral_amplitudes = vec![1.0; size];
        // Java aliases the enhanced array to the spectral array here.
        self.base.enhanced_spectral_amplitudes = self.base.spectral_amplitudes.clone();

        self.gain = 0.0;
    }

    pub fn is_erasure_frame(&self) -> bool {
        self.base.frame_type == FrameType::Erasure
    }

    /// Port of `isFrameMuted` (unused by the synthesizer, kept for parity).
    pub fn is_frame_muted(&self) -> bool {
        self.base.error_rate > 0.096 || self.base.repeat_count >= 4
    }

    /// Port of `setVoicingDecisions(int b1)`.
    fn set_voicing_decisions(&mut self, b1: u32) {
        let decisions = &VOICING_DECISIONS[b1 as usize];

        let l = self.base.l;
        let mut voicing = vec![false; l + 1];

        for band in 1..=l {
            let voice_index =
                (band as f32 * self.base.frequency * 16.0 / std::f32::consts::TAU) as usize;
            voicing[band] = decisions[voice_index];
        }

        self.base.voicing = voicing;
    }

    /// Port of `setGain`.
    fn set_gain(&mut self, b2: u32, previous: &AmbeModelParameters) {
        let (gain, adjustment) = DIFFERENTIAL_GAINS[b2 as usize];

        // Alg 26
        self.gain = (gain + adjustment) + (0.5 * previous.gain);
    }

    /// Port of `decodePRBAVector` (algorithms 27 to 46).
    #[allow(clippy::too_many_arguments)]
    fn decode_prba_vector(
        &mut self,
        b3: u32,
        b4: u32,
        b5: u32,
        b6: u32,
        b7: u32,
        b8: u32,
        previous: &AmbeModelParameters,
    ) {
        let l = self.base.l;
        // Java's (float)Math.PI used inside the cosine arguments.
        let pi = std::f32::consts::PI;
        let one_over_two_sqr_two = 1.0f32 / (2.0f32 * ((2.0f32 as f64).sqrt() as f32));

        let mut g = [0.0f32; 9];
        let (g2, g3, g4) = PRBA24[b3 as usize];
        g[2] = g2;
        g[3] = g3;
        g[4] = g4;
        let (g5, g6, g7, g8) = PRBA58[b4 as usize];
        g[5] = g5;
        g[6] = g6;
        g[7] = g7;
        g[8] = g8;

        // Alg 27 & 28: inverse DCT of G. Java multiplies by the double
        // literal 2.0, so each += accumulates in f64 and narrows to f32.
        let mut r = [0.0f32; 9];
        for i in 1..=8usize {
            r[i] = g[1];

            for m in 2..=8usize {
                let arg = (pi * (m - 1) as f32 * (i as f32 - 0.5)) / 8.0;
                let cos = (arg as f64).cos() as f32;
                r[i] = ((r[i] as f64) + (2.0 * g[m] as f64 * cos as f64)) as f32;
            }
        }

        let mut c = [[0.0f32; 18]; 5];

        // Alg 29, 31, 33, 35
        c[1][1] = 0.5 * (r[1] + r[2]);
        c[2][1] = 0.5 * (r[3] + r[4]);
        c[3][1] = 0.5 * (r[5] + r[6]);
        c[4][1] = 0.5 * (r[7] + r[8]);

        // Alg 30, 32, 34, 36
        c[1][2] = one_over_two_sqr_two * (r[1] - r[2]);
        c[2][2] = one_over_two_sqr_two * (r[3] - r[4]);
        c[3][2] = one_over_two_sqr_two * (r[5] - r[6]);
        c[4][2] = one_over_two_sqr_two * (r[7] - r[8]);

        let j = LMPR_BLOCK_LENGTHS[l];

        // Alg 37: the Java switch assigns at most four higher order
        // coefficients even when the block length exceeds six.
        for i in 1..=4usize {
            if j[i] > 2 {
                let coefficients: &[f32; 4] = match i {
                    1 => &HOCB5[b5 as usize],
                    2 => &HOCB6[b6 as usize],
                    3 => &HOCB7[b7 as usize],
                    _ => &HOCB8[b8 as usize],
                };

                let count = (j[i] - 2).min(4);
                c[i][3..3 + count].copy_from_slice(&coefficients[..count]);
            }
        }

        // Alg 38 & 39: inverse DCT of C to produce c(i,k) rearranged as T.
        let mut t = vec![0.0f32; l + 1];
        let mut l_pointer = 1usize;

        for i in 1..=4usize {
            for jj in 1..=j[i] {
                let mut acc = c[i][1];

                for k in 2..=j[i] {
                    let arg = (pi * (k - 1) as f32 * (jj as f32 - 0.5)) / j[i] as f32;
                    acc += 2.0f32 * c[i][k] * ((arg as f64).cos() as f32);
                }

                t[l_pointer] = acc;
                l_pointer += 1;
            }
        }

        let previous_l = previous.base.l;

        // Alg 40 & 41
        let kappa = previous_l as f32 / l as f32;

        let mut k = vec![0.0f32; l + 1];
        let mut k_floor = vec![0usize; l + 1];
        let mut s = vec![0.0f32; l + 1];

        // Alg 44: Java writes previousA[0] = previousA[1] into the previous
        // frame's array; that frame is discarded after this decode, so a
        // local copy is equivalent.
        let mut previous_a = previous.base.log2_spectral_amplitudes.clone();
        previous_a[0] = previous_a[1];

        for band in 1..=l {
            k[band] = kappa * band as f32;
            k_floor[band] = ((k[band] as f64).floor()) as i32 as usize;
            s[band] = k[band] - k_floor[band] as f32;
        }

        // Alg 42 & 43: pre-compute sums
        let mut summation43 = 0.0f32;
        let mut lambda_sum = 0.0f32;

        for band in 1..=l {
            let akl_previous = if k_floor[band] <= previous_l {
                previous_a[k_floor[band]]
            } else {
                previous_a[previous_l]
            };

            let plus1 = if band < l { band + 1 } else { l };
            let akl_plus1_previous = if k_floor[plus1] <= previous_l {
                previous_a[k_floor[plus1]]
            } else {
                previous_a[previous_l]
            };

            summation43 += ((1.0 - s[band]) * akl_previous) + (s[band] * akl_plus1_previous);
            lambda_sum += t[band];
        }

        lambda_sum /= l as f32;

        // Alg 42: the log division runs in f64 then narrows, as in Java.
        let gain = self.gain - (0.5 * (((l as f64).ln() / (2.0f64).ln()) as f32)) - lambda_sum;

        let mut log_spectral_amplitudes = vec![0.0f32; l + 1];
        // Java seeds index 0 with 1.0 even though the slot is unused.
        log_spectral_amplitudes[0] = 1.0;

        let mut spectral_amplitudes = vec![0.0f32; l + 1];

        let unvoiced_coefficient = 0.2046f32 / ((self.base.frequency as f64).sqrt() as f32);

        summation43 *= 0.65 / l as f32;

        for band in 1..=l {
            // Alg 44 & 45
            let akl_previous = if k_floor[band] == 0 {
                previous_a[1]
            } else if k_floor[band] <= previous_l {
                previous_a[k_floor[band]]
            } else {
                previous_a[previous_l]
            };

            let l_plus1 = if band < l { band + 1 } else { l };
            let akl_plus1_previous = if k_floor[l_plus1] <= previous_l {
                previous_a[k_floor[l_plus1]]
            } else {
                previous_a[previous_l]
            };

            // Alg 43
            log_spectral_amplitudes[band] = t[band]
                + (0.65 * (1.0 - s[band]) * akl_previous)
                + (0.65 * s[band] * akl_plus1_previous)
                - summation43
                + gain;

            // Alg 46: spectral magnitude depends on the band's voicing
            // decision.
            let magnitude = ((0.693f32 * log_spectral_amplitudes[band]) as f64).exp() as f32;
            spectral_amplitudes[band] = if self.base.voicing[band] {
                magnitude
            } else {
                unvoiced_coefficient * magnitude
            };
        }

        self.base.log2_spectral_amplitudes = log_spectral_amplitudes;
        self.base.set_spectral_amplitudes(
            spectral_amplitudes,
            previous.base.local_energy,
            previous.base.amplitude_threshold,
        );
    }
}

impl Default for AmbeModelParameters {
    fn default() -> Self {
        Self::new()
    }
}

/// Port of `AMBEFrame`: a 72-bit AMBE 3600x2450 voice or tone frame.
#[derive(Clone, Debug)]
pub struct AmbeFrame {
    frame_type: FrameType,
    fundamental: FundamentalFrequency,
    errors: [u32; 2],
    b: [u32; 9],
    tone: Option<Tone>,
    tone_amplitude: u32,
}

impl AmbeFrame {
    /// Decodes a 9-byte (72-bit) AMBE frame.
    pub fn new(frame_data: &[u8]) -> Self {
        let frame = BitFrame::from_bytes(frame_data, false);

        let mut c0 = extract_vector(&frame, &VECTOR_C0);
        let mut c1 = extract_vector(&frame, &VECTOR_C1);
        let c2 = extract_vector(&frame, &VECTOR_C2);
        let c3 = extract_vector(&frame, &VECTOR_C3);

        // Error check C0, then descramble and error check C1.
        let mut errors = [0u32; 2];
        errors[0] = golay24_check_and_correct(&mut c0, 0);
        c1.xor(0, 23, modulation_vector(c0.get_int(&VECTOR_U0)));
        errors[1] = golay23_check_and_correct(&mut c1, 0);

        let b0 = (c0.get_int(&VECTOR_U0_B0_HIGH) << 3) + c3.get_int(&VECTOR_U3_B0_LOW);
        let error_count = errors[0] + errors[1];

        let mut fundamental = FundamentalFrequency::from_value(b0);
        let mut b = [0u32; 9];
        let frame_type;

        // Process as either a tone frame or a voice frame.
        if error_count < 6
            && c0.get_int(&VECTOR_U0_TONE_CHECK) == U0_TONE_FRAME_VALUE
            && (c3.get_int(&VECTOR_U3_TONE_CHECK) == U3_TONE_FRAME_VALUE
                || c1.get_int(&VECTOR_U1_HIGH_TONE_VERIFY)
                    == c1.get_int(&VECTOR_U1_LOW_TONE_VERIFY))
        {
            frame_type = FrameType::Tone;
        } else {
            let mut decoded_type = fundamental.frame_type();

            // A tone-valued fundamental here is a high bit error rate
            // artifact; override to the W120 erasure so a frame repeat
            // sequence happens, same as jmbe.
            if decoded_type == FrameType::Tone {
                fundamental = W120;
                decoded_type = fundamental.frame_type();
            }

            frame_type = decoded_type;

            b[0] = b0;
            b[1] = (c0.get_int(&VECTOR_U0_B1_HIGH) << 1) + c3.get_int(&VECTOR_U3_B1_LOW);
            b[2] = (c0.get_int(&VECTOR_U0_B2_HIGH) << 1) + c3.get_int(&VECTOR_U3_B2_LOW);
            b[3] = (c1.get_int(&VECTOR_U1_B3_HIGH) << 1) + c3.get_int(&VECTOR_U3_B3_LOW);
            b[4] = (c1.get_int(&VECTOR_U1_B4_HIGH) << 3) + c3.get_int(&VECTOR_U3_B4_LOW);
            b[5] = (c2.get_int(&VECTOR_U2_B5_HIGH) << 1) + c3.get_int(&VECTOR_U3_B5_LOW);
            b[6] = (c2.get_int(&VECTOR_U2_B6_HIGH) << 1) + c3.get_int(&VECTOR_U3_B6_LOW);
            b[7] = (c2.get_int(&VECTOR_U2_B7_HIGH) << 1) + c3.get_int(&VECTOR_U3_B7_LOW);
            b[8] = (c2.get_int(&VECTOR_U2_B8_HIGH) << 2) + c3.get_int(&VECTOR_U3_B8_LOW);
        }

        let (tone, tone_amplitude) = if frame_type == FrameType::Tone {
            (
                Some(Tone::from_value(c1.get_int(&VECTOR_U1_ID))),
                (c0.get_int(&VECTOR_U0_AD_HIGH) << 1) + c3.get_int(&VECTOR_U3_AD_LOW),
            )
        } else {
            (None, 0)
        };

        Self {
            frame_type,
            fundamental,
            errors,
            b,
            tone,
            tone_amplitude,
        }
    }

    pub fn frame_type(&self) -> FrameType {
        self.frame_type
    }

    pub fn fundamental_frequency(&self) -> FundamentalFrequency {
        self.fundamental
    }

    /// Error counts for the C0 and C1 blocks.
    pub fn errors(&self) -> [u32; 2] {
        self.errors
    }

    pub fn is_tone_frame(&self) -> bool {
        self.frame_type == FrameType::Tone
    }

    /// The decoded tone for TONE frames, for metadata inspection.
    pub fn tone(&self) -> Option<Tone> {
        self.tone
    }

    /// Port of `getVoiceParameters`; panics for TONE frames like the Java
    /// throw.
    pub fn voice_parameters(&self, previous: &AmbeModelParameters) -> AmbeModelParameters {
        assert!(
            self.frame_type != FrameType::Tone,
            "Frame type TONE does not provide model parameters"
        );
        AmbeModelParameters::from_frame(self.fundamental, &self.b, &self.errors, previous)
    }

    /// Port of `getToneParameters`; panics for non TONE frames like the Java
    /// throw.
    pub fn tone_parameters(&self) -> ToneParameters {
        match self.tone {
            Some(tone) => ToneParameters {
                tone,
                amplitude: self.tone_amplitude,
            },
            None => panic!("Frame type {:?} does not provide tone model parameters", self.frame_type),
        }
    }
}

/// Extracts the listed frame bit positions into a new vector frame.
fn extract_vector(frame: &BitFrame, indexes: &[usize]) -> BitFrame {
    let mut vector = BitFrame::new(indexes.len());

    for (pointer, &bit) in indexes.iter().enumerate() {
        if frame.get(bit) {
            vector.set(pointer);
        }
    }

    vector
}

/// Port of `getModulationVector`: the 23-bit pseudo random sequence xor'd
/// over vector C1, returned MSB first with bit 0 in the top position.
fn modulation_vector(seed: u32) -> u32 {
    // Alg 52
    let mut pr = 16i32 * seed as i32;
    let mut vector = 0u32;

    for x in 0..23 {
        // Alg 53 simplified to a modulus, as in jmbe.
        pr = (173 * pr + 13849) % 65536;

        // Alg 54: values 32768 and above are a one.
        if pr >= 32768 {
            vector |= 1 << (22 - x);
        }
    }

    vector
}

/// Port of `AMBESynthesizer` plus the `AMBEAudioCodec` byte-frame entry
/// point: produces 160 samples of 8 kHz audio per 72-bit AMBE frame.
pub struct AmbeSynthesizer {
    mbe: MbeSynthesizer,
    tone_generator: ToneGenerator,
    previous_frame: AmbeModelParameters,
}

impl AmbeSynthesizer {
    pub fn new() -> Self {
        Self {
            mbe: MbeSynthesizer::new(),
            tone_generator: ToneGenerator::new(),
            previous_frame: AmbeModelParameters::new(),
        }
    }

    /// Resets the previous frame to the defaults, matching
    /// `AMBESynthesizer.reset`.
    pub fn reset(&mut self) {
        self.previous_frame = AmbeModelParameters::new();
    }

    /// Decodes a 9-byte AMBE frame to audio, matching
    /// `AMBEAudioCodec.getAudio(byte[])`.
    pub fn decode(&mut self, frame_data: &[u8]) -> [f32; SAMPLES_PER_FRAME] {
        self.decode_frame(&AmbeFrame::new(frame_data))
    }

    /// Port of `AMBESynthesizer.getAudio(AMBEFrame)`.
    pub fn decode_frame(&mut self, frame: &AmbeFrame) -> [f32; SAMPLES_PER_FRAME] {
        if frame.is_tone_frame() {
            let tone_parameters = frame.tone_parameters();

            if tone_parameters.is_valid_tone() {
                self.tone_generator.generate(&tone_parameters)
            } else {
                // Java sets the repeat count to its current value here (no
                // increment); ported as the same no-op.
                if !self.previous_frame.base.is_max_frame_repeat() {
                    self.mbe
                        .get_voice(&self.previous_frame.base, &self.previous_frame.base)
                } else {
                    // Frame muting procedure
                    self.previous_frame = AmbeModelParameters::new();
                    self.mbe.get_white_noise()
                }
            }
        } else {
            let parameters = frame.voice_parameters(&self.previous_frame);

            if !parameters.base.is_max_frame_repeat() {
                let audio = if parameters.is_erasure_frame() {
                    self.mbe.get_white_noise()
                } else {
                    self.mbe.get_voice(&parameters.base, &self.previous_frame.base)
                };

                self.previous_frame = parameters;
                audio
            } else {
                // Frame muting procedure
                self.previous_frame = AmbeModelParameters::new();
                self.mbe.get_white_noise()
            }
        }
    }
}

impl Default for AmbeSynthesizer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edac::GOLAY_CHECKSUMS;

    /// Golay(23,12) codeword: 12 data bits then the 11 checksum bits.
    fn golay23_codeword(data: u32) -> u32 {
        let mut checksum = 0u32;
        for i in 0..12 {
            if (data >> (11 - i)) & 1 == 1 {
                checksum ^= GOLAY_CHECKSUMS[i];
            }
        }
        (data << 11) | checksum
    }

    /// Golay(24,12) codeword: Golay(23,12) plus an even parity bit.
    fn golay24_codeword(data: u32) -> u32 {
        let codeword = golay23_codeword(data);
        (codeword << 1) | (codeword.count_ones() % 2)
    }

    fn place(bits: &mut [bool; 72], indexes: &[usize], value: u32) {
        let width = indexes.len();
        for (i, &index) in indexes.iter().enumerate() {
            bits[index] = (value >> (width - 1 - i)) & 1 == 1;
        }
    }

    /// Packs the coset vectors through the interleave maps into frame bytes.
    fn build_frame(c0: u32, c1: u32, c2: u32, c3: u32) -> [u8; 9] {
        let mut bits = [false; 72];
        place(&mut bits, &VECTOR_C0, c0);
        place(&mut bits, &VECTOR_C1, c1);
        place(&mut bits, &VECTOR_C2, c2);
        place(&mut bits, &VECTOR_C3, c3);

        let mut bytes = [0u8; 9];
        for (i, bit) in bits.iter().enumerate() {
            if *bit {
                bytes[i / 8] |= 1 << (7 - (i % 8));
            }
        }
        bytes
    }

    fn assert_valid_audio(audio: &[f32; SAMPLES_PER_FRAME]) {
        assert!(audio.iter().all(|s| s.is_finite()));
        assert!(audio.iter().all(|s| s.abs() <= 0.95));
    }

    #[test]
    fn fundamental_frequency_table_boundaries() {
        let w0 = FundamentalFrequency::from_value(0);
        assert_eq!(w0.l(), 9);
        assert_eq!(w0.frame_type(), FrameType::Voice);
        let expected = (0.049971f64 * 2.0 * std::f64::consts::PI) as f32;
        assert_eq!(w0.frequency(), expected);

        assert_eq!(FundamentalFrequency::from_value(119).l(), 56);
        assert_eq!(
            FundamentalFrequency::from_value(120).frame_type(),
            FrameType::Erasure
        );
        assert_eq!(FundamentalFrequency::from_value(120).frequency(), 0.0);
        assert_eq!(
            FundamentalFrequency::from_value(124).frame_type(),
            FrameType::Silence
        );
        assert_eq!(FundamentalFrequency::from_value(124).l(), 15);
        assert_eq!(FundamentalFrequency::from_value(125).l(), 14);
        assert_eq!(
            FundamentalFrequency::from_value(127).frame_type(),
            FrameType::Tone
        );
    }

    #[test]
    fn differential_gain_boundaries() {
        assert_eq!(DIFFERENTIAL_GAINS[0], (-2.0, 1.0));
        assert_eq!(DIFFERENTIAL_GAINS[31], (6.874496, 1.0));
    }

    #[test]
    fn lmpr_block_length_boundaries() {
        assert_eq!(LMPR_BLOCK_LENGTHS[0], [0, 0, 0, 0, 0]);
        assert_eq!(LMPR_BLOCK_LENGTHS[9], [0, 2, 2, 2, 3]);
        assert_eq!(LMPR_BLOCK_LENGTHS[56], [0, 11, 13, 15, 17]);
    }

    #[test]
    fn tone_lookup_and_classification() {
        assert_eq!(Tone::from_value(4), INVALID_TONE);
        assert_eq!(Tone::from_value(123), INVALID_TONE);
        assert_eq!(Tone::from_value(200), INVALID_TONE);

        let t5 = Tone::from_value(5);
        assert_eq!(t5.frequency1, 156.25);
        assert_eq!(t5.tone_type(), ToneType::Discrete);
        assert!(!t5.has_frequency2());

        assert_eq!(Tone::from_value(122).tone_type(), ToneType::Discrete);
        assert_eq!(Tone::from_value(128).tone_type(), ToneType::Dtmf);
        assert_eq!(Tone::from_value(143).tone_type(), ToneType::Dtmf);
        assert_eq!(Tone::from_value(144).tone_type(), ToneType::Knox);
        assert_eq!(Tone::from_value(159).tone_type(), ToneType::Knox);
        assert_eq!(Tone::from_value(160).tone_type(), ToneType::CallProgress);

        let t163 = Tone::from_value(163);
        assert_eq!(t163.tone_type(), ToneType::CallProgress);
        assert_eq!(t163.label, "CALL PROGRESS");
        assert!(t163.has_frequency2());
    }

    #[test]
    fn modulation_vector_starts_with_known_bits() {
        // Hand computed from alg 52 to 54 for seed 0: pr values 13849,
        // 50430, 21951, 10284 give bits 0, 1, 0, 0.
        assert_eq!(modulation_vector(0) >> 19, 0b0100);
    }

    #[test]
    fn tone_frame_parses_and_decodes() {
        // U0: tone check 63 in bits 0-5, amplitude high 42 in bits 6-11.
        let u0 = (63 << 6) | 42;
        let c0 = golay24_codeword(u0);
        // C1 data: tone id 100 in bits 0-7, zeros in bits 8-11, scrambled
        // with the modulation vector seeded from U0.
        let c1 = golay23_codeword(100 << 4) ^ modulation_vector(u0);
        let bytes = build_frame(c0, c1, 0, 0);

        let frame = AmbeFrame::new(&bytes);
        assert_eq!(frame.frame_type(), FrameType::Tone);
        assert!(frame.is_tone_frame());
        assert_eq!(frame.errors(), [0, 0]);

        let tone = frame.tone().unwrap();
        assert_eq!(tone.value, 100);
        assert_eq!(tone.frequency1, 3125.0);
        assert_eq!(tone.tone_type(), ToneType::Discrete);

        let tone_parameters = frame.tone_parameters();
        assert!(tone_parameters.is_valid_tone());
        assert_eq!(tone_parameters.amplitude, 84);

        let mut synthesizer = AmbeSynthesizer::new();
        let audio = synthesizer.decode(&bytes);
        assert_valid_audio(&audio);
        assert!(audio.iter().any(|s| s.abs() > 0.1));
    }

    #[test]
    fn voice_frame_parses_and_decodes() {
        // U0 = 0x531: b0 high 5, b1 high 3, b2 high 1.
        let u0 = 0x531;
        let c0 = golay24_codeword(u0);
        // C1 data: b3 high 0x40 in bits 0-7, b4 high 2 in bits 8-11.
        let c1 = golay23_codeword((0x40 << 4) | 0b0010) ^ modulation_vector(u0);
        let bytes = build_frame(c0, c1, 0, 0);

        let frame = AmbeFrame::new(&bytes);
        assert_eq!(frame.frame_type(), FrameType::Voice);
        assert_eq!(frame.errors(), [0, 0]);
        assert_eq!(frame.fundamental_frequency(), FundamentalFrequency(40));
        assert_eq!(frame.b, [40, 6, 2, 128, 16, 0, 0, 0, 0]);
        assert!(frame.tone().is_none());

        let previous = AmbeModelParameters::new();
        let parameters = frame.voice_parameters(&previous);
        assert_eq!(parameters.base.l, 17);
        assert_eq!(parameters.base.frame_type, FrameType::Voice);
        // Gain: G2 table entry plus adjustment, previous gain zero.
        assert!((parameters.gain - (0.297941 + 1.25)).abs() < 1e-6);
        // Voicing vector V6 is [t, t, t, f, t, t, t, t]; band 8 maps to
        // voice index 3.
        assert!(parameters.base.voicing[1]);
        assert!(!parameters.base.voicing[8]);
        assert_eq!(parameters.base.voicing.len(), 18);
        assert_eq!(parameters.base.log2_spectral_amplitudes.len(), 18);
        assert!(parameters
            .base
            .spectral_amplitudes
            .iter()
            .all(|a| a.is_finite()));

        let mut synthesizer = AmbeSynthesizer::new();
        for _ in 0..4 {
            let audio = synthesizer.decode(&bytes);
            assert_valid_audio(&audio);
        }
    }

    #[test]
    fn erasure_frame_produces_white_noise() {
        // U0 = 0xF00: b0 high 15 makes b0 = 120, the W120 erasure, while
        // the tone check reads 60 and stays a non tone frame.
        let u0 = 0xF00;
        let c0 = golay24_codeword(u0);
        let c1 = golay23_codeword(0) ^ modulation_vector(u0);
        let bytes = build_frame(c0, c1, 0, 0);

        let frame = AmbeFrame::new(&bytes);
        assert_eq!(frame.frame_type(), FrameType::Erasure);

        let mut synthesizer = AmbeSynthesizer::new();
        let audio = synthesizer.decode(&bytes);
        assert_valid_audio(&audio);
        assert!(audio.iter().any(|s| *s != 0.0));
        assert!(synthesizer.previous_frame.is_erasure_frame());
    }

    #[test]
    fn invalid_tone_frame_falls_back_to_previous_frame() {
        // Tone id 0 is below the valid tone range, so the decoded tone is
        // INVALID and the synthesizer replays the previous frame.
        let u0 = 63 << 6;
        let c0 = golay24_codeword(u0);
        let c1 = golay23_codeword(0) ^ modulation_vector(u0);
        let bytes = build_frame(c0, c1, 0, 0);

        let frame = AmbeFrame::new(&bytes);
        assert!(frame.is_tone_frame());
        assert!(!frame.tone_parameters().is_valid_tone());

        let mut synthesizer = AmbeSynthesizer::new();
        let audio = synthesizer.decode(&bytes);
        assert_valid_audio(&audio);
    }

    #[test]
    fn default_parameters_match_java_initializers() {
        let parameters = AmbeModelParameters::new();
        assert_eq!(parameters.fundamental, W124);
        assert_eq!(parameters.base.frame_type, FrameType::Voice);
        assert_eq!(parameters.base.l, 15);
        assert_eq!(parameters.base.voicing, vec![false; 16]);
        assert_eq!(parameters.base.log2_spectral_amplitudes, vec![0.0; 16]);
        assert_eq!(parameters.base.spectral_amplitudes, vec![1.0; 16]);
        assert_eq!(parameters.base.enhanced_spectral_amplitudes, vec![1.0; 16]);
        assert_eq!(parameters.gain, 0.0);
        assert_eq!(parameters.base.local_energy, 75000.0);
        assert_eq!(parameters.base.amplitude_threshold, 20480);
        assert_eq!(parameters.base.repeat_count, 0);
        let expected_frequency = (std::f64::consts::PI / 32.0 * 2.0 * std::f64::consts::PI) as f32;
        assert_eq!(parameters.base.frequency, expected_frequency);
    }
}
