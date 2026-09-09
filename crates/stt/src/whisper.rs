//! One transmission of PCM in, one transcript out.
//!
//! Greedy decoding only. The reference implementation retries a chunk at
//! rising temperatures when the first attempt looks degenerate, which needs a
//! sampler and a seed; a receiver wants the same audio to produce the same
//! row twice, and a bad transcript that says it is bad through `avg_logprob`
//! is more useful here than a different bad transcript. The thresholds the
//! fallback used are still applied, as a verdict rather than as a retry.

use candle_core::{Device, IndexOp, Tensor};
use candle_nn::ops::softmax;
use candle_nn::VarBuilder;
use candle_transformers::models::whisper::{self as m, audio, Config};
use common::{Error, Result};
use tokenizers::Tokenizer;

use crate::model::{Files, Flavour};

/// 30 seconds at 16 kHz: the window Whisper's encoder was trained on. Shorter
/// audio is zero-padded to it rather than run short, because a truncated
/// window shifts the positional embeddings and the model starts inventing
/// endings for sentences that were not cut off.
const WINDOW: usize = 30 * m::SAMPLE_RATE;

/// That window in seconds, for a caller deciding how much audio to hold: a
/// buffer longer than this is not one read, it is several, and the cost of a
/// re-read grows with every second kept.
pub const WINDOW_S: f64 = 30.0;

/// What one 30-second window came out as.
#[derive(Clone, Debug)]
pub struct Segment {
    pub start_s: f64,
    pub end_s: f64,
    pub text: String,
    /// Mean log probability of the tokens chosen. Below about -1.0 the text
    /// is usually wrong, which is the threshold OpenAI's decoder retries at.
    pub avg_logprob: f64,
    /// The model's own probability that the window is not speech at all.
    pub no_speech_prob: f64,
}

impl Segment {
    /// Whether the model thinks this is speech it read correctly. The two
    /// thresholds are Whisper's own.
    pub fn credible(&self) -> bool {
        self.no_speech_prob < m::NO_SPEECH_THRESHOLD && self.avg_logprob > m::LOGPROB_THRESHOLD
    }
}

/// Everything one call transcribed to.
#[derive(Clone, Debug, Default)]
pub struct Transcript {
    pub text: String,
    pub segments: Vec<Segment>,
    /// The language the model said it heard, where it says. Whisper is
    /// told or assumes; Qwen3-ASR names it.
    pub language: Option<String>,
}

impl Transcript {
    pub fn is_empty(&self) -> bool {
        self.text.trim().is_empty()
    }

    /// Whether the model believes any window of this was speech it read
    /// correctly, by its own two thresholds.
    ///
    /// A verdict beside the words rather than instead of them. Dropping the
    /// text on this was the whole of what a caller saw, so a receiver reading
    /// a fading handheld showed nothing at all and looked broken, when what
    /// had happened was that the model was unsure and said so.
    pub fn credible(&self) -> bool {
        self.segments.iter().any(|s| s.credible())
    }

    /// The model's own probability that none of this was speech, worst
    /// window first.
    pub fn no_speech_prob(&self) -> f64 {
        self.segments.iter().map(|s| s.no_speech_prob).fold(f64::NAN, f64::min)
    }

    /// Whether any window held speech at all, whatever the model made of the
    /// words in it.
    ///
    /// Weaker than [`credible`](Self::credible) and asking a different
    /// question. A fading handheld is speech read badly; a fan, a rainstorm
    /// and an open squelch are not speech, and the model says so here rather
    /// than through the log probability of the sentence it invented for them.
    pub fn speech(&self) -> bool {
        self.segments
            .iter()
            .any(|s| s.no_speech_prob < m::NO_SPEECH_THRESHOLD && !s.text.trim().is_empty())
    }

    /// Mean log probability across the windows that were kept, or NaN when
    /// nothing was. A row shows this beside the text: a transcript without a
    /// confidence is as half a row as a packet without a level.
    pub fn avg_logprob(&self) -> f64 {
        let kept: Vec<f64> = self.segments.iter().map(|s| s.avg_logprob).collect();
        if kept.is_empty() {
            return f64::NAN;
        }
        kept.iter().sum::<f64>() / kept.len() as f64
    }
}

enum Weights {
    Normal(m::model::Whisper),
    Quantized(m::quantized_model::Whisper),
}

impl Weights {
    fn config(&self) -> &Config {
        match self {
            Self::Normal(m) => &m.config,
            Self::Quantized(m) => &m.config,
        }
    }

    fn encoder(&mut self, x: &Tensor, flush: bool) -> candle_core::Result<Tensor> {
        match self {
            Self::Normal(m) => m.encoder.forward(x, flush),
            Self::Quantized(m) => m.encoder.forward(x, flush),
        }
    }

    fn decoder(&mut self, x: &Tensor, xa: &Tensor, flush: bool) -> candle_core::Result<Tensor> {
        match self {
            Self::Normal(m) => m.decoder.forward(x, xa, flush),
            Self::Quantized(m) => m.decoder.forward(x, xa, flush),
        }
    }

    fn final_linear(&self, x: &Tensor) -> candle_core::Result<Tensor> {
        match self {
            Self::Normal(m) => m.decoder.final_linear(x),
            Self::Quantized(m) => m.decoder.final_linear(x),
        }
    }
}

pub struct Whisper {
    weights: Weights,
    tokenizer: Tokenizer,
    device: Device,
    mel_filters: Vec<f32>,
    suppress: Tensor,
    sot: u32,
    eot: u32,
    transcribe: u32,
    no_timestamps: u32,
    no_speech: u32,
    language: Option<u32>,
}

impl Whisper {
    /// Load a model onto `device`. `language` is an ISO code such as "en" and
    /// must be `None` for an English-only model, which has no token for it.
    pub fn load(files: &Files, device: Device, language: Option<&str>) -> Result<Self> {
        let config: Config = serde_json::from_str(&std::fs::read_to_string(&files.config)?)
            .map_err(|e| Error::other(format!("whisper config: {e}")))?;
        let tokenizer = Tokenizer::from_file(&files.tokenizer)
            .map_err(|e| Error::other(format!("whisper tokenizer: {e}")))?;

        let mel_bytes: &[u8] = match config.num_mel_bins {
            80 => include_bytes!("melfilters.bytes").as_slice(),
            128 => include_bytes!("melfilters128.bytes").as_slice(),
            n => return Err(Error::other(format!("unexpected num_mel_bins {n}"))),
        };
        let mut mel_filters = vec![0f32; mel_bytes.len() / 4];
        <byteorder::LittleEndian as byteorder::ByteOrder>::read_f32_into(
            mel_bytes,
            &mut mel_filters,
        );

        let suppress: Vec<f32> = (0..config.vocab_size as u32)
            .map(|i| if config.suppress_tokens.contains(&i) { f32::NEG_INFINITY } else { 0f32 })
            .collect();
        let suppress = Tensor::new(suppress.as_slice(), &device).map_err(candle)?;

        let weights = if files.quantized {
            let vb = candle_transformers::quantized_var_builder::VarBuilder::from_gguf(
                &files.weights,
                &device,
            )
            .map_err(candle)?;
            Weights::Quantized(m::quantized_model::Whisper::load(&vb, config).map_err(candle)?)
        } else {
            let vb = unsafe {
                VarBuilder::from_mmaped_safetensors(&[&files.weights], m::DTYPE, &device)
                    .map_err(candle)?
            };
            Weights::Normal(m::model::Whisper::load(&vb, config).map_err(candle)?)
        };

        let language = match (files.flavour, language) {
            (Flavour::English, Some(l)) if l != "en" => {
                return Err(Error::other(format!("this model is English only, not {l}")))
            }
            (Flavour::English, _) => None,
            (Flavour::Multilingual, l) => {
                let l = l.unwrap_or("en");
                Some(token(&tokenizer, &format!("<|{l}|>"))?)
            }
        };

        Ok(Self {
            sot: token(&tokenizer, m::SOT_TOKEN)?,
            eot: token(&tokenizer, m::EOT_TOKEN)?,
            transcribe: token(&tokenizer, m::TRANSCRIBE_TOKEN)?,
            no_timestamps: token(&tokenizer, m::NO_TIMESTAMPS_TOKEN)?,
            no_speech: m::NO_SPEECH_TOKENS
                .iter()
                .find_map(|t| token(&tokenizer, t).ok())
                .ok_or_else(|| Error::other("tokenizer has no non-speech token"))?,
            weights,
            tokenizer,
            device,
            mel_filters,
            suppress,
            language,
        })
    }

    /// Transcribe one call. `rate` is the codec's, and the resampling to
    /// 16 kHz happens here so a caller never has to know Whisper's.
    pub fn transcribe(&mut self, pcm: &[f32], rate: f64) -> Result<Transcript> {
        let pcm = crate::to_whisper_rate(pcm, rate);
        let mut out = Transcript::default();
        for (i, chunk) in pcm.chunks(WINDOW).enumerate() {
            let mut window = chunk.to_vec();
            window.resize(WINDOW, 0.0);
            let start_s = (i * WINDOW) as f64 / m::SAMPLE_RATE as f64;
            let seg = self.window(&window, start_s, chunk.len())?;
            // Every window's words are kept, whatever the model thought of
            // them; `credible` is how it says what it thought. Judging here
            // threw the reading away where a caller could not see that there
            // had been one.
            if !seg.text.trim().is_empty() {
                if !out.text.is_empty() {
                    out.text.push(' ');
                }
                out.text.push_str(seg.text.trim());
            }
            out.segments.push(seg);
        }
        Ok(out)
    }

    fn window(&mut self, pcm: &[f32], start_s: f64, real: usize) -> Result<Segment> {
        let config = self.weights.config();
        let bins = config.num_mel_bins;
        let mel = audio::pcm_to_mel(config, pcm, &self.mel_filters);
        let frames = mel.len() / bins;
        let mel = Tensor::from_vec(mel, (1, bins, frames), &self.device).map_err(candle)?;
        // `pcm_to_mel` appends a chunk of padding frames of its own, so a
        // window that is already exactly 30 seconds comes back longer than
        // the encoder's context. Keep the frames the audio produced.
        let mel = mel.narrow(2, 0, frames.min(m::N_FRAMES)).map_err(candle)?;
        let mut seg = self.decode(&mel)?;
        seg.start_s = start_s;
        seg.end_s = start_s + real as f64 / m::SAMPLE_RATE as f64;
        Ok(seg)
    }

    fn decode(&mut self, mel: &Tensor) -> Result<Segment> {
        let features = self.weights.encoder(mel, true).map_err(candle)?;
        let sample_len = self.weights.config().max_target_positions / 2;
        let max_positions = self.weights.config().max_target_positions;

        let mut tokens = vec![self.sot];
        if let Some(lang) = self.language {
            tokens.push(lang);
        }
        tokens.push(self.transcribe);
        tokens.push(self.no_timestamps);
        let prompt = tokens.len();

        let mut sum_logprob = 0f64;
        let mut no_speech_prob = f64::NAN;
        for i in 0..sample_len {
            let t = Tensor::new(tokens.as_slice(), &self.device)
                .map_err(candle)?
                .unsqueeze(0)
                .map_err(candle)?;
            let ys = self.weights.decoder(&t, &features, i == 0).map_err(candle)?;

            if i == 0 {
                let logits = self
                    .weights
                    .final_linear(&ys.i(..1).map_err(candle)?)
                    .map_err(candle)?
                    .i(0)
                    .map_err(candle)?
                    .i(0)
                    .map_err(candle)?;
                no_speech_prob = softmax(&logits, 0)
                    .map_err(candle)?
                    .i(self.no_speech as usize)
                    .map_err(candle)?
                    .to_scalar::<f32>()
                    .map_err(candle)? as f64;
            }

            let (_, seq_len, _) = ys.dims3().map_err(candle)?;
            let logits = self
                .weights
                .final_linear(&ys.i((..1, seq_len - 1..)).map_err(candle)?)
                .map_err(candle)?
                .i(0)
                .map_err(candle)?
                .i(0)
                .map_err(candle)?
                .broadcast_add(&self.suppress)
                .map_err(candle)?;

            let v: Vec<f32> = logits.to_vec1().map_err(candle)?;
            let next = v
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.total_cmp(b))
                .map(|(i, _)| i as u32)
                .unwrap_or(self.eot);
            tokens.push(next);
            let p = softmax(&logits, candle_core::D::Minus1)
                .map_err(candle)?
                .i(next as usize)
                .map_err(candle)?
                .to_scalar::<f32>()
                .map_err(candle)? as f64;
            if next == self.eot || tokens.len() > max_positions {
                break;
            }
            sum_logprob += p.ln();
        }

        let text = self
            .tokenizer
            .decode(&tokens, true)
            .map_err(|e| Error::other(format!("whisper detokenise: {e}")))?;
        let sampled = tokens.len().saturating_sub(prompt).max(1);
        Ok(Segment {
            start_s: 0.0,
            end_s: 0.0,
            text,
            avg_logprob: sum_logprob / sampled as f64,
            no_speech_prob,
        })
    }
}

fn token(tokenizer: &Tokenizer, t: &str) -> Result<u32> {
    tokenizer.token_to_id(t).ok_or_else(|| Error::other(format!("no token id for {t}")))
}

fn candle(e: candle_core::Error) -> Error {
    Error::other(format!("candle: {e}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resampling_lands_on_whisper_rate() {
        let pcm = vec![0.0f32; 8000];
        let out = crate::to_whisper_rate(&pcm, 8000.0);
        let ratio = out.len() as f64 / pcm.len() as f64;
        assert!((ratio - 2.0).abs() < 0.01, "{} samples from 8 kHz second", out.len());
    }

    #[test]
    fn a_pass_through_rate_is_not_resampled() {
        let pcm = vec![0.25f32; 100];
        assert_eq!(crate::to_whisper_rate(&pcm, 16_000.0), pcm);
    }
}
