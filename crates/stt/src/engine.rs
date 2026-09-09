//! One decoder or the other, behind the call the receiver makes.

use candle_core::Device;
use common::{Error, Result};

use crate::{qwen3, Family, Files, Segment, Transcript, Whisper};

/// A loaded model of whichever family the files were.
pub enum Engine {
    Whisper(Whisper),
    Qwen3(qwen3::AsrInference),
}

impl Engine {
    /// Load whatever `files` are onto `device`. `language` is a hint for
    /// Whisper's multilingual models and ignored by Qwen3-ASR, which names
    /// the language it heard instead.
    pub fn load(files: &Files, device: Device, language: Option<&str>) -> Result<Self> {
        match files.family {
            Family::Whisper => Whisper::load(files, device, language).map(Self::Whisper),
            Family::Qwen3Asr => {
                let dir = files
                    .config
                    .parent()
                    .ok_or_else(|| Error::other("qwen3: config has no directory"))?;
                qwen3::AsrInference::load(dir, device)
                    .map(Self::Qwen3)
                    .map_err(|e| Error::other(format!("qwen3: {e}")))
            }
        }
    }

    pub fn family(&self) -> Family {
        match self {
            Self::Whisper(_) => Family::Whisper,
            Self::Qwen3(_) => Family::Qwen3Asr,
        }
    }

    /// One utterance of PCM at `rate` to text.
    ///
    /// Qwen3-ASR reads the whole utterance at once and reports no
    /// confidence, so its transcript is one segment marked as read well
    /// whenever it said anything at all. It does say what it heard nothing
    /// in: an empty string, which is carried as no speech.
    pub fn transcribe(&mut self, pcm: &[f32], rate: f64) -> Result<Transcript> {
        match self {
            Self::Whisper(w) => w.transcribe(pcm, rate),
            Self::Qwen3(q) => {
                let pcm = crate::to_whisper_rate(pcm, rate);
                let seconds = pcm.len() as f64 / crate::RATE;
                let r = q
                    .transcribe_samples(&pcm, qwen3::TranscribeOptions::default())
                    .map_err(|e| Error::other(format!("qwen3: {e}")))?;
                let text = r.text.trim().to_string();
                let heard = !text.is_empty();
                Ok(Transcript {
                    text: text.clone(),
                    segments: vec![Segment {
                        start_s: 0.0,
                        end_s: seconds,
                        text,
                        avg_logprob: 0.0,
                        no_speech_prob: if heard { 0.0 } else { 1.0 },
                    }],
                    language: Some(r.language),
                })
            }
        }
    }
}
