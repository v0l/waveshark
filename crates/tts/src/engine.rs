//! A loaded voice, and one sentence at a time through it.
//!
//! What the receiver asks for is samples for a sentence; what that takes is a
//! dictionary lookup, a forward pass and an inverse transform. The model is
//! loaded once and kept, because the load is a third of a gigabyte off disc
//! and every reply after the first should not pay it.

use crate::kokoro::Kokoro;
use crate::{DeviceChoice, Files, g2p};
use candle_core::{Device, Tensor};
use common::{Error, Result};

pub struct Engine {
    model: Kokoro,
    english: g2p::English,
    /// The style tensors for the chosen voice, `[510, 1, 256]`.
    voice: Tensor,
    /// The voice's own pace. A setting nothing reaches yet: the model takes
    /// it, and there is nowhere an operator would set it from.
    speed: f64,
}

impl Engine {
    pub fn load(files: &Files, device: Device) -> Result<Self> {
        let model = Kokoro::load(&files.config, &files.weights, device.clone())?;
        let english = g2p::English::load(&files.lexicon)?;
        let voice = read_voice(&files.voice, &device)?;
        Ok(Self { model, english, voice, speed: 1.0 })
    }

    /// Load on what `choice` names, and say what that turned out to be and
    /// why, when it is not what was asked for.
    ///
    /// A card with no room left is the common case on a machine already
    /// running a language model, and the fall back to the processor is worth
    /// taking here: this model is ahead of real time on a processor, so the
    /// receiver still answers inside an over. It is worth saying out loud all
    /// the same, which is the third string.
    pub fn load_on(files: &Files, choice: DeviceChoice) -> Result<(Self, String, String)> {
        let device = choice.open()?;
        let label = crate::device_label(&device);
        let on_gpu = !matches!(device, Device::Cpu);
        match Self::load(files, device) {
            Ok(v) => Ok((v, label, String::new())),
            Err(e) if on_gpu && choice == DeviceChoice::Auto => {
                tracing::warn!("{label} could not load the speech model ({e}); using the CPU");
                let note = format!("{label} could not load it: {e}");
                let v = Self::load(files, Device::Cpu)?;
                Ok((v, crate::device_label(&Device::Cpu), note))
            }
            Err(e) => Err(e),
        }
    }

    /// Samples a second, which is the vocoder's rate and not a choice.
    pub fn rate(&self) -> f64 {
        crate::kokoro::RATE
    }

    /// Say `text`, up to `max_s` of speech.
    ///
    /// A sentence longer than the model attends over is said in pieces, split
    /// where a person would breathe, because the alternative is a refusal
    /// halfway through an over.
    pub fn say(&mut self, text: &str, max_s: f64) -> Result<Vec<f32>> {
        let mut out = Vec::new();
        for part in sentences(text) {
            let said = self.english.say(&part);
            if said.phonemes.is_empty() {
                continue;
            }
            if !said.guessed.is_empty() {
                tracing::debug!("guessed at {}", said.guessed.join(", "));
            }
            out.extend(self.model.say(&said.phonemes, &self.voice, self.speed)?);
            if out.len() as f64 / self.rate() >= max_s {
                break;
            }
        }
        out.truncate((max_s * self.rate()) as usize);
        Ok(out)
    }

    /// What the front end made of a sentence, for a settings pane that wants
    /// to show what a word will be said as.
    pub fn phonemes(&self, text: &str) -> g2p::Said {
        self.english.say(text)
    }
}

impl std::fmt::Debug for Engine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Kokoro at {} Hz", crate::kokoro::RATE)
    }
}

/// A voice file: 510 style vectors of 256 floats, little endian, one per
/// sentence length.
fn read_voice(path: &std::path::Path, device: &Device) -> Result<Tensor> {
    let raw = std::fs::read(path)?;
    if raw.len() % (4 * crate::kokoro::STYLE_FLOATS) != 0 {
        return Err(Error::other(format!("{} is not a voice", path.display())));
    }
    let values: Vec<f32> =
        raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let rows = values.len() / crate::kokoro::STYLE_FLOATS;
    Tensor::from_vec(values, (rows, 1, crate::kokoro::STYLE_FLOATS), device)
        .map_err(|e| Error::other(format!("voice: {e}")))
}

/// Split a passage where a person would breathe.
///
/// The model attends over about five hundred phonemes and rushes a sentence
/// near that, so a long reply is said a sentence at a time and joined. The
/// split is on the punctuation a sentence ends with, keeping it, because the
/// model reads a full stop as a fall and a question mark as a rise.
fn sentences(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut current = String::new();
    for c in text.chars() {
        current.push(c);
        if matches!(c, '.' | '!' | '?') && current.trim().len() > 1 {
            out.push(std::mem::take(&mut current).trim().to_string());
        }
    }
    let rest = current.trim();
    if !rest.is_empty() {
        out.push(rest.to_string());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_passage_is_split_where_somebody_would_breathe() {
        let parts = sentences("Radio check. How do you read me? Over.");
        assert_eq!(parts, ["Radio check.", "How do you read me?", "Over."]);
    }

    #[test]
    fn a_passage_without_an_ending_is_one_piece() {
        assert_eq!(sentences("station calling"), ["station calling"]);
    }
}
