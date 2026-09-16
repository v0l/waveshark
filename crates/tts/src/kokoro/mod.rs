//! Kokoro: a phoneme language model, a prosody predictor and a vocoder.
//!
//! Eighty-two million parameters, which is small enough to run ahead of real
//! time on a processor and to load beside everything else the receiver is
//! doing. It reads phonemes rather than letters, so the front end in
//! [`crate::g2p`] is what turns a sentence into something it can say, and a
//! voice is a tensor rather than a description: 510 style vectors, one per
//! sentence length, published per speaker.
//!
//! The order of the passes is the whole architecture. The phoneme model reads
//! the sentence; the prosody predictor decides how long each phoneme lasts
//! and at what pitch; the sentence is stretched to those durations; and the
//! vocoder turns the stretched sentence, the pitch and the loudness into
//! samples. Nothing is autoregressive, so a sentence costs one pass rather
//! than one pass per frame, which is the difference from what was here
//! before.

mod albert;
mod generator;
mod layers;
mod prosody;
pub mod stft;
mod text;

use candle_core::{DType, Device, Tensor};
use candle_nn::VarBuilder;
use common::{Error, Result};
use std::collections::HashMap;
use std::path::Path;

/// Samples a second the vocoder makes, which is a property of the model.
pub const RATE: f64 = 24_000.0;

/// Floats in one style vector: the vocoder's half and the prosody's half.
pub const STYLE_FLOATS: usize = 256;

/// The dimensions and the vocabulary, as the published `config.json` gives
/// them. Read rather than assumed: a second Kokoro checkpoint changes the
/// vocabulary, and a receiver that assumed this one's would speak nonsense
/// rather than fail.
#[derive(Debug, serde::Deserialize)]
pub struct Config {
    pub n_token: usize,
    pub hidden_dim: usize,
    pub style_dim: usize,
    pub n_layer: usize,
    pub max_dur: usize,
    pub text_encoder_kernel_size: usize,
    pub plbert: PlBert,
    pub istftnet: IstftNet,
    pub vocab: HashMap<String, u32>,
}

#[derive(Debug, serde::Deserialize)]
pub struct PlBert {
    pub hidden_size: usize,
    pub num_attention_heads: usize,
    pub intermediate_size: usize,
    pub max_position_embeddings: usize,
    pub num_hidden_layers: usize,
    /// Not in the file: ALBERT's embedding is narrower than its hidden layers
    /// and is widened on the way in, and this checkpoint's is 128.
    #[serde(default = "PlBert::embedding_size")]
    pub embedding_size: usize,
}

impl PlBert {
    fn embedding_size() -> usize {
        128
    }
}

#[derive(Debug, serde::Deserialize)]
pub struct IstftNet {
    pub upsample_kernel_sizes: Vec<usize>,
    pub upsample_rates: Vec<usize>,
    pub gen_istft_hop_size: usize,
    pub gen_istft_n_fft: usize,
    pub resblock_dilation_sizes: Vec<Vec<usize>>,
    pub resblock_kernel_sizes: Vec<usize>,
    pub upsample_initial_channel: usize,
}

pub struct Kokoro {
    bert: albert::Albert,
    bert_encoder: candle_nn::Linear,
    predictor: prosody::ProsodyPredictor,
    text_encoder: text::TextEncoder,
    decoder: generator::Decoder,
    config: Config,
    device: Device,
}

impl Kokoro {
    /// Load the checkpoint at `weights` with the shapes `config` gives.
    ///
    /// The checkpoint is a PyTorch file holding five state dicts rather than
    /// one flat set of tensors, so each part is opened by name. Everything is
    /// single precision: the model is small, and half precision on a
    /// convolutional vocoder costs quality it cannot spare.
    pub fn load(config: &Path, weights: &Path, device: Device) -> Result<Self> {
        let text = std::fs::read_to_string(config)?;
        let config: Config =
            serde_json::from_str(&text).map_err(|e| Error::other(format!("config: {e}")))?;
        let sub = |name: &'static str| -> Result<VarBuilder<'static>> {
            let tensors = candle_core::pickle::PthTensors::new(weights, Some(name))
                .map_err(|e| Error::other(format!("{name}: {e}")))?;
            // Every tensor in the checkpoint is under `module.`, from the
            // data-parallel wrapper the model was trained in.
            Ok(VarBuilder::from_backend(Box::new(tensors), DType::F32, device.clone()).pp("module"))
        };
        Ok(Self {
            bert: albert::Albert::load(&config, sub("bert")?).map_err(candle)?,
            bert_encoder: candle_nn::linear(
                config.plbert.hidden_size,
                config.hidden_dim,
                sub("bert_encoder")?,
            )
            .map_err(candle)?,
            predictor: prosody::ProsodyPredictor::load(&config, sub("predictor")?)
                .map_err(candle)?,
            text_encoder: text::TextEncoder::load(&config, sub("text_encoder")?).map_err(candle)?,
            decoder: generator::Decoder::load(&config, sub("decoder")?).map_err(candle)?,
            config,
            device,
        })
    }

    /// Token ids for a phoneme string, silently dropping anything the model
    /// has no symbol for, and bracketed by the silence token the model was
    /// trained to start and end on.
    pub fn tokens(&self, phonemes: &str) -> Vec<u32> {
        let mut ids = vec![0u32];
        ids.extend(phonemes.chars().filter_map(|c| {
            let mut buf = [0u8; 4];
            self.config.vocab.get(c.encode_utf8(&mut buf) as &str).copied()
        }));
        ids.push(0);
        ids
    }

    /// The longest sentence the phoneme model can attend over.
    pub fn max_tokens(&self) -> usize {
        self.config.plbert.max_position_embeddings
    }

    /// Say `phonemes` in `voice`, at `speed` times the pace the voice implies.
    ///
    /// `voice` is the published style tensor, `[510, 1, 2 * style_dim]`: one
    /// vector per sentence length, because how a speaker paces a short reply
    /// is not how they pace a long one. The first half of the chosen vector
    /// steers the vocoder and the second half the prosody.
    pub fn say(&self, phonemes: &str, voice: &Tensor, speed: f64) -> Result<Vec<f32>> {
        let ids = self.tokens(phonemes);
        if ids.len() > self.max_tokens() {
            return Err(Error::other(format!(
                "{} phonemes is more than the model's {}",
                ids.len(),
                self.max_tokens()
            )));
        }
        Ok(self.generate(&ids, voice, speed).map_err(candle)?.audio)
    }

    /// The same pass, with what each stage produced kept.
    ///
    /// This is what the port is checked against: every stage has a tensor the
    /// reference implementation also produces, so a fault can be attributed
    /// to one of them rather than heard as a wrong noise at the end.
    pub fn stages(&self, phonemes: &str, voice: &Tensor, speed: f64) -> Result<Stages> {
        let ids = self.tokens(phonemes);
        self.generate(&ids, voice, speed).map_err(candle)
    }

    fn generate(&self, ids: &[u32], voice: &Tensor, speed: f64) -> candle_core::Result<Stages> {
        let style = voice.get(ids.len() - 2)?.to_device(&self.device)?;
        let style = match style.rank() {
            1 => style.unsqueeze(0)?,
            _ => style,
        };
        let half = self.config.style_dim;
        let voiced = style.narrow(1, 0, half)?.contiguous()?;
        let spoken = style.narrow(1, half, half)?.contiguous()?;

        let ids = Tensor::from_vec(ids.to_vec(), (1, ids.len()), &self.device)?;
        let hidden = self.bert.forward(&ids)?;
        let d_en = candle_nn::Module::forward(&self.bert_encoder, &hidden)?
            .transpose(1, 2)?
            .contiguous()?;

        let d = self.predictor.encode(&d_en, &spoken)?;
        let durations = self.predictor.durations(&d, speed)?;
        let alignment = alignment_matrix(&durations, &self.device)?;

        let en = d.transpose(1, 2)?.contiguous()?.matmul(&alignment)?;
        let (f0, energy, inner) = self.predictor.pitch_and_energy(&en, &spoken)?;
        let t_en = self.text_encoder.forward(&ids)?;
        let asr = t_en.matmul(&alignment)?;

        let (audio, decoder) = self.decoder.traced(&asr, &f0, &energy, &voiced)?;
        Ok(Stages {
            hidden,
            d_en,
            d,
            durations,
            en,
            f0,
            energy,
            t_en,
            asr,
            inner,
            decoder,
            audio: audio.flatten_all()?.to_vec1()?,
        })
    }
}

/// What each stage of one pass produced.
pub struct Stages {
    pub hidden: Tensor,
    pub d_en: Tensor,
    pub d: Tensor,
    pub durations: Vec<usize>,
    pub en: Tensor,
    pub f0: Tensor,
    pub energy: Tensor,
    pub t_en: Tensor,
    pub asr: Tensor,
    /// The shared recurrent output and each prosody block after it.
    pub inner: Vec<Tensor>,
    /// Named stages of the vocoder.
    pub decoder: Vec<(String, Tensor)>,
    pub audio: Vec<f32>,
}

/// The matrix that stretches one column per phoneme into one column per
/// frame: a one in the row of the phoneme that frame belongs to.
///
/// Multiplying by it is how the encoded sentence is held for as long as the
/// durations say. It is a matrix rather than a repeat because the same
/// stretch has to be applied to two different encodings, and because the
/// reference builds it this way.
fn alignment_matrix(durations: &[usize], device: &Device) -> candle_core::Result<Tensor> {
    let frames: usize = durations.iter().sum();
    let mut m = vec![0f32; durations.len() * frames];
    let mut at = 0;
    for (i, &d) in durations.iter().enumerate() {
        for f in at..at + d {
            m[i * frames + f] = 1.0;
        }
        at += d;
    }
    Tensor::from_vec(m, (1, durations.len(), frames), device)
}

fn candle(e: candle_core::Error) -> Error {
    Error::other(format!("kokoro: {e}"))
}
