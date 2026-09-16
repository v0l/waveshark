//! The phoneme encoder: what each symbol sounds like, in context.
//!
//! An embedding, three convolutions wide enough to see a couple of phonemes
//! either side, and a recurrent layer that carries the rest of the sentence.
//! Its output is stretched over time by the durations the prosody predictor
//! chooses, and that stretched form is what the vocoder actually reads.

use super::Config;
use super::layers::{BiLstm, ChannelNorm, weight_norm_conv1d};
use candle_core::{Result, Tensor};
use candle_nn::{Conv1dConfig, Embedding, Module, VarBuilder};

pub struct TextEncoder {
    embedding: Embedding,
    convs: Vec<(candle_nn::Conv1d, ChannelNorm)>,
    lstm: BiLstm,
}

impl TextEncoder {
    pub fn load(cfg: &Config, vb: VarBuilder) -> Result<Self> {
        let c = cfg.hidden_dim;
        let k = cfg.text_encoder_kernel_size;
        let conv = Conv1dConfig { padding: (k - 1) / 2, ..Conv1dConfig::default() };
        let mut convs = Vec::new();
        for i in 0..cfg.n_layer {
            let vb = vb.pp("cnn").pp(i);
            convs.push((
                weight_norm_conv1d(c, c, k, conv, true, vb.pp(0))?,
                ChannelNorm::load(c, vb.pp(1))?,
            ));
        }
        Ok(Self {
            embedding: candle_nn::embedding(cfg.n_token, c, vb.pp("embedding"))?,
            convs,
            lstm: BiLstm::load(c, c / 2, vb.pp("lstm"))?,
        })
    }

    /// `ids` is `[batch, time]`; the result is `[batch, channels, time]`, the
    /// way round everything downstream of here wants it.
    pub fn forward(&self, ids: &Tensor) -> Result<Tensor> {
        let mut x = self.embedding.forward(ids)?.transpose(1, 2)?.contiguous()?;
        for (conv, norm) in &self.convs {
            x = candle_nn::ops::leaky_relu(&norm.forward(&conv.forward(&x)?)?, 0.2)?;
        }
        let x = self.lstm.forward(&x.transpose(1, 2)?.contiguous()?)?;
        x.transpose(1, 2)?.contiguous()
    }
}
