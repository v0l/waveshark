//! The pieces StyleTTS 2 is built from, in the shapes the checkpoint stores.
//!
//! Three things here are not candle's own and are why this file exists. A
//! weight-normalised convolution keeps its weight as a direction and a length
//! and has to be multiplied out at load. Adaptive normalisation takes its
//! scale and shift from the voice rather than from a parameter. And every
//! recurrent layer in the model is bidirectional, which candle spells as two
//! layers read in opposite directions and joined.

use candle_core::{D, Result, Tensor};
use candle_nn::rnn::{Direction, LSTMConfig};
use candle_nn::{Conv1d, Conv1dConfig, ConvTranspose1d, ConvTranspose1dConfig, Module, RNN};
use candle_nn::{Linear, VarBuilder};

/// A convolution stored as `weight_v` and `weight_g`: a direction per output
/// channel and the length to scale it to.
///
/// PyTorch keeps it that way so the length trains separately from the
/// direction. Nothing here trains, so the two are multiplied out once and the
/// layer is an ordinary convolution afterwards.
pub fn weight_norm_conv1d(
    in_c: usize,
    out_c: usize,
    kernel: usize,
    cfg: Conv1dConfig,
    bias: bool,
    vb: VarBuilder,
) -> Result<Conv1d> {
    let v = vb.get((out_c, in_c / cfg.groups, kernel), "weight_v")?;
    let g = vb.get((out_c, 1, 1), "weight_g")?;
    let w = scale_to_length(&v, &g)?;
    let b = match bias {
        true => Some(vb.get(out_c, "bias")?),
        false => None,
    };
    Ok(Conv1d::new(w, b, cfg))
}

/// The same, for the transposed convolutions the generator upsamples with.
/// Their direction tensor is `[in, out/groups, kernel]`, and the length is
/// still one per input channel.
pub fn weight_norm_conv_transpose1d(
    in_c: usize,
    out_c: usize,
    kernel: usize,
    cfg: ConvTranspose1dConfig,
    vb: VarBuilder,
) -> Result<ConvTranspose1d> {
    let v = vb.get((in_c, out_c / cfg.groups, kernel), "weight_v")?;
    let g = vb.get((in_c, 1, 1), "weight_g")?;
    let w = scale_to_length(&v, &g)?;
    let b = vb.get(out_c, "bias")?;
    Ok(ConvTranspose1d::new(w, Some(b), cfg))
}

fn scale_to_length(v: &Tensor, g: &Tensor) -> Result<Tensor> {
    let norm = v.sqr()?.sum_keepdim(2)?.sum_keepdim(1)?.sqrt()?;
    v.broadcast_mul(&g.broadcast_div(&norm)?)
}

/// Layer normalisation across channels of a `[batch, channels, time]` tensor.
///
/// The model keeps its parameters as `gamma` and `beta` rather than the
/// `weight` and `bias` candle's own layer norm looks for, and it normalises
/// the channel axis of a tensor whose channels are in the middle, so the
/// tensor is turned end for end around the call.
pub struct ChannelNorm {
    gamma: Tensor,
    beta: Tensor,
}

impl ChannelNorm {
    pub fn load(channels: usize, vb: VarBuilder) -> Result<Self> {
        Ok(Self { gamma: vb.get(channels, "gamma")?, beta: vb.get(channels, "beta")? })
    }

    pub fn forward(&self, x: &Tensor) -> Result<Tensor> {
        let x = x.transpose(1, 2)?.contiguous()?;
        let x = candle_nn::ops::layer_norm(&x, &self.gamma, &self.beta, 1e-5)?;
        x.transpose(1, 2)?.contiguous()
    }
}

/// Layer normalisation whose scale and shift come from the voice.
///
/// `fc` reads the style vector and produces a gamma and a beta per channel,
/// so the same weights speak in whichever voice they are handed. The
/// normalisation itself carries no parameters.
pub struct AdaLayerNorm {
    fc: Linear,
    channels: usize,
}

impl AdaLayerNorm {
    pub fn load(style: usize, channels: usize, vb: VarBuilder) -> Result<Self> {
        Ok(Self { fc: candle_nn::linear(style, channels * 2, vb.pp("fc"))?, channels })
    }

    /// `x` is `[batch, time, channels]`, `s` is `[batch, style]`.
    pub fn forward(&self, x: &Tensor, s: &Tensor) -> Result<Tensor> {
        let h = self.fc.forward(s)?;
        let gamma = h.narrow(D::Minus1, 0, self.channels)?.unsqueeze(1)?;
        let beta = h.narrow(D::Minus1, self.channels, self.channels)?.unsqueeze(1)?;
        let x = normalise_last(x, 1e-5)?;
        x.broadcast_mul(&(gamma + 1.0)?)?.broadcast_add(&beta)
    }
}

/// Instance normalisation with the scale and shift taken from the voice.
///
/// The difference from [`AdaLayerNorm`] is which axis is normalised: here
/// each channel is normalised along time, which is what makes the statistics
/// of the generated audio the speaker's rather than the sentence's. The
/// checkpoint carries no affine parameters for the normalisation itself, so
/// it is the plain form.
pub struct AdaIn1d {
    fc: Linear,
    channels: usize,
}

impl AdaIn1d {
    pub fn load(style: usize, channels: usize, vb: VarBuilder) -> Result<Self> {
        Ok(Self { fc: candle_nn::linear(style, channels * 2, vb.pp("fc"))?, channels })
    }

    /// `x` is `[batch, channels, time]`, `s` is `[batch, style]`.
    pub fn forward(&self, x: &Tensor, s: &Tensor) -> Result<Tensor> {
        let h = self.fc.forward(s)?;
        let gamma = h.narrow(D::Minus1, 0, self.channels)?.unsqueeze(2)?;
        let beta = h.narrow(D::Minus1, self.channels, self.channels)?.unsqueeze(2)?;
        // Along time, not across channels: each channel is normalised over
        // the utterance, which is what makes the statistics of the output the
        // speaker's rather than the sentence's.
        let x = normalise_last(x, 1e-5)?;
        x.broadcast_mul(&(gamma + 1.0)?)?.broadcast_add(&beta)
    }
}

/// Zero mean and unit variance along the last axis, with no parameters.
fn normalise_last(x: &Tensor, eps: f64) -> Result<Tensor> {
    let mean = x.mean_keepdim(D::Minus1)?;
    let centred = x.broadcast_sub(&mean)?;
    let var = centred.sqr()?.mean_keepdim(D::Minus1)?;
    centred.broadcast_div(&(var + eps)?.sqrt()?)
}

/// Snake: `x + sin(ax)^2 / a`, the periodic activation the generator uses so
/// a resonance can survive a residual block.
pub fn snake(x: &Tensor, alpha: &Tensor) -> Result<Tensor> {
    let ax = x.broadcast_mul(alpha)?;
    x + ax.sin()?.sqr()?.broadcast_div(alpha)?
}

/// A bidirectional single-layer LSTM, which is what every recurrent layer in
/// this model is.
///
/// Two candle LSTMs, one reading each way, their outputs concatenated per
/// step. PyTorch names the backward half with a `_reverse` suffix, which is
/// what `Direction::Backward` asks the var builder for.
pub struct BiLstm {
    forward: candle_nn::LSTM,
    backward: candle_nn::LSTM,
}

impl BiLstm {
    pub fn load(in_dim: usize, hidden: usize, vb: VarBuilder) -> Result<Self> {
        let cfg = LSTMConfig::default();
        let back = LSTMConfig { direction: Direction::Backward, ..LSTMConfig::default() };
        Ok(Self {
            forward: candle_nn::lstm(in_dim, hidden, cfg, vb.clone())?,
            backward: candle_nn::lstm(in_dim, hidden, back, vb)?,
        })
    }

    /// `x` is `[batch, time, in_dim]`, and the result `[batch, time, 2*hidden]`.
    pub fn forward(&self, x: &Tensor) -> Result<Tensor> {
        let ahead = RNN::states_to_tensor(&self.forward, &self.forward.seq(x)?)?;
        let behind = self.backward.seq(&flip_time(x)?)?;
        let behind = flip_time(&RNN::states_to_tensor(&self.backward, &behind)?)?;
        Tensor::cat(&[ahead, behind], D::Minus1)?.contiguous()
    }
}

fn flip_time(x: &Tensor) -> Result<Tensor> {
    let n = x.dim(1)?;
    let idx: Vec<u32> = (0..n as u32).rev().collect();
    let idx = Tensor::from_vec(idx, n, x.device())?;
    x.index_select(&idx, 1)?.contiguous()
}

/// Repeat each step of a `[batch, channels, time]` tensor `factor` times,
/// which is the nearest-neighbour upsampling the prosody blocks use.
///
/// By index rather than by candle's own `upsample_nearest1d`, which has no
/// CUDA kernel: asked for one the model stops with `upsample-nearest1d is
/// not supported on cuda`, after the weights are on the card.
pub fn upsample_nearest(x: &Tensor, factor: usize) -> Result<Tensor> {
    let n = x.dim(D::Minus1)?;
    let idx: Vec<u32> = (0..n * factor).map(|i| (i / factor) as u32).collect();
    let idx = Tensor::from_vec(idx, n * factor, x.device())?;
    x.index_select(&idx, D::Minus1)?.contiguous()
}

/// A reflection pad of one sample on the left, which the last upsampling step
/// needs so the generator's output length comes out right.
pub fn reflect_pad_left(x: &Tensor) -> Result<Tensor> {
    let second = x.narrow(D::Minus1, 1, 1)?;
    Tensor::cat(&[second, x.clone()], D::Minus1)?.contiguous()
}
