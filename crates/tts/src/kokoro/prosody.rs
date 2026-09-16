//! How long each phoneme lasts, at what pitch, and how loud.
//!
//! Three questions, one network. The duration encoder reads the sentence with
//! the voice appended to every step, so how long a syllable is held is a
//! property of the speaker as much as of the word. Durations come off it
//! directly; pitch and loudness come off a second pass over the same features
//! once they have been stretched out to the length the durations imply.

use super::Config;
use super::layers::weight_norm_conv_transpose1d;
use super::layers::{AdaIn1d, AdaLayerNorm, BiLstm, snake, upsample_nearest, weight_norm_conv1d};
use candle_core::{D, Result, Tensor};
use candle_nn::{Conv1d, Conv1dConfig, ConvTranspose1d, ConvTranspose1dConfig, Module, VarBuilder};

pub struct ProsodyPredictor {
    encoder: DurationEncoder,
    lstm: BiLstm,
    duration: candle_nn::Linear,
    shared: BiLstm,
    f0: Vec<AdainResBlk1d>,
    n: Vec<AdainResBlk1d>,
    f0_proj: Conv1d,
    n_proj: Conv1d,
}

impl ProsodyPredictor {
    pub fn load(cfg: &Config, vb: VarBuilder) -> Result<Self> {
        let d = cfg.hidden_dim;
        let s = cfg.style_dim;
        let blocks = |vb: VarBuilder| -> Result<Vec<AdainResBlk1d>> {
            Ok(vec![
                AdainResBlk1d::load(d, d, s, false, vb.pp(0))?,
                AdainResBlk1d::load(d, d / 2, s, true, vb.pp(1))?,
                AdainResBlk1d::load(d / 2, d / 2, s, false, vb.pp(2))?,
            ])
        };
        let one = Conv1dConfig::default();
        Ok(Self {
            encoder: DurationEncoder::load(cfg, vb.pp("text_encoder"))?,
            lstm: BiLstm::load(d + s, d / 2, vb.pp("lstm"))?,
            duration: candle_nn::linear(d, cfg.max_dur, vb.pp("duration_proj").pp("linear_layer"))?,
            shared: BiLstm::load(d + s, d / 2, vb.pp("shared"))?,
            f0: blocks(vb.pp("F0"))?,
            n: blocks(vb.pp("N"))?,
            f0_proj: candle_nn::conv1d(d / 2, 1, 1, one, vb.pp("F0_proj"))?,
            n_proj: candle_nn::conv1d(d / 2, 1, 1, one, vb.pp("N_proj"))?,
        })
    }

    /// The encoded sentence with the voice folded in, `[batch, time, d+style]`.
    pub fn encode(&self, d_en: &Tensor, s: &Tensor) -> Result<Tensor> {
        self.encoder.forward(d_en, s)
    }

    /// Frames each phoneme lasts, at least one each.
    ///
    /// The head predicts a distribution over lengths and the length is its
    /// total, which is the reference's `sigmoid(...).sum(-1)`: fifty
    /// independent gates rather than one regression, so a long vowel is
    /// several gates open rather than one large number.
    pub fn durations(&self, d: &Tensor, speed: f64) -> Result<Vec<usize>> {
        let x = self.lstm.forward(d)?;
        let total = candle_nn::ops::sigmoid(&self.duration.forward(&x)?)?.sum(D::Minus1)?;
        let total = (total / speed)?.flatten_all()?.to_vec1::<f32>()?;
        Ok(total.iter().map(|v| (v.round() as i64).max(1) as usize).collect())
    }

    /// Pitch and loudness over the stretched sentence, with what each block
    /// produced kept: the trace is what the port is checked against.
    pub fn pitch_and_energy(
        &self,
        en: &Tensor,
        s: &Tensor,
    ) -> Result<(Tensor, Tensor, Vec<Tensor>)> {
        let shared = self.shared.forward(&en.transpose(1, 2)?.contiguous()?)?;
        let x = shared.transpose(1, 2)?.contiguous()?;
        let mut trace = vec![shared];
        let mut f0 = x.clone();
        for b in &self.f0 {
            f0 = b.forward(&f0, s)?;
            trace.push(f0.clone());
        }
        let mut n = x;
        for b in &self.n {
            n = b.forward(&n, s)?;
        }
        Ok((self.f0_proj.forward(&f0)?.squeeze(1)?, self.n_proj.forward(&n)?.squeeze(1)?, trace))
    }
}

/// The sentence read three times over, with the voice concatenated to every
/// step before each pass.
///
/// Appending the style again after each layer is not redundancy: the
/// normalisation between passes is itself steered by the voice, so what the
/// next recurrent layer sees has already been shaped by it twice.
struct DurationEncoder {
    lstms: Vec<(BiLstm, AdaLayerNorm)>,
}

impl DurationEncoder {
    fn load(cfg: &Config, vb: VarBuilder) -> Result<Self> {
        let (d, s) = (cfg.hidden_dim, cfg.style_dim);
        let mut lstms = Vec::new();
        for i in 0..cfg.n_layer {
            lstms.push((
                BiLstm::load(d + s, d / 2, vb.pp("lstms").pp(i * 2))?,
                AdaLayerNorm::load(s, d, vb.pp("lstms").pp(i * 2 + 1))?,
            ));
        }
        Ok(Self { lstms })
    }

    /// `d_en` is `[batch, d, time]` and `s` is `[batch, style]`; the result is
    /// `[batch, time, d + style]`.
    fn forward(&self, d_en: &Tensor, s: &Tensor) -> Result<Tensor> {
        let time = d_en.dim(D::Minus1)?;
        let style = s.unsqueeze(1)?.broadcast_as((s.dim(0)?, time, s.dim(1)?))?.contiguous()?;
        let mut x = Tensor::cat(&[d_en.transpose(1, 2)?.contiguous()?, style.clone()], D::Minus1)?;
        for (lstm, norm) in &self.lstms {
            let hidden = lstm.forward(&x)?;
            x = Tensor::cat(&[norm.forward(&hidden, s)?, style.clone()], D::Minus1)?
                .contiguous()?;
        }
        Ok(x)
    }
}

/// A residual block whose normalisation is the voice.
///
/// Two convolutions, each preceded by an adaptive normalisation and a leaky
/// rectifier, summed with the input and scaled by `1/sqrt(2)` so the variance
/// does not grow with depth. The upsampling variant doubles the rate through
/// a depthwise transposed convolution, which is how the prosody curves reach
/// twice the frame rate of the phonemes.
pub struct AdainResBlk1d {
    norm1: AdaIn1d,
    norm2: AdaIn1d,
    conv1: Conv1d,
    conv2: Conv1d,
    shortcut: Option<Conv1d>,
    pool: Option<ConvTranspose1d>,
}

impl AdainResBlk1d {
    pub fn load(
        dim_in: usize,
        dim_out: usize,
        style: usize,
        upsample: bool,
        vb: VarBuilder,
    ) -> Result<Self> {
        let same = Conv1dConfig { padding: 1, ..Conv1dConfig::default() };
        let one = Conv1dConfig::default();
        let pool = match upsample {
            false => None,
            true => Some(weight_norm_conv_transpose1d(
                dim_in,
                dim_in,
                3,
                ConvTranspose1dConfig {
                    padding: 1,
                    output_padding: 1,
                    stride: 2,
                    dilation: 1,
                    groups: dim_in,
                },
                vb.pp("pool"),
            )?),
        };
        Ok(Self {
            norm1: AdaIn1d::load(style, dim_in, vb.pp("norm1"))?,
            norm2: AdaIn1d::load(style, dim_out, vb.pp("norm2"))?,
            conv1: weight_norm_conv1d(dim_in, dim_out, 3, same, true, vb.pp("conv1"))?,
            conv2: weight_norm_conv1d(dim_out, dim_out, 3, same, true, vb.pp("conv2"))?,
            shortcut: match dim_in == dim_out {
                true => None,
                false => {
                    Some(weight_norm_conv1d(dim_in, dim_out, 1, one, false, vb.pp("conv1x1"))?)
                }
            },
            pool,
        })
    }

    pub fn forward(&self, x: &Tensor, s: &Tensor) -> Result<Tensor> {
        let mut h = candle_nn::ops::leaky_relu(&self.norm1.forward(x, s)?, 0.2)?;
        if let Some(pool) = &self.pool {
            h = pool.forward(&h)?;
        }
        let h = self.conv1.forward(&h)?;
        let h = candle_nn::ops::leaky_relu(&self.norm2.forward(&h, s)?, 0.2)?;
        let h = self.conv2.forward(&h)?;

        let mut sc = x.clone();
        if self.pool.is_some() {
            sc = upsample_nearest(&sc, 2)?;
        }
        if let Some(conv) = &self.shortcut {
            sc = conv.forward(&sc)?;
        }
        (h + sc)? * (0.5f64).sqrt()
    }
}

/// The generator's residual block, which differs from [`AdainResBlk1d`] in
/// having three dilated pairs and a Snake activation rather than a rectifier.
pub struct AdainResBlock1 {
    convs1: Vec<Conv1d>,
    convs2: Vec<Conv1d>,
    adain1: Vec<AdaIn1d>,
    adain2: Vec<AdaIn1d>,
    alpha1: Vec<Tensor>,
    alpha2: Vec<Tensor>,
}

impl AdainResBlock1 {
    pub fn load(
        channels: usize,
        kernel: usize,
        dilations: &[usize],
        style: usize,
        vb: VarBuilder,
    ) -> Result<Self> {
        let pad = |d: usize| (kernel * d - d) / 2;
        let mut convs1 = Vec::new();
        let mut convs2 = Vec::new();
        let mut adain1 = Vec::new();
        let mut adain2 = Vec::new();
        let mut alpha1 = Vec::new();
        let mut alpha2 = Vec::new();
        for (i, &d) in dilations.iter().enumerate() {
            let c1 = Conv1dConfig { padding: pad(d), dilation: d, ..Conv1dConfig::default() };
            let c2 = Conv1dConfig { padding: pad(1), ..Conv1dConfig::default() };
            convs1.push(weight_norm_conv1d(
                channels,
                channels,
                kernel,
                c1,
                true,
                vb.pp("convs1").pp(i),
            )?);
            convs2.push(weight_norm_conv1d(
                channels,
                channels,
                kernel,
                c2,
                true,
                vb.pp("convs2").pp(i),
            )?);
            adain1.push(AdaIn1d::load(style, channels, vb.pp("adain1").pp(i))?);
            adain2.push(AdaIn1d::load(style, channels, vb.pp("adain2").pp(i))?);
            alpha1.push(vb.get((1, channels, 1), &format!("alpha1.{i}"))?);
            alpha2.push(vb.get((1, channels, 1), &format!("alpha2.{i}"))?);
        }
        Ok(Self { convs1, convs2, adain1, adain2, alpha1, alpha2 })
    }

    pub fn forward(&self, x: &Tensor, s: &Tensor) -> Result<Tensor> {
        let mut x = x.clone();
        for i in 0..self.convs1.len() {
            let h = self.adain1[i].forward(&x, s)?;
            let h = self.convs1[i].forward(&snake(&h, &self.alpha1[i])?)?;
            let h = self.adain2[i].forward(&h, s)?;
            let h = self.convs2[i].forward(&snake(&h, &self.alpha2[i])?)?;
            x = (h + x)?;
        }
        Ok(x)
    }
}
