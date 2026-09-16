//! The vocoder: phonemes, pitch and loudness in, samples out.
//!
//! An ISTFTNet, which is a HiFi-GAN generator that stops short of the
//! waveform: it predicts a magnitude and a phase per frame and an inverse
//! short-time transform makes the samples. Stopping at the spectrum is what
//! makes it cheap enough to be ahead of real time on a processor, because the
//! last two upsampling stages a waveform generator would need are an FFT
//! instead.
//!
//! It is excited rather than free-running: a bank of harmonics at the
//! predicted pitch, plus noise where the pitch says the sound is unvoiced, is
//! transformed and fed into every upsampling stage. That is what keeps the
//! pitch of the output the pitch that was asked for.

use super::Config;
use super::layers::{reflect_pad_left, weight_norm_conv_transpose1d, weight_norm_conv1d};
use super::prosody::{AdainResBlk1d, AdainResBlock1};
use super::stft::Stft;
use candle_core::{D, Result, Tensor};
use candle_nn::{Conv1d, Conv1dConfig, ConvTranspose1d, ConvTranspose1dConfig, Module, VarBuilder};

pub struct Decoder {
    encode: AdainResBlk1d,
    decode: Vec<AdainResBlk1d>,
    f0_conv: Conv1d,
    n_conv: Conv1d,
    asr_res: Conv1d,
    generator: Generator,
}

impl Decoder {
    pub fn load(cfg: &Config, vb: VarBuilder) -> Result<Self> {
        let s = cfg.style_dim;
        let halve = Conv1dConfig { padding: 1, stride: 2, ..Conv1dConfig::default() };
        let one = Conv1dConfig::default();
        Ok(Self {
            encode: AdainResBlk1d::load(cfg.hidden_dim + 2, 1024, s, false, vb.pp("encode"))?,
            decode: vec![
                AdainResBlk1d::load(1024 + 2 + 64, 1024, s, false, vb.pp("decode").pp(0))?,
                AdainResBlk1d::load(1024 + 2 + 64, 1024, s, false, vb.pp("decode").pp(1))?,
                AdainResBlk1d::load(1024 + 2 + 64, 1024, s, false, vb.pp("decode").pp(2))?,
                AdainResBlk1d::load(1024 + 2 + 64, 512, s, true, vb.pp("decode").pp(3))?,
            ],
            f0_conv: weight_norm_conv1d(1, 1, 3, halve, true, vb.pp("F0_conv"))?,
            n_conv: weight_norm_conv1d(1, 1, 3, halve, true, vb.pp("N_conv"))?,
            asr_res: weight_norm_conv1d(512, 64, 1, one, true, vb.pp("asr_res").pp(0))?,
            generator: Generator::load(cfg, vb.pp("generator"))?,
        })
    }

    /// `asr` is the stretched sentence `[1, 512, frames]`, `f0` and `energy`
    /// are twice that long, and `s` is the first half of the voice.
    /// The samples, and what each stage of the vocoder produced on the way:
    /// the trace is what the port is checked against, stage by stage.
    pub fn traced(
        &self,
        asr: &Tensor,
        f0: &Tensor,
        energy: &Tensor,
        s: &Tensor,
    ) -> Result<(Tensor, Vec<(String, Tensor)>)> {
        let f0_half = self.f0_conv.forward(&f0.unsqueeze(1)?)?;
        let n_half = self.n_conv.forward(&energy.unsqueeze(1)?)?;
        let x = Tensor::cat(&[asr.clone(), f0_half.clone(), n_half.clone()], 1)?;
        let mut x = self.encode.forward(&x, s)?;
        let mut trace = vec![
            ("F0h".to_string(), f0_half.clone()),
            ("Nh".to_string(), n_half.clone()),
            ("enc".to_string(), x.clone()),
        ];
        let asr_res = self.asr_res.forward(asr)?;
        trace.push(("asr_res".to_string(), asr_res.clone()));
        for (i, block) in self.decode.iter().enumerate() {
            x = Tensor::cat(&[x, asr_res.clone(), f0_half.clone(), n_half.clone()], 1)?;
            x = block.forward(&x, s)?;
            trace.push((format!("dec{i}"), x.clone()));
        }
        let (audio, mut inner) = self.generator.traced(&x, s, f0)?;
        trace.append(&mut inner);
        Ok((audio, trace))
    }
}

pub struct Generator {
    ups: Vec<ConvTranspose1d>,
    noise_convs: Vec<Conv1d>,
    noise_res: Vec<AdainResBlock1>,
    resblocks: Vec<AdainResBlock1>,
    conv_post: Conv1d,
    source: SourceModule,
    stft: Stft,
    kernels: usize,
    upsample_scale: usize,
}

impl Generator {
    fn load(cfg: &Config, vb: VarBuilder) -> Result<Self> {
        let g = &cfg.istftnet;
        let s = cfg.style_dim;
        let mut ups = Vec::new();
        let mut noise_convs = Vec::new();
        let mut noise_res = Vec::new();
        let mut resblocks = Vec::new();
        for (i, (&rate, &kernel)) in
            g.upsample_rates.iter().zip(g.upsample_kernel_sizes.iter()).enumerate()
        {
            let in_c = g.upsample_initial_channel >> i;
            let out_c = g.upsample_initial_channel >> (i + 1);
            ups.push(weight_norm_conv_transpose1d(
                in_c,
                out_c,
                kernel,
                ConvTranspose1dConfig {
                    padding: (kernel - rate) / 2,
                    output_padding: 0,
                    stride: rate,
                    dilation: 1,
                    groups: 1,
                },
                vb.pp("ups").pp(i),
            )?);
            for (j, (&k, d)) in
                g.resblock_kernel_sizes.iter().zip(g.resblock_dilation_sizes.iter()).enumerate()
            {
                let at = i * g.resblock_kernel_sizes.len() + j;
                resblocks.push(AdainResBlock1::load(out_c, k, d, s, vb.pp("resblocks").pp(at))?);
            }
            // The excitation is brought to each stage's rate by a stride, so
            // the last stage reads it a sample at a time and the earlier one
            // in strides of what is left to upsample.
            let (conv, res) = match i + 1 < g.upsample_rates.len() {
                true => {
                    let stride: usize = g.upsample_rates[i + 1..].iter().product();
                    let cfg1 = Conv1dConfig {
                        padding: stride.div_ceil(2),
                        stride,
                        ..Conv1dConfig::default()
                    };
                    (
                        candle_nn::conv1d(
                            g.gen_istft_n_fft + 2,
                            out_c,
                            stride * 2,
                            cfg1,
                            vb.pp("noise_convs").pp(i),
                        )?,
                        AdainResBlock1::load(out_c, 7, &[1, 3, 5], s, vb.pp("noise_res").pp(i))?,
                    )
                }
                false => (
                    candle_nn::conv1d(
                        g.gen_istft_n_fft + 2,
                        out_c,
                        1,
                        Conv1dConfig::default(),
                        vb.pp("noise_convs").pp(i),
                    )?,
                    AdainResBlock1::load(out_c, 11, &[1, 3, 5], s, vb.pp("noise_res").pp(i))?,
                ),
            };
            noise_convs.push(conv);
            noise_res.push(res);
        }
        let post = Conv1dConfig { padding: 3, ..Conv1dConfig::default() };
        let last = g.upsample_initial_channel >> g.upsample_rates.len();
        Ok(Self {
            conv_post: weight_norm_conv1d(
                last,
                g.gen_istft_n_fft + 2,
                7,
                post,
                true,
                vb.pp("conv_post"),
            )?,
            source: SourceModule::load(vb.pp("m_source"))?,
            stft: Stft::new(g.gen_istft_n_fft, g.gen_istft_hop_size),
            kernels: g.resblock_kernel_sizes.len(),
            upsample_scale: g.upsample_rates.iter().product::<usize>() * g.gen_istft_hop_size,
            ups,
            noise_convs,
            noise_res,
            resblocks,
        })
    }

    fn traced(
        &self,
        x: &Tensor,
        s: &Tensor,
        f0: &Tensor,
    ) -> Result<(Tensor, Vec<(String, Tensor)>)> {
        let device = x.device().clone();
        let excitation = self.source.excite(f0, self.upsample_scale)?;
        let (mag, phase) = self.stft.transform(&excitation);
        let frames = mag.len() / (self.stft.bins());
        let har = Tensor::from_vec(
            mag.iter().chain(phase.iter()).copied().collect::<Vec<f32>>(),
            (1, self.stft.bins() * 2, frames),
            &device,
        )?;

        let mut trace = vec![
            (
                "har_source".to_string(),
                Tensor::from_vec(excitation.clone(), (1, excitation.len()), &device)?,
            ),
            ("har".to_string(), har.clone()),
        ];
        let mut x = x.clone();
        for i in 0..self.ups.len() {
            let src = self.noise_res[i].forward(&self.noise_convs[i].forward(&har)?, s)?;
            x = candle_nn::ops::leaky_relu(&x, 0.1)?;
            x = self.ups[i].forward(&x)?;
            if i + 1 == self.ups.len() {
                x = reflect_pad_left(&x)?;
            }
            x = (x + src)?;
            let mut sum: Option<Tensor> = None;
            for j in 0..self.kernels {
                let y = self.resblocks[i * self.kernels + j].forward(&x, s)?;
                sum = Some(match sum {
                    None => y,
                    Some(acc) => (acc + y)?,
                });
            }
            x = (sum.expect("a generator with no residual blocks") / self.kernels as f64)?;
            trace.push((format!("up{i}"), x.clone()));
        }
        let x = self.conv_post.forward(&candle_nn::ops::leaky_relu(&x, 0.01)?)?;
        trace.push(("post".to_string(), x.clone()));
        let bins = self.stft.bins();
        let mag = x.narrow(1, 0, bins)?.exp()?;
        let phase = x.narrow(1, bins, bins)?.sin()?;
        trace.push(("spec".to_string(), mag.clone()));
        trace.push(("phase".to_string(), phase.clone()));
        let samples = self.stft.inverse(
            &mag.flatten_all()?.to_vec1::<f32>()?,
            &phase.flatten_all()?.to_vec1::<f32>()?,
            x.dim(D::Minus1)?,
        );
        let len = samples.len();
        Ok((Tensor::from_vec(samples, (1, len), &device)?, trace))
    }
}

/// The harmonic and noise excitation the vocoder is driven by.
///
/// Nine sinusoids at multiples of the predicted pitch, mixed down to one
/// signal by a learned weighting, with noise where the pitch says the sound
/// is unvoiced. It is plain arithmetic over a sample rate of audio rather
/// than a tensor graph: on a card the round trip costs less than the kernels
/// would, and this way the phase accumulates exactly once.
struct SourceModule {
    weights: Vec<f32>,
    bias: f32,
}

impl SourceModule {
    const HARMONICS: usize = 9;
    const RATE: f64 = 24_000.0;
    const SINE_AMP: f64 = 0.1;
    const NOISE_STD: f64 = 0.003;
    /// Below this the frame is unvoiced, and the excitation is noise alone.
    const VOICED_HZ: f64 = 10.0;

    fn load(vb: VarBuilder) -> Result<Self> {
        let w = vb.get((1, Self::HARMONICS), "l_linear.weight")?;
        Ok(Self {
            weights: w.flatten_all()?.to_vec1()?,
            bias: vb.get(1, "l_linear.bias")?.to_vec1::<f32>()?[0],
        })
    }

    /// One sample of excitation per sample of output, from a pitch curve at
    /// the frame rate.
    ///
    /// The reference accumulates phase at the frame rate and interpolates the
    /// ramp back up; accumulating per sample is the same ramp to within half
    /// a frame of offset, and it cannot drift. What matters either way is
    /// that phase is carried across frames: a harmonic restarted at each
    /// boundary is a buzz at the frame rate.
    fn excite(&self, f0: &Tensor, scale: usize) -> Result<Vec<f32>> {
        let frames: Vec<f32> = f0.flatten_all()?.to_vec1()?;
        let mut noise = Noise::new(0x5eed_1e55);
        let mut out = vec![0f32; frames.len() * scale];
        // Every harmonic but the fundamental starts at a phase of its own.
        // Started together they sum to an impulse at the pitch period, and
        // the excitation is a buzz rather than a voice; the reference draws
        // these at random for the same reason.
        let mut phase = [0f64; Self::HARMONICS];
        for p in phase.iter_mut().skip(1) {
            *p = noise.next_f64();
        }
        for (i, sample) in out.iter_mut().enumerate() {
            let hz = frames[i / scale] as f64;
            let voiced = hz > Self::VOICED_HZ;
            let amp = match voiced {
                true => Self::NOISE_STD,
                false => Self::SINE_AMP / 3.0,
            };
            let mut mixed = self.bias as f64;
            for (h, (phase, weight)) in phase.iter_mut().zip(&self.weights).enumerate() {
                *phase = (*phase + hz * (h + 1) as f64 / Self::RATE).fract();
                let sine = match voiced {
                    true => (*phase * std::f64::consts::TAU).sin() * Self::SINE_AMP,
                    false => 0.0,
                };
                mixed += (sine + amp * noise.normal()) * *weight as f64;
            }
            *sample = mixed.tanh() as f32;
        }
        Ok(out)
    }
}

/// Gaussian noise from a counter, so a voice sounds the same twice.
///
/// The reference draws from torch's generator, which nothing here can
/// reproduce; what matters is that the noise is noise of the right size, and
/// that a receiver saying the same thing twice says it the same way.
struct Noise(u64);

impl Noise {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_f64(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }

    /// Box-Muller, one of the pair kept.
    fn normal(&mut self) -> f64 {
        let u = self.next_f64().max(1e-12);
        let v = self.next_f64();
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}
