use anyhow::Result;
use candle_core::{DType, Device, IndexOp, Tensor};
use candle_nn::ops::softmax_last_dim;
use candle_nn::{Conv2d, Conv2dConfig, LayerNorm, Module};
use std::collections::HashMap;

use super::config::AudioEncoderConfig;
use super::linear::LinearW;

// ─── Weight helpers (dense / safetensors path) ────────────────────────────────

fn get_w(weights: &HashMap<String, Tensor>, name: &str) -> anyhow::Result<Tensor> {
    weights.get(name).cloned().ok_or_else(|| anyhow::anyhow!("weight not found: {}", name))
}

fn load_linear(weights: &HashMap<String, Tensor>, prefix: &str) -> Result<LinearW> {
    Ok(LinearW::new(
        get_w(weights, &format!("{}.weight", prefix))?,
        weights.get(&format!("{}.bias", prefix)).cloned(),
    ))
}

fn load_layer_norm(weights: &HashMap<String, Tensor>, prefix: &str, eps: f64) -> Result<LayerNorm> {
    Ok(LayerNorm::new(
        get_w(weights, &format!("{}.weight", prefix))?,
        get_w(weights, &format!("{}.bias", prefix))?,
        eps,
    ))
}

fn load_conv2d(
    weights: &HashMap<String, Tensor>,
    prefix: &str,
    stride: usize,
    padding: usize,
) -> Result<Conv2d> {
    Ok(Conv2d::new(
        get_w(weights, &format!("{}.weight", prefix))?,
        weights.get(&format!("{}.bias", prefix)).cloned(),
        Conv2dConfig { stride, padding, ..Default::default() },
    ))
}

// ─── Audio Encoder Self-Attention ─────────────────────────────────────────────

struct AudioAttention {
    q_proj: LinearW,
    k_proj: LinearW,
    v_proj: LinearW,
    out_proj: LinearW,
    num_heads: usize,
    head_dim: usize,
}

impl AudioAttention {
    fn load(
        weights: &HashMap<String, Tensor>,
        prefix: &str,
        num_heads: usize,
        d_model: usize,
    ) -> Result<Self> {
        let head_dim = d_model / num_heads;
        Ok(Self {
            q_proj: load_linear(weights, &format!("{}.q_proj", prefix))?,
            k_proj: load_linear(weights, &format!("{}.k_proj", prefix))?,
            v_proj: load_linear(weights, &format!("{}.v_proj", prefix))?,
            out_proj: load_linear(weights, &format!("{}.out_proj", prefix))?,
            num_heads,
            head_dim,
        })
    }

    fn forward(&self, x: &Tensor, mask: Option<&Tensor>) -> Result<Tensor> {
        let (bsz, seq_len, _) = x.dims3()?;
        let nh = self.num_heads;
        let hd = self.head_dim;

        let q = self
            .q_proj
            .forward(x)?
            .reshape((bsz, seq_len, nh, hd))?
            .transpose(1, 2)?
            .contiguous()?;
        let k = self
            .k_proj
            .forward(x)?
            .reshape((bsz, seq_len, nh, hd))?
            .transpose(1, 2)?
            .contiguous()?;
        let v = self
            .v_proj
            .forward(x)?
            .reshape((bsz, seq_len, nh, hd))?
            .transpose(1, 2)?
            .contiguous()?;

        let scale = (hd as f64).sqrt();
        let mut attn: Tensor = (q.matmul(&k.transpose(2, 3)?.contiguous()?)? * (1.0 / scale))?;

        if let Some(m) = mask {
            attn = attn.broadcast_add(&m.to_dtype(attn.dtype())?)?;
        }

        let attn = softmax_last_dim(&attn)?;
        let out = attn.matmul(&v)?;
        let out = out.transpose(1, 2)?.contiguous()?.reshape((bsz, seq_len, nh * hd))?;
        self.out_proj.forward(&out).map_err(Into::into)
    }
}

// ─── Audio Encoder FFN ────────────────────────────────────────────────────────

struct AudioFfn {
    fc1: LinearW,
    fc2: LinearW,
}

impl AudioFfn {
    fn load(weights: &HashMap<String, Tensor>, prefix: &str) -> Result<Self> {
        Ok(Self {
            fc1: load_linear(weights, &format!("{}.fc1", prefix))?,
            fc2: load_linear(weights, &format!("{}.fc2", prefix))?,
        })
    }

    fn forward(&self, x: &Tensor) -> Result<Tensor> {
        self.fc2.forward(&self.fc1.forward(x)?.gelu_erf()?).map_err(Into::into)
    }
}

// ─── Audio Encoder Layer ──────────────────────────────────────────────────────

struct AudioEncoderLayer {
    self_attn_layer_norm: LayerNorm,
    self_attn: AudioAttention,
    final_layer_norm: LayerNorm,
    ffn: AudioFfn,
}

impl AudioEncoderLayer {
    fn load(
        weights: &HashMap<String, Tensor>,
        prefix: &str,
        num_heads: usize,
        d_model: usize,
    ) -> Result<Self> {
        Ok(Self {
            self_attn_layer_norm: load_layer_norm(
                weights,
                &format!("{}.self_attn_layer_norm", prefix),
                1e-5,
            )?,
            self_attn: AudioAttention::load(
                weights,
                &format!("{}.self_attn", prefix),
                num_heads,
                d_model,
            )?,
            final_layer_norm: load_layer_norm(
                weights,
                &format!("{}.final_layer_norm", prefix),
                1e-5,
            )?,
            ffn: AudioFfn::load(weights, prefix)?,
        })
    }

    fn forward(&self, x: &Tensor, mask: Option<&Tensor>) -> Result<Tensor> {
        // Pre-norm + self-attention + residual
        let h = self.self_attn_layer_norm.forward(x)?;
        let h = self.self_attn.forward(&h, mask)?;
        let x = (x + &h)?;

        // Pre-norm + FFN + residual
        let h = self.final_layer_norm.forward(&x)?;
        let h = self.ffn.forward(&h)?;
        (&x + &h).map_err(Into::into)
    }
}

// ─── Sinusoidal positional embedding ─────────────────────────────────────────

fn create_sinusoidal_embedding(max_len: usize, dim: usize, device: &Device) -> Result<Tensor> {
    let half_dim = dim / 2;
    let log_timescale = (10000.0f64).ln() / (half_dim as f64 - 1.0);

    let mut embeddings = vec![0.0f32; max_len * dim];
    for pos in 0..max_len {
        for i in 0..half_dim {
            let inv_ts = (-(i as f64) * log_timescale).exp();
            let angle = pos as f64 * inv_ts;
            embeddings[pos * dim + i] = angle.sin() as f32;
            embeddings[pos * dim + half_dim + i] = angle.cos() as f32;
        }
    }

    Tensor::from_vec(embeddings, (max_len, dim), device).map_err(Into::into)
}

// ─── Windowed attention mask ──────────────────────────────────────────────────

/// Build a block-diagonal attention mask for windowed encoder attention.
///
/// Tokens are grouped into windows of `window_size`. Tokens within the same
/// window can attend to each other (mask = 0.0); tokens in different windows
/// are blocked (mask = -inf). The encoder attention is bidirectional (not causal).
///
/// Returns a mask of shape [1, 1, seq_len, seq_len] suitable for broadcast_add
/// onto attention scores.
fn build_windowed_mask(seq_len: usize, window_size: usize, device: &Device) -> Result<Tensor> {
    if window_size == 0 || seq_len <= window_size {
        // All tokens fit in one window — no masking needed.
        return Tensor::zeros((1, 1, seq_len, seq_len), DType::F32, device).map_err(Into::into);
    }

    let mut mask_data = vec![0.0f32; seq_len * seq_len];
    for i in 0..seq_len {
        let win_i = i / window_size;
        for j in 0..seq_len {
            let win_j = j / window_size;
            if win_i != win_j {
                mask_data[i * seq_len + j] = f32::NEG_INFINITY;
            }
        }
    }
    Tensor::from_vec(mask_data, (1, 1, seq_len, seq_len), device).map_err(Into::into)
}

// ─── Encoder Cache for Incremental Encoding ──────────────────────────────────

/// Cached encoder state for incremental (streaming) encoding.
///
/// Stores the post-projection output of fully-completed attention windows.
/// Since windowed attention makes each window independent, completed windows
/// never change and can be reused across streaming steps.
pub struct EncoderCache {
    /// Post-projection outputs for completed windows, each [window_tokens, output_dim].
    completed_windows: Vec<Tensor>,
    /// Number of full mel-frame chunks already committed to completed windows.
    committed_chunks: usize,
}

impl EncoderCache {
    /// Create an empty encoder cache.
    pub fn new() -> Self {
        Self { completed_windows: Vec::new(), committed_chunks: 0 }
    }

    /// Total number of cached tokens across all completed windows.
    pub fn cached_tokens(&self) -> usize {
        self.completed_windows.iter().map(|t| t.dims()[0]).sum()
    }
}

impl Default for EncoderCache {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Audio Encoder ────────────────────────────────────────────────────────────

pub(crate) struct AudioEncoder {
    conv2d1: Conv2d,
    conv2d2: Conv2d,
    conv2d3: Conv2d,
    conv_out: LinearW,
    positional_embedding: Tensor,
    layers: Vec<AudioEncoderLayer>,
    ln_post: LayerNorm,
    proj1: LinearW,
    proj2: LinearW,
    config: AudioEncoderConfig,
}

impl AudioEncoder {
    pub(crate) fn load(
        weights: &HashMap<String, Tensor>,
        prefix: &str,
        config: &AudioEncoderConfig,
        device: &Device,
    ) -> Result<Self> {
        let conv2d1 = load_conv2d(weights, &format!("{}.conv2d1", prefix), 2, 1)?;
        let conv2d2 = load_conv2d(weights, &format!("{}.conv2d2", prefix), 2, 1)?;
        let conv2d3 = load_conv2d(weights, &format!("{}.conv2d3", prefix), 2, 1)?;
        let conv_out = load_linear(weights, &format!("{}.conv_out", prefix))?;

        let mut layers = Vec::new();
        for i in 0..config.encoder_layers {
            let layer = AudioEncoderLayer::load(
                weights,
                &format!("{}.layers.{}", prefix, i),
                config.encoder_attention_heads,
                config.d_model,
            )?;
            layers.push(layer);
        }

        let ln_post = load_layer_norm(weights, &format!("{}.ln_post", prefix), 1e-5)?;
        let proj1 = load_linear(weights, &format!("{}.proj1", prefix))?;
        let proj2 = load_linear(weights, &format!("{}.proj2", prefix))?;

        let positional_embedding =
            create_sinusoidal_embedding(config.max_source_positions, config.d_model, device)?;

        Ok(Self {
            conv2d1,
            conv2d2,
            conv2d3,
            conv_out,
            positional_embedding,
            layers,
            ln_post,
            proj1,
            proj2,
            config: config.clone(),
        })
    }

    /// Encode mel spectrogram [num_mel_bins, num_frames] → [num_tokens, output_dim]
    pub(crate) fn forward(&self, mel: &Tensor) -> Result<Tensor> {
        let num_frames = mel.dims()[1];
        // Logical chunk = n_window * 2 = 100 mel frames (matches official windowed attention).
        let chunk_size = self.config.n_window * 2;

        let num_full = num_frames / chunk_size;
        let tail = num_frames % chunk_size;
        let num_chunks = num_full + if tail > 0 { 1 } else { 0 };

        // Collect chunks as F32 (conv2d runs in F32).
        let mut chunk_mels: Vec<Tensor> = Vec::with_capacity(num_chunks);
        let mut chunk_valid_tokens: Vec<usize> = Vec::with_capacity(num_chunks);

        for i in 0..num_full {
            let start = i * chunk_size;
            let chunk = mel.narrow(1, start, chunk_size)?.unsqueeze(0)?;
            chunk_mels.push(chunk);
            chunk_valid_tokens.push(Self::feat_extract_output_length(chunk_size));
        }

        if tail > 0 {
            let start = num_full * chunk_size;
            let tail_mel = mel.narrow(1, start, tail)?;
            let pad_frames = chunk_size - tail;
            let device = mel.device();
            let pad = Tensor::zeros((mel.dims()[0], pad_frames), DType::F32, device)?;
            let padded = Tensor::cat(&[&tail_mel.to_dtype(DType::F32)?, &pad], 1)?.unsqueeze(0)?;
            chunk_mels.push(padded);
            chunk_valid_tokens.push(Self::feat_extract_output_length(tail));
        }

        // Stack chunks and cast to conv weight's native dtype.
        // batched: [num_chunks, 1, mel_bins, chunk_size]
        let refs: Vec<&Tensor> = chunk_mels.iter().collect();
        let compute_dtype = self.conv2d1.weight().dtype();
        let batched = Tensor::cat(&refs, 0)?.unsqueeze(1)?.to_dtype(compute_dtype)?;

        // Conv stem with GELU activations.
        let x = self.conv2d1.forward(&batched)?.gelu_erf()?;
        let x = self.conv2d2.forward(&x)?.gelu_erf()?;
        let x = self.conv2d3.forward(&x)?.gelu_erf()?;

        // Reshape: [b, c, f, t] -> [b, t, c*f]
        let (b, c, f, t) = x.dims4()?;
        let reshaped = x.permute((0, 3, 1, 2))?.contiguous()?.reshape((b, t, c * f))?;

        // Linear projection.
        let conv_out = self.conv_out.forward(&reshaped)?;

        // Add positional embedding, cast to match conv_out's dtype.
        let pos_emb =
            self.positional_embedding.narrow(0, 0, t)?.unsqueeze(0)?.to_dtype(conv_out.dtype())?;
        let conv_out = conv_out.broadcast_add(&pos_emb)?;
        // conv_out: [num_chunks, tokens_per_chunk, d_model]

        // Collect valid tokens from all chunks and concatenate.
        let mut all_valid: Vec<Tensor> = Vec::with_capacity(num_chunks);
        for (idx, &valid) in chunk_valid_tokens.iter().enumerate() {
            let chunk_tokens = conv_out.i(idx)?.narrow(0, 0, valid)?;
            all_valid.push(chunk_tokens);
        }
        // Concatenate: [total_tokens, d_model]
        let refs: Vec<&Tensor> = all_valid.iter().collect();
        let hidden = Tensor::cat(&refs, 0)?;
        let total_tokens = hidden.dims()[0];

        // Add batch dim: [1, total_tokens, d_model]
        let mut hidden = hidden.unsqueeze(0)?;

        // Build windowed attention mask (matches official Python implementation).
        // Each window covers `chunks_per_window` chunks worth of tokens.
        // Tokens within the same window attend to each other; cross-window is blocked.
        let tokens_per_chunk = Self::feat_extract_output_length(chunk_size);
        let chunks_per_window = self.config.n_window_infer / chunk_size;
        let window_size = tokens_per_chunk * chunks_per_window;
        let mask = build_windowed_mask(total_tokens, window_size, mel.device())?;

        // Transformer encoder with windowed attention.
        for layer in &self.layers {
            hidden = layer.forward(&hidden, Some(&mask))?;
        }

        // Output projection: LN → Linear → GELU → Linear
        let hidden = self.ln_post.forward(&hidden)?;
        let hidden = self.proj2.forward(&self.proj1.forward(&hidden)?.gelu_erf()?)?;

        // Remove batch dim: [num_tokens, output_dim]
        hidden.squeeze(0).map_err(Into::into)
    }

    /// Incremental encode: given the full mel spectrogram and an encoder cache,
    /// only re-encodes the current (incomplete) attention window. Completed windows
    /// are read from the cache and any newly completed windows are committed.
    ///
    /// Returns `[total_tokens, output_dim]` — the full encoder output
    /// (cached windows concatenated with the freshly computed current window),
    /// with the output projection (LN → proj1 → GELU → proj2) already applied.
    pub(crate) fn forward_incremental(
        &self,
        mel: &Tensor,
        cache: &mut EncoderCache,
    ) -> Result<Tensor> {
        let num_frames = mel.dims()[1];
        let chunk_size = self.config.n_window * 2;
        let tokens_per_chunk = Self::feat_extract_output_length(chunk_size);
        let chunks_per_window = self.config.n_window_infer / chunk_size;
        let window_size = tokens_per_chunk * chunks_per_window; // tokens per window

        // Count total chunks from the mel
        let num_full = num_frames / chunk_size;
        let tail = num_frames % chunk_size;
        let num_chunks = num_full + if tail > 0 { 1 } else { 0 };

        // How many complete windows are in the current mel?
        let total_full_chunks = num_full; // only full-sized chunks count toward complete windows
        let num_complete_windows = total_full_chunks / chunks_per_window;
        let committed_windows = cache.completed_windows.len();

        // Process any newly completed windows
        for win_idx in committed_windows..num_complete_windows {
            let start_chunk = win_idx * chunks_per_window;
            let window_output =
                self.encode_window(mel, chunk_size, start_chunk, chunks_per_window, window_size)?;
            cache.completed_windows.push(window_output);
        }
        cache.committed_chunks = num_complete_windows * chunks_per_window;

        // Now encode the current partial window (chunks after the last complete window)
        let partial_start_chunk = num_complete_windows * chunks_per_window;
        let partial_num_chunks = num_chunks - partial_start_chunk;

        if partial_num_chunks == 0 && !cache.completed_windows.is_empty() {
            // All chunks fit into complete windows; just concatenate cache
            let refs: Vec<&Tensor> = cache.completed_windows.iter().collect();
            return Tensor::cat(&refs, 0).map_err(Into::into);
        }

        // Encode the partial window
        let partial_output = if partial_num_chunks > 0 {
            // Gather chunk token counts for the partial window
            let mut chunk_valid: Vec<usize> = Vec::new();
            let mut chunk_mels: Vec<Tensor> = Vec::new();

            for i in 0..partial_num_chunks {
                let chunk_idx = partial_start_chunk + i;
                if chunk_idx < num_full {
                    let start = chunk_idx * chunk_size;
                    let chunk = mel.narrow(1, start, chunk_size)?.unsqueeze(0)?;
                    chunk_mels.push(chunk);
                    chunk_valid.push(tokens_per_chunk);
                } else if tail > 0 {
                    // Last partial chunk — pad to chunk_size
                    let start = num_full * chunk_size;
                    let tail_mel = mel.narrow(1, start, tail)?;
                    let pad_frames = chunk_size - tail;
                    let device = mel.device();
                    let pad = Tensor::zeros((mel.dims()[0], pad_frames), DType::F32, device)?;
                    let padded =
                        Tensor::cat(&[&tail_mel.to_dtype(DType::F32)?, &pad], 1)?.unsqueeze(0)?;
                    chunk_mels.push(padded);
                    chunk_valid.push(Self::feat_extract_output_length(tail));
                }
            }

            if chunk_mels.is_empty() {
                None
            } else {
                let refs: Vec<&Tensor> = chunk_mels.iter().collect();
                let compute_dtype = self.conv2d1.weight().dtype();
                let batched = Tensor::cat(&refs, 0)?.unsqueeze(1)?.to_dtype(compute_dtype)?;

                let x = self.conv2d1.forward(&batched)?.gelu_erf()?;
                let x = self.conv2d2.forward(&x)?.gelu_erf()?;
                let x = self.conv2d3.forward(&x)?.gelu_erf()?;

                let (b, c, f, t) = x.dims4()?;
                let reshaped = x.permute((0, 3, 1, 2))?.contiguous()?.reshape((b, t, c * f))?;
                let conv_out = self.conv_out.forward(&reshaped)?;

                let pos_emb = self
                    .positional_embedding
                    .narrow(0, 0, t)?
                    .unsqueeze(0)?
                    .to_dtype(conv_out.dtype())?;
                let conv_out = conv_out.broadcast_add(&pos_emb)?;

                // Collect valid tokens
                let mut all_valid: Vec<Tensor> = Vec::new();
                for (idx, &valid) in chunk_valid.iter().enumerate() {
                    let chunk_tokens = conv_out.i(idx)?.narrow(0, 0, valid)?;
                    all_valid.push(chunk_tokens);
                }
                let refs: Vec<&Tensor> = all_valid.iter().collect();
                let hidden = Tensor::cat(&refs, 0)?;
                let partial_total = hidden.dims()[0];

                // Run transformer with windowed mask (single window)
                let mut hidden = hidden.unsqueeze(0)?;
                let mask = build_windowed_mask(partial_total, window_size, mel.device())?;
                for layer in &self.layers {
                    hidden = layer.forward(&hidden, Some(&mask))?;
                }

                // Output projection
                let hidden = self.ln_post.forward(&hidden)?;
                let hidden = self.proj2.forward(&self.proj1.forward(&hidden)?.gelu_erf()?)?;
                Some(hidden.squeeze(0)?)
            }
        } else {
            None
        };

        // Concatenate cached + partial
        let mut all_parts: Vec<&Tensor> = cache.completed_windows.iter().collect();
        if let Some(ref partial) = partial_output {
            all_parts.push(partial);
        }

        if all_parts.is_empty() {
            anyhow::bail!("no audio tokens produced");
        }

        Tensor::cat(&all_parts, 0).map_err(Into::into)
    }

    /// Encode a single complete attention window: run conv stem, pos embed,
    /// transformer layers with windowed mask, and output projection.
    /// Returns [window_tokens, output_dim].
    fn encode_window(
        &self,
        mel: &Tensor,
        chunk_size: usize,
        start_chunk: usize,
        num_chunks: usize,
        window_size: usize,
    ) -> Result<Tensor> {
        let tokens_per_chunk = Self::feat_extract_output_length(chunk_size);

        let mut chunk_mels: Vec<Tensor> = Vec::with_capacity(num_chunks);
        for i in 0..num_chunks {
            let chunk_idx = start_chunk + i;
            let start = chunk_idx * chunk_size;
            let chunk = mel.narrow(1, start, chunk_size)?.unsqueeze(0)?;
            chunk_mels.push(chunk);
        }

        let refs: Vec<&Tensor> = chunk_mels.iter().collect();
        let compute_dtype = self.conv2d1.weight().dtype();
        let batched = Tensor::cat(&refs, 0)?.unsqueeze(1)?.to_dtype(compute_dtype)?;

        let x = self.conv2d1.forward(&batched)?.gelu_erf()?;
        let x = self.conv2d2.forward(&x)?.gelu_erf()?;
        let x = self.conv2d3.forward(&x)?.gelu_erf()?;

        let (b, c, f, t) = x.dims4()?;
        let reshaped = x.permute((0, 3, 1, 2))?.contiguous()?.reshape((b, t, c * f))?;
        let conv_out = self.conv_out.forward(&reshaped)?;

        let pos_emb =
            self.positional_embedding.narrow(0, 0, t)?.unsqueeze(0)?.to_dtype(conv_out.dtype())?;
        let conv_out = conv_out.broadcast_add(&pos_emb)?;

        // Collect valid tokens (all chunks are full-size, so all have tokens_per_chunk)
        let mut all_valid: Vec<Tensor> = Vec::with_capacity(num_chunks);
        for idx in 0..num_chunks {
            let chunk_tokens = conv_out.i(idx)?.narrow(0, 0, tokens_per_chunk)?;
            all_valid.push(chunk_tokens);
        }
        let refs: Vec<&Tensor> = all_valid.iter().collect();
        let hidden = Tensor::cat(&refs, 0)?;
        let total_tokens = hidden.dims()[0];

        // Transformer with windowed mask
        let mut hidden = hidden.unsqueeze(0)?;
        let mask = build_windowed_mask(total_tokens, window_size, mel.device())?;
        for layer in &self.layers {
            hidden = layer.forward(&hidden, Some(&mask))?;
        }

        // Output projection
        let hidden = self.ln_post.forward(&hidden)?;
        let hidden = self.proj2.forward(&self.proj1.forward(&hidden)?.gelu_erf()?)?;
        hidden.squeeze(0).map_err(Into::into)
    }

    pub(crate) fn feat_extract_output_length(input_frames: usize) -> usize {
        let after_conv = |len: usize| -> usize { (len - 1) / 2 + 1 };
        after_conv(after_conv(after_conv(input_frames)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use candle_core::Device;

    #[test]
    fn test_build_windowed_mask_single_window() {
        // All tokens fit in one window → all zeros (no masking)
        let mask = build_windowed_mask(10, 104, &Device::Cpu).unwrap();
        assert_eq!(mask.dims(), &[1, 1, 10, 10]);
        let data: Vec<f32> = mask.flatten_all().unwrap().to_vec1().unwrap();
        assert!(data.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_build_windowed_mask_two_windows() {
        // 6 tokens, window_size=3 → 2 windows: [0,1,2] and [3,4,5]
        let mask = build_windowed_mask(6, 3, &Device::Cpu).unwrap();
        assert_eq!(mask.dims(), &[1, 1, 6, 6]);
        let data: Vec<f32> =
            mask.squeeze(0).unwrap().squeeze(0).unwrap().flatten_all().unwrap().to_vec1().unwrap();

        // Within window 0 (rows 0-2, cols 0-2): all 0.0
        for i in 0..3 {
            for j in 0..3 {
                assert_eq!(data[i * 6 + j], 0.0, "({},{}) should be 0", i, j);
            }
        }
        // Cross-window (row 0, col 3): -inf
        for i in 0..3 {
            for j in 3..6 {
                assert_eq!(data[i * 6 + j], f32::NEG_INFINITY, "({},{}) should be -inf", i, j);
            }
        }
        // Within window 1 (rows 3-5, cols 3-5): all 0.0
        for i in 3..6 {
            for j in 3..6 {
                assert_eq!(data[i * 6 + j], 0.0, "({},{}) should be 0", i, j);
            }
        }
    }

    #[test]
    fn test_build_windowed_mask_zero_window() {
        // window_size=0 → no masking
        let mask = build_windowed_mask(5, 0, &Device::Cpu).unwrap();
        let data: Vec<f32> = mask.flatten_all().unwrap().to_vec1().unwrap();
        assert!(data.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_windowed_mask_dimensions_realistic() {
        // Realistic: 0.6B model with 8 chunks → 104 tokens, window_size=104
        // All fit in one window
        let mask = build_windowed_mask(104, 104, &Device::Cpu).unwrap();
        let data: Vec<f32> = mask.flatten_all().unwrap().to_vec1().unwrap();
        assert!(data.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_windowed_mask_dimensions_multi_window() {
        // 16 chunks → 208 tokens, window_size=104 → 2 windows
        let mask = build_windowed_mask(208, 104, &Device::Cpu).unwrap();
        assert_eq!(mask.dims(), &[1, 1, 208, 208]);
        let data: Vec<f32> =
            mask.squeeze(0).unwrap().squeeze(0).unwrap().flatten_all().unwrap().to_vec1().unwrap();
        // Token 0 (window 0) → token 103 (window 0): 0.0
        assert_eq!(data[0 * 208 + 103], 0.0);
        // Token 0 (window 0) → token 104 (window 1): -inf
        assert_eq!(data[0 * 208 + 104], f32::NEG_INFINITY);
        // Token 104 (window 1) → token 207 (window 1): 0.0
        assert_eq!(data[104 * 208 + 207], 0.0);
    }

    #[test]
    fn test_feat_extract_output_length() {
        // 3 stride-2 convs: (len-1)/2 + 1 applied 3 times
        // 100 → 50 → 25 → 13
        assert_eq!(AudioEncoder::feat_extract_output_length(100), 13);
        // 50 → 25 → 13 → 7
        assert_eq!(AudioEncoder::feat_extract_output_length(50), 7);
        // 1 → 1 → 1 → 1
        assert_eq!(AudioEncoder::feat_extract_output_length(1), 1);
    }
}
