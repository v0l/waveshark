use anyhow::Result;
use candle_core::{Device, Tensor};
use candle_nn::ops::softmax_last_dim;
use candle_nn::{Module, RmsNorm};
use std::collections::HashMap;

use super::config::TextDecoderConfig;
use super::linear::LinearW;

// ─── Weight helpers (dense / safetensors path) ────────────────────────────────

fn get_w(weights: &HashMap<String, Tensor>, name: &str) -> anyhow::Result<Tensor> {
    weights.get(name).cloned().ok_or_else(|| anyhow::anyhow!("weight not found: {}", name))
}

fn load_linear(weights: &HashMap<String, Tensor>, prefix: &str) -> Result<LinearW> {
    Ok(LinearW::new(get_w(weights, &format!("{}.weight", prefix))?, None))
}

fn load_rms_norm(weights: &HashMap<String, Tensor>, prefix: &str, eps: f64) -> Result<RmsNorm> {
    Ok(RmsNorm::new(get_w(weights, &format!("{}.weight", prefix))?, eps))
}

// ─── MRoPE ───────────────────────────────────────────────────────────────────

pub(crate) fn compute_mrope_cos_sin(
    position_ids: &[Vec<i64>; 3],
    head_dim: usize,
    rope_theta: f64,
    mrope_section: &[usize],
    interleaved: bool,
    device: &Device,
) -> Result<(Tensor, Tensor)> {
    let half_dim = head_dim / 2;
    let seq_len = position_ids[0].len();

    let inv_freq: Vec<f64> =
        (0..half_dim).map(|i| 1.0 / rope_theta.powf(2.0 * i as f64 / head_dim as f64)).collect();

    let dim_map = if interleaved {
        build_interleaved_dim_map(mrope_section, half_dim)
    } else {
        build_contiguous_dim_map(mrope_section, half_dim)
    };

    let mut cos_vals = vec![0.0f32; seq_len * head_dim];
    let mut sin_vals = vec![0.0f32; seq_len * head_dim];

    for t in 0..seq_len {
        for j in 0..half_dim {
            let dim = dim_map[j];
            let pos = position_ids[dim][t] as f64;
            let angle = pos * inv_freq[j];
            let c = angle.cos() as f32;
            let s = angle.sin() as f32;
            cos_vals[t * head_dim + j] = c;
            sin_vals[t * head_dim + j] = s;
            cos_vals[t * head_dim + j + half_dim] = c;
            sin_vals[t * head_dim + j + half_dim] = s;
        }
    }

    let cos = Tensor::from_vec(cos_vals, (seq_len, head_dim), device)?;
    let sin = Tensor::from_vec(sin_vals, (seq_len, head_dim), device)?;

    Ok((cos, sin))
}

fn build_contiguous_dim_map(sections: &[usize], total: usize) -> Vec<usize> {
    let mut map = Vec::with_capacity(total);
    for (dim, &size) in sections.iter().enumerate() {
        for _ in 0..size {
            if map.len() >= total {
                break;
            }
            map.push(dim);
        }
    }
    while map.len() < total {
        map.push(sections.len() - 1);
    }
    map
}

fn build_interleaved_dim_map(sections: &[usize], total: usize) -> Vec<usize> {
    let n_dims = sections.len();
    let mut map = Vec::with_capacity(total);
    let mut counts = vec![0usize; n_dims];

    while map.len() < total {
        let prev_len = map.len();
        for dim in 0..n_dims {
            if map.len() >= total {
                break;
            }
            if counts[dim] < sections[dim] {
                map.push(dim);
                counts[dim] += 1;
            }
        }
        if map.len() == prev_len {
            break;
        }
    }
    map
}

/// Apply rotary embeddings.
/// `cos` and `sin` must already be cast to `x.dtype()` and shaped [1, 1, seq, head_dim].
/// The cast and unsqueeze are done once upstream in `TextDecoder::forward`.
fn apply_rotary_emb(x: &Tensor, cos: &Tensor, sin: &Tensor) -> Result<Tensor> {
    let x_rotated = rotate_half(x)?;
    (x.broadcast_mul(cos)? + x_rotated.broadcast_mul(sin)?).map_err(Into::into)
}

fn rotate_half(x: &Tensor) -> Result<Tensor> {
    let last_dim = x.dims()[x.rank() - 1];
    let half = last_dim / 2;
    let x1 = x.narrow(x.rank() - 1, 0, half)?;
    let x2 = x.narrow(x.rank() - 1, half, half)?;
    let neg_x2 = (x2 * (-1.0f64))?;
    Tensor::cat(&[&neg_x2, &x1], x.rank() - 1).map_err(Into::into)
}

fn repeat_kv(x: Tensor, n_rep: usize) -> Result<Tensor> {
    if n_rep == 1 {
        return Ok(x);
    }
    let (bsz, num_kv, seq, hd) = x.dims4()?;
    x.unsqueeze(2)?
        .expand((bsz, num_kv, n_rep, seq, hd))?
        .reshape((bsz, num_kv * n_rep, seq, hd))
        .map_err(Into::into)
}

// ─── KV Cache ─────────────────────────────────────────────────────────────────

pub(crate) struct KvCache {
    layers: Vec<Option<(Tensor, Tensor)>>,
}

impl KvCache {
    pub(crate) fn new(num_layers: usize) -> Self {
        Self { layers: vec![None; num_layers] }
    }

    pub(crate) fn get(&self, layer: usize) -> Option<&(Tensor, Tensor)> {
        self.layers[layer].as_ref()
    }

    pub(crate) fn set(&mut self, layer: usize, cache: (Tensor, Tensor)) {
        self.layers[layer] = Some(cache);
    }

    pub(crate) fn seq_len(&self) -> usize {
        self.layers[0].as_ref().map(|(k, _)| k.dims()[2]).unwrap_or(0)
    }
}

// ─── Text Attention (GQA + QK-norm + MRoPE) ───────────────────────────────────

struct TextAttention {
    q_proj: LinearW,
    k_proj: LinearW,
    v_proj: LinearW,
    o_proj: LinearW,
    q_norm: RmsNorm,
    k_norm: RmsNorm,
    num_q_heads: usize,
    num_kv_heads: usize,
    head_dim: usize,
}

impl TextAttention {
    fn load(
        weights: &HashMap<String, Tensor>,
        prefix: &str,
        num_q_heads: usize,
        num_kv_heads: usize,
        head_dim: usize,
        rms_norm_eps: f64,
    ) -> Result<Self> {
        Ok(Self {
            q_proj: load_linear(weights, &format!("{}.q_proj", prefix))?,
            k_proj: load_linear(weights, &format!("{}.k_proj", prefix))?,
            v_proj: load_linear(weights, &format!("{}.v_proj", prefix))?,
            o_proj: load_linear(weights, &format!("{}.o_proj", prefix))?,
            q_norm: load_rms_norm(weights, &format!("{}.q_norm", prefix), rms_norm_eps)?,
            k_norm: load_rms_norm(weights, &format!("{}.k_norm", prefix), rms_norm_eps)?,
            num_q_heads,
            num_kv_heads,
            head_dim,
        })
    }

    fn forward(
        &self,
        x: &Tensor,
        cos: &Tensor,
        sin: &Tensor,
        kv_cache: Option<&(Tensor, Tensor)>,
        mask: Option<&Tensor>,
    ) -> Result<(Tensor, (Tensor, Tensor))> {
        let (bsz, seq_len, _) = x.dims3()?;
        let nqh = self.num_q_heads;
        let nkvh = self.num_kv_heads;
        let hd = self.head_dim;

        let q = self
            .q_proj
            .forward(x)?
            .reshape((bsz, seq_len, nqh, hd))?
            .transpose(1, 2)?
            .contiguous()?;
        let k = self
            .k_proj
            .forward(x)?
            .reshape((bsz, seq_len, nkvh, hd))?
            .transpose(1, 2)?
            .contiguous()?;
        let v = self
            .v_proj
            .forward(x)?
            .reshape((bsz, seq_len, nkvh, hd))?
            .transpose(1, 2)?
            .contiguous()?;

        // QK normalization (applied per-head, on last dim)
        let q = self.q_norm.forward(&q)?;
        let k = self.k_norm.forward(&k)?;

        // Apply RoPE
        let q = apply_rotary_emb(&q, cos, sin)?;
        let k = apply_rotary_emb(&k, cos, sin)?;

        // Append to KV cache
        let (k, v) = if let Some((past_k, past_v)) = kv_cache {
            let k = Tensor::cat(&[past_k, &k], 2)?;
            let v = Tensor::cat(&[past_v, &v], 2)?;
            (k, v)
        } else {
            (k, v)
        };

        let n_rep = nqh / nkvh;
        let new_cache = (k.clone(), v.clone());
        let k = repeat_kv(k, n_rep)?;
        let v = repeat_kv(v, n_rep)?;

        // Attention
        let scale = (hd as f64).sqrt();
        let mut attn: Tensor = (q.matmul(&k.transpose(2, 3)?.contiguous()?)? * (1.0 / scale))?;

        if let Some(m) = mask {
            // Cast mask to attn's dtype (mask is F32, attn may be BF16).
            attn = attn.broadcast_add(&m.to_dtype(attn.dtype())?)?;
        }

        let attn = softmax_last_dim(&attn)?;
        let out = attn.matmul(&v)?;
        let out = out.transpose(1, 2)?.contiguous()?.reshape((bsz, seq_len, nqh * hd))?;
        let out = self.o_proj.forward(&out)?;

        Ok((out, new_cache))
    }
}

// ─── SwiGLU MLP ───────────────────────────────────────────────────────────────

struct TextMlp {
    gate_proj: LinearW,
    up_proj: LinearW,
    down_proj: LinearW,
}

impl TextMlp {
    fn load(weights: &HashMap<String, Tensor>, prefix: &str) -> Result<Self> {
        Ok(Self {
            gate_proj: load_linear(weights, &format!("{}.gate_proj", prefix))?,
            up_proj: load_linear(weights, &format!("{}.up_proj", prefix))?,
            down_proj: load_linear(weights, &format!("{}.down_proj", prefix))?,
        })
    }

    fn forward(&self, x: &Tensor) -> Result<Tensor> {
        let gate = self.gate_proj.forward(x)?.silu()?;
        let up = self.up_proj.forward(x)?;
        self.down_proj.forward(&(gate * up)?).map_err(Into::into)
    }
}

// ─── Text Decoder Layer ───────────────────────────────────────────────────────

struct TextDecoderLayer {
    input_layernorm: RmsNorm,
    self_attn: TextAttention,
    post_attention_layernorm: RmsNorm,
    mlp: TextMlp,
}

impl TextDecoderLayer {
    fn load(
        weights: &HashMap<String, Tensor>,
        prefix: &str,
        num_q_heads: usize,
        num_kv_heads: usize,
        head_dim: usize,
        rms_norm_eps: f64,
    ) -> Result<Self> {
        Ok(Self {
            input_layernorm: load_rms_norm(
                weights,
                &format!("{}.input_layernorm", prefix),
                rms_norm_eps,
            )?,
            self_attn: TextAttention::load(
                weights,
                &format!("{}.self_attn", prefix),
                num_q_heads,
                num_kv_heads,
                head_dim,
                rms_norm_eps,
            )?,
            post_attention_layernorm: load_rms_norm(
                weights,
                &format!("{}.post_attention_layernorm", prefix),
                rms_norm_eps,
            )?,
            mlp: TextMlp::load(weights, &format!("{}.mlp", prefix))?,
        })
    }

    fn forward(
        &self,
        x: &Tensor,
        cos: &Tensor,
        sin: &Tensor,
        kv_cache: Option<&(Tensor, Tensor)>,
        mask: Option<&Tensor>,
    ) -> Result<(Tensor, (Tensor, Tensor))> {
        // Pre-norm + attention + residual
        let h = self.input_layernorm.forward(x)?;
        let (h, new_cache) = self.self_attn.forward(&h, cos, sin, kv_cache, mask)?;
        let x = (x + &h)?;

        // Pre-norm + MLP + residual
        let h = self.post_attention_layernorm.forward(&x)?;
        let h = self.mlp.forward(&h)?;
        let out = (&x + &h)?;

        Ok((out, new_cache))
    }
}

// ─── Causal mask ─────────────────────────────────────────────────────────────

pub(crate) fn create_causal_mask(
    seq_len: usize,
    past_len: usize,
    device: &Device,
) -> Result<Tensor> {
    let total_len = past_len + seq_len;
    let mut mask_data = vec![0.0f32; seq_len * total_len];
    for i in 0..seq_len {
        for j in 0..total_len {
            if j > past_len + i {
                mask_data[i * total_len + j] = f32::NEG_INFINITY;
            }
        }
    }
    Tensor::from_vec(mask_data, (1, 1, seq_len, total_len), device).map_err(Into::into)
}

// ─── Text Decoder ────────────────────────────────────────────────────────────

pub(crate) struct TextDecoder {
    embed_tokens: Tensor,
    layers: Vec<TextDecoderLayer>,
    norm: RmsNorm,
    lm_head: LinearW,
}

impl TextDecoder {
    pub(crate) fn load(
        weights: &HashMap<String, Tensor>,
        prefix: &str,
        config: &TextDecoderConfig,
    ) -> Result<Self> {
        let embed_tokens = get_w(weights, &format!("{}.embed_tokens.weight", prefix))?;

        let mut layers = Vec::new();
        for i in 0..config.num_hidden_layers {
            let layer = TextDecoderLayer::load(
                weights,
                &format!("{}.layers.{}", prefix, i),
                config.num_attention_heads,
                config.num_key_value_heads,
                config.head_dim,
                config.rms_norm_eps,
            )?;
            layers.push(layer);
        }

        let norm = load_rms_norm(weights, &format!("{}.norm", prefix), config.rms_norm_eps)?;
        // Tie lm_head weights to embed_tokens (weight tying).
        let lm_head = LinearW::new(embed_tokens.clone(), None);

        Ok(Self { embed_tokens, layers, norm, lm_head })
    }

    /// Look up token embeddings. Returns the native dtype of the embedding table (BF16).
    pub(crate) fn embed(&self, input_ids: &Tensor) -> Result<Tensor> {
        self.embed_tokens.index_select(input_ids, 0).map_err(Into::into)
    }

    pub(crate) fn forward(
        &self,
        hidden_states: &Tensor, // [1, seq, hidden]
        cos: &Tensor,           // [seq, head_dim], F32
        sin: &Tensor,           // [seq, head_dim], F32
        kv_cache: &mut KvCache,
        mask: Option<&Tensor>,
    ) -> Result<Tensor> {
        let mut hidden = hidden_states.clone();

        // Cast cos/sin to compute dtype and unsqueeze to [1, 1, seq, head_dim] once.
        let compute_dtype = hidden.dtype();
        let cos4d = cos.to_dtype(compute_dtype)?.unsqueeze(0)?.unsqueeze(0)?;
        let sin4d = sin.to_dtype(compute_dtype)?.unsqueeze(0)?.unsqueeze(0)?;

        for (i, layer) in self.layers.iter().enumerate() {
            let cache = kv_cache.get(i);
            let (h, new_cache) = layer.forward(&hidden, &cos4d, &sin4d, cache, mask)?;
            kv_cache.set(i, new_cache);
            hidden = h;
        }

        // LN → lm_head (weight-tied to embed_tokens).
        // LinearW::Dense: candle_nn::Linear::forward has 3D fast path.
        // LinearW::Quant: QMatMul::forward uses broadcast_matmul (also 3D-safe).
        self.lm_head.forward(&self.norm.forward(&hidden)?).map_err(Into::into)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use candle_core::Device;

    #[test]
    fn test_causal_mask_prefill() {
        let device = Device::Cpu;
        let mask = create_causal_mask(3, 0, &device).unwrap();
        assert_eq!(mask.dims(), &[1, 1, 3, 3]);
        let data: Vec<f32> =
            mask.squeeze(0).unwrap().squeeze(0).unwrap().flatten_all().unwrap().to_vec1().unwrap();
        // Row 0: [0, -inf, -inf]
        assert_eq!(data[0], 0.0);
        assert_eq!(data[1], f32::NEG_INFINITY);
        assert_eq!(data[2], f32::NEG_INFINITY);
        // Row 1: [0, 0, -inf]
        assert_eq!(data[3], 0.0);
        assert_eq!(data[4], 0.0);
        assert_eq!(data[5], f32::NEG_INFINITY);
        // Row 2: [0, 0, 0]
        assert_eq!(data[6], 0.0);
        assert_eq!(data[7], 0.0);
        assert_eq!(data[8], 0.0);
    }

    #[test]
    fn test_causal_mask_decode_step() {
        let device = Device::Cpu;
        // seq=1, past=5 → total_len=6; j > 5+0 never triggers → all zeros
        let mask = create_causal_mask(1, 5, &device).unwrap();
        assert_eq!(mask.dims(), &[1, 1, 1, 6]);
        let data: Vec<f32> = mask.flatten_all().unwrap().to_vec1().unwrap();
        assert!(data.iter().all(|&v| v == 0.0), "decode-step mask should be all zeros");
    }

    #[test]
    fn test_build_contiguous_dim_map() {
        let sections = [2usize, 3, 3];
        let map = build_contiguous_dim_map(&sections, 8);
        assert_eq!(map, vec![0, 0, 1, 1, 1, 2, 2, 2]);
    }

    #[test]
    fn test_build_interleaved_dim_map() {
        let sections = [2usize, 2, 2];
        let map = build_interleaved_dim_map(&sections, 6);
        assert_eq!(map, vec![0, 1, 2, 0, 1, 2]);
    }

    #[test]
    fn test_compute_mrope_cos_sin_zero_positions() {
        let device = Device::Cpu;
        let seq_len = 4;
        let head_dim = 8;
        let mrope_section = [2usize, 3, 3];
        // All positions = 0 → angle = 0 → cos = 1, sin = 0 everywhere
        let position_ids: [Vec<i64>; 3] =
            [vec![0i64; seq_len], vec![0i64; seq_len], vec![0i64; seq_len]];
        let (cos, sin) =
            compute_mrope_cos_sin(&position_ids, head_dim, 10000.0, &mrope_section, false, &device)
                .unwrap();
        assert_eq!(cos.dims(), &[seq_len, head_dim]);
        assert_eq!(sin.dims(), &[seq_len, head_dim]);
        let cos_data: Vec<f32> = cos.flatten_all().unwrap().to_vec1().unwrap();
        let sin_data: Vec<f32> = sin.flatten_all().unwrap().to_vec1().unwrap();
        for (i, (&c, &s)) in cos_data.iter().zip(sin_data.iter()).enumerate() {
            assert!((c - 1.0).abs() < 1e-6, "cos[{}] should be 1.0, got {}", i, c);
            assert!(s.abs() < 1e-6, "sin[{}] should be 0.0, got {}", i, s);
        }
    }

    #[test]
    fn test_compute_mrope_cos_sin_pythagorean() {
        // cos²+sin² must equal 1 at every (t, j)
        let device = Device::Cpu;
        let head_dim = 8;
        let mrope_section = [2usize, 2, 2];
        let position_ids: [Vec<i64>; 3] = [vec![0, 1, 2], vec![0, 1, 2], vec![0, 1, 2]];
        let (cos, sin) =
            compute_mrope_cos_sin(&position_ids, head_dim, 10000.0, &mrope_section, false, &device)
                .unwrap();
        let cos_data: Vec<f32> = cos.flatten_all().unwrap().to_vec1().unwrap();
        let sin_data: Vec<f32> = sin.flatten_all().unwrap().to_vec1().unwrap();
        for (i, (&c, &s)) in cos_data.iter().zip(sin_data.iter()).enumerate() {
            let r = c * c + s * s;
            assert!((r - 1.0).abs() < 1e-5, "cos²+sin² at [{}] = {}, want 1", i, r);
        }
    }

    #[test]
    fn test_rotate_half() {
        let device = Device::Cpu;
        // x = [[1, 2, 3, 4]] → rotate_half → [[-3, -4, 1, 2]]
        let x = Tensor::from_vec(vec![1.0f32, 2.0, 3.0, 4.0], (1, 4), &device).unwrap();
        let rotated = rotate_half(&x).unwrap();
        let data: Vec<f32> = rotated.flatten_all().unwrap().to_vec1().unwrap();
        assert_eq!(data, vec![-3.0f32, -4.0, 1.0, 2.0]);
    }
}
