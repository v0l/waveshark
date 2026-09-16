//! The phoneme language model that tells the rest of the network what the
//! sentence is doing.
//!
//! An ALBERT trained on phonemes rather than words (StyleTTS 2 calls it
//! PL-BERT). ALBERT's trick is that every layer is the same layer: one set of
//! weights read twelve times over, which is why a twelve layer encoder here
//! is a few megabytes. The embedding is narrow and is widened once on the way
//! in, for the same reason.
//!
//! Nothing masks: this model is handed one sentence at a time with no
//! padding, so the attention mask the reference builds is all ones and is
//! left out rather than carried through every call.

use super::Config;
use candle_core::{D, Result, Tensor};
use candle_nn::{Embedding, LayerNorm, Linear, Module, VarBuilder};

pub struct Albert {
    word: Embedding,
    position: Embedding,
    token_type: Embedding,
    embed_norm: LayerNorm,
    widen: Linear,
    layer: Layer,
    layers: usize,
}

impl Albert {
    pub fn load(cfg: &Config, vb: VarBuilder) -> Result<Self> {
        let p = &cfg.plbert;
        let e = vb.pp("embeddings");
        let group = vb.pp("encoder").pp("albert_layer_groups").pp(0).pp("albert_layers").pp(0);
        Ok(Self {
            word: candle_nn::embedding(cfg.n_token, p.embedding_size, e.pp("word_embeddings"))?,
            position: candle_nn::embedding(
                p.max_position_embeddings,
                p.embedding_size,
                e.pp("position_embeddings"),
            )?,
            token_type: candle_nn::embedding(2, p.embedding_size, e.pp("token_type_embeddings"))?,
            embed_norm: candle_nn::layer_norm(p.embedding_size, 1e-12, e.pp("LayerNorm"))?,
            widen: candle_nn::linear(
                p.embedding_size,
                p.hidden_size,
                vb.pp("encoder").pp("embedding_hidden_mapping_in"),
            )?,
            layer: Layer::load(p, group)?,
            layers: p.num_hidden_layers,
        })
    }

    /// `ids` is `[batch, time]`; the result is `[batch, time, hidden]`.
    pub fn forward(&self, ids: &Tensor) -> Result<Tensor> {
        let (_, len) = ids.dims2()?;
        let positions = Tensor::arange(0u32, len as u32, ids.device())?.unsqueeze(0)?;
        let types = positions.zeros_like()?;
        let x = self.word.forward(ids)?.broadcast_add(&self.position.forward(&positions)?)?;
        let x = x.broadcast_add(&self.token_type.forward(&types)?)?;
        let mut x = self.widen.forward(&self.embed_norm.forward(&x)?)?;
        for _ in 0..self.layers {
            x = self.layer.forward(&x)?;
        }
        Ok(x)
    }
}

struct Layer {
    query: Linear,
    key: Linear,
    value: Linear,
    dense: Linear,
    attn_norm: LayerNorm,
    ffn: Linear,
    ffn_out: Linear,
    out_norm: LayerNorm,
    heads: usize,
}

impl Layer {
    fn load(p: &super::PlBert, vb: VarBuilder) -> Result<Self> {
        let a = vb.pp("attention");
        let h = p.hidden_size;
        Ok(Self {
            query: candle_nn::linear(h, h, a.pp("query"))?,
            key: candle_nn::linear(h, h, a.pp("key"))?,
            value: candle_nn::linear(h, h, a.pp("value"))?,
            dense: candle_nn::linear(h, h, a.pp("dense"))?,
            attn_norm: candle_nn::layer_norm(h, 1e-12, a.pp("LayerNorm"))?,
            ffn: candle_nn::linear(h, p.intermediate_size, vb.pp("ffn"))?,
            ffn_out: candle_nn::linear(p.intermediate_size, h, vb.pp("ffn_output"))?,
            out_norm: candle_nn::layer_norm(h, 1e-12, vb.pp("full_layer_layer_norm"))?,
            heads: p.num_attention_heads,
        })
    }

    fn forward(&self, x: &Tensor) -> Result<Tensor> {
        let (b, t, h) = x.dims3()?;
        let per_head = h / self.heads;
        let split = |t2: Tensor| -> Result<Tensor> {
            t2.reshape((b, t, self.heads, per_head))?.transpose(1, 2)?.contiguous()
        };
        let q = split(self.query.forward(x)?)?;
        let k = split(self.key.forward(x)?)?;
        let v = split(self.value.forward(x)?)?;
        let scores = (q.matmul(&k.transpose(2, 3)?)? / (per_head as f64).sqrt())?;
        let weights = candle_nn::ops::softmax(&scores, D::Minus1)?;
        let context = weights.matmul(&v)?.transpose(1, 2)?.reshape((b, t, h))?;
        let attention = self.attn_norm.forward(&(self.dense.forward(&context)? + x)?)?;
        // Two linears with a Gaussian error unit between them, then the
        // residual. `gelu` is the tanh approximation, which is what this
        // checkpoint was trained with.
        let ffn = self.ffn_out.forward(&self.ffn.forward(&attention)?.gelu()?)?;
        self.out_norm.forward(&(ffn + attention)?)
    }
}
