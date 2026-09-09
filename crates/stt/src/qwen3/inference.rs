use anyhow::Context;
use candle_core::{DType, Device, Tensor};
use std::collections::HashMap;
use std::path::Path;
use std::sync::Mutex;
use tracing::{debug, info};

use super::config::AsrConfig;
use super::decoder::{compute_mrope_cos_sin, create_causal_mask, KvCache, TextDecoder};
use super::encoder::AudioEncoder;
use super::mel::MelExtractor;
use super::AsrError;

// Special token IDs
pub(crate) const IM_END_TOKEN_ID: i64 = 151645;
pub(crate) const ENDOFTEXT_TOKEN_ID: i64 = 151643;
// ASR-specific separator token (not in base Qwen3 tokenizer vocab, hence decodes to "")
pub(crate) const ASR_TEXT_SEP_TOKEN_ID: u32 = 151704;

pub(crate) const MEL_SAMPLE_RATE: u32 = 16000;
const N_FFT: usize = 400; // Whisper-compatible FFT window (25ms @ 16kHz)
const HOP_LENGTH: usize = 160; // Whisper-compatible hop size  (10ms @ 16kHz)

// Prompt structure token IDs (Qwen3 chat template)
pub(crate) const TOK_IM_START: i64 = 151644; // <|im_start|>
pub(crate) const TOK_SYSTEM: i64 = 8948; // "system"
pub(crate) const TOK_NEWLINE: i64 = 198; // "\n"
pub(crate) const TOK_IM_END: i64 = IM_END_TOKEN_ID; // 151645
pub(crate) const TOK_USER: i64 = 872; // "user"
pub(crate) const TOK_ASSISTANT: i64 = 77091; // "assistant"

/// Options controlling the transcription behaviour.
///
/// Construct via [`TranscribeOptions::default()`] and then mutate the fields
/// you need to override:
///
/// ```
/// # use stt::qwen3::TranscribeOptions;
/// let mut opts = TranscribeOptions::default();
/// opts.language = Some("english".into());
/// ```
#[non_exhaustive]
pub struct TranscribeOptions {
    /// Force a specific language (e.g. `"english"`). `None` enables auto-detection.
    pub language: Option<String>,
    /// Maximum number of new tokens to generate. Default: 512.
    pub max_new_tokens: usize,
}

impl Default for TranscribeOptions {
    fn default() -> Self {
        Self { language: None, max_new_tokens: 512 }
    }
}

impl TranscribeOptions {
    /// Set the maximum number of new tokens to generate.
    pub fn with_max_new_tokens(mut self, max_new_tokens: usize) -> Self {
        self.max_new_tokens = max_new_tokens;
        self
    }

    /// Force a specific language (e.g. `"english"`).
    pub fn with_language(mut self, language: impl Into<String>) -> Self {
        self.language = Some(language.into());
        self
    }
}

#[non_exhaustive]
pub struct TranscribeResult {
    pub text: String,
    pub language: String,
    pub raw_output: String,
}

pub(crate) struct AsrInferenceInner {
    pub(crate) audio_encoder: AudioEncoder,
    pub(crate) text_decoder: TextDecoder,
    pub(crate) mel_extractor: MelExtractor,
    pub(crate) tokenizer: tokenizers::Tokenizer,
    pub(crate) config: AsrConfig,
    pub(crate) device: Device,
}

// SAFETY: The raw pointers inside candle Metal tensors point to heap-allocated
// buffers managed via Arc, not thread-local storage. Transferring ownership to
// another thread is therefore safe. Concurrent access is prevented by the
// enclosing Mutex<AsrInferenceInner>.
unsafe impl Send for AsrInferenceInner {}

pub struct AsrInference {
    pub(crate) inner: Mutex<AsrInferenceInner>,
}
// AsrInference: Send + Sync automatically — Mutex<T>: Send+Sync when T: Send.

impl AsrInference {
    pub fn load(model_dir: &Path, device: Device) -> super::Result<Self> {
        info!("Loading config...");
        let config = AsrConfig::from_file(&model_dir.join("config.json"))
            .context("load config")
            .map_err(AsrError::ModelLoad)?;

        info!("Loading weights (this may take a moment)...");
        let mut weights = load_weights(model_dir, &device)
            .context("load weights")
            .map_err(AsrError::ModelLoad)?;
        info!("Loaded {} weight tensors", weights.len());

        maybe_convert_weights_for_cpu(&mut weights, &device);

        info!("Loading tokenizer...");
        let tokenizer = tokenizers::Tokenizer::from_file(model_dir.join("tokenizer.json"))
            .map_err(|e| anyhow::anyhow!("tokenizer load failed: {}", e))
            .map_err(AsrError::ModelLoad)?;

        info!("Model loaded successfully.");
        Self::build_engine(config, weights, tokenizer, device).map_err(AsrError::ModelLoad)
    }

    fn build_engine(
        config: AsrConfig,
        weights: HashMap<String, Tensor>,
        tokenizer: tokenizers::Tokenizer,
        device: Device,
    ) -> anyhow::Result<Self> {
        info!("Loading audio encoder...");
        let audio_encoder = AudioEncoder::load(
            &weights,
            "thinker.audio_tower",
            &config.thinker_config.audio_config,
            &device,
        )
        .context("load audio encoder")?;

        info!("Loading text decoder...");
        let text_decoder =
            TextDecoder::load(&weights, "thinker.model", &config.thinker_config.text_config)
                .context("load text decoder")?;

        let mel_extractor = MelExtractor::new(
            N_FFT,
            HOP_LENGTH,
            config.thinker_config.audio_config.num_mel_bins,
            MEL_SAMPLE_RATE,
        );

        let inner = AsrInferenceInner {
            audio_encoder,
            text_decoder,
            mel_extractor,
            tokenizer,
            config,
            device,
        };
        Ok(AsrInference { inner: Mutex::new(inner) })
    }

    /// Transcribe directly from pre-loaded 16 kHz f32 samples.
    pub fn transcribe_samples(
        &self,
        samples: &[f32],
        options: TranscribeOptions,
    ) -> super::Result<TranscribeResult> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| AsrError::Inference(anyhow::anyhow!("mutex poisoned")))?;
        inner.run_inference(samples, &options).map_err(AsrError::Inference)
    }
}

impl AsrInferenceInner {
    pub(crate) fn run_inference(
        &self,
        samples: &[f32],
        options: &TranscribeOptions,
    ) -> anyhow::Result<TranscribeResult> {
        let audio_embeds = self.encode_audio(samples)?;
        let generated_ids = self.generate(
            &audio_embeds,
            options.language.as_deref(),
            None, // no prefix
            options.max_new_tokens,
        )?;
        self.decode_result(&generated_ids, options.language.as_deref())
    }

    /// Extract mel spectrogram and run the audio encoder.
    /// Returns audio embeddings [num_audio_tokens, output_dim].
    pub(crate) fn encode_audio(&self, samples: &[f32]) -> anyhow::Result<Tensor> {
        let (mel_data, n_mels, n_frames) = self.mel_extractor.extract(samples)?;
        debug!("Mel: {}×{} frames", n_mels, n_frames);
        let mel = Tensor::from_vec(mel_data, (n_mels, n_frames), &self.device)?;
        let audio_embeds = self.audio_encoder.forward(&mel)?;
        info!("Audio tokens: {}", audio_embeds.dims()[0]);
        Ok(audio_embeds)
    }

    /// Run the full decoder pipeline: build prompt, prefill, generate tokens.
    ///
    /// `prefix_text`: optional text to prepend to the assistant turn (for streaming
    /// rollback). The prefix tokens are included in the prompt and the model
    /// generates continuation tokens after them.
    ///
    /// Returns the raw generated token IDs (not including prompt/prefix tokens).
    pub(crate) fn generate(
        &self,
        audio_embeds: &Tensor,
        language: Option<&str>,
        prefix_text: Option<&str>,
        max_new_tokens: usize,
    ) -> anyhow::Result<Vec<u32>> {
        let num_audio_tokens = audio_embeds.dims()[0];

        // Build prompt token IDs (with optional prefix)
        let (input_ids, audio_start_pos) =
            self.build_prompt(num_audio_tokens, language, prefix_text)?;
        let seq_len = input_ids.len();

        // Build embeddings, inject audio at the audio pad positions
        let before_ids: Vec<i64> = input_ids[..audio_start_pos].to_vec();
        let after_ids: Vec<i64> = input_ids[audio_start_pos + num_audio_tokens..].to_vec();

        let before_t =
            Tensor::from_vec(before_ids, (audio_start_pos,), &self.device)?.to_dtype(DType::U32)?;
        let after_t = Tensor::from_vec(
            after_ids,
            (input_ids.len() - audio_start_pos - num_audio_tokens,),
            &self.device,
        )?
        .to_dtype(DType::U32)?;

        let before_emb = self.text_decoder.embed(&before_t)?;
        let after_emb = self.text_decoder.embed(&after_t)?;
        let audio_emb = audio_embeds.to_dtype(before_emb.dtype())?;

        let hidden_states = Tensor::cat(&[&before_emb, &audio_emb, &after_emb], 0)?.unsqueeze(0)?;

        // Precompute MRoPE cos/sin table
        let text_cfg = &self.config.thinker_config.text_config;
        let total_positions = seq_len + max_new_tokens;
        let all_pos: Vec<i64> = (0..total_positions as i64).collect();
        let full_ids: [Vec<i64>; 3] = [all_pos.clone(), all_pos.clone(), all_pos.clone()];
        let (cos_table, sin_table) = compute_mrope_cos_sin(
            &full_ids,
            text_cfg.head_dim,
            text_cfg.rope_theta,
            &text_cfg.mrope_section(),
            text_cfg.mrope_interleaved(),
            &self.device,
        )?;

        let cos = cos_table.narrow(0, 0, seq_len)?;
        let sin = sin_table.narrow(0, 0, seq_len)?;

        // Prefill
        let mask = create_causal_mask(seq_len, 0, &self.device)?;
        let mut kv_cache = KvCache::new(text_cfg.num_hidden_layers);

        let logits =
            self.text_decoder.forward(&hidden_states, &cos, &sin, &mut kv_cache, Some(&mask))?;

        // Autoregressive generation
        let mut generated_ids: Vec<u32> = Vec::new();
        let eos_ids: &[i64] = &[ENDOFTEXT_TOKEN_ID, IM_END_TOKEN_ID];

        let mut next_logits = logits.narrow(1, seq_len - 1, 1)?.squeeze(1)?;
        let mut current_pos = seq_len;

        for step_idx in 0..max_new_tokens {
            let next_token = next_logits.argmax(1)?.to_vec1::<u32>()?[0];

            if tracing::enabled!(tracing::Level::DEBUG) {
                let logits_f32 = next_logits.to_dtype(candle_core::DType::F32)?;
                let logits_vec = logits_f32.to_vec2::<f32>()?[0].clone();
                let mut indexed: Vec<(f32, u32)> =
                    logits_vec.iter().enumerate().map(|(i, &v)| (v, i as u32)).collect();
                indexed.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
                let top10: Vec<String> = indexed
                    .iter()
                    .take(10)
                    .map(|(score, tok)| format!("{}({:.2})", tok, score))
                    .collect();
                debug!("  step {:2}: top10 = {}  chosen={}", step_idx, top10.join(" "), next_token);
            }

            if eos_ids.contains(&(next_token as i64)) {
                break;
            }

            generated_ids.push(next_token);

            let next_id_t = Tensor::from_vec(vec![next_token], (1,), &self.device)?;
            let next_emb = self.text_decoder.embed(&next_id_t)?.unsqueeze(0)?;

            let new_cos = cos_table.narrow(0, current_pos, 1)?;
            let new_sin = sin_table.narrow(0, current_pos, 1)?;

            let past_len = kv_cache.seq_len();
            let step_mask = create_causal_mask(1, past_len, &self.device)?;

            let step_logits = self.text_decoder.forward(
                &next_emb,
                &new_cos,
                &new_sin,
                &mut kv_cache,
                Some(&step_mask),
            )?;

            next_logits = step_logits.squeeze(1)?;
            current_pos += 1;
        }

        info!("Generated {} tokens", generated_ids.len());
        Ok(generated_ids)
    }

    /// Decode generated token IDs into a TranscribeResult.
    pub(crate) fn decode_result(
        &self,
        generated_ids: &[u32],
        language: Option<&str>,
    ) -> anyhow::Result<TranscribeResult> {
        let raw_text = self
            .tokenizer
            .decode(generated_ids, true)
            .map_err(|e| anyhow::anyhow!("decode: {}", e))?;

        let (lang, text) = if language.is_some() {
            ("forced".to_string(), raw_text.trim().to_string())
        } else if let Some(sep_pos) =
            generated_ids.iter().position(|&id| id == ASR_TEXT_SEP_TOKEN_ID)
        {
            let lang_ids: Vec<u32> = generated_ids[..sep_pos].to_vec();
            let text_ids: Vec<u32> = generated_ids[sep_pos + 1..].to_vec();
            let lang_raw = self
                .tokenizer
                .decode(&lang_ids, true)
                .map_err(|e| anyhow::anyhow!("decode lang: {}", e))?;
            let text_raw = self
                .tokenizer
                .decode(&text_ids, true)
                .map_err(|e| anyhow::anyhow!("decode text: {}", e))?;
            let lang = lang_raw.strip_prefix("language ").unwrap_or(&lang_raw).trim().to_string();
            (lang, text_raw.trim().to_string())
        } else {
            parse_asr_output(&raw_text, false)
        };
        Ok(TranscribeResult { text, language: lang, raw_output: raw_text })
    }

    /// Encode text into token IDs using the tokenizer.
    #[allow(dead_code)]
    pub(crate) fn tokenizer_encode(&self, text: &str) -> anyhow::Result<Vec<u32>> {
        let enc =
            self.tokenizer.encode(text, false).map_err(|e| anyhow::anyhow!("encode: {}", e))?;
        Ok(enc.get_ids().to_vec())
    }

    /// Decode token IDs into text using the tokenizer.
    pub(crate) fn tokenizer_decode(&self, ids: &[u32]) -> anyhow::Result<String> {
        self.tokenizer.decode(ids, true).map_err(|e| anyhow::anyhow!("decode: {}", e))
    }

    /// Build prompt token IDs with optional prefix text in the assistant turn.
    pub(crate) fn build_prompt(
        &self,
        num_audio_tokens: usize,
        language: Option<&str>,
        prefix_text: Option<&str>,
    ) -> anyhow::Result<(Vec<i64>, usize)> {
        let cfg = &self.config.thinker_config;
        let mut tokens: Vec<i64> = vec![
            TOK_IM_START,
            TOK_SYSTEM,
            TOK_NEWLINE,
            TOK_IM_END,
            TOK_NEWLINE,
            TOK_IM_START,
            TOK_USER,
            TOK_NEWLINE,
            cfg.audio_start_token_id,
        ];

        let audio_start_pos = tokens.len();
        tokens.extend(std::iter::repeat_n(cfg.audio_token_id, num_audio_tokens));

        tokens.extend_from_slice(&[cfg.audio_end_token_id, TOK_IM_END, TOK_NEWLINE, TOK_IM_START]);

        if let Some(lang) = language {
            tokens.push(TOK_ASSISTANT);
            tokens.push(TOK_NEWLINE);
            let prefix = format!("language {}", capitalize_first(lang));
            let enc = self
                .tokenizer
                .encode(prefix.as_str(), false)
                .map_err(|e| anyhow::anyhow!("encode: {}", e))?;
            tokens.extend(enc.get_ids().iter().map(|&id| id as i64));
        } else {
            tokens.push(TOK_ASSISTANT);
            tokens.push(TOK_NEWLINE);
        }

        // Append prefix text tokens (for streaming rollback)
        if let Some(prefix) = prefix_text {
            if !prefix.is_empty() {
                let enc = self
                    .tokenizer
                    .encode(prefix, false)
                    .map_err(|e| anyhow::anyhow!("encode prefix: {}", e))?;
                tokens.extend(enc.get_ids().iter().map(|&id| id as i64));
            }
        }

        Ok((tokens, audio_start_pos))
    }
}

fn parse_asr_output(raw: &str, language_forced: bool) -> (String, String) {
    if language_forced {
        return ("forced".to_string(), raw.trim().to_string());
    }
    let raw = raw.trim();
    if let Some(rest) = raw.strip_prefix("language ") {
        if let Some(pos) = rest.find("<asr_text>") {
            let lang = rest[..pos].trim().to_string();
            let text = rest[pos + "<asr_text>".len()..].trim().to_string();
            return (lang, text);
        }
        // Find first non-alphabetic char to split lang from text
        let mut lang_end = rest.len();
        for (i, c) in rest.char_indices() {
            if c.is_whitespace() || !c.is_alphabetic() {
                lang_end = i;
                break;
            }
        }
        if lang_end > 0 && lang_end < rest.len() {
            let lang = rest[..lang_end].to_string();
            let text = rest[lang_end..].trim().to_string();
            return (lang, text);
        }
    }
    ("unknown".to_string(), raw.to_string())
}

fn capitalize_first(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        None => String::new(),
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
    }
}

/// Convert BF16/F16 weight tensors to F32 when running on CPU.
///
/// candle's CPU backend does not support BF16/F16 matmul. Metal and CUDA
/// handle these natively, so this conversion only triggers on CPU.
fn maybe_convert_weights_for_cpu(weights: &mut HashMap<String, Tensor>, device: &Device) {
    if !device.is_cpu() {
        return;
    }
    let mut converted = 0usize;
    for (name, tensor) in weights.iter_mut() {
        match tensor.dtype() {
            DType::BF16 | DType::F16 => match tensor.to_dtype(DType::F32) {
                Ok(t) => {
                    *tensor = t;
                    converted += 1;
                }
                Err(e) => {
                    tracing::warn!("Failed to convert {name} to F32: {e}");
                }
            },
            _ => {}
        }
    }
    if converted > 0 {
        info!("Converted {converted} weight tensors from BF16/F16 to F32 for CPU inference");
    }
}

/// Load safetensors weights from a directory (single file or sharded).
fn load_weights(model_dir: &Path, device: &Device) -> anyhow::Result<HashMap<String, Tensor>> {
    // Check for sharded model
    let index_path = model_dir.join("model.safetensors.index.json");
    if index_path.exists() {
        let index_content = std::fs::read_to_string(&index_path)?;
        let index: serde_json::Value = serde_json::from_str(&index_content)?;
        let weight_map =
            index["weight_map"].as_object().ok_or_else(|| anyhow::anyhow!("invalid index.json"))?;

        let mut shard_files: std::collections::HashSet<String> = std::collections::HashSet::new();
        for v in weight_map.values() {
            if let Some(s) = v.as_str() {
                shard_files.insert(s.to_string());
            }
        }

        let mut all_weights = HashMap::new();
        for shard in shard_files {
            let shard_path = model_dir.join(&shard);
            let w = candle_core::safetensors::load(&shard_path, device)?;
            all_weights.extend(w);
        }
        return Ok(all_weights);
    }

    // Single file
    let model_path = model_dir.join("model.safetensors");
    candle_core::safetensors::load(&model_path, device).map_err(Into::into)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convert_bf16_to_f32_on_cpu() {
        let device = Device::Cpu;
        let t = Tensor::zeros((2, 3), DType::BF16, &device).unwrap();
        let mut weights = HashMap::from([("w".to_string(), t)]);
        maybe_convert_weights_for_cpu(&mut weights, &device);
        assert_eq!(weights["w"].dtype(), DType::F32);
    }

    #[test]
    fn test_convert_f16_to_f32_on_cpu() {
        let device = Device::Cpu;
        let t = Tensor::zeros((2, 3), DType::F16, &device).unwrap();
        let mut weights = HashMap::from([("w".to_string(), t)]);
        maybe_convert_weights_for_cpu(&mut weights, &device);
        assert_eq!(weights["w"].dtype(), DType::F32);
    }

    #[test]
    fn test_convert_preserves_f32() {
        let device = Device::Cpu;
        let t = Tensor::zeros((2, 3), DType::F32, &device).unwrap();
        let mut weights = HashMap::from([("w".to_string(), t)]);
        maybe_convert_weights_for_cpu(&mut weights, &device);
        assert_eq!(weights["w"].dtype(), DType::F32);
    }

    #[test]
    fn test_convert_mixed_dtypes() {
        let device = Device::Cpu;
        let bf16 = Tensor::zeros((2, 2), DType::BF16, &device).unwrap();
        let f32_ = Tensor::zeros((2, 2), DType::F32, &device).unwrap();
        let f16 = Tensor::zeros((2, 2), DType::F16, &device).unwrap();
        let mut weights = HashMap::from([
            ("a".to_string(), bf16),
            ("b".to_string(), f32_),
            ("c".to_string(), f16),
        ]);
        maybe_convert_weights_for_cpu(&mut weights, &device);
        assert_eq!(weights["a"].dtype(), DType::F32);
        assert_eq!(weights["b"].dtype(), DType::F32);
        assert_eq!(weights["c"].dtype(), DType::F32);
    }

    #[test]
    fn test_convert_preserves_values() {
        let device = Device::Cpu;
        let data = vec![1.0f32, 2.0, 3.0, 4.0];
        let t =
            Tensor::from_vec(data.clone(), (2, 2), &device).unwrap().to_dtype(DType::BF16).unwrap();
        let mut weights = HashMap::from([("w".to_string(), t)]);
        maybe_convert_weights_for_cpu(&mut weights, &device);
        let result = weights["w"].to_vec2::<f32>().unwrap();
        assert_eq!(result, vec![vec![1.0, 2.0], vec![3.0, 4.0]]);
    }

    #[test]
    fn test_convert_empty_map() {
        let device = Device::Cpu;
        let mut weights = HashMap::new();
        maybe_convert_weights_for_cpu(&mut weights, &device);
        assert!(weights.is_empty());
    }
}
