//! Qwen3-ASR, read through candle.
//!
//! Taken from alan890104/qwen3-asr-rs (MIT, see `LICENSE` beside this file)
//! at 0.2.2 and kept here because the crate pins a candle two versions older
//! than the one Whisper is read through, and two candles in one binary are
//! two `Device` types that cannot be handed to each other. What was dropped:
//! the WAV loader and its resampler, since the receiver resamples for
//! itself; and the downloader, which built its own HTTP client, where every
//! request this program makes goes out under one name (`crates/httpc`) and
//! the model is fetched through `hf_hub` like Whisper's is.
//!
//! The model is an audio encoder in front of a Qwen3 text decoder. It reads
//! the whole utterance at once rather than in 30 second windows, names the
//! language it heard, and is markedly better than Whisper of the same size
//! on noisy or accented speech at the cost of being an LLM: 0.6B parameters
//! is 1.7 GB of weights and it wants a GPU to keep up with a conversation.

mod config;
mod decoder;
mod encoder;
mod inference;
mod linear;
mod mel;
mod streaming;

pub use encoder::EncoderCache;
pub use inference::{AsrInference, TranscribeOptions, TranscribeResult};
pub use streaming::{StreamingOptions, StreamingState};

#[derive(Debug)]
pub enum AsrError {
    ModelLoad(anyhow::Error),
    AudioDecode(anyhow::Error),
    Inference(anyhow::Error),
}

impl std::fmt::Display for AsrError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ModelLoad(e) => write!(f, "model load failed: {e:#}"),
            Self::AudioDecode(e) => write!(f, "audio decode failed: {e:#}"),
            Self::Inference(e) => write!(f, "inference failed: {e:#}"),
        }
    }
}

impl std::error::Error for AsrError {}

pub type Result<T> = std::result::Result<T, AsrError>;

/// Build the Qwen3 tokenizer JSON from vocab.json, merges.txt, and tokenizer_config.json.
/// The added_tokens list is derived from tokenizer_config.json's added_tokens_decoder field,
/// so no special tokens need to be hardcoded here.
pub(crate) fn tokenizer_json(
    vocab: &str,
    merges: &str,
    tok_config: &str,
) -> anyhow::Result<Vec<u8>> {
    let vocab_val: serde_json::Value = serde_json::from_str(vocab)?;
    let merges_vec: Vec<&str> =
        merges.lines().filter(|l| !l.starts_with('#') && !l.is_empty()).collect();

    // Build added_tokens from tokenizer_config.json's added_tokens_decoder.
    let tok_cfg: serde_json::Value = serde_json::from_str(tok_config)?;
    let mut added_tokens: Vec<serde_json::Value> = Vec::new();
    if let Some(decoder_map) = tok_cfg["added_tokens_decoder"].as_object() {
        let mut entries: Vec<(u64, &serde_json::Value)> = decoder_map
            .iter()
            .filter_map(|(k, v)| k.parse::<u64>().ok().map(|id| (id, v)))
            .collect();
        entries.sort_by_key(|(id, _)| *id);
        for (id, v) in &entries {
            added_tokens.push(serde_json::json!({
                "id": id,
                "content": v["content"],
                "single_word": false,
                "lstrip": false,
                "rstrip": false,
                "normalized": false,
                "special": v["special"]
            }));
        }
    }
    let added_tokens = serde_json::Value::Array(added_tokens);

    let tokenizer_json = serde_json::json!({
        "version": "1.0",
        "truncation": null,
        "padding": null,
        "added_tokens": added_tokens,
        "normalizer": {"type": "NFC"},
        "pre_tokenizer": {
            "type": "Sequence",
            "pretokenizers": [
                {
                    "type": "Split",
                    "pattern": {"Regex": "(?i:'s|'t|'re|'ve|'m|'ll|'d)|[^\\r\\n\\p{L}\\p{N}]?\\p{L}+|\\p{N}| ?[^\\s\\p{L}\\p{N}]+[\\r\\n]*|\\s*[\\r\\n]+|\\s+(?!\\S)|\\s+"},
                    "behavior": "Isolated",
                    "invert": false
                },
                {
                    "type": "ByteLevel",
                    "add_prefix_space": false,
                    "trim_offsets": false,
                    "use_regex": false
                }
            ]
        },
        "post_processor": {
            "type": "ByteLevel",
            "add_prefix_space": false,
            "trim_offsets": false,
            "use_regex": false
        },
        "decoder": {
            "type": "ByteLevel",
            "add_prefix_space": false,
            "trim_offsets": false,
            "use_regex": false
        },
        "model": {
            "type": "BPE",
            "dropout": null,
            "unk_token": null,
            "continuing_subword_prefix": "",
            "end_of_word_suffix": "",
            "fuse_unk": false,
            "byte_fallback": false,
            "ignore_merges": false,
            "vocab": vocab_val,
            "merges": merges_vec
        }
    });

    serde_json::to_vec(&tokenizer_json).map_err(Into::into)
}
