use anyhow::Result;
use candle_core::Tensor;
use tracing::{debug, info};

use super::encoder::EncoderCache;
use super::inference::{AsrInference, AsrInferenceInner, TranscribeResult};
use super::AsrError;

/// Options for streaming transcription.
#[non_exhaustive]
pub struct StreamingOptions {
    /// Force a specific language (e.g. `"english"`). `None` enables auto-detection.
    pub language: Option<String>,
    /// Audio chunk size in seconds. Default: 2.0.
    pub chunk_size_sec: f32,
    /// Number of initial chunks before prefix conditioning kicks in. Default: 2.
    pub unfixed_chunk_num: usize,
    /// Number of tokens to roll back from the end when building prefix. Default: 5.
    pub unfixed_token_num: usize,
    /// Maximum new tokens per streaming step. Default: 32.
    pub max_new_tokens_streaming: usize,
    /// Maximum new tokens for the final flush. Default: 512.
    pub max_new_tokens_final: usize,
    /// Optional context text from a previous session (e.g. last ~200 chars of prior
    /// transcript). Used as the prefix during cold-start chunks to guide vocabulary
    /// and style consistency across session resets. After cold start, the normal
    /// rollback mechanism takes over with real transcription tokens.
    pub initial_text: Option<String>,
}

impl Default for StreamingOptions {
    fn default() -> Self {
        Self {
            language: None,
            chunk_size_sec: 2.0,
            unfixed_chunk_num: 2,
            unfixed_token_num: 5,
            max_new_tokens_streaming: 32,
            max_new_tokens_final: 512,
            initial_text: None,
        }
    }
}

impl StreamingOptions {
    /// Set the audio chunk size in seconds.
    pub fn with_chunk_size_sec(mut self, sec: f32) -> Self {
        self.chunk_size_sec = sec;
        self
    }

    /// Set the number of initial cold-start chunks.
    pub fn with_unfixed_chunk_num(mut self, n: usize) -> Self {
        self.unfixed_chunk_num = n;
        self
    }

    /// Set the number of tokens to roll back from the end.
    pub fn with_unfixed_token_num(mut self, n: usize) -> Self {
        self.unfixed_token_num = n;
        self
    }

    /// Set the maximum new tokens per streaming step.
    pub fn with_max_new_tokens_streaming(mut self, n: usize) -> Self {
        self.max_new_tokens_streaming = n;
        self
    }

    /// Set the maximum new tokens for the final flush.
    pub fn with_max_new_tokens_final(mut self, n: usize) -> Self {
        self.max_new_tokens_final = n;
        self
    }

    /// Force a specific language.
    pub fn with_language(mut self, language: impl Into<String>) -> Self {
        self.language = Some(language.into());
        self
    }

    /// Provide context text from a previous session for cross-session continuity.
    ///
    /// During cold-start chunks (before real rollback tokens are available),
    /// this text is used as the decoder prefix, guiding the model's vocabulary
    /// and style to be consistent with the prior transcript. After cold start,
    /// the normal rollback mechanism takes over automatically.
    pub fn with_initial_text(mut self, text: impl Into<String>) -> Self {
        let t = text.into();
        self.initial_text = if t.is_empty() { None } else { Some(t) };
        self
    }
}

/// Opaque streaming transcription state.
///
/// Created by [`AsrInference::init_streaming`]. Feed audio chunks via
/// [`AsrInference::feed_audio`] and finalize with [`AsrInference::finish_streaming`].
pub struct StreamingState {
    /// Unconsumed audio samples (partial chunk)
    buffer: Vec<f32>,
    /// All audio samples accumulated from the start
    audio_accum: Vec<f32>,
    /// Number of samples per chunk
    chunk_size_samples: usize,
    /// Number of chunks processed so far
    chunk_id: usize,
    /// Raw generated token IDs from the last generation (full sequence including prefix)
    raw_token_ids: Vec<u32>,
    /// Streaming options
    options: StreamingOptions,
    /// Detected or forced language
    language: String,
    /// Current best transcription text
    text: String,
    /// Encoder cache for incremental encoding (avoids re-encoding completed windows)
    encoder_cache: EncoderCache,
}

impl AsrInference {
    /// Initialize a new streaming transcription session.
    pub fn init_streaming(&self, options: StreamingOptions) -> StreamingState {
        let chunk_size_samples = (options.chunk_size_sec * 16000.0) as usize;
        StreamingState {
            buffer: Vec::new(),
            audio_accum: Vec::new(),
            chunk_size_samples,
            chunk_id: 0,
            raw_token_ids: Vec::new(),
            options,
            language: String::new(),
            text: String::new(),
            encoder_cache: EncoderCache::new(),
        }
    }

    /// Feed audio samples (16 kHz f32) into the streaming state.
    ///
    /// When enough samples accumulate to form a chunk, runs inference on all
    /// accumulated audio and returns the latest transcription. Returns `None`
    /// if the buffer hasn't accumulated a full chunk yet.
    pub fn feed_audio(
        &self,
        state: &mut StreamingState,
        samples: &[f32],
    ) -> super::Result<Option<TranscribeResult>> {
        state.buffer.extend_from_slice(samples);

        if !try_drain_chunk(state) {
            return Ok(None);
        }

        // Acquire lock once for the entire step
        let inner = self
            .inner
            .lock()
            .map_err(|_| AsrError::Inference(anyhow::anyhow!("mutex poisoned")))?;

        let result = run_streaming_step(&inner, state).map_err(AsrError::Inference)?;

        Ok(Some(result))
    }

    /// Finish the streaming session: flush any remaining buffered audio and run
    /// a final inference pass with higher token budget.
    ///
    /// Returns the final transcription result.
    pub fn finish_streaming(&self, state: &mut StreamingState) -> super::Result<TranscribeResult> {
        if !flush_remaining_buffer(state) {
            return Ok(TranscribeResult {
                text: String::new(),
                language: String::new(),
                raw_output: String::new(),
            });
        }

        // Acquire lock once for the entire final step
        let inner = self
            .inner
            .lock()
            .map_err(|_| AsrError::Inference(anyhow::anyhow!("mutex poisoned")))?;

        // Use incremental encoder for efficiency
        let audio_embeds = encode_audio_incremental(&inner, state).map_err(AsrError::Inference)?;

        let prefix = build_prefix(&inner, state);

        let generated_ids = inner
            .generate(
                &audio_embeds,
                state.options.language.as_deref(),
                prefix.as_deref(),
                state.options.max_new_tokens_final,
            )
            .map_err(AsrError::Inference)?;

        let full_ids = combine_prefix_and_generated(state, &prefix, &generated_ids);

        let result = inner
            .decode_result(&full_ids, state.options.language.as_deref())
            .map_err(AsrError::Inference)?;

        state.text = result.text.clone();
        state.language = result.language.clone();
        state.raw_token_ids = full_ids;

        Ok(result)
    }
}

/// Encode audio using incremental encoder (leverages window cache).
fn encode_audio_incremental(
    inner: &AsrInferenceInner,
    state: &mut StreamingState,
) -> Result<Tensor> {
    let (mel_data, n_mels, n_frames) = inner.mel_extractor.extract(&state.audio_accum)?;
    debug!("Mel: {}×{} frames (incremental)", n_mels, n_frames);
    let mel = Tensor::from_vec(mel_data, (n_mels, n_frames), &inner.device)?;
    let audio_embeds = inner.audio_encoder.forward_incremental(&mel, &mut state.encoder_cache)?;
    info!(
        "Audio tokens (incremental): {} (cached: {})",
        audio_embeds.dims()[0],
        state.encoder_cache.cached_tokens()
    );
    Ok(audio_embeds)
}

/// Run one streaming inference step (called with lock already held).
fn run_streaming_step(
    inner: &AsrInferenceInner,
    state: &mut StreamingState,
) -> Result<TranscribeResult> {
    // Use incremental encoder to avoid re-encoding completed windows
    let audio_embeds = encode_audio_incremental(inner, state)?;

    let prefix = build_prefix(inner, state);

    info!(
        "Streaming step: chunk_id={}, accum_samples={}, prefix={:?}",
        state.chunk_id,
        state.audio_accum.len(),
        prefix.as_deref().unwrap_or("(none)"),
    );

    let generated_ids = inner.generate(
        &audio_embeds,
        state.options.language.as_deref(),
        prefix.as_deref(),
        state.options.max_new_tokens_streaming,
    )?;

    let full_ids = combine_prefix_and_generated(state, &prefix, &generated_ids);

    let result = inner.decode_result(&full_ids, state.options.language.as_deref())?;

    // Update state
    state.raw_token_ids = full_ids;
    state.text = result.text.clone();
    state.language = result.language.clone();

    Ok(result)
}

/// Compute the prefix token IDs for rollback conditioning (pure logic, no tokenizer).
///
/// Returns `None` during cold start, when no tokens exist, or when all tokens
/// would be rolled back. Otherwise returns the slice of tokens to keep.
pub(crate) fn compute_prefix_ids(state: &StreamingState) -> Option<&[u32]> {
    if state.chunk_id <= state.options.unfixed_chunk_num {
        return None;
    }
    if state.raw_token_ids.is_empty() {
        return None;
    }
    let keep = state.raw_token_ids.len().saturating_sub(state.options.unfixed_token_num);
    if keep == 0 {
        return None;
    }
    Some(&state.raw_token_ids[..keep])
}

/// Build the prefix text for the current streaming step using the rollback strategy.
///
/// For the first `unfixed_chunk_num` chunks (cold start), uses `initial_text`
/// from options (if set) to provide cross-session context. After cold start,
/// we take the previous raw token output and drop the last `unfixed_token_num`
/// tokens, then decode the remaining to text.
pub(crate) fn build_prefix(inner: &AsrInferenceInner, state: &StreamingState) -> Option<String> {
    // During cold start, use initial_text for cross-session context if available.
    if state.chunk_id <= state.options.unfixed_chunk_num {
        return state.options.initial_text.clone();
    }

    let prefix_ids = compute_prefix_ids(state)?;
    let prefix_text = inner.tokenizer_decode(prefix_ids).ok()?;

    if prefix_text.is_empty() {
        None
    } else {
        Some(prefix_text)
    }
}

/// Try to drain one full chunk from the buffer into the audio accumulator.
///
/// Returns `true` if a chunk was drained (and `chunk_id` incremented).
fn try_drain_chunk(state: &mut StreamingState) -> bool {
    if state.buffer.len() < state.chunk_size_samples {
        return false;
    }
    let chunk: Vec<f32> = state.buffer.drain(..state.chunk_size_samples).collect();
    state.audio_accum.extend_from_slice(&chunk);
    state.chunk_id += 1;
    true
}

/// Flush any remaining buffered audio into the accumulator.
///
/// Returns `true` if the accumulator is non-empty (i.e. there is audio to process).
fn flush_remaining_buffer(state: &mut StreamingState) -> bool {
    if !state.buffer.is_empty() {
        let remaining: Vec<f32> = state.buffer.drain(..).collect();
        state.audio_accum.extend_from_slice(&remaining);
        state.chunk_id += 1;
    }
    !state.audio_accum.is_empty()
}

/// Combine prefix token IDs (from rollback) with newly generated token IDs.
pub(crate) fn combine_prefix_and_generated(
    state: &StreamingState,
    prefix: &Option<String>,
    generated_ids: &[u32],
) -> Vec<u32> {
    if prefix.is_none() || state.raw_token_ids.is_empty() {
        return generated_ids.to_vec();
    }

    let keep = state.raw_token_ids.len().saturating_sub(state.options.unfixed_token_num);
    if keep == 0 {
        return generated_ids.to_vec();
    }

    let mut full = Vec::with_capacity(keep + generated_ids.len());
    full.extend_from_slice(&state.raw_token_ids[..keep]);
    full.extend_from_slice(generated_ids);
    full
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper to create a StreamingState with controllable internals for testing.
    fn make_state(
        chunk_size_sec: f32,
        unfixed_chunk_num: usize,
        unfixed_token_num: usize,
    ) -> StreamingState {
        let options = StreamingOptions {
            language: None,
            chunk_size_sec,
            unfixed_chunk_num,
            unfixed_token_num,
            max_new_tokens_streaming: 32,
            max_new_tokens_final: 512,
            initial_text: None,
        };
        let chunk_size_samples = (chunk_size_sec * 16000.0) as usize;
        StreamingState {
            buffer: Vec::new(),
            audio_accum: Vec::new(),
            chunk_size_samples,
            chunk_id: 0,
            raw_token_ids: Vec::new(),
            options,
            language: String::new(),
            text: String::new(),
            encoder_cache: EncoderCache::new(),
        }
    }

    // ── StreamingOptions defaults ────────────────────────────────────────

    #[test]
    fn test_streaming_options_default() {
        let opts = StreamingOptions::default();
        assert_eq!(opts.chunk_size_sec, 2.0);
        assert_eq!(opts.unfixed_chunk_num, 2);
        assert_eq!(opts.unfixed_token_num, 5);
        assert_eq!(opts.max_new_tokens_streaming, 32);
        assert_eq!(opts.max_new_tokens_final, 512);
        assert!(opts.language.is_none());
        assert!(opts.initial_text.is_none());
    }

    // ── chunk_size_samples calculation ───────────────────────────────────

    #[test]
    fn test_chunk_size_samples_2sec() {
        let state = make_state(2.0, 2, 5);
        assert_eq!(state.chunk_size_samples, 32000);
    }

    #[test]
    fn test_chunk_size_samples_1sec() {
        let state = make_state(1.0, 2, 5);
        assert_eq!(state.chunk_size_samples, 16000);
    }

    #[test]
    fn test_chunk_size_samples_half_sec() {
        let state = make_state(0.5, 2, 5);
        assert_eq!(state.chunk_size_samples, 8000);
    }

    // ── try_drain_chunk ─────────────────────────────────────────────────

    #[test]
    fn test_drain_not_enough_for_chunk() {
        let mut state = make_state(2.0, 2, 5);
        state.buffer = vec![0.0; 16000]; // 1 second < 2 second chunk
        assert!(!try_drain_chunk(&mut state));
        assert_eq!(state.chunk_id, 0);
        assert!(state.audio_accum.is_empty());
        assert_eq!(state.buffer.len(), 16000); // buffer unchanged
    }

    #[test]
    fn test_drain_exactly_one_chunk() {
        let mut state = make_state(2.0, 2, 5);
        state.buffer = vec![0.1; 32000]; // exactly one chunk
        assert!(try_drain_chunk(&mut state));
        assert_eq!(state.chunk_id, 1);
        assert_eq!(state.audio_accum.len(), 32000);
        assert!(state.buffer.is_empty());
    }

    #[test]
    fn test_drain_partial_remainder() {
        let mut state = make_state(2.0, 2, 5);
        state.buffer = vec![0.1; 40000]; // 1.25 chunks worth
        assert!(try_drain_chunk(&mut state));
        assert_eq!(state.chunk_id, 1);
        assert_eq!(state.audio_accum.len(), 32000);
        assert_eq!(state.buffer.len(), 8000); // leftover
    }

    // ── combine_prefix_and_generated ────────────────────────────────────

    #[test]
    fn test_combine_no_prefix() {
        let state = make_state(2.0, 2, 5);
        let generated = vec![100, 200, 300];
        let result = combine_prefix_and_generated(&state, &None, &generated);
        assert_eq!(result, vec![100, 200, 300]);
    }

    #[test]
    fn test_combine_with_prefix_and_rollback() {
        let mut state = make_state(2.0, 2, 5);
        state.raw_token_ids = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        let prefix = Some("some prefix".to_string());
        let generated = vec![100, 200];
        let result = combine_prefix_and_generated(&state, &prefix, &generated);
        assert_eq!(result, vec![1, 2, 3, 4, 5, 100, 200]);
    }

    #[test]
    fn test_combine_with_prefix_but_empty_raw() {
        let state = make_state(2.0, 2, 5);
        let prefix = Some("prefix".to_string());
        let generated = vec![100, 200];
        let result = combine_prefix_and_generated(&state, &prefix, &generated);
        assert_eq!(result, vec![100, 200]);
    }

    #[test]
    fn test_combine_raw_shorter_than_unfixed() {
        let mut state = make_state(2.0, 2, 5);
        state.raw_token_ids = vec![1, 2, 3];
        let prefix = Some("prefix".to_string());
        let generated = vec![100, 200];
        let result = combine_prefix_and_generated(&state, &prefix, &generated);
        assert_eq!(result, vec![100, 200]);
    }

    #[test]
    fn test_combine_raw_exactly_unfixed() {
        let mut state = make_state(2.0, 2, 5);
        state.raw_token_ids = vec![1, 2, 3, 4, 5];
        let prefix = Some("prefix".to_string());
        let generated = vec![100];
        let result = combine_prefix_and_generated(&state, &prefix, &generated);
        assert_eq!(result, vec![100]);
    }

    #[test]
    fn test_combine_raw_one_more_than_unfixed() {
        let mut state = make_state(2.0, 2, 5);
        state.raw_token_ids = vec![10, 20, 30, 40, 50, 60];
        let prefix = Some("prefix".to_string());
        let generated = vec![100, 200];
        let result = combine_prefix_and_generated(&state, &prefix, &generated);
        assert_eq!(result, vec![10, 100, 200]);
    }

    // ── compute_prefix_ids ──────────────────────────────────────────────

    #[test]
    fn test_prefix_cold_start_chunk1() {
        let mut state = make_state(2.0, 2, 5);
        state.chunk_id = 1;
        state.raw_token_ids = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        assert_eq!(compute_prefix_ids(&state), None);
    }

    #[test]
    fn test_prefix_cold_start_chunk2() {
        let mut state = make_state(2.0, 2, 5);
        state.chunk_id = 2;
        state.raw_token_ids = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        assert_eq!(compute_prefix_ids(&state), None);
    }

    #[test]
    fn test_prefix_eligible_chunk3() {
        let mut state = make_state(2.0, 2, 5);
        state.chunk_id = 3;
        state.raw_token_ids = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        assert_eq!(compute_prefix_ids(&state), Some(&[1, 2, 3, 4, 5][..]));
    }

    #[test]
    fn test_prefix_empty_raw_tokens() {
        let mut state = make_state(2.0, 2, 5);
        state.chunk_id = 5;
        assert_eq!(compute_prefix_ids(&state), None);
    }

    #[test]
    fn test_prefix_too_few_tokens() {
        let mut state = make_state(2.0, 2, 5);
        state.chunk_id = 5;
        state.raw_token_ids = vec![1, 2];
        assert_eq!(compute_prefix_ids(&state), None);
    }

    #[test]
    fn test_prefix_exactly_unfixed_tokens() {
        let mut state = make_state(2.0, 2, 5);
        state.chunk_id = 5;
        state.raw_token_ids = vec![1, 2, 3, 4, 5];
        assert_eq!(compute_prefix_ids(&state), None);
    }

    // ── flush_remaining_buffer ──────────────────────────────────────────

    #[test]
    fn test_flush_empty_state_returns_false() {
        let mut state = make_state(2.0, 2, 5);
        assert!(!flush_remaining_buffer(&mut state));
        assert!(state.buffer.is_empty());
        assert!(state.audio_accum.is_empty());
        assert_eq!(state.chunk_id, 0);
    }

    #[test]
    fn test_flush_buffer_to_accum() {
        let mut state = make_state(2.0, 2, 5);
        state.buffer = vec![0.1; 5000];
        assert!(flush_remaining_buffer(&mut state));
        assert!(state.buffer.is_empty());
        assert_eq!(state.audio_accum.len(), 5000);
        assert_eq!(state.chunk_id, 1);
    }

    #[test]
    fn test_flush_empty_buffer_with_existing_accum() {
        let mut state = make_state(2.0, 2, 5);
        state.audio_accum = vec![0.1; 32000]; // already has audio
        state.chunk_id = 1;
        assert!(flush_remaining_buffer(&mut state));
        assert_eq!(state.chunk_id, 1); // unchanged, buffer was empty
        assert_eq!(state.audio_accum.len(), 32000);
    }

    // ── EncoderCache ────────────────────────────────────────────────────

    #[test]
    fn test_encoder_cache_new_empty() {
        let cache = EncoderCache::new();
        assert_eq!(cache.cached_tokens(), 0);
    }

    #[test]
    fn test_encoder_cache_default() {
        let cache = EncoderCache::default();
        assert_eq!(cache.cached_tokens(), 0);
    }

    // ── Multiple chunks via try_drain_chunk ──────────────────────────────

    #[test]
    fn test_multi_chunk_accumulation() {
        let mut state = make_state(1.0, 2, 5); // 1 second chunks = 16000 samples

        for i in 0..3 {
            let samples = vec![0.01 * (i as f32 + 1.0); 16000];
            state.buffer.extend_from_slice(&samples);
            assert!(try_drain_chunk(&mut state));
        }

        assert_eq!(state.chunk_id, 3);
        assert_eq!(state.audio_accum.len(), 48000);
        assert!(state.buffer.is_empty());
    }

    // ── Rollback prefix evolution over chunks ───────────────────────────

    #[test]
    fn test_prefix_evolution_across_chunks() {
        let mut state = make_state(2.0, 2, 5);

        // Chunk 1: cold start → no prefix
        state.chunk_id = 1;
        assert_eq!(compute_prefix_ids(&state), None);

        // Chunk 2: still cold start
        state.chunk_id = 2;
        state.raw_token_ids = vec![10, 20, 30, 40, 50, 60, 70, 80];
        assert_eq!(compute_prefix_ids(&state), None);

        // Chunk 3: prefix kicks in, rollback 5 → keep first 3
        state.chunk_id = 3;
        let prefix_ids = compute_prefix_ids(&state).unwrap();
        assert_eq!(prefix_ids, &[10, 20, 30]);

        // Simulate generation: combine prefix + new tokens
        let new_generated = vec![100, 200, 300, 400];
        let prefix = Some("dummy".to_string()); // just needs to be Some
        let full = combine_prefix_and_generated(&state, &prefix, &new_generated);
        assert_eq!(full, vec![10, 20, 30, 100, 200, 300, 400]);
        state.raw_token_ids = full;

        // Chunk 4: rollback 5 from 7 → keep first 2
        state.chunk_id = 4;
        let prefix_ids = compute_prefix_ids(&state).unwrap();
        assert_eq!(prefix_ids, &[10, 20]);
    }

    // ── Language option forwarding ──────────────────────────────────────

    #[test]
    fn test_streaming_options_with_language() {
        let opts = StreamingOptions { language: Some("english".to_string()), ..Default::default() };
        assert_eq!(opts.language.as_deref(), Some("english"));
    }

    #[test]
    fn test_streaming_options_auto_language() {
        let opts = StreamingOptions::default();
        assert!(opts.language.is_none());
    }

    // ── initial_text ──────────────────────────────────────────────────────

    #[test]
    fn test_streaming_options_default_initial_text_is_none() {
        let opts = StreamingOptions::default();
        assert!(opts.initial_text.is_none());
    }

    #[test]
    fn test_with_initial_text_sets_value() {
        let opts = StreamingOptions::default().with_initial_text("previous transcript context");
        assert_eq!(opts.initial_text.as_deref(), Some("previous transcript context"));
    }

    #[test]
    fn test_with_initial_text_empty_string_becomes_none() {
        let opts = StreamingOptions::default().with_initial_text("");
        assert!(opts.initial_text.is_none());
    }

    #[test]
    fn test_with_initial_text_chained_with_other_builders() {
        let opts = StreamingOptions::default()
            .with_language("chinese")
            .with_initial_text("之前的文字")
            .with_chunk_size_sec(3.0);
        assert_eq!(opts.language.as_deref(), Some("chinese"));
        assert_eq!(opts.initial_text.as_deref(), Some("之前的文字"));
        assert_eq!(opts.chunk_size_sec, 3.0);
    }

    #[test]
    fn test_compute_prefix_ids_cold_start_returns_none_regardless_of_initial_text() {
        // compute_prefix_ids is pure token logic — it always returns None during cold start.
        // The build_prefix function is responsible for returning initial_text instead.
        let mut state = make_state(2.0, 2, 5);
        state.options.initial_text = Some("context".to_string());
        state.chunk_id = 1;
        state.raw_token_ids = vec![1, 2, 3, 4, 5, 6, 7, 8];
        assert_eq!(compute_prefix_ids(&state), None);
    }

    #[test]
    fn test_compute_prefix_ids_after_cold_start_ignores_initial_text() {
        // After cold start, normal rollback is used regardless of initial_text.
        let mut state = make_state(2.0, 2, 5);
        state.options.initial_text = Some("context".to_string());
        state.chunk_id = 3;
        state.raw_token_ids = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        assert_eq!(compute_prefix_ids(&state), Some(&[1, 2, 3, 4, 5][..]));
    }
}
