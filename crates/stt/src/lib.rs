//! Speech to text, locally, with no network and no service to run.
//!
//! Whisper through candle: mel filterbank, encoder, greedy decode with the
//! temperature fallback the reference implementation uses. The receiver
//! already has the hardest part of an ASR pipeline, which is knowing when
//! somebody was talking and on what channel, so what is left is turning one
//! transmission's PCM into one string with a confidence beside it.
//!
//! Not in the graph: this crate holds no state the receiver owns and does no
//! routing. `nodes::TranscribeNode` is the node, and it calls this.

mod model;
mod whisper;

#[cfg(feature = "hub")]
pub use model::fetch;
pub use model::{Files, Flavour};
pub use whisper::{Segment, Transcript, Whisper};

/// What Whisper wants, and what the codecs give us.
///
/// Every vocoder in the receiver produces 8 kHz and Whisper was trained at
/// 16 kHz, so everything is resampled on the way in. The upper half of that
/// band is empty afterwards, which the model tolerates but which is worth
/// remembering when a transcript of a DMR call reads worse than a transcript
/// of the same words spoken into a microphone.
pub const RATE: f64 = 16_000.0;

/// Where a downloaded or hand-placed model lives.
pub fn default_dir() -> std::path::PathBuf {
    dirs_home().join(".local/share/waveshark/models")
}

fn dirs_home() -> std::path::PathBuf {
    std::env::var_os("HOME")
        .map(std::path::PathBuf::from)
        .unwrap_or_default()
}

/// Resample to [`RATE`], which is what every entry point here expects.
pub fn to_whisper_rate(pcm: &[f32], rate: f64) -> Vec<f32> {
    if (rate - RATE).abs() < 1.0 {
        return pcm.to_vec();
    }
    let mut rs = audio::Resampler::new(rate, RATE, 8);
    let mut out = Vec::with_capacity((pcm.len() as f64 * RATE / rate) as usize + 16);
    rs.process(pcm, &mut out);
    out
}
