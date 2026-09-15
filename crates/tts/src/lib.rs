//! Speech from text, locally, with no network and no service to run.
//!
//! Parler TTS through candle: a T5 encoder reads a description of the voice,
//! a decoder generates DAC audio tokens conditioned on it and on the words,
//! and the DAC decoder turns those into samples. The model takes plain text
//! rather than phonemes, which is why it is the one here: nothing in this
//! program has to know how a word is pronounced, and there is no espeak to
//! install.
//!
//! What it costs is size and arithmetic. The weights are about three and a
//! half gigabytes, and generation is autoregressive at the codec's frame
//! rate, so a card generates faster than the speech plays and a CPU does not.
//! The receiver treats it the way it treats Whisper: fetched once, loaded on
//! first use, and run on whatever device is fastest.
//!
//! Not in the graph, and it keys nothing: this crate turns a sentence into
//! samples. What is done with them is `agent::channel`'s business.

mod voice;

pub use hfmodel::{Fetching, OnProgress};
pub use voice::{Files, Voice};

/// The model fetched when nobody names another.
///
/// Mini rather than large: a third of the size and quick enough on a card to
/// answer inside an over. Large is a directory away for anybody who wants it.
pub const DEFAULT_REPO: &str = "parler-tts/parler-tts-mini-v1";

/// How the voice is described to the model when nobody says otherwise.
///
/// Parler is steered by a sentence rather than by a voice id, and this one is
/// written for the channel it comes out of: close, dry and level, because
/// reverberation and dynamics do not survive a 2.5 kHz deviation FM link.
pub const DEFAULT_DESCRIPTION: &str = "A clear, level male voice speaking at a measured pace, very close to the microphone, \
     with no background noise and no reverberation.";

/// Where a downloaded or hand-placed model lives.
pub fn default_dir() -> std::path::PathBuf {
    std::env::var_os("HOME")
        .map(std::path::PathBuf::from)
        .unwrap_or_default()
        .join(".local/share/waveshark/models/parler")
}

/// The fastest device candle was built for and can actually run on.
///
/// The same reasoning as `stt::best_device`, and the same fallback: a card
/// that opens and then cannot launch a kernel is not an error to stop for.
/// Here the fallback hurts more, since the CPU is slower than real time, so
/// the caller is told which device it got.
pub fn best_device() -> candle_core::Device {
    #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
    if let Ok(d) = candle_core::Device::new_cuda(0)
        && runs(&d)
    {
        return d;
    }
    #[cfg(target_vendor = "apple")]
    if let Ok(d) = candle_core::Device::new_metal(0)
        && runs(&d)
    {
        return d;
    }
    candle_core::Device::Cpu
}

pub fn device_label(d: &candle_core::Device) -> String {
    match d {
        candle_core::Device::Cpu => "CPU".into(),
        candle_core::Device::Cuda(_) => "CUDA".into(),
        candle_core::Device::Metal(_) => "Metal".into(),
    }
}

/// Whether a device can do arithmetic, not merely be opened.
#[allow(dead_code)]
fn runs(d: &candle_core::Device) -> bool {
    let Ok(a) = candle_core::Tensor::new(&[1.0f32, 2.0], d) else {
        return false;
    };
    a.matmul(&a.reshape((2, 1)).unwrap_or(a.clone())).is_ok() || a.sum_all().is_ok()
}
