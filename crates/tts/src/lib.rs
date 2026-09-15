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

mod catalogue;
mod engine;
mod voice;

pub use catalogue::{
    DEFAULT_MODEL, DeviceChoice, Family, MODELS, Model, Precision, devices, dir_for, family_of,
    installed, label_of, model, model_dir, repo_of,
};
pub use engine::Engine;
pub use hfmodel::{Fetching, OnProgress};
pub use voice::{Files, Voice};

/// The model fetched when nobody names another.
///
/// Mini rather than large: a third of the size and quick enough on a card to
/// answer inside an over. Large is a directory away for anybody who wants it.
pub const DEFAULT_REPO: &str = "parler-tts/parler-tts-mini-v1";

/// How the voice is described to the model when nobody says otherwise.
///
/// Parler is steered by a sentence rather than by a voice id, and it follows
/// the shape of the sentences it was trained on rather than the meaning of
/// any wording. A description written from first principles for a radio
/// channel, asking for a level voice very close to the microphone with no
/// background noise, came out as a man whispering: taken together those are
/// how an intimate recording is described, and that is what it made.
///
/// So: one of the named speakers the v1 models were trained with, and nothing
/// about the recording. Parler's own guide asks for a line about the audio
/// quality, but a clean take is what it makes when nobody says otherwise, and
/// asking for one is what produced the whispering.
pub const DEFAULT_DESCRIPTION: &str = "Jon speaks in a clear and confident voice at a moderate pace, projecting as if \
     reading a message aloud.";

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
