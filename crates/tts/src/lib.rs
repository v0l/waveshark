//! Speech from text, locally, with no network and no service to run.
//!
//! Kokoro through candle: a phoneme model reads the sentence, a prosody
//! predictor decides how long each sound lasts and at what pitch, and a
//! vocoder turns that into samples. Eighty-two million parameters, about
//! 330 MB on disc, and ahead of real time on a processor, so a receiver with
//! no card still answers inside an over.
//!
//! It reads phonemes rather than letters, and [`g2p`] is what turns a
//! sentence into them: a dictionary of ninety thousand words published with
//! the model, and a guess for anything outside it.
//!
//! Not in the graph, and it keys nothing: this crate turns a sentence into
//! samples. What is done with them is `agent::channel`'s business.

mod catalogue;
mod engine;
mod files;
pub mod g2p;
pub mod kokoro;

pub use catalogue::{DEFAULT_VOICE, DeviceChoice, VOICES, Voice, devices, label_of, voice};
pub use engine::Engine;
pub use files::{Files, LEXICON_REPO, MODEL_REPO, VOICE_REPO, installed};
pub use hfmodel::{Fetching, OnProgress};

/// Where a downloaded or hand-placed model lives.
pub fn default_dir() -> std::path::PathBuf {
    std::env::var_os("HOME")
        .map(std::path::PathBuf::from)
        .unwrap_or_default()
        .join(".local/share/waveshark/models/kokoro")
}

/// The fastest device candle was built for and can actually run on.
///
/// The same reasoning as `stt::best_device`, and the same fallback: a card
/// that opens and then cannot launch a kernel is not an error to stop for.
/// Here the fallback costs less than it used to, since this model is faster
/// than the speech it makes either way.
pub fn best_device() -> candle_core::Device {
    #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
    if let Ok(d) = open_cuda(0)
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

/// A CUDA device this process opened, opened once and never given back, for
/// the reason `stt::open_cuda` gives: the statically linked CUDA runtime
/// tears its context down at exit and a cuBLAS handle destroyed afterwards
/// segfaults.
#[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
pub fn open_cuda(n: usize) -> Result<candle_core::Device, candle_core::Error> {
    use std::sync::Mutex;
    static OPEN: Mutex<Vec<(usize, candle_core::Device)>> = Mutex::new(Vec::new());
    let mut open = OPEN.lock().unwrap_or_else(|e| e.into_inner());
    if let Some((_, d)) = open.iter().find(|(i, _)| *i == n) {
        return Ok(d.clone());
    }
    let d = candle_core::Device::new_cuda(n)?;
    open.push((n, d.clone()));
    Ok(d)
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
    a.affine(2.0, 1.0).and_then(|t| t.sum_all()).and_then(|t| t.to_scalar::<f32>()).is_ok()
}
