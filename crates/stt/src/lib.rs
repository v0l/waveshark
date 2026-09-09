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

mod catalogue;
mod engine;
mod model;
pub mod qwen3;
mod whisper;

pub use catalogue::{
    default_model_in, devices, installed, label_of, model, model_dir, repo_of, DeviceChoice,
    DeviceEntry, Family, Model, DEFAULT_MODEL, MODELS,
};
pub use engine::Engine;
pub use model::{ensure, fetch, Files, Flavour, DEFAULT_REPO};
pub use whisper::{Segment, Transcript, Whisper, WINDOW_S};

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
    std::env::var_os("HOME").map(std::path::PathBuf::from).unwrap_or_default()
}

/// The fastest device candle was built for and can actually run on.
///
/// CUDA where the build asked for it, Metal on a Mac, and the CPU otherwise.
/// A GPU that fails is not an error worth stopping for: Whisper runs on the
/// CPU, slower, and a slow transcript beats none.
///
/// Opening the device is not the test. A driver older than the toolkit the
/// kernels were built with opens happily and then fails on the first launch
/// with `CUDA_ERROR_UNSUPPORTED_PTX_VERSION`, which measured here is what a
/// current card and a candle built against CUDA 13 do, so the check is a real
/// multiplication.
pub fn best_device() -> candle_core::Device {
    #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
    if let Ok(d) = candle_core::Device::new_cuda(0) {
        if runs(&d) {
            return d;
        }
        tracing::warn!("CUDA opened but cannot run kernels; transcribing on the CPU");
    }
    #[cfg(target_vendor = "apple")]
    if let Ok(d) = candle_core::Device::new_metal(0) {
        if runs(&d) {
            return d;
        }
        tracing::warn!("Metal opened but cannot run kernels; transcribing on the CPU");
    }
    candle_core::Device::Cpu
}

/// What a device is called, for a pane that has to say where the model is
/// running. A transcript arriving slowly on the CPU and one arriving quickly
/// on a card look the same on screen otherwise.
pub fn device_label(d: &candle_core::Device) -> String {
    match d {
        candle_core::Device::Cpu => "CPU".into(),
        candle_core::Device::Cuda(c) => {
            #[cfg(all(feature = "cuda", not(target_vendor = "apple")))]
            {
                let k = c.cuda_stream().context().ordinal();
                return devices()
                    .into_iter()
                    .find(|e| e.choice == DeviceChoice::Cuda(k))
                    .map(|e| e.label)
                    .unwrap_or_else(|| format!("CUDA {k}"));
            }
            #[cfg(not(all(feature = "cuda", not(target_vendor = "apple"))))]
            {
                let _ = c;
                "CUDA".into()
            }
        }
        candle_core::Device::Metal(_) => "Metal".into(),
    }
}

/// Whether a device can do the smallest thing the model will ask of it.
#[allow(dead_code)]
pub(crate) fn runs(d: &candle_core::Device) -> bool {
    let go = || -> candle_core::Result<f32> {
        let a = candle_core::Tensor::new(&[[1.0f32, 2.0], [3.0, 4.0]], d)?;
        a.matmul(&a)?.sum_all()?.to_scalar::<f32>()
    };
    go().is_ok()
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
