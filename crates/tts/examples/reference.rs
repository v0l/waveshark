//! Compare the port against tensors dumped from the reference implementation.
//!
//! ```sh
//! python3 tools/kokoro_reference.py /tmp/kokoro_ref.safetensors
//! cargo run --release -p tts --example reference -- \
//!     ~/.local/share/waveshark/models/kokoro/config.json \
//!     ~/.local/share/waveshark/models/kokoro/kokoro-v1_0.pth \
//!     ~/.local/share/waveshark/models/kokoro/voices/af_heart.bin \
//!     /tmp/kokoro_ref.safetensors
//! ```
//!
//! Every stage but the vocoder excitation matches to the last digit. The
//! excitation carries random phase, so it and everything after it are
//! compared by ear and by what a reading model makes of the result.

use candle_core::{Device, Tensor};

fn rel(a: &Tensor, b: &Tensor) -> anyhow::Result<f32> {
    let a = a.flatten_all()?.to_dtype(candle_core::DType::F32)?;
    let b = b.flatten_all()?.to_dtype(candle_core::DType::F32)?;
    let n = a.dim(0)?.min(b.dim(0)?);
    let (a, b) = (a.narrow(0, 0, n)?, b.narrow(0, 0, n)?);
    let err = (&a - &b)?.sqr()?.mean_all()?.to_scalar::<f32>()?.sqrt();
    let rms = b.sqr()?.mean_all()?.to_scalar::<f32>()?.sqrt();
    Ok(100.0 * err / rms.max(1e-9))
}

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let config = args.next().expect("config.json");
    let weights = args.next().expect("kokoro-v1_0.pth");
    let voice = args.next().expect("voice.bin");
    let refs = args.next().expect("reference.safetensors");

    let device = Device::Cpu;
    let model = tts::kokoro::Kokoro::load(config.as_ref(), weights.as_ref(), device.clone())?;
    let raw = std::fs::read(&voice)?;
    let vals: Vec<f32> =
        raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect();
    let rows = vals.len() / 256;
    let voice = Tensor::from_vec(vals, (rows, 1, 256), &device)?;
    let want = candle_core::safetensors::load(&refs, &device)?;

    let s = model.stages("hɛlˈO wˌɜɹld", &voice, 1.0)?;
    println!("bert      {:>7.3}%  {:?}", rel(&s.hidden, &want["bert"])?, s.hidden.dims());
    println!("d_en      {:>7.3}%  {:?}", rel(&s.d_en, &want["d_en"])?, s.d_en.dims());
    println!("d         {:>7.3}%  {:?}", rel(&s.d, &want["d"])?, s.d.dims());
    let dur: Vec<f32> = want["dur"].flatten_all()?.to_vec1()?;
    println!(
        "durations {:?} vs {:?}",
        s.durations,
        dur.iter().map(|v| *v as usize).collect::<Vec<_>>()
    );
    println!("en        {:>7.3}%  {:?}", rel(&s.en, &want["en"])?, s.en.dims());
    for (i, t) in s.inner.iter().enumerate() {
        let key = if i == 0 { "shared".to_string() } else { format!("F0blk{}", i - 1) };
        println!("{key:<9} {:>7.3}%  {:?}", rel(t, &want[&key])?, t.dims());
    }
    println!("F0        {:>7.3}%  {:?}", rel(&s.f0, &want["F0"])?, s.f0.dims());
    println!("N         {:>7.3}%  {:?}", rel(&s.energy, &want["N"])?, s.energy.dims());
    println!("t_en      {:>7.3}%  {:?}", rel(&s.t_en, &want["t_en"])?, s.t_en.dims());
    println!("asr       {:>7.3}%  {:?}", rel(&s.asr, &want["asr"])?, s.asr.dims());
    for (name, t) in &s.decoder {
        println!("{name:<9} {:>7.3}%  {:?}", rel(t, &want[name])?, t.dims());
    }
    // The inverse transform on its own: their spectrum, our samples.
    {
        let stft = tts::kokoro::stft::Stft::new(20, 5);
        let mag: Vec<f32> = want["spec"].flatten_all()?.to_vec1()?;
        let ph: Vec<f32> = want["phase"].flatten_all()?.to_vec1()?;
        let frames = want["spec"].dims()[2];
        let mine = stft.inverse(&mag, &ph, frames);
        let mine = Tensor::from_vec(mine, want["audio"].dims1()?, &device)?;
        println!("istft     {:>7.3}%  from their spectrum", rel(&mine, &want["audio"])?);
    }
    let ours = Tensor::from_vec(s.audio.clone(), s.audio.len(), &device)?;
    println!("audio     {:>7.3}%  {} samples", rel(&ours, &want["audio"])?, s.audio.len());
    Ok(())
}
