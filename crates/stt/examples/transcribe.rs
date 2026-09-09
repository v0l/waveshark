//! Transcribe a wav file, to check a model directory works before the
//! receiver depends on it.
//!
//! ```sh
//! cargo run --release -p stt --example transcribe -- jfk.wav [model-id] [device]
//! ```
//!
//! The model is a catalogue id (`whisper-base.en`, `qwen3-asr-0.6b`) or a
//! directory; the device is `auto`, `cpu`, `cuda:0` or `metal`.

use std::path::PathBuf;

fn main() -> common::Result<()> {
    let mut args = std::env::args().skip(1);
    let wav = args.next().unwrap_or_else(|| {
        eprintln!("usage: transcribe <file.wav> [model-id|model-dir] [device]");
        std::process::exit(2)
    });
    let model = args.next().unwrap_or_else(|| stt::DEFAULT_MODEL.to_string());
    let device = stt::DeviceChoice::parse(&args.next().unwrap_or_default());
    let dir: PathBuf = if PathBuf::from(&model).join("config.json").exists() {
        PathBuf::from(&model)
    } else {
        stt::model_dir(&stt::default_dir(), &model)
    };

    let files = stt::ensure(&stt::repo_of(&model), &dir)?;
    println!("{} in {}", files.family.label(), dir.display());

    let (pcm, rate) = read_wav(&wav)?;
    let (mut w, on, note) = stt::Engine::load_on(&files, device, None)?;
    println!("on {on}");
    if !note.is_empty() {
        println!("{note}");
    }
    // Twice, because the first read on a GPU is mostly the driver compiling
    // kernels and says nothing about how fast the model is.
    let t0 = std::time::Instant::now();
    let _ = w.transcribe(&pcm, rate)?;
    let first = t0.elapsed().as_secs_f64();
    let t0 = std::time::Instant::now();
    let out = w.transcribe(&pcm, rate)?;
    let secs = pcm.len() as f64 / rate;
    println!("{secs:.1}s audio in {:.1}s (first read {first:.1}s)", t0.elapsed().as_secs_f64());
    for s in &out.segments {
        println!(
            "{:6.1}-{:5.1}  logprob {:+.2}  no-speech {:.2}  {}",
            s.start_s,
            s.end_s,
            s.avg_logprob,
            s.no_speech_prob,
            s.text.trim()
        );
    }
    if let Some(l) = &out.language {
        println!("language: {l}");
    }
    println!("\n{}", out.text);
    Ok(())
}

fn read_wav(path: &str) -> common::Result<(Vec<f32>, f64)> {
    let r =
        hound::WavReader::open(path).map_err(|e| common::Error::other(format!("{path}: {e}")))?;
    let spec = r.spec();
    let ch = spec.channels.max(1) as usize;
    let pcm: Vec<f32> = match spec.sample_format {
        hound::SampleFormat::Float => r.into_samples::<f32>().filter_map(|s| s.ok()).collect(),
        hound::SampleFormat::Int => {
            let scale = 1.0 / (1i64 << (spec.bits_per_sample - 1)) as f32;
            r.into_samples::<i32>().filter_map(|s| s.ok()).map(|s| s as f32 * scale).collect()
        }
    };
    let mono = pcm.chunks(ch).map(|f| f.iter().sum::<f32>() / ch as f32).collect();
    Ok((mono, spec.sample_rate as f64))
}
