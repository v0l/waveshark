//! Transcribe a wav file, to check a model directory works before the
//! receiver depends on it.
//!
//! ```sh
//! cargo run --release -p stt --example transcribe -- jfk.wav
//! ```

use std::path::PathBuf;

fn main() -> common::Result<()> {
    let mut args = std::env::args().skip(1);
    let wav = args.next().unwrap_or_else(|| {
        eprintln!("usage: transcribe <file.wav> [model-dir]");
        std::process::exit(2)
    });
    let dir: PathBuf = args
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| stt::default_dir().join("whisper"));

    let files = stt::ensure(stt::DEFAULT_REPO, &dir)?;

    let (pcm, rate) = read_wav(&wav)?;
    let mut w = stt::Whisper::load(&files, stt::best_device(), None)?;
    let t0 = std::time::Instant::now();
    let out = w.transcribe(&pcm, rate)?;
    let secs = pcm.len() as f64 / rate;
    println!("{:.1}s audio in {:.1}s", secs, t0.elapsed().as_secs_f64());
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
            r.into_samples::<i32>()
                .filter_map(|s| s.ok())
                .map(|s| s as f32 * scale)
                .collect()
        }
    };
    let mono = pcm
        .chunks(ch)
        .map(|f| f.iter().sum::<f32>() / ch as f32)
        .collect();
    Ok((mono, spec.sample_rate as f64))
}
