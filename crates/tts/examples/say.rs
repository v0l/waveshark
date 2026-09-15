//! Say something, to a WAV file.
//!
//! `cargo run --release -p tts --example say -- "radio check, how do you read"`
//!
//! The first run fetches the model into
//! `~/.local/share/waveshark/models/parler/<id>`. Which model, where it runs
//! and at what precision are the environment: `TTS_MODEL=parler-large-v1`,
//! `TTS_DEVICE=cuda:0`, `TTS_PRECISION=half`. `tts::MODELS` is the list.

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let text = args.next().unwrap_or_else(|| "radio check, how do you read me".to_string());
    let root = args.next().map(std::path::PathBuf::from).unwrap_or_else(tts::default_dir);
    let id = std::env::var("TTS_MODEL").unwrap_or_else(|_| tts::DEFAULT_MODEL.to_string());
    let dir = tts::dir_for(&root, &id);
    println!("{} ({})", tts::label_of(&id), tts::repo_of(&id));

    let start = std::time::Instant::now();
    let files = tts::Files::ensure(&tts::repo_of(&id), &dir, &mut |f| {
        if f.total > 0 && f.done == f.total {
            println!("{} {:.0} MB", f.file, f.total as f64 / 1e6);
        }
    })?;
    println!(
        "model in {} ({:.1} GB), {:?}",
        dir.display(),
        files.bytes() as f64 / 1e9,
        start.elapsed()
    );

    let choice = tts::DeviceChoice::parse(&std::env::var("TTS_DEVICE").unwrap_or_default());
    let precision = tts::Precision::parse(&std::env::var("TTS_PRECISION").unwrap_or_default());
    println!("asking for {} at {}", choice.id(), precision.label());
    let start = std::time::Instant::now();
    let (mut voice, on, note) = tts::Engine::load_on(
        &files,
        tts::family_of(&id),
        choice,
        precision,
        tts::DEFAULT_DESCRIPTION,
    )?;
    println!("loaded onto {on}");
    if !note.is_empty() {
        println!("note: {note}");
    }
    if let Some(t) = std::env::var("TTS_TEMP").ok().and_then(|v| v.parse().ok()) {
        voice.set_temperature(t);
    }
    println!("loaded in {:?}", start.elapsed());

    let start = std::time::Instant::now();
    let max_s = std::env::var("TTS_MAX").ok().and_then(|v| v.parse().ok()).unwrap_or(20.0);
    let pcm = voice.say(&text, max_s)?;
    let seconds = pcm.len() as f64 / voice.rate();
    println!(
        "{:.1} s of speech in {:?} ({:.1}x real time)",
        seconds,
        start.elapsed(),
        seconds / start.elapsed().as_secs_f64()
    );

    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: voice.rate() as u32,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let out = std::env::temp_dir().join("said.wav");
    let mut w = hound::WavWriter::create(&out, spec)?;
    for s in pcm {
        w.write_sample((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)?;
    }
    w.finalize()?;
    println!("wrote {}", out.display());
    Ok(())
}
