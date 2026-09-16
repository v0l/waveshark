//! Say something, to a WAV file.
//!
//! `cargo run --release -p tts --example say -- "radio check, how do you read"`
//!
//! The first run fetches the model, the voice and the pronunciation
//! dictionary into `~/.local/share/waveshark/models/kokoro`. Which speaker
//! and where it runs are the environment: `TTS_VOICE=bm_george`,
//! `TTS_DEVICE=cuda:0`. `tts::VOICES` is the list.

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let text = args.next().unwrap_or_else(|| "radio check, how do you read me".to_string());
    let dir = args.next().map(std::path::PathBuf::from).unwrap_or_else(tts::default_dir);
    let voice = std::env::var("TTS_VOICE").unwrap_or_else(|_| tts::DEFAULT_VOICE.to_string());
    println!("{}", tts::label_of(&voice));

    let start = std::time::Instant::now();
    let files = tts::Files::ensure(&dir, &voice, &mut |f| {
        if f.total > 0 && f.done == f.total {
            println!("{} {:.0} MB", f.file, f.total as f64 / 1e6);
        }
    })?;
    println!(
        "model in {} ({:.0} MB), {:?}",
        dir.display(),
        files.bytes() as f64 / 1e6,
        start.elapsed()
    );

    let choice = tts::DeviceChoice::parse(&std::env::var("TTS_DEVICE").unwrap_or_default());
    let start = std::time::Instant::now();
    let (mut engine, on, note) = tts::Engine::load_on(&files, choice)?;
    println!("loaded on {on} in {:?}", start.elapsed());
    if !note.is_empty() {
        println!("{note}");
    }

    let said = engine.phonemes(&text);
    println!("{}", said.phonemes);
    if !said.guessed.is_empty() {
        println!("guessed at: {}", said.guessed.join(", "));
    }

    let start = std::time::Instant::now();
    let samples = engine.say(&text, 60.0)?;
    let secs = samples.len() as f64 / engine.rate();
    println!(
        "{secs:.2} s of speech in {:?} ({:.1}x)",
        start.elapsed(),
        secs / start.elapsed().as_secs_f64()
    );

    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: engine.rate() as u32,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    let mut w = hound::WavWriter::create("said.wav", spec)?;
    for s in &samples {
        w.write_sample((s.clamp(-1.0, 1.0) * i16::MAX as f32) as i16)?;
    }
    w.finalize()?;
    println!("said.wav");
    Ok(())
}
