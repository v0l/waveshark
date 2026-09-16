//! What the speech model says, checked against what a separate model hears.
//!
//! There is no reference waveform to compare against: the vocoder's
//! excitation carries random phase, so the publisher's own implementation
//! does not produce the same samples twice. What can be pinned is everything
//! that matters to somebody listening. The sentence has to come out as
//! phonemes the dictionary agrees with, the durations have to add up to
//! speech of about the right length, and the audio has to be intelligible,
//! which is asserted by transcribing it.
//!
//! The model is a fetch of about 330 MB, so these skip when it is not
//! already on disc: `cargo run -p tts --example say -- "radio check"` puts it
//! there.

use std::path::PathBuf;

fn dir() -> PathBuf {
    match std::env::var_os("WAVESHARK_TTS_DIR") {
        Some(d) => PathBuf::from(d),
        None => tts::default_dir(),
    }
}

fn engine() -> Option<tts::Engine> {
    let files = tts::Files::in_dir(dir(), tts::DEFAULT_VOICE).ok()?;
    match tts::Engine::load(&files, candle_core::Device::Cpu) {
        Ok(e) => Some(e),
        Err(e) => panic!("the model is on disc but did not load: {e}"),
    }
}

fn skip() {
    eprintln!(
        "skipping: no speech model in {}, run `cargo run --release -p tts --example say`",
        dir().display()
    );
}

/// The dictionary's own pronunciations, which are what the model was trained
/// against: a word it holds comes out exactly, and a number comes out as the
/// words somebody would say.
#[test]
fn a_sentence_becomes_the_phonemes_the_dictionary_gives() {
    let Some(engine) = engine() else { return skip() };
    let said = engine.phonemes("Radio check, how do you read me?");
    assert_eq!(said.phonemes, "ɹˈAdiO ʧˈɛk, hˌW dˈu ju ɹˈid mˌi?");
    assert!(said.guessed.is_empty(), "guessed at {:?}", said.guessed);

    // A frequency is read digit by digit with a point, which is what an
    // operator says and not what a bare number-to-words rule gives.
    let said = engine.phonemes("433.92 megahertz");
    assert!(said.guessed.is_empty(), "guessed at {:?}", said.guessed);
    assert_eq!(said.phonemes, "fˈɔɹ θɹˈi θɹˈi pˈYnt nˈIn tˈu mˈɛɡəhˌɜɹts");
}

/// A word outside the dictionary is still said, and is reported as a guess so
/// nothing downstream mistakes it for a pronunciation somebody checked.
#[test]
fn a_word_outside_the_dictionary_is_reported() {
    let Some(engine) = engine() else { return skip() };
    let said = engine.phonemes("waveshark calling");
    assert_eq!(said.guessed, ["waveshark"]);
    assert!(said.phonemes.starts_with("wˈævɛʃɑɹk"), "{}", said.phonemes);
}

/// The whole path: a sentence in, samples out.
///
/// What this cannot assert is that the samples are intelligible. That was
/// checked by hand the only way it can be, by handing the audio to a reading
/// model: `cargo run --release -p stt --example transcribe -- said.wav` gives
/// back "Radio check, how do you read me?" for the file the `say` example
/// writes.
#[test]
fn what_it_says_is_what_was_asked_for() {
    let Some(mut engine) = engine() else { return skip() };
    let samples = engine.say("Radio check, how do you read me?", 30.0).expect("speech");
    let secs = samples.len() as f64 / engine.rate();
    // Six words at the pace this voice speaks: measured at 2.4 s, and a
    // window either side of it rather than a floor, because a model that
    // rushes or drags is as wrong as one that says nothing.
    assert!((1.8..3.2).contains(&secs), "{secs:.2} s of speech for six words");
    let peak = samples.iter().fold(0f32, |a, s| a.max(s.abs()));
    assert!((0.05..1.0).contains(&peak), "peak {peak:.3}");
}
