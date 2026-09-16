//! The speech model on this machine, held between replies.
//!
//! Loading it is a third of a gigabyte off disc, plus a dictionary of ninety
//! thousand words, so the loaded model is kept: the first reply of a session
//! waits for it and every reply after that does not. It is dropped by
//! nothing, because a receiver that has spoken once will speak again.

use super::Said;
use crate::agent::config::Config;
use std::sync::{Mutex, OnceLock};

/// The one voice, and what it was loaded with, so a changed speaker, device
/// or directory reloads rather than being ignored.
struct Held {
    engine: tts::Engine,
    name: String,
    dir: std::path::PathBuf,
    device: tts::DeviceChoice,
}

fn held() -> &'static Mutex<Option<Held>> {
    static HELD: OnceLock<Mutex<Option<Held>>> = OnceLock::new();
    HELD.get_or_init(|| Mutex::new(None))
}

/// Which speaker: a name from the catalogue, or any the repository has.
fn name(config: &Config) -> String {
    match config.voice_local.trim() {
        "" => tts::DEFAULT_VOICE.to_string(),
        name => name.to_string(),
    }
}

/// Where the model, the voices and the dictionary are kept.
fn dir(config: &Config) -> std::path::PathBuf {
    match config.voice_dir.trim() {
        "" => tts::default_dir(),
        d => std::path::PathBuf::from(d),
    }
}

/// Say `text`, blocking the thread it is called on.
pub fn speak(config: &Config, text: &str) -> Result<Said, String> {
    let (name, dir) = (name(config), dir(config));
    let device = tts::DeviceChoice::parse(&config.voice_device);
    let mut slot = held().lock().map_err(|_| "the speech model is poisoned".to_string())?;
    let stale = slot.as_ref().is_none_or(|h| h.name != name || h.dir != dir || h.device != device);
    if stale {
        // Reported as it goes, because this is hundreds of megabytes over
        // somebody's home connection and a silent wait of that length cannot
        // be told from a fetch that has hung.
        let files = tts::Files::ensure(&dir, &name, &mut |f| {
            super::fetching(&f.file, f.done, f.total, f.files_done, f.files)
        })
        .map_err(|e| fault(e.to_string()))?;
        super::report(|h| h.state = crate::transcripts::ModelState::Loading);
        let (engine, on, note) =
            tts::Engine::load_on(&files, device).map_err(|e| fault(e.to_string()))?;
        tracing::info!("speech model on {on}");
        super::report(|h| {
            h.state = crate::transcripts::ModelState::Ready;
            h.device = on.clone();
            // Why it is not where it was asked to be, which is the difference
            // between a receiver that answers inside an over and one that
            // takes a minute: a card already holding something else has no
            // room for these weights, and the fall back is silent otherwise.
            h.note = note.clone();
            h.fetch = Default::default();
        });
        *slot = Some(Held { engine, name, dir, device });
    }
    let held = slot.as_mut().expect("just loaded");
    let at = std::time::Instant::now();
    let samples = held
        .engine
        .say(text, crate::agent::channel::MAX_OVER_S)
        .map_err(|e| fault(e.to_string()))?;
    let rate = held.engine.rate();
    super::report(|h| {
        h.state = crate::transcripts::ModelState::Ready;
        h.reads += 1;
        h.last_ms = at.elapsed().as_millis() as u64;
        h.last_audio_s = samples.len() as f64 / rate.max(1.0);
    });
    Ok(Said { rate, samples })
}

/// Say why it cannot speak, and hand the reason back.
fn fault(why: String) -> String {
    super::report(|h| h.state = crate::transcripts::ModelState::Failed(why.clone()));
    why
}
