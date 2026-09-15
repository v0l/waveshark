//! The speech model on this machine, held between replies.
//!
//! Loading Parler is three and a half gigabytes off disc and onto a card, so
//! the loaded model is kept: the first reply of a session waits for it and
//! every reply after that does not. It is dropped by nothing, because a
//! receiver that has spoken once will speak again and the alternative is
//! paying the load on every over.

use super::Said;
use crate::agent::config::Config;
use std::sync::{Mutex, OnceLock};

/// The one voice, and what it was loaded with, so a changed description or a
/// changed directory reloads rather than being ignored.
struct Held {
    voice: tts::Voice,
    dir: std::path::PathBuf,
    description: String,
}

fn held() -> &'static Mutex<Option<Held>> {
    static HELD: OnceLock<Mutex<Option<Held>>> = OnceLock::new();
    HELD.get_or_init(|| Mutex::new(None))
}

/// Where the weights are: what the operator set, or the usual place.
fn dir(config: &Config) -> std::path::PathBuf {
    match config.voice_dir.trim() {
        "" => tts::default_dir(),
        d => std::path::PathBuf::from(d),
    }
}

/// Say `text`, blocking the thread it is called on.
pub fn speak(config: &Config, text: &str) -> Result<Said, String> {
    let dir = dir(config);
    let description = match config.voice_description.trim() {
        "" => tts::DEFAULT_DESCRIPTION.to_string(),
        d => d.to_string(),
    };
    let mut slot = held().lock().map_err(|_| "the speech model is poisoned".to_string())?;
    let stale = slot.as_ref().is_none_or(|h| h.dir != dir || h.description != description);
    if stale {
        // Reported as it goes, because this is gigabytes over somebody's home
        // connection and a silent wait of that length cannot be told from a
        // fetch that has hung.
        let files = tts::Files::ensure(
            match config.voice_repo.trim() {
                "" => tts::DEFAULT_REPO,
                r => r,
            },
            &dir,
            &mut |f| super::fetching(&f.file, f.done, f.total, f.files_done, f.files),
        )
        .map_err(|e| fault(e.to_string()))?;
        super::report(|h| h.state = crate::transcripts::ModelState::Loading);
        let (voice, device) =
            tts::Voice::load_best(&files, &description).map_err(|e| fault(e.to_string()))?;
        tracing::info!("speech model on {device}");
        super::report(|h| {
            h.state = crate::transcripts::ModelState::Ready;
            h.device = device.clone();
            h.fetch = Default::default();
        });
        *slot = Some(Held { voice, dir, description });
    }
    let held = slot.as_mut().expect("just loaded");
    let at = std::time::Instant::now();
    let samples = held
        .voice
        .say(text, crate::agent::channel::MAX_OVER_S)
        .map_err(|e| fault(e.to_string()))?;
    let rate = held.voice.rate();
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
