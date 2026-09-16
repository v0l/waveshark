//! The four files a voice is, and fetching them once.
//!
//! They come from three repositories, which is worth knowing when one of them
//! is slow: the model and its configuration from the publisher, the style
//! tensors from a mirror that publishes them as plain float32 rather than as
//! pickled torch tensors, and the pronunciation dictionary from the front end
//! the model was trained against.
//!
//! Altogether about 330 MB, once, against the three and a half gigabytes the
//! model here before it took.

use common::{Error, Result};
use std::path::{Path, PathBuf};

/// Where the model itself is published.
pub const MODEL_REPO: &str = "hexgrad/Kokoro-82M";
/// The style tensors, as raw float32 rather than as torch pickles: 510 by 256
/// per voice, byte for byte what the publisher's `.pt` holds.
pub const VOICE_REPO: &str = "onnx-community/Kokoro-82M-v1.0-ONNX";
/// The pronunciation dictionary the model was trained against.
pub const LEXICON_REPO: &str = "hexgrad/misaki";
/// American English, which is the accent the default voice speaks.
pub const LEXICON_FILE: &str = "us_gold.json";
const WEIGHTS: &str = "kokoro-v1_0.pth";

#[derive(Clone, Debug)]
pub struct Files {
    pub config: PathBuf,
    pub weights: PathBuf,
    pub lexicon: PathBuf,
    /// The style tensors for one voice.
    pub voice: PathBuf,
}

impl Files {
    /// The files for `voice` in `dir`, if they are all there.
    pub fn in_dir(dir: impl AsRef<Path>, voice: &str) -> Result<Self> {
        let dir = dir.as_ref();
        let need = |p: PathBuf| -> Result<PathBuf> {
            match p.exists() {
                true => Ok(p),
                false => Err(Error::other(format!("{} is missing", p.display()))),
            }
        };
        Ok(Self {
            config: need(dir.join("config.json"))?,
            weights: need(dir.join(WEIGHTS))?,
            lexicon: need(dir.join(LEXICON_FILE))?,
            voice: need(dir.join("voices").join(format!("{voice}.bin")))?,
        })
    }

    /// The same, fetching whatever is not there yet.
    pub fn ensure(dir: impl AsRef<Path>, voice: &str, on: hfmodel::OnProgress<'_>) -> Result<Self> {
        let dir = dir.as_ref();
        if let Ok(f) = Self::in_dir(dir, voice) {
            return Ok(f);
        }
        let voice_file = format!("voices/{voice}.bin");
        let seen = std::cell::RefCell::new(on);
        let mut report = |f: &hfmodel::Fetching| (seen.borrow_mut())(f);
        let fetch = hfmodel::Fetch::new(MODEL_REPO, "main", dir, &mut report)?;
        fetch.get("config.json")?;
        fetch.get(WEIGHTS)?;
        drop(fetch);
        let mut report = |f: &hfmodel::Fetching| (seen.borrow_mut())(f);
        let voices = hfmodel::Fetch::new(VOICE_REPO, "main", dir, &mut report)?;
        voices.get(&voice_file)?;
        drop(voices);
        let mut report = |f: &hfmodel::Fetching| (seen.borrow_mut())(f);
        let lexicon = hfmodel::Fetch::dataset(LEXICON_REPO, "main", dir, &mut report)?;
        lexicon.get(LEXICON_FILE)?;
        drop(lexicon);
        Self::in_dir(dir, voice)
    }

    /// What the voice takes on disc.
    pub fn bytes(&self) -> u64 {
        hfmodel::bytes_of(&[
            self.config.clone(),
            self.weights.clone(),
            self.lexicon.clone(),
            self.voice.clone(),
        ])
    }
}

/// Voices already fetched into `dir`, in catalogue order, with anything else
/// found beside them after.
pub fn installed(dir: &Path) -> Vec<String> {
    let has = |id: &str| dir.join("voices").join(format!("{id}.bin")).exists();
    let mut out: Vec<String> =
        crate::VOICES.iter().filter(|v| has(v.id)).map(|v| v.id.to_string()).collect();
    let Ok(rd) = std::fs::read_dir(dir.join("voices")) else { return out };
    for e in rd.flatten() {
        let name = e.file_name().to_string_lossy().trim_end_matches(".bin").to_string();
        if !out.contains(&name) && crate::voice(&name).is_none() {
            out.push(name);
        }
    }
    out
}
