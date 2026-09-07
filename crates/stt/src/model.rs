//! Finding the three files a Whisper model is, without a network.
//!
//! A model is a config, a tokenizer and weights. Kept as paths rather than as
//! a repo name because the receiver must work with no route to the internet:
//! the files are fetched once, or placed by hand, and after that
//! transcription is as offline as demodulation is.

use common::{Error, Result};
use std::path::{Path, PathBuf};

/// Which Whisper the files are, insofar as it changes what the decoder does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flavour {
    /// English-only: no language token in the prompt, and asking for one is
    /// an error rather than a hint.
    English,
    /// Multilingual: the prompt carries a language token.
    Multilingual,
}

/// The files of one model, and whether the weights are GGUF.
#[derive(Clone, Debug)]
pub struct Files {
    pub config: PathBuf,
    pub tokenizer: PathBuf,
    pub weights: PathBuf,
    pub quantized: bool,
    pub flavour: Flavour,
}

impl Files {
    /// The layout Hugging Face publishes: `config.json`, `tokenizer.json` and
    /// either `model.safetensors` or a single `.gguf`.
    pub fn in_dir(dir: impl AsRef<Path>) -> Result<Self> {
        let dir = dir.as_ref();
        let config = required(dir, "config.json")?;
        let tokenizer = required(dir, "tokenizer.json")?;
        let (weights, quantized) = match first_existing(dir, &["model.safetensors"]) {
            Some(p) => (p, false),
            None => (gguf_in(dir)?, true),
        };
        // Read out of the config rather than off the directory name. An
        // English-only model has one token fewer, because it carries no
        // language tokens to choose between, and a model fetched into
        // `models/whisper` has a name that says nothing at all: guessing from
        // it put a language token in front of a decoder that has none and
        // every transcript came back as the wrong words.
        let flavour = match vocab_size(&config) {
            Some(n) if n <= 51_864 => Flavour::English,
            _ => Flavour::Multilingual,
        };
        Ok(Self {
            config,
            tokenizer,
            weights,
            quantized,
            flavour,
        })
    }
}

/// How many tokens the model was trained with, from its config.
fn vocab_size(config: &Path) -> Option<usize> {
    let text = std::fs::read_to_string(config).ok()?;
    let v: serde_json::Value = serde_json::from_str(&text).ok()?;
    v.get("vocab_size")?.as_u64().map(|n| n as usize)
}

fn required(dir: &Path, name: &str) -> Result<PathBuf> {
    let p = dir.join(name);
    if p.exists() {
        Ok(p)
    } else {
        Err(Error::other(format!("{} has no {name}", dir.display())))
    }
}

fn first_existing(dir: &Path, names: &[&str]) -> Option<PathBuf> {
    names.iter().map(|n| dir.join(n)).find(|p| p.exists())
}

fn gguf_in(dir: &Path) -> Result<PathBuf> {
    let mut found: Vec<PathBuf> = std::fs::read_dir(dir)?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().and_then(|e| e.to_str()) == Some("gguf"))
        .collect();
    found.sort();
    found.into_iter().next().ok_or_else(|| {
        Error::other(format!(
            "{} has neither model.safetensors nor a .gguf",
            dir.display()
        ))
    })
}

/// The model fetched when nobody names one.
///
/// English-only and small enough to download without asking: 74 MB of
/// weights, and it reads squelched FM and vocoded speech about as well as
/// anything does. A bigger one is a directory away.
pub const DEFAULT_REPO: &str = "openai/whisper-base.en";

/// The files in `dir`, fetching them first if they are not there.
///
/// The download happens on the worker thread, on the first call worth
/// reading, so a receiver that never hears speech never reaches the network
/// and one that does is not made to wait at startup.
pub fn ensure(repo: &str, dir: impl AsRef<Path>) -> Result<Files> {
    let dir = dir.as_ref();
    match Files::in_dir(dir) {
        Ok(f) => Ok(f),
        Err(_) => fetch(repo, "main", dir),
    }
}

/// Fetch a model from the hub into `dir`, once.
pub fn fetch(repo: &str, revision: &str, dir: impl AsRef<Path>) -> Result<Files> {
    use hf_hub::api::sync::ApiBuilder;

    let dir = dir.as_ref();
    std::fs::create_dir_all(dir)?;
    let api = ApiBuilder::new()
        .build()
        .map_err(|e| Error::other(format!("hub: {e}")))?
        .repo(hf_hub::Repo::with_revision(
            repo.to_string(),
            hf_hub::RepoType::Model,
            revision.to_string(),
        ));
    for name in ["config.json", "tokenizer.json", "model.safetensors"] {
        let src = api
            .get(name)
            .map_err(|e| Error::other(format!("hub {name}: {e}")))?;
        let dst = dir.join(name);
        if !dst.exists() {
            std::fs::copy(&src, &dst)?;
        }
    }
    Files::in_dir(dir)
}
