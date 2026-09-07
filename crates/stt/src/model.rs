//! Finding the three files a Whisper model is, without a network.
//!
//! A model is a config, a tokenizer and weights. Kept as paths rather than as
//! a repo name because the receiver must work with no route to the internet:
//! the files are placed once, by hand or by the `hub` feature, and after that
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
        // The English-only models are named for it and their tokenizer has no
        // language tokens; reading the name is enough and avoids loading the
        // tokenizer twice.
        let name = dir
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or_default()
            .to_lowercase();
        let flavour = if name.contains(".en") || name.contains("-en") {
            Flavour::English
        } else {
            Flavour::Multilingual
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

/// Fetch a model from the hub into `dir`, once.
#[cfg(feature = "hub")]
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
