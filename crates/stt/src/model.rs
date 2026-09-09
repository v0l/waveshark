//! Finding the three files a Whisper model is, without a network.
//!
//! A model is a config, a tokenizer and weights. Kept as paths rather than as
//! a repo name because the receiver must work with no route to the internet:
//! the files are fetched once, or placed by hand, and after that
//! transcription is as offline as demodulation is.

use crate::Family;
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
    /// Which decoder reads these files, from `model_type` in the config.
    pub family: Family,
}

impl Files {
    /// The layout Hugging Face publishes: `config.json`, `tokenizer.json` and
    /// either `model.safetensors`, an index over shards of it, or a single
    /// `.gguf`. For a sharded model `weights` is the index, and `bytes`
    /// counts the shards.
    pub fn in_dir(dir: impl AsRef<Path>) -> Result<Self> {
        let dir = dir.as_ref();
        let config = required(dir, "config.json")?;
        let tokenizer = required(dir, "tokenizer.json")?;
        let (weights, quantized) =
            match first_existing(dir, &["model.safetensors", "model.safetensors.index.json"]) {
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
        let family = match model_type(&config).as_deref() {
            Some("qwen3_asr") => Family::Qwen3Asr,
            _ => Family::Whisper,
        };
        Ok(Self { config, tokenizer, weights, quantized, flavour, family })
    }

    /// What the three files take on disc. Shown beside the directory, since
    /// "a model is here" and "90 MB of model is here" are different claims to
    /// somebody deciding whether to fetch a larger one.
    pub fn bytes(&self) -> u64 {
        let mut files = vec![self.config.clone(), self.tokenizer.clone(), self.weights.clone()];
        if let Some(dir) = self.weights.parent() {
            files.extend(shards(&self.weights).into_iter().map(|s| dir.join(s)));
        }
        files.into_iter().filter_map(|p| std::fs::metadata(p).ok()).map(|m| m.len()).sum()
    }
}

/// The shard files an index names, or nothing for a file that is not one.
fn shards(index: &Path) -> Vec<String> {
    if index.file_name().and_then(|n| n.to_str()) != Some("model.safetensors.index.json") {
        return Vec::new();
    }
    let Ok(text) = std::fs::read_to_string(index) else {
        return Vec::new();
    };
    let Ok(v) = serde_json::from_str::<serde_json::Value>(&text) else {
        return Vec::new();
    };
    let mut out: Vec<String> = v
        .get("weight_map")
        .and_then(|m| m.as_object())
        .map(|m| m.values().filter_map(|v| v.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    out.sort();
    out.dedup();
    out
}

/// How many tokens the model was trained with, from its config.
fn vocab_size(config: &Path) -> Option<usize> {
    let text = std::fs::read_to_string(config).ok()?;
    let v: serde_json::Value = serde_json::from_str(&text).ok()?;
    v.get("vocab_size")?.as_u64().map(|n| n as usize)
}

/// What the config says the architecture is.
fn model_type(config: &Path) -> Option<String> {
    let text = std::fs::read_to_string(config).ok()?;
    let v: serde_json::Value = serde_json::from_str(&text).ok()?;
    v.get("model_type")?.as_str().map(str::to_string)
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
        Error::other(format!("{} has neither model.safetensors nor a .gguf", dir.display()))
    })
}

/// The model fetched when nobody names one.
///
/// English-only and small enough to download without asking: 74 MB of
/// weights, and it reads squelched FM and vocoded speech about as well as
/// anything does. A bigger one is a directory away.
pub const DEFAULT_REPO: &str = "openai/whisper-base.en";

/// How far a download has got, reported as it goes.
///
/// A model is between 74 MB and several gigabytes over somebody's home
/// connection, and without this the interface can only say "downloading" for
/// as long as it takes, which is indistinguishable from a fetch that has
/// hung.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fetching {
    /// The file being fetched now.
    pub file: String,
    /// Bytes of it that have arrived, and what it is altogether. The total
    /// is zero until the hub answers with a length.
    pub done: u64,
    pub total: u64,
    /// Files finished before this one, and how many there are to do. The
    /// count is only known as the fetch walks the repository, so it grows.
    pub files_done: usize,
    pub files: usize,
}

/// What the hub calls as bytes arrive, folded into a [`Fetching`] and handed
/// on. Held by reference so one report survives every file of a fetch.
struct Report<'a, 'b> {
    seen: &'a std::cell::RefCell<Fetching>,
    on: &'a std::cell::RefCell<OnProgress<'b>>,
}

impl hf_hub::api::Progress for Report<'_, '_> {
    fn init(&mut self, size: usize, filename: &str) {
        let mut f = self.seen.borrow_mut();
        f.file = filename.to_string();
        f.done = 0;
        f.total = size as u64;
        (self.on.borrow_mut())(&f);
    }

    fn update(&mut self, size: usize) {
        let mut f = self.seen.borrow_mut();
        f.done += size as u64;
        (self.on.borrow_mut())(&f);
    }

    fn finish(&mut self) {
        let mut f = self.seen.borrow_mut();
        f.done = f.total;
        (self.on.borrow_mut())(&f);
    }
}

/// Somewhere to report progress to. A closure rather than a trait, because
/// the one caller keeps it behind a mutex the interface reads.
pub type OnProgress<'a> = &'a mut dyn FnMut(&Fetching);

/// The files in `dir`, fetching them first if they are not there.
///
/// The download happens on the worker thread, on the first call worth
/// reading, so a receiver that never hears speech never reaches the network
/// and one that does is not made to wait at startup.
pub fn ensure(repo: &str, dir: impl AsRef<Path>) -> Result<Files> {
    ensure_with(repo, dir, &mut |_| {})
}

/// The same, telling `on` how the download is going.
pub fn ensure_with(repo: &str, dir: impl AsRef<Path>, on: OnProgress<'_>) -> Result<Files> {
    let dir = dir.as_ref();
    match Files::in_dir(dir) {
        Ok(f) => Ok(f),
        Err(_) => fetch_with(repo, "main", dir, on),
    }
}

/// Fetch a model from the hub into `dir`, once.
pub fn fetch(repo: &str, revision: &str, dir: impl AsRef<Path>) -> Result<Files> {
    fetch_with(repo, revision, dir, &mut |_| {})
}

/// The same, reporting each file's progress to `on`.
pub fn fetch_with(
    repo: &str,
    revision: &str,
    dir: impl AsRef<Path>,
    on: OnProgress<'_>,
) -> Result<Files> {
    use hf_hub::api::sync::ApiBuilder;

    let dir = dir.as_ref();
    std::fs::create_dir_all(dir)?;
    let api = ApiBuilder::new().build().map_err(|e| Error::other(format!("hub: {e}")))?.repo(
        hf_hub::Repo::with_revision(
            repo.to_string(),
            hf_hub::RepoType::Model,
            revision.to_string(),
        ),
    );
    let seen = std::cell::RefCell::new(Fetching::default());
    let on = std::cell::RefCell::new(on);
    let get = |name: &str| -> Result<PathBuf> {
        {
            let mut f = seen.borrow_mut();
            f.file = name.to_string();
            f.done = 0;
            f.total = 0;
            f.files = f.files.max(f.files_done + 1);
        }
        let src = api
            .download_with_progress(name, Report { seen: &seen, on: &on })
            .map_err(|e| Error::other(format!("hub {name}: {e}")))?;
        {
            let mut f = seen.borrow_mut();
            f.files_done += 1;
            f.done = f.total;
        }
        (on.borrow_mut())(&seen.borrow());
        let dst = dir.join(name);
        if !dst.exists() {
            std::fs::copy(&src, &dst)?;
        }
        Ok(dst)
    };
    let config = get("config.json")?;
    // One file, or an index and the shards it names. Asking the hub which
    // rather than reading the listing: a 404 on the index is the answer.
    match get("model.safetensors.index.json") {
        Ok(index) => {
            for shard in shards(&index) {
                get(&shard)?;
            }
        }
        Err(_) => {
            get("model.safetensors")?;
        }
    }
    // Qwen3-ASR publishes no tokenizer.json, only the vocabulary, the merges
    // and the special tokens it would be built from. Whisper publishes the
    // built one.
    if model_type(&config).as_deref() == Some("qwen3_asr") {
        let read = |name: &str| -> Result<String> { Ok(std::fs::read_to_string(get(name)?)?) };
        let vocab = read("vocab.json")?;
        let merges = read("merges.txt")?;
        let tok_config = read("tokenizer_config.json")?;
        let json = crate::qwen3::tokenizer_json(&vocab, &merges, &tok_config)
            .map_err(|e| Error::other(format!("qwen3 tokenizer: {e}")))?;
        std::fs::write(dir.join("tokenizer.json"), json)?;
    } else {
        get("tokenizer.json")?;
    }
    Files::in_dir(dir)
}
