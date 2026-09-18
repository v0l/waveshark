//! The protocol descriptions the receiver runs: the fetched set over the
//! built-in one, and the operator's own files over both.
//!
//! `decode::script` holds the set; this is where it is filled from disk.
//! The fetched tree is `datasets::git::PROTOCOLS` in the cache, and a
//! user's files are `$XDG_CONFIG_HOME/waveshark/protocols/*.yaml`, which is
//! where a layout is worked on before it is sent upstream.

use decode::script::{self, Installed};
use parking_lot::RwLock;
use std::path::{Path, PathBuf};

static LAST: RwLock<Option<Installed>> = RwLock::new(None);

/// What the last load installed and refused
pub fn last() -> Option<Installed> {
    LAST.read().clone()
}

/// `$XDG_CONFIG_HOME/waveshark/protocols`
pub fn user_dir() -> Option<PathBuf> {
    crate::session::Session::path().map(|p| p.with_file_name("protocols"))
}

/// Read every description on disk and install it, later sources winning
pub fn load() -> Installed {
    let mut files: Vec<(String, String)> = Vec::new();
    if let Some(cache) = crate::data::cache() {
        let repo = &datasets::git::PROTOCOLS;
        let root = repo.cache_dir(cache);
        for rel in datasets::git::files_with(repo, cache, "yaml") {
            read_into(&root.join(&rel), &mut files);
        }
    }
    if let Some(dir) = user_dir()
        && let Ok(entries) = std::fs::read_dir(&dir)
    {
        let mut paths: Vec<PathBuf> = entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("yaml")))
            .collect();
        paths.sort();
        for p in paths {
            read_into(&p, &mut files);
        }
    }
    let got = script::install(&files);
    for (path, why) in &got.refused {
        tracing::warn!(path, "protocol description refused: {why}");
    }
    tracing::info!(
        installed = got.names.len(),
        refused = got.refused.len(),
        "protocol descriptions"
    );
    *LAST.write() = Some(got.clone());
    got
}

fn read_into(path: &Path, files: &mut Vec<(String, String)>) {
    match std::fs::read_to_string(path) {
        Ok(text) => files.push((path.display().to_string(), text)),
        Err(e) => tracing::warn!(path = %path.display(), "unreadable protocol description: {e}"),
    }
}
