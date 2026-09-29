//! The protocol descriptions the receiver runs: the fetched set, and the
//! operator's own files over it. Nothing is built in.
//!
//! `decode::script` holds the set; this is where it is filled from disk.
//! The fetched tree is `datasets::git::PROTOCOLS` in the cache, and a
//! user's files are `$XDG_CONFIG_HOME/waveshark/protocols/*.yaml`, which is
//! where a layout is worked on before it is sent upstream.

use decode::script::{self, Installed};
use parking_lot::RwLock;
use std::path::PathBuf;

#[cfg(not(target_arch = "wasm32"))]
#[path = "protocols/fetched.rs"]
mod published;
#[cfg(target_arch = "wasm32")]
#[path = "protocols/bundled.rs"]
mod published;

#[cfg(target_arch = "wasm32")]
pub use published::fetch;

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
    let mut files = published::files();
    if let Some(dir) = user_dir() {
        let yaml = common::store::list(&dir)
            .into_iter()
            .filter(|p| p.extension().is_some_and(|x| x.eq_ignore_ascii_case("yaml")));
        for p in yaml {
            match common::store::read(&p) {
                Ok(text) => files.push((p.display().to_string(), text)),
                Err(e) => {
                    tracing::warn!(path = %p.display(), "unreadable protocol description: {e}")
                }
            }
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
