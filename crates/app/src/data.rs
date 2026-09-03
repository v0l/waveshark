//! The cached reference datasets, and the thread that keeps them current.
//!
//! `datasets` knows how to fetch and parse; this decides when, and holds what
//! came back where a frame can read it without waiting. Nothing here blocks
//! the UI: a dataset is absent until it is not, and every view that uses one
//! has to draw without it.
//!
//! Airports load at startup because the map wants them within a second of
//! opening. The radioid registries do not, because the DMR user dump is 85 MB
//! and nothing asks it a question until a digital voice frame arrives, so
//! they load on first use and stay loaded.

use datasets::airports::Airport;
use datasets::radioid::{Repeater, Users};
use datasets::Cache;
use parking_lot::RwLock;
use std::sync::{Arc, OnceLock};

/// The airports the map draws.
///
/// Handed out as `&'static [Airport]` because a tooltip borrows an airport
/// across a frame and the alternative is cloning the row on every hover. The
/// snapshot is leaked rather than freed: it is replaced at most once per run,
/// when a refresh finds a new file, and a reader holding the old one has no
/// way to say when it is done with it.
static AIRPORTS: RwLock<&'static [Airport]> = RwLock::new(&[]);

/// The zoom at which airports first appear on the map. Below this the view is
/// wide enough that every marker would be a blob under a handful of aircraft,
/// and the range rings already say where the interesting things are.
pub const SHOW_ZOOM: f64 = 9.0;

pub fn airports() -> &'static [Airport] {
    *AIRPORTS.read()
}

fn publish_airports(v: Vec<Airport>) {
    tracing::info!(count = v.len(), "airports loaded");
    *AIRPORTS.write() = Vec::leak(v);
}

fn cache() -> Option<&'static Cache> {
    static CACHE: OnceLock<Option<Cache>> = OnceLock::new();
    CACHE
        .get_or_init(|| match Cache::at_default_dir() {
            Ok(c) => Some(c),
            Err(e) => {
                tracing::warn!("no dataset cache: {e}");
                None
            }
        })
        .as_ref()
}

/// Load what the map needs, then ask whether it changed. Both happen on one
/// background thread: the first call is a download on a cold cache and a file
/// read on a warm one, and the second is a conditional request that usually
/// answers 304 and costs nothing.
pub fn start() {
    std::thread::Builder::new()
        .name("datasets".into())
        .spawn(|| {
            let Some(cache) = cache() else { return };
            match datasets::airports::load(cache) {
                Ok(a) => publish_airports(a),
                Err(e) => tracing::warn!("airports unavailable: {e}"),
            }
            match datasets::airports::refresh(cache) {
                Ok(Some(a)) => publish_airports(a),
                Ok(None) => {}
                Err(e) => tracing::warn!("airports not refreshed: {e}"),
            }
        })
        .ok();
}

/// A dataset loaded the first time something asks for it, on a thread of its
/// own, and then held. Reading it while it loads gives `None`, which is the
/// same answer as a machine with no network: a caller that cannot cope with
/// that has no business asking.
struct Lazy<T> {
    held: RwLock<Option<Arc<T>>>,
    loading: std::sync::atomic::AtomicBool,
}

impl<T: Send + Sync + 'static> Lazy<T> {
    const fn new() -> Self {
        Self { held: RwLock::new(None), loading: std::sync::atomic::AtomicBool::new(false) }
    }

    fn get(&'static self, what: &'static str, load: fn(&Cache) -> Result<T, datasets::Error>) -> Option<Arc<T>> {
        if let Some(v) = self.held.read().clone() {
            return Some(v);
        }
        if self.loading.swap(true, std::sync::atomic::Ordering::AcqRel) {
            return None;
        }
        std::thread::Builder::new()
            .name(format!("dataset-{what}"))
            .spawn(move || {
                let Some(cache) = cache() else { return };
                match load(cache) {
                    Ok(v) => *self.held.write() = Some(Arc::new(v)),
                    Err(e) => tracing::warn!("{what} unavailable: {e}"),
                }
                // Left set on failure: a dump that would not download is not
                // going to download on the next frame either, and retrying
                // per frame would hammer the registry.
            })
            .ok();
        None
    }
}

static USERS: Lazy<Users> = Lazy::new();
static NXDN: Lazy<Users> = Lazy::new();
static REPEATERS: Lazy<Vec<Repeater>> = Lazy::new();

/// The DMR ID registry: what the number in a DMR frame belongs to. Nothing
/// decodes DMR yet, so these three have no caller in the tree; they are the
/// half of the lookup that does not depend on the decoder.
#[allow(dead_code)]
pub fn dmr_users() -> Option<Arc<Users>> {
    USERS.get("dmr-users", datasets::radioid::load_users)
}

/// The NXDN ID registry, the same question for NXDN.
#[allow(dead_code)]
pub fn nxdn_users() -> Option<Arc<Users>> {
    NXDN.get("nxdn-users", datasets::radioid::load_nxdn)
}

/// Registered DMR repeaters, with their output frequency and colour code.
#[allow(dead_code)]
pub fn dmr_repeaters() -> Option<Arc<Vec<Repeater>>> {
    REPEATERS.get("dmr-repeaters", datasets::radioid::load_repeaters)
}

/// Download or revalidate every dataset and report what happened, for
/// `--fetch-data`. Warming the cache before going somewhere without a
/// connection is the point, so this waits and prints rather than logging.
pub fn fetch_all() {
    let Ok(cache) = Cache::at_default_dir() else {
        eprintln!("no cache directory");
        return;
    };
    println!("cache: {}", cache.dir().display());
    // Each pair is written refresh first, and arguments evaluate in order, so
    // the count reported is of the file after any update rather than before.
    each(
        "airports",
        datasets::airports::refresh(&cache).map(|u| u.is_some()),
        datasets::airports::load(&cache).map(|a| a.len()),
    );
    each(
        "dmr repeaters",
        datasets::radioid::refresh_repeaters(&cache).map(|u| u.is_some()),
        datasets::radioid::load_repeaters(&cache).map(|r| r.len()),
    );
    each(
        "nxdn ids",
        datasets::radioid::refresh_nxdn(&cache).map(|u| u.is_some()),
        datasets::radioid::load_nxdn(&cache).map(|u| u.len()),
    );
    each(
        "dmr ids",
        datasets::radioid::refresh_users(&cache).map(|u| u.is_some()),
        datasets::radioid::load_users(&cache).map(|u| u.len()),
    );
}

fn each(what: &str, refreshed: Result<bool, datasets::Error>, count: Result<usize, datasets::Error>) {
    let state = match refreshed {
        Ok(true) => "updated".to_string(),
        Ok(false) => "current".to_string(),
        Err(e) => format!("not refreshed: {e}"),
    };
    match count {
        Ok(n) => println!("{what}: {n}, {state}"),
        Err(e) => println!("{what}: {e}"),
    }
}
