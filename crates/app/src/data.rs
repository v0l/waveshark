//! The cached reference datasets, and the threads that keep them current.
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
use datasets::gateways::Gateway;
use datasets::radioid::{Repeater, Users};
use datasets::{Cache, When};
use parking_lot::RwLock;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::sync::OnceLock;
use std::time::{SystemTime, UNIX_EPOCH};

/// The zoom at which airports first appear on the map. Below this the view is
/// wide enough that every marker would be a blob under a handful of aircraft,
/// and the range rings already say where the interesting things are.
pub const SHOW_ZOOM: f64 = 9.0;

/// The airports the map draws.
///
/// Handed out as `&'static [Airport]` because a tooltip borrows an airport
/// across a frame and the alternative is cloning the row on every hover. The
/// snapshot is leaked rather than freed: it is replaced only when a refresh
/// finds a new file, and a reader holding the old one has no way to say when
/// it is done with it.
static AIRPORTS: RwLock<&'static [Airport]> = RwLock::new(&[]);

pub fn airports() -> &'static [Airport] {
    *AIRPORTS.read()
}

static USERS: RwLock<Option<Arc<Users>>> = RwLock::new(None);
static NXDN: RwLock<Option<Arc<Users>>> = RwLock::new(None);
static REPEATERS: RwLock<Option<Arc<Vec<Repeater>>>> = RwLock::new(None);
static GATEWAYS: RwLock<Option<Arc<Vec<Gateway>>>> = RwLock::new(None);

/// The DMR ID registry: what the number in a DMR frame belongs to.
///
/// Asking starts the load and returns nothing; the answer is there a few
/// seconds later. Nothing decodes DMR yet, so this and the two below have no
/// caller in the tree: they are the half of the lookup that does not depend
/// on the decoder.
#[allow(dead_code)]
pub fn dmr_users() -> Option<Arc<Users>> {
    on_demand(Which::DmrIds, &USERS)
}

/// The NXDN ID registry, the same question for NXDN.
#[allow(dead_code)]
pub fn nxdn_users() -> Option<Arc<Users>> {
    on_demand(Which::NxdnIds, &NXDN)
}

/// Registered DMR repeaters, with their output frequency and colour code.
#[allow(dead_code)]
pub fn dmr_repeaters() -> Option<Arc<Vec<Repeater>>> {
    on_demand(Which::Repeaters, &REPEATERS)
}

/// Where the digital voice networks can be reached: an address, a port, and
/// the channels within it that carry the mode spoken there.
#[allow(dead_code)]
pub fn gateways() -> Option<Arc<Vec<Gateway>>> {
    on_demand(Which::Gateways, &GATEWAYS)
}

fn on_demand<T>(which: Which, held: &'static RwLock<Option<Arc<T>>>) -> Option<Arc<T>> {
    if let Some(v) = held.read().clone() {
        return Some(v);
    }
    // A dataset that failed to download is not going to download on the next
    // frame either, so the attempt is made once and then only on request.
    if !which.attempted() {
        load(which, When::IfDue);
    }
    None
}

/// One cached dataset, as the settings pane lists them.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Which {
    Airports,
    Repeaters,
    DmrIds,
    NxdnIds,
    Gateways,
}

impl Which {
    pub const ALL: [Which; 5] = [
        Which::Airports,
        Which::Repeaters,
        Which::DmrIds,
        Which::NxdnIds,
        Which::Gateways,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Which::Airports => "Airports",
            Which::Repeaters => "DMR repeaters",
            Which::DmrIds => "DMR IDs",
            Which::NxdnIds => "NXDN IDs",
            Which::Gateways => "Digital voice gateways",
        }
    }

    /// Where it comes from, for the line under the name.
    pub fn publisher(self) -> &'static str {
        match self {
            Which::Airports => "ourairports.com",
            // One row, one publisher to name. Several is a list nobody can
            // read in a caption, so the pane says how many instead.
            Which::Gateways => match datasets::gateways::HOST_FILES {
                [one] => one.publisher,
                _ => "several host files",
            },
            _ => "radioid.net",
        }
    }

    /// What the dataset is for, so the pane says why it is being downloaded.
    pub fn about(self) -> &'static str {
        match self {
            Which::Airports => {
                "Airfields and their tower, ground and ATIS frequencies, drawn on the map \
                 under the aircraft."
            }
            Which::Repeaters => {
                "Registered DMR repeaters with their output frequency, offset and colour code."
            }
            Which::DmrIds => {
                "Every registered DMR ID. A digital voice frame carries a number, and this \
                 is what turns it into a callsign without asking anybody over the network."
            }
            Which::NxdnIds => "The same registry for NXDN.",
            Which::Gateways => {
                "Where the digital voice networks can be reached: the address and port of \
                 every reflector, and which of its channels carry the mode spoken there."
            }
        }
    }

    fn sources(self) -> Vec<datasets::Source> {
        use datasets::{airports, gateways, radioid};
        match self {
            Which::Airports => vec![airports::airports_source(), airports::frequencies_source()],
            Which::Repeaters => vec![radioid::repeaters_source()],
            Which::DmrIds => vec![radioid::users_source()],
            Which::NxdnIds => vec![radioid::nxdn_source()],
            Which::Gateways => gateways::sources(),
        }
    }

    fn index(self) -> usize {
        Self::ALL.iter().position(|w| *w == self).unwrap_or(0)
    }

    /// How many rows are held, or `None` when it is not loaded.
    fn rows(self) -> Option<usize> {
        match self {
            Which::Airports => match airports().len() {
                0 => None,
                n => Some(n),
            },
            Which::Repeaters => REPEATERS.read().as_ref().map(|r| r.len()),
            Which::DmrIds => USERS.read().as_ref().map(|u| u.len()),
            Which::NxdnIds => NXDN.read().as_ref().map(|u| u.len()),
            Which::Gateways => GATEWAYS.read().as_ref().map(|g| g.len()),
        }
    }

    fn attempted(self) -> bool {
        WORK[self.index()].attempted.load(Ordering::Acquire)
    }
}

/// What a load or refresh is doing, for the settings pane to draw. One slot
/// per dataset, in [`Which::ALL`] order.
struct Work {
    busy: AtomicBool,
    attempted: AtomicBool,
    error: RwLock<Option<String>>,
}

impl Work {
    const fn new() -> Self {
        Self {
            busy: AtomicBool::new(false),
            attempted: AtomicBool::new(false),
            error: RwLock::new(None),
        }
    }
}

static WORK: [Work; 5] = [Work::new(), Work::new(), Work::new(), Work::new(), Work::new()];

/// A dataset as the settings pane shows it: what is held, how big it is on
/// disk, when it was last checked, and whatever went wrong last time.
pub struct Row {
    pub which: Which,
    pub rows: Option<usize>,
    /// Bytes on disk across the dataset's files. Zero when nothing is cached.
    pub bytes: u64,
    /// Seconds since the last successful check, or `None` if never checked.
    pub checked_ago: Option<u64>,
    pub busy: bool,
    pub error: Option<String>,
}

pub fn status() -> Vec<Row> {
    let cache = cache();
    Which::ALL
        .into_iter()
        .map(|which| {
            let mut bytes = 0;
            let mut oldest: Option<u64> = None;
            let mut present = true;
            for src in which.sources() {
                let s = cache.map(|c| c.status(&src)).unwrap_or_default();
                match s.bytes {
                    Some(b) => bytes += b,
                    None => present = false,
                }
                // A dataset of two files is as fresh as its stalest half. A
                // stamp of zero is a clock that was not readable when the
                // file landed, which is not a check in 1970.
                if let Some(c) = s.checked.filter(|c| *c > 0) {
                    oldest = Some(oldest.map_or(c, |o: u64| o.min(c)));
                }
            }
            let w = &WORK[which.index()];
            Row {
                which,
                rows: which.rows(),
                bytes,
                checked_ago: present.then(|| oldest.map(|c| now().saturating_sub(c))).flatten(),
                busy: w.busy.load(Ordering::Acquire),
                error: w.error.read().clone(),
            }
        })
        .collect()
}

pub fn cache_dir() -> Option<PathBuf> {
    cache().map(|c| c.dir().to_path_buf())
}

/// Check now, whatever the age of the last check, and reload what changed.
/// This is the refresh button.
pub fn refresh(which: Which) {
    load(which, When::Now);
}

/// Load or refresh one dataset on a thread of its own, so a slow 85 MB
/// download does not hold up the three small ones or the frame.
fn load(which: Which, when: When) {
    let w = &WORK[which.index()];
    if w.busy.swap(true, Ordering::AcqRel) {
        return;
    }
    w.attempted.store(true, Ordering::Release);
    let name = which.label();
    let started = std::thread::Builder::new()
        .name(format!("dataset-{}", which.index()))
        .spawn(move || {
            let outcome = cache().map_or_else(
                || Err("no cache directory".to_string()),
                |c| work(which, c, when).map_err(|e| e.to_string()),
            );
            match &outcome {
                Ok(()) => tracing::info!(dataset = name, "dataset ready"),
                Err(e) => tracing::warn!(dataset = name, "dataset unavailable: {e}"),
            }
            *WORK[which.index()].error.write() = outcome.err();
            WORK[which.index()].busy.store(false, Ordering::Release);
        })
        .is_ok();
    if !started {
        w.busy.store(false, Ordering::Release);
    }
}

/// Read what is cached, then ask whether it changed. On a cold cache the
/// first step is the download and the second answers 304 straight away; on a
/// warm one the first step is a file read.
fn work(which: Which, cache: &Cache, when: When) -> Result<(), datasets::Error> {
    use datasets::{airports, gateways, radioid};
    match which {
        Which::Airports => {
            if airports().is_empty() {
                publish_airports(airports::load(cache)?);
            }
            if let Some(a) = airports::refresh(cache, when)? {
                publish_airports(a);
            }
        }
        Which::Repeaters => {
            if REPEATERS.read().is_none() {
                *REPEATERS.write() = Some(Arc::new(radioid::load_repeaters(cache)?));
            }
            if let Some(r) = radioid::refresh_repeaters(cache, when)? {
                *REPEATERS.write() = Some(Arc::new(r));
            }
        }
        Which::DmrIds => {
            if USERS.read().is_none() {
                *USERS.write() = Some(Arc::new(radioid::load_users(cache)?));
            }
            if let Some(u) = radioid::refresh_users(cache, when)? {
                *USERS.write() = Some(Arc::new(u));
            }
        }
        Which::NxdnIds => {
            if NXDN.read().is_none() {
                *NXDN.write() = Some(Arc::new(radioid::load_nxdn(cache)?));
            }
            if let Some(u) = radioid::refresh_nxdn(cache, when)? {
                *NXDN.write() = Some(Arc::new(u));
            }
        }
        Which::Gateways => {
            if GATEWAYS.read().is_none() {
                *GATEWAYS.write() = Some(Arc::new(gateways::load(cache)?));
            }
            if let Some(g) = gateways::refresh(cache, when)? {
                *GATEWAYS.write() = Some(Arc::new(g));
            }
        }
    }
    Ok(())
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

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// Load what the map needs. The registries are left alone: they are large,
/// and nothing has asked them a question yet.
pub fn start() {
    load(Which::Airports, When::IfDue);
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
        datasets::airports::refresh(&cache, When::Now).map(|u| u.is_some()),
        datasets::airports::load(&cache).map(|a| a.len()),
    );
    each(
        "dmr repeaters",
        datasets::radioid::refresh_repeaters(&cache, When::Now).map(|u| u.is_some()),
        datasets::radioid::load_repeaters(&cache).map(|r| r.len()),
    );
    each(
        "nxdn ids",
        datasets::radioid::refresh_nxdn(&cache, When::Now).map(|u| u.is_some()),
        datasets::radioid::load_nxdn(&cache).map(|u| u.len()),
    );
    each(
        "dmr ids",
        datasets::radioid::refresh_users(&cache, When::Now).map(|u| u.is_some()),
        datasets::radioid::load_users(&cache).map(|u| u.len()),
    );
    each(
        "gateways",
        datasets::gateways::refresh(&cache, When::Now).map(|u| u.is_some()),
        datasets::gateways::load(&cache).map(|g| g.len()),
    );
}

fn each(
    what: &str,
    refreshed: Result<bool, datasets::Error>,
    count: Result<usize, datasets::Error>,
) {
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

/// Sizes in the settings pane, where a byte count is noise and a rounded
/// number is the whole point.
pub fn fmt_bytes(b: u64) -> String {
    match b {
        0 => "—".into(),
        b if b < 1 << 20 => format!("{} kB", b >> 10),
        b => format!("{:.1} MB", b as f64 / (1u64 << 20) as f64),
    }
}

/// How long ago, in the coarsest unit that still says something.
pub fn fmt_ago(secs: u64) -> String {
    match secs {
        s if s < 90 => "just now".into(),
        s if s < 5400 => format!("{} min ago", s / 60),
        s if s < 172_800 => format!("{} h ago", s / 3600),
        s => format!("{} days ago", s / 86_400),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_round_to_something_readable() {
        assert_eq!(fmt_bytes(0), "—");
        assert_eq!(fmt_bytes(930_667), "908 kB");
        assert_eq!(fmt_bytes(84_506_836), "80.6 MB");
    }

    #[test]
    fn ages_read_as_a_person_would_say_them() {
        assert_eq!(fmt_ago(5), "just now");
        assert_eq!(fmt_ago(600), "10 min ago");
        assert_eq!(fmt_ago(7200), "2 h ago");
        assert_eq!(fmt_ago(400_000), "4 days ago");
    }

    #[test]
    fn every_dataset_has_at_least_one_source_and_its_own_slot() {
        for w in Which::ALL {
            assert!(!w.sources().is_empty(), "{} has no source", w.label());
        }
        let idx: Vec<usize> = Which::ALL.iter().map(|w| w.index()).collect();
        assert_eq!(idx, [0, 1, 2, 3, 4], "slot indices must be distinct");
        assert_eq!(WORK.len(), Which::ALL.len());
    }
}
