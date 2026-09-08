//! The satellites being watched: their elements, where they are now, and
//! when they are next overhead.
//!
//! Two costs, kept apart because they are different by three orders of
//! magnitude. Asking where one satellite is now is a single propagation and
//! is done while drawing, once per satellite per frame. Asking when a
//! hundred satellites next rise is a search over a day, thousands of
//! propagations each, and that runs on a thread of its own and is published
//! when it is finished.
//!
//! What is held here is derived, never authoritative: the elements are the
//! dataset's, the station is the receiver's, and everything below is thrown
//! away and recomputed when either changes.

use datasets::tle::Group;
use orbit::{Pass, Sat, Station};
use parking_lot::{Mutex, RwLock};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;

/// How far ahead passes are searched. A day covers everything in low orbit
/// several times over and is a table a person can read to the end of.
pub const WINDOW_S: i64 = 86_400;

/// Recomputed no more often than this, however much the station jitters: a
/// GPS fix moving by metres does not change when anything rises.
const RECOMPUTE_EVERY_S: u64 = 300;

/// A pass, and what it is a pass of.
#[derive(Clone, Debug, PartialEq)]
pub struct Upcoming {
    pub norad: u64,
    pub name: String,
    pub pass: Pass,
}

/// The propagators for one group, wound up once and kept.
///
/// Winding up a hundred sets costs milliseconds and is done off the elements
/// as published, so this is rebuilt when the dataset is refreshed rather than
/// kept in step with it by hand.
#[derive(Default)]
pub struct Sky {
    sats: Vec<Arc<Sat>>,
    /// Which group these came from, so a change of group rebuilds them.
    group: Option<&'static Group>,
    /// How many sets the dataset held when they were built, which is what
    /// says a refresh has landed.
    from_rows: usize,
}

static SKY: RwLock<Option<Arc<Sky>>> = RwLock::new(None);
static PASSES: RwLock<Option<Arc<Vec<Upcoming>>>> = RwLock::new(None);
static COMPUTED_AT: AtomicU64 = AtomicU64::new(0);
static COMPUTING: AtomicBool = AtomicBool::new(false);
/// What the last computation was for, so a change of station, group or
/// threshold is noticed rather than waited out.
static COMPUTED_FOR: Mutex<Option<(&'static Group, i64, i64, i64)>> = Mutex::new(None);

impl Sky {
    pub fn sats(&self) -> &[Arc<Sat>] {
        &self.sats
    }

    /// One object by its catalogue number.
    pub fn get(&self, norad: u64) -> Option<&Arc<Sat>> {
        self.sats.iter().find(|s| s.norad == norad)
    }
}

/// The propagators for a group, building them if the elements have landed.
///
/// `None` means the dataset is not here yet, which is also what starts the
/// download: nothing propagates until somebody looks.
pub fn sky(group: &'static Group) -> Option<Arc<Sky>> {
    let held = SKY.read().clone();
    let rows = crate::data::satellites(group);
    if let (Some(s), Some(r)) = (&held, &rows) {
        if s.group == Some(group) && s.from_rows == r.len() {
            return held;
        }
    }
    let rows = rows?;
    let mut sats = Vec::with_capacity(rows.len());
    let mut refused = 0usize;
    for e in rows.iter() {
        match Sat::from_elements(e) {
            Ok(s) => sats.push(Arc::new(s)),
            // A set the model will not take is one object missing, not a
            // group that fails: the file is a hundred independent objects.
            Err(_) => refused += 1,
        }
    }
    if refused > 0 {
        tracing::warn!(group = group.name, refused, "elements the propagator would not take");
    }
    let built = Arc::new(Sky { sats, group: Some(group), from_rows: rows.len() });
    *SKY.write() = Some(built.clone());
    Some(built)
}

/// Passes over the station, soonest first, or `None` until the first search
/// has finished.
///
/// Asking is what starts the search, and starting it again while it runs is
/// nothing: the answer arrives when it arrives.
pub fn passes(
    group: &'static Group,
    at: Station,
    now_s: i64,
    min_el_deg: f64,
) -> Option<Arc<Vec<Upcoming>>> {
    let key =
        (group, (at.lat_deg * 1000.0) as i64, (at.lon_deg * 1000.0) as i64, min_el_deg as i64);
    let stale = COMPUTED_FOR.lock().as_ref() != Some(&key)
        || now_s as u64 >= COMPUTED_AT.load(Ordering::Relaxed) + RECOMPUTE_EVERY_S;
    if stale {
        recompute(group, at, now_s, min_el_deg, key);
    }
    PASSES.read().clone()
}

fn recompute(
    group: &'static Group,
    at: Station,
    now_s: i64,
    min_el_deg: f64,
    key: (&'static Group, i64, i64, i64),
) {
    if COMPUTING.swap(true, Ordering::SeqCst) {
        return;
    }
    let Some(sky) = sky(group) else {
        COMPUTING.store(false, Ordering::SeqCst);
        return;
    };
    let _ = std::thread::Builder::new().name("sat-passes".into()).spawn(move || {
        let mut out: Vec<Upcoming> = Vec::new();
        for s in sky.sats() {
            for pass in s.passes(at, now_s, WINDOW_S, min_el_deg) {
                out.push(Upcoming { norad: s.norad, name: s.name.clone(), pass });
            }
        }
        out.sort_by_key(|u| u.pass.rise_s);
        *PASSES.write() = Some(Arc::new(out));
        COMPUTED_AT.store(now_s as u64, Ordering::Relaxed);
        *COMPUTED_FOR.lock() = Some(key);
        COMPUTING.store(false, Ordering::SeqCst);
    });
}

/// Whether a search is running, for a view that would otherwise look empty
/// rather than busy.
pub fn computing() -> bool {
    COMPUTING.load(Ordering::Relaxed)
}

pub fn now_s() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

/// `in 4m 20s`, `now`, or `12m ago`: what a person reads off a pass table.
pub fn in_when(secs: i64) -> String {
    let (word, s) = match secs {
        s if s < -1 => ("ago", -s),
        -1..=30 => return "now".into(),
        s => ("in", s),
    };
    let (h, m, sec) = (s / 3600, s % 3600 / 60, s % 60);
    let body = match (h, m) {
        (0, 0) => format!("{sec}s"),
        (0, _) => format!("{m}m {sec:02}s"),
        _ => format!("{h}h {m:02}m"),
    };
    match word {
        "in" => format!("in {body}"),
        _ => format!("{body} ago"),
    }
}

/// `14:32:05`, in UTC, which is the clock everything about an orbit is in.
pub fn utc_hms(at_s: i64) -> String {
    let s = at_s.rem_euclid(86_400);
    format!("{:02}:{:02}:{:02}", s / 3600, s % 3600 / 60, s % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_countdown_reads_as_a_person_would_say_it() {
        assert_eq!(in_when(0), "now");
        assert_eq!(in_when(45), "in 45s");
        assert_eq!(in_when(3 * 60 + 7), "in 3m 07s");
        assert_eq!(in_when(2 * 3600 + 5 * 60), "in 2h 05m");
        assert_eq!(in_when(-90), "1m 30s ago");
    }

    #[test]
    fn a_time_of_day_is_utc_and_wraps() {
        assert_eq!(utc_hms(0), "00:00:00");
        assert_eq!(utc_hms(1_705_322_433), "12:40:33");
    }
}
