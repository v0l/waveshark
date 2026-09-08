//! Where beaconDB thinks a cell is, asked one cell at a time.
//!
//! The OpenCelliD export is a file: a country's cells read off disc, and a
//! position costs a binary search. beaconDB has no export yet, so a position
//! is a request, and a request cannot happen while a frame is being drawn.
//! What is here is the machinery that difference forces: an answer cache the
//! map reads without blocking, a queue of cells nobody has asked about yet,
//! and one thread that asks.
//!
//! Off unless the operator turns it on. Asking beaconDB where a cell is
//! tells beaconDB which cells this receiver has heard, which is a thing an
//! operator should decide rather than discover.
//!
//! A miss is cached as firmly as a hit. Most cells are misses, and a miss
//! that is not remembered is a request every time the map is drawn.

use parking_lot::{Condvar, Mutex, RwLock};
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use survey::beacondb::Cell;

/// What is known about one cell.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Answer {
    /// Queued or in flight.
    Asking,
    /// A position, and the metres it may be out by. beaconDB answers where a
    /// receiver seeing this cell is, which with one cell and nothing else is
    /// the cell's own estimated position and no better than its accuracy.
    At { lat: f64, lon: f64, accuracy_m: f64 },
    /// beaconDB has never heard of it, or would not say.
    Unknown,
}

/// Cells nobody wants to ask about twice.
static ANSWERS: RwLock<Option<HashMap<Cell, Answer>>> = RwLock::new(None);

/// What has been asked for and not yet answered.
static QUEUE: Mutex<VecDeque<Cell>> = Mutex::new(VecDeque::new());
static WAKE: Condvar = Condvar::new();

/// Whether lookups happen at all.
static ON: AtomicBool = AtomicBool::new(false);
static STARTED: AtomicBool = AtomicBool::new(false);

/// A map full of cells would otherwise queue hundreds at once. What is on
/// screen is a few dozen, and the rest can be asked about on the next pass.
const QUEUE_MAX: usize = 64;

/// One request at a time with a gap between them: this is somebody else's
/// server, and a map being panned must not become a flood.
const GAP: std::time::Duration = std::time::Duration::from_millis(250);

pub fn set_lookup(on: bool) {
    ON.store(on, Ordering::Relaxed);
    if !on {
        QUEUE.lock().clear();
    }
}

pub fn lookup_on() -> bool {
    ON.load(Ordering::Relaxed)
}

/// Who to credit where these positions are drawn.
pub const CREDIT: crate::data::Credit = crate::data::Credit {
    name: "beaconDB",
    licence: "public domain",
    url: "https://beacondb.net/",
};

/// What is known about a cell, asking if nothing has been.
///
/// Never blocks: the answer to a cell nobody has asked about is `Asking`,
/// and the map draws nothing for it until the thread comes back.
pub fn position(c: Cell) -> Option<Answer> {
    if !lookup_on() {
        return None;
    }
    if let Some(a) = ANSWERS.read().as_ref().and_then(|m| m.get(&c).copied()) {
        return Some(a);
    }
    ANSWERS.write().get_or_insert_with(HashMap::new).insert(c, Answer::Asking);
    let mut q = QUEUE.lock();
    if q.len() < QUEUE_MAX {
        q.push_back(c);
        WAKE.notify_one();
    } else {
        // Not asked and not remembered as asking, or it would never be asked
        // again once the queue drained.
        if let Some(m) = ANSWERS.write().as_mut() {
            m.remove(&c);
        }
    }
    Some(Answer::Asking)
}

/// Start the thread that asks. Called once, from the interface's setup.
pub fn start() {
    if STARTED.swap(true, Ordering::SeqCst) {
        return;
    }
    let _ = std::thread::Builder::new().name("beacondb-lookup".into()).spawn(run);
}

fn run() {
    loop {
        let next = {
            let mut q = QUEUE.lock();
            loop {
                if let Some(c) = q.pop_front() {
                    break c;
                }
                WAKE.wait(&mut q);
            }
        };
        let answer = match survey::beacondb::locate_cell(next, "gsm") {
            Ok(Some((lat, lon, accuracy_m))) => Answer::At { lat, lon, accuracy_m },
            Ok(None) => Answer::Unknown,
            Err(e) => {
                // A network that is not there is not an answer about the
                // cell: forgetting it means the next draw asks again, which
                // is what an operator who has just reconnected expects.
                tracing::debug!(error = %e, "beacondb lookup failed");
                if let Some(m) = ANSWERS.write().as_mut() {
                    m.remove(&next);
                }
                std::thread::sleep(GAP * 8);
                continue;
            }
        };
        if let Some(m) = ANSWERS.write().as_mut() {
            m.insert(next, answer);
        }
        std::thread::sleep(GAP);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cell(cid: u64) -> Cell {
        Cell { mcc: 272, mnc: 1, lac: 1, cid }
    }

    /// One test rather than two, because the switch and the queue are
    /// process-wide: two tests of them run at once and take each other's
    /// state away.
    #[test]
    fn nothing_is_asked_while_it_is_off_and_nothing_is_asked_twice() {
        set_lookup(false);
        assert_eq!(position(cell(1)), None);
        assert!(QUEUE.lock().is_empty());

        set_lookup(true);
        QUEUE.lock().clear();
        // The first ask queues the cell; the second finds it already asked.
        assert_eq!(position(cell(2)), Some(Answer::Asking));
        assert_eq!(position(cell(2)), Some(Answer::Asking));
        assert_eq!(QUEUE.lock().iter().filter(|c| **c == cell(2)).count(), 1);
        set_lookup(false);
    }
}
