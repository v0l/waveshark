//! Background TEA1 key recovery: the register search off the hot path.
//!
//! [`crate::tea::recover_tea1`] is the arithmetic; it sweeps a slice of the
//! 2^32 register space, which takes far too long for the radio thread. This
//! runs it on a pool of worker threads and reports through a channel, so a
//! node can start a search when it has collision material and poll for the
//! answer without ever blocking. A found register is a TEA1 key
//! ([`crate::tea::Key::Tea1`]).

use crate::tea::{recover_tea1, Collision};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, TryRecvError};
use std::sync::Arc;

/// What a running search has to say when polled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Progress {
    /// Still sweeping.
    Running,
    /// Found the register: a TEA1 short key.
    Found(u32),
    /// The whole space was swept and nothing fit; the collisions were not a
    /// real equal-plaintext set, or the timestamps were wrong.
    Exhausted,
}

/// A recovery in flight. Dropping it signals the workers to stop.
pub struct Search {
    rx: Receiver<Option<u32>>,
    stop: Arc<AtomicBool>,
    workers: usize,
    done: usize,
    result: Option<u32>,
}

impl Drop for Search {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

impl Search {
    /// Start a search over the whole register space, `threads` workers wide.
    /// Each worker sweeps its slice and reports a hit or its completion; the
    /// first hit sets the stop flag so the rest wind down.
    pub fn start(frames: Vec<Collision>, threads: usize) -> Self {
        Self::start_range(frames, threads, 0..1u64 << 32)
    }

    /// Start a search over an explicit register range. The whole-space
    /// [`start`](Self::start) is the real use; a small range keeps a test
    /// quick while running the same code.
    pub fn start_range(frames: Vec<Collision>, threads: usize, range: core::ops::Range<u64>) -> Self {
        let threads = threads.max(1);
        let stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = std::sync::mpsc::channel();
        let span = range.end - range.start;
        let chunk = span.div_ceil(threads as u64);
        let frames = Arc::new(frames);
        for w in 0..threads {
            let start = range.start + w as u64 * chunk;
            if start >= range.end {
                // Fewer chunks than workers; report this one done immediately.
                let _ = tx.send(None);
                continue;
            }
            let end = (start + chunk).min(range.end);
            let (tx, stop, frames) = (tx.clone(), stop.clone(), frames.clone());
            std::thread::spawn(move || {
                // Sweep in blocks so the stop flag is seen promptly.
                const STEP: u64 = 1 << 20;
                let mut at = start;
                let mut hit = None;
                while at < end && !stop.load(Ordering::Relaxed) {
                    let to = (at + STEP).min(end);
                    let hits = recover_tea1(&frames, at..to);
                    if let Some(&k) = hits.first() {
                        hit = Some(k);
                        break;
                    }
                    at = to;
                }
                if let Some(k) = hit {
                    stop.store(true, Ordering::Relaxed);
                    let _ = tx.send(Some(k));
                } else {
                    let _ = tx.send(None);
                }
            });
        }
        drop(tx);
        Search { rx, stop, workers: threads, done: 0, result: None }
    }

    /// Non-blocking check on the search.
    pub fn poll(&mut self) -> Progress {
        loop {
            match self.rx.try_recv() {
                Ok(Some(k)) => {
                    self.result = Some(k);
                    self.stop.store(true, Ordering::Relaxed);
                    return Progress::Found(k);
                }
                Ok(None) => {
                    self.done += 1;
                }
                Err(TryRecvError::Empty) => {
                    return match self.result {
                        Some(k) => Progress::Found(k),
                        None => Progress::Running,
                    };
                }
                Err(TryRecvError::Disconnected) => break,
            }
        }
        match self.result {
            Some(k) => Progress::Found(k),
            None if self.done >= self.workers => Progress::Exhausted,
            None => Progress::Running,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tea::Timestamp;

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn recovers_the_reference_key_in_the_background() {
        // The teatime crack vector; the reference short key is 0x111.
        let ts = |frame| Timestamp { tn: 1, frame, multiframe: 30, hyperframe: 110, uplink: false };
        let frames = vec![
            Collision { ts: ts(6), ct: hex("151ef027") },
            Collision { ts: ts(7), ct: hex("4d00159e") },
        ];
        // A window that contains the key, swept across 4 workers.
        let mut s = Search::start_range(frames, 4, 0..0x1_0000);
        let mut spins = 0;
        loop {
            match s.poll() {
                Progress::Found(k) => {
                    assert_eq!(k, 0x111);
                    break;
                }
                Progress::Exhausted => panic!("swept the window without the key"),
                Progress::Running => {
                    spins += 1;
                    assert!(spins < 5_000, "search did not finish");
                    std::thread::sleep(std::time::Duration::from_millis(1));
                }
            }
        }
    }

    #[test]
    fn reports_exhausted_when_no_key_fits() {
        let ts = |frame| Timestamp { tn: 1, frame, multiframe: 30, hyperframe: 110, uplink: false };
        // Two frames whose plaintext does NOT agree under any key in the
        // window: distinct ciphertext with no common register there.
        let frames = vec![
            Collision { ts: ts(6), ct: hex("151ef027") },
            Collision { ts: ts(7), ct: hex("4d00159e") },
        ];
        // A window that excludes 0x111.
        let mut s = Search::start_range(frames, 2, 0x2_0000..0x2_1000);
        loop {
            match s.poll() {
                Progress::Exhausted => break,
                Progress::Found(_) => panic!("no key should fit this window"),
                Progress::Running => std::thread::sleep(std::time::Duration::from_millis(1)),
            }
        }
    }
}
