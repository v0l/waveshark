//! Where a stream with no sequence numbers lost samples.
//!
//! A protocol that numbers its samples says what went missing; `rtl_tcp` does
//! not, and the server's own overruns are printed to its stdout and never
//! sent. The only evidence left at this end is when a block arrived against
//! how many samples came with it.
//!
//! A network stall is not a loss: TCP keeps what it could not deliver, so a
//! held connection arrives late and then in a burst, and every sample is
//! there. A loss is a stall that never catches up. So what is measured is the
//! lowest lateness in a recent window, which the drain of a stall pulls back
//! down and a real loss leaves raised for good.

use common::Sps;
use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// How far back the lowest lateness is looked for.
///
/// Long enough that the burst after a stall has drained and pulled the floor
/// back down, short enough that a real loss is not sat on. A stall is drained
/// at whatever the link will carry, so this is the ratio of the link's rate
/// to the stream's: a second holds a 400 ms stall on a link with any headroom
/// at all.
const WINDOW: Duration = Duration::from_secs(1);

/// How much the floor must rise before samples are called lost.
///
/// Measured over loopback against a paced server, with every core of this
/// machine busy: the lowest lateness in a window moves by 0.07 ms at
/// 2.4 MS/s and by 2 ms at 240 kS/s over six seconds. Fifty milliseconds is
/// twenty-five times the worst of that, and a network link's own floor moves
/// further than a loopback one does.
const MARGIN: f64 = 0.050;

/// Lost samples, counted from when blocks arrive.
pub struct Gaps {
    rate: f64,
    start: Instant,
    /// Samples handed on, plus every gap declared, which is what the stream's
    /// timebase counts.
    counted: u64,
    /// The lowest lateness this stream has settled at, which is the pipeline's
    /// own latency until something is lost.
    floor: Option<f64>,
    seen: VecDeque<(Instant, f64)>,
}

impl Gaps {
    /// Start counting now, for a stream running at this rate.
    pub fn new(rate: Sps) -> Self {
        Self {
            rate: rate.as_f64().max(1.0),
            start: Instant::now(),
            counted: 0,
            floor: None,
            seen: VecDeque::new(),
        }
    }

    /// Take a block of `samples` that has just arrived, and say how many
    /// samples went missing before it.
    ///
    /// The answer is nought for a stream that is keeping up and for one that
    /// stalled and caught up. What it returns has already been added to
    /// [`Gaps::counted`], so a caller numbering its blocks by that stays on
    /// the true timebase.
    pub fn arrived(&mut self, samples: u64) -> u64 {
        self.at(Instant::now(), samples)
    }

    fn at(&mut self, now: Instant, samples: u64) -> u64 {
        self.counted += samples;
        let late = now.duration_since(self.start).as_secs_f64() - self.counted as f64 / self.rate;
        self.seen.push_back((now, late));
        while self.seen.front().is_some_and(|(t, _)| now.duration_since(*t) > WINDOW) {
            self.seen.pop_front();
        }
        let lowest = self.seen.iter().map(|(_, l)| *l).fold(f64::INFINITY, f64::min);
        let floor = match self.floor {
            Some(f) => f,
            None => {
                self.floor = Some(lowest);
                return 0;
            }
        };
        if lowest <= floor + MARGIN {
            self.floor = Some(floor.min(lowest));
            return 0;
        }
        // The stream is permanently behind where its own rate says it should
        // be, and the only thing that does that is samples that were never
        // sent. Count them, and start the window again: every lateness in it
        // was measured against a count that has just changed.
        let lost = ((lowest - floor) * self.rate).round() as u64;
        self.counted += lost;
        self.seen.clear();
        lost
    }

    /// Samples this stream has carried, gaps included. The sequence number of
    /// the next block is this less the block's own length.
    pub fn counted(&self) -> u64 {
        self.counted
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Blocks arriving exactly on time, then one late burst that catches up.
    /// Nothing is lost either way, and saying otherwise would put a hole in a
    /// recording that has every sample in it.
    #[test]
    fn a_stall_that_catches_up_has_lost_nothing() {
        let rate = Sps(240_000);
        let mut g = Gaps::new(rate);
        let block = 4_800u64;
        let period = 0.020;
        let t0 = g.start;
        let mut lost = 0;
        for i in 1..=50u64 {
            lost += g.at(t0 + Duration::from_secs_f64(i as f64 * period), block);
        }
        assert_eq!(lost, 0, "a stream keeping up loses nothing");

        // Held for 400 ms, then the backlog arrives in 20 ms.
        let mut at = 50.0 * period + 0.400;
        for _ in 0..20u64 {
            at += 0.001;
            lost += g.at(t0 + Duration::from_secs_f64(at), block);
        }
        for i in 1..=100u64 {
            let when = at + i as f64 * period;
            lost += g.at(t0 + Duration::from_secs_f64(when), block);
        }
        assert_eq!(lost, 0, "the burst pulled the floor back down");
        assert_eq!(g.counted(), 170 * block, "and the count is every sample sent");
    }

    /// A server that stopped sending for 400 ms and then carried on from the
    /// present is 400 ms of samples short, and nothing later makes that up.
    #[test]
    fn a_stall_that_never_catches_up_is_counted_as_lost() {
        let rate = Sps(240_000);
        let mut g = Gaps::new(rate);
        let block = 4_800u64;
        let period = 0.020;
        let t0 = g.start;
        let mut at = 0.0;
        for _ in 0..50u64 {
            at += period;
            assert_eq!(g.at(t0 + Duration::from_secs_f64(at), block), 0);
        }
        at += 0.400;
        let mut lost = 0;
        // The window has to roll past the stall before the lowest lateness in
        // it is the new one, which is what tells a loss from a stall.
        for _ in 0..100u64 {
            at += period;
            lost += g.at(t0 + Duration::from_secs_f64(at), block);
        }
        let wanted = (0.400 * 240_000.0) as u64;
        assert_eq!(lost, wanted, "400 ms at 240 kS/s");
        assert_eq!(g.counted(), 150 * block + wanted, "the timebase carries the hole");

        // Once counted it is not counted again, however long the stream runs.
        let mut after = 0;
        for _ in 0..200u64 {
            at += period;
            after += g.at(t0 + Duration::from_secs_f64(at), block);
        }
        assert_eq!(after, 0, "a gap is declared once");
    }

    /// A loss smaller than the margin is left alone: at 240 kS/s that is
    /// under 12000 samples, and a figure below the jitter of the link cannot
    /// be told from one.
    #[test]
    fn a_loss_under_the_margin_is_not_declared() {
        let rate = Sps(240_000);
        let mut g = Gaps::new(rate);
        let block = 4_800u64;
        let period = 0.020;
        let t0 = g.start;
        let mut at = 0.0;
        let mut lost = 0;
        for _ in 0..50u64 {
            at += period;
            lost += g.at(t0 + Duration::from_secs_f64(at), block);
        }
        at += 0.040;
        for _ in 0..100u64 {
            at += period;
            lost += g.at(t0 + Duration::from_secs_f64(at), block);
        }
        assert_eq!(lost, 0);
        assert_eq!(g.counted(), 150 * block);
    }
}
