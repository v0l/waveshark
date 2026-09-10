//! The locks the node holds, and how well each of them has done.
//!
//! A [`Lock`] is a front end's statement about a transmitter it has learned;
//! this is what the node does with it. Every open source is offered to every
//! lock before the burst classifier runs, and the best claim over
//! [`pipeline::lock::CLAIM_THRESHOLD`] wins: that source is read by the
//! claiming protocol alone, with the settings the lock carries, and nothing
//! else is built on it.
//!
//! Which makes a wrong lock dangerous, since it would swallow a band. So the
//! node keeps the score: of the sources a lock claimed, how many the claiming
//! front end read something from. A lock that keeps being wrong loses
//! confidence, stops winning the claims it fits worst, and is dropped.
//!
//! Two older things in this node are locks in all but name and are still
//! written separately. A remembered channel ([`super::memory`], from
//! `Stickiness::Latch`) is a lock over a raster of one frequency with no
//! schedule, and a `Request::Claim` (the camera) is a lock over a range that
//! answers `Mine` to everything in it. What keeps them apart is not the
//! prediction but what follows from it: both shut the detector out of their
//! frequencies, and a lock does not, because a hopper's channels are a
//! hundredth of its band each and closing eighty of them would close the band
//! to everything else on it. Folding them together means giving a lock that
//! choice as well, and unpicking [`super::AutoNode::apply_locked`], the
//! spectrum's locked-channel markers and the side channels a decoder asks
//! for, all of which are keyed on a channel having a frequency and a width
//! rather than a raster.

use common::SourceBlock;
use pipeline::lock::{Lock, Verdict};
use pipeline::registry::Settings;

use super::AutoNode;

/// A held lock's name, so a slot can say which lock claimed it across
/// rebuilds and drops without holding an index into a moving list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct LockId(u64);

/// Claims a lock must have made before its accuracy is judged. Under this
/// it is taken on the front end's word: a lock published on a link that has
/// just decoded is right about the next burst more often than not, and
/// condemning it on its first quiet source would drop it before it had been
/// tested.
const JUDGE_AFTER: u32 = 8;

/// The share of a lock's claims that must have produced a decode for it to
/// be kept at all.
///
/// Low, because a claim that reads nothing is ordinary rather than wrong: a
/// hop can open on a fade, on the tail of a packet, or on a source the
/// detector split in two. Measured on the 61.44 MS/s capture of one
/// ExpressLRS handset, twelve of the twenty-nine visits decode and the rest
/// are real visits of the same link. A lock whose bursts decode less than a
/// fifth of the time is not one its front end has learned.
const ACCURACY_FLOOR: f64 = 0.2;

/// The share above which a lock is believed as published.
///
/// Between this and [`ACCURACY_FLOOR`] a lock's confidence falls off, so it
/// gives up the sources it fits worst before it gives up all of them. What
/// it must not do is fall off with the decode rate itself: most of a
/// hopper's visits pass without a decode and the lock is right about every
/// one of them. Whether a burst decoded is evidence about the lock only
/// when hardly any of them do.
const TRUSTED_ACCURACY: f64 = 2.0 * ACCURACY_FLOOR;

/// One claim of each kind assumed before the first, so a lock starts at full
/// accuracy and falls as unconfirmed claims accumulate rather than swinging
/// on the first one.
const PRIOR: u32 = 1;

/// One lock and its record.
struct Held {
    id: LockId,
    /// The protocol that published it, taken from the front end that asked
    /// rather than from anything the lock says about itself: a node cannot
    /// hand a band to a protocol it is not.
    protocol: &'static str,
    lock: Lock,
    /// The confidence the front end published. The running confidence in
    /// `lock` is this scaled by how many claims have decoded.
    declared: f32,
    claimed: u32,
    decoded: u32,
}

impl Held {
    /// The share of this lock's claims that produced a decode, smoothed by
    /// [`PRIOR`].
    fn accuracy(&self) -> f64 {
        f64::from(self.decoded + PRIOR) / f64::from(self.claimed + PRIOR)
    }

    /// The confidence the lock runs at: what it published while it is being
    /// read at [`TRUSTED_ACCURACY`] or better, falling off below that.
    fn standing(&self) -> f32 {
        self.declared * (self.accuracy() / TRUSTED_ACCURACY).min(1.0) as f32
    }

    fn failing(&self) -> bool {
        self.claimed >= JUDGE_AFTER && self.accuracy() < ACCURACY_FLOOR
    }
}

/// What a lock's claim on a source amounts to: which protocol reads it, on
/// what channel, and what that front end is to be told.
pub(super) struct Claimed {
    pub(super) id: LockId,
    pub(super) protocol: &'static str,
    pub(super) width_hz: f64,
    pub(super) settings: Settings,
}

#[derive(Default)]
pub(super) struct Locks {
    held: Vec<Held>,
    made: u64,
}

impl Locks {
    /// Hold a lock a front end has published, or refresh one it has already
    /// published, keeping the record that lock has built up.
    ///
    /// One lock per protocol and transmitter: every front end the node
    /// places on a claimed source knows the same link and says so again on
    /// its first decode, and each of those is the same statement rather than
    /// a new one.
    pub(super) fn hold(&mut self, protocol: &'static str, lock: Lock) -> Option<LockId> {
        let same = |h: &&mut Held| h.protocol == protocol && h.lock.transmitter == lock.transmitter;
        if let Some(h) = self.held.iter_mut().find(same) {
            h.declared = lock.confidence;
            h.lock = lock;
            h.lock.confidence = h.standing();
            return None;
        }
        let id = LockId(self.made);
        self.made += 1;
        self.held.push(Held {
            id,
            protocol,
            declared: lock.confidence,
            lock,
            claimed: 0,
            decoded: 0,
        });
        Some(id)
    }

    /// The lock that claims this source, if any: the best claim over the
    /// threshold.
    ///
    /// Every lock is asked rather than the first over the threshold taken,
    /// because two locks that both fit are a question about which
    /// transmitter this is and the better claim is the answer. There are
    /// only ever a handful of locks, and each answer is a divide and a
    /// compare; see [`Lock::claim`] for the budget.
    pub(super) fn claim(&self, b: &SourceBlock) -> Option<Claimed> {
        let mut best: Option<(f32, &Held)> = None;
        for h in &self.held {
            let c = h.lock.claim(b);
            if c.verdict != Verdict::Mine {
                continue;
            }
            if best.is_none_or(|(score, _)| c.score > score) {
                best = Some((c.score, h));
            }
        }
        let (_, h) = best?;
        Some(Claimed {
            id: h.id,
            protocol: h.protocol,
            width_hz: h.lock.width_hz,
            settings: h.lock.settings.clone(),
        })
    }

    /// Say what became of a source a lock claimed: whether the front end it
    /// was handed to read anything from it. Returns the lock's name where
    /// this was the claim that dropped it.
    pub(super) fn scored(&mut self, id: LockId, decoded: bool) -> Option<(&'static str, String)> {
        let h = self.held.iter_mut().find(|h| h.id == id)?;
        h.claimed += 1;
        h.decoded += u32::from(decoded);
        h.lock.confidence = h.standing();
        if !h.failing() {
            return None;
        }
        let gone = (h.protocol, h.lock.transmitter.clone());
        self.held.retain(|h| h.id != id);
        Some(gone)
    }

    /// Every lock held, as (protocol, transmitter, confidence).
    pub(super) fn held(&self) -> Vec<(&'static str, &str, f32)> {
        self.held
            .iter()
            .map(|h| (h.protocol, h.lock.transmitter.as_str(), h.lock.confidence))
            .collect()
    }
}

impl AutoNode {
    /// The transmitters a front end inside has learned this session, as
    /// (front end, what it calls the transmitter, how sure it still is).
    /// The frequency counterpart of [`AutoNode::remembered`]: that is where
    /// something was heard, this is what will be heard and where.
    pub fn locked_transmitters(&self) -> Vec<(&'static str, &str, f32)> {
        self.locks.held()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{SourceId, SourceState};
    use pipeline::lock::Raster;

    fn lock(transmitter: &str) -> Lock {
        Lock {
            transmitter: transmitter.into(),
            raster: Raster { start_hz: 2_400_400_000.0, step_hz: 1_000_000.0, count: 80 },
            tolerance_hz: 250_000.0,
            width_hz: 812_500.0,
            width_share: (0.7, 2.0),
            confidence: 1.0,
            settings: Settings::new(),
        }
    }

    fn source(center_hz: u64, bandwidth_hz: f64) -> SourceBlock {
        SourceBlock {
            id: SourceId(1),
            state: SourceState::Opened,
            center_hz,
            bandwidth_hz,
            signal_hz: bandwidth_hz / 1.5,
            rate: 4_000_000.0,
            start_sample: 0,
            snr_db: 20.0,
            samples: Vec::new(),
        }
    }

    /// A hop of a locked link is claimed for the protocol that published the
    /// lock, with what that front end learned; a Wi-Fi carrier on the same
    /// channel is not.
    #[test]
    fn a_lock_claims_the_sources_it_predicted_and_no_others() {
        let mut locks = Locks::default();
        let mut l = lock("6f37");
        l.settings.insert("link".into(), pipeline::ParamValue::Text("6f37".into()));
        assert!(locks.hold("elrs", l).is_some());
        let c = locks.claim(&source(2_430_591_000, 958_125.0)).expect("a claim");
        assert_eq!(c.protocol, "elrs");
        assert_eq!(c.width_hz, 812_500.0);
        assert_eq!(c.settings.get("link").and_then(|v| v.as_str()), Some("6f37"));
        assert!(locks.claim(&source(2_436_400_000, 4_140_000.0)).is_none(), "a Wi-Fi carrier");
        assert!(locks.claim(&source(2_402_008_000, 1_065_000.0)).is_none(), "BLE advertising");
    }

    /// The same transmitter published again is the same lock: every front
    /// end placed on a claimed hop says it too, and each of those is not a
    /// new statement.
    #[test]
    fn one_lock_per_protocol_and_transmitter() {
        let mut locks = Locks::default();
        let id = locks.hold("elrs", lock("6f37")).expect("a new lock");
        assert!(locks.hold("elrs", lock("6f37")).is_none(), "the same link again");
        assert!(locks.hold("elrs", lock("aa11")).is_some(), "another link");
        assert_eq!(locks.held().len(), 2);
        // And the record follows the lock across a refresh, so a front end
        // that keeps republishing cannot wash out its own score.
        for _ in 0..JUDGE_AFTER {
            locks.scored(id, false);
        }
        assert_eq!(locks.held().len(), 1, "{:?}", locks.held());
    }

    /// A lock whose claims stop decoding loses confidence, gives up the
    /// sources it fits worst, and is dropped. This is what keeps a wrong
    /// lock from swallowing a band.
    #[test]
    fn a_lock_whose_claims_do_not_decode_is_dropped() {
        let mut locks = Locks::default();
        let id = locks.hold("elrs", lock("6f37")).expect("a new lock");
        let edge = source(2_430_590_000, 812_500.0);
        let middle = source(2_430_400_000, 812_500.0);
        assert!(locks.claim(&edge).is_some(), "claimed at full confidence");

        // Four claims that read nothing: the confidence is down and the
        // sources at the edge of the tolerance are no longer claimed,
        // while one right on a channel still is.
        for _ in 0..4 {
            assert!(locks.scored(id, false).is_none());
        }
        assert!(locks.claim(&edge).is_none(), "still claiming what it fits worst");
        assert!(locks.claim(&middle).is_some(), "gave up what it fits best too early");

        // Two more, and it claims nothing at all.
        for _ in 0..2 {
            assert!(locks.scored(id, false).is_none());
        }
        let confidence = locks.held()[0].2;
        assert!((0.0..0.5).contains(&confidence), "{confidence}");
        assert!(locks.claim(&middle).is_none(), "{:?}", locks.held());

        // And past the point where it has been tested, it is dropped by
        // name, so what is holding a band can be said.
        let mut dropped = None;
        for _ in 0..JUDGE_AFTER {
            dropped = dropped.or(locks.scored(id, false));
        }
        assert_eq!(dropped, Some(("elrs", "6f37".to_string())));
        assert!(locks.held().is_empty());
        assert!(locks.scored(id, false).is_none(), "a lock that is gone is not scored again");
    }

    /// A lock that mostly reads is kept, even though most of a hopper's
    /// visits pass without a decode: twelve of twenty-nine on the capture
    /// this floor was measured against.
    #[test]
    fn a_lock_that_reads_a_fifth_of_what_it_claims_is_kept() {
        let mut locks = Locks::default();
        let id = locks.hold("elrs", lock("6f37")).expect("a new lock");
        for k in 0..29 {
            assert!(locks.scored(id, k % 29 < 12).is_none(), "dropped after {k}");
        }
        assert_eq!(locks.held()[0].2, 1.0, "a lock read this often is believed as published");
        assert!(locks.claim(&source(2_430_400_000, 812_500.0)).is_some());
    }

    /// The better claim wins where two locks both fit, so which transmitter
    /// a source belongs to is decided by fit rather than by which front end
    /// spoke first.
    #[test]
    fn the_better_claim_takes_the_source() {
        let mut locks = Locks::default();
        locks.hold("elrs", lock("6f37"));
        let mut other = lock("aa11");
        other.raster = Raster { start_hz: 2_430_600_000.0, step_hz: 1_000_000.0, count: 2 };
        locks.hold("lora", other);
        // Right on the second lock's channel and 200 kHz off the first's.
        let c = locks.claim(&source(2_430_600_000, 812_500.0)).expect("a claim");
        assert_eq!(c.protocol, "lora");
        // And the other way about.
        let c = locks.claim(&source(2_430_400_000, 812_500.0)).expect("a claim");
        assert_eq!(c.protocol, "elrs");
    }
}
