//! A front end's statement that it has learned a transmitter well enough to
//! recognise its bursts cheaply.
//!
//! A detector finds what is transmitting by looking at it. That is the right
//! answer for something nothing has heard before and the wrong one for a
//! transmitter a decoder has already read and can predict: an ExpressLRS
//! handset hops across eighty channels a megahertz apart, and a detector that
//! knows nothing opens each visit as a new source, classifies it, and builds
//! and tears down a chirp decoder for it. Measured on a busy 2.4 GHz band,
//! that is fifty-three sources in two seconds for one transmitter.
//!
//! A lock is what the decoder knows, written down so the node that placed it
//! can act on it before anything expensive runs: the channels the transmitter
//! uses, how wide its bursts are, and how sure the front end is. When a
//! source opens, every lock is asked ([`Lock::claim`]); a claim over
//! [`CLAIM_THRESHOLD`] wins, and the source is fed to that protocol's front
//! end alone.
//!
//! What is deliberately not here is a schedule. A hopper whose sequence is
//! known could say when it will next be on the air and a lock could check the
//! source's start against that, but nothing can produce one yet: ExpressLRS
//! learns two bytes of the binding UID off the air and `hop_sequence` needs
//! four, so the sequence is only known when an operator has typed the binding
//! phrase. Adding one would need the burst's own sample index carried out of
//! the shared chirp reader and the stream's origin in the span given to the
//! decoder, neither of which exists today.

use crate::registry::Settings;
use common::SourceBlock;

/// The channels a locked transmitter uses: the first one, the step between
/// them, and how many there are.
///
/// A hop set, a channel plan, or one frequency with a count of one. Written
/// as three numbers rather than a list because that is what a plan is, and
/// because a claim then costs a divide and a round however many channels it
/// holds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Raster {
    pub start_hz: f64,
    pub step_hz: f64,
    pub count: usize,
}

impl Raster {
    /// One frequency, for a transmitter that does not move.
    pub fn one(hz: f64) -> Self {
        Self { start_hz: hz, step_hz: 0.0, count: 1 }
    }

    /// The channel nearest `hz` and how far off it is, in hertz, signed.
    /// `None` for a raster with no channels in it.
    pub fn nearest(&self, hz: f64) -> Option<(f64, f64)> {
        if self.count == 0 {
            return None;
        }
        let k = match self.step_hz > 0.0 {
            true => {
                ((hz - self.start_hz) / self.step_hz).round().clamp(0.0, (self.count - 1) as f64)
            }
            false => 0.0,
        };
        let on = self.start_hz + k * self.step_hz;
        Some((on, hz - on))
    }
}

/// What a lock makes of a source.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// This is the locked transmitter. Read it with that front end and
    /// build nothing else on it.
    Mine,
    /// It is not: the wrong frequency, or the wrong width.
    NotMine,
    /// It could be, and this lock is not sure enough to say so. Treated as
    /// `NotMine` by whatever placed it, and kept apart from it because the
    /// two mean different things about the lock rather than about the
    /// source.
    Unsure,
}

/// A lock's answer about one source: how sure, and what to do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Claim {
    /// In [0, 1]. Zero for a source that fails the lock's own tests.
    pub score: f32,
    pub verdict: Verdict,
}

impl Claim {
    /// Nothing of this lock's: the wrong frequency or the wrong width.
    pub fn not_mine() -> Self {
        Self { score: 0.0, verdict: Verdict::NotMine }
    }
}

/// The score a claim must reach for the source to be handed over.
///
/// Half, so a lock at full confidence claims anything that fits its raster
/// and its width, and one whose claims have stopped decoding gives up the
/// marginal ones first and then all of them. See [`Lock::claim`] for how a
/// score is arrived at.
pub const CLAIM_THRESHOLD: f32 = 0.5;

/// How much of its confidence a lock loses across the reach of its own
/// tolerance: a source right on a channel scores the lock's confidence, one
/// at the edge of what it will accept scores half of it.
const EDGE_LOSS: f64 = 0.5;

/// What a front end has learned about a transmitter, as the thing that
/// placed the front end can use it.
#[derive(Clone, Debug, PartialEq)]
pub struct Lock {
    /// What the front end calls this transmitter: a link id, an address, a
    /// network. Two locks of one protocol are told apart by it, and it is
    /// what a person reads in the log.
    pub transmitter: String,
    /// The channels it uses.
    pub raster: Raster,
    /// How far off a channel of the raster a source may be measured and
    /// still be that channel. A source is a power centroid of the bins that
    /// stood over the floor, so it lands near a channel rather than on it:
    /// measured on a 61.44 MS/s capture of one ExpressLRS handset, the
    /// twenty-nine visits sat 9 to 195 kHz above their channel. Never more
    /// than half the step, or two channels could claim the same source.
    pub tolerance_hz: f64,
    /// The channel the transmitter keys, in hertz, which is what the front
    /// end placed on a claimed source is built for.
    pub width_hz: f64,
    /// The share of `width_hz` a source may measure and still be one of
    /// this transmitter's bursts, as (least, most). A clean channel measures
    /// a little over its width and a strong one up to twice.
    pub width_share: (f64, f64),
    /// How sure the front end is, in [0, 1]. Whatever holds the lock scales
    /// this by how many of its claims have decoded, so a lock that keeps
    /// being wrong stops winning claims and is dropped.
    pub confidence: f32,
    /// What the front end learned, so the one placed on a claimed burst
    /// starts knowing it rather than learning it again from the first
    /// packets of every visit.
    pub settings: Settings,
}

impl Lock {
    /// Whether this source is the locked transmitter's.
    ///
    /// The budget is the point of the whole arrangement. This runs on every
    /// source the detector opens, ahead of the burst classifier it is there
    /// to replace, and the classifier costs about 1.3 ms of a core per
    /// burst. Fifty microseconds is the ceiling, and what is written here is
    /// well under it: arithmetic on the numbers the detector has already
    /// measured. `b.samples` is the block the source was cut into, for a
    /// lock that needs to look at the signal itself; none does yet, and one
    /// that does still has to answer inside the same budget.
    ///
    /// The score is the lock's confidence, less up to [`EDGE_LOSS`] of it
    /// for a source at the far edge of the tolerance. A source that fails
    /// either test scores nothing: the raster and the width are tests, and
    /// only the fit inside them is a matter of degree.
    pub fn claim(&self, b: &SourceBlock) -> Claim {
        let share = b.bandwidth_hz / self.width_hz.max(1.0);
        if share < self.width_share.0 || share > self.width_share.1 {
            return Claim::not_mine();
        }
        let Some((_, off)) = self.raster.nearest(b.center_hz as f64) else {
            return Claim::not_mine();
        };
        let off = off.abs();
        if off > self.tolerance_hz {
            return Claim::not_mine();
        }
        let fit = 1.0 - EDGE_LOSS * off / self.tolerance_hz.max(1.0);
        let score = (f64::from(self.confidence) * fit).clamp(0.0, 1.0) as f32;
        let verdict = match score >= CLAIM_THRESHOLD {
            true => Verdict::Mine,
            false => Verdict::Unsure,
        };
        Claim { score, verdict }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{SourceId, SourceState};

    /// The ExpressLRS 2.4 GHz hop set: eighty channels a megahertz apart
    /// from 2400.4 MHz, keyed 812.5 kHz wide.
    fn elrs_lock() -> Lock {
        Lock {
            transmitter: "6f37".into(),
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

    #[test]
    fn a_channel_of_the_raster_is_the_nearest_one_and_the_offset_from_it() {
        let r = Raster { start_hz: 2_400_400_000.0, step_hz: 1_000_000.0, count: 80 };
        assert_eq!(r.nearest(2_400_400_000.0), Some((2_400_400_000.0, 0.0)));
        assert_eq!(r.nearest(2_430_591_000.0), Some((2_430_400_000.0, 191_000.0)));
        assert_eq!(r.nearest(2_430_200_000.0), Some((2_430_400_000.0, -200_000.0)));
        // Past the end of the raster the offset says so rather than
        // wrapping onto a channel that does not exist.
        let (on, off) = r.nearest(2_500_000_000.0).expect("a channel");
        assert_eq!(on, 2_479_400_000.0);
        assert!(off > 20e6, "{off}");
        // One channel and no step, which is what a transmitter that does not
        // move looks like written this way.
        let one = Raster { start_hz: 869_525_000.0, step_hz: 0.0, count: 1 };
        assert_eq!(one.nearest(869_500_000.0), Some((869_525_000.0, -25_000.0)));
        assert_eq!(Raster { start_hz: 0.0, step_hz: 1.0, count: 0 }.nearest(0.0), None);
    }

    /// A hop of the locked link, measured as the detector measures one: on a
    /// channel of the raster to within a fifth of the spacing, and about a
    /// channel wide.
    #[test]
    fn a_hop_on_the_raster_at_the_right_width_is_claimed() {
        let lock = elrs_lock();
        let c = lock.claim(&source(2_430_591_000, 958_125.0));
        assert_eq!(c.verdict, Verdict::Mine, "{c:?}");
        assert!(c.score > 0.5 && c.score <= 1.0, "{c:?}");
        // Right on the channel scores the lock's whole confidence.
        let on = lock.claim(&source(2_430_400_000, 812_500.0));
        assert_eq!(on.score, 1.0, "{on:?}");
        assert!(on.score > c.score, "nearer the channel is a better claim");
    }

    #[test]
    fn a_source_off_the_raster_or_the_wrong_width_is_not_claimed() {
        let lock = elrs_lock();
        // Bluetooth advertising channel 37, which is 392 kHz off the nearest
        // ExpressLRS channel and about the same width as one.
        assert_eq!(lock.claim(&source(2_402_008_000, 1_065_000.0)), Claim::not_mine());
        // A Wi-Fi carrier on an ExpressLRS channel: right frequency, five
        // times the width.
        assert_eq!(lock.claim(&source(2_436_400_000, 4_140_000.0)), Claim::not_mine());
        // And a sensor channel far under it.
        assert_eq!(lock.claim(&source(2_436_400_000, 40_000.0)), Claim::not_mine());
    }

    /// A lock whose confidence has been cut by claims that did not decode
    /// gives up the sources that fit worst before it gives up the rest.
    #[test]
    fn a_lock_losing_confidence_claims_only_what_fits_best() {
        let mut lock = elrs_lock();
        lock.confidence = 0.6;
        let near = lock.claim(&source(2_430_420_000, 812_500.0));
        assert_eq!(near.verdict, Verdict::Mine, "{near:?}");
        let far = lock.claim(&source(2_430_600_000, 812_500.0));
        assert_eq!(far.verdict, Verdict::Unsure, "{far:?}");
        // Unsure is not NotMine: the source fits, the lock does not vouch
        // for it.
        assert!(far.score > 0.0, "{far:?}");
        lock.confidence = 0.3;
        assert_eq!(lock.claim(&source(2_430_400_000, 812_500.0)).verdict, Verdict::Unsure);
    }
}
