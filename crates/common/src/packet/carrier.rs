//! L1: the reception itself.

use std::sync::Arc;

use crate::{C32, IqBurst, SourceId};

/// Quietest a level can read, in dBFS
///
/// A channel that has been digitally silent since the receiver started has a
/// power below anything a sample can express, and a level that is not a number
/// cannot be sorted, compared or drawn.
pub const SILENCE_DBFS: f32 = -200.0;

/// The wall clock now, in microseconds since the epoch
///
/// What a front end stamps a reception with. One implementation because three
/// front ends rounding the same clock three ways put packets of one burst a
/// microsecond apart.
pub fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// Microseconds in a day, which is what a clock with no date wraps on.
const DAY_US: i64 = 86_400 * 1_000_000;

/// A transmitter's time of day placed on the receiver's calendar, in
/// microseconds since the epoch.
///
/// A sender keying hours, minutes and seconds and no date leaves the day to
/// whoever hears it, and the receiver knows one thing about that: it is
/// hearing the transmission now. So the date is whichever of yesterday,
/// today and tomorrow puts the two clocks closest together, which is what
/// carries a transmission across midnight in either direction.
pub fn dated(time_of_day_us: u64, now_us: u64) -> u64 {
    let (day, now) = (time_of_day_us as i64 % DAY_US, now_us as i64);
    let today = now.div_euclid(DAY_US) * DAY_US + day;
    [today - DAY_US, today, today + DAY_US]
        .into_iter()
        .min_by_key(|at| (at - now).abs())
        .unwrap_or(today)
        .max(0) as u64
}

/// Mean power of a run of samples, against a full scale sample
pub fn mean_power(samples: &[C32]) -> f32 {
    match samples.is_empty() {
        true => 0.0,
        false => samples.iter().map(|c| c.norm_sqr()).sum::<f32>() / samples.len() as f32,
    }
}

/// A power as a level, floored at [`SILENCE_DBFS`]
pub fn dbfs(power: f32) -> f32 {
    10.0 * power.max(1e-20).log10()
}

/// When and where a transmission was heard, and how strongly
///
/// The only mandatory layer, and the only constructor takes a level, so there
/// is no way to put a packet on the bus without saying how loud it was. A
/// front end holds the samples the burst came from and is the one thing that
/// can measure it; a level taken later from the span is a level of the band.
#[derive(Clone, Debug, PartialEq)]
pub struct Carrier {
    /// Wall clock of the burst, in microseconds since the epoch
    pub at_us: u64,
    /// How long it held the channel, in microseconds
    pub duration_us: u32,
    /// Where it was received: the channel's own centre in a bank, the
    /// advertising channel a frame arrived on, not the tuner's dial
    pub center_hz: u64,
    /// The width it was heard through, which is part of what was heard: the
    /// same burst read through 31 kHz and through 125 kHz is not the same
    /// recording
    pub bandwidth_hz: u32,
    /// Received level in dB, referenced to a full scale sample at the front
    /// end's input. Comparable between packets on one receiver, and not a
    /// field strength
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    /// Which front end heard it: a tuner, a channel of a bank, a remote feed
    pub source: SourceId,
    /// The samples it was read from, when the front end kept them
    ///
    /// Shared rather than copied, since every layer of one burst refers to the
    /// same samples, and carried in memory only: a log keeps the layers, a
    /// list keeps the samples of what it is showing.
    pub iq: Option<Arc<IqBurst>>,
}

impl Carrier {
    /// A reception at what the front end measured
    pub fn heard(
        at_us: u64,
        center_hz: u64,
        bandwidth_hz: u32,
        rssi_dbfs: f32,
        snr_db: f32,
        source: SourceId,
    ) -> Self {
        debug_assert!(rssi_dbfs.is_finite() && snr_db.is_finite(), "a level nothing measured");
        Self { at_us, duration_us: 0, center_hz, bandwidth_hz, rssi_dbfs, snr_db, source, iq: None }
    }

    /// A reception measured off the samples it was read from
    ///
    /// The one implementation of what a packet was heard at, so a front end
    /// that makes bytes and a classifier that measures a burst cannot mean
    /// different things by the same number. Mean power of the samples, and
    /// how far that stands above the floor whoever holds the channel reports.
    /// Not calibrated to the antenna and not quite to the converter either,
    /// since every filter between the two has gain: a comparable number
    /// between packets on one receiver, and not a field strength.
    pub fn measured(
        at_us: u64,
        center_hz: u64,
        bandwidth_hz: u32,
        samples: &[C32],
        floor_dbfs: f32,
        source: SourceId,
    ) -> Self {
        let rssi_dbfs = dbfs(mean_power(samples));
        Self::heard(
            at_us,
            center_hz,
            bandwidth_hz,
            rssi_dbfs,
            (rssi_dbfs - floor_dbfs).max(0.0),
            source,
        )
    }

    pub fn lasting(mut self, duration_us: u32) -> Self {
        self.duration_us = duration_us;
        self
    }

    pub fn with_iq(mut self, iq: Arc<IqBurst>) -> Self {
        self.iq = Some(iq);
        self
    }

    pub fn seconds(&self) -> f64 {
        self.duration_us as f64 / 1e6
    }
}

/// Where and when a stream is, so whatever reads it can say what it heard
///
/// A detector knows timings and a level; it does not know the frequency it is
/// parked on, the clock, or which front end it belongs to, and every one of
/// those used to be stamped on afterwards by whichever node happened to hold
/// the detector. This is those four numbers travelling with the stream, so a
/// reception is put together once, where the samples are.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Heard {
    /// Wall clock of the first sample of the block being read
    pub at_us: u64,
    /// Index of that sample in the stream, so a detector reporting where a
    /// burst began says when it began
    pub sample: u64,
    pub rate: f64,
    pub center_hz: u64,
    /// The width being read, which is the width anything found in it was
    /// heard through
    pub bandwidth_hz: u32,
    pub source: SourceId,
}

impl Heard {
    pub fn new(rate: f64, center_hz: u64, bandwidth_hz: u32, source: SourceId) -> Self {
        Self { at_us: 0, sample: 0, rate: rate.max(1.0), center_hz, bandwidth_hz, source }
    }

    /// The block about to be read, at the clock and stream position it starts
    pub fn at(mut self, at_us: u64, sample: u64) -> Self {
        self.at_us = at_us;
        self.sample = sample;
        self
    }

    /// When a sample of this stream was on the air
    pub fn when(&self, sample: u64) -> u64 {
        let ahead = sample.saturating_sub(self.sample) as f64 / self.rate * 1e6;
        self.at_us + ahead as u64
    }

    /// A reception of what was found at `sample`, measured off its samples
    pub fn carrier(&self, sample: u64, samples: &[C32], floor_dbfs: f32) -> Carrier {
        Carrier::measured(
            self.when(sample),
            self.center_hz,
            self.bandwidth_hz,
            samples,
            floor_dbfs,
            self.source,
        )
        .lasting((samples.len() as f64 / self.rate * 1e6) as u32)
    }

    /// A reception of what was found at `sample`, at a level its reader
    /// measured for itself
    pub fn reported(&self, sample: u64, rssi_dbfs: f32, snr_db: f32, duration_us: u32) -> Carrier {
        Carrier::heard(
            self.when(sample),
            self.center_hz,
            self.bandwidth_hz,
            rssi_dbfs,
            snr_db,
            self.source,
        )
        .lasting(duration_us)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_level_is_measured_the_same_way_wherever_it_is_taken() {
        // Half scale in one quadrature is a quarter of the power, which is
        // the number the auto node's own measurement has always reported.
        let half = vec![C32::new(0.5, 0.0); 1024];
        assert!((dbfs(mean_power(&half)) + 6.02).abs() < 0.05, "{}", dbfs(mean_power(&half)));
        let c = Carrier::measured(1, 433_920_000, 31_250, &half, -60.0, SourceId(0));
        assert!((c.rssi_dbfs + 6.02).abs() < 0.05);
        assert!((c.snr_db - 53.98).abs() < 0.05, "{}", c.snr_db);
    }

    #[test]
    fn a_stream_says_when_and_where_what_it_heard_was() {
        // The detector counts samples; nothing else in the chain should have
        // to turn those into a time, and two places doing it is two answers.
        let h = Heard::new(1_000_000.0, 868_300_000, 250_000, SourceId(2)).at(1_000_000, 4_096);
        assert_eq!(h.when(4_096), 1_000_000, "the block's own first sample");
        assert_eq!(h.when(5_096), 1_001_000, "a thousand samples is a millisecond at 1 MS/s");
        let c = h.carrier(5_096, &[C32::new(0.5, 0.0); 1000], -60.0);
        assert_eq!(c.at_us, 1_001_000);
        assert_eq!((c.center_hz, c.bandwidth_hz, c.source), (868_300_000, 250_000, SourceId(2)));
        assert_eq!(c.duration_us, 1_000);
        assert!((c.rssi_dbfs + 6.02).abs() < 0.05);
    }

    /// A clock with no date lands on the day that puts it nearest the
    /// receiver's, so a transmission heard either side of midnight is dated
    /// the day it was actually sent.
    #[test]
    fn a_time_of_day_takes_the_nearest_date() {
        // 2023-11-14 22:13:20 UTC, which is 80_000 seconds into the day.
        let now = 1_700_000_000_000_000u64;
        assert_eq!(dated(80_000_000_000, now), now);
        assert_eq!(dated(79_999_000_000, now), now - 1_000_000);
        // Ten seconds past midnight, heard two hours before it: tomorrow.
        assert_eq!(dated(10_000_000, now), now + 6_410_000_000);
        // And the mirror: a receiver two hours past midnight hearing a clock
        // at ten to midnight dates it yesterday.
        let after = 1_700_000_000_000_000u64 + 14_000_000_000;
        assert_eq!(dated(86_390_000_000, after), after - 7_610_000_000);
        assert_eq!(dated(0, 0), 0);
    }

    #[test]
    fn silence_reads_as_silence_rather_than_as_nothing() {
        let c = Carrier::measured(1, 433_920_000, 31_250, &[], SILENCE_DBFS, SourceId(0));
        assert_eq!(c.rssi_dbfs, SILENCE_DBFS);
        assert_eq!(c.snr_db, 0.0, "a silent channel is not infinitely above its floor");
    }
}
