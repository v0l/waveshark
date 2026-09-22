//! The counts dump1090 publishes as stats.json, which is what graphs1090
//! draws.
//!
//! Two periods, because those are the two the collectd plugin reads: the run
//! so far, and the minute before last. A number nothing here measures is left
//! out of the file rather than sent as a zero, so a graph of it is empty
//! instead of wrong.

use crate::track::{Fix, Seen};
use std::time::{Duration, Instant};

/// How often the running minute becomes the reported one.
pub const MINUTE: Duration = Duration::from_secs(60);

/// The most single-bit corrections a frame can have had, plus the frames that
/// needed none.
pub const ERROR_BINS: usize = 2;

/// Where a frame came from, which decides which counter it lands in.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Source {
    Air,
    Network,
}

#[derive(Clone)]
pub struct Counts {
    pub start: f64,
    pub end: f64,
    pub accepted: [u64; ERROR_BINS],
    pub remote_accepted: u64,
    pub samples_processed: u64,
    pub samples_dropped: u64,
    pub strong_signals: u64,
    pub messages: u64,
    pub messages_by_df: [u64; 32],
    pub cpr_global_ok: u64,
    pub cpr_local_ok: u64,
    pub tracks: u64,
    pub single_message_tracks: u64,
    power: f64,
    powers: u64,
    peak_dbfs: f32,
}

impl Counts {
    fn new(at: f64) -> Self {
        Self {
            start: at,
            end: at,
            accepted: [0; ERROR_BINS],
            remote_accepted: 0,
            samples_processed: 0,
            samples_dropped: 0,
            strong_signals: 0,
            messages: 0,
            messages_by_df: [0; 32],
            cpr_global_ok: 0,
            cpr_local_ok: 0,
            tracks: 0,
            single_message_tracks: 0,
            power: 0.0,
            powers: 0,
            peak_dbfs: f32::NEG_INFINITY,
        }
    }

    /// Mean power of the frames read off the air, in dBFS.
    pub fn signal_dbfs(&self) -> Option<f64> {
        (self.powers > 0).then(|| 10.0 * (self.power / self.powers as f64).log10())
    }

    pub fn peak_dbfs(&self) -> Option<f64> {
        self.peak_dbfs.is_finite().then_some(self.peak_dbfs as f64)
    }

    fn frame(&mut self, df: u8, rssi_dbfs: f32, from: Source, corrected: usize) {
        self.messages += 1;
        self.messages_by_df[(df & 31) as usize] += 1;
        match from {
            Source::Network => self.remote_accepted += 1,
            Source::Air => {
                self.accepted[corrected.min(ERROR_BINS - 1)] += 1;
                if rssi_dbfs.is_finite() {
                    self.power += 10f64.powf(rssi_dbfs as f64 / 10.0);
                    self.powers += 1;
                    self.peak_dbfs = self.peak_dbfs.max(rssi_dbfs);
                    self.strong_signals += (rssi_dbfs > -3.0) as u64;
                }
            }
        }
    }
}

pub struct Stats {
    pub total: Counts,
    pub last_minute: Counts,
    running: Counts,
    rolled: Instant,
}

impl Stats {
    pub fn new(at: f64) -> Self {
        Self {
            total: Counts::new(at),
            last_minute: Counts::new(at),
            running: Counts::new(at),
            rolled: Instant::now(),
        }
    }

    pub fn frame(&mut self, df: u8, rssi_dbfs: f32, from: Source, corrected: usize) {
        self.total.frame(df, rssi_dbfs, from, corrected);
        self.running.frame(df, rssi_dbfs, from, corrected);
    }

    pub fn seen(&mut self, s: &Seen) {
        for c in [&mut self.total, &mut self.running] {
            c.tracks += s.fresh as u64;
            match s.fix {
                Fix::Global => c.cpr_global_ok += 1,
                Fix::Local => c.cpr_local_ok += 1,
                Fix::None => {}
            }
        }
    }

    pub fn expired(&mut self, single_message: u64) {
        for c in [&mut self.total, &mut self.running] {
            c.single_message_tracks += single_message;
        }
    }

    pub fn samples(&mut self, processed: u64, dropped: u64) {
        for c in [&mut self.total, &mut self.running] {
            c.samples_processed += processed;
            c.samples_dropped += dropped;
        }
    }

    /// Close the period at `at`, and report the minute if one has passed.
    pub fn tick(&mut self, at: f64, now: Instant) -> bool {
        self.total.end = at;
        self.running.end = at;
        if now.saturating_duration_since(self.rolled) < MINUTE {
            return false;
        }
        self.rolled = now;
        self.last_minute = std::mem::replace(&mut self.running, Counts::new(at));
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The minute graphs1090 draws is the minute that finished, and the run
    /// so far is everything.
    #[test]
    fn the_running_minute_becomes_the_reported_one_and_the_total_keeps_growing() {
        let base = Instant::now();
        let mut s = Stats::new(0.0);
        for _ in 0..10 {
            s.frame(17, -12.0, Source::Air, 0);
        }
        s.frame(17, -12.0, Source::Air, 1);
        s.frame(11, f32::NEG_INFINITY, Source::Network, 0);

        assert!(!s.tick(30.0, base + Duration::from_secs(30)), "half a minute is not a minute");
        assert_eq!(s.last_minute.messages, 0, "nothing has been reported yet");

        assert!(s.tick(61.0, base + Duration::from_secs(61)), "the minute rolled");
        assert_eq!(s.last_minute.messages, 12);
        assert_eq!(s.last_minute.accepted, [10, 1]);
        assert_eq!(s.last_minute.remote_accepted, 1, "a frame handed back is not one heard");
        assert_eq!(s.last_minute.messages_by_df[17], 11);
        assert_eq!(s.total.messages, 12);

        s.frame(17, -12.0, Source::Air, 0);
        assert!(!s.tick(62.0, base + Duration::from_secs(62)));
        assert_eq!(s.last_minute.messages, 12, "the reported minute is closed");
        assert_eq!(s.total.messages, 13, "the run so far is not");
        assert_eq!(s.total.end, 62.0);
    }

    /// A level is a power, so eleven frames at -20 dBFS and one at 0 dBFS
    /// report -10.34 dBFS and not the -18.3 their decibels average to.
    #[test]
    fn the_signal_reported_is_the_mean_power_and_not_the_mean_of_the_decibels() {
        let mut s = Stats::new(0.0);
        for _ in 0..11 {
            s.frame(17, -20.0, Source::Air, 0);
        }
        s.frame(17, 0.0, Source::Air, 0);
        let signal = s.total.signal_dbfs().expect("a mean level");
        assert!((signal - -10.34).abs() < 0.01, "{signal} dBFS");
        assert_eq!(s.total.peak_dbfs(), Some(0.0));
        assert_eq!(s.total.strong_signals, 1, "frames past -3 dBFS");

        // A frame handed back over the network was never heard here, so it
        // is not in the level and not in what the aerial accepted.
        s.frame(17, f32::NEG_INFINITY, Source::Network, 0);
        assert!((s.total.signal_dbfs().unwrap() - signal).abs() < 1e-9);
        assert_eq!(s.total.accepted, [12, 0]);
        assert!(Stats::new(0.0).total.signal_dbfs().is_none(), "nothing heard, nothing to report");
    }
}
