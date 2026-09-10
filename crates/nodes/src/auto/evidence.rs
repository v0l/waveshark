//! What the detector alone knows about a source nothing read.
//!
//! The burst router is placed only where its width fits
//! ([`crate::protocol::router_max_width_hz`]), so a source wider than any
//! front end can read has nothing measuring the bursts inside it. Left at
//! that, an unknown five megahertz signal would leave no trace at all: the
//! detector found it, opened it, cut it out, and the log would say nothing.
//!
//! So the detector's own measurement becomes the row. It knows less than the
//! classifier did, and says so: a centre, a width, how long the transmission
//! lasted, the level it stood at, and no modulation. That is the same shape
//! of evidence as the classifier's row for a burst no front end read, and it
//! carries the samples it was measured from for the same reason.

use common::{Modulation, Package, Packet, C32};

use super::member::Ring;

/// How often a source that has not closed yet leaves a row, in seconds.
///
/// A carrier is on all day and a row a block would be a list of nothing
/// else. The same interval the burst router reports a continuing
/// transmission at, for the same reason.
const REPORT_S: f64 = 5.0;

/// The detector's measurement of one open source, accumulating while it runs.
pub(super) struct Evidence {
    center_hz: u64,
    /// What the detector measured the signal at, not the width of the stream
    /// it was cut out into: the row is about the transmission.
    width_hz: f32,
    snr_db: f32,
    rate: f64,
    start_sample: u64,
    /// Samples of the source read so far, which is how long it has been on
    /// the air.
    read: u64,
    /// Loudest block since the last row, so a row says what the transmission
    /// stood at rather than what the channel was doing between bursts.
    peak_pow: f32,
    /// Seconds of the source at the last row, for a transmission that runs
    /// on past [`REPORT_S`].
    reported_s: Option<f64>,
}

impl Evidence {
    pub(super) fn new(center_hz: u64, width_hz: f64, snr_db: f32, rate: f64) -> Self {
        Self {
            center_hz,
            width_hz: width_hz as f32,
            snr_db,
            rate: rate.max(1.0),
            start_sample: 0,
            read: 0,
            peak_pow: 0.0,
            reported_s: None,
        }
    }

    /// Where in the span the source began, so the row can be lined up with a
    /// waterfall the way a routed burst's is.
    pub(super) fn from_sample(mut self, span_sample: u64) -> Self {
        self.start_sample = span_sample;
        self
    }

    pub(super) fn push(&mut self, iq: &[C32]) {
        if iq.is_empty() {
            return;
        }
        let pow = iq.iter().map(|c| c.norm_sqr()).sum::<f32>() / iq.len() as f32;
        self.peak_pow = self.peak_pow.max(pow);
        self.read += iq.len() as u64;
    }

    /// The row this source has earned, if it has earned one yet: when it
    /// closes, and every [`REPORT_S`] while it stays open.
    pub(super) fn row(&mut self, at_us: u64, closed: bool, ring: &Ring) -> Option<Packet> {
        let seconds = self.read as f64 / self.rate;
        let due = closed || seconds - self.reported_s.unwrap_or(0.0) >= REPORT_S;
        if !due || self.read == 0 {
            return None;
        }
        let since = self.reported_s.unwrap_or(0.0);
        self.reported_s = Some(seconds);
        let pow = std::mem::take(&mut self.peak_pow);
        let mut p = Packet::of_pulses(
            at_us,
            self.width_hz as u32,
            Package {
                pulses: Vec::new(),
                snr_db: self.snr_db,
                rssi_dbfs: 10.0 * pow.max(1e-20).log10(),
                start_sample: self.start_sample,
                center_hz: self.center_hz,
                modulation: None,
            },
        );
        p.measure = Some(common::Measure {
            // Nothing classified it. A row that named a modulation here
            // would be this code guessing, which is the one thing the log
            // is for keeping out of.
            modulation: Modulation::Unknown,
            confidence: 0.0,
            front_end: common::FrontEnd::None,
            mode: None,
            duration_us: ((seconds - since) * 1e6) as u32,
            bandwidth_hz: self.width_hz,
            baud: 0.0,
            separation_hz: 0.0,
            sweep_hz_s: 0.0,
            symbol_period_us: 0.0,
        });
        p.iq = ring.burst(ring.base(), ring.end());
        Some(p)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use pipeline::port::StreamSpec;

    fn ring(rate: f64, samples: usize) -> Ring {
        let mut r = Ring::new(StreamSpec::iq(rate, Hz::mhz(433)));
        r.keeps = true;
        r.push(&vec![C32::new(0.5, 0.0); samples]);
        r
    }

    /// A source that opens and closes inside a second leaves one row, and
    /// the row says how long it was on the air and how strong it was.
    #[test]
    fn a_source_that_closes_leaves_one_row() {
        let rate = 1e6;
        let mut e = Evidence::new(433_000_000, 5e6, 21.0, rate).from_sample(4_096);
        let block = vec![C32::new(0.5, 0.0); 100_000];
        e.push(&block);
        assert!(e.row(0, false, &ring(rate, 1)).is_none(), "0.1 s is not a report");
        e.push(&block);
        let p = e.row(0, true, &ring(rate, 4_096)).expect("a row when it closes");
        let m = p.measure.clone().expect("the detector's measurement");
        assert_eq!(m.modulation, Modulation::Unknown);
        assert_eq!(m.duration_us, 200_000, "0.2 s of source");
        assert_eq!(m.bandwidth_hz, 5e6);
        let common::PacketBody::Pulses(pkg) = &p.body else { panic!("{p:?}") };
        assert_eq!(pkg.start_sample, 4_096);
        assert_eq!(pkg.snr_db, 21.0);
        // Half scale in one quadrature is a quarter of the power.
        assert!((pkg.rssi_dbfs - (-6.02)).abs() < 0.05, "{}", pkg.rssi_dbfs);
        assert!(p.iq.is_some(), "a row with no samples behind it");
    }

    /// A carrier that never closes still says it is there, every
    /// [`REPORT_S`] and not oftener.
    #[test]
    fn a_source_that_runs_on_reports_on_an_interval() {
        let rate = 1e6;
        let mut e = Evidence::new(433_000_000, 5e6, 21.0, rate);
        let r = ring(rate, 4_096);
        let block = vec![C32::new(0.5, 0.0); 100_000];
        let mut rows = 0;
        // Twelve seconds of a transmission that never ends.
        for _ in 0..120 {
            e.push(&block);
            if e.row(0, false, &r).is_some() {
                rows += 1;
            }
        }
        assert_eq!(rows, 2, "one row per {REPORT_S} s");
    }
}
