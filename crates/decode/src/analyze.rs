//! Reading bits off a burst nobody claims.
//!
//! A burst that matches no protocol is not nothing. It is a device, and the
//! only way anyone ever adds support for one is by looking at its timings
//! first. Showing "unrecognised burst" and dropping it makes the receiver
//! useless for exactly the case it should be best at, so an unmatched package
//! is analysed here instead: the coding is inferred from the pulse widths, the
//! symbol timings are estimated, and the bits are sliced out under that guess.
//!
//! The guess is a guess, and is labelled as one. It is right often enough to
//! recognise a repeating device ID across several receptions, which is where
//! reverse engineering starts. rtl_433's `-A` analyser exists for the same
//! reason and works the same way.
//!
//! # How the coding is told apart
//!
//! From the shape of the two histograms, which is the same reasoning done by
//! hand when reading a pulse train off a scope:
//!
//! - Two mark widths, one gap width: the mark carries the bit. PWM.
//! - One mark width, two gap widths: the gap carries the bit. PPM.
//! - Two widths in *both*, at roughly T and 2T: Manchester.
//! - One of each: fixed-period keying with no bit in the widths at all, so
//!   every width is an integer number of symbols. NRZ.

use crate::bits::BitBuffer;
use crate::slicer::{slice, Coding, Timing};
use common::pulse::Package;

/// What a burst looks like, and the bits that fall out under that reading.
#[derive(Clone, Debug, PartialEq)]
pub struct Analysis {
    pub coding: Coding,
    /// Short symbol width in microseconds.
    pub short_us: u32,
    /// Long symbol width, equal to `short_us` for codings that have only one.
    pub long_us: u32,
    pub pulses: usize,
    /// On-air time, excluding the gap that ended the burst.
    pub duration_us: u64,
    pub bits: BitBuffer,
}

impl Analysis {
    /// One line naming the coding and its timings, for a packet list.
    pub fn summary(&self) -> String {
        let coding = match self.coding {
            Coding::Pwm => "PWM",
            Coding::Ppm => "PPM",
            Coding::Manchester => "Manchester",
            Coding::Nrz => "NRZ",
        };
        let timing = if self.long_us > self.short_us {
            format!("{}/{} us", self.short_us, self.long_us)
        } else {
            format!("{} us", self.short_us)
        };
        format!(
            "{coding} {timing}, {} pulses, {} bits, {:.1} ms",
            self.pulses,
            self.bits.len(),
            self.duration_us as f64 / 1000.0
        )
    }
}

/// Bucket width for the histograms, in microseconds.
///
/// Wide enough to absorb the systematic bias a threshold detector puts on
/// every edge, narrow enough to keep 500 us and 1000 us symbols apart.
const TOL_US: u32 = 120;

/// Infer a coding and slice the bits out under it.
///
/// `None` when the burst is too short or too irregular to say anything about,
/// which is better than inventing a reading for what was probably noise.
pub fn analyze(pkg: &Package) -> Option<Analysis> {
    if pkg.pulses.len() < 4 {
        return None;
    }
    let marks = cluster(pkg.mark_histogram(TOL_US));
    let gaps = cluster(pkg.gap_histogram(TOL_US));
    if marks.is_empty() {
        return None;
    }

    let (coding, short_us, long_us) = match (marks.len(), gaps.len()) {
        (2, _) if gaps.len() < 2 => (Coding::Pwm, marks[0], marks[1]),
        (1, 2) => (Coding::Ppm, gaps[0], gaps[1]),
        (2, 2) => {
            // Two widths on both sides is Manchester when the pair is one
            // symbol and two, and a four-state line code otherwise. Only the
            // first is worth guessing at.
            let ratio = marks[1] as f32 / marks[0].max(1) as f32;
            if (1.6..2.6).contains(&ratio) {
                (Coding::Manchester, marks[0], marks[1])
            } else {
                (Coding::Nrz, marks[0].min(gaps[0]), marks[0].min(gaps[0]))
            }
        }
        // One width each way carries nothing in the widths themselves, so the
        // symbol period is that width and the data is in the run lengths.
        (1, _) => {
            let sym = marks[0].min(*gaps.first().unwrap_or(&marks[0]));
            (Coding::Nrz, sym, sym)
        }
        _ => {
            // Three or more clusters is either several devices overlapping or
            // a detector coming apart. Fall back to the narrowest width as a
            // symbol period rather than pretending to know the coding.
            let sym = *marks.first()?;
            (Coding::Nrz, sym, sym)
        }
    };

    let t = Timing {
        coding,
        short_us,
        long_us,
        sync_us: 0,
        // Deliberately loose. These timings came from this very burst, so a
        // width that sits between them is jitter, not evidence of a different
        // protocol, and refusing the whole burst over one stray pulse throws
        // away the only look anyone will get at an unknown device.
        tolerance_us: (long_us / 2).max(TOL_US),
        reset_us: pkg.pulses.last().map(|p| p.gap).unwrap_or(0),
    };
    let bits = slice(pkg, &t).ok()?;
    if bits.is_empty() {
        return None;
    }
    Some(Analysis {
        coding,
        short_us,
        long_us,
        pulses: pkg.pulses.len(),
        duration_us: pkg.duration_us(),
        bits,
    })
}

/// Widths that occur often enough to be symbols, shortest first.
///
/// A cluster holding a single pulse out of forty is a glitch, and letting one
/// through turns a clean two-symbol PWM burst into an unclassifiable three.
fn cluster(hist: Vec<(u32, usize)>) -> Vec<u32> {
    let total: usize = hist.iter().map(|(_, n)| *n).sum();
    if total == 0 {
        return Vec::new();
    }
    // A tenth of the pulses, and never a single one: one stray width out of
    // forty turns a clean two-symbol burst into an unclassifiable three. Short
    // bursts have no such luxury, so below eight pulses every width counts.
    let floor = if total >= 8 { (total / 10).max(2) } else { 1 };
    let mut v: Vec<u32> = hist
        .into_iter()
        .filter(|(_, n)| *n >= floor)
        .map(|(c, _)| c)
        .collect();
    v.sort_unstable();
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::pulse::Pulse;

    fn pkg(pulses: &[(u32, u32)]) -> Package {
        Package {
            pulses: pulses.iter().map(|(m, g)| Pulse { mark: *m, gap: *g }).collect(),
            snr_db: 20.0,
            rssi_dbfs: -12.0,
            start_sample: 0,
            center_hz: 0,
            modulation: None,
        }
    }

    /// A PWM train: the mark carries the bit, gaps are fixed.
    fn pwm(bits: &[u8]) -> Package {
        pkg(&bits
            .iter()
            .map(|b| (if *b == 1 { 500 } else { 1500 }, 500))
            .collect::<Vec<_>>())
    }

    #[test]
    fn two_mark_widths_and_one_gap_reads_as_pwm() {
        let a = analyze(&pwm(&[1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1])).expect("analysis");
        assert_eq!(a.coding, Coding::Pwm);
        assert!(a.short_us.abs_diff(500) < 60, "{a:?}");
        assert!(a.long_us.abs_diff(1500) < 60, "{a:?}");
        // Short mark is a one, matching rtl_433, so the bits come back as sent.
        assert_eq!(a.bits.as_bytes()[0], 0b1011_0010);
    }

    #[test]
    fn one_mark_width_and_two_gaps_reads_as_ppm() {
        let p = pkg(&[
            (400, 500),
            (400, 1500),
            (400, 500),
            (400, 1500),
            (400, 1500),
            (400, 500),
            (400, 500),
            (400, 1500),
            (400, 9000),
        ]);
        let a = analyze(&p).expect("analysis");
        assert_eq!(a.coding, Coding::Ppm);
        assert_eq!(a.bits.len(), 8, "the terminating gap is not a bit");
        assert_eq!(a.bits.as_bytes()[0], 0b0101_1001);
    }

    #[test]
    fn widths_at_t_and_2t_on_both_sides_read_as_manchester() {
        let p = pkg(&[
            (200, 400),
            (400, 200),
            (200, 200),
            (400, 400),
            (200, 400),
            (400, 200),
            (200, 200),
            (400, 400),
        ]);
        let a = analyze(&p).expect("analysis");
        assert_eq!(a.coding, Coding::Manchester, "{a:?}");
    }

    #[test]
    fn a_single_width_each_way_reads_as_fixed_period_keying() {
        let p = pkg(&[(100, 100); 12]);
        let a = analyze(&p).expect("analysis");
        assert_eq!(a.coding, Coding::Nrz);
        assert_eq!(a.short_us, 100);
    }

    #[test]
    fn a_lone_odd_width_does_not_invent_a_third_symbol() {
        // One stray mark among forty must not turn PWM into something else.
        let mut p = pwm(&[1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1]);
        p.pulses[7].mark = 2600;
        let a = analyze(&p).expect("analysis");
        assert_eq!(a.coding, Coding::Pwm, "{a:?}");
    }

    #[test]
    fn too_short_a_burst_is_not_guessed_at() {
        assert!(analyze(&pkg(&[(500, 500), (1500, 500)])).is_none());
    }

    #[test]
    fn the_summary_names_the_coding_and_its_timings() {
        let a = analyze(&pwm(&[1, 0, 1, 1, 0, 0, 1, 0])).expect("analysis");
        let s = a.summary();
        assert!(s.starts_with("PWM"), "{s}");
        assert!(s.contains("pulses"), "{s}");
        assert!(s.contains("bits"), "{s}");
    }
}
