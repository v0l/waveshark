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
use crate::framing::{frame_from_preamble, Framing, MIN_PREAMBLE_BITS};
use crate::slicer::{slice, Coding, Timing};
use crate::whiten::{read_framed, Framed};
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
    /// Where the alternating preamble ended, and the bits after it aligned to
    /// the transmitter's own byte boundaries rather than to whatever edge the
    /// detector triggered on. `None` when the burst has no preamble to align
    /// to. See [`crate::framing`].
    pub framing: Option<Framing>,
    /// The frame read out of the aligned bits, where they turn out to carry
    /// TI-style length-and-CRC framing, whitened or not. See
    /// [`crate::whiten`].
    pub framed: Option<Framed>,
}

impl Analysis {
    /// The bytes worth showing an operator: aligned to the frame where there
    /// was a preamble to align to, and the raw slicer output where there was
    /// not.
    ///
    /// Showing the raw output unconditionally is what makes two receptions of
    /// one device look like two different devices, since the slicer's phase is
    /// whatever the trigger edge happened to be.
    pub fn frame_bytes(&self) -> &[u8] {
        match &self.framing {
            Some(f) => f.frame.as_bytes(),
            None => self.bits.as_bytes(),
        }
    }
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
        let mut s = format!(
            "{coding} {timing}, {} pulses, {} bits, {:.1} ms",
            self.pulses,
            self.bits.len(),
            self.duration_us as f64 / 1000.0
        );
        // The preamble length and the bytes right after it are what identify a
        // device across receptions, long before anything decodes it, so they
        // belong on the one line a scanning operator reads.
        if let Some(f) = &self.framing {
            s.push_str(&format!(
                ", {} bit preamble, {} byte frame, sync {}",
                f.preamble_bits,
                f.content_bytes(),
                f.sync_hex()
            ));
            if !f.repeats.is_empty() {
                s.push_str(&format!(", {} copies", f.repeats.len() + 1));
            }
        }
        if let Some(f) = &self.framed {
            s.push_str(&format!(
                ", {} byte {}frame, CRC ok",
                f.payload.len(),
                if f.whitened { "PN9-whitened " } else { "" }
            ));
        }
        s
    }
}

/// Widest bucket for the histograms, in microseconds.
///
/// Wide enough to absorb the systematic bias a threshold detector puts on
/// every edge, narrow enough to keep 500 us and 1000 us symbols apart. It is
/// the ceiling, not the bucket: see [`tolerance`].
const TOL_US: u32 = 120;

/// Bucket width for this burst, from its own shortest common width.
///
/// A fixed 120 us is right for the half-millisecond symbols of an OOK
/// sensor and wrong by an order of magnitude for a 19.2 kbit/s FSK frame,
/// whose 52 us and 104 us runs it folds into one bucket and reports as 63.
/// So the bucket is a fraction of the width a fifth of the runs are shorter
/// than, which is the symbol or close to it, and never wider than the
/// ceiling.
fn tolerance(pkg: &Package) -> u32 {
    let n = pkg.pulses.len().saturating_sub(1);
    let mut widths: Vec<u32> = pkg.pulses[..n]
        .iter()
        .flat_map(|p| [p.mark, p.gap])
        .filter(|w| *w > 0)
        .collect();
    if widths.len() < 4 {
        return TOL_US;
    }
    widths.sort_unstable();
    let short = widths[widths.len() / 5];
    (short / 3).clamp(8, TOL_US)
}

/// Infer a coding and slice the bits out under it.
///
/// `None` when the burst is too short or too irregular to say anything about,
/// which is better than inventing a reading for what was probably noise.
pub fn analyze(pkg: &Package) -> Option<Analysis> {
    if pkg.pulses.len() < 4 {
        return None;
    }
    let tol = tolerance(pkg);
    let marks = cluster(pkg.mark_histogram(tol));
    let gaps = cluster(pkg.gap_histogram(tol));
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
            //
            // A run of three symbols settles it on its own, whatever the
            // clusters say: Manchester puts a transition in the middle of
            // every bit, so nothing on the wire can stay at one level for
            // longer than two half-symbols, and a single 3T run proves the
            // coding is not Manchester. The clusters cannot see this, because
            // a burst opening with forty bits of alternating preamble buries
            // the handful of long runs in the payload under the count floor.
            // That is why the same 19.2 kbit/s frame was reported as
            // Manchester on one reception and NRZ on the next.
            let ratio = marks[1] as f32 / marks[0].max(1) as f32;
            let symbol = marks[0].min(gaps[0]);
            if (1.6..2.6).contains(&ratio) && longest_run(pkg) <= symbol * 5 / 2 {
                (Coding::Manchester, marks[0], marks[1])
            } else {
                (Coding::Nrz, symbol, symbol)
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
        tolerance_us: (long_us / 2).max(tol),
        reset_us: pkg.pulses.last().map(|p| p.gap).unwrap_or(0),
    };
    let bits = slice(pkg, &t).ok()?;
    if bits.is_empty() {
        return None;
    }
    // The slicer's polarity is a coin toss for FSK, since which tone it called
    // the mark depends on the tuner's sideband as much as on the transmitter.
    // The preamble looks the same either way, so alignment is unaffected, but
    // the framing check has to be offered both readings or half the devices on
    // the band never read.
    let framing = frame_from_preamble(&bits, MIN_PREAMBLE_BITS);
    let framed = framing.as_ref().and_then(read_frame_of);
    Some(Analysis {
        coding,
        short_us,
        long_us,
        pulses: pkg.pulses.len(),
        duration_us: pkg.duration_us(),
        bits,
        framing,
        framed,
    })
}

/// Read a length-and-CRC frame out of aligned bits, if there is one there.
///
/// Both slicer polarities and every bit offset the preamble cut might have
/// swallowed are offered, and the CRC decides. Nothing else can: the cut has
/// no way to see where an alternating preamble stopped and an alternating sync
/// word began, and the mark tone is whichever way round the tuner's sideband
/// left it.
fn read_frame_of(f: &Framing) -> Option<Framed> {
    let n = f.rolled_back.len();
    (0..=crate::framing::ROLLBACK).find_map(|skip| {
        if skip >= n {
            return None;
        }
        let at = f.rolled_back.slice(skip, n - skip);
        read_framed(at.as_bytes()).or_else(|| read_framed(at.inverted().as_bytes()))
    })
}

/// The longest mark or gap in the burst, excluding the gap that ended it:
/// that one is the silence afterwards and says nothing about the coding.
fn longest_run(pkg: &Package) -> u32 {
    let n = pkg.pulses.len().saturating_sub(1);
    pkg.pulses
        .iter()
        .take(n)
        .flat_map(|p| [p.mark, p.gap])
        .chain(pkg.pulses.last().map(|p| p.mark))
        .max()
        .unwrap_or(0)
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
    fn a_fast_fsk_frame_is_read_at_its_own_symbol() {
        // 19.2 kbit/s two-tone keying off the air: runs of one to five
        // 52 us symbols. A 120 us bucket folded the ones and twos together
        // and called the symbol 63 us.
        let runs = [
            (52, 52), (52, 52), (49, 52), (56, 49), (52, 52), (52, 49), (56, 52), (52, 105),
            (56, 154), (154, 49), (59, 101), (255, 63), (150, 150), (56, 52), (52, 52),
            (49, 108), (52, 49), (108, 49), (161, 154), (157, 52), (150, 210), (105, 311),
            (210, 52), (49, 56), (101, 49), (266, 101), (105, 101), (49, 157), (210, 52),
            (262, 154), (101, 157), (157, 49), (56, 52), (206, 157), (52, 308), (52, 10000),
        ];
        let a = analyze(&pkg(&runs)).expect("an analysis");
        assert_eq!(a.coding, Coding::Nrz);
        assert!((48..=56).contains(&a.short_us), "symbol read as {} us", a.short_us);
    }

    /// An NRZ burst as the FSK detector hands one over: runs of like symbols
    /// merged into marks and gaps. `stretch` adds a symbol to the first mark,
    /// which is what a gate opening a fraction of a symbol early does, and is
    /// the reason two receptions of one device come out at different bit
    /// offsets.
    fn nrz_package(bytes: &[u8], sym_us: u32, stretch: bool) -> Package {
        let mut bits: Vec<bool> = Vec::new();
        for &b in bytes {
            for i in (0..8).rev() {
                bits.push(b >> i & 1 != 0);
            }
        }
        // A burst begins on a mark: leading zeros are silence the detector
        // never saw.
        let first = bits.iter().position(|b| *b).unwrap_or(0);
        let mut runs: Vec<(bool, u32)> = Vec::new();
        for &b in &bits[first..] {
            match runs.last_mut() {
                Some(last) if last.0 == b => last.1 += 1,
                _ => runs.push((b, 1)),
            }
        }
        if stretch {
            runs[0].1 += 1;
        }
        let mut pulses: Vec<(u32, u32)> = Vec::new();
        let mut i = 0;
        while i < runs.len() {
            let mark = if runs[i].0 { let w = runs[i].1; i += 1; w } else { 0 };
            let gap = if i < runs.len() && !runs[i].0 { let w = runs[i].1; i += 1; w } else { 0 };
            pulses.push((mark * sym_us, gap * sym_us));
        }
        // The silence that ended the burst. Without it the slicer has no reset
        // gap to cap zero runs against and truncates every run of zeros in the
        // payload.
        if let Some(last) = pulses.last_mut() {
            last.1 = sym_us * 20;
        }
        pkg(&pulses)
    }

    /// Preamble, sync, then a PN9-whitened length-and-CRC frame: what a
    /// CC1101-class transmitter puts on the air with its default settings.
    fn whitened_transmission(payload: &[u8]) -> Vec<u8> {
        transmission(&[0x2d, 0xd4], payload)
    }

    fn transmission(sync: &[u8], payload: &[u8]) -> Vec<u8> {
        let mut frame = vec![payload.len() as u8];
        frame.extend_from_slice(payload);
        frame.extend_from_slice(&crate::whiten::crc16_ti(&frame).to_be_bytes());
        let mut air = vec![0xaa, 0xaa, 0xaa, 0xaa, 0xaa];
        air.extend_from_slice(sync);
        air.extend_from_slice(&crate::whiten::pn9(&frame));
        air
    }

    #[test]
    fn two_receptions_of_one_transmitter_come_out_as_the_same_bytes() {
        // The observation this alignment exists for. The slicer's phase is
        // whatever the gate's trigger edge happened to be, so the raw bits
        // differ between receptions and the two hex dumps share no byte, which
        // makes one device look like two.
        let air = whitened_transmission(&[0xa5, 0x4d, 0xca, 0x18, 0x25, 0x30, 0xbb, 0x1d, 0x6d]);
        let a = analyze(&nrz_package(&air, 52, false)).expect("a");
        let b = analyze(&nrz_package(&air, 52, true)).expect("b");
        assert_ne!(a.bits.as_bytes(), b.bits.as_bytes(), "the two phases were identical, so this test says nothing");
        assert_eq!(a.frame_bytes(), b.frame_bytes(), "alignment did not survive a phase shift");
    }

    #[test]
    fn the_frame_is_aligned_to_the_sync_word_and_read_through_its_whitening() {
        let payload = [0xa5u8, 0x4d, 0xca, 0x18, 0x25, 0x30, 0xbb, 0x1d, 0x6d];
        let air = whitened_transmission(&payload);
        let a = analyze(&nrz_package(&air, 52, false)).expect("analysis");
        let f = a.framing.as_ref().expect("a preamble");
        assert!(f.preamble_bits >= 32, "preamble read as {} bits", f.preamble_bits);
        assert!(f.sync_hex().starts_with("2dd4"), "sync came out as {}", f.sync_hex());
        let framed = a.framed.as_ref().expect("a frame");
        assert!(framed.whitened);
        assert_eq!(framed.payload, payload);
        assert!(a.summary().contains("PN9-whitened"), "{}", a.summary());
    }

    #[test]
    fn a_sync_word_that_carries_on_alternating_still_reads() {
        // 0xb4 opens 1011, and the preamble ends on a 0, so the alternating
        // run swallows the first three bits of the sync word and the cut lands
        // that far into it. Only the CRC can say where the frame really
        // started, so the check has to be offered the earlier offsets.
        let payload = [0xa5u8, 0x4d, 0xca, 0x18, 0x25, 0x30, 0xbb, 0x1d, 0x6d];
        let air = transmission(&[0xb4, 0xd2], &payload);
        let a = analyze(&nrz_package(&air, 52, false)).expect("analysis");
        let framed = a.framed.as_ref().expect("a frame the CRC found");
        assert_eq!(framed.payload, payload);
        assert!(framed.whitened);
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

