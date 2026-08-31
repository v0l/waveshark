//! Pulse slicers: pulse timings to bits.
//!
//! These are the reusable middle layer that makes wide protocol support
//! practical. rtl_433 covers hundreds of devices with a handful of slicers,
//! because almost every cheap ISM transmitter uses one of a few coding
//! schemes and differs only in its timings and payload layout. So a new
//! protocol is usually a table entry plus a payload parser, not new DSP.

use crate::bits::BitBuffer;
use dsp::pulse::Package;

/// How a pulse train encodes bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Coding {
    /// Pulse width modulation: the *mark* length carries the bit, gaps are
    /// fixed. Short mark is 1, long mark is 0, matching rtl_433 so that
    /// published protocol parameters can be used unchanged.
    Pwm,
    /// Pulse position modulation: marks are uniform and the *gap* carries the
    /// bit. Short gap is 0, long gap is 1, again matching rtl_433, whose PPM
    /// slicer uses the opposite polarity to its PWM one.
    Ppm,
    /// Manchester: each bit is a transition at mid-symbol. The signal is read
    /// as alternating half-symbol runs; a full-length mark or gap is two
    /// half-bits. This is the rtl_433 convention (plain IEEE Manchester),
    /// where a bit is a low-then-high or high-then-low half-pair. The slicer
    /// pairs from the package's first mark, so it is aligned when that mark is
    /// the first half of a symbol; a decoder finds its sync across the
    /// produced stream (it may start a half-symbol in), the way
    /// `OregonV3::decode` searches for its sync word.
    Manchester,
    /// Non-return-to-zero: mark and gap lengths are integer multiples of one
    /// symbol period, marks being 1 and gaps 0.
    Nrz,
}

/// Timing parameters for a protocol, all in microseconds.
///
/// The field names deliberately mirror rtl_433's `r_device`, so a protocol
/// definition can be transcribed from there without reinterpretation.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    pub coding: Coding,
    pub short_us: u32,
    pub long_us: u32,
    /// Fixed gap for PWM, or the sync mark width. Zero when unused.
    pub sync_us: u32,
    /// Tolerance when matching a width. Zero picks a sensible default of a
    /// quarter of the short width.
    pub tolerance_us: u32,
    /// Gap that ends a packet.
    pub reset_us: u32,
}

impl Timing {
    pub fn pwm(short_us: u32, long_us: u32, reset_us: u32) -> Self {
        Self { coding: Coding::Pwm, short_us, long_us, sync_us: 0, tolerance_us: 0, reset_us }
    }

    pub fn ppm(short_us: u32, long_us: u32, reset_us: u32) -> Self {
        Self { coding: Coding::Ppm, short_us, long_us, sync_us: 0, tolerance_us: 0, reset_us }
    }

    /// PWM with a sync mark, which several protocols put before every frame.
    ///
    /// The sync carries no bit. It is dropped rather than used to split the
    /// package, so a burst of repeats slices into one long buffer and the
    /// decoder finds its frame in there by checksum.
    pub fn pwm_sync(short_us: u32, long_us: u32, sync_us: u32, reset_us: u32) -> Self {
        Self { coding: Coding::Pwm, short_us, long_us, sync_us, tolerance_us: 0, reset_us }
    }

    pub fn with_tolerance(mut self, us: u32) -> Self {
        self.tolerance_us = us;
        self
    }

    fn tol(&self) -> u32 {
        if self.tolerance_us > 0 {
            self.tolerance_us
        } else {
            (self.short_us / 4).max(50)
        }
    }

    /// Midpoint between short and long, used to classify a width.
    ///
    /// Classifying against a midpoint rather than matching each width within a
    /// tolerance is what makes this robust to the systematic bias every
    /// envelope detector has. A threshold partway up the pulse edge shortens
    /// every measured mark by the same amount, so absolute widths drift while
    /// their *ratio* holds. Measured widths here run about 55 us under
    /// rtl_433's published figures for exactly this reason, and midpoint
    /// classification is entirely unbothered by it.
    fn midpoint(&self) -> u32 {
        (self.short_us + self.long_us) / 2
    }
}

/// Why a package failed to slice, for diagnostics rather than silent rejection.
#[derive(Clone, Debug, PartialEq)]
pub enum SliceError {
    TooFewPulses { got: usize, need: usize },
    /// A width matched neither symbol.
    BadWidth { index: usize, width_us: u32 },
}

impl std::fmt::Display for SliceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooFewPulses { got, need } => write!(f, "only {got} pulses, need {need}"),
            Self::BadWidth { index, width_us } => {
                write!(f, "pulse {index} has an unclassifiable width of {width_us} us")
            }
        }
    }
}

/// Slice a package into bits according to `t`.
pub fn slice(pkg: &Package, t: &Timing) -> Result<BitBuffer, SliceError> {
    match t.coding {
        Coding::Pwm => slice_pwm(pkg, t),
        Coding::Ppm => slice_ppm(pkg, t),
        Coding::Manchester => slice_manchester(pkg, t),
        Coding::Nrz => slice_nrz(pkg, t),
    }
}

fn slice_pwm(pkg: &Package, t: &Timing) -> Result<BitBuffer, SliceError> {
    if pkg.pulses.len() < 8 {
        return Err(SliceError::TooFewPulses { got: pkg.pulses.len(), need: 8 });
    }
    let mid = t.midpoint();
    let mut b = BitBuffer::with_capacity(pkg.pulses.len());
    for (i, p) in pkg.pulses.iter().enumerate() {
        // Reject anything far outside both symbols rather than forcing it to
        // the nearer one; a wildly wrong width means the package is not this
        // protocol, and guessing would manufacture plausible-looking rubbish.
        let lo = t.short_us.saturating_sub(t.tol() * 2);
        let hi = t.long_us + t.tol() * 2;
        if t.sync_us > 0 && p.mark.abs_diff(t.sync_us) <= t.tol() {
            continue;
        }
        if p.mark < lo || p.mark > hi {
            return Err(SliceError::BadWidth { index: i, width_us: p.mark });
        }
        b.push(p.mark < mid);
    }
    Ok(b)
}

fn slice_ppm(pkg: &Package, t: &Timing) -> Result<BitBuffer, SliceError> {
    if pkg.pulses.len() < 8 {
        return Err(SliceError::TooFewPulses { got: pkg.pulses.len(), need: 8 });
    }
    let mid = t.midpoint();
    let mut b = BitBuffer::with_capacity(pkg.pulses.len());
    // The final gap is the terminating timeout and carries no bit.
    for p in &pkg.pulses[..pkg.pulses.len() - 1] {
        b.push(p.gap >= mid);
    }
    Ok(b)
}

fn slice_manchester(pkg: &Package, t: &Timing) -> Result<BitBuffer, SliceError> {
    let raw = slice_manchester_half(pkg, t)?;
    Ok(manchester_decode(&raw, 0))
}

/// Slice a package into its raw half-symbol level stream: each bit is one
/// half-symbol, mark as 1 and gap as 0, in the order they appear. This is the
/// form rtl_433's OOK_PCM slicer hands to `bitbuffer_manchester_decode`.
///
/// Most Manchester protocols can pair straight from bit 0 and use
/// [`slice`]`/`[`Coding::Manchester`]. A few (Somfy RTS) carry a sync word
/// across the half-symbol stream whose length does not align the data with
/// bit 0, so the decoder must find the frame in this raw stream and pair from
/// an explicit offset with [`manchester_decode`].
pub fn slice_manchester_half(pkg: &Package, t: &Timing) -> Result<BitBuffer, SliceError> {
    if pkg.pulses.len() < 4 {
        return Err(SliceError::TooFewPulses { got: pkg.pulses.len(), need: 4 });
    }
    let half = t.short_us.max(1);
    let mut b = BitBuffer::with_capacity(pkg.pulses.len());
    for (i, p) in pkg.pulses.iter().enumerate() {
        b.extend(true, manchester_halves(p.mark, half));
        if i + 1 < pkg.pulses.len() {
            b.extend(false, manchester_halves(p.gap, half));
        }
    }
    Ok(b)
}

/// Manchester-decode a half-symbol stream (`mark=1`, `gap=0`) starting at
/// `start`, the rtl_433 `bitbuffer_manchester_decode` equivalent: read a bit
/// from each pair, where a level transition means the second half carries the
/// bit (bit 1 = low-then-high, bit 0 = the reverse). A same-level pair is not
/// a Manchester symbol, so it is dropped rather than guessed.
pub fn manchester_decode(raw: &BitBuffer, start: usize) -> BitBuffer {
    let mut out = BitBuffer::with_capacity(raw.len() / 2);
    let mut i = start;
    while i + 1 < raw.len() {
        let (a, b) = (raw.get(i), raw.get(i + 1));
        if let (Some(a), Some(second)) = (a, b) {
            if a != second {
                out.push(second);
            }
        }
        i += 2;
    }
    out
}

/// Number of half-symbols a mark or gap spans. A run narrower than half a
/// symbol (including the zero-width edge some envelope detectors emit) spans
/// nothing; manufacturing a level for it would turn a spurious pulse into a
/// fabricated bit.
fn manchester_halves(width_us: u32, half: u32) -> usize {
    (width_us as f32 / half as f32).round() as usize
}

fn slice_nrz(pkg: &Package, t: &Timing) -> Result<BitBuffer, SliceError> {
    let sym = t.short_us.max(1);
    let mut b = BitBuffer::with_capacity(64);
    for (i, p) in pkg.pulses.iter().enumerate() {
        let m = (p.mark as f32 / sym as f32).round() as usize;
        for _ in 0..m {
            b.push(true);
        }
        if i + 1 < pkg.pulses.len() {
            let g = (p.gap as f32 / sym as f32).round() as usize;
            for _ in 0..g {
                b.push(false);
            }
        }
    }
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use dsp::pulse::Pulse;

    fn pkg(pulses: &[(u32, u32)]) -> Package {
        Package {
            pulses: pulses.iter().map(|(m, g)| Pulse { mark: *m, gap: *g }).collect(),
            snr_db: 20.0,
            rssi_dbfs: -12.0,
            start_sample: 0,
            center_hz: 0,
        }
    }

    #[test]
    fn pwm_short_is_one_long_is_zero() {
        // Fine Offset timings: 544 short, 1524 long.
        let t = Timing::pwm(544, 1524, 2800);
        let p = pkg(&[
            (544, 1000),
            (544, 1000),
            (1524, 1000),
            (544, 1000),
            (1524, 1000),
            (1524, 1000),
            (544, 1000),
            (1524, 1000),
        ]);
        let b = slice(&p, &t).unwrap();
        assert_eq!(b.as_bytes(), &[0b1101_0010]);
    }

    #[test]
    fn pwm_tolerates_the_systematic_short_bias_of_a_real_detector() {
        // Real measured widths from the Fine Offset capture: every mark comes
        // out about 55 us short of the published figure because the detector
        // thresholds partway up the edge. Midpoint classification must not
        // care.
        let t = Timing::pwm(544, 1524, 2800);
        let p = pkg(&[
            (489, 975),
            (489, 975),
            (1465, 975),
            (489, 975),
            (1465, 975),
            (1465, 975),
            (489, 975),
            (1465, 975),
        ]);
        let b = slice(&p, &t).unwrap();
        assert_eq!(b.as_bytes(), &[0b1101_0010]);
    }

    #[test]
    fn pwm_rejects_a_width_belonging_to_no_symbol() {
        let t = Timing::pwm(544, 1524, 2800);
        let mut p = pkg(&[(544, 1000); 8]);
        p.pulses[3].mark = 9000;
        match slice(&p, &t) {
            Err(SliceError::BadWidth { index, width_us }) => {
                assert_eq!(index, 3);
                assert_eq!(width_us, 9000);
            }
            other => panic!("expected BadWidth, got {other:?}"),
        }
    }

    #[test]
    fn ppm_reads_gaps_and_ignores_the_terminator() {
        let t = Timing::ppm(500, 1500, 5000);
        let p = pkg(&[
            (400, 500),
            (400, 1500),
            (400, 500),
            (400, 1500),
            (400, 500),
            (400, 500),
            (400, 1500),
            (400, 500),
            (400, 9999), // terminating timeout, contributes no bit
        ]);
        let b = slice(&p, &t).unwrap();
        assert_eq!(b.len(), 8, "terminator must not become a bit");
        // Long gap is 1, the way rtl_433 reads PPM. Getting this backwards
        // makes every transcribed table decode to nonsense.
        assert_eq!(b.as_bytes(), &[0b0101_0010]);
    }

    #[test]
    fn a_sync_mark_carries_no_bit() {
        // LaCrosse TX141TH: four 833 us sync marks, then 625 us data pulses.
        let t = Timing::pwm_sync(208, 417, 833, 1700);
        let mut pulses = vec![(833, 833); 4];
        pulses.extend([(417, 208), (208, 417), (208, 417), (417, 208)]);
        pulses.extend([(417, 208), (417, 208), (208, 417), (208, 417)]);
        let b = slice(&pkg(&pulses), &t).unwrap();
        assert_eq!(b.len(), 8, "sync marks became bits");
        // Short mark is 1, so the long-mark bits read as zeros here.
        assert_eq!(b.as_bytes(), &[0b0110_0011]);
    }

    #[test]
    fn too_short_a_package_is_rejected_with_a_reason() {
        let t = Timing::pwm(544, 1524, 2800);
        let p = pkg(&[(544, 1000); 3]);
        assert_eq!(
            slice(&p, &t),
            Err(SliceError::TooFewPulses { got: 3, need: 8 })
        );
    }

    // Build a Package from a bit stream using plain IEEE Manchester: bit 1 is
    // a low-then-high half-pair (rising at the symbol center), bit 0 is
    // high-then-low. Consecutive halves alternate as on the wire; a same-level
    // boundary (a long run) appears where equal bits meet.
    fn manchester_ieee(bits: &[bool], half: u32) -> Package {
        let mut lv: Vec<bool> = Vec::new();
        for &b in bits {
            lv.push(!b); // first half
            lv.push(b);  // second half
        }
        let mut runs: Vec<(bool, u32)> = Vec::new();
        for &level in &lv {
            if let Some(last) = runs.last_mut() {
                if last.0 == level {
                    last.1 += 1;
                    continue;
                }
            }
            runs.push((level, 1));
        }
        // Convert to (mark_us, gap_us) pairs. The package starts on a mark, so
        // the first run is high; a leading low (which the envelope detector
        // sees only as the pre-first-mark gap) is dropped, which is the phase
        // choice the slicer makes.
        let mut pulses: Vec<(u32, u32)> = Vec::new();
        let mut idx = 0;
        let _leading = if runs[0].0 { 0 } else { runs[0].1; idx = 1; 0 };
        loop {
            let mut mw = 0;
            while idx < runs.len() && runs[idx].0 {
                mw += runs[idx].1;
                idx += 1;
            }
            let mut gw = 0;
            while idx < runs.len() && !runs[idx].0 {
                gw += runs[idx].1;
                idx += 1;
            }
            if mw == 0 {
                break; // trailing gap only
            }
            pulses.push((mw * half, gw * half));
            if idx >= runs.len() {
                break;
            }
        }
        // Terminator gap so the slicer's final gap check is exercised.
        if *runs.last().map(|(l, _)| l).unwrap_or(&false) {
            pulses.push((0, half));
        }
        Package {
            pulses: pulses.into_iter().map(|(m, g)| Pulse { mark: m, gap: g }).collect(),
            snr_db: 20.0,
            rssi_dbfs: -12.0,
            center_hz: 0,
            start_sample: 0,
        }
    }

    #[test]
    fn manchester_recovers_ieee_bits_when_aligned() {
        // The slicer pairs from the package's first mark, so it is aligned when
        // that mark is the first half of a symbol, i.e. the first bit is 0
        // (high-then-low). Both alternating bits (boundary transitions) and
        // equal bits (long runs) must come back intact.
        let bits: Vec<bool> = vec![false, true, true, false, true, false, false, true];
        let p = manchester_ieee(&bits, 640);
        let t = Timing { coding: Coding::Manchester, short_us: 640, long_us: 1280, ..Timing::pwm(640, 1280, 0) };
        let out = slice(&p, &t).unwrap();
        assert_eq!(out.len(), bits.len(), "bit count");
        for (i, &b) in bits.iter().enumerate() {
            assert_eq!(out.get(i), Some(b), "bit {i}");
        }
    }

    #[test]
    fn manchester_zero_width_edge_adds_no_bit() {
        // Some envelope detectors emit a zero-width edge. It must contribute no
        // level and therefore no bit, not a fabricated one.
        let t = Timing { coding: Coding::Manchester, short_us: 640, long_us: 1280, ..Timing::pwm(640, 1280, 0) };
        let bits = vec![false, true, true, false, true, false, false, true];
        let mut p = manchester_ieee(&bits, 640);
        let clean = slice(&p, &t).unwrap();
        assert_eq!(clean.len(), 8);
        // Prepend a degenerate pulse (zero mark and zero gap): it must
        // contribute no level and therefore no bit.
        p.pulses.insert(0, Pulse { mark: 0, gap: 0 });
        let out = slice(&p, &t).unwrap();
        assert_eq!(out.len(), 8, "zero-width edge became a bit");
    }

    #[test]
    fn nrz_expands_multi_symbol_runs() {
        let t = Timing { coding: Coding::Nrz, ..Timing::pwm(100, 100, 5000) };
        // 2 symbols high, 1 low, then 1 high. The trailing 300 us gap is the
        // terminating timeout, so it contributes no bits.
        let p = pkg(&[(200, 100), (100, 300)]);
        let b = slice(&p, &t).unwrap();
        assert_eq!(b.len(), 4, "terminating gap must not become bits");
        assert_eq!(b.extract(0, 4), Some(0b1101));
    }
}
