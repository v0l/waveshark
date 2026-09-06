//! Recovering RDS bits from the multiplex.
//!
//! The subcarrier sits at 57 kHz, which is the third harmonic of the 19 kHz
//! pilot, and the 1187.5 bit/s data clock is exactly the pilot divided by 16.
//! Both are therefore already available from the stereo PLL, so this needs no
//! carrier recovery and no timing loop of its own: the phase that tracks the
//! pilot generates the carrier, and the same phase divided by 16 generates the
//! symbol clock. That is the whole reason RDS was specified this way.
//!
//! The data is differential Manchester, so each bit occupies one symbol with a
//! guaranteed mid-symbol transition, and the decoded value is the difference
//! between successive symbols rather than their absolute level.

use crate::fir::{lowpass, FirDecimReal};
use std::f64::consts::TAU;

pub const CARRIER_HZ: f64 = 57_000.0;
pub const BAUD: f64 = 1187.5;
/// Nominal pilot, used only until the tracked one is available.
const PILOT_NOMINAL: f64 = 19_000.0;

/// Taps for the shaping filter, from the transition band a Kaiser needs to get
/// from the 2.4 kHz edge to the first thing worth rejecting.
fn shaping_taps(sym_rate: f64) -> usize {
    let transition = DATA_BW * 0.6 / sym_rate;
    let n = ((60.0 - 7.95) / (2.285 * std::f64::consts::TAU * transition)).ceil() as usize;
    (n | 1).clamp(31, 255)
}
/// Bandwidth of the data either side of the carrier.
const DATA_BW: f64 = 2_400.0;

/// Number of symbol-timing hypotheses run in parallel.
///
/// The pilot fixes the clock *frequency* exactly, so the symbol phase is a
/// constant to be found rather than a drift to be tracked. That makes a bank
/// of fixed hypotheses simpler and more robust than a timing loop: there is no
/// loop bandwidth to tune and nothing to lose lock on.
const ARMS: usize = 8;

/// Symbols between timing re-selections.
const RESELECT_SYMS: u64 = 64;

/// Smoothing for timing and carrier estimates, as a fraction per symbol.
/// About fifty symbols, so both resolve well inside the first block sync
/// attempt rather than after it.
const ENERGY_ALPHA: f64 = 0.02;
const CARRIER_ALPHA: f64 = 0.02;

/// Symbols before timing may be frozen, and the margin over the runner-up
/// required to freeze it.
const LOCK_SYMS: u64 = 128;
const LOCK_MARGIN: f64 = 1.05;

/// How far ahead a rival must be to take over, so noise cannot flip the
/// choice back and forth once timing has settled.
const SWITCH_MARGIN: f64 = 1.05;

/// One symbol-timing hypothesis.
#[derive(Clone)]
struct Arm {
    /// Offset within the symbol, a fixed fraction.
    offset: f64,
    /// Index of the symbol currently being integrated, so a boundary is a
    /// change of index rather than a locally accumulated phase. This keeps the
    /// output identical however the caller blocks its input.
    idx: i64,
    first: (f64, f64),
    second: (f64, f64),
    n1: u32,
    n2: u32,
    /// Long-run mean step magnitude. The arm sampling on the symbol boundary
    /// integrates across a transition and averages towards zero, so this
    /// separates the correct phase from the rest.
    energy: f64,
    /// Accumulated square of the symbol vector. BPSK has a 180 degree
    /// ambiguity that squaring removes, so the argument of this is twice the
    /// carrier phase offset.
    carrier: (f64, f64),
    prev_sym: Option<u8>,
}

impl Arm {
    fn new(offset: f64) -> Self {
        Self {
            offset,
            idx: i64::MIN,
            first: (0.0, 0.0),
            second: (0.0, 0.0),
            n1: 0,
            n2: 0,
            energy: 0.0,
            carrier: (0.0, 0.0),
            prev_sym: None,
        }
    }

    fn reset(&mut self, offset: f64) {
        *self = Arm::new(offset);
    }
}

/// Symbol rate the baseband is brought down to, as a multiple of the baud.
/// Twelve samples a symbol leaves the timing bank resolving to about an eighth
/// of a sample without carrying any more rate than the detector needs.
const SYMBOL_OVERSAMPLE: f64 = 12.0;

pub struct RdsDemod {
    rate: f64,
    /// Rate the symbol detector runs at, after decimation.
    sym_rate: f64,
    /// Coarse decimation, then the sharp filter at a rate where it is cheap.
    ///
    /// A 2.4 kHz lowpass directly at the multiplex rate cannot be built: at
    /// 341 kHz a 255-tap Kaiser has a 4.9 kHz transition, twice the width of
    /// the passband it is meant to define, so the data is attenuated and phase
    /// distorted rather than filtered. Decimating first makes the same filter
    /// narrow in normalised terms and far cheaper.
    i_c: FirDecimReal,
    q_c: FirDecimReal,
    i_lp: FirDecimReal,
    q_lp: FirDecimReal,
    i_mid: Vec<f32>,
    q_mid: Vec<f32>,
    dec: usize,
    i_buf: Vec<f32>,
    q_buf: Vec<f32>,
    i_out: Vec<f32>,
    q_out: Vec<f32>,
    arms: Vec<Arm>,
    /// Which hypothesis is currently believed correct.
    best: usize,
    /// Symbols since the last re-selection. Counted rather than re-selecting
    /// per block, so the output does not depend on how the caller chunks its
    /// input.
    syms: u64,
    /// Timing has settled: reported, but not used to stop adapting.
    locked: bool,
    /// Previous pilot phase, for unwrapping.
    prev_pilot: Option<f64>,
    /// Symbols elapsed since the stream started, carried across blocks.
    cum: f64,
    /// Symbol position for each decimated sample still to be consumed.
    sym_pos: Vec<f64>,
    /// Position within the decimation, so the queue stays aligned with the
    /// filters across block boundaries.
    dec_ctr: usize,
    level: f64,
}

impl RdsDemod {
    pub fn new(rate: f64) -> Self {
        let want = BAUD * SYMBOL_OVERSAMPLE;
        let dec = ((rate / want).floor() as usize).max(1);
        let sym_rate = rate / dec as f64;
        // Two separate jobs, and one filter cannot do both. `design_hz` places
        // its stopband where the first alias folds down, which is the right
        // answer for decimating and the wrong one for detection: at these
        // rates it leaves a 7.4 kHz cutoff around 2.4 kHz of data, so three
        // times more noise reaches the detector than signal. The decimator
        // stops aliasing; the shaping filter afterwards defines the bandwidth,
        // and at the decimated rate it is narrow in normalised terms and cheap.
        let shape = lowpass(shaping_taps(sym_rate), DATA_BW / sym_rate, 60.0);
        Self {
            rate,
            sym_rate,
            i_c: FirDecimReal::design_hz(rate, dec, DATA_BW, 60.0),
            q_c: FirDecimReal::design_hz(rate, dec, DATA_BW, 60.0),
            i_lp: FirDecimReal::new(shape.clone(), 1),
            q_lp: FirDecimReal::new(shape, 1),
            i_mid: Vec::new(),
            q_mid: Vec::new(),
            dec,
            i_buf: Vec::new(),
            q_buf: Vec::new(),
            i_out: Vec::new(),
            q_out: Vec::new(),
            arms: (0..ARMS).map(|k| Arm::new(k as f64 / ARMS as f64)).collect(),
            best: 0,
            syms: 0,
            locked: false,
            prev_pilot: None,
            cum: 0.0,
            sym_pos: Vec::new(),
            dec_ctr: 0,
            level: 0.0,
        }
    }

    /// Symbol amplitude, for judging whether RDS is present at all.
    pub fn level(&self) -> f32 {
        self.level as f32
    }

    /// Which timing hypothesis is in use, and how far ahead of the runner-up
    /// it is. A ratio near 1 means timing is not resolved.
    /// Whether symbol timing has been decided and frozen.
    pub fn timing_locked(&self) -> bool {
        self.locked
    }

    pub fn timing(&self) -> (usize, f64) {
        let best = self.arms[self.best].energy;
        let next = self
            .arms
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != self.best)
            .map(|(_, a)| a.energy)
            .fold(0.0f64, f64::max);
        (self.best, if next > 1e-12 { best / next } else { 1.0 })
    }

    /// Filter sizes, for reporting what the chain actually costs.
    pub fn cost(&self) -> String {
        format!(
            "decimate {} taps /{}, shape {} taps, symbol rate {:.0} Hz",
            self.i_c.taps(),
            self.dec,
            self.i_lp.taps(),
            self.sym_rate
        )
    }

    pub fn reset(&mut self) {
        self.i_c.reset();
        self.q_c.reset();
        self.i_lp.reset();
        self.q_lp.reset();
        for (k, a) in self.arms.iter_mut().enumerate() {
            a.reset(k as f64 / ARMS as f64);
        }
        self.best = 0;
        self.syms = 0;
        self.locked = false;
        self.prev_pilot = None;
        self.cum = 0.0;
        self.sym_pos.clear();
        self.dec_ctr = 0;
        self.level = 0.0;
    }

    /// Demodulate a block, appending recovered bits to `bits`.
    ///
    /// `pilot_phase` must hold the stereo PLL's phase for each input sample,
    /// which is what makes the carrier and clock coherent with the broadcast.
    pub fn process(&mut self, mpx: &[f32], pilot_phase: &[f64], bits: &mut Vec<u8>) {
        debug_assert_eq!(mpx.len(), pilot_phase.len());
        self.i_buf.clear();
        self.q_buf.clear();
        self.i_buf.reserve(mpx.len());
        self.q_buf.reserve(mpx.len());

        // Coherent mix to baseband using the pilot's third harmonic. Both
        // quadratures are kept: the standard locks the 57 kHz subcarrier to
        // the pilot's third harmonic but not its phase, so which axis carries
        // the data is not known in advance.
        let d = self.dec;
        for (&x, &p) in mpx.iter().zip(pilot_phase) {
            let c = 3.0 * p;
            let (si, co) = c.sin_cos();
            self.i_buf.push((x as f64 * co) as f32);
            self.q_buf.push((x as f64 * si) as f32);

            // The symbol clock is the pilot divided by sixteen, so advance it
            // from the tracked pilot rather than a nominal baud rate. A
            // receiver crystal is tens of ppm off and a fixed increment drifts
            // against the transmitter for as long as it runs; taking it from
            // the pilot cancels that exactly, which is why the standard tied
            // the two together.
            let step = match self.prev_pilot {
                Some(prev) => {
                    let mut dp = p - prev;
                    while dp > std::f64::consts::PI {
                        dp -= TAU;
                    }
                    while dp < -std::f64::consts::PI {
                        dp += TAU;
                    }
                    dp
                }
                None => TAU * PILOT_NOMINAL / self.rate,
            };
            self.prev_pilot = Some(p);
            self.cum += step / TAU / 16.0;

            if self.dec_ctr == 0 {
                self.sym_pos.push(self.cum);
            }
            self.dec_ctr = (self.dec_ctr + 1) % d;
        }
        self.i_mid.clear();
        self.q_mid.clear();
        let mut i_mid = std::mem::take(&mut self.i_mid);
        let mut q_mid = std::mem::take(&mut self.q_mid);
        self.i_c.process(&self.i_buf, &mut i_mid);
        self.q_c.process(&self.q_buf, &mut q_mid);

        self.i_out.clear();
        self.q_out.clear();
        let mut i_out = std::mem::take(&mut self.i_out);
        let mut q_out = std::mem::take(&mut self.q_out);
        self.i_lp.process(&i_mid, &mut i_out);
        self.q_lp.process(&q_mid, &mut q_out);
        self.i_mid = i_mid;
        self.q_mid = q_mid;

        let n = i_out.len().min(q_out.len()).min(self.sym_pos.len());
        for k in 0..n {
            let pos = self.sym_pos[k];
            let best = self.best;
            let mut boundary = false;
            let v = i_out[k] as f64;
            let w = q_out[k] as f64;

            for (ai, arm) in self.arms.iter_mut().enumerate() {
                let t = pos + arm.offset;
                let idx = t.floor() as i64;
                if arm.idx == i64::MIN {
                    arm.idx = idx;
                }

                // Close the previous symbol first, then accumulate this sample
                // into the new one.
                if idx > arm.idx {
                    arm.idx = idx;
                    let inv1 = if arm.n1 > 0 { 1.0 / arm.n1 as f64 } else { 0.0 };
                    let inv2 = if arm.n2 > 0 { 1.0 / arm.n2 as f64 } else { 0.0 };
                    // Manchester: the mid-symbol step carries the symbol.
                    let sx = arm.first.0 * inv1 - arm.second.0 * inv2;
                    let sy = arm.first.1 * inv1 - arm.second.1 * inv2;
                    arm.first = (0.0, 0.0);
                    arm.second = (0.0, 0.0);
                    arm.n1 = 0;
                    arm.n2 = 0;

                    if inv1 != 0.0 && inv2 != 0.0 {
                        arm.energy += ENERGY_ALPHA * ((sx * sx + sy * sy).sqrt() - arm.energy);
                        // Squaring folds the two BPSK phases together, so this
                        // converges on the modulation axis rather than
                        // cancelling to zero.
                        arm.carrier.0 += CARRIER_ALPHA * ((sx * sx - sy * sy) - arm.carrier.0);
                        arm.carrier.1 += CARRIER_ALPHA * ((2.0 * sx * sy) - arm.carrier.1);

                        let theta = 0.5 * arm.carrier.1.atan2(arm.carrier.0);
                        let (st, ct) = theta.sin_cos();
                        // Rotate onto the estimated axis and take its sign.
                        let proj = sx * ct + sy * st;
                        let sym = if proj >= 0.0 { 1u8 } else { 0u8 };

                        if ai == best {
                            boundary = true;
                            self.level += 0.01 * (proj.abs() - self.level);
                            if let Some(p) = arm.prev_sym {
                                // Differential: the bit is the change.
                                bits.push(sym ^ p);
                            }
                        }
                        arm.prev_sym = Some(sym);
                    }
                }

                if t - arm.idx as f64 >= 0.5 {
                    arm.second.0 += v;
                    arm.second.1 += w;
                    arm.n2 += 1;
                } else {
                    arm.first.0 += v;
                    arm.first.1 += w;
                    arm.n1 += 1;
                }
            }

            if boundary {
                self.syms += 1;
                // Switching arms slips a bit, because a different hypothesis
                // wraps at a different moment, and block sync has to recover.
                // Freezing the choice to avoid that is worse: measured on air
                // the frozen arm was not the strongest one, so it stayed on a
                // worse hypothesis indefinitely. Hysteresis keeps switching
                // rare instead.
                if self.syms.is_multiple_of(RESELECT_SYMS) {
                    // Hysteresis against the incumbent only. Comparing each
                    // candidate against a running maximum instead means an arm
                    // that leads by less than the margin can never take over,
                    // which pins the choice to whichever arm happened to be
                    // ahead first.
                    let (arg, top) = self
                        .arms
                        .iter()
                        .enumerate()
                        .fold((0usize, f64::MIN), |acc, (i, a)| {
                            if a.energy > acc.1 { (i, a.energy) } else { acc }
                        });
                    if top > self.arms[self.best].energy * SWITCH_MARGIN {
                        self.best = arg;
                    }
                    if self.syms >= LOCK_SYMS && self.timing().1 > LOCK_MARGIN {
                        self.locked = true;
                    }
                }
            }
        }

        // Anything not consumed this call stays for the next one.
        self.sym_pos.drain(..n);
        self.i_out = i_out;
        self.q_out = q_out;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    const RATE: f64 = 228_000.0;

    /// Modulate bits onto a 57 kHz subcarrier the way a transmitter does, and
    /// return the multiplex along with the pilot phase per sample.
    pub(super) fn modulate_pub(bits: &[u8]) -> (Vec<f32>, Vec<f64>) { modulate(bits) }

    fn modulate(bits: &[u8]) -> (Vec<f32>, Vec<f64>) {
        modulate_rot(bits, 0.0)
    }

    /// As `modulate`, but with the subcarrier rotated against the pilot.
    fn modulate_rot(bits: &[u8], rot: f64) -> (Vec<f32>, Vec<f64>) {
        let sps = RATE / BAUD;
        let n = (bits.len() as f64 * sps) as usize;
        let mut mpx = Vec::with_capacity(n);
        let mut phase = Vec::with_capacity(n);

        // Differential encode, then Manchester.
        let mut level = 0u8;
        let mut syms = Vec::with_capacity(bits.len());
        for b in bits {
            level ^= b;
            syms.push(level);
        }

        for i in 0..n {
            let t = i as f64 / RATE;
            let sym_pos = i as f64 / sps;
            let idx = sym_pos as usize;
            let frac = sym_pos - idx as f64;
            let s = syms.get(idx).copied().unwrap_or(0);
            // Manchester: high then low for a 1, inverted for a 0.
            let chip = if (frac < 0.5) == (s == 1) { 1.0 } else { -1.0 };
            let pilot_ph = TAU * 19_000.0 * t;
            mpx.push((0.2 * chip * (3.0 * pilot_ph + rot).cos()) as f32);
            phase.push(pilot_ph % TAU);
        }
        (mpx, phase)
    }

    #[test]
    fn a_clean_subcarrier_decodes_back_to_its_bits() {
        let bits: Vec<u8> = (0..400).map(|i| ((i * 7 + i / 3) % 2) as u8).collect();
        let (mpx, ph) = modulate(&bits);
        let mut d = RdsDemod::new(RATE);
        let mut out = Vec::new();
        d.process(&mpx, &ph, &mut out);

        assert!(out.len() > 300, "only {} bits recovered", out.len());
        // Skip acquisition: the timing search and the carrier estimate both
        // need about a hundred symbols, and block sync has not started yet.
        let want = &bits[..];
        let got = &out[120..];
        let mut best = 0usize;
        let mut best_score = 0usize;
        for off in 0..40usize {
            let score = got
                .iter()
                .zip(want[off..].iter())
                .take(200)
                .filter(|(a, b)| a == b)
                .count();
            if score > best_score {
                best_score = score;
                best = off;
            }
        }
        let _ = best;
        assert!(best_score > 190, "only {best_score}/200 bits matched");
    }

    #[test]
    fn timing_is_found_from_any_symbol_phase() {
        // The pilot fixes the clock frequency but not where the symbol
        // boundary falls: that depends on transmitter and receiver filter
        // delay. Starting every test at phase zero hid this entirely, and on
        // real signal it was the difference between decoding and not.
        let bits: Vec<u8> = (0..500).map(|i| ((i * 5 + i / 7) % 2) as u8).collect();
        let (mpx, ph) = modulate(&bits);
        let sps = (RATE / BAUD) as usize;
        for shift in [0usize, sps / 4, sps / 2, 3 * sps / 4] {
            let mut d = RdsDemod::new(RATE);
            let mut out = Vec::new();
            d.process(&mpx[shift..], &ph[shift..], &mut out);
            let got = &out[150..];
            let best = (0..60usize)
                .map(|off| {
                    got.iter().zip(bits[off..].iter()).take(200).filter(|(a, b)| a == b).count()
                })
                .max()
                .unwrap_or(0);
            assert!(
                best > 195,
                "phase shift {shift}: only {best}/200 bits matched, timing not recovered"
            );
        }
    }

    #[test]
    fn the_data_is_recovered_whichever_axis_the_carrier_lands_on() {
        // The standard locks the 57 kHz subcarrier to the pilot's third
        // harmonic but not to its phase, so a receiver that keeps only the
        // in-phase component loses everything when the station happens to
        // transmit in quadrature.
        let bits: Vec<u8> = (0..500).map(|i| ((i * 3 + 1) % 2) as u8).collect();
        for rot in [0.0f64, std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_2] {
            let (mpx, ph) = modulate_rot(&bits, rot);
            let mut d = RdsDemod::new(RATE);
            let mut out = Vec::new();
            d.process(&mpx, &ph, &mut out);
            let got = &out[150..];
            let best = (0..60usize)
                .map(|off| {
                    got.iter().zip(bits[off..].iter()).take(200).filter(|(a, b)| a == b).count()
                })
                .max()
                .unwrap_or(0);
            assert!(
                best > 195,
                "carrier rotated {:.0} deg: only {best}/200 matched",
                rot.to_degrees()
            );
        }
    }

    #[test]
    fn silence_produces_no_confident_level() {
        let mut d = RdsDemod::new(RATE);
        let mut out = Vec::new();
        let n = 20_000;
        let mpx = vec![0.0f32; n];
        let ph: Vec<f64> = (0..n).map(|i| TAU * 19_000.0 * i as f64 / RATE % TAU).collect();
        d.process(&mpx, &ph, &mut out);
        assert!(d.level() < 1e-6, "reported level {} on silence", d.level());
    }

    #[test]
    fn the_bit_rate_matches_the_standard() {
        // 1187.5 baud is exactly the pilot divided by 16, which is what makes
        // the clock recoverable without a timing loop.
        assert!((BAUD * 16.0 - 19_000.0).abs() < 1e-9);
        assert!((CARRIER_HZ - 3.0 * 19_000.0).abs() < 1e-9);
    }

    #[test]
    fn block_boundaries_do_not_lose_or_duplicate_bits() {
        let bits: Vec<u8> = (0..300).map(|i| (i % 3 == 0) as u8).collect();
        let (mpx, ph) = modulate(&bits);
        let whole = {
            let mut d = RdsDemod::new(RATE);
            let mut o = Vec::new();
            d.process(&mpx, &ph, &mut o);
            o
        };
        let split = {
            let mut d = RdsDemod::new(RATE);
            let mut o = Vec::new();
            for (m, p) in mpx.chunks(1000).zip(ph.chunks(1000)) {
                d.process(m, p, &mut o);
            }
            o
        };
        assert_eq!(whole.len(), split.len(), "block splitting changed the bit count");
        assert_eq!(whole, split, "block splitting changed the bits");
    }
}

#[cfg(test)]
mod diag {
    use super::*;

    const RATE: f64 = 228_000.0;

    #[test]
    #[ignore]
    fn timing_search() {
        let bits: Vec<u8> = (0..400).map(|i| ((i * 7 + i / 3) % 2) as u8).collect();
        let (mpx, ph) = super::tests::modulate_pub(&bits);
        let mut d = RdsDemod::new(RATE);
        let mut out = Vec::new();
        for (m, p) in mpx.chunks(20_000).zip(ph.chunks(20_000)) {
            d.process(m, p, &mut out);
            let e: Vec<String> = d.arms.iter().map(|a| format!("{:.3}", a.energy)).collect();
            println!(
                "bits {:4} best {} margin {:.2} locked {} | {}",
                out.len(),
                d.best,
                d.timing().1,
                d.locked,
                e.join(" ")
            );
        }
        // Where do the errors fall?
        let mut best_off = 0;
        let mut best_score = 0;
        for off in 0..40usize {
            let sc = out[40..].iter().zip(bits[off..].iter()).take(200).filter(|(a, b)| a == b).count();
            if sc > best_score { best_score = sc; best_off = off; }
        }
        let mism: Vec<usize> = out[40..].iter().zip(bits[best_off..].iter()).take(200)
            .enumerate().filter(|(_, (a, b))| a != b).map(|(i, _)| i).collect();
        println!("off {best_off} score {best_score}/200 mismatches at {mism:?}");
    }
}
