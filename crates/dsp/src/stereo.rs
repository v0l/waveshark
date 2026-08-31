//! FM stereo decoding: recover L and R from the multiplex.
//!
//! The broadcast multiplex is
//!
//! ```text
//! MPX = (L+R) + (L-R)·cos(2π·38000·t) + K·cos(2π·19000·t)
//! ```
//!
//! so the difference signal is a double-sideband suppressed-carrier subcarrier
//! whose carrier is exactly twice the pilot. Recovering it needs a carrier
//! locked in both frequency *and* phase, which is what the PLL here provides.
//!
//! Ported from the approach in the MIT-licensed `fmradio` crate, with two
//! changes. That implementation filters L-R through a 257-tap FIR but takes
//! L+R straight from the input, so the two arms differ by the filter's group
//! delay and separation suffers; here both arms go through identical filters
//! and stay aligned. It also band-passes the pilot before the phase detector,
//! which adds a phase shift between the recovered carrier and the subcarrier
//! it has to demodulate; the loop filter is already far narrower than any
//! practical prefilter, so this feeds the phase detector directly.

use crate::fir::{lowpass, FirDecimReal};
use std::f64::consts::TAU;

const PILOT_HZ: f64 = 19_000.0;
/// Stereo blend glide time. Long enough that a brief dropout does not audibly
/// pump, short enough to follow a real change in reception.
const BLEND_TAU_S: f64 = 0.1;
/// Audio bandwidth of each channel.
const AUDIO_HZ: f64 = 15_000.0;

pub struct StereoDecoder {
    rate: f64,
    /// Tracked pilot phase in radians.
    phase: f64,
    /// Tracked pilot frequency in Hz.
    freq: f64,
    err_lp: f64,
    err_alpha: f64,
    kp: f64,
    ki: f64,
    /// Low-passed in-phase pilot amplitude, the lock indicator.
    lock_lp: f64,
    /// Current stereo blend, 0 for mono and 1 for full separation. Smoothed
    /// per block so a signal hovering at the threshold does not pump.
    blend: f32,
    blend_lo: f32,
    blend_hi: f32,
    /// Per-sample smoothing coefficient, so the glide takes the same wall time
    /// regardless of how the caller happens to block up its input.
    blend_alpha: f32,
    /// Low-passed input magnitude, to normalise the lock indicator.
    level_lp: f64,
    sum_lp: FirDecimReal,
    diff_lp: FirDecimReal,
    sum: Vec<f32>,
    diff: Vec<f32>,
    mixed: Vec<f32>,
    /// Tracked pilot phase for each sample of the last block. RDS needs this:
    /// its carrier is the third harmonic and its bit clock the sixteenth
    /// sub-harmonic of the same pilot.
    phases: Vec<f64>,
}

impl StereoDecoder {
    pub fn new(rate: f64) -> Self {
        // 15 kHz audio in a 19 kHz guard: a few hundred taps at the multiplex
        // rate, and both arms share the design so their delays match exactly.
        let taps = lowpass(255, AUDIO_HZ / rate, 70.0);
        // ~50 Hz loop filter. Narrow enough that the audio, the difference
        // subcarrier and RDS all average away in the phase detector, leaving
        // only the pilot.
        let fc = 50.0;
        Self {
            rate,
            phase: 0.0,
            freq: PILOT_HZ,
            err_lp: 0.0,
            err_alpha: (TAU * fc / (rate + TAU * fc)) as f64,
            kp: 0.15,
            ki: 0.0005,
            lock_lp: 0.0,
            blend: 0.0,
            // Below the first the pilot is too weak to trust, above the second
            // it is solid. Between them separation is scaled continuously.
            blend_lo: 0.20,
            blend_hi: 0.55,
            blend_alpha: (1.0 - (-1.0 / (BLEND_TAU_S * rate)).exp()) as f32,
            level_lp: 0.0,
            sum_lp: FirDecimReal::new(taps.clone(), 1),
            diff_lp: FirDecimReal::new(taps, 1),
            sum: Vec::new(),
            diff: Vec::new(),
            mixed: Vec::new(),
            phases: Vec::new(),
        }
    }

    /// 0 when there is no pilot, rising towards 1 as it locks.
    pub fn lock(&self) -> f32 {
        if self.level_lp > 1e-9 {
            // The pilot is nominally 8 to 10% of full deviation, so a locked
            // reading is far below 1; scale so a healthy pilot reads near 1.
            ((self.lock_lp.abs() / self.level_lp) / 0.09).clamp(0.0, 1.0) as f32
        } else {
            0.0
        }
    }

    pub fn is_locked(&self) -> bool {
        self.lock() > 0.3
    }

    /// Recovered pilot frequency, for diagnostics.
    pub fn pilot_freq(&self) -> f64 {
        self.freq
    }

    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.freq = PILOT_HZ;
        self.err_lp = 0.0;
        self.lock_lp = 0.0;
        self.blend = 0.0;
        self.level_lp = 0.0;
        self.sum_lp.reset();
        self.diff_lp.reset();
    }

    /// How much separation is currently being applied, 0 mono to 1 full.
    ///
    /// A hard mono/stereo switch pumps audibly when the signal sits near the
    /// threshold, so this glides instead, in the same spirit as the high
    /// blend. Mono is just the endpoint, not a different output format: the
    /// channel count never changes.
    pub fn blend(&self) -> f32 {
        self.blend
    }

    /// Pilot confidence range over which separation is scaled in.
    pub fn set_blend_range(&mut self, lo: f32, hi: f32) {
        self.blend_lo = lo;
        self.blend_hi = hi.max(lo + 1e-3);
    }

    /// Decode one block of multiplex into left and right.
    /// Run the pilot loop over a block, recording the phase at every sample.
    ///
    /// Always run, even when only mono audio is wanted. RDS is locked to the
    /// pilot's third harmonic and takes its symbol clock from the pilot
    /// divided by sixteen, so a receiver that stops tracking the pilot
    /// because the listener asked for mono stops decoding RDS as well. That
    /// is not what mono means, and it used to be what happened: `phases` was
    /// left empty, and the RDS demodulator zips against it, so it silently
    /// read nothing at all.
    ///
    /// `want_diff` builds the doubled carrier the difference arm needs, which
    /// mono has no use for.
    fn track(&mut self, mpx: &[f32], want_diff: bool) {
        let inc = TAU / self.rate;
        self.mixed.clear();
        self.phases.clear();
        self.phases.reserve(mpx.len());
        if want_diff {
            self.mixed.reserve(mpx.len());
        }

        for &x in mpx {
            let v = x as f64;
            // The carrier for this sample must use this sample's phase. Mixing
            // with the phase after the loop update is one sample late, which
            // at 38 kHz on a 288 kHz multiplex is 47.5 degrees and costs a
            // factor of cos(47.5) in the difference arm.
            let phase = self.phase;
            self.phases.push(phase);
            let (s, c) = phase.sin_cos();

            // Locking to a cosine pilot: the error is -mpx·sin(phase), whose
            // average is (K/2)·sin(pilot - phase), and the in-phase product
            // mpx·cos(phase) averages to the pilot amplitude once locked.
            let err_raw = -v * s;
            self.err_lp += self.err_alpha * (err_raw - self.err_lp);
            self.lock_lp += self.err_alpha * (v * c - self.lock_lp);
            self.level_lp += self.err_alpha * (v.abs() - self.level_lp);

            self.freq += self.ki * self.err_lp;
            // A real pilot is within a few hundred ppm; a wider clamp just
            // lets the loop chase audio during silence.
            self.freq = self.freq.clamp(PILOT_HZ * 0.999, PILOT_HZ * 1.001);
            self.phase += inc * self.freq + self.kp * self.err_lp;
            if self.phase > TAU {
                self.phase -= TAU;
            } else if self.phase < 0.0 {
                self.phase += TAU;
            }

            // Coherent demodulation by the second harmonic. The factor of two
            // undoes the half that falls out of the product.
            if want_diff {
                self.mixed.push((2.0 * v * (2.0 * phase).cos()) as f32);
            }
        }
    }

    pub fn process(&mut self, mpx: &[f32], left: &mut Vec<f32>, right: &mut Vec<f32>) {
        self.track(mpx, true);

        self.sum.clear();
        self.diff.clear();
        self.sum_lp.process(mpx, &mut self.sum);
        self.diff_lp.process(&self.mixed, &mut self.diff);

        // Smoothstep so the ends of the range are flat and the transition has
        // no corner for the ear to catch.
        let t = ((self.lock() - self.blend_lo) / (self.blend_hi - self.blend_lo)).clamp(0.0, 1.0);
        let target = t * t * (3.0 - 2.0 * t);
        let alpha = self.blend_alpha;

        left.clear();
        right.clear();
        left.reserve(self.sum.len());
        right.reserve(self.sum.len());
        for (s, d) in self.sum.iter().zip(&self.diff) {
            self.blend += alpha * (target - self.blend);
            let d = d * self.blend;
            left.push(s + d);
            right.push(s - d);
        }
    }

    /// Pilot phase for each sample of the last processed block.
    pub fn phases(&self) -> &[f64] {
        &self.phases
    }

    /// Mono output, for when there is no pilot or stereo is not wanted.
    pub fn process_mono(&mut self, mpx: &[f32], out: &mut Vec<f32>) {
        // The pilot loop still runs: see `track`. Only the difference arm is
        // skipped, which is the part mono actually does not want.
        self.track(mpx, false);
        out.clear();
        self.sum_lp.process(mpx, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: f64 = 288_000.0;

    /// Mono is an audio decision, not an RDS one.
    ///
    /// The pilot phase is what RDS mixes and clocks against, and the RDS
    /// demodulator zips its input against it, so a phase buffer left empty
    /// does not fail loudly: it decodes nothing and looks like a station with
    /// no RDS. Asking for mono used to do exactly that.
    #[test]
    fn mono_still_tracks_the_pilot_for_rds() {
        let n = 4096;
        let l = tone(n, 1000.0, 0.4, 0.0);
        let mpx = multiplex(&l, &l, 0.1);
        let mut d = StereoDecoder::new(RATE);
        let mut out = Vec::new();
        d.process_mono(&mpx, &mut out);
        assert_eq!(
            d.phases().len(),
            mpx.len(),
            "mono left the pilot phase unusable, so RDS would read nothing"
        );
        assert!(!out.is_empty(), "mono produced no audio");
    }

    /// The two paths must agree about the pilot, or switching to mono would
    /// make the loop start again from nothing.
    #[test]
    fn mono_and_stereo_track_the_pilot_the_same_way() {
        let n = 8192;
        let l = tone(n, 1000.0, 0.4, 0.0);
        let r = tone(n, 700.0, 0.4, 1.0);
        let mpx = multiplex(&l, &r, 0.1);
        let (mut a, mut b) = (StereoDecoder::new(RATE), StereoDecoder::new(RATE));
        let (mut x, mut y, mut z) = (Vec::new(), Vec::new(), Vec::new());
        a.process(&mpx, &mut x, &mut y);
        b.process_mono(&mpx, &mut z);
        assert_eq!(a.phases().len(), b.phases().len());
        let worst = a
            .phases()
            .iter()
            .zip(b.phases())
            .map(|(p, q)| (p - q).abs())
            .fold(0.0f64, f64::max);
        assert!(worst < 1e-9, "the two paths disagree about the pilot by {worst}");
    }

    fn tone(n: usize, hz: f64, amp: f64, phase: f64) -> Vec<f64> {
        (0..n).map(|i| amp * (TAU * hz * i as f64 / RATE + phase).sin()).collect()
    }

    /// Build a broadcast multiplex from separate left and right channels.
    fn multiplex(l: &[f64], r: &[f64], pilot: f64) -> Vec<f32> {
        l.iter()
            .zip(r)
            .enumerate()
            .map(|(i, (a, b))| {
                let t = i as f64 / RATE;
                let sum = a + b;
                let diff = a - b;
                (sum + diff * (TAU * 2.0 * PILOT_HZ * t).cos()
                    + pilot * (TAU * PILOT_HZ * t).cos()) as f32
            })
            .collect()
    }

    fn rms(v: &[f32]) -> f64 {
        (v.iter().map(|x| (*x as f64) * (*x as f64)).sum::<f64>() / v.len().max(1) as f64).sqrt()
    }

    /// Skip the filter's settling time and the PLL's acquisition.
    fn settled(v: &[f32]) -> &[f32] {
        &v[v.len() / 2..]
    }

    #[test]
    fn the_pll_locks_to_a_real_pilot() {
        let n = 200_000;
        let mpx = multiplex(&tone(n, 1000.0, 0.3, 0.0), &tone(n, 1000.0, 0.3, 0.0), 0.1);
        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        d.process(&mpx, &mut l, &mut r);
        assert!(d.is_locked(), "did not lock, indicator {:.3}", d.lock());
        assert!(
            (d.pilot_freq() - PILOT_HZ).abs() < 20.0,
            "locked to {:.1} Hz",
            d.pilot_freq()
        );
    }

    #[test]
    fn there_is_no_lock_without_a_pilot() {
        let n = 200_000;
        // Same programme, pilot amplitude zero.
        let mpx = multiplex(&tone(n, 1000.0, 0.3, 0.0), &tone(n, 800.0, 0.3, 0.0), 0.0);
        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        d.process(&mpx, &mut l, &mut r);
        assert!(!d.is_locked(), "claimed lock on a mono signal: {:.3}", d.lock());
    }

    #[test]
    fn a_signal_only_on_the_left_stays_on_the_left() {
        let n = 300_000;
        let l_in = tone(n, 1000.0, 0.4, 0.0);
        let r_in = vec![0.0; n];
        let mpx = multiplex(&l_in, &r_in, 0.1);

        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        d.process(&mpx, &mut l, &mut r);
        let sep = 20.0 * (rms(settled(&l)) / rms(settled(&r)).max(1e-12)).log10();
        // A one-sample carrier offset drops this to about 14 dB, so the
        // threshold is set well above that to catch it.
        assert!(sep > 40.0, "only {sep:.1} dB of separation");
    }

    #[test]
    fn a_signal_only_on_the_right_stays_on_the_right() {
        let n = 300_000;
        let mpx = multiplex(&vec![0.0; n], &tone(n, 1000.0, 0.4, 0.0), 0.1);
        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        d.process(&mpx, &mut l, &mut r);
        let sep = 20.0 * (rms(settled(&r)) / rms(settled(&l)).max(1e-12)).log10();
        assert!(sep > 40.0, "only {sep:.1} dB of separation");
    }

    #[test]
    fn the_arms_stay_aligned_so_a_mono_signal_cancels() {
        // Identical L and R means L-R is zero, which only comes out right if
        // the sum and difference arms have the same group delay.
        let n = 300_000;
        let t = tone(n, 1000.0, 0.4, 0.0);
        let mpx = multiplex(&t, &t, 0.1);
        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        d.process(&mpx, &mut l, &mut r);
        let (a, b) = (rms(settled(&l)), rms(settled(&r)));
        let imbalance = 20.0 * (a / b.max(1e-12)).log10();
        assert!(imbalance.abs() < 1.0, "channels differ by {imbalance:.2} dB on mono");
    }

    #[test]
    fn block_boundaries_do_not_disturb_the_output() {
        let n = 300_000;
        let mpx = multiplex(&tone(n, 1000.0, 0.4, 0.0), &vec![0.0; n], 0.1);
        let one = {
            let mut d = StereoDecoder::new(RATE);
            let (mut l, mut r) = (Vec::new(), Vec::new());
            d.process(&mpx, &mut l, &mut r);
            l
        };
        let split = {
            let mut d = StereoDecoder::new(RATE);
            let mut acc = Vec::new();
            let (mut l, mut r) = (Vec::new(), Vec::new());
            for chunk in mpx.chunks(4096) {
                d.process(chunk, &mut l, &mut r);
                acc.extend_from_slice(&l);
            }
            acc
        };
        assert_eq!(one.len(), split.len());
        for (i, (a, b)) in one.iter().zip(&split).enumerate().skip(1000) {
            assert!((a - b).abs() < 1e-3, "sample {i} differs: {a} vs {b}");
        }
    }

    #[test]
    fn a_weak_pilot_blends_down_to_mono() {
        // A pilot too weak to trust must collapse separation rather than
        // decode noise into the difference channel.
        let n = 300_000;
        let l_in = tone(n, 1000.0, 0.4, 0.0);
        let mpx = multiplex(&l_in, &vec![0.0; n], 0.002);
        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        d.process(&mpx, &mut l, &mut r);
        assert!(d.blend() < 0.2, "held stereo on a weak pilot: blend {:.2}", d.blend());
        let sep = 20.0 * (rms(settled(&l)) / rms(settled(&r)).max(1e-12)).log10();
        assert!(sep < 6.0, "still separating at {sep:.1} dB with no usable pilot");
    }

    #[test]
    fn a_strong_pilot_reaches_full_separation() {
        let n = 300_000;
        let mpx = multiplex(&tone(n, 1000.0, 0.4, 0.0), &vec![0.0; n], 0.1);
        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        d.process(&mpx, &mut l, &mut r);
        assert!(d.blend() > 0.99, "blend only reached {:.3}", d.blend());
    }

    #[test]
    fn the_blend_glide_does_not_depend_on_block_size() {
        // Smoothing per block rather than per sample makes the glide time a
        // function of how the caller chunks its input, which is a bug that
        // only shows up once the graph feeds a different block size.
        let n = 200_000;
        let mpx = multiplex(&tone(n, 1000.0, 0.4, 0.0), &vec![0.0; n], 0.1);
        let run = |chunk: usize| {
            let mut d = StereoDecoder::new(RATE);
            let (mut l, mut r) = (Vec::new(), Vec::new());
            for c in mpx.chunks(chunk) {
                d.process(c, &mut l, &mut r);
            }
            d.blend()
        };
        let (a, b) = (run(1024), run(65536));
        assert!((a - b).abs() < 1e-3, "blend {a:.4} at 1024 but {b:.4} at 65536");
    }

    #[test]
    fn mono_output_rejects_the_pilot_and_subcarrier() {
        let n = 200_000;
        let mpx = multiplex(&tone(n, 1000.0, 0.4, 0.0), &vec![0.0; n], 0.1);
        let mut d = StereoDecoder::new(RATE);
        let mut out = Vec::new();
        d.process_mono(&mpx, &mut out);
        let g = |f: f64| {
            let x = settled(&out);
            let k = TAU * f / RATE;
            let c = 2.0 * k.cos();
            let (mut a, mut b) = (0.0f64, 0.0f64);
            for &v in x {
                let s = v as f64 + c * a - b;
                b = a;
                a = s;
            }
            (a * a + b * b - c * a * b).sqrt() / x.len() as f64
        };
        let audio = g(1000.0);
        assert!(g(PILOT_HZ) < audio * 0.01, "pilot leaked into mono audio");
        assert!(g(38_000.0) < audio * 0.01, "subcarrier leaked into mono audio");
    }
}

#[cfg(test)]
mod diag {
    use super::*;
    use std::f64::consts::TAU;
    const RATE: f64 = 288_000.0;

    #[test]
    #[ignore]
    fn measure_phase_error() {
        let n = 400_000usize;
        let mpx: Vec<f32> = (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                let l = 0.4 * (TAU * 1000.0 * t).sin();
                (l + l * (TAU * 38_000.0 * t).cos() + 0.1 * (TAU * 19_000.0 * t).cos()) as f32
            })
            .collect();
        let mut d = StereoDecoder::new(RATE);
        let (mut l, mut r) = (Vec::new(), Vec::new());
        // Feed in blocks and report the tracked phase against the true pilot.
        let mut consumed = 0usize;
        for chunk in mpx.chunks(40_000) {
            d.process(chunk, &mut l, &mut r);
            consumed += chunk.len();
            let true_phase = (TAU * 19_000.0 * (consumed as f64) / RATE) % TAU;
            let mut e = d.phase - true_phase;
            while e > std::f64::consts::PI { e -= TAU; }
            while e < -std::f64::consts::PI { e += TAU; }
            let g = |x: &[f32], f: f64| {
                let k = TAU * f / RATE; let c = 2.0 * k.cos();
                let (mut a, mut b) = (0.0f64, 0.0f64);
                for &v in x { let t = v as f64 + c * a - b; b = a; a = t; }
                (a * a + b * b - c * a * b).sqrt() / x.len() as f64
            };
            let rms = |x: &[f32]| (x.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / x.len() as f64).sqrt();
            println!(
                "n={consumed:>7} err {:+7.2}deg  L {:.4} R {:.4} sep {:5.1}dB | R@1k {:.5} R@2k {:.5} R@dc {:.5}",
                e.to_degrees(), rms(&l), rms(&r),
                20.0*(rms(&l)/rms(&r).max(1e-12)).log10(),
                g(&r, 1000.0), g(&r, 2000.0), g(&r, 1.0)
            );
        }
    }
}
