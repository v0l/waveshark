//! FIR filter design and application.

use crate::window::{kaiser, kaiser_beta_for_atten};
use common::C32;

fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-12 {
        1.0
    } else {
        let px = std::f64::consts::PI * x;
        px.sin() / px
    }
}

/// Windowed-sinc lowpass. `cutoff` is in cycles per sample (0..0.5), measured
/// to the -6 dB point. `taps` should be odd for a true linear-phase type-I
/// filter; an even count is bumped up by one.
pub fn lowpass(taps: usize, cutoff: f64, atten_db: f64) -> Vec<f32> {
    let n = if taps.is_multiple_of(2) { taps + 1 } else { taps };
    let beta = kaiser_beta_for_atten(atten_db);
    let w = kaiser(n, beta);
    let mid = (n - 1) as f64 / 2.0;
    let mut h: Vec<f32> = (0..n)
        .map(|i| (2.0 * cutoff * sinc(2.0 * cutoff * (i as f64 - mid))) as f32 * w[i])
        .collect();
    // Normalise to unity DC gain so cascading filters does not change level.
    let dc: f32 = h.iter().sum();
    if dc.abs() > 1e-20 {
        for v in &mut h {
            *v /= dc;
        }
    }
    h
}

/// Number of taps needed for a given transition width, per Kaiser's estimate.
/// `transition` is in cycles per sample.
pub fn estimate_taps(transition: f64, atten_db: f64) -> usize {
    let n = ((atten_db - 8.0) / (2.285 * 2.0 * std::f64::consts::PI * transition)).ceil();
    (n.max(3.0) as usize) | 1
}

/// Prototype lowpass for an `channels`-path polyphase filter bank.
///
/// Length is forced to `channels * taps_per_branch` exactly, because the
/// polyphase decomposition requires every branch to hold the same tap count.
/// Cutoff sits at half a channel width so adjacent channels cross at -6 dB.
pub fn pfb_prototype(channels: usize, taps_per_branch: usize, atten_db: f64) -> Vec<f32> {
    let n = channels * taps_per_branch;
    let beta = kaiser_beta_for_atten(atten_db);
    let w = kaiser(n, beta);
    let cutoff = 0.5 / channels as f64;
    let mid = (n - 1) as f64 / 2.0;
    let mut h: Vec<f32> = (0..n)
        .map(|i| (2.0 * cutoff * sinc(2.0 * cutoff * (i as f64 - mid))) as f32 * w[i])
        .collect();
    // Scale so a full-scale tone lands at unity in its channel: each branch
    // sees 1/channels of the energy, and the DFT sums the branches back up.
    let dc: f32 = h.iter().sum();
    let g = channels as f32 / dc;
    for v in &mut h {
        *v *= g;
    }
    h
}

/// Direct-form FIR over complex samples with real taps, keeping state across
/// calls so block boundaries are seamless.
#[derive(Clone)]
pub struct Fir {
    taps: Vec<f32>,
    hist: Vec<C32>,
}

impl Fir {
    pub fn new(taps: Vec<f32>) -> Self {
        let n = taps.len();
        Self { taps, hist: vec![C32::new(0.0, 0.0); n] }
    }

    pub fn len(&self) -> usize {
        self.taps.len()
    }

    pub fn is_empty(&self) -> bool {
        self.taps.is_empty()
    }

    pub fn reset(&mut self) {
        self.hist.fill(C32::new(0.0, 0.0));
    }

    /// Filter `input`, appending `input.len()` samples to `out`.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        out.reserve(input.len());
        let n = self.taps.len();
        for &x in input {
            self.hist.copy_within(0..n - 1, 1);
            self.hist[0] = x;
            let mut acc = C32::new(0.0, 0.0);
            for (h, s) in self.taps.iter().zip(self.hist.iter()) {
                acc += s * *h;
            }
            out.push(acc);
        }
    }
}

/// Decimating FIR. Only computes the outputs it keeps, so cost scales with the
/// output rate rather than the input rate.
///
/// Each output is one dot product over the last `n` inputs, and the whole
/// block of them is done in one call with the history joined onto the input,
/// so the inner loop is a straight run over contiguous memory. The taps are
/// stored reversed and doubled up, one copy per component, so an eight-lane
/// register holds four complex samples against four taps and the loop is a
/// plain multiply-accumulate with no shuffling. Measured at 40 MS/s into /15
/// with 381 taps, this is 0.70 ms per 10 ms block against 3.9 ms for the
/// sample-at-a-time loop it replaced.
#[derive(Clone)]
pub struct FirDecim {
    /// The original taps, kept for `taps()` and for anyone who designs a
    /// cascade from them.
    taps: Vec<f32>,
    /// Reversed and interleaved: `[h[n-1], h[n-1], h[n-2], h[n-2], ...]`,
    /// front-padded with zeros to a whole number of registers.
    lanes: Vec<f32>,
    /// Complex samples the padded window spans.
    win: usize,
    /// The last `win - 1` inputs, which the next block's first outputs need.
    tail: Vec<C32>,
    /// `tail` then the block, so every window is one slice.
    joined: Vec<C32>,
    factor: usize,
    /// Inputs seen since the last output.
    phase: usize,
}

/// Lanes in the register the kernels are written for.
const LANES: usize = 8;

/// The inner loop's view of complex samples: pairs of floats.
///
/// `Complex<f32>` is `repr(C)` with `re` then `im`, so this is the same bytes,
/// and it lets the kernel run over one flat slice.
fn as_floats(v: &[C32]) -> &[f32] {
    // SAFETY: Complex<f32> is repr(C) { re: f32, im: f32 } with no padding, so
    // n of them are exactly 2n f32 at the same address and alignment.
    unsafe { std::slice::from_raw_parts(v.as_ptr() as *const f32, v.len() * 2) }
}

/// Every output of one block, as dot products over `joined`.
///
/// `lanes` is twice `win` long.
fn decimate_block(joined: &[C32], lanes: &[f32], win: usize, first: usize, factor: usize, out: &mut Vec<C32>) {
    let x = as_floats(joined);
    let n2 = win * 2;
    debug_assert_eq!(lanes.len(), n2);
    let dot = Dot::pick();
    let mut start = first;
    while start + win <= joined.len() {
        let s = dot.run(&x[start * 2..start * 2 + n2], lanes);
        out.push(C32::new(s[0] + s[2] + s[4] + s[6], s[1] + s[3] + s[5] + s[7]));
        start += factor;
    }
}

/// Eight running sums over two equal slices, lane `k` holding every eighth
/// product from `k`. The caller folds the lanes however its layout needs.
///
/// Picked once per block rather than once per output, and by hand rather
/// than by a vector crate, because a crate chooses its instruction set when
/// it is compiled, which for a shipped binary is the SSE2 baseline. The AVX2
/// path is four FMAs in flight per iteration; the portable one is the same
/// loop written with `wide`, which is two SSE ops per step on x86 and NEON
/// on ARM.
#[derive(Clone, Copy)]
enum Dot {
    #[cfg(target_arch = "x86_64")]
    Avx2Fma,
    Portable,
}

impl Dot {
    #[inline]
    fn pick() -> Self {
        #[cfg(target_arch = "x86_64")]
        if is_x86_feature_detected!("avx2") && is_x86_feature_detected!("fma") {
            return Dot::Avx2Fma;
        }
        Dot::Portable
    }

    #[inline]
    fn run(self, w: &[f32], h: &[f32]) -> [f32; LANES] {
        debug_assert_eq!(w.len(), h.len());
        debug_assert_eq!(w.len() % LANES, 0);
        match self {
            #[cfg(target_arch = "x86_64")]
            // SAFETY: only constructed after the detection in `pick`.
            Dot::Avx2Fma => unsafe { dot_avx2(w, h) },
            Dot::Portable => dot_wide(w, h),
        }
    }
}

#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2,fma")]
unsafe fn dot_avx2(w: &[f32], h: &[f32]) -> [f32; LANES] {
    use std::arch::x86_64::*;
    let n = w.len();
    let (wp, hp) = (w.as_ptr(), h.as_ptr());
    let (mut a0, mut a1, mut a2, mut a3) =
        (_mm256_setzero_ps(), _mm256_setzero_ps(), _mm256_setzero_ps(), _mm256_setzero_ps());
    let mut i = 0;
    while i + 32 <= n {
        a0 = _mm256_fmadd_ps(_mm256_loadu_ps(wp.add(i)), _mm256_loadu_ps(hp.add(i)), a0);
        a1 = _mm256_fmadd_ps(_mm256_loadu_ps(wp.add(i + 8)), _mm256_loadu_ps(hp.add(i + 8)), a1);
        a2 = _mm256_fmadd_ps(_mm256_loadu_ps(wp.add(i + 16)), _mm256_loadu_ps(hp.add(i + 16)), a2);
        a3 = _mm256_fmadd_ps(_mm256_loadu_ps(wp.add(i + 24)), _mm256_loadu_ps(hp.add(i + 24)), a3);
        i += 32;
    }
    while i + 8 <= n {
        a0 = _mm256_fmadd_ps(_mm256_loadu_ps(wp.add(i)), _mm256_loadu_ps(hp.add(i)), a0);
        i += 8;
    }
    let s = _mm256_add_ps(_mm256_add_ps(a0, a1), _mm256_add_ps(a2, a3));
    let mut out = [0.0f32; LANES];
    _mm256_storeu_ps(out.as_mut_ptr(), s);
    out
}

#[inline]
fn dot_wide(w: &[f32], h: &[f32]) -> [f32; LANES] {
    use wide::f32x8;
    let (mut a0, mut a1, mut a2, mut a3) = (f32x8::ZERO, f32x8::ZERO, f32x8::ZERO, f32x8::ZERO);
    let mut wc = w.chunks_exact(LANES * 4);
    let mut hc = h.chunks_exact(LANES * 4);
    for (wq, hq) in (&mut wc).zip(&mut hc) {
        a0 = load(&wq[0..8]).mul_add(load(&hq[0..8]), a0);
        a1 = load(&wq[8..16]).mul_add(load(&hq[8..16]), a1);
        a2 = load(&wq[16..24]).mul_add(load(&hq[16..24]), a2);
        a3 = load(&wq[24..32]).mul_add(load(&hq[24..32]), a3);
    }
    for (wq, hq) in wc.remainder().chunks_exact(LANES).zip(hc.remainder().chunks_exact(LANES)) {
        a0 = load(wq).mul_add(load(hq), a0);
    }
    ((a0 + a1) + (a2 + a3)).to_array()
}

#[inline(always)]
fn load(v: &[f32]) -> wide::f32x8 {
    wide::f32x8::from(<[f32; 8]>::try_from(v).unwrap())
}

impl FirDecim {
    pub fn new(taps: Vec<f32>, factor: usize) -> Self {
        assert!(factor >= 1, "decimation factor must be >= 1");
        let n = taps.len();
        // Pad at the front rather than the end: a zero tap in front reads an
        // older sample that always exists, where one behind would read past
        // the newest sample of the block.
        let win = n.div_ceil(LANES / 2) * (LANES / 2);
        let pad = win - n;
        let mut lanes = vec![0.0f32; pad * 2];
        for &h in taps.iter().rev() {
            lanes.push(h);
            lanes.push(h);
        }
        Self {
            taps,
            lanes,
            win,
            tail: vec![C32::new(0.0, 0.0); win - 1],
            joined: Vec::new(),
            factor,
            phase: 0,
        }
    }

    /// Design and build a decimator in one step. Transition band is placed so
    /// the passband keeps `passband_ratio` of the output Nyquist.
    pub fn design(factor: usize, passband_ratio: f64, atten_db: f64) -> Self {
        let out_nyq = 0.5 / factor as f64;
        let cutoff = out_nyq * passband_ratio;
        let transition = out_nyq - cutoff;
        let taps = estimate_taps(transition.max(1e-4), atten_db);
        Self::new(lowpass(taps, cutoff, atten_db), factor)
    }

    /// Design from real frequencies rather than a passband fraction.
    ///
    /// `passband_hz` is the half-bandwidth that must survive; everything that
    /// would alias into it is pushed into the stopband. Deriving the filter
    /// from the signal instead of from the decimation factor is what keeps it
    /// short: driving the output rate down to the channel bandwidth leaves no
    /// transition band at all and the tap count explodes.
    pub fn design_hz(rate: f64, factor: usize, passband_hz: f64, atten_db: f64) -> Self {
        let out_rate = rate / factor as f64;
        let pb = passband_hz.min(out_rate * 0.45);
        // The first alias folds down from `out_rate`, but the filter can never
        // do anything above the input Nyquist, so clamp there. Without this a
        // factor of 1 asks for a stopband that does not exist.
        let stop = (out_rate - pb).max(pb * 1.05).min(rate * 0.5);
        let transition = ((stop - pb) / rate).max(1e-4);
        let taps = estimate_taps(transition, atten_db);
        let cutoff = (pb + (stop - pb) * 0.5) / rate;
        Self::new(lowpass(taps, cutoff, atten_db), factor)
    }

    pub fn factor(&self) -> usize {
        self.factor
    }

    pub fn taps(&self) -> usize {
        self.taps.len()
    }

    /// Group delay in *output* samples. A symmetric FIR delays by half its
    /// length, and the graph needs this to align the branches of a fan-in.
    pub fn latency(&self) -> usize {
        self.taps.len() / 2 / self.factor
    }

    pub fn reset(&mut self) {
        self.tail.fill(C32::new(0.0, 0.0));
        self.phase = 0;
    }

    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        if input.is_empty() {
            return;
        }
        out.reserve(input.len() / self.factor + 1);
        self.joined.clear();
        self.joined.extend_from_slice(&self.tail);
        self.joined.extend_from_slice(input);
        // The first output lands on the input that brings the phase round to
        // `factor`; its window ends there and starts `win` samples earlier,
        // which is index 0 of `joined` when that input is the first.
        let first = self.factor - 1 - self.phase;
        decimate_block(&self.joined, &self.lanes, self.win, first, self.factor, out);
        self.phase = (self.phase + input.len()) % self.factor;
        let keep = self.tail.len();
        let from = self.joined.len() - keep;
        self.tail.copy_from_slice(&self.joined[from..]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::TAU;

    fn tone(n: usize, freq: f64) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = TAU * freq as f32 * i as f32;
                C32::new(p.cos(), p.sin())
            })
            .collect()
    }

    fn rms(v: &[C32]) -> f32 {
        (v.iter().map(|c| c.norm_sqr()).sum::<f32>() / v.len() as f32).sqrt()
    }

    #[test]
    fn lowpass_has_unity_dc_gain() {
        let h = lowpass(63, 0.1, 60.0);
        assert!((h.iter().sum::<f32>() - 1.0).abs() < 1e-5);
    }

    #[test]
    fn lowpass_passes_dc_and_stops_nyquist() {
        let h = lowpass(127, 0.05, 80.0);
        let mut f = Fir::new(h);
        let mut out = Vec::new();
        f.process(&tone(4096, 0.0), &mut out);
        assert!((rms(&out[512..]) - 1.0).abs() < 0.01, "passband: {}", rms(&out[512..]));

        let mut f = Fir::new(lowpass(127, 0.05, 80.0));
        let mut out = Vec::new();
        f.process(&tone(4096, 0.25), &mut out);
        let stop = 20.0 * rms(&out[512..]).log10();
        assert!(stop < -70.0, "stopband only {stop} dB");
    }

    #[test]
    fn decimator_preserves_a_slow_tone() {
        // 0.01 cycles/sample in, decimate by 8 -> 0.08 cycles/sample out.
        let mut d = FirDecim::design(8, 0.8, 80.0);
        let mut out = Vec::new();
        d.process(&tone(8192, 0.01), &mut out);
        assert_eq!(out.len(), 1024);
        assert!((rms(&out[256..]) - 1.0).abs() < 0.05, "got {}", rms(&out[256..]));
    }

    #[test]
    fn decimator_rejects_out_of_band() {
        // 0.3 cycles/sample would alias badly if not filtered first.
        let mut d = FirDecim::design(8, 0.8, 80.0);
        let mut out = Vec::new();
        d.process(&tone(8192, 0.3), &mut out);
        let lvl = 20.0 * rms(&out[256..]).log10();
        assert!(lvl < -60.0, "alias leaked at {lvl} dB");
    }

    /// The sample-at-a-time loop the block kernel replaced, kept as the
    /// definition of what a decimator produces.
    fn reference(taps: &[f32], factor: usize, chunks: &[&[C32]]) -> Vec<C32> {
        let n = taps.len();
        let mut hist = vec![C32::new(0.0, 0.0); n];
        let mut phase = 0;
        let mut out = Vec::new();
        for chunk in chunks {
            for &x in *chunk {
                hist.rotate_right(1);
                hist[0] = x;
                phase += 1;
                if phase == factor {
                    phase = 0;
                    let mut acc = C32::new(0.0, 0.0);
                    for (k, &h) in taps.iter().enumerate() {
                        acc += hist[k] * h;
                    }
                    out.push(acc);
                }
            }
        }
        out
    }

    #[test]
    fn the_block_kernel_matches_the_sample_loop_at_every_phase() {
        // Every tap count that pads differently, every factor, and chunk
        // sizes that leave the phase counter at every value between calls.
        let sig: Vec<C32> = (0..3001)
            .map(|i| {
                let p = (i as f32 * 0.37).sin() * 3.0;
                C32::new(p.cos() + 0.1 * i as f32 % 1.0, p.sin())
            })
            .collect();
        for taps in [1usize, 3, 4, 5, 8, 9, 31, 32, 33, 101] {
            let h: Vec<f32> = (0..taps).map(|k| ((k * 7 + 3) % 11) as f32 * 0.1 - 0.5).collect();
            for factor in [1usize, 2, 3, 15, 48] {
                for chunk in [1usize, 7, 37, 500] {
                    let chunks: Vec<&[C32]> = sig.chunks(chunk).collect();
                    let want = reference(&h, factor, &chunks);
                    let mut d = FirDecim::new(h.clone(), factor);
                    let mut got = Vec::new();
                    for c in &chunks {
                        d.process(c, &mut got);
                    }
                    assert_eq!(got.len(), want.len(), "taps {taps} factor {factor} chunk {chunk}");
                    for (i, (a, b)) in got.iter().zip(&want).enumerate() {
                        assert!(
                            (a - b).norm() < 1e-3,
                            "taps {taps} factor {factor} chunk {chunk} sample {i}: {a} vs {b}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn both_kernels_agree() {
        // The portable path is what a machine without AVX2 runs, and nothing
        // else here exercises it on one that has it.
        let w: Vec<f32> = (0..96).map(|i| (i as f32 * 0.71).sin()).collect();
        let h: Vec<f32> = (0..96).map(|i| (i as f32 * 0.13).cos()).collect();
        let a = Dot::Portable.run(&w, &h);
        let b = Dot::pick().run(&w, &h);
        for k in 0..LANES {
            assert!((a[k] - b[k]).abs() < 1e-4, "lane {k}: {} vs {}", a[k], b[k]);
        }
    }

    #[test]
    fn block_boundaries_are_seamless() {
        let taps = lowpass(63, 0.1, 60.0);
        let sig = tone(1000, 0.02);

        let mut a = Fir::new(taps.clone());
        let mut one = Vec::new();
        a.process(&sig, &mut one);

        let mut b = Fir::new(taps);
        let mut split = Vec::new();
        for chunk in sig.chunks(37) {
            b.process(chunk, &mut split);
        }
        assert_eq!(one.len(), split.len());
        for (x, y) in one.iter().zip(split.iter()) {
            assert!((x - y).norm() < 1e-6);
        }
    }
}


/// Decimating FIR over real samples, the same shape as [`FirDecim`].
#[derive(Clone)]
pub struct FirDecimReal {
    taps: Vec<f32>,
    /// Reversed, front-padded to a whole number of registers.
    lanes: Vec<f32>,
    win: usize,
    tail: Vec<f32>,
    joined: Vec<f32>,
    factor: usize,
    phase: usize,
}

fn decimate_block_real(joined: &[f32], lanes: &[f32], win: usize, first: usize, factor: usize, out: &mut Vec<f32>) {
    let dot = Dot::pick();
    let mut start = first;
    while start + win <= joined.len() {
        let s = dot.run(&joined[start..start + win], lanes);
        out.push(s.iter().sum());
        start += factor;
    }
}

impl FirDecimReal {
    pub fn new(taps: Vec<f32>, factor: usize) -> Self {
        assert!(factor >= 1, "decimation factor must be >= 1");
        let n = taps.len();
        let win = n.div_ceil(LANES) * LANES;
        let mut lanes = vec![0.0f32; win - n];
        lanes.extend(taps.iter().rev());
        Self {
            taps,
            lanes,
            win,
            tail: vec![0.0; win - 1],
            joined: Vec::new(),
            factor,
            phase: 0,
        }
    }

    pub fn design_hz(rate: f64, factor: usize, passband_hz: f64, atten_db: f64) -> Self {
        let out_rate = rate / factor as f64;
        let pb = passband_hz.min(out_rate * 0.45);
        // The first alias folds down from `out_rate`, but the filter can never
        // do anything above the input Nyquist, so clamp there. Without this a
        // factor of 1 asks for a stopband that does not exist.
        let stop = (out_rate - pb).max(pb * 1.05).min(rate * 0.5);
        let transition = ((stop - pb) / rate).max(1e-4);
        let taps = estimate_taps(transition, atten_db);
        let cutoff = (pb + (stop - pb) * 0.5) / rate;
        Self::new(lowpass(taps, cutoff, atten_db), factor)
    }

    pub fn taps(&self) -> usize {
        self.taps.len()
    }

    pub fn factor(&self) -> usize {
        self.factor
    }

    pub fn reset(&mut self) {
        self.tail.fill(0.0);
        self.phase = 0;
    }

    pub fn process(&mut self, input: &[f32], out: &mut Vec<f32>) {
        if input.is_empty() {
            return;
        }
        out.reserve(input.len() / self.factor + 1);
        self.joined.clear();
        self.joined.extend_from_slice(&self.tail);
        self.joined.extend_from_slice(input);
        let first = self.factor - 1 - self.phase;
        decimate_block_real(&self.joined, &self.lanes, self.win, first, self.factor, out);
        self.phase = (self.phase + input.len()) % self.factor;
        let keep = self.tail.len();
        let from = self.joined.len() - keep;
        self.tail.copy_from_slice(&self.joined[from..]);
    }
}

#[cfg(test)]
mod decim_hz_tests {
    use super::*;

    fn tone(n: usize, hz: f64, rate: f64) -> Vec<f32> {
        (0..n).map(|i| (std::f64::consts::TAU * hz * i as f64 / rate).sin() as f32).collect()
    }

    fn rms(v: &[f32]) -> f32 {
        (v.iter().map(|x| x * x).sum::<f32>() / v.len().max(1) as f32).sqrt()
    }

    #[test]
    fn design_hz_is_far_cheaper_than_squeezing_the_output_rate() {
        // Decimating 2.304 MS/s to a 12.5 kHz channel needs ~8000 taps if the
        // output rate is driven down to the bandwidth, and a few hundred if
        // the filter is designed around the signal instead.
        let squeezed = FirDecim::design(184, 0.8, 70.0);
        let sane = FirDecim::design_hz(2_304_000.0, 48, 6_250.0, 70.0);
        assert!(squeezed.taps() > 4000, "expected the naive design to be huge");
        assert!(sane.taps() < 600, "sane design still {} taps", sane.taps());
    }

    #[test]
    fn the_passband_survives_and_aliases_do_not() {
        let rate = 2_304_000.0;
        let mut d = FirDecimReal::design_hz(rate, 48, 6_250.0, 70.0);
        let mut pass = Vec::new();
        d.process(&tone(200_000, 4_000.0, rate), &mut pass);
        let mut d2 = FirDecimReal::design_hz(rate, 48, 6_250.0, 70.0);
        let mut alias = Vec::new();
        // 48 kHz output: 44 kHz folds back to 4 kHz and must be rejected.
        d2.process(&tone(200_000, 44_000.0, rate), &mut alias);
        let db = 20.0 * (rms(&alias[500..]) / rms(&pass[500..])).log10();
        assert!(db < -60.0, "alias only {db:.1} dB down");
    }

    #[test]
    fn the_real_decimator_matches_the_complex_one() {
        let rate = 288_000.0;
        let sig = tone(20_000, 3_000.0, rate);
        let mut r = FirDecimReal::design_hz(rate, 6, 15_000.0, 70.0);
        let mut c = FirDecim::design_hz(rate, 6, 15_000.0, 70.0);
        let mut ro = Vec::new();
        let mut co = Vec::new();
        r.process(&sig, &mut ro);
        c.process(&sig.iter().map(|&v| C32::new(v, 0.0)).collect::<Vec<_>>(), &mut co);
        assert_eq!(ro.len(), co.len());
        for (a, b) in ro.iter().zip(&co) {
            assert!((a - b.re).abs() < 1e-4, "{a} vs {}", b.re);
        }
    }

    #[test]
    fn decimation_by_one_is_still_filtered_not_bypassed() {
        let rate = 48_000.0;
        let mut d = FirDecimReal::design_hz(rate, 1, 5_000.0, 70.0);
        let mut out = Vec::new();
        d.process(&tone(20_000, 23_000.0, rate), &mut out);
        assert_eq!(out.len(), 20_000, "a factor of 1 must not drop samples");
        assert!(rms(&out[2000..]) < 0.02, "23 kHz survived at {}", rms(&out[2000..]));

        let mut d2 = FirDecimReal::design_hz(rate, 1, 5_000.0, 70.0);
        let mut keep = Vec::new();
        d2.process(&tone(20_000, 2_000.0, rate), &mut keep);
        assert!(rms(&keep[2000..]) > 0.6, "passband lost at {}", rms(&keep[2000..]));
    }
}

