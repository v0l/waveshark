//! Changing a stream's rate by a ratio no decimator can reach.
//!
//! Everything else in this crate decimates by a whole number, because that is
//! what a receiver usually wants: a channel is cut out of a span and the rate
//! it lands on is the span's over some integer. A decoder that insists on one
//! exact rate breaks that. An 802.11 symbol is 64 subcarriers 312.5 kHz apart,
//! so its receiver has to run at 20 MS/s and nothing else; from a LimeSDR's
//! 61.44 MS/s the reachable rates are 20.48, 15.36, 12.288, and none of them
//! is 20. Two and a half percent out is not a small error here: the transform
//! window slips a sample and a half per symbol and nothing decodes.
//!
//! So: interpolate by `l`, filter, keep one sample in `m`, all in one pass.
//! The filter is stored as `l` phases of the same prototype, and only the
//! phase an output needs is ever evaluated, so the cost is the taps per phase
//! per output sample rather than the whole filter at the interpolated rate.

use common::C32;

/// A polyphase resampler by a ratio of whole numbers.
pub struct Rational {
    l: usize,
    m: usize,
    /// Taps per phase, which is the cost of one output sample.
    per_phase: usize,
    /// `phases[p][k]` is tap `p + k * l` of the prototype.
    phases: Vec<Vec<f32>>,
    /// The newest `per_phase` samples twice over, so tap `k` of any phase is
    /// one index below the last, with no wrap to test and nothing to shift.
    /// Half a history per input sample is what shifting the window cost, and
    /// it was as much work again as the taps.
    hist: Vec<C32>,
    /// Where the next input sample goes, and so where the newest one is.
    w: usize,
    acc: usize,
    /// Buffers for [`Rational::process_real`], kept so a per-block call does
    /// not allocate.
    scratch_in: Vec<C32>,
    scratch_out: Vec<C32>,
}

impl Rational {
    /// A resampler from `rate_in` to `rate_out`, or `None` when the ratio in
    /// lowest terms is too large to build a filter for.
    ///
    /// `max_denominator` bounds that: 61.44 to 20 MS/s is 125/384 and cheap,
    /// while two rates that share nothing would ask for a filter with a phase
    /// per output sample of a second.
    pub fn new(rate_in: f64, rate_out: f64, max_denominator: usize) -> Option<Self> {
        let (l, m) = ratio(rate_out, rate_in, max_denominator)?;
        Some(Self::with_ratio(l, m))
    }

    /// The closest resampler to `rate_in` into `rate_out` with a denominator
    /// no larger than `max_denominator`, which always exists.
    ///
    /// For a rate that is not a whole number of hertz. DVB-T runs at 64/7
    /// megasamples a second, so against a radio at 20 MS/s the ratio is 16/35
    /// exactly and against an awkward one it is a continued fraction a few
    /// terms deep. The error left over is parts per billion, which a
    /// decoder's own timing recovery carries the way it carries a crystal.
    pub fn approx(rate_in: f64, rate_out: f64, max_denominator: usize) -> Self {
        let (l, m) = approximate(rate_out / rate_in, max_denominator);
        Self::with_ratio(l, m)
    }

    /// Interpolate by `l` and decimate by `m`, whatever those mean in rates.
    pub fn with_ratio(l: usize, m: usize) -> Self {
        // Designed at the interpolated rate, stopping below whichever Nyquist
        // is lower: the input's when interpolating, the output's when
        // decimating. A tenth of margin keeps the transition out of the band
        // a decoder cares about.
        Self::with_cutoff(l, m, 0.45 / l.max(m) as f64)
    }

    /// The same with the passband edge stated, in cycles per sample of the
    /// interpolated rate.
    ///
    /// For a caller that is cutting a stream into a share of a wider one
    /// rather than changing its rate: several tuners stitched into one span
    /// need each slice trimmed to exactly the width it owns, so the slices
    /// tile instead of overlapping.
    pub fn with_cutoff(l: usize, m: usize, cutoff: f64) -> Self {
        assert!(l >= 1 && m >= 1);
        let per_phase = 24;
        let taps = crate::fir::lowpass((per_phase * l) | 1, cutoff, 60.0);
        let mut phases = vec![Vec::with_capacity(per_phase); l];
        for (p, phase) in phases.iter_mut().enumerate() {
            for k in 0..per_phase {
                // Gain `l`, because interpolation spreads one sample's energy
                // over `l` of them.
                phase.push(taps.get(p + k * l).copied().unwrap_or(0.0) * l as f32);
            }
        }
        Self {
            l,
            m,
            per_phase,
            phases,
            hist: vec![C32::default(); per_phase * 2],
            w: 0,
            acc: 0,
            scratch_in: Vec::new(),
            scratch_out: Vec::new(),
        }
    }

    /// Output samples per input sample, as the ratio it was built for.
    pub fn ratio(&self) -> f64 {
        self.l as f64 / self.m as f64
    }

    /// Whether this would pass its input through untouched.
    pub fn is_identity(&self) -> bool {
        self.l == 1 && self.m == 1
    }

    pub fn reset(&mut self) {
        self.hist.iter_mut().for_each(|x| *x = C32::default());
        self.w = 0;
        self.acc = 0;
    }

    /// Resample `input`, appending to `out`. Continuous across calls.
    pub fn process(&mut self, input: &[C32], out: &mut Vec<C32>) {
        if self.is_identity() {
            out.extend_from_slice(input);
            return;
        }
        out.reserve(input.len() * self.l / self.m + 1);
        let n = self.per_phase;
        for &x in input {
            self.hist[self.w] = x;
            self.hist[self.w + n] = x;
            while self.acc < self.l {
                let h = &self.phases[self.acc];
                let mut sum = C32::default();
                // Tap `k` is `k` samples below the newest, which the doubled
                // history holds at a contiguous run of indices.
                let mut at = self.w + n;
                for &t in h.iter() {
                    sum += self.hist[at] * t;
                    at -= 1;
                }
                out.push(sum);
                self.acc += self.m;
            }
            self.acc -= self.l;
            self.w += 1;
            if self.w == n {
                self.w = 0;
            }
        }
    }

    /// The same for real samples: audio, a discriminator's output, an
    /// envelope. The imaginary half costs half the multiplies and is thrown
    /// away, which is cheap enough that a second filter is not worth keeping.
    pub fn process_real(&mut self, input: &[f32], out: &mut Vec<f32>) {
        if self.is_identity() {
            out.extend_from_slice(input);
            return;
        }
        self.scratch_in.clear();
        self.scratch_in.extend(input.iter().map(|x| C32::new(*x, 0.0)));
        self.scratch_out.clear();
        let (mut i, mut o) =
            (std::mem::take(&mut self.scratch_in), std::mem::take(&mut self.scratch_out));
        self.process(&i, &mut o);
        out.extend(o.iter().map(|c| c.re));
        i.clear();
        self.scratch_in = i;
        self.scratch_out = o;
    }
}

/// How to get from a radio's rate to the rate a decoder wants: decimate by a
/// whole number first, then resample what is left.
///
/// The decimation factor is not simply the ratio rounded. 2.048 MS/s over 46
/// is 44521.7, which has no small ratio to 44100 at all, so a node that
/// rounded refused the rate its radio was running at. Over 40 it is 51200,
/// and 51200 to 44100 is 512 over 441. So try every factor from the largest
/// down and take the first whose remainder is a ratio worth filtering.
///
/// Where no factor leaves an exact ratio at all, the largest one is taken and
/// the remainder approximated. A radio's rate is not always a whole number of
/// convenient hertz: DVB-T asks for 64/7 megasamples a second, and 9142857
/// over any integer has no small exact ratio to 105000, so VDL Mode 2 refused
/// the span it was handed and took the graph down with it. The approximation
/// lands parts per billion out, which a decoder's own timing recovery carries
/// the way it carries a crystal.
///
/// Returns the factor and the resampler, the latter `None` where the
/// decimation alone lands on the wanted rate.
pub fn stage(
    rate_in: f64,
    rate_out: f64,
    max_denominator: usize,
) -> Option<(usize, Option<Rational>)> {
    if rate_in < rate_out {
        return None;
    }
    let most = (rate_in / rate_out).floor().max(1.0) as usize;
    for factor in (1..=most).rev() {
        let mid = rate_in / factor as f64;
        if (mid - rate_out).abs() < 1.0 {
            return Some((factor, None));
        }
        if let Some(r) = Rational::new(mid, rate_out, max_denominator) {
            return Some((factor, Some(r)));
        }
    }
    // Decimating the most leaves the resampler running at the lowest rate,
    // where its phases are cheapest.
    let mid = rate_in / most as f64;
    Some((most, Some(Rational::approx(mid, rate_out, max_denominator))))
}

/// The best rational approximation to `x` with a denominator no larger than
/// `max_denominator`, by continued fractions.
fn approximate(x: f64, max_denominator: usize) -> (usize, usize) {
    assert!(x > 0.0 && x.is_finite(), "a ratio is a positive number");
    let (mut p0, mut q0, mut p1, mut q1) = (0usize, 1usize, 1usize, 0usize);
    let mut v = x;
    loop {
        let a = v.floor() as usize;
        let (p, q) = (a * p1 + p0, a * q1 + q0);
        if q > max_denominator {
            break;
        }
        (p0, q0, p1, q1) = (p1, q1, p, q);
        let rest = v - a as f64;
        if rest < 1e-12 {
            break;
        }
        v = 1.0 / rest;
    }
    // The first term of a ratio below one is zero, which is a convergent of
    // 0/1 and not a resampler; the next term is the first usable one.
    (p1.max(1), q1.max(1))
}

/// `a / b` in lowest terms, or `None` when the denominator is over the bound.
fn ratio(a: f64, b: f64, max_denominator: usize) -> Option<(usize, usize)> {
    // Both rates are whole numbers of hertz in every case this receiver has,
    // so the ratio is exact rather than approximated.
    let (a, b) = (a.round() as u64, b.round() as u64);
    if a == 0 || b == 0 {
        return None;
    }
    let g = gcd(a, b);
    let (l, m) = ((a / g) as usize, (b / g) as usize);
    (l.max(m) <= max_denominator).then_some((l, m))
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

#[cfg(test)]
mod tests {
    use super::Rational;

    /// Every rate a radio runs at reaches every rate a decoder wants.
    ///
    /// The exact search finds a small ratio where one exists, and where none
    /// does the remainder is approximated rather than refused. 9142857 is the
    /// DVB-T rate rounded to hertz and has no small exact ratio to anything:
    /// VDL Mode 2 asked for 105 kHz of it, got nothing, refused its input and
    /// took the whole receiver's graph down. What is asserted here is the
    /// rate that comes out, since that is what a demodulator's clock recovery
    /// has to carry.
    #[test]
    fn an_awkward_span_still_reaches_the_rate_a_decoder_wants() {
        let cases = [
            // The span in the photograph, into VDL Mode 2 and into SSTV's
            // audio rate.
            (9_142_857.0, 105_000.0),
            (9_142_857.0, 44_100.0),
            // And the ordinary ones, which must not get worse.
            (2_400_000.0, 105_000.0),
            (2_048_000.0, 44_100.0),
            (250_000.0, 105_000.0),
            (20_000_000.0, 105_000.0),
            (61_440_000.0, 44_100.0),
        ];
        for (rate_in, want) in cases {
            let (factor, r) = super::stage(rate_in, want, 4096)
                .unwrap_or_else(|| panic!("{rate_in} cannot reach {want}"));
            let mid = rate_in / factor as f64;
            assert!(mid >= want, "{rate_in} decimated by {factor} is below {want}");
            let got = r.as_ref().map(|r| mid * r.ratio()).unwrap_or(mid);
            let ppm = (got - want) / want * 1e6;
            // Ten parts per million, which is what the exact search already
            // allows: it matches on the rates rounded to whole hertz, so
            // 2.048 MS/s into 44.1 kHz has always landed 9.8 ppm out through
            // a ratio it calls exact. Every demodulator here recovers its own
            // symbol clock and carries that the way it carries a crystal.
            assert!(ppm.abs() < 10.0, "{rate_in} into {want} lands {ppm:.3} ppm out");
        }
    }

    /// A rate below what is wanted is the one case with no answer: there are
    /// not enough samples, and inventing them is not resampling.
    #[test]
    fn a_span_narrower_than_the_rate_has_no_answer() {
        assert!(super::stage(44_100.0, 105_000.0, 4096).is_none());
    }

    /// DVB-T runs at 64/7 megasamples a second, which no radio rate divides
    /// into and which is not a whole number of hertz. Against the rates the
    /// radios here run at, the ratio is exact and small; the approximation
    /// only has to work at all, and it has to land within a part per million
    /// so the decoder's own timing carries what is left.
    #[test]
    fn an_irrational_rate_is_approximated_closely() {
        let want = 64_000_000.0 / 7.0;
        for rate in [10_000_000.0, 12_000_000.0, 20_000_000.0, 30_720_000.0, 61_440_000.0] {
            let r = Rational::approx(rate, want, 4096);
            let got = rate * r.ratio();
            let ppm = (got - want) / want * 1e6;
            assert!(ppm.abs() < 1.0, "{rate} lands {ppm} ppm out");
        }
        // 20 MS/s is 35/16 of the DVB-T rate, so the approximation is the
        // exact ratio rather than anything near it.
        assert_eq!(Rational::approx(20_000_000.0, want, 4096).ratio(), 16.0 / 35.0);
    }

    /// Every rate a radio in this receiver runs at has to reach the rates the
    /// decoders want. The ones that bit: 2.048 MS/s to 44.1 kHz, which is
    /// what a HackRF hands an SSTV channel.
    #[test]
    fn every_radio_rate_reaches_the_decoder_rates() {
        for rate in [
            250_000.0,
            1_024_000.0,
            2_048_000.0,
            2_400_000.0,
            2_880_000.0,
            8_000_000.0,
            10_000_000.0,
            20_000_000.0,
            61_440_000.0,
        ] {
            for want in [44_100.0, 48_000.0, 105_000.0, 12_500.0] {
                let (factor, _) = super::stage(rate, want, 4096)
                    .unwrap_or_else(|| panic!("{rate} to {want} has no path"));
                let mid = rate / factor as f64;
                assert!(mid >= want, "{rate} to {want} decimated below the wanted rate");
            }
        }
    }

    use super::*;

    fn tone(hz: f64, rate: f64, n: usize) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let ph = std::f32::consts::TAU * (hz / rate) as f32 * i as f32;
                C32::new(ph.cos(), ph.sin())
            })
            .collect()
    }

    /// The frequency of the strongest bin, and its amplitude.
    fn peak(v: &[C32], rate: f64) -> (f64, f32) {
        use rustfft::FftPlanner;
        let n = 8192.min(v.len() / 2 * 2);
        let mut buf: Vec<C32> = v[v.len() - n..].to_vec();
        FftPlanner::new().plan_fft_forward(n).process(&mut buf);
        let (i, p) = buf
            .iter()
            .enumerate()
            .map(|(i, x)| (i, x.norm()))
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        let k = if i > n / 2 { i as f64 - n as f64 } else { i as f64 };
        (k * rate / n as f64, p / n as f32)
    }

    /// The one this exists for: a LimeSDR's 61.44 MS/s decimated by three
    /// lands on 20.48, and 802.11 needs 20.
    #[test]
    fn a_tone_keeps_its_frequency_and_amplitude_across_the_awkward_ratio() {
        let (rate_in, rate_out) = (20_480_000.0, 20_000_000.0);
        let mut r = Rational::new(rate_in, rate_out, 512).expect("a ratio");
        assert_eq!((r.l, r.m), (125, 128));

        let iq = tone(1_000_000.0, rate_in, 200_000);
        let mut out = Vec::new();
        // In uneven blocks, because a radio delivers what it delivers.
        for block in iq.chunks(4097) {
            r.process(block, &mut out);
        }
        let want = (iq.len() as f64 * r.ratio()) as usize;
        assert!(out.len().abs_diff(want) <= 2, "{} against {want}", out.len());
        let (hz, _) = peak(&out, rate_out);
        assert!((hz - 1_000_000.0).abs() < 5_000.0, "{hz} Hz");
        // The level of a tone is its own, not a bin's: 1 MHz does not land on
        // a bin of this transform and the scalloping alone is 2 dB.
        let settled = &out[out.len() / 2..];
        let rms = (settled.iter().map(|x| x.norm_sqr()).sum::<f32>() / settled.len() as f32).sqrt();
        assert!((rms - 1.0).abs() < 0.02, "level {rms}");
    }

    /// A tone above the output's Nyquist is stopped rather than folded back
    /// in, which is the whole job of the filter inside this.
    #[test]
    fn what_will_not_fit_in_the_output_is_filtered_out() {
        let (rate_in, rate_out) = (20_480_000.0, 5_120_000.0);
        let mut r = Rational::new(rate_in, rate_out, 512).expect("a ratio");
        assert_eq!((r.l, r.m), (1, 4));
        let mut out = Vec::new();
        r.process(&tone(4_000_000.0, rate_in, 100_000), &mut out);
        let settled = &out[out.len() / 2..];
        let rms = (settled.iter().map(|x| x.norm_sqr()).sum::<f32>() / settled.len() as f32).sqrt();
        assert!(rms < 0.01, "an alias came through at {rms}");
    }

    #[test]
    fn a_ratio_no_filter_could_hold_is_refused_and_one_to_one_is_free() {
        // Two rates with nothing in common.
        assert!(Rational::new(1_000_003.0, 48_000.0, 512).is_none());
        let mut r = Rational::new(20e6, 20e6, 512).expect("a ratio");
        assert!(r.is_identity());
        let iq = tone(1e6, 20e6, 1000);
        let mut out = Vec::new();
        r.process(&iq, &mut out);
        assert_eq!(out, iq);
    }
}
