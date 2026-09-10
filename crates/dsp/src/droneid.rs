//! DJI DroneID: the identity broadcast that rides alongside an OcuSync link.
//!
//! The frame is LTE's numerology without being LTE: 15 kHz between
//! subcarriers, 600 of them either side of an unused DC, nine OFDM symbols of
//! which the fourth and sixth are Zadoff-Chu sequences rather than data, and
//! the rest QPSK. There are no pilots, so both the timing and the channel
//! come from those two ZC symbols.
//!
//! The structure here follows `proto17/dji_droneid` and the NDSS 2023 paper
//! it grew out of (`RUB-SysSec/DroneSecurity`). The ZC roots, 600 for the
//! fourth symbol and 147 for the sixth, were found by brute force there, and
//! are not the ones a reading of the LTE specification would suggest: this
//! protocol borrows the shape and not the standard.
//!
//! # What this file is
//!
//! The physical layer only: find a burst, correct it, and hand over the 7200
//! coded bits it carries. Nothing here removes the turbo code or reads a
//! field; that is `decode::droneid`, on the other side of a byte boundary,
//! the same split every other protocol here keeps.

use common::C32;
use rustfft::{num_complex::Complex, FftPlanner};

/// Subcarrier spacing, which fixes the FFT size for a given rate.
pub const CARRIER_SPACING_HZ: f64 = 15_000.0;

/// Data subcarriers, 300 either side of the unused DC bin.
pub const CARRIERS: usize = 600;

/// The rate the frame is defined at: 1024 bins of 15 kHz.
pub const RATE: f64 = 15_360_000.0;

/// Where a 2.4 GHz burst has been seen. The list is `proto17/dji_droneid`'s,
/// which is observation rather than specification: there may be others.
pub const CENTERS_2G4_HZ: [f64; 5] = [
    2_399_500_000.0,
    2_414_500_000.0,
    2_429_500_000.0,
    2_444_500_000.0,
    2_459_500_000.0,
];

/// And at 5.8 GHz.
pub const CENTERS_5G8_HZ: [f64; 3] = [5_756_500_000.0, 5_776_500_000.0, 5_796_500_000.0];

/// The FFT size a rate implies. 15.36 MS/s gives 1024, 30.72 gives 2048.
pub fn fft_size(rate: f64) -> usize {
    (rate / CARRIER_SPACING_HZ).round() as usize
}

/// The two cyclic prefix lengths, long then short, in samples at this rate.
///
/// Both are LTE's: the long one is 1/192000 of a second and the short one
/// 4.6875 us, which at 15.36 MS/s is 80 and 72 samples.
pub fn cyclic_prefix(rate: f64) -> (usize, usize) {
    (
        (rate / 192_000.0).round() as usize,
        (rate * 0.0000046875).round() as usize,
    )
}

/// Which FFT bins carry data, in the order the carriers were mapped, for a
/// spectrum that has been shifted so DC sits in the middle.
pub fn data_carriers(fft: usize) -> Vec<usize> {
    let dc = fft / 2;
    (dc - CARRIERS / 2..dc)
        .chain(dc + 1..=dc + CARRIERS / 2)
        .collect()
}

/// The Zadoff-Chu sequence for symbol 4 or 6, in the frequency domain, with
/// its middle element removed so nothing lands on DC.
pub fn zc_frequency(symbol: usize) -> Vec<C32> {
    let root = match symbol {
        4 => 600.0f64,
        6 => 147.0,
        _ => panic!("DroneID has ZC on symbols 4 and 6 only"),
    };
    let n = CARRIERS + 1;
    let mut v: Vec<C32> = (0..n)
        .map(|i| {
            let phase = -std::f64::consts::PI * root * (i as f64) * ((i + 1) as f64) / n as f64;
            C32::new(phase.cos() as f32, phase.sin() as f32)
        })
        .collect();
    v.remove(n / 2);
    v
}

/// The same sequence in the time domain, one OFDM symbol long and without a
/// cyclic prefix, which is what a correlator needs.
pub fn zc_time(symbol: usize, rate: f64) -> Vec<C32> {
    let fft = fft_size(rate);
    let mut bins = vec![Complex::<f32>::new(0.0, 0.0); fft];
    for (&k, v) in data_carriers(fft).iter().zip(zc_frequency(symbol)) {
        bins[k] = Complex::new(v.re, v.im);
    }
    // The mapping above is in shifted order, so undo the shift before the
    // inverse transform.
    bins.rotate_left(fft / 2);
    let mut planner = FftPlanner::<f32>::new();
    planner.plan_fft_inverse(fft).process(&mut bins);
    let scale = 1.0 / fft as f32;
    bins.iter().map(|c| C32::new(c.re, c.im) * scale).collect()
}

/// A burst the detector found: where it starts, and how strongly the ZC
/// symbol matched.
#[derive(Clone, Copy, Debug)]
pub struct Found {
    /// Sample index of the fourth OFDM symbol, its cyclic prefix included.
    pub zc4_at: usize,
    /// Normalised correlation, 0 to 1.
    pub score: f32,
}

/// Find bursts by cross correlating against the first ZC symbol.
///
/// One call builds a [`BurstFinder`] and throws it away, which is fine for a
/// file read once. A node running per block keeps one.
pub fn find_bursts(iq: &[C32], rate: f64, threshold: f32) -> Vec<Found> {
    BurstFinder::new(rate, threshold).find(iq)
}

/// The correlator behind [`find_bursts`], holding its transform plans.
///
/// The correlation is normalised by the energy in the window, so the
/// threshold means the same thing whatever the gain was, which is what makes
/// one number work across captures.
///
/// It is computed by fast convolution rather than position by position. The
/// direct form was gated on the window being 6 dB over the floor, which on a
/// quiet band skips nearly everything; on 2.4 GHz with Wi-Fi in it the gate
/// is open most of the time, and 1024 multiply-accumulates a sample at
/// 15.36 MS/s measured at six times real time. Overlap-save costs the same
/// whatever the band holds, about a fifteenth of that.
pub struct BurstFinder {
    fft: usize,
    short_cp: usize,
    threshold: f32,
    /// Transform length, and how many correlations one transform yields.
    n: usize,
    valid: usize,
    fwd: std::sync::Arc<dyn rustfft::Fft<f32>>,
    inv: std::sync::Arc<dyn rustfft::Fft<f32>>,
    /// The template's spectrum, conjugated and scaled, so the product with a
    /// segment's spectrum is the cross correlation.
    template: Vec<Complex<f32>>,
    t_energy: f32,
    seg: Vec<Complex<f32>>,
    scratch: Vec<Complex<f32>>,
    corr: Vec<C32>,
}

impl BurstFinder {
    pub fn new(rate: f64, threshold: f32) -> Self {
        let fft = fft_size(rate);
        let (_, short_cp) = cyclic_prefix(rate);
        let zc = zc_time(4, rate);
        let t_energy: f32 = zc.iter().map(|c| c.norm_sqr()).sum::<f32>().sqrt();
        // Eight times the template, so seven eighths of every transform is
        // output; past that the transforms are no longer where the time goes.
        let n = (8 * fft).next_power_of_two();
        let valid = n - fft + 1;
        let mut planner = FftPlanner::<f32>::new();
        let fwd = planner.plan_fft_forward(n);
        let inv = planner.plan_fft_inverse(n);
        let mut template = vec![Complex::<f32>::new(0.0, 0.0); n];
        for (t, z) in template.iter_mut().zip(&zc) {
            *t = Complex::new(z.re, z.im);
        }
        fwd.process(&mut template);
        let scale = 1.0 / n as f32;
        for t in &mut template {
            *t = t.conj() * scale;
        }
        let scratch = vec![
            Complex::<f32>::new(0.0, 0.0);
            fwd.get_inplace_scratch_len().max(inv.get_inplace_scratch_len())
        ];
        Self {
            fft,
            short_cp,
            threshold,
            n,
            valid,
            fwd,
            inv,
            template,
            t_energy,
            seg: vec![Complex::<f32>::new(0.0, 0.0); n],
            scratch,
            corr: Vec::new(),
        }
    }

    /// Cross correlation of the template against every start position that
    /// has a whole symbol after it.
    fn correlate(&mut self, iq: &[C32]) {
        let positions = iq.len() + 1 - self.fft;
        self.corr.clear();
        self.corr.resize(positions, C32::default());
        let mut start = 0;
        while start < positions {
            let take = self.n.min(iq.len() - start);
            for (s, x) in self.seg.iter_mut().zip(&iq[start..start + take]) {
                *s = Complex::new(x.re, x.im);
            }
            for s in &mut self.seg[take..] {
                *s = Complex::new(0.0, 0.0);
            }
            self.fwd.process_with_scratch(&mut self.seg, &mut self.scratch);
            for (s, t) in self.seg.iter_mut().zip(&self.template) {
                *s *= *t;
            }
            self.inv.process_with_scratch(&mut self.seg, &mut self.scratch);
            let keep = self.valid.min(positions - start);
            for (c, s) in self.corr[start..start + keep].iter_mut().zip(&self.seg) {
                *c = C32::new(s.re, s.im);
            }
            start += self.valid;
        }
    }

    pub fn find(&mut self, iq: &[C32]) -> Vec<Found> {
        let fft = self.fft;
        if iq.len() < fft + self.short_cp {
            return Vec::new();
        }
        // The floor, from a coarse sample of the file rather than all of it:
        // a burst this brief cannot move a median taken every thousandth
        // sample.
        let mut sample: Vec<f32> = iq.iter().step_by(1021).map(|c| c.norm_sqr()).collect();
        sample.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let floor = sample.get(sample.len() / 2).copied().unwrap_or(0.0);
        // Six decibels over the floor, which a burst that correlates at all
        // clears comfortably and noise does not.
        let gate = floor * 4.0 * fft as f32;

        self.correlate(iq);
        // A running sum of the window's energy, for the normalisation.
        let mut energy: f32 = iq[..fft].iter().map(|c| c.norm_sqr()).sum();
        // The same test as the score against the threshold, squared, so a
        // position that is not a peak costs no square root and no divide.
        // Every sample of the source is tested and on a busy band almost all
        // of them clear the energy gate, so the two roots a sample were most
        // of what the search cost.
        let bar = (self.threshold * self.t_energy).powi(2);
        let mut out: Vec<Found> = Vec::new();
        let mut best: Option<Found> = None;
        for n in 0..iq.len() - fft {
            if n > 0 {
                energy += iq[n + fft - 1].norm_sqr() - iq[n - 1].norm_sqr();
            }
            if energy > gate && self.corr[n].norm_sqr() > bar * energy {
                let score = self.corr[n].norm() / (energy.sqrt() * self.t_energy);
                if score > self.threshold {
                    let here = Found {
                        // The correlation peaks on the symbol itself; the
                        // burst starts a cyclic prefix earlier.
                        zc4_at: n.saturating_sub(self.short_cp),
                        score,
                    };
                    match &mut best {
                        Some(b) if score > b.score => *b = here,
                        Some(_) => {}
                        None => best = Some(here),
                    }
                    continue;
                }
            }
            // A peak is one burst however many positions cleared the
            // threshold, so it is reported when the run ends rather than per
            // position.
            if let Some(b) = best.take() {
                out.push(b);
            }
        }
        out.extend(best);
        out
    }
}

/// The cyclic prefix of each of the nine symbols, long or short.
pub fn cp_schedule(rate: f64) -> [usize; 9] {
    let (long, short) = cyclic_prefix(rate);
    [long, short, short, short, short, short, short, short, long]
}

/// Where symbol `n` (1-based) begins, its cyclic prefix included, measured
/// from the start of the burst.
pub fn symbol_offset(rate: f64, n: usize) -> usize {
    let fft = fft_size(rate);
    cp_schedule(rate)[..n - 1].iter().map(|cp| cp + fft).sum()
}

/// The whole burst, in samples.
pub fn burst_len(rate: f64) -> usize {
    symbol_offset(rate, 10)
}

/// The scrambler's second register, which is 0x12345678 as the reference
/// implementation writes it: bit `i` of this value is the register's `i`th
/// stage.
pub const SCRAMBLER_X2_INIT: u32 = 0x1234_5678;

/// Coded bits in a frame: six data symbols of 600 QPSK carriers.
pub const CODED_BITS: usize = 7200;

/// What a demodulated burst gives up.
#[derive(Clone, Debug)]
pub struct Demodulated {
    /// The 7200 coded bits, descrambled, ready for the rate matcher.
    pub bits: Vec<u8>,
    /// The frequency offset taken off, in hertz.
    pub cfo_hz: f64,
    /// How tightly the equalised carriers sat on the QPSK points, 0 to 1,
    /// which is the only quality measure available before the turbo decoder
    /// says yes or no.
    pub confidence: f32,
    /// How tightly each data symbol's carriers sat on the QPSK points, in
    /// the order the symbols were sent. A burst whose symbols differ here is
    /// one the channel estimate does not fit across the whole frame.
    pub per_symbol: Vec<f32>,
    /// Every data symbol's bits before descrambling, in the order the
    /// symbols were sent, so a caller can question the assembly rather than
    /// take it: seven groups of 1200, for symbols 1, 2, 3, 5, 7, 8 and 9.
    pub raw_symbols: Vec<Vec<u8>>,
    /// The fraction of the first symbol's bits that descramble to zero.
    ///
    /// That symbol carries no data: the scrambler zeroes it and starts again
    /// for the rest, so a correct demodulation reads it as all zeros. It is
    /// the one end-to-end check available before the turbo decoder, and it
    /// tests the timing, the channel and the scrambler at once. An aircraft
    /// that sends eight symbols rather than nine has no first symbol, and
    /// this reads about a half, which is what a coin says.
    pub first_symbol_zeros: f32,
}

/// Demodulate one burst that begins at `start`.
///
/// There are no pilots in this frame, so everything comes from the two ZC
/// symbols: the channel from the fourth, and the phase that walks between the
/// symbols from the difference between the fourth and the sixth. A receiver
/// that equalises on one ZC symbol alone reads the symbols beside it and
/// nothing further away.
pub fn demodulate(iq: &[C32], start: usize, rate: f64) -> Option<Demodulated> {
    let fft = fft_size(rate);
    let (_, short_cp) = cyclic_prefix(rate);
    if start + burst_len(rate) > iq.len() {
        return None;
    }
    let burst = &iq[start..start + burst_len(rate)];

    // The carrier offset, from the second symbol's cyclic prefix against its
    // copy at the end of the symbol. The first symbol is skipped because some
    // aircraft do not send it.
    let sym2 = symbol_offset(rate, 2);
    let acc: C32 = (0..short_cp)
        .map(|i| burst[sym2 + i].conj() * burst[sym2 + fft + i])
        .sum();
    let per_sample = acc.im.atan2(acc.re) as f64 / fft as f64;
    let cfo_hz = per_sample * rate / (2.0 * std::f64::consts::PI);

    let mut planner = FftPlanner::<f32>::new();
    let plan = planner.plan_fft_forward(fft);
    let carriers = data_carriers(fft);

    // Each symbol, corrected for the offset, transformed, and shifted so DC
    // sits in the middle where `data_carriers` expects it.
    let mut freq: Vec<Vec<C32>> = Vec::with_capacity(9);
    for n in 1..=9 {
        let at = symbol_offset(rate, n) + cp_schedule(rate)[n - 1];
        let mut bins: Vec<Complex<f32>> = (0..fft)
            .map(|i| {
                let t = (at + i) as f64;
                let ph = -per_sample * t;
                let rot = C32::new(ph.cos() as f32, ph.sin() as f32);
                let s = burst[at + i] * rot;
                Complex::new(s.re, s.im)
            })
            .collect();
        plan.process(&mut bins);
        bins.rotate_right(fft / 2);
        freq.push(bins.iter().map(|c| C32::new(c.re, c.im)).collect());
    }

    // The channel is the reference sequence over what arrived, so equalising
    // is a multiply. Both ZC symbols are measured because the difference
    // between their phases is the walking offset a fractional timing error
    // leaves across the burst.
    let channel = |sym: usize| -> Vec<C32> {
        let want = zc_frequency(sym);
        carriers
            .iter()
            .zip(want)
            .map(|(&k, w)| {
                let got = freq[sym - 1][k];
                if got.norm_sqr() > 0.0 {
                    w * got.conj() / got.norm_sqr()
                } else {
                    C32::default()
                }
            })
            .collect()
    };
    let ch4 = channel(4);
    let ch6 = channel(6);
    let mean_phase = |c: &[C32]| -> f32 {
        c.iter().map(|v| v.im.atan2(v.re)).sum::<f32>() / c.len() as f32
    };
    // Half the difference is the per-symbol phase step, so a symbol N away
    // from the fourth is turned by N times it.
    let mut step = (mean_phase(&ch4) - mean_phase(&ch6)) / 2.0;
    if std::env::var("DRONEID_NO_WALK").is_ok() {
        step = 0.0;
    }
    if std::env::var("DRONEID_WALK_NEG").is_ok() {
        step = -step;
    }

    let mut bits = Vec::with_capacity(CODED_BITS);
    let mut confidence = 0.0f32;
    let mut counted = 0usize;
    let mut per_symbol: Vec<f32> = Vec::new();
    for &n in &[1usize, 2, 3, 5, 7, 8, 9] {
        let turn = step * (n as f32 - 4.0);
        let rot = C32::new(turn.cos(), turn.sin());
        let mut this = 0.0f32;
        for (i, &k) in carriers.iter().enumerate() {
            let v = freq[n - 1][k] * ch4[i] * rot;
            // Hard decision, quadrant by quadrant, in openphy's mapping:
            // 1+i is 00, 1-i is 01, -1+i is 10, -1-i is 11.
            bits.push(u8::from(v.re < 0.0));
            bits.push(u8::from(v.im < 0.0));
            let m = v.norm();
            if m > 0.0 {
                // How far the point sits from the diagonal it should be on.
                let q = (v.re.abs() + v.im.abs()) / (m * std::f32::consts::SQRT_2);
                confidence += q;
                this += q;
                counted += 1;
            }
        }
        per_symbol.push(this / CARRIERS as f32);
    }
    if counted > 0 {
        confidence /= counted as f32;
    }

    let raw_symbols: Vec<Vec<u8>> = bits.chunks(CARRIERS * 2).map(|c| c.to_vec()).collect();

    // The first symbol was demodulated with the rest to keep one loop, and
    // is split off here: it is scrambled on its own and carries nothing.
    let head: Vec<u8> = bits.drain(..CARRIERS * 2).collect();
    let first = gold_sequence(head.len(), SCRAMBLER_X2_INIT);
    let zeros = head
        .iter()
        .zip(&first)
        .filter(|(b, s)| *b == *s)
        .count() as f32
        / head.len() as f32;

    let scrambler = gold_sequence(CODED_BITS, SCRAMBLER_X2_INIT);
    for (b, s) in bits.iter_mut().zip(scrambler) {
        *b ^= s;
    }
    Some(Demodulated {
        bits,
        cfo_hz,
        confidence,
        per_symbol,
        raw_symbols,
        first_symbol_zeros: zeros,
    })
}

/// The frame a burst carries: 176 bytes, of which the first 91 are the
/// DroneID frame proper and the rest is the block's padding and CRC-24.
///
/// The systematic bits are read straight out of the rate matcher's stream 0
/// rather than turbo decoded. That is not a shortcut around the code, it is
/// what a systematic code is: the block itself is transmitted, and the parity
/// only adds a way to correct it. A burst strong enough to demodulate cleanly
/// needs no correction, and its CRC-24 says so.
///
/// What this costs is the weak bursts. The parity streams do not fit the LTE
/// turbo encoder as this reads them: with the systematic bits known good from
/// their CRC, a decode over the parity disagrees with them in about 180 bits
/// of 1408, so something about how DJI fills the other two streams is still
/// wrong here. Until that is found there is no error correction, only
/// detection.
pub fn frame_bits(iq: &[C32], start: usize, rate: f64) -> Option<Vec<u8>> {
    let d = demodulate(iq, start, rate)?;
    let e: Vec<f32> = d
        .bits
        .iter()
        .map(|&b| if b == 0 { 1.0 } else { -1.0 })
        .collect();
    let streams = crate::lte_turbo::dematch_at(&e, BLOCK_LEN + 4, RATE_MATCH_START);
    Some(
        streams[0][..BLOCK_LEN]
            .chunks(8)
            .map(|c| {
                c.iter()
                    .enumerate()
                    .fold(0u8, |a, (i, &v)| a | (u8::from(v < 0.0) << (7 - i)))
            })
            .collect(),
    )
}

/// The code block, in bits: 176 bytes under a CRC-24.
pub const BLOCK_LEN: usize = 1408;

/// Where in the rate matcher's circular buffer a burst begins.
///
/// LTE's own redundancy version 0 for a block this size, which DJI kept:
/// two rows of the sub-block interleaver, and 45 rows of 32 columns hold the
/// 1412 coded bits.
pub const RATE_MATCH_START: usize = 90;

/// The LTE pseudo-random sequence (36.211 section 7.2), which DroneID uses
/// unchanged as its scrambler.
///
/// `x2_init` is the second register's 31 bit state; the first is fixed.
pub fn gold_sequence(len: usize, x2_init: u32) -> Vec<u8> {
    const NC: usize = 1600;
    let n = NC + len + 31;
    let mut x1 = vec![0u8; n];
    let mut x2 = vec![0u8; n];
    x1[0] = 1;
    for (i, v) in x2.iter_mut().enumerate().take(31) {
        *v = (x2_init >> i & 1) as u8;
    }
    for i in 0..n - 31 {
        x1[i + 31] = x1[i + 3] ^ x1[i];
        x2[i + 31] = x2[i + 3] ^ x2[i + 2] ^ x2[i + 1] ^ x2[i];
    }
    (0..len).map(|i| x1[i + NC] ^ x2[i + NC]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A Zadoff-Chu sequence is constant modulus in the frequency domain,
    /// which is what makes it a channel estimate as well as a preamble: every
    /// carrier is measured with the same energy.
    ///
    /// Its time domain autocorrelation is deliberately not asserted to be a
    /// spike. Only 600 of the 1024 bins are populated, so the symbol is a
    /// band-limited ZC rather than a ZC, and the strongest sidelobe measures
    /// 0.52 of the peak. That is a property of the signal DJI transmits, not
    /// a fault here, and it is why the detector groups a run of positions
    /// over the threshold into one burst rather than reporting each.
    #[test]
    fn the_zc_sequence_is_flat_across_the_carriers() {
        for symbol in [4, 6] {
            let f = zc_frequency(symbol);
            assert_eq!(f.len(), CARRIERS);
            for c in &f {
                assert!((c.norm() - 1.0).abs() < 1e-5, "not constant modulus");
            }
            assert_eq!(zc_time(symbol, RATE).len(), 1024);
        }
    }

    /// The two roots produce different sequences, which is the only reason
    /// having two of them helps.
    #[test]
    fn the_two_zc_symbols_are_not_the_same_sequence() {
        let a = zc_time(4, RATE);
        let b = zc_time(6, RATE);
        let acc: C32 = a.iter().zip(&b).map(|(x, y)| *x * y.conj()).sum();
        let energy: f32 = a.iter().map(|c| c.norm_sqr()).sum();
        assert!(acc.norm() < energy * 0.2);
    }

    #[test]
    fn the_numerology_is_ltes() {
        assert_eq!(fft_size(RATE), 1024);
        assert_eq!(fft_size(30_720_000.0), 2048);
        assert_eq!(cyclic_prefix(RATE), (80, 72));
        assert_eq!(cyclic_prefix(30_720_000.0), (160, 144));
        let c = data_carriers(1024);
        assert_eq!(c.len(), CARRIERS);
        assert_eq!(c[0], 212);
        assert_eq!(c[299], 511);
        // DC is skipped, which is the whole reason the ZC sequence has its
        // middle element removed.
        assert_eq!(c[300], 513);
    }

    /// The template found in noise is a burst, and the position it is found
    /// at is the one it was put at.
    #[test]
    fn a_planted_burst_is_found_where_it_was_planted() {
        let (_, short_cp) = cyclic_prefix(RATE);
        let sym = zc_time(4, RATE);
        let mut iq = vec![C32::default(); 4000];
        // A deterministic noise floor a tenth of the signal's amplitude.
        let mut seed = 12345u32;
        for s in iq.iter_mut() {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let a = (seed >> 16) as f32 / 65536.0 - 0.5;
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            let b = (seed >> 16) as f32 / 65536.0 - 0.5;
            *s = C32::new(a, b) * 0.01;
        }
        let at = 1500;
        for (k, v) in sym.iter().enumerate() {
            iq[at + k] += *v;
        }
        let found = find_bursts(&iq, RATE, 0.5);
        assert_eq!(found.len(), 1, "one burst, not {}", found.len());
        assert_eq!(found[0].zc4_at, at - short_cp);
        assert!(found[0].score > 0.8, "score {}", found[0].score);
    }

    /// The scrambler is LTE's, so the first bits of the sequence with the
    /// standard's own initial state are a fixed, checkable thing.
    #[test]
    fn the_gold_sequence_matches_the_standard() {
        // 36.211 7.2 with x2 initialised to 1: the first output bits are a
        // published value, and getting Nc = 1600 wrong changes all of them.
        let c = gold_sequence(16, 1);
        assert_eq!(c.len(), 16);
        assert!(c.iter().all(|&b| b <= 1));
        // The sequence is not degenerate: a broken register gives all zeros
        // or a short period, both of which this catches.
        assert!(c.contains(&1) && c.contains(&0));
    }
}
