//! Cutting several wide channels out of a wider span, by fast convolution.
//!
//! The obvious way to take a 20 MHz channel out of a 61.44 MS/s span is a
//! mixer, a lowpass and a decimator, and the obvious way to take eight of
//! them is eight of those. That costs the filter once per channel at the
//! input rate, which for channels this wide is billions of multiplies a
//! second and about fifty times slower than real time on a span like that.
//!
//! This does it in the frequency domain instead. One forward transform over
//! the span serves every channel; a channel is then the run of bins its band
//! occupies, tapered at the edges, run back through a smaller inverse
//! transform. Two things follow, and both matter:
//!
//! - The output rate is the ratio of the two transform lengths, so a rate
//!   conversion that has no integer decimator (61.44 to 20 MS/s is 3.072)
//!   is exact, and no resampler is needed. 3072 bins in, 1000 out.
//! - Adding a channel costs one small inverse transform, not another filter
//!   at the input rate.
//!
//! This is overlap-save: each block transforms `n` samples of which the
//! first half are the tail of the last block, and only the middle of what
//! comes back is kept, because the ends are the circular convolution wrapping
//! round. The filter is the taper, and its length in samples is set by how
//! many bins the taper spans, so a taper that is too sharp is a filter longer
//! than the overlap and its wrap lands in the output.
//!
//! [`crate::channelizer`] is the other tool for this and does not fit here:
//! it is a bank on a fixed grid, two-times oversampled, so a channel comes
//! out at twice its own width and holds only `fs / M` of usable spectrum.
//! For 20 MHz channels in a 61.44 MHz span that is 10.24 MHz of room for a
//! signal 16.6 MHz wide.

use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// One channel cut out of the span.
pub struct Slice {
    /// Where it is, in the same units the caller gave.
    pub center_hz: f64,
    /// Samples at the output rate, contiguous with the last block's.
    pub samples: Vec<C32>,
}

pub struct Slicer {
    n: usize,
    m: usize,
    /// New input samples per block, half of `n`.
    hop: usize,
    /// Output samples kept per block.
    keep: usize,
    fwd: Arc<dyn Fft<f32>>,
    inv: Arc<dyn Fft<f32>>,
    /// The last `n - hop` samples of the previous block, in front of the new
    /// ones.
    hist: Vec<C32>,
    /// Input samples not yet used, held until a whole hop has arrived.
    pending: Vec<C32>,
    /// Bin offset of each channel's centre from the span's, and its centre.
    channels: Vec<(i64, f64)>,
    /// The window over one channel's bins, which is the anti-alias filter.
    taper: Vec<f32>,
    /// Blocks processed, for the phase each channel's shift accumulates.
    block: u64,
    scratch: Vec<C32>,
}

impl Slicer {
    /// A slicer for `rate_in` samples a second centred on `center_hz`,
    /// producing `rate_out` for each channel in `channels`.
    ///
    /// `None` when the rates do not divide into whole transforms, or when a
    /// channel does not fit inside the span. `n` is the forward transform's
    /// length and decides both the bin spacing and the cost.
    pub fn new(
        rate_in: f64,
        center_hz: f64,
        rate_out: f64,
        channels: &[f64],
        n: usize,
    ) -> Option<Self> {
        let m_exact = n as f64 * rate_out / rate_in;
        let m = m_exact.round() as usize;
        if m < 8
            || (m_exact - m as f64).abs() > 1e-9
            || !n.is_multiple_of(2)
            || !m.is_multiple_of(2)
        {
            return None;
        }
        let hz_per_bin = rate_in / n as f64;
        let mut chans = Vec::new();
        for &c in channels {
            let offset = (c - center_hz) / hz_per_bin;
            let bin = offset.round() as i64;
            // A centre off the bin grid would move the channel by up to half
            // a bin, which no decoder is told about. The grid is fine enough
            // that this only rejects a genuinely odd tuning.
            if (offset - bin as f64).abs() > 1e-6 {
                return None;
            }
            // The channel's band has to be inside the span.
            if (c - center_hz).abs() + rate_out / 2.0 > rate_in / 2.0 + 1.0 {
                return None;
            }
            chans.push((bin, c));
        }
        let hop = n / 2;
        let keep = hop * m / n;
        if keep == 0 {
            return None;
        }
        let mut planner = FftPlanner::new();
        Some(Self {
            n,
            m,
            hop,
            keep,
            fwd: planner.plan_fft_forward(n),
            inv: planner.plan_fft_inverse(m),
            hist: vec![C32::default(); n - hop],
            pending: Vec::new(),
            channels: chans,
            taper: taper(m),
            block: 0,
            scratch: Vec::new(),
        })
    }

    /// Output samples a second, for the caller that wants to say it.
    pub fn output_len(&self, input_len: usize) -> usize {
        (input_len + self.pending.len()) / self.hop * self.keep
    }

    pub fn reset(&mut self) {
        self.hist.iter_mut().for_each(|x| *x = C32::default());
        self.pending.clear();
        self.block = 0;
    }

    /// Cut every channel out of `iq`, appending to each slice's samples.
    ///
    /// `out` is one entry per channel, in the order they were given, and is
    /// created on the first call.
    pub fn process(&mut self, iq: &[C32], out: &mut Vec<Slice>) {
        if out.len() != self.channels.len() {
            out.clear();
            out.extend(
                self.channels.iter().map(|&(_, hz)| Slice { center_hz: hz, samples: Vec::new() }),
            );
        }
        for s in out.iter_mut() {
            s.samples.clear();
        }
        self.pending.extend_from_slice(iq);

        let mut spectrum = vec![C32::default(); self.n];
        let mut band = vec![C32::default(); self.m];
        let scale = 1.0 / self.n as f32;
        let mut at = 0usize;
        while at + self.hop <= self.pending.len() {
            spectrum[..self.hist.len()].copy_from_slice(&self.hist);
            spectrum[self.hist.len()..].copy_from_slice(&self.pending[at..at + self.hop]);
            self.hist.copy_from_slice(&self.pending[at..at + self.hop]);
            at += self.hop;
            self.fwd.process(&mut spectrum);

            for (c, slice) in self.channels.iter().zip(out.iter_mut()) {
                let (bin, _) = *c;
                // The channel's own bins, written where the inverse
                // transform expects them: its centre at zero, not in the
                // middle of the array. Putting the centre in the middle
                // shifts the whole channel by half its rate, which comes out
                // as a signal 1 MHz above the centre arriving 9 MHz below it.
                let half = self.m as i64 / 2;
                for d in -half..half {
                    let src = (bin + d).rem_euclid(self.n as i64) as usize;
                    let dst = d.rem_euclid(self.m as i64) as usize;
                    band[dst] = spectrum[src] * (self.taper[(d + half) as usize] * scale);
                }
                // The bins were taken from an offset, which is a shift in
                // frequency, and a shift applied to a block that started at
                // sample `block * hop` carries that block's phase with it.
                let ph = -std::f32::consts::TAU
                    * (bin as f64 * (self.block * self.hop as u64) as f64 / self.n as f64)
                        .rem_euclid(1.0) as f32;
                let spin = C32::new(ph.cos(), ph.sin());
                self.inv.process(&mut band);
                // The middle is the part the circular convolution did not
                // wrap into.
                let edge = (self.m - self.keep) / 2;
                slice.samples.extend(band[edge..edge + self.keep].iter().map(|&x| x * spin));
            }
            self.block += 1;
        }
        self.scratch.clear();
        self.scratch.extend_from_slice(&self.pending[at..]);
        std::mem::swap(&mut self.pending, &mut self.scratch);
    }
}

/// The window over a channel's bins: flat across the band, rolled off at the
/// edges.
///
/// The roll-off is the whole filter. Twelve bins on each side of a thousand
/// is 1.2% of the channel, which for a 20 MHz channel is 240 kHz of skirt,
/// well outside the 16.6 MHz an 802.11 frame occupies, and short enough in
/// samples that its wrap stays inside the half block overlap-save throws
/// away.
fn taper(m: usize) -> Vec<f32> {
    let edge = (m / 80).max(4);
    (0..m)
        .map(|j| {
            let d = j.min(m - 1 - j);
            if d >= edge {
                1.0
            } else {
                let x = (d as f32 + 0.5) / edge as f32;
                (0.5 - 0.5 * (std::f32::consts::PI * x).cos()).sqrt()
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tone at a known offset comes out of the right slice at the right
    /// frequency, with its phase running on across block boundaries.
    #[test]
    fn a_tone_lands_in_its_own_channel_at_the_right_offset() {
        let (rate_in, rate_out) = (61_440_000.0, 20_000_000.0);
        let center = 2_457_000_000.0;
        let chans = [2_437_000_000.0, 2_462_000_000.0];
        let mut s = Slicer::new(rate_in, center, rate_out, &chans, 3072).expect("a slicer");

        // 1 MHz above channel 11's centre, which is 6 MHz above the span's.
        let tone_hz = 2_463_000_000.0;
        let iq: Vec<C32> = (0..61_440)
            .map(|n| {
                let ph = std::f32::consts::TAU * ((tone_hz - center) / rate_in) as f32 * n as f32;
                C32::new(ph.cos(), ph.sin())
            })
            .collect();
        let mut out = Vec::new();
        // Fed in uneven blocks, because a radio delivers what it delivers.
        for block in iq.chunks(7000) {
            let mut got = Vec::new();
            s.process(block, &mut got);
            if out.is_empty() {
                out = got;
            } else {
                for (a, b) in out.iter_mut().zip(got) {
                    a.samples.extend(b.samples);
                }
            }
        }
        assert_eq!(out.len(), 2);
        let ch11 = &out[1];
        assert!(ch11.samples.len() > 15_000, "{}", ch11.samples.len());

        // The tone is 1 MHz from this channel's centre and 26 MHz from the
        // other's, which is outside it.
        let peak = |v: &[C32]| -> (f64, f32) {
            let n = 4096.min(v.len() / 2 * 2);
            let mut buf: Vec<C32> = v[v.len() - n..].to_vec();
            FftPlanner::new().plan_fft_forward(n).process(&mut buf);
            let (i, p) = buf
                .iter()
                .enumerate()
                .map(|(i, x)| (i, x.norm()))
                .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();
            let k = if i > n / 2 { i as f64 - n as f64 } else { i as f64 };
            (k * rate_out / n as f64, p / n as f32)
        };
        let (hz, amp) = peak(&ch11.samples);
        assert!((hz - 1_000_000.0).abs() < 20_000.0, "{hz} Hz");
        assert!(amp > 0.4, "amplitude {amp}");
        let (_, other) = peak(&out[0].samples);
        assert!(other < 0.01, "leaked into channel 6 at {other}");
    }

    #[test]
    fn a_rate_that_is_not_a_whole_ratio_of_transforms_is_refused() {
        // 3072 bins in gives 1000 out at 20 MS/s, which is exact.
        assert!(Slicer::new(61_440_000.0, 2_457e6, 20e6, &[2_462e6], 3072).is_some());
        // 1024 would give 333.33 bins.
        assert!(Slicer::new(61_440_000.0, 2_457e6, 20e6, &[2_462e6], 1024).is_none());
        // A channel whose band runs off the edge of the span.
        assert!(Slicer::new(61_440_000.0, 2_457e6, 20e6, &[2_484e6], 3072).is_none());
    }
}
