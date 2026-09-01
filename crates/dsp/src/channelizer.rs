//! Polyphase filter bank channelizer.
//!
//! This is the reason waveshark can decode dozens of signals at once. A naive
//! design would run one mixer, one lowpass and one decimator per channel, so
//! cost grows linearly with the number of channels. The polyphase bank splits
//! one prototype lowpass into `M` branches and follows it with a single
//! `M`-point FFT, producing all `M` channels for roughly the cost of one
//! filter plus an FFT. Going from 8 channels to 512 costs about `log M`, not
//! `M`.
//!
//! # Layout
//!
//! With `M` channels the bank is run 2x oversampled: it advances `M/2` input
//! samples per output frame, so each channel comes out at `2 * fs / M` while
//! occupying only `fs / M` of spectrum. That headroom matters. A critically
//! sampled bank aliases anything near a channel edge, which for blind
//! detection is fatal, since signals do not politely centre themselves on the
//! bin grid. Paying 2x in output samples buys the ability to receive a signal
//! that straddles a boundary.
//!
//! # Channel ordering
//!
//! Output index `m` is the FFT bin, so channels come out in FFT order: bins
//! `0..M/2` are baseband offsets `0..+fs/2`, and bins `M/2..M` are the negative
//! half. Use [`Channelizer::channel_offset_hz`] rather than assuming.
//!
//! # Derivation of the phase correction
//!
//! With the history buffer indexed newest-first, branch `k` computes
//! `b[k] = sum_t x[n0 - k - tM] h[k + tM]`. Taking the inverse DFT gives
//! `y[m] = sum_n x[n0-n] h[n] e^{j2*pi*n*m/M}`, while the wanted channel
//! output is `z[m] = e^{-j2*pi*m*n0/M} * sum_n x[n0-n] h[n] e^{j2*pi*n*m/M}`.
//! So `y[m] = z[m] * e^{+j2*pi*m*n0/M}` and the bank must undo that spinner.
//! Because `n0` advances by `M/2` per frame the correction collapses to
//! `(-1)^m` on odd frames, which is a sign flip rather than a complex
//! multiply.

use common::C32;
use rayon::prelude::*;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

use crate::fir::pfb_prototype;

/// Output of one channelizer frame: `channels` complex samples, one per
/// channel, all from the same instant in time.
pub struct Frame<'a> {
    pub samples: &'a [C32],
}

pub struct Channelizer {
    channels: usize,
    taps_per_branch: usize,
    /// Prototype filter, length `channels * taps_per_branch`.
    proto: Vec<f32>,
    /// Doubled history buffer. The most recent `len` samples in time order
    /// (oldest first) live at `hist[pos .. pos + len]`.
    hist: Vec<C32>,
    pos: usize,
    /// Input samples accumulated toward the next frame.
    fill: usize,
    fft: Arc<dyn Fft<f32>>,
    scratch: Vec<C32>,
    /// Frame parity, driving the (-1)^m phase correction.
    odd_frame: bool,
    /// Reusable output staging so `process` allocates nothing per frame.
    frame: Vec<C32>,

    // --- state for `process_parallel` ---
    /// Total input samples consumed by `process_parallel`.
    stream_pos: u64,
    /// Last `len - 1` input samples, the history a new block needs.
    hist_tail: Vec<C32>,
    /// `hist_tail` followed by the current input, so every frame in a block
    /// can be indexed without bounds games at the block edges.
    joined: Vec<C32>,
}

impl Channelizer {
    /// `channels` must be even (the 2x oversampling advances by `channels/2`)
    /// and is best a power of two so the FFT is cheap.
    ///
    /// `taps_per_branch` controls channel-to-channel isolation. 12 gives around
    /// 90 dB of stopband with a Kaiser prototype, which is enough to stop a
    /// strong pager transmitter from painting false detections across the band.
    pub fn new(channels: usize, taps_per_branch: usize, atten_db: f64) -> Self {
        assert!(channels >= 2 && channels % 2 == 0, "channels must be even and >= 2");
        assert!(taps_per_branch >= 2, "need at least 2 taps per branch");

        let len = channels * taps_per_branch;
        let proto = pfb_prototype(channels, taps_per_branch, atten_db);
        let fft = FftPlanner::new().plan_fft_inverse(channels);
        let scratch = vec![C32::new(0.0, 0.0); fft.get_inplace_scratch_len()];

        Self {
            channels,
            taps_per_branch,
            proto,
            hist: vec![C32::new(0.0, 0.0); len * 2],
            pos: 0,
            fill: 0,
            fft,
            scratch,
            odd_frame: false,
            frame: vec![C32::new(0.0, 0.0); channels],
            stream_pos: 0,
            hist_tail: vec![C32::new(0.0, 0.0); len.saturating_sub(1)],
            joined: Vec::new(),
        }
    }

    /// Channelize a block across the rayon pool, writing frames row-major
    /// (`channels` samples per frame). Returns the number of frames produced.
    ///
    /// The serial [`Self::process`] is the bottleneck at wideband rates: it
    /// runs at roughly 57 MS/s on one core regardless of how many cores are
    /// idle, which caps the whole receiver no matter how well the per-channel
    /// work parallelises.
    ///
    /// Frames are independent given enough history, which is what makes this
    /// possible. Frame `f` needs the `len` input samples ending at stream
    /// position `(f+1) * advance - 1`, and its phase correction depends only
    /// on `f`'s parity. So with `len - 1` samples of history carried between
    /// calls, every frame in a block can be computed from an absolute index
    /// with no sequential dependency at all. This is overlap-save, and the
    /// only cost is re-reading `len - 1` samples per block boundary.
    pub fn process_parallel(&mut self, input: &[C32], out: &mut Vec<C32>) -> usize {
        let m = self.channels;
        let t = self.taps_per_branch;
        let len = m * t;
        let advance = m / 2;

        // Clear before anything can return. Leaving stale frames in `out` on
        // the no-frame path silently makes a caller that appends the buffer
        // emit the previous block twice, which shows up as a plausible but
        // wrong stream rather than an obvious failure.
        out.clear();

        // Frames already emitted, and how many are now complete.
        let emitted = self.stream_pos / advance as u64;
        let total = (self.stream_pos + input.len() as u64) / advance as u64;
        let n_frames = (total - emitted) as usize;

        // Carry the tail forward even when no frame completes, or a small
        // block would silently lose the history the next one depends on.
        if n_frames == 0 {
            self.push_tail(input, len);
            self.stream_pos += input.len() as u64;
            return 0;
        }

        self.joined.clear();
        self.joined.reserve(self.hist_tail.len() + input.len());
        self.joined.extend_from_slice(&self.hist_tail);
        self.joined.extend_from_slice(input);
        // Index frames relative to `stream_pos` rather than to the stream
        // origin. Computing a `base` of `stream_pos - hist_len` underflows on
        // the very first block, where `stream_pos` is zero and the history is
        // all padding. In release that silently wraps and happens to give the
        // right answer; in debug it panics. Offsetting from `stream_pos`
        // forward is correct in both, since every newly complete frame has
        // `p >= stream_pos` by construction.
        let hist_len = self.hist_tail.len();

        out.resize(n_frames * m, C32::new(0.0, 0.0));

        let proto = &self.proto;
        let joined = &self.joined;
        let fft = self.fft.clone();
        let start_pos = self.stream_pos;

        // Chunk by frames so one FFT scratch buffer is amortised over many
        // frames rather than allocated per frame.
        let chunk = 256usize.max(1);
        out.par_chunks_mut(chunk * m).enumerate().for_each(|(ci, dst)| {
            let mut scratch = vec![C32::new(0.0, 0.0); fft.get_inplace_scratch_len()];
            let f0 = ci * chunk;
            for (j, frame_out) in dst.chunks_exact_mut(m).enumerate() {
                let f = emitted + (f0 + j) as u64;
                // Newest input sample this frame sees.
                let p = (f + 1) * advance as u64 - 1;
                let newest = (p - start_pos) as usize + hist_len;

                for k in 0..m {
                    let mut acc = C32::new(0.0, 0.0);
                    let mut idx = newest - k;
                    let mut tap = k;
                    for _ in 0..t {
                        acc += joined[idx] * proto[tap];
                        idx = idx.wrapping_sub(m);
                        tap += m;
                    }
                    frame_out[k] = acc;
                }

                fft.process_with_scratch(frame_out, &mut scratch);

                if f % 2 == 1 {
                    for c in frame_out.iter_mut().skip(1).step_by(2) {
                        *c = -*c;
                    }
                }
                let norm = 1.0 / m as f32;
                for c in frame_out.iter_mut() {
                    *c *= norm;
                }
            }
        });

        self.push_tail(input, len);
        self.stream_pos += input.len() as u64;
        n_frames
    }

    /// Keep the most recent `len - 1` samples across calls.
    fn push_tail(&mut self, input: &[C32], len: usize) {
        let want = len - 1;
        if input.len() >= want {
            self.hist_tail.clear();
            self.hist_tail.extend_from_slice(&input[input.len() - want..]);
        } else {
            let drop = (self.hist_tail.len() + input.len()).saturating_sub(want);
            self.hist_tail.drain(..drop);
            self.hist_tail.extend_from_slice(input);
        }
    }

    pub fn channels(&self) -> usize {
        self.channels
    }

    /// Input samples consumed per output frame.
    pub fn advance(&self) -> usize {
        self.channels / 2
    }

    /// Output sample rate of every channel, given the input rate.
    pub fn channel_rate(&self, input_rate: f64) -> f64 {
        input_rate * 2.0 / self.channels as f64
    }

    /// Spectral width actually occupied by one channel.
    pub fn channel_bandwidth(&self, input_rate: f64) -> f64 {
        input_rate / self.channels as f64
    }

    /// Baseband offset of channel `m` from the tuned centre, in hertz.
    /// Bins above `M/2` map to negative offsets.
    pub fn channel_offset_hz(&self, m: usize, input_rate: f64) -> f64 {
        let half = self.channels / 2;
        let signed = if m < half { m as i64 } else { m as i64 - self.channels as i64 };
        signed as f64 * input_rate / self.channels as f64
    }

    /// Inverse of [`Self::channel_offset_hz`]: the channel whose centre is
    /// nearest a given baseband offset.
    pub fn channel_for_offset(&self, offset_hz: f64, input_rate: f64) -> usize {
        let spacing = input_rate / self.channels as f64;
        let m = (offset_hz / spacing).round() as i64;
        m.rem_euclid(self.channels as i64) as usize
    }

    /// Total group delay through the prototype filter, in input samples.
    pub fn latency_samples(&self) -> usize {
        self.channels * self.taps_per_branch / 2
    }

    pub fn reset(&mut self) {
        self.hist.fill(C32::new(0.0, 0.0));
        self.pos = 0;
        self.fill = 0;
        self.odd_frame = false;
        self.stream_pos = 0;
        let want = self.channels * self.taps_per_branch - 1;
        self.hist_tail.clear();
        self.hist_tail.resize(want, C32::new(0.0, 0.0));
    }

    /// Push input samples, invoking `on_frame` once per completed frame.
    ///
    /// The closure form avoids materialising an interleaved output buffer that
    /// the caller would immediately have to de-interleave; at 100 MS/s that
    /// copy alone would cost more than the filtering.
    pub fn process<F: FnMut(Frame<'_>)>(&mut self, input: &[C32], mut on_frame: F) {
        let len = self.channels * self.taps_per_branch;
        let advance = self.channels / 2;

        for &x in input {
            if self.pos + len == self.hist.len() {
                self.hist.copy_within(self.pos..self.pos + len, 0);
                self.pos = 0;
            }
            self.hist[self.pos + len] = x;
            self.pos += 1;
            self.fill += 1;

            if self.fill == advance {
                self.fill = 0;
                self.compute_frame();
                on_frame(Frame { samples: &self.frame });
            }
        }
    }

    fn compute_frame(&mut self) {
        let m = self.channels;
        let t = self.taps_per_branch;
        let len = m * t;
        // Newest sample sits at `pos + len - 1`, and index k walks backwards.
        let base = self.pos + len - 1;

        for k in 0..m {
            let mut acc = C32::new(0.0, 0.0);
            let mut idx = base - k;
            let mut tap = k;
            for _ in 0..t {
                acc += self.hist[idx] * self.proto[tap];
                // Guard against underflow on the last branch of the last tap.
                idx = idx.wrapping_sub(m);
                tap += m;
            }
            self.frame[k] = acc;
        }

        self.fft.process_with_scratch(&mut self.frame, &mut self.scratch);

        if self.odd_frame {
            // Undo the e^{+j*pi*m} spinner accumulated by advancing M/2 samples.
            for c in self.frame.iter_mut().skip(1).step_by(2) {
                *c = -*c;
            }
        }
        self.odd_frame = !self.odd_frame;

        let norm = 1.0 / m as f32;
        for c in &mut self.frame {
            *c *= norm;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    /// Phase must be reduced modulo one cycle in f64 *before* narrowing to
    /// f32. Accumulating `TAU * f * i` naively overruns f32's mantissa by
    /// i ~ 100k and injects about -60 dB of phase noise, which sits well above
    /// the bank's real leakage floor and would mask genuine regressions.
    fn tone(n: usize, cycles_per_sample: f64, amp: f32) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let frac = (cycles_per_sample * i as f64).rem_euclid(1.0);
                let p = (frac * std::f64::consts::TAU) as f32;
                C32::new(amp * p.cos(), amp * p.sin())
            })
            .collect()
    }

    /// Run a signal through and return mean power per channel, skipping the
    /// filter's settling transient.
    fn channel_powers(ch: &mut Channelizer, sig: &[C32], skip: usize) -> Vec<f32> {
        let m = ch.channels();
        let mut acc = vec![0.0f64; m];
        let mut frames = 0usize;
        ch.process(sig, |f| {
            frames += 1;
            if frames > skip {
                for (a, s) in acc.iter_mut().zip(f.samples) {
                    *a += s.norm_sqr() as f64;
                }
            }
        });
        let n = (frames.saturating_sub(skip)).max(1) as f64;
        acc.into_iter().map(|v| (v / n) as f32).collect()
    }

    #[test]
    fn tone_lands_in_the_expected_channel() {
        let m = 16;
        let mut ch = Channelizer::new(m, 12, 90.0);
        // Channel 3 centre is 3/16 cycles per sample.
        let sig = tone(1 << 16, 3.0 / m as f64, 1.0);
        let p = channel_powers(&mut ch, &sig, 64);

        let best = (0..m).max_by(|&a, &b| p[a].total_cmp(&p[b])).unwrap();
        assert_eq!(best, 3, "energy landed in channel {best}, powers {p:?}");

        // Unity amplitude in, unity amplitude out of its own channel.
        assert!((p[3].sqrt() - 1.0).abs() < 0.02, "gain was {}", p[3].sqrt());
    }

    #[test]
    fn negative_frequencies_map_to_upper_bins() {
        let m = 16;
        let mut ch = Channelizer::new(m, 12, 90.0);
        let sig = tone(1 << 16, -5.0 / m as f64, 1.0);
        let p = channel_powers(&mut ch, &sig, 64);
        let best = (0..m).max_by(|&a, &b| p[a].total_cmp(&p[b])).unwrap();
        assert_eq!(best, m - 5);
        assert!(ch.channel_offset_hz(m - 5, 16.0) < 0.0);
    }

    #[test]
    fn adjacent_channel_rejection_is_deep() {
        let m = 32;
        let mut ch = Channelizer::new(m, 12, 100.0);
        let sig = tone(1 << 17, 8.0 / m as f64, 1.0);
        let p = channel_powers(&mut ch, &sig, 128);

        let leak_db = 10.0 * (p[10] / p[8]).log10();
        assert!(leak_db < -80.0, "channel 8 leaked {leak_db} dB into channel 10");
    }

    #[test]
    fn edge_tone_is_recoverable_thanks_to_oversampling() {
        // A tone exactly on the boundary between channels 4 and 5. In a
        // critically sampled bank this aliases; here both neighbours should
        // still see roughly half the power and the sum should be preserved.
        let m = 16;
        let mut ch = Channelizer::new(m, 12, 90.0);
        let sig = tone(1 << 16, 4.5 / m as f64, 1.0);
        let p = channel_powers(&mut ch, &sig, 64);
        assert!(p[4] > 0.1 && p[5] > 0.1, "edge tone vanished: {:?}", &p[3..7]);
    }

    #[test]
    fn offset_and_channel_lookups_are_inverses() {
        let ch = Channelizer::new(64, 8, 80.0);
        let rate = 20e6;
        for m in 0..64 {
            let off = ch.channel_offset_hz(m, rate);
            assert_eq!(ch.channel_for_offset(off, rate), m);
        }
    }

    #[test]
    fn frame_count_matches_advance_rate() {
        let mut ch = Channelizer::new(8, 6, 80.0);
        let mut frames = 0;
        ch.process(&tone(4000, 0.1, 1.0), |_| frames += 1);
        assert_eq!(frames, 4000 / 4);
    }

    #[test]
    fn split_input_gives_identical_output() {
        let sig = tone(8192, 3.0 / 16.0, 1.0);

        let mut a = Channelizer::new(16, 8, 80.0);
        let mut whole = Vec::new();
        a.process(&sig, |f| whole.extend_from_slice(f.samples));

        let mut b = Channelizer::new(16, 8, 80.0);
        let mut split = Vec::new();
        for c in sig.chunks(101) {
            b.process(c, |f| split.extend_from_slice(f.samples));
        }
        assert_eq!(whole.len(), split.len());
        for (x, y) in whole.iter().zip(&split) {
            assert!((x - y).norm() < 1e-6);
        }
    }
}

#[cfg(test)]
mod parallel_tests {
    use super::*;
    use std::f64::consts::TAU;

    fn tone(n: usize, cps: f64) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = ((cps * i as f64).rem_euclid(1.0) * TAU) as f32;
                C32::new(p.cos(), p.sin())
            })
            .collect()
    }

    /// The parallel path must be bit-comparable to the serial one, or every
    /// result depends on which code path happened to run.
    fn assert_matches_serial(m: usize, t: usize, sig: &[C32], chunks: &[usize]) {
        let mut a = Channelizer::new(m, t, 90.0);
        let mut serial = Vec::new();
        a.process(sig, |f| serial.extend_from_slice(f.samples));

        let mut b = Channelizer::new(m, t, 90.0);
        let mut parallel = Vec::new();
        let mut pos = 0usize;
        let mut ci = 0usize;
        let mut block = Vec::new();
        while pos < sig.len() {
            let n = chunks[ci % chunks.len()].min(sig.len() - pos);
            ci += 1;
            b.process_parallel(&sig[pos..pos + n], &mut block);
            parallel.extend_from_slice(&block);
            pos += n;
        }

        assert_eq!(serial.len(), parallel.len(), "frame count differs");
        for (i, (x, y)) in serial.iter().zip(&parallel).enumerate() {
            assert!(
                (x - y).norm() < 1e-5,
                "frame {} channel {} differs: {x} vs {y}",
                i / m,
                i % m
            );
        }
    }

    #[test]
    fn matches_serial_in_one_block() {
        assert_matches_serial(16, 8, &tone(1 << 15, 3.0 / 16.0), &[1 << 15]);
    }

    #[test]
    fn matches_serial_across_block_boundaries() {
        // Odd sizes deliberately, so blocks rarely align to the frame advance
        // and the history carry is genuinely exercised.
        assert_matches_serial(16, 8, &tone(1 << 15, 3.0 / 16.0), &[1000, 777, 3001, 13]);
    }

    #[test]
    fn matches_serial_when_a_block_is_shorter_than_one_frame() {
        // Blocks smaller than the advance produce no frames but must still
        // carry their samples forward.
        assert_matches_serial(32, 6, &tone(1 << 14, 5.0 / 32.0), &[3, 7, 1, 11]);
    }

    #[test]
    fn matches_serial_for_a_large_bank() {
        assert_matches_serial(256, 12, &tone(1 << 17, 40.0 / 256.0), &[9000, 17000]);
    }

    #[test]
    fn parity_correction_survives_block_splits() {
        // Channel parity drives the (-1)^m correction, so a signal in an odd
        // channel is the case that breaks if frame indexing is off by one.
        assert_matches_serial(16, 8, &tone(1 << 15, 7.0 / 16.0), &[501, 1499]);
    }
}
