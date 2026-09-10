//! Finding what is transmitting: the noise floor, the runs of bins over it,
//! and the tracks those runs become.

use super::{bin_of_hz, frame_start, hz_of_bin, Owned, Source, SourceConfig, SourceEvent};
use crate::window;
use common::{SourceId, C32};
use rayon::prelude::*;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;


/// The scalars of one frame's floor pass: what every bin does with that
/// frame, decided once from the shared counters.
#[derive(Clone, Copy, Debug)]
struct FrameStep {
    frame: u64,
    silent: bool,
    seed: bool,
    settled: bool,
    completing: bool,
    stored: usize,
    head: usize,
    bias: f32,
}

/// Bins per task of the floor pass, and per chunk of [`Rows`].
const BIN_CHUNK: usize = 256;

/// The floor pass's output for every frame of a block: one chunk of bins
/// per task, each holding every frame of its bins, so the tasks write
/// nothing in common.
#[derive(Default)]
struct Rows {
    chunks: Vec<RowChunk>,
}

#[derive(Default)]
struct RowChunk {
    /// Frame-major within the chunk: frame `f`, bin `i` of the chunk, at
    /// `f * BIN_CHUNK + i`.
    ratio: Vec<f32>,
    raw_ratio: Vec<f32>,
    power: Vec<f32>,
    floor: Vec<f32>,
}

/// One frame's bins, for the frame pass.
///
/// `ratio` is the smoothed power over the floor, zero before the floor is
/// known, and `raw_ratio` this frame's own power over it, unsmoothed.
/// Sensitivity comes from the smoothed power and timing from the raw: a
/// 45 dB signal takes thirty frames of smoothing to decay under the close
/// threshold after it stops, and a source that lingered that long would be
/// timed 8 ms late; the raw frame says at once that it is gone.
struct Bins<'a> {
    rows: &'a Rows,
    f: usize,
}

impl Bins<'_> {
    #[inline]
    fn at(&self, i: usize) -> (&RowChunk, usize) {
        (&self.rows.chunks[i / BIN_CHUNK], self.f * BIN_CHUNK + i % BIN_CHUNK)
    }
    #[inline]
    fn ratio(&self, i: usize) -> f32 {
        let (c, k) = self.at(i);
        c.ratio[k]
    }
    #[inline]
    fn raw_ratio(&self, i: usize) -> f32 {
        let (c, k) = self.at(i);
        c.raw_ratio[k]
    }
    #[inline]
    fn power(&self, i: usize) -> f32 {
        let (c, k) = self.at(i);
        c.power[k]
    }
    #[inline]
    fn floor(&self, i: usize) -> f32 {
        let (c, k) = self.at(i);
        c.floor[k]
    }
}

/// The noise floor of every bin at once, by minimum statistics on the
/// smoothed power.
///
/// A struct of arrays rather than a floor per bin: every bin's window
/// advances in step, so the counters are shared and the bias is one number
/// per frame, and the bins are updated across the pool in chunks. The
/// per-bin pass is what bounds the span the detector keeps up with, and a
/// deque and two logarithms per bin per frame put that at eight megahertz.
struct FloorBank {
    sub_len: usize,
    sub_count: usize,
    /// Frames into the sub-window being filled.
    filled: usize,
    /// Sub-windows stored so far, up to `sub_count`.
    stored: usize,
    /// Slot the next completed sub-window goes into.
    head: usize,
    /// Running minimum of the sub-window being filled, per bin.
    current: Vec<f32>,
    /// Minimum of each stored sub-window, `sub_count` per bin, bin-major.
    mins: Vec<f32>,
    /// Minimum over the stored sub-windows, per bin.
    min: Vec<f32>,
}

impl FloorBank {
    fn new(n: usize, sub_len: usize, sub_count: usize) -> Self {
        let sub_count = sub_count.max(1);
        Self {
            sub_len: sub_len.max(1),
            sub_count,
            filled: 0,
            stored: 0,
            head: 0,
            current: vec![f32::INFINITY; n],
            mins: vec![f32::INFINITY; n * sub_count],
            min: vec![f32::INFINITY; n],
        }
    }

    /// Whether the frame about to be fed completes a sub-window.
    fn completing(&self) -> bool {
        self.filled + 1 >= self.sub_len
    }

    /// Frames the minimum will have been taken over once this frame is in,
    /// which decides the bias correction.
    fn frames_after(&self) -> usize {
        if self.completing() {
            (self.stored + 1).min(self.sub_count) * self.sub_len
        } else {
            self.stored * self.sub_len + self.filled + 1
        }
    }

    /// Step the shared counters once every bin has been fed.
    fn advance(&mut self) {
        self.filled += 1;
        if self.filled >= self.sub_len {
            self.filled = 0;
            self.head = (self.head + 1) % self.sub_count;
            self.stored = (self.stored + 1).min(self.sub_count);
        }
    }

    fn reset(&mut self) {
        self.filled = 0;
        self.stored = 0;
        self.head = 0;
        self.current.fill(f32::INFINITY);
        self.mins.fill(f32::INFINITY);
        self.min.fill(f32::INFINITY);
    }
}

/// Feed one bin's smoothed power and return its raw minimum. `stored` is
/// the sub-window count after this frame.
#[inline]
fn floor_update(
    p: f32,
    current: &mut f32,
    mins: &mut [f32],
    min: &mut f32,
    completing: bool,
    head: usize,
    stored: usize,
) -> f32 {
    *current = current.min(p);
    if completing {
        mins[head] = *current;
        *current = f32::INFINITY;
        *min = mins[..stored].iter().copied().fold(f32::INFINITY, f32::min);
        *min
    } else if stored > 0 {
        min.min(*current)
    } else {
        // Nothing complete yet: the running minimum stands in, so
        // detection is merely insensitive for the first fraction of a
        // second rather than absent.
        *current
    }
}

/// Ratio by which the minimum of `k` frames of smoothed noise power sits
/// below the mean, inverted, so multiplying the minimum by it gives the mean.
///
/// Smoothing with coefficient `alpha` averages about `2/alpha - 1` frames,
/// which brings the relative spread of the power down to the reciprocal of
/// the square root of that. The minimum over `k` such frames, of which only
/// every `n_eff`th is independent, falls about `sqrt(2 ln k_eff)` spreads
/// below the mean. Measured on white noise this lands the corrected floor
/// within a decibel of the true noise power, where the fixed square-root
/// rule in [`crate::detect`] sits 6 to 9 dB above it.
pub(super) fn floor_bias(alpha: f32, k: usize) -> f32 {
    let n_eff = (2.0 / alpha - 1.0).max(1.0);
    let spread = 1.0 / n_eff.sqrt();
    let k_eff = (k as f32 / n_eff).max(1.0);
    let c = (2.0 * k_eff.ln()).max(0.0).sqrt();
    let ratio = (1.0 - c * spread).max(0.25);
    1.0 / ratio
}

/// A run of hot bins in one frame.
#[derive(Clone, Copy, Debug)]
struct Segment {
    /// Every bin over the close threshold, gaps bridged.
    lo: usize,
    hi: usize,
    /// The bins within `extent_db` of the run's peak. A strong signal is
    /// over the floor far beyond its own width: sharp keying puts sidebands
    /// across hundreds of kilohertz at 60 dB, and its onset splashes wider
    /// still for a frame. Those bins are its run, and they are not its
    /// width.
    occ_lo: usize,
    occ_hi: usize,
    peak_db: f32,
    /// Mean power over the floor across the run this frame, unsmoothed. A
    /// run whose raw power is under the close threshold is the smoother
    /// remembering a signal that has stopped, and does not count as seeing
    /// it. The mean rather than the peak, because the peak of forty bins of
    /// noise is the largest of forty exponential draws and clears 5 dB more
    /// often than not.
    raw_db: f32,
    /// Power-weighted centre of the occupied bins.
    centroid: f64,
}

/// A source being followed, or a candidate not yet old enough to be one.
#[derive(Clone, Debug)]
struct Track {
    src: Source,
    /// Bins the source's run covered in the last frame it was seen, which
    /// is what it is matched and its presence measured on.
    lo_bin: usize,
    hi_bin: usize,
    /// Bins holding its power in that frame, which is what its extent is.
    occ_lo: usize,
    occ_hi: usize,
    /// Consecutive frames matched, for the candidate stage.
    hits: usize,
    /// Weakest and strongest the run's peak has been, in dB over the floor,
    /// which is what says whether anything is being sent.
    peak_lo: f32,
    peak_hi: f32,
    /// Consecutive frames unmatched, for the hang.
    misses: usize,
    open: bool,
    /// Frame index the candidate was first seen in.
    born: u64,
    /// Frame index the source was last seen in.
    last_frame: u64,
    /// Width the source opened at, in hertz, which is what its extraction
    /// was designed for.
    opened_hz: f64,
    /// Centre the source opened at, in hertz. A transmitter that outgrew
    /// its extraction leaves that extent; two-level keying does not.
    opened_center_hz: f64,
    /// Power-weighted centre of the strongest run matching this frame.
    frame_centroid: f64,
    /// Centre of the occupied bins at each of the last [`GROWTH_FRAMES`]
    /// matched frames, as a ring. A sweep's centre moves across the band; a
    /// keyed carrier's stays put while its edges flicker as sidebands cross
    /// the threshold, and the flicker widened the lifetime extent enough to
    /// pass for growth when growth was what was measured.
    centres: [f64; GROWTH_FRAMES],
    seen_frames: usize,
    /// Sum of centroids and count, for the centre over the opening frames.
    centroid_sum: f64,
    centroid_n: u32,
    matched: bool,
    /// Whether the run that matched this frame is far under the source's
    /// own peak. It keeps the source present but does not widen it: the
    /// gap between a handheld's slots, where the smoothed tail of a 75 dB
    /// burst bridged to the tuner's centre spur as a run under 30 dB, would
    /// otherwise have made a 12 kHz source 120 kHz wide.
    faint: bool,
}

/// Watches a wideband stream as a spectrogram and reports sources.
pub struct SourceDetector {
    cfg: SourceConfig,
    /// Channels where a front end is already listening and no source may
    /// open.
    ///
    /// Filtering the events afterwards was not enough: the tracks still
    /// existed, so the spectrum drew a dozen detections inside the channel
    /// the BLE front end owns, and the receiver looked like it was ignoring
    /// its own decision. A channel somebody is reading is not a place to go
    /// looking for something to read.
    locked: Vec<Owned>,
    /// The tuner's own centre, as offsets from the stream centre. See
    /// [`SourceDetector::set_spur`].
    spur: Option<(f64, f64)>,
    /// Candidates refused because [`SourceConfig::max_open`] was reached,
    /// counted so a receiver can say it is dropping signal rather than
    /// silently reading less of the band.
    capped: u64,
    rate: f64,
    n: usize,
    hop: usize,
    fft: Arc<dyn Fft<f32>>,
    win: Vec<f32>,
    /// Raw power per bin for every frame of the block, and whether each
    /// frame was silent, filled across the pool before the frame pass.
    spectra: Vec<f32>,
    silent: Vec<bool>,
    /// Whether each frame of the block had samples at the converter's rails.
    saturated: Vec<bool>,
    /// Whether the frame being tracked did.
    ///
    /// A converter driven past full scale makes its own spectrum: the floor
    /// comes up ten decibels or more, and products of the signal stand
    /// across the span within twenty decibels of it, so the run is the
    /// whole band and its extent hundreds of kilohertz. A handheld keyed
    /// beside a HackRF put a quarter of the samples on the rails and read
    /// as a 400 kHz source, which no channel front end would take, while a
    /// channel placed by hand on the same frequency decoded it through:
    /// the signal itself is still there, and still the strongest thing by
    /// twenty decibels. So in such a frame the strongest run is the only
    /// one believed, and its extent is the bins within a few decibels of
    /// its peak, which is the signal's own lobe and not the receiver's
    /// products of it. A second transmitter on the air at the same moment
    /// is lost for as long as the saturation lasts, which is the receiver's
    /// state and not the detector's to fix.
    frame_saturated: bool,
    /// Samples carried between calls, so a frame can span input blocks.
    pending: Vec<C32>,
    /// Wideband index of `pending[0]`.
    consumed: u64,
    alpha: f32,
    /// Smoothed power per bin, in display order (lowest frequency first).
    power: Vec<f32>,
    /// Every frame of the block's ratios, powers and floors, frame-major,
    /// written by the floor pass and read by the frame pass.
    rows: Rows,
    floor: FloorBank,
    /// Ceiling on each bin's floor, from the floor of the bins around it, or
    /// infinity where there is not enough history to measure one. Applied to
    /// the minimum before the bias, so a bin whose own minimum is the signal
    /// standing in it is floored at what its neighbours read instead.
    cap: Vec<f32>,
    /// Bins each cap is measured over, and the frame the caps were last
    /// measured in. Measured once a sub-window, because it answers where the
    /// noise is and not what is transmitting.
    cap_bins: usize,
    cap_at: u64,
    /// The frame the caps were first measured in, which is when a fixture
    /// of the receiver first shows; zero until then.
    cap_first: u64,
    /// Scratch for the median, kept so a chunk is not allocated per frame.
    cap_scratch: Vec<f32>,
    /// Bins the cap leaves alone, where minimum statistics rule as they
    /// always did. The tuner's residual DC is a permanent hump the cap
    /// would otherwise unhide and report forever; learned as floor it costs
    /// nothing, and a real device on the same frequency still opens because
    /// its silences let the minimum fall to the noise.
    cap_skip: Option<(usize, usize)>,
    /// Smoothed power over the floor in the last frame, as a ratio; zero
    /// before the floor is known. Kept linear: a logarithm per bin per frame
    /// is what the span the detector keeps up with was being spent on.
    ratio: Vec<f32>,
    /// Bins the stream's declared bandwidth reaches. Outside it is filter
    /// roll-off, which is not a signal.
    bin_lo: usize,
    bin_hi: usize,
    frame: u64,
    /// Frame the smoother last restarted from, after silence.
    settle_at: u64,
    /// Whether every frame so far was silent, so the first real one seeds.
    silent_so_far: bool,
    hang_frames: usize,
    next_id: u64,
    tracks: Vec<Track>,
    segs: Vec<Segment>,
    events: Vec<SourceEvent>,
}

impl SourceDetector {
    /// `bandwidth` is the width of the stream that is signal rather than
    /// roll-off; pass the rate when it is all usable.
    pub fn new(rate: f64, bandwidth: f64, cfg: SourceConfig) -> Self {
        assert!(rate > 0.0, "source detector needs a positive sample rate");
        assert!(cfg.close_db < cfg.open_db, "close_db must be below open_db for hysteresis");
        let n = cfg.fft_size_at(rate);
        let hop = n / 2;
        let fft = FftPlanner::new().plan_fft_forward(n);
        let frames_per_s = rate / hop as f64;
        let memory = (cfg.floor_memory_s * frames_per_s).max(8.0) as usize;
        let sub_count = 32usize;
        let sub_len = memory.div_ceil(sub_count).max(1);
        let alpha = 1.0 / cfg.integrate_frames.max(1) as f32;
        let hang_frames = ((cfg.hang_us as f64 * 1e-6 * frames_per_s).ceil() as usize).max(1);

        let bw = if bandwidth > 0.0 { bandwidth.min(rate) } else { rate };
        let half_bins = (bw / 2.0 / (rate / n as f64)).floor() as usize;
        let bin_lo = (n / 2).saturating_sub(half_bins);
        let bin_hi = (n / 2 + half_bins).min(n - 1);

        Self {
            cfg,
            locked: Vec::new(),
            spur: None,
            capped: 0,
            rate,
            n,
            hop,
            fft,
            // Blackman-Harris rather than Hann: a strong carrier through
            // Hann's -31 dB sidelobes reads tens of kilohertz wide, and the
            // extent is what the extraction is designed from.
            win: window::blackman_harris(n),
            spectra: Vec::new(),
            silent: Vec::new(),
            saturated: Vec::new(),
            frame_saturated: false,
            pending: Vec::new(),
            consumed: 0,
            alpha,
            power: vec![0.0; n],
            floor: FloorBank::new(n, sub_len, sub_count),
            cap: vec![f32::INFINITY; n],
            // At least 64 bins to take a median over, and never more than
            // there are: a narrow stream's detector has fewer, and a clamp
            // with its bounds crossed is a panic rather than a floor.
            cap_bins: if cfg.floor_chunk_bins > 0 {
                cfg.floor_chunk_bins.min(n)
            } else {
                (n / 8).max(64).min(n)
            },
            cap_at: 0,
            cap_first: 0,
            cap_scratch: Vec::new(),
            cap_skip: None,
            rows: Rows::default(),
            ratio: vec![0.0; n],
            bin_lo,
            bin_hi,
            frame: 0,
            settle_at: 0,
            silent_so_far: true,
            hang_frames,
            next_id: 1,
            tracks: Vec::new(),
            segs: Vec::new(),
            events: Vec::new(),
        }
    }

    pub fn fft_size(&self) -> usize {
        self.n
    }

    /// Limit detection to a band, as offsets from the stream centre.
    ///
    /// A band is cut from a span by a decimation that is a power of two, so
    /// what arrives is up to twice the width asked for. Sources outside the
    /// wanted band are real transmitters, and without this the receiver
    /// reports sensors from outside the band a scanner block declared, and
    /// spends the work to read them.
    pub fn set_band(&mut self, lo_hz: f64, hi_hz: f64) {
        let lo = self.bin_of(lo_hz).floor().max(0.0) as usize;
        let hi = self.bin_of(hi_hz).ceil().max(1.0) as usize - 1;
        self.bin_lo = self.bin_lo.max(lo);
        self.bin_hi = self.bin_hi.min(hi.min(self.n - 1));
        if self.bin_hi < self.bin_lo {
            self.bin_hi = self.bin_lo;
        }
    }

    /// Where the tuner's own centre is, as offsets from the stream centre.
    ///
    /// A direct-conversion receiver's DC offset is not steady: a strong
    /// signal anywhere in the span modulates it with its own envelope, and a
    /// DC block passes that as readily as any other keying. Two things follow
    /// from that and both are here because both are about this band. The
    /// floor cap is left off, since the residual DC is a permanent hump the
    /// cap would otherwise unhide. And a source opening there is refused
    /// while anything else is transmitting, since that is when the offset
    /// moves; a device that really sits on the centre still opens when it
    /// transmits by itself.
    pub fn set_spur(&mut self, lo_hz: f64, hi_hz: f64) {
        let lo = self.bin_of(lo_hz).floor().max(0.0) as usize;
        let hi = (self.bin_of(hi_hz).ceil() as usize).min(self.n - 1);
        self.cap_skip = (lo <= hi).then_some((lo, hi));
        self.spur = Some((lo_hz, hi_hz));
    }

    /// Which bin an offset from the stream centre falls in, unrounded.
    fn bin_of(&self, hz: f64) -> f64 {
        bin_of_hz(hz, self.n, self.bin_hz())
    }

    pub fn hop(&self) -> usize {
        self.hop
    }

    pub fn bin_hz(&self) -> f64 {
        self.rate / self.n as f64
    }

    /// Frames per second, which is the time resolution of detection.
    pub fn frame_rate(&self) -> f64 {
        self.rate / self.hop as f64
    }

    /// Wideband samples between a source starting and its opening being
    /// reported, at most. What a ring in front of the extractor has to hold.
    pub fn latency_samples(&self) -> usize {
        (self.cfg.min_frames + 1) * self.hop + self.n
    }

    /// Whether the detector is still measuring the floor, and so cannot yet
    /// say that nothing is transmitting.
    ///
    /// A stream is not looked at for its first [`SETTLE_FRAMES`] frames,
    /// which at 20 MS/s is over a tenth of a second: a consumer that reads
    /// only while a source is open is deaf for all of it unless it knows to
    /// keep reading until the detector can answer. A stream silent from its
    /// first sample never settles, because there is nothing to measure a
    /// floor against, and it is also the one case where nothing being open
    /// is certainly right.
    pub fn settling(&self) -> bool {
        !self.silent_so_far && self.frame < self.settle_at + SETTLE_FRAMES
    }

    /// Channels a front end is already reading, as offsets from the centre.
    /// Nothing is opened inside one; see [`Owned`].
    pub fn set_owned(&mut self, channels: Vec<Owned>) {
        self.locked = channels;
    }

    /// Sources currently open, in the order they opened.
    pub fn live(&self) -> impl Iterator<Item = &Source> {
        self.tracks.iter().filter(|t| t.open).map(|t| &t.src)
    }

    /// Smoothed SNR per bin from the last frame in dB, lowest frequency
    /// first; minus infinity where the floor is not known yet.
    pub fn snr_db(&self) -> Vec<f32> {
        self.ratio
            .iter()
            .map(|r| if *r > 0.0 { 10.0 * r.log10() } else { f32::NEG_INFINITY })
            .collect()
    }

    /// Wideband samples consumed into complete frames so far.
    pub fn position(&self) -> u64 {
        self.consumed
    }

    pub fn reset(&mut self) {
        self.pending.clear();
        self.consumed = 0;
        self.power.fill(0.0);
        self.floor.reset();
        self.cap.fill(f32::INFINITY);
        self.cap_at = 0;
        self.cap_first = 0;
        self.ratio.fill(0.0);
        self.frame = 0;
        self.settle_at = 0;
        self.silent_so_far = true;
        self.tracks.clear();
        self.events.clear();
    }

    /// Let a block go by without looking at it.
    ///
    /// For a consumer that knows there is nothing to find: an analogue camera
    /// holding the whole span, where every run inside the carrier is a piece
    /// of the picture. Nothing is buffered, so the detector starts again from
    /// the next block it is given rather than reading a stale one.
    pub fn idle(&mut self, _samples: usize) {
        self.pending.clear();
        self.events.clear();
    }

    /// Consume a block and return the sources that opened or closed in it.
    pub fn process(&mut self, input: &[C32]) -> &[SourceEvent] {
        self.events.clear();
        self.pending.extend_from_slice(input);
        let n = self.n;
        let hop = self.hop;
        let count = if self.pending.len() >= n { (self.pending.len() - n) / hop + 1 } else { 0 };

        // The transforms first, across the pool: every frame's spectrum is
        // independent of every other's. The floor, the runs and the tracks
        // are then a pass in frame order.
        self.spectra.resize(count * n, 0.0);
        self.silent.resize(count, false);
        self.saturated.resize(count, false);
        let pending = &self.pending;
        let win = &self.win;
        let fft = &self.fft;
        let half = n / 2;
        self.spectra
            .par_chunks_mut(n)
            .zip(self.silent.par_iter_mut())
            .zip(self.saturated.par_iter_mut())
            .enumerate()
            .for_each_init(
                || (vec![C32::default(); n], vec![C32::default(); fft.get_inplace_scratch_len()]),
                |(buf, scratch), (f, ((spec, silent), saturated))| {
                    let frame = &pending[f * hop..f * hop + n];
                    // At the rail, not past it: a converter stops at full
                    // scale, and a value beyond it is a synthesised stream
                    // that was never clipped at all.
                    let at_rail = |v: f32| (RAIL..=RAIL_TOP).contains(&v.abs());
                    let rails = frame.iter().filter(|c| at_rail(c.re) || at_rail(c.im)).count();
                    // One sample in ten on the rail. A pair of tones at
                    // half scale each touches it one time in fifty and is
                    // not clipping; a handheld beside a HackRF put one in
                    // five there.
                    *saturated = rails * 10 > n;
                    // Silence is measured after the mean is removed, because
                    // a tuner settling does not always deliver zeros:
                    // rtl_433's captures open with a quarter second of byte
                    // value zero, which is a full-scale constant. That is a
                    // carrier at DC and nothing anywhere else, and it
                    // empties every other bin's floor just as zeros would.
                    let n_f = n as f32;
                    let mean = frame.iter().sum::<C32>() / n_f;
                    let ac: f32 = frame.iter().map(|c| (c - mean).norm_sqr()).sum();
                    *silent = ac < 1e-12 || ac < mean.norm_sqr() * n_f * 1e-9;
                    for i in 0..n {
                        buf[i] = frame[i] * win[i];
                    }
                    fft.process_with_scratch(buf, scratch);
                    for i in 0..n {
                        spec[i] = buf[(i + half) % n].norm_sqr();
                    }
                },
            );

        let spectra = std::mem::take(&mut self.spectra);
        let silent = std::mem::take(&mut self.silent);
        let saturated = std::mem::take(&mut self.saturated);

        // The floor pass runs bin-major over a run of frames: every bin
        // steps through the frames on its own, and the bins are shared out
        // across the pool once per run rather than once per frame. Frame by
        // frame it was 3900 fork-joins a second at 16 MS/s over a few
        // microseconds of arithmetic each, and serial it was a fifth of real
        // time at 20 MS/s on one thread. A run ends where the caps are due,
        // since they are read across every bin's floor as it stood then.
        let mut steps: Vec<FrameStep> = Vec::with_capacity(count);
        let mut run_start = 0usize;
        for f in 0..count {
            let (step, caps_due) = self.plan_frame(silent[f]);
            if caps_due {
                if f > run_start {
                    self.floor_run(&spectra, run_start, &steps[run_start..f]);
                }
                self.measure_caps_at(step.frame);
                run_start = f;
            }
            steps.push(step);
        }
        if count > run_start {
            self.floor_run(&spectra, run_start, &steps[run_start..count]);
        }

        let rows = std::mem::take(&mut self.rows);
        // The frames either side too: the converter clips a symbol
        // or two after the signal's edge lit the band, and it is that
        // edge frame's splash, read with the ordinary margin, that
        // opened a 12 kHz signal 70 kHz wide.
        let sat: Vec<bool> = (0..count)
            .map(|f| {
                let lo = f.saturating_sub(SATURATION_SMEAR);
                let hi = (f + SATURATION_SMEAR).min(count - 1);
                saturated[lo..=hi].iter().any(|s| *s)
            })
            .collect();
        // Every frame's runs at once, since a frame's segmentation reads its
        // own bins and nothing else: the tracking that follows is a state
        // machine in frame order and has to be serial, but this was two
        // thirds of that serial pass, and at 20 MS/s the pass was most of
        // what detection cost.
        let mut per_frame: Vec<Vec<Segment>> = (0..count)
            .into_par_iter()
            .map(|f| self.segment(&Bins { rows: &rows, f }, sat[f]))
            .collect();
        for (f, step) in steps.iter().enumerate() {
            self.frame_saturated = sat[f];
            self.frame = step.frame;
            let bins = Bins { rows: &rows, f };
            self.segs = std::mem::take(&mut per_frame[f]);
            self.track(&bins);
        }
        if let Some(last) = steps.last() {
            self.frame = last.frame + 1;
            let bins = Bins { rows: &rows, f: count - 1 };
            for i in 0..n {
                self.ratio[i] = bins.ratio(i);
            }
        }
        self.rows = rows;
        self.spectra = spectra;
        self.silent = silent;
        self.saturated = saturated;

        let pos = count * hop;
        self.pending.drain(..pos);
        self.consumed += pos as u64;
        self.refuse_claimed();
        &self.events
    }

    /// The scalars of one frame's floor pass, stepping the shared counters
    /// past it, and whether the caps are due before it.
    fn plan_frame(&mut self, silent: bool) -> (FrameStep, bool) {
        // The smoother is seeded from the first frame and the floor waits
        // for it to settle. Starting the smoother from zero puts a run of
        // near-zero frames into every bin's minimum, and for the whole of
        // the floor's memory afterwards every bin reads tens of dB hot.
        self.silent_so_far = self.silent_so_far && silent;
        if silent {
            self.settle_at = self.frame + 1;
        }
        // Settling: the floor is not measured until the stream has run for
        // a while. A filter upstream fades in over its first few hundred
        // samples, and a floor taken while the smoother was still catching
        // up from that fade is a floor learned low, which every later frame
        // then clears.
        let settled = self.frame >= self.settle_at + SETTLE_FRAMES;
        let seed = self.frame == self.settle_at;
        let measure = settled && !silent;
        let completing = measure && self.floor.completing();
        let stored = if completing {
            (self.floor.stored + 1).min(self.floor.sub_count)
        } else {
            self.floor.stored
        };
        let step = FrameStep {
            frame: self.frame,
            silent,
            seed,
            settled,
            completing,
            stored,
            head: self.floor.head,
            bias: if measure { floor_bias(self.alpha, self.floor.frames_after()) } else { 1.0 },
        };
        let caps_due = measure && self.caps_due(self.frame);
        if measure {
            self.floor.advance();
        }
        self.frame += 1;
        (step, caps_due)
    }

    /// The floor pass over frames `steps`, whose spectra start at row
    /// `first` of `spectra`, leaving each frame's ratios in `rows`.
    fn floor_run(&mut self, spectra: &[f32], first: usize, steps: &[FrameStep]) {
        let n = self.n;
        let count = first + steps.len();
        let chunks = n.div_ceil(BIN_CHUNK);
        self.rows.chunks.resize_with(chunks, RowChunk::default);
        for c in &mut self.rows.chunks {
            for v in [&mut c.ratio, &mut c.raw_ratio, &mut c.power, &mut c.floor] {
                if v.len() < count * BIN_CHUNK {
                    v.resize(count * BIN_CHUNK, 0.0);
                }
            }
        }
        let alpha = self.alpha;
        let sc = self.floor.sub_count;
        self.power
            .par_chunks_mut(BIN_CHUNK)
            .zip(self.floor.current.par_chunks_mut(BIN_CHUNK))
            .zip(self.floor.min.par_chunks_mut(BIN_CHUNK))
            .zip(self.floor.mins.par_chunks_mut(BIN_CHUNK * sc))
            .zip(self.cap.par_chunks(BIN_CHUNK))
            .zip(self.rows.chunks.par_iter_mut())
            .enumerate()
            .for_each(|(ci, (((((power, current), min), mins), cap), rows))| {
                let b0 = ci * BIN_CHUNK;
                let w = power.len();
                for (k, step) in steps.iter().enumerate() {
                    let f = first + k;
                    let raw = &spectra[f * n + b0..f * n + b0 + w];
                    let at = f * BIN_CHUNK;
                    let ratio = &mut rows.ratio[at..at + w];
                    let raw_ratio = &mut rows.raw_ratio[at..at + w];
                    let out_power = &mut rows.power[at..at + w];
                    let floor = &mut rows.floor[at..at + w];
                    for i in 0..w {
                        let p = raw[i];
                        if step.silent {
                            ratio[i] = 0.0;
                            raw_ratio[i] = 0.0;
                            out_power[i] = power[i];
                            floor[i] = 0.0;
                            continue;
                        }
                        if step.seed {
                            power[i] = p;
                        } else {
                            power[i] += alpha * (p - power[i]);
                        }
                        out_power[i] = power[i];
                        if !step.settled {
                            ratio[i] = 0.0;
                            raw_ratio[i] = 0.0;
                            floor[i] = 0.0;
                            continue;
                        }
                        let m = floor_update(
                            power[i],
                            &mut current[i],
                            &mut mins[i * sc..(i + 1) * sc],
                            &mut min[i],
                            step.completing,
                            step.head,
                            step.stored,
                        );
                        let fl = m.min(cap[i]) * step.bias;
                        if fl > 0.0 && fl.is_finite() {
                            floor[i] = fl;
                            ratio[i] = power[i] / fl;
                            raw_ratio[i] = p / fl;
                        } else {
                            floor[i] = 0.0;
                            ratio[i] = 0.0;
                            raw_ratio[i] = 0.0;
                        }
                    }
                }
            });
    }

    /// Set each bin's ceiling from the median floor of the chunk it is in.
    ///
    /// The median is taken over the minimum statistics rather than over this
    /// frame's power, so a burst passing through a chunk cannot lift the
    /// ceiling for the bins beside it, and the number it produces is the one
    /// the rest of the floor is already expressed in.
    fn caps_due(&self, frame: u64) -> bool {
        let refresh = self.floor.sub_len as u64;
        if self.cap_at != 0 && frame < self.cap_at + refresh {
            return false;
        }
        // Nothing complete yet: the running minimum is still falling towards
        // the noise, and a ceiling from it would be measured on a floor that
        // is about to move.
        self.floor.stored > 0
    }

    fn measure_caps_at(&mut self, frame: u64) {
        self.cap_at = frame.max(1);
        if self.cap_first == 0 {
            self.cap_first = self.cap_at;
        }
        let ratio = 10f32.powf(self.cfg.floor_cap_db / 10.0);
        let width = self.cap_bins.max(1);
        let mut lo = self.bin_lo;
        while lo <= self.bin_hi {
            let hi = (lo + width - 1).min(self.bin_hi);
            self.cap_scratch.clear();
            self.cap_scratch.extend(self.floor.min[lo..=hi].iter().copied().filter(|m| m.is_finite()));
            let cap = if self.cap_scratch.is_empty() {
                f32::INFINITY
            } else {
                let k = self.cap_scratch.len() / 2;
                let (_, med, _) = self.cap_scratch.select_nth_unstable_by(k, f32::total_cmp);
                *med * ratio
            };
            self.cap[lo..=hi].fill(cap);
            lo = hi + 1;
        }
        if let Some((lo, hi)) = self.cap_skip {
            self.cap[lo..=hi.min(self.n - 1)].fill(f32::INFINITY);
        }
    }

    /// A bin's smoothed power above the floor, as a weight for the centroid
    /// and the extent.
    #[inline]
    fn excess(bins: &Bins, i: usize) -> f64 {
        if bins.ratio(i) <= 0.0 {
            return 0.0;
        }
        (bins.power(i) - bins.floor(i)).max(0.0) as f64
    }

    /// Group the hot bins of this frame into runs.
    fn segment(&self, bins: &Bins, frame_saturated: bool) -> Vec<Segment> {
        let mut segs = Vec::new();
        let close = self.cfg.close_db;
        let close_r = 10f32.powf(close / 10.0);
        let guard = self.cfg.guard_bins;
        let mut runs: Vec<(usize, usize)> = Vec::new();
        let mut cur: Option<(usize, usize)> = None;
        let mut gap = 0usize;
        for i in self.bin_lo..=self.bin_hi {
            let hot = bins.ratio(i) >= close_r;
            match (&mut cur, hot) {
                (Some(r), true) => {
                    r.1 = i;
                    gap = 0;
                }
                (Some(r), false) => {
                    gap += 1;
                    if gap > guard {
                        runs.push(*r);
                        cur = None;
                        gap = 0;
                    }
                }
                (None, true) => {
                    cur = Some((i, i));
                    gap = 0;
                }
                (None, false) => {}
            }
        }
        if let Some(r) = cur {
            runs.push(r);
        }

        // Saturated: the strongest run is the signal, the rest are what the
        // converter made of it. Judged on smoothed power over the floor.
        // Saturated: the strongest run is the signal, and with it only what
        // could be the other tone of the same transmitter, within a few dB
        // and a pair's distance; a LaCrosse sensor keying 120 kHz apart
        // saturates the same way and is still two tones. The rest is what
        // the converter made of it.
        if frame_saturated && !runs.is_empty() {
            let peak = |r: &(usize, usize)| (r.0..=r.1).map(|i| bins.ratio(i)).fold(0.0f32, f32::max);
            let best = runs.iter().copied().max_by(|a, b| peak(a).total_cmp(&peak(b))).unwrap();
            let top = peak(&best);
            let pair_bins = (self.cfg.pair_hz / self.bin_hz()).round() as usize;
            runs.retain(|r| {
                let gap = if r.0 > best.1 { r.0 - best.1 } else { best.0.saturating_sub(r.1) };
                peak(r) * 10f32.powf(SATURATED_PAIR_DB / 10.0) >= top && gap <= pair_bins
            });
        }
        let extent_db = if frame_saturated { SATURATED_EXTENT_DB } else { self.cfg.extent_db };

        for (lo, hi) in runs {
            let mut peak_r = 0.0f32;
            let mut raw_sum = 0.0f32;
            let mut peak_bin = lo;
            let mut peak_w = -1.0f64;
            let mut peak_raw_w = 0.0f64;
            for i in lo..=hi {
                peak_r = peak_r.max(bins.ratio(i));
                raw_sum += bins.raw_ratio(i);
                let w = Self::excess(bins, i);
                if w > peak_w {
                    peak_w = w;
                    peak_bin = i;
                }
                let raw_w = (bins.floor(i) * (bins.raw_ratio(i) - 1.0)).max(0.0) as f64;
                peak_raw_w = peak_raw_w.max(raw_w);
            }
            let raw_mean = raw_sum / (hi - lo + 1) as f32;
            if raw_mean < close_r {
                continue;
            }
            let peak_db = 10.0 * peak_r.max(1e-20).log10();
            let raw_db = 10.0 * raw_mean.max(1e-20).log10();

            // Every bin within `extent_db` of the peak, and the span they
            // cover. Not a walk out from the peak: a strong burst's onset
            // lights a run across the whole band for a frame, and a walk
            // that stops at the first gap under the peak keeps one tone of a
            // two-tone signal and loses the other. The tones are within a
            // few dB of each other, a keyed carrier's splash and sidelobes
            // are tens of dB down, and that difference is the extent.
            //
            // Measured under the peak this frame reads raw as well as under
            // the smoothed one. A chirp's tone moves on every frame, so the
            // smoothed peak is the decaying trace of where it was, tens of
            // dB under where it is; and a receiver's front end driven hard
            // lifts its whole floor by a dozen dB while the signal lasts.
            // Against the decayed peak the lifted floor was within the
            // margin, and a 62 kHz MeshCore channel measured 566 kHz.
            //
            // Only in a run that has swallowed the band, and only where the
            // two disagree by more than a burst's onset does: a keyed
            // signal's raw spectrum is a few spikes and leads its smoothed
            // envelope by several dB every frame, and cutting on it took a
            // DMR channel down to 6 kHz and a sensor's 30 kHz down to 10.
            let flooded = (hi - lo + 1) * 2 > self.bin_hi + 1 - self.bin_lo;
            let over = flooded && peak_raw_w > peak_w * 10f64.powf(RAW_EXTENT_LEAD_DB / 10.0);
            let reference = if over { peak_raw_w } else { peak_w };
            let floor_w = reference * 10f64.powf(-(extent_db as f64) / 10.0);
            let (mut a, mut b) = (peak_bin, peak_bin);
            for i in lo..=hi {
                if Self::excess(bins, i) >= floor_w {
                    a = a.min(i);
                    b = b.max(i);
                }
            }
            // The run of a saturated frame is the whole band; matched on
            // that, the source would take every later run in the span.
            let (lo, hi) = if frame_saturated {
                (a.saturating_sub(guard).max(lo), (b + guard).min(hi))
            } else {
                (lo, hi)
            };
            let mut wsum = 0.0f64;
            let mut w_all = 0.0f64;
            for i in a..=b {
                let w = Self::excess(bins, i);
                wsum += w * i as f64;
                w_all += w;
            }
            let centroid = if w_all > 0.0 { wsum / w_all } else { (a + b) as f64 / 2.0 };
            segs.push(Segment { lo, hi, occ_lo: a, occ_hi: b, peak_db, raw_db, centroid });
        }
        segs
    }

    /// Match this frame's runs to the sources being followed, open the
    /// candidates that have lasted, and close the sources that have not.
    /// One frame's runs, taken into the tracks: what each source is now, what
    /// has just appeared, and what has opened or closed.
    fn track(&mut self, bins: &Bins) {
        let assigned = self.assign();
        self.absorb(&assigned, bins);
        let born = self.admit(&assigned, bins);
        self.pair_newborn(born);
        self.advance();
    }

    /// Which track each run of this frame belongs to, or none for a run that
    /// is something new.
    fn assign(&self) -> Vec<Option<usize>> {
        let guard = self.cfg.guard_bins;
        let n = self.n;
        let bin_hz = self.bin_hz();

        // Every run is matched against where each source *was*, before any
        // of them is moved. Matching against a source already narrowed by an
        // earlier run in the same frame loses the later runs of the same
        // signal, and each of those then opens as a source of its own: a
        // keyed carrier, whose spectrum has nulls that come and go from
        // frame to frame, opened fifty times in a fifth of a second that
        // way. An open source is matched on everything it has ever covered
        // rather than on its last frame, for the same reason.
        let ranges: Vec<(usize, usize)> = self
            .tracks
            .iter()
            .map(|t| {
                let (mut lo, mut hi) = (t.lo_bin, t.hi_bin);
                if t.open {
                    let lo_b = (t.src.lo_hz / bin_hz + (n / 2) as f64).floor().max(0.0) as usize;
                    let hi_b = (t.src.hi_hz / bin_hz + (n / 2) as f64).ceil().max(1.0) as usize - 1;
                    lo = lo.min(lo_b);
                    hi = hi.max(hi_b);
                }
                (lo.saturating_sub(guard), (hi + guard).min(n - 1))
            })
            .collect();

        let mut assigned: Vec<Option<usize>> = vec![None; self.segs.len()];
        for (si, s) in self.segs.iter().enumerate() {
            let mut best: Option<(usize, usize)> = None;
            for (ti, &(lo, hi)) in ranges.iter().enumerate() {
                if s.hi < lo || s.lo > hi {
                    continue;
                }
                let overlap = s.hi.min(hi) - s.lo.max(lo) + 1;
                if best.is_none_or(|(_, o)| overlap > o) {
                    best = Some((ti, overlap));
                }
            }
            assigned[si] = best.map(|(ti, _)| ti);
        }

        assigned
    }

    /// Fold this frame's runs into the sources they belong to.
    fn absorb(&mut self, assigned: &[Option<usize>], bins: &Bins) {
        let n = self.n;

        // A source is still there if the raw power where it was last seen
        // says so. A run overlapping its range is not enough on its own: a
        // burst's abrupt end splashes one wide frame, the extent grows to
        // match, and from then on single bins of noise inside that extent
        // reach the close threshold a few times a frame and would keep the
        // source alive for as long as they kept coming. A run loud enough
        // to open a source by itself does count, so a transmitter drifting
        // out of the bins it was in is followed rather than reopened.
        let close = self.cfg.close_db;
        let close_r = 10f32.powf(close / 10.0);
        let open = self.cfg.open_db;
        let present: Vec<bool> = self
            .tracks
            .iter()
            .map(|t| {
                // Where its power was, not the run it was part of: the
                // splash of a burst ending bridged the run to a spur 40 kHz
                // away, and the spur's power kept the source present after
                // the burst was gone.
                let occ = t.occ_lo..=t.occ_hi.min(n - 1);
                let count = occ.clone().count().max(1) as f32;
                let mean = occ.map(|i| bins.raw_ratio(i)).sum::<f32>() / count;
                mean >= close_r
            })
            .collect();

        for t in &mut self.tracks {
            t.matched = false;
        }
        // Each source takes its runs strongest first. The strongest is the
        // source; a weaker one joins it only if it could open a source by
        // itself, as the other tone of a keyed pair could. A single bin of
        // noise inside the range is not that, and folded in it widened the
        // extent, which then read as growth and had the source reopened at
        // twice its width.
        let mut by_track: Vec<Vec<usize>> = vec![Vec::new(); self.tracks.len()];
        for (si, ti) in assigned.iter().enumerate() {
            if let Some(ti) = ti {
                by_track[*ti].push(si);
            }
        }
        for (ti, runs) in by_track.iter_mut().enumerate() {
            runs.sort_by(|a, b| self.segs[*b].peak_db.partial_cmp(&self.segs[*a].peak_db).unwrap());
            for &si in runs.iter() {
                let s = self.segs[si];
                let strong = s.peak_db >= open && s.raw_db >= close;
                let t = &mut self.tracks[ti];
                if t.matched {
                    if !strong {
                        continue;
                    }
                    t.lo_bin = t.lo_bin.min(s.lo);
                    t.hi_bin = t.hi_bin.max(s.hi);
                    t.occ_lo = t.occ_lo.min(s.occ_lo);
                    t.occ_hi = t.occ_hi.max(s.occ_hi);
                } else {
                    if !present[ti] && !strong {
                        continue;
                    }
                    t.lo_bin = s.lo;
                    t.hi_bin = s.hi;
                    t.occ_lo = s.occ_lo;
                    t.occ_hi = s.occ_hi;
                    // Runs were sorted by peak, so this is the strongest
                    // this frame: where the signal is, rather than the
                    // middle of everything it lit up.
                    t.frame_centroid = s.centroid;
                    t.matched = true;
                    t.faint = s.peak_db < t.src.peak_snr_db - self.cfg.extent_db;
                    t.hits += 1;
                    t.misses = 0;
                    t.last_frame = self.frame;
                    t.src.frames += 1;
                }
                t.src.peak_snr_db = t.src.peak_snr_db.max(s.peak_db);
                t.peak_lo = t.peak_lo.min(s.peak_db);
                t.peak_hi = t.peak_hi.max(s.peak_db);
                if !t.open {
                    t.centroid_sum += s.centroid;
                    t.centroid_n += 1;
                }
            }
        }
    }

    /// Open a candidate for every run that belongs to nothing yet, and say
    /// how many tracks there were before them.
    fn admit(&mut self, assigned: &[Option<usize>], bins: &Bins) -> usize {
        let n = self.n;
        let bin_hz = self.bin_hz();

        // A strong signal brings a forest with it: its image at the
        // tuner's rejection, intermodulation and reciprocal mixing across
        // the span, all narrow, all tens of dB under it, all born as it
        // keys and gone as it stops. Minimum statistics learn them as floor
        // only if they stay for the floor's whole memory, so a transmitter
        // that keys for a moment every few seconds reopened its forest every
        // time: thirty sources in a frame, each with a full set of decoders.
        // Nothing that opens this far under a source still younger than the
        // floor's memory is believed. What it costs is a transmitter at that
        // margin keying up inside the strong one's first seconds.
        //
        // Not one standing on the tuner's own centre: a direct-conversion
        // receiver's DC offset follows the envelope of whatever is in the
        // span, opens and closes with it, and is refused downstream, so as
        // the strongest thing in the band it would have hidden every
        // transmitter that made it move. Candidates count as well as open
        // sources, since a burst's image is a frame or two behind it.
        let memory = (self.floor.sub_len * self.floor.sub_count) as u64;
        let skip = self.cap_skip;
        let off_centre = |lo: usize, hi: usize| !skip.is_some_and(|(a, b)| lo <= b && hi >= a);
        let dominant = self
            .tracks
            .iter()
            .filter(|t| self.frame - t.born <= memory)
            .filter(|t| off_centre(t.occ_lo, t.occ_hi))
            .map(|t| t.src.peak_snr_db)
            .fold(f32::NEG_INFINITY, f32::max);
        // And the strongest run being born this frame, since the image of a
        // burst is born in the same frame as the burst: a MeshCore advert at
        // 59 dB and its mirror at 28 dB opened together, and the mirror was
        // read as a chirp sweeping the wrong way for the whole packet.
        let dominant = self
            .segs
            .iter()
            .enumerate()
            .filter(|(si, s)| assigned[*si].is_none() && off_centre(s.occ_lo, s.occ_hi))
            .map(|(_, s)| s.peak_db)
            .fold(dominant, f32::max);
        // How much of the span is lit at once. A band with things
        // transmitting on it has a few channels above the threshold; a band
        // under a transmission far wider than anything here reads has most
        // of it, and the spikes of that transmission's own spectrum are not
        // sources. Measured in hertz rather than as a fraction of the span,
        // because what it separates is "wider than anything readable" from
        // "several readable channels", and both of those are widths: as a
        // fraction, a 500 kHz LoRa channel fills a fifth of a 2.4 MS/s span
        // and would read as a blanket.
        let close_r = 10f32.powf(self.cfg.close_db / 10.0);
        let lit_hz =
            (self.bin_lo..=self.bin_hi).filter(|i| bins.ratio(*i) >= close_r).count() as f64
                * self.bin_hz();
        let blanket = lit_hz > BLANKET_HZ;
        let born = self.tracks.len();
        for (si, s) in self.segs.iter().enumerate() {
            if assigned[si].is_some() || s.peak_db < self.cfg.open_db {
                continue;
            }
            // Under a blanket only something well clear of it is a
            // transmitter worth following. Wi-Fi on 2.4 GHz lit four fifths
            // of a 16 MHz span and its spikes stood 15 to 20 dB over the
            // floor, which opened a new source every 40 ms, most of them a
            // few tens of kilohertz wide, on frequencies nothing was
            // transmitting on.
            if blanket && s.peak_db < self.cfg.open_db + BLANKET_CLEAR_DB {
                continue;
            }
            if !self.locked.is_empty() {
                let hz = hz_of_bin(s.centroid, n, bin_hz);
                if self.locked.iter().any(|o| o.holds(hz)) {
                    continue;
                }
            }
            if s.occ_hi + 1 - s.occ_lo < self.cfg.min_bins {
                continue;
            }
            if s.peak_db < dominant - SPUR_DB {
                continue;
            }
            let id = SourceId(self.next_id);
            self.next_id += 1;
            self.tracks.push(Track {
                src: Source {
                    id,
                    lo_hz: 0.0,
                    hi_hz: 0.0,
                    center_hz: 0.0,
                    start_sample: frame_start(self.frame, self.hop as u64),
                    end_sample: None,
                    peak_snr_db: s.peak_db,
                    frames: 1,
                },
                lo_bin: s.lo,
                hi_bin: s.hi,
                occ_lo: s.occ_lo,
                occ_hi: s.occ_hi,
                hits: 1,
                peak_lo: s.peak_db,
                peak_hi: s.peak_db,
                misses: 0,
                open: false,
                born: self.frame,
                last_frame: self.frame,
                opened_hz: 0.0,
                opened_center_hz: 0.0,
                frame_centroid: s.centroid,
                centres: [0.0; GROWTH_FRAMES],
                seen_frames: 0,
                centroid_sum: s.centroid,
                centroid_n: 1,
                matched: true,
                faint: false,
            });
        }

        born
    }

    /// Merge the candidates born this frame that are one transmitter.
    fn pair_newborn(&mut self, born: usize) {
        let bin_hz = self.bin_hz();

        // Runs born together and near each other are one transmitter: the
        // two tones of a frequency-shift-keyed signal, keyed up in the same
        // frame with a gap between them wider than any guard should bridge.
        // Only runs of comparable strength pair: the tones of one
        // transmitter are within a few dB of each other, where a keyed
        // carrier's onset splash is tens of dB under it and a spur
        // elsewhere in the band is whatever it happens to be.
        let pair_bins = (self.cfg.pair_hz / bin_hz).round() as usize;
        let mut k = born;
        while k + 1 < self.tracks.len() {
            let (a, b) = (&self.tracks[k], &self.tracks[k + 1]);
            let alike = (a.src.peak_snr_db - b.src.peak_snr_db).abs() <= 12.0;
            if alike && b.lo_bin.saturating_sub(a.hi_bin) <= pair_bins {
                let b = self.tracks.remove(k + 1);
                let a = &mut self.tracks[k];
                a.lo_bin = a.lo_bin.min(b.lo_bin);
                a.hi_bin = a.hi_bin.max(b.hi_bin);
                a.occ_lo = a.occ_lo.min(b.occ_lo);
                a.occ_hi = a.occ_hi.max(b.occ_hi);
                a.src.peak_snr_db = a.src.peak_snr_db.max(b.src.peak_snr_db);
                // The centre of a pair is between its tones, which is where
                // a discriminator wants zero, not the centre of either.
                a.centroid_sum = (a.occ_lo + a.occ_hi) as f64 / 2.0;
                a.centroid_n = 1;
                self.next_id = b.src.id.0;
            } else {
                k += 1;
            }
        }

    }

    /// Carry every track forward: what it covers now, whether it opens,
    /// whether a wider stream supersedes it, and whether it has gone.
    fn advance(&mut self) {
        let n = self.n;
        let bin_hz = self.bin_hz();

        // Extents are kept in hertz on the source so they survive the bins
        // moving, and updated from whatever the track saw this frame.
        let hop = self.hop as u64;
        let min_frames = self.cfg.min_frames;
        let steady_db = self.cfg.steady_db;
        // Candidates born before this frame appeared with the floor cap,
        // as a fixture of the receiver does, and have to move before they
        // are believed. Before the cap exists nothing steady can be a
        // candidate at all, so until then every candidate is a transmission.
        let fixture_until = if self.cap_first == 0 {
            0
        } else {
            self.cap_first + (self.cfg.fixture_s * self.rate / self.hop as f64) as u64
        };
        let hang = self.hang_frames;
        let regrow = self.cfg.regrow;
        let integrate = self.cfg.integrate_frames.max(1);
        let max_width = self.cfg.max_width_hz;
        let max_open = self.cfg.max_open.max(1);
        let mut open_count = self.tracks.iter().filter(|t| t.open).count();
        let open_now = &mut open_count;
        let capped = &mut self.capped;
        // The quietest thing currently open, which is what a louder
        // candidate takes the place of when the cap is reached.
        let weakest = self
            .tracks
            .iter()
            .filter(|t| t.open)
            .map(|t| (t.src.id, t.src.peak_snr_db))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        let mut displace: Option<SourceId> = None;
        let displaced = &mut displace;
        let events = &mut self.events;
        let next_id = &mut self.next_id;
        self.tracks.retain_mut(|t| {
            let lo_hz = (t.occ_lo as f64 - (n / 2) as f64) * bin_hz;
            let hi_hz = (t.occ_hi as f64 + 1.0 - (n / 2) as f64) * bin_hz;
            // A run that would take an open source past the widest thing
            // read here is not that source: the span lit end to end by a
            // saturated converter, standing over a sensor's channel. The
            // source misses the frame instead, and closes if that goes on.
            let flood = t.open && hi_hz.max(t.src.hi_hz) - lo_hz.min(t.src.lo_hz) > max_width;
            if t.matched && !flood {
                if t.open {
                    if !t.faint {
                        t.src.lo_hz = t.src.lo_hz.min(lo_hz);
                        t.src.hi_hz = t.src.hi_hz.max(hi_hz);
                    }
                    let width = t.src.bandwidth_hz();
                    let centre = t.frame_centroid;
                    // Recorded only once the smoother has settled after
                    // opening, when the extent has filled in.
                    let settle = 2 * integrate;
                    t.seen_frames += 1;
                    let mut before = centre;
                    if t.seen_frames > settle {
                        let slot = (t.seen_frames - settle) % GROWTH_FRAMES;
                        before = t.centres[slot];
                        t.centres[slot] = centre;
                    }
                    // Outgrown its extraction, and moving: the same
                    // transmitter again, under a new id, at the width it has
                    // turned out to have and from where it began.
                    let sweeping =
                        t.seen_frames > settle + GROWTH_FRAMES && (centre - before).abs() >= 4.0;
                    // Moving inside the extent it opened with is what a
                    // two-level keyed carrier does: the centroid sits on
                    // whichever tone is being sent, and on a wM-Bus meter
                    // those are 100 kHz apart, which read as a sweep and
                    // reopened the meter as a source three times its width,
                    // centred between the tones and off the signal, with
                    // every front end placed for the narrow one dropped.
                    let centre_hz = hz_of_bin(centre, n, bin_hz);
                    let left = (centre_hz - t.opened_center_hz).abs() > t.opened_hz / 2.0;
                    if sweeping && left && width > t.opened_hz * regrow + 2.0 * bin_hz {
                        t.seen_frames = 0;
                        events.push(SourceEvent::Superseded(t.src));
                        t.src.id = SourceId(*next_id);
                        *next_id += 1;
                        t.src.center_hz = (t.src.lo_hz + t.src.hi_hz) / 2.0;
                        t.opened_hz = t.src.bandwidth_hz();
                        t.opened_center_hz = t.src.center_hz;
                        events.push(SourceEvent::Opened(t.src));
                    }
                } else {
                    t.src.lo_hz = lo_hz;
                    t.src.hi_hz = hi_hz;
                    // Too wide to be anything read here, but kept as a
                    // candidate rather than dropped: it takes the runs
                    // inside it that would otherwise each be born as a
                    // source of their own, and opens if it narrows.
                    let fits = hi_hz - lo_hz <= max_width;
                    let moved = t.peak_hi - t.peak_lo >= steady_db;
                    // Room for it, or louder than the quietest thing already
                    // open, which is then closed. Reported as closed rather
                    // than dropped, so a front end reading it is torn down
                    // the same way it would be at the end of a transmission.
                    // Room, or louder by a clear margin than the quietest
                    // thing open, which is then closed to make room. Without
                    // the margin a pair of sources within a decibel of each
                    // other would take turns evicting one another.
                    let mut room = *open_now < max_open;
                    if !room {
                        if let Some((id, db)) = weakest {
                            if displaced.is_none() && t.peak_hi > db + 3.0 {
                                *displaced = Some(id);
                                room = true;
                            }
                        }
                        if !room {
                            *capped += 1;
                        }
                    }
                    if room && fits && t.hits >= min_frames && (moved || t.born >= fixture_until) {
                        *open_now += 1;
                        t.open = true;
                        let c = t.centroid_sum / t.centroid_n.max(1) as f64;
                        t.src.center_hz = hz_of_bin(c, n, bin_hz);
                        t.opened_hz = t.src.bandwidth_hz();
                        t.opened_center_hz = t.src.center_hz;
                        events.push(SourceEvent::Opened(t.src));
                    }
                }
                return true;
            }
            if !t.open {
                // A candidate has to be there every frame; one that was not
                // is noise that reached the threshold once.
                return false;
            }
            t.misses += 1;
            if t.misses > hang {
                // Seen up to the end of the last frame it appeared in.
                t.src.end_sample = Some(frame_start(t.last_frame, hop) + n as u64);
                events.push(SourceEvent::Closed(t.src));
                return false;
            }
            true
        });
        if let Some(id) = displace {
            let hop = self.hop as u64;
            let n = self.n;
            self.tracks.retain_mut(|t| {
                if t.src.id != id {
                    return true;
                }
                t.src.end_sample = Some(frame_start(t.last_frame, hop) + n as u64);
                self.events.push(SourceEvent::Closed(t.src));
                false
            });
        }
    }

    /// Drop the sources that opened somewhere they may not.
    ///
    /// At the end of the block rather than as a track is born, because a
    /// track born before a front end claimed the channel can open after it,
    /// and because whether an opening on the tuner's own centre is a
    /// transmitter depends on what else is on the air when it opens. The
    /// track goes with the event: a refused source that stayed in the list
    /// was drawn on the spectrum, counted against the cap, and closed later
    /// as if it had been read, which is a receiver contradicting its own
    /// decision.
    fn refuse_claimed(&mut self) {
        if self.events.is_empty() || (self.locked.is_empty() && self.spur.is_none()) {
            return;
        }
        let spur = self.spur;
        let in_spur = |hz: f64| spur.is_some_and(|(lo, hi)| (lo..=hi).contains(&hz));
        // The offset follows another transmission's envelope, so it is only
        // the spur while there is one to follow.
        let others = self
            .tracks
            .iter()
            .filter(|t| t.open && !in_spur(t.src.center_hz))
            .count();
        let locked = &self.locked;
        let mut refused: Vec<SourceId> = Vec::new();
        self.events.retain(|e| {
            let SourceEvent::Opened(s) = e else { return true };
            let owned = locked
                .iter()
                .any(|o| o.holds(s.center_hz) && s.bandwidth_hz() <= o.max_width_hz);
            if owned || (others > 0 && in_spur(s.center_hz)) {
                refused.push(s.id);
                return false;
            }
            true
        });
        if refused.is_empty() {
            return;
        }
        self.tracks.retain(|t| !refused.contains(&t.src.id));
        self.events.retain(|e| !refused.contains(&e.source().id));
    }

    /// Candidates refused because the cap on open sources was reached.
    pub fn capped(&self) -> u64 {
        self.capped
    }

}

/// Spectrum lit at once beyond which a frame is taken to be under one wide
/// transmission rather than to hold many narrow ones.
///
/// Wi-Fi is 20 MHz and nothing here reads anything over about half a
/// megahertz, so a span with megahertz of it above the threshold is a span
/// with something on it that cannot be read, whatever its spectrum's spikes
/// look like.
const BLANKET_HZ: f64 = 4_000_000.0;

/// How far over the opening threshold a run must stand to be believed while
/// the band is blanketed.
const BLANKET_CLEAR_DB: f32 = 15.0;

/// Frames over which a source's movement is measured before it is reopened.
/// Four milliseconds at the default resolution: a chirp at the highest
/// spreading factor moves several bins in that, and nothing keyed does.
const GROWTH_FRAMES: usize = 16;

/// How far under a young strong source a new candidate is taken to be one
/// of its spurs rather than a transmitter. An RTL-SDR's image sits about
/// 32 dB down and its intermodulation products 28 to 34 dB under a 44 dB
/// burst; a second transmitter within this margin still opens.
const SPUR_DB: f32 = 25.0;

/// How far this frame's raw peak has to stand over the smoothed one before
/// the extent is measured under it instead. Three costs a corpus capture;
/// seven loses a MeshCore packet.
const RAW_EXTENT_LEAD_DB: f64 = 5.0;

/// Sample magnitude, on either axis, taken to be the converter's rail. Every
/// driver here delivers full scale as one.
const RAIL: f32 = 0.98;
/// Just past full scale; nothing a converter delivered is above it.
const RAIL_TOP: f32 = 1.02;

/// Frames either side of a saturated one treated the same way.
const SATURATION_SMEAR: usize = 2;

/// How close to the strongest run another run must be, in a saturated
/// frame, to be kept as the other tone of the same transmitter. The two
/// tones of a LaCrosse sensor read within a decibel of each other; a keying
/// product born in the same frame 50 kHz away read 12 dB down and, paired
/// with the signal, put the source's centre between them.
const SATURATED_PAIR_DB: f32 = 6.0;

/// Extent margin used in a saturated frame: the signal's own lobe, under the
/// products the converter adds around it.
const SATURATED_EXTENT_DB: f32 = 12.0;

/// Frames after a start or a silence before the floor is measured. At the
/// default resolution that is eight milliseconds: longer than any filter's
/// fade-in, shorter than the lead-in of every capture in the corpus.
const SETTLE_FRAMES: u64 = 32;
