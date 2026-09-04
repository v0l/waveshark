//! Source detection and extraction: find what is transmitting in a wideband
//! stream, and hand each transmitter over as its own stream.
//!
//! The channel bank decided a signal's width before it had seen the signal.
//! A grid of fixed channels was split over a span, a power gate watched every
//! channel, and whatever landed in one was read through that channel's
//! filter, however wide the signal really was. That is why there were four
//! grids of different widths over the same band, and why one burst arrived
//! in several of them at once.
//!
//! This inverts the order. The span is watched as a spectrogram, one FFT
//! frame at a time. Each bin tracks its own noise floor, so a bin above it is
//! a bin with something in it, and a run of such bins that persists from one
//! frame to the next is a *source*: something transmitting at a centre with
//! a width, from one instant until another. Width and centre come out of the
//! measurement rather than going into it.
//!
//! Each source then gets its own extraction, mixed to baseband and decimated
//! to a rate that fits the width just measured, from a short ring of the
//! wideband stream so the lead-in the detector needed to make up its mind is
//! not lost. What comes out is a run of [`SourceBlock`]s per source, in
//! order, with no gaps: a stream, however brief. A packet four milliseconds
//! long is a stream that runs four milliseconds; a broadcast carrier is one
//! that never closes. Whatever reads them is built when the source opens and
//! dropped when it closes, and does not have to know which it was given.
//!
//! # Where the floor comes from
//!
//! Minimum statistics per bin, as in [`crate::detect`]: the minimum of the
//! smoothed power over a window longer than any transmission, corrected for
//! the bias a minimum carries. The correction here is derived from the
//! smoothing rather than fixed, because the smoothed power of noise is not
//! exponential any more and the square-root rule of thumb over-corrects it by
//! several dB, which is sensitivity thrown away.
//!
//! # What this cannot do yet
//!
//! Two transmitters overlapping in frequency at the same time are one source,
//! and will read as nothing sensible. A frequency hopper is a new source per
//! hop. A source's extraction is designed at the width it had when it opened;
//! one whose extent keeps growing afterwards, a sweep, is reopened at the
//! full width from its start, but one that widens once and holds is read
//! through the filter it opened with.

use crate::fir::{self, FirDecim};
use crate::mixer::Mixer;
use crate::window;
use common::{SourceBlock, SourceId, SourceState, C32};
use rayon::prelude::*;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

#[derive(Clone, Copy, Debug)]
pub struct SourceConfig {
    /// FFT frame length, or zero to pick one giving bins near `bin_hz`.
    pub fft_size: usize,
    /// Bin width to aim for when `fft_size` is zero.
    ///
    /// Narrower bins hear a weak narrow signal better, since less noise is
    /// integrated beside it, and cost time resolution: a bin of `b` hertz is
    /// a frame of `1/b` seconds. Two kilohertz puts a 1.5 kbit/s sensor in a
    /// couple of bins and resolves half a millisecond, which is the pulse
    /// width the fastest ISM devices key at.
    pub bin_hz: f64,
    /// SNR a bin must reach for a new source to open there, in dB.
    pub open_db: f32,
    /// SNR below which a bin no longer counts towards a source, in dB. Held
    /// under `open_db` so a signal fading around one threshold does not open
    /// and close a source every frame.
    pub close_db: f32,
    /// Frames of exponential power smoothing before thresholding. The power
    /// of one frame of noise in one bin is exponentially distributed and
    /// cannot be thresholded on its own; four frames bring its spread to a
    /// few dB.
    pub integrate_frames: usize,
    /// How far back the floor looks, in seconds. On its own this must exceed
    /// the longest transmission expected, or the transmission is learned as
    /// floor; `floor_cap_db` is what removes that requirement.
    pub floor_memory_s: f64,
    /// How far a bin's floor may sit above the floor of the bins around it,
    /// in dB, before it is taken to be a signal rather than noise.
    ///
    /// A floor from minimum statistics alone cannot see a carrier that never
    /// stops: the minimum in its bin is the carrier, so it reads as its own
    /// noise and its SNR is zero. TETRA base station downlinks are the case
    /// that showed it, on air continuously and already transmitting before
    /// the receiver tuned to them, so they were never reported at all. The
    /// band answers what time cannot: a carrier is a few bins out of
    /// hundreds, and the bins beside it say what the noise is.
    ///
    /// The cap is the headroom left for the floor's own shape across a
    /// chunk, so it is set by tilt, not by any signal. Filter roll-off,
    /// tuner gain slope and a distant transmitter's shoulder are a few dB
    /// over a few hundred kilohertz, and a signal worth reporting is tens.
    /// Set it high enough that no chunk trips it and the detector goes back
    /// to minimum statistics alone.
    pub floor_cap_db: f32,
    /// Bins the cap's floor is measured over, or zero for an eighth of the
    /// frame, and never fewer than sixty-four.
    ///
    /// The measure is a median, so it holds while the signals in a chunk are
    /// under half of it: a 25 kHz channel in a chunk of a quarter megahertz
    /// is a fifteenth. Wider chunks tolerate wider signals and follow the
    /// floor's shape less closely. A signal that fills its chunk is
    /// indistinguishable from a noise floor by this test and is left to the
    /// minimum statistics, which is the right answer for a band nobody can
    /// see past.
    pub floor_chunk_bins: usize,
    /// Consecutive frames a run of bins must persist before it is a source.
    pub min_frames: usize,
    /// Silence before a source closes, in microseconds.
    ///
    /// Also what holds a source open across the gaps inside it. Fine Offset
    /// stations repeat their frame three times with 8 ms between repeats,
    /// and those three belong to one source and one package. A DMR radio
    /// on one slot of two keys 30 ms bursts with 30 ms between them, and a
    /// hang shorter than that gap closed the source at every burst: each
    /// slot opened as a transmission of its own, with a fresh decoder that
    /// had a burst to read and no superframe to read it in.
    pub hang_us: u32,
    /// Bins of gap to bridge inside one source. A keyed signal has nulls in
    /// its spectrum, and a null is not the edge of the signal.
    pub guard_bins: usize,
    /// How far apart two runs can be and still be one transmitter, when
    /// they appear in the same frame, in hertz.
    ///
    /// Frequency-shift keying is two tones with nothing between them, and
    /// the gap is often far wider than either tone: a LaCrosse sensor keys
    /// tones 120 kHz apart that are 10 kHz wide. They are one source, and
    /// the extraction has to hold both or the discriminator has nothing to
    /// discriminate. Two transmitters keying up in the same half
    /// millisecond within this distance are merged too, which costs a wider
    /// extraction and nothing else.
    pub pair_hz: f64,
    /// Samples handed over from before the source opened, in microseconds.
    /// A demodulator's gate needs noise to measure the signal against, and
    /// the detector took a few frames to decide, so the burst's own start is
    /// already behind by the time it opens.
    pub lead_us: u32,
    /// Samples handed over after the source closed, in microseconds. What a
    /// pulse front end needs to see the silence that ends a package.
    pub tail_us: u32,
    /// Output rate as a multiple of the extracted width.
    pub oversample: f64,
    /// Lowest rate a source is extracted at. A pulse read at a few kS/s has
    /// no timing left to measure.
    pub min_rate_hz: f64,
    /// Width kept around a source, as a multiple of the width measured. The
    /// bins above threshold are the loud middle of a signal, not its edges.
    pub width_margin: f64,
    /// How far under a source's peak a bin can be and still count towards
    /// its extent, in dB. Bounds the extent of a strong signal, whose keying
    /// sidebands and switching transients sit over the floor far beyond
    /// anything a receiver would call its width.
    pub extent_db: f32,
    /// Stopband of the extraction filter, in dB.
    pub atten_db: f64,
    /// Movement in a candidate's peak power, in dB, before it is taken to be
    /// a transmission rather than a fixture of the receiver.
    ///
    /// Now that a carrier which never stops is no longer absorbed into the
    /// floor, the receiver's own spurs are not either: the tuner's leakage at
    /// the centre, a switching supply's harmonic, a bare oscillator. What
    /// separates those from a transmission is not width or strength but that
    /// nothing is being sent. A modulated carrier's strongest bin moves by
    /// several dB from frame to frame however constant its envelope, and an
    /// unmodulated one does not move at all.
    ///
    /// A candidate that has not moved this far is held as a candidate rather
    /// than discarded, so it opens on the frame it first does. The cost is
    /// that a genuinely unmodulated carrier, a beacon sending nothing, is
    /// never reported; it is indistinguishable from a spur by any measure
    /// this detector has.
    ///
    /// Asked only of a candidate that appeared when the floor cap first
    /// did, within `fixture_s` of it. Minimum statistics hide a fixture
    /// until the cap unhides it, so that is the frame every fixture is born
    /// in; a candidate born later came from nothing, and that is movement
    /// enough. Asked of everything, it cost a short on-off keyed burst:
    /// smoothed over the integration, its peak did not move three decibels
    /// in the whole of its life.
    pub steady_db: f32,
    /// How long after the floor cap is first measured a new candidate is
    /// still taken to be possibly a fixture, in seconds, and so has to move
    /// `steady_db` before it opens. A few frames of integration is all a
    /// fixture needs to appear once it is unhidden.
    pub fixture_s: f64,
    /// Fewest bins a run must occupy to open a source.
    ///
    /// Two, because nothing keyed is one bin wide, and what is one bin wide
    /// is a spur: the tuner's own leakage, a switching supply's harmonic, a
    /// bare oscillator. Each of those would otherwise be a carrier reported
    /// every half second for as long as the receiver ran.
    pub min_bins: usize,
    /// Growth of a source's extent past the width it opened at that has it
    /// reopened at the new width, as a ratio.
    ///
    /// A slow chirp sweeps a few kilohertz in the frames it takes to open
    /// and hundreds over its symbol, so the width it opens at is a sliver of
    /// what it is. An extraction designed at open would keep the sliver and
    /// lose the sweep. When the measured extent outgrows the extraction, the
    /// stream is closed as superseded and a new one opened at the full width
    /// from the transmitter's start, which is what the history below is for.
    pub regrow: f64,
    /// Wideband samples kept behind the newest, in seconds, so a reopened
    /// source can start again from where it began.
    pub history_s: f64,
    /// Widest a source may be, in hertz. Nothing wider opens, and an open
    /// source stops growing rather than pass it.
    ///
    /// A receiver driven into saturation lights its whole span: the floor
    /// comes up, intermodulation lines stand every few hundred kilohertz,
    /// and the detector reads one thing as wide as the input. Cut out at
    /// the full rate and handed to every front end that will take it, that
    /// cost more than the rest of the band together and decoded nothing,
    /// since nothing this receiver reads is wider than a 500 kHz LoRa
    /// channel. Set it above the widest signal a front end reads.
    pub max_width_hz: f64,
}

impl Default for SourceConfig {
    fn default() -> Self {
        Self {
            fft_size: 0,
            bin_hz: 2_000.0,
            open_db: 10.0,
            close_db: 5.0,
            integrate_frames: 4,
            floor_memory_s: 2.0,
            floor_cap_db: 8.0,
            floor_chunk_bins: 0,
            min_frames: 2,
            hang_us: 40_000,
            guard_bins: 2,
            pair_hz: 150_000.0,
            lead_us: 5_000,
            tail_us: 30_000,
            oversample: 2.5,
            min_rate_hz: 25_000.0,
            width_margin: 1.5,
            extent_db: 20.0,
            atten_db: 60.0,
            steady_db: 3.0,
            fixture_s: 0.05,
            min_bins: 2,
            regrow: 1.5,
            history_s: 0.3,
            max_width_hz: 600_000.0,
        }
    }
}

impl SourceConfig {
    /// The frame length this configuration uses at a given rate.
    pub fn fft_size_at(&self, rate: f64) -> usize {
        if self.fft_size >= 16 {
            return self.fft_size.next_power_of_two();
        }
        let n = (rate / self.bin_hz.max(1.0)).round().max(16.0) as usize;
        n.next_power_of_two().clamp(16, 1 << 15)
    }
}

/// One transmitter, as the detector sees it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Source {
    pub id: SourceId,
    /// Extent, as offsets from the stream centre in hertz. Grows over the
    /// source's life and never shrinks: a signal that was wide for a moment
    /// was that wide.
    pub lo_hz: f64,
    pub hi_hz: f64,
    /// Power-weighted centre of the first frames, as an offset in hertz.
    pub center_hz: f64,
    /// Wideband sample index of the frame the source was first seen in.
    pub start_sample: u64,
    /// Wideband sample index the source was last seen up to, once closed.
    pub end_sample: Option<u64>,
    pub peak_snr_db: f32,
    /// Frames the source was seen in.
    pub frames: u64,
}

impl Source {
    /// Width of the extent measured so far, in hertz.
    pub fn bandwidth_hz(&self) -> f64 {
        self.hi_hz - self.lo_hz
    }
}

/// A source opening or closing. Everything in between is a source that is
/// simply still there, which [`SourceDetector::live`] lists.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SourceEvent {
    Opened(Source),
    Closed(Source),
    /// Closed because it outgrew its width; an `Opened` for the same
    /// transmitter, under a new id and at the new width, follows in the same
    /// batch.
    Superseded(Source),
}

impl SourceEvent {
    pub fn source(&self) -> &Source {
        match self {
            SourceEvent::Opened(s) | SourceEvent::Closed(s) | SourceEvent::Superseded(s) => s,
        }
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
fn floor_bias(alpha: f32, k: usize) -> f32 {
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
    /// The floor as a power, per bin, once measured.
    floor_lin: Vec<f32>,
    /// Smoothed power over the floor, as a ratio; zero before the floor is
    /// known. Kept linear: a logarithm per bin per frame is what the span
    /// the detector keeps up with was being spent on.
    ratio: Vec<f32>,
    /// This frame's own power over the floor, unsmoothed, as a ratio.
    ///
    /// Sensitivity comes from the smoothed power and timing from this. A
    /// 45 dB signal takes thirty frames of smoothing to decay under the
    /// close threshold after it stops, and a source that lingered that long
    /// would be timed 8 ms late; the raw frame says at once that it is gone.
    raw_ratio: Vec<f32>,
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
            floor_lin: vec![0.0; n],
            ratio: vec![0.0; n],
            raw_ratio: vec![0.0; n],
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
        let bin = self.bin_hz();
        let half = (self.n / 2) as f64;
        let lo = ((lo_hz / bin) + half).floor().max(0.0) as usize;
        let hi = ((hi_hz / bin) + half).ceil().max(1.0) as usize - 1;
        self.bin_lo = self.bin_lo.max(lo);
        self.bin_hi = self.bin_hi.min(hi.min(self.n - 1));
        if self.bin_hi < self.bin_lo {
            self.bin_hi = self.bin_lo;
        }
    }

    /// Leave the floor cap off between two offsets from the stream centre,
    /// for a fixture of the receiver that lives at a known frequency.
    pub fn exempt_from_cap(&mut self, lo_hz: f64, hi_hz: f64) {
        let bin = self.bin_hz();
        let half = (self.n / 2) as f64;
        let lo = ((lo_hz / bin) + half).floor().max(0.0) as usize;
        let hi = (((hi_hz / bin) + half).ceil() as usize).min(self.n - 1);
        self.cap_skip = (lo <= hi).then_some((lo, hi));
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
        self.floor_lin.fill(0.0);
        self.ratio.fill(0.0);
        self.raw_ratio.fill(0.0);
        self.frame = 0;
        self.settle_at = 0;
        self.silent_so_far = true;
        self.tracks.clear();
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
        // independent of every other's, and at 20 MS/s the transforms are
        // the whole cost of the detector. The floor, the runs and the tracks
        // are then a cheap pass in frame order.
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
        for f in 0..count {
            // The frames either side too: the converter clips a symbol
            // or two after the signal's edge lit the band, and it is that
            // edge frame's splash, read with the ordinary margin, that
            // opened a 12 kHz signal 70 kHz wide.
            let lo = f.saturating_sub(SATURATION_SMEAR);
            let hi = (f + SATURATION_SMEAR).min(count - 1);
            self.frame_saturated = saturated[lo..=hi].iter().any(|s| *s);
            self.frame_from(&spectra[f * n..(f + 1) * n], silent[f]);
        }
        self.spectra = spectra;
        self.silent = silent;
        self.saturated = saturated;

        let pos = count * hop;
        self.pending.drain(..pos);
        self.consumed += pos as u64;
        &self.events
    }

    /// One frame, given its raw power per bin in display order.
    fn frame_from(&mut self, raw: &[f32], silent: bool) {
        let alpha = self.alpha;
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

        // Every bin, across the pool. The floor's counters are shared, so
        // the bias is one number for the frame and the shared state is
        // stepped once afterwards.
        let measure = settled && !silent;
        let completing = measure && self.floor.completing();
        let stored = if completing {
            (self.floor.stored + 1).min(self.floor.sub_count)
        } else {
            self.floor.stored
        };
        let head = self.floor.head;
        let sc = self.floor.sub_count;
        let bias = if measure { floor_bias(alpha, self.floor.frames_after()) } else { 1.0 };
        if measure {
            self.measure_caps();
        }
        const CHUNK: usize = 256;
        self.power
            .par_chunks_mut(CHUNK)
            .zip(self.floor.current.par_chunks_mut(CHUNK))
            .zip(self.floor.min.par_chunks_mut(CHUNK))
            .zip(self.floor.mins.par_chunks_mut(CHUNK * sc))
            .zip(self.floor_lin.par_chunks_mut(CHUNK))
            .zip(self.ratio.par_chunks_mut(CHUNK))
            .zip(self.raw_ratio.par_chunks_mut(CHUNK))
            .zip(self.cap.par_chunks(CHUNK))
            .zip(raw.par_chunks(CHUNK))
            .for_each(
                |((((((((power, current), min), mins), floor), ratio), raw_ratio), cap), raw)| {
                for i in 0..power.len() {
                    let p = raw[i];
                    if silent {
                        ratio[i] = 0.0;
                        raw_ratio[i] = 0.0;
                        continue;
                    }
                    if seed {
                        power[i] = p;
                    } else {
                        power[i] += alpha * (p - power[i]);
                    }
                    if !settled {
                        ratio[i] = 0.0;
                        raw_ratio[i] = 0.0;
                        continue;
                    }
                    let m = floor_update(
                        power[i],
                        &mut current[i],
                        &mut mins[i * sc..(i + 1) * sc],
                        &mut min[i],
                        completing,
                        head,
                        stored,
                    );
                    let f = m.min(cap[i]) * bias;
                    if f > 0.0 && f.is_finite() {
                        floor[i] = f;
                        ratio[i] = power[i] / f;
                        raw_ratio[i] = p / f;
                    } else {
                        floor[i] = 0.0;
                        ratio[i] = 0.0;
                        raw_ratio[i] = 0.0;
                    }
                }
                },
            );
        if measure {
            self.floor.advance();
        }

        self.segment();
        self.track();
        self.frame += 1;
    }

    /// Set each bin's ceiling from the median floor of the chunk it is in.
    ///
    /// The median is taken over the minimum statistics rather than over this
    /// frame's power, so a burst passing through a chunk cannot lift the
    /// ceiling for the bins beside it, and the number it produces is the one
    /// the rest of the floor is already expressed in.
    fn measure_caps(&mut self) {
        let refresh = self.floor.sub_len as u64;
        if self.cap_at != 0 && self.frame < self.cap_at + refresh {
            return;
        }
        // Nothing complete yet: the running minimum is still falling towards
        // the noise, and a ceiling from it would be measured on a floor that
        // is about to move.
        if self.floor.stored == 0 {
            return;
        }
        self.cap_at = self.frame.max(1);
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
    fn excess(&self, i: usize) -> f64 {
        if self.ratio[i] <= 0.0 {
            return 0.0;
        }
        (self.power[i] - self.floor_lin[i]).max(0.0) as f64
    }

    /// Group the hot bins of this frame into runs.
    fn segment(&mut self) {
        self.segs.clear();
        let close = self.cfg.close_db;
        let close_r = 10f32.powf(close / 10.0);
        let guard = self.cfg.guard_bins;
        let mut runs: Vec<(usize, usize)> = Vec::new();
        let mut cur: Option<(usize, usize)> = None;
        let mut gap = 0usize;
        for i in self.bin_lo..=self.bin_hi {
            let hot = self.ratio[i] >= close_r;
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
        if self.frame_saturated && !runs.is_empty() {
            let peak = |r: &(usize, usize)| (r.0..=r.1).map(|i| self.ratio[i]).fold(0.0f32, f32::max);
            let best = runs.iter().copied().max_by(|a, b| peak(a).total_cmp(&peak(b))).unwrap();
            let top = peak(&best);
            let pair_bins = (self.cfg.pair_hz / self.bin_hz()).round() as usize;
            runs.retain(|r| {
                let gap = if r.0 > best.1 { r.0 - best.1 } else { best.0.saturating_sub(r.1) };
                peak(r) * 10f32.powf(SATURATED_PAIR_DB / 10.0) >= top && gap <= pair_bins
            });
        }
        let extent_db = if self.frame_saturated { SATURATED_EXTENT_DB } else { self.cfg.extent_db };

        for (lo, hi) in runs {
            let mut peak_r = 0.0f32;
            let mut raw_sum = 0.0f32;
            let mut peak_bin = lo;
            let mut peak_w = -1.0f64;
            let mut peak_raw_w = 0.0f64;
            for i in lo..=hi {
                peak_r = peak_r.max(self.ratio[i]);
                raw_sum += self.raw_ratio[i];
                let w = self.excess(i);
                if w > peak_w {
                    peak_w = w;
                    peak_bin = i;
                }
                let raw_w = (self.floor_lin[i] * (self.raw_ratio[i] - 1.0)).max(0.0) as f64;
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
                if self.excess(i) >= floor_w {
                    a = a.min(i);
                    b = b.max(i);
                }
            }
            // The run of a saturated frame is the whole band; matched on
            // that, the source would take every later run in the span.
            let (lo, hi) = if self.frame_saturated {
                (a.saturating_sub(guard).max(lo), (b + guard).min(hi))
            } else {
                (lo, hi)
            };
            let mut wsum = 0.0f64;
            let mut w_all = 0.0f64;
            for i in a..=b {
                let w = self.excess(i);
                wsum += w * i as f64;
                w_all += w;
            }
            let centroid = if w_all > 0.0 { wsum / w_all } else { (a + b) as f64 / 2.0 };
            self.segs.push(Segment { lo, hi, occ_lo: a, occ_hi: b, peak_db, raw_db, centroid });
        }
    }

    /// Match this frame's runs to the sources being followed, open the
    /// candidates that have lasted, and close the sources that have not.
    fn track(&mut self) {
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
                if best.map_or(true, |(_, o)| overlap > o) {
                    best = Some((ti, overlap));
                }
            }
            assigned[si] = best.map(|(ti, _)| ti);
        }

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
                let bins = t.occ_lo..=t.occ_hi.min(n - 1);
                let count = bins.clone().count().max(1) as f32;
                let mean = bins.map(|i| self.raw_ratio[i]).sum::<f32>() / count;
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
        let born = self.tracks.len();
        for (si, s) in self.segs.iter().enumerate() {
            if assigned[si].is_some() || s.peak_db < self.cfg.open_db {
                continue;
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
                centres: [0.0; GROWTH_FRAMES],
                seen_frames: 0,
                centroid_sum: s.centroid,
                centroid_n: 1,
                matched: true,
                faint: false,
            });
        }

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
                    let centre = (t.occ_lo + t.occ_hi) as f64 / 2.0;
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
                    if sweeping && width > t.opened_hz * regrow + 2.0 * bin_hz {
                        t.seen_frames = 0;
                        events.push(SourceEvent::Superseded(t.src));
                        t.src.id = SourceId(*next_id);
                        *next_id += 1;
                        t.src.center_hz = (t.src.lo_hz + t.src.hi_hz) / 2.0;
                        t.opened_hz = t.src.bandwidth_hz();
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
                    if fits && t.hits >= min_frames && (moved || t.born >= fixture_until) {
                        t.open = true;
                        let c = t.centroid_sum / t.centroid_n.max(1) as f64;
                        t.src.center_hz = (c + 0.5 - (n / 2) as f64) * bin_hz;
                        t.opened_hz = t.src.bandwidth_hz();
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
    }

}

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

fn frame_start(frame: u64, hop: u64) -> u64 {
    frame * hop
}

/// One source's extraction: a mixer and two decimators with a cursor into
/// the ring.
///
/// Two stages because one cannot be both cheap and sharp. The final filter
/// has to stop just past the signal's edge, or noise from there out to the
/// output Nyquist and its alias reach the demodulator: measured on the Fine
/// Offset recording that cost 5 dB against a channel bank, the difference
/// between decoding and not. A filter that sharp designed at the input rate
/// runs to tens of thousands of taps at 20 MS/s. So a coarse stage with a
/// wide transition brings the rate down first, and the sharp one runs at a
/// rate where it is a couple of hundred taps.
struct Chan {
    id: SourceId,
    center_hz: u64,
    bandwidth_hz: f64,
    signal_hz: f64,
    out_rate: f64,
    mixer: Mixer,
    /// Coarse stage, absent when the total decimation is small.
    coarse: Option<FirDecim>,
    fir: FirDecim,
    /// Wideband index of the next sample to extract.
    cursor: u64,
    /// Wideband index to stop at, once the source has closed.
    end: Option<u64>,
    /// Closed because a wider stream took over: the last block says so.
    superseded: bool,
    opened: bool,
    snr_db: f32,
    mixed: Vec<C32>,
    out: Vec<C32>,
}

impl Chan {
    /// Mix and decimate a run of wideband samples, appending to `out`.
    fn extract(&mut self, input: &[C32], out: &mut Vec<C32>) {
        self.mixed.clear();
        self.mixer.process(input, &mut self.mixed);
        match &mut self.coarse {
            Some(c) => {
                let mut mid = Vec::with_capacity(self.mixed.len() / c.factor() + 1);
                c.process(&self.mixed, &mut mid);
                self.fir.process(&mid, out);
            }
            None => self.fir.process(&self.mixed, out),
        }
    }
}

/// A decimator whose stopband starts just past the passband, so the noise
/// beside a signal stops there too, rather than at the output Nyquist.
///
/// Bounded at 1024 taps: past that the transition is widened instead, which
/// trades a little noise for a filter that still runs.
fn sharp_decimator(rate: f64, factor: usize, passband_hz: f64, atten_db: f64) -> FirDecim {
    let out_rate = rate / factor as f64;
    let pb = passband_hz.min(out_rate * 0.45);
    let mut stop = (pb * 1.35).min(out_rate - pb).max(pb * 1.05);
    let mut taps = fir::estimate_taps(((stop - pb) / rate).max(1e-4), atten_db);
    if taps > 1024 {
        taps = 1024;
        // Roughly, taps scale with the reciprocal of the transition width.
        let transition = fir::estimate_taps(1e-4, atten_db) as f64 * 1e-4 / 1024.0;
        stop = (pb + transition * rate).min(out_rate - pb).max(pb * 1.05);
    }
    let cutoff = (pb + (stop - pb) * 0.5) / rate;
    FirDecim::new(fir::lowpass(taps, cutoff, atten_db), factor)
}

/// Turns the detector's sources into streams, from a ring of the wideband
/// input.
pub struct SourceExtractor {
    cfg: SourceConfig,
    rate: f64,
    center_hz: f64,
    ring: Vec<C32>,
    /// Wideband index of `ring[0]`.
    base: u64,
    /// Samples the ring keeps behind the newest block.
    keep: usize,
    lead: u64,
    tail: u64,
    chans: Vec<Chan>,
}

impl SourceExtractor {
    /// `center_hz` is the RF centre of the wideband stream, which is what
    /// the blocks are stamped relative to. `keep` is the detector's
    /// [`SourceDetector::latency_samples`], the furthest back an opening can
    /// refer to.
    pub fn new(rate: f64, center_hz: f64, keep: usize, cfg: SourceConfig) -> Self {
        let lead = (cfg.lead_us as f64 * 1e-6 * rate) as u64;
        let tail = (cfg.tail_us as f64 * 1e-6 * rate) as u64;
        Self {
            cfg,
            rate,
            center_hz,
            ring: Vec::new(),
            base: 0,
            // Plus room for the filters to be primed from before the
            // lead-in; a longer prime than this falls back on the lead-in.
            // And the history a reopened source starts again from.
            keep: (keep + lead as usize + 4096).max((cfg.history_s * rate) as usize),
            lead,
            tail,
            chans: Vec::new(),
        }
    }

    /// Sources being extracted, open or draining their tail.
    pub fn active(&self) -> usize {
        self.chans.len()
    }

    pub fn reset(&mut self) {
        self.ring.clear();
        self.base = 0;
        self.chans.clear();
    }

    /// Design an extraction for a source: its centre, and a rate that fits
    /// its width with room for the edges the detector did not see.
    fn open(&mut self, s: &Source) {
        let bw = (s.bandwidth_hz() * self.cfg.width_margin).max(self.cfg.bin_hz * 2.0);
        let want = (bw * self.cfg.oversample).max(self.cfg.min_rate_hz);
        // Whether the rate came from the width or from the floor, which
        // decides what the extraction filter should keep; see below.
        let floored = bw * self.cfg.oversample < self.cfg.min_rate_hz;
        let total = ((self.rate / want).floor() as usize).max(1);
        // Coarse by as much as leaves the sharp stage a few times the width
        // to work in, and only when there is enough decimation to share.
        let f1 = ((self.rate / (bw * 6.0)).floor() as usize).clamp(1, total);
        let (f1, f2) = if f1 >= 2 && total / f1 >= 1 { (f1, total / f1) } else { (1, total) };
        let rate1 = self.rate / f1 as f64;
        let out_rate = rate1 / f2 as f64;
        // Normally the measured half-width: the stream is cut out around the
        // signal and its neighbours are filtered away. A narrow source is
        // different, because its rate comes from the floor rather than from
        // its width, and there the measurement is the wrong thing to filter
        // to. What is measured is the bins within `extent_db` of the peak,
        // which for a clean 12.5 kHz channel can be the two-bin minimum: a
        // 2 kHz passband over a 25 kHz stream cut the sidebands off an M17
        // transmission and left a demodulator that could see the carrier and
        // read nothing from it. When the rate is at the floor the stream is
        // wider than the signal asked for, so it is filled.
        let pb = if floored { (out_rate * 0.4).max(bw / 2.0) } else { bw / 2.0 };
        let coarse =
            (f1 > 1).then(|| FirDecim::design_hz(self.rate, f1, pb, self.cfg.atten_db));
        let fir = sharp_decimator(rate1, f2, pb, self.cfg.atten_db);
        let start = s.start_sample.saturating_sub(self.lead).max(self.base);
        self.chans.push(Chan {
            id: s.id,
            center_hz: (self.center_hz + s.center_hz).max(0.0) as u64,
            bandwidth_hz: bw.min(out_rate),
            signal_hz: s.bandwidth_hz(),
            out_rate,
            mixer: Mixer::new(-s.center_hz, self.rate),
            coarse,
            fir,
            cursor: start,
            end: None,
            superseded: false,
            opened: false,
            snr_db: s.peak_snr_db,
            mixed: Vec::new(),
            out: Vec::new(),
        });
    }

    /// Append a block and the detector's verdict on it, and produce a block
    /// per source being extracted.
    ///
    /// `input` must be the same samples the detector was just given, in the
    /// same order, so the indices in its events land in this ring.
    pub fn process(&mut self, input: &[C32], events: &[SourceEvent], out: &mut Vec<SourceBlock>) {
        self.ring.extend_from_slice(input);
        let end = self.base + self.ring.len() as u64;

        for e in events {
            match e {
                SourceEvent::Opened(s) => self.open(s),
                SourceEvent::Closed(s) => {
                    if let Some(c) = self.chans.iter_mut().find(|c| c.id == s.id) {
                        c.snr_db = s.peak_snr_db;
                        c.end = Some(s.end_sample.unwrap_or(end) + self.tail);
                    }
                }
                SourceEvent::Superseded(s) => {
                    // Ends now, with no tail: what follows belongs to the
                    // wider stream.
                    if let Some(c) = self.chans.iter_mut().find(|c| c.id == s.id) {
                        c.end = Some(c.cursor);
                        c.superseded = true;
                    }
                }
            }
        }

        let base = self.base;
        let ring = &self.ring;
        let blocks: Vec<SourceBlock> = self
            .chans
            .par_iter_mut()
            .filter_map(|c| {
                let stop = c.end.map_or(end, |e| e.min(end));
                let from = c.cursor.max(base);
                let state = if !c.opened {
                    SourceState::Opened
                } else if c.end.is_some_and(|e| stop >= e) {
                    if c.superseded { SourceState::Superseded } else { SourceState::Closed }
                } else {
                    SourceState::Running
                };
                c.out.clear();
                if !c.opened {
                    // Prime the filter so the stream does not open with its
                    // transient. A fresh filter's first output is the first
                    // sample through an empty history, near zero whatever
                    // the input, and a gate downstream that seeds its noise
                    // estimate from the first sample it sees then has a
                    // floor of nothing and opens on the noise that follows.
                    // Whatever the ring holds before the lead-in is real
                    // noise; failing that, the lead-in itself, twice.
                    let taps = (c.fir.taps() * c.coarse.as_ref().map_or(1, |f| f.factor())
                        + c.coarse.as_ref().map_or(0, |f| f.taps())) as u64;
                    let (p0, p1) = if from > base + taps {
                        (from - taps, from)
                    } else {
                        (from, (from + taps).min(stop))
                    };
                    if p1 > p0 {
                        let mut discard = Vec::new();
                        c.extract(&ring[(p0 - base) as usize..(p1 - base) as usize], &mut discard);
                    }
                }
                if stop > from {
                    let slice = &ring[(from - base) as usize..(stop - base) as usize];
                    let mut out = std::mem::take(&mut c.out);
                    c.extract(slice, &mut out);
                    c.out = out;
                }
                let start_sample = from;
                c.cursor = stop.max(c.cursor);
                if c.out.is_empty() && state == SourceState::Running {
                    return None;
                }
                c.opened = true;
                Some(SourceBlock {
                    id: c.id,
                    state,
                    center_hz: c.center_hz,
                    bandwidth_hz: c.bandwidth_hz,
                    signal_hz: c.signal_hz,
                    rate: c.out_rate,
                    start_sample,
                    snr_db: c.snr_db,
                    samples: std::mem::take(&mut c.out),
                })
            })
            .collect();

        let closed: Vec<SourceId> = blocks
            .iter()
            .filter(|b| matches!(b.state, SourceState::Closed | SourceState::Superseded))
            .map(|b| b.id)
            .collect();
        self.chans.retain(|c| !closed.contains(&c.id));
        out.extend(blocks);

        // Nothing still refers to anything older than the newest `keep`
        // samples, and every cursor is at the ring's end. Trimmed only once
        // the ring holds twice that, so the shift of what is kept happens
        // once per history length rather than once per block: at 20 MS/s
        // the history is six million samples, and moving it down every
        // block was the whole cost of an empty band.
        if self.ring.len() >= 2 * self.keep {
            let drop = self.ring.len() - self.keep;
            self.ring.drain(..drop);
            self.base += drop as u64;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn noise(n: usize, amp: f32, seed: u64) -> Vec<C32> {
        // xorshift, Box-Muller: deterministic Gaussian noise.
        let mut s = seed.max(1);
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 11) as f64 / (1u64 << 53) as f64
        };
        (0..n)
            .map(|_| {
                let u1 = next().max(1e-12);
                let u2 = next();
                let r = (-2.0 * u1.ln()).sqrt();
                let th = std::f64::consts::TAU * u2;
                C32::new((r * th.cos()) as f32 * amp, (r * th.sin()) as f32 * amp)
            })
            .collect()
    }

    fn tone(n: usize, hz: f64, rate: f64, amp: f32) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let ph = std::f64::consts::TAU * hz * i as f64 / rate;
                C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32)
            })
            .collect()
    }

    /// A phase-keyed carrier at `hz`, `baud` symbols a second: a TETRA
    /// downlink's shape, which is what the bare tone that `tone` gives is
    /// not. The envelope is constant and the spectrum moves, and both
    /// matter. A detector tells a transmission from the receiver's own
    /// leakage by the spectrum moving, and a carrier whose envelope fades
    /// would close and reopen on its own fades rather than on the air.
    fn modulated(n: usize, hz: f64, baud: f64, rate: f64, amp: f32, seed: u64) -> Vec<C32> {
        let hold = (rate / baud).round().max(1.0) as usize;
        let mut state = seed | 1;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut out = Vec::with_capacity(n);
        let mut phase = 0.0f64;
        let mut step = 0.0f64;
        for i in 0..n {
            if i % hold == 0 {
                // Quarter turns, as pi/4-DQPSK keys, and turned into over
                // the symbol rather than jumped. An unshaped jump every few
                // dozen samples is a spectrum tens of times wider than the
                // keying, and the detector rightly reopens it at that width.
                step = std::f64::consts::FRAC_PI_4 * (1.0 + 2.0 * (next() * 4.0).floor())
                    / hold as f64;
            }
            phase += step;
            let ph = std::f64::consts::TAU * hz * i as f64 / rate + phase;
            out.push(C32::new(amp * ph.cos() as f32, amp * ph.sin() as f32));
        }
        out
    }

    const RATE: f64 = 1_000_000.0;

    fn cfg() -> SourceConfig {
        SourceConfig { floor_memory_s: 0.5, ..Default::default() }
    }

    #[test]
    fn the_floor_lands_on_the_noise() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let n = noise(1_000_000, 0.1, 7);
        d.process(&n);
        let snr = d.snr_db();
        let mean = snr.iter().sum::<f32>() / snr.len() as f32;
        assert!(mean.abs() < 1.5, "mean bin SNR on pure noise is {mean} dB, floor is off");
        assert!(d.live().count() == 0, "noise opened a source");
    }

    #[test]
    fn nothing_opens_on_noise() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut opened = 0;
        for chunk in noise(3_000_000, 0.1, 3).chunks(16384) {
            opened += d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Opened(_))).count();
        }
        assert_eq!(opened, 0, "noise alone opened {opened} sources");
    }

    #[test]
    fn a_burst_is_one_source_with_its_frequency_and_extent() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 11);
        // 50 ms of tone at +123 kHz, starting at 400 ms.
        let start = 400_000;
        let t = tone(50_000, 123_000.0, RATE, 1.0);
        for (i, s) in t.iter().enumerate() {
            x[start + i] += *s;
        }
        let mut events = Vec::new();
        for chunk in x.chunks(8192) {
            events.extend_from_slice(d.process(chunk));
        }
        let opened: Vec<_> = events.iter().filter_map(|e| match e {
            SourceEvent::Opened(s) => Some(*s),
            _ => None,
        }).collect();
        assert_eq!(opened.len(), 1, "{events:?}");
        let s = opened[0];
        assert!((s.center_hz - 123_000.0).abs() < 2.0 * d.bin_hz(), "centre {}", s.center_hz);
        assert!(s.lo_hz < 123_000.0 && s.hi_hz > 123_000.0, "{s:?}");
        assert!((s.start_sample as i64 - start as i64).abs() < 2 * d.fft_size() as i64, "start {}", s.start_sample);
        let closed: Vec<_> = events.iter().filter_map(|e| match e {
            SourceEvent::Closed(s) => Some(*s),
            _ => None,
        }).collect();
        assert_eq!(closed.len(), 1, "{events:?}");
        let end = closed[0].end_sample.unwrap();
        assert!((end as i64 - (start + 50_000) as i64).abs() < 2 * d.fft_size() as i64, "end {end}");
    }

    #[test]
    fn two_transmitters_apart_in_frequency_are_two_sources() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 5);
        for (i, (a, b)) in tone(100_000, -200_000.0, RATE, 0.5)
            .iter()
            .zip(tone(100_000, 310_000.0, RATE, 0.5))
            .enumerate()
        {
            x[500_000 + i] += a + b;
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some(s.center_hz),
                _ => None,
            }));
        }
        opened.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(opened.len(), 2, "{opened:?}");
        assert!((opened[0] + 200_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
        assert!((opened[1] - 310_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
    }

    #[test]
    fn a_narrow_source_is_given_the_whole_stream_it_was_cut_at() {
        // A clean narrowband channel measures a couple of bins across, its
        // rate comes from the floor rather than from that measurement, and
        // the extraction must not then filter the stream down to the two
        // bins that were measured. The source is handed to the extractor
        // rather than detected, so the width under test is exactly the one
        // written here: a signal 4 kHz off the centre, which is where M17's
        // outer symbols and a pager's deviation both sit, has to survive.
        let c = cfg();
        let at = 120_000.0;
        let mut x = noise(400_000, 0.05, 17);
        for (i, s) in tone(300_000, at, RATE, 0.4).iter().enumerate() {
            x[50_000 + i] += *s;
        }
        for (i, s) in tone(300_000, at + 4_000.0, RATE, 0.4).iter().enumerate() {
            x[50_000 + i] += *s;
        }
        let src = Source {
            id: SourceId(1),
            lo_hz: at - c.bin_hz,
            hi_hz: at + c.bin_hz,
            center_hz: at,
            start_sample: 50_000,
            end_sample: None,
            peak_snr_db: 30.0,
            frames: 4,
        };
        assert!(src.bandwidth_hz() * c.width_margin * c.oversample < c.min_rate_hz);

        // Opened once the ring holds the samples the source began in, as
        // the detector's own latency arranges on a live stream.
        let mut e = SourceExtractor::new(RATE, 100e6, 40_000, c);
        let mut blocks = Vec::new();
        let ev = [SourceEvent::Opened(src)];
        for (k, chunk) in x.chunks(8192).enumerate() {
            e.process(chunk, if k == 8 { &ev } else { &[] }, &mut blocks);
        }
        let rate = blocks.first().expect("nothing extracted").rate;
        // The floor, not the width, decided the rate: the integer split
        // between the two decimation stages can leave it a little above.
        assert!(rate >= c.min_rate_hz, "rate {rate} under the floor");
        let all: Vec<C32> = blocks.iter().flat_map(|b| b.samples.iter().copied()).collect();
        let body = &all[all.len() / 3..all.len() * 2 / 3];
        // Each tone against its own frequency in the extracted stream. Both
        // were sent at the same level, so filtering to the measured width
        // shows up as one of them arriving quieter than the other.
        let level = |hz: f64| {
            let mut acc = C32::new(0.0, 0.0);
            for (i, s) in body.iter().enumerate() {
                let ph = -std::f64::consts::TAU * hz * i as f64 / rate;
                acc += s * C32::new(ph.cos() as f32, ph.sin() as f32);
            }
            acc.norm() / body.len() as f32
        };
        let ratio = 20.0 * (level(4_000.0) / level(0.0)).log10();
        assert!(ratio > -6.0, "the signal 4 kHz out came through {ratio:.0} dB down");
    }

    #[test]
    fn the_extracted_stream_holds_the_signal_at_baseband() {
        let c = cfg();
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut e = SourceExtractor::new(RATE, 100e6, d.latency_samples(), c);
        let mut x = noise(1_000_000, 0.05, 9);
        let start = 300_000;
        // A tone keyed on and off at 1 kHz: an amplitude-keyed signal a few
        // kHz wide, which is what the extraction has to keep intact. About
        // 40 dB per bin, which is a strong sensor and not a laboratory one.
        for (i, s) in tone(200_000, -87_500.0, RATE, 0.3).iter().enumerate() {
            let on = (i / 500) % 2 == 0;
            if on {
                x[start + i] += *s;
            }
        }
        let mut blocks = Vec::new();
        for chunk in x.chunks(8192) {
            let ev = d.process(chunk).to_vec();
            e.process(chunk, &ev, &mut blocks);
        }
        assert!(!blocks.is_empty(), "nothing extracted");
        let ids: std::collections::BTreeSet<_> = blocks.iter().map(|b| b.id).collect();
        assert_eq!(ids.len(), 1, "one source, got {ids:?}");
        let states: Vec<_> = blocks.iter().map(|b| (b.state, b.start_sample, b.samples.len())).collect();
        assert_eq!(blocks.first().unwrap().state, SourceState::Opened, "{states:?}");
        assert_eq!(blocks.last().unwrap().state, SourceState::Closed, "{states:?}");
        // Contiguous: every block starts where the previous one stopped.
        let rate = blocks[0].rate;
        assert!(rate >= c.min_rate_hz, "rate {rate}");
        let factor = (RATE / rate).round() as u64;
        let mut expect = blocks[0].start_sample;
        let mut all = Vec::new();
        for b in &blocks {
            // Within a decimation step: the decimator keeps its own phase,
            // so a block's count is a floor or a ceiling of its share.
            let off = b.start_sample as i64 - expect as i64;
            assert!(off.unsigned_abs() < factor, "gap before block at {}", b.start_sample);
            expect = b.start_sample + b.samples.len() as u64 * factor;
            all.extend_from_slice(&b.samples);
        }
        // The lead-in is noise and the keyed tone sits at DC after that:
        // its residual frequency is near zero and its envelope alternates.
        let lead = (c.lead_us as f64 * 1e-6 * rate) as usize;
        let body_len = (100_000.0 * rate / RATE) as usize;
        let body = &all[lead + 1000..lead + body_len];
        let mut acc = C32::new(0.0, 0.0);
        for w in body.windows(2) {
            if w[0].norm() > 0.15 && w[1].norm() > 0.15 {
                acc += w[1] * w[0].conj();
            }
        }
        let residual_hz = acc.arg() as f64 / std::f64::consts::TAU * rate;
        let b0 = &blocks[0];
        // The centre is a centroid over bins, so it is good to a fraction of
        // a bin and no better, and a fraction of a bin is what the width
        // margin exists to cover.
        assert!(
            residual_hz.abs() < d.bin_hz(),
            "signal is {residual_hz} Hz off baseband; centre {} width {} rate {}",
            b0.center_hz as f64 - 100e6,
            b0.bandwidth_hz,
            b0.rate
        );
        let on = body.iter().filter(|s| s.norm() > 0.15).count();
        let ratio = on as f64 / body.len() as f64;
        assert!((0.35..0.65).contains(&ratio), "keying lost, on ratio {ratio}");
        assert!(RATE / rate >= 2.0, "a 4 kHz signal came out at {rate}, no decimation");
        assert_eq!(e.active(), 0, "the source was dropped once closed");
    }

    #[test]
    fn a_continuous_carrier_stays_open() {
        let c = SourceConfig { floor_memory_s: 5.0, ..cfg() };
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut x = noise(2_000_000, 0.05, 13);
        for (i, s) in tone(1_800_000, 50_000.0, RATE, 0.3).iter().enumerate() {
            x[200_000 + i] += *s;
        }
        let mut closed = 0;
        for chunk in x.chunks(8192) {
            closed += d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Closed(_))).count();
        }
        assert_eq!(closed, 0);
        assert_eq!(d.live().count(), 1);
    }

    #[test]
    fn a_carrier_already_on_when_the_stream_starts_is_found() {
        // A TETRA base station downlink is on before the receiver tunes to
        // it and stays on. Nothing in the stream is that bin without it, so
        // minimum statistics have nothing to measure and the bins beside it
        // are the only thing that says what the noise is.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(4_000_000, 0.05, 31);
        for (i, s) in modulated(4_000_000, 50_000.0, 18_000.0, RATE, 0.3, 32).iter().enumerate() {
            x[i] += *s;
        }
        let mut opened = 0;
        for chunk in x.chunks(8192) {
            opened += d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Opened(_))).count();
        }
        assert!(opened > 0, "a permanent carrier never opened a source");
    }

    #[test]
    fn a_carrier_outlasting_the_floor_memory_stays_open() {
        // Same detector, carrier starting after the floor has been learned
        // and running far longer than the memory.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(5_000_000, 0.05, 33);
        for (i, s) in modulated(4_000_000, 50_000.0, 18_000.0, RATE, 0.3, 34).iter().enumerate() {
            x[1_000_000 + i] += *s;
        }
        let mut opened = 0;
        let mut closed = 0;
        for chunk in x.chunks(8192) {
            for e in d.process(chunk) {
                match e {
                    SourceEvent::Opened(_) => opened += 1,
                    SourceEvent::Closed(_) => closed += 1,
                    _ => {}
                }
            }
        }
        assert_eq!(opened, 1, "opened {opened}");
        assert_eq!(closed, 0, "the carrier was learned as floor and closed under it");
    }

    #[test]
    fn leading_silence_does_not_become_the_floor() {
        // rtl_433's captures open with a run of zeros while the tuner
        // settles. A zero in the minimum is a floor of nothing.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = vec![C32::new(0.0, 0.0); 50_000];
        x.extend(noise(950_000, 0.05, 21));
        for (i, s) in tone(50_000, 80_000.0, RATE, 0.3).iter().enumerate() {
            x[500_000 + i] += *s;
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some((s.center_hz, s.peak_snr_db)),
                _ => None,
            }));
        }
        assert_eq!(opened.len(), 1, "{opened:?}");
        assert!((opened[0].0 - 80_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
        assert!(opened[0].1 < 60.0, "an SNR of {} dB means the floor is nothing", opened[0].1);
    }

    #[test]
    fn a_settling_tuner_s_constant_is_silence_too() {
        // Byte value zero in a cu8 is minus one on both rails: a full-scale
        // constant, which is a carrier at DC and an empty floor everywhere
        // else.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = vec![C32::new(-1.0, -1.0); 250_000];
        x.extend(noise(750_000, 0.05, 23));
        for (i, s) in tone(50_000, 80_000.0, RATE, 0.3).iter().enumerate() {
            x[600_000 + i] += *s;
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some((s.center_hz, s.peak_snr_db)),
                _ => None,
            }));
        }
        assert_eq!(opened.len(), 1, "{opened:?}");
        assert!((opened[0].0 - 80_000.0).abs() < 2.0 * d.bin_hz(), "{opened:?}");
        assert!(opened[0].1 < 60.0, "an SNR of {} dB means the floor is nothing", opened[0].1);
    }

    #[test]
    fn the_two_tones_of_an_fsk_burst_are_one_source() {
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 31);
        // 17 kbit/s two-tone keying, tones 120 kHz apart, as a LaCrosse
        // sensor does it.
        let (f0, f1) = (-72_000.0, 48_000.0);
        let mut ph = 0.0f64;
        for i in 0..20_000usize {
            let bit = (i / 58) % 3 == 0;
            let f = if bit { f1 } else { f0 };
            ph += std::f64::consts::TAU * f / RATE;
            x[500_000 + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
        }
        let mut opened = Vec::new();
        for chunk in x.chunks(8192) {
            opened.extend(d.process(chunk).iter().filter_map(|e| match e {
                SourceEvent::Opened(s) => Some(*s),
                _ => None,
            }));
        }
        assert_eq!(opened.len(), 1, "{opened:?}");
        let s = opened[0];
        assert!(s.lo_hz < f0 && s.hi_hz > f1, "extent {}..{} misses a tone", s.lo_hz, s.hi_hz);
        let mid = (f0 + f1) / 2.0;
        assert!((s.center_hz - mid).abs() < 4.0 * d.bin_hz(), "centre {} for tones at {f0} and {f1}", s.center_hz);
    }

    #[test]
    fn a_slow_chirp_is_reopened_at_its_full_width() {
        // 200 kHz swept in 40 ms, over and over: a symbol of chirp spread
        // spectrum at a high spreading factor. In the frames it takes to
        // open it is a tone a few kilohertz wide.
        let c = cfg();
        let mut d = SourceDetector::new(RATE, RATE, c);
        let mut e = SourceExtractor::new(RATE, 100e6, d.latency_samples(), c);
        let mut x = noise(1_000_000, 0.05, 41);
        let start = 300_000usize;
        let mut ph = 0.0f64;
        for i in 0..200_000usize {
            let t = (i % 40_000) as f64 / 40_000.0;
            let f = -100_000.0 + 200_000.0 * t;
            ph += std::f64::consts::TAU * f / RATE;
            x[start + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
        }
        let mut events = Vec::new();
        let mut blocks = Vec::new();
        for chunk in x.chunks(8192) {
            let ev = d.process(chunk).to_vec();
            events.extend_from_slice(&ev);
            e.process(chunk, &ev, &mut blocks);
        }
        let opened: Vec<Source> = events.iter().filter_map(|e| match e {
            SourceEvent::Opened(s) => Some(*s),
            _ => None,
        }).collect();
        assert!(opened.len() >= 2, "never reopened: {events:?}");
        let last = opened.last().unwrap();
        assert!(last.bandwidth_hz() > 150_000.0, "final width {}", last.bandwidth_hz());
        assert!(events.iter().any(|e| matches!(e, SourceEvent::Superseded(_))));
        // The wide stream starts where the transmitter did, not where it was
        // noticed to be wide.
        let wide: Vec<&SourceBlock> = blocks.iter().filter(|b| b.id == last.id).collect();
        assert!(!wide.is_empty());
        assert!(wide[0].start_sample < start as u64 + 20_000, "restarted at {}", wide[0].start_sample);
        assert!(wide[0].rate >= 375_000.0, "rate {}", wide[0].rate);
        let old: Vec<&SourceBlock> = blocks.iter().filter(|b| b.id == opened[0].id).collect();
        assert_eq!(old.last().unwrap().state, SourceState::Superseded);
        // And the reopened stream is one stream: every block starts where
        // the one before it stopped.
        let factor = (RATE / wide[0].rate).round() as u64;
        let mut expect = wide[0].start_sample;
        for b in &wide {
            let off = b.start_sample as i64 - expect as i64;
            assert!(off.unsigned_abs() < factor, "reopened stream jumps at {} (expected {expect})", b.start_sample);
            expect = b.start_sample + b.samples.len() as u64 * factor;
        }
    }

    #[test]
    fn a_spur_one_bin_wide_opens_nothing() {
        // A bare tone: the tuner's leakage or a bare oscillator. Nothing
        // keyed is this narrow, and a carrier reported every half second
        // for as long as the receiver runs is a list of nothing.
        let mut d = SourceDetector::new(RATE, RATE, cfg());
        let mut x = noise(1_000_000, 0.05, 43);
        // Exactly on a bin centre, so it stays one bin wide.
        let hz = 64.0 * d.bin_hz();
        for (i, s) in tone(1_000_000, hz, RATE, 0.3).iter().enumerate() {
            x[i] += *s;
        }
        let mut opened = 0;
        for chunk in x.chunks(8192) {
            opened += d.process(chunk).iter().filter(|e| matches!(e, SourceEvent::Opened(_))).count();
        }
        assert_eq!(opened, 0, "a one-bin spur opened a source");
    }

    #[test]
    fn floor_bias_is_modest_for_smoothed_power() {
        // Sanity on the derivation: a few frames of smoothing over a few
        // hundred frames of window is a correction of a few dB, not ten.
        let b = floor_bias(0.25, 1024);
        let db = 10.0 * b.log10();
        assert!((1.0..8.0).contains(&db), "{db} dB");
    }
}



