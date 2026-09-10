//! Analogue television baseband: sync separation and a picture out of it.
//!
//! Analogue video is not a protocol. A camera produces composite video, a
//! transmitter frequency modulates a carrier with it, and that is the whole
//! stack: no framing, no addressing, no integrity check anywhere. What
//! carries structure is the video itself, and it is the structure television
//! had in 1960: a horizontal sync pulse below black at the start of every
//! line, a pattern of broad pulses between fields, and two interlaced fields
//! to a frame. A model aircraft's video link, a security camera and a bench
//! pattern generator all produce the same thing; what differs is the band it
//! is sent on.
//!
//! So this file takes the output of an FM demodulator and finds those pulses.
//! Levels are relative, because the demodulator's output scale depends on the
//! deviation it was told and a transmitter's deviation is whatever its
//! designer chose: sync tip and blanking are measured from the signal itself
//! rather than assumed, the way `crate::ble`'s gate tracks its own floor.
//!
//! # Colour
//!
//! Optional, and off unless asked for. PAL puts chrominance on a 4.43 MHz
//! subcarrier in quadrature, U on one axis and V on the other, and flips the
//! sign of V every line: that is what the P and the A stand for, and it is
//! what makes a phase error tint alternate lines in opposite directions so
//! the eye averages it away. The receiver does the averaging properly, with
//! a delay line: the U and V of a line are meaned with the line above.
//!
//! The reference is the burst on the back porch, ten cycles at 135 or 225
//! degrees. One burst alone cannot say which, since a phase error and a line
//! flip look the same; the mean of two consecutive bursts is the -U axis,
//! which is why the phase is estimated over a pair.
//!
//! Not here: audio. Most transmitters put it on a 6.0 or 6.5 MHz subcarrier,
//! which is another demodulator on this same baseband. The line spectrum of
//! the capture in `testdata` has it plainly at 6.5 MHz.

use crate::fir::{FirDecim, FirDecimReal};
use crate::FmDemod;
use common::C32;

/// Which set of timings the camera is using.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Standard {
    /// 625 lines, 25 frames, 15.625 kHz line rate. What almost every camera
    /// sold outside North America produces.
    Pal,
    /// 525 lines, 29.97 frames, 15.734 kHz line rate.
    Ntsc,
}

impl Standard {
    pub fn line_hz(self) -> f64 {
        match self {
            Self::Pal => 15_625.0,
            Self::Ntsc => 15_734.264,
        }
    }

    /// Lines in a whole frame, both fields, including the ones with no
    /// picture in them.
    pub fn lines(self) -> usize {
        match self {
            Self::Pal => 625,
            Self::Ntsc => 525,
        }
    }

    /// The shape of the picture, which is not the shape of the sample grid.
    ///
    /// Both standards are 4:3. How many samples a line is cut into is a
    /// property of the receiver's clock and says nothing about what the
    /// camera saw: at 20 MS/s a 640 by 288 field drawn from its own numbers
    /// comes out at 10:9, which is a picture squeezed in from the sides.
    pub fn aspect(self) -> f32 {
        4.0 / 3.0
    }

    /// Picture lines in one field.
    pub fn active_lines(self) -> usize {
        match self {
            Self::Pal => 288,
            Self::Ntsc => 240,
        }
    }

    /// Seconds of horizontal sync at the start of a line.
    pub fn sync_s(self) -> f64 {
        match self {
            Self::Pal => 4.7e-6,
            Self::Ntsc => 4.7e-6,
        }
    }

    /// Seconds from the sync edge to the start of the picture.
    pub fn back_porch_s(self) -> f64 {
        match self {
            Self::Pal => 5.7e-6,
            Self::Ntsc => 4.5e-6,
        }
    }

    /// Seconds of picture on a line.
    pub fn active_s(self) -> f64 {
        match self {
            Self::Pal => 52.0e-6,
            Self::Ntsc => 52.6e-6,
        }
    }

    pub fn line_s(self) -> f64 {
        1.0 / self.line_hz()
    }

    /// The colour subcarrier.
    pub fn subcarrier_hz(self) -> f64 {
        match self {
            Self::Pal => 4_433_618.75,
            Self::Ntsc => 3_579_545.45,
        }
    }

    /// The standard a measured line period names, or `None` when it is
    /// neither. The two are 0.7% apart, which is far wider than a
    /// transmitter's timebase error, so measuring settles it.
    pub fn from_line_period(period_s: f64) -> Option<Self> {
        [Self::Pal, Self::Ntsc]
            .into_iter()
            .find(|s| (period_s / s.line_s() - 1.0).abs() < 0.003)
    }

    /// What a setting or a menu calls it, and what [`FromStr`] reads back.
    pub fn label(self) -> &'static str {
        match self {
            Self::Pal => "pal",
            Self::Ntsc => "ntsc",
        }
    }
}

impl std::str::FromStr for Standard {
    type Err = common::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.trim().to_ascii_lowercase().as_str() {
            "pal" => Ok(Self::Pal),
            "ntsc" => Ok(Self::Ntsc),
            other => Err(common::Error::other(format!(
                "no video standard called {other:?}"
            ))),
        }
    }
}

impl std::fmt::Display for Standard {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

/// Whether a demodulated baseband is analogue video, and which standard.
///
/// The positive test, rather than "try to decode it and see". A camera keys a
/// sync pulse of about 4.7 us at the start of every line, so the gaps between
/// those pulses cluster hard at the line period: 64 us for PAL, 63.55 for
/// NTSC, the two 0.7% apart, which no transmitter's timebase error reaches.
/// Noise and every other modulation here give gaps scattered across the
/// range instead.
///
/// So the measurement is the median gap, and the evidence is how many of the
/// gaps agree with it. `agreement` is that fraction, and it separates the two
/// cases a median alone cannot: a real camera puts nearly every gap within a
/// percent of the median, while a burst that happens to have two pulses the
/// right distance apart puts almost none there.
///
/// The samples must be filtered first, exactly as [`SyncSeparator`] filters:
/// a discriminator on a 20 MHz span carries all of that bandwidth's noise and
/// a single sample above the threshold ends a run that has to last 4.7 us.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lock {
    pub standard: Standard,
    /// Sync pulses found.
    pub pulses: usize,
    /// Fraction of the gaps between them within one percent of the median.
    pub agreement: f32,
    /// Lines the gaps account for, against the lines the window holds.
    ///
    /// What decides a lock. See [`find_lines`] for why it is not
    /// `agreement`.
    pub coverage: f32,
}

/// What the line test made of a window, for a tool asking why a capture did
/// not lock. The same arithmetic as [`find_lines`], reported rather than
/// reduced to a yes or a no.
#[derive(Clone, Copy, Debug, Default)]
pub struct LockAttempt {
    /// Sync tip and blanking percentiles, and the slice between them.
    pub tip: f32,
    pub black: f32,
    /// Runs of the right length found.
    pub pulses: usize,
    /// Median gap between them, in seconds.
    pub median_s: f64,
    /// Fraction of gaps within a percent of the median.
    pub agreement: f32,
    /// Lines those gaps account for, against the lines the window holds.
    pub coverage: f32,
    pub standard: Option<Standard>,
}

/// Look for a line rate in demodulated baseband, and say what was found.
pub fn examine_lines(baseband: &[f32], rate: f64) -> LockAttempt {
    let mut a = LockAttempt::default();
    let take = ((0.04 * rate) as usize).min(baseband.len());
    let base = &baseband[..take];
    let taps = ((0.25e-6 * rate) as usize).max(1);
    if base.len() < taps * 4 {
        return a;
    }
    let mut sorted: Vec<f32> = base.to_vec();
    sorted.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    a.tip = sorted[sorted.len() / 50];
    a.black = sorted[sorted.len() * 13 / 100];
    if a.black <= a.tip {
        return a;
    }
    let thresh = (a.tip + a.black) / 2.0;
    let (mut edges, mut low) = (Vec::new(), 0usize);
    let (lo, hi) = ((2e-6 * rate) as usize, (8e-6 * rate) as usize);
    let mut sum: f32 = base[..taps].iter().sum();
    for i in 0..base.len() - taps {
        let mean = sum / taps as f32;
        sum += base[i + taps] - base[i];
        if mean < thresh {
            low += 1;
        } else {
            if (lo..=hi).contains(&low) {
                edges.push(i);
            }
            low = 0;
        }
    }
    a.pulses = edges.len();
    if edges.len() < 2 {
        return a;
    }
    let mut gaps: Vec<f64> = edges.windows(2).map(|w| (w[1] - w[0]) as f64 / rate).collect();
    gaps.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    a.median_s = gaps[gaps.len() / 2];
    a.standard = Standard::from_line_period(a.median_s);
    let agree = gaps.iter().filter(|g| (*g / a.median_s - 1.0).abs() < 0.01).count();
    a.agreement = agree as f32 / gaps.len() as f32;
    a.coverage = coverage(&gaps, a.median_s, base.len() as f64 / rate);
    a
}

/// How much of a window whole lines account for.
///
/// A gap of one line period is one line, and a gap of `k` of them is `k`
/// lines with `k - 1` sync pulses lost to noise, which is a fade rather than
/// a disagreement. What comes out is the fraction of the window's lines the
/// pulse train explains, and it is the measure a lock is decided on.
fn coverage(gaps: &[f64], median_s: f64, window_s: f64) -> f32 {
    if median_s <= 0.0 || window_s <= 0.0 {
        return 0.0;
    }
    let mut lines = 0.0f64;
    for g in gaps {
        let k = (g / median_s).round();
        if (1.0..=8.0).contains(&k) && (g / (k * median_s) - 1.0).abs() < 0.01 {
            lines += k;
        }
    }
    (lines / (window_s / median_s)) as f32
}

/// Look for a line rate in demodulated baseband.
///
/// `None` when there is no pulse train at either standard's rate, which is
/// what everything that is not video looks like.
pub fn find_lines(baseband: &[f32], rate: f64) -> Option<Lock> {
    // Two fields is enough to see six hundred lines, and more is only more
    // cost on a source that is not video.
    let take = ((0.04 * rate) as usize).min(baseband.len());
    let base = &baseband[..take];
    let taps = ((0.25e-6 * rate) as usize).max(1);
    if base.len() < taps * 4 {
        return None;
    }

    let mut sorted: Vec<f32> = base.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let (tip, black) = (sorted[sorted.len() / 50], sorted[sorted.len() * 13 / 100]);
    // A signal with no sync in it has no gap between those percentiles worth
    // slicing between, and slicing anyway finds runs in the noise.
    if black <= tip {
        return None;
    }
    let thresh = (tip + black) / 2.0;

    let (mut edges, mut low) = (Vec::new(), 0usize);
    let (lo, hi) = ((2e-6 * rate) as usize, (8e-6 * rate) as usize);
    // A running mean over the same quarter microsecond the separator uses,
    // carried rather than resummed: this runs over every sample of a wide
    // source.
    let mut sum: f32 = base[..taps].iter().sum();
    for i in 0..base.len() - taps {
        let mean = sum / taps as f32;
        sum += base[i + taps] - base[i];
        if mean < thresh {
            low += 1;
        } else {
            if (lo..=hi).contains(&low) {
                edges.push(i);
            }
            low = 0;
        }
    }
    // Six hundred lines are on offer in two fields. A hundred is a signal
    // that has been keying steadily for six milliseconds and is still a long
    // way short of a camera.
    if edges.len() < 100 {
        return None;
    }

    let mut gaps: Vec<f64> = edges
        .windows(2)
        .map(|w| (w[1] - w[0]) as f64 / rate)
        .collect();
    gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = gaps[gaps.len() / 2];
    let standard = Standard::from_line_period(median)?;
    let agree = gaps
        .iter()
        .filter(|g| (*g / median - 1.0).abs() < 0.01)
        .count();
    let agreement = agree as f32 / gaps.len() as f32;
    let coverage = coverage(&gaps, median, base.len() as f64 / rate);
    // Decided on coverage rather than on agreement, and measured rather than
    // guessed. Agreement asks what fraction of the gaps are a line long,
    // which noise ruins twice over: a spurious edge inside a line makes two
    // gaps that are neither, and a sync pulse lost to noise makes one gap of
    // two lines that is not one either. On a weak 5.8 GHz camera 40 dB down
    // the band that put agreement at 0.2 to 0.5, below the half this used to
    // ask for, while the median gap was 64.0 us every time: the line rate was
    // never in doubt and the test threw the picture away regardless.
    //
    // Coverage asks instead how much of the window whole lines explain, which
    // a lost pulse does not penalise. The strong AKK capture scores 0.99, the
    // weak one 0.3 to 0.6, and noise scores nothing because its median is at
    // no line period at all.
    (coverage > 0.25).then_some(Lock {
        standard,
        pulses: edges.len(),
        agreement,
        coverage,
    })
}


/// The sound a camera sends beside its picture.
///
/// Analogue video links put audio on an FM subcarrier of the composite
/// baseband: 6.5 MHz on the AKK capture here, at 49.5 dB against 29.5 for the
/// colour burst, and 5.5, 6.0 or 6.8 on other transmitters. It is inside the
/// discriminator's output rather than beside the carrier, so hearing it needs
/// the whole 20 MHz of the transmission and not the 10 the picture is read
/// from: the sound path is why the front end keeps a wideband discriminator
/// after the picture stopped needing one.
///
/// What comes out is at whatever rate the two decimations leave, around
/// 50 kHz, and it is labelled with that: everything downstream of a voice
/// port resamples anyway, and resampling twice is worse than once.
pub struct Sound {
    rate: f64,
    /// The subcarrier, once it has been found. `None` while looking.
    hz: Option<f64>,
    /// Composite kept for the search, which needs a frame of it.
    probe: Vec<f32>,
    /// Phase of the mixer that brings the subcarrier to zero, in turns, kept
    /// across blocks so it does not click every block boundary.
    phase: f64,
    mixed: Vec<C32>,
    down: FirDecim,
    demod: FmDemod,
    fm: Vec<f32>,
    audio: FirDecimReal,
    /// One-pole de-emphasis, and its state.
    deemph: f32,
    last: f32,
    out_rate: f64,
}

/// Candidate subcarriers, in hertz. Every analogue link this author has seen
/// uses one of these; a transmitter using another is heard as silence rather
/// than as noise, which is the right failure.
const SOUND_HZ: [f64; 4] = [5.5e6, 6.0e6, 6.5e6, 6.8e6];

/// How far either side of a candidate the search looks, and the width the
/// mixer's filter keeps: an FM subcarrier at 50 to 100 kHz deviation with
/// 15 kHz of audio on it needs a couple of hundred kilohertz.
const SOUND_WIDTH_HZ: f64 = 220e3;

/// Peak deviation of the subcarrier mapped to full scale.
const SOUND_DEVIATION_HZ: f64 = 100e3;

/// De-emphasis time constant. 50 us is what European FM uses and what these
/// transmitters copy; without it the sound is thin and hissy.
const DEEMPHASIS_S: f64 = 50e-6;

/// Samples between reseeding the mixer's rotation from its exact phase.
///
/// A thousand steps of an f32 rotation drift by about a millionth, which is
/// far inside what a 100 kHz-deviation subcarrier is read to; a whole block
/// of them at 20 MS/s would not be.
const RESEED: usize = 1024;

impl Sound {
    /// A reader for composite at `rate`, or `None` when the stream cannot
    /// hold a subcarrier at all: below about 13 MS/s the 6.5 MHz one is
    /// outside the baseband and there is nothing to hear.
    pub fn new(rate: f64) -> Option<Self> {
        let lowest = SOUND_HZ[0] + SOUND_WIDTH_HZ;
        if rate / 2.0 <= lowest {
            return None;
        }
        // Two stages: the span down to a couple of hundred kilohertz, where
        // the subcarrier is demodulated, then that down to something an
        // audio bus can take. One stage from 20 MS/s to 50 kHz would be a
        // filter of thousands of taps.
        let f1 = (rate / (2.0 * SOUND_WIDTH_HZ)).floor().max(1.0) as usize;
        let r1 = rate / f1 as f64;
        let f2 = (r1 / 44.1e3).floor().max(1.0) as usize;
        let out_rate = r1 / f2 as f64;
        let alpha = 1.0 - (-1.0 / (DEEMPHASIS_S * out_rate)).exp();
        Some(Self {
            rate,
            hz: None,
            probe: Vec::new(),
            phase: 0.0,
            mixed: Vec::new(),
            down: FirDecim::design_hz(rate, f1, SOUND_WIDTH_HZ / 2.0, 60.0),
            demod: FmDemod::new(r1, SOUND_DEVIATION_HZ),
            fm: Vec::new(),
            audio: FirDecimReal::design_hz(r1, f2, 15e3, 60.0),
            deemph: alpha as f32,
            last: 0.0,
            out_rate,
        })
    }

    /// The subcarrier being read, once one has been found.
    pub fn subcarrier_hz(&self) -> Option<f64> {
        self.hz
    }

    /// The rate the audio comes out at.
    pub fn rate(&self) -> f64 {
        self.out_rate
    }

    /// Forget the subcarrier and look again: what to do when the picture has
    /// gone, since the next transmitter may put its sound elsewhere.
    pub fn reset(&mut self) {
        self.hz = None;
        self.probe.clear();
        self.phase = 0.0;
        self.last = 0.0;
    }

    /// Read a block of composite, appending whatever audio came out.
    pub fn process(&mut self, base: &[f32], out: &mut Vec<f32>) {
        let Some(hz) = self.hz.or_else(|| self.look(base)) else {
            return;
        };
        // Mix the subcarrier to zero. The phase runs from a counter kept
        // across blocks: restarting it each block puts a step in the
        // demodulator's output at every boundary, which is a click at the
        // block rate.
        //
        // A rotation rather than a sine a sample, as the chroma reference in
        // [`SyncSeparator::chroma_of`] is: this runs over the whole span, so
        // at 20 MS/s a pair of double-precision trig calls a sample was forty
        // million of them a second and most of what hearing a camera cost.
        // Reseeded from the exact phase every [`RESEED`] samples, since an
        // f32 rotation left to run for a whole block drifts in amplitude as
        // well as in phase.
        let step = hz / self.rate;
        let turn = -std::f64::consts::TAU * step;
        let (dc, ds) = (turn.cos() as f32, turn.sin() as f32);
        self.mixed.clear();
        self.mixed.reserve(base.len());
        let (mut c, mut s) = (0.0f32, 0.0f32);
        for (k, &x) in base.iter().enumerate() {
            if k % RESEED == 0 {
                let a = -std::f64::consts::TAU * (self.phase + step * k as f64).fract();
                c = a.cos() as f32;
                s = a.sin() as f32;
            }
            self.mixed.push(C32::new(x * c, x * s));
            let (nc, ns) = (c * dc - s * ds, s * dc + c * ds);
            c = nc;
            s = ns;
        }
        self.phase = (self.phase + step * base.len() as f64).fract();
        let mixed = std::mem::take(&mut self.mixed);
        let mut narrow = Vec::new();
        self.down.process(&mixed, &mut narrow);
        self.mixed = mixed;
        self.fm.clear();
        self.demod.process(&narrow, &mut self.fm);
        let mut audio = Vec::new();
        self.audio.process(&self.fm, &mut audio);
        for v in &audio {
            self.last += self.deemph * (*v - self.last);
            out.push(self.last);
        }
    }

    /// Which of the candidate subcarriers is there, if any.
    ///
    /// Measured rather than assumed: a transmitter's sound sits at one of a
    /// handful of frequencies and which one is a fact about the unit. The
    /// test is the power in a narrow band around each candidate against the
    /// baseband either side of them, so a camera sending no sound at all is
    /// answered with `None` instead of a band of noise being demodulated
    /// into hiss.
    fn look(&mut self, base: &[f32]) -> Option<f64> {
        self.probe.extend_from_slice(base);
        // A frame of composite. Long enough that a 250 kHz bin holds
        // hundreds of cycles, short enough to decide within a field.
        let want = (0.02 * self.rate) as usize;
        if self.probe.len() < want {
            return None;
        }
        let probe = std::mem::take(&mut self.probe);
        let mut best: Option<(f64, f32)> = None;
        for hz in SOUND_HZ {
            if hz + SOUND_WIDTH_HZ / 2.0 >= self.rate / 2.0 {
                continue;
            }
            let on = band_power(&probe, self.rate, hz, SOUND_WIDTH_HZ);
            // The floor a megahertz below it, which on a camera's baseband
            // is the quiet stretch between the colour burst and the sound.
            let off = band_power(&probe, self.rate, hz - 1.0e6, SOUND_WIDTH_HZ);
            let snr = on / off.max(1e-20);
            if snr > 10.0 && best.is_none_or(|(_, b)| snr > b) {
                best = Some((hz, snr));
            }
        }
        self.hz = best.map(|(hz, _)| hz);
        self.hz
    }
}

/// Power in a band of a real signal, by mixing it down and averaging: a
/// whole transform is not worth it for four candidates.
fn band_power(x: &[f32], rate: f64, hz: f64, width_hz: f64) -> f32 {
    // A moving average of the mixed signal is a lowpass at about `width`,
    // which is all the selectivity this test needs.
    let n = ((rate / width_hz).round() as usize).max(1);
    let step = hz / rate;
    let (mut re, mut im) = (0.0f32, 0.0f32);
    let mut power = 0.0f64;
    let mut taken = 0usize;
    for (i, &v) in x.iter().enumerate() {
        let a = -std::f64::consts::TAU * (i as f64 * step).fract();
        re += v * a.cos() as f32;
        im += v * a.sin() as f32;
        if (i + 1) % n == 0 {
            power += f64::from(re * re + im * im) / (n * n) as f64;
            taken += 1;
            re = 0.0;
            im = 0.0;
        }
    }
    (power / taken.max(1) as f64) as f32
}

/// One field, as luma samples.
#[derive(Clone, Debug)]
pub struct Field {
    pub width: usize,
    pub height: usize,
    /// How wide the picture is against its height when it is drawn, which
    /// the sample grid does not say.
    pub aspect: f32,
    /// Row major, 0 for sync-black and 255 for peak white.
    pub luma: Vec<u8>,
    /// Row major RGB triples, when colour was asked for and the burst was
    /// found. `None` means the field was read as luma only, which is what a
    /// monochrome camera and a lost burst both look like.
    pub rgb: Option<Vec<u8>>,
    /// Lines whose sync was found, out of `height`. A field assembled from
    /// half its lines is a picture of a fade, and a caller deciding whether
    /// to show it needs the number.
    pub lines_seen: usize,
}

/// Sync separator: level tracking, horizontal sync detection, and fields
/// assembled from the lines between vertical pulses.
pub struct SyncSeparator {
    rate: f64,
    standard: Standard,
    width: usize,
    /// Sync tip and blanking level, tracked from the signal.
    sync_level: f32,
    black_level: f32,
    /// Samples since the last horizontal sync edge.
    since_sync: usize,
    /// The current field's lines.
    lines: Vec<Vec<u8>>,
    lines_seen: usize,
    /// Whether the level tracker has seen enough to be trusted.
    primed: bool,
    hist: Vec<f32>,
    /// Samples since a field last completed, which is how a lost picture is
    /// told from a running one.
    since_field: usize,
    /// Samples of a run below the sync threshold, for telling a horizontal
    /// pulse from a vertical one.
    low_run: usize,
    /// Running mean over `smooth_len` samples, and the window behind it.
    ///
    /// A discriminator reading a 20 MHz span gives a video baseband with all
    /// of that bandwidth's noise on it, and the picture occupies 5 MHz of it.
    /// Slicing that unfiltered does not find a sync pulse at all: a single
    /// noisy sample above the threshold ends the run, and the run has to last
    /// 4.7 us. Averaging over a quarter of a microsecond keeps the pulse,
    /// which is nearly twenty times longer, and throws most of the noise
    /// away. Off air this was the difference between no fields and every
    /// field.
    /// Ring of the last `smooth_len` samples, and where the next goes. A
    /// plain array rather than a deque: this runs on every sample of a 20 MS/s
    /// span, and the deque's bookkeeping was most of what the separator cost.
    smooth: Vec<f32>,
    smooth_at: usize,
    smooth_filled: usize,
    smooth_sum: f32,
    smooth_len: usize,
    /// Where the last line started, so the picture can be cut out of it.
    line: Vec<f32>,
    /// The same line unfiltered. The running mean that makes sync findable
    /// has its cutoff below the colour subcarrier, so chroma has to be taken
    /// from the samples as they arrived.
    raw_line: Vec<f32>,
    /// Whether to demodulate chroma at all.
    colour: bool,
    /// Chroma rows of the field being assembled, as (U, V) per pixel.
    chroma: Vec<Vec<(f32, f32)>>,
    /// The reference axis the last line's burst gave, which the next line's
    /// is resolved against.
    ///
    /// A burst is 135 or 225 degrees and one on its own cannot say which, so
    /// the choice is made by continuity: the axis a receiver locks to does
    /// not move between lines, and the polarity that keeps it still is the
    /// right one. This is the delay line's other job.
    last_axis: Option<f32>,
    /// Samples since the separator was built.
    ///
    /// The subcarrier phase is counted from here rather than from the start
    /// of each line, because PAL's subcarrier is continuous across lines and
    /// sync edges land on whole samples: at 20 MS/s one sample of jitter is
    /// 80 degrees of subcarrier, so a per-line origin rotates the colour by
    /// most of a turn from line to line.
    sample: u64,
    /// Where in that count the line being collected started.
    line_start: u64,
    stats: Stats,
}

/// What the separator saw, for telling a signal it cannot lock to from one
/// that is not there.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    pub line_syncs: u64,
    pub broad_pulses: u64,
    pub lines_kept: u64,
    pub fields: u64,
    /// Fields thrown away for holding too few lines.
    pub fields_short: u64,
    pub sync_level: f32,
    pub black_level: f32,
}

impl SyncSeparator {
    /// `width` is the picture width to resample each line to. 720 is what a
    /// PAL line holds at broadcast sampling; a small camera is usually 600 or
    /// 800, and the sampling here is of the demodulated baseband rather than
    /// of the camera's own pixels, so the number is a choice and not a
    /// measurement.
    pub fn new(rate: f64, standard: Standard, width: usize) -> Self {
        Self {
            rate,
            standard,
            width,
            sync_level: 0.0,
            black_level: 0.0,
            since_sync: 0,
            lines: Vec::new(),
            lines_seen: 0,
            primed: false,
            hist: Vec::new(),
            since_field: 0,
            low_run: 0,
            smooth: Vec::new(),
            smooth_at: 0,
            smooth_filled: 0,
            smooth_sum: 0.0,
            smooth_len: ((0.25e-6 * rate).round() as usize).max(1),
            line: Vec::new(),
            raw_line: Vec::new(),
            colour: false,
            chroma: Vec::new(),
            last_axis: None,
            sample: 0,
            line_start: 0,
            stats: Stats::default(),
        }
    }

    pub fn stats(&self) -> Stats {
        Stats {
            sync_level: self.sync_level,
            black_level: self.black_level,
            ..self.stats
        }
    }

    /// Demodulate colour as well as luma. Costs a quadrature demodulation and
    /// two box filters per line.
    pub fn with_colour(mut self) -> Self {
        self.colour = true;
        self
    }

    pub fn standard(&self) -> Standard {
        self.standard
    }

    fn samples(&self, seconds: f64) -> usize {
        (seconds * self.rate).round() as usize
    }

    /// Sync tip and blanking, from the distribution of the signal itself.
    ///
    /// A line is 7.3% sync and another 11% porches, and nothing in a picture
    /// goes below blanking, so the sorted samples have the tip in the bottom
    /// twentieth and blanking around the eighth. Taking blanking any higher
    /// reads picture as black level, which stretches the white point and
    /// washes the picture out: at the 30th percentile a full-scale ramp came
    /// back peaking at 61%.
    fn prime(&mut self) {
        let Some((tip, black)) = Self::levels_of(&self.hist) else { return };
        self.sync_level = tip;
        self.black_level = black;
        self.primed = self.black_level > self.sync_level;
    }

    /// The sync tip and blanking level of a stretch of baseband, as
    /// percentiles: sync is the bottom 2% of every line and blanking the
    /// bottom 13%, which is what the shape of composite video makes them.
    fn levels_of(samples: &[f32]) -> Option<(f32, f32)> {
        if samples.len() < 100 {
            return None;
        }
        let mut v = samples.to_vec();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        Some((v[v.len() / 50], v[v.len() * 13 / 100]))
    }

    /// Measure the levels again when the picture has stopped arriving.
    ///
    /// They are measured once, and on the air they go stale: a transmitter
    /// fades, the receiver's gain moves, the tuner drifts, and the whole
    /// baseband shifts with them. The threshold then sits above blanking or
    /// below the sync tip, no run is long enough to be a pulse, and the
    /// picture stops for good with the separator still believing it is
    /// primed. Three field periods of nothing is that, and nothing else: a
    /// camera sends one every twenty milliseconds, and a link too weak to
    /// hold sync gets the same treatment, which costs a re-measurement it
    /// was not going to use anyway.
    ///
    /// Re-measured rather than tracked continuously. The percentiles that
    /// find blanking hold over a field and a half of anything; over a
    /// shorter window of a bright picture they land inside the picture
    /// instead, and following them there washes the whole thing white.
    fn relevel(&mut self, seen: usize) {
        self.since_field += seen;
        let field_s = self.standard.line_s() * (self.standard.lines() / 2) as f64;
        if (self.since_field as f64) < 3.0 * field_s * self.rate {
            return;
        }
        self.since_field = 0;
        self.primed = false;
        self.hist.clear();
        self.lines.clear();
        self.chroma.clear();
        self.lines_seen = 0;
        self.last_axis = None;
    }

    fn threshold(&self) -> f32 {
        // Halfway between the tip and blanking, which is where a sync
        // separator has always sliced.
        (self.sync_level + self.black_level) / 2.0
    }

    /// One sample of the running mean.
    fn smoothed(&mut self, x: f32) -> f32 {
        if self.smooth.len() < self.smooth_len {
            self.smooth = vec![0.0; self.smooth_len];
            self.smooth_at = 0;
            self.smooth_filled = 0;
            self.smooth_sum = 0.0;
        }
        self.smooth_sum += x - self.smooth[self.smooth_at];
        self.smooth[self.smooth_at] = x;
        self.smooth_at = (self.smooth_at + 1) % self.smooth_len;
        self.smooth_filled = (self.smooth_filled + 1).min(self.smooth_len);
        self.smooth_sum / self.smooth_filled as f32
    }

    /// Feed demodulated baseband. Whole fields come back as they complete.
    pub fn process(&mut self, baseband: &[f32], out: &mut Vec<Field>) {
        let before = out.len();
        let mut filtered: Vec<f32> = Vec::with_capacity(baseband.len());
        for &x in baseband {
            let v = self.smoothed(x);
            filtered.push(v);
        }
        let raw = baseband;
        let baseband = &filtered[..];
        if !self.primed {
            self.since_field = 0;
            self.hist.extend_from_slice(baseband);
            // A field and a half, so the sample includes sync, blanking and
            // picture whatever the phase.
            if self.hist.len() as f64 > 0.03 * self.rate {
                self.prime();
                self.hist.clear();
            }
            if !self.primed {
                return;
            }
        }

        let thresh = self.threshold();
        // A horizontal pulse is 4.7 us and a vertical broad pulse 27.3; the
        // line either side of that is where they are told apart.
        let h_min = self.samples(self.standard.sync_s() * 0.6);
        let broad = self.samples(self.standard.sync_s() * 3.0);
        for (i, &x) in baseband.iter().enumerate() {
            self.line.push(x);
            if self.colour {
                self.raw_line.push(raw[i]);
            }
            self.since_sync += 1;
            self.sample += 1;
            if x < thresh {
                self.low_run += 1;
                continue;
            }
            let run = std::mem::take(&mut self.low_run);
            if run < h_min {
                continue;
            }
            if run >= broad {
                self.stats.broad_pulses += 1;
                // A broad pulse: the field is over. What is in hand is the
                // field, whether or not every line arrived.
                self.finish_field(out);
                self.line_start = self.sample;
                self.since_sync = 0;
                self.line.clear();
                self.raw_line.clear();
                continue;
            }
            // An ordinary line sync. The line just ended is cut and kept.
            self.stats.line_syncs += 1;
            let start = self.line_start;
            self.line_start = self.sample;
            self.take_line(run, start);
            self.since_sync = 0;
        }
        // A picture that stopped arriving is a picture whose levels have
        // gone stale; see `relevel`.
        if out.len() > before {
            self.since_field = 0;
        } else {
            self.relevel(baseband.len());
        }
    }

    /// Cut the picture out of the line that just ended and store it.
    ///
    /// The buffer runs from the end of the previous line's sync pulse, so it
    /// holds the back porch, the picture, the front porch and the sync that
    /// ended it. `sync_run` is that trailing sync, which is where the
    /// picture's far end is measured back from.
    fn take_line(&mut self, sync_run: usize, line_start: u64) {
        let line = std::mem::take(&mut self.line);
        self.line = Vec::with_capacity(line.len());
        // Taken here rather than beside the chroma below, because every
        // return in between would otherwise leave it holding this line as
        // well as the next and put the subcarrier phase a line out.
        let raw = std::mem::take(&mut self.raw_line);
        self.raw_line = Vec::with_capacity(raw.len());
        if self.lines.len() >= self.standard.active_lines() {
            return;
        }
        let start = self.samples(self.standard.back_porch_s());
        let want = self.samples(self.standard.active_s());
        if line.len() < start + sync_run + want / 2 {
            return;
        }
        let span = (line.len() - start - sync_run).min(want);
        let scale = self.black_level - self.sync_level;
        // White is about 0.7 V above blanking where sync is 0.3 V below it,
        // so the picture range is a bit over twice the sync depth. Taking it
        // from the levels rather than from a constant keeps the picture right
        // when a transmitter's deviation differs.
        let white = self.black_level + scale * 7.0 / 3.0;
        let mut row = vec![0u8; self.width];
        for (i, p) in row.iter_mut().enumerate() {
            let at = start + i * span / self.width.max(1);
            let v = line.get(at).copied().unwrap_or(self.black_level);
            let norm = (v - self.black_level) / (white - self.black_level).max(1e-6);
            *p = (norm.clamp(0.0, 1.0) * 255.0) as u8;
        }
        self.lines.push(row);
        self.lines_seen += 1;
        self.stats.lines_kept += 1;

        if self.colour {
            let uv = self.chroma_of(&raw, line_start, start, span, scale);
            self.chroma.push(uv);
        }
    }

    /// Demodulate one line's chroma against its own burst.
    ///
    /// Returns (U, V) per output pixel, already through the delay line: the
    /// mean with the line above, which is what cancels a phase error in PAL
    /// rather than leaving it as alternating tint.
    fn chroma_of(
        &mut self,
        raw: &[f32],
        line_start: u64,
        start: usize,
        span: usize,
        scale: f32,
    ) -> Vec<(f32, f32)> {
        let w = std::f64::consts::TAU * self.standard.subcarrier_hz() / self.rate;
        // Phase of the free-running subcarrier at an offset into this line.
        // Straight into f64 rather than through a modulo: an f64 holds the
        // product exactly for hours of samples, and folding the index instead
        // would step the phase every time it wrapped, since the subcarrier is
        // not a whole number of samples.
        let phase_at = |k: usize| (w * (line_start + k as u64) as f64).rem_euclid(std::f64::consts::TAU) as f32;
        // The burst sits on the back porch, about 0.9 us after the sync ends
        // and ten cycles long.
        let b0 = self.samples(0.8e-6);
        let b1 = self.samples(3.2e-6).min(raw.len());
        if b1 <= b0 + 8 {
            return Vec::new();
        }
        let mean = raw[b0..b1].iter().sum::<f32>() / (b1 - b0) as f32;
        let (mut bi, mut bq) = (0.0f32, 0.0f32);
        for (n, &x) in raw[b0..b1].iter().enumerate() {
            let p = phase_at(b0 + n);
            bi += (x - mean) * p.cos();
            bq += (x - mean) * p.sin();
        }
        let n = (b1 - b0) as f32;
        let (bi, bq) = (2.0 * bi / n, 2.0 * bq / n);
        let amp = (bi * bi + bq * bq).sqrt();
        // A burst is half the sync depth. Much less than that is a burst that
        // was not there, and demodulating noise against it invents colour.
        if amp < 0.15 * scale {
            self.last_axis = None;
            return Vec::new();
        }
        let phase = bq.atan2(bi);

        // Both polarities of the line give a candidate axis, 90 degrees
        // apart. The one that keeps the axis where the last line put it is
        // the right one; with no previous line there is nothing to be
        // continuous with, so the line is left grey rather than guessed.
        // A burst is the vector -U plus or minus V, which is 135 degrees from
        // the U axis on one side or the other.
        let eighth = 0.75 * std::f32::consts::PI;
        let axis_plus = wrap(phase - eighth);
        let axis_minus = wrap(phase + eighth);
        let Some(prev_axis) = self.last_axis else {
            self.last_axis = Some(axis_plus);
            return Vec::new();
        };
        let (theta, swing) = if wrap(axis_plus - prev_axis).abs() < wrap(axis_minus - prev_axis).abs()
        {
            (axis_plus, 1.0f32)
        } else {
            (axis_minus, -1.0f32)
        };
        self.last_axis = Some(theta);

        // Quadrature demodulate against that reference and box filter to
        // about 1.3 MHz, which is the chroma bandwidth.
        let taps = ((self.rate / 2.6e6).round() as usize).max(1);
        // The burst is the reference for chroma the way the sync tip is for
        // luma: it is sent at 0.15 V where white is 0.7 V above blanking, so
        // dividing by the measured burst and multiplying by that ratio puts
        // U and V in the same units as the luma the picture is built from.
        // Leaving chroma in volts against a luma normalised to white halves
        // the saturation, which off air looks like a camera with the colour
        // turned down rather than like a bug.
        let gain = (0.15 / 0.7) / amp;
        // The reference, once per line, as a rotation rather than a sine per
        // sample. Every output pixel integrates eight samples against it, so
        // a 20 MS/s span was asking for seventy million sines a second and
        // the separator cost nearly twice real time on its own. Seeded from
        // the exact phase at the start of the line and advanced by one
        // sample's rotation, which over a line is far inside the precision
        // the burst itself is measured to.
        let need = (span + taps).min(raw.len().saturating_sub(start));
        let (dc, ds) = ((w as f32).cos(), (w as f32).sin());
        let mut osc: Vec<(f32, f32)> = Vec::with_capacity(need);
        let p0 = phase_at(start) + theta;
        let (mut c, mut sn) = (p0.cos(), p0.sin());
        for _ in 0..need {
            osc.push((c, sn));
            let (nc, ns) = (c * dc - sn * ds, sn * dc + c * ds);
            c = nc;
            sn = ns;
        }
        let mut out = Vec::with_capacity(self.width);
        for i in 0..self.width {
            let at = start + i * span / self.width.max(1);
            let (mut u, mut v) = (0.0f32, 0.0f32);
            let mut count = 0.0f32;
            for k in 0..taps {
                let Some(&x) = raw.get(at + k) else { break };
                let Some(&(pc, ps)) = osc.get(at + k - start) else { break };
                u += x * pc;
                v += x * ps;
                count += 1.0;
            }
            if count == 0.0 {
                out.push((0.0, 0.0));
                continue;
            }
            // Scaled by the burst, so the picture's colours are referred to
            // the amplitude the transmitter sent rather than to the
            // demodulator's own scale.
            let u = 2.0 * u / count * gain;
            let v = 2.0 * v / count * gain * swing;
            out.push((u, v));
        }
        // The delay line: mean with the line above.
        if let Some(prev_row) = self.chroma.last() {
            if prev_row.len() == out.len() {
                for (o, p) in out.iter_mut().zip(prev_row) {
                    o.0 = (o.0 + p.0) / 2.0;
                    o.1 = (o.1 + p.1) / 2.0;
                }
            }
        }
        out
    }

    fn finish_field(&mut self, out: &mut Vec<Field>) {
        let rgb = self.colour.then(|| self.to_rgb()).flatten();
        let height = self.standard.active_lines();
        let seen = std::mem::take(&mut self.lines_seen);
        if seen < height / 4 {
            // Fewer than a quarter of the lines is a fade or a false start,
            // not a picture.
            self.lines.clear();
            self.chroma.clear();
            self.last_axis = None;
            self.stats.fields_short += 1;
            return;
        }
        let mut luma = Vec::with_capacity(self.width * height);
        for r in 0..height {
            match self.lines.get(r) {
                Some(row) => luma.extend_from_slice(row),
                // A line that never arrived is black rather than a repeat of
                // its neighbour: a receiver should be able to see what it
                // missed.
                None => luma.extend(std::iter::repeat_n(0u8, self.width)),
            }
        }
        self.lines.clear();
        self.chroma.clear();
        self.last_axis = None;
        self.stats.fields += 1;
        out.push(Field {
            width: self.width,
            height,
            aspect: self.standard.aspect(),
            luma,
            rgb,
            lines_seen: seen,
        });
    }

    /// Combine the luma rows and the chroma rows into RGB.
    ///
    /// `None` when too few lines carried a burst to call it a colour picture,
    /// which is what a monochrome camera looks like and is not a failure.
    fn to_rgb(&self) -> Option<Vec<u8>> {
        let height = self.standard.active_lines();
        let with_burst = self.chroma.iter().filter(|r| !r.is_empty()).count();
        if with_burst * 2 < self.lines.len() {
            return None;
        }
        let mut out = Vec::with_capacity(self.width * height * 3);
        // The rows are found once a row and not once a pixel: at 640 by 288,
        // fifty fields a second, the two bounds-checked lookups a pixel were
        // most of what building the picture cost.
        for r in 0..height {
            let luma = self.lines.get(r).map(|row| &row[..]).unwrap_or(&[]);
            let chroma = self.chroma.get(r).map(|row| &row[..]).unwrap_or(&[]);
            for c in 0..self.width {
                let y = luma.get(c).map(|&v| f32::from(v) / 255.0).unwrap_or(0.0);
                let (u, v) = chroma.get(c).copied().unwrap_or((0.0, 0.0));
                // U and V are the weighted colour differences, so undoing the
                // weights gives B-Y and R-Y, and green follows from the luma
                // equation.
                let bmy = u / 0.493;
                let rmy = v / 0.877;
                let g = y - 0.5094 * rmy - 0.1942 * bmy;
                for ch in [y + rmy, g, y + bmy] {
                    out.push((ch.clamp(0.0, 1.0) * 255.0) as u8);
                }
            }
        }
        Some(out)
    }

    pub fn reset(&mut self) {
        self.since_sync = 0;
        self.lines.clear();
        self.lines_seen = 0;
        self.low_run = 0;
        self.line.clear();
        self.hist.clear();
        self.primed = false;
        self.raw_line.clear();
        self.chroma.clear();
        self.last_axis = None;
    }
}

/// An angle folded into -pi to pi.
fn wrap(a: f32) -> f32 {
    let t = std::f32::consts::TAU;
    let mut a = a % t;
    if a > std::f32::consts::PI {
        a -= t;
    }
    if a < -std::f32::consts::PI {
        a += t;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build composite video the way a camera does: sync, back porch,
    /// picture, front porch, with a vertical pulse between fields. Levels are
    /// the 1 V standard, sync at -0.3 and white at +0.7 above blanking.
    fn synth(standard: Standard, rate: f64, fields: usize, ramp: bool) -> Vec<f32> {
        let n = |s: f64| (s * rate).round() as usize;
        let line = n(standard.line_s());
        let sync = n(standard.sync_s());
        let back = n(standard.back_porch_s());
        let active = n(standard.active_s());
        let mut out = Vec::new();
        for _ in 0..fields {
            for r in 0..standard.active_lines() {
                out.extend(std::iter::repeat_n(-0.3f32, sync));
                out.extend(std::iter::repeat_n(0.0f32, back));
                for i in 0..active {
                    // A horizontal ramp, so a picture read back at the wrong
                    // offset is visibly wrong rather than plausibly grey.
                    let v = if ramp {
                        i as f32 / active as f32
                    } else {
                        r as f32 / standard.active_lines() as f32
                    };
                    out.push(v * 0.7);
                }
                let used = sync + back + active;
                out.extend(std::iter::repeat_n(0.0f32, line.saturating_sub(used)));
            }
            // The vertical interval, one broad pulse long enough to be told
            // from a line sync.
            out.extend(std::iter::repeat_n(-0.3f32, n(27.3e-6)));
            out.extend(std::iter::repeat_n(0.0f32, line));
        }
        out
    }

    #[test]
    fn a_field_comes_back_with_its_lines() {
        let rate = 16e6;
        let v = synth(Standard::Pal, rate, 3, true);
        let mut sep = SyncSeparator::new(rate, Standard::Pal, 320);
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        assert!(!out.is_empty(), "no field came out");
        let f = out.last().unwrap();
        assert_eq!(f.height, 288);
        assert_eq!(f.width, 320);
        assert!(
            f.lines_seen > 250,
            "only {} lines of 288 were found",
            f.lines_seen
        );
    }

    /// The picture has to come back the right way round and at the right
    /// level: a ramp that reads flat means the line was cut in the wrong
    /// place, and one that reads reversed means the sample order is wrong.
    #[test]
    fn the_picture_is_the_one_that_was_transmitted() {
        let rate = 16e6;
        let v = synth(Standard::Pal, rate, 3, true);
        let mut sep = SyncSeparator::new(rate, Standard::Pal, 320);
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        let f = out.last().expect("a field");
        let row = &f.luma[f.width * 100..f.width * 101];
        assert!(row[10] < 40, "the left of a ramp should be dark: {}", row[10]);
        assert!(row[300] > 200, "the right should be white: {}", row[300]);
        for w in row.windows(2).step_by(16) {
            assert!(w[1] + 8 >= w[0], "the ramp is not monotonic");
        }
    }

    /// Two standards, told apart by the line period rather than by being
    /// configured. They are 0.7% apart, which no transmitter's timebase
    /// error reaches.
    #[test]
    fn the_line_period_names_the_standard() {
        assert_eq!(Standard::from_line_period(64e-6), Some(Standard::Pal));
        assert_eq!(Standard::from_line_period(63.55e-6), Some(Standard::Ntsc));
        assert_eq!(Standard::from_line_period(50e-6), None);
    }

    /// Build one line of PAL with a burst and a chroma vector on it. `uv` is
    /// the colour in the same weighted units the decoder reports, and `swing`
    /// is the line's V polarity.
    ///
    /// The subcarrier phase is counted from the start of the back porch,
    /// which is where the decoder's line buffer begins.
    fn colour_line(
        out: &mut Vec<f32>,
        standard: Standard,
        rate: f64,
        y: f32,
        uv: (f32, f32),
        swing: f32,
    ) {
        let n = |s: f64| (s * rate).round() as usize;
        let w = std::f64::consts::TAU * standard.subcarrier_hz() / rate;
        // The subcarrier runs continuously through the whole signal, sync
        // pulses included, as a transmitter's does.
        let base = out.len();
        let phase_at = |k: usize| (w * (base + k) as f64) as f32;
        out.extend(std::iter::repeat_n(-0.3f32, n(standard.sync_s())));

        let back = n(standard.back_porch_s());
        let burst_at = n(0.9e-6);
        let burst_len = n(2.25e-6);
        let sync_len = n(standard.sync_s());
        for k in 0..back {
            let p = phase_at(sync_len + k);
            // The burst is -U plus or minus V at 0.15, which is 135 or 225
            // degrees.
            let v = if (burst_at..burst_at + burst_len).contains(&k) {
                0.15 * (-p.cos() + swing * p.sin()) / std::f32::consts::SQRT_2
            } else {
                0.0
            };
            out.push(v);
        }
        let active = n(standard.active_s());
        for k in 0..active {
            let p = phase_at(sync_len + back + k);
            out.push(y * 0.7 + uv.0 * p.cos() + swing * uv.1 * p.sin());
        }
        let used = sync_len + back + active;
        out.extend(std::iter::repeat_n(
            0.0f32,
            n(standard.line_s()).saturating_sub(used),
        ));
    }

    /// Colour bars through the whole chain: a burst, a chroma vector, the
    /// line flip, and the delay line that makes the flip worth having.
    #[test]
    fn a_colour_field_comes_back_as_the_colours_that_were_sent() {
        let rate = 20e6;
        let standard = Standard::Pal;
        // A mid blue and a mid red, in the weighted units the decoder
        // reports: U is 0.493(B-Y) and V is 0.877(R-Y).
        let blue = (0.493 * 0.5, 0.0);
        let mut v = Vec::new();
        for _ in 0..3 {
            for r in 0..standard.active_lines() {
                let swing = if r % 2 == 0 { 1.0 } else { -1.0 };
                colour_line(&mut v, standard, rate, 0.5, blue, swing);
            }
            v.extend(std::iter::repeat_n(-0.3f32, (27.3e-6 * rate) as usize));
            v.extend(std::iter::repeat_n(0.0f32, (standard.line_s() * rate) as usize));
        }

        let mut sep = SyncSeparator::new(rate, standard, 320).with_colour();
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        let f = out.last().expect("a field");
        let rgb = f.rgb.as_ref().expect("colour");
        // Sample the middle of the picture, away from the edges where the
        // box filter is still filling.
        let at = (f.width * 150 + 160) * 3;
        let (r, g, b) = (rgb[at], rgb[at + 1], rgb[at + 2]);
        assert!(
            b > r + 40 && b > g + 40,
            "a blue vector should read blue, got r{r} g{g} b{b}"
        );
    }

    /// The positive test: a camera's sync pulses agree with each other, and
    /// nothing else does. This is what keeps a video front end off the wide
    /// sources that are not cameras, rather than letting it demodulate every
    /// block of one forever.
    #[test]
    fn a_camera_is_told_from_everything_else_by_its_line_rate() {
        let rate = 16e6;
        let lock = find_lines(&synth(Standard::Pal, rate, 3, true), rate).expect("a camera");
        assert_eq!(lock.standard, Standard::Pal);
        assert!(lock.agreement > 0.9, "agreement {}", lock.agreement);
        assert!(lock.pulses > 500, "{} pulses", lock.pulses);

        let ntsc = find_lines(&synth(Standard::Ntsc, rate, 3, true), rate).expect("a camera");
        assert_eq!(ntsc.standard, Standard::Ntsc);
    }

    #[test]
    fn noise_is_not_a_camera() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let noise: Vec<f32> = (0..2_000_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 40) as f32 / 8_388_608.0 - 1.0
            })
            .collect();
        assert_eq!(find_lines(&noise, 16e6), None);

        // Nor is a keyed carrier: a burst train at some other rate has gaps
        // that do not land on a line period, which is the case a median alone
        // would fall for.
        let rate = 16e6;
        let mut keyed = Vec::new();
        for i in 0..2000 {
            let on = ((i as f64 * 0.0007 * rate) as usize % 90) + 40;
            keyed.extend(std::iter::repeat_n(-0.3f32, on));
            keyed.extend(std::iter::repeat_n(0.2f32, 700));
        }
        assert_eq!(find_lines(&keyed, rate), None);
    }

    #[test]
    fn noise_produces_no_field() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let v: Vec<f32> = (0..2_000_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 40) as f32 / 8_388_608.0 - 1.0
            })
            .collect();
        let mut sep = SyncSeparator::new(16e6, Standard::Pal, 320);
        let mut out = Vec::new();
        sep.process(&v, &mut out);
        assert!(out.is_empty(), "noise produced {} fields", out.len());
    }

    /// The picture's shape is the standard's, not the sample grid's. Drawn
    /// from its own numbers a field is 10:9 and looks like a 4:3 picture
    /// with the sides pushed in.
    #[test]
    fn a_field_carries_the_shape_of_the_picture_and_not_of_its_samples() {
        let rate = 20e6;
        let mut sep = SyncSeparator::new(rate, Standard::Pal, 640);
        let mut fields = Vec::new();
        sep.process(&synth(Standard::Pal, rate, 3, true), &mut fields);
        let f = fields.first().expect("a field");
        assert!((f.aspect - 4.0 / 3.0).abs() < 1e-6, "{}", f.aspect);
        assert_ne!(
            f.aspect,
            f.width as f32 / (f.height as f32 * 2.0),
            "the sample grid decided the shape"
        );
    }

    /// The levels are measured again when the picture stops, or a receiver
    /// whose gain moves slices at the wrong height and the picture is gone
    /// for good with the separator still believing it is primed.
    #[test]
    fn a_picture_comes_back_after_the_level_moves_under_it() {
        let rate = 20e6;
        let mut base = synth(Standard::Pal, rate, 24, true);
        // Half way through, the whole baseband shifts and shrinks: a fade, a
        // gain step, or a tuner that drifted.
        let half = base.len() / 2;
        for x in &mut base[half..] {
            *x = *x * 0.6 - 0.25;
        }
        let mut sep = SyncSeparator::new(rate, Standard::Pal, 640);
        let mut fields = Vec::new();
        // In blocks, the way the graph feeds it.
        for block in base.chunks(16_384) {
            sep.process(block, &mut fields);
        }
        // Half the fields are before the step. What matters is that they
        // start again after it: without the re-measurement the separator
        // produced the first eleven and then nothing at all, for good.
        assert!(fields.len() > 13, "only {} fields, so the level step lost it", fields.len());
        let last = fields.last().expect("a field after the step");
        assert!(last.lines_seen > 200, "{} lines after the step", last.lines_seen);
    }
}
