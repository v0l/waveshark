//! What kind of signal is this? Blind modulation classification for one burst.
//!
//! Every front end in this crate answers a question that has already been
//! decided: [`crate::pulse`] assumes the burst is keyed on amplitude,
//! [`crate::fsk`] that it is keyed on frequency with two tones,
//! [`crate::c4fm`] that it has four. Deciding in advance means running every
//! front end over every channel and letting the wrong ones produce nothing,
//! which is what the ISM graph did: an envelope path and a discriminator path
//! over the same samples, both paid for on every channel, all the time.
//!
//! This module measures the burst instead and says what it is. The classes are
//! the ones a receiver can tell apart from a short capture without decoding
//! anything, which is fewer than the list of modulations that exist and more
//! than the two the banks currently run.
//!
//! # What this cannot do, by construction
//!
//! Two protocols using the same modulation are indistinguishable here. That is
//! correct: telling a doorbell from a tyre pressure sensor is the protocol
//! layer's job, and both are OOK PWM to any measurement made before slicing.
//!
//! A spread-spectrum signal below the noise floor has no features to measure
//! until it is despread, so GPS is not "unknown", it is invisible. Anything
//! wider than the channel it arrived in is measured through a filter that
//! removed most of it, which is why [`Features::bandwidth_hz`] is reported
//! next to the channel width rather than on its own: a burst that fills its
//! channel is evidence about the *bank*, not about the signal, and the honest
//! answer is to say so and re-run it in a wider tier.
//!
//! And a classification is a measurement with a confidence, never a claim. A
//! burst that fits nothing well is [`Modulation::Unknown`] with its features
//! attached, which is what the packet log is for.
//!
//! # How it decides
//!
//! Features first, then a set of hypotheses that each score themselves against
//! those features, then the best score if it clears the runner-up by a margin.
//! Not a cascade of thresholds: a cascade commits to amplitude keying before
//! it has looked at the frequency track, and the first mistake is unrecoverable
//! because nothing downstream reconsiders it.

mod amplitude;
mod chirp;
pub mod cyclo;
mod frequency;
pub mod hypothesis;
mod dsss;
pub mod mode;
mod ofdm;
mod phase;
mod steady;
mod tones;
mod zoom;

pub use hypothesis::{Evidence, Hypothesis};

use crate::window;
use common::C32;
use rustfft::FftPlanner;

/// Every hypothesis the classifier will consider.
///
/// Adding a modulation is adding a file and a line here. Nothing else in this
/// module knows how many there are, and no hypothesis can see another's
/// reasoning, so a fix to one is isolated to its own file by construction.
pub fn hypotheses() -> &'static [&'static dyn Hypothesis] {
    &[
        &amplitude::Ook,
        &amplitude::Ask,
        &frequency::Fsk2,
        &frequency::Msk,
        &frequency::Fsk4,
        &phase::Psk2,
        &phase::Psk4,
        &phase::Dqpsk,
        &chirp::Chirp,
        &ofdm::Ofdm,
        &dsss::Dsss,
        &steady::NoiseLike,
        &steady::Carrier,
    ]
}

/// What the burst was keyed on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modulation {
    /// Amplitude keyed all the way off. The OOK front end reads these.
    Ook,
    /// Amplitude keyed, but not to zero. Shallow ASK, which needs
    /// [`crate::ask`] rather than the plain envelope path.
    Ask,
    /// Two tones. [`crate::fsk`] reads these.
    Fsk2,
    /// Four levels. [`crate::c4fm`] reads these.
    Fsk4,
    /// Two tones at a modulation index near 0.5, which is MSK and its
    /// filtered relative GMSK. Worth separating from plain FSK because the
    /// tones overlap and a hard threshold on the discriminator loses to a
    /// matched receiver by several dB.
    Msk,
    /// Phase keyed, two states.
    Psk2,
    /// Phase keyed, four states.
    Psk4,
    /// Four phases shifted by an eighth of a turn every symbol, which is
    /// what TETRA and the trunked systems key. The fourth power alternates
    /// sign each symbol, so instead of one line it shows a pair a symbol
    /// rate apart.
    Dqpsk,
    /// Frequency swept linearly, which is chirp spread spectrum and radar.
    Chirp,
    /// Many carriers with a cyclic prefix. Told from the rest of the
    /// noise-like family by the prefix repeating at one lag.
    Ofdm,
    /// A single carrier spread by a chip sequence, which repeats in the
    /// envelope even though the data cancels it in the complex samples.
    Dsss,
    /// Modulated, but with no keying structure to find: flat spectrum,
    /// Gaussian amplitude. OFDM and direct sequence spread spectrum both land
    /// here, and so does interference.
    NoiseLike,
    /// Present and steady. An unmodulated carrier, a leaking oscillator, or
    /// the quiet half of a signal whose data has not started yet.
    Carrier,
    /// Measured, and nothing fit.
    Unknown,
}

impl Modulation {
    pub fn label(&self) -> &'static str {
        match self {
            Modulation::Ook => "OOK",
            Modulation::Ask => "ASK",
            Modulation::Fsk2 => "2-FSK",
            Modulation::Fsk4 => "4-FSK",
            Modulation::Msk => "MSK",
            Modulation::Psk2 => "BPSK",
            Modulation::Psk4 => "QPSK",
            Modulation::Dqpsk => "pi/4-DQPSK",
            Modulation::Chirp => "chirp",
            Modulation::Ofdm => "OFDM",
            Modulation::Dsss => "DSSS",
            Modulation::NoiseLike => "noise-like",
            Modulation::Carrier => "carrier",
            Modulation::Unknown => "unknown",
        }
    }

    /// Whether a front end in this crate can read it today.
    pub fn has_front_end(&self) -> bool {
        matches!(self, Modulation::Ook | Modulation::Ask | Modulation::Fsk2 | Modulation::Fsk4)
    }
}

/// Everything measured from the burst, kept whatever the verdict.
///
/// The features are the useful part of an unknown burst. A modulation nobody
/// has written a decoder for still has a bandwidth, a symbol rate and a
/// deviation, and those three are most of what identifying a device needs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Features {
    pub samples: usize,
    pub duration_us: f64,
    /// Ratio of the lower envelope level to the upper one. Near zero for
    /// on-off keying, near one for anything constant envelope. Measured from
    /// the levels themselves, so it does not move with the duty cycle.
    pub envelope_ratio: f32,
    /// Levels found in the envelope: one for a constant envelope, two for
    /// anything keyed on amplitude.
    pub envelope_modes: u8,
    /// Fraction of the burst spent at the upper envelope level. A packet is
    /// mostly silence and a constant-envelope transmission is not, which is
    /// what stops a window of mostly noise being read as a modulation.
    pub duty: f32,
    /// Median length of a run at the upper envelope level, in symbols.
    ///
    /// This is what tells amplitude keying from a keyed carrier that happens
    /// to be transmitted in bursts. On-off keying holds its level for a symbol
    /// or three, because that is what it is keying; a frequency-keyed packet
    /// holds it for the whole packet, and the gaps in the window are between
    /// transmissions rather than inside one. Duty cycle alone cannot see the
    /// difference: a window holding three FSK repeats is 30% on, and so is an
    /// OOK burst.
    pub level_run_symbols: f32,
    /// Occupied bandwidth holding 99% of the power, in hertz.
    pub bandwidth_hz: f32,
    /// Occupied bandwidth as a fraction of the channel. Above about 0.8 the
    /// measurement is of the channel filter rather than of the signal.
    pub channel_fill: f32,
    /// Geometric over arithmetic mean of the spectrum inside the occupied
    /// band. Near one for noise and OFDM, well below for anything with tones.
    pub flatness: f32,
    /// Share of the power in the strongest three bins. Near one for a bare
    /// carrier, near zero for anything spread across its channel.
    ///
    /// Flatness cannot answer this: a carrier occupies three bins and is
    /// perfectly flat across the three, so measuring inside the occupied band
    /// makes the narrowest signal there is look like noise.
    pub peakiness: f32,
    /// Peaks found in the histogram of instantaneous frequency: 1, 2, 4 or 0
    /// when there is no structure to count.
    pub tones: u8,
    /// Spacing between the outermost frequency peaks, in hertz.
    pub separation_hz: f32,
    /// Estimated symbol rate from the transition line, in baud.
    pub baud: f32,
    /// Strength of that line against the surrounding spectrum. Below about 3
    /// there is no symbol clock to be found.
    pub baud_line: f32,
    /// Peak deviation over half the symbol rate: the modulation index. 0.5 is
    /// MSK, and anything above about 1.5 is plain wide FSK.
    pub mod_index: f32,
    /// Fraction of the burst whose frequency slope matches the median slope,
    /// which is what a linear sweep looks like however often it wraps.
    pub chirp_fit: f32,
    /// That slope, in hertz per second.
    pub chirp_rate: f32,
    /// Strength of the spectral line in the squared and fourth-power signals,
    /// which is how a phase-keyed carrier gives itself away.
    pub square_line: f32,
    pub quartic_line: f32,
    /// Share of the fourth-power spectrum in a pair of lines, which is what
    /// pi/4-shifted QPSK leaves there: its two constellations differ by an
    /// eighth of a turn, so the fourth power flips sign every symbol and
    /// its line splits in two, a symbol rate apart. When a pair is found
    /// `baud` is their separation, which is a better measurement of the
    /// rate than the transition spectrum gives at a few samples a symbol.
    /// The eighth power would collapse the pair to one line, and was tried;
    /// on a pulse-shaped carrier at seven samples a symbol it is too weak
    /// to read.
    pub quartic_pair: f32,
    /// Kurtosis of the envelope, 3.0 for a Gaussian. Multi-carrier signals sit
    /// near 3; a keyed single carrier sits well below.
    pub kurtosis: f32,
    pub snr_db: f32,

    /// Strongest autocorrelation at a lag past the signal's own correlation
    /// width, and how far it stands above the median across lags.
    ///
    /// This is what splits the noise-like class. A cyclic prefix repeats a
    /// piece of every symbol verbatim, so it puts a *spike* at one lag; a
    /// narrowband signal, a bare carrier or a DC offset correlate across a
    /// plateau of them. Without the second number every narrowband burst in a
    /// wide capture passes for OFDM, which is what happened when this was
    /// tried with the peak alone.
    pub cyclic: f32,
    pub cyclic_ratio: f32,
    /// The lag of that peak, in seconds. A symbol period, if it is real: 66.7
    /// us for LTE's 15 kHz subcarriers, 4 us for 802.11a and its descendants.
    pub cyclic_period_s: f32,
    /// The same test on envelope power rather than on the complex samples.
    ///
    /// A spreading code repeats in the envelope, but its data flips the sign
    /// of every symbol, and those flips cancel the complex autocorrelation
    /// exactly. Squaring removes the sign. This is the feature that names
    /// 802.11b beacons, which the complex test cannot see at all.
    pub env_cyclic: f32,
    pub env_cyclic_ratio: f32,
}

/// One classified burst.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BurstClass {
    pub modulation: Modulation,
    /// How far the winning hypothesis scored above the runner-up, 0 to 1.
    ///
    /// Not a probability. It is the margin, which is the number that matters
    /// when the question is whether to act on the verdict: a burst that fits
    /// two classes equally well should be routed to neither.
    pub confidence: f32,
    /// The winner's own score, before the margin.
    pub score: f32,
    pub features: Features,
}

#[derive(Clone, Copy, Debug)]
pub struct ClassifyConfig {
    /// Channel width the burst arrived in, in hertz. Used to say when the
    /// measured bandwidth is really the channel filter's.
    pub channel_hz: f32,
    /// Shortest burst worth classifying, in samples.
    pub min_samples: usize,
    /// FFT size for the spectral features. 1024 is about 30 Hz of resolution
    /// at 31.25 kHz and 500 Hz at 500 kHz, which is finer than any decision
    /// here needs.
    pub fft_size: usize,
    /// Winning score below this is [`Modulation::Unknown`] whatever the
    /// margin.
    pub min_score: f32,
    /// Margin over the runner-up below which the verdict is unknown.
    pub min_margin: f32,
    /// Occupied fraction of the span below which the burst is brought down to
    /// its own bandwidth before measuring. See [`zoom`].
    ///
    /// Zero disables it, which is right for a channelized bank: there the
    /// channel is already the signal's width, and halving further would throw
    /// away the skirts that say what the keying is.
    pub zoom_below: f32,
}

impl Default for ClassifyConfig {
    fn default() -> Self {
        Self {
            channel_hz: 0.0,
            min_samples: 256,
            fft_size: 1024,
            min_score: 0.45,
            min_margin: 0.05,
            // Off by default, so a channelized bank behaves exactly as it did
            // before this existed. A caller handing over wideband captures
            // turns it on.
            zoom_below: 0.0,
        }
    }
}

/// Measure a burst and say what modulation it carries.
///
/// `iq` is the burst as the channel delivered it, including the gaps inside
/// it: an on-off keyed packet is mostly silence, and removing the silence
/// removes the evidence.
pub struct Classifier {
    cfg: ClassifyConfig,
    rate: f64,
    planner: FftPlanner<f32>,
    win: Vec<f32>,
    /// Scratch, reused across bursts: nothing here allocates per burst after
    /// the first of a given size.
    spec: Vec<f32>,
    spec_real: Vec<f32>,
    spec_pow: Vec<f32>,
    /// Frequency track over the whole burst, holding its last value wherever
    /// the carrier was too weak to measure one.
    freq: Vec<f32>,
    /// The same track, keeping only the samples that had a carrier. Tone
    /// counting and sweep fitting want those and nothing else.
    freq_hi: Vec<f32>,
    amp: Vec<f32>,
    /// The envelope in decibels, for the level histogram.
    db: Vec<f32>,
    /// Where the burst changed, at one entry per sample gap.
    trans: Vec<f32>,
    scratch: Vec<f32>,
    fft_buf: Vec<C32>,
}

impl Classifier {
    pub fn new(rate: f64, cfg: ClassifyConfig) -> Self {
        Self {
            cfg,
            rate,
            planner: FftPlanner::new(),
            win: window::blackman_harris(cfg.fft_size),
            spec: Vec::new(),
            spec_real: Vec::new(),
            spec_pow: Vec::new(),
            freq: Vec::new(),
            freq_hi: Vec::new(),
            amp: Vec::new(),
            db: Vec::new(),
            trans: Vec::new(),
            scratch: Vec::new(),
            fft_buf: Vec::new(),
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    pub fn classify(&mut self, iq: &[C32]) -> BurstClass {
        let features = self.features(iq);
        decide(&features, &self.cfg)
    }

    /// Fraction of the span the signal occupies, and where its centre is.
    ///
    /// A cheap first pass whose only job is to decide whether the burst needs
    /// bringing down to its own bandwidth before the real measurement.
    fn survey(&mut self, iq: &[C32]) -> (f32, f64) {
        self.spectrum(iq);
        let spec = std::mem::take(&mut self.spec);
        let (bw, _) = occupied(&spec, &mut self.scratch, self.rate as f32);
        // Centre of mass of what stands above the noise floor.
        self.scratch.clear();
        self.scratch.extend_from_slice(&spec);
        self.scratch
            .sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let floor = self.scratch[spec.len() / 2];
        let (mut num, mut den) = (0.0f64, 0.0f64);
        for (i, &v) in spec.iter().enumerate() {
            let p = (v - floor).max(0.0) as f64;
            num += p * (i as f64 - spec.len() as f64 / 2.0);
            den += p;
        }
        let centre = if den > 0.0 {
            num / den / spec.len() as f64 * self.rate
        } else {
            0.0
        };
        self.spec = spec;
        ((bw / self.rate as f32).clamp(0.0, 1.0), centre)
    }

    /// Measure without judging. Exposed because the features are worth logging
    /// for a burst the hypotheses all reject.
    pub fn features(&mut self, iq: &[C32]) -> Features {
        // Trim to where the carrier was actually on, then measure that.
        //
        // Without this every packet is amplitude keyed, because a burst
        // handed over by a detector has silence at both ends and the envelope
        // has two levels whatever the transmitter did with them. What
        // separates on-off keying from a frequency-keyed packet is that the
        // amplitude changes *during* the transmission, and that question can
        // only be asked once the transmission's edges are known.
        let (a, b) = self.extent(iq);
        let (trimmed, samples) = if b - a >= self.cfg.min_samples && b - a < iq.len() * 9 / 10 {
            (iq[a..b].to_vec(), iq.len())
        } else {
            (iq.to_vec(), iq.len())
        };

        // Bring a signal that is a small part of its span down to its own
        // bandwidth first. In the channel bank this never fires, because the
        // channelizer has already done it; on a capture it is the difference
        // between measuring the transmission and measuring the channel.
        let (occ, centre) = self.survey(&trimmed);
        let mut f = if occ < self.cfg.zoom_below && occ > 0.0 {
            let (z, zrate) = zoom::to_signal(&trimmed, self.rate, centre, occ, self.cfg.zoom_below);
            let outer = self.rate;
            self.rate = zrate;
            let f = self.measure(&z);
            self.rate = outer;
            f
        } else {
            self.measure(&trimmed)
        };
        f.samples = samples;
        f.duration_us = samples as f64 * 1e6 / self.rate;
        f
    }

    /// First and last sample with a carrier, with a symbol either side.
    fn extent(&mut self, iq: &[C32]) -> (usize, usize) {
        self.amp.clear();
        self.amp.extend(iq.iter().map(|c| c.norm()));
        let (ratio, modes, top, _) = envelope_levels(&mut self.scratch, &self.amp, &mut self.db);
        if modes < 2 {
            return (0, iq.len());
        }
        // Halfway between the two levels the histogram found, in decibels,
        // rather than at a fixed fraction of the upper one.
        //
        // A fixed fraction assumes how far below the carrier the silence sits,
        // which is the signal to noise ratio, which is the one thing a burst
        // handed over by a detector has not promised. At 0.3 of the top, any
        // burst whose noise floor is within ten decibels of its carrier has
        // every sample above the threshold: the extent is then the whole
        // window, nothing is trimmed, and the measurement that follows is of
        // the silence. That is what BLE packets do at thirteen decibels, and
        // it is why they were read as amplitude keying.
        let edges = |amp: &[f32], floor: f32| {
            let first = amp.iter().position(|v| *v >= floor).unwrap_or(0);
            let last = amp.iter().rposition(|v| *v >= floor).map_or(amp.len(), |i| i + 1);
            (first, last)
        };

        // Three tenths of the upper level, which is what the corpus wants: its
        // devices key all the way down, so a threshold well below the carrier
        // still sits far above their silence.
        let (first, last) = edges(&self.amp, top * 0.3);
        if last > first && last - first < iq.len() * 9 / 10 {
            return (first, last);
        }

        // Nothing was trimmed, so that threshold is below this burst's own
        // noise: the silence around it is within ten decibels of its carrier.
        // Halfway between the two levels the histogram found is the threshold
        // that does not assume a signal to noise ratio. BLE packets at
        // thirteen decibels land here, and were measured as amplitude keying
        // until they did.
        //
        // Smoothed first, and only here. The first and last crossings are
        // otherwise set by single samples, and a Rayleigh tail reaches far
        // enough past its mean that one spike in the silence pins the extent
        // back to the window edge. Smoothing the primary path instead moves
        // every corpus burst's edges and costs five captures, so it stays out
        // of the case that already works.
        let span = (self.amp.len() / 256).clamp(4, 64);
        boxcar(&mut self.amp, span);
        let (first, last) = edges(&self.amp, top * ratio.max(1e-6).sqrt());
        if last <= first {
            return (0, iq.len());
        }
        (first, last)
    }

    fn measure(&mut self, iq: &[C32]) -> Features {
        let mut f = Features {
            samples: iq.len(),
            duration_us: iq.len() as f64 * 1e6 / self.rate,
            ..Default::default()
        };
        if iq.len() < self.cfg.min_samples {
            return f;
        }

        self.amp.clear();
        self.amp.extend(iq.iter().map(|c| c.norm()));
        let (ratio, modes, top, duty) =
            envelope_levels(&mut self.scratch, &self.amp, &mut self.db);
        f.envelope_ratio = ratio;
        f.envelope_modes = modes;
        f.duty = duty;
        f.kurtosis = kurtosis(&self.amp);

        self.spectrum(iq);
        let (bw, flat) = {
            let spec = std::mem::take(&mut self.spec);
            let r = occupied(&spec, &mut self.scratch, self.rate as f32);
            self.spec = spec;
            r
        };
        f.bandwidth_hz = bw;
        f.flatness = flat;
        f.peakiness = peakiness(&self.spec);
        f.channel_fill = if self.cfg.channel_hz > 0.0 { bw / self.cfg.channel_hz } else { 0.0 };

        // The frequency track only means anything where there is a signal to
        // take the phase difference of. Between the pulses of an OOK burst it
        // is the noise's phase, and letting that into the histogram invents
        // tones out of silence. Holding the last measured value across a gap
        // keeps the track the same length as the burst without inventing a
        // transition at each end of the silence.
        // Half the upper envelope level. A percentile would be the noise floor
        // on a burst that is mostly gaps, and every gap's phase would then
        // reach the tone histogram as if it were a signal.
        let floor = 0.5 * top;
        self.freq.clear();
        self.freq_hi.clear();
        let hz_per_rad = (self.rate / std::f64::consts::TAU) as f32;
        let mut held = 0.0f32;
        for w in iq.windows(2) {
            if w[0].norm() > floor && w[1].norm() > floor {
                let d = w[1] * w[0].conj();
                if d.norm_sqr() > 0.0 {
                    held = d.arg() * hz_per_rad;
                    self.freq_hi.push(held);
                }
            }
            self.freq.push(held);
        }

        // The symbol rate first, because counting levels needs it. At ten
        // decibels the per-sample frequency noise is as large as the spacing
        // between four levels, so a histogram of raw samples has one hump
        // where the signal has four. Averaging over a fraction of a symbol is
        // what opens that up, and a fraction of a symbol is not a length
        // anything knows before the clock is estimated. The two-level case
        // survives without it, which is exactly why the four-level case was
        // the one that failed.
        let (baud, line) = {
            let amp = std::mem::take(&mut self.amp);
            let freq = std::mem::take(&mut self.freq);
            let r = self.symbol_line(iq, &amp, &freq);
            self.amp = amp;
            self.freq = freq;
            r
        };
        f.baud = baud;
        f.baud_line = line;

        let sps = if baud > 0.0 { (self.rate as f32 / baud) as usize } else { 0 };

        // Runs measured on a smoothed envelope. Sample by sample the envelope
        // crosses its own threshold constantly on noise alone, and the median
        // run is then one sample for every signal there is.
        let run = {
            let mut amp = std::mem::take(&mut self.amp);
            boxcar(&mut amp, (sps / 4).clamp(1, 64));
            let r = median_high_run(&amp, top * ratio.max(0.05).sqrt());
            self.amp = amp;
            r
        };
        f.level_run_symbols = if sps > 0 { run / sps as f32 } else { 0.0 };
        let smooth = (sps / 4).clamp(1, 64);
        boxcar(&mut self.freq_hi, smooth);

        let (tones, mut separation) = tone_peaks(&mut self.scratch, &self.freq_hi);

        // Two tones are most of the width they occupy. When the frequency
        // track says otherwise, it is the track that is wrong: at a few
        // samples per symbol its per-sample noise exceeds the deviation, and
        // Bluetooth measured its 500 kHz pair as 72 kHz that way. The
        // spectrum integrates a symbol into each bin and does not have that
        // failure, so it takes over exactly where the track breaks down.
        // Only where the track already found a pair. Asked about a burst it
        // found no tones in, the spectrum answers with whatever structure the
        // sidebands have, and on the corpus that turned amplitude keying into
        // four-level frequency keying. The count stays the track's; what the
        // spectrum corrects is how far apart it put them.
        if tones >= 2 && f.bandwidth_hz > 0.0 && separation < 0.3 * f.bandwidth_hz {
            let spec = std::mem::take(&mut self.spec);
            let (st, ssep) = tones::from_spectrum(&spec, self.rate as f32, 6.0);
            self.spec = spec;
            if st == tones && ssep > separation {
                separation = ssep;
            }
        }
        f.tones = tones;
        f.separation_hz = separation;

        let (fit, rate_hz_s) = chirp_fit(&self.freq_hi, self.rate);
        f.chirp_fit = fit;
        f.chirp_rate = rate_hz_s;

        if baud > 0.0 && separation > 0.0 {
            // The modulation index compares the tone separation, which is
            // twice the peak deviation, with the symbol rate.
            f.mod_index = separation / baud;
        }

        f.square_line = self.power_line(iq, 2);
        f.quartic_line = self.power_line(iq, 4);
        let (pair, sep_hz) = self.line_pair();
        f.quartic_pair = pair;
        // The pair is a symbol rate only for one carrier keyed in phase: two
        // tones give the fourth power two lines as well, a tone's spacing
        // apart, and that is not a clock.
        if pair > 0.0 && f.tones == 1 && f.square_line < 0.2 {
            f.baud = sep_hz;
        }

        // Cyclostationarity last, because it needs the occupied fraction to
        // know which lags are meaningful: inside a signal's own correlation
        // width every burst correlates with itself, and a lag floor of a fixed
        // number of samples measures the bandwidth instead of the structure.
        let occupied = if self.rate > 0.0 {
            (f.bandwidth_hz as f64 / self.rate).clamp(0.001, 1.0) as f32
        } else {
            1.0
        };
        let floor = cyclo::lag_floor(occupied);
        let c = cyclo::complex(iq, floor);
        let e = cyclo::envelope(iq, floor);
        // Below the level noise alone reaches, a peak is not evidence of
        // anything, and reporting it invites a hypothesis to believe it.
        let bound = cyclo::noise_bound(iq.len());
        // A repeat has to repeat: a lag longer than an eighth of the burst has
        // been seen at most eight times, and one at the floor is the signal's
        // own width rather than a period.
        let credible = |k: usize| k >= 2 * floor && iq.len() >= 8 * k.max(1);
        if c.peak > bound && credible(c.lag) {
            f.cyclic = c.peak;
            f.cyclic_ratio = c.ratio;
            f.cyclic_period_s = c.lag as f32 / self.rate as f32;
        }
        if e.peak > 3.0 * bound && credible(e.lag) {
            f.env_cyclic = e.peak;
            f.env_cyclic_ratio = e.ratio;
        }
        f
    }

    /// Welch-averaged power spectrum of the burst, in display order.
    fn spectrum(&mut self, iq: &[C32]) {
        let n = self.cfg.fft_size;
        self.spec.clear();
        self.spec.resize(n, 0.0);
        if iq.len() < n {
            // Too short for a full frame: one zero-padded transform is a
            // coarser estimate but still an honest one.
            self.fft_buf.clear();
            self.fft_buf.extend(iq.iter().copied());
            self.fft_buf.resize(n, C32::new(0.0, 0.0));
            for (i, v) in self.fft_buf.iter_mut().enumerate() {
                *v *= self.win[i];
            }
            self.planner.plan_fft_forward(n).process(&mut self.fft_buf);
            for (i, v) in self.fft_buf.iter().enumerate() {
                self.spec[(i + n / 2) % n] = v.norm_sqr();
            }
            return;
        }

        let hop = n / 2;
        let mut frames = 0.0f32;
        let mut start = 0;
        while start + n <= iq.len() {
            self.fft_buf.clear();
            self.fft_buf.extend(iq[start..start + n].iter().zip(&self.win).map(|(c, w)| *c * *w));
            self.planner.plan_fft_forward(n).process(&mut self.fft_buf);
            for (i, v) in self.fft_buf.iter().enumerate() {
                self.spec[(i + n / 2) % n] += v.norm_sqr();
            }
            frames += 1.0;
            start += hop;
        }
        if frames > 0.0 {
            for v in self.spec.iter_mut() {
                *v /= frames;
            }
        }
    }

    /// Symbol rate from the transition line.
    ///
    /// Every modulation changes something at the symbol boundary and nowhere
    /// else, so the *changes* are an impulse train at the symbol rate whatever
    /// is being keyed. Amplitude keying jumps the envelope, frequency keying
    /// jumps the discriminator, phase keying jumps it harder still, and the
    /// sum of the two normalised jump signals is impulsive for all three. Its
    /// spectrum has a line at the baud.
    ///
    /// The keyed waveform itself does not. Random NRZ data has a spectral
    /// *null* at the symbol rate, which is why estimating the baud from the
    /// signal rather than from its transitions reads noise.
    fn symbol_line(&mut self, iq: &[C32], amp: &[f32], freq: &[f32]) -> (f32, f32) {
        let n = self.cfg.fft_size;
        if iq.len() < n + 2 {
            return (0.0, 0.0);
        }

        // Frequency jumps, on the track measured where there was signal, and
        // envelope jumps, which are all an on-off keyed burst has.
        self.trans.clear();
        self.trans.resize(iq.len() - 1, 0.0);
        // Smoothed before differencing. A discriminator's per-sample noise is
        // hundreds of hertz where a minimum-shift keyed transition moves the
        // tone by a thousand, so the raw difference buries the transition it
        // is there to find.
        add_jumps(&mut self.trans, freq, 4);
        add_jumps(&mut self.trans, amp, 4);

        self.welch_real(n);
        // Between four and a thousand samples per symbol. Below four there is
        // nothing to interpolate and above a thousand a burst holds too few
        // symbols to have a rate. Eight was the limit here for a while, and
        // an 18 kbaud carrier in a stream at 120 kS/s sits under seven: its
        // clock was outside the search and a slot-rate leak in the lowest
        // bin was reported as its symbol rate.
        let lo = (n / 1000).max(2);
        let hi = n / 4;
        if hi <= lo + 2 {
            return (0.0, 0.0);
        }
        let band = &self.spec_real[lo..hi];
        let total: f32 = band.iter().sum();
        if total <= 0.0 {
            return (0.0, 0.0);
        }
        let mean = total / band.len() as f32;
        let mut best = 0usize;
        for (i, &p) in band.iter().enumerate() {
            if p > band[best] {
                best = i;
            }
        }

        // A line at twice the symbol rate is as real as the one at the symbol
        // rate and picking it doubles every estimate downstream, so walk back
        // to the lowest sub-harmonic that is still clearly a line.
        let mut idx = best + lo;
        let mut strength = self.spec_real[idx] / mean;
        while idx.is_multiple_of(2) && idx / 2 >= lo {
            let half = idx / 2;
            let s = self.spec_real[half] / mean;
            let is_peak = self.spec_real[half] >= self.spec_real[half - 1]
                && self.spec_real[half] >= self.spec_real[half + 1];
            if !is_peak || s < strength * 0.5 {
                break;
            }
            idx = half;
            strength = s;
        }

        let baud = idx as f32 * self.rate as f32 / n as f32;
        (baud, strength)
    }

    /// Welch-average the real signal in `self.trans` into `self.spec_real`.
    fn welch_real(&mut self, n: usize) {
        self.spec_real.clear();
        self.spec_real.resize(n / 2, 0.0);
        let hop = n / 2;
        let mut frames = 0.0f32;
        let mut start = 0;
        while start + n <= self.trans.len() {
            let seg = &self.trans[start..start + n];
            let mean = seg.iter().sum::<f32>() / n as f32;
            self.fft_buf.clear();
            self.fft_buf.extend(seg.iter().zip(&self.win).map(|(v, w)| C32::new((v - mean) * w, 0.0)));
            self.planner.plan_fft_forward(n).process(&mut self.fft_buf);
            for i in 0..n / 2 {
                self.spec_real[i] += self.fft_buf[i].norm_sqr();
            }
            frames += 1.0;
            start += hop;
        }
        if frames > 0.0 {
            for v in self.spec_real.iter_mut() {
                *v /= frames;
            }
        }
    }

    /// Share of the power that the strongest bin holds after raising the
    /// signal to `power`.
    ///
    /// Squaring a BPSK signal collapses its two phases onto one and leaves a
    /// tone; the fourth power does the same for QPSK. Nothing else on the band
    /// does that, which makes this the one positive test for phase keying that
    /// does not need a coherent receiver first. Both powers collapse BPSK, so
    /// it is QPSK that is identified by the *absence* of the squared line.
    ///
    /// Amplitude is normalised away first, because the test is about phase and
    /// an on-off keyed burst would otherwise put most of its power in the
    /// strongest bin for no better reason than being loud in the middle.
    fn power_line(&mut self, iq: &[C32], power: u32) -> f32 {
        let n = self.cfg.fft_size;
        if iq.len() < n {
            return 0.0;
        }
        let hop = n / 2;
        self.spec_pow.clear();
        self.spec_pow.resize(n, 0.0);
        let mut frames = 0.0f32;
        let mut start = 0;
        while start + n <= iq.len() {
            self.fft_buf.clear();
            self.fft_buf.extend(iq[start..start + n].iter().zip(&self.win).map(|(c, w)| {
                let m = c.norm();
                if m <= 0.0 {
                    return C32::new(0.0, 0.0);
                }
                let unit = *c / m;
                let mut v = unit;
                for _ in 1..power {
                    v *= unit;
                }
                v * *w
            }));
            self.planner.plan_fft_forward(n).process(&mut self.fft_buf);
            for (i, v) in self.fft_buf.iter().enumerate() {
                self.spec_pow[i] += v.norm_sqr();
            }
            frames += 1.0;
            start += hop;
        }
        if frames == 0.0 {
            return 0.0;
        }

        let total: f32 = self.spec_pow.iter().sum();
        if total <= 0.0 {
            return 0.0;
        }
        // The window spreads a tone over three bins, so a line that is really
        // one tone is undercounted by taking a single bin.
        let mut best = 0.0f32;
        for i in 0..self.spec_pow.len() {
            let a = self.spec_pow[(i + self.spec_pow.len() - 1) % self.spec_pow.len()];
            let c = self.spec_pow[(i + 1) % self.spec_pow.len()];
            best = best.max(a + self.spec_pow[i] + c);
        }
        best / total
    }

    /// Share of the last power-law spectrum held by its strongest line and
    /// a partner of comparable strength elsewhere, and how far apart they
    /// are in hertz.
    ///
    /// Read off `self.spec_pow` as [`Self::power_line`] left it, so it is
    /// asked right after the fourth power. Zero when the strongest line
    /// stands alone, which is what plain phase keying leaves: the second
    /// strongest bin of a spectrum that is one line and noise is noise.
    fn line_pair(&self) -> (f32, f32) {
        let n = self.spec_pow.len();
        if n < 16 {
            return (0.0, 0.0);
        }
        let total: f32 = self.spec_pow.iter().sum();
        if total <= 0.0 {
            return (0.0, 0.0);
        }
        let tri = |i: usize| -> f32 {
            let i = i % n;
            self.spec_pow[(i + n - 1) % n] + self.spec_pow[i] + self.spec_pow[(i + 1) % n]
        };
        let (mut best, mut at) = (0.0f32, 0usize);
        for i in 0..n {
            let t = tri(i);
            if t > best {
                best = t;
                at = i;
            }
        }
        // The partner: a peak of its own, clear of the first line's skirt.
        let guard = 4usize;
        let (mut partner, mut at2) = (0.0f32, 0usize);
        for i in 0..n {
            let d = (i + n - at) % n;
            if d.min(n - d) < guard {
                continue;
            }
            let t = tri(i);
            if t > partner && t >= tri(i + n - 1) && t >= tri(i + 1) {
                partner = t;
                at2 = i;
            }
        }
        // Two lines, not the two tallest bumps of a spectrum that has none:
        // each has to hold a share of the whole that noise never puts in
        // three bins.
        if partner < best * 0.3 || partner < total * 0.05 {
            return (0.0, 0.0);
        }
        let d = (at2 + n - at) % n;
        let sep_bins = d.min(n - d);
        let sep_hz = sep_bins as f64 * self.rate / n as f64;
        ((best + partner) / total, sep_hz as f32)
    }
}

/// Median length of a run of samples above `level`, in samples.
fn median_high_run(amp: &[f32], level: f32) -> f32 {
    let mut runs: Vec<usize> = Vec::new();
    let mut run = 0usize;
    for &v in amp {
        if v >= level {
            run += 1;
        } else if run > 0 {
            runs.push(run);
            run = 0;
        }
    }
    if run > 0 {
        runs.push(run);
    }
    if runs.is_empty() {
        return 0.0;
    }
    runs.sort_unstable();
    runs[runs.len() / 2] as f32
}

/// Add the normalised absolute differences of `src` into `dst`.
///
/// Normalised by the mean jump, so a frequency track measured in tens of
/// kilohertz and an envelope measured in units contribute equally: what is
/// wanted from both is where the jumps are, not how large.
/// Smooth in place with a boxcar of `width` samples.
fn boxcar(v: &mut Vec<f32>, width: usize) {
    if width <= 1 || v.len() <= width {
        return;
    }
    let mut acc = 0.0f32;
    let mut i = 0;
    while i < v.len() {
        acc += v[i];
        if i >= width {
            acc -= v[i - width];
        }
        let n = (i + 1).min(width) as f32;
        let mean = acc / n;
        // Written back behind the read head so the window keeps seeing raw
        // samples: smoothing a signal with its own smoothed history is a
        // different filter, and a much longer one.
        if i >= width {
            v[i - width] = mean;
        }
        i += 1;
    }
    let tail = v.len().saturating_sub(width);
    v.truncate(tail.max(1));
}

fn add_jumps(dst: &mut [f32], src: &[f32], smooth: usize) {
    let w = smooth.max(1);
    if src.len() < 2 * w + 2 {
        return;
    }
    // Difference of two adjacent means of `w` samples: a boxcar smoother and a
    // difference in one pass, so nothing is allocated for the smoothed copy.
    let mean_at = |i: usize| -> f32 { src[i..i + w].iter().sum::<f32>() / w as f32 };
    let n = src.len() - 2 * w;
    let mut total = 0.0f32;
    for i in 0..n {
        total += (mean_at(i + w) - mean_at(i)).abs();
    }
    let mean = total / n as f32;
    if mean <= 0.0 {
        return;
    }
    for i in 0..n.min(dst.len()) {
        dst[i] += (mean_at(i + w) - mean_at(i)).abs() / mean;
    }
}

/// Score every hypothesis and take the best, if it is far enough ahead.
fn decide(f: &Features, cfg: &ClassifyConfig) -> BurstClass {
    if f.samples < cfg.min_samples {
        return BurstClass { modulation: Modulation::Unknown, confidence: 0.0, score: 0.0, features: *f };
    }

    let evidence = Evidence::from(f);
    let scores = hypotheses().iter().map(|h| (h.modulation(), h.score(f, &evidence)));

    let mut best = (Modulation::Unknown, 0.0f32);
    let mut second = 0.0f32;
    for (m, s) in scores {
        if s > best.1 {
            second = best.1;
            best = (m, s);
        } else if s > second {
            second = s;
        }
    }

    let margin = best.1 - second;
    let modulation = if best.1 < cfg.min_score || margin < cfg.min_margin {
        Modulation::Unknown
    } else {
        best.0
    };
    BurstClass { modulation, confidence: margin, score: best.1, features: *f }
}

fn percentile(scratch: &mut Vec<f32>, samples: &[f32], q: f32) -> f32 {
    scratch.clear();
    scratch.extend(samples.iter().copied().filter(|v| v.is_finite()));
    if scratch.is_empty() {
        return 0.0;
    }
    scratch.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    scratch[(((scratch.len() - 1) as f32) * q).round() as usize]
}

fn kurtosis(amp: &[f32]) -> f32 {
    let n = amp.len() as f32;
    if n < 4.0 {
        return 0.0;
    }
    let mean = amp.iter().sum::<f32>() / n;
    let var = amp.iter().map(|v| (v - mean).powi(2)).sum::<f32>() / n;
    if var <= 0.0 {
        return 0.0;
    }
    amp.iter().map(|v| (v - mean).powi(4)).sum::<f32>() / n / (var * var)
}

/// Occupied bandwidth holding 99% of the power, and the spectral flatness
/// inside it.
///
/// The narrowest window containing the power, not the distance between the
/// outermost bins that hold any: a single spur at the channel edge would
/// otherwise report a signal ten times its real width.
fn occupied(spec: &[f32], scratch: &mut Vec<f32>, rate: f32) -> (f32, f32) {
    let n = spec.len();
    if n < 8 {
        return (0.0, 1.0);
    }

    // Against the noise floor, not against the total. A burst detector hands
    // over a window in which the signal occupies part of the channel and noise
    // occupies all of it, so 99% of the raw power is 99% of the channel and
    // every signal measures the same width: the whole span. Subtracting the
    // median bin, which is noise wherever the signal is narrower than half the
    // channel, is what makes the number mean anything.
    scratch.clear();
    scratch.extend_from_slice(spec);
    scratch.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let floor = scratch[n / 2];

    let excess = |i: usize| (spec[i] - floor).max(0.0);
    let total: f32 = (0..n).map(excess).sum();
    if total <= 0.0 {
        return (0.0, 1.0);
    }
    let target = total * 0.99;
    let mut sum = 0.0;
    let mut lo = 0usize;
    let mut best = n;
    let mut best_lo = 0usize;
    for hi in 0..n {
        sum += excess(hi);
        while sum - excess(lo) >= target && lo < hi {
            sum -= excess(lo);
            lo += 1;
        }
        if sum >= target && hi - lo + 1 < best {
            best = hi - lo + 1;
            best_lo = lo;
        }
    }
    let bw = best as f32 * rate / n as f32;

    let band = &spec[best_lo..best_lo + best];
    let mut log_sum = 0.0f64;
    let mut lin_sum = 0.0f64;
    for &v in band {
        log_sum += (v.max(1e-30) as f64).ln();
        lin_sum += v as f64;
    }
    let geo = (log_sum / band.len() as f64).exp();
    let arith = lin_sum / band.len() as f64;
    let flatness = if arith > 0.0 { (geo / arith) as f32 } else { 1.0 };
    (bw, flatness.clamp(0.0, 1.0))
}

/// Share of the power held by the strongest three adjacent bins.
fn peakiness(spec: &[f32]) -> f32 {
    let total: f32 = spec.iter().sum();
    if total <= 0.0 || spec.len() < 8 {
        return 0.0;
    }
    let mut best = 0.0f32;
    for i in 1..spec.len() - 1 {
        best = best.max(spec[i - 1] + spec[i] + spec[i + 1]);
    }
    best / total
}

/// Count the peaks in the histogram of instantaneous frequency, and measure
/// how far apart the outermost two are.
///
/// Returns a count of 1, 2 or 4 only. Three peaks is not a modulation anyone
/// sends, so a count of three means the histogram is being read wrong and the
/// honest answer is 0: no structure found.
fn tone_peaks(scratch: &mut Vec<f32>, freq: &[f32]) -> (u8, f32) {
    let peaks = histogram_peaks(scratch, freq, 0.25);
    match peaks.len() {
        1 => (1, 0.0),
        2 => (2, peaks[1] - peaks[0]),
        4 => (4, peaks[3] - peaks[0]),
        _ => (0, 0.0),
    }
}

/// Where the clusters are in a set of samples, as values rather than counts.
///
/// A histogram, smoothed, with every local maximum that clears `floor` times
/// the tallest peak and is not adjacent to a taller one. `floor` is the caller's
/// because the two uses want different things from it: frequency levels are
/// sent about equally often, so a quarter is a safe cut, while an on-off keyed
/// envelope spends most of a burst in the gaps and its transmitting level is a
/// small fraction of the tallest bin.
fn histogram_peaks(scratch: &mut Vec<f32>, samples: &[f32], floor: f32) -> Vec<f32> {
    if samples.len() < 64 {
        return Vec::new();
    }
    // Trim the outliers before setting the range. One discriminator spike near
    // a zero crossing would otherwise spread the bins over megahertz.
    let lo = percentile(scratch, samples, 0.01);
    let hi = percentile(scratch, samples, 0.99);
    if hi <= lo || !(hi - lo).is_finite() {
        return vec![lo];
    }

    const BINS: usize = 48;
    let mut hist = [0f32; BINS];
    let width = (hi - lo) / BINS as f32;
    for &v in samples {
        if v >= lo && v <= hi {
            let b = (((v - lo) / width) as usize).min(BINS - 1);
            hist[b] += 1.0;
        }
    }
    // Three-bin smoothing: without it, shot noise splits one cluster in two.
    let mut smooth = [0f32; BINS];
    for i in 0..BINS {
        let a = hist[i.saturating_sub(1)];
        let b = hist[i];
        let c = hist[(i + 1).min(BINS - 1)];
        smooth[i] = (a + b + c) / 3.0;
    }
    let peak = smooth.iter().cloned().fold(0.0f32, f32::max);
    if peak <= 0.0 {
        return Vec::new();
    }

    let guard = 2usize;
    let mut bins: Vec<usize> = Vec::new();
    let mut last: Option<usize> = None;
    for i in 0..BINS {
        if smooth[i] < peak * floor {
            continue;
        }
        let from = i.saturating_sub(guard);
        let to = (i + guard).min(BINS - 1);
        if (from..=to).all(|j| smooth[j] <= smooth[i]) && last.is_none_or(|p| i - p > guard) {
            bins.push(i);
            last = Some(i);
        }
    }

    // Two bumps are two clusters only if something separates them. Noise
    // spread alone produces several local maxima on one cluster, and calling
    // those two levels is how a clean constant-envelope signal ends up read as
    // amplitude keying. A real valley drops to half the shorter of the two.
    let mut merged: Vec<usize> = Vec::new();
    for b in bins {
        match merged.last().copied() {
            Some(prev) => {
                let valley = smooth[prev..=b].iter().cloned().fold(f32::INFINITY, f32::min);
                if valley > 0.5 * smooth[prev].min(smooth[b]) {
                    if smooth[b] > smooth[prev] {
                        *merged.last_mut().unwrap() = b;
                    }
                } else {
                    merged.push(b);
                }
            }
            None => merged.push(b),
        }
    }
    merged.iter().map(|&i| lo + (i as f32 + 0.5) * width).collect()
}

/// The envelope's two levels, how far apart they are, and how much of the
/// burst is spent at the upper one.
///
/// Measured from the histogram of the envelope in decibels rather than from
/// percentiles, because percentiles answer a question about duty cycle and the
/// question here is about levels. A packet is mostly silence: a tenth of an
/// on-off keyed burst is transmitting, so its tenth percentile is the noise
/// floor, and so is a frequency-keyed burst's, and the two then look alike
/// while being opposites. Decibels rather than volts because the two clusters
/// are decades apart in amplitude and a linear histogram puts both of them in
/// the bottom two bins.
///
/// Returns the ratio of the levels in amplitude, the number of levels found,
/// the upper level, and the fraction of samples at or above the midpoint.
fn envelope_levels(scratch: &mut Vec<f32>, amp: &[f32], db: &mut Vec<f32>) -> (f32, u8, f32, f32) {
    db.clear();
    db.extend(amp.iter().map(|v| 20.0 * v.max(1e-9).log10()));
    let mut peaks = histogram_peaks(scratch, db, 0.05);
    // Two levels a decibel apart are not two levels. The valley test in
    // `histogram_peaks` asks whether two bumps are separate clusters, which is
    // a different question from whether they are separate *levels*: a tight
    // cluster's own ripple splits into bumps with real valleys between them.
    //
    // Two decibels, not more, and the ceiling is measured: at three the
    // corpus drops from 52 captures to 48, because shallow ASK devices key
    // their two levels only a few decibels apart and merging those loses the
    // keying entirely.
    const MIN_LEVEL_DB: f32 = 2.0;
    let mut kept: Vec<f32> = Vec::new();
    for p in peaks.drain(..) {
        match kept.last_mut() {
            Some(prev) if (p - *prev).abs() < MIN_LEVEL_DB => *prev = prev.max(p),
            _ => kept.push(p),
        }
    }
    let peaks = kept;
    let hi_db = peaks.last().copied().unwrap_or(0.0);
    let hi = 10f32.powf(hi_db / 20.0);
    if peaks.len() < 2 {
        let duty = amp.iter().filter(|v| **v >= hi * 0.5).count() as f32 / amp.len().max(1) as f32;
        return (1.0, peaks.len().max(1) as u8, hi, duty);
    }
    let lo = 10f32.powf(peaks[0] / 20.0);
    let mid = (lo * hi).sqrt();
    let duty = amp.iter().filter(|v| **v >= mid).count() as f32 / amp.len().max(1) as f32;
    (lo / hi, peaks.len().min(255) as u8, hi, duty)
}

/// How much of the burst sweeps at one steady rate, and what that rate is.
///
/// A sweep is not a histogram feature: its frequency visits every value
/// equally, which is exactly what noise does. What separates them is the
/// derivative, constant for a sweep and uncorrelated for noise.
///
/// Measured over blocks rather than per sample. A discriminator's per-sample
/// noise is hundreds of hertz where the sweep advances by tens, so the
/// per-sample slope is all noise and none of it is the signal; the median over
/// a block of samples is not. Blocks also make the test read a sawtooth the
/// way it should, which matters because chirp spread spectrum restarts its
/// sweep every symbol: the wrapping blocks disagree with the median slope and
/// the rest agree, so a sawtooth scores as most of a sweep rather than none.
fn chirp_fit(freq: &[f32], rate: f64) -> (f32, f32) {
    const BLOCK: usize = 32;
    if freq.len() < BLOCK * 8 {
        return (0.0, 0.0);
    }
    let mut medians: Vec<f32> = Vec::with_capacity(freq.len() / BLOCK);
    let mut block = [0f32; BLOCK];
    for chunk in freq.chunks_exact(BLOCK) {
        block.copy_from_slice(chunk);
        block.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        medians.push(block[BLOCK / 2]);
    }

    let slopes: Vec<f32> = medians.windows(2).map(|w| w[1] - w[0]).collect();
    if slopes.len() < 8 {
        return (0.0, 0.0);
    }
    let mut sorted = slopes.clone();
    sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = sorted[sorted.len() / 2];

    // A sweep has to go somewhere: the whole track's span, divided over the
    // blocks it took, is the slope a single pass would need. Anything much
    // flatter than that is a signal sitting still, whatever its noise agrees
    // with.
    // Robust span, not the extremes: a sawtooth's wrap is a slope of the whole
    // sweep in one block, and letting that set the scale hides the sweep it
    // came from.
    let span = sorted[sorted.len() * 9 / 10] - sorted[sorted.len() / 10];
    if span <= 0.0 || median.abs() < span * 0.02 {
        return (0.0, 0.0);
    }
    let tol = median.abs() * 0.5;
    let agree = slopes.iter().filter(|s| (**s - median).abs() <= tol).count();
    (agree as f32 / slopes.len() as f32, median * rate as f32 / BLOCK as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Score a burst dumped from the receiver, for reading why a verdict
    /// came out as it did: `SR_BURST_FILE` names interleaved f32 IQ and
    /// `SR_BURST_RATE` its rate. Does nothing without them.
    #[test]
    fn score_a_dumped_burst() {
        let (Some(path), Some(rate)) = (std::env::var_os("SR_BURST_FILE"), std::env::var("SR_BURST_RATE").ok())
        else {
            return;
        };
        let rate: f64 = rate.parse().unwrap();
        let bytes = std::fs::read(path).unwrap();
        let iq: Vec<C32> = bytes
            .chunks_exact(8)
            .map(|b| {
                C32::new(
                    f32::from_le_bytes(b[0..4].try_into().unwrap()),
                    f32::from_le_bytes(b[4..8].try_into().unwrap()),
                )
            })
            .collect();
        let mut c = Classifier::new(rate, ClassifyConfig::default());
        let f = c.features(&iq);
        eprintln!("features: {f:#?}");
        let e = Evidence::from(&f);
        eprintln!("evidence: {e:#?}");
        for h in hypotheses() {
            eprintln!("{:?}: {:.3}", h.modulation(), h.score(&f, &e));
        }
        let class = decide(&f, &ClassifyConfig::default());
        eprintln!("verdict: {:?} conf {:.2}", class.modulation, class.confidence);
    }

    const RATE: f64 = 200_000.0;

    struct Gen {
        seed: u64,
        noise: f32,
        phase: f64,
        out: Vec<C32>,
    }

    impl Gen {
        fn new(noise: f32) -> Self {
            Self { seed: 0x1234_5678, noise, phase: 0.0, out: Vec::new() }
        }

        fn rng(&mut self) -> f32 {
            self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((self.seed >> 33) as f32 / (1u64 << 30) as f32 - 1.0) * self.noise
        }

        fn bit(&mut self) -> usize {
            self.seed = self.seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((self.seed >> 41) & 3) as usize
        }

        /// Advance `n` samples at frequency `hz` and amplitude `amp`.
        fn tone(&mut self, hz: f64, amp: f32, n: usize) {
            for _ in 0..n {
                self.phase = (self.phase + std::f64::consts::TAU * hz / RATE)
                    .rem_euclid(std::f64::consts::TAU);
                let (s, c) = (self.phase.sin() as f32, self.phase.cos() as f32);
                let (i, q) = (self.rng(), self.rng());
                self.out.push(C32::new(amp * c + i, amp * s + q));
            }
        }

        /// Advance `n` samples at a fixed phase and amplitude, which is what a
        /// phase-keyed symbol is.
        fn phase_symbol(&mut self, carrier_hz: f64, offset: f64, n: usize) {
            for _ in 0..n {
                self.phase = (self.phase + std::f64::consts::TAU * carrier_hz / RATE)
                    .rem_euclid(std::f64::consts::TAU);
                let p = self.phase + offset;
                let (i, q) = (self.rng(), self.rng());
                self.out.push(C32::new(p.cos() as f32 + i, p.sin() as f32 + q));
            }
        }
    }

    fn classify(iq: &[C32]) -> BurstClass {
        let cfg = ClassifyConfig { channel_hz: RATE as f32, ..Default::default() };
        Classifier::new(RATE, cfg).classify(iq)
    }

    /// Samples per symbol at 2000 baud, the rate every generated burst here
    /// keys at unless it says otherwise.
    const SPS: usize = (RATE as usize) / 2_000;

    fn ook(depth: f32) -> Vec<C32> {
        ook_at(depth, 0.01)
    }

    fn ook_at(depth: f32, noise: f32) -> Vec<C32> {
        let mut g = Gen::new(noise);
        for _ in 0..300 {
            let on = g.bit().is_multiple_of(2);
            let amp = if on { 1.0 } else { depth };
            g.tone(1_000.0, amp, SPS);
        }
        g.out
    }

    fn fsk(deviation: f64, levels: usize) -> Vec<C32> {
        fsk_at(deviation, levels, 0.01)
    }

    fn fsk_at(deviation: f64, levels: usize, noise: f32) -> Vec<C32> {
        let mut g = Gen::new(noise);
        for _ in 0..300 {
            let sym = g.bit() % levels;
            let offset = if levels == 2 {
                if sym == 0 { -deviation } else { deviation }
            } else {
                deviation * crate::fourlevel::IDEAL[sym] as f64 / 3.0
            };
            g.tone(offset, 1.0, SPS);
        }
        g.out
    }

    fn psk(states: usize) -> Vec<C32> {
        let mut g = Gen::new(0.01);
        for _ in 0..300 {
            let sym = g.bit() % states;
            let offset = std::f64::consts::TAU * sym as f64 / states as f64;
            g.phase_symbol(500.0, offset, SPS);
        }
        g.out
    }

    /// Four phases shifted by an eighth of a turn every symbol, which is
    /// what TETRA keys.
    fn dqpsk() -> Vec<C32> {
        let mut g = Gen::new(0.01);
        let mut phase = 0.0f64;
        for _ in 0..300 {
            let sym = g.bit() % 4;
            phase += std::f64::consts::TAU * (sym as f64 / 4.0 + 1.0 / 8.0);
            g.phase_symbol(500.0, phase.rem_euclid(std::f64::consts::TAU), SPS);
        }
        g.out
    }

    fn chirp() -> Vec<C32> {
        let mut g = Gen::new(0.01);
        // Eight sweeps across 40 kHz, which is the shape chirp spread spectrum
        // sends: a sawtooth, not one long ramp.
        for _ in 0..8 {
            let n = SPS * 32;
            for k in 0..n {
                let f = -20_000.0 + 40_000.0 * k as f64 / n as f64;
                g.tone(f, 1.0, 1);
            }
        }
        g.out
    }

    fn carrier() -> Vec<C32> {
        let mut g = Gen::new(0.01);
        g.tone(2_000.0, 1.0, SPS * 300);
        g.out
    }

    fn noise_like() -> Vec<C32> {
        // Many carriers at once, which is what OFDM is and what it looks like:
        // Gaussian amplitude, flat spectrum, no symbol clock.
        let mut g = Gen::new(1.0);
        g.tone(0.0, 0.0, SPS * 300);
        g.out
    }

    #[test]
    fn tells_on_off_keying_from_a_carrier() {
        assert_eq!(classify(&ook(0.0)).modulation, Modulation::Ook);
        assert_eq!(classify(&carrier()).modulation, Modulation::Carrier);
    }

    #[test]
    fn tells_shallow_keying_from_deep() {
        // Half amplitude in the low state is 6 dB of depth, which the envelope
        // path latches through and `ask_detect` exists for.
        assert_eq!(classify(&ook(0.5)).modulation, Modulation::Ask);
        assert_eq!(classify(&ook(0.02)).modulation, Modulation::Ook);
    }

    #[test]
    fn tells_two_tones_from_four() {
        assert_eq!(classify(&fsk(20_000.0, 2)).modulation, Modulation::Fsk2);
        assert_eq!(classify(&fsk(20_000.0, 4)).modulation, Modulation::Fsk4);
    }

    #[test]
    fn tells_msk_from_wide_fsk() {
        // Deviation of a quarter the symbol rate is a modulation index of 0.5,
        // which is the definition of MSK.
        let msk = fsk(500.0, 2);
        let wide = fsk(20_000.0, 2);
        assert_eq!(classify(&msk).modulation, Modulation::Msk);
        assert_eq!(classify(&wide).modulation, Modulation::Fsk2);
    }

    #[test]
    fn tells_phase_keying_from_frequency_keying() {
        assert_eq!(classify(&psk(2)).modulation, Modulation::Psk2);
        assert_eq!(classify(&psk(4)).modulation, Modulation::Psk4);
        // The shifted kind is told apart by what its fourth power leaves: a
        // pair of lines a symbol rate apart rather than one. It was read as
        // OFDM off the air, on the slot structure alone, until it was.
        let c = classify(&dqpsk());
        assert_eq!(c.modulation, Modulation::Dqpsk, "{:?}", c.features);
        assert!(c.features.quartic_pair > 0.3, "pair {}", c.features.quartic_pair);
        assert!((c.features.baud - 2_000.0).abs() < 100.0, "baud {}", c.features.baud);
    }

    #[test]
    fn finds_a_sweep_that_a_histogram_cannot() {
        let c = classify(&chirp());
        assert_eq!(c.modulation, Modulation::Chirp);
        assert!(c.features.chirp_rate.abs() > 1e6, "sweep rate came out at {}", c.features.chirp_rate);
    }

    #[test]
    fn calls_a_flat_spectrum_noise_like() {
        assert_eq!(classify(&noise_like()).modulation, Modulation::NoiseLike);
    }

    #[test]
    fn measures_the_bandwidth_and_the_symbol_rate() {
        let c = classify(&fsk(20_000.0, 2));
        assert!(
            (c.features.bandwidth_hz - 45_000.0).abs() < 15_000.0,
            "bandwidth came out at {} Hz",
            c.features.bandwidth_hz
        );
        assert!((c.features.baud - 2_000.0).abs() < 300.0, "baud came out at {}", c.features.baud);
    }

    #[test]
    fn survives_a_signal_ten_db_over_the_noise() {
        // Amplitude 1.0 against 0.3 of noise per component, which is about
        // 10 dB and is a weak burst by the standards of the packets these
        // banks decode.
        assert_eq!(classify(&ook_at(0.0, 0.3)).modulation, Modulation::Ook);
        assert_eq!(classify(&fsk_at(20_000.0, 2, 0.3)).modulation, Modulation::Fsk2);
        assert_eq!(classify(&fsk_at(20_000.0, 4, 0.3)).modulation, Modulation::Fsk4);
    }

    #[test]
    fn a_burst_too_short_to_measure_is_not_guessed_at() {
        let mut g = Gen::new(0.01);
        g.tone(1_000.0, 1.0, 64);
        assert_eq!(classify(&g.out).modulation, Modulation::Unknown);
    }

    #[test]
    fn the_channel_fill_says_when_the_measurement_is_of_the_channel() {
        let cfg = ClassifyConfig { channel_hz: 25_000.0, ..Default::default() };
        let c = Classifier::new(RATE, cfg).classify(&fsk(20_000.0, 2));
        assert!(c.features.channel_fill > 1.0, "fill came out at {}", c.features.channel_fill);
    }
}
