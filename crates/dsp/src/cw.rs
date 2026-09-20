//! CW: a keyed tone, tracked and turned back into marks and gaps.
//!
//! On-off keying of a carrier is the oldest modulation there is and the only
//! one whose symbol clock is a person's wrist. Two things make it awkward
//! where [`crate::pulse::OokDetector`] alone would do:
//!
//! - The keyed carrier is a tone somewhere in the passband, not the passband
//!   itself. An envelope taken over the whole channel is the envelope of
//!   every station in it, so a station two hundred hertz away keys our
//!   timings as surely as the one being read.
//! - Nobody is tuned exactly. Where the tone sits is the offset between the
//!   dial and the transmitter, and an operator zero-beating by ear leaves it
//!   anywhere over a few hundred hertz.
//!
//! So the tone is measured before it is followed: windows of audio go to a
//! [`ToneMeter`], the bin whose level *moves* names the pitch, and the audio
//! is mixed by that pitch and decimated hard, which is the narrow filter. The
//! magnitude of what survives is the envelope, and from there it is
//! [`crate::pulse::OokDetector`] with the time constants of a hand rather
//! than of a sensor.
//!
//! Keyed rather than loudest, because the loudest thing in a CW passband is
//! usually not the station being read. A steady carrier sits at one level
//! and an idling neighbour at another; only a hand on a key swings a bin
//! between noise and signal every few tens of milliseconds. Measured on a
//! synthetic passband, a 1 kHz carrier of the same amplitude as a keyed
//! 600 Hz tone takes every window's peak, and picking the loudest bin reads
//! nothing at all.
//!
//! Out come [`Package`]s of mark and gap microseconds, which is exactly what
//! `decode::morse::decode` reads. Speed is not a setting anywhere here: the
//! decoder measures the dot from the burst, because the operator's speed is
//! whatever it is.

use crate::fir::FirDecim;
use crate::mixer::Mixer;
use crate::pulse::{OokDetector, PulseConfig};
use crate::tone::ToneMeter;
use common::C32;
use common::packet::Detection;

#[derive(Clone, Copy, Debug)]
pub struct CwConfig {
    /// Where the tone may be, in hertz. Below the low end a pitch is felt
    /// rather than heard, and above the high end it is outside what an
    /// operator would tune to.
    pub pitch_hz: (f64, f64),
    /// Half-width of the filter kept around the tracked tone, in hertz.
    pub bandwidth_hz: f64,
    /// Window the pitch is measured over, in microseconds.
    pub window_us: f32,
    /// How far a bin's level must swing across the recent windows before it
    /// is taken for a keyed tone, in dB.
    pub pitch_swing_db: f32,
    /// How far a bin must stand over the median bin to be a candidate at
    /// all, in dB. What keeps the pitch off a noise bin during a silence.
    pub pitch_snr_db: f32,
    /// Silence that ends a transmission, in microseconds.
    pub reset_us: u32,
    /// Marks shorter than this are key clicks or noise, in microseconds.
    pub min_mark_us: u32,
    /// Elements a transmission must hold to be published.
    pub min_pulses: usize,
}

impl Default for CwConfig {
    fn default() -> Self {
        Self {
            // A CW operator tunes for a pitch they can hear against the
            // noise, which in practice is 400 to 1000 Hz; the range is wider
            // than that because the dial is not where the pitch is.
            pitch_hz: (250.0, 1_800.0),
            // Wide enough for the keying sidebands of the fastest fist and
            // narrow enough to lose the next station up: at 40 wpm a dot is
            // 30 ms, whose spectrum is a few tens of hertz wide.
            bandwidth_hz: 120.0,
            // 32 ms, which is a 31 Hz bin and still shorter than the 30 ms
            // dot of a 40 wpm fist, so a mark fills at least one window.
            window_us: 32_000.0,
            // Measured on a synthetic passband at 8 kHz, over 32 ms windows:
            // a keyed 600 Hz tone swings its bin by 57 dB between mark and
            // gap and a steady carrier by 0.0 dB, so anything in between
            // separates them. A noise bin swings as much as 46 dB, because
            // one window in fifty lands near a null, which is why the swing
            // is not the whole test; see `follow`.
            pitch_swing_db: 15.0,
            pitch_snr_db: 6.0,
            // A word gap at 5 wpm is 1.7 s, so anything under two seconds
            // would cut a slow sender's transmission into words.
            reset_us: 2_000_000,
            // A dot at 60 wpm is 20 ms, and half of one is the shortest
            // thing that can be a symbol here.
            min_mark_us: 10_000,
            // Two elements is a letter. Five is about two letters, which is
            // the least that is worth showing somebody.
            min_pulses: 5,
        }
    }
}

/// A keyed tone in, marks and gaps out.
pub struct CwDetector {
    cfg: CwConfig,
    rate: f64,
    tone_hz: f64,
    /// Whether a keyed tone has been found at all. Before it has, the
    /// filter is sitting on the middle of the search range and whatever it
    /// passes is not evidence of anything.
    tuned: bool,
    meter: ToneMeter,
    mixer: Mixer,
    decim: FirDecim,
    ook: OokDetector,
    /// Audio being gathered towards the next pitch window.
    window: Vec<f32>,
    /// The bins the pitch may be in, as a half-open range.
    bins: (usize, usize),
    /// Magnitudes of those bins for each recent window, oldest first.
    history: std::collections::VecDeque<Vec<f32>>,
    mags: Vec<f32>,
    /// Windows waiting to be filtered, oldest first. See [`MIN_HISTORY`].
    pending: std::collections::VecDeque<Vec<f32>>,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    envelope: Vec<f32>,
}

/// Windows of history behind the pitch test. A second and a half at the
/// default window, which holds the gaps around a letter at any speed a
/// person sends at.
const LEVEL_HISTORY: usize = 48;

/// Windows before the pitch is chosen at all, and so also the number held
/// back before they are filtered.
///
/// The pitch cannot be known until some of the transmission has arrived, and
/// re-tuning the filter re-seeds the levels on the noise the new tone sits
/// in, which takes a further time constant. Both of those happen while the
/// operator is sending, so without a delay the first element is gone:
/// measured on a synthetic 20 wpm `SOS`, reading each window as it arrives
/// returns 8 of the 9 elements and the delay returns all 9.
const MIN_HISTORY: usize = 3;

/// The filter around the tracked tone, and the decimation to the envelope.
///
/// Designed from the bandwidth wanted rather than from what would alias
/// ([`FirDecim::design_band`]), which is the difference between hearing one
/// station and hearing the passband: measured, a steady carrier 400 Hz from
/// the tone survived a filter designed the other way, held the gate closed,
/// and nothing decoded at all. Here it is 60 dB down.
fn narrow(rate: f64, factor: usize, bandwidth_hz: f64) -> FirDecim {
    let pass = bandwidth_hz.min(rate * 0.2);
    FirDecim::design_band(rate, factor, pass, pass * 2.0, 60.0)
}

impl CwDetector {
    pub fn new(rate: f64, cfg: CwConfig) -> Self {
        let window = ((cfg.window_us as f64 * rate / 1e6) as usize).max(64);
        // Four samples across the shortest mark survive the decimation: a
        // 10 ms mark at a 1 kHz envelope is ten samples.
        let factor = (rate / 1_000.0).floor().max(1.0) as usize;
        let envelope_rate = rate / factor as f64;
        let pitch = 0.5 * (cfg.pitch_hz.0 + cfg.pitch_hz.1);
        let meter = ToneMeter::new(rate);
        let bins = (meter.bin_of(cfg.pitch_hz.0, window), meter.bin_of(cfg.pitch_hz.1, window) + 1);
        Self {
            cfg,
            rate,
            tone_hz: pitch,
            tuned: false,
            bins,
            meter,
            mixer: Mixer::new(-pitch, rate),
            decim: narrow(rate, factor, cfg.bandwidth_hz),
            ook: OokDetector::new(
                envelope_rate,
                PulseConfig {
                    reset_us: cfg.reset_us,
                    min_mark_us: cfg.min_mark_us,
                    min_pulses: cfg.min_pulses,
                    // A hand-sent mark is tens of milliseconds, so the
                    // envelope estimator may be a hundred times slower than
                    // the one that follows a sensor's 100 us symbol.
                    tau_us: 50_000.0,
                    hysteresis: 0.3,
                    min_snr_db: 6.0,
                    // The keying is not a dropout to be healed: a gap
                    // between two dots is a symbol in its own right and
                    // merging it destroys the letter.
                    merge_dropouts: false,
                    ..PulseConfig::default()
                },
            ),
            window: Vec::with_capacity(window),
            history: std::collections::VecDeque::with_capacity(LEVEL_HISTORY),
            mags: Vec::new(),
            pending: std::collections::VecDeque::with_capacity(MIN_HISTORY + 1),
            mixed: Vec::new(),
            narrow: Vec::new(),
            envelope: Vec::new(),
        }
    }

    /// The pitch being followed, in hertz. What the operator is mistuned by,
    /// give or take the transmitter's own offset.
    pub fn tone_hz(&self) -> f64 {
        self.tone_hz
    }

    pub fn snr_db(&self) -> f32 {
        self.ook.snr_db()
    }

    /// Samples in the window the pitch is measured over.
    fn window_len(&self) -> usize {
        ((self.cfg.window_us as f64 * self.rate / 1e6) as usize).max(64)
    }

    pub fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.ook.reset();
        self.window.clear();
        self.history.clear();
        self.pending.clear();
        self.tuned = false;
    }

    /// Feed a block of audio, appending completed transmissions to `out`.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Detection>) {
        let want = self.window_len();
        let mut rest = audio;
        while !rest.is_empty() {
            let take = want.saturating_sub(self.window.len()).min(rest.len());
            self.window.extend_from_slice(&rest[..take]);
            rest = &rest[take..];
            if self.window.len() < want {
                break;
            }
            let window = std::mem::take(&mut self.window);
            self.follow(&window);
            self.pending.push_back(window);
            if self.pending.len() > MIN_HISTORY {
                let due = self.pending.pop_front().unwrap_or_default();
                self.read(&due, out);
                self.window = due;
                self.window.clear();
            }
        }
    }

    /// Publish whatever is still open, for the end of a stream.
    pub fn flush(&mut self, out: &mut Vec<Detection>) {
        while let Some(due) = self.pending.pop_front() {
            self.read(&due, out);
        }
        if !self.window.is_empty() {
            let window = std::mem::take(&mut self.window);
            self.read(&window, out);
            self.window = window;
            self.window.clear();
        }
        self.ook.flush(out);
    }

    /// Move the filter onto the bin that is being keyed.
    ///
    /// Every window's spectrum is kept for a second and a half, and the bin
    /// whose level swings furthest across that history wins, provided it
    /// also stands over the median bin so that a silence cannot promote
    /// noise. Nothing qualifying leaves the pitch where it was, so the
    /// filter stays on the station through its gaps instead of wandering
    /// off after whatever was loudest in them.
    fn follow(&mut self, window: &[f32]) {
        self.mags.clear();
        self.meter.magnitudes(window, &mut self.mags);
        let (lo, hi) = (self.bins.0, self.bins.1.min(self.mags.len()));
        if lo >= hi {
            return;
        }
        if self.history.len() == LEVEL_HISTORY {
            self.history.pop_front();
        }
        self.history.push_back(self.mags[lo..hi].to_vec());
        if self.history.len() < MIN_HISTORY {
            return;
        }

        let width = hi - lo;
        let mut peaks: Vec<f32> = Vec::with_capacity(width);
        let mut best: Option<(f32, usize)> = None;
        for k in 0..width {
            let mut top = 0.0f32;
            let mut bottom = f32::MAX;
            for w in &self.history {
                top = top.max(w[k]);
                bottom = bottom.min(w[k]);
            }
            peaks.push(top);
            let swing = 20.0 * (top.max(1e-12) / bottom.max(1e-12)).log10();
            // Of the bins that are keyed, the loudest. Not the one that
            // swings furthest: a tone on a bin centre leaks into its
            // neighbour, and the neighbour's quiet level is lower, so it
            // swings further than the tone itself and the pitch lands a bin
            // and a half out.
            if swing >= self.cfg.pitch_swing_db && best.is_none_or(|(p, _)| top > p) {
                best = Some((top, k));
            }
        }
        let Some((_, k)) = best else { return };
        let mut sorted = peaks.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = sorted[sorted.len() / 2];
        let over = 20.0 * (peaks[k].max(1e-12) / median.max(1e-12)).log10();
        if over < self.cfg.pitch_snr_db {
            return;
        }

        // Interpolated on the window the bin was loudest in rather than on
        // the one in hand, which is as likely as not a gap where that bin
        // holds noise and its neighbours say nothing about the pitch.
        let loudest = self
            .history
            .iter()
            .max_by(|a, b| a[k].partial_cmp(&b[k]).unwrap_or(std::cmp::Ordering::Equal))
            .map(|w| w.as_slice())
            .unwrap_or(&self.mags[lo..hi]);
        let hz = self.meter.bin_hz(loudest, k, window.len())
            + lo as f64 * self.rate / window.len() as f64;
        if hz < self.cfg.pitch_hz.0 || hz > self.cfg.pitch_hz.1 {
            return;
        }
        // The mixer phase is continuous across a retune and the envelope
        // is a magnitude, so a step in the pitch costs nothing. What a step
        // does cost is the levels: they were measured on whatever the old
        // filter passed, and after a move of more than the filter's own
        // width that is a different signal. Measured with them kept, a
        // neighbouring carrier passed by the filter's first position sets
        // the noise estimate to its own level, and the station the filter
        // then moves onto stays under the threshold for a second and a
        // half, which is most of a transmission.
        if (hz - self.tone_hz).abs() > 1.0 {
            if (hz - self.tone_hz).abs() > self.cfg.bandwidth_hz {
                self.ook.reset();
            }
            self.tone_hz = hz;
            self.mixer.set_shift(-hz, self.rate);
        }
        self.tuned = true;
    }

    /// Mix the window down by the tracked pitch, narrow it, and hand the
    /// magnitude to the pulse detector.
    fn read(&mut self, window: &[f32], out: &mut Vec<Detection>) {
        if !self.tuned {
            return;
        }
        self.mixed.clear();
        self.mixed.extend(window.iter().map(|s| C32::new(*s, 0.0)));
        self.mixer.process_in_place(&mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.envelope.clear();
        self.envelope.extend(self.narrow.iter().map(|c| c.norm()));
        self.ook.process(&self.envelope, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Pulse;

    const RATE: f64 = 8_000.0;

    /// A keyed tone: the pulses of `pkg` at `hz`, with noise throughout and
    /// a lead-in and lead-out of it either side.
    pub(super) fn keyed(pkg: &[Pulse], hz: f64, amp: f32, noise: f32) -> Vec<f32> {
        let mut seed = 0x2b3c_4d5e_6f70_8192u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            ((seed >> 40) as f32 / 8_388_608.0 - 1.0) * noise
        };
        let samples = |us: u32| (us as f64 * RATE / 1e6).round() as usize;
        let mut out: Vec<f32> = Vec::new();
        let mut n = 0usize;
        let push = |out: &mut Vec<f32>,
                    count: usize,
                    on: bool,
                    n: &mut usize,
                    rng: &mut dyn FnMut() -> f32| {
            for _ in 0..count {
                let t = *n as f64 / RATE;
                let tone = match on {
                    true => amp * (std::f64::consts::TAU * hz * t).sin() as f32,
                    false => 0.0,
                };
                out.push(tone + rng());
                *n += 1;
            }
        };
        push(&mut out, samples(500_000), false, &mut n, &mut rng);
        for p in pkg {
            push(&mut out, samples(p.mark), true, &mut n, &mut rng);
            push(&mut out, samples(p.gap), false, &mut n, &mut rng);
        }
        push(&mut out, samples(500_000), false, &mut n, &mut rng);
        out
    }

    /// The timings of `SOS` at 20 wpm: a dot is 60 ms.
    pub(super) fn sos() -> Vec<Pulse> {
        let dot = 60_000u32;
        let mut p = Vec::new();
        for (i, el) in "...---...".chars().enumerate() {
            let mark = if el == '-' { dot * 3 } else { dot };
            // Characters are three dots apart, elements one.
            let gap = if i == 2 || i == 5 { dot * 3 } else { dot };
            p.push(Pulse { mark, gap });
        }
        p.last_mut().unwrap().gap = dot * 7;
        p
    }

    fn run(det: &mut CwDetector, audio: &[f32]) -> Vec<Detection> {
        let mut out = Vec::new();
        for block in audio.chunks(1_024) {
            det.process(block, &mut out);
        }
        det.flush(&mut out);
        out
    }

    /// One transmission out, with its nine elements, and the marks measured
    /// back to within a tenth of a dot.
    #[test]
    fn a_keyed_tone_comes_back_as_its_elements() {
        let audio = keyed(&sos(), 700.0, 0.5, 0.02);
        let mut det = CwDetector::new(RATE, CwConfig::default());
        let out = run(&mut det, &audio);
        assert_eq!(out.len(), 1, "{} transmissions", out.len());
        assert_eq!(out[0].pulses().len(), 9, "{} elements", out[0].pulses().len());
        for (got, want) in out[0].pulses().iter().zip(sos()) {
            let err = got.mark as i64 - want.mark as i64;
            assert!(err.abs() < 6_000, "a {} us mark read as {} us", want.mark, got.mark);
        }
        assert!((det.tone_hz() - 700.0).abs() < 20.0, "pitch read as {:.0} Hz", det.tone_hz());
    }

    /// Nobody is tuned exactly, so the pitch is found rather than assumed.
    /// Measured: every pitch from 300 to 1500 Hz reads its nine elements,
    /// and one outside the range declared in the config reads nothing at
    /// all rather than reading something else.
    #[test]
    fn the_pitch_is_found_wherever_the_operator_left_it() {
        for hz in [300.0, 500.0, 700.0, 1_000.0, 1_500.0] {
            let audio = keyed(&sos(), hz, 0.5, 0.02);
            let mut det = CwDetector::new(RATE, CwConfig::default());
            let out = run(&mut det, &audio);
            assert_eq!(out.len(), 1, "{hz} Hz: {} transmissions", out.len());
            assert_eq!(out[0].pulses().len(), 9, "{hz} Hz: {} elements", out[0].pulses().len());
            assert!((det.tone_hz() - hz).abs() < 25.0, "{hz} Hz read as {:.0}", det.tone_hz());
        }
        let audio = keyed(&sos(), 2_400.0, 0.5, 0.02);
        let mut det = CwDetector::new(RATE, CwConfig::default());
        assert_eq!(run(&mut det, &audio).len(), 0, "a tone outside the range was read");
    }

    /// A second station in the passband keys nothing: the filter follows one
    /// pitch, and the other is 400 Hz away and 60 dB down by the time the
    /// envelope is taken.
    #[test]
    fn a_station_beside_the_one_being_read_does_not_key_it() {
        let wanted = keyed(&sos(), 600.0, 0.5, 0.02);
        // A continuous carrier is the worst neighbour there is: it keys
        // nothing itself and would hold the envelope high for ever.
        let mut mixed = wanted.clone();
        for (i, s) in mixed.iter_mut().enumerate() {
            let t = i as f64 / RATE;
            *s += 0.5 * (std::f64::consts::TAU * 1_000.0 * t).sin() as f32;
        }
        let mut det = CwDetector::new(RATE, CwConfig::default());
        let out = run(&mut det, &mixed);
        assert_eq!(out.len(), 1, "{} transmissions beside a carrier", out.len());
        assert_eq!(out[0].pulses().len(), 9, "{} elements", out[0].pulses().len());
    }

    /// The numbers behind the pitch test, measured on the same synthetic
    /// passband the detector is checked on: what a hand on a key does to a
    /// bin, what a steady carrier does, and what noise alone does.
    ///
    /// The last is why the swing is not the whole test. A noise bin swings
    /// nearly as far as a keyed one, because over fifty windows one of them
    /// lands in a null, so the level over the median bin has to decide as
    /// well.
    #[test]
    fn a_keyed_bin_swings_where_a_carrier_does_not() {
        let n = 256;
        let mut meter = ToneMeter::new(RATE);
        let swing = |meter: &mut ToneMeter, sig: &[f32], bin: usize, skip: usize| {
            let (mut top, mut bottom) = (0.0f32, f32::MAX);
            let mut mags = Vec::new();
            for win in sig.chunks(n).skip(skip).take(48) {
                if win.len() < n {
                    continue;
                }
                mags.clear();
                meter.magnitudes(win, &mut mags);
                top = top.max(mags[bin]);
                bottom = bottom.min(mags[bin]);
            }
            20.0 * (top / bottom.max(1e-12)).log10()
        };

        let cw = keyed(&sos(), 600.0, 0.5, 0.02);
        let bin = meter.bin_of(600.0, n);
        let keyed_db = swing(&mut meter, &cw, bin, 16);
        assert!(keyed_db > 40.0, "a keyed bin swung only {keyed_db:.1} dB");

        let carrier: Vec<f32> = (0..cw.len())
            .map(|i| 0.5 * (std::f64::consts::TAU * 1_000.0 * i as f64 / RATE).sin() as f32)
            .collect();
        let bin = meter.bin_of(1_000.0, n);
        let steady_db = swing(&mut meter, &carrier, bin, 16);
        assert!(steady_db < 3.0, "a steady carrier swung {steady_db:.1} dB");

        let noise = keyed(&[], 0.0, 0.0, 0.02);
        let worst = (8..58).map(|b| swing(&mut meter, &noise, b, 0)).fold(0.0f32, f32::max);
        assert!(worst > 20.0, "noise swung only {worst:.1} dB, so the median test is idle");
    }

    /// Two minutes of noise produces nothing. There is no check anywhere in
    /// Morse, so what refuses a transmission is the level test and the
    /// element count and nothing else.
    #[test]
    fn noise_produces_no_transmissions() {
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let audio: Vec<f32> = (0..(RATE as usize * 120)).map(|_| 0.05 * rng()).collect();
        let mut det = CwDetector::new(RATE, CwConfig::default());
        let out = run(&mut det, &audio);
        assert_eq!(out.len(), 0, "{} transmissions out of two minutes of noise", out.len());
    }
}
