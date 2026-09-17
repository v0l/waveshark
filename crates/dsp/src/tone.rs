//! What frequency is this piece of audio?
//!
//! One question, asked of a short window of real samples: the strongest tone
//! in it, to better than a bin. Anything that carries its information in the
//! frequency of an audio tone wants this, which is SSTV today and weather fax
//! or a tone-keyed telemetry link tomorrow.
//!
//! A Hann window, a real FFT, the largest bin, and a barycentric
//! interpolation across its neighbours for the fraction of a bin. The
//! interpolation is what makes a 4 ms window good enough to tell 1500 Hz from
//! 1503 Hz, which is one shade of grey in an SSTV picture.

use rustfft::num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::collections::HashMap;
use std::sync::Arc;

pub struct ToneMeter {
    rate: f64,
    planner: FftPlanner<f32>,
    plans: HashMap<usize, Arc<dyn Fft<f32>>>,
    window: HashMap<usize, Vec<f32>>,
    buf: Vec<Complex32>,
}

impl ToneMeter {
    pub fn new(rate: f64) -> Self {
        Self {
            rate,
            planner: FftPlanner::new(),
            plans: HashMap::new(),
            window: HashMap::new(),
            buf: Vec::new(),
        }
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// The strongest tone in `samples`, in hertz. Windows of any length are
    /// allowed and each length's plan and window are kept, since a decoder
    /// asks the same few lengths many thousands of times.
    pub fn peak_hz(&mut self, samples: &[f32]) -> f64 {
        let n = samples.len();
        let mut mags = Vec::new();
        self.magnitudes(samples, &mut mags);
        let peak = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);
        self.bin_hz(&mags, peak, n)
    }

    /// Every bin of the window's magnitude spectrum up to Nyquist, appended
    /// to `out`. For a caller choosing a tone by something other than which
    /// is loudest: a keyed tone is the one whose level moves, and only the
    /// whole spectrum says which that is.
    pub fn magnitudes(&mut self, samples: &[f32], out: &mut Vec<f32>) {
        let n = samples.len();
        if n < 4 {
            return;
        }
        let planner = &mut self.planner;
        let fft = self.plans.entry(n).or_insert_with(|| planner.plan_fft_forward(n)).clone();
        let window = self.window.entry(n).or_insert_with(|| hann(n));

        self.buf.clear();
        self.buf.extend(samples.iter().zip(window.iter()).map(|(s, w)| Complex32::new(s * w, 0.0)));
        fft.process(&mut self.buf);

        // Real input, so only the first half says anything.
        let half = n / 2 + 1;
        out.extend(self.buf[..half].iter().map(|c| c.norm()));
    }

    /// Where a peak at bin `at` of an `n` point window really sits, in
    /// hertz, with its neighbours saying where between the bins it is.
    pub fn bin_hz(&self, mags: &[f32], at: usize, n: usize) -> f64 {
        if mags.is_empty() || n == 0 {
            return 0.0;
        }
        (interpolate(mags, at.min(mags.len() - 1)) * self.rate) / n as f64
    }

    /// The bin a frequency falls in, for a window of `n` samples.
    pub fn bin_of(&self, hz: f64, n: usize) -> usize {
        ((hz * n as f64 / self.rate).round().max(0.0)) as usize
    }
}

/// The peak's position in bins, using the two neighbours to find where
/// between them it really is.
fn interpolate(mags: &[f32], at: usize) -> f64 {
    let left = if at == 0 { mags[at] } else { mags[at - 1] };
    let right = if at + 1 >= mags.len() { mags[at] } else { mags[at + 1] };
    let denom = left + mags[at] + right;
    if denom == 0.0 {
        return 0.0;
    }
    at as f64 + ((right - left) / denom) as f64
}

fn hann(n: usize) -> Vec<f32> {
    // The symmetric window, as every decoder this is checked against uses.
    (0..n)
        .map(|i| {
            let x = std::f64::consts::PI * i as f64 / (n - 1) as f64;
            (x.sin() * x.sin()) as f32
        })
        .collect()
}

/// One steady tone, heard from `start_s` for `seconds`.
///
/// What a sequential tone scheme sends: a paging A tone, the B tone after
/// it, a long group tone. The run is closed when the tone stops or moves,
/// so its length is evidence about what was sent rather than a window
/// length chosen here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Run {
    pub hz: f64,
    pub start_s: f64,
    pub seconds: f64,
    /// Peak magnitude of the tone across the run, for a level readout.
    pub level: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct RunConfig {
    /// The window a tone is measured over. 25 ms is a 40 Hz bin before
    /// interpolation, which separates the closest pair in a Motorola tone
    /// set (about 6% apart, so 17 Hz at the bottom of the range).
    pub window_s: f64,
    /// Tones outside this are not a signalling tone: below is FM rumble and
    /// above is the top of a voice channel.
    pub band_hz: (f64, f64),
    /// How much of a window's energy has to be in the peak and its two
    /// neighbours before the window is called a tone.
    ///
    /// Measured on a 25 ms window at 8 kS/s: a pure tone scores 0.994, and
    /// synthetic speech with ten harmonics on a moving pitch scores 0.635 to
    /// 0.645 window by window. Real speech spreads wider than that, so the
    /// threshold sits above the synthetic worst case rather than between.
    pub purity: f32,
    /// Below this peak magnitude, relative to a full scale sine, the window
    /// is silence.
    pub floor: f32,
    /// How far a tone may wander between windows and still be the same run,
    /// as a fraction of its frequency. A transmitter's tone is within a
    /// tenth of a percent; this is the reading's own spread.
    pub drift: f64,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self { window_s: 0.025, band_hz: (250.0, 3_000.0), purity: 0.85, floor: 0.02, drift: 0.02 }
    }
}

/// Audio in, steady tones out.
pub struct ToneRuns {
    meter: ToneMeter,
    cfg: RunConfig,
    window: usize,
    held: Vec<f32>,
    mags: Vec<f32>,
    fed_s: f64,
    open: Option<Run>,
}

impl ToneRuns {
    pub fn new(rate: f64, cfg: RunConfig) -> Self {
        let window = ((rate * cfg.window_s).round() as usize).max(16);
        Self {
            meter: ToneMeter::new(rate),
            cfg,
            window,
            held: Vec::new(),
            mags: Vec::new(),
            fed_s: 0.0,
            open: None,
        }
    }

    pub fn reset(&mut self) {
        self.held.clear();
        self.open = None;
        self.fed_s = 0.0;
    }

    /// The run that is still being heard, for a readout.
    pub fn open(&self) -> Option<Run> {
        self.open
    }

    /// Feed a block of audio, appending every run that ended inside it.
    pub fn process(&mut self, audio: &[f32], out: &mut Vec<Run>) {
        self.held.extend_from_slice(audio);
        let rate = self.meter.rate();
        let step = self.window as f64 / rate;
        let mut at = 0;
        while at + self.window <= self.held.len() {
            let heard = self.measure(at);
            match (heard, self.open) {
                (Some((hz, level)), Some(mut run))
                    if (hz - run.hz).abs() <= run.hz * self.cfg.drift =>
                {
                    // The same tone: lengthen it, and let the frequency
                    // settle towards what the longer look says.
                    run.seconds += step;
                    run.hz += (hz - run.hz) * step / run.seconds;
                    run.level = run.level.max(level);
                    self.open = Some(run);
                }
                (heard, was) => {
                    if let Some(run) = was {
                        out.push(run);
                    }
                    self.open = heard.map(|(hz, level)| Run {
                        hz,
                        start_s: self.fed_s,
                        seconds: step,
                        level,
                    });
                }
            }
            self.fed_s += step;
            at += self.window;
        }
        self.held.drain(..at);
    }

    /// The tone in one window, where there is one: its frequency and level.
    fn measure(&mut self, at: usize) -> Option<(f64, f32)> {
        self.mags.clear();
        let window: Vec<f32> = self.held[at..at + self.window].to_vec();
        let mut mags = std::mem::take(&mut self.mags);
        self.meter.magnitudes(&window, &mut mags);
        let out = self.peak(&mags);
        self.mags = mags;
        out
    }

    fn peak(&self, mags: &[f32]) -> Option<(f64, f32)> {
        let (peak, level) = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, v)| (i, *v))?;
        // The window is Hann weighted, so a full scale sine peaks at a
        // quarter of the window length.
        if level < self.cfg.floor * self.window as f32 / 4.0 {
            return None;
        }
        let total: f32 = mags.iter().map(|m| m * m).sum();
        let near: f32 =
            mags[peak.saturating_sub(1)..(peak + 2).min(mags.len())].iter().map(|m| m * m).sum();
        if total <= 0.0 || near / total < self.cfg.purity {
            return None;
        }
        let hz = self.meter.bin_hz(mags, peak, self.window);
        (self.cfg.band_hz.0..=self.cfg.band_hz.1).contains(&hz).then_some((hz, level))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f64, rate: f64, n: usize) -> Vec<f32> {
        (0..n).map(|i| (std::f64::consts::TAU * hz * i as f64 / rate).sin() as f32).collect()
    }

    /// A tone between two bins is found between them, which is the whole
    /// reason for the interpolation: at these window lengths a bin is 50 Hz.
    /// The barycentric estimate is biased towards the bin centre, so a tone
    /// half way between two bins is the worst case and lands about 6 Hz low;
    /// that bias is what the reference decoders have too.
    #[test]
    fn a_tone_off_the_bin_grid_is_still_read_closely() {
        let rate = 44_100.0;
        let n = 882; // 20 ms, a 50 Hz bin
        let mut m = ToneMeter::new(rate);
        for hz in [1200.0, 1500.0, 1900.0, 2300.0, 1723.0] {
            let got = m.peak_hz(&tone(hz, rate, n));
            assert!((got - hz).abs() < 8.0, "{hz} Hz read as {got:.1}");
        }
    }

    /// The window a picture is sampled with is a few hundred samples, and
    /// that is where the estimate has to hold up.
    #[test]
    fn a_short_window_still_names_the_tone() {
        let rate = 44_100.0;
        let n = 215; // about 4.9 ms, which is one Martin 1 pixel window
        let mut m = ToneMeter::new(rate);
        for hz in [1500.0, 1800.0, 2300.0] {
            let got = m.peak_hz(&tone(hz, rate, n));
            assert!((got - hz).abs() < 25.0, "{hz} Hz read as {got:.1}");
        }
    }

    const RATE: f64 = 8_000.0;

    fn silence(seconds: f64) -> Vec<f32> {
        vec![0.0; (RATE * seconds) as usize]
    }

    fn held(hz: f64, seconds: f64) -> Vec<f32> {
        let n = (RATE * seconds) as usize;
        (0..n).map(|i| 0.5 * (std::f64::consts::TAU * hz * i as f64 / RATE).sin() as f32).collect()
    }

    /// Somebody talking: ten harmonics on a pitch that moves, which is the
    /// thing a tone detector on a voice channel has to refuse.
    fn talking(seconds: f64) -> Vec<f32> {
        let n = (RATE * seconds) as usize;
        let mut phase = vec![0.0f64; 11];
        (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                let pitch = 190.0 + 40.0 * (std::f64::consts::TAU * 2.0 * t).sin();
                let mut v = 0.0;
                for h in 1..=10 {
                    phase[h] += std::f64::consts::TAU * pitch * h as f64 / RATE;
                    v += phase[h].sin() / h as f64;
                }
                (v * 0.4) as f32
            })
            .collect()
    }

    /// A tone held, then another: two runs with the frequencies and the
    /// lengths that were sent, which is everything a paging decode reads.
    #[test]
    fn two_tones_in_sequence_are_two_runs_of_the_lengths_sent() {
        let mut r = ToneRuns::new(RATE, RunConfig::default());
        let mut out = Vec::new();
        r.process(&silence(0.2), &mut out);
        r.process(&held(947.3, 1.0), &mut out);
        r.process(&held(332.5, 3.0), &mut out);
        r.process(&silence(0.3), &mut out);
        assert_eq!(out.len(), 2, "{out:?}");
        // A 25 ms window is a 40 Hz bin, and the barycentric peak is biased
        // towards the bin centre: measured, 947.3 Hz reads 950.7 and 332.5
        // reads 331.9, so a few hertz is as close as this window comes.
        assert!((out[0].hz - 947.3).abs() < 5.0, "A read as {:.1}", out[0].hz);
        assert!((out[1].hz - 332.5).abs() < 5.0, "B read as {:.1}", out[1].hz);
        // A window either side is the most the boundaries can cost.
        assert!((out[0].seconds - 1.0).abs() <= 0.05, "A ran {:.3} s", out[0].seconds);
        assert!((out[1].seconds - 3.0).abs() <= 0.05, "B ran {:.3} s", out[1].seconds);
        assert!((out[0].start_s - 0.2).abs() <= 0.05, "A started at {:.3}", out[0].start_s);
    }

    /// Speech is not a tone, and neither is silence. This is the test the
    /// purity threshold exists for: a voice channel is speech nearly all the
    /// time and a run off it would be a page nobody sent.
    #[test]
    fn speech_and_silence_produce_no_runs() {
        let mut r = ToneRuns::new(RATE, RunConfig::default());
        let mut out = Vec::new();
        r.process(&talking(6.0), &mut out);
        r.process(&silence(6.0), &mut out);
        assert_eq!(out.len(), 0, "{out:?}");
    }

    /// Two tones a Motorola set puts next to each other are two runs, not
    /// one that wandered: the closest pair in a group is about 6% apart and
    /// the drift allowance is 2%.
    #[test]
    fn a_neighbouring_tone_in_the_same_group_is_a_new_run() {
        let mut r = ToneRuns::new(RATE, RunConfig::default());
        let mut out = Vec::new();
        r.process(&held(600.9, 1.0), &mut out);
        r.process(&held(637.5, 1.0), &mut out);
        r.process(&silence(0.2), &mut out);
        assert_eq!(out.len(), 2, "{out:?}");
        assert!((out[0].hz - 600.9).abs() < 2.0);
        assert!((out[1].hz - 637.5).abs() < 2.0);
    }
}
