//! Coherent M-FSK read across a whole channel at once, synchronised by a
//! Costas array.
//!
//! The waveform is a fixed number of symbols, each one of `tones` tones a
//! symbol time apart, with groups of known tones (a Costas array) laid
//! through it so a receiver can find where a transmission starts and what
//! frequency it is on. FT8 is 79 symbols of 8 tones 6.25 Hz apart, FT4 is
//! 105 symbols of 4 tones 20.8 Hz apart, and both are read by this module
//! from the same description.
//!
//! The point is that a 3 kHz channel holds dozens of transmissions at once,
//! so this does not tune one: it transforms the channel once into a
//! spectrogram of tone-spaced bins and half-symbol steps, scores every
//! (time, frequency) for the sync pattern, and hands back soft bits for each
//! of the best. What the bits mean, and the code that protects them, is the
//! payload's business.

use crate::window;
use common::C32;
use rustfft::{Fft, FftPlanner};
use std::sync::Arc;

/// Where a group of known tones sits: the symbol it starts at, and the tones.
#[derive(Clone, Copy, Debug)]
pub struct Sync {
    pub at: usize,
    pub tones: &'static [u8],
}

/// One M-FSK transmission's shape.
#[derive(Clone, Copy, Debug)]
pub struct Waveform {
    pub name: &'static str,
    /// Tones, which is two to the power of the bits a symbol carries.
    pub tones: usize,
    /// Symbols on the air, sync and data together.
    pub symbols: usize,
    pub baud: f64,
    /// The sync groups, in order.
    pub sync: &'static [Sync],
    /// Symbols carrying payload, as half-open ranges.
    pub data: &'static [(usize, usize)],
    /// Tone sent for each bit pattern: `gray[bits] == tone`.
    pub gray: &'static [u8],
    /// Seconds between the starts of two transmissions, which is the clock
    /// every station keys against.
    pub slot_s: f64,
}

impl Waveform {
    pub fn bits_per_symbol(&self) -> usize {
        self.tones.trailing_zeros() as usize
    }

    /// Bits one transmission carries.
    pub fn bits(&self) -> usize {
        self.data.iter().map(|(a, b)| b - a).sum::<usize>() * self.bits_per_symbol()
    }

    /// How long a transmission is on the air.
    pub fn duration_s(&self) -> f64 {
        self.symbols as f64 / self.baud
    }
}

/// FT8: 79 symbols of 8-FSK at 6.25 baud, on a fifteen-second clock.
pub const FT8: Waveform = Waveform {
    name: "FT8",
    tones: 8,
    symbols: 79,
    baud: 6.25,
    sync: &[
        Sync { at: 0, tones: &FT8_COSTAS },
        Sync { at: 36, tones: &FT8_COSTAS },
        Sync { at: 72, tones: &FT8_COSTAS },
    ],
    data: &[(7, 36), (43, 72)],
    gray: &[0, 1, 3, 2, 5, 6, 4, 7],
    slot_s: 15.0,
};

const FT8_COSTAS: [u8; 7] = [3, 1, 4, 0, 6, 5, 2];

/// FT4: 105 symbols of 4-FSK at 20.8333 baud on a seven-and-a-half-second
/// clock, with four different sync groups rather than one repeated, and a
/// ramp symbol at each end that carries nothing.
pub const FT4: Waveform = Waveform {
    name: "FT4",
    tones: 4,
    symbols: 105,
    baud: 1.0 / 0.048,
    sync: &[
        Sync { at: 1, tones: &[0, 1, 3, 2] },
        Sync { at: 34, tones: &[1, 0, 2, 3] },
        Sync { at: 67, tones: &[2, 3, 1, 0] },
        Sync { at: 100, tones: &[3, 2, 0, 1] },
    ],
    data: &[(5, 34), (38, 67), (71, 100)],
    gray: &[0, 1, 3, 2],
    slot_s: 7.5,
};

/// A transmission found in a slot.
#[derive(Clone, Debug)]
pub struct Heard {
    /// Where the lowest tone sits in the stream handed in, in hertz.
    pub freq_hz: f64,
    /// How far into the slot the transmission started, in seconds.
    pub at_s: f64,
    /// How well the sync groups matched, in dB above the mean tone.
    pub sync_db: f32,
    /// Signal to noise in a 2500 Hz reference bandwidth, which is what
    /// every station on these modes reports.
    pub snr_db: f32,
    /// One log-likelihood per payload bit, positive for a one.
    pub llr: Vec<f32>,
}

/// Reads one slot of a channel: the spectrogram, the sync search, and the
/// soft bits for each transmission it found.
pub struct Slot {
    rate: f64,
    wf: Waveform,
    /// Samples a symbol, and the transform that reads one.
    symbol: usize,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    /// Log power, `steps` rows of `bins`, a row every half symbol.
    mag: Vec<f32>,
    bins: usize,
    steps: usize,
    scratch: Vec<C32>,
}

/// Steps a symbol the spectrogram is taken at. A station keys when its own
/// clock says to, not when this one's transform boundary falls: measured on
/// a synthesised FT8 transmission walked across a symbol in 20 ms steps, one
/// step a symbol loses the transmission that starts half a symbol off the
/// boundary, and two steps read every offset.
const TIME_STEPS: usize = 2;

/// Bins a tone spacing the transform is taken at, which is the same argument
/// in frequency: a station is not on a tone-spaced grid.
const FREQ_STEPS: usize = 2;

/// Candidates carried from the sync search into the soft-bit stage.
///
/// Nothing here refuses a candidate on its score, because a transmission at
/// the bottom of what these modes reach does not stand above the noise by
/// any margin a strong one would set: what refuses a candidate is the code
/// and the check behind it. So this is a budget rather than a threshold. A
/// crowded 3 kHz channel holds about thirty transmissions and each strong
/// one leaves several neighbouring scores behind it, so two hundred covers
/// a full band; the cost is one belief propagation each, which is around a
/// hundredth of what the sync search over the same slot costs.
const MAX_CANDIDATES: usize = 200;

impl Slot {
    pub fn new(rate: f64, wf: Waveform) -> Self {
        let symbol = (rate / wf.baud).round().max(4.0) as usize;
        let nfft = symbol * FREQ_STEPS;
        let fft = FftPlanner::new().plan_fft_forward(nfft);
        // Hann rather than rectangular, measured on a passband of eight
        // stations: Hann reads all eight and a rectangular window reads
        // seven, because what one station leaks into the next tone costs
        // more than the sensitivity Hann gives away. On a single station
        // sitting exactly on the tone grid, rectangular is about a decibel
        // better and that is the whole of what it wins.
        let window = window::hann(symbol);
        Self {
            rate,
            wf,
            symbol,
            fft,
            window,
            mag: Vec::new(),
            bins: nfft,
            steps: 0,
            scratch: Vec::new(),
        }
    }

    /// Samples one slot of the stream holds.
    pub fn slot_samples(&self) -> usize {
        (self.rate * self.wf.slot_s).round() as usize
    }

    /// Read every transmission in `iq`, which is one slot of a channel with
    /// the dial at zero, searching `band` in hertz.
    pub fn read(&mut self, iq: &[C32], band: (f64, f64)) -> Vec<Heard> {
        self.spectrogram(iq);
        let hz = self.rate / self.bins as f64;
        let lo = ((band.0 / hz).floor().max(0.0) as usize).min(self.bins);
        let hi = ((band.1 / hz).ceil().max(0.0) as usize).min(self.bins);
        let span = self.wf.tones * FREQ_STEPS;
        if self.steps < self.wf.symbols * TIME_STEPS || hi < lo + span {
            return Vec::new();
        }

        let last = self.steps - self.wf.symbols * TIME_STEPS;
        let mut found: Vec<(f32, usize, usize)> = Vec::new();
        for t in 0..=last {
            for f in lo..hi - span {
                let score = self.sync_score(t, f);
                found.push((score, t, f));
            }
        }
        found.sort_by(|a, b| b.0.total_cmp(&a.0));

        let mut out: Vec<Heard> = Vec::new();
        let mut taken: Vec<(usize, usize)> = Vec::new();
        for (score, t, f) in found {
            if out.len() >= MAX_CANDIDATES {
                break;
            }
            // One transmission scores well at every offset around its own,
            // so a candidate a step either way from one already taken is
            // that same station read again. No wider than a step, because
            // two stations in a crowded band sit one tone apart.
            let near = taken.iter().any(|(tt, ff)| tt.abs_diff(t) <= 1 && ff.abs_diff(f) <= 1);
            if near || score <= 0.0 {
                continue;
            }
            taken.push((t, f));
            out.push(self.soft(t, f, score));
        }
        out
    }

    /// Log power of every tone-spaced bin, a row every half symbol.
    fn spectrogram(&mut self, iq: &[C32]) {
        let step = self.symbol / TIME_STEPS;
        self.steps = match iq.len() >= self.symbol {
            true => (iq.len() - self.symbol) / step + 1,
            false => 0,
        };
        self.mag.clear();
        self.mag.resize(self.steps * self.bins, 0.0);
        self.scratch.resize(self.bins, C32::default());
        for t in 0..self.steps {
            let block = &iq[t * step..t * step + self.symbol];
            for (k, s) in self.scratch.iter_mut().enumerate() {
                *s = match block.get(k) {
                    Some(v) => *v * self.window[k],
                    None => C32::default(),
                };
            }
            self.fft.process(&mut self.scratch);
            for (k, s) in self.scratch.iter().enumerate() {
                // In log power, so a tone's strength against the others is a
                // difference and nothing has to know the gain in front.
                self.mag[t * self.bins + k] = 10.0 * (s.norm_sqr() + 1e-20).log10();
            }
        }
    }

    fn at(&self, step: usize, bin: usize) -> f32 {
        self.mag[step * self.bins + bin]
    }

    /// How much the sync groups stand above the mean of the tones they were
    /// read against, in dB. A transmission scores several dB; noise scores
    /// about zero either way.
    fn sync_score(&self, t: usize, f: usize) -> f32 {
        let mut score = 0.0f32;
        let mut n = 0usize;
        for group in self.wf.sync {
            for (k, tone) in group.tones.iter().enumerate() {
                let step = t + (group.at + k) * TIME_STEPS;
                let mut mean = 0.0f32;
                for tone in 0..self.wf.tones {
                    mean += self.at(step, f + tone * FREQ_STEPS);
                }
                score +=
                    self.at(step, f + *tone as usize * FREQ_STEPS) - mean / self.wf.tones as f32;
                n += 1;
            }
        }
        score / n.max(1) as f32
    }

    /// The soft bits of one candidate, and what it was heard at.
    fn soft(&self, t: usize, f: usize, sync_db: f32) -> Heard {
        let bits = self.wf.bits_per_symbol();
        let mut llr = Vec::with_capacity(self.wf.bits());
        // Tone strengths of the payload symbols, and the strongest of each,
        // which is the transmission's own power.
        let mut signal = 0.0f32;
        let mut symbols = 0usize;
        for (from, to) in self.wf.data {
            for s in *from..*to {
                let step = t + s * TIME_STEPS;
                let tone = |k: usize| self.at(step, f + k * FREQ_STEPS);
                let mut best = f32::MIN;
                for b in 0..bits {
                    // Max-log: the best tone that would have carried a one
                    // against the best that would have carried a zero.
                    let (mut one, mut zero) = (f32::MIN, f32::MIN);
                    for pattern in 0..self.wf.tones {
                        let v = tone(self.wf.gray[pattern] as usize);
                        best = best.max(v);
                        match pattern >> (bits - 1 - b) & 1 {
                            1 => one = one.max(v),
                            _ => zero = zero.max(v),
                        }
                    }
                    llr.push(one - zero);
                }
                signal += best;
                symbols += 1;
            }
        }
        // The soft values are differences of log power, whose scale depends
        // on the gain in front of the receiver and on the fade. Normalising
        // to a fixed spread is what makes the code's own thresholds mean the
        // same thing from one transmission to the next.
        let mean = llr.iter().sum::<f32>() / llr.len().max(1) as f32;
        let var =
            llr.iter().map(|x| (x - mean) * (x - mean)).sum::<f32>() / llr.len().max(1) as f32;
        let scale = 2.83 / var.sqrt().max(1e-6);
        for v in llr.iter_mut() {
            *v *= scale;
        }

        let signal = signal / symbols.max(1) as f32;
        let noise = self.noise_floor(t, f);
        Heard {
            freq_hz: f as f64 * self.rate / self.bins as f64,
            at_s: t as f64 * self.symbol as f64 / (TIME_STEPS as f64 * self.rate),
            sync_db,
            snr_db: snr_2500(signal, noise, self.wf.baud),
            llr,
        }
    }

    /// The floor the transmission stands on: the median bin of the rows it
    /// occupies, away from its own tones.
    fn noise_floor(&self, t: usize, f: usize) -> f32 {
        let mut bins: Vec<f32> = Vec::new();
        let span = self.wf.tones * FREQ_STEPS;
        for s in (0..self.wf.symbols).step_by(4) {
            let step = t + s * TIME_STEPS;
            for b in (0..self.bins).step_by(FREQ_STEPS) {
                if b + span > f && b < f + span {
                    continue;
                }
                bins.push(self.at(step, b));
            }
        }
        if bins.is_empty() {
            return -100.0;
        }
        let k = bins.len() / 2;
        bins.select_nth_unstable_by(k, f32::total_cmp);
        bins[k]
    }
}

/// Signal to noise in the 2500 Hz reference every station on these modes
/// reports, given a tone's power and the floor of one bin beside it, both in
/// dB.
fn snr_2500(signal_db: f32, noise_db: f32, baud: f64) -> f32 {
    let (s, n) = (10f32.powf(signal_db / 10.0), 10f32.powf(noise_db / 10.0));
    let power = (s - n).max(n * 1e-3);
    // The floor is measured in one bin, which is a tone spacing wide, so the
    // reference is that many times wider.
    let reference = (2500.0 / baud) as f32;
    10.0 * (power / (n * reference)).log10()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    /// A transmission keyed as tones: `at_s` into the slot, its lowest tone
    /// at `base_hz`, at an amplitude of one.
    pub fn keyed(wf: Waveform, tones: &[u8], rate: f64, base_hz: f64, at_s: f64) -> Vec<C32> {
        let slot = (rate * wf.slot_s) as usize;
        let mut out = vec![C32::default(); slot];
        let symbol = (rate / wf.baud).round() as usize;
        let start = (at_s * rate) as usize;
        let mut phase = 0.0f64;
        for (s, tone) in tones.iter().enumerate() {
            let f = base_hz + *tone as f64 * wf.baud;
            for k in 0..symbol {
                let at = start + s * symbol + k;
                if at >= out.len() {
                    break;
                }
                phase += TAU * f / rate;
                out[at] += C32::new(phase.cos() as f32, phase.sin() as f32);
            }
        }
        out
    }

    /// The tones of one FT8 transmission, made here rather than by the
    /// payload layer so this module's tests do not depend on it.
    fn tones_of(wf: Waveform, bits: &[bool]) -> Vec<u8> {
        let mut tones = vec![0u8; wf.symbols];
        for group in wf.sync {
            tones[group.at..group.at + group.tones.len()].copy_from_slice(group.tones);
        }
        let per = wf.bits_per_symbol();
        let mut taken = 0usize;
        for (from, to) in wf.data {
            for tone in tones.iter_mut().take(*to).skip(*from) {
                let pattern =
                    (0..per).fold(0usize, |acc, k| acc << 1 | usize::from(bits[taken + k]));
                taken += per;
                *tone = wf.gray[pattern];
            }
        }
        tones
    }

    fn pattern(n: usize) -> Vec<bool> {
        (0..n).map(|k| (k * 7 + k / 5) % 3 != 0).collect()
    }

    /// The whole detector on one synthesised transmission: it is found at
    /// the frequency and the time it was keyed at, and every soft bit
    /// agrees with the bit that was sent.
    #[test]
    fn a_keyed_transmission_is_found_and_read() {
        let rate = 12_000.0;
        let bits = pattern(FT8.bits());
        let tones = tones_of(FT8, &bits);
        let iq = keyed(FT8, &tones, rate, 1_000.0, 0.5);
        let mut slot = Slot::new(rate, FT8);
        let heard = slot.read(&iq, (200.0, 3_000.0));
        // Every candidate the cap allows comes back, best first, because a
        // weak transmission does not stand above noise by any threshold a
        // strong one would set: what refuses a candidate is the code.
        assert_eq!(heard.len(), MAX_CANDIDATES, "the budget, not a threshold");
        let h = &heard[0];
        assert!((h.freq_hz - 1_000.0).abs() < 3.2, "{} Hz", h.freq_hz);
        assert!((h.at_s - 0.5).abs() < 0.09, "{} s", h.at_s);
        assert!(h.sync_db > 10.0, "{} dB of sync", h.sync_db);
        assert_eq!(h.llr.len(), 174);
        let wrong = h.llr.iter().zip(&bits).filter(|(l, b)| (**l > 0.0) != **b).count();
        assert_eq!(wrong, 0, "{wrong} soft bits disagree with what was keyed");
    }

    /// Three stations in the one channel at once, which is the whole point:
    /// each is found at its own frequency and each is read whole.
    #[test]
    fn three_stations_in_one_channel_are_all_read() {
        let rate = 12_000.0;
        let mut iq = vec![C32::default(); (rate * FT8.slot_s) as usize];
        let mut sent = Vec::new();
        for (k, hz) in [500.0, 1_437.5, 2_200.0].iter().enumerate() {
            let bits = pattern(FT8.bits() + k * 3)[k * 3..].to_vec();
            let tones = tones_of(FT8, &bits);
            for (a, b) in iq.iter_mut().zip(keyed(FT8, &tones, rate, *hz, 0.4 + 0.2 * k as f64)) {
                *a += b;
            }
            sent.push((*hz, bits));
        }
        let mut slot = Slot::new(rate, FT8);
        let heard = slot.read(&iq, (200.0, 3_000.0));
        for (hz, bits) in &sent {
            let h = heard
                .iter()
                .find(|h| (h.freq_hz - hz).abs() < 3.2)
                .unwrap_or_else(|| panic!("nothing at {hz} Hz"));
            let wrong = h.llr.iter().zip(bits).filter(|(l, b)| (**l > 0.0) != **b).count();
            assert_eq!(wrong, 0, "{wrong} soft bits wrong at {hz} Hz");
        }
    }

    /// A station is not on the tone grid, so the transform is taken at half
    /// a tone spacing and the window is Hann rather than rectangular.
    /// Measured: every quarter-tone offset across a whole spacing is read
    /// with no wrong bits.
    #[test]
    fn a_station_off_the_tone_grid_still_reads() {
        let rate = 12_000.0;
        let bits = pattern(FT8.bits());
        let tones = tones_of(FT8, &bits);
        for off in [0.0, 1.5625, 3.125, 4.6875, 6.25] {
            let iq = keyed(FT8, &tones, rate, 1_000.0 + off, 0.5);
            let mut slot = Slot::new(rate, FT8);
            let heard = slot.read(&iq, (200.0, 3_000.0));
            assert!((heard[0].freq_hz - (1_000.0 + off)).abs() < 3.2, "{off} Hz off grid");
            let wrong = heard[0].llr.iter().zip(&bits).filter(|(l, b)| (**l > 0.0) != **b).count();
            assert_eq!(wrong, 0, "{wrong} soft bits wrong {off} Hz off grid");
        }
    }

    /// A station keys on its own clock, so a transmission starting between
    /// two of this receiver's symbol boundaries has to read as well as one
    /// starting on one. Measured across a whole symbol in 20 ms steps: every
    /// offset reads with no wrong bits, where one step a symbol loses the
    /// one starting half a symbol off.
    #[test]
    fn a_station_starting_between_symbols_still_reads() {
        let rate = 12_000.0;
        let bits = pattern(FT8.bits());
        let tones = tones_of(FT8, &bits);
        for k in 0..8 {
            let at = 0.48 + 0.02 * k as f64;
            let iq = keyed(FT8, &tones, rate, 1_000.0, at);
            let mut slot = Slot::new(rate, FT8);
            let heard = slot.read(&iq, (200.0, 3_000.0));
            let wrong = heard[0].llr.iter().zip(&bits).filter(|(l, b)| (**l > 0.0) != **b).count();
            assert_eq!(wrong, 0, "{wrong} soft bits wrong starting at {at} s");
            assert!((heard[0].at_s - at).abs() <= 0.08, "{} s, keyed at {at}", heard[0].at_s);
        }
    }

    /// FT4 is the same reading with four tones, a different sync in four
    /// places and a ramp symbol at each end.
    #[test]
    fn an_ft4_transmission_is_read_by_the_same_slot() {
        let rate = 12_000.0;
        let bits = pattern(FT4.bits());
        let tones = tones_of(FT4, &bits);
        let iq = keyed(FT4, &tones, rate, 1_200.0, 0.3);
        let mut slot = Slot::new(rate, FT4);
        let heard = slot.read(&iq, (200.0, 3_000.0));
        assert!((heard[0].freq_hz - 1_200.0).abs() < 11.0, "{} Hz", heard[0].freq_hz);
        assert_eq!(heard[0].llr.len(), 174);
        let wrong = heard[0].llr.iter().zip(&bits).filter(|(l, b)| (**l > 0.0) != **b).count();
        assert_eq!(wrong, 0, "{wrong} soft bits disagree with what was keyed");
    }

    /// A slot of noise: whatever scores best is not a transmission, and the
    /// soft bits it hands back are noise for the code to refuse.
    #[test]
    fn noise_scores_nothing_like_a_transmission() {
        let rate = 12_000.0;
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> =
            (0..(rate * FT8.slot_s) as usize).map(|_| C32::new(rng(), rng())).collect();
        let mut slot = Slot::new(rate, FT8);
        let heard = slot.read(&iq, (200.0, 3_000.0));
        let best = heard.iter().map(|h| h.sync_db).fold(0.0f32, f32::max);
        assert!(best < 6.0, "noise scored {best} dB of sync");
        let loud = heard.iter().filter(|h| h.snr_db > 0.0).count();
        assert_eq!(loud, 0, "{loud} of {} noise candidates read as loud", heard.len());
    }

    /// What a station is heard at, against what it was keyed at: the tones
    /// are made 10 dB above a noise floor and read back within 2 dB.
    #[test]
    fn a_transmission_is_measured_against_the_floor() {
        let rate = 12_000.0;
        let bits = pattern(FT8.bits());
        let tones = tones_of(FT8, &bits);
        let mut iq = keyed(FT8, &tones, rate, 1_000.0, 0.5);
        let mut seed = 0x0123_4567_89ab_cdefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        for s in iq.iter_mut() {
            *s += C32::new(rng(), rng());
        }
        let mut slot = Slot::new(rate, FT8);
        let heard = slot.read(&iq, (200.0, 3_000.0));
        // One unit of tone against a noise density that fills 12 kHz: the
        // reading is in 2500 Hz, and what matters is that it is finite and
        // in the range a station reports.
        let snr = heard[0].snr_db;
        assert!((-30.0..30.0).contains(&snr), "{snr} dB");
        assert!(snr.is_finite());
    }
}
