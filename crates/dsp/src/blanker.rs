//! Removing impulse noise from IQ before a channel filter spreads it.

use common::C32;

pub const DEFAULT_THRESHOLD_DB: f32 = 18.0;
pub const THRESHOLD_RANGE_DB: (f32, f32) = (10.0, 30.0);

const GUARD: usize = 12;
const LONGEST_PULSE_S: f64 = 100e-6;
const AVERAGE_S: f64 = 10e-3;

pub fn delay_at(rate: f64) -> usize {
    longest_at(rate) + 2 * GUARD + 1
}

fn longest_at(rate: f64) -> usize {
    ((rate * LONGEST_PULSE_S).ceil() as usize).max(1)
}

#[derive(Clone, Debug)]
pub struct Blanker {
    limit: f32,
    alpha: f32,
    mean: f32,
    guard: usize,
    longest: usize,
    ring: Vec<C32>,
    flagged: Vec<bool>,
    pos: usize,
    hold: usize,
    since: usize,
    run: usize,
    last_good: C32,
    into_gap: usize,
    ahead: usize,
}

impl Blanker {
    pub fn new(rate: f64, threshold_db: f32) -> Self {
        let delay = delay_at(rate);
        let mut b = Self {
            limit: 1.0,
            alpha: (1.0 / (rate * AVERAGE_S).max(1.0)) as f32,
            mean: 0.0,
            guard: GUARD,
            longest: longest_at(rate),
            ring: vec![C32::new(0.0, 0.0); delay],
            flagged: vec![false; delay],
            pos: 0,
            hold: 0,
            since: usize::MAX,
            run: 0,
            last_good: C32::new(0.0, 0.0),
            into_gap: 0,
            ahead: 0,
        };
        b.set_threshold_db(threshold_db);
        b
    }

    pub fn set_threshold_db(&mut self, db: f32) {
        let db = db.clamp(THRESHOLD_RANGE_DB.0, THRESHOLD_RANGE_DB.1);
        self.limit = 10f32.powf(db / 10.0);
    }

    pub fn delay(&self) -> usize {
        self.ring.len()
    }

    pub fn reset(&mut self) {
        self.mean = 0.0;
        self.ring.fill(C32::new(0.0, 0.0));
        self.flagged.fill(false);
        self.pos = 0;
        self.hold = 0;
        self.since = usize::MAX;
        self.run = 0;
        self.last_good = C32::new(0.0, 0.0);
        self.into_gap = 0;
        self.ahead = 0;
    }

    fn is_pulse(&mut self, p: f32) -> bool {
        if self.mean <= 0.0 {
            self.mean = p;
        }
        let loud = p > self.limit * self.mean;
        self.run = if loud { self.run + 1 } else { 0 };
        if loud && self.run <= self.longest {
            self.since = 0;
            return true;
        }
        if loud {
            self.mean = self.mean.max(p);
        }
        self.since = self.since.saturating_add(1);
        if self.since > self.guard {
            self.mean += self.alpha * (p - self.mean);
        }
        false
    }

    fn fill(&mut self, at: usize) -> C32 {
        let n = self.ring.len();
        let from = if self.into_gap > 0 { self.ahead.saturating_sub(1).max(1) } else { 1 };
        let ahead = (from..=n).find(|j| !self.flagged[(at + j) % n]);
        self.ahead = ahead.unwrap_or(n);
        match ahead {
            Some(j) => {
                let next = self.ring[(at + j) % n];
                let t = (self.into_gap + 1) as f32 / (self.into_gap + j + 1) as f32;
                self.last_good + (next - self.last_good) * t
            }
            None => self.last_good,
        }
    }

    pub fn process(&mut self, buf: &mut [C32]) -> usize {
        let n = self.ring.len();
        let mut blanked = 0;
        for s in buf.iter_mut() {
            let x = *s;
            if self.is_pulse(x.norm_sqr()) {
                for back in 1..=self.guard {
                    self.flagged[(self.pos + n - back) % n] = true;
                }
                self.hold = self.guard + 1;
            }
            let at = self.pos;
            let (oldest, gap) = (self.ring[at], self.flagged[at]);
            self.ring[at] = x;
            self.flagged[at] = self.hold > 0;
            self.hold = self.hold.saturating_sub(1);
            self.pos = if at + 1 == n { 0 } else { at + 1 };
            *s = if gap {
                blanked += 1;
                let filled = self.fill(at);
                self.into_gap += 1;
                filled
            } else {
                self.last_good = oldest;
                self.into_gap = 0;
                oldest
            };
        }
        blanked
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fir::{FirDecim, lowpass};
    use std::f64::consts::TAU;

    const RATE: f64 = 240_000.0;
    const TONE_HZ: f64 = 1_000.0;

    struct Rng(u64);

    impl Rng {
        fn uniform(&mut self) -> f64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
        }

        fn gaussian(&mut self, sigma: f64) -> C32 {
            let r = (-2.0 * self.uniform().ln()).sqrt() * sigma;
            let a = TAU * self.uniform();
            C32::new((r * a.cos()) as f32, (r * a.sin()) as f32)
        }
    }

    fn tone(n: usize, hz: f64, amp: f32) -> Vec<C32> {
        (0..n)
            .map(|k| {
                let ph = TAU * hz * k as f64 / RATE;
                C32::new(ph.cos() as f32, ph.sin() as f32) * amp
            })
            .collect()
    }

    fn add_noise(x: &mut [C32], sigma: f64, seed: u64) {
        let mut r = Rng(seed);
        for s in x {
            *s += r.gaussian(sigma / std::f64::consts::SQRT_2);
        }
    }

    fn add_impulses(x: &mut [C32], per_s: f64, amp: f32) -> usize {
        let every = (RATE / per_s) as usize;
        let mut spikes = vec![C32::new(0.0, 0.0); x.len()];
        let mut n = 0;
        for k in (every / 2..x.len()).step_by(every) {
            let turn = (k as f32 * 0.37).sin();
            spikes[k] = C32::new(turn, (1.0 - turn * turn).sqrt()) * amp;
            n += 1;
        }
        let mut shaped = Vec::new();
        crate::fir::Fir::new(lowpass(31, 0.4, 60.0)).process(&spikes, &mut shaped);
        for (s, p) in x.iter_mut().zip(shaped) {
            *s += p;
        }
        n
    }

    fn channel_snr_db(x: &[C32]) -> f64 {
        let mixed: Vec<C32> = x
            .iter()
            .enumerate()
            .map(|(k, s)| {
                let ph = -TAU * TONE_HZ * k as f64 / RATE;
                *s * C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();
        let mut y = Vec::new();
        FirDecim::design_hz(RATE, 40, 1_500.0, 70.0).process(&mixed, &mut y);
        let y = &y[200..];
        let mean = y.iter().fold(C32::new(0.0, 0.0), |a, s| a + *s) / y.len() as f32;
        let noise = y.iter().map(|s| (*s - mean).norm_sqr() as f64).sum::<f64>() / y.len() as f64;
        10.0 * (mean.norm_sqr() as f64 / noise).log10()
    }

    fn blank(x: &[C32], db: f32) -> (Vec<C32>, usize) {
        let mut b = Blanker::new(RATE, db);
        let mut y = x.to_vec();
        let mut blanked = 0;
        for chunk in y.chunks_mut(4_096) {
            blanked += b.process(chunk);
        }
        (y, blanked)
    }

    fn weak_tone_with_ignition() -> (Vec<C32>, usize) {
        let mut x = tone(2 * RATE as usize, TONE_HZ, 0.01);
        add_noise(&mut x, 0.0028, 7);
        let pulses = add_impulses(&mut x, 100.0, 2.0);
        (x, pulses)
    }

    #[test]
    fn ignition_at_100_hz_takes_a_weak_tone_from_27_db_to_5_and_blanking_gives_back_26() {
        let mut clean = tone(2 * RATE as usize, TONE_HZ, 0.01);
        add_noise(&mut clean, 0.0028, 7);
        let (dirty, pulses) = weak_tone_with_ignition();
        let (fixed, blanked) = blank(&dirty, DEFAULT_THRESHOLD_DB);
        let (clean_db, dirty_db, fixed_db) =
            (channel_snr_db(&clean), channel_snr_db(&dirty), channel_snr_db(&fixed));
        assert_eq!(pulses, 200);
        assert!((27.0..28.0).contains(&clean_db), "clean channel {clean_db:.1} dB, measured 27.5");
        assert!((4.0..5.5).contains(&dirty_db), "pulsed channel {dirty_db:.1} dB, measured 4.6");
        assert!(
            (26.5..clean_db).contains(&fixed_db),
            "blanked channel {fixed_db:.1} dB, floor 26.5 and ceiling the clean channel"
        );
        assert_eq!(blanked, 6_265, "25 a pulse and the ringing that retriggers");
    }

    #[test]
    fn zeroing_the_gap_instead_of_bridging_it_would_leave_the_channel_at_20_db() {
        let (dirty, _) = weak_tone_with_ignition();
        let mut b = Blanker::new(RATE, DEFAULT_THRESHOLD_DB);
        let mut bridged = dirty.clone();
        b.process(&mut bridged);
        let d = b.delay();
        let mut zeroed = bridged.clone();
        for k in d..dirty.len() {
            if bridged[k] != dirty[k - d] {
                zeroed[k] = C32::new(0.0, 0.0);
            }
        }
        let (zero_db, bridge_db) = (channel_snr_db(&zeroed), channel_snr_db(&bridged));
        assert!((19.0..20.5).contains(&zero_db), "zeroed {zero_db:.1} dB, measured 19.8");
        assert!(bridge_db > zero_db + 6.0, "bridged {bridge_db:.1} dB against zeroed {zero_db:.1}");
    }

    #[test]
    fn twelve_db_is_the_lowest_threshold_that_leaves_ten_seconds_of_noise_alone() {
        let mut x = vec![C32::new(0.0, 0.0); 10 * RATE as usize];
        add_noise(&mut x, 0.01, 3);
        assert_eq!(blank(&x, 12.0).1, 0);
        assert_eq!(blank(&x, DEFAULT_THRESHOLD_DB).1, 0);
        assert_eq!(blank(&x, THRESHOLD_RANGE_DB.0).1, 2_500, "10 dB blanks a tenth of a percent");
    }

    #[test]
    fn the_top_of_the_range_still_catches_pulses_40_db_over_the_floor() {
        let (dirty, _) = weak_tone_with_ignition();
        let (fixed, blanked) = blank(&dirty, THRESHOLD_RANGE_DB.1);
        assert_eq!(blanked, 5_004);
        let db = channel_snr_db(&fixed);
        assert!((24.5..26.0).contains(&db), "{db:.1} dB at a 30 dB threshold, measured 25.2");
    }

    #[test]
    fn a_strong_tone_am_and_fm_pass_unchanged_behind_the_delay() {
        let n = RATE as usize;
        let carrier = tone(n, 20_000.0, 0.9);
        let am: Vec<C32> = carrier
            .iter()
            .enumerate()
            .map(|(k, s)| *s * (0.5 + 0.5 * (TAU * 1_000.0 * k as f64 / RATE).cos() as f32))
            .collect();
        let fm: Vec<C32> = (0..n)
            .scan(0.0f64, |ph, k| {
                *ph += TAU * 5_000.0 * (TAU * 1_000.0 * k as f64 / RATE).sin() / RATE;
                Some(C32::new(ph.cos() as f32, ph.sin() as f32) * 0.9)
            })
            .collect();
        let d = Blanker::new(RATE, DEFAULT_THRESHOLD_DB).delay();
        assert_eq!(d, 49);
        for (name, x) in [("carrier", carrier), ("am", am), ("fm", fm)] {
            let mut x = x;
            add_noise(&mut x, 0.001, 11);
            let (y, blanked) = blank(&x, DEFAULT_THRESHOLD_DB);
            assert_eq!(blanked, 0, "{name} was blanked");
            assert_eq!(&y[d..], &x[..n - d], "{name} changed on the way through");
        }
    }

    #[test]
    fn a_carrier_keyed_40_db_over_the_floor_loses_only_its_first_100_us() {
        let n = RATE as usize / 2;
        let mut x = vec![C32::new(0.0, 0.0); n];
        add_noise(&mut x, 0.001, 5);
        let on = n / 2;
        for (k, s) in tone(n - on, 3_000.0, 0.1).into_iter().enumerate() {
            x[on + k] += s;
        }
        let (y, blanked) = blank(&x, DEFAULT_THRESHOLD_DB);
        let b = Blanker::new(RATE, DEFAULT_THRESHOLD_DB);
        assert_eq!(b.longest, 24);
        assert_eq!(blanked, b.longest + 2 * b.guard, "blanked {blanked} of the carrier");
        let (d, after) = (b.delay(), on + b.longest + b.guard);
        assert_eq!(&y[after + d..], &x[after..n - d]);
    }
}
