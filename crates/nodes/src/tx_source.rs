//! What every packet transmitter shares: when the next one goes out.
//!
//! A protocol source node builds one transmission and hands it to a
//! modulator. What it must not do is build one per block: a block of clock at
//! 2.4 MS/s is a few milliseconds and a POCSAG page is seconds of air, so a
//! source keying once a block hands the radio a hundred times more signal
//! than there is time to send it in, and the queue in front of the converter
//! grows until the transmission an operator hears is minutes old.
//!
//! So the pace is kept in air time. Each block of clock is worth its own
//! duration, and the next transmission goes out only once the clock has
//! caught up with what was handed over last time plus the gap between
//! repeats.

use std::time::Duration;

/// The gap left between repeats of a transmission.
pub const DEFAULT_PAUSE_MS: f64 = 1_000.0;

/// A budget of air time, in microseconds.
#[derive(Clone, Debug)]
pub struct Pace {
    /// Clock time seen since the last reset.
    elapsed_us: f64,
    /// Air time already handed to the modulator, plus the pauses between.
    sent_us: f64,
    pause: Duration,
    /// Transmissions handed over since the last reset, which is what says a
    /// stage is sending rather than merely built.
    sent: u64,
}

impl Default for Pace {
    fn default() -> Self {
        Self {
            elapsed_us: 0.0,
            sent_us: 0.0,
            pause: Duration::from_millis(DEFAULT_PAUSE_MS as u64),
            sent: 0,
        }
    }
}

impl Pace {
    /// Count a block of `samples` at `rate` as that much air time.
    pub fn clock(&mut self, samples: usize, rate: f64) {
        if rate > 0.0 {
            self.elapsed_us += samples as f64 * 1e6 / rate;
        }
    }

    /// Whether the clock has caught up with what was sent.
    pub fn due(&self) -> bool {
        self.elapsed_us >= self.sent_us
    }

    /// Record a transmission of `us` microseconds going out.
    pub fn spent(&mut self, us: f64) {
        self.sent_us += us + self.pause.as_micros() as f64;
        self.sent += 1;
    }

    pub fn sent(&self) -> u64 {
        self.sent
    }

    pub fn pause_ms(&self) -> f64 {
        self.pause.as_millis() as f64
    }

    pub fn set_pause_ms(&mut self, ms: f64) {
        self.pause = Duration::from_millis(ms.clamp(0.0, 60_000.0) as u64);
    }

    pub fn reset(&mut self) {
        self.elapsed_us = 0.0;
        self.sent_us = 0.0;
        self.sent = 0;
    }
}

/// How long a run of pulse timings takes on the air, in microseconds.
pub fn air_time_us(pulses: &[common::pulse::Pulse]) -> f64 {
    pulses.iter().map(|p| f64::from(p.mark) + f64::from(p.gap)).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::pulse::Pulse;

    /// Half a second of transmission with a second between repeats is one
    /// transmission every one and a half seconds, whatever the block size.
    #[test]
    fn a_transmission_goes_out_at_the_rate_the_clock_allows() {
        let mut p = Pace::default();
        p.set_pause_ms(1_000.0);
        let rate = 48_000.0;
        let block = 480; // 10 ms
        let mut sent = 0;
        for _ in 0..500 {
            // 5 s of clock
            p.clock(block, rate);
            if p.due() {
                p.spent(500_000.0);
                sent += 1;
            }
        }
        assert_eq!(sent, 4, "5 s of clock carries four 1.5 s slots and starts a fifth");
        assert_eq!(p.sent(), 4);
    }

    #[test]
    fn air_time_is_the_sum_of_every_mark_and_gap() {
        let pulses = [Pulse { mark: 100, gap: 50 }, Pulse { mark: 200, gap: 0 }];
        assert_eq!(air_time_us(&pulses), 350.0);
    }
}
