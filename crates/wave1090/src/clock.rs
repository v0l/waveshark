//! The 12 MHz clock a Beast timestamp carries.
//!
//! The sample counter is the only clock a software receiver has, so the
//! timestamp is the frame's sample index scaled to twelfths of a microsecond.
//! An mlat client solves for each receiver's offset and rate, which makes a
//! constant bias harmless and a step fatal: it fits a line through the
//! timestamps of frames its peers also heard, and a clock that jumps loses
//! sync until the fit is thrown away and rebuilt.
//!
//! Two things step it. Samples the radio dropped are time that passed without
//! the counter moving, so the block sequence number rather than the count of
//! samples handed over is what the clock is made from; and a frame found in
//! the overlap between two calls to the demodulator can be reported after a
//! later one, which reads as the clock going backwards.

/// Mode S counts time in twelfths of a microsecond, and every Beast client
/// reads the timestamp that way.
pub const BEAST_CLOCK_HZ: f64 = 12_000_000.0;

/// The timestamp dump1090 gives a frame it cannot time, which a client skips
/// when it is looking for frames to synchronise a clock on
pub const UNTIMED: u64 = 0x0000_ffff_ffff_ffff;

/// Ticks from the start of the preamble to the instant a Beast reports
///
/// A Beast and a Radarcape timestamp the end of bit 56, whatever the frame's
/// length, and dump1090 follows them: `mm.timestampMsg = sampleTimestamp +
/// j*5 + (8 + 56) * 12 + bestphase` in its `demod_2400.c`. Preamble and 56
/// bits is 64 microseconds, and the demodulator here reports the start of the
/// preamble, so this is what makes the two mean the same instant.
pub const BEAST_REPORTS_AT: u64 = (8 + 56) * 12;

/// What the sample stream did between one block and the next
#[derive(Debug, PartialEq, Eq)]
pub enum Step {
    /// Contiguous with the block before it
    Continuous,
    /// Not contiguous, missing this many samples, and zero where the source
    /// started counting again rather than skipping ahead
    Broke(u64),
}

/// The Beast clock, kept in step with the sample stream behind it
pub struct Clock {
    ticks_per_sample: f64,
    /// Stream sample index the demodulator's own index zero refers to
    base: u64,
    /// Stream sample index the next block should begin at
    next_seq: u64,
    started: bool,
    /// Highest tick given out, unmasked so the 48 bit wrap is not a step
    last: u64,
}

impl Clock {
    pub fn new(rate: f64) -> Self {
        Self {
            ticks_per_sample: BEAST_CLOCK_HZ / rate,
            base: 0,
            next_seq: 0,
            started: false,
            last: 0,
        }
    }

    /// Take a block, saying whether the stream ran on without it.
    ///
    /// `seq` counts samples the source produced, dropped ones included, which
    /// is the whole point of it: a buffer the driver threw away because
    /// nothing read it in time is still 27 ms of aircraft moving.
    pub fn block(&mut self, seq: u64, len: usize) -> Step {
        let step = match self.started && seq != self.next_seq {
            true => Step::Broke(seq.saturating_sub(self.next_seq)),
            false => Step::Continuous,
        };
        if step != Step::Continuous || !self.started {
            self.base = seq;
        }
        self.started = true;
        self.next_seq = seq + len as u64;
        step
    }

    /// The timestamp for a frame the demodulator put at `at_sample` plus
    /// `at_frac` of a sample, or
    /// [`UNTIMED`] where that is not after the last frame's.
    ///
    /// Out of order rather than merely equal: the two searches each sort what
    /// they found, but a frame in the overlap between two calls can be found
    /// on the second of them and belong before the first one's last frame.
    /// Measured on radarpi, one frame in 7151 and by 8.3 us. Sending the
    /// frame with no usable time keeps it for the feeders and keeps it out of
    /// the clock fit.
    pub fn at(&mut self, at_sample: u64, at_frac: f32) -> u64 {
        // The fraction is what makes this a 12 MHz clock rather than a sample
        // counter with five written after it: at 2.4 MS/s a sample is five
        // ticks, so a whole index can only ever land on a multiple of five and
        // carries the sample grid's own jitter into an mlat fit.
        let at = (self.base + at_sample) as f64 + at_frac as f64;
        let ticks = (at * self.ticks_per_sample).max(0.0) as u64 + BEAST_REPORTS_AT;
        if ticks <= self.last {
            return UNTIMED;
        }
        self.last = ticks;
        ticks & UNTIMED
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// At 2.4 MS/s a sample is five ticks exactly, so the arithmetic is
    /// checkable by eye.
    #[test]
    fn a_sample_is_five_ticks_at_the_rate_the_decoder_is_tuned_for() {
        let mut c = Clock::new(2.4e6);
        assert_eq!(c.block(0, 65_536), Step::Continuous);
        assert_eq!(c.at(100, 0.0), 500 + BEAST_REPORTS_AT);
        assert_eq!(c.at(65_000, 0.0), 325_000 + BEAST_REPORTS_AT);
    }

    /// The clock counts ticks, not samples.
    ///
    /// A sample is five ticks at 2.4 MS/s, so a timestamp built from a whole
    /// sample index can only ever be a multiple of five and is 416 ns coarse
    /// where the format offers 83 ns. An mlat client fits a line through these
    /// and calls what does not sit on it an outlier.
    #[test]
    fn a_fraction_of_a_sample_is_a_tick_and_not_nothing() {
        let mut c = Clock::new(2.4e6);
        c.block(0, 65_536);
        assert_eq!(c.at(100, 0.2), 501 + BEAST_REPORTS_AT, "a fifth of a sample is one tick");
        assert_eq!(c.at(100, 0.6), 503 + BEAST_REPORTS_AT);
        // Backwards inside one sample is still backwards.
        assert_eq!(c.at(100, 0.4), UNTIMED);
        // A refined peak can put a frame before the sample it was indexed at,
        // and landing on a tick already given out is as untimeable as landing
        // behind one.
        assert_eq!(c.at(101, -0.4), UNTIMED);
        assert_eq!(c.at(101, 0.2), 506 + BEAST_REPORTS_AT);
    }

    /// The fault this is all about: a dropped buffer is time that passed, and
    /// a clock that does not count it steps back by the whole buffer.
    #[test]
    fn samples_the_radio_dropped_still_pass_on_the_clock() {
        let mut c = Clock::new(2.4e6);
        c.block(0, 65_536);
        assert_eq!(c.at(1_000, 0.0), 5_000 + BEAST_REPORTS_AT);
        // The next block begins 65_536 samples late: the driver threw one
        // away. The demodulator is reset and counts from zero again, so the
        // frame at its sample 1000 is 131_072 + 1000 into the stream.
        assert_eq!(c.block(131_072, 65_536), Step::Broke(65_536));
        assert_eq!(c.at(1_000, 0.0), (131_072 + 1_000) * 5 + BEAST_REPORTS_AT);
    }

    #[test]
    fn a_contiguous_block_leaves_the_demodulator_counting() {
        let mut c = Clock::new(2.4e6);
        c.block(0, 65_536);
        assert_eq!(c.block(65_536, 65_536), Step::Continuous);
        // The demodulator's index is still the stream's, so no rebasing.
        assert_eq!(c.at(70_000, 0.0), 350_000 + BEAST_REPORTS_AT);
    }

    #[test]
    fn a_frame_out_of_order_goes_out_with_no_time_rather_than_a_wrong_one() {
        let mut c = Clock::new(2.4e6);
        c.block(0, 65_536);
        assert_eq!(c.at(20_000, 0.0), 100_000 + BEAST_REPORTS_AT);
        assert_eq!(c.at(19_980, 0.0), UNTIMED, "a frame before the last one");
        assert_eq!(c.at(20_000, 0.0), UNTIMED, "and the same frame twice");
        assert_eq!(c.at(20_001, 0.0), 100_005 + BEAST_REPORTS_AT, "the next one is timed again");
    }

    /// A remote source reconnecting counts from where its server is, which
    /// can be behind where this one had got to.
    #[test]
    fn a_source_that_starts_counting_again_rebases_rather_than_going_backwards() {
        let mut c = Clock::new(2.4e6);
        c.block(1_000_000, 65_536);
        assert_eq!(c.at(10, 0.0), 5_000_050 + BEAST_REPORTS_AT);
        assert_eq!(c.block(40, 65_536), Step::Broke(0), "no count of what is missing");
        // Behind the last frame's tick, so untimed until the clock catches
        // up, which is the honest answer and not a frame lost.
        assert_eq!(c.at(10, 0.0), UNTIMED);
    }

    /// The counter on the wire is 48 bits and rolls over every 6.5 hours, so
    /// a receiver that treated the roll as a step backwards would stop being
    /// usable for mlat after its first one.
    #[test]
    fn the_six_byte_counter_rolling_over_is_not_a_step() {
        let mut c = Clock::new(2.4e6);
        let just_under = (UNTIMED / 5) - 1;
        c.block(just_under, 65_536);
        assert_eq!(c.at(0, 0.0), (just_under * 5 + BEAST_REPORTS_AT) & UNTIMED);
        assert_eq!(
            c.at(2, 0.0),
            (just_under * 5 + 10 + BEAST_REPORTS_AT) & UNTIMED,
            "the tick after the wrap, counted from zero"
        );
    }
}
