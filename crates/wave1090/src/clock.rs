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

    /// The timestamp for a frame the demodulator put at `at_sample`, or
    /// [`UNTIMED`] where that is not after the last frame's.
    ///
    /// Out of order rather than merely equal: the two searches each sort what
    /// they found, but a frame in the overlap between two calls can be found
    /// on the second of them and belong before the first one's last frame.
    /// Measured on radarpi, one frame in 7151 and by 8.3 us. Sending the
    /// frame with no usable time keeps it for the feeders and keeps it out of
    /// the clock fit.
    pub fn at(&mut self, at_sample: u64) -> u64 {
        let ticks = ((self.base + at_sample) as f64 * self.ticks_per_sample) as u64;
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
        assert_eq!(c.at(100), 500);
        assert_eq!(c.at(65_000), 325_000);
    }

    /// The fault this is all about: a dropped buffer is time that passed, and
    /// a clock that does not count it steps back by the whole buffer.
    #[test]
    fn samples_the_radio_dropped_still_pass_on_the_clock() {
        let mut c = Clock::new(2.4e6);
        c.block(0, 65_536);
        assert_eq!(c.at(1_000), 5_000);
        // The next block begins 65_536 samples late: the driver threw one
        // away. The demodulator is reset and counts from zero again, so the
        // frame at its sample 1000 is 131_072 + 1000 into the stream.
        assert_eq!(c.block(131_072, 65_536), Step::Broke(65_536));
        assert_eq!(c.at(1_000), (131_072 + 1_000) * 5);
    }

    #[test]
    fn a_contiguous_block_leaves_the_demodulator_counting() {
        let mut c = Clock::new(2.4e6);
        c.block(0, 65_536);
        assert_eq!(c.block(65_536, 65_536), Step::Continuous);
        // The demodulator's index is still the stream's, so no rebasing.
        assert_eq!(c.at(70_000), 350_000);
    }

    #[test]
    fn a_frame_out_of_order_goes_out_with_no_time_rather_than_a_wrong_one() {
        let mut c = Clock::new(2.4e6);
        c.block(0, 65_536);
        assert_eq!(c.at(20_000), 100_000);
        assert_eq!(c.at(19_980), UNTIMED, "a frame before the last one");
        assert_eq!(c.at(20_000), UNTIMED, "and the same frame twice");
        assert_eq!(c.at(20_001), 100_005, "the next one is timed again");
    }

    /// A remote source reconnecting counts from where its server is, which
    /// can be behind where this one had got to.
    #[test]
    fn a_source_that_starts_counting_again_rebases_rather_than_going_backwards() {
        let mut c = Clock::new(2.4e6);
        c.block(1_000_000, 65_536);
        assert_eq!(c.at(10), 5_000_050);
        assert_eq!(c.block(40, 65_536), Step::Broke(0), "no count of what is missing");
        // Behind the last frame's tick, so untimed until the clock catches
        // up, which is the honest answer and not a frame lost.
        assert_eq!(c.at(10), UNTIMED);
    }

    /// The counter on the wire is 48 bits and rolls over every 6.5 hours, so
    /// a receiver that treated the roll as a step backwards would stop being
    /// usable for mlat after its first one.
    #[test]
    fn the_six_byte_counter_rolling_over_is_not_a_step() {
        let mut c = Clock::new(2.4e6);
        let just_under = (UNTIMED / 5) - 1;
        c.block(just_under, 65_536);
        assert_eq!(c.at(0), (just_under * 5) & UNTIMED);
        assert_eq!(c.at(2), 4, "the tick after the wrap, counted from zero");
    }
}
