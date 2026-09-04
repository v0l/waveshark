//! Source vocabulary: a transmitter found in a wideband stream, and the
//! samples extracted for it.
//!
//! These live in `common` for the same reason [`crate::pulse`] does: they are
//! carried on graph edges, and the graph must not depend on the DSP that
//! happens to produce them.
//!
//! A source is the unit the receiver works in once it stops assuming a
//! channel grid. Something is transmitting at a frequency with a width, from
//! one instant until another, and everything it does in between is one
//! continuous stream of samples at a rate suited to that width. A packet
//! lasting four milliseconds is such a stream, and so is a broadcast carrier
//! that never stops; the difference between them is how long the stream runs,
//! and nothing downstream has to know which it is dealing with.

use crate::C32;

/// Identifies one source for as long as it transmits. Never reused within a
/// stream, so a consumer keyed on it cannot confuse a new transmitter with
/// the one that just went quiet at the same frequency.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SourceId(pub u64);

/// Where a block sits in its source's life.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SourceState {
    /// The first block: a consumer builds whatever it keeps per source here.
    Opened,
    /// The source is still transmitting.
    Running,
    /// The last block, which may be empty. The consumer can drop its state
    /// once it has processed this.
    Closed,
    /// The last block, and a wider stream for the same transmitter follows
    /// under a new id from the transmitter's start. The consumer should drop
    /// its state without reading anything more into what it has: the burst
    /// continues in the new stream, and a decoder flushed here would report
    /// half of it.
    Superseded,
}

/// One run of a source's samples, at the source's own rate.
///
/// Blocks for one source arrive in order and with no gaps, so a consumer can
/// treat the sequence as a stream. Blocks for different sources are
/// independent of one another.
#[derive(Clone, Debug, PartialEq)]
pub struct SourceBlock {
    pub id: SourceId,
    pub state: SourceState,
    /// RF centre of `samples`, in hertz.
    pub center_hz: u64,
    /// Width the extraction kept, in hertz. Wider than the signal by a
    /// margin, since the edges measured on a weak burst are the loud middle
    /// of it and not its extent.
    pub bandwidth_hz: f64,
    /// Width the detector measured for the signal itself, in hertz, before
    /// that margin. What a front end that asks whether the signal fills a
    /// channel should look at.
    pub signal_hz: f64,
    /// Sample rate of `samples`.
    pub rate: f64,
    /// Index in the *wideband* stream of the instant `samples` begins, for
    /// placing the block in time against the stream it came from.
    pub start_sample: u64,
    /// Peak SNR the detector measured for the source so far, in dB.
    pub snr_db: f32,
    pub samples: Vec<C32>,
}
