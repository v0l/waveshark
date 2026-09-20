//! Where Lms6 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::lms6;
use dsp::fsk::BitSync;

pub struct Lms6;

impl Signal for Lms6 {
    fn id(&self) -> &'static str {
        "lms6"
    }

    fn label(&self) -> &'static str {
        "lms6"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["lms6-403", "lockheed"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 4.0 * BAUD,
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        403_000_000.0
    }

    /// The channel the recording is tuned to. A sonde is found by scanning
    /// and the recording is the scan's result, so the middle of the span is
    /// the channel.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut sync = BitSync::with_bandwidth(rate_hz, BAUD, OCCUPIED_HZ);
        if !sync.usable() {
            return Reading::default();
        }
        let mut framer = lms6::Framer::new();
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            sync.process(b, framer.sink());
            for bytes in framer.take() {
                if let Some(d) = lms6::read(&bytes) {
                    rows.push(d);
                }
            }
            framer.trim();
        }
        Reading::from(rows).at(center_hz)
    }
}

/// The meteorological aids band.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Coded bits a second. Half of them survive the trellis, so the frame rate
/// is one a second.
pub const BAUD: f64 = 4_800.0;

/// The channel an LMS6 is tuned to.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// What the signal occupies.
pub const OCCUPIED_HZ: f64 = 12_000.0;
