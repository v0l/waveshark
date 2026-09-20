//! Where Wmbus can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use dsp::wmbus::Demod;

pub struct Wmbus;

impl Signal for Wmbus {
    fn id(&self) -> &'static str {
        "wmbus"
    }

    fn label(&self) -> &'static str {
        "wmbus"
    }

    fn placement(&self) -> Placement {
        Placement::Bands(dsp::wmbus::BANDS.to_vec())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 0.0,
            feed_rate_hz: 0.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        868_950_000.0
    }

    /// The channel the recording is tuned to: mode T and C are the same
    /// demodulator, which says which it read.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut demod = Demod::new(rate_hz);
        if !demod.usable() {
            return Reading::default();
        }
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            for f in demod.process(b) {
                if let Some(d) = decode::wmbus::read(&f.bytes) {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(center_hz)
    }
}

/// Width a mode T or C transmission occupies, for the port and the log.
pub const CHANNEL_WIDTH_HZ: f64 = 250_000.0;
