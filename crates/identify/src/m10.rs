//! Where M10 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::m10;
use dsp::fsk::BitSync;

pub struct M10;

impl Signal for M10 {
    fn id(&self) -> &'static str {
        "m10"
    }

    fn label(&self) -> &'static str {
        "m10"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["m20", "meteomodem"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 4.0 * BAUD,
            feed_rate_hz: 96_000.0,
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
        let mut framer = m10::Framer::new();
        let center = common::Hz(center_hz as u64);
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            sync.process(b, framer.sink());
            for bytes in framer.take() {
                if let Some(d) = m10::decoded(&bytes, center) {
                    rows.push(d);
                }
            }
            framer.trim();
        }
        rows.into()
    }
}

/// The meteorological aids band.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Chips a second. An M10 keys 9615 and an M20 9600, which is a sixth of a
/// chip apart over a whole frame and well inside what the clock recovery
/// follows, so both are read at the one rate.
pub const BAUD: f64 = 9_615.0;

/// The channel a Meteomodem sonde is tuned to.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// What the signal occupies.
pub const OCCUPIED_HZ: f64 = 20_000.0;
