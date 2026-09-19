//! Where Dfm can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::dfm;
use dsp::fsk::BitSync;

pub struct Dfm;

impl Signal for Dfm {
    fn id(&self) -> &'static str {
        "dfm"
    }

    fn label(&self) -> &'static str {
        "dfm"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["graw", "dfm09", "dfm17"]
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
        let mut framer = dfm::Framer::new();
        let center = common::Hz(center_hz as u64);
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            sync.process(b, framer.sink());
            for bytes in framer.take() {
                if let Some(d) = dfm::decoded(&bytes, center) {
                    rows.push(d);
                }
            }
            framer.trim();
        }
        rows.into()
    }
}

/// The meteorological aids band, the same one the RS41 is launched into.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Chips a second. Two chips to a bit, so the sonde sends 1250 bits a
/// second.
pub const BAUD: f64 = 2_500.0;

/// The channel a DFM is tuned to. The meteorological band is stepped in
/// 10 kHz, and the sonde occupies most of a 12.5 kHz channel.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// What the signal occupies, which is the intermediate filter zilog80's
/// decoder defaults to.
pub const OCCUPIED_HZ: f64 = 12_000.0;
