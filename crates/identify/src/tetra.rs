//! Where Tetra can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::tetra;
use dsp::tetra::OCCUPIED_HZ;
use dsp::tetra::{TetraConfig, TetraDemod, TetraRx};

/// TETRA as the auto node knows it: placed by band, not width. Unlike the
/// amateur channels its carriers live in licensed downlink allocations, and
/// its hunt correlates continuously, not worth paying on every 433 MHz
/// burst.
pub struct Tetra;

impl Signal for Tetra {
    fn id(&self) -> &'static str {
        "tetra"
    }

    fn label(&self) -> &'static str {
        "tetra"
    }

    fn placement(&self) -> Placement {
        Placement::Bands(dsp::tetra::DOWNLINK_BANDS.to_vec())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: MIN_RATE_HZ,
            feed_rate_hz: 300_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        390_000_000.0
    }

    /// The carrier the recording is tuned to: every block its slots carry,
    /// as the rows they decode to.
    ///
    /// The main carrier alone. A cell grants a call a traffic carrier and
    /// the receiver opens one, timed off this carrier's clock; that needs a
    /// second stream cut from the span, which is a thing the graph does. A
    /// grant read here is a row and nothing is opened for it. No decryption
    /// either: a key belongs to an operator's settings, not to a recording.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, OCCUPIED_HZ, DEMOD_HZ)
        else {
            return Reading::default();
        };
        let mut demod = TetraDemod::new(chan.rate_hz, TetraConfig::default());
        let mut rx = TetraRx::new();
        let (mut narrow, mut bursts, mut blocks) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            bursts.clear();
            demod.process(&narrow, &mut bursts);
            blocks.clear();
            for burst in &bursts {
                rx.push(burst, &mut blocks);
            }
            for block in &blocks {
                let Some(event) = tetra::Event::from_block(block) else { continue };
                if let Some(d) = tetra::decoded(&event.to_bytes(), chan.hz()) {
                    rows.push(d);
                }
            }
        }
        rows.into()
    }
}

/// The raster TETRA carriers sit on.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// The least stream rate worth building the demodulator for. The source
/// extractor's floor of 25 kS/s clears it; the occupied signal only just
/// fits there, and the demodulator's tests show it still reads.
pub const MIN_RATE_HZ: f64 = OCCUPIED_HZ;

/// Rate the demodulator likes to run at: four samples a symbol.
pub const DEMOD_HZ: f64 = 72_000.0;
