//! Where Gsm can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::gsm;
use dsp::gsm::{GsmConfig, Hit, SchDetector};

pub struct Gsm;

impl Signal for Gsm {
    fn id(&self) -> &'static str {
        "gsm"
    }

    fn label(&self) -> &'static str {
        "gsm"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["gsm-sch"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(dsp::gsm::DOWNLINK_BANDS.to_vec())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            // Three samples a symbol is the floor the detector refuses
            // below; fed by band it is given four, since the burst is
            // sampled where the training sequence says, not where a sample
            // happens to land, so the interpolator wants something to work
            // with.
            min_rate_hz: dsp::gsm::SYMBOL_RATE * 3.0,
            feed_rate_hz: 1_200_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The beacon carrier the recording is tuned to: its synchronisation
    /// bursts and the blocks its control channels carry.
    ///
    /// The beacon alone. A cell hands a handset to another carrier, and
    /// following that needs a second stream cut from the span and timed off
    /// this one's clock, which is a thing the graph does and a caller with a
    /// file does not have. An assignment read here is a row like any other
    /// and nothing is opened for it.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if !SchDetector::rate_is_enough(rate_hz) {
            return Reading::default();
        }
        let mut det = SchDetector::new(rate_hz, center_hz, center_hz, GsmConfig::default());
        let center = common::Hz(center_hz as u64);
        let mut hits = Vec::new();
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            hits.clear();
            det.process(b, &mut hits);
            for hit in &hits {
                // Two kinds of evidence off one carrier: the synchronisation
                // burst's 25 bit field, and the 23 byte blocks the broadcast
                // and common control channels carry. The length is what
                // tells them apart, which is the same job the band does for
                // everything else on it.
                let bytes = match hit {
                    Hit::Sync(s) => match dsp::gsm::sch::pack(&s.sch) {
                        Some(b) => b.to_vec(),
                        None => continue,
                    },
                    Hit::Block(b) => b.bytes.to_vec(),
                };
                rows.extend(gsm::rows(&bytes, center));
            }
        }
        rows.into()
    }
}

/// What one carrier occupies, and the width a burst was heard through.
pub const CHANNEL_WIDTH_HZ: f64 = dsp::gsm::CHANNEL_SPACING_HZ;

/// Where to look when nothing says otherwise: the middle of the E-GSM 900
/// downlink, which is the band most likely to hold a beacon in Europe. There
/// is no frequency worth compiling in beyond that, so the scanner table
/// carries the channel and this is only what an unconfigured node opens on.
pub const DEFAULT_HZ: f64 = 947_400_000.0;
