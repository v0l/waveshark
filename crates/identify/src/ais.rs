//! Where Ais can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::ais;
use dsp::ais::CHANNEL_HZ;
use dsp::ais::{AisConfig, AisDetector};

/// AIS as the auto node and the tables know it: both channels at once,
/// since stations alternate between them and half of them is half the
/// traffic.
pub struct Ais;

impl Signal for Ais {
    fn id(&self) -> &'static str {
        "ais"
    }

    fn label(&self) -> &'static str {
        "ais"
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![(CHANNEL_HZ[0] - CHANNEL_WIDTH_HZ, CHANNEL_HZ[1] + CHANNEL_WIDTH_HZ)])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_HZ[1] - CHANNEL_HZ[0] + 2.0 * CHANNEL_WIDTH_HZ],
            // The detector mixes both channels itself and wants room between
            // them, so this stays well above their separation.
            min_rate_hz: 150_000.0,
            feed_rate_hz: 600_000.0,
            span_wide: true,
            families: &[],
        }
    }

    /// Both channels at once, the way the receiver reads them: the
    /// demodulator is handed the span and says which of the two each frame
    /// arrived on.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let edge = rate_hz / 2.0 - CHANNEL_WIDTH_HZ;
        if CHANNEL_HZ.iter().any(|c| (c - center_hz).abs() > edge) {
            return Reading::default();
        }
        let mut det = AisDetector::new(rate_hz, center_hz, AisConfig::default());
        let mut frames = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            det.process(b, &mut frames);
        }
        frames
            .iter()
            .filter_map(|f| {
                let parsed = ais::parse(&f.payload).ok()?;
                Some(ais::read(&parsed))
            })
            .collect::<Vec<_>>()
            .into()
    }
}

/// The width one AIS channel occupies, which is what a frame was heard
/// through whichever of the two carried it.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;
