//! Where Ble can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::ble::CHANNEL_WIDTH_HZ;
use dsp::ble::ADV_CHANNELS;
use dsp::ble::{BleConfig, BleDetector};

/// BLE advertising as the auto node and the tables know it: whichever of
/// the three channels the span holds, read off the span because an
/// advertisement is 80 us of a hopping device that may never be heard
/// twice, which is not enough for a source to open around.
pub struct Ble;

impl Signal for Ble {
    fn id(&self) -> &'static str {
        "ble"
    }

    fn label(&self) -> &'static str {
        "ble"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["bluetooth"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(ADV_CHANNELS.iter().map(|(_, hz)| *hz).collect())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 4_000_000.0,
            feed_rate_hz: 8_000_000.0,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        2_426_000_000.0
    }

    /// The three advertising channels the span reaches, read together.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz < self.shape().min_rate_hz {
            return Reading::default();
        }
        let mut det = BleDetector::new(rate_hz, center_hz, BleConfig::default());
        if det.channels().is_empty() {
            return Reading::default();
        }
        let mut frames = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            det.process(b, &mut frames);
        }
        frames
            .iter()
            .filter_map(|f| {
                let hz = ADV_CHANNELS
                    .iter()
                    .find(|(c, _)| *c == f.channel)
                    .map(|(_, hz)| *hz)
                    .unwrap_or(center_hz);
                decode::ble::read(&f.pdu, common::Hz(hz as u64))
            })
            .collect::<Vec<_>>()
            .into()
    }
}
