//! Where Ieee802154 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::ieee802154::CHANNEL_WIDTH_HZ;
use dsp::oqpsk::{OQPSK_2450, OqpskConfig, OqpskDetector};
use dsp::oqpsk::{channel_2450_hz, channels_2450};

/// 802.15.4 as the auto node and the tables know it: whichever of the sixteen
/// channels the span holds, read off the span because a frame is a
/// millisecond of a device that may not transmit again for an hour, which is
/// not enough for a source to open around.
pub struct Ieee802154;

impl Signal for Ieee802154 {
    fn id(&self) -> &'static str {
        "ieee802154"
    }

    fn label(&self) -> &'static str {
        "802.15.4"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["zigbee", "thread", "matter"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(channels_2450().into_iter().map(|(_, hz)| hz).collect())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 6_000_000.0,
            feed_rate_hz: 8_000_000.0,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        channel_2450_hz(11).unwrap()
    }

    /// Every channel of the 2450 MHz plan the span covers.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz < self.shape().min_rate_hz {
            return Reading::default();
        }
        let mut det = OqpskDetector::new(
            rate_hz,
            center_hz,
            OQPSK_2450,
            &channels_2450(),
            OqpskConfig::default(),
        );
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
                let hz = channel_2450_hz(f.channel).unwrap_or(center_hz);
                decode::ieee802154::read(&f.psdu, common::Hz(hz as u64))
            })
            .collect::<Vec<_>>()
            .into()
    }
}
