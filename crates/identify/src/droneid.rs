//! Where DroneId can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;

/// DroneID as the auto node and the tables know it.
pub struct DroneId;

impl Signal for DroneId {
    fn id(&self) -> &'static str {
        "droneid"
    }

    fn label(&self) -> &'static str {
        "droneid"
    }

    fn placement(&self) -> Placement {
        Placement::Channels(channels())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[WIDTH_HZ],
            min_rate_hz: RATE_HZ,
            feed_rate_hz: RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// Every 10 MHz DroneID centre the span covers.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz + 1.0 < RATE_HZ {
            return Reading::default();
        }
        let Some(mut span) =
            dsp::droneid::DroneIdSpan::new(rate_hz, center_hz, &channels(), THRESHOLD)
        else {
            return Reading::default();
        };
        let mut bursts = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            span.process(b, &mut bursts);
        }
        bursts
            .iter()
            .filter_map(|b| decode::droneid::read(&decode::droneid::wrap(&b.frame)))
            .collect::<Vec<_>>()
            .into()
    }
}

/// Every centre DroneID has been seen on, 2.4 GHz then 5.8.
pub fn channels() -> Vec<f64> {
    dsp::droneid::CENTERS_2G4_HZ
        .iter()
        .chain(dsp::droneid::CENTERS_5G8_HZ.iter())
        .copied()
        .collect()
}

/// The centre a receiver picks when it has to pick one: the middle of the
/// 2.4 GHz set, where the bursts in the bench captures were.
pub const DEFAULT_HZ: f64 = 2_444_500_000.0;

/// The rate the frame is defined at, which is also the rate this asks for.
pub const RATE_HZ: f64 = dsp::droneid::RATE;

/// The occupied bandwidth: 600 carriers 15 kHz apart, plus guards.
pub const WIDTH_HZ: f64 = dsp::droneid::WIDTH_HZ;

/// How strongly the Zadoff-Chu symbol has to correlate. Off air, a burst
/// scores 0.88 to 0.99 and nothing else in a busy 2.4 GHz band comes near
/// half of that, so this is not a knife edge.
pub const THRESHOLD: f32 = 0.5;
