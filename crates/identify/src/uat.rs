//! Where Uat can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::uat::{self};
use dsp::fsk::{SyncBurst, SyncDetector};

/// UAT as the auto node and the tables know it: the 978 MHz channel, read
/// off the span because a frame is over before a detector could open a
/// source on it.
pub struct Uat;

impl Signal for Uat {
    fn id(&self) -> &'static str {
        "uat"
    }

    fn label(&self) -> &'static str {
        "uat"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["uat978", "adsb978", "fisb"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(vec![uat::CHANNEL_HZ])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            // Two samples a bit is the floor the correlator reads at.
            min_rate_hz: 2.0 * uat::BAUD,
            feed_rate_hz: 2_400_000.0,
            span_wide: true,
            families: &[],
        }
    }

    /// 978 MHz off the span, the code deciding what is a burst: a sync word
    /// that no Reed-Solomon codeword follows was noise.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz < 2.0 * uat::BAUD
            || (center_hz - uat::CHANNEL_HZ).abs() > rate_hz / 2.0 - CHANNEL_WIDTH_HZ / 2.0
        {
            return Reading::default();
        }
        let mut det = SyncDetector::new(rate_hz, uat::BAUD, uat::patterns());
        let mut bursts = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            det.process_valid(b, &mut bursts, &|b: &SyncBurst| uat::correct(b).is_some());
        }
        let center = common::Hz(uat::CHANNEL_HZ as u64);
        bursts
            .iter()
            .filter_map(uat::correct)
            .filter_map(|c| {
                let frame = uat::parse(&c.data)?;
                Some(uat::decoded(&frame, &c.data, center))
            })
            .flatten()
            .collect::<Vec<_>>()
            .into()
    }
}

/// What the signal occupies: 1.04 Mbit/s keyed 312.5 kHz either side, which
/// is about 1.4 MHz by Carson's rule, plus room for a tuner's error.
pub const CHANNEL_WIDTH_HZ: f64 = 2_000_000.0;
