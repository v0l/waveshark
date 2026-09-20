//! Where Iridium can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::iridium::{self, RING_ALERT_HZ};
use dsp::dqpsk::{DqpskConfig, DqpskDemod};

/// Iridium as the auto node and the tables know it.
pub struct Iridium;

impl Signal for Iridium {
    fn id(&self) -> &'static str {
        "iridium"
    }

    fn label(&self) -> &'static str {
        "iridium"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["ira", "ring alert", "iridium-ra"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![(iridium::BASE_HZ, iridium::SIMPLEX_BAND_HZ.1)])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[iridium::CHANNEL_WIDTH_HZ],
            // The channel and the doppler either side of it, which is what
            // the demodulator searches over.
            min_rate_hz: 150_000.0,
            feed_rate_hz: FEED_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        RING_ALERT_HZ
    }

    /// A simplex ring alert channel off the recording.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let half_pass = iridium::CHANNEL_WIDTH_HZ / 2.0 + iridium::DOPPLER_HZ;
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, half_pass * 2.0, WORK_HZ)
        else {
            return Reading::default();
        };
        let mut demod = DqpskDemod::new(chan.rate_hz, DqpskConfig::IRIDIUM);
        let (mut narrow, mut bursts) = (Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            bursts.clear();
            demod.process(&narrow, &mut bursts);
            for burst in &bursts {
                // Read here as well, so a burst whose blocks do not check
                // never becomes a row: an access word comes up in noise
                // eventually, and its blocks do not.
                let Some(bytes) = iridium::pack(&burst.bits) else { continue };
                if let Some(d) = iridium::read(&bytes, chan.hz()) {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// The rate to ask the receiver for. Two work rates, so the channel filter
/// has somewhere to roll off.
pub const FEED_HZ: f64 = 500_000.0;

/// The rate the demodulator runs at: ten samples a symbol at 25 kbaud, which
/// is enough to put the symbol clock inside a tenth of a symbol without
/// interpolating.
pub const WORK_HZ: f64 = 250_000.0;
