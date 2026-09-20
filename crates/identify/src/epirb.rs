//! Where Epirb can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::epirb;
use dsp::biphase::BiphaseDemod;

pub struct Epirb;

impl Signal for Epirb {
    fn id(&self) -> &'static str {
        "epirb"
    }

    fn label(&self) -> &'static str {
        "epirb"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["cospas-sarsat", "sarsat", "plb", "elt", "406"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: WORK_HZ,
            feed_rate_hz: FEED_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// 406 MHz off the recording. A beacon is on for half a second every
    /// fifty, so a file either holds a burst or holds nothing.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, WORK_HZ)
        else {
            return Reading::default();
        };
        let mut demod = BiphaseDemod::new(chan.rate_hz, BAUD);
        let mut framer = epirb::Framer::new();
        let (mut narrow, mut chips) = (Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            chips.clear();
            demod.process(&narrow, &mut chips);
            for chip in &chips {
                if let Some(message) = framer.push(*chip)
                    && let Some(d) = epirb::read(&message)
                {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// The 406 MHz distress band. Nothing else may transmit in it, and beacons
/// sit on channels 3 kHz apart across the lower half of it.
pub const BAND: (f64, f64) = (406_000_000.0, 406_100_000.0);

/// How much of the band one decoder reads. Wide enough to hold a beacon
/// keyed anywhere near the channel it was set to, and to leave the carrier
/// tracker something to find.
pub const CHANNEL_WIDTH_HZ: f64 = 20_000.0;

/// The channel to offer when somebody places one by hand.
pub const DEFAULT_HZ: f64 = 406_025_000.0;

/// The rate to ask the receiver for, which decimates to [`WORK_HZ`] by four.
pub const FEED_HZ: f64 = 38_400.0;

/// The rate the chips are recovered at: twelve samples a chip, which is what
/// the zero-crossing clock wants and no more.
pub const WORK_HZ: f64 = 9_600.0;

pub const BAUD: f64 = 400.0;
