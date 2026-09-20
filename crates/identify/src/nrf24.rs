//! Where Nrf24 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::nrf24;
use decode::nrf24::BAND;

pub struct Nrf24;

impl Signal for Nrf24 {
    fn id(&self) -> &'static str {
        "nrf24"
    }

    fn label(&self) -> &'static str {
        "nrf24"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["xn297", "shockburst"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 1_000_000.0,
            feed_rate_hz: WORK_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        2_441_000_000.0
    }

    /// Both bit rates on the channel the recording is tuned to: nothing on
    /// the air says which a burst is until it frames.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, WORK_HZ)
        else {
            return Reading::default();
        };
        let mut readers: Vec<nrf24::Reader> =
            nrf24::BAUDS.iter().map(|b| nrf24::Reader::new(chan.rate_hz, *b)).collect();
        if readers.iter().all(|r| !r.usable()) {
            return Reading::default();
        }
        let (mut narrow, mut found) = (Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            found.clear();
            for r in &mut readers {
                r.read(&narrow, &mut found);
            }
            for (_at, p) in &found {
                if let Some(d) = nrf24::read(&p.on_air()) {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// The channel one burst occupies. A 1 Mbit/s link keys about 320 kHz of
/// deviation, which is a megahertz by Carson, and the channels are spaced a
/// megahertz apart.
pub const CHANNEL_WIDTH_HZ: f64 = 1_000_000.0;

/// Rate the channel is cut down to before the bit clocks read it: four
/// samples a symbol at the faster rate, which is where [`BitSync`] stops.
pub const WORK_HZ: f64 = 4_000_000.0;
