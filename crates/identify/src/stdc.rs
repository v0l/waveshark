//! Where Stdc can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::inmarsat::{BAND_HZ, stdc};
use dsp::bpsk::{BpskConfig, BpskDemod};

pub struct Stdc;

impl Signal for Stdc {
    fn id(&self) -> &'static str {
        "stdc"
    }

    fn label(&self) -> &'static str {
        "std-c"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["inmarsat-c", "inmarsatc", "egc", "safetynet"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND_HZ])
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

    /// Inmarsat-C off the channel the recording is tuned to.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, WORK_HZ)
        else {
            return Reading::default();
        };
        let mut demod = BpskDemod::new(chan.rate_hz, BpskConfig::INMARSAT_C);
        let mut framer = stdc::Framer::new();
        let (mut narrow, mut soft, mut frames) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            soft.clear();
            demod.process(&narrow, &mut soft);
            frames.clear();
            framer.process(&soft, &mut frames);
            for f in &frames {
                for p in stdc::packets(&f.bytes) {
                    if !p.check_ok {
                        continue;
                    }
                    if let Some(d) = decode::inmarsat::read(&p.bytes) {
                        rows.push(d);
                    }
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// What one channel occupies. The carrier is 1200 symbols a second and a
/// receiver is told to give it 5 to 10 kHz.
pub const CHANNEL_WIDTH_HZ: f64 = 6_000.0;

/// A network control station's common channel, which is what an idle
/// terminal listens to. Which one is in view depends on the ocean region, so
/// this is only what the node is built with before it is told otherwise.
pub const DEFAULT_HZ: f64 = 1_541_450_000.0;

/// The rate to ask the receiver for, which decimates to [`WORK_HZ`] by four.
pub const FEED_HZ: f64 = 38_400.0;

/// The rate the symbols are recovered at: eight samples a symbol.
pub const WORK_HZ: f64 = 9_600.0;
