//! Where Flex can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use dsp::FmDemod;
use dsp::flex::CHANNEL_WIDTH_HZ;
use dsp::flex::DEVIATION_HZ;
use dsp::flex::{FlexConfig, FlexDemod};

pub struct Flex;

impl Signal for Flex {
    fn id(&self) -> &'static str {
        "flex"
    }

    fn label(&self) -> &'static str {
        "FLEX pager"
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility, Usage::Ism])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 192_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        169_650_000.0
    }

    /// The channel the recording is tuned to, off the discriminator.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut demod = FlexDemod::new(chan.rate_hz, FlexConfig::default());
        let (mut narrow, mut audio, mut frames) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            frames.clear();
            demod.process(&audio, &mut frames);
            for f in &frames {
                rows.extend(decode::flex::decoded(&f.to_bytes(), chan.hz()));
            }
        }
        rows.into()
    }
}

/// Audio rate the discriminator output is decimated to. A whole number of
/// samples per symbol at both FLEX speeds: 24 and 12.
pub const AUDIO_HZ: f64 = 38_400.0;
