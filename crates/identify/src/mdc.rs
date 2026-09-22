//! Where Mdc can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::mdc1200;
use dsp::FmDemod;
use dsp::msk::{MskConfig, MskDemod};

pub struct Mdc;

impl Signal for Mdc {
    fn id(&self) -> &'static str {
        "mdc1200"
    }

    fn label(&self) -> &'static str {
        "mdc-1200"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["mdc", "ani"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility, Usage::Amateur])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 96_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, off the discriminator.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut msk = MskDemod::new(chan.rate_hz, MskConfig::FFSK1200);
        let mut framer = mdc1200::Framer::default();
        let (mut narrow, mut audio, mut bits) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            bits.clear();
            msk.process(&audio, &mut bits);
            for &bit in &bits {
                if let Some(info) = framer.push(bit)
                    && let Some(d) = mdc1200::read(&info)
                {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// The channel an LMR transmission occupies. Narrowband, which is what every
/// fleet was made to move to.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// A VHF business channel, which is where most of this traffic is. Nothing
/// about MDC is band specific: it is the frequency the node is built with
/// until the scanner table or an operator says another.
pub const DEFAULT_HZ: f64 = 154_000_000.0;

/// Audio rate the discriminator output is decimated to: well above the
/// 1800 Hz tone, and a rate the correlators are tested at.
pub const AUDIO_HZ: f64 = 24_000.0;

/// Peak deviation of a narrowband channel.
pub const DEVIATION_HZ: f64 = 2_500.0;
