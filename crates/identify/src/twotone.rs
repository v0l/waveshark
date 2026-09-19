//! Where TwoTone can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::twotone::{self, Sequential};
use dsp::FmDemod;
use dsp::tone::{RunConfig, ToneRuns};

pub struct TwoTone;

impl Signal for TwoTone {
    fn id(&self) -> &'static str {
        "twotone"
    }

    fn label(&self) -> &'static str {
        "two-tone"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["quick call", "qcii", "two tone paging"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, off the discriminator.
    ///
    /// Nothing on the air says whose pager a pair belongs to, so a row here
    /// carries the tones and no name: naming them is a table the operator
    /// keeps, which a recording does not come with.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut tones = ToneRuns::new(chan.rate_hz, RunConfig::default());
        let mut pages = Sequential::default();
        let (mut narrow, mut audio, mut runs) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            runs.clear();
            tones.process(&audio, &mut runs);
            for run in &runs {
                if let Some(page) = pages.run(*run)
                    && let Some(d) = twotone::decoded(&twotone::framed(&page, None), chan.hz())
                {
                    rows.push(d);
                }
            }
        }
        rows.into()
    }
}

/// A dispatch channel is wide FM, and a narrowband one fits inside it.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// A VHF fire and ambulance dispatch channel, which is the traffic this
/// reads. Nothing about the scheme is band specific: it is the frequency the
/// node is built with until the scanner table or an operator says another.
pub const DEFAULT_HZ: f64 = 154_000_000.0;

/// Peak deviation of a wideband LMR channel.
pub const DEVIATION_HZ: f64 = 5_000.0;

/// Audio rate the discriminator output is decimated to. Twice the top of the
/// tone band with room for the filter.
pub const AUDIO_HZ: f64 = 8_000.0;
