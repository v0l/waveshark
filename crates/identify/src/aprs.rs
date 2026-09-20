//! Where Aprs can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::ax25;
use dsp::FmDemod;
use dsp::afsk::{AfskConfig, AfskDemod};

pub struct Aprs;

impl Signal for Aprs {
    fn id(&self) -> &'static str {
        "aprs"
    }

    fn label(&self) -> &'static str {
        "aprs"
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Amateur])
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
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, off the discriminator: 1200
    /// baud AFSK in HDLC, which is what an AX.25 packet channel is.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut afsk = AfskDemod::new(chan.rate_hz, AfskConfig::default());
        let (mut narrow, mut audio, mut frames) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            frames.clear();
            afsk.process(&audio, &mut frames);
            for bytes in &frames {
                if let Ok(f) = ax25::parse(bytes) {
                    rows.push(decode::aprs::read(&f));
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// The channel a 2 m packet transmission occupies.
pub const CHANNEL_WIDTH_HZ: f64 = 16_000.0;

/// Where APRS is across Europe. North America uses 144.390 and Japan 144.640;
/// the scanner configuration decides which, and this is only the default the
/// node is built with before it is told.
pub const DEFAULT_HZ: f64 = 144_800_000.0;

/// Audio rate the discriminator output is decimated to. Comfortably above the
/// 2200 Hz upper tone and a rate the correlators are tested at.
pub const AUDIO_HZ: f64 = 48_000.0;

/// Peak deviation a 2 m packet channel uses.
pub const DEVIATION_HZ: f64 = 3_000.0;
