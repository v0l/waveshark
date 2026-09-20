//! Where Imet can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::imet;
use dsp::FmDemod;
use dsp::afsk::{AfskBits, AfskConfig};

pub struct Imet;

impl Signal for Imet {
    fn id(&self) -> &'static str {
        "imet"
    }

    fn label(&self) -> &'static str {
        "imet"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["imet4", "intermet"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: AUDIO_HZ,
            feed_rate_hz: AUDIO_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        403_000_000.0
    }

    /// The channel the recording is tuned to, read as AFSK off the
    /// discriminator.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut bits = AfskBits::new(chan.rate_hz, AfskConfig::default());
        let mut framer = imet::Framer::new();
        let (mut narrow, mut audio, mut symbols) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            symbols.clear();
            bits.process(&audio, &mut symbols);
            for sym in &symbols {
                if let Some(run) = framer.push(*sym)
                    && let Some(d) = imet::read(&run, chan.hz())
                {
                    rows.push(d);
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// Audio rate the discriminator output is decimated to, as for APRS: well
/// above the 2200 Hz upper tone.
pub const AUDIO_HZ: f64 = 48_000.0;

/// The meteorological aids band, where every sonde is.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// The channel an iMet occupies. Wider than the other sondes' because this
/// one is voice-bandwidth FM with tones in it rather than keyed data.
pub const CHANNEL_WIDTH_HZ: f64 = 16_000.0;

/// Peak deviation an iMet keys.
pub const DEVIATION_HZ: f64 = 3_000.0;
