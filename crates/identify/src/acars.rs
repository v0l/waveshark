//! Where Acars can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::acars;
use dsp::AmDemod;
use dsp::msk::{MskConfig, MskDemod};

pub struct Acars;

impl Signal for Acars {
    fn id(&self) -> &'static str {
        "acars"
    }

    fn label(&self) -> &'static str {
        "acars"
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![(129e6, 137e6)])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 100_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The middle of the recording as the channel: ACARS is on a band of
    /// 25 kHz channels and the recording was tuned to one of them.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut am = AmDemod::new(chan.rate_hz, CARRIER_TRACK_HZ);
        let mut msk = MskDemod::new(chan.rate_hz, MskConfig::ACARS);
        let mut framer = acars::Framer::new();
        let (mut narrow, mut audio, mut bits, mut blocks) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            am.process(&narrow, &mut audio);
            bits.clear();
            msk.process(&audio, &mut bits);
            blocks.clear();
            framer.process(&bits, &mut blocks);
            for block in &blocks {
                if let Some(m) = acars::parse(block) {
                    rows.push(acars::read(&m));
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// An airband channel is 25 kHz on the grid and the signal inside it is a few
/// kilohertz of MSK on an AM carrier.
pub const CHANNEL_WIDTH_HZ: f64 = 15_000.0;

/// The primary ACARS channel across Europe. North America uses 131.550 and
/// there are half a dozen others; the scanner table decides, and this is only
/// what the node is built with before it is told.
pub const DEFAULT_HZ: f64 = 131_725_000.0;

/// Audio rate the envelope is decimated to, which is what `acarsdec` works at
/// and what the demodulator's constants were measured at.
pub const AUDIO_HZ: f64 = 12_500.0;

/// How fast the carrier estimate follows a fading aircraft. A few hertz: fast
/// enough for an aircraft turning, slow enough to leave the 1200 Hz tone
/// alone.
pub const CARRIER_TRACK_HZ: f64 = 5.0;
