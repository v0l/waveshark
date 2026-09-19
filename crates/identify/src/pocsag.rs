//! Where Pocsag can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use dsp::FmDemod;
use dsp::pocsag::DEVIATION_HZ;
use dsp::pocsag::{PocsagConfig, PocsagDemod};

pub struct Pocsag;

impl Signal for Pocsag {
    fn id(&self) -> &'static str {
        "pocsag"
    }

    fn label(&self) -> &'static str {
        "pager"
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
        439_987_500.0
    }

    /// The channel the recording is tuned to, off the discriminator.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, AUDIO_HZ)
        else {
            return Reading::default();
        };
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut demod = PocsagDemod::new(chan.rate_hz, PocsagConfig::default());
        let (mut narrow, mut audio, mut sends) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
            sends.clear();
            demod.process(&audio, &mut sends);
            for t in &sends {
                rows.extend(decode::pocsag::decoded(&t.to_bytes(), chan.hz()));
            }
        }
        rows.into()
    }
}

/// The channel a POCSAG transmitter occupies: 4.5 kHz deviation at up to 2400
/// bits per second is about 12.5 kHz by Carson, and the allocations are
/// 12.5 or 25 kHz.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// Audio rate the discriminator output is decimated to. A whole number of
/// samples per bit at every rate POCSAG uses: 75, 32 and 16.
pub const AUDIO_HZ: f64 = 38_400.0;
