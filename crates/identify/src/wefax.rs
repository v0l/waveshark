//! Where Wefax can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::wefax;
use dsp::ssb::{Sideband, SsbDemod};

pub struct Wefax;

impl Signal for Wefax {
    fn id(&self) -> &'static str {
        "wefax"
    }

    fn label(&self) -> &'static str {
        "wefax"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["radiofax", "weatherfax", "fax"]
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: AUDIO_HZ,
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// A weather chart off a sideband receiver. Pictures rather than
    /// rows, so what it reports is how many came out whole.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some((factor, mut resample)) = dsp::resample::stage(rate_hz, AUDIO_HZ, 4096) else {
            return Reading::default();
        };
        let Some(mut chan) = crate::Channel::new(
            rate_hz,
            center_hz,
            center_hz,
            CHANNEL_WIDTH_HZ,
            rate_hz / factor as f64,
        ) else {
            return Reading::default();
        };
        let mut ssb = SsbDemod::new(chan.rate_hz, Sideband::Upper, PASS_LOW_HZ, PASS_HIGH_HZ);
        let mut rx = wefax::Receiver::new(AUDIO_HZ);
        let (mut narrow, mut audio, mut at_rate) = (Vec::new(), Vec::new(), Vec::new());
        let mut pictures = 0usize;
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            ssb.process(&narrow, &mut audio);
            let feed = match resample.as_mut() {
                Some(r) => {
                    at_rate.clear();
                    r.process_real(&audio, &mut at_rate);
                    &at_rate
                }
                None => &audio,
            };
            if let Some(rows) = rx.push(feed)
                && rows.complete
            {
                pictures += 1;
            }
        }
        Reading { pictures, ..Reading::default() }
    }
}

/// The rate the picture is read at. A line is 1809 pixels in half a second at
/// 120 lines a minute, so this is twelve samples a pixel, and the
/// discriminator has nothing to gain from more.
pub const AUDIO_HZ: f64 = 44_100.0;

/// The channel a fax broadcast occupies: 400 Hz either side of a tone at
/// 1900, and the room a sideband filter needs around that.
pub const CHANNEL_WIDTH_HZ: f64 = 3_000.0;

/// Hamburg/Pinneberg on 7880 kHz, which is the schedule most of Europe
/// listens to and is on the air around the clock.
pub const DEFAULT_HZ: f64 = 7_880_000.0;

/// What the sideband filter passes: the shift with room either side for a
/// transmitter tuned a little off.
pub const PASS_LOW_HZ: f64 = 1_100.0;

pub const PASS_HIGH_HZ: f64 = 2_700.0;
