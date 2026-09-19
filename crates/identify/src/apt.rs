//! Where Apt can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::apt;
use dsp::FmDemod;

pub struct Apt;

impl Signal for Apt {
    fn id(&self) -> &'static str {
        "apt"
    }

    fn label(&self) -> &'static str {
        "apt"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["noaa"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(SATELLITES.iter().map(|(_, hz)| *hz).collect())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: AUDIO_HZ,
            feed_rate_hz: 100_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// A satellite's line scan off the discriminator. Pictures rather than
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
        let mut fm = FmDemod::new(chan.rate_hz, DEVIATION_HZ);
        let mut rx = apt::Receiver::new(AUDIO_HZ);
        let (mut narrow, mut audio, mut at_rate) = (Vec::new(), Vec::new(), Vec::new());
        let mut pictures = 0usize;
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            audio.clear();
            fm.process(&narrow, &mut audio);
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

/// The rate the picture is read at: ten samples a word, which is a whole
/// number of them and leaves the channel room, since a 34 kHz channel cannot
/// be demodulated at the 20.8 kHz that five samples a word would give.
pub const AUDIO_HZ: f64 = 10.0 * apt::WORD_RATE;

/// The channel a satellite occupies: the transponder is 34 kHz wide and the
/// spacing on the band is 40.
pub const CHANNEL_WIDTH_HZ: f64 = 40_000.0;

/// NOAA 19, which is the one most likely to be overhead and working.
pub const DEFAULT_HZ: f64 = 137_100_000.0;

/// The three birds still sending pictures, and what they are called.
pub const SATELLITES: [(&str, f64); 3] =
    [("NOAA 19", 137_100_000.0), ("NOAA 15", 137_620_000.0), ("NOAA 18", 137_912_500.0)];

/// Peak deviation of the downlink, which the guide gives as 17 kHz.
pub const DEVIATION_HZ: f64 = 17_000.0;
