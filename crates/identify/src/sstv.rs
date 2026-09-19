//! Where Sstv can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;
use decode::sstv;
use dsp::FmDemod;

pub struct Sstv;

impl Signal for Sstv {
    fn id(&self) -> &'static str {
        "sstv"
    }

    fn label(&self) -> &'static str {
        "sstv"
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Amateur])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            // The picture is read off 44.1 kHz of audio and the stage only
            // decimates, so a stream slower than that is one it cannot
            // reach, whatever the channel in it is worth.
            min_rate_hz: AUDIO_HZ,
            feed_rate_hz: 100_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, off the discriminator. What
    /// this reads is pictures rather than rows, so what it reports is how
    /// many came out whole.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some((factor, resample)) = dsp::resample::stage(rate_hz, AUDIO_HZ, 4096) else {
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
        let mut resample = resample;
        let mut rx = sstv::Receiver::new(AUDIO_HZ);
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
            if let Some(lines) = rx.push(feed)
                && lines.complete
            {
                pictures += 1;
            }
        }
        Reading { pictures, ..Reading::default() }
    }
}

/// The rate the picture is read at.
///
/// Not a free choice: the analysis windows are counted in samples, so the
/// same recording decoded at 22.05 kHz differs from its 44.1 kHz decode by a
/// mean of 4.5 counts a channel, and at 11.025 kHz by 6.6.
pub const AUDIO_HZ: f64 = 44_100.0;

/// A 2 m FM channel.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// The two metre calling frequency, which is where SSTV lives across Europe.
/// The shortwave calling frequencies are 14.230 and 7.171, and those want a
/// sideband demodulator in front rather than this node's own.
pub const DEFAULT_HZ: f64 = 144_500_000.0;

/// Deviation mapped to full scale on the discriminator. Only the tone scale
/// depends on it, and the decoder reads frequencies rather than amplitudes,
/// so this need only be in the right region.
pub const DEVIATION_HZ: f64 = 3_000.0;
