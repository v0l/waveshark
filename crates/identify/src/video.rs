//! Where Video can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use dsp::FmDemod;
use dsp::video::SyncSeparator;

/// Analogue video as the auto node knows it: on the span, where the
/// channel plan reaches, and owning the band only once it has a picture.
///
/// A camera's carrier is not a channel a detector can cut out: FM video at
/// 5.8 GHz occupies the best part of twenty megahertz, and what a detector
/// measures is the few megahertz around the carrier that stand above the
/// floor. Cut to that, the picture is gone. And claiming the span before
/// there is a picture would turn the band off for everything else on the
/// chance a camera turns up.
pub struct Video;

impl Signal for Video {
    fn id(&self) -> &'static str {
        "video"
    }

    fn label(&self) -> &'static str {
        "video"
    }

    fn placement(&self) -> Placement {
        Placement::Channels(
            decode::video_channels::channels().iter().map(|ch| ch.hz as f64).collect(),
        )
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[2.0 * CHANNEL_HALF_HZ],
            // PAL luma reaches 5 MHz with the colour subcarrier at 4.43, so
            // a slower stream cannot be carrying a picture.
            min_rate_hz: 12e6,
            feed_rate_hz: WORK_RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }

    /// An analogue camera's carrier, read off the span it occupies. A field
    /// is what it produces, so fields are what it counts.
    ///
    /// The standard is not forced here the way an operator can force it on a
    /// channel: `dsp::video::find_lines` is given two fields of baseband and
    /// says which standard the line rate is, and a recording that is not a
    /// camera gets no lock and no fields.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz < self.shape().min_rate_hz {
            return Reading::default();
        }
        let factor = decimation(rate_hz);
        let rate = rate_hz / factor as f64;
        let mut narrow =
            (factor > 1).then(|| dsp::FirDecim::design_hz(rate_hz, factor, rate * 0.4, 60.0));
        let mut demod = FmDemod::new(rate, DEVIATION_HZ);
        let mut sep: Option<SyncSeparator> = None;
        let (mut narrowed, mut base, mut priming, mut fields) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        let mut count = 0usize;
        for b in iq.chunks(crate::BLOCK) {
            base.clear();
            match narrow.as_mut() {
                Some(f) => {
                    narrowed.clear();
                    f.process(b, &mut narrowed);
                    demod.process(&narrowed, &mut base);
                }
                None => demod.process(b, &mut base),
            }
            if sep.is_none() {
                // Two fields' worth before deciding, so the test has hundreds
                // of lines to judge rather than a handful.
                priming.extend_from_slice(&base);
                if priming.len() as f64 <= 0.04 * rate {
                    continue;
                }
                let held = std::mem::take(&mut priming);
                let Some(lock) = dsp::video::find_lines(&held, rate) else {
                    continue;
                };
                sep = Some(SyncSeparator::new(rate, lock.standard, WIDTH));
                // The samples that decided it are still video.
                base.splice(0..0, held);
            }
            let Some(s) = sep.as_mut() else { continue };
            fields.clear();
            s.process(&base, &mut fields);
            count += fields.len();
        }
        let _ = center_hz;
        Reading { pictures: count, ..Reading::default() }
    }
}

/// Half of what a channel of the plan occupies.
pub const CHANNEL_HALF_HZ: f64 = 9e6;

/// What the front end would rather read, in samples per second.
///
/// Enough for the whole FM signal (4.6 MHz measured on the AKK capture) and
/// for the 4.43 MHz colour subcarrier in the baseband that comes out of it,
/// and no more: the noise a discriminator sees is the bandwidth it is
/// handed. A line is then 640 samples, which is exactly the width a field is
/// resampled to.
pub const WORK_RATE_HZ: f64 = 10e6;

/// Peak deviation mapped to full scale. Only the contrast depends on it, and
/// the separator normalises again from the sync tip, so it need not be exact:
/// the 5.8 GHz transmitter measured here was about 1 MHz rms.
pub const DEVIATION_HZ: f64 = 6e6;

/// What the picture is resampled to. A PAL line holds about 720 samples at
/// broadcast rates and a small camera rather fewer, so this is a choice rather
/// than a measurement.
pub const WIDTH: usize = 640;

/// How much to divide a span by to reach [`WORK_RATE_HZ`] without going
/// under it. A 20 MS/s span stays whole, since halving it would leave 10.
pub fn decimation(rate: f64) -> usize {
    let mut f = 1usize;
    while rate / (f * 2) as f64 >= WORK_RATE_HZ {
        f *= 2;
    }
    f
}
