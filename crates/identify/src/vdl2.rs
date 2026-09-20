//! Where Vdl2 can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::vdl2;
use dsp::d8psk::{D8pskConfig, D8pskDemod};
use dsp::resample::Rational;

pub struct Vdl2;

impl Signal for Vdl2 {
    fn id(&self) -> &'static str {
        "vdl2"
    }

    fn label(&self) -> &'static str {
        "vdl2"
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND_HZ])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 105_000.0,
            feed_rate_hz: 200_000.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The channel the recording is tuned to, resampled to the 105 kHz the
    /// D8PSK demodulator reads.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let want = D8pskConfig::VDL2.rate();
        if rate_hz < want {
            return Reading::default();
        }
        let Some((factor, resample)) = dsp::resample::stage(rate_hz, want, 4096) else {
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
        let mut resample = resample.unwrap_or_else(|| Rational::with_ratio(1, 1));
        let mut demod = D8pskDemod::new(D8pskConfig::VDL2);
        let (mut narrow, mut at_rate, mut bursts) = (Vec::new(), Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            at_rate.clear();
            resample.process(&narrow, &mut at_rate);
            bursts.clear();
            // The demodulator cannot know how long a burst is: the length is
            // in the header, which is in the burst.
            let mut done = |bits: &[bool]| vdl2::wanted_bits(bits).is_some_and(|n| bits.len() >= n);
            demod.process(&at_rate, &mut done, &mut bursts);
            for burst in &bursts {
                for bytes in vdl2::frame_bytes(&burst.bits) {
                    if let Some(f) = vdl2::parse_frame(&bytes) {
                        rows.push(vdl2::read(&f));
                    }
                }
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// Where VDL Mode 2 is allocated, which is wider than the European group.
///
/// 136.100 is an ARINC channel in North America and sits nearly a megahertz
/// below the rest, so a band that started at 136.65 was a band that could not
/// decode it however it was tuned.
pub const BAND_HZ: (f64, f64) = (136_050_000.0, 137_000_000.0);

/// An airband channel on the 25 kHz grid.
pub const CHANNEL_WIDTH_HZ: f64 = 25_000.0;

/// The busiest of the European VDL2 channels and the one every ground station
/// carries: the common signalling channel.
pub const DEFAULT_HZ: f64 = 136_975_000.0;
