//! Where DrmProtocol can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::{C32, Decoded};
use decode::drm;

pub struct DrmProtocol;

impl Signal for DrmProtocol {
    fn id(&self) -> &'static str {
        "drm"
    }

    fn label(&self) -> &'static str {
        "drm"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["digital radio mondiale"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND_HZ])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: RATE_HZ,
            feed_rate_hz: RATE_HZ,
            span_wide: false,
            // A broadcast is on the air without stopping, so there is no
            // burst for the classifier to name.
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The multiplex the recording is tuned to: every service its SDC
    /// names, as a row, once the file has been read through.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some((factor, mut resample)) = dsp::resample::stage(rate_hz, RATE_HZ, 4096) else {
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
        let mut rx = drm::DrmReceiver::new(dsp::drm::Mode::B);
        let (mut narrow, mut at_rate) = (Vec::new(), Vec::new());
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            at_rate.clear();
            match resample.as_mut() {
                Some(r) => {
                    r.process(&narrow, &mut at_rate);
                    rx.push(&at_rate);
                }
                None => {
                    rx.push(&narrow);
                }
            }
        }
        let m = rx.multiplex().clone();
        let rows: Vec<Decoded> = m
            .services
            .iter()
            .filter_map(|s| {
                let label = m.label(s.short_id).map(str::to_string);
                drm::service_decoded(&rx, *s, label, chan.hz(), 0.0)
            })
            .collect();
        rows.into()
    }
}

/// Long wave up to the top of the shortwave broadcast bands, which is
/// everywhere DRM is allocated.
pub const BAND_HZ: (f64, f64) = (148_500.0, 30_000_000.0);

/// What a transmission occupies at its widest.
pub const CHANNEL_WIDTH_HZ: f64 = dsp::drm::CHANNEL_WIDTH_HZ;

/// Where a receiver that has been told nothing else points: the 75 metre
/// broadcast band, which is inside every shortwave front end's range.
pub const DEFAULT_HZ: f64 = 3_965_000.0;

/// The rate the modes are defined at.
pub const RATE_HZ: f64 = dsp::drm::RATE_HZ;
