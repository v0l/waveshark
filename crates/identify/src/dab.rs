//! Where DabProtocol can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::dab;

pub struct DabProtocol;

impl Signal for DabProtocol {
    fn id(&self) -> &'static str {
        "dab"
    }

    fn label(&self) -> &'static str {
        "dab"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["dab+"]
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
            // An ensemble is on the air without stopping, so there is no
            // burst for the classifier to name and nothing to wait for.
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The multiplex the recording is tuned to: the ensemble and every
    /// service its tables name, as rows, once the file has been read
    /// through.
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
        let mut rx = dab::DabReceiver::new(dsp::dab::Mode::I);
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
        let mut rows = Vec::new();
        if let Some(d) = dab::ensemble_read(&rx) {
            rows.push(d);
        }
        let ids: Vec<u32> = rx.ensemble().stations().map(|s| s.id).collect();
        for id in ids {
            if let Some(d) = dab::service_read(&rx, id) {
                rows.push(d);
            }
        }
        Reading::from(rows).at(chan.hz().as_f64())
    }
}

/// Band III as Europe allocates it to DAB: blocks 5A at 174.928 MHz to 13F
/// at 239.200 MHz.
pub const BAND_HZ: (f64, f64) = (174_000_000.0, 240_000_000.0);

/// What an ensemble occupies.
pub const CHANNEL_WIDTH_HZ: f64 = dsp::dab::CHANNEL_WIDTH_HZ;

/// Block 11D, which carries the British national ensemble and is as good a
/// place as any to point a receiver that has been told nothing else.
pub const DEFAULT_HZ: f64 = 222_064_000.0;

/// The rate every transmission mode is defined at.
pub const RATE_HZ: f64 = dsp::dab::RATE_HZ;
