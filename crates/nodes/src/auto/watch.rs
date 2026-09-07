//! Watching the band: the detector and extractor, what they are kept out
//! of, and the span-wide decoders placed where the span reaches them.

use common::Result;
use dsp::{SourceDetector, SourceExtractor};
use pipeline::port::StreamSpec;

use super::{AutoNode, Member};
use crate::protocol::{self, Placed, Stickiness};

impl AutoNode {
    /// Limit detection to a band inside the input, or `None` for all of it.
    pub fn set_band(&mut self, band: Option<(f64, f64)>) {
        self.band = band;
        self.apply_band();
    }

    /// The tuner's own centre, where a source may open only when nothing
    /// else is transmitting.
    ///
    /// A direct-conversion receiver's DC offset is not steady: a strong
    /// signal anywhere in the span modulates it with its own envelope, and
    /// the DC block passes that as readily as any other keying. Read as a
    /// source it was an unknown 10 kHz wide, exactly as long as the sensor
    /// burst beside it, for every packet that sensor sent. It never happens
    /// alone, so a source at the centre is refused only while another is
    /// open elsewhere, and a device that really sits on the centre still
    /// opens when it transmits by itself.
    pub fn set_spur(&mut self, hz: Option<f64>) {
        self.spur = hz;
        self.apply_band();
    }

    pub fn band(&self) -> Option<(f64, f64)> {
        self.band
    }

    /// The channel plan on this band: a frequency the plan lands on and the
    /// spacing, in hertz. A source found close to a channel is locked to it;
    /// see [`snap_to_raster`].
    pub fn set_raster(&mut self, raster: Option<(f64, f64)>) {
        self.raster = raster.filter(|(_, step)| *step > 0.0);
    }

    pub fn raster(&self) -> Option<(f64, f64)> {
        self.raster
    }

    pub(super) fn apply_band(&mut self) {
        if let (Some(d), Some((lo, hi))) = (self.detector.as_mut(), self.band) {
            let c = self.center.as_f64();
            d.set_band(lo - c, hi - c);
        }
        // Three bins either side, tested against the source's centre.
        self.spur_band = match (self.detector.as_ref(), self.spur) {
            (Some(d), Some(hz)) => Some((hz - 3.0 * d.bin_hz(), hz + 3.0 * d.bin_hz())),
            _ => None,
        };
        // And the floor cap left off there: the residual DC is a permanent
        // hump the cap would otherwise unhide, and reported it is an unknown
        // at the centre of every span for as long as the receiver runs.
        if let (Some(d), Some((lo, hi))) = (self.detector.as_mut(), self.spur_band) {
            let c = self.center.as_f64();
            d.exempt_from_cap(lo - c, hi - c);
        }
    }

    pub(super) fn rebuild(&mut self) -> Result<()> {
        if self.rate <= 0.0 {
            return Ok(());
        }
        let d = SourceDetector::new(self.rate, self.input_bw, self.cfg);
        let keep = d.latency_samples();
        self.extractor = Some(SourceExtractor::new(
            self.rate,
            self.center.as_f64(),
            keep,
            self.cfg,
        ));
        self.detector = Some(d);
        self.slots.clear();
        self.pending_sticky = self.sticky.iter().map(|s| s.id).collect();

        // The span-wide decoders, where the span reaches what they are for.
        // Each one is asked where it belongs and what it owns; nothing here
        // knows which protocols those are.
        let mut spec = StreamSpec::iq(self.rate, self.center);
        spec.bandwidth = self.input_bw;
        let c = self.center.as_f64();
        let half = self.input_bw / 2.0;
        self.wide.clear();
        let covers = |lo: f64, hi: f64| c - half <= lo && hi <= c + half;
        for p in protocol::all() {
            let shape = p.shape();
            if !shape.span_wide || self.rate < shape.min_rate_hz {
                continue;
            }
            for (lo, hi) in p.placement().bands(shape.widths[0]) {
                if !covers(lo, hi) {
                    continue;
                }
                let at = Placed {
                    center_hz: (lo + hi) / 2.0,
                    width_hz: hi - lo,
                    rate: self.rate,
                    snr_db: f32::NAN,
                };
                let mut m = Member::place(*p, spec, at, &Default::default(), &self.reg)?;
                // One that latches owns its band from the moment the span
                // reaches it; one that claims owns nothing until it says so.
                if matches!(p.stickiness(), Stickiness::Latch { .. }) {
                    m.band = Some((lo, hi));
                }
                self.wide.push(m);
            }
        }
        self.apply_band();
        self.apply_locked();

        let nominal = StreamSpec::iq(self.cfg.min_rate_hz, self.center);
        self.template = Some(crate::ism_decode_graph(nominal)?);
        Ok(())
    }
}

/// Lock a source onto the channel plan when it is plainly on it.
///
/// A source is a measurement: the power centroid of the bins that stood over
/// the floor in its first frames, and the run of them with a margin. On a
/// band with a plan that is the wrong answer to a right question. A TETRA
/// carrier found at 391.1812 MHz, 24.6 kHz wide, is the 391.175 MHz channel
/// seen through a tuner a few parts per million out, and every opening
/// would otherwise measure it slightly differently, cut it out at a
/// different width, and log it at a frequency nobody's plan lists.
///
/// So: within 0.4 of a step of a channel, and between 0.4 and 1.6 of a
/// step wide, a source is the channel, and takes its centre and its width.
/// Anything else is left as measured; a plan says where channels are, not
/// that nothing else transmits. The reach is what a tuner tens of parts per
/// million out needs at UHF: a third of a step left one carrier 8.6 kHz off
/// its channel unlocked while its neighbour 6 kHz off locked.
pub(super) fn snap_to_raster(
    s: &mut dsp::Source,
    (origin, step): (f64, f64),
    stream_center_hz: f64,
) {
    let hz = stream_center_hz + s.center_hz;
    let on = origin + ((hz - origin) / step).round() * step;
    let near = (hz - on).abs() <= step * 0.4;
    let width = s.bandwidth_hz();
    let fits = width >= step * 0.4 && width <= step * 1.6;
    if near && fits {
        s.center_hz = on - stream_center_hz;
        s.lo_hz = s.center_hz - step / 2.0;
        s.hi_hz = s.center_hz + step / 2.0;
    }
}
