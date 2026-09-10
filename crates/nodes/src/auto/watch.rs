//! Watching the band: the detector and extractor, what they are kept out
//! of, and the span-wide decoders placed where the span reaches them.

use common::{Result, SourceBlock, SourceId, C32};
use dsp::{Owned, SourceConfig, SourceDetector, SourceEvent, SourceExtractor};
use pipeline::port::StreamSpec;

use super::{AutoNode, Member};
use crate::protocol::{self, Placed, Stickiness};

/// The pair that finds what is on the span and cuts it out, and everything
/// that decides where they may look.
///
/// One owner, because the two are one instrument: the extractor's ring has to
/// be long enough for the detector's latency, every channel the detector is
/// kept out of is a channel something else is cutting out, and neither exists
/// until the graph has said what the span is.
#[derive(Default)]
pub(super) struct Watch {
    detector: Option<SourceDetector>,
    extractor: Option<SourceExtractor>,
    /// The band detection is limited to inside the input, in absolute hertz,
    /// or None for all of it.
    band: Option<(f64, f64)>,
    /// The tuner's own centre, and the band around it once the resolution is
    /// known. See [`AutoNode::set_spur`].
    spur: Option<f64>,
    spur_band: Option<(f64, f64)>,
    /// The channel plan on this band, as an origin and a step in hertz, when
    /// there is one. See [`snap_to_raster`].
    raster: Option<(f64, f64)>,
}

impl Watch {
    /// Whether there is anything to watch with yet: both are built when the
    /// graph negotiates, and until then the node has not been told the span.
    pub(super) fn ready(&self) -> bool {
        self.detector.is_some() && self.extractor.is_some()
    }

    /// Wideband samples consumed so far, which is the clock everything here
    /// is timed on.
    pub(super) fn position(&self) -> u64 {
        self.detector.as_ref().map_or(0, |d| d.position())
    }

    pub(super) fn live(&self) -> Vec<dsp::Source> {
        self.detector
            .as_ref()
            .map(|d| d.live().copied().collect())
            .unwrap_or_default()
    }

    /// Put a detector built with new thresholds in place of the running one,
    /// keeping the extractor and its ring. False before the graph has said
    /// what the span is: there is nothing to replace, and nothing to build
    /// one from either.
    pub(super) fn rebuild_detector(&mut self, build: impl FnOnce() -> SourceDetector) -> bool {
        match self.detector.as_mut() {
            Some(old) => {
                *old = build();
                true
            }
            None => false,
        }
    }

    pub(super) fn set_owned(&mut self, channels: Vec<Owned>) {
        if let Some(d) = self.detector.as_mut() {
            d.set_owned(channels);
        }
    }

    /// What opened and closed in this block, snapped to the channel plan
    /// where there is one.
    ///
    /// `idle` when something else has claimed the whole span: there is
    /// nothing to find in a span that is one transmission, and the block goes
    /// by unread rather than being buffered.
    pub(super) fn admit(&mut self, iq: &[C32], center_hz: f64, idle: bool, out: &mut Vec<SourceEvent>) {
        let Some(d) = self.detector.as_mut() else { return };
        if idle {
            d.idle(iq.len());
            return;
        }
        out.extend_from_slice(d.process(iq));
        // A source plainly on a channel of the plan is that channel: what is
        // cut out, and what is reported, is the channel rather than this
        // frame's measurement of it.
        let Some(raster) = self.raster else { return };
        for ev in out.iter_mut() {
            if let SourceEvent::Opened(s) = ev {
                snap_to_raster(s, raster, center_hz);
            }
        }
    }

    /// Cut every open source out of the block, as a run of samples each.
    pub(super) fn cut(
        &mut self,
        iq: &[C32],
        events: &[SourceEvent],
        blocks: &mut Vec<SourceBlock>,
    ) {
        if let Some(e) = self.extractor.as_mut() {
            e.process(iq, events, blocks);
        }
    }

    /// Cut a channel out that no source opened, from where the stream is now.
    pub(super) fn open_channel(&mut self, id: SourceId, center_hz: f64, width_hz: f64) {
        let from = self.position();
        if let Some(e) = self.extractor.as_mut() {
            e.open_channel(id, center_hz, width_hz, from);
        }
    }

    pub(super) fn close_channel(&mut self, id: SourceId) {
        if let Some(e) = self.extractor.as_mut() {
            e.close(id);
        }
    }

    /// Microseconds the shared bank spent this block, streaming and catching
    /// up, for the cost view.
    pub(super) fn bank_cost(&mut self) -> (u64, u64) {
        self.extractor.as_mut().map_or((0, 0), |e| e.take_bank_cost())
    }

    pub(super) fn reset(&mut self) {
        if let Some(d) = self.detector.as_mut() {
            d.reset();
        }
        if let Some(e) = self.extractor.as_mut() {
            e.reset();
        }
    }
}

impl AutoNode {
    /// The detector's settings for this span.
    ///
    /// The widest source worth opening is the widest channel any protocol
    /// reads, with the margin a strong signal's skirts add; asked of the
    /// registry rather than kept as a number here, which is how 2.4 GHz
    /// LoRa at 812.5 kHz was refused at the door for as long as the number
    /// said 600. Never more than a quarter of the span, though: a strong
    /// narrowband transmitter lights most of a 2 MHz span, and a source
    /// that wide is a saturated receiver rather than a signal, and one
    /// that grows into it supersedes the narrow source that was reading
    /// the transmission. The MeshCore advert at 51 dB in a 2.048 MS/s span
    /// is the capture that says so.
    pub(super) fn detector_cfg(&self) -> SourceConfig {
        let widest = protocol::all()
            .iter()
            .filter(|p| !p.shape().span_wide)
            .flat_map(|p| p.shape().widths.iter().copied())
            .fold(0.0, f64::max);
        let mut cfg = self.cfg;
        let want = (widest * cfg.width_margin * 1.5).min(self.rate / 4.0);
        cfg.max_width_hz = cfg.max_width_hz.max(want);
        cfg
    }

    /// Limit detection to a band inside the input, or `None` for all of it.
    pub fn set_band(&mut self, band: Option<(f64, f64)>) {
        self.watch.band = band;
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
        self.watch.spur = hz;
        self.apply_band();
    }

    pub fn band(&self) -> Option<(f64, f64)> {
        self.watch.band
    }

    /// The channel plan on this band: a frequency the plan lands on and the
    /// spacing, in hertz. A source found close to a channel is locked to it;
    /// see [`snap_to_raster`].
    pub fn set_raster(&mut self, raster: Option<(f64, f64)>) {
        self.watch.raster = raster.filter(|(_, step)| *step > 0.0);
    }

    pub fn raster(&self) -> Option<(f64, f64)> {
        self.watch.raster
    }

    pub(super) fn apply_band(&mut self) {
        let c = self.center.as_f64();
        let w = &mut self.watch;
        if let (Some(d), Some((lo, hi))) = (w.detector.as_mut(), w.band) {
            d.set_band(lo - c, hi - c);
        }
        // Three bins either side, tested against the source's centre.
        w.spur_band = match (w.detector.as_ref(), w.spur) {
            (Some(d), Some(hz)) => Some((hz - 3.0 * d.bin_hz(), hz + 3.0 * d.bin_hz())),
            _ => None,
        };
        if let (Some(d), Some((lo, hi))) = (w.detector.as_mut(), w.spur_band) {
            d.set_spur(lo - c, hi - c);
        }
    }

    pub(super) fn rebuild(&mut self) -> Result<()> {
        if self.rate <= 0.0 {
            return Ok(());
        }
        let d = SourceDetector::new(self.rate, self.input_bw, self.detector_cfg());
        let keep = d.latency_samples();
        self.watch.extractor = Some(SourceExtractor::new(
            self.rate,
            self.center.as_f64(),
            keep,
            self.cfg,
        ));
        self.watch.detector = Some(d);
        self.slots.clear();
        self.memory.cut_again();

        // The span-wide decoders, where the span reaches what they are for.
        // Each one is asked where it belongs and what it owns; nothing here
        // knows which protocols those are.
        let mut spec = StreamSpec::iq(self.rate, self.center);
        spec.bandwidth = self.input_bw;
        let c = self.center.as_f64();
        let half = self.input_bw / 2.0;
        self.wide.clear();
        self.wide_ring = super::Ring::new(spec);
        let covers = |lo: f64, hi: f64| c - half <= lo && hi <= c + half;
        for p in protocol::all() {
            let shape = p.shape();
            if !shape.span_wide || self.rate < shape.min_rate_hz {
                continue;
            }
            // One per span, not one per band the plan happens to name. A
            // span-wide front end reads the span itself, so a second copy of
            // it demodulates the same samples again for the same result: the
            // 5.8 GHz video plan calls 5865 A1 and 5866 B8, both of their
            // 18 MHz windows fit in a 20 MS/s span at 5865, and the receiver
            // ran two full-span FM demodulators and published every field of
            // the picture twice. Where several bands cover, the one nearest
            // the centre is the one the span is really on.
            let mut bands: Vec<(f64, f64)> =
                p.placement().bands(shape.widths[0]).into_iter().filter(|(lo, hi)| covers(*lo, *hi)).collect();
            bands.sort_by(|a, b| {
                let off = |(lo, hi): &(f64, f64)| ((lo + hi) / 2.0 - c).abs();
                off(a).total_cmp(&off(b))
            });
            bands.truncate(1);
            for (lo, hi) in bands {
                // Cut the span down to the rate the protocol asked to be
                // fed, the way the receiver's own extraction does for a
                // front end the scanner table places. A span-wide decoder
                // used to be handed the raw span whatever it declared, so a
                // Mode S correlator that wants 2.4 MS/s ran over 20 and cost
                // 120% of a core on an empty band. What a decoder cannot
                // avoid is the noise it is handed: this is the same saving
                // twice over, in work and in signal to noise.
                let offset = (lo + hi) / 2.0 - c;
                let (factor, rate) = match p.narrow_span() {
                    true => protocol::span_feed(self.rate, offset, &shape),
                    false => (1, self.rate),
                };
                let at = Placed {
                    center_hz: (lo + hi) / 2.0,
                    width_hz: hi - lo,
                    rate,
                    snr_db: f32::NAN,
                    // A front end over the span is not cut out of anything,
                    // so there is no position in a span to tell it.
                    origin: None,
                };
                // Four fifths of the new Nyquist and sixty decibels: a
                // transition band wide enough that the filter stays short,
                // and more rejection than a decoder can tell.
                let pre: Vec<crate::NodeSpec> = match factor {
                    1 => Vec::new(),
                    f => vec![crate::NodeSpec::new("decimate")
                        .i("factor", f as i64)
                        .f("passband", 0.8)
                        .f("atten_db", 60.0)],
                };
                let mut m =
                    Member::place_behind(*p, spec, at, &pre, &Default::default(), &self.reg)?;
                m.placed_band = Some((lo, hi));
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
