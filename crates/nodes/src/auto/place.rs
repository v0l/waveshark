//! Which decoders a source gets, and when.

use common::{Hz, Result, SourceBlock, SourceId, C32};
use pipeline::port::StreamSpec;

use super::evidence::Evidence;
use super::member::Ring;
use super::{AutoNode, Member};
use crate::protocol::{self, Origin, Placed, Protocol};
use crate::NodeSpec;

/// One open source and the decoders reading it.
pub(super) struct Slot {
    pub(super) id: SourceId,
    /// Where the stream was cut from, which is what a packet off it is
    /// labelled with and what a remembered channel is matched against.
    pub(super) center_hz: Hz,
    pub(super) members: Vec<Member>,
    /// A front end has read something from this source. From then on the
    /// burst front end's measurement of it is not news: a row saying what
    /// the carrier looks like beside rows saying what it said.
    pub(super) heard: bool,
    /// The stream the members were built for, and the width the detector
    /// measured, for a front end placed after the source opened.
    pub(super) spec: StreamSpec,
    pub(super) signal_hz: f64,
    /// Protocol and channel width that waited for the classifier's verdict
    /// and have had it for this source: placed, or ruled out. Keyed by
    /// width as well as protocol, so a second network on the same channel
    /// at another width is still placed.
    pub(super) tried: Vec<(&'static str, u64)>,
    /// How many verdicts had been considered when they were last asked.
    pub(super) verdicts_seen: usize,
    /// A channel remembered from earlier, which runs the one decoder that
    /// earned it and nothing else.
    pub(super) remembered: bool,
    /// Where the stream sits in the span, as a decoder placed on it that has
    /// to be timed from another stream is told.
    pub(super) origin: Origin,
    /// The stream itself, kept once for every front end reading it.
    pub(super) ring: Ring,
    /// The detector's own measurement of this source, for a source too wide
    /// for the burst router, which is then the only evidence there is that
    /// anything transmitted. `None` where the router is reading it.
    pub(super) evidence: Option<Evidence>,
}

impl AutoNode {
    /// The decoders a source of this shape gets.
    ///
    /// The burst front end always. Then every protocol whose placement
    /// covers the frequency, whose declared channel the source could be
    /// (the stream must carry it, and the measured width must be within
    /// reach of it, so a fat or splattered measurement does not put a
    /// 12.5 kHz decoder on a 200 kHz signal), and which does not wait for
    /// the classifier's verdict; each decides for itself whether the bits
    /// are its own. A decoder that will not build is left out rather than
    /// fatal: the source still has the front end, and one decoder's refusal
    /// is not a reason to stop the receiver.
    pub(super) fn open(&self, b: &SourceBlock) -> Result<Slot> {
        let mut spec = StreamSpec::iq(b.rate, Hz(b.center_hz));
        spec.bandwidth = b.bandwidth_hz.min(b.rate);
        // Where this stream sits in the span, so a decoder that has to be
        // timed from another carrier's decoder can say where in the span
        // its timing was measured, and the other can find that in its own
        // samples.
        let origin = Origin {
            span_sample: b.start_sample,
            span_rate_hz: self.rate,
        };
        if let Some(st) = self.memory.find(b.id) {
            let p = protocol::by_id(st.name)
                .ok_or_else(|| common::Error::other(format!("no protocol {:?}", st.name)))?;
            let at = Placed {
                center_hz: st.center_hz,
                width_hz: st.width_hz,
                rate: b.rate,
                snr_db: b.snr_db,
                origin: Some(origin),
            };
            let m = Member::place(p, spec, at, &st.settings, &self.reg)?;
            // The classifier rides along on a remembered channel, so a
            // second transmitter that shares the frequency is named rather
            // than fed to a demodulator that cannot read it: two LoRa
            // networks at different spreading factors and bandwidths do
            // exactly this. It is affordable because the router measures
            // each burst shape once and skips the repeats.
            let mut members = vec![m];
            if routable(b) {
                members.extend(self.classifier(b, spec).ok());
            }
            return Ok(Slot {
                id: b.id,
                center_hz: Hz(b.center_hz),
                members,
                heard: true,
                spec,
                signal_hz: b.signal_hz,
                tried: Vec::new(),
                verdicts_seen: 0,
                remembered: true,
                origin,
                ring: Ring::new(spec),
                evidence: None,
            });
        }
        let mut members = Vec::new();
        let mut evidence = None;
        if routable(b) {
            members.push(self.classifier(b, spec)?);
        } else {
            evidence = Some(
                Evidence::new(b.center_hz, b.signal_hz, b.snr_db, b.rate)
                    .from_sample(b.start_sample),
            );
        }
        let hz = b.center_hz as f64;
        for p in protocol::all() {
            let shape = p.shape();
            if shape.span_wide || !shape.families.is_empty() {
                continue;
            }
            if !candidate(*p, hz, b.bandwidth_hz, b.rate) {
                continue;
            }
            for w in p.widths_for(hz, b.bandwidth_hz) {
                let at = Placed {
                    center_hz: hz,
                    width_hz: w,
                    rate: b.rate,
                    snr_db: b.snr_db,
                    origin: Some(origin),
                };
                if let Ok(m) = Member::place(*p, spec, at, &Default::default(), &self.reg) {
                    members.push(m);
                }
            }
        }
        Ok(Slot {
            id: b.id,
            center_hz: Hz(b.center_hz),
            members,
            heard: false,
            spec,
            signal_hz: b.signal_hz,
            tried: Vec::new(),
            verdicts_seen: 0,
            remembered: false,
            origin,
            ring: Ring::new(spec),
            evidence,
        })
    }

    /// The burst front end for a source, told how strong the detector found
    /// it: a stream that begins inside a transmission is otherwise read as
    /// noise from its first sample to its last.
    fn classifier(&self, b: &SourceBlock, spec: StreamSpec) -> Result<Member> {
        let route = NodeSpec::new("burst_route").f("source_snr_db", b.snr_db as f64);
        let mut m = Member::classifier(spec, route, &self.reg)?;
        m.source_snr_db = b.snr_db;
        Ok(m)
    }

    /// Place the decoders that wait for the classifier's verdict, once it
    /// has named a burst of this source, and read them the source's samples
    /// so far from the ring the burst front end kept.
    ///
    /// Placed on the verdict and not on the width because a decoder that
    /// waits is one too dear to run on every source that measures the right
    /// width: LoRa dechirped six spreading factors on every source over
    /// 44 kHz, and on a band of hard-keyed sensors most sources measure that
    /// wide from their splatter. The verdict costs nothing extra: the burst
    /// front end classifies every burst anyway. What it costs is latency,
    /// since a burst is named when it ends or half a second in, and the ring
    /// is what pays that back: a short packet is read whole from it after
    /// the fact, and a long one is caught up and then followed live. The
    /// catching up is spread over the blocks that follow rather than done
    /// here; see [`Member::catch_up`].
    pub(super) fn place_on_verdict(&mut self, k: usize, closed: bool) {
        let reg = &self.reg;
        let slot = &mut self.slots[k];
        let Some(router) = slot.members.iter().find(|m| m.router.is_some()) else {
            return;
        };
        let verdicts = router.verdicts.clone();
        slot.verdicts_seen = verdicts.len();
        let hz = slot.center_hz.as_f64();
        let snr = slot.members.first().map_or(f32::NAN, |m| m.source_snr_db);
        for p in protocol::all() {
            let shape = p.shape();
            if shape.span_wide || slot.tried.contains(&(p.id(), 0)) {
                continue;
            }
            if !shape.families.iter().any(|f| verdicts.iter().any(|(m, _)| m == f)) {
                continue;
            }
            let width = shape
                .families
                .iter()
                .filter_map(|f| verdicts.iter().find(|(m, _)| m == f).map(|(_, w)| *w))
                .fold(0.0f64, f64::max)
                .max(0.0);
            // What the classifier measured the burst at, where it measured
            // anything, and otherwise what the detector measured the source
            // at. On a remembered channel the source's width is the channel
            // a front end already owns, which is the wrong answer for a
            // narrower signal sharing it.
            let width = if width > 0.0 { width } else { slot.signal_hz };
            if !candidate(*p, hz, width, slot.spec.rate) {
                slot.tried.push((p.id(), 0));
                continue;
            }
            for w in p.widths_for(hz, width) {
                if slot.tried.contains(&(p.id(), w as u64)) {
                    continue;
                }
                slot.tried.push((p.id(), w as u64));
                let at = Placed {
                    center_hz: hz,
                    width_hz: w,
                    rate: slot.spec.rate,
                    snr_db: snr,
                    // Not where the source began: this decoder starts on the
                    // history the stream kept, which is where its own first
                    // sample is.
                    origin: Some(slot.origin.advanced(slot.ring.base(), slot.spec.rate)),
                };
                let Ok(mut m) = Member::place(*p, slot.spec, at, &Default::default(), reg) else {
                    continue;
                };
                // The samples the source has produced so far, then the
                // flush if it has already closed.
                m.catch_up_from(&slot.ring);
                if closed {
                    let quiet = vec![C32::new(0.0, 0.0); (m.flush_s * slot.spec.rate) as usize];
                    m.catch_up(&quiet);
                }
                slot.members.push(m);
            }
        }
    }
}

/// Whether the burst router belongs on this source at all.
///
/// It is a decoder with a width like any other, and its width is what its
/// consumers can read; see [`protocol::router_max_width_hz`]. A source wider
/// than that gets [`Evidence`] instead.
pub(super) fn routable(b: &SourceBlock) -> bool {
    b.bandwidth_hz <= protocol::router_max_width_hz()
}

/// Whether a source at `hz`, measured `width_hz` wide and cut out at
/// `rate`, could be a channel of this protocol.
pub(super) fn candidate(p: &dyn Protocol, hz: f64, width_hz: f64, rate: f64) -> bool {
    let shape = p.shape();
    rate >= shape.min_rate_hz
        && p.placement().covers(hz, shape.widths[0])
        && p.accepts_width(hz, width_hz)
}
