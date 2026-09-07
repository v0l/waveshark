//! Which decoders a source gets, and when.

use common::{Hz, Packet, Result, SourceBlock, SourceId, C32};
use pipeline::event::Event;
use pipeline::port::StreamSpec;

use super::{AutoNode, Member};
use crate::protocol::{self, Placed, Protocol};
use crate::NodeSpec;

/// One open source and the decoders reading it.
pub(super) struct Slot {
    pub(super) id: SourceId,
    pub(super) center_hz: u64,
    pub(super) members: Vec<Member>,
    /// A front end has read something from this source. From then on the
    /// burst front end's measurement of it is not news: a row saying what
    /// the carrier looks like beside rows saying what it said.
    pub(super) heard: bool,
    /// The stream the members were built for, and the width the detector
    /// measured, for a front end placed after the source opened.
    pub(super) spec: StreamSpec,
    pub(super) signal_hz: f64,
    /// Protocols that wait for the classifier's verdict and have had it
    /// for this source: placed, or ruled out.
    pub(super) tried: Vec<&'static str>,
    /// How many verdicts had been considered when they were last asked.
    pub(super) verdicts_seen: usize,
    /// A channel remembered from earlier, which runs the one decoder that
    /// earned it and nothing else.
    pub(super) remembered: bool,
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
        if let Some(st) = self.sticky.iter().find(|s| s.id == b.id) {
            let p = protocol::by_id(st.name)
                .ok_or_else(|| common::Error::other(format!("no protocol {:?}", st.name)))?;
            let at = Placed {
                center_hz: st.center_hz,
                width_hz: st.width_hz,
                rate: b.rate,
                snr_db: b.snr_db,
            };
            let m = Member::place(p, spec, at, &self.reg)?;
            return Ok(Slot {
                id: b.id,
                center_hz: b.center_hz,
                members: vec![m],
                heard: true,
                spec,
                signal_hz: b.signal_hz,
                tried: Vec::new(),
                verdicts_seen: 0,
                remembered: true,
            });
        }
        // The front end is told how strong the detector found the source,
        // so a stream that begins inside a transmission is not read as
        // noise from its first sample to its last.
        let route = NodeSpec::new("burst_route").f("source_snr_db", b.snr_db as f64);
        let mut classifier = Member::classifier(spec, route, &self.reg)?;
        classifier.source_snr_db = b.snr_db;
        let mut members = vec![classifier];
        let hz = b.center_hz as f64;
        for p in protocol::all() {
            let shape = p.shape();
            if shape.span_wide || !shape.families.is_empty() {
                continue;
            }
            if !candidate(*p, hz, b.bandwidth_hz, b.rate) {
                continue;
            }
            for w in p.widths_for(b.bandwidth_hz) {
                let at = Placed {
                    center_hz: hz,
                    width_hz: w,
                    rate: b.rate,
                    snr_db: b.snr_db,
                };
                if let Ok(m) = Member::place(*p, spec, at, &self.reg) {
                    members.push(m);
                }
            }
        }
        Ok(Slot {
            id: b.id,
            center_hz: b.center_hz,
            members,
            heard: false,
            spec,
            signal_hz: b.signal_hz,
            tried: Vec::new(),
            verdicts_seen: 0,
            remembered: false,
        })
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
    /// the fact, and a long one is caught up and then followed live.
    pub(super) fn place_on_verdict(
        &mut self,
        k: usize,
        at_us: u64,
        closed: bool,
        ev: &mut Vec<Event>,
        pk: &mut Vec<Packet>,
        heard: &mut Vec<(&'static str, f64)>,
    ) {
        let reg = &self.reg;
        let slot = &mut self.slots[k];
        let Some(router) = slot.members.iter().find(|m| m.router.is_some()) else {
            return;
        };
        let verdicts = router.verdicts.clone();
        slot.verdicts_seen = verdicts.len();
        let hz = slot.center_hz as f64;
        let snr = slot.members.first().map_or(f32::NAN, |m| m.source_snr_db);
        let mut history: Option<Vec<C32>> = None;
        for p in protocol::all() {
            let shape = p.shape();
            if shape.span_wide || slot.tried.contains(&p.id()) {
                continue;
            }
            if !shape.families.iter().any(|f| verdicts.contains(f)) {
                continue;
            }
            slot.tried.push(p.id());
            if !candidate(*p, hz, slot.signal_hz, slot.spec.rate) {
                continue;
            }
            let history = history.get_or_insert_with(|| {
                slot.members
                    .iter()
                    .find(|m| m.router.is_some())
                    .map(|m| m.ring.clone())
                    .unwrap_or_default()
            });
            for w in p.widths_for(slot.signal_hz) {
                let at = Placed {
                    center_hz: hz,
                    width_hz: w,
                    rate: slot.spec.rate,
                    snr_db: snr,
                };
                let Ok(mut m) = Member::place(*p, slot.spec, at, reg) else {
                    continue;
                };
                let before = pk.len();
                // The samples the source has produced so far, in the blocks
                // the live path would have handed over, then the flush if
                // it has already closed.
                for chunk in history.chunks(16_384) {
                    ev.extend(m.run(chunk, at_us, pk));
                }
                if closed {
                    let quiet = vec![C32::new(0.0, 0.0); (m.flush_s * slot.spec.rate) as usize];
                    ev.extend(m.run(&quiet, at_us, pk));
                }
                if pk.len() > before {
                    slot.heard = true;
                    heard.push((m.name, m.channel_hz));
                }
                slot.members.push(m);
            }
        }
    }
}

/// Whether a source at `hz`, measured `width_hz` wide and cut out at
/// `rate`, could be a channel of this protocol.
pub(super) fn candidate(p: &dyn Protocol, hz: f64, width_hz: f64, rate: f64) -> bool {
    let shape = p.shape();
    rate >= shape.min_rate_hz
        && p.placement().covers(hz, shape.widths[0])
        && p.accepts_width(width_hz)
}
