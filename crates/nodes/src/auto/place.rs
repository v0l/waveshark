//! Which decoders a source gets, when, and what they make of a block.

use common::{Hz, Packet, PacketBody, Result, SourceBlock, SourceId, SourceState, C32};
use pipeline::event::Event;
use pipeline::port::StreamSpec;
use rayon::prelude::*;
use std::time::Instant;

use super::evidence::Evidence;
use super::locks::{Claimed, LockId};
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
    /// The lock that claimed this source, where one did, so what the front
    /// end it was handed to made of it is scored against that lock.
    pub(super) locked: Option<LockId>,
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

/// What one front end on one source made of a block.
struct MemberResult {
    name: &'static str,
    /// Each event with the front end that produced it, since they are merged
    /// with the other members' before anything is answered.
    events: Vec<(&'static str, Event)>,
    packets: Vec<Packet>,
    /// The channel width this front end was placed for, when it read
    /// something. The classifier measuring a burst is not reading it.
    read: Option<f64>,
    spent_us: u64,
}

/// What the front ends on one source made of a block.
pub(super) struct SlotResult {
    /// Which slot, since the fanout returns them in whatever order they
    /// finished.
    pub(super) k: usize,
    /// Each event with the front end that produced it, so a request is
    /// answered to the one that asked rather than to a name a node inside it
    /// wrote about itself.
    pub(super) events: Vec<(&'static str, Event)>,
    pub(super) packets: Vec<Packet>,
    /// The source has closed and nothing is still catching up on it.
    pub(super) done: bool,
    /// The front ends that read something, and the channel width each was
    /// placed for.
    pub(super) heard: Vec<(&'static str, f64)>,
    /// Processor time per front end, for the cost view.
    pub(super) spent: Vec<(&'static str, u64)>,
}

impl Slot {
    /// Run every front end on this source over one block of it, and say what
    /// they made of it.
    ///
    /// `b` is the block the extractor cut for this source, or `None` where
    /// the source has closed: its decoders run on, on nothing, only while
    /// one of them is still reading history.
    ///
    /// One task per front end and not one per source: the members share
    /// nothing but the block they read, and per-source tasks left an M17
    /// member decoding voice alone on one lane while the others sat
    /// finished.
    pub(super) fn run_block(
        &mut self,
        k: usize,
        b: Option<&SourceBlock>,
        at_us: u64,
    ) -> Option<SlotResult> {
        if b.is_none() && !self.members.iter().any(|m| m.behind()) {
            return None;
        }
        let (samples, rate, state) = match b {
            Some(b) => (&b.samples[..], b.rate, b.state),
            None => (&[][..], self.spec.rate, SourceState::Closed),
        };
        // The flush is fed once, on the block that closed the source, not on
        // every block after it.
        let closed = b.is_some() && state == SourceState::Closed;
        // The samples are kept for the evidence row too: a row that cannot
        // say what it was read from is half a row, whether a classifier or
        // the detector measured it.
        self.ring.keeps = self.evidence.is_some() || self.members.iter().any(|m| m.keeps_samples);
        self.ring.push(samples);
        let ring = &self.ring;
        let per: Vec<MemberResult> = self
            .members
            .par_iter_mut()
            .map(|m| {
                let mut pk = Vec::new();
                let t = Instant::now();
                let mut ev = m.run(samples, at_us, &mut pk, ring);
                if closed {
                    let quiet = vec![C32::new(0.0, 0.0); (m.flush_s * rate) as usize];
                    ev.extend(m.run(&quiet, at_us, &mut pk, ring));
                }
                let us = t.elapsed().as_micros() as u64;
                let read = m.router.is_none() && !pk.is_empty();
                // Which front end spoke, taken from the one that was run
                // rather than from a name a node inside it wrote about
                // itself: a request routed by that is routed by a spelling.
                MemberResult {
                    name: m.name,
                    events: ev.into_iter().map(|e| (m.name, e)).collect(),
                    packets: pk,
                    read: read.then_some(m.channel_hz),
                    spent_us: us,
                }
            })
            .collect();
        let mut events = Vec::new();
        let mut packets = Vec::new();
        let mut heard = Vec::new();
        let mut spent = Vec::new();
        // What the detector measured, where nothing else measured anything:
        // the row an unknown wideband signal leaves.
        if let Some(e) = self.evidence.as_mut() {
            e.push(samples);
            let ended = matches!(state, SourceState::Closed | SourceState::Superseded);
            packets.extend(e.row(at_us, ended, &self.ring));
        }
        for r in per {
            events.extend(r.events);
            packets.extend(r.packets);
            spent.push((r.name, r.spent_us));
            if let Some(width) = r.read {
                self.heard = true;
                heard.push((r.name, width));
            }
        }
        // A measurement of a source a front end reads is not news.
        if self.heard {
            packets.retain(|p| {
                !(p.measure.is_some() && matches!(&p.body, PacketBody::Pulses(v) if v.is_empty()))
            });
        }
        // Done once the source has closed and nothing is still catching up
        // on it.
        let done = matches!(state, SourceState::Closed | SourceState::Superseded)
            && !self.members.iter().any(|m| m.behind());
        if state == SourceState::Superseded {
            // A wider stream for the same transmitter takes over from its
            // start. Whatever this one made of the sliver it had is half a
            // burst, and half a burst is not evidence.
            packets.clear();
            events.retain(|(_, e)| !matches!(e, Event::Decoded(_)));
        }
        Some(SlotResult { k, events, packets, done, heard, spent })
    }
}

impl AutoNode {
    /// The slot a source gets: where its stream sits in the span, and the
    /// decoders on it.
    ///
    /// A channel this node remembered runs the one front end that earned it;
    /// anything else gets what [`found`] says a source of that shape gets.
    pub(super) fn open(&self, b: &SourceBlock, claimed: Option<Claimed>) -> Result<Slot> {
        let mut spec = StreamSpec::iq(b.rate, Hz(b.center_hz));
        spec.bandwidth = b.bandwidth_hz.min(b.rate);
        // Where this stream sits in the span, so a decoder that has to be
        // timed from another carrier's decoder can say where in the span
        // its timing was measured, and the other can find that in its own
        // samples.
        let origin = Origin { span_sample: b.start_sample, span_rate_hz: self.rate };
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
                members.extend(classifier(b, spec, &self.reg).ok());
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
                locked: None,
                origin,
                ring: Ring::new(spec),
                evidence: None,
            });
        }
        // A source a lock claimed is that transmitter's, and the front end
        // that learned it is the only thing built on it: no classifier, no
        // decoder waiting on a verdict, no channel decoders whose width it
        // could be. It is built for the channel the lock names and told what
        // that front end learned, so it starts knowing the link rather than
        // recovering it again from the first packets of every visit.
        //
        // The detector's own measurement still rides along, as it does on a
        // source too wide to classify: a burst that reads is a row about
        // what was said, and one that does not is still a row saying
        // something transmitted there. It costs no signal processing.
        if let Some(c) = claimed {
            let p = protocol::by_id(c.protocol)
                .ok_or_else(|| common::Error::other(format!("no protocol {:?}", c.protocol)))?;
            let at = Placed {
                center_hz: b.center_hz as f64,
                width_hz: c.width_hz,
                rate: b.rate,
                snr_db: b.snr_db,
                origin: Some(origin),
            };
            let m = Member::place(p, spec, at, &c.settings, &self.reg)?;
            return Ok(Slot {
                id: b.id,
                center_hz: Hz(b.center_hz),
                members: vec![m],
                heard: false,
                spec,
                signal_hz: b.signal_hz,
                tried: Vec::new(),
                verdicts_seen: 0,
                remembered: false,
                locked: Some(c.id),
                origin,
                ring: Ring::new(spec),
                evidence: Some(
                    Evidence::new(b.center_hz, b.signal_hz, b.snr_db, b.rate)
                        .from_sample(b.start_sample),
                ),
            });
        }
        let (members, evidence) = found(b, spec, origin, &self.reg)?;
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
            locked: None,
            origin,
            ring: Ring::new(spec),
            evidence,
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

/// The decoders a source nothing has read before gets, and the evidence it
/// leaves if none of them can measure it.
///
/// The burst front end where the source is narrow enough for anything to
/// read what it says ([`routable`]), and otherwise the detector's own
/// measurement in its place. Then every protocol whose placement covers the
/// frequency, whose declared channel the source could be, and which does not
/// wait for the classifier's verdict.
///
/// A free function rather than a method, because the answer depends on the
/// source, the registry and nothing else: it can be asked, and checked,
/// without a detector or a running node.
pub(super) fn found(
    b: &SourceBlock,
    spec: StreamSpec,
    origin: Origin,
    reg: &pipeline::registry::Registry,
) -> Result<(Vec<Member>, Option<Evidence>)> {
    let mut members = Vec::new();
    let mut evidence = None;
    if routable(b) {
        members.push(classifier(b, spec, reg)?);
    } else {
        evidence = Some(
            Evidence::new(b.center_hz, b.signal_hz, b.snr_db, b.rate).from_sample(b.start_sample),
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
            // A decoder that will not build is left out rather than fatal:
            // the source still has the front end, and one decoder's refusal
            // is not a reason to stop the receiver.
            if let Ok(m) = Member::place(*p, spec, at, &Default::default(), reg) {
                members.push(m);
            }
        }
    }
    Ok((members, evidence))
}

/// The burst front end for a source, told how strong the detector found it:
/// a stream that begins inside a transmission is otherwise read as noise
/// from its first sample to its last.
fn classifier(
    b: &SourceBlock,
    spec: StreamSpec,
    reg: &pipeline::registry::Registry,
) -> Result<Member> {
    let route = NodeSpec::new("burst_route").f("source_snr_db", b.snr_db as f64);
    let mut m = Member::classifier(spec, route, reg)?;
    m.source_snr_db = b.snr_db;
    Ok(m)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry;

    /// A slot built by hand, with one front end on it, so what the fanout
    /// does with a block can be checked without a detector, an extractor or
    /// a node around it.
    fn slot(rate: f64, center: Hz, chain: Vec<NodeSpec>) -> Slot {
        let spec = StreamSpec::iq(rate, center);
        let m = Member::build("burst_route", None, spec, chain, &registry()).expect("a chain");
        Slot {
            id: SourceId(1),
            center_hz: center,
            members: vec![m],
            heard: false,
            spec,
            signal_hz: 25_000.0,
            tried: Vec::new(),
            verdicts_seen: 0,
            remembered: false,
            locked: None,
            origin: Origin { span_sample: 0, span_rate_hz: rate },
            ring: Ring::new(spec),
            evidence: None,
        }
    }

    fn block(id: SourceId, rate: f64, state: SourceState, samples: Vec<C32>) -> SourceBlock {
        SourceBlock {
            id,
            state,
            center_hz: 433_920_000,
            bandwidth_hz: 25_000.0,
            signal_hz: 25_000.0,
            rate,
            start_sample: 0,
            snr_db: 20.0,
            samples,
        }
    }

    /// A block with no source behind it is only run while a front end is
    /// still reading history, and a source that closes says so once.
    #[test]
    fn a_slot_runs_while_it_has_a_block_or_a_backlog() {
        let rate = 250_000.0;
        let mut s = slot(rate, Hz::mhz(434), vec![NodeSpec::new("burst_route")]);
        assert!(s.run_block(0, None, 0).is_none(), "nothing to read and nothing behind");

        let quiet = vec![C32::new(0.001, 0.0); 4_096];
        let r = s
            .run_block(3, Some(&block(SourceId(1), rate, SourceState::Running, quiet.clone())), 0)
            .expect("a running source is read");
        assert_eq!(r.k, 3, "the slot is named in the result, since the fanout reorders");
        assert!(!r.done);
        assert_eq!(r.heard, [], "the classifier measuring a burst is not reading it");

        let r = s
            .run_block(3, Some(&block(SourceId(1), rate, SourceState::Closed, quiet)), 0)
            .expect("the closing block is read");
        assert!(r.done, "a closed source with nothing catching up is done");
    }

    /// What a source too wide for the burst router leaves: the detector's
    /// measurement, once, when the source closes.
    #[test]
    fn a_slot_with_no_classifier_reports_what_the_detector_measured() {
        let rate = 250_000.0;
        let mut s = slot(rate, Hz::mhz(434), vec![NodeSpec::new("burst_route")]);
        s.members.clear();
        s.evidence = Some(Evidence::new(434_000_000, 5e6, 21.0, rate));
        let loud = vec![C32::new(0.5, 0.0); 4_096];
        let r = s
            .run_block(0, Some(&block(SourceId(1), rate, SourceState::Running, loud.clone())), 0)
            .expect("a running source is read");
        assert!(r.packets.is_empty(), "nothing to say until it closes");
        let r = s
            .run_block(0, Some(&block(SourceId(1), rate, SourceState::Closed, loud)), 0)
            .expect("the closing block is read");
        assert_eq!(r.packets.len(), 1, "one row for the whole transmission");
        let m = r.packets[0].measure.as_ref().expect("the measurement");
        assert_eq!(m.bandwidth_hz, 5e6);
        assert!(r.packets[0].iq.is_some(), "and the samples it was measured from");
    }

    /// What a source gets is decided by the source and the registry, and
    /// can be asked without a node: the classifier where anything could read
    /// what it says, and the channel decoders whose width it could be.
    #[test]
    fn what_a_source_gets_is_asked_of_the_registry_alone() {
        let reg = registry();
        let rate = 250_000.0;
        let origin = Origin { span_sample: 0, span_rate_hz: rate };
        let at = |hz: u64, width_hz: f64, rate: f64| {
            let mut spec = StreamSpec::iq(rate, Hz(hz));
            spec.bandwidth = width_hz;
            let mut b = block(SourceId(1), rate, SourceState::Opened, Vec::new());
            b.center_hz = hz;
            b.bandwidth_hz = width_hz;
            b.signal_hz = width_hz / 1.5;
            found(&b, spec, origin, &reg).expect("a placement")
        };

        let (members, evidence) = at(433_920_000, 25_000.0, rate);
        let names: Vec<&str> = members.iter().map(|m| m.name).collect();
        assert!(names.contains(&"burst_route"), "{names:?}");
        assert!(names.contains(&"pocsag"), "a 25 kHz channel could be a pager: {names:?}");
        assert!(evidence.is_none(), "the classifier is the evidence here");

        // Nothing here waits for a verdict: LoRa is placed later, on one.
        assert!(!names.contains(&"lora"), "{names:?}");

        // And a source no front end could read leaves the detector's own
        // measurement instead of a classifier.
        let (members, evidence) = at(2_462_000_000, 6.5e6, 20e6);
        assert!(members.iter().all(|m| m.router.is_none()), "a 6.5 MHz source was classified");
        assert!(evidence.is_some());
    }

    /// A stream a wider one supersedes leaves nothing: whatever was read off
    /// the sliver is half a burst.
    #[test]
    fn a_superseded_slot_reports_nothing_it_read() {
        let rate = 250_000.0;
        let mut s = slot(rate, Hz::mhz(434), vec![NodeSpec::new("burst_route")]);
        s.evidence = Some(Evidence::new(434_000_000, 5e6, 21.0, rate));
        let loud = vec![C32::new(0.5, 0.0); 4_096];
        let r = s
            .run_block(0, Some(&block(SourceId(1), rate, SourceState::Superseded, loud)), 0)
            .expect("the superseding block is read");
        assert!(r.packets.is_empty(), "{:?}", r.packets.len());
        assert!(r.done);
    }
}
