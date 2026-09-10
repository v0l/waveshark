//! One node that finds and decodes whatever is in the span, on its own.
//!
//! In at one end is complex baseband at whatever rate and centre the radio
//! has. Out at the other are packets: bursts as timings for the protocol
//! tables, and frames from the demodulators that produce bytes. Nothing in
//! between is told a frequency, a width or a modulation.
//!
//! Inside are the two things a span holds. Most of what transmits is found
//! by watching the span as a spectrogram, as [`dsp::source`] does: a source
//! opens where something appears, is cut out at a rate that fits its width,
//! and gets decoders of its own for as long as it lasts. The burst front end
//! always, since it measures the burst and picks the demodulator itself; and
//! the narrowband frame decoders whose channel a source of that width could
//! be, a pager or a packet channel, which decide for themselves whether the
//! bits are theirs. A pager channel is a pager channel at 153 MHz and at
//! 440 MHz, and a receiver that has to be told which is not detecting.
//!
//! The rest is what a spectrogram cannot find. A Mode S reply is 120
//! microseconds of pulses two megahertz wide, shorter than a frame; AIS is
//! two channels 50 kHz apart that stations alternate between. Those
//! demodulators watch the whole span themselves, and run when the span
//! covers the frequency they are for. That is the one piece of knowledge
//! about where things are that the node keeps, because it is knowledge about
//! the world rather than about this radio: 1090 MHz is 1090 MHz everywhere.

use common::{Hz, Packet, Result, SourceBlock};
use dsp::{SourceConfig, SourceDetector, SourceEvent};
use pipeline::event::{Event, Request};
use pipeline::graph::Topology;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Registry, Settings, SettingsExt, StageDesc};
use pipeline::Graph;
use rayon::prelude::*;
use std::collections::{BTreeMap, HashMap};
use std::time::Instant;

use crate::protocol::{self, Wake};

mod evidence;
mod locks;
mod member;
mod memory;
mod place;
mod requests;
mod watch;

use locks::Locks;
use member::{Member, Ring};
use memory::{Memory, STICKY_ID_BASE};
use place::{Slot, SlotResult};
use watch::Watch;

/// SNR a bin must reach before the auto node opens a source there.
///
/// Above the detector's own default because this node builds a decoder chain
/// for everything it opens: on a band with a strong transmitter on it, the
/// weakest openings are mostly the splash and spurs around one signal, and
/// each of them costs a chain.
pub const AUTO_OPEN_DB: f32 = 15.0;

pub struct AutoNode {
    label: String,
    cfg: SourceConfig,
    rate: f64,
    center: Hz,
    input_bw: f64,
    /// What finds the sources and cuts them out, and where it may look.
    watch: Watch,
    reg: Registry,
    slots: Vec<Slot>,
    /// Decoders that watch the whole span, and the bands they own, in
    /// absolute hertz, where no source is opened.
    wide: Vec<Member>,
    /// The span itself, behind them: every span-wide front end is handed the
    /// same block, so the samples a packet of theirs leaves with are kept
    /// once.
    wide_ring: Ring,
    /// The burst front end at a nominal rate, for the view and the
    /// parameters before any source has opened.
    template: Option<Graph>,
    events: Vec<SourceEvent>,
    blocks: Vec<SourceBlock>,
    /// Sources decoders were built for, over the node's life.
    built: u64,
    /// The channels the receiver has decided to keep listening on.
    memory: Memory,
    /// The transmitters a front end has learned well enough to claim their
    /// bursts before anything else runs.
    locks: Locks,
    /// What each channel has announced about itself, so a source that
    /// closes and opens again, or decoders rebuilt with the graph, do not
    /// log the same cell's identity a second time.
    announced: HashMap<u64, Vec<Vec<u8>>>,
    /// Where a block's time went: watching the band, cutting sources out,
    /// the front ends as a whole, and each kind of front end's processor
    /// time summed over every source it was running on.
    phases: BTreeMap<String, pipeline::cost::Ring>,
    /// Scratch for the per-kind sums of one block.
    phase_sum: BTreeMap<&'static str, u64>,
}
impl AutoNode {
    pub fn new(label: impl Into<String>, cfg: SourceConfig) -> Self {
        Self {
            label: label.into(),
            cfg,
            rate: 0.0,
            center: Hz(0),
            input_bw: 0.0,
            watch: Watch::default(),
            reg: crate::registry(),
            slots: Vec::new(),
            wide: Vec::new(),
            wide_ring: Ring::new(StreamSpec::iq(0.0, Hz(0))),
            template: None,
            events: Vec::new(),
            blocks: Vec::new(),
            built: 0,
            memory: Memory::default(),
            locks: Locks::default(),
            announced: HashMap::new(),
            phases: BTreeMap::new(),
            phase_sum: BTreeMap::new(),
        }
    }

    /// Start cutting out the channels kept from earlier: opened once, at
    /// their own width, never closed. The extractor takes them from the
    /// current position, so a channel kept before a rebuild starts again
    /// where the new span begins.
    fn open_kept_channels(&mut self) {
        let c0 = self.center.as_f64();
        let half = self.input_bw / 2.0;
        for id in self.memory.take_pending() {
            let Some(st) = self.memory.find(id) else {
                continue;
            };
            let (hz, width) = (st.center_hz, st.width_hz);
            if (hz - c0).abs() + width / 2.0 > half {
                continue;
            }
            // Not while the source that earned it is still open: two
            // decoders on one channel are every burst twice in the log, and
            // the new one would start mid-transmission without the header
            // the old one read. It takes over once that source closes.
            let busy = self.slots.iter().any(|sl| {
                sl.id.0 < STICKY_ID_BASE && (sl.center_hz.as_f64() - hz).abs() <= width / 2.0
            });
            if busy {
                self.memory.wait_for(id);
                continue;
            }
            self.watch.open_channel(id, hz, width);
        }
        for id in self.memory.take_expiring() {
            self.watch.close_channel(id);
        }
    }

    /// Every front end whose channel the source could be was built for it and
    /// asked; the one that read a frame has answered what the source is, and
    /// from here it alone reads it.
    ///
    /// The others were each a decoder's worth of work per block and, for a
    /// pager or a packet channel, a second row saying the same burst was
    /// nothing. Where several widths of one protocol read, the protocol says
    /// which to keep.
    fn latch(&mut self, k: usize, heard: &[(&'static str, f64)]) {
        if heard.is_empty() || self.slots[k].members.len() <= 1 {
            return;
        }
        let mut keep: Vec<(&'static str, f64)> = Vec::new();
        for p in protocol::all() {
            let mut widths: Vec<f64> =
                heard.iter().filter(|(n, _)| *n == p.id()).map(|(_, w)| *w).collect();
            if widths.is_empty() {
                continue;
            }
            p.resolve_widths(&mut widths);
            keep.extend(widths.into_iter().map(|w| (p.id(), w)));
        }
        // The classifier stays on a remembered channel. The detector is
        // locked out of one, so nothing else will ever find a second
        // transmitter sharing the frequency, and dropping the classifier
        // there is what made a LoRa network at another bandwidth invisible
        // for the session. On an ordinary source it goes as before: the
        // detector is still watching and will open the other signal itself.
        let remembered = self.slots[k].remembered;
        self.slots[k].members.retain(|m| {
            (remembered && m.router.is_some())
                || keep.iter().any(|(n, w)| *n == m.name && *w == m.channel_hz)
        });
    }

    /// A cell's identity, once, per channel, whatever the decoders that read
    /// it have been through since. Each protocol on the source says which of
    /// its packets are the same news.
    fn announce_once(&mut self, k: usize, packets: Vec<Packet>, out: &mut Vec<Packet>) {
        let seen = self.announced.entry(self.slots[k].center_hz.0).or_default();
        let members = &self.slots[k].members;
        out.extend(packets.into_iter().filter(|p| {
            let key =
                members.iter().filter_map(|m| m.protocol).find_map(|proto| proto.dedupe_key(p));
            let Some(key) = key else { return true };
            if seen.contains(&key) {
                return false;
            }
            seen.push(key);
            true
        }));
    }

    fn phase(&mut self, name: &str, us: u64, block_s: f64) {
        let ring = match self.phases.get_mut(name) {
            Some(r) => r,
            None => self.phases.entry(name.to_string()).or_default(),
        };
        ring.push(us.min(u32::MAX as u64) as u32, block_s);
    }

    /// Sources open right now.
    pub fn live(&self) -> Vec<dsp::Source> {
        self.watch.live()
    }

    /// Sources with decoders on them right now.
    pub fn active(&self) -> usize {
        self.slots.len()
    }

    /// Sources decoders were built for since the node was made.
    pub fn built(&self) -> u64 {
        self.built
    }

    /// The span-wide decoders running, by stage name.
    pub fn wide(&self) -> Vec<&'static str> {
        self.wide.iter().map(|m| m.name).collect()
    }

    /// Speech from every front end inside, read off the ports it came out
    /// on, whatever protocol produced it: a source found a moment ago has
    /// decoders built for it there and then, and they are as much a part of
    /// the receiver as a stage somebody placed by hand.
    fn inner_voice(&self, out: &mut Vec<common::Voice>) {
        // The span-wide front ends as well as the ones on a source: a camera
        // is span-wide and the sound on its subcarrier is speech like any
        // other. Reading only the slots is why a picture arrived with no
        // sound at all.
        for m in &self.wide {
            for t in &m.voice {
                let Some(v) = m.graph.buf(*t).and_then(|p| p.as_voice()) else {
                    continue;
                };
                out.extend(v.iter().cloned());
            }
        }
        for slot in &self.slots {
            for m in &slot.members {
                for t in &m.voice {
                    let Some(v) = m.graph.buf(*t).and_then(|p| p.as_voice()) else {
                        continue;
                    };
                    out.extend(v.iter().cloned());
                }
            }
        }
    }

    fn inner_video(&self, out: &mut Vec<common::VideoFrame>) {
        for m in &self.wide {
            for t in &m.video {
                let Some(v) = m.graph.buf(*t).and_then(|p| p.as_video()) else {
                    continue;
                };
                out.extend(v.iter().cloned());
            }
        }
        for slot in &self.slots {
            for m in &slot.members {
                for t in &m.video {
                    let Some(v) = m.graph.buf(*t).and_then(|p| p.as_video()) else {
                        continue;
                    };
                    out.extend(v.iter().cloned());
                }
            }
        }
    }
}

/// What one span-wide front end made of a block.
struct WideResult {
    name: &'static str,
    events: Vec<Event>,
    packets: Vec<Packet>,
    spent_us: u64,
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// What comes out: everything decoded, and everything heard.
const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;
const OUT_VIDEO: usize = 2;

impl Node for AutoNode {
    fn name(&self) -> &str {
        DESC.name
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        3
    }

    /// Every chain running inside: the span-wide front ends, and each front
    /// end on each source found. All of them, because each is a different
    /// chain over a different stream, and one of them drawn with a number
    /// beside it says a bank's kind of thing about something that is not a
    /// bank. Before anything has opened, the burst front end a source would
    /// get, so the node is not an empty box on a quiet band.
    fn subgraphs(&self) -> Vec<Topology> {
        let mut out: Vec<Topology> = self.wide.iter().map(|m| m.graph.topology()).collect();
        out.extend(self.slots.iter().flat_map(|s| s.members.iter()).map(|m| m.graph.topology()));
        if out.is_empty() {
            out.extend(self.template.as_ref().map(|g| g.topology()));
        }
        out
    }

    /// The band watched, the tuner's own spur inside it and the channel plan
    /// on it: all settled before the node goes into the graph, since what a
    /// detector opens on is decided as the graph negotiates and the span has
    /// usually moved since this node was last built.
    fn configure(&mut self, settings: &Settings) {
        self.set_band(crate::band_of(settings));
        let spur = settings.f64_or("spur_hz", 0.0);
        self.set_spur((spur > 0.0).then_some(spur));
        let step = settings.f64_or("raster_hz", 0.0);
        self.set_raster((step > 0.0).then(|| (settings.f64_or("raster_origin_hz", 0.0), step)));
    }

    /// Every decoder this node is running, span-wide and on a source, so
    /// something asked of every node in the receiver is asked of them too:
    /// the key manager reaches a front end the auto node placed a moment
    /// ago as surely as one the scanner table did.
    fn each_inner(&self, f: &mut dyn FnMut(&dyn Node)) {
        for m in self.wide.iter().chain(self.slots.iter().flat_map(|s| s.members.iter())) {
            for (id, _) in m.graph.order() {
                if let Some(n) = m.graph.node(id) {
                    f(n);
                }
            }
        }
    }

    fn each_inner_mut(&mut self, f: &mut dyn FnMut(&mut dyn Node)) {
        for m in
            self.wide.iter_mut().chain(self.slots.iter_mut().flat_map(|s| s.members.iter_mut()))
        {
            let ids: Vec<_> = m.graph.order().map(|(id, _)| id).collect();
            for id in ids {
                if let Some(n) = m.graph.node_mut(id) {
                    f(n);
                }
            }
        }
    }

    fn phases(&self) -> Vec<(String, pipeline::cost::Cost)> {
        self.phases.iter().map(|(n, r)| (n.clone(), r.cost())).collect()
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other(format!("{}: needs IQ", self.label)));
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        self.input_bw =
            if i.spec.bandwidth > 0.0 { i.spec.bandwidth.min(i.spec.rate) } else { i.spec.rate };
        self.rebuild()?;
        // Packets are events in time, not a sampled stream, and each one
        // carries its own frequency and width.
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.rate = 0.0;
        out.bandwidth = 0.0;
        // Speech from whatever front end inside is carrying it, at the
        // vocoder's rate rather than the radio's.
        let mut voice = out.with_kind(PortKind::Voice);
        voice.rate = crate::m17_nodes::VOICE_HZ;
        // Pictures from whatever front end inside is producing them. A field
        // is not a sampled stream, so the rate says nothing and the frame
        // carries its own geometry.
        let video = out.with_kind(PortKind::Video);
        Ok(vec![out, voice, video])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        // The whole call, so what the phases below do not account for can be
        // reported rather than guessed at: a block slower than the sum of its
        // nodes is a block spending time somewhere nobody is looking.
        let t_block = Instant::now();
        let iq = inputs[0].as_iq().unwrap_or(&[]);
        if !self.watch.ready() {
            return Ok(());
        }
        let at_us = now_us();
        let rate = c.inputs[0].spec.rate.max(1.0);
        let out = outputs[OUT_PACKETS].packets_mut();

        // The sources: find, cut out, and read. The span-wide decoders run
        // beside them, in the same fanout, once the blocks are known.
        let mut events: Vec<Event> = Vec::new();
        self.events.clear();
        let c0 = self.center.as_f64();
        let block_s = c.block_seconds;
        // While a camera is locked, the span is that camera and there is
        // nothing to detect in it. Every run inside a 20 MHz FM carrier is a
        // piece of the picture, and opening each one costs an extraction and
        // a set of front ends that report sensors nobody transmitted.
        // Measured on the AKK capture: detection, extraction and the front
        // ends together ran at nearly four times real time on them, which is
        // a receiver that cannot keep up rather than one that reads more. The
        // picture going away puts all of it back.
        let watching = self.claimed_whole_span();
        // What a front end already owns, and the tuner's own centre, the
        // detector refuses for itself; see [`AutoNode::apply_locked`] and
        // [`dsp::SourceDetector::set_spur`].
        let t_detect = Instant::now();
        let mut found = std::mem::take(&mut self.events);
        self.watch.admit(iq, c0, watching, &mut found);
        self.events = found;
        let detect_us = t_detect.elapsed().as_micros() as u64;
        self.open_kept_channels();
        self.memory.set_now(self.watch.position() as f64 / rate);
        self.blocks.clear();
        let t_extract = Instant::now();
        let mut blocks = std::mem::take(&mut self.blocks);
        self.watch.cut(iq, &self.events, &mut blocks);
        self.blocks = blocks;
        let extract_us = t_extract.elapsed().as_micros() as u64;
        let (bank_feed_us, bank_start_us) = self.watch.bank_cost();
        for ev in &self.events {
            if let SourceEvent::Opened(s) = ev {
                events.push(Event::Detection {
                    center: Hz((self.center.as_f64() + s.center_hz).max(0.0) as u64),
                    bandwidth: s.bandwidth_hz(),
                    snr_db: s.peak_snr_db,
                    at: s.start_sample as f64 / rate,
                });
            }
        }
        // Every source is offered to the locks before anything is built on
        // it: a transmitter a front end has already learned is read by that
        // front end alone, and costs one extraction and nothing else. See
        // [`locks`].
        let blocks = std::mem::take(&mut self.blocks);
        for b in &blocks {
            if !self.slots.iter().any(|s| s.id == b.id) {
                let claimed = self.locks.claim(b);
                let slot = self.open(b, claimed)?;
                self.slots.push(slot);
                self.built += 1;
            }
        }
        self.blocks = blocks;

        // Every member of every source is a task of its own, not one task
        // per source: the members share nothing but the block they read, and
        // per-source tasks left an m17 member decoding voice alone on one
        // lane while the others sat finished. The span-wide decoders join
        // the same fanout, since a Mode S correlator over the whole span
        // costs more than any narrowband member.
        let blocks = &self.blocks;
        // A front end that has claimed the whole span is the only thing on
        // it, and that goes for the other span-wide decoders as much as for
        // the detector: an OFDM correlator over 20 MS/s of FM camera carrier
        // is eighteen times real time spent proving there is no Wi-Fi in a
        // picture. Whichever member holds the claim keeps running, so the
        // claim can be given back.
        let span = (c0 - self.input_bw / 2.0, c0 + self.input_bw / 2.0);
        let claimant = |m: &Member| m.band.is_some_and(|(a, b)| a <= span.0 && span.1 <= b);
        // The span, kept once for every front end over it rather than once
        // each: they are all handed the same block. A gated front end needs
        // it whether or not it produces packets, since the lead-in it wakes
        // on comes out of it.
        self.wide_ring.keeps = self
            .wide
            .iter()
            .any(|m| m.keeps_samples || m.protocol.is_some_and(|p| p.wakes_on() != Wake::Always));
        let t_ring = Instant::now();
        self.wide_ring.push(iq);
        let ring_us = t_ring.elapsed().as_micros() as u64;
        // What the detector has open, which is what a gated span-wide front
        // end runs on, and how much of the lead-in it missed getting there.
        let detecting = self.watch.detecting();
        let lead = self.watch.latency_samples();
        let wide_ring = &self.wide_ring;
        let wide = &mut self.wide;
        let slots = &mut self.slots;
        let t_fronts = Instant::now();
        let (wide_results, results): (Vec<WideResult>, Vec<SlotResult>) = rayon::join(
            || {
                wide.par_iter_mut()
                    .map(|m| {
                        // Not this block: nothing the detector can see is on
                        // the air, something else owns the span, or this
                        // front end is sampling the air rather than reading
                        // all of it.
                        let awake =
                            m.awake(detecting, iq.len(), rate) && (!watching || claimant(m));
                        if !awake {
                            m.sleep(iq.len());
                            return WideResult {
                                name: m.name,
                                events: Vec::new(),
                                packets: Vec::new(),
                                spent_us: 0,
                            };
                        }
                        m.wake(wide_ring, lead, iq.len());
                        if !m.wants(iq.len(), rate) {
                            m.skip(iq.len());
                            return WideResult {
                                name: m.name,
                                events: Vec::new(),
                                packets: Vec::new(),
                                spent_us: 0,
                            };
                        }
                        let mut packets = Vec::new();
                        let t = Instant::now();
                        let events = m.run(iq, at_us, &mut packets, wide_ring);
                        if !packets.is_empty() {
                            m.read_something();
                        }
                        WideResult {
                            name: m.name,
                            events,
                            packets,
                            spent_us: t.elapsed().as_micros() as u64,
                        }
                    })
                    .collect()
            },
            || {
                slots
                    .par_iter_mut()
                    .enumerate()
                    .filter_map(|(k, slot)| {
                        slot.run_block(k, blocks.iter().find(|b| b.id == slot.id), at_us)
                    })
                    .collect()
            },
        );
        let fronts_us = t_fronts.elapsed().as_micros() as u64;
        let tail_us = (t_block.elapsed().as_micros() as u64)
            .saturating_sub(detect_us + extract_us + fronts_us + ring_us);
        self.phase_sum.clear();
        // What was asked, by the source it was asked on and the front end
        // that asked. Answered once the slots have settled.
        let mut asked: Vec<(Option<usize>, &'static str, Request)> = Vec::new();
        for w in wide_results {
            for e in w.events {
                match e {
                    Event::Request(request) => asked.push((None, w.name, request)),
                    e => events.push(e),
                }
            }
            out.extend(w.packets);
            *self.phase_sum.entry(w.name).or_default() += w.spent_us;
        }
        let mut results = results;
        results.sort_by_key(|r| r.k);
        for r in &results {
            for (name, us) in &r.spent {
                *self.phase_sum.entry(name).or_default() += us;
            }
        }
        self.phase("detect", detect_us, block_s);
        self.phase("extract", extract_us, block_s);
        self.phase("extract bank", bank_feed_us, block_s);
        self.phase("extract catch-up", bank_start_us, block_s);
        self.phase("fronts", fronts_us, block_s);
        // Keeping the span for the front ends over it, and everything else
        // this node does with a block. Measured because a spike outside every
        // phase used to be blamed on the graph runner: it is 90 us and 3 us a
        // block at 20 MS/s, so the block's time is in the phases above.
        self.phase("wide ring", ring_us, block_s);
        self.phase("tail", tail_us, block_s);
        let sums: Vec<(&'static str, u64)> = self.phase_sum.iter().map(|(n, u)| (*n, *u)).collect();
        for (name, us) in sums {
            self.phase(&format!("{name} cpu"), us, block_s);
        }
        let mut closed = Vec::new();
        for SlotResult { k, events: ev, packets: pk, done, heard, .. } in results {
            let center = self.slots[k].center_hz;
            let named = self.slots[k]
                .members
                .iter()
                .any(|m| m.verdicts.len() > self.slots[k].verdicts_seen);
            if named {
                self.place_on_verdict(k, done);
            }
            for (name, width) in &heard {
                if let Some(e) = self.remember(name, center.as_f64(), *width) {
                    c.emit(e);
                }
            }
            self.latch(k, &heard);
            // A remembered channel that is still decoding is kept; one that
            // has gone quiet for its hold is given back to the detector.
            if !pk.is_empty() {
                self.memory.heard_at(self.slots[k].center_hz.as_f64());
            }
            for (name, e) in ev {
                match e {
                    Event::Request(request) => asked.push((Some(k), name, request)),
                    e => events.push(e),
                }
            }
            self.announce_once(k, pk, out);
            // A decoder placed this block on a source that has already
            // closed still has the history to read; the slot stays until
            // it has.
            if done && !self.slots[k].members.iter().any(|m| m.behind()) {
                closed.push(self.slots[k].id);
                // What became of a claimed source: a lock is right about
                // this transmitter exactly as often as the front end it
                // handed the burst to read something.
                if let Some(id) = self.slots[k].locked {
                    let heard = self.slots[k].heard;
                    if let Some((protocol, transmitter)) = self.locks.scored(id, heard) {
                        c.emit(Event::Warning {
                            message: format!(
                                "{protocol} {transmitter}: too few of the bursts it claimed \
                                 decoded; giving them back to the detector"
                            ),
                        });
                    }
                }
            }
        }
        self.slots.retain(|s| !closed.contains(&s.id));
        // What the decoders asked for, answered here where it can be, and
        // handed on where it cannot. After the slots are settled, so a
        // release or a reshape closes what is there now.
        for (k, name, r) in asked {
            let mut said = Vec::new();
            if let Some(r) = self.answer(k, name, r, &mut said) {
                c.emit(Event::Request(r));
            }
            for e in said {
                c.emit(e);
            }
        }
        let expired = self.memory.expire();
        if !expired.is_empty() {
            self.apply_locked();
            for id in expired {
                let Some(st) = self.slots.iter().find(|s| s.id == id) else {
                    continue;
                };
                if let Some(m) = st.members.first() {
                    c.emit(Event::Warning {
                        message: format!(
                            "{} quiet on {:.4} MHz; giving the channel back",
                            m.name,
                            st.center_hz.as_f64() / 1e6
                        ),
                    });
                }
            }
        }
        self.inner_voice(outputs[OUT_VOICE].voice_mut());
        self.inner_video(outputs[OUT_VIDEO].video_mut());

        for e in events {
            match &e {
                // Warnings are per burst and per source; across a whole band
                // they arrive in the thousands.
                Event::Warning { .. } => {}
                _ => c.emit(e),
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.watch.reset();
        self.slots.clear();
        self.wide_ring.reset();
        for m in &mut self.wide {
            m.graph.reset();
        }
    }

    /// The detector's knobs, then the burst front end's.
    fn params(&self) -> Vec<Param> {
        let mut p = vec![
            Param::float("open_db", self.cfg.open_db as f64, 3.0..=40.0)
                .unit("dB")
                .label("SNR that opens a source"),
            Param::float("close_db", self.cfg.close_db as f64, 1.0..=40.0)
                .unit("dB")
                .label("SNR that closes it again"),
            Param::float("hang_ms", self.cfg.hang_us as f64 / 1e3, 1.0..=2_000.0)
                .unit("ms")
                .label("Silence that closes a source")
                .log(),
            Param::float("bin_hz", self.cfg.bin_hz, 100.0..=100_000.0)
                .unit("Hz")
                .label("Spectral resolution")
                .log(),
        ];
        if let Some(t) = &self.template {
            p.extend(t.topology().nodes.into_iter().flat_map(|n| n.params));
        }
        p
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let f = v.as_f64().unwrap_or_default();
        match name {
            "open_db" => {
                self.cfg.open_db = f as f32;
                self.cfg.close_db = self.cfg.close_db.min(self.cfg.open_db - 1.0);
            }
            "close_db" => self.cfg.close_db = (f as f32).min(self.cfg.open_db - 1.0),
            "hang_ms" => self.cfg.hang_us = (f * 1e3).max(1.0) as u32,
            "bin_hz" => {
                self.cfg.bin_hz = f.max(1.0);
                return self.rebuild();
            }
            "raster_hz" => {
                let origin = self.raster().map(|(o, _)| o).unwrap_or(0.0);
                self.set_raster((f > 0.0).then_some((origin, f)));
            }
            "raster_origin_hz" => {
                if let Some((_, step)) = self.raster() {
                    self.set_raster(Some((f, step)));
                }
            }
            _ => {
                // A front end's own knob: set on the template, so sources
                // that open later start with it, and on every copy running
                // now, on a source or over the span. A knob that reached the
                // sources and not the span-wide front ends was a knob that
                // did nothing at 1090 MHz.
                let mut found = false;
                let mut err = None;
                let mut apply = |g: &mut Graph| {
                    let ids: Vec<_> = g.topology().nodes.iter().map(|n| n.id).collect();
                    for id in ids {
                        let Some(node) = g.node_mut(id) else { continue };
                        if !node.params().iter().any(|p| p.name == name) {
                            continue;
                        }
                        found = true;
                        if let Err(e) = node.set_param(name, v.clone()) {
                            err = Some(e);
                        }
                    }
                };
                if let Some(t) = &mut self.template {
                    apply(t);
                }
                for m in self
                    .wide
                    .iter_mut()
                    .chain(self.slots.iter_mut().flat_map(|s| s.members.iter_mut()))
                {
                    apply(&mut m.graph);
                }
                return match err {
                    Some(e) => Err(e),
                    None if found => Ok(()),
                    None => Err(common::Error::other(format!(
                        "{}: unknown parameter {name:?}",
                        self.label
                    ))),
                };
            }
        }
        // Thresholds and timings: read every frame, so the detector is
        // built again with them and nothing else changes.
        let cfg = self.detector_cfg();
        let (rate, bw) = (self.rate, self.input_bw);
        if self.watch.rebuild_detector(|| SourceDetector::new(rate, bw, cfg)) {
            self.apply_band();
            self.apply_locked();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{SourceState, C32};
    use pipeline::node::Node;

    fn spec(rate: f64, center: Hz) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, center), latency: 0 }
    }

    #[test]
    fn the_node_turns_iq_into_packets() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        let out = Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(433))]).unwrap();
        assert_eq!(out[0].kind, PortKind::Packets);
        assert!(n.wide().is_empty(), "nothing span-wide belongs at 433 MHz");
        assert!(!Node::subgraphs(&n).is_empty(), "the burst front end is shown before any source");
    }

    /// Noise with a keyed carrier `offset` hertz up from the centre for the
    /// last stretch of it.
    fn keyed(rate: f64, offset: f64) -> Vec<C32> {
        keyed_for(rate, offset, 600_000)
    }

    /// The same, as long as the detector needs at the span's own rate: the
    /// floor is not measured for the first thirty-two frames, and a frame at
    /// 20 MS/s is eight times the samples it is at 2.4.
    fn keyed_for(rate: f64, offset: f64, samples: usize) -> Vec<C32> {
        let mut seed = 0x51u64;
        let mut iq: Vec<C32> = (0..samples)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let a = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let b = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                C32::new(a * 0.05, b * 0.05)
            })
            .collect();
        let from = samples / 2;
        for i in 0..samples / 6 {
            if (i / 500) % 2 == 0 {
                let ph = std::f64::consts::TAU * offset * i as f64 / rate;
                iq[from + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
            }
        }
        iq
    }

    /// Where the node said sources opened, as offsets from the centre.
    fn openings(n: &mut AutoNode, rate: f64, center: Hz, iq: &[C32]) -> Vec<f64> {
        let ins = [spec(rate, center)];
        let mut opened = Vec::new();
        for block in iq.chunks(16_384) {
            let input = Payload::Iq(block.to_vec());
            let mut out = [
                Payload::Packets(Vec::new()),
                Payload::Voice(Vec::new()),
                Payload::Video(Vec::new()),
            ];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(n, &[&input], &mut out, &mut ctx).unwrap();
            opened.extend(events.iter().filter_map(|e| match e {
                Event::Detection { center: c, .. } => Some(c.as_f64() - center.as_f64()),
                _ => None,
            }));
        }
        opened
    }

    /// The video front end runs on the span, not on a source.
    ///
    /// This test used to assert the opposite, and the opposite did not work:
    /// a detector measures the few megahertz of a camera's carrier that stand
    /// above the floor, the front end was placed on that, and filtering a
    /// twenty megahertz FM transmission down to four leaves no line rate to
    /// lock to. So the only question here is whether the span could be
    /// carrying a picture at all.
    #[test]
    fn the_video_front_end_runs_where_a_picture_would_fit() {
        let placed = |rate: f64| -> bool {
            let mut n = AutoNode::new("auto", SourceConfig::default());
            Node::negotiate(&mut n, &[spec(rate, Hz::mhz(5800))]).unwrap();
            n.wide().contains(&"video")
        };
        assert!(placed(20e6), "a 20 MS/s span at 5.8 GHz reads no video");
        // PAL luma reaches 5 MHz with the subcarrier at 4.43, so a slower
        // span cannot be carrying a picture whatever else is in it.
        assert!(!placed(8e6));
    }

    /// A front end that has claimed the whole span is the only thing reading
    /// it. The other span-wide decoders stop too, not only the detector: an
    /// OFDM correlator over 20 MS/s of FM camera carrier measured eighteen
    /// times real time to prove there is no Wi-Fi inside a picture.
    #[test]
    fn a_claim_on_the_whole_span_stops_the_other_span_wide_decoders() {
        // 5805 is Wi-Fi channel 161 and channel A4 of the video plan, so
        // both front ends are on this span.
        let (rate, center) = (20e6, Hz::mhz(5805));
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(rate, center)]).unwrap();
        assert!(n.wide().contains(&"video") && n.wide().contains(&"wifi"), "{:?}", n.wide());

        // Something on the air, since both of these are gated on the
        // detector having found something; see [`protocol::Wake`].
        let busy = keyed_for(rate, 4e6, 2_000_000);
        let run = |n: &mut AutoNode, block: &[C32]| -> Vec<(String, u64)> {
            let ins = [spec(rate, center)];
            let input = Payload::Iq(block.to_vec());
            let mut out = [
                Payload::Packets(Vec::new()),
                Payload::Voice(Vec::new()),
                Payload::Video(Vec::new()),
            ];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(n, &[&input], &mut out, &mut ctx).unwrap();
            // What this block cost, not the running mean the chain view
            // draws: a mean is still warm the block after the work stopped.
            n.phase_sum.iter().map(|(k, v)| (k.to_string(), *v)).collect()
        };
        let spent = |ph: &[(String, u64)], name: &str| -> u64 {
            ph.iter().find(|(k, _)| k == name).map(|(_, v)| *v).unwrap_or(0)
        };

        let mut before = Vec::new();
        for block in busy.chunks(16_384) {
            before = run(&mut n, block);
        }
        assert!(spent(&before, "wifi") > 0, "{before:?}");
        let block = &busy[busy.len() - 16_384..];

        // The camera says the span is its picture.
        let mut out = Vec::new();
        n.answer(
            None,
            "video",
            Request::Claim { lo_hz: center.as_f64() - rate, hi_hz: center.as_f64() + rate },
            &mut out,
        );
        let after = run(&mut n, block);
        assert_eq!(spent(&after, "wifi"), 0, "{after:?}");
        assert!(spent(&after, "video") > 0, "the claimant still reads {after:?}");

        // And giving it back puts everything else back on the span.
        n.answer(None, "video", Request::Release, &mut out);
        let back = run(&mut n, block);
        assert!(spent(&back, "wifi") > 0, "{back:?}");
    }

    /// One span-wide front end per protocol, whatever the plan calls the
    /// span. The 5.8 GHz video plan lists 5865 as A1 and 5866 as B8, and
    /// both channels' 18 MHz fit inside a 20 MS/s span at 5865, so the
    /// receiver demodulated the whole span twice and published every field
    /// twice: 41 fields a second arriving at the bus for a camera sending 20.
    /// A decoder that works per sample is handed the rate it asked for, not
    /// the span. Mode S wants 2.4 MS/s for a 1 Mbit/s pulse train and cost
    /// 127% of a core reading an empty 20 MS/s band; narrowed it costs 37%.
    /// A decoder that cuts its own channels out gets the span as before,
    /// because a filter in front of it is a second pass for nothing.
    #[test]
    fn a_per_sample_decoder_is_handed_the_rate_it_asked_for() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(20e6, Hz::mhz(1090))]).unwrap();
        let m = n.wide.iter().find(|m| m.name == "mode_s").expect("a mode s front end");
        let topo = m.graph.topology();
        let node = topo.nodes.iter().find(|x| x.kind == "mode_s").expect("the decoder");
        let fed = node.inputs[0].1.rate;
        assert!(fed <= 5e6 && fed >= 2e6, "mode s was handed {fed} S/s");

        // BLE reads a span and cuts its three advertising channels out of
        // it, so it keeps the whole span.
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(20e6, Hz(2_426_000_000))]).unwrap();
        let m = n.wide.iter().find(|m| m.name == "ble").expect("a ble front end");
        let topo = m.graph.topology();
        let node = topo.nodes.iter().find(|x| x.kind == "ble").expect("the decoder");
        assert_eq!(node.inputs[0].1.rate, 20e6);
    }

    /// How many of `blocks` blocks of `iq` a span-wide front end read.
    fn blocks_read(
        n: &mut AutoNode,
        rate: f64,
        center: Hz,
        iq: &[C32],
        name: &str,
    ) -> (usize, usize) {
        let ins = [spec(rate, center)];
        let mut read = 0usize;
        let mut blocks = 0usize;
        for block in iq.chunks(131_072) {
            let input = Payload::Iq(block.to_vec());
            let mut out = [
                Payload::Packets(Vec::new()),
                Payload::Voice(Vec::new()),
                Payload::Video(Vec::new()),
            ];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(n, &[&input], &mut out, &mut ctx).unwrap();
            blocks += 1;
            if n.phase_sum.iter().any(|(k, v)| *k == name && *v > 0) {
                read += 1;
            }
        }
        (read, blocks)
    }

    /// A front end gated on the detector reads none of a band nothing is
    /// transmitting in. Wi-Fi read 7.5 seconds of an empty 5.8 GHz band in
    /// 3.1 seconds of CPU for no frames at all, which is an afternoon spent
    /// proving a band is quiet, and the camera beside it cost most of a core
    /// demodulating a picture nobody was sending.
    #[test]
    fn a_gated_front_end_reads_nothing_while_nothing_is_transmitting() {
        let (rate, center) = (20e6, Hz::mhz(5805));
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(rate, center)]).unwrap();
        let quiet = vec![C32::new(0.0, 0.0); (2.0 * rate) as usize];
        for name in ["wifi", "video"] {
            let (read, blocks) = blocks_read(&mut n, rate, center, &quiet, name);
            assert_eq!(read, 0, "{name} read {read} of {blocks} empty blocks");
        }
    }

    /// And once something is transmitting, a front end whose traffic repeats
    /// samples the air rather than reading all of it: every network beacons
    /// ten times a second, so a fifth of the air names them all.
    #[test]
    fn a_sampling_front_end_reads_a_fraction_of_the_air() {
        let (rate, center) = (20e6, Hz::mhz(5805));
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(rate, center)]).unwrap();
        // A fifth of a second in every second is the cycle, so this has to
        // run for seconds to measure it: a fifth of a second of 20 MS/s
        // played over and over, which is what a band with something on it
        // looks like without holding gigabytes of it. The first pass wakes
        // the front end and is not counted.
        let busy = keyed_for(rate, 4e6, 4_000_000);
        let (_, warm) = blocks_read(&mut n, rate, center, &busy, "wifi");
        assert!(warm > 0);
        let (mut read, mut blocks) = (0, 0);
        for _ in 0..12 {
            let (r, b) = blocks_read(&mut n, rate, center, &busy, "wifi");
            read += r;
            blocks += b;
        }
        // A fifth of the blocks, give or take where the window falls in a
        // block.
        let share = read as f64 / blocks as f64;
        assert!((0.1..0.4).contains(&share), "{read} of {blocks} blocks");
        // And the camera, which reads a carrier that is there all the time,
        // is handed every block it is awake for.
        let video = n.wide.iter().find(|m| m.name == "video").expect("a camera front end");
        assert!(matches!(
            video.protocol.map(|p| p.watch()),
            Some(crate::protocol::Watch::Everything)
        ));
    }

    #[test]
    fn a_span_wide_front_end_is_placed_once_per_span() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(20e6, Hz::mhz(5865))]).unwrap();
        let video = n.wide().iter().filter(|w| **w == "video").count();
        assert_eq!(video, 1, "{:?}", n.wide());
    }

    #[test]
    fn a_source_near_a_channel_of_the_plan_is_that_channel() {
        // A source is a measurement, and a measurement of a channel that a
        // plan lists is the channel seen through a tuner a few parts per
        // million out. Locked, it is cut out and logged as the channel;
        // left as measured it is a different frequency every time it opens.
        let rate = 1_000_000.0;
        let center = Hz::mhz(434);
        let iq = keyed(rate, 356_000.0);
        let mut plain = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut plain, &[spec(rate, center)]).unwrap();
        let measured = openings(&mut plain, rate, center, &iq);
        assert!(measured.iter().any(|o| (o - 356_000.0).abs() < 5_000.0), "{measured:?}");
        assert!(!measured.iter().any(|o| (o - 350_000.0).abs() < 1.0), "not on the grid yet");

        let mut planned = AutoNode::new("auto", SourceConfig::default());
        planned.set_raster(Some((0.0, 25_000.0)));
        Node::negotiate(&mut planned, &[spec(rate, center)]).unwrap();
        let locked = openings(&mut planned, rate, center, &iq);
        assert!(locked.iter().any(|o| (o - 350_000.0).abs() < 1.0), "{locked:?}");

        // Half a channel off the grid is not on it, and stays as measured.
        let iq = keyed(rate, 362_500.0);
        let mut planned = AutoNode::new("auto", SourceConfig::default());
        planned.set_raster(Some((0.0, 25_000.0)));
        Node::negotiate(&mut planned, &[spec(rate, center)]).unwrap();
        let between = openings(&mut planned, rate, center, &iq);
        assert!(between.iter().any(|o| (o - 362_500.0).abs() < 5_000.0), "{between:?}");
        assert!(!between
            .iter()
            .any(|o| (o - 350_000.0).abs() < 1.0 || (o - 375_000.0).abs() < 1.0));
    }

    #[test]
    fn nothing_opens_on_the_tuner_s_own_centre() {
        // A direct-conversion receiver's offset follows a strong signal's
        // envelope, and that is a burst at the centre for as long as the
        // signal lasts. With the centre declared, a burst there while
        // another source is open is not a source.
        let rate = 1_000_000.0;
        let center = Hz::mhz(434);
        let mut n = AutoNode::new("auto", SourceConfig::default());
        n.set_spur(Some(center.as_f64()));
        Node::negotiate(&mut n, &[spec(rate, center)]).unwrap();
        let mut seed = 0x51u64;
        let mut iq: Vec<C32> = (0..600_000)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let a = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                let b = (seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5;
                C32::new(a * 0.05, b * 0.05)
            })
            .collect();
        // 100 ms keyed at the centre, and the same 350 kHz up: far enough
        // that the two are not taken for the tones of one transmitter.
        for i in 0..100_000usize {
            let on = (i / 500) % 2 == 0;
            if on {
                iq[300_000 + i] += C32::new(0.3, 0.0);
                let ph = std::f64::consts::TAU * 350_000.0 * i as f64 / rate;
                iq[300_000 + i] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
            }
        }
        let ins = [spec(rate, center)];
        let mut opened = Vec::new();
        for block in iq.chunks(16_384) {
            let input = Payload::Iq(block.to_vec());
            // Packets and speech: the node has a port for each.
            let mut out = [
                Payload::Packets(Vec::new()),
                Payload::Voice(Vec::new()),
                Payload::Video(Vec::new()),
            ];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(&mut n, &[&input], &mut out, &mut ctx).unwrap();
            opened.extend(events.iter().filter_map(|e| match e {
                Event::Detection { center: c, .. } => Some(c.as_f64() - center.as_f64()),
                _ => None,
            }));
        }
        assert!(
            opened.iter().any(|o| (o - 350_000.0).abs() < 10_000.0),
            "the real one opened: {opened:?}"
        );
        assert!(!opened.iter().any(|o| o.abs() < 10_000.0), "the spur opened: {opened:?}");
    }

    #[test]
    fn a_source_at_the_extraction_floor_still_gets_the_channel_decoders() {
        // A clean 12.5 kHz transmission measures a few kilohertz across at
        // the detector's 20 dB extent, so its extraction lands on the
        // 25 kHz floor. That is wide enough to hold the channel, and every
        // narrowband decoder has to be built for it.
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(433))]).unwrap();
        let b = SourceBlock {
            id: common::SourceId(1),
            state: SourceState::Opened,
            center_hz: 433_475_000,
            // The two-bin minimum the detector can report, which is what a
            // clean 12.5 kHz channel measures at its 20 dB extent.
            bandwidth_hz: 4_000.0,
            signal_hz: 4_000.0,
            rate: n.cfg.min_rate_hz,
            start_sample: 0,
            snr_db: 20.0,
            samples: Vec::new(),
        };
        let slot = n.open(&b, None).unwrap();
        let names: Vec<&str> = slot.members.iter().map(|m| m.name).collect();
        assert!(names.contains(&"m17"), "{names:?}");
        assert!(names.contains(&"pocsag"), "{names:?}");
    }

    /// The burst router is placed where its width fits and nowhere else.
    ///
    /// On a 2.4 GHz span the detector opens a source megahertz wide for the
    /// Wi-Fi in the band, and the router used to run its per-sample gate
    /// over all of it and classify every frame at milliseconds each, for a
    /// verdict nothing could read: the pulse front ends refuse a burst that
    /// wide and the only decoder that waits for a verdict reads chirp
    /// channels. What a source that wide leaves instead is the detector's
    /// own measurement.
    #[test]
    fn the_burst_router_goes_on_a_source_its_consumers_could_read() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(20e6, Hz(2_462_000_000))]).unwrap();
        let source = |id: u64, width_hz: f64| SourceBlock {
            id: common::SourceId(id),
            state: SourceState::Opened,
            center_hz: 2_463_000_000,
            bandwidth_hz: width_hz,
            signal_hz: width_hz / 1.5,
            rate: (width_hz * 2.5).max(n.cfg.min_rate_hz),
            start_sample: 0,
            snr_db: 20.0,
            samples: Vec::new(),
        };
        let wide = n.open(&source(1, 6.5e6), None).unwrap();
        assert!(wide.members.iter().all(|m| m.router.is_none()), "a 6.5 MHz source was classified");
        assert!(wide.evidence.is_some(), "and left no evidence of itself");

        // An ExpressLRS channel visit measures over a megahertz and must
        // keep its verdict: that is what places the decoder.
        let elrs = n.open(&source(2, 1.4e6), None).unwrap();
        assert!(elrs.members.iter().any(|m| m.router.is_some()), "a chirp channel");
        assert!(elrs.evidence.is_none(), "the classifier is the evidence here");

        let sensor = n.open(&source(3, 40e3), None).unwrap();
        assert!(sensor.members.iter().any(|m| m.router.is_some()), "a sensor channel");
    }

    #[test]
    fn a_remembered_channel_belongs_to_its_front_end_alone() {
        // Once a front end has read a channel, that channel is its: the
        // detector's openings inside it are dropped, whatever width they
        // measure, so no other decoder is built there and the same burst is
        // not logged twice. A wide measurement of the same transmitter used
        // to slip past the width tolerance and bring every narrowband
        // decoder with it.
        //
        // The classifier is the exception and rides along. The detector is
        // locked out of the channel, so it is the only thing that can
        // notice a second transmitter sharing the frequency, which is what
        // two LoRa networks at different bandwidths are.
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(433))]).unwrap();
        assert!(n.remembered().is_empty());
        n.remember("pocsag", 433_475_000.0, 25_000.0);
        assert_eq!(n.remembered(), [("pocsag", 433_475_000.0, 25_000.0)]);
        // And the slot built for it holds that front end and nothing else.
        let b = SourceBlock {
            id: n.memory.channels()[0].id,
            state: SourceState::Opened,
            center_hz: 433_475_000,
            bandwidth_hz: 25_000.0,
            signal_hz: 25_000.0,
            rate: n.cfg.min_rate_hz,
            start_sample: 0,
            snr_db: 20.0,
            samples: Vec::new(),
        };
        let slot = n.open(&b, None).unwrap();
        let names: Vec<&str> = slot.members.iter().map(|m| m.name).collect();
        assert_eq!(
            names,
            ["pocsag", "burst_route"],
            "a locked channel runs its front end and the classifier"
        );
    }

    /// Two networks on one frequency at different bandwidths both get a
    /// decoder, even after one of them has locked the channel.
    ///
    /// Meshtastic on a 250 kHz LoRa channel and MeshCore on 125 kHz in the
    /// same place: the 250 kHz demodulator dechirps nothing of the narrower
    /// one, the detector is locked out of a remembered channel, so without
    /// the classifier riding along the second network is invisible for the
    /// rest of the session. The width it measures is what places the
    /// decoder; the channel's own width is the wrong answer here.
    #[test]
    fn a_second_bandwidth_on_a_remembered_channel_gets_its_own_decoder() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(869))]).unwrap();
        n.remember("lora", 869_525_000.0, 250_000.0);
        let b = SourceBlock {
            id: n.memory.channels()[0].id,
            state: SourceState::Opened,
            center_hz: 869_525_000,
            bandwidth_hz: 250_000.0,
            signal_hz: 250_000.0,
            rate: 1_000_000.0,
            start_sample: 0,
            snr_db: 20.0,
            samples: Vec::new(),
        };
        let slot = n.open(&b, None).unwrap();
        n.slots.push(slot);
        let router = n.slots[0]
            .members
            .iter_mut()
            .find(|m| m.router.is_some())
            .expect("a remembered channel keeps the classifier");
        router.verdicts.push((dsp::Modulation::Chirp, 125_000.0));
        n.place_on_verdict(0, false);
        let placed: Vec<f64> =
            n.slots[0].members.iter().filter(|m| m.name == "lora").map(|m| m.channel_hz).collect();
        assert!(placed.contains(&125_000.0), "placed {placed:?}");
        assert!(placed.contains(&250_000.0), "the remembered channel went: {placed:?}");
    }

    /// A channel kept with a hold is given back once nothing has decoded
    /// on it for that long: the detector may open there again, and the
    /// decoder that was reading it is gone.
    #[test]
    fn a_remembered_channel_with_a_hold_is_forgotten_when_it_goes_quiet() {
        let rate = 1_000_000.0;
        let center = Hz::mhz(434);
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(rate, center)]).unwrap();
        n.remember_for("pocsag", 434_100_000.0, 25_000.0, Some(0.5), None, Default::default());
        assert_eq!(n.remembered().len(), 1);
        let iq = keyed(rate, 356_000.0);
        let ins = [spec(rate, center)];
        let mut forgotten_at = None;
        for (i, block) in iq.chunks(16_384).enumerate() {
            let input = Payload::Iq(block.to_vec());
            let mut out = [
                Payload::Packets(Vec::new()),
                Payload::Voice(Vec::new()),
                Payload::Video(Vec::new()),
            ];
            let (mut events, mut tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
            Node::process(&mut n, &[&input], &mut out, &mut ctx).unwrap();
            if n.remembered().is_empty() && forgotten_at.is_none() {
                forgotten_at = Some(i as f64 * 16_384.0 / rate);
            }
        }
        let at = forgotten_at.expect("the channel was kept forever");
        assert!(at > 0.45 && at < 1.0, "forgotten at {at} s");
        assert!(
            n.slots.iter().all(|s| s.id.0 < STICKY_ID_BASE),
            "the decoder on it is still running"
        );
        // And one kept for the session is still there.
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(rate, center)]).unwrap();
        n.remember("pocsag", 434_100_000.0, 25_000.0);
        openings(&mut n, rate, center, &iq);
        assert_eq!(n.remembered().len(), 1);
    }

    #[test]
    fn the_span_wide_decoders_run_where_the_span_reaches_them() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(1090))]).unwrap();
        assert_eq!(n.wide(), ["mode_s"]);
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(162))]).unwrap();
        assert_eq!(n.wide(), ["ais"]);
        Node::negotiate(&mut n, &[spec(250_000.0, Hz::mhz(1090))]).unwrap();
        assert!(n.wide().is_empty(), "Mode S needs 2 MS/s");
        // Bluetooth advertising is one of these and not a scanner block:
        // the three channels are where the standard put them, and this is
        // the only thing that places the front end on them.
        Node::negotiate(&mut n, &[spec(20_000_000.0, Hz::mhz(2426))]).unwrap();
        assert_eq!(n.wide(), ["ble"]);
        // And it owns its channel from the moment the span reaches it,
        // rather than after something decodes there: the spectrum draws it
        // as a locked channel and the detector stays out of it.
        let locked = n.locked_channels();
        assert_eq!(locked.len(), 1, "{locked:?}");
        assert_eq!(locked[0].0, "ble");
        assert!((locked[0].1 - 2_426_000_000.0).abs() < 1.0, "{locked:?}");
        Node::negotiate(&mut n, &[spec(20_000_000.0, Hz::mhz(2450))]).unwrap();
        assert!(n.wide().is_empty(), "no advertising channel inside that span");
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(2426))]).unwrap();
        assert!(n.wide().is_empty(), "BLE needs 4 MS/s");
    }
    /// A camera owns the span while it is reading a picture. Before this the
    /// detector opened the pieces of the carrier as sources and every front
    /// end ran on each of them, which cost more than the camera did.
    #[test]
    fn a_front_end_that_claims_a_band_keeps_it() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(20e6, Hz::mhz(5865))]).unwrap();
        assert!(
            n.locked_channels().iter().all(|(name, ..)| *name != "video"),
            "the span was claimed before anything was being read"
        );
        // Nothing is claimed until a front end says it is reading something,
        // and what it says is taken as it comes: the auto node answers the
        // same request from every front end and knows nothing about video.
        assert!(!n.claimed_whole_span());
        let mut said = Vec::new();
        let left = n.answer(
            None,
            "video",
            Request::Claim { lo_hz: 5_855_000_000.0, hi_hz: 5_875_000_000.0 },
            &mut said,
        );
        assert!(left.is_none(), "a claim is the auto node's to answer");
        let owned = n.locked_channels();
        let (_, hz, w) =
            owned.iter().find(|(name, ..)| *name == "video").expect("the claim was not taken");
        assert!((hz - 5_865_000_000.0).abs() < 1.0, "{hz}");
        assert!((w - 20e6).abs() < 1.0, "{w}");
        // And it is kept: a picture fades and comes back, and a claim that
        // followed the signal would hand the band to the detector between
        // every field.
        assert!(n.claimed_whole_span(), "a claim over the span leaves nothing to detect");
        // What needs the dial is handed back.
        let left = n.answer(None, "video", Request::Retune { center_hz: 1e9 }, &mut said);
        assert_eq!(left, Some(Request::Retune { center_hz: 1e9 }));
    }

    /// A decoder on a source can ask for a channel beside it, and the
    /// channel is remembered for the protocol it named, tied to the asker:
    /// a trunked control channel sending a call to a traffic carrier is the
    /// case, and the traffic channel goes when the control channel is
    /// forgotten.
    #[test]
    fn a_decoder_can_ask_for_a_side_channel_and_it_goes_with_the_asker() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(395))]).unwrap();
        n.remember_for("tetra", 395_100_000.0, 25_000.0, Some(1.0), None, Default::default());
        let b = SourceBlock {
            id: n.memory.channels()[0].id,
            state: SourceState::Opened,
            center_hz: 395_100_000,
            bandwidth_hz: 25_000.0,
            signal_hz: 25_000.0,
            rate: n.cfg.min_rate_hz,
            start_sample: 0,
            snr_db: 20.0,
            samples: Vec::new(),
        };
        let slot = n.open(&b, None).unwrap();
        n.slots.push(slot);
        let mut said = Vec::new();
        let ask = Request::OpenChannel {
            protocol: "tetra".into(),
            center_hz: 395_300_000.0,
            width_hz: 25_000.0,
            role: "traffic".into(),
            hold_s: Some(30.0),
            settings: Default::default(),
        };
        assert!(n.answer(Some(0), "tetra", ask, &mut said).is_none());
        let at: Vec<f64> = n.remembered().into_iter().map(|(_, hz, _)| hz).collect();
        assert_eq!(at, [395_100_000.0, 395_300_000.0]);
        // Outside the span it is not this node's to open.
        let far = Request::OpenChannel {
            protocol: "tetra".into(),
            center_hz: 420_000_000.0,
            width_hz: 25_000.0,
            role: "traffic".into(),
            hold_s: None,
            settings: Default::default(),
        };
        assert_eq!(n.answer(Some(0), "tetra", far.clone(), &mut said), Some(far));
        // The parent goes, and the traffic channel with it.
        let parent = n.memory.channels()[0].id;
        n.forget(&[parent]);
        assert!(n.remembered().is_empty(), "{:?}", n.remembered());
    }

    /// What the asker said the decoder needs reaches it, with where its
    /// stream sits in the span: a GSM carrier a beacon sent a phone to is
    /// built timed from the beacon.
    #[test]
    fn a_channel_asked_for_is_built_with_what_the_asker_said() {
        let mut n = AutoNode::new("auto", SourceConfig::default());
        Node::negotiate(&mut n, &[spec(2_400_000.0, Hz::mhz(947))]).unwrap();
        let mut said = Vec::new();
        let mut settings = Settings::new();
        settings.insert("timeslot".into(), ParamValue::Int(1));
        settings.insert("anchor_span_sample".into(), ParamValue::Float(12_345.0));
        settings.insert("anchor_frame".into(), ParamValue::Int(100));
        settings.insert("tsc".into(), ParamValue::Int(6));
        let ask = Request::OpenChannel {
            protocol: "gsm".into(),
            center_hz: 947_800_000.0,
            width_hz: 200_000.0,
            role: "SDCCH/8".into(),
            hold_s: Some(60.0),
            settings,
        };
        assert!(n.answer(None, "gsm", ask, &mut said).is_none());
        let b = SourceBlock {
            id: n.memory.channels()[0].id,
            state: SourceState::Opened,
            center_hz: 947_800_000,
            bandwidth_hz: 200_000.0,
            signal_hz: 200_000.0,
            rate: 1_200_000.0,
            start_sample: 50_000,
            snr_db: 20.0,
            samples: Vec::new(),
        };
        let slot = n.open(&b, None).unwrap();
        let m = &slot.members[0];
        let gsm = m
            .graph
            .order()
            .filter_map(|(id, _)| m.graph.node(id))
            .map(|node| node.as_any())
            .find_map(|a| a.downcast_ref::<crate::gsm_nodes::GsmNode>())
            .expect("a gsm node");
        assert!(gsm.anchored(), "the beacon's timing never reached it");
    }
}

/// The setting names this stage reads beyond the ones every watcher takes.
const LABEL: &str = "label";

/// What the box is called when a description does not name it.
const DEFAULT_LABEL: &str = "Auto";
const BANK_CHANNEL_HZ: &str = "bank_channel_hz";
const BANK_MIN_CHANNELS: &str = "bank_min_channels";

pub const DESC: StageDesc = StageDesc {
    name: "auto",
    summary: "Find and decode everything in the span on its own: sources \
              wherever something transmits, and the span-wide decoders \
              where the span reaches them",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn Node>> {
    let mut cfg = crate::source_nodes::watch_config(
        s,
        SourceConfig { open_db: AUTO_OPEN_DB, ..Default::default() },
    );
    cfg.bank_channel_hz = s.f64_or(BANK_CHANNEL_HZ, cfg.bank_channel_hz);
    cfg.bank_min_channels = s.f64_or(BANK_MIN_CHANNELS, cfg.bank_min_channels as f64) as usize;
    let mut n = AutoNode::new(s.str_or(LABEL, DEFAULT_LABEL), cfg);
    Node::configure(&mut n, s);
    Ok(Box::new(n))
}
