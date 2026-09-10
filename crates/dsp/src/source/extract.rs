//! Cutting each source out: one mixer and two decimators per source, fed
//! either from the wideband ring or from the shared bank.

use super::bank::{bank_channel, Bank, BANK_IDLE_S};
use super::{Source, SourceConfig, SourceEvent};
use crate::fir::{self, FirDecim};
use crate::mixer::Mixer;
use common::{SourceBlock, SourceId, SourceState, C32};
use rayon::prelude::*;

/// [`design_stages`], with a cursor into what feeds it.
struct Chan {
    id: SourceId,
    center_hz: u64,
    bandwidth_hz: f64,
    signal_hz: f64,
    out_rate: f64,
    mixer: Mixer,
    /// Coarse stage, absent when the total decimation is small.
    coarse: Option<FirDecim>,
    fir: FirDecim,
    /// Where the samples come from: the wideband ring, or one channel of
    /// the shared bank.
    feed: Feed,
    /// Wideband index of the next sample to extract.
    cursor: u64,
    /// Wideband index to stop at, once the source has closed.
    end: Option<u64>,
    /// Closed because a wider stream took over: the last block says so.
    superseded: bool,
    opened: bool,
    /// What the detector measured for this source, or NaN for a channel the
    /// caller opened itself, which the detector never looked at.
    snr_db: f32,
    mixed: Vec<C32>,
    out: Vec<C32>,
}

/// What a source is cut out of.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Feed {
    /// The wideband ring: the mixer and the coarse stage run at the input
    /// rate. What every source did before the bank, and what a wide one or
    /// one straddling a channel edge still does.
    Direct,
    /// Channel `m` of the shared bank, at the bank's rate, summed with
    /// channel `m + 1` when the source lies across the edge between them.
    /// `next` is the bank frame to read next.
    Bank { m: usize, pair: bool, next: u64 },
}

impl Chan {
    /// Mix and decimate a run of wideband samples, appending to `out`.
    fn extract(&mut self, input: &[C32], out: &mut Vec<C32>) {
        self.mixed.clear();
        self.mixer.process(input, &mut self.mixed);
        match &mut self.coarse {
            Some(c) => {
                let mut mid = Vec::with_capacity(self.mixed.len() / c.factor() + 1);
                c.process(&self.mixed, &mut mid);
                self.fir.process(&mid, out);
            }
            None => self.fir.process(&self.mixed, out),
        }
    }
}

/// The two stages that bring a source from `rate` down to about `want`:
/// coarse by as much as leaves the sharp stage a few times the width to
/// work in, and only when there is enough decimation to share.
///
/// Two stages because one cannot be both cheap and sharp. The final filter
/// has to stop just past the signal's edge, or noise from there out to the
/// output Nyquist and its alias reach the demodulator: measured on the Fine
/// Offset recording that cost 5 dB against a channel bank, the difference
/// between decoding and not. A filter that sharp designed at the input rate
/// runs to tens of thousands of taps at 20 MS/s.
///
/// `floored` says the rate came from the floor rather than from the width,
/// and there the measurement is the wrong thing to filter to. What is
/// measured is the bins within `extent_db` of the peak, which for a clean
/// 12.5 kHz channel can be the two-bin minimum: a 2 kHz passband over a
/// 25 kHz stream cut the sidebands off an M17 transmission and left a
/// demodulator that could see the carrier and read nothing from it. When
/// the rate is at the floor the stream is wider than the signal asked for,
/// so it is filled.
fn design_stages(
    rate: f64,
    bw: f64,
    want: f64,
    floored: bool,
    atten_db: f64,
) -> (Option<FirDecim>, FirDecim, f64) {
    let total = ((rate / want).floor() as usize).max(1);
    let f1 = ((rate / (bw * 6.0)).floor() as usize).clamp(1, total);
    let (f1, f2) = if f1 >= 2 && total / f1 >= 1 { (f1, total / f1) } else { (1, total) };
    let rate1 = rate / f1 as f64;
    let out_rate = rate1 / f2 as f64;
    let pb = if floored { (out_rate * 0.4).max(bw / 2.0) } else { bw / 2.0 };
    let coarse = (f1 > 1).then(|| FirDecim::design_hz(rate, f1, pb, atten_db));
    (coarse, sharp_decimator(rate1, f2, pb, atten_db), out_rate)
}

/// A decimator whose stopband starts just past the passband, so the noise
/// beside a signal stops there too, rather than at the output Nyquist.
///
/// Bounded at 1024 taps: past that the transition is widened instead, which
/// trades a little noise for a filter that still runs.
fn sharp_decimator(rate: f64, factor: usize, passband_hz: f64, atten_db: f64) -> FirDecim {
    let out_rate = rate / factor as f64;
    let pb = passband_hz.min(out_rate * 0.45);
    let mut stop = (pb * 1.35).min(out_rate - pb).max(pb * 1.05);
    let mut taps = fir::estimate_taps(((stop - pb) / rate).max(1e-4), atten_db);
    if taps > 1024 {
        taps = 1024;
        // Roughly, taps scale with the reciprocal of the transition width.
        let transition = fir::estimate_taps(1e-4, atten_db) as f64 * 1e-4 / 1024.0;
        stop = (pb + transition * rate).min(out_rate - pb).max(pb * 1.05);
    }
    let cutoff = (pb + (stop - pb) * 0.5) / rate;
    FirDecim::new(fir::lowpass(taps, cutoff, atten_db), factor)
}

/// The wideband history every source is cut from.
///
/// A ring with a wrapping write index rather than a buffer that is drained
/// when it doubles. Draining moves everything still held down by whatever
/// was dropped, and what is held is the history a reopened source starts
/// again from: at 61.44 MS/s that is 18 million samples, so one block in
/// every hundred and forty spent 6 ms memmoving 148 MB, measured on the busy
/// span capture, in a block whose whole budget is 2.1 ms. Wrapping costs the
/// same per sample written and nothing per block.
pub(super) struct History {
    buf: Vec<C32>,
    /// Wideband index of the oldest sample held.
    base: u64,
    /// How many are held, which is the buffer's length once it has filled.
    len: usize,
    /// Where in the buffer the next sample goes.
    head: usize,
}

impl History {
    pub(super) fn new(cap: usize) -> Self {
        Self { buf: vec![C32::default(); cap.max(1)], base: 0, len: 0, head: 0 }
    }

    /// Wideband index of the oldest sample held.
    pub(super) fn first(&self) -> u64 {
        self.base
    }

    /// One past the newest.
    pub(super) fn end(&self) -> u64 {
        self.base + self.len as u64
    }

    pub(super) fn clear(&mut self) {
        self.base = 0;
        self.len = 0;
        self.head = 0;
    }

    pub(super) fn push(&mut self, input: &[C32]) {
        let cap = self.buf.len();
        // A block longer than the whole history: only the newest `cap` of it
        // can be held, and the rest is counted as having gone past.
        let (input, skipped) = match input.len() > cap {
            true => (&input[input.len() - cap..], (input.len() - cap) as u64),
            false => (input, 0),
        };
        self.base += skipped;
        // In two pieces where the write reaches the end of the buffer and
        // carries on at its start.
        let (head, tail) = input.split_at((cap - self.head).min(input.len()));
        for chunk in [head, tail] {
            if chunk.is_empty() {
                continue;
            }
            self.buf[self.head..self.head + chunk.len()].copy_from_slice(chunk);
            self.head = (self.head + chunk.len()) % cap;
        }
        let was = self.len;
        self.len = (self.len + input.len()).min(cap);
        // Whatever the write ran over is no longer held.
        self.base += (was + input.len() - self.len) as u64;
    }

    /// The two runs holding `[a, b)`, oldest first. The second is empty
    /// unless the range crosses the buffer's end.
    pub(super) fn parts(&self, a: u64, b: u64) -> (&[C32], &[C32]) {
        let cap = self.buf.len();
        let a = a.max(self.base);
        let b = b.min(self.end());
        if b <= a {
            return (&[], &[]);
        }
        let start = (self.head + cap - self.len) % cap;
        let from = (start + (a - self.base) as usize) % cap;
        let n = (b - a) as usize;
        match from + n <= cap {
            true => (&self.buf[from..from + n], &[]),
            false => (&self.buf[from..], &self.buf[..n - (cap - from)]),
        }
    }
}

/// Turns the detector's sources into streams, from a ring of the wideband
/// input.
pub struct SourceExtractor {
    cfg: SourceConfig,
    rate: f64,
    center_hz: f64,
    ring: History,
    lead: u64,
    tail: u64,
    chans: Vec<Chan>,
    /// Channels the caller asked for itself, waiting for the next block.
    commands: Vec<Command>,
    /// The shared bank narrow sources are cut from, on spans wide enough to
    /// need it.
    bank: Option<Bank>,
}

/// A channel the caller opened or closed itself, rather than one the
/// detector found.
///
/// Held until the next [`SourceExtractor::process`], so a channel asked for
/// between blocks lands in the same order, against the same ring, as the
/// detector's own events do.
enum Command {
    Open { id: SourceId, offset_hz: f64, width_hz: f64, from: u64 },
    Close(SourceId),
}

impl SourceExtractor {
    /// `center_hz` is the RF centre of the wideband stream, which is what
    /// the blocks are stamped relative to. `keep` is the detector's
    /// [`SourceDetector::latency_samples`], the furthest back an opening can
    /// refer to.
    pub fn new(rate: f64, center_hz: f64, keep: usize, cfg: SourceConfig) -> Self {
        let lead = (cfg.lead_us as f64 * 1e-6 * rate) as u64;
        let tail = (cfg.tail_us as f64 * 1e-6 * rate) as u64;
        // The detector's latency, plus room for the filters to be primed from
        // before the lead-in; a longer prime than this falls back on the
        // lead-in. And never less than the history a reopened source starts
        // again from. The bank reaches back as far, so any source the ring
        // could serve it can too.
        let keep = (keep + lead as usize + 4096).max((cfg.history_s * rate) as usize);
        Self {
            cfg,
            rate,
            center_hz,
            // Twice what a source can reach back for, which is what the
            // buffer this replaced held at its fullest: it grew to twice
            // `keep` and was then drained back to `keep`, so anything asked
            // of it got between one and two of them. A ring that held one
            // lost a decode of the source corpus.
            ring: History::new(2 * keep),
            lead,
            tail,
            chans: Vec::new(),
            commands: Vec::new(),
            bank: Bank::new(rate, &cfg, keep),
        }
    }

    /// Whether the shared bank is streaming, for tests and the view.
    pub fn bank_running(&self) -> bool {
        self.bank.as_ref().is_some_and(|b| b.running)
    }

    /// Microseconds the bank spent this block streaming, and catching up
    /// from the ring when a source started it. Reading resets both.
    pub fn take_bank_cost(&mut self) -> (u64, u64) {
        match &mut self.bank {
            Some(b) => (std::mem::take(&mut b.feed_us), std::mem::take(&mut b.start_us)),
            None => (0, 0),
        }
    }

    /// How many open sources are read from the bank rather than the ring.
    pub fn banked(&self) -> usize {
        self.chans.iter().filter(|c| matches!(c.feed, Feed::Bank { .. })).count()
    }

    /// Sources being extracted, open or draining their tail.
    pub fn active(&self) -> usize {
        self.chans.len()
    }

    /// Cut a channel out that no source opened: one the caller has decided
    /// to listen on whatever the detector makes of it. `from` is the
    /// wideband sample index to start at. Nothing measured its level, so
    /// its blocks carry no SNR.
    pub fn open_channel(&mut self, id: SourceId, center_hz: f64, width_hz: f64, from: u64) {
        self.commands.push(Command::Open {
            id,
            offset_hz: center_hz - self.center_hz,
            width_hz,
            from,
        });
    }

    /// Stop cutting a channel out. Its last block says it closed, as a
    /// source the detector closed would.
    pub fn close(&mut self, id: SourceId) {
        self.commands.push(Command::Close(id));
    }

    pub fn reset(&mut self) {
        self.ring.clear();
        self.chans.clear();
        self.commands.clear();
        if let Some(b) = &mut self.bank {
            b.running = false;
            b.base = 0;
            b.head = 0;
        }
    }

    /// Design an extraction for a source: its centre, and a rate that fits
    /// its width with room for the edges the detector did not see.
    fn open(&mut self, s: &Source) {
        self.cut(s.id, s.center_hz, s.bandwidth_hz(), s.start_sample, s.peak_snr_db);
    }

    /// Design an extraction at `offset_hz` from the stream's centre, `width_hz`
    /// wide, from wideband sample `from`.
    fn cut(&mut self, id: SourceId, offset_hz: f64, width_hz: f64, from: u64, snr_db: f32) {
        let bw = (width_hz * self.cfg.width_margin).max(self.cfg.bin_hz * 2.0);
        let want = (bw * self.cfg.oversample).max(self.cfg.min_rate_hz);
        // Whether the rate came from the width or from the floor, which
        // decides what the extraction filter should keep.
        let floored = bw * self.cfg.oversample < self.cfg.min_rate_hz;
        let start = from.saturating_sub(self.lead).max(self.ring.first());
        // How far back the stream starts is bounded by the work of reading
        // it, in samples of the stream rather than in time. A candidate can
        // sit unopened for as long as it is present, too wide or with no
        // room, and opening it from its birth handed a 20 MHz source's
        // front ends half a second of 15 MS/s in one block, 170 ms measured
        // on a 2.4 GHz capture. A million samples of the stream is half a
        // second for a sensor at 2 MS/s, which is its whole burst however
        // long it waited for room, and 66 ms of a Wi-Fi source, a hundred of
        // its frames. Bounding it by time instead cut the sensors: the busy
        // span test loses twelve of 116 packets at 20 ms of lead.
        let end = self.ring.end();
        let decim = (self.rate / (want.max(bw))).max(1.0) as u64;
        let start = start.max(end.saturating_sub(CATCH_UP_SAMPLES * decim));

        let (feed, rate, shift) = match self.bank_feed(offset_hz, bw, want, start) {
            Some(feed @ Feed::Bank { m, .. }) => {
                let bank = self.bank.as_ref().unwrap();
                (feed, bank.rate, offset_hz - bank.chan.channel_offset_hz(m, self.rate))
            }
            _ => (Feed::Direct, self.rate, offset_hz),
        };
        let (coarse, fir, out_rate) = design_stages(rate, bw, want, floored, self.cfg.atten_db);
        self.chans.push(Chan {
            id,
            center_hz: (self.center_hz + offset_hz).max(0.0) as u64,
            bandwidth_hz: bw.min(out_rate),
            signal_hz: width_hz,
            out_rate,
            mixer: Mixer::new(-shift, rate),
            coarse,
            fir,
            feed,
            cursor: start,
            end: None,
            superseded: false,
            opened: false,
            snr_db,
            mixed: Vec::new(),
            out: Vec::new(),
        });
    }

    /// The bank channel a source can be read from, starting the bank if it
    /// is not running, or nothing when the source needs the wideband ring:
    /// too wide for a channel, across a channel edge, wanting a rate the
    /// channel cannot give, or starting further back than the bank keeps.
    fn bank_feed(&mut self, offset_hz: f64, bw: f64, want: f64, start: u64) -> Option<Feed> {
        let bank = self.bank.as_mut()?;
        if want > bank.rate {
            return None;
        }
        let (lo, hi) = (offset_hz - bw / 2.0, offset_hz + bw / 2.0);
        let nearest = bank.chan.channel_for_offset(offset_hz, self.rate);
        let f_near = bank.chan.channel_offset_hz(nearest, self.rate);
        let (m, pair) = if lo >= f_near - bank.flat && hi <= f_near + bank.flat {
            (nearest, false)
        } else if bw <= bank.spacing {
            // Across an edge: the pair on the side the source leans to.
            let m = if offset_hz >= f_near { nearest } else { (nearest + bank.m - 1) % bank.m };
            let f_m = bank.chan.channel_offset_hz(m, self.rate);
            if lo < f_m - bank.flat || hi > f_m + bank.spacing + bank.flat {
                return None;
            }
            (m, true)
        } else {
            return None;
        };
        // Priming wants the filter's length of frames before the lead-in.
        let prime = fir::estimate_taps(1e-4, self.cfg.atten_db).min(1024) as u64;
        let first = bank.frame_at(start);
        let need = first.saturating_sub(prime);
        if bank.running {
            // Started before the store reaches: the ring has it.
            if need < bank.base {
                return None;
            }
        } else {
            let from = bank.sample_at(need).max(self.ring.first());
            if from < self.ring.first() + bank.delay {
                return None;
            }
            bank.start(&self.ring, from);
            if need < bank.base {
                return None;
            }
        }
        bank.idle = 0;
        Some(Feed::Bank { m, pair, next: first })
    }

    /// Append a block and the detector's verdict on it, and produce a block
    /// per source being extracted.
    ///
    /// `input` must be the same samples the detector was just given, in the
    /// same order, so the indices in its events land in this ring. The
    /// events must be the detector's own; a channel the caller wants for
    /// reasons of its own goes through [`Self::open_channel`] and
    /// [`Self::close`], which take effect here too.
    pub fn process(&mut self, input: &[C32], events: &[SourceEvent], out: &mut Vec<SourceBlock>) {
        self.ring.push(input);
        let end = self.ring.end();
        if let Some(b) = &mut self.bank {
            if b.running {
                b.feed(input, end - input.len() as u64);
            }
        }

        for e in events {
            match e {
                SourceEvent::Opened(s) => self.open(s),
                SourceEvent::Closed(s) => {
                    if let Some(c) = self.chans.iter_mut().find(|c| c.id == s.id) {
                        c.snr_db = s.peak_snr_db;
                        c.end = Some(s.end_sample.unwrap_or(end) + self.tail);
                    }
                }
                SourceEvent::Superseded(s) => {
                    // Ends now, with no tail: what follows belongs to the
                    // wider stream.
                    if let Some(c) = self.chans.iter_mut().find(|c| c.id == s.id) {
                        c.end = Some(c.cursor);
                        c.superseded = true;
                    }
                }
            }
        }
        for cmd in std::mem::take(&mut self.commands) {
            match cmd {
                Command::Open { id, offset_hz, width_hz, from } => {
                    self.cut(id, offset_hz, width_hz, from, f32::NAN)
                }
                Command::Close(id) => {
                    if let Some(c) = self.chans.iter_mut().find(|c| c.id == id) {
                        c.end = Some(end + self.tail);
                    }
                }
            }
        }

        let ring = &self.ring;
        let bank = self.bank.as_ref();
        let pace = (input.len() as u64 * CATCH_UP_PACE).max(1);
        let blocks: Vec<SourceBlock> = self
            .chans
            .par_iter_mut()
            .filter_map(|c| match c.feed {
                Feed::Direct => Self::extract_block(c, &Gather::Ring { ring }, c.cursor, end, pace),
                Feed::Bank { m, pair, next } => {
                    let b = bank?;
                    let g = Gather::Bank { bank: b, m, pair };
                    Self::extract_block(c, &g, next, end, (pace / b.adv).max(1))
                }
            })
            .collect();

        let closed: Vec<SourceId> = blocks
            .iter()
            .filter(|b| matches!(b.state, SourceState::Closed | SourceState::Superseded))
            .map(|b| b.id)
            .collect();
        self.chans.retain(|c| !closed.contains(&c.id));
        out.extend(blocks);

        if let Some(b) = &mut self.bank {
            if b.running {
                if self.chans.iter().any(|c| matches!(c.feed, Feed::Bank { .. })) {
                    b.idle = 0;
                } else {
                    b.idle += input.len() as u64;
                    if b.idle as f64 >= BANK_IDLE_S * self.rate {
                        b.running = false;
                        b.base = 0;
                        b.head = 0;
                    }
                }
            }
        }
    }

    /// One block of a source, from wherever its samples come from.
    ///
    /// `from` is the position it starts at, in whatever the feed counts:
    /// wideband samples for the ring, bank frames for the bank. The two paths
    /// were the same state machine written twice, and the second copy is how
    /// a source read from the bank came to be timed a block late.
    fn extract_block(
        c: &mut Chan,
        g: &Gather,
        from: u64,
        end: u64,
        pace: u64,
    ) -> Option<SourceBlock> {
        let avail = g.last(end);
        let from = from.max(g.first());
        // A stream behind the ring's end catches up at `pace` rather than
        // all at once. A source opened from history handed its front ends
        // the whole lead-in in one block, half a second of a 20 MHz span,
        // and that block ran at a twentieth of real time; paced, the same
        // history arrives over a few blocks and the stream is a few blocks
        // late for them, which a decoder cannot tell from a longer filter.
        let stop = c.end.map_or(avail, |e| g.at(e).min(avail)).min(from + pace);
        let state = if !c.opened {
            SourceState::Opened
        } else if c.end.is_some_and(|e| stop >= g.at(e)) {
            if c.superseded {
                SourceState::Superseded
            } else {
                SourceState::Closed
            }
        } else {
            SourceState::Running
        };
        c.out.clear();
        let mut scratch = Vec::new();
        if !c.opened {
            // Prime the filter so the stream does not open with its
            // transient. A fresh filter's first output is the first sample
            // through an empty history, near zero whatever the input, and a
            // gate downstream that seeds its noise estimate from the first
            // sample it sees then has a floor of nothing and opens on the
            // noise that follows. Whatever the feed holds before the lead-in
            // is real noise; failing that, the lead-in itself, twice.
            let taps = (c.fir.taps() * c.coarse.as_ref().map_or(1, |f| f.factor())
                + c.coarse.as_ref().map_or(0, |f| f.taps())) as u64;
            let (p0, p1) = if from > g.first() + taps {
                (from - taps, from)
            } else {
                (from, (from + taps).min(stop))
            };
            if p1 > p0 {
                let mut discard = Vec::new();
                c.extract(g.samples(p0, p1, &mut scratch), &mut discard);
            }
        }
        if stop > from {
            let mut out = std::mem::take(&mut c.out);
            let samples = g.samples(from, stop, &mut scratch);
            c.extract(samples, &mut out);
            c.out = out;
        }
        let start_sample = g.sample(from);
        g.advance(c, from, stop, end);
        if c.out.is_empty() && state == SourceState::Running {
            return None;
        }
        c.opened = true;
        Some(SourceBlock {
            id: c.id,
            state,
            center_hz: c.center_hz,
            bandwidth_hz: c.bandwidth_hz,
            signal_hz: c.signal_hz,
            rate: c.out_rate,
            start_sample,
            snr_db: c.snr_db,
            samples: std::mem::take(&mut c.out),
        })
    }
}

/// Where a channel's samples come from, and how positions in it relate to the
/// wideband sample count everything else here is indexed by.
/// The most samples of its own stream a source opens with from history.
const CATCH_UP_SAMPLES: u64 = 1 << 20;

/// How many blocks' worth of history a stream behind the ring's end reads
/// in one block. Eight: at four the busy span test loses one packet of 116
/// and at two it loses two, so a stream that lags is not read quite as it
/// would be live, and eight is the least that reads the same. With the
/// catch-up bounded above, eight is at most a million samples in a block.
const CATCH_UP_PACE: u64 = 8;

enum Gather<'a> {
    Ring { ring: &'a History },
    Bank { bank: &'a Bank, m: usize, pair: bool },
}

impl Gather<'_> {
    /// The oldest position still held.
    fn first(&self) -> u64 {
        match self {
            Gather::Ring { ring } => ring.first(),
            Gather::Bank { bank, .. } => bank.base,
        }
    }

    /// One past the newest position held, given the wideband end.
    fn last(&self, end: u64) -> u64 {
        match self {
            Gather::Ring { .. } => end,
            Gather::Bank { bank, .. } => bank.end(),
        }
    }

    /// The position wideband sample `x` falls at.
    fn at(&self, x: u64) -> u64 {
        match self {
            Gather::Ring { .. } => x,
            Gather::Bank { bank, .. } => bank.frame_at(x),
        }
    }

    /// The wideband sample a position sits on.
    fn sample(&self, pos: u64) -> u64 {
        match self {
            Gather::Ring { .. } => pos,
            Gather::Bank { bank, .. } => bank.sample_at(pos),
        }
    }

    /// Positions `[a, b)` as samples. The ring hands its own slice over; the
    /// bank has to be read channel by channel, into `scratch`.
    fn samples<'b>(&'b self, a: u64, b: u64, scratch: &'b mut Vec<C32>) -> &'b [C32] {
        match self {
            Gather::Ring { ring } => match ring.parts(a, b) {
                (run, []) => run,
                (head, tail) => {
                    // Only where the range crosses the buffer's end, which is
                    // one block in a history length, and only the range asked
                    // for rather than the whole ring.
                    scratch.clear();
                    scratch.extend_from_slice(head);
                    scratch.extend_from_slice(tail);
                    scratch
                }
            },
            Gather::Bank { bank, m, pair } => {
                scratch.clear();
                bank_channel(bank, *m, *pair, a, b, scratch);
                scratch
            }
        }
    }

    /// Put the channel's cursor where this block left off.
    fn advance(&self, c: &mut Chan, from: u64, stop: u64, end: u64) {
        match self {
            Gather::Ring { .. } => c.cursor = stop.max(c.cursor),
            Gather::Bank { bank, m, pair } => {
                let reached = stop.max(from);
                c.feed = Feed::Bank { m: *m, pair: *pair, next: reached };
                c.cursor = bank.sample_at(reached).min(end);
            }
        }
    }
}

#[cfg(test)]
mod history_tests {
    use super::History;
    use common::C32;

    fn run(from: u64, n: usize) -> Vec<C32> {
        (0..n).map(|i| C32::new((from as usize + i) as f32, 0.0)).collect()
    }

    /// Every sample read back out of the ring is the one that was written at
    /// that index, across as many wraps as it takes: an off-by-one here is a
    /// source cut from the wrong samples and no test downstream would say
    /// which.
    #[test]
    fn what_comes_out_is_what_went_in_wherever_the_write_wrapped() {
        let cap = 100usize;
        let mut h = History::new(cap);
        let mut written = 0u64;
        for block in [30usize, 30, 30, 30, 7, 55] {
            h.push(&run(written, block));
            written += block as u64;
            assert_eq!(h.end(), written);
            assert_eq!(h.first(), written.saturating_sub(cap as u64));
            let (a, b) = h.parts(h.first(), h.end());
            let got: Vec<f32> = a.iter().chain(b).map(|c| c.re).collect();
            let want: Vec<f32> = (h.first()..h.end()).map(|i| i as f32).collect();
            assert_eq!(got, want, "after {written} samples");
        }
        // A run asked for inside what is held, wrapped or not.
        let (a, b) = h.parts(h.end() - 10, h.end());
        let got: Vec<f32> = a.iter().chain(b).map(|c| c.re).collect();
        assert_eq!(got, ((written - 10)..written).map(|i| i as f32).collect::<Vec<_>>());
        // And a block longer than the whole ring keeps its newest `cap`.
        h.push(&run(written, 250));
        written += 250;
        assert_eq!(h.first(), written - cap as u64);
        let (a, b) = h.parts(h.first(), h.end());
        let got: Vec<f32> = a.iter().chain(b).map(|c| c.re).collect();
        assert_eq!(got, ((written - cap as u64)..written).map(|i| i as f32).collect::<Vec<_>>());
    }
}
