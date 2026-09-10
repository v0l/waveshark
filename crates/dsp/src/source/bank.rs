//! One polyphase bank over the span, feeding every narrow source at once.

use super::SourceConfig;
use crate::channelizer::Channelizer;
use common::C32;

/// Extracting a source straight from the wideband ring costs a mixer and a
/// coarse filter at the input rate, per source: at 16 MS/s that measured 7%
/// of a core for a 25 kHz sensor and 20% for a LoRa channel, and thirty
/// bursting devices on a busy band were three cores before a single front
/// end ran. The bank pays the input rate once and hands each source the one
/// channel that holds it, at the channel rate, which at 16 MS/s is 32 times
/// less to mix and filter per source.
///
/// A source across the edge between two channels is read from both: the
/// 2x oversampled channels cross at -6 dB with complementary roll-offs, so
/// the sum, with the upper channel shifted down a spacing, is flat through
/// the edge ([`Channelizer::pair_rotation`]). With that every source up to
/// a channel wide is served from the bank, and only a wider one goes to the
/// ring.
///
/// It runs only while something needs it. The lead-in a new source wants is
/// a few milliseconds, so the bank is caught up from the wideband ring the
/// moment a source is placed on it, streams while any source reads from it,
/// and stops when the last one closes: an empty band pays nothing.
pub(super) struct Bank {
    pub(super) chan: Channelizer,
    pub(super) m: usize,
    /// Input samples per frame, and the prototype's group delay in input
    /// samples: frame `f` is centred on input sample `(f + 1) * adv - 1 - delay`.
    pub(super) adv: u64,
    pub(super) delay: u64,
    /// Every channel's output at every frame kept, a circular store of
    /// `cap` rows of `m`: frame `f` is at row `f % cap`. Frames `base..head`
    /// are valid. Sized to reach as far back as the wideband ring does, so
    /// any source the ring could serve the bank can too; a fixed store
    /// rather than a growing one because shifting tens of megabytes down
    /// when it filled stalled the block it happened in.
    pub(super) frames: Vec<C32>,
    pub(super) cap: usize,
    pub(super) base: u64,
    pub(super) head: u64,
    /// Whether the bank is streaming, and the input sample its stream is
    /// indexed from, a multiple of `adv` so its frames line up with the
    /// wideband count.
    pub(super) running: bool,
    pub(super) origin: u64,
    /// Input samples since a source last read from the bank.
    pub(super) idle: u64,
    /// Microseconds spent streaming and catching up in the current block,
    /// for the node above to report.
    pub(super) feed_us: u64,
    pub(super) start_us: u64,
    pub(super) rate: f64,
    /// Channel spacing, and how far from a channel's centre a signal reads
    /// flat. A source within `flat` of one centre is read from that channel;
    /// one across the edge between two is read from both, which reads flat
    /// from `-flat` below the lower centre to `flat` above the upper.
    pub(super) spacing: f64,
    pub(super) flat: f64,
    pub(super) scratch: Vec<C32>,
}

/// Channel width the bank aims for.
///
/// A source is read from a channel only when the whole of its extraction
/// sits inside that channel's flat half width, so the wider the channel the
/// more sources qualify: at a megahertz nearly every sensor, pager and voice
/// channel does, most LoRa channels do, and only a source across a channel
/// edge or wider than half a channel goes to the wideband ring. The price
/// is the channel rate the source is then mixed and filtered from, two
/// megasamples here against sixteen from the ring, which is still the bulk
/// of the saving; below that the second stage is the same one the ring path
/// runs.
pub const BANK_CHANNEL_HZ: f64 = 1_000_000.0;
/// Below this many channels the direct path costs so little that the bank
/// is not worth its fixed cost.
pub const BANK_MIN_CHANNELS: usize = 8;
/// Taps per branch. Sets the transition width and so how much of each
/// channel reads flat, not the stopband, which the Kaiser window fixes at
/// the attenuation asked for. Eight leaves about half of each channel flat
/// at 16 MS/s and costs half what sixteen did per input sample; a source in
/// the other half is read from the pair, which costs a second gather and
/// nothing else.
const BANK_TAPS: usize = 8;
/// Seconds with nothing reading before the bank stops. Catching up again
/// costs a block's worth of channelizing, so on a band where something keys
/// up every second or so the bank should simply stay running.
pub(super) const BANK_IDLE_S: f64 = 2.0;

impl Bank {
    pub(super) fn new(rate: f64, cfg: &SourceConfig, keep_samples: usize) -> Option<Self> {
        if cfg.bank_min_channels == 0 || cfg.bank_channel_hz <= 0.0 {
            return None;
        }
        let m = ((rate / cfg.bank_channel_hz).log2().round() as i32).max(0);
        let m = (1usize << m).clamp(2, 1024);
        if m < cfg.bank_min_channels.max(2) {
            return None;
        }
        let atten_db = cfg.atten_db;
        let chan = Channelizer::new(m, BANK_TAPS, atten_db);
        let adv = chan.advance() as u64;
        Some(Self {
            delay: chan.latency_samples() as u64,
            adv,
            m,
            cap: keep_samples / adv as usize + 2048,
            frames: Vec::new(),
            base: 0,
            head: 0,
            running: false,
            origin: 0,
            idle: 0,
            feed_us: 0,
            start_us: 0,
            rate: chan.channel_rate(rate),
            spacing: chan.channel_bandwidth(rate),
            flat: chan.flat_half_width_hz(rate),
            scratch: Vec::new(),
            chan,
        })
    }

    /// The frame centred nearest wideband sample `x`.
    pub(super) fn frame_at(&self, x: u64) -> u64 {
        (x + self.delay) / self.adv
    }

    /// Wideband sample frame `f` is centred on.
    pub(super) fn sample_at(&self, f: u64) -> u64 {
        ((f + 1) * self.adv).saturating_sub(1 + self.delay)
    }

    pub(super) fn end(&self) -> u64 {
        self.head
    }

    pub(super) fn row(&self, f: u64) -> usize {
        (f % self.cap as u64) as usize * self.m
    }

    /// Feed input indexed from wideband sample `at` and keep the frames.
    pub(super) fn feed(&mut self, input: &[C32], at: u64) {
        debug_assert_eq!(at, self.origin + self.chan_pos());
        let t = std::time::Instant::now();
        let n = self.chan.process_parallel(input, &mut self.scratch);
        self.feed_us += t.elapsed().as_micros() as u64;
        if n == 0 {
            return;
        }
        if self.frames.is_empty() {
            self.frames.resize(self.cap * self.m, C32::new(0.0, 0.0));
        }
        if self.head == self.base {
            // The channelizer counts frames from its origin; the first one
            // out covers the origin's first `adv` samples.
            let first = self.origin / self.adv + (self.chan_pos() - input.len() as u64) / self.adv;
            self.base = first;
            self.head = first;
        }
        for (k, frame) in self.scratch.chunks_exact(self.m).enumerate() {
            let row = self.row(self.head + k as u64);
            self.frames[row..row + self.m].copy_from_slice(frame);
        }
        self.head += n as u64;
        self.base = self.base.max(self.head.saturating_sub(self.cap as u64));
    }

    fn chan_pos(&self) -> u64 {
        self.chan.stream_position()
    }

    /// Start streaming so that frames from wideband sample `from` on exist,
    /// caught up from the ring.
    pub(super) fn start(&mut self, ring: &[C32], ring_base: u64, from: u64) {
        let origin = (from / self.adv) * self.adv;
        let origin = origin.max((ring_base / self.adv + 1) * self.adv);
        self.chan.reset();
        self.base = 0;
        self.head = 0;
        self.origin = origin;
        self.running = true;
        self.idle = 0;
        let off = (origin - ring_base) as usize;
        if off < ring.len() {
            let t = std::time::Instant::now();
            let input = &ring[off..];
            self.feed(input, origin);
            self.start_us += t.elapsed().as_micros() as u64;
        }
    }
}

pub(super) fn bank_channel(bank: &Bank, m: usize, pair: bool, a: u64, b: u64, out: &mut Vec<C32>) {
    let a = a.max(bank.base);
    let b = b.min(bank.end());
    if b <= a {
        return;
    }
    out.reserve((b - a) as usize);
    if !pair {
        out.extend((a..b).map(|f| bank.frames[bank.row(f) + m]));
        return;
    }
    let upper = (m + 1) % bank.m;
    // The rotation depends on the frame count since the bank's reset, which
    // is the frame index less the frame the origin sits at.
    let origin_frame = bank.origin / bank.adv;
    let rot = [bank.chan.pair_rotation(0), bank.chan.pair_rotation(1)];
    for f in a..b {
        let row = bank.row(f);
        let r = rot[((f - origin_frame) % 2) as usize];
        out.push(bank.frames[row + m] + bank.frames[row + upper] * r);
    }
}
