//! DMR as a graph node.
//!
//! The same shape as the M17 front end: the channel is narrowband FM, so the
//! node mixes it down, filters it and discriminates it. What comes out is
//! four-level FSK at 4800 baud, two-slot TDMA. This node recovers the symbol
//! clock with a Gardner loop that runs continuously across blocks, correlates
//! the 48-bit sync words (ETSI TS 102 361-1 clause 9.1.1) to find the burst
//! boundaries, and reads the three 72-bit AMBE frames a voice burst carries.
//!
//! The vocoder (`crates/mbe`) is behind the `ambe` feature, off by default,
//! because AMBE is patent-encumbered. Without it the node still finds the
//! transmission and reports the channel; with it, the AMBE frames become
//! 8 kHz speech on the voice bus.
//!
//! Only burst A of a voice superframe carries a sync word; the other five
//! carry an EMB field instead. So the framer locks a burst clock rather than
//! hunting for a sync each time: once a sync is found, the next burst is one
//! TDMA frame later, and each is confirmed by its own sync or by its EMB
//! passing the QR(16,7,6) check. Hunting per superframe threw a whole 360 ms
//! away whenever one sync was marginal, which split a single over into three
//! rows in the packet log.
//!
//! Who is talking comes from the link control (`decode::dmr`): the voice LC
//! header opens a transmission, the terminator closes it, and the embedded LC
//! spread across bursts B to E repeats it every superframe, so a receiver
//! that came in late still has the talkgroup and the radio ID within 360 ms.
//!
//! What is not here yet: slot 2 is not separated from slot 1, so the node
//! follows whichever slot it locks onto first.

use common::Result;
use decode::dmr::{self, LinkControl};
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// Tag byte identifying a packet body this node wrote, so the log labeler can
/// tell a DMR voice-over row from any other `Frame` and refuse everything
/// else. "DV" for DMR voice.
const DMR_TAG: [u8; 2] = *b"DV";

/// Body length: tag, burst count, flags, destination, source.
const BODY_LEN: usize = 2 + 4 + 1 + 4 + 4;

const FLAG_HAVE_LC: u8 = 0x01;
const FLAG_GROUP: u8 = 0x02;
const FLAG_ENCRYPTED: u8 = 0x04;
const FLAG_EMERGENCY: u8 = 0x08;

/// Serialise a finished voice over: the tag, the burst count (each burst is
/// one 60 ms slot of speech) and the link control if one was heard.
fn encode_voice_over(bursts: u32, lc: Option<LinkControl>) -> Vec<u8> {
    let mut v = DMR_TAG.to_vec();
    v.extend_from_slice(&bursts.to_be_bytes());
    let mut flags = 0u8;
    if let Some(lc) = lc {
        flags |= FLAG_HAVE_LC;
        if lc.group() {
            flags |= FLAG_GROUP;
        }
        if lc.encrypted() {
            flags |= FLAG_ENCRYPTED;
        }
        if lc.emergency() {
            flags |= FLAG_EMERGENCY;
        }
    }
    v.push(flags);
    v.extend_from_slice(&lc.map_or(0, |l| l.dst).to_be_bytes());
    v.extend_from_slice(&lc.map_or(0, |l| l.src).to_be_bytes());
    v
}

/// Recognise and describe a DMR voice-over row for the packet log. Returns
/// `None` for anything this node did not write, so it is safe to try on every
/// frame the way `m17_decoded` is.
///
/// A row with a link control names its talkgroup and radio, and says `voice`,
/// which is what puts it in the call list rather than only in the log.
pub fn dmr_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if bytes.len() != BODY_LEN || bytes[..2] != DMR_TAG {
        return None;
    }
    let bursts = u32::from_be_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
    let flags = bytes[6];
    let dst = u32::from_be_bytes([bytes[7], bytes[8], bytes[9], bytes[10]]);
    let src = u32::from_be_bytes([bytes[11], bytes[12], bytes[13], bytes[14]]);
    // One burst is one 60 ms slot on this logical channel.
    let seconds = f64::from(bursts) * 0.06;
    let mut fields = vec![
        ("seconds".to_string(), Value::Float(seconds)),
        ("bursts".to_string(), Value::Int(i64::from(bursts))),
    ];
    if flags & FLAG_HAVE_LC != 0 {
        let group = flags & FLAG_GROUP != 0;
        fields.push(("voice".to_string(), Value::Bool(true)));
        fields.push(("to".to_string(), Value::Text(dst.to_string())));
        fields.push(("from".to_string(), Value::Text(src.to_string())));
        fields.push((
            "call_type".to_string(),
            Value::Text(if group { "group" } else { "private" }.to_string()),
        ));
        if flags & FLAG_ENCRYPTED != 0 {
            fields.push(("encrypted".to_string(), Value::Bool(true)));
            fields.push(("encryption".to_string(), Value::Text("privacy".to_string())));
        }
        if flags & FLAG_EMERGENCY != 0 {
            fields.push(("emergency".to_string(), Value::Bool(true)));
        }
    }
    let detail = fields
        .iter()
        .map(|(k, v)| format!("{k}={v}"))
        .collect::<Vec<_>>()
        .join(" ");
    Some(
        Decoded::bytes("DMR-Voice", center, 0.0, bytes.to_vec())
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("4FSK"),
    )
}

/// A common DMR simplex frequency in Region 1, and only the default before the
/// scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 433_450_000.0;

/// 12.5 kHz channel grid.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// One-sided filter cutoff. A compliant DMR signal is ~9.5 kHz wide (±4.75),
/// but handsets over-deviate badly (a DM-1701 measured ±6.3 kHz outer
/// levels, ~14 kHz occupied), so pass well past nominal or the outer symbols
/// are clipped and the four-level eye closes.
const FILTER_CUTOFF_HZ: f64 = 10_000.0;

/// Discriminator output rate: ~10 samples per symbol at 4800 baud.
const AUDIO_HZ: f64 = 48_000.0;

/// Symbol rate.
const BAUD: f64 = 4_800.0;

/// Roll-off of the root raised cosine DMR transmits with, and so of the
/// matched filter here (TS 102 361-1 clause 6.2.1).
///
/// It is not optional. Without it the sync words still correlate, because
/// they use only the outer two levels, but the inner levels never separate:
/// on the corpus capture every BPTC(196,96) in the file failed with nine bad
/// rows out of nine, and with the filter the same bursts come out clean. A
/// receiver without it finds transmissions and can say nothing about them.
const RRC_ALPHA: f64 = 0.2;

/// Vocoder output rate.
pub const VOICE_HZ: f64 = 8_000.0;

/// Nominal outer-symbol deviation. Nothing downstream depends on the exact
/// value: the slicer fits its own levels per burst.
const DEVIATION_HZ: f64 = 1_944.0;

/// A burst is 108 payload + 48 sync/embedded + 108 payload bits, which at two
/// bits a symbol is 54 + 24 + 54 = 132 symbols.
const SYM_PAYLOAD: usize = 54;
const SYM_SYNC: usize = 24;
const SYM_BURST: usize = SYM_PAYLOAD + SYM_SYNC + SYM_PAYLOAD;

/// Bursts on one timeslot are 288 symbols apart (60 ms, one two-slot TDMA
/// frame). A voice superframe is six of them.
const SLOT_STRIDE: usize = 288;
const SUPERFRAME_BURSTS: usize = 6;

/// How far either side of the expected burst position to look when locked.
/// The Gardner loop holds the symbol clock; this absorbs the symbol or two a
/// re-lock after fading can be out by.
const REANCHOR: usize = 2;

/// Bursts that pass no check before the clock is abandoned and the sync hunt
/// starts again. Six is one superframe, long enough to ride through a fade
/// that would otherwise end the over.
const MAX_MISSES: u32 = 8;

/// Longest transmission kept as audio, in seconds.
#[cfg(feature = "ambe")]
const MAX_VOICE_SECONDS: f64 = 120.0;

/// Voice bursts an over needs before it is worth a row, unless a link control
/// was read. A real transmission is at least one superframe of six; a stray
/// burst is the EMB check passing on noise, which it does a few times in a
/// hundred, and each one used to reach the packet log as an over of 60 ms.
const MIN_BURSTS: u32 = 3;

/// The DMR sync words as level-index strings (0=-3,1=-1,2=+1,3=+3), derived
/// from the canonical hex by mapping each dibit 01,00,10,11. Voice bursts and
/// data bursts carry different words, which is how a voice superframe is told
/// from signalling.
///
/// Beware: each voice word is the exact inverse of its data word (invert
/// `MS_voice` symbol by symbol and `MS_data` is what comes out). A
/// discriminator whose sign is unknown therefore cannot tell a voice burst
/// from a data burst by the sync alone, and picking the wrong one locks the
/// framer onto a transmission it then reads as signalling that never
/// decodes. `Framer::confirm_voice` is what settles it.
const SYNCS: [(&str, &str, bool); 6] = [
    ("BS_voice", "303333000330030030330030", true),
    ("BS_data", "030000333003303303003303", false),
    ("MS_voice", "300030033303033330030003", true),
    ("MS_data", "033303300030300003303330", false),
    ("T1_voice", "330333303000303033300000", true),
    ("T2_voice", "300300000333003333033300", true),
];

/// Streaming Gardner symbol-timing recovery on the discriminator output.
///
/// Runs across blocks: the loop state (fractional read position, period,
/// previous symbol) survives from one `process` call to the next, so the
/// clock tracks the difference between the two crystals over a whole call
/// rather than restarting every block the way a per-burst detector would.
struct SymbolSync {
    sps: f64,
    period: f64,
    pos: f64,
    prev: f32,
    /// Samples not yet consumed, with `pos` indexing into them.
    buf: Vec<f32>,
    /// Running mean square, for normalising the timing error.
    power: f32,
    loop_gain: f64,
}

impl SymbolSync {
    fn new(rate: f64) -> Self {
        let sps = rate / BAUD;
        Self {
            sps,
            period: sps,
            pos: sps,
            prev: 0.0,
            buf: Vec::new(),
            power: 1e-6,
            loop_gain: 0.003,
        }
    }

    fn reset(&mut self) {
        self.period = self.sps;
        self.pos = self.sps;
        self.prev = 0.0;
        self.buf.clear();
        self.power = 1e-6;
    }

    fn interp(&self, p: f64) -> f32 {
        if p <= 0.0 {
            return *self.buf.first().unwrap_or(&0.0);
        }
        let i = p.floor() as usize;
        if i + 1 >= self.buf.len() {
            return *self.buf.last().unwrap_or(&0.0);
        }
        let f = (p - i as f64) as f32;
        self.buf[i] * (1.0 - f) + self.buf[i + 1] * f
    }

    /// Feed discriminator samples, append recovered symbol values to `out`.
    fn push(&mut self, samples: &[f32], out: &mut Vec<f32>) {
        self.buf.extend_from_slice(samples);
        // Need half a period of history behind `pos` for the Gardner midpoint
        // and one sample ahead for interpolation.
        while self.pos + 1.0 < self.buf.len() as f64 {
            if self.pos - self.period * 0.5 < 0.0 {
                break;
            }
            let cur = self.interp(self.pos);
            let mid = self.interp(self.pos - self.period * 0.5);
            out.push(cur);
            self.power = 0.999 * self.power + 0.001 * cur * cur;
            let e = ((cur - self.prev) * mid / self.power.max(1e-6)).clamp(-1.0, 1.0) as f64;
            self.prev = cur;
            self.pos += self.period - self.loop_gain * e * self.sps;
            // Keep the period near nominal; the crystals differ by ppm, not %.
            self.period = self.sps;
        }
        // Drain consumed samples so the buffer stays bounded, keeping a period
        // of history behind the read position.
        let keep_from = (self.pos - self.period).floor().max(0.0) as usize;
        if keep_from > 0 && keep_from <= self.buf.len() {
            self.buf.drain(..keep_from);
            self.pos -= keep_from as f64;
        }
    }
}

/// Finds bursts in the symbol stream and reads what they carry.
///
/// Holds a rolling window of symbol values with an absolute index, so a burst
/// whose start arrived in one block can still be read when the rest of it
/// arrives in the next.
struct Framer {
    /// Symbol values, oldest first.
    marks: Vec<f32>,
    /// Absolute index of `marks[0]`.
    base: usize,
    /// Next absolute index to test for a sync word while hunting.
    scan: usize,
    /// First symbol of the next expected burst, once the clock is locked.
    next: Option<usize>,
    /// Consecutive expected bursts that passed no check.
    misses: u32,
    /// Bursts since the last voice sync, so B to F of a superframe are known
    /// by where they are rather than by an EMB field that decodes noise as
    /// valid a fair fraction of the time.
    since_sync: usize,
    /// Colour code of the system being followed, so another user of the same
    /// channel does not steal the lock.
    colour: Option<u8>,
    /// Sync polarity once locked: the discriminator's sign is receiver-set.
    polarity: Option<bool>,
    /// Parsed sync patterns as level indices.
    patterns: Vec<(&'static str, Vec<u8>, bool)>,
    /// The four embedded LC fragments of a superframe, as they arrive.
    embedded: dmr::EmbeddedLc,
}

/// One thing the framer found.
pub enum DmrEvent {
    /// A voice burst: three 72-bit AMBE frames, 9 bytes each. `start` marks
    /// burst A, the one carrying the voice sync.
    Voice { frames: [[u8; 9]; 3], start: bool },
    /// Who is talking, from a header, a terminator or an embedded LC.
    Lc(LinkControl),
    /// A data/signalling burst, by its slot type (`dmr::DT_*`), or `None`
    /// when the slot type would not decode.
    Data(Option<u8>),
}

impl Framer {
    fn new() -> Self {
        let patterns = SYNCS
            .iter()
            .map(|(n, p, v)| (*n, p.bytes().map(|c| c - b'0').collect::<Vec<u8>>(), *v))
            .collect();
        Self {
            marks: Vec::new(),
            base: 0,
            scan: 0,
            next: None,
            misses: 0,
            since_sync: usize::MAX,
            colour: None,
            polarity: None,
            patterns,
            embedded: dmr::EmbeddedLc::new(),
        }
    }

    fn reset(&mut self) {
        self.marks.clear();
        self.base = 0;
        self.scan = 0;
        self.next = None;
        self.misses = 0;
        self.since_sync = usize::MAX;
        self.colour = None;
        self.polarity = None;
        self.embedded.reset();
    }

    /// Fit four level centres to a window by percentiles. The window must
    /// contain all four levels for the inner two centres to be right, so it
    /// is always a whole burst or more, never the sync symbols alone (which
    /// carry only the outer two levels).
    fn centers(window: &[f32]) -> [f32; 4] {
        let mut sorted: Vec<f32> = window.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let q = |f: f32| sorted[((sorted.len() as f32 * f) as usize).min(sorted.len() - 1)];
        [q(0.12), q(0.37), q(0.62), q(0.87)]
    }

    /// Map symbol values to level indices 0..3 using given centres.
    fn apply(vals: &[f32], centers: &[f32; 4], flip: bool) -> Vec<u8> {
        vals.iter()
            .map(|&v| {
                let mut best = 0u8;
                let mut bd = f32::INFINITY;
                for (i, &c) in centers.iter().enumerate() {
                    let d = (v - c).abs();
                    if d < bd {
                        bd = d;
                        best = i as u8;
                    }
                }
                if flip {
                    3 - best
                } else {
                    best
                }
            })
            .collect()
    }

    /// Level index -> dibit (DMR +3=01,+1=00,-1=10,-3=11), MSB first.
    fn dibit(l: u8) -> [u8; 2] {
        match l {
            3 => [0, 1],
            2 => [0, 0],
            1 => [1, 0],
            _ => [1, 1],
        }
    }

    /// The 264 bits of the burst starting at absolute index `start`, or None
    /// if it is not fully buffered. Levels are fitted over the whole burst,
    /// which is the only window that contains all four of them.
    fn burst_bits(&self, start: usize, flip: bool) -> Option<Vec<u8>> {
        if start < self.base || start + SYM_BURST > self.base + self.marks.len() {
            return None;
        }
        let s = start - self.base;
        let window = &self.marks[s..s + SYM_BURST];
        let centers = Self::centers(window);
        let lv = Self::apply(window, &centers, flip);
        let mut bits = Vec::with_capacity(SYM_BURST * 2);
        for &l in &lv {
            bits.extend_from_slice(&Self::dibit(l));
        }
        Some(bits)
    }

    /// The three AMBE frames of a voice burst: 108 bits either side of the
    /// middle field, nine bytes each.
    fn voice_frames(bits: &[u8]) -> [[u8; 9]; 3] {
        let payload: Vec<u8> = bits[..SYM_PAYLOAD * 2]
            .iter()
            .chain(&bits[(SYM_PAYLOAD + SYM_SYNC) * 2..])
            .copied()
            .collect();
        let mut frames = [[0u8; 9]; 3];
        for (f, frame) in frames.iter_mut().enumerate() {
            for (b, byte) in payload[f * 72..(f + 1) * 72].chunks(8).enumerate() {
                frame[b] = byte.iter().fold(0u8, |v, &bit| (v << 1) | (bit & 1));
            }
        }
        frames
    }

    /// Whether the burst at `start` really is burst B of a voice superframe,
    /// used to settle the polarity: its middle field has to hold an EMB the
    /// QR(16,7,6) accepts, which the same burst read the other way up does
    /// not. Without this a wrongly inverted lock reads a voice call as
    /// signalling and drops it.
    fn confirm_voice(&self, start: usize, flip: bool) -> bool {
        let Some(bits) = self.burst_bits(start, flip) else {
            return false;
        };
        let mid = &bits[SYM_PAYLOAD * 2..(SYM_PAYLOAD + SYM_SYNC) * 2];
        let mut emb_bits = mid[..8].to_vec();
        emb_bits.extend_from_slice(&mid[40..48]);
        dmr::emb(&emb_bits).is_some_and(|e| self.colour.is_none_or(|c| c == e.colour))
    }

    /// Dibit -> level index, the inverse of [`Framer::dibit`].
    fn level(d: &[u8]) -> u8 {
        match (d[0] & 1, d[1] & 1) {
            (0, 1) => 3,
            (0, 0) => 2,
            (1, 0) => 1,
            _ => 0,
        }
    }

    /// Read the burst starting at absolute index `start` and say what it is.
    ///
    /// `hunting` is the stricter test used with no clock: a sync word has to
    /// match closely and an EMB is not enough, because seven information bits
    /// will match noise often enough to lock onto nothing.
    fn classify(&self, start: usize, flip: bool, hunting: bool) -> Option<Burst> {
        let bits = self.burst_bits(start, flip)?;
        let mid = &bits[SYM_PAYLOAD * 2..(SYM_PAYLOAD + SYM_SYNC) * 2];
        let lv: Vec<u8> = mid.chunks(2).map(Self::level).collect();
        let tol = if hunting { 2 } else { 4 };
        for (_name, pat, voice) in &self.patterns {
            let err = lv.iter().zip(pat).filter(|(a, b)| a != b).count();
            if err > tol {
                continue;
            }
            if *voice {
                return Some(Burst::Voice {
                    frames: Self::voice_frames(&bits),
                    start: true,
                    lcss: 0,
                    embedded: Vec::new(),
                });
            }
            let mut slot = bits[98..108].to_vec();
            slot.extend_from_slice(&bits[156..166]);
            // A data sync with no readable slot type is a sync word matched
            // in noise: the Golay(20,8) over it is the second opinion.
            let (cc, dt) = dmr::slot_type(&slot)?;
            if self.colour.is_some_and(|c| c != cc) {
                return None;
            }
            let lc = match dt {
                dmr::DT_VOICE_LC_HEADER | dmr::DT_TERMINATOR_LC => {
                    let mut info = bits[0..98].to_vec();
                    info.extend_from_slice(&bits[166..264]);
                    dmr::full_lc(&info)
                }
                _ => None,
            };
            return Some(Burst::Data {
                colour: Some(cc),
                data_type: Some(dt),
                lc,
            });
        }
        if hunting {
            return None;
        }
        // No sync, so this should be burst B to F of a superframe: the middle
        // field is EMB, embedded signalling, EMB. Which fragment it carries
        // is decided by the position in the superframe, since the burst clock
        // is a stronger statement than seven information bits are.
        let mut emb_bits = mid[..8].to_vec();
        emb_bits.extend_from_slice(&mid[40..48]);
        let e = dmr::emb(&emb_bits);
        let pos = self.since_sync.saturating_add(1);
        if let Some(e) = e {
            if self.colour.is_some_and(|c| c != e.colour) {
                return None;
            }
        } else if pos >= SUPERFRAME_BURSTS {
            // Out of the superframe the sync anchored, with nothing in the
            // burst itself saying it is voice: this is noise or another
            // system, not the transmission being followed.
            return None;
        }
        // Burst B carries the first LC fragment, C and D continuations, E
        // the last; F carries none. `since_sync` still counts the burst
        // before this one, so B is one past a zero.
        let lcss = match pos {
            1 => 1,
            2 | 3 => 3,
            4 => 2,
            _ => 0,
        };
        Some(Burst::Voice {
            frames: Self::voice_frames(&bits),
            start: false,
            lcss,
            embedded: mid[8..40].to_vec(),
        })
    }

    /// Append recovered symbols and pull out the bursts they complete.
    fn push(&mut self, syms: &[f32], out: &mut Vec<DmrEvent>) {
        self.marks.extend_from_slice(syms);
        loop {
            let last = self.base + self.marks.len();
            match self.next {
                Some(next) => {
                    if next + SYM_BURST + REANCHOR > last || next < self.base + REANCHOR {
                        break;
                    }
                    let flip = self.polarity.unwrap_or(false);
                    // Nearest first: a burst is far more likely on time than
                    // a symbol out, and taking the first match at the wrong
                    // offset would drag the clock off.
                    let mut hit = None;
                    for off in [0isize, -1, 1, -2, 2] {
                        if off.unsigned_abs() > REANCHOR {
                            continue;
                        }
                        let at = next.wrapping_add_signed(off);
                        if let Some(b) = self.classify(at, flip, false) {
                            hit = Some((at, b));
                            break;
                        }
                    }
                    match hit {
                        Some((at, burst)) => {
                            self.misses = 0;
                            self.next = Some(at + SLOT_STRIDE);
                            self.emit(burst, out);
                        }
                        None => {
                            self.misses += 1;
                            self.since_sync = self.since_sync.saturating_add(1);
                            if self.misses > MAX_MISSES {
                                self.next = None;
                                self.colour = None;
                                self.embedded.reset();
                                self.scan = self.scan.max(next);
                            } else {
                                self.next = Some(next + SLOT_STRIDE);
                            }
                        }
                    }
                }
                None => {
                    self.scan = self.scan.max(self.base);
                    let mut locked = false;
                    // Confirming a voice lock reads the following burst, so
                    // hunting needs that much buffered before it commits.
                    let mut waiting = false;
                    while self.scan + SYM_BURST <= last {
                        let polarities: [bool; 2] = match self.polarity {
                            Some(p) => [p, p],
                            None => [false, true],
                        };
                        let mut found = None;
                        for flip in polarities {
                            let Some(b) = self.classify(self.scan, flip, true) else {
                                continue;
                            };
                            if matches!(b, Burst::Voice { start: true, .. }) {
                                if self.scan + SLOT_STRIDE + SYM_BURST > last {
                                    waiting = true;
                                    break;
                                }
                                if !self.confirm_voice(self.scan + SLOT_STRIDE, flip) {
                                    continue;
                                }
                            }
                            found = Some((flip, b));
                            break;
                        }
                        if waiting {
                            break;
                        }
                        if let Some((flip, burst)) = found {
                            self.polarity = Some(flip);
                            self.misses = 0;
                            self.next = Some(self.scan + SLOT_STRIDE);
                            self.emit(burst, out);
                            locked = true;
                            break;
                        }
                        self.scan += 1;
                    }
                    if !locked || waiting {
                        break;
                    }
                }
            }
        }
        // Drain marks behind whatever is still to be read.
        let keep = self
            .next
            .map_or(self.scan, |n| n.saturating_sub(REANCHOR))
            .min(self.scan.max(self.base));
        if keep > self.base {
            let drop = (keep - self.base).min(self.marks.len());
            self.marks.drain(..drop);
            self.base += drop;
        }
    }

    /// Turn a read burst into the events the node acts on, gathering the
    /// embedded link control as the fragments arrive.
    fn emit(&mut self, burst: Burst, out: &mut Vec<DmrEvent>) {
        match burst {
            Burst::Voice {
                frames,
                start,
                lcss,
                embedded,
            } => {
                if start {
                    self.since_sync = 0;
                    self.embedded.reset();
                } else {
                    self.since_sync = self.since_sync.saturating_add(1);
                    if let Some(lc) = self.embedded.push(lcss, &embedded) {
                        out.push(DmrEvent::Lc(lc));
                    }
                }
                out.push(DmrEvent::Voice { frames, start });
            }
            Burst::Data {
                colour,
                data_type,
                lc,
            } => {
                self.since_sync = usize::MAX;
                if let Some(cc) = colour {
                    self.colour = Some(cc);
                }
                if let Some(lc) = lc {
                    out.push(DmrEvent::Lc(lc));
                }
                out.push(DmrEvent::Data(data_type));
            }
        }
    }
}

/// What one burst turned out to be, before the framer folds it into events.
enum Burst {
    Voice {
        frames: [[u8; 9]; 3],
        start: bool,
        lcss: u8,
        embedded: Vec<u8>,
    },
    Data {
        colour: Option<u8>,
        data_type: Option<u8>,
        lc: Option<LinkControl>,
    },
}

pub struct DmrNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    rrc: FirDecimReal,
    sync: SymbolSync,
    framer: Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    /// Discriminator output through the matched filter.
    shaped: Vec<f32>,
    syms: Vec<f32>,
    /// Speech decoded this block, for listening live.
    voice_now: Vec<f32>,
    /// Whole-transmission speech, for the packet log.
    voice: Vec<f32>,
    /// Whether a voice transmission is in progress.
    talking: bool,
    /// Who is talking, once a header, terminator or embedded LC has said.
    lc: Option<LinkControl>,
    /// Voice bursts in the transmission in progress, for its duration: each
    /// burst is one 60 ms slot of speech.
    voice_bursts: u32,
    /// Bursts since the last voice sync, to notice a transmission ending.
    idle_bursts: u32,
    /// Input samples since the last voice burst. A transmission that simply
    /// stops (the carrier drops, no terminator heard) ends when no voice has
    /// arrived for a while, the way M17 closes on a silent clock. Counted in
    /// samples, not blocks, so it does not depend on how the caller chunks
    /// the stream (a live radio and the auto node's flush differ).
    silent_samples: u64,
    /// Input sample rate, for the silence timeout.
    in_rate: f64,
    accepted: u64,
    #[cfg(feature = "ambe")]
    synth: mbe::ambe::AmbeSynthesizer,
}

impl Default for DmrNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;

impl DmrNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, FILTER_CUTOFF_HZ, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            rrc: FirDecimReal::new(rrc_taps(AUDIO_HZ / BAUD, RRC_ALPHA, 8), 1),
            sync: SymbolSync::new(AUDIO_HZ),
            framer: Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            shaped: Vec::new(),
            syms: Vec::new(),
            voice_now: Vec::new(),
            voice: Vec::new(),
            talking: false,
            lc: None,
            voice_bursts: 0,
            idle_bursts: 0,
            silent_samples: 0,
            in_rate: AUDIO_HZ,
            accepted: 0,
            #[cfg(feature = "ambe")]
            synth: mbe::ambe::AmbeSynthesizer::new(),
        }
    }

    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    pub fn voice_now(&self) -> &[f32] {
        &self.voice_now
    }

    /// Decode one voice burst's three AMBE frames to speech, muting frames the
    /// Golay check says are too damaged (which is what a burst that is not
    /// really voice, or badly received, looks like).
    #[cfg(feature = "ambe")]
    fn decode_voice(&mut self, frames: &[[u8; 9]; 3]) {
        let cap = (MAX_VOICE_SECONDS * VOICE_HZ) as usize;
        for f in frames {
            let e = mbe::ambe::AmbeFrame::new(f).errors();
            let audio = if e[0] + e[1] <= 4 {
                self.synth.decode(f)
            } else {
                [0.0f32; 160]
            };
            for s in audio {
                self.voice_now.push(s);
                if self.voice.len() < cap {
                    self.voice.push(s);
                }
            }
        }
    }

    #[cfg(not(feature = "ambe"))]
    fn decode_voice(&mut self, _frames: &[[u8; 9]; 3]) {}

    fn take_voice(&mut self) -> Option<std::sync::Arc<common::Speech>> {
        if self.voice.is_empty() {
            return None;
        }
        let pcm = std::mem::take(&mut self.voice);
        Some(std::sync::Arc::new(common::Speech {
            pcm,
            rate: VOICE_HZ,
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use pipeline::node::NodeCtx;
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec {
            spec: StreamSpec::iq(rate, Hz(center as u64)),
            latency: 0,
        }
    }

    #[test]
    fn negotiates_a_channel_inside_the_span() {
        let mut n = DmrNode::default();
        assert!(n.negotiate(&[spec(2_048_000.0, DEFAULT_HZ)]).is_ok());
        assert!(n.negotiate(&[spec(2_048_000.0, 460_000_000.0)]).is_err());
    }

    #[test]
    fn labels_only_its_own_voice_over() {
        let body = encode_voice_over(50, None);
        let d = dmr_decoded(&body, common::Hz(433_450_000)).expect("a DMR row");
        assert_eq!(d.protocol, "DMR-Voice");
        // 50 bursts x 60 ms = 3.0 s.
        assert!(
            d.detail
                .as_deref()
                .unwrap_or_default()
                .contains("seconds=3"),
            "{:?}",
            d.detail
        );
        // Without a link control there is nobody to put in the call list.
        assert!(!d.fields.iter().any(|(k, _)| k == "to"));

        // With one, the row names the talkgroup and the radio, and says it
        // is voice, which is what the call table needs to keep it.
        let lc = LinkControl {
            flco: dmr::FLCO_GROUP,
            fid: 0,
            options: 0,
            dst: 91,
            src: 2_345_678,
        };
        let d =
            dmr_decoded(&encode_voice_over(6, Some(lc)), common::Hz(433_450_000)).expect("a row");
        let get = |k: &str| {
            d.fields
                .iter()
                .find(|(n, _)| n == k)
                .map(|(_, v)| v.to_string())
                .unwrap_or_default()
        };
        assert_eq!(get("to"), "91");
        assert_eq!(get("from"), "2345678");
        assert_eq!(get("voice"), "true");
        assert_eq!(get("call_type"), "group");
        // Not anyone else's frame.
        assert!(dmr_decoded(b"random", common::Hz(0)).is_none());
        assert!(dmr_decoded(b"DV", common::Hz(0)).is_none());
    }

    /// Run a capture through the node's own path and report what came out:
    /// speech samples, packet rows and the link control on each.
    fn replay(path: &str, rate: f64, center: f64, hz: f64) -> (usize, Vec<common::Packet>) {
        let raw = std::fs::read(path).unwrap();
        let iq: Vec<common::C32> = raw
            .chunks_exact(2)
            .map(|c| common::C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
            .collect();
        let mut node = DmrNode::new(hz);
        node.negotiate(&[spec(rate, center)]).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let (mut live, mut packets) = (0usize, Vec::new());
        for chunk in iq.chunks(65_536) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = [Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], &mut out, &mut ctx).unwrap();
            if let [Payload::Packets(ps), Payload::Voice(vs)] = &out {
                live += vs.iter().map(|v| v.pcm.len()).sum::<usize>();
                packets.extend(ps.iter().cloned());
            }
        }
        (live, packets)
    }

    /// A real off-air transmission has to come out as one row naming who was
    /// talking, not as several anonymous ones. Skips cleanly without the file.
    #[test]
    #[ignore]
    fn reads_link_control_from_a_real_capture() {
        let path = format!("{}/dmr_dev/dmr_good1.cu8", std::env::var("HOME").unwrap());
        if !std::path::Path::new(&path).exists() {
            eprintln!("no capture at {path}; skipping");
            return;
        }
        let (_, packets) = replay(&path, 2_048_000.0, 434_000_000.0, 434_000_000.0 + 448_200.0);
        for p in &packets {
            let common::PacketBody::Frame(b) = &p.body else {
                continue;
            };
            let d = dmr_decoded(b, common::Hz(p.center_hz)).expect("a DMR row");
            eprintln!("{}", d.detail.unwrap_or_default());
        }
        assert_eq!(packets.len(), 1, "one transmission should be one row");
        let common::PacketBody::Frame(b) = &packets[0].body else {
            panic!("a frame")
        };
        let d = dmr_decoded(b, common::Hz(0)).expect("a DMR row");
        assert!(
            d.fields.iter().any(|(k, _)| k == "to"),
            "no link control was read"
        );
    }

    /// End to end on a real off-air capture: run the node's own path (mix,
    /// discriminate, Gardner timing, sync, AMBE) and require it to produce
    /// speech. Only asserts audio with the `ambe` feature, since the vocoder
    /// is what turns the frames into samples. Skips cleanly without the file.
    #[cfg(feature = "ambe")]
    #[test]
    #[ignore]
    fn decodes_a_real_dmr_capture() {
        let path = format!("{}/dmr_dev/dmr_good1.cu8", std::env::var("HOME").unwrap());
        if !std::path::Path::new(&path).exists() {
            eprintln!("no capture at {path}; skipping");
            return;
        }
        let raw = std::fs::read(&path).unwrap();
        let rate = 2_048_000.0;
        // The capture is at 433.45 center but the signal sits ~449 kHz up in
        // the file recorded at 434.0; tune the node's channel to where it is.
        let center = 434_000_000.0;
        let hz = 434_000_000.0 + 448_200.0;
        let iq: Vec<common::C32> = raw
            .chunks_exact(2)
            .map(|c| common::C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
            .collect();

        let mut node = DmrNode::new(hz);
        node.negotiate(&[spec(rate, center)]).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut live = 0usize;
        for chunk in iq.chunks(65_536) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = [Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], &mut out, &mut ctx).unwrap();
            if let [_, Payload::Voice(vs)] = &out {
                live += vs.iter().map(|v| v.pcm.len()).sum::<usize>();
            }
        }
        eprintln!(
            "decoded {live} voice samples ({:.2}s)",
            live as f32 / VOICE_HZ as f32
        );
        // This capture holds two voice superframes (the operator spoke
        // briefly): 2 x 6 bursts x 3 frames x 160 samples = 5760. Require a
        // healthy fraction so a regression that loses framing is caught.
        assert!(
            live >= 4000,
            "expected the voice superframes to decode, got {live} samples"
        );
    }
}

impl Node for DmrNode {
    fn name(&self) -> &str {
        "dmr"
    }

    fn channels(&self) -> &'static [f64] {
        &[CHANNEL_WIDTH_HZ]
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dmr reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other(
                "dmr needs its channel inside the span",
            ));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, FILTER_CUTOFF_HZ, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.rrc = FirDecimReal::new(rrc_taps(audio_rate / BAUD, RRC_ALPHA, 8), 1);
        self.sync = SymbolSync::new(audio_rate);
        self.framer = Framer::new();
        self.in_rate = rate;

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        let mut voice = out.with_kind(PortKind::Voice);
        voice.rate = VOICE_HZ;
        Ok(vec![out, voice])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else {
            return Ok(());
        };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);
        let raw = std::mem::take(&mut self.audio);
        let mut shaped = std::mem::take(&mut self.shaped);
        shaped.clear();
        self.rrc.process(&raw, &mut shaped);
        self.audio = raw;

        self.syms.clear();
        let mut syms = std::mem::take(&mut self.syms);
        self.sync.push(&shaped, &mut syms);
        self.shaped = shaped;

        let mut events = Vec::new();
        self.framer.push(&syms, &mut events);
        self.syms = syms;

        self.voice_now.clear();
        let mut ended = false;
        let had_voice = events.iter().any(|e| matches!(e, DmrEvent::Voice { .. }));
        for e in events {
            match e {
                DmrEvent::Voice { frames, .. } => {
                    if !self.talking {
                        self.voice.clear();
                        self.voice_bursts = 0;
                        self.lc = None;
                        self.talking = true;
                    }
                    self.decode_voice(&frames);
                    self.voice_bursts += 1;
                    self.idle_bursts = 0;
                }
                DmrEvent::Lc(lc) => self.lc = Some(lc),
                DmrEvent::Data(dt) => match dt {
                    // The terminator is the transmission saying it is over,
                    // which is the only end that is not a guess.
                    Some(dmr::DT_TERMINATOR_LC) => ended = self.talking,
                    // A header opens one: keep the LC it just carried and
                    // start counting from here.
                    Some(dmr::DT_VOICE_LC_HEADER) => {
                        if !self.talking {
                            self.voice.clear();
                            self.voice_bursts = 0;
                            self.talking = true;
                        }
                        self.idle_bursts = 0;
                        self.silent_samples = 0;
                    }
                    _ => {
                        // Signalling with no voice between it means the
                        // channel has moved on without a terminator heard.
                        if self.talking {
                            self.idle_bursts += 1;
                            if self.idle_bursts >= 8 {
                                ended = true;
                            }
                        }
                    }
                },
            }
        }

        // End on silence too: a carrier that drops after the last voice burst
        // leaves no terminator to count, so an over that saw voice is closed
        // once enough input time passes with none. A superframe is 360 ms and
        // the framer rides out eight missed bursts (480 ms) before it lets go
        // of the clock, so the timeout has to be longer than both or a fade
        // mid-over becomes two rows in the log. It was 0.5 s, and a single
        // transmission arrived as three.
        if self.talking {
            if had_voice {
                self.silent_samples = 0;
            } else {
                self.silent_samples += iq.len() as u64;
                if self.silent_samples as f64 >= self.in_rate * 1.5 {
                    ended = true;
                }
            }
        }

        let (from, to) = match self.lc {
            Some(lc) => (Some(lc.src.to_string()), Some(lc.dst.to_string())),
            None => (None, None),
        };
        outputs[OUT_VOICE].voice_mut().push(common::Voice {
            system: "DMR",
            channel_hz: self.channel_hz,
            to,
            from,
            rate: VOICE_HZ,
            pcm: std::mem::take(&mut self.voice_now),
        });

        if ended {
            let keep = self.voice_bursts >= MIN_BURSTS || self.lc.is_some();
            if !keep {
                self.voice.clear();
                self.talking = false;
                self.lc = None;
                self.voice_bursts = 0;
                self.idle_bursts = 0;
                self.silent_samples = 0;
                return Ok(());
            }
            let at_us = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_micros() as u64)
                .unwrap_or(0);
            let audio = self.take_voice();
            let bursts = self.voice_bursts;
            let lc = self.lc.take();
            self.talking = false;
            self.voice_bursts = 0;
            self.idle_bursts = 0;
            self.silent_samples = 0;
            self.accepted += 1;
            outputs[OUT_PACKETS].packets_mut().push(common::Packet {
                at_us,
                center_hz: self.channel_hz as u64,
                bandwidth_hz: CHANNEL_WIDTH_HZ as u32,
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: Some("4FSK"),
                body: common::PacketBody::Frame(encode_voice_over(bursts, lc)),
                iq: None,
                audio,
                measure: None,
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.sync.reset();
        self.framer.reset();
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.rrc.reset();
        self.voice.clear();
        self.voice_now.clear();
        self.talking = false;
        self.lc = None;
        self.voice_bursts = 0;
        self.idle_bursts = 0;
        self.silent_samples = 0;
        #[cfg(feature = "ambe")]
        {
            self.synth = mbe::ambe::AmbeSynthesizer::new();
        }
    }
}
