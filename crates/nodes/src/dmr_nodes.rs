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

mod dmr_ambe;
use dmr_ambe::Vocoder;

/// Tag identifying a packet body this node wrote: one DMR burst. "DB".
///
/// A packet is one burst off the air, the 264 bits of it as received, with
/// what the framer knew when it read them: where in the superframe it sat,
/// the colour code, and the link control in force for the transmission,
/// which the burst itself carries only if it is a header or a terminator.
/// Everything the log shows about it is read back out of these bytes, so a
/// replay decodes the same burst again and a decoder written later gets its
/// chance at it. What the packet does not carry is the whole over: that is
/// reconstructed downstream from the run of bursts, the way a stream is
/// followed across frames.
const DMR_TAG: [u8; 2] = *b"DB";

/// Body: tag, position, colour, flags, destination, source, 264 bits.
const BODY_LEN: usize = 2 + 1 + 1 + 1 + 4 + 4 + BURST_BYTES;
/// 264 bits, packed most significant bit first.
const BURST_BYTES: usize = SYM_BURST * 2 / 8;

/// Position byte: voice bursts A to F of a superframe, or a burst with a
/// data sync, whose slot type is in the bits.
const POS_DATA: u8 = 0xff;

const FLAG_HAVE_LC: u8 = 0x01;
const FLAG_GROUP: u8 = 0x02;
const FLAG_ENCRYPTED: u8 = 0x04;
const FLAG_EMERGENCY: u8 = 0x08;

/// The tag of the row the node used to write, one per over, kept readable
/// so an old log still labels.
const OVER_TAG: [u8; 2] = *b"DV";
const OVER_LEN: usize = 2 + 4 + 1 + 4 + 4;

fn pack_bits(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8).map(|c| c.iter().fold(0u8, |v, &b| (v << 1) | (b & 1))).collect()
}

fn unpack_bits(bytes: &[u8]) -> Vec<u8> {
    bytes.iter().flat_map(|b| (0..8).rev().map(move |i| (b >> i) & 1)).collect()
}

fn lc_flags(lc: Option<&LinkControl>) -> u8 {
    let Some(lc) = lc else { return 0 };
    let mut flags = FLAG_HAVE_LC;
    if lc.group() {
        flags |= FLAG_GROUP;
    }
    if lc.encrypted() {
        flags |= FLAG_ENCRYPTED;
    }
    if lc.emergency() {
        flags |= FLAG_EMERGENCY;
    }
    flags
}

/// Serialise one burst with the framer's context for it.
fn encode_burst(pos: u8, colour: Option<u8>, lc: Option<&LinkControl>, bits: &[u8]) -> Vec<u8> {
    let mut v = DMR_TAG.to_vec();
    v.push(pos);
    v.push(colour.unwrap_or(0xff));
    v.push(lc_flags(lc));
    v.extend_from_slice(&lc.map_or(0, |l| l.dst).to_be_bytes());
    v.extend_from_slice(&lc.map_or(0, |l| l.src).to_be_bytes());
    v.extend(pack_bits(bits));
    v
}

/// The AMBE frames of a voice burst's bits, as [`Framer::voice_frames`]
/// reads them, for anything that wants to hear a logged burst again.
pub fn burst_voice_frames(bytes: &[u8]) -> Option<[[u8; 9]; 3]> {
    if bytes.len() != BODY_LEN || bytes[..2] != DMR_TAG || bytes[2] == POS_DATA {
        return None;
    }
    Some(Framer::voice_frames(&unpack_bits(&bytes[13..])))
}

fn lc_fields(flags: u8, dst: u32, src: u32, fields: &mut Vec<(String, common::Value)>) {
    use common::Value;
    if flags & FLAG_HAVE_LC == 0 {
        return;
    }
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

/// Recognise and describe a DMR row for the packet log. Returns `None` for
/// anything this node did not write, so it is safe to try on every frame the
/// way `m17_decoded` is.
///
/// A voice burst is `DMR-Voice`, 60 ms of the channel, `live` while the
/// transmission runs; a header is the same with the over starting, and a
/// terminator ends it. A row with a link control names its talkgroup and
/// radio and says `voice`, which is what puts it in the call list rather
/// than only in the log.
pub fn dmr_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if bytes.len() == OVER_LEN && bytes[..2] == OVER_TAG {
        return over_decoded(bytes, center);
    }
    if bytes.len() != BODY_LEN || bytes[..2] != DMR_TAG {
        return None;
    }
    let pos = bytes[2];
    let colour = bytes[3];
    let flags = bytes[4];
    let dst = u32::from_be_bytes([bytes[5], bytes[6], bytes[7], bytes[8]]);
    let src = u32::from_be_bytes([bytes[9], bytes[10], bytes[11], bytes[12]]);
    let bits = unpack_bits(&bytes[13..]);
    let mut fields = Vec::new();
    if colour != 0xff {
        fields.push(("colour_code".to_string(), Value::Int(i64::from(colour))));
    }
    let model = if pos == POS_DATA {
        let mut slot = bits[98..108].to_vec();
        slot.extend_from_slice(&bits[156..166]);
        let dt = dmr::slot_type(&slot).map(|(_, dt)| dt);
        match dt {
            Some(dmr::DT_VOICE_LC_HEADER) => {
                lc_fields(flags, dst, src, &mut fields);
                fields.push(("live".to_string(), Value::Bool(true)));
                "DMR-Header"
            }
            Some(dmr::DT_TERMINATOR_LC) => {
                lc_fields(flags, dst, src, &mut fields);
                "DMR-Terminator"
            }
            Some(dt) => {
                fields.push(("data_type".to_string(), Value::Int(i64::from(dt))));
                "DMR-Data"
            }
            None => "DMR-Data",
        }
    } else {
        // One burst is one 60 ms slot on this logical channel.
        fields.push(("seconds".to_string(), Value::Float(0.06)));
        fields.push(("burst".to_string(), Value::Text(((b'A' + pos.min(5)) as char).to_string())));
        lc_fields(flags, dst, src, &mut fields);
        fields.push(("live".to_string(), Value::Bool(true)));
        "DMR-Voice"
    };
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    Some(
        Decoded::bytes(model, center, 0.0, bytes.to_vec())
            .with_detail(detail)
            .with_fields(fields)
            .with_modulation("4FSK"),
    )
}

/// The old one-row-per-over body, for logs written before bursts were logged.
fn over_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let bursts = u32::from_be_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
    let flags = bytes[6];
    let dst = u32::from_be_bytes([bytes[7], bytes[8], bytes[9], bytes[10]]);
    let src = u32::from_be_bytes([bytes[11], bytes[12], bytes[13], bytes[14]]);
    let mut fields = vec![
        ("seconds".to_string(), Value::Float(f64::from(bursts) * 0.06)),
        ("bursts".to_string(), Value::Int(i64::from(bursts))),
    ];
    lc_fields(flags, dst, src, &mut fields);
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
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

/// Silence after the last voice burst before the link control it was under
/// is forgotten, so a transmission that dropped without a terminator does
/// not lend its talkgroup to the next one. Longer than a superframe (360 ms)
/// and the framer's eight missed bursts (480 ms).
const OVER_SILENCE_S: f64 = 1.5;

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
        Self { sps, period: sps, pos: sps, prev: 0.0, buf: Vec::new(), power: 1e-6, loop_gain: 0.003 }
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

/// One thing the framer found. `at` is the absolute symbol index the burst
/// began at and `bits` its 264 bits as received.
pub enum DmrEvent {
    /// A voice burst: three 72-bit AMBE frames, 9 bytes each. `pos` is its
    /// place in the superframe, 0 for burst A, the one carrying the sync.
    Voice { at: usize, bits: Vec<u8>, frames: [[u8; 9]; 3], pos: u8 },
    /// Who is talking, from a header, a terminator or an embedded LC.
    Lc(LinkControl),
    /// A data/signalling burst, by its slot type (`dmr::DT_*`), or `None`
    /// when the slot type would not decode.
    Data { at: usize, bits: Vec<u8>, data_type: Option<u8> },
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
                    bits,
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
            return Some(Burst::Data { colour: Some(cc), data_type: Some(dt), lc, bits });
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
            bits,
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
                            self.emit(at, burst, out);
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
                            self.emit(self.scan, burst, out);
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
        let keep =
            self.next.map_or(self.scan, |n| n.saturating_sub(REANCHOR)).min(self.scan.max(self.base));
        if keep > self.base {
            let drop = (keep - self.base).min(self.marks.len());
            self.marks.drain(..drop);
            self.base += drop;
        }
    }

    /// Turn a read burst into the events the node acts on, gathering the
    /// embedded link control as the fragments arrive.
    fn emit(&mut self, at: usize, burst: Burst, out: &mut Vec<DmrEvent>) {
        match burst {
            Burst::Voice { frames, start, lcss, embedded, bits } => {
                if start {
                    self.since_sync = 0;
                    self.embedded.reset();
                } else {
                    self.since_sync = self.since_sync.saturating_add(1);
                    if let Some(lc) = self.embedded.push(lcss, &embedded) {
                        out.push(DmrEvent::Lc(lc));
                    }
                }
                let pos = self.since_sync.min(5) as u8;
                out.push(DmrEvent::Voice { at, bits, frames, pos });
            }
            Burst::Data { colour, data_type, lc, bits } => {
                self.since_sync = usize::MAX;
                if let Some(cc) = colour {
                    self.colour = Some(cc);
                }
                if let Some(lc) = lc {
                    out.push(DmrEvent::Lc(lc));
                }
                out.push(DmrEvent::Data { at, bits, data_type });
            }
        }
    }
}

/// What one burst turned out to be, before the framer folds it into events.
enum Burst {
    Voice { frames: [[u8; 9]; 3], start: bool, lcss: u8, embedded: Vec<u8>, bits: Vec<u8> },
    Data { colour: Option<u8>, data_type: Option<u8>, lc: Option<LinkControl>, bits: Vec<u8> },
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
    /// Whether a voice transmission is in progress.
    talking: bool,
    /// Who is talking, once a header, terminator or embedded LC has said.
    lc: Option<LinkControl>,
    /// Bursts since the last voice sync, to notice a transmission ending.
    idle_bursts: u32,
    /// Input samples since the last voice burst, so a transmission that
    /// simply stops lets go of its link control after a while.
    silent_samples: u64,
    /// Input sample rate, for the silence timeout.
    in_rate: f64,
    /// Channel samples at `audio_rate`, kept behind the symbol clock so each
    /// burst's packet can carry the samples it was read from; `narrow_base`
    /// is the absolute index of `ring[0]`.
    ring: Vec<common::C32>,
    ring_base: usize,
    audio_rate: f64,
    accepted: u64,
    /// The AMBE speech path. A zero-size stub without the `ambe` feature, so
    /// the node reads signalling only there and decodes no speech.
    vocoder: Vocoder,
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
            talking: false,
            lc: None,
            idle_bursts: 0,
            silent_samples: 0,
            in_rate: AUDIO_HZ,
            ring: Vec::new(),
            ring_base: 0,
            audio_rate: AUDIO_HZ,
            accepted: 0,
            vocoder: Vocoder::new(),
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

    /// Decode one voice burst's three AMBE frames to speech, for the live
    /// bus and for the burst's own packet. Without the `ambe` feature the
    /// vocoder yields no samples, so this returns `None`.
    fn decode_voice(&mut self, frames: &[[u8; 9]; 3]) -> Option<std::sync::Arc<common::Speech>> {
        let pcm = self.vocoder.decode_burst(frames);
        if pcm.is_empty() {
            return None;
        }
        self.voice_now.extend_from_slice(&pcm);
        Some(std::sync::Arc::new(common::Speech { pcm, rate: VOICE_HZ }))
    }

    /// The channel samples a burst was read from, by its symbol index. The
    /// filters ahead of the symbol clock delay the symbols by a few dozen
    /// samples, which is inside the burst's own guard.
    fn burst_iq(&self, at: usize) -> Option<std::sync::Arc<common::IqBurst>> {
        let sps = self.audio_rate / BAUD;
        let start = (at as f64 * sps) as usize;
        let len = (SYM_BURST as f64 * sps) as usize;
        if start < self.ring_base || start + len > self.ring_base + self.ring.len() {
            return None;
        }
        let s = start - self.ring_base;
        Some(std::sync::Arc::new(common::IqBurst {
            rate: self.audio_rate,
            center_hz: self.channel_hz as u64,
            samples: self.ring[s..s + len].to_vec(),
        }))
    }

    /// One burst as a packet: its bits, the framer's context, its speech
    /// and the samples it came from.
    fn packet(&mut self, at: usize, pos: u8, bits: &[u8], audio: Option<std::sync::Arc<common::Speech>>) -> common::Packet {
        let at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        self.accepted += 1;
        common::Packet {
            at_us,
            center_hz: self.channel_hz as u64,
            bandwidth_hz: CHANNEL_WIDTH_HZ as u32,
            rssi_dbfs: f32::NAN,
            snr_db: f32::NAN,
            modulation: Some("4FSK"),
            body: common::PacketBody::Frame(encode_burst(pos, self.framer.colour, self.lc.as_ref(), bits)),
            iq: self.burst_iq(at),
            audio,
            measure: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use pipeline::node::NodeCtx;
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn negotiates_a_channel_inside_the_span() {
        let mut n = DmrNode::default();
        assert!(n.negotiate(&[spec(2_048_000.0, DEFAULT_HZ)]).is_ok());
        assert!(n.negotiate(&[spec(2_048_000.0, 460_000_000.0)]).is_err());
    }

    #[test]
    fn labels_only_its_own_bursts() {
        let bits = vec![0u8; SYM_BURST * 2];
        let body = encode_burst(2, Some(1), None, &bits);
        let d = dmr_decoded(&body, common::Hz(433_450_000)).expect("a DMR row");
        assert_eq!(d.protocol, "DMR-Voice");
        // One burst is 60 ms of the channel, and the over is still running.
        assert!(d.detail.as_deref().unwrap_or_default().contains("seconds=0.06"), "{:?}", d.detail);
        assert!(d.detail.as_deref().unwrap_or_default().contains("burst=C"), "{:?}", d.detail);
        assert!(d.fields.iter().any(|(k, v)| k == "live" && *v == common::Value::Bool(true)));
        // Without a link control there is nobody to put in the call list.
        assert!(!d.fields.iter().any(|(k, _)| k == "to"));
        // The bits come back out as they went in.
        assert_eq!(unpack_bits(&body[13..]), bits);

        // With one, the row names the talkgroup and the radio, and says it
        // is voice, which is what the call table needs to keep it.
        let lc = LinkControl { flco: dmr::FLCO_GROUP, fid: 0, options: 0, dst: 91, src: 2_345_678 };
        let d = dmr_decoded(&encode_burst(0, None, Some(&lc), &bits), common::Hz(433_450_000)).expect("a row");
        let get = |k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert_eq!(get("to"), "91");
        assert_eq!(get("from"), "2345678");
        assert_eq!(get("voice"), "true");
        assert_eq!(get("call_type"), "group");
        // Not anyone else's frame.
        assert!(dmr_decoded(b"random", common::Hz(0)).is_none());
        assert!(dmr_decoded(b"DB", common::Hz(0)).is_none());
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

    /// The corpus capture against what the transmission says about itself:
    /// one over, talkgroup 9, radio 1234567, three and a half seconds of it.
    /// `testdata/fixtures.toml` says what the capture is evidence of and how
    /// those values were established. Skips cleanly without the file.
    #[test]
    fn reads_one_over_and_its_link_control_off_air() {
        const NAME: &str = "dmr_tg9_433.45M_2048k.cu8";
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/dmr_tg9_433.45M_2048k.cu8");
        if !std::path::Path::new(path).exists() {
            eprintln!("skipping: {NAME} absent, run testdata/fetch.sh");
            return;
        }
        let (live, packets) = replay(path, 2_048_000.0, 433_450_000.0, 433_900_000.0);
        // One packet per burst off the air: the headers, the voice, the
        // terminator, each carrying the link control in force. The over is
        // the run of them, added up downstream.
        let rows: Vec<Decoded> = packets
            .iter()
            .filter_map(|p| match &p.body {
                common::PacketBody::Frame(b) => dmr_decoded(b, common::Hz(p.center_hz)),
                _ => None,
            })
            .collect();
        assert_eq!(rows.len(), packets.len(), "every packet labels as DMR");
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        let voice: Vec<&Decoded> = rows.iter().filter(|d| d.protocol == "DMR-Voice").collect();
        let seconds: f64 = voice.iter().map(|d| get(d, "seconds").parse::<f64>().unwrap_or(0.0)).sum();
        assert!(seconds > 3.0, "the over ran {seconds:.2} s of voice bursts, expected the whole 3.6");
        // Every voice burst after the header names the call, so the call list
        // has it from the first burst and not from the terminator.
        let named = voice.iter().filter(|d| get(d, "to") == "9" && get(d, "from") == "1234567").count();
        assert!(named * 10 > voice.len() * 9, "{named} of {} voice bursts carried the link control", voice.len());
        assert!(voice.iter().all(|d| get(d, "live") == "true"));
        assert!(rows.iter().any(|d| d.protocol == "DMR-Header"), "no header row");
        assert!(rows.iter().any(|d| d.protocol == "DMR-Terminator"), "no terminator row");
        // Each burst carries the samples it was read from, and its bits read
        // back as AMBE frames.
        assert!(packets.iter().all(|p| p.iq.as_ref().is_some_and(|q| !q.samples.is_empty())), "a burst without its samples");
        let frames = packets
            .iter()
            .filter_map(|p| match &p.body {
                common::PacketBody::Frame(b) => burst_voice_frames(b),
                _ => None,
            })
            .count();
        assert_eq!(frames, voice.len());

        // The vocoder is what turns the AMBE frames into samples, so speech
        // is only asserted where it is built in. Ten superframes of it.
        if cfg!(feature = "ambe") {
            let secs = live as f64 / VOICE_HZ;
            assert!(secs > 3.0, "decoded {secs:.2} s of speech, expected the whole over");
        }
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
            return Err(common::Error::other("dmr needs its channel inside the span"));
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
        self.audio_rate = audio_rate;
        self.ring.clear();
        self.ring_base = 0;

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
        // Behind the symbol clock by what the framer may still read: a
        // burst and its re-anchoring, plus the filters' delay, in samples.
        self.ring.extend_from_slice(&self.narrow);
        let sps = self.audio_rate / BAUD;
        let keep = ((SYM_BURST + SLOT_STRIDE + MAX_MISSES as usize * SLOT_STRIDE) as f64 * sps) as usize;
        if self.ring.len() > keep * 2 {
            let drop = self.ring.len() - keep;
            self.ring.drain(..drop);
            self.ring_base += drop;
        }
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
        let had_voice = events.iter().any(|e| matches!(e, DmrEvent::Voice { .. }));
        let mut packets = Vec::new();
        for e in events {
            match e {
                DmrEvent::Voice { at, bits, frames, pos } => {
                    if !self.talking {
                        self.lc = None;
                        self.talking = true;
                    }
                    self.idle_bursts = 0;
                    let audio = self.decode_voice(&frames);
                    packets.push(self.packet(at, pos, &bits, audio));
                }
                DmrEvent::Lc(lc) => self.lc = Some(lc),
                DmrEvent::Data { at, bits, data_type } => {
                    // The link control a header or terminator carries is in
                    // the event before this one, so the packet has it.
                    packets.push(self.packet(at, POS_DATA, &bits, None));
                    match data_type {
                        // The terminator is the transmission saying it is
                        // over, which is the only end that is not a guess.
                        Some(dmr::DT_TERMINATOR_LC) => {
                            self.talking = false;
                            self.lc = None;
                        }
                        Some(dmr::DT_VOICE_LC_HEADER) => {
                            self.talking = true;
                            self.idle_bursts = 0;
                            self.silent_samples = 0;
                        }
                        _ => {
                            // Signalling with no voice between it means the
                            // channel has moved on without a terminator.
                            if self.talking {
                                self.idle_bursts += 1;
                                if self.idle_bursts >= 8 {
                                    self.talking = false;
                                    self.lc = None;
                                }
                            }
                        }
                    }
                }
            }
        }

        // A carrier that drops after the last voice burst leaves no
        // terminator: the link control it was under is let go after enough
        // input time with no voice, so the next transmission does not
        // inherit it. The over itself ends downstream, where a run of bursts
        // with nothing after it is the end of one.
        if self.talking {
            if had_voice {
                self.silent_samples = 0;
            } else {
                self.silent_samples += iq.len() as u64;
                if self.silent_samples as f64 >= self.in_rate * OVER_SILENCE_S {
                    self.talking = false;
                    self.lc = None;
                    self.silent_samples = 0;
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
        outputs[OUT_PACKETS].packets_mut().extend(packets);
        Ok(())
    }

    fn reset(&mut self) {
        self.sync.reset();
        self.framer.reset();
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.rrc.reset();
        self.voice_now.clear();
        self.talking = false;
        self.lc = None;
        self.idle_bursts = 0;
        self.ring.clear();
        self.ring_base = 0;
        self.silent_samples = 0;
        self.vocoder.reset();
    }
}
