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
//! What is not here yet: the embedded Link Control (who is talking, to which
//! talkgroup) needs a BPTC decode of the signalling bursts, and slot 2 is not
//! separated from slot 1. Both are decode-side work that can grow on top of
//! this without moving where the processing lives.

use common::Result;
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// Tag byte identifying a packet body this node wrote, so the log labeler can
/// tell a DMR voice-over row from any other `Frame` and refuse everything
/// else. "DV" for DMR voice.
const DMR_TAG: [u8; 2] = *b"DV";

/// Serialise a finished voice over: the tag and the burst count, which is the
/// only fact we have without the embedded Link Control decode. Each burst is
/// one 60 ms slot of speech.
fn encode_voice_over(bursts: u32) -> Vec<u8> {
    let mut v = DMR_TAG.to_vec();
    v.extend_from_slice(&bursts.to_be_bytes());
    v
}

/// Recognise and describe a DMR voice-over row for the packet log. Returns
/// `None` for anything this node did not write, so it is safe to try on every
/// frame the way `m17_decoded` is.
pub fn dmr_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if bytes.len() != 6 || bytes[..2] != DMR_TAG {
        return None;
    }
    let bursts = u32::from_be_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]);
    // One burst is one 60 ms slot on this logical channel.
    let seconds = f64::from(bursts) * 0.06;
    let fields = vec![
        ("seconds".to_string(), Value::Float(seconds)),
        ("bursts".to_string(), Value::Int(i64::from(bursts))),
    ];
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

/// One-sided filter cutoff. A compliant DMR signal is ~9.5 kHz wide (±4.75),
/// but handsets over-deviate badly (a DM-1701 measured ±6.3 kHz outer
/// levels, ~14 kHz occupied), so pass well past nominal or the outer symbols
/// are clipped and the four-level eye closes.
const FILTER_CUTOFF_HZ: f64 = 10_000.0;

/// Discriminator output rate: ~10 samples per symbol at 4800 baud.
const AUDIO_HZ: f64 = 48_000.0;

/// Symbol rate.
const BAUD: f64 = 4_800.0;

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

/// Longest transmission kept as audio, in seconds.
#[cfg(feature = "ambe")]
const MAX_VOICE_SECONDS: f64 = 120.0;

/// The DMR sync words as level-index strings (0=-3,1=-1,2=+1,3=+3), derived
/// from the canonical hex by mapping each dibit 01,00,10,11. Voice bursts and
/// data bursts carry different words, which is how a voice superframe is told
/// from signalling.
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

/// Finds sync words in the symbol stream and reads voice bursts out.
///
/// Holds a rolling window of symbol values with an absolute index, so a sync
/// found near the end of one block can still anchor a superframe whose later
/// bursts arrive in the next.
struct Framer {
    /// Symbol values, oldest first.
    marks: Vec<f32>,
    /// Absolute index of `marks[0]`.
    base: usize,
    /// Next absolute index to test for a sync word.
    scan: usize,
    /// Sync polarity once locked: the discriminator's sign is receiver-set.
    polarity: Option<bool>,
    /// Parsed sync patterns as level indices.
    patterns: Vec<(&'static str, Vec<u8>, bool)>,
}

/// One thing the framer found.
pub enum DmrEvent {
    /// A voice burst: three 72-bit AMBE frames, 9 bytes each.
    Voice([[u8; 9]; 3]),
    /// A voice transmission began (a voice sync after silence).
    VoiceStart,
    /// A data/signalling burst was seen (LC header, terminator, idle).
    Data,
}

impl Framer {
    fn new() -> Self {
        let patterns = SYNCS
            .iter()
            .map(|(n, p, v)| (*n, p.bytes().map(|c| c - b'0').collect::<Vec<u8>>(), *v))
            .collect();
        Self { marks: Vec::new(), base: 0, scan: 0, polarity: None, patterns }
    }

    fn reset(&mut self) {
        self.marks.clear();
        self.base = 0;
        self.scan = 0;
        self.polarity = None;
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

    /// Read the three AMBE frames from a burst whose sync starts at absolute
    /// index `sync_at`. Returns None if that burst is not fully buffered.
    fn read_voice(&self, sync_at: usize, flip: bool) -> Option<[[u8; 9]; 3]> {
        let start = sync_at.checked_sub(SYM_PAYLOAD)?;
        let end = sync_at + SYM_SYNC + SYM_PAYLOAD;
        if start < self.base || end > self.base + self.marks.len() {
            return None;
        }
        let s = start - self.base;
        // Fit levels over the whole burst, then take the payload symbols.
        let centers = Self::centers(&self.marks[s..s + SYM_BURST]);
        let lv = Self::apply(&self.marks[s..s + SYM_BURST], &centers, flip);
        let mut bits: Vec<u8> = Vec::with_capacity(216);
        for &l in &lv[0..SYM_PAYLOAD] {
            bits.extend_from_slice(&Self::dibit(l));
        }
        for &l in &lv[SYM_PAYLOAD + SYM_SYNC..SYM_BURST] {
            bits.extend_from_slice(&Self::dibit(l));
        }
        let mut frames = [[0u8; 9]; 3];
        for (f, frame) in frames.iter_mut().enumerate() {
            for (b, byte) in bits[f * 72..(f + 1) * 72].chunks(8).enumerate() {
                let mut v = 0u8;
                for (k, &bit) in byte.iter().enumerate() {
                    v |= bit << (7 - k);
                }
                frame[b] = v;
            }
        }
        Some(frames)
    }

    /// Test for a sync word at absolute index `at`, either polarity if not
    /// yet locked. Returns (name, is_voice, flip).
    fn sync_at(&self, at: usize) -> Option<(&'static str, bool, bool)> {
        if at < self.base || at + SYM_SYNC > self.base + self.marks.len() {
            return None;
        }
        // Fit levels over a burst-wide window centred on the sync so all four
        // levels are present; a fit over the sync symbols alone sees only the
        // outer two and mismaps everything.
        let rel = at - self.base;
        let lo = rel.saturating_sub(SYM_PAYLOAD);
        let hi = (rel + SYM_SYNC + SYM_PAYLOAD).min(self.marks.len());
        let centers = Self::centers(&self.marks[lo..hi]);
        let raw = &self.marks[rel..rel + SYM_SYNC];
        let polarities: &[bool] = match &self.polarity {
            Some(p) => std::slice::from_ref(p),
            None => &[false, true],
        };
        for &flip in polarities {
            let lv = Self::apply(raw, &centers, flip);
            for (name, pat, voice) in &self.patterns {
                let err = lv.iter().zip(pat).filter(|(a, b)| a != b).count();
                if err <= 2 {
                    return Some((name, *voice, flip));
                }
            }
        }
        None
    }

    /// Append recovered symbols and pull out any complete voice superframes.
    fn push(&mut self, syms: &[f32], out: &mut Vec<DmrEvent>) {
        self.marks.extend_from_slice(syms);
        let last = self.base + self.marks.len();
        // Scan for syncs where a whole burst is buffered behind the position.
        while self.scan + SYM_SYNC + SYM_PAYLOAD <= last {
            if self.scan < self.base + SYM_PAYLOAD {
                self.scan = self.base + SYM_PAYLOAD;
                continue;
            }
            if let Some((_name, voice, flip)) = self.sync_at(self.scan) {
                self.polarity = Some(flip);
                if voice {
                    // A voice sync anchors this superframe: bursts A..F at
                    // +288k. Only proceed once all six are buffered.
                    let need = self.scan + SLOT_STRIDE * (SUPERFRAME_BURSTS - 1) + SYM_SYNC + SYM_PAYLOAD;
                    if need > last {
                        break;
                    }
                    out.push(DmrEvent::VoiceStart);
                    for k in 0..SUPERFRAME_BURSTS {
                        if let Some(f) = self.read_voice(self.scan + SLOT_STRIDE * k, flip) {
                            out.push(DmrEvent::Voice(f));
                        }
                    }
                    self.scan += SLOT_STRIDE * SUPERFRAME_BURSTS;
                } else {
                    out.push(DmrEvent::Data);
                    self.scan += SLOT_STRIDE;
                }
            } else {
                self.scan += 1;
            }
        }
        // Drain marks well behind the scan position.
        if self.scan > self.base + SYM_PAYLOAD * 2 {
            let drop = self.scan - self.base - SYM_PAYLOAD * 2;
            if drop <= self.marks.len() {
                self.marks.drain(..drop);
                self.base += drop;
            }
        }
    }
}

pub struct DmrNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    sync: SymbolSync,
    framer: Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    syms: Vec<f32>,
    /// Speech decoded this block, for listening live.
    voice_now: Vec<f32>,
    /// Whole-transmission speech, for the packet log.
    voice: Vec<f32>,
    /// Whether a voice transmission is in progress.
    talking: bool,
    /// Voice bursts in the transmission in progress, for its duration: each
    /// burst is one 60 ms slot of speech.
    voice_bursts: u32,
    /// Bursts since the last voice sync, to notice a transmission ending.
    idle_bursts: u32,
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
            sync: SymbolSync::new(AUDIO_HZ),
            framer: Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            syms: Vec::new(),
            voice_now: Vec::new(),
            voice: Vec::new(),
            talking: false,
            voice_bursts: 0,
            idle_bursts: 0,
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
        Some(std::sync::Arc::new(common::Speech { pcm, rate: VOICE_HZ }))
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
    fn labels_only_its_own_voice_over() {
        let body = encode_voice_over(50);
        let d = dmr_decoded(&body, common::Hz(433_450_000)).expect("a DMR row");
        assert_eq!(d.protocol, "DMR-Voice");
        // 50 bursts x 60 ms = 3.0 s.
        assert!(d.detail.as_deref().unwrap_or_default().contains("seconds=3"), "{:?}", d.detail);
        // Not anyone else's frame.
        assert!(dmr_decoded(b"random", common::Hz(0)).is_none());
        assert!(dmr_decoded(b"DV", common::Hz(0)).is_none());
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
        eprintln!("decoded {live} voice samples ({:.2}s)", live as f32 / VOICE_HZ as f32);
        // This capture holds two voice superframes (the operator spoke
        // briefly): 2 x 6 bursts x 3 frames x 160 samples = 5760. Require a
        // healthy fraction so a regression that loses framing is caught.
        assert!(live >= 4000, "expected the voice superframes to decode, got {live} samples");
    }
}

impl Node for DmrNode {
    fn name(&self) -> &str {
        "dmr"
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
        self.sync = SymbolSync::new(audio_rate);
        self.framer = Framer::new();

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
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);

        self.syms.clear();
        let audio = std::mem::take(&mut self.audio);
        let mut syms = std::mem::take(&mut self.syms);
        self.sync.push(&audio, &mut syms);
        self.audio = audio;

        let mut events = Vec::new();
        self.framer.push(&syms, &mut events);
        self.syms = syms;

        self.voice_now.clear();
        let mut ended = false;
        for e in events {
            match e {
                DmrEvent::VoiceStart => {
                    if !self.talking {
                        self.voice.clear();
                        self.voice_bursts = 0;
                    }
                    self.talking = true;
                    self.idle_bursts = 0;
                }
                DmrEvent::Voice(frames) => {
                    self.decode_voice(&frames);
                    self.voice_bursts += 1;
                    self.idle_bursts = 0;
                }
                DmrEvent::Data => {
                    // Several data bursts with no voice between means the
                    // transmission is over (terminator, then silence).
                    if self.talking {
                        self.idle_bursts += 1;
                        if self.idle_bursts >= 4 {
                            ended = true;
                        }
                    }
                }
            }
        }

        let (from, to) = (None, None);
        outputs[OUT_VOICE].voice_mut().push(common::Voice {
            system: "DMR",
            channel_hz: self.channel_hz,
            to,
            from,
            rate: VOICE_HZ,
            pcm: std::mem::take(&mut self.voice_now),
        });

        if ended {
            let at_us = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_micros() as u64)
                .unwrap_or(0);
            let audio = self.take_voice();
            let bursts = self.voice_bursts;
            self.talking = false;
            self.voice_bursts = 0;
            self.idle_bursts = 0;
            self.accepted += 1;
            outputs[OUT_PACKETS].packets_mut().push(common::Packet {
                at_us,
                center_hz: self.channel_hz as u64,
                bandwidth_hz: CHANNEL_WIDTH_HZ as u32,
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: Some("4FSK"),
                body: common::PacketBody::Frame(encode_voice_over(bursts)),
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
        self.voice.clear();
        self.voice_now.clear();
        self.talking = false;
        self.voice_bursts = 0;
        self.idle_bursts = 0;
        #[cfg(feature = "ambe")]
        {
            self.synth = mbe::ambe::AmbeSynthesizer::new();
        }
    }
}
