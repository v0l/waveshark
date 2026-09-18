//! NXDN as a graph node.
//!
//! The same front end as P25 and DMR, because the waveform is the same one:
//! the channel is narrowband FM, so the node mixes it down, filters it,
//! discriminates it and hands the result to `dsp::c4fm::SymbolClock`. NXDN is
//! FDMA, so what comes out is one continuous stream of symbols with a 20 bit
//! sync word every 192 of them, and the framer hunts that sync.
//!
//! Two channel widths, one node: 12.5 kHz is 4800 symbols a second and
//! 6.25 kHz is 2400, and nothing else differs. Which one a transmitter used
//! is not in the bits, so it is the clock that says, and the width the
//! scanner table placed the decoder at is what sets the clock.
//!
//! What a frame says is read by `decode::nxdn`: the link information channel,
//! then the slow channel's radio access number, then the voice call message
//! off a stolen half frame or off four slow channels in a row.
//!
//! No speech: the AMBE+2 vocoder is not wired here, so a voice frame reaches
//! the bus as a packet naming the call and the channel time it took, and the
//! call list gets its airtime with no audio behind it.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use common::bands::Usage;
use decode::nxdn::{self, Cipher, MessageType, Superframe};
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// A UK business radio channel in the 12.5 kHz part of the band, and only the
/// default before the scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 453_050_000.0;

/// The wide channel and its symbol rate, and the narrow one and its: 9600 and
/// 4800 bit/s over two bits a symbol (NXDN TS 1-A Table 2.3-1).
pub const WIDE_HZ: f64 = 12_500.0;
pub const NARROW_HZ: f64 = 6_250.0;
pub const WIDE_BAUD: f64 = 4_800.0;
pub const NARROW_BAUD: f64 = 2_400.0;

/// Discriminator output rate: ten samples a symbol at 4800 baud.
const AUDIO_HZ: f64 = 48_000.0;

/// Roll-off of the root raised cosine NXDN shapes its symbols with, and so of
/// the matched filter here (TS 1-A clause 3.4).
const RRC_ALPHA: f64 = 0.2;

/// Symbols held before the framer will slice: two frames, because the four
/// levels are fitted over the window and a sync word carries only three of
/// them.
const WINDOW: usize = 2 * nxdn::FRAME_DIBITS;

/// Channel samples kept behind the symbol clock, in seconds, so a frame's own
/// samples are still there when its packet is built.
///
/// A frame is 40 ms, and the framer reads one up to three frames after its
/// symbols arrived, but the ring also has to outlast one input block: a
/// second of samples can arrive in a single call from a file, and four
/// frames of history is then gone before anything in it is asked for.
const KEEP_S: f64 = 1.0;

/// Tag identifying a packet body this node wrote. "NX".
const NXDN_TAG: [u8; 2] = *b"NX";

/// Tag, LICH, flags, RAN, message type, call type, source, destination,
/// cipher, key, voice channels.
const HEAD_LEN: usize = 2 + 1 + 1 + 1 + 1 + 1 + 2 + 2 + 1 + 1 + 1;

const FLAG_OUTBOUND: u8 = 0x01;
const FLAG_HAVE_RAN: u8 = 0x02;
const FLAG_HAVE_MSG: u8 = 0x04;
const FLAG_HAVE_CALL: u8 = 0x08;
const FLAG_GROUP: u8 = 0x10;
const FLAG_EMERGENCY: u8 = 0x20;
const FLAG_ENCRYPTED: u8 = 0x40;
const FLAG_NARROW: u8 = 0x80;

/// Speech is AMBE+2 at 3600 bit/s, which is what both channel widths carry.
const CODEC: &str = "AMBE+2 3600";

/// Nominal outer-symbol deviation for a width: +-2400 Hz at 12.5 kHz and
/// +-1050 at 6.25 (TS 1-A Table 3.3-1). Nothing downstream depends on the
/// exact value, since the slicer fits its own levels.
fn deviation_hz(baud: f64) -> f64 {
    if baud >= WIDE_BAUD { 2_400.0 } else { 1_050.0 }
}

/// One-sided filter cutoff, wide enough for the outer symbols and the
/// transmitter's drift.
fn cutoff_hz(baud: f64) -> f64 {
    if baud >= WIDE_BAUD { 7_000.0 } else { 3_500.0 }
}

pub fn width_for(baud: f64) -> f64 {
    if baud >= WIDE_BAUD { WIDE_HZ } else { NARROW_HZ }
}

/// What one frame turned out to be, and where the speech in it puts the call.
pub struct NxdnFrame {
    /// Absolute symbol index the sync word began at.
    pub at: usize,
    pub frame: nxdn::Frame,
    /// The message, off this frame's stolen half or off the superframe the
    /// slow channel just completed.
    pub message: Option<nxdn::Message>,
}

fn encode_frame(f: &NxdnFrame, narrow: bool) -> Vec<u8> {
    let mut v = NXDN_TAG.to_vec();
    v.push(f.frame.lich.raw);
    let mut flags = 0u8;
    if f.frame.lich.outbound {
        flags |= FLAG_OUTBOUND;
    }
    if narrow {
        flags |= FLAG_NARROW;
    }
    let ran = f.frame.ran().inspect(|_| flags |= FLAG_HAVE_RAN).unwrap_or(0);
    let call = f.message.as_ref().and_then(|m| m.call);
    if let Some(c) = &call {
        flags |= FLAG_HAVE_CALL;
        if c.call_type.group() {
            flags |= FLAG_GROUP;
        }
        if c.emergency {
            flags |= FLAG_EMERGENCY;
        }
        if c.cipher != Cipher::Clear {
            flags |= FLAG_ENCRYPTED;
        }
    }
    if f.message.is_some() {
        flags |= FLAG_HAVE_MSG;
    }
    v.push(flags);
    v.push(ran);
    v.push(match f.message.as_ref().map(|m| m.kind) {
        Some(MessageType::Other(t)) => t,
        Some(k) => message_code(k),
        None => 0xff,
    });
    v.push(call.map(|c| call_code(c.call_type)).unwrap_or(0xff));
    v.extend_from_slice(&call.map(|c| c.source).unwrap_or(0).to_be_bytes());
    v.extend_from_slice(&call.map(|c| c.dest).unwrap_or(0).to_be_bytes());
    v.push(call.map(|c| cipher_code(c.cipher)).unwrap_or(0));
    v.push(call.map(|c| c.key_id).unwrap_or(0));
    v.push(f.frame.voice_slots as u8);
    if let Some(m) = &f.message {
        v.extend_from_slice(&m.bytes);
    }
    v
}

fn message_code(k: MessageType) -> u8 {
    match k {
        MessageType::VCall => 0x01,
        MessageType::VCallIv => 0x03,
        MessageType::VCallAssign => 0x04,
        MessageType::VCallAssignDup => 0x05,
        MessageType::TxReleaseExt => 0x07,
        MessageType::TxRelease => 0x08,
        MessageType::DCallHeader => 0x09,
        MessageType::DCallData => 0x0b,
        MessageType::DCallAck => 0x0c,
        MessageType::HeadDelay => 0x0f,
        MessageType::Idle => 0x10,
        MessageType::Disconnect => 0x11,
        MessageType::Other(v) => v,
    }
}

fn call_code(c: decode::nxdn::CallType) -> u8 {
    use decode::nxdn::CallType;
    match c {
        CallType::Broadcast => 0,
        CallType::Group => 1,
        CallType::Idle => 2,
        CallType::Session => 3,
        CallType::Individual => 4,
        CallType::Interconnect => 6,
        CallType::SpeedDial => 7,
        CallType::Other(v) => v,
    }
}

fn cipher_code(c: Cipher) -> u8 {
    match c {
        Cipher::Clear => 0,
        Cipher::Scrambler => 1,
        Cipher::Des => 2,
        Cipher::Aes => 3,
    }
}

/// How long one frame holds the channel, at a width: 384 bits at 9600 or
/// 4800 bit/s.
fn frame_seconds(narrow: bool) -> f64 {
    nxdn::FRAME_DIBITS as f64 / if narrow { NARROW_BAUD } else { WIDE_BAUD }
}

/// What the packet log calls the row: the message where the frame carried
/// one, speech where it carried that, and NXDN alone where all that read was
/// the slow channel.
fn row_label(have_message: bool, kind: MessageType, voice: bool) -> &'static str {
    if !have_message {
        return if voice { "NXDN-Voice" } else { "NXDN" };
    }
    match kind {
        MessageType::VCall => "NXDN-VCALL",
        MessageType::VCallIv => "NXDN-VCALL_IV",
        MessageType::VCallAssign => "NXDN-VCALL_ASSGN",
        MessageType::VCallAssignDup => "NXDN-VCALL_ASSGN_DUP",
        MessageType::TxRelease => "NXDN-TX_REL",
        MessageType::TxReleaseExt => "NXDN-TX_REL_EXT",
        MessageType::Disconnect => "NXDN-DISC",
        MessageType::DCallHeader => "NXDN-DCALL_HDR",
        MessageType::DCallData => "NXDN-DCALL_DATA",
        MessageType::DCallAck => "NXDN-DCALL_ACK",
        MessageType::HeadDelay => "NXDN-HEAD_DLY",
        MessageType::Idle => "NXDN-IDLE",
        MessageType::Other(_) => "NXDN-Message",
    }
}

/// Recognise and describe an NXDN row for the packet log. `None` for anything
/// this node did not write, so it is safe to try on every frame.
///
/// A frame carrying speech is that much of the channel and says so; where it
/// named the call it names the talkgroup and the radio, which is what puts it
/// in the call list rather than only in the log. Nothing here is written by a
/// person, so nothing is marked as written.
pub fn nxdn_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if bytes.len() < HEAD_LEN || bytes[..2] != NXDN_TAG {
        return None;
    }
    let lich = nxdn::Lich::from_dibits(&nxdn::Lich::dibits(bytes[2] >> 1))?;
    let flags = bytes[3];
    let ran = bytes[4];
    let kind = MessageType::from_bits(bytes[5]);
    let call_type = decode::nxdn::CallType::from_bits(bytes[6]);
    let source = u16::from_be_bytes([bytes[7], bytes[8]]);
    let dest = u16::from_be_bytes([bytes[9], bytes[10]]);
    let cipher = Cipher::from_bits(bytes[11]);
    let key_id = bytes[12];
    let voice_slots = bytes[13];
    let narrow = flags & FLAG_NARROW != 0;

    let mut fields: Vec<(String, Value)> = vec![
        ("channel".to_string(), Value::Text(lich.rf.label().to_string())),
        (
            "direction".to_string(),
            Value::Text(if flags & FLAG_OUTBOUND != 0 { "out" } else { "in" }.to_string()),
        ),
        ("width".to_string(), Value::Text(if narrow { "6.25k" } else { "12.5k" }.to_string())),
    ];
    if flags & FLAG_HAVE_RAN != 0 {
        fields.push(("ran".to_string(), Value::Int(i64::from(ran))));
    }
    if flags & FLAG_HAVE_MSG != 0 {
        fields.push(("message".to_string(), Value::Text(kind.label())));
    }
    if flags & FLAG_HAVE_CALL != 0 {
        fields.push(("from".to_string(), Value::Int(i64::from(source))));
        fields.push(("to".to_string(), Value::Int(i64::from(dest))));
        fields.push(("call_type".to_string(), Value::Text(call_type.label())));
        if flags & FLAG_EMERGENCY != 0 {
            fields.push(("emergency".to_string(), Value::Bool(true)));
        }
        if flags & FLAG_ENCRYPTED != 0 {
            fields.push(("encrypted".to_string(), Value::Bool(true)));
            fields.push(("algorithm".to_string(), Value::Text(cipher.label().to_string())));
            fields.push(("key_id".to_string(), Value::Int(i64::from(key_id))));
        }
    }
    // What the frame carried of speech, not how long it held the channel: a
    // frame with a half stolen for signalling is half a frame of talking.
    let seconds = frame_seconds(narrow) * f64::from(voice_slots) / 4.0;
    if voice_slots > 0 {
        fields.push(("voice".to_string(), Value::Bool(true)));
        fields.push(("codec".to_string(), Value::Text(CODEC.to_string())));
        fields.push(("seconds".to_string(), Value::Float(seconds)));
        fields.push(("live".to_string(), Value::Bool(true)));
    }

    let label = row_label(flags & FLAG_HAVE_MSG != 0, kind, voice_slots > 0);
    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes(label, center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk4)
        // The LICH passed its parity and whatever else is here passed a CRC;
        // nothing reaches this point that did not.
        .with_crc(Some(true));
    if flags & FLAG_HAVE_CALL != 0 {
        use pipeline::event::Party;
        let to = match flags & FLAG_GROUP != 0 {
            true => Party::group(dest.to_string()),
            false => Party::unit(dest.to_string()),
        };
        d.link = Some(pipeline::event::Link::between(Party::unit(source.to_string()), to));
        d.identity = Some(common::Identity::new("nxdn", source.to_string()));
    }
    if voice_slots > 0 {
        d.airtime = Some(common::Airtime {
            seconds,
            voice: true,
            live: true,
            secrecy: match flags & FLAG_ENCRYPTED != 0 {
                true => common::Secrecy::Encrypted(None),
                false => common::Secrecy::Clear,
            },
            codec: Some(CODEC),
        });
    }
    Some(d)
}

/// Finds frames in the symbol stream and reads what they carry.
///
/// A rolling window of symbol values with an absolute index, so a frame whose
/// sync arrived in one block is read when the rest of it arrives in the next.
/// The slow channel's quarters are assembled here rather than in the decoder
/// because only the framer sees consecutive frames.
struct Framer {
    marks: Vec<f32>,
    base: usize,
    scan: usize,
    /// Which way up the discriminator is, once a frame has settled it.
    polarity: Option<bool>,
    superframe: Superframe,
}

impl Framer {
    fn new() -> Self {
        Self {
            marks: Vec::new(),
            base: 0,
            scan: 0,
            polarity: None,
            superframe: Superframe::default(),
        }
    }

    fn reset(&mut self) {
        self.marks.clear();
        self.base = 0;
        self.scan = 0;
        self.polarity = None;
        self.superframe.reset();
    }

    /// Level index to dibit: NXDN sends +3 as 01, +1 as 00, -1 as 10 and -3
    /// as 11 (TS 1-A Table 3.3-1), and the slicer numbers levels up the band.
    fn dibit(level: u8, flip: bool) -> u8 {
        match if flip { 3 - level } else { level } {
            3 => 1,
            2 => 0,
            1 => 2,
            _ => 3,
        }
    }

    /// Append recovered symbols and pull out the frames they complete.
    fn push(&mut self, syms: &[f32], out: &mut Vec<NxdnFrame>) {
        self.marks.extend_from_slice(syms);
        if self.marks.len() < WINDOW {
            return;
        }
        let Some(levels) = dsp::c4fm::slice(&self.marks) else {
            return;
        };
        let polarities: [bool; 2] = match self.polarity {
            Some(p) => [p, p],
            None => [false, true],
        };
        let mut from = self.scan.saturating_sub(self.base);
        for flip in polarities {
            let dibits: Vec<u8> = levels.iter().map(|l| Self::dibit(*l, flip)).collect();
            let mut at = from;
            let mut found = false;
            while let Some(f) = nxdn::find(&dibits, at) {
                at = f.at + nxdn::FRAME_DIBITS;
                found = true;
                self.polarity = Some(flip);
                // The quarter goes in whether or not this frame also stole a
                // half for the same message: leaving it out on the frame
                // that opens a call loses that whole superframe.
                let whole =
                    f.sacch.filter(|_| f.lich.superframe()).and_then(|s| self.superframe.push(&s));
                let message = f
                    .facch1
                    .first()
                    .cloned()
                    .or_else(|| whole.as_deref().and_then(nxdn::message))
                    .filter(|m| m.kind != MessageType::Idle);
                out.push(NxdnFrame { at: self.base + f.at, frame: f, message });
            }
            from = at;
            if found || self.polarity.is_some() {
                break;
            }
        }
        // Where the hunt reached, less a frame of history so a sync word
        // straddling two blocks is still found.
        self.scan = self.base + from;
        let keep = self.scan.saturating_sub(nxdn::FRAME_DIBITS);
        if keep > self.base {
            let drop = (keep - self.base).min(self.marks.len());
            self.marks.drain(..drop);
            self.base += drop;
        }
    }
}

pub struct NxdnNode {
    channel_hz: f64,
    baud: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    rrc: FirDecimReal,
    clock: SymbolClock,
    framer: Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    shaped: Vec<f32>,
    syms: Vec<f32>,
    meter: crate::FrameMeter,
    audio_rate: f64,
    accepted: u64,
}

impl Default for NxdnNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, WIDE_BAUD)
    }
}

impl NxdnNode {
    pub fn new(channel_hz: f64, baud: f64) -> Self {
        let baud = if baud >= WIDE_BAUD { WIDE_BAUD } else { NARROW_BAUD };
        Self {
            channel_hz,
            baud,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, cutoff_hz(baud), 60.0),
            fm: FmDemod::new(AUDIO_HZ, deviation_hz(baud)),
            rrc: FirDecimReal::new(rrc_taps(AUDIO_HZ / baud, RRC_ALPHA, 8), 1),
            clock: SymbolClock::new(AUDIO_HZ, baud),
            framer: Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            shaped: Vec::new(),
            syms: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, KEEP_S),
            audio_rate: AUDIO_HZ,
            accepted: 0,
        }
    }

    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    fn width_hz(&self) -> f64 {
        width_for(self.baud)
    }

    /// One frame as a packet: what it said, the level it was heard at and the
    /// samples it was sliced from, found by the frame's symbol index.
    fn packet(&mut self, frame: &NxdnFrame) -> common::Packet {
        let at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        self.accepted += 1;
        let sps = self.audio_rate / self.baud;
        let bytes = encode_frame(frame, self.baud < WIDE_BAUD);
        let start = (frame.at as f64 * sps) as u64;
        let len = (nxdn::FRAME_DIBITS as f64 * sps) as usize;
        let snr_db = self.meter.snr_db_at(start, len);
        let measured = self.meter.frame_measured(bytes, start, len, snr_db);
        common::Packet::of_frame(at_us, self.width_hz() as u32, measured)
    }
}

impl Simple for NxdnNode {
    fn name(&self) -> &str {
        "nxdn"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("nxdn reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        let width = self.width_hz();
        if (self.channel_hz - center).abs() > rate / 2.0 - width / 2.0 {
            return Err(common::Error::other("nxdn needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, cutoff_hz(self.baud), 60.0);
        self.fm = FmDemod::new(audio_rate, deviation_hz(self.baud));
        self.rrc = FirDecimReal::new(rrc_taps(audio_rate / self.baud, RRC_ALPHA, 8), 1);
        self.clock = SymbolClock::new(audio_rate, self.baud);
        self.framer = Framer::new();
        self.audio_rate = audio_rate;
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, KEEP_S);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = width;
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);

        let raw = std::mem::take(&mut self.audio);
        let mut shaped = std::mem::take(&mut self.shaped);
        shaped.clear();
        self.rrc.process(&raw, &mut shaped);
        self.audio = raw;

        let mut syms = std::mem::take(&mut self.syms);
        syms.clear();
        self.clock.push(&shaped, &mut syms);
        self.shaped = shaped;

        let mut frames = Vec::new();
        self.framer.push(&syms, &mut frames);
        self.syms = syms;

        for f in &frames {
            // A frame whose LICH passed and whose payload did not says only
            // that something is on the channel, and on noise that is all a
            // false sync word ever says. See `nxdn::noise_is_not_a_frame`.
            if !f.frame.read_anything() {
                continue;
            }
            let p = self.packet(f);
            o.packets_mut().push(p);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.rrc.reset();
        self.clock.reset();
        self.framer.reset();
        self.meter.reset();
    }
}

pub struct Nxdn;

impl Protocol for Nxdn {
    fn id(&self) -> &'static str {
        "nxdn"
    }
    fn label(&self) -> &'static str {
        "NXDN"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["nexedge", "idas", "nxdn96", "nxdn48"]
    }
    /// A business radio channel on the land mobile allocations: NXDN is on
    /// VHF and UHF, and what makes one an NXDN channel is what is keyed on it.
    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility])
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        nxdn_decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[WIDE_HZ, NARROW_HZ],
            min_rate_hz: WIDE_HZ,
            feed_rate_hz: 192_000.0,
            span_wide: false,
            families: &[],
        }
    }
    /// Which of the two widths a source is, decided at the geometric mean of
    /// them: a 6.25 kHz channel measures wider than it is and a 12.5 kHz one
    /// narrower, and the halfway point on a log scale splits them evenly.
    fn widths_for(&self, _hz: f64, source_width_hz: f64) -> Vec<f64> {
        match source_width_hz < (WIDE_HZ * NARROW_HZ).sqrt() {
            true => vec![NARROW_HZ],
            false => vec![WIDE_HZ],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Packets]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        let baud = match at.width_hz < (WIDE_HZ * NARROW_HZ).sqrt() {
            true => NARROW_BAUD,
            false => WIDE_BAUD,
        };
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz).f(BAUD, baud)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";
/// Symbols a second, which is the channel width in the only form the decoder
/// cares about.
const BAUD: &str = "baud";

pub const DESC: StageDesc = StageDesc {
    name: "nxdn",
    summary: "One NXDN channel: 4-level FSK at 4800 or 2400 baud, RAN and call ids",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(NxdnNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), s.f64_or(BAUD, WIDE_BAUD))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::nxdn::{Call, CallType, Sacch, Steal};
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    fn group_call() -> Call {
        Call {
            call_type: CallType::Group,
            source: 1234,
            dest: 5678,
            cipher: Cipher::Clear,
            key_id: 0,
            emergency: false,
            duplex: false,
        }
    }

    /// Key a run of dibits as 4-level FSK at `rate`, at the given offset from
    /// the tuner's centre: the deviation NXDN keys, through an integrator.
    fn keyed(dibits: &[u8], rate: f64, baud: f64, offset_hz: f64, noise: f32) -> Vec<common::C32> {
        let sps = rate / baud;
        let dev = deviation_hz(baud);
        let mut out = Vec::with_capacity((dibits.len() as f64 * sps) as usize);
        let mut phase = 0.0f64;
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut noise_sample = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 40) as f32 / 8_388_608.0 - 0.5) * noise
        };
        for d in dibits {
            let hz = match d & 3 {
                1 => dev,
                0 => dev / 3.0,
                2 => -dev / 3.0,
                _ => -dev,
            };
            for _ in 0..sps.round() as usize {
                phase += std::f64::consts::TAU * (hz + offset_hz) / rate;
                let (s, c) = phase.sin_cos();
                out.push(common::C32::new(c as f32 + noise_sample(), s as f32 + noise_sample()));
            }
        }
        out
    }

    /// Run IQ through the node and return the rows it wrote.
    fn replay(iq: &[common::C32], rate: f64, center: f64, hz: f64, baud: f64) -> Vec<Decoded> {
        let mut node = NxdnNode::new(hz, baud);
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut rows = Vec::new();
        for chunk in iq.chunks(16_384) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(ps) = &out {
                for p in ps {
                    if let common::PacketBody::Frame(f) = &p.body {
                        assert!(p.rssi_dbfs().is_finite() && p.snr_db().is_finite());
                        assert!(p.samples().is_some_and(|q| !q.samples.is_empty()));
                        rows.extend(nxdn_decoded(&f.bytes, common::Hz(p.center_hz())));
                    }
                }
            }
        }
        rows
    }

    /// Symbols with all four levels and no sync word in them, keyed either
    /// side of a call so the clock is running as a receiver's would be.
    /// Random rather than a repeating pattern, which a timing loop locks to
    /// the wrong phase of.
    fn tail() -> Vec<u8> {
        let mut seed = 0x51ed_2701_dead_1234u64;
        (0..1_500)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (seed >> 33) as u8 & 3
            })
            .collect()
    }

    /// A call as a radio sends it: a voice frame with the call message in a
    /// stolen half, then frames of speech with the message going out over the
    /// slow channel a quarter at a time.
    fn call_frames(ran: u8, call: &Call, frames: usize) -> Vec<u8> {
        let message = decode::nxdn::call_bits(MessageType::VCall, call);
        let mut padded = message.clone();
        padded.resize(72, false);
        let mut dibits = tail();
        for n in 0..frames {
            let quarter = n % 4;
            let mut data = [false; 18];
            data.copy_from_slice(&padded[quarter * 18..quarter * 18 + 18]);
            let sacch = Sacch { ran, structure: 3 - quarter as u8, data };
            let steal = if n == 0 { Steal::First } else { Steal::None };
            let mut payload = sacch.air();
            for (half, stolen) in steal.stolen().iter().enumerate() {
                let _ = half;
                match stolen {
                    true => payload.extend(decode::nxdn::facch1_air(&message)),
                    false => payload.extend(std::iter::repeat_n(false, 144)),
                }
            }
            dibits.extend(decode::nxdn::keyed(decode::nxdn::rdch_lich(steal, true), &payload));
        }
        dibits.extend(tail());
        dibits
    }

    #[test]
    fn negotiates_a_channel_inside_the_span() {
        let mut n = NxdnNode::default();
        assert!(n.negotiate(&spec(2_048_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_048_000.0, 460_000_000.0)).is_err());
    }

    #[test]
    fn labels_only_its_own_frames() {
        let frame = NxdnFrame {
            at: 0,
            frame: nxdn::read(
                &decode::nxdn::keyed(
                    decode::nxdn::rdch_lich(Steal::None, true),
                    &Sacch { ran: 12, structure: 3, data: [false; 18] }.air(),
                )[10..],
            )
            .expect("a frame"),
            message: decode::nxdn::message(&decode::nxdn::call_bits(
                MessageType::VCall,
                &group_call(),
            )),
        };
        let bytes = encode_frame(&frame, false);
        let d = nxdn_decoded(&bytes, Hz(453_050_000)).expect("an NXDN row");
        assert_eq!(d.protocol, "NXDN-VCALL");
        let get = |k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert_eq!(get("ran"), "12");
        assert_eq!(get("from"), "1234");
        assert_eq!(get("to"), "5678");
        assert_eq!(get("call_type"), "group");
        assert_eq!(get("width"), "12.5k");
        assert_eq!(get("channel"), "RDCH");
        let air = d.airtime.as_ref().expect("a voice frame with no airtime");
        assert_eq!(air.seconds, 0.04, "384 bits at 9600 bit/s is 40 ms a frame");
        assert!(air.voice && air.live);
        assert_eq!(air.codec, Some(CODEC));
        // Nobody wrote this, so it is not a message.
        assert!(!d.written);
        assert!(nxdn_decoded(b"random", Hz(0)).is_none());
        assert!(nxdn_decoded(b"NX", Hz(0)).is_none());
    }

    /// A keyed call off the node's own front end: every frame of it read, the
    /// talkgroup and the radio off the stolen half, and the same call again
    /// once the slow channel has sent all four quarters.
    #[test]
    fn reads_a_keyed_call_through_the_front_end() {
        let rate = 96_000.0;
        let iq = keyed(&call_frames(9, &group_call(), 8), rate, WIDE_BAUD, 0.0, 0.0);
        let rows = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.len(), 8, "eight frames keyed, {} read", rows.len());
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert!(rows.iter().all(|d| get(d, "ran") == "9"), "every frame names the system");
        let named: Vec<&Decoded> = rows.iter().filter(|d| get(d, "from") == "1234").collect();
        assert_eq!(named.len(), 3, "the stolen half and two completed superframes");
        assert!(named.iter().all(|d| get(d, "to") == "5678"));
        assert!(named.iter().all(|d| get(d, "call_type") == "group"));
        // Nothing is encrypted, so no row claims a key.
        assert_eq!(rows.iter().filter(|d| get(d, "encrypted") == "true").count(), 0);
        // Every frame carries speech, and the first carries half as much
        // because it stole a half for the call message.
        assert_eq!(rows.iter().filter(|d| get(d, "voice") == "true").count(), 8);
        let seconds: Vec<f64> =
            rows.iter().filter_map(|d| d.airtime.as_ref()).map(|a| a.seconds).collect();
        assert_eq!(seconds.len(), 8);
        assert_eq!(seconds[0], 0.02, "two of four voice channels in the opening frame");
        assert!(seconds[1..].iter().all(|s| *s == 0.04), "40 ms of speech a frame after it");
    }

    /// Off frequency and in noise, which is what a receiver actually hands
    /// the node. Measured: 2.5 kHz off with noise at a tenth of the carrier
    /// still reads every frame.
    #[test]
    fn reads_a_call_off_frequency_and_in_noise() {
        let rate = 96_000.0;
        let iq = keyed(&call_frames(31, &group_call(), 8), rate, WIDE_BAUD, -2_500.0, 0.1);
        let rows = replay(&iq, rate, DEFAULT_HZ + 2_500.0, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.len(), 8);
        let rans = rows
            .iter()
            .filter(|d| d.fields.iter().any(|(k, v)| k == "ran" && v.to_string() == "31"))
            .count();
        assert_eq!(rans, 8);
    }

    /// The narrow channel is the same frame at half the clock.
    #[test]
    fn reads_the_six_and_a_quarter_kilohertz_channel() {
        let rate = 96_000.0;
        let iq = keyed(&call_frames(1, &group_call(), 6), rate, NARROW_BAUD, 0.0, 0.0);
        let rows = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, NARROW_BAUD);
        assert_eq!(rows.len(), 6);
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert!(rows.iter().all(|d| get(d, "width") == "6.25k"));
        // The opening frame stole a half, so it is half of the 80 ms a
        // 6.25 kHz frame holds the channel for.
        assert_eq!(rows[0].airtime.as_ref().map(|a| a.seconds), Some(0.04));
        assert_eq!(rows[1].airtime.as_ref().map(|a| a.seconds), Some(0.08), "80 ms a frame");
    }

    /// An enciphered call says so, and says under which key, without
    /// pretending to any speech it cannot read.
    #[test]
    fn an_enciphered_call_names_its_key() {
        let call = Call {
            cipher: Cipher::Aes,
            key_id: 21,
            emergency: true,
            call_type: CallType::Individual,
            source: 40001,
            dest: 40002,
            duplex: false,
        };
        let rate = 96_000.0;
        let iq = keyed(&call_frames(3, &call, 4), rate, WIDE_BAUD, 0.0, 0.0);
        let rows = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.len(), 4);
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        let named: Vec<&Decoded> = rows.iter().filter(|d| get(d, "from") == "40001").collect();
        assert_eq!(named.len(), 2, "the stolen half and one completed superframe");
        assert!(named.iter().all(|d| get(d, "algorithm") == "AES"));
        assert!(named.iter().all(|d| get(d, "key_id") == "21"));
        assert!(named.iter().all(|d| get(d, "emergency") == "true"));
        assert!(named.iter().all(|d| get(d, "call_type") == "individual"));
        assert_eq!(
            named[0].airtime.as_ref().map(|a| a.secrecy.clone()),
            Some(common::Secrecy::Encrypted(None))
        );
    }

    /// Two minutes of noise, and nothing said about any of it.
    #[test]
    fn noise_is_not_a_call() {
        let rate = 96_000.0;
        let mut seed = 0xdead_beef_cafe_f00du64;
        let iq: Vec<common::C32> = (0..(rate * 120.0) as usize)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let i = (seed >> 40) as f32 / 8_388_608.0 - 0.5;
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let q = (seed >> 40) as f32 / 8_388_608.0 - 0.5;
                common::C32::new(i, q)
            })
            .collect();
        let rows = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.len(), 0, "two minutes of noise read as {} frames", rows.len());
    }
}
