//! Every over the receiver hears, kept as audio.
//!
//! The packet log keeps what was on the air as timings and bytes, and speech
//! is deliberately not in it: an hour of a busy repeater is gigabytes, and a
//! frame with a megabyte of audio hanging off it is not evidence a decoder
//! can read. But a voice channel left running overnight is the case this
//! receiver is for, and a transcript is not the recording: a model that heard
//! "fifty two" where somebody said "fifty two zero" has thrown the only copy
//! away. So speech gets a log of its own, in a form small enough to leave on
//! for a week.
//!
//! # A record is a call
//!
//! One record per over: when it started, the channel it was on, the system,
//! who was talking, the group or party called, how long it ran, how loud it
//! was, and the speech itself. That is what a scanner's list shows and what
//! somebody looking for a transmission searches by, so the header is readable
//! without touching the audio.
//!
//! # The format
//!
//! The packet log's shape, on the shared segment writer in
//! [`crate::segments`]: a folder of dated files, appended, trimmed oldest
//! first to keep under a cap.
//!
//! ```text
//! file   := "WSCAL\0" u16 version
//! record := u32 body_len, u8 kind, u8 codec, u16 frames,
//!           u64 at_us, u64 channel_hz, u32 duration_ms, f32 peak,
//!           str system, str from, str to,
//!           [u16 len, opus_packet] * frames
//! str    := u8 len, utf8
//! ```
//!
//! Length first, so a reader that does not know a `kind` skips the record
//! rather than misparsing it, and a receiver killed mid-write costs the last
//! record rather than the file.
//!
//! # Opus, at sixteen kilohertz
//!
//! Speech off a radio channel has nothing above 4 kHz in it: analogue NFM is
//! filtered at 3 kHz and every vocoder here (AMBE, Codec 2, ACELP) is
//! narrowband by construction. So the audio is resampled to 16 kHz mono and
//! encoded at [`BITRATE`], which is 2 kB a second: an hour of somebody
//! actually talking is 7 MB, and a day of a busy talkgroup fits in what a
//! minute of raw IQ takes. Silence between overs costs nothing because a
//! record only exists while somebody is transmitting.
//!
//! Pure Rust ([`opus_rs`]) rather than libopus, so a release build needs no C
//! toolchain and no system codec.

use common::Result;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::path::PathBuf;

use crate::segments::{self, Format, Segments};
use opus_rs::{Application, OpusDecoder, OpusEncoder};

const MAGIC: &[u8; 6] = b"WSCAL\0";
const VERSION: u16 = 1;
const EXT: &str = "wscal";

/// A call: a header and the Opus packets of one over.
pub const KIND_CALL: u8 = 1;
/// Opus, mono, 16 kHz, 20 ms a packet.
pub const CODEC_OPUS: u8 = 1;

/// The rate the audio is kept at. Every voice mode on this receiver is
/// narrowband, so 16 kHz is wider than anything it can hear.
pub const RATE: f64 = 16_000.0;

/// Samples in one Opus packet: 20 ms, which is what the codec is tuned for
/// and small enough that a call cut short loses a syllable rather than a
/// word.
pub const FRAME: usize = 320;

/// Bits a second. Wideband SILK at this rate is clean on speech, and it is
/// the number that makes an overnight watch affordable: 2 kB a second.
pub const BITRATE: i32 = 16_000;

/// How long a call stays open with nothing arriving on it.
///
/// Longer than the gap between two syllables and shorter than the gap
/// between two overs, so a conversation is a file per over rather than one
/// file with the whole net in it.
const HANG_S: f64 = 1.5;

/// A call that never ends is cut here and continues in the next record: a
/// carrier stuck open is otherwise one record that grows until the receiver
/// stops.
const MAX_CALL_S: f64 = 300.0;

/// Shorter than this is not an over. A squelch crash, the tail of a repeater
/// and the first block of a channel opening are all well under it.
const MIN_CALL_S: f64 = 0.4;

/// Below this a block is silence rather than speech, so a channel sitting
/// open with the squelch defeated does not record the noise floor for ever.
/// The transcriber's floor, for the same reason and off the same tap.
const FLOOR: f32 = 0.004;

/// How much of the disk the call folder may take.
///
/// At [`BITRATE`] that is 74 hours of somebody talking, which is a long
/// unattended watch on any real channel: the recording only runs while a
/// transmission does.
pub const DEFAULT_MAX_BYTES: u64 = 512 << 20;

/// Segments of a day's calls. Smaller than the packet log's, because these
/// files are a hundred times slower to fill and a trim should cost the
/// oldest hour rather than the oldest week.
const FORMAT: Format = Format {
    ext: EXT,
    magic: MAGIC,
    version: VERSION,
    segment_bytes: 32 << 20,
    buf_bytes: 64 << 10,
    flush_every: std::time::Duration::from_millis(500),
};

/// What the recorder is doing, for the pane that offers its switch.
///
/// A snapshot rather than a handle on the node: the graph is rebuilt on every
/// retune and the interface is on another thread.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Recorder {
    /// The node in the running graph, so a view can set its parameters.
    pub node: usize,
    pub on: bool,
    /// Calls written since the receiver started.
    pub calls: u64,
    /// What the folder holds, and where it is.
    pub bytes: u64,
    pub dir: String,
    /// At its limit with nothing left to delete, which is the one state that
    /// looks like "nobody is talking" and is not.
    pub full: bool,
    /// Overs being recorded at this moment.
    pub recording: usize,
}

/// One over, as the log keeps it.
#[derive(Clone, Debug, PartialEq)]
pub struct Call {
    /// When it started, in microseconds since the epoch.
    pub at_us: u64,
    /// Centre of the channel it was heard on.
    pub channel_hz: u64,
    /// The system it came over: "M17", "DMR", "TETRA", or what an analogue
    /// channel is called on the audio bus.
    pub system: String,
    /// Who was talking, where the system says. An analogue channel does not.
    pub from: Option<String>,
    /// The talkgroup, reflector or party called.
    pub to: Option<String>,
    pub duration_ms: u32,
    /// Loudest sample in the over as it arrived on the bus tap, before the
    /// recorder's own limiter. A tuned analogue channel with its AGC up
    /// reaches ten or twenty times full scale, so this is a reading of the
    /// channel rather than a number to scale the audio by.
    pub peak: f32,
    /// The over itself, one Opus packet per 20 ms.
    pub frames: Vec<Vec<u8>>,
}

impl Call {
    pub fn seconds(&self) -> f64 {
        self.duration_ms as f64 / 1000.0
    }

    /// The speech back, ready to play or to hand to a transcriber.
    pub fn speech(&self) -> Option<common::Speech> {
        let mut dec = OpusDecoder::new(RATE as i32, 1).ok()?;
        let mut pcm = Vec::with_capacity(self.frames.len() * FRAME);
        let mut block = vec![0.0f32; FRAME];
        for f in &self.frames {
            // A torn packet ends the audio rather than the record: what was
            // decoded before it is still what was said.
            let Ok(n) = dec.decode(f, FRAME, &mut block) else { break };
            pcm.extend_from_slice(&block[..n.min(block.len())]);
        }
        Some(common::Speech { pcm, rate: RATE })
    }
}

/// Where calls are written, beside the packet log and the pictures.
pub fn calls_dir() -> PathBuf {
    crate::packetlog::PacketLog::default_dir()
        .map(|d| d.with_file_name("calls"))
        .unwrap_or_else(|| std::env::temp_dir().join("waveshark-calls"))
}

/// The folder's files, for a receiver with no log open to ask.
pub fn folder_bytes(dir: &std::path::Path) -> u64 {
    segments::folder_bytes(dir, EXT)
}

pub struct CallLog {
    seg: Segments,
}

impl CallLog {
    pub fn new(dir: PathBuf) -> Self {
        let mut seg = Segments::new(dir, FORMAT);
        seg.set_cap(Some(DEFAULT_MAX_BYTES));
        Self { seg }
    }

    pub fn with_cap(mut self, cap: Option<u64>) -> Self {
        self.seg.set_cap(cap);
        self
    }

    pub fn total(&self) -> u64 {
        self.seg.total()
    }

    pub fn full(&self) -> bool {
        self.seg.full()
    }

    pub fn write(&mut self, call: &Call) {
        self.seg.append(call.at_us, &encode_record(call));
    }

    /// Called between blocks whether or not a call ended, so the last over
    /// before a channel goes quiet reaches the disk.
    pub fn tick(&mut self) {
        self.seg.flush_due();
        self.seg.refresh_folder();
    }

    pub fn flush(&mut self) {
        self.seg.flush();
    }
}

fn put_str(out: &mut Vec<u8>, s: &str) {
    let b = s.as_bytes();
    let n = b.len().min(u8::MAX as usize);
    out.push(n as u8);
    out.extend_from_slice(&b[..n]);
}

fn take_str(r: &[u8], at: &mut usize) -> Option<String> {
    let n = *r.get(*at)? as usize;
    *at += 1;
    let s = String::from_utf8_lossy(r.get(*at..*at + n)?).into_owned();
    *at += n;
    Some(s)
}

fn encode_record(c: &Call) -> Vec<u8> {
    let frames = c.frames.len().min(u16::MAX as usize);
    let mut rec = Vec::with_capacity(64 + c.frames.iter().map(|f| f.len() + 2).sum::<usize>());
    rec.extend_from_slice(&0u32.to_le_bytes());
    rec.push(KIND_CALL);
    rec.push(CODEC_OPUS);
    rec.extend_from_slice(&(frames as u16).to_le_bytes());
    rec.extend_from_slice(&c.at_us.to_le_bytes());
    rec.extend_from_slice(&c.channel_hz.to_le_bytes());
    rec.extend_from_slice(&c.duration_ms.to_le_bytes());
    rec.extend_from_slice(&c.peak.to_le_bytes());
    put_str(&mut rec, &c.system);
    put_str(&mut rec, c.from.as_deref().unwrap_or(""));
    put_str(&mut rec, c.to.as_deref().unwrap_or(""));
    for f in &c.frames[..frames] {
        let n = f.len().min(u16::MAX as usize);
        rec.extend_from_slice(&(n as u16).to_le_bytes());
        rec.extend_from_slice(&f[..n]);
    }
    let len = (rec.len() - 4) as u32;
    rec[..4].copy_from_slice(&len.to_le_bytes());
    rec
}

/// Read a log back.
///
/// A log nothing can read is a log nobody keeps, so the reader ships with the
/// writer and is tested against it. A truncated final record, which is what a
/// receiver killed mid-write leaves, ends the iteration rather than failing.
pub fn read(path: impl AsRef<std::path::Path>) -> std::io::Result<Vec<Call>> {
    Ok(parse(&std::fs::read(path)?))
}

pub fn parse(buf: &[u8]) -> Vec<Call> {
    let mut out = Vec::new();
    let Some(mut at) = segments::after_magic(buf, MAGIC) else {
        return out;
    };
    while at + 4 <= buf.len() {
        let len = u32::from_le_bytes(buf[at..at + 4].try_into().unwrap()) as usize;
        at += 4;
        if len < HEAD_LEN || at + len > buf.len() {
            break;
        }
        let r = &buf[at..at + len];
        at += len;
        if let Some(c) = record(r, true) {
            out.push(c);
        }
    }
    out
}

/// One record's body, with the audio only where it is asked for.
///
/// An unknown kind or codec is skipped by its length rather than guessed at,
/// which is the whole reason the length comes first.
fn record(r: &[u8], audio: bool) -> Option<Call> {
    if r.len() < HEAD_LEN || r[0] != KIND_CALL || r[1] != CODEC_OPUS {
        return None;
    }
    let frames = u16::from_le_bytes(r[2..4].try_into().unwrap()) as usize;
    let mut p = HEAD_LEN;
    let system = take_str(r, &mut p)?;
    let from = take_str(r, &mut p)?;
    let to = take_str(r, &mut p)?;
    let mut packets = Vec::new();
    if audio {
        packets.reserve(frames);
        for _ in 0..frames {
            let n = r.get(p..p + 2).map(|b| u16::from_le_bytes(b.try_into().unwrap()) as usize)?;
            p += 2;
            let f = r.get(p..p + n)?;
            p += n;
            packets.push(f.to_vec());
        }
    }
    Some(Call {
        at_us: u64::from_le_bytes(r[4..12].try_into().unwrap()),
        channel_hz: u64::from_le_bytes(r[12..20].try_into().unwrap()),
        duration_ms: u32::from_le_bytes(r[20..24].try_into().unwrap()),
        peak: f32::from_le_bytes(r[24..28].try_into().unwrap()),
        system,
        from: (!from.is_empty()).then_some(from),
        to: (!to.is_empty()).then_some(to),
        frames: packets,
    })
}

/// One record in the folder: what its header says, and where the audio is.
///
/// The list a view draws is headers; the audio is a megabyte an over and is
/// read only when somebody plays one. So a browse walks the files and keeps
/// the position of each record rather than its speech.
#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
    /// The call, with `frames` empty: [`speech_of`] fetches those.
    pub call: Call,
    pub file: PathBuf,
    /// Where the record's body starts in the file, and how long it is.
    pub at: u64,
    pub len: u32,
}

/// The newest `most` calls in a folder, newest first.
///
/// Files are read one header at a time and seeked over the audio, so a folder
/// holding a week of a busy talkgroup costs a few hundred kilobytes to list
/// rather than the gigabyte it is.
pub fn browse(dir: &std::path::Path, most: usize) -> Vec<Entry> {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    // Named for the day they hold, so the newest file is the last name.
    let mut files: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == EXT))
        .collect();
    files.sort();
    let mut out: Vec<Entry> = Vec::new();
    for f in files.iter().rev() {
        out.extend(headers(f));
        if out.len() >= most {
            break;
        }
    }
    out.sort_by(|a, b| b.call.at_us.cmp(&a.call.at_us));
    out.truncate(most);
    out
}

/// Every record in one file, headers only.
pub fn headers(path: &std::path::Path) -> Vec<Entry> {
    use std::io::{Read, Seek, SeekFrom};
    let mut out = Vec::new();
    let Ok(mut f) = std::fs::File::open(path) else {
        return out;
    };
    let mut head = [0u8; MAGIC.len() + 2];
    if f.read_exact(&mut head).is_err() || segments::after_magic(&head, MAGIC).is_none() {
        return out;
    }
    let mut at = head.len() as u64;
    let mut buf = vec![0u8; HEAD_LEN + 3 * (1 + u8::MAX as usize)];
    loop {
        let mut len = [0u8; 4];
        if f.read_exact(&mut len).is_err() {
            break;
        }
        let len = u32::from_le_bytes(len);
        at += 4;
        if (len as usize) < HEAD_LEN {
            break;
        }
        // The header and the three strings, which is all a row needs. What a
        // short read leaves in the buffer is the tail of the record before
        // it, so the length parsed from is the length read.
        let want = (len as usize).min(buf.len());
        let Ok(got) = read_upto(&mut f, &mut buf[..want]) else { break };
        if let Some(call) = record(&buf[..got], false) {
            out.push(Entry { call, file: path.to_path_buf(), at, len });
        }
        if f.seek(SeekFrom::Start(at + u64::from(len))).is_err() {
            break;
        }
        at += u64::from(len);
    }
    out
}

fn read_upto(f: &mut std::fs::File, buf: &mut [u8]) -> std::io::Result<usize> {
    use std::io::Read;
    let mut got = 0;
    while got < buf.len() {
        match f.read(&mut buf[got..])? {
            0 => break,
            n => got += n,
        }
    }
    Ok(got)
}

/// The speech of one listed call, read from the file it is in.
pub fn speech_of(e: &Entry) -> Option<common::Speech> {
    use std::io::{Read, Seek, SeekFrom};
    let mut f = std::fs::File::open(&e.file).ok()?;
    f.seek(SeekFrom::Start(e.at)).ok()?;
    let mut buf = vec![0u8; e.len as usize];
    f.read_exact(&mut buf).ok()?;
    record(&buf, true)?.speech()
}

/// Every over in `entries`, oldest first, as one piece of audio.
///
/// A conversation is what somebody wants to hear, and it is kept as one
/// record per over: played one at a time that is a row of buttons rather than
/// a conversation. `gap_s` of silence stands in for the pause between overs,
/// so a reply does not begin on the last syllable of the call it answers.
/// Overs that will not decode are left out rather than replaced with silence.
///
/// `max_s` is a ceiling on how much is joined, newest kept: the audio is held
/// as samples and resampled again by the player, so an unbounded join of a
/// week of a repeater is hundreds of megabytes before a sound comes out.
pub fn timeline(entries: &[Entry], gap_s: f64, max_s: f64) -> Option<common::Speech> {
    let mut order: Vec<&Entry> = entries.iter().collect();
    order.sort_by_key(|e| e.call.at_us);
    // Trimmed from the front, because the end of a conversation is the part
    // somebody is catching up on.
    let mut budget = max_s;
    let mut from = order.len();
    while from > 0 && budget > 0.0 {
        from -= 1;
        budget -= order[from].call.seconds() + gap_s;
    }
    let order = &order[from..];
    let gap = vec![0.0f32; (gap_s.max(0.0) * RATE) as usize];
    let mut pcm: Vec<f32> = Vec::new();
    for e in order.iter() {
        let Some(s) = speech_of(e) else { continue };
        if !pcm.is_empty() {
            pcm.extend_from_slice(&gap);
        }
        pcm.extend_from_slice(&s.pcm);
    }
    (!pcm.is_empty()).then_some(common::Speech { pcm, rate: RATE })
}

/// What one call is written out as: when it was, who was talking and where.
///
/// The time is in it because that is what somebody searching a folder of
/// exports has; the index because two overs a second apart would otherwise
/// be one file.
pub fn wav_name(c: &Call, k: usize) -> String {
    let when = segments::when(c.at_us).format("%Y%m%d-%H%M%S").to_string();
    let who = c.from.as_deref().or(c.to.as_deref()).unwrap_or(&c.system);
    let who: String =
        who.chars().map(|c| if c.is_ascii_alphanumeric() { c } else { '-' }).collect();
    format!("{k:04}_{when}_{who}_{:.4}MHz.wav", c.channel_hz as f64 / 1e6)
}

/// Write each over out as a WAV, and say how many were written.
///
/// WAV because the point of an export is a file something else can open, and
/// Opus in a bespoke container is not that. An over that will not decode is
/// counted as a failure rather than written empty.
pub fn export(entries: &[Entry], dir: &std::path::Path) -> std::io::Result<(usize, usize)> {
    std::fs::create_dir_all(dir)?;
    let mut order: Vec<&Entry> = entries.iter().collect();
    order.sort_by_key(|e| e.call.at_us);
    let (mut done, mut failed) = (0, 0);
    for (k, e) in order.iter().enumerate() {
        match speech_of(e) {
            Some(s) => {
                crate::mix::write_wav(&dir.join(wav_name(&e.call, k)), &s)?;
                done += 1;
            }
            None => failed += 1,
        }
    }
    Ok((done, failed))
}

/// Kind, codec, frame count, time, channel, duration and peak.
const HEAD_LEN: usize = 1 + 1 + 2 + 8 + 8 + 4 + 4;

/// A call being recorded: what is known about it, and the encoder part way
/// through it.
struct Open {
    at_us: u64,
    system: String,
    channel_hz: f64,
    from: Option<String>,
    to: Option<String>,
    /// The rate the front end produces, and the resampler to [`RATE`]. A
    /// vocoder at 8 kHz and a broadcast strip at 48 need different ones, and
    /// a front end already at 16 needs none.
    in_rate: f64,
    resample: Option<audio::Resampler>,
    /// Samples at [`RATE`] not yet a whole Opus frame.
    pending: Vec<f32>,
    /// Blocks with nothing in them since the last speech, held rather than
    /// encoded: a pause between two words belongs in the recording and the
    /// hang that ends the over does not, and which one this is is not known
    /// until somebody either speaks again or does not.
    tail: Vec<f32>,
    encoder: OpusEncoder,
    frames: Vec<Vec<u8>>,
    peak: f32,
    /// The limiter's running peak, and the gain it had on the last block.
    ///
    /// The tap is the bus before the faders, and a strip with AGC hands it
    /// audio well outside ±1: the PMR446 capture arrives at seventeen times
    /// full scale. Opus takes floats around unity, so without this every
    /// record is a square wave. It decays rather than holding, or one loud
    /// syllable would leave the rest of the over inaudible.
    level: f32,
    gain: f32,
    /// Seconds since anything above [`FLOOR`] arrived, which is what ends
    /// the call.
    quiet_s: f64,
    /// Seconds encoded, which is the call's length: the pauses inside an
    /// over are kept, since a recording with the gaps cut is not what was
    /// transmitted.
    seconds: f64,
}

impl Open {
    fn new(v: &common::Voice, at_us: u64) -> Option<Self> {
        let mut encoder = OpusEncoder::new(RATE as i32, 1, Application::Voip).ok()?;
        encoder.bitrate_bps = BITRATE;
        encoder.use_cbr = true;
        let in_rate = v.rate.max(1.0);
        Some(Self {
            at_us,
            system: v.system.to_string(),
            channel_hz: v.channel_hz,
            from: v.from.clone(),
            to: v.to.clone(),
            in_rate,
            // Four taps: this is speech on its way to a 3 kHz channel.
            resample: ((in_rate - RATE).abs() > 1.0)
                .then(|| audio::Resampler::new(in_rate, RATE, 4)),
            pending: Vec::new(),
            tail: Vec::new(),
            encoder,
            frames: Vec::new(),
            peak: 0.0,
            level: 1.0,
            gain: 1.0,
            quiet_s: 0.0,
            seconds: 0.0,
        })
    }

    fn push(&mut self, pcm: &[f32], loud: bool) {
        if !loud {
            self.tail.extend_from_slice(pcm);
            return;
        }
        let mut block = 0.0f32;
        for s in pcm {
            block = block.max(s.abs());
        }
        self.peak = self.peak.max(block);
        // Half a second's decay at a fiftieth of a second a block, so the
        // gain follows a channel getting louder at once and a channel
        // getting quieter over a syllable or two.
        self.level = block.max(self.level * 0.96).max(1.0);
        let want = 0.9 / self.level;
        let held = std::mem::take(&mut self.tail);
        let mut scaled = Vec::with_capacity(held.len() + pcm.len());
        // Ramped across the block rather than stepped, which would be heard
        // as a click at every change.
        let n = (held.len() + pcm.len()).max(1) as f32;
        for (k, s) in held.iter().chain(pcm.iter()).enumerate() {
            let t = k as f32 / n;
            scaled.push(s * (self.gain + (want - self.gain) * t));
        }
        self.gain = want;
        match self.resample.as_mut() {
            Some(r) => r.process(&scaled, &mut self.pending),
            None => self.pending.extend_from_slice(&scaled),
        }
        let mut buf = [0u8; 1275];
        while self.pending.len() >= FRAME {
            let frame: Vec<f32> = self.pending.drain(..FRAME).collect();
            match self.encoder.encode(&frame, FRAME, &mut buf) {
                Ok(n) if n > 0 => {
                    self.frames.push(buf[..n].to_vec());
                    self.seconds += FRAME as f64 / RATE;
                }
                // A frame the encoder refused is a frame of the over gone,
                // and the rest of it is still worth keeping.
                _ => self.seconds += FRAME as f64 / RATE,
            }
        }
    }

    fn finish(self) -> Option<Call> {
        if self.seconds < MIN_CALL_S || self.frames.is_empty() {
            return None;
        }
        Some(Call {
            at_us: self.at_us,
            channel_hz: self.channel_hz.max(0.0) as u64,
            system: self.system,
            from: self.from,
            to: self.to,
            duration_ms: (self.seconds * 1000.0) as u32,
            peak: self.peak,
            frames: self.frames,
        })
    }
}

/// Writes every over the receiver hears to the call log.
///
/// It hangs off the audio bus tap, which carries every strip before the
/// faders and the subscriptions: what the receiver heard, not what the
/// operator chose to listen to. A recording made downstream of the faders
/// would carry the listener's volume setting and would stop when they muted
/// the channel to answer the phone.
pub struct CallLogNode {
    enabled: bool,
    dir: PathBuf,
    cap: Option<u64>,
    log: Option<CallLog>,
    /// The calls in progress, one per conversation. A Vec rather than a map:
    /// a receiver hears a handful of channels at once, and the order is the
    /// order they opened in.
    open: Vec<(common::ConversationKey, Open)>,
    written: u64,
    dropped: u64,
}

impl Default for CallLogNode {
    fn default() -> Self {
        Self::new(calls_dir())
    }
}

impl CallLogNode {
    pub fn new(dir: PathBuf) -> Self {
        Self {
            enabled: false,
            dir,
            cap: Some(DEFAULT_MAX_BYTES),
            log: None,
            open: Vec::new(),
            written: 0,
            dropped: 0,
        }
    }

    /// Calls written since the receiver started.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn written(&self) -> u64 {
        self.written
    }

    /// Whether the folder is at its limit with nothing left to delete.
    pub fn full(&self) -> bool {
        self.log.as_ref().is_some_and(|l| l.full())
    }

    /// What it is doing, for the interface.
    ///
    /// The folder is measured rather than reported as zero when nothing has
    /// been written yet: a receiver started this morning still has last
    /// night's recordings on the disk.
    pub fn recorder(&self) -> Recorder {
        Recorder {
            node: 0,
            on: self.enabled,
            calls: self.written,
            bytes: match self.log.as_ref() {
                Some(l) => l.total(),
                None => folder_bytes(&self.dir),
            },
            dir: self.dir.display().to_string(),
            full: self.full(),
            recording: self.open.len(),
        }
    }

    fn write(&mut self, call: Call) {
        let log = self.log.get_or_insert_with(|| CallLog::new(self.dir.clone()).with_cap(self.cap));
        log.write(&call);
        self.written += 1;
    }

    /// End every call in progress and write what they hold.
    fn close_all(&mut self) {
        for (_, open) in std::mem::take(&mut self.open) {
            match open.finish() {
                Some(call) => self.write(call),
                None => self.dropped += 1,
            }
        }
    }
}

impl Simple for CallLogNode {
    fn name(&self) -> &str {
        "call_log"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Voice {
            return Err(common::Error::other("the call log reads the audio bus tap"));
        }
        // A sink still declares an output spec, because the graph gives every
        // node a slot. Nothing is written to it.
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        if !self.enabled {
            // Whatever was being recorded when it was switched off is still
            // an over somebody heard, so it is written rather than dropped.
            if !self.open.is_empty() {
                self.close_all();
            }
            return Ok(());
        }
        let mut heard: Vec<common::ConversationKey> = Vec::new();
        for v in i.as_voice().unwrap_or(&[]) {
            let loud = v.pcm.iter().any(|s| s.abs() > FLOOR);
            let key = common::ConversationKey::of(v);
            if loud {
                heard.push(key.clone());
            }
            let at = self.open.iter().position(|(k, _)| *k == key);
            match at {
                Some(k) => {
                    let (_, open) = &mut self.open[k];
                    // A front end that changed rate mid-call is a rebuilt
                    // graph, not a new speaker: the call ends here and the
                    // next block opens another with the new rate.
                    if (open.in_rate - v.rate.max(1.0)).abs() > 1.0 {
                        let (_, open) = self.open.remove(k);
                        match open.finish() {
                            Some(call) => self.write(call),
                            None => self.dropped += 1,
                        }
                        continue;
                    }
                    if loud {
                        open.quiet_s = 0.0;
                    }
                    open.push(&v.pcm, loud);
                }
                // Silence does not start a call. A channel sitting open with
                // nobody on it would otherwise be one record after another
                // of the noise floor.
                None if loud => {
                    if let Some(mut open) = Open::new(v, now_us()) {
                        open.push(&v.pcm, true);
                        self.open.push((key, open));
                    }
                }
                None => {}
            }
        }

        // Anything not heard from this block is that much closer to over.
        let dt = c.block_seconds.max(0.0);
        for (key, open) in self.open.iter_mut() {
            if !heard.contains(key) {
                open.quiet_s += dt;
            }
        }
        let mut done = Vec::new();
        for (key, open) in std::mem::take(&mut self.open) {
            match open.quiet_s >= HANG_S || open.seconds >= MAX_CALL_S {
                true => done.push(open),
                false => self.open.push((key, open)),
            }
        }
        for open in done {
            match open.finish() {
                Some(call) => self.write(call),
                None => self.dropped += 1,
            }
        }
        if let Some(log) = self.log.as_mut() {
            log.tick();
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("enabled", self.enabled).label("Record calls"),
            Param::text("dir", self.dir.display().to_string()).label("Folder"),
            Param::int("cap_mb", self.cap.map(|c| (c >> 20) as i64).unwrap_or(0), 0..=1 << 20)
                .label("Keep at most, MB"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => {
                let on = v.as_bool().unwrap_or(false);
                if !on {
                    self.close_all();
                }
                self.enabled = on;
                Ok(())
            }
            "dir" => {
                let dir = PathBuf::from(v.as_str().unwrap_or_default());
                if !dir.as_os_str().is_empty() && dir != self.dir {
                    self.close_all();
                    self.dir = dir;
                    self.log = None;
                }
                Ok(())
            }
            "cap_mb" => {
                let mb = v.as_i64().unwrap_or(0);
                self.cap = (mb > 0).then_some((mb as u64) << 20);
                self.log = None;
                Ok(())
            }
            other => Err(common::Error::other(format!("call_log has no {other}"))),
        }
    }
}

/// A receiver stopped mid-over keeps it. The alternative is that the
/// transmission somebody left the receiver running for is the one that is
/// not in the file.
impl Drop for CallLogNode {
    fn drop(&mut self) {
        self.close_all();
        if let Some(log) = self.log.as_mut() {
            log.flush();
        }
    }
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

pub const DESC: StageDesc = StageDesc {
    name: "call_log",
    summary: "Writes every over the receiver hears to disk as Opus, with who \
              was talking, on what, and for how long",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let dir = match s.get("dir").and_then(|v| v.as_str()).filter(|d| !d.is_empty()) {
        Some(d) => PathBuf::from(d),
        None => calls_dir(),
    };
    let mut n = CallLogNode::new(dir);
    let mb = s.i64_or("cap_mb", (DEFAULT_MAX_BYTES >> 20) as i64);
    n.cap = (mb > 0).then_some((mb as u64) << 20);
    n.enabled = s.bool_or("enabled", false);
    Ok(Box::new(n) as Box<dyn pipeline::node::Node>)
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-calllog-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    /// A tone at the rate a vocoder produces, which is what a call is made
    /// of as far as this node is concerned.
    fn voice(seconds: f64, amplitude: f32, from: &str) -> common::Voice {
        let rate = 8_000.0;
        let n = (seconds * rate) as usize;
        let pcm = (0..n)
            .map(|k| amplitude * (k as f32 * std::f32::consts::TAU * 440.0 / rate as f32).sin())
            .collect();
        common::Voice {
            system: "M17",
            channel_hz: 434_000_000.0,
            to: Some("ALL".into()),
            from: Some(from.into()),
            code: None,
            rate,
            channels: 1,
            pcm,
        }
    }

    fn silence(seconds: f64, from: &str) -> common::Voice {
        let mut v = voice(seconds, 0.0, from);
        v.pcm = vec![0.0; v.pcm.len()];
        v
    }

    fn run(n: &mut CallLogNode, v: &[common::Voice], block_s: f64) {
        let rate = v.first().map(|v| v.rate).unwrap_or(8_000.0);
        let ins = [PortSpec {
            spec: StreamSpec { kind: PortKind::Voice, rate, ..Default::default() },
            latency: 0,
        }];
        let (tags, mut events, mut out_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut out_tags);
        ctx.block_seconds = block_s;
        let mut out = Payload::empty_of(PortKind::Voice);
        Simple::process(n, &Payload::Voice(v.to_vec()), &mut out, &mut ctx).unwrap();
    }

    #[test]
    fn a_call_is_one_record_with_who_said_it_and_for_how_long() {
        let d = dir("one");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        // Two seconds of speech in blocks of a fifth, then the hang.
        for _ in 0..10 {
            run(&mut n, &[voice(0.2, 0.3, "M0ABC")], 0.2);
        }
        for _ in 0..10 {
            run(&mut n, &[silence(0.2, "M0ABC")], 0.2);
        }
        assert_eq!(n.written(), 1, "one over is one record");
        drop(n);

        let files: Vec<PathBuf> =
            std::fs::read_dir(&d).unwrap().flatten().map(|e| e.path()).collect();
        assert_eq!(files.len(), 1, "one segment, got {files:?}");
        let calls = read(&files[0]).unwrap();
        assert_eq!(calls.len(), 1);
        let c = &calls[0];
        assert_eq!(c.system, "M17");
        assert_eq!(c.from.as_deref(), Some("M0ABC"));
        assert_eq!(c.to.as_deref(), Some("ALL"));
        assert_eq!(c.channel_hz, 434_000_000);
        // Two seconds of speech, to the Opus frame: the hang that ended the
        // over is not part of it, and the resampler's first samples are the
        // twenty milliseconds missing off the end.
        assert_eq!(c.duration_ms, 1_980, "a two second over came back as {} ms", c.duration_ms);
        assert_eq!(c.frames.len(), (c.duration_ms as usize) / 20, "a frame is 20 ms");
        assert!(c.peak > 0.25 && c.peak <= 0.31, "the peak was {}", c.peak);

        // And the audio decodes back to a tone of the same length and level.
        let speech = c.speech().expect("the audio decodes");
        assert_eq!(speech.rate, RATE);
        assert!(
            (speech.seconds() - c.seconds()).abs() < 0.05,
            "{} s of audio in a {} s call",
            speech.seconds(),
            c.seconds()
        );
        let peak = speech.pcm.iter().fold(0.0f32, |a, s| a.max(s.abs()));
        assert!(peak > 0.15, "the recording came back at {peak}, which is not the tone");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn two_overs_are_two_records_and_silence_is_none() {
        let d = dir("two");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        for who in ["M0ABC", "M0XYZ"] {
            for _ in 0..5 {
                run(&mut n, &[voice(0.2, 0.3, who)], 0.2);
            }
            for _ in 0..10 {
                run(&mut n, &[silence(0.2, who)], 0.2);
            }
        }
        // And a channel sitting open with nobody on it, which must not be a
        // record at all.
        for _ in 0..20 {
            run(&mut n, &[silence(0.2, "M0QRP")], 0.2);
        }
        assert_eq!(n.written(), 2, "two overs and a quiet channel");
        drop(n);
        let files: Vec<PathBuf> =
            std::fs::read_dir(&d).unwrap().flatten().map(|e| e.path()).collect();
        let calls = read(&files[0]).unwrap();
        assert_eq!(calls.len(), 2);
        assert_eq!(calls[0].from.as_deref(), Some("M0ABC"));
        assert_eq!(calls[1].from.as_deref(), Some("M0XYZ"));
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_receiver_stopped_mid_over_keeps_it() {
        let d = dir("killed");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        for _ in 0..10 {
            run(&mut n, &[voice(0.2, 0.3, "M0ABC")], 0.2);
        }
        assert_eq!(n.written(), 0, "nothing is written while the key is still down");
        drop(n);
        let files: Vec<PathBuf> =
            std::fs::read_dir(&d).unwrap().flatten().map(|e| e.path()).collect();
        assert_eq!(read(&files[0]).unwrap().len(), 1, "the over in progress was lost");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn an_over_costs_about_two_kilobytes_a_second() {
        // The number the overnight watch depends on: 16 kbit/s and nothing
        // written between overs.
        let d = dir("size");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        for _ in 0..50 {
            run(&mut n, &[voice(0.2, 0.3, "M0ABC")], 0.2);
        }
        drop(n);
        let files: Vec<PathBuf> =
            std::fs::read_dir(&d).unwrap().flatten().map(|e| e.path()).collect();
        let bytes = std::fs::metadata(&files[0]).unwrap().len();
        let calls = read(&files[0]).unwrap();
        let rate = bytes as f64 / calls[0].seconds();
        assert!((1_700.0..2_400.0).contains(&rate), "ten seconds of speech cost {rate} B/s");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// What the calls pane's log table draws from: headers without the
    /// audio, newest first, and the speech fetched for the one played.
    #[test]
    fn a_folder_lists_newest_first_and_fetches_one_over() {
        let d = dir("browse");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        for who in ["M0ABC", "M0XYZ", "M0QRP"] {
            for _ in 0..5 {
                run(&mut n, &[voice(0.2, 0.3, who)], 0.2);
            }
            for _ in 0..10 {
                run(&mut n, &[silence(0.2, who)], 0.2);
            }
        }
        drop(n);

        let listed = browse(&d, 100);
        assert_eq!(listed.len(), 3);
        // Newest first, which is the opposite of the order they were
        // written: the row worth reading is the one that just happened.
        let who: Vec<&str> =
            listed.iter().filter_map(|e| e.call.from.as_deref()).collect::<Vec<_>>();
        assert_eq!(who, ["M0QRP", "M0XYZ", "M0ABC"]);
        assert_eq!(listed[0].call.system, "M17");
        assert_eq!(listed[0].call.to.as_deref(), Some("ALL"));
        assert_eq!(listed[0].call.channel_hz, 434_000_000);
        assert_eq!(listed[0].call.duration_ms, 980);
        assert!(
            listed.iter().all(|e| e.call.frames.is_empty()),
            "a listing carried the audio it exists to avoid reading"
        );

        let speech = speech_of(&listed[0]).expect("the over reads back");
        assert_eq!(speech.rate, RATE);
        assert!((speech.seconds() - 0.98).abs() < 0.05, "{} s", speech.seconds());
        let peak = speech.pcm.iter().fold(0.0f32, |a, s| a.max(s.abs()));
        assert!(peak > 0.15, "the over came back at {peak}");

        // A cap keeps the newest, so a week of a busy talkgroup lists as
        // fast as an empty folder.
        let two = browse(&d, 2);
        assert_eq!(two.len(), 2);
        assert_eq!(two[0].call.from.as_deref(), Some("M0QRP"));
        let _ = std::fs::remove_dir_all(&d);
    }

    /// Three overs joined into one conversation, in the order they happened
    /// and with the pause between them, and written out as three WAVs.
    #[test]
    fn a_conversation_plays_and_exports_as_one_set() {
        let d = dir("timeline");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        for who in ["M0ABC", "M0XYZ", "M0QRP"] {
            for _ in 0..5 {
                run(&mut n, &[voice(0.2, 0.3, who)], 0.2);
            }
            for _ in 0..10 {
                run(&mut n, &[silence(0.2, who)], 0.2);
            }
        }
        drop(n);

        let listed = browse(&d, 100);
        assert_eq!(listed.len(), 3);
        let joined = timeline(&listed, 0.4, 600.0).expect("a conversation");
        // Three overs of 0.98 s and two gaps of 0.4 s, whatever order the
        // listing was in.
        assert!((joined.seconds() - (3.0 * 0.98 + 0.8)).abs() < 0.1, "{} s", joined.seconds());

        // The ceiling keeps the newest overs: two of the three, not the
        // first two seconds of the afternoon.
        let capped = timeline(&listed, 0.4, 2.2).expect("a conversation");
        assert!((capped.seconds() - (2.0 * 0.98 + 0.4)).abs() < 0.1, "{} s", capped.seconds());
        // The gap is silence and lands between the first two overs, which is
        // what makes a reply audibly a reply rather than an interruption.
        let at = (RATE * 1.1) as usize;
        let quiet =
            joined.pcm[at..at + (RATE * 0.2) as usize].iter().fold(0.0f32, |a, s| a.max(s.abs()));
        assert!(quiet < 0.01, "the pause between overs carried {quiet}");

        let wavs = d.join("wav");
        assert_eq!(export(&listed, &wavs).expect("written"), (3, 0));
        let mut names: Vec<String> = std::fs::read_dir(&wavs)
            .expect("the folder")
            .flatten()
            .map(|e| e.file_name().to_string_lossy().into_owned())
            .collect();
        names.sort();
        assert_eq!(names.len(), 3);
        // Oldest first, numbered in the order they were said rather than in
        // the order the newest-first table lists them.
        assert!(names[0].starts_with("0000_"), "{}", names[0]);
        assert!(names[0].contains("M0ABC"), "{}", names[0]);
        assert!(names[2].contains("M0QRP"), "{}", names[2]);
        assert!(names.iter().all(|n| n.ends_with("434.0000MHz.wav")), "{names:?}");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_file_that_is_not_a_call_log_reads_as_empty() {
        assert!(parse(b"this is not a call log at all").is_empty());
        assert!(parse(b"").is_empty());
    }

    #[test]
    fn a_torn_tail_costs_one_record() {
        let d = dir("torn");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        for who in ["M0ABC", "M0XYZ", "M0QRP"] {
            for _ in 0..5 {
                run(&mut n, &[voice(0.2, 0.3, who)], 0.2);
            }
            for _ in 0..10 {
                run(&mut n, &[silence(0.2, who)], 0.2);
            }
        }
        drop(n);
        let files: Vec<PathBuf> =
            std::fs::read_dir(&d).unwrap().flatten().map(|e| e.path()).collect();
        let mut raw = std::fs::read(&files[0]).unwrap();
        raw.truncate(raw.len() - 9);
        assert_eq!(parse(&raw).len(), 2, "a torn tail took a good record with it");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn switching_it_off_writes_what_was_being_recorded() {
        let d = dir("off");
        let mut n = CallLogNode::new(d.clone());
        Node::set_param(&mut n, "enabled", ParamValue::Bool(true)).unwrap();
        for _ in 0..10 {
            run(&mut n, &[voice(0.2, 0.3, "M0ABC")], 0.2);
        }
        Node::set_param(&mut n, "enabled", ParamValue::Bool(false)).unwrap();
        assert_eq!(n.written(), 1);
        // And nothing is recorded while it is off.
        for _ in 0..10 {
            run(&mut n, &[voice(0.2, 0.3, "M0XYZ")], 0.2);
        }
        assert_eq!(n.written(), 1);
        let _ = std::fs::remove_dir_all(&d);
    }
}
