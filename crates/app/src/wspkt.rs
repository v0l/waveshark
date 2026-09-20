//! Every burst the receiver hears, written to disk as it arrives.
//!
//! On by default and with no switch in the interface, because the value of a
//! log like this is entirely in having it already: the interesting
//! transmission is always the one that happened before anyone thought to
//! press record. A band left running overnight is a test corpus, and a corpus
//! of real bursts is the only honest way to tell whether a change to a
//! decoder helped.
//!
//! # What is stored
//!
//! The evidence, which is the carrier, the keying and the frame, and never a
//! conclusion. That cut is structural rather than a rule anybody has to
//! remember: a packet's protocol layers are what decoders made of its bytes,
//! and running them again over the same bytes is what lets a protocol written
//! in October read a burst recorded in September. Speech is not here either.
//! An over is stated on the voice port, where the audio it is about travels,
//! and an hour of a busy repeater is gigabytes.
//!
//! Undecoded bursts are written too, and they are the ones that matter most.
//! A burst that no protocol claimed leaves no other trace at all, and it is
//! the raw material for adding the protocol that would have claimed it.
//!
//! # The format
//!
//! A little-endian binary stream, one file per day, appended:
//!
//! ```text
//! file    := "WSPKT\0" u16 version
//! record  := u32 len, u8 layers, layer * layers
//! layer   := u8 tag, u32 len, body
//! carrier := u64 at_us, u32 duration_us, u64 center_hz, u32 bandwidth_hz,
//!            f32 rssi_dbfs, f32 snr_db, u64 source
//! keying  := str modulation, u8 how, f32 confidence, f32 bandwidth_hz,
//!            f32 baud, f32 separation_hz, f32 sweep_hz_s,
//!            f32 symbol_period_us, u8 spreading, f32 evm,
//!            u8 symbol_kind, u32 n, body
//! frame   := u32 n, byte * n, u8 integrity, [u16 corrected], u8 has_framing,
//!            [u32 preamble_bits, u16 sync_len, byte * sync_len,
//!             str whitening, u8 fec]
//! iq      := f64 rate, u64 center_hz, u32 n, [u32 zlen], sample * n
//! str     := u16 len, byte * len
//! ```
//!
//! Every layer carries its own length, so a reader skips one it does not know
//! rather than guessing at it, and a layer gaining a field is a compatible
//! change instead of a version bump. The record's length does the same for a
//! whole record, and means a receiver killed mid-write costs the last record
//! rather than the file: a short tail cannot be mistaken for a complete one.
//!
//! The modulation is written as its own label rather than as a code invented
//! here. A second numbering of an enum that already has one is a numbering
//! that covers the five cases somebody needed, and the sixth protocol comes
//! back saying nothing about how it was keyed.
//!
//! Samples are sixteen bits a component, which is more than any converter
//! here has, and zstd where that helps. They are kept because timings and
//! bytes are what one demodulator made of a burst, and a different
//! demodulator, or the same one fixed, wants the burst. A row without them
//! can be read again but not re-read.

use std::io::Read;
use std::path::PathBuf;
use std::sync::Arc;

use common::packet::{
    Carrier, Fec, Frame, Framing, Integrity, Keying, KeyingParams, Knowledge, Packet, Symbols,
};
use common::{IqBurst, Modulation, Pulse, SourceId};

use crate::segments::{self, Format, Segments};

/// How much of the disk the whole log folder may take.
///
/// A limit per day answered the wrong question. Nobody wants to know what one
/// file may reach; they want to know what the receiver may take, so the oldest
/// days go to keep the folder under this.
pub const DEFAULT_MAX_BYTES: u64 = 2 << 30;

const EXT: &str = "wspkt";
const MAGIC: &[u8; 6] = b"WSPKT\0";

/// Layers, one length-prefixed block each, rather than a record shape per
/// kind of evidence.
const VERSION: u16 = 3;

/// How large one segment grows before the next is started.
///
/// The log is trimmed by deleting whole files, so this is how coarsely it can
/// be trimmed. A day was the unit until a busy 2.4 GHz session put 122 GB in
/// one file: the cap could then only stop the log, because the one file over
/// it was the one being written.
const SEGMENT_BYTES: u64 = 256 << 20;

const FORMAT: Format = Format {
    ext: EXT,
    magic: MAGIC,
    version: VERSION,
    segment_bytes: SEGMENT_BYTES,
    // A pulse record is a few hundred bytes and 1090 MHz can produce thousands
    // of frames a second, so the buffer holds a busy second rather than a
    // single burst.
    buf_bytes: 256 << 10,
    flush_every: std::time::Duration::from_millis(250),
};

const TAG_CARRIER: u8 = 1;
const TAG_KEYING: u8 = 2;
const TAG_FRAME: u8 = 3;
const TAG_IQ: u8 = 4;

/// Samples a record may carry, which is the end of the burst when there are
/// more.
///
/// Four million was "well past anything a burst can be", and on a 2.4 GHz
/// source read at a couple of megasamples that is exactly what a burst was:
/// sixteen megabytes a record, and a day's log of 122 GB. A quarter of a
/// million samples is a megabyte before compression and a tenth of a second
/// at 2.4 MS/s, which holds any packet this receiver decodes.
const IQ_MAX_SAMPLES: usize = 1 << 18;

/// Level 1: the samples are noise-like and do not compress far whatever is
/// spent on them, and this runs at gigabytes a second, which a sink on the
/// radio's thread needs.
const IQ_ZSTD_LEVEL: i32 = 1;

/// Marks a compressed sample block in the count field.
const IQ_COMPRESSED: u32 = 0x8000_0000;

pub struct PacketLog {
    seg: Segments,
}

impl PacketLog {
    /// `$XDG_DATA_HOME/waveshark/packets`, or `~/.local/share` when unset
    pub fn default_dir() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_DATA_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))?;
        Some(base.join("waveshark").join("packets"))
    }

    /// Beside the log folder, where the device database lives
    pub fn default_survey_path() -> Option<PathBuf> {
        Self::default_dir().and_then(|d| d.parent().map(|p| p.join("survey.sqlite")))
    }

    pub fn new(dir: PathBuf) -> Self {
        let mut seg = Segments::new(dir, FORMAT);
        seg.set_cap(Some(DEFAULT_MAX_BYTES));
        Self { seg }
    }

    /// Change the folder's limit. `None` lifts it, which is what a receiver
    /// left running on 1090 MHz for a week wants
    pub fn with_cap(mut self, cap: Option<u64>) -> Self {
        self.seg.set_cap(cap);
        self
    }

    /// What the folder holds: the segment being written and every one kept
    pub fn total(&self) -> u64 {
        self.seg.total()
    }

    pub fn written(&self) -> u64 {
        self.seg.written()
    }

    pub fn full(&self) -> bool {
        self.seg.full()
    }

    pub fn refresh_folder(&mut self) {
        self.seg.refresh_folder();
    }

    /// Put everything buffered on the disk now
    pub fn flush(&mut self) {
        self.seg.flush();
    }

    /// A chance to write, called between blocks whether or not anything
    /// arrived, so a band that went quiet still gets its last burst down
    pub fn tick(&mut self) {
        self.seg.flush_due();
        self.seg.refresh_folder();
    }

    /// Write one reception's evidence
    pub fn write(&mut self, p: &Packet) {
        if !carries_evidence(p) {
            return;
        }
        self.seg.append(p.carrier.at_us, &record(p));
    }
}

impl nodes::PacketSink for PacketLog {
    fn write(&mut self, p: &Packet) {
        PacketLog::write(self, p);
    }

    fn written(&self) -> u64 {
        PacketLog::written(self)
    }

    fn bytes(&self) -> u64 {
        self.total()
    }

    fn full(&self) -> bool {
        PacketLog::full(self)
    }

    fn flush(&mut self) {
        PacketLog::flush(self);
    }
}

/// What a log folder holds, for a receiver with no log open to ask
pub fn folder_bytes(dir: &std::path::Path) -> u64 {
    segments::folder_bytes(dir, EXT)
}

/// Whether there is anything here a decoder could read
///
/// Symbols, bytes or samples. A carrier on its own is a record of something
/// having happened, which the survey already keeps, in a file whose whole
/// purpose is to be decoded again later. A measured burst counts: what the
/// classifier made of a chirp is the whole record of a transmission nothing
/// decoded.
fn carries_evidence(p: &Packet) -> bool {
    let keyed = p
        .keying
        .as_ref()
        .is_some_and(|k| !k.symbols.is_empty() || matches!(k.how, Knowledge::Measured { .. }));
    let framed = p.frame.as_ref().is_some_and(|f| !f.bytes.is_empty());
    let sampled = p.carrier.iq.as_ref().is_some_and(|q| !q.samples.is_empty());
    keyed || framed || sampled
}

/// One record: its layers, each with its own length
fn record(p: &Packet) -> Vec<u8> {
    let mut layers: Vec<(u8, Vec<u8>)> = vec![(TAG_CARRIER, put_carrier(&p.carrier))];
    if let Some(k) = &p.keying {
        layers.push((TAG_KEYING, put_keying(k)));
    }
    if let Some(f) = &p.frame {
        layers.push((TAG_FRAME, put_frame(f)));
    }
    if let Some(q) = p.carrier.iq.as_deref().filter(|q| !q.samples.is_empty()) {
        layers.push((TAG_IQ, put_iq(q)));
    }
    let body: usize = 1 + layers.iter().map(|(_, b)| 5 + b.len()).sum::<usize>();
    let mut rec = Vec::with_capacity(4 + body);
    rec.extend_from_slice(&(body as u32).to_le_bytes());
    rec.push(layers.len() as u8);
    for (tag, b) in &layers {
        rec.push(*tag);
        rec.extend_from_slice(&(b.len() as u32).to_le_bytes());
        rec.extend_from_slice(b);
    }
    rec
}

fn put_str(out: &mut Vec<u8>, s: &str) {
    let b = &s.as_bytes()[..s.len().min(u16::MAX as usize)];
    out.extend_from_slice(&(b.len() as u16).to_le_bytes());
    out.extend_from_slice(b);
}

fn put_carrier(c: &Carrier) -> Vec<u8> {
    let mut o = Vec::with_capacity(40);
    o.extend_from_slice(&c.at_us.to_le_bytes());
    o.extend_from_slice(&c.duration_us.to_le_bytes());
    o.extend_from_slice(&c.center_hz.to_le_bytes());
    o.extend_from_slice(&c.bandwidth_hz.to_le_bytes());
    o.extend_from_slice(&c.rssi_dbfs.to_le_bytes());
    o.extend_from_slice(&c.snr_db.to_le_bytes());
    o.extend_from_slice(&c.source.0.to_le_bytes());
    o
}

fn put_keying(k: &Keying) -> Vec<u8> {
    let mut o = Vec::new();
    put_str(&mut o, k.modulation.label());
    match k.how {
        Knowledge::Measured { confidence } => {
            o.push(1);
            o.extend_from_slice(&confidence.to_le_bytes());
        }
        Knowledge::Configured => {
            o.push(0);
            o.extend_from_slice(&0f32.to_le_bytes());
        }
    }
    let p = &k.params;
    for v in [p.bandwidth_hz, p.baud, p.separation_hz, p.sweep_hz_s, p.symbol_period_us] {
        o.extend_from_slice(&v.to_le_bytes());
    }
    // Zero for none, since no signal has a spreading factor of nothing.
    o.push(p.spreading.unwrap_or(0));
    o.extend_from_slice(&p.evm.to_le_bytes());
    let n = k.symbols.len();
    o.push(match &k.symbols {
        Symbols::None => 0,
        Symbols::Pulses(_) => 1,
        Symbols::Hard(_) => 2,
        Symbols::Soft(_) => 3,
    });
    o.extend_from_slice(&(n as u32).to_le_bytes());
    match &k.symbols {
        Symbols::None => {}
        Symbols::Pulses(v) => {
            for p in v {
                o.extend_from_slice(&p.mark.to_le_bytes());
                o.extend_from_slice(&p.gap.to_le_bytes());
            }
        }
        Symbols::Hard(v) => o.extend_from_slice(v),
        Symbols::Soft(v) => {
            for s in v {
                o.extend_from_slice(&s.to_le_bytes());
            }
        }
    }
    o
}

fn put_frame(f: &Frame) -> Vec<u8> {
    let mut o = Vec::with_capacity(f.bytes.len() + 16);
    o.extend_from_slice(&(f.bytes.len() as u32).to_le_bytes());
    o.extend_from_slice(&f.bytes);
    match f.integrity {
        Integrity::Unchecked => o.push(0),
        Integrity::Passed => o.push(1),
        Integrity::Failed => o.push(2),
        Integrity::Corrected { symbols } => {
            o.push(3);
            o.extend_from_slice(&symbols.to_le_bytes());
        }
    }
    match &f.framing {
        None => o.push(0),
        Some(fr) => {
            o.push(1);
            o.extend_from_slice(&fr.preamble_bits.to_le_bytes());
            o.extend_from_slice(&(fr.sync.len() as u16).to_le_bytes());
            o.extend_from_slice(&fr.sync);
            put_str(&mut o, fr.whitening.unwrap_or(""));
            o.push(fr.fec.map(fec_code).unwrap_or(0));
        }
    }
    o
}

/// The samples, tail first when there are too many
///
/// A packet ends where the front end stopped reading, so the last samples are
/// the ones with the signal in them and the first are the silence before it.
fn put_iq(q: &IqBurst) -> Vec<u8> {
    let from = q.samples.len().saturating_sub(IQ_MAX_SAMPLES);
    let samples = &q.samples[from..];
    let mut raw = Vec::with_capacity(samples.len() * 4);
    for s in samples {
        for v in [s.re, s.im] {
            raw.extend_from_slice(
                &((v * 32767.0).round().clamp(-32768.0, 32767.0) as i16).to_le_bytes(),
            );
        }
    }
    let mut o = Vec::with_capacity(raw.len() + 20);
    o.extend_from_slice(&q.rate.to_le_bytes());
    o.extend_from_slice(&q.center_hz.to_le_bytes());
    // Compressed only when it helps: a burst of full-scale noise does not
    // compress, and a block that grew would cost the disk and the reader both.
    match zstd::encode_all(&raw[..], IQ_ZSTD_LEVEL) {
        Ok(z) if z.len() + 4 < raw.len() => {
            o.extend_from_slice(&((samples.len() as u32) | IQ_COMPRESSED).to_le_bytes());
            o.extend_from_slice(&(z.len() as u32).to_le_bytes());
            o.extend_from_slice(&z);
        }
        _ => {
            o.extend_from_slice(&(samples.len() as u32).to_le_bytes());
            o.extend_from_slice(&raw);
        }
    }
    o
}

fn fec_code(f: Fec) -> u8 {
    match f {
        Fec::Hamming => 1,
        Fec::Golay => 2,
        Fec::Bch => 3,
        Fec::ReedSolomon => 4,
        Fec::Convolutional => 5,
        Fec::Turbo => 6,
        Fec::Ldpc => 7,
    }
}

fn fec_of(c: u8) -> Option<Fec> {
    Some(match c {
        1 => Fec::Hamming,
        2 => Fec::Golay,
        3 => Fec::Bch,
        4 => Fec::ReedSolomon,
        5 => Fec::Convolutional,
        6 => Fec::Turbo,
        7 => Fec::Ldpc,
        _ => return None,
    })
}

/// A cursor over one layer's body, which returns `None` rather than panicking
/// on a short read: a torn or a foreign record must cost that record only.
struct Take<'a> {
    b: &'a [u8],
    at: usize,
}

impl<'a> Take<'a> {
    fn new(b: &'a [u8]) -> Self {
        Self { b, at: 0 }
    }

    fn bytes(&mut self, n: usize) -> Option<&'a [u8]> {
        let s = self.b.get(self.at..self.at + n)?;
        self.at += n;
        Some(s)
    }

    fn u8(&mut self) -> Option<u8> {
        Some(self.bytes(1)?[0])
    }

    fn u16(&mut self) -> Option<u16> {
        Some(u16::from_le_bytes(self.bytes(2)?.try_into().ok()?))
    }

    fn u32(&mut self) -> Option<u32> {
        Some(u32::from_le_bytes(self.bytes(4)?.try_into().ok()?))
    }

    fn u64(&mut self) -> Option<u64> {
        Some(u64::from_le_bytes(self.bytes(8)?.try_into().ok()?))
    }

    fn f32(&mut self) -> Option<f32> {
        Some(f32::from_le_bytes(self.bytes(4)?.try_into().ok()?))
    }

    fn f64(&mut self) -> Option<f64> {
        Some(f64::from_le_bytes(self.bytes(8)?.try_into().ok()?))
    }

    fn str(&mut self) -> Option<String> {
        let n = self.u16()? as usize;
        Some(String::from_utf8_lossy(self.bytes(n)?).into_owned())
    }

    fn rest(&mut self) -> &'a [u8] {
        let r = &self.b[self.at.min(self.b.len())..];
        self.at = self.b.len();
        r
    }
}

fn take_carrier(b: &[u8]) -> Option<Carrier> {
    let mut t = Take::new(b);
    let at_us = t.u64()?;
    let duration_us = t.u32()?;
    let center_hz = t.u64()?;
    let bandwidth_hz = t.u32()?;
    let rssi_dbfs = t.f32()?;
    let snr_db = t.f32()?;
    let source = SourceId(t.u64()?);
    Some(
        Carrier::heard(at_us, center_hz, bandwidth_hz, rssi_dbfs, snr_db, source)
            .lasting(duration_us),
    )
}

fn take_keying(b: &[u8]) -> Option<Keying> {
    let mut t = Take::new(b);
    let label = t.str()?;
    let measured = t.u8()? == 1;
    let confidence = t.f32()?;
    let params = KeyingParams {
        bandwidth_hz: t.f32()?,
        baud: t.f32()?,
        separation_hz: t.f32()?,
        sweep_hz_s: t.f32()?,
        symbol_period_us: t.f32()?,
        spreading: match t.u8()? {
            0 => None,
            sf => Some(sf),
        },
        // Read where the record has it. A field added to the end of a layer
        // is a compatible change only if its absence reads as absent rather
        // than as a short record, which is the whole point of the layer
        // carrying its own length.
        evm: t.f32().unwrap_or(0.0),
    };
    let kind = t.u8()?;
    let n = t.u32()? as usize;
    let symbols = match kind {
        0 => Symbols::None,
        1 => {
            let mut v = Vec::with_capacity(n.min(b.len() / 8));
            for _ in 0..n {
                let (mark, gap) = (t.u32()?, t.u32()?);
                v.push(Pulse { mark, gap });
            }
            Symbols::Pulses(v)
        }
        2 => Symbols::Hard(t.bytes(n)?.to_vec()),
        3 => {
            let mut v = Vec::with_capacity(n.min(b.len() / 4));
            for _ in 0..n {
                v.push(t.f32()?);
            }
            Symbols::Soft(v)
        }
        _ => return None,
    };
    let modulation = Modulation::from_label(&label).unwrap_or_default();
    let how = match measured {
        true => Knowledge::Measured { confidence },
        false => Knowledge::Configured,
    };
    Some(Keying { modulation, how, params, symbols })
}

fn take_frame(b: &[u8]) -> Option<Frame> {
    let mut t = Take::new(b);
    let n = t.u32()? as usize;
    let bytes = t.bytes(n)?.to_vec();
    let integrity = match t.u8()? {
        0 => Integrity::Unchecked,
        1 => Integrity::Passed,
        2 => Integrity::Failed,
        3 => Integrity::Corrected { symbols: t.u16()? },
        _ => return None,
    };
    let framing = match t.u8()? {
        0 => None,
        _ => {
            let preamble_bits = t.u32()?;
            let sync_len = t.u16()? as usize;
            let sync = t.bytes(sync_len)?.to_vec();
            let whitening = t.str()?;
            let fec = fec_of(t.u8()?);
            Some(Framing {
                preamble_bits,
                sync,
                // Interned, because a framing names one of a handful of
                // whiteners and the type is what a decoder compares.
                whitening: whitener(&whitening),
                fec,
            })
        }
    };
    Some(Frame { bytes, framing, integrity })
}

/// The whiteners a framing can name, as `&'static str` again after a round
/// trip. An unknown one reads as none rather than leaking a string.
fn whitener(s: &str) -> Option<&'static str> {
    ["pn9", "ccitt", "ibm", "ieee802154", "gsm"].into_iter().find(|w| *w == s)
}

fn take_iq(b: &[u8]) -> Option<Arc<IqBurst>> {
    let mut t = Take::new(b);
    let rate = t.f64()?;
    let center_hz = t.u64()?;
    let count = t.u32()?;
    let n = (count & !IQ_COMPRESSED) as usize;
    let raw: std::borrow::Cow<'_, [u8]> = match count & IQ_COMPRESSED != 0 {
        true => {
            let len = t.u32()? as usize;
            zstd::decode_all(t.bytes(len)?).ok()?.into()
        }
        false => t.rest().into(),
    };
    let samples = raw
        .chunks_exact(4)
        .take(n)
        .map(|c| {
            let i = i16::from_le_bytes([c[0], c[1]]) as f32 / 32767.0;
            let q = i16::from_le_bytes([c[2], c[3]]) as f32 / 32767.0;
            common::C32::new(i, q)
        })
        .collect();
    Some(Arc::new(IqBurst { rate, center_hz, samples }))
}

/// Where the records start, or `None` for a file this cannot read
///
/// The version is checked rather than skipped. A reader that steps over it
/// reads a future file's records as though they were its own, and reports
/// whatever that happens to make rather than saying it cannot.
fn after_header(buf: &[u8]) -> Option<usize> {
    if buf.len() < MAGIC.len() + 2 || &buf[..MAGIC.len()] != MAGIC {
        return None;
    }
    let v = u16::from_le_bytes(buf[MAGIC.len()..MAGIC.len() + 2].try_into().ok()?);
    (v <= VERSION).then_some(MAGIC.len() + 2)
}

/// One record's layers, as a packet with nothing decoded
///
/// An unknown layer is skipped by its length. A record with no carrier is not
/// a reception at all and is dropped.
fn take_record(body: &[u8]) -> Option<Packet> {
    let mut t = Take::new(body);
    let count = t.u8()?;
    let (mut carrier, mut keying, mut frame, mut iq) = (None, None, None, None);
    for _ in 0..count {
        let tag = t.u8()?;
        let len = t.u32()? as usize;
        let b = t.bytes(len)?;
        match tag {
            TAG_CARRIER => carrier = take_carrier(b),
            TAG_KEYING => keying = take_keying(b),
            TAG_FRAME => frame = take_frame(b),
            TAG_IQ => iq = take_iq(b),
            _ => {}
        }
    }
    let mut c = carrier?;
    c.iq = iq;
    Some(Packet { carrier: c, keying, frame, stack: Vec::new() })
}

pub fn parse(buf: &[u8]) -> Vec<Packet> {
    let Some(at) = after_header(buf) else { return Vec::new() };
    let mut out = Vec::new();
    let mut at = at;
    while at + 4 <= buf.len() {
        let len = u32::from_le_bytes(buf[at..at + 4].try_into().unwrap()) as usize;
        at += 4;
        let Some(body) = buf.get(at..at + len) else { break };
        at += len;
        if let Some(p) = take_record(body) {
            out.push(p);
        }
    }
    out
}

/// Read a log back
///
/// A log nothing can read is a log nobody keeps, so the reader ships with the
/// writer and is tested against it. A truncated final record, which is what a
/// receiver killed mid-write leaves, ends the iteration rather than failing.
pub fn read(path: impl AsRef<std::path::Path>) -> std::io::Result<Vec<Packet>> {
    let mut f = std::fs::File::open(path)?;
    let mut buf = Vec::new();
    f.read_to_end(&mut buf)?;
    Ok(parse(&buf))
}

/// Read a log one record at a time
///
/// A segment is a quarter of a gigabyte and its samples expand as they are
/// decompressed, so a day of a busy band does not fit in the memory of the
/// machine that recorded it. Anything walking a log rather than searching it
/// should come through here.
pub struct Records<R: std::io::BufRead> {
    r: R,
    started: bool,
}

impl<R: std::io::BufRead> Records<R> {
    pub fn new(r: R) -> Self {
        Self { r, started: false }
    }

    fn header(&mut self) -> Option<()> {
        let mut head = [0u8; MAGIC.len() + 2];
        self.r.read_exact(&mut head).ok()?;
        after_header(&head).map(|_| ())
    }
}

impl<R: std::io::BufRead> Iterator for Records<R> {
    type Item = Packet;

    fn next(&mut self) -> Option<Packet> {
        if !self.started {
            self.started = true;
            self.header()?;
        }
        loop {
            let mut len = [0u8; 4];
            self.r.read_exact(&mut len).ok()?;
            let mut body = vec![0u8; u32::from_le_bytes(len) as usize];
            self.r.read_exact(&mut body).ok()?;
            if let Some(p) = take_record(&body) {
                return Some(p);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::segments::day_of;
    use common::C32;

    /// 2026-08-31T12:00:00Z, in microseconds.
    const AT: u64 = 1_788_177_600_000_000;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-wspkt-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn carrier(center: u64) -> Carrier {
        Carrier::heard(AT, center, 31_250, -21.25, 18.5, SourceId(7)).lasting(184_000)
    }

    fn burst(center: u64) -> Packet {
        Packet::heard(carrier(center)).keyed(
            Keying::measured(
                Modulation::Ook,
                0.91,
                KeyingParams { bandwidth_hz: 11_700.0, baud: 1_500.0, ..KeyingParams::default() },
            )
            .with(Symbols::Pulses(vec![
                Pulse { mark: 500, gap: 1500 },
                Pulse { mark: 1500, gap: 500 },
                Pulse { mark: 500, gap: 9000 },
            ])),
        )
    }

    fn one(p: &Packet) -> Packet {
        let mut buf = MAGIC.to_vec();
        buf.extend_from_slice(&VERSION.to_le_bytes());
        buf.extend_from_slice(&record(p));
        let got = parse(&buf);
        assert_eq!(got.len(), 1, "one record in, {} out", got.len());
        got.into_iter().next().unwrap()
    }

    #[test]
    fn every_shape_of_evidence_comes_back_as_it_was_heard() {
        // The whole point of storing what the demodulator produced: what comes
        // back has to be good enough to decode again, whichever layer the
        // evidence is in.
        let timings = burst(433_920_000);
        assert_eq!(one(&timings), timings);

        // A chirp nothing read: a measurement and nothing else.
        let chirp = Packet::heard(carrier(868_300_000)).keyed(Keying::measured(
            Modulation::Chirp,
            0.83,
            KeyingParams {
                bandwidth_hz: 125_000.0,
                sweep_hz_s: 30_500_000.0,
                spreading: Some(9),
                ..KeyingParams::default()
            },
        ));
        assert_eq!(one(&chirp), chirp);

        // Bytes from a demodulator that makes frames rather than timings.
        let modes = Packet::heard(carrier(1_090_000_000))
            .keyed(Keying::configured(Modulation::Ppm))
            .framed(
                Frame::of(vec![0x8d, 0x48, 0x40, 0xd6, 0x20, 0x2c, 0xc3])
                    .checked(Integrity::Corrected { symbols: 2 })
                    .found_by(Framing {
                        preamble_bits: 16,
                        sync: vec![0xd3, 0x91],
                        whitening: Some("pn9"),
                        fec: Some(Fec::ReedSolomon),
                    }),
            );
        assert_eq!(one(&modes), modes);

        // Undecided symbols, for a decoder that does its own correction.
        let soft = Packet::heard(carrier(137_100_000))
            .keyed(Keying::configured(Modulation::Fsk2).with(Symbols::Soft(vec![0.5, -0.25, 1.0])));
        assert_eq!(one(&soft), soft);

        let hard = Packet::heard(carrier(162_025_000))
            .keyed(Keying::configured(Modulation::Gmsk).with(Symbols::Hard(vec![1, 0, 1, 1])));
        assert_eq!(one(&hard), hard);
    }

    #[test]
    fn a_keying_the_reader_did_not_expect_still_comes_back() {
        // The old log coded five modulations into a byte and lost the rest, so
        // a burst read as anything else came back saying nothing about how it
        // was keyed. The label is the enum's own, so there is one list.
        for m in [Modulation::Psk2, Modulation::Dqpsk, Modulation::Ofdm, Modulation::Dsss] {
            let p = Packet::heard(carrier(2_450_000_000)).keyed(Keying::configured(m));
            assert_eq!(one(&p).keying.unwrap().modulation, m, "{} was lost", m.label());
        }
    }

    #[test]
    fn a_frame_keeps_its_samples_and_its_carrier_keeps_the_rest() {
        let mut p = Packet::heard(carrier(869_618_000))
            .framed(Frame::of(vec![0x4c, 0x6f, 0x52, 0x61]).checked(Integrity::Passed));
        p.carrier.iq = Some(Arc::new(IqBurst {
            rate: 62_500.0,
            center_hz: 869_618_000,
            samples: (0..1000)
                .map(|i| C32::new((i as f32 * 0.01).sin() * 0.5, (i as f32 * 0.01).cos() * 0.5))
                .collect(),
        }));
        let got = one(&p);
        assert_eq!(got.frame, p.frame);
        assert_eq!(got.carrier.source, SourceId(7), "which front end heard it was lost");
        assert_eq!(got.carrier.duration_us, 184_000);
        let (a, b) = (got.carrier.iq.as_ref().unwrap(), p.carrier.iq.as_ref().unwrap());
        assert_eq!((a.rate, a.center_hz, a.samples.len()), (b.rate, b.center_hz, b.samples.len()));
        // Sixteen bits a component, so the samples come back within a
        // quantisation step and not bit for bit.
        let err = a.samples.iter().zip(&b.samples).map(|(x, y)| (x - y).norm()).fold(0.0, f32::max);
        assert!(err < 1e-4, "samples moved by {err}");
    }

    #[test]
    fn a_layer_the_reader_does_not_know_is_skipped() {
        // What makes a new layer a compatible change rather than a version
        // bump: an old reader steps over it by its length and reports the
        // record without it.
        let p = burst(433_920_000);
        let mut rec = record(&p);
        let extra = [0xEEu8, 4, 0, 0, 0, 1, 2, 3, 4];
        let body = u32::from_le_bytes(rec[..4].try_into().unwrap()) as usize + extra.len();
        rec[..4].copy_from_slice(&(body as u32).to_le_bytes());
        rec[4] += 1;
        rec.extend_from_slice(&extra);
        let mut buf = MAGIC.to_vec();
        buf.extend_from_slice(&VERSION.to_le_bytes());
        buf.extend_from_slice(&rec);
        assert_eq!(parse(&buf), vec![p]);
    }

    #[test]
    fn a_file_from_a_later_version_is_not_guessed_at() {
        let p = burst(433_920_000);
        let mut buf = MAGIC.to_vec();
        buf.extend_from_slice(&(VERSION + 1).to_le_bytes());
        buf.extend_from_slice(&record(&p));
        assert!(parse(&buf).is_empty(), "a future file was read as this one");
        assert!(parse(b"this is not a packet log at all").is_empty());
        assert!(parse(b"").is_empty());
    }

    #[test]
    fn the_samples_are_compressed_and_bounded() {
        // Two things a day's log depends on: a record carries the end of the
        // burst rather than every sample a front end had, and what it does
        // carry is compressed. A 2.4 GHz session wrote 122 GB in a day without
        // either.
        let d = dir("iqsize");
        let mut log = PacketLog::new(d.clone());
        let mut p = burst(868_300_000);
        // A quiet channel with a burst at the end of it, which is the shape
        // the ring behind a front end has.
        let mut samples = vec![C32::new(0.0, 0.0); 400_000];
        for (i, s) in samples.iter_mut().enumerate().skip(390_000) {
            let ph = i as f32 * 0.7;
            *s = C32::new(ph.cos() * 0.5, ph.sin() * 0.5);
        }
        p.carrier.iq =
            Some(Arc::new(IqBurst { rate: 1_000_000.0, center_hz: 868_300_000, samples }));
        log.write(&p);
        log.flush();
        let path = d.join(format!("{}.000.{EXT}", day_of(AT)));
        let size = std::fs::metadata(&path).unwrap().len();
        assert!(size < 200_000, "a record of {size} bytes for one burst");
        let got = read(&path).unwrap();
        let iq = got[0].carrier.iq.as_ref().expect("the samples came back");
        assert_eq!(iq.samples.len(), IQ_MAX_SAMPLES, "kept {} samples", iq.samples.len());
        // The end of the burst, which is where the signal was.
        assert!(iq.samples.last().unwrap().norm() > 0.4, "the tail is silence");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_receiver_killed_mid_write_costs_one_record() {
        let d = dir("torn");
        let mut log = PacketLog::new(d.clone());
        for _ in 0..3 {
            log.write(&burst(868_300_000));
        }
        log.flush();
        let path = d.join(format!("{}.000.{EXT}", day_of(AT)));
        let mut raw = std::fs::read(&path).unwrap();
        raw.truncate(raw.len() - 9);
        assert_eq!(parse(&raw).len(), 2, "a torn tail took a good record with it");
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_log_is_read_back_without_holding_it_all() {
        let d = dir("stream");
        let mut log = PacketLog::new(d.clone());
        for _ in 0..500 {
            log.write(&burst(868_300_000));
        }
        log.flush();
        let path = d.join(format!("{}.000.{EXT}", day_of(AT)));
        let f = std::io::BufReader::new(std::fs::File::open(&path).unwrap());
        let mut n = 0;
        for p in Records::new(f) {
            assert_eq!(p.carrier.center_hz, 868_300_000);
            n += 1;
        }
        assert_eq!(n, 500);
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn a_carrier_with_nothing_on_it_is_not_written() {
        // A record of something having happened, in a file whose whole purpose
        // is to be decoded again later.
        let d = dir("bare");
        let mut log = PacketLog::new(d.clone());
        log.write(&Packet::heard(carrier(446_000_000)));
        log.write(&burst(868_300_000));
        log.flush();
        let got = read(d.join(format!("{}.000.{EXT}", day_of(AT)))).unwrap();
        assert_eq!(got.len(), 1, "the bare carrier went into the log");
        assert!(got[0].keying.is_some());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_day_rolls_over_into_a_second_file() {
        let d = dir("roll");
        let mut log = PacketLog::new(d.clone());
        log.write(&burst(433_920_000));
        let mut tomorrow = burst(433_920_000);
        tomorrow.carrier.at_us += 86_400_000_000;
        log.write(&tomorrow);
        assert_eq!(
            read(d.join(format!("{}.000.{EXT}", day_of(AT)))).unwrap().len(),
            1,
            "yesterday was lost at the roll"
        );
        log.flush();
        assert!(d.join(format!("{}.000.{EXT}", day_of(AT + 86_400_000_000))).exists());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn the_oldest_segments_go_to_keep_the_folder_under_its_limit() {
        let d = dir("prune");
        std::fs::create_dir_all(&d).unwrap();
        for day in ["2026-08-28", "2026-08-29", "2026-08-30"] {
            std::fs::write(d.join(format!("{day}.{EXT}")), vec![0u8; 40_000]).unwrap();
        }
        let mut log = PacketLog::new(d.clone()).with_cap(Some(100_000));
        for _ in 0..100 {
            log.write(&burst(868_300_000));
        }
        log.flush();
        assert!(!d.join(format!("2026-08-28.{EXT}")).exists(), "the oldest segment was kept");
        assert!(d.join(format!("2026-08-30.{EXT}")).exists(), "a recent segment was thrown away");
        assert!(!log.full(), "logging stopped although there was room to make");
        assert!(log.total() < 100_000, "the folder is {} bytes", log.total());
        let _ = std::fs::remove_dir_all(&d);
    }
}
