//! P25 phase 1 as a graph node.
//!
//! The same front end as DMR, because the waveform is the same: the channel
//! is narrowband FM, so the node mixes it down, filters it, discriminates it
//! and hands the result to `dsp::c4fm::SymbolClock`, which keeps a symbol
//! clock across blocks. P25 is FDMA rather than TDMA, so what comes out is
//! one continuous stream of 4800 baud dibits with a 48-bit sync word every
//! frame, and the framer hunts that sync rather than following a burst clock.
//!
//! What a frame says is read by `decode::p25`: the network access code and
//! the data unit id out of the BCH-protected network identifier, and out of a
//! voice frame the link control, which names the talkgroup and the radio, or
//! the encryption sync, which names the key the speech is under.
//!
//! No speech: the IMBE vocoder is not built, so a voice frame reaches the bus
//! as a packet naming the call and carrying its bits, and the call list gets
//! its 180 ms of airtime without any audio behind it.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use common::bands::Usage;
use decode::p25::{self, Duid, Encryption, LinkControl};
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The P25 national interoperability calling channel, VCALL10, and only the
/// default before the scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 155_752_500.0;

/// 12.5 kHz channel grid.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// Symbol rate.
const BAUD: f64 = 4_800.0;

/// Discriminator output rate: ten samples a symbol.
const AUDIO_HZ: f64 = 48_000.0;

/// Nominal outer-symbol deviation: C4FM keys +-1800 Hz and +-600 Hz. Nothing
/// downstream depends on the exact value, since the slicer fits its own
/// levels.
const DEVIATION_HZ: f64 = 1_800.0;

/// One-sided filter cutoff, wide enough for the outer symbols and the
/// transmitter's drift.
const FILTER_CUTOFF_HZ: f64 = 7_000.0;

/// Roll-off of the raised cosine P25 shapes its symbols with, and so of the
/// matched filter here (TIA-102.BAAA clause 6).
const RRC_ALPHA: f64 = 0.2;

/// Wrong dibits tolerated in a 48-bit sync word. Two of twenty-four: with
/// three the false match rate off noise stops being negligible, and a frame
/// needing more than two put back has a network identifier that will not
/// pass its BCH either.
const SYNC_TOLERANCE: usize = 2;

/// Symbols held before the framer will slice: a whole voice frame, because
/// the four levels are fitted over the window and a sync word carries only
/// the outer two.
const WINDOW: usize = p25::LDU_DIBITS;

/// Channel samples kept behind the symbol clock, in seconds: the framer
/// reads a frame once the one after it has arrived and keeps a frame of
/// history behind the hunt, so a frame's own samples are up to three frames
/// old by the time its packet is built.
const KEEP_S: f64 = (4 * p25::LDU_DIBITS) as f64 / BAUD;

/// Tag identifying a packet body this node wrote. "P1".
///
/// The body is what the frame said about itself: the network access code, the
/// data unit id, and where the frame carried identities, those. The link
/// control or encryption sync it was read from travels with it, so a reader
/// later can take more out of the same bytes.
const P25_TAG: [u8; 2] = *b"P1";

/// Tag, NAC, data unit id, flags, destination, source.
const HEAD_LEN: usize = 2 + 2 + 1 + 1 + 4 + 4;

const FLAG_HAVE_LC: u8 = 0x01;
const FLAG_GROUP: u8 = 0x02;
const FLAG_ENCRYPTED: u8 = 0x04;
const FLAG_EMERGENCY: u8 = 0x08;
const FLAG_HAVE_ES: u8 = 0x10;

/// Speech is IMBE at 4400 bit/s under 2800 of FEC, and P25 phase 1 has no
/// other vocoder.
const CODEC: &str = "IMBE 4400";

/// One voice frame is nine IMBE frames of 20 ms.
const VOICE_SECONDS: f64 = 0.18;

/// What one frame turned out to be.
pub struct P25Frame {
    /// Absolute symbol index the sync word began at.
    pub at: usize,
    pub nac: u16,
    pub duid: Duid,
    pub lc: Option<LinkControl>,
    pub es: Option<Encryption>,
}

/// Serialise a frame as the bytes that reach the bus.
fn encode_frame(f: &P25Frame) -> Vec<u8> {
    let mut v = P25_TAG.to_vec();
    v.extend_from_slice(&f.nac.to_be_bytes());
    v.push(f.duid.as_bits());
    let mut flags = 0u8;
    let (mut dst, mut src) = (0u32, 0u32);
    if let Some(lc) = &f.lc {
        flags |= FLAG_HAVE_LC;
        if let Some(tg) = lc.talkgroup() {
            flags |= FLAG_GROUP;
            dst = u32::from(tg);
        } else if let Some(t) = lc.target() {
            dst = t;
        }
        src = lc.source().unwrap_or(0);
        if lc.encrypted() {
            flags |= FLAG_ENCRYPTED;
        }
        if lc.emergency() {
            flags |= FLAG_EMERGENCY;
        }
    }
    if let Some(es) = &f.es {
        flags |= FLAG_HAVE_ES;
        if !es.clear() {
            flags |= FLAG_ENCRYPTED;
        }
    }
    v.push(flags);
    v.extend_from_slice(&dst.to_be_bytes());
    v.extend_from_slice(&src.to_be_bytes());
    if let Some(lc) = &f.lc {
        v.extend_from_slice(&lc.bytes);
    } else if let Some(es) = &f.es {
        v.extend_from_slice(&es.mi);
        v.push(es.algid);
        v.extend_from_slice(&es.kid.to_be_bytes());
    }
    v
}

/// Recognise and describe a P25 row for the packet log. `None` for anything
/// this node did not write, so it is safe to try on every frame.
///
/// A voice frame is 180 ms of the channel and says so; where it carried the
/// link control it names the talkgroup and the radio, which is what puts the
/// call in the call list rather than only in the log. Nothing here is written
/// by a person, so nothing is marked as written.
pub fn p25_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    if bytes.len() < HEAD_LEN || bytes[..2] != P25_TAG {
        return None;
    }
    let nac = u16::from_be_bytes([bytes[2], bytes[3]]);
    let duid = Duid::from_bits(bytes[4]);
    let flags = bytes[5];
    let dst = u32::from_be_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]);
    let src = u32::from_be_bytes([bytes[10], bytes[11], bytes[12], bytes[13]]);
    let payload = &bytes[HEAD_LEN..];

    let mut fields: Vec<(String, Value)> = vec![
        ("nac".to_string(), Value::Text(format!("{nac:03X}"))),
        ("frame".to_string(), Value::Text(duid.name().to_string())),
    ];
    if flags & FLAG_HAVE_LC != 0 {
        fields.push(("to".to_string(), Value::Text(dst.to_string())));
        fields.push(("from".to_string(), Value::Text(src.to_string())));
        fields.push((
            "call_type".to_string(),
            Value::Text(if flags & FLAG_GROUP != 0 { "group" } else { "private" }.to_string()),
        ));
        if flags & FLAG_EMERGENCY != 0 {
            fields.push(("emergency".to_string(), Value::Bool(true)));
        }
    }
    if flags & FLAG_HAVE_ES != 0 && payload.len() == 12 {
        let (algid, kid) = (payload[9], u16::from_be_bytes([payload[10], payload[11]]));
        if algid != Encryption::CLEAR {
            fields.push(("algorithm".to_string(), Value::Text(p25::algorithm(algid).to_string())));
            fields.push(("key_id".to_string(), Value::Int(i64::from(kid))));
        }
    }
    if flags & FLAG_ENCRYPTED != 0 {
        fields.push(("encrypted".to_string(), Value::Bool(true)));
    }
    if duid.voice() {
        fields.push(("voice".to_string(), Value::Bool(true)));
        fields.push(("codec".to_string(), Value::Text(CODEC.to_string())));
        fields.push(("seconds".to_string(), Value::Float(VOICE_SECONDS)));
        fields.push(("live".to_string(), Value::Bool(true)));
    }

    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes(duid.label(), center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Fsk4)
        // The network identifier passed its BCH and, on a voice frame, the
        // words passed Hamming and Reed-Solomon; nothing reaches here that
        // did not.
        .with_crc(Some(true));
    if flags & FLAG_HAVE_LC != 0 {
        use pipeline::event::Party;
        let to = if flags & FLAG_GROUP != 0 {
            Party::group(dst.to_string())
        } else {
            Party::unit(dst.to_string())
        };
        d.link = Some(pipeline::event::Link::between(Party::unit(src.to_string()), to));
        d.identity = Some(common::Identity::new("p25", src.to_string()));
    }
    if duid.voice() {
        d.airtime = Some(common::Airtime {
            seconds: VOICE_SECONDS,
            voice: true,
            live: true,
            secrecy: if flags & FLAG_ENCRYPTED != 0 {
                common::Secrecy::Encrypted(None)
            } else {
                common::Secrecy::Clear
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
/// Each frame is found by its own sync word rather than by a clock: P25 puts
/// frames back to back with no gaps, and a hunt costs one comparison a symbol
/// where a predicted boundary would need every frame length in the standard.
struct Framer {
    marks: Vec<f32>,
    base: usize,
    scan: usize,
    /// Which way up the discriminator is, once a frame has settled it.
    polarity: Option<bool>,
}

impl Framer {
    fn new() -> Self {
        Self { marks: Vec::new(), base: 0, scan: 0, polarity: None }
    }

    fn reset(&mut self) {
        self.marks.clear();
        self.base = 0;
        self.scan = 0;
        self.polarity = None;
    }

    /// Level index to dibit: P25 sends +3 as 01, +1 as 00, -1 as 10 and -3
    /// as 11 (TIA-102.BAAA clause 6.2).
    fn dibit(level: u8, flip: bool) -> u8 {
        match if flip { 3 - level } else { level } {
            3 => 1,
            2 => 0,
            1 => 2,
            _ => 3,
        }
    }

    /// Append recovered symbols and pull out the frames they complete.
    fn push(&mut self, syms: &[f32], out: &mut Vec<P25Frame>) {
        self.marks.extend_from_slice(syms);
        if self.marks.len() < WINDOW {
            return;
        }
        let Some(levels) = dsp::c4fm::slice(&self.marks) else {
            return;
        };
        let mut i = self.scan.saturating_sub(self.base);
        'hunt: while i + p25::HEAD_DIBITS + p25::SYNC_DIBITS.len() <= levels.len() {
            let polarities: [bool; 2] = match self.polarity {
                Some(p) => [p, p],
                None => [false, true],
            };
            let mut read = None;
            for flip in polarities {
                let wrong = p25::SYNC_DIBITS
                    .iter()
                    .enumerate()
                    .filter(|(k, d)| Self::dibit(levels[i + k], flip) != **d)
                    .count();
                if wrong > SYNC_TOLERANCE {
                    continue;
                }
                // A voice frame is only read once all of it has arrived; the
                // scan stays where it is until then.
                let want = (i + p25::LDU_DIBITS).min(levels.len());
                let dibits: Vec<u8> =
                    levels[i..want].iter().map(|l| Self::dibit(*l, flip)).collect();
                let data = p25::status_free(&dibits);
                let Some(nid) = p25::nid(&data) else { continue };
                if nid.duid.voice() && data.len() < p25::LDU_DATA_DIBITS {
                    // The rest of the frame has not arrived. Stop here with
                    // the hunt where it is, so the next block reads it once.
                    break 'hunt;
                }
                let words = p25::words(&data);
                let (lc, es) = match (nid.duid, words) {
                    (Duid::Voice1, Some(w)) => (LinkControl::from_words(&w), None),
                    (Duid::Voice2, Some(w)) => (None, Encryption::from_words(&w)),
                    _ => (None, None),
                };
                // A link control that is itself enciphered describes nothing,
                // so it is carried but not read for identities.
                let lc = lc.filter(|lc| !lc.protected());
                read = Some((
                    flip,
                    P25Frame { at: self.base + i, nac: nid.nac, duid: nid.duid, lc, es },
                ));
                break;
            }
            match read {
                Some((flip, frame)) => {
                    self.polarity = Some(flip);
                    out.push(frame);
                    i += p25::HEAD_DIBITS;
                }
                None => i += 1,
            }
        }
        self.scan = self.base + i;
        // Drain what is behind the hunt, keeping a frame of history so a sync
        // straddling two blocks is still found.
        let keep = self.scan.saturating_sub(p25::LDU_DIBITS);
        if keep > self.base {
            let drop = (keep - self.base).min(self.marks.len());
            self.marks.drain(..drop);
            self.base += drop;
        }
    }
}

pub struct P25Node {
    channel_hz: f64,
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

impl Default for P25Node {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl P25Node {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, FILTER_CUTOFF_HZ, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            rrc: FirDecimReal::new(rrc_taps(AUDIO_HZ / BAUD, RRC_ALPHA, 8), 1),
            clock: SymbolClock::new(AUDIO_HZ, BAUD),
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

    /// One frame as a packet: what it said, the level it was heard at and the
    /// samples it was sliced from, found by the frame's symbol index.
    fn packet(&mut self, frame: &P25Frame) -> common::Packet {
        let at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        self.accepted += 1;
        let sps = self.audio_rate / BAUD;
        let bytes = encode_frame(frame);
        let start = (frame.at as f64 * sps) as u64;
        let len = (p25::LDU_DIBITS as f64 * sps) as usize;
        let snr_db = self.meter.snr_db_at(start, len);
        let measured = self.meter.frame_measured(bytes, start, len, snr_db);
        common::Packet::of_frame(at_us, CHANNEL_WIDTH_HZ as u32, measured)
    }
}

impl Simple for P25Node {
    fn name(&self) -> &str {
        "p25"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("p25 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("p25 needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, FILTER_CUTOFF_HZ, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.rrc = FirDecimReal::new(rrc_taps(audio_rate / BAUD, RRC_ALPHA, 8), 1);
        self.clock = SymbolClock::new(audio_rate, BAUD);
        self.framer = Framer::new();
        self.audio_rate = audio_rate;
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, KEEP_S);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
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

pub struct P25;

impl Protocol for P25 {
    fn id(&self) -> &'static str {
        "p25"
    }
    fn label(&self) -> &'static str {
        "P25"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["apco25", "p25p1"]
    }
    /// A 12.5 kHz channel anywhere: P25 is on VHF, UHF, 700 and 800 MHz, and
    /// what makes one a P25 channel is what is keyed on it.
    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Utility])
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        p25_decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: CHANNEL_WIDTH_HZ,
            feed_rate_hz: 192_000.0,
            span_wide: false,
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Packets]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "p25",
    summary: "One P25 phase 1 channel: C4FM at 4800 baud, talkgroup and radio id",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(P25Node::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A group voice call: talkgroup 1234, radio 5679413.
    fn group_lc() -> [u8; 9] {
        [0x00, 0x00, 0x00, 0x00, 0x04, 0xd2, 0x56, 0xa9, 0x35]
    }

    /// The encryption sync of a call in the clear.
    fn clear_es() -> [u8; 12] {
        let mut es = [0u8; 12];
        es[9] = Encryption::CLEAR;
        es
    }

    /// Key a run of dibits as C4FM at `rate`, at the given offset from the
    /// tuner's centre: the deviation P25 keys, through an integrator.
    fn keyed(dibits: &[u8], rate: f64, offset_hz: f64, noise: f32) -> Vec<common::C32> {
        let sps = rate / BAUD;
        let mut out = Vec::with_capacity((dibits.len() as f64 * sps) as usize);
        let mut phase = 0.0f64;
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut noise_sample = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 40) as f32 / 8_388_608.0 - 0.5) * noise
        };
        for d in dibits {
            // 01 is +1800 Hz, 00 is +600, 10 is -600, 11 is -1800.
            let hz = match d & 3 {
                1 => 1_800.0,
                0 => 600.0,
                2 => -600.0,
                _ => -1_800.0,
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
    fn replay(iq: &[common::C32], rate: f64, center: f64, hz: f64) -> Vec<Decoded> {
        let mut node = P25Node::new(hz);
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
                        rows.extend(p25_decoded(&f.bytes, common::Hz(p.center_hz())));
                    }
                }
            }
        }
        rows
    }

    /// Dibits with all four levels and no sync word in them, keyed either
    /// side of a call: ahead of it so the symbol clock is running as a
    /// receiver's would be when a call starts, and after it so the last
    /// frame is complete in the framer's window. Random rather than a
    /// repeating pattern, which a timing loop locks to the wrong phase of.
    fn tail() -> Vec<u8> {
        let mut seed = 0x51ed_2701_dead_1234u64;
        (0..1_000)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (seed >> 33) as u8 & 3
            })
            .collect()
    }

    /// Ten voice frames, alternating the two kinds, as a call is sent.
    fn call(nac: u16) -> Vec<u8> {
        let mut dibits = tail();
        for i in 0..10 {
            let (duid, hex) = if i % 2 == 0 {
                (Duid::Voice1, p25::hex_words(&group_lc()))
            } else {
                (Duid::Voice2, p25::hex_words(&clear_es()))
            };
            dibits.extend(p25::frame_dibits(nac, duid, &hex));
        }
        dibits.extend(tail());
        dibits
    }

    #[test]
    fn negotiates_a_channel_inside_the_span() {
        let mut n = P25Node::default();
        assert!(n.negotiate(&spec(2_048_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_048_000.0, 460_000_000.0)).is_err());
    }

    #[test]
    fn labels_only_its_own_frames() {
        let bytes = encode_frame(&P25Frame {
            at: 0,
            nac: 0x293,
            duid: Duid::Voice1,
            lc: Some(LinkControl { bytes: group_lc(), repaired: 0 }),
            es: None,
        });
        let d = p25_decoded(&bytes, Hz(155_752_500)).expect("a P25 row");
        assert_eq!(d.protocol, "P25-Voice");
        let get = |k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert_eq!(get("nac"), "293");
        assert_eq!(get("to"), "1234");
        assert_eq!(get("from"), "5679413");
        assert_eq!(get("call_type"), "group");
        let air = d.airtime.as_ref().expect("a voice frame with no airtime");
        assert_eq!(air.seconds, VOICE_SECONDS);
        assert!(air.voice && air.live);
        assert_eq!(air.codec, Some(CODEC));
        assert_eq!(air.secrecy, common::Secrecy::Clear);
        // Nobody wrote this, so it is not a message.
        assert!(!d.written);
        assert!(p25_decoded(b"random", Hz(0)).is_none());
        assert!(p25_decoded(b"P1", Hz(0)).is_none());
    }

    /// A keyed call off the node's own front end: every frame of it read, the
    /// talkgroup and radio on every voice frame that carried the link
    /// control, and the key named on the other half.
    #[test]
    fn reads_a_keyed_call_through_the_front_end() {
        let rate = 96_000.0;
        let iq = keyed(&call(0x293), rate, 0.0, 0.0);
        let rows = replay(&iq, rate, 155_752_500.0, 155_752_500.0);
        assert_eq!(rows.len(), 10, "ten frames keyed, {} read", rows.len());
        assert!(rows.iter().all(|d| d.protocol == "P25-Voice"));
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert!(rows.iter().all(|d| get(d, "nac") == "293"));
        let named = rows.iter().filter(|d| get(d, "to") == "1234").count();
        assert_eq!(named, 5, "five frames carry the link control");
        assert!(
            rows.iter().filter(|d| get(d, "to") == "1234").all(|d| get(d, "from") == "5679413")
        );
        // Nothing is encrypted, so no row claims a key.
        assert_eq!(rows.iter().filter(|d| get(d, "encrypted") == "true").count(), 0);
        // Every row is 180 ms of speech in the call list.
        assert_eq!(rows.iter().filter(|d| d.airtime.is_some()).count(), 10);
    }

    /// Off frequency and in noise, which is what a receiver actually hands
    /// the node. Measured: 2.5 kHz off with noise at a tenth of the carrier
    /// still reads every frame.
    #[test]
    fn reads_a_call_off_frequency_and_in_noise() {
        let rate = 96_000.0;
        let iq = keyed(&call(0x4d2), rate, -2_500.0, 0.1);
        let rows = replay(&iq, rate, 155_755_000.0, 155_752_500.0);
        assert_eq!(rows.len(), 10);
        let nacs = rows
            .iter()
            .filter(|d| d.fields.iter().any(|(k, v)| k == "nac" && v.to_string() == "4D2"))
            .count();
        assert_eq!(nacs, 10);
    }

    /// An enciphered call says so, and says under which key, without
    /// pretending to any speech.
    #[test]
    fn an_enciphered_call_names_its_key() {
        let mut lc = group_lc();
        // Service options: enciphered, and an emergency.
        lc[2] = 0xc0;
        let mut es = [0u8; 12];
        es[..9].copy_from_slice(&[9, 8, 7, 6, 5, 4, 3, 2, 1]);
        es[9] = 0xaa;
        es[10] = 0x00;
        es[11] = 0x2a;
        let mut dibits = tail();
        dibits.extend(p25::frame_dibits(0x293, Duid::Voice1, &p25::hex_words(&lc)));
        dibits.extend(p25::frame_dibits(0x293, Duid::Voice2, &p25::hex_words(&es)));
        dibits.extend(tail());
        let rate = 96_000.0;
        let rows = replay(&keyed(&dibits, rate, 0.0, 0.0), rate, DEFAULT_HZ, DEFAULT_HZ);
        assert_eq!(rows.len(), 2);
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert_eq!(get(&rows[0], "emergency"), "true");
        assert_eq!(get(&rows[0], "encrypted"), "true");
        assert_eq!(get(&rows[1], "algorithm"), "ADP");
        assert_eq!(get(&rows[1], "key_id"), "42");
        assert_eq!(rows[0].airtime.as_ref().unwrap().secrecy, common::Secrecy::Encrypted(None));
    }

    /// Minutes of noise, and nothing said about any of it.
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
        let rows = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ);
        assert_eq!(rows.len(), 0, "two minutes of noise read as {} frames", rows.len());
    }
}
