//! FT8 and FT4 as graph nodes: one HF channel in, every station in it out.
//!
//! The channel is 3 kHz of an SSB passband and it holds dozens of
//! transmissions at once, so this does not tune a signal. It mixes the dial
//! to zero, decimates to 12 kHz, and hands whole slots to
//! [`dsp::mfsk::Slot`], which finds every Costas-synchronised transmission in
//! the passband and hands back soft bits for each. The (174,91) code and the
//! CRC-14 behind it are [`decode::ft8`], and what reaches the bus is a
//! transmission that satisfied both.
//!
//! # The clock
//!
//! Nothing else the receiver reads is aligned to the wall clock. A station
//! keys at the start of a fifteen-second slot (seven and a half for FT4) and
//! stops 12.6 seconds later, so the slot boundary is where the decoder cuts
//! and the wall clock is the only thing that says where that is. The node
//! throws away whatever it is handed until the next boundary and then works
//! in whole slots. A station keyed up to two seconds late is still read,
//! because the sync search covers every offset the slot leaves.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
use decode::ft8;
use dsp::mfsk::{self, Slot, Waveform};
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The 20 m FT8 dial, which is the busiest frequency on any amateur band.
pub const DEFAULT_HZ: f64 = 14_074_000.0;

/// The 20 m FT4 dial.
pub const FT4_DEFAULT_HZ: f64 = 14_080_000.0;

/// The channel a decoder needs cut out for it: the dial and the passband
/// above it, plus as much below, because a channel is cut symmetrically
/// around the frequency it is placed at and every station sits above the
/// dial.
pub const CHANNEL_WIDTH_HZ: f64 = 6_000.0;

/// The passband itself, which is what a station shares with every other.
pub const PASSBAND_HZ: f64 = 3_000.0;

/// Rate the channel is read at. Two samples per hertz of passband, and a
/// whole number of samples a symbol at both modes' baud rates.
const AUDIO_HZ: f64 = 12_000.0;

/// The part of the passband stations use, above the dial. Below 200 Hz is a
/// receiver's own high-pass and its carrier leak.
const BAND: (f64, f64) = (200.0, PASSBAND_HZ);

/// Belief propagation passes before a candidate is given up on. Measured on
/// a synthesised channel of eight stations: 20 passes reads all eight, 10
/// reads seven, and 50 reads no more than 20 at four times the cost.
const LDPC_PASSES: usize = 40;

/// The dial frequencies stations are told to use, by band, in hertz.
const FT8_DIALS: &[f64] = &[
    1_840_000.0,
    3_573_000.0,
    5_357_000.0,
    7_074_000.0,
    10_136_000.0,
    14_074_000.0,
    18_100_000.0,
    21_074_000.0,
    24_915_000.0,
    28_074_000.0,
    50_313_000.0,
    144_174_000.0,
];

const FT4_DIALS: &[f64] = &[
    3_575_000.0,
    7_047_500.0,
    10_140_000.0,
    14_080_000.0,
    18_104_000.0,
    21_140_000.0,
    24_919_000.0,
    28_180_000.0,
    50_318_000.0,
];

/// Which of the two modes a node is reading.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Mode {
    #[default]
    Ft8,
    Ft4,
}

impl Mode {
    pub const ALL: [Mode; 2] = [Mode::Ft8, Mode::Ft4];

    fn waveform(self) -> Waveform {
        match self {
            Mode::Ft8 => mfsk::FT8,
            Mode::Ft4 => mfsk::FT4,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Mode::Ft8 => "FT8",
            Mode::Ft4 => "FT4",
        }
    }

    /// The byte a frame opens with, so a reader off the bus knows which mode
    /// read it without guessing from the dial.
    fn tag(self) -> u8 {
        match self {
            Mode::Ft8 => 8,
            Mode::Ft4 => 4,
        }
    }

    fn of_tag(tag: u8) -> Option<Mode> {
        Mode::ALL.iter().copied().find(|m| m.tag() == tag)
    }
}

impl std::fmt::Display for Mode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

impl std::str::FromStr for Mode {
    type Err = ();
    fn from_str(s: &str) -> std::result::Result<Self, ()> {
        match s.trim().to_ascii_lowercase().as_str() {
            "ft4" => Ok(Mode::Ft4),
            "ft8" => Ok(Mode::Ft8),
            _ => Err(()),
        }
    }
}

pub struct Ft8Node {
    dial_hz: f64,
    mode: Mode,
    rate: f64,
    factor: usize,
    mixer: Mixer,
    decim: FirDecim,
    slot: Slot,
    mixed: Vec<common::C32>,
    audio: Vec<common::C32>,
    /// Samples still to be thrown away before the next slot boundary.
    skip: usize,
    /// What has come in since the last boundary.
    buffer: Vec<common::C32>,
    meter: crate::FrameMeter,
    slots: u64,
    read: u64,
}

impl Default for Ft8Node {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, Mode::Ft8)
    }
}

impl Ft8Node {
    pub fn new(dial_hz: f64, mode: Mode) -> Self {
        let mut n = Self {
            dial_hz,
            mode,
            rate: AUDIO_HZ,
            factor: 1,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, PASSBAND_HZ + 500.0, 60.0),
            slot: Slot::new(AUDIO_HZ, mode.waveform()),
            mixed: Vec::new(),
            audio: Vec::new(),
            skip: 0,
            buffer: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, dial_hz as u64, 1.0),
            slots: 0,
            read: 0,
        };
        n.align_to_clock();
        n
    }

    /// Transmissions read since the node was built.
    pub fn read_count(&self) -> u64 {
        self.read
    }

    /// Slots looked at since the node was built.
    pub fn slots(&self) -> u64 {
        self.slots
    }

    fn audio_rate(&self) -> f64 {
        self.rate / self.factor as f64
    }

    fn rebuild(&mut self) {
        let audio = self.audio_rate();
        self.decim = FirDecim::design_hz(self.rate, self.factor, PASSBAND_HZ + 500.0, 60.0);
        self.slot = Slot::new(audio, self.mode.waveform());
        self.meter = crate::FrameMeter::new(audio, self.dial_hz as u64, 1.0);
        self.buffer.clear();
        self.align_to_clock();
    }

    /// Throw away everything until the next slot boundary, which is a whole
    /// number of slots since the hour by definition of both modes.
    fn align_to_clock(&mut self) {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        let slot_s = self.mode.waveform().slot_s;
        let into = now.rem_euclid(slot_s);
        self.skip = ((slot_s - into) * self.audio_rate()).round() as usize;
    }

    /// Read one slot: every candidate the sync search found, through the
    /// code and the check, deduped where one station was found twice.
    fn read_slot(&mut self, slot: &[common::C32]) -> Vec<(Vec<u8>, f64, f32)> {
        self.slots += 1;
        let mut out: Vec<(Vec<u8>, f64, f32)> = Vec::new();
        for heard in self.slot.read(slot, BAND) {
            let (bits, failed) = ft8::code().decode(&heard.llr, LDPC_PASSES);
            if failed != 0 {
                continue;
            }
            // What was on the air, which for FT4 is the payload through its
            // scrambling sequence: the check the station computed is over
            // that, so undoing it here would fail the check.
            let message = &bits[..ft8::MESSAGE_BITS];
            if !ft8::crc_ok(message) {
                continue;
            }
            // And it has to say something. The all-zero codeword satisfies
            // every check and carries a zero CRC, so weak soft bits settle
            // on it and the payload layer is what refuses it.
            let mut payload = message.to_vec();
            if self.mode == Mode::Ft4 {
                ft8::scramble_ft4(&mut payload);
            }
            if ft8::unpack(&payload).is_none() {
                continue;
            }
            let mut bytes = vec![self.mode.tag()];
            bytes.extend(pack(message));
            if out.iter().any(|(b, _, _)| *b == bytes) {
                continue;
            }
            out.push((bytes, heard.freq_hz, heard.snr_db));
        }
        self.read += out.len() as u64;
        out
    }
}

/// Bits as bytes, the first bit most significant, padded with zeros.
fn pack(bits: &[bool]) -> Vec<u8> {
    bits.chunks(8)
        .map(|c| c.iter().enumerate().fold(0u8, |acc, (k, &b)| acc | u8::from(b) << (7 - k)))
        .collect()
}

fn unpack_bits(bytes: &[u8], n: usize) -> Vec<bool> {
    (0..n).map(|k| bytes[k / 8] >> (7 - k % 8) & 1 != 0).collect()
}

impl Simple for Ft8Node {
    fn name(&self) -> &str {
        match self.mode {
            Mode::Ft8 => FT8_DESC.name,
            Mode::Ft4 => FT4_DESC.name,
        }
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![("slots".into(), self.slots.to_string()), ("read".into(), self.read.to_string())]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ft8 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.dial_hz - center).abs() > rate / 2.0 - PASSBAND_HZ {
            return Err(common::Error::other("ft8 needs its passband inside the span"));
        }
        self.rate = rate;
        self.factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        if self.audio_rate() < 2.0 * PASSBAND_HZ {
            return Err(common::Error::other("ft8 needs a 3 kHz passband"));
        }
        self.mixer = Mixer::new(center - self.dial_hz, rate);
        self.rebuild();

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.dial_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.audio.clear();
        self.decim.process(&self.mixed, &mut self.audio);
        self.meter.feed(&self.audio);

        let audio = std::mem::take(&mut self.audio);
        let mut rest = audio.as_slice();
        if self.skip > 0 {
            let drop = self.skip.min(rest.len());
            self.skip -= drop;
            rest = &rest[drop..];
        }
        self.buffer.extend_from_slice(rest);
        self.audio = audio;

        let want = self.slot.slot_samples();
        while self.buffer.len() >= want {
            let slot: Vec<common::C32> = self.buffer.drain(..want).collect();
            for (bytes, freq_hz, snr_db) in self.read_slot(&slot) {
                let center = (self.dial_hz + freq_hz).round().max(0.0) as u64;
                let frame = self.meter.frame(bytes);
                o.frames_mut().push(common::Frame { snr_db, ..frame }.at(center));
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.buffer.clear();
        self.meter.reset();
        self.align_to_clock();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(CHANNEL_HZ, self.dial_hz, 1e5..=1e9).unit("Hz").label("Dial")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => self.dial_hz = v.as_f64().unwrap_or(self.dial_hz),
            _ => return Err(common::Error::other(format!("ft8: unknown parameter {name:?}"))),
        }
        self.rebuild();
        Ok(())
    }
}

/// What a frame off the bus says: the message, and who said what to whom.
pub fn ft8_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let mode = Mode::of_tag(*bytes.first()?)?;
    if bytes.len() != 1 + ft8::MESSAGE_BITS.div_ceil(8) {
        return None;
    }
    let mut message = unpack_bits(&bytes[1..], ft8::MESSAGE_BITS);
    if !ft8::crc_ok(&message) {
        return None;
    }
    // FT4 keys the payload through a fixed sequence, under the check.
    if mode == Mode::Ft4 {
        ft8::scramble_ft4(&mut message);
    }
    let m = ft8::unpack(&message)?;
    let mut fields = vec![("message".into(), common::Value::Text(m.text.clone()))];
    if let Some(to) = &m.to {
        fields.push(("to".into(), common::Value::Text(to.clone())));
    }
    if let Some(from) = &m.from {
        fields.push(("from".into(), common::Value::Text(from.clone())));
    }
    if let Some(grid) = &m.grid {
        fields.push(("grid".into(), common::Value::Text(grid.clone())));
    }
    if let Some(report) = m.report {
        fields.push(("report_db".into(), common::Value::Int(report as i64)));
    }
    let mut d = Decoded::bytes(mode.label(), center, 0.0, bytes[1..].to_vec())
        .with_modulation(match mode {
            Mode::Ft8 => common::Modulation::Fsk8,
            Mode::Ft4 => common::Modulation::Fsk4,
        })
        .with_crc(Some(true))
        .with_detail(m.text.clone())
        .with_fields(fields)
        // An operator's radio sent it on their behalf, to a station they
        // named: a call, a report and an acknowledgement are a conversation
        // however short the form is.
        .written()
        .with_text(m.text.clone());
    if let (Some(from), Some(to)) = (&m.from, &m.to) {
        d = d.with_link(pipeline::event::Link::between(
            pipeline::event::Party::unit(from.clone()),
            match to.as_str() {
                "CQ" | "QRZ" | "DE" => pipeline::event::Party::group(to.clone()),
                _ => pipeline::event::Party::unit(to.clone()),
            },
        ));
    }
    if let Some((lat, lon)) = m.grid.as_deref().and_then(ft8::grid_position) {
        d = d.at_position(common::Position {
            lat,
            lon,
            altitude_m: None,
            speed_kt: None,
            course_deg: None,
        });
    }
    Some(d)
}

/// Whether a frame off the bus is one of these: the mode byte a front end
/// wrote, the length, and the check the transmitter computed.
fn is_ftx(bytes: &[u8]) -> bool {
    bytes.len() == 1 + ft8::MESSAGE_BITS.div_ceil(8)
        && bytes.first().copied().and_then(Mode::of_tag).is_some()
        && ft8::crc_ok(&unpack_bits(&bytes[1..], ft8::MESSAGE_BITS))
}

/// The shape both modes declare, which differs only in the dials.
fn shape() -> Shape {
    Shape {
        widths: &[CHANNEL_WIDTH_HZ],
        min_rate_hz: 8_000.0,
        feed_rate_hz: AUDIO_HZ,
        span_wide: false,
        families: &[],
    }
}

pub struct Ft8;

impl Protocol for Ft8 {
    fn id(&self) -> &'static str {
        "ft8"
    }
    fn label(&self) -> &'static str {
        "ft8"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["wsjt", "jt"]
    }
    /// The dials stations are told to use. A station is not found anywhere
    /// else, because the whole mode depends on everybody being in the same
    /// 3 kHz.
    fn placement(&self) -> Placement {
        Placement::Channels(FT8_DIALS.to_vec())
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn shape(&self) -> Shape {
        shape()
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    fn reports_position(&self) -> bool {
        true
    }
    /// The dial is HF and the frame carries the mode byte its front end
    /// wrote, so the claim is by that tag rather than by where it was heard.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !is_ftx(bytes) || bytes[0] != Mode::Ft8.tag() {
            return None;
        }
        Some(ft8_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} FT8", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz: hz + PASSBAND_HZ / 2.0, width_hz: PASSBAND_HZ, label: "FT8".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(FT8_DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub struct Ft4;

impl Protocol for Ft4 {
    fn id(&self) -> &'static str {
        "ft4"
    }
    fn label(&self) -> &'static str {
        "ft4"
    }
    fn placement(&self) -> Placement {
        Placement::Channels(FT4_DIALS.to_vec())
    }
    fn default_hz(&self) -> f64 {
        FT4_DEFAULT_HZ
    }
    fn shape(&self) -> Shape {
        shape()
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !is_ftx(bytes) || bytes[0] != Mode::Ft4.tag() {
            return None;
        }
        Some(ft8_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} FT4", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz: hz + PASSBAND_HZ / 2.0, width_hz: PASSBAND_HZ, label: "FT4".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(FT4_DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

const CHANNEL_HZ: &str = "channel_hz";

pub const FT8_DESC: StageDesc = StageDesc {
    name: "ft8",
    summary: "One FT8 passband: every station in 3 kHz, on the fifteen-second clock",
    category: Category::Decode,
    feeds_bus: true,
};

pub const FT4_DESC: StageDesc = StageDesc {
    name: "ft4",
    summary: "One FT4 passband: every station in 3 kHz, on the seven-and-a-half-second clock",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build_ft8(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Ft8Node::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), Mode::Ft8)))
}

pub fn build_ft4(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Ft8Node::new(s.f64_or(CHANNEL_HZ, FT4_DEFAULT_HZ), Mode::Ft4)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};
    use std::f64::consts::TAU;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// One transmission keyed into a slot: the message packed, checked,
    /// coded and mapped to tones, then keyed at `audio_hz` above the dial
    /// starting `at_s` into the slot.
    fn keyed(
        mode: Mode,
        payload: [bool; ft8::PAYLOAD_BITS],
        rate: f64,
        audio_hz: f64,
        at_s: f64,
        amplitude: f32,
    ) -> Vec<C32> {
        let wf = mode.waveform();
        let mut payload = payload;
        if mode == Mode::Ft4 {
            ft8::scramble_ft4(&mut payload);
        }
        let word = ft8::encode(&payload);
        let mut tones = vec![0u8; wf.symbols];
        for group in wf.sync {
            tones[group.at..group.at + group.tones.len()].copy_from_slice(group.tones);
        }
        let per = wf.bits_per_symbol();
        let mut taken = 0usize;
        for (from, to) in wf.data {
            for slot in tones.iter_mut().take(*to).skip(*from) {
                let pattern =
                    (0..per).fold(0usize, |acc, k| acc << 1 | usize::from(word[taken + k]));
                taken += per;
                *slot = wf.gray[pattern];
            }
        }

        let mut out = vec![C32::default(); (rate * wf.slot_s) as usize];
        let symbol = (rate / wf.baud).round() as usize;
        let start = (at_s * rate) as usize;
        let mut phase = 0.0f64;
        for (s, tone) in tones.iter().enumerate() {
            let f = audio_hz + *tone as f64 * wf.baud;
            for k in 0..symbol {
                let at = start + s * symbol + k;
                if at >= out.len() {
                    break;
                }
                phase += TAU * f / rate;
                out[at] += amplitude * C32::new(phase.cos() as f32, phase.sin() as f32);
            }
        }
        out
    }

    /// Feed a stream through the node, from the slot boundary, and collect
    /// what reached the bus.
    fn run(node: &mut Ft8Node, iq: &[C32], rate: f64, center: f64) -> Vec<common::Frame> {
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in iq.chunks(4096) {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f);
            }
        }
        frames
    }

    /// A node reading from the start of the stream, which is what the wall
    /// clock does for it on the air.
    fn at_slot_start(dial: f64, mode: Mode) -> Ft8Node {
        let mut node = Ft8Node::new(dial, mode);
        node.negotiate(&spec(AUDIO_HZ, dial)).unwrap();
        node.skip = 0;
        node
    }

    /// The whole path: a station calling CQ, keyed 1 kHz up the passband,
    /// read back as the message it sent with the square it sent it from.
    #[test]
    fn a_cq_is_read_off_a_slot() {
        let payload = ft8::pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let iq = keyed(Mode::Ft8, payload, AUDIO_HZ, 1_000.0, 0.5, 1.0);
        let mut node = at_slot_start(DEFAULT_HZ, Mode::Ft8);
        let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);

        assert_eq!(frames.len(), 1, "{} transmissions", frames.len());
        assert_eq!(node.slots(), 1);
        let f = &frames[0];
        // The frame is heard where the station was: the dial plus where it
        // sat in the passband, to within a tone.
        assert!(f.center_hz.abs_diff((DEFAULT_HZ + 1_000.0) as u64) <= 7, "{}", f.center_hz);
        assert!(f.snr_db.is_finite() && f.rssi_dbfs.is_finite());
        assert!(f.iq.is_some(), "the samples it was read from");

        let d = ft8_decoded(&f.bytes, Hz(f.center_hz)).expect("a decode");
        assert_eq!(d.protocol, "FT8");
        assert_eq!(d.text.as_deref(), Some("CQ MI0ABC IO74"));
        assert_eq!(d.crc_ok, Some(true));
        assert!(d.written, "an operator's station called another");
        assert_eq!(d.field("from").map(|v| v.to_string()).as_deref(), Some("MI0ABC"));
        assert_eq!(d.field("grid").map(|v| v.to_string()).as_deref(), Some("IO74"));
        let p = d.position.expect("the square it sent");
        assert!((p.lat - 54.5).abs() < 1e-6 && (p.lon - -5.0).abs() < 1e-6);
    }

    /// The point of the mode: a passband holds a whole conversation's worth
    /// of stations at once, and one pass over the slot reads all of them.
    #[test]
    fn eight_stations_in_one_passband_are_all_read() {
        let sent = [
            ("CQ", "MI0ABC", "IO74"),
            ("MI0ABC", "G4XYZ", "IO91"),
            ("G4XYZ", "MI0ABC", "-12"),
            ("MI0ABC", "G4XYZ", "R-08"),
            ("G4XYZ", "MI0ABC", "RRR"),
            ("CQ", "EI7DEF", "IO53"),
            ("EI7DEF", "DL1GHI", "JO31"),
            ("DL1GHI", "EI7DEF", "73"),
        ];
        let mut iq = vec![C32::default(); (AUDIO_HZ * mfsk::FT8.slot_s) as usize];
        for (k, (to, from, extra)) in sent.iter().enumerate() {
            let payload = ft8::pack_standard(to, from, extra).expect(from);
            // Spread across the passband, each starting at its own moment
            // within the two seconds a station may be late by.
            let hz = 400.0 + 300.0 * k as f64;
            let at = 0.2 + 0.15 * (k % 4) as f64;
            for (a, b) in iq.iter_mut().zip(keyed(Mode::Ft8, payload, AUDIO_HZ, hz, at, 0.5)) {
                *a += b;
            }
        }
        let mut node = at_slot_start(DEFAULT_HZ, Mode::Ft8);
        let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);
        assert_eq!(frames.len(), 8, "{} of 8 stations read", frames.len());
        assert_eq!(node.read_count(), 8);

        let mut read: Vec<String> = frames
            .iter()
            .map(|f| ft8_decoded(&f.bytes, Hz(f.center_hz)).expect("a decode").text.unwrap())
            .collect();
        read.sort();
        let mut want: Vec<String> = sent.iter().map(|(a, b, c)| format!("{a} {b} {c}")).collect();
        want.sort();
        assert_eq!(read, want);
    }

    /// FT4 is the same slot read at four tones on a seven-and-a-half-second
    /// clock, with the payload keyed through its scrambling sequence.
    #[test]
    fn an_ft4_exchange_is_read() {
        let payload = ft8::pack_standard("G4XYZ", "MI0ABC", "R+05").unwrap();
        let iq = keyed(Mode::Ft4, payload, AUDIO_HZ, 1_500.0, 0.4, 1.0);
        let mut node = at_slot_start(FT4_DEFAULT_HZ, Mode::Ft4);
        let frames = run(&mut node, &iq, AUDIO_HZ, FT4_DEFAULT_HZ);
        assert_eq!(frames.len(), 1, "{} transmissions", frames.len());
        let d = ft8_decoded(&frames[0].bytes, Hz(frames[0].center_hz)).expect("a decode");
        assert_eq!(d.protocol, "FT4");
        assert_eq!(d.text.as_deref(), Some("G4XYZ MI0ABC R+05"));
        assert_eq!(d.field("report_db").and_then(|v| v.as_i64()), Some(5));
        assert_eq!(d.position, None, "a report says nothing about where");
    }

    /// A station keyed into noise 10 dB below it, which is what a quiet
    /// band looks like, and the same station buried in noise that is
    /// louder than it, which is what the mode is for and what this
    /// receiver does not yet reach.
    #[test]
    fn a_station_reads_through_noise_until_it_does_not() {
        let payload = ft8::pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        // A tone against noise filling the passband, by amplitude. What
        // the decoder itself reports for each is in the assertion: it reads
        // a station down to about -15 dB in 2500 Hz and loses it by -18,
        // where WSJT-X is still reading to around -21.
        for (amplitude, want, snr) in [(0.10f32, 1, -10.0), (0.05, 1, -15.2), (0.035, 0, 0.0)] {
            let mut iq = keyed(Mode::Ft8, payload, AUDIO_HZ, 1_000.0, 0.5, amplitude);
            for s in iq.iter_mut() {
                *s += C32::new(rng(), rng());
            }
            let mut node = at_slot_start(DEFAULT_HZ, Mode::Ft8);
            let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);
            assert_eq!(frames.len(), want, "at an amplitude of {amplitude}");
            if let Some(f) = frames.first() {
                assert!((f.snr_db - snr).abs() < 1.0, "{} dB at {amplitude}", f.snr_db);
            }
            for f in &frames {
                let d = ft8_decoded(&f.bytes, Hz(f.center_hz)).expect("a decode");
                assert_eq!(d.text.as_deref(), Some("CQ MI0ABC IO74"), "wrong text at {amplitude}");
            }
        }
    }

    /// A minute of noise, which is four slots, and nothing comes off it:
    /// the code has to converge and the CRC-14 has to pass, and noise does
    /// neither.
    #[test]
    fn noise_produces_no_transmissions() {
        let mut seed = 0x0123_4567_89ab_cdefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> =
            (0..(AUDIO_HZ * 60.0) as usize).map(|_| C32::new(rng(), rng())).collect();
        let mut node = at_slot_start(DEFAULT_HZ, Mode::Ft8);
        let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);
        assert_eq!(node.slots(), 4, "four slots of noise");
        assert_eq!(frames.len(), 0, "{} transmissions out of noise", frames.len());
    }

    /// A frame off the bus is claimed by the mode that keyed it and by
    /// nothing else: the tag byte says which, and the check refuses a frame
    /// that is neither.
    #[test]
    fn a_frame_is_claimed_by_the_mode_that_wrote_it() {
        let payload = ft8::pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let message = ft8::with_crc(&payload);
        let mut bytes = vec![8u8];
        bytes.extend(pack(&message));
        assert!(is_ftx(&bytes));
        let p = common::Packet::of_frame(
            0,
            PASSBAND_HZ as u32,
            common::Frame::measured(bytes.clone(), -40.0, 12.0).at(DEFAULT_HZ as u64),
        );
        assert_eq!(Ft8.read_frame(&p, &bytes).map(|r| r.len()), Some(1));
        assert_eq!(Ft4.read_frame(&p, &bytes), None, "the tag says FT8");

        bytes[0] = 4;
        assert!(is_ftx(&bytes), "the same message keyed as FT4 checks too");
        assert_eq!(Ft8.read_frame(&p, &bytes), None);

        bytes[0] = 2;
        assert!(!is_ftx(&bytes), "no mode keys that");
        bytes[0] = 8;
        bytes[5] ^= 1;
        assert!(!is_ftx(&bytes), "a wrong bit fails the check");
        assert!(!is_ftx(&bytes[..8]), "too short to be a message");
    }

    #[test]
    fn the_node_refuses_a_span_without_its_passband() {
        let mut n = Ft8Node::default();
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ + 30_000.0)).is_err());
        // 8 kHz complex still holds a 3 kHz passband above the dial.
        assert!(n.negotiate(&spec(8_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(4_000.0, DEFAULT_HZ)).is_err(), "too narrow for 3 kHz");
    }
}
