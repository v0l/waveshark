//! RTTY as a graph node: a channel of the span in, a run of Baudot codes out.
//!
//! The signal is keyed FSK on the carrier itself rather than tones inside an
//! FM channel, so there is no discriminator in this path at all: the node
//! mixes the channel down, narrows it, and hands the baseband to
//! [`dsp::fsk::TonePair`], which decides mark against space one symbol at a
//! time. Above that is an asynchronous line older than any of it, a start
//! bit, five data bits and a stop, and above that the Baudot tables in
//! [`decode::rtty`].
//!
//! # Which way up
//!
//! Nothing on the air says whether the higher tone is the mark. Sidebands
//! get inverted along the way, so the same station is upright to one
//! listener and upside down to another, and an operator should not have to
//! know which. Both readings are framed at once, and what closes a run is
//! the tones falling undecided rather than either reading's idea of idle.
//! The one that framed more characters wins, and if neither framed enough of
//! them that printed, nothing is published: RTTY carries no check at all, so
//! the framing and the tables are the only evidence there is.
//!
//! The channel is where it is tuned and the node does not search for the
//! station. Measured on a keyed 170 Hz shift at 45.45 baud, an over reads
//! whole up to 25 Hz off channel and loses characters beyond that.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::rtty::{self, Shift, Speed};
use dsp::afsk::Symbol;
use dsp::fsk::TonePair;
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The 20 m RTTY sub-band, which is where a station is most likely to be
/// found at any hour.
pub const DEFAULT_HZ: f64 = 14_083_000.0;

/// The channel an RTTY station occupies. The widest shift in use is 850 Hz,
/// and the 250 Hz an amateur station takes sits well inside that.
pub const CHANNEL_WIDTH_HZ: f64 = 1_000.0;

/// Rate the channel is decimated to before the tone pair reads it. Five
/// times the widest shift, so the correlators have room and every speed has
/// far more than the four samples a symbol they need.
const AUDIO_HZ: f64 = 8_000.0;

/// Symbol times with no character framed either way up before a run is
/// closed and published. A stop element is at most two symbols and the next
/// character follows it, so this is the line resting rather than a gap
/// inside an over: about half a second at 45 baud.
const IDLE_SYMBOLS: usize = 24;

/// Undecided symbols in a row that close a run. One is a fade or a symbol
/// the correlators straddled; a pair is the station having stopped.
const QUIET_SYMBOLS: usize = 2;

/// Characters a run must hold before it is worth publishing. Below this a
/// run is as likely to be noise framed by luck as anything anybody sent.
const MIN_CHARS: usize = 6;

/// How much of a run has to be a character the tables have. Letters case
/// has one for every code but zero, so this is a low bar by itself and the
/// framing is what does the real refusing.
const MIN_PRINTABLE: f32 = 0.9;

/// The longest run held before it is forced out, in characters.
const MAX_CHARS: usize = 4_096;

pub struct RttyNode {
    channel_hz: f64,
    speed: Speed,
    shift: Shift,
    /// The stream as negotiated: its rate and how far it is decimated.
    rate: f64,
    factor: usize,
    mixer: Mixer,
    decim: FirDecim,
    tones: TonePair,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    symbols: Vec<Symbol>,
    line: Line,
    meter: crate::FrameMeter,
    runs: u64,
}

impl Default for RttyNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, Speed::default(), Shift::default())
    }
}

impl RttyNode {
    pub fn new(channel_hz: f64, speed: Speed, shift: Shift) -> Self {
        Self {
            channel_hz,
            speed,
            shift,
            // All replaced at negotiation, when the real rate is known.
            rate: AUDIO_HZ,
            factor: 1,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            tones: TonePair::new(AUDIO_HZ, speed.baud(), shift.hz()),
            mixed: Vec::new(),
            narrow: Vec::new(),
            symbols: Vec::new(),
            line: Line::default(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            runs: 0,
        }
    }

    /// Runs published since the node was built.
    pub fn runs(&self) -> u64 {
        self.runs
    }

    /// How wide the channel filter is kept: the shift plus room for the
    /// sidebands the keying puts either side of each tone.
    fn passband_hz(&self) -> f64 {
        self.shift.hz() / 2.0 + 2.0 * self.speed.baud()
    }

    fn rebuild(&mut self) {
        let (rate, factor) = (self.rate, self.factor);
        let audio_rate = rate / factor as f64;
        self.decim = FirDecim::design_hz(rate, factor, self.passband_hz(), 60.0);
        self.tones = TonePair::new(audio_rate, self.speed.baud(), self.shift.hz());
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0);
    }
}

/// The asynchronous line, read both ways up at once.
///
/// One framer per polarity, and a run closes on the channel falling quiet or
/// on neither framer having read a character for a while, so the closing
/// does not depend on knowing which way up the station is.
#[derive(Default)]
struct Line {
    upright: Uart,
    inverted: Uart,
    since_char: usize,
    undecided: usize,
}

impl Line {
    /// Feed one symbol, and hand back a run of codes where it ended one.
    fn push(&mut self, sym: Symbol) -> Option<Vec<u8>> {
        let flipped = Symbol { mark: !sym.mark, quiet: sym.quiet };
        let a = self.upright.push(sym);
        let b = self.inverted.push(flipped);
        self.since_char = match a || b {
            true => 0,
            false => self.since_char + 1,
        };
        self.undecided = match sym.quiet {
            true => self.undecided + 1,
            false => 0,
        };
        let resting = self.undecided >= QUIET_SYMBOLS || self.since_char >= IDLE_SYMBOLS;
        if !resting {
            return match self.upright.codes.len().max(self.inverted.codes.len()) >= MAX_CHARS {
                true => self.take(),
                false => None,
            };
        }
        self.take()
    }

    /// Close whatever is open, and publish the better reading of it.
    fn take(&mut self) -> Option<Vec<u8>> {
        let up = std::mem::take(&mut self.upright);
        self.undecided = 0;
        let down = std::mem::take(&mut self.inverted);
        self.since_char = 0;
        // More characters framed is the first evidence, because a station
        // read upside down loses its stop bits and frames almost nothing.
        // Where both framed the same count, the tables decide.
        let best = match (up.codes.len(), down.codes.len()) {
            (a, b) if a > b => up.codes,
            (a, b) if b > a => down.codes,
            _ if rtty::printable(&down.codes) > rtty::printable(&up.codes) => down.codes,
            _ => up.codes,
        };
        if best.len() < MIN_CHARS || rtty::printable(&best) < MIN_PRINTABLE {
            return None;
        }
        Some(best)
    }
}

/// A start bit, five data bits with the first on the air least significant,
/// and a stop element of mark.
///
/// The stop is read as one symbol however long the station holds it: a
/// teleprinter's one and a half or a machine's two are both mark, and the
/// next start bit is a falling edge the tone pair's clock resynchronises on.
#[derive(Default)]
struct Uart {
    partial: Option<(u8, u32)>,
    codes: Vec<u8>,
}

impl Uart {
    /// Feed one symbol, returning whether it completed a character.
    fn push(&mut self, sym: Symbol) -> bool {
        if sym.quiet {
            self.partial = None;
            return false;
        }
        match &mut self.partial {
            // A mark between characters is the line resting.
            None if sym.mark => false,
            // A space between characters is a start bit.
            None => {
                self.partial = Some((0, 0));
                false
            }
            Some((code, have)) => {
                if *have < 5 {
                    *code |= u8::from(sym.mark) << *have;
                    *have += 1;
                    return false;
                }
                // The stop element. A space here is a framing slip, and the
                // character it would have made is not a character.
                let (code, ok) = (*code, sym.mark);
                self.partial = None;
                if ok && self.codes.len() < MAX_CHARS {
                    self.codes.push(code);
                }
                ok
            }
        }
    }
}

impl Simple for RttyNode {
    fn name(&self) -> &str {
        "rtty"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("rtty reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("rtty needs its channel inside the span"));
        }
        self.rate = rate;
        self.factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.rebuild();
        if !self.tones.usable() {
            return Err(common::Error::other("rtty needs four samples a symbol"));
        }

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);

        let mut symbols = std::mem::take(&mut self.symbols);
        symbols.clear();
        self.tones.process(&self.narrow, &mut symbols);
        for sym in &symbols {
            if let Some(run) = self.line.push(*sym) {
                self.runs += 1;
                o.frames_mut().push(self.meter.frame(run));
            }
        }
        self.symbols = symbols;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.tones.reset();
        self.line = Line::default();
        self.meter.reset();
    }

    fn params(&self) -> Vec<Param> {
        let speed = Speed::ALL.iter().position(|s| *s == self.speed).unwrap_or(0);
        let shift = Shift::ALL.iter().position(|s| *s == self.shift).unwrap_or(0);
        vec![
            Param::float(CHANNEL_HZ, self.channel_hz, 1e5..=1e9).unit("Hz").label("Channel"),
            Param::choice(SPEED, speed, Speed::ALL.iter().map(|s| s.to_string()).collect())
                .unit("baud")
                .label("Speed"),
            Param::choice(SHIFT, shift, Shift::ALL.iter().map(|s| s.to_string()).collect())
                .unit("Hz")
                .label("Shift"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => self.channel_hz = v.as_f64().unwrap_or(self.channel_hz),
            SPEED => {
                let k = v.as_i64().unwrap_or(0).clamp(0, Speed::ALL.len() as i64 - 1) as usize;
                self.speed = Speed::ALL[k];
            }
            SHIFT => {
                let k = v.as_i64().unwrap_or(0).clamp(0, Shift::ALL.len() as i64 - 1) as usize;
                self.shift = Shift::ALL[k];
            }
            _ => return Err(common::Error::other(format!("rtty: unknown parameter {name:?}"))),
        }
        self.rebuild();
        Ok(())
    }
}

/// What a run of codes off the bus becomes: one row, the text a teleprinter
/// would have printed.
pub fn rtty_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    let text = rtty::text(bytes);
    if text.trim().is_empty() {
        return None;
    }
    let fields = vec![
        ("characters".into(), common::Value::Int(bytes.len() as i64)),
        ("message".into(), common::Value::Text(text.clone())),
    ];
    Some(
        Decoded::bytes("RTTY", center, 0.0, bytes.to_vec())
            .with_modulation(common::Modulation::Fsk2)
            .with_detail(format!("{} characters", bytes.len()))
            .with_fields(fields)
            // An operator typed it and sent it to whoever was listening, so
            // it belongs in the message view beside anything else somebody
            // wrote, rather than in the packet list with the machines.
            .written()
            .with_text(text),
    )
}

/// Whether a frame off the bus could be a run of Baudot codes. Five bits is
/// all a code has, so anything with a byte above 31 in it was framed by
/// something else.
fn is_baudot(bytes: &[u8]) -> bool {
    bytes.len() >= MIN_CHARS
        && bytes.iter().all(|b| *b < 32)
        && rtty::printable(bytes) >= MIN_PRINTABLE
}

pub struct Rtty;

/// An over, keyed as pulse timings.
///
/// The mirror of [`RttyNode`]: `decode::rtty::encode` picks the codes and the
/// shift characters between them, `decode::rtty::line` adds the start and
/// stop elements, and `dsp::pulse::nrz` turns those into the timings
/// [`crate::mod_nodes::FskModNode`] keys. A mark is the upper tone, which is
/// upright for a station on the upper sideband; the receiver reads both ways
/// up regardless.
///
/// The line is keyed at twice the baud because a stop element is one and a
/// half bit times and whole bits cannot express that.
pub struct RttyTxNode {
    text: String,
    speed: Speed,
    shift: Shift,
    stop: rtty::Stop,
    rate: f64,
    pace: crate::tx_source::Pace,
}

impl Default for RttyTxNode {
    fn default() -> Self {
        Self {
            text: String::new(),
            speed: Speed::default(),
            shift: Shift::default(),
            stop: rtty::Stop::default(),
            rate: 0.0,
            pace: crate::tx_source::Pace::default(),
        }
    }
}

/// Elements keyed per bit time. Two, so a stop of one and a half bits is
/// three of them.
const ELEMENTS_PER_BIT: usize = 2;

impl RttyTxNode {
    pub fn new(text: &str, speed: Speed, shift: Shift) -> Self {
        Self { text: text.into(), speed, shift, ..Default::default() }
    }

    /// The whole over as timings, idle marks and all.
    pub fn over(&self) -> Vec<common::pulse::Pulse> {
        let codes = rtty::encode(&self.text);
        let bits = rtty::line(&codes, self.stop, ELEMENTS_PER_BIT);
        dsp::pulse::nrz(&bits, self.speed.baud() * ELEMENTS_PER_BIT as f64)
    }

    pub fn sent(&self) -> u64 {
        self.pace.sent()
    }
}

impl Simple for RttyTxNode {
    fn name(&self) -> &str {
        RTTY_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            ("keying".into(), format!("{} baud, {} Hz", self.speed, self.shift)),
            ("overs".into(), self.pace.sent().to_string()),
        ]
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("rtty_tx needs a clock to key against"));
        }
        self.rate = input.spec.rate;
        let mut out = input.spec.with_kind(PortKind::Pulses);
        out.flow = pipeline::port::Flow::Tx;
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.pace.clock(i.len(), self.rate);
        if !self.pace.due() || self.text.is_empty() {
            return Ok(());
        }
        let pulses = self.over();
        if pulses.is_empty() {
            return Ok(());
        }
        self.pace.spent(crate::tx_source::air_time_us(&pulses));
        o.pulses_mut().push(common::pulse::Package { pulses, ..Default::default() });
        Ok(())
    }

    fn reset(&mut self) {
        self.pace.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::text(TEXT, self.text.clone()).label("Over"),
            Param::choice(
                SPEED,
                Speed::ALL.iter().position(|s| *s == self.speed).unwrap_or(0),
                Speed::ALL.iter().map(|s| s.to_string()).collect(),
            )
            .label("Speed")
            .unit("baud"),
            Param::choice(
                SHIFT,
                Shift::ALL.iter().position(|s| *s == self.shift).unwrap_or(0),
                Shift::ALL.iter().map(|s| s.to_string()).collect(),
            )
            .label("Shift")
            .unit("Hz"),
            Param::float(PAUSE_MS, self.pace.pause_ms(), 0.0..=60_000.0)
                .label("Between overs")
                .unit("ms"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            TEXT => {
                self.text = match value {
                    ParamValue::Text(t) => t,
                    _ => return Err(common::Error::other("rtty_tx: an over is text")),
                }
            }
            SPEED => self.speed = speed_of(&value).unwrap_or_default(),
            SHIFT => self.shift = shift_of(&value).unwrap_or_default(),
            PAUSE_MS => self.pace.set_pause_ms(value.as_f64().unwrap_or(1_000.0)),
            _ => return Err(common::Error::other(format!("rtty_tx: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

impl Protocol for Rtty {
    fn id(&self) -> &'static str {
        "rtty"
    }
    fn label(&self) -> &'static str {
        "rtty"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["baudot", "teleprinter"]
    }
    /// Amateur HF, the utility and weather circuits, and a few VHF links:
    /// a teleprinter is wherever somebody put one.
    fn placement(&self) -> Placement {
        Placement::Anywhere
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 2.0 * CHANNEL_WIDTH_HZ,
            feed_rate_hz: AUDIO_HZ,
            span_wide: false,
            families: &[],
        }
    }
    /// A run of five-bit codes is a shape no other decoder on the bus
    /// produces, but nothing in RTTY checks, so the claim is HF-wide and
    /// late rather than specific.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 30_000_000 }
    }
    fn read_frame(&self, _p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !is_baudot(bytes) {
            return None;
        }
        Some(rtty_decoded(bytes, common::Hz(_p.center_hz())).into_iter().collect())
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} RTTY", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "RTTY".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// The teleprinter into the two-tone modulator, at the shift the
    /// receiver defaults to reading.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(RTTY_TX.name),
            modulator: NodeSpec::new(crate::mod_nodes::FSK_MOD.name)
                .f("shift_hz", Shift::default().hz()),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A station keying `text`: idle marks, then a start bit, five data
    /// bits with the first on the air least significant, and a stop element
    /// of one and a half bit times, which is what a teleprinter sends and
    /// what the framer has to read without being told.
    fn keyed(text: &str, rate: f64, offset_hz: f64, invert: bool) -> Vec<C32> {
        let (baud, shift) = (Speed::default().baud(), Shift::default().hz());
        let mut elements: Vec<(bool, f64)> = vec![(true, 40.0)];
        for code in rtty::encode(text) {
            elements.push((false, 1.0));
            elements.extend((0..5).map(|k| (code >> k & 1 != 0, 1.0)));
            elements.push((true, rtty::Stop::OneAndHalf.bits()));
        }
        elements.push((true, 40.0));

        let mut out = Vec::new();
        let mut phase = 0.0f64;
        let mut elapsed = 0.0f64;
        for (mark, bits) in elements {
            elapsed += bits;
            let mark = mark != invert;
            let f = offset_hz + if mark { shift / 2.0 } else { -shift / 2.0 };
            while (out.len() as f64) < elapsed * rate / baud {
                phase += std::f64::consts::TAU * f / rate;
                out.push(C32::new(0.5 * phase.cos() as f32, 0.5 * phase.sin() as f32));
            }
        }
        out
    }

    fn run(node: &mut RttyNode, iq: &[C32], rate: f64, center: f64) -> Vec<Vec<u8>> {
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
                frames.extend(f.into_iter().map(|x| x.bytes));
            }
        }
        frames
    }

    const OVER: &str = "CQ CQ DE MI0ABC MI0ABC K";

    /// The whole path on synthetic RF: a keyed carrier off centre in the
    /// span, into the node, out as the text that was typed.
    #[test]
    fn a_keyed_over_is_read_back_as_text() {
        let (rate, center) = (48_000.0, 14_080_000.0);
        let channel = center + 3_000.0;
        let iq = keyed(OVER, rate, 0.0, false);
        // The station sits 3 kHz up the span, which the node has to mix
        // away before any of it is a tone.
        let mut ph = 0.0f64;
        let iq: Vec<C32> = iq
            .iter()
            .map(|s| {
                ph += std::f64::consts::TAU * 3_000.0 / rate;
                s * C32::new(ph.cos() as f32, ph.sin() as f32)
            })
            .collect();

        let mut node = RttyNode::new(channel, Speed::default(), Shift::default());
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);

        assert_eq!(frames.len(), 1, "{} runs off the air", frames.len());
        let d = rtty_decoded(&frames[0], Hz(channel as u64)).expect("a decode");
        assert_eq!(d.text.as_deref(), Some(OVER));
        assert_eq!(d.protocol, "RTTY");
        assert!(d.written, "an operator typed it");
        // The text is 24 characters and the five-bit code needs four shifts
        // to reach the digits in the two calls and back again.
        assert_eq!(d.field("characters").and_then(|v| v.as_i64()), Some(29));
        assert_eq!(d.crc_ok, None, "nothing in RTTY checks");
    }

    /// Keyed by the transmit stage, modulated, and read back: the encoder,
    /// the line discipline and the framer all agree or the text does not
    /// come back.
    #[test]
    fn an_over_this_receiver_keyed_is_an_over_this_receiver_reads() {
        let (rate, center) = (8_000.0, DEFAULT_HZ);
        let mut tx = RttyTxNode::new(OVER, Speed::default(), Shift::default());
        let keyed = tx.negotiate(&spec(rate, center)).unwrap();

        let mut pulses = Payload::empty_of(PortKind::Pulses);
        let ins = [spec(rate, center)];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(&mut tx, &Payload::Real(vec![0.0; 4096]), &mut pulses, &mut ctx).unwrap();
        assert_eq!(tx.sent(), 1);
        // 29 codes of seven and a half bit times, and eight idle bits
        // either side, at 45.45 baud.
        let air = crate::tx_source::air_time_us(&pulses.as_pulses().unwrap()[0].pulses) / 1e6;
        assert!((air - (29.0 * 7.5 + 16.0) / 45.45).abs() < 1e-3, "{air} s of air");

        let mut modulator = crate::mod_nodes::FskModNode::new(0.0, Shift::default().hz(), 0.5);
        modulator.negotiate(&PortSpec { spec: keyed, latency: 0 }).unwrap();
        let mut iq = Payload::Iq(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(&mut modulator, &pulses, &mut iq, &mut ctx).unwrap();
        let mut iq = match iq {
            Payload::Iq(v) => v,
            _ => unreachable!("a modulator produces baseband"),
        };
        // The carrier drops between overs, and that silence is what closes
        // the run: an over has no end of message and the framer publishes
        // when the tones fall undecided.
        iq.extend(std::iter::repeat_n(C32::new(0.0, 0.0), rate as usize / 2));

        let mut node = RttyNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 1, "{} runs off the air", frames.len());
        let d = rtty_decoded(&frames[0], Hz(center as u64)).expect("a decode");
        assert_eq!(d.text.as_deref(), Some(OVER));
        assert_eq!(d.field("characters").and_then(|v| v.as_i64()), Some(29));
    }

    /// The same over with mark and space the other way about, which is what
    /// a sideband inversion anywhere along the path produces. The operator
    /// is not asked which way up the station is.
    #[test]
    fn an_inverted_station_reads_the_same() {
        let (rate, center) = (8_000.0, 14_083_000.0);
        for invert in [false, true] {
            let iq = keyed(OVER, rate, 0.0, invert);
            let mut node = RttyNode::default();
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            assert_eq!(frames.len(), 1, "inverted={invert}: {} runs", frames.len());
            let d = rtty_decoded(&frames[0], Hz(center as u64)).expect("a decode");
            assert_eq!(d.text.as_deref(), Some(OVER), "inverted={invert}");
        }
    }

    /// How far an operator may leave a station mistuned, measured: a
    /// 170 Hz shift at 45.45 baud reads whole up to 25 Hz off and loses
    /// characters beyond that, because the correlator that reads a tone is
    /// only as wide as two cycles of the shift. Half a shift out reads
    /// nothing rather than reading something else.
    #[test]
    fn a_mistuned_station_still_reads() {
        let (rate, center) = (8_000.0, 14_083_000.0);
        let near = keyed(OVER, rate, 25.0, false);
        let mut node = RttyNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &near, rate, center);
        assert_eq!(frames.len(), 1);
        assert_eq!(rtty::text(&frames[0]), OVER);

        let far = keyed(OVER, rate, 85.0, false);
        let mut node = RttyNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        for f in run(&mut node, &far, rate, center) {
            assert_ne!(rtty::text(&f), OVER, "a station half a shift off read anyway");
        }
    }

    /// Twenty seconds of noise produces nothing. The line will frame
    /// characters out of anything; the run length and the tables are what
    /// refuse them, and there is no check behind either.
    #[test]
    fn noise_produces_no_runs() {
        let (rate, center) = (8_000.0, 14_083_000.0);
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> = (0..rate as usize * 20).map(|_| C32::new(rng(), rng())).collect();
        let mut node = RttyNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 0, "{} runs out of twenty seconds of noise", frames.len());
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = RttyNode::default();
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ + 30_000.0)).is_err());
        assert!(n.negotiate(&spec(2_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(500.0, DEFAULT_HZ)).is_err());
    }

    /// A run of five-bit codes is what reaches the bus, and a frame with a
    /// byte no Baudot code can hold belongs to another decoder.
    #[test]
    fn only_five_bit_codes_are_claimed() {
        assert!(is_baudot(&rtty::encode("TEST DE MI0ABC")));
        assert!(!is_baudot(&rtty::encode("OM")), "two characters is not a run");
        assert!(!is_baudot(&[0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47]));
        assert!(!is_baudot(&[0; 16]), "an unassigned code is not a character");
    }
}

/// The carrier this stage is pointed at, and how it is keyed.
const CHANNEL_HZ: &str = "channel_hz";
const SPEED: &str = "speed";
const SHIFT: &str = "shift";
const TEXT: &str = "text";
const PAUSE_MS: &str = "pause_ms";

pub const DESC: StageDesc = StageDesc {
    name: "rtty",
    summary: "One RTTY channel: Baudot at 45.45 to 200 baud, 170 to 850 Hz shift",
    category: Category::Decode,
    feeds_bus: true,
};

pub const RTTY_TX: StageDesc = StageDesc {
    name: "rtty_tx",
    summary: "Key an over in Baudot, start and stop elements and all",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let speed = s.get(SPEED).and_then(speed_of).unwrap_or_default();
    let shift = s.get(SHIFT).and_then(shift_of).unwrap_or_default();
    Ok(Box::new(RttyNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), speed, shift)))
}

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let speed = s.get(SPEED).and_then(speed_of).unwrap_or_default();
    let shift = s.get(SHIFT).and_then(shift_of).unwrap_or_default();
    let mut n = RttyTxNode::new(s.str_or(TEXT, ""), speed, shift);
    n.pace.set_pause_ms(s.f64_or(PAUSE_MS, crate::tx_source::DEFAULT_PAUSE_MS));
    Ok(Box::new(n))
}

/// A saved patch holds the choice as an index and a person writing one holds
/// it as a number of baud, so both are read here and parsed once.
fn speed_of(v: &ParamValue) -> Option<Speed> {
    match v {
        ParamValue::Choice(k) => Speed::ALL.get(*k).copied(),
        ParamValue::Text(s) => s.parse().ok(),
        other => other.as_f64().map(|b| b.to_string()).and_then(|s| s.parse().ok()),
    }
}

fn shift_of(v: &ParamValue) -> Option<Shift> {
    match v {
        ParamValue::Choice(k) => Shift::ALL.get(*k).copied(),
        ParamValue::Text(s) => s.parse().ok(),
        other => other.as_f64().map(|h| h.to_string()).and_then(|s| s.parse().ok()),
    }
}
