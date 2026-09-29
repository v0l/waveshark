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
pub use decode::rtty::read;
use decode::rtty::{self, Shift, Speed};
pub use decode::rtty::{IDLE_SYMBOLS, MAX_CHARS, MIN_CHARS, MIN_PRINTABLE, QUIET_SYMBOLS};
use dsp::afsk::Symbol;
use dsp::fsk::{TonePair, TonePairFinder};
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::rtty::AUDIO_HZ;
pub use identify::rtty::CHANNEL_WIDTH_HZ;
pub use identify::rtty::DEFAULT_HZ;
pub use identify::rtty::Rtty;
use identify::rtty::{Score, best};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct RttyNode {
    channel_hz: f64,
    speed: Option<Speed>,
    shift: Option<Shift>,
    rate: f64,
    factor: usize,
    mixer: Mixer,
    decim: FirDecim,
    readers: Vec<Reader>,
    finder: Option<TonePairFinder>,
    waiting: Vec<common::C32>,
    retune: Mixer,
    offset_hz: f64,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    tuned: Vec<common::C32>,
    symbols: Vec<Symbol>,
    meter: crate::FrameMeter,
    runs: u64,
}

struct Reader {
    speed: Speed,
    shift: Shift,
    tones: TonePair,
    line: rtty::Framer,
    held: Vec<Vec<u8>>,
}

impl Default for RttyNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, Some(Speed::default()), Some(Shift::default()))
    }
}

impl RttyNode {
    pub fn new(channel_hz: f64, speed: Option<Speed>, shift: Option<Shift>) -> Self {
        let mut n = Self {
            channel_hz,
            speed,
            shift,
            rate: AUDIO_HZ,
            factor: 1,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            readers: Vec::new(),
            finder: None,
            waiting: Vec::new(),
            retune: Mixer::new(0.0, 1.0),
            offset_hz: 0.0,
            mixed: Vec::new(),
            narrow: Vec::new(),
            tuned: Vec::new(),
            symbols: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0)
                .keyed_as(common::Modulation::Fsk2),
            runs: 0,
        };
        n.rebuild();
        n
    }

    pub fn any(channel_hz: f64) -> Self {
        Self::new(channel_hz, None, None)
    }

    pub fn runs(&self) -> u64 {
        self.runs
    }

    pub fn reading(&self) -> Option<(Speed, Shift)> {
        match self.readers.as_slice() {
            [one] => Some((one.speed, one.shift)),
            _ => None,
        }
    }

    fn speeds(&self) -> Vec<Speed> {
        self.speed.map_or(Speed::ALL.to_vec(), |s| vec![s])
    }

    fn shifts(&self) -> Vec<Shift> {
        self.shift.map_or(Shift::ALL.to_vec(), |s| vec![s])
    }

    pub fn offset_hz(&self) -> f64 {
        self.offset_hz
    }

    fn passband_hz(&self) -> f64 {
        let widest = self.shifts().iter().map(|s| s.hz()).fold(0.0, f64::max);
        let fastest = self.speeds().iter().map(|s| s.baud()).fold(0.0, f64::max);
        widest / 2.0 + 2.0 * fastest + REACH_HZ
    }

    fn rebuild(&mut self) {
        let (rate, factor) = (self.rate, self.factor);
        let audio_rate = rate / factor as f64;
        self.decim = FirDecim::design_hz(rate, factor, self.passband_hz(), 60.0);
        self.readers = self
            .speeds()
            .into_iter()
            .flat_map(|speed| self.shifts().into_iter().map(move |shift| (speed, shift)))
            .map(|(speed, shift)| Reader {
                speed,
                shift,
                tones: TonePair::new(audio_rate, speed.baud(), shift.hz()),
                line: rtty::Framer::new(),
                held: Vec::new(),
            })
            .filter(|r| r.tones.usable())
            .collect();
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0)
            .keyed_as(common::Modulation::Fsk2);
        self.finder = Some(TonePairFinder::new(audio_rate, FIND_RESOLUTION_HZ));
        self.waiting.clear();
        self.offset_hz = 0.0;
        self.retune = Mixer::new(0.0, audio_rate);
    }

    fn tune(&mut self) -> bool {
        let shifts: Vec<f64> = self.shifts().iter().map(|s| s.hz()).collect();
        let Some(f) = self.finder.as_mut() else { return true };
        f.push(&self.narrow);
        self.waiting.extend_from_slice(&self.narrow);
        let audio_rate = self.rate / self.factor as f64;
        let keep = (2.0 * FIND_S * audio_rate) as usize;
        if self.waiting.len() > keep {
            let drop = self.waiting.len() - keep;
            self.waiting.drain(..drop);
        }
        if f.seconds() < FIND_S {
            return false;
        }
        let Some((offset, _)) = f.find(&shifts, REACH_HZ, FIND_OVER_DB) else {
            if f.seconds() >= FIND_GIVE_UP_S {
                self.finder = Some(TonePairFinder::new(audio_rate, FIND_RESOLUTION_HZ));
            }
            return false;
        };
        self.offset_hz = offset;
        self.retune = Mixer::new(-offset, audio_rate);
        self.finder = None;
        self.narrow = std::mem::take(&mut self.waiting);
        true
    }

    fn settle(&mut self) -> Vec<Vec<u8>> {
        let scores: Vec<Score> =
            self.readers.iter().map(|r| Score::of(&r.line, &r.tones)).collect();
        let Some(k) = best(&scores).filter(|k| scores[*k].clean >= SETTLE_CLEAN) else {
            return Vec::new();
        };
        let mut winner = self.readers.swap_remove(k);
        let held = std::mem::take(&mut winner.held);
        self.readers = vec![winner];
        held
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
        if self.readers.is_empty() {
            return Err(common::Error::other("rtty needs four samples a symbol"));
        }

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);
        let searching = self.finder.is_some();
        if !self.tune() {
            return Ok(());
        }
        if searching && self.offset_hz.abs() > RESHAPE_HZ {
            let at = self.channel_hz + self.offset_hz;
            c.request(pipeline::Request::Reshape {
                lo_hz: at - CHANNEL_WIDTH_HZ / 2.0,
                hi_hz: at + CHANNEL_WIDTH_HZ / 2.0,
            });
        }
        self.tuned.clear();
        self.retune.process(&self.narrow, &mut self.tuned);

        let mut symbols = std::mem::take(&mut self.symbols);
        for r in self.readers.iter_mut() {
            symbols.clear();
            r.tones.process(&self.tuned, &mut symbols);
            for sym in &symbols {
                if let Some(run) = r.line.push(*sym) {
                    r.held.push(run);
                }
            }
        }
        let read = match self.readers.len() {
            1 => std::mem::take(&mut self.readers[0].held),
            _ => self.settle(),
        };
        self.symbols = symbols;
        for run in read {
            self.runs += 1;
            o.packets_mut().push(self.meter.packet_now(run));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.meter.reset();
        self.rebuild();
    }

    fn params(&self) -> Vec<Param> {
        let speed = self
            .speed
            .map_or(Speed::ALL.len(), |v| Speed::ALL.iter().position(|s| *s == v).unwrap_or(0));
        let shift = self
            .shift
            .map_or(Shift::ALL.len(), |v| Shift::ALL.iter().position(|s| *s == v).unwrap_or(0));
        let with_any = |mut v: Vec<String>| {
            v.push(ANY.into());
            v
        };
        vec![
            Param::float(CHANNEL_HZ, self.channel_hz, 1e5..=1e9).unit("Hz").label("Channel"),
            Param::choice(
                SPEED,
                speed,
                with_any(Speed::ALL.iter().map(|s| s.to_string()).collect()),
            )
            .unit("baud")
            .label("Speed"),
            Param::choice(
                SHIFT,
                shift,
                with_any(Shift::ALL.iter().map(|s| s.to_string()).collect()),
            )
            .unit("Hz")
            .label("Shift"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => self.channel_hz = v.as_f64().unwrap_or(self.channel_hz),
            SPEED => self.speed = speed_of(&v),
            SHIFT => self.shift = shift_of(&v),
            _ => return Err(common::Error::other(format!("rtty: unknown parameter {name:?}"))),
        }
        self.rebuild();
        Ok(())
    }
}

const ANY: &str = "any";

const REACH_HZ: f64 = 250.0;

const FIND_RESOLUTION_HZ: f64 = 4.0;

const FIND_S: f64 = 1.0;

const FIND_GIVE_UP_S: f64 = 10.0;

const FIND_OVER_DB: f32 = 10.0;

const RESHAPE_HZ: f64 = 100.0;

const SETTLE_CLEAN: i64 = 20;

/// Whether a frame off the bus could be a run of Baudot codes. Five bits is
/// all a code has, so anything with a byte above 31 in it was framed by
/// something else.
fn is_baudot(bytes: &[u8]) -> bool {
    bytes.len() >= MIN_CHARS
        && bytes.iter().all(|b| *b < 32)
        && rtty::printable(bytes) >= MIN_PRINTABLE
}

impl Protocol for Rtty {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }

    /// Amateur HF, the utility and weather circuits, and a few VHF links:
    /// a teleprinter is wherever somebody put one.

    /// A run of five-bit codes is a shape no other decoder on the bus
    /// produces, but nothing in RTTY checks, so the claim is HF-wide and
    /// late rather than specific.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 30_000_000 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !is_baudot(bytes) {
            return None;
        }
        Some(read(bytes).into_iter().collect())
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
    /// The line, then the shift a station keys: the narrow 170 Hz, since
    /// that is what the default speed goes with and what the other end
    /// expects on the amateur bands.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(RTTY_TX.name),
            modulator: NodeSpec::new(crate::mod_nodes::FSK_MOD.name)
                .f("shift_hz", Shift::default().hz())
                .f("offset_hz", 0.0),
            sends: crate::protocol::Sends::Words(TEXT),
        })
    }
}

/// Text as a keyed teleprinter line.
///
/// The transmit mirror of [`RttyNode`]: the Baudot codes come from
/// [`decode::rtty::encode`], which is the table the receiver reads back, and
/// each is framed as a start element of space, five data bits with the least
/// significant first, and a stop element of mark.
///
/// The stop is two bit times rather than the teleprinter's one and a half,
/// because the keyer works in whole bits and the receiver reads the stop as
/// one element however long it is held. The line rests at mark before and
/// after the over, which is what closes the run at the far end.
pub struct RttyTxNode {
    text: String,
    speed: Speed,
    shift: Shift,
    keyer: crate::tx_nodes::Keyer,
    rate: f64,
}

/// Bit times of resting mark in front of an over and behind it. The far end
/// closes a run after 24 symbols with no character framed, so the tail has
/// to be longer than that or two overs arrive as one.
const IDLE_BITS: usize = 32;

impl Default for RttyTxNode {
    fn default() -> Self {
        Self::new("", Speed::default(), Shift::default())
    }
}

impl RttyTxNode {
    pub fn new(text: &str, speed: Speed, shift: Shift) -> Self {
        let mut n = Self {
            text: text.into(),
            speed,
            shift,
            keyer: crate::tx_nodes::Keyer::new(speed.baud(), 0.0),
            rate: 0.0,
        };
        n.reload();
        n
    }

    fn reload(&mut self) {
        self.keyer.set_baud(self.speed.baud());
        if self.text.is_empty() {
            self.keyer.load(Vec::new());
            return;
        }
        let mut bits = vec![true; IDLE_BITS];
        for code in rtty::encode(&self.text) {
            bits.push(false);
            bits.extend((0..5).map(|k| code >> k & 1 != 0));
            bits.extend([true, true]);
        }
        bits.extend(std::iter::repeat_n(true, IDLE_BITS));
        self.keyer.load(bits);
    }
}

impl Simple for RttyTxNode {
    fn name(&self) -> &str {
        RTTY_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            ("keying".into(), format!("{} baud, {} Hz", self.speed, self.shift)),
            ("overs".into(), self.keyer.passes().to_string()),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("rtty_tx needs a clock to key against"));
        }
        self.rate = i.spec.rate;
        Ok(StreamSpec {
            kind: PortKind::Timings,
            rate: i.spec.rate,
            center: i.spec.center,
            bandwidth: CHANNEL_WIDTH_HZ,
            channels: 1,
            flow: pipeline::port::Flow::Tx,
            domain: pipeline::port::Domain::Baseband,
        })
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if i.is_empty() {
            return Ok(());
        }
        o.timings_mut().extend(self.keyer.take(i.len(), self.rate));
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        let speed = Speed::ALL.iter().position(|&s| s == self.speed).unwrap_or(0);
        let shift = Shift::ALL.iter().position(|&s| s == self.shift).unwrap_or(0);
        vec![
            Param::text(TEXT, self.text.clone()).label("Over"),
            Param::choice(SPEED, speed, Speed::ALL.iter().map(|s| s.to_string()).collect())
                .label("Speed")
                .unit("baud"),
            Param::choice(SHIFT, shift, Shift::ALL.iter().map(|s| s.to_string()).collect())
                .label("Shift")
                .unit("Hz"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            TEXT => self.text = value.as_str().unwrap_or_default().to_string(),
            SPEED => self.speed = speed_of(&value).unwrap_or_default(),
            SHIFT => self.shift = shift_of(&value).unwrap_or_default(),
            _ => return Err(common::Error::other(format!("rtty_tx: unknown parameter {name:?}"))),
        }
        self.reload();
        Ok(())
    }
}

/// The carrier this stage is pointed at, and how it is keyed.
const CHANNEL_HZ: &str = "channel_hz";
const SPEED: &str = "speed";
const SHIFT: &str = "shift";
const TEXT: &str = "text";

pub const RTTY_TX: StageDesc = StageDesc {
    name: "rtty_tx",
    summary: "Key an over as Baudot: a start element, five bits and a stop",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let speed = s.get(SPEED).and_then(speed_of).unwrap_or_default();
    let shift = s.get(SHIFT).and_then(shift_of).unwrap_or_default();
    Ok(Box::new(RttyTxNode::new(s.str_or(TEXT, ""), speed, shift)))
}

pub const DESC: StageDesc = StageDesc {
    name: "rtty",
    summary: "One RTTY channel: Baudot at 45.45 to 200 baud, 170 to 850 Hz shift",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let speed = s.get(SPEED).and_then(speed_of);
    let shift = s.get(SHIFT).and_then(shift_of);
    Ok(Box::new(RttyNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), speed, shift)))
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

#[cfg(test)]
mod tests {
    use super::*;

    /// The text a layer states, where it states one.
    fn wrote(d: &common::packet::Proto) -> Option<String> {
        d.facts.iter().find_map(|f| match f {
            common::packet::Fact::Message(w) => Some(w.text.clone()),
            _ => None,
        })
    }
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
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes().to_vec()));
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

        let mut node = RttyNode::new(channel, Some(Speed::default()), Some(Shift::default()));
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);

        assert_eq!(frames.len(), 1, "{} runs off the air", frames.len());
        let d = read(&frames[0]).expect("a decode");
        assert_eq!(wrote(&d).as_deref(), Some(OVER));
        assert_eq!((d.id, d.kind), ("rtty", "text"));

        // The text is 24 characters and the five-bit code needs four shifts
        // to reach the digits in the two calls and back again, which is what
        // the frame holds.
        assert_eq!(frames[0].len(), 29);
    }

    /// The transmitter into the receiver: an over keyed by the chain the
    /// protocol declares, read back by the node that reads real stations.
    ///
    /// What it pins beyond the synthetic keying above is that the shift the
    /// protocol asks the modulator for, the framing the source builds and
    /// the pacing against the radio's clock all agree with the receiver.
    #[test]
    fn an_over_keyed_by_the_transmit_chain_is_read_back() {
        let (rate, center) = (48_000.0, 14_083_000.0);
        // 24 characters and five shifts at 45.45 baud, eight bits each,
        // with 32 bit times of resting mark either side: 296 bits, which is
        // 6.5 seconds.
        let air = crate::tx_nodes::transmit_for(
            &Rtty,
            rate,
            Hz(center as u64),
            6.6,
            &[(TEXT, ParamValue::Text(OVER.into()))],
        );
        // 78 blocks of 4096 samples, which is a whole number of bits here.
        assert_eq!(air.len(), 318_912, "6.6 s of samples at {rate}");

        let mut node = RttyNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &air, rate, center);
        assert_eq!(frames.len(), 1, "{} runs off the air", frames.len());
        let d = read(&frames[0]).expect("a decode");
        assert_eq!(wrote(&d).as_deref(), Some(OVER));
        assert_eq!(frames[0].len(), 29);
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
            let d = read(&frames[0]).expect("a decode");
            assert_eq!(wrote(&d).as_deref(), Some(OVER), "inverted={invert}");
        }
    }

    #[test]
    fn a_mistuned_station_is_found_and_read_whole() {
        let (rate, center) = (8_000.0, 14_083_000.0);
        for off in [25.0, 85.0, 200.0, -200.0] {
            let iq = keyed(OVER, rate, off, false);
            let mut node = RttyNode::default();
            node.negotiate(&spec(rate, center)).unwrap();
            let frames = run(&mut node, &iq, rate, center);
            assert_eq!(frames.len(), 1, "{off} Hz off");
            assert_eq!(rtty::text(&frames[0]), OVER, "{off} Hz off");
            assert!(
                (node.offset_hz() - off).abs() <= 4.0,
                "{off} Hz off found at {}",
                node.offset_hz()
            );
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
    fn a_node_told_nothing_finds_the_speed_and_shift_it_reads() {
        let (rate, center) = (8_000.0, 14_083_000.0);
        let iq = keyed(OVER, rate, 0.0, false);
        let mut node = RttyNode::any(center);
        node.negotiate(&spec(rate, center)).unwrap();
        let frames = run(&mut node, &iq, rate, center);
        assert_eq!(frames.len(), 1);
        assert_eq!(rtty::text(&frames[0]), OVER);
        assert_eq!(node.reading(), Some((Speed::Baud45, Shift::Narrow)));
    }

    #[test]
    fn a_node_told_nothing_reads_nothing_out_of_noise() {
        let (rate, center) = (8_000.0, 14_083_000.0);
        let mut seed = 0x0dd_ba11_5eed_cafeu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> = (0..rate as usize * 20).map(|_| C32::new(rng(), rng())).collect();
        let mut node = RttyNode::any(center);
        node.negotiate(&spec(rate, center)).unwrap();
        assert_eq!(run(&mut node, &iq, rate, center).len(), 0);
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
