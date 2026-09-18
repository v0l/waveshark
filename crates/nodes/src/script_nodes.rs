//! A described protocol as a graph node: a channel in, the frames its
//! description reads out, through the demodulator the description names.
//!
//! Wiring only. The waveform is [`dsp::fsk::BitSync`] at the baud and
//! deviation the description's `radio` block declares, and the frame is
//! [`decode::script::Scripted`], which holds the sync word, the checks and
//! the layout. A description with a `radio` is a [`Protocol`] here as well
//! as a pulse protocol, and the registry lists one per installed
//! description, so a file fetched from the dataset appears in the mode menu
//! and the scanner table without a build.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness, TxChain};
use common::Result;
use common::Value;
use decode::Protocol as _;
use decode::bits::BitBuffer;
use decode::script::{self, Scripted};
use dsp::fsk::BitSync;
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use pipeline::{Param, ParamValue};
use std::collections::BTreeMap;

/// Setting naming the description a node reads with
pub const DESC_NAME: &str = "desc";
pub const CHANNEL_HZ: &str = "channel_hz";

/// Bits kept behind the search, so a frame split across two blocks is
/// still whole when the second arrives
fn keep_bits(p: &Scripted) -> usize {
    let f = &p.desc().frame;
    (f.sync_bits + f.bits * 2 + 16) * 2
}

pub struct ScriptNode {
    /// None for a node built with no description named, which negotiates
    /// nothing and reads nothing
    proto: Option<Scripted>,
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    sync: BitSync,
    bits: Vec<bool>,
    /// Bits dropped off the front, so a frame's position is a stream
    /// position rather than one in what is left
    dropped: u64,
    /// Where the search has reached, in stream positions
    read_from: u64,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    meter: crate::FrameMeter,
    accepted: u64,
}

impl ScriptNode {
    pub fn new(proto: Scripted, channel_hz: f64) -> Self {
        let r = proto.desc().radio.as_ref().expect("a radio description");
        let fsk = r.fsk.expect("validated");
        let work = r.rate_hz();
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(work, 1, r.width_hz / 2.0, 60.0),
            sync: BitSync::with_bandwidth(
                work,
                fsk.baud,
                fsk.bandwidth_hz.unwrap_or(2.0 * (fsk.deviation_hz + fsk.baud / 2.0)),
            ),
            bits: Vec::new(),
            dropped: 0,
            read_from: 0,
            mixed: Vec::new(),
            narrow: Vec::new(),
            meter: crate::FrameMeter::new(work, channel_hz as u64, 0.05),
            accepted: 0,
            proto: Some(proto),
        }
    }

    /// A node naming no description, for a registry that builds every stage
    /// once to ask its name
    pub fn empty() -> Self {
        Self {
            proto: None,
            channel_hz: 0.0,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(1.0e6, 1, 1.0e5, 60.0),
            sync: BitSync::new(1.0e6, 1.0e5),
            bits: Vec::new(),
            dropped: 0,
            read_from: 0,
            mixed: Vec::new(),
            narrow: Vec::new(),
            meter: crate::FrameMeter::new(1.0e6, 0, 0.05),
            accepted: 0,
        }
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    fn placement(&self) -> Placement {
        self.proto.as_ref().map_or(Placement::Anywhere, |p| placement_of(p.desc()))
    }
}

fn placement_of(d: &script::Desc) -> Placement {
    let r = d.radio.as_ref().expect("a radio description");
    if !r.channels.is_empty() {
        Placement::Channels(r.channels.clone())
    } else {
        Placement::Bands(r.bands.iter().map(|[lo, hi]| (*lo, *hi)).collect())
    }
}

impl Simple for ScriptNode {
    fn name(&self) -> &str {
        "script"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("a described protocol reads complex baseband"));
        }
        let Some(proto) = &self.proto else {
            return Err(common::Error::other("no description named"));
        };
        let r = proto.desc().radio.as_ref().expect("a radio description");
        let fsk = r.fsk.expect("validated");
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if !self.placement().covers(self.channel_hz, r.width_hz) {
            self.channel_hz = center;
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - r.width_hz / 2.0 {
            return Err(common::Error::other("the channel is outside the span"));
        }
        let want = r.rate_hz();
        let mut factor = 1usize;
        while rate / (factor * 2) as f64 >= want {
            factor *= 2;
        }
        let work = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, r.width_hz / 2.0, 60.0);
        self.sync = BitSync::with_bandwidth(
            work,
            fsk.baud,
            fsk.bandwidth_hz.unwrap_or(2.0 * (fsk.deviation_hz + fsk.baud / 2.0)),
        );
        if !self.sync.usable() {
            return Err(common::Error::other(format!(
                "{} needs four samples a symbol at {} baud",
                proto.desc().name,
                fsk.baud
            )));
        }
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 0.05);
        self.bits.clear();
        self.dropped = 0;
        self.read_from = 0;
        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = r.width_hz.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let (Some(iq), Some(proto)) = (i.as_iq(), &self.proto) else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);
        self.sync.process(&self.narrow, &mut self.bits);

        let from = (self.read_from - self.dropped) as usize;
        let mut buf = BitBuffer::with_capacity(self.bits.len() - from);
        for b in &self.bits[from..] {
            buf.push(*b);
        }
        let found = proto.frames(&buf);
        let out = o.frames_mut();
        let mut consumed = from;
        for (_at, end, r) in &found {
            self.accepted += 1;
            out.push(self.meter.frame(r.raw.clone()));
            consumed = consumed.max(from + end);
        }
        self.read_from = self.dropped + consumed as u64;

        let keep = self.bits.len().min(keep_bits(proto));
        let cut = self.bits.len() - keep;
        if cut > 0 {
            self.bits.drain(..cut);
            self.dropped += cut as u64;
            self.read_from = self.read_from.max(self.dropped);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.sync.reset();
        self.meter.reset();
        self.bits.clear();
        self.dropped = 0;
        self.read_from = 0;
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "script",
    summary: "One channel of a described protocol, read through the demodulator it names",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let name = s.str_or(DESC_NAME, "");
    if name.is_empty() {
        return Ok(Box::new(ScriptNode::empty()));
    }
    let proto = script::named(name)
        .filter(|p| p.desc().radio.is_some())
        .ok_or_else(|| common::Error::other(format!("no radio description called {name:?}")))?;
    let hz = s.f64_or(CHANNEL_HZ, proto.desc().radio.as_ref().map_or(0.0, |r| r.default_hz()));
    Ok(Box::new(ScriptNode::new(proto, hz)))
}

/// A description with a radio, as the registry sees it
pub struct ScriptedProtocol {
    proto: Scripted,
    id: &'static str,
    widths: &'static [f64],
}

impl ScriptedProtocol {
    pub fn new(proto: Scripted) -> Self {
        let d = proto.desc();
        let r = d.radio.as_ref().expect("a radio description");
        let id = r.id.clone().unwrap_or_else(|| d.name.to_lowercase());
        let id: &'static str = Box::leak(id.into_boxed_str());
        let widths: &'static [f64] = Box::leak(vec![r.width_hz].into_boxed_slice());
        Self { proto, id, widths }
    }

    /// The row a frame off the bus becomes
    fn decoded(&self, bytes: &[u8], center: common::Hz) -> Option<Decoded> {
        let frame = BitBuffer::from_bytes(bytes).slice(0, self.proto.desc().frame.bits);
        let r = self.proto.read(&frame).ok()?;
        let mut d = Decoded::bytes(self.proto.name(), center, 0.0, bytes.to_vec())
            .with_text(r.to_string())
            .with_detail(r.fields_line())
            .with_fields(r.fields.iter().map(|(k, v)| (k.clone(), v.clone())).collect())
            .with_types(r.types.iter().map(|(k, t)| (k.clone(), *t)).collect())
            .with_modulation(common::Modulation::Fsk2)
            .with_crc(r.crc_valid);
        if let Some(id) = &r.device {
            d = d.by(common::Identity::new(format!("ism:{}", r.model), id.clone()));
        }
        Some(d)
    }
}

impl Protocol for ScriptedProtocol {
    fn id(&self) -> &'static str {
        self.id
    }
    fn label(&self) -> &'static str {
        self.proto.name()
    }
    fn placement(&self) -> Placement {
        placement_of(self.proto.desc())
    }
    fn default_hz(&self) -> f64 {
        self.proto.desc().radio.as_ref().map_or(0.0, |r| r.default_hz())
    }
    fn shape(&self) -> Shape {
        let r = self.proto.desc().radio.as_ref().expect("a radio description");
        Shape {
            widths: self.widths,
            min_rate_hz: r.rate_hz() / 2.0,
            feed_rate_hz: r.rate_hz(),
            span_wide: false,
            families: &[],
        }
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: self.widths[0] as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !self.placement().covers(hz, self.widths[0]) {
            return None;
        }
        // reading the bytes again is the claim: a frame of another
        // protocol in the same band fails the checks and is handed on
        self.decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: self.widths[0], label: self.proto.name().to_uppercase() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).s(DESC_NAME, self.proto.name()).f(CHANNEL_HZ, at.center_hz)]
    }

    /// The transmit mirror of the chain: the described frame from the
    /// fields the source card holds, through the FSK modulator at twice
    /// the deviation for a shift
    fn transmit(&self) -> Option<TxChain> {
        let fsk = self.proto.desc().radio.as_ref()?.fsk?;
        let mut source = NodeSpec::new(SCRIPT_TX.name).s(DESC_NAME, self.proto.name());
        // the vector's fields are the card's defaults, so keying with
        // nothing set still sends something that reads back
        for (k, l) in self.proto.desc().vectors.first()?.fields.iter() {
            let text = match l.value() {
                Value::Int(n) => n.to_string(),
                Value::Float(x) => x.to_string(),
                Value::Bool(b) => b.to_string(),
                Value::Text(t) => t,
            };
            source = source.s(&format!("{FIELD_PREFIX}{k}"), &text);
        }
        Some(TxChain {
            source,
            modulator: NodeSpec::new(crate::mod_nodes::FSK_MOD.name)
                .f("shift_hz", fsk.deviation_hz * 2.0)
                .f("offset_hz", 0.0),
        })
    }
}

/// Every installed description with a radio, as protocols
pub fn protocols() -> Vec<ScriptedProtocol> {
    script::current()
        .into_iter()
        .filter(|p| p.desc().radio.is_some())
        .map(ScriptedProtocol::new)
        .collect()
}

/// The installed set is process wide, so the tests that put a description
/// in it take this in turn rather than racing each other
#[cfg(test)]
static INSTALLING: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    /// A two-level FSK link nothing else in the tree reads: 38400 baud,
    /// 20 kHz deviation, an 8 byte frame behind a d391 sync with a CRC-8
    const LINK: &str = r#"
name: Test-Link
radio: { fsk: { baud: 38400, deviation_hz: 20000 }, bands: [[433.0e6, 434.8e6]], width_hz: 100000 }
frame: { bits: 64, find: sync, sync: "aad391", sync_bits: 24 }
check: { kind: crc8, poly: 0x31, init: 0, over: [0, 56], at: 56 }
fields:
  - { name: id, bits: 16, data: int }
  - { name: temperature_c, bits: 16, type: int, data: float, unit: c, scale: 0.01 }
  - { name: battery_mv, bits: 16, data: int, unit: mv }
  - { name: seq, bits: 8, data: int }
  - { bits: 8, hidden: true }
vectors:
  - { hex: "a5 c3 2b 6e 0b 5a 35 48", fields: { id: 0xa5c3, temperature_c: 111.18, battery_mv: 2906, seq: 53 } }
"#;

    #[test]
    fn a_described_fsk_link_is_read_off_the_air() {
        let d = script::Desc::parse(LINK).unwrap();
        let proto = Scripted::new(d);
        script::check(&proto).unwrap();
        let fields =
            proto.desc().vectors[0].fields.iter().map(|(k, l)| (k.clone(), l.value())).collect();
        let air = proto.air_bits(&fields).unwrap();
        // idle before and after, as a real link has
        let mut stream = BitBuffer::new();
        for _ in 0..32 {
            stream.push(true);
            stream.push(false);
        }
        for i in 0..air.len() {
            stream.push(air.get(i).unwrap());
        }
        for _ in 0..8 {
            stream.push(true);
            stream.push(false);
        }
        let bits: Vec<bool> = (0..stream.len()).map(|i| stream.get(i).unwrap()).collect();
        let (rate, center) = (1_000_000.0, 433_900_000.0);
        let iq = dsp::fsk::modulate(&bits, rate, 38400.0, 20000.0, 0.5);
        let quiet = vec![C32::new(0.0, 0.0); (rate * 0.002) as usize];
        let air = [&quiet[..], &iq[..], &quiet[..]].concat();

        // keyed 20 kHz above the centre of the span, so the mixer has work to do
        let mut node = ScriptNode::new(proto, 433_920_000.0);
        let ins = [PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }];
        let out = node.negotiate(&ins[0]).unwrap();
        assert_eq!(out.kind, PortKind::Frames);
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in air.chunks(8192) {
            let mut o = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&Payload::Iq(block.to_vec()), &mut o, &mut ctx).unwrap();
            if let Payload::Frames(f) = o {
                frames.extend(f);
            }
        }
        assert_eq!(frames.len(), 1, "one frame off the air");
        assert_eq!(node.accepted(), 1);
        assert!(frames[0].rssi_dbfs.is_finite() && frames[0].snr_db.is_finite());

        let p = ScriptedProtocol::new(Scripted::new(script::Desc::parse(TEST_LINK).unwrap()));
        let row = p.decoded(&frames[0].bytes, Hz(433_920_000)).expect("the frame reads back");
        assert_eq!(row.field("temperature_c"), Some(&common::Value::Float(111.18)));
        assert_eq!(row.field("battery_mv"), Some(&common::Value::Int(2906)));
        assert_eq!(
            row.field_type("temperature_c"),
            Some(common::FieldType {
                data: common::Data::Float,
                unit: Some(common::Unit::Celsius)
            })
        );
    }

    /// The transmit chain, built from the protocol's own declaration and
    /// run through the registry, into the receive chain: key, modulate,
    /// demodulate, decode, and check the fields of what comes back
    #[test]
    fn the_transmit_chain_feeds_the_receive_chain_through_the_registry() {
        let _installing = INSTALLING.lock().unwrap_or_else(|e| e.into_inner());
        let got = script::install(&[("tx.yaml".into(), TEST_LINK.into())]);
        assert_eq!(got.names, ["Tx-Test-Link"], "{:?}", got.refused);
        let p = crate::protocol::by_id("tx-test-link").expect("registered");

        // key it the way the receiver does: the transmit clock into the
        // chain the protocol declares
        let rate = 1_000_000.0f64;
        let clock = StreamSpec {
            kind: PortKind::Real,
            rate,
            center: Hz(433_900_000),
            channels: 1,
            flow: pipeline::port::Flow::Tx,
            domain: pipeline::port::Domain::Baseband,
            ..Default::default()
        };
        let tx = p.transmit().expect("a described protocol transmits");
        let mut g = crate::build_chain(clock, &[tx.source, tx.modulator], &crate::registry())
            .expect("the transmit chain builds through the registry");
        let mut air: Vec<C32> = Vec::new();
        for _ in 0..20 {
            {
                let buf = g.input_buf();
                buf.clear();
                buf.real_mut().resize(16_384, 0.0);
            }
            g.run().expect("the transmit chain runs");
            match g.output().as_iq() {
                // a block the keyer rested through is silence on the air
                Some(iq) if !iq.is_empty() => air.extend_from_slice(iq),
                _ => air.extend(std::iter::repeat_n(C32::new(0.0, 0.0), 16_384)),
            }
        }
        assert!(air.iter().any(|s| s.norm() > 0.1), "the chain put something on the air");

        // and read it back the way the receive chain does, through the
        // registry's own stage builder
        let at = crate::protocol::Placed {
            center_hz: 433_920_000.0,
            width_hz: 100_000.0,
            rate,
            snr_db: 20.0,
            origin: None,
        };
        let chain = p.chain(at);
        let mut r =
            crate::build_chain(StreamSpec::iq(rate, Hz(433_900_000)), &chain, &crate::registry())
                .expect("the receive chain builds through the registry");
        let mut frames = Vec::new();
        for block in air.chunks(8192) {
            r.feed_iq(block).expect("the receive chain runs");
            frames.extend(r.output().as_frames().unwrap_or(&[]).iter().cloned());
        }
        // the keyer sends the frame again and again while the key is down,
        // and every copy has to read as the same reading
        assert!(!frames.is_empty(), "nothing came back off the air");
        for f in &frames {
            let packet = common::Packet::of_frame(0, 100_000, f.clone());
            let row = p.read_frame(&packet, &f.bytes).expect("the frame claims itself");
            assert_eq!(row.len(), 1);
            assert_eq!(row[0].field("id"), Some(&Value::Int(0xa5c3)));
            assert_eq!(row[0].field("temperature_c"), Some(&Value::Float(111.18)));
            assert_eq!(row[0].field("battery_mv"), Some(&Value::Int(2906)));
            assert_eq!(row[0].field("seq"), Some(&Value::Int(53)));
            assert_eq!(
                row[0].field_type("temperature_c"),
                Some(common::FieldType {
                    data: common::Data::Float,
                    unit: Some(common::Unit::Celsius)
                })
            );
        }

        script::install(&[]);
        assert!(crate::protocol::by_id("tx-test-link").is_none(), "gone once uninstalled");
    }

    #[test]
    fn an_installed_radio_description_is_a_protocol_in_the_registry() {
        let _installing = INSTALLING.lock().unwrap_or_else(|e| e.into_inner());
        let got = script::install(&[("link.yaml".into(), TEST_LINK.into())]);
        assert_eq!(got.names, ["Tx-Test-Link"], "{:?}", got.refused);
        let p = crate::protocol::by_id("tx-test-link").expect("registered under its id");
        assert_eq!(p.label(), "Tx-Test-Link");
        assert!(p.placement().covers(433_920_000.0, 100_000.0));
        let at = Placed {
            center_hz: 433_920_000.0,
            width_hz: 100_000.0,
            rate: 1_000_000.0,
            snr_db: 20.0,
            origin: None,
        };
        let chain = p.chain(at);
        let g = crate::build_chain(
            StreamSpec::iq(1_000_000.0, Hz(433_900_000)),
            &chain,
            &crate::registry(),
        )
        .expect("the chain builds through the registry");
        assert_eq!(g.order().count(), 1);
        script::install(&[]);
        assert!(crate::protocol::by_id("tx-test-link").is_none(), "gone once uninstalled");
    }
}

/// What the source card carries besides the description's name: a field of
/// the frame, as the operator set it
pub const FIELD_PREFIX: &str = "field.";

pub struct ScriptTxNode {
    /// None for a node built with no description named, which keys nothing
    proto: Option<Scripted>,
    keyer: crate::tx_nodes::Keyer,
    rate: f64,
    /// The frame's fields as the card holds them, by name
    fields: BTreeMap<String, Value>,
}

impl ScriptTxNode {
    /// A transmitter for `proto`, carrying `fields`: the frame it keys
    pub fn with_fields(proto: Scripted, fields: &BTreeMap<String, Value>) -> Self {
        let baud =
            proto.desc().radio.as_ref().and_then(|r| r.fsk.map(|f| f.baud)).unwrap_or(4800.0);
        // a burst protocol stops between passes rather than idling on a
        // carrier, which is what the burst detector at the other end cuts
        // its packages on
        let mut n = Self {
            proto: Some(proto),
            keyer: crate::tx_nodes::Keyer::new(baud, 10.0).resting_silent(),
            rate: 0.0,
            fields: fields.clone(),
        };
        n.load(fields);
        n
    }

    /// A node naming no description
    pub fn unloaded() -> Self {
        Self {
            proto: None,
            keyer: crate::tx_nodes::Keyer::new(4800.0, 10.0),
            rate: 0.0,
            fields: BTreeMap::new(),
        }
    }

    fn load(&mut self, fields: &BTreeMap<String, Value>) {
        let Some(proto) = &self.proto else { return };
        let air = match proto.air_bits(fields) {
            Ok(air) => air,
            Err(e) => {
                tracing::warn!("{} will not key: {e}", proto.name());
                self.keyer.load(Vec::new());
                return;
            }
        };
        let bits: Vec<bool> = (0..air.len()).map(|i| air.get(i).unwrap_or(false)).collect();
        self.keyer.load(bits);
    }
}

impl Simple for ScriptTxNode {
    fn name(&self) -> &str {
        "script_tx"
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = vec![(
            "protocol".into(),
            self.proto.as_ref().map_or(String::new(), |p| p.name().to_string()),
        )];
        for (k, v) in &self.fields {
            out.push((k.clone(), v.to_string()));
        }
        if self.keyer.passes() > 0 {
            out.push(("sent".into(), self.keyer.passes().to_string()));
        }
        out
    }

    fn params(&self) -> Vec<Param> {
        let mut out = vec![
            Param::text(
                DESC_NAME,
                self.proto.as_ref().map_or(String::new(), |p| p.name().to_string()),
            )
            .label("Protocol"),
        ];
        let names: Vec<String> = self
            .proto
            .as_ref()
            .map(|p| {
                p.desc()
                    .fields
                    .iter()
                    .filter_map(|it| match it {
                        decode::script::desc::Item::Field(f) if f.is_reported() => {
                            Some(f.name.clone())
                        }
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default();
        for n in names {
            let v = self.fields.get(&n);
            out.push(
                Param::text(
                    &format!("{FIELD_PREFIX}{n}"),
                    v.map_or(String::new(), |v| v.to_string()),
                )
                .label(&n),
            );
        }
        out
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let Some(rest) = name.strip_prefix(FIELD_PREFIX) else {
            return Err(common::Error::other(format!("script_tx: unknown parameter {name:?}")));
        };
        let v = match value {
            ParamValue::Int(n) => Value::Int(n),
            ParamValue::Float(x) => Value::Float(x),
            ParamValue::Text(t) if !t.is_empty() => match t.parse::<i64>() {
                Ok(n) => Value::Int(n),
                Err(_) => match t.parse::<f64>() {
                    Ok(x) => Value::Float(x),
                    Err(_) => Value::Text(t.clone()),
                },
            },
            _ => return Ok(()),
        };
        self.fields.insert(rest.to_string(), v);
        self.keyer.set_baud(self.keyer.baud());
        if let Some(p) = &self.proto {
            self.keyer
                .set_baud(p.desc().radio.as_ref().and_then(|r| r.fsk).map_or(4800.0, |f| f.baud));
        }
        self.load(&self.fields.clone());
        Ok(())
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("script_tx needs a clock to key against"));
        }
        self.rate = i.spec.rate;
        let width = self
            .proto
            .as_ref()
            .and_then(|p| p.desc().radio.as_ref())
            .map_or(100_000.0, |r| r.width_hz);
        Ok(StreamSpec {
            kind: PortKind::Pulses,
            rate: i.spec.rate,
            center: i.spec.center,
            bandwidth: width,
            channels: 1,
            flow: pipeline::port::Flow::Tx,
            domain: pipeline::port::Domain::Baseband,
        })
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if i.is_empty() {
            return Ok(());
        }
        let pkg = self.keyer.take(i.len(), self.rate);
        if !pkg.pulses.is_empty() {
            o.pulses_mut().push(pkg);
        }
        Ok(())
    }
}

pub const SCRIPT_TX: StageDesc = StageDesc {
    name: "script_tx",
    summary: "Key a frame of a described protocol from the fields its card holds",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let name = s.str_or(DESC_NAME, "");
    if name.is_empty() {
        // a registry that builds every stage once to ask its name gets a
        // transmitter keyed to nothing
        return Ok(Box::new(ScriptTxNode::unloaded()));
    }
    let proto = script::named(name)
        .filter(|p| p.desc().radio.is_some())
        .ok_or_else(|| common::Error::other(format!("no radio description called {name:?}")))?;
    let fields = fields_of(s);
    Ok(Box::new(ScriptTxNode::with_fields(proto, &fields)))
}

/// The frame's fields as the source card holds them, `field.<name>`: a
/// number as written, and a field the card leaves alone takes its vector's
fn fields_of(s: &Settings) -> BTreeMap<String, Value> {
    let mut fields = BTreeMap::new();
    for (k, v) in s.iter() {
        let Some(rest) = k.strip_prefix(FIELD_PREFIX) else { continue };
        match v {
            pipeline::ParamValue::Text(t) if !t.is_empty() => match t.parse::<i64>() {
                Ok(n) => {
                    fields.insert(rest.to_string(), Value::Int(n));
                }
                Err(_) => {
                    if let Ok(x) = t.parse::<f64>() {
                        fields.insert(rest.to_string(), Value::Float(x));
                    } else {
                        fields.insert(rest.to_string(), Value::Text(t.clone()));
                    }
                }
            },
            pipeline::ParamValue::Int(n) => {
                fields.insert(rest.to_string(), Value::Int(*n));
            }
            pipeline::ParamValue::Float(x) => {
                fields.insert(rest.to_string(), Value::Float(*x));
            }
            _ => {}
        }
    }
    let name = s.str_or(DESC_NAME, "");
    if let Some(p) = script::named(name)
        && let Some(vector) = p.desc().vectors.first()
    {
        for (k, l) in vector.fields.iter() {
            fields.entry(k.clone()).or_insert_with(|| l.value());
        }
    }
    fields
}

/// A described FSK link for the transmit tests: the same frame the
/// receive tests use, under a name of its own
#[cfg(test)]
const TEST_LINK: &str = r#"
name: Tx-Test-Link
radio: { fsk: { baud: 38400, deviation_hz: 20000 }, bands: [[433.0e6, 434.8e6]], width_hz: 100000 }
frame: { bits: 64, find: sync, sync: "aad391", sync_bits: 24 }
check: { kind: crc8, poly: 0x31, init: 0, over: [0, 56], at: 56 }
fields:
  - { name: id, bits: 16, data: int }
  - { name: temperature_c, bits: 16, type: int, data: float, unit: c, scale: 0.01 }
  - { name: battery_mv, bits: 16, data: int, unit: mv }
  - { name: seq, bits: 8, data: int }
  - { bits: 8, hidden: true }
vectors:
  - { hex: "a5 c3 2b 6e 0b 5a 35 48", fields: { id: 0xa5c3, temperature_c: 111.18, battery_mv: 2906, seq: 53 } }
"#;

#[cfg(test)]
fn tx_fields() -> BTreeMap<String, Value> {
    BTreeMap::from([
        ("id".to_string(), Value::Int(0xa5c3)),
        ("temperature_c".to_string(), Value::Float(111.18)),
        ("battery_mv".to_string(), Value::Int(2906)),
        ("seq".to_string(), Value::Int(53)),
    ])
}

#[cfg(test)]
mod tx_tests {
    use super::*;
    use common::{C32, Hz};
    use decode::script::Desc;

    #[test]
    fn a_described_frame_is_keyed_and_read_back_off_the_air() {
        let _installing = INSTALLING.lock().unwrap_or_else(|e| e.into_inner());
        let proto = Scripted::new(Desc::parse(TEST_LINK).unwrap());

        // key it the way the transmit chain does: the source at the clock's
        // rate, the modulator after it
        let rate = 1_000_000.0f64;
        let clock = PortSpec {
            spec: StreamSpec {
                kind: PortKind::Real,
                rate,
                center: Hz(433_900_000),
                channels: 1,
                flow: pipeline::port::Flow::Tx,
                domain: pipeline::port::Domain::Baseband,
                ..Default::default()
            },
            latency: 0,
        };
        let mut tx =
            ScriptTxNode::with_fields(Scripted::new(Desc::parse(TEST_LINK).unwrap()), &tx_fields());
        let spec = tx.negotiate(&clock).unwrap();
        assert_eq!(spec.kind, PortKind::Pulses);
        let ins = [clock];
        let tags = Vec::new();
        let mut pulses = Vec::new();
        for _ in 0..40 {
            let mut o = Payload::Pulses(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            tx.process(&Payload::Real(vec![0.0; 16_384]), &mut o, &mut ctx).unwrap();
            if let Payload::Pulses(p) = o {
                pulses.extend(p);
            }
        }
        assert!(!pulses.is_empty(), "the keyer keyed something");

        // one pass of the frame, whole: the keyer's silent stop between
        // passes is the burst detector's package boundary, and the gap it
        // leaves is ten bit times of 26 us each
        let all = pulses.iter().flat_map(|pkg| pkg.pulses.iter()).collect::<Vec<_>>();
        // one pass is 88 air bits at 26 us, so 88 pulses of one bit each
        // once the keyer's runs are broken; take one pass plus slack
        let pass = &all[..all.len().min(88)];
        eprintln!("pass {} of {} pulses", pass.len(), all.len());
        let mut level_bits: Vec<bool> = Vec::new();
        for p in pass {
            level_bits.extend(std::iter::repeat_n(
                true,
                (p.mark as f64 / 1e6 * 38400.0).round() as usize,
            ));
            level_bits.extend(std::iter::repeat_n(
                false,
                (p.gap as f64 / 1e6 * 38400.0).round() as usize,
            ));
        }
        let iq = dsp::fsk::modulate(&level_bits, rate, 38400.0, 20000.0, 0.5);
        let quiet = vec![C32::new(0.0, 0.0); (rate * 0.002) as usize];
        let air = [&quiet[..], &iq[..], &quiet[..]].concat();

        // keyed 20 kHz above the centre of the span, so the mixer has work to do
        let mut node =
            ScriptNode::new(Scripted::new(Desc::parse(TEST_LINK).unwrap()), 433_920_000.0);
        let ins = [PortSpec { spec: StreamSpec::iq(rate, Hz(433_900_000)), latency: 0 }];
        let out = node.negotiate(&ins[0]).unwrap();
        assert_eq!(out.kind, PortKind::Frames);
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in air.chunks(8192) {
            let mut o = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&Payload::Iq(block.to_vec()), &mut o, &mut ctx).unwrap();
            if let Payload::Frames(f) = o {
                frames.extend(f);
            }
        }
        assert!((1..=2).contains(&frames.len()), "the frame came back, {}", frames.len());
        assert!(frames[0].rssi_dbfs.is_finite() && frames[0].snr_db.is_finite());

        let p = ScriptedProtocol::new(Scripted::new(Desc::parse(TEST_LINK).unwrap()));
        let row = p.decoded(&frames[0].bytes, Hz(433_920_000)).expect("the frame reads back");
        assert_eq!(row.field("temperature_c"), Some(&common::Value::Float(111.18)));
        assert_eq!(row.field("battery_mv"), Some(&common::Value::Int(2906)));
        assert_eq!(
            row.field_type("temperature_c"),
            Some(common::FieldType {
                data: common::Data::Float,
                unit: Some(common::Unit::Celsius)
            })
        );
    }

    #[test]
    fn an_installed_radio_description_is_a_protocol_in_the_registry() {
        let _installing = INSTALLING.lock().unwrap_or_else(|e| e.into_inner());
        let got = script::install(&[("link.yaml".into(), TEST_LINK.into())]);
        assert_eq!(got.names, ["Tx-Test-Link"], "{:?}", got.refused);
        let p = crate::protocol::by_id("tx-test-link").expect("registered under its id");
        assert_eq!(p.label(), "Tx-Test-Link");
        assert!(p.placement().covers(433_920_000.0, 100_000.0));
        let at = Placed {
            center_hz: 433_920_000.0,
            width_hz: 100_000.0,
            rate: 1_000_000.0,
            snr_db: 20.0,
            origin: None,
        };
        let chain = p.chain(at);
        let g = crate::build_chain(
            StreamSpec::iq(1_000_000.0, Hz(433_900_000)),
            &chain,
            &crate::registry(),
        )
        .expect("the chain builds through the registry");
        assert_eq!(g.order().count(), 1);
        script::install(&[]);
        assert!(crate::protocol::by_id("tx-test-link").is_none(), "gone once uninstalled");
    }
}
