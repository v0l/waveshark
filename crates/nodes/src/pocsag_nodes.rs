//! POCSAG as a graph node.
//!
//! The same shape as APRS and for the same reason: the channel is ordinary
//! narrowband FM, so the node mixes the pager channel down, filters it,
//! discriminates it, and hands the audio to `dsp::pocsag`, which does the bit
//! recovery, the sync search and the error correction. The message tables are
//! `decode::pocsag`. Neither of those knows about pipelines.
//!
//! What reaches the bus is a transmission's codewords, every one of which
//! passed BCH(31,21) or was corrected by it. Where APRS carries an AX.25
//! frame that passed a check sequence, this carries a run of codewords that
//! passed theirs.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use crate::tx_source::{Pace, air_time_us};
use common::Result;
use decode::pocsag::{self, Body};
use dsp::pocsag::{DEVIATION_HZ, PocsagConfig, PocsagDemod, Transmission};
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Domain, Flow, Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// A common UK and European paging channel, and only the default the node is
/// built with before the scanner table tells it where to listen.
pub const DEFAULT_HZ: f64 = 153_350_000.0;

/// The channel a POCSAG transmitter occupies: 4.5 kHz deviation at up to 2400
/// bits per second is about 12.5 kHz by Carson, and the allocations are
/// 12.5 or 25 kHz.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// Audio rate the discriminator output is decimated to. A whole number of
/// samples per bit at every rate POCSAG uses: 75, 32 and 16.
const AUDIO_HZ: f64 = 38_400.0;

pub struct PocsagNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    demod: PocsagDemod,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    meter: crate::FrameMeter,
    sends: Vec<Transmission>,
    accepted: u64,
}

impl Default for PocsagNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl PocsagNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            demod: PocsagDemod::new(AUDIO_HZ, PocsagConfig::default()),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            sends: Vec::new(),
            accepted: 0,
        }
    }

    /// Transmissions accepted since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for PocsagNode {
    fn name(&self) -> &str {
        "pocsag"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("pocsag reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("pocsag needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        // Built for the rate it will actually be handed rather than for a
        // nominal 38.4 kHz, since the decimation factor has to be an integer
        // and the span decides what that leaves.
        self.demod = PocsagDemod::new(audio_rate, PocsagConfig::default());
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 2.0);

        let mut out = i.spec.with_kind(PortKind::Frames);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);

        self.meter.feed(&self.narrow);
        self.sends.clear();
        let audio = std::mem::take(&mut self.audio);
        self.demod.process(&audio, &mut self.sends);
        self.audio = audio;

        let out = o.frames_mut();
        for t in &self.sends {
            self.accepted += 1;
            out.push(self.meter.frame(t.to_bytes()));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.demod.reset();
    }
}

/// The decodes a transmission's codewords become: one per page.
///
/// A transmission carries a transmitter's whole queue, so it is several pages
/// to several pagers, and each is a row of its own. What they share is the
/// bytes they came out of, which travel with each so that a log holds the
/// evidence rather than a rendering of it.
pub fn pocsag_decoded(bytes: &[u8], center: common::Hz) -> Vec<Decoded> {
    use common::Value;
    let codewords = Transmission::codewords_from_bytes(bytes);
    pocsag::parse(&codewords)
        .into_iter()
        .map(|m| {
            let mut fields: Vec<(String, Value)> = vec![
                ("address".into(), Value::Int(i64::from(m.address))),
                ("function".into(), Value::Int(i64::from(m.function))),
            ];
            let (protocol, text) = match &m.body {
                Body::Tone => ("POCSAG-Tone", None),
                Body::Numeric(s) => ("POCSAG-Numeric", Some(s.clone())),
                Body::Alpha(s) => ("POCSAG-Alpha", Some(s.clone())),
            };
            if let Some(t) = &text {
                fields.push(("message".into(), Value::Text(t.clone())));
            }
            let detail = match &text {
                Some(t) => format!("address={} {t}", m.address),
                None => format!("address={} tone only", m.address),
            };
            let mut d = Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
                .by(common::Identity::new("pocsag", m.address.to_string()))
                .with_link(pipeline::event::Link {
                    from: None,
                    to: Some(pipeline::event::Party::unit(m.address.to_string())),
                })
                .with_detail(detail)
                .with_fields(fields)
                .with_modulation(common::Modulation::Fsk2)
                // Every codeword read here either verified against
                // BCH(31,21) or was corrected by it, which is a real
                // integrity check rather than a plausibility argument.
                .with_crc(Some(true));
            if let Some(t) = text {
                // A page is written to somebody, whether a person typed it or
                // an alarm system did: either way it is addressed to whoever
                // carries the pager.
                d = d.written().with_text(t);
            }
            d
        })
        .collect()
}

/// How a page's message is coded on the air. The three bodies ITU-R M.584-2
/// allows, named so a picker can offer them.
const FORMATS: [(&str, u8); 3] = [("Alphanumeric", 3), ("Numeric", 0), ("Tone only", 0)];

/// A page, keyed as pulse timings.
///
/// The transmit mirror of [`PocsagNode`] down to the same three layers read
/// backwards: `decode::pocsag::encode` builds the codeword contents,
/// `dsp::pocsag::encode_bits` adds the BCH parity and the batching, and
/// `dsp::pulse::nrz` turns the bits into the mark and gap timings
/// [`crate::mod_nodes::FskModNode`] keys. Nothing new is encoded here.
///
/// A mark is the upper tone, so a one is transmitted high. Which way up a
/// pager reads it does not matter: the sync search accepts the complement
/// and inverts everything after it, which is why off-air recordings decode
/// both ways.
pub struct PocsagTxNode {
    address: u32,
    function: u8,
    message: String,
    /// One of [`dsp::pocsag::BAUDS`].
    baud: f64,
    /// Which of [`FORMATS`] the message is coded as.
    format: usize,
    rate: f64,
    pace: Pace,
}

impl Default for PocsagTxNode {
    fn default() -> Self {
        Self {
            address: 1_234_567,
            function: 3,
            message: String::new(),
            baud: 1200.0,
            format: 0,
            rate: 0.0,
            pace: Pace::default(),
        }
    }
}

impl PocsagTxNode {
    pub fn new(address: u32, function: u8, message: &str, baud: f64) -> Self {
        let mut n = Self { message: message.into(), ..Default::default() };
        n.address = address & 0x1F_FFFF;
        n.function = function & 3;
        n.baud = nearest_baud(baud);
        n
    }

    fn body(&self) -> Body {
        match FORMATS[self.format.min(FORMATS.len() - 1)].0 {
            "Numeric" => Body::Numeric(self.message.clone()),
            "Tone only" => Body::Tone,
            _ => Body::Alpha(self.message.clone()),
        }
    }

    /// The whole transmission as timings: preamble, batches and all.
    pub fn page(&self) -> Vec<common::pulse::Pulse> {
        let contents = pocsag::encode(self.address, self.function, &self.body());
        dsp::pulse::nrz(&dsp::pocsag::encode_bits(&contents), self.baud)
    }

    /// Pages handed to the modulator since the stage was built.
    pub fn sent(&self) -> u64 {
        self.pace.sent()
    }
}

/// The rate nearest `hz` that POCSAG is actually sent at. A transmitter
/// between two of them is a transmitter no pager can read.
fn nearest_baud(hz: f64) -> f64 {
    dsp::pocsag::BAUDS
        .iter()
        .copied()
        .min_by(|a, b| (a - hz).abs().total_cmp(&(b - hz).abs()))
        .unwrap_or(1200.0)
}

impl Simple for PocsagTxNode {
    fn name(&self) -> &str {
        POCSAG_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            ("to".into(), self.address.to_string()),
            ("baud".into(), format!("{:.0}", self.baud)),
            ("pages".into(), self.pace.sent().to_string()),
        ]
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("pocsag_tx needs a clock to key against"));
        }
        self.rate = input.spec.rate;
        Ok(StreamSpec {
            // Microsecond timings, so the port has no rate of its own; the
            // modulator's is passed through the way `sub_tx` passes it.
            kind: PortKind::Pulses,
            rate: input.spec.rate,
            center: input.spec.center,
            bandwidth: CHANNEL_WIDTH_HZ,
            channels: 1,
            flow: Flow::Tx,
            domain: Domain::Baseband,
        })
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.pace.clock(i.len(), self.rate);
        if !self.pace.due() {
            return Ok(());
        }
        let pulses = self.page();
        if pulses.is_empty() {
            return Ok(());
        }
        self.pace.spent(air_time_us(&pulses));
        o.pulses_mut().push(common::pulse::Package { pulses, ..Default::default() });
        Ok(())
    }

    fn reset(&mut self) {
        self.pace.reset();
    }

    fn params(&self) -> Vec<Param> {
        let bauds = dsp::pocsag::BAUDS.iter().map(|b| format!("{b:.0}")).collect();
        vec![
            Param::int(ADDRESS, i64::from(self.address), 0..=0x1F_FFFF).label("Address"),
            Param::int(FUNCTION, i64::from(self.function), 0..=3).label("Function"),
            Param::choice(FORMAT, self.format, FORMATS.iter().map(|(n, _)| (*n).into()).collect())
                .label("Format"),
            Param::text(MESSAGE, self.message.clone()).label("Message"),
            Param::choice(BAUD, nearest_index(self.baud), bauds).label("Rate").unit("baud"),
            Param::float(PAUSE_MS, self.pace.pause_ms(), 0.0..=60_000.0)
                .label("Between pages")
                .unit("ms"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            ADDRESS => self.address = value.as_i64().unwrap_or(0).clamp(0, 0x1F_FFFF) as u32,
            FUNCTION => self.function = value.as_i64().unwrap_or(3).clamp(0, 3) as u8,
            FORMAT => self.format = value.as_i64().unwrap_or(0).clamp(0, 2) as usize,
            MESSAGE => {
                self.message = match value {
                    ParamValue::Text(t) => t,
                    _ => return Err(common::Error::other("pocsag_tx: a message is text")),
                }
            }
            // A choice by index, and a rate by value, both land here: the
            // picker sends 0, 1 or 2 and a patch may carry 512.
            BAUD => {
                let v = value.as_f64().unwrap_or(1200.0);
                self.baud = match v < dsp::pocsag::BAUDS.len() as f64 {
                    true => dsp::pocsag::BAUDS[v as usize],
                    false => nearest_baud(v),
                };
            }
            PAUSE_MS => self.pace.set_pause_ms(value.as_f64().unwrap_or(1_000.0)),
            _ => {
                return Err(common::Error::other(format!("pocsag_tx: unknown parameter {name:?}")));
            }
        }
        Ok(())
    }
}

fn nearest_index(baud: f64) -> usize {
    dsp::pocsag::BAUDS.iter().position(|b| *b == baud).unwrap_or(1)
}

pub struct Pocsag;

impl Protocol for Pocsag {
    fn id(&self) -> &'static str {
        "pocsag"
    }
    fn label(&self) -> &'static str {
        "pager"
    }
    fn placement(&self) -> Placement {
        Placement::Anywhere
    }
    /// The widest of the paging allocations, so every narrower claim inside
    /// them is offered a frame first.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 65_000_000 }
    }
    /// A transmitter empties its queue in one go, so a transmission is a row
    /// per page rather than one row.
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !dsp::pocsag::is_pager_band(p.center_hz() as f64) {
            return None;
        }
        Some(pocsag_decoded(bytes, common::Hz(p.center_hz())))
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
    /// The amateur DAPNET channel: amateur rather than commercial because
    /// it is the one paging frequency that is the same across Europe.
    fn default_hz(&self) -> f64 {
        439_987_500.0
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} pager", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "POCSAG".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// The page keyer into the two-tone modulator. The shift is twice the
    /// deviation the receiver's discriminator is scaled for, because the
    /// modulator is told the distance between the tones and a pager channel
    /// is specified as the swing either side of the carrier.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(POCSAG_TX.name).f(BAUD, 1200.0),
            modulator: NodeSpec::new(crate::mod_nodes::FSK_MOD.name)
                .f("shift_hz", DEVIATION_HZ * 2.0),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    fn run(n: &mut impl Simple, i: &Payload, o: &mut Payload, rate: f64, center: f64) {
        let ins = [spec(rate, center)];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        n.process(i, o, &mut ctx).unwrap();
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = PocsagNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 160_000_000.0)).is_err());
        assert!(n.negotiate(&spec(25_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(10_000.0, DEFAULT_HZ)).is_err());
    }

    /// The whole path on synthetic RF: an FM carrier keyed with POCSAG bits,
    /// into the node, out as an addressed page.
    ///
    /// The point is that the three layers agree. Each is tested alone and
    /// each could be self-consistently wrong; only running them together
    /// shows that the codewords the framer packs are the ones the message
    /// tables read, in the frame positions the address depends on.
    #[test]
    fn a_modulated_transmission_becomes_a_page() {
        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let contents = pocsag::encode(1_234_568, 3, &Body::Alpha("MOVE TO CHANNEL 2".into()));
        let bits = dsp::pocsag::encode_bits(&contents);

        // Keyed FSK at 1200 baud: the bit rate is not announced anywhere in
        // the signal, so the node has to find it.
        let sps = (rate / 1200.0) as usize;
        let mut iq = Vec::with_capacity(bits.len() * sps);
        let mut phase = 0.0f64;
        for &b in &bits {
            let f = if b { -DEVIATION_HZ } else { DEVIATION_HZ };
            for _ in 0..sps {
                phase += std::f64::consts::TAU * f / rate;
                iq.push(common::C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }

        let mut node = PocsagNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        // A pager transmission has no closing flag: it ends when the signal
        // does, or when the batch that should have followed is not there. So
        // the trailing silence is not decoration, it is the thing that closes
        // the transmission, and it has to be long enough to be heard as
        // silence through the channel filter.
        let quiet = vec![common::C32::new(0.0, 0.0); 400_000];
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes));
            }
        }

        assert_eq!(frames.len(), 1, "expected one transmission off the air");
        let decodes = pocsag_decoded(&frames[0], Hz(center as u64));
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[0].text.as_deref(), Some("MOVE TO CHANNEL 2"));
        assert_eq!(decodes[0].media_type, pipeline::event::media::TEXT);
        assert!(decodes[0].written, "a page is written to whoever carries the pager");
        assert_eq!(decodes[0].crc_ok, Some(true));
        let get = |k: &str| decodes[0].fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("address"), Some(common::Value::Int(1_234_568)));
    }

    /// Keyed by the transmit stage, modulated, and read back by the receive
    /// stage: the round trip that says the encoder and the decoder agree
    /// about every layer between them.
    #[test]
    fn a_page_this_receiver_transmitted_is_a_page_this_receiver_reads() {
        let (rate, center) = (192_000.0, DEFAULT_HZ);
        let mut tx = PocsagTxNode::new(1_234_568, 3, "MOVE TO CHANNEL 2", 1200.0);
        let keyed = tx.negotiate(&spec(rate, center)).unwrap();

        let mut pulses = Payload::empty_of(PortKind::Pulses);
        run(&mut tx, &Payload::Real(vec![0.0; 4096]), &mut pulses, rate, center);
        assert_eq!(tx.sent(), 1, "one page per due slot");
        let air: f64 = air_time_us(&pulses.as_pulses().unwrap()[0].pulses) / 1e6;
        // 576 preamble bits and one batch of 544 at 1200 baud.
        assert!((air - 1120.0 / 1200.0).abs() < 1e-3, "{air} s of air");

        let mut modulator = crate::mod_nodes::FskModNode::new(0.0, DEVIATION_HZ * 2.0, 0.5);
        modulator.negotiate(&PortSpec { spec: keyed, latency: 0 }).unwrap();
        let mut iq = Payload::Iq(Vec::new());
        run(&mut modulator, &pulses, &mut iq, rate, center);
        let iq = match iq {
            Payload::Iq(v) => v,
            _ => unreachable!("a modulator produces baseband"),
        };
        // The timings at the clock's rate, to the sample: the modulator
        // rounds each mark and gap on its own, so a whole transmission may
        // land a sample either side of its microseconds.
        assert!((iq.len() as f64 - air * rate).abs() <= 1.0, "{} samples", iq.len());

        let mut rx = PocsagNode::default();
        rx.negotiate(&spec(rate, center)).unwrap();
        let quiet = vec![common::C32::new(0.0, 0.0); 32_000];
        let mut frames: Vec<Vec<u8>> = Vec::new();
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            let mut out = Payload::Frames(Vec::new());
            run(&mut rx, &Payload::Iq(block.to_vec()), &mut out, rate, center);
            if let Payload::Frames(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes));
            }
        }

        assert_eq!(frames.len(), 1, "one transmission back off the air");
        let decodes = pocsag_decoded(&frames[0], Hz(center as u64));
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[0].text.as_deref(), Some("MOVE TO CHANNEL 2"));
        let get = |k: &str| decodes[0].fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("address"), Some(common::Value::Int(1_234_568)));
        assert_eq!(get("function"), Some(common::Value::Int(3)));
    }

    /// A numeric page is the same round trip through the other body, which is
    /// the one whose four-bit codes are reversed in the codeword.
    #[test]
    fn a_numeric_page_keeps_its_digits() {
        let mut tx = PocsagTxNode::new(2_000_002, 0, "112233", 512.0);
        tx.format = 1;
        let contents = pocsag::encode(2_000_002, 0, &Body::Numeric("112233".into()));
        let words: Vec<u32> = contents.into_iter().map(dsp::pocsag::encode_codeword).collect();
        let t = Transmission { codewords: words, baud: 512, corrected: 0, lost: 0 };
        let decodes = pocsag_decoded(&t.to_bytes(), Hz(DEFAULT_HZ as u64));
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].text.as_deref(), Some("112233"));
        // 512 baud is five times the air of 2400 for the same page.
        assert_eq!(tx.baud, 512.0);
        let long = air_time_us(&tx.page());
        tx.set_param(BAUD, ParamValue::Float(2400.0)).unwrap();
        assert_eq!(tx.baud, 2400.0);
        assert!((long / air_time_us(&tx.page()) - 4.6875).abs() < 0.01, "512 against 2400 baud");
    }

    /// A rate between two of the three is a transmission no pager can read,
    /// so the stage moves to the nearest one rather than keying it.
    #[test]
    fn a_rate_pocsag_does_not_use_is_pulled_to_the_nearest_one() {
        assert_eq!(nearest_baud(1000.0), 1200.0);
        assert_eq!(nearest_baud(600.0), 512.0);
        assert_eq!(nearest_baud(9600.0), 2400.0);
        let mut n = PocsagTxNode::default();
        // The picker sends an index; a patch carries a rate. Both land here.
        n.set_param(BAUD, ParamValue::Int(2)).unwrap();
        assert_eq!(n.baud, 2400.0);
        n.set_param(BAUD, ParamValue::Float(512.0)).unwrap();
        assert_eq!(n.baud, 512.0);
    }

    /// A transmitter empties its queue in one go, so one transmission is
    /// several pages to several pagers and each is a row of its own.
    #[test]
    fn one_transmission_becomes_a_row_for_each_page() {
        let mut contents = pocsag::encode(1_000_001, 3, &Body::Alpha("FIRST".into()));
        contents.extend(pocsag::encode(2_000_002, 0, &Body::Numeric("112".into())));
        let words: Vec<u32> = contents.into_iter().map(dsp::pocsag::encode_codeword).collect();
        let t = Transmission { codewords: words, baud: 1200, corrected: 0, lost: 0 };

        let decodes = pocsag_decoded(&t.to_bytes(), Hz(DEFAULT_HZ as u64));
        assert_eq!(decodes.len(), 2);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[1].protocol, "POCSAG-Numeric");
        assert_eq!(decodes[1].text.as_deref(), Some("112"));
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

/// What the transmit side is set with.
const ADDRESS: &str = "address";
const FUNCTION: &str = "function";
const FORMAT: &str = "format";
const MESSAGE: &str = "message";
const BAUD: &str = "baud";
const PAUSE_MS: &str = "pause_ms";

pub const DESC: StageDesc = StageDesc {
    name: "pocsag",
    summary: "One pager channel: narrowband FM and POCSAG at 512 to 2400 baud",
    category: Category::Decode,
    feeds_bus: true,
};

pub const POCSAG_TX: StageDesc = StageDesc {
    name: "pocsag_tx",
    summary: "Page a pager: an address, a message and the batches around them",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(PocsagNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = PocsagTxNode::new(
        s.i64_or(ADDRESS, 1_234_567) as u32,
        s.i64_or(FUNCTION, 3) as u8,
        s.str_or(MESSAGE, ""),
        s.f64_or(BAUD, 1200.0),
    );
    n.format = s.i64_or(FORMAT, 0).clamp(0, 2) as usize;
    n.pace.set_pause_ms(s.f64_or(PAUSE_MS, crate::tx_source::DEFAULT_PAUSE_MS));
    Ok(Box::new(n))
}
