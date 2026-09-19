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
use common::Result;
pub use decode::pocsag::decoded;
use decode::pocsag::{self, Body};
use dsp::pocsag::{DEVIATION_HZ, PocsagConfig, PocsagDemod, Transmission};
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::pocsag::AUDIO_HZ;
pub use identify::pocsag::CHANNEL_WIDTH_HZ;
pub use identify::pocsag::Pocsag;
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// A common UK and European paging channel, and only the default the node is
/// built with before the scanner table tells it where to listen.
pub const DEFAULT_HZ: f64 = 153_350_000.0;

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

/// A page, keyed as the bits of a whole transmission.
///
/// The transmit mirror of [`PocsagNode`]: preamble, sync words, the address
/// codeword in the frame its address demands and the message codewords after
/// it, all from [`decode::pocsag::encode`] and [`dsp::pocsag::encode_bits`],
/// which are the same two layers the receiver reads back. What leaves is
/// timings, and [`crate::mod_nodes::FskModNode`] puts them on a carrier.
///
/// A mark is the upper tone, and POCSAG sends a binary zero as the positive
/// deviation, so the bits are inverted on their way to the keyer. A receiver
/// reads the transmission either way up, but a pager does not.
pub struct PocsagTxNode {
    address: u32,
    function: u8,
    message: String,
    keyer: crate::tx_nodes::Keyer,
    rate: f64,
}

impl Default for PocsagTxNode {
    fn default() -> Self {
        Self::new(1_234_567, 3, "", 1200.0)
    }
}

impl PocsagTxNode {
    pub fn new(address: u32, function: u8, message: &str, baud: f64) -> Self {
        // Idle between passes: a second of silence at every speed, so a
        // receiver hears one transmission end before the next preamble
        // starts rather than reading two as one.
        let mut n = Self {
            address,
            function: function & 3,
            message: message.into(),
            keyer: crate::tx_nodes::Keyer::new(baud, baud),
            rate: 0.0,
        };
        n.reload();
        n
    }

    /// Encode what the settings now say, from the top.
    fn reload(&mut self) {
        let body = match self.message.is_empty() {
            true => Body::Tone,
            // A numeric pager shows only digits, and nothing on the air says
            // which kind the address belongs to, so text is sent as text.
            false => Body::Alpha(self.message.clone()),
        };
        let contents = pocsag::encode(self.address, self.function, &body);
        let bits: Vec<bool> = dsp::pocsag::encode_bits(&contents).iter().map(|&b| !b).collect();
        self.keyer.load(bits);
    }
}

impl Simple for PocsagTxNode {
    fn name(&self) -> &str {
        POCSAG_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = vec![
            ("paging".into(), self.address.to_string()),
            ("baud".into(), format!("{:.0}", self.keyer.baud())),
        ];
        if self.keyer.passes() > 0 {
            out.push(("sent".into(), self.keyer.passes().to_string()));
        }
        out
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("pocsag_tx needs a clock to key against"));
        }
        self.rate = i.spec.rate;
        Ok(StreamSpec {
            kind: PortKind::Pulses,
            // The timings are microseconds and the port has no rate of its
            // own; the clock is passed on for the modulator behind. See
            // `MorseKeyNode`.
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
        o.pulses_mut().extend(self.keyer.take(i.len(), self.rate));
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            // 21 bits of address, of which the low three are the frame the
            // address codeword must sit in.
            Param::int(ADDRESS, i64::from(self.address), 0..=2_097_151).label("Address"),
            Param::int(FUNCTION, i64::from(self.function), 0..=3).label("Function"),
            Param::text(MESSAGE, self.message.clone()).label("Message"),
            Param::choice(
                BAUD_PARAM,
                dsp::pocsag::BAUDS.iter().position(|&b| b == self.keyer.baud()).unwrap_or(1),
                dsp::pocsag::BAUDS.iter().map(|b| format!("{b:.0}")).collect(),
            )
            .label("Speed")
            .unit("baud"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        match name {
            ADDRESS => self.address = value.as_i64().unwrap_or(0).clamp(0, 2_097_151) as u32,
            FUNCTION => self.function = value.as_i64().unwrap_or(3).clamp(0, 3) as u8,
            MESSAGE => self.message = value.as_str().unwrap_or_default().to_string(),
            BAUD_PARAM => {
                let i = value.as_i64().unwrap_or(1).clamp(0, 2) as usize;
                self.keyer.set_baud(dsp::pocsag::BAUDS[i]);
            }
            _ => {
                return Err(common::Error::other(format!("pocsag_tx: unknown parameter {name:?}")));
            }
        }
        self.reload();
        Ok(())
    }
}

impl Protocol for Pocsag {
    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
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
        Some(decoded(bytes, common::Hz(p.center_hz())))
    }

    /// The amateur DAPNET channel: amateur rather than commercial because
    /// it is the one paging frequency that is the same across Europe.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} pager", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "POCSAG".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// The page, then the deviation a pager expects: 4.5 kHz either side of
    /// the carrier is a 9 kHz separation between the tones.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(POCSAG_TX.name),
            modulator: NodeSpec::new(crate::mod_nodes::FSK_MOD.name)
                .f("shift_hz", DEVIATION_HZ * 2.0)
                .f("offset_hz", 0.0),
        })
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "pocsag",
    summary: "One pager channel: narrowband FM and POCSAG at 512 to 2400 baud",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(PocsagNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

/// What the page says, and who it is for.
const ADDRESS: &str = "address";
const FUNCTION: &str = "function";
const MESSAGE: &str = "message";
const BAUD_PARAM: &str = "baud";

pub const POCSAG_TX: StageDesc = StageDesc {
    name: "pocsag_tx",
    summary: "Key a page as POCSAG: preamble, batches and the address codeword",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(PocsagTxNode::new(
        s.f64_or(ADDRESS, 1_234_567.0) as u32,
        s.f64_or(FUNCTION, 3.0) as u8,
        s.str_or(MESSAGE, ""),
        s.f64_or(BAUD_PARAM, 1200.0),
    )))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
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
        let decodes = decoded(&frames[0], Hz(center as u64));
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[0].text.as_deref(), Some("MOVE TO CHANNEL 2"));
        assert_eq!(decodes[0].media_type, pipeline::event::media::TEXT);
        assert!(decodes[0].written, "a page is written to whoever carries the pager");
        assert_eq!(decodes[0].crc_ok, Some(true));
        let get = |k: &str| decodes[0].fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("address"), Some(common::Value::Int(1_234_568)));
    }

    /// The transmitter into the receiver: a page keyed by the chain the
    /// protocol declares, read back off the air by the node that reads real
    /// pagers.
    ///
    /// The round trip is the test that matters. The encoder and the decoder
    /// share the codeword tables, so either could be self-consistently
    /// wrong; what this pins is that the deviation, the bit polarity and the
    /// pacing agree with a receiver that has read pagers off the air.
    #[test]
    fn a_page_keyed_by_the_transmit_chain_is_read_back() {
        let (rate, center) = (240_000.0, Hz(DEFAULT_HZ as u64));
        // One pass at 1200 baud is 1120 bits, which is 0.93 s, so a second
        // and a half holds one whole transmission and part of the next.
        let air = crate::tx_nodes::transmit_for(
            &Pocsag,
            rate,
            center,
            1.5,
            &[(MESSAGE, ParamValue::Text("WAVESHARK".into()))],
        );
        // 88 blocks of 4096 samples, less the 48 owed to the keyer when the
        // clock ran out part way through a bit and the rounding of each
        // pulse's edges onto a sample.
        assert_eq!(air.len(), 360_400, "1.5 s of samples at {rate}");

        let mut node = PocsagNode::new(DEFAULT_HZ);
        node.negotiate(&spec(rate, DEFAULT_HZ)).unwrap();
        let ins = [spec(rate, DEFAULT_HZ)];
        let tags = Vec::new();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); 40_000];
        for block in [&air[..], &quiet[..]] {
            for chunk in block.chunks(8_192) {
                let input = Payload::Iq(chunk.to_vec());
                let mut out = Payload::Frames(Vec::new());
                let (mut events, mut new_tags) = (Vec::new(), Vec::new());
                let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
                node.process(&input, &mut out, &mut ctx).unwrap();
                if let Payload::Frames(f) = out {
                    frames.extend(f.into_iter().map(|x| x.bytes));
                }
            }
        }

        assert_eq!(frames.len(), 1, "one whole transmission in a second and a half");
        let decodes = decoded(&frames[0], Hz(DEFAULT_HZ as u64));
        assert_eq!(decodes.len(), 1);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[0].text.as_deref(), Some("WAVESHARK"));
        let get = |k: &str| decodes[0].fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("address"), Some(common::Value::Int(1_234_567)));
        assert_eq!(get("function"), Some(common::Value::Int(3)));
    }

    /// Every speed the receiver searches is a speed it can be keyed at, and
    /// the bits leave at the rate the clock says rather than as fast as the
    /// blocks arrive.
    #[test]
    fn each_speed_keys_its_own_number_of_bits_a_second() {
        for baud in dsp::pocsag::BAUDS {
            let mut n = PocsagTxNode::new(1_000_001, 0, "X", baud);
            let rate = 240_000.0;
            n.negotiate(&PortSpec {
                spec: StreamSpec { kind: PortKind::Real, rate, ..Default::default() },
                latency: 0,
            })
            .unwrap();
            let ins = [spec(rate, DEFAULT_HZ)];
            let tags = Vec::new();
            let mut us = 0u64;
            for _ in 0..(rate as usize / 4_096) {
                let mut out = Payload::empty_of(PortKind::Pulses);
                let (mut events, mut new_tags) = (Vec::new(), Vec::new());
                let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
                Simple::process(&mut n, &Payload::Real(vec![0.0; 4_096]), &mut out, &mut ctx)
                    .unwrap();
                for p in out.as_pulses().unwrap_or(&[]) {
                    us +=
                        p.pulses.iter().map(|x| u64::from(x.mark) + u64::from(x.gap)).sum::<u64>();
                }
            }
            // 58 blocks of 4096 is 0.9899 s, and the last partial bit is
            // owed rather than sent, so the timings are a bit short of it.
            let seconds = us as f64 / 1e6;
            assert!(
                (0.985..=0.990).contains(&seconds),
                "{baud} baud keyed {seconds:.4} s of timings in 0.9899 s"
            );
        }
    }

    /// A transmitter empties its queue in one go, so one transmission is
    /// several pages to several pagers and each is a row of its own.
    #[test]
    fn one_transmission_becomes_a_row_for_each_page() {
        let mut contents = pocsag::encode(1_000_001, 3, &Body::Alpha("FIRST".into()));
        contents.extend(pocsag::encode(2_000_002, 0, &Body::Numeric("112".into())));
        let words: Vec<u32> = contents.into_iter().map(dsp::pocsag::encode_codeword).collect();
        let t = Transmission { codewords: words, baud: 1200, corrected: 0, lost: 0 };

        let decodes = decoded(&t.to_bytes(), Hz(DEFAULT_HZ as u64));
        assert_eq!(decodes.len(), 2);
        assert_eq!(decodes[0].protocol, "POCSAG-Alpha");
        assert_eq!(decodes[1].protocol, "POCSAG-Numeric");
        assert_eq!(decodes[1].text.as_deref(), Some("112"));
    }
}
