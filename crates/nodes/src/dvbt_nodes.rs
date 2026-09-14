//! DVB-T as a graph node: an 8 MHz television multiplex, from the air to
//! transport packets.
//!
//! The layers are elsewhere, which is the point of the layering. The OFDM
//! symbol, its pilots and its transmission parameters are `dsp::dvbt`; the
//! convolutional code is `dsp::conv`; the interleavers, Reed-Solomon and the
//! randomiser are `decode::dvbt`. What this module does is put them in order
//! and hand the result to the graph.
//!
//! One thing here is not just wiring. The outer coding is aligned to the
//! super frame: four frames of sixty-eight symbols carry a whole number of
//! transport packets, whatever the constellation and code rate, so a receiver
//! that starts the Viterbi on symbol zero of frame zero has the puncturing in
//! phase and the bytes on their boundaries for nothing. That is why the TPS
//! has to be read before a single packet can come out, and why nothing is
//! read for the first sixty-eight symbols.

use crate::NodeSpec;
use crate::protocol::{Placed, Placement, Protocol, Shape};
use common::{C32, Result};
use decode::dvbt::{Outer, OuterTx, TsPacket};
use decode::mpeg2;
use decode::mpegts::Mux;
use dsp::conv;
use dsp::dvbt::{self, Inner, Mode, Params, Symbol};
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The rate the receiver hands the front end, which is the standard's own.
pub const RATE_HZ: f64 = dvbt::RATE_HZ;
/// The channel a multiplex owns.
pub const CHANNEL_WIDTH_HZ: f64 = dvbt::CHANNEL_WIDTH_HZ;

/// A whole DVB-T receiver: samples in, transport packets out.
pub struct DvbtReceiver {
    front: dvbt::Dvbt,
    inner: Option<Inner>,
    viterbi: conv::Viterbi,
    outer: Outer,
    params: Option<Params>,
    /// Whether the super frame boundary has been seen and the inner decoder
    /// started on it.
    started: bool,
    symbols: Vec<Symbol>,
    soft: Vec<f32>,
    bits: Vec<u8>,
    bytes: Vec<u8>,
    /// Bits of a byte not yet whole.
    partial: (u8, u8),
}

impl Default for DvbtReceiver {
    fn default() -> Self {
        Self::new()
    }
}

impl DvbtReceiver {
    pub fn new() -> Self {
        Self {
            front: dvbt::Dvbt::new(),
            inner: None,
            viterbi: conv::Viterbi::default(),
            outer: Outer::new(),
            params: None,
            started: false,
            symbols: Vec::new(),
            soft: Vec::new(),
            bits: Vec::new(),
            bytes: Vec::new(),
            partial: (0, 0),
        }
    }

    /// The multiplex's parameters, once the TPS has said what they are.
    pub fn params(&self) -> Option<Params> {
        self.params
    }

    /// The mode and guard, which are known before the parameters are.
    pub fn mode_guard(&self) -> Option<(Mode, dvbt::Guard)> {
        self.front.mode_guard()
    }

    /// How the outer code is faring.
    pub fn stats(&self) -> decode::dvbt::Stats {
        self.outer.stats
    }

    /// Signal to noise on the pilots of the last symbol read.
    pub fn snr_db(&self) -> Option<f32> {
        self.symbols.last().map(|s| s.snr_db)
    }

    /// Read what `iq` holds, appending every transport packet it completes.
    pub fn push(&mut self, iq: &[C32], out: &mut Vec<TsPacket>) {
        self.symbols.clear();
        let mut symbols = std::mem::take(&mut self.symbols);
        self.front.push(iq, &mut symbols);
        for symbol in &symbols {
            self.symbol(symbol, out);
        }
        self.symbols = symbols;
    }

    fn symbol(&mut self, symbol: &Symbol, out: &mut Vec<TsPacket>) {
        let Some(params) = self.front.params() else { return };
        if self.params != Some(params) {
            // A different multiplex, or the first word read: everything below
            // the carriers is about to change shape.
            self.params = Some(params);
            self.inner = Some(Inner::new(params.mode, params.constellation));
            self.started = false;
        }
        let (Some(index), Some(frame)) = (symbol.index, symbol.frame) else { return };
        if !self.started {
            if frame != 0 || index != 0 {
                return;
            }
            self.started = true;
            self.viterbi.reset();
            self.outer.reset();
            self.partial = (0, 0);
        }
        let Some(inner) = &mut self.inner else { return };

        self.soft.clear();
        inner.demodulate(&symbol.cells, &symbol.csi, index, &mut self.soft);
        self.bits.clear();
        let rate = params.code_rate_hp;
        self.viterbi.push(&self.soft, rate.mask(), &mut self.bits);

        self.bytes.clear();
        let (mut acc, mut have) = self.partial;
        for &bit in &self.bits {
            acc = (acc << 1) | bit;
            have += 1;
            if have == 8 {
                self.bytes.push(acc);
                acc = 0;
                have = 0;
            }
        }
        self.partial = (acc, have);
        self.outer.push(&self.bytes, out);
    }
}

/// A whole DVB-T transmitter: transport packets in, samples out.
///
/// It is here so the receiver can be tested against a multiplex whose every
/// packet is known, and it is the same order of stages a transmit chain would
/// draw.
pub struct DvbtModulator {
    params: Params,
    outer: OuterTx,
    encoder: conv::Encoder,
    inner: Inner,
    ofdm: dvbt::tx::Modulator,
    /// Coded bits waiting for a symbol to be full.
    coded: Vec<u8>,
    /// Information bits waiting for a puncturing period to be full.
    pending: Vec<u8>,
    cells: Vec<C32>,
}

impl DvbtModulator {
    pub fn new(params: Params) -> Self {
        Self {
            outer: OuterTx::new(),
            encoder: conv::Encoder::new(),
            inner: Inner::new(params.mode, params.constellation),
            ofdm: dvbt::tx::Modulator::new(params),
            coded: Vec::new(),
            pending: Vec::new(),
            cells: Vec::new(),
            params,
        }
    }

    /// One transport packet onto the air. Samples appear once enough packets
    /// have gone in to fill a symbol.
    pub fn push(&mut self, packet: &[u8], out: &mut Vec<C32>) {
        let mut bytes = Vec::new();
        self.outer.push(packet, &mut bytes);
        for byte in bytes {
            for i in (0..8).rev() {
                self.pending.push((byte >> i) & 1);
            }
        }
        let mask = self.params.code_rate_hp.mask();
        let whole = self.pending.len() - self.pending.len() % (mask.len() / 2);
        let sent = self.encoder.punctured(&self.pending[..whole], mask, conv::First::X);
        self.coded.extend_from_slice(&sent);
        self.pending.drain(..whole);
        let per_symbol = self.inner.bits_per_symbol();
        while self.coded.len() >= per_symbol {
            let index = self.ofdm.symbol_index();
            self.cells.clear();
            self.inner.modulate(&self.coded[..per_symbol], index, &mut self.cells);
            self.ofdm.modulate(&self.cells, out);
            self.coded.drain(..per_symbol);
        }
    }
}

/// The UHF television band as Europe allocates it now, and the remains of
/// band III. What is in them is one 8 MHz multiplex per channel.
fn bands() -> Vec<(f64, f64)> {
    vec![(174_000_000.0, 230_000_000.0), (470_000_000.0, 694_000_000.0)]
}

/// One 8 MHz multiplex as a stage: samples in, its transport stream out, and
/// what it says about itself on the bus.
pub struct DvbtNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    resample: Rational,
    rx: DvbtReceiver,
    mux: Mux,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    at_rate: Vec<C32>,
    packets: Vec<TsPacket>,
    /// What has already been said on the bus, so a multiplex that repeats its
    /// tables every half second does not repeat itself on the packet list.
    told: Option<Params>,
    named: Vec<u16>,
    at: f64,
    /// The video decoder, and which stream it is being fed.
    video: mpeg2::Decoder,
    watching: Option<u16>,
    /// The service an operator asked for, or none to take the first one the
    /// multiplex describes that has a picture in it.
    wanted: Option<u16>,
    pictures: Vec<mpeg2::Picture>,
    sequence: u64,
}

impl Default for DvbtNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl DvbtNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(RATE_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            resample: Rational::with_ratio(1, 1),
            rx: DvbtReceiver::new(),
            mux: Mux::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            at_rate: Vec::new(),
            packets: Vec::new(),
            told: None,
            named: Vec::new(),
            at: 0.0,
            video: mpeg2::Decoder::new(),
            watching: None,
            wanted: None,
            pictures: Vec::new(),
            sequence: 0,
        }
    }

    /// Watch one service by its identifier, or none to take whichever the
    /// multiplex describes first.
    pub fn watch(&mut self, service: Option<u16>) {
        if self.wanted != service {
            self.wanted = service;
            self.watching = None;
            self.video = mpeg2::Decoder::new();
        }
    }

    /// The service being watched, and the packet identifier its pictures are
    /// on, once the tables have named one.
    pub fn watching(&self) -> Option<u16> {
        self.watching
    }

    /// Point the demux at the video of whichever service is wanted, as soon
    /// as the programme map names it.
    fn follow_video(&mut self) {
        if self.watching.is_some() {
            return;
        }
        let service = match self.wanted {
            Some(id) => self.mux.service(id).cloned(),
            None => self.mux.services.iter().find(|s| s.video().is_some()).cloned(),
        };
        let Some(pid) = service.as_ref().and_then(|s| s.video()).map(|v| v.pid) else {
            return;
        };
        self.mux.follow(pid);
        self.watching = Some(pid);
    }

    /// Decode whatever picture is still held back, for a recording that has
    /// run out rather than a transmission that has stopped.
    ///
    /// A picture ends where the next one starts, so the last picture of a
    /// stream is still inside the decoder when the samples run out. On the
    /// air the next picture is forty milliseconds away and nothing needs
    /// this; at the end of a capture it is the difference between a picture
    /// and none.
    pub fn flush(&mut self, out: &mut Vec<common::VideoFrame>) {
        let mut pictures = std::mem::take(&mut self.pictures);
        pictures.clear();
        self.video.flush(&mut pictures);
        for p in &pictures {
            let frame = self.frame(p);
            out.push(frame);
        }
        self.pictures = pictures;
    }

    /// A decoded picture as the video bus carries it.
    fn frame(&mut self, p: &mpeg2::Picture) -> common::VideoFrame {
        self.sequence += 1;
        let name = self
            .mux
            .services
            .iter()
            .find(|s| s.video().is_some_and(|v| Some(v.pid) == self.watching))
            .and_then(|s| s.name.clone());
        common::VideoFrame {
            system: DVB,
            channel_hz: self.channel_hz,
            label: name,
            width: p.width,
            height: p.height,
            // Broadcast pictures are 16:9 and their samples are square at
            // this size, so the grid is the shape.
            aspect: p.width as f32 / p.height as f32,
            pixels: common::Pixels::Rgb8,
            samples: std::sync::Arc::new(p.rgb()),
            lines_seen: p.height,
            sequence: self.sequence,
            update: common::Update::Whole,
            // A picture every half second or so, each superseding the last,
            // which is what an intra picture out of a broadcast is.
            cadence: common::Cadence::Live,
        }
    }

    /// The multiplex as far as its tables have described it, for a pane that
    /// wants to list what is on it.
    pub fn mux(&self) -> &Mux {
        &self.mux
    }

    /// The multiplex itself, once the TPS has said what it is.
    fn announce(&mut self, params: Params, c: &mut NodeCtx<'_>) {
        let snr = self.rx.snr_db().unwrap_or(0.0);
        let mut fields = vec![
            ("mode".into(), common::Value::Text(params.mode.label().into())),
            ("guard".into(), common::Value::Text(params.guard.label().into())),
            ("constellation".into(), common::Value::Text(params.constellation.label().into())),
            ("code_rate".into(), common::Value::Text(params.code_rate_hp.label().into())),
            ("bitrate".into(), common::Value::Float(params.bitrate())),
            ("snr_db".into(), common::Value::Float(snr as f64)),
        ];
        if let Some(cell) = params.cell_id {
            fields.push(("cell_id".into(), common::Value::Text(format!("{cell:04X}"))));
        }
        let detail = format!("{} {:.1} Mbit/s", params.label(), params.bitrate() / 1e6);
        c.emit(pipeline::event::Event::Decoded(
            Decoded::bytes("DVB-T", common::Hz(self.channel_hz as u64), self.at, Vec::new())
                .with_detail(detail)
                .with_fields(fields)
                .with_modulation(common::Modulation::Ofdm)
                .with_crc(Some(true)),
        ));
    }

    /// A service, once the description table has named it.
    fn announce_service(&mut self, id: u16, c: &mut NodeCtx<'_>) {
        let Some(service) = self.mux.service(id) else { return };
        let Some(name) = service.name.clone() else { return };
        let mut fields = vec![
            ("service".into(), common::Value::Text(name.clone())),
            ("service_id".into(), common::Value::Int(id as i64)),
        ];
        if let Some(p) = &service.provider {
            fields.push(("provider".into(), common::Value::Text(p.clone())));
        }
        if let Some(v) = service.video() {
            fields.push(("video".into(), common::Value::Text(v.kind.label().into())));
        }
        if let Some(a) = service.audio() {
            fields.push(("audio".into(), common::Value::Text(a.kind.label().into())));
        }
        if service.scrambled {
            fields.push(("scrambled".into(), common::Value::Text("yes".into())));
        }
        let detail = match &service.provider {
            Some(p) => format!("{name} ({p})"),
            None => name.clone(),
        };
        // A service keeps its identity across multiplexes and retunes, which
        // is what the device list rows on.
        let who = common::Identity::new("dvb-service", format!("{id}")).named(name);
        c.emit(pipeline::event::Event::Decoded(
            Decoded::bytes("DVB-T", common::Hz(self.channel_hz as u64), self.at, Vec::new())
                .by(who)
                .with_detail(detail)
                .with_fields(fields)
                .with_modulation(common::Modulation::Ofdm)
                .with_crc(Some(true)),
        ));
    }
}

/// What the video bus calls a picture off the television multiplex.
pub const DVB: &str = "DVB-T";

impl pipeline::node::Node for DvbtNode {
    fn name(&self) -> &str {
        "dvbt"
    }

    fn num_inputs(&self) -> usize {
        1
    }

    /// The transport stream, and the pictures read out of it.
    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dvbt reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < RATE_HZ {
            return Err(common::Error::other("dvbt needs 9.14 MS/s of channel"));
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("dvbt needs its channel inside the span"));
        }
        // 64/7 megasamples a second is not a whole number of hertz, so this
        // asks for the nearest rational rather than an exact one: decimate as
        // far as whole samples allow, then resample what is left.
        let factor = (rate / RATE_HZ).floor().max(1.0) as usize;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.resample = Rational::approx(rate / factor as f64, RATE_HZ, 4096);
        self.rx = DvbtReceiver::new();
        self.mux = Mux::new();
        self.told = None;
        self.named.clear();

        let mut out = i.spec.with_kind(PortKind::Bytes);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        // The stream this puts out is transport packets, at whatever rate the
        // multiplex carries them.
        out.rate = self.rx.params().map(|p| p.bitrate() / 8.0).unwrap_or(RATE_HZ);
        let mut video = out.with_kind(PortKind::Video);
        // A picture is not a sampled stream: it carries its own geometry and
        // arrives when the multiplex sends one.
        video.rate = 0.0;
        Ok(vec![out, video])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
        self.at = c.timestamp();
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.at_rate.clear();
        self.resample.process(&self.narrow, &mut self.at_rate);

        self.packets.clear();
        let mut packets = std::mem::take(&mut self.packets);
        self.rx.push(&self.at_rate, &mut packets);
        let out = outputs[0].bytes_mut();
        for p in &packets {
            self.mux.push(&p.bytes);
            out.extend_from_slice(&p.bytes);
        }
        self.packets = packets;
        self.follow_video();

        // The pictures, where the demux is pointed at a service's video.
        let mut pictures = std::mem::take(&mut self.pictures);
        pictures.clear();
        for pes in self.mux.take_pes() {
            if Some(pes.pid) == self.watching {
                self.video.push(&pes.data, &mut pictures);
            }
        }
        for p in &pictures {
            let frame = self.frame(p);
            outputs[1].video_mut().push(frame);
        }
        self.pictures = pictures;

        if let Some(params) = self.rx.params() {
            if self.told != Some(params) {
                self.told = Some(params);
                self.announce(params, c);
            }
        }
        let fresh: Vec<u16> = self
            .mux
            .services
            .iter()
            .filter(|s| s.name.is_some() && !self.named.contains(&s.id))
            .map(|s| s.id)
            .collect();
        for id in fresh {
            self.named.push(id);
            self.announce_service(id, c);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.rx = DvbtReceiver::new();
        self.mux = Mux::new();
        self.video = mpeg2::Decoder::new();
        self.watching = None;
        self.told = None;
        self.named.clear();
    }
}

/// The middle of the first UK multiplex channel, which is as good a place to
/// start as any: channel 21, 474 MHz.
pub const DEFAULT_HZ: f64 = 474_000_000.0;

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub struct Dvbt;

impl Protocol for Dvbt {
    fn id(&self) -> &'static str {
        "dvbt"
    }
    fn label(&self) -> &'static str {
        "dvbt"
    }
    fn placement(&self) -> Placement {
        Placement::Bands(bands())
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: RATE_HZ,
            feed_rate_hz: RATE_HZ,
            span_wide: false,
            // A multiplex is on the air without stopping, so there is no
            // burst for the classifier to name and nothing to wait for.
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.0} DVB-T", hz / 1e6)
    }
    /// A transport stream and the pictures in it. The stream is 24 megabits
    /// a second of packets, which is for a stage above to read rather than a
    /// list for a person; the pictures go to the video pane.
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Bytes, PortKind::Video]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "dvbt",
    summary: "One DVB-T multiplex: OFDM, the inner and outer codes, its transport stream",
    category: Category::Decode,
    // What it puts on its port is the transport stream. What reaches the
    // packet list is the multiplex and its services, emitted rather than
    // carried on a wire.
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DvbtNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use dvbt::{CodeRate, Constellation, Guard, Hierarchy};

    /// A transport packet that says which packet it is, twice: in the
    /// continuity counter and in every payload byte.
    pub(super) fn packet(n: u16) -> [u8; 188] {
        let mut p = [0u8; 188];
        p[0] = 0x47;
        p[1] = 0x01;
        p[2] = 0x23;
        p[3] = 0x10 | (n % 16) as u8;
        for (i, b) in p[4..].iter_mut().enumerate() {
            *b = (i as u16).wrapping_mul(7).wrapping_add(n) as u8;
        }
        p
    }

    /// The multiplex with a transport stream in it, at the standard's rate.
    pub(super) fn on_air(params: Params, packets: usize) -> Vec<C32> {
        let mut tx = DvbtModulator::new(params);
        let mut air = Vec::new();
        for n in 0..packets {
            tx.push(&packet(n as u16), &mut air);
        }
        air
    }

    fn noise(seed: u64, sigma: f32, n: usize) -> Vec<C32> {
        let mut state = seed | 1;
        let mut next = || {
            state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
            ((state >> 33) as f32 / (1u32 << 31) as f32).clamp(1e-7, 1.0 - 1e-7)
        };
        (0..n)
            .map(|_| {
                let (u, v) = (next(), next());
                let r = (-2.0 * u.ln()).sqrt() * sigma / 2f32.sqrt();
                C32::new(
                    r * (std::f32::consts::TAU * v).cos(),
                    r * (std::f32::consts::TAU * v).sin(),
                )
            })
            .collect()
    }

    /// A multiplex built packet by packet, put on the air, read back off it,
    /// and the packets are the packets that went in.
    ///
    /// 2K at 1/32 and QPSK 1/2 because the test has to carry a super frame,
    /// which is 272 symbols: at 64-QAM that is a megabyte of samples per
    /// second of air for no more evidence.
    #[test]
    fn a_multiplex_goes_out_and_comes_back() {
        let params = Params {
            mode: Mode::M2k,
            guard: Guard::G1_32,
            constellation: Constellation::Qpsk,
            hierarchy: Hierarchy::None,
            code_rate_hp: CodeRate::R1_2,
            code_rate_lp: CodeRate::R1_2,
            cell_id: Some(0x2F1A),
        };
        let mut tx = DvbtModulator::new(params);
        let mut air = Vec::new();
        // Three super frames: one to lock the TPS, one to start the inner
        // decoder on, and one to read packets out of.
        let packets = 3 * 504;
        for n in 0..packets {
            tx.push(&packet(n as u16), &mut air);
        }
        assert_eq!(air.len(), 3_446_784, "samples of air, which is 2176 symbols");

        // 30 dB of signal to noise, which QPSK 1/2 reads with room to spare.
        let hiss = noise(0xA5A5, 10f32.powf(-30.0 / 20.0), air.len());
        let samples: Vec<C32> = air.iter().zip(&hiss).map(|(a, n)| a + n).collect();

        let mut rx = DvbtReceiver::new();
        let mut got = Vec::new();
        for block in samples.chunks(8192) {
            rx.push(block, &mut got);
        }

        assert_eq!(rx.params(), Some(params), "the TPS says what the multiplex is");
        // What is lost is the wait for a super frame boundary to start the
        // inner decoder on, the eleven codewords inside the outer
        // interleaver, and the Viterbi's traceback: 268 packets of 1512.
        assert_eq!(got.len(), 1244, "packets read of {packets} sent");
        assert_eq!(rx.stats().corrected, 0, "a clean channel needs no correction");
        assert_eq!(rx.stats().uncorrectable, 0, "a clean channel loses no codeword");

        // The packets are in order and whole: find where the first one read
        // sits in what was sent, and every one after it follows.
        let first = (0..packets)
            .find(|&n| packet(n as u16) == got[0].bytes)
            .expect("a packet that was sent");
        for (i, p) in got.iter().enumerate() {
            assert_eq!(p.bytes, packet((first + i) as u16), "packet {i}");
            assert_eq!(p.pid(), 0x123);
        }
    }
}

#[cfg(test)]
mod node_tests {
    use super::tests::on_air;
    use super::*;
    use common::Hz;
    use dvbt::{CodeRate, Constellation, Guard, Hierarchy};
    use pipeline::node::Node;

    /// Every radio rate the receiver sees reaches 64/7 megasamples a second,
    /// which is not a whole number of hertz and so is approximated.
    #[test]
    fn an_awkward_radio_rate_still_reaches_the_standard_rate() {
        for rate in [10_000_000.0, 12_000_000.0, 16_000_000.0, 20_000_000.0, 61_440_000.0] {
            let mut n = DvbtNode::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(474_000_000)), latency: 0 };
            n.negotiate(&[spec]).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
            let got = rate / (rate / RATE_HZ).floor() * n.resample.ratio();
            let ppm = (got - RATE_HZ) / RATE_HZ * 1e6;
            assert!(ppm.abs() < 1.0, "{rate} lands {ppm} ppm out");
        }
    }

    /// A rate too low for the channel, and a channel outside the span, are
    /// both refused rather than half read.
    #[test]
    fn a_narrow_span_is_refused() {
        let mut n = DvbtNode::new(DEFAULT_HZ);
        let thin = PortSpec { spec: StreamSpec::iq(8_000_000.0, Hz(474_000_000)), latency: 0 };
        assert!(n.negotiate(&[thin]).is_err(), "8 MS/s does not hold the channel");
        let far = PortSpec { spec: StreamSpec::iq(20_000_000.0, Hz(500_000_000)), latency: 0 };
        assert!(n.negotiate(&[far]).is_err(), "the channel is not in the span");
    }

    /// The whole stage: a multiplex two megahertz off the middle of a 20 MS/s
    /// span, mixed down, resampled to a rate that is not a whole number of
    /// hertz, decoded, and its own tables read back off it.
    #[test]
    fn a_multiplex_off_centre_in_a_wide_span_is_read() {
        let params = Params {
            mode: Mode::M2k,
            guard: Guard::G1_32,
            constellation: Constellation::Qpsk,
            hierarchy: Hierarchy::None,
            code_rate_hp: CodeRate::R1_2,
            code_rate_lp: CodeRate::R1_2,
            cell_id: Some(0x2F1A),
        };
        let air = on_air(params, 3 * 504);
        // Up to the radio's rate, then away from the middle of its span.
        let mut up = Rational::approx(RATE_HZ, 20_000_000.0, 4096);
        let mut wide = Vec::new();
        up.process(&air, &mut wide);
        let mut mixer = Mixer::new(-2_000_000.0, 20_000_000.0);
        let mut span = Vec::new();
        mixer.process(&wide, &mut span);

        let mut node = DvbtNode::new(474_000_000.0);
        let spec = PortSpec { spec: StreamSpec::iq(20_000_000.0, Hz(476_000_000)), latency: 0 };
        let out_spec = node.negotiate(&[spec]).expect("a channel in the span");
        assert_eq!(out_spec[0].kind, PortKind::Bytes);
        assert_eq!(out_spec[1].kind, PortKind::Video);
        assert_eq!(out_spec[0].center, Hz(474_000_000));

        let mut events = Vec::new();
        let mut stream = Vec::new();
        for block in span.chunks(65_536) {
            let payload = Payload::Iq(block.to_vec());
            let mut out = [Payload::empty_of(PortKind::Bytes), Payload::empty_of(PortKind::Video)];
            let ins = [spec];
            let tags = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
            stream.extend_from_slice(out[0].as_bytes().unwrap_or(&[]));
        }

        assert_eq!(node.rx.params(), Some(params), "the TPS came through the resampler");
        assert_eq!(stream.len() % 188, 0, "whole transport packets");
        assert!(stream.len() >= 188 * 900, "{} bytes of transport stream", stream.len());
        // Two announcements for the multiplex and none for a service: the
        // parameters are said once when the first TPS word decodes, and again
        // a frame later when the second half of the cell identifier arrives.
        // What was modulated is packets on one PID with no tables in them, so
        // nothing names a service.
        let announcements = events
            .iter()
            .filter(|e| matches!(e, pipeline::event::Event::Decoded(d) if d.protocol == "DVB-T"))
            .count();
        assert_eq!(announcements, 2, "the multiplex is announced on what changed, not per symbol");
    }
}
