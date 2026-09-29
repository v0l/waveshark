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
pub use crate::broadcast::{ANY, SERVICE, Tail, Want, service_label};
use crate::broadcast::{Broadcast, SOUND_RATE_HZ};
use crate::protocol::{Placed, Placement, Protocol, Shape};
use common::{C32, Result};
use decode::dvbt::{self as dvbtdec, OuterTx, TsPacket};
use decode::mpegts::{self, Mux};
use dsp::conv;
use dsp::dvbt::{self, Inner, Mode, Params};
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::dvbt::CHANNEL_WIDTH_HZ;
pub use identify::dvbt::DEFAULT_HZ;
pub use identify::dvbt::Dvbt;
pub use identify::dvbt::RATE_HZ;
pub use identify::dvbt::bands;
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The receiver on a thread of its own.
///
/// Reading a multiplex costs about six tenths of a second for every second
/// of signal: the trellis alone runs at two and a half times real time and
/// nothing above it is free. On the graph's own thread that is six tenths of
/// every block's budget spent before anything else in the receiver has run,
/// which is what a spectrum stuttering while a multiplex decodes looks like.
/// Here it is one core's work beside the graph rather than inside it.
struct Offloaded {
    iq: Option<crossbeam_channel::Sender<Vec<C32>>>,
    /// Buffers coming back to be filled again, so a block is not a fresh
    /// half megabyte every seven milliseconds.
    spare: crossbeam_channel::Receiver<Vec<C32>>,
    give_back: crossbeam_channel::Sender<Vec<C32>>,
    packets: crossbeam_channel::Receiver<Vec<TsPacket>>,
    heard: std::sync::Arc<std::sync::Mutex<Heard>>,
    /// Blocks the thread was too far behind to take. Not silent: a receiver
    /// that cannot keep up says so rather than reporting a bad signal.
    pub dropped: u64,
    thread: Option<common::thread::JoinHandle<()>>,
}

/// What the thread has to say about the multiplex, for the stages that ask
/// between blocks.
#[derive(Clone, Copy, Default)]
struct Heard {
    params: Option<Params>,
    snr_db: Option<f32>,
    locked: bool,
    reading: bool,
    stats: dvbtdec::Stats,
}

/// Blocks of samples waiting to be read. About a tenth of a second at the
/// standard's rate.
const QUEUE: usize = 16;

/// How long the graph will wait on a full queue before giving the block up.
const WAIT: common::time::Duration = common::time::Duration::from_millis(200);

impl Offloaded {
    fn new() -> Self {
        let (iq, work) = crossbeam_channel::bounded::<Vec<C32>>(QUEUE);
        let (give_back, spare) = crossbeam_channel::bounded::<Vec<C32>>(QUEUE + 2);
        let (send, packets) = crossbeam_channel::bounded::<Vec<TsPacket>>(QUEUE);
        let heard = std::sync::Arc::new(std::sync::Mutex::new(Heard::default()));
        let mine = heard.clone();
        let back = give_back.clone();
        let thread = common::thread::Builder::new()
            .name("dvbt".into())
            .spawn(move || {
                let mut rx = dvbtdec::DvbtReceiver::new();
                let mut out = Vec::new();
                let mut reading = false;
                while let Ok(block) = work.recv() {
                    out.clear();
                    let was = rx.stats();
                    rx.push(&block, &mut out);
                    let _ = back.try_send(block);
                    let now = rx.stats();
                    if now.packets > was.packets {
                        reading = true;
                    } else if now.uncorrectable > was.uncorrectable || !rx.synced() {
                        reading = false;
                    }
                    *mine.lock().expect("the reading") = Heard {
                        params: rx.params(),
                        snr_db: rx.snr_db(),
                        locked: rx.locked(),
                        reading,
                        stats: now,
                    };
                    if send.send(std::mem::take(&mut out)).is_err() {
                        return;
                    }
                }
            })
            .ok();
        Self { iq: Some(iq), spare, give_back, packets, heard, dropped: 0, thread }
    }

    /// Hand over a block.
    ///
    /// Waits while the thread is behind, but not for long: a recording read
    /// faster than it can be decoded is held back here, which is what keeps
    /// a replay whole, while a radio that cannot be held back at all loses
    /// the block instead of the samples piling up behind it.
    fn push(&mut self, iq: &[C32]) {
        let mut block = self.spare.try_recv().unwrap_or_default();
        block.clear();
        block.extend_from_slice(iq);
        let Some(tx) = &self.iq else { return };
        if crate::wait::send_within(&tx, block, WAIT).is_err() {
            self.dropped += 1;
        }
    }

    /// Whatever has been decoded since the last call.
    fn take(&mut self, out: &mut Vec<TsPacket>) {
        while let Ok(mut packets) = self.packets.try_recv() {
            out.append(&mut packets);
            let _ = self.give_back.try_send(Vec::new());
        }
    }

    /// Stop feeding it and take everything it has left.
    fn finish(&mut self, out: &mut Vec<TsPacket>) {
        self.iq = None;
        while let Ok(mut packets) = self.packets.recv() {
            out.append(&mut packets);
        }
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }

    fn heard(&self) -> Heard {
        *self.heard.lock().expect("the reading")
    }
}

impl Drop for Offloaded {
    fn drop(&mut self) {
        self.iq = None;
        self.packets = crossbeam_channel::never();
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
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
            encoder: conv::Encoder::new(conv::K7_X_FIRST),
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
        let sent = self.encoder.punctured(&self.pending[..whole], mask);
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

/// One 8 MHz multiplex as a stage: samples in, its transport stream out, and
/// what it says about itself on the bus.
pub struct DvbtNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    resample: Rational,
    rx: Offloaded,
    tv: Broadcast,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    at_rate: Vec<C32>,
    packets: Vec<TsPacket>,
    /// What has already been said on the bus, so a multiplex that repeats its
    /// tables every half second does not repeat itself on the packet list.
    told: Option<Params>,
    at: f64,
}

impl Default for DvbtNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl DvbtNode {
    /// What the TPS says the multiplex being read is, or `None` before it
    /// has locked.
    pub fn heard_params(&self) -> Option<Params> {
        self.rx.heard().params
    }

    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(RATE_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            resample: Rational::with_ratio(1, 1),
            rx: Offloaded::new(),
            tv: Broadcast::new(DVB, channel_hz),
            mixed: Vec::new(),
            narrow: Vec::new(),
            at_rate: Vec::new(),
            packets: Vec::new(),
            told: None,
            at: 0.0,
        }
    }

    pub fn watch(&mut self, service: Option<u16>) {
        self.tv.watch(service);
    }

    pub fn want(&mut self, want: Want) {
        self.tv.want(want);
    }

    pub fn watching(&self) -> Option<u16> {
        self.tv.watching()
    }

    pub fn wanted(&self) -> &Want {
        self.tv.wanted()
    }

    #[cfg(feature = "ffmpeg")]
    pub fn media_fault(&self) -> Option<String> {
        self.tv.media_fault()
    }

    pub fn services(&self) -> &[mpegts::Service] {
        self.tv.services()
    }

    pub fn broadcast(&self) -> &Broadcast {
        &self.tv
    }

    /// Decode whatever picture is still held back, for a recording that has
    /// run out rather than a transmission that has stopped.
    ///
    /// A picture ends where the next one starts, so the last picture of a
    /// stream is still inside the decoder when the samples run out, and a
    /// recording played faster than real time finishes with most of a
    /// second of sound still in flight. On the air the next picture is forty
    /// milliseconds away and nothing needs this; at the end of a capture it
    /// is the difference between a picture and none. What is returned is the
    /// sound that was still held, which has no port left to go to.
    pub fn flush(&mut self, out: &mut Vec<common::VideoFrame>) -> Tail {
        // The receiver is on a thread, so the last blocks of samples are
        // still being read when the recording ends.
        let mut packets = Vec::new();
        self.rx.finish(&mut packets);
        self.tv.flush(&packets, out)
    }

    pub fn mux(&self) -> &Mux {
        self.tv.mux()
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

    /// The transport stream, the pictures read out of it, and their sound.
    fn num_outputs(&self) -> usize {
        3
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dvbt reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        // A part in a million under is still the rate: 64/7 MS/s is not a
        // whole number of hertz, and a recording of it is.
        if rate < RATE_HZ * (1.0 - 1e-6) {
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
        self.rx = Offloaded::new();
        self.tv.retune();
        self.told = None;

        let mut out = i.spec.with_kind(PortKind::Bytes);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        // The stream this puts out is transport packets, at whatever rate the
        // multiplex carries them.
        out.rate = self.rx.heard().params.map(|p| p.bitrate() / 8.0).unwrap_or(RATE_HZ);
        let mut video = out.with_kind(PortKind::Video);
        // A picture is not a sampled stream: it carries its own geometry and
        // arrives when the multiplex sends one.
        video.rate = 0.0;
        // The sound of whichever service is being watched, one channel at
        // the rate the bus mixes at. Audio rather than speech: a broadcast
        // belongs to the channel an operator opened, with that channel's
        // fader, its mute and its meter, and it is not a conversation for
        // the call list.
        let mut sound = out.with_kind(PortKind::Real);
        sound.rate = SOUND_RATE_HZ;
        sound.channels = 1;
        sound.bandwidth = 0.0;
        Ok(vec![out, video, sound])
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
        self.rx.push(&self.at_rate);
        self.rx.take(&mut packets);
        self.tv.push(&packets, outputs[0].bytes_mut());
        self.packets = packets;
        // The pictures. The whole multiplex goes to the container decoder,
        // programmes, codecs, clocks and all, and what comes back is already
        // the programme that was asked for.
        let (_, rest) = outputs.split_at_mut(1);
        let (video, sound) = rest.split_at_mut(1);
        self.tv.play(c.block_seconds, &mut video[0], &mut sound[0]);
        self.tv.publish(c, 1);

        if let Some(params) = self.rx.heard().params
            && self.told != Some(params)
        {
            self.told = Some(params);
            let snr = self.rx.heard().snr_db.unwrap_or(0.0);
            let carrier =
                crate::locked(self.channel_hz as u64, CHANNEL_WIDTH_HZ as u32, &self.narrow, snr);
            c.emit(pipeline::event::Event::Decoded(
                common::packet::Packet::heard(carrier)
                    .keyed(common::packet::Keying::configured(common::Modulation::Ofdm))
                    .decoded(dvbtdec::multiplex_read(params)),
            ));
        }
        for id in self.tv.fresh_services() {
            if let Some(d) = dvbtdec::service_read(self.tv.mux(), id) {
                let carrier = crate::locked(
                    self.channel_hz as u64,
                    CHANNEL_WIDTH_HZ as u32,
                    &self.narrow,
                    self.rx.heard().snr_db.unwrap_or(0.0),
                );
                c.emit(pipeline::event::Event::Decoded(
                    common::packet::Packet::heard(carrier)
                        .keyed(common::packet::Keying::configured(common::Modulation::Ofdm))
                        .decoded(d),
                ));
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.rx = Offloaded::new();
        self.tv.reset();
        self.told = None;
    }

    /// The services, as a choice that grows as the multiplex describes them.
    fn params(&self) -> Vec<pipeline::param::Param> {
        vec![self.tv.param()]
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        let heard = self.rx.heard();
        Some(match (heard.locked, heard.params.is_some(), heard.reading) {
            (false, ..) => pipeline::Acquisition::Searching,
            (true, true, true) => pipeline::Acquisition::Locked,
            (true, ..) => pipeline::Acquisition::Acquiring,
        })
    }

    fn readings(&self) -> Vec<(String, String)> {
        let heard = self.rx.heard();
        let mut out = Vec::new();
        if let Some(p) = heard.params {
            out.push(("multiplex".into(), p.label()));
        }
        if let Some(snr) = heard.snr_db.filter(|_| heard.locked) {
            out.push(("pilots".into(), format!("{snr:.1} dB")));
        }
        if heard.stats.packets + heard.stats.uncorrectable > 0 {
            out.push(("packets".into(), heard.stats.packets.to_string()));
            out.push(("uncorrectable".into(), heard.stats.uncorrectable.to_string()));
        }
        if self.rx.dropped > 0 {
            out.push(("blocks lost".into(), self.rx.dropped.to_string()));
        }
        out
    }

    /// Set by position in the list this node just published, by service
    /// identifier, or by name. A position is what a menu sends and an
    /// identifier is what survives the list growing, so both are taken and
    /// neither is guessed at: zero as a position is the first entry, zero as
    /// an identifier is no service at all.
    fn set_param(&mut self, name: &str, v: pipeline::ParamValue) -> Result<()> {
        if name != SERVICE {
            return Err(common::Error::other(format!("dvbt: unknown parameter {name:?}")));
        }
        self.tv.set_service("dvbt", v)
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

impl Protocol for Dvbt {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::Continuously
    }

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

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.0} DVB-T", hz / 1e6)
    }
    /// A transport stream, the pictures in it and their sound. The stream is
    /// 24 megabits a second of packets, which is for a stage above to read
    /// rather than a list for a person; the pictures go to the video pane and
    /// the sound to the audio bus.
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Bytes, PortKind::Video, PortKind::Real]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        // The source's bit rate and the modulator's parameters are the same
        // multiplex described twice, so both come from one `Params`: a
        // source feeding a rate the modulation cannot carry is a multiplex
        // that stuffs or backs up.
        let params = Params::typical();
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(TS_SOURCE.name).f(BITRATE, params.bitrate()),
            modulator: NodeSpec::new(DVBT_MOD.name)
                .f(MODE, mode_choice(params.mode) as f64)
                .f(GUARD, guard_choice(params.guard) as f64)
                .f(CONSTELLATION, constellation_choice(params.constellation) as f64)
                .f(CODE_RATE, code_choice(params.code_rate_hp) as f64),
            sends: crate::protocol::Sends::File(PATH),
        })
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
    let mut node = DvbtNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ));
    // What was being watched before the rebuild. Nothing is checked here:
    // the tables have not arrived yet, so a name is taken on trust and
    // matched when the service turns up.
    node.want(Want::from_settings(s));
    Ok(Box::new(node))
}

/// A transport stream off the disk, clocked by the radio.
///
/// The multiplex's bit rate is set by the modulation, not by the file, so
/// this hands over exactly that many bytes a second and no more: a file
/// recorded at a different rate plays fast or slow rather than running the
/// modulator dry or backing it up. A file that runs out starts again, which
/// is what a test card does.
///
/// Anything that is not already a transport stream is re-encoded into one at
/// the multiplex's rate, so a film in a `.mkv` or a clip off a phone can be
/// transmitted without being converted first.
///
/// With no file it puts out null packets, so a chain built before anybody
/// has chosen anything transmits a real, empty multiplex rather than
/// nothing.
pub struct TsSourceNode {
    path: String,
    bitrate: f64,
    /// The radio's rate, which is what turns a block of samples into the
    /// length of time this has to fill.
    rate: f64,
    file: Option<Source>,
    /// Bytes owed from the last block, kept as a fraction so a byte rate
    /// that does not divide the block size still comes out right.
    owed: f64,
    packet: Vec<u8>,
    loops: u64,
}

impl Default for TsSourceNode {
    fn default() -> Self {
        Self {
            path: String::new(),
            bitrate: Params::typical().bitrate(),
            rate: 0.0,
            file: None,
            owed: 0.0,
            packet: vec![0u8; mpegts::PACKET],
            loops: 0,
        }
    }
}

impl TsSourceNode {
    pub fn new(path: &str, bitrate: f64) -> Self {
        let mut n = Self { path: path.into(), ..Default::default() };
        n.bitrate = bitrate.max(1.0);
        n.open();
        n
    }

    fn open(&mut self) {
        self.file = match self.path.is_empty() {
            // Nothing chosen: colour bars and a tone rather than an empty
            // multiplex, so a receiver tuned to the transmission shows that
            // everything between the two is working.
            #[cfg(feature = "ffmpeg")]
            true => Some(Source::Encoded(decode::transcode::ToTs::bars(self.bitrate))),
            #[cfg(not(feature = "ffmpeg"))]
            true => None,
            false => match common::fs::blocking::File::open(&self.path) {
                Ok(f) => Some(self.read_as(f)),
                Err(e) => {
                    tracing::warn!("ts_source: {}: {e}", self.path);
                    None
                }
            },
        };
        self.loops = 0;
    }

    /// A transport stream is read as it is; anything else goes through
    /// ffmpeg. What it is is read off the front of the file rather than off
    /// its name, because a transport stream is often called something else.
    fn read_as(&self, f: common::fs::blocking::File) -> Source {
        use std::io::Read;
        let mut head = [0u8; 2 * mpegts::PACKET + 1];
        let mut probe = std::io::BufReader::new(f);
        // Filled, not read once: a short read is a read, and a transport
        // stream sniffed from the first sixty bytes of itself looks like
        // something else and goes through the encoder.
        let mut n = 0;
        while n < head.len() {
            match probe.read(&mut head[n..]) {
                Ok(0) | Err(_) => break,
                Ok(got) => n += got,
            }
        }
        #[cfg(feature = "ffmpeg")]
        if !decode::transcode::is_transport_stream(&head[..n]) {
            tracing::info!("ts_source: re-encoding {} as a multiplex", self.path);
            return Source::Encoded(decode::transcode::ToTs::open(&self.path, self.bitrate));
        }
        #[cfg(not(feature = "ffmpeg"))]
        if !head[..n].starts_with(&[0x47]) {
            tracing::warn!("ts_source: {} is not a transport stream", self.path);
        }
        let _ = n;
        let _ = std::io::Seek::rewind(&mut probe);
        Source::Packets(probe)
    }

    /// Times the file has been round, which is what says a transmission is
    /// repeating rather than running out.
    pub fn loops(&self) -> u64 {
        self.loops
    }

    /// One packet of nothing, which is what a multiplex sends when it has
    /// nothing: PID 0x1FFF and no payload worth reading.
    fn null_packet(&mut self) {
        self.packet.clear();
        self.packet.extend_from_slice(&[0x47, 0x1F, 0xFF, 0x10]);
        self.packet.resize(mpegts::PACKET, 0xFF);
    }

    /// The next packet from the file, or a null one where there is no file.
    fn next_packet(&mut self) -> &[u8] {
        use std::io::Read;
        let Some(f) = self.file.as_mut() else {
            self.null_packet();
            return &self.packet;
        };
        self.packet.resize(mpegts::PACKET, 0);
        let mut got = 0;
        while got < mpegts::PACKET {
            match f.read(&mut self.packet[got..]) {
                Ok(0) => {
                    // Round again from the top. Anything short of a packet is
                    // thrown away rather than joined to the first packet of
                    // the next pass, which would be a packet that was never
                    // in the file. What ffmpeg is encoding starts itself
                    // over; ending means it will send no more at all.
                    if !f.rewind() {
                        self.null_packet();
                        return &self.packet;
                    }
                    self.loops += 1;
                    got = 0;
                }
                Ok(n) => got += n,
                Err(_) => {
                    self.null_packet();
                    return &self.packet;
                }
            }
        }
        &self.packet
    }
}

impl pipeline::node::Simple for TsSourceNode {
    fn name(&self) -> &str {
        TS_SOURCE.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        let what = match self.path.is_empty() {
            true => "the test card".to_string(),
            false => std::path::Path::new(&self.path)
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| self.path.clone()),
        };
        let mut out = vec![("sending".into(), what)];
        if self.loops > 0 {
            out.push(("times round".into(), self.loops.to_string()));
        }
        out
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("ts_source needs a clock to run against"));
        }
        self.rate = input.spec.rate;
        Ok(StreamSpec {
            kind: PortKind::Bytes,
            // The clock, passed on: what the bytes are worth in time is this
            // stage's own business, and the modulator behind needs the
            // radio's rate to know what to resample to.
            rate: input.spec.rate,
            center: input.spec.center,
            bandwidth: input.spec.bandwidth,
            channels: 1,
            flow: pipeline::port::Flow::Tx,
            domain: pipeline::port::Domain::Baseband,
        })
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        if self.rate <= 0.0 {
            return Ok(());
        }
        self.owed += input.len() as f64 / self.rate * self.bitrate / 8.0;
        let out = output.bytes_mut();
        while self.owed >= mpegts::PACKET as f64 {
            self.owed -= mpegts::PACKET as f64;
            let packet = self.next_packet().to_vec();
            out.extend_from_slice(&packet);
        }
        Ok(())
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        vec![
            pipeline::param::Param::text(PATH, self.path.clone()).label("Transport stream"),
            pipeline::param::Param::float(BITRATE, self.bitrate, 1.0..=100e6)
                .label("Bit rate")
                .unit("bit/s"),
        ]
    }

    fn set_param(&mut self, name: &str, value: pipeline::ParamValue) -> Result<()> {
        match name {
            PATH => {
                let want = value.as_str().unwrap_or_default().to_string();
                if want != self.path {
                    self.path = want;
                    self.open();
                }
                Ok(())
            }
            BITRATE => {
                self.bitrate = value.as_f64().unwrap_or(self.bitrate).max(1.0);
                Ok(())
            }
            _ => Err(common::Error::other(format!("ts_source: unknown parameter {name:?}"))),
        }
    }

    fn reset(&mut self) {
        self.open();
        self.owed = 0.0;
    }
}

/// Where a transmitted stream comes from: the file, or ffmpeg encoding it.
enum Source {
    Packets(std::io::BufReader<common::fs::blocking::File>),
    #[cfg(feature = "ffmpeg")]
    Encoded(decode::transcode::ToTs),
}

impl std::io::Read for Source {
    fn read(&mut self, out: &mut [u8]) -> std::io::Result<usize> {
        match self {
            Self::Packets(f) => f.read(out),
            #[cfg(feature = "ffmpeg")]
            Self::Encoded(t) => t.read(out),
        }
    }
}

impl Source {
    /// Start the file again, or say that there is no starting it again.
    fn rewind(&mut self) -> bool {
        match self {
            Self::Packets(f) => std::io::Seek::rewind(f).is_ok(),
            // The thread starts its own next pass, so nothing arriving here
            // means ffmpeg has stopped for good.
            #[cfg(feature = "ffmpeg")]
            Self::Encoded(_) => false,
        }
    }
}

/// A transport stream onto the air: bytes in, an 8 MHz multiplex out.
///
/// The modulator runs at the standard's own 64/7 MS/s whatever the radio is
/// sampling at, and a resampler puts it on the radio's rate, so a
/// transmission does not depend on the device happening to offer 9.142857
/// MS/s. Short of packets it stuffs the multiplex with nulls, which is what
/// a real one does between programmes rather than leaving a hole.
pub struct DvbtModNode {
    params: Params,
    tx: Modulating,
    level: f32,
    out_rate: f64,
    /// Bytes that did not make a whole packet last block.
    pending: Vec<u8>,
    clipped: u64,
}

/// The modulator on a thread of its own: packets in, samples at the radio's
/// rate out.
///
/// A multiplex costs about a third of a processor to build, and the thread
/// the graph runs on has a radio to keep fed: with the modulation on it, a
/// transmission stutters and everything else the receiver draws stutters
/// with it. The queue either side is what smooths a block arriving late.
struct Modulating {
    packets: Option<crossbeam_channel::Sender<Vec<u8>>>,
    air: crossbeam_channel::Receiver<Vec<C32>>,
    give_back: crossbeam_channel::Sender<Vec<C32>>,
    thread: Option<common::thread::JoinHandle<()>>,
    /// Packets the queue could not take, which is a hole in the multiplex.
    dropped: u64,
}

impl Modulating {
    fn new(params: Params, out_rate: f64) -> Self {
        let (packets, work) = crossbeam_channel::bounded::<Vec<u8>>(TX_QUEUE);
        let (send, air) = crossbeam_channel::bounded::<Vec<C32>>(TX_QUEUE);
        let (give_back, spare) = crossbeam_channel::bounded::<Vec<C32>>(TX_QUEUE + 2);
        let back: crossbeam_channel::Receiver<Vec<C32>> = spare.clone();
        let thread = common::thread::Builder::new()
            .name("dvbt-mod".into())
            .spawn(move || {
                let mut tx = DvbtModulator::new(params);
                let mut resample = Rational::approx(dvbt::RATE_HZ, out_rate.max(1.0), 1 << 14);
                let mut baseband = Vec::new();
                while let Ok(bytes) = work.recv() {
                    baseband.clear();
                    for packet in bytes.chunks_exact(mpegts::PACKET) {
                        tx.push(packet, &mut baseband);
                    }
                    if baseband.is_empty() {
                        continue;
                    }
                    let mut out = back.try_recv().unwrap_or_default();
                    out.clear();
                    resample.process(&baseband, &mut out);
                    if send.send(out).is_err() {
                        return;
                    }
                }
            })
            .ok();
        Self { packets: Some(packets), air, give_back, thread, dropped: 0 }
    }

    /// Hand over whole packets, waiting only as long as a transmission can
    /// afford to.
    fn push(&mut self, bytes: Vec<u8>) {
        let Some(tx) = &self.packets else { return };
        if crate::wait::send_within(&tx, bytes, WAIT).is_err() {
            self.dropped += 1;
        }
    }

    /// Whatever the thread has finished, onto the air.
    fn take(&mut self, out: &mut Vec<C32>) {
        while let Ok(block) = self.air.try_recv() {
            out.extend_from_slice(&block);
            let _ = self.give_back.try_send(block);
        }
    }

    /// Drain what is still being modulated. For a test, which has an end.
    ///
    /// Taking as it goes rather than joining first: the thread is blocked on
    /// handing over a block, so waiting for it without emptying the queue
    /// waits for ever.
    fn finish(&mut self, out: &mut Vec<C32>) {
        self.packets = None;
        while self.thread.as_ref().is_some_and(|t| !t.is_finished()) {
            self.take(out);
            std::thread::sleep(common::time::Duration::from_millis(1));
        }
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        self.take(out);
    }
}

impl Drop for Modulating {
    fn drop(&mut self) {
        self.packets = None;
        while self.thread.as_ref().is_some_and(|t| !t.is_finished()) {
            while self.air.try_recv().is_ok() {}
            std::thread::sleep(common::time::Duration::from_millis(1));
        }
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

/// Blocks of packets waiting to be modulated, and blocks of samples waiting
/// to go out.
const TX_QUEUE: usize = 8;

impl Default for DvbtModNode {
    fn default() -> Self {
        Self::new(Params::typical(), 0.25)
    }
}

impl DvbtModNode {
    pub fn new(params: Params, level: f32) -> Self {
        Self {
            tx: Modulating::new(params, dvbt::RATE_HZ),
            level,
            out_rate: 0.0,
            pending: Vec::new(),
            clipped: 0,
            params,
        }
    }

    /// Packets the modulating thread could not be given in time, which is a
    /// hole in the multiplex.
    pub fn dropped(&self) -> u64 {
        self.tx.dropped
    }

    /// Samples the level held back from clipping. OFDM peaks about ten
    /// decibels above its own average, so a level set for the average is
    /// what decides whether those peaks survive.
    pub fn clipped(&self) -> u64 {
        self.clipped
    }

    /// What the multiplex carries, which is what the source has to feed it.
    pub fn bitrate(&self) -> f64 {
        self.params.bitrate()
    }

    fn rebuild(&mut self) {
        self.tx = Modulating::new(self.params, self.out_rate.max(dvbt::RATE_HZ));
        self.pending.clear();
    }

    /// Everything still being modulated, for a test or an example that has
    /// an end to reach.
    pub fn flush(&mut self, out: &mut Vec<C32>) {
        let at = out.len();
        self.tx.finish(out);
        self.scale(&mut out[at..]);
    }

    /// The transmit level, and the clip an OFDM peak runs into.
    fn scale(&mut self, out: &mut [C32]) {
        for v in out.iter_mut() {
            *v *= self.level;
            if v.norm() > 1.0 {
                *v /= v.norm();
                self.clipped += 1;
            }
        }
    }
}

impl pipeline::node::Simple for DvbtModNode {
    fn name(&self) -> &str {
        DVBT_MOD.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = vec![("multiplex".into(), format!("{:.2} Mbit/s", self.bitrate() / 1e6))];
        if self.tx.dropped > 0 {
            out.push(("packets lost".into(), self.tx.dropped.to_string()));
        }
        if self.clipped > 0 {
            out.push(("clipped".into(), self.clipped.to_string()));
        }
        out
    }

    fn negotiate(&mut self, input: &PortSpec) -> Result<StreamSpec> {
        if input.spec.kind != PortKind::Bytes {
            return Err(common::Error::other("dvbt_mod takes a transport stream as bytes"));
        }
        if input.spec.rate <= 0.0 {
            return Err(common::Error::other("dvbt_mod needs the radio's rate to modulate at"));
        }
        if input.spec.rate < CHANNEL_WIDTH_HZ {
            // Not refused: the chain is drawn whether or not a key is down,
            // and refusing here would take the receiver down with it. What
            // goes out is the multiplex filtered to the span it was given,
            // which no receiver will read.
            tracing::warn!(
                "dvbt_mod: {:.3} MS/s is narrower than the 8 MHz multiplex",
                input.spec.rate / 1e6
            );
        }
        self.out_rate = input.spec.rate;
        self.tx = Modulating::new(self.params, self.out_rate);
        Ok(StreamSpec {
            kind: PortKind::Iq,
            rate: self.out_rate,
            center: input.spec.center,
            bandwidth: CHANNEL_WIDTH_HZ,
            channels: 1,
            flow: pipeline::port::Flow::Tx,
            domain: pipeline::port::Domain::Baseband,
        })
    }

    fn process(
        &mut self,
        input: &Payload,
        output: &mut Payload,
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(bytes) = input.as_bytes() else {
            return Ok(());
        };
        self.pending.extend_from_slice(bytes);
        let whole = self.pending.len() - self.pending.len() % mpegts::PACKET;
        if whole > 0 {
            let packets: Vec<u8> = self.pending.drain(..whole).collect();
            self.tx.push(packets);
        }
        let out = output.iq_mut();
        let at = out.len();
        self.tx.take(out);
        self.scale(&mut out[at..]);
        Ok(())
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        use pipeline::param::Param;
        vec![
            Param::choice(MODE, mode_choice(self.params.mode), labels(MODES)).label("Carriers"),
            Param::choice(GUARD, guard_choice(self.params.guard), labels(GUARDS))
                .label("Guard interval"),
            Param::choice(
                CONSTELLATION,
                constellation_choice(self.params.constellation),
                labels(CONSTELLATIONS),
            )
            .label("Constellation"),
            Param::choice(CODE_RATE, code_choice(self.params.code_rate_hp), labels(CODE_RATES))
                .label("Code rate"),
            Param::float(LEVEL, self.level as f64, 0.0..=1.0).label("Level"),
        ]
    }

    fn set_param(&mut self, name: &str, value: pipeline::ParamValue) -> Result<()> {
        // Read as a whole number, not as a float: a choice arrives as
        // `Choice`, which is not a float, and reading it as one took every
        // setting from the menu as position zero. Picking 64-QAM set QPSK.
        let pick = |value: &pipeline::ParamValue, n: usize| {
            value.as_i64().map(|v| (v.max(0) as usize).min(n - 1)).unwrap_or(0)
        };
        match name {
            MODE => {
                self.params.mode = MODES[pick(&value, MODES.len())].0;
                self.rebuild();
                Ok(())
            }
            GUARD => {
                self.params.guard = GUARDS[pick(&value, GUARDS.len())].0;
                self.rebuild();
                Ok(())
            }
            CONSTELLATION => {
                self.params.constellation = CONSTELLATIONS[pick(&value, CONSTELLATIONS.len())].0;
                self.rebuild();
                Ok(())
            }
            CODE_RATE => {
                let rate = CODE_RATES[pick(&value, CODE_RATES.len())].0;
                self.params.code_rate_hp = rate;
                self.params.code_rate_lp = rate;
                self.rebuild();
                Ok(())
            }
            LEVEL => {
                self.level = value.as_f64().unwrap_or(0.25).clamp(0.0, 1.0) as f32;
                Ok(())
            }
            _ => Err(common::Error::other(format!("dvbt_mod: unknown parameter {name:?}"))),
        }
    }

    fn reset(&mut self) {
        self.rebuild();
        self.clipped = 0;
    }
}

/// The modulation an operator can pick, each with the word it is known by.
const MODES: &[(Mode, &str)] = &[(Mode::M2k, "2k"), (Mode::M8k, "8k")];
const GUARDS: &[(dvbt::Guard, &str)] = &[
    (dvbt::Guard::G1_32, "1/32"),
    (dvbt::Guard::G1_16, "1/16"),
    (dvbt::Guard::G1_8, "1/8"),
    (dvbt::Guard::G1_4, "1/4"),
];
const CONSTELLATIONS: &[(dvbt::Constellation, &str)] = &[
    (dvbt::Constellation::Qpsk, "QPSK"),
    (dvbt::Constellation::Qam16, "16-QAM"),
    (dvbt::Constellation::Qam64, "64-QAM"),
];
const CODE_RATES: &[(dvbt::CodeRate, &str)] = &[
    (dvbt::CodeRate::R1_2, "1/2"),
    (dvbt::CodeRate::R2_3, "2/3"),
    (dvbt::CodeRate::R3_4, "3/4"),
    (dvbt::CodeRate::R5_6, "5/6"),
    (dvbt::CodeRate::R7_8, "7/8"),
];

fn labels<T>(list: &[(T, &str)]) -> Vec<String> {
    list.iter().map(|(_, l)| (*l).to_string()).collect()
}

fn mode_choice(mode: Mode) -> usize {
    MODES.iter().position(|(m, _)| *m == mode).unwrap_or(1)
}

fn guard_choice(guard: dvbt::Guard) -> usize {
    GUARDS.iter().position(|(g, _)| *g == guard).unwrap_or(0)
}

fn constellation_choice(c: dvbt::Constellation) -> usize {
    CONSTELLATIONS.iter().position(|(x, _)| *x == c).unwrap_or(2)
}

fn code_choice(r: dvbt::CodeRate) -> usize {
    CODE_RATES.iter().position(|(x, _)| *x == r).unwrap_or(1)
}

/// What a transmit chain's two stages are called and what they read.
const PATH: &str = "path";
const BITRATE: &str = "bitrate";
const MODE: &str = "mode";
const GUARD: &str = "guard";
const CONSTELLATION: &str = "constellation";
const CODE_RATE: &str = "code_rate";
const LEVEL: &str = "level";

pub const TS_SOURCE: StageDesc = StageDesc {
    name: "ts_source",
    summary: "Read a transport stream off the disk at the multiplex's own bit rate",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_ts_source(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(TsSourceNode::new(
        s.str_or(PATH, ""),
        s.f64_or(BITRATE, Params::typical().bitrate()),
    )))
}

pub const DVBT_MOD: StageDesc = StageDesc {
    name: "dvbt_mod",
    summary: "Modulate a transport stream as a DVB-T multiplex",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_dvbt_mod(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut node = DvbtModNode::default();
    for (name, n) in [
        (MODE, MODES.len()),
        (GUARD, GUARDS.len()),
        (CONSTELLATION, CONSTELLATIONS.len()),
        (CODE_RATE, CODE_RATES.len()),
    ] {
        if let Some(v) = s.get(name).and_then(|v| v.as_f64()) {
            pipeline::node::Node::set_param(
                &mut node,
                name,
                pipeline::ParamValue::Float(v.min(n as f64 - 1.0)),
            )?;
        }
    }
    if let Some(v) = s.get(LEVEL).and_then(|v| v.as_f64()) {
        pipeline::node::Node::set_param(&mut node, LEVEL, pipeline::ParamValue::Float(v))?;
    }
    Ok(Box::new(node))
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

    pub(super) fn noise(seed: u64, sigma: f32, n: usize) -> Vec<C32> {
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

    #[test]
    fn a_receiver_thread_with_a_full_queue_behind_it_is_let_go() {
        let mut rx = Offloaded::new();
        for _ in 0..2 * QUEUE + 2 {
            rx.push(&[C32::default(); 4096]);
        }
        let (done, gone) = crossbeam_channel::bounded(1);
        common::thread::spawn(move || {
            drop(rx);
            let _ = done.send(());
        });
        assert!(
            gone.recv_timeout(common::time::Duration::from_secs(2)).is_ok(),
            "dropping the stage waited on a thread blocked handing back packets nobody reads"
        );
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

        let mut rx = dvbtdec::DvbtReceiver::new();
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
    use super::tests::{noise, on_air, packet};
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
    /// The transmit chain as the graph builds it, read back by the receiver.
    ///
    /// Both stages as nodes rather than the modulator on its own: what this
    /// pins is that a file on the disk, clocked by the radio's own rate and
    /// resampled off the standard's 64/7 MS/s, comes back as the packets
    /// that were in the file.
    #[test]
    fn a_file_transmitted_as_a_multiplex_is_read_back_off_the_air() {
        use pipeline::node::Simple;

        let params = Params {
            mode: Mode::M2k,
            guard: Guard::G1_32,
            constellation: Constellation::Qpsk,
            hierarchy: Hierarchy::None,
            code_rate_hp: CodeRate::R1_2,
            code_rate_lp: CodeRate::R1_2,
            cell_id: Some(0x2F1A),
        };
        let path = common::platform::scratch_dir().join("waveshark-dvbt-tx-test.ts");
        let mut file = Vec::new();
        for n in 0..240u16 {
            file.extend_from_slice(&packet(n));
        }
        std::fs::write(&path, &file).expect("a transport stream on the disk");

        // Four hundred milliseconds of radio, which is long enough for the
        // receiver to lock the TPS, start the inner decoder and read packets
        // out of the third super frame.
        let radio_hz = 20_000_000.0;
        let block = 65_536;
        let blocks = (0.45 * radio_hz / block as f64).ceil() as usize;

        let mut source = TsSourceNode::new(path.to_str().unwrap_or_default(), params.bitrate());
        let mut modulator = DvbtModNode::new(params, 0.25);
        let clock = PortSpec {
            spec: StreamSpec {
                kind: PortKind::Real,
                rate: radio_hz,
                center: Hz(474_000_000),
                channels: 1,
                flow: pipeline::port::Flow::Tx,
                ..Default::default()
            },
            latency: 0,
        };
        let bytes = PortSpec {
            spec: Simple::negotiate(&mut source, &clock).expect("the source takes a clock"),
            latency: 0,
        };
        let air = Simple::negotiate(&mut modulator, &bytes).expect("the modulator takes bytes");
        assert_eq!(air.kind, PortKind::Iq);
        assert_eq!(air.rate, radio_hz, "the modulator puts out the radio's rate");

        let mut span = Vec::new();
        let mut sent = 0usize;
        for _ in 0..blocks {
            let mut ts = Payload::empty_of(PortKind::Bytes);
            let mut iq = Payload::empty_of(PortKind::Iq);
            let ins = [clock];
            let (tags, mut events, mut new_tags) = (Vec::new(), Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Simple::process(&mut source, &Payload::Real(vec![0.0; block]), &mut ts, &mut ctx)
                .expect("the source runs");
            sent += ts.as_bytes().map(|b| b.len()).unwrap_or(0);
            Simple::process(&mut modulator, &ts, &mut iq, &mut ctx).expect("the modulator runs");
            span.extend_from_slice(iq.as_iq().unwrap_or(&[]));
        }
        // The modulator is a thread behind the graph, so the last blocks of
        // an over are still being built when the samples run out.
        modulator.flush(&mut span);
        // The bit rate is the multiplex's, so the bytes taken off the file
        // and the samples put on the air are both what that much time holds.
        let seconds = blocks as f64 * block as f64 / radio_hz;
        let want_bytes = seconds * params.bitrate() / 8.0;
        assert!(
            (sent as f64 - want_bytes).abs() < 2.0 * mpegts::PACKET as f64,
            "{sent} bytes off the file, {want_bytes:.0} in {seconds:.3} s of multiplex"
        );
        assert!(
            (span.len() as f64 - seconds * radio_hz).abs() < 0.02 * seconds * radio_hz,
            "{} samples for {seconds:.3} s at {radio_hz}",
            span.len()
        );

        let mut node = DvbtNode::new(474_000_000.0);
        let spec = PortSpec { spec: StreamSpec::iq(radio_hz, Hz(474_000_000)), latency: 0 };
        node.negotiate(&[spec]).expect("a channel in the span");
        let mut stream = Vec::new();
        let mut events = Vec::new();
        for chunk in span.chunks(block) {
            let payload = Payload::Iq(chunk.to_vec());
            let mut out = [
                Payload::empty_of(PortKind::Bytes),
                Payload::empty_of(PortKind::Video),
                Payload::empty_of(PortKind::Real),
            ];
            let ins = [spec];
            let (tags, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
            stream.extend_from_slice(out[0].as_bytes().unwrap_or(&[]));
        }
        let mut frames = Vec::new();
        stream.extend_from_slice(&node.flush(&mut frames).bytes);

        assert_eq!(node.rx.heard().params, Some(params), "the TPS says what was transmitted");
        let packets = stream.len() / mpegts::PACKET;
        assert!(packets >= 900, "only {packets} packets came back");
        // Every packet is one of the file's, and they arrive in the order
        // they were written.
        let first = (0..240u16)
            .find(|n| stream[..mpegts::PACKET] == packet(*n))
            .expect("the first packet back is one of the file's");
        let mut wrong = 0;
        for (i, got) in stream.chunks_exact(mpegts::PACKET).enumerate() {
            let want = packet(((first as usize + i) % 240) as u16);
            wrong += (got != want) as usize;
        }
        assert_eq!(wrong, 0, "{wrong} of {packets} packets are not the ones in the file");
        let _ = std::fs::remove_file(&path);
    }

    fn read_through_the_stage(air: &[C32]) -> DvbtNode {
        let mut node = DvbtNode::new(474_000_000.0);
        let spec = PortSpec { spec: StreamSpec::iq(RATE_HZ, Hz(474_000_000)), latency: 0 };
        node.negotiate(&[spec]).expect("a channel in the span");
        let mut events = Vec::new();
        for chunk in air.chunks(65_536) {
            let payload = Payload::Iq(chunk.to_vec());
            let mut out = [
                Payload::empty_of(PortKind::Bytes),
                Payload::empty_of(PortKind::Video),
                Payload::empty_of(PortKind::Real),
            ];
            let ins = [spec];
            let (tags, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
        }
        node.flush(&mut Vec::new());
        node
    }

    fn reading(node: &DvbtNode, caption: &str) -> Option<String> {
        Node::readings(node).into_iter().find(|(c, _)| c == caption).map(|(_, v)| v)
    }

    #[test]
    fn the_stage_says_whether_it_is_locked_and_what_it_is_reading() {
        let idle = DvbtNode::new(474_000_000.0);
        assert_eq!(Node::acquisition(&idle), Some(pipeline::Acquisition::Searching));

        let qpsk = Params {
            mode: Mode::M2k,
            guard: Guard::G1_32,
            constellation: Constellation::Qpsk,
            hierarchy: Hierarchy::None,
            code_rate_hp: CodeRate::R1_2,
            code_rate_lp: CodeRate::R1_2,
            cell_id: None,
        };
        let clean = read_through_the_stage(&on_air(qpsk, 3 * 504));
        assert_eq!(Node::acquisition(&clean), Some(pipeline::Acquisition::Locked));
        assert_eq!(reading(&clean, "multiplex").as_deref(), Some("2k 1/32 QPSK 1/2"));
        assert_eq!(reading(&clean, "packets").as_deref(), Some("1244"));
        assert_eq!(reading(&clean, "uncorrectable").as_deref(), Some("0"));

        let qam64 = Params {
            constellation: Constellation::Qam64,
            code_rate_hp: CodeRate::R2_3,
            code_rate_lp: CodeRate::R2_3,
            ..qpsk
        };
        let air = on_air(qam64, 3 * 2016);
        let power = air.iter().map(|c| c.norm_sqr()).sum::<f32>() / air.len() as f32;
        let hiss = noise(0x11_6DB, (power / 10f32.powf(1.16)).sqrt(), air.len());
        let noisy: Vec<C32> = air.iter().zip(&hiss).map(|(a, n)| a + n).collect();
        let stuck = read_through_the_stage(&noisy);
        assert_eq!(
            Node::acquisition(&stuck),
            Some(pipeline::Acquisition::Acquiring),
            "at 11.6 dB 64-QAM 2/3 is found and its TPS read, and not one codeword corrects"
        );
        assert_eq!(reading(&stuck, "multiplex").as_deref(), Some("2k 1/32 64-QAM 2/3"));
        assert_eq!(reading(&stuck, "packets").as_deref(), Some("0"));
        assert_eq!(reading(&stuck, "uncorrectable").as_deref(), Some("2161"));
    }

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
            let mut out = [
                Payload::empty_of(PortKind::Bytes),
                Payload::empty_of(PortKind::Video),
                Payload::empty_of(PortKind::Real),
            ];
            let ins = [spec];
            let tags = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
            stream.extend_from_slice(out[0].as_bytes().unwrap_or(&[]));
        }
        // The decoding thread is still reading when the samples run out.
        let mut frames = Vec::new();
        stream.extend_from_slice(&node.flush(&mut frames).bytes);

        assert_eq!(node.rx.heard().params, Some(params), "the TPS came through the resampler");
        assert_eq!(stream.len() % 188, 0, "whole transport packets");
        assert!(stream.len() >= 188 * 900, "{} bytes of transport stream", stream.len());
        // Two announcements for the multiplex and none for a service: the
        // parameters are said once when the first TPS word decodes, and again
        // a frame later when the second half of the cell identifier arrives.
        // What was modulated is packets on one PID with no tables in them, so
        // nothing names a service.
        let announcements = events
            .iter()
            .filter(|e| matches!(
                e,
                pipeline::event::Event::Decoded(p) if p.innermost().is_some_and(|l| l.id == "dvbt")
            ))
            .count();
        assert_eq!(announcements, 2, "the multiplex is announced on what changed, not per symbol");
    }
}
