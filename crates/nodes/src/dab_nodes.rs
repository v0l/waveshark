//! DAB as a graph node: one band III ensemble, from the air to its station
//! list.
//!
//! The layers are elsewhere. The null symbol, the phase reference and the
//! differential QPSK are `dsp::dab`; the convolutional code is `dsp::conv`;
//! the puncturing, the energy dispersal, the block check and the ensemble's
//! tables are `decode::dab`. What this module does is put them in order and
//! say on the bus what the ensemble calls itself.

use crate::NodeSpec;
use crate::broadcast::{OFF, SERVICE, Want};
use crate::playout::{Playout, RATE_HZ as SOUND_RATE_HZ};
use crate::protocol::{Placed, Placement, Protocol, Shape, Stickiness};
use common::{C32, Result};
use decode::dab::{self, Audio, Ensemble, Service, superframe};
use dsp::dab::Mode;
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::dab::BAND_HZ;
pub use identify::dab::CHANNEL_WIDTH_HZ;
pub use identify::dab::DEFAULT_HZ;
pub use identify::dab::DabProtocol;
pub use identify::dab::RATE_HZ;
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub const FIRST: &str = "first station";

enum Framing {
    Plus(superframe::Reader),
    Layer2,
    Unread,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Heard {
    pub frames: u64,
    pub superframes: u64,
    pub units: u64,
    pub lost: u64,
    pub samples: u64,
}

struct Listener {
    framing: Framing,
    format: Option<superframe::Format>,
    #[cfg(feature = "ffmpeg")]
    codec: Option<decode::sound::Sound>,
    heard: Heard,
}

impl Listener {
    fn new(audio: Option<Audio>) -> Self {
        Self {
            framing: match audio {
                Some(Audio::AacPlus) => Framing::Plus(superframe::Reader::new()),
                Some(Audio::Mp2) => Framing::Layer2,
                Some(Audio::Other(_)) | None => Framing::Unread,
            },
            format: None,
            #[cfg(feature = "ffmpeg")]
            codec: None,
            heard: Heard::default(),
        }
    }

    #[cfg_attr(not(feature = "ffmpeg"), allow(unused_variables))]
    fn hear(&mut self, frame: &[u8], pcm: &mut Vec<f32>) {
        self.heard.frames += 1;
        match &mut self.framing {
            Framing::Plus(reader) => {
                let Some(sf) = reader.push(frame) else { return };
                self.heard.superframes += 1;
                self.heard.units += sf.units.len() as u64;
                self.heard.lost += sf.lost as u64;
                #[cfg(feature = "ffmpeg")]
                if self.format != Some(sf.format) {
                    self.codec = decode::sound::Sound::aac(&sf.format.config()).ok();
                }
                self.format = Some(sf.format);
                #[cfg(feature = "ffmpeg")]
                if let Some(codec) = &mut self.codec {
                    for unit in &sf.units {
                        codec.push(unit, pcm);
                    }
                }
            }
            Framing::Layer2 => {
                #[cfg(feature = "ffmpeg")]
                {
                    if self.codec.is_none() {
                        self.codec = decode::sound::Sound::mp2().ok();
                    }
                    if let Some(codec) = &mut self.codec {
                        codec.push(frame, pcm);
                    }
                }
            }
            Framing::Unread => {}
        }
    }
}

/// One ensemble as a stage: samples in, its fast information blocks out, and
/// what it says about itself on the bus.
pub struct DabNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    resample: Rational,
    rx: dab::DabReceiver,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    at_rate: Vec<C32>,
    /// Whether the ensemble itself has been announced, and which services
    /// have, so a table repeated every 96 ms is said once.
    told: Option<String>,
    named: Vec<u32>,
    at: f64,
    wanted: Want,
    pending: Option<usize>,
    station: Option<u32>,
    listener: Listener,
    frames: Vec<Vec<u8>>,
    pcm: Vec<f32>,
    playout: Playout,
    #[cfg(test)]
    kept: Vec<Vec<u8>>,
}

impl Default for DabNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl DabNode {
    /// A statement about the multiplex this receiver is locked to, as a
    /// reception: what it says is said on the strength of what is coming in.
    fn locked_on(&self, said: common::packet::Proto) -> common::packet::Packet {
        let carrier = crate::locked(
            self.channel_hz as u64,
            CHANNEL_WIDTH_HZ as u32,
            &self.at_rate,
            self.rx.snr_db(),
        );
        common::packet::Packet::heard(carrier)
            .keyed(common::packet::Keying::configured(common::Modulation::Ofdm))
            .decoded(said)
    }

    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(RATE_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            resample: Rational::with_ratio(1, 1),
            rx: dab::DabReceiver::new(Mode::I),
            mixed: Vec::new(),
            narrow: Vec::new(),
            at_rate: Vec::new(),
            told: None,
            named: Vec::new(),
            at: 0.0,
            wanted: Want::Any,
            pending: None,
            station: None,
            listener: Listener::new(None),
            frames: Vec::new(),
            pcm: Vec::new(),
            playout: Playout::default(),
            #[cfg(test)]
            kept: Vec::new(),
        }
    }

    /// The ensemble as it has been read so far.
    pub fn ensemble(&self) -> &Ensemble {
        self.rx.ensemble()
    }

    pub fn stats(&self) -> decode::dab::Stats {
        self.rx.stats()
    }

    pub fn want(&mut self, want: Want) {
        self.wanted = want;
        self.pending = None;
    }

    fn want_from(&mut self, s: &Settings) {
        match s.get(SERVICE) {
            Some(pipeline::ParamValue::Choice(0)) => self.want(Want::Any),
            Some(pipeline::ParamValue::Choice(n)) => {
                self.wanted = Want::Off;
                self.pending = Some(*n);
            }
            _ => self.want(Want::from_settings(s)),
        }
    }

    pub fn wanted(&self) -> &Want {
        &self.wanted
    }

    pub fn listening(&self) -> Option<&Service> {
        self.station.and_then(|id| self.rx.ensemble().service(id))
    }

    pub fn heard(&self) -> Heard {
        self.listener.heard
    }

    pub fn format(&self) -> Option<superframe::Format> {
        self.listener.format
    }

    #[cfg(feature = "ffmpeg")]
    pub fn native(&self) -> Option<(u32, u32)> {
        self.listener.codec.as_ref().map(|c| (c.native_hz, c.native_channels))
    }

    fn chosen(&self) -> Option<&Service> {
        let mut stations = self.rx.ensemble().stations();
        match &self.wanted {
            Want::Off => None,
            Want::Any => stations.next(),
            Want::Id(id) => stations.find(|s| s.id == *id as u32),
            Want::Named(n) => stations.find(|s| s.name.as_deref() == Some(n.as_str())),
        }
    }

    fn follow(&mut self) {
        if let Some(n) = self.pending
            && let Some(s) = self.stations().get(n - 1)
        {
            let want = s.name.clone().map_or(Want::Id(s.id as u16), Want::Named);
            self.want(want);
        }
        let (id, audio) = match self.chosen() {
            Some(s) => (Some(s.id), s.audio()),
            None => (None, None),
        };
        let sub = audio.and_then(|(sub, _)| self.rx.ensemble().sub_channel(sub).copied());
        if id == self.station && self.rx.listening() == sub.as_ref() {
            return;
        }
        self.station = id;
        self.rx.listen(sub);
        self.listener = Listener::new(audio.map(|(_, a)| a));
        self.playout.clear();
    }

    fn stations(&self) -> Vec<&Service> {
        self.rx.ensemble().stations().collect()
    }

    fn choices(&self) -> Vec<String> {
        let mut out = vec![FIRST.to_string()];
        out.extend(self.stations().iter().filter_map(|s| s.name.clone()));
        out.push(OFF.to_string());
        out
    }

    fn choice(&self) -> usize {
        let stations = self.stations();
        match &self.wanted {
            Want::Any => 0,
            Want::Off => stations.len() + 1,
            Want::Id(id) => stations.iter().position(|s| s.id == *id as u32).map_or(0, |n| n + 1),
            Want::Named(n) => stations
                .iter()
                .position(|s| s.name.as_deref() == Some(n.as_str()))
                .map_or(0, |i| i + 1),
        }
    }

    fn set_station(&mut self, v: pipeline::ParamValue) -> Result<()> {
        let stations = self.stations();
        let want = match v {
            pipeline::ParamValue::Choice(n) if n == stations.len() + 1 => Want::Off,
            pipeline::ParamValue::Choice(0) => Want::Any,
            pipeline::ParamValue::Choice(n) => {
                let s = stations
                    .get(n - 1)
                    .ok_or_else(|| common::Error::other("dab: no such station"))?;
                s.name.clone().map_or(Want::Id(s.id as u16), Want::Named)
            }
            pipeline::ParamValue::Int(id) if id < 0 => Want::Off,
            pipeline::ParamValue::Int(0) => Want::Any,
            pipeline::ParamValue::Int(id) => Want::Id(
                u16::try_from(id).map_err(|_| common::Error::other("dab: a station is 16 bits"))?,
            ),
            pipeline::ParamValue::Text(ref t) if t == FIRST || t.is_empty() => Want::Any,
            pipeline::ParamValue::Text(ref t) if t == OFF => Want::Off,
            pipeline::ParamValue::Text(t) => Want::Named(t),
            _ => return Err(common::Error::other("dab: a station is a name or a number")),
        };
        self.want(want);
        Ok(())
    }
}

impl pipeline::node::Node for DabNode {
    fn name(&self) -> &str {
        "dab"
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dab reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < RATE_HZ {
            return Err(common::Error::other("dab needs 2.048 MS/s of channel"));
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("dab needs its ensemble inside the span"));
        }
        let (factor, resample) = dsp::resample::stage(rate, RATE_HZ, 4096)
            .ok_or_else(|| common::Error::other("dab cannot reach 2.048 MS/s from here"))?;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.resample = resample.unwrap_or_else(|| Rational::with_ratio(1, 1));
        self.restart();

        let mut out = i.spec.with_kind(PortKind::Bytes);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        // What leaves is the fast information channel, which is 96 kbit/s of
        // blocks whatever the ensemble carries.
        out.rate = 12_000.0;
        let mut sound = out.with_kind(PortKind::Real);
        sound.rate = SOUND_RATE_HZ;
        sound.channels = 1;
        sound.bandwidth = 0.0;
        Ok(vec![out, sound])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        if let Some(iq) = inputs[0].as_iq() {
            self.at = c.timestamp();
            self.mixed.clear();
            self.mixer.process(iq, &mut self.mixed);
            self.narrow.clear();
            self.decim.process(&self.mixed, &mut self.narrow);
            self.at_rate.clear();
            self.resample.process(&self.narrow, &mut self.at_rate);
            self.rx.push(&self.at_rate);
            self.follow();
            self.frames.clear();
            self.rx.take_heard(&mut self.frames);
            #[cfg(test)]
            self.kept.extend(self.frames.iter().cloned());
            self.pcm.clear();
            for frame in &self.frames {
                self.listener.hear(frame, &mut self.pcm);
            }
            self.listener.heard.samples += self.pcm.len() as u64;
            self.playout.arrive(None, &self.pcm);
        }
        outputs[1].real_mut().extend(self.playout.sound_for(c.block_seconds));

        let named = self.rx.ensemble().name.clone();
        if named.is_some()
            && self.told != named
            && let Some(d) = dab::ensemble_read(&self.rx)
        {
            self.told = named;
            c.emit(pipeline::event::Event::Decoded(self.locked_on(d)));
        }
        let fresh: Vec<u32> = self
            .rx
            .ensemble()
            .stations()
            .filter(|s| !self.named.contains(&s.id))
            .map(|s| s.id)
            .collect();
        for id in fresh {
            self.named.push(id);
            if let Some(d) = dab::service_read(&self.rx, id) {
                c.emit(pipeline::event::Event::Decoded(self.locked_on(d)));
            }
        }
        Ok(())
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        Some(match (self.rx.locked(), self.rx.stats().good > 0) {
            (false, _) => pipeline::Acquisition::Searching,
            (true, false) => pipeline::Acquisition::Acquiring,
            (true, true) => pipeline::Acquisition::Locked,
        })
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = Vec::new();
        let ensemble = self.rx.ensemble();
        if let Some(name) = &ensemble.name {
            out.push(("ensemble".into(), name.trim().to_string()));
        }
        if !ensemble.services.is_empty() {
            out.push(("services".into(), ensemble.services.len().to_string()));
        }
        if let Some(q) = self.rx.stats().quality() {
            out.push(("fic".into(), format!("{:.0}% intact", q * 100.0)));
        }
        if let Some(s) = self.listening() {
            let name = s.name.clone().unwrap_or_else(|| format!("{:04X}", s.id));
            let kind = s.audio().map(|(_, a)| a.label()).unwrap_or_default();
            out.push(("station".into(), format!("{name} {kind}").trim().to_string()));
        }
        if let Some(sub) = self.rx.listening() {
            out.push((
                "subchannel".into(),
                format!("{} kbit/s {}", sub.bitrate_kbps, sub.protection.label()),
            ));
        }
        let heard = self.listener.heard;
        if heard.units + heard.lost > 0 {
            out.push((
                "audio units".into(),
                format!("{} of {}", heard.units, heard.units + heard.lost),
            ));
        }
        out
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.restart();
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        vec![
            pipeline::param::Param::choice(SERVICE, self.choice(), self.choices())
                .label("Listening"),
        ]
    }

    fn set_param(&mut self, name: &str, v: pipeline::ParamValue) -> Result<()> {
        if name != SERVICE {
            return Err(common::Error::other(format!("dab: unknown parameter {name:?}")));
        }
        self.set_station(v)
    }
}

impl DabNode {
    fn restart(&mut self) {
        self.rx = dab::DabReceiver::new(Mode::I);
        self.told = None;
        self.named.clear();
        self.station = None;
        self.listener = Listener::new(None);
        self.playout.clear();
    }
}

impl Protocol for DabProtocol {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::Continuously
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

    /// Band III, which is where every European ensemble is. The L band was
    /// allocated to it too and nothing is left transmitting there.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} DAB", hz / 1e6)
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    /// The fast information channel's blocks.
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Bytes, PortKind::Real]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The ensemble this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "dab",
    summary: "One DAB ensemble: OFDM, the fast information channel, its stations and their sound",
    category: Category::Decode,
    // What it puts on its port is the fast information channel. What reaches
    // the packet list is the ensemble and its services, emitted rather than
    // carried on a wire.
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut node = DabNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ));
    node.want_from(s);
    Ok(Box::new(node))
}

#[cfg(test)]
pub const SHARK: superframe::Format =
    superframe::Format { dac_48k: true, sbr: true, stereo: false, ps: true, surround: 0 };

#[cfg(test)]
fn sub_channels() -> [dab::SubChannel; 2] {
    use decode::dab::{Protection, SubChannel};
    [
        SubChannel {
            id: 1,
            start: 0,
            size: 72,
            bitrate_kbps: 96,
            protection: Protection::EqualA { level: 3 },
        },
        SubChannel {
            id: 2,
            start: 72,
            size: 96,
            bitrate_kbps: 64,
            protection: Protection::EqualA { level: 1 },
        },
    ]
}

#[cfg(test)]
pub fn programme(station: usize, frames: usize) -> Vec<Vec<u8>> {
    let sub = sub_channels()[station];
    let len = sub.bitrate_kbps as usize * 3;
    match station {
        0 => {
            let s = sub.bitrate_kbps as usize / 8;
            let room = SHARK.room(s);
            (0..frames.div_ceil(superframe::FRAMES))
                .flat_map(|k| {
                    let units: Vec<Vec<u8>> = (0..SHARK.units())
                        .map(|u| {
                            let size = room / SHARK.units()
                                + if u + 1 == SHARK.units() { room % SHARK.units() } else { 0 };
                            (0..size).map(|i| (i * 7 + u * 13 + k * 29) as u8).collect()
                        })
                        .collect();
                    superframe::encode(SHARK, &units, s)
                })
                .take(frames)
                .collect()
        }
        _ => (0..frames).map(|k| (0..len).map(|i| (i + 3 * k) as u8).collect()).collect(),
    }
}

/// An ensemble on the air, for a test that has to know what is in one.
#[cfg(test)]
pub fn transmit(seconds: f64) -> Vec<C32> {
    use decode::dab::msc;
    use decode::dab::{Audio, FicTx, ProgrammeType};
    let mode = Mode::I;
    let mut tx = FicTx::new();
    tx.ensemble(0xC1AB)
        .ensemble_label(0xC1AB, "WAVESHARK")
        .sub_channel(1, 0, 72, 3)
        .sub_channel(2, 72, 96, 1)
        .service(0xC221, 1, Audio::AacPlus)
        .service_label(0xC221, "Shark FM")
        .programme_type(0xC221, ProgrammeType::Pop)
        .service(0xC222, 2, Audio::Mp2)
        .service_label(0xC222, "Reef Radio")
        .programme_type(0xC222, ProgrammeType::News);
    let fic: Vec<u8> = {
        let bits = tx.frame_bits(mode.fibs());
        let mut coded = Vec::new();
        for block in bits.chunks(decode::dab::CODEWORD_DATA) {
            let mut bytes = Vec::with_capacity(block.len() / 8);
            for byte in block.chunks(8) {
                bytes.push(byte.iter().fold(0u8, |acc, &b| (acc << 1) | b));
            }
            coded.extend(decode::dab::encode(&bytes));
        }
        coded
    };

    let mut modulator = dsp::dab::tx::Modulator::new(mode);
    let mut out = Vec::new();
    let frames = (seconds / mode.frame_seconds()).ceil() as usize;
    let per_frame = (modulator.bits_per_frame() - fic.len()) / msc::CIF_BITS;
    let mut stations: Vec<_> = sub_channels()
        .into_iter()
        .enumerate()
        .map(|(k, sub)| {
            let profile = msc::Profile::of(&sub).expect("a profile");
            (sub, profile, msc::Interleaver::default(), programme(k, frames * per_frame))
        })
        .collect();
    let mut bits = vec![0u8; modulator.bits_per_frame()];
    bits[..fic.len()].copy_from_slice(&fic);
    for f in 0..frames {
        for c in 0..per_frame {
            let cif = &mut bits[fic.len() + c * msc::CIF_BITS..fic.len() + (c + 1) * msc::CIF_BITS];
            cif.fill(0);
            for (sub, profile, interleaver, frames) in stations.iter_mut() {
                let coded = interleaver.push(&msc::encode(profile, &frames[f * per_frame + c]));
                let at = sub.start as usize * msc::CU_BITS;
                cif[at..at + coded.len()].copy_from_slice(&coded);
            }
        }
        modulator.frame(&bits, &mut out);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use common::packet::Proto;
    use decode::dab::ProgrammeType;
    use pipeline::node::Node;

    fn replay(n: &mut DabNode, iq: &[C32], rate: f64, center: f64) -> (Vec<Proto>, Vec<f32>) {
        let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 };
        n.negotiate(std::slice::from_ref(&spec)).expect("an ensemble in the span");
        let ins = [spec];
        let tags = Vec::new();
        let (mut said, mut sound) = (Vec::new(), Vec::new());
        for chunk in iq.chunks(65_536) {
            let mut outs = [Payload::empty_of(PortKind::Bytes), Payload::empty_of(PortKind::Real)];
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            ctx.block_seconds = chunk.len() as f64 / rate;
            let input = Payload::Iq(chunk.to_vec());
            n.process(&[&input], &mut outs, &mut ctx).expect("read");
            sound.extend_from_slice(outs[1].as_real().unwrap_or(&[]));
            said.extend(events.into_iter().filter_map(|e| match e {
                pipeline::event::Event::Decoded(p) => p.stack.last().cloned(),
                _ => None,
            }));
        }
        (said, sound)
    }

    fn melbourne() -> Option<common::IqBuf> {
        const NAME: &str = "dab_melbourne_9a_202.928M_2500k.cs16";
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(NAME);
        if !p.exists() {
            eprintln!("skipping: {NAME} absent, run testdata/fetch.sh");
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    fn follows(kept: &[Vec<u8>], sent: &[Vec<u8>]) -> Option<usize> {
        let at = sent.iter().position(|f| Some(f) == kept.first())?;
        (sent.get(at..at + kept.len()) == Some(kept)).then_some(at)
    }

    #[test]
    fn both_stations_read_back_the_frames_they_sent() {
        let air = transmit(3.0);
        for (station, name, superframes) in [(0, "Shark FM", 20u64), (1, "Reef Radio", 0)] {
            let mut n = DabNode::new(DEFAULT_HZ);
            n.want(Want::Named(name.into()));
            replay(&mut n, &air, RATE_HZ, DEFAULT_HZ);
            let sent = programme(station, 200);
            assert_eq!(n.kept.len(), 105, "{name}");
            let at = follows(&n.kept, &sent).unwrap_or_else(|| panic!("{name} out of order"));
            assert_eq!(at, 4, "{name} starts on the frame after the one acquisition took");
            let heard = n.heard();
            assert_eq!(heard.frames, 105, "{name}");
            assert_eq!(heard.superframes, superframes, "{name}");
            assert_eq!((heard.units, heard.lost), (3 * superframes, 0), "{name}");
            assert_eq!(n.listening().and_then(|s| s.name.as_deref()), Some(name));
        }
    }

    #[test]
    fn a_station_chosen_from_the_list_is_the_same_station_after_a_rebuild() {
        let mut settings = Settings::new();
        settings.insert(SERVICE.into(), pipeline::ParamValue::Choice(2));
        let built = build(&settings).expect("a node");
        let mut n = *built.into_any().downcast::<DabNode>().expect("a dab node");
        replay(&mut n, &transmit(2.0), RATE_HZ, DEFAULT_HZ);
        assert_eq!(n.wanted(), &Want::Named("Reef Radio".into()));
        assert_eq!(n.listening().and_then(|s| s.name.as_deref()), Some("Reef Radio"));
        assert_eq!(n.kept.len(), 61);
        assert_eq!(follows(&n.kept, &programme(1, 100)), Some(4));
    }

    #[test]
    fn a_station_left_off_reads_no_subchannel() {
        let mut n = DabNode::new(DEFAULT_HZ);
        n.want(Want::Off);
        let (said, sound) = replay(&mut n, &transmit(2.0), RATE_HZ, DEFAULT_HZ);
        assert_eq!(said.len(), 3, "the ensemble and its two stations");
        assert_eq!(n.kept.len(), 0);
        assert!(n.rx.listening().is_none());
        assert_eq!(sound.iter().filter(|v| **v != 0.0).count(), 0);
    }

    const WELLE_IO_2_4: [(&str, usize, u32); 2] =
        [("Nova 100", 57, 0x10BB_96B0), ("LightDigital", 69, 0x93B9_CBED)];

    #[test]
    fn melbourne_stations_read_the_bytes_welle_io_2_4_read() {
        let Some(buf) = melbourne() else { return };
        let (rate, center) = (buf.rate.as_f64(), buf.center.as_f64());
        for (name, frames, crc) in WELLE_IO_2_4 {
            let mut n = DabNode::new(center);
            n.want(Want::Named(name.into()));
            let (_, sound) = replay(&mut n, &buf.samples, rate, center);
            assert_eq!(n.kept.len(), frames, "{name}");
            let bytes = n.kept.concat();
            assert_eq!(
                decode::bits::crc32(&bytes, 0x04C1_1DB7, 0xFFFF_FFFF),
                crc,
                "{name}: CRC-32/MPEG-2 of the logical frames differs from those welle.io 2.4 \
                 dumped with welle-cli -D off the capture resampled to 2.048 MS/s"
            );
            let heard = n.heard();
            let superframes = frames as u64 / 5;
            assert_eq!(heard.superframes, superframes, "{name}");
            let units = superframes * n.format().expect("a format").units() as u64;
            assert_eq!((heard.units, heard.lost), (units, 0), "{name}");
            if cfg!(feature = "ffmpeg") {
                let whole = superframes * 5760;
                assert!(
                    (whole - 5760..=whole).contains(&heard.samples),
                    "{name}: {} samples of {whole}, the floor a superframe short for the \
                     decoder's delay",
                    heard.samples
                );
                #[cfg(feature = "ffmpeg")]
                {
                    let format = n.format().expect("a format");
                    let channels = if format.stereo { 2 } else { 1 };
                    assert_eq!(
                        n.native(),
                        Some((format.core_hz(), channels)),
                        "{name}: ffmpeg 7.1 to 8.1 decode the AAC core of a 960 sample HE-AAC \
                         stream and not its SBR or PS; one with FFmpeg commit \
                         'aacdec: add support for 960-frame HE-AAC (DAB+) decoding' puts out \
                         {} Hz",
                        format.output_hz()
                    );
                }
                let played = sound.iter().filter(|v| **v != 0.0).count() as u64;
                assert!(played > heard.samples / 2, "{name}: {played} samples reached the bus");
            } else {
                assert_eq!(heard.samples, 0);
            }
        }
    }

    /// Two seconds of a synthesised ensemble, read the whole way through: the
    /// ensemble's name, both stations, their programme types and the bit
    /// rates their subchannels are carried at.
    #[test]
    fn an_ensemble_reads_its_own_station_list() {
        let air = transmit(2.0);
        let mut rx = dab::DabReceiver::new(Mode::I);
        let good = rx.push(&air);
        // Twenty-one frames of 96 ms went out and all twenty-one came back.
        assert_eq!(rx.frames(), 21);
        assert_eq!(rx.stats().codewords, 84);
        // Four codewords a frame, three blocks a codeword, every one of them
        // checking out on a signal with nothing on it.
        assert_eq!(good, 252);
        assert_eq!(rx.stats().good, 252);
        assert_eq!(rx.stats().bad, 0);

        let e = rx.ensemble();
        assert_eq!(e.name.as_deref(), Some("WAVESHARK"));
        assert_eq!(e.id, Some(0xC1AB));
        assert_eq!(e.services.len(), 2);
        assert_eq!(e.sub_channels.len(), 2);
        let names: Vec<&str> = e.stations().filter_map(|s| s.name.as_deref()).collect();
        assert_eq!(names, vec!["Shark FM", "Reef Radio"]);
        let shark = e.service(0xC221).expect("the first station");
        assert_eq!(shark.programme_type, Some(ProgrammeType::Pop));
        assert_eq!(shark.audio(), Some((1, Audio::AacPlus)));
        assert_eq!(e.sub_channel(1).expect("its subchannel").bitrate_kbps, 96);
        let reef = e.service(0xC222).expect("the second station");
        assert_eq!(reef.programme_type, Some(ProgrammeType::News));
        assert_eq!(reef.audio(), Some((2, Audio::Mp2)));
        assert_eq!(e.sub_channel(2).expect("its subchannel").bitrate_kbps, 64);
        assert!(rx.snr_db() > 40.0, "a clean ensemble reads {} dB", rx.snr_db());
    }

    /// The same ensemble with noise on it.
    ///
    /// Acquisition costs the first frame, so twenty of the twenty-one
    /// transmitted are read and 240 blocks is everything there is to get.
    /// Measured down the range: 240 at 15, 12, 9 and 6 dB, 239 at 4 dB, 233
    /// at 3 dB and 172 at 2 dB, which is where a rate 1/3 code gives up.
    #[test]
    fn a_noisy_ensemble_still_names_itself() {
        for (snr_db, floor) in [(6.0f32, 240usize), (3.0, 220)] {
            let air = noisy(transmit(2.0), snr_db);
            let mut rx = dab::DabReceiver::new(Mode::I);
            let good = rx.push(&air);
            assert_eq!(rx.frames(), 20);
            assert!(good >= floor, "{good} blocks of 240 at {snr_db} dB, wanted {floor}");
            assert_eq!(rx.ensemble().name.as_deref(), Some("WAVESHARK"));
            assert_eq!(rx.ensemble().stations().count(), 2);
        }
    }

    fn noisy(mut air: Vec<C32>, snr_db: f32) -> Vec<C32> {
        let signal: f32 = air.iter().map(|s| s.norm_sqr()).sum::<f32>() / air.len().max(1) as f32;
        let level = (signal / 10f32.powf(snr_db / 10.0)).sqrt() / 2f32.sqrt();
        let mut state = 0x5eed_1234u32;
        for s in air.iter_mut() {
            let mut next = || {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (state >> 16) as i16 as f32 / 32768.0 * level * 1.732
            };
            *s += C32::new(next(), next());
        }
        air
    }

    #[test]
    fn a_noisy_station_is_mended_by_its_reed_solomon() {
        for (snr_db, clean, superframes, units, lost) in [(4.0, 25, 19, 57, 0), (3.0, 0, 13, 0, 39)]
        {
            let mut n = DabNode::new(DEFAULT_HZ);
            n.want(Want::Named("Shark FM".into()));
            replay(&mut n, &noisy(transmit(3.0), snr_db), RATE_HZ, DEFAULT_HZ);
            let sent = programme(0, 200);
            assert_eq!(n.kept.len(), 101, "at {snr_db} dB");
            assert_eq!(n.kept.iter().filter(|f| sent.contains(f)).count(), clean, "at {snr_db} dB");
            let heard = n.heard();
            assert_eq!(
                (heard.superframes, heard.units, heard.lost),
                (superframes, units, lost),
                "at {snr_db} dB"
            );
        }
    }

    /// Minutes of noise into the whole node: no lock, no block, no ensemble.
    #[test]
    fn noise_names_nothing() {
        let mut rx = dab::DabReceiver::new(Mode::I);
        assert!(rx.listen(Some(sub_channels()[0])));
        let mut state = 0xfeed_beefu32;
        for _ in 0..600 {
            let air: Vec<C32> = (0..Mode::I.frame())
                .map(|_| {
                    let mut next = || {
                        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                        (state >> 16) as i16 as f32 / 32768.0
                    };
                    C32::new(next(), next())
                })
                .collect();
            assert_eq!(rx.push(&air), 0);
        }
        assert_eq!(rx.stats().good, 0);
        assert!(!rx.locked());
        let mut heard = Vec::new();
        rx.take_heard(&mut heard);
        assert_eq!(heard.len(), 0);
        assert_eq!(rx.msc_stats().map(|m| m.cifs), Some(0));
        assert_eq!(*rx.ensemble(), Ensemble::default());
    }

    #[test]
    fn the_ensemble_has_to_be_inside_the_span_and_wide_enough() {
        let mut n = DabNode::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(2_048_000.0, Hz(200_000_000)), latency: 0 };
        assert!(n.negotiate(std::slice::from_ref(&far)).is_err());
        let thin = PortSpec { spec: StreamSpec::iq(1_000_000.0, Hz(222_064_000)), latency: 0 };
        assert!(n.negotiate(std::slice::from_ref(&thin)).is_err());
        for rate in [2_048_000.0, 2_400_000.0, 8_000_000.0, 20_000_000.0] {
            let mut n = DabNode::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(222_064_000)), latency: 0 };
            let out = n
                .negotiate(std::slice::from_ref(&spec))
                .unwrap_or_else(|e| panic!("{rate} refused: {e}"));
            assert_eq!(out[0].kind, PortKind::Bytes);
            assert_eq!(out[0].center, Hz(222_064_000));
            assert_eq!((out[1].kind, out[1].rate), (PortKind::Real, SOUND_RATE_HZ));
        }
    }
}
