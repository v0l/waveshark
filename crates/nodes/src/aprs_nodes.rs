//! APRS as a graph node.
//!
//! Two layers of demodulation, which is what makes this different in shape
//! from Mode S and AIS. The channel is ordinary narrowband FM, so the node
//! mixes 144.800 down, filters it to a voice channel, and discriminates it
//! exactly as a listening channel would. The data is then in the *audio*, as
//! Bell 202 tones, and `dsp::afsk` takes it from there.
//!
//! Everything above the tones is shared with AIS: NRZI, HDLC flags, bit
//! destuffing and the X.25 check sequence all live in `dsp::hdlc`, because
//! AX.25 is HDLC. What reaches the bus is an AX.25 frame that has already
//! proved itself.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::aprs::decoded;
pub use decode::aprs::round;
use decode::ax25;
use dsp::afsk::{AfskConfig, AfskDemod};
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::aprs::Aprs;
pub use identify::aprs::CHANNEL_WIDTH_HZ;
pub use identify::aprs::DEFAULT_HZ;
pub use identify::aprs::{AUDIO_HZ, DEVIATION_HZ};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct AprsNode {
    /// The frequency this node is tuned to, which the scanner table sets.
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    afsk: AfskDemod,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    meter: crate::FrameMeter,
    frames: Vec<Vec<u8>>,
    accepted: u64,
}

impl Default for AprsNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl AprsNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            afsk: AfskDemod::new(AUDIO_HZ, AfskConfig::default()),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 2.0),
            frames: Vec::new(),
            accepted: 0,
        }
    }

    /// Frames that passed their check sequence since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for AprsNode {
    fn name(&self) -> &str {
        "aprs"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("aprs reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("aprs needs its channel inside the span"));
        }
        // Decimate to an audio rate the tone correlators can work at. The
        // exact rate follows from the span, so the AFSK side is built from
        // what it will actually be handed rather than from a nominal 48 kHz.
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.afsk = AfskDemod::new(audio_rate, AfskConfig::default());
        // Measured on the channel rather than on the span: a 16 kHz packet
        // channel inside 2.4 MS/s of band is 0.7% of the power, so a level
        // taken before the mixer is a level of everything else.
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
        self.frames.clear();
        let audio = std::mem::take(&mut self.audio);
        self.afsk.process(&audio, &mut self.frames);
        self.audio = audio;

        let out = o.frames_mut();
        for f in &self.frames {
            self.accepted += 1;
            out.push(self.meter.frame(f.clone()));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.afsk.reset();
    }
}

impl Protocol for Aprs {
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

    /// A station beacons where it is, which is what the network is for.
    fn reports_position(&self) -> bool {
        true
    }

    /// The 2 m packet segment, which is inside the VHF paging allocation:
    /// two protocols really do share that spectrum, and the narrower window
    /// is the better claim.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: 2_000_000 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        if !dsp::afsk::is_packet_band(p.center_hz() as f64) {
            return None;
        }
        let center = common::Hz(p.center_hz());
        Some(ax25::parse(bytes).map(|f| vec![decoded(&f, bytes, center)]).unwrap_or_default())
    }

    /// Where APRS is across Europe. North America is 144.390 and Japan
    /// 144.640.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} APRS", hz / 1e6)
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// The frame as Bell 202 audio, then the deviation a 2 m packet channel
    /// is keyed at. The tones are audio, so what puts them on the air is the
    /// same FM modulator a voice channel uses. One source, whether the frame
    /// came from the operator's beacon fields or from a KISS client.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(APRS_TX.name),
            modulator: NodeSpec::new(crate::mod_nodes::FM_MOD.name)
                .f("deviation_hz", DEVIATION_HZ)
                .f("offset_hz", 0.0),
        })
    }
}

/// A frame as Bell 202 audio: what a KISS client sent, or the operator's own
/// beacon.
///
/// The transmit mirror of [`AprsNode`] and built from the same layers read
/// back: [`decode::ax25::encode`] lays out the frame, `dsp::hdlc` stuffs it
/// and adds the check sequence, and `dsp::afsk` turns the bits into the two
/// tones. What leaves is audio, because on a packet channel the data is in
/// the audio and the carrier is ordinary narrowband FM.
///
/// A channel has one transmit source, so this is where both kinds of frame
/// meet. A client's frame goes first: somebody at a keyboard is waiting on
/// it, where a beacon repeats anyway.
///
/// The audio is built once for a whole frame and handed out a block at a
/// time, because a frame is a quarter of a second and a block is a
/// millisecond or two.
pub struct AprsTxNode {
    source: String,
    destination: String,
    path: String,
    info: String,
    rate: f64,
    /// The beacon the fields describe, as audio, or empty for nothing to say.
    beacon: Vec<f32>,
    /// The frame going out now.
    audio: Vec<f32>,
    at: usize,
    /// Whether [`Self::audio`] came from a client rather than the beacon.
    from_client: bool,
    /// Samples of silence left before the beacon goes again.
    rest: usize,
    sent: u64,
    /// Where the KISS TNC listens, and the server once one is running there.
    /// Resolved when a frame is wanted rather than when the node is built,
    /// because the transmit chain is built on its own thread and in no fixed
    /// order against the graph that serves.
    tnc_addr: std::net::SocketAddr,
    tnc: Option<std::sync::Arc<crate::kiss_nodes::Tnc>>,
}

/// Flags in front of a frame, which is what the far end's clock settles on.
/// A tenth of a second at 1200 baud, which is what a TNC sends.
const LEAD_FLAGS: usize = 16;

/// Silence between one beacon and the next while the key is held, in
/// seconds. A tracker beacons every minute or two; this is only what keeps
/// two frames from arriving as one.
const BEACON_GAP_S: f64 = 2.0;

impl Default for AprsTxNode {
    fn default() -> Self {
        Self::new("N0CALL", "APRS", "WIDE1-1", "")
    }
}

impl AprsTxNode {
    pub fn new(source: &str, destination: &str, path: &str, info: &str) -> Self {
        Self {
            source: source.into(),
            destination: destination.into(),
            path: path.into(),
            info: info.into(),
            rate: 0.0,
            beacon: Vec::new(),
            audio: Vec::new(),
            at: 0,
            from_client: false,
            rest: 0,
            sent: 0,
            tnc_addr: crate::kiss_nodes::default_address(),
            tnc: None,
        }
    }

    /// Keying for the TNC at one address, whether or not it is serving yet.
    pub fn keying(addr: std::net::SocketAddr) -> Self {
        let mut node = Self::new("N0CALL", "APRS", "WIDE1-1", "");
        node.tnc_addr = addr;
        node
    }

    /// Keying for one TNC in particular, which is how a test hands it the
    /// server it just started.
    pub fn attach(tnc: std::sync::Arc<crate::kiss_nodes::Tnc>) -> Self {
        let mut node = Self::new("N0CALL", "APRS", "WIDE1-1", "");
        node.tnc_addr = tnc.address();
        node.tnc = Some(tnc);
        node
    }

    /// Frames put on the air since the chain was built.
    pub fn sent(&self) -> u64 {
        self.sent
    }

    /// The TNC, looked up once it is serving.
    pub(crate) fn attached(&mut self) -> Option<&std::sync::Arc<crate::kiss_nodes::Tnc>> {
        if self.tnc.is_none() {
            self.tnc = crate::kiss_nodes::running(self.tnc_addr);
        }
        self.tnc.as_ref()
    }

    /// Lay out the next frame: a client's if one is queued, else the beacon.
    fn load(&mut self) {
        self.at = 0;
        self.audio.clear();
        let queued = self.attached().cloned().and_then(|tnc| {
            let frame = tnc.next_frame()?;
            let flags = tnc
                .params()
                .lead_flags(dsp::afsk::BELL202.baud)
                .max(crate::kiss_nodes::MIN_LEAD_FLAGS);
            Some(dsp::afsk::encode(&frame, self.rate, flags))
        });
        match queued {
            Some(audio) => {
                self.audio = audio;
                self.from_client = true;
            }
            None => {
                self.audio = self.beacon.clone();
                self.from_client = false;
            }
        }
    }

    /// The frame the settings describe, or `None` where a callsign will not
    /// parse or there is nothing to say. Refusing is the point: a station
    /// identifies itself or it does not transmit.
    fn frame(&self) -> Option<Vec<u8>> {
        if self.info.is_empty() {
            return None;
        }
        let path: std::result::Result<Vec<ax25::Address>, ()> =
            self.path.split(',').map(str::trim).filter(|s| !s.is_empty()).map(str::parse).collect();
        Some(ax25::encode(&ax25::ui(
            self.destination.parse().ok()?,
            self.source.parse().ok()?,
            path.ok()?,
            self.info.as_bytes(),
        )))
    }

    /// Build the audio for one beacon at the rate the chain runs at.
    fn reload(&mut self) {
        self.at = 0;
        self.rest = 0;
        self.audio.clear();
        self.beacon = match (self.rate > 0.0, self.frame()) {
            (true, Some(f)) => dsp::afsk::encode(&f, self.rate, LEAD_FLAGS),
            _ => Vec::new(),
        };
    }
}

impl Simple for AprsTxNode {
    fn name(&self) -> &str {
        APRS_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        let what = match self.frame() {
            Some(_) => format!("{} to {}", self.source, self.destination),
            None => "nothing to beacon".into(),
        };
        let (queued, clients) = match &self.tnc {
            Some(t) => (t.queued().to_string(), t.connected().to_string()),
            None => ("no TNC is being served".into(), "0".into()),
        };
        vec![
            ("beaconing".into(), what),
            ("sent".into(), self.sent.to_string()),
            ("queued".into(), queued),
            ("clients".into(), clients),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("aprs_tx needs a clock to key against"));
        }
        self.rate = i.spec.rate;
        self.reload();
        let mut out = i.spec.with_kind(PortKind::Real);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let want = i.len();
        if want == 0 {
            return Ok(());
        }
        let out = o.real_mut();
        let mut left = want;
        while left > 0 {
            if self.rest > 0 {
                let n = self.rest.min(left);
                out.resize(out.len() + n, 0.0);
                self.rest -= n;
                left -= n;
                continue;
            }
            if self.at >= self.audio.len() {
                self.load();
            }
            if self.audio.is_empty() {
                // Nothing to say is silence, not a refusal: the chain is
                // drawn and keyed whether or not there is a beacon in it or
                // a client connected.
                out.resize(out.len() + left, 0.0);
                return Ok(());
            }
            let n = (self.audio.len() - self.at).min(left);
            out.extend_from_slice(&self.audio[self.at..self.at + n]);
            self.at += n;
            left -= n;
            if self.at >= self.audio.len() {
                self.sent += 1;
                // A queue drains back to back; a beacon waits, so two of them
                // do not arrive as one frame.
                self.rest = match self.from_client {
                    true => 0,
                    false => (BEACON_GAP_S * self.rate) as usize,
                };
            }
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::text(SOURCE, self.source.clone()).label("From"),
            Param::text(DESTINATION, self.destination.clone()).label("To"),
            Param::text(PATH, self.path.clone()).label("Path"),
            Param::text(INFO, self.info.clone()).label("Report"),
        ]
    }

    fn set_param(&mut self, name: &str, value: ParamValue) -> Result<()> {
        let v = value.as_str().unwrap_or_default().to_string();
        match name {
            SOURCE => self.source = v,
            DESTINATION => self.destination = v,
            PATH => self.path = v,
            INFO => self.info = v,
            _ => return Err(common::Error::other(format!("aprs_tx: unknown parameter {name:?}"))),
        }
        self.reload();
        Ok(())
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "aprs",
    summary: "One APRS channel: narrowband FM, Bell 202 AFSK, AX.25",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(AprsNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

/// Who is beaconing, to whom, by what path, and what they are saying.
const SOURCE: &str = "source";
const DESTINATION: &str = "destination";
const PATH: &str = "path";
const INFO: &str = "info";

pub const APRS_TX: StageDesc = StageDesc {
    name: "aprs_tx",
    summary: "Beacon an AX.25 UI frame as Bell 202 audio, ready for an FM carrier",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(AprsTxNode::new(
        s.str_or(SOURCE, "N0CALL"),
        s.str_or(DESTINATION, "APRS"),
        s.str_or(PATH, "WIDE1-1"),
        s.str_or(INFO, ""),
    )))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A UI frame with an uncompressed position, as a tracker would send.
    fn ui_frame() -> Vec<u8> {
        let mut f = Vec::new();
        for (call, ssid, last) in [("APRS  ", 0u8, false), ("EI2ABC", 9, true)] {
            for c in call.bytes() {
                f.push(c << 1);
            }
            f.push(0x60 | (ssid << 1) | u8::from(last));
        }
        f.push(0x03);
        f.push(0xF0);
        f.extend_from_slice(b"!5338.00N/00615.00W>088/036on the road");
        f
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = AprsNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, 144_800_000.0)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 150_000_000.0)).is_err());
        assert!(n.negotiate(&spec(25_000.0, 144_800_000.0)).is_ok());
        assert!(n.negotiate(&spec(12_000.0, 144_800_000.0)).is_err());
    }

    /// The whole path on synthetic RF: an FM carrier keyed with Bell 202
    /// tones, into the node, out as a positioned station.
    ///
    /// This is the test that proves the two layers of modulation are the
    /// right way round and that the audio rate the node builds itself for is
    /// the one it is actually handed.
    #[test]
    fn a_modulated_frame_becomes_a_station_at_the_right_place() {
        let frame = ui_frame();
        let (rate, center) = (2_400_000.0, 144_800_000.0);

        // Bell 202 audio, then FM modulated onto the channel.
        let audio = dsp::afsk::encode(&frame, 48_000.0, 16);
        let mut iq = Vec::with_capacity(audio.len() * 50);
        let mut phase = 0.0f64;
        for &a in &audio {
            // Each audio sample held for the decimation factor, which is a
            // crude interpolation and all the discriminator needs.
            for _ in 0..(rate / 48_000.0) as usize {
                phase += std::f64::consts::TAU * (f64::from(a) * DEVIATION_HZ) / rate;
                iq.push(common::C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }

        let mut node = AprsNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); 8192];
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

        assert_eq!(frames.len(), 1, "expected one frame off the air");
        assert_eq!(frames[0], frame, "the frame came back changed");

        let parsed = ax25::parse(&frames[0]).expect("an AX.25 frame");
        assert_eq!(parsed.source.to_string(), "EI2ABC-9");
        let d = decoded(&parsed, &frames[0], Hz(144_800_000));
        assert_eq!(d.protocol, "APRS-Position");
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("lat"), Some(common::Value::Float(53.63333)));
        assert_eq!(get("lon"), Some(common::Value::Float(-6.25)));
        assert_eq!(get("from"), Some(common::Value::Text("EI2ABC-9".into())));
    }

    /// The transmitter into the receiver: a beacon keyed by the chain the
    /// protocol declares, read back by the node that reads real stations,
    /// with the check sequence the receiver insists on computed over what
    /// was actually sent.
    #[test]
    fn a_beacon_keyed_by_the_transmit_chain_is_read_back() {
        let (rate, center) = (96_000.0, 144_800_000.0);
        let report = "!5338.00N/00615.00W-waveshark";
        // A 45-byte frame with 16 lead flags is 500 bits at 1200 baud,
        // which is 0.42 s; a second holds one beacon and part of the gap
        // after it.
        let air = crate::tx_nodes::transmit_for(
            &Aprs,
            rate,
            Hz(center as u64),
            1.0,
            &[
                (SOURCE, ParamValue::Text("MI0ABC-9".into())),
                (PATH, ParamValue::Text("WIDE1-1".into())),
                (INFO, ParamValue::Text(report.into())),
            ],
        );
        assert_eq!(air.len(), 98_304, "a second of samples at {rate}");

        let mut node = AprsNode::new(center);
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames: Vec<Vec<u8>> = Vec::new();
        for chunk in air.chunks(4_096) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes));
            }
        }

        assert_eq!(frames.len(), 1, "one beacon in a second");
        let parsed = ax25::parse(&frames[0]).expect("an AX.25 frame");
        assert_eq!(parsed.source.to_string(), "MI0ABC-9");
        assert_eq!(parsed.destination.to_string(), "APRS");
        assert_eq!(parsed.path.len(), 1);
        assert_eq!(parsed.path[0].to_string(), "WIDE1-1");
        assert_eq!(parsed.info, report.as_bytes());
        let d = decoded(&parsed, &frames[0], Hz(center as u64));
        assert_eq!(d.protocol, "APRS-Position");
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("lat"), Some(common::Value::Float(53.63333)));
        assert_eq!(get("lon"), Some(common::Value::Float(-6.25)));
        assert_eq!(get("from"), Some(common::Value::Text("MI0ABC-9".into())));
    }

    /// A station with nothing to say transmits nothing, rather than keying a
    /// carrier or a frame with an empty report in it.
    #[test]
    fn a_beacon_with_no_report_is_silence() {
        let rate = 96_000.0;
        let mut n = AprsTxNode::default();
        n.negotiate(&PortSpec {
            spec: StreamSpec { kind: PortKind::Real, rate, ..Default::default() },
            latency: 0,
        })
        .unwrap();
        let ins = [spec(rate, DEFAULT_HZ)];
        let tags = Vec::new();
        let mut out = Payload::empty_of(PortKind::Real);
        let (mut events, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Simple::process(&mut n, &Payload::Real(vec![0.0; 4_096]), &mut out, &mut ctx).unwrap();
        let audio = out.as_real().unwrap();
        assert_eq!(audio.len(), 4_096, "a block of time is a block of audio");
        assert!(audio.iter().all(|&v| v == 0.0), "silence, not a carrier");
    }

    /// AX.25 that is not APRS still reaches the log, because it passed a real
    /// integrity check and something is out there transmitting it.
    #[test]
    fn a_non_aprs_frame_is_still_reported() {
        let mut f = ui_frame();
        // An information frame rather than an unnumbered one, which is AX.25
        // carrying something that is not APRS at all.
        f[14] = 0x00;
        let parsed = ax25::parse(&f).unwrap();
        assert!(!parsed.is_ui());
        let d = decoded(&parsed, &f, Hz(144_800_000));
        assert_eq!(d.protocol, "AX25");
        assert_eq!(d.crc_ok, Some(true));
    }
}
