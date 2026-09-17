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
use decode::{aprs, ax25};
use dsp::afsk::{AfskConfig, AfskDemod};
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Where APRS is across Europe. North America uses 144.390 and Japan 144.640;
/// the scanner configuration decides which, and this is only the default the
/// node is built with before it is told.
pub const DEFAULT_HZ: f64 = 144_800_000.0;

/// The channel a 2 m packet transmission occupies.
pub const CHANNEL_WIDTH_HZ: f64 = 16_000.0;

/// Audio rate the discriminator output is decimated to. Comfortably above the
/// 2200 Hz upper tone and a rate the correlators are tested at.
const AUDIO_HZ: f64 = 48_000.0;

/// Peak deviation a 2 m packet channel uses.
const DEVIATION_HZ: f64 = 3_000.0;

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

/// The decode an AX.25 frame becomes.
pub fn aprs_decoded(frame: &ax25::Frame, bytes: &[u8], center: common::Hz) -> Decoded {
    use common::Value;
    let mut fields: Vec<(String, Value)> = Vec::new();
    fields.push(("from".into(), Value::Text(frame.source.to_string())));
    fields.push(("to".into(), Value::Text(frame.destination.to_string())));
    if !frame.path.is_empty() {
        let path: Vec<String> = frame.path.iter().map(|a| a.to_string()).collect();
        fields.push(("path".into(), Value::Text(path.join(","))));
    }

    // The destination is not only an address: Mic-E hides half its latitude
    // in there, so the payload cannot be read without it.
    let aprs_report =
        frame.is_ui().then(|| aprs::parse(&frame.info, &frame.destination.call)).flatten();

    let mut fix = None;
    let mut media = pipeline::event::media::BYTES;
    let mut written = false;
    let mut report = common::ReportDetail::Bare;
    let protocol = match &aprs_report {
        Some(aprs::Report::Position { position, comment }) => {
            fix = Some(common::Position {
                lat: position.lat,
                lon: position.lon,
                altitude_m: position.altitude_ft.map(|f| f64::from(f) * 0.3048),
                speed_kt: position.speed_kt,
                course_deg: position.course_deg,
            });
            report = common::ReportDetail::Aprs {
                symbol_table: position.symbol_table,
                symbol_code: position.symbol_code,
                comment: comment.clone(),
            };
            fields.push(("lat".into(), Value::Float(round(position.lat, 5))));
            fields.push(("lon".into(), Value::Float(round(position.lon, 5))));
            if let Some(v) = position.course_deg {
                fields.push(("track_deg".into(), Value::Float(v)));
            }
            if let Some(v) = position.speed_kt {
                fields.push(("ground_speed_kt".into(), Value::Float(v)));
            }
            if let Some(v) = position.altitude_ft {
                fields.push(("altitude_ft".into(), Value::Int(i64::from(v))));
            }
            if let Some(c) = comment {
                fields.push(("comment".into(), Value::Text(c.clone())));
            }
            "APRS-Position"
        }
        Some(aprs::Report::Status(s)) => {
            fields.push(("status".into(), Value::Text(s.clone())));
            "APRS-Status"
        }
        Some(aprs::Report::Message { to, text }) => {
            fields.push(("addressee".into(), Value::Text(to.clone())));
            fields.push(("message".into(), Value::Text(text.clone())));
            media = pipeline::event::media::TEXT;
            // A message is addressed to a station and was typed by whoever
            // sent it. A position, a status and a telemetry frame are the
            // radio talking about itself.
            written = true;
            "APRS-Message"
        }
        Some(aprs::Report::Other(k)) => {
            fields.push(("data_type".into(), Value::Text(k.to_string())));
            "APRS-Other"
        }
        // Plenty of AX.25 is not APRS at all, and a frame that reached here
        // passed its check sequence, so it is reported rather than dropped.
        None => "AX25",
    };

    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
        // A callsign is both the address and the name: there is nothing else
        // to call an APRS station.
        .by(common::Identity::new("aprs", frame.source.to_string()).named(frame.source.to_string()))
        // The AX.25 addresses. A destination on APRS is usually a software
        // identifier rather than a station, which is why it is a group: it
        // is a label many senders share, not somebody listening.
        .with_link(pipeline::event::Link::between(
            pipeline::event::Party::unit(frame.source.to_string()),
            pipeline::event::Party::group(frame.destination.to_string()),
        ))
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation(common::Modulation::Afsk)
        // Every frame here passed the X.25 frame check sequence in the
        // demodulator, which is a real integrity check.
        .with_crc(Some(true));
    d.position = fix;
    d.report = report;
    d.media_type = media;
    d.written = written;
    d
}

fn round(v: f64, places: i32) -> f64 {
    let f = 10f64.powi(places);
    (v * f).round() / f
}

pub struct Aprs;

/// A beacon, as Bell 202 audio ready for the FM modulator.
///
/// The mirror of [`AprsNode`] and the same two layers of modulation read
/// backwards: `decode::ax25::encode` builds the frame, `dsp::afsk::encode`
/// adds the check sequence, the flags and the NRZI and turns the bits into
/// tones, and the modulator puts the tones on a carrier. Audio rather than
/// timings, because Bell 202 is two tones inside a voice channel and not two
/// carriers: an FSK modulator here would key the radio 1200 Hz either side of
/// the channel and no packet station would hear it.
pub struct AprsTxNode {
    source: ax25::Address,
    destination: ax25::Address,
    path: Vec<ax25::Address>,
    info: String,
    rate: f64,
    pace: crate::tx_source::Pace,
}

/// What a station with no software of its own puts in the destination, as
/// the APRS specification's tocall registry has it for an unknown
/// experimental transmitter.
const DEFAULT_TOCALL: &str = "APZ001";

/// Flags sent before the frame, to give the receiver's tone correlators and
/// bit clock something to lock to. The convention is a third of a second,
/// which at 1200 baud and eight bits a flag is fifty of them; sixteen is what
/// a tracker sends and what the tests here read back.
const LEAD_FLAGS: usize = 16;

impl Default for AprsTxNode {
    fn default() -> Self {
        Self {
            source: ax25::Address { call: "N0CALL".into(), ssid: 0, repeated: false },
            destination: ax25::Address { call: DEFAULT_TOCALL.into(), ssid: 0, repeated: false },
            path: Vec::new(),
            info: String::new(),
            rate: 0.0,
            pace: crate::tx_source::Pace::default(),
        }
    }
}

impl AprsTxNode {
    pub fn new(source: &str, info: &str) -> Self {
        let mut n = Self { info: info.into(), ..Default::default() };
        if let Ok(a) = source.parse() {
            n.source = a;
        }
        n
    }

    /// The frame this stage would send, before any modulation.
    pub fn frame(&self) -> Vec<u8> {
        ax25::encode(&self.destination, &self.source, &self.path, self.info.as_bytes())
    }

    pub fn sent(&self) -> u64 {
        self.pace.sent()
    }
}

impl Simple for AprsTxNode {
    fn name(&self) -> &str {
        APRS_TX.name
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            ("as".into(), self.source.to_string()),
            ("beacons".into(), self.pace.sent().to_string()),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("aprs_tx needs a clock to key against"));
        }
        self.rate = i.spec.rate;
        let mut out = i.spec.with_kind(PortKind::Real);
        out.flow = pipeline::port::Flow::Tx;
        out.channels = 1;
        // The tones and the sidebands the keying puts around them, which is
        // what the modulator adds to its deviation under Carson's rule.
        out.bandwidth = 2.0 * (dsp::afsk::SPACE_HZ + dsp::afsk::BAUD);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.pace.clock(i.len(), self.rate);
        if !self.pace.due() || self.info.is_empty() {
            return Ok(());
        }
        let audio = dsp::afsk::encode(&self.frame(), self.rate, LEAD_FLAGS);
        self.pace.spent(audio.len() as f64 * 1e6 / self.rate);
        o.real_mut().extend_from_slice(&audio);
        Ok(())
    }

    fn reset(&mut self) {
        self.pace.reset();
    }

    fn params(&self) -> Vec<pipeline::param::Param> {
        use pipeline::param::Param;
        let path: Vec<String> = self.path.iter().map(|a| a.to_string()).collect();
        vec![
            Param::text(SOURCE, self.source.to_string()).label("Callsign"),
            Param::text(DESTINATION, self.destination.to_string()).label("To"),
            Param::text(PATH, path.join(",")).label("Path"),
            Param::text(INFO, self.info.clone()).label("Packet"),
            Param::float(PAUSE_MS, self.pace.pause_ms(), 0.0..=600_000.0)
                .label("Between beacons")
                .unit("ms"),
        ]
    }

    fn set_param(&mut self, name: &str, value: pipeline::param::ParamValue) -> Result<()> {
        use pipeline::param::ParamValue;
        let text = match (&value, name) {
            (ParamValue::Text(t), _) => t.clone(),
            (_, PAUSE_MS) => String::new(),
            _ => return Err(common::Error::other(format!("aprs_tx: {name:?} is text"))),
        };
        match name {
            SOURCE => {
                self.source = text
                    .parse()
                    .map_err(|_| common::Error::other("aprs_tx: a callsign and an SSID"))?
            }
            DESTINATION => {
                self.destination = text
                    .parse()
                    .map_err(|_| common::Error::other("aprs_tx: a callsign and an SSID"))?
            }
            // A path is written the way a log prints it, `WIDE1-1,WIDE2-1`,
            // and an address in it that will not parse is refused rather
            // than dropped: a beacon sent down half a path is a beacon
            // repeated somewhere the operator did not ask for.
            PATH => {
                let mut path = Vec::new();
                for part in text.split(',').map(str::trim).filter(|p| !p.is_empty()) {
                    path.push(
                        part.parse::<ax25::Address>()
                            .map_err(|_| common::Error::other(format!("aprs_tx: {part:?}")))?,
                    );
                }
                self.path = path;
            }
            INFO => self.info = text,
            PAUSE_MS => self.pace.set_pause_ms(value.as_f64().unwrap_or(30_000.0)),
            _ => return Err(common::Error::other(format!("aprs_tx: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

impl Protocol for Aprs {
    fn id(&self) -> &'static str {
        "aprs"
    }
    /// A station beacons where it is, which is what the network is for.
    fn reports_position(&self) -> bool {
        true
    }
    fn label(&self) -> &'static str {
        "aprs"
    }
    fn placement(&self) -> Placement {
        Placement::Anywhere
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
        Some(ax25::parse(bytes).map(|f| vec![aprs_decoded(&f, bytes, center)]).unwrap_or_default())
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
    /// Where APRS is across Europe. North America is 144.390 and Japan
    /// 144.640.
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} APRS", hz / 1e6)
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
    /// The beacon into the FM modulator, at the deviation the receiver's
    /// discriminator is scaled for.
    fn transmit(&self) -> Option<crate::protocol::TxChain> {
        Some(crate::protocol::TxChain {
            source: NodeSpec::new(APRS_TX.name),
            modulator: NodeSpec::new(crate::mod_nodes::FM_MOD.name).f("deviation_hz", DEVIATION_HZ),
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
        let d = aprs_decoded(&parsed, &frames[0], Hz(144_800_000));
        assert_eq!(d.protocol, "APRS-Position");
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("lat"), Some(common::Value::Float(53.63333)));
        assert_eq!(get("lon"), Some(common::Value::Float(-6.25)));
        assert_eq!(get("from"), Some(common::Value::Text("EI2ABC-9".into())));
    }

    /// Beaconed by the transmit stage, modulated, and read back as a
    /// positioned station: the frame builder, the check sequence, the tones
    /// and the discriminator all agree or the callsign does not come back.
    #[test]
    fn a_beacon_this_receiver_sent_is_a_station_this_receiver_reads() {
        let (rate, center) = (192_000.0, DEFAULT_HZ);
        let mut tx = AprsTxNode::new("EI2ABC-9", "!5338.00N/00615.00W>088/036on the road");
        tx.set_param(PATH, pipeline::param::ParamValue::Text("WIDE1-1,WIDE2-1".into())).unwrap();
        let audio_spec = tx.negotiate(&spec(rate, center)).unwrap();
        assert_eq!(audio_spec.kind, PortKind::Real);

        let ins = [spec(rate, center)];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let mut audio = Payload::Real(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(&mut tx, &Payload::Real(vec![0.0; 4096]), &mut audio, &mut ctx).unwrap();
        assert_eq!(tx.sent(), 1);
        // Two addresses, two digipeaters, a control byte and a PID, then the
        // position report.
        assert_eq!(tx.frame().len(), 4 * 7 + 2 + 38);

        let mut modulator = crate::mod_nodes::FmModNode::new(0.0, DEVIATION_HZ, 0.5);
        modulator.negotiate(&PortSpec { spec: audio_spec, latency: 0 }).unwrap();
        let mut iq = Payload::Iq(Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
        Simple::process(&mut modulator, &audio, &mut iq, &mut ctx).unwrap();
        let iq = match iq {
            Payload::Iq(v) => v,
            _ => unreachable!("a modulator produces baseband"),
        };

        let mut node = AprsNode::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let quiet = vec![common::C32::new(0.0, 0.0); 8192];
        let mut frames: Vec<Vec<u8>> = Vec::new();
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            let mut out = Payload::Frames(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(&mut node, &Payload::Iq(block.to_vec()), &mut out, &mut ctx).unwrap();
            if let Payload::Frames(f) = out {
                frames.extend(f.into_iter().map(|x| x.bytes));
            }
        }

        assert_eq!(frames.len(), 1, "{} frames off the air", frames.len());
        assert_eq!(frames[0], tx.frame(), "the frame came back changed");
        let parsed = ax25::parse(&frames[0]).expect("an AX.25 frame");
        assert_eq!(parsed.source.to_string(), "EI2ABC-9");
        assert_eq!(parsed.destination.to_string(), "APZ001");
        assert_eq!(parsed.path.len(), 2);
        assert_eq!(parsed.path[1].to_string(), "WIDE2-1");
        let d = aprs_decoded(&parsed, &frames[0], Hz(center as u64));
        assert_eq!(d.protocol, "APRS-Position");
        let get = |k: &str| d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone());
        assert_eq!(get("lat"), Some(common::Value::Float(53.63333)));
        assert_eq!(get("lon"), Some(common::Value::Float(-6.25)));
    }

    /// A path that will not parse is refused rather than dropped: a beacon
    /// sent down half a path is repeated somewhere nobody asked for.
    #[test]
    fn a_path_that_will_not_parse_is_refused() {
        use pipeline::param::ParamValue;
        let mut n = AprsTxNode::default();
        assert!(n.set_param(PATH, ParamValue::Text("WIDE1-1,NOTACALLSIGN".into())).is_err());
        assert!(n.path.is_empty(), "nothing of a bad path is kept");
        assert!(n.set_param(SOURCE, ParamValue::Text("EI2ABC-99".into())).is_err());
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
        let d = aprs_decoded(&parsed, &f, Hz(144_800_000));
        assert_eq!(d.protocol, "AX25");
        assert_eq!(d.crc_ok, Some(true));
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

/// What the transmit side is set with.
const SOURCE: &str = "source";
const DESTINATION: &str = "destination";
const PATH: &str = "path";
const INFO: &str = "info";
const PAUSE_MS: &str = "pause_ms";

pub const DESC: StageDesc = StageDesc {
    name: "aprs",
    summary: "One APRS channel: narrowband FM, Bell 202 AFSK, AX.25",
    category: Category::Decode,
    feeds_bus: true,
};

pub const APRS_TX: StageDesc = StageDesc {
    name: "aprs_tx",
    summary: "Beacon an APRS packet: an AX.25 UI frame as Bell 202 tones",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(AprsNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

pub fn build_tx(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = AprsTxNode::new(s.str_or(SOURCE, "N0CALL"), s.str_or(INFO, ""));
    if let Ok(a) = s.str_or(DESTINATION, DEFAULT_TOCALL).parse() {
        n.destination = a;
    }
    n.path = s
        .str_or(PATH, "")
        .split(',')
        .map(str::trim)
        .filter(|p| !p.is_empty())
        .filter_map(|p| p.parse().ok())
        .collect();
    // A beacon every half minute, which is what a tracker in a vehicle
    // sends and far slower than the pace a packet source defaults to.
    n.pace.set_pause_ms(s.f64_or(PAUSE_MS, 30_000.0));
    Ok(Box::new(n))
}
