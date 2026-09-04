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

use common::Result;
use decode::{aprs, ax25};
use dsp::afsk::{AfskConfig, AfskDemod};
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

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

    fn channels(&self) -> &'static [f64] {
        &[CHANNEL_WIDTH_HZ]
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

        self.frames.clear();
        let audio = std::mem::take(&mut self.audio);
        self.afsk.process(&audio, &mut self.frames);
        self.audio = audio;

        let out = o.frames_mut();
        for f in &self.frames {
            self.accepted += 1;
            out.push(f.clone());
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
    let report = frame
        .is_ui()
        .then(|| aprs::parse(&frame.info, &frame.destination.call))
        .flatten();

    let protocol = match &report {
        Some(aprs::Report::Position { position, comment }) => {
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
    Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation("AFSK")
        // Every frame here passed the X.25 frame check sequence in the
        // demodulator, which is a real integrity check.
        .with_crc(Some(true))
}

fn round(v: f64, places: i32) -> f64 {
    let f = 10f64.powi(places);
    (v * f).round() / f
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
                frames.extend(f);
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
