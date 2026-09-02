//! M17 as a graph node.
//!
//! The same shape as the pager and APRS front ends: the channel is ordinary
//! narrowband FM, so the node mixes it down, filters it to a voice channel and
//! discriminates it. What comes out of the discriminator is four-level FSK at
//! 4800 baud rather than audio, and `dsp::m17` reads the frames out of it.
//! Putting the transmissions back together, six link information chunks into a
//! link setup frame, 25 byte fragments into a packet, is `decode::m17`.
//!
//! What reaches the bus is a whole transmission rather than a frame: a voice
//! stream is 25 frames a second and none of them means anything on its own,
//! whereas "M0ABC called M17-M17 C for nine seconds" is one row in a log.

use common::Result;
use decode::m17::{self, Assembler, DataType, Event};
use dsp::m17::{Frame, M17Config, M17Demod, CHANNEL_WIDTH_HZ as OCCUPIED_HZ, DEVIATION_HZ};
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::{media, Decoded};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// The M17 calling frequency in Region 1, and only the default the node is
/// built with before the scanner table says where to listen.
pub const DEFAULT_HZ: f64 = 433_475_000.0;

/// The channel an M17 transmission occupies. The signal is 9 kHz wide and the
/// allocations are on a 12.5 kHz grid, so this is the grid rather than the
/// signal: it is what decides whether a channel fits inside the span.
pub const CHANNEL_WIDTH_HZ: f64 = 12_500.0;

/// Audio rate the discriminator output is decimated to. Ten samples per
/// symbol at 4800 baud, which is what the specification recommends for the
/// shaping filter either end.
const AUDIO_HZ: f64 = 48_000.0;

pub struct M17Node {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    demod: M17Demod,
    assembler: Assembler,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    frames: Vec<Frame>,
    /// Audio samples since the node started, which is the clock the assembler
    /// closes a transmission on.
    samples: u64,
    accepted: u64,
}

impl Default for M17Node {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl M17Node {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, OCCUPIED_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            demod: M17Demod::new(AUDIO_HZ, M17Config::default()),
            assembler: Assembler::new(AUDIO_HZ),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            frames: Vec::new(),
            samples: 0,
            accepted: 0,
        }
    }

    /// Transmissions reported since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }
}

impl Simple for M17Node {
    fn name(&self) -> &str {
        "m17"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("m17 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ {
            return Err(common::Error::other("m17 needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, OCCUPIED_HZ / 2.0, 60.0);
        // Scaled so the outer symbols land near ±1. Nothing downstream
        // depends on that, since the symbol slicer fits its own levels per
        // frame, but it keeps the numbers in the range everything else here
        // works in.
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.demod = M17Demod::new(audio_rate, M17Config::default());
        self.assembler = Assembler::new(audio_rate);
        self.samples = 0;

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
        self.samples += self.audio.len() as u64;

        self.frames.clear();
        let audio = std::mem::take(&mut self.audio);
        let mut frames = std::mem::take(&mut self.frames);
        self.demod.process(&audio, &mut frames);
        self.audio = audio;

        let mut events = Vec::new();
        for f in &frames {
            events.extend(self.assembler.push(f));
        }
        self.frames = frames;
        // A transmission that stopped mid-stream ends when nothing more is
        // heard, so the assembler has to be told the time even when no frame
        // arrived.
        events.extend(self.assembler.poll(self.samples));

        let out = o.frames_mut();
        for e in &events {
            self.accepted += 1;
            out.push(e.to_bytes());
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.demod.reset();
        self.assembler = Assembler::new(AUDIO_HZ);
    }
}

/// The row a transmission becomes.
pub fn m17_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let event = Event::parse(bytes)?;
    let lsf = match &event {
        Event::LinkSetup { lsf, .. } => Some(lsf),
        Event::Packet { lsf, .. } | Event::Stream { lsf, .. } => lsf.as_ref(),
    };
    let mut fields: Vec<(String, Value)> = lsf.map(m17::fields).unwrap_or_default();
    let mut text = None;

    let protocol = match &event {
        Event::LinkSetup { late, .. } => {
            if *late {
                // Rebuilt from the link information channel rather than heard
                // outright, which is worth saying: it means the receiver
                // joined the transmission after it started.
                fields.push(("late_entry".into(), Value::Bool(true)));
            }
            "M17-Setup"
        }
        Event::Packet { data, .. } => {
            let (id, payload) = data.split_first().unwrap_or((&0, &[]));
            if let Some(name) = m17::packet_protocol(*id) {
                fields.push(("packet_type".into(), Value::Text(name.into())));
            } else {
                fields.push(("packet_type".into(), Value::Int(i64::from(*id))));
            }
            fields.push(("bytes".into(), Value::Int(data.len() as i64)));
            // SMS is a null-terminated UTF-8 string, and every other type may
            // or may not be text. Only the one the specification says is text
            // is shown as text.
            if *id == 0x05 {
                let s = String::from_utf8_lossy(payload).trim_end_matches('\0').to_string();
                fields.push(("message".into(), Value::Text(s.clone())));
                text = Some(s);
            }
            "M17-Packet"
        }
        Event::Stream { frames, complete, .. } => {
            fields.push(("frames".into(), Value::Int(i64::from(*frames))));
            // 40 ms a frame, which is the only duration in the protocol that
            // needs no clock to measure.
            fields.push(("seconds".into(), Value::Float(f64::from(*frames) * 0.04)));
            if !complete {
                fields.push(("truncated".into(), Value::Bool(true)));
            }
            match lsf.map(|l| l.data_type()) {
                Some(DataType::Voice) | Some(DataType::VoiceData) => "M17-Voice",
                _ => "M17-Stream",
            }
        }
    };

    let detail = fields.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join(" ");
    let mut d = Decoded::bytes(protocol, center, 0.0, bytes.to_vec())
        .with_detail(detail)
        .with_fields(fields)
        .with_modulation("4FSK")
        // Every link setup here passed the CRC over its 28 bytes and every
        // packet the CRC over the whole of it; a transmission whose checks
        // failed never became an event.
        .with_crc(Some(true));
    if let Some(t) = text {
        d = d.with_media(media::TEXT).with_text(t);
    }
    Some(d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::m17::Address;
    use dsp::m17::{fec, frame_symbols, preamble_symbols, Kind, BAUD};

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = M17Node::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 145_000_000.0)).is_err());
        assert!(n.negotiate(&spec(20_000.0, DEFAULT_HZ)).is_err());
    }

    fn link_setup(dst: &str, src: &str, type_field: u16) -> [u8; 30] {
        let mut b = [0u8; 30];
        b[..6].copy_from_slice(&Address::encode(dst).to_be_bytes()[2..]);
        b[6..12].copy_from_slice(&Address::encode(src).to_be_bytes()[2..]);
        b[12..14].copy_from_slice(&type_field.to_be_bytes());
        let crc = fec::crc16(&b[..28]);
        b[28..].copy_from_slice(&crc.to_be_bytes());
        b
    }

    /// Symbols keyed onto a carrier at the span's rate: 4-FSK with the outer
    /// levels at 2.4 kHz, which is what the radio puts on the air.
    fn modulate(symbols: &[f32], rate: f64, center_offset: f64) -> Vec<common::C32> {
        let sps = rate / BAUD;
        let mut phase = 0.0f64;
        let mut iq = Vec::with_capacity((symbols.len() as f64 * sps) as usize);
        for &s in symbols {
            let f = center_offset + f64::from(s) / 3.0 * DEVIATION_HZ;
            for _ in 0..sps as usize {
                phase += std::f64::consts::TAU * f / rate;
                iq.push(common::C32::new(phase.cos() as f32, phase.sin() as f32));
            }
        }
        iq
    }

    fn run(iq: &[common::C32], rate: f64, center: f64) -> Vec<Vec<u8>> {
        let mut node = M17Node::default();
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); (rate * 0.5) as usize];
        for block in [&quiet[..], iq, &quiet[..]] {
            for chunk in block.chunks(65_536) {
                let input = Payload::Iq(chunk.to_vec());
                let mut out = Payload::Frames(Vec::new());
                let (mut events, mut new_tags) = (Vec::new(), Vec::new());
                let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
                node.process(&input, &mut out, &mut ctx).unwrap();
                if let Payload::Frames(f) = out {
                    frames.extend(f);
                }
            }
        }
        frames
    }

    /// The whole path on synthetic RF: a voice transmission keyed onto a
    /// carrier, through the node, out as a link setup and a stream.
    ///
    /// Each layer is tested on its own and each could be self-consistently
    /// wrong. What this adds is that the frequencies, the filter and the
    /// discriminator in front of the demodulator leave symbols it can read at
    /// all, which no test of the demodulator alone can show.
    #[test]
    fn a_transmission_off_the_air_becomes_a_call() {
        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let setup = link_setup("M17-M17 C", "M0ABC", 1 | 2 << 1 | 5 << 7);
        let mut symbols = preamble_symbols();
        symbols.extend(frame_symbols(Kind::Lsf, &setup, 0, &[0; 6]));
        for n in 0..25u16 {
            let cnt = (n % 6) as usize;
            let mut lich = [0u8; 6];
            lich[..5].copy_from_slice(&setup[cnt * 5..cnt * 5 + 5]);
            lich[5] = (cnt as u8) << 5;
            let payload = [0x55u8; 16];
            symbols.extend(frame_symbols(Kind::Stream, &payload, n, &lich));
        }

        // A kilohertz off frequency, because a handheld is.
        let iq = modulate(&symbols, rate, 1_000.0);
        let frames = run(&iq, rate, center);
        let rows: Vec<Decoded> =
            frames.iter().filter_map(|f| m17_decoded(f, Hz(center as u64))).collect();
        assert_eq!(rows.len(), 2, "expected a setup row and a stream row: {rows:?}");

        assert_eq!(rows[0].protocol, "M17-Setup");
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone())
        };
        assert_eq!(get(&rows[0], "from"), Some(common::Value::Text("M0ABC".into())));
        assert_eq!(get(&rows[0], "to"), Some(common::Value::Text("M17-M17 C".into())));
        assert_eq!(get(&rows[0], "mode"), Some(common::Value::Text("voice".into())));
        assert_eq!(get(&rows[0], "can"), Some(common::Value::Int(5)));
        assert_eq!(rows[0].crc_ok, Some(true));

        assert_eq!(rows[1].protocol, "M17-Voice");
        assert_eq!(get(&rows[1], "frames"), Some(common::Value::Int(25)));
        assert_eq!(get(&rows[1], "seconds"), Some(common::Value::Float(1.0)));
    }

    /// A text message sent in packet mode, which is the other thing an M17
    /// radio transmits and the one that carries readable content.
    #[test]
    fn a_packet_transmission_becomes_a_message() {
        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let setup = link_setup("BROADCAST", "M0ABC", 0);
        let mut data = vec![0x05u8];
        data.extend_from_slice(b"CQ CQ CQ de M0ABC, testing M17 packet mode");
        data.push(0);
        let crc = fec::crc16(&data);
        data.extend_from_slice(&crc.to_be_bytes());

        let mut symbols = preamble_symbols();
        symbols.extend(frame_symbols(Kind::Lsf, &setup, 0, &[0; 6]));
        for (i, chunk) in data.chunks(25).enumerate() {
            let mut contents = [0u8; 26];
            contents[..chunk.len()].copy_from_slice(chunk);
            let eof = (i + 1) * 25 >= data.len();
            let counter = if eof { chunk.len() as u8 } else { i as u8 };
            contents[25] = u8::from(eof) << 7 | counter << 2;
            symbols.extend(frame_symbols(Kind::Packet, &contents, 0, &[0; 6]));
        }

        let frames = run(&modulate(&symbols, rate, 0.0), rate, center);
        let rows: Vec<Decoded> =
            frames.iter().filter_map(|f| m17_decoded(f, Hz(center as u64))).collect();
        let packet = rows.iter().find(|r| r.protocol == "M17-Packet").expect("no packet row");
        assert_eq!(packet.text.as_deref(), Some("CQ CQ CQ de M0ABC, testing M17 packet mode"));
        assert_eq!(packet.media_type, media::TEXT);
        assert_eq!(
            packet.fields.iter().find(|(n, _)| n == "packet_type").map(|(_, v)| v.clone()),
            Some(common::Value::Text("SMS".into()))
        );
    }
}
