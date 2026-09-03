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

use codec2::{Codec2, Codec2Mode};
use common::Result;
use decode::m17::{self, Assembler, DataType, Event};
use dsp::m17::{Body, Frame, M17Config, M17Demod, CHANNEL_WIDTH_HZ as OCCUPIED_HZ, DEVIATION_HZ};
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::event::{media, Decoded};
use pipeline::node::{Node, NodeCtx, PortSpec};
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

/// Rate Codec 2 speaks. Everything downstream resamples from this rather than
/// the codec being asked for something it does not do.
pub const VOICE_HZ: f64 = 8_000.0;

/// Bytes of Codec 2 3200 in half a stream frame. A stream frame carries 128
/// bits of payload, which at 3200 bit/s is two 20 ms codec frames.
const C2_FRAME_BYTES: usize = 8;

/// Longest transmission kept as audio, in seconds.
///
/// A held microphone is minutes long and nobody replays it from a packet list.
/// At 8 kHz mono this cap is about 4 MB, and the cut is at the end rather than
/// the start: what is worth hearing is how the call began.
const MAX_VOICE_SECONDS: f64 = 120.0;

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
    /// The vocoder, and the speech of the transmission being heard now.
    ///
    /// Held across blocks because a transmission spans many of them, and the
    /// codec carries state between frames: decoding each block with a fresh
    /// one would restart the synthesiser twenty-five times a second.
    codec: Codec2,
    voice: Vec<f32>,
    /// Speech decoded in this block, for anything listening live.
    voice_now: Vec<f32>,
    /// Whether the payload being decoded is voice at all. A data stream is
    /// 128 bits of something else, and running the vocoder over it produces
    /// noise that sounds like a fault.
    voice_stream: bool,
    /// Who is talking and who to, from the link setup of the transmission in
    /// progress. What a listener subscribes by.
    talking: Option<(String, String)>,
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
            codec: Codec2::new(Codec2Mode::MODE_3200),
            voice: Vec::new(),
            voice_now: Vec::new(),
            voice_stream: false,
            talking: None,
            samples: 0,
            accepted: 0,
        }
    }

    /// Transmissions reported since the node was built.
    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// Speech decoded in the last block, at [`VOICE_HZ`], for listening live.
    ///
    /// Empty when nothing is transmitting or when the stream is not voice.
    pub fn voice_now(&self) -> &[f32] {
        &self.voice_now
    }

    /// The channel this node is listening on.
    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    /// The source and destination of the transmission being heard, while one
    /// is in progress.
    pub fn talking(&self) -> Option<(&str, &str)> {
        self.talking.as_ref().map(|(f, t)| (f.as_str(), t.as_str()))
    }

    /// Decode both Codec 2 frames in one stream payload.
    fn decode_voice(&mut self, payload: &[u8; 16]) {
        if !self.voice_stream {
            return;
        }
        let cap = (MAX_VOICE_SECONDS * VOICE_HZ) as usize;
        let mut pcm = [0i16; 160];
        for half in payload.chunks_exact(C2_FRAME_BYTES) {
            self.codec.decode(&mut pcm, half);
            for s in pcm {
                let v = s as f32 / 32768.0;
                self.voice_now.push(v);
                if self.voice.len() < cap {
                    self.voice.push(v);
                }
            }
        }
    }

    /// The speech of the transmission that just ended, if it had any.
    fn take_voice(&mut self) -> Option<std::sync::Arc<common::Speech>> {
        if self.voice.is_empty() {
            return None;
        }
        let pcm = std::mem::take(&mut self.voice);
        Some(std::sync::Arc::new(common::Speech { pcm, rate: VOICE_HZ }))
    }
}

/// Two outputs: what was decoded, and what it sounded like.
///
/// The speech is a port rather than something a listener reaches in and
/// reads, because a receiver's audio path is part of what it is doing and
/// belongs in the drawing of it. It cannot share the packet port: a packet is
/// a conclusion that travels once, and this is forty milliseconds of a
/// conversation that has to reach the mixer while it is still worth hearing.
const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;

impl Node for M17Node {
    fn name(&self) -> &str {
        "m17"
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("m17 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
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

        // Packets rather than frames, because a voice transmission carries
        // its speech as well as its bytes and only a packet has somewhere to
        // put it. The bus takes packets from a front end unchanged.
        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        let mut voice = out.with_kind(PortKind::Voice);
        // The vocoder's rate, not the graph's: a listener resamples from it.
        voice.rate = VOICE_HZ;
        Ok(vec![out, voice])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
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

        self.voice_now.clear();
        let mut events = Vec::new();
        for f in &frames {
            // The vocoder runs here rather than in the assembler: what a
            // stream frame carries is 40 ms of speech, and holding it until
            // the transmission ends would mean nobody could listen to it.
            if let Body::Stream { payload, .. } = &f.body {
                let p = *payload;
                self.decode_voice(&p);
            }
            for e in self.assembler.push(f) {
                if let Event::LinkSetup { lsf, .. } = &e {
                    // A new transmission: what the vocoder holds belongs to
                    // the last one, and whether to run it at all is decided
                    // by what the link setup says this stream is.
                    self.voice.clear();
                    self.voice_stream = lsf.is_stream()
                        && matches!(lsf.data_type(), DataType::Voice | DataType::VoiceData);
                    self.talking = self
                        .voice_stream
                        .then(|| (lsf.source().to_string(), lsf.destination().to_string()));
                }
                events.push(e);
            }
        }
        self.frames = frames;
        // A transmission that stopped mid-stream ends when nothing more is
        // heard, so the assembler has to be told the time even when no frame
        // arrived.
        events.extend(self.assembler.poll(self.samples));

        // The channel is reported whether or not anybody is on it, so a
        // listener can see the front end is there before it has heard
        // anything, and so a meter has a row to sit on.
        let (from, to) = match self.talking() {
            Some((f, t)) => (Some(f.to_string()), Some(t.to_string())),
            None => (None, None),
        };
        outputs[OUT_VOICE].voice_mut().push(common::Voice {
            system: "M17",
            channel_hz: self.channel_hz,
            to,
            from,
            rate: VOICE_HZ,
            pcm: std::mem::take(&mut self.voice_now),
        });

        let center_hz = self.channel_hz as u64;
        let at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        let out = outputs[OUT_PACKETS].packets_mut();
        for e in &events {
            self.accepted += 1;
            // The speech goes with the row that ends the transmission, which
            // is the one a log shows with a duration on it.
            let audio = matches!(e, Event::Stream { .. })
                .then(|| self.take_voice())
                .flatten();
            if matches!(e, Event::Stream { .. }) {
                // The transmission ended, so there is nobody to subscribe to
                // until the next link setup.
                self.talking = None;
                self.voice_stream = false;
            }
            out.push(common::Packet {
                at_us,
                center_hz,
                bandwidth_hz: CHANNEL_WIDTH_HZ as u32,
                // A frame that reached here passed its checks; the front end
                // measures no level per transmission.
                rssi_dbfs: f32::NAN,
                snr_db: f32::NAN,
                modulation: Some("4FSK"),
                body: common::PacketBody::Frame(e.to_bytes()),
                iq: None,
                audio,
                measure: None,
            });
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.talking = None;
        self.voice.clear();
        self.voice_now.clear();
        self.voice_stream = false;
        self.codec = Codec2::new(Codec2Mode::MODE_3200);
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.demod.reset();
        self.assembler = Assembler::new(AUDIO_HZ);
    }
}

/// Decode a run of stream payloads into speech.
///
/// The payloads are what the log holds, sixteen bytes a frame, so this is how
/// a transmission is heard again from a recording rather than from the
/// receiver that decoded it live.
pub fn decode_stream_voice(payloads: &[u8]) -> Vec<f32> {
    let mut c2 = Codec2::new(Codec2Mode::MODE_3200);
    let mut pcm = [0i16; 160];
    let mut out = Vec::with_capacity(payloads.len() / C2_FRAME_BYTES * 160);
    for half in payloads.chunks_exact(C2_FRAME_BYTES) {
        c2.decode(&mut pcm, half);
        out.extend(pcm.iter().map(|s| *s as f32 / 32768.0));
    }
    out
}

/// Payload bytes as hex, so a row carries what was on the air in a form that
/// can be pasted into another decoder.
fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// The row a transmission becomes.
pub fn m17_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    use common::Value;
    let event = Event::parse(bytes)?;
    let lsf = match &event {
        Event::LinkSetup { lsf, .. } => Some(lsf),
        Event::Packet { lsf, .. }
        | Event::Stream { lsf, .. }
        | Event::StreamFrame { lsf, .. } => lsf.as_ref(),
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
        // One frame of a stream, which is what the log holds and what the
        // audio is rebuilt from. The row is deliberately thin: a list showing
        // twenty-five of these a second is a list nobody reads, and the
        // interface folds them into the transmission they belong to.
        Event::StreamFrame { number, payload, .. } => {
            fields.push(("frame".into(), Value::Int(i64::from(*number))));
            // 40 ms, the one duration in M17 that needs no clock, so anything
            // counting airtime can add these up without waiting for the
            // transmission to end.
            fields.push(("seconds".into(), Value::Float(0.04)));
            fields.push(("payload".into(), Value::Text(hex(payload))));
            match lsf.map(|l| l.data_type()) {
                Some(DataType::Voice) | Some(DataType::VoiceData) => "M17-Voice",
                _ => "M17-Stream",
            }
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
        assert!(n.negotiate(&[spec(2_400_000.0, DEFAULT_HZ)]).is_ok());
        assert!(n.negotiate(&[spec(2_400_000.0, 145_000_000.0)]).is_err());
        // A stream that holds the channel is enough, even at the source
        // extractor's 25 kHz floor; one narrower than the channel is not.
        assert!(n.negotiate(&[spec(25_000.0, DEFAULT_HZ)]).is_ok());
        assert!(n.negotiate(&[spec(10_000.0, DEFAULT_HZ)]).is_err());
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

    /// The node's output, as the bytes of each event it emitted.
    ///
    /// It produces packets rather than bare frames now, because a voice
    /// transmission carries its speech as well as its bytes.
    fn run(iq: &[common::C32], rate: f64, center: f64) -> Vec<Vec<u8>> {
        let mut node = M17Node::default();
        node.negotiate(&[spec(rate, center)]).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); (rate * 0.5) as usize];
        for block in [&quiet[..], iq, &quiet[..]] {
            for chunk in block.chunks(65_536) {
                let input = Payload::Iq(chunk.to_vec());
                let mut out = Payload::Packets(Vec::new());
                let (mut events, mut new_tags) = (Vec::new(), Vec::new());
                let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
                node.process(&[&input], std::slice::from_mut(&mut out), &mut ctx).unwrap();
                if let Payload::Packets(ps) = out {
                    frames.extend(ps.into_iter().filter_map(|p| match p.body {
                        common::PacketBody::Frame(b) => Some(b),
                        _ => None,
                    }));
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
        let all: Vec<Decoded> =
            frames.iter().filter_map(|f| m17_decoded(f, Hz(center as u64))).collect();
        // Every frame is on the bus as evidence; these are the two rows that
        // describe the transmission as a whole.
        let rows: Vec<Decoded> = all
            .iter()
            .filter(|d| !d.fields.iter().any(|(k, _)| k == "frame"))
            .cloned()
            .collect();
        assert_eq!(rows.len(), 2, "expected a setup row and a stream row: {rows:?}");
        assert_eq!(all.len(), 27, "25 frames, a setup and a summary: {}", all.len());

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

    /// A frame produced by an independent implementation, decoded by ours.
    ///
    /// Every other test here checks this code against itself: our encoder
    /// makes the symbols our decoder reads, and a puncture pattern or a bit
    /// order that is wrong in both is invisible. These symbols came from
    /// m17core 0.1.0, a separate M17 implementation, encoding a stream frame
    /// number 7 whose payload is the bytes below. Agreement means the frame
    /// layer matches somebody else's reading of the specification, which is
    /// what makes the vocoder's output trustworthy: a payload one bit out of
    /// place decodes to speech-shaped noise rather than to nothing.
    #[test]
    fn a_frame_from_another_implementation_decodes_to_its_payload() {
        const REFERENCE: [f32; 192] = [
    -3.0, -3.0, -3.0, -3.0, 3.0, 3.0, -3.0, 3.0, 1.0, 3.0, 3.0, -3.0, -1.0, -3.0, -1.0, 1.0,
    1.0, 3.0, 3.0, 3.0, 1.0, 3.0, 3.0, -3.0, 1.0, -3.0, 1.0, -3.0, 3.0, 3.0, -3.0, 3.0,
    1.0, -1.0, -1.0, 1.0, -3.0, 3.0, -1.0, 3.0, 3.0, 1.0, -3.0, -3.0, 1.0, -3.0, -1.0, 3.0,
    3.0, -3.0, 1.0, -3.0, -3.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0, -1.0, -3.0, 3.0, -3.0, -3.0,
    -1.0, 3.0, 3.0, -3.0, 1.0, 3.0, 1.0, -1.0, -3.0, 1.0, 3.0, -3.0, -1.0, 3.0, 3.0, 1.0,
    1.0, -3.0, -1.0, 3.0, -3.0, -3.0, -3.0, -3.0, 1.0, -1.0, -3.0, 1.0, 1.0, -1.0, 1.0, 1.0,
    3.0, -1.0, 1.0, -1.0, -3.0, 3.0, 3.0, 1.0, -1.0, 3.0, -3.0, 3.0, -3.0, -3.0, -3.0, 3.0,
    -3.0, -1.0, 3.0, -1.0, 1.0, 1.0, 1.0, 3.0, 3.0, 1.0, 3.0, -3.0, -3.0, -1.0, 1.0, 1.0,
    -3.0, 1.0, 3.0, -3.0, -3.0, 1.0, 1.0, 3.0, 3.0, -1.0, -3.0, -1.0, -1.0, 3.0, 1.0, -1.0,
    -3.0, -1.0, -1.0, 3.0, 3.0, 1.0, -3.0, -1.0, -1.0, -1.0, -3.0, 3.0, -3.0, -3.0, 3.0, -3.0,
    3.0, 1.0, -1.0, -3.0, -1.0, 1.0, 3.0, -3.0, -1.0, -3.0, -1.0, 3.0, -1.0, -1.0, -3.0, 1.0,
    -1.0, 1.0, -1.0, -1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -3.0, -3.0, 3.0, 1.0, 1.0, -1.0, 1.0,
        ];
        const PAYLOAD: [u8; 16] = [
            0x03, 0x14, 0x25, 0x36, 0x47, 0x58, 0x69, 0x7a, 0x8b, 0x9c, 0xad, 0xbe, 0xcf, 0xe0,
            0xf1, 0x02,
        ];

        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let mut symbols = preamble_symbols();
        // Repeated, because one frame in isolation is not a transmission: the
        // demodulator needs a run of them to lock to.
        for _ in 0..8 {
            symbols.extend_from_slice(&REFERENCE);
        }
        let iq = modulate(&symbols, rate, 0.0);
        let frames = run(&iq, rate, center);
        let payloads: Vec<[u8; 16]> = frames
            .iter()
            .filter_map(|f| match m17::Event::parse(f) {
                Some(m17::Event::StreamFrame { number: 7, payload, .. }) => Some(payload),
                _ => None,
            })
            .collect();
        assert!(!payloads.is_empty(), "nothing decoded from the reference frames");
        for p in &payloads {
            assert_eq!(*p, PAYLOAD, "payload differs from the reference");
        }
    }

    /// The speech of a voice stream comes out of the vocoder and travels with
    /// the row that ends the transmission.
    ///
    /// The payload here is not real speech, so what it decodes to is noise.
    /// What the test is about is the plumbing: that a voice stream is
    /// vocoded at all, that 25 frames make a second of audio at the codec's
    /// rate, and that it arrives attached to the packet rather than lost.
    #[test]
    fn a_voice_stream_is_decoded_to_speech() {
        let (rate, center) = (2_400_000.0, DEFAULT_HZ);
        let setup = link_setup("M17-M17 C", "M0ABC", 1 | 2 << 1 | 5 << 7);
        let mut symbols = preamble_symbols();
        symbols.extend(frame_symbols(Kind::Lsf, &setup, 0, &[0; 6]));
        for n in 0..25u16 {
            let cnt = (n % 6) as usize;
            let mut lich = [0u8; 6];
            lich[..5].copy_from_slice(&setup[cnt * 5..cnt * 5 + 5]);
            lich[5] = (cnt as u8) << 5;
            symbols.extend(frame_symbols(Kind::Stream, &[0x55u8; 16], n, &lich));
        }
        let iq = modulate(&symbols, rate, 1_000.0);

        let mut node = M17Node::default();
        node.negotiate(&[spec(rate, center)]).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let quiet = vec![common::C32::new(0.0, 0.0); (rate * 0.5) as usize];
        let mut speech: Option<std::sync::Arc<common::Speech>> = None;
        let mut live = 0usize;
        for block in [&quiet[..], &iq[..], &quiet[..]] {
            for chunk in block.chunks(65_536) {
                let input = Payload::Iq(chunk.to_vec());
                let mut out = Payload::Packets(Vec::new());
                let (mut events, mut new_tags) = (Vec::new(), Vec::new());
                let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
                node.process(&[&input], std::slice::from_mut(&mut out), &mut ctx).unwrap();
                live += node.voice_now().len();
                if let Payload::Packets(ps) = out {
                    for p in ps {
                        if p.audio.is_some() {
                            speech = p.audio;
                        }
                    }
                }
            }
        }
        let speech = speech.expect("the stream row carried no speech");
        assert_eq!(speech.rate, VOICE_HZ);
        // 25 frames of 40 ms, allowing for the last one closing the stream.
        assert!(
            (speech.seconds() - 1.0).abs() < 0.1,
            "{} seconds of speech from a one second transmission",
            speech.seconds()
        );
        assert_eq!(live, speech.pcm.len(), "what was published live is what was kept");
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
