//! P25 phase 1 as a graph node.
//!
//! The same front end as DMR, because the waveform is the same: the channel
//! is narrowband FM, so the node mixes it down, filters it, discriminates it
//! and hands the result to `dsp::c4fm::SymbolClock`, which keeps a symbol
//! clock across blocks. P25 is FDMA rather than TDMA, so what comes out is
//! one continuous stream of 4800 baud dibits with a 48-bit sync word every
//! frame, and the framer hunts that sync rather than following a burst clock.
//!
//! What a frame says is read by `decode::p25`: the network access code and
//! the data unit id out of the BCH-protected network identifier, and out of a
//! voice frame the link control, which names the talkgroup and the radio, or
//! the encryption sync, which names the key the speech is under.
//!
//! No speech: the IMBE vocoder is not built, so a voice frame reaches the bus
//! as a packet naming the call and carrying its bits, and the call list gets
//! its 180 ms of airtime without any audio behind it.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::p25::CODEC;
pub use decode::p25::FLAG_EMERGENCY;
pub use decode::p25::FLAG_ENCRYPTED;
pub use decode::p25::FLAG_GROUP;
pub use decode::p25::FLAG_HAVE_ES;
pub use decode::p25::FLAG_HAVE_LC;
pub use decode::p25::HEAD_LEN;
pub use decode::p25::P25_TAG;
pub use decode::p25::VOICE_SECONDS;

/// Rate the voice port is declared at. Nothing here decodes IMBE, so no
/// samples travel on it; the rate is what the vocoder would produce.
pub const VOICE_HZ: f64 = 8_000.0;

const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;
pub use decode::p25::encode_frame;
pub use decode::p25::read;
use decode::p25::{self, P25Frame};
pub use decode::p25::{SYNC_TOLERANCE, WINDOW};
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::p25::BAUD;
pub use identify::p25::CHANNEL_WIDTH_HZ;
pub use identify::p25::DEFAULT_HZ;
pub use identify::p25::P25;
pub use identify::p25::{AUDIO_HZ, DEVIATION_HZ, RRC_ALPHA};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// One-sided filter cutoff, wide enough for the outer symbols and the
/// transmitter's drift.
const FILTER_CUTOFF_HZ: f64 = 7_000.0;

/// Channel samples kept behind the symbol clock, in seconds: the framer
/// reads a frame once the one after it has arrived and keeps a frame of
/// history behind the hunt, so a frame's own samples are up to three frames
/// old by the time its packet is built.
const KEEP_S: f64 = (4 * p25::LDU_DIBITS) as f64 / BAUD;

pub struct P25Node {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    rrc: FirDecimReal,
    clock: SymbolClock,
    framer: p25::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    shaped: Vec<f32>,
    syms: Vec<f32>,
    meter: crate::FrameMeter,
    audio_rate: f64,
    accepted: u64,
    /// Who the last link control named, so the voice frames between them
    /// belong to the call it opened.
    talking: Option<(String, String)>,
    /// What the last encryption sync said protects the speech.
    secrecy: common::Secrecy,
}

impl Default for P25Node {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl P25Node {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, FILTER_CUTOFF_HZ, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            rrc: FirDecimReal::new(rrc_taps(AUDIO_HZ / BAUD, RRC_ALPHA, 8), 1),
            clock: SymbolClock::new(AUDIO_HZ, BAUD),
            framer: p25::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            shaped: Vec::new(),
            syms: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, KEEP_S)
                .keyed_as(common::Modulation::Fsk4),
            audio_rate: AUDIO_HZ,
            accepted: 0,
            talking: None,
            secrecy: common::Secrecy::Unsaid,
        }
    }

    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// One frame as a packet: what it said, the level it was heard at and the
    /// samples it was sliced from, found by the frame's symbol index.
    /// The call this frame is part of, for the voice port.
    ///
    /// A frame that carries speech is 180 ms of the channel whoever is
    /// listening; the link control names the parties, and a frame without one
    /// belongs to the call the last one opened.
    fn call(&mut self, frame: &P25Frame) -> Option<common::Voice> {
        if !frame.duid.voice() {
            return None;
        }
        if let Some(lc) = frame.lc.as_ref().filter(|lc| !lc.encrypted())
            && let (Some(src), Some(tg)) = (lc.source(), lc.talkgroup())
        {
            self.talking = Some((src.to_string(), tg.to_string()));
        }
        // The encryption sync says what protects the speech, and it is sent
        // on the other kind of voice frame, so what it said stands until the
        // next one says otherwise.
        if let Some(es) = frame.es.as_ref() {
            self.secrecy = match es.algid == p25::Encryption::CLEAR {
                true => common::Secrecy::Clear,
                false => common::Secrecy::Encrypted(None),
            };
        }
        let secrecy = self.secrecy.clone();
        // A protected link control cannot be read, and the call is still on
        // the channel: the network access code is what the frame did say, so
        // it stands for the group until something names one.
        let (from, to) = match self.talking.clone() {
            Some(pair) => pair,
            None => (String::new(), format!("NAC {:03X}", frame.nac)),
        };
        Some(common::Voice {
            system: "P25",
            channel_hz: self.channel_hz,
            to: Some(to),
            from: Some(from).filter(|f| !f.is_empty()),
            code: None,
            over: Some(common::Over::new(Some(CODEC)).protected_by(secrecy).lasting(VOICE_SECONDS)),
            rate: VOICE_HZ,
            channels: 1,
            pcm: Vec::new(),
        })
    }

    fn packet(&mut self, frame: &P25Frame) -> common::packet::Packet {
        self.accepted += 1;
        let sps = self.audio_rate / BAUD;
        let bytes = encode_frame(frame);
        let start = (frame.at as f64 * sps) as u64;
        let len = (p25::LDU_DIBITS as f64 * sps) as usize;
        let snr_db = self.meter.snr_db_at(start, len);
        self.meter.packet_measured(bytes, start, len, snr_db)
    }
}

impl Node for P25Node {
    fn name(&self) -> &str {
        "p25"
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("p25 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("p25 needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, FILTER_CUTOFF_HZ, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.rrc = FirDecimReal::new(rrc_taps(audio_rate / BAUD, RRC_ALPHA, 8), 1);
        self.clock = SymbolClock::new(audio_rate, BAUD);
        self.framer = p25::Framer::new();
        self.audio_rate = audio_rate;
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, KEEP_S)
            .keyed_as(common::Modulation::Fsk4);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        // The call itself, on the port a call is stated on. Nothing here
        // decodes IMBE, so the speech is empty and the over carries what the
        // network said: who is talking, in which vocoder, under what cipher,
        // and how much of the channel this frame was.
        let mut voice = out.with_kind(PortKind::Voice);
        voice.rate = VOICE_HZ;
        voice.channels = 1;
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
        self.meter.feed(&self.narrow);
        self.audio.clear();
        self.fm.process(&self.narrow, &mut self.audio);

        let raw = std::mem::take(&mut self.audio);
        let mut shaped = std::mem::take(&mut self.shaped);
        shaped.clear();
        self.rrc.process(&raw, &mut shaped);
        self.audio = raw;

        let mut syms = std::mem::take(&mut self.syms);
        syms.clear();
        self.clock.push(&shaped, &mut syms);
        self.shaped = shaped;

        let mut frames = Vec::new();
        self.framer.push(&syms, &mut frames);
        self.syms = syms;

        for f in &frames {
            if let Some(v) = self.call(f) {
                outputs[OUT_VOICE].voice_mut().push(v);
            }
            let p = self.packet(f);
            outputs[OUT_PACKETS].packets_mut().push(p);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.rrc.reset();
        self.clock.reset();
        self.framer.reset();
        self.meter.reset();
    }
}

impl Protocol for P25 {
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

    /// A 12.5 kHz channel anywhere: P25 is on VHF, UHF, 700 and 800 MHz, and
    /// what makes one a P25 channel is what is keyed on it.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        read(bytes).map(|d| vec![d])
    }

    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Packets, PortKind::Voice]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "p25",
    summary: "One P25 phase 1 channel: C4FM at 4800 baud, talkgroup and radio id",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(P25Node::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::p25::{Duid, Encryption, LinkControl};
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// A group voice call: talkgroup 1234, radio 5679413.
    fn group_lc() -> [u8; 9] {
        [0x00, 0x00, 0x00, 0x00, 0x04, 0xd2, 0x56, 0xa9, 0x35]
    }

    /// The encryption sync of a call in the clear.
    fn clear_es() -> [u8; 12] {
        let mut es = [0u8; 12];
        es[9] = Encryption::CLEAR;
        es
    }

    /// Key a run of dibits as C4FM at `rate`, at the given offset from the
    /// tuner's centre: the deviation P25 keys, through an integrator.
    fn keyed(dibits: &[u8], rate: f64, offset_hz: f64, noise: f32) -> Vec<common::C32> {
        let sps = rate / BAUD;
        let mut out = Vec::with_capacity((dibits.len() as f64 * sps) as usize);
        let mut phase = 0.0f64;
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut noise_sample = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 40) as f32 / 8_388_608.0 - 0.5) * noise
        };
        for d in dibits {
            // 01 is +1800 Hz, 00 is +600, 10 is -600, 11 is -1800.
            let hz = match d & 3 {
                1 => 1_800.0,
                0 => 600.0,
                2 => -600.0,
                _ => -1_800.0,
            };
            for _ in 0..sps.round() as usize {
                phase += std::f64::consts::TAU * (hz + offset_hz) / rate;
                let (s, c) = phase.sin_cos();
                out.push(common::C32::new(c as f32 + noise_sample(), s as f32 + noise_sample()));
            }
        }
        out
    }

    /// Run IQ through the node and return the rows it wrote.
    fn replay(
        iq: &[common::C32],
        rate: f64,
        center: f64,
        hz: f64,
    ) -> (Vec<common::packet::Proto>, Vec<common::Voice>) {
        let mut node = P25Node::new(hz);
        node.negotiate(&[spec(rate, center)]).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let (mut rows, mut voices) = (Vec::new(), Vec::new());
        for chunk in iq.chunks(16_384) {
            let input = Payload::Iq(chunk.to_vec());
            let mut outs = [Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], &mut outs, &mut ctx).unwrap();
            let [packets, voice] = outs;
            if let Payload::Voice(v) = voice {
                voices.extend(v);
            }
            if let Payload::Packets(ps) = &packets {
                for p in ps {
                    if p.bytes().is_empty() {
                        continue;
                    }
                    assert!(p.carrier.rssi_dbfs.is_finite() && p.carrier.snr_db.is_finite());
                    assert!(p.carrier.iq.as_ref().is_some_and(|q| !q.samples.is_empty()));
                    rows.extend(read(p.bytes()));
                }
            }
        }
        (rows, voices)
    }

    /// What a layer states about the site it was heard from.
    fn site(d: &common::packet::Proto) -> Option<u16> {
        d.facts.iter().find_map(|f| match f {
            common::packet::Fact::Infrastructure(c) => c.site_code,
            _ => None,
        })
    }

    /// Dibits with all four levels and no sync word in them, keyed either
    /// side of a call: ahead of it so the symbol clock is running as a
    /// receiver's would be when a call starts, and after it so the last
    /// frame is complete in the framer's window. Random rather than a
    /// repeating pattern, which a timing loop locks to the wrong phase of.
    fn tail() -> Vec<u8> {
        let mut seed = 0x51ed_2701_dead_1234u64;
        (0..1_000)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (seed >> 33) as u8 & 3
            })
            .collect()
    }

    /// Ten voice frames, alternating the two kinds, as a call is sent.
    fn call(nac: u16) -> Vec<u8> {
        let mut dibits = tail();
        for i in 0..10 {
            let (duid, hex) = if i % 2 == 0 {
                (Duid::Voice1, p25::hex_words(&group_lc()))
            } else {
                (Duid::Voice2, p25::hex_words(&clear_es()))
            };
            dibits.extend(p25::frame_dibits(nac, duid, &hex));
        }
        dibits.extend(tail());
        dibits
    }

    #[test]
    fn negotiates_a_channel_inside_the_span() {
        let mut n = P25Node::default();
        assert!(n.negotiate(&[spec(2_048_000.0, DEFAULT_HZ)]).is_ok());
        assert!(n.negotiate(&[spec(2_048_000.0, 460_000_000.0)]).is_err());
    }

    #[test]
    fn labels_only_its_own_frames() {
        let bytes = encode_frame(&P25Frame {
            at: 0,
            nac: 0x293,
            duid: Duid::Voice1,
            lc: Some(LinkControl { bytes: group_lc(), repaired: 0 }),
            es: None,
        });
        let d = read(&bytes).expect("a P25 row");
        assert_eq!((d.id, d.kind), ("p25", "voice-1"));
        assert_eq!(site(&d), Some(0x293), "the network access code names the site");
        assert_eq!(
            (
                d.parties().0.unwrap_or_default().to_string(),
                d.parties().1.unwrap_or_default().to_string()
            ),
            ("5679413".to_string(), "1234".to_string())
        );
        assert_eq!(d.link.to.map(|p| p.kind), Some(common::packet::PartyKind::Group));
        // Nobody wrote this, so it is not a message, and how long it held
        // the channel is the over rather than a field on the frame.
        assert!(d.facts.iter().all(|f| !matches!(f, common::packet::Fact::Message(_))));
        assert!(read(b"random").is_none());
        assert!(read(b"P1").is_none());
    }

    /// A keyed call off the node's own front end: every frame of it read, the
    /// talkgroup and radio on every voice frame that carried the link
    /// control, and the key named on the other half.
    #[test]
    fn reads_a_keyed_call_through_the_front_end() {
        let rate = 96_000.0;
        let iq = keyed(&call(0x293), rate, 0.0, 0.0);
        let (rows, voices) = replay(&iq, rate, 155_752_500.0, 155_752_500.0);
        assert_eq!(rows.len(), 10, "ten frames keyed, {} read", rows.len());
        assert!(rows.iter().all(|d| d.kind.starts_with("voice")));
        assert!(rows.iter().all(|d| site(d) == Some(0x293)));
        let named = rows.iter().filter(|d| d.parties().1.unwrap_or_default() == "1234").count();
        assert_eq!(named, 5, "five frames carry the link control");
        assert!(
            rows.iter().filter(|d| d.parties().1.unwrap_or_default() == "1234").all(|d| d
                .parties()
                .0
                .unwrap_or_default()
                == "5679413")
        );
        // Every frame is 180 ms of the channel in the call list, and none of
        // it is enciphered.
        assert_eq!(voices.len(), 10);
        assert!(voices.iter().all(|v| v.over.as_ref().is_some_and(|o| {
            o.seconds == VOICE_SECONDS && o.codec == Some(CODEC) && !o.encrypted()
        })));
    }

    /// Off frequency and in noise, which is what a receiver actually hands
    /// the node. Measured: 2.5 kHz off with noise at a tenth of the carrier
    /// still reads every frame.
    #[test]
    fn reads_a_call_off_frequency_and_in_noise() {
        let rate = 96_000.0;
        let iq = keyed(&call(0x4d2), rate, -2_500.0, 0.1);
        let (rows, _) = replay(&iq, rate, 155_755_000.0, 155_752_500.0);
        assert_eq!(rows.len(), 10);
        assert_eq!(rows.iter().filter(|d| site(d) == Some(0x4d2)).count(), 10);
    }

    /// An enciphered call says so, and says under which key, without
    /// pretending to any speech.
    #[test]
    fn an_enciphered_call_names_its_key() {
        let mut lc = group_lc();
        // Service options: enciphered, and an emergency.
        lc[2] = 0xc0;
        let mut es = [0u8; 12];
        es[..9].copy_from_slice(&[9, 8, 7, 6, 5, 4, 3, 2, 1]);
        es[9] = 0xaa;
        es[10] = 0x00;
        es[11] = 0x2a;
        let mut dibits = tail();
        dibits.extend(p25::frame_dibits(0x293, Duid::Voice1, &p25::hex_words(&lc)));
        dibits.extend(p25::frame_dibits(0x293, Duid::Voice2, &p25::hex_words(&es)));
        dibits.extend(tail());
        let rate = 96_000.0;
        let (rows, voices) = replay(&keyed(&dibits, rate, 0.0, 0.0), rate, DEFAULT_HZ, DEFAULT_HZ);
        assert_eq!(rows.len(), 2);
        // A radio declaring an emergency is told to somebody rather than
        // spelled in a field.
        assert!(
            rows[0].facts.iter().any(|f| matches!(f, common::packet::Fact::Alert(_))),
            "{:?}",
            rows[0].facts
        );
        // The encryption sync rides on the other kind of voice frame, so the
        // call is judged from the frame after the one that opened it.
        assert_eq!(
            voices.last().and_then(|v| v.over.as_ref()).map(|o| o.secrecy.clone()),
            Some(common::Secrecy::Encrypted(None))
        );
    }

    /// Minutes of noise, and nothing said about any of it.
    #[test]
    fn noise_is_not_a_call() {
        let rate = 96_000.0;
        let mut seed = 0xdead_beef_cafe_f00du64;
        let iq: Vec<common::C32> = (0..(rate * 120.0) as usize)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let i = (seed >> 40) as f32 / 8_388_608.0 - 0.5;
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let q = (seed >> 40) as f32 / 8_388_608.0 - 0.5;
                common::C32::new(i, q)
            })
            .collect();
        let rows = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ);
        assert_eq!(rows.0.len(), 0, "two minutes of noise read as {} frames", rows.0.len());
    }
}
