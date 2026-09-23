//! NXDN as a graph node.
//!
//! The same front end as P25 and DMR, because the waveform is the same one:
//! the channel is narrowband FM, so the node mixes it down, filters it,
//! discriminates it and hands the result to `dsp::c4fm::SymbolClock`. NXDN is
//! FDMA, so what comes out is one continuous stream of symbols with a 20 bit
//! sync word every 192 of them, and the framer hunts that sync.
//!
//! Two channel widths, one node: 12.5 kHz is 4800 symbols a second and
//! 6.25 kHz is 2400, and nothing else differs. Which one a transmitter used
//! is not in the bits, so it is the clock that says, and the width the
//! scanner table placed the decoder at is what sets the clock.
//!
//! What a frame says is read by `decode::nxdn`: the link information channel,
//! then the slow channel's radio access number, then the voice call message
//! off a stolen half frame or off four slow channels in a row.
//!
//! Speech is AMBE+2 at 3600 bit/s, four 72-bit channels to a frame and two to
//! a half that was not stolen for signalling, which is the same vocoder rate
//! DMR keys. The vocoder (`crates/mbe`) is behind the `ambe` feature, off by
//! default, because AMBE is patent-encumbered: without it a voice frame still
//! reaches the bus naming the call and the channel time it took, and the call
//! list gets its airtime with no audio behind it.

use crate::NodeSpec;
use crate::ambe::Vocoder;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::nxdn::CODEC;

/// Rate the vocoder produces, and the rate the voice port is declared at.
pub const VOICE_HZ: f64 = 8_000.0;

const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;
pub use decode::nxdn::FLAG_EMERGENCY;
pub use decode::nxdn::FLAG_ENCRYPTED;
pub use decode::nxdn::FLAG_GROUP;
pub use decode::nxdn::FLAG_HAVE_CALL;
pub use decode::nxdn::FLAG_HAVE_MSG;
pub use decode::nxdn::FLAG_HAVE_RAN;
pub use decode::nxdn::FLAG_NARROW;
pub use decode::nxdn::FLAG_OUTBOUND;
pub use decode::nxdn::HEAD_LEN;
pub use decode::nxdn::NARROW_BAUD;
pub use decode::nxdn::NXDN_TAG;
pub use decode::nxdn::WIDE_BAUD;
pub use decode::nxdn::encode_frame;
pub use decode::nxdn::frame_kind;
pub use decode::nxdn::frame_seconds;
pub use decode::nxdn::read;
use decode::nxdn::{self, NxdnFrame};
pub use decode::nxdn::{WINDOW, cipher_code};
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::nxdn::DEFAULT_HZ;
pub use identify::nxdn::NARROW_HZ;
pub use identify::nxdn::Nxdn;
pub use identify::nxdn::WIDE_HZ;
pub use identify::nxdn::width_for;
pub use identify::nxdn::{AUDIO_HZ, RRC_ALPHA, deviation_hz};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Channel samples kept behind the symbol clock, in seconds, so a frame's own
/// samples are still there when its packet is built.
///
/// A frame is 40 ms, and the framer reads one up to three frames after its
/// symbols arrived, but the ring also has to outlast one input block: a
/// second of samples can arrive in a single call from a file, and four
/// frames of history is then gone before anything in it is asked for.
const KEEP_S: f64 = 1.0;

/// One-sided filter cutoff, wide enough for the outer symbols and the
/// transmitter's drift.
fn cutoff_hz(baud: f64) -> f64 {
    if baud >= WIDE_BAUD { 7_000.0 } else { 3_500.0 }
}

pub struct NxdnNode {
    channel_hz: f64,
    baud: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    rrc: FirDecimReal,
    clock: SymbolClock,
    framer: nxdn::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    shaped: Vec<f32>,
    syms: Vec<f32>,
    meter: crate::FrameMeter,
    vocoder: Vocoder,
    audio_rate: f64,
    accepted: u64,
    /// The call the last message named, so the voice frames after it
    /// belong to it.
    talking: Option<nxdn::Call>,
}

impl Default for NxdnNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, WIDE_BAUD)
    }
}

impl NxdnNode {
    pub fn new(channel_hz: f64, baud: f64) -> Self {
        let baud = if baud >= WIDE_BAUD { WIDE_BAUD } else { NARROW_BAUD };
        Self {
            channel_hz,
            baud,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, cutoff_hz(baud), 60.0),
            fm: FmDemod::new(AUDIO_HZ, deviation_hz(baud)),
            rrc: FirDecimReal::new(rrc_taps(AUDIO_HZ / baud, RRC_ALPHA, 8), 1),
            clock: SymbolClock::new(AUDIO_HZ, baud),
            framer: nxdn::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            shaped: Vec::new(),
            syms: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, KEEP_S)
                .keyed_as(common::Modulation::Fsk4),
            vocoder: Vocoder::new(),
            audio_rate: AUDIO_HZ,
            accepted: 0,
            talking: None,
        }
    }

    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    fn width_hz(&self) -> f64 {
        width_for(self.baud)
    }

    /// One frame as a packet: what it said, the level it was heard at and the
    /// samples it was sliced from, found by the frame's symbol index.
    /// The call this frame is part of, for the voice port.
    ///
    /// What the frame carried of speech, not how long it held the channel: a
    /// frame with a half stolen for signalling is half a frame of talking.
    /// The parties come from the last message that named them, since the
    /// frames between two call messages belong to the call they opened, and
    /// a call joined part way through is named by the superframe the slow
    /// channel completes.
    ///
    /// The speech is the frame's own voice channels through the vocoder,
    /// except where the network said they are enciphered, which is speech
    /// nothing here can read.
    fn call(&mut self, frame: &NxdnFrame) -> Option<common::Voice> {
        let named = frame
            .frame
            .facch1
            .iter()
            .find_map(|m| m.call)
            .or_else(|| frame.message.as_ref().and_then(|m| m.call));
        if let Some(c) = named {
            self.talking = Some(c);
        }
        if frame.frame.voice_slots == 0 {
            return None;
        }
        let c = self.talking?;
        let seconds =
            nxdn::frame_seconds(self.baud < WIDE_BAUD) * frame.frame.voice_slots as f64 / 4.0;
        let secrecy = match c.cipher {
            nxdn::Cipher::Clear => common::Secrecy::Clear,
            other => common::Secrecy::Encrypted(Some(other.label().to_string())),
        };
        let pcm = match c.cipher {
            nxdn::Cipher::Clear => self.vocoder.decode_channels(&frame.frame.voice),
            _ => Vec::new(),
        };
        Some(common::Voice {
            system: "NXDN",
            channel_hz: self.channel_hz,
            to: Some(c.dest.to_string()),
            from: Some(c.source.to_string()),
            code: None,
            over: Some(common::Over::new(Some(CODEC)).protected_by(secrecy).lasting(seconds)),
            rate: VOICE_HZ,
            channels: 1,
            pcm,
        })
    }

    fn packet(&mut self, frame: &NxdnFrame) -> common::packet::Packet {
        self.accepted += 1;
        let sps = self.audio_rate / self.baud;
        let bytes = encode_frame(frame, self.baud < WIDE_BAUD);
        let start = (frame.at as f64 * sps) as u64;
        let len = (nxdn::FRAME_DIBITS as f64 * sps) as usize;
        let snr_db = self.meter.snr_db_at(start, len);
        self.meter.packet_measured(bytes, start, len, snr_db)
    }
}

impl Node for NxdnNode {
    fn name(&self) -> &str {
        "nxdn"
    }

    fn num_outputs(&self) -> usize {
        2
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("nxdn reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        let width = self.width_hz();
        if (self.channel_hz - center).abs() > rate / 2.0 - width / 2.0 {
            return Err(common::Error::other("nxdn needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, cutoff_hz(self.baud), 60.0);
        self.fm = FmDemod::new(audio_rate, deviation_hz(self.baud));
        self.rrc = FirDecimReal::new(rrc_taps(audio_rate / self.baud, RRC_ALPHA, 8), 1);
        self.clock = SymbolClock::new(audio_rate, self.baud);
        self.framer = nxdn::Framer::new();
        self.audio_rate = audio_rate;
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, KEEP_S)
            .keyed_as(common::Modulation::Fsk4);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = width;
        out.rate = 0.0;
        // The call itself, on the port a call is stated on: the speech the
        // vocoder read, and the over carrying what the network said about
        // the transmission.
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
            // A frame whose LICH passed and whose payload did not says only
            // that something is on the channel, and on noise that is all a
            // false sync word ever says. See `nxdn::noise_is_not_a_frame`.
            if !f.frame.read_anything() {
                continue;
            }
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
        self.vocoder.reset();
        self.talking = None;
    }
}

impl Protocol for Nxdn {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
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

    /// A business radio channel on the land mobile allocations: NXDN is on
    /// VHF and UHF, and what makes one an NXDN channel is what is keyed on it.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        read(bytes).map(|d| vec![d])
    }

    /// Which of the two widths a source is, decided at the geometric mean of
    /// them: a 6.25 kHz channel measures wider than it is and a 12.5 kHz one
    /// narrower, and the halfway point on a log scale splits them evenly.
    fn widths_for(&self, _hz: f64, source_width_hz: f64) -> Vec<f64> {
        match source_width_hz < (WIDE_HZ * NARROW_HZ).sqrt() {
            true => vec![NARROW_HZ],
            false => vec![WIDE_HZ],
        }
    }

    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Packets, PortKind::Voice]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        let baud = match at.width_hz < (WIDE_HZ * NARROW_HZ).sqrt() {
            true => NARROW_BAUD,
            false => WIDE_BAUD,
        };
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz).f(BAUD, baud)]
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";
/// Symbols a second, which is the channel width in the only form the decoder
/// cares about.
const BAUD: &str = "baud";

pub const DESC: StageDesc = StageDesc {
    name: "nxdn",
    summary: "One NXDN channel: 4-level FSK at 4800 or 2400 baud, RAN and call ids",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(NxdnNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), s.f64_or(BAUD, WIDE_BAUD))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::nxdn::{Call, CallType, Cipher, MessageType, Sacch, Steal};
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    fn group_call() -> Call {
        Call {
            call_type: CallType::Group,
            source: 1234,
            dest: 5678,
            cipher: Cipher::Clear,
            key_id: 0,
            emergency: false,
            duplex: false,
        }
    }

    /// Key a run of dibits as 4-level FSK at `rate`, at the given offset from
    /// the tuner's centre: the deviation NXDN keys, through an integrator.
    fn keyed(dibits: &[u8], rate: f64, baud: f64, offset_hz: f64, noise: f32) -> Vec<common::C32> {
        let sps = rate / baud;
        let dev = deviation_hz(baud);
        let mut out = Vec::with_capacity((dibits.len() as f64 * sps) as usize);
        let mut phase = 0.0f64;
        let mut seed = 0x1234_5678_9abc_def0u64;
        let mut noise_sample = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 40) as f32 / 8_388_608.0 - 0.5) * noise
        };
        for d in dibits {
            let hz = match d & 3 {
                1 => dev,
                0 => dev / 3.0,
                2 => -dev / 3.0,
                _ => -dev,
            };
            for _ in 0..sps.round() as usize {
                phase += std::f64::consts::TAU * (hz + offset_hz) / rate;
                let (s, c) = phase.sin_cos();
                out.push(common::C32::new(c as f32 + noise_sample(), s as f32 + noise_sample()));
            }
        }
        out
    }

    /// Run IQ through the node and return what it said and what it heard.
    fn replay(
        iq: &[common::C32],
        rate: f64,
        center: f64,
        hz: f64,
        baud: f64,
    ) -> (Vec<common::packet::Proto>, Vec<common::Voice>) {
        let mut node = NxdnNode::new(hz, baud);
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
            if let Payload::Packets(ps) = packets {
                rows.extend(ps.iter().filter_map(|p| read(p.bytes())));
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

    /// Symbols with all four levels and no sync word in them, keyed either
    /// side of a call so the clock is running as a receiver's would be.
    /// Random rather than a repeating pattern, which a timing loop locks to
    /// the wrong phase of.
    fn tail() -> Vec<u8> {
        let mut seed = 0x51ed_2701_dead_1234u64;
        (0..1_500)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                (seed >> 33) as u8 & 3
            })
            .collect()
    }

    /// One voice channel of AMBE bits, told apart by its place in the call.
    /// All zero is a well formed frame the Golay check passes, which is what
    /// the vocoder has to be given to say anything at all.
    fn speech(n: usize) -> [bool; decode::nxdn::VOICE_BITS] {
        let mut bits = [false; decode::nxdn::VOICE_BITS];
        bits[decode::nxdn::VOICE_BITS - 1] = n % 2 == 1;
        bits
    }

    /// A call as a radio sends it: a voice frame with the call message in a
    /// stolen half, then frames of speech with the message going out over the
    /// slow channel a quarter at a time.
    fn call_frames(ran: u8, call: &Call, frames: usize) -> Vec<u8> {
        let message = decode::nxdn::call_bits(MessageType::VCall, call);
        let mut padded = message.clone();
        padded.resize(72, false);
        let mut dibits = tail();
        for n in 0..frames {
            let quarter = n % 4;
            let mut data = [false; 18];
            data.copy_from_slice(&padded[quarter * 18..quarter * 18 + 18]);
            let sacch = Sacch { ran, structure: 3 - quarter as u8, data };
            let steal = if n == 0 { Steal::First } else { Steal::None };
            let mut payload = sacch.air();
            for (half, stolen) in steal.stolen().iter().enumerate() {
                match stolen {
                    true => payload.extend(decode::nxdn::facch1_air(&message)),
                    false => payload.extend(decode::nxdn::voice_air(&[
                        speech(n * 4 + half * 2),
                        speech(n * 4 + half * 2 + 1),
                    ])),
                }
            }
            dibits.extend(decode::nxdn::keyed(decode::nxdn::rdch_lich(steal, true), &payload));
        }
        dibits.extend(tail());
        dibits
    }

    #[test]
    fn negotiates_a_channel_inside_the_span() {
        let mut n = NxdnNode::default();
        assert!(n.negotiate(&[spec(2_048_000.0, DEFAULT_HZ)]).is_ok());
        assert!(n.negotiate(&[spec(2_048_000.0, 460_000_000.0)]).is_err());
    }

    #[test]
    fn labels_only_its_own_frames() {
        let frame = NxdnFrame {
            at: 0,
            frame: nxdn::frame(
                &decode::nxdn::keyed(
                    decode::nxdn::rdch_lich(Steal::None, true),
                    &Sacch { ran: 12, structure: 3, data: [false; 18] }.air(),
                )[10..],
            )
            .expect("a frame"),
            message: decode::nxdn::message(&decode::nxdn::call_bits(
                MessageType::VCall,
                &group_call(),
            )),
        };
        let bytes = encode_frame(&frame, false);
        let d = read(&bytes).expect("an NXDN row");
        assert_eq!((d.id, d.kind), ("nxdn", "vcall"));
        assert_eq!(site(&d), Some(12), "the radio access number names the site");
        assert_eq!(
            (
                d.parties().0.unwrap_or_default().to_string(),
                d.parties().1.unwrap_or_default().to_string()
            ),
            ("1234".to_string(), "5678".to_string())
        );
        // A talkgroup is many listeners under one name, which the decoder
        // says rather than leaving it to be read off the digits.
        assert_eq!(d.link.to.map(|p| p.kind), Some(common::packet::PartyKind::Group));
        // Nobody wrote this, so it is not a message, and how long it held
        // the channel is the over rather than a field on the frame.
        assert!(d.facts.iter().all(|f| !matches!(f, common::packet::Fact::Message(_))));
        assert!(read(b"random").is_none());
        assert!(read(b"NX").is_none());
    }

    /// A keyed call off the node's own front end: every frame of it read, the
    /// talkgroup and the radio off the stolen half, and the same call again
    /// once the slow channel has sent all four quarters.
    #[test]
    fn reads_a_keyed_call_through_the_front_end() {
        let rate = 96_000.0;
        let iq = keyed(&call_frames(9, &group_call(), 8), rate, WIDE_BAUD, 0.0, 0.0);
        let (rows, voices) = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.len(), 8, "eight frames keyed, {} read", rows.len());
        assert!(rows.iter().all(|d| site(d) == Some(9)), "every frame names the site");
        let named: Vec<_> =
            rows.iter().filter(|d| d.parties().0.unwrap_or_default() == "1234").collect();
        assert_eq!(named.len(), 3, "the stolen half and two completed superframes");
        assert!(named.iter().all(|d| d.parties().1.unwrap_or_default() == "5678"));
        // Every frame carries speech, and the first carries half as much
        // because it stole a half for the call message. That is the over,
        // stated once on the voice port.
        assert_eq!(voices.len(), 8);
        let seconds: Vec<f64> =
            voices.iter().filter_map(|v| v.over.as_ref()).map(|o| o.seconds).collect();
        assert_eq!(seconds[0], 0.02, "two of four voice channels in the opening frame");
        assert!(seconds[1..].iter().all(|s| *s == 0.04), "40 ms of speech a frame after it");
        // Nothing is encrypted, so nothing claims a key.
        assert!(voices.iter().all(|v| v.over.as_ref().is_some_and(|o| !o.encrypted())));
    }

    /// The speech the frames carried, through the vocoder: thirty voice
    /// channels over the eight frames, two in the opening frame and four in
    /// each one after it, and 20 ms of 8 kHz audio out of every one.
    #[test]
    fn a_keyed_call_comes_out_as_speech() {
        let rate = 96_000.0;
        let iq = keyed(&call_frames(9, &group_call(), 8), rate, WIDE_BAUD, 0.0, 0.0);
        let (_, voices) = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, WIDE_BAUD);
        let samples: usize = voices.iter().map(|v| v.pcm.len()).sum();
        if !cfg!(feature = "ambe") {
            assert_eq!(samples, 0, "no vocoder is built in, so no speech is claimed");
            return;
        }
        assert_eq!(voices[0].pcm.len(), 2 * 160, "two voice channels in the opening frame");
        assert!(voices[1..].iter().all(|v| v.pcm.len() == 4 * 160), "four channels a frame after");
        assert_eq!(samples, 30 * 160, "thirty voice channels, {samples} samples");
        assert_eq!(samples as f64 / VOICE_HZ, 0.6, "0.6 s of speech off 0.32 s of channel");
        assert!(
            voices.iter().flat_map(|v| &v.pcm).all(|s| s.is_finite() && s.abs() <= 1.0),
            "a sample the vocoder could not have produced"
        );
        assert!(voices.iter().flat_map(|v| &v.pcm).any(|s| *s != 0.0), "every sample was silence");
        assert!(voices.iter().all(|v| v.rate == VOICE_HZ && v.channels == 1));
    }

    /// Off frequency and in noise, which is what a receiver actually hands
    /// the node. Measured: 2.5 kHz off with noise at a tenth of the carrier
    /// still reads every frame.
    #[test]
    fn reads_a_call_off_frequency_and_in_noise() {
        let rate = 96_000.0;
        let iq = keyed(&call_frames(31, &group_call(), 8), rate, WIDE_BAUD, -2_500.0, 0.1);
        let (rows, _) = replay(&iq, rate, DEFAULT_HZ + 2_500.0, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.len(), 8);
        assert_eq!(rows.iter().filter(|d| site(d) == Some(31)).count(), 8);
    }

    /// The narrow channel is the same frame at half the clock.
    #[test]
    fn reads_the_six_and_a_quarter_kilohertz_channel() {
        let rate = 96_000.0;
        let iq = keyed(&call_frames(1, &group_call(), 6), rate, NARROW_BAUD, 0.0, 0.0);
        let (rows, voices) = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, NARROW_BAUD);
        assert_eq!(rows.len(), 6);
        // The opening frame stole a half, so it is half of the 80 ms a
        // 6.25 kHz frame holds the channel for.
        let seconds: Vec<f64> =
            voices.iter().filter_map(|v| v.over.as_ref()).map(|o| o.seconds).collect();
        assert_eq!(seconds[0], 0.04);
        assert_eq!(seconds[1], 0.08, "80 ms a frame");
    }

    /// An enciphered call says so, and says under which key, without
    /// pretending to any speech it cannot read.
    #[test]
    fn an_enciphered_call_names_its_key() {
        let call = Call {
            cipher: Cipher::Aes,
            key_id: 21,
            emergency: true,
            call_type: CallType::Individual,
            source: 40001,
            dest: 40002,
            duplex: false,
        };
        let rate = 96_000.0;
        let iq = keyed(&call_frames(3, &call, 4), rate, WIDE_BAUD, 0.0, 0.0);
        let (rows, voices) = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.len(), 4);
        let named: Vec<_> =
            rows.iter().filter(|d| d.parties().0.unwrap_or_default() == "40001").collect();
        assert_eq!(named.len(), 2, "the stolen half and one completed superframe");
        // A private call names one radio at each end, not a talkgroup.
        assert!(
            named.iter().all(
                |d| d.link.to.as_ref().map(|p| p.kind) == Some(common::packet::PartyKind::Unit)
            )
        );
        // A radio declaring an emergency is the one thing here somebody has
        // to be told about.
        assert!(
            named
                .iter()
                .all(|d| d.facts.iter().any(|f| matches!(f, common::packet::Fact::Alert(_))))
        );
        // What protects the speech is the network's own word, said beside
        // the audio it is about, and nothing pretends to have read it.
        assert_eq!(
            voices.first().and_then(|v| v.over.as_ref()).map(|o| o.secrecy.clone()),
            Some(common::Secrecy::Encrypted(Some("AES".into())))
        );
        assert_eq!(voices.iter().map(|v| v.pcm.len()).sum::<usize>(), 0, "enciphered speech");
    }

    /// Thirty seconds of noise, and nothing said about any of it.
    #[test]
    fn noise_is_not_a_call() {
        let rate = 96_000.0;
        let mut seed = 0xdead_beef_cafe_f00du64;
        let iq: Vec<common::C32> = (0..(rate * 30.0) as usize)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let i = (seed >> 40) as f32 / 8_388_608.0 - 0.5;
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let q = (seed >> 40) as f32 / 8_388_608.0 - 0.5;
                common::C32::new(i, q)
            })
            .collect();
        let rows = replay(&iq, rate, DEFAULT_HZ, DEFAULT_HZ, WIDE_BAUD);
        assert_eq!(rows.0.len(), 0, "thirty seconds of noise read as {} frames", rows.0.len());
    }
}
