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
pub use decode::p25::decoded;
pub use decode::p25::encode_frame;
use decode::p25::{self, P25Frame};
use decode::p25::{Duid, Encryption, LinkControl};
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
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
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
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, KEEP_S),
            audio_rate: AUDIO_HZ,
            accepted: 0,
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
    fn packet(&mut self, frame: &P25Frame) -> common::Packet {
        let at_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        self.accepted += 1;
        let sps = self.audio_rate / BAUD;
        let bytes = encode_frame(frame);
        let start = (frame.at as f64 * sps) as u64;
        let len = (p25::LDU_DIBITS as f64 * sps) as usize;
        let snr_db = self.meter.snr_db_at(start, len);
        let measured = self.meter.frame_measured(bytes, start, len, snr_db);
        common::Packet::of_frame(at_us, CHANNEL_WIDTH_HZ as u32, measured)
    }
}

impl Simple for P25Node {
    fn name(&self) -> &str {
        "p25"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
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
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, KEEP_S);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
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
            let p = self.packet(f);
            o.packets_mut().push(p);
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
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        decoded(bytes, common::Hz(p.center_hz())).map(|d| vec![d])
    }

    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Packets]
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
    fn replay(iq: &[common::C32], rate: f64, center: f64, hz: f64) -> Vec<Decoded> {
        let mut node = P25Node::new(hz);
        node.negotiate(&spec(rate, center)).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut rows = Vec::new();
        for chunk in iq.chunks(16_384) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(ps) = &out {
                for p in ps {
                    if let common::PacketBody::Frame(f) = &p.body {
                        assert!(p.rssi_dbfs().is_finite() && p.snr_db().is_finite());
                        assert!(p.samples().is_some_and(|q| !q.samples.is_empty()));
                        rows.extend(decoded(&f.bytes, common::Hz(p.center_hz())));
                    }
                }
            }
        }
        rows
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
        assert!(n.negotiate(&spec(2_048_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_048_000.0, 460_000_000.0)).is_err());
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
        let d = decoded(&bytes, Hz(155_752_500)).expect("a P25 row");
        assert_eq!(d.protocol, "P25-Voice");
        let get = |k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert_eq!(get("nac"), "293");
        assert_eq!(get("to"), "1234");
        assert_eq!(get("from"), "5679413");
        assert_eq!(get("call_type"), "group");
        let air = d.airtime.as_ref().expect("a voice frame with no airtime");
        assert_eq!(air.seconds, VOICE_SECONDS);
        assert!(air.voice && air.live);
        assert_eq!(air.codec, Some(CODEC));
        assert_eq!(air.secrecy, common::Secrecy::Clear);
        // Nobody wrote this, so it is not a message.
        assert!(!d.written);
        assert!(decoded(b"random", Hz(0)).is_none());
        assert!(decoded(b"P1", Hz(0)).is_none());
    }

    /// A keyed call off the node's own front end: every frame of it read, the
    /// talkgroup and radio on every voice frame that carried the link
    /// control, and the key named on the other half.
    #[test]
    fn reads_a_keyed_call_through_the_front_end() {
        let rate = 96_000.0;
        let iq = keyed(&call(0x293), rate, 0.0, 0.0);
        let rows = replay(&iq, rate, 155_752_500.0, 155_752_500.0);
        assert_eq!(rows.len(), 10, "ten frames keyed, {} read", rows.len());
        assert!(rows.iter().all(|d| d.protocol == "P25-Voice"));
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert!(rows.iter().all(|d| get(d, "nac") == "293"));
        let named = rows.iter().filter(|d| get(d, "to") == "1234").count();
        assert_eq!(named, 5, "five frames carry the link control");
        assert!(
            rows.iter().filter(|d| get(d, "to") == "1234").all(|d| get(d, "from") == "5679413")
        );
        // Nothing is encrypted, so no row claims a key.
        assert_eq!(rows.iter().filter(|d| get(d, "encrypted") == "true").count(), 0);
        // Every row is 180 ms of speech in the call list.
        assert_eq!(rows.iter().filter(|d| d.airtime.is_some()).count(), 10);
    }

    /// Off frequency and in noise, which is what a receiver actually hands
    /// the node. Measured: 2.5 kHz off with noise at a tenth of the carrier
    /// still reads every frame.
    #[test]
    fn reads_a_call_off_frequency_and_in_noise() {
        let rate = 96_000.0;
        let iq = keyed(&call(0x4d2), rate, -2_500.0, 0.1);
        let rows = replay(&iq, rate, 155_755_000.0, 155_752_500.0);
        assert_eq!(rows.len(), 10);
        let nacs = rows
            .iter()
            .filter(|d| d.fields.iter().any(|(k, v)| k == "nac" && v.to_string() == "4D2"))
            .count();
        assert_eq!(nacs, 10);
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
        let rows = replay(&keyed(&dibits, rate, 0.0, 0.0), rate, DEFAULT_HZ, DEFAULT_HZ);
        assert_eq!(rows.len(), 2);
        let get = |d: &Decoded, k: &str| {
            d.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string()).unwrap_or_default()
        };
        assert_eq!(get(&rows[0], "emergency"), "true");
        assert_eq!(get(&rows[0], "encrypted"), "true");
        assert_eq!(get(&rows[1], "algorithm"), "ADP");
        assert_eq!(get(&rows[1], "key_id"), "42");
        assert_eq!(rows[0].airtime.as_ref().unwrap().secrecy, common::Secrecy::Encrypted(None));
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
        assert_eq!(rows.len(), 0, "two minutes of noise read as {} frames", rows.len());
    }
}
