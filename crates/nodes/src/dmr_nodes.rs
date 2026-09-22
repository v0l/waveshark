//! DMR as a graph node.
//!
//! The same shape as the M17 front end: the channel is narrowband FM, so the
//! node mixes it down, filters it and discriminates it. What comes out is
//! four-level FSK at 4800 baud, two-slot TDMA. This node recovers the symbol
//! clock with a Gardner loop that runs continuously across blocks, correlates
//! the 48-bit sync words (ETSI TS 102 361-1 clause 9.1.1) to find the burst
//! boundaries, and reads the three 72-bit AMBE frames a voice burst carries.
//!
//! The vocoder (`crates/mbe`) is behind the `ambe` feature, off by default,
//! because AMBE is patent-encumbered. Without it the node still finds the
//! transmission and reports the channel; with it, the AMBE frames become
//! 8 kHz speech on the voice bus.
//!
//! Only burst A of a voice superframe carries a sync word; the other five
//! carry an EMB field instead. So the framer locks a burst clock rather than
//! hunting for a sync each time: once a sync is found, the next burst is one
//! TDMA frame later, and each is confirmed by its own sync or by its EMB
//! passing the QR(16,7,6) check. Hunting per superframe threw a whole 360 ms
//! away whenever one sync was marginal, which split a single over into three
//! rows in the packet log.
//!
//! Who is talking comes from the link control (`decode::dmr`): the voice LC
//! header opens a transmission, the terminator closes it, and the embedded LC
//! spread across bursts B to E repeats it every superframe, so a receiver
//! that came in late still has the talkgroup and the radio ID within 360 ms.
//!
//! What is not here yet: slot 2 is not separated from slot 1, so the node
//! follows whichever slot it locks onto first.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
use decode::dmr::{self, DmrEvent, Framer, LinkControl};
use dsp::c4fm::SymbolClock;
use dsp::fir::FirDecimReal;
use dsp::m17::rrc_taps;
use dsp::{FirDecim, FmDemod, Mixer};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

use crate::ambe::Vocoder;
pub use decode::dmr::BODY_LEN;
pub use decode::dmr::BURST_BYTES;
pub use decode::dmr::CODEC;
pub use decode::dmr::DMR_TAG;
pub use decode::dmr::FLAG_EMERGENCY;
pub use decode::dmr::FLAG_ENCRYPTED;
pub use decode::dmr::FLAG_GROUP;
pub use decode::dmr::FLAG_HAVE_LC;
pub use decode::dmr::OVER_LEN;
pub use decode::dmr::OVER_TAG;
pub use decode::dmr::POS_DATA;
pub use decode::dmr::PRIVACY;
pub use decode::dmr::REANCHOR;
pub use decode::dmr::SLOT_STRIDE;
pub use decode::dmr::SYM_BURST;
pub use decode::dmr::SYM_PAYLOAD;
pub use decode::dmr::SYM_SYNC;
pub use decode::dmr::read;
pub use decode::dmr::unpack_bits;
pub use decode::dmr::{MAX_MISSES, lc_flags, pack_bits};
use identify::Signal;
pub use identify::dmr::CHANNEL_WIDTH_HZ;
pub use identify::dmr::DEFAULT_HZ;
pub use identify::dmr::Dmr;
pub use identify::dmr::{AUDIO_HZ, BAUD, DEVIATION_HZ, RRC_ALPHA};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Motorola's feature set, whose privacy bit means Basic Privacy when no
/// key id was signalled.
const FID_MOTOROLA: u8 = 0x10;

/// The keystream to undo on a transmission's speech, from the DMR keys
/// held: one named for its talkgroup, else one named `*` for every group.
/// A one byte key is a Basic Privacy key number. Longer keys are the
/// enhanced and AES ciphers, which this does not undo.
fn privacy_keystream(lc: &LinkControl) -> Option<[bool; 49]> {
    if !lc.encrypted() || lc.fid != FID_MOTOROLA {
        return None;
    }
    let held = decode::channel_keys::for_system(decode::channel_keys::System::Dmr);
    let group = lc.dst.to_string();
    held.iter()
        .find(|k| k.name == group)
        .or_else(|| held.iter().find(|k| k.name == "*"))
        .filter(|k| k.key.len() == 1)
        .and_then(|k| decode::dmr_bp::keystream(k.key[0]))
}

/// The AMBE frames of a voice burst's bits, as [`Framer::voice_frames`]
/// reads them, for anything that wants to hear a logged burst again.
pub fn burst_voice_frames(bytes: &[u8]) -> Option<[[u8; 9]; 3]> {
    if bytes.len() != BODY_LEN || bytes[..2] != DMR_TAG || bytes[2] == POS_DATA {
        return None;
    }
    Some(Framer::voice_frames(&unpack_bits(&bytes[13..])))
}

/// Silence after the last voice burst before the link control it was under
/// is forgotten, so a transmission that dropped without a terminator does
/// not lend its talkgroup to the next one. Longer than a superframe (360 ms)
/// and the framer's eight missed bursts (480 ms).
const OVER_SILENCE_S: f64 = 1.5;

/// One-sided filter cutoff. A compliant DMR signal is ~9.5 kHz wide (±4.75),
/// but handsets over-deviate badly (a DM-1701 measured ±6.3 kHz outer
/// levels, ~14 kHz occupied), so pass well past nominal or the outer symbols
/// are clipped and the four-level eye closes.
const FILTER_CUTOFF_HZ: f64 = 10_000.0;

/// Vocoder output rate.
pub const VOICE_HZ: f64 = 8_000.0;

/// Channel samples kept behind the symbol clock, in seconds: everything the
/// framer may still read, which is a burst plus its re-anchoring.
const KEEP_S: f64 = (SYM_BURST + SLOT_STRIDE + MAX_MISSES as usize * SLOT_STRIDE) as f64 / BAUD;

pub struct DmrNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    rrc: FirDecimReal,
    sync: SymbolClock,
    framer: dmr::Framer,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    /// Discriminator output through the matched filter.
    shaped: Vec<f32>,
    syms: Vec<f32>,
    /// Speech decoded this block, for listening live.
    voice_now: Vec<f32>,
    /// Whether a voice transmission is in progress.
    talking: bool,
    /// Who is talking, once a header, terminator or embedded LC has said.
    lc: Option<LinkControl>,
    /// Bursts since the last voice sync, to notice a transmission ending.
    idle_bursts: u32,
    /// Input samples since the last voice burst, so a transmission that
    /// simply stops lets go of its link control after a while.
    silent_samples: u64,
    /// Input sample rate, for the silence timeout.
    in_rate: f64,
    /// The channel behind the symbol clock: what each burst was heard at and
    /// the samples it was sliced from.
    meter: crate::FrameMeter,
    audio_rate: f64,
    accepted: u64,
    /// The AMBE speech path. A zero-size stub without the `ambe` feature, so
    /// the node reads signalling only there and decodes no speech.
    vocoder: Vocoder,
}

impl Default for DmrNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

const OUT_PACKETS: usize = 0;
const OUT_VOICE: usize = 1;

impl DmrNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, FILTER_CUTOFF_HZ, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            rrc: FirDecimReal::new(rrc_taps(AUDIO_HZ / BAUD, RRC_ALPHA, 8), 1),
            sync: SymbolClock::new(AUDIO_HZ, BAUD),
            framer: dmr::Framer::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            shaped: Vec::new(),
            syms: Vec::new(),
            voice_now: Vec::new(),
            talking: false,
            lc: None,
            idle_bursts: 0,
            silent_samples: 0,
            in_rate: AUDIO_HZ,
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, KEEP_S)
                .keyed_as(common::Modulation::Fsk4),
            audio_rate: AUDIO_HZ,
            accepted: 0,
            vocoder: Vocoder::new(),
        }
    }

    pub fn channel_hz(&self) -> f64 {
        self.channel_hz
    }

    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    pub fn voice_now(&self) -> &[f32] {
        &self.voice_now
    }

    /// Decode one voice burst's three AMBE frames to speech, for the live
    /// bus and for the burst's own packet. Without the `ambe` feature the
    /// vocoder yields no samples, so this returns `None`.
    fn decode_voice(&mut self, frames: &[[u8; 9]; 3]) -> Option<std::sync::Arc<common::Speech>> {
        let keystream = self.lc.as_ref().and_then(privacy_keystream);
        let pcm = self.vocoder.decode_burst(frames, keystream.as_ref());
        if pcm.is_empty() {
            return None;
        }
        self.voice_now.extend_from_slice(&pcm);
        Some(std::sync::Arc::new(common::Speech { pcm, rate: VOICE_HZ }))
    }

    /// One burst as a packet: its bits, the framer's context, its speech,
    /// the level it was heard at and the samples it was sliced from.
    ///
    /// The samples are found by the burst's symbol index. The filters ahead
    /// of the symbol clock delay the symbols by a few dozen samples, which is
    /// inside the burst's own guard.
    /// One burst as a reception. The speech it carried is not on it: an over
    /// is stated once, on the voice port this node also publishes.
    fn packet(&mut self, at: usize, pos: u8, bits: &[u8]) -> common::packet::Packet {
        self.accepted += 1;
        let sps = self.audio_rate / BAUD;
        let bytes = dmr::encode_burst(pos, self.framer.colour, self.lc.as_ref(), bits);
        let (start, len) = ((at as f64 * sps) as u64, (SYM_BURST as f64 * sps) as usize);
        let snr_db = self.meter.snr_db_at(start, len);
        self.meter.packet_measured(bytes, start, len, snr_db)
    }
}

impl Protocol for Dmr {
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

    /// Like M17 it runs wherever it is put, so it is recognised by its own
    /// tagged body rather than by band.
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

impl Node for DmrNode {
    fn name(&self) -> &str {
        "dmr"
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
            return Err(common::Error::other("dmr reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("dmr needs its channel inside the span"));
        }
        let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        let audio_rate = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, FILTER_CUTOFF_HZ, 60.0);
        self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
        self.rrc = FirDecimReal::new(rrc_taps(audio_rate / BAUD, RRC_ALPHA, 8), 1);
        self.sync = SymbolClock::new(audio_rate, BAUD);
        self.framer = dmr::Framer::new();
        self.in_rate = rate;
        self.audio_rate = audio_rate;
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, KEEP_S)
            .keyed_as(common::Modulation::Fsk4);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        out.rate = 0.0;
        let mut voice = out.with_kind(PortKind::Voice);
        voice.rate = VOICE_HZ;
        Ok(vec![out, voice])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else {
            return Ok(());
        };
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

        self.syms.clear();
        let mut syms = std::mem::take(&mut self.syms);
        self.sync.push(&shaped, &mut syms);
        self.shaped = shaped;

        let mut events = Vec::new();
        self.framer.push(&syms, &mut events);
        self.syms = syms;

        self.voice_now.clear();
        let had_voice = events.iter().any(|e| matches!(e, DmrEvent::Voice { .. }));
        let mut packets = Vec::new();
        for e in events {
            match e {
                DmrEvent::Voice { at, bits, frames, pos } => {
                    if !self.talking {
                        self.lc = None;
                        self.talking = true;
                    }
                    self.idle_bursts = 0;
                    let _ = self.decode_voice(&frames);
                    packets.push(self.packet(at, pos, &bits));
                }
                DmrEvent::Lc(lc) => self.lc = Some(lc),
                DmrEvent::Data { at, bits, data_type } => {
                    // The link control a header or terminator carries is in
                    // the event before this one, so the packet has it.
                    packets.push(self.packet(at, POS_DATA, &bits));
                    match data_type {
                        // The terminator is the transmission saying it is
                        // over, which is the only end that is not a guess.
                        Some(dmr::DT_TERMINATOR_LC) => {
                            self.talking = false;
                            self.lc = None;
                        }
                        Some(dmr::DT_VOICE_LC_HEADER) => {
                            self.talking = true;
                            self.idle_bursts = 0;
                            self.silent_samples = 0;
                        }
                        _ => {
                            // Signalling with no voice between it means the
                            // channel has moved on without a terminator.
                            if self.talking {
                                self.idle_bursts += 1;
                                if self.idle_bursts >= 8 {
                                    self.talking = false;
                                    self.lc = None;
                                }
                            }
                        }
                    }
                }
            }
        }

        // A carrier that drops after the last voice burst leaves no
        // terminator: the link control it was under is let go after enough
        // input time with no voice, so the next transmission does not
        // inherit it. The over itself ends downstream, where a run of bursts
        // with nothing after it is the end of one.
        if self.talking {
            if had_voice {
                self.silent_samples = 0;
            } else {
                self.silent_samples += iq.len() as u64;
                if self.silent_samples as f64 >= self.in_rate * OVER_SILENCE_S {
                    self.talking = false;
                    self.lc = None;
                    self.silent_samples = 0;
                }
            }
        }

        let (from, to) = match self.lc {
            Some(lc) => (Some(lc.src.to_string()), Some(lc.dst.to_string())),
            None => (None, None),
        };
        outputs[OUT_VOICE].voice_mut().push(common::Voice {
            system: "DMR",
            channel_hz: self.channel_hz,
            to,
            from,
            // Coded squelch is an analogue thing; a decoder names the group
            // itself.
            code: None,
            over: self.lc.map(|lc| {
                common::Over::new(Some(CODEC)).protected_by(match lc.encrypted() {
                    true => common::Secrecy::Encrypted(Some(dmr::PRIVACY.into())),
                    false => common::Secrecy::Clear,
                })
            }),
            rate: VOICE_HZ,
            channels: 1,
            pcm: std::mem::take(&mut self.voice_now),
        });
        outputs[OUT_PACKETS].packets_mut().extend(packets);
        Ok(())
    }

    fn reset(&mut self) {
        self.sync.reset();
        self.framer.reset();
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.rrc.reset();
        self.voice_now.clear();
        self.talking = false;
        self.lc = None;
        self.idle_bursts = 0;
        self.meter.reset();
        self.silent_samples = 0;
        self.vocoder.reset();
    }
}

/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "dmr",
    summary: "One DMR channel: narrowband FM, 4-FSK at 4800 baud, two-slot TDMA voice",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(DmrNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use pipeline::node::NodeCtx;
    use pipeline::port::StreamSpec;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    #[test]
    fn negotiates_a_channel_inside_the_span() {
        let mut n = DmrNode::default();
        assert!(n.negotiate(&[spec(2_048_000.0, DEFAULT_HZ)]).is_ok());
        assert!(n.negotiate(&[spec(2_048_000.0, 460_000_000.0)]).is_err());
    }

    #[test]
    fn labels_only_its_own_bursts() {
        let bits = vec![0u8; SYM_BURST * 2];
        let body = dmr::encode_burst(2, Some(1), None, &bits);
        let d = read(&body).expect("a DMR row");
        assert_eq!((d.id, d.kind), ("dmr", "voice"));
        // The colour code tells two cells sharing a channel apart.
        assert!(d.facts.iter().any(|f| matches!(
            f,
            common::packet::Fact::Infrastructure(c) if c.site_code == Some(1)
        )));
        // Without a link control there is nobody to put in the call list.
        assert_eq!(d.parties(), (None, None));
        // The bits come back out as they went in.
        assert_eq!(unpack_bits(&body[13..]), bits);

        // With one, the row names the talkgroup and the radio, and says it
        // is voice, which is what the call table needs to keep it.
        let lc = LinkControl { flco: dmr::FLCO_GROUP, fid: 0, options: 0, dst: 91, src: 2_345_678 };
        let d = read(&dmr::encode_burst(0, None, Some(&lc), &bits)).expect("a row");
        assert_eq!(d.parties(), (Some("2345678"), Some("91")));
        assert_eq!(d.link.to.as_ref().map(|p| p.kind), Some(common::packet::PartyKind::Group));
        assert_eq!(d.subject.as_ref().map(|e| e.id.to_string()).as_deref(), Some("2345678"));
        // Not anyone else's frame.
        assert!(read(b"random").is_none());
        assert!(read(b"DB").is_none());
    }

    /// Run a capture through the node's own path and report what came out:
    /// speech samples, packet rows and the link control on each.
    fn replay(path: &str, rate: f64, center: f64, hz: f64) -> (usize, Vec<common::packet::Packet>) {
        let raw = std::fs::read(path).unwrap();
        let iq: Vec<common::C32> = raw
            .chunks_exact(2)
            .map(|c| common::C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
            .collect();
        let mut node = DmrNode::new(hz);
        node.negotiate(&[spec(rate, center)]).unwrap();
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let (mut live, mut packets) = (0usize, Vec::new());
        for chunk in iq.chunks(65_536) {
            let input = Payload::Iq(chunk.to_vec());
            let mut out = [Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&[&input], &mut out, &mut ctx).unwrap();
            if let [Payload::Packets(ps), Payload::Voice(vs)] = &out {
                live += vs.iter().map(|v| v.pcm.len()).sum::<usize>();
                packets.extend(ps.iter().cloned());
            }
        }
        (live, packets)
    }

    /// The corpus capture against what the transmission says about itself:
    /// one over, talkgroup 9, radio 1234567, three and a half seconds of it.
    /// `testdata/fixtures.toml` says what the capture is evidence of and how
    /// those values were established. Skips cleanly without the file.
    #[test]
    fn reads_one_over_and_its_link_control_off_air() {
        const NAME: &str = "dmr_tg9_433.45M_2048k.cu8";
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../testdata/dmr_tg9_433.45M_2048k.cu8");
        if !std::path::Path::new(path).exists() {
            eprintln!("skipping: {NAME} absent, run testdata/fetch.sh");
            return;
        }
        let (live, packets) = replay(path, 2_048_000.0, 433_450_000.0, 433_900_000.0);
        // One packet per burst off the air: the headers, the voice, the
        // terminator, each carrying the link control in force. The over is
        // the run of them, added up downstream.
        let rows: Vec<common::packet::Proto> =
            packets.iter().filter_map(|p| read(p.bytes())).collect();
        assert_eq!(rows.len(), packets.len(), "every packet labels as DMR");
        let voice: Vec<_> = rows.iter().filter(|d| d.kind == "voice").collect();
        // One burst is 60 ms of the channel, and the over is the run of them.
        let seconds = voice.len() as f64 * 0.06;
        assert!(
            seconds > 3.0,
            "the over ran {seconds:.2} s of voice bursts, expected the whole 3.6"
        );
        // Every voice burst after the header names the call, so the call list
        // has it from the first burst and not from the terminator.
        let named = voice.iter().filter(|d| d.parties() == (Some("1234567"), Some("9"))).count();
        assert!(
            named * 10 > voice.len() * 9,
            "{named} of {} voice bursts carried the link control",
            voice.len()
        );
        assert!(rows.iter().any(|d| d.kind == "voice_header"), "no header row");
        assert!(rows.iter().any(|d| d.kind == "terminator"), "no terminator row");
        // Each burst carries what it was heard at, the samples it was read
        // from, and bits that read back as AMBE frames.
        assert!(
            packets.iter().all(|p| p.carrier.iq.as_ref().is_some_and(|q| !q.samples.is_empty())),
            "a burst without its samples"
        );
        assert!(
            packets.iter().all(|p| p.carrier.rssi_dbfs.is_finite() && p.carrier.snr_db.is_finite()),
            "a burst without its level"
        );
        let frames = packets.iter().filter_map(|p| burst_voice_frames(p.bytes())).count();
        assert_eq!(frames, voice.len());

        // The vocoder is what turns the AMBE frames into samples, so speech
        // is only asserted where it is built in. Ten superframes of it.
        if cfg!(feature = "ambe") {
            let secs = live as f64 / VOICE_HZ;
            assert!(secs > 3.0, "decoded {secs:.2} s of speech, expected the whole over");
        }
    }
}
