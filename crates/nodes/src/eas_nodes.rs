//! EAS and SAME as a graph node: a weather or voice channel in, the alert
//! header out.
//!
//! Two layers of modulation, the same shape as APRS and MDC. The channel is
//! ordinary FM, so the node mixes it down, filters it and discriminates it;
//! the header is then in the *audio* as AFSK at 520.83 baud, which
//! [`dsp::afsk`] reads and [`decode::eas`] frames and votes.
//!
//! It also takes audio directly. An alert is relayed on any NFM channel a
//! station cares to put it on, and where one is already tuned the samples
//! have been discriminated once already: wiring this stage onto that audio
//! reads the alert without a second front end cutting the same channel out
//! of the span again.
//!
//! What reaches the bus is the header text, voted across the three copies
//! the standard sends. Nothing in SAME carries a check sequence, so the form
//! of the header is the whole of the check and a burst that is not a well
//! formed header never becomes a row.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::eas::read;
use decode::eas::{self};
use dsp::afsk::{AfskBits, AfskConfig, SAME};
use dsp::{FirDecim, FmDemod, Mixer};
use identify::Signal;
pub use identify::eas::CHANNEL_WIDTH_HZ;
pub use identify::eas::DEFAULT_HZ;
pub use identify::eas::Eas;
pub use identify::eas::WEATHER_CHANNELS_HZ;
pub use identify::eas::{AUDIO_HZ, DEVIATION_HZ};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

const CHANNEL_HZ: &str = "channel_hz";

pub struct EasNode {
    channel_hz: f64,
    /// False where the stage was handed audio a channel had already
    /// discriminated, in which case the front end below is not built.
    from_iq: bool,
    mixer: Mixer,
    decim: FirDecim,
    fm: FmDemod,
    bits: AfskBits,
    framer: eas::Framer,
    burst: eas::Assembler,
    audio_rate: f64,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    audio: Vec<f32>,
    symbols: Vec<dsp::afsk::Symbol>,
    meter: crate::FrameMeter,
    read: u64,
}

impl Default for EasNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl EasNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            from_iq: true,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            fm: FmDemod::new(AUDIO_HZ, DEVIATION_HZ),
            bits: AfskBits::with_tones(AUDIO_HZ, SAME, AfskConfig::default()),
            framer: eas::Framer::new(),
            burst: eas::Assembler::new(),
            audio_rate: AUDIO_HZ,
            mixed: Vec::new(),
            narrow: Vec::new(),
            audio: Vec::new(),
            symbols: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, channel_hz as u64, 6.0)
                .keyed_as(common::Modulation::Afsk),
            read: 0,
        }
    }

    /// Alerts published since the node was built.
    pub fn read(&self) -> u64 {
        self.read
    }

    /// Groups of bursts that framed and were no header: a channel with
    /// something SAME shaped on it that never reads is a different fault
    /// from a quiet channel.
    pub fn refused(&self) -> u64 {
        self.burst.refused()
    }

    /// Bit decisions from a block of audio into the framer, and whatever
    /// that and the vote produce.
    fn read_audio(&mut self, out: &mut Vec<Vec<u8>>) {
        let mut symbols = std::mem::take(&mut self.symbols);
        symbols.clear();
        let audio = std::mem::take(&mut self.audio);
        self.bits.process(&audio, &mut symbols);
        for sym in &symbols {
            // A quiet channel still produces symbols, and clocking those in
            // is how a preamble gets invented out of nothing.
            let burst = match sym.quiet {
                true => self.framer.quiet(),
                false => self.framer.push(sym.mark),
            };
            if let Some(burst) = burst
                && let Some(header) = self.burst.push(burst)
            {
                out.push(header);
            }
        }
        // The three copies take about a second each with a second between
        // them, so time passing is what ends a group of two.
        if let Some(header) = self.burst.advance(audio.len() as f64 / self.audio_rate.max(1.0)) {
            out.push(header);
        }
        self.audio = audio;
        self.symbols = symbols;
    }
}

impl Simple for EasNode {
    fn name(&self) -> &str {
        "eas"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        let rate = i.spec.rate;
        self.from_iq = match i.spec.kind {
            PortKind::Iq => true,
            // Audio a listening channel already discriminated.
            PortKind::Real | PortKind::Voice => false,
            _ => return Err(common::Error::other("eas reads complex baseband or audio")),
        };
        let audio_rate = if self.from_iq {
            let center = i.spec.center.as_f64();
            if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
                return Err(common::Error::other("eas needs its channel inside the span"));
            }
            let factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
            let audio_rate = rate / factor as f64;
            self.mixer = Mixer::new(center - self.channel_hz, rate);
            self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
            self.fm = FmDemod::new(audio_rate, DEVIATION_HZ);
            audio_rate
        } else {
            rate
        };
        if audio_rate < 4.0 * SAME.mark_hz {
            return Err(common::Error::other("eas needs room for the 2083 Hz mark tone"));
        }
        self.audio_rate = audio_rate;
        self.bits = AfskBits::with_tones(audio_rate, SAME, AfskConfig::default());
        self.framer.reset();
        self.burst.reset();
        // Measured on the channel rather than on the span, and kept for six
        // seconds because an alert is three copies and a row carries the
        // samples the last of them was read from.
        self.meter = crate::FrameMeter::new(audio_rate, self.channel_hz as u64, 6.0)
            .keyed_as(common::Modulation::Afsk);

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(CHANNEL_HZ, self.channel_hz, 1e5..=1e10).unit("Hz").label("Channel")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => {
                self.channel_hz = v.as_f64().unwrap_or(self.channel_hz);
                Ok(())
            }
            _ => Err(common::Error::other(format!("eas: unknown parameter {name:?}"))),
        }
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = vec![("read".into(), self.read.to_string())];
        if self.refused() > 0 {
            out.push(("refused".into(), self.refused().to_string()));
        }
        out
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        self.audio.clear();
        if self.from_iq {
            let Some(iq) = i.as_iq() else { return Ok(()) };
            self.mixed.clear();
            self.mixer.process(iq, &mut self.mixed);
            self.narrow.clear();
            self.decim.process(&self.mixed, &mut self.narrow);
            self.meter.feed(&self.narrow);
            let mut audio = std::mem::take(&mut self.audio);
            self.fm.process(&self.narrow, &mut audio);
            self.audio = audio;
        } else {
            let Some(samples) = i.as_real() else { return Ok(()) };
            // Audio is what there is to measure here: whatever cut this
            // channel out measured the radio, and this stage measures what
            // it was handed.
            self.narrow.clear();
            self.narrow.extend(samples.iter().map(|&s| common::C32::new(s, 0.0)));
            self.meter.feed(&self.narrow);
            self.audio.extend_from_slice(samples);
        }

        let mut headers = Vec::new();
        self.read_audio(&mut headers);
        for header in headers {
            self.read += 1;
            o.packets_mut().push(self.meter.packet_now(header));
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.fm.reset();
        self.bits.reset();
        self.framer.reset();
        self.burst.reset();
        self.meter.reset();
    }
}

impl Protocol for Eas {
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

    /// The seven weather radio channels, which is where it can be found
    /// without anybody tuning anything. A station relaying an alert on its
    /// own channel is read by putting this stage on that channel's audio.

    /// The header says what it is: `ZCZC` and a form nothing else on the bus
    /// has. That matters because an alert rides any FM channel anybody
    /// relays it on, so where it was heard says nothing about it.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        read(bytes).map(|d| vec![d])
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} EAS", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: CHANNEL_WIDTH_HZ, label: "EAS".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "eas",
    summary: "One FM channel: the SAME header in front of an emergency alert",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(EasNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};
    use decode::eas::Header;

    const RATE: f64 = 48_000.0;
    const AUDIO: f64 = 24_000.0;

    /// The National Weather Service's own example header: a tornado warning
    /// for two Missouri counties, half an hour, from the Kansas City office.
    const TOR: &str = "ZCZC-WXR-TOR-029095-029183+0030-1250100-KEAX/NWS-";

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    fn noise() -> impl FnMut() -> f32 {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        }
    }

    /// `copies` headers keyed as SAME audio, a second of silence between
    /// them, as a weather radio sends them.
    fn keyed_audio(header: &str, copies: usize) -> Vec<f32> {
        let mut audio = vec![0.0; AUDIO as usize / 2];
        for _ in 0..copies {
            audio.extend(dsp::afsk::modulate(&eas::encode_bits(header), AUDIO, SAME));
            audio.extend(std::iter::repeat_n(0.0, AUDIO as usize));
        }
        audio
    }

    /// The same, keyed onto an FM channel with `offset` hertz of mistuning
    /// and `noise` of added noise.
    fn keyed(header: &str, copies: usize, offset: f64, level: f32) -> Vec<C32> {
        let audio = keyed_audio(header, copies);
        let mut rng = noise();
        let per = (RATE / AUDIO) as usize;
        let mut iq = Vec::with_capacity(audio.len() * per);
        let mut phase = 0.0f64;
        for &a in &audio {
            for _ in 0..per {
                phase += std::f64::consts::TAU * (f64::from(a) * DEVIATION_HZ + offset) / RATE;
                iq.push(C32::new(
                    phase.cos() as f32 + level * rng(),
                    phase.sin() as f32 + level * rng(),
                ));
            }
        }
        iq
    }

    fn run(
        node: &mut EasNode,
        input: impl Fn(usize) -> Payload,
        blocks: usize,
    ) -> Vec<common::packet::Packet> {
        let ins = [spec(RATE, DEFAULT_HZ)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for b in 0..blocks {
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input(b), &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                frames.extend(f);
            }
        }
        frames
    }

    fn run_iq(node: &mut EasNode, iq: &[C32]) -> Vec<common::packet::Packet> {
        let chunks: Vec<&[C32]> = iq.chunks(4096).collect();
        run(node, |b| Payload::Iq(chunks[b].to_vec()), chunks.len())
    }

    fn run_audio(node: &mut EasNode, audio: &[f32]) -> Vec<common::packet::Packet> {
        let chunks: Vec<&[f32]> = audio.chunks(2048).collect();
        run(node, |b| Payload::Real(chunks[b].to_vec()), chunks.len())
    }

    fn iq_node(center: f64) -> EasNode {
        let mut n = EasNode::new(center);
        n.negotiate(&spec(RATE, center)).unwrap();
        n
    }

    fn audio_node() -> EasNode {
        let mut n = EasNode::new(DEFAULT_HZ);
        let spec = StreamSpec { kind: PortKind::Real, rate: AUDIO, ..Default::default() };
        n.negotiate(&PortSpec { spec, latency: 0 }).unwrap();
        n
    }

    #[test]
    fn the_node_refuses_a_span_without_its_channel() {
        let mut n = EasNode::default();
        assert!(n.negotiate(&spec(2_400_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(2_400_000.0, 155_000_000.0)).is_err());
        // Audio too slow for the mark tone is refused as surely.
        let slow = StreamSpec { kind: PortKind::Real, rate: 8_000.0, ..Default::default() };
        assert!(n.negotiate(&PortSpec { spec: slow, latency: 0 }).is_err());
    }

    /// The whole path on synthetic RF: three copies keyed onto a weather
    /// radio channel, into the node, out as the warning that was sent.
    #[test]
    fn three_keyed_copies_come_back_as_one_alert() {
        let mut n = iq_node(DEFAULT_HZ);
        let frames = run_iq(&mut n, &keyed(TOR, 3, 0.0, 0.0));
        assert_eq!(frames.len(), 1, "{} alerts off the air", frames.len());
        assert_eq!(n.read(), 1);
        assert_eq!(n.refused(), 0);

        assert!(frames[0].carrier.rssi_dbfs.is_finite() && frames[0].carrier.snr_db.is_finite());
        let d = read(frames[0].bytes()).expect("a decode");
        assert_eq!((d.id, d.kind), ("eas", "alert"));
        // A machine emitted it and it interrupts somebody, so it is an alert
        // and never a message. The whole of what the station sent is its
        // text; the counties and the times are in the frame.
        let common::packet::Fact::Alert(a) = &d.facts[0] else {
            panic!("an alert, got {:?}", d.facts);
        };
        assert_eq!(a.kind, common::packet::AlertKind::Weather);
        assert_eq!(a.severity, common::packet::Severity::Immediate);
        let said = a.text.clone().unwrap_or_default();
        assert!(said.contains("Tornado Warning"), "{said}");
        assert!(said.contains("KEAX/NWS"), "{said}");
        assert!(d.wrote().is_none(), "a machine emitted it, nobody wrote it");
    }

    /// The end of message burst closes the alert and reads as itself.
    #[test]
    fn the_end_of_message_burst_reads() {
        let mut n = iq_node(DEFAULT_HZ);
        let frames = run_iq(&mut n, &keyed("NNNN", 3, 0.0, 0.0));
        assert_eq!(frames.len(), 1);
        let d = read(frames[0].bytes()).expect("a decode");
        assert_eq!(d.kind, "end_of_message");
        assert!(d.subject.is_none(), "the end of a message names nobody");
    }

    /// The other input: audio a listening channel already discriminated,
    /// which is how an alert relayed on somebody else's FM channel is read.
    #[test]
    fn the_same_alert_reads_off_a_channel_audio() {
        let mut n = audio_node();
        let frames = run_audio(&mut n, &keyed_audio(TOR, 3));
        assert_eq!(frames.len(), 1, "{} alerts off the audio", frames.len());
        assert_eq!(
            eas::parse(&frames[0].bytes()),
            eas::parse(TOR.trim_end_matches('-').as_bytes())
        );
    }

    /// Two copies and then silence still publish: an alert held back for a
    /// third copy that the air ate is an alert nobody was told about.
    #[test]
    fn two_copies_and_silence_still_publish() {
        let mut n = iq_node(DEFAULT_HZ);
        let mut iq = keyed(TOR, 2, 0.0, 0.0);
        // Seven seconds of quiet, which is past the assembler's wait.
        iq.extend(std::iter::repeat_n(C32::new(0.0, 0.0), (RATE * 7.0) as usize));
        let frames = run_iq(&mut n, &iq);
        assert_eq!(frames.len(), 1);
        assert_eq!(n.read(), 1);
    }

    /// Nobody is tuned exactly, and mistuning costs an AFSK alert almost
    /// nothing: an offset is a constant in the discriminator's output and
    /// the tone correlators do not look at DC. Measured on this synthetic
    /// alert: it reads whole to 6 kHz out, which is past the deviation, and
    /// is gone at 8 kHz, where the channel filter has taken it.
    #[test]
    fn a_mistuned_alert_still_reads() {
        for offset in [-6_000.0, -3_000.0, 0.0, 3_000.0, 6_000.0] {
            let mut n = iq_node(DEFAULT_HZ);
            let frames = run_iq(&mut n, &keyed(TOR, 3, offset, 0.0));
            assert_eq!(frames.len(), 1, "{offset} Hz off: {} alerts", frames.len());
        }
        let mut n = iq_node(DEFAULT_HZ);
        assert_eq!(run_iq(&mut n, &keyed(TOR, 3, 8_000.0, 0.0)).len(), 0);
    }

    /// An alert under noise still reads, and a worse one reads nothing
    /// rather than a wrong county. Measured on this synthetic alert: noise
    /// of 1.2 times the carrier amplitude across the 48 kS/s channel still
    /// reads all three copies, and at 1.3 times nothing reads at all.
    #[test]
    fn an_alert_under_noise_reads_and_a_worse_one_reads_nothing() {
        let read = |level| {
            let mut n = iq_node(DEFAULT_HZ);
            let frames = run_iq(&mut n, &keyed(TOR, 3, 0.0, level));
            (frames.len(), n.refused(), frames.first().and_then(|f| eas::parse(f.bytes())))
        };
        let (count, refused, alert) = read(1.2);
        assert_eq!((count, refused), (1, 0));
        let Some(Header::Alert(a)) = alert else { panic!("not an alert") };
        assert_eq!((a.event_code.as_str(), a.station.as_str()), ("TOR", "KEAX/NWS"));
        assert_eq!(read(1.3).0, 0, "an alert was read at 1.3 times the noise");
        assert_eq!(read(3.0).0, 0, "an alert was invented at three times the noise");
    }

    /// Two minutes of noise produces nothing. The preamble and the form of
    /// the header are what stand between a busy channel and an invented
    /// tornado warning.
    #[test]
    fn noise_produces_no_alerts() {
        let mut rng = noise();
        let iq: Vec<C32> =
            (0..(RATE * 120.0) as usize).map(|_| C32::new(rng() * 0.3, rng() * 0.3)).collect();
        let mut n = iq_node(DEFAULT_HZ);
        let frames = run_iq(&mut n, &iq);
        assert_eq!(frames.len(), 0, "noise made {} alerts", frames.len());
        assert_eq!(n.read(), 0);
    }
}
