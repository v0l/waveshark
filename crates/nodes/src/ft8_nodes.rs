//! FT8 and FT4 as graph nodes: one HF channel in, every station in it out.
//!
//! The channel is 3 kHz of an SSB passband and it holds dozens of
//! transmissions at once, so this does not tune a signal. It mixes the dial
//! to zero, decimates to 12 kHz, and hands whole slots to
//! [`dsp::mfsk::Slot`], which finds every Costas-synchronised transmission in
//! the passband and hands back soft bits for each. The (174,91) code and the
//! CRC-14 behind it are [`decode::ft8`], and what reaches the bus is a
//! transmission that satisfied both.
//!
//! # The grid
//!
//! A station keys at the start of a fifteen-second slot (seven and a half for
//! FT4) and stops 12.6 seconds later, so the slot boundary is where the
//! decoder cuts and a cut in the wrong place loses every transmission that
//! straddles it. Nothing else the receiver reads is aligned to a clock and
//! this is not either: [`decode::ft8::Slots`] finds the grid in the air, so a
//! replayed capture is cut where the recording was keyed rather than where
//! the machine's clock happens to be, and a live receiver reads a band its
//! own clock is minutes wrong about.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Mark, Placed, Placement, Protocol, Shape, Stickiness};
use common::Result;
use decode::ft8;
pub use decode::ft8::Mode;
pub use decode::ft8::PASSBAND_HZ;
pub use decode::ft8::read;
pub use decode::ft8::unpack_bits;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::ft8::Ft8;
pub use identify::ft8::{
    AUDIO_HZ, CHANNEL_WIDTH_HZ, DEFAULT_HZ, FT4_DEFAULT_HZ, FT4_DIALS, FT8_DIALS, Ft4, shape,
};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

pub struct Ft8Node {
    dial_hz: f64,
    mode: Mode,
    rate: f64,
    factor: usize,
    mixer: Mixer,
    decim: FirDecim,
    slots: ft8::Slots,
    mixed: Vec<common::C32>,
    audio: Vec<common::C32>,
    heard: Vec<ft8::Transmission>,
    meter: crate::FrameMeter,
    read: u64,
}

impl Default for Ft8Node {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, Mode::Ft8)
    }
}

impl Ft8Node {
    pub fn new(dial_hz: f64, mode: Mode) -> Self {
        Self {
            dial_hz,
            mode,
            rate: AUDIO_HZ,
            factor: 1,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(AUDIO_HZ, 1, PASSBAND_HZ + 500.0, 60.0),
            slots: ft8::Slots::new(AUDIO_HZ, mode),
            mixed: Vec::new(),
            audio: Vec::new(),
            heard: Vec::new(),
            meter: crate::FrameMeter::new(AUDIO_HZ, dial_hz as u64, 1.0)
                .keyed_as(common::Modulation::Fsk8),
            read: 0,
        }
    }

    /// Transmissions read since the node was built.
    pub fn read_count(&self) -> u64 {
        self.read
    }

    /// Windows looked at since the node was built: whole slots once the grid
    /// is known, double-length searches before that.
    pub fn windows(&self) -> u64 {
        self.slots.windows()
    }

    fn audio_rate(&self) -> f64 {
        self.rate / self.factor as f64
    }

    fn rebuild(&mut self) {
        let audio = self.audio_rate();
        self.decim = FirDecim::design_hz(self.rate, self.factor, PASSBAND_HZ + 500.0, 60.0);
        self.slots = ft8::Slots::new(audio, self.mode);
        self.meter = crate::FrameMeter::new(audio, self.dial_hz as u64, 1.0)
            .keyed_as(common::Modulation::Fsk8);
    }
}

impl Simple for Ft8Node {
    fn name(&self) -> &str {
        match self.mode {
            Mode::Ft8 => FT8_DESC.name,
            Mode::Ft4 => FT4_DESC.name,
        }
    }

    fn readings(&self) -> Vec<(String, String)> {
        let grid = match self.slots.grid_s() {
            Some(s) => format!("{s:.1} s"),
            None => "searching".into(),
        };
        vec![
            ("windows".into(), self.windows().to_string()),
            ("read".into(), self.read.to_string()),
            ("grid".into(), grid),
        ]
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("ft8 reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.dial_hz - center).abs() > rate / 2.0 - PASSBAND_HZ {
            return Err(common::Error::other("ft8 needs its passband inside the span"));
        }
        self.rate = rate;
        self.factor = (rate / AUDIO_HZ).round().max(1.0) as usize;
        if self.audio_rate() < 2.0 * PASSBAND_HZ {
            return Err(common::Error::other("ft8 needs a 3 kHz passband"));
        }
        self.mixer = Mixer::new(center - self.dial_hz, rate);
        self.rebuild();

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.dial_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.audio.clear();
        self.decim.process(&self.mixed, &mut self.audio);
        self.meter.feed(&self.audio);

        let audio = std::mem::take(&mut self.audio);
        let mut heard = std::mem::take(&mut self.heard);
        self.slots.push(&audio, &mut heard);
        self.audio = audio;
        self.read += heard.len() as u64;
        for t in heard.drain(..) {
            let center = (self.dial_hz + t.freq_hz).round().max(0.0) as u64;
            // The decoder measured this signal against the slot's own
            // noise, which is a better number than the channel's mean,
            // and the tone it was found on is where it was heard.
            let mut p = self.meter.packet_now(t.bytes).at_center(center);
            p.carrier.snr_db = t.snr_db;
            o.packets_mut().push(p);
        }
        self.heard = heard;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.slots.reset();
        self.meter.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![Param::float(CHANNEL_HZ, self.dial_hz, 1e5..=1e9).unit("Hz").label("Dial")]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            CHANNEL_HZ => self.dial_hz = v.as_f64().unwrap_or(self.dial_hz),
            _ => return Err(common::Error::other(format!("ft8: unknown parameter {name:?}"))),
        }
        self.rebuild();
        Ok(())
    }
}

/// Whether a frame off the bus is one of these: the mode byte a front end
/// wrote, the length, and the check the transmitter computed.
fn is_ftx(bytes: &[u8]) -> bool {
    bytes.len() == 1 + ft8::MESSAGE_BITS.div_ceil(8)
        && bytes.first().copied().and_then(Mode::of_tag).is_some()
        && ft8::crc_ok(&unpack_bits(&bytes[1..], ft8::MESSAGE_BITS))
}

impl Protocol for Ft8 {
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

    /// The dials stations are told to use. A station is not found anywhere
    /// else, because the whole mode depends on everybody being in the same
    /// 3 kHz.

    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    fn reports_position(&self) -> bool {
        true
    }
    /// The dial is HF and the frame carries the mode byte its front end
    /// wrote, so the claim is by that tag rather than by where it was heard.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !is_ftx(bytes) || bytes[0] != Mode::Ft8.tag() {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} FT8", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz: hz + PASSBAND_HZ / 2.0, width_hz: PASSBAND_HZ, label: "FT8".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(FT8_DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

impl Protocol for Ft4 {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::InBursts
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }

    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Tagged
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        if !is_ftx(bytes) || bytes[0] != Mode::Ft4.tag() {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} FT4", hz / 1e6)
    }
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz: hz + PASSBAND_HZ / 2.0, width_hz: PASSBAND_HZ, label: "FT4".into() }]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(FT4_DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

const CHANNEL_HZ: &str = "channel_hz";

pub const FT8_DESC: StageDesc = StageDesc {
    name: "ft8",
    summary: "One FT8 passband: every station in 3 kHz, on the fifteen-second clock",
    category: Category::Decode,
    feeds_bus: true,
};

pub const FT4_DESC: StageDesc = StageDesc {
    name: "ft4",
    summary: "One FT4 passband: every station in 3 kHz, on the seven-and-a-half-second clock",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build_ft8(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Ft8Node::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), Mode::Ft8)))
}

pub fn build_ft4(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(Ft8Node::new(s.f64_or(CHANNEL_HZ, FT4_DEFAULT_HZ), Mode::Ft4)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};
    use dsp::mfsk;
    use std::f64::consts::TAU;

    fn spec(rate: f64, center: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(center as u64)), latency: 0 }
    }

    /// One transmission keyed into a slot: the message packed, checked,
    /// coded and mapped to tones, then keyed at `audio_hz` above the dial
    /// starting `at_s` into the slot.
    fn keyed(
        mode: Mode,
        payload: [bool; ft8::PAYLOAD_BITS],
        rate: f64,
        audio_hz: f64,
        at_s: f64,
        amplitude: f32,
    ) -> Vec<C32> {
        let wf = mode.waveform();
        let mut payload = payload;
        if mode == Mode::Ft4 {
            ft8::scramble_ft4(&mut payload);
        }
        let word = ft8::encode(&payload);
        let mut tones = vec![0u8; wf.symbols];
        for group in wf.sync {
            tones[group.at..group.at + group.tones.len()].copy_from_slice(group.tones);
        }
        let per = wf.bits_per_symbol();
        let mut taken = 0usize;
        for (from, to) in wf.data {
            for slot in tones.iter_mut().take(*to).skip(*from) {
                let pattern =
                    (0..per).fold(0usize, |acc, k| acc << 1 | usize::from(word[taken + k]));
                taken += per;
                *slot = wf.gray[pattern];
            }
        }

        let mut out = vec![C32::default(); (rate * wf.slot_s) as usize];
        let symbol = (rate / wf.baud).round() as usize;
        let start = (at_s * rate) as usize;
        let mut phase = 0.0f64;
        for (s, tone) in tones.iter().enumerate() {
            let f = audio_hz + *tone as f64 * wf.baud;
            for k in 0..symbol {
                let at = start + s * symbol + k;
                if at >= out.len() {
                    break;
                }
                phase += TAU * f / rate;
                out[at] += amplitude * C32::new(phase.cos() as f32, phase.sin() as f32);
            }
        }
        out
    }

    /// Feed a stream through the node, from the slot boundary, and collect
    /// what reached the bus.
    fn run(node: &mut Ft8Node, iq: &[C32], rate: f64, center: f64) -> Vec<common::packet::Packet> {
        let ins = [spec(rate, center)];
        let tags = Vec::new();
        let mut frames = Vec::new();
        for block in iq.chunks(4096) {
            let input = Payload::Iq(block.to_vec());
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            node.process(&input, &mut out, &mut ctx).unwrap();
            if let Payload::Packets(f) = out {
                frames.extend(f);
            }
        }
        frames
    }

    /// A node tuned to a dial, reading a stream that starts wherever it
    /// starts: nothing tells it where the grid is.
    fn tuned(dial: f64, mode: Mode) -> Ft8Node {
        let mut node = Ft8Node::new(dial, mode);
        node.negotiate(&spec(AUDIO_HZ, dial)).unwrap();
        node
    }

    /// Silence enough to finish the search window, which is two slots: with
    /// no grid the node reads double-length windows, and a stream of one
    /// slot never fills one.
    fn padded(mode: Mode, mut iq: Vec<C32>) -> Vec<C32> {
        iq.resize(2 * (AUDIO_HZ * mode.waveform().slot_s) as usize, C32::default());
        iq
    }

    /// The whole path: a station calling CQ, keyed 1 kHz up the passband,
    /// read back as the message it sent with the square it sent it from.
    #[test]
    fn a_cq_is_read_off_a_slot() {
        let payload = ft8::pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let iq = padded(Mode::Ft8, keyed(Mode::Ft8, payload, AUDIO_HZ, 1_000.0, 0.5, 1.0));
        let mut node = tuned(DEFAULT_HZ, Mode::Ft8);
        let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);

        assert_eq!(frames.len(), 1, "{} transmissions", frames.len());
        assert_eq!(node.windows(), 1, "one search window read");
        let grid = node.slots.grid_s().expect("the grid, found in the air");
        assert!(
            (grid - 0.25).abs() < 0.1,
            "the cut is 0.25 s before the station, which keyed 0.5 s in, so the grid is at \
             0.25 s and not {grid:.2} s"
        );
        let f = &frames[0];
        // The frame is heard where the station was: the dial plus where it
        // sat in the passband, to within a tone.
        assert!(
            f.carrier.center_hz.abs_diff((DEFAULT_HZ + 1_000.0) as u64) <= 7,
            "{}",
            f.carrier.center_hz
        );
        assert!(f.carrier.snr_db.is_finite() && f.carrier.rssi_dbfs.is_finite());
        assert!(f.carrier.iq.is_some(), "the samples it was read from");

        let d = read(f.bytes()).expect("a decode");
        assert_eq!((d.id, d.kind), ("ft8", "message"));
        // An operator's station called another, which is somebody writing.
        assert_eq!(d.wrote(), Some("CQ MI0ABC IO74"));
        assert_eq!(d.parties(), (Some("MI0ABC"), Some("CQ")));
        let p = d.placed().expect("the square it sent");
        assert!((p.lat - 54.5).abs() < 1e-6 && (p.lon - -5.0).abs() < 1e-6);
    }

    /// The point of the mode: a passband holds a whole conversation's worth
    /// of stations at once, and one pass over the slot reads all of them.
    #[test]
    fn eight_stations_in_one_passband_are_all_read() {
        let sent = [
            ("CQ", "MI0ABC", "IO74"),
            ("MI0ABC", "G4XYZ", "IO91"),
            ("G4XYZ", "MI0ABC", "-12"),
            ("MI0ABC", "G4XYZ", "R-08"),
            ("G4XYZ", "MI0ABC", "RRR"),
            ("CQ", "EI7DEF", "IO53"),
            ("EI7DEF", "DL1GHI", "JO31"),
            ("DL1GHI", "EI7DEF", "73"),
        ];
        let mut iq = vec![C32::default(); 2 * (AUDIO_HZ * mfsk::FT8.slot_s) as usize];
        for (k, (to, from, extra)) in sent.iter().enumerate() {
            let payload = ft8::pack_standard(to, from, extra).expect(from);
            // Spread across the passband, each starting at its own moment
            // within the two seconds a station may be late by.
            let hz = 400.0 + 300.0 * k as f64;
            let at = 0.2 + 0.15 * (k % 4) as f64;
            for (a, b) in iq.iter_mut().zip(keyed(Mode::Ft8, payload, AUDIO_HZ, hz, at, 0.5)) {
                *a += b;
            }
        }
        let mut node = tuned(DEFAULT_HZ, Mode::Ft8);
        let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);
        assert_eq!(frames.len(), 8, "{} of 8 stations read", frames.len());
        assert_eq!(node.read_count(), 8);

        let mut read: Vec<String> = frames
            .iter()
            .map(|f| read(f.bytes()).expect("a decode").wrote().unwrap().to_string())
            .collect();
        read.sort();
        let mut want: Vec<String> = sent.iter().map(|(a, b, c)| format!("{a} {b} {c}")).collect();
        want.sort();
        assert_eq!(read, want);
    }

    /// FT4 is the same slot read at four tones on a seven-and-a-half-second
    /// clock, with the payload keyed through its scrambling sequence.
    #[test]
    fn an_ft4_exchange_is_read() {
        let payload = ft8::pack_standard("G4XYZ", "MI0ABC", "R+05").unwrap();
        let iq = padded(Mode::Ft4, keyed(Mode::Ft4, payload, AUDIO_HZ, 1_500.0, 0.4, 1.0));
        let mut node = tuned(FT4_DEFAULT_HZ, Mode::Ft4);
        let frames = run(&mut node, &iq, AUDIO_HZ, FT4_DEFAULT_HZ);
        assert_eq!(frames.len(), 1, "{} transmissions", frames.len());
        let d = read(&frames[0].bytes()).expect("a decode");
        assert_eq!((d.id, d.kind), ("ft4", "message"));
        assert_eq!(d.wrote(), Some("G4XYZ MI0ABC R+05"));
        // A report is a signal report, and says nothing about where either
        // station is: only a grid square does that.
        assert_eq!(d.placed(), None, "a report says nothing about where");
    }

    /// A station keyed into noise 10 dB below it, which is what a quiet
    /// band looks like, and the same station buried in noise that is
    /// louder than it, which is what the mode is for and what this
    /// receiver does not yet reach.
    #[test]
    fn a_station_reads_through_noise_until_it_does_not() {
        let payload = ft8::pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let mut seed = 0xfeed_face_dead_beefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        // A tone against noise filling the passband, by amplitude. What
        // the decoder itself reports for each is in the assertion: it reads
        // a station down to about -15 dB in 2500 Hz and loses it by -18,
        // where WSJT-X is still reading to around -21.
        for (amplitude, want, snr) in [(0.10f32, 1, -10.0), (0.05, 1, -15.2), (0.035, 0, 0.0)] {
            let mut iq =
                padded(Mode::Ft8, keyed(Mode::Ft8, payload, AUDIO_HZ, 1_000.0, 0.5, amplitude));
            for s in iq.iter_mut() {
                *s += C32::new(rng(), rng());
            }
            let mut node = tuned(DEFAULT_HZ, Mode::Ft8);
            let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);
            assert_eq!(frames.len(), want, "at an amplitude of {amplitude}");
            if let Some(f) = frames.first() {
                assert!(
                    (f.carrier.snr_db - snr).abs() < 1.0,
                    "{} dB at {amplitude}",
                    f.carrier.snr_db
                );
            }
            for f in &frames {
                let d = read(f.bytes()).expect("a decode");
                assert_eq!(d.wrote(), Some("CQ MI0ABC IO74"), "wrong text at {amplitude}");
            }
        }
    }

    /// A minute of noise, and nothing comes off it: the code has to
    /// converge and the CRC-14 has to pass, and noise does neither.
    ///
    /// A band with nothing on it is searched for ever, which costs no more
    /// than cutting it into slots did: this minute takes 0.16 s of one core
    /// as two search windows and took 0.23 s as four slots, because the
    /// transform over the samples is the work and the sync search over the
    /// offsets a longer window adds is not.
    #[test]
    fn noise_produces_no_transmissions() {
        let mut seed = 0x0123_4567_89ab_cdefu64;
        let mut rng = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 40) as f32 / 8_388_608.0 - 1.0
        };
        let iq: Vec<C32> =
            (0..(AUDIO_HZ * 60.0) as usize).map(|_| C32::new(rng(), rng())).collect();
        let mut node = tuned(DEFAULT_HZ, Mode::Ft8);
        let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);
        assert_eq!(
            node.windows(),
            2,
            "noise never says where the grid is, so a minute of it is two search windows \
             rather than four slots"
        );
        assert_eq!(node.slots.grid_s(), None, "noise is not a grid");
        assert_eq!(frames.len(), 0, "{} transmissions out of noise", frames.len());
    }

    /// The point of the search: the same stream started at any moment of the
    /// slot reads the same transmissions.
    ///
    /// A replayed capture is the case that matters. The wall clock says
    /// nothing about samples recorded minutes or years ago, and cutting at
    /// where it happens to be loses every transmission that straddles the
    /// cut: a station is on the air for 12.64 of the 15 seconds, so a cut
    /// more than 2.36 seconds into the slot takes part of one.
    ///
    /// Seven slots, five of them keyed: the first is empty so that a phase
    /// anywhere inside it still hands over every transmission whole, and
    /// the last gives the fifth slot room to finish.
    #[test]
    fn a_stream_is_read_at_any_phase_of_the_slot() {
        let sent = [
            ("CQ", "MI0ABC", "IO74"),
            ("MI0ABC", "G4XYZ", "IO91"),
            ("G4XYZ", "MI0ABC", "-12"),
            ("MI0ABC", "G4XYZ", "R-08"),
            ("CQ", "EI7DEF", "IO53"),
        ];
        let slot = (AUDIO_HZ * mfsk::FT8.slot_s) as usize;
        let mut iq = vec![C32::default(); 7 * slot];
        for (k, (to, from, extra)) in sent.iter().enumerate() {
            let payload = ft8::pack_standard(to, from, extra).expect(from);
            let at = (k + 1) * slot;
            let one = keyed(Mode::Ft8, payload, AUDIO_HZ, 1_000.0 + 200.0 * k as f64, 0.4, 1.0);
            for (a, b) in iq[at..].iter_mut().zip(one) {
                *a += b;
            }
        }
        for phase_s in [0.0, 1.5, 3.7, 7.0, 11.2, 14.5] {
            let from = (phase_s * AUDIO_HZ) as usize;
            let mut node = tuned(DEFAULT_HZ, Mode::Ft8);
            let frames = run(&mut node, &iq[from..], AUDIO_HZ, DEFAULT_HZ);
            let mut read: Vec<String> = frames
                .iter()
                .map(|f| read(f.bytes()).expect("a decode").wrote().unwrap().to_string())
                .collect();
            read.sort();
            let mut want: Vec<String> =
                sent.iter().map(|(a, b, c)| format!("{a} {b} {c}")).collect();
            want.sort();
            assert_eq!(read, want, "started {phase_s} s into the slot");
            // The grid is where the stations keyed, which is 0.4 seconds in
            // less the quarter second the cut is placed before them, seen
            // from wherever the stream was cut into.
            let grid = node.slots.grid_s().expect("the grid");
            let want = (0.15 - phase_s).rem_euclid(mfsk::FT8.slot_s);
            assert!((grid - want).abs() < 0.2, "the grid is at {grid:.2} s, wanted {want:.2} s");
        }
    }

    /// A grid found once is not kept for ever: a stream that stops saying
    /// where it is, because the band went quiet or the replay moved, is
    /// searched again after a minute of slots with nothing in them.
    #[test]
    fn a_quiet_minute_sends_the_node_looking_for_the_grid_again() {
        let payload = ft8::pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let slot = (AUDIO_HZ * mfsk::FT8.slot_s) as usize;
        let mut iq = vec![C32::default(); 7 * slot];
        for (a, b) in iq.iter_mut().zip(keyed(Mode::Ft8, payload, AUDIO_HZ, 1_000.0, 0.4, 1.0)) {
            *a += b;
        }
        let mut node = tuned(DEFAULT_HZ, Mode::Ft8);
        let frames = run(&mut node, &iq, AUDIO_HZ, DEFAULT_HZ);
        assert_eq!(frames.len(), 1, "the one station that keyed");
        // One search window, then four empty slots, and the fourth gives the
        // grid up: what is left is under a window and is not read at all.
        assert_eq!(node.windows(), 5, "a search window and four slots");
        assert_eq!(node.slots.grid_s(), None, "the grid was given up");
    }

    /// A frame off the bus is claimed by the mode that keyed it and by
    /// nothing else: the tag byte says which, and the check refuses a frame
    /// that is neither.
    #[test]
    fn a_frame_is_claimed_by_the_mode_that_wrote_it() {
        let payload = ft8::pack_standard("CQ", "MI0ABC", "IO74").unwrap();
        let message = ft8::with_crc(&payload);
        let mut bytes = vec![8u8];
        bytes.extend(ft8::pack(&message));
        assert!(is_ftx(&bytes));
        let p = crate::measured(DEFAULT_HZ as u64, PASSBAND_HZ as u32, bytes.clone(), -40.0, 12.0);
        assert_eq!(Ft8.stated(&p).map(|r| r.len()), Some(1));
        assert_eq!(Ft4.stated(&p), None, "the tag says FT8");

        bytes[0] = 4;
        assert!(is_ftx(&bytes), "the same message keyed as FT4 checks too");
        let p = crate::measured(DEFAULT_HZ as u64, PASSBAND_HZ as u32, bytes.clone(), -40.0, 12.0);
        assert_eq!(Ft8.stated(&p), None);

        bytes[0] = 2;
        assert!(!is_ftx(&bytes), "no mode keys that");
        bytes[0] = 8;
        bytes[5] ^= 1;
        assert!(!is_ftx(&bytes), "a wrong bit fails the check");
        assert!(!is_ftx(&bytes[..8]), "too short to be a message");
    }

    #[test]
    fn the_node_refuses_a_span_without_its_passband() {
        let mut n = Ft8Node::default();
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(48_000.0, DEFAULT_HZ + 30_000.0)).is_err());
        // 8 kHz complex still holds a 3 kHz passband above the dial.
        assert!(n.negotiate(&spec(8_000.0, DEFAULT_HZ)).is_ok());
        assert!(n.negotiate(&spec(4_000.0, DEFAULT_HZ)).is_err(), "too narrow for 3 kHz");
    }
}
