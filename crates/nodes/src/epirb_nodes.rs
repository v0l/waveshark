//! COSPAS-SARSAT 406 MHz distress beacons as a stage: a channel in,
//! messages out.
//!
//! The waveform is [`dsp::biphase`] and the message is [`decode::epirb`];
//! what is here is the wiring between them. A burst is 160 ms of unmodulated
//! carrier and then 112 or 144 bits at 400 baud, so the demodulator has a
//! quarter of a second of carrier to settle its reference on before the
//! preamble arrives, and the whole transmission is over in half a second.
//!
//! The preamble is matched on the chips rather than on bits, which is what
//! says where a bit starts and which way up the modulation was. Both frame
//! synchronisation patterns are searched: a beacon under test keys the
//! complemented one so the satellites ignore it, and a receiver on the
//! ground should say what it heard either way.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::epirb::detail;
pub use decode::epirb::read;
use decode::epirb::{self};
use dsp::biphase::BiphaseDemod;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::epirb::BAND;
pub use identify::epirb::BAUD;
pub use identify::epirb::CHANNEL_WIDTH_HZ;
pub use identify::epirb::DEFAULT_HZ;
pub use identify::epirb::Epirb;
pub use identify::epirb::FEED_HZ;
pub use identify::epirb::WORK_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

pub struct EpirbNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    demod: BiphaseDemod,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    meter: crate::FrameMeter,
    framer: epirb::Framer,
    scratch: Vec<f32>,
}

impl Default for EpirbNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl EpirbNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(WORK_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            demod: BiphaseDemod::new(WORK_HZ, BAUD),
            mixed: Vec::new(),
            narrow: Vec::new(),
            meter: crate::FrameMeter::new(WORK_HZ, channel_hz as u64, 2.0)
                .keyed_as(common::Modulation::Psk2),
            framer: epirb::Framer::new(),
            scratch: Vec::new(),
        }
    }

    /// Messages that checked.
    pub fn frames(&self) -> u64 {
        self.framer.frames()
    }
}

impl Simple for EpirbNode {
    fn name(&self) -> &str {
        "epirb"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("epirb reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        self.channel_hz = center;
        let factor = (rate / WORK_HZ).round().max(1.0) as usize;
        let work = rate / factor as f64;
        self.mixer = Mixer::new(0.0, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.demod = BiphaseDemod::new(work, BAUD);
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 2.0)
            .keyed_as(common::Modulation::Psk2);
        self.framer.reset();

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.bandwidth = CHANNEL_WIDTH_HZ.min(rate);
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);

        let mut chips = std::mem::take(&mut self.scratch);
        chips.clear();
        self.demod.process(&self.narrow, &mut chips);
        for chip in &chips {
            if let Some(message) = self.framer.push(*chip) {
                o.packets_mut().push(self.meter.packet_now(message));
            }
        }
        self.scratch = chips;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.framer.reset();
        self.meter.reset();
    }
}

impl Protocol for Epirb {
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

    /// The whole allocation rather than one channel: beacons are assigned
    /// channels 3 kHz apart across it, and nothing else may transmit there,
    /// so anything found in the band is worth pointing this at.

    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND.1 - BAND.0) as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(BAND.0..BAND.1).contains(&hz) {
            return None;
        }
        Some(read(bytes).into_iter().collect())
    }
    fn reports_position(&self) -> bool {
        true
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new("epirb")]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "epirb",
    summary: "COSPAS-SARSAT 406 MHz distress beacons: EPIRBs, PLBs and ELTs",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(EpirbNode::default()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::C32;
    use decode::epirb::Mode;

    /// The message an EPIRB with an MMSI transmits, as bytes off the air.
    fn a_message() -> Vec<u8> {
        let num = |value: u64, width: usize| -> Vec<bool> {
            (0..width).rev().map(|i| value >> i & 1 != 0).collect()
        };
        let mut pdf1 = vec![true, false];
        pdf1.extend(num(232, 10));
        pdf1.extend(num(0b0010, 4));
        pdf1.extend(num(123_456, 20));
        pdf1.extend(num(1, 4));
        pdf1.push(false);
        pdf1.extend(num(53 * 4 + 1, 9));
        pdf1.push(true);
        pdf1.extend(num(10 * 4 + 1, 10));
        let mut pdf2 = vec![true, true, false, true, true, false];
        pdf2.push(true);
        pdf2.extend(num(6, 5));
        pdf2.extend(num(9, 4));
        pdf2.push(false);
        pdf2.extend(num(3, 5));
        pdf2.extend(num(7, 4));
        keyed_bytes(&pdf1, &pdf2)
    }

    fn keyed_bytes(pdf1: &[bool], pdf2: &[bool]) -> Vec<u8> {
        use decode::bits::{BCH_63_51_GEN, BCH_127_106_GEN, bch_parity};
        let mut bits = Mode::Distress.preamble();
        bits.extend_from_slice(pdf1);
        let p = bch_parity(pdf1, BCH_127_106_GEN, 21);
        bits.extend((0..21).rev().map(|i| p >> i & 1 != 0));
        bits.extend_from_slice(pdf2);
        let p = bch_parity(pdf2, BCH_63_51_GEN, 12);
        bits.extend((0..12).rev().map(|i| p >> i & 1 != 0));
        bits.chunks(8).map(|b| b.iter().fold(0u8, |acc, v| acc << 1 | u8::from(*v))).collect()
    }

    fn bits_of(bytes: &[u8]) -> Vec<bool> {
        bytes.iter().flat_map(|b| (0..8).map(move |i| b >> (7 - i) & 1 != 0)).collect()
    }

    /// A burst as a beacon sends it: 160 ms of unmodulated carrier, then the
    /// message keyed biphase-L at 1.1 radians, then silence.
    fn a_burst(bytes: &[u8], rate: f64, offset_hz: f64, quiet_s: f64) -> Vec<C32> {
        let carrier = |n: usize, from: usize| -> Vec<C32> {
            (from..from + n)
                .map(|i| {
                    C32::from_polar(
                        1.0,
                        (std::f64::consts::TAU * offset_hz * i as f64 / rate) as f32,
                    )
                })
                .collect()
        };
        let mut out = carrier((rate * 0.16) as usize, 0);
        out.extend(dsp::biphase::key_biphase(&bits_of(bytes), rate, BAUD, 1.1, offset_hz, 150e-6));
        out.extend(std::iter::repeat_n(C32::new(0.0, 0.0), (rate * quiet_s) as usize));
        out
    }

    fn noisy(iq: &[C32], amplitude: f32, seed: u64) -> Vec<C32> {
        let mut s = seed;
        let mut next = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 40) as f32 / 8_388_608.0 - 1.0
        };
        iq.iter().map(|x| x + C32::new(next() * amplitude, next() * amplitude)).collect()
    }

    fn heard(stream: &[C32], rate: f64) -> Vec<Vec<u8>> {
        let mut n = EpirbNode::new(DEFAULT_HZ);
        n.demod = BiphaseDemod::new(rate, BAUD);
        let mut got = Vec::new();
        for block in stream.chunks(1024) {
            let mut chips = Vec::new();
            n.demod.process(block, &mut chips);
            for chip in chips {
                if let Some(message) = n.framer.push(chip) {
                    got.push(message);
                }
            }
        }
        got
    }

    /// The whole of this file on a keyed burst: the carrier, the chips, the
    /// preamble matched on them, the message and the fields.
    #[test]
    fn a_keyed_burst_is_read_off_the_carrier() {
        let rate = 9_600.0;
        let air = a_message();
        let got = heard(&a_burst(&air, rate, 700.0, 0.1), rate);
        assert_eq!(got.len(), 1, "{} messages", got.len());
        assert_eq!(got[0], air, "the bytes are not the ones that were keyed");

        let d = read(&got[0]).expect("a decode");
        assert_eq!((d.id, d.kind), ("epirb", "distress"));
        assert_eq!(
            d.subject.as_ref().map(|e| e.id.to_string()).as_deref(),
            Some("1D043C4802FFBFF")
        );
        // Somebody is in trouble, which is the whole point of one of these.
        let common::packet::Fact::Alert(a) = &d.facts[1] else {
            panic!("an alert, got {:?}", d.facts);
        };
        assert_eq!(a.kind, common::packet::AlertKind::Distress);
        assert!(a.text.clone().unwrap_or_default().contains("EPIRB"), "{a:?}");
        let p = d.placed().expect("a position");
        assert!((p.lat - 53.36).abs() < 1e-6, "{}", p.lat);
        assert!((p.lon + 10.192_222).abs() < 1e-5, "{}", p.lon);
    }

    /// A beacon under test keys the complemented frame synchronisation so
    /// the satellites ignore it. A receiver on the ground says what it
    /// heard, and says which it was.
    #[test]
    fn a_self_test_burst_is_read_and_labelled() {
        let rate = 9_600.0;
        let mut air = a_message();
        // The self-test pattern is the last eight bits of the normal one
        // complemented, which is bits 17 to 24 of the message.
        // Bits 17 to 24 are the whole of the third byte.
        air[2] ^= 0xff;
        let got = heard(&a_burst(&air, rate, 0.0, 0.1), rate);
        assert_eq!(got.len(), 1, "{} messages", got.len());
        let d = read(&got[0]).expect("a decode");
        // A drill is an alert of its own kind, so a listener can tell it
        // from the real thing.
        assert_eq!(d.kind, "self_test");
        assert!(d.facts.iter().any(|f| matches!(
            f,
            common::packet::Fact::Alert(a) if a.kind == common::packet::AlertKind::Test
        )));
        assert_eq!(
            d.subject.as_ref().map(|e| e.id.to_string()).as_deref(),
            Some("1D043C4802FFBFF")
        );
    }

    /// How many of `n` bursts come back unchanged with noise of `amp` added
    /// to each component of every sample.
    fn read_bursts(n: usize, amp: f32, seed: u64) -> usize {
        let rate = 9_600.0;
        let air = a_message();
        let mut stream = Vec::new();
        for _ in 0..n {
            stream.extend(a_burst(&air, rate, 300.0, 0.05));
        }
        let stream = noisy(&stream, amp, seed);
        heard(&stream, rate).iter().filter(|m| **m == air).count()
    }

    /// Twenty bursts in noise, which is the whole of this file twenty times
    /// over. Every burst is read at 7.8 dB of signal to noise, which is what
    /// the chips themselves were measured to survive, and 19 of 20 at
    /// 6.2 dB, where the last one is lost in the demodulator rather than in
    /// the codes.
    #[test]
    fn a_beacon_repeating_in_noise_is_read_every_time() {
        let seed = 0x51ee_beef_1234_5678;
        assert_eq!(read_bursts(20, 0.5, seed), 20, "at 7.8 dB");
        assert_eq!(read_bursts(20, 0.6, seed), 19, "at 6.2 dB");
    }

    /// Ten minutes of noise produces no beacon. The preamble will match
    /// something eventually; the two codes are what refuse it.
    #[test]
    fn noise_produces_no_beacons() {
        let rate = 9_600.0;
        let quiet: Vec<C32> = vec![C32::new(0.0, 0.0); (rate * 600.0) as usize];
        let stream = noisy(&quiet, 1.0, 0xdead_beef_cafe_f00d);
        let got = heard(&stream, rate);
        assert_eq!(got.len(), 0, "{} beacons out of ten minutes of noise", got.len());
    }
}
