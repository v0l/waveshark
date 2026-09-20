//! Inmarsat Classic Aero as a graph node: one P channel in, signal units
//! out, and the ACARS an aircraft is being sent.
//!
//! The waveform is [`dsp::msk`], the same demodulator ACARS uses on a VHF
//! airband channel, at 600 or 1200 bits a second instead of 2400: the
//! channel is mixed down, filtered, put on an audio carrier three quarters
//! of the bit rate up and read as real audio, because that is the form the
//! demodulator takes. The frames are [`decode::inmarsat::aero`].
//!
//! Two kinds of thing reach the bus from here, and they are told apart by
//! length: a signal unit is twelve bytes, and assembled user data is longer
//! and opens with the two 0xFF bytes and the start of header that satellite
//! ACARS is wrapped in. Only units whose CRC agreed are sent on, and fill is
//! dropped, so a row is something that was received.

use crate::NodeSpec;
use crate::protocol::{FrameClaim, Placed, Placement, Protocol, Shape};
use common::Result;
pub use decode::inmarsat::aero_read;
use decode::inmarsat::{BAND_HZ, aero};
pub use decode::inmarsat::{CARRIER_RATIO, config};
use dsp::msk::MskDemod;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::aero::Aero;
pub use identify::aero::CHANNEL_WIDTH_HZ;
pub use identify::aero::DEFAULT_HZ;
pub use identify::aero::FEED_HZ;
pub use identify::aero::WORK_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// Bits a second, which is what tells one P channel from another.
const RATE_BPS: &str = "rate_bps";
/// The carrier this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub struct AeroNode {
    channel_hz: f64,
    rate: aero::Rate,
    mixer: Mixer,
    decim: FirDecim,
    /// Puts the channel on an audio carrier, where the demodulator wants it.
    upmix: Mixer,
    msk: MskDemod,
    framer: aero::Framer,
    assembler: aero::Assembler,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    shifted: Vec<common::C32>,
    audio: Vec<f32>,
    bits: Vec<bool>,
    hard: Vec<u8>,
    frames: Vec<aero::Frame>,
    meter: crate::FrameMeter,
    units: u64,
    messages: u64,
}

impl Default for AeroNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, aero::Rate::P1200)
    }
}

impl AeroNode {
    pub fn new(channel_hz: f64, rate: aero::Rate) -> Self {
        Self {
            channel_hz,
            rate,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(WORK_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            upmix: Mixer::new(-rate.baud() * CARRIER_RATIO, WORK_HZ),
            msk: MskDemod::new(WORK_HZ, config(rate)),
            framer: aero::Framer::new(rate),
            assembler: aero::Assembler::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            shifted: Vec::new(),
            audio: Vec::new(),
            bits: Vec::new(),
            hard: Vec::new(),
            frames: Vec::new(),
            meter: crate::FrameMeter::new(WORK_HZ, channel_hz as u64, 10.0)
                .keyed_as(common::Modulation::Msk),
            units: 0,
            messages: 0,
        }
    }

    /// Signal units whose CRC agreed since the node was built.
    pub fn units(&self) -> u64 {
        self.units
    }

    /// User data messages assembled out of them.
    pub fn messages(&self) -> u64 {
        self.messages
    }
}

impl Simple for AeroNode {
    fn name(&self) -> &str {
        "aero"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("aero reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("aero needs its channel inside the span"));
        }
        let factor = (rate / WORK_HZ).round().max(1.0) as usize;
        let work = rate / factor as f64;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.upmix = Mixer::new(-self.rate.baud() * CARRIER_RATIO, work);
        self.msk = MskDemod::new(work, config(self.rate));
        self.framer.reset();
        self.assembler.reset();
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 10.0)
            .keyed_as(common::Modulation::Msk);

        let mut out = i.spec.with_kind(PortKind::Packets);
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
        self.meter.feed(&self.narrow);

        self.shifted.clear();
        self.upmix.process(&self.narrow, &mut self.shifted);
        self.audio.clear();
        self.audio.extend(self.shifted.iter().map(|s| s.re));

        self.bits.clear();
        let mut bits = std::mem::take(&mut self.bits);
        self.msk.process(&self.audio, &mut bits);
        self.hard.clear();
        self.hard.extend(bits.iter().map(|b| u8::from(*b)));
        self.bits = bits;

        self.frames.clear();
        let mut frames = std::mem::take(&mut self.frames);
        self.framer.process(&self.hard, &mut frames);

        let out = o.packets_mut();
        for f in &frames {
            for su in &f.sus {
                if !su.crc_ok || su.kind() == aero::SuType::Fill {
                    continue;
                }
                self.units += 1;
                out.push(self.meter.packet_now(su.bytes.to_vec()));
                if let Some(user) = self.assembler.update(su.data()) {
                    self.messages += 1;
                    out.push(self.meter.packet_now(user.bytes));
                }
            }
        }
        self.frames = frames;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.upmix.reset();
        self.msk.reset();
        self.framer.reset();
        self.assembler.reset();
        self.meter.reset();
    }
}

impl Protocol for Aero {
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

    /// The L-band downlinks to mobiles. The aeronautical channels sit in the
    /// top of the band, but which ones are keyed depends on the beam, so the
    /// band is the claim.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} AERO", hz / 1e6)
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND_HZ.1 - 1_545_000_000.0) as u64 }
    }
    fn stated(&self, p: &common::packet::Packet) -> Option<Vec<common::packet::Proto>> {
        let bytes = p.bytes();
        let hz = p.center_hz() as f64;
        if !(1_545_000_000.0..BAND_HZ.1).contains(&hz) {
            return None;
        }
        Some(aero_read(bytes).into_iter().collect())
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "aero",
    summary: "One Inmarsat Aero P channel: 600 or 1200 bps MSK, satellite ACARS",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let rate = match s.f64_or(RATE_BPS, 1200.0) as u32 {
        600 => aero::Rate::P600,
        _ => aero::Rate::P1200,
    };
    Ok(Box::new(AeroNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ), rate)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};

    fn read(n: &mut AeroNode, rate: f64, iq: &[C32]) -> Vec<Vec<u8>> {
        let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(DEFAULT_HZ as u64)), latency: 0 };
        n.negotiate(&spec).expect("a channel");
        let ins = [spec];
        let tags = Vec::new();
        let mut got = Vec::new();
        for chunk in iq.chunks(16_384) {
            let mut out = Payload::Packets(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            n.process(&Payload::Iq(chunk.to_vec()), &mut out, &mut ctx).expect("read");
            if let Payload::Packets(f) = out {
                got.extend(f.into_iter().map(|x| x.bytes().to_vec()));
            }
        }
        got
    }

    #[test]
    fn the_channel_has_to_be_inside_the_span() {
        let mut n = AeroNode::default();
        let far = PortSpec { spec: StreamSpec::iq(200_000.0, Hz(1_530_000_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let near = PortSpec { spec: StreamSpec::iq(200_000.0, Hz(1_545_000_000)), latency: 0 };
        let out = n.negotiate(&near).expect("a channel in the span");
        assert_eq!(out.kind, PortKind::Packets);
        assert_eq!(out.center, Hz(DEFAULT_HZ as u64));
    }

    /// A channel assignment, as a row: the satellite talking about itself,
    /// so it carries its fields and nobody wrote it.
    #[test]
    fn a_signal_unit_becomes_a_row() {
        let su = aero::su_with_crc(&[0x34, 0x40, 0x62, 0x1A, 0x2A, 0x00, 0x11, 0x22, 0x33, 0x44]);
        let d = aero_read(&su).expect("a row");
        assert_eq!((d.id, d.kind), ("aero", "channel_assignment"));
        assert!(d.wrote().is_none(), "the satellite is talking about itself");
    }

    /// Assembled user data is an ACARS block, and reads as one: the same
    /// ARINC 618 fields a VHF channel carries, with the aircraft named.
    #[test]
    fn assembled_user_data_reads_as_acars() {
        let block = b"2.EI-DEO\x15Q01\x02S01AEIN123ENGINE OK\x03";
        let mut user: Vec<u8> = vec![0xFF, 0xFF, 0x01];
        user.extend(block.iter().copied());
        let d = aero_read(&user).expect("a row");
        assert_eq!(d.id, "aero-acars");
        // The aircraft either way: an uplink is addressed to the aeroplane,
        // and the block names it by its registration and its flight.
        let who = d.subject.as_ref().expect("the aircraft");
        assert_eq!(who.id.to_string(), "EI-DEO");
        assert_eq!(who.name.as_deref(), Some("EIN123"));
    }

    /// Ten minutes of noise on the channel, and nothing reaches the bus:
    /// the unique word comes up by chance in that many bits, and no signal
    /// unit behind it ever checks.
    #[test]
    fn noise_produces_no_units() {
        let rate = 38_400.0;
        let mut s = 17u64;
        let iq: Vec<C32> = (0..(rate as usize * 600))
            .map(|_| {
                let mut next = || {
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (s >> 33) as f32 / (1u64 << 30) as f32 - 1.0
                };
                C32::new(next(), next())
            })
            .collect();
        let mut n = AeroNode::default();
        assert_eq!(read(&mut n, rate, &iq).len(), 0);
        assert_eq!(n.units(), 0);
        assert_eq!(n.messages(), 0);
    }
}
