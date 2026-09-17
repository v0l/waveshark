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
use decode::inmarsat::{BAND_HZ, aero};
use dsp::msk::{MskConfig, MskDemod};
use dsp::{FirDecim, Mixer};
use pipeline::event::{Decoded, media};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// One of the aeronautical P channels. Which ones a satellite keys depends
/// on the beam, so this is only what the node is built with before the
/// scanner table tells it otherwise.
pub const DEFAULT_HZ: f64 = 1_545_025_000.0;

/// What one low rate channel occupies: 1200 bits a second of MSK is about
/// 1.8 kHz, and the channels are on a 5 kHz grid.
pub const CHANNEL_WIDTH_HZ: f64 = 5_000.0;

/// The audio rate the channel is read at: eight samples a bit at 1200,
/// sixteen at 600.
const WORK_HZ: f64 = 9_600.0;

/// The rate to ask the receiver for, which decimates to [`WORK_HZ`] by four.
const FEED_HZ: f64 = 38_400.0;

/// Where the two tones are centred, as a fraction of the bit rate.
///
/// The demodulator counts its bit clock in turns of this carrier, and the
/// one ratio it is known to read is ACARS's 1800 Hz against 2400 baud.
const CARRIER_RATIO: f64 = 0.75;

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
            meter: crate::FrameMeter::new(WORK_HZ, channel_hz as u64, 10.0),
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

fn config(rate: aero::Rate) -> MskConfig {
    MskConfig { baud: rate.baud(), carrier_hz: rate.baud() * CARRIER_RATIO }
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
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 10.0);

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

        let out = o.frames_mut();
        for f in &frames {
            for su in &f.sus {
                if !su.crc_ok || su.kind() == aero::SuType::Fill {
                    continue;
                }
                self.units += 1;
                out.push(self.meter.frame(su.bytes.to_vec()));
                if let Some(user) = self.assembler.update(su.data()) {
                    self.messages += 1;
                    out.push(self.meter.frame(user.bytes));
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

/// The row a signal unit or an assembled message becomes.
///
/// A signal unit is the satellite talking about itself: a channel
/// assignment, a log on acknowledgement, a table of frequencies. An
/// assembled message is ACARS, which is an aircraft's computer and a ground
/// station's, so neither is `written` and both carry their fields.
pub fn aero_decoded(bytes: &[u8], center: common::Hz) -> Option<Decoded> {
    if let Some(block) = aero::acars_block(bytes) {
        let m = decode::acars::parse(block)?;
        let mut d = crate::acars_nodes::acars_decoded(&m, block, center);
        d.protocol = "Aero-ACARS";
        return Some(d.with_modulation(common::Modulation::Msk));
    }
    if bytes.len() != aero::SU_BYTES {
        return None;
    }
    let kind = aero::SuType::of(bytes[0]);
    let mut fields: Vec<(String, common::Value)> = vec![
        ("unit".into(), common::Value::Text(kind.label().into())),
        ("type".into(), common::Value::Text(format!("{:02X}", bytes[0]))),
    ];
    let mut who = common::Identity::new("aero", "ges");
    // A user data unit names the aircraft and the ground station; the rest
    // of the units are the network's own business.
    if kind == aero::SuType::UserDataInitial {
        let aes = u32::from_be_bytes([0, bytes[1], bytes[2], bytes[3]]);
        fields.push(("aes".into(), common::Value::Text(format!("{aes:06X}"))));
        fields.push(("ges".into(), common::Value::Int(i64::from(bytes[4]))));
        who = common::Identity::new("icao", format!("{aes:06X}"));
    }
    Some(
        Decoded::bytes("Aero", center, 0.0, bytes.to_vec())
            .with_modulation(common::Modulation::Msk)
            .with_crc(Some(aero::su_crc_ok(bytes)))
            .with_detail(kind.label().to_string())
            .with_media(media::BYTES)
            .with_fields(fields)
            .by(who),
    )
}

pub struct Aero;

impl Protocol for Aero {
    fn id(&self) -> &'static str {
        "aero"
    }
    fn label(&self) -> &'static str {
        "aero"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["satcom", "aero-l", "satacars"]
    }
    /// The L-band downlinks to mobiles. The aeronautical channels sit in the
    /// top of the band, but which ones are keyed depends on the beam, so the
    /// band is the claim.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![(1_545_000_000.0, BAND_HZ.1)])
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: WORK_HZ,
            feed_rate_hz: FEED_HZ,
            span_wide: false,
            families: &[],
        }
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} AERO", hz / 1e6)
    }
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Band { width_hz: (BAND_HZ.1 - 1_545_000_000.0) as u64 }
    }
    fn read_frame(&self, p: &common::Packet, bytes: &[u8]) -> Option<Vec<Decoded>> {
        let hz = p.center_hz() as f64;
        if !(1_545_000_000.0..BAND_HZ.1).contains(&hz) {
            return None;
        }
        Some(aero_decoded(bytes, common::Hz(p.center_hz())).into_iter().collect())
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
            let mut out = Payload::Frames(Vec::new());
            let (mut events, mut new_tags) = (Vec::new(), Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            n.process(&Payload::Iq(chunk.to_vec()), &mut out, &mut ctx).expect("read");
            if let Payload::Frames(f) = out {
                got.extend(f.into_iter().map(|x| x.bytes));
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
        assert_eq!(out.kind, PortKind::Frames);
        assert_eq!(out.center, Hz(DEFAULT_HZ as u64));
    }

    /// A channel assignment, as a row: the satellite talking about itself,
    /// so it carries its fields and nobody wrote it.
    #[test]
    fn a_signal_unit_becomes_a_row() {
        let su = aero::su_with_crc(&[0x34, 0x40, 0x62, 0x1A, 0x2A, 0x00, 0x11, 0x22, 0x33, 0x44]);
        let d = aero_decoded(&su, Hz(DEFAULT_HZ as u64)).expect("a row");
        assert_eq!(d.protocol, "Aero");
        assert_eq!(d.crc_ok, Some(true));
        assert!(!d.written);
        assert_eq!(d.detail.as_deref(), Some("channel assignment"));
    }

    /// Assembled user data is an ACARS block, and reads as one: the same
    /// ARINC 618 fields a VHF channel carries, with the aircraft named.
    #[test]
    fn assembled_user_data_reads_as_acars() {
        let block = b"2.EI-DEO\x15Q01\x02S01AEIN123ENGINE OK\x03";
        let mut user: Vec<u8> = vec![0xFF, 0xFF, 0x01];
        user.extend(block.iter().copied());
        let d = aero_decoded(&user, Hz(DEFAULT_HZ as u64)).expect("a row");
        assert_eq!(d.protocol, "Aero-ACARS");
        let detail = d.detail.clone().unwrap_or_default();
        assert!(detail.contains("registration=EI-DEO"), "{detail}");
        assert!(detail.contains("flight=EIN123"), "{detail}");
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
