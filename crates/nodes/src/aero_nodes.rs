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
pub use decode::inmarsat::{CARRIER_RATIO, OQPSK, config};
use dsp::msk::MskDemod;
use dsp::qpsk::QpskDemod;
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::aero::Aero;
pub use identify::aero::CHANNEL_WIDTH_HZ;
pub use identify::aero::DEFAULT_HZ;
pub use identify::aero::FEED_HZ;
pub use identify::aero::WIDE_CHANNEL_HZ;
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
    demod: Demod,
    framer: aero::Framer,
    assembler: aero::Assembler,
    mixed: Vec<common::C32>,
    narrow: Vec<common::C32>,
    soft: Vec<f32>,
    frames: Vec<aero::Frame>,
    meter: crate::FrameMeter,
    units: u64,
    messages: u64,
}

enum Demod {
    Msk {
        /// Puts the channel on an audio carrier, where the demodulator wants it.
        upmix: Mixer,
        msk: MskDemod,
        shifted: Vec<common::C32>,
        audio: Vec<f32>,
        bits: Vec<bool>,
    },
    Oqpsk {
        resample: Option<Rational>,
        qpsk: Box<QpskDemod>,
        at_rate: Vec<common::C32>,
        symbols: Vec<common::C32>,
    },
}

impl Demod {
    fn new(rate: aero::Rate, work_hz: f64) -> Result<Self> {
        Ok(match rate {
            aero::Rate::P600 | aero::Rate::P1200 => Demod::Msk {
                upmix: Mixer::new(rate.baud() * CARRIER_RATIO, work_hz),
                msk: MskDemod::new(work_hz, config(rate)),
                shifted: Vec::new(),
                audio: Vec::new(),
                bits: Vec::new(),
            },
            aero::Rate::P10500 => {
                let resample = match dsp::resample::stage(work_hz, OQPSK.rate(), 4096) {
                    Some((1, resample)) => resample,
                    _ => return Err(common::Error::other("aero 10500 needs 21 kS/s or more")),
                };
                Demod::Oqpsk {
                    resample,
                    qpsk: Box::new(QpskDemod::new(OQPSK)),
                    at_rate: Vec::new(),
                    symbols: Vec::new(),
                }
            }
        })
    }

    fn work_hz(rate: aero::Rate, stream_hz: f64) -> f64 {
        match rate {
            aero::Rate::P600 | aero::Rate::P1200 => {
                stream_hz / (stream_hz / WORK_HZ).round().max(1.0)
            }
            aero::Rate::P10500 => stream_hz / (stream_hz / OQPSK.rate()).floor().max(1.0),
        }
    }

    fn modulation(rate: aero::Rate) -> common::Modulation {
        match rate {
            aero::Rate::P600 | aero::Rate::P1200 => common::Modulation::Msk,
            aero::Rate::P10500 => common::Modulation::Psk4,
        }
    }

    fn process(&mut self, narrow: &[common::C32], soft: &mut Vec<f32>) {
        soft.clear();
        match self {
            Demod::Msk { upmix, msk, shifted, audio, bits } => {
                shifted.clear();
                upmix.process(narrow, shifted);
                audio.clear();
                audio.extend(shifted.iter().map(|s| s.re));
                bits.clear();
                msk.process(audio, bits);
                soft.extend(bits.iter().map(|b| if *b { -1.0 } else { 1.0 }));
            }
            Demod::Oqpsk { resample, qpsk, at_rate, symbols } => {
                let feed = match resample {
                    Some(r) => {
                        at_rate.clear();
                        r.process(narrow, at_rate);
                        &at_rate[..]
                    }
                    None => narrow,
                };
                symbols.clear();
                qpsk.process(feed, symbols);
                soft.extend(symbols.iter().flat_map(|s| [s.re, s.im]));
            }
        }
    }

    fn reset(&mut self) {
        match self {
            Demod::Msk { upmix, msk, .. } => {
                upmix.reset();
                msk.reset();
            }
            Demod::Oqpsk { resample, qpsk, .. } => {
                if let Some(r) = resample {
                    r.reset();
                }
                qpsk.reset();
            }
        }
    }
}

impl Default for AeroNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ, aero::Rate::P1200)
    }
}

impl AeroNode {
    pub fn new(channel_hz: f64, rate: aero::Rate) -> Self {
        let work = Demod::work_hz(rate, FEED_HZ);
        Self {
            channel_hz,
            rate,
            // All replaced at negotiation, when the real rate is known.
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(FEED_HZ, 1, width_of(rate) / 2.0, 60.0),
            demod: Demod::new(rate, work).expect("the feed rate reads every rate"),
            framer: aero::Framer::new(rate),
            assembler: aero::Assembler::new(),
            mixed: Vec::new(),
            narrow: Vec::new(),
            soft: Vec::new(),
            frames: Vec::new(),
            meter: crate::FrameMeter::new(work, channel_hz as u64, 10.0)
                .keyed_as(Demod::modulation(rate)),
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

fn width_of(rate: aero::Rate) -> f64 {
    match rate {
        aero::Rate::P600 | aero::Rate::P1200 => CHANNEL_WIDTH_HZ,
        aero::Rate::P10500 => WIDE_CHANNEL_HZ,
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
        let width = width_of(self.rate);
        if (self.channel_hz - center).abs() > rate / 2.0 - width / 2.0 {
            return Err(common::Error::other("aero needs its channel inside the span"));
        }
        let work = Demod::work_hz(self.rate, rate);
        let factor = (rate / work).round().max(1.0) as usize;
        self.demod = Demod::new(self.rate, work)?;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, width / 2.0, 60.0);
        self.framer.reset();
        self.assembler.reset();
        self.meter = crate::FrameMeter::new(work, self.channel_hz as u64, 10.0)
            .keyed_as(Demod::modulation(self.rate));

        let mut out = i.spec.with_kind(PortKind::Packets);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = width;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.meter.feed(&self.narrow);

        self.demod.process(&self.narrow, &mut self.soft);
        self.frames.clear();
        self.framer.process_soft(&self.soft, &mut self.frames);

        let out = o.packets_mut();
        for f in &self.frames {
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
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.demod.reset();
        self.framer.reset();
        self.assembler.reset();
        self.meter.reset();
    }
}

impl Protocol for Aero {
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
    fn widths_for(&self, _hz: f64, _source_width_hz: f64) -> Vec<f64> {
        vec![CHANNEL_WIDTH_HZ, WIDE_CHANNEL_HZ]
    }

    fn dedupe_key(&self, p: &common::packet::Packet) -> Option<Vec<u8>> {
        let bytes = p.bytes();
        let user = matches!(
            aero::SuType::of(*bytes.first()?),
            aero::SuType::UserDataInitial | aero::SuType::UserDataSubsequent
        );
        (bytes.len() == aero::SU_BYTES && !user).then(|| [b"aero".as_slice(), bytes].concat())
    }

    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![
            NodeSpec::new(DESC.name)
                .f(CHANNEL_HZ, at.center_hz)
                .f(RATE_BPS, rate_for_width(at.width_hz).baud()),
        ]
    }
}

fn rate_for_width(width_hz: f64) -> aero::Rate {
    match width_hz < (CHANNEL_WIDTH_HZ * WIDE_CHANNEL_HZ).sqrt() {
        true => aero::Rate::P1200,
        false => aero::Rate::P10500,
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "aero",
    summary: "One Inmarsat Aero P channel: 600 or 1200 bps MSK or 10500 bps OQPSK, satellite ACARS",
    category: Category::Decode,
    feeds_bus: true,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let rate = match s.f64_or(RATE_BPS, 1200.0) as u32 {
        600 => aero::Rate::P600,
        10500 => aero::Rate::P10500,
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
        let user = aero::acars_user_data(block);
        let d = aero_read(&user).expect("a row");
        assert_eq!(d.id, "aero-acars");
        // The aircraft either way: an uplink is addressed to the aeroplane,
        // and the block names it by its registration and its flight.
        let who = d.subject.as_ref().expect("the aircraft");
        assert_eq!(who.id.to_string(), "EI-DEO");
        assert_eq!(who.name.as_deref(), Some("EIN123"));
    }

    /// A frame keyed on the channel and read back off it: the whole node,
    /// from complex baseband through the audio carrier the demodulator wants
    /// to the signal units, which is what the recording of a P channel would
    /// otherwise be the only evidence of.
    ///
    /// Six units a frame, five of them the halves of one satellite ACARS
    /// block, so the assembled message is the sixth row.
    fn a_keyed_frame(rate: aero::Rate) -> (Vec<Vec<u8>>, u64, u64) {
        a_keyed_frame_off(rate, 0.0, 1)
    }

    fn a_keyed_frame_off(
        rate: aero::Rate,
        off_hz: f64,
        idle_frames: usize,
    ) -> (Vec<Vec<u8>>, u64, u64) {
        let block = b"2.EI-DEO\x15Q01\x02S01AEIN123ENGINE OK\x03";
        let user = aero::acars_user_data(block);
        let rest = user.len() - 2;
        let follow = rest.div_ceil(8) as u8;
        let last = rest - (follow as usize - 1) * 8;
        let mut sus: Vec<[u8; aero::SU_BYTES]> = vec![aero::su_with_crc(&[
            0x71,
            0x40,
            0x62,
            0x1A,
            0x2A,
            0x35,
            follow,
            (last as u8) << 4,
            user[0],
            user[1],
        ])];
        for k in 0..follow as usize {
            let from = 2 + k * 8;
            let take = if k + 1 == follow as usize { last } else { 8 };
            let mut ssu = vec![0xC0 | (follow - 1 - k as u8), 0x35];
            ssu.extend_from_slice(&user[from..from + take]);
            ssu.resize(10, 0);
            sus.push(aero::su_with_crc(&ssu));
        }
        let fill = aero::su_with_crc(&[0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        let per_frame = rate.coded_bits() / 2 / 8 / aero::SU_BYTES;
        sus.resize(per_frame, fill);
        let idle = vec![fill; per_frame];
        let mut frames: Vec<(u16, &[[u8; aero::SU_BYTES]])> = vec![(0x1000, &idle); idle_frames];
        frames.push((0x1234, &sus));
        let keyed = aero::encode_frames(rate, &frames);
        let iq = match rate {
            aero::Rate::P600 | aero::Rate::P1200 => {
                let mut bits: Vec<bool> = vec![false; 32];
                bits.extend(keyed.iter().map(|b| *b == 1));
                bits.extend(std::iter::repeat_n(false, 32));
                dsp::msk::modulate(&bits, FEED_HZ, config(rate), 0.0, 0.5)
            }
            aero::Rate::P10500 => {
                let mut bits = vec![0u8; 64];
                bits.extend(&keyed);
                bits.extend(std::iter::repeat_n(0, 256));
                let at_rate = dsp::qpsk::modulate(&bits, OQPSK);
                let mut iq = Vec::new();
                Rational::new(OQPSK.rate(), FEED_HZ, 4096).unwrap().process(&at_rate, &mut iq);
                iq
            }
        };

        let mut mixed = Vec::new();
        Mixer::new(-off_hz, FEED_HZ).process(&iq, &mut mixed);
        let mut n = AeroNode::new(DEFAULT_HZ, rate);
        let got = read(&mut n, FEED_HZ, &mixed);
        (got, n.units(), n.messages())
    }

    #[test]
    fn a_frame_keyed_on_the_channel_is_read_back() {
        for rate in [aero::Rate::P600, aero::Rate::P1200, aero::Rate::P10500] {
            let (rows, units, messages) = a_keyed_frame(rate);
            assert_eq!(units, 6, "{rate:?}: signal units whose CRC agreed");
            assert_eq!(messages, 1, "{rate:?}: messages assembled");
            assert_eq!(rows.len(), 7, "{rate:?}: six units and the message they carry");
            let who = aero_read(rows.last().expect("the message"))
                .expect("a row")
                .subject
                .expect("the aircraft");
            assert_eq!(who.id.to_string(), "EI-DEO");
            assert_eq!(who.name.as_deref(), Some("EIN123"));
        }
    }

    #[test]
    fn a_source_is_tried_at_both_widths_and_each_reads_at_its_own_rate() {
        assert_eq!(Aero.widths_for(DEFAULT_HZ, 3_000.0), [CHANNEL_WIDTH_HZ, WIDE_CHANNEL_HZ]);
        let rate_at = |width_hz| {
            let at = Placed {
                center_hz: DEFAULT_HZ,
                width_hz,
                rate: FEED_HZ,
                snr_db: 20.0,
                origin: None,
            };
            Aero.chain(at)[0].settings.f64_or(RATE_BPS, 0.0)
        };
        assert_eq!(rate_at(CHANNEL_WIDTH_HZ), 1200.0);
        assert_eq!(rate_at(WIDE_CHANNEL_HZ), 10500.0);
    }

    #[test]
    fn the_network_talking_about_itself_is_logged_once_and_user_data_every_time() {
        let packet = |bytes: Vec<u8>| {
            crate::FrameMeter::new(WORK_HZ, DEFAULT_HZ as u64, 1.0).packet_now(bytes)
        };
        let key =
            |data: &[u8]| Aero.dedupe_key(&packet(aero::su_with_crc(data).to_vec())).is_some();
        assert!(key(&[0x0A, 0x3D, 0, 0, 0, 0, 0, 0, 0, 0]), "a system table");
        assert!(key(&[0x26, 0x01, 0, 0, 0, 0, 0, 0, 0, 0]), "the 10500 channel's padding");
        assert!(!key(&[0x71, 0xA6, 0xB5, 0x93, 0x44, 0x72, 0x03, 0x10, 0xFF, 0xFF]), "user data");
        assert!(
            !key(&[0xC2, 0x72, 0x01, 0x32, 0xAE, 0xCE, 0xB5, 0xB3, 0x31, 0x51]),
            "its continuation"
        );
        let message = aero::acars_user_data(b"2.N531QS9_\x7fR\x03");
        assert!(Aero.dedupe_key(&packet(message)).is_none(), "a message");
    }

    #[test]
    fn a_10500_channel_pulls_in_500_hz_off_in_five_seconds_and_not_700() {
        let read = |off_hz, idle_frames| {
            let (_, units, messages) = a_keyed_frame_off(aero::Rate::P10500, off_hz, idle_frames);
            (units, messages)
        };
        assert_eq!(read(100.0, 1), (6, 1), "100 Hz off after one frame");
        assert_eq!(read(300.0, 1), (0, 0), "300 Hz off after one frame");
        assert_eq!(read(300.0, 4), (6, 1), "300 Hz off after four frames");
        assert_eq!(read(500.0, 10), (6, 1), "500 Hz off after ten frames");
        assert_eq!(read(700.0, 10), (0, 0), "700 Hz off, past an eighth of the symbol rate");
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
        for speed in [aero::Rate::P600, aero::Rate::P1200, aero::Rate::P10500] {
            let mut n = AeroNode::new(DEFAULT_HZ, speed);
            assert_eq!(read(&mut n, rate, &iq).len(), 0, "{speed:?}");
            assert_eq!((n.units(), n.messages()), (0, 0), "{speed:?}");
        }
    }
}
