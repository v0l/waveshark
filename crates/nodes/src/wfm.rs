//! Broadcast FM as a single node: stereo audio plus RDS.
//!
//! The multiplex carries audio, a 19 kHz pilot, the difference signal on 38 kHz
//! and RDS on 57 kHz, and all of them come off one PLL. Splitting this across
//! separate nodes would mean either running three PLLs on the same pilot or
//! inventing a port type to pass a phase array between them, so it stays one
//! node with the discriminator, stereo decoder and RDS chain inside.
//!
//! The audio port is always two interleaved channels. Mono is the blend
//! reaching zero, not a different output format: changing the channel count
//! mid-stream would mean reopening the audio device every time reception
//! wobbled.

use common::Result;
use dsp::NoiseMeter;
use dsp::rds::{BlockSync, GroupDecoder, RdsDemod};
use dsp::{FmDemod, StereoDecoder};
use pipeline::event::Event;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec, Tag, TagValue};
use pipeline::registry::{Category, Settings, StageDesc};

/// Peak deviation of broadcast FM.
const DEVIATION_HZ: f64 = 75_000.0;

/// Twice the subcarrier and its sidebands, which is the slowest stream the
/// multiplex can be built at: below this the 57 kHz data folds back over the
/// audio.
pub const RDS_TX_MIN_RATE_HZ: f64 = 130_000.0;

/// The narrowest span a station can be transmitted in: Carson's rule on
/// 75 kHz of deviation with the data sidebands at the top of the multiplex.
/// Below this the modulator refuses the multiplex, so a chain drawn here
/// leaves the station out rather than losing the transmitter with it.
pub const RDS_STATION_MIN_RATE_HZ: f64 =
    2.0 * (DEVIATION_HZ + dsp::rds::demod::CARRIER_HZ + dsp::rds::demod::BAUD);

/// The top of a broadcast programme. Above this the audio walks into the
/// 19 kHz pilot, and a pilot with programme on it is a station a receiver
/// hears as mono with no data.
const PROGRAMME_HZ: f64 = 15_000.0;

/// Taps for that cut, which is a multiply each on every sample of the
/// multiplex. Measured on the designed response at 320 kS/s: 63 taps has
/// already taken 0.9 dB off 10 kHz and leaves the pilot only 16 dB down,
/// and 255 buys another 40 dB at 19 kHz on a band with nothing up there.
const PROGRAMME_TAPS: usize = 127;

pub struct WfmDemodNode {
    demod: FmDemod,
    stereo: StereoDecoder,
    noise: NoiseMeter,
    rds: Option<RdsDemod>,
    sync: BlockSync,
    groups: GroupDecoder,
    stereo_enabled: bool,
    rds_enabled: bool,
    mpx: Vec<f32>,
    left: Vec<f32>,
    right: Vec<f32>,
    bits: Vec<u8>,
    was_locked: bool,
    last_text: Option<String>,
    samples: u64,
}

impl Default for WfmDemodNode {
    fn default() -> Self {
        Self::new()
    }
}

impl WfmDemodNode {
    pub fn new() -> Self {
        Self {
            demod: FmDemod::new(1.0, DEVIATION_HZ),
            stereo: StereoDecoder::new(1.0),
            noise: NoiseMeter::new(1.0),
            rds: None,
            sync: BlockSync::new(),
            groups: GroupDecoder::new(),
            stereo_enabled: true,
            rds_enabled: true,
            mpx: Vec::new(),
            left: Vec::new(),
            right: Vec::new(),
            bits: Vec::new(),
            was_locked: false,
            last_text: None,
            samples: 0,
        }
    }

    pub fn mono(mut self) -> Self {
        self.stereo_enabled = false;
        self
    }

    pub fn without_rds(mut self) -> Self {
        self.rds_enabled = false;
        self
    }

    /// Station information accumulated so far.
    /// Groups decoded, blocks rejected, and whether framing is held.
    pub fn rds_stats(&self) -> (u64, u64, bool) {
        (self.sync.groups, self.sync.errors, self.sync.is_synced())
    }

    pub fn station(&self) -> &dsp::rds::Station {
        self.groups.station()
    }

    /// How much stereo separation is currently applied, 0 mono to 1 full.
    pub fn blend(&self) -> f32 {
        self.stereo.blend()
    }

    fn emit_rds(&mut self, c: &mut NodeCtx<'_>) {
        let center = c.inputs[0].spec.center;
        let before = self.groups.station().clone();
        for b in std::mem::take(&mut self.bits) {
            if let Some(g) = self.sync.push(b) {
                self.groups.push(&g);
            }
        }
        let now = self.groups.station();
        if now.name != before.name || now.radiotext != before.radiotext {
            let text = render(now);
            // Only report when the rendering actually changed, or a station
            // repeating its name every 80 ms would flood the event log.
            if self.last_text.as_deref() != Some(text.as_str()) {
                self.last_text = Some(text.clone());
                let mut proto = common::packet::Proto::new("rds", "station");
                if let Some(name) = now.name.clone() {
                    proto = proto.saying(common::packet::Fact::Named(
                        common::packet::Named::new(name, common::packet::ThingKind::Station)
                            .fixed(),
                    ));
                }
                if let Some(rt) = now.radiotext.clone() {
                    proto = proto.saying(common::packet::Fact::Playing(rt));
                }
                let carrier = crate::off_audio(center.0, 0, &[]);
                c.emit(Event::Decoded(
                    common::packet::Packet::heard(carrier)
                        .framed(common::packet::Frame::of(text.into_bytes()))
                        .decoded(proto),
                ));
            }
        }
    }
}

fn render(s: &dsp::rds::Station) -> String {
    let mut out = String::new();
    if let Some(pi) = s.pi {
        out.push_str(&format!("PI={pi:04X}"));
    }
    if let Some(n) = &s.name {
        out.push_str(&format!(" \"{n}\""));
    }
    if let Some(p) = s.pty_name() {
        out.push_str(&format!(" [{p}]"));
    }
    if let Some(rt) = &s.radiotext {
        out.push_str(&format!(" {rt}"));
    }
    out
}

impl Node for WfmDemodNode {
    fn name(&self) -> &str {
        "wfm_demod"
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        1
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("wfm_demod needs an IQ input"));
        }
        let rate = i.spec.rate;
        if rate < 130_000.0 {
            // The pilot is at 19 kHz and RDS at 57 kHz, so anything that does
            // not reach past 57 kHz cannot carry them at all.
            return Err(common::Error::other(format!(
                "wfm_demod needs at least 130 kHz to reach the 57 kHz subcarrier, got {rate:.0}"
            )));
        }
        self.demod = FmDemod::new(rate, DEVIATION_HZ);
        self.stereo = StereoDecoder::new(rate);
        self.noise = NoiseMeter::new(rate);
        self.rds = self.rds_enabled.then(|| RdsDemod::new(rate));
        self.sync = BlockSync::new();
        self.groups.reset();
        // Two interleaved channels. The port's sample rate is twice the frame
        // rate, which the channel count now says outright rather than leaving
        // downstream filters to infer it from a rate that looks too high.
        Ok(vec![i.spec.with_kind(PortKind::Real).with_rate(rate).with_channels(2)])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let iq = inputs[0].as_iq().unwrap();
        self.mpx.clear();
        self.demod.process(iq, &mut self.mpx);

        // Measured here because it needs the discriminator output above the
        // audio band, which no longer exists after decimation. Tagged rather
        // than emitted so it rate-scales down the chain to whatever is
        // listening for it.
        let noise = self.noise.process(&self.mpx);
        c.tag(Tag::new(self.samples * 2, "noise", TagValue::Float(noise as f64)));

        if self.stereo_enabled {
            self.stereo.process(&self.mpx, &mut self.left, &mut self.right);
        } else {
            self.stereo.process_mono(&self.mpx, &mut self.left);
            self.right.clear();
            self.right.extend_from_slice(&self.left);
        }

        let out = outputs[0].real_mut();
        out.reserve(self.left.len() * 2);
        for (l, r) in self.left.iter().zip(&self.right) {
            out.push(*l);
            out.push(*r);
        }

        if self.rds_enabled
            && let Some(rds) = &mut self.rds
        {
            self.bits.clear();
            rds.process(&self.mpx, self.stereo.phases(), &mut self.bits);
            self.emit_rds(c);
        }

        let locked = self.stereo.is_locked();
        if locked != self.was_locked {
            // Tag the exact sample, so anything downstream knows where the
            // transition landed rather than only that it happened.
            c.tag(Tag::new(self.samples * 2, "stereo_lock", TagValue::Int(locked as i64)));
            self.was_locked = locked;
        }
        self.samples += self.left.len() as u64;
        Ok(())
    }

    fn reset(&mut self) {
        self.demod.reset();
        self.stereo.reset();
        if let Some(r) = &mut self.rds {
            r.reset();
        }
        self.sync = BlockSync::new();
        self.groups.reset();
        self.was_locked = false;
        self.last_text = None;
        self.samples = 0;
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("stereo", self.stereo_enabled).label("Stereo"),
            Param::bool("rds", self.rds_enabled).label("RDS"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "stereo" => {
                self.stereo_enabled = v.as_bool().unwrap_or(true);
                Ok(())
            }
            "rds" => {
                self.rds_enabled = v.as_bool().unwrap_or(true);
                Ok(())
            }
            _ => Err(common::Error::other(format!("wfm_demod: unknown parameter {name:?}"))),
        }
    }
}

/// The multiplex a broadcast station transmits: a pilot, the data
/// subcarrier, and whatever audio is underneath.
///
/// The mirror of [`WfmDemodNode`]'s RDS half: `dsp::rds::tx` builds the
/// groups, the blocks and the differential Manchester, and this paces them
/// against the clock and hands the multiplex to the FM modulator. Audio is
/// the port it takes rather than a file, so a tone, a microphone or a mixer
/// output can be the programme; with nothing connected the station carries
/// its identity and silence.
///
/// The programme arrives on the second input at the stream's own rate, which
/// is what `mic` resamples to, and is folded to mono and cut off at 15 kHz
/// here: the pilot is at 19 kHz and anything the programme puts near it is
/// heard as a station with no pilot at all.
///
/// Mono only. The difference signal on 38 kHz is the other half of a stereo
/// multiplex and needs a second channel to carry, which this port does not
/// have.
pub struct RdsTxNode {
    station: dsp::rds::tx::Station,
    /// The bit stream of one round of groups, repeated.
    bits: Vec<bool>,
    /// The multiplex, which keeps the pilot and the subcarrier running
    /// across blocks and across repeats of the groups.
    mpx: Option<dsp::rds::tx::Multiplex>,
    rate: f64,
    /// The programme, folded to mono and band limited, block by block.
    programme: Vec<f32>,
    /// What keeps the programme out of the pilot.
    band: Option<crate::RealFir>,
    channels: usize,
}

impl Default for RdsTxNode {
    fn default() -> Self {
        Self {
            station: dsp::rds::tx::Station::default(),
            bits: Vec::new(),
            mpx: None,
            rate: 0.0,
            programme: Vec::new(),
            band: None,
            channels: 0,
        }
    }
}

impl RdsTxNode {
    pub fn new(pi: u16, name: &str, radiotext: &str) -> Self {
        let mut n = Self::default();
        n.station.pi = pi;
        n.station.name = name.into();
        n.station.radiotext = radiotext.into();
        n
    }

    /// Build one round of groups as a multiplex at the negotiated rate. A
    /// round rather than a group: a receiver only shows a name once all four
    /// of its segments have arrived.
    fn build_round(&mut self) {
        self.bits = dsp::rds::tx::bits(&self.station.groups());
        self.mpx = (self.rate > 0.0).then(|| dsp::rds::tx::Multiplex::new(&self.bits, self.rate));
    }

    /// Groups in one round, which is four for the name and one per four
    /// characters of radiotext.
    pub fn groups(&self) -> usize {
        self.bits.len() / 104
    }
}

impl Node for RdsTxNode {
    fn name(&self) -> &str {
        RDS_TX.name
    }

    /// The clock, and the programme to carry under the pilot.
    fn num_inputs(&self) -> usize {
        2
    }

    fn num_outputs(&self) -> usize {
        1
    }

    /// A station with nothing on its programme input transmits its identity
    /// over silence, which is what a test transmitter does.
    fn optional_inputs(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = inputs[0];
        if i.spec.rate <= 0.0 {
            return Err(common::Error::other("rds_tx needs a clock to run against"));
        }
        if i.spec.rate < RDS_TX_MIN_RATE_HZ {
            return Err(common::Error::other(format!(
                "rds_tx needs at least {RDS_TX_MIN_RATE_HZ:.0} Hz to carry the 57 kHz \
                 subcarrier, got {:.0}",
                i.spec.rate
            )));
        }
        self.rate = i.spec.rate;
        self.channels = 0;
        self.band = None;
        if let Some(a) = inputs.get(1).filter(|a| !a.spec.is_silence()) {
            if a.spec.kind != PortKind::Real && a.spec.kind != PortKind::Voice {
                return Err(common::Error::other("rds_tx carries audio as its programme"));
            }
            // `fm_mod` takes one real sample per output sample, so the
            // programme has to arrive at the multiplex's rate already; `mic`
            // interpolates to whatever it is negotiated at.
            if (a.spec.rate - i.spec.rate).abs() > 1.0 {
                return Err(common::Error::other(format!(
                    "rds_tx wants its programme at {:.0}, got {:.0}",
                    i.spec.rate, a.spec.rate
                )));
            }
            self.channels = a.spec.channels.max(1);
            self.band = Some(crate::RealFir::new(dsp::fir::lowpass(
                PROGRAMME_TAPS,
                PROGRAMME_HZ / i.spec.rate,
                60.0,
            )));
        }
        self.build_round();
        let mut out = i.spec.with_kind(PortKind::Real).with_channels(1);
        out.flow = pipeline::port::Flow::Tx;
        // The multiplex reaches the top of the data sidebands, which is what
        // the modulator adds to its deviation.
        out.bandwidth = 2.0 * (dsp::rds::demod::CARRIER_HZ + dsp::rds::demod::BAUD);
        Ok(vec![out])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let n = inputs[0].len();
        self.programme.clear();
        if let Some(band) = self.band.as_mut() {
            let channels = self.channels.max(1);
            let audio = inputs.get(1).and_then(|p| p.as_real()).unwrap_or(&[]);
            let scale = 1.0 / channels as f32;
            self.programme.extend(
                audio.chunks_exact(channels).map(|f| f.iter().sum::<f32>() * scale).take(n),
            );
            band.process(&mut self.programme);
        }
        let Some(mpx) = self.mpx.as_mut() else { return Ok(()) };
        // The rounds run back to back with no gap: RDS is continuous on a
        // broadcast station, and what a receiver does at a join is pull its
        // loop back in, which costs it the groups either side.
        mpx.push(&self.programme, n, outputs[0].real_mut());
        Ok(())
    }

    fn reset(&mut self) {
        if let Some(m) = self.mpx.as_mut() {
            m.reset();
        }
        if let Some(b) = self.band.as_mut() {
            b.reset();
        }
    }

    fn readings(&self) -> Vec<(String, String)> {
        vec![
            ("as".into(), self.station.name.clone()),
            ("pi".into(), format!("{:04X}", self.station.pi)),
            ("groups".into(), self.groups().to_string()),
            (
                "programme".into(),
                match self.band.is_some() {
                    true => "connected".into(),
                    false => "silence".into(),
                },
            ),
        ]
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::text("pi", format!("{:04X}", self.station.pi)).label("PI code"),
            Param::text("name", self.station.name.clone()).label("Station"),
            Param::text("radiotext", self.station.radiotext.clone()).label("Radiotext"),
            Param::int("pty", i64::from(self.station.pty), 0..=31).label("Programme type"),
            Param::bool("music", self.station.music).label("Music"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            // Four hex digits, which is how a PI code is published and how
            // every receiver shows one.
            "pi" => {
                let text = v.as_str().unwrap_or_default().trim().to_string();
                self.station.pi = u16::from_str_radix(text.trim_start_matches("0x"), 16)
                    .map_err(|_| common::Error::other("rds_tx: a PI code is four hex digits"))?;
            }
            "name" => self.station.name = v.as_str().unwrap_or_default().to_string(),
            "radiotext" => self.station.radiotext = v.as_str().unwrap_or_default().to_string(),
            "pty" => self.station.pty = v.as_i64().unwrap_or(0).clamp(0, 31) as u8,
            "music" => self.station.music = v.as_bool().unwrap_or(false),
            _ => return Err(common::Error::other(format!("rds_tx: unknown parameter {name:?}"))),
        }
        self.build_round();
        Ok(())
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "wfm_demod",
    summary: "Broadcast FM with stereo and RDS, from a wide IF",
    category: Category::Demod,
    feeds_bus: false,
};

pub const RDS_TX: StageDesc = StageDesc {
    name: "rds_tx",
    summary: "Transmit an RDS multiplex: a station name, a PI code and radiotext",
    category: Category::Transmit,
    feeds_bus: false,
};

pub fn build(_s: &Settings) -> Result<Box<dyn Node>> {
    Ok(Box::new(WfmDemodNode::new()))
}

pub fn build_rds_tx(s: &Settings) -> Result<Box<dyn Node>> {
    use pipeline::registry::SettingsExt;
    let pi = u16::from_str_radix(s.str_or("pi", "5343"), 16).unwrap_or(0x5343);
    Ok(Box::new(RdsTxNode::new(pi, s.str_or("name", "WAVESHRK"), s.str_or("radiotext", ""))))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Simple;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, common::Hz(98_000_000)), latency: 0 }
    }

    /// A station transmitted by this receiver and read back by it: the
    /// groups, the offset words, the differential Manchester and the
    /// subcarrier all agree or the name does not come back.
    ///
    /// The rate is 320 kS/s. A broadcast carrier deviating 75 kHz with data
    /// out at 58 kHz occupies 266 kHz by Carson, so the modulator refuses the
    /// 250 kS/s a tuner reading only the audio would use, and it is right to:
    /// the subcarrier folds back over the multiplex.
    #[test]
    fn a_station_this_receiver_transmitted_is_one_this_receiver_reads() {
        let rate = 320_000.0;
        let mut tx = RdsTxNode::new(0xC479, "WAVESHRK", "A TEST OF THE RDS TRANSMITTER");
        let mpx_spec = tx.negotiate(&[spec(rate)]).unwrap();
        // Four groups spell the name, and the 29 character message takes
        // eight more at four characters a group.
        assert_eq!(tx.groups(), 12);

        let mut modulator = crate::mod_nodes::FmModNode::new(0.0, DEVIATION_HZ, 0.5);
        Simple::negotiate(&mut modulator, &PortSpec { spec: mpx_spec[0], latency: 0 }).unwrap();
        let mut rx = WfmDemodNode::new().mono();
        rx.negotiate(&[spec(rate)]).unwrap();

        let ins = [spec(rate)];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let block = 8192;
        for _ in 0..(4.0 * rate / block as f64) as usize {
            let clock = Payload::Real(vec![0.0; block]);
            let mut mpx = Payload::Real(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            tx.process(&[&clock], std::slice::from_mut(&mut mpx), &mut ctx).unwrap();

            let mut iq = Payload::Iq(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(&mut modulator, &mpx, &mut iq, &mut ctx).unwrap();

            let mut audio = Payload::Real(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            rx.process(&[&iq], std::slice::from_mut(&mut audio), &mut ctx).unwrap();
        }

        let (groups, errors, synced) = rx.rds_stats();
        assert!(synced, "the block synchroniser never framed");
        // Four seconds at 1187.5 bit/s is 45 groups of 104 bits, and 44 of
        // them are framed: the rest go by while the demodulator's timing
        // bank is choosing an arm and the synchroniser is finding the
        // offset words. Every block after that passes its syndrome.
        assert_eq!(groups, 44, "groups in four seconds");
        // Nothing rejected at all. Three measurements got it there: shaping
        // the biphase to the 2.4 kHz the specification allows took 26
        // rejects down to 4, carrying the differential level across the
        // repeat took 4 down to 1, and holding the modulation axis through
        // the branch cut a quadrature subcarrier sits on took 1 down to 0.
        assert_eq!(errors, 0, "blocks rejected");
        assert_eq!(rx.station().pi, Some(0xC479));
        assert_eq!(rx.station().name.as_deref(), Some("WAVESHRK"));
        assert_eq!(rx.station().radiotext.as_deref(), Some("A TEST OF THE RDS TRANSMITTER"));
    }

    /// One frequency's amplitude in a real block, by Goertzel: the audio is
    /// four seconds at 320 kS/s and a whole transform of it says nothing a
    /// handful of bins do not.
    fn amplitude_at(audio: &[f32], hz: f64, rate: f64) -> f32 {
        let w = std::f64::consts::TAU * hz / rate;
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (n, s) in audio.iter().enumerate() {
            let p = w * n as f64;
            re += f64::from(*s) * p.cos();
            im += f64::from(*s) * p.sin();
        }
        (2.0 * (re * re + im * im).sqrt() / audio.len() as f64) as f32
    }

    /// A station with a programme under it, through the modulator and back
    /// out of the receiver's own demodulator.
    fn station_over(programme: &[f32], channels: usize, rate: f64) -> (WfmDemodNode, Vec<f32>) {
        let mut tx = RdsTxNode::new(0xC479, "WAVESHRK", "A TEST OF THE RDS TRANSMITTER");
        let audio_in = PortSpec {
            spec: StreamSpec { kind: PortKind::Real, rate, channels, ..Default::default() },
            latency: 0,
        };
        let mpx_spec = tx.negotiate(&[spec(rate), audio_in]).unwrap();
        let mut modulator = crate::mod_nodes::FmModNode::new(0.0, DEVIATION_HZ, 0.5);
        Simple::negotiate(&mut modulator, &PortSpec { spec: mpx_spec[0], latency: 0 }).unwrap();
        let mut rx = WfmDemodNode::new().mono();
        rx.negotiate(&[spec(rate)]).unwrap();

        let ins = [spec(rate)];
        let (mut ev, mut tg) = (Vec::new(), Vec::new());
        let block = 8192 * channels;
        let mut heard: Vec<f32> = Vec::new();
        for b in 0..(4.0 * rate / 8192.0) as usize {
            let clock = Payload::Real(vec![0.0; 8192]);
            let here = Payload::Real(programme[b * block..(b + 1) * block].to_vec());
            let mut mpx = Payload::Real(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            tx.process(&[&clock, &here], std::slice::from_mut(&mut mpx), &mut ctx).unwrap();

            let mut iq = Payload::Iq(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            Simple::process(&mut modulator, &mpx, &mut iq, &mut ctx).unwrap();

            let mut audio = Payload::Real(Vec::new());
            let mut ctx = NodeCtx::new(0, &ins, &[], &mut ev, &mut tg);
            rx.process(&[&iq], std::slice::from_mut(&mut audio), &mut ctx).unwrap();
            if let Payload::Real(a) = audio {
                heard.extend(a.chunks_exact(2).map(|f| f[0]));
            }
        }
        (rx, heard)
    }

    fn tone(hz: f64, level: f32, rate: f64, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| level * (std::f64::consts::TAU * hz * i as f64 / rate).sin() as f32)
            .collect()
    }

    /// The whole point of the second input: a station that announces a name
    /// and plays nothing is a station nobody would leave running. Both
    /// halves have to survive the same multiplex, so what is pinned is the
    /// tone at the frequency it went in at and the name and the radiotext
    /// still arriving on the subcarrier above it.
    #[test]
    fn a_tone_under_the_station_is_heard_and_the_station_still_reads() {
        let rate = 320_000.0;
        let programme = tone(1_000.0, 0.2, rate, 8192 * 157);
        let (rx, heard) = station_over(&programme, 1, rate);

        let (groups, errors, synced) = rx.rds_stats();
        assert!(synced, "the block synchroniser never framed under a programme");
        // The same 44 groups the silent station reads, and nothing rejected,
        // so the programme costs the data nothing.
        assert_eq!(groups, 44, "groups in four seconds with a tone underneath");
        assert_eq!(errors, 0, "blocks rejected");
        assert_eq!(rx.station().pi, Some(0xC479));
        assert_eq!(rx.station().name.as_deref(), Some("WAVESHRK"));
        assert_eq!(rx.station().radiotext.as_deref(), Some("A TEST OF THE RDS TRANSMITTER"));

        // What the pilot and the data leave: 1 - 0.09 - 0.04 of what was
        // handed in, which is the level the multiplex says it mixes at.
        let at = amplitude_at(&heard, 1_000.0, rate);
        assert!((0.173..=0.175).contains(&at), "a 0.2 tone came back at {at:.5}, wanted 0.174");
        let off = amplitude_at(&heard, 3_000.0, rate);
        assert!(off < 1e-3, "the programme spread to 3 kHz at {off:.5}");
    }

    /// Two channels are summed rather than refused: a mono station carrying
    /// a stereo programme transmits the sum, which is what the difference
    /// signal on 38 kHz would have been taken from.
    #[test]
    fn a_stereo_programme_is_folded_to_the_mono_a_station_transmits() {
        let rate = 320_000.0;
        let left = tone(1_000.0, 0.2, rate, 8192 * 157);
        let right = tone(4_000.0, 0.2, rate, 8192 * 157);
        let mut both = Vec::with_capacity(left.len() * 2);
        for (l, r) in left.iter().zip(&right) {
            both.push(*l);
            both.push(*r);
        }
        let (rx, heard) = station_over(&both, 2, rate);

        assert_eq!(rx.station().name.as_deref(), Some("WAVESHRK"));
        // Halved by the fold, then the 0.87 the multiplex leaves the
        // programme: 0.2 in each channel is 0.087 of each out.
        for hz in [1_000.0, 4_000.0] {
            let at = amplitude_at(&heard, hz, rate);
            assert!((0.086..=0.088).contains(&at), "{hz} Hz came back at {at:.5}, wanted 0.087");
        }
    }

    /// A programme reaching the pilot is a station with no pilot, so the cut
    /// happens here rather than being left to whatever is connected.
    ///
    /// Measured through this receiver at 320 kS/s with a full scale tone on
    /// the programme input: at 19.5 kHz unfiltered the station reads zero
    /// groups and never gives its name, and through this filter it reads all
    /// 44 and names itself. 10 kHz passes untouched either way.
    #[test]
    fn a_programme_above_fifteen_kilohertz_is_cut_before_it_reaches_the_pilot() {
        let rate = 320_000.0;
        let (rx, heard) = station_over(&tone(19_500.0, 1.0, rate, 8192 * 157), 1, rate);
        let at = amplitude_at(&heard, 19_500.0, rate);
        assert!(at < 1e-4, "19.5 kHz reached the multiplex at {at:.6}");
        let (groups, _, _) = rx.rds_stats();
        assert_eq!(groups, 44, "groups under a full scale tone on the pilot");
        assert_eq!(rx.station().name.as_deref(), Some("WAVESHRK"), "unfiltered this reads nothing");

        let (rx, heard) = station_over(&tone(10_000.0, 0.2, rate, 8192 * 157), 1, rate);
        let at = amplitude_at(&heard, 10_000.0, rate);
        assert!((0.173..=0.175).contains(&at), "10 kHz came back at {at:.5}, wanted 0.174");
        assert_eq!(rx.rds_stats().0, 44, "groups under a 10 kHz programme");
    }

    /// The taps are what decides where the programme stops, and the cost is
    /// a multiply a tap on every sample of the multiplex.
    ///
    /// Measured on the designed response at 320 kS/s. 63 taps has already
    /// taken 0.9 dB off 10 kHz, which is inside the programme, and leaves a
    /// component on the pilot only 16 dB down; 255 taps buys another 40 dB
    /// at 19 kHz on a band that has nothing up there, for twice the work.
    #[test]
    fn the_programme_filter_is_flat_to_ten_kilohertz_and_gone_by_the_pilot() {
        let rate = 320_000.0;
        let taps = dsp::fir::lowpass(PROGRAMME_TAPS, PROGRAMME_HZ / rate, 60.0);
        let db = |hz: f64| {
            let w = std::f64::consts::TAU * hz / rate;
            let (mut re, mut im) = (0.0f64, 0.0f64);
            for (k, t) in taps.iter().enumerate() {
                re += f64::from(*t) * (w * k as f64).cos();
                im -= f64::from(*t) * (w * k as f64).sin();
            }
            20.0 * (re * re + im * im).sqrt().log10()
        };
        assert!(db(1_000.0).abs() < 0.05, "1 kHz at {:.2} dB", db(1_000.0));
        assert!(db(10_000.0).abs() < 0.05, "10 kHz at {:.2} dB, 63 taps takes 0.9", db(10_000.0));
        assert!((-6.2..=-5.8).contains(&db(15_000.0)), "15 kHz at {:.2} dB", db(15_000.0));
        assert!(db(19_000.0) < -38.0, "the pilot at {:.2} dB, 63 taps leaves 16", db(19_000.0));
    }

    /// `fm_mod` takes one real sample per output sample, so a programme at
    /// any other rate would be transmitted at the wrong speed. Everything in
    /// the tree that feeds a transmitter resamples itself; `mic` does.
    #[test]
    fn a_programme_at_another_rate_is_refused() {
        let mut tx = RdsTxNode::default();
        let at = |rate: f64| PortSpec {
            spec: StreamSpec { kind: PortKind::Real, rate, channels: 1, ..Default::default() },
            latency: 0,
        };
        assert!(tx.negotiate(&[spec(320_000.0), at(320_000.0)]).is_ok());
        assert!(tx.negotiate(&[spec(320_000.0), at(48_000.0)]).is_err());
        // Nothing connected is a station carrying its identity over silence.
        let silent = PortSpec { spec: StreamSpec::silence(), latency: 0 };
        assert!(tx.negotiate(&[spec(320_000.0), silent]).is_ok());
    }

    /// What the programme is does not change what the data reads.
    ///
    /// Measured through this receiver at 320 kS/s over four seconds. With the
    /// modulation axis free to flip at the branch cut, a tone at 0.8 of full
    /// deviation read 24 groups with 51 blocks rejected at 100 Hz, 28 with 44
    /// at 1 kHz, 43 with 1 at 5 kHz and 34 with 25 at 12 kHz, and the 100 Hz
    /// station never gave its name. Every one of them now reads the full 44
    /// with nothing rejected.
    #[test]
    fn no_programme_tone_or_level_costs_the_station_a_group() {
        let rate = 320_000.0;
        for level in [0.2f32, 0.8] {
            for hz in [100.0f64, 1_000.0, 5_000.0, 12_000.0] {
                let (rx, _) = station_over(&tone(hz, level, rate, 8192 * 157), 1, rate);
                let (groups, errors, _) = rx.rds_stats();
                assert_eq!(groups, 44, "{hz} Hz at {level}: groups in four seconds");
                assert_eq!(errors, 0, "{hz} Hz at {level}: blocks rejected");
                assert_eq!(rx.station().name.as_deref(), Some("WAVESHRK"), "{hz} Hz at {level}");
            }
        }
    }

    /// The subcarrier is at 57 kHz, so a stream that cannot reach it is
    /// refused rather than transmitting something that folds over the audio.
    #[test]
    fn a_stream_too_narrow_for_the_subcarrier_is_refused() {
        let mut tx = RdsTxNode::default();
        assert!(tx.negotiate(&[spec(48_000.0)]).is_err());
        assert!(tx.negotiate(&[spec(320_000.0)]).is_ok());
    }
}
