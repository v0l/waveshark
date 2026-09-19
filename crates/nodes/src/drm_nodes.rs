//! DRM as a graph node: one 10 kHz shortwave or medium wave channel, from
//! the air to the services it carries and what they are called.
//!
//! The layers are elsewhere. The OFDM, the pilots and the cell map are
//! `dsp::drm`; the rate 1/6 code is `dsp::conv`; the interleaver, the energy
//! dispersal, the checks and the fields are `decode::drm`. What this module
//! does is put them in order and say on the bus what the multiplex says
//! about itself.
//!
//! The sound is not decoded. A DRM service is xHE-AAC, which needs a decoder
//! no build here links, so what reaches the packet list is the multiplex and
//! its services: their identifiers, languages, programme types and labels.

use crate::NodeSpec;
use crate::protocol::{Placed, Placement, Protocol, Shape, Stickiness};
use common::{C32, Result};
use decode::drm::{self as drmdec, DrmReceiver, Fac, Multiplex, SdcMode, Service, Stats};
use dsp::drm::{Mode, Occupancy};
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use identify::Signal;
pub use identify::drm::BAND_HZ;
pub use identify::drm::CHANNEL_WIDTH_HZ;
pub use identify::drm::DEFAULT_HZ;
pub use identify::drm::DrmProtocol;
pub use identify::drm::RATE_HZ;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// One multiplex as a stage.
pub struct DrmNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    resample: Rational,
    rx: DrmReceiver,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    at_rate: Vec<C32>,
    /// Which services have been announced, and under what label, so a table
    /// repeated every 400 ms is said once and again when it changes.
    told: Vec<(u32, Option<String>)>,
    at: f64,
}

impl Default for DrmNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl DrmNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(RATE_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            resample: Rational::with_ratio(1, 1),
            rx: DrmReceiver::new(Mode::B),
            mixed: Vec::new(),
            narrow: Vec::new(),
            at_rate: Vec::new(),
            told: Vec::new(),
            at: 0.0,
        }
    }

    pub fn multiplex(&self) -> &Multiplex {
        self.rx.multiplex()
    }

    pub fn stats(&self) -> Stats {
        self.rx.stats
    }
}

impl Simple for DrmNode {
    fn name(&self) -> &str {
        "drm"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("drm reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < RATE_HZ {
            return Err(common::Error::other("drm needs 12 kS/s of channel"));
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("drm needs its channel inside the span"));
        }
        let (factor, resample) = dsp::resample::stage(rate, RATE_HZ, 4096)
            .ok_or_else(|| common::Error::other("drm cannot reach 12 kS/s from here"))?;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.resample = resample.unwrap_or_else(|| Rational::with_ratio(1, 1));
        self.rx = DrmReceiver::new(Mode::B);
        self.told.clear();

        let mut out = i.spec.with_kind(PortKind::Bytes);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        // What leaves is the signalling this reads, which is 64 bits of fast
        // access channel every 400 ms.
        out.rate = 160.0;
        Ok(out)
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(iq) = i.as_iq() else { return Ok(()) };
        self.at = c.timestamp();
        self.mixed.clear();
        self.mixer.process(iq, &mut self.mixed);
        self.narrow.clear();
        self.decim.process(&self.mixed, &mut self.narrow);
        self.at_rate.clear();
        self.resample.process(&self.narrow, &mut self.at_rate);
        self.rx.push(&self.at_rate);

        let m = self.rx.multiplex();
        let fresh: Vec<(Service, Option<String>)> = m
            .services
            .iter()
            .map(|s| (*s, m.label(s.short_id).map(str::to_string)))
            .filter(|(s, label)| !self.told.iter().any(|(id, told)| *id == s.id && told == label))
            .collect();
        for (service, label) in fresh {
            self.told.retain(|(id, _)| *id != service.id);
            self.told.push((service.id, label.clone()));
            if let Some(d) = drmdec::service_decoded(
                &self.rx,
                service,
                label,
                common::Hz(self.channel_hz as u64),
                self.at,
            ) {
                c.emit(pipeline::event::Event::Decoded(d));
            }
        }
        let _ = o;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.rx = DrmReceiver::new(Mode::B);
        self.told.clear();
    }
}

impl Protocol for DrmProtocol {
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

    /// Long, medium and shortwave, which is where DRM is allocated. The
    /// 26 MHz band at the top of it carries the local transmissions.

    fn stage_label(&self, hz: f64) -> String {
        format!("{:.0} DRM", hz / 1e3)
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    /// The signalling channels. The multiplex is not decoded, so nothing
    /// reaches the audio bus.
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Bytes]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The channel this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "drm",
    summary: "One DRM channel: OFDM, its fast access channel, its services",
    category: Category::Decode,
    // What reaches the packet list is the multiplex and its services,
    // emitted rather than carried on a wire.
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DrmNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

/// A multiplex on the air, for a test that has to know what is in one: two
/// services, named, described one per frame the way a real transmitter
/// rotates them.
#[cfg(test)]
pub fn transmit(mode: Mode, occ: Occupancy, frames: usize) -> Vec<C32> {
    use decode::dab::ProgrammeType;
    use decode::drm::{Language, MscMode, SdcMode};

    let services = [
        Service {
            short_id: 0,
            id: 0xA1_B2C3,
            language: Language::English,
            audio: true,
            programme: ProgrammeType::News,
        },
        Service {
            short_id: 1,
            id: 0x0D_1E2F,
            language: Language::German,
            audio: true,
            programme: ProgrammeType::Pop,
        },
    ];
    let labels = vec![decode::drm::label_entity(0, "Shark"), decode::drm::label_entity(1, "Reef")];
    let sdc = decode::drm::encode_sdc(&labels, mode, occ).expect("a channel this wide");
    let mut tx = dsp::drm::tx::Modulator::new(mode, occ);
    let mut out = Vec::new();
    for i in 0..frames {
        let fac = Fac {
            frame_id: (i % 3) as u8,
            occupancy: Some(occ),
            long_interleave: true,
            msc: MscMode::Sm16,
            sdc: SdcMode::Qam4,
            audio_services: 2,
            data_services: 0,
            service: services[i % services.len()],
        };
        let cells = decode::drm::encode_fac(fac);
        let carried: &[C32] = if fac.frame_id == 0 { &sdc } else { &[] };
        tx.frame(&cells, carried, &mut out);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;
    use decode::dab::ProgrammeType;
    use decode::drm::Language;

    /// Four seconds of a synthesised multiplex, read the whole way through:
    /// both services, their identifiers, languages, programme types and the
    /// labels the description channel gives them.
    #[test]
    fn a_multiplex_reads_its_own_service_list() {
        let air = transmit(Mode::B, Occupancy::Full10, 10);
        let mut rx = DrmReceiver::new(Mode::B);
        let good = rx.push(&air);
        // Ten frames of 400 ms went out and seven came back: the search for
        // where a frame begins looks over two frames, so the last two are
        // left, and acquisition costs the first.
        assert_eq!(rx.stats.frames, 7);
        assert_eq!(good, 7);
        assert_eq!(rx.stats.fac_ok, 7);
        assert_eq!(rx.stats.fac_bad, 0);
        // A description channel every third frame, and three of them in
        // seven frames read.
        assert_eq!(rx.stats.sdc_ok, 3);
        assert_eq!(rx.stats.sdc_skipped, 0);

        let m = rx.multiplex();
        assert_eq!(m.mode, Some(Mode::B));
        assert_eq!(m.occupancy, Some(Occupancy::Full10));
        assert_eq!(m.audio_services, 2);
        assert_eq!(m.data_services, 0);
        assert_eq!(m.services.len(), 2);
        let first = m.service(0).expect("the first service");
        let label = m.label(0);
        assert_eq!(first.id, 0xA1_B2C3);
        assert_eq!(first.language, Language::English);
        assert_eq!(first.programme, ProgrammeType::News);
        assert_eq!(label, Some("Shark"));
        let second = m.service(1).expect("the second service");
        let label = m.label(1);
        assert_eq!(second.id, 0x0D_1E2F);
        assert_eq!(second.language, Language::German);
        assert_eq!(second.programme, ProgrammeType::Pop);
        assert_eq!(label, Some("Reef"));
        assert!(rx.snr_db() > 40.0, "a clean multiplex reads {} dB", rx.snr_db());
        assert_eq!(rx.stats.quality(), Some(1.0));
    }

    /// The same multiplex with noise on it. Measured down the range: six
    /// frames read at 15 and 12 dB and five from 9 dB down, every fast
    /// access channel of them checking out to 4 dB, three of five at 3 dB
    /// and none at 1 dB, which is where a rate 3/5 code over 4-QAM gives up.
    /// The description channel, which is four times the block at half the
    /// rate, follows it down: both labels arrive at 4 dB and neither at 3.
    #[test]
    fn a_noisy_multiplex_still_names_its_services() {
        for (snr_db, floor) in [(12.0f32, 6usize), (4.0, 5)] {
            let mut air = transmit(Mode::B, Occupancy::Full10, 10);
            let signal: f32 =
                air.iter().map(|s| s.norm_sqr()).sum::<f32>() / air.len().max(1) as f32;
            let level = (signal / 10f32.powf(snr_db / 10.0)).sqrt() / 2f32.sqrt();
            let mut state = 0x5eed_1234u32;
            for s in air.iter_mut() {
                let mut next = || {
                    state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (state >> 16) as i16 as f32 / 32768.0 * level * 1.732
                };
                *s += C32::new(next(), next());
            }
            let mut rx = DrmReceiver::new(Mode::B);
            let good = rx.push(&air);
            assert_eq!(good, floor, "{good} frames at {snr_db} dB, wanted {floor}");
            let m = rx.multiplex();
            assert_eq!(m.services.len(), 2, "at {snr_db} dB");
            assert_eq!(m.label(0), Some("Shark"), "at {snr_db} dB");
            assert_eq!(m.label(1), Some("Reef"), "at {snr_db} dB");
        }
    }

    /// Minutes of noise into the whole receiver: no frame, no service, no
    /// lock.
    #[test]
    fn noise_names_nothing() {
        let mut rx = DrmReceiver::new(Mode::B);
        let mut state = 0xfeed_beefu32;
        for _ in 0..150 {
            let air: Vec<C32> = (0..Mode::B.frame())
                .map(|_| {
                    let mut next = || {
                        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                        (state >> 16) as i16 as f32 / 32768.0
                    };
                    C32::new(next(), next())
                })
                .collect();
            assert_eq!(rx.push(&air), 0);
        }
        assert_eq!(rx.stats.fac_ok, 0);
        assert!(!rx.locked());
        assert_eq!(*rx.multiplex(), Multiplex::default());
    }

    #[test]
    fn the_channel_has_to_be_inside_the_span_and_wide_enough() {
        let mut n = DrmNode::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(48_000.0, Hz(7_200_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let thin = PortSpec { spec: StreamSpec::iq(8_000.0, Hz(3_965_000)), latency: 0 };
        assert!(n.negotiate(&thin).is_err());
        for rate in [12_000.0, 48_000.0, 240_000.0, 2_400_000.0] {
            let mut n = DrmNode::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(3_965_000)), latency: 0 };
            let out = n.negotiate(&spec).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
            assert_eq!(out.kind, PortKind::Bytes);
            assert_eq!(out.center, Hz(3_965_000));
        }
    }
}
