//! DAB as a graph node: one band III ensemble, from the air to its station
//! list.
//!
//! The layers are elsewhere. The null symbol, the phase reference and the
//! differential QPSK are `dsp::dab`; the convolutional code is `dsp::conv`;
//! the puncturing, the energy dispersal, the block check and the ensemble's
//! tables are `decode::dab`. What this module does is put them in order and
//! say on the bus what the ensemble calls itself.
//!
//! What comes out is the ensemble and its services: their names, programme
//! types, subchannels and bit rates, which is what a person tuning wants to
//! see. The sound is not decoded, so nothing reaches the audio bus: the main
//! service channel, its time interleaving and the HE-AAC superframes of DAB+
//! are a layer this does not have.

use crate::NodeSpec;
use crate::protocol::{Placed, Placement, Protocol, Shape, Stickiness};
use common::{C32, Result};
#[cfg(test)]
use decode::dab::Audio;
use decode::dab::{Ensemble, Fic, ProgrammeType};
use dsp::dab::{self, Dab, Mode, Symbol};
use dsp::resample::Rational;
use dsp::{FirDecim, Mixer};
use pipeline::event::Decoded;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

/// The rate every transmission mode is defined at.
pub const RATE_HZ: f64 = dab::RATE_HZ;
/// What an ensemble occupies.
pub const CHANNEL_WIDTH_HZ: f64 = dab::CHANNEL_WIDTH_HZ;
/// Block 11D, which carries the British national ensemble and is as good a
/// place as any to point a receiver that has been told nothing else.
pub const DEFAULT_HZ: f64 = 222_064_000.0;

/// Band III as Europe allocates it to DAB: blocks 5A at 174.928 MHz to 13F
/// at 239.200 MHz.
pub const BAND_HZ: (f64, f64) = (174_000_000.0, 240_000_000.0);

/// A whole DAB receiver: samples in, an ensemble's tables out.
pub struct DabReceiver {
    front: Dab,
    fic: Fic,
    symbols: Vec<Symbol>,
    snr_db: f32,
}

impl Default for DabReceiver {
    fn default() -> Self {
        Self::new(Mode::I)
    }
}

impl DabReceiver {
    pub fn new(mode: Mode) -> Self {
        Self { front: Dab::new(mode), fic: Fic::new(), symbols: Vec::new(), snr_db: f32::NAN }
    }

    /// The ensemble as its tables describe it so far.
    pub fn ensemble(&self) -> &Ensemble {
        self.fic.ensemble()
    }

    pub fn stats(&self) -> decode::dab::Stats {
        self.fic.stats
    }

    pub fn locked(&self) -> bool {
        self.front.locked()
    }

    pub fn frames(&self) -> u64 {
        self.front.frames()
    }

    /// Signal to noise off the last symbol read, or NaN before any was.
    pub fn snr_db(&self) -> f32 {
        self.snr_db
    }

    /// The frequency error the front end is correcting.
    pub fn offset_hz(&self) -> f64 {
        self.front.offset_hz()
    }

    /// Read what `iq` holds. Returns the blocks that passed their check.
    pub fn push(&mut self, iq: &[C32]) -> usize {
        let mut symbols = std::mem::take(&mut self.symbols);
        symbols.clear();
        self.front.push(iq, &mut symbols);
        let fic_symbols = self.front.mode().fic_symbols();
        let mut good = 0;
        for symbol in &symbols {
            self.snr_db = symbol.snr_db;
            // The fast information channel is the first symbols of a frame;
            // the rest is the main service channel, which nothing here reads.
            if symbol.index <= fic_symbols {
                good += self.fic.push(&symbol.soft);
            }
        }
        self.symbols = symbols;
        good
    }
}

/// One ensemble as a stage: samples in, its fast information blocks out, and
/// what it says about itself on the bus.
pub struct DabNode {
    channel_hz: f64,
    mixer: Mixer,
    decim: FirDecim,
    resample: Rational,
    rx: DabReceiver,
    mixed: Vec<C32>,
    narrow: Vec<C32>,
    at_rate: Vec<C32>,
    /// Whether the ensemble itself has been announced, and which services
    /// have, so a table repeated every 96 ms is said once.
    told: Option<String>,
    named: Vec<u32>,
    at: f64,
}

impl Default for DabNode {
    fn default() -> Self {
        Self::new(DEFAULT_HZ)
    }
}

impl DabNode {
    pub fn new(channel_hz: f64) -> Self {
        Self {
            channel_hz,
            mixer: Mixer::new(0.0, 1.0),
            decim: FirDecim::design_hz(RATE_HZ, 1, CHANNEL_WIDTH_HZ / 2.0, 60.0),
            resample: Rational::with_ratio(1, 1),
            rx: DabReceiver::new(Mode::I),
            mixed: Vec::new(),
            narrow: Vec::new(),
            at_rate: Vec::new(),
            told: None,
            named: Vec::new(),
            at: 0.0,
        }
    }

    /// The ensemble as it has been read so far.
    pub fn ensemble(&self) -> &Ensemble {
        self.rx.ensemble()
    }

    pub fn stats(&self) -> decode::dab::Stats {
        self.rx.stats()
    }

    /// The ensemble, once it has a name.
    fn announce(&mut self, c: &mut NodeCtx<'_>) {
        let e = self.rx.ensemble();
        let Some(name) = e.name.clone() else { return };
        let stats = self.rx.stats();
        let mut fields = vec![
            ("ensemble".into(), common::Value::Text(name.clone())),
            ("mode".into(), common::Value::Text(self.rx.front.mode().label().into())),
            ("services".into(), common::Value::Int(e.services.len() as i64)),
            ("snr_db".into(), common::Value::Float(self.rx.snr_db() as f64)),
        ];
        if let Some(id) = e.id {
            fields.push(("ensemble_id".into(), common::Value::Text(format!("{id:04X}"))));
        }
        if let Some(q) = stats.quality() {
            fields.push(("blocks_ok".into(), common::Value::Float((100.0 * q) as f64)));
        }
        let detail = match e.id {
            Some(id) => format!("{name} ({id:04X})"),
            None => name.clone(),
        };
        self.told = Some(name);
        c.emit(pipeline::event::Event::Decoded(
            Decoded::bytes("DAB", common::Hz(self.channel_hz as u64), self.at, Vec::new())
                .with_detail(detail)
                .with_fields(fields)
                .with_modulation(common::Modulation::Ofdm)
                .with_crc(Some(true)),
        ));
    }

    /// A service, once the tables have named it.
    fn announce_service(&mut self, id: u32, c: &mut NodeCtx<'_>) {
        let e = self.rx.ensemble();
        let Some(service) = e.service(id) else { return };
        let Some(name) = service.name.clone() else { return };
        let audio = service.audio();
        let sub = audio.and_then(|(id, _)| e.sub_channel(id)).copied();
        let mut fields = vec![
            ("service".into(), common::Value::Text(name.clone())),
            ("service_id".into(), common::Value::Text(format!("{id:04X}"))),
        ];
        if let Some((_, kind)) = audio {
            fields.push(("audio".into(), common::Value::Text(kind.label())));
        }
        if let Some(pty) = service.programme_type.filter(|p| *p != ProgrammeType::None) {
            fields.push(("programme".into(), common::Value::Text(pty.label().into())));
        }
        if let Some(sub) = sub {
            fields.push(("subchannel".into(), common::Value::Int(sub.id as i64)));
            fields.push(("bitrate".into(), common::Value::Int(sub.bitrate_kbps as i64)));
            fields.push(("protection".into(), common::Value::Text(sub.protection.label())));
        }
        let detail = match (audio.map(|(_, k)| k.label()), sub.map(|s| s.bitrate_kbps)) {
            (Some(kind), Some(rate)) => format!("{name} ({kind}, {rate} kbit/s)"),
            (Some(kind), None) => format!("{name} ({kind})"),
            _ => name.clone(),
        };
        // A service keeps its identifier across ensembles and retunes, which
        // is what a station list rows on.
        let who = common::Identity::new("dab-service", format!("{id:04X}")).named(name);
        c.emit(pipeline::event::Event::Decoded(
            Decoded::bytes("DAB", common::Hz(self.channel_hz as u64), self.at, Vec::new())
                .by(who)
                .with_detail(detail)
                .with_fields(fields)
                .with_modulation(common::Modulation::Ofdm)
                .with_crc(Some(true)),
        ));
    }
}

impl Simple for DabNode {
    fn name(&self) -> &str {
        "dab"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("dab reads complex baseband"));
        }
        let (rate, center) = (i.spec.rate, i.spec.center.as_f64());
        if rate < RATE_HZ {
            return Err(common::Error::other("dab needs 2.048 MS/s of channel"));
        }
        if (self.channel_hz - center).abs() > rate / 2.0 - CHANNEL_WIDTH_HZ / 2.0 {
            return Err(common::Error::other("dab needs its ensemble inside the span"));
        }
        let (factor, resample) = dsp::resample::stage(rate, RATE_HZ, 4096)
            .ok_or_else(|| common::Error::other("dab cannot reach 2.048 MS/s from here"))?;
        self.mixer = Mixer::new(center - self.channel_hz, rate);
        self.decim = FirDecim::design_hz(rate, factor, CHANNEL_WIDTH_HZ / 2.0, 60.0);
        self.resample = resample.unwrap_or_else(|| Rational::with_ratio(1, 1));
        self.rx = DabReceiver::new(Mode::I);
        self.told = None;
        self.named.clear();

        let mut out = i.spec.with_kind(PortKind::Bytes);
        out.center = common::Hz(self.channel_hz as u64);
        out.bandwidth = CHANNEL_WIDTH_HZ;
        // What leaves is the fast information channel, which is 96 kbit/s of
        // blocks whatever the ensemble carries.
        out.rate = 12_000.0;
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

        let named = self.rx.ensemble().name.clone();
        if named.is_some() && self.told != named {
            self.announce(c);
        }
        let fresh: Vec<u32> = self
            .rx
            .ensemble()
            .stations()
            .filter(|s| !self.named.contains(&s.id))
            .map(|s| s.id)
            .collect();
        for id in fresh {
            self.named.push(id);
            self.announce_service(id, c);
        }
        let _ = o;
        Ok(())
    }

    fn reset(&mut self) {
        self.mixer.reset();
        self.decim.reset();
        self.rx = DabReceiver::new(Mode::I);
        self.told = None;
        self.named.clear();
    }
}

pub struct DabProtocol;

impl Protocol for DabProtocol {
    fn id(&self) -> &'static str {
        "dab"
    }
    fn label(&self) -> &'static str {
        "dab"
    }
    fn aliases(&self) -> &'static [&'static str] {
        &["dab+"]
    }
    /// Band III, which is where every European ensemble is. The L band was
    /// allocated to it too and nothing is left transmitting there.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND_HZ])
    }
    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: RATE_HZ,
            feed_rate_hz: RATE_HZ,
            span_wide: false,
            // An ensemble is on the air without stopping, so there is no
            // burst for the classifier to name and nothing to wait for.
            families: &[],
        }
    }
    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.3} DAB", hz / 1e6)
    }
    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }
    /// The fast information channel's blocks. The sound is not read, so
    /// nothing goes to the audio bus.
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Bytes]
    }
    fn chain(&self, at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name).f(CHANNEL_HZ, at.center_hz)]
    }
}

/// The ensemble this stage is pointed at.
const CHANNEL_HZ: &str = "channel_hz";

pub const DESC: StageDesc = StageDesc {
    name: "dab",
    summary: "One DAB ensemble: OFDM, the fast information channel, its stations",
    category: Category::Decode,
    // What it puts on its port is the fast information channel. What reaches
    // the packet list is the ensemble and its services, emitted rather than
    // carried on a wire.
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(DabNode::new(s.f64_or(CHANNEL_HZ, DEFAULT_HZ))))
}

/// An ensemble on the air, for a test that has to know what is in one.
#[cfg(test)]
pub fn transmit(seconds: f64) -> Vec<C32> {
    use decode::dab::FicTx;
    let mode = Mode::I;
    let mut tx = FicTx::new();
    tx.ensemble(0xC1AB)
        .ensemble_label(0xC1AB, "WAVESHARK")
        .sub_channel(1, 0, 72, 3)
        .sub_channel(2, 72, 96, 1)
        .service(0xC221, 1, Audio::AacPlus)
        .service_label(0xC221, "Shark FM")
        .programme_type(0xC221, ProgrammeType::Pop)
        .service(0xC222, 2, Audio::Mp2)
        .service_label(0xC222, "Reef Radio")
        .programme_type(0xC222, ProgrammeType::News);
    let fic: Vec<u8> = {
        let bits = tx.frame_bits(mode.fibs());
        let mut coded = Vec::new();
        for block in bits.chunks(decode::dab::CODEWORD_DATA) {
            let mut bytes = Vec::with_capacity(block.len() / 8);
            for byte in block.chunks(8) {
                bytes.push(byte.iter().fold(0u8, |acc, &b| (acc << 1) | b));
            }
            coded.extend(decode::dab::encode(&bytes));
        }
        coded
    };

    let mut modulator = dab::tx::Modulator::new(mode);
    let mut out = Vec::new();
    let frames = (seconds / mode.frame_seconds()).ceil() as usize;
    let mut bits = vec![0u8; modulator.bits_per_frame()];
    bits[..fic.len()].copy_from_slice(&fic);
    for _ in 0..frames {
        modulator.frame(&bits, &mut out);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Hz;

    /// Two seconds of a synthesised ensemble, read the whole way through: the
    /// ensemble's name, both stations, their programme types and the bit
    /// rates their subchannels are carried at.
    #[test]
    fn an_ensemble_reads_its_own_station_list() {
        let air = transmit(2.0);
        let mut rx = DabReceiver::new(Mode::I);
        let good = rx.push(&air);
        // Twenty-one frames of 96 ms went out and twenty came back: a frame
        // is only read once the phase reference of the one after it has timed
        // it, so the last frame of a recording is always left.
        assert_eq!(rx.frames(), 20);
        assert_eq!(rx.stats().codewords, 80);
        // Four codewords a frame, three blocks a codeword, every one of them
        // checking out on a signal with nothing on it.
        assert_eq!(good, 240);
        assert_eq!(rx.stats().good, 240);
        assert_eq!(rx.stats().bad, 0);

        let e = rx.ensemble();
        assert_eq!(e.name.as_deref(), Some("WAVESHARK"));
        assert_eq!(e.id, Some(0xC1AB));
        assert_eq!(e.services.len(), 2);
        assert_eq!(e.sub_channels.len(), 2);
        let names: Vec<&str> = e.stations().filter_map(|s| s.name.as_deref()).collect();
        assert_eq!(names, vec!["Shark FM", "Reef Radio"]);
        let shark = e.service(0xC221).expect("the first station");
        assert_eq!(shark.programme_type, Some(ProgrammeType::Pop));
        assert_eq!(shark.audio(), Some((1, Audio::AacPlus)));
        assert_eq!(e.sub_channel(1).expect("its subchannel").bitrate_kbps, 96);
        let reef = e.service(0xC222).expect("the second station");
        assert_eq!(reef.programme_type, Some(ProgrammeType::News));
        assert_eq!(reef.audio(), Some((2, Audio::Mp2)));
        assert_eq!(e.sub_channel(2).expect("its subchannel").bitrate_kbps, 64);
        assert!(rx.snr_db() > 40.0, "a clean ensemble reads {} dB", rx.snr_db());
    }

    /// The same ensemble with noise on it.
    ///
    /// Acquisition costs the first frame, so nineteen of the twenty-one
    /// transmitted are read and 228 blocks is everything there is to get.
    /// Measured down the range: 228 at 15, 12, 9 and 6 dB, 227 at 4 dB, 221
    /// at 3 dB and 159 at 2 dB, which is where a rate 1/3 code gives up.
    #[test]
    fn a_noisy_ensemble_still_names_itself() {
        for (snr_db, floor) in [(6.0f32, 228usize), (3.0, 200)] {
            let mut air = transmit(2.0);
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
            let mut rx = DabReceiver::new(Mode::I);
            let good = rx.push(&air);
            assert_eq!(rx.frames(), 19);
            assert!(good >= floor, "{good} blocks of 228 at {snr_db} dB, wanted {floor}");
            assert_eq!(rx.ensemble().name.as_deref(), Some("WAVESHARK"));
            assert_eq!(rx.ensemble().stations().count(), 2);
        }
    }

    /// Minutes of noise into the whole node: no lock, no block, no ensemble.
    #[test]
    fn noise_names_nothing() {
        let mut rx = DabReceiver::new(Mode::I);
        let mut state = 0xfeed_beefu32;
        for _ in 0..600 {
            let air: Vec<C32> = (0..Mode::I.frame())
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
        assert_eq!(rx.stats().good, 0);
        assert!(!rx.locked());
        assert_eq!(*rx.ensemble(), Ensemble::default());
    }

    #[test]
    fn the_ensemble_has_to_be_inside_the_span_and_wide_enough() {
        let mut n = DabNode::new(DEFAULT_HZ);
        let far = PortSpec { spec: StreamSpec::iq(2_048_000.0, Hz(200_000_000)), latency: 0 };
        assert!(n.negotiate(&far).is_err());
        let thin = PortSpec { spec: StreamSpec::iq(1_000_000.0, Hz(222_064_000)), latency: 0 };
        assert!(n.negotiate(&thin).is_err());
        for rate in [2_048_000.0, 2_400_000.0, 8_000_000.0, 20_000_000.0] {
            let mut n = DabNode::new(DEFAULT_HZ);
            let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(222_064_000)), latency: 0 };
            let out = n.negotiate(&spec).unwrap_or_else(|e| panic!("{rate} refused: {e}"));
            assert_eq!(out.kind, PortKind::Bytes);
            assert_eq!(out.center, Hz(222_064_000));
        }
    }
}
