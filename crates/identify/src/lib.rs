//! What a recording holds, from the samples and the tuning alone.
//!
//! The receiver's auto node answers two separate questions: what could be on
//! this frequency, and what should the graph do about it. Only the first
//! applies to a program that has a capture and a dial reading and no graph at
//! all, so it lives here, below `nodes`, with nothing under it but `common`,
//! `dsp` and `decode`.
//!
//! A [`Signal`] is the part of a protocol that can answer it: where it can
//! be, what stream it reads, and how to read one. `nodes` takes its band and
//! shape from these same descriptions and wires the same `dsp` front end to
//! the same `decode` payload, so a receiver and a program reading a file
//! cannot drift apart.

pub mod acars;
pub mod aero;
pub mod ais;
pub mod aprs;
pub mod apt;
pub mod ble;
mod chan;
pub mod dab;
pub mod dfm;
pub mod dmr;
pub mod drm;
pub mod droneid;
pub mod dvbt;
pub mod eas;
pub mod elrs;
pub mod epirb;
pub mod flex;
pub mod ft8;
pub mod gsm;
pub mod ieee802154;
pub mod imet;
pub mod iridium;
pub mod lms6;
pub mod lora;
pub mod lrpt;
pub mod m10;
pub mod m17;
pub mod mdc;
pub mod meisei;
pub mod modes;
pub mod morse;
pub mod mrz;
pub mod nrf24;
pub mod nxdn;
pub mod p25;
pub use chan::{Channel, strongest};
mod place;
pub mod pocsag;
pub mod rs41;
pub mod rtty;
pub mod sstv;
pub mod stdc;
pub mod tempest;
pub mod tetra;
pub mod twotone;
pub mod uat;
pub mod vdl2;
pub mod video;
pub mod wefax;
pub mod wifi;
pub mod wmbus;
pub mod zwave;

use common::C32;
use common::packet::Proto;
pub use place::{CHANNEL_WIDTH_TOLERANCE, Placement, Shape};

/// What reading a recording as one protocol found.
#[derive(Clone, Debug, PartialEq)]
pub struct Ident {
    /// The registry id, which is the word the receiver names it by.
    pub protocol: &'static str,
    pub label: &'static str,
    /// Rows that passed the protocol's own checks. Nothing reports zero: an
    /// identification is what decoded, not a shape that looked right.
    pub frames: usize,
    /// What named itself, in the order first heard: an aircraft address, a
    /// sonde's serial, a network name.
    pub identities: Vec<String>,
    /// Where in the span it was read, which for a protocol on a channel is
    /// the channel and not the middle of the recording.
    pub center_hz: f64,
    /// Everything it read, for a caller that wants the statements rather
    /// than the verdict.
    pub rows: Vec<Proto>,
}

impl Ident {
    fn of(s: &dyn Signal, read: Reading, center_hz: f64) -> Self {
        let Reading { rows, pictures: _, voice_s: _, center_hz: _ } = read;
        // The id rather than the name a transmitter gives itself: one
        // aircraft sends both, and two spellings of one transmitter read as
        // two transmitters.
        let mut identities: Vec<String> = Vec::new();
        for id in rows.iter().filter_map(|r| r.subject.as_ref()).map(|e| e.id.to_string()) {
            if !identities.contains(&id) {
                identities.push(id);
            }
        }
        Self { protocol: s.id(), label: s.label(), frames: rows.len(), identities, center_hz, rows }
    }
}

/// What reading a recording as one protocol produced.
///
/// Rows for a protocol whose decoder makes packets, pictures for one that
/// makes pictures, seconds for one that makes speech. A protocol is named by
/// whichever of these it produced, because a picture is evidence of SSTV in
/// exactly the way a row is evidence of a pager.
#[derive(Default, Clone, Debug, PartialEq)]
pub struct Reading {
    pub rows: Vec<Proto>,
    /// Where in the span it was read, for a protocol that cut a channel out
    /// of it. A statement says nothing about where it was heard, so the
    /// reader that opened the channel says.
    pub center_hz: Option<f64>,
    /// Whole pictures completed.
    pub pictures: usize,
    /// Seconds of speech decoded.
    pub voice_s: f64,
}

impl From<Vec<Proto>> for Reading {
    fn from(rows: Vec<Proto>) -> Self {
        Self { rows, ..Self::default() }
    }
}

impl Reading {
    /// Where the channel this was read on sat
    pub fn at(mut self, center_hz: f64) -> Self {
        self.center_hz = Some(center_hz);
        self
    }
}

impl Reading {
    /// What it read, counted in the units it produces: a row, a picture, or
    /// a second of speech.
    pub fn count(&self) -> usize {
        self.rows.len() + self.pictures + self.voice_s as usize
    }
}

/// A protocol that can be read out of a recording on its own.
pub trait Signal: Send + Sync {
    /// The stage registry's name for the decoder, and the word a table or a
    /// saved channel names it by.
    fn id(&self) -> &'static str;

    /// What it is called where a person reads it.
    fn label(&self) -> &'static str;

    /// Other words it answers to.
    fn aliases(&self) -> &'static [&'static str] {
        &[]
    }

    fn placement(&self) -> Placement;

    fn shape(&self) -> Shape;

    /// The frequency to offer when somebody places it by hand: the calling
    /// channel, the one allocation, the middle of the band.
    fn default_hz(&self) -> f64 {
        self.placement().default_hz().unwrap_or(0.0)
    }

    /// Read the recording as this protocol: the rows it decoded, which is
    /// empty where none of it is there. `iq` is the whole span and
    /// `center_hz` the middle of it, so a protocol on a channel finds its
    /// own channel inside the span.
    ///
    /// Whole rather than block by block because the caller has a file; a
    /// front end that needs blocks cuts its own.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading;
}

/// Every protocol that can be read without a graph.
///
/// Shorter than the receiver's own registry, and it says which of the two
/// questions each protocol has been split for. A protocol added here is one
/// `nodes` then takes its band and shape from.
pub fn all() -> &'static [&'static dyn Signal] {
    static ALL: &[&dyn Signal] = &[
        &modes::ModeS,
        &ais::Ais,
        &uat::Uat,
        &ble::Ble,
        &ieee802154::Ieee802154,
        &wifi::Wifi,
        &droneid::DroneId,
        &video::Video,
        &tempest::Tempest,
        &acars::Acars,
        &sstv::Sstv,
        &apt::Apt,
        &lrpt::Lrpt,
        &wefax::Wefax,
        &vdl2::Vdl2,
        &dvbt::Dvbt,
        &dab::DabProtocol,
        &drm::DrmProtocol,
        &aprs::Aprs,
        &pocsag::Pocsag,
        &flex::Flex,
        &rtty::Rtty,
        &ft8::Ft8,
        &ft8::Ft4,
        &morse::Morse,
        &mdc::Mdc,
        &eas::Eas,
        &twotone::TwoTone,
        &nrf24::Nrf24,
        &m17::M17,
        &dmr::Dmr,
        &p25::P25,
        &nxdn::Nxdn,
        &tetra::Tetra,
        &gsm::Gsm,
        &lora::Lora,
        &elrs::Elrs,
        &wmbus::Wmbus,
        &zwave::ZWave,
        &rs41::Rs41,
        &dfm::Dfm,
        &m10::M10,
        &imet::Imet,
        &meisei::Meisei,
        &mrz::Mrz,
        &lms6::Lms6,
        &epirb::Epirb,
        &stdc::Stdc,
        &aero::Aero,
        &iridium::Iridium,
    ];
    ALL
}

/// The protocols worth trying on a span `rate_hz` wide around `center_hz`.
///
/// The band check the caller would otherwise write for itself, and the reason
/// a 405 MHz recording is never offered to Mode S. A protocol whose decoder
/// refuses the rate is left out here rather than refusing later.
pub fn candidates(rate_hz: f64, center_hz: f64) -> Vec<&'static dyn Signal> {
    let (lo, hi) = (center_hz - rate_hz / 2.0, center_hz + rate_hz / 2.0);
    all()
        .iter()
        .copied()
        .filter(|s| {
            let shape = s.shape();
            rate_hz >= shape.min_rate_hz && s.placement().reaches(lo, hi, shape.widths[0])
        })
        .collect()
}

/// What is in a recording, or `None` where nothing placed there read it.
///
/// Every candidate is run and the one that read most frames wins, because a
/// frame that passed a CRC is the only evidence worth ranking on: a shape
/// that looks like FSK is what the caller already had.
pub fn identify(iq: &[C32], rate_hz: f64, center_hz: f64) -> Option<Ident> {
    identify_all(iq, rate_hz, center_hz).into_iter().next()
}

/// Frames before a recording is named.
///
/// Two, because one is what noise manages. Two minutes of thermal noise at
/// 2.4 MS/s, 288 million samples, produced exactly one Mode S frame whose
/// 24-bit parity checked, and two minutes of the sonde band produced no RS41
/// frame at all; a 24-bit check over that many preamble candidates is
/// expected to pass about once. Nothing real is this thin: the corpus 1090
/// MHz capture has 76 frames in four seconds.
pub const MIN_FRAMES: usize = 2;

/// Every candidate that read something, most frames first.
pub fn identify_all(iq: &[C32], rate_hz: f64, center_hz: f64) -> Vec<Ident> {
    let mut found = read_all(iq, rate_hz, center_hz);
    found.retain(|i| i.frames >= MIN_FRAMES);
    found
}

/// What every candidate read, most frames first and before [`MIN_FRAMES`]
/// is applied, so a caller measuring a decoder's false frames sees the ones
/// the threshold is there to swallow.
pub fn read_all(iq: &[C32], rate_hz: f64, center_hz: f64) -> Vec<Ident> {
    let mut found: Vec<Ident> = candidates(rate_hz, center_hz)
        .into_iter()
        .map(|s| {
            let read = s.read(iq, rate_hz, center_hz);
            let at = read.center_hz.unwrap_or(center_hz);
            Ident::of(s, read, at)
        })
        .collect();
    found.sort_by(|a, b| b.frames.cmp(&a.frames));
    found
}

/// The signal registered under a name, so a caller can ask for one protocol
/// rather than the whole table.
pub fn by_id(id: &str) -> Option<&'static dyn Signal> {
    all().iter().copied().find(|s| s.id() == id)
}

/// The channels of this protocol that fall inside a span.
///
/// A protocol placed on fixed channels is read on each of the ones the
/// recording covers; one placed on a band has no channel of its own to
/// offer, so the middle of the recording is the channel, which is what a
/// receiver handed a stream the detector cut also assumes.
pub fn channels_in(s: &dyn Signal, rate_hz: f64, center_hz: f64) -> Vec<f64> {
    let width = s.shape().widths.first().copied().unwrap_or(0.0);
    let (lo, hi) =
        (center_hz - rate_hz / 2.0 + width / 2.0, center_hz + rate_hz / 2.0 - width / 2.0);
    match s.placement() {
        Placement::Channels(chs) | Placement::Within(chs, _) => {
            chs.into_iter().filter(|c| (lo..=hi).contains(c)).collect()
        }
        _ => vec![center_hz],
    }
}

/// Samples handed to a demodulator at a time, the way a radio delivers them,
/// so a caller reading a whole file still crosses block boundaries the way
/// the receiver does.
pub(crate) const BLOCK: usize = 65_536;

#[cfg(test)]
mod tests {
    use super::*;

    /// A recording is only offered the protocols that could be in it.
    #[test]
    fn the_band_decides_what_is_tried() {
        let named =
            |rate, center| candidates(rate, center).iter().map(|s| s.id()).collect::<Vec<_>>();
        assert_eq!(named(2_400_000.0, 1_090_000_000.0), ["mode_s"]);
        // The sonde band holds the radiosondes keyed in it and nothing else.
        assert_eq!(named(31_250.0, 405_800_240.0), ["rs41", "dfm", "ims100", "mrz", "lms6"]);
        // 1090 MHz at a rate the correlator refuses is nobody's.
        assert_eq!(named(1_000_000.0, 1_090_000_000.0), [] as [&str; 0]);
    }

    #[test]
    fn ids_are_unique_and_answer_to_their_own_names() {
        let mut ids: Vec<&str> = all().iter().map(|s| s.id()).collect();
        let count = ids.len();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), count);
        assert_eq!(by_id("mode_s").map(|s| s.label()), Some("mode s"));
        assert_eq!(by_id("rs41").map(|s| s.label()), Some("rs41"));
        assert!(by_id("nothing anybody reads").is_none());
    }
}
