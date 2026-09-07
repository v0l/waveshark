//! What the receiver knows about each protocol it can read, in one place.
//!
//! A protocol is a description: where in the spectrum it lives, what shape
//! of stream its decoder reads, what the receiver should do with a channel
//! once it has read there, and the chain of stages that reads it. The auto
//! node asks these questions of every registered protocol when it finds a
//! source; the scanner table asks them to place a decoder by name; the strip
//! asks them to offer a channel a mode. None of them keeps a table of its
//! own, so a protocol added here is found, placeable and selectable without
//! any of them being touched.
//!
//! The stage registry still builds nodes by name. A protocol refers to stage
//! names and says how to wire them; it is the layer above.

use crate::NodeSpec;
use common::Packet;
use dsp::Modulation;

/// Where in the spectrum a protocol's transmitters can be.
#[derive(Clone, Debug, PartialEq)]
pub enum Placement {
    /// Wherever something the right shape is found: a pager channel is a
    /// pager channel at 153 MHz and at 440 MHz.
    Anywhere,
    /// Inside licensed allocations, in absolute hertz: the TETRA downlinks,
    /// the GSM downlinks. Knowledge about the world rather than this radio.
    Bands(Vec<(f64, f64)>),
    /// On fixed frequencies the standard put it on: 1090 MHz, the two AIS
    /// channels, the three BLE advertising channels.
    Channels(Vec<f64>),
}

impl Placement {
    /// Whether a transmitter at `hz` could be this protocol. A channel
    /// placement counts within half its width.
    pub fn covers(&self, hz: f64, width_hz: f64) -> bool {
        match self {
            Placement::Anywhere => true,
            Placement::Bands(bands) => bands.iter().any(|(lo, hi)| (*lo..*hi).contains(&hz)),
            Placement::Channels(chs) => chs.iter().any(|c| (c - hz).abs() <= width_hz / 2.0),
        }
    }

    /// The bands a span-wide decoder is placed on, each with the width it
    /// owns: one per channel, or the band itself.
    pub fn bands(&self, width_hz: f64) -> Vec<(f64, f64)> {
        match self {
            Placement::Anywhere => Vec::new(),
            Placement::Bands(bands) => bands.clone(),
            Placement::Channels(chs) => {
                chs.iter().map(|c| (c - width_hz / 2.0, c + width_hz / 2.0)).collect()
            }
        }
    }

    /// Whether this protocol has a frequency of its own to offer, the way a
    /// strip channel picked from a menu wants one.
    pub fn default_hz(&self) -> Option<f64> {
        match self {
            Placement::Anywhere => None,
            Placement::Bands(b) => b.first().map(|(lo, hi)| (lo + hi) / 2.0),
            Placement::Channels(c) => c.first().copied(),
        }
    }
}

/// The stream a protocol's decoder reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shape {
    /// The channel widths it is keyed at, in hertz. For a span-wide decoder,
    /// the width it owns around each placement.
    pub widths: &'static [f64],
    /// The slowest stream it will accept.
    pub min_rate_hz: f64,
    /// Reads the span itself, where the span reaches its placement, rather
    /// than a source cut out of it: Mode S is shorter than a detector frame,
    /// a camera's carrier is the whole span.
    pub span_wide: bool,
    /// The modulations the burst classifier would name it. A decoder listed
    /// here is built once the classifier has named a burst on the source,
    /// and reads the samples it missed from the ring; one listing none is
    /// built the moment the source opens.
    pub families: &'static [Modulation],
}

/// What the receiver does with a channel once a protocol has read on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stickiness {
    /// Nothing kept: the next transmission is found again. For a hopper, or
    /// a burst that is over before a channel is worth remembering.
    Forget,
    /// The channel is cut out for this protocol alone from then on, for the
    /// session. A span-wide decoder with this owns its band from the moment
    /// the span reaches it.
    Latch,
    /// The decoder says what it is reading through `Node::claimed_hz`, and
    /// owns that once it does, and nothing before.
    Claim,
}

/// A channel a decoder is being built for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placed {
    pub center_hz: f64,
    pub width_hz: f64,
    /// The rate of the stream it will be handed.
    pub rate: f64,
    /// How strong the detector found it, or NaN where nothing measured.
    pub snr_db: f32,
}

/// How much wider than its declared channel a source may measure and still
/// have that channel's decoder placed on it. A clean channel measures a
/// little over its width (an M17 12.5 kHz channel lands around 25 kHz once
/// extracted); splatter and a nearby spur can measure it far wider, and past
/// this a 12.5 kHz decoder does not belong on the signal. Three times keeps
/// the old ~40 kHz ceiling for a 12.5 kHz channel while scaling with width.
pub const CHANNEL_WIDTH_TOLERANCE: f64 = 3.0;

pub trait Protocol: Send + Sync {
    /// The stage registry's name for the decoder, and the word a table or a
    /// saved channel names it by.
    fn id(&self) -> &'static str;

    /// What it is called where a person reads it.
    fn label(&self) -> &'static str;

    fn placement(&self) -> Placement;

    fn shape(&self) -> Shape;

    fn stickiness(&self) -> Stickiness {
        Stickiness::Latch
    }

    /// Whether a source measured this wide could be a channel of this
    /// protocol. Within reach of one of the declared widths unless the
    /// protocol says otherwise.
    fn accepts_width(&self, source_width_hz: f64) -> bool {
        self.shape()
            .widths
            .iter()
            .any(|w| source_width_hz <= w * CHANNEL_WIDTH_TOLERANCE)
    }

    /// The channel widths to try on a source measured this wide. One,
    /// unless the protocol is keyed at several and the measurement sits
    /// between them.
    fn widths_for(&self, _source_width_hz: f64) -> Vec<f64> {
        self.shape().widths.first().copied().into_iter().collect()
    }

    /// The stages that read a placed channel, in order, given the stream
    /// described by `at`. Data rather than nodes, so a strip channel and a
    /// found source draw the same chain.
    fn chain(&self, at: Placed) -> Vec<NodeSpec>;

    /// The identity of a packet that is the same news each time it repeats,
    /// so a cell's broadcast is logged once per channel rather than once a
    /// frame. None for a packet that is always news.
    fn dedupe_key(&self, _p: &Packet) -> Option<Vec<u8>> {
        None
    }
}

/// Every protocol compiled into this build.
pub fn all() -> &'static [&'static dyn Protocol] {
    static ALL: &[&dyn Protocol] = &[
        &crate::modes_nodes::ModeS,
        &crate::ais_nodes::Ais,
        &crate::ble_nodes::Ble,
        &crate::video_nodes::Video,
        &crate::aprs_nodes::Aprs,
        &crate::pocsag_nodes::Pocsag,
        &crate::m17_nodes::M17,
        &crate::dmr_nodes::Dmr,
        &crate::tetra_nodes::Tetra,
        &crate::gsm_nodes::Gsm,
        &crate::lora_nodes::Lora,
        &crate::wmbus_nodes::Wmbus,
    ];
    ALL
}

/// The protocol registered under a name.
pub fn by_id(id: &str) -> Option<&'static dyn Protocol> {
    all().iter().copied().find(|p| p.id() == id)
}

/// The protocols that read one channel of a declared width, which is what
/// a strip channel or a scanner block at a frequency can be given.
pub fn channel_protocols() -> impl Iterator<Item = &'static dyn Protocol> {
    all().iter().copied().filter(|p| !p.shape().span_wide)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_protocol_names_a_registered_stage() {
        let reg = crate::registry();
        for p in all() {
            assert!(reg.contains(p.id()), "{} has no stage", p.id());
            let at = Placed {
                center_hz: p.placement().default_hz().unwrap_or(433_000_000.0),
                width_hz: p.shape().widths[0],
                rate: p.shape().min_rate_hz.max(p.shape().widths[0] * 2.0),
                snr_db: 20.0,
            };
            let chain = p.chain(at);
            assert!(!chain.is_empty(), "{} builds no chain", p.id());
            for s in &chain {
                assert!(reg.contains(&s.kind), "{}: no stage {}", p.id(), s.kind);
            }
        }
    }

    #[test]
    fn ids_are_unique() {
        let mut ids: Vec<&str> = all().iter().map(|p| p.id()).collect();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), all().len());
    }

    #[test]
    fn a_band_placement_covers_its_band_and_nothing_else() {
        let p = Placement::Bands(vec![(390e6, 400e6)]);
        assert!(p.covers(391e6, 25e3));
        assert!(!p.covers(401e6, 25e3));
        let c = Placement::Channels(vec![1090e6]);
        assert!(c.covers(1090.5e6, 2e6));
        assert!(!c.covers(1092e6, 2e6));
        assert_eq!(c.bands(2e6), vec![(1089e6, 1091e6)]);
    }
}
