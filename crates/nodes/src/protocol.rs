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
use pipeline::port::PortKind;

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
    /// The rate to hand it when the receiver cuts a band out for it, which
    /// leaves the decoder room above its floor: four samples a symbol where
    /// three is refused, a transition band for its own filter. Zero to let
    /// the receiver choose from the width.
    pub feed_rate_hz: f64,
    /// Reads the span itself, where the span reaches its placement, rather
    /// than a source cut out of it: Mode S is shorter than a detector frame,
    /// a camera's carrier is the whole span.
    pub span_wide: bool,
    /// The modulations the burst classifier would name it. A decoder listed
    /// here is built once the classifier has named a burst on the source,
    /// and reads the samples it missed from the ring; one listing none is
    /// built the moment the source opens.
    ///
    /// For a decoder that is dear to run and whose modulation the
    /// classifier names reliably, which so far is LoRa and its chirp. It
    /// is not a general saving: measured on the off-air M17 capture, the
    /// classifier names the handheld's 4-FSK `Unknown` for the whole
    /// transmission, so a voice decoder gated on `Fsk4` would never have
    /// been built. A decoder that is cheap beside the classifier, or whose
    /// modulation the classifier is unsure of, lists nothing and is built
    /// on open.
    pub families: &'static [Modulation],
}

/// What the receiver does with a channel once a protocol has read on it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Stickiness {
    /// Nothing kept: the next transmission is found again. For a hopper, or
    /// a burst that is over before a channel is worth remembering.
    Forget,
    /// The channel is cut out for this protocol alone from then on: for the
    /// session, or until nothing has decoded on it for `hold_s` seconds. A
    /// span-wide decoder with this owns its band from the moment the span
    /// reaches it.
    Latch { hold_s: Option<f64> },
    /// The decoder says what it is reading through `Request::Claim`, and
    /// owns that once it does, and nothing before.
    Claim,
}

impl Stickiness {
    /// Kept for the session, which is what most channels want: a channel
    /// that has produced a decoded frame is one that will produce another.
    pub const SESSION: Stickiness = Stickiness::Latch { hold_s: None };
}

/// How much of the stream a span-wide decoder needs while nothing has been
/// read on it.
///
/// A source cut out of the span is only read while it is transmitting; a
/// span-wide decoder is handed every block for as long as the receiver runs,
/// whether or not anything it can read is on the air. Measured on a 5.8 GHz
/// capture, the Wi-Fi front end read 7.5 seconds of air in 3.1 seconds of
/// CPU and returned no frames at all, because there was no Wi-Fi there: four
/// seconds of that file is an empty band.
///
/// What makes duty cycling honest for one protocol and dishonest for another
/// is whether its traffic repeats. A beacon goes out ten times a second per
/// network, so a fifth of the air names every network within a second. A
/// Mode S squitter or a sensor packet happens once and is gone.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Watch {
    /// Every sample, always. What it reads happens once.
    Everything,
    /// `on_s` of every `every_s`, until it reads something, and then
    /// everything for `hold_s` after the last thing it read, so a
    /// conversation is followed rather than sampled.
    Sampled { on_s: f64, every_s: f64, hold_s: f64 },
}

/// How much to divide a span by before handing it to a span-wide decoder,
/// and the rate that leaves.
///
/// A decoder declares the rate it wants (`Shape::feed_rate_hz`) and the
/// receiver's own extraction obeys it for a front end the scanner table
/// places. The auto node used to ignore it and hand over the raw span, so a
/// Mode S correlator asking for 2.4 MS/s ran over 20 and cost 127% of a core
/// on an empty band against 37% once narrowed.
///
/// `offset_hz` is how far the band it was placed on sits from the middle of
/// the span, and it is kept rather than mixed away: what survives is wide
/// enough to still hold the band, so nothing here has to shift the signal.
/// A band far enough off centre simply is not narrowed.
///
/// Only for a decoder that asks (see [`Protocol::narrow_span`]).
pub fn span_feed(rate: f64, offset_hz: f64, shape: &Shape) -> (usize, f64) {
    if shape.feed_rate_hz <= 0.0 {
        return (1, rate);
    }
    let want = shape.feed_rate_hz + 2.0 * offset_hz.abs();
    let mut factor = 1usize;
    while rate / (factor * 2) as f64 >= want {
        factor *= 2;
    }
    (factor, rate / factor as f64)
}

/// A marker on the spectrum for a placed channel.
#[derive(Clone, Debug, PartialEq)]
pub struct Mark {
    pub hz: f64,
    pub width_hz: f64,
    pub label: String,
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

    /// The frequency to offer when somebody places it by hand: the calling
    /// channel, the one allocation, the middle of the band.
    fn default_hz(&self) -> f64 {
        self.placement().default_hz().unwrap_or(0.0)
    }

    fn shape(&self) -> Shape;

    /// What the last stage of the chain puts out, by port: frames from a
    /// decoder that produces bytes, packets from one that builds its own,
    /// speech and pictures beside them where it carries any. Written down
    /// rather than asked of a node because the receiver draws its wires
    /// from a description, before any node exists to negotiate with; the
    /// test beside the registry checks it against what the chain says.
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Frames]
    }

    /// What a stage placed at `hz` is called in the chain view.
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} {}", hz / 1e6, self.label().to_uppercase())
    }

    /// The markers the spectrum draws for a decoder placed at `hz`.
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark {
            hz,
            width_hz: self.shape().widths[0],
            label: self.label().to_uppercase(),
        }]
    }

    fn stickiness(&self) -> Stickiness {
        Stickiness::SESSION
    }

    /// Whether the receiver should cut the span down to `feed_rate_hz`
    /// before this decoder sees it.
    ///
    /// True for one that works per sample at whatever rate it is handed: a
    /// Mode S correlator, a video discriminator and sync separator. False,
    /// and by default, for one that cuts its own channels out of the span,
    /// where a filter in front is a second pass over the same samples to
    /// save a decoder that was not reading them anyway. Measured on an empty
    /// 20 MS/s span: narrowing takes Mode S from 127% of a core to 37%, and
    /// puts AIS up from 8% to 15% and BLE from 13% to 23%.
    fn narrow_span(&self) -> bool {
        false
    }

    /// How much of the span this decoder needs while it is finding nothing.
    /// Everything, unless what it reads repeats often enough that a sample
    /// of the air finds it just as surely.
    fn watch(&self) -> Watch {
        Watch::Everything
    }

    /// Of the channel widths that each read something on one source, the
    /// ones to keep. All of them unless the protocol knows better: two
    /// widths of one protocol reading the same packet is usually one of
    /// them reading half of it.
    fn resolve_widths(&self, _heard: &mut Vec<f64>) {}

    /// Whether a source at `hz`, measured `source_width_hz` wide, could be
    /// a channel of this protocol. Within reach of one of the declared
    /// widths unless the protocol says otherwise; the frequency is for a
    /// protocol keyed differently in different bands.
    fn accepts_width(&self, _hz: f64, source_width_hz: f64) -> bool {
        self.shape()
            .widths
            .iter()
            .any(|w| source_width_hz <= w * CHANNEL_WIDTH_TOLERANCE)
    }

    /// The channel widths to try on a source at `hz` measured
    /// `source_width_hz` wide. One, unless the protocol is keyed at several
    /// and the measurement sits between them.
    fn widths_for(&self, _hz: f64, _source_width_hz: f64) -> Vec<f64> {
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
        &crate::wifi_nodes::Wifi,
        &crate::droneid_nodes::DroneId,
        &crate::video_nodes::Video,
        &crate::aprs_nodes::Aprs,
        &crate::pocsag_nodes::Pocsag,
        &crate::m17_nodes::M17,
        &crate::dmr_nodes::Dmr,
        &crate::tetra_nodes::Tetra,
        &crate::gsm_nodes::Gsm,
        &crate::lora_nodes::Lora,
        &crate::elrs_nodes::Elrs,
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
            let hz = p.default_hz();
            assert!(hz > 0.0, "{} offers no frequency", p.id());
            let shape = p.shape();
            let rate = shape.min_rate_hz.max(shape.widths[0] * 4.0).max(25_000.0);
            let at = Placed {
                center_hz: hz,
                width_hz: shape.widths[0],
                rate,
                snr_db: 20.0,
            };
            let chain = p.chain(at);
            assert!(!chain.is_empty(), "{} builds no chain", p.id());
            // The declared outputs are what the built chain negotiates.
            let spec = pipeline::port::StreamSpec::iq(rate, common::Hz(hz as u64));
            let g = crate::build_chain(spec, &chain, &reg)
                .unwrap_or_else(|e| panic!("{}: {e}", p.id()));
            let (tail, _) = g.order().last().expect("a tail");
            let outs = g.node(tail).map(|n| n.num_outputs()).unwrap_or(0);
            let kinds: Vec<PortKind> =
                (0..outs).filter_map(|k| g.spec_of(tail.out(k)).map(|s| s.kind)).collect();
            assert_eq!(kinds, p.outputs(), "{}", p.id());
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
