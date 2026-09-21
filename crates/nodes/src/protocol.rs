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
use common::packet::{Packet, Proto};
use pipeline::port::PortKind;

/// Where a protocol can be and what stream it reads live below this crate,
/// in `identify`, so a program with a recording and a tuning can ask the
/// same table without the flow graph.
pub use identify::{CHANNEL_WIDTH_TOLERANCE, Placement, Shape};

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

/// What has to be transmitting before a span-wide decoder is handed a block
/// at all.
///
/// A source cut out of the span is only read while something is on the air
/// there, because the detector found it first. A span-wide decoder has no
/// such thing in front of it: it is handed every sample for as long as the
/// receiver runs, and on an empty band that is a core spent proving the band
/// is empty. Measured on the 5.8 GHz camera capture, the Wi-Fi front end
/// read 7.5 seconds of air in 3.1 seconds of CPU and returned no frames at
/// all, because there was no Wi-Fi there.
///
/// The detector is the thing in front of it. What decides whether that works
/// is how long the traffic lasts against how long the detector takes to
/// notice: a Wi-Fi frame is over 200 us and a camera's carrier is on for
/// seconds, so the detector has a source open before either decoder needs
/// one, while a Mode S reply is 120 us and would be over before anything
/// opened.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Wake {
    /// Every block, whatever the detector has found. What it reads is
    /// shorter than the detector takes to find it, or is on a channel the
    /// detector is kept out of and will therefore never open: Mode S, AIS,
    /// and Bluetooth advertising, which owns its three channels from the
    /// moment the span reaches them.
    Always,
    /// While the detector has any source open in the span, and for `hold_s`
    /// after the last one closed.
    ///
    /// Any source, rather than one as wide as the signal this decoder
    /// reads, because the detector's extent is every bin within 20 dB of a
    /// run's peak and that is far narrower than the transmission: measured,
    /// one 802.11b beacon capture opens sources of 9 kHz, 181 kHz, 571 kHz
    /// and 1.9 MHz for the same access point, and a camera's 20 MHz carrier
    /// opens runs of 55 kHz to 1.4 MHz and never one wider. A width
    /// threshold that let both of those through would let everything
    /// through.
    Detected { hold_s: f64 },
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

/// How a frame on the packet bus is recognised as this protocol's, and so
/// how specific the claim is.
///
/// Two protocols can both be able to read the same bytes, so the order they
/// are offered a frame in decides which of them gets it. This is that order,
/// written down as a property of each protocol rather than as the position
/// of an `if` in the consumer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum FrameClaim {
    /// By a tag the protocol's own front end wrote into the bytes and its
    /// decoder checks again. The most specific claim there is, and the only
    /// one open to a protocol that runs wherever somebody puts it: M17 is on
    /// the 2 m channels APRS uses and on the 70 cm ones beside the pager
    /// bands, so its frequency says nothing about it.
    Tagged,
    /// By where the frame was received, in a window `width_hz` wide. Where
    /// two windows overlap the narrower is the better claim: 144 to 146 MHz
    /// sits inside the VHF paging allocation, and the 420 to 430 MHz TETRA
    /// downlinks inside the UHF one.
    Band { width_hz: u64 },
    /// Nothing this protocol produces reaches the bus as a frame.
    Never,
}

/// A marker on the spectrum for a placed channel.
#[derive(Clone, Debug, PartialEq)]
pub struct Mark {
    pub hz: f64,
    pub width_hz: f64,
    pub label: String,
}

/// Where a stream sits in the span it was cut from, so a position in it can
/// be named to a decoder reading another stream of the same span.
///
/// A GSM carrier a beacon sent a phone to has no synchronisation burst of its
/// own, and the only clock it can be timed from is the beacon's: the beacon
/// says which span sample it last synchronised on, and the decoder on the
/// other carrier finds that sample in its own stream. Neither can do it
/// without knowing where its stream starts in the span, which only whatever
/// cut the stream out can say.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Origin {
    /// Span sample the stream's first sample was cut from.
    pub span_sample: u64,
    pub span_rate_hz: f64,
}

impl Origin {
    /// The span sample `pos` samples into a stream running at `rate`.
    pub fn span_sample_at(&self, pos: f64, rate: f64) -> f64 {
        self.span_sample as f64 + pos * self.span_rate_hz / rate.max(1.0)
    }

    /// The other way about: how far into a stream running at `rate` the span
    /// sample `span` falls.
    pub fn stream_pos_at(&self, span: f64, rate: f64) -> f64 {
        (span - self.span_sample as f64) * rate / self.span_rate_hz.max(1.0)
    }

    /// The origin of a stream that starts `pos` samples into this one: what a
    /// decoder placed later, on the history the stream kept, is reading from.
    pub fn advanced(&self, pos: u64, rate: f64) -> Self {
        Self {
            span_sample: self.span_sample_at(pos as f64, rate).max(0.0) as u64,
            span_rate_hz: self.span_rate_hz,
        }
    }

    /// Put it where the stage that needs it will find it. The registry builds
    /// stages from named settings, so this is where the two names are spelled
    /// and [`Origin::read`] is the only place they are read.
    pub fn write(&self, s: &mut pipeline::registry::Settings) {
        s.insert(
            "span_origin_sample".into(),
            pipeline::param::ParamValue::Float(self.span_sample as f64),
        );
        s.insert("span_rate_hz".into(), pipeline::param::ParamValue::Float(self.span_rate_hz));
    }

    pub fn read(s: &pipeline::registry::Settings) -> Option<Self> {
        use pipeline::registry::SettingsExt;
        let sample = s.get("span_origin_sample")?.as_f64()?;
        Some(Self {
            span_sample: sample.max(0.0) as u64,
            span_rate_hz: s.f64_or("span_rate_hz", 0.0),
        })
    }
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
    /// Where the stream it will read sits in the span, where whatever cut it
    /// out knows: a decoder timed from another carrier's needs it, and one
    /// placed on the span itself has no span position to be told.
    pub origin: Option<Origin>,
}

/// The widest source the burst router is placed on, in hertz.
///
/// The router is a decoder like every other and is placed where its width
/// fits, except that its shape is decided by what reads its output rather
/// than by a channel of its own: the pulse front ends inside it read sensor
/// channels up to [`dsp::route::MAX_PULSE_CHANNEL_HZ`], and the protocols
/// that wait for one of its verdicts ([`Shape::families`], so far LoRa and
/// ExpressLRS) read channels of their declared widths. So it is asked of the
/// registry, like everything else the auto node wants to know.
///
/// Above this nothing consumes the verdict. Measured on the 2.4 GHz DroneID
/// capture, the detector opens a 20 MHz source for the Wi-Fi in the band and
/// the router then ran its per-sample gate over the whole of it and
/// classified every Wi-Fi frame at 3 to 6 ms each: 100 of the 147 million
/// samples every router saw were that one source, and no front end could
/// read a burst of it. A source wider than this leaves the detector's own
/// measurement as its evidence row instead; see [`super::auto`].
pub fn router_max_width_hz() -> f64 {
    let widest = all()
        .iter()
        .filter(|p| !p.shape().families.is_empty())
        .flat_map(|p| p.shape().widths.iter().copied())
        .fold(dsp::route::MAX_PULSE_CHANNEL_HZ, f64::max);
    widest * CHANNEL_WIDTH_TOLERANCE
}

pub trait Protocol: Send + Sync {
    /// The stage registry's name for the decoder, and the word a table or a
    /// saved channel names it by.
    fn id(&self) -> &'static str;

    /// What it is called where a person reads it.
    fn label(&self) -> &'static str;

    /// Other words a scanner table or a saved channel may name it by, beside
    /// its id and its label. Here rather than in a table beside the parser,
    /// so a protocol arrives with every word it answers to.
    fn aliases(&self) -> &'static [&'static str] {
        &[]
    }

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
        &[PortKind::Packets]
    }

    /// What a stage placed at `hz` is called in the chain view.
    fn stage_label(&self, hz: f64) -> String {
        format!("{:.4} {}", hz / 1e6, self.label().to_uppercase())
    }

    /// The markers the spectrum draws for a decoder placed at `hz`.
    fn marks(&self, hz: f64) -> Vec<Mark> {
        vec![Mark { hz, width_hz: self.shape().widths[0], label: self.label().to_uppercase() }]
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

    /// What has to be on the air before this decoder is handed a block.
    ///
    /// Only asked of a span-wide decoder: everything else is placed on a
    /// source and so is gated on the detector already. [`Wake::Always`]
    /// unless what it reads lasts longer than the detector takes to find it.
    fn wakes_on(&self) -> Wake {
        Wake::Always
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
        self.shape().widths.iter().any(|w| source_width_hz <= w * CHANNEL_WIDTH_TOLERANCE)
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

    /// The one stage that reads this protocol off audio a listening channel
    /// has already demodulated, or `None` for a protocol that needs the
    /// samples themselves.
    ///
    /// `hz` is where the channel is tuned, and is carried only so a row says
    /// where it was heard: the mixing and the discrimination happened in the
    /// channel, and nothing is left for the stage to tune. A relayed alert, a
    /// picture off a sideband channel and a satellite pass somebody is
    /// already listening to are all read this way, without a second front end
    /// cutting the same channel out of the span again.
    fn audio_stage(&self, _hz: f64) -> Option<NodeSpec> {
        None
    }

    /// The stages that transmit this protocol: what supplies the payload and
    /// what modulates it, the transmit chain's mirror of [`Protocol::chain`].
    ///
    /// `None` for a protocol nothing can key up yet, which is most of them.
    /// Refusing is the point: the alternative is a transmitter keyed in a
    /// mode the other end cannot read.
    fn transmit(&self) -> Option<TxChain> {
        None
    }

    /// Whether what this protocol reads says where the transmitter was, so
    /// the tracker is worth attaching to the bus: a squitter carrying an
    /// aircraft's own position, a vessel's, a station's beacon.
    fn reports_position(&self) -> bool {
        false
    }

    /// How a frame off the packet bus is recognised as this protocol's.
    fn frame_claim(&self) -> FrameClaim {
        FrameClaim::Never
    }

    /// The rows a frame off the packet bus becomes.
    ///
    /// `None` where the frame is not this protocol's, so the next protocol
    /// is offered it. `Some` of nothing where it was this protocol's and did
    /// not read: a frame from a TETRA downlink is a TETRA frame whatever its
    /// bytes turn out to be, and handing it on would only invite a weaker
    /// parser to guess at it.
    ///
    /// Several rows where one transmission carries several messages: a pager
    /// empties its queue in one go, and a GSM paging request names up to four
    /// handsets.
    fn stated(&self, _p: &Packet) -> Option<Vec<Proto>> {
        None
    }

    /// The identity of a packet that is the same news each time it repeats,
    /// so a cell's broadcast is logged once per channel rather than once a
    /// frame. None for a packet that is always news.
    fn dedupe_key(&self, _p: &Packet) -> Option<Vec<u8>> {
        None
    }
}

/// What a protocol puts on the air, as the two stages a transmit chain draws
/// between the clock and the radio.
#[derive(Clone, Debug)]
pub struct TxChain {
    /// Where the payload comes from: a file, a keyer, a microphone.
    pub source: NodeSpec,
    /// What turns it into samples.
    pub modulator: NodeSpec,
}

/// Every protocol compiled into this build, and every description with a
/// radio installed over it.
///
/// The compiled set is static. The described set changes when a dataset
/// lands or a file under the config directory is edited, so the list is
/// rebuilt when `decode::script::generation` moves and leaked, which keeps
/// every reference handed out before still good: a description set changes
/// a handful of times in a run and each leak is a few hundred bytes.
pub fn all() -> &'static [&'static dyn Protocol] {
    static BUILT: std::sync::RwLock<Option<(u64, &'static [&'static dyn Protocol])>> =
        std::sync::RwLock::new(None);
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let generation = decode::script::generation();
    if let Some((g, list)) = *BUILT.read().unwrap_or_else(|e| e.into_inner())
        && g == generation
    {
        return list;
    }
    let _building = LOCK.lock().unwrap_or_else(|e| e.into_inner());
    if let Some((g, list)) = *BUILT.read().unwrap_or_else(|e| e.into_inner())
        && g == generation
    {
        return list;
    }
    let mut v: Vec<&'static dyn Protocol> = compiled().to_vec();
    for p in crate::script_nodes::protocols() {
        let leaked: &'static dyn Protocol = Box::leak(Box::new(p));
        v.push(leaked);
    }
    let list: &'static [&'static dyn Protocol] = Box::leak(v.into_boxed_slice());
    *BUILT.write().unwrap_or_else(|e| e.into_inner()) = Some((generation, list));
    list
}

/// The protocols written in this tree.
fn compiled() -> &'static [&'static dyn Protocol] {
    static ALL: &[&dyn Protocol] = &[
        &crate::modes_nodes::ModeS,
        &crate::ais_nodes::Ais,
        &crate::uat_nodes::Uat,
        &crate::ble_nodes::Ble,
        &crate::ieee802154_nodes::Ieee802154,
        &crate::wifi_nodes::Wifi,
        &crate::droneid_nodes::DroneId,
        &crate::video_nodes::Video,
        &crate::acars_nodes::Acars,
        &crate::sstv_nodes::Sstv,
        &crate::apt_nodes::Apt,
        &crate::lrpt_nodes::Lrpt,
        &crate::wefax_nodes::Wefax,
        &crate::vdl2_nodes::Vdl2,
        &crate::dvbt_nodes::Dvbt,
        &crate::dab_nodes::DabProtocol,
        &crate::drm_nodes::DrmProtocol,
        &crate::aprs_nodes::Aprs,
        &crate::pocsag_nodes::Pocsag,
        &crate::flex_nodes::Flex,
        &crate::rtty_nodes::Rtty,
        &crate::ft8_nodes::Ft8,
        &crate::ft8_nodes::Ft4,
        &crate::morse_nodes::Morse,
        &crate::mdc_nodes::Mdc,
        &crate::eas_nodes::Eas,
        &crate::twotone_nodes::TwoTone,
        &crate::nrf24_nodes::Nrf24,
        &crate::m17_nodes::M17,
        &crate::dmr_nodes::Dmr,
        &crate::p25_nodes::P25,
        &crate::nxdn_nodes::Nxdn,
        &crate::tetra_nodes::Tetra,
        &crate::gsm_nodes::Gsm,
        &crate::lora_nodes::Lora,
        &crate::elrs_nodes::Elrs,
        &crate::wmbus_nodes::Wmbus,
        &crate::zwave_nodes::ZWave,
        &crate::rs41_nodes::Rs41,
        &crate::dfm_nodes::Dfm,
        &crate::m10_nodes::M10,
        &crate::imet_nodes::Imet,
        &crate::meisei_nodes::Meisei,
        &crate::mrz_nodes::Mrz,
        &crate::lms6_nodes::Lms6,
        &crate::epirb_nodes::Epirb,
        &crate::stdc_nodes::Stdc,
        &crate::aero_nodes::Aero,
        &crate::iridium_nodes::Iridium,
    ];
    ALL
}

/// Every protocol, in the order a frame on the bus is offered to them:
/// the most specific claim first.
pub fn frame_readers() -> Vec<&'static dyn Protocol> {
    let mut ps: Vec<&'static dyn Protocol> = all().to_vec();
    ps.sort_by_key(|p| p.frame_claim());
    ps
}

/// The protocol registered under a name.
pub fn by_id(id: &str) -> Option<&'static dyn Protocol> {
    all().iter().copied().find(|p| p.id() == id)
}

/// The protocol a word names: its registry id, the label a person reads, or
/// one of the words it also answers to.
///
/// The one parse, so a scanner block and a saved channel mode cannot
/// disagree about what `adsb` means.
pub fn by_word(word: &str) -> Option<&'static dyn Protocol> {
    let w = word.trim();
    all().iter().copied().find(|p| {
        p.id().eq_ignore_ascii_case(w)
            || p.label().eq_ignore_ascii_case(w)
            || p.aliases().iter().any(|a| a.eq_ignore_ascii_case(w))
    })
}

/// The protocols that read one channel of a declared width, which is what
/// a strip channel or a scanner block at a frequency can be given.
pub fn channel_protocols() -> impl Iterator<Item = &'static dyn Protocol> {
    all().iter().copied().filter(|p| !p.shape().span_wide)
}

/// The protocols a listening channel's audio can be handed to, in the order
/// the registry lists them.
pub fn audio_readers() -> Vec<&'static dyn Protocol> {
    all().iter().copied().filter(|p| p.audio_stage(p.default_hz()).is_some()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// No narrowband land mobile or amateur decoder is placed in the 2.4 GHz
    /// band.
    ///
    /// Every narrow source a busy 2.4 GHz band opens used to get M17, DMR,
    /// P25, POCSAG, FLEX, MDC-1200, two-tone, APRS and SSTV built on it,
    /// about 4 ms of processor time in a 2.13 ms block, and every row they
    /// produced was thrown away again by the band test in `read`.
    #[test]
    fn the_24_ghz_band_gets_none_of_the_land_mobile_decoders() {
        for id in [
            "m17", "dmr", "p25", "pocsag", "flex", "mdc1200", "twotone", "aprs", "sstv", "rtty",
            "morse", "wefax",
        ] {
            let p = by_id(id).unwrap_or_else(|| panic!("{id} is registered"));
            for hz in [2_402e6, 2_431e6, 2_450e6, 2_480e6, 5_800e6] {
                assert!(
                    !p.placement().covers(hz, p.shape().widths[0]),
                    "{id} is placed at {:.0} MHz",
                    hz / 1e6
                );
            }
        }
    }

    /// And the frequencies the corpus was recorded on still get theirs: a
    /// handheld in the 433 MHz licence-free band is where most of this
    /// traffic actually is, so licence-free is not the same answer as
    /// 2.4 GHz.
    #[test]
    fn the_captures_still_reach_the_decoder_that_reads_them() {
        for (id, hz) in [
            ("dmr", 433.45e6),
            ("m17", 434.02e6),
            ("pocsag", 433.92e6),
            ("lora", 869.525e6),
            ("aprs", 144.8e6),
        ] {
            let p = by_id(id).unwrap_or_else(|| panic!("{id} is registered"));
            assert!(
                p.placement().covers(hz, p.shape().widths[0]),
                "{id} is not placed at {:.3} MHz",
                hz / 1e6
            );
        }
    }

    /// Where a protocol says it can be, the ribbon names it.
    ///
    /// Europe had nothing at all between PMR446 and LTE 800, so a receiver
    /// tuned to a television multiplex said UNALLOCATED and offered narrow
    /// FM. The decoder knew the band the whole time.
    #[test]
    fn a_television_multiplex_is_named_where_the_decoder_says_it_is() {
        for (lo, hi) in by_id("dvbt").expect("dvbt").placement().bands(8.0e6) {
            for hz in [lo + 1e6, (lo + hi) / 2.0, hi - 1e6] {
                let name = common::bands::name_at_in(common::bands::Plan::Europe, hz);
                assert!(
                    matches!(name, "UHF TV" | "DAB / Band III"),
                    "{:.1} MHz is {name}",
                    hz / 1e6
                );
            }
        }
        // Channel 21 is the bottom of the UHF plan and every channel above
        // it is 8 MHz on: 429 is the capture's own frequency, not a
        // broadcast one, so it stays unallocated.
        assert_eq!(common::bands::snap_in(common::bands::Plan::Europe, 475.3e6), 474.0e6);
        assert_eq!(common::bands::snap_in(common::bands::Plan::Europe, 601.0e6), 602.0e6);
        // A dish, once the LNB's oscillator is set as the offset: the dial
        // reads what came out of the sky rather than what is on the cable.
        assert_eq!(
            common::bands::name_at_in(common::bands::Plan::Europe, 11.778e9),
            "Satellite TV (Ku)"
        );
        assert_eq!(
            common::bands::name_at_in(common::bands::Plan::Americas, 12.2e9),
            "Satellite TV (Ku)"
        );
    }

    #[test]
    fn the_narrowest_band_wins_when_they_overlap() {
        // 433.92 is inside both the 70 cm amateur band and ISM 433; the ISM
        // allocation is the more useful label and the narrower entry.
        assert_eq!(common::bands::name_at_in(common::bands::Plan::Europe, 433.92e6), "ISM 433");
        // Same for the transponder frequencies inside the DME allocation.
        assert_eq!(common::bands::name_at_in(common::bands::Plan::Europe, 1090.0e6), "ADS-B");
        assert_eq!(
            common::bands::name_at_in(common::bands::Plan::Europe, 1030.05e6),
            "SSR interrogation"
        );
        assert_eq!(common::bands::name_at_in(common::bands::Plan::Europe, 1000.0e6), "DME / TACAN");
    }

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
                origin: None,
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

    /// Every protocol that says it transmits names stages that exist and
    /// negotiate from the transmit clock to samples for the radio.
    ///
    /// The chain view, the mode menu and the radio all take a protocol at
    /// its word here, and a transmit chain is only built when a key goes
    /// down, so a name that is not in the registry would be a panic at the
    /// worst moment rather than a refusal.
    #[test]
    fn every_transmit_chain_builds_and_ends_in_samples() {
        let reg = crate::registry();
        let mut keyed = Vec::new();
        for p in all() {
            let Some(tx) = p.transmit() else { continue };
            keyed.push(p.id());
            let rate = p.shape().min_rate_hz.max(48_000.0);
            let clock = pipeline::port::StreamSpec {
                kind: PortKind::Real,
                rate,
                center: common::Hz(p.default_hz() as u64),
                channels: 1,
                flow: pipeline::port::Flow::Tx,
                domain: pipeline::port::Domain::Baseband,
                ..Default::default()
            };
            let g = crate::build_chain(clock, &[tx.source, tx.modulator], &reg)
                .unwrap_or_else(|e| panic!("{}: {e}", p.id()));
            let (tail, _) = g.order().last().expect("a tail");
            let out = g.spec_of(tail.out(0)).expect("an output");
            assert_eq!(out.kind, PortKind::Iq, "{} does not end in samples", p.id());
            assert_eq!(out.rate, rate, "{} transmits at the wrong rate", p.id());
            assert_eq!(out.flow, pipeline::port::Flow::Tx, "{} is not a transmission", p.id());
        }
        assert_eq!(
            keyed,
            ["ble", "sstv", "dvbt", "aprs", "pocsag", "rtty"],
            "what this build can key up"
        );
    }

    /// Every protocol that says a listening channel's audio is enough for it
    /// names a stage that takes audio at the rate a strip channel produces
    /// and puts out what the protocol declared.
    ///
    /// The strip takes a protocol at its word here: a stage that refused the
    /// port would be a channel wired to nothing rather than a refusal the
    /// operator could see.
    #[test]
    fn every_audio_reader_negotiates_a_channel_of_audio() {
        let reg = crate::registry();
        let mut read = Vec::new();
        for p in audio_readers() {
            read.push(p.id());
            let hz = p.default_hz();
            let spec = pipeline::port::StreamSpec {
                kind: PortKind::Real,
                rate: 48_000.0,
                center: common::Hz(hz as u64),
                channels: 1,
                ..Default::default()
            };
            let stage = p.audio_stage(hz).expect("an audio stage");
            let g = crate::build_chain(spec, std::slice::from_ref(&stage), &reg)
                .unwrap_or_else(|e| panic!("{}: {e}", p.id()));
            let (tail, _) = g.order().last().expect("a tail");
            let outs = g.node(tail).map(|n| n.num_outputs()).unwrap_or(0);
            let kinds: Vec<PortKind> =
                (0..outs).filter_map(|k| g.spec_of(tail.out(k)).map(|s| s.kind)).collect();
            assert_eq!(kinds, p.outputs(), "{}", p.id());
            let out = g.spec_of(tail.out(0)).expect("an output");
            assert_eq!(out.center, common::Hz(hz as u64), "{} loses the dial", p.id());
        }
        assert_eq!(read, ["sstv", "apt", "wefax", "eas"], "what a channel's audio can be given to");
    }

    /// Nothing is left out of the frame walk, and everything whose spectrum
    /// the pager bands swallow is offered a frame before the pager: 144 to
    /// 146 MHz sits inside the VHF paging allocation, the 420 to 430 MHz
    /// TETRA downlinks inside the UHF one, and a protocol recognised by a tag
    /// can be anywhere at all.
    #[test]
    fn every_protocol_is_offered_a_frame_and_the_pager_is_offered_it_late() {
        let order = frame_readers();
        assert_eq!(order.len(), all().len());
        let at = |id: &str| order.iter().position(|p| p.id() == id).expect(id);
        let pocsag = at("pocsag");
        for id in ["m17", "dmr", "lora", "elrs", "aprs", "tetra"] {
            assert!(at(id) < pocsag, "{id} is offered a frame after the pager");
        }
    }

    /// What a protocol says it puts out and what its stage says about the
    /// packet bus are one fact written twice, and a front end that produced
    /// packets nothing wired to the bus would decode into silence.
    #[test]
    fn a_stage_feeds_the_bus_exactly_when_its_protocol_produces_frames() {
        let reg = crate::registry();
        for p in all() {
            let desc = reg.desc(p.id()).unwrap_or_else(|| panic!("{} has no stage", p.id()));
            let produces =
                matches!(p.outputs().first(), Some(PortKind::Packets | PortKind::Packets));
            assert_eq!(desc.feeds_bus, produces, "{}", p.id());
        }
    }

    /// One parse for a word naming a protocol, whichever of its names was
    /// written: the scanner table and a saved channel mode both come through
    /// here, and they used to disagree about what `adsb` meant.
    #[test]
    fn a_protocol_answers_to_its_id_its_label_and_its_aliases() {
        let id = |word| by_word(word).map(|p| p.id());
        assert_eq!(id("mode_s"), Some("mode_s"), "its registry id");
        assert_eq!(id("mode s"), Some("mode_s"), "its label");
        assert_eq!(id("ADSB"), Some("mode_s"), "an alias, in any case");
        assert_eq!(id("pager"), Some("pocsag"), "the pager is labelled one");
        assert_eq!(id("bluetooth"), Some("ble"));
        assert_eq!(id("gsm-sch"), Some("gsm"));
        assert_eq!(id("nothing anybody reads"), None);
        // Every alias names the protocol that claims it, and no two claim
        // the same word.
        let mut words: Vec<&str> = all().iter().flat_map(|p| p.aliases().iter().copied()).collect();
        let count = words.len();
        words.sort();
        words.dedup();
        assert_eq!(words.len(), count, "two protocols answer to one word");
    }

    /// Every protocol compiled in is described once, below the graph.
    ///
    /// The receiver and anything naming a recording have to agree about
    /// where a protocol is and what stream it reads, or the second is no
    /// longer evidence about the first. They agree by both asking
    /// `identify`, and this checks that none has been added to one and not
    /// the other.
    #[test]
    fn every_protocol_is_described_once_below_the_graph() {
        use identify::Signal;
        let mut theirs: Vec<&str> = identify::all().iter().map(|s| s.id()).collect();
        let mut ours: Vec<&str> = compiled().iter().map(|p| p.id()).collect();
        theirs.sort();
        ours.sort();
        assert_eq!(ours, theirs, "a protocol is described in one place and not the other");
        for s in identify::all() {
            let p = by_id(s.id()).unwrap_or_else(|| panic!("{} is registered", s.id()));
            assert_eq!(p.placement(), s.placement(), "{}", s.id());
            assert_eq!(p.shape(), s.shape(), "{}", s.id());
            assert_eq!(p.label(), s.label(), "{}", s.id());
            assert_eq!(p.aliases(), s.aliases(), "{}", s.id());
            assert_eq!(p.default_hz(), s.default_hz(), "{}", s.id());
        }
    }

    #[test]
    fn ids_are_unique() {
        let mut ids: Vec<&str> = all().iter().map(|p| p.id()).collect();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), all().len());
    }

    /// The burst router is placed by what reads it, and both its consumers
    /// are in the registry: a source wider than the widest of them holds
    /// nothing either could read, and used to cost a per-sample gate over a
    /// whole 20 MHz span.
    #[test]
    fn the_burst_router_is_as_wide_as_what_reads_it_and_no_wider() {
        let max = router_max_width_hz();
        let widest = all()
            .iter()
            .filter(|p| !p.shape().families.is_empty())
            .flat_map(|p| p.shape().widths.iter().copied())
            .fold(0.0, f64::max);
        assert!(widest > 0.0, "nothing waits for a verdict any more");
        assert!(max >= widest, "a chirp channel of {widest} Hz gets no verdict");
        assert!(
            max >= dsp::route::MAX_PULSE_CHANNEL_HZ,
            "a sensor channel gets no pulse front end"
        );
        // And well under a span: this is the whole saving.
        assert!(max < 5e6, "{max} Hz is most of a 20 MHz span");
    }
}
