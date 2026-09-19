//! Where a protocol can be and what stream its decoder reads.
//!
//! Below the graph on purpose: this is the table that says a 1090 MHz
//! recording is worth offering to Mode S and a 405 MHz one is not, and
//! anything holding a capture and a tuning can ask it.

use dsp::Modulation;

/// Where in the spectrum a protocol's transmitters can be.
#[derive(Clone, Debug, PartialEq)]
pub enum Placement {
    /// Wherever the band plan says that service is: a pager is on the utility
    /// allocations, M17 on the amateur ones, LoRa on the licence-free ones.
    ///
    /// Naming the service rather than the megahertz is what keeps a decoder
    /// off a band it was never on. Before this, every narrow source a busy
    /// 2.4 GHz band opened got M17, DMR, P25, POCSAG, FLEX, MDC-1200,
    /// two-tone, APRS and SSTV built on it, which measured about 4 ms of a
    /// 2.13 ms block. It also follows the regional plan, which 902 to
    /// 928 MHz is the reason for: licence-free in the Americas and the GSM
    /// uplink in Europe.
    ///
    /// Auto mode is what reads this. An operator who wants a decoder
    /// somewhere else adds the channel by hand and names the protocol, and
    /// no table is consulted.
    Usage(&'static [common::bands::Usage]),
    /// Inside licensed allocations, in absolute hertz: the TETRA downlinks,
    /// the GSM downlinks. Knowledge about the world the plan does not carry.
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
            Placement::Usage(u) => common::bands::at(hz).is_some_and(|b| u.contains(&b.usage)),
            Placement::Bands(bands) => bands.iter().any(|(lo, hi)| (*lo..*hi).contains(&hz)),
            Placement::Channels(chs) => chs.iter().any(|c| (c - hz).abs() <= width_hz / 2.0),
        }
    }

    /// Whether any of this placement falls inside the span `lo_hz` to
    /// `hi_hz`, which is the question a whole recording asks: a transmitter
    /// anywhere in the span could be this protocol, and the middle of the
    /// span says nothing on its own.
    pub fn reaches(&self, lo_hz: f64, hi_hz: f64, width_hz: f64) -> bool {
        self.bands(width_hz).iter().any(|(lo, hi)| *lo < hi_hz && *hi > lo_hz)
    }

    /// The bands a span-wide decoder is placed on, each with the width it
    /// owns: one per channel, or the band itself.
    pub fn bands(&self, width_hz: f64) -> Vec<(f64, f64)> {
        match self {
            Placement::Usage(u) => common::bands::ranges_for(u),
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
            // A service is not a frequency, so a protocol placed by one says
            // where to put a hand-placed channel itself.
            Placement::Usage(_) => None,
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

/// How much wider than its declared channel a source may measure and still
/// have that channel's decoder placed on it. A clean channel measures a
/// little over its width (an M17 12.5 kHz channel lands around 25 kHz once
/// extracted); splatter and a nearby spur can measure it far wider, and past
/// this a 12.5 kHz decoder does not belong on the signal. Three times keeps
/// the old ~40 kHz ceiling for a 12.5 kHz channel while scaling with width.
pub const CHANNEL_WIDTH_TOLERANCE: f64 = 3.0;

#[cfg(test)]
mod tests {
    use super::*;

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

    /// A span reaches a placement its middle is nowhere near: 2 MS/s at
    /// 1089 MHz still holds 1090.
    #[test]
    fn a_span_reaches_a_placement_its_middle_misses() {
        let c = Placement::Channels(vec![1090e6]);
        assert!(!c.covers(1089e6, 100e3));
        assert!(c.reaches(1088e6, 1090e6, 100e3));
        assert!(!c.reaches(1080e6, 1085e6, 100e3));
        let b = Placement::Bands(vec![(400e6, 406e6)]);
        assert!(b.reaches(405.79e6, 405.82e6, 10e3));
        assert!(!b.reaches(406.1e6, 406.2e6, 10e3));
    }
}
