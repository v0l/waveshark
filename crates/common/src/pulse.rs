//! Pulse-train vocabulary.
//!
//! These types live in `common` rather than `dsp` because they are carried on
//! graph edges, and the graph must not depend on the DSP implementation that
//! happens to produce them. The detector lives in `dsp`; the shape of what it
//! emits belongs to everybody.

use crate::{Decoded, C32};

/// One mark/gap pair, in microseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pulse {
    /// Carrier present, in microseconds.
    pub mark: u32,
    /// Carrier absent, in microseconds. The final gap of a package is the
    /// timeout that ended it and carries no information.
    pub gap: u32,
}

/// A complete burst: the pulses between two long silences.
///
/// One of the two kinds of evidence a packet carries, the other being
/// [`Frame`]. A detector produces this, a slicer reads it, and it travels on
/// [`crate::PacketBody::Pulses`] with the level and the frequency it was
/// measured at rather than having those copied onto the packet around it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Package {
    pub pulses: Vec<Pulse>,
    /// Estimated SNR of the burst, in dB.
    pub snr_db: f32,
    /// Received level in dB, referenced to a full scale sample *at the
    /// detector's input*.
    ///
    /// Worth having next to the SNR rather than instead of it: a strong packet
    /// in a noisy channel and a weak one in a quiet channel can share an SNR,
    /// and only the level tells them apart or says the front end is clipping.
    ///
    /// Not calibrated to the antenna, and not quite calibrated to the ADC
    /// either: every filter between the two has gain, so a very strong signal
    /// reads a little above zero rather than pinning at it. Measured on the
    /// Fine Offset capture through a channelizer it comes out at +1 dB. It is
    /// a comparable number between packets on one receiver, which is what it
    /// is for, and not a field strength.
    pub rssi_dbfs: f32,
    /// Sample index where the burst started, for correlating with a waterfall.
    pub start_sample: u64,
    /// Where the burst was received, in Hz.
    ///
    /// Stamped by the detector from the stream it read, which in a channel
    /// bank is the channel's own centre rather than the tuner's. A package
    /// that has been separated from the port it arrived on is otherwise
    /// unplaceable, and a burst without a frequency is not evidence of much.
    pub center_hz: u64,
    /// What the burst was measured to be keyed on, when something measured it.
    ///
    /// `None` where the front end was chosen in advance rather than from the
    /// signal, which is every chain that runs one demodulator by
    /// configuration. A guess from the channel width stands in there, and it
    /// is a guess: a 125 kHz channel holds an OOK sensor as readily as an FSK
    /// one.
    pub modulation: Option<crate::Modulation>,
}

impl Package {
    pub fn len(&self) -> usize {
        self.pulses.len()
    }

    /// Rejoin marks split by a dropout too short to be a symbol.
    ///
    /// A weak burst does not fail cleanly. Its envelope crosses back under the
    /// threshold for a few microseconds in the middle of a mark, and what was
    /// one symbol arrives as two marks with a sliver of gap between them. Every
    /// timing measured after that is wrong, so a capture that is 20 dB above
    /// the point where the bits stop being recoverable can still decode
    /// nothing at all.
    ///
    /// The threshold for "too short" is taken from the burst rather than set
    /// in advance, which is Universal Radio Hacker's trick: the shortest
    /// widths that occur *often* are the symbol, so anything well under them is
    /// damage. A constant cannot do this because the symbol width is what
    /// varies between devices, by two orders of magnitude across this corpus.
    ///
    /// Returns how many joins were made.
    pub fn merge_dropouts(&mut self) -> usize {
        let Some(tol) = self.glitch_tolerance_us() else { return 0 };
        let before = self.pulses.len();
        let mut out: Vec<Pulse> = Vec::with_capacity(before);
        for p in self.pulses.iter().copied() {
            match out.last_mut() {
                // The previous pulse's gap was a dropout, not a gap: the two
                // marks and the sliver between them are one mark.
                Some(prev) if prev.gap > 0 && prev.gap < tol => {
                    prev.mark += prev.gap + p.mark;
                    prev.gap = p.gap;
                }
                _ => out.push(p),
            }
        }
        // A tolerance that swallows most of the burst was the wrong estimate,
        // and half a burst rewritten is worse than a burst left alone.
        if out.len() * 2 < before {
            return 0;
        }
        self.pulses = out;
        before - self.pulses.len()
    }

    /// Width below which a gap is damage rather than a symbol.
    ///
    /// An eighth of the median gap. The median is what the transmitter is
    /// doing: a burst coming apart grows short fragments at both ends of the
    /// distribution but its middle stays where it was, so the middle is the
    /// only stable thing to measure against. The shortest width is not, which
    /// is the trap: as a burst fragments, the shortest width is itself damage,
    /// so a tolerance derived from it shrinks exactly when it needs to grow.
    ///
    /// An eighth is well under the ratio between the two symbols of every
    /// coding here, which is two to one at its narrowest, so a real short
    /// symbol is never mistaken for a dropout.
    fn glitch_tolerance_us(&self) -> Option<u32> {
        if self.pulses.len() < 3 {
            return None;
        }
        // The final gap is the timeout that ended the burst rather than a gap
        // the transmitter sent, so it is left out of the estimate.
        let mut gaps: Vec<u32> =
            self.pulses[..self.pulses.len() - 1].iter().map(|p| p.gap).collect();
        gaps.retain(|g| *g > 0);
        if gaps.len() < 2 {
            return None;
        }
        gaps.sort_unstable();
        Some((gaps[gaps.len() / 2] / 8).max(1))
    }

    pub fn is_empty(&self) -> bool {
        self.pulses.is_empty()
    }

    /// Total on-air duration in microseconds, excluding the trailing timeout.
    pub fn duration_us(&self) -> u64 {
        self.pulses.iter().map(|p| p.mark as u64 + p.gap as u64).sum::<u64>()
            - self.pulses.last().map(|p| p.gap as u64).unwrap_or(0)
    }

    /// Histogram of mark widths, bucketed to `tol_us`.
    ///
    /// Reading this is how an unknown protocol gets identified by hand: PWM
    /// shows two clear mark clusters, PPM shows one mark cluster and two gap
    /// clusters, Manchester shows clusters at T and 2T in both.
    pub fn mark_histogram(&self, tol_us: u32) -> Vec<(u32, usize)> {
        histogram(self.pulses.iter().map(|p| p.mark), tol_us)
    }

    pub fn gap_histogram(&self, tol_us: u32) -> Vec<(u32, usize)> {
        // The trailing gap is a timeout, not signal, so leave it out.
        let n = self.pulses.len().saturating_sub(1);
        histogram(self.pulses[..n].iter().map(|p| p.gap), tol_us)
    }
}

fn histogram(vals: impl Iterator<Item = u32>, tol_us: u32) -> Vec<(u32, usize)> {
    let mut buckets: Vec<(u32, usize, u64)> = Vec::new();
    for v in vals {
        match buckets.iter_mut().find(|(c, _, _)| v.abs_diff(*c) <= tol_us) {
            Some((c, n, sum)) => {
                *n += 1;
                *sum += v as u64;
                *c = (*sum / *n as u64) as u32;
            }
            None => buckets.push((v, 1, v as u64)),
        }
    }
    buckets.sort_by_key(|(c, _, _)| *c);
    buckets.into_iter().map(|(c, n, _)| (c, n)).collect()
}

/// One packet on the bus: what a demodulator produced, and where.
///
/// The common currency between everything that produces packets and
/// everything that consumes them. A channel bank produces timings, a Mode S
/// demodulator produces frames, and a log, a list, a map or a tracker take
/// either without caring which front end was involved.
///
/// What is *not* here is a parse. A model name and a field map are
/// conclusions, and a conclusion travelling in place of its evidence cannot
/// be checked, corrected or re-decoded later.
#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    /// Wall clock when the block carrying it was processed, in microseconds
    /// since the epoch.
    pub at_us: u64,

    /// The width it was heard through. The same burst read through a 31 kHz
    /// channel and a 125 kHz one is not the same recording.
    pub bandwidth_hz: u32,
    /// The evidence itself: a burst's timings or a demodulator's frame, each
    /// carrying where it was heard and how strongly.
    ///
    /// Those used to be four more fields here, copied in from whichever of
    /// the two produced the packet and copied back out by `package()`. Two
    /// places to look for a level is one too many: a frame that measured its
    /// own preamble and a packet that had a level filled in for it disagreed,
    /// and nothing said which to believe.
    pub body: PacketBody,
    /// The burst itself, as complex samples at the rate it was read at,
    /// when the front end kept them.
    ///
    /// Shared rather than copied, since the packages of one burst all refer
    /// to the same samples, and carried in memory only: a log keeps
    /// timings and measurements, a list keeps the samples of what it shows.
    /// They are what an unknown device is worked out from, the way Universal
    /// Radio Hacker shows a burst beside its bits.
    pub iq: Option<std::sync::Arc<IqBurst>>,
    /// Speech the demodulator decoded, for a protocol that carries voice.
    ///
    /// The payload of a voice transmission is what was said, and no rendering
    /// of its bytes is that. Shared for the same reason the samples are: a
    /// minute of speech is megabytes, and the log, the player and anything
    /// writing it to a file all want the one copy.
    pub audio: Option<std::sync::Arc<Speech>>,
    /// What the protocols made of it, filled in by the one stage that runs
    /// them and read by everything downstream.
    ///
    /// A conclusion travelling *beside* its evidence, never in place of it:
    /// the timings and the bytes are still here, so a decode can be checked
    /// against them or made again by a decoder written later. Empty until the
    /// protocols node has seen the packet, and empty afterwards for a burst
    /// nothing claimed.
    ///
    /// This is what stops each view parsing for itself. The map used to
    /// re-parse ADS-B, AIS and APRS, and the device database ran the whole
    /// table a second time, so a decoder fix reached the packet list and not
    /// the map until somebody noticed.
    pub decodes: Vec<Decoded>,
    /// What the burst was measured to be, when something measured it before
    /// deciding how to read it. Travels with the timings because it is
    /// evidence about the same burst: a chirp's sweep rate or a keyed
    /// signal's tone separation is what identifies a device the tables do
    /// not know, and a burst no front end reads has nothing else to say.
    pub measure: Option<Measure>,
}

/// A burst's samples, as the front end that read it saw them: the lead-in,
/// the burst and the silence that ended it.
#[derive(Clone, Debug, PartialEq)]
pub struct IqBurst {
    /// Sample rate of `samples`.
    pub rate: f64,
    /// RF centre the samples are at baseband from.
    pub center_hz: u64,
    pub samples: Vec<C32>,
}

/// One demodulated frame, with what it was heard at.
///
/// The bytes alone were what this port used to carry, and every front end
/// that produces frames rather than pulses lost its level, its noise and its
/// samples at that boundary: a Mode S frame arrives from a demodulator that
/// measured the preamble, a BLE advertisement from one that measured the
/// burst and the floor either side of it, and both reached the packet list
/// reading NaN. A frame is evidence, and evidence carries how strong it was
/// and what it looked like.
#[derive(Clone, Debug, PartialEq)]
pub struct Frame {
    pub bytes: Vec<u8>,
    /// Where it was received. The front end's own answer, which is finer
    /// than the port's where a span holds more than one channel it reads:
    /// three BLE advertising channels, two AIS channels.
    pub center_hz: u64,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    /// The samples it was read from, when the front end kept them.
    pub iq: Option<std::sync::Arc<IqBurst>>,
}

impl Frame {
    /// A frame at what the front end that read it measured.
    ///
    /// There is no constructor without a level. A front end holds the samples
    /// its frame came from and is the only thing that can say how strong it
    /// was, so a frame reaching the bus without one is a front end that has
    /// not been finished; [`crate::Packet::fill_level`] is for a source that
    /// measured the extraction rather than the burst, and a remote feed
    /// passes on what its own receiver reported.
    pub fn measured(bytes: Vec<u8>, rssi_dbfs: f32, snr_db: f32) -> Self {
        Self { bytes, center_hz: 0, rssi_dbfs, snr_db, iq: None }
    }

    pub fn at(mut self, center_hz: u64) -> Self {
        self.center_hz = center_hz;
        self
    }

    pub fn with_iq(mut self, iq: std::sync::Arc<IqBurst>) -> Self {
        self.iq = Some(iq);
        self
    }
}

/// What a burst was measured to be, before any decoder read it.
#[derive(Clone, Debug, PartialEq)]
pub struct Measure {
    /// The classifier's verdict.
    pub modulation: crate::Modulation,
    /// How far that verdict stood above the runner-up, 0 to 1.
    pub confidence: f32,
    /// Which front end the burst was sent to.
    pub front_end: FrontEnd,
    /// The mode the parameters place it in, when one is known: "LoRa SF9
    /// BW125".
    pub mode: Option<String>,
    pub duration_us: u32,
    pub bandwidth_hz: f32,
    /// Symbol rate in baud, zero when none was found.
    pub baud: f32,
    /// Tone separation in hertz, for keyed signals; zero otherwise.
    pub separation_hz: f32,
    /// Sweep rate in hertz per second, for chirps; zero otherwise.
    pub sweep_hz_s: f32,
    /// Period of a repeating structure in microseconds, a cyclic prefix or
    /// a chip sequence; zero otherwise.
    pub symbol_period_us: f32,
}

/// The front ends a burst can be sent to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum FrontEnd {
    Ook,
    Ask,
    Fsk,
    C4fm,
    /// Both the envelope and the discriminator path, for a burst that could
    /// be either.
    OokFsk,
    /// Measured and sent nowhere.
    #[default]
    None,
}

impl FrontEnd {
    pub fn label(self) -> &'static str {
        match self {
            Self::Ook => "ook",
            Self::Ask => "ask",
            Self::Fsk => "fsk",
            Self::C4fm => "c4fm",
            Self::OokFsk => "ook+fsk",
            Self::None => "none",
        }
    }

    /// What a stored label names, or `None` for one this does not know.
    pub fn from_label(s: &str) -> Option<Self> {
        Self::ALL.iter().copied().find(|f| f.label() == s)
    }

    pub const ALL: [Self; 6] =
        [Self::Ook, Self::Ask, Self::Fsk, Self::C4fm, Self::OokFsk, Self::None];
}

impl std::fmt::Display for FrontEnd {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.label())
    }
}

impl Measure {

    /// One line a list can show: what it was, how sure, and the numbers
    /// that identify it.
    ///
    /// Only the numbers that belong to the verdict: the tone separation of
    /// a keyed signal, the sweep of a chirp, the period of a multi-carrier
    /// or spread signal. Every feature is measured on every burst, and the
    /// ones that belong to the hypotheses that lost are noise here.
    pub fn summary(&self) -> String {
        let mut parts = vec![format!("{} {:.2}", self.modulation, self.confidence)];
        if let Some(m) = &self.mode {
            parts.push(m.clone());
        }
        if self.bandwidth_hz > 0.0 {
            parts.push(format!("{:.1} kHz wide", self.bandwidth_hz / 1e3));
        }
        use crate::Modulation as M;
        let keyed = matches!(
            self.modulation,
            M::Ook | M::Ask | M::Fsk2 | M::Fsk4 | M::Msk | M::Psk2 | M::Psk4 | M::Dqpsk
        );
        if keyed && self.baud > 0.0 {
            parts.push(format!("{:.0} baud", self.baud));
        }
        if matches!(self.modulation, M::Fsk2 | M::Fsk4 | M::Msk) && self.separation_hz > 0.0 {
            parts.push(format!("tones {:.1} kHz apart", self.separation_hz / 1e3));
        }
        if self.modulation == M::Chirp && self.sweep_hz_s.abs() > 0.0 {
            parts.push(format!("sweep {:.1} MHz/s", self.sweep_hz_s / 1e6));
        }
        if matches!(self.modulation, M::Ofdm | M::Dsss) && self.symbol_period_us > 0.0 {
            parts.push(format!("period {:.1} us", self.symbol_period_us));
        }
        parts.push(format!("{:.1} ms", self.duration_us as f64 / 1e3));
        parts.join(", ")
    }
}

/// Decoded speech, at whatever rate the codec produces.
#[derive(Clone, Debug, PartialEq)]
pub struct Speech {
    pub pcm: Vec<f32>,
    pub rate: f64,
}

impl Speech {
    pub fn seconds(&self) -> f64 {
        self.pcm.len() as f64 / self.rate.max(1.0)
    }
}

/// Speech a front end is producing right now, and who is producing it.
///
/// The recorded [`Speech`] of a transmission travels with the packet that
/// ends it. This is the other half, and it cannot travel that way: somebody
/// listening wants the forty milliseconds that were just decoded, not the
/// whole over once it is finished. Every front end that carries voice reports
/// it the same way, so a listener is written once rather than once per
/// protocol.
#[derive(Clone, Debug, PartialEq)]
pub struct Voice {
    /// The system it belongs to: "M17", "DMR". What a subscription names.
    pub system: &'static str,
    /// Centre of the channel it was heard on, in hertz.
    pub channel_hz: f64,
    /// The group or party being called, when a transmission is in progress.
    pub to: Option<String>,
    /// Who is talking, when the system says.
    pub from: Option<String>,
    pub rate: f64,
    /// Decoded in the last block. Empty when the channel is idle, which is
    /// still worth reporting: it says the front end is there and listening.
    pub pcm: Vec<f32>,
}

/// Which conversation a block of speech belongs to.
///
/// One type rather than a string each end formats for itself. The audio bus
/// keys who is talking now on it, the transcriber files what was said under
/// it, the call list rows on it and a meter is read back by it, and a row
/// showed no transcript the moment any two of them spelled it differently.
#[derive(Clone, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct ConversationKey {
    /// The system it was heard on: "M17", "DMR", or what an analogue channel
    /// is called on the bus.
    pub system: String,
    /// Centre of the channel, to the hertz. Rounded from the frequency the
    /// front end reported, so the key can be compared and hashed.
    pub channel_hz: u64,
    /// The talkgroup, reflector or party being called, where the system
    /// names one.
    pub to: Option<String>,
    /// Who is talking, where the system says. `None` on a meter's key: a
    /// meter belongs to the channel and the group, and a caller changing
    /// mid-conversation must not move it.
    pub from: Option<String>,
}

/// How far apart two frequencies can be and still be the same channel.
///
/// A rounding rather than a channel: the narrowest grid anything here uses
/// is 12.5 kHz, and the same talkgroup found by two front ends a few hundred
/// hertz apart is one conversation rather than two rows.
pub const CHANNEL_MATCH_HZ: f64 = 500.0;

impl ConversationKey {
    pub fn new(system: impl Into<String>, channel_hz: f64) -> Self {
        Self {
            system: system.into(),
            channel_hz: channel_hz.max(0.0) as u64,
            to: None,
            from: None,
        }
    }

    pub fn to(mut self, to: Option<String>) -> Self {
        self.to = to;
        self
    }

    pub fn from(mut self, from: Option<String>) -> Self {
        self.from = from;
        self
    }

    /// The key of the conversation this speech is part of.
    pub fn of(v: &Voice) -> Self {
        Self::new(v.system, v.channel_hz).to(v.to.clone()).from(v.from.clone())
    }

    /// The same conversation with nobody named as talking: what a meter is
    /// filed under, since a trunked carrier holds several groups and each of
    /// them meters on its own.
    pub fn meter(&self) -> Self {
        Self { from: None, ..self.clone() }
    }

    /// Whether two keys name the same conversation, allowing for the
    /// hundreds of hertz two front ends can disagree about a channel by.
    /// A caller either side may be unnamed: on TETRA the grant names who is
    /// talking and the traffic that follows does not.
    pub fn same_conversation(&self, other: &Self) -> bool {
        self.system == other.system
            && self.to == other.to
            && self.channel_hz.abs_diff(other.channel_hz) < CHANNEL_MATCH_HZ as u64
            && (self.from == other.from || self.from.is_none() || other.from.is_none())
    }
}

impl std::fmt::Display for ConversationKey {
    /// What a person, a log line or a saved transcript reads.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}:{}:{}:{}",
            self.system,
            self.channel_hz,
            self.to.as_deref().unwrap_or(""),
            self.from.as_deref().unwrap_or("")
        )
    }
}

/// How a video frame's samples are laid out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pixels {
    /// One byte a pixel, 0 for black and 255 for peak white.
    Luma8,
    /// Three bytes a pixel, red then green then blue.
    Rgb8,
}

impl Pixels {
    pub fn bytes(self) -> usize {
        match self {
            Self::Luma8 => 1,
            Self::Rgb8 => 3,
        }
    }
}

/// A picture as it was received, with what it was received from.
///
/// The video counterpart of [`Voice`], and here for the same reason: a field
/// does not arrive at the rate the graph negotiated. It arrives fifty times a
/// second in one lump, at the camera's rate rather than the receiver's, and
/// it carries what it came from. Squeezed through a byte port it would lose
/// its geometry; through a real-valued one it would be a lie about the rate.
///
/// The samples are behind an `Arc` because a field is half a megabyte and the
/// receiver hands the same one to a pane, a recorder and whatever else is
/// watching. Nothing here copies it.
#[derive(Clone, Debug, PartialEq)]
pub struct VideoFrame {
    /// What produced it: "analogue video", and later "APT", "SSTV". What a
    /// subscription names, the way [`Voice::system`] is.
    ///
    /// The system, not the use it is being put to: a model aircraft's camera
    /// and a security camera send the same composite video and differ only in
    /// where they send it, which `channel_hz` and `label` already say.
    pub system: &'static str,
    /// Centre of the channel it was received on.
    pub channel_hz: f64,
    /// What the channel is called where it has a name: a channel of the
    /// 5.8 GHz plan pilots use, a satellite, a callsign. `None` on a band with
    /// no naming convention, which is a fact about the band rather than a
    /// gap.
    pub label: Option<String>,
    pub width: usize,
    pub height: usize,
    /// How wide the picture is against its height when drawn.
    ///
    /// Not `width` over `height`: how many samples a line was cut into is a
    /// fact about the receiver's clock, and a field is half a frame. Both
    /// analogue standards are 4:3, and a picture drawn from the sample grid
    /// instead came out squeezed in from the sides.
    pub aspect: f32,
    pub pixels: Pixels,
    pub samples: std::sync::Arc<Vec<u8>>,
    /// Rows that were actually received, out of `height`.
    ///
    /// Analogue video has no integrity check of any kind, so this is the only
    /// thing a viewer has to judge a picture by: a field assembled from a
    /// third of its lines is a picture of a fade, and a receiver that draws it
    /// without saying so is claiming more than it knows.
    pub lines_seen: usize,
    /// Fields since the front end started, so a viewer can tell a repeated
    /// frame from a still picture.
    pub sequence: u64,
}

impl VideoFrame {
    /// How complete the picture is, 0 to 1.
    pub fn completeness(&self) -> f32 {
        if self.height == 0 {
            return 0.0;
        }
        self.lines_seen as f32 / self.height as f32
    }
}

/// What the demodulator actually produced.
#[derive(Clone, Debug, PartialEq)]
pub enum PacketBody {
    /// A burst a detector found, as mark and gap timings.
    Pulses(Package),
    /// A whole frame from a demodulator that produces bytes.
    Frame(Frame),
}

impl Packet {
    /// A burst a detector found, on the bus.
    pub fn of_pulses(at_us: u64, bandwidth_hz: u32, package: Package) -> Self {
        Self {
            at_us,
            bandwidth_hz,
            body: PacketBody::Pulses(package),
            iq: None,
            audio: None,
            measure: None,
            decodes: Vec::new(),
        }
    }

    /// A frame a demodulator produced, on the bus.
    pub fn of_frame(at_us: u64, bandwidth_hz: u32, frame: Frame) -> Self {
        Self {
            at_us,
            bandwidth_hz,
            body: PacketBody::Frame(frame),
            iq: None,
            audio: None,
            measure: None,
            decodes: Vec::new(),
        }
    }

    /// Fill in a level the evidence did not carry.
    ///
    /// For a front end that measures the source rather than the burst: the
    /// auto node knows how loud its extraction was and the demodulator inside
    /// it may not have measured anything. Only fills what is missing, so a
    /// measurement always wins over an estimate.
    pub fn fill_level(&mut self, rssi_dbfs: f32, snr_db: f32) {
        let (r, s) = match &mut self.body {
            PacketBody::Pulses(p) => (&mut p.rssi_dbfs, &mut p.snr_db),
            PacketBody::Frame(f) => (&mut f.rssi_dbfs, &mut f.snr_db),
        };
        if r.is_nan() {
            *r = rssi_dbfs;
        }
        if s.is_nan() {
            *s = snr_db;
        }
    }

    /// Say where it was heard, for a front end whose evidence did not.
    pub fn set_center(&mut self, hz: u64) {
        match &mut self.body {
            PacketBody::Pulses(p) => p.center_hz = hz,
            PacketBody::Frame(f) => f.center_hz = hz,
        }
    }

    /// The burst's timings, ready to hand to a decoder.
    pub fn package(&self) -> Option<&Package> {
        match &self.body {
            PacketBody::Pulses(p) => Some(p),
            PacketBody::Frame(_) => None,
        }
    }

    pub fn frame(&self) -> Option<&[u8]> {
        match &self.body {
            PacketBody::Frame(f) => Some(&f.bytes),
            PacketBody::Pulses(_) => None,
        }
    }

    /// Where it was received, in hertz: the channel's own centre in a bank,
    /// the advertising channel a BLE frame arrived on.
    pub fn center_hz(&self) -> u64 {
        match &self.body {
            PacketBody::Pulses(p) => p.center_hz,
            PacketBody::Frame(f) => f.center_hz,
        }
    }

    /// Received level in dBFS, as whatever produced the evidence measured it.
    pub fn rssi_dbfs(&self) -> f32 {
        match &self.body {
            PacketBody::Pulses(p) => p.rssi_dbfs,
            PacketBody::Frame(f) => f.rssi_dbfs,
        }
    }

    pub fn snr_db(&self) -> f32 {
        match &self.body {
            PacketBody::Pulses(p) => p.snr_db,
            PacketBody::Frame(f) => f.snr_db,
        }
    }

    /// What the burst was measured to be keyed on. See
    /// [`Package::modulation`]. A frame comes from a front end chosen in
    /// advance, so its keying is the front end's and not a measurement.
    pub fn modulation(&self) -> Option<crate::Modulation> {
        match &self.body {
            PacketBody::Pulses(p) => p.modulation,
            PacketBody::Frame(_) => None,
        }
    }

    /// The samples behind it: the burst's own, or the packet's where the
    /// front end attached them to the packet instead.
    pub fn samples(&self) -> Option<&std::sync::Arc<IqBurst>> {
        match &self.body {
            PacketBody::Frame(f) if f.iq.is_some() => f.iq.as_ref(),
            _ => self.iq.as_ref(),
        }
    }
}

#[cfg(test)]
mod dropout_tests {
    use super::*;

    fn pkg(pulses: &[(u32, u32)]) -> Package {
        Package {
            pulses: pulses.iter().map(|(m, g)| Pulse { mark: *m, gap: *g }).collect(),
            ..Default::default()
        }
    }

    #[test]
    fn a_dropout_inside_a_mark_is_rejoined() {
        // 500 us marks on a 500 us raster, with one mark split by a 20 us dip.
        let mut p = pkg(&[(500, 500), (240, 20), (240, 500), (500, 500)]);
        assert_eq!(p.merge_dropouts(), 1);
        assert_eq!(
            p.pulses,
            pkg(&[(500, 500), (500, 500), (500, 500)]).pulses,
            "the split mark should read as one 500 us mark again"
        );
    }

    #[test]
    fn a_real_short_symbol_is_left_alone() {
        // PWM: 250 and 500 us marks, 250 us gaps. Nothing here is damage, and
        // a tolerance that ate the short symbol would destroy every bit.
        let mut p = pkg(&[(250, 250), (500, 250), (250, 250), (500, 250)]);
        assert_eq!(p.merge_dropouts(), 0);
        assert_eq!(p.pulses.len(), 4);
    }

    #[test]
    fn one_odd_reading_does_not_set_the_tolerance() {
        // A single 8 us sliver must not make 8 us the yardstick: the estimate
        // comes from the shortest width that repeats.
        let mut p = pkg(&[(8, 500), (500, 500), (500, 500), (500, 500)]);
        p.merge_dropouts();
        assert!(p.pulses.len() >= 3);
    }

    #[test]
    fn a_burst_that_would_be_mostly_rewritten_is_left_alone() {
        // Every gap under the tolerance means the estimate was wrong, and
        // collapsing the burst to one pulse invents a signal that was not sent.
        let mut p = pkg(&[(100, 5), (100, 5), (100, 5), (100, 5), (100, 5)]);
        let before = p.pulses.clone();
        p.merge_dropouts();
        assert_eq!(p.pulses, before);
    }
}
