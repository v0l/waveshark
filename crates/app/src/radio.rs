//! Background RF thread: owns the device, publishes spectrum frames, and
//! demodulates whichever channel is selected for audio.

use crate::chain::Plan;
use audio::AudioPlayer;
use common::{C32, GainMode, Hz, Sps};
use crossbeam_channel::{Receiver, Sender, TrySendError, bounded};
use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering},
};

/// What a strip channel does with the band it is tuned to.
///
/// A channel used to be a demodulator and nothing else, so the only way to
/// read one digital channel was a scanner block over the span, which then
/// swept every other channel in it as well. A decode channel is the front end
/// on its own: one extraction at the frequency the channel is tuned to, the
/// decoder behind it, and nothing else running.
///
/// The decode arm names a registry stage rather than listing protocols here,
/// so a front end that declares a channel width can be put on a strip channel
/// without this enum learning about it.
#[derive(Clone, PartialEq, Debug)]
pub enum ChanMode {
    Audio(Demod),
    Decode(String),
    /// The auto front end over a band of the operator's choosing, rather than
    /// over the one a scanner block was written about.
    ///
    /// The same node the scanner table places, put where somebody points
    /// instead: it finds what transmits inside the channel's width, measures
    /// each source and gives it the decoder that reads it. A band worth
    /// watching that no block covers is then a channel, not a config file
    /// edit and a restart.
    Auto,
}

/// What an auto channel watches until its width is set by hand. Wide enough
/// to hold a handful of narrowband transmitters, narrow enough that the
/// detector's resolution over it is still a few hundred hertz.
pub const AUTO_CHANNEL_HZ: f64 = 200_000.0;

impl ChanMode {
    pub fn label(&self) -> String {
        match self {
            ChanMode::Audio(d) => d.label().to_string(),
            ChanMode::Decode(kind) => crate::chain::front_label(kind),
            ChanMode::Auto => "AUTO".into(),
        }
    }

    /// The demodulator this channel plays, if it plays one.
    pub fn demod(&self) -> Option<Demod> {
        match self {
            ChanMode::Audio(d) => Some(*d),
            ChanMode::Decode(_) | ChanMode::Auto => None,
        }
    }

    /// Whether this channel is a front end rather than something played: its
    /// packets belong on the bus, and it is heard only if what it found has
    /// speech in it.
    pub fn is_decode(&self) -> bool {
        matches!(self, ChanMode::Decode(_) | ChanMode::Auto)
    }

    /// Occupied bandwidth, two-sided: what the channel covers on the
    /// spectrum, and what a filter in front of it has to pass.
    ///
    /// This is the mode's own width. What a channel actually uses is
    /// [`ChannelSpec::bandwidth`], which is this unless the operator set one.
    pub fn bandwidth(&self) -> f64 {
        match self {
            ChanMode::Audio(d) => d.bandwidth(),
            ChanMode::Decode(kind) => crate::chain::front_width(kind).unwrap_or(12_500.0),
            ChanMode::Auto => AUTO_CHANNEL_HZ,
        }
    }

    /// The least span this channel can be built in, at a given width.
    pub fn min_rate_for(&self, bandwidth: f64) -> f64 {
        match self {
            // Room for the channel filter's transition band, and never below
            // the rate the demodulator was designed at.
            ChanMode::Audio(d) => d.if_rate().max(bandwidth * IF_HEADROOM),
            // The front end mixes and decimates its own channel out of
            // whatever it is handed, so what it needs is a stream that holds
            // the channel at all. Where the protocol says what that is, it
            // is taken: twice the width is a guess, and a guess refuses
            // DVB-T, whose 8 MHz channel is read from a stream only 1.14
            // times as wide because the standard says so.
            // A span-wide decoder reads less than its whole allocation on
            // purpose, so the declared rate stands alone rather than being
            // held up to the width.
            ChanMode::Decode(kind) => nodes::protocol::by_id(kind)
                .map(|p| p.shape().min_rate_hz)
                .filter(|r| *r > 0.0)
                .unwrap_or(bandwidth * 2.0),
            ChanMode::Auto => bandwidth * 2.0,
        }
    }

    /// A number a stage id can be keyed on, so a channel that changed mode is
    /// not the same channel and does not reuse filters designed for the old
    /// one.
    pub(crate) fn key(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut h = std::collections::hash_map::DefaultHasher::new();
        match self {
            ChanMode::Audio(d) => (0u8, *d as u8).hash(&mut h),
            ChanMode::Decode(kind) => (1u8, kind).hash(&mut h),
            ChanMode::Auto => 2u8.hash(&mut h),
        }
        h.finish()
    }
}

/// How far above a channel's own width its IF has to run. Decimating to the
/// width itself leaves the filter no transition band, which is the same
/// reason [`Demod::if_rate`] sits where it does.
pub const IF_HEADROOM: f64 = 1.25;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Demod {
    Wfm,
    Nfm,
    Am,
    /// Upper sideband, the amateur convention above 10 MHz.
    Usb,
    /// Lower sideband, the convention on 160, 80 and 40 metres.
    Lsb,
    /// Morse, which is upper sideband through a narrow filter.
    Cw,
}

impl Demod {
    pub fn label(self) -> &'static str {
        match self {
            Demod::Wfm => "WFM",
            Demod::Nfm => "NFM",
            Demod::Am => "AM",
            Demod::Usb => "USB",
            Demod::Lsb => "LSB",
            Demod::Cw => "CW",
        }
    }

    /// Whether this mode listens to one sideband of the dial frequency.
    pub fn is_ssb(self) -> bool {
        matches!(self, Demod::Usb | Demod::Lsb | Demod::Cw)
    }

    pub(crate) fn sideband(self) -> dsp::ssb::Sideband {
        match self {
            Demod::Lsb => dsp::ssb::Sideband::Lower,
            _ => dsp::ssb::Sideband::Upper,
        }
    }

    /// Where the squelch opens by default, or None for a mode with none.
    ///
    /// Public because a control has to show the value in use before the
    /// operator has touched anything, and inventing a second copy of these
    /// numbers in the interface is how the two drift apart.
    pub fn default_squelch_db(self) -> Option<f32> {
        match self {
            // Measured, not guessed. Through this chain an empty channel
            // reads about 6.4 dB and an FM signal reads 24 dB and hardly
            // moves with signal strength, because FM captures. Sitting in the
            // middle of that gap keeps noise out with room for the reading to
            // wander, which live it does by a couple of dB.
            //
            // It was 9 dB, which is inside the noise's own variation: any
            // excursion opened the squelch, and the hysteresis then held it
            // open on noise indefinitely.
            Demod::Nfm => Some(14.0),
            // Off, at the bottom of the control's range.
            //
            // A level squelch has no fixed sensible setting: measured on an
            // empty 2 m channel the audio sits at -26 dBFS in AM, -36 in USB
            // and -59 in CW, and all three move with the RF gain. A number
            // picked here would be doing nothing on one mode and muting a
            // station on another, and SSB is normally listened to wide open
            // anyway. Drag it up against the meter to set one.
            Demod::Am | Demod::Usb | Demod::Lsb | Demod::Cw => Some(-90.0),
            Demod::Wfm => None,
        }
    }

    /// The range a squelch control should span for this mode, and whether the
    /// measurement is a noise ratio rather than a level.
    pub fn squelch_range(self) -> (f32, f32, bool) {
        match self {
            // How much of the signal is not noise: 0 dB is an empty channel
            // and 25 dB is full quieting.
            Demod::Nfm => (0.0, 25.0, true),
            _ => (-90.0, -10.0, false),
        }
    }

    /// The pitch a CW signal is heard at.
    ///
    /// The receiver is tuned this far below the carrier so that the dial
    /// reads the transmitted frequency rather than the note in the operator's
    /// ears, which is the convention every other radio follows and the one
    /// that makes two stations agree about where they are.
    pub fn cw_pitch(self) -> f64 {
        match self {
            Demod::Cw => 700.0,
            _ => 0.0,
        }
    }

    /// Occupied channel bandwidth, two-sided.
    pub fn bandwidth(self) -> f64 {
        match self {
            // Carson: 2 * (75 kHz deviation + 57 kHz highest modulating
            // frequency). The highest is RDS, not audio: taking 15 kHz gives
            // 180 kHz and cuts off precisely the sidebands that carry the
            // subcarrier, which decodes audio perfectly and RDS barely at all.
            Demod::Wfm => 264_000.0,
            Demod::Nfm => 12_500.0,
            Demod::Am => 10_000.0,
            // Twice the audio bandwidth, because only one sideband is there
            // but the IF filter around it is symmetric: half of this has to
            // reach the far edge of the sideband or the top of the voice is
            // filtered off before the demodulator sees it.
            Demod::Usb | Demod::Lsb => 6_000.0,
            Demod::Cw => 4_000.0,
        }
    }

    /// Sample rate to run the demodulator at.
    ///
    /// Comfortably above the channel bandwidth, never equal to it. Decimating
    /// until the output rate matches the bandwidth leaves no transition band,
    /// and the anti-alias filter then needs thousands of taps: 7947 for NFM
    /// against 281 here, with a history buffer too big for L2.
    pub(crate) fn if_rate(self) -> f64 {
        match self {
            // Must clear the 264 kHz occupied bandwidth with room for a
            // transition band.
            Demod::Wfm => 330_000.0,
            // The sideband filter runs here rather than after a further
            // decimation, because it is the thing that defines the channel
            // and 363 taps at this rate is a few percent of one core.
            Demod::Nfm | Demod::Am | Demod::Usb | Demod::Lsb | Demod::Cw => 48_000.0,
        }
    }

    /// Audio bandwidth after demodulation.
    pub(crate) fn audio_bw(self) -> f64 {
        match self {
            Demod::Wfm => 15_000.0,
            Demod::Nfm => 4_000.0,
            Demod::Am => 5_000.0,
            Demod::Usb | Demod::Lsb => 3_000.0,
            Demod::Cw => 1_200.0,
        }
    }

    pub(crate) fn deviation(self) -> f64 {
        match self {
            Demod::Wfm => 75_000.0,
            Demod::Nfm => 5_000.0,
            Demod::Am | Demod::Usb | Demod::Lsb | Demod::Cw => 0.0,
        }
    }
}

/// How much of a meter's reading survives a block once the sound stops.
const METER_FALL: f32 = 0.88;

/// Reopen a radio at a new rate and start it streaming again.
///
/// The gain is passed back in because opening a device resets it, and a span
/// change that silently returned the receiver to its default gain would look
/// like the antenna had fallen out.
fn restart(
    entry: &crate::devices::Entry,
    rate: Sps,
    center: Hz,
    front: &FrontEnd,
    ppm: f64,
    offset: f64,
) -> common::Result<(Box<dyn common::Device>, Box<dyn common::RxStream>)> {
    // The device needs a moment to release its USB claim; reopening
    // immediately gets "already in use".
    std::thread::sleep(std::time::Duration::from_millis(150));
    let mut dev = crate::devices::open(entry)?;
    dev.set_rate(rate)?;
    // Reopening resets the correction and the converter, and a span change
    // that silently threw either away would put every frequency back where it
    // was wrong.
    dev.correct(ppm);
    dev.set_offset(offset);
    dev.set_dial(center)?;
    front.apply(dev.as_mut());
    let stream = dev.start_rx()?;
    Ok((dev, stream))
}

/// What the front end is set to, per stage and per switch.
///
/// A reopened device starts at its defaults, so every stage and switch has to
/// be put back and not just the total: a HackRF reopened for a span change
/// came back with the baseband VGA at zero, which looks like the antenna
/// fell out. Read off the device rather than from the commands, so a driver
/// that distributes a total across its stages or quantises one reports what
/// it actually did.
#[derive(Clone, Debug, Default)]
struct FrontEnd {
    gains: Vec<(String, GainMode)>,
    toggles: Vec<(String, bool)>,
}

impl FrontEnd {
    fn read(dev: &dyn common::Device) -> Self {
        Self {
            gains: dev.gains(),
            toggles: dev.toggles().into_iter().map(|t| (t.name, t.on)).collect(),
        }
    }

    fn apply(&self, dev: &mut dyn common::Device) {
        for (stage, mode) in &self.gains {
            if let Err(e) = dev.set_gain(stage, *mode) {
                tracing::warn!("could not restore {stage} gain: {e}");
            }
        }
        for (name, on) in &self.toggles {
            if let Err(e) = dev.set_toggle(name, *on) {
                tracing::warn!("could not restore {name}: {e}");
            }
        }
    }
}

/// Open the radio for transmit, and say what the graph should key.
///
/// No graph is built here. The transmitter is stages in the receiver's own
/// patch, so keying is a change to the plan and a rebuild: that is what makes
/// a transmission visible in the chain view, tappable, and parameterised like
/// everything else the receiver does.
///
/// The radio is not taken away from the receiver either. A half duplex driver
/// keeps the receive stream alive and feeds it a noise floor for the
/// duration, so an over does not throw away the spectrum's averaging, every
/// channel's squelch and every decoder's part-built frame; a full duplex
/// radio goes on hearing the band while it transmits.
fn tx_plan_for(ch: &ChannelSpec, center: Hz) -> Option<crate::chain::TxPlan> {
    let mode = tx_mode_for(&ch.mode)?;
    let tx = ch.spec_to_transmit();
    let on_air = Hz((center.as_f64() + ch.offset_hz + tx.shift_hz).max(0.0) as u64);
    Some(crate::chain::TxPlan { spec: tx, mode, on_air })
}

/// The transmit chain to draw, from the channels as they are now.
///
/// One channel, because the radio has one transmitter and the chain view has
/// one transmit chain to draw. The channel going on air is that one; with
/// nothing keyed it is the first that can transmit, which is what the graph
/// holds ready.
///
/// `on_air` matters as soon as there are two transmit channels, which is one
/// recalled from the bank beside the one already on the strip: without it the
/// rebuild that puts a key on air drew the first channel's chain instead, so
/// a channel set to MIC transmitted the other one's test tone.
fn derive_tx(plan: &Plan, can_transmit: bool, on_air: Option<u64>) -> Option<crate::chain::TxPlan> {
    if !can_transmit {
        return None;
    }
    let keyed = on_air
        .and_then(|id| plan.channels.iter().find(|c| c.id == id))
        .and_then(|c| tx_plan_for(c, plan.center));
    keyed.or_else(|| plan.channels.iter().find_map(|c| tx_plan_for(c, plan.center)))
}

fn key_up(
    dev: &mut dyn common::Device,
    ch: &ChannelSpec,
    tx: &TxSpec,
    center: Hz,
    gain_db: f32,
    mic: &Option<audio::AudioCapture>,
    voice: &Option<std::sync::Arc<dyn audio::AudioSource>>,
    sub: &Option<SubFile>,
) -> common::Result<(crate::chain::TxPlan, crate::chain::TxSinks)> {
    // Where the channel transmits: its own frequency plus the repeater
    // shift, which is zero for simplex.
    let on_air = Hz((center.as_f64() + ch.offset_hz + tx.shift_hz).max(0.0) as u64);
    // The channel's own mode, because a channel is one frequency and one
    // mode: a radio that listens in NFM and keys up in AM cannot be worked.
    let mode = tx_mode_for(&ch.mode).ok_or_else(|| {
        common::Error::other(format!("nothing here transmits {} yet", ch.mode.label()))
    })?;
    if !dev.info().can_transmit() {
        return Err(common::Error::TxUnsupported);
    }
    if !dev.info().covers_tx(on_air) {
        return Err(common::Error::other(format!("{on_air} is outside what this radio transmits")));
    }
    // A half duplex radio has one synthesiser, so it is retuned for the over
    // and the receiver hears the transmit frequency while it lasts. A full
    // duplex one has a synthesiser per direction, which is what makes a
    // repeater pair one radio listening on the output and transmitting on the
    // input at the same time.
    let half = dev.info().tx.as_ref().is_some_and(|t| t.half_duplex);
    match half {
        true if on_air != dev.center() => dev.set_center(on_air)?,
        true => {}
        false => dev.set_tx_center(on_air)?,
    }
    // By the names the device gave, rather than by a list of radios kept
    // here: a HackRF has an amp and a TXVGA, a LimeSDR has one distributed
    // gain, and a driver added later will have its own.
    let want = (gain_db + tx.trim_db).max(0.0);
    let stages: Vec<String> = dev
        .info()
        .tx
        .as_ref()
        .map(|t| t.gain_stages.iter().map(|s| s.name.clone()).collect())
        .unwrap_or_default();
    for name in stages {
        // The front end amp is a switch, not a level, and switching it in
        // because the gain was turned up is a 14 dB surprise.
        let db = if name == "amp" { 0.0 } else { want };
        dev.set_tx_gain(&name, GainMode::Manual(db))?;
    }

    // The microphone is already open, because the channel asked for it when
    // it was set to MIC rather than when it was keyed: the meter has to move
    // before an operator can set a level against it.
    let src = match tx.source {
        // A data mode transmits a file, whatever the channel's source says:
        // refusing to key up for want of a microphone nothing will read
        // would be refusing for no reason.
        _ if matches!(mode, TxMode::Digital(_)) => None,
        TxSource::Mic => {
            Some(mic.as_ref().ok_or_else(|| common::Error::other("no microphone is open"))?.tap())
        }
        TxSource::Agent => {
            Some(voice.clone().ok_or_else(|| common::Error::other("the agent has no voice"))?)
        }
        // The file is already on the node; there is nothing to hand in at
        // key-up.
        TxSource::Sub | TxSource::Tone => None,
    };

    Ok((
        crate::chain::TxPlan { spec: *tx, mode, on_air },
        crate::chain::TxSinks { stream: Some(dev.start_tx()?), mic: src, sub: sub.clone() },
    ))
}

/// Put the correction on the device, and say how much of it the receiver has
/// to apply itself.
///
/// Only the RTL-SDR corrects its own reference. Every other driver takes the
/// call, does nothing, and goes on reporting zero, which is what made the
/// setting look broken: the box snapped back to 0 the moment it was let go
/// and nothing moved. Where the device will not do it, the offset is applied
/// to every frequency asked for instead, which is the same correction one
/// step further out.
/// Shortest gap between retunes.
///
/// A retune is a blocking USB control transfer costing about 25 ms on the
/// RTL-SDR, and it stalls sample reading while it happens. At this spacing it
/// takes roughly a fifth of the time and the spectrum keeps updating; issuing
/// one per frame instead leaves nothing over to read with and the display
/// freezes for as long as the drag lasts.
const MIN_TUNE_GAP: std::time::Duration = std::time::Duration::from_millis(120);

/// How often the running chain is republished, for the throughput on its
/// wires. Fast enough to watch, slow enough that cloning the topology is
/// nothing beside the DSP.
const CHAIN_PUBLISH: std::time::Duration = std::time::Duration::from_millis(250);

/// Overridable so the benchmark can measure what happens without the spacing.
fn tune_gap() -> std::time::Duration {
    match std::env::var("SR_TUNE_GAP_MS").ok().and_then(|v| v.parse().ok()) {
        Some(ms) => std::time::Duration::from_millis(ms),
        None => MIN_TUNE_GAP,
    }
}

/// A Flipper `.sub` file, parsed and ready to key.
#[derive(Clone, Debug, PartialEq)]
pub struct SubFile {
    /// Where it came from, which is all a person knows it by.
    pub path: String,
    /// The parsed file: frequency, preset and the pulses themselves.
    pub file: decode::subghz::SubGhz,
}

impl SubFile {
    /// What the strip says the file is.
    pub fn label(&self) -> String {
        self.file.protocol.clone()
    }

    /// Parse a `.sub` file from disk.
    pub fn open(path: &std::path::Path) -> Result<Self, decode::subghz::SubError> {
        let text =
            std::fs::read_to_string(path).map_err(|_| decode::subghz::SubError::NotASubFile)?;
        Ok(Self { path: path.display().to_string(), file: decode::subghz::parse(&text)? })
    }
}

/// Instructions from the interface to the radio.
#[derive(Clone)]
pub enum Cmd {
    Center(Hz),
    Rate(Sps),
    /// The complete set of channels to demodulate and mix.
    Channels(Vec<ChannelSpec>),
    Fft(usize),
    /// Spectrum frames per second delivered to the UI.
    Refresh(f32),
    /// Exponential averaging applied to the spectrum, 1.0 for none.
    Smoothing(f32),
    /// Remove the centre spur a direct-conversion receiver produces.
    DcBlock(bool),
    /// Set one named gain stage on the radio itself.
    GainStage(String, GainMode),
    /// Flip one of the radio's own switches: bias tee, digital AGC and so on.
    Toggle(String, bool),
    /// Pick one of the radio's list settings, such as an antenna port.
    Choice(String, String),
    /// Set one parameter on one node of the running graph, by node id.
    NodeParam(usize, String, pipeline::param::ParamValue),
    /// Set one parameter on a stage the receiver draws for itself, by the
    /// id it is drawn under, which the interface knows without asking the
    /// graph: the master on the speaker, the calls level on the bus. The
    /// same route as `NodeParam` once it lands, so the setting is an edit
    /// and survives a rebuild.
    StageParam(u64, String, pipeline::param::ParamValue),
    /// Reference oscillator correction, in parts per million.
    Ppm(f64),
    /// What to add to the tuner's frequency to get the frequency at the
    /// aerial, in hertz. Positive for a converter that mixes down, such as a
    /// satellite LNB; negative for one that mixes up, such as an HF
    /// upconverter. Zero for an aerial straight into the radio.
    Offset(f64),
    /// Narrow the span in software by decimating what the radio delivers.
    ///
    /// A HackRF cannot sample below 2 MS/s, so a 12.5 kHz channel is a
    /// fraction of a pixel wide on any sensible display. This trades span for
    /// resolution without the radio being involved.
    Zoom(usize),
    /// Decode every channel in the span, or stop doing so.
    Decode(bool),
    /// What to listen to on the call bus, as the whole set of standing
    /// instructions rather than an edit to them: the same bargain the
    /// scanner table and the feeds make.
    CallSubs(Vec<crate::mix::calls::Subscription>),
    /// Which picture to watch, as a whole set of rules like `CallSubs`: an
    /// input of the video bus, or everything, which is what a receiver
    /// watching one channel wants and what a receiver scanning several does.
    WatchVideo(Vec<crate::videobus::Rule>),
    /// Play a transmission that has already been decoded, once.
    ///
    /// The sink belongs to this thread, so replaying from the packet list is
    /// a message rather than the interface opening its own audio device.
    Play(std::sync::Arc<common::Speech>),
    /// Drop what is being played back, wherever it got to.
    StopPlaying,
    /// Write every burst that decodes to this directory, with an optional
    /// budget in megabytes, or stop recording.
    Record(Option<(std::path::PathBuf, Option<u64>)>),
    /// Start or stop writing the raw span to a file.
    CaptureIq(bool),
    /// What the heatmap recorder keeps: whether it is running, how often it
    /// takes a row and how much it may hold.
    Heatmap(crate::chain::HeatPlan),
    /// Write what the heatmap holds as a file, coloured and scaled as the
    /// interface is showing it. Done on this thread because the readings are
    /// a node's, and the node belongs to the graph running here.
    ExportHeatmap {
        what: crate::heatmap::Export,
        ramp: crate::heatmap::Ramp,
        floor: f32,
        ceil: f32,
    },
    /// Size the capture folder may reach before writing stops, in bytes.
    CaptureCap(u64),
    /// Where the receiver is, in degrees, which lets the flight tracker
    /// resolve a position from a single frame instead of waiting for a pair.
    Location(f64, f64),
    /// Record a survey to this file, or stop. The device database is a node
    /// on the packet bus, so like the packet log this is a command to the
    /// radio thread rather than a setting the interface keeps.
    Survey(Option<std::path::PathBuf>),
    /// Walk the dial across a band, or stop walking. The walk is a node on
    /// the packet bus, so this is a command to the radio thread like the
    /// survey; where it goes and what it does with a hit is the plan's,
    /// because every step rebuilds the graph under it.
    BandScan(crate::chain::BandScan),
    /// Upload what is heard to wigle.net as this account, or `None` to stop.
    /// The feed is a node on the packet bus, so this is a command like the
    /// survey rather than a setting the interface keeps to itself.
    Wigle(Option<survey::Account>),
    /// Submit what is heard to beaconDB, or stop. Another node on the packet
    /// bus, and a command for the same reason the WiGLE feed is one.
    BeaconDb(bool),
    /// Publish every device heard to this MQTT broker, so Home Assistant
    /// builds them, or `None` to stop. A node on the same bus again.
    HomeAssistant(Option<nodes::Publish>),
    /// Read the receiver's own position from this GPS, or `None` for the
    /// local gpsd, which is what the reader looks for on its own. There is no
    /// off: a fix moves the station position, and no fix leaves it alone.
    ///
    /// The reader itself is not on this thread. This only names it, which is
    /// why choosing a GPS works with no radio running.
    Gps(Option<gps::Transport>),
    /// Log every burst the front ends detect to this directory, or stop.
    ///
    /// Sent to the radio thread rather than kept in the interface because the
    /// log is a node in the graph: what it writes is what the demodulators
    /// produced, which never reaches the interface at all.
    PacketLog(Option<std::path::PathBuf>),
    /// Where every over heard is kept, or nothing to stop keeping them.
    RecordCalls(Option<std::path::PathBuf>),
    /// Whether what is heard is read back into words, with which weights and
    /// on which device.
    Transcribe {
        on: bool,
        model: String,
        device: String,
    },
    /// Size the packet log's folder may reach, or `None` to let it grow until
    /// the disk says otherwise. The oldest days go to keep it under.
    PacketLogCap(Option<u64>),
    /// Packet feeds from other receivers, as the complete set: the graph is
    /// rebuilt from a plan, so a change is the new list rather than an
    /// instruction to add or remove one.
    Feeds(Vec<nodes::FeedSpec>),
    /// The scanner table, as the complete set for the same reason feeds are:
    /// the graph is rebuilt from a plan, so a change is the new table rather
    /// than an instruction to edit one row of it.
    Scanners(crate::scanners::Scanners),
    /// Which sound devices to use, by name, or empty for the system default.
    ///
    /// Both go to the radio thread because both belong to it: the mix is
    /// written from there and the microphone is opened there, for the length
    /// of an over and no longer.
    Audio {
        out: String,
        input: String,
    },
    /// What the agent says, as a source the transmit chain reads when a
    /// channel is set to [`TxSource::Agent`].
    Voice(std::sync::Arc<dyn audio::AudioSource>),
    /// A Flipper `.sub` file, parsed, for a channel set to
    /// [`TxSource::Sub`]. `None` clears it. Handed in whole rather than as
    /// a path so the radio thread never parses mid-over.
    SubFile(Option<SubFile>),
    /// Key a channel by id, or unkey with `None`.
    ///
    /// One command for the whole receiver rather than one per channel: every
    /// radio here that transmits is half duplex, so keying stops reception,
    /// and two channels keyed at once is not a state the hardware has.
    Key(Option<u64>),
    /// The radio's transmit gain, in dB, which a channel's own trim is added
    /// to when it is keyed.
    TxGain(f32),
    /// Whether the graph is being edited, for the view. Nothing about what
    /// runs depends on it: the operator's edits apply either way.
    Manual(bool),
    /// What the operator changed about the graph, as the whole set: it is
    /// put on top of whatever the receiver draws next, so a retune keeps it.
    Edits(crate::patch::Edits),
    /// Install a TETRA key for a cell colour on every front end, so its
    /// enciphered traffic decodes. From the key manager.
    #[cfg(feature = "tea")]
    TetraKey {
        colour: u8,
        key: decode::tea::Key,
    },
    /// Install a TA61 identity secret for a cell colour, so its encrypted
    /// identities show as real subscribers. From the key manager.
    #[cfg(feature = "tea")]
    TetraIdSecret {
        colour: u8,
        c: [u8; 8],
    },
    Stop,
}

/// One channel the receiver should be demodulating.
///
/// The whole set is sent whenever any of it changes, and the radio thread
/// works out what that means: a different frequency or mode rebuilds that
/// channel's chain, a different volume does not.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelSpec {
    /// Stable across edits, so a channel keeps its chain when its neighbour
    /// is removed. An index would not survive that.
    pub id: u64,
    /// What the strip calls it, which is what its input on the bus is
    /// called too.
    pub label: String,
    /// From the receiver's centre frequency.
    pub offset_hz: f64,
    /// What it does with that frequency: play it, or decode it.
    pub mode: ChanMode,
    /// The channel's width, or `None` for whatever the mode asks for.
    ///
    /// Set by hand when the mode's width is the wrong one: a 25 kHz repeater
    /// clipped by a 12.5 kHz filter, a crowded SSB channel that wants 2.4
    /// rather than 3 kHz, or an auto channel told to watch a band the
    /// operator picked off the spectrum rather than one a scanner block was
    /// written about.
    pub bandwidth_hz: Option<f64>,
    /// None leaves the mode's own default.
    pub squelch_db: Option<f32>,
    pub agc: bool,
    /// Treat what is heard here as speech: end an over on the squelch, put
    /// the transmission on the packet bus with its audio, and list it as a
    /// call.
    ///
    /// A property of the channel rather than of the mode, because the mode
    /// cannot know: NFM carries a repeater, a telemetry link and a paging
    /// tone alike, and only whoever tuned it knows which. What it costs when
    /// it is wrong is rows in the call list for something nobody said.
    pub voice: bool,
    /// What this channel does when it is keyed, or `None` for a channel that
    /// only listens, which is every channel until somebody says otherwise.
    ///
    /// Part of the channel rather than a list of its own because a repeater
    /// channel is one channel: it listens on the output and transmits on the
    /// input, and two entries kept in step by hand is how an operator ends up
    /// transmitting on the wrong half of the pair.
    pub tx: Option<TxSpec>,
}

impl ChannelSpec {
    /// What this channel puts on the air, whether or not anybody has said.
    ///
    /// Whether a channel can transmit is its mode's question and not a switch
    /// on the channel: it transmits in the mode it receives, so a mode with a
    /// modulator behind it transmits and one without does not. [`Self::tx`]
    /// is only what an operator changed about it, and asking for it before
    /// they had is what left a channel just added or recalled with a key on
    /// screen and no transmitter in the graph.
    pub fn spec_to_transmit(&self) -> TxSpec {
        self.tx.unwrap_or_default()
    }

    /// The width this channel is really built at.
    pub fn bandwidth(&self) -> f64 {
        // A width below a hundred hertz is a mis-set control rather than a
        // channel, and it would design a filter with thousands of taps.
        self.bandwidth_hz.filter(|b| *b >= 100.0).unwrap_or_else(|| self.mode.bandwidth())
    }

    /// The least span this channel can be built in, at its own width.
    pub fn min_rate(&self) -> f64 {
        self.mode.min_rate_for(self.bandwidth())
    }

    /// Whether a span at this rate covers the channel and can hold it.
    ///
    /// The rate is allowed a part in a million under, because a standard's
    /// rate is not always a whole number of hertz while a recording's always
    /// is: DVB-T asks for 64/7 MS/s, which is 9142857.14, and a file at
    /// 9142857 is that stream. Compared exactly, the receiver drops the
    /// channel for a seventh of a hertz.
    pub fn fits_rate(&self, rate: f64) -> bool {
        self.offset_hz.abs() <= rate / 2.0 && rate >= self.min_rate() * (1.0 - 1e-6)
    }
}

/// What a channel puts on the air when it is keyed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TxSpec {
    /// What is modulated: the microphone, or a test tone.
    pub source: TxSource,
    /// Microphone gain, as a multiplier before the modulator.
    ///
    /// Set against the meter and left alone. There is no levelling on the
    /// transmit side: an AGC has nothing to level against between words, so
    /// it winds up and puts the room on air at full deviation every time the
    /// talker pauses.
    pub mic_gain: f32,
    /// Added to the channel's receive frequency when transmitting: the
    /// repeater shift, and zero for simplex.
    pub shift_hz: f64,
    /// The tone the modulator is fed under [`TxSource::Tone`], and the tone
    /// laid over speech under [`TxSource::Mic`] when it is wanted.
    pub tone_hz: f64,
    /// This channel's own offset from the radio's transmit gain, so one
    /// channel into a dummy load and another into an antenna do not need the
    /// gain moved between them.
    pub trim_db: f32,
}

impl Default for TxSpec {
    fn default() -> Self {
        Self {
            mic_gain: 3.0,
            // A test tone, because the safe default is one that does not open
            // the microphone: keying should not put the room on air until
            // somebody has said it should.
            source: TxSource::Tone,
            shift_hz: 0.0,
            tone_hz: 1_000.0,
            trim_db: 0.0,
        }
    }
}

/// What a keyed channel puts through the modulator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxSource {
    /// A steady tone, which is what a deviation or power check wants.
    Tone,
    /// The microphone, opened when the channel is keyed and closed when it is
    /// let go. A radio that holds it open between overs is listening to the
    /// room between overs.
    Mic,
    /// What the agent has to say, from the queue it writes into. The key
    /// follows the queue: see `agent::channel`.
    Agent,
    /// A Flipper `.sub` file's pulses, keyed as they stand. The file is
    /// handed in whole by the interface, so the chain reads what was parsed
    /// rather than re-reading a path; see `Cmd::SubFile`.
    Sub,
}

impl TxSource {
    pub fn label(self) -> &'static str {
        match self {
            Self::Tone => "TONE",
            Self::Mic => "MIC",
            Self::Agent => "AGENT",
            Self::Sub => "SUB",
        }
    }
}

/// How a keyed channel is modulated.
///
/// Not a setting: it follows the channel's own mode, because a channel is one
/// frequency and one mode and a radio that receives NFM and transmits AM on
/// the same channel is a radio nobody can work. See [`tx_mode_for`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxMode {
    /// 2.5 kHz deviation, the 12.5 kHz channel standard.
    Nfm,
    /// 5 kHz deviation, on the 25 kHz grid. No receive mode selects it, so
    /// nothing transmits it outside the tests that pin its deviation.
    #[cfg_attr(not(test), allow(dead_code))]
    Fm,
    /// 75 kHz deviation, which is broadcast FM and 200 kHz wide.
    Wfm,
    /// Carrier left in, as an airband or broadcast receiver expects.
    Am,
    /// An unmodulated carrier, for measuring what the transmitter is doing.
    Carrier,
    /// A protocol with a transmit chain of its own, named by its id: the
    /// stages come off the registry rather than from the list here, because
    /// a data mode's source is not a microphone and its modulator is not one
    /// of these four.
    Digital(&'static str),
}

/// What a channel transmits, from what it receives.
///
/// `None` for a mode with no modulator behind it yet: single sideband needs
/// one, and a decode channel needs the protocol's encoder. Refusing is the
/// point, since the alternative is keying up in a mode the other end cannot
/// read.
pub fn tx_mode_for(mode: &ChanMode) -> Option<TxMode> {
    match mode {
        ChanMode::Audio(Demod::Nfm) => Some(TxMode::Nfm),
        ChanMode::Audio(Demod::Wfm) => Some(TxMode::Wfm),
        ChanMode::Audio(Demod::Am) => Some(TxMode::Am),
        ChanMode::Audio(Demod::Cw) => Some(TxMode::Carrier),
        ChanMode::Audio(Demod::Usb | Demod::Lsb) => None,
        ChanMode::Auto => None,
        // Whether a decoder transmits is the decoder's own answer, not a
        // list kept here.
        ChanMode::Decode(kind) => nodes::protocol::all()
            .iter()
            .find(|p| p.id() == kind && p.transmit().is_some())
            .map(|p| TxMode::Digital(p.id())),
    }
}

impl TxMode {
    /// Whether what it transmits is data rather than audio, which decides
    /// what its chain is made of and whether it has a meter to show before
    /// the key goes down.
    pub fn is_digital(self) -> bool {
        matches!(self, Self::Digital(_))
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Nfm => "NFM",
            Self::Fm => "FM",
            Self::Wfm => "WFM",
            Self::Am => "AM",
            Self::Carrier => "CW",
            Self::Digital(id) => nodes::protocol::all()
                .iter()
                .find(|p| p.id() == id)
                .map(|p| p.label())
                .unwrap_or(id),
        }
    }
}

/// What one running channel is doing, for its controls to show.
#[derive(Clone, Copy, Debug, Default)]
pub struct ChannelState {
    pub id: u64,
    pub agc_gain_db: f32,
    pub squelch_open: bool,
    pub squelch_db: f32,
    pub stereo_blend: f32,
    /// What it is putting into the mix, at its own fader setting, for the
    /// meter beside that fader.
    pub level: f32,
}

/// One spectrum update.
pub struct Frame {
    pub db: Vec<f32>,
    pub adc: nodes::AdcHealth,
    pub center: f64,
    pub rate: f64,
    /// Spectrum stages the operator added, each covering whatever was wired
    /// into it rather than the span.
    pub extra: Vec<Spectrum>,
}

/// One extra spectrum, as the interface draws it.
#[derive(Clone, Debug, PartialEq)]
pub struct Spectrum {
    /// The patch stage it belongs to.
    pub tag: u64,
    pub db: Vec<f32>,
    pub center: f64,
    pub rate: f64,
}

/// One decoded packet, as the UI logs and draws it.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodeRecord {
    /// When it was decoded, for ordering the log and placing the waterfall
    /// mark.
    pub at: std::time::Instant,
    /// Centre of the channel it arrived on. Not the tuned frequency: the whole
    /// point is that these come from wherever in the span they happened.
    pub freq: f64,
    /// Width of that channel, which differs between the two banks and is what
    /// says how far apart two reports have to be to be different bursts.
    pub channel_hz: f64,
    /// The protocol that claimed the burst, by the name its decoder
    /// publishes, or `None` for a burst nothing claimed.
    pub model: Option<&'static str>,
    /// How it was keyed.
    pub modulation: common::Modulation,
    /// Fields for a decode, inferred coding and timings for an unknown.
    pub detail: String,
    /// The same fields, structured.
    ///
    /// This is what makes the packet list a bus rather than a display: a map,
    /// a chart or an image pane reads these rather than the bytes or the
    /// summary line.
    pub fields: Vec<(String, common::Value)>,
    /// What the payload is, as a media type, so a view can claim packets it
    /// knows how to render without knowing the protocol that made them.
    pub media_type: &'static str,
    /// Whether somebody wrote it. The message view's entry condition; see
    /// [`common::Decoded::written`].
    pub written: bool,
    /// Received level in dBFS, and signal to noise in dB.
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    pub bytes: Vec<u8>,
    /// `None` when the protocol has no integrity check, which must stay
    /// visible: an unchecked decode from a noisy band is often wrong.
    pub crc: Option<bool>,
    /// Who the transmission was between, as the decoder named them. The
    /// links directory is built from this and not from the fields: a party
    /// is a kind and an identifier, and reading one out of a display string
    /// is how a talkgroup and a callsign end up in the same row.
    pub link: Option<pipeline::event::Link>,
    /// What the transmission said about the transmitter besides where it was,
    /// as the decoder recovered it: an aircraft's altitude, a vessel's
    /// heading, a handset's sticks. Typed for the same reason `link` is.
    pub report: common::ReportDetail,
    /// Who transmitted, where the decoder could say. The device database rows
    /// on this, and a view holding one row per transmitter needs it for the
    /// same reason: an identifier read out of a display field is a number two
    /// protocols can both produce.
    pub identity: Option<common::Identity>,
    /// The burst's samples, for the view that shows a packet, when the
    /// front end kept them.
    pub iq: Option<std::sync::Arc<common::IqBurst>>,
    /// The keying as the front end timed it, for a burst that arrived as
    /// widths rather than as bytes. What a `.sub` file is written from: for
    /// a keyed remote the widths are the signal, and the samples are far
    /// too much to keep for every row.
    pub pulses: Option<std::sync::Arc<common::Package>>,
    /// What was said, for a voice protocol. This is the payload of such a
    /// transmission: the bytes of a vocoded stream say nothing to anybody.
    pub audio: Option<std::sync::Arc<common::Speech>>,
    /// How long it held the channel, whether it carried speech, and what
    /// protects it. What the call list is built from, as the decoder said it
    /// rather than as a reader guessed from field names.
    pub airtime: Option<common::Airtime>,
}

impl DecodeRecord {
    /// The column headings [`Self::line`] prints under.
    pub fn line_header() -> String {
        format!(
            "{:>8}  {:>13}  {:<10} {:>6} {:>5}  {:<22} {:>3}  info",
            "time", "frequency", "mod", "rssi", "snr", "protocol", "len"
        )
    }

    /// One line in the packet list's columns, timed from `since`.
    pub fn line(&self, since: std::time::Instant) -> String {
        format!(
            "{:>8.3}  {:>9.4} MHz  {:<10} {:>6.1} {:>5.1}  {:<22} {:>3}  {}",
            self.at.saturating_duration_since(since).as_secs_f64(),
            self.freq / 1e6,
            self.modulation,
            self.rssi_dbfs,
            self.snr_db,
            self.protocol(),
            self.bytes.len(),
            self.detail
        )
    }

    /// What the row is named as, for a person reading it: the protocol that
    /// claimed the burst, or that nothing did.
    pub fn protocol(&self) -> &'static str {
        self.model.unwrap_or(nodes::UNKNOWN)
    }

    /// The system a call, a message or a link on this row belongs to:
    /// `M17-Voice` and `M17-Packet` are both M17, so every mode of one
    /// system shares a row wherever rows are folded together.
    pub fn system(&self) -> &'static str {
        let name = self.protocol();
        name.split('-').next().unwrap_or(name)
    }

    /// A bare record, for tests that need one to hand to something else.
    #[cfg(test)]
    pub fn for_test(freq: f64, model: &'static str) -> Self {
        Self {
            at: std::time::Instant::now(),
            freq,
            channel_hz: 31_250.0,
            model: (model != nodes::UNKNOWN).then_some(model),
            modulation: common::Modulation::Ook,
            detail: String::new(),
            fields: Vec::new(),
            media_type: pipeline::event::media::BYTES,
            written: false,
            rssi_dbfs: -20.0,
            snr_db: 15.0,
            bytes: vec![1, 2, 3],
            crc: Some(true),
            link: None,
            report: common::ReportDetail::Bare,
            identity: None,
            iq: None,
            pulses: None,
            audio: None,
            airtime: None,
        }
    }

    /// Whether any protocol claimed this burst.
    pub fn is_known(&self) -> bool {
        self.model.is_some()
    }
}

/// Scan a buffer while recording, as the radio thread does. Test support.
/// A receiver set up to sweep a capture, the way the live one sweeps the air.
pub(crate) fn replay_receiver(
    buf: &common::IqBuf,
    rec: Option<crate::record::Recorder>,
) -> anyhow::Result<crate::chain::Receiver> {
    // A 1090 MHz capture goes through the wideband path instead of the
    // channel banks, the same way the live receiver decides: 1090 carries
    // nothing the ISM banks understand, so running them there only spends CPU
    // inventing unknown bursts out of Mode S.
    // A capture goes through whatever front end the scanner table puts on
    // its frequency, the same way the live receiver decides.
    let plan = replay_plan(buf, rec.is_some());
    Ok(crate::chain::Receiver::build(
        &plan,
        crate::chain::Sinks { recorder: rec, ..Default::default() },
    )?)
}

/// The plan a capture is replayed under, as the live receiver would decide
/// it from the scanner table.
pub(crate) fn replay_plan(buf: &common::IqBuf, record: bool) -> Plan {
    let rate = buf.rate.as_f64();
    let scanners = crate::scanners::Scanners::load();
    let fronts = scanners.fronts(buf.center.as_f64(), rate);
    Plan {
        center: buf.center,
        rate,
        zoom: 1,
        // A file has already been through whatever the receiver did to it.
        dc_block: false,
        refresh_hz: 30.0,
        smoothing: crate::chain::DEFAULT_SMOOTHING,
        fft: 1024,
        channels: Vec::new(),
        fronts,
        feeds: Vec::new(),
        tx: None,
        scan: Default::default(),
        heat: Default::default(),
        edits: Default::default(),
        record,
        capture: false,
        capture_dir: crate::chain::default_capture_dir(),
        capture_format: common::SampleFormat::Cu8,
        log: false,
        calls: None,
        transcribe: false,
        transcribe_model: String::new(),
        transcribe_device: String::new(),
        settings: Default::default(),
    }
}

/// The file format a device's samples are captured in.
///
/// Its own depth where that is a file format, and sixteen bit signed where
/// the driver hands over floats: the converters behind those are twelve to
/// fourteen bits, so sixteen loses nothing and floats would double the file
/// to carry zeros.
fn capture_format_for(native: common::SampleFormat) -> common::SampleFormat {
    use common::SampleFormat::*;
    match native {
        Cu8 => Cu8,
        Cs8 => Cs8,
        Cs16 | Cf32 => Cs16,
    }
}

/// When a block's signal arrived, given the moment its processing finished.
///
/// A decode is stamped with the start of the block that carried it rather than
/// the moment the decoder finished with it. The burst is somewhere inside
/// those samples, and a stamp taken afterwards drifts by however long decoding
/// took, which on a loaded machine is longer than the block itself.
fn block_start(finished: std::time::Instant, samples: usize, rate: f64) -> std::time::Instant {
    finished - std::time::Duration::from_secs_f64(samples as f64 / rate.max(1.0))
}

/// What one block decoded to, and what the recorder should keep of it.
///
/// One place, used by the live loop and by a replay, because a replay that
/// harvested differently would be evidence about a different receiver. The
/// copies of a burst other channels read are already gone: the dedupe is a
/// node in the graph, so every consumer of the bus sees the rows this
/// returns.
pub(crate) fn harvest(
    rx: &mut crate::chain::Receiver,
    at: std::time::Instant,
) -> Vec<DecodeRecord> {
    let found = rx.decodes(at);
    if let Some(r) = rx.recorder_mut() {
        for d in &found {
            r.capture(d);
        }
    }
    found
}

/// Sweep a capture as the radio thread does, block by block.
///
/// Blocks are the size the radio delivers, because deduplication depends on
/// how a burst falls across block boundaries and a whole-file call would not
/// exercise it.
pub(crate) fn replay_blocks(
    rx: &mut crate::chain::Receiver,
    buf: &common::IqBuf,
) -> Vec<DecodeRecord> {
    let mut out = Vec::new();
    let rate = buf.rate.as_f64().max(1.0);
    for block in buf.samples.chunks(16_384) {
        if rx.process(block).is_err() {
            break;
        }
        let at = block_start(std::time::Instant::now(), block.len(), rate);
        out.extend(harvest(rx, at));
    }
    out
}

/// Scan a buffer while recording, as the radio thread does. Test support.
#[cfg(test)]
pub fn scan_with_recorder(
    buf: &common::IqBuf,
    rec: crate::record::Recorder,
) -> (Vec<DecodeRecord>, Option<crate::record::Recorder>) {
    let mut rx = match replay_receiver(buf, Some(rec)) {
        Ok(rx) => rx,
        Err(_) => return (Vec::new(), None),
    };
    let out = replay_blocks(&mut rx, buf);
    (out, rx.take_recorder())
}

/// Run a capture through the same chain the live receiver uses.
///
/// The point of recording bursts is to be able to try again without waiting
/// for a device to transmit, so replay has to go through the same code the
/// receiver does, not a simplified copy of it.
pub fn replay(path: impl AsRef<std::path::Path>) -> anyhow::Result<Vec<DecodeRecord>> {
    let src = sources::FileSource::open(path.as_ref())?;
    let buf = src.read_all()?;
    let mut rx = replay_receiver(&buf, None)?;
    Ok(replay_blocks(&mut rx, &buf))
}

/// A source the detector has, or recently had, open.
#[derive(Clone, Copy, Debug)]
pub struct SeenSource {
    pub source: crate::chain::LiveSource,
    pub last_seen: std::time::Instant,
    pub live: bool,
}

/// How long a closed source stays on the waterfall.
pub const SOURCE_LINGER: std::time::Duration = std::time::Duration::from_secs(6);

/// Blocks of speed history kept for the sparkline in the head. At a few
/// hundred blocks a second this is a second or two of the recent past, which
/// is as far back as a reading anybody can act on goes.
pub const SPEED_HISTORY: usize = 96;

/// The levels as the nodes hold them, with the revision that moves whenever
/// something other than the strip changed one.
///
/// A revision rather than a comparison: the strip sends its own levels down
/// and reads these back, and only a change it did not make should move its
/// faders.
#[derive(Clone, Debug, Default)]
pub struct Levels {
    pub rev: u64,
    pub audio: crate::chain::MixLevels,
    pub channels: Vec<crate::chain::ChannelLevels>,
}

/// Every fader in the running graph, as the strip draws them. A level is
/// set on one by its stage id, the same route the chain view uses.
#[derive(Clone, Debug, Default)]
pub struct Strips {
    pub inputs: Vec<crate::chain::StripState>,
}

pub struct Status {
    pub dropped: AtomicU64,
    pub running: AtomicBool,
    pub audio_backlog: AtomicU64,
    /// How fast the graph ran each block against the time that block
    /// covered, newest last. One is exactly real time: below it the receiver
    /// cannot keep up and the radio will start dropping samples, and how far
    /// above it the trace sits is the headroom left for another channel.
    speed: parking_lot::Mutex<std::collections::VecDeque<f32>>,
    pub error: parking_lot::Mutex<Option<String>>,
    /// What the last rebuild could not put in the graph: a front end the span
    /// cannot hold, a channel too near its edge.
    ///
    /// Its own slot rather than a second use of `error`, because the two are
    /// different in kind. A fault happened once; this is a standing verdict on
    /// the graph that is running, republished by every rebuild, and it used to
    /// be written over `error` a few lines after a refused edit had been
    /// reported there, so the operator never saw why their edit went back.
    pub refused: parking_lot::Mutex<Option<String>>,
    /// Stereo separation currently applied, as f32 bits.
    blend: AtomicU32,

    /// The radio's own controls, republished whenever one of them moves.
    radio: parking_lot::Mutex<RadioControls>,
    /// What each running channel is doing, one entry per channel.
    channels: parking_lot::Mutex<Vec<ChannelState>>,
    /// Station name, programme type and radiotext per channel, for the WFM
    /// channels that are decoding RDS. Keyed by channel id: two channels on
    /// two stations each have their own, and sharing one would print the
    /// first channel's name over every other.
    stations: parking_lot::Mutex<Vec<(u64, StationInfo)>>,
    /// The picture the video bus is publishing, when anything is producing
    /// one.
    ///
    /// One field rather than a stream: a field is half a megabyte, the
    /// interface redraws when it likes, and a viewer that is two fields
    /// behind is showing something that was true 40 ms ago. The frame is the
    /// `Arc` the front end made, so republishing it copies a pointer.
    video: parking_lot::Mutex<Option<common::VideoFrame>>,
    /// Every input of the video bus: which one, what it is called, and how
    /// complete its last picture was. What a pane offers to switch between.
    video_inputs: parking_lot::Mutex<Vec<crate::chain::VideoInput>>,
    /// The television multiplexes being decoded, and the services on them.
    multiplexes: parking_lot::Mutex<Vec<crate::chain::Multiplex>>,
    /// Pictures written to disk, newest last, so a view can say where they
    /// went without watching the directory itself.
    pictures: parking_lot::Mutex<Vec<std::path::PathBuf>>,
    /// Shape of the chain currently demodulating, republished on every rebuild.
    chain: parking_lot::Mutex<Option<pipeline::graph::Topology>>,
    /// What each scope stage in the chain is seeing, by node id.
    scopes: parking_lot::Mutex<Vec<(usize, nodes::ScopeFrame)>>,
    /// Delay through that chain in milliseconds, as f32 bits.
    chain_latency: AtomicU32,
    /// Packets decoded across the whole span since the radio started.
    pub decoded: AtomicU64,
    /// Channels each bank is splitting the span into, zero when decoding is
    /// off. Narrow ones run the OOK front end, wide ones the FSK front end.
    pub scan_channels: AtomicU64,
    pub scan_channels_wide: AtomicU64,
    /// Whether a band is being watched for sources, and the sources open
    /// right now or closed within the last few seconds, republished every
    /// block for the waterfall to mark.
    ///
    /// The recently closed ones are the point. A sensor's burst lasts tens
    /// of milliseconds, and a mark that only lasts as long as the source is
    /// open is a flash nobody can read.
    pub sources_on: AtomicBool,
    pub sources: parking_lot::Mutex<Vec<SeenSource>>,
    /// Aircraft whose address has proved itself, when tuned to 1090 MHz.
    pub aircraft: AtomicU64,
    /// The aircraft the tracker in the graph is holding, republished at the
    /// display's frame rate.
    pub track_list: parking_lot::Mutex<Vec<crate::tracks::Track>>,
    /// The transcriber itself: which model, where it is, what it is running
    /// on and whether it is reading anything. `None` where the graph has no
    /// transcriber, which is every build made without the `stt` feature.
    pub transcriber: parking_lot::Mutex<Option<crate::transcripts::Engine>>,
    /// The call recorder: whether it is on, what it has written, and where.
    /// `None` until a receiver is built.
    pub recorder: parking_lot::Mutex<Option<crate::calllog::Recorder>>,
    /// What has been said, as the receiver's own transcript rather than a
    /// copy of it: the node writes into this from the radio thread and the
    /// view takes a snapshot when its sequence number moves. Empty until a
    /// receiver is built, which is what an interface with no radio shows.
    pub transcript: parking_lot::Mutex<crate::transcripts::SharedLog>,
    /// What the raw span capture has written, and where. Off unless somebody
    /// switched it on, which is the usual state.
    pub capture_on: AtomicBool,
    pub capture_bytes: AtomicU64,
    /// What the whole capture folder holds, which is what the limit is on.
    pub capture_folder: AtomicU64,
    pub capture_full: AtomicBool,
    pub capture_file: parking_lot::Mutex<Option<String>>,
    /// Size of the day's log file, and whether it has stopped growing.
    pub log_bytes: AtomicU64,
    pub log_full: std::sync::atomic::AtomicBool,
    /// What each packet feed is doing, for the packet log settings.
    pub feeds: parking_lot::Mutex<Vec<crate::chain::FeedStatus>>,
    /// Whether anything the tracker can resolve a position from is running,
    /// locally or from a feed.
    pub tracking: AtomicBool,
    /// Software zoom currently applied, 1 for none.
    pub zoom: AtomicU64,
    /// Whether the operator owns the shape of the graph.
    pub manual: AtomicBool,
    /// Whether this radio can transmit at all, so the strip knows whether to
    /// offer a key at all rather than offering one that always fails.
    pub can_transmit: AtomicBool,
    /// The channel being transmitted on, or zero for none. Half duplex, so
    /// there is one of these and not one per channel: the radio cannot key
    /// two channels at once and the interface should not be able to say so.
    pub keyed: AtomicU64,
    /// Transfers the radio sent as silence during the last transmission.
    pub tx_underruns: AtomicU64,
    /// What the microphone is hearing, as f32 bits.
    pub mic_level: AtomicU32,
    /// The microphone is arriving clipped from the capture side.
    pub mic_clipped: AtomicBool,
    /// The radio's transmit gain, in dB, as the device took it.
    pub tx_gain_db: AtomicU32,
    /// The levels as the nodes hold them, republished when a setting made
    /// through the chain view changed one, so the strip can follow.
    levels: parking_lot::Mutex<Levels>,
    /// The patch the receiver is actually running, which is not always the
    /// one last sent: an edit that will not build is refused and the previous
    /// one goes back.
    patch: parking_lot::Mutex<Option<(crate::patch::Patch, crate::patch::Patch)>>,
    /// Bumped whenever the radio thread replaces it, so the interface can
    /// tell its own edit from one being handed back.
    pub patch_rev: AtomicU64,
    /// Bursts written to the packet log since the receiver started.
    pub logged: AtomicU64,
    /// What the survey holds, and how many receptions have been attributed to
    /// a device since the receiver started. Zero when nothing is recording.
    pub survey_devices: AtomicU64,
    pub survey_sightings: AtomicU64,
    pub survey_heard: AtomicU64,
    /// What the WiGLE feed is doing: what is spooled, what has been sent, and
    /// why the last attempt failed.
    pub wigle: parking_lot::Mutex<Option<nodes::WigleStatus>>,
    /// The same for the beaconDB feed.
    pub beacondb: parking_lot::Mutex<Option<nodes::BeaconDbStatus>>,
    /// The walk over a band: where it is, what it has heard and whether it
    /// has stopped on something.
    pub band_scan: parking_lot::Mutex<Option<nodes::ScanStatus>>,
    /// What the heatmap holds and where the last export went.
    pub heatmap: parking_lot::Mutex<Option<crate::heatmap::HeatmapStatus>>,
    /// And for the feed into the house: the broker, whether it is up, and
    /// how many devices have been announced to it.
    pub homeassistant: parking_lot::Mutex<Option<nodes::HomeAssistantStatus>>,
    /// Every input of the bus, and where the bus is in the graph, so a strip
    /// the operator drew can be given a level by the same route the chain
    /// view uses.
    strips: parking_lot::Mutex<Strips>,
    /// What each voice source put into the mix last block, by the
    /// conversation it belongs to. The meter on a call's own row, which
    /// separates "nothing was decoded" from "it was decoded and you still
    /// cannot hear it": two different faults that sound identical.
    call_levels: parking_lot::Mutex<Vec<(common::ConversationKey, f32)>>,
    /// What the bus mixed last block, labelled: what is being heard now.
    playing: parking_lot::Mutex<Vec<crate::mix::bus::Playing>>,
    /// Who the bus is hearing, and who it has just stopped hearing, since the
    /// interface last took them. Appended by the radio thread every block
    /// and drained by the interface every frame: the ending of a call is
    /// reported once and must not be lost between two frames.
    pub heard: parking_lot::Mutex<Vec<crate::mix::heard::LiveCall>>,
    /// The TETRA cells heard and their key state, for the key manager.
    tetra_keys: parking_lot::Mutex<Vec<nodes::tetra_nodes::KeyStatus>>,
    /// Peak of the whole mix as it left for the speaker, and of the call
    /// bus's share of it, for the meters beside the master and call faders.
    out_level: AtomicU32,
    call_level: AtomicU32,
    /// What the call bus's gain control is adding, in dB, as f32 bits.
    call_gain_db: AtomicU32,
    /// Seconds of playback left in the replay stage, as f32 bits: what the
    /// strip shows a playback by, since a replay is on no channel.
    replay_left_s: AtomicU32,
}

/// Everything the radio itself can be set to, and what it is set to now.
///
/// Read back from the driver rather than remembered by the interface, because
/// the hardware quantises: ask an R820T for 30 dB and it gives 29.7, ask a
/// HackRF's LNA for 20 and it gives 16. A control showing the request rather
/// than the result is lying about the receiver.
#[derive(Clone, Debug)]
pub struct RadioControls {
    pub stages: Vec<(common::GainStage, GainMode)>,
    /// The transmit gain stages, empty on a receiver. Kept apart from the
    /// receive ones because they are different hardware: a HackRF's transmit
    /// chain shares nothing with its LNA and baseband VGA.
    pub tx_stages: Vec<common::GainStage>,
    pub toggles: Vec<common::Toggle>,
    pub choices: Vec<common::Choice>,
    /// Read by the agent surface, which reports the whole control set; the
    /// settings modal keeps its own copy.
    #[cfg_attr(not(feature = "mcp"), allow(dead_code))]
    pub ppm: f64,
    /// What the dial reads above the tuner, in hertz, and zero for an aerial
    /// straight into the radio.
    #[cfg_attr(not(feature = "mcp"), allow(dead_code))]
    pub offset: f64,
    /// Where the tuner reaches, in hertz: the lowest and highest of its
    /// ranges. What the dial is clamped to, which used to be the RTL-SDR's
    /// 24 to 1766 MHz whatever radio was connected.
    pub reach: (f64, f64),
    /// Where it transmits, or `None` for a radio that does not. Published
    /// beside the receive reach because a control that offers to key a
    /// frequency the radio cannot reach is a control that fails when it is
    /// pressed.
    pub tx_reach: Option<(f64, f64)>,
    /// Whether the tuner can be moved at all. A network stream is pinned by
    /// whoever feeds it, and its dial is a readout.
    pub tunable: bool,
}

impl Default for RadioControls {
    fn default() -> Self {
        Self {
            stages: Vec::new(),
            tx_stages: Vec::new(),
            toggles: Vec::new(),
            choices: Vec::new(),
            ppm: 0.0,
            offset: 0.0,
            reach: (24e6, 1766e6),
            tx_reach: None,
            tunable: true,
        }
    }
}

impl RadioControls {
    /// The correction and the converter come off the device rather than out
    /// of the driver: a driver that cannot correct itself reports zero, which
    /// would throw away what was just typed, and the reach on the aerial's
    /// side is not the reach of the tuner.
    fn read(dev: &dyn common::Device) -> Self {
        let (ppm, offset) = (dev.asked_ppm(), dev.offset());
        let now = dev.gains();
        let stages = dev
            .info()
            .gain_stages
            .iter()
            .map(|st| {
                let mode = now
                    .iter()
                    .find(|(n, _)| *n == st.name)
                    .map(|(_, m)| *m)
                    .unwrap_or(GainMode::Manual(*st.range.start()));
                (st.clone(), mode)
            })
            .collect();
        let tx_stages = dev.info().tx.as_ref().map(|t| t.gain_stages.clone()).unwrap_or_default();
        Self {
            stages,
            tx_stages,
            toggles: dev.toggles(),
            choices: dev.choices(),
            ppm,
            offset,
            // Already on the aerial's side of the converter: the front end
            // moves the ranges it reports with the offset.
            reach: dev.reach(),
            tx_reach: dev.info().tx.as_ref().and_then(|t| {
                let offset = dev.tuning().offset;
                let lo =
                    t.ranges.iter().map(|r| r.range.start().as_f64()).fold(f64::INFINITY, f64::min);
                let hi = t.ranges.iter().map(|r| r.range.end().as_f64()).fold(0.0f64, f64::max);
                (lo.is_finite() && hi > lo)
                    .then(|| ((lo + offset).max(0.0), (hi + offset).max(0.0)))
            }),
            // A stream is pinned by whoever feeds it and a capture by
            // whoever recorded it; both dials are readouts.
            tunable: !matches!(
                dev.info().kind,
                common::device::DriverKind::IqStream | common::device::DriverKind::File
            ),
        }
    }
}

/// What the UI shows about the tuned station.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct StationInfo {
    pub pi: Option<u16>,
    pub name: Option<String>,
    pub pty: Option<&'static str>,
    pub radiotext: Option<String>,
    /// Groups accepted and blocks rejected. The ratio is the honest measure of
    /// RDS reception: a station can be loud and still undecodable.
    pub groups: u64,
    pub block_errors: u64,
    pub synced: bool,
}

impl StationInfo {
    pub fn is_empty(&self) -> bool {
        self.pi.is_none() && self.name.is_none() && self.radiotext.is_none()
    }
}

impl Default for Status {
    fn default() -> Self {
        Self {
            dropped: AtomicU64::new(0),
            running: AtomicBool::new(false),
            audio_backlog: AtomicU64::new(0),
            speed: parking_lot::Mutex::new(std::collections::VecDeque::with_capacity(
                SPEED_HISTORY,
            )),
            survey_devices: AtomicU64::new(0),
            survey_sightings: AtomicU64::new(0),
            survey_heard: AtomicU64::new(0),
            wigle: parking_lot::Mutex::new(None),
            beacondb: parking_lot::Mutex::new(None),
            band_scan: parking_lot::Mutex::new(None),
            heatmap: parking_lot::Mutex::new(None),
            homeassistant: parking_lot::Mutex::new(None),
            strips: parking_lot::Mutex::new(Strips::default()),
            call_levels: parking_lot::Mutex::new(Vec::new()),
            playing: parking_lot::Mutex::new(Vec::new()),
            heard: parking_lot::Mutex::new(Vec::new()),
            tetra_keys: parking_lot::Mutex::new(Vec::new()),
            out_level: AtomicU32::new(0),
            call_level: AtomicU32::new(0),
            call_gain_db: AtomicU32::new(0),
            replay_left_s: AtomicU32::new(0),
            error: parking_lot::Mutex::new(None),
            refused: parking_lot::Mutex::new(None),
            blend: AtomicU32::new(0),

            radio: parking_lot::Mutex::new(RadioControls::default()),
            channels: parking_lot::Mutex::new(Vec::new()),
            stations: parking_lot::Mutex::new(Vec::new()),
            video: parking_lot::Mutex::new(None),
            video_inputs: parking_lot::Mutex::new(Vec::new()),
            multiplexes: parking_lot::Mutex::new(Vec::new()),
            pictures: parking_lot::Mutex::new(Vec::new()),
            chain: parking_lot::Mutex::new(None),
            scopes: parking_lot::Mutex::new(Vec::new()),
            chain_latency: AtomicU32::new(0),
            decoded: AtomicU64::new(0),
            scan_channels: AtomicU64::new(0),
            scan_channels_wide: AtomicU64::new(0),
            sources_on: AtomicBool::new(false),
            sources: parking_lot::Mutex::new(Vec::new()),
            aircraft: AtomicU64::new(0),
            logged: AtomicU64::new(0),
            track_list: parking_lot::Mutex::new(Vec::new()),
            transcriber: parking_lot::Mutex::new(None),
            recorder: parking_lot::Mutex::new(None),
            transcript: parking_lot::Mutex::new(Default::default()),
            capture_on: AtomicBool::new(false),
            capture_bytes: AtomicU64::new(0),
            capture_folder: AtomicU64::new(0),
            capture_full: AtomicBool::new(false),
            capture_file: parking_lot::Mutex::new(None),
            log_bytes: AtomicU64::new(0),
            log_full: std::sync::atomic::AtomicBool::new(false),
            feeds: parking_lot::Mutex::new(Vec::new()),
            tracking: AtomicBool::new(false),
            zoom: AtomicU64::new(1),
            manual: AtomicBool::new(false),
            can_transmit: AtomicBool::new(false),
            keyed: AtomicU64::new(0),
            tx_underruns: AtomicU64::new(0),
            mic_level: AtomicU32::new(0),
            mic_clipped: AtomicBool::new(false),
            tx_gain_db: AtomicU32::new(0),
            patch: parking_lot::Mutex::new(None),
            levels: parking_lot::Mutex::new(Levels::default()),
            patch_rev: AtomicU64::new(0),
        }
    }
}

impl Status {
    /// Publish the patch the receiver is running, and the one it drew for
    /// itself underneath the operator's edits.
    fn set_patch(&self, rx: &crate::chain::Receiver) {
        *self.patch.lock() = Some((rx.patch().clone(), rx.base().clone()));
        self.patch_rev.fetch_add(1, Ordering::Relaxed);
    }

    /// The patch the receiver is running, the one it drew before the edits,
    /// and which revision they are.
    pub fn patch(&self) -> (u64, Option<(crate::patch::Patch, crate::patch::Patch)>) {
        (self.patch_rev.load(Ordering::Relaxed), self.patch.lock().clone())
    }

    /// The receiver's transcript, for a view that wants to read or clear it.
    pub fn transcript(&self) -> crate::transcripts::SharedLog {
        self.transcript.lock().clone()
    }

    /// The levels as the graph holds them, and a revision that moves only
    /// when something other than the strip changed one.
    pub fn levels(&self) -> Levels {
        self.levels.lock().clone()
    }

    fn set_levels(
        &self,
        audio: crate::chain::MixLevels,
        channels: Vec<crate::chain::ChannelLevels>,
    ) {
        let mut held = self.levels.lock();
        *held = Levels { rev: held.rev + 1, audio, channels };
    }

    pub fn blend(&self) -> f32 {
        f32::from_bits(self.blend.load(Ordering::Relaxed))
    }

    /// The radio's gain stages and switches, as they currently are.
    /// What the call bus's gain control is adding, in dB.
    pub fn call_gain_db(&self) -> f32 {
        f32::from_bits(self.call_gain_db.load(Ordering::Relaxed))
    }

    /// The TETRA cells heard and their key state, for the key manager.
    pub fn tetra_keys(&self) -> Vec<nodes::tetra_nodes::KeyStatus> {
        self.tetra_keys.lock().clone()
    }

    /// What each voice source last put into the mix, for the meters.
    pub fn call_levels(&self) -> Vec<(common::ConversationKey, f32)> {
        self.call_levels.lock().clone()
    }

    /// What is being heard now: everything the bus mixed last block, by
    /// system, frequency, group and caller, with its level.
    /// What the audio bus is mixing, for anything that wants to ask. No pane
    /// draws it: the call list is where a conversation appears and the strip
    /// is where a channel does.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn playing(&self) -> Vec<crate::mix::bus::Playing> {
        self.playing.lock().clone()
    }

    /// Every fader as the strip draws it.
    pub fn strips(&self) -> Strips {
        self.strips.lock().clone()
    }

    fn set_strips(&self, mut inputs: Vec<crate::chain::StripState>) {
        let mut held = self.strips.lock();
        for s in &mut inputs {
            if let Some(prev) = held.inputs.iter().find(|p| p.stage == s.stage) {
                s.level = s.level.max(prev.level * METER_FALL);
            }
        }
        *held = Strips { inputs };
    }

    /// The mix's own level, and the call bus's share of it.
    pub fn out_level(&self) -> f32 {
        f32::from_bits(self.out_level.load(Ordering::Relaxed))
    }

    pub fn call_level(&self) -> f32 {
        f32::from_bits(self.call_level.load(Ordering::Relaxed))
    }

    /// Seconds of a played-back over still to come, or zero.
    pub fn replay_left_s(&self) -> f32 {
        f32::from_bits(self.replay_left_s.load(Ordering::Relaxed))
    }

    /// Rise instantly, fall slowly. A meter that tracked the block peak both
    /// ways flickers at the block rate and reads as noise; speech is mostly
    /// gaps, and the gaps are not what anybody is trying to see.
    fn set_level(cell: &AtomicU32, peak: f32) {
        let prev = f32::from_bits(cell.load(Ordering::Relaxed));
        cell.store(peak.max(prev * METER_FALL).to_bits(), Ordering::Relaxed);
    }

    pub fn radio(&self) -> RadioControls {
        self.radio.lock().clone()
    }

    fn set_radio(&self, c: RadioControls) {
        *self.radio.lock() = c;
    }

    /// What every running channel is doing.
    pub fn channel_states(&self) -> Vec<ChannelState> {
        self.channels.lock().clone()
    }

    /// One channel's state by id, for the controls that belong to it.
    pub fn channel_state(&self, id: u64) -> Option<ChannelState> {
        self.channels.lock().iter().find(|c| c.id == id).copied()
    }

    fn set_channel_states(&self, mut states: Vec<ChannelState>) {
        let mut held = self.channels.lock();
        for s in &mut states {
            if let Some(prev) = held.iter().find(|p| p.id == s.id) {
                s.level = s.level.max(prev.level * METER_FALL);
            }
        }
        *held = states;
    }

    fn set_blend(&self, v: f32) {
        self.blend.store(v.to_bits(), Ordering::Relaxed);
    }

    pub fn chain(&self) -> Option<pipeline::graph::Topology> {
        self.chain.lock().clone()
    }

    fn push_speed(&self, x: f32) {
        let mut h = self.speed.lock();
        if h.len() == SPEED_HISTORY {
            h.pop_front();
        }
        h.push_back(x);
    }

    /// The recent speed trace, oldest first.
    pub fn speed_history(&self) -> Vec<f32> {
        self.speed.lock().iter().copied().collect()
    }

    pub fn chain_latency(&self) -> f64 {
        f64::from(f32::from_bits(self.chain_latency.load(Ordering::Relaxed)))
    }

    /// The scopes' latest frames, for the inspector.
    pub fn scopes(&self) -> Vec<(usize, nodes::ScopeFrame)> {
        self.scopes.lock().clone()
    }

    fn set_chain(&self, t: Option<pipeline::graph::Topology>, latency_ms: f64) {
        *self.chain.lock() = t;
        self.chain_latency.store((latency_ms as f32).to_bits(), Ordering::Relaxed);
    }

    /// What one channel is receiving, or nothing when it is not decoding RDS.
    pub fn station_for(&self, id: u64) -> Option<StationInfo> {
        self.stations.lock().iter().find(|(k, _)| *k == id).map(|(_, s)| s.clone())
    }

    /// The first channel's station, for the headless probe, which runs one.
    pub fn station(&self) -> StationInfo {
        self.stations.lock().first().map(|(_, s)| s.clone()).unwrap_or_default()
    }

    fn set_station(&self, id: u64, s: &dsp::rds::Station, groups: u64, errors: u64, synced: bool) {
        let next = StationInfo {
            pi: s.pi,
            name: s.name.clone(),
            pty: s.pty_name(),
            radiotext: s.radiotext.clone(),
            groups,
            block_errors: errors,
            synced,
        };
        let mut cur = self.stations.lock();
        // Only take the write cost when something actually changed; this runs
        // on every audio block, for every channel.
        match cur.iter_mut().find(|(k, _)| *k == id) {
            Some((_, cur)) if *cur != next => *cur = next,
            Some(_) => {}
            None => cur.push((id, next)),
        }
    }

    /// The picture being received, if any.
    pub fn video(&self) -> Option<common::VideoFrame> {
        self.video.lock().clone()
    }

    /// What the video bus is receiving, whether or not it is being watched.
    pub fn video_inputs(&self) -> Vec<crate::chain::VideoInput> {
        self.video_inputs.lock().clone()
    }

    /// The television multiplexes being decoded, with their services.
    pub fn multiplexes(&self) -> Vec<crate::chain::Multiplex> {
        self.multiplexes.lock().clone()
    }

    /// Pictures written to disk this session, newest last.
    pub fn pictures(&self) -> Vec<std::path::PathBuf> {
        self.pictures.lock().clone()
    }

    fn set_video_inputs(&self, inputs: Vec<crate::chain::VideoInput>) {
        let mut cur = self.video_inputs.lock();
        if *cur != inputs {
            *cur = inputs;
        }
    }

    /// Publish a field, or clear the pane when the receiver stops producing
    /// them: a still picture left on the screen after the transmitter went
    /// away is the worst thing a video pane can do.
    ///
    /// A picture is new when anything about it is, not when its number is.
    /// The number counts fields for a camera, but names the picture for a
    /// still, so an SSTV transmission keeps one number for two minutes while
    /// its lines fill in: comparing numbers alone published the first line
    /// and nothing after it.
    fn set_video(&self, frame: Option<common::VideoFrame>) {
        let mut cur = self.video.lock();
        let same = match (cur.as_ref(), frame.as_ref()) {
            (Some(a), Some(b)) => {
                a.sequence == b.sequence
                    && a.lines_seen == b.lines_seen
                    && a.channel_hz == b.channel_hz
                    && (a.width, a.height) == (b.width, b.height)
            }
            (None, None) => true,
            _ => false,
        };
        if !same {
            *cur = frame;
        }
    }

    /// Drop the stations of channels that are no longer running, so a name
    /// cannot linger over a channel that has been retuned or removed.
    fn keep_stations(&self, ids: &[u64]) {
        let mut cur = self.stations.lock();
        cur.retain(|(id, _)| ids.contains(id));
        if cur.is_empty() {
            self.set_blend(0.0);
        }
    }
}

pub struct Radio {
    pub cmd: Sender<Cmd>,
    pub frames: Receiver<Frame>,
    /// Packets decoded anywhere in the span, in the order they were found.
    pub decodes: Receiver<Vec<DecodeRecord>>,
    pub status: Arc<Status>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Radio {
    /// Start streaming from an RTL-SDR. `repaint` is called on every frame so
    /// the UI wakes without polling.
    #[allow(clippy::too_many_arguments)]
    pub fn start(
        entry: crate::devices::Entry,
        center: Hz,
        rate: Sps,
        // The local oscillator of whatever is on the cable, which the dial
        // reads above the tuner. Zero for an aerial.
        offset: f64,
        fft: usize,
        repaint: impl Fn() + Send + 'static,
    ) -> Self {
        let (cmd_tx, cmd_rx) = bounded(64);
        // Depth 2: the UI only ever draws the newest spectrum, so queuing more
        // just adds latency between the radio and what is on screen.
        let (frame_tx, frame_rx) = bounded(2);
        // Deeper than the spectrum queue, and for the opposite reason: a
        // dropped spectrum frame is replaced 30 times a second, but a dropped
        // decode is a packet that will not come again.
        let (dec_tx, dec_rx) = bounded(64);
        let status = Arc::new(Status::default());
        let st = status.clone();

        let handle = std::thread::Builder::new()
            .name("radio".into())
            .spawn(move || {
                if let Err(e) =
                    run(entry, center, rate, offset, fft, cmd_rx, frame_tx, dec_tx, &st, repaint)
                {
                    *st.error.lock() = Some(e.to_string());
                }
                st.running.store(false, Ordering::Relaxed);
            })
            .expect("spawn radio thread");

        Self { cmd: cmd_tx, frames: frame_rx, decodes: dec_rx, status, handle: Some(handle) }
    }

    /// The whole receiver on a radio somebody else opened.
    ///
    /// For a test: `sources::FileRadio` hears a capture and keeps what it
    /// transmits, so everything from a command arriving to a sample reaching
    /// the antenna runs exactly as it does on a HackRF. Nothing above this is
    /// test-only, which is the point.
    #[cfg(test)]
    pub fn on_device(dev: Box<dyn common::Device>, center: Hz, rate: Sps, fft: usize) -> Self {
        let (cmd_tx, cmd_rx) = bounded(64);
        let (frame_tx, frame_rx) = bounded(2);
        let (dec_tx, dec_rx) = bounded(64);
        let status = Arc::new(Status::default());
        let st = status.clone();
        let handle = std::thread::Builder::new()
            .name("radio".into())
            .spawn(move || {
                let built = RadioThread::with_device(
                    dev,
                    None,
                    center,
                    rate,
                    0.0,
                    fft,
                    cmd_rx,
                    frame_tx,
                    dec_tx,
                    &st,
                    || {},
                );
                let ran = match built {
                    Ok(t) => t.run(),
                    Err(e) => Err(e),
                };
                if let Err(e) = ran {
                    *st.error.lock() = Some(e.to_string());
                }
                st.running.store(false, Ordering::Relaxed);
            })
            .expect("spawn radio thread");
        Self { cmd: cmd_tx, frames: frame_rx, decodes: dec_rx, status, handle: Some(handle) }
    }

    pub fn send(&self, c: Cmd) {
        let _ = self.cmd.try_send(c);
    }
}

/// How long a radio is given to stop before it is abandoned.
///
/// A USB call that never returns is not a thing this process can cancel, so
/// the choice is between waiting for it and leaving it. Waiting means the
/// window is frozen with nothing on screen to say why, which is how a radio
/// that stopped responding took the whole interface with it; leaving it means
/// a thread and a USB claim are held until the process exits, and the
/// operator can carry on, change device, or close the window.
const STOP_GRACE: std::time::Duration = std::time::Duration::from_millis(1500);

impl Drop for Radio {
    fn drop(&mut self) {
        self.send(Cmd::Stop);
        let Some(h) = self.handle.take() else { return };
        // Joined on another thread, so this one can give up on it. The
        // handle is moved in, so abandoning it leaks a thread rather than
        // leaving a dangling join.
        let (tx, rx) = bounded::<()>(1);
        let waiter = std::thread::Builder::new().name("radio-stop".into()).spawn(move || {
            let _ = h.join();
            let _ = tx.send(());
        });
        if waiter.is_err() {
            return;
        }
        if rx.recv_timeout(STOP_GRACE).is_err() {
            tracing::warn!(
                "the radio did not stop within {:?}; abandoning its thread and USB claim",
                STOP_GRACE
            );
        }
    }
}

/// One listening channel on its own, for tests and benchmarks.
///
/// A thin holder around a [`crate::chain::Receiver`] carrying a single
/// channel, so that measuring a chain measures the chain the receiver builds.
/// It used to construct its own copy of the audio branch, which drifted:
/// whatever was true of a filter here was not necessarily true of the one the
/// radio ran.
#[cfg_attr(not(test), allow(dead_code))]
pub struct Audio {
    rx: crate::chain::Receiver,
    pcm: Vec<f32>,
}

#[cfg_attr(not(test), allow(dead_code))]
impl Audio {
    pub fn new(offset: f64, rate: f64, mode: Demod, _target: f64) -> Self {
        let spec = ChannelSpec {
            id: 1,
            label: String::new(),
            offset_hz: offset,
            mode: ChanMode::Audio(mode),
            bandwidth_hz: None,
            squelch_db: None,
            voice: false,
            agc: true,
            tx: None,
        };
        let plan = Plan {
            center: Hz(0),
            rate,
            zoom: 1,
            dc_block: false,
            refresh_hz: 30.0,
            smoothing: crate::chain::DEFAULT_SMOOTHING,
            fft: 1024,
            channels: vec![spec],
            fronts: Vec::new(),
            edits: Default::default(),
            record: false,
            capture: false,
            heat: Default::default(),
            capture_dir: crate::chain::default_capture_dir(),
            capture_format: common::SampleFormat::Cu8,
            log: false,
            calls: None,
            transcribe: false,
            transcribe_model: String::new(),
            transcribe_device: String::new(),
            feeds: Vec::new(),
            tx: None,
            scan: Default::default(),
            settings: Default::default(),
        };
        let rx = crate::chain::Receiver::build(&plan, Default::default()).expect("audio chain");
        Self { rx, pcm: Vec::new() }
    }

    fn chan(&self) -> &crate::chain::Chan {
        &self.rx.channels()[0]
    }

    pub fn cost(&self) -> String {
        self.chan().detail.clone()
    }

    pub fn latency_ms(&self) -> f64 {
        self.rx.latency_ms(0)
    }

    /// How much gain the AGC is applying, or zero in a mode without one.
    pub fn agc_gain_db(&self) -> f32 {
        self.chan().agc_gain_db
    }

    /// What the squelch measured on the last block, in dB.
    pub fn squelch_db(&self) -> f32 {
        self.chan().squelch_db
    }

    pub fn audio_rate(&self) -> f64 {
        self.chan().audio_rate
    }

    pub fn topology(&self) -> pipeline::graph::Topology {
        self.rx.topology()
    }

    pub fn process(&mut self, input: &[C32], gain: f32) -> &[f32] {
        self.pcm.clear();
        if self.rx.process(input).is_err() {
            return &self.pcm;
        }
        self.pcm.extend(self.rx.channel_audio(0).iter().map(|v| v * gain));
        &self.pcm
    }
}

/// Whether the radio thread carries on after a step, or is finished.
enum Flow {
    Go,
    Stop,
}

/// What a read of the radio produced.
enum Block {
    Samples(common::IqBuf),
    /// The radio stopped delivering and was reopened; there is nothing to
    /// process this time round.
    Restarted,
    /// The radio did not come back.
    Lost,
}

/// The transmit side of the radio, between and during overs.
struct Tx {
    /// The radio's own transmit gain.
    gain_db: f32,
    /// A parsed `.sub` file, for a channel set to [`TxSource::Sub`]. Held
    /// beside the gain rather than in the plan because it is handed in
    /// whole, the way the agent's voice is, and a plan is copied around.
    sub_file: Option<SubFile>,
    /// Blocks since the key went down, for the note said once a second.
    blocks_since_key: u64,
    /// The channel whose key is down but whose transmitter is still being
    /// built, so the strip is not told it is on air before it is.
    keying_for: Option<u64>,
    /// The last channel that went on air, which is the one the drawn chain
    /// stays on once the key comes up. Without it the chain view fell back
    /// to whichever transmit channel comes first on the strip, so recalling
    /// a second from a bank and working it left the chain showing the other.
    last_keyed: Option<u64>,
    /// When the transmitter was last on air, so the transcriber stays deaf a
    /// moment past the key coming up: the audio already in the demodulator
    /// when the key lifted is still the receiver's own voice.
    last_on_air: Option<std::time::Instant>,
}

/// How long past the key coming up the transcriber stays deaf.
///
/// One demodulator's worth of audio in flight, not a hang time: what the
/// receiver said must not be written down, and what somebody says straight
/// after must be.
const DEAF_TAIL: std::time::Duration = std::time::Duration::from_millis(500);

/// The speaker and the microphone, and the devices they were asked for.
struct AudioIo {
    out: String,
    input: String,
    /// Held only so the output stream stays open: dropping it closes the
    /// device the sink writes to.
    _player: Option<AudioPlayer>,
    /// Open for as long as the receiver runs, so the strip's meter is live
    /// and anything that wants speech can take a tap.
    mic: Option<audio::AudioCapture>,
}

/// The radio thread: a device, the graph it feeds, and everything a command
/// changes about either.
///
/// One block is one turn of [`RadioThread::run`]: the commands that arrived,
/// a retune if one is due, a rebuild if anything asked for one, a read, the
/// graph, what is published from it, and the audio it produced.
struct RadioThread<'a, R: Fn()> {
    /// What the device was opened from, so it can be opened again. Absent on
    /// a radio handed in already open, which cannot be reopened and does not
    /// need to be.
    entry: Option<crate::devices::Entry>,
    dev: Box<dyn common::Device>,
    /// The stream the radio is delivering on. Absent only between letting one
    /// go and opening the next, which is a state the thread does not run in:
    /// a reopen that fails ends it.
    stream: Option<Box<dyn common::RxStream>>,
    /// Tracked so a restart can put it back: reopening a device resets every
    /// stage and switch, and a span change that silently returned them to
    /// their defaults would look like the antenna had fallen out.
    front: FrontEnd,
    /// What the receiver should be doing. Everything that acts on a sample is
    /// in the graph this describes, so a command changes the plan and the
    /// graph is rebuilt from it, rather than each command reaching into a
    /// different object.
    plan: Plan,
    rx: crate::chain::Receiver,
    /// What to run follows from where the dial is, and that mapping is
    /// configuration rather than structure.
    scanners: crate::scanners::Scanners,
    audio: AudioIo,
    tx: Tx,
    /// What the agent has to say, when there is an agent channel. Handed
    /// over once and read whenever such a channel is keyed.
    voice: Option<std::sync::Arc<dyn audio::AudioSource>>,
    status: &'a Status,
    cmd: Receiver<Cmd>,
    frames: Sender<Frame>,
    decodes: Sender<Vec<DecodeRecord>>,
    repaint: R,
    /// Where the dial has been asked to go, held until a retune is affordable.
    want_center: Option<Hz>,
    last_tune: std::time::Instant,
    tune_gap: std::time::Duration,
    last_chain: std::time::Instant,
    /// The last edits that built, to fall back on when an edit does not.
    last_edits: Option<crate::patch::Edits>,
    needs_rebuild: bool,
    /// The operator's own decoding switch: off, and no front end is built at
    /// all, which is the expensive thing the receiver does.
    scan_on: bool,
    records: Vec<DecodeRecord>,
    /// Everything decoded since the receiver started, which is what the
    /// counter on screen reads.
    hits: u64,
}

impl<'a, R: Fn()> RadioThread<'a, R> {
    #[allow(clippy::too_many_arguments)]
    fn open(
        entry: crate::devices::Entry,
        center: Hz,
        rate: Sps,
        offset: f64,
        fft: usize,
        cmd: Receiver<Cmd>,
        frames: Sender<Frame>,
        decodes: Sender<Vec<DecodeRecord>>,
        status: &'a Status,
        repaint: R,
    ) -> anyhow::Result<Self> {
        let dev = crate::devices::open(&entry)?;
        Self::with_device(
            dev,
            Some(entry),
            center,
            rate,
            offset,
            fft,
            cmd,
            frames,
            decodes,
            status,
            repaint,
        )
    }

    /// The same, on a radio somebody else opened.
    ///
    /// The one seam a test needs: everything above this line is a USB claim,
    /// and everything below it is the receiver. `sources::FileRadio` is a
    /// radio made of memory that hears a capture and keeps what it
    /// transmits, which is what makes keying testable at all.
    fn with_device(
        mut dev: Box<dyn common::Device>,
        entry: Option<crate::devices::Entry>,
        center: Hz,
        rate: Sps,
        offset: f64,
        fft: usize,
        cmd: Receiver<Cmd>,
        frames: Sender<Frame>,
        decodes: Sender<Vec<DecodeRecord>>,
        status: &'a Status,
        repaint: R,
    ) -> anyhow::Result<Self> {
        // What is on the cable is known before the radio is opened, and has
        // to be: with a converter the saved dial is in the Ku band, and a
        // tuner asked for that whole refuses, which used to stop the radio
        // starting at all.
        dev.set_offset(offset);
        // Clamp to what this radio can actually do: the app's last span and
        // the last dial may have come from a different device entirely.
        let info_rates = dev.info().rate_range.clone();
        let rate = Sps(rate.0.clamp(info_rates.start().0, info_rates.end().0));
        dev.set_rate(rate)?;
        let (lo, hi) = dev.reach();
        dev.set_dial(Hz(center.as_f64().clamp(lo, hi).round() as u64))?;
        dev.set_gain("tuner", GainMode::Auto)?;

        // The device the session asked for arrives as a command once the
        // interface is up, so this is the default until then.
        let (player, sink) = match cfg!(test) {
            // A test radio hears a capture as fast as the machine will run it,
            // and a keyed channel sends the 1 kHz tone: opening the speaker
            // beeps at whoever is running the tests.
            true => (None, None),
            false => match AudioPlayer::open(48_000) {
                Ok((p, s)) => (Some(p), Some(s)),
                Err(e) => {
                    *status.error.lock() = Some(format!("no audio output: {e}"));
                    (None, None)
                }
            },
        };

        let stream = dev.start_rx()?;
        status.running.store(true, Ordering::Relaxed);
        status.set_radio(RadioControls::read(dev.as_ref()));

        let mut plan = Plan {
            center: dev.dial(),
            rate: dev.rate().as_f64(),
            zoom: 1,
            dc_block: true,
            refresh_hz: 30.0,
            smoothing: crate::chain::DEFAULT_SMOOTHING,
            fft,
            channels: Vec::new(),
            // Resolved from the scanner table below, once the tuning is known.
            fronts: Vec::new(),
            // Read off disc here as well as sent by the interface: the first
            // graph is built before any command has arrived, and a switch
            // that only reached the graph on the rebuild after was off for
            // however long that took, or for good when nothing else asked
            // for a rebuild.
            edits: crate::patch::Edits::load().map(|(e, _)| e).unwrap_or_default(),
            record: false,
            capture: false,
            heat: Default::default(),
            capture_dir: crate::chain::default_capture_dir(),
            capture_format: capture_format_for(dev.info().native_format),
            // Switched on as soon as the interface says where to write; the
            // default is on, and the command arrives with the first frame.
            log: false,
            calls: None,
            transcribe: false,
            transcribe_model: String::new(),
            transcribe_device: String::new(),
            // Feeds arrive from the session or the settings modal, as a
            // command.
            feeds: Vec::new(),
            tx: None,
            scan: Default::default(),
            settings: Default::default(),
        };
        let scanners = crate::scanners::Scanners::load();
        plan.fronts = fronts_here(&scanners, &plan, true);
        let mut rx = crate::chain::Receiver::build(&plan, Default::default())?;
        rx.set_speaker(sink);
        publish_chain(status, &rx);
        *status.transcript.lock() = rx.transcript().clone();
        status.can_transmit.store(dev.info().can_transmit(), Ordering::Relaxed);

        let gap = tune_gap();
        let mut this = Self {
            entry,
            dev,
            stream: Some(stream),
            front: FrontEnd::default(),
            plan,
            rx,
            scanners,
            audio: AudioIo { out: String::new(), input: String::new(), _player: player, mic: None },
            tx: Tx {
                gain_db: 0.0,
                sub_file: None,
                blocks_since_key: 0,
                keying_for: None,
                last_keyed: None,
                last_on_air: None,
            },
            voice: None,
            status,
            cmd,
            frames,
            decodes,
            repaint,
            want_center: None,
            last_tune: std::time::Instant::now() - gap,
            tune_gap: gap,
            last_chain: std::time::Instant::now(),
            last_edits: None,
            needs_rebuild: false,
            scan_on: true,
            records: Vec::new(),
            hits: 0,
        };
        this.open_mic();
        this.remember_front();
        Ok(this)
    }

    /// Note what the front end is set to, for whatever reopens the radio.
    ///
    /// Read after each change rather than while reopening: the device that
    /// has to be put back is often one that has stopped answering, and its
    /// own account of its gain is then whatever the last read failed to get.
    fn remember_front(&mut self) {
        self.front = FrontEnd::read(self.dev.as_ref());
    }

    fn run(mut self) -> anyhow::Result<()> {
        loop {
            if let Flow::Stop = self.apply_commands() {
                return Ok(());
            }
            self.retune()?;
            if let Flow::Stop = self.rebuild() {
                return Ok(());
            }
            let buf = match self.read_block() {
                Block::Samples(b) => b,
                Block::Restarted => continue,
                Block::Lost => return Ok(()),
            };
            // Timed from here rather than around the loop: the read is where
            // the thread waits for the radio, so counting it would measure
            // real time against itself and always say exactly 1x.
            let work = std::time::Instant::now();
            let block_secs = buf.samples.len() as f64 / self.plan.rate.max(1.0);

            let _b = tracing::info_span!("block").entered();
            self.meter_mic();
            // Whether the monitor stage draws what is going out on the span.
            // Only while the radio is deaf: a full duplex one hears its own
            // transmission for real, and mirroring on top of that would draw
            // it twice.
            let silent = self.stream.as_ref().is_some_and(|s| s.silent());
            let on_air = self.rx.tx_on_air();
            self.rx.set_tx_monitor(on_air && silent);
            // And nothing reads that loopback as speech: what the receiver
            // said is not what the receiver heard.
            let now = std::time::Instant::now();
            if on_air {
                self.tx.last_on_air = Some(now);
            }
            let deaf =
                on_air || self.tx.last_on_air.is_some_and(|t| now.duration_since(t) < DEAF_TAIL);
            self.rx.set_transcriber_deaf(deaf);
            // A radio unplugged mid-over ends the over itself, and the key
            // has to come up with it: a lit key over a transmitter that
            // stopped transmitting is worse than no key at all.
            if self.rx.tx_lost() {
                *self.status.error.lock() =
                    Some("the radio stopped taking samples: the transmission ended".into());
                self.unkey();
            }

            if let Flow::Stop = self.process(&buf.samples) {
                return Ok(());
            }
            {
                let _s = tracing::info_span!("publish_spectrum").entered();
                if let Flow::Stop = self.publish_spectrum() {
                    return Ok(());
                }
            }
            {
                let _s = tracing::info_span!("publish_status").entered();
                self.publish_status();
            }

            // Stamped at the start of the block rather than at the moment the
            // decode fell out of it, by the same arithmetic a replay uses.
            let at = block_start(std::time::Instant::now(), buf.samples.len(), self.plan.rate);
            {
                let _s = tracing::info_span!("harvest").entered();
                if let Flow::Stop = self.harvest_decodes(at) {
                    return Ok(());
                }
            }
            {
                let _a = tracing::info_span!("audio").entered();
                self.meter_audio();
            }
            {
                let _s = tracing::info_span!("stations").entered();
                self.publish_stations();
            }
            self.status.push_speed((block_secs / work.elapsed().as_secs_f64().max(1e-9)) as f32);
        }
    }

    /// Every command that has arrived since the last block.
    fn apply_commands(&mut self) -> Flow {
        let batch: Vec<Cmd> = self.cmd.try_iter().collect();
        for c in batch {
            if let Flow::Stop = self.apply(c) {
                return Flow::Stop;
            }
        }
        Flow::Go
    }

    fn apply(&mut self, c: Cmd) -> Flow {
        match c {
            Cmd::Stop => {
                if let Some(s) = self.stream.as_mut() {
                    s.stop();
                }
                return Flow::Stop;
            }
            // Held rather than applied. A drag issues one of these per
            // displayed frame and only the last is worth anything, so
            // applying each in turn spends the whole budget retuning to
            // frequencies already superseded.
            Cmd::Center(f) => self.want_center = Some(f),
            Cmd::Audio { out, input } => self.set_audio_devices(out, input),
            // Held by the receiver as well, because the transmit chain is
            // drawn on every rebuild and its source stage has to be built
            // from something: handed in only at key-up, the chain that was
            // rebuilt in between came back reading the microphone.
            Cmd::Voice(src) => {
                self.voice = Some(src.clone());
                self.rx.set_agent_voice(Some(src));
                self.needs_rebuild = true;
            }
            Cmd::SubFile(f) => {
                self.tx.sub_file = f;
                self.needs_rebuild = true;
            }
            Cmd::TxGain(db) => {
                self.tx.gain_db = db.max(0.0);
                self.status.tx_gain_db.store(self.tx.gain_db.to_bits(), Ordering::Relaxed);
            }
            // Unkeying while not keyed is what the interface sends when it
            // loses the button, and it is not an error.
            Cmd::Key(None) => self.unkey(),
            // Already keyed. The interface repeats this while the key is
            // held, because it cannot know the over has started until the
            // status comes back, and keying twice would open a second
            // transmitter on a radio that has one.
            Cmd::Key(Some(_)) if self.rx.keyed() => {}
            Cmd::Key(Some(id)) => self.key(id),
            Cmd::Rate(r) => return self.set_rate(r),
            Cmd::NodeParam(id, name, value) => self.set_node_param(id, &name, value),
            Cmd::StageParam(stage, name, value) => match self.rx.node_of_stage(stage) {
                Some(id) => self.set_node_param(id.0, &name, value),
                None => {
                    *self.status.error.lock() = Some(format!("{name}: no such stage is running"))
                }
            },
            Cmd::Channels(specs) => self.set_channels(specs),
            Cmd::Fft(n) => {
                self.plan.fft = n;
                self.needs_rebuild = true;
            }
            Cmd::Refresh(hz) => {
                self.plan.refresh_hz = hz.clamp(1.0, 120.0);
                self.rx.set_refresh(self.plan.refresh_hz);
            }
            Cmd::Smoothing(v) => {
                self.plan.smoothing = v.clamp(0.01, 1.0);
                self.rx.set_smoothing(self.plan.smoothing);
            }
            Cmd::DcBlock(on) => {
                self.plan.dc_block = on;
                self.rx.set_dc_block(on);
            }
            Cmd::GainStage(stage, mode) => {
                if let Err(e) = self.dev.set_gain(&stage, mode) {
                    *self.status.error.lock() = Some(format!("{stage} gain: {e}"));
                }
                // The driver snaps to what the hardware supports, so the
                // control has to be told what it actually got rather than what
                // it asked for.
                self.remember_front();
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                self.rx.remeasure_dc();
            }
            Cmd::Toggle(name, on) => {
                if let Err(e) = self.dev.set_toggle(&name, on) {
                    *self.status.error.lock() = Some(format!("{name}: {e}"));
                }
                self.remember_front();
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                // Any of these changes the offset, and a stale estimate shows
                // up as a spur that was not there a moment ago.
                self.rx.remeasure_dc();
            }
            Cmd::Choice(name, value) => return self.set_choice(&name, &value),
            Cmd::Ppm(v) => {
                self.dev.correct(v);
                // Nothing moves until the tuner is asked for a frequency
                // again, so ask now: a correction that only took effect on the
                // next drag of the dial is a correction nobody can see
                // themselves setting.
                self.want_center = Some(self.plan.center);
                self.last_tune = std::time::Instant::now() - self.tune_gap;
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                self.needs_rebuild = true;
            }
            Cmd::Offset(hz) => {
                // The dial moves with the offset, so the receiver stays on
                // the signal it was on and the tuner is not asked for a
                // frequency it cannot reach. Setting 9750 on a dial at
                // 474 MHz otherwise asks for minus 9.2 GHz, and the dial is
                // then stranded below everything the radio can do.
                let moved = hz - self.dev.offset();
                self.dev.set_offset(hz);
                self.plan.center = Hz((self.plan.center.as_f64() + moved).max(0.0) as u64);
                // Same as a correction: nothing moves until the tuner is
                // asked again, and a setting that only took effect on the
                // next drag of the dial is one nobody can see themselves set.
                self.want_center = Some(self.plan.center);
                self.last_tune = std::time::Instant::now() - self.tune_gap;
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                self.needs_rebuild = true;
            }
            Cmd::Record(dir) => self.set_recording(dir),
            // No rebuild: the graph already holds the stage, switched off, so
            // a capture starts on the block after the button and keeps every
            // source the auto node has open.
            Cmd::CaptureIq(on) => {
                self.plan.capture = on;
                self.rx.set_capture(on);
            }
            Cmd::Location(lat, lon) => self.rx.set_location(lat, lon),
            Cmd::Survey(path) => {
                self.plan.settings.survey_path = path;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Wigle(account) => {
                self.plan.settings.wigle = account;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::BeaconDb(on) => {
                self.plan.settings.beacondb = on;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Heatmap(heat) => {
                if heat != self.plan.heat {
                    self.plan.heat = heat;
                    self.needs_rebuild = true;
                }
            }
            Cmd::ExportHeatmap { what, ramp, floor, ceil } => {
                let dir = crate::heatmap::heatmaps_dir();
                match self.rx.heatmap_mut() {
                    Some(n) => match n.export(&dir, what, ramp, floor, ceil) {
                        Ok(p) => tracing::info!("heatmap written: {}", p.display()),
                        Err(e) => {
                            *self.status.error.lock() = Some(format!("no heatmap written: {e}"))
                        }
                    },
                    None => {
                        *self.status.error.lock() =
                            Some("the heatmap recorder is not in the graph".into())
                    }
                }
            }
            Cmd::BandScan(scan) => {
                if scan != self.plan.scan {
                    self.plan.scan = scan;
                    self.needs_rebuild = true;
                }
            }
            Cmd::HomeAssistant(broker) => {
                self.plan.settings.homeassistant = broker;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Gps(transport) => crate::station::set_source(transport),
            Cmd::PacketLogCap(cap) => self.rx.set_log_cap(cap),
            Cmd::CaptureCap(bytes) => self.rx.set_capture_cap(bytes),
            Cmd::Feeds(feeds) => {
                if feeds != self.plan.feeds {
                    self.plan.feeds = feeds;
                    self.needs_rebuild = true;
                }
            }
            Cmd::Scanners(table) => {
                // A different table can mean a different front end on the
                // frequency the dial is already on, so this rebuilds rather
                // than waiting for the next retune.
                if table != self.scanners {
                    self.scanners = table;
                    self.needs_rebuild = true;
                }
            }
            // A lock on editing in the view, and nothing to the receiver: what
            // the operator changed applies either way, and what they did not
            // follows the dial either way.
            Cmd::Manual(on) => self.status.manual.store(on, Ordering::Relaxed),
            Cmd::Edits(e) => {
                if e != self.plan.edits {
                    self.plan.edits = e;
                    self.needs_rebuild = true;
                }
            }
            #[cfg(feature = "tea")]
            Cmd::TetraKey { colour, key } => self.rx.set_tetra_key(colour, key),
            #[cfg(feature = "tea")]
            Cmd::TetraIdSecret { colour, c } => self.rx.set_tetra_id_secret(colour, c),
            // The recorder and the transcriber are switched from the record
            // rather than by editing their stages, so they arrive here like
            // every other setting and are put into the plan the graph is
            // drawn from.
            Cmd::RecordCalls(dir) => {
                if dir != self.plan.calls {
                    self.plan.calls = dir;
                    self.needs_rebuild = true;
                }
            }
            Cmd::Transcribe { on, model, device } => {
                let want = (on, model, device);
                let held = (
                    self.plan.transcribe,
                    self.plan.transcribe_model.clone(),
                    self.plan.transcribe_device.clone(),
                );
                if want != held {
                    (
                        self.plan.transcribe,
                        self.plan.transcribe_model,
                        self.plan.transcribe_device,
                    ) = want;
                    self.needs_rebuild = true;
                }
            }
            Cmd::PacketLog(dir) => {
                self.plan.log = dir.is_some();
                self.rx.set_packet_log(dir);
                self.needs_rebuild = true;
            }
            Cmd::Zoom(n) => {
                let n = n.clamp(1, 64);
                if n != self.plan.zoom {
                    self.plan.zoom = n;
                    self.needs_rebuild = true;
                    self.status.zoom.store(n as u64, Ordering::Relaxed);
                }
            }
            Cmd::Decode(on) => {
                self.scan_on = on;
                self.needs_rebuild = true;
            }
            // The bus is a node, so it is rebuilt with the graph. What it was
            // told is in the plan, and the plan is what a rebuild hands the
            // bus that comes back: a retune must not silently unsubscribe.
            Cmd::CallSubs(subs) => {
                self.plan.settings.calls = subs;
                self.rx.apply_settings(&self.plan);
            }
            // In the plan for the same reason the call subscriptions are: a
            // rebuild must not silently change what is being watched.
            Cmd::WatchVideo(rules) => {
                self.plan.settings.watching = rules;
                self.rx.apply_settings(&self.plan);
            }
            Cmd::Play(speech) => {
                if let Some(r) = self.rx.replay_mut() {
                    r.play(&speech);
                }
            }
            Cmd::StopPlaying => {
                if let Some(r) = self.rx.replay_mut() {
                    r.stop();
                }
            }
        }
        Flow::Go
    }

    /// Open the microphone, once, for whatever wants speech.
    ///
    /// One capture for the receiver rather than one per consumer: the meter on
    /// the strip, the transmitter and anything else that grows a use for
    /// speech each take a tap, and every tap hears every sample. Opening it
    /// per keyed channel meant the meter only moved once it was too late to
    /// set a level against, and two consumers would have taken samples from
    /// each other.
    ///
    /// A device that will not open is logged and left alone: a receiver that
    /// works is more useful than one that refuses to start because there is no
    /// microphone in the machine. It is not the receiver's error either, since
    /// nothing has asked for speech yet; keying a channel whose source is the
    /// microphone is what says so, and that says it there.
    fn open_mic(&mut self) {
        if self.audio.mic.is_none() {
            let opened = match self.audio.input.is_empty() {
                true => audio::AudioCapture::open(48_000),
                false => audio::AudioCapture::open_named(&self.audio.input, 48_000),
            };
            match opened {
                Ok(c) => {
                    tracing::info!("microphone: {}", c.device_name());
                    self.audio.mic = Some(c);
                }
                Err(e) => {
                    tracing::warn!("no microphone: {e}");
                    self.status.mic_level.store(0f32.to_bits(), Ordering::Relaxed);
                }
            }
        }
        self.rx.set_microphone(self.audio.mic.as_ref().map(|m| m.tap()));
    }

    /// Move to the speaker and microphone the session asked for.
    fn set_audio_devices(&mut self, out: String, input: String) {
        if input != self.audio.input {
            self.audio.input = input;
            self.audio.mic = None;
            self.open_mic();
            self.needs_rebuild = true;
        }
        if out == self.audio.out {
            return;
        }
        self.audio.out = out;
        // Dropping the old player first: a host that only allows one stream
        // per device refuses the second one while the first is still open.
        self.audio._player = None;
        self.rx.set_speaker(None);
        let opened = match self.audio.out.is_empty() {
            true => AudioPlayer::open(48_000),
            false => AudioPlayer::open_named(&self.audio.out, 48_000),
        };
        match opened {
            Ok((p, s)) => {
                self.audio._player = Some(p);
                self.rx.set_speaker(Some(s));
            }
            Err(e) => *self.status.error.lock() = Some(format!("cannot open that speaker: {e}")),
        }
    }

    /// Take the radio back off the transmit stage.
    fn unkey(&mut self) {
        // A key let up before the rebuild it was waiting for: the rebuild
        // must not go on to announce it on air.
        self.tx.keying_for = None;
        if !self.rx.keyed() && self.status.keyed.load(Ordering::Relaxed) == 0 {
            return;
        }
        tracing::info!("unkeyed");
        // The stages stay; what goes is the radio. Dropping it drains the
        // queue before the carrier stops and, on a half duplex radio, hands
        // the receiver its radio back.
        self.status.tx_underruns.store(self.rx.unkey(), Ordering::Relaxed);
        self.status.keyed.store(0, Ordering::Relaxed);
        // Back where the receiver was. A half duplex radio has one
        // synthesiser, so keying moved it to the transmit frequency; leaving
        // it there means the waterfall comes back tuned to wherever the
        // channel transmits, which looks like reception never resumed at all.
        if self.dev.dial() != self.plan.center {
            if let Err(e) = self.dev.set_dial(self.plan.center) {
                *self.status.error.lock() =
                    Some(format!("could not retune after transmitting: {e}"));
            }
        }
        self.status.set_radio(RadioControls::read(self.dev.as_ref()));
    }

    /// Put a channel on air.
    ///
    /// The receive stream is left running. On a half duplex radio the driver
    /// feeds it a noise floor for the length of the over, so the spectrum, the
    /// channels and the decoders keep their state and the waterfall shows the
    /// gap rather than stopping; on a full duplex one it goes on hearing the
    /// band.
    fn key(&mut self, id: u64) {
        let Some(ch) = self.plan.channels.iter().find(|c| c.id == id).cloned() else {
            *self.status.error.lock() = Some("there is no such channel to key".into());
            return;
        };
        let tx = ch.spec_to_transmit();
        let up = key_up(
            self.dev.as_mut(),
            &ch,
            &tx,
            self.plan.center,
            self.tx.gain_db,
            &self.audio.mic,
            &self.voice,
            &self.tx.sub_file,
        );
        let (tx_plan, mut sinks) = match up {
            Ok(got) => got,
            Err(e) => {
                tracing::warn!("cannot transmit: {e}");
                *self.status.error.lock() = Some(format!("cannot transmit: {e}"));
                return;
            }
        };
        // The stages are already in the graph, so keying hands the transmit
        // stage a radio rather than building anything: a rebuild here would
        // restart the spectrum's averaging twice an over. Whether they are
        // the stages this channel wants is a question about the chain and
        // not about the channel, and asking it of the whole plan rebuilt for
        // a frequency that no stage reads.
        let same = self.plan.tx.is_some_and(|was| was.same_chain(&tx_plan))
            && self.rx.tx_ready(sinks.mic.is_some());
        self.plan.tx = Some(tx_plan);
        // What the monitor draws the over on. A setting, because it is the
        // one thing two channels of a mode differ by.
        self.rx.set_tx_shift(tx_plan.on_air.as_f64() - self.plan.center.as_f64());
        let stream = sinks.stream.take();
        if same {
            if let Some(s) = stream {
                self.rx.key(s);
            }
            tracing::info!("keyed channel {}", ch.id);
            self.status.keyed.store(ch.id, Ordering::Relaxed);
            self.tx.last_keyed = Some(ch.id);
            return;
        }
        // The chain in the graph is not the one this channel wants, or there
        // is none yet. Build it, with the radio going to the transmitter to
        // be put on it as it is built.
        sinks.stream = stream;
        self.rx.set_transmitter(Some(sinks));
        self.needs_rebuild = true;
        // Said only once the radio is actually transmitting, so ON AIR
        // means on air.
        self.tx.keying_for = Some(ch.id);
    }

    /// Stop the stream and let go of it.
    ///
    /// Dropped rather than only stopped: the driver counts a stopped stream as
    /// still holding the radio until its handle is gone.
    fn release_stream(&mut self) {
        if let Some(mut s) = self.stream.take() {
            s.stop();
        }
    }

    /// Change the span the radio delivers.
    fn set_rate(&mut self, r: Sps) -> Flow {
        // A HackRF's streaming reader owns the device and its control channel
        // does not carry the sample rate, so the radio has to be stopped,
        // reopened and started again. Asking anyway used to fail, and the
        // failure propagated out of this loop and killed the thread: changing
        // bandwidth stopped the receiver dead.
        if self.dev.rate_needs_restart() {
            let Some(entry) = self.entry.clone() else {
                *self.status.error.lock() =
                    Some("this radio cannot change span without being reopened".into());
                return Flow::Go;
            };
            self.release_stream();
            match restart(
                &entry,
                r,
                self.plan.center,
                &self.front,
                self.dev.asked_ppm(),
                self.dev.offset(),
            ) {
                Ok((d, s)) => {
                    self.dev = d;
                    self.stream = Some(s);
                }
                Err(e) => {
                    *self.status.error.lock() = Some(format!("cannot change span: {e}"));
                    return Flow::Stop;
                }
            }
        } else if let Err(e) = self.dev.set_rate(r) {
            *self.status.error.lock() = Some(format!("cannot change span: {e}"));
            return Flow::Go;
        }
        self.plan.rate = self.dev.rate().as_f64();
        self.remember_front();
        self.status.set_radio(RadioControls::read(self.dev.as_ref()));
        self.needs_rebuild = true;
        Flow::Go
    }

    /// Set one of the device's own choices, restarting the stream where the
    /// choice describes the stream rather than a setting on it: a LimeSDR's
    /// receive channel is a different stream entirely.
    fn set_choice(&mut self, name: &str, value: &str) -> Flow {
        if self.dev.choice_needs_restart(name) {
            self.release_stream();
            if let Err(e) = self.dev.set_choice(name, value) {
                *self.status.error.lock() = Some(format!("{name}: {e}"));
            }
            match self.dev.start_rx() {
                Ok(s) => self.stream = Some(s),
                Err(e) => {
                    *self.status.error.lock() = Some(format!("cannot restart after {name}: {e}"));
                    return Flow::Stop;
                }
            }
        } else if let Err(e) = self.dev.set_choice(name, value) {
            *self.status.error.lock() = Some(format!("{name}: {e}"));
        }
        self.status.set_radio(RadioControls::read(self.dev.as_ref()));
        self.rx.remeasure_dc();
        Flow::Go
    }

    /// Set a parameter on one node of the running graph.
    fn set_node_param(&mut self, id: usize, name: &str, value: pipeline::param::ParamValue) {
        match self.rx.set_node_param(id, name, value) {
            // A parameter that changes the stream's shape needs the graph
            // negotiated again around it; the rest take effect on the next
            // block.
            Ok(true) => self.needs_rebuild = true,
            Ok(false) => publish_chain(self.status, &self.rx),
            Err(e) => *self.status.error.lock() = Some(format!("{name}: {e}")),
        }
        // The receiver wrote it into its description. What that changed is
        // either the operator's edit, which the next rebuild has to start
        // from, or a level the strip owns, which the strip has to be told of.
        self.plan.edits = self.rx.edits();
        pull_levels(&self.rx, &mut self.plan, self.status);
        self.status.set_patch(&self.rx);
    }

    /// The complete set of channels the strip is asking for.
    fn set_channels(&mut self, specs: Vec<ChannelSpec>) {
        self.plan.channels = specs;
        // The transmit chain follows the strip like every other derived stage:
        // change a channel's mode or its shift and the chain view shows what
        // would go out, keyed or not.
        let want = derive_tx(
            &self.plan,
            self.status.can_transmit.load(Ordering::Relaxed),
            self.tx.keying_for.or(self.tx.last_keyed),
        );
        if want != self.plan.tx && !self.rx.keyed() {
            self.plan.tx = want;
            self.needs_rebuild = true;
        }
        // A squelch or gain change is a number on a node that is already
        // there. Rebuilding for it threw away the spectrum's averaging and
        // every channel's state, once per frame for as long as the slider was
        // held.
        if self.rx.params_only(&self.plan) {
            self.rx.apply_params(&self.plan);
            publish_chain(self.status, &self.rx);
        } else {
            self.needs_rebuild = true;
        }
    }

    /// Start or stop recording the span.
    fn set_recording(&mut self, dir: Option<(std::path::PathBuf, Option<u64>)>) {
        let rec = match dir {
            Some((d, mb)) => {
                match crate::record::Recorder::new(&d, self.plan.eff_rate(), self.plan.center) {
                    Ok(r) => Some(match mb {
                        Some(mb) => r.with_budget(mb << 20),
                        None => r,
                    }),
                    Err(e) => {
                        *self.status.error.lock() =
                            Some(format!("cannot record to {}: {e}", d.display()));
                        None
                    }
                }
            }
            None => None,
        };
        self.plan.record = rec.is_some();
        self.rx.set_recorder(rec);
        self.needs_rebuild = true;
    }

    /// Move the dial, no more often than a retune can be afforded.
    ///
    /// Retuning costs about 25 ms on the RTL-SDR, more than a frame at 60 Hz,
    /// and it blocks the thread that reads samples. Spacing them out keeps the
    /// spectrum live while a drag is in progress; the last requested frequency
    /// is always reached because the pending one is held until it can be
    /// applied.
    fn retune(&mut self) -> anyhow::Result<()> {
        let Some(f) = self.want_center else { return Ok(()) };
        if self.last_tune.elapsed() < self.tune_gap {
            return Ok(());
        }
        let _t = tracing::info_span!("set_center").entered();
        self.dev.set_dial(f)?;
        // The plan is labelled with where the receiver is, not with what the
        // tuner was asked for: the dial, the spectrum and every channel offset
        // are read against it.
        self.plan.center = self.dev.dial();
        self.needs_rebuild = true;
        self.want_center = None;
        self.last_tune = std::time::Instant::now();
        Ok(())
    }

    /// Draw the graph again from the plan, if anything asked for it.
    fn rebuild(&mut self) -> Flow {
        if !self.needs_rebuild {
            return Flow::Go;
        }
        let _t = tracing::info_span!("rebuild").entered();
        // The banks understand nothing on either wideband band, so running
        // them there only spends CPU inventing unknown bursts.
        self.plan.fronts = fronts_here(&self.scanners, &self.plan, self.scan_on);
        // The transmit chain follows the dial too: a channel's transmit
        // frequency is its offset from wherever the receiver is now.
        let before: Vec<u64> = self.rx.channels().iter().map(|c| c.spec.id).collect();
        let keying_now = self.tx.keying_for.take();
        // A key waiting on this rebuild is not keyed yet, so the chain has to
        // be drawn for the channel about to go on air rather than for
        // whichever one comes first on the strip.
        if !self.rx.keyed() {
            self.plan.tx = derive_tx(
                &self.plan,
                self.status.can_transmit.load(Ordering::Relaxed),
                keying_now.or(self.tx.last_keyed),
            );
        }
        if let Err(e) = self.rx.rebuild(&self.plan) {
            // A patch is drawn wire by wire, so most of the time it is half a
            // graph, and a type mismatch between two stages is an ordinary
            // step rather than a fault. Going back to the last edits that
            // built keeps the receiver running while it is said; without this
            // an edit could stop the radio dead.
            let Some(good) = self.last_edits.clone() else {
                *self.status.error.lock() = Some(format!("cannot build the chain: {e}"));
                return Flow::Stop;
            };
            *self.status.error.lock() = Some(format!("the patch was refused: {e}"));
            self.plan.edits = good;
            if let Err(e) = self.rx.rebuild(&self.plan) {
                *self.status.error.lock() = Some(format!("cannot build the chain: {e}"));
                return Flow::Stop;
            }
        } else {
            // Only a shape that built is worth going back to.
            self.last_edits = Some(self.plan.edits.clone());
        }
        // A key that was waiting on this rebuild: the radio went in with the
        // graph, so this is the moment it is actually on air, or the moment to
        // say it is not.
        if let Some(id) = keying_now {
            if self.rx.tx_on_air() {
                tracing::info!("keyed channel {id}");
                self.status.keyed.store(id, Ordering::Relaxed);
                self.tx.last_keyed = Some(id);
            } else {
                *self.status.error.lock() =
                    Some("the transmit chain did not build; nothing is on air".into());
                // Let the key back up with it. A key held down over a
                // transmitter that never got a chain is a state nothing can
                // leave: every further key is ignored as already keyed.
                self.unkey();
            }
        }
        // Its own slot, not the fault line: a front end the span cannot hold
        // is a standing verdict on the graph that was just built, and writing
        // it over `error` threw away whatever went wrong a few lines above,
        // every rebuild.
        *self.status.refused.lock() = self.rx.refused.clone();
        // A channel that was rebuilt has lost its RDS state, and its old
        // station name must not sit over whatever it is tuned to now.
        let kept: Vec<u64> = self
            .rx
            .channels()
            .iter()
            .filter(|c| before.contains(&c.spec.id) && c.kept)
            .map(|c| c.spec.id)
            .collect();
        self.status.keep_stations(&kept);
        // Every channel covers a different frequency now, so nothing already
        // reported can be the same burst as anything arriving.
        self.rx.reset_dedupe();
        let (rate, center) = (self.plan.eff_rate(), self.plan.center);
        if let Some(r) = self.rx.recorder_mut() {
            r.retune(rate, center);
        }
        self.status.logged.store(self.rx.logged(), Ordering::Relaxed);
        // What is running, described the way the view draws it, and what the
        // receiver drew underneath the edits, which is what an edited copy is
        // read against.
        self.status.set_patch(&self.rx);
        publish_chain(self.status, &self.rx);
        // The edits brought the levels with them, and the strip has to be
        // shown what the nodes now hold.
        pull_levels(&self.rx, &mut self.plan, self.status);
        self.needs_rebuild = false;
        Flow::Go
    }

    /// One block of samples from the radio, reopening it if it has stopped.
    ///
    /// Reopening is worth trying, because the usual cause is the board
    /// resetting itself and coming back a second later, and the alternative is
    /// a window that has to be restarted to speak to a radio that is present.
    fn read_block(&mut self) -> Block {
        let _read = tracing::info_span!("rf_read").entered();
        let Some(stream) = self.stream.as_mut() else { return Block::Lost };
        let e = match stream.read() {
            Ok(b) => {
                let dropped = stream.dropped();
                self.status.dropped.store(dropped, Ordering::Relaxed);
                return Block::Samples(b);
            }
            // The radio has gone: unplugged, reset by hand, or wedged past
            // what its driver could recover.
            Err(e) => e,
        };
        tracing::warn!("receive stopped: {e}");
        *self.status.error.lock() = Some(format!("radio stopped: {e}; reopening"));
        let mut back = None;
        // A radio handed in already open has nowhere to be opened from, so
        // one that stops has stopped.
        let entry = self.entry.clone();
        for attempt in entry.iter().flat_map(|_| 1..=3) {
            std::thread::sleep(std::time::Duration::from_millis(400 * attempt));
            match restart(
                entry.as_ref().expect("the loop runs only where there is one"),
                Sps(self.plan.rate as u64),
                self.plan.center,
                &self.front,
                self.dev.asked_ppm(),
                self.dev.offset(),
            ) {
                Ok(got) => {
                    back = Some(got);
                    break;
                }
                Err(e) => tracing::warn!("reopen {attempt} failed: {e}"),
            }
        }
        match back {
            Some((d, s)) => {
                self.dev = d;
                self.stream = Some(s);
                self.status.set_radio(RadioControls::read(self.dev.as_ref()));
                *self.status.error.lock() = Some("radio came back".into());
                self.needs_rebuild = true;
                Block::Restarted
            }
            None => {
                *self.status.error.lock() =
                    Some("the radio is gone; pick it again once it is back".into());
                Block::Lost
            }
        }
    }

    /// What the microphone is hearing, whether or not anything is keyed.
    ///
    /// While an over is running the microphone stage has already taken those
    /// samples out of the ring, so the reading comes from the graph instead of
    /// from the capture.
    fn meter_mic(&mut self) {
        let Some(c) = self.audio.mic.as_ref() else { return };
        let keyed_now = self.rx.tx_state();
        let peak = match keyed_now {
            Some(tx) if self.rx.keyed() => tx.mic_peak,
            _ => c.peak(),
        };
        // Said once a second while keyed, because a transmission that stops is
        // the hardest thing here to see after the fact: the carrier is gone
        // and nothing on screen says why.
        if let (Some(tx), 0) = (keyed_now, self.tx.blocks_since_key % 50) {
            if self.rx.keyed() {
                tracing::info!(
                    "on air: {} samples, {} unfilled, mic {peak:.2}",
                    tx.written,
                    tx.underruns
                );
                self.status.tx_underruns.store(tx.underruns, Ordering::Relaxed);
            }
        }
        self.tx.blocks_since_key = self.tx.blocks_since_key.wrapping_add(1);
        self.status.mic_level.store(peak.to_bits(), Ordering::Relaxed);
        self.status.mic_clipped.store(self.rx.keyed() && self.rx.mic_clipped(), Ordering::Relaxed);
    }

    /// Put the block through the graph, which is everything the receiver does
    /// with it.
    fn process(&mut self, samples: &[C32]) -> Flow {
        let _g = tracing::info_span!("graph").entered();
        if let Err(e) = self.rx.process(samples) {
            *self.status.error.lock() = Some(format!("chain: {e}"));
            return Flow::Stop;
        }
        if let Some(w) = self.rx.take_warnings().pop() {
            *self.status.error.lock() = Some(w);
        }
        self.answer_requests();
        Flow::Go
    }

    /// Move the dial where a stage asked it to be.
    ///
    /// Only the walk over a band asks, and only while it is walking: a
    /// decoder that wants a frequency in the span asks the same way, and
    /// answering that would move the dial out from under whatever the
    /// operator was listening to. The ask is clamped to what the tuner
    /// reaches, so a band edge typed past it walks the part that exists.
    fn answer_requests(&mut self) {
        let reach = self.status.radio.lock().reach;
        for (_stage, request) in self.rx.take_requests() {
            let pipeline::Request::Retune { center_hz } = request else { continue };
            if !self.plan.scan.running {
                continue;
            }
            let hz = center_hz.clamp(reach.0, reach.1);
            if hz > 0.0 {
                self.want_center = Some(Hz(hz as u64));
            }
        }
    }

    /// The spectrum frame, and everything else read at the display's rate.
    fn publish_spectrum(&mut self) -> Flow {
        if !self.rx.spectrum_ready() {
            return Flow::Go;
        }
        // The fix is read at the display's rate rather than per block: a GPS
        // reports once a second and a block is seven milliseconds, so asking
        // per block is two hundred locks for one new number.
        // A fix moves the station; losing the sky leaves it where it was.
        self.rx.set_fix(crate::station::fix());
        {
            // Read at the display's rate: the status counts spool files on
            // disc, which is a directory listing and not a number worth taking
            // per block.
            let now = self.rx.wigle_status();
            let mut held = self.status.wigle.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.beacondb_status();
            let mut held = self.status.beacondb.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.scan_status();
            let mut held = self.status.band_scan.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.heatmap_status();
            let mut held = self.status.heatmap.lock();
            if *held != now {
                *held = now;
            }
        }
        {
            let now = self.rx.homeassistant_status();
            let mut held = self.status.homeassistant.lock();
            if *held != now {
                *held = now;
            }
        }
        if let Some((devices, sightings, heard)) = self.rx.survey_counts() {
            self.status.survey_devices.store(devices, Ordering::Relaxed);
            self.status.survey_sightings.store(sightings, Ordering::Relaxed);
            self.status.survey_heard.store(heard, Ordering::Relaxed);
        }
        // Published with the spectrum rather than every block: the table is
        // redrawn at the display's rate, and cloning it 140 times a second for
        // a pane nobody may be looking at is wasted work.
        if self.rx.tracking() {
            let rows = self.rx.tracks(std::time::Instant::now());
            self.status.aircraft.store(rows.len() as u64, Ordering::Relaxed);
            *self.status.track_list.lock() = rows;
        }
        #[cfg(feature = "stt")]
        {
            *self.status.transcriber.lock() = self.rx.transcriber();
        }
        *self.status.recorder.lock() = self.rx.recorder();
        if !self.plan.feeds.is_empty() {
            *self.status.feeds.lock() = self.rx.feed_status();
        }
        // The chain carries what each wire is measured to be passing, so it is
        // republished while it runs rather than only when its shape changes: a
        // graph drawn once at build time reports the throughput it had before
        // any samples went through it, which is none.
        if self.last_chain.elapsed() >= CHAIN_PUBLISH {
            publish_chain(self.status, &self.rx);
            self.last_chain = std::time::Instant::now();
        }
        // Scopes are a display and refresh with the spectrum, not with the
        // chain: a scope republished once a second is a scope showing a
        // second-old picture.
        let scopes = self.rx.scopes();
        if !scopes.is_empty() || !self.status.scopes.lock().is_empty() {
            *self.status.scopes.lock() = scopes;
        }
        // The rate the spectrum sees rather than the one the radio delivers:
        // in manual mode a stage can sit between the two, and an axis drawn
        // from the wrong one puts every signal in the wrong place.
        let seen = self.rx.spectrum_rate();
        let extra = self.rx.patch_spectra();
        let f = Frame {
            db: self.rx.power_db().to_vec(),
            adc: self.rx.adc(),
            center: self.plan.center.as_f64(),
            rate: if seen > 0.0 { seen } else { self.plan.eff_rate() },
            extra,
        };
        // Drop rather than block: the radio must never stall waiting for the
        // UI, and a stale spectrum is worthless anyway.
        match self.frames.try_send(f) {
            Ok(()) | Err(TrySendError::Full(_)) => {}
            Err(TrySendError::Disconnected(_)) => return Flow::Stop,
        }
        (self.repaint)();
        Flow::Go
    }

    /// What the receiver is holding, published every block.
    fn publish_status(&mut self) {
        self.status.tracking.store(self.rx.tracking(), Ordering::Relaxed);
        {
            self.rx.refresh_capture_folder();
            let cap = self.rx.capture();
            self.status.capture_on.store(self.rx.capturing(), Ordering::Relaxed);
            self.status.capture_bytes.store(cap.map(|c| c.bytes()).unwrap_or(0), Ordering::Relaxed);
            self.status
                .capture_folder
                .store(cap.map(|c| c.folder_bytes()).unwrap_or(0), Ordering::Relaxed);
            self.status.capture_full.store(cap.is_some_and(|c| c.is_full()), Ordering::Relaxed);
            *self.status.capture_file.lock() =
                cap.and_then(|c| c.path()).map(|p| p.display().to_string());
        }
        self.rx.refresh_log_folder();
        self.status.logged.store(self.rx.logged(), Ordering::Relaxed);
        self.status.set_video(self.rx.watched_video());
        self.status.set_video_inputs(self.rx.video_inputs());
        *self.status.multiplexes.lock() = self.rx.multiplexes();
        if let Some(saved) = self.rx.pictures_saved() {
            let mut cur = self.status.pictures.lock();
            if cur.len() != saved.len() {
                *cur = saved;
            }
        }
        self.status.log_bytes.store(self.rx.log_bytes(), Ordering::Relaxed);
        self.status.log_full.store(self.rx.log_full(), Ordering::Relaxed);
        let chans = self.rx.bank_channels();
        self.status
            .scan_channels
            .store(chans.first().copied().unwrap_or(0) as u64, Ordering::Relaxed);
        self.status
            .scan_channels_wide
            .store(chans.get(1).copied().unwrap_or(0) as u64, Ordering::Relaxed);
        self.status.sources_on.store(self.rx.has_sources(), Ordering::Relaxed);
        let now = std::time::Instant::now();
        let mut seen = self.status.sources.lock();
        for e in seen.iter_mut() {
            e.live = false;
        }
        for s in self.rx.live_sources() {
            // Matched within kind: a locked channel and a detection can sit on
            // the same frequency, and they are two different statements about
            // it.
            let same = seen.iter_mut().find(|e| {
                e.source.locked_to == s.locked_to
                    && (e.source.center_hz - s.center_hz).abs()
                        < e.source.bandwidth_hz.max(s.bandwidth_hz) / 2.0
            });
            match same {
                Some(e) => {
                    e.source = s;
                    e.last_seen = now;
                    e.live = true;
                }
                None => seen.push(SeenSource { source: s, last_seen: now, live: true }),
            }
        }
        seen.retain(|e| e.live || now.duration_since(e.last_seen) < SOURCE_LINGER);
    }

    /// What the block decoded to, on its way to the packet list.
    fn harvest_decodes(&mut self, at: std::time::Instant) -> Flow {
        self.records.clear();
        self.records.extend(harvest(&mut self.rx, at));
        if self.rx.recorder_mut().is_some_and(|r| r.is_full()) {
            let mb = self.rx.recorder_mut().map(|r| r.written() >> 20).unwrap_or(0);
            *self.status.error.lock() = Some(format!("recording stopped: wrote {mb} MB"));
            self.plan.record = false;
            self.rx.set_recorder(None);
            self.needs_rebuild = true;
        }
        if self.records.is_empty() {
            return Flow::Go;
        }
        self.hits += self.records.len() as u64;
        self.status.decoded.store(self.hits, Ordering::Relaxed);
        // Never block the radio thread on a UI that is behind; a dropped batch
        // is reported by the counter going up without the log growing to
        // match.
        match self.decodes.try_send(std::mem::take(&mut self.records)) {
            Ok(()) | Err(TrySendError::Full(_)) => {}
            Err(TrySendError::Disconnected(_)) => return Flow::Stop,
        }
        (self.repaint)();
        Flow::Go
    }

    /// Read the meters back off the audio path.
    ///
    /// Everything that is heard was mixed on the bus and played by the
    /// speaker, in the graph: every channel at its fader, every subscribed
    /// call, a replay. Nothing here touches the sound card.
    fn meter_audio(&mut self) {
        self.status.set_channel_states(self.rx.channel_states());
        self.status.set_strips(self.rx.strips());
        *self.status.tetra_keys.lock() = self.rx.tetra_key_status();
        if let Some(h) = self.rx.heard_mut() {
            let calls = h.take_calls();
            if !calls.is_empty() {
                let mut heard = self.status.heard.lock();
                // A running call replaces its last report; an ended one is
                // kept, since it is the only report that says so.
                for c in calls {
                    // A call whose labels filled in part way through the
                    // over updates the row it was reported under, rather
                    // than appearing beside it as a second transmission.
                    let under = c.was.clone().unwrap_or_else(|| c.key());
                    match heard.iter_mut().find(|h| !h.over && h.key() == under) {
                        Some(h) => *h = c,
                        None => heard.push(c),
                    }
                }
            }
        }
        if let Some(h) = self.rx.heard() {
            Status::set_level(&self.status.call_level, h.peak());
            *self.status.call_levels.lock() = h.levels();
        }
        if let Some(c) = self.rx.calls() {
            self.status.call_gain_db.store(c.agc_gain_db().to_bits(), Ordering::Relaxed);
        }
        if let Some(b) = self.rx.audio() {
            *self.status.playing.lock() = b.playing().to_vec();
        }
        let left = self.rx.replay().map(|r| r.left()).unwrap_or(0.0);
        self.status.replay_left_s.store((left as f32).to_bits(), Ordering::Relaxed);
        if let Some(s) = self.rx.speaker() {
            Status::set_level(&self.status.out_level, s.peak());
            self.status.audio_backlog.store(s.backlog().max(0) as u64, Ordering::Relaxed);
        }
    }

    /// What the RDS decoder has read, whatever there is to play it on.
    ///
    /// It is a demodulator in the graph and not something the speaker does, so
    /// a receiver with no audio device still names the station: the headless
    /// probe reads exactly this.
    fn publish_stations(&self) {
        let wfm = |c: &&crate::chain::Chan| c.spec.mode == ChanMode::Audio(Demod::Wfm);
        for w in self.rx.channels().iter().filter(wfm) {
            let (g, e, sy) = w.rds_stats;
            self.status.set_station(w.spec.id, &w.station, g, e, sy);
        }
        if let Some(w) = self.rx.channels().iter().find(wfm) {
            self.status.set_blend(w.blend);
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn run(
    entry: crate::devices::Entry,
    center: Hz,
    rate: Sps,
    offset: f64,
    fft: usize,
    cmd: Receiver<Cmd>,
    frames: Sender<Frame>,
    decodes: Sender<Vec<DecodeRecord>>,
    status: &Status,
    repaint: impl Fn(),
) -> anyhow::Result<()> {
    RadioThread::open(entry, center, rate, offset, fft, cmd, frames, decodes, status, repaint)?
        .run()
}

/// Tell the strip what the nodes hold, when it differs from what it was
/// last told.
///
/// A fader or a squelch set through the chain view lands on the node, and
/// the strip is what the operator reads, so it has to follow. A squelch or
/// a gain control is also a plan value, which the next rebuild draws from,
/// so the plan follows too; a level is not, since the fader keeps it. A
/// revision moves only when something changed, so what the strip sends
/// itself does not come back to it.
fn pull_levels(rx: &crate::chain::Receiver, plan: &mut Plan, status: &Status) {
    let (audio, chans) = rx.levels();
    let was = status.levels();
    for c in &chans {
        if let Some(have) = plan.channels.iter_mut().find(|h| h.id == c.id) {
            have.label = c.label.clone();
            have.squelch_db = c.squelch_db;
            have.agc = c.agc;
        }
    }
    if audio != was.audio || chans != was.channels {
        status.set_levels(audio, chans);
    }
}

/// Every front end the span covers, or none of them.
///
/// `decode_on` is the operator's own switch: turning decoding off stops the
/// front ends being built at all, which is the expensive thing the receiver
/// does. It does not change which of them belong here.
fn fronts_here(
    scanners: &crate::scanners::Scanners,
    plan: &Plan,
    decode_on: bool,
) -> Vec<crate::scanners::FrontAt> {
    if !decode_on {
        return Vec::new();
    }
    scanners.fronts(plan.center.as_f64(), plan.eff_rate())
}

/// Publish the chain the receiver is running, for the chain view.
///
/// There is one graph and it holds everything, so this is no longer a choice
/// between chains: what is drawn is what runs.
fn publish_chain(status: &Status, rx: &crate::chain::Receiver) {
    // Both chains as one: the receiver's, and the transmitter's from the
    // thread that runs it, with its ids moved out of the way.
    status.set_chain(
        Some(crate::transmit::merged(&rx.topology(), rx.tx_topology().as_ref())),
        rx.latency_ms(0),
    );
}

/// A plan that only scans, for tests about the shape of the receiver.
#[cfg(test)]
fn plan_at(rate: f64, center: Hz) -> Plan {
    Plan {
        center,
        rate,
        zoom: 1,
        dc_block: false,
        refresh_hz: 30.0,
        smoothing: crate::chain::DEFAULT_SMOOTHING,
        fft: 1024,
        channels: Vec::new(),
        fronts: vec![crate::scanners::FrontAt {
            front: crate::scanners::Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
            // The whole span: these tests are about the shape of the
            // receiver, not about which band a block covers.
            band: (0.0, f64::INFINITY),
        }],
        scan: Default::default(),
        heat: Default::default(),
        edits: Default::default(),
        record: false,
        capture: false,
        capture_dir: crate::chain::default_capture_dir(),
        capture_format: common::SampleFormat::Cu8,
        log: false,
        calls: None,
        transcribe: false,
        transcribe_model: String::new(),
        transcribe_device: String::new(),
        feeds: Vec::new(),
        tx: None,
        settings: Default::default(),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::chain::OOK_CHANNEL_HZ;

    /// The transmit chain the receiver would draw for a plan, on a radio that
    /// can transmit with nothing keyed. Named for what it is outside this
    /// file: the strip asks what would go out, and this answers.
    pub(crate) fn transmit_plan(plan: &Plan) -> Option<crate::chain::TxPlan> {
        derive_tx(plan, true, None)
    }

    /// Two transmit channels, which is what recalling one from the bank
    /// makes: the chain is drawn for the one going on air, not for whichever
    /// is first on the strip. Without this the rebuild that puts a key on air
    /// replaced the keyed channel's plan with the first channel's, so a
    /// channel set to MIC transmitted the other one's test tone.
    #[test]
    fn the_transmit_chain_follows_the_channel_going_on_air() {
        let mut plan = crate::chain::tests::plan(2_400_000.0, Hz(145_000_000));
        let ch = |id: u64, offset: f64, source: TxSource| ChannelSpec {
            id,
            label: format!("CH{id}"),
            offset_hz: offset,
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: None,
            squelch_db: None,
            voice: false,
            agc: true,
            tx: Some(TxSpec { source, ..Default::default() }),
        };
        plan.channels = vec![ch(1, 25_000.0, TxSource::Tone), ch(2, -50_000.0, TxSource::Mic)];
        let first = derive_tx(&plan, true, None).expect("a chain to hold ready");
        assert_eq!(first.spec.source, TxSource::Tone);
        assert_eq!(first.on_air, Hz(145_025_000));
        let keyed = derive_tx(&plan, true, Some(2)).expect("the keyed channel's chain");
        assert_eq!(keyed.spec.source, TxSource::Mic);
        assert_eq!(keyed.on_air, Hz(144_950_000));
        // A channel that cannot transmit does not take the chain away from
        // one that can.
        plan.channels.push(ChannelSpec { id: 3, tx: None, ..plan.channels[0].clone() });
        assert_eq!(derive_tx(&plan, true, Some(3)), Some(first));
        assert_eq!(derive_tx(&plan, false, Some(2)), None);
    }

    /// Wait for the radio thread, which runs on its own clock. Fails the
    /// test rather than hanging.
    fn until(what: &str, mut ready: impl FnMut() -> bool) {
        let until = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while !ready() {
            assert!(std::time::Instant::now() < until, "waited ten seconds for {what}");
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    fn strip_channel(id: u64, offset: f64) -> ChannelSpec {
        ChannelSpec {
            id,
            label: format!("CH{id}"),
            offset_hz: offset,
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: None,
            squelch_db: None,
            agc: true,
            voice: false,
            // As the strip sends it: nobody has said anything about
            // transmitting, and the mode decides.
            tx: None,
        }
    }

    /// A channel added and keyed, on a radio, end to end.
    ///
    /// Everything between a command arriving and a sample reaching the
    /// antenna: the plan, the derived transmit chain, the graph, the
    /// transmitter's thread and the device. This is the join that kept
    /// breaking, and it could not be tested until there was a radio to test
    /// it on.
    #[test]
    fn a_channel_added_and_keyed_reaches_the_antenna() {
        let center = Hz(446_000_000);
        let rate = Sps(2_400_000);
        let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
        let watch = dev.watcher();
        let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        until("the radio to say it transmits", || {
            radio.status.can_transmit.load(Ordering::Relaxed)
        });

        radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));
        radio.send(Cmd::Key(Some(1)));
        until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
        // Enough of an over to read: a tenth of a second at the span's rate.
        until("a tenth of a second on the antenna", || watch.transmitted_len() > 240_000);
        assert!(watch.keyed(), "the device was never asked to transmit");
        assert_eq!(radio.status.error.lock().clone(), None, "it went on air and still complained");

        // What actually reached the antenna, rather than how much of it.
        // The default source is a 1 kHz tone, so the whole pipeline is
        // judged by whether a discriminator reads 1 kHz back off it at the
        // narrow band deviation: the clock, the tone, the modulator, the
        // sink and the executor that ran them.
        let air = watch.transmitted();
        let mut demod = dsp::FmDemod::new(rate.as_f64(), nodes::NBFM_DEVIATION_HZ);
        let mut audio = Vec::new();
        demod.process(&air[4_800..], &mut audio);
        let seg = &audio[1_000..];
        let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
        let hz = crossings as f64 * rate.as_f64() / seg.len() as f64;
        assert!((hz - 1_000.0).abs() < 20.0, "the air carried {hz:.0} Hz, not a 1 kHz tone");
        let peak = seg.iter().fold(0.0f32, |m, v| m.max(v.abs()));
        assert!(peak > 0.5, "the tone is there at {peak}, too quiet to be full deviation");
        let level = air[4_800].norm();
        assert!(
            air[4_800..].iter().all(|s| (s.norm() - level).abs() < 0.05),
            "an FM carrier holds its envelope"
        );

        radio.send(Cmd::Key(None));
        until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
        until("the device to be given back", || !watch.keyed());
        let sent = watch.transmitted_len();
        std::thread::sleep(std::time::Duration::from_millis(50));
        assert_eq!(watch.transmitted_len(), sent, "it went on transmitting after the key came up");
    }

    /// Two channels, worked one after the other, on a radio.
    ///
    /// Recalling a second channel beside the first is what a bank is for, and
    /// the second one has to key up as readily as the first. The two are one
    /// transmit chain, so nothing between them is rebuilt and the failure is
    /// silent: the key lights and nothing goes out.
    #[test]
    fn a_second_channel_keys_up_as_readily_as_the_first() {
        let center = Hz(145_000_000);
        let rate = Sps(2_400_000);
        let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
        let watch = dev.watcher();
        let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        until("the radio to say it transmits", || {
            radio.status.can_transmit.load(Ordering::Relaxed)
        });
        radio.send(Cmd::Channels(vec![strip_channel(1, 25_000.0), strip_channel(2, -50_000.0)]));

        let mut was = 0;
        for id in [1, 2, 1, 2] {
            radio.send(Cmd::Key(Some(id)));
            until(&format!("channel {id} to go on air"), || {
                radio.status.keyed.load(Ordering::Relaxed) == id
            });
            until(&format!("channel {id} on the antenna"), || watch.transmitted_len() > was);
            radio.send(Cmd::Key(None));
            until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
            was = watch.transmitted_len();
        }
        assert_eq!(
            radio.status.error.lock().clone(),
            None,
            "every over went out and it complained"
        );
    }

    /// The dial and the span move under a running receiver.
    ///
    /// Both go through the radio thread and both redraw the graph: a retune
    /// is a device call and a rebuild with every channel at a new offset, and
    /// a span change may take the device down and open it again. Neither was
    /// under test, and a receiver that stops when somebody turns the dial is
    /// the one fault nobody would report as a bug in a decoder.
    #[test]
    fn the_dial_and_the_span_move_without_stopping_the_receiver() {
        let dev = sources::FileRadio::silent(Hz(145_000_000), Sps(2_400_000)).as_fast_as_it_can();
        let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        radio.send(Cmd::Channels(vec![strip_channel(1, 25_000.0), strip_channel(2, -50_000.0)]));

        // Read off the spectrum frames, because that is what the window
        // draws: a dial that moved and a waterfall that did not is the fault
        // this would be reported as.
        let seen = |what: f64, pick: fn(&Frame) -> f64| -> bool {
            radio.frames.try_iter().any(|f| (pick(&f) - what).abs() < 1.0)
        };
        for hz in [433_920_000.0, 136_825_000.0, 95_800_000.0, 145_000_000.0] {
            radio.send(Cmd::Center(Hz(hz as u64)));
            until(&format!("the spectrum to arrive at {hz}"), || seen(hz, |f| f.center));
            assert!(radio.status.running.load(Ordering::Relaxed), "the radio stopped at {hz}");
        }
        for rate in [2_048_000.0, 9_142_857.0, 250_000.0, 2_400_000.0] {
            radio.send(Cmd::Rate(Sps(rate as u64)));
            until(&format!("the spectrum to arrive at {rate}"), || seen(rate, |f| f.rate));
            assert!(radio.status.running.load(Ordering::Relaxed), "the radio stopped at {rate}");
        }
        // Still alive, still hearing, and still holding both channels.
        assert!(radio.status.running.load(Ordering::Relaxed));
        assert_eq!(radio.status.error.lock().clone(), None);
    }

    /// A full duplex radio hears the band through its own transmission.
    ///
    /// The half duplex case is the one every test uses, because it is what a
    /// HackRF is. On a radio with a synthesiser per direction the receiver
    /// must not retune to transmit, must not draw the loopback over the span,
    /// and must go on decoding while the key is down.
    #[test]
    fn a_full_duplex_radio_keeps_receiving_through_an_over() {
        let dev = sources::FileRadio::hearing(
            Hz(145_000_000),
            Sps(2_400_000),
            vec![common::C32::new(0.25, 0.0); 4_096],
        )
        .half_duplex(false)
        .as_fast_as_it_can();
        let watch = dev.watcher();
        let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        until("the radio to say it transmits", || {
            radio.status.can_transmit.load(Ordering::Relaxed)
        });
        radio.send(Cmd::Channels(vec![strip_channel(1, 25_000.0)]));
        radio.send(Cmd::Key(Some(1)));
        until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
        until("something on the antenna", || watch.transmitted_len() > 0);

        // The dial has not moved: that is what the second synthesiser is for,
        // and the spectrum is still arriving from where it was.
        let moved = radio.frames.try_iter().any(|f| (f.center - 145_000_000.0).abs() > 1.0);
        assert!(!moved, "a full duplex radio retuned itself to transmit");
        radio.send(Cmd::Key(None));
        until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
        assert!(radio.status.running.load(Ordering::Relaxed));
    }

    /// Several channels decode at once through the radio thread.
    #[test]
    fn every_channel_on_the_strip_is_built_by_the_radio_thread() {
        let dev = sources::FileRadio::silent(Hz(145_000_000), Sps(2_400_000)).as_fast_as_it_can();
        let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        let want: Vec<ChannelSpec> =
            (1..=4).map(|i| strip_channel(i, i as f64 * 100_000.0 - 250_000.0)).collect();
        radio.send(Cmd::Channels(want));
        // The inputs of the audio bus, which is where a built channel shows
        // up: the levels are only republished when one of them changed.
        let built = || radio.status.strips().inputs.iter().filter(|s| s.channel.is_some()).count();
        until("all four channels to be built", || built() == 4);
        assert_eq!(radio.status.error.lock().clone(), None);
        // And closing one leaves the other three.
        radio.send(Cmd::Channels(vec![strip_channel(1, -150_000.0)]));
        until("three to go", || built() == 1);
        assert!(radio.status.running.load(Ordering::Relaxed));
    }

    /// The walk over a band moves the dial, on the radio thread.
    ///
    /// The node only asks; the thread is the only thing holding the device,
    /// so a walk that is not answered here is a walk that never happens.
    #[test]
    fn a_band_walk_moves_the_dial_through_the_radio_thread() {
        let dev = sources::FileRadio::silent(Hz(145_000_000), Sps(2_400_000)).as_fast_as_it_can();
        let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        radio.send(Cmd::BandScan(crate::chain::BandScan {
            running: true,
            lo_hz: 144e6,
            hi_hz: 146e6,
            step_hz: 500_000.0,
            dwell_s: 0.2,
            on_hit: nodes::OnHit::Log,
        }));
        // Where the spectrum says the receiver is, in the order it went
        // there. 144 to 146 MHz in half megahertz steps is four centres, the
        // first of them half a step inside the low edge.
        let stops = [144_250_000.0, 144_750_000.0, 145_250_000.0, 145_750_000.0];
        let mut walked: Vec<f64> = Vec::new();
        until("the dial to walk three steps", || {
            for f in radio.frames.try_iter() {
                if walked.last().is_none_or(|last| (last - f.center).abs() > 1.0) {
                    walked.push(f.center);
                }
            }
            walked.len() >= 4
        });
        assert_eq!(walked[0], 145_000_000.0, "the dial started somewhere else");
        assert_eq!(walked[1], stops[0], "the first step is not the bottom of the band");
        // Which of the four the rest are is the file's pace rather than the
        // walk's: this radio hands over a capture as fast as the machine
        // will read it, so the walk's own clock runs ahead of a dial that
        // can only be retuned every 120 ms and some asks are overtaken.
        for hz in &walked[1..] {
            assert!(stops.contains(hz), "the dial went to {hz}, which is not a step of {stops:?}");
        }
        assert_eq!(radio.status.error.lock().clone(), None);
        // And stopping the walk leaves the dial where it was.
        radio.send(Cmd::BandScan(crate::chain::BandScan::default()));
        let held = *walked.last().expect("somewhere");
        std::thread::sleep(std::time::Duration::from_millis(400));
        for f in radio.frames.try_iter() {
            if (f.center - held).abs() > 1.0 {
                walked.push(f.center);
            }
        }
        assert_eq!(walked.len(), 4, "the dial kept moving after the walk stopped: {walked:?}");
    }

    /// A capture through the whole receiver, on the radio thread.
    ///
    /// Every other replay test drives `chain::Receiver` directly, which is
    /// the graph but not the loop around it: the command queue, the retune,
    /// the rebuild, the read, the harvest and the publish are the radio
    /// thread's, and none of them was under test. This runs the same capture
    /// through the real thread on a radio made of memory, and pins the same
    /// sensor the replay test pins.
    #[test]
    fn a_capture_played_into_the_radio_thread_is_decoded() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
        if !p.exists() {
            eprintln!("skipping: fineoffset_wh1080_433.92M_250k.cu8 absent, run testdata/fetch.sh");
            return;
        }
        let dev = sources::FileRadio::playing(&p).expect("the capture opens").as_fast_as_it_can();
        let (center, rate) = (Hz(433_920_000), Sps(250_000));
        let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));

        // What the scanner table puts on 433.92: the ISM banks, which is how
        // the live receiver finds a sensor nobody tuned.
        let mut heard = Vec::new();
        until("the sensor to be read", || {
            heard.extend(radio.decodes.try_iter().flatten());
            heard.iter().any(|r| r.model.as_deref() == Some("Fineoffset-WHx080"))
        });
        let r = heard
            .iter()
            .find(|r| r.model.as_deref() == Some("Fineoffset-WHx080"))
            .expect("the loop above found one");
        // The same station the replay test reads off this capture, with the
        // transmitter's own CRC rather than a plausibility argument.
        assert_eq!(r.crc, Some(true), "{r:?}");
        assert!(r.detail.contains("station_id=196"), "read as {}", r.detail);
        assert!(r.detail.contains("temperature_c=16.2"), "read as {}", r.detail);
        assert!(r.detail.contains("humidity_pct=89"), "read as {}", r.detail);
        assert!((r.freq - 433_920_000.0).abs() < 100_000.0, "read at {:.4} MHz", r.freq / 1e6);
        // Every packet reaching the bus carries what it was heard at.
        assert!(r.rssi_dbfs.is_finite() && r.snr_db.is_finite(), "no measurement on {r:?}");
    }

    /// A radio unplugged mid-over ends the over, on the radio thread.
    ///
    /// The key has to come up with it. A lit key over a transmitter that
    /// stopped is worse than no key: nothing else on the screen would say
    /// the transmission had ended.
    #[test]
    fn a_radio_unplugged_mid_over_brings_the_key_up() {
        let center = Hz(446_000_000);
        let rate = Sps(2_400_000);
        let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
        let watch = dev.watcher();
        let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
        until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
        until("the radio to say it transmits", || {
            radio.status.can_transmit.load(Ordering::Relaxed)
        });
        radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));
        radio.send(Cmd::Key(Some(1)));
        until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
        until("something on the antenna", || watch.transmitted_len() > 0);

        watch.unplug();
        until("the key to come up on its own", || radio.status.keyed.load(Ordering::Relaxed) == 0);
        let said = radio.status.error.lock().clone().unwrap_or_default();
        assert!(said.contains("the radio stopped taking samples"), "it said {said:?} instead");
    }

    /// Where an over goes out is not part of the chain that makes it.
    ///
    /// Two channels of one mode are the same stages, so keying between them
    /// is the radio moving and nothing else: no rebuild, and so no restart of
    /// the spectrum's averaging or of whatever the source has open. Comparing
    /// whole plans instead made every such key-up a rebuild, and the rebuild
    /// then found it had nothing to do.
    #[test]
    fn a_frequency_is_not_part_of_the_transmit_chain() {
        use crate::chain::TxPlan;
        let here = TxPlan { spec: TxSpec::default(), mode: TxMode::Nfm, on_air: Hz(145_500_000) };
        let there = TxPlan { on_air: Hz(433_500_000), ..here };
        assert!(here.same_chain(&there));
        assert_ne!(here, there, "they are two plans still: the radio and the monitor read it");

        let mic = TxPlan { spec: TxSpec { source: TxSource::Mic, ..here.spec }, ..here };
        assert!(!here.same_chain(&mic), "a microphone is a different source stage");
        let louder = TxPlan { spec: TxSpec { mic_gain: 9.0, ..mic.spec }, ..mic };
        assert!(!mic.same_chain(&louder), "the gain is a setting on the source stage");
        let shifted = TxPlan { spec: TxSpec { shift_hz: -600_000.0, ..here.spec }, ..here };
        assert!(here.same_chain(&shifted), "a repeater shift only moves the radio");
    }

    #[test]
    fn a_channel_outside_the_span_is_refused_rather_than_demodulated() {
        // Restoring a session tuned elsewhere leaves channels behind that the
        // radio is no longer sampling. Demodulating one shifts a frequency
        // that was never received down to baseband, and the result is noise
        // that sounds like a dead station.
        let rate = 2_400_000.0;
        let inside = ChannelSpec {
            id: 1,
            label: String::new(),
            offset_hz: -400_000.0,
            mode: ChanMode::Audio(Demod::Wfm),
            bandwidth_hz: None,
            squelch_db: None,
            voice: false,
            agc: true,
            tx: None,
        };
        let outside = ChannelSpec { id: 2, offset_hz: -994_200_000.0, ..inside.clone() };
        assert!(inside.offset_hz.abs() <= rate / 2.0);
        assert!(outside.offset_hz.abs() > rate / 2.0, "95.8 MHz is not inside a 1090 MHz span");
    }

    #[test]
    fn each_channel_keeps_its_own_station() {
        // Two WFM channels are normally two different stations, and a shared
        // slot printed the first one's name under both.
        let s = Status::default();
        let named = |n: &str, pi: u16| dsp::rds::Station {
            pi: Some(pi),
            name: Some(n.into()),
            ..Default::default()
        };
        s.set_station(1, &named("SPIRIT", 0x2208), 10, 0, true);
        s.set_station(2, &named("HEART", 0xC479), 8, 1, true);

        assert_eq!(s.station_for(1).unwrap().name.as_deref(), Some("SPIRIT"));
        assert_eq!(s.station_for(2).unwrap().name.as_deref(), Some("HEART"));
        assert_eq!(s.station_for(2).unwrap().groups, 8);
        // A channel with no RDS shows nothing rather than a neighbour's name.
        assert!(s.station_for(3).is_none());

        // A removed or rebuilt channel takes its station with it.
        s.keep_stations(&[2]);
        assert!(s.station_for(1).is_none());
        assert_eq!(s.station_for(2).unwrap().name.as_deref(), Some("HEART"));
    }

    fn block(n: usize) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = std::f64::consts::TAU * 0.1 * i as f64;
                C32::new(p.cos() as f32 * 0.5, p.sin() as f32 * 0.5)
            })
            .collect()
    }

    /// A carrier `offset_hz` from the centre, modulated by an audio tone.
    ///
    /// Enough to tell a demodulator that works from one that does not: an SSB
    /// receiver tuned to the carrier should hear the tone at its own pitch.
    ///
    /// `start` is the sample index the block begins at, because a receiver
    /// hears one continuous signal and not the same block over and over: a
    /// buffer replayed back to back has a phase step at every seam, and that
    /// step is a click with energy on both sidebands. It measured as 58 dB of
    /// apparent leakage into a sideband the filter actually rejects by 93 dB.
    pub(crate) fn ssb_signal(
        rate: f64,
        carrier_hz: f64,
        tone_hz: f64,
        start: usize,
        n: usize,
    ) -> Vec<C32> {
        (start..start + n)
            .map(|i| {
                let t = i as f64 / rate;
                // One sideband only: a single complex exponential at the
                // carrier plus the tone is exactly what an SSB transmitter
                // puts on the air for a single audio tone.
                let p = std::f64::consts::TAU * (carrier_hz + tone_hz) * t;
                C32::new(0.3 * p.cos() as f32, 0.3 * p.sin() as f32)
            })
            .collect()
    }

    fn audio_rms(pcm: &[f32]) -> f32 {
        if pcm.is_empty() {
            return 0.0;
        }
        (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len() as f32).sqrt()
    }

    /// How far down a station on the wrong sideband is, through the whole
    /// chain.
    ///
    /// Measured through the AGC's gain rather than the audio level, because
    /// the AGC drives everything to the same level by design: a rejected
    /// signal comes out as loud as a wanted one and 60 dB more amplified, so
    /// the audio level says nothing about rejection and the gain says
    /// everything.
    fn sideband_rejection_db(mode: Demod, wanted_hz: f64, image_hz: f64) -> f32 {
        let rate = 2_304_000.0;
        let offset = 120_000.0;
        let mut gains = [0.0f32; 2];
        for (i, tone) in [wanted_hz, image_hz].iter().enumerate() {
            let mut a = Audio::new(offset, rate, mode, 48_000.0);
            // Four seconds of audio. A rejected signal takes the AGC's whole
            // release to climb to where it settles, and the gain is held flat
            // during the hang, so anything that watches for the gain to stop
            // moving stops early and measures the hang instead of the filter.
            const N: usize = 262_144;
            for k in 0..36 {
                a.process(&ssb_signal(rate, offset, *tone, k * N, N), 1.0);
            }
            gains[i] = a.agc_gain_db();
        }
        gains[1] - gains[0]
    }

    #[test]
    fn upper_sideband_hears_a_station_above_the_dial_and_not_below() {
        let sep = sideband_rejection_db(Demod::Usb, 1_000.0, -1_000.0);
        assert!(sep > 30.0, "the wrong sideband was only {sep:.1} dB down");
    }

    #[test]
    fn lower_sideband_is_the_other_way_round() {
        let sep = sideband_rejection_db(Demod::Lsb, -1_000.0, 1_000.0);
        assert!(sep > 30.0, "the wrong sideband was only {sep:.1} dB down");
    }

    #[test]
    fn a_weak_ssb_signal_comes_out_at_the_same_level_as_a_strong_one() {
        // What the AGC is for: two stations 40 dB apart should not need the
        // volume control moved between them.
        let rate = 2_304_000.0;
        let offset = 120_000.0;
        let mut loud = Audio::new(offset, rate, Demod::Usb, 48_000.0);
        let mut quiet = Audio::new(offset, rate, Demod::Usb, 48_000.0);

        const N: usize = 262_144;
        let block = |k: usize| ssb_signal(rate, offset, 1_000.0, k * N, N);
        let quieter = |b: &[C32]| b.iter().map(|s| s * 0.01).collect::<Vec<_>>();
        // A few blocks each, because the gain needs a moment to settle and
        // the first block is where it is still moving.
        for k in 0..3 {
            loud.process(&block(k), 1.0);
            quiet.process(&quieter(&block(k)), 1.0);
        }
        let a = 20.0 * audio_rms(loud.process(&block(3), 1.0)).max(1e-9).log10();
        let b = 20.0 * audio_rms(quiet.process(&quieter(&block(3)), 1.0)).max(1e-9).log10();
        assert!(
            (a - b).abs() < 6.0,
            "a 40 dB difference at the antenna came out as {:.1} dB of audio",
            a - b
        );
    }

    #[test]
    fn cw_is_tuned_so_the_dial_reads_the_carrier() {
        // Tuned exactly to a Morse carrier, the operator should hear the
        // pitch, not silence and not some arbitrary beat note.
        let rate = 2_304_000.0;
        let offset = 120_000.0;
        const N: usize = 262_144;
        let mut cw = Audio::new(offset, rate, Demod::Cw, 48_000.0);
        for k in 0..3 {
            cw.process(&ssb_signal(rate, offset, 0.0, k * N, N), 1.0);
        }
        let on = audio_rms(cw.process(&ssb_signal(rate, offset, 0.0, 3 * N, N), 1.0));
        assert!(on > 0.02, "a carrier on the dial frequency produced {on:.4} of audio");

        // And a station 2 kHz away is outside a 500 Hz filter.
        let mut cw2 = Audio::new(offset, rate, Demod::Cw, 48_000.0);
        for k in 0..3 {
            cw2.process(&ssb_signal(rate, offset, 2_000.0, k * N, N), 1.0);
        }
        let off = audio_rms(cw2.process(&ssb_signal(rate, offset, 2_000.0, 3 * N, N), 1.0));
        assert!(off < on / 10.0, "a station 2 kHz away was audible at {off:.4} against {on:.4}");
    }

    fn fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    /// A silent buffer, for building a receiver whose shape is the point.
    fn empty_buf(rate: f64, center: Hz) -> common::IqBuf {
        common::IqBuf {
            samples: vec![C32::default(); 1024],
            rate: Sps(rate as u64),
            center,
            seq: 0,
        }
    }

    #[test]
    fn each_bank_splits_the_span_to_the_width_its_front_end_wants() {
        for rate in [250_000.0, 1_024_000.0, 2_400_000.0, 20_000_000.0] {
            for (want, lo, hi) in [
                (12_500.0, 6_000.0, 30_000.0),
                (OOK_CHANNEL_HZ, 15_000.0, 70_000.0),
                // The wide tier of the scanner table, in `scanners::DEFAULT_WIDTHS`.
                (125_000.0, 60_000.0, 260_000.0),
                (500_000.0, 240_000.0, 1_100_000.0),
            ] {
                let n = nodes::BankNode::channels_for(rate, want);
                assert_eq!(n % 2, 0, "{rate} at {want} Hz gave an odd count {n}");
                let width = rate / n as f64;
                assert!(
                    (lo..hi).contains(&width) || n == 2,
                    "{rate} at {want} Hz gave {n} channels of {width} Hz"
                );
            }
        }
    }

    #[test]
    fn an_ism_band_is_watched_for_sources_and_not_channelized() {
        // What the shipped table asks for on an ISM band: one detector that
        // finds transmitters where they are, rather than a set of channel
        // grids at guessed widths.
        let rx = replay_receiver(&empty_buf(2_400_000.0, Hz::mhz(868)), None).unwrap();
        let labels: Vec<String> = rx.topology().nodes.iter().map(|n| n.label.clone()).collect();
        assert!(rx.has_sources(), "no source detector on the 868 MHz band: {labels:?}");
        assert!(
            rx.bank_channels().is_empty(),
            "a bank tier is still running: {:?}",
            rx.bank_channels()
        );
        assert!(rx.live_sources().is_empty(), "an empty band has no sources");
    }

    #[test]
    fn a_narrow_span_still_gets_a_usable_bank() {
        // Two channels is the floor: the channelizer needs an even count and
        // one channel would just be a decimator.
        assert_eq!(nodes::BankNode::channels_for(1_000.0, OOK_CHANNEL_HZ), 2);
        assert!(
            nodes::BankNode::channels_for(1e9, OOK_CHANNEL_HZ) <= 1024,
            "the count has to stay bounded"
        );
    }

    /// An ISM sensor through the whole receiver and out to a broker.
    ///
    /// The publisher, the node and the packet bus each have tests of their
    /// own; what none of them said is that a sensor heard by the live
    /// receiver reaches a broker, which is the only thing an operator can
    /// see. A socket that speaks enough MQTT to accept the connection stands
    /// in for the broker, and what arrives on it is what Home Assistant
    /// would read.
    #[test]
    fn a_sensor_heard_by_the_receiver_reaches_the_house() {
        use std::io::{Read, Write};
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
        if !p.exists() {
            eprintln!("skipping: fineoffset_wh1080_433.92M_250k.cu8 absent, run testdata/fetch.sh");
            return;
        }
        let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();

        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("a port");
        let port = listener.local_addr().unwrap().port();
        let (tx, seen) = std::sync::mpsc::channel::<(String, String)>();
        std::thread::spawn(move || {
            let (mut sock, _) = listener.accept().expect("a connection");
            let mut held = Vec::new();
            let mut chunk = [0u8; 4096];
            loop {
                let n = match sock.read(&mut chunk) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                held.extend_from_slice(&chunk[..n]);
                while let Some((kind, flags, body, used)) = nodes::mqtt_packet(&held) {
                    held.drain(..used);
                    match kind {
                        1 => {
                            if sock.write_all(&[0x20, 0x02, 0x00, 0x00]).is_err() {
                                return;
                            }
                        }
                        3 => {
                            let tl = u16::from_be_bytes([body[0], body[1]]) as usize;
                            let topic = String::from_utf8_lossy(&body[2..2 + tl]).to_string();
                            let at = 2 + tl + if (flags >> 1) & 3 > 0 { 2 } else { 0 };
                            let payload = String::from_utf8_lossy(&body[at..]).to_string();
                            if tx.send((topic, payload)).is_err() {
                                return;
                            }
                        }
                        _ => {}
                    }
                }
            }
        });

        let mut rx = replay_receiver(&buf, None).expect("a receiver");
        let mut plan = replay_plan(&buf, false);
        plan.settings.homeassistant = Some(nodes::Publish {
            broker: nodes::Broker { port, ..nodes::Broker::new("127.0.0.1") },
            spaces: "ism".into(),
            buses: true,
        });
        rx.apply_settings(&plan);
        let up = std::time::Instant::now();
        while !rx.homeassistant_status().is_some_and(|s| s.connected)
            && up.elapsed() < std::time::Duration::from_secs(5)
        {
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let st = rx.homeassistant_status();
        assert!(st.as_ref().is_some_and(|s| s.connected), "never connected: {st:?}");

        let rows = replay_blocks(&mut rx, &buf);
        assert!(rows.iter().any(|r| r.model == Some("Fineoffset-WHx080")), "{rows:?}");
        let st = rx.homeassistant_status().unwrap();
        assert_eq!(st.devices, 1, "{st:?}");
        assert_eq!(st.dropped, 0, "{st:?}");

        let mut got: Vec<(String, String)> = Vec::new();
        while let Ok(m) = seen.recv_timeout(std::time::Duration::from_secs(2)) {
            got.push(m);
            if got.iter().any(|(t, _)| t.ends_with("/state")) {
                break;
            }
        }
        let topics: Vec<&str> = got.iter().map(|(t, _)| t.as_str()).collect();
        let state = got
            .iter()
            .find(|(t, _)| {
                t.starts_with("waveshark/ism_fineoffset_whx080/") && t.ends_with("/state")
            })
            .unwrap_or_else(|| panic!("no reading reached the broker: {topics:?}"));
        assert!(state.1.contains("\"temperature_c\""), "{}", state.1);
        assert!(
            topics
                .iter()
                .any(|t| t.starts_with("homeassistant/sensor/waveshark_ism_fineoffset_whx080_")
                    && t.ends_with("/temperature_c/config")),
            "{topics:?}"
        );
    }

    /// The M17 capture: three seconds of a busy 433 MHz band with an
    /// OpenRTX handheld on the calling channel, recorded 550 kHz off centre.
    fn m17_fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/m17_openrtx_434.02M_2400k.cu8");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    /// The 5.8 GHz camera: an AKK RaceRunner with a PAL camera on it.
    fn camera_fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/pal_camera_5865M_20000k.cs8");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    /// A camera through the whole receiver, which is where it was broken.
    ///
    /// `decode::video` read this capture from the first commit, because that
    /// test hands it the samples. The receiver never saw it: the source
    /// detector refuses a run wider than the widest narrowband signal, so a
    /// carrier megahertz wide never opened as one source, the runs inside it
    /// opened instead, and a camera arrived as a packet list of sensors that
    /// were not there.
    /// A still that grew is published again. Keying on the picture number
    /// alone left the pane holding the first line of an SSTV transmission
    /// while the bus filled the rest in.
    #[test]
    fn a_picture_filling_in_is_published_each_time() {
        let status = Status::default();
        let frame = |lines: usize| common::VideoFrame {
            system: "SSTV",
            channel_hz: 144_500_000.0,
            label: Some("Martin 1".into()),
            width: 2,
            height: 4,
            aspect: 4.0 / 3.0,
            pixels: common::Pixels::Rgb8,
            samples: std::sync::Arc::new(vec![0u8; 2 * 4 * 3]),
            lines_seen: lines,
            sequence: 1,
            update: common::Update::Whole,
            cadence: common::Cadence::Still,
        };
        status.set_video(Some(frame(1)));
        assert_eq!(status.video().map(|f| f.lines_seen), Some(1));
        status.set_video(Some(frame(3)));
        assert_eq!(status.video().map(|f| f.lines_seen), Some(3), "the picture grew");
        status.set_video(None);
        assert!(status.video().is_none(), "and it can be cleared");
    }

    #[test]
    fn a_camera_reaches_the_video_bus_through_the_receiver() {
        let Some(buf) = camera_fixture() else {
            eprintln!("skipping: pal_camera_5865M_20000k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).expect("a receiver");
        let _ = replay_blocks(&mut rx, &buf);
        let bus = rx.video().expect("a video bus");
        let fed = bus.bus().channels().iter().filter(|c| c.is_fed()).count();
        let picture = bus.bus().thumbnails().next().is_some();
        assert!(picture, "no picture on the video bus; {fed} channels fed");
        // And once it has a picture the camera owns the span: it asked for
        // the band, and the auto node closed the detector out of it.
        let owned = rx
            .live_sources()
            .into_iter()
            .find(|s| s.locked_to == Some("video"))
            .expect("the camera never claimed its band");
        // The channel of the plan, which is what a camera occupies and what
        // the front end was placed on. Not the sampled span: the front end
        // reads a band-limited 10 MS/s of it and cannot claim what it was
        // never handed, so the auto node widens the claim to the band it
        // placed the decoder on.
        assert!(owned.bandwidth_hz >= 18e6, "{owned:?}");

        // And the sound that came with it. A camera puts audio on a
        // subcarrier of the same transmission, 6.5 MHz up the baseband on
        // this one, and it arrives on a voice port like any other speech: a
        // picture with no sound is half a receiver.
        let heard = rx.voices();
        let sound = heard.iter().find(|v| v.system == "analogue video").expect("no sound");
        assert!(sound.rate > 30e3 && sound.rate < 60e3, "{} Hz", sound.rate);
        // It names no party, because it has none: the sound half of a
        // transmission is not a call, and a row for it would say only that a
        // transmitter is on the air, which the picture says already.
        assert_eq!(sound.to, None);
        let peak = sound.pcm.iter().fold(0.0f32, |a: f32, s: &f32| a.max(s.abs()));
        // The capture is a quiet room, so this is small; what it may not be
        // is zero, which is what a subcarrier nobody demodulated sounds
        // like.
        assert!(peak > 1e-3, "the sound port carried silence, peak {peak}");
    }

    /// The radiosonde capture: 40 s of a Vaisala RS41 recorded near London
    /// and published by SDRangel, at its own centre with the sonde 9.76 kHz
    /// off it.
    fn rs41_fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/rs41_herstmonceux_405.80024M_31.25k.cs16");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    /// A weather balloon read the whole way through the receiver: detector,
    /// raster, remembered channel, decoder, map.
    ///
    /// Everything about this capture is hostile to a decoder placed on a
    /// source and nothing else. The transmission is 4 dB over the floor and
    /// lasts 534 ms a second, so the detector opens it late and closes it
    /// again, measuring a slightly different centre each time; the sonde sits
    /// two thirds of the way to the edge of a 31 kHz span. What makes it work
    /// is the band's 10 kHz raster turning forty measurements into one
    /// channel, and the latch keeping that channel once a frame has decoded
    /// on it. Take either away and this reads one frame in forty seconds.
    #[test]
    fn a_radiosonde_is_found_and_tracked_through_the_receiver() {
        let Some(buf) = rs41_fixture() else {
            eprintln!(
                "skipping: rs41_herstmonceux_405.80024M_31.25k.cs16 absent, run testdata/fetch.sh"
            );
            return;
        };
        fn field<'a>(r: &'a DecodeRecord, k: &str) -> Option<&'a common::Value> {
            r.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v)
        }
        let mut rx = replay_receiver(&buf, None).expect("a receiver");
        let rows = replay_blocks(&mut rx, &buf);
        let sonde: Vec<&DecodeRecord> = rows.iter().filter(|r| r.model == Some("rs41")).collect();
        // 28 of the 40 transmissions in the capture. The first seven go to
        // finding it: a channel is not remembered until a frame has decoded
        // on it, and until then every burst is a fresh decoder that started
        // after the header had already gone by.
        assert_eq!(sonde.len(), 28, "{} sonde frames", sonde.len());
        assert!(sonde.iter().all(|r| r.crc == Some(true)), "a frame failed a block CRC");
        assert!(
            sonde.iter().all(|r| r.bytes.len() == decode::rs41::FRAME_STD),
            "a frame was not a standard 320 byte one"
        );

        // One sonde, named, on one channel of the raster.
        let serials: std::collections::BTreeSet<String> =
            sonde.iter().filter_map(|r| r.identity.as_ref().map(|i| i.id.clone())).collect();
        assert_eq!(serials, ["S1720982".to_string()].into_iter().collect());
        for r in &sonde {
            assert_eq!(r.freq, 405_810_000.0, "off the 10 kHz raster");
            assert_eq!(r.modulation, common::Modulation::Fsk2);
        }

        // Consecutive frame numbers, which is the sonde's own clock: one a
        // second, none missed once it is being tracked.
        let nums: Vec<i64> =
            sonde.iter().filter_map(|r| field(r, "frame").and_then(|v| v.as_i64())).collect();
        assert_eq!(nums.len(), 28);
        assert_eq!(nums[0], 3409, "{nums:?}");
        assert_eq!(*nums.last().unwrap(), 3441, "{nums:?}");
        assert!(nums.windows(2).all(|w| w[1] > w[0]), "out of order: {nums:?}");
        // Once it has the channel it holds it: twenty in a row without a
        // gap, through twenty transmitter silences of 466 ms each.
        let run = nums
            .windows(2)
            .fold((1, 1), |(best, run), w| {
                let run = if w[1] == w[0] + 1 { run + 1 } else { 1 };
                (best.max(run), run)
            })
            .0;
        assert_eq!(run, 20, "longest unbroken run in {nums:?}");

        // And where it was: climbing through 10.3 km over Sussex, drifting
        // east, which is a 12 UTC Herstmonceux sounding an hour after launch.
        let last = sonde.last().unwrap();
        let f = |k: &str| field(last, k).and_then(|v| v.as_f64()).unwrap_or(f64::NAN);
        assert!((f("altitude_m") - 10_500.5).abs() < 1.0, "{}", f("altitude_m"));
        assert!((f("climb_ms") - 4.30).abs() < 0.05, "{}", f("climb_ms"));
        assert!((f("battery_v") - 2.7).abs() < 0.05, "{}", f("battery_v"));
        assert_eq!(field(last, "state").map(|v| v.to_string()).as_deref(), Some("ascending"));

        // The map reads the tracker and the tracker reads `position`, so
        // this is the test that a balloon is drawn: one track, labelled with
        // the serial, with a trail behind it. It drifted 900 m east across
        // the capture, which is the whole of the trail.
        let tracks = rx.tracks(std::time::Instant::now());
        assert_eq!(tracks.len(), 1, "{tracks:?}");
        let t = &tracks[0];
        assert_eq!(t.id, crate::tracks::TrackId::Sonde("S1720982".into()));
        assert_eq!(t.id.system(), "Radiosonde");
        let (lat, lon) = t.position.expect("no fix on the map");
        assert!((lat - 50.7898).abs() < 1e-3, "{lat}");
        assert!((lon - 0.9226).abs() < 1e-3, "{lon}");
        // One point per frame read, and a balloon at 10 km in a westerly
        // drifting east across the whole of them.
        assert_eq!(t.trail.len(), 28);
        let east = t.trail.last().unwrap().1 - t.trail[0].1;
        assert!((east - 0.0230).abs() < 1e-3, "drifted {east} degrees east");

        // And its thermometer, which is what the balloon was sent up for.
        //
        // A sonde sends a sixteenth of its factory calibration a second, so
        // there is no temperature on the first frame and there is one by the
        // end: the counts in a frame are ratios, and the tracker is where
        // the pieces that turn them into degrees are joined. Twenty-eight
        // pieces here, one per frame read.
        let crate::tracks::Detail::Sonde {
            temperature_c,
            humidity_pct,
            calibration_pieces,
            altitude_m,
            descending,
            ..
        } = t.detail
        else {
            panic!("the track is not a sonde: {:?}", t.detail);
        };
        assert_eq!(calibration_pieces, 28);
        assert!(!descending);
        assert!((altitude_m - 10_500.5).abs() < 1.0, "{altitude_m}");
        // The Met Office published this ascent: Herstmonceux (03882), 00 UTC
        // on 27 December 2021, which is the hour the frames' own GPS time
        // gives. Its profile reads -59.4 C at 10475 m and -59.3 C at 10586 m,
        // against the -59.1 C read here at 10500 m. Nothing about that number
        // comes from this repository: it is a second reduction, of the same
        // balloon, from the station that launched it.
        let t_c = temperature_c.expect("no temperature");
        assert!((t_c + 59.1).abs() < 0.2, "{t_c} C against the published -59.4 at 10475 m");
        // Humidity is the empirical fit rather than Vaisala's own reduction,
        // and the published profile says 55% at 10475 m.
        let rh = humidity_pct.expect("no humidity");
        assert!((rh - 55.7).abs() < 0.5, "{rh}% against the published 55%");
    }

    /// The BLE capture: 2 s of advertising channel 38, tuned onto the channel
    /// so the packets are read across the tuner's own DC spike.
    fn ble_fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/offair/gfsk_ble_2426M_20000k.cs8");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    /// Bluetooth advertising, through the whole receiver: the 2.4 GHz block
    /// puts `auto` on the span, `auto` runs the BLE front end across it
    /// because channel 38 is inside it, and what comes back is what the
    /// devices in the room were saying.
    ///
    /// Every packet counted here passed the link layer's CRC-24, so a run
    /// that produces the wrong number is a demodulator that got worse rather
    /// than a threshold that moved.
    #[test]
    fn bluetooth_advertising_is_found_and_read() {
        let Some(buf) = ble_fixture() else {
            eprintln!("skipping: gfsk_ble_2426M_20000k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        // The shipped table rather than whatever is in this machine's config:
        // a block the operator deleted should not fail the corpus.
        let fronts =
            crate::scanners::Scanners::default().fronts(buf.center.as_f64(), buf.rate.as_f64());
        assert!(
            fronts.iter().any(|f| f.front == crate::scanners::Front::Auto),
            "the table put nothing on a span covering channel 38: {fronts:?}"
        );
        let mut plan = replay_plan(&buf, false);
        plan.fronts = fronts;
        let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
        let out = replay_blocks(&mut rx, &buf);
        let ble: Vec<&DecodeRecord> = out.iter().filter(|r| r.model == Some("BLE-Adv")).collect();
        assert!(ble.len() >= 6, "read {} advertisements, expected the 8 in the capture", ble.len());
        for r in &ble {
            assert_eq!(r.crc, Some(true), "a packet without its CRC got through: {r:?}");
            assert!(
                (r.freq - 2_426_000_000.0).abs() < 1e6,
                "reported at {} Hz rather than on channel 38",
                r.freq
            );
            assert!(r.detail.contains("channel=38"), "read as {}", r.detail);
            assert!(r.detail.contains("address="), "no address in {}", r.detail);
        }
        every_row_carries_its_measurements(&ble);
    }

    fn wifi_fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/offair/ofdm_wifi_frames_2462M_20000k.cs8");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    /// 802.11 through the whole receiver: the table puts `auto` on a 20 MHz
    /// span, `auto` runs the Wi-Fi front end across it because channel 11 is
    /// what the span is, and what comes back is the traffic between an access
    /// point and one station.
    ///
    /// The capture was cut around forty-one frames identified by bandwidth
    /// when it was recorded; the receiver reads ninety-four, because the
    /// margin either side of each one holds traffic too, three of them are
    /// 802.11n aggregates whose subframes are frames in their own right, and
    /// the rest are 802.11b at 1 Mbit/s, which the bandwidth rule that cut
    /// the capture was selecting against. Every one passed a CRC-32 over the
    /// whole frame, so the count is evidence rather than a threshold: fewer
    /// is a receiver that got worse.
    #[test]
    fn wifi_frames_are_found_and_read() {
        let Some(buf) = wifi_fixture() else {
            eprintln!("skipping: ofdm_wifi_frames_2462M_20000k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        let fronts =
            crate::scanners::Scanners::default().fronts(buf.center.as_f64(), buf.rate.as_f64());
        assert!(
            fronts.iter().any(|f| f.front == crate::scanners::Front::Auto),
            "the table put nothing on a span covering channel 11: {fronts:?}"
        );
        let mut plan = replay_plan(&buf, false);
        plan.fronts = fronts;
        let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
        let out = replay_blocks(&mut rx, &buf);
        let wifi: Vec<&DecodeRecord> = out.iter().filter(|r| r.model == Some("802.11")).collect();
        assert!(wifi.len() >= 88, "read {} frames, expected 94", wifi.len());
        for r in &wifi {
            assert_eq!(r.crc, Some(true), "a frame without its FCS got through: {r:?}");
            assert!(
                (r.freq - 2_462_000_000.0).abs() < 1e6,
                "reported at {} Hz rather than on channel 11",
                r.freq
            );
            assert!(r.detail.contains("channel=11"), "read as {}", r.detail);
        }
        // The two devices talking to each other, by their own addresses.
        let all = wifi.iter().map(|r| r.detail.clone()).collect::<Vec<_>>().join(" ");
        assert!(
            wifi.iter().any(|r| {
                r.link
                    .as_ref()
                    .and_then(|l| l.from.as_ref())
                    .is_some_and(|p| p.label().contains("70:03:9F:0D:A9:8D"))
            }),
            "the station that sent the data frames is not named: {all}"
        );
        // The 802.11n frames in the capture: MCS 7 with the short guard
        // interval, carried inside an aggregate.
        let ht: Vec<&&DecodeRecord> =
            wifi.iter().filter(|r| r.detail.contains("phy=MCS")).collect();
        assert!(!ht.is_empty(), "no HT frame read");
        for r in &ht {
            assert!(r.detail.contains("aggregated=1"), "{}", r.detail);
        }
        every_row_carries_its_measurements(&wifi);
    }

    /// DJI DroneID through the whole receiver: the 2.4 GHz block puts `auto`
    /// on the span, `auto` runs the DroneID front end across it because one
    /// of the five centres is inside it, and what comes back is the aircraft
    /// naming itself.
    ///
    /// The count and the sequence numbers are the same seven bursts
    /// `nodes/tests/droneid_capture.rs` reads through the node alone, so a
    /// difference between the two is the receiver around the front end and
    /// not the front end. Nothing decoded here at all until the front end
    /// read the span rather than a source cut out of it, because the source
    /// the detector opened for a 720 us burst is not the 10 MHz channel the
    /// frame occupies.
    #[test]
    fn a_drone_naming_itself_is_read_off_the_span() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/droneid_mini4k_2444.5M_15360k.cs8");
        if !p.exists() {
            eprintln!("skipping: droneid_mini4k_2444.5M_15360k.cs8 absent, run testdata/fetch.sh");
            return;
        }
        let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
        let mut rx = replay_receiver(&buf, None).unwrap();
        let out = replay_blocks(&mut rx, &buf);
        let dji: Vec<&DecodeRecord> =
            out.iter().filter(|r| r.model == Some("DJI-DroneID")).collect();
        let seq: Vec<String> = dji
            .iter()
            .filter_map(|r| {
                r.detail.split_whitespace().find(|w| w.starts_with("sequence=")).map(str::to_string)
            })
            .collect();
        assert_eq!(seq.len(), 7, "read {} bursts: {seq:?}", dji.len());
        assert_eq!(
            seq,
            [437, 439, 440, 440, 441, 442, 444].map(|n| format!("sequence={n}")).to_vec()
        );
        for r in &dji {
            assert_eq!(r.crc, Some(true), "a frame without its CRC got through: {r:?}");
            assert!(r.detail.contains("serial=F8PJC254J001JR4R"), "read as {}", r.detail);
            assert!(
                (r.freq - 2_444_500_000.0).abs() < 1e6,
                "reported at {} Hz rather than on the centre it was read on",
                r.freq
            );
        }
        every_row_carries_its_measurements(&dji);
    }

    /// 802.11b beacons off the mixed capture: the same band, the same access
    /// point, and the announcement rather than the traffic.
    ///
    /// This capture was labelled `mixed` and its 3.4 millisecond bursts
    /// recorded as unidentified. They are 1 Mbit/s direct sequence beacons,
    /// which is what the interval of 102.4 ms says and what the receiver now
    /// reads: a network name, an access point, and a check over the whole
    /// frame.
    #[test]
    fn beacons_name_their_network() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/offair/ofdm_wifi_2462M_20000k.cs8");
        if !p.exists() {
            eprintln!("skipping: ofdm_wifi_2462M_20000k.cs8 absent, run testdata/fetch.sh");
            return;
        }
        let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
        let mut plan = replay_plan(&buf, false);
        plan.fronts =
            crate::scanners::Scanners::default().fronts(buf.center.as_f64(), buf.rate.as_f64());
        let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
        let out = replay_blocks(&mut rx, &buf);
        let wifi: Vec<&DecodeRecord> = out.iter().filter(|r| r.model == Some("802.11")).collect();
        let beacons: Vec<&&DecodeRecord> =
            wifi.iter().filter(|r| r.detail.contains("type=beacon")).collect();
        assert!(!beacons.is_empty(), "no beacon read from {} frames", wifi.len());
        for b in &beacons {
            assert!(b.detail.contains("ssid=darknet"), "{}", b.detail);
            assert!(b.detail.contains("phy=1 Mbit/s"), "{}", b.detail);
            assert_eq!(b.crc, Some(true));
        }
        every_row_carries_its_measurements(&wifi);
    }

    /// The rule every row in the list obeys, whichever front end made it: a
    /// level, a signal to noise ratio, and the samples it was read from.
    ///
    /// Without these a row cannot be sorted by strength, a fade cannot be
    /// told from a decoder that broke, and there is nothing to look at when
    /// the bytes are wrong. Frames used to lose all three at the port
    /// boundary, which carried bytes and nothing else, so every front end
    /// that produces frames rather than pulses reported NaN.
    /// A GSM beacon through the whole receiver: the scanner table puts the
    /// GSM front end on the carrier, the front end finds the tone, reads the
    /// burst a frame later, and the row that reaches the list names the cell.
    ///
    /// Synthetic, and that is the weakness worth writing down: the modulator
    /// here and the demodulator under test share every assumption either of
    /// them makes about GSM. What it does prove is the wiring, which is where
    /// a front end usually breaks: that the table's channel reaches the node,
    /// that the extraction leaves the carrier inside the span it hands over,
    /// and that what the node puts on the bus comes back out of the packet
    /// list as a cell rather than as four unexplained bytes.
    #[test]
    fn a_gsm_beacon_is_read_through_the_receiver() {
        let center = Hz(947_400_000);
        let rate = 2_400_000.0;
        let want = dsp::gsm::Sch { ncc: 5, bcc: 3, frame_number: 51 * 26 * 42 + 21 };
        let buf = common::IqBuf::new(gsm_beacon(&want), center, common::Sps(rate as u64), 0);

        let mut plan = replay_plan(&buf, false);
        plan.fronts = vec![crate::scanners::FrontAt {
            front: crate::scanners::Front::protocol("gsm", center.as_f64()),
            band: (center.as_f64() - 200_000.0, center.as_f64() + 200_000.0),
        }];
        let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();
        let out = replay_blocks(&mut rx, &buf);

        let cells: Vec<&DecodeRecord> = out.iter().filter(|r| r.model == Some("GSM-SCH")).collect();
        assert_eq!(cells.len(), 2, "expected both bursts, got {out:?}");
        let r = cells[0];
        assert_eq!(r.crc, Some(true), "the parity is what makes a burst a burst");
        assert!(r.detail.contains("ARFCN 62"), "read as {}", r.detail);
        assert!(r.detail.contains("BSIC 53"), "read as {}", r.detail);
        assert!(r.detail.contains("frame 55713"), "read as {}", r.detail);
        every_row_carries_its_measurements(&cells);

        // And the block the broadcast channel carried in the four frames
        // after it, which is the row that says whose cell this is.
        let si: Vec<&DecodeRecord> = out.iter().filter(|r| r.model == Some("GSM-SI")).collect();
        assert_eq!(si.len(), 1, "expected one system information block, got {out:?}");
        assert_eq!(si[0].detail, "SI3 262-01 LAC 100 CI 4660");
        every_row_carries_its_measurements(&si);
    }

    /// A frequency correction burst, the synchronisation burst one TDMA
    /// frame after it, and the four bursts of broadcast channel the
    /// multiframe puts after that, at the receiver's rate and with a little
    /// noise so the floor a level is measured against is a floor.
    fn gsm_beacon(sch: &dsp::gsm::Sch) -> Vec<common::C32> {
        use dsp::gsm;
        let sps = 8;
        let work = gsm::SYMBOL_RATE * sps as f64;
        let lead = 200.0;
        let total = ((lead * 2.0 + 13.0 * gsm::FRAME_SYMBOLS) * sps as f64) as usize;
        let mut base = vec![common::C32::new(0.0, 0.0); total];
        let mut place = |at: f64, wave: &[common::C32]| {
            let at = (at * sps as f64) as usize;
            base[at..at + wave.len()].copy_from_slice(wave);
        };
        // Two beacons ten frames apart, which is what the control multiframe
        // holds and what the receiver needs: a synchronisation burst is
        // reported only once a second one agrees with it about the time.
        for n in 0..2u32 {
            let at = lead + 10.0 * f64::from(n) * gsm::FRAME_SYMBOLS;
            let this = gsm::Sch { frame_number: sch.frame_number + 10 * n, ..*sch };
            place(at, &gsm::modulate(&[0u8; gsm::BURST_BITS], sps));
            place(
                at + gsm::FRAME_SYMBOLS,
                &gsm::modulate(&gsm::sch_burst_bits(&this).unwrap(), sps),
            );
        }
        // A system information type 3 on the broadcast channel, in the four
        // frames after the first synchronisation burst: the cell identity
        // and the location area, which is what a receiver is here for.
        let mut block = [0x2Bu8; 23];
        block[..10].copy_from_slice(&[0x49, 0x06, 0x1B, 0x12, 0x34, 0x62, 0xF2, 0x10, 0x00, 0x64]);
        for (n, data) in gsm::bcch::encode(&block).unwrap().iter().enumerate() {
            let bits = gsm::normal_burst_bits(data, usize::from(sch.bcc));
            place(lead + (2.0 + n as f64) * gsm::FRAME_SYMBOLS, &gsm::modulate(&bits, sps));
        }

        let ratio = work / 2_400_000.0;
        let n = (base.len() as f64 / ratio) as usize - 1;
        let mut seed = 0x1357_9BDFu32;
        let mut rand = move || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            (seed as f32 / u32::MAX as f32) - 0.5
        };
        // A little noise, so the floor the level is measured against is a
        // floor rather than a divide by zero.
        (0..n)
            .map(|i| base[(i as f64 * ratio) as usize] + common::C32::new(rand(), rand()) * 0.05)
            .collect()
    }

    fn every_row_carries_its_measurements(rows: &[&DecodeRecord]) {
        assert!(!rows.is_empty(), "nothing to check");
        for r in rows {
            assert!(r.rssi_dbfs.is_finite(), "{} has no level: {:?}", r.protocol(), r.rssi_dbfs);
            assert!(r.snr_db.is_finite(), "{} has no SNR: {:?}", r.protocol(), r.snr_db);
            let iq = r.iq.as_ref().unwrap_or_else(|| panic!("{} kept no samples", r.protocol()));
            assert!(!iq.samples.is_empty(), "{} kept an empty burst", r.protocol());
            assert!(iq.rate > 0.0 && iq.center_hz > 0, "{} samples with no stream", r.protocol());
        }
    }

    /// A survey built by replaying the BLE capture through the whole
    /// receiver, with the GPS saying the receiver was in one place.
    ///
    /// The point of the test is the seam between the three parts: the front
    /// end finds packets, the survey node turns a decode into an identity,
    /// and the database holds one row per transmitter with the position the
    /// receiver was at. A device list built from a capture whose contents are
    /// known is the only way to see all three working at once.
    #[test]
    fn a_survey_records_the_devices_heard_and_where_from() {
        let Some(buf) = ble_fixture() else {
            eprintln!("skipping: gfsk_ble_2426M_20000k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        let fronts =
            crate::scanners::Scanners::default().fronts(buf.center.as_f64(), buf.rate.as_f64());
        let mut plan = replay_plan(&buf, false);
        plan.fronts = fronts;
        let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default()).unwrap();

        let dir = std::env::temp_dir().join(format!("waveshark-survey-{}", std::process::id()));
        let path = dir.join("survey.sqlite");
        let _ = std::fs::remove_file(&path);
        plan.settings.survey_path = Some(path.clone());
        rx.apply_settings(&plan);
        rx.set_fix(Some(gps::Fix {
            lat: 53.5137,
            lon: -6.2431,
            hdop: Some(0.9),
            ..Default::default()
        }));

        let _ = replay_blocks(&mut rx, &buf);
        let rows = rx.survey_devices(survey::Query::default());
        assert!(!rows.is_empty(), "the capture decodes and nothing was recorded");
        assert!(rows.iter().all(|d| d.protocol == "ble"), "{rows:?}");
        // The advertiser that dominates this capture.
        let d = rows
            .iter()
            .find(|d| d.ident == "6C:70:CB:EF:72:4D")
            .unwrap_or_else(|| panic!("the Samsung advertiser is missing: {rows:?}"));
        assert!(d.packets >= 4, "only {} receptions attributed to it", d.packets);
        // A sighting says where the receiver was, not where the device is.
        let s = rx.survey_sightings(d.id);
        assert!(!s.is_empty());
        assert_eq!((s[0].lat, s[0].lon), (Some(53.5137), Some(-6.2431)));
        assert_eq!(d.best_lat, Some(53.5137), "the strongest sighting keeps its position");
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn tetra_fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/tetra_downlink_391.5M_2400k.cu8");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    fn lora_fixture(which: char) -> Option<common::IqBuf> {
        let name = match which {
            // Tuned 525 kHz under the channel at 2.4 MS/s, so the packet is
            // off centre and no rate divides to two samples a chip.
            'c' => "lora_sf11_meshtastic_c_869.0M_2400k.cu8",
            _ => "lora_sf11_meshtastic_a_869.525M_2000k.cs16",
        };
        let name = if which == 'b' { "lora_sf11_meshtastic_b_869.525M_2000k.cs16" } else { name };
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join(format!("../../testdata/offair/{name}"));
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }

    /// A Meshtastic transmission the receiver has to find, measure, decide
    /// is a chirp, and read, with nothing told to it.
    ///
    /// This is the capture energy detection could not see: LoRa spreads its
    /// power under the noise and the excursion inside the channel stayed
    /// below 10 dB, so two capture sessions were written off as empty before
    /// the source detector was rewritten. Every stage between the samples
    /// and the row is being tested here: the detector opening a source that
    /// wide, the auto node placing a LoRa front end on it because the width
    /// is one of its channels, the demodulator finding the spreading factor
    /// without being told, and the frame behind it passing both the header
    /// checksum and the transmitter's own CRC.
    #[test]
    fn a_meshtastic_transmission_is_found_and_read() {
        for which in ['a', 'b', 'c'] {
            let Some(buf) = lora_fixture(which) else {
                eprintln!("skipping: fixture absent, run testdata/fetch.sh");
                return;
            };
            let mut rx = replay_receiver(&buf, None).unwrap();
            let out = replay_blocks(&mut rx, &buf);
            let rows: Vec<String> = out
                .iter()
                .map(|r| format!("{:.4} MHz {} {}", r.freq / 1e6, r.protocol(), r.detail))
                .collect();
            let r = out
                .iter()
                .find(|r| r.model == Some("Meshtastic"))
                .unwrap_or_else(|| panic!("capture {which}: nothing read it: {rows:?}"));
            // The transmitter's CRC, not a plausibility argument.
            assert_eq!(r.crc, Some(true), "capture {which}: {r:?}");
            assert!(r.detail.contains("SF11 BW250k 4/5"), "capture {which}: read as {}", r.detail);
            // Both captures are from a node addressing the whole mesh.
            assert!(r.detail.contains("to everyone"), "capture {which}: read as {}", r.detail);
            let hz = r.freq;
            assert!((hz - 869_525_000.0).abs() < 250_000.0, "capture {which}: read at {hz} Hz");
            if which == 'c' {
                // What the node said, against the public default key.
                assert!(r.detail.contains("050d3664 to everyone"), "capture c: {}", r.detail);
                assert!(r.detail.contains("\"Hi\""), "capture c: {}", r.detail);
            }
        }
    }

    /// A MeshCore advert on the European preset, which is the one LoRa
    /// channel the receiver did not have: 62.5 kHz at SF8, from a node in
    /// the same building, strong enough that the detector measures it at
    /// up to twice its width and the front end lifts the whole span while
    /// it lasts. Found, placed, read, and the node's name and position
    /// come out of the signed advert.
    #[test]
    fn a_meshcore_advert_is_found_and_read() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/offair/meshcore_advert_868.9M_2048k.cu8");
        if !p.exists() {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        }
        let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
        let mut rx = replay_receiver(&buf, None).unwrap();
        let out = replay_blocks(&mut rx, &buf);
        let rows: Vec<String> = out
            .iter()
            .map(|r| format!("{:.4} MHz {} {}", r.freq / 1e6, r.protocol(), r.detail))
            .collect();
        let r = out
            .iter()
            .find(|r| r.model == Some("MeshCore"))
            .unwrap_or_else(|| panic!("nothing read it: {rows:?}"));
        assert_eq!(r.crc, Some(true), "{r:?}");
        assert!(r.detail.contains("SF8 BW63k 4/8"), "read as {}", r.detail);
        assert!(r.detail.contains("\"Kieran\""), "read as {}", r.detail);
        assert!((r.freq - 869_618_000.0).abs() < 62_500.0, "read at {} Hz", r.freq);
        // The packet carries what it was: its samples and its level.
        assert!(r.iq.as_ref().is_some_and(|q| !q.samples.is_empty()), "no samples on the row");
        assert!(r.snr_db.is_finite() && r.rssi_dbfs.is_finite(), "no level on the row");
    }

    /// Two TETRA base station downlinks, on for every one of the capture's
    /// ten seconds, that the receiver never reports.
    #[test]
    fn a_permanent_tetra_downlink_is_found() {
        let Some(buf) = tetra_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).unwrap();
        let mut seen: Vec<(f64, Option<f32>)> = Vec::new();
        for block in buf.samples.chunks(16_384) {
            if rx.process(block).is_err() {
                break;
            }
            for s in rx.live_sources() {
                if !seen.iter().any(|(hz, _)| (hz - s.center_hz).abs() < 12_500.0) {
                    seen.push((s.center_hz, s.snr_db));
                }
            }
        }
        let near = |hz: f64| seen.iter().any(|(c, _)| (c - hz).abs() < 12_500.0);
        assert!(near(391_181_000.0), "391.181 MHz was never opened: {seen:?}");
        assert!(near(391_704_500.0), "391.7045 MHz was never opened: {seen:?}");
    }

    /// The key manager reaches a front end the receiver built for itself.
    ///
    /// Nothing here places a TETRA stage: the scanner table watches the band,
    /// the auto node builds a front end on each carrier it finds, and the key
    /// status is read by asking every node in the receiver whether it takes
    /// keys. Read off a stage held by name, an encrypted network the receiver
    /// found for itself could never be given one.
    #[test]
    fn the_key_manager_reaches_a_front_end_the_receiver_found_for_itself() {
        let Some(buf) = tetra_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).unwrap();
        replay_blocks(&mut rx, &buf);
        let keys = rx.tetra_key_status();
        // One row per cell heard, both on the same network, as the cells
        // themselves broadcast it: the control carrier at 391.175 MHz says
        // its traffic is enciphered (air interface encryption 3) and the
        // other carrier is in the clear.
        assert_eq!(keys.len(), 2, "{keys:?}");
        assert!(keys.iter().all(|k| (k.mcc, k.mnc) == (272, 6838)), "{keys:?}");
        let mut colours: Vec<u8> = keys.iter().map(|k| k.colour).collect();
        colours.sort();
        assert_eq!(colours, [3, 5], "{keys:?}");
        assert!(keys.iter().any(|k| k.aie == 3), "the encrypting cell was not seen: {keys:?}");
    }

    /// Finding the carriers is half of it. The scanner block promises that
    /// each is measured and logged, so the list says which channels are
    /// busy; a source that never closes never used to reach the list at all,
    /// because a burst front end reports a burst when it ends.
    #[test]
    fn a_permanent_tetra_downlink_is_logged_with_its_measurement() {
        let Some(buf) = tetra_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        // Replayed in two halves with the graph rebuilt between them, which
        // is what a retune or a setting does live: every source's decoders
        // are built again.
        let plan = replay_plan(&buf, false);
        let mut rx = crate::chain::Receiver::build(&plan, Default::default()).unwrap();
        let half = buf.samples.len() / 2;
        let first = common::IqBuf::new(buf.samples[..half].to_vec(), buf.center, buf.rate, 0);
        let second =
            common::IqBuf::new(buf.samples[half..].to_vec(), buf.center, buf.rate, half as u64);
        let mut out = replay_blocks(&mut rx, &first);
        rx.rebuild(&plan).unwrap();
        out.extend(replay_blocks(&mut rx, &second));
        let rows: Vec<String> = out
            .iter()
            .map(|r| {
                format!("{:.4} MHz {} {} {}", r.freq / 1e6, r.protocol(), r.modulation, r.detail)
            })
            .collect();
        for hz in [391_181_000.0, 391_704_500.0] {
            let mine: Vec<&DecodeRecord> =
                out.iter().filter(|r| (r.freq - hz).abs() < 12_500.0).collect();
            assert!(!mine.is_empty(), "{:.4} MHz was never logged: {rows:?}", hz / 1e6);
            // Every row says what it was heard at: the front end measures the
            // slot each block came out of rather than handing over a frame
            // with nothing on it.
            every_row_carries_its_measurements(&mine);
            // Logged as the channel the plan lists, not as this tuner's
            // measurement of it: the band is on a 25 kHz raster and the
            // carrier was found a few kilohertz off it.
            let channel = (hz / 25_000.0).round() * 25_000.0;
            assert!(
                mine.iter().all(|r| (r.freq - channel).abs() < 1.0),
                "{:.4} MHz was logged at {:?}",
                hz / 1e6,
                mine.iter().map(|r| r.freq).collect::<Vec<_>>()
            );
            // The cell's identity once, and once only, though its decoders
            // were built twice.
            let sync = mine.iter().filter(|r| r.model == Some("TETRA-Sync")).count();
            let sysinfo = mine.iter().filter(|r| r.model == Some("TETRA-Sysinfo")).count();
            assert_eq!((sync, sysinfo), (1, 1), "{:.4} MHz: {rows:?}", hz / 1e6);
            assert!(
                mine.iter().any(|r| r.detail.contains("mcc=272")),
                "{:.4} MHz: the network was not named",
                hz / 1e6
            );
            // The cell's own map of its neighbours, read off the network
            // broadcast the control carrier sends: every list it cycles
            // through once, each cell on this network's band. The other
            // carrier is an idle traffic carrier and broadcasts nothing but
            // its identity.
            let network: Vec<&&DecodeRecord> =
                mine.iter().filter(|r| r.model == Some("TETRA-Network")).collect();
            if hz == 391_181_000.0 {
                assert!(!network.is_empty(), "{:.4} MHz: no network broadcast: {rows:?}", hz / 1e6);
            }
            assert!(network.len() <= 8, "{:.4} MHz: {} network rows", hz / 1e6, network.len());
            for r in &network {
                let cells: Vec<&(String, common::Value)> =
                    r.fields.iter().filter(|(k, _)| k.starts_with("cell_")).collect();
                assert!(!cells.is_empty(), "{:?}", r.fields);
                for (_, v) in cells {
                    let text = v.to_string();
                    let mhz: f64 = text
                        .split(" at ")
                        .nth(1)
                        .and_then(|t| t.split(' ').next())
                        .and_then(|m| m.parse().ok())
                        .unwrap_or_else(|| panic!("no frequency in {text:?}"));
                    assert!((390.0..400.0).contains(&mhz), "{text}");
                }
            }
            // A measurement of what the carrier looks like is not news once
            // a front end is reading it: at most the one piece cut before
            // the front end found its first sync burst.
            let measured: Vec<&&DecodeRecord> = mine.iter().filter(|r| r.model.is_none()).collect();
            assert!(
                measured.len() <= 1,
                "{:.4} MHz measured {} times while being read: {rows:?}",
                hz / 1e6,
                measured.len()
            );
            assert!(
                measured.iter().all(
                    |r| r.modulation == common::Modulation::Dqpsk && r.detail.contains("TETRA")
                ),
                "{:.4} MHz was measured as {:?}",
                hz / 1e6,
                measured
                    .iter()
                    .map(|r| format!("{} {}", r.modulation, r.detail))
                    .collect::<Vec<_>>()
            );
        }
    }

    /// The network in the capture enciphers its air interface, so no call
    /// control PDU is readable; the MAC headers are, and they say which
    /// groups are being addressed. That is worth logging, with what protects
    /// it, so a key that undoes it later has a row to change. It is not
    /// worth a call row: an enciphered SDU addressed to a radio is as likely
    /// to be a registration or a data session as speech, and this twelve
    /// seconds contains no traffic channel grant to say otherwise.
    #[test]
    fn an_encrypting_tetra_network_names_its_busy_groups_but_not_as_calls() {
        let Some(buf) = tetra_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).unwrap();
        let out = replay_blocks(&mut rx, &buf);
        let calls: Vec<&DecodeRecord> =
            out.iter().filter(|r| r.model == Some("TETRA-Call")).collect();
        assert!(!calls.is_empty(), "no call rows from {} rows", out.len());
        let field = |r: &DecodeRecord, k: &str| {
            r.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string())
        };
        let groups: Vec<String> = calls.iter().filter_map(|r| field(r, "to")).collect();
        assert!(groups.iter().any(|g| g == "10223295" || g == "15835885"), "addressed {groups:?}");
        assert!(
            calls.iter().all(|r| field(r, "encryption").as_deref() == Some("AIE-3")),
            "{:?}",
            calls
                .iter()
                .map(|r| format!(
                    "{:?} {:?} {:?} {:?}",
                    field(r, "pdu"),
                    field(r, "to"),
                    field(r, "encryption"),
                    r.detail
                ))
                .collect::<Vec<_>>()
        );
        // Not a row per slot: an address that keeps being addressed is one
        // row every couple of seconds.
        assert!(calls.len() <= 12, "{} rows in twelve seconds", calls.len());

        // A MAC header says an address is being talked to, not that anybody
        // is talking: behind an enciphered SDU it is as likely to be a radio
        // registering or a data session. Those rows belong in the log and
        // not in a list of voice calls.
        let mut list = crate::calls::Calls::new();
        for r in calls.iter().filter(|r| field(r, "pdu").as_deref() == Some("MAC-RESOURCE")) {
            assert!(!list.update(r, r.at), "a bare MAC header earned a call row: {:?}", r.fields);
        }
        assert!(
            list.is_empty(),
            "nothing here proved a voice call: {:?}",
            calls.iter().filter_map(|r| field(r, "pdu")).collect::<Vec<_>>()
        );
    }

    #[test]
    fn an_m17_handheld_on_a_busy_band_is_found_and_read() {
        // Synthesised M17 does not fail the way this capture did. A generated
        // transmission is measured wide enough to clear every width threshold
        // in the receiver; a real one, cleanly shaped, measures a couple of
        // kilohertz at the detector's twenty decibel extent, and three
        // thresholds threw it away in turn: the channel decoders were not
        // built for a source that narrow, the extraction filtered the stream
        // down to the two bins that were measured, and the tracker let a
        // neighbouring signal take the runs it needed to stay open.
        let Some(buf) = m17_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).unwrap();
        let out = replay_blocks(&mut rx, &buf);

        let m17: Vec<&DecodeRecord> =
            out.iter().filter(|r| r.protocol().starts_with("M17")).collect();
        assert!(!m17.is_empty(), "nothing read as M17 from {} rows", out.len());
        // The callsign is in the link setup frame that opens the
        // transmission and repeated across the link information channel, so
        // reading it back means the demodulator, the framing, the Golay and
        // the CRC all worked on a signal nobody synthesised.
        assert!(
            m17.iter().any(|r| r.detail.contains("from=OPNRTX")),
            "no callsign: {:?}",
            m17.iter().map(|r| &r.detail).take(4).collect::<Vec<_>>()
        );
        // The receiver was told no frequency at all, so this is the
        // detector's own answer, within a couple of channel widths of the
        // calling channel.
        let hz = m17[0].freq;
        assert!((hz - 433_475_000.0).abs() < 25_000.0, "read at {hz} Hz");
        // Most of the over, not a frame or two of it. A receiver that opens a
        // source, reads three frames and loses it is the failure this capture
        // was recorded for.
        let voice = m17.iter().filter(|r| r.model == Some("M17-Voice")).count();
        assert!(voice >= 20, "only {voice} voice frames of a 2.5 second over");
        // And each of those rows says how it was heard. The front end that
        // read them measures the channel itself; it used to hand over frames
        // with no level at all and rely on whatever placed it to fill one in.
        every_row_carries_its_measurements(&m17);
    }

    #[test]
    fn a_decode_channel_reads_its_frequency_with_the_scanner_switched_off() {
        // The point of a decode channel: one front end at a fixed centre and
        // width, and nothing else running. Before this the only way to read
        // one frequency was a scanner block, which searched the span it
        // covered whether or not anything else in it was wanted.
        let Some(buf) = m17_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut plan = replay_plan(&buf, false);
        plan.fronts.clear();
        plan.channels = vec![ChannelSpec {
            id: 1,
            label: "M17".into(),
            offset_hz: 433_475_000.0 - buf.center.as_f64(),
            mode: ChanMode::Decode("m17".into()),
            bandwidth_hz: None,
            squelch_db: None,
            voice: false,
            agc: true,
            tx: None,
        }];
        let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a channel");
        let out = replay_blocks(&mut rx, &buf);

        let m17: Vec<&DecodeRecord> =
            out.iter().filter(|r| r.protocol().starts_with("M17")).collect();
        assert!(!m17.is_empty(), "nothing read as M17 from {} rows", out.len());
        assert!(
            m17.iter().any(|r| r.detail.contains("from=OPNRTX")),
            "no callsign: {:?}",
            m17.iter().map(|r| &r.detail).take(4).collect::<Vec<_>>()
        );
        // The frequency the channel was set to, not one anything searched
        // for: a decode channel is told where to listen.
        let hz = m17[0].freq;
        assert!((hz - 433_475_000.0).abs() < 1.0, "read at {hz} Hz");
        // Placed by the strip, so nothing above it measures anything: the
        // auto node's fill is not in this path at all, and a row still has
        // its level, its ratio to the floor and the samples behind it.
        every_row_carries_its_measurements(&m17);
    }

    #[test]
    fn an_auto_channel_finds_and_reads_what_is_in_its_own_bandwidth() {
        // An auto channel is the scanner table's front end pointed by hand:
        // no block covers this capture's frequency, nothing was told what the
        // signal is, and the width searched is the one set on the strip.
        let Some(buf) = m17_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut plan = replay_plan(&buf, false);
        plan.fronts.clear();
        plan.channels = vec![ChannelSpec {
            id: 1,
            label: "Watch".into(),
            offset_hz: 433_475_000.0 - buf.center.as_f64(),
            mode: ChanMode::Auto,
            bandwidth_hz: Some(100_000.0),
            squelch_db: None,
            voice: false,
            agc: true,
            tx: None,
        }];
        let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a channel");
        let out = replay_blocks(&mut rx, &buf);

        let m17: Vec<&DecodeRecord> =
            out.iter().filter(|r| r.protocol().starts_with("M17")).collect();
        assert!(!m17.is_empty(), "nothing read as M17 from {} rows", out.len());
        assert!(
            m17.iter().any(|r| r.detail.contains("from=OPNRTX")),
            "no callsign: {:?}",
            m17.iter().map(|r| &r.detail).take(4).collect::<Vec<_>>()
        );
        // The detector's own answer, in absolute frequency: a source found
        // inside the channel is reported where it is on the dial.
        let hz = m17[0].freq;
        assert!((hz - 433_475_000.0).abs() < 25_000.0, "read at {hz} Hz");
    }

    #[test]
    fn a_transmission_the_receiver_found_itself_is_audible() {
        // Decoding a call and being able to hear it are two different
        // things, and for a while this receiver did the first without the
        // second: live speech was read from the one M17 stage the scanner
        // table places, so every transmission the auto node found for itself
        // played back as silence. Speech is a port now and the call bus is a
        // node on the end of it, so this is the whole path from the air to
        // the mixer.
        let Some(buf) = m17_fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).unwrap();
        let calls = rx.calls_mut().expect("the calls are always there");
        calls.set_subscriptions(vec![crate::mix::calls::Subscription::new(
            crate::mix::calls::Rule::Everything,
        )]);

        let mut pcm: Vec<f32> = Vec::new();
        let mut heard = None;
        for block in buf.samples.chunks(16_384) {
            if rx.process(block).is_err() {
                break;
            }
            // One side of the stereo mix, which carries speech on both.
            pcm.extend(rx.audio_out().0.iter().step_by(2));
            heard = heard.or_else(|| rx.audio().and_then(|b| b.last_heard()).map(str::to_string));
        }
        assert_eq!(heard.as_deref(), Some("OPNRTX to BROADCAST"), "nobody was heard");
        // The bus resamples to its output rate, and the over is seconds
        // long. Half a second of it is enough to say the vocoder ran on live
        // frames and the mix reached the far end.
        let seconds = pcm.len() as f64 / rx.audio().unwrap().out_rate();
        assert!(seconds > 0.5, "only {seconds:.2} s of speech");
        // Speech, not a run of zeros: a decoder that returns silence for
        // every frame would pass every assertion above.
        let rms = (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len() as f32).sqrt();
        assert!(rms > 1e-3, "the mix is silent at {rms:e} rms");
    }

    #[test]
    fn the_scanner_decodes_a_real_transmission_without_being_tuned_to_it() {
        // Nothing here selects a frequency, a modulation or a protocol. The
        // capture is fed in as if it had just arrived from the device.
        let Some(buf) = fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).unwrap();
        let out = replay_blocks(&mut rx, &buf);

        assert!(!out.is_empty(), "nothing decoded from a capture that contains a packet");
        // Unrecognised bursts are reported too, so pick out the real one
        // rather than assuming it arrived first.
        let r = out
            .iter()
            .find(|r| r.model == Some("Fineoffset-WHx080"))
            .unwrap_or_else(|| panic!("only unknowns: {out:?}"));
        assert_eq!(r.crc, Some(true), "{r:?}");
        assert!(r.detail.contains("temperature_c=16.2"), "{}", r.detail);
        // Structured, not just printed: a chart or a map has to be able to
        // read a field without parsing the summary line back apart.
        assert_eq!(
            r.fields.iter().find(|(k, _)| k == "temperature_c").map(|(_, v)| v.as_f64()),
            Some(Some(16.2))
        );
        assert_eq!(r.modulation, common::Modulation::Ook);
        // A real reception from a recording made near full scale: strong, and
        // well clear of the noise.
        assert!(r.snr_db > 6.0, "snr came out as {}", r.snr_db);
        // Referenced to full scale at the detector, so filter gain can put a
        // very strong packet slightly over zero. What matters is that it is a
        // real measurement rather than a placeholder.
        assert!((-60.0..=6.0).contains(&r.rssi_dbfs), "rssi came out as {} dB", r.rssi_dbfs);
        // One row, not five: the FSK branch reads the same burst and the
        // neighbouring channels see its skirts, and all of that is one packet.
        assert_eq!(out.len(), 1, "the same burst was logged more than once: {out:#?}");
        // The frequency reported is the channel's, not the tuner's, which is
        // what makes a waterfall mark land on the signal.
        let off = (r.freq - buf.center.as_f64()).abs();
        assert!(off < buf.rate.as_f64() / 2.0, "{} Hz is outside the span", r.freq);
    }

    #[test]
    fn a_decode_is_stamped_when_the_block_started_not_when_it_finished() {
        // Arithmetic rather than a race against the clock. This used to
        // compare the stamp against an instant taken before the whole replay,
        // which quietly asserted that decoding beat real time: every block
        // processed before the first decode had to fit inside one block's
        // worth of signal. On a loaded machine, or in a debug build, it failed
        // for a reason that had nothing to do with stamping, which is the
        // other test's job.
        let finished = std::time::Instant::now();
        let at = block_start(finished, 16_384, 250_000.0);
        // 16384 samples at 250 kS/s is 65.536 ms of signal.
        let back = finished.duration_since(at).as_secs_f64();
        assert!((back - 0.065_536).abs() < 1e-9, "stamped {back}s before the block ended");
        assert!(at < finished, "the stamp must precede the block it came from");
        // A rate of zero must not divide by it.
        assert!(block_start(finished, 16_384, 0.0) < finished);
    }

    /// And the same thing where it is actually used, which no arithmetic test
    /// can check: a replayed decode must not be stamped in the future.
    #[test]
    fn a_replayed_decode_is_not_stamped_in_the_future() {
        let Some(buf) = fixture() else {
            eprintln!("skipping: fixture absent, run testdata/fetch.sh");
            return;
        };
        let mut rx = replay_receiver(&buf, None).unwrap();
        let out = replay_blocks(&mut rx, &buf);
        let done = std::time::Instant::now();
        let rec = out.first().expect("a decode");
        assert!(rec.at < done, "a decode is stamped after the replay that produced it");
    }

    #[test]
    fn retuning_clears_state_rather_than_carrying_it_across() {
        // A burst half-collected at one frequency must not finish at another.
        let mut rx = replay_receiver(&empty_buf(250_000.0, Hz::mhz(433)), None).unwrap();
        rx.process(&block(8192)).unwrap();
        let mut plan = plan_at(250_000.0, Hz::mhz(868));
        plan.fronts = vec![crate::scanners::FrontAt {
            front: crate::scanners::Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
            band: (0.0, f64::INFINITY),
        }];
        rx.rebuild(&plan).unwrap();
        rx.process(&block(8192)).unwrap();
        let out = rx.decodes(std::time::Instant::now());
        assert!(out.is_empty(), "a steady tone decoded as {out:?}");
    }

    #[test]
    fn the_scanner_keeps_up_with_the_stream() {
        // Decoding the whole span is only worth having if it runs in real
        // time; if it does not, it is stealing from the thread that has to
        // drain USB and the radio drops samples instead.
        if cfg!(debug_assertions) {
            eprintln!("skipping: an unoptimised build says nothing about throughput");
            return;
        }
        // SCAN_RATE=16000000 asks the same question of a wideband span.
        let rate =
            std::env::var("SCAN_RATE").ok().and_then(|v| v.parse().ok()).unwrap_or(2_400_000.0);
        let mut rx = replay_receiver(&empty_buf(rate, Hz::mhz(868)), None).unwrap();
        let b = block(262_144);
        // One pass to warm the filters and the pool.
        rx.process(&b).unwrap();

        let t = std::time::Instant::now();
        let blocks = 20;
        for _ in 0..blocks {
            rx.process(&b).unwrap();
        }
        let secs = t.elapsed().as_secs_f64();
        let audio_secs = blocks as f64 * b.len() as f64 / rate;
        let x = audio_secs / secs;
        eprintln!("scanner: {x:.1}x real time watching for sources");
        assert!(x > 1.0, "the scanner ran at only {x:.2}x real time");
    }

    #[test]
    fn scratch_buffers_do_not_grow_across_blocks() {
        // Every stage appends to its output. If one is not cleared it grows
        // without bound and each block re-filters the whole history, which
        // looks like the radio slowly seizing up rather than an obvious fault.
        // Measured by time rather than by reaching into buffers, which the
        // chain no longer exposes now it is a graph. A buffer that is never
        // cleared refilters its whole history, so the cost per block climbs;
        // that is the symptom either way.
        let mut a = Audio::new(120_000.0, 2_304_000.0, Demod::Wfm, 48_000.0);
        let b = block(8192);
        // The quickest of three passes, because a shared machine's jitter only
        // ever adds time: a loaded CI runner read 3.3x off single passes of a
        // chain that was not growing at all.
        let cost = |a: &mut Audio| {
            (0..3)
                .map(|_| {
                    let t = std::time::Instant::now();
                    for _ in 0..10 {
                        a.process(&b, 0.5);
                    }
                    t.elapsed().as_secs_f64()
                })
                .fold(f64::MAX, f64::min)
        };
        let first = cost(&mut a);
        for _ in 0..5 {
            cost(&mut a);
        }
        let later = cost(&mut a);
        assert!(later < first * 3.0, "cost per block climbed from {first:.4}s to {later:.4}s");
        assert!(a.pcm.len() <= b.len(), "audio output grew");
    }

    #[test]
    fn output_length_is_steady_block_to_block() {
        let mut a = Audio::new(0.0, 2_304_000.0, Demod::Nfm, 48_000.0);
        let b = block(4800);
        let first = a.process(&b, 0.5).len();
        for _ in 0..10 {
            let n = a.process(&b, 0.5).len();
            assert!(
                (n as i64 - first as i64).abs() <= 1,
                "block produced {n} samples after {first}"
            );
        }
    }

    #[test]
    fn every_mode_runs_faster_than_real_time() {
        // The audio chain shares the radio thread with USB draining, so
        // anything near 1x drops samples.
        //
        // The bound is deliberately far below what any developer machine
        // manages, because it has to hold on the slowest shared CI runner too:
        // this one reads 6.4x here and 3.0x on a two core VM. It is a guard
        // against a chain that has gone accidentally quadratic, not a
        // performance target. Real numbers come from --bench-audio.
        // Only meaningful in a release build. The workspace optimises
        // dependencies in dev but not the crate under test, so this code runs
        // unoptimised here and reads a fraction of real time no matter how
        // healthy the chain is. CI runs the suite with --release, which is
        // where the guard bites.
        if cfg!(debug_assertions) {
            eprintln!("skipping: throughput is only measurable in a release build");
            return;
        }
        let rate = 2_304_000.0;
        let b = block(131_072);
        for mode in [Demod::Wfm, Demod::Nfm, Demod::Am, Demod::Usb, Demod::Cw] {
            let mut a = Audio::new(120_000.0, rate, mode, 48_000.0);
            a.process(&b, 0.5);
            let t = std::time::Instant::now();
            for _ in 0..4 {
                a.process(&b, 0.5);
            }
            let x = (4.0 * b.len() as f64 / rate) / t.elapsed().as_secs_f64();
            assert!(x > 1.5, "{} only ran at {x:.1}x real time", mode.label());
        }
    }

    #[test]
    fn the_chain_does_not_delay_the_audio_audibly() {
        // Driving the IF rate down to the channel bandwidth leaves no
        // transition band and asks for thousands of taps, which shows up as
        // delay. The graph adds up each filter's group delay, so this catches
        // it wherever in the chain it happens rather than at one filter that
        // was remembered to be checked.
        for mode in [Demod::Wfm, Demod::Nfm, Demod::Am, Demod::Usb, Demod::Cw] {
            let a = Audio::new(0.0, 2_304_000.0, mode, 48_000.0);
            let ms = a.latency_ms();
            assert!(ms < 40.0, "{} delays audio by {ms:.1} ms", mode.label());
        }
    }

    #[test]
    fn the_chain_is_a_graph_with_every_stage_named() {
        // The point of building on the graph: the stages can be listed, which
        // is what the chain view draws.
        let a = Audio::new(0.0, 2_304_000.0, Demod::Wfm, 48_000.0);
        let topo = a.topology();
        let names: Vec<&str> = topo.nodes.iter().map(|n| n.label.as_str()).collect();
        assert!(names.contains(&"Mixer"), "{names:?}");
        assert!(names.contains(&"WFM demod"), "{names:?}");
        assert!(names.contains(&"High blend"), "{names:?}");
        assert!(names.iter().all(|n| !n.is_empty()));
    }

    #[test]
    fn am_skips_de_emphasis_and_uses_an_envelope_detector() {
        let a = Audio::new(0.0, 2_304_000.0, Demod::Am, 48_000.0);
        let topo = a.topology();
        let names: Vec<&str> = topo.nodes.iter().map(|n| n.label.as_str()).collect();
        assert!(names.contains(&"AM envelope"), "{names:?}");
        assert!(!names.contains(&"De-emphasis"), "{names:?}");
    }

    #[test]
    fn the_audio_rate_is_close_to_what_was_asked_for() {
        for mode in [Demod::Wfm, Demod::Nfm, Demod::Am, Demod::Usb, Demod::Cw] {
            let a = Audio::new(0.0, 2_304_000.0, mode, 48_000.0);
            let r = a.audio_rate();
            assert!((r - 48_000.0).abs() < 12_000.0, "{} gave {r} Hz", mode.label());
        }
    }

    /// A PMR446 handheld through the whole receiver, from IQ to words.
    ///
    /// The path this proves is the one that has no test anywhere else: the
    /// span is decimated to a channel, the channel is demodulated as narrow
    /// FM, its audio goes on the bus as speech because the channel says it
    /// is voice, the transcriber on the bus tap collects it, and a line of
    /// text comes out with the words that were spoken into the handheld. Any
    /// one of those failing shows up here as an empty transcript, which is
    /// exactly what it looks like on screen.
    ///
    /// Skipped without a model, since fetching one is not something a test
    /// should do to somebody's machine.
    #[cfg(feature = "stt")]
    #[test]
    fn a_handheld_on_pmr446_arrives_as_words() {
        let Some(buf) = pmr446_fixture() else {
            eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        let dir = crate::chain::default_model_dir();
        if !dir.join("config.json").exists() {
            eprintln!("skipping: no whisper model in {}", dir.display());
            return;
        }
        // PMR446 channel 1. The capture is tuned 49.1 kHz below it, which is
        // what the dial was set to rather than anything about the signal.
        const CHANNEL_HZ: f64 = 446_049_100.0;
        let mut plan = replay_plan(&buf, false);
        plan.fronts.clear();
        plan.channels = vec![ChannelSpec {
            id: 1,
            label: "PMR1".into(),
            offset_hz: CHANNEL_HZ - buf.center.as_f64(),
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: None,
            // Open: the transmission is what the file holds, and a squelch
            // decision is not what this test is about.
            squelch_db: Some(-200.0),
            agc: true,
            voice: true,
            tx: None,
        }];
        let since = std::time::Instant::now();
        let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");
        // Transcription is off in the graph the receiver draws, because
        // writing down what people said is not something to start doing
        // because nobody said otherwise. Switching it on is what an operator
        // does, and is what this test is about.
        let id = rx.node_of_stage(crate::chain::derived::TRANSCRIBE).expect("a transcriber");
        rx.set_node_param(id.0, "enabled", pipeline::ParamValue::Bool(true)).expect("the switch");
        // The receiver's own transcript, so what this test reads is what
        // this receiver heard.
        let log = rx.transcript().clone();
        let _ = replay_blocks(&mut rx, &buf);
        // The model runs on its own thread, so the answer arrives after the
        // samples have run out, the way it does in the receiver.
        let silence = vec![C32::default(); 16_384];
        let mut said: Vec<crate::transcripts::Utterance> = Vec::new();
        for _ in 0..600 {
            let _ = rx.process(&silence);
            said = log
                .lock()
                .recent(usize::MAX)
                .into_iter()
                .filter(|u| u.at >= since && u.key.channel_hz == CHANNEL_HZ as u64)
                .cloned()
                .collect();
            if said.iter().any(|u| u.settled) {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        let text = said.iter().map(|u| u.text.as_str()).collect::<Vec<_>>().join(" ");
        let words = text.to_lowercase();
        // One transmission is one line. More than one means the utterance
        // was cut where nobody paused, which is the failure this count is
        // here to catch; none means nothing reached the model at all.
        assert_eq!(said.len(), 1, "read as {text:?}");
        // Spaces removed before looking for the digits: the model writes a
        // spoken "one two three" as "123" or as "1 2 3" depending on what it
        // makes of the pauses, and both are the number that was said.
        let digits = words.replace(char::is_whitespace, "");
        assert!(digits.contains("123"), "read as {text:?}");
        assert!(words.contains("test"), "read as {text:?}");
        // On the channel it was heard on, since the key is what the call
        // list and the transcript view meet on.
        let key = said[0].key.clone();
        assert_eq!(key.channel_hz, CHANNEL_HZ as u64, "read on {key}");
    }

    /// The handheld on one analogue strip, with the channel's voice mark
    /// either way. Returns what the tap heard, the calls it made and how many
    /// blocks the speaker was handed nothing on.
    fn pmr446_strip(
        buf: &common::IqBuf,
        voice: bool,
    ) -> (Vec<crate::mix::heard::LiveCall>, crate::calls::Calls, usize, f32) {
        let mut plan = replay_plan(buf, false);
        plan.fronts.clear();
        plan.channels = vec![ChannelSpec {
            id: 1,
            label: "PMR1".into(),
            offset_hz: 446_049_100.0 - buf.center.as_f64(),
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: None,
            squelch_db: None,
            agc: true,
            voice,
            tx: None,
        }];
        let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");
        assert!(
            !rx.topology().nodes.iter().any(|n| n.kind == "packet_bus"),
            "an analogue channel put something on the packet bus"
        );

        let mut calls = crate::calls::Calls::new();
        let mut heard: Vec<crate::mix::heard::LiveCall> = Vec::new();
        let mut pcm: Vec<f32> = Vec::new();
        let mut silent_blocks = 0;
        for block in buf.samples.chunks(16_384) {
            if rx.process(block).is_err() {
                break;
            }
            let out = rx.audio_out().0;
            if out.is_empty() {
                silent_blocks += 1;
            }
            pcm.extend(out.iter().step_by(2));
            assert!(rx.decodes(std::time::Instant::now()).is_empty(), "speech is not a packet");
            for c in rx.heard_mut().expect("the tap").take_calls() {
                calls.hear(&c);
                heard.push(c);
            }
        }

        let rms = (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len() as f32).sqrt();
        (heard, calls, silent_blocks, rms)
    }

    /// A channel marked as voice is heard through its own fader, and is a row
    /// on the call list without anything of it reaching the packet bus.
    ///
    /// An analogue over used to be wrapped in an empty packet so the call
    /// list, which read only the packet bus, would see it: that put a row
    /// saying nothing into the packet log for every transmission, made the
    /// channel inaudible until something subscribed to it, and was wrong in
    /// principle, since there is no packet in analogue speech. The tap is
    /// what reads it, and the voice mark is the operator saying people talk
    /// here, which is the same statement a decoder makes with `Airtime::voice`
    /// and the reason the agent's own channel is listed.
    #[test]
    fn a_voice_channel_is_heard_and_is_a_call() {
        let Some(buf) = pmr446_fixture() else {
            eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        let (heard, calls, silent_blocks, rms) = pmr446_strip(&buf, true);

        // Heard, with no subscription to anything: the fader is the strip's.
        assert_eq!(
            silent_blocks, 0,
            "the bus handed the speaker nothing on {silent_blocks} blocks"
        );
        assert!(rms > 0.01, "the channel is silent at the speaker: {rms:e} rms");

        assert!(!heard.is_empty(), "a channel marked as voice made no call");
        assert!(heard.iter().all(|c| c.to == "PMR1"), "a call not named for the strip");
        let now = std::time::Instant::now();
        let active = calls.active(now);
        assert_eq!(active.len(), 1, "one channel, one row: {active:?}");
        assert_eq!(active[0].to, "PMR1");
        assert_eq!(active[0].system, crate::mix::fader::ANALOGUE);
        assert_eq!(active[0].channel_hz, 446_049_100.0);
        assert!(active[0].seconds > 1.0, "airtime of {}s", active[0].seconds);
        // The coded squelch this handheld is set to, read off the audio: an
        // FM carrier says nothing about who is on it, and for analogue
        // traffic the tone is the only group there is.
        assert_eq!(active[0].code.as_deref(), Some("141.3"), "the tone was not read");
        assert!(
            heard.iter().any(|c| c.code.as_deref() == Some("141.3")),
            "the tone never reached the tap's call"
        );
    }

    /// The same channel with the mark off: audible on the strip, written down
    /// by the transcriber, and no row. A mode and a frequency do not say
    /// whether what is coming out is a conversation, a repeater idling or an
    /// airband loop, so nothing here guesses.
    #[test]
    fn a_channel_not_marked_as_voice_is_heard_and_is_not_a_call() {
        let Some(buf) = pmr446_fixture() else {
            eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        let (heard, calls, silent_blocks, rms) = pmr446_strip(&buf, false);
        assert_eq!(silent_blocks, 0, "the bus handed the speaker nothing");
        assert!(rms > 0.01, "the channel is silent at the speaker: {rms:e} rms");
        assert!(heard.is_empty(), "an unmarked channel became a call: {heard:?}");
        assert!(calls.active(std::time::Instant::now()).is_empty());
    }

    /// The same handheld, kept as audio: the whole path from IQ to a record
    /// on the disk, with the file read back and decoded.
    ///
    /// What it proves that the node's own tests cannot: the tap really
    /// carries a tuned analogue channel's audio, at the rate the strip runs
    /// at, labelled with the channel it was heard on, and the recorder is in
    /// the graph the receiver draws rather than only in a test's patch.
    #[test]
    fn a_handheld_on_pmr446_is_recorded_and_reads_back() {
        let Some(buf) = pmr446_fixture() else {
            eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
            return;
        };
        const CHANNEL_HZ: f64 = 446_049_100.0;
        let dir = std::env::temp_dir().join(format!("sr-calls-replay-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut plan = replay_plan(&buf, false);
        plan.fronts.clear();
        plan.channels = vec![ChannelSpec {
            id: 1,
            label: "PMR1".into(),
            offset_hz: CHANNEL_HZ - buf.center.as_f64(),
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: None,
            squelch_db: Some(-200.0),
            agc: true,
            voice: true,
            tx: None,
        }];
        let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");
        let id = rx.node_of_stage(crate::chain::derived::CALL_LOG).expect("a call log");
        rx.set_node_param(id.0, "dir", pipeline::ParamValue::Text(dir.display().to_string()))
            .expect("the folder");
        rx.set_node_param(id.0, "enabled", pipeline::ParamValue::Bool(true)).expect("the switch");
        let _ = replay_blocks(&mut rx, &buf);
        // The over ends with the samples, so the hang has to run out before
        // the record is written: that is what a real channel going quiet
        // does.
        let silence = vec![C32::default(); 16_384];
        for _ in 0..80 {
            let _ = rx.process(&silence);
        }
        let recorded = rx.recorder().expect("the recorder reports");
        assert_eq!(recorded.calls, 1, "one transmission is one record");
        drop(rx);

        let files: Vec<std::path::PathBuf> =
            std::fs::read_dir(&dir).expect("the folder").flatten().map(|e| e.path()).collect();
        assert_eq!(files.len(), 1, "one segment, got {files:?}");
        let calls = crate::calllog::read(&files[0]).expect("the log reads back");
        assert_eq!(calls.len(), 1);
        let c = &calls[0];
        assert_eq!(c.channel_hz, CHANNEL_HZ as u64, "recorded on {} Hz", c.channel_hz);
        assert_eq!(c.system, crate::mix::fader::ANALOGUE);
        // The capture is six seconds with the handheld keyed for about five
        // of them, and the recording holds the speech rather than the file.
        assert!(
            (3.5..=6.0).contains(&c.seconds()),
            "a five second over came back as {:.2} s",
            c.seconds()
        );
        let speech = c.speech().expect("the audio decodes");
        assert_eq!(speech.rate, crate::calllog::RATE);
        let rms = (speech.pcm.iter().map(|v| v * v).sum::<f32>() / speech.pcm.len() as f32).sqrt();
        assert!((0.03..0.2).contains(&rms), "the recording reads {rms:.4} rms");
        // The tap carries this channel at seventeen times full scale, so
        // without the limiter every sample would be a square wave. A handful
        // of samples on the codec's ringing is not clipping.
        let clipped = speech.pcm.iter().filter(|s| s.abs() > 0.99).count();
        assert!(clipped < 50, "{clipped} of {} samples are clipped", speech.pcm.len());
        assert!(c.peak > 10.0, "the tap's level is not being reported: {}", c.peak);
        // 16 kbit/s and nothing between overs: a six second over is 12 kB.
        let bytes = std::fs::metadata(&files[0]).expect("the segment").len();
        assert!((11_000..14_000).contains(&bytes), "six seconds of speech cost {bytes} bytes");
        let _ = std::fs::remove_dir_all(&dir);
    }

    fn pmr446_fixture() -> Option<common::IqBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../testdata/pmr446_test_446.0M_512k.cs8");
        if !p.exists() {
            return None;
        }
        sources::FileSource::open(&p).ok()?.read_all().ok()
    }
}

#[cfg(test)]
mod squelch_probe {
    use super::*;

    /// An FM carrier modulated by a tone, plus noise, at a given SNR.
    fn fm_plus_noise(rate: f64, offset: f64, snr_db: f32, start: usize, n: usize) -> Vec<C32> {
        let dev = 2_500.0;
        let tone = 1_000.0;
        let a = 10f32.powf(snr_db / 20.0);
        let mut x = 12345u32.wrapping_add(start as u32) | 1;
        (start..start + n)
            .map(|i| {
                let t = i as f64 / rate;
                let ph = std::f64::consts::TAU * offset * t
                    + (dev / tone) * (std::f64::consts::TAU * tone * t).sin();
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                let nr = x as f32 / u32::MAX as f32 - 0.5;
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                let ni = x as f32 / u32::MAX as f32 - 0.5;
                C32::new(a * ph.cos() as f32 + nr, a * ph.sin() as f32 + ni)
            })
            .collect()
    }

    #[test]
    fn the_squelch_threshold_sits_between_noise_and_a_signal() {
        // The whole calibration in one test. If a change to the audio chain
        // moves either reading, the default stops being in the gap and this
        // fails rather than the squelch quietly passing hiss.
        let rate = 2_304_000.0;
        let offset = 120_000.0;
        const N: usize = 262_144;
        let read = |snr: f32| {
            let mut a = Audio::new(offset, rate, Demod::Nfm, 48_000.0);
            let mut last = 0.0;
            for k in 0..6 {
                a.process(&fm_plus_noise(rate, offset, snr, k * N, N), 1.0);
                last = a.squelch_db();
            }
            last
        };
        // -200 dB of signal is noise alone; 0 dB is a signal no stronger than
        // the noise it arrives with, which FM still captures.
        let (noise, signal) = (read(-200.0), read(0.0));
        let default = Demod::Nfm.default_squelch_db().unwrap();
        assert!(
            noise + 4.0 < default && default < signal - 4.0,
            "noise reads {noise:.1} dB and a signal {signal:.1} dB, \
             which leaves no room for a threshold at {default:.1} dB"
        );
    }
}

#[cfg(test)]
mod zoom_tests {
    use super::*;

    /// A tone that is a fraction of a channel wide on the full span.
    fn tone(rate: f64, hz: f64, n: usize) -> Vec<C32> {
        (0..n)
            .map(|i| {
                let p = std::f64::consts::TAU * hz * i as f64 / rate;
                C32::new(0.5 * p.cos() as f32, 0.5 * p.sin() as f32)
            })
            .collect()
    }

    #[test]
    fn narrowing_the_span_puts_more_bins_across_a_channel() {
        // The reason this exists: on a HackRF's narrowest span a 12.5 kHz
        // channel is a fraction of a pixel, and no cursor can be placed on it.
        let native = 2_000_000.0;
        let fft = 2048;
        for zoom in [1usize, 32] {
            let rate = native / zoom as f64;
            let bins_per_channel = 12_500.0 / (rate / fft as f64);
            if zoom == 1 {
                assert!(bins_per_channel < 15.0, "{bins_per_channel:.1} bins already");
            } else {
                assert!(
                    bins_per_channel > 400.0,
                    "only {bins_per_channel:.0} bins across a channel at /{zoom}"
                );
            }
        }
    }

    /// The receiver zoomed in, with nothing else attached, so what is being
    /// measured is the narrowing and not the rest of the chain.
    fn zoomed(native: f64, zoom: usize) -> crate::chain::Receiver {
        let mut plan = plan_at(native, Hz::mhz(433));
        plan.zoom = zoom;
        plan.fronts.clear();
        crate::chain::Receiver::build(&plan, Default::default()).expect("a zoom chain")
    }

    /// How long one chain takes to eat a second of signal.
    fn seconds_to_process(rx: &mut crate::chain::Receiver, sig: &[C32]) -> f64 {
        rx.process(&sig[..1024]).unwrap();
        let t = std::time::Instant::now();
        rx.process(sig).unwrap();
        t.elapsed().as_secs_f64()
    }

    // Timing, so it needs optimisation to mean anything.
    #[test]
    #[cfg_attr(debug_assertions, ignore = "timing test, run with --release")]
    fn narrowing_costs_little_enough_to_run_alongside_everything_else() {
        // It runs at the head of the graph, ahead of the spectrum, the banks
        // and the audio, so anything near real time here stalls all three.
        //
        // Measured against the same span with no narrowing in it rather than
        // against the clock. An absolute figure says as much about the machine
        // as about the code: this desktop runs it at ten times real time and a
        // shared CI runner managed two, while running the other two hundred
        // tests on the same cores, so a threshold in real time has to be set
        // so low it would miss a regression. A ratio survives that, because
        // whatever slows one side slows the other.
        //
        // Narrowing is not free and is not meant to be: the decimator costs
        // its tap count per output sample, where the chain it is compared
        // against is one FFT. That comes out at 6.6 to 6.8 across every zoom
        // factor here, so the bar is twelve.
        let native = 2_400_000.0;
        let sig = tone(native, 30_000.0, 2_400_000);
        let flat = seconds_to_process(&mut zoomed(native, 1), &sig);
        for zoom in [2usize, 8, 32] {
            let took = seconds_to_process(&mut zoomed(native, zoom), &sig);
            let ratio = took / flat.max(1e-9);
            eprintln!(
                "zoom /{zoom}: {:.0}x real time, {ratio:.2} times the unzoomed chain",
                1.0 / took
            );
            assert!(
                ratio < 12.0,
                "narrowing by {zoom} costs {ratio:.1} times the chain without it"
            );
            // And a floor against the catastrophic case, loose enough that no
            // runner can trip it on contention alone.
            assert!(took < 1.0, "narrowing by {zoom} took {took:.2} s for one second of signal");
        }
    }

    #[test]
    fn what_survives_the_narrowing_is_what_was_inside_it() {
        // Decimating without filtering folds the rest of the span on top of
        // what is left, and a folded signal cannot be told from a real one.
        let native = 2_000_000.0;
        let zoom = 8;
        let keep = native / zoom as f64 / 2.0;
        let mut plan = plan_at(native, Hz::mhz(433));
        plan.zoom = zoom;
        plan.fronts.clear();
        plan.channels = vec![ChannelSpec {
            id: 1,
            label: String::new(),
            offset_hz: 0.0,
            mode: ChanMode::Audio(Demod::Nfm),
            bandwidth_hz: None,
            squelch_db: Some(-200.0),
            agc: false,
            voice: false,
            tx: None,
        }];
        let mut rx =
            crate::chain::Receiver::build(&plan, Default::default()).expect("a zoom chain");
        // A signal well outside the narrowed span, which must not appear.
        rx.process(&tone(native, keep * 4.0, 262_144)).unwrap();
        let out = rx.zoomed_samples();
        let tail = &out[out.len() / 2..];
        let leaked = tail.iter().map(|c| c.norm()).fold(0.0f32, f32::max);
        let db = 20.0 * leaked.max(1e-12).log10();
        assert!(db < -60.0, "a signal outside the span folded in at {db:.1} dBFS");
    }

    /// Every capture in the corpus, through the whole receiver, at twice the
    /// speed it was recorded at, on four threads.
    ///
    /// The dashboard's speed trace is this number live, and a block that
    /// takes longer than the samples in it is a block the radio drops. So the
    /// rule is per block and not a mean: one slow block in a hundred is a lag
    /// spike, and a mean of 20x hides it. The first blocks of a capture
    /// allocate, fault pages in and open whatever the detector finds, so they
    /// are run and not judged, and a short capture is repeated until enough
    /// blocks have been timed to say anything.
    ///
    /// Twice, because the machine this is measured on is not the machine it
    /// runs on: a laptop's core is about half as fast, and 1x here is a
    /// receiver that drops samples there. Four threads for the same reason,
    /// and because measured on 48 the pool made nothing faster: the work in
    /// a block is serial, so what a laptop lacks in cores it does not miss.
    ///
    /// Blocks are the size a HackRF delivers, which is the worst case: a
    /// bigger block is more work between two reads of the clock.
    #[test]
    #[cfg_attr(debug_assertions, ignore = "timing test, run with --release")]
    fn every_capture_runs_faster_than_real_time() {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(4).build().expect("a pool");
        pool.install(every_capture_runs_at_twice_real_time);
    }

    fn every_capture_runs_at_twice_real_time() {
        const BLOCK: usize = 131_072;
        const WARM: usize = 4;
        const TIMED: usize = 32;
        /// Blocks slower than this, in multiples of real time, fail.
        const FLOOR_X: f64 = 2.0;
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata");
        let mut files: Vec<std::path::PathBuf> = ["", "offair", "rtl433"]
            .iter()
            .filter_map(|d| std::fs::read_dir(root.join(d)).ok())
            .flatten()
            .flatten()
            .map(|e| e.path())
            .filter(|p| {
                matches!(
                    p.extension().and_then(|e| e.to_str()),
                    Some("cu8" | "cs8" | "cs16" | "cf32")
                )
            })
            .collect();
        files.sort();
        if files.is_empty() {
            eprintln!("skipping: no captures in testdata, run testdata/fetch.sh");
            return;
        }
        // Captures known not to keep up, each with the reason, so the test
        // stays a gate for everything else while the reason is worked on.
        // One that starts keeping up fails the test until it is taken off
        // the list, so the list cannot outlive its reasons.
        //
        // Every reason here was measured with `--bench-iq`, which prints the
        // auto node's phases; none of them is the span-wide burst router any
        // more, which is what they all used to say.
        //
        // What they mostly say now is the classifier. It costs about 1.3 ms
        // of one core per burst, measured on a 30000 sample burst at
        // 2.5 MS/s, and that is spread over a level histogram, four
        // Welch-averaged transforms and two autocorrelations, none of which
        // is more than a quarter of it; halving the window it measures takes
        // the corpus from 46 of 52 captures named right to 42, so the cost is
        // the measurement rather than an overhead around it. A band where
        // several megahertz-wide sources burst at once therefore asks for
        // more classification than a block has time for, and that is a
        // throughput problem rather than a spike.
        const KNOWN_SLOW: &[(&str, &str)] = &[
            (
                "ism24_busy_2431M_61440k.cs16",
                "a whole 2.4 GHz band at 61.44 MS/s, which is five things at once, measured on \
                 four threads as processor time in a 2.13 ms block: the Wi-Fi front end on the \
                 channels it is on, 2.3 ms; the BLE front end on two advertising channels, \
                 1.0; the detector over 61 MS/s, 1.2, of which half is eight 32768 point \
                 transforms and a third the floor pass; cutting the fifty-three sources the \
                 band opens out of the span, 0.9; and the DroneID correlator on the three \
                 centres the span holds, 0.6. Six milliseconds of processor time for a two \
                 millisecond block, and the phases are sequential, so four threads return 1.4 \
                 rather than 4. Nothing here is a spike any more: the front ends were 4.2 ms \
                 and the worst block 127",
            ),
            (
                "droneid_mini4k_2444.5M_15360k.cs8",
                "the DroneID correlator over the 10 MHz centre the aircraft is on, 1.8 ms of an \
                 8.5 ms block, and the classifier behind the source the burst opens, 1.9. The \
                 correlator runs whenever anything is transmitting in that channel, which on \
                 this capture is the aircraft's own link in nearly every block; what it buys is \
                 the seven bursts, which the receiver did not read at all while the front end \
                 was placed on a source instead of on the span",
            ),
            (
                "odid_bt5lr_holybro_2474M_20000k.cs8",
                "classifying the coded-PHY bursts of several megahertz-wide sources at once, \
                 5 ms of a 6.5 ms block on average and 14 in the worst. They are auxiliary \
                 advertising on data channels, which the BLE front end does not read, so no \
                 front end has already decoded what is being measured",
            ),
            (
                "odid_holybro_2431M_20000k.cs8",
                "the same, with the chirp decoders behind it: the classifier names an 855 kHz \
                 Bluetooth burst a chirp at full confidence, and LoRa and ExpressLRS are each \
                 placed on that for 2.9 ms a block",
            ),
            (
                "offair/elrs_100hz_2415M_20000k.cs8",
                "classifying a handset's channel visit, 44 ms and 1.4 MHz wide, which is one \
                 burst that costs three blocks, beside the chirp decoder it places",
            ),
            (
                "offair/ofdm_wifi_2462M_20000k.cs8",
                "the Wi-Fi front end reading the channel it is on, 4 ms of a 6.5 ms block, \
                 beside 1.8 ms cutting the sources the same signal opens out of the span",
            ),
            (
                "offair/ofdm_wifi_frames_2462M_20000k.cs8",
                "the same, and DroneID correlating over the 2459.5 MHz centre the span holds, \
                 which the Wi-Fi traffic lights in every block",
            ),
            (
                "offair/gfsk_ble_2426M_20000k.cs8",
                "the closest of them: about 2.7x on four threads with the BLE front end and \
                 the detector at 1.0 and 0.9 ms each of 6.5, and one block in ten at half that \
                 speed. The span holds a DroneID centre too, and the advertising channel sits \
                 inside it, so every advertisement lights that correlator as well",
            ),
            (
                "pal_camera_5865M_20000k.cs8",
                "the video front end itself: 3.8 ms of every 6.5 ms block, which is two FM \
                 discriminators, one at 10 MS/s for the picture and one at 20 for the sound \
                 subcarrier, and the field assembly between them",
            ),
        ];
        let mut slow: Vec<String> = Vec::new();
        let mut recovered: Vec<String> = Vec::new();
        for path in &files {
            let name = path.strip_prefix(&root).unwrap_or(path).display().to_string();
            let known = KNOWN_SLOW.iter().find(|(n, _)| *n == name).map(|(_, why)| *why);
            let buf = match sources::FileSource::open(path).and_then(|s| s.read_all()) {
                Ok(b) if b.samples.len() >= BLOCK => b,
                Ok(_) => {
                    eprintln!("{name}: shorter than one block, not timed");
                    continue;
                }
                Err(e) => panic!("{name}: {e}"),
            };
            let rate = buf.rate.as_f64().max(1.0);
            let block_secs = BLOCK as f64 / rate;
            let mut rx = replay_receiver(&buf, None).expect(&name);
            let mut us: Vec<f64> = Vec::new();
            'passes: for _ in 0..64 {
                for chunk in buf.samples.chunks(BLOCK) {
                    if chunk.len() < BLOCK {
                        break;
                    }
                    let t = std::time::Instant::now();
                    rx.process(chunk).expect(&name);
                    us.push(t.elapsed().as_secs_f64() * 1e6);
                    let at = block_start(std::time::Instant::now(), chunk.len(), rate);
                    let _ = harvest(&mut rx, at);
                    if us.len() >= WARM + TIMED {
                        break 'passes;
                    }
                }
            }
            let timed = &us[WARM.min(us.len().saturating_sub(1))..];
            let worst = timed.iter().copied().fold(0.0f64, f64::max);
            let mut sorted = timed.to_vec();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let median = sorted[sorted.len() / 2];
            let x = |us: f64| block_secs * 1e6 / us.max(1e-9);
            let over = timed.iter().filter(|&&b| b * FLOOR_X > block_secs * 1e6).count();
            eprintln!(
                "{name}: {} blocks, median {:.1}x, worst {:.2}x{}",
                timed.len(),
                x(median),
                x(worst),
                if over > 0 { format!(", {over} under {FLOOR_X}x") } else { String::new() },
            );
            // A listed capture comes off the list once it clears the floor
            // with room to spare, not the first time it lands over it: one
            // that sits at the floor would otherwise fail one run in two.
            let clear = x(worst) >= FLOOR_X * 1.5;
            match (over > 0, known) {
                (true, None) => {
                    slow.push(format!("{name}: worst block {:.2}x, floor {FLOOR_X}x", x(worst)))
                }
                (true, Some(why)) => eprintln!("{name}: known slow, {why}"),
                (false, Some(_)) if clear => recovered.push(name.clone()),
                (false, _) => {}
            }
        }
        assert!(slow.is_empty(), "captures the receiver cannot keep up with:\n{}", slow.join("\n"));
        assert!(
            recovered.is_empty(),
            "captures that keep up now and should come off KNOWN_SLOW:\n{}",
            recovered.join("\n")
        );
    }
}

#[cfg(test)]
mod front_end_tests {
    use super::*;
    use common::{Device, DeviceInfo, DriverKind, GainStage, Toggle, TunerRange};

    /// A radio with three stages that quantise, a switch, and a "tuner"
    /// handle that distributes a total across the stages. All three
    /// behaviours a reopen has to survive, and all three a HackRF has.
    struct ThreeStages {
        info: DeviceInfo,
        tuning: common::Tuning,
        amp: bool,
        lna: u32,
        vga: u32,
        bias_tee: bool,
    }

    impl ThreeStages {
        fn new() -> Self {
            let stage = |name: &str, hi: f32, step: f32| GainStage {
                name: name.into(),
                label: name.into(),
                range: 0.0..=hi,
                values: Vec::new(),
                step,
                auto: false,
            };
            Self {
                info: DeviceInfo {
                    kind: DriverKind::HackRf,
                    id: "stub".into(),
                    label: "Stub".into(),
                    tuner: "none".into(),
                    ranges: vec![TunerRange {
                        label: "rx",
                        range: Hz(1_000_000)..=Hz(6_000_000_000),
                    }],
                    rates: Vec::new(),
                    rate_range: Sps(2_000_000)..=Sps(20_000_000),
                    gain_stages: vec![
                        stage("amp", 14.0, 14.0),
                        stage("lna", 40.0, 8.0),
                        stage("vga", 62.0, 2.0),
                    ],
                    native_format: common::SampleFormat::Cs8,
                    usable_bandwidth_ratio: 0.75,
                    tx: None,
                },
                tuning: common::Tuning::default(),
                amp: false,
                lna: 0,
                vga: 0,
                bias_tee: false,
            }
        }
    }

    impl common::Device for ThreeStages {
        fn info(&self) -> &DeviceInfo {
            &self.info
        }
        fn set_center(&mut self, _f: Hz) -> common::Result<()> {
            Ok(())
        }
        fn center(&self) -> Hz {
            Hz(100_000_000)
        }
        fn tuning(&self) -> &common::Tuning {
            &self.tuning
        }
        fn tuning_mut(&mut self) -> &mut common::Tuning {
            &mut self.tuning
        }
        fn set_rate(&mut self, _r: Sps) -> common::Result<()> {
            Ok(())
        }
        fn rate(&self) -> Sps {
            Sps(2_000_000)
        }
        fn set_gain(&mut self, stage: &str, mode: GainMode) -> common::Result<()> {
            let db = match mode {
                GainMode::Auto => 32.0,
                GainMode::Manual(db) => db,
            };
            match stage {
                "tuner" => {
                    self.amp = db > 102.0;
                    let rest = (db - if self.amp { 14.0 } else { 0.0 }).max(0.0);
                    self.lna = ((rest / 2.0) as u32 / 8 * 8).min(40);
                    self.vga = ((rest - self.lna as f32) as u32 / 2 * 2).min(62);
                }
                "amp" => self.amp = db >= 7.0,
                "lna" => self.lna = (db as u32 / 8 * 8).min(40),
                "vga" => self.vga = (db as u32 / 2 * 2).min(62),
                _ => return Err(common::Error::other("no such stage")),
            }
            Ok(())
        }
        fn gains(&self) -> Vec<(String, GainMode)> {
            vec![
                ("amp".into(), GainMode::Manual(if self.amp { 14.0 } else { 0.0 })),
                ("lna".into(), GainMode::Manual(self.lna as f32)),
                ("vga".into(), GainMode::Manual(self.vga as f32)),
            ]
        }
        fn toggles(&self) -> Vec<Toggle> {
            vec![Toggle {
                name: "bias_tee".into(),
                label: "Bias tee".into(),
                help: String::new(),
                on: self.bias_tee,
            }]
        }
        fn set_toggle(&mut self, name: &str, on: bool) -> common::Result<()> {
            match name {
                "bias_tee" => self.bias_tee = on,
                _ => return Err(common::Error::other("no such switch")),
            }
            Ok(())
        }
        fn start_rx(&mut self) -> common::Result<Box<dyn common::RxStream>> {
            Err(common::Error::other("not a real radio"))
        }
    }

    /// Reopening for a span change puts every stage back, not the total: the
    /// HackRF came back with the VGA at zero because only "tuner" was
    /// remembered, and a driver that distributes a total does not land on
    /// what the operator set stage by stage.
    #[test]
    fn a_reopen_restores_every_stage_and_switch() {
        let mut was = ThreeStages::new();
        was.set_gain("lna", GainMode::Manual(24.0)).unwrap();
        was.set_gain("vga", GainMode::Manual(45.0)).unwrap();
        was.set_gain("amp", GainMode::Manual(14.0)).unwrap();
        was.set_toggle("bias_tee", true).unwrap();
        // What the hardware landed on, which is not quite what was asked for.
        assert_eq!(
            was.gains(),
            vec![
                ("amp".to_string(), GainMode::Manual(14.0)),
                ("lna".to_string(), GainMode::Manual(24.0)),
                ("vga".to_string(), GainMode::Manual(44.0)),
            ]
        );

        let front = FrontEnd::read(&was);
        let mut back = ThreeStages::new();
        assert_eq!(back.gains()[2].1, GainMode::Manual(0.0), "a fresh device is at its defaults");
        front.apply(&mut back);

        assert_eq!(back.gains(), was.gains());
        assert!(back.bias_tee, "the bias tee is a front end setting and goes back too");
    }
}
