use super::*;

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

    pub fn passband(&self, width_hz: Option<f64>, low_hz: Option<f64>) -> common::Passband {
        match self {
            ChanMode::Audio(d) => d.passband(width_hz, low_hz),
            ChanMode::Decode(_) | ChanMode::Auto => common::Passband::around(
                width_hz
                    .filter(|w| *w >= common::demod::NARROWEST_HZ)
                    .unwrap_or_else(|| self.bandwidth()),
            ),
        }
    }

    pub fn if_reach(&self, p: common::Passband) -> f64 {
        match self {
            ChanMode::Audio(d) => d.if_reach(p),
            ChanMode::Decode(_) | ChanMode::Auto => p.reach(),
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

/// The audio mode a channel is listened to in, with everything that follows
/// from it. In `common` because the allocation table names one per band.
pub use common::Demod;

/// Which sideband a mode listens to, as the demodulator names it.
pub(crate) fn sideband(d: Demod) -> dsp::ssb::Sideband {
    if d.is_lower() { dsp::ssb::Sideband::Lower } else { dsp::ssb::Sideband::Upper }
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
    /// Set one of the radio's plain numbers, such as a per-tuner trim.
    Number(String, f64),
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
    /// What starts a capture file: the switch, or energy in the span with
    /// its threshold, pre-roll and hang.
    CaptureTrigger(crate::chain::CapturePlan),
    /// What the trace and the waterfall each take out of a spectrum frame:
    /// the newest transform, the mean of the frame, or the loudest each bin
    /// reached in it.
    Detectors {
        trace: dsp::spectrum::Detector,
        waterfall: dsp::spectrum::Detector,
    },
    /// What the heatmap recorder keeps: whether it is running, how often it
    /// takes a row and how much it may hold.
    Heatmap(crate::chain::HeatPlan),
    /// Write what the heatmap holds as a file, coloured and scaled as the
    /// interface is showing it. Done on this thread because the readings are
    /// a node's, and the node belongs to the graph running here.
    ExportHeatmap {
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
    /// Where the span is served to network subscribers, or `None` to stop
    /// serving it. The listening socket outlives the graph, so turning it off
    /// only takes the stage out; the port is given up when the process ends.
    IqStream(Option<crate::chain::IqStreamPlan>),
    /// The radios served beside the span, as the complete set for the same
    /// reason the feeds are one.
    IqStreamTuners(Vec<crate::chain::TunerServePlan>),
    /// Where to serve a KISS TNC, or `None` to serve none.
    Kiss(Option<std::net::SocketAddr>),
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
    /// The capture a channel on [`TxSource::Capture`] replays, or `None` to
    /// send nothing. A plan value rather than a sink, unlike the `.sub` file:
    /// a path is something a stage's settings can carry, so the chain view
    /// says which file is loaded and a rebuild does not lose it.
    TxCapture(Option<TxCapture>),
    /// The station a WFM channel identifies itself as, or `None` for a
    /// carrier with no data on it.
    Rds(Option<RdsStation>),
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
    pub audio_low_hz: Option<f64>,
    /// None leaves the mode's own default.
    pub squelch_db: Option<f32>,
    pub agc: bool,
    pub blanker: Option<f32>,
    pub denoise: bool,
    pub denoise_db: f32,
    /// Treat what is heard here as speech: end an over on the squelch, put
    /// the transmission on the packet bus with its audio, and list it as a
    /// call.
    ///
    /// A property of the channel rather than of the mode, because the mode
    /// cannot know: NFM carries a repeater, a telemetry link and a paging
    /// tone alike, and only whoever tuned it knows which. What it costs when
    /// it is wrong is rows in the call list for something nobody said.
    pub voice: bool,
    /// A protocol read off this channel's audio, beside playing it, or `None`
    /// for a channel that is only listened to.
    ///
    /// An alert relayed on a broadcast channel, a picture on the calling
    /// frequency and a weather chart are all in the audio of a channel
    /// somebody has already tuned, so the samples have been mixed down and
    /// discriminated once already. Reading one off that audio costs a stage;
    /// the other way costs a second front end cutting the same channel out of
    /// the span again, placed by hand. Which protocols can be asked for is
    /// [`nodes::protocol::audio_readers`], not a list here.
    pub reads: Option<String>,
    /// What this channel does when it is keyed, or `None` for a channel that
    /// only listens, which is every channel until somebody says otherwise.
    ///
    /// Part of the channel rather than a list of its own because a repeater
    /// channel is one channel: it listens on the output and transmits on the
    /// input, and two entries kept in step by hand is how an operator ends up
    /// transmitting on the wrong half of the pair.
    pub tx: Option<TxSpec>,
    /// The coded squelch this channel opens on, or `None` to hear whoever
    /// is there.
    ///
    /// A tone rather than a level: two groups share the frequency, both are
    /// loud, and only one of them is this channel's. Programmed rather than
    /// set against the day's signal, so it comes off the memory bank with
    /// the channel and goes back into it.
    pub tone: Option<dsp::squelch::Coded>,
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

    pub fn passband(&self) -> common::Passband {
        self.mode.passband(self.bandwidth_hz, self.audio_low_hz)
    }

    /// The width this channel is really built at.
    pub fn bandwidth(&self) -> f64 {
        self.passband().width()
    }

    pub fn if_reach(&self) -> f64 {
        self.mode.if_reach(self.passband())
    }

    /// The least span this channel can be built in, at its own width.
    pub fn min_rate(&self) -> f64 {
        self.mode.min_rate_for(2.0 * self.if_reach())
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
