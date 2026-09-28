use super::*;

pub(super) const TRANSMITTER_OFF: &str =
    "cannot transmit: a stage of the transmit chain is switched off";

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
pub(super) fn tx_plan_for(ch: &ChannelSpec, center: Hz) -> Option<crate::chain::TxPlan> {
    let tx = ch.spec_to_transmit();
    let mode = tx_mode_for(&ch.mode, tx.source)?;
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
pub(super) fn derive_tx(
    plan: &Plan,
    can_transmit: bool,
    on_air: Option<u64>,
) -> Option<crate::chain::TxPlan> {
    if !can_transmit {
        return None;
    }
    let keyed = on_air
        .and_then(|id| plan.channels.iter().find(|c| c.id == id))
        .and_then(|c| tx_plan_for(c, plan.center));
    keyed.or_else(|| plan.channels.iter().find_map(|c| tx_plan_for(c, plan.center)))
}

pub(super) fn key_up(
    dev: &mut dyn common::Device,
    ch: &ChannelSpec,
    tx: &TxSpec,
    center: Hz,
    gain_db: f32,
    mic: &Option<Arc<dyn audio::AudioSource>>,
    voice: &Option<std::sync::Arc<dyn audio::AudioSource>>,
    sub: &Option<SubFile>,
) -> common::Result<(crate::chain::TxPlan, crate::chain::TxSinks)> {
    // Where the channel transmits: its own frequency plus the repeater
    // shift, which is zero for simplex.
    let on_air = Hz((center.as_f64() + ch.offset_hz + tx.shift_hz).max(0.0) as u64);
    // The channel's own mode, because a channel is one frequency and one
    // mode: a radio that listens in NFM and keys up in AM cannot be worked.
    let mode = tx_mode_for(&ch.mode, tx.source).ok_or_else(|| {
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
    let stages: Vec<(String, bool)> = dev
        .info()
        .tx
        .as_ref()
        .map(|t| t.gain_stages.iter().map(|s| (s.name.clone(), s.is_switch())).collect())
        .unwrap_or_default();
    for (name, switch) in stages {
        // The front end amp is a switch, not a level, and switching it in
        // because the gain was turned up is a 14 dB surprise.
        let db = if switch { 0.0 } else { want };
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
            Some(mic.clone().ok_or_else(|| common::Error::other("no microphone is open"))?)
        }
        TxSource::Agent => {
            Some(voice.clone().ok_or_else(|| common::Error::other("the agent has no voice"))?)
        }
        // The file is already on the node; there is nothing to hand in at
        // key-up.
        TxSource::Sub | TxSource::Tone | TxSource::Capture => None,
    };

    Ok((
        crate::chain::TxPlan { spec: *tx, mode, on_air },
        crate::chain::TxSinks { stream: Some(dev.start_tx()?), mic: src, sub: sub.clone() },
    ))
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
    /// Whether speech keys the channel, and how.
    pub vox: VoxSpec,
    /// A courtesy tone at the end of an over, or zero for none, and the
    /// pitch it is sent at. On a channel with no squelch tail the far end
    /// has nothing else to tell it the over finished, and the end of an over
    /// is the same end whether a voice, a hand or the agent let the key up.
    pub roger_style: nodes::RogerStyle,
    pub roger_ms: f64,
    pub roger_hz: f64,
    pub roger_lead_ms: f64,
    pub tone: Option<dsp::squelch::Coded>,
}

/// What lets a voice key the transmitter instead of a hand.
///
/// Only under [`TxSource::Mic`]: a tone has no pauses to key between, and
/// the agent already decides when it is talking.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VoxSpec {
    pub on: bool,
    /// The level a voice has to reach, as an amplitude in 0..1: the same
    /// scale the meter beside the control shows, because a threshold set
    /// against a different number than the one being compared is a threshold
    /// nobody trusts.
    pub threshold: f32,
    /// How long the key stays down after the voice drops under it, so a gap
    /// between two sentences is not two overs.
    pub tail_ms: f64,
    /// Whether what the speaker is playing raises the threshold. Off for a
    /// headset, where nothing coming out of it reaches the microphone.
    pub anti_trip: bool,
}

impl Default for VoxSpec {
    fn default() -> Self {
        Self {
            on: false,
            threshold: nodes::DEFAULT_VOX_THRESHOLD,
            tail_ms: nodes::DEFAULT_VOX_TAIL_MS,
            anti_trip: true,
        }
    }
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
            vox: VoxSpec::default(),
            roger_style: nodes::RogerStyle::Tone,
            roger_ms: 0.0,
            roger_hz: 1_000.0,
            roger_lead_ms: nodes::QUINDAR_LEAD_MS,
            tone: None,
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
    /// A recorded span, sent back out as it stands. Nothing is modulated:
    /// the file is already IQ, so the chain resamples it to the rate the
    /// radio is transmitting at and the mixer behind it carries whatever
    /// offset the operator wants it sent at. See `Cmd::TxCapture`.
    Capture,
}

impl TxSource {
    pub fn label(self) -> &'static str {
        match self {
            Self::Tone => "TONE",
            Self::Mic => "MIC",
            Self::Agent => "AGENT",
            Self::Sub => "SUB",
            Self::Capture => "IQ",
        }
    }
}

/// A capture chosen to be transmitted: where it is, and what its name says it
/// holds.
///
/// Resolved once in the interface, where `sources::parse_filename` already
/// lives, rather than in the stage that plays it: a rate guessed on the radio
/// thread puts a signal of the wrong width on the air and nothing downstream
/// can tell.
#[derive(Clone, Debug, PartialEq)]
pub struct TxCapture {
    pub path: std::path::PathBuf,
    pub rate: Sps,
    /// What it was recorded at, where the name says. The difference between
    /// this and where it is being sent is the operator's to set.
    pub center: Option<Hz>,
    /// How the samples are laid out, which the extension usually says and an
    /// operator says where it does not.
    pub format: common::SampleFormat,
    pub seconds: f64,
}

impl TxCapture {
    /// Take a capture as the operator describes it. `None` only when the path
    /// is not a file: nothing here guesses a rate, because a rate guessed
    /// wrong puts a signal of the wrong width on the air.
    pub fn new(
        path: &std::path::Path,
        rate: Sps,
        center: Option<Hz>,
        format: common::SampleFormat,
    ) -> Option<Self> {
        let len = std::fs::metadata(path).ok().filter(|m| m.is_file())?.len();
        if rate.0 == 0 {
            return None;
        }
        Some(Self {
            path: path.to_path_buf(),
            rate,
            center,
            format,
            seconds: (len / format.bytes_per_sample() as u64) as f64 / rate.as_f64(),
        })
    }

    /// Read what a capture's name says it holds, or nothing where it does not
    /// say enough to replay it.
    pub fn open(path: &std::path::Path) -> Option<Self> {
        let meta = sources::parse_filename(path);
        Self::new(path, meta.rate?, meta.center, meta.format?)
    }

    /// What the strip calls it, which is the file's own name.
    pub fn label(&self) -> String {
        self.path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| self.path.display().to_string())
    }
}

/// What a broadcast station says about itself on the 57 kHz subcarrier.
///
/// In the plan beside the capture a channel replays, and for the same
/// reason: all three fields are things a stage's settings can carry, so the
/// chain view says what is being transmitted and a rebuild keeps it. It
/// belongs to the receiver rather than to a channel because a transmitter is
/// one station however many channels are set to WFM.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RdsStation {
    /// The programme identification code, which is how a receiver tells two
    /// transmitters of the same programme apart.
    pub pi: u16,
    /// The eight characters a receiver shows as the station.
    pub name: String,
    /// The scrolling message under it, or empty for none.
    pub radiotext: String,
}

impl Default for RdsStation {
    fn default() -> Self {
        Self { pi: DEFAULT_PI, name: "WAVESHRK".into(), radiotext: String::new() }
    }
}

/// The country and programme this receiver identifies itself with until
/// somebody sets one: 0x5343 is what `rds_tx` is built with.
pub const DEFAULT_PI: u16 = 0x5343;

impl RdsStation {
    /// A PI code as it is published and as every receiver shows one: four
    /// hex digits. Nothing else is a code, so the field says so rather than
    /// transmitting whatever half of it parsed.
    pub fn parse_pi(text: &str) -> Option<u16> {
        let t = text.trim().trim_start_matches("0x").trim_start_matches("0X");
        (t.len() == 4).then(|| u16::from_str_radix(t, 16).ok()).flatten()
    }

    /// The eight characters that go out, which is what a receiver shows:
    /// longer is cut and shorter is padded, because the name is sent as four
    /// pairs whatever was typed.
    pub fn label(&self) -> String {
        let mut s: String = self.name.chars().take(8).collect();
        while s.chars().count() < 8 {
            s.push(' ');
        }
        s
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
    /// A recorded span played back as it stands. There is no modulator: the
    /// file is IQ already, and what stands in that stage's place is the
    /// mixer that moves it off the dial.
    Iq,
    /// A protocol with a transmit chain of its own, named by its id: the
    /// stages come off the registry rather than from the list here, because
    /// a data mode's source is not a microphone and its modulator is not one
    /// of these four.
    Digital(&'static str),
}

/// What a channel transmits, from what it receives and what it is sending.
///
/// `None` for a mode with no modulator behind it yet: single sideband needs
/// one, and a decode channel needs the protocol's encoder. Refusing is the
/// point, since the alternative is keying up in a mode the other end cannot
/// read.
///
/// A `.sub` file is the exception, and the source is a parameter for it: its
/// pulses are keyed carrier whatever the channel hears in, so a channel
/// replaying a remote can listen on the classifier and decode the remotes
/// around it rather than run an NFM demodulator it has no use for.
pub fn tx_mode_for(mode: &ChanMode, source: TxSource) -> Option<TxMode> {
    if source == TxSource::Sub {
        return Some(TxMode::Carrier);
    }
    // A recording is already what went on the air, so the channel's mode says
    // nothing about how to send it: whatever it was received in, it goes out
    // as the samples that were written down.
    if source == TxSource::Capture {
        return Some(TxMode::Iq);
    }
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
            Self::Iq => "IQ",
            Self::Digital(id) => nodes::protocol::all()
                .iter()
                .find(|p| p.id() == id)
                .map(|p| p.label())
                .unwrap_or(id),
        }
    }
}

/// The transmit side of the radio, between and during overs.
pub(super) struct Tx {
    /// The radio's own transmit gain.
    pub(super) gain_db: f32,
    /// A parsed `.sub` file, for a channel set to [`TxSource::Sub`]. Held
    /// beside the gain rather than in the plan because it is handed in
    /// whole, the way the agent's voice is, and a plan is copied around.
    pub(super) sub_file: Option<SubFile>,
    /// Blocks since the key went down, for the note said once a second.
    pub(super) blocks_since_key: u64,
    /// The channel whose key is down but whose transmitter is still being
    /// built, so the strip is not told it is on air before it is.
    pub(super) keying_for: Option<u64>,
    /// The last channel that went on air, which is the one the drawn chain
    /// stays on once the key comes up. Without it the chain view fell back
    /// to whichever transmit channel comes first on the strip, so recalling
    /// a second from a bank and working it left the chain showing the other.
    pub(super) last_keyed: Option<u64>,
    /// When the transmitter was last on air, so the transcriber stays deaf a
    /// moment past the key coming up: the audio already in the demodulator
    /// when the key lifted is still the receiver's own voice.
    pub(super) last_on_air: Option<std::time::Instant>,
    /// The channel a voice keyed, so the same voice stopping lets it up and
    /// a hand on the key is left alone.
    pub(super) vox_keyed: Option<u64>,
    /// When the over ended, while the courtesy tone that closes it is still
    /// going out. The radio goes back when the tone has, so a half duplex
    /// radio is not retuned out from under it.
    pub(super) ending: Option<std::time::Instant>,
    pub(super) back_to_receive: bool,
}

/// How long past the key coming up the transcriber stays deaf.
///
/// One demodulator's worth of audio in flight, not a hang time: what the
/// receiver said must not be written down, and what somebody says straight
/// after must be.
pub(super) const DEAF_TAIL: std::time::Duration = std::time::Duration::from_millis(500);

/// The longest an over is held open for its courtesy tone: the longest tone
/// that can be set ([`nodes::ROGER_MAX_MS`]) and a block or two for the
/// chain to have made it.
pub(super) const ROGER_LIMIT: std::time::Duration = std::time::Duration::from_millis(1_500);

impl<'a, R: Fn()> RadioThread<'a, R> {
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
    pub(super) fn open_mic(&mut self) {
        if self.audio.mic.is_none() && self.audio.given.is_none() && !cfg!(test) {
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
        self.rx.set_microphone(self.mic_tap());
    }

    pub(super) fn mic_tap(&self) -> Option<Arc<dyn audio::AudioSource>> {
        match &self.audio.given {
            Some(src) => Some(src.clone()),
            None => self.audio.mic.as_ref().map(|m| m.tap()),
        }
    }

    #[cfg(test)]
    pub(super) fn hand_microphone(&mut self, src: Arc<dyn audio::AudioSource>) {
        self.audio.given = Some(src);
        self.open_mic();
    }

    /// Take the radio back off the transmit stage.
    ///
    /// The over's end is announced to the transmit chain first, because that
    /// is the one place that knows whether anything is still to be sent: a
    /// channel with a courtesy tone keeps the key down for it, and the radio
    /// goes back on a later block when [`Self::finish_over`] sees it out.
    pub(super) fn unkey(&mut self) {
        // A key let up before the rebuild it was waiting for: the rebuild
        // must not go on to announce it on air.
        self.tx.keying_for = None;
        if self.tx.ending.is_some() {
            return;
        }
        if !self.rx.keyed() && self.status.keyed.load(Ordering::Relaxed) == 0 {
            return;
        }
        if self.rx.end_over() {
            self.tx.ending = Some(std::time::Instant::now());
            return;
        }
        self.unkey_now();
    }

    /// The tone that ends the over is out, or has had long enough: the radio
    /// goes back.
    ///
    /// The deadline is what covers a radio unplugged mid-tone. The chain is
    /// clocked by the device taking samples away, so a device that stopped
    /// taking them is a stage that never finishes sending and a key that
    /// never comes up.
    pub(super) fn finish_over(&mut self) {
        let Some(since) = self.tx.ending else { return };
        if self.rx.sending_roger() && since.elapsed() < ROGER_LIMIT {
            return;
        }
        self.tx.ending = None;
        self.unkey_now();
    }

    pub(super) fn unkey_now(&mut self) {
        self.tx.keying_for = None;
        self.tx.ending = None;
        if !self.rx.keyed() && self.status.keyed.load(Ordering::Relaxed) == 0 {
            return;
        }
        tracing::info!("unkeyed");
        // The stages stay; what goes is the radio. Dropping it drains the
        // queue before the carrier stops and, on a half duplex radio, hands
        // the receiver its radio back.
        self.status.tx_underruns.store(self.rx.unkey(), Ordering::Relaxed);
        self.status.keyed.store(0, Ordering::Relaxed);
        self.tx.back_to_receive = true;
        self.back_to_receive();
    }

    pub(super) fn back_to_receive(&mut self) {
        if !self.tx.back_to_receive || self.rx.tx_draining() {
            return;
        }
        self.tx.back_to_receive = false;
        // Back where the receiver was. A half duplex radio has one
        // synthesiser, so keying moved it to the transmit frequency; leaving
        // it there means the waterfall comes back tuned to wherever the
        // channel transmits, which looks like reception never resumed at all.
        if self.dev.dial() != self.plan.center
            && let Err(e) = self.dev.set_dial(self.plan.center)
        {
            *self.status.error.lock() = Some(format!("could not retune after transmitting: {e}"));
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
    pub(super) fn key(&mut self, id: u64) {
        // Keyed again while the last over's tone was going out: it is one
        // over now, and nothing is waiting to be given back.
        self.tx.ending = None;
        self.tx.back_to_receive = false;
        let Some(ch) = self.plan.channels.iter().find(|c| c.id == id).cloned() else {
            *self.status.error.lock() = Some("there is no such channel to key".into());
            return;
        };
        if self.rx.transmitter_off() {
            *self.status.error.lock() = Some(TRANSMITTER_OFF.into());
            return;
        }
        let tx = ch.spec_to_transmit();
        let mic = self.mic_tap();
        let up = key_up(
            self.dev.as_mut(),
            &ch,
            &tx,
            self.plan.center,
            self.tx.gain_db,
            &mic,
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

    /// What the microphone is hearing, whether or not anything is keyed.
    ///
    /// While an over is running the microphone stage has already taken those
    /// samples out of the ring, so the reading comes from the graph instead of
    /// from the capture.
    pub(super) fn meter_mic(&mut self) {
        let Some(c) = self.audio.mic.as_ref() else { return };
        let keyed_now = self.rx.tx_state();
        let peak = match keyed_now {
            Some(tx) if self.rx.keyed() => tx.mic_peak,
            _ => c.peak(),
        };
        // Said once a second while keyed, because a transmission that stops is
        // the hardest thing here to see after the fact: the carrier is gone
        // and nothing on screen says why.
        if let (Some(tx), 0) = (keyed_now, self.tx.blocks_since_key % 50)
            && self.rx.keyed()
        {
            tracing::info!(
                "on air: {} samples, {} unfilled, mic {peak:.2}",
                tx.written,
                tx.underruns
            );
            self.status.tx_underruns.store(tx.underruns, Ordering::Relaxed);
        }
        self.tx.blocks_since_key = self.tx.blocks_since_key.wrapping_add(1);
        self.status.mic_level.store(peak.to_bits(), Ordering::Relaxed);
        self.status.mic_clipped.store(self.rx.keyed() && self.rx.mic_clipped(), Ordering::Relaxed);
    }

    /// Key and unkey from the vox, if a channel has one.
    ///
    /// The decision is the vox stage's, taken on the audio that would go out;
    /// what this does is the half a node cannot, because keying retunes a
    /// half duplex radio and hands its stream over, and only this thread
    /// holds the device.
    ///
    /// A key pressed by hand is left alone: an operator who keys a vox
    /// channel gets the over they asked for, and the vox lets it up when they
    /// stop talking, which is what every radio with both does.
    pub(super) fn vox(&mut self) {
        let Some(state) = self.rx.vox_state() else {
            self.tx.vox_keyed = None;
            return;
        };
        self.status.vox_level.store(state.level.to_bits(), Ordering::Relaxed);
        self.status.vox_open.store(state.open, Ordering::Relaxed);
        self.status.vox_held.store(state.held, Ordering::Relaxed);
        let keyed = self.status.keyed.load(Ordering::Relaxed);
        match (state.open, keyed) {
            (true, 0) => {
                // The channel the drawn transmit chain belongs to, which is
                // the one the vox is measuring for.
                let Some(id) = self.vox_channel() else { return };
                self.tx.vox_keyed = Some(id);
                self.key(id);
            }
            (false, k) if k != 0 && self.tx.vox_keyed == Some(k) => {
                self.tx.vox_keyed = None;
                self.unkey();
            }
            _ => {}
        }
    }

    /// The channel a vox would key: the one the transmit chain was drawn for,
    /// by the same rule [`derive_tx`] picks it.
    pub(super) fn vox_channel(&self) -> Option<u64> {
        let has_vox = |c: &&ChannelSpec| {
            c.tx.is_some_and(|t| t.source == TxSource::Mic && t.vox.on)
                && tx_plan_for(c, self.plan.center).is_some()
        };
        let last = self
            .tx
            .last_keyed
            .and_then(|id| self.plan.channels.iter().find(|c| c.id == id))
            .filter(has_vox);
        last.or_else(|| self.plan.channels.iter().find(has_vox)).map(|c| c.id)
    }
}
