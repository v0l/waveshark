//! Background RF thread: owns the device, publishes spectrum frames, and
//! demodulates whichever channel is selected for audio.

use audio::AudioPlayer;
use crate::chain::Plan;
use common::{GainMode, Hz, Sps, C32};
use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use std::sync::{
    atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering},
    Arc,
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
}

impl ChanMode {
    pub fn label(&self) -> String {
        match self {
            ChanMode::Audio(d) => d.label().to_string(),
            ChanMode::Decode(kind) => crate::chain::front_label(kind),
        }
    }

    /// The demodulator this channel plays, if it plays one.
    pub fn demod(&self) -> Option<Demod> {
        match self {
            ChanMode::Audio(d) => Some(*d),
            ChanMode::Decode(_) => None,
        }
    }

    pub fn is_decode(&self) -> bool {
        matches!(self, ChanMode::Decode(_))
    }

    /// Occupied bandwidth, two-sided: what the channel covers on the
    /// spectrum, and what a filter in front of it has to pass.
    pub fn bandwidth(&self) -> f64 {
        match self {
            ChanMode::Audio(d) => d.bandwidth(),
            ChanMode::Decode(kind) => crate::chain::front_width(kind).unwrap_or(12_500.0),
        }
    }

    /// The least span this channel can be built in.
    pub fn min_rate(&self) -> f64 {
        match self {
            ChanMode::Audio(d) => d.if_rate(),
            // The front end mixes and decimates its own channel out of
            // whatever it is handed, so what it needs is a stream that holds
            // the channel at all.
            ChanMode::Decode(_) => self.bandwidth() * 2.0,
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
        }
        h.finish()
    }
}

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
    gain: GainMode,
    ppm: f64,
) -> common::Result<(Box<dyn common::Device>, Box<dyn common::RxStream>, f64)> {
    // The device needs a moment to release its USB claim; reopening
    // immediately gets "already in use".
    std::thread::sleep(std::time::Duration::from_millis(150));
    let mut dev = crate::devices::open(entry)?;
    dev.set_rate(rate)?;
    // Reopening resets the correction, and a span change that silently threw
    // it away would put every frequency back where it was wrong.
    let soft = apply_ppm(dev.as_mut(), ppm);
    dev.set_center(tuned(center, soft))?;
    let _ = dev.set_gain("tuner", gain);
    let stream = dev.start_rx()?;
    Ok((dev, stream, soft))
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
fn apply_ppm(dev: &mut dyn common::Device, ppm: f64) -> f64 {
    let _ = dev.set_ppm(ppm);
    match (dev.ppm() - ppm).abs() < 0.001 {
        true => 0.0,
        false => ppm,
    }
}

/// What to ask the hardware for so the receiver ends up on `want`.
///
/// A reference running fast by `ppm` puts the local oscillator that much
/// above where it was asked for, so the request goes that much below.
fn tuned(want: Hz, ppm: f64) -> Hz {
    match ppm == 0.0 {
        true => want,
        false => Hz((want.as_f64() / (1.0 + ppm * 1e-6)).round().max(0.0) as u64),
    }
}

/// The inverse: where the receiver actually is, given what the hardware was
/// asked for. What the dial and the spectrum are labelled with.
fn untuned(hw: Hz, ppm: f64) -> Hz {
    match ppm == 0.0 {
        true => hw,
        false => Hz((hw.as_f64() * (1.0 + ppm * 1e-6)).round().max(0.0) as u64),
    }
}

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

/// Largest sample in a buffer, which is what a meter reads.
fn peak_of(pcm: &[f32]) -> f32 {
    pcm.iter().fold(0.0f32, |a, v| a.max(v.abs()))
}

/// Overridable so the benchmark can measure what happens without the spacing.
fn tune_gap() -> std::time::Duration {
    match std::env::var("SR_TUNE_GAP_MS").ok().and_then(|v| v.parse().ok()) {
        Some(ms) => std::time::Duration::from_millis(ms),
        None => MIN_TUNE_GAP,
    }
}

pub enum Cmd {
    Center(Hz),
    Rate(Sps),
    /// The complete set of channels to demodulate and mix.
    Channels(Vec<ChannelSpec>),
    /// Master volume, which the whole mix leaves the bus at.
    Volume(f32),
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
    /// Reference oscillator correction, in parts per million.
    Ppm(f64),
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
    CallSubs(Vec<crate::audiobus::Subscription>),
    /// Stop a replay part way through.
    StopPlay,
    /// The level all call audio is heard at, from the channel strip.
    CallVolume { volume: f32, muted: bool },
    /// Whether the call bus levels what it passes on.
    CallAgc(bool),
    /// Play a transmission that has already been decoded, once.
    ///
    /// The sink belongs to this thread, so replaying from the packet list is
    /// a message rather than the interface opening its own audio device.
    Play(std::sync::Arc<common::Speech>),
    /// Write every burst that decodes to this directory, with an optional
    /// budget in megabytes, or stop recording.
    Record(Option<(std::path::PathBuf, Option<u64>)>),
    /// Start or stop writing the raw span to a file.
    CaptureIq(bool),
    /// Size the capture folder may reach before writing stops, in bytes.
    CaptureCap(u64),
    /// Where the receiver is, in degrees, which lets the flight tracker
    /// resolve a position from a single frame instead of waiting for a pair.
    Location(f64, f64),
    /// Log every burst the front ends detect to this directory, or stop.
    ///
    /// Sent to the radio thread rather than kept in the interface because the
    /// log is a node in the graph: what it writes is what the demodulators
    /// produced, which never reaches the interface at all.
    PacketLog(Option<std::path::PathBuf>),
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
    /// Whether the graph is being edited, for the view. Nothing about what
    /// runs depends on it: the operator's edits apply either way.
    Manual(bool),
    /// What the operator changed about the graph, as the whole set: it is
    /// put on top of whatever the receiver draws next, so a retune keeps it.
    Edits(crate::patch::Edits),
    /// Install a TETRA key for a cell colour on every front end, so its
    /// enciphered traffic decodes. From the key manager.
    #[cfg(feature = "tea")]
    TetraKey { colour: u8, key: decode::tea::Key },
    /// Install a TA61 identity secret for a cell colour, so its encrypted
    /// identities show as real subscribers. From the key manager.
    #[cfg(feature = "tea")]
    TetraIdSecret { colour: u8, c: [u8; 8] },
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
    pub volume: f32,
    pub muted: bool,
    /// None leaves the mode's own default.
    pub squelch_db: Option<f32>,
    pub agc: bool,
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
    /// Protocol name, or "unknown" for a burst nothing claimed.
    pub model: String,
    /// How it was keyed: OOK, FSK, ASK.
    pub modulation: &'static str,
    /// Fields for a decode, inferred coding and timings for an unknown.
    pub detail: String,
    /// The same fields, structured.
    ///
    /// This is what makes the packet list a bus rather than a display: a map,
    /// a chart or an image pane reads these rather than the bytes or the
    /// summary line. See `docs/views.md`.
    pub fields: Vec<(String, common::Value)>,
    /// What the payload is, as a media type, so a view can claim packets it
    /// knows how to render without knowing the protocol that made them.
    pub media_type: &'static str,
    /// Received level in dBFS, and signal to noise in dB.
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    pub bytes: Vec<u8>,
    /// `None` when the protocol has no integrity check, which must stay
    /// visible: an unchecked decode from a noisy band is often wrong.
    pub crc: Option<bool>,
    /// The burst's samples, for the view that shows a packet, when the
    /// front end kept them.
    pub iq: Option<std::sync::Arc<common::IqBurst>>,
    /// What was said, for a voice protocol. This is the payload of such a
    /// transmission: the bytes of a vocoded stream say nothing to anybody.
    pub audio: Option<std::sync::Arc<common::Speech>>,
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
            self.model,
            self.bytes.len(),
            self.detail
        )
    }

    /// A bare record, for tests that need one to hand to something else.
    #[cfg(test)]
    pub fn for_test(freq: f64, model: &str) -> Self {
        Self {
            at: std::time::Instant::now(),
            freq,
            channel_hz: 31_250.0,
            model: model.to_string(),
            modulation: "OOK",
            detail: String::new(),
            fields: Vec::new(),
            media_type: pipeline::event::media::BYTES,
            rssi_dbfs: -20.0,
            snr_db: 15.0,
            bytes: vec![1, 2, 3],
            crc: Some(true),
            iq: None,
            audio: None,
        }
    }

    /// Whether any protocol claimed this burst.
    pub fn is_known(&self) -> bool {
        self.model != "unknown"
    }
}

/// Decodes everything audible in the current span, without being tuned or told
/// what to look for.
///
/// Two channelizers, not one, because the two front ends want opposite things
/// from a channel. Measured on the Fine Offset capture by adding noise until
/// decoding stops, a 1.5 kbit/s OOK sensor survives down to 12.3 dB
/// peak-to-noise in a 31 kHz channel and only 22.9 dB in a 125 kHz one: the
/// detector integrates noise across the whole channel while the signal
/// occupies a sliver of it, so a wide channel costs 10.6 dB for nothing. An
/// FSK transmitter needs the opposite, because its two tones are tens of kHz
/// apart and a narrow channel simply cuts one of them off: the same synthetic
/// packet reads as 46 bits at 110 us a symbol in a 125 kHz channel and as
/// eight bits of nonsense in a 31 kHz one.
///
/// Neither front end needs the signal centred, which is what makes any of this
/// work: the OOK path is an envelope detector and does not care where in the
/// channel the carrier sits, and the FSK path measures both tones from the
/// burst itself, so a SAW transmitter tens of kHz off nominal reads the same
/// as one on frequency. Width therefore costs sensitivity and nothing else.
/// Bursts already reported, for long enough to recognise the same one
/// arriving again from another channel.
///
/// Deduping within a block is not enough. Reads from the radio are short,
/// about seven milliseconds at 2.3 MS/s, and a burst that starts near the end
/// of one is finished by the detectors in the next, so the copies from
/// neighbouring channels straddle the boundary. Measured on live 868 MHz
/// traffic, one transmission appeared as four rows 31 kHz apart.
#[derive(Default)]
struct Dedupe {
    recent: Vec<Reported>,
}

impl Dedupe {
    /// Whether a burst is new, remembering it if so.
    fn accept(&mut self, r: &DecodeRecord, now: std::time::Instant) -> bool {
        self.recent.retain(|k| now.saturating_duration_since(k.at) < DEDUPE_WINDOW);
        if self.recent.iter().any(|k| same_burst(k, r)) {
            return false;
        }
        self.recent.push(Reported {
            at: r.at,
            freq: r.freq,
            channel_hz: r.channel_hz,
            modulation: r.modulation,
            known: r.is_known(),
        });
        true
    }

    fn clear(&mut self) {
        self.recent.clear();
    }
}

/// A burst that has already been logged.
#[derive(Clone, Copy, Debug)]
struct Reported {
    at: std::time::Instant,
    freq: f64,
    channel_hz: f64,
    modulation: &'static str,
    /// Whether a protocol claimed it.
    known: bool,
}

/// How long a burst stays in that memory.
///
/// A block is roughly a tenth of a second, and a burst that starts near the
/// end of one is finished by the detectors in the next, so its copies from
/// neighbouring channels straddle the boundary and a per-block comparison
/// misses half of them. Measured on live 868 MHz traffic, one transmission
/// appeared as four rows 31 kHz apart across two blocks.
///
/// Long enough to cover that, short enough that a device repeating its packet
/// two or three times a second still gets a row per repeat.
const DEDUPE_WINDOW: std::time::Duration = std::time::Duration::from_millis(300);

/// Scan a buffer while recording, as the radio thread does. Test support.
/// A receiver set up to sweep a capture, the way the live one sweeps the air.
pub(crate) fn replay_receiver(buf: &common::IqBuf, rec: Option<crate::record::Recorder>) -> anyhow::Result<crate::chain::Receiver> {
    let rate = buf.rate.as_f64();
    // A 1090 MHz capture goes through the wideband path instead of the
    // channel banks, the same way the live receiver decides: 1090 carries
    // nothing the ISM banks understand, so running them there only spends CPU
    // inventing unknown bursts out of Mode S.
    // A capture goes through whatever front end the scanner table puts on
    // its frequency, the same way the live receiver decides.
    let plan = replay_plan(buf, rec.is_some());
    Ok(crate::chain::Receiver::build(&plan, crate::chain::Sinks { recorder: rec, ..Default::default() })?)
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
        fft: 1024,
        channels: Vec::new(),
        audio: crate::chain::AudioPlan::default(),
        fronts,
        feeds: Vec::new(),
        edits: Default::default(),
        record,
        capture_dir: crate::chain::default_capture_dir(),
        log: false,
    }
}

/// Sweep a capture as the radio thread does, block by block.
///
/// Blocks are the size the radio delivers, because deduplication depends on
/// how a burst falls across block boundaries and a whole-file call would not
/// exercise it.
/// When a block's signal arrived, given the moment its processing finished.
///
/// A decode is stamped with the start of the block that carried it rather than
/// the moment the decoder finished with it. The burst is somewhere inside
/// those samples, and a stamp taken afterwards drifts by however long decoding
/// took, which on a loaded machine is longer than the block itself.
fn block_start(finished: std::time::Instant, samples: usize, rate: f64) -> std::time::Instant {
    finished - std::time::Duration::from_secs_f64(samples as f64 / rate.max(1.0))
}

/// What the bus is subscribed to, kept where a rebuild cannot lose it.
///
/// Every level on the bus is a setting the plan carries and the patch draws,
/// so those come back with the graph. A subscription is a rule rather than
/// a number, and the patch has no way to write one, so the set is kept here
/// and handed to whatever bus a rebuild produces.
#[derive(Default)]
struct BusSettings {
    subs: Vec<crate::audiobus::Subscription>,
}

impl BusSettings {
    fn apply(&self, rx: &mut crate::chain::Receiver) {
        if let Some(n) = rx.audio_mut() {
            n.bus_mut().set_subscriptions(self.subs.clone());
        }
    }
}

pub(crate) fn replay_blocks(rx: &mut crate::chain::Receiver, buf: &common::IqBuf) -> Vec<DecodeRecord> {
    let mut dedupe = Dedupe::default();
    let mut out = Vec::new();
    let rate = buf.rate.as_f64().max(1.0);
    for block in buf.samples.chunks(16_384) {
        if rx.process(block).is_err() {
            break;
        }
        let at = block_start(std::time::Instant::now(), block.len(), rate);
        let mut found = rx.decodes(at);
        dedupe_neighbours(&mut found);
        let seen = out.len();
        out.extend(found.into_iter().filter(|r| !r.model.is_empty() && dedupe.accept(r, at)));
        if let Some(r) = rx.recorder_mut() {
            for d in &out[seen..] {
                r.capture(d);
            }
        }
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

/// Drop the copies of a burst that other channels also reported.
///
/// Channels overlap by design: a two times oversampled channelizer hands
/// adjacent channels each other's transition band, so a transmitter sitting
/// anywhere near an edge is genuinely present in two of them, and its
/// sidebands reach further still. Each of those channels runs its own
/// detector, reads a mangled copy of the same burst, and reports it. Measured
/// on a synthetic FSK packet, the channel holding the signal read it correctly
/// as 46 bits at 110 us a symbol while its neighbour reported 139 bits at
/// 36 us: not a second device, just the same one seen through a filter skirt.
/// Running two banks over the same air makes this certain rather than likely.
///
/// The strongest report of a burst wins, and a real decode beats an unknown
/// however loud, because a protocol that matched its own CRC is better
/// evidence than a stronger guess. Marked by clearing the model rather than
/// removed here, so the caller can compact once.
/// Whether a new report is the same burst as one already logged.
///
/// Same channel through the same front end is a second transmission, which on
/// a device that repeats its packet is exactly what should be logged. Same
/// channel through the other front end is one burst read twice. Anything else
/// near enough in frequency is one signal seen through a filter skirt: two and
/// a half channels either side, taken from the wider of the two reports
/// because that is the one whose skirts reach furthest.
fn same_burst(kept: &Reported, new: &DecodeRecord) -> bool {
    // A real decode is never a copy of a guess. The front end names what it
    // measured about every burst, including the ones it read nothing from,
    // and a measurement of noise a few kilohertz off a sensor a moment
    // before it keyed up must not stand in for the sensor's packet.
    if new.is_known() && !kept.known {
        return false;
    }
    let d = (kept.freq - new.freq).abs();
    if d < 1.0 && (kept.channel_hz - new.channel_hz).abs() < 1.0 {
        return kept.modulation != new.modulation;
    }
    d <= 2.5 * kept.channel_hz.max(new.channel_hz)
}

fn dedupe_neighbours(block: &mut [DecodeRecord]) {
    let mut order: Vec<usize> = (0..block.len()).collect();
    order.sort_by(|&a, &b| {
        let key = |r: &DecodeRecord| (r.is_known(), r.rssi_dbfs);
        let (ka, kb) = (key(&block[a]), key(&block[b]));
        kb.0.cmp(&ka.0).then(kb.1.total_cmp(&ka.1))
    });

    let mut kept: Vec<(f64, f64, &'static str, bool)> = Vec::new();
    for i in order {
        let dup = kept.iter().any(|(kf, kw, km, known)| {
            same_burst(
                &Reported {
                    at: block[i].at,
                    freq: *kf,
                    channel_hz: *kw,
                    modulation: km,
                    known: *known,
                },
                &block[i],
            )
        });
        if dup {
            block[i].model.clear();
        } else {
            kept.push((block[i].freq, block[i].channel_hz, block[i].modulation, block[i].is_known()));
        }
    }
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
    /// Shape of the chain currently demodulating, republished on every rebuild.
    chain: parking_lot::Mutex<Option<pipeline::graph::Topology>>,
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
    /// Whether the wideband Mode S path is the one running.
    pub modes_on: AtomicBool,
    /// Whether the AIS path is the one running.
    pub ais_on: AtomicBool,
    /// Whether the APRS path is the one running.
    pub aprs_on: AtomicBool,
    /// Whether the pager path is the one running.
    pub pocsag_on: AtomicBool,
    pub m17_on: AtomicBool,
    /// Software zoom currently applied, 1 for none.
    pub zoom: AtomicU64,
    /// Whether the operator owns the shape of the graph.
    pub manual: AtomicBool,
    /// The levels as the nodes hold them, republished when a setting made
    /// through the chain view changed one, so the strip can follow.
    levels: parking_lot::Mutex<(u64, crate::chain::AudioPlan, Vec<ChannelSpec>)>,
    /// The patch the receiver is actually running, which is not always the
    /// one last sent: an edit that will not build is refused and the previous
    /// one goes back.
    patch: parking_lot::Mutex<Option<(crate::patch::Patch, crate::patch::Patch)>>,
    /// Bumped whenever the radio thread replaces it, so the interface can
    /// tell its own edit from one being handed back.
    pub patch_rev: AtomicU64,
    /// Bursts written to the packet log since the receiver started.
    pub logged: AtomicU64,
    /// Whether anything is subscribed on the call bus, whether a recorded
    /// transmission is playing, and what the bus last passed through.
    pub call_audio: AtomicBool,
    /// Every input of the bus, and where the bus is in the graph, so a strip
    /// the operator drew can be given a level by the same route the chain
    /// view uses.
    strips: parking_lot::Mutex<(Option<usize>, Vec<crate::chain::StripState>)>,
    pub replaying: AtomicBool,
    pub call_heard: parking_lot::Mutex<Option<String>>,
    /// What each voice source put into the mix last block, keyed as
    /// `system:channel`. The meter on a call's own row, which separates
    /// "nothing was decoded" from "it was decoded and you still cannot hear
    /// it": two different faults that sound identical.
    call_levels: parking_lot::Mutex<Vec<(String, f32)>>,
    /// The TETRA cells heard and their key state, for the key manager.
    tetra_keys: parking_lot::Mutex<Vec<nodes::tetra_nodes::KeyStatus>>,
    /// Peak of the whole mix as it left for the speaker, and of the call
    /// bus's share of it, for the meters beside the master and call faders.
    out_level: AtomicU32,
    call_level: AtomicU32,
    /// Voice transmissions written to disk since the receiver started.
    pub calls_written: AtomicU64,
    /// What the call bus's gain control is adding, in dB, as f32 bits.
    call_gain_db: AtomicU32,
}

/// Everything the radio itself can be set to, and what it is set to now.
///
/// Read back from the driver rather than remembered by the interface, because
/// the hardware quantises: ask an R820T for 30 dB and it gives 29.7, ask a
/// HackRF's LNA for 20 and it gives 16. A control showing the request rather
/// than the result is lying about the receiver.
#[derive(Clone, Debug, Default)]
pub struct RadioControls {
    pub stages: Vec<(common::GainStage, GainMode)>,
    pub toggles: Vec<common::Toggle>,
    pub choices: Vec<common::Choice>,
    pub ppm: f64,
}

impl RadioControls {
    /// `ppm` is the correction in force, which is not always the device's
    /// own: one that cannot correct itself is corrected by the radio thread,
    /// and reading the setting back off the driver would report zero and
    /// throw away what was just typed.
    fn read(dev: &dyn common::Device, ppm: f64) -> Self {
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
        Self { stages, toggles: dev.toggles(), choices: dev.choices(), ppm }
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
            call_audio: AtomicBool::new(false),
            strips: parking_lot::Mutex::new((None, Vec::new())),
            replaying: AtomicBool::new(false),
            call_heard: parking_lot::Mutex::new(None),
            call_levels: parking_lot::Mutex::new(Vec::new()),
            tetra_keys: parking_lot::Mutex::new(Vec::new()),
            out_level: AtomicU32::new(0),
            call_level: AtomicU32::new(0),
            calls_written: AtomicU64::new(0),
            call_gain_db: AtomicU32::new(0),
            error: parking_lot::Mutex::new(None),
            blend: AtomicU32::new(0),

            radio: parking_lot::Mutex::new(RadioControls::default()),
            channels: parking_lot::Mutex::new(Vec::new()),
            stations: parking_lot::Mutex::new(Vec::new()),
            chain: parking_lot::Mutex::new(None),
            chain_latency: AtomicU32::new(0),
            decoded: AtomicU64::new(0),
            scan_channels: AtomicU64::new(0),
            scan_channels_wide: AtomicU64::new(0),
            sources_on: AtomicBool::new(false),
            sources: parking_lot::Mutex::new(Vec::new()),
            aircraft: AtomicU64::new(0),
            logged: AtomicU64::new(0),
            track_list: parking_lot::Mutex::new(Vec::new()),
            capture_on: AtomicBool::new(false),
            capture_bytes: AtomicU64::new(0),
            capture_folder: AtomicU64::new(0),
            capture_full: AtomicBool::new(false),
            capture_file: parking_lot::Mutex::new(None),
            log_bytes: AtomicU64::new(0),
            log_full: std::sync::atomic::AtomicBool::new(false),
            feeds: parking_lot::Mutex::new(Vec::new()),
            modes_on: AtomicBool::new(false),
            ais_on: AtomicBool::new(false),
            aprs_on: AtomicBool::new(false),
            pocsag_on: AtomicBool::new(false),
            m17_on: AtomicBool::new(false),
            zoom: AtomicU64::new(1),
            manual: AtomicBool::new(false),
            patch: parking_lot::Mutex::new(None),
            levels: parking_lot::Mutex::new((0, crate::chain::AudioPlan::default(), Vec::new())),
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

    /// The levels as the graph holds them, and a revision that moves only
    /// when something other than the strip changed one.
    pub fn levels(&self) -> (u64, crate::chain::AudioPlan, Vec<ChannelSpec>) {
        self.levels.lock().clone()
    }

    fn set_levels(&self, audio: crate::chain::AudioPlan, chans: Vec<ChannelSpec>) {
        let mut held = self.levels.lock();
        *held = (held.0 + 1, audio, chans);
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
    pub fn call_levels(&self) -> Vec<(String, f32)> {
        self.call_levels.lock().clone()
    }

    /// Every input of the bus as the strip draws it, and the bus's node id.
    pub fn strips(&self) -> (Option<usize>, Vec<crate::chain::StripState>) {
        self.strips.lock().clone()
    }

    fn set_strips(&self, node: Option<usize>, mut strips: Vec<crate::chain::StripState>) {
        let mut held = self.strips.lock();
        for s in &mut strips {
            if let Some(prev) = held.1.iter().find(|p| p.port == s.port) {
                s.level = s.level.max(prev.level * METER_FALL);
            }
        }
        *held = (node, strips);
    }

    /// The mix's own level, and the call bus's share of it.
    pub fn out_level(&self) -> f32 {
        f32::from_bits(self.out_level.load(Ordering::Relaxed))
    }

    pub fn call_level(&self) -> f32 {
        f32::from_bits(self.call_level.load(Ordering::Relaxed))
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
    pub fn start(
        entry: crate::devices::Entry,
        center: Hz,
        rate: Sps,
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
                    run(entry, center, rate, fft, cmd_rx, frame_tx, dec_tx, &st, repaint)
                {
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

impl Drop for Radio {
    fn drop(&mut self) {
        self.send(Cmd::Stop);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
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
            volume: 1.0,
            muted: false,
            squelch_db: None,
            agc: true,
        };
        let plan = Plan {
            center: Hz(0),
            rate,
            zoom: 1,
            dc_block: false,
            refresh_hz: 30.0,
            fft: 1024,
            channels: vec![spec],
            audio: crate::chain::AudioPlan::default(),
            fronts: Vec::new(),
            edits: Default::default(),
            record: false,
            capture_dir: crate::chain::default_capture_dir(),
            log: false,
            feeds: Vec::new(),
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

#[allow(clippy::too_many_arguments)]
fn run(
    entry: crate::devices::Entry,
    center: Hz,
    rate: Sps,
    fft: usize,
    cmd: Receiver<Cmd>,
    frames: Sender<Frame>,
    decodes: Sender<Vec<DecodeRecord>>,
    status: &Status,
    repaint: impl Fn(),
) -> anyhow::Result<()> {
    let mut dev = crate::devices::open(&entry)?;
    // Clamp to what this radio can actually do: the app's last span may have
    // come from a different device entirely.
    let info_rates = dev.info().rate_range.clone();
    let rate = Sps(rate.0.clamp(info_rates.start().0, info_rates.end().0));
    dev.set_rate(rate)?;
    dev.set_center(center)?;
    dev.set_gain("tuner", GainMode::Auto)?;
    // Tracked so a restart can put it back: reopening a device resets it, and
    // a span change that silently returned the gain to its default would look
    // like the antenna had fallen out.
    let mut gain = GainMode::Auto;

    let (_player, mut sink) = match AudioPlayer::open(48_000) {
        Ok((p, s)) => (Some(p), Some(s)),
        Err(e) => {
            *status.error.lock() = Some(format!("no audio output: {e}"));
            (None, None)
        }
    };

    let mut stream = dev.start_rx()?;
    status.running.store(true, Ordering::Relaxed);
    status.set_radio(RadioControls::read(dev.as_ref(), 0.0));

    // What the receiver should be doing. Everything that acts on a sample is
    // in the graph this describes, so a command changes the plan and the
    // graph is rebuilt from it, rather than each command reaching into a
    // different object.
    let mut plan = Plan {
        center: dev.center(),
        rate: dev.rate().as_f64(),
        zoom: 1,
        dc_block: true,
        refresh_hz: 30.0,
        fft,
        channels: Vec::new(),
        audio: crate::chain::AudioPlan::default(),
        // Resolved from the scanner table below, once the tuning is known.
        fronts: Vec::new(),
        edits: Default::default(),
        record: false,
        capture_dir: crate::chain::default_capture_dir(),
        // Switched on as soon as the interface says where to write; the
        // default is on, and the command arrives with the first frame.
        log: false,
        // Feeds arrive from the session or the settings modal, as a command.
        feeds: Vec::new(),
    };
    // What to run follows from where the dial is, and that mapping is
    // configuration rather than structure.
    let mut scanners = crate::scanners::Scanners::load();
    plan.fronts = fronts_here(&scanners, &plan, true);
    let mut rx = crate::chain::Receiver::build(&plan, Default::default())?;
    publish_chain(status, &rx);

    let mut records: Vec<DecodeRecord> = Vec::new();
    // Whether the raw span is being written, held here because the stage is
    // rebuilt with the graph and comes back switched off.
    let mut capture_on = false;
    let mut dedupe = Dedupe::default();
    let mut hits = 0u64;
    let mut scan_on = true;
    // The last edits that built, to fall back on when an edit does not.
    let mut last_edits: Option<crate::patch::Edits> = None;
    let mut rebuild = false;
    // The correction asked for, and however much of it this thread has to
    // apply because the device would not.
    let mut ppm = 0.0f64;
    let mut soft_ppm = 0.0f64;
    let mut want_center: Option<Hz> = None;
    let mut last_chain = std::time::Instant::now();
    // What the bus is subscribed to, kept outside the graph because the bus
    // is a node and a rebuild can hand back a new one.
    let mut calls = BusSettings::default();
    let mut call_dir: Option<std::path::PathBuf> = None;
    let mut call_rec = crate::callrec::CallRecorder::default();
    let gap = tune_gap();
    let mut last_tune = std::time::Instant::now() - gap;

    loop {
        for c in cmd.try_iter() {
            match c {
                Cmd::Stop => {
                    stream.stop();
                    return Ok(());
                }
                // Held rather than applied. A drag issues one of these per
                // displayed frame and only the last is worth anything, so
                // applying each in turn spends the whole budget retuning to
                // frequencies already superseded.
                Cmd::Center(f) => want_center = Some(f),
                Cmd::Rate(r) => {
                    // A HackRF's streaming reader owns the device and its
                    // control channel does not carry the sample rate, so the
                    // radio has to be stopped, reopened and started again.
                    // Asking anyway used to fail, and the failure propagated
                    // out of this loop and killed the thread: changing
                    // bandwidth stopped the receiver dead.
                    if dev.rate_needs_restart() {
                        stream.stop();
                        drop(stream);
                        match restart(&entry, r, plan.center, gain, ppm) {
                            Ok((d, s, soft)) => {
                                dev = d;
                                stream = s;
                                soft_ppm = soft;
                            }
                            Err(e) => {
                                *status.error.lock() = Some(format!("cannot change span: {e}"));
                                return Ok(());
                            }
                        }
                    } else if let Err(e) = dev.set_rate(r) {
                        *status.error.lock() = Some(format!("cannot change span: {e}"));
                        continue;
                    }
                    plan.rate = dev.rate().as_f64();
                    rebuild = true;
                }
                Cmd::NodeParam(id, name, value) => {
                    match rx.set_node_param(id, &name, value) {
                        // A parameter that changes the stream's shape needs
                        // the graph negotiated again around it; the rest take
                        // effect on the next block.
                        Ok(true) => rebuild = true,
                        Ok(false) => publish_chain(status, &rx),
                        Err(e) => *status.error.lock() = Some(format!("{name}: {e}")),
                    }
                    // The receiver wrote it into its description. What that
                    // changed is either the operator's edit, which the next
                    // rebuild has to start from, or a level the strip owns,
                    // which the strip has to be told of.
                    plan.edits = rx.edits();
                    pull_levels(&rx, &mut plan, status);
                    status.set_patch(&rx);
                }
                Cmd::Channels(specs) => {
                    plan.channels = specs;
                    // A squelch or gain change is a number on a node that is
                    // already there. Rebuilding for it threw away the
                    // spectrum's averaging and every channel's state, once per
                    // frame for as long as the slider was held.
                    if rx.params_only(&plan) {
                        rx.apply_params(&plan);
                        publish_chain(status, &rx);
                    } else {
                        rebuild = true;
                    }
                }
                Cmd::Volume(v) => {
                    plan.audio.master = v;
                    if let Some(b) = rx.audio_mut() {
                        b.bus_mut().set_master(v, plan.audio.muted);
                    }
                }
                Cmd::Fft(n) => {
                    plan.fft = n;
                    rebuild = true;
                }
                Cmd::Refresh(hz) => {
                    plan.refresh_hz = hz.clamp(1.0, 120.0);
                    rx.set_refresh(plan.refresh_hz);
                }
                Cmd::Smoothing(v) => rx.set_smoothing(v.clamp(0.01, 1.0)),
                Cmd::DcBlock(on) => {
                    plan.dc_block = on;
                    rx.set_dc_block(on);
                }
                Cmd::GainStage(stage, mode) => {
                    if let Err(e) = dev.set_gain(&stage, mode) {
                        *status.error.lock() = Some(format!("{stage} gain: {e}"));
                    }
                    // Reopening for a rate change resets the device, so the
                    // tuner setting has to survive outside it.
                    if stage == "tuner" {
                        gain = mode;
                    }
                    // The driver snaps to what the hardware supports, so the
                    // control has to be told what it actually got rather than
                    // what it asked for.
                    status.set_radio(RadioControls::read(dev.as_ref(), ppm));
                    rx.remeasure_dc();
                }
                Cmd::Toggle(name, on) => {
                    if let Err(e) = dev.set_toggle(&name, on) {
                        *status.error.lock() = Some(format!("{name}: {e}"));
                    }
                    status.set_radio(RadioControls::read(dev.as_ref(), ppm));
                    // Any of these changes the offset, and a stale estimate
                    // shows up as a spur that was not there a moment ago.
                    rx.remeasure_dc();
                }
                Cmd::Choice(name, value) => {
                    // Some of these describe the stream rather than a setting
                    // on it: a LimeSDR's receive channel is a different stream
                    // entirely, so it has to be torn down and set up again.
                    if dev.choice_needs_restart(&name) {
                        // Dropped rather than only stopped: the driver counts
                        // a stopped stream as still holding the radio until
                        // its handle is gone.
                        stream.stop();
                        drop(stream);
                        if let Err(e) = dev.set_choice(&name, &value) {
                            *status.error.lock() = Some(format!("{name}: {e}"));
                        }
                        match dev.start_rx() {
                            Ok(s) => stream = s,
                            Err(e) => {
                                *status.error.lock() =
                                    Some(format!("cannot restart after {name}: {e}"));
                                return Ok(());
                            }
                        }
                    } else if let Err(e) = dev.set_choice(&name, &value) {
                        *status.error.lock() = Some(format!("{name}: {e}"));
                    }
                    status.set_radio(RadioControls::read(dev.as_ref(), ppm));
                    rx.remeasure_dc();
                }
                Cmd::Ppm(v) => {
                    ppm = v;
                    soft_ppm = apply_ppm(dev.as_mut(), v);
                    // Nothing moves until the tuner is asked for a frequency
                    // again, so ask now: a correction that only took effect
                    // on the next drag of the dial is a correction nobody
                    // can see themselves setting.
                    want_center = Some(plan.center);
                    last_tune = std::time::Instant::now() - gap;
                    status.set_radio(RadioControls::read(dev.as_ref(), ppm));
                    rebuild = true;
                }
                Cmd::Record(dir) => {
                    let rec = match dir {
                        Some((d, mb)) => match crate::record::Recorder::new(
                            &d,
                            plan.eff_rate(),
                            plan.center,
                        ) {
                            Ok(r) => Some(match mb {
                                Some(mb) => r.with_budget(mb << 20),
                                None => r,
                            }),
                            Err(e) => {
                                *status.error.lock() =
                                    Some(format!("cannot record to {}: {e}", d.display()));
                                None
                            }
                        },
                        None => None,
                    };
                    plan.record = rec.is_some();
                    rx.set_recorder(rec);
                    rebuild = true;
                }
                // No rebuild: the graph already holds the stage, switched
                // off, so a capture starts on the block after the button and
                // keeps every source the auto node has open.
                Cmd::CaptureIq(on) => {
                    capture_on = on;
                    rx.set_capture(on);
                }
                Cmd::Location(lat, lon) => rx.set_location(lat, lon),
                Cmd::PacketLogCap(cap) => rx.set_log_cap(cap),
                Cmd::CaptureCap(bytes) => rx.set_capture_cap(bytes),
                Cmd::Feeds(feeds) => {
                    if feeds != plan.feeds {
                        plan.feeds = feeds;
                        rebuild = true;
                    }
                }
                Cmd::Scanners(table) => {
                    // A different table can mean a different front end on the
                    // frequency the dial is already on, so this rebuilds
                    // rather than waiting for the next retune.
                    if table != scanners {
                        scanners = table;
                        rebuild = true;
                    }
                }
                // A lock on editing in the view, and nothing to the
                // receiver: what the operator changed applies either way,
                // and what they did not follows the dial either way.
                Cmd::Manual(on) => status.manual.store(on, Ordering::Relaxed),
                Cmd::Edits(e) => {
                    if e != plan.edits {
                        plan.edits = e;
                        rebuild = true;
                    }
                }
                #[cfg(feature = "tea")]
                Cmd::TetraKey { colour, key } => rx.set_tetra_key(colour, key),
                #[cfg(feature = "tea")]
                Cmd::TetraIdSecret { colour, c } => rx.set_tetra_id_secret(colour, c),
                Cmd::PacketLog(dir) => {
                    plan.log = dir.is_some();
                    // Voice is written beside the log rather than into it: a
                    // record of what was on the air stays small, and what it
                    // sounded like is a file per transmission.
                    call_dir = dir.clone().map(|d| d.join("calls"));
                    rx.set_packet_log(dir);
                    rebuild = true;
                }
                Cmd::Zoom(n) => {
                    let n = n.clamp(1, 64);
                    if n != plan.zoom {
                        plan.zoom = n;
                        rebuild = true;
                        status.zoom.store(n as u64, Ordering::Relaxed);
                    }
                }
                Cmd::Decode(on) => {
                    scan_on = on;
                    rebuild = true;
                }
                // The bus is a node, so it is rebuilt with the graph. What
                // it was told is kept here as well, and given to whatever
                // bus comes back: a retune must not silently unsubscribe.
                Cmd::CallSubs(subs) => {
                    calls.subs = subs.clone();
                    if let Some(b) = rx.audio_mut() {
                        b.bus_mut().set_subscriptions(subs);
                    }
                }
                Cmd::StopPlay => {
                    if let Some(b) = rx.audio_mut() {
                        b.bus_mut().stop_replay();
                    }
                }
                Cmd::CallVolume { volume, muted } => {
                    plan.audio.calls = volume;
                    plan.audio.calls_muted = muted;
                    if let Some(b) = rx.audio_mut() {
                        b.bus_mut().set_calls(volume, muted);
                    }
                }
                Cmd::CallAgc(on) => {
                    plan.audio.agc = on;
                    if let Some(b) = rx.audio_mut() {
                        b.bus_mut().set_agc(on);
                    }
                }
                Cmd::Play(speech) => {
                    if let Some(b) = rx.audio_mut() {
                        b.bus_mut().play(&speech);
                    }
                }
            }
        }

        // Retuning costs about 25 ms on the RTL-SDR, more than a frame at
        // 60 Hz, and it blocks the thread that reads samples. Spacing them out
        // keeps the spectrum live while a drag is in progress; the last
        // requested frequency is always reached because the pending one is
        // held until it can be applied.
        if let Some(f) = want_center {
            if last_tune.elapsed() >= gap {
                let _t = tracing::info_span!("set_center").entered();
                dev.set_center(tuned(f, soft_ppm))?;
                // The plan is labelled with where the receiver is, not with
                // what the tuner was asked for: the dial, the spectrum and
                // every channel offset are read against it.
                plan.center = untuned(dev.center(), soft_ppm);
                rebuild = true;
                want_center = None;
                last_tune = std::time::Instant::now();
            }
        }

        if rebuild {
            let _t = tracing::info_span!("rebuild").entered();
            // The banks understand nothing on either wideband band, so
            // running them there only spends CPU inventing unknown bursts.
            plan.fronts = fronts_here(&scanners, &plan, scan_on);
            let before: Vec<u64> = rx.channels().iter().map(|c| c.spec.id).collect();
            if let Err(e) = rx.rebuild(&plan) {
                // A patch is drawn wire by wire, so most of the time it is
                // half a graph, and a type mismatch between two stages is an
                // ordinary step rather than a fault. Going back to the last
                // edits that built keeps the receiver running while it is
                // said; without this an edit could stop the radio dead.
                let Some(good) = last_edits.clone() else {
                    *status.error.lock() = Some(format!("cannot build the chain: {e}"));
                    return Ok(());
                };
                *status.error.lock() = Some(format!("the patch was refused: {e}"));
                plan.edits = good;
                if let Err(e) = rx.rebuild(&plan) {
                    *status.error.lock() = Some(format!("cannot build the chain: {e}"));
                    return Ok(());
                }
            } else {
                // Only a shape that built is worth going back to.
                last_edits = Some(plan.edits.clone());
            }
            *status.error.lock() = rx.refused.clone();
            // A channel that was rebuilt has lost its RDS state, and its old
            // station name must not sit over whatever it is tuned to now.
            let kept: Vec<u64> = rx
                .channels()
                .iter()
                .filter(|c| before.contains(&c.spec.id) && c.kept)
                .map(|c| c.spec.id)
                .collect();
            status.keep_stations(&kept);
            // Every channel covers a different frequency now, so nothing
            // already reported can be the same burst as anything arriving.
            dedupe.clear();
            if let Some(r) = rx.recorder_mut() {
                r.retune(plan.eff_rate(), plan.center);
            }
            status.logged.store(rx.logged(), Ordering::Relaxed);
            // The stage comes back switched off, as the derived graph draws
            // it. A capture running across a retune has to be switched on
            // again, and it starts a new file: the old one's name says which
            // frequency and rate every sample in it was taken at.
            rx.set_capture(capture_on);
            // A bus built afresh is subscribed to nothing, exactly as the
            // capture comes back switched off.
            calls.apply(&mut rx);
            // What is running, described the way the view draws it, and
            // what the receiver drew underneath the edits, which is what an
            // edited copy is read against.
            status.set_patch(&rx);
            publish_chain(status, &rx);
            rebuild = false;
        }

        let read_span = tracing::info_span!("rf_read").entered();
        let buf = match stream.read() {
            Ok(b) => b,
            Err(_) => return Ok(()),
        };
        drop(read_span);
        status.dropped.store(stream.dropped(), Ordering::Relaxed);
        // Timed from here rather than around the loop: the read is where the
        // thread waits for the radio, so counting it would measure real time
        // against itself and always say exactly 1x.
        let work = std::time::Instant::now();
        let block_secs = buf.samples.len() as f64 / plan.rate.max(1.0);

        {
            let _g = tracing::info_span!("graph").entered();
            if let Err(e) = rx.process(&buf.samples) {
                *status.error.lock() = Some(format!("chain: {e}"));
                return Ok(());
            }
        }

        if rx.spectrum_ready() {
            // Published with the spectrum rather than every block: the table
            // is redrawn at the display's rate, and cloning it 140 times a
            // second for a pane nobody may be looking at is wasted work.
            if rx.tracking() {
                let rows = rx.tracks(std::time::Instant::now());
                status.aircraft.store(rows.len() as u64, Ordering::Relaxed);
                *status.track_list.lock() = rows;
            }
            if !plan.feeds.is_empty() {
                *status.feeds.lock() = rx.feed_status();
            }
            // The chain carries what each wire is measured to be passing, so
            // it is republished while it runs rather than only when its shape
            // changes: a graph drawn once at build time reports the throughput
            // it had before any samples went through it, which is none.
            if last_chain.elapsed() >= CHAIN_PUBLISH {
                publish_chain(status, &rx);
                last_chain = std::time::Instant::now();
            }
            // The rate the spectrum sees rather than the one the radio
            // delivers: in manual mode a stage can sit between the two, and
            // an axis drawn from the wrong one puts every signal in the
            // wrong place.
            let seen = rx.spectrum_rate();
            let extra = rx
                .patch_spectra()
                .into_iter()
                .map(|(tag, db, center, rate)| Spectrum { tag, db, center, rate })
                .collect();
            let f = Frame {
                db: rx.power_db().to_vec(),
                center: plan.center.as_f64(),
                rate: if seen > 0.0 { seen } else { plan.eff_rate() },
                extra,
            };
            // Drop rather than block: the radio must never stall waiting for
            // the UI, and a stale spectrum is worthless anyway.
            match frames.try_send(f) {
                Ok(()) | Err(TrySendError::Full(_)) => {}
                Err(TrySendError::Disconnected(_)) => return Ok(()),
            }
            repaint();
        }

        status.modes_on.store(rx.modes_on(), Ordering::Relaxed);
        status.ais_on.store(rx.ais_on(), Ordering::Relaxed);
        status.aprs_on.store(rx.aprs_on(), Ordering::Relaxed);
        status.pocsag_on.store(rx.pocsag_on(), Ordering::Relaxed);
        status.m17_on.store(rx.m17_on(), Ordering::Relaxed);
        {
            rx.refresh_capture_folder();
            let cap = rx.capture();
            status.capture_on.store(rx.capturing(), Ordering::Relaxed);
            status.capture_bytes.store(cap.map(|c| c.bytes()).unwrap_or(0), Ordering::Relaxed);
            status
                .capture_folder
                .store(cap.map(|c| c.folder_bytes()).unwrap_or(0), Ordering::Relaxed);
            status.capture_full.store(cap.is_some_and(|c| c.is_full()), Ordering::Relaxed);
            *status.capture_file.lock() =
                cap.and_then(|c| c.path()).map(|p| p.display().to_string());
        }
        status.logged.store(rx.logged(), Ordering::Relaxed);
        status.log_bytes.store(rx.log_bytes(), Ordering::Relaxed);
        status.log_full.store(rx.log_full(), Ordering::Relaxed);
        let chans = rx.bank_channels();
        status.scan_channels.store(chans.first().copied().unwrap_or(0) as u64, Ordering::Relaxed);
        status
            .scan_channels_wide
            .store(chans.get(1).copied().unwrap_or(0) as u64, Ordering::Relaxed);
        status.sources_on.store(rx.has_sources(), Ordering::Relaxed);
        {
            let now = std::time::Instant::now();
            let mut seen = status.sources.lock();
            for e in seen.iter_mut() {
                e.live = false;
            }
            for s in rx.live_sources() {
                let same = seen.iter_mut().find(|e| {
                    (e.source.center_hz - s.center_hz).abs()
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

        // Stamped at the start of the block rather than at the moment the
        // decode fell out of it. The packet happened somewhere inside the
        // block, and a pulse detector only closes a package once it has seen
        // the silence afterwards, so "now" is always late by up to a block.
        let block =
            std::time::Duration::from_secs_f64(buf.samples.len() as f64 / plan.rate.max(1.0));
        let at = std::time::Instant::now() - block;

        records.clear();
        records.extend(rx.decodes(at));
        // Speech goes to a file per over, assembled from the bursts that
        // carried it and written when the over ends or goes quiet.
        if let Some(dir) = &call_dir {
            let mut done: Vec<crate::callrec::Finished> = Vec::new();
            for r in &records {
                done.extend(call_rec.feed(r, at, dir));
            }
            done.extend(call_rec.tick(at, dir));
            for f in done {
                match crate::audiobus::write_wav(&f.path, &f.speech) {
                    Ok(()) => status.calls_written.fetch_add(1, Ordering::Relaxed),
                    Err(e) => {
                        *status.error.lock() =
                            Some(format!("cannot write {}: {e}", f.path.display()));
                        0
                    }
                };
            }
        }
        dedupe_neighbours(&mut records);
        records.retain(|r| !r.model.is_empty() && dedupe.accept(r, at));
        if let Some(r) = rx.recorder_mut() {
            for d in &records {
                r.capture(d);
            }
            if r.is_full() {
                let mb = r.written() >> 20;
                *status.error.lock() = Some(format!("recording stopped: wrote {mb} MB"));
                plan.record = false;
                rx.set_recorder(None);
                rebuild = true;
            }
        }
        if !records.is_empty() {
            hits += records.len() as u64;
            status.decoded.store(hits, Ordering::Relaxed);
            // Never block the radio thread on a UI that is behind; a dropped
            // batch is reported by the counter going up without the log
            // growing to match.
            match decodes.try_send(std::mem::take(&mut records)) {
                Ok(()) | Err(TrySendError::Full(_)) => {}
                Err(TrySendError::Disconnected(_)) => return Ok(()),
            }
            records = Vec::new();
            repaint();
        }

        let _a = tracing::info_span!("audio").entered();
        // Everything that is heard was mixed on the bus, in the graph: every
        // channel at its fader, every subscribed call, a replay. What is
        // left to do here is hand the block to the device and read the
        // meters back.
        status.set_channel_states(rx.channel_states());
        status.set_strips(rx.audio_node_id(), rx.strips());
        *status.tetra_keys.lock() = rx.tetra_key_status();
        if let Some(b) = rx.audio().map(|n| n.bus()) {
            Status::set_level(&status.call_level, b.voice_peak());
            *status.call_levels.lock() = b.levels();
            status.call_gain_db.store(b.agc_gain_db().to_bits(), Ordering::Relaxed);
            status.call_audio.store(b.listening(), Ordering::Relaxed);
            status.replaying.store(b.replaying(), Ordering::Relaxed);
            *status.call_heard.lock() = b.last_heard().map(str::to_string);
        }
        if let Some(s) = sink.as_mut() {
            let (out, rate) = rx.audio_out();
            if !out.is_empty() {
                Status::set_level(&status.out_level, peak_of(out));
                s.write_adaptive_stereo(out, rate);
                status.audio_backlog.store(s.backlog().max(0) as u64, Ordering::Relaxed);
            } else {
                Status::set_level(&status.out_level, 0.0);
            }
            let wfm = |c: &&crate::chain::Chan| c.spec.mode == ChanMode::Audio(Demod::Wfm);
            for w in rx.channels().iter().filter(wfm) {
                let (g, e, sy) = w.rds_stats;
                status.set_station(w.spec.id, &w.station, g, e, sy);
            }
            if let Some(w) = rx.channels().iter().find(wfm) {
                status.set_blend(w.blend);
            }
        }

        status.push_speed((block_secs / work.elapsed().as_secs_f64().max(1e-9)) as f32);
    }
}

/// Take the levels the nodes hold into the plan, and tell the strip when
/// they differ from what it last said.
///
/// A fader or a squelch set through the chain view lands on the node. The
/// plan is what the next rebuild draws from, so it has to follow; and the
/// strip is what the operator reads, so it has to follow too. A revision
/// moves only when something changed, so what the strip sends itself does
/// not come back to it.
fn pull_levels(rx: &crate::chain::Receiver, plan: &mut Plan, status: &Status) {
    let (audio, chans) = rx.levels();
    let mut changed = audio != plan.audio;
    plan.audio = audio;
    for c in chans {
        if let Some(have) = plan.channels.iter_mut().find(|h| h.id == c.id) {
            if *have != c {
                *have = c;
                changed = true;
            }
        }
    }
    if changed {
        status.set_levels(plan.audio, plan.channels.clone());
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
    status.set_chain(Some(rx.topology()), rx.latency_ms(0));
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
        fft: 1024,
        channels: Vec::new(),
        audio: crate::chain::AudioPlan::default(),
        fronts: vec![crate::scanners::FrontAt {
            front: crate::scanners::Front::Banks(crate::scanners::DEFAULT_WIDTHS.to_vec()),
            // The whole span: these tests are about the shape of the
            // receiver, not about which band a block covers.
            band: (0.0, f64::INFINITY),
        }],
        edits: Default::default(),
        record: false,
        capture_dir: crate::chain::default_capture_dir(),
        log: false,
        feeds: Vec::new(),
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::chain::{FSK_CHANNEL_HZ, OOK_CHANNEL_HZ};

    /// The correction has to move the request the opposite way to the error,
    /// and it has to come back to the frequency the operator asked for, or
    /// the dial reads one thing and the receiver hears another.
    #[test]
    fn a_correction_offsets_the_request_and_reads_back_where_it_started() {
        let want = Hz(145_000_000);
        let hw = tuned(want, 20.0);
        assert!(hw.get() < want.get(), "a fast reference is asked for a lower frequency");
        let moved = want.get() - hw.get();
        assert!((2_800..3_000).contains(&moved), "20 ppm of 145 MHz moved {moved} Hz");
        assert_eq!(untuned(hw, 20.0), want);
        assert_eq!(untuned(tuned(want, -7.5), -7.5), want);
        assert_eq!(tuned(want, 0.0), want);
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
            volume: 1.0,
            muted: false,
            squelch_db: None,
            agc: true,
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
        common::IqBuf { samples: vec![C32::default(); 1024], rate: Sps(rate as u64), center, seq: 0 }
    }

    #[test]
    fn each_bank_splits_the_span_to_the_width_its_front_end_wants() {
        for rate in [250_000.0, 1_024_000.0, 2_400_000.0, 20_000_000.0] {
            for (want, lo, hi) in [
                (12_500.0, 6_000.0, 30_000.0),
                (OOK_CHANNEL_HZ, 15_000.0, 70_000.0),
                (FSK_CHANNEL_HZ, 60_000.0, 260_000.0),
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
        assert!(rx.bank_channels().is_empty(), "a bank tier is still running: {:?}", rx.bank_channels());
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
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!("../../testdata/offair/{name}"));
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
                .map(|r| format!("{:.4} MHz {} {}", r.freq / 1e6, r.model, r.detail))
                .collect();
            let r = out
                .iter()
                .find(|r| r.model == "Meshtastic")
                .unwrap_or_else(|| panic!("capture {which}: nothing read it: {rows:?}"));
            // The transmitter's CRC, not a plausibility argument.
            assert_eq!(r.crc, Some(true), "capture {which}: {r:?}");
            assert!(
                r.detail.contains("SF11 BW250k 4/5"),
                "capture {which}: read as {}",
                r.detail
            );
            // Both captures are from a node addressing the whole mesh.
            assert!(
                r.detail.contains("to everyone"),
                "capture {which}: read as {}",
                r.detail
            );
            let hz = r.freq;
            assert!(
                (hz - 869_525_000.0).abs() < 250_000.0,
                "capture {which}: read at {hz} Hz"
            );
            if which == 'c' {
                // What the node said, against the public default key.
                assert!(r.detail.contains("050d3664 to everyone"), "capture c: {}", r.detail);
                assert!(r.detail.contains("\"Hi\""), "capture c: {}", r.detail);
            }
        }
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
        let mut seen: Vec<(f64, f32)> = Vec::new();
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
        let second = common::IqBuf::new(buf.samples[half..].to_vec(), buf.center, buf.rate, half as u64);
        let mut out = replay_blocks(&mut rx, &first);
        rx.rebuild(&plan).unwrap();
        out.extend(replay_blocks(&mut rx, &second));
        let rows: Vec<String> = out
            .iter()
            .map(|r| format!("{:.4} MHz {} {} {}", r.freq / 1e6, r.model, r.modulation, r.detail))
            .collect();
        for hz in [391_181_000.0, 391_704_500.0] {
            let mine: Vec<&DecodeRecord> =
                out.iter().filter(|r| (r.freq - hz).abs() < 12_500.0).collect();
            assert!(!mine.is_empty(), "{:.4} MHz was never logged: {rows:?}", hz / 1e6);
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
            let sync = mine.iter().filter(|r| r.model == "TETRA-Sync").count();
            let sysinfo = mine.iter().filter(|r| r.model == "TETRA-Sysinfo").count();
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
            let network: Vec<&&DecodeRecord> = mine.iter().filter(|r| r.model == "TETRA-Network").collect();
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
            let measured: Vec<&&DecodeRecord> = mine.iter().filter(|r| r.model == "unknown").collect();
            assert!(
                measured.len() <= 1,
                "{:.4} MHz measured {} times while being read: {rows:?}",
                hz / 1e6,
                measured.len()
            );
            assert!(
                measured.iter().all(|r| r.modulation == "pi/4-DQPSK" && r.detail.contains("TETRA")),
                "{:.4} MHz was measured as {:?}",
                hz / 1e6,
                measured.iter().map(|r| format!("{} {}", r.modulation, r.detail)).collect::<Vec<_>>()
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
        let calls: Vec<&DecodeRecord> = out.iter().filter(|r| r.model == "TETRA-Call").collect();
        assert!(!calls.is_empty(), "no call rows from {} rows", out.len());
        let field = |r: &DecodeRecord, k: &str| {
            r.fields.iter().find(|(n, _)| n == k).map(|(_, v)| v.to_string())
        };
        let groups: Vec<String> = calls.iter().filter_map(|r| field(r, "to")).collect();
        assert!(
            groups.iter().any(|g| g == "10223295" || g == "15835885"),
            "addressed {groups:?}"
        );
        assert!(
            calls.iter().all(|r| field(r, "encryption").as_deref() == Some("AIE-3")),
            "{:?}",
            calls.iter().map(|r| format!("{:?} {:?} {:?} {:?}", field(r, "pdu"), field(r, "to"), field(r, "encryption"), r.detail)).collect::<Vec<_>>()
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

        let m17: Vec<&DecodeRecord> = out.iter().filter(|r| r.model.starts_with("M17")).collect();
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
        let voice = m17.iter().filter(|r| r.model == "M17-Voice").count();
        assert!(voice >= 20, "only {voice} voice frames of a 2.5 second over");
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
            volume: 1.0,
            muted: false,
            squelch_db: None,
            agc: true,
        }];
        let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a channel");
        let out = replay_blocks(&mut rx, &buf);

        let m17: Vec<&DecodeRecord> = out.iter().filter(|r| r.model.starts_with("M17")).collect();
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
        let bus = rx.audio_mut().expect("the bus is always there");
        bus.bus_mut().set_subscriptions(vec![crate::audiobus::Subscription::new(
            crate::audiobus::Rule::Everything,
        )]);
        bus.bus_mut().set_master(1.0, false);

        let mut pcm: Vec<f32> = Vec::new();
        let mut heard = None;
        for block in buf.samples.chunks(16_384) {
            if rx.process(block).is_err() {
                break;
            }
            // One side of the stereo mix, which carries speech on both.
            pcm.extend(rx.audio_out().0.iter().step_by(2));
            heard = heard.or_else(|| {
                rx.audio().and_then(|c| c.bus().last_heard()).map(str::to_string)
            });
        }
        assert_eq!(heard.as_deref(), Some("OPNRTX to BROADCAST"), "nobody was heard");
        // The bus resamples to its output rate, and the over is seconds
        // long. Half a second of it is enough to say the vocoder ran on live
        // frames and the mix reached the far end.
        let seconds = pcm.len() as f64 / rx.audio().unwrap().bus().out_rate();
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
            .find(|r| r.model == "Fineoffset-WHx080")
            .unwrap_or_else(|| panic!("only unknowns: {out:?}"));
        assert_eq!(r.crc, Some(true), "{r:?}");
        assert!(r.detail.contains("temperature_c=16.2"), "{}", r.detail);
        // Structured, not just printed: a chart or a map has to be able to
        // read a field without parsing the summary line back apart.
        assert_eq!(
            r.fields.iter().find(|(k, _)| k == "temperature_c").map(|(_, v)| v.as_f64()),
            Some(Some(16.2))
        );
        assert_eq!(r.modulation, "OOK");
        // A real reception from a recording made near full scale: strong, and
        // well clear of the noise.
        assert!(r.snr_db > 6.0, "snr came out as {}", r.snr_db);
        // Referenced to full scale at the detector, so filter gain can put a
        // very strong packet slightly over zero. What matters is that it is a
        // real measurement rather than a placeholder.
        assert!(
            (-60.0..=6.0).contains(&r.rssi_dbfs),
            "rssi came out as {} dB",
            r.rssi_dbfs
        );
        // One row, not five: the FSK branch reads the same burst and the
        // neighbouring channels see its skirts, and all of that is one packet.
        assert_eq!(out.len(), 1, "the same burst was logged more than once: {out:#?}");
        // The frequency reported is the channel's, not the tuner's, which is
        // what makes a waterfall mark land on the signal.
        let off = (r.freq - buf.center.as_f64()).abs();
        assert!(off < buf.rate.as_f64() / 2.0, "{} Hz is outside the span", r.freq);
    }

    fn rec(freq: f64, model: &str, rssi: f32) -> DecodeRecord {
        DecodeRecord {
            at: std::time::Instant::now(),
            freq,
            model: model.into(),
            channel_hz: 125_000.0,
            modulation: "FSK",
            detail: String::new(),
            fields: Vec::new(),
            media_type: pipeline::event::media::BYTES,
            rssi_dbfs: rssi,
            snr_db: 20.0,
            bytes: vec![1, 2, 3],
            crc: None,
            iq: None,
            audio: None,
        }
    }

    #[test]
    fn the_same_burst_seen_by_two_channels_is_reported_once() {
        // Channels overlap, so a strong transmitter is genuinely present in
        // its neighbours, where the detectors read a mangled copy of it. The
        // loudest reading wins and the skirts are dropped.
        let w = 125_000.0;
        let mut block = vec![
            rec(868_100_000.0, "unknown", -54.0),
            rec(868_100_000.0 + w, "unknown", -38.0),
            rec(868_100_000.0 - w, "unknown", -61.0),
        ];
        assert_eq!(block[0].modulation, "FSK");
        dedupe_neighbours(&mut block);
        let kept: Vec<&DecodeRecord> = block.iter().filter(|r| !r.model.is_empty()).collect();
        assert_eq!(kept.len(), 1, "kept {kept:#?}");
        assert_eq!(kept[0].rssi_dbfs, -38.0, "the strongest reading should win");
    }

    #[test]
    fn a_real_decode_beats_a_louder_guess() {
        let mut block = vec![
            rec(868_100_000.0, "unknown", -20.0),
            rec(868_100_000.0 + 125_000.0, "Fineoffset-WHx080", -44.0),
        ];
        dedupe_neighbours(&mut block);
        let kept: Vec<&DecodeRecord> = block.iter().filter(|r| !r.model.is_empty()).collect();
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].model, "Fineoffset-WHx080", "a CRC beats a stronger guess");
    }

    #[test]
    fn two_devices_far_apart_are_both_kept() {
        let mut block =
            vec![rec(868_100_000.0, "unknown", -40.0), rec(869_000_000.0, "unknown", -50.0)];
        dedupe_neighbours(&mut block);
        assert_eq!(block.iter().filter(|r| !r.model.is_empty()).count(), 2);
    }

    #[test]
    fn a_device_that_repeats_its_packet_is_logged_every_time() {
        // Two bursts on one channel through one front end are two
        // transmissions, not one seen twice, and a sensor that sends its
        // reading three times should show three rows.
        let mut block = vec![
            rec(868_100_000.0, "unknown", -40.0),
            rec(868_100_000.0, "unknown", -41.0),
        ];
        dedupe_neighbours(&mut block);
        assert_eq!(block.iter().filter(|r| !r.model.is_empty()).count(), 2);
    }

    #[test]
    fn one_burst_read_by_both_front_ends_is_logged_once() {
        // The OOK and FSK branches see the same channel, so a burst can be
        // decoded by one and guessed at by the other. That is one packet.
        let mut ook = rec(868_100_000.0, "Fineoffset-WHx080", -44.0);
        ook.modulation = "OOK";
        let mut block = vec![rec(868_100_000.0, "unknown", -30.0), ook];
        dedupe_neighbours(&mut block);
        let kept: Vec<&DecodeRecord> = block.iter().filter(|r| !r.model.is_empty()).collect();
        assert_eq!(kept.len(), 1);
        assert_eq!(kept[0].model, "Fineoffset-WHx080");
    }

    fn ook_at(freq: f64, at: std::time::Instant) -> DecodeRecord {
        let mut r = rec(freq, "unknown", -30.0);
        r.channel_hz = OOK_CHANNEL_HZ;
        r.modulation = "OOK";
        r.at = at;
        r
    }

    #[test]
    fn a_burst_split_across_two_blocks_is_still_reported_once() {
        // Observed on live 868 MHz traffic: one transmission arrived as four
        // rows 31 kHz apart, because reads from the radio are milliseconds
        // long and each was deduped alone.
        let mut sc = Dedupe::default();
        let t0 = std::time::Instant::now();
        let block = std::time::Duration::from_millis(7);
        let mut kept = 0;
        for (n, freq) in [868_362_300.0, 868_393_400.0, 868_331_100.0].iter().enumerate() {
            let at = t0 + block * n as u32;
            if sc.accept(&ook_at(*freq, at), at) {
                kept += 1;
            }
        }
        assert_eq!(kept, 1, "one burst logged as {kept} rows");
    }

    #[test]
    fn a_device_repeating_on_its_own_channel_is_logged_every_time() {
        // Same channel through the same front end is a second transmission,
        // not a second reading of the first, and a sensor that sends its
        // packet three times should show three rows.
        let mut sc = Dedupe::default();
        let t0 = std::time::Instant::now();
        for n in 0..3u32 {
            let at = t0 + std::time::Duration::from_millis(60) * n;
            assert!(sc.accept(&ook_at(868_362_300.0, at), at), "repeat {n} was swallowed");
        }
    }

    #[test]
    fn a_neighbour_is_only_a_duplicate_while_the_burst_is_recent() {
        let mut sc = Dedupe::default();
        let t0 = std::time::Instant::now();
        assert!(sc.accept(&ook_at(868_362_300.0, t0), t0));

        let soon = t0 + std::time::Duration::from_millis(50);
        assert!(!sc.accept(&ook_at(868_393_400.0, soon), soon), "a skirt slipped through");

        // Long enough later and it is a different burst that happens to be
        // next door, which is the whole reason the memory expires.
        let later = t0 + DEDUPE_WINDOW + std::time::Duration::from_millis(10);
        assert!(sc.accept(&ook_at(868_393_400.0, later), later), "the memory never expired");
    }

    #[test]
    fn the_dedupe_memory_is_shorter_than_a_repeating_device() {
        // Long enough to cover a block boundary, short enough that a sensor
        // repeating its packet two or three times a second still gets a row
        // per repeat.
        assert!(DEDUPE_WINDOW >= std::time::Duration::from_millis(250));
        assert!(DEDUPE_WINDOW <= std::time::Duration::from_millis(400));
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
        let rate = 2_400_000.0;
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
        let cost = |a: &mut Audio| {
            let t = std::time::Instant::now();
            for _ in 0..10 {
                a.process(&b, 0.5);
            }
            t.elapsed().as_secs_f64()
        };
        let first = cost(&mut a);
        for _ in 0..5 {
            cost(&mut a);
        }
        let later = cost(&mut a);
        assert!(
            later < first * 3.0,
            "cost per block climbed from {first:.4}s to {later:.4}s"
        );
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
            volume: 1.0,
            muted: false,
            squelch_db: Some(-200.0),
            agc: false,
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
}

