//! The audio bus: everything that reaches the speaker, in one place.
//!
//! Every listening channel and every voice front end ends here, and what an
//! operator hears is decided here and nowhere else. The alternative, which
//! this replaces, was a sum in the radio loop: the faders, the master, the
//! clip and the meters lived outside the graph, so a demodulator drawn by
//! hand had nothing to be wired to and a channel the strip could not name
//! was silent.
//!
//! # Shape
//!
//! One input per strip. A real input is an analog channel, mono or stereo,
//! at whatever rate its chain left it: it is resampled, given the strip's
//! level and mute, and summed. A voice input carries speech as it is
//! decoded, each block labelled with who is talking and to whom; those are
//! matched against subscriptions rather than heard whole, because one voice
//! port carries every conversation on a system and "play whatever decoded
//! last" is not a receiver anybody can use. Anything unmatched is dropped
//! rather than mixed quietly: an operator listening to one group must be
//! able to trust that what they hear is that group.
//!
//! The last input is always spare and fed by nothing. That is what a chain
//! drawn by hand is wired into, and the receiver draws a new spare once it
//! is taken.
//!
//! A transmission replayed from the packet log goes through the same bus,
//! because it is the same audio path and the master level should mean the
//! same thing for both.

use common::{Error, Result, Speech};
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;

/// The rate the speaker is fed at, and every strip is brought to.
pub const OUT_HZ: f64 = 48_000.0;

/// Most the gain control will lift a transmission, in decibels.
///
/// A vocoder's output level is whatever the transmitting radio's microphone
/// gain was, and that is not something a listener can fix at the far end:
/// measured on real M17 traffic here, speech peaked at -37 dBFS and averaged
/// -57, which is inaudible once the call and master levels have had their
/// share. Thirty decibels covers a handheld set low. More than that and the
/// vocoder's own noise between words comes up with the speech.
const MAX_GAIN_DB: f32 = 30.0;

/// What a subscription matches on.
///
/// Deliberately data rather than a closure: the set is edited in the
/// interface, saved in the session, and has to be comparable so a rebuild can
/// tell whether anything changed.
#[derive(Clone, Debug, PartialEq)]
pub enum Rule {
    /// Everything any source decodes.
    Everything,
    /// One talkgroup, reflector or destination, whatever the system calls it.
    Group(String),
    /// One caller, wherever they transmit.
    Caller(String),
    /// Whatever is heard on one channel, to within its own width.
    Channel(f64),
    /// One system: every M17 call, every DMR call.
    System(String),
}

impl Rule {
    /// Whether this rule covers a transmission.
    pub fn matches(&self, v: &Voice) -> bool {
        match self {
            Rule::Everything => true,
            // Case-insensitive because a callsign is written both ways and
            // nobody means a different aircraft by it.
            Rule::Group(g) => v.to.eq_ignore_ascii_case(g),
            Rule::Caller(c) => v.from.as_deref().is_some_and(|f| f.eq_ignore_ascii_case(c)),
            // Half a kilohertz, which is a rounding rather than a channel:
            // the narrowest grid anything here uses is 12.5 kHz.
            Rule::Channel(hz) => (v.channel_hz - hz).abs() < 500.0,
            Rule::System(s) => v.system.eq_ignore_ascii_case(s),
        }
    }

    /// How the interface writes it.
    pub fn label(&self) -> String {
        match self {
            Rule::Everything => "everything".into(),
            Rule::Group(g) => format!("group {g}"),
            Rule::Caller(c) => format!("caller {c}"),
            Rule::Channel(hz) => format!("channel {:.4} MHz", hz / 1e6),
            Rule::System(s) => format!("system {s}"),
        }
    }
}

/// One standing instruction: what to listen to, and how loudly.
#[derive(Clone, Debug, PartialEq)]
pub struct Subscription {
    pub rule: Rule,
    pub volume: f32,
    pub muted: bool,
}

impl Subscription {
    pub fn new(rule: Rule) -> Self {
        Self { rule, volume: 0.8, muted: false }
    }

    fn gain(&self) -> f32 {
        if self.muted {
            0.0
        } else {
            self.volume.clamp(0.0, 2.0)
        }
    }
}

/// A block of speech from one source, as it was decoded.
#[derive(Clone, Debug, PartialEq)]
pub struct Voice<'a> {
    /// The system it came from, which is what a `System` rule names.
    pub system: &'a str,
    pub channel_hz: f64,
    /// The group or party being called.
    pub to: &'a str,
    /// Who is talking, when the system says.
    pub from: Option<&'a str>,
    pub pcm: &'a [f32],
    pub rate: f64,
}

/// What is wired into one input, as the graph negotiated it.
enum Feed {
    /// Nothing: the spare input, or a wire not yet drawn.
    Silent,
    /// Real audio at some rate, mono or stereo, with a resampler per side
    /// when the rate is not the speaker's.
    Audio { channels: usize, rs: Vec<audio::Resampler> },
    /// Speech with its labels, matched by subscription.
    Voice,
}

/// One input to the bus: a level and a mute for whatever is wired in, and a
/// meter reading what it put into the mix.
pub struct Strip {
    pub label: String,
    pub volume: f32,
    pub muted: bool,
    /// Peak this block after the fader, for the meter beside it.
    pub peak: f32,
    feed: Feed,
}

impl Strip {
    fn new() -> Self {
        Self { label: String::new(), volume: 0.8, muted: false, peak: 0.0, feed: Feed::Silent }
    }

    fn gain(&self) -> f32 {
        if self.muted {
            0.0
        } else {
            self.volume.clamp(0.0, 1.0)
        }
    }

    /// Whether this input carries speech rather than audio.
    pub fn is_voice(&self) -> bool {
        matches!(self.feed, Feed::Voice)
    }

    /// Whether anything is wired in at all.
    pub fn is_fed(&self) -> bool {
        !matches!(self.feed, Feed::Silent)
    }
}

/// Mixes every strip and every subscribed call, at the rate the speaker
/// wants, in stereo.
pub struct AudioBus {
    out_rate: f64,
    strips: Vec<Strip>,
    subs: Vec<Subscription>,
    /// The level the whole mix leaves at, and whether it leaves at all.
    master: f32,
    muted: bool,
    /// One level for every call, beside the master: a call is not a channel
    /// anybody tuned, it is whatever the front ends decode, and mixing it
    /// belongs where every other level in the receiver is set.
    calls: f32,
    calls_muted: bool,
    /// Peak per voice source since the last block, for a meter on the row it
    /// belongs to. Decayed rather than reset, so a meter tracks speech
    /// instead of flickering with every syllable.
    peaks: HashMap<String, f32>,
    /// One resampler per voice source, because each carries filter state
    /// and two sources at the same rate are still two different streams.
    rs: HashMap<String, audio::Resampler>,
    /// This block's speech, mono at `out_rate`, before it joins the mix.
    voice: Vec<f32>,
    /// This block's mix, stereo interleaved at `out_rate`.
    mix: Vec<f32>,
    scratch: Vec<f32>,
    lane: Vec<f32>,
    lane_out: Vec<f32>,
    /// A transmission being replayed, already at `out_rate`.
    replay: std::collections::VecDeque<f32>,
    /// The gain control every call passes through, and whether it is on.
    ///
    /// The same [`dsp::agc::Agc`] a listening channel uses, with the same voice
    /// constants: attack fast enough that a loud caller cannot blast, release
    /// slow enough that the gain does not climb audibly between words, and a
    /// hang time so a pause is not treated as a fade. One instance for the
    /// bus rather than one per source, because what it is levelling is the
    /// output somebody is listening to.
    agc: dsp::agc::Agc,
    agc_on: bool,
    /// What was last heard through the bus, for the interface to show.
    last: Option<String>,
    /// Peak of the speech share of this block's mix.
    voice_peak: f32,
}

impl AudioBus {
    pub fn new(out_rate: f64) -> Self {
        Self {
            out_rate,
            strips: vec![Strip::new()],
            subs: Vec::new(),
            master: 0.5,
            muted: false,
            calls: 0.8,
            calls_muted: false,
            peaks: HashMap::new(),
            rs: HashMap::new(),
            voice: Vec::new(),
            mix: Vec::new(),
            scratch: Vec::new(),
            lane: Vec::new(),
            lane_out: Vec::new(),
            replay: std::collections::VecDeque::new(),
            agc: {
                let mut a = dsp::agc::Agc::voice(out_rate);
                a.set_max_gain_db(MAX_GAIN_DB);
                a
            },
            agc_on: true,
            last: None,
            voice_peak: 0.0,
        }
    }

    pub fn out_rate(&self) -> f64 {
        self.out_rate
    }

    pub fn strips(&self) -> &[Strip] {
        &self.strips
    }

    pub fn strip_mut(&mut self, k: usize) -> Option<&mut Strip> {
        self.strips.get_mut(k)
    }

    /// Change how many inputs there are, keeping the levels of the ones that
    /// stay: the set of chains feeding the bus changes with every retune,
    /// and a fader that reset each time would be no fader at all.
    pub fn set_inputs(&mut self, n: usize) {
        self.strips.resize_with(n.max(1), Strip::new);
    }

    pub fn subscriptions(&self) -> &[Subscription] {
        &self.subs
    }

    pub fn set_subscriptions(&mut self, subs: Vec<Subscription>) {
        self.subs = subs;
    }

    /// Whether the gain control is levelling calls.
    pub fn set_agc(&mut self, on: bool) {
        if on != self.agc_on {
            self.agc.reset();
        }
        self.agc_on = on;
    }

    pub fn agc_on(&self) -> bool {
        self.agc_on
    }

    /// What the gain control is adding right now, in decibels.
    pub fn agc_gain_db(&self) -> f32 {
        if self.agc_on {
            self.agc.gain_db()
        } else {
            0.0
        }
    }

    /// The level the whole mix leaves at, and whether it leaves at all.
    pub fn set_master(&mut self, volume: f32, muted: bool) {
        self.master = volume.clamp(0.0, 1.0);
        self.muted = muted;
    }

    pub fn master(&self) -> (f32, bool) {
        (self.master, self.muted)
    }

    /// The level every subscribed call is heard at, and whether any is.
    pub fn set_calls(&mut self, volume: f32, muted: bool) {
        self.calls = volume.clamp(0.0, 1.0);
        self.calls_muted = muted;
    }

    pub fn calls(&self) -> (f32, bool) {
        (self.calls, self.calls_muted)
    }

    /// Whether any call at all is being listened to, which decides whether a
    /// source needs to do the work of decoding speech.
    pub fn listening(&self) -> bool {
        !self.muted && !self.calls_muted && self.subs.iter().any(|s| !s.muted)
    }

    /// What each voice source put into the mix last block, keyed as
    /// `system:channel`, for a meter on its row.
    pub fn levels(&self) -> Vec<(String, f32)> {
        self.peaks.iter().map(|(k, v)| (k.clone(), *v)).collect()
    }

    /// The key a voice source's meter is filed under.
    pub fn key_of(system: &str, channel_hz: f64) -> String {
        format!("{system}:{channel_hz:.0}")
    }

    /// What the subscriptions say about one transmission: the gain to mix it
    /// at, before the master, or `None` to ignore it.
    ///
    /// The loudest matching subscription wins rather than their sum, so
    /// covering one group twice does not make it twice as loud.
    pub fn gain_for(&self, v: &Voice) -> Option<f32> {
        if self.muted || self.calls_muted {
            return None;
        }
        self.subs
            .iter()
            .filter(|s| s.rule.matches(v))
            .map(|s| s.gain() * self.calls)
            .fold(None, |acc, g| Some(acc.map_or(g, |a: f32| a.max(g))))
    }

    /// Publish a block of speech. Returns whether any of it was mixed.
    pub fn push(&mut self, v: Voice<'_>) -> bool {
        let Some(gain) = self.gain_for(&v) else { return false };
        if v.pcm.is_empty() || gain <= 0.0 {
            return false;
        }
        let key = Self::key_of(v.system, v.channel_hz);
        let rs = self
            .rs
            .entry(key.clone())
            .or_insert_with(|| audio::Resampler::new(v.rate, self.out_rate, 4));
        self.scratch.clear();
        rs.process(v.pcm, &mut self.scratch);

        // Levelled before the subscription's own volume, so what an operator
        // sets is a level relative to other calls rather than a fight with
        // whoever transmitted loudest.
        if self.agc_on {
            self.agc.process(&mut self.scratch);
        }
        if self.voice.len() < self.scratch.len() {
            self.voice.resize(self.scratch.len(), 0.0);
        }
        for (m, s) in self.voice.iter_mut().zip(self.scratch.iter()) {
            *m += s * gain;
        }
        let peak = self.scratch.iter().fold(0.0f32, |a, s| a.max((s * gain).abs()));
        let e = self.peaks.entry(key).or_insert(0.0);
        *e = e.max(peak);
        self.last = Some(match v.from {
            Some(f) => format!("{f} to {}", v.to),
            None => v.to.to_string(),
        });
        true
    }

    /// Put one strip's audio into the mix at its own level.
    ///
    /// `pcm` is interleaved with as many channels as the strip was
    /// negotiated with; a mono strip is heard on both sides, which is what a
    /// receiver does with a mono station anyway, and it means a broadcast in
    /// stereo can share the output with a narrowband channel that has no
    /// such thing.
    pub fn feed(&mut self, k: usize, pcm: &[f32]) {
        let Some(strip) = self.strips.get_mut(k) else { return };
        let gain = strip.gain();
        let Feed::Audio { channels, rs } = &mut strip.feed else { return };
        let ch = (*channels).max(1);
        if pcm.is_empty() {
            return;
        }
        // Brought to the speaker's rate first. Two channels at slightly
        // different rates summed sample for sample play one of them at the
        // wrong pitch, which is what happened when this was a loop in the
        // radio thread that took the last channel's rate for all of them.
        let frames = if rs.is_empty() {
            self.scratch.clear();
            self.scratch.extend_from_slice(pcm);
            pcm.len() / ch
        } else {
            let mut frames = 0;
            self.scratch.clear();
            for (side, r) in rs.iter_mut().enumerate() {
                self.lane.clear();
                self.lane.extend(pcm.iter().skip(side).step_by(ch).copied());
                self.lane_out.clear();
                r.process(&self.lane, &mut self.lane_out);
                if side == 0 {
                    frames = self.lane_out.len();
                    self.scratch.resize(frames * ch, 0.0);
                }
                for (f, v) in self.lane_out.iter().take(frames).enumerate() {
                    self.scratch[f * ch + side] = *v;
                }
            }
            frames
        };
        if self.mix.len() < frames * 2 {
            self.mix.resize(frames * 2, 0.0);
        }
        let mut peak = 0.0f32;
        for f in 0..frames {
            let (l, r) = if ch >= 2 {
                (self.scratch[f * ch], self.scratch[f * ch + 1])
            } else {
                (self.scratch[f], self.scratch[f])
            };
            let (l, r) = (l * gain, r * gain);
            self.mix[f * 2] += l;
            self.mix[f * 2 + 1] += r;
            peak = peak.max(l.abs()).max(r.abs());
        }
        strip.peak = strip.peak.max(peak);
    }

    /// Queue a decoded transmission to be played once, replacing whatever was
    /// already playing: two at a time is noise, not a review.
    pub fn play(&mut self, speech: &Arc<Speech>) {
        let mut rs = audio::Resampler::new(speech.rate, self.out_rate, 4);
        let mut out = Vec::with_capacity(speech.pcm.len() * 8);
        rs.process(&speech.pcm, &mut out);
        self.replay.clear();
        self.replay.extend(out);
    }

    pub fn stop_replay(&mut self) {
        self.replay.clear();
    }

    pub fn replaying(&self) -> bool {
        !self.replay.is_empty()
    }

    /// Seconds of replay left to play.
    pub fn replay_left(&self) -> f64 {
        self.replay.len() as f64 / self.out_rate.max(1.0)
    }

    pub fn last_heard(&self) -> Option<&str> {
        self.last.as_deref()
    }

    /// Peak of the speech share of the last mix, after the call level.
    pub fn voice_peak(&self) -> f32 {
        self.voice_peak
    }

    /// This block's audio: every strip, every subscribed call and a slice
    /// of any replay, held inside full scale, as stereo at the output rate.
    ///
    /// The master level and mute are not applied here. They are the node's
    /// parameters and the strip still sets them here, but what they govern is
    /// the sound card: applied to the mix instead, a mute took a queue length
    /// to be heard and left anything wired downstream of the bus playing, and
    /// a recording made from a tap would carry the listener's volume setting.
    /// [`crate::radio`] reads them off the node and drives the sink with them.
    ///
    /// `frames` is what the block is worth in time at the output rate, so a
    /// replay runs at real time rather than arriving all at once. Zero means
    /// nothing else is producing audio, and the replay sets the pace itself.
    ///
    /// Clipped rather than scaled to fit: several channels at once can sum
    /// past full scale, and quietly turning everything down would make the
    /// level of the channel being listened to depend on how busy its
    /// neighbours are.
    pub fn render(&mut self, frames: usize) -> &[f32] {
        if !self.replay.is_empty() {
            let want = if frames > 0 { frames } else { (self.out_rate / 50.0) as usize };
            let take = want.min(self.replay.len());
            if self.voice.len() < take {
                self.voice.resize(take, 0.0);
            }
            for (i, s) in self.replay.drain(..take).enumerate() {
                self.voice[i] += s;
            }
        }
        self.voice_peak = self.voice.iter().fold(0.0f32, |a, v| a.max(v.abs()));
        if self.mix.len() < self.voice.len() * 2 {
            self.mix.resize(self.voice.len() * 2, 0.0);
        }
        for (i, v) in self.voice.iter().enumerate() {
            self.mix[i * 2] += v;
            self.mix[i * 2 + 1] += v;
        }
        for v in self.mix.iter_mut() {
            *v = v.clamp(-1.0, 1.0);
        }
        &self.mix
    }

    /// Drop this block's audio, once it has been taken, and let the meters
    /// fall back towards zero.
    pub fn clear(&mut self) {
        self.mix.clear();
        self.voice.clear();
        for s in &mut self.strips {
            s.peak *= 0.7;
            if s.peak < 0.002 {
                s.peak = 0.0;
            }
        }
        for v in self.peaks.values_mut() {
            *v *= 0.7;
        }
        self.peaks.retain(|_, v| *v > 0.002);
    }
}

/// Write speech to a 16 bit WAV, which is what everything else can open.
///
/// Voice is kept out of the packet log deliberately: an hour of a busy
/// repeater is gigabytes and the log is a record of what was on the air, not
/// of what it sounded like. A transmission worth keeping is worth a file of
/// its own, and a file is what a spectrogram, a player or another decoder can
/// be pointed at.
pub fn write_wav(path: &Path, speech: &Speech) -> std::io::Result<()> {
    use std::io::Write;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let rate = speech.rate.max(1.0) as u32;
    let n = speech.pcm.len() as u32;
    let data_len = n * 2;
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    f.write_all(b"RIFF")?;
    f.write_all(&(36 + data_len).to_le_bytes())?;
    f.write_all(b"WAVEfmt ")?;
    f.write_all(&16u32.to_le_bytes())?;
    f.write_all(&1u16.to_le_bytes())?; // PCM
    f.write_all(&1u16.to_le_bytes())?; // mono
    f.write_all(&rate.to_le_bytes())?;
    f.write_all(&(rate * 2).to_le_bytes())?; // bytes per second
    f.write_all(&2u16.to_le_bytes())?; // block align
    f.write_all(&16u16.to_le_bytes())?;
    f.write_all(b"data")?;
    f.write_all(&data_len.to_le_bytes())?;
    for s in &speech.pcm {
        let v = (s.clamp(-1.0, 1.0) * 32767.0) as i16;
        f.write_all(&v.to_le_bytes())?;
    }
    f.flush()
}

/// Peak and RMS of a transmission, in dBFS.
///
/// Shown beside the replay button, because "I can hear nothing" has two
/// causes that look identical from the speaker: nothing was decoded, or it
/// was decoded quietly and something later in the path lost it.
pub fn levels_db(speech: &Speech) -> (f32, f32) {
    let peak = speech.pcm.iter().fold(0.0f32, |a, v| a.max(v.abs()));
    let rms = (speech.pcm.iter().map(|v| v * v).sum::<f32>()
        / speech.pcm.len().max(1) as f32)
        .sqrt();
    let db = |v: f32| if v > 0.0 { 20.0 * v.log10() } else { -120.0 };
    (db(peak), db(rms))
}

/// The bus as a node, which is the only way it is ever built.
///
/// One input per strip and one output carrying the mix, so the whole audio
/// path is drawn like everything else the receiver does. Every level on it
/// is a parameter, which is what lets the chain view set it, a patch save
/// it, and the strip read it back.
pub struct AudioBusNode {
    bus: AudioBus,
}

impl AudioBusNode {
    pub fn new(out_rate: f64) -> Self {
        Self { bus: AudioBus::new(out_rate) }
    }

    pub fn bus(&self) -> &AudioBus {
        &self.bus
    }

    pub fn bus_mut(&mut self) -> &mut AudioBus {
        &mut self.bus
    }

    /// The name of one strip's parameter, as the patch and the chain view
    /// write it.
    pub fn param_of(k: usize, what: &str) -> String {
        format!("{what}{k}")
    }

    /// Split a per-strip parameter name into what it sets and which strip.
    fn per_strip(name: &str) -> Option<(&str, usize)> {
        for what in ["vol", "mute", "label"] {
            if let Some(k) = name.strip_prefix(what).and_then(|k| k.parse().ok()) {
                return Some((what, k));
            }
        }
        None
    }
}

impl pipeline::node::Node for AudioBusNode {
    fn name(&self) -> &str {
        "audio_bus"
    }

    fn as_any(&self) -> Option<&dyn std::any::Any> {
        Some(self)
    }

    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(self)
    }

    /// So a rebuild carries the bus across rather than dropping what it was
    /// subscribed to, its levels and whatever it was playing.
    fn into_any(self: Box<Self>) -> Option<Box<dyn std::any::Any>> {
        Some(self)
    }

    fn num_inputs(&self) -> usize {
        self.bus.strips.len().max(1)
    }

    /// A mixer has a spare input by nature.
    fn optional_inputs(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let out_rate = self.bus.out_rate;
        for (k, i) in inputs.iter().enumerate() {
            let feed = match i.spec.kind {
                PortKind::Voice => Feed::Voice,
                PortKind::Real if i.spec.is_silence() => Feed::Silent,
                PortKind::Real => {
                    let ch = i.spec.channels.max(1);
                    let rate = i.spec.frame_rate();
                    let rs = if (rate - out_rate).abs() < 0.5 {
                        Vec::new()
                    } else {
                        (0..ch).map(|_| audio::Resampler::new(rate, out_rate, 8)).collect()
                    };
                    Feed::Audio { channels: ch, rs }
                }
                other => {
                    return Err(Error::other(format!(
                        "the audio bus takes audio or speech, and input {k} carries {other:?}"
                    )))
                }
            };
            if let Some(s) = self.bus.strips.get_mut(k) {
                s.feed = feed;
            }
        }
        Ok(vec![StreamSpec {
            kind: PortKind::Real,
            rate: out_rate * 2.0,
            center: common::Hz(0),
            bandwidth: 0.0,
            channels: 2,
        }])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        for (k, p) in inputs.iter().enumerate() {
            match p {
                Payload::Voice(voices) => {
                    for v in voices {
                        let Some(to) = v.to.as_deref() else { continue };
                        if v.pcm.is_empty() {
                            continue;
                        }
                        self.bus.push(Voice {
                            system: v.system,
                            channel_hz: v.channel_hz,
                            to,
                            from: v.from.as_deref(),
                            pcm: &v.pcm,
                            rate: v.rate,
                        });
                    }
                }
                Payload::Real(pcm) => self.bus.feed(k, pcm),
                _ => {}
            }
        }
        // What this block is worth in audio, from the run's own clock.
        let frames = (ctx.block_seconds * self.bus.out_rate()).round() as usize;
        let out = outputs[0].real_mut();
        out.extend_from_slice(self.bus.render(frames));
        self.bus.clear();
        Ok(())
    }

    fn reset(&mut self) {
        self.bus.stop_replay();
        self.bus.clear();
    }

    fn params(&self) -> Vec<Param> {
        let mut p = vec![
            Param::float("master", self.bus.master as f64, 0.0..=1.0).label("Master"),
            Param::bool("muted", self.bus.muted).label("Muted"),
            Param::float("calls", self.bus.calls as f64, 0.0..=1.0).label("Calls"),
            Param::bool("calls_muted", self.bus.calls_muted).label("Calls muted"),
            Param::bool("agc", self.bus.agc_on).label("Call AGC"),
        ];
        for (k, s) in self.bus.strips.iter().enumerate() {
            if !s.is_fed() {
                continue;
            }
            let name = if s.label.is_empty() { format!("input {k}") } else { s.label.clone() };
            p.push(
                Param::float(&Self::param_of(k, "vol"), s.volume as f64, 0.0..=1.0)
                    .label(&format!("{name} level")),
            );
            p.push(Param::bool(&Self::param_of(k, "mute"), s.muted).label(&format!("{name} mute")));
        }
        p
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let num = |v: &ParamValue| {
            v.as_f64().map(|f| f as f32).ok_or_else(|| Error::other("expected a number"))
        };
        let flag = |v: &ParamValue| v.as_bool().ok_or_else(|| Error::other("expected a switch"));
        match name {
            "master" => self.bus.master = num(&v)?.clamp(0.0, 1.0),
            "muted" => self.bus.muted = flag(&v)?,
            "calls" => self.bus.calls = num(&v)?.clamp(0.0, 1.0),
            "calls_muted" => self.bus.calls_muted = flag(&v)?,
            "agc" => self.bus.set_agc(flag(&v)?),
            "inputs" => {
                let n = v.as_i64().ok_or_else(|| Error::other("expected a count"))?;
                self.bus.set_inputs(n.max(1) as usize);
            }
            _ => {
                let Some((what, k)) = Self::per_strip(name) else {
                    return Err(Error::other(format!("audio_bus: unknown parameter {name:?}")));
                };
                // A level for a strip the bus has not been told about yet
                // grows the bus: settings arrive in name order, and a fader
                // must not be lost for arriving before the count.
                if self.bus.strips.len() <= k {
                    self.bus.set_inputs(k + 1);
                }
                let s = &mut self.bus.strips[k];
                match what {
                    "vol" => s.volume = num(&v)?.clamp(0.0, 1.0),
                    "mute" => s.muted = flag(&v)?,
                    "label" => {
                        s.label = v.as_str().unwrap_or_default().to_string();
                    }
                    _ => unreachable!(),
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn voice<'a>(to: &'a str, from: &'a str, pcm: &'a [f32]) -> Voice<'a> {
        Voice {
            system: "M17",
            channel_hz: 433_475_000.0,
            to,
            from: Some(from),
            pcm,
            rate: 8_000.0,
        }
    }

    fn bus(rules: &[Rule]) -> AudioBus {
        let mut b = AudioBus::new(48_000.0);
        b.set_master(1.0, false);
        b.set_calls(1.0, false);
        b.set_subscriptions(rules.iter().cloned().map(Subscription::new).collect());
        b
    }

    /// Mono, from the stereo the bus renders: the two sides are equal for
    /// anything that is not a stereo strip.
    fn left(stereo: &[f32]) -> Vec<f32> {
        stereo.iter().step_by(2).copied().collect()
    }

    #[test]
    fn the_strip_mutes_every_call_at_once() {
        // One level for the lot, from the channel strip, so muting is one
        // action rather than one per group being watched.
        let mut b = bus(&[Rule::Everything]);
        let pcm = vec![0.5f32; 160];
        b.set_calls(0.5, true);
        assert!(!b.listening());
        assert!(!b.push(voice("ALL", "M0ABC", &pcm)));
        b.set_calls(0.5, false);
        assert!(b.push(voice("ALL", "M0ABC", &pcm)));
        assert!(b.levels().iter().any(|(_, v)| *v > 0.0), "the meter saw it");
    }

    #[test]
    fn nothing_is_heard_without_a_subscription() {
        // The default is silence. A receiver that plays whatever decodes is
        // unusable on a band with three conversations on it.
        let mut b = AudioBus::new(48_000.0);
        let pcm = vec![0.5f32; 160];
        assert!(!b.listening());
        assert!(!b.push(voice("ALL", "M0ABC", &pcm)));
        assert!(b.render(0).is_empty());
    }

    #[test]
    fn a_group_subscription_admits_that_group_only() {
        let mut b = bus(&[Rule::Group("M17-M17 C".into())]);
        let pcm = vec![0.5f32; 160];
        assert!(b.push(voice("M17-M17 C", "M0ABC", &pcm)), "the subscribed group");
        assert!(!b.push(voice("ALL", "M0XYZ", &pcm)), "somebody else's conversation");
        // 160 samples at 8 kHz become about 960 at 48 kHz, on each side.
        let out = left(b.render(0));
        assert!(out.len() > 900 && out.len() < 1000, "{} samples out", out.len());
        assert!(out.iter().any(|v| *v > 0.1), "the audio came through silent");
    }

    #[test]
    fn a_caller_is_followed_wherever_they_transmit() {
        // The other way an operator listens: not to a group but to a person,
        // who may key up on any of the channels being watched.
        let mut b = bus(&[Rule::Caller("M0ABC".into())]);
        let pcm = vec![0.25f32; 160];
        let mut elsewhere = voice("SOME-OTHER-GROUP", "M0ABC", &pcm);
        elsewhere.channel_hz = 144_800_000.0;
        assert!(b.push(elsewhere));
        assert!(!b.push(voice("M17-M17 C", "M0XYZ", &pcm)));
    }

    #[test]
    fn a_muted_subscription_is_not_a_quiet_one() {
        let mut b = bus(&[Rule::Everything]);
        let pcm = vec![0.5f32; 160];
        b.set_subscriptions(vec![Subscription { rule: Rule::Everything, volume: 0.8, muted: true }]);
        assert!(!b.push(voice("ALL", "M0ABC", &pcm)));
        assert!(!b.listening(), "a bus with only muted rules has nothing to decode for");
    }

    #[test]
    fn two_rules_covering_one_call_do_not_double_it() {
        // Subscribing to a group and to somebody talking on it is one
        // instruction twice, not twice the volume.
        let mut b = AudioBus::new(48_000.0);
        b.set_subscriptions(vec![
            Subscription { rule: Rule::Group("ALL".into()), volume: 0.5, muted: false },
            Subscription { rule: Rule::Caller("M0ABC".into()), volume: 0.9, muted: false },
        ]);
        b.set_calls(1.0, false);
        let pcm = vec![1.0f32; 160];
        assert_eq!(b.gain_for(&voice("ALL", "M0ABC", &pcm)), Some(0.9), "the louder rule wins");
    }

    #[test]
    fn a_replay_is_paced_by_the_block_rather_than_dumped() {
        // Handing the sink a whole transmission at once plays it at whatever
        // rate the device drains, which is not the rate it was spoken at.
        let mut b = AudioBus::new(48_000.0);
        let speech = Arc::new(Speech { pcm: vec![0.5; 8_000], rate: 8_000.0 });
        b.play(&speech);
        assert!(b.replaying());
        assert!((b.replay_left() - 1.0).abs() < 0.05, "{} s queued", b.replay_left());
        let n = left(b.render(1_200)).len();
        assert_eq!(n, 1_200, "a block's worth at a time");
        b.clear();
        assert!(b.replaying(), "the rest is still queued");
        b.stop_replay();
        assert!(!b.replaying());
    }

    #[test]
    fn a_transmission_writes_a_wav_that_says_what_it_holds() {
        let dir = std::env::temp_dir().join(format!("waveshark-call-{}", std::process::id()));
        let path = dir.join("call.wav");
        let speech = Speech { pcm: vec![0.5, -0.5, 0.25, 0.0], rate: 8_000.0 };
        write_wav(&path, &speech).expect("wav");
        let bytes = std::fs::read(&path).expect("read back");
        assert_eq!(&bytes[..4], b"RIFF");
        assert_eq!(&bytes[8..12], b"WAVE");
        // 44 byte header, then one sixteen bit sample per value.
        assert_eq!(bytes.len(), 44 + speech.pcm.len() * 2);
        assert_eq!(u32::from_le_bytes(bytes[24..28].try_into().unwrap()), 8_000);
        let (peak, rms) = levels_db(&speech);
        assert!((peak + 6.02).abs() < 0.1, "peak {peak}");
        assert!(rms < peak, "rms {rms} is not below the peak");
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A tone at a given level, for feeding the gain control something with
    /// an envelope rather than a step.
    fn tone(level: f32, n: usize) -> Vec<f32> {
        (0..n)
            .map(|i| level * (i as f32 * 0.3).sin())
            .collect()
    }

    #[test]
    fn a_quiet_transmission_is_brought_up_to_a_usable_level() {
        // The level a vocoder produces is whoever transmitted's microphone
        // gain, and on the real M17 traffic measured here that was -37 dBFS
        // peak. Passing that on as it is means an operator hears nothing.
        let mut b = bus(&[Rule::Everything]);
        let quiet = tone(0.01, 1600);
        for _ in 0..8 {
            assert!(b.push(voice("ALL", "M0ABC", &quiet)));
            b.clear();
        }
        b.push(voice("ALL", "M0ABC", &quiet));
        let peak = b.render(0).iter().fold(0.0f32, |a, v| a.max(v.abs()));
        assert!(peak > 0.05, "a quiet call came through at {peak:.3}");
        assert!(peak <= 1.0, "and it must not be lifted past full scale: {peak:.3}");
        assert!(b.agc_gain_db() > 12.0, "the gain control barely moved: {:.1} dB", b.agc_gain_db());
    }

    #[test]
    fn a_loud_transmission_is_not_lifted_further() {
        let mut b = bus(&[Rule::Everything]);
        let loud = tone(0.6, 1600);
        for _ in 0..8 {
            b.push(voice("ALL", "M0ABC", &loud));
            b.clear();
        }
        assert!(
            b.agc_gain_db() <= 0.1,
            "a loud call was given {:.1} dB it did not need",
            b.agc_gain_db()
        );
    }

    #[test]
    fn the_gain_control_can_be_switched_off() {
        // An operator comparing two signals wants what arrived, not what a
        // gain control made of it.
        let mut b = bus(&[Rule::Everything]);
        b.set_agc(false);
        let quiet = tone(0.01, 1600);
        for _ in 0..8 {
            b.push(voice("ALL", "M0ABC", &quiet));
            b.clear();
        }
        b.push(voice("ALL", "M0ABC", &quiet));
        let peak = b.render(0).iter().fold(0.0f32, |a, v| a.max(v.abs()));
        assert!(peak < 0.02, "the level was changed with the control off: {peak:.3}");
        assert_eq!(b.agc_gain_db(), 0.0);
    }

    #[test]
    fn a_channel_rule_matches_the_channel_it_names() {
        let mut b = bus(&[Rule::Channel(433_475_000.0)]);
        let pcm = vec![0.5f32; 160];
        assert!(b.push(voice("ALL", "M0ABC", &pcm)));
        let mut other = voice("ALL", "M0ABC", &pcm);
        other.channel_hz = 433_500_000.0;
        assert!(!b.push(other), "a channel 25 kHz away is a different channel");
    }

    /// A bus with `n` real inputs at `rate`, negotiated as the graph would.
    fn strips(n: usize, rate: f64, channels: usize) -> AudioBusNode {
        use pipeline::node::Node;
        let mut node = AudioBusNode::new(48_000.0);
        node.set_param("inputs", ParamValue::Int(n as i64)).unwrap();
        let spec = StreamSpec {
            kind: PortKind::Real,
            rate: rate * channels as f64,
            center: common::Hz(0),
            bandwidth: 0.0,
            channels,
        };
        let ins: Vec<PortSpec> = (0..n).map(|_| PortSpec { spec, latency: 0 }).collect();
        node.negotiate(&ins).unwrap();
        node.bus_mut().set_master(1.0, false);
        node
    }

    #[test]
    fn a_mono_strip_is_heard_on_both_sides() {
        let mut n = strips(1, 48_000.0, 1);
        n.bus_mut().strip_mut(0).unwrap().volume = 1.0;
        n.bus_mut().feed(0, &[0.5, -0.5]);
        assert_eq!(n.bus_mut().render(0), &[0.5, 0.5, -0.5, -0.5]);
    }

    #[test]
    fn strips_sum_rather_than_replace_each_other() {
        // The whole point of a mixer: two stations at once, not the last one
        // to be processed.
        let mut n = strips(2, 48_000.0, 1);
        for k in 0..2 {
            n.bus_mut().strip_mut(k).unwrap().volume = 1.0;
            n.bus_mut().feed(k, &[0.25; 4]);
        }
        let out = n.bus_mut().render(0);
        assert!(out.iter().all(|v| (*v - 0.5).abs() < 1e-6), "{out:?}");
    }

    #[test]
    fn a_stereo_strip_keeps_its_sides_apart() {
        let mut n = strips(1, 48_000.0, 2);
        n.bus_mut().strip_mut(0).unwrap().volume = 1.0;
        n.bus_mut().feed(0, &[1.0, -1.0, 1.0, -1.0]);
        assert_eq!(n.bus_mut().render(0), &[1.0, -1.0, 1.0, -1.0]);
    }

    #[test]
    fn the_mix_clips_rather_than_scaling_everything_down() {
        // Several channels at once can sum past full scale, and quietly
        // turning everything down would make the level of the channel being
        // listened to depend on how busy its neighbours are.
        let mut n = strips(2, 48_000.0, 1);
        for k in 0..2 {
            n.bus_mut().strip_mut(k).unwrap().volume = 1.0;
            n.bus_mut().feed(k, &[0.8; 4]);
        }
        let out = n.bus_mut().render(0);
        assert!(out.iter().all(|v| *v == 1.0), "{out:?}");
    }

    #[test]
    fn a_strip_at_another_rate_is_brought_to_the_speakers() {
        // Two channels at slightly different rates summed sample for sample
        // play one of them at the wrong pitch. Each strip is resampled on
        // its own.
        let mut n = strips(1, 24_000.0, 1);
        n.bus_mut().strip_mut(0).unwrap().volume = 1.0;
        let pcm = tone(0.5, 2_400);
        n.bus_mut().feed(0, &pcm);
        let out = left(n.bus_mut().render(0));
        assert!(out.len() > 4_600 && out.len() <= 4_800, "{} frames out", out.len());
    }

    #[test]
    fn a_muted_strip_and_the_fader_both_reach_the_meter() {
        let mut n = strips(1, 48_000.0, 1);
        {
            let s = n.bus_mut().strip_mut(0).unwrap();
            s.volume = 0.5;
        }
        n.bus_mut().feed(0, &[1.0; 4]);
        assert!((n.bus().strips()[0].peak - 0.5).abs() < 1e-6, "the meter reads after the fader");
        n.bus_mut().clear();
        n.bus_mut().strip_mut(0).unwrap().muted = true;
        n.bus_mut().feed(0, &[1.0; 4]);
        assert!(n.bus_mut().render(0).iter().all(|v| *v == 0.0), "a muted strip is silent");
    }

    #[test]
    fn every_level_is_a_parameter() {
        // What lets the chain view set a fader, a patch save it, and the
        // strip read it back: there is one route to every level.
        use pipeline::node::Node;
        let mut n = strips(2, 48_000.0, 1);
        n.set_param("vol1", ParamValue::Float(0.25)).unwrap();
        n.set_param("mute0", ParamValue::Bool(true)).unwrap();
        n.set_param("label1", ParamValue::Text("CH2".into())).unwrap();
        n.set_param("master", ParamValue::Float(0.75)).unwrap();
        n.set_param("calls", ParamValue::Float(0.6)).unwrap();
        assert_eq!(n.bus().strips()[1].volume, 0.25);
        assert!(n.bus().strips()[0].muted);
        assert_eq!(n.bus().strips()[1].label, "CH2");
        assert_eq!(n.bus().master(), (0.75, false));
        assert_eq!(n.bus().calls(), (0.6, false));
        let names: Vec<String> = n.params().into_iter().map(|p| p.name).collect();
        for want in ["master", "calls", "agc", "vol0", "mute1"] {
            assert!(names.iter().any(|n| n == want), "{want} is not a parameter: {names:?}");
        }
        // A level for a strip not yet counted grows the bus rather than
        // being lost, since settings arrive in name order.
        n.set_param("vol5", ParamValue::Float(0.1)).unwrap();
        assert_eq!(n.bus().strips().len(), 6);
    }
}
