//! The call bus: every voice source in one place, and what reaches the
//! speaker decided by subscription rather than by whichever front end
//! happened to decode last.
//!
//! The packet bus does this for packets, and voice wants it more. A receiver
//! watching several channels can have three people talking at once, on two
//! systems, and "play whatever is decoding" is not a receiver anybody can use.
//! What an operator wants is a standing instruction: this talkgroup, that
//! caller, everything on that channel, and nothing else.
//!
//! # Shape
//!
//! Sources publish blocks of speech as they decode them, each labelled with
//! who is talking and to whom. Subscriptions are matched against those labels
//! and carry their own level, so two groups can be monitored at different
//! volumes. Anything unmatched is dropped rather than mixed quietly: an
//! operator listening to one group must be able to trust that what they hear
//! is that group.
//!
//! A transmission replayed from the packet log goes through the same bus,
//! because it is the same audio path and the master level should mean the
//! same thing for both.

use common::Speech;
use std::path::Path;
use std::collections::HashMap;
use std::sync::Arc;

/// Level the makeup gain aims a transmission at.
const TARGET_PEAK: f32 = 0.5;
/// Most it will lift one. Thirty decibels covers a handheld with its
/// microphone gain low; beyond that it is amplifying the vocoder's own noise
/// between words.
const MAX_MAKEUP: f32 = 32.0;
/// How quickly the tracked peak falls back when a transmission goes quiet.
const RELEASE_S: f32 = 2.0;

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

/// Mixes what is subscribed to, at the rate the output wants.
pub struct CallBus {
    out_rate: f64,
    subs: Vec<Subscription>,
    /// One level for all of it, from the channel strip, because that is where
    /// every other level in this receiver lives.
    master: f32,
    muted: bool,
    /// Peak per source since the last block, for a meter on the row it
    /// belongs to. Decayed rather than reset, so a meter tracks speech
    /// instead of flickering with every syllable.
    peaks: HashMap<String, f32>,
    /// One resampler per source, because each carries filter state and two
    /// sources at the same rate are still two different streams.
    rs: HashMap<String, audio::Resampler>,
    /// This block's mix, at `out_rate`.
    mix: Vec<f32>,
    scratch: Vec<f32>,
    /// A transmission being replayed, already at `out_rate`.
    replay: std::collections::VecDeque<f32>,
    /// Makeup gain, and the peak it is tracking.
    ///
    /// A vocoder's output level follows whatever the transmitting radio's
    /// microphone gain was, and on a handheld that is often thirty decibels
    /// below full scale: measured on a real M17 transmission here, the speech
    /// peaked at -37 dBFS and averaged -57, which is inaudible once the
    /// channel and master levels have had their share of it. This brings a
    /// transmission up to a usable level the way a receiver's AGC does,
    /// slowly and per source rather than per syllable.
    makeup: f32,
    tracked_peak: f32,
    /// What was last heard through the bus, for the interface to show.
    last: Option<String>,
}

impl CallBus {
    pub fn new(out_rate: f64) -> Self {
        Self {
            out_rate,
            subs: Vec::new(),
            master: 0.8,
            muted: false,
            peaks: HashMap::new(),
            rs: HashMap::new(),
            mix: Vec::new(),
            scratch: Vec::new(),
            replay: std::collections::VecDeque::new(),
            makeup: 1.0,
            tracked_peak: 0.0,
            last: None,
        }
    }

    pub fn subscriptions(&self) -> &[Subscription] {
        &self.subs
    }

    pub fn set_subscriptions(&mut self, subs: Vec<Subscription>) {
        self.subs = subs;
    }

    /// The level every subscription is heard at, and whether the lot is
    /// muted. Set from the channel strip.
    pub fn set_master(&mut self, volume: f32, muted: bool) {
        self.master = volume.clamp(0.0, 1.0);
        self.muted = muted;
    }

    pub fn master(&self) -> (f32, bool) {
        (self.master, self.muted)
    }

    /// Whether anything at all is being listened to, which decides whether a
    /// source needs to do the work of decoding speech.
    pub fn listening(&self) -> bool {
        !self.muted && self.subs.iter().any(|s| !s.muted)
    }

    /// What each source put into the mix last block, keyed as
    /// `system:channel`, for a meter on its row.
    pub fn levels(&self) -> Vec<(String, f32)> {
        self.peaks.iter().map(|(k, v)| (k.clone(), *v)).collect()
    }

    /// The key a source's meter is filed under.
    pub fn key_of(system: &str, channel_hz: f64) -> String {
        format!("{system}:{channel_hz:.0}")
    }

    /// What the subscriptions say about one transmission: the gain to mix it
    /// at, or `None` to ignore it.
    ///
    /// The loudest matching subscription wins rather than their sum, so
    /// covering one group twice does not make it twice as loud.
    pub fn gain_for(&self, v: &Voice) -> Option<f32> {
        if self.muted {
            return None;
        }
        self.subs
            .iter()
            .filter(|s| s.rule.matches(v))
            .map(|s| s.gain() * self.master)
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

        // Track the loudest thing heard lately and lift it towards the
        // target. Attack is immediate so the first syllable is not lost;
        // release runs on a time constant so a gap between words does not
        // wind the gain up on the silence.
        let block_peak = self.scratch.iter().fold(0.0f32, |a, s| a.max(s.abs()));
        if block_peak > self.tracked_peak {
            self.tracked_peak = block_peak;
        } else {
            let secs = self.scratch.len() as f32 / self.out_rate as f32;
            self.tracked_peak *= (-secs / RELEASE_S).exp();
        }
        self.makeup = if self.tracked_peak > 1e-5 {
            (TARGET_PEAK / self.tracked_peak).clamp(1.0, MAX_MAKEUP)
        } else {
            1.0
        };
        let gain = gain * self.makeup;
        if self.mix.len() < self.scratch.len() {
            self.mix.resize(self.scratch.len(), 0.0);
        }
        for (m, s) in self.mix.iter_mut().zip(self.scratch.iter()) {
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

    /// This block's audio: whatever was published and subscribed to, plus a
    /// slice of any replay, as mono at the output rate.
    ///
    /// `frames` is what the rest of the mixer is carrying this block, so a
    /// replay runs at real time rather than arriving all at once. Zero means
    /// nothing else is producing audio, and the replay sets the pace itself.
    pub fn take(&mut self, frames: usize) -> &[f32] {
        if !self.replay.is_empty() {
            let want = if frames > 0 { frames } else { (self.out_rate / 50.0) as usize };
            let take = want.min(self.replay.len());
            if self.mix.len() < take {
                self.mix.resize(take, 0.0);
            }
            for (i, s) in self.replay.drain(..take).enumerate() {
                self.mix[i] += s;
            }
        }
        &self.mix
    }

    /// Drop this block's audio, once it has been mixed, and let the meters
    /// fall back towards zero.
    pub fn clear(&mut self) {
        self.mix.clear();
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

    fn bus(rules: &[Rule]) -> CallBus {
        let mut b = CallBus::new(48_000.0);
        b.set_master(1.0, false);
        b.set_subscriptions(rules.iter().cloned().map(Subscription::new).collect());
        b
    }

    #[test]
    fn the_strip_mutes_everything_at_once() {
        // One level for the lot, from the channel strip, so muting is one
        // action rather than one per group being watched.
        let mut b = bus(&[Rule::Everything]);
        let pcm = vec![0.5f32; 160];
        b.set_master(0.5, true);
        assert!(!b.listening());
        assert!(!b.push(voice("ALL", "M0ABC", &pcm)));
        b.set_master(0.5, false);
        assert!(b.push(voice("ALL", "M0ABC", &pcm)));
        assert!(b.levels().iter().any(|(_, v)| *v > 0.0), "the meter saw it");
    }

    #[test]
    fn nothing_is_heard_without_a_subscription() {
        // The default is silence. A receiver that plays whatever decodes is
        // unusable on a band with three conversations on it.
        let mut b = CallBus::new(48_000.0);
        let pcm = vec![0.5f32; 160];
        assert!(!b.listening());
        assert!(!b.push(voice("ALL", "M0ABC", &pcm)));
        assert!(b.take(0).is_empty());
    }

    #[test]
    fn a_group_subscription_admits_that_group_only() {
        let mut b = bus(&[Rule::Group("M17-M17 C".into())]);
        let pcm = vec![0.5f32; 160];
        assert!(b.push(voice("M17-M17 C", "M0ABC", &pcm)), "the subscribed group");
        assert!(!b.push(voice("ALL", "M0XYZ", &pcm)), "somebody else's conversation");
        // 160 samples at 8 kHz become about 960 at 48 kHz.
        let out = b.take(0);
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
        let mut b = CallBus::new(48_000.0);
        b.set_subscriptions(vec![
            Subscription { rule: Rule::Group("ALL".into()), volume: 0.5, muted: false },
            Subscription { rule: Rule::Caller("M0ABC".into()), volume: 0.9, muted: false },
        ]);
        b.set_master(1.0, false);
        let pcm = vec![1.0f32; 160];
        assert_eq!(b.gain_for(&voice("ALL", "M0ABC", &pcm)), Some(0.9), "the louder rule wins");
    }

    #[test]
    fn a_replay_is_paced_by_the_block_rather_than_dumped() {
        // Handing the sink a whole transmission at once plays it at whatever
        // rate the device drains, which is not the rate it was spoken at.
        let mut b = CallBus::new(48_000.0);
        let speech = Arc::new(Speech { pcm: vec![0.5; 8_000], rate: 8_000.0 });
        b.play(&speech);
        assert!(b.replaying());
        assert!((b.replay_left() - 1.0).abs() < 0.05, "{} s queued", b.replay_left());
        let n = b.take(1_200).len();
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

    #[test]
    fn a_quiet_transmission_is_brought_up_to_a_usable_level() {
        // The level a vocoder produces is whoever transmitted's microphone
        // gain, and on the real M17 traffic measured here that was -37 dBFS
        // peak. Passing that on as it is means an operator hears nothing.
        let mut b = bus(&[Rule::Everything]);
        let quiet = vec![0.01f32; 1600];
        for _ in 0..4 {
            assert!(b.push(voice("ALL", "M0ABC", &quiet)));
            b.clear();
        }
        b.push(voice("ALL", "M0ABC", &quiet));
        let peak = b.take(0).iter().fold(0.0f32, |a, v| a.max(v.abs()));
        assert!(peak > 0.1, "a quiet call came through at {peak:.3}");
        assert!(peak <= 1.0, "and it must not be lifted past full scale: {peak:.3}");
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
}
