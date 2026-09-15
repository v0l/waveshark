//! Which calls are heard, and how loudly.
//!
//! Every voice front end ends here. A voice port carries every conversation
//! on a system, each block labelled with who is talking and to whom, and
//! "play whatever decoded last" is not a receiver anybody can use: blocks
//! are matched against subscriptions and anything unmatched is dropped
//! rather than mixed quietly, so an operator listening to one group can
//! trust that what they hear is that group. What matches is levelled and
//! brought to the speaker's rate, and leaves for the bus as it arrived,
//! with its labels: the bus mixes it, and can still say whose voice it
//! mixed. All of it under a level of its own beside the master, because a
//! call is not a channel anybody tuned.

use common::{Error, Result};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::collections::HashMap;

pub const KIND: &str = "calls";

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
    /// Everything any source decodes. The pane offers a group or a caller
    /// today; this is the rest of the vocabulary the bus answers to.
    #[cfg_attr(not(test), allow(dead_code))]
    Everything,
    /// One talkgroup, reflector or destination, whatever the system calls it.
    Group(String),
    /// One caller, wherever they transmit.
    Caller(String),
    /// Whatever is heard on one channel, to within its own width.
    #[cfg_attr(not(test), allow(dead_code))]
    Channel(f64),
    /// One system: every M17 call, every DMR call.
    #[cfg_attr(not(test), allow(dead_code))]
    System(String),
}

impl Rule {
    /// Whether this rule covers a transmission.
    pub fn matches(&self, v: &Heard) -> bool {
        match self {
            Rule::Everything => true,
            // Case-insensitive because a callsign is written both ways and
            // nobody means a different aircraft by it.
            Rule::Group(g) => v.to.eq_ignore_ascii_case(g),
            Rule::Caller(c) => v.from.is_some_and(|f| f.eq_ignore_ascii_case(c)),
            Rule::Channel(hz) => (v.channel_hz - hz).abs() < common::CHANNEL_MATCH_HZ,
            Rule::System(s) => v.system.eq_ignore_ascii_case(s),
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
        if self.muted { 0.0 } else { self.volume.clamp(0.0, 2.0) }
    }
}

/// A block of speech from one source, as it was decoded: what arrived,
/// borrowed from the [`common::Voice`] it came in on.
#[derive(Clone, Debug, PartialEq)]
pub struct Heard<'a> {
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

pub struct CallsNode {
    inputs: usize,
    out_rate: f64,
    subs: Vec<Subscription>,
    volume: f32,
    muted: bool,
    /// The gain control every call passes through, and whether it is on.
    ///
    /// The same [`dsp::agc::Agc`] a listening channel uses, with the same
    /// voice constants: attack fast enough that a loud caller cannot blast,
    /// release slow enough that the gain does not climb audibly between
    /// words, and a hang time so a pause is not treated as a fade. One
    /// instance rather than one per source, because what it is levelling is
    /// the output somebody is listening to.
    agc: dsp::agc::Agc,
    agc_on: bool,
    /// One resampler per carrier, because each carries filter state and two
    /// sources at the same rate are still two different streams. Per
    /// carrier rather than per group: only one group on a channel is ever
    /// speaking, and a map keyed by group would grow for as long as the
    /// receiver runs.
    rs: HashMap<String, audio::Resampler>,
    scratch: Vec<f32>,
    /// This block's speech, each admitted block labelled, at `out_rate`.
    admitted: Vec<common::Voice>,
    /// What was last heard, for the interface to show.
    last: Option<String>,
    /// Peak of this block's mix.
    peak: f32,
}

impl CallsNode {
    pub fn new(out_rate: f64) -> Self {
        Self {
            inputs: 1,
            out_rate,
            subs: Vec::new(),
            volume: 0.8,
            muted: false,
            agc: {
                let mut a = dsp::agc::Agc::voice(out_rate);
                a.set_max_gain_db(MAX_GAIN_DB);
                a
            },
            agc_on: true,
            rs: HashMap::new(),
            scratch: Vec::new(),
            admitted: Vec::new(),
            last: None,
            peak: 0.0,
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn subscriptions(&self) -> &[Subscription] {
        &self.subs
    }

    pub fn set_subscriptions(&mut self, subs: Vec<Subscription>) {
        self.subs = subs;
    }

    /// The level every subscribed call is heard at, and whether any is.
    pub fn level(&self) -> (f32, bool) {
        (self.volume, self.muted)
    }

    pub fn agc_on(&self) -> bool {
        self.agc_on
    }

    fn set_agc(&mut self, on: bool) {
        if on != self.agc_on {
            self.agc.reset();
        }
        self.agc_on = on;
    }

    /// What the gain control is adding right now, in decibels.
    pub fn agc_gain_db(&self) -> f32 {
        if self.agc_on { self.agc.gain_db() } else { 0.0 }
    }

    /// Whether any call at all is being listened to, which decides whether a
    /// source needs to do the work of decoding speech.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn listening(&self) -> bool {
        !self.muted && self.subs.iter().any(|s| !s.muted)
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn last_heard(&self) -> Option<&str> {
        self.last.as_deref()
    }

    /// Peak of the last block's mix.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// What the subscriptions say about one transmission: the gain to mix it
    /// at, before the master, or `None` to ignore it.
    ///
    /// The loudest matching subscription wins rather than their sum, so
    /// covering one group twice does not make it twice as loud.
    pub fn gain_for(&self, v: &Heard) -> Option<f32> {
        if self.muted {
            return None;
        }
        self.subs
            .iter()
            .filter(|s| s.rule.matches(v))
            .map(|s| s.gain() * self.volume)
            .fold(None, |acc, g| Some(acc.map_or(g, |a: f32| a.max(g))))
    }

    /// Admit a block of speech. Returns whether it was.
    pub fn push(&mut self, v: &common::Voice) -> bool {
        let Some(to) = v.to.as_deref() else {
            return self.push_audio(v);
        };
        let heard = Heard {
            system: v.system,
            channel_hz: v.channel_hz,
            to,
            from: v.from.as_deref(),
            pcm: &v.pcm,
            rate: v.rate,
        };
        let Some(gain) = self.gain_for(&heard) else {
            return false;
        };
        let admitted = self.admit(v, gain);
        if admitted {
            self.last = Some(match &v.from {
                Some(f) => format!("{f} to {to}"),
                None => to.to_string(),
            });
        }
        admitted
    }

    /// Mix audio from a front end that is not a call.
    ///
    /// A camera's sound subcarrier is speech in every sense that matters to
    /// a speaker and in none that matters to a call list: it names no party,
    /// nobody keyed up to start it and nothing ends it but the transmitter
    /// going away. So there is nothing for a subscription to match and
    /// nothing to wait for one: it is heard because the receiver is
    /// receiving it, under the calls level like the rest of what the front
    /// ends produce.
    pub fn push_audio(&mut self, v: &common::Voice) -> bool {
        if self.muted {
            return false;
        }
        self.admit(v, self.volume)
    }

    /// Resample, level and label one block for the bus.
    fn admit(&mut self, v: &common::Voice, gain: f32) -> bool {
        if v.pcm.is_empty() || gain <= 0.0 {
            return false;
        }
        let pcm = v.mono();
        self.scratch.clear();
        if (v.rate - self.out_rate).abs() < 0.5 {
            self.scratch.extend_from_slice(&pcm);
        } else {
            let rs = self
                .rs
                .entry(format!("{}:{:.0}", v.system, v.channel_hz))
                .or_insert_with(|| audio::Resampler::new(v.rate, self.out_rate, 4));
            rs.process(&pcm, &mut self.scratch);
        }
        // Levelled before the subscription's own volume, so what an operator
        // sets is a level relative to other calls rather than a fight with
        // whoever transmitted loudest.
        if self.agc_on {
            self.agc.process(&mut self.scratch);
        }
        for s in &mut self.scratch {
            *s *= gain;
        }
        self.peak = self.scratch.iter().fold(self.peak, |a, s| a.max(s.abs()));
        self.admitted.push(common::Voice {
            rate: self.out_rate,
            channels: 1,
            pcm: std::mem::take(&mut self.scratch),
            ..v.clone()
        });
        true
    }

    /// What was admitted this block, and clear it for the next.
    pub fn take(&mut self) -> Vec<common::Voice> {
        std::mem::take(&mut self.admitted)
    }
}

impl Node for CallsNode {
    fn name(&self) -> &str {
        KIND
    }

    fn num_inputs(&self) -> usize {
        self.inputs.max(1)
    }

    fn optional_inputs(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        for (k, i) in inputs.iter().enumerate() {
            match i.spec.kind {
                PortKind::Voice => {}
                PortKind::Real if i.spec.is_silence() => {}
                other => {
                    return Err(Error::other(format!(
                        "calls takes speech, and input {k} carries {other:?}"
                    )));
                }
            }
        }
        Ok(vec![StreamSpec {
            kind: PortKind::Voice,
            rate: self.out_rate,
            center: common::Hz(0),
            bandwidth: 0.0,
            channels: 1,
            ..Default::default()
        }])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        self.peak = 0.0;
        for p in inputs {
            let Payload::Voice(voices) = p else { continue };
            for v in voices {
                self.push(v);
            }
        }
        let admitted = self.take();
        if let Some(o) = outputs.first_mut() {
            *o.voice_mut() = admitted;
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.admitted.clear();
        self.rs.clear();
        self.agc.reset();
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::float("vol", self.volume as f64, 0.0..=1.0).label("Calls"),
            Param::bool("mute", self.muted).label("Calls muted"),
            Param::bool("agc", self.agc_on).label("Call AGC"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        let num = |v: &ParamValue| {
            v.as_f64().map(|f| f as f32).ok_or_else(|| Error::other("expected a number"))
        };
        let flag = |v: &ParamValue| v.as_bool().ok_or_else(|| Error::other("expected a switch"));
        match name {
            "vol" => self.volume = num(&v)?.clamp(0.0, 1.0),
            "mute" => self.muted = flag(&v)?,
            "agc" => self.set_agc(flag(&v)?),
            "inputs" => {
                let n = v.as_i64().ok_or_else(|| Error::other("expected a count"))?;
                self.inputs = n.max(1) as usize;
            }
            _ => return Err(Error::other(format!("calls: unknown parameter {name:?}"))),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn voice(to: &str, from: &str, pcm: &[f32]) -> common::Voice {
        common::Voice {
            system: "M17",
            channel_hz: 433_475_000.0,
            to: Some(to.into()),
            from: Some(from.into()),
            rate: 8_000.0,
            channels: 1,
            pcm: pcm.to_vec(),
        }
    }

    fn heard(v: &common::Voice) -> Heard<'_> {
        Heard {
            system: v.system,
            channel_hz: v.channel_hz,
            to: v.to.as_deref().unwrap_or(""),
            from: v.from.as_deref(),
            pcm: &v.pcm,
            rate: v.rate,
        }
    }

    /// Everything admitted this block, summed.
    fn mixed(c: &mut CallsNode) -> Vec<f32> {
        let mut out = Vec::new();
        for v in c.take() {
            if out.len() < v.pcm.len() {
                out.resize(v.pcm.len(), 0.0);
            }
            for (o, s) in out.iter_mut().zip(&v.pcm) {
                *o += s;
            }
        }
        out
    }

    fn calls(rules: &[Rule]) -> CallsNode {
        let mut c = CallsNode::new(48_000.0);
        c.set_param("vol", ParamValue::Float(1.0)).unwrap();
        c.set_subscriptions(rules.iter().cloned().map(Subscription::new).collect());
        c
    }

    #[test]
    fn the_mute_silences_every_call_at_once() {
        // One level for the lot, so muting is one action rather than one
        // per group being watched.
        let mut c = calls(&[Rule::Everything]);
        let pcm = vec![0.5f32; 160];
        c.set_param("mute", ParamValue::Bool(true)).unwrap();
        assert!(!c.listening());
        assert!(!c.push(&voice("ALL", "M0ABC", &pcm)));
        c.set_param("mute", ParamValue::Bool(false)).unwrap();
        assert!(c.push(&voice("ALL", "M0ABC", &pcm)));
    }

    #[test]
    fn nothing_is_heard_without_a_subscription() {
        // The default is silence. A receiver that plays whatever decodes is
        // unusable on a band with three conversations on it.
        let mut c = CallsNode::new(48_000.0);
        let pcm = vec![0.5f32; 160];
        assert!(!c.listening());
        assert!(!c.push(&voice("ALL", "M0ABC", &pcm)));
        assert!(mixed(&mut c).is_empty());
    }

    #[test]
    fn a_group_subscription_admits_that_group_only() {
        let mut c = calls(&[Rule::Group("M17-M17 C".into())]);
        let pcm = vec![0.5f32; 160];
        assert!(c.push(&voice("M17-M17 C", "M0ABC", &pcm)), "the subscribed group");
        assert!(!c.push(&voice("ALL", "M0XYZ", &pcm)), "somebody else's conversation");
        // 160 samples at 8 kHz become about 960 at 48 kHz.
        let out = mixed(&mut c);
        assert!(out.len() > 900 && out.len() < 1000, "{} samples out", out.len());
        assert!(out.iter().any(|v| *v > 0.1), "the audio came through silent");
    }

    #[test]
    fn a_caller_is_followed_wherever_they_transmit() {
        // The other way an operator listens: not to a group but to a person,
        // who may key up on any of the channels being watched.
        let mut c = calls(&[Rule::Caller("M0ABC".into())]);
        let pcm = vec![0.25f32; 160];
        let mut elsewhere = voice("SOME-OTHER-GROUP", "M0ABC", &pcm);
        elsewhere.channel_hz = 144_800_000.0;
        assert!(c.push(&elsewhere));
        assert!(!c.push(&voice("M17-M17 C", "M0XYZ", &pcm)));
    }

    #[test]
    fn a_muted_subscription_is_not_a_quiet_one() {
        let mut c = calls(&[Rule::Everything]);
        let pcm = vec![0.5f32; 160];
        c.set_subscriptions(vec![Subscription {
            rule: Rule::Everything,
            volume: 0.8,
            muted: true,
        }]);
        assert!(!c.push(&voice("ALL", "M0ABC", &pcm)));
        assert!(!c.listening(), "with only muted rules there is nothing to decode for");
    }

    #[test]
    fn two_rules_covering_one_call_do_not_double_it() {
        // Subscribing to a group and to somebody talking on it is one
        // instruction twice, not twice the volume.
        let mut c = CallsNode::new(48_000.0);
        c.set_subscriptions(vec![
            Subscription { rule: Rule::Group("ALL".into()), volume: 0.5, muted: false },
            Subscription { rule: Rule::Caller("M0ABC".into()), volume: 0.9, muted: false },
        ]);
        c.set_param("vol", ParamValue::Float(1.0)).unwrap();
        let pcm = vec![1.0f32; 160];
        let v = voice("ALL", "M0ABC", &pcm);
        assert_eq!(c.gain_for(&heard(&v)), Some(0.9), "the louder rule wins");
    }

    /// A tone at a given level, for feeding the gain control something with
    /// an envelope rather than a step.
    fn tone(level: f32, n: usize) -> Vec<f32> {
        (0..n).map(|i| level * (i as f32 * 0.3).sin()).collect()
    }

    #[test]
    fn a_quiet_transmission_is_brought_up_to_a_usable_level() {
        // The level a vocoder produces is whoever transmitted's microphone
        // gain, and on the real M17 traffic measured here that was -37 dBFS
        // peak. Passing that on as it is means an operator hears nothing.
        let mut c = calls(&[Rule::Everything]);
        let quiet = tone(0.01, 1600);
        for _ in 0..8 {
            assert!(c.push(&voice("ALL", "M0ABC", &quiet)));
            c.take();
        }
        c.push(&voice("ALL", "M0ABC", &quiet));
        let peak = mixed(&mut c).iter().fold(0.0f32, |a, v| a.max(v.abs()));
        assert!(peak > 0.05, "a quiet call came through at {peak:.3}");
        assert!(peak <= 1.0, "and it must not be lifted past full scale: {peak:.3}");
    }

    #[test]
    fn the_gain_control_can_be_switched_off() {
        let mut c = calls(&[Rule::Everything]);
        c.set_param("agc", ParamValue::Bool(false)).unwrap();
        let quiet = tone(0.01, 1600);
        for _ in 0..8 {
            c.push(&voice("ALL", "M0ABC", &quiet));
            c.take();
        }
        c.push(&voice("ALL", "M0ABC", &quiet));
        let peak = mixed(&mut c).iter().fold(0.0f32, |a, v| a.max(v.abs()));
        assert!(peak < 0.02, "off means what arrived: {peak:.3}");
        assert_eq!(c.agc_gain_db(), 0.0);
    }

    #[test]
    fn audio_that_names_no_party_is_heard_under_the_calls_level() {
        let mut c = CallsNode::new(48_000.0);
        c.set_param("vol", ParamValue::Float(0.5)).unwrap();
        let sound = common::Voice {
            system: "DVB-T",
            channel_hz: 474_000_000.0,
            to: None,
            from: None,
            rate: 48_000.0,
            channels: 1,
            pcm: vec![0.5; 480],
        };
        assert!(c.push(&sound));
        let out = c.take();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].pcm.len(), 480);
        assert_eq!(out[0].system, "DVB-T", "it leaves labelled as it arrived");
        c.set_param("mute", ParamValue::Bool(true)).unwrap();
        assert!(!c.push(&sound));
    }

    #[test]
    fn what_is_admitted_leaves_with_its_labels() {
        let mut c = calls(&[Rule::Everything]);
        assert!(c.push(&voice("ALL", "M0ABC", &[0.5; 160])));
        let out = c.take();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].to.as_deref(), Some("ALL"));
        assert_eq!(out[0].from.as_deref(), Some("M0ABC"));
        assert_eq!(out[0].rate, 48_000.0, "at the speaker's rate");
        assert_eq!(c.last_heard(), Some("M0ABC to ALL"));
    }
}
