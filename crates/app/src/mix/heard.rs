//! Everything the receiver hears, in one place, before anybody decides
//! whether to listen to it.
//!
//! Every fader's tap and every voice front end arrives here, labelled, and
//! leaves as one stream for the transcriber. This is also where "who is
//! talking now" is known, since every voice passes: the call list is fed
//! from here, and it must not depend on which fader is up or which group is
//! subscribed. A call used to be a packet, wrapped in an empty frame so the
//! list would see it; speech is not a packet, it is audio, and this is
//! where audio meets.

use common::{Error, Result};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::param::ParamValue;
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::collections::HashMap;

pub const KIND: &str = "heard";

/// Below this peak a block of speech is silence. A squelched analogue
/// channel delivers zeros and a vocoder between overs delivers near enough.
const SPEECH_FLOOR: f32 = 0.004;

/// Silence that ends a call. Long enough to survive a squelch chattering at
/// the edge of a repeater, short enough that a reply is a new row.
const HANG_S: f64 = 0.7;

/// One conversation the receiver is hearing, or has just stopped hearing.
#[derive(Clone, Debug, PartialEq)]
pub struct LiveCall {
    pub system: String,
    pub channel_hz: f64,
    pub to: String,
    pub from: Option<String>,
    pub first: std::time::Instant,
    pub last: std::time::Instant,
    /// Seconds somebody was actually talking, not the span of the call.
    pub seconds: f64,
    pub peak: f32,
    pub quiet_s: f64,
    /// The hang time has passed since the last speech: this is the last
    /// report of this call.
    pub over: bool,
}

impl LiveCall {
    /// The conversation this is, as the transcriber and the call list key it,
    /// so a row here and a line there are the same thing.
    pub fn key(&self) -> common::ConversationKey {
        common::ConversationKey::new(&self.system, self.channel_hz)
            .to(Some(self.to.clone()))
            .from(self.from.clone())
    }
}

pub struct HeardNode {
    inputs: usize,
    /// Who is talking now, by conversation.
    live: HashMap<common::ConversationKey, LiveCall>,
    /// Peak per conversation since the last block, for a meter on the row
    /// it belongs to. Decayed rather than reset, so a meter tracks speech
    /// instead of flickering with every syllable.
    peaks: HashMap<common::ConversationKey, f32>,
    /// Loudest call this block, whichever path its audio takes to the
    /// speaker.
    peak: f32,
}

impl HeardNode {
    pub fn new() -> Self {
        Self { inputs: 1, live: HashMap::new(), peaks: HashMap::new(), peak: 0.0 }
    }

    /// What each conversation put out last block, for a meter on its row.
    /// Keyed with nobody named as talking: see
    /// [`common::ConversationKey::meter`].
    pub fn levels(&self) -> Vec<(common::ConversationKey, f32)> {
        self.peaks.iter().map(|(k, v)| (k.clone(), *v)).collect()
    }

    /// The loudest call heard in the last block.
    pub fn peak(&self) -> f32 {
        self.peak
    }

    /// Who is talking now and who has just stopped, oldest first. A call that
    /// has ended is reported once, with `over` set, and then forgotten: the
    /// call list keeps the history, this keeps only the present.
    pub fn take_calls(&mut self) -> Vec<LiveCall> {
        let mut out: Vec<LiveCall> = self.live.values().cloned().collect();
        out.sort_by_key(|c| c.first);
        self.live.retain(|_, c| !c.over);
        out
    }

    /// Fold one block of speech into the table of who is talking now.
    ///
    /// Every block of every call a front end decoded comes through here,
    /// whether or not anybody is listening to it. Silence is what ends a
    /// call, after [`HANG_S`].
    ///
    /// A channel on the strip does not: a mode and a frequency do not say
    /// whether what is coming out is a conversation, a repeater's idle hiss
    /// or the airband. Its audio is on the tap under the analogue name for
    /// the transcriber, and that name is what keeps it off the list.
    pub fn track(&mut self, v: &common::Voice, block_s: f64) {
        let Some(to) = v.to.as_deref() else {
            return;
        };
        if v.system == super::fader::ANALOGUE {
            return;
        }
        let key = common::ConversationKey::of(v);
        let peak = v.pcm.iter().fold(0.0f32, |a, s| a.max(s.abs()));
        // Keyed with nobody talking, because a trunked carrier holds several
        // groups: keyed by frequency alone the whole column moved whenever
        // any one of them spoke, and keyed by caller a meter would move to a
        // new row every time somebody else took the group.
        let m = self.peaks.entry(key.meter()).or_insert(0.0);
        *m = m.max(peak);
        self.peak = self.peak.max(peak);
        let talking = peak > SPEECH_FLOOR;
        let now = std::time::Instant::now();
        match self.live.get_mut(&key) {
            Some(c) if talking => {
                c.last = now;
                c.quiet_s = 0.0;
                c.seconds += block_s;
                c.peak = c.peak.max(peak);
            }
            Some(c) => {
                c.quiet_s += block_s;
                if c.quiet_s >= HANG_S {
                    c.over = true;
                }
            }
            None if talking => {
                self.live.insert(
                    key,
                    LiveCall {
                        system: v.system.to_string(),
                        channel_hz: v.channel_hz,
                        to: to.to_string(),
                        from: v.from.clone(),
                        first: now,
                        last: now,
                        seconds: block_s,
                        peak,
                        quiet_s: 0.0,
                        over: false,
                    },
                );
            }
            None => {}
        }
    }

    /// Let the meters fall back towards zero. A block with no speech in it
    /// is a gap between words, not the end of the transmission.
    fn decay(&mut self) {
        self.peak *= 0.7;
        for v in self.peaks.values_mut() {
            *v *= 0.7;
        }
        self.peaks.retain(|_, v| *v > 0.002);
    }
}

impl Default for HeardNode {
    fn default() -> Self {
        Self::new()
    }
}

impl Node for HeardNode {
    fn name(&self) -> &str {
        KIND
    }

    fn num_inputs(&self) -> usize {
        self.inputs.max(1)
    }

    /// Like a mixer, it has a spare input for the next thing to be heard.
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
                        "the tap takes speech, and input {k} carries {other:?}"
                    )));
                }
            }
        }
        Ok(vec![StreamSpec {
            kind: PortKind::Voice,
            rate: super::OUT_HZ,
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
        ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        self.decay();
        let Some(out) = outputs.first_mut() else { return Ok(()) };
        for p in inputs {
            let Payload::Voice(voices) = p else { continue };
            for v in voices {
                if v.pcm.is_empty() {
                    continue;
                }
                self.track(v, ctx.block_seconds);
                out.voice_mut().push(v.clone());
            }
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.live.clear();
        self.peaks.clear();
        self.peak = 0.0;
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "inputs" => {
                let n = v.as_i64().ok_or_else(|| Error::other("expected a count"))?;
                self.inputs = n.max(1) as usize;
            }
            _ => return Err(Error::other(format!("heard: unknown parameter {name:?}"))),
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

    #[test]
    fn each_group_on_a_carrier_meters_on_its_own() {
        // A trunked system puts several groups on one frequency. Keyed by
        // frequency alone every row in the call list read the same level, so
        // the whole column moved whenever anybody spoke.
        let mut h = HeardNode::new();
        h.track(&voice("TG100", "M0ABC", &[0.5; 160]), 0.02);
        let levels = h.levels();
        let meter = |to: &str| {
            common::ConversationKey::new("M17", 433_475_000.0).to(Some(to.into())).meter()
        };
        let (loud, quiet) = (meter("TG100"), meter("TG200"));
        assert!(levels.iter().any(|(k, v)| *k == loud && *v > 0.1), "{levels:?}");
        assert!(!levels.iter().any(|(k, _)| *k == quiet), "a silent group read a level");
        assert_eq!(h.peak(), 0.5);
    }

    #[test]
    fn a_call_is_listed_while_somebody_talks_and_once_when_they_stop() {
        let mut h = HeardNode::new();
        h.track(&voice("ALL", "M0ABC", &[0.5; 160]), 0.02);
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1);
        assert!(!calls[0].over);
        assert_eq!(calls[0].to, "ALL");
        // Quiet for longer than the hang time ends it.
        for _ in 0..40 {
            h.track(&voice("ALL", "M0ABC", &[0.0; 160]), 0.02);
        }
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1);
        assert!(calls[0].over);
        assert!(h.take_calls().is_empty(), "an ended call is reported once");
    }

    #[test]
    fn a_tuned_channel_is_heard_and_is_not_a_call() {
        // Analogue speech is on the tap for the transcriber, under the
        // strip's label. Nothing about a mode and a frequency says it is a
        // conversation, so it is not a row in the call list.
        let mut h = HeardNode::new();
        let v = common::Voice {
            system: super::super::fader::ANALOGUE,
            channel_hz: 145_500_000.0,
            to: Some("CH1".into()),
            from: None,
            rate: 48_000.0,
            channels: 1,
            pcm: vec![0.5; 480],
        };
        h.track(&v, 0.01);
        assert!(h.take_calls().is_empty());
        assert!(h.levels().is_empty());
    }
}
