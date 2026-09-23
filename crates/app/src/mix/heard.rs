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
    /// The coded squelch the traffic is using, where the channel has one:
    /// "141.3" for a CTCSS tone, "D023" for a DCS code.
    pub code: Option<String>,
    pub first: std::time::Instant,
    pub last: std::time::Instant,
    /// Seconds somebody was actually talking, not the span of the call.
    pub seconds: f64,
    pub peak: f32,
    pub quiet_s: f64,
    /// The hang time has passed since the last speech: this is the last
    /// report of this call.
    pub over: bool,
    /// What the system said about the transmission: the vocoder and what
    /// protects it. Only a decoded call has one; an analogue channel says
    /// nothing about itself.
    pub said: Option<common::Over>,
    /// The conversation this was reported under before its labels filled in.
    ///
    /// Analogue identity arrives late: a coded squelch takes half a second of
    /// audio to read and a PTT-ID about the same, so the first blocks of an
    /// over are heard under the bare channel name. The key carries the
    /// labels, so this says which row to update rather than leaving a list
    /// with the same transmission on two lines. Set once, on the report that
    /// changed it.
    pub was: Option<common::ConversationKey>,
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
        // Said once: the row it was reported under has been updated by the
        // time anybody asks again.
        for c in self.live.values_mut() {
            c.was = None;
        }
        self.live.retain(|_, c| !c.over);
        out
    }

    /// Fold one block of speech into the table of who is talking now.
    ///
    /// Every block of every call a front end decoded comes through here,
    /// whether or not anybody is listening to it. Silence is what ends a
    /// call, after [`HANG_S`].
    ///
    /// A channel on the strip counts only where somebody said it carries
    /// people talking: a mode and a frequency do not say whether what is
    /// coming out is a conversation, a repeater's idle hiss or the airband.
    /// That is the channel's voice mark, which is what puts a label on the
    /// analogue tap, so an unnamed party is what keeps a channel off the
    /// list.
    pub fn track(&mut self, v: &common::Voice, block_s: f64) {
        let Some(to) = v.called() else {
            return;
        };
        let key = common::ConversationKey::of(v);
        let peak = v.pcm.iter().fold(0.0f32, |a, s| a.max(s.abs()));
        // A system that counts the channel for itself is talking whether or
        // not the speech could be decoded: a P25 or NXDN call is 180 ms of
        // the channel per frame, and a list that waited for audio showed
        // nothing at all on a network whose vocoder is not built in.
        let stated = v.over.as_ref().map(|o| o.seconds).filter(|s| *s > 0.0);
        let block_s = stated.unwrap_or(block_s);
        // Keyed with nobody talking, because a trunked carrier holds several
        // groups: keyed by frequency alone the whole column moved whenever
        // any one of them spoke, and keyed by caller a meter would move to a
        // new row every time somebody else took the group.
        let m = self.peaks.entry(key.meter()).or_insert(0.0);
        *m = m.max(peak);
        self.peak = self.peak.max(peak);
        let talking = peak > SPEECH_FLOOR || stated.is_some();
        let now = std::time::Instant::now();
        // An over whose labels fill in part way through is the same over.
        // Analogue identity arrives late by nature: a PTT-ID is tones that
        // take half a second to settle, a coded squelch tone needs half a
        // second of window, and a DCS code needs two words of it. The key
        // carries both labels, so without this the call list showed one
        // transmission as two or three rows, the first of them nameless.
        if !self.live.contains_key(&key) {
            let vaguer = self
                .live
                .iter()
                .find(|(k, _)| {
                    k.system == v.system
                        && k.channel_hz == key.channel_hz
                        && k.from.as_ref().is_none_or(|f| Some(f) == v.from.as_ref())
                        && k.to.as_ref().is_some_and(|t| to.starts_with(t.as_str()))
                })
                .map(|(k, _)| k.clone());
            if let Some(old) = vaguer
                && let Some(mut c) = self.live.remove(&old)
            {
                // The row the call was reported under has to be closed, or
                // the list keeps a transmission that never ends beside the
                // one it turned into.
                c.was = Some(old);
                c.to = to.to_string();
                c.from = v.from.clone().or(c.from);
                self.live.insert(key.clone(), c);
            }
        }
        match self.live.get_mut(&key) {
            Some(c) if talking => {
                c.last = now;
                c.quiet_s = 0.0;
                c.seconds += block_s;
                c.peak = c.peak.max(peak);
                // A station that names itself part way through the over is
                // the same call: an analogue radio sends its PTT-ID as tones
                // that take half a second to settle, and some send it at the
                // end of the over rather than the start.
                if c.from.is_none() {
                    c.from = v.from.clone();
                }
                // The group takes half a second of audio to read, so it
                // arrives after the over has started, and the newest reading
                // wins: an operator who changes the code on the radio is
                // watching the list to see it change.
                if v.code.is_some() {
                    c.code = v.code.clone();
                }
                // A grant names the cipher and the traffic that follows says
                // nothing, so what was said stands until something says
                // otherwise.
                if v.over.is_some() {
                    c.said = v.over.clone();
                }
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
                        code: v.code.clone(),
                        first: now,
                        last: now,
                        seconds: block_s,
                        peak,
                        quiet_s: 0.0,
                        over: false,
                        said: v.over.clone(),
                        was: None,
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
            code: None,
            over: None,
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

    /// Analogue audio off a channel, labelled as the fader labels it: with
    /// the strip's name when the channel is marked as voice, and with
    /// nothing when it is not.
    fn tuned(to: Option<&str>) -> common::Voice {
        common::Voice {
            system: super::super::fader::ANALOGUE,
            channel_hz: 145_500_000.0,
            to: to.map(str::to_string),
            from: None,
            code: None,
            over: None,
            rate: 48_000.0,
            channels: 1,
            pcm: vec![0.5; 480],
        }
    }

    /// The coded squelch reaches the call beside the labels, and fills in
    /// after the over has started because that is when it is read.
    #[test]
    fn a_call_carries_the_coded_squelch_the_group_is_using() {
        let mut h = HeardNode::new();
        let mut first = tuned(Some("CH1"));
        first.code = None;
        h.track(&first, 0.02);
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1);
        assert_eq!(calls[0].code, None, "half a second of audio has not been read yet");

        // The detector settles and the group arrives, on the same over.
        let mut coded = tuned(Some("CH1"));
        coded.code = Some("D023".into());
        h.track(&coded, 0.02);
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1, "the group made a second row: {calls:?}");
        assert_eq!(calls[0].code.as_deref(), Some("D023"));
        assert_eq!(calls[0].to, "CH1", "the group is beside the name, not in it");

        // And the newest reading wins: somebody changing the code on the
        // radio is watching the list to see it change.
        let mut again = tuned(Some("CH1"));
        again.code = Some("D131".into());
        h.track(&again, 0.02);
        let calls = h.take_calls();
        assert_eq!(calls[0].code.as_deref(), Some("D131"), "the group never changed");
    }

    /// A radio that names itself part way through the over is one call, not
    /// two rows.
    ///
    /// An analogue PTT-ID is tones at the head of the over and they take half
    /// a second to settle, so the first blocks of speech arrive with nobody
    /// named and the rest with the unit on them. The key carries who is
    /// talking, so the call moved to a key of its own and the list showed the
    /// same over twice.
    #[test]
    fn a_station_that_names_itself_mid_over_is_one_call() {
        let mut h = HeardNode::new();
        let mut anonymous = tuned(Some("CH1"));
        anonymous.from = None;
        h.track(&anonymous, 0.02);
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1);
        assert_eq!(calls[0].from, None);

        let mut named = tuned(Some("CH1"));
        named.from = Some("123".into());
        h.track(&named, 0.02);
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1, "the same over became two rows: {calls:?}");
        assert_eq!(calls[0].from.as_deref(), Some("123"));
        assert!(calls[0].seconds >= 0.04, "the over it was already holding: {}", calls[0].seconds);

        // And the next block of the same over adds to it rather than
        // starting again.
        h.track(&named, 0.02);
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1);
        assert_eq!(calls[0].from.as_deref(), Some("123"));
    }

    #[test]
    fn a_tuned_channel_is_heard_and_is_not_a_call() {
        // Unmarked analogue speech is on the tap for the transcriber only.
        // Nothing about a mode and a frequency says it is a conversation, so
        // it is not a row in the call list.
        let mut h = HeardNode::new();
        h.track(&tuned(None), 0.01);
        assert!(h.take_calls().is_empty());
        assert!(h.levels().is_empty());
    }

    #[test]
    fn a_channel_marked_as_voice_is_a_call() {
        // The voice mark is the operator saying people talk on this channel,
        // which is exactly what the call list asks of a decoder. The agent's
        // own channel is one of these.
        let mut h = HeardNode::new();
        h.track(&tuned(Some("CH1")), 0.01);
        let calls = h.take_calls();
        assert_eq!(calls.len(), 1);
        assert_eq!(calls[0].to, "CH1");
        assert_eq!(calls[0].system, super::super::fader::ANALOGUE);
        assert_eq!(calls[0].channel_hz, 145_500_000.0);
        assert_eq!(h.levels().len(), 1);
    }
}
