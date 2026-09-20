//! Who is talking on an analogue channel, and which group, off the audio.
//!
//! A demodulated channel says nothing about the station on it: an FM carrier
//! carries a voice and no identity. Two things in the audio fill that in, and
//! this stage reads both and labels the speech with them.
//!
//! **Who**, from the unit number a radio sends as DTMF when the key goes
//! down, at the end of the over, or both. Baofeng calls it PTT-ID, Kenwood
//! and Motorola call it ANI, and it is the same tones: [`dsp::dtmf`] for the
//! digits and [`decode::dtmf`] for the sequences they make. It is off by
//! default on every radio that has it, so an over with no number is the
//! ordinary case.
//!
//! **Which group**, from the coded squelch, which the squelch stage reads and
//! publishes as a tag: it is the half of a squelch that decides whom to hear,
//! and it is read where the tone is still in the audio. Two users of one
//! frequency with different codes cannot hear each other, so the code is what
//! tells their traffic apart, and it is the only identity most analogue
//! traffic carries.
//!
//! The identity belongs to one over and is dropped when the squelch shuts:
//! whoever keys up next is somebody else until they say who they are. The
//! group is not, because a coded squelch describes the traffic on the channel
//! rather than one transmission, so it is kept until a different one is
//! heard.

use common::Result;
use pipeline::event::Event;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, StageDesc};

/// Below this peak the channel is quiet. The same floor the audio bus and the
/// tap use to decide whether anybody is talking.
const FLOOR: f32 = 0.004;

/// How long the channel has to be quiet before the identity is dropped, in
/// seconds.
///
/// An identity belongs to one over and the squelch shutting is the over
/// ending, so this is not a hang time: it is the slack for a block boundary
/// and a squelch tail, and nothing more. Whoever keys up next is nobody
/// until they say who they are, because a station replying with PTT-ID
/// switched off published under the previous station's number is worse than
/// one published with no number at all.
///
/// A squelched channel delivers true silence, so this measures the squelch
/// and not a pause in speech: a gap inside an over keeps the carrier and
/// stays above the floor.
const HOLD_S: f64 = 0.2;

pub struct IdentNode {
    dtmf: dsp::dtmf::Dtmf,
    runs: decode::dtmf::Sequences,
    /// The group heard now, as a radio names it: "141.3" or "D023".
    group: Option<String>,
    /// Who is on the channel now, and how long the channel has been quiet.
    caller: Option<String>,
    quiet_s: f64,
    /// What the channel is called, which is what the tap keys a conversation
    /// by, and where it is.
    label: String,
    channel_hz: f64,
    /// Whether the identity is read at all. Off makes this a stage that
    /// labels the channel and nothing more.
    enabled: bool,
    /// The clock the digits are timed against: the audio's own.
    fed_s: f64,
    rate: f64,
    channels: usize,
    /// Identities read since the graph was built, for a readout.
    reads: u64,
    /// The block the last sequence was read from, so a statement made here
    /// can say how loud the audio it came off was.
    heard: Vec<f32>,
}

impl Default for IdentNode {
    fn default() -> Self {
        Self::new()
    }
}

impl IdentNode {
    pub fn new() -> Self {
        Self {
            dtmf: dsp::dtmf::Dtmf::new(1.0),
            runs: decode::dtmf::Sequences::new(),
            group: None,
            caller: None,
            quiet_s: 0.0,
            label: String::new(),
            channel_hz: 0.0,
            enabled: true,
            fed_s: 0.0,
            rate: 1.0,
            channels: 1,
            reads: 0,
            heard: Vec::new(),
        }
    }

    /// Who the channel is hearing, while it is hearing them.
    pub fn caller(&self) -> Option<&str> {
        self.caller.as_deref()
    }

    /// The group on the channel now, from its coded squelch: a CTCSS tone as
    /// "141.3", or a DCS code as "D023".
    pub fn group(&self) -> Option<&str> {
        self.group.as_deref()
    }

    /// What the conversation is called: the channel, and nothing else.
    ///
    /// The group is deliberately not in it. A coded squelch takes half a
    /// second of audio to read, so putting it in the name changed the name
    /// part way through the over, and the conversation key is what the call
    /// list, the recorder and the transcriber all file by: one transmission
    /// became two rows, two recordings and two lines of transcript, the
    /// first of each unnamed. The group is published beside the audio
    /// instead, on this stage and as a decode.
    fn called(&self) -> String {
        self.label.clone()
    }

    /// Digits read and not yet settled into an identity, for a readout: a
    /// sequence arriving looks like nothing at all otherwise.
    pub fn held(&self) -> &str {
        self.runs.held()
    }

    pub fn reads(&self) -> u64 {
        self.reads
    }

    /// The group, said once where the chain view and the log can see it.
    fn said_group(&mut self, c: &mut NodeCtx<'_>) {
        let Some(group) = self.group.clone() else { return };
        let carrier = crate::off_audio(self.channel_hz as u64, 0, &self.heard);
        c.emit(Event::Decoded(
            common::packet::Packet::heard(carrier).decoded(
                common::packet::Proto::new("ident", "coded_squelch")
                    .between(common::packet::Link::from(common::packet::Party::group(group))),
            ),
        ));
    }

    /// One sequence, which is an identity or somebody pressing keys.
    fn settle(&mut self, seq: &decode::dtmf::Sequence, c: &mut NodeCtx<'_>) {
        let Some(id) = seq.id() else { return };
        let fresh = self.caller.as_deref() != Some(id);
        self.caller = Some(id.to_string());
        self.reads += 1;
        if !fresh {
            return;
        }
        // An identity is a fact about the channel, so it is said once, where
        // the chain view and the log can see it.
        let carrier = crate::off_audio(self.channel_hz as u64, 0, &self.heard);
        c.emit(Event::Decoded(
            common::packet::Packet::heard(carrier).decoded(
                common::packet::Proto::new("ident", "ptt_id")
                    .by(common::packet::Entity::new(
                        "radio-unit",
                        common::packet::Id::Text(id.to_string()),
                    ))
                    .between(common::packet::Link::from(common::packet::Party::unit(id))),
            ),
        ));
    }
}

impl Simple for IdentNode {
    fn name(&self) -> &str {
        "ident"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Real {
            return Err(common::Error::other("the identity stage reads channel audio"));
        }
        self.rate = i.spec.frame_rate().max(1.0);
        self.channels = i.spec.channels.max(1);
        // Where the channel is, taken from the stream rather than set: the
        // tap keys a conversation by frequency, and the frequency is what
        // the graph already agreed this audio came from.
        self.channel_hz = i.spec.center.as_f64();
        self.dtmf = dsp::dtmf::Dtmf::new(self.rate);
        self.runs = decode::dtmf::Sequences::new();
        Ok(StreamSpec {
            kind: PortKind::Voice,
            rate: self.rate * self.channels as f64,
            center: i.spec.center,
            bandwidth: 0.0,
            channels: self.channels,
            ..Default::default()
        })
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("enabled", self.enabled).label("Read the identity and the group"),
            Param::text("label", self.label.clone()).label("Channel"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => {
                self.enabled = v.as_bool().unwrap_or(self.enabled);
                if !self.enabled {
                    self.caller = None;
                    self.group = None;
                    self.runs.take();
                    self.dtmf.reset();
                }
                Ok(())
            }
            "label" => {
                self.label = v.as_str().unwrap_or_default().to_string();
                Ok(())
            }
            _ => Err(common::Error::other(format!("no parameter {name}"))),
        }
    }

    fn readings(&self) -> Vec<(String, String)> {
        let mut out = Vec::new();
        if let Some(g) = &self.group {
            out.push(("group".into(), g.clone()));
        }
        if let Some(c) = &self.caller {
            out.push(("caller".into(), c.clone()));
        }
        if !self.runs.held().is_empty() {
            out.push(("digits".into(), self.runs.held().to_string()));
        }
        if self.reads > 0 {
            out.push(("read".into(), self.reads.to_string()));
        }
        out
    }

    fn process(&mut self, i: &Payload, o: &mut Payload, c: &mut NodeCtx<'_>) -> Result<()> {
        let Some(pcm) = i.as_real() else { return Ok(()) };
        if pcm.is_empty() {
            return Ok(());
        }
        self.heard.clear();
        self.heard.extend_from_slice(pcm);
        let frames = pcm.len() / self.channels.max(1);
        let block_s = frames as f64 / self.rate;
        self.fed_s += block_s;
        let loud = pcm.iter().any(|s| s.abs() > FLOOR);
        if self.enabled {
            // Mono, because the tones are on both sides of anything stereo
            // and a pair detector wants one signal.
            let mono: Vec<f32> = match self.channels {
                1 => pcm.to_vec(),
                ch => pcm.chunks(ch).map(|f| f.iter().sum::<f32>() / ch as f32).collect(),
            };
            // The group, from the squelch that read it. It is the half of a
            // squelch that decides whom to hear, so it is read where the
            // tone is still in the audio; by here it has been filtered out
            // and there would be nothing to read.
            let read = c.in_tags(0).iter().find_map(|t| match (t.key, &t.value) {
                ("squelch_code", pipeline::port::TagValue::Text(code)) => Some(code.clone()),
                _ => None,
            });
            if let Some(group) = read.filter(|g| Some(g) != self.group.as_ref()) {
                self.group = Some(group);
                self.said_group(c);
            }
            for digit in self.dtmf.push(&mono) {
                if let Some(seq) = self.runs.digit(digit.key, digit.at_s, digit.level_db) {
                    self.settle(&seq, c);
                }
            }
            if let Some(seq) = self.runs.settled(self.fed_s) {
                self.runs.take();
                self.settle(&seq, c);
            }
        }
        // The identity lasts as long as the over does. A squelched channel
        // delivers silence, and silence is the over ending.
        match loud {
            true => self.quiet_s = 0.0,
            false => {
                self.quiet_s += block_s;
                // On the block the channel goes quiet on, and not on every
                // one after it.
                if self.quiet_s >= HOLD_S && self.quiet_s - block_s < HOLD_S {
                    // The group is not dropped with the over. A coded
                    // squelch is a property of the traffic on the channel
                    // rather than of one transmission, and it takes half a
                    // second of audio to read: dropped at every squelch
                    // close, every over began under the bare channel name
                    // and was relabelled once the code came through. It is
                    // replaced when a different one is heard.
                    // An identity sent at the end of the over arrives just
                    // before the squelch shuts, so what is held is settled
                    // rather than thrown away.
                    if let Some(seq) = self.runs.take() {
                        self.settle(&seq, c);
                    }
                    self.caller = None;
                    self.dtmf.reset();
                }
            }
        }
        o.voice_mut().push(common::Voice {
            system: common::ANALOGUE,
            channel_hz: self.channel_hz,
            // Named after the channel, which is how the tap keys a
            // conversation nobody decoded.
            to: Some(self.called()),
            from: self.caller.clone(),
            code: self.group.clone(),
            over: None,
            rate: self.rate,
            channels: self.channels,
            pcm: pcm.to_vec(),
        });
        Ok(())
    }

    fn reset(&mut self) {
        self.dtmf.reset();
        self.runs.take();
        self.caller = None;
        self.group = None;
        self.quiet_s = 0.0;
        self.fed_s = 0.0;
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "ident",
    summary: "Read the coded squelch and a radio's PTT-ID off channel audio, and say who is \
              talking and in which group",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(_s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    Ok(Box::new(IdentNode::new()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use pipeline::node::Node;

    const RATE: f64 = 8_000.0;

    /// A node negotiated on one channel of audio at `RATE`.
    fn node(label: &str) -> IdentNode {
        let mut n = IdentNode::new();
        let spec = StreamSpec {
            kind: PortKind::Real,
            rate: RATE,
            center: common::Hz(145_500_000),
            channels: 1,
            ..Default::default()
        };
        Simple::negotiate(&mut n, &PortSpec { spec, latency: 0 }).expect("audio in, speech out");
        Simple::set_param(&mut n, "label", ParamValue::Text(label.into())).unwrap();
        n
    }

    /// One block through, and what it published.
    fn run(n: &mut IdentNode, pcm: &[f32]) -> Vec<common::Voice> {
        run_tagged(n, pcm, None)
    }

    /// The same, with what the squelch in front of it said about the group.
    fn run_tagged(n: &mut IdentNode, pcm: &[f32], code: Option<&str>) -> Vec<common::Voice> {
        let spec =
            StreamSpec { kind: PortKind::Real, rate: RATE, channels: 1, ..Default::default() };
        let ins = [PortSpec { spec, latency: 0 }];
        let said: Vec<pipeline::port::Tag> = code
            .map(|c| {
                vec![pipeline::port::Tag::new(
                    0,
                    "squelch_code",
                    pipeline::port::TagValue::Text(c.to_string()),
                )]
            })
            .unwrap_or_default();
        let tags = [&said[..]];
        let (mut events, mut new_tags) = (Vec::new(), Vec::new());
        let mut out = [Payload::Voice(Vec::new())];
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        ctx.block_seconds = pcm.len() as f64 / RATE;
        Node::process(n, &[&Payload::Real(pcm.to_vec())], &mut out, &mut ctx).expect("a block");
        out[0].as_voice().unwrap_or(&[]).to_vec()
    }

    /// The tones of a dialled string, at fifty milliseconds a digit with
    /// fifty of silence between, which is what a radio sends.
    fn dialled(digits: &str) -> Vec<f32> {
        let mut out = Vec::new();
        for key in digits.chars() {
            let (row, col) = [
                ['1', '2', '3', 'A'],
                ['4', '5', '6', 'B'],
                ['7', '8', '9', 'C'],
                ['*', '0', '#', 'D'],
            ]
            .iter()
            .enumerate()
            .find_map(|(r, cols)| cols.iter().position(|k| *k == key).map(|c| (r, c)))
            .expect("a key on the pad");
            let n = (RATE * 0.05) as usize;
            out.extend((0..n).map(|i| {
                let t = i as f64 / RATE;
                let a = (std::f64::consts::TAU * dsp::dtmf::LOW[row] * t).sin();
                let b = (std::f64::consts::TAU * dsp::dtmf::HIGH[col] * t).sin();
                ((a + b) * 0.25) as f32
            }));
            out.extend(vec![0.0f32; n]);
        }
        out
    }

    /// Speech at a plausible level, standing in for somebody talking.
    ///
    /// The pitch moves, because a steady one is a tone: a voice held at one
    /// frequency reads as coded squelch, and telling the two apart is what
    /// the group detector is for.
    fn talking(seconds: f64) -> Vec<f32> {
        let n = (RATE * seconds) as usize;
        let mut phase = [0.0f64; 11];
        (0..n)
            .map(|i| {
                let t = i as f64 / RATE;
                let pitch = 190.0 + 40.0 * (std::f64::consts::TAU * 2.0 * t).sin();
                let mut v = 0.0;
                for h in 1..=10 {
                    phase[h] += std::f64::consts::TAU * pitch * h as f64 / RATE;
                    v += phase[h].sin() / h as f64;
                }
                (v * 0.2) as f32
            })
            .collect()
    }

    /// The whole point: a radio sends its number when the key goes down, and
    /// the speech that follows is published as that station's.
    #[test]
    fn an_over_that_identifies_itself_is_published_with_the_caller() {
        let mut n = node("CH1");
        // Nobody has said anything, so the audio is the channel's and
        // nobody's in particular.
        let quiet = run(&mut n, &talking(0.2));
        assert_eq!(quiet.len(), 1);
        assert_eq!(quiet[0].to.as_deref(), Some("CH1"));
        assert_eq!(quiet[0].from, None, "an over with no identity names nobody");
        assert_eq!(quiet[0].system, common::ANALOGUE);
        assert_eq!(quiet[0].channel_hz, 145_500_000.0);

        // The key goes down with PTT-ID on: three digits, then the voice.
        run(&mut n, &dialled("123"));
        // The gap after the last digit settles the sequence.
        let said = run(&mut n, &talking(0.6));
        assert_eq!(n.caller(), Some("123"));
        assert_eq!(said[0].from.as_deref(), Some("123"), "the speech is that radio's");
        assert_eq!(n.reads(), 1);

        // More of the same over is still theirs.
        let more = run(&mut n, &talking(0.5));
        assert_eq!(more[0].from.as_deref(), Some("123"));

        // The over ends. The next one is somebody else until they say
        // otherwise, so the identity goes with the silence.
        run(&mut n, &vec![0.0f32; (RATE * (HOLD_S + 0.5)) as usize]);
        assert_eq!(n.caller(), None, "the identity outlived the over");
        let next = run(&mut n, &talking(0.3));
        assert_eq!(next[0].from, None);
    }

    /// A second radio identifying itself takes the channel: the caller is
    /// whoever last said who they were, not the first one heard.
    #[test]
    fn a_second_station_takes_the_channel() {
        let mut n = node("CH1");
        run(&mut n, &dialled("123"));
        run(&mut n, &talking(0.6));
        assert_eq!(n.caller(), Some("123"));
        run(&mut n, &dialled("456"));
        let said = run(&mut n, &talking(0.6));
        assert_eq!(said[0].from.as_deref(), Some("456"));
        assert_eq!(n.reads(), 2);
    }

    /// A station answering with no identity of its own is nobody, not the
    /// station before it.
    ///
    /// Two radios taking turns on one channel, one of them with PTT-ID
    /// switched off, which is the default on every radio that has it. The
    /// identity used to be held for two seconds past the audio, so the
    /// silent radio's over was published under the other one's number: the
    /// call list, the transcript and the agent all named the wrong station.
    #[test]
    fn a_reply_with_no_identity_is_not_the_station_before_it() {
        let mut n = node("CH1");
        run(&mut n, &dialled("123"));
        let first = run(&mut n, &talking(0.8));
        assert_eq!(first[0].from.as_deref(), Some("123"));

        // The over ends: the squelch shuts, which is silence.
        run(&mut n, &vec![0.0f32; (RATE * 0.7) as usize]);
        assert_eq!(n.caller(), None, "the identity outlived the over");

        // And the reply, keyed a moment later by a radio that sends nothing.
        let reply = run(&mut n, &talking(1.0));
        assert_eq!(reply[0].from, None, "somebody else's number");
        assert_eq!(reply[0].to.as_deref(), Some("CH1"), "still a call on the channel");
    }

    /// The group comes off the tag the squelch publishes, not off the audio.
    ///
    /// The tone is gone by here: the squelch reads it where it is still in
    /// the audio and filters it out on the way through, so a stage looking
    /// for it in the speech would find nothing and every over would be
    /// published under the bare channel name.
    #[test]
    fn the_group_is_whatever_the_squelch_said() {
        let mut n = node("CH1");
        let said = run_tagged(&mut n, &talking(0.5), Some("D023"));
        assert_eq!(said[0].code.as_deref(), Some("D023"));
        assert_eq!(n.group(), Some("D023"));

        // And it outlives the over: a coded squelch describes the traffic on
        // the channel rather than one transmission.
        run(&mut n, &vec![0.0f32; (RATE * 0.5) as usize]);
        let next = run(&mut n, &talking(0.5));
        assert_eq!(next[0].code.as_deref(), Some("D023"));
    }

    /// Keys pressed on a repeater are not an identity: one digit is a
    /// control, and a long string is a telephone number.
    #[test]
    fn a_control_code_is_not_an_identity() {
        let mut n = node("CH1");
        run(&mut n, &dialled("7"));
        run(&mut n, &talking(0.6));
        assert_eq!(n.caller(), None, "one digit became a station");
        run(&mut n, &dialled("0123456789"));
        run(&mut n, &talking(0.6));
        assert_eq!(n.caller(), None, "a dialled number became a station");
        assert_eq!(n.reads(), 0);
    }

    /// Switched off it is a labelling stage and nothing else, which is what
    /// a channel with no identities on it wants.
    #[test]
    fn reading_can_be_switched_off() {
        let mut n = node("CH1");
        Simple::set_param(&mut n, "enabled", ParamValue::Bool(false)).unwrap();
        run(&mut n, &dialled("123"));
        let said = run(&mut n, &talking(0.5));
        assert_eq!(n.caller(), None);
        assert_eq!(said[0].to.as_deref(), Some("CH1"), "still labelled for the call list");
    }
}
