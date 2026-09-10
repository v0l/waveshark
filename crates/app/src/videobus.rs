//! The video bus: every picture the receiver has, in one place.
//!
//! The counterpart of [`crate::audiobus`], and it exists for the same reason:
//! a picture that arrives somewhere other than the graph is a picture no
//! patch can route, no view can find and no recorder can take. Every front
//! end that produces an image ends here.
//!
//! # Why it does not mix
//!
//! Audio sums. Two transmissions at once are two voices in a room, and a
//! mixer is the right instrument. Video does not: two pictures added together
//! are neither picture. So this bus selects rather than mixes. It keeps the
//! latest field from every input, so a view can show thumbnails of everything
//! being received, and it publishes one of them as the output, which is what
//! a full-screen view and a recorder read.
//!
//! What carries over from the audio bus is everything else: one input per
//! strip, a label per strip, the last input always spare so a chain drawn by
//! hand has somewhere to go, and subscriptions deciding what is watched
//! rather than "whatever arrived last", because one video port carries every
//! channel a receiver is watching.
//!
//! # What a picture is worth
//!
//! Analogue video has no integrity check anywhere, so [`VideoFrame::lines_seen`]
//! is the only quality a viewer has. The bus keeps it and does not smooth
//! over it: a field assembled from a third of its lines is offered as such,
//! and [`VideoBus::watched`] prefers a complete picture over a fragment when
//! a rule matches more than one input.

use common::{Result, VideoFrame};
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};

/// What a subscription matches on.
///
/// Data rather than a closure, for the reason the audio bus's rules are: the
/// set is edited in the interface, saved with the session, and has to be
/// comparable so a rebuild can tell whether anything changed.
///
/// Two rules, because two are what the interface offers. The audio bus grew
/// rules for a caller, a group and a system because an operator following a
/// conversation needs them; a viewer chooses a picture by pointing at it, and
/// a rule set nobody can reach from the screen is a rule set nobody has
/// tested.
#[derive(Clone, Debug, PartialEq)]
pub enum Rule {
    /// Whatever is being received, wherever: the bus then shows the most
    /// complete picture it has.
    Everything,
    /// One transmission, by the key its channel is kept under.
    ///
    /// Not an input of the bus: everything the auto node finds arrives on one
    /// port, so keying on the port collapsed four cameras into one picture
    /// that flickered between them. What tells them apart is where they were
    /// received, which every field carries.
    Channel(String),
}

impl Rule {
    pub fn matches(&self, f: &VideoFrame) -> bool {
        match self {
            Rule::Everything => true,
            Rule::Channel(k) => *k == key_of(f),
        }
    }
}

/// What tells one transmission from another: the system and where it was
/// received, to the kilohertz. Two cameras on adjacent channels of the plan
/// are 19 MHz apart and nothing rounds them together.
pub fn key_of(f: &VideoFrame) -> String {
    format!("{}:{}", f.system, (f.channel_hz / 1e3).round() as i64)
}

/// One transmission the bus has seen, and the last picture from it.
#[derive(Clone, Debug, Default)]
pub struct Channel {
    /// What it is kept under; see [`key_of`].
    pub key: String,
    /// What to call it: the channel of the plan where the band has one, else
    /// the frequency.
    pub label: String,
    pub channel_hz: f64,
    /// Ignore this one entirely.
    pub muted: bool,
    pub last: Option<VideoFrame>,
    /// Fields that have arrived on it, so a view can show a rate and tell a
    /// live channel from one that stopped.
    pub fields: u64,
    /// Seconds since the last field, so a channel that stopped can be shown
    /// as stopped rather than dropped from the list mid-glance.
    pub since_s: f64,
}

impl Channel {
    pub fn is_fed(&self) -> bool {
        self.fields > 0
    }

    /// Whether it is still arriving.
    pub fn live(&self) -> bool {
        self.since_s <= HOLD_S
    }
}

pub struct VideoBus {
    /// One per transmission, in the order they were first heard.
    channels: Vec<Channel>,
    rules: Vec<Rule>,
    /// The picture published this block, if any.
    out: Option<VideoFrame>,
    /// And the last one published, kept for whoever is looking.
    ///
    /// A field is one block in fifty: a camera sends fifty a second and the
    /// graph runs a thousand blocks a second, so a view that polls this
    /// between fields used to find nothing and the pane stayed empty while
    /// the chain view showed fifteen fields a second arriving.
    held: Option<VideoFrame>,
    /// How long since one arrived, so a picture is not held on the screen
    /// after the transmitter has gone.
    since_s: f64,
}

impl Default for VideoBus {
    fn default() -> Self {
        Self::new()
    }
}

impl VideoBus {
    pub fn new() -> Self {
        Self {
            channels: Vec::new(),
            rules: vec![Rule::Everything],
            out: None,
            held: None,
            since_s: f64::INFINITY,
        }
    }

    pub fn channels(&self) -> &[Channel] {
        &self.channels
    }

    pub fn channel(&self, key: &str) -> Option<&Channel> {
        self.channels.iter().find(|c| c.key == key)
    }

    pub fn channel_mut(&mut self, key: &str) -> Option<&mut Channel> {
        self.channels.iter_mut().find(|c| c.key == key)
    }

    pub fn set_rules(&mut self, rules: Vec<Rule>) {
        self.rules = rules;
    }

    /// Take a field, whichever input it arrived on.
    pub fn push(&mut self, _input: usize, frame: VideoFrame) {
        let key = key_of(&frame);
        let label =
            frame.label.clone().unwrap_or_else(|| format!("{:.3} MHz", frame.channel_hz / 1e6));
        let k = match self.channels.iter().position(|c| c.key == key) {
            Some(k) => k,
            None => {
                self.channels.push(Channel {
                    key: key.clone(),
                    label,
                    channel_hz: frame.channel_hz,
                    ..Default::default()
                });
                while self.channels.len() > MAX_CHANNELS {
                    self.channels.remove(0);
                }
                self.channels.len() - 1
            }
        };
        let c = &mut self.channels[k];
        c.fields += 1;
        c.since_s = 0.0;
        c.last = Some(frame.clone());
        if c.muted {
            return;
        }
        let f = frame;
        if !self.rules.iter().any(|r| r.matches(&f)) {
            return;
        }
        // A more complete picture wins. Two inputs matching one rule is a
        // receiver watching two channels at once, and showing the one that
        // arrived last would flicker between them; showing the better of the
        // two is at least a decision.
        let better = self.out.as_ref().is_none_or(|cur| f.completeness() > cur.completeness());
        if better {
            self.out = Some(f.clone());
        }
        let fresher = self
            .held
            .as_ref()
            .is_none_or(|cur| self.since_s > 0.0 || f.completeness() >= cur.completeness());
        if fresher {
            self.held = Some(f);
        }
        self.since_s = 0.0;
    }

    /// What should be shown, or `None` when nothing has arrived lately.
    ///
    /// Held rather than published per block, because a viewer polls at the
    /// screen's rate and a field arrives on one block in fifty. Dropped after
    /// [`HOLD_S`] of silence: a still picture of a transmitter that has gone
    /// away is the worst thing this bus could hand a view.
    pub fn watched(&self) -> Option<&VideoFrame> {
        (self.since_s <= HOLD_S).then(|| self.held.as_ref()).flatten()
    }

    /// The picture to publish on the output port this block, which is only
    /// the one that arrived.
    pub fn published(&self) -> Option<&VideoFrame> {
        self.out.as_ref()
    }

    /// Time passing. Every channel ages; the ones that went away stop being
    /// live and the held picture is dropped once nothing is arriving.
    pub fn idle(&mut self, seconds: f64) {
        let dt = seconds.max(0.0);
        self.since_s += dt;
        for c in &mut self.channels {
            c.since_s += dt;
        }
    }

    /// The channels with a picture in the last [`HOLD_S`], newest first.
    pub fn live(&self) -> impl Iterator<Item = &Channel> {
        self.channels.iter().filter(|c| c.live())
    }

    /// The last field of every channel, for a view of everything at once.
    pub fn thumbnails(&self) -> impl Iterator<Item = (&str, &VideoFrame)> {
        self.channels.iter().filter_map(|c| c.last.as_ref().map(|f| (c.key.as_str(), f)))
    }

    pub fn clear(&mut self) {
        self.out = None;
    }

    /// The picture being shown, without forgetting the channels.
    pub fn forget_watched(&mut self) {
        self.out = None;
        self.held = None;
        self.since_s = f64::INFINITY;
    }

    /// Everything, for a receiver that has stopped or been retuned.
    pub fn forget(&mut self) {
        self.out = None;
        self.held = None;
        self.since_s = f64::INFINITY;
        self.channels.clear();
    }
}

/// How long a picture is worth showing after the last field arrived. Long
/// enough to ride a dropout on a fading link, short enough that nobody
/// mistakes a still of a departed transmitter for a live picture.
const HOLD_S: f64 = 0.5;

/// Transmissions kept. A band holds forty channels of the analogue plan and
/// nothing tunes across more than one band at a time.
const MAX_CHANNELS: usize = 64;

/// The bus as a node.
pub struct VideoBusNode {
    bus: VideoBus,
    inputs: usize,
}

impl Default for VideoBusNode {
    fn default() -> Self {
        Self::new()
    }
}

impl VideoBusNode {
    pub fn new() -> Self {
        Self { bus: VideoBus::new(), inputs: 1 }
    }

    pub fn bus(&self) -> &VideoBus {
        &self.bus
    }

    pub fn bus_mut(&mut self) -> &mut VideoBus {
        &mut self.bus
    }

    fn muted_param(name: &str) -> Option<&str> {
        name.strip_prefix("mute:")
    }
}

impl pipeline::node::Node for VideoBusNode {
    fn name(&self) -> &str {
        "video_bus"
    }

    /// One per feed drawn into it, plus the spare a chain drawn by hand
    /// lands on. The transmissions are told apart by what they carry, not by
    /// which wire brought them.
    fn num_inputs(&self) -> usize {
        self.inputs.max(1)
    }

    /// A bus has a spare input by nature.
    fn optional_inputs(&self) -> bool {
        true
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        for (k, i) in inputs.iter().enumerate() {
            // The spare input nothing is wired into arrives as silence, the
            // way it does on the audio bus: a bus has one by nature, and it
            // is what a chain drawn by hand is dropped onto.
            let spare = i.spec.kind == PortKind::Real && i.spec.is_silence();
            if i.spec.kind != PortKind::Video && !spare {
                return Err(common::Error::other(format!(
                    "the video bus takes pictures, and input {k} carries {:?}",
                    i.spec.kind
                )));
            }
        }
        Ok(vec![StreamSpec {
            kind: PortKind::Video,
            rate: 0.0,
            center: common::Hz(0),
            bandwidth: 0.0,
            ..Default::default()
        }])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        _ctx: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let mut arrived = false;
        for (k, p) in inputs.iter().enumerate() {
            for f in p.as_video().unwrap_or(&[]) {
                self.bus.push(k, f.clone());
                arrived = true;
            }
        }
        if !arrived {
            self.bus.idle(_ctx.block_seconds);
        }
        if let Some(f) = self.bus.published().cloned() {
            outputs[0].video_mut().push(f);
        }
        self.bus.clear();
        Ok(())
    }

    fn reset(&mut self) {
        self.bus.forget();
    }

    /// A switch per transmission rather than per wire, named after the
    /// channel: which input a picture arrived on is not something an operator
    /// can see or would recognise.
    fn params(&self) -> Vec<Param> {
        let mut p = vec![Param::int("inputs", self.inputs as i64, 1..=32).label("Inputs")];
        for c in self.bus.channels() {
            p.push(
                Param::bool(&format!("mute:{}", c.key), c.muted).label(&format!("{} off", c.label)),
            );
        }
        p
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        if name == "inputs" {
            self.inputs = v.as_i64().unwrap_or(1).max(1) as usize;
            return Ok(());
        }
        match Self::muted_param(name) {
            Some(key) => {
                if let Some(c) = self.bus.channel_mut(key) {
                    c.muted = v.as_bool().unwrap_or(false);
                }
                Ok(())
            }
            None => Err(common::Error::other(format!("the video bus has no {name}"))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::Pixels;

    fn frame(channel_hz: f64, label: &str, lines: usize) -> VideoFrame {
        VideoFrame {
            system: "analogue video",
            channel_hz,
            label: Some(label.into()),
            width: 4,
            height: 288,
            aspect: 4.0 / 3.0,
            pixels: Pixels::Luma8,
            samples: std::sync::Arc::new(vec![0u8; 4 * 288]),
            lines_seen: lines,
            sequence: 1,
        }
    }

    /// Two cameras on one wire are two channels. Everything the auto node
    /// finds arrives on its one video port, so keying on the port collapsed
    /// them into a single picture that flickered between the two.
    #[test]
    fn the_bus_keeps_a_channel_per_transmission_not_per_wire() {
        let mut bus = VideoBus::new();
        bus.push(0, frame(5_800e6, "F4", 288));
        bus.push(0, frame(5_865e6, "A1 or B8", 288));
        assert_eq!(bus.thumbnails().count(), 2);
        assert_eq!(bus.channels()[0].fields, 1);
        assert_eq!(bus.channels()[0].label, "F4");
        // And the same camera again is the same channel, whatever wire it
        // came in on.
        bus.push(1, frame(5_800e6, "F4", 288));
        assert_eq!(bus.channels().len(), 2);
        assert_eq!(bus.channels()[0].fields, 2);
    }

    /// Pictures do not sum, so the bus chooses. Where a rule matches two
    /// inputs, the more complete picture wins rather than whichever arrived
    /// last, which would flicker between two channels.
    #[test]
    fn a_fragment_does_not_displace_a_whole_picture() {
        let mut bus = VideoBus::new();
        bus.push(0, frame(5_800e6, "F4", 288));
        bus.push(1, frame(5_865e6, "A1 or B8", 60));
        let w = bus.watched().expect("something watched");
        assert_eq!(w.lines_seen, 288);
        assert_eq!(w.label.as_deref(), Some("F4"));
    }

    #[test]
    fn a_subscription_decides_what_is_watched() {
        let mut bus = VideoBus::new();
        // The chosen input wins even though the other picture is more
        // complete, which is the point: an operator watching one channel is
        // not asking for the best signal.
        bus.push(0, frame(5_865e6, "A1 or B8", 200));
        let key = bus.channels()[0].key.clone();
        bus.set_rules(vec![Rule::Channel(key)]);
        bus.push(0, frame(5_800e6, "F4", 288));
        bus.push(0, frame(5_865e6, "A1 or B8", 200));
        assert_eq!(bus.watched().and_then(|f| f.label.as_deref()), Some("A1 or B8"));
    }

    /// A muted channel is still received and still shows a thumbnail: muting
    /// says "do not put this on the screen", not "stop looking".
    #[test]
    fn a_muted_channel_is_still_received() {
        let mut bus = VideoBus::new();
        bus.push(0, frame(5_800e6, "F4", 288));
        let key = bus.channels()[0].key.clone();
        bus.channel_mut(&key).unwrap().muted = true;
        bus.forget_watched();
        bus.push(0, frame(5_800e6, "F4", 288));
        assert!(bus.watched().is_none(), "a muted channel is not shown");
        assert_eq!(bus.thumbnails().count(), 1, "but it is still received");
        assert_eq!(bus.channels()[0].fields, 2);
    }

    #[test]
    fn the_bus_refuses_a_port_that_does_not_carry_pictures() {
        use pipeline::node::Node;
        let mut n = VideoBusNode::new();
        let audio = PortSpec {
            spec: StreamSpec { kind: PortKind::Real, rate: 48_000.0, ..Default::default() },
            latency: 0,
        };
        assert!(n.negotiate(&[audio]).is_err());
    }

    /// A field arrives on one block in fifty and a view polls between them,
    /// so the bus holds the last picture rather than publishing it for the
    /// single block it landed in. The pane was empty for exactly this reason
    /// while the chain view showed fifteen fields a second going in.
    #[test]
    fn a_picture_is_still_there_between_fields() {
        let mut bus = VideoBus::new();
        bus.push(0, frame(5_865_000_000.0, "A1", 280));
        assert!(bus.watched().is_some());
        bus.clear();
        for _ in 0..40 {
            bus.idle(0.01);
            assert!(bus.watched().is_some(), "the picture went between fields");
        }
    }

    /// And it goes when the transmitter does. A still of something that has
    /// left the air is the one thing a video pane must not show.
    #[test]
    fn a_picture_does_not_outlive_the_transmission() {
        let mut bus = VideoBus::new();
        bus.push(0, frame(5_865_000_000.0, "A1", 280));
        // The block it arrived in publishes it on the port; later blocks
        // publish nothing, and the held picture goes with the transmission.
        assert!(bus.published().is_some());
        bus.clear();
        assert!(bus.published().is_none());
        bus.idle(HOLD_S + 0.01);
        assert!(bus.watched().is_none());
    }
}
