use crate::NodeSpec;
use crate::protocol::{Placed, Placement, Protocol, Shape, Stickiness};
use common::C32;
use common::{Cadence, Pixels, Result, Update, VideoFrame};
use decode::display;
use decode::videoleak::{Locked, Read, Reader};
use identify::Signal;
pub use identify::tempest::{DEFAULT_HZ, MIN_RATE_HZ, Tempest};
use pipeline::event::Request;
use pipeline::node::{NodeCtx, PortSpec};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};

const SYSTEM: &str = "display leakage";

pub const LAG_S: f64 = 0.05;

enum Job {
    Block(Vec<C32>),
    Force(Option<&'static display::Mode>),
    Depth(f64),
    Nudge(isize, isize),
    Align(bool),
    Reset,
}

struct Done {
    samples: usize,
    us: u32,
    read: Read,
    seen: Seen,
    spent: Vec<C32>,
}

#[derive(Clone, Copy, Default)]
struct Seen {
    lock: Option<Locked>,
    frames: u64,
    held: (u64, u64),
    drift_ppm: f64,
}

impl Seen {
    fn of(r: &Reader) -> Self {
        Self {
            lock: r.locked().copied(),
            frames: r.frames(),
            held: r.held(),
            drift_ppm: r.drift_ppm(),
        }
    }
}

struct Worker {
    jobs: crossbeam_channel::Sender<Job>,
    done: crossbeam_channel::Receiver<Done>,
    ahead: usize,
}

impl Worker {
    fn spawn(mut reader: Reader) -> Option<Self> {
        let (jobs, work) = crossbeam_channel::unbounded::<Job>();
        let (finished, done) = crossbeam_channel::unbounded::<Done>();
        std::thread::Builder::new()
            .name("tempest".into())
            .spawn(move || {
                while let Ok(job) = work.recv() {
                    let block = match job {
                        Job::Block(b) => b,
                        Job::Force(m) => {
                            reader.force(m);
                            continue;
                        }
                        Job::Depth(d) => {
                            reader.set_depth(d);
                            continue;
                        }
                        Job::Nudge(x, y) => {
                            reader.set_nudge(x, y);
                            continue;
                        }
                        Job::Align(a) => {
                            reader.set_align(a);
                            continue;
                        }
                        Job::Reset => {
                            reader.reset();
                            continue;
                        }
                    };
                    let t = std::time::Instant::now();
                    let read = reader.push(&block);
                    let us = t.elapsed().as_micros().min(u32::MAX as u128) as u32;
                    let seen = Seen::of(&reader);
                    let done = Done { samples: block.len(), us, read, seen, spent: block };
                    if finished.send(done).is_err() {
                        return;
                    }
                }
            })
            .ok()?;
        Some(Self { jobs, done, ahead: 0 })
    }
}

pub struct TempestNode {
    worker: Option<Worker>,
    seen: Seen,
    spare: Vec<Vec<C32>>,
    reading: pipeline::cost::Ring,
    forced: Option<&'static display::Mode>,
    depth: i64,
    nudge: (i64, i64),
    align: bool,
    rate: f64,
    center_hz: f64,
}

impl Default for TempestNode {
    fn default() -> Self {
        Self::new(None)
    }
}

impl TempestNode {
    pub fn new(forced: Option<&'static display::Mode>) -> Self {
        Self {
            worker: None,
            seen: Seen::default(),
            spare: Vec::new(),
            reading: pipeline::cost::Ring::default(),
            forced,
            depth: DEFAULT_DEPTH,
            nudge: (0, 0),
            align: true,
            rate: MIN_RATE_HZ,
            center_hz: 0.0,
        }
    }

    fn send(&self, job: Job) {
        if let Some(w) = &self.worker {
            let _ = w.jobs.send(job);
        }
    }

    fn send_settings(&self) {
        self.send(Job::Force(self.forced));
        self.send(Job::Depth(self.depth as f64));
        self.send(Job::Nudge(self.nudge.0 as isize, self.nudge.1 as isize));
        self.send(Job::Align(self.align));
    }

    fn take(&mut self, done: Done, out: &mut Payload, c: &mut NodeCtx<'_>) {
        self.seen = done.seen;
        self.reading.push(done.us, done.samples as f64 / self.rate);
        self.spare.push(done.spent);
        if done.read.locked {
            c.request(Request::Claim {
                lo_hz: self.center_hz - self.rate / 2.0,
                hi_hz: self.center_hz + self.rate / 2.0,
            });
        }
        if done.read.released {
            c.request(Request::Release);
        }
        let Some(p) = done.read.picture else { return };
        out.video_mut().push(VideoFrame {
            system: SYSTEM,
            channel_hz: self.center_hz,
            label: self.seen.lock.map(|l| l.label()),
            width: p.width,
            height: p.height,
            aspect: p.aspect,
            pixels: Pixels::Luma8,
            lines_seen: p.height,
            samples: std::sync::Arc::new(p.gray),
            sequence: p.sequence,
            update: Update::Whole,
            cadence: Cadence::Live,
            sent_at_us: None,
        });
    }
}

const DEFAULT_DEPTH: i64 = 8;

impl pipeline::node::Node for TempestNode {
    fn name(&self) -> &str {
        "tempest"
    }

    fn num_inputs(&self) -> usize {
        1
    }

    fn num_outputs(&self) -> usize {
        1
    }

    fn negotiate(&mut self, inputs: &[PortSpec]) -> Result<Vec<StreamSpec>> {
        let i = &inputs[0];
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("a screen is read off complex baseband"));
        }
        if i.spec.rate < MIN_RATE_HZ {
            return Err(common::Error::other(format!(
                "a screen needs {:.0} MS/s to be more than a smudge",
                MIN_RATE_HZ / 1e6
            )));
        }
        self.rate = i.spec.rate;
        self.center_hz = i.spec.center.as_f64();
        let mut reader = Reader::new(self.rate);
        reader.set_dial(self.center_hz);
        self.worker = Worker::spawn(reader);
        self.seen = Seen::default();
        self.send_settings();
        let mut out = i.spec.with_kind(PortKind::Video);
        out.rate = 0.0;
        Ok(vec![out])
    }

    fn process(
        &mut self,
        inputs: &[&Payload],
        outputs: &mut [Payload],
        c: &mut NodeCtx<'_>,
    ) -> Result<()> {
        let Some(iq) = inputs[0].as_iq() else { return Ok(()) };
        let lag = (LAG_S * self.rate) as usize;
        let mut block = self.spare.pop().unwrap_or_default();
        block.clear();
        block.extend_from_slice(iq);
        let Some(w) = self.worker.as_mut() else { return Ok(()) };
        if w.jobs.send(Job::Block(block)).is_err() {
            self.worker = None;
            return Ok(());
        }
        w.ahead += iq.len();
        while self.worker.as_ref().is_some_and(|w| w.ahead > lag) {
            let Some(w) = self.worker.as_mut() else { break };
            let Ok(done) = w.done.recv() else {
                self.worker = None;
                break;
            };
            w.ahead -= done.samples;
            self.take(done, &mut outputs[0], c);
        }
        Ok(())
    }

    fn acquisition(&self) -> Option<pipeline::Acquisition> {
        Some(match (self.seen.lock, self.seen.frames) {
            (None, _) => pipeline::Acquisition::Searching,
            (Some(_), 0..=1) => pipeline::Acquisition::Acquiring,
            (Some(_), _) => pipeline::Acquisition::Locked,
        })
    }

    fn phases(&self) -> Vec<(String, pipeline::cost::Cost)> {
        vec![("reading, on its own thread".into(), self.reading.cost())]
    }

    fn readings(&self) -> Vec<(String, String)> {
        let Some(l) = self.seen.lock else { return Vec::new() };
        vec![
            ("mode".into(), l.mode.map_or_else(|| "not in the table".to_string(), |m| m.label())),
            ("frame".into(), format!("{:.3} Hz", l.frame_hz)),
            ("lines".into(), format!("{} at {:.3} kHz", l.periods.lines, l.line_hz / 1e3)),
            ("drift".into(), format!("{:+.2} ppm", self.seen.drift_ppm)),
            ("held".into(), {
                let (matched, judged) = self.seen.held;
                format!("{matched} of {judged} frames")
            }),
        ]
    }

    fn reset(&mut self) {
        self.send(Job::Reset);
    }

    fn params(&self) -> Vec<Param> {
        let mut modes = vec![AUTO.to_string()];
        modes.extend(display::labels());
        let at = self
            .forced
            .and_then(|m| display::modes().iter().position(|n| n == m))
            .map_or(0, |i| i + 1);
        vec![
            Param::choice(MODE, at, modes).label("Mode"),
            Param::int(DEPTH, self.depth, 1..=64).label("Average").unit("frames"),
            Param::int(NUDGE_X, self.nudge.0, -2048..=2048).label("Across"),
            Param::int(NUDGE_Y, self.nudge.1, -2250..=2250).label("Down"),
            Param::bool(ALIGN, self.align).label("Find blanking"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            MODE => self.forced = mode_setting(Some(&v)),
            DEPTH => self.depth = v.as_i64().unwrap_or(DEFAULT_DEPTH).clamp(1, 64),
            NUDGE_X => self.nudge.0 = v.as_i64().unwrap_or(0),
            NUDGE_Y => self.nudge.1 = v.as_i64().unwrap_or(0),
            ALIGN => self.align = v.as_bool().unwrap_or(true),
            other => return Err(common::Error::other(format!("tempest has no {other}"))),
        }
        self.send_settings();
        Ok(())
    }
}

impl Protocol for Tempest {
    fn arrives(&self) -> crate::protocol::Arrives {
        crate::protocol::Arrives::Continuously
    }

    fn id(&self) -> &'static str {
        Signal::id(self)
    }
    fn label(&self) -> &'static str {
        Signal::label(self)
    }
    fn aliases(&self) -> &'static [&'static str] {
        Signal::aliases(self)
    }
    fn placement(&self) -> Placement {
        Signal::placement(self)
    }
    fn shape(&self) -> Shape {
        Signal::shape(self)
    }
    fn default_hz(&self) -> f64 {
        Signal::default_hz(self)
    }

    fn stickiness(&self) -> Stickiness {
        Stickiness::Claim
    }
    fn outputs(&self) -> &'static [PortKind] {
        &[PortKind::Video]
    }
    fn chain(&self, _at: Placed) -> Vec<NodeSpec> {
        vec![NodeSpec::new(DESC.name)]
    }
}

const MODE: &str = "mode";
const DEPTH: &str = "average";
const NUDGE_X: &str = "across";
const NUDGE_Y: &str = "down";
const ALIGN: &str = "align";
const AUTO: &str = "auto";

pub const DESC: StageDesc = StageDesc {
    name: "tempest",
    summary: "A display's picture off the leakage from its cable",
    category: Category::Decode,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mut n = TempestNode::new(mode_setting(s.get(MODE)));
    n.depth = s.i64_or(DEPTH, DEFAULT_DEPTH).clamp(1, 64);
    n.align = s.bool_or(ALIGN, true);
    n.nudge = (s.i64_or(NUDGE_X, 0), s.i64_or(NUDGE_Y, 0));
    Ok(Box::new(n))
}

/// The mode a saved setting names, as a label or as a place in the list.
///
/// The interface sends a choice as its position, which is what a rebuild
/// then reads back: a mode picked by hand was read as a label, found
/// nothing, and the decoder went back to searching for the mode on every
/// retune.
fn mode_setting(v: Option<&ParamValue>) -> Option<&'static display::Mode> {
    match v? {
        ParamValue::Choice(0) => None,
        ParamValue::Choice(i) => display::modes().get(i - 1),
        other => other.as_str().filter(|t| *t != AUTO).and_then(display::by_label),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::{C32, Hz};
    use pipeline::node::Node;

    fn spec(rate: f64) -> PortSpec {
        PortSpec { spec: StreamSpec::iq(rate, Hz(DEFAULT_HZ as u64)), latency: 0 }
    }

    fn run(n: &mut TempestNode, iq: &[C32], rate: f64) -> (Vec<VideoFrame>, Vec<Request>) {
        let ins = [spec(rate)];
        let tags = Vec::new();
        let (mut frames, mut asked) = (Vec::new(), Vec::new());
        for chunk in iq.chunks(1 << 16) {
            let mut events = Vec::new();
            let mut new_tags = Vec::new();
            let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            let mut out = [Payload::empty_of(PortKind::Video)];
            let input = Payload::Iq(chunk.to_vec());
            Node::process(n, &[&input], &mut out, &mut ctx).expect("process");
            frames.extend(out[0].as_video().unwrap_or(&[]).iter().cloned());
            asked.extend(events.into_iter().filter_map(|e| match e {
                pipeline::event::Event::Request(r) => Some(r),
                _ => None,
            }));
        }
        (frames, asked)
    }

    fn sxga(rate: f64, seconds: f64) -> Vec<C32> {
        let m = display::by_label("1280x1024 60 Hz").expect("the mode");
        let mut state = 0x0DDB_A11C_0FFE_E123u64;
        let mut rand = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state >> 40) as f32 / 8_388_608.0 - 0.5
        };
        (0..(rate * seconds) as usize)
            .map(|i| {
                let pixel = i as f64 * m.pixel_clock_hz as f64 / rate;
                let x = pixel as usize % m.total_width;
                let y = (pixel / m.total_width as f64) as usize % m.total_height;
                let lit = match (x < m.width, y < m.height) {
                    (true, true) => {
                        let (u, v) = (x as f64 / m.width as f64, y as f64 / m.height as f64);
                        0.35 + 0.5 * ((u > 0.1 && u < 0.6 && v > 0.2 && v < 0.7) as u8 as f32)
                    }
                    _ => 0.0,
                };
                let phase = i as f64 * std::f64::consts::TAU * 1_234.0 / rate;
                let a = lit + 0.3 + rand() * 0.5;
                C32::new((a as f64 * phase.cos()) as f32, (a as f64 * phase.sin()) as f32)
            })
            .collect()
    }

    #[test]
    fn a_screen_on_the_span_reaches_the_video_port() {
        let rate = 20e6;
        let mut n = TempestNode::default();
        let out = Node::negotiate(&mut n, &[spec(rate)]).expect("a span");
        assert_eq!(out[0].kind, PortKind::Video);
        let (frames, asked) = run(&mut n, &sxga(rate, 0.5 + LAG_S), rate);
        assert_eq!(
            asked.iter().filter(|r| matches!(r, Request::Claim { .. })).count(),
            1,
            "claims of the span"
        );
        assert_eq!(frames.len(), 2, "pictures in half a second, read {LAG_S} s behind");
        let f = &frames[0];
        assert_eq!(f.system, "display leakage");
        assert_eq!(f.label.as_deref(), Some("1280x1024 60 Hz"));
        assert_eq!((f.width, f.height), (313, 1066));
        assert_eq!(f.samples.len(), 313 * 1066);
        assert_eq!(f.cadence, Cadence::Live);
        assert!(frames.windows(2).all(|w| w[1].sequence > w[0].sequence));
        let readings = n.readings();
        assert_eq!(readings[0], ("mode".to_string(), "1280x1024 60 Hz".to_string()));
        assert_eq!(n.acquisition(), Some(pipeline::Acquisition::Locked));
    }

    #[test]
    fn noise_on_the_span_claims_nothing() {
        let rate = 20e6;
        let mut n = TempestNode::default();
        Node::negotiate(&mut n, &[spec(rate)]).expect("a span");
        let mut state = 0x5555_AAAA_1234_9876u64;
        let iq: Vec<C32> = (0..(rate * 2.0) as usize)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let re = (state >> 40) as f32 / 8_388_608.0 - 0.5;
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                C32::new(re, (state >> 40) as f32 / 8_388_608.0 - 0.5)
            })
            .collect();
        let (frames, asked) = run(&mut n, &iq, rate);
        assert_eq!(
            (frames.len(), asked.len()),
            (0, 0),
            "pictures and requests off two seconds of noise"
        );
        assert_eq!(n.acquisition(), Some(pipeline::Acquisition::Searching));
    }

    #[test]
    fn a_span_too_narrow_for_a_picture_is_refused() {
        let mut n = TempestNode::default();
        assert!(Node::negotiate(&mut n, &[spec(2.4e6)]).is_err());
        assert!(Node::negotiate(&mut n, &[spec(MIN_RATE_HZ)]).is_ok());
    }

    /// A mode picked in the interface survives the rebuild that follows it.
    ///
    /// The interface sends a choice as its place in the list, and a rebuild
    /// builds the stage again from its settings. Reading that back as a
    /// label found nothing and left the decoder searching, so every retune
    /// undid the operator's choice.
    #[test]
    fn a_mode_picked_by_hand_survives_a_rebuild() {
        let at = display::modes()
            .iter()
            .position(|m| m.label() == "1920x1080 60 Hz")
            .expect("the mode is in the table");
        let mut settings = Settings::new();
        settings.insert(MODE.into(), ParamValue::Choice(at + 1));
        let built = build(&settings).expect("a screen decoder");
        let mode = built.params().into_iter().find(|p| p.name == MODE).expect("the mode");
        assert_eq!(mode.value, ParamValue::Choice(at + 1), "the choice came back as auto");

        // A file that named the mode in words is read the same way, and so
        // is one that says nothing.
        let mut named = Settings::new();
        named.insert(MODE.into(), ParamValue::Text("1024x768 60 Hz".into()));
        assert_eq!(mode_setting(named.get(MODE)).map(|m| m.total_height), Some(806));
        assert_eq!(mode_setting(None), None);
        assert_eq!(mode_setting(Some(&ParamValue::Choice(0))), None, "auto is the first choice");
    }

    #[test]
    fn the_mode_is_offered_as_the_table_and_set_from_it() {
        let mut n = TempestNode::default();
        let modes = n.params().into_iter().find(|p| p.name == MODE).expect("a mode choice");
        match modes.range {
            pipeline::param::ParamRange::Choices(c) => {
                assert_eq!(c.len(), display::modes().len() + 1, "auto and every mode");
                assert_eq!(c[0], "auto");
            }
            other => panic!("the mode is not a choice: {other:?}"),
        }
        n.set_param(MODE, ParamValue::Text("1024x768 60 Hz".into())).expect("a mode");
        assert_eq!(n.forced.map(|m| m.total_height), Some(806));
        n.set_param(MODE, ParamValue::Choice(0)).expect("auto");
        assert_eq!(n.forced, None);
    }
}
