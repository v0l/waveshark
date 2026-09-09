//! What was said, as it is being said, off the audio bus.
//!
//! The other transcriber read whole overs off the packet bus, which meant
//! nothing appeared until somebody stopped talking and nothing appeared at
//! all for a channel that never produced a packet. This one is wired to the
//! bus tap: every strip's audio and every decoded voice arrive here block by
//! block, before the faders and before the subscriptions, so the receiver
//! writes down what it heard rather than what the operator chose to listen
//! to.
//!
//! # A conversation is a key
//!
//! Speech has no address, so one is made from what the receiver knows:
//! `{proto}:{freq}:{chan}:{speaker}`, with the parts it does not know left
//! empty. An FM channel on 145.5 MHz is `Audio:145500000::`; a DMR call to
//! talkgroup 9 from radio 1234567 is `DMR:435000000:9:1234567`. That is
//! deliberately a string and not a struct: a view looks a conversation up by
//! it, two blocks of the same call agree on it without coordinating, and a
//! system nobody has written yet fills in the parts it has.
//!
//! # Streaming
//!
//! Whisper reads a window, not a stream, so "streaming" here is a window
//! re-read as it grows: while somebody is talking, the audio so far is sent
//! to the model every [`PARTIAL_EVERY_S`] seconds and what comes back
//! replaces the running text for that key. When the speech stops the whole
//! utterance goes once more and that result is kept. The partials are what
//! makes it feel live; the final pass is the one worth reading, because a
//! model given the whole sentence punctuates and corrects what it guessed
//! from half of it.
//!
//! Nothing is held for longer than the model's own window ([`stt::WINDOW_S`],
//! thirty seconds). A repeater left keyed used to grow one buffer until it
//! hit a two-minute cap, and every partial re-read all of it: ninety seconds
//! held meant three windows decoded every two seconds, for one line that had
//! not been written yet. So a run of speech that reaches the window is cut
//! and the part before the cut is settled: the cut is placed at the quietest
//! moment in the last few seconds, which is a pause if there is one, and what
//! comes after it starts the next line. The words carry on; what stops
//! growing is the buffer.
//!
//! An open channel that is not speech is stopped by the model rather than by
//! a threshold. Squelch noise is loud enough to collect, so the level test
//! alone read a hissing repeater as somebody talking for as long as it hissed;
//! after two windows come back with nothing credible in them, that
//! conversation is left alone until it goes quiet again.

use common::Result;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// How often a partial is asked for while somebody is still talking.
pub const PARTIAL_EVERY_S: f64 = 2.0;

/// Silence that ends an utterance. Long enough for the pause in the middle of
/// a sentence, short enough that a reply is a new line.
const HANG_S: f64 = 0.8;

/// Below this the block is silence. A squelched channel delivers zeros and an
/// open one delivers noise, and the noise is what has to be rejected.
const FLOOR: f32 = 0.004;

/// Longest run of speech held before it is cut into a settled line and a
/// fresh one. The model's own window: holding more is holding audio that
/// cannot be read in one pass, and paying for the whole of it again on every
/// partial.
#[cfg(feature = "stt")]
const HOLD_S: f64 = stt::WINDOW_S;
#[cfg(not(feature = "stt"))]
const HOLD_S: f64 = 30.0;

/// How far back from the cut a pause is looked for. Long enough to find the
/// gap between two sentences, short enough that the line before the cut is
/// most of the window.
const CUT_SEARCH_S: f64 = 3.0;

/// Windows a conversation may come back empty before it is left alone. Two,
/// because one window of a caller thinking is not evidence of anything.
const DUDS_BEFORE_DEAF: u8 = 2;

/// Hard cap on what is held when the model is behind. Beyond this the audio
/// is dropped rather than queued: a transcript of what was said ten minutes
/// ago is worth less than keeping up with what is being said now.
const MAX_HELD_S: f64 = 90.0;

/// Conversations kept before the oldest is forgotten.
const MAX_KEYS: usize = 512;

/// Utterances kept per conversation.
const MAX_PER_KEY: usize = 64;

/// One thing somebody said, or as much of it as has been heard.
#[derive(Clone, Debug, PartialEq)]
pub struct Utterance {
    pub key: String,
    /// When it started, on the receiver's clock.
    pub at: Instant,
    pub seconds: f64,
    pub text: String,
    /// Whether the speech had finished when this was read. A partial is
    /// replaced by the next one; a settled utterance never changes again.
    pub settled: bool,
    /// The model's own mean log probability, near zero for a confident read
    /// and below about -1 for a guess.
    pub confidence: f32,
    /// Whether the model believed this was speech it read correctly, by its
    /// own two thresholds. False is a reading worth showing and worth
    /// doubting, which is why the words are kept and this is carried beside
    /// them.
    pub credible: bool,
}

/// Who is talking, in the parts the receiver knows.
///
/// Kept as its own type so a caller builds a key rather than formatting one,
/// which is how the two ends stay in step.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Speaker {
    pub proto: String,
    pub freq_hz: u64,
    /// The talkgroup, reflector or destination, where the system has one.
    pub channel: Option<String>,
    /// Who is talking, where the system says.
    pub speaker: Option<String>,
}

impl Speaker {
    pub fn key(&self) -> String {
        format!(
            "{}:{}:{}:{}",
            self.proto,
            self.freq_hz,
            self.channel.as_deref().unwrap_or(""),
            self.speaker.as_deref().unwrap_or(""),
        )
    }

    /// The other direction, for a view holding a key and wanting the parts.
    pub fn parse(key: &str) -> Option<Self> {
        let mut it = key.splitn(4, ':');
        let proto = it.next()?.to_string();
        let freq_hz = it.next()?.parse().ok()?;
        let some = |s: &str| (!s.is_empty()).then(|| s.to_string());
        Some(Self {
            proto,
            freq_hz,
            channel: some(it.next().unwrap_or_default()),
            speaker: some(it.next().unwrap_or_default()),
        })
    }
}

/// Everything that has been said, by conversation.
///
/// One for the whole program, behind [`log`]. The node writes to it and the
/// interface reads it, and neither owns it: the node is a stage in a graph
/// that is rebuilt on every retune, and a log that lived inside it was
/// emptied every time the dial moved, which on screen was three reads and
/// no lines. A transcript outlives any one graph the way the call list
/// does.
///
/// In memory and bounded. Nothing here is written to disk yet: the packet log
/// holds evidence and a transcript is not evidence, so where transcripts
/// belong on disk is a decision that has not been made.
#[derive(Debug, Default)]
pub struct TranscriptLog {
    by_key: HashMap<String, Vec<Utterance>>,
    /// Keys in the order they were last spoken on, oldest first.
    order: Vec<String>,
    /// Bumped on every push, so a reader can tell whether anything changed
    /// without comparing the contents.
    seq: u64,
}

/// The one transcript.
pub type SharedLog = std::sync::Arc<parking_lot::Mutex<TranscriptLog>>;

/// The program's transcript, which every transcriber writes to and the
/// transcript view reads.
pub fn log() -> &'static SharedLog {
    static LOG: std::sync::OnceLock<SharedLog> = std::sync::OnceLock::new();
    LOG.get_or_init(Default::default)
}

impl TranscriptLog {
    /// Add or replace. One utterance is one start time on one key, so a
    /// reading of speech already held replaces what is there: that is what
    /// makes a growing window read as one line getting longer rather than as
    /// a page of half sentences, and it is what lets a view fold the same
    /// published window in on every frame without collecting duplicates.
    pub fn push(&mut self, u: Utterance) {
        let list = self.by_key.entry(u.key.clone()).or_default();
        match list.last_mut() {
            Some(last) if last.at == u.at => *last = u.clone(),
            _ => list.push(u.clone()),
        }
        if list.len() > MAX_PER_KEY {
            list.remove(0);
        }
        self.order.retain(|k| k != &u.key);
        self.order.push(u.key);
        while self.order.len() > MAX_KEYS {
            let gone = self.order.remove(0);
            self.by_key.remove(&gone);
        }
        self.seq += 1;
    }

    /// How many pushes there have been, for a reader deciding whether to
    /// look again.
    pub fn seq(&self) -> u64 {
        self.seq
    }

    /// A copy, for a view to draw from without holding the lock while it
    /// draws.
    pub fn snapshot(&self) -> Self {
        Self { by_key: self.by_key.clone(), order: self.order.clone(), seq: self.seq }
    }

    /// Everything said on one conversation, oldest first.
    pub fn of(&self, key: &str) -> &[Utterance] {
        self.by_key.get(key).map(|v| v.as_slice()).unwrap_or(&[])
    }

    /// The last thing said on one conversation, whether or not it has
    /// finished.
    pub fn latest(&self, key: &str) -> Option<&Utterance> {
        self.by_key.get(key).and_then(|v| v.last())
    }

    /// Conversations heard, most recent last.
    pub fn keys(&self) -> &[String] {
        &self.order
    }

    /// The last `n` utterances across every conversation, newest last.
    pub fn recent(&self, n: usize) -> Vec<&Utterance> {
        let mut all: Vec<&Utterance> = self.by_key.values().flatten().collect();
        all.sort_by_key(|u| u.at);
        if all.len() > n {
            all.drain(..all.len() - n);
        }
        all
    }

    /// Whether anything was read on one conversation, which is what decides
    /// whether a call is worth offering a way into this log.
    pub fn has(&self, key: &str) -> bool {
        self.by_key.get(key).is_some_and(|v| !v.is_empty())
    }

    /// Lines held, across every conversation.
    pub fn len(&self) -> usize {
        self.by_key.values().map(|v| v.len()).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.by_key.is_empty()
    }

    pub fn clear(&mut self) {
        self.by_key.clear();
        self.order.clear();
        self.seq += 1;
    }
}

/// Where the model is in its life, which the node cannot see for itself: the
/// fetching, the loading and the reading all happen on the worker thread.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum ModelState {
    /// Nothing has asked for it yet. The model is loaded by the first speech
    /// worth reading, so a receiver that hears none never fetches one.
    #[default]
    Cold,
    /// Being downloaded from the hub, which is the one thing here that needs
    /// a network.
    Fetching,
    Loading,
    Ready,
    /// It cannot be used, and why.
    Failed(String),
}

impl ModelState {
    /// What a pane prints for it. The reason is shown beside this rather than
    /// inside it, because it is a sentence and this is a word.
    pub fn label(&self) -> &'static str {
        match self {
            Self::Cold => "not loaded",
            Self::Fetching => "downloading",
            Self::Loading => "loading",
            Self::Ready => "ready",
            Self::Failed(_) => "failed",
        }
    }
}

/// How far a model download has got. The app's own copy of what `stt`
/// reports, so the interface does not depend on the feature being built.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fetch {
    pub file: String,
    pub done: u64,
    pub total: u64,
    pub files_done: usize,
    pub files: usize,
}

impl Fetch {
    /// The fraction of the file in hand, or `None` when the hub has not said
    /// how big it is.
    pub fn fraction(&self) -> Option<f32> {
        (self.total > 0).then(|| (self.done as f64 / self.total as f64) as f32)
    }
}

/// One model a pane can offer.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ModelChoice {
    pub id: String,
    pub label: String,
    /// Roughly what it takes to fetch, in bytes, or 0 for a model that is
    /// not in the catalogue.
    pub bytes: u64,
    /// Whether its files are already on disc.
    pub present: bool,
}

/// What the transcriber is and what it is doing, for the view that shows it.
///
/// Which model, where its files are, whether they are there at all, what it
/// is running on and how much it has read. Without this the pane can say
/// only that no text has appeared, which is the same picture for a model
/// that was never downloaded, one that failed to load, one running on a CPU
/// too slow to keep up, and a band where nobody is talking.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Engine {
    /// The node in the running graph, so a view can set its parameters.
    pub node: usize,
    pub enabled: bool,
    /// Which model, by catalogue id, and the repository it is fetched from.
    pub model: String,
    pub repo: String,
    /// Where its files are kept.
    pub dir: String,
    /// What the chosen model is called.
    pub label: String,
    /// Every model that can be picked: the catalogue, plus anything on disc
    /// under the models directory that is not in it.
    pub models: Vec<ModelChoice>,
    /// Where it was asked to run, and every device it could be asked to run
    /// on, as ids and labels.
    pub device_choice: String,
    pub devices: Vec<(String, String)>,
    /// Whether a usable model is in that directory already, and what it
    /// takes up.
    pub present: bool,
    pub bytes: u64,
    /// What the files in that directory actually are: the weights file and
    /// whether they carry language tokens. The repository is only where they
    /// would be fetched from, so on a receiver whose directory was filled by
    /// hand, or by an earlier run asking for another model, the two disagree
    /// and this is the half that is running.
    pub weights: String,
    pub flavour: String,
    pub state: ModelState,
    /// How far the download has got, while one is running: the file, the
    /// bytes of it, and how many files are done. Without it the card says
    /// "downloading" for as long as a multi-gigabyte model takes, which
    /// looks exactly like a fetch that has hung.
    pub fetch: Fetch,
    /// What it is running on, once it has loaded: CPU, CUDA or Metal.
    pub device: String,
    /// Why that is not what Auto reached for first, when it is not: the
    /// card had no room, or opened and could not run.
    pub note: String,
    /// Windows read since it loaded, and what the last one cost against how
    /// much audio it was. A receiver whose model is slower than real time is
    /// a receiver that will fall behind, and this is where that shows.
    pub reads: u64,
    pub last_ms: u64,
    pub last_audio_s: f64,
    /// Speech being collected right now, and on how many conversations.
    pub holding_s: f64,
    pub speakers: usize,
    /// Whether the model has a window in front of it at this moment.
    pub busy: bool,
}

impl Engine {
    /// How much faster than real time the last read was. Below 1 the model
    /// cannot keep up with somebody talking continuously.
    pub fn speed(&self) -> Option<f64> {
        (self.last_ms > 0 && self.last_audio_s > 0.0)
            .then(|| self.last_audio_s / (self.last_ms as f64 / 1000.0))
    }
}

/// What is being collected for one key.
struct Talking {
    pcm: Vec<f32>,
    rate: f64,
    started: Instant,
    quiet_s: f64,
    /// Seconds of audio at the last time a partial was asked for.
    asked_at_s: f64,
    /// A job is out and its answer has not come back.
    waiting: bool,
    /// The speech has stopped and the last reading is owed.
    finished: bool,
    /// Windows in a row the model has found nothing credible in.
    duds: u8,
    /// The model has said, twice, that this is not speech. Nothing more is
    /// collected until the channel goes quiet, which is what ends it.
    deaf: bool,
}

impl Talking {
    fn seconds(&self) -> f64 {
        self.pcm.len() as f64 / self.rate
    }
}

/// Where to cut a run of speech that has filled the model's window.
///
/// The quietest twentieth of a second in the last [`CUT_SEARCH_S`], which is
/// the pause between two sentences where there is one. Cutting at the end of
/// the buffer instead splits whatever word was being said across two lines,
/// and the model reads each half as a different word.
fn cut_point(pcm: &[f32], rate: f64) -> usize {
    let step = ((rate * 0.05) as usize).max(1);
    let back = ((rate * CUT_SEARCH_S) as usize).min(pcm.len());
    let from = pcm.len() - back;
    let mut best = pcm.len();
    let mut quietest = f32::INFINITY;
    let mut at = from;
    while at + step <= pcm.len() {
        let energy: f32 = pcm[at..at + step].iter().map(|s| s.abs()).sum();
        if energy < quietest {
            quietest = energy;
            best = at + step / 2;
        }
        at += step;
    }
    best
}

/// The streaming transcriber, as a node on the audio bus tap.
pub struct LiveTranscribeNode {
    /// Where the lines go: the program's one transcript, unless a test
    /// handed this node one of its own.
    log: SharedLog,
    talking: HashMap<String, Talking>,
    enabled: bool,
    /// Shortest run of speech worth reading. A squelch tail transcribes as
    /// "Thank you." with high confidence.
    min_speech_s: f64,
    #[cfg(feature = "stt")]
    worker: Option<Worker>,
    /// The models directory; each model has a directory of its own in it.
    #[cfg(feature = "stt")]
    root: std::path::PathBuf,
    /// A directory named outright, which wins over root and model. What a
    /// test hands the node so it reads whatever is there.
    #[cfg(feature = "stt")]
    explicit_dir: Option<std::path::PathBuf>,
    #[cfg(feature = "stt")]
    model_id: String,
    #[cfg(feature = "stt")]
    device: stt::DeviceChoice,
    /// What is on disc under `root`, read when something changes rather
    /// than every time a pane asks.
    #[cfg(feature = "stt")]
    installed: Vec<String>,
    #[cfg(feature = "stt")]
    reported: bool,
    /// What the model is doing, written by the thread that has it.
    #[cfg(feature = "stt")]
    health: std::sync::Arc<parking_lot::Mutex<Health>>,
}

/// The worker thread's half of [`Engine`].
#[cfg(feature = "stt")]
#[derive(Debug, Default)]
struct Health {
    state: ModelState,
    device: String,
    /// Why it is not where it was asked to be, when it is not.
    note: String,
    reads: u64,
    last_ms: u64,
    last_audio_s: f64,
    fetch: Fetch,
    present: bool,
    bytes: u64,
    weights: String,
    flavour: String,
}

#[cfg(feature = "stt")]
impl Health {
    /// Record what is on disc, or that nothing is.
    fn describe(&mut self, files: Option<&stt::Files>) {
        self.present = files.is_some();
        self.bytes = files.map(|f| f.bytes()).unwrap_or(0);
        self.weights = files
            .and_then(|f| f.weights.file_name().map(|n| n.to_string_lossy().into_owned()))
            .unwrap_or_default();
        self.flavour = match files.map(|f| (f.family, f.flavour)) {
            Some((stt::Family::Qwen3Asr, _)) => "Qwen3-ASR, multilingual".into(),
            Some((stt::Family::Whisper, stt::Flavour::English)) => "Whisper, English only".into(),
            Some((stt::Family::Whisper, stt::Flavour::Multilingual)) => {
                "Whisper, multilingual".into()
            }
            None => String::new(),
        };
    }
}

impl Default for LiveTranscribeNode {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveTranscribeNode {
    pub fn new() -> Self {
        Self {
            log: log().clone(),
            talking: HashMap::new(),
            enabled: true,
            min_speech_s: 0.6,
            #[cfg(feature = "stt")]
            worker: None,
            #[cfg(feature = "stt")]
            root: std::path::PathBuf::new(),
            #[cfg(feature = "stt")]
            explicit_dir: None,
            #[cfg(feature = "stt")]
            model_id: stt::DEFAULT_MODEL.to_string(),
            #[cfg(feature = "stt")]
            device: stt::DeviceChoice::Auto,
            #[cfg(feature = "stt")]
            installed: Vec::new(),
            #[cfg(feature = "stt")]
            reported: false,
            #[cfg(feature = "stt")]
            health: std::sync::Arc::new(parking_lot::Mutex::new(Health::default())),
        }
    }

    /// Read whatever model is in one directory, whatever it is called.
    #[cfg(feature = "stt")]
    pub fn in_dir(mut self, dir: impl Into<std::path::PathBuf>) -> Self {
        self.explicit_dir = Some(dir.into());
        self.look();
        self
    }

    /// Keep models under `root`, one directory each.
    #[cfg(feature = "stt")]
    pub fn under(mut self, root: impl Into<std::path::PathBuf>) -> Self {
        self.root = root.into();
        self.look();
        self
    }

    /// Where the chosen model's files are, or would be fetched to.
    #[cfg(feature = "stt")]
    fn dir(&self) -> std::path::PathBuf {
        match &self.explicit_dir {
            Some(d) => d.clone(),
            None => stt::model_dir(&self.root, &self.model_id),
        }
    }

    /// The catalogue and what is on disc beside it, for the pick list.
    #[cfg(feature = "stt")]
    fn models(&self) -> Vec<ModelChoice> {
        let here = &self.installed;
        let mut out: Vec<ModelChoice> = stt::MODELS
            .iter()
            .map(|m| ModelChoice {
                id: m.id.to_string(),
                label: m.label.to_string(),
                bytes: m.mb as u64 * 1_000_000,
                present: here.iter().any(|h| h == m.id),
            })
            .collect();
        for h in here {
            if !out.iter().any(|m| m.id == *h) {
                out.push(ModelChoice { id: h.clone(), label: h.clone(), bytes: 0, present: true });
            }
        }
        out
    }

    /// Whether a model is on disc where this node would look, and how large
    /// it is. Three stats, taken when the directory changes rather than per
    /// block.
    #[cfg(feature = "stt")]
    fn look(&mut self) {
        self.installed = stt::installed(&self.root);
        let mut h = self.health.lock();
        h.describe(stt::Files::in_dir(self.dir()).ok().as_ref());
    }

    /// Forget the loaded model, so the next thing worth reading loads the
    /// one now chosen on the device now chosen.
    #[cfg(feature = "stt")]
    fn reload(&mut self) {
        self.worker = None;
        self.reported = false;
        *self.health.lock() = Health::default();
        self.look();
    }

    /// What this node is and what it is doing, for the transcript view.
    pub fn engine(&self) -> Engine {
        #[allow(unused_mut)]
        let mut e = Engine {
            enabled: self.enabled,
            speakers: self.talking.len(),
            holding_s: self.talking.values().map(|t| t.pcm.len() as f64 / t.rate).sum(),
            busy: self.talking.values().any(|t| t.waiting),
            ..Default::default()
        };
        #[cfg(feature = "stt")]
        {
            e.model = self.model_id.clone();
            e.label = stt::label_of(&self.model_id);
            e.repo = stt::repo_of(&self.model_id);
            e.dir = self.dir().display().to_string();
            e.models = self.models();
            e.device_choice = self.device.id();
            e.devices = stt::devices().into_iter().map(|d| (d.choice.id(), d.label)).collect();
            let h = self.health.lock();
            e.state = h.state.clone();
            e.device = h.device.clone();
            e.note = h.note.clone();
            e.reads = h.reads;
            e.last_ms = h.last_ms;
            e.last_audio_s = h.last_audio_s;
            e.fetch = h.fetch.clone();
            e.present = h.present;
            e.bytes = h.bytes;
            e.weights = h.weights.clone();
            e.flavour = h.flavour.clone();
        }
        e
    }

    /// Which model, by catalogue id or by any Whisper repository name.
    #[cfg(feature = "stt")]
    pub fn model(mut self, id: &str) -> Self {
        self.model_id = id.to_string();
        self.look();
        self
    }

    #[cfg(feature = "stt")]
    pub fn on(mut self, device: stt::DeviceChoice) -> Self {
        self.device = device;
        self
    }

    /// Write to a transcript of the caller's own rather than the program's,
    /// so a test reads what it produced and nothing else.
    pub fn into_log(mut self, log: SharedLog) -> Self {
        self.log = log;
        self
    }

    pub fn log(&self) -> &SharedLog {
        &self.log
    }

    /// Fold one block of somebody's audio in, and say whether they have
    /// stopped talking.
    ///
    /// Split out from `process` because the decision of what is speech and
    /// when it ended is worth testing without a model behind it.
    fn collect(&mut self, key: String, v: &common::Voice, block_s: f64, at: Instant) -> bool {
        let loud = v.pcm.iter().any(|s| s.abs() > FLOOR);
        let entry = self.talking.entry(key).or_insert_with(|| Talking {
            pcm: Vec::new(),
            rate: v.rate.max(1.0),
            started: at,
            quiet_s: 0.0,
            asked_at_s: 0.0,
            waiting: false,
            finished: false,
            duds: 0,
            deaf: false,
        });
        // A conversation the model has twice said is not speech collects
        // nothing until it goes quiet, which is the channel closing.
        if entry.deaf {
            if !loud {
                entry.pcm.clear();
                entry.deaf = false;
                entry.duds = 0;
            }
            return false;
        }
        if loud {
            entry.quiet_s = 0.0;
            entry.pcm.extend_from_slice(&v.pcm);
        } else if entry.pcm.is_empty() {
            // Nothing collected yet, so this is a channel sitting open rather
            // than a pause in the middle of a sentence.
            return false;
        } else {
            entry.quiet_s += block_s;
            // The pause is kept: cutting it out of the audio joins two words
            // that were not said together and the model reads them as one.
            entry.pcm.extend_from_slice(&v.pcm);
        }
        entry.quiet_s >= HANG_S || entry.seconds() >= MAX_HELD_S
    }

    /// What the model made of a settled window, so a conversation it found
    /// nothing in twice is left alone. Squelch noise is loud enough to
    /// collect and there is no threshold that tells it from speech; the model
    /// already decides, and this is that decision being used.
    fn read_back(&mut self, key: &str, anything: bool) {
        let Some(t) = self.talking.get_mut(key) else {
            return;
        };
        if anything {
            t.duds = 0;
            return;
        }
        t.duds = t.duds.saturating_add(1);
        if t.duds >= DUDS_BEFORE_DEAF {
            t.deaf = true;
            t.pcm.clear();
        }
    }

    /// Seconds of audio held for a key, for tests and for a status line.
    pub fn held_seconds(&self, key: &str) -> f64 {
        self.talking.get(key).map(|t| t.pcm.len() as f64 / t.rate).unwrap_or(0.0)
    }
}

/// The key one block of speech belongs to.
pub fn key_of(v: &common::Voice) -> String {
    Speaker {
        proto: v.system.to_string(),
        freq_hz: v.channel_hz.max(0.0) as u64,
        channel: v.to.clone(),
        speaker: v.from.clone(),
    }
    .key()
}

#[cfg(feature = "stt")]
mod work {
    use super::*;
    use crossbeam_channel::{bounded, Receiver, Sender, TryRecvError};

    pub(super) struct Job {
        pub key: String,
        pub at: Instant,
        pub pcm: Vec<f32>,
        pub rate: f64,
        pub settled: bool,
    }

    /// The worker's word that a job is finished, so the node can send the
    /// next one. The text itself does not come this way: the worker writes
    /// it into the transcript directly, so a read that finishes after the
    /// node that asked for it has gone is still written down.
    pub(super) struct Done {
        pub key: String,
        pub at: Instant,
        pub settled: bool,
        /// Whether the model heard speech, or what went wrong.
        pub result: Result<bool>,
    }

    pub(super) struct Worker {
        pub jobs: Sender<Job>,
        pub done: Receiver<Done>,
    }

    impl LiveTranscribeNode {
        /// The model thread, started by the first thing worth reading.
        pub(super) fn worker(&mut self) -> Option<&Worker> {
            if self.worker.is_none() {
                let (jobs_tx, jobs_rx) = bounded::<Job>(32);
                let (done_tx, done_rx) = bounded::<Done>(32);
                let dir = self.dir();
                let repo = stt::repo_of(&self.model_id);
                let device = self.device;
                let health = self.health.clone();
                let log = self.log.clone();
                std::thread::Builder::new()
                    .name("whisper-live".into())
                    .spawn(move || run(dir, repo, device, health, log, jobs_rx, done_tx))
                    .ok()?;
                self.worker = Some(Worker { jobs: jobs_tx, done: done_rx });
            }
            self.worker.as_ref()
        }

        /// Take whatever the model has finished and put it in the log.
        pub(super) fn drain(&mut self, c: &mut NodeCtx<'_>) {
            loop {
                let done = match self.worker.as_ref().map(|w| w.done.try_recv()) {
                    Some(Ok(d)) => d,
                    Some(Err(TryRecvError::Empty)) | None => break,
                    Some(Err(TryRecvError::Disconnected)) => {
                        self.worker = None;
                        break;
                    }
                };
                if let Some(t) = self.talking.get_mut(&done.key) {
                    t.waiting = false;
                }
                if done.settled {
                    if let Ok(speech) = done.result {
                        // Whether it was speech, not whether it was read
                        // well: the model returns a plausible sentence for a
                        // fan or an open squelch, so words alone are not
                        // evidence of anybody talking, and a weak handheld
                        // read badly is still somebody talking.
                        self.read_back(&done.key, speech);
                    }
                    // The last reading of an utterance is the end of it, and
                    // holding it into the next one would read the same words
                    // again with somebody else's in front. Unless what is
                    // held is already the next one: a run of speech that
                    // filled the window was cut, and what came after the cut
                    // is a line of its own that is still being spoken.
                    let carried = self.talking.get(&done.key).is_some_and(|t| t.started != done.at);
                    if carried {
                        self.pump(&done.key);
                    } else {
                        self.talking.remove(&done.key);
                    }
                } else {
                    self.pump(&done.key);
                }
                match done.result {
                    Ok(_) => {}
                    Err(e) => {
                        if !self.reported {
                            self.reported = true;
                            c.emit(pipeline::event::Event::Warning {
                                stage: "transcribe_live".into(),
                                message: format!("{e}"),
                            });
                        }
                    }
                }
            }
        }

        /// Ask for whatever this key is owed: the last reading if the
        /// speech has stopped, another partial if it is still going and
        /// enough has arrived since the last one, and nothing while the model
        /// still has the previous window.
        pub(super) fn pump(&mut self, key: &str) {
            let Some(t) = self.talking.get(key) else {
                return;
            };
            if t.waiting {
                return;
            }
            let seconds = t.pcm.len() as f64 / t.rate;
            let short = seconds < self.min_speech_s;
            if t.finished {
                // Nothing worth reading, so the buffer goes rather than
                // waiting for the model to say so.
                if short {
                    self.talking.remove(key);
                } else {
                    self.ask(key, true);
                }
            } else if seconds >= HOLD_S {
                // Somebody is still talking and the buffer has reached what
                // the model reads in one pass. Settle what is there and keep
                // only what came after the pause it was cut at, so the next
                // read is one window and not two.
                self.cut(key);
            } else if !short && seconds - t.asked_at_s >= PARTIAL_EVERY_S {
                self.ask(key, false);
            }
        }

        /// Settle the speech held so far and carry the rest into a new line.
        pub(super) fn cut(&mut self, key: &str) {
            let Some(t) = self.talking.get_mut(key) else {
                return;
            };
            if t.waiting {
                return;
            }
            let at = cut_point(&t.pcm, t.rate);
            let head: Vec<f32> = t.pcm[..at].to_vec();
            let tail: Vec<f32> = t.pcm[at..].to_vec();
            let job =
                Job { key: key.to_string(), at: t.started, pcm: head, rate: t.rate, settled: true };
            if self.worker().is_some_and(|w| w.jobs.try_send(job).is_ok()) {
                if let Some(t) = self.talking.get_mut(key) {
                    // The tail is a new utterance, with its own start time,
                    // and it waits for the head to come back rather than
                    // queueing a second window behind it.
                    t.pcm = tail;
                    t.started = Instant::now();
                    t.asked_at_s = 0.0;
                    t.waiting = true;
                }
            }
        }

        /// Send what is held for a key to the model.
        pub(super) fn ask(&mut self, key: &str, settled: bool) {
            let Some(t) = self.talking.get(key) else {
                return;
            };
            if t.waiting {
                return;
            }
            let job = Job {
                key: key.to_string(),
                at: t.started,
                pcm: t.pcm.clone(),
                rate: t.rate,
                settled,
            };
            let asked = t.seconds();
            if self.worker().is_some_and(|w| w.jobs.try_send(job).is_ok()) {
                if let Some(t) = self.talking.get_mut(key) {
                    t.waiting = true;
                    t.asked_at_s = asked;
                }
            }
        }
    }

    fn run(
        dir: std::path::PathBuf,
        repo: String,
        choice: stt::DeviceChoice,
        health: std::sync::Arc<parking_lot::Mutex<Health>>,
        log: SharedLog,
        jobs: Receiver<Job>,
        done: Sender<Done>,
    ) {
        let have = stt::Files::in_dir(&dir).is_ok();
        health.lock().state = if have { ModelState::Loading } else { ModelState::Fetching };
        let mut label = String::new();
        let mut note = String::new();
        let files = stt::ensure_with(&repo, &dir, &mut |p| {
            let mut h = health.lock();
            h.fetch = Fetch {
                file: p.file.clone(),
                done: p.done,
                total: p.total,
                files_done: p.files_done,
                files: p.files,
            };
        })
        .map(|f| {
            let mut h = health.lock();
            h.describe(Some(&f));
            h.state = ModelState::Loading;
            f
        });
        let loaded = files.and_then(|f| {
            stt::Engine::load_on(&f, choice, None).map(|(m, on, why)| {
                label = on;
                note = why;
                m
            })
        });
        let mut model = match loaded {
            Ok(m) => {
                let mut h = health.lock();
                h.state = ModelState::Ready;
                h.device = label;
                h.note = note;
                drop(h);
                m
            }
            Err(e) => {
                let msg = format!("{}: {e}", stt::label_of(&repo));
                health.lock().state = ModelState::Failed(msg.clone());
                while let Ok(job) = jobs.recv() {
                    let _ = done.send(Done {
                        key: job.key,
                        at: job.at,
                        settled: job.settled,
                        result: Err(common::Error::other(&msg)),
                    });
                }
                return;
            }
        };
        // What the model is handed, as files, when asked. The one way to
        // tell a model that reads nothing from audio that has nothing in it.
        let dump = std::env::var_os("WAVESHARK_DUMP_STT").map(std::path::PathBuf::from);
        let mut dumped = 0u32;
        while let Ok(job) = jobs.recv() {
            let seconds = job.pcm.len() as f64 / job.rate.max(1.0);
            if let Some(dir) = &dump {
                let name = format!("{dumped:04}_{}_{seconds:.1}s.wav", job.key.replace(':', "_"));
                let speech = common::Speech { pcm: job.pcm.clone(), rate: job.rate };
                let _ = crate::audiobus::write_wav(&dir.join(name), &speech);
                dumped += 1;
            }
            let started = Instant::now();
            let result = model.transcribe(&job.pcm, job.rate);
            {
                let mut h = health.lock();
                h.reads += 1;
                h.last_ms = started.elapsed().as_millis() as u64;
                h.last_audio_s = seconds;
            }
            // Written down here, on the thread that read it, and not handed
            // back to the node: the node is a stage in a graph that is
            // rebuilt whenever a channel comes or goes, and a reading that
            // came back to a node that had been rebuilt was thrown away
            // with the channel it was sent on.
            let verdict = match &result {
                Ok(t) => {
                    let text = t.text.trim().to_string();
                    if !text.is_empty() {
                        log.lock().push(Utterance {
                            key: job.key.clone(),
                            at: job.at,
                            seconds,
                            text,
                            settled: job.settled,
                            confidence: t.avg_logprob() as f32,
                            credible: t.credible(),
                        });
                    }
                    Ok(t.speech())
                }
                Err(e) => Err(common::Error::other(format!("{e}"))),
            };
            if done
                .send(Done {
                    key: job.key.clone(),
                    at: job.at,
                    settled: job.settled,
                    result: verdict,
                })
                .is_err()
            {
                // Nobody to tell: the node is gone. Whatever this window
                // read is then the last word on that utterance, since
                // nothing will ask for the rest of it.
                if !job.settled {
                    let mut log = log.lock();
                    if let Some(u) = log.by_key.get_mut(&job.key).and_then(|v| v.last_mut()) {
                        if u.at == job.at {
                            u.settled = true;
                            log.seq += 1;
                        }
                    }
                }
                break;
            }
        }
    }
}

#[cfg(feature = "stt")]
use work::Worker;

impl Simple for LiveTranscribeNode {
    fn name(&self) -> &str {
        "transcribe_live"
    }

    fn is_sink(&self) -> bool {
        true
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Voice {
            return Err(common::Error::other("the live transcriber reads the audio bus tap"));
        }
        Ok(i.spec.clone())
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        #[cfg(feature = "stt")]
        self.drain(_c);
        if !self.enabled {
            self.talking.clear();
            return Ok(());
        }
        let at = Instant::now();
        let block_s = _c.block_seconds;
        let mut seen: Vec<String> = Vec::new();
        for v in i.as_voice().unwrap_or(&[]) {
            let key = key_of(v);
            if self.collect(key.clone(), v, block_s, at) {
                if let Some(t) = self.talking.get_mut(&key) {
                    t.finished = true;
                }
            }
            if !seen.contains(&key) {
                seen.push(key);
            }
        }
        #[cfg(feature = "stt")]
        for key in seen {
            self.pump(&key);
        }
        Ok(())
    }

    fn reset(&mut self) {
        self.talking.clear();
    }

    fn params(&self) -> Vec<Param> {
        #[allow(unused_mut)]
        let mut out = vec![
            Param::bool("enabled", self.enabled).label("Transcribe what is heard"),
            Param::float("min_speech_s", self.min_speech_s, 0.1..=5.0)
                .unit("s")
                .label("Shortest speech worth reading"),
        ];
        // Offered as choices so the chain inspector draws a list, and set
        // by id so what the patch records is a name and not a position in
        // a list that grows.
        #[cfg(feature = "stt")]
        {
            let models = self.models();
            let at = models.iter().position(|m| m.id == self.model_id).unwrap_or(0);
            out.push(
                Param::choice("model", at, models.into_iter().map(|m| m.label).collect())
                    .label("Model"),
            );
            let devices = stt::devices();
            let at = devices.iter().position(|d| d.choice == self.device).unwrap_or(0);
            out.push(
                Param::choice("device", at, devices.into_iter().map(|d| d.label).collect())
                    .label("Run on"),
            );
        }
        out
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => self.enabled = v.as_bool().unwrap_or(self.enabled),
            "min_speech_s" => self.min_speech_s = v.as_f64().unwrap_or(self.min_speech_s),
            #[cfg(feature = "stt")]
            "root" => {
                self.root = std::path::PathBuf::from(v.as_str().unwrap_or_default());
                self.look();
            }
            #[cfg(feature = "stt")]
            "dir" => {
                self.explicit_dir = v.as_str().filter(|s| !s.is_empty()).map(Into::into);
                self.look();
            }
            #[cfg(feature = "stt")]
            "model" => {
                let id = match &v {
                    ParamValue::Choice(i) => self.models().get(*i).map(|m| m.id.clone()),
                    _ => v.as_str().map(str::to_string),
                };
                if let Some(id) = id.filter(|id| *id != self.model_id) {
                    self.model_id = id;
                    self.reload();
                }
            }
            #[cfg(feature = "stt")]
            "device" => {
                let choice = match &v {
                    ParamValue::Choice(i) => stt::devices().get(*i).map(|d| d.choice),
                    _ => v.as_str().map(stt::DeviceChoice::parse),
                };
                if let Some(c) = choice.filter(|c| *c != self.device) {
                    self.device = c;
                    self.reload();
                }
            }
            // Load it now rather than on the first thing worth reading. The
            // fetch is tens of megabytes and the load is seconds, and an
            // operator who has just switched transcription on should be able
            // to find out whether it works without waiting for somebody to
            // talk.
            #[cfg(feature = "stt")]
            "load" => {
                if v.as_bool().unwrap_or(false) {
                    self.worker();
                }
            }
            _ => {
                return Err(common::Error::other(format!(
                    "transcribe_live: unknown parameter {name:?}"
                )))
            }
        }
        Ok(())
    }
}

/// How long a partial is worth showing after nothing more arrives. Used by a
/// view deciding whether a line is still live.
pub const LIVE: Duration = Duration::from_secs(10);

#[cfg(test)]
mod tests {
    use super::*;

    fn voice(
        system: &'static str,
        hz: f64,
        to: Option<&str>,
        from: Option<&str>,
        level: f32,
        n: usize,
    ) -> common::Voice {
        common::Voice {
            system,
            channel_hz: hz,
            to: to.map(|s| s.to_string()),
            from: from.map(|s| s.to_string()),
            rate: 8_000.0,
            pcm: vec![level; n],
        }
    }

    #[test]
    fn a_key_says_what_the_receiver_knows_and_no_more() {
        let fm = voice(crate::audiobus::ANALOGUE, 145_500_000.0, None, None, 0.1, 8);
        assert_eq!(key_of(&fm), "Audio:145500000::");
        let dmr = voice("DMR", 435_000_000.0, Some("9"), Some("1234567"), 0.1, 8);
        assert_eq!(key_of(&dmr), "DMR:435000000:9:1234567");
    }

    /// The parts come back out, so a view holding a key can say who was
    /// talking without keeping a second copy of it.
    #[test]
    fn a_key_reads_back_as_its_parts() {
        let s = Speaker::parse("DMR:435000000:9:1234567").expect("a key");
        assert_eq!(s.proto, "DMR");
        assert_eq!(s.freq_hz, 435_000_000);
        assert_eq!(s.channel.as_deref(), Some("9"));
        assert_eq!(s.speaker.as_deref(), Some("1234567"));
        let bare = Speaker::parse("Audio:145500000::").expect("a key");
        assert_eq!(bare.channel, None);
        assert_eq!(bare.speaker, None);
        assert_eq!(bare.key(), "Audio:145500000::");
    }

    /// Somebody talking, pausing for breath, and stopping. The pause is not
    /// the end of the utterance: cutting on the first quiet block turns one
    /// sentence into four.
    #[test]
    fn a_pause_for_breath_does_not_end_an_utterance() {
        let mut n = LiveTranscribeNode::new();
        let key = "Audio:145500000::".to_string();
        let at = Instant::now();
        let block = 0.1;
        let loud = voice(crate::audiobus::ANALOGUE, 145_500_000.0, None, None, 0.2, 800);
        let quiet = voice(crate::audiobus::ANALOGUE, 145_500_000.0, None, None, 0.0, 800);
        for _ in 0..10 {
            assert!(!n.collect(key.clone(), &loud, block, at));
        }
        // Half the hang time of silence, then more speech.
        for _ in 0..4 {
            assert!(!n.collect(key.clone(), &quiet, block, at));
        }
        assert!(!n.collect(key.clone(), &loud, block, at));
        for _ in 0..8 {
            n.collect(key.clone(), &quiet, block, at);
        }
        assert!(n.collect(key.clone(), &quiet, block, at), "the utterance never ended");
        assert!(n.held_seconds(&key) > 1.0, "the audio was thrown away");
    }

    /// A channel sitting open with nobody on it collects nothing, or every
    /// squelched receiver would hand the model an hour of noise.
    #[test]
    fn an_open_channel_with_nobody_on_it_is_not_speech() {
        let mut n = LiveTranscribeNode::new();
        let quiet = voice(crate::audiobus::ANALOGUE, 145_500_000.0, None, None, 0.0, 800);
        for _ in 0..100 {
            assert!(!n.collect("Audio:145500000::".into(), &quiet, 0.1, Instant::now()));
        }
        assert_eq!(n.held_seconds("Audio:145500000::"), 0.0);
    }

    /// A partial is replaced by the next reading of the same utterance; a
    /// settled one is its own line.
    #[test]
    fn a_growing_utterance_is_one_line_that_gets_longer() {
        let mut log = TranscriptLog::default();
        let at = Instant::now();
        let u = |text: &str, settled: bool| Utterance {
            key: "Audio:145500000::".into(),
            at,
            seconds: 1.0,
            text: text.into(),
            settled,
            confidence: -0.2,
            credible: true,
        };
        log.push(u("all stations", false));
        log.push(u("all stations this is", false));
        assert_eq!(log.of("Audio:145500000::").len(), 1);
        assert_eq!(log.latest("Audio:145500000::").unwrap().text, "all stations this is");
        log.push(u("All stations, this is EI2ABC.", true));
        assert_eq!(log.of("Audio:145500000::").len(), 1, "the settled text is the same utterance");
        assert!(log.latest("Audio:145500000::").unwrap().settled);
    }

    /// The whole path with a model behind it: speech in as voice blocks,
    /// text out of the log. Skipped without a model, since fetching one is
    /// not something a test should do to somebody's machine.
    #[cfg(feature = "stt")]
    #[test]
    fn speech_arrives_as_text_in_the_log() {
        // A wav of somebody talking, named by `WAVESHARK_TEST_WAV`. Not in
        // the corpus: that holds IQ a decoder is asserted against, and this
        // is a sanity check on the path from a voice block to a line of text.
        let dir = crate::chain::default_model_dir();
        let named = std::env::var("WAVESHARK_TEST_WAV").unwrap_or_else(|_| "/tmp/jfk.wav".into());
        let wav = std::path::Path::new(&named);
        if !dir.join("config.json").exists() || !wav.exists() {
            println!("no model in {} or no {named}; skipping", dir.display());
            return;
        }
        let r = hound::WavReader::open(wav).expect("the wav");
        let rate = r.spec().sample_rate as f64;
        let pcm: Vec<f32> =
            r.into_samples::<i16>().filter_map(|s| s.ok()).map(|s| s as f32 / 32768.0).collect();

        let mut n = LiveTranscribeNode::new().in_dir(&dir).into_log(Default::default());
        let mut events = Vec::new();
        let tags = Vec::new();
        let mut new_tags = Vec::new();
        let ins = [PortSpec {
            spec: StreamSpec { kind: PortKind::Voice, rate, ..Default::default() },
            latency: 0,
        }];
        let block = (rate * 0.1) as usize;
        let mut blocks: Vec<Vec<f32>> = pcm.chunks(block).map(|c| c.to_vec()).collect();
        // Silence on the end, so the utterance is finished rather than still
        // being spoken when the samples run out.
        blocks.extend((0..20).map(|_| vec![0.0; block]));
        let key = "Audio:145500000::";
        for b in blocks {
            let payload = Payload::Voice(vec![common::Voice {
                system: crate::audiobus::ANALOGUE,
                channel_hz: 145_500_000.0,
                to: None,
                from: None,
                rate,
                pcm: b,
            }]);
            let mut out = Payload::Voice(Vec::new());
            let mut c = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            c.block_seconds = 0.1;
            n.process(&payload, &mut out, &mut c).unwrap();
        }
        // The model is on its own thread, so the answer arrives on a later
        // block the way it does in the receiver.
        for _ in 0..600 {
            if n.log().lock().latest(key).is_some_and(|u| u.settled) {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
            let mut out = Payload::Voice(Vec::new());
            let mut c = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            c.block_seconds = 0.1;
            n.process(&Payload::Voice(Vec::new()), &mut out, &mut c).unwrap();
        }
        let u = n.log().lock().latest(key).cloned().expect("nothing was transcribed");
        println!("{:?} {:?}", u.settled, u.text);
        assert!(u.text.to_lowercase().contains("country"), "read as {:?}", u.text);
    }

    /// The node that asked for a reading is gone by the time it comes back,
    /// which is what a rebuild does to it: the audio bus is rebuilt whenever
    /// a channel comes or goes, and the transcriber hangs off the bus. The
    /// reading still has to be written down. It was not: the worker handed
    /// its text back to the node, and a dropped node is a closed channel,
    /// so every read that finished across a rebuild vanished. On screen that
    /// was a card saying the model had read the words and a transcript with
    /// no lines in it.
    #[cfg(feature = "stt")]
    #[test]
    fn a_reading_outlives_the_node_that_asked_for_it() {
        let dir = crate::chain::default_model_dir();
        let named = std::env::var("WAVESHARK_TEST_WAV").unwrap_or_else(|_| "/tmp/jfk.wav".into());
        let wav = std::path::Path::new(&named);
        if !dir.join("config.json").exists() || !wav.exists() {
            println!("no model in {} or no {named}; skipping", dir.display());
            return;
        }
        let r = hound::WavReader::open(wav).expect("the wav");
        let rate = r.spec().sample_rate as f64;
        let pcm: Vec<f32> =
            r.into_samples::<i16>().filter_map(|s| s.ok()).map(|s| s as f32 / 32768.0).collect();
        let log: SharedLog = Default::default();
        let mut n = LiveTranscribeNode::new().in_dir(&dir).into_log(log.clone());
        // The model up first, so the read below is the model reading and
        // not the model loading.
        n.set_param("load", ParamValue::Bool(true)).unwrap();
        while n.engine().state != ModelState::Ready {
            std::thread::sleep(Duration::from_millis(50));
        }
        let mut events = Vec::new();
        let tags = Vec::new();
        let mut new_tags = Vec::new();
        let ins = [PortSpec {
            spec: StreamSpec { kind: PortKind::Voice, rate, ..Default::default() },
            latency: 0,
        }];
        let block = (rate * 0.1) as usize;
        let mut blocks: Vec<Vec<f32>> = pcm.chunks(block).map(|c| c.to_vec()).collect();
        blocks.extend((0..20).map(|_| vec![0.0; block]));
        for b in blocks {
            let payload = Payload::Voice(vec![common::Voice {
                system: crate::audiobus::ANALOGUE,
                channel_hz: 145_500_000.0,
                to: None,
                from: None,
                rate,
                pcm: b,
            }]);
            let mut out = Payload::Voice(Vec::new());
            let mut c = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            c.block_seconds = 0.1;
            n.process(&payload, &mut out, &mut c).unwrap();
        }
        assert!(n.engine().busy, "nothing was asked for");
        // The rebuild: the node goes while the model still has the window.
        // What was asked for is the whole utterance so far; whether the node
        // would have asked for it again as settled is beside the point, since
        // it is not there to ask.
        drop(n);
        let key = "Audio:145500000::";
        for _ in 0..600 {
            if log.lock().latest(key).is_some() {
                break;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        let u = log.lock().latest(key).cloned().expect("the reading died with the node");
        // The first partial, since that is what was in flight: the node was
        // gone before it could ask for the rest, so this is the last word.
        assert!(u.text.to_lowercase().contains("fellow"), "read as {:?}", u.text);
        assert!(u.settled, "a partial nobody will replace is not a partial");
    }

    /// The interface folds the published window in on every frame, so the
    /// same utterances arrive over and over. One utterance is one start time
    /// on one key, so the log has to be the same size after the tenth pass
    /// as after the first.
    #[test]
    fn the_same_window_folded_in_again_is_the_same_log() {
        let mut log = TranscriptLog::default();
        let at = Instant::now();
        let window = |text: &str| {
            vec![
                Utterance {
                    key: "Audio:145500000::".into(),
                    at,
                    seconds: 2.0,
                    text: text.into(),
                    settled: true,
                    confidence: -0.2,
                    credible: true,
                },
                Utterance {
                    key: "DMR:435000000:9:1234567".into(),
                    at: at + Duration::from_secs(3),
                    seconds: 1.0,
                    text: "go ahead".into(),
                    settled: false,
                    confidence: -0.4,
                    credible: true,
                },
            ]
        };
        for _ in 0..10 {
            for u in window("all stations, this is EI2ABC") {
                log.push(u);
            }
        }
        assert_eq!(log.len(), 2);
        assert_eq!(log.keys().len(), 2);
        // And the partial still becomes the settled line rather than a
        // second one beside it.
        for u in window("all stations, this is EI2ABC") {
            log.push(Utterance { settled: true, text: format!("{} over", u.text), ..u });
        }
        assert_eq!(log.len(), 2);
        assert_eq!(log.of("DMR:435000000:9:1234567").len(), 1);
        assert_eq!(log.latest("DMR:435000000:9:1234567").unwrap().text, "go ahead over");
        assert!(log.has("Audio:145500000::"));
        assert!(!log.has("Audio:433000000::"), "a conversation nobody spoke on");
    }

    /// The model can be brought up without waiting for somebody to talk, and
    /// it says what it is running on when it is. That is the whole of what
    /// the transcript view's card reads.
    #[cfg(feature = "stt")]
    #[test]
    fn the_model_loads_on_request_and_says_where_it_is_running() {
        let dir = crate::chain::default_model_dir();
        if !dir.join("config.json").exists() {
            println!("no model in {}; skipping", dir.display());
            return;
        }
        let mut n = LiveTranscribeNode::new().in_dir(&dir);
        let cold = n.engine();
        assert_eq!(cold.state, ModelState::Cold);
        assert!(cold.present, "the files are in {}", dir.display());
        assert!(cold.bytes > 0);
        assert!(!cold.weights.is_empty(), "what is on disc is not named");
        assert_eq!(cold.reads, 0);

        n.set_param("load", ParamValue::Bool(true)).unwrap();
        let waited = std::time::Instant::now();
        while n.engine().state != ModelState::Ready && waited.elapsed().as_secs() < 120 {
            std::thread::sleep(Duration::from_millis(100));
        }
        let up = n.engine();
        assert_eq!(up.state, ModelState::Ready, "the model never loaded");
        assert!(
            ["CPU", "GPU", "Metal"].iter().any(|d| up.device.starts_with(d)),
            "running on {:?}",
            up.device
        );
    }

    /// A run of speech that fills the window is cut where it is quietest, so
    /// the line before the cut ends at a pause rather than in the middle of a
    /// word.
    #[test]
    fn a_full_window_is_cut_at_the_pause_and_not_at_the_end() {
        let rate = 8_000.0;
        let mut pcm = vec![0.2f32; (rate * 30.0) as usize];
        // A gap two seconds from the end, which is inside the search.
        let gap = pcm.len() - (rate * 2.0) as usize;
        for s in &mut pcm[gap..gap + (rate * 0.2) as usize] {
            *s = 0.0;
        }
        let at = cut_point(&pcm, rate);
        assert!(at >= gap && at <= gap + (rate * 0.2) as usize, "cut at {at}, gap at {gap}");
        // And with nothing quieter than anything else it still cuts inside
        // the search rather than losing the last seconds.
        let flat = vec![0.2f32; (rate * 30.0) as usize];
        let at = cut_point(&flat, rate);
        assert!(at >= flat.len() - (rate * CUT_SEARCH_S) as usize);
        assert!(at <= flat.len());
    }

    /// Squelch noise is loud enough to collect and no threshold tells it from
    /// speech, so the model's own verdict is what stops it: two windows with
    /// nothing credible in them and the conversation is left alone until the
    /// channel closes. Without this a hissing repeater is read for as long as
    /// it hisses, which is what a receiver holding ninety seconds of nothing
    /// looks like on screen.
    #[test]
    fn a_channel_the_model_finds_nothing_in_is_left_alone() {
        let mut n = LiveTranscribeNode::new();
        let key = "Audio:145500000::".to_string();
        let noise = voice(crate::audiobus::ANALOGUE, 145_500_000.0, None, None, 0.02, 800);
        let quiet = voice(crate::audiobus::ANALOGUE, 145_500_000.0, None, None, 0.0, 800);
        for _ in 0..50 {
            n.collect(key.clone(), &noise, 0.1, Instant::now());
        }
        assert!(n.held_seconds(&key) > 4.0, "noise is collected, since it is loud");

        n.read_back(&key, false);
        assert!(n.held_seconds(&key) > 4.0, "one empty window is not evidence");
        n.read_back(&key, false);
        assert_eq!(n.held_seconds(&key), 0.0, "the second one is");
        for _ in 0..50 {
            n.collect(key.clone(), &noise, 0.1, Instant::now());
        }
        assert_eq!(n.held_seconds(&key), 0.0, "and nothing more is collected");

        // Until the channel closes, which is what starts it listening again.
        n.collect(key.clone(), &quiet, 0.1, Instant::now());
        for _ in 0..20 {
            n.collect(key.clone(), &noise, 0.1, Instant::now());
        }
        assert!(n.held_seconds(&key) > 1.0);
    }

    #[test]
    fn conversations_are_kept_apart_and_the_oldest_is_forgotten() {
        let mut log = TranscriptLog::default();
        for i in 0..(MAX_KEYS + 8) {
            log.push(Utterance {
                key: format!("Audio:{i}::"),
                at: Instant::now(),
                seconds: 1.0,
                text: format!("{i}"),
                settled: true,
                confidence: -0.2,
                credible: true,
            });
        }
        assert_eq!(log.keys().len(), MAX_KEYS);
        assert!(log.of("Audio:0::").is_empty(), "the oldest conversation was kept");
        assert_eq!(
            log.latest(&format!("Audio:{}::", MAX_KEYS + 7)).unwrap().text,
            format!("{}", MAX_KEYS + 7)
        );
    }
}
