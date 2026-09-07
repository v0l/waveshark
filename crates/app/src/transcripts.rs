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

/// Longest run of speech kept as one utterance. A repeater left keyed would
/// otherwise grow a buffer without bound.
const MAX_UTTERANCE_S: f64 = 120.0;

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
/// In memory and bounded. Nothing here is written to disk yet: the packet log
/// holds evidence and a transcript is not evidence, so where transcripts
/// belong on disk is a decision that has not been made.
#[derive(Debug, Default)]
pub struct TranscriptLog {
    by_key: HashMap<String, Vec<Utterance>>,
    /// Keys in the order they were last spoken on, oldest first.
    order: Vec<String>,
}

impl TranscriptLog {
    /// Add or replace. An unsettled utterance replaces the last unsettled one
    /// on the same key, which is what makes a growing window read as one line
    /// getting longer rather than as a page of half sentences.
    pub fn push(&mut self, u: Utterance) {
        let list = self.by_key.entry(u.key.clone()).or_default();
        match list.last_mut() {
            Some(last) if !last.settled && last.at == u.at => *last = u.clone(),
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

    pub fn is_empty(&self) -> bool {
        self.by_key.is_empty()
    }

    pub fn clear(&mut self) {
        self.by_key.clear();
        self.order.clear();
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
}

/// The streaming transcriber, as a node on the audio bus tap.
pub struct LiveTranscribeNode {
    log: TranscriptLog,
    talking: HashMap<String, Talking>,
    enabled: bool,
    /// Shortest run of speech worth reading. A squelch tail transcribes as
    /// "Thank you." with high confidence.
    min_speech_s: f64,
    #[cfg(feature = "stt")]
    worker: Option<Worker>,
    #[cfg(feature = "stt")]
    dir: std::path::PathBuf,
    #[cfg(feature = "stt")]
    repo: String,
    #[cfg(feature = "stt")]
    reported: bool,
}

impl Default for LiveTranscribeNode {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveTranscribeNode {
    pub fn new() -> Self {
        Self {
            log: TranscriptLog::default(),
            talking: HashMap::new(),
            enabled: true,
            min_speech_s: 0.6,
            #[cfg(feature = "stt")]
            worker: None,
            #[cfg(feature = "stt")]
            dir: std::path::PathBuf::new(),
            #[cfg(feature = "stt")]
            repo: stt::DEFAULT_REPO.to_string(),
            #[cfg(feature = "stt")]
            reported: false,
        }
    }

    #[cfg(feature = "stt")]
    pub fn in_dir(mut self, dir: impl Into<std::path::PathBuf>) -> Self {
        self.dir = dir.into();
        self
    }

    #[cfg(feature = "stt")]
    pub fn model(mut self, repo: &str) -> Self {
        self.repo = repo.to_string();
        self
    }

    pub fn log(&self) -> &TranscriptLog {
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
        });
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
        let seconds = entry.pcm.len() as f64 / entry.rate;
        entry.quiet_s >= HANG_S || seconds >= MAX_UTTERANCE_S
    }

    /// Seconds of audio held for a key, for tests and for a status line.
    pub fn held_seconds(&self, key: &str) -> f64 {
        self.talking
            .get(key)
            .map(|t| t.pcm.len() as f64 / t.rate)
            .unwrap_or(0.0)
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

    pub(super) struct Done {
        pub key: String,
        pub at: Instant,
        pub seconds: f64,
        pub settled: bool,
        pub result: Result<stt::Transcript>,
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
                let dir = self.dir.clone();
                let repo = self.repo.clone();
                std::thread::Builder::new()
                    .name("whisper-live".into())
                    .spawn(move || run(dir, repo, jobs_rx, done_tx))
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
                // The last reading of an utterance is the end of it: what was
                // held is now text, and holding it into the next one would
                // read the same words again with somebody else's in front.
                if done.settled {
                    self.talking.remove(&done.key);
                } else {
                    self.pump(&done.key);
                }
                match done.result {
                    Ok(t) => {
                        let text = t.text.trim().to_string();
                        if !text.is_empty() {
                            self.log.push(Utterance {
                                key: done.key,
                                at: done.at,
                                seconds: done.seconds,
                                text,
                                settled: done.settled,
                                confidence: t.avg_logprob() as f32,
                            });
                        }
                    }
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
            let Some(t) = self.talking.get(key) else { return };
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
            } else if !short && seconds - t.asked_at_s >= PARTIAL_EVERY_S {
                self.ask(key, false);
            }
        }

        /// Send what is held for a key to the model.
        pub(super) fn ask(&mut self, key: &str, settled: bool) {
            let Some(t) = self.talking.get(key) else { return };
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
            let asked = t.pcm.len() as f64 / t.rate;
            if self.worker().is_some_and(|w| w.jobs.try_send(job).is_ok()) {
                if let Some(t) = self.talking.get_mut(key) {
                    t.waiting = true;
                    t.asked_at_s = asked;
                }
            }
        }
    }

    fn run(dir: std::path::PathBuf, repo: String, jobs: Receiver<Job>, done: Sender<Done>) {
        let loaded =
            stt::ensure(&repo, &dir).and_then(|f| stt::Whisper::load(&f, stt::best_device(), None));
        let mut model = match loaded {
            Ok(m) => m,
            Err(e) => {
                let msg = format!("whisper in {}: {e}", dir.display());
                while let Ok(job) = jobs.recv() {
                    let _ = done.send(Done {
                        key: job.key,
                        at: job.at,
                        seconds: 0.0,
                        settled: job.settled,
                        result: Err(common::Error::other(&msg)),
                    });
                }
                return;
            }
        };
        while let Ok(job) = jobs.recv() {
            let seconds = job.pcm.len() as f64 / job.rate.max(1.0);
            let result = model.transcribe(&job.pcm, job.rate);
            if done
                .send(Done { key: job.key, at: job.at, seconds, settled: job.settled, result })
                .is_err()
            {
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
            return Err(common::Error::other(
                "the live transcriber reads the audio bus tap",
            ));
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
        vec![
            Param::bool("enabled", self.enabled).label("Transcribe what is heard"),
            Param::float("min_speech_s", self.min_speech_s, 0.1..=5.0)
                .unit("s")
                .label("Shortest speech worth reading"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => self.enabled = v.as_bool().unwrap_or(self.enabled),
            "min_speech_s" => self.min_speech_s = v.as_f64().unwrap_or(self.min_speech_s),
            #[cfg(feature = "stt")]
            "dir" => self.dir = std::path::PathBuf::from(v.as_str().unwrap_or_default()),
            #[cfg(feature = "stt")]
            "model" => {
                if let Some(t) = v.as_str() {
                    if t != self.repo {
                        self.repo = t.to_string();
                        self.worker = None;
                        self.reported = false;
                    }
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

    fn voice(system: &'static str, hz: f64, to: Option<&str>, from: Option<&str>, level: f32, n: usize) -> common::Voice {
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
        assert!(
            n.collect(key.clone(), &quiet, block, at),
            "the utterance never ended"
        );
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

        let mut n = LiveTranscribeNode::new().in_dir(&dir);
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
            if n.log().latest(key).is_some_and(|u| u.settled) {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
            let mut out = Payload::Voice(Vec::new());
            let mut c = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
            c.block_seconds = 0.1;
            n.process(&Payload::Voice(Vec::new()), &mut out, &mut c).unwrap();
        }
        let u = n.log().latest(key).expect("nothing was transcribed");
        println!("{:?} {:?}", u.settled, u.text);
        assert!(
            u.text.to_lowercase().contains("country"),
            "read as {:?}",
            u.text
        );
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
            });
        }
        assert_eq!(log.keys().len(), MAX_KEYS);
        assert!(log.of("Audio:0::").is_empty(), "the oldest conversation was kept");
        assert_eq!(log.latest(&format!("Audio:{}::", MAX_KEYS + 7)).unwrap().text, format!("{}", MAX_KEYS + 7));
    }
}
