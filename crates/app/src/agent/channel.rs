//! Talking to the agent over the air.
//!
//! A channel marked as the agent's is an ordinary channel: NFM on PMR446 or
//! anywhere else, demodulated, squelched and transcribed like any other. What
//! this adds is the loop between the transcript and the transmitter. An over
//! that begins with the wake word is put to the model with every tool it has,
//! the answer is spoken by the speech server, and the samples go into a
//! [`audio::Speaker`] the transmit chain reads.
//!
//! Two rules keep it off other people's overs. It answers nothing that does
//! not address it by name, and it keys nothing until the squelch has been
//! shut for a hang time: a station drawing breath mid-over must not be
//! transmitted over, and the agent has no ears while it transmits.

use super::{Desk, chat, config::Config, voice};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Longest a single answer may hold the channel, in seconds.
///
/// A model that decides to read out a packet list would otherwise key up for
/// several minutes, which on a shared channel is the worst thing this can do.
/// The text is cut to fit before it is ever spoken.
pub const MAX_OVER_S: f64 = 30.0;

/// The rate the queue runs at, whatever the speech server produced.
///
/// Fixed rather than taken from the reply: the transmit chain reads the rate
/// once, when it is built, and the radio thread holds the queue from the
/// moment it starts. A queue that changed rate would have to be replaced, and
/// the stage would go on resampling at the old one.
pub const VOICE_RATE: f64 = 24_000.0;

/// Words a second, for guessing how long a reply will take to say.
///
/// Measured against the usual synthetic voices reading plain sentences: a
/// little under three words a second. Only used to cut a reply down before
/// paying for the speech, so it is deliberately pessimistic.
const WORDS_PER_S: f64 = 2.6;

/// What the agent on the air is doing.
///
/// Every step of it, because they take different lengths of time and fail in
/// different ways: a model downloading three gigabytes, a chat server not
/// answering and a card generating speech all used to read as "thinking",
/// and an operator watching that word for two minutes cannot tell which of
/// them is happening or whether anything is happening at all.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Waiting for somebody to say its name.
    Listening,
    /// Somebody did, and the chat model has the question.
    Asking,
    /// There is an answer, and it is being turned into speech.
    Speaking,
    /// There is an answer to give, waiting for the channel to be free.
    Holding,
    /// Keyed, saying it.
    OnAir,
}

impl State {
    /// What a pane prints for it.
    pub fn label(self) -> &'static str {
        match self {
            Self::Listening => "listening",
            Self::Asking => "asking the model",
            Self::Speaking => "making speech",
            Self::Holding => "waiting for the channel",
            Self::OnAir => "on air",
        }
    }

    /// Whether the agent is working on an answer rather than waiting for a
    /// question.
    pub fn busy(self) -> bool {
        !matches!(self, Self::Listening)
    }
}

/// Which half of the background work is running, written by the task and
/// read by whoever draws the state: the two are seconds and minutes apart
/// and there is nothing else to tell them by.
#[derive(Clone, Default)]
struct Stage(Arc<AtomicBool>);

impl Stage {
    fn speaking(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    fn state(&self) -> State {
        match self.0.load(Ordering::Relaxed) {
            true => State::Speaking,
            false => State::Asking,
        }
    }
}

/// What the interface should do about it this frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Move {
    /// Key this channel; there is something queued to say.
    Key(u64),
    /// Let go: the queue ran dry, or the answer went on too long.
    Unkey,
}

/// The last over offered to the agent, and what became of it.
///
/// Every over on the channel is read and most are not for the agent. Without
/// this the readout says "listening" whether nobody has spoken, somebody has
/// spoken and not used the name, or the agent cannot answer at all, and the
/// three are the same picture: an operator talking into a channel that never
/// replies has nothing to go on.
#[derive(Clone, Debug, PartialEq)]
pub struct Heard {
    pub at: Instant,
    pub text: String,
    /// Why it was not taken, or `None` when it was.
    pub passed: Option<Passed>,
}

/// Why an over the agent heard was not answered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Passed {
    /// It does not start with the agent's name.
    NotAddressed,
    /// The name and nothing after it.
    NothingAsked,
    /// It is still working on the one before.
    Busy,
    /// It cannot answer at all: see `Config::voice_fault`.
    Mute,
}

impl Passed {
    pub fn label(self) -> &'static str {
        match self {
            Self::NotAddressed => "not addressed to it",
            Self::NothingAsked => "nothing asked",
            Self::Busy => "still on the last one",
            Self::Mute => "it cannot answer",
        }
    }
}

/// What the agent heard and what it said, for the pane.
pub struct Exchange {
    pub at: Instant,
    pub heard: String,
    pub said: Result<String, String>,
}

pub struct AgentChannel {
    /// The channel it listens and answers on, or `None` for off.
    pub on: Option<u64>,
    pub state: State,
    /// What it has said and been told, newest last.
    pub log: Vec<Exchange>,
    /// The samples waiting to go out, which the transmit chain reads.
    speaker: Arc<audio::Speaker>,
    /// The conversation, which is not the one in the Agent view: a voice
    /// channel is a different correspondent.
    history: Vec<serde_json::Value>,
    /// The answer being worked on.
    pending: Option<crossbeam_channel::Receiver<Answer>>,
    /// What the last over said, so the reply can be logged beside it.
    asked: String,
    /// When the squelch was last open, which is what the hang is measured
    /// from.
    last_busy: Option<Instant>,
    /// When the key went down, so an over that will not end can be ended.
    keyed_at: Option<Instant>,
    /// Utterances already answered, by the instant they started: the
    /// transcript replaces a partial with a settled reading of the same over,
    /// and answering both would be answering twice.
    answered: Option<Instant>,
    /// Which half of the pending work is running.
    stage: Stage,
    /// The last over offered to it, taken or not.
    pub last: Option<Heard>,
    stop: Arc<AtomicBool>,
}

/// Speech at the queue's rate, resampled where the server used another.
fn at_voice_rate(said: &voice::Said) -> Vec<f32> {
    if (said.rate - VOICE_RATE).abs() < 1.0 {
        return said.samples.clone();
    }
    let mut out = Vec::new();
    // Four taps: this is speech on its way to a 3 kHz channel.
    audio::Resampler::new(said.rate, VOICE_RATE, 4).process(&said.samples, &mut out);
    out
}

/// What the background work comes back with.
struct Answer {
    said: Result<String, String>,
    history: Vec<serde_json::Value>,
    speech: Option<voice::Said>,
}

impl Default for AgentChannel {
    fn default() -> Self {
        Self {
            on: None,
            state: State::Listening,
            log: Vec::new(),
            speaker: Arc::new(audio::Speaker::new(VOICE_RATE)),
            history: Vec::new(),
            pending: None,
            asked: String::new(),
            last_busy: None,
            keyed_at: None,
            answered: None,
            stage: Stage::default(),
            last: None,
            stop: Arc::new(AtomicBool::new(false)),
        }
    }
}

impl AgentChannel {
    /// The queue the transmit chain reads. One for the life of the receiver,
    /// so a rebuild of the graph does not lose what is queued.
    pub fn speaker(&self) -> Arc<audio::Speaker> {
        self.speaker.clone()
    }

    /// Stop everything: drop what was going to be said, and let go.
    pub fn stand_down(&mut self) -> Option<Move> {
        self.stop.store(true, Ordering::Relaxed);
        self.speaker.cut();
        self.pending = None;
        let keyed = self.keyed_at.take().is_some();
        self.state = State::Listening;
        keyed.then_some(Move::Unkey)
    }

    /// Whether this over is addressed to the agent, and what is left of it
    /// once the name is taken off.
    ///
    /// Loose on purpose: a transcript of speech off a squelched FM channel
    /// carries commas, full stops and whatever the model made of the syllable
    /// before the name. What is required is that the name is near the start.
    pub fn addressed(wake: &str, text: &str) -> Option<String> {
        let wake = wake.trim().to_lowercase();
        if wake.is_empty() {
            return None;
        }
        let lower = text.to_lowercase();
        let at = lower.find(&wake)?;
        // Near the start: a name in the middle of a sentence is somebody
        // talking about the agent, not to it. Four words of room, because
        // "this is M0ABC, shark, ..." is how a station actually opens.
        if lower[..at].split_whitespace().count() > 4 {
            return None;
        }
        let rest = text[at + wake.len()..].trim_start_matches([' ', ',', '.', ':', '?', '!', '-']);
        Some(rest.trim().to_string())
    }

    /// A reply cut to what will fit in one over.
    fn to_the_point(text: &str) -> String {
        let words: Vec<&str> = text.split_whitespace().collect();
        let most = (MAX_OVER_S * WORDS_PER_S) as usize;
        match words.len() > most {
            true => {
                format!("{}, and there is more than will fit in one over", words[..most].join(" "))
            }
            false => words.join(" "),
        }
    }

    /// What the model is told before anything else, on top of the receiver's
    /// own brief.
    fn brief(config: &Config) -> String {
        format!(
            "You are answering over a radio channel, by voice, to somebody who cannot see a \
             screen. Keep it to one or two short sentences, no lists, no punctuation a \
             speaker cannot say. Say numbers as words a listener can follow. You are called \
             {}.",
            config.wake.trim()
        )
    }

    /// Something was said on the channel. Returns whether it was taken.
    pub fn heard(
        &mut self,
        config: &Config,
        desk: &Desk,
        rt: &tokio::runtime::Handle,
        at: Instant,
        text: &str,
    ) -> bool {
        if self.on.is_none() {
            return false;
        }
        // Said once per over rather than per frame: the same reading is
        // offered again every frame it stays in the transcript, and a note
        // that rewrote itself sixty times a second is the same note.
        let mut pass = |a: &mut Self, why: Passed| {
            if a.last.as_ref().map(|h| h.at) != Some(at) {
                a.last = Some(Heard { at, text: text.to_string(), passed: Some(why) });
            }
            false
        };
        if config.voice_fault().is_some() {
            return pass(self, Passed::Mute);
        }
        if self.answered == Some(at) {
            return false;
        }
        if self.state.busy() {
            return pass(self, Passed::Busy);
        }
        let Some(question) = Self::addressed(&config.wake, text) else {
            return pass(self, Passed::NotAddressed);
        };
        if question.is_empty() {
            return pass(self, Passed::NothingAsked);
        }
        self.last = Some(Heard { at, text: text.to_string(), passed: None });
        self.answered = Some(at);
        self.asked = question.to_string();
        self.stage = Stage::default();
        self.state = State::Asking;
        self.stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = crossbeam_channel::unbounded();
        self.pending = Some(rx);
        let mut history = self.history.clone();
        if history.is_empty() {
            history.push(serde_json::json!({ "role": "system", "content": Self::brief(config) }));
        }
        let (config, desk, question) = (config.clone(), desk.clone(), question.to_string());
        let stage = self.stage.clone();
        rt.spawn(async move {
            let answer = match chat::ask_once(config.clone(), history, desk, &question).await {
                Ok((said, history)) => {
                    let said = Self::to_the_point(&said);
                    if said.is_empty() {
                        Answer { said: Err("the model said nothing".into()), history, speech: None }
                    } else {
                        stage.speaking();
                        match voice::speak(&config, &said).await {
                            Ok(speech) => Answer { said: Ok(said), history, speech: Some(speech) },
                            Err(e) => Answer { said: Err(e), history, speech: None },
                        }
                    }
                }
                Err(e) => Answer { said: Err(e), history: Vec::new(), speech: None },
            };
            let _ = tx.send(answer);
        });
        true
    }

    /// Take what the background work finished, and decide about the key.
    ///
    /// `busy` is the channel's own squelch: the hang is measured from the
    /// last frame it was open.
    pub fn poll(&mut self, config: &Config, now: Instant, busy: bool) -> Option<Move> {
        let channel = self.on?;
        if busy {
            self.last_busy = Some(now);
        }
        // Which half of the work is running, while it is running: the model
        // has the question, or the answer is being made into speech.
        if self.pending.is_some() && matches!(self.state, State::Asking | State::Speaking) {
            self.state = self.stage.state();
        }
        if let Some(rx) = self.pending.as_ref()
            && let Ok(answer) = rx.try_recv()
        {
            self.pending = None;
            if !answer.history.is_empty() {
                self.history = answer.history;
            }
            match &answer.speech {
                Some(s) if !s.samples.is_empty() => {
                    self.speaker.say(&at_voice_rate(s));
                    self.state = State::Holding;
                }
                _ => self.state = State::Listening,
            }
            self.log.push(Exchange {
                at: now,
                heard: std::mem::take(&mut self.asked),
                said: answer.said,
            });
        }
        match self.state {
            State::Holding => {
                let quiet = match self.last_busy {
                    Some(t) => now.duration_since(t).as_secs_f64(),
                    None => f64::INFINITY,
                };
                if busy || quiet < config.hang_s.max(0.0) {
                    return None;
                }
                self.state = State::OnAir;
                self.keyed_at = Some(now);
                Some(Move::Key(channel))
            }
            State::OnAir => {
                let over_ran = self
                    .keyed_at
                    .is_some_and(|t| now.duration_since(t) > Duration::from_secs_f64(MAX_OVER_S));
                if self.speaker.waiting() > 0 && !over_ran {
                    return None;
                }
                self.speaker.cut();
                self.keyed_at = None;
                self.state = State::Listening;
                Some(Move::Unkey)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config {
            model: "m".into(),
            speech: crate::agent::config::Speech::Server,
            voice_url: "http://s/v1".into(),
            wake: "shark".into(),
            hang_s: 1.5,
            ..Config::default()
        }
    }

    /// Only an over that says the name, near the start, and what is left of
    /// it is the question.
    #[test]
    fn it_answers_to_its_name_and_nothing_else() {
        let f = |t: &str| AgentChannel::addressed("shark", t);
        assert_eq!(f("Shark, what is on the air?").as_deref(), Some("what is on the air?"));
        assert_eq!(f("shark what is on 446").as_deref(), Some("what is on 446"));
        // A callsign before the name is how somebody actually talks.
        assert_eq!(f("this is M0ABC shark tune to 145.5").as_deref(), Some("tune to 145.5"));
        // Talking about it rather than to it.
        assert_eq!(f("I was telling Bob that the shark thing is listening"), None);
        assert_eq!(f("nothing to do with it"), None);
        // No wake word set is no answering at all.
        assert_eq!(AgentChannel::addressed("", "shark hello"), None);
    }

    /// A reply too long for one over is cut before it is paid for, and the
    /// listener is told it was.\
    #[test]
    fn a_long_answer_is_cut_to_one_over() {
        let short = AgentChannel::to_the_point("  two   words  ");
        assert_eq!(short, "two words");
        let long = "word ".repeat(200);
        let cut = AgentChannel::to_the_point(&long);
        let words = cut.split_whitespace().count();
        assert!(words < 90, "{words} words is more than half a minute of speech");
        assert!(cut.ends_with("one over"), "{cut}");
    }

    /// The key waits for the channel to be quiet, and for the hang after it.
    #[test]
    fn it_waits_for_the_channel_before_it_keys() {
        let c = config();
        let mut a = AgentChannel { on: Some(3), state: State::Holding, ..Default::default() };
        a.speaker.say(&[0.1; 2_400]);
        let t0 = Instant::now();
        // Somebody is still talking.
        assert_eq!(a.poll(&c, t0, true), None);
        // The squelch shuts, but the hang has not run.
        assert_eq!(a.poll(&c, t0 + Duration::from_millis(500), false), None);
        assert_eq!(a.state, State::Holding);
        // Quiet for longer than the hang: key.
        let at = t0 + Duration::from_millis(1_700);
        assert_eq!(a.poll(&c, at, false), Some(Move::Key(3)));
        assert_eq!(a.state, State::OnAir);
        // Still saying it.
        assert_eq!(a.poll(&c, at + Duration::from_millis(100), false), None);
        // The queue drains, so the over ends.
        let mut out = Vec::new();
        audio::AudioSource::take(a.speaker.as_ref(), &mut out, 2_400);
        assert_eq!(a.poll(&c, at + Duration::from_millis(200), false), Some(Move::Unkey));
        assert_eq!(a.state, State::Listening);
    }

    /// An over that will not end is ended: a speech server that keeps
    /// producing, or a queue nothing is draining, must not hold the channel.
    #[test]
    fn an_over_that_runs_long_is_let_go() {
        let c = config();
        let mut a = AgentChannel { on: Some(1), state: State::Holding, ..Default::default() };
        a.speaker.say(&[0.1; 48_000]);
        let t0 = Instant::now();
        assert_eq!(a.poll(&c, t0, false), Some(Move::Key(1)));
        let late = t0 + Duration::from_secs_f64(MAX_OVER_S + 1.0);
        assert_eq!(a.poll(&c, late, false), Some(Move::Unkey));
        assert_eq!(a.speaker.waiting(), 0, "what was left is thrown away, not saved up");
    }

    /// Speech from a server at another rate is put on the queue at the
    /// queue's, because the transmit stage read that rate when it was built.
    #[test]
    fn speech_is_resampled_to_the_rate_the_chain_was_built_at() {
        let half = voice::Said { samples: vec![0.2; 12_000], rate: 12_000.0 };
        let at = at_voice_rate(&half);
        let ratio = at.len() as f64 / half.samples.len() as f64;
        assert!((ratio - 2.0).abs() < 0.01, "a second at 12 kHz is a second at 24 kHz");
        let same = voice::Said { samples: vec![0.2; 100], rate: VOICE_RATE };
        assert_eq!(at_voice_rate(&same).len(), 100, "nothing is resampled for nothing");
    }

    /// Standing down mid-over lets go and drops what was queued.
    #[test]
    fn standing_down_stops_the_transmission() {
        let mut a = AgentChannel { on: Some(2), state: State::OnAir, ..Default::default() };
        a.keyed_at = Some(Instant::now());
        a.speaker.say(&[0.5; 1_000]);
        assert_eq!(a.stand_down(), Some(Move::Unkey));
        assert_eq!(a.state, State::Listening);
        assert_eq!(a.speaker.waiting(), 0);
        // Standing down when it was not keyed asks for nothing.
        assert_eq!(a.stand_down(), None);
    }

    /// Nothing is answered while the agent is already busy with an over, and
    /// nothing is answered twice because the transcript settled it again.
    #[test]
    fn one_over_is_answered_once() {
        let c = config();
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        let at = Instant::now();
        assert!(a.heard(&c, &desk, rt.handle(), at, "shark what is on the air"));
        assert_eq!(a.state, State::Asking, "it has the question and is asking the model");
        // The same over read again, and a second question while it is busy.
        assert!(!a.heard(&c, &desk, rt.handle(), at, "shark what is on the air"));
        assert!(!a.heard(&c, &desk, rt.handle(), at + Duration::from_secs(1), "shark again"));
    }

    /// Every over the agent hears is accounted for, taken or not.
    ///
    /// The readout used to say "listening" whether nobody had spoken,
    /// somebody had spoken without using the name, or the agent could not
    /// answer at all because no wake word had ever been set. All three are an
    /// operator talking into a channel that never replies, and nothing on the
    /// screen told them apart.
    #[test]
    fn an_over_that_is_not_answered_says_why() {
        use super::Passed;
        let c = config();
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        let mut clock = Instant::now();
        let mut say = |a: &mut AgentChannel, c: &Config, text: &str| {
            clock += Duration::from_secs(1);
            a.heard(c, &desk, rt.handle(), clock, text)
        };

        // Somebody talking on the channel, to somebody else.
        assert!(!say(&mut a, &c, "uh, hey, can you hear that?"));
        let h = a.last.clone().expect("it heard the over");
        assert_eq!(h.text, "uh, hey, can you hear that?");
        assert_eq!(h.passed, Some(Passed::NotAddressed));

        // The name and nothing after it.
        assert!(!say(&mut a, &c, "shark"));
        assert_eq!(a.last.as_ref().and_then(|h| h.passed), Some(Passed::NothingAsked));

        // A question, taken.
        assert!(say(&mut a, &c, "shark what is on the air"));
        assert_eq!(a.last.as_ref().and_then(|h| h.passed), None, "it was taken");
        assert_eq!(a.state, State::Asking);

        // And another while it is working on the first.
        assert!(!say(&mut a, &c, "shark are you there"));
        assert_eq!(a.last.as_ref().and_then(|h| h.passed), Some(Passed::Busy));

        // A receiver whose wake word nobody ever set hears everything and can
        // answer none of it, and says so rather than saying "listening".
        let mut mute = AgentChannel { on: Some(1), ..Default::default() };
        let no_name = Config { wake: String::new(), ..config() };
        assert_eq!(no_name.voice_fault(), Some("no wake word"));
        assert!(!say(&mut mute, &no_name, "shark what is on the air"));
        assert_eq!(mute.last.as_ref().and_then(|h| h.passed), Some(Passed::Mute));
    }

    /// The state says which half of the work is running.
    ///
    /// Asking a model and making speech out of its answer are seconds and
    /// minutes apart, and the speech half may be fetching gigabytes. One word
    /// covering both is a readout an operator cannot use to decide whether to
    /// wait or to go and look at the settings.
    #[test]
    fn the_state_says_whether_it_is_asking_or_speaking() {
        let c = config();
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        assert_eq!(a.state, State::Listening);
        assert_eq!(a.state.label(), "listening");
        assert!(!a.state.busy());

        assert!(a.heard(&c, &desk, rt.handle(), Instant::now(), "shark what is on the air"));
        assert_eq!(a.state, State::Asking);
        assert!(a.state.busy(), "a second question must not be taken while one is running");
        assert_eq!(a.state.label(), "asking the model");

        // The task reaches the speech half, and the next poll says so.
        a.stage.speaking();
        assert_eq!(a.poll(&c, Instant::now(), false), None);
        assert_eq!(a.state, State::Speaking);
        assert_eq!(a.state.label(), "making speech");

        // And every state has a word of its own: a label shared between two
        // of them is the readout this replaced.
        let all = [State::Listening, State::Asking, State::Speaking, State::Holding, State::OnAir];
        let mut words: Vec<&str> = all.iter().map(|s| s.label()).collect();
        words.sort_unstable();
        words.dedup();
        assert_eq!(words.len(), all.len());
    }

    /// A server that answers one request with a fixed body, and says what it
    /// was asked.
    fn stub(body: Vec<u8>, kind: &'static str) -> (String, std::sync::mpsc::Receiver<String>) {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("a port");
        let url = format!("http://{}/v1", listener.local_addr().expect("an address"));
        let (seen, asked) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            use std::io::{Read, Write};
            let Some(Ok(mut s)) = listener.incoming().next() else { return };
            let mut head = Vec::new();
            let mut byte = [0u8; 1];
            while !head.ends_with(b"\r\n\r\n") {
                match s.read(&mut byte) {
                    Ok(1) => head.push(byte[0]),
                    _ => return,
                }
            }
            let text = String::from_utf8_lossy(&head).to_string();
            let len = text
                .lines()
                .find_map(|l| {
                    l.to_ascii_lowercase()
                        .strip_prefix("content-length:")
                        .and_then(|v| v.trim().parse::<usize>().ok())
                })
                .unwrap_or(0);
            let mut got = vec![0u8; len];
            let _ = s.read_exact(&mut got);
            let _ = seen.send(String::from_utf8_lossy(&got).to_string());
            let _ = s.write_all(
                format!(
                    "HTTP/1.1 200 OK\r\nContent-Type: {kind}\r\nContent-Length: {}\r\n\r\n",
                    body.len()
                )
                .as_bytes(),
            );
            let _ = s.write_all(&body);
            let _ = s.flush();
            // Held open until the client has read it all.
            std::thread::sleep(Duration::from_millis(200));
        });
        (url, asked)
    }

    /// A WAV of `n` samples at the queue's own rate.
    fn wav(n: usize) -> Vec<u8> {
        let data: Vec<u8> = (0..n).flat_map(|_| 1_000i16.to_le_bytes()).collect();
        let mut v = Vec::new();
        v.extend(b"RIFF");
        v.extend(((36 + data.len()) as u32).to_le_bytes());
        v.extend(b"WAVEfmt ");
        v.extend(16u32.to_le_bytes());
        v.extend(1u16.to_le_bytes());
        v.extend(1u16.to_le_bytes());
        v.extend((VOICE_RATE as u32).to_le_bytes());
        v.extend((VOICE_RATE as u32 * 2).to_le_bytes());
        v.extend(2u16.to_le_bytes());
        v.extend(16u16.to_le_bytes());
        v.extend(b"data");
        v.extend((data.len() as u32).to_le_bytes());
        v.extend(data);
        v
    }

    /// The whole way round: an over addressed to the agent becomes a
    /// question to the model, an answer, speech in the queue, a key when the
    /// channel is quiet, and the key let go when the queue runs out.
    #[test]
    fn an_over_becomes_an_answer_on_the_air() {
        let said = "four four six point zero five zero";
        let reply = format!(
            "data: {}\n\ndata: [DONE]\n\n",
            serde_json::json!({ "choices": [{ "delta": { "content": said } }] })
        );
        let (chat_url, chat_seen) = stub(reply.into_bytes(), "text/event-stream");
        // A quarter of a second of speech.
        let (voice_url, voice_seen) = stub(wav(VOICE_RATE as usize / 4), "audio/wav");
        let c = Config {
            url: chat_url,
            model: "stub".into(),
            speech: crate::agent::config::Speech::Server,
            voice_url,
            voice_model: "stub-tts".into(),
            wake: "shark".into(),
            hang_s: 0.5,
            ..Config::default()
        };
        let rt = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .expect("a runtime");
        let (desk, _asks) = Desk::new();
        let mut a = AgentChannel { on: Some(4), ..Default::default() };
        let t0 = Instant::now();
        assert!(a.heard(&c, &desk, rt.handle(), t0, "shark, what frequency are we on"));

        // The channel is busy while the other station finishes its over, so
        // nothing keys however quickly the answer comes back.
        let mut now = t0;
        let mut keyed = None;
        for _ in 0..200 {
            if let Some(m) = a.poll(&c, now, true) {
                panic!("keyed while the channel was busy: {m:?}");
            }
            if a.state == State::Holding {
                break;
            }
            std::thread::sleep(Duration::from_millis(20));
            now += Duration::from_millis(20);
        }
        assert_eq!(
            a.state,
            State::Holding,
            "the answer never arrived: {:?}",
            a.log.iter().map(|x| x.said.clone()).collect::<Vec<_>>()
        );
        assert_eq!(a.log.len(), 1);
        assert_eq!(a.log[0].heard, "what frequency are we on");
        assert!(a.log[0].said.is_ok(), "{:?}", a.log[0].said);
        assert!(a.speaker.waiting() > 0, "there is speech queued");

        // The squelch shuts. The hang has to run before it keys.
        assert_eq!(a.poll(&c, now, false), None);
        now += Duration::from_millis(600);
        keyed = a.poll(&c, now, false).or(keyed);
        assert_eq!(keyed, Some(Move::Key(4)));

        // The chain drains the queue, and the over ends when it is empty.
        let mut out = Vec::new();
        audio::AudioSource::take(a.speaker.as_ref(), &mut out, VOICE_RATE as usize);
        assert_eq!(out.len(), VOICE_RATE as usize / 4, "a quarter of a second was queued");
        assert_eq!(a.poll(&c, now, false), Some(Move::Unkey));

        // The model was asked the question with the wake word taken off, and
        // the speech server was asked to say what the model answered.
        let asked = chat_seen.recv_timeout(Duration::from_secs(2)).expect("the model was asked");
        assert!(asked.contains("what frequency are we on"), "{asked}");
        assert!(asked.contains("radio channel"), "the voice brief is sent: {asked}");
        let spoken = voice_seen.recv_timeout(Duration::from_secs(2)).expect("speech was asked for");
        assert!(spoken.contains(said), "{spoken}");
    }

    /// With nowhere to send the speech, it stays off the air rather than
    /// keying up with nothing to say.
    #[test]
    fn without_a_speech_server_it_never_keys() {
        let c = Config {
            model: "m".into(),
            wake: "shark".into(),
            speech: crate::agent::config::Speech::Server,
            ..Config::default()
        };
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        assert!(!a.heard(&c, &desk, rt.handle(), Instant::now(), "shark hello"));
        assert_eq!(a.state, State::Listening);
    }
}
