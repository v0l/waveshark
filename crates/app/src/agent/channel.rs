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
//! not address it by name, unless it is already in a conversation, and it
//! keys nothing until the squelch has been shut for a hang time: a station
//! drawing breath mid-over must not be transmitted over, and the agent has
//! no ears while it transmits.

use super::{Desk, chat, config::Config, voice};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

/// Longest a single answer may hold the channel, in seconds.
///
/// A model that decides to read out a packet list would otherwise key up for
/// several minutes, which on a shared channel is the worst thing this can do.
/// The text is cut to fit before it is ever spoken; what was spoken is then
/// let run, because a voice that reads slower than the guess used to cut the
/// text must not be cut again mid-sentence.
pub const MAX_OVER_S: f64 = 30.0;

/// How long past the queued speech an over may run before it is let go:
/// a queue nothing is draining must not hold the channel, and this is the
/// margin between "still draining" and "stuck".
const OVERRUN_S: f64 = 5.0;

/// The rate the queue runs at, whatever the speech server produced.
///
/// Fixed rather than taken from the reply: the transmit chain reads the rate
/// once, when it is built, and the radio thread holds the queue from the
/// moment it starts. A queue that changed rate would have to be replaced, and
/// the stage would go on resampling at the old one.
pub const VOICE_RATE: f64 = 24_000.0;

/// Silence keyed before the first word and after the last, in seconds.
///
/// A receiving radio's squelch takes a moment to open once the carrier
/// arrives, and a repeater longer still; speech that starts with the key
/// loses its first syllable at the far end. The tail is for the same gap
/// on the way out, and for a listener's squelch tail not to chop the last
/// word. Measured against a handheld on PMR446: a quarter second in front
/// was still clipping the first word through a repeater, and half a second
/// after was the least that sounded finished.
pub const LEAD_S: f64 = 0.4;
pub const TAIL_S: f64 = 0.5;

/// Speech as it goes to the transmitter: the lead, the words, the tail.
fn keyed(pcm: &[f32]) -> Vec<f32> {
    let lead = (LEAD_S * VOICE_RATE) as usize;
    let tail = (TAIL_S * VOICE_RATE) as usize;
    let mut out = Vec::with_capacity(lead + pcm.len() + tail);
    out.resize(lead, 0.0);
    out.extend_from_slice(pcm);
    out.resize(out.len() + tail, 0.0);
    out
}

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
    /// Who said it, where the radio said: a unit number off an analogue
    /// PTT-ID, or the caller of a decoded call.
    pub from: Option<String>,
    /// Why it was not taken, or `None` when it was.
    pub passed: Option<Passed>,
}

/// Why an over the agent heard was not answered.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Passed {
    /// It does not start with the agent's name.
    NotAddressed,
    /// It is still working on the one before.
    Busy,
    /// It cannot answer at all: see `Config::voice_fault`.
    Mute,
    /// The agent's own transmission, heard back.
    ItsOwn,
}

impl Passed {
    pub fn label(self) -> &'static str {
        match self {
            Self::NotAddressed => "not addressed to it",
            Self::Busy => "still on the last one",
            Self::Mute => "it cannot answer",
            Self::ItsOwn => "its own transmission",
        }
    }
}

/// What the agent heard and what it said, for the pane.
pub struct Exchange {
    pub at: Instant,
    pub heard: String,
    pub said: Result<String, String>,
    /// The speech itself, at the queue's rate, so it can be sent again.
    ///
    /// "Say again" is the commonest thing anybody says on a radio channel,
    /// and the model has nothing to add to it: the words are already decided
    /// and remaking them costs the whole of a generation, which on a CPU is
    /// the best part of a minute. Kept rather than regenerated, and kept as
    /// samples rather than text so the second over is the same over.
    spoke: Option<Arc<Vec<f32>>>,
}

impl Exchange {
    /// Whether there is something to send again.
    pub fn can_repeat(&self) -> bool {
        self.spoke.as_ref().is_some_and(|s| !s.is_empty())
    }
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
    /// When the key went down, so an over that will not end can be ended,
    /// and how long what was queued then should take to go out.
    keyed_at: Option<Instant>,
    keyed_for_s: f64,
    /// When it was last transmitting, from key down to key up.
    ///
    /// A half duplex radio feeds the receiver its own transmission for the
    /// length of the over, so what the agent says is demodulated, transcribed
    /// and offered back to it as something somebody said. Its replies name
    /// it, because a station says who it is, so it answered itself: every
    /// answer became a question and the channel filled with the agent talking
    /// to nobody.
    ///
    /// The end of it is also where the follow window is measured from: an
    /// over soon after the agent has spoken is more of the same conversation.
    spoke: Option<(Instant, Instant)>,
    /// The newest over already dealt with, by the instant it started.
    ///
    /// A mark rather than the last one answered. The interface offers the
    /// whole of the recent transcript on every frame, so with two overs in it
    /// the agent answered one, saw the other was not the one it had just
    /// answered, answered that, and then found the first one new again: two
    /// questions asked once each came back round and round for as long as
    /// they stayed in the window.
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
            keyed_for_s: 0.0,
            spoke: None,
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
        // The half of an over that did go out is still its own voice on the
        // channel, so the window closes here as well as at a clean unkey.
        let keyed = self.keyed_at.take();
        if let Some(from) = keyed {
            self.spoke = Some((from, Instant::now()));
        }
        self.state = State::Listening;
        keyed.map(|_| Move::Unkey)
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

    /// The question as the model is given it, with whoever asked in front of
    /// it.
    ///
    /// A channel carries more than one station, and an answer often depends
    /// on which of them is talking: "say that again" and "tune to my
    /// frequency" are about the station that said them. Where the radio does
    /// not say who it was, the question goes on its own rather than with an
    /// invented caller.
    fn from_station(who: Option<&str>, question: &str) -> String {
        match who {
            Some(unit) if !unit.trim().is_empty() => {
                format!("Station {} says: {question}", unit.trim())
            }
            _ => question.to_string(),
        }
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
             screen. Talk like a radio operator: clipped, plain, one thought per over, the \
             answer first and nothing after it. No greetings, no filler, no restating the \
             question. Keep it to one or two short sentences, no lists, no punctuation a \
             speaker cannot say, and no dashes: write a full stop or a comma instead. Say \
             numbers as words a listener can follow. Say so plainly if you do not know. \
             You are called {}.\n\n\
             What you are given is a speech model's reading of the channel, so it is not \
             always words. Where the radio said who was talking, the over begins \
             \"Station <number> says:\", which is that radio's own identity code and not \
             part of what was said: answer the station, use the number if you need to name \
             them, and never read the prefix back. An over with no number in front of it is \
             from somebody whose radio does not send one, so do not guess who it was or \
             assume it is the station before. Two stations may take turns on the channel, \
             and the numbers are how you tell them apart.\n\n\
             Text in asterisks, brackets or parentheses, such as *BANG*, [MUSIC] or \
             (door slams), is the speech model describing a noise it heard rather than \
             anything anybody said. Treat it as a sound on the channel: worth answering if \
             somebody asks about it, worth mentioning if it matters, and never read aloud as \
             if it were speech. An over that is only such a description is a noise and not a \
             question.",
            config.wake.trim()
        )
    }

    /// Whether the agent is hearing itself: an over that began after the key
    /// went down, while the key is still down.
    ///
    /// Nothing is judged by how long ago the agent spoke. The transcriber is
    /// shut off for the whole over and a moment past it by the radio thread,
    /// which is the only thing that knows when the transmitter is really on
    /// air, so text the agent is offered at all is text somebody said. The
    /// window this used to be, the over plus the hang, threw away an over
    /// that began while the agent was still talking and was read once the
    /// key came up, which is exactly a person answering as soon as they can.
    fn was_speaking(&self, at: Instant) -> bool {
        self.keyed_at.is_some_and(|from| at >= from)
    }

    /// How long it will go on answering without being named, or `None` when
    /// the next over has to say it.
    ///
    /// Measured from the end of its own last over, so every answer starts the
    /// window again and a conversation carries on as long as somebody keeps
    /// talking. Naming the agent works throughout; this is only about not
    /// having to.
    pub fn following(&self, config: &Config, at: Instant) -> Option<f64> {
        if config.follow_s <= 0.0 {
            return None;
        }
        let (_, ended) = self.spoke?;
        let since = at.saturating_duration_since(ended).as_secs_f64();
        (since <= config.follow_s).then_some(config.follow_s - since)
    }

    /// Send an answer again, exactly as it went out the first time.
    ///
    /// The words are already decided, so this neither asks the model nor
    /// makes the speech again: it queues the samples and waits for the
    /// channel the way the first over did. Refused while it is busy, because
    /// two answers queued at once would be transmitted as one.
    pub fn repeat(&mut self, nth: usize) -> bool {
        if self.on.is_none() || self.state.busy() {
            return false;
        }
        let Some(pcm) = self.log.get(nth).and_then(|x| x.spoke.clone()) else {
            return false;
        };
        if pcm.is_empty() {
            return false;
        }
        self.speaker.say(&keyed(&pcm));
        self.state = State::Holding;
        true
    }

    /// Something was said on the channel. Returns whether it was taken.
    ///
    /// `from` is whoever the radio says was talking, when it says: the agent
    /// is told, because on a channel with more than one station on it the
    /// answer to "say that again" depends on who asked.
    pub fn heard(
        &mut self,
        config: &Config,
        desk: &Desk,
        rt: &tokio::runtime::Handle,
        at: Instant,
        from: Option<&str>,
        text: &str,
    ) -> bool {
        if self.on.is_none() {
            return false;
        }
        // Said once per over rather than per frame: the same reading is
        // offered again every frame it stays in the transcript, and a note
        // that rewrote itself sixty times a second is the same note.
        let who = from.map(str::to_string);
        let pass = |a: &mut Self, why: Passed| {
            if a.last.as_ref().map(|h| h.at) != Some(at) {
                a.last = Some(Heard {
                    at,
                    text: text.to_string(),
                    from: who.clone(),
                    passed: Some(why),
                });
            }
            false
        };
        if config.voice_fault().is_some() {
            return pass(self, Passed::Mute);
        }
        // Nothing is taken while the key is down: the samples the transmitter
        // is putting out are the only thing that can be on the channel.
        if self.was_speaking(at) {
            return pass(self, Passed::ItsOwn);
        }
        // Anything at or before the mark has had its turn. The transcript
        // also replaces a partial reading with a settled one of the same
        // over, which arrives under the same instant and is the same over.
        if self.answered.is_some_and(|mark| at <= mark) {
            return false;
        }
        if self.state.busy() {
            return pass(self, Passed::Busy);
        }
        // Named, or already talking to it: within the follow window the whole
        // over is the question, name or no name.
        let question = match Self::addressed(&config.wake, text) {
            Some(q) => q,
            None if self.following(config, at).is_some() => text.trim().to_string(),
            None => return pass(self, Passed::NotAddressed),
        };
        // The name on its own is a station calling, and a station answers a
        // call: saying nothing looked like a receiver that had not heard.
        // The model has nothing to add to "go ahead" and would take seconds
        // to say it, so this one line is not put to it.
        if question.is_empty() {
            return self.answer_the_call(config, rt, at, from, text);
        }
        self.last = Some(Heard { at, text: text.to_string(), from: who.clone(), passed: None });
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
        let question = Self::from_station(who.as_deref(), &question);
        let (config, desk) = (config.clone(), desk.clone());
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

    /// Answer a call that asked nothing: the name, and an invitation to go
    /// on. The follow window opens behind it, so the question itself needs
    /// no name.
    fn answer_the_call(
        &mut self,
        config: &Config,
        rt: &tokio::runtime::Handle,
        at: Instant,
        from: Option<&str>,
        text: &str,
    ) -> bool {
        self.last = Some(Heard {
            at,
            text: text.to_string(),
            from: from.map(str::to_string),
            passed: None,
        });
        self.answered = Some(at);
        let said = format!("{} here, go ahead.", config.wake.trim());
        self.speak_line(config, rt, said, text.trim().to_string());
        true
    }

    /// Say a line nothing on the air asked for: what a `say` tool hands over,
    /// or an operator typing into the pane.
    ///
    /// The words are taken as given and only cut to one over. Everything
    /// after that is the channel's: it waits for the squelch and the hang the
    /// same way an answer does, so a line handed over mid-conversation does
    /// not transmit on top of somebody.
    pub fn say(
        &mut self,
        config: &Config,
        rt: &tokio::runtime::Handle,
        text: &str,
    ) -> Result<String, String> {
        if self.on.is_none() {
            return Err(
                "no channel is the agent's: set a channel's transmit source to AGENT".to_string()
            );
        }
        if let Some(why) = config.speech_fault() {
            return Err(format!("there is no voice to say it with: {why}"));
        }
        if self.state.busy() {
            return Err(format!("it is {} already", self.state.label()));
        }
        let said = Self::to_the_point(text.trim());
        if said.is_empty() {
            return Err("nothing to say".to_string());
        }
        self.speak_line(config, rt, said.clone(), "asked to say this".to_string());
        Ok(said)
    }

    /// Put one line of speech in hand: the words are already decided, so the
    /// model is not asked and the conversation is left as it was.
    fn speak_line(
        &mut self,
        config: &Config,
        rt: &tokio::runtime::Handle,
        said: String,
        heard: String,
    ) {
        self.asked = heard;
        self.stage = Stage::default();
        // Straight to the speech: there is no model half to this one.
        self.stage.speaking();
        self.state = State::Speaking;
        self.stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = crossbeam_channel::unbounded();
        self.pending = Some(rx);
        let config = config.clone();
        rt.spawn(async move {
            // An empty history leaves the conversation as it was: the model
            // was not asked and has not heard this.
            let answer = match voice::speak(&config, &said).await {
                Ok(speech) => Answer { said: Ok(said), history: Vec::new(), speech: Some(speech) },
                Err(e) => Answer { said: Err(e), history: Vec::new(), speech: None },
            };
            let _ = tx.send(answer);
        });
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
            let spoke = match &answer.speech {
                Some(s) if !s.samples.is_empty() => {
                    let pcm = Arc::new(at_voice_rate(s));
                    self.speaker.say(&keyed(&pcm));
                    self.state = State::Holding;
                    Some(pcm)
                }
                _ => {
                    self.state = State::Listening;
                    None
                }
            };
            self.log.push(Exchange {
                at: now,
                heard: std::mem::take(&mut self.asked),
                said: answer.said,
                spoke,
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
                self.keyed_for_s = self.speaker.seconds();
                Some(Move::Key(channel))
            }
            State::OnAir => {
                // Let go when the queue is empty, or when it has held the
                // key well past what it held at key-up: the words were cut
                // to fit before they were spoken, so the limit here is on a
                // queue that is not draining, not on the answer.
                let allowed = Duration::from_secs_f64(self.keyed_for_s + OVERRUN_S);
                let over_ran = self.keyed_at.is_some_and(|t| now.duration_since(t) > allowed);
                if self.speaker.waiting() > 0 && !over_ran {
                    return None;
                }
                self.speaker.cut();
                if let Some(from) = self.keyed_at.take() {
                    self.spoke = Some((from, now));
                }
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
            // Off unless a test asks for it, so every other test is about
            // the name.
            follow_s: 0.0,
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

    /// The model is told who is talking when the radio says.
    ///
    /// A PTT-ID off an analogue channel or a decoded call's caller both
    /// arrive the same way, and on a channel with two stations on it the
    /// answer to "say that again" depends on which of them asked.
    #[test]
    fn the_model_is_told_who_asked() {
        let f = AgentChannel::from_station;
        assert_eq!(f(Some("123"), "what is on the air"), "Station 123 says: what is on the air");
        assert_eq!(f(Some(" 4321 "), "go ahead"), "Station 4321 says: go ahead");
        // An over with nobody named is the ordinary case: PTT-ID is off by
        // default on every radio that has it.
        assert_eq!(f(None, "what is on the air"), "what is on the air");
        assert_eq!(f(Some(""), "what is on the air"), "what is on the air");
    }

    /// The model is told what it is reading: a station's number in front of
    /// an over, and a noise the speech model described rather than heard
    /// somebody say.
    ///
    /// Without the first it read the prefix back on the air as though it were
    /// part of the question. Without the second it answered *BANG* as a word
    /// and apologised for not understanding.
    #[test]
    fn the_brief_explains_the_prefix_and_the_noises() {
        let brief = AgentChannel::brief(&config());
        assert!(brief.contains("called shark"), "{brief}");
        assert!(brief.contains("Station <number> says:"));
        assert!(brief.contains("never read the prefix back"));
        assert!(brief.contains("*BANG*"));
        assert!(brief.contains("describing a noise"));
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
        // Two seconds queued and nothing draining it.
        a.speaker.say(&[0.1; 48_000]);
        let t0 = Instant::now();
        assert_eq!(a.poll(&c, t0, false), Some(Move::Key(1)));
        let still = t0 + Duration::from_secs_f64(2.0 + OVERRUN_S - 1.0);
        assert_eq!(a.poll(&c, still, false), None, "within what it was keyed for");
        let late = t0 + Duration::from_secs_f64(2.0 + OVERRUN_S + 1.0);
        assert_eq!(a.poll(&c, late, false), Some(Move::Unkey));
        assert_eq!(a.speaker.waiting(), 0, "what was left is thrown away, not saved up");
    }

    /// A long answer is let run to its end: the voice reads slower than
    /// the guess the text was cut by, and a thirty second answer cut at
    /// thirty seconds ended mid-sentence.
    #[test]
    fn a_long_answer_is_not_cut_off_while_it_is_still_going_out() {
        let c = config();
        let mut a = AgentChannel { on: Some(1), state: State::Holding, ..Default::default() };
        let forty = (40.0 * VOICE_RATE) as usize;
        a.speaker.say(&vec![0.1; forty]);
        let t0 = Instant::now();
        assert_eq!(a.poll(&c, t0, false), Some(Move::Key(1)));
        // Thirty five seconds in, the chain has taken most of it and is
        // still taking.
        let mut out = Vec::new();
        audio::AudioSource::take(a.speaker.as_ref(), &mut out, (35.0 * VOICE_RATE) as usize);
        let mid = t0 + Duration::from_secs_f64(35.0);
        assert_eq!(a.poll(&c, mid, false), None, "cut off with five seconds still to say");
        audio::AudioSource::take(a.speaker.as_ref(), &mut out, forty);
        assert_eq!(a.poll(&c, t0 + Duration::from_secs_f64(41.0), false), Some(Move::Unkey));
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
        assert!(a.heard(&c, &desk, rt.handle(), at, None, "shark what is on the air"));
        assert_eq!(a.state, State::Asking, "it has the question and is asking the model");
        // The same over read again, and a second question while it is busy.
        assert!(!a.heard(&c, &desk, rt.handle(), at, None, "shark what is on the air"));
        assert!(!a.heard(&c, &desk, rt.handle(), at + Duration::from_secs(1), None, "shark again"));
    }

    /// Two overs in the window are answered once each, not round and round.
    ///
    /// The interface offers the whole of the recent transcript on every
    /// frame. Remembering only the last over answered, the agent took the
    /// first, saw the second was not the one it had just taken, took that,
    /// and then found the first new again: two questions asked once each came
    /// back for as long as they stayed in the window, and the channel filled
    /// with answers to things nobody had said twice.
    #[test]
    fn an_over_is_answered_once_however_often_it_is_offered() {
        let c = config();
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        let t0 = Instant::now();
        let first = t0;
        let second = t0 + Duration::from_secs(20);

        // The first is taken; the agent is busy, so the second waits.
        assert!(a.heard(&c, &desk, rt.handle(), first, None, "shark how is it going"));
        assert!(!a.heard(&c, &desk, rt.handle(), second, None, "shark how is it going"));

        // Free again, and the window offered whole on the next frame. The
        // second is new, the first is not.
        a.state = State::Listening;
        a.pending = None;
        assert!(
            !a.heard(&c, &desk, rt.handle(), first, None, "shark how is it going"),
            "answered twice"
        );
        assert!(a.heard(&c, &desk, rt.handle(), second, None, "shark how is it going"));

        // And round again: neither is new now.
        a.state = State::Listening;
        a.pending = None;
        for _ in 0..3 {
            assert!(!a.heard(&c, &desk, rt.handle(), first, None, "shark how is it going"));
            assert!(!a.heard(&c, &desk, rt.handle(), second, None, "shark how is it going"));
        }
    }

    /// The agent does not answer itself.
    ///
    /// A half duplex radio feeds the receiver its own transmission for the
    /// length of the over, so what the agent says is demodulated, transcribed
    /// and offered back as something somebody said. Its replies name it,
    /// because a station says who it is, so every answer became a question.
    #[test]
    fn what_it_transmitted_is_not_a_question() {
        let c = config();
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), state: State::Holding, ..Default::default() };
        let t0 = Instant::now();

        // On air from t0.
        assert_eq!(a.poll(&c, t0, false), Some(Move::Key(1)));
        assert_eq!(a.state, State::OnAir);
        let mid = t0 + Duration::from_secs(2);
        assert!(
            !a.heard(
                &c,
                &desk,
                rt.handle(),
                mid,
                None,
                "shark here, the receiver is idle and ready"
            ),
            "it answered its own voice"
        );
        assert_eq!(a.last.as_ref().and_then(|h| h.passed), Some(Passed::ItsOwn));

        // The key comes up, and an over read out of the transcript now is
        // somebody's, however far back it began: they started talking while
        // the agent was still going, which is a person coming back as soon
        // as they can. Judged by how long ago the agent spoke, as it was,
        // this over was thrown away.
        let up = t0 + Duration::from_secs(4);
        assert_eq!(a.poll(&c, up, false), Some(Move::Unkey));
        assert!(a.heard(&c, &desk, rt.handle(), mid, None, "shark what about the decode settings"));

        // And promptly after the key, the same.
        a.state = State::Listening;
        a.pending = None;
        let after = up + Duration::from_secs_f64(0.2);
        assert!(a.heard(&c, &desk, rt.handle(), after, None, "shark how is it going"));
    }

    /// Having answered, it goes on answering without being named, until the
    /// window since its own last over runs out.
    #[test]
    fn a_conversation_carries_on_without_the_name() {
        let c = Config { follow_s: 30.0, ..config() };
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        let t0 = Instant::now();

        // Nothing has been said yet, so the name is still wanted.
        assert!(a.following(&c, t0).is_none());
        assert!(!a.heard(&c, &desk, rt.handle(), t0, None, "what is on the air"));
        assert_eq!(a.last.as_ref().and_then(|h| h.passed), Some(Passed::NotAddressed));

        // It says something, and the window opens at the key coming up.
        a.state = State::Holding;
        assert_eq!(a.poll(&c, t0, false), Some(Move::Key(1)));
        let up = t0 + Duration::from_secs(3);
        assert_eq!(a.poll(&c, up, false), Some(Move::Unkey));

        // The next over is the question whole, name or no name.
        let next = up + Duration::from_secs(4);
        assert!(a.heard(&c, &desk, rt.handle(), next, None, "and what about 145.5"));
        assert_eq!(a.asked, "and what about 145.5", "the whole over, with nothing taken off");
        assert!(a.following(&c, next).is_some_and(|left| left > 20.0 && left < 30.0));

        // The window is measured from its own over, so it runs out while
        // nobody is talking to it.
        a.state = State::Listening;
        a.pending = None;
        let late = up + Duration::from_secs_f64(31.0);
        assert!(a.following(&c, late).is_none());
        assert!(!a.heard(&c, &desk, rt.handle(), late, None, "anybody about on this channel"));
        assert_eq!(a.last.as_ref().and_then(|h| h.passed), Some(Passed::NotAddressed));
        // And the name still works after it.
        assert!(a.heard(
            &c,
            &desk,
            rt.handle(),
            late + Duration::from_secs(1),
            None,
            "shark you there"
        ));

        // With the window off, only the name does.
        let named = Config { follow_s: 0.0, ..c.clone() };
        let mut strict = AgentChannel { on: Some(1), ..Default::default() };
        strict.spoke = Some((t0, up));
        assert!(strict.following(&named, up + Duration::from_secs(1)).is_none());
        assert!(!strict.heard(
            &named,
            &desk,
            rt.handle(),
            up + Duration::from_secs(1),
            None,
            "go on"
        ));
    }

    /// The name on its own is a call, and a call is answered.
    ///
    /// It used to be read as an over with no question in it and passed over
    /// in silence, which on the air is a receiver that did not hear: "hey
    /// shark" is how anybody opens, and the question comes in the next over.
    #[test]
    fn the_name_on_its_own_is_answered() {
        let c = config();
        let (desk, _asks) = Desk::new();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        let at = Instant::now();

        assert!(a.heard(&c, &desk, rt.handle(), at, None, "hey shark"));
        assert_eq!(a.last.as_ref().and_then(|h| h.passed), None, "it was taken");
        // Straight to the speech: the model is not asked to say go ahead.
        assert_eq!(a.state, State::Speaking);
        assert_eq!(a.asked, "hey shark");
        assert!(a.history.is_empty(), "the model has not been told about it");
    }

    /// An answer can be sent again without asking for another.
    ///
    /// "Say again" is the commonest thing anybody says on a channel, and the
    /// words are already decided: remaking them costs a whole generation,
    /// which on a CPU is the best part of a minute, and would come back
    /// worded differently. The same samples go out again.
    #[test]
    fn an_answer_can_be_sent_again() {
        let mut a = AgentChannel { on: Some(1), ..Default::default() };
        // As `poll` files one: an answer that went out, and one that did not.
        a.log.push(Exchange {
            at: Instant::now(),
            heard: "shark what is on the air".into(),
            said: Ok("nothing is on the air".into()),
            spoke: Some(Arc::new(vec![0.2f32; 4_800])),
        });
        a.log.push(Exchange {
            at: Instant::now(),
            heard: "shark again".into(),
            said: Err("the model said nothing".into()),
            spoke: None,
        });
        assert!(a.log[0].can_repeat());
        assert!(!a.log[1].can_repeat(), "nothing went out, so there is nothing to repeat");

        assert!(a.repeat(0), "the first answer is there to send again");
        assert_eq!(a.state, State::Holding, "it waits for the channel, as the first over did");
        assert!(!a.repeat(1), "an answer that never went out");
        assert!(!a.repeat(9), "an exchange that is not there");

        // And not while it is working on something else: two answers queued
        // at once go out as one over.
        assert!(!a.repeat(0), "it is already holding one");
        // Nor on a channel nobody gave it.
        let mut off = AgentChannel::default();
        off.log.push(Exchange {
            at: Instant::now(),
            heard: String::new(),
            said: Ok("hello".into()),
            spoke: Some(Arc::new(vec![0.2f32; 64])),
        });
        assert!(!off.repeat(0));
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
            a.heard(c, &desk, rt.handle(), clock, None, text)
        };

        // Somebody talking on the channel, to somebody else.
        assert!(!say(&mut a, &c, "uh, hey, can you hear that?"));
        let h = a.last.clone().expect("it heard the over");
        assert_eq!(h.text, "uh, hey, can you hear that?");
        assert_eq!(h.passed, Some(Passed::NotAddressed));

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

        assert!(a.heard(&c, &desk, rt.handle(), Instant::now(), None, "shark what is on the air"));
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
        assert!(a.heard(&c, &desk, rt.handle(), t0, None, "shark, what frequency are we on"));

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
        audio::AudioSource::take(a.speaker.as_ref(), &mut out, 2 * VOICE_RATE as usize);
        let (lead, tail) = ((LEAD_S * VOICE_RATE) as usize, (TAIL_S * VOICE_RATE) as usize);
        assert_eq!(
            out.len(),
            lead + VOICE_RATE as usize / 4 + tail,
            "a quarter of a second was queued, keyed with silence either side"
        );
        assert!(out[..lead].iter().all(|s| *s == 0.0), "silence before the first word");
        assert!(out[out.len() - tail..].iter().all(|s| *s == 0.0), "and after the last");
        assert!(out[lead..lead + 100].iter().any(|s| *s != 0.0), "the words are in between");
        assert_eq!(a.poll(&c, now, false), Some(Move::Unkey));

        // The model was asked the question with the wake word taken off, and
        // the speech server was asked to say what the model answered.
        let asked = chat_seen.recv_timeout(Duration::from_secs(2)).expect("the model was asked");
        assert!(asked.contains("what frequency are we on"), "{asked}");
        assert!(asked.contains("radio channel"), "the voice brief is sent: {asked}");
        let spoken = voice_seen.recv_timeout(Duration::from_secs(2)).expect("speech was asked for");
        assert!(spoken.contains(said), "{spoken}");
        assert!(spoken.contains("\"pcm\""), "asked for raw PCM, which every server has: {spoken}");
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
        assert!(!a.heard(&c, &desk, rt.handle(), Instant::now(), None, "shark hello"));
        assert_eq!(a.state, State::Listening);
    }
}
