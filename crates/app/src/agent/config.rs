//! Which model the Agent view talks to, and where it lives.
//!
//! Its own file rather than a corner of the session, for the reason the
//! scanner table gives: the session is rewritten every couple of seconds and
//! this one holds a key somebody pasted in. `key = value` lines, unknown keys
//! ignored, missing ones defaulted, so a file written by a later version
//! still loads in an earlier one.

use std::path::PathBuf;

/// Where an OpenAI-compatible server is by convention, if nobody says.
///
/// Any server speaking `/v1/chat/completions` with tool calls will do:
/// OpenAI itself, OpenRouter, llama.cpp, vLLM, Ollama's compatibility
/// endpoint. Local first, because a receiver with a model on the same machine
/// needs no account and sends nothing anywhere.
pub const DEFAULT_URL: &str = "http://127.0.0.1:11434/v1";

/// How many tool calls one question may take before the loop gives up.
///
/// A model that has been asked what is on the air will tune, look, open a
/// channel, look again. Twenty is more than any answer has needed and few
/// enough that a model in a loop stops rather than driving the radio all
/// night.
pub const DEFAULT_STEPS: usize = 20;

/// How long the channel must be quiet before the agent keys, in seconds.
///
/// Long enough that a station drawing breath mid-over is not transmitted
/// over, short enough that the answer still belongs to the question. The
/// squelch closing is the other half of this: the wait starts from there.
pub const DEFAULT_HANG_S: f64 = 1.5;

/// How long after it has spoken the agent goes on answering without being
/// named, in seconds.
///
/// Saying the name on every over is how a conversation opens, not how it is
/// held. Once it has answered, the next over on the channel is more of the
/// same conversation, and the window runs again from the end of each of its
/// own transmissions. Zero means the name is wanted every time, which is
/// what to set on a channel other people are using.
pub const DEFAULT_FOLLOW_S: f64 = 30.0;

/// Where the agent's voice is made.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Speech {
    /// A model on this machine, the way speech is read here. Nothing to run
    /// and nothing sent anywhere, at the cost of the weights and a card.
    #[default]
    Local,
    /// The server the chat talks to, at its `/audio/speech`: one address
    /// and one key for both, which is what OpenAI and the hosted services
    /// that copy it offer.
    Chat,
    /// A second OpenAI-compatible `/v1/audio/speech`, with an address and a
    /// key of its own: a local chat server rarely has one.
    Server,
}

impl Speech {
    pub const ALL: [Speech; 3] = [Speech::Local, Speech::Chat, Speech::Server];

    pub fn id(self) -> &'static str {
        match self {
            Self::Local => "local",
            Self::Chat => "chat",
            Self::Server => "server",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Local => "a model here",
            Self::Chat => "the model's server",
            Self::Server => "a speech server",
        }
    }

    /// Whether the voice is fetched over HTTP rather than made here.
    pub fn is_remote(self) -> bool {
        !matches!(self, Self::Local)
    }

    /// Anything unrecognised is the local model, which is the default and
    /// the one that needs nothing else running.
    pub fn parse(text: &str) -> Self {
        match text.trim().to_lowercase().as_str() {
            "chat" | "same" | "model" => Self::Chat,
            "server" | "remote" | "openai" => Self::Server,
            _ => Self::Local,
        }
    }
}

/// Where speech heard on the air is read back into words.
///
/// The same three places the voice comes from, and for the same reasons: a
/// model on this machine needs nothing running and wants a card, and a
/// machine without one can hand the audio to whatever is already answering
/// the chat.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Reading {
    /// The local model in `crates/stt`, chosen and fetched on the Transcript
    /// pane. What the receiver has always done.
    #[default]
    Local,
    /// The chat's own server, at its `/audio/transcriptions`.
    Chat,
    /// A transcription server with an address and a key of its own.
    Server,
}

impl Reading {
    pub const ALL: [Reading; 3] = [Reading::Local, Reading::Chat, Reading::Server];

    pub fn id(self) -> &'static str {
        match self {
            Self::Local => "local",
            Self::Chat => "chat",
            Self::Server => "server",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Local => "a model here",
            Self::Chat => "the model's server",
            Self::Server => "a reading server",
        }
    }

    /// Whether the audio goes out over HTTP rather than into a model here.
    pub fn is_remote(self) -> bool {
        !matches!(self, Self::Local)
    }

    pub fn parse(text: &str) -> Self {
        match text.trim().to_lowercase().as_str() {
            "chat" | "same" | "model" => Self::Chat,
            "server" | "remote" | "openai" => Self::Server,
            _ => Self::Local,
        }
    }
}

/// Where the transcriber sends audio, or nothing for the model here.
///
/// A snapshot rather than a parameter on the stage: `derived_patch` draws the
/// transcriber from the plan, which is the radio's, and a key has no business
/// in the graph or in the edits file beside it. Published by whoever holds the
/// settings, read by the worker thread when it starts.
pub fn reading_server() -> Option<(String, String, String)> {
    published().lock().clone()
}

/// Say where speech is read, for the transcriber to pick up on its next read.
pub fn publish_reading(config: &Config) {
    let want = match config.reading_endpoint() {
        Some((url, key)) if !config.read_model.trim().is_empty() => {
            Some((url, config.read_model.trim().to_string(), key.to_string()))
        }
        _ => None,
    };
    *published().lock() = want;
}

fn published() -> &'static parking_lot::Mutex<Option<(String, String, String)>> {
    static P: std::sync::OnceLock<parking_lot::Mutex<Option<(String, String, String)>>> =
        std::sync::OnceLock::new();
    P.get_or_init(Default::default)
}

#[derive(Clone, Debug, PartialEq)]
pub struct Config {
    /// The base, without `/chat/completions`.
    pub url: String,
    pub model: String,
    /// Sent as a bearer token, or empty for a server that wants none.
    pub key: String,
    /// Tool calls one question may take.
    pub steps: usize,
    /// Anything to add to what the model is told the receiver is.
    pub brief: String,

    /// Where speech comes from at all.
    pub speech: Speech,

    /// The model on this machine: which one, where the weights are kept, and
    /// the sentence that describes how it should sound. Empty means the
    /// shipped defaults.
    ///
    /// `voice_repo` is a catalogue id (`tts::MODELS`) or, for a model that
    /// shipped after this build, any repository name: an id that is not in
    /// the list is read as one.
    pub voice_repo: String,
    pub voice_dir: String,
    pub voice_description: String,
    /// Where it runs: `auto`, `cpu`, `cuda:0`, `metal`. Auto is the fastest
    /// that will take it, falling back to the CPU and saying so.
    pub voice_device: String,
    /// `full` or `half`. Half reads half the bytes per frame, which is the
    /// whole of the speed on an autoregressive decoder making one frame at a
    /// time.
    pub voice_precision: String,

    /// A speech server of its own, for [`Speech::Server`]. Under
    /// [`Speech::Chat`] the chat's `url` and `key` serve instead.
    pub voice_url: String,
    pub voice_model: String,
    /// The voice that server names, as it names it.
    pub voice: String,
    /// A key for the speech server, or empty to use the chat's.
    pub voice_key: String,
    /// What the agent answers to on the air. An over that does not start
    /// with this is heard and ignored.
    pub wake: String,
    /// How long after the channel goes quiet before it keys, in seconds.
    pub hang_s: f64,
    /// How long after its own over it answers without being named, in
    /// seconds. Zero for the name on every over.
    pub follow_s: f64,

    /// Where speech off the air is read back into words.
    pub reading: Reading,
    /// A transcription server of its own, for [`Reading::Server`]. Under
    /// [`Reading::Chat`] the chat's `url` and `key` serve instead.
    pub read_url: String,
    /// What to ask that server for: `whisper-1` on OpenAI.
    pub read_model: String,
    /// A key for the transcription server, or empty to use the chat's.
    pub read_key: String,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            url: DEFAULT_URL.into(),
            model: String::new(),
            key: String::new(),
            steps: DEFAULT_STEPS,
            brief: String::new(),
            speech: Speech::Local,
            voice_repo: String::new(),
            voice_dir: String::new(),
            voice_description: String::new(),
            voice_device: String::new(),
            voice_precision: String::new(),
            voice_url: String::new(),
            voice_model: "tts-1".into(),
            voice: "alloy".into(),
            voice_key: String::new(),
            wake: String::new(),
            hang_s: DEFAULT_HANG_S,
            follow_s: DEFAULT_FOLLOW_S,
            reading: Reading::Local,
            read_url: String::new(),
            read_model: String::new(),
            read_key: String::new(),
        }
    }
}

impl Config {
    /// `$XDG_CONFIG_HOME/waveshark/agent`, beside the session.
    pub fn path() -> Option<PathBuf> {
        let base = std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
        Some(base.join("waveshark").join("agent"))
    }

    pub fn load() -> Self {
        let text = Self::path().and_then(|p| std::fs::read_to_string(p).ok());
        text.map(|t| Self::parse(&t)).unwrap_or_default()
    }

    pub fn save(&self) -> std::io::Result<()> {
        let Some(path) = Self::path() else {
            return Err(std::io::Error::other("no config directory"));
        };
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(path, self.render())
    }

    pub fn parse(text: &str) -> Self {
        let mut c = Self::default();
        for line in text.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let Some((key, value)) = line.split_once('=') else { continue };
            let value = value.trim();
            match key.trim() {
                "url" => c.url = value.to_string(),
                "model" => c.model = value.to_string(),
                "key" => c.key = value.to_string(),
                "steps" => c.steps = value.parse().unwrap_or(DEFAULT_STEPS),
                // Written on one line with the breaks escaped, since the file
                // is one setting per line and a brief is a paragraph.
                "brief" => c.brief = value.replace("\\n", "\n"),
                "speech" => c.speech = Speech::parse(value),
                "voice_repo" => c.voice_repo = value.to_string(),
                "voice_device" => c.voice_device = value.to_string(),
                "voice_precision" => c.voice_precision = value.to_string(),
                "voice_dir" => c.voice_dir = value.to_string(),
                "voice_description" => c.voice_description = value.replace("\\n", "\n"),
                "voice_url" => c.voice_url = value.to_string(),
                "voice_model" => c.voice_model = value.to_string(),
                "voice" => c.voice = value.to_string(),
                "voice_key" => c.voice_key = value.to_string(),
                "wake" => c.wake = value.to_string(),
                "hang_s" => c.hang_s = value.parse().unwrap_or(DEFAULT_HANG_S),
                "follow_s" => c.follow_s = value.parse().unwrap_or(DEFAULT_FOLLOW_S),
                "reading" => c.reading = Reading::parse(value),
                "read_url" => c.read_url = value.to_string(),
                "read_model" => c.read_model = value.to_string(),
                "read_key" => c.read_key = value.to_string(),
                _ => {}
            }
        }
        c
    }

    pub fn render(&self) -> String {
        format!(
            "# The model the Agent view talks to. Any server speaking the OpenAI\n\
             # chat completions API with tool calls.\n\
             url = {}\n\
             model = {}\n\
             key = {}\n\
             steps = {}\n\
             brief = {}\n\
             \n\
             # Talking to the agent over the air. A channel is made the \
             agent's on\n\
             # the strip; it answers an over that starts with the wake word \
             and\n\
             # nothing else. speech is local, chat or server: chat asks the model's\n\
             # own server, at url with key, for voice_model (tts-1 on OpenAI,\n\
             # openrouter/hexgrad/kokoro-82m on OpenRouter) in the voice named.\n\
             speech = {}\n\
             voice_repo = {}\n\
             voice_dir = {}\n\
             voice_device = {}\n\
             voice_precision = {}\n\
             voice_description = {}\n\
             voice_url = {}\n\
             voice_model = {}\n\
             voice = {}\n\
             voice_key = {}\n\
             wake = {}\n\
             hang_s = {}\n\
             follow_s = {}\n\
             \n\
             # Reading what is heard on the air. local is the model on the \
             Transcript\n\
             # pane; chat asks the model's own server and server one of its own, \
             both\n\
             # at /audio/transcriptions for read_model (whisper-1 on OpenAI).\n\
             reading = {}\n\
             read_url = {}\n\
             read_model = {}\n\
             read_key = {}\n",
            self.url,
            self.model,
            self.key,
            self.steps,
            self.brief.replace('\n', "\\n"),
            self.speech.id(),
            self.voice_repo,
            self.voice_dir,
            self.voice_device,
            self.voice_precision,
            self.voice_description.replace('\n', "\\n"),
            self.voice_url,
            self.voice_model,
            self.voice,
            self.voice_key,
            self.wake,
            self.hang_s,
            self.follow_s,
            self.reading.id(),
            self.read_url,
            self.read_model,
            self.read_key
        )
    }

    /// Why the agent cannot answer on the air, or nothing.
    pub fn voice_fault(&self) -> Option<&'static str> {
        if self.fault().is_some() {
            return self.fault();
        }
        if let Some(why) = self.speech_fault() {
            return Some(why);
        }
        if self.wake.trim().is_empty() {
            return Some("no wake word");
        }
        None
    }

    /// Why nothing can be turned into speech, or nothing.
    ///
    /// Apart from the wake word and the chat model: saying a line somebody
    /// handed over needs a voice and nothing else, and refusing it for want
    /// of a name to answer to is refusing it for the wrong reason.
    pub fn speech_fault(&self) -> Option<&'static str> {
        if self.speech == Speech::Server
            && (self.voice_url.trim().is_empty() || self.voice_model.trim().is_empty())
        {
            return Some("no speech server");
        }
        if self.speech == Speech::Chat && self.voice_model.trim().is_empty() {
            return Some("no speech model");
        }
        if self.speech.is_remote() && self.voice.trim().is_empty() {
            return Some("no voice");
        }
        if self.speech == Speech::Local && !cfg!(feature = "tts") {
            return Some("this build has no speech model");
        }
        None
    }

    /// Where the request goes.
    pub fn endpoint(&self) -> String {
        format!("{}/chat/completions", self.url.trim_end_matches('/'))
    }

    /// Where audio is sent to be read and what key to send with it, or
    /// `None` for the model on this machine.
    pub fn reading_endpoint(&self) -> Option<(String, &str)> {
        let (base, key) = match self.reading {
            Reading::Local => return None,
            Reading::Chat => (self.url.trim(), self.key.trim()),
            Reading::Server => (
                self.read_url.trim(),
                match self.read_key.trim() {
                    "" => self.key.trim(),
                    k => k,
                },
            ),
        };
        if base.is_empty() {
            return None;
        }
        Some((format!("{}/audio/transcriptions", base.trim_end_matches('/')), key))
    }

    /// Why speech cannot be read on a server, or nothing. `None` also for
    /// the model here, which the Transcript pane answers for.
    pub fn reading_fault(&self) -> Option<&'static str> {
        if !self.reading.is_remote() {
            return None;
        }
        if self.reading_endpoint().is_none() {
            return Some("no reading server");
        }
        if self.read_model.trim().is_empty() {
            return Some("no reading model");
        }
        None
    }

    /// Where speech is asked for and what key to send, for a voice that
    /// comes over HTTP: the chat's server or the speech server's own.
    /// `None` for the local model.
    pub fn speech_endpoint(&self) -> Option<(String, &str)> {
        let (base, key) = match self.speech {
            Speech::Local => return None,
            Speech::Chat => (self.url.trim(), self.key.trim()),
            Speech::Server => (
                self.voice_url.trim(),
                match self.voice_key.trim() {
                    "" => self.key.trim(),
                    k => k,
                },
            ),
        };
        if base.is_empty() {
            return None;
        }
        Some((format!("{}/audio/speech", base.trim_end_matches('/')), key))
    }

    /// Why the chat cannot run, or nothing.
    pub fn fault(&self) -> Option<&'static str> {
        if self.url.trim().is_empty() {
            return Some("no server address");
        }
        if self.model.trim().is_empty() {
            return Some("no model chosen");
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_config_survives_a_round_trip() {
        let c = Config {
            url: "https://api.example.com/v1".into(),
            model: "some-model".into(),
            key: "sk-secret".into(),
            steps: 7,
            brief: "two\nlines".into(),
            speech: Speech::Server,
            voice_repo: "parler-large-v1".into(),
            voice_dir: "/srv/models/parler".into(),
            voice_device: "cuda:1".into(),
            voice_precision: "half".into(),
            voice_description: "a level voice\nclose to the microphone".into(),
            voice_url: "http://127.0.0.1:8880/v1".into(),
            voice_model: "kokoro".into(),
            voice: "af_sky".into(),
            voice_key: "another".into(),
            wake: "shark".into(),
            hang_s: 2.0,
            follow_s: 12.0,
            reading: Reading::Chat,
            read_url: "http://127.0.0.1:9000/v1".into(),
            read_model: "whisper-1".into(),
            read_key: "third".into(),
        };
        assert_eq!(Config::parse(&c.render()), c);
    }

    #[test]
    fn an_empty_file_is_the_defaults() {
        assert_eq!(Config::parse(""), Config::default());
        assert_eq!(Config::parse("nonsense\n[block]\n"), Config::default());
    }

    #[test]
    fn the_endpoint_takes_one_slash_however_the_url_ends() {
        let mut c = Config { url: "http://h/v1".into(), ..Config::default() };
        assert_eq!(c.endpoint(), "http://h/v1/chat/completions");
        c.url = "http://h/v1/".into();
        assert_eq!(c.endpoint(), "http://h/v1/chat/completions");
    }

    /// Answering on the air needs more than answering on screen does, and
    /// which one is missing is the whole of the message. What it needs
    /// depends on where the voice comes from.
    #[test]
    fn talking_over_the_air_says_what_it_is_missing() {
        // A speech server has to be named. The local model does not: it is
        // fetched the first time it is asked for.
        let mut c = Config { model: "m".into(), speech: Speech::Server, ..Config::default() };
        assert_eq!(c.voice_fault(), Some("no speech server"));
        c.voice_url = "http://s/v1".into();
        assert_eq!(c.voice_fault(), Some("no wake word"));
        c.wake = "shark".into();
        assert_eq!(c.voice_fault(), None);
        c.model.clear();
        assert_eq!(c.voice_fault(), Some("no model chosen"));

        let local = Config {
            model: "m".into(),
            wake: "shark".into(),
            speech: Speech::Local,
            ..Config::default()
        };
        let missing = match cfg!(feature = "tts") {
            true => None,
            false => Some("this build has no speech model"),
        };
        assert_eq!(local.voice_fault(), missing);
    }

    /// Where the voice comes from is parsed once, and anything unreadable is
    /// the local model rather than a receiver that silently stops talking.
    #[test]
    fn where_speech_comes_from_is_read_from_the_file() {
        assert_eq!(Speech::parse("server"), Speech::Server);
        assert_eq!(Speech::parse(" SERVER "), Speech::Server);
        assert_eq!(Speech::parse("local"), Speech::Local);
        assert_eq!(Speech::parse("chat"), Speech::Chat);
        assert_eq!(Speech::parse("nonsense"), Speech::Local);
        assert_eq!(Config::parse("speech = chat").speech, Speech::Chat);
        assert_eq!(Config::parse("speech = server").speech, Speech::Server);
        assert_eq!(Config::default().speech, Speech::Local);
    }

    /// The chat's server can be the voice too: one address and one key,
    /// and only the speech model and voice have to be named.
    #[test]
    fn the_voice_can_come_from_the_chat_server() {
        let mut c = Config {
            url: "https://api.example.com/v1/".into(),
            key: "sk-chat".into(),
            model: "m".into(),
            wake: "shark".into(),
            speech: Speech::Chat,
            voice_url: "http://elsewhere/v1".into(),
            voice_key: "other".into(),
            ..Config::default()
        };
        assert_eq!(
            c.speech_endpoint(),
            Some(("https://api.example.com/v1/audio/speech".into(), "sk-chat")),
            "the chat's address and key, not the speech server's"
        );
        assert_eq!(c.voice_fault(), None);
        c.voice.clear();
        assert_eq!(c.voice_fault(), Some("no voice"), "a server refuses an empty voice");
        c.voice = "af_sky".into();
        c.voice_model.clear();
        assert_eq!(c.voice_fault(), Some("no speech model"));
        // The speech server's own, when that is what is asked for; its key
        // falls back to the chat's.
        c.speech = Speech::Server;
        c.voice_model = "tts-1".into();
        assert_eq!(c.speech_endpoint(), Some(("http://elsewhere/v1/audio/speech".into(), "other")));
        c.voice_key.clear();
        assert_eq!(c.speech_endpoint().map(|(_, k)| k), Some("sk-chat"));
        c.speech = Speech::Local;
        assert_eq!(c.speech_endpoint(), None);
    }

    /// Where speech is read is one setting, in one place, and the address
    /// and key are the chat's or its own.
    #[test]
    fn reading_can_be_handed_to_the_model_s_own_server() {
        let mut c = Config {
            url: "https://api.example.com/v1/".into(),
            key: "sk-chat".into(),
            model: "m".into(),
            reading: Reading::Local,
            ..Config::default()
        };
        assert_eq!(c.reading_endpoint(), None, "the model here is not a server");
        assert_eq!(c.reading_fault(), None, "the Transcript pane answers for the local model");

        c.reading = Reading::Chat;
        assert_eq!(c.reading_fault(), Some("no reading model"));
        c.read_model = "whisper-1".into();
        assert_eq!(
            c.reading_endpoint(),
            Some(("https://api.example.com/v1/audio/transcriptions".into(), "sk-chat"))
        );
        assert_eq!(c.reading_fault(), None);

        // Its own server, with its own key, falling back to the chat's.
        c.reading = Reading::Server;
        assert_eq!(c.reading_fault(), Some("no reading server"));
        c.read_url = "http://127.0.0.1:9000/v1".into();
        c.read_key = "sk-read".into();
        assert_eq!(
            c.reading_endpoint(),
            Some(("http://127.0.0.1:9000/v1/audio/transcriptions".into(), "sk-read"))
        );
        c.read_key.clear();
        assert_eq!(c.reading_endpoint().map(|(_, k)| k), Some("sk-chat"));

        // And what the transcriber picks up is the whole of it, or nothing
        // for the model here.
        publish_reading(&c);
        let (url, model, key) = reading_server().expect("a server to read on");
        assert_eq!(url, "http://127.0.0.1:9000/v1/audio/transcriptions");
        assert_eq!(model, "whisper-1");
        assert_eq!(key, "sk-chat");
        c.reading = Reading::Local;
        publish_reading(&c);
        assert_eq!(reading_server(), None);
    }

    #[test]
    fn a_config_with_no_model_says_so() {
        assert_eq!(Config::default().fault(), Some("no model chosen"));
        let ok = Config { model: "m".into(), ..Config::default() };
        assert_eq!(ok.fault(), None);
    }
}
