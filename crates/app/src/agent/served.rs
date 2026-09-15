//! What a server offers: its models, and the voices each speech model
//! takes.
//!
//! Asked of the server rather than typed, because a model id is a string
//! only the server can check and a voice name is one only that model
//! knows: `alloy` is OpenAI's, `af_sky` is Kokoro's, and a name the model
//! does not have is a 400 with a Zod trace in it. `GET /models` is the one
//! listing every OpenAI-compatible server has, and OpenRouter's carries the
//! voices on it.
//!
//! One fetch per server address, on a thread of its own with a runtime of
//! its own, the way the release check is done: it is one request, it
//! happens when a dialog opens, and the interface's runtime is for the
//! conversation.

use parking_lot::RwLock;
use serde_json::Value;
use std::collections::HashMap;
use std::sync::OnceLock;

/// One model the server lists.
#[derive(Clone, Debug, PartialEq)]
pub struct Model {
    pub id: String,
    /// Whether it makes speech, and so belongs in the voice picker rather
    /// than the chat's.
    pub speech: bool,
    /// The voices it takes, where the server says. Empty means it did not.
    pub voices: Vec<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum State {
    Fetching,
    Ready(Vec<Model>),
    Failed(String),
}

/// What is known about one server.
#[derive(Clone, Debug, PartialEq)]
pub struct Served {
    pub state: State,
}

impl Served {
    pub fn models(&self) -> &[Model] {
        match &self.state {
            State::Ready(m) => m,
            _ => &[],
        }
    }

    /// The ids of what answers a chat.
    pub fn chat_models(&self) -> Vec<String> {
        self.models().iter().filter(|m| !m.speech).map(|m| m.id.clone()).collect()
    }

    /// The ids of what makes speech.
    pub fn speech_models(&self) -> Vec<String> {
        self.models().iter().filter(|m| m.speech).map(|m| m.id.clone()).collect()
    }

    /// What could read speech. No server marks a transcription model as
    /// such, so this is everything it lists that is not a voice: the name is
    /// typed anyway where the list is wrong.
    pub fn reading_models(&self) -> Vec<String> {
        self.models().iter().filter(|m| !m.speech).map(|m| m.id.clone()).collect()
    }

    /// The voices one model takes.
    pub fn voices_of(&self, model: &str) -> Vec<String> {
        self.models()
            .iter()
            .find(|m| m.id == model.trim())
            .map(|m| m.voices.clone())
            .unwrap_or_default()
    }
}

fn held() -> &'static RwLock<HashMap<String, Served>> {
    static H: OnceLock<RwLock<HashMap<String, Served>>> = OnceLock::new();
    H.get_or_init(Default::default)
}

/// The base address as the map keys it: without the trailing slash and
/// the whitespace a pasted URL comes with.
fn key_of(url: &str) -> String {
    url.trim().trim_end_matches('/').to_string()
}

/// What is known about a server, or nothing if it has never been asked.
pub fn served(url: &str) -> Option<Served> {
    held().read().get(&key_of(url)).cloned()
}

/// Ask a server what it has, unless it is being asked already or has
/// answered. `again` asks whether or not it has.
pub fn fetch(url: &str, key: &str, again: bool) {
    let base = key_of(url);
    if base.is_empty() {
        return;
    }
    {
        let mut h = held().write();
        match h.get(&base) {
            Some(Served { state: State::Fetching }) => return,
            Some(_) if !again => return,
            _ => {}
        }
        h.insert(base.clone(), Served { state: State::Fetching });
    }
    let key = key.trim().to_string();
    let spawned = std::thread::Builder::new().name("models".into()).spawn(move || {
        let outcome = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|e| e.to_string())
            .and_then(|rt| rt.block_on(ask(&base, &key)));
        let state = match outcome {
            Ok(models) => State::Ready(models),
            Err(e) => State::Failed(e),
        };
        held().write().insert(base, Served { state });
    });
    if spawned.is_err() {
        held().write().remove(&key_of(url));
    }
}

/// OpenRouter's own listing of what speaks, which carries the voices. A
/// proxy in front of it (routstr, for one) lists ids alone, under an
/// `openrouter/` prefix, so the voices for those are looked up here.
const OPENROUTER_SPEECH: &str = "https://openrouter.ai/api/v1/models?output_modalities=speech";

/// The prefix such a proxy puts on what it relays.
const RELAYED: &str = "openrouter/";

async fn ask(base: &str, key: &str) -> Result<Vec<Model>, String> {
    let client = httpc::client(std::time::Duration::from_secs(15)).map_err(|e| e.to_string())?;
    let mut req = client.get(format!("{base}/models"));
    if !key.is_empty() {
        req = req.bearer_auth(key);
    }
    let resp = req.send().await.map_err(|e| e.to_string())?;
    let status = resp.status();
    let text = resp.text().await.map_err(|e| e.to_string())?;
    if !status.is_success() {
        return Err(format!("{status}: {}", text.trim()));
    }
    let v: Value = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let mut models = parse(&v);
    // Relayed models with no voices of their own: ask upstream. Not a
    // fault if upstream cannot be reached; the ids still work typed.
    let relayed = models.iter().any(|m| m.id.starts_with(RELAYED) && m.voices.is_empty());
    if relayed
        && let Ok(resp) = client.get(OPENROUTER_SPEECH).send().await
        && let Ok(text) = resp.text().await
        && let Ok(v) = serde_json::from_str::<Value>(&text)
    {
        fill_relayed(&mut models, &parse(&v));
    }
    Ok(models)
}

/// Give every `openrouter/<id>` in `models` what OpenRouter says of `<id>`.
fn fill_relayed(models: &mut [Model], upstream: &[Model]) {
    for m in models.iter_mut() {
        let Some(id) = m.id.strip_prefix(RELAYED) else { continue };
        if let Some(u) = upstream.iter().find(|u| u.id == id) {
            m.speech = true;
            if m.voices.is_empty() {
                m.voices = u.voices.clone();
            }
        }
    }
}

/// The models in a `/models` answer, as OpenAI, OpenRouter, Ollama and
/// llama.cpp each write one.
pub fn parse(v: &Value) -> Vec<Model> {
    let list = v.get("data").or_else(|| v.get("models")).and_then(Value::as_array);
    let Some(list) = list else {
        return Vec::new();
    };
    let mut out: Vec<Model> = list.iter().filter_map(model_of).collect();
    out.sort_by(|a, b| a.id.cmp(&b.id));
    out.dedup_by(|a, b| a.id == b.id);
    out
}

fn model_of(m: &Value) -> Option<Model> {
    let id = m.get("id").or_else(|| m.get("name"))?.as_str()?.to_string();
    let strings = |v: Option<&Value>| -> Vec<String> {
        v.and_then(Value::as_array)
            .map(|a| a.iter().filter_map(Value::as_str).map(str::to_string).collect())
            .unwrap_or_default()
    };
    // OpenRouter says what a model puts out, top level or under its
    // architecture. A server that says nothing is read off the name: a
    // speech model is called one.
    let outputs = {
        let top = strings(m.get("output_modalities"));
        if top.is_empty() {
            strings(m.get("architecture").and_then(|a| a.get("output_modalities")))
        } else {
            top
        }
    };
    let speech = if outputs.is_empty() {
        let l = id.to_ascii_lowercase();
        l.contains("tts") || l.contains("speech") || l.contains("kokoro")
    } else {
        outputs.iter().any(|o| o == "speech" || o == "audio")
            && !outputs.iter().any(|o| o == "text")
    };
    let voices = strings(m.get("supported_voices").or_else(|| m.get("voices")));
    Some(Model { id, speech, voices })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn openrouter_says_which_models_speak_and_with_which_voices() {
        let v = json!({"data": [
            {"id": "hexgrad/kokoro-82m", "architecture": {"output_modalities": ["audio"]},
             "supported_voices": ["af_sky", "af_bella"]},
            {"id": "openai/gpt-4o-mini", "architecture": {"output_modalities": ["text"]}},
            {"id": "openai/gpt-4o-mini-tts", "output_modalities": ["speech"],
             "supported_voices": ["alloy"]},
        ]});
        let s = Served { state: State::Ready(parse(&v)) };
        assert_eq!(s.chat_models(), vec!["openai/gpt-4o-mini"]);
        assert_eq!(s.speech_models(), vec!["hexgrad/kokoro-82m", "openai/gpt-4o-mini-tts"]);
        assert_eq!(s.voices_of("hexgrad/kokoro-82m"), vec!["af_sky", "af_bella"]);
        assert!(s.voices_of("openai/gpt-4o-mini").is_empty());
    }

    #[test]
    fn a_server_that_says_nothing_about_modality_is_read_off_the_name() {
        // OpenAI and Ollama list ids and nothing else.
        let v = json!({"data": [{"id": "tts-1"}, {"id": "gpt-4o"}, {"id": "qwen3:8b"}]});
        let s = Served { state: State::Ready(parse(&v)) };
        assert_eq!(s.chat_models(), vec!["gpt-4o", "qwen3:8b"]);
        assert_eq!(s.speech_models(), vec!["tts-1"]);
        // Ollama's own listing, for a server pointed at it directly.
        let v = json!({"models": [{"name": "llama3:8b"}]});
        assert_eq!(parse(&v)[0].id, "llama3:8b");
        assert!(parse(&json!({})).is_empty());
    }

    #[test]
    fn a_proxy_that_relays_openrouter_gets_its_voices_from_upstream() {
        // routstr lists `openrouter/hexgrad/kokoro-82m` with no voices and
        // no modality; OpenRouter's own listing has both.
        let mut mine = parse(&json!({"data": [
            {"id": "openrouter/hexgrad/kokoro-82m"},
            {"id": "openrouter/openai/gpt-4o"},
            {"id": "agent"},
        ]}));
        let upstream = parse(&json!({"data": [
            {"id": "hexgrad/kokoro-82m", "architecture": {"output_modalities": ["speech"]},
             "supported_voices": ["af_sky"]},
        ]}));
        fill_relayed(&mut mine, &upstream);
        let s = Served { state: State::Ready(mine) };
        assert_eq!(s.speech_models(), vec!["openrouter/hexgrad/kokoro-82m"]);
        assert_eq!(s.voices_of("openrouter/hexgrad/kokoro-82m"), vec!["af_sky"]);
        assert_eq!(s.chat_models(), vec!["agent", "openrouter/openai/gpt-4o"]);
    }

    #[test]
    fn a_server_is_keyed_however_its_address_was_typed() {
        assert_eq!(key_of(" https://h/v1/ "), "https://h/v1");
        assert!(served("http://never-asked/v1").is_none());
    }
}
