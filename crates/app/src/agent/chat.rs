//! The conversation in the Agent view: a model with the run of the receiver.
//!
//! The model is anything speaking the OpenAI chat completions API with tool
//! calls, named by [`config::Config`]. What it can do is [`catalog::all`],
//! the same list the MCP server publishes, and every call it makes goes onto
//! the same desk, so a model that tunes moves the dial on screen.
//!
//! Nothing here draws. The work runs on the interface's tokio runtime and
//! reports back over a channel the pane drains, which is what lets the same
//! loop serve a window, a voice channel, or a front end on another machine.

use super::{Desk, catalog, config::Config};
use serde_json::{Value, json};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// How long one completion may take. Long, because a local model on a CPU
/// answers in minutes and the request is what holds the conversation open.
const PATIENCE: std::time::Duration = std::time::Duration::from_secs(600);

/// What the pane draws, in the order it happened.
pub enum Turn {
    /// What the operator asked.
    You(String),
    /// What the model said.
    Said(String),
    /// A tool the model called, and what the receiver answered.
    Did {
        name: String,
        args: String,
        /// `None` while it is running.
        answer: Option<Result<String, String>>,
    },
    /// Why the exchange stopped.
    Fault(String),
}

/// What the worker tells the pane.
enum Update {
    /// More of what the model is saying.
    Delta(String),
    /// The model has stopped saying that.
    Said,
    Calling {
        name: String,
        args: String,
    },
    Answered(Result<String, String>),
    Fault(String),
    Done,
}

pub struct Chat {
    pub config: Config,
    pub turns: Vec<Turn>,
    /// What is being typed.
    pub draft: String,
    /// The conversation as the model sees it, which is not what is drawn: it
    /// carries the tool call ids the protocol needs and the JSON the receiver
    /// answered with.
    history: Vec<Value>,
    updates: Option<crossbeam_channel::Receiver<Update>>,
    stop: Arc<AtomicBool>,
    /// Whether the model is saying something right now, so a delta extends
    /// the last turn instead of starting another.
    speaking: bool,
}

impl Default for Chat {
    fn default() -> Self {
        let config = Config::load();
        // The transcriber reads where this says, and it is a stage in a graph
        // that knows nothing about the agent: say so once, here, and again
        // whenever the settings change.
        super::config::publish_reading(&config);
        Self {
            config,
            turns: Vec::new(),
            draft: String::new(),
            history: Vec::new(),
            updates: None,
            stop: Arc::new(AtomicBool::new(false)),
            speaking: false,
        }
    }
}

impl Chat {
    /// Whether the model is working on something.
    pub fn busy(&self) -> bool {
        self.updates.is_some()
    }

    /// How much has been said, which is what the tab's dot counts.
    pub fn turns(&self) -> usize {
        self.turns.len()
    }

    pub fn clear(&mut self) {
        self.turns.clear();
        self.history.clear();
    }

    /// Stop the exchange after the call in flight.
    pub fn interrupt(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }

    /// Ask, and start the loop that answers.
    pub fn ask(&mut self, text: &str, desk: Desk, rt: &tokio::runtime::Handle) {
        let text = text.trim();
        if text.is_empty() || self.busy() {
            return;
        }
        if let Some(fault) = self.config.fault() {
            self.turns.push(Turn::You(text.to_string()));
            self.turns.push(Turn::Fault(format!("{fault}: set one in the Agent settings")));
            return;
        }
        self.turns.push(Turn::You(text.to_string()));
        self.history.push(json!({ "role": "user", "content": text }));
        let (tx, rx) = crossbeam_channel::unbounded();
        self.updates = Some(rx);
        self.speaking = false;
        self.stop = Arc::new(AtomicBool::new(false));
        let job = Worker {
            config: self.config.clone(),
            history: self.history.clone(),
            desk,
            out: tx,
            stop: self.stop.clone(),
            bell: None,
        };
        rt.spawn(async move { job.run().await });
    }

    /// Take what the worker has said since the last frame.
    ///
    /// The history the worker builds is rebuilt here from the same updates
    /// rather than sent back whole, so what is drawn and what the model is
    /// told cannot disagree.
    pub fn poll(&mut self) {
        let Some(rx) = self.updates.as_ref() else { return };
        let mut done = false;
        for u in rx.try_iter().collect::<Vec<_>>() {
            match u {
                Update::Delta(text) => {
                    if self.speaking
                        && let Some(Turn::Said(s)) = self.turns.last_mut()
                    {
                        s.push_str(&text);
                        continue;
                    }
                    // Models open with a blank line or two more often than
                    // not. Drawn, that is an empty gap under AGENT that reads
                    // as a reply which has not arrived yet, and the turn is
                    // not started until there are words in it.
                    let text = text.trim_start();
                    if text.is_empty() {
                        continue;
                    }
                    self.turns.push(Turn::Said(text.to_string()));
                    self.speaking = true;
                }
                Update::Said => {
                    self.speaking = false;
                    // And the same at the end, where a model signs off with
                    // a newline: a turn that ends in blank lines pushes
                    // whatever comes next down the pane for no reason.
                    if let Some(Turn::Said(s)) = self.turns.last_mut() {
                        s.truncate(s.trim_end().len());
                    }
                }
                Update::Calling { name, args } => {
                    self.speaking = false;
                    self.turns.push(Turn::Did { name, args, answer: None });
                }
                Update::Answered(r) => {
                    if let Some(Turn::Did { answer, .. }) = self
                        .turns
                        .iter_mut()
                        .rev()
                        .find(|t| matches!(t, Turn::Did { answer: None, .. }))
                    {
                        *answer = Some(r);
                    }
                }
                Update::Fault(e) => {
                    self.speaking = false;
                    self.turns.push(Turn::Fault(e));
                }
                Update::Done => done = true,
            }
        }
        if done {
            self.updates = None;
            self.speaking = false;
            self.remember();
        }
    }

    /// Fold what was drawn back into what the model is told next time.
    ///
    /// Only the sentences and the calls, not the JSON the receiver answered
    /// with: a packet list from ten minutes ago is stale and expensive, and
    /// the model can ask again. What it must keep is what it did, or it
    /// repeats itself.
    fn remember(&mut self) {
        for t in &self.turns {
            match t {
                Turn::Said(s) => {
                    let m = json!({ "role": "assistant", "content": s });
                    if self.history.last() != Some(&m) {
                        self.history.push(m);
                    }
                }
                Turn::Did { name, args, answer } => {
                    let said = match answer {
                        Some(Ok(_)) => format!("called {name}({args}) and read the answer"),
                        Some(Err(e)) => format!("called {name}({args}): {e}"),
                        None => continue,
                    };
                    let m = json!({ "role": "assistant", "content": said });
                    if self.history.last() != Some(&m) {
                        self.history.push(m);
                    }
                }
                _ => {}
            }
        }
    }
}

/// Ask once, with the tools, and hand back what the model said last and the
/// conversation it leaves behind.
///
/// What the Agent view does over several frames, for a caller that wants the
/// answer as a sentence: the voice channel, which has nowhere to draw a
/// half-finished reply and has to wait for the whole of it before it keys.
pub async fn ask_once(
    config: Config,
    mut history: Vec<Value>,
    desk: Desk,
    question: &str,
) -> Result<(String, Vec<Value>), String> {
    history.push(json!({ "role": "user", "content": question }));
    let (out, updates) = crossbeam_channel::unbounded();
    let mut worker =
        Worker { config, history, desk, out, stop: Arc::new(AtomicBool::new(false)), bell: None };
    worker.exchange().await?;
    let said: String = updates
        .try_iter()
        .filter_map(|u| match u {
            Update::Delta(t) => Some(t),
            _ => None,
        })
        .collect();
    Ok((said, worker.history))
}

/// One exchange, running off the interface thread.
struct Worker {
    config: Config,
    history: Vec<Value>,
    desk: Desk,
    out: crossbeam_channel::Sender<Update>,
    stop: Arc<AtomicBool>,
    bell: Option<super::Bell>,
}

impl Worker {
    fn say(&self, u: Update) {
        let _ = self.out.send(u);
        if let Some(b) = &self.bell {
            b.ring();
        }
    }

    async fn run(mut self) {
        // The pane is redrawn by whatever the desk wakes, so an answer that
        // arrives while nothing else is happening still appears.
        self.bell = Some(self.desk.bell());
        if let Err(e) = self.exchange().await {
            self.say(Update::Fault(e));
        }
        self.say(Update::Done);
    }

    async fn exchange(&mut self) -> Result<(), String> {
        let client = httpc::client(PATIENCE).map_err(|e| e.to_string())?;
        let tools = tool_list();
        for _ in 0..self.config.steps.max(1) {
            if self.stop.load(Ordering::Relaxed) {
                return Err("stopped".into());
            }
            let reply = self.complete(&client, &tools).await?;
            let calls = reply.calls;
            let mut message = json!({ "role": "assistant", "content": reply.text });
            if !calls.is_empty() {
                message["tool_calls"] = Value::Array(
                    calls
                        .iter()
                        .map(|c| {
                            json!({
                                "id": c.id,
                                "type": "function",
                                "function": { "name": c.name, "arguments": c.args },
                            })
                        })
                        .collect(),
                );
            }
            self.history.push(message);
            if calls.is_empty() {
                self.say(Update::Said);
                return Ok(());
            }
            for call in calls {
                let answer = self.run_tool(&call).await;
                let content = match &answer {
                    Ok(v) => v.clone(),
                    Err(e) => json!({ "error": e }).to_string(),
                };
                self.history.push(json!({
                    "role": "tool",
                    "tool_call_id": call.id,
                    "content": content,
                }));
                self.say(Update::Answered(answer));
            }
        }
        Err(format!("stopped after {} tool calls", self.config.steps.max(1)))
    }

    /// Put one call on the desk, and say what came back.
    async fn run_tool(&self, call: &Call) -> Result<String, String> {
        self.say(Update::Calling { name: call.name.clone(), args: call.args.clone() });
        let Some(tool) = catalog::find(&call.name) else {
            return Err(format!("no tool called {}", call.name));
        };
        let args: Value = if call.args.trim().is_empty() {
            Value::Null
        } else {
            serde_json::from_str(&call.args).map_err(|e| format!("bad arguments: {e}"))?
        };
        let action = tool.action(args)?;
        let mut value = self.desk.ask(action).await?;
        // A picture is a megabyte of base64 and the model is being talked to
        // over a text channel; it is already on the operator's screen.
        if let Some(o) = value.as_object_mut()
            && o.remove("png_base64").is_some()
        {
            o.insert("note".into(), json!("the picture is in the window"));
        }
        Ok(value.to_string())
    }

    /// One completion, streamed.
    async fn complete(&self, client: &reqwest::Client, tools: &Value) -> Result<Reply, String> {
        let mut messages = vec![json!({ "role": "system", "content": self.brief() })];
        messages.extend(self.history.iter().cloned());
        let body = json!({
            "model": self.config.model,
            "messages": messages,
            "tools": tools,
            "stream": true,
        });
        let mut req = client.post(self.config.endpoint()).json(&body);
        if !self.config.key.trim().is_empty() {
            req = req.bearer_auth(self.config.key.trim());
        }
        let resp = req.send().await.map_err(|e| e.to_string())?;
        let status = resp.status();
        if !status.is_success() {
            let body = resp.text().await.unwrap_or_default();
            return Err(format!("{status}: {}", body.trim()));
        }
        self.read_stream(resp).await
    }

    /// Server-sent events into a reply.
    async fn read_stream(&self, mut resp: reqwest::Response) -> Result<Reply, String> {
        let mut reply = Reply::default();
        let mut buf = String::new();
        while let Some(chunk) = resp.chunk().await.map_err(|e| e.to_string())? {
            if self.stop.load(Ordering::Relaxed) {
                return Err("stopped".into());
            }
            buf.push_str(&String::from_utf8_lossy(&chunk));
            // An event ends at a blank line, and a chunk can hold part of
            // one, so what is left after the last break stays in the buffer.
            while let Some(end) = buf.find("\n\n").or_else(|| buf.find("\r\n\r\n")) {
                let skip = if buf[end..].starts_with("\r\n") { 4 } else { 2 };
                let event = buf[..end].to_string();
                buf.drain(..end + skip);
                for line in event.lines() {
                    let Some(data) = line.strip_prefix("data:") else { continue };
                    let data = data.trim();
                    if data.is_empty() || data == "[DONE]" {
                        continue;
                    }
                    let v: Value = match serde_json::from_str(data) {
                        Ok(v) => v,
                        // A server that says something unparsable mid-stream
                        // is not a reason to throw away what it has said.
                        Err(_) => continue,
                    };
                    self.take_delta(&v, &mut reply);
                }
            }
        }
        Ok(reply)
    }

    /// One streamed chunk folded into the reply being built.
    fn take_delta(&self, v: &Value, reply: &mut Reply) {
        let Some(delta) = v.pointer("/choices/0/delta") else { return };
        if let Some(text) = delta.get("content").and_then(|c| c.as_str())
            && !text.is_empty()
        {
            reply.text.push_str(text);
            self.say(Update::Delta(text.to_string()));
        }
        let Some(calls) = delta.get("tool_calls").and_then(|c| c.as_array()) else { return };
        for c in calls {
            // The index is the identity: a name arrives in one chunk and its
            // arguments over the next several, keyed by nothing else.
            let i = c.get("index").and_then(|i| i.as_u64()).unwrap_or(0) as usize;
            while reply.calls.len() <= i {
                reply.calls.push(Call::default());
            }
            let slot = &mut reply.calls[i];
            if let Some(id) = c.get("id").and_then(|i| i.as_str()) {
                slot.id = id.to_string();
            }
            if let Some(f) = c.get("function") {
                if let Some(n) = f.get("name").and_then(|n| n.as_str()) {
                    slot.name.push_str(n);
                }
                if let Some(a) = f.get("arguments").and_then(|a| a.as_str()) {
                    slot.args.push_str(a);
                }
            }
        }
    }

    fn brief(&self) -> String {
        match self.config.brief.trim() {
            "" => catalog::BRIEF.to_string(),
            extra => format!("{}\n\n{extra}", catalog::BRIEF),
        }
    }
}

#[derive(Default)]
struct Reply {
    text: String,
    calls: Vec<Call>,
}

#[derive(Default, Clone)]
struct Call {
    id: String,
    name: String,
    args: String,
}

/// The catalogue as the chat completions API wants it.
fn tool_list() -> Value {
    Value::Array(
        catalog::all()
            .iter()
            .map(|t| {
                json!({
                    "type": "function",
                    "function": {
                        "name": t.name,
                        "description": t.about,
                        "parameters": t.schema,
                    },
                })
            })
            .collect(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every tool reaches the model with a name, a sentence and an object
    /// schema. A model sent a tool with no parameters object calls it with
    /// nothing and the call fails at the far end.
    #[test]
    fn the_model_is_sent_the_whole_catalogue() {
        let list = tool_list();
        let list = list.as_array().expect("an array of tools");
        assert_eq!(list.len(), catalog::all().len());
        assert_eq!(list.len(), 79);
        for t in list {
            let f = &t["function"];
            assert!(f["name"].as_str().is_some_and(|n| !n.is_empty()));
            assert!(f["description"].as_str().is_some_and(|d| d.len() > 10));
            assert_eq!(f["parameters"]["type"], "object");
        }
    }

    /// A reply arrives with blank lines round it and is drawn without them.
    ///
    /// Models open with a newline or two more often than not, and a chat pane
    /// that prints them shows an empty gap under AGENT: an operator reads
    /// that as a reply that has not come, and waits for one that already has.
    #[test]
    fn the_blank_lines_a_model_wraps_its_reply_in_are_not_drawn() {
        let (tx, rx) = crossbeam_channel::unbounded();
        let mut chat = Chat { updates: Some(rx), ..Default::default() };
        for d in ["\n\n", "\n", "Hey! I am driving the radio", " here.", "\n\n"] {
            let _ = tx.send(Update::Delta(d.to_string()));
        }
        let _ = tx.send(Update::Said);
        chat.poll();
        assert_eq!(
            chat.turns.iter().filter(|t| matches!(t, Turn::Said(_))).count(),
            1,
            "the blank deltas started turns of their own"
        );
        let Some(Turn::Said(said)) = chat.turns.last() else { panic!("no reply came out") };
        assert_eq!(said, "Hey! I am driving the radio here.");

        // A newline inside the reply is the model's own paragraph and stays.
        let (tx, rx) = crossbeam_channel::unbounded();
        let mut chat = Chat { updates: Some(rx), ..Default::default() };
        let _ = tx.send(Update::Delta("\n first\n\nsecond \n".into()));
        let _ = tx.send(Update::Said);
        chat.poll();
        let Some(Turn::Said(said)) = chat.turns.last() else { panic!("no reply") };
        assert_eq!(said, "first\n\nsecond");
    }

    /// A tool call arrives a character at a time, and the name and the
    /// arguments belong to whichever index they carried.
    #[test]
    fn a_streamed_tool_call_is_reassembled() {
        let (out, _rx) = crossbeam_channel::unbounded();
        let (desk, _asks) = Desk::new();
        let w = Worker {
            config: Config::default(),
            history: Vec::new(),
            desk,
            out,
            stop: Arc::new(AtomicBool::new(false)),
            bell: None,
        };
        let mut reply = Reply::default();
        let chunks = [
            json!({"choices":[{"delta":{"tool_calls":[{"index":0,"id":"c1","function":{"name":"tu"}}]}}]}),
            json!({"choices":[{"delta":{"tool_calls":[{"index":0,"function":{"name":"ne","arguments":"{\"mhz\":"}}]}}]}),
            json!({"choices":[{"delta":{"tool_calls":[{"index":0,"function":{"arguments":"145.5}"}}]}}]}),
            json!({"choices":[{"delta":{"content":"tuning"}}]}),
        ];
        for c in chunks {
            w.take_delta(&c, &mut reply);
        }
        assert_eq!(reply.text, "tuning");
        assert_eq!(reply.calls.len(), 1);
        assert_eq!(reply.calls[0].id, "c1");
        assert_eq!(reply.calls[0].name, "tune");
        assert_eq!(reply.calls[0].args, "{\"mhz\":145.5}");
        let tool = catalog::find(&reply.calls[0].name).expect("tune is a tool");
        let args: Value = serde_json::from_str(&reply.calls[0].args).expect("parsable arguments");
        assert!(tool.action(args).is_ok(), "the reassembled call is an action");
    }

    /// Two calls in one reply keep their own arguments.
    #[test]
    fn two_calls_do_not_run_together() {
        let (out, _rx) = crossbeam_channel::unbounded();
        let (desk, _asks) = Desk::new();
        let w = Worker {
            config: Config::default(),
            history: Vec::new(),
            desk,
            out,
            stop: Arc::new(AtomicBool::new(false)),
            bell: None,
        };
        let mut reply = Reply::default();
        w.take_delta(
            &json!({"choices":[{"delta":{"tool_calls":[
                {"index":0,"id":"a","function":{"name":"status","arguments":"{}"}},
                {"index":1,"id":"b","function":{"name":"packets","arguments":"{\"limit\":5}"}}
            ]}}]}),
            &mut reply,
        );
        assert_eq!(reply.calls.len(), 2);
        assert_eq!(reply.calls[0].name, "status");
        assert_eq!(reply.calls[1].args, "{\"limit\":5}");
    }

    /// A stub server that speaks the API, so the loop can be run whole:
    /// stream, tool call, desk, second request, answer.
    ///
    /// `replies` are sent in order, one per request, each as a body of SSE
    /// events. The requests are kept, since what the model is told the second
    /// time round is most of what this is checking.
    fn stub_server(replies: Vec<Vec<Value>>) -> (String, std::sync::mpsc::Receiver<Value>) {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("a port");
        let url = format!("http://{}/v1", listener.local_addr().expect("an address"));
        let (seen, asked) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            use std::io::{Read, Write};
            let mut left = replies.into_iter();
            for stream in listener.incoming() {
                let Ok(mut s) = stream else { return };
                // One connection may carry both requests, since the client
                // keeps it alive between them.
                loop {
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
                    let mut body = vec![0u8; len];
                    if s.read_exact(&mut body).is_err() {
                        return;
                    }
                    let _ = seen.send(serde_json::from_slice(&body).unwrap_or(Value::Null));
                    let Some(events) = left.next() else { return };
                    let mut sse = String::new();
                    for e in events {
                        sse.push_str(&format!("data: {e}\n\n"));
                    }
                    sse.push_str("data: [DONE]\n\n");
                    let head = format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n\
                         Content-Length: {}\r\n\r\n",
                        sse.len()
                    );
                    if s.write_all(head.as_bytes()).is_err() || s.write_all(sse.as_bytes()).is_err()
                    {
                        return;
                    }
                    let _ = s.flush();
                }
            }
        });
        (url, asked)
    }

    /// The whole exchange, against a server that calls one tool and then
    /// answers: the call reaches the desk, its answer goes back to the model,
    /// and what the model says last is what the pane is left holding.
    #[test]
    fn a_tool_call_reaches_the_receiver_and_its_answer_goes_back() {
        let (url, asked) = stub_server(vec![
            vec![json!({"choices":[{"delta":{"tool_calls":[
                {"index":0,"id":"c1","function":{"name":"status","arguments":"{}"}}
            ]}}]})],
            vec![
                json!({"choices":[{"delta":{"content":"the radio is "}}]}),
                json!({"choices":[{"delta":{"content":"on 446 MHz"}}]}),
            ],
        ]);
        let (desk, asks) = Desk::new();
        // The interface, in as much as the test needs one.
        let radio = std::thread::spawn(move || {
            let ask = asks.recv().expect("one tool call");
            assert!(matches!(ask.action, super::super::Action::Status));
            let _ = ask.reply.send(Ok(json!({ "center_hz": 446_050_000.0 })));
        });
        let (out, updates) = crossbeam_channel::unbounded();
        let worker = Worker {
            config: Config { url, model: "stub".into(), ..Config::default() },
            history: vec![json!({ "role": "user", "content": "where is the radio" })],
            desk,
            out,
            stop: Arc::new(AtomicBool::new(false)),
            bell: None,
        };
        let rt =
            tokio::runtime::Builder::new_current_thread().enable_all().build().expect("a runtime");
        rt.block_on(worker.run());
        radio.join().expect("the interface answered");

        let mut said = String::new();
        let mut called = Vec::new();
        let mut answers = Vec::new();
        let mut faults = Vec::new();
        for u in updates.try_iter() {
            match u {
                Update::Delta(t) => said.push_str(&t),
                Update::Calling { name, .. } => called.push(name),
                Update::Answered(r) => answers.push(r),
                Update::Fault(e) => faults.push(e),
                Update::Said | Update::Done => {}
            }
        }
        assert!(faults.is_empty(), "the exchange faulted: {faults:?}");
        assert_eq!(called, vec!["status"]);
        assert_eq!(answers.len(), 1);
        assert_eq!(answers[0].as_deref(), Ok("{\"center_hz\":446050000.0}"));
        assert_eq!(said, "the radio is on 446 MHz");

        // Two requests: the question, then the question with the call and
        // the receiver's answer behind it.
        let first = asked.recv().expect("a first request");
        assert_eq!(first["model"], "stub");
        assert_eq!(first["stream"], true);
        assert_eq!(first["tools"].as_array().map(|t| t.len()), Some(catalog::all().len()));
        assert_eq!(first["messages"][0]["role"], "system");
        assert_eq!(first["messages"][1]["content"], "where is the radio");
        let second = asked.recv().expect("a second request");
        let msgs = second["messages"].as_array().expect("messages").clone();
        assert_eq!(msgs.len(), 4, "system, question, the call, its answer");
        assert_eq!(msgs[2]["tool_calls"][0]["function"]["name"], "status");
        assert_eq!(msgs[3]["role"], "tool");
        assert_eq!(msgs[3]["tool_call_id"], "c1");
        assert_eq!(msgs[3]["content"], "{\"center_hz\":446050000.0}");
    }

    /// A server that refuses is reported to the operator rather than left as
    /// a conversation that stops for no stated reason.
    #[test]
    fn a_server_that_refuses_is_reported() {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("a port");
        let url = format!("http://{}/v1", listener.local_addr().expect("an address"));
        std::thread::spawn(move || {
            use std::io::{Read, Write};
            if let Some(Ok(mut s)) = listener.incoming().next() {
                let mut buf = [0u8; 4096];
                let _ = s.read(&mut buf);
                let body = "{\"error\":\"no such model\"}";
                let _ = s.write_all(
                    format!(
                        "HTTP/1.1 404 Not Found\r\nContent-Length: {}\r\n\r\n{body}",
                        body.len()
                    )
                    .as_bytes(),
                );
            }
        });
        let (desk, _asks) = Desk::new();
        let (out, updates) = crossbeam_channel::unbounded();
        let worker = Worker {
            config: Config { url, model: "absent".into(), ..Config::default() },
            history: vec![json!({ "role": "user", "content": "hello" })],
            desk,
            out,
            stop: Arc::new(AtomicBool::new(false)),
            bell: None,
        };
        let rt =
            tokio::runtime::Builder::new_current_thread().enable_all().build().expect("a runtime");
        rt.block_on(worker.run());
        let faults: Vec<String> = updates
            .try_iter()
            .filter_map(|u| match u {
                Update::Fault(e) => Some(e),
                _ => None,
            })
            .collect();
        assert_eq!(faults.len(), 1);
        assert!(faults[0].contains("404"), "{}", faults[0]);
        assert!(faults[0].contains("no such model"), "{}", faults[0]);
    }

    /// A question with nowhere to send it is answered in the pane rather
    /// than silently dropped.
    #[test]
    fn a_chat_with_no_model_says_so() {
        let mut chat = Chat { config: Config::default(), ..Chat::default() };
        chat.config.model.clear();
        let rt = tokio::runtime::Builder::new_current_thread().build().expect("a runtime");
        let (desk, _asks) = Desk::new();
        chat.ask("what is on the air", desk, rt.handle());
        assert_eq!(chat.turns.len(), 2);
        assert!(matches!(chat.turns[1], Turn::Fault(_)));
        assert!(!chat.busy(), "nothing was started");
    }
}
