//! The loop that puts a question to a model with the receiver's tools.
//!
//! The model is anything speaking the OpenAI chat completions API with tool
//! calls, named by [`config::Config`]. What it can do is [`catalog::all`],
//! the same list the MCP server publishes, and every call it makes goes onto
//! the same desk, so a model that tunes moves the dial on screen.
//!
//! Nothing here draws. The voice channel waits on it; the conversation in the
//! Agent view is `egui_bench::agent`.

use super::{Desk, catalog, config::Config};
use serde_json::{Value, json};

/// How long one completion may take. Long, because a local model on a CPU
/// answers in minutes and the request is what holds the conversation open.
const PATIENCE: common::time::Duration = common::time::Duration::from_secs(600);

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
    let mut worker = Worker { config, history, desk, said: String::new() };
    worker.exchange().await?;
    Ok((worker.said, worker.history))
}

/// One exchange, running off the interface thread.
struct Worker {
    config: Config,
    history: Vec<Value>,
    desk: Desk,
    said: String,
}

impl Worker {
    async fn exchange(&mut self) -> Result<(), String> {
        let client = httpc::client(PATIENCE).map_err(|e| e.to_string())?;
        let tools = tool_list();
        for _ in 0..self.config.steps.max(1) {
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
            }
        }
        Err(format!("stopped after {} tool calls", self.config.steps.max(1)))
    }

    /// Put one call on the desk, and say what came back.
    async fn run_tool(&self, call: &Call) -> Result<String, String> {
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
    async fn complete(&mut self, client: &reqwest::Client, tools: &Value) -> Result<Reply, String> {
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
    async fn read_stream(&mut self, resp: reqwest::Response) -> Result<Reply, String> {
        let mut reply = Reply::default();
        let mut buf = String::new();
        let mut chunks = httpc::Chunks::of(resp);
        while let Some(chunk) = chunks.next().await.map_err(|e| e.to_string())? {
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
    fn take_delta(&mut self, v: &Value, reply: &mut Reply) {
        let Some(delta) = v.pointer("/choices/0/delta") else { return };
        if let Some(text) = delta.get("content").and_then(|c| c.as_str())
            && !text.is_empty()
        {
            reply.text.push_str(text);
            self.said.push_str(text);
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
        assert_eq!(list.len(), 80);
        for t in list {
            let f = &t["function"];
            assert!(f["name"].as_str().is_some_and(|n| !n.is_empty()));
            assert!(f["description"].as_str().is_some_and(|d| d.len() > 10));
            assert_eq!(f["parameters"]["type"], "object");
        }
    }

    /// A tool call arrives a character at a time, and the name and the
    /// arguments belong to whichever index they carried.
    #[test]
    fn a_streamed_tool_call_is_reassembled() {
        let (desk, _asks) = Desk::new();
        let mut w =
            Worker { config: Config::default(), history: Vec::new(), desk, said: String::new() };
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
        let (desk, _asks) = Desk::new();
        let mut w =
            Worker { config: Config::default(), history: Vec::new(), desk, said: String::new() };
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
        common::thread::spawn(move || {
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
        let radio = common::thread::spawn(move || {
            let ask = asks.recv().expect("one tool call");
            assert!(matches!(ask.action, super::super::Action::Status));
            let _ = ask.reply.send(Ok(json!({ "center_hz": 446_050_000.0 })));
        });
        let rt =
            tokio::runtime::Builder::new_current_thread().enable_all().build().expect("a runtime");
        let config = Config { url, model: "stub".into(), ..Config::default() };
        let (said, history) = rt
            .block_on(ask_once(config, Vec::new(), desk, "where is the radio"))
            .expect("an answer");
        radio.join().expect("the interface answered");

        assert_eq!(said, "the radio is on 446 MHz");
        assert_eq!(history.last().map(|m| m["role"].clone()), Some(json!("assistant")));

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
        common::thread::spawn(move || {
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
        let rt =
            tokio::runtime::Builder::new_current_thread().enable_all().build().expect("a runtime");
        let config = Config { url, model: "absent".into(), ..Config::default() };
        let fault = rt
            .block_on(ask_once(config, Vec::new(), desk, "hello"))
            .expect_err("the server refused");
        assert!(fault.contains("404"), "{fault}");
        assert!(fault.contains("no such model"), "{fault}");
    }
}
