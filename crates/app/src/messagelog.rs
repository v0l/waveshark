//! What was written over the air, kept as lines.
//!
//! The message view holds a session's worth in memory and loses it when the
//! receiver stops, which is the wrong way round for the thing it holds: a
//! page or a short data message is a few dozen bytes that somebody wrote once
//! and will never send again. A band left running overnight should be
//! readable in the morning.
//!
//! # A line each, as text
//!
//! JSON Lines, one file a day, under `$XDG_DATA_HOME/waveshark/messages`. Not
//! the packet log's binary format, and for the opposite reason: that file
//! holds thousands of bursts a second and a hundred bytes of quoting per
//! pulse would turn a night into gigabytes, while this holds a few hundred
//! short strings a day and its whole value is that `grep` reads it without
//! this program. The packet log still has the bursts these were decoded from,
//! so nothing here is the only copy of anything.
//!
//! # A node, not a view
//!
//! It hangs off the packet bus beside the survey and the house feed, so a
//! receiver with no interface open writes the same file, and so an operator
//! can see it, switch it off and wire it elsewhere.

use common::Result;
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::Instant;

use crate::messages::{Message, Messages};

/// Where the messages are written, beside the packet log and the calls.
pub fn messages_dir() -> PathBuf {
    crate::wspkt::PacketLog::default_dir()
        .map(|d| d.with_file_name("messages"))
        .unwrap_or_else(|| std::env::temp_dir().join("waveshark-messages"))
}

/// How much of the folder is read back when the receiver starts.
///
/// Enough that last night is on screen and a week of a busy pager band is
/// not: the file is the record, and the view is the recent past.
pub const LOAD_DAYS: u64 = 2;

fn path_for(dir: &Path, at_us: u64) -> PathBuf {
    dir.join(format!("{}.jsonl", crate::segments::day_of(at_us)))
}

/// Append one message, opening the day's file as needed.
///
/// Errors are swallowed: a read-only home must not take the receiver down,
/// and a message that failed to be written is still on screen and still in
/// the packet log as the burst it was decoded from.
pub fn append(dir: &Path, m: &Message) {
    if std::fs::create_dir_all(dir).is_err() {
        return;
    }
    let line = line_of(m);
    let Ok(mut f) =
        std::fs::OpenOptions::new().create(true).append(true).open(path_for(dir, m.at_us))
    else {
        return;
    };
    let _ = writeln!(f, "{line}");
}

fn line_of(m: &Message) -> String {
    serde_json::json!({
        "at": crate::segments::iso_of(m.at_us),
        "at_us": m.at_us,
        "system": m.system,
        "channel_hz": m.channel_hz,
        "from": m.from,
        "to": m.to,
        "text": m.text,
        "destination": m.destination,
    })
    .to_string()
}

/// Read a day's file back.
pub fn read(path: &Path) -> Vec<Message> {
    let Ok(body) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    let now = Instant::now();
    let now_us = crate::messages::now_us();
    body.lines().filter_map(|l| message_of(l, now, now_us)).collect()
}

/// Every message written in the last `days` days, oldest first.
pub fn recent(dir: &Path, days: u64) -> Vec<Message> {
    let now_us = crate::messages::now_us();
    let mut out = Vec::new();
    for back in (0..=days).rev() {
        let at = now_us.saturating_sub(back * 86_400 * 1_000_000);
        let path = path_for(dir, at);
        if path.exists() {
            out.extend(read(&path));
        }
    }
    out.sort_by_key(|m| m.at_us);
    out
}

/// One line back into a message.
///
/// The clock time is what was written down; the receiver's own clock started
/// when it did, so the age a view shows is worked out from the difference. A
/// message from before the receiver started reads as its full age rather than
/// as having just arrived.
fn message_of(line: &str, now: Instant, now_us: u64) -> Option<Message> {
    let v: serde_json::Value = serde_json::from_str(line).ok()?;
    let at_us = v.get("at_us")?.as_u64()?;
    let text = v.get("text")?.as_str()?.to_string();
    let ago = std::time::Duration::from_micros(now_us.saturating_sub(at_us));
    let at = now.checked_sub(ago).unwrap_or(now);
    let s = |k: &str| v.get(k).and_then(|x| x.as_str()).map(str::to_string);
    let channel_hz = v.get("channel_hz").and_then(|x| x.as_f64()).unwrap_or(0.0);
    let destination = s("destination").or_else(|| {
        decode::p2000::on_channel(channel_hz).then(|| decode::p2000::destination(&text)).flatten()
    });
    Some(Message {
        system: s("system").unwrap_or_default(),
        channel_hz,
        from: s("from"),
        to: s("to"),
        text,
        destination,
        first: at,
        last: at,
        at_us,
        heard: 1,
        // From the file: this receiver has not heard it in this session, and
        // may not even be pointed at the band any more.
        logged: true,
    })
}

/// Writes every message the receiver hears to the folder.
pub struct MessageLogNode {
    enabled: bool,
    dir: PathBuf,
    /// The repeat rule, which is the message view's: a page sent twice is one
    /// message and one line, not two.
    seen: Messages,
    written: u64,
}

impl Default for MessageLogNode {
    fn default() -> Self {
        Self::new(messages_dir())
    }
}

impl MessageLogNode {
    pub fn new(dir: PathBuf) -> Self {
        Self { enabled: true, dir, seen: Messages::default(), written: 0 }
    }

    /// Messages written since the receiver started.
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn written(&self) -> u64 {
        self.written
    }
}

impl Simple for MessageLogNode {
    fn name(&self) -> &str {
        "message_log"
    }

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Packets {
            return Err(common::Error::other("the message log reads the packet bus"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        if !self.enabled {
            return Ok(());
        }
        let at = Instant::now();
        for p in i.as_packets().unwrap_or(&[]) {
            for (layer, said) in p.facts() {
                let common::packet::Fact::Message(w) = said else { continue };
                let Some(m) = Message::of(layer, p.carrier.center_hz as f64, &w.text, at) else {
                    continue;
                };
                if self.seen.push(m.clone(), at) {
                    append(&self.dir, &m);
                    self.written += 1;
                }
            }
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool("enabled", self.enabled).label("Write messages down"),
            Param::text("dir", self.dir.display().to_string()).label("Folder"),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            "enabled" => {
                self.enabled = v.as_bool().unwrap_or(true);
                Ok(())
            }
            "dir" => {
                let dir = PathBuf::from(v.as_str().unwrap_or_default());
                if !dir.as_os_str().is_empty() {
                    self.dir = dir;
                }
                Ok(())
            }
            other => Err(common::Error::other(format!("message_log has no {other}"))),
        }
    }
}

pub const DESC: StageDesc = StageDesc {
    name: "message_log",
    summary: "Writes everything somebody wrote over the air to a file a day",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let dir = match s.get("dir").and_then(|v| v.as_str()).filter(|d| !d.is_empty()) {
        Some(d) => PathBuf::from(d),
        None => messages_dir(),
    };
    let mut n = MessageLogNode::new(dir);
    n.enabled = s.bool_or("enabled", true);
    Ok(Box::new(n) as Box<dyn pipeline::node::Node>)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::packet::{Carrier, Fact, Frame, Link, Party, Proto};
    use pipeline::node::Node;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-msglog-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        d
    }

    fn heard(hz: u64, layer: Proto) -> common::packet::Packet {
        let carrier =
            Carrier::heard(common::packet::now_us(), hz, 12_500, -30.0, 20.0, common::SourceId(0));
        common::packet::Packet::heard(carrier).framed(Frame::of(vec![1])).decoded(layer)
    }

    fn page(text: &str, to: &str) -> common::packet::Packet {
        heard(
            153_350_000,
            Proto::new("POCSAG", "alpha")
                .between(Link { from: None, to: Some(Party::unit(to)) })
                .saying(Fact::message(text)),
        )
    }

    fn run(n: &mut MessageLogNode, packets: Vec<common::packet::Packet>) {
        let ins = [PortSpec {
            spec: StreamSpec { kind: PortKind::Packets, rate: 1.0, ..Default::default() },
            latency: 0,
        }];
        let (tags, mut events, mut out_tags) = (Vec::new(), Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut out_tags);
        let mut out = Payload::empty_of(PortKind::Packets);
        Simple::process(n, &Payload::Packets(packets), &mut out, &mut ctx).unwrap();
    }

    #[test]
    fn a_page_is_one_line_however_often_it_is_repeated() {
        let d = dir("repeat");
        let mut n = MessageLogNode::new(d.clone());
        // POCSAG sends the same page twice, and a pager network retransmits.
        for _ in 0..3 {
            run(&mut n, vec![page("CALL CONTROL", "1234567")]);
        }
        run(&mut n, vec![page("ATTEND STATION", "7654321")]);
        assert_eq!(n.written(), 2, "a repeat was written again");

        let back = recent(&d, 0);
        assert_eq!(back.len(), 2);
        assert_eq!(back[0].text, "CALL CONTROL");
        assert_eq!(back[0].to.as_deref(), Some("1234567"));
        assert_eq!(back[0].system, "POCSAG");
        assert_eq!(back[0].channel_hz, 153_350_000.0);
        assert_eq!(back[1].text, "ATTEND STATION");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// The file is what somebody reads tomorrow, so it has to be readable
    /// without this program: one JSON object a line, with the clock time in
    /// it.
    #[test]
    fn a_line_is_json_with_the_time_in_it() {
        let d = dir("json");
        let mut n = MessageLogNode::new(d.clone());
        run(&mut n, vec![page("PUMP 3 FAULT", "1111111")]);
        let file = std::fs::read_dir(&d).unwrap().flatten().next().unwrap().path();
        let body = std::fs::read_to_string(&file).unwrap();
        assert_eq!(body.lines().count(), 1, "{body}");
        let v: serde_json::Value = serde_json::from_str(body.lines().next().unwrap()).unwrap();
        assert_eq!(v["text"], "PUMP 3 FAULT");
        assert_eq!(v["system"], "POCSAG");
        let at = v["at"].as_str().expect("a time somebody can read");
        assert_eq!(at.len(), 20, "{at} is not an ISO 8601 UTC stamp");
        assert!(at.ends_with('Z'), "{at}");
        let _ = std::fs::remove_dir_all(&d);
    }

    /// A decode nobody wrote does not reach the file, whatever its fields are
    /// called: the same rule as the view.
    #[test]
    fn a_dispatch_keeps_where_it_sent_the_units_across_a_restart() {
        let d = dir("destination");
        let mut n = MessageLogNode::new(d.clone());
        let p = heard(
            169_650_000,
            Proto::new("flex", "FLEX-Alpha")
                .between(Link { from: None, to: Some(Party::group("1420999, 1423001")) })
                .saying(Fact::message("A1 Zuidsingel Venray 87804"))
                .saying(Fact::Destination("Zuidsingel, Venray".into())),
        );
        run(&mut n, vec![p, page("PUMP 3 FAULT", "1111111")]);
        let back: Vec<Option<String>> = recent(&d, 0).into_iter().map(|m| m.destination).collect();
        assert_eq!(back, [Some("Zuidsingel, Venray".to_string()), None]);
        let _ = std::fs::remove_dir_all(&d);

        let before = r#"{"at_us":1,"system":"flex","channel_hz":169650000.0,"text":"A1 Zuidsingel Venray 87804"}"#;
        let elsewhere = r#"{"at_us":1,"system":"flex","channel_hz":929612500.0,"text":"A1 Zuidsingel Venray 87804"}"#;
        let (now, now_us) = (Instant::now(), crate::messages::now_us());
        let read = |l: &str| message_of(l, now, now_us).and_then(|m| m.destination);
        assert_eq!(
            read(before).as_deref(),
            Some("Zuidsingel, Venray"),
            "a line written before destinations"
        );
        assert_eq!(read(elsewhere), None, "only the P2000 channel is read as P2000");
    }

    #[test]
    fn a_machine_talking_is_not_written_down() {
        let d = dir("machine");
        let mut n = MessageLogNode::new(d.clone());
        let p = heard(
            95_800_000,
            Proto::new("rds", "radiotext").saying(Fact::Playing("NOW PLAYING".into())),
        );
        run(&mut n, vec![p]);
        assert_eq!(n.written(), 0);
        assert!(!d.exists() || recent(&d, 0).is_empty());
        let _ = std::fs::remove_dir_all(&d);
    }

    #[test]
    fn switching_it_off_stops_the_file_growing() {
        let d = dir("off");
        let mut n = MessageLogNode::new(d.clone());
        run(&mut n, vec![page("ONE", "1")]);
        Node::set_param(&mut n, "enabled", ParamValue::Bool(false)).unwrap();
        run(&mut n, vec![page("TWO", "2")]);
        assert_eq!(n.written(), 1);
        assert_eq!(recent(&d, 0).len(), 1);
        let _ = std::fs::remove_dir_all(&d);
    }
}
