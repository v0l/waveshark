//! Text sent over the air: who wrote it, who it was for, and what it said.
//!
//! The call list answers who is talking. This answers what was written, which
//! is the other half of a trunked network's traffic and the whole of a pager's.
//!
//! # Fed from the bus, not from a protocol
//!
//! Nothing here knows what TETRA or M17 is. A message is any decode carrying a
//! text field: `text` or `message`, with `from` and `to` read the same way the
//! call list reads them, and `addressee` and `address` besides, because APRS
//! names the recipient inside the payload rather than in the frame header and
//! a pager has nothing but an address. A decoder added later joins this view
//! by naming its fields the same way.
//!
//! # A repeat is not a new message
//!
//! A pager sends the same page twice, a TETRA short data message is
//! retransmitted until it is acknowledged, and an M17 link setup frame carries
//! its metadata on every frame of the stream. The same words from the same
//! sender to the same recipient inside [`REPEAT`] are one message with a
//! count, not a screen of duplicates. That window is deliberately generous:
//! two identical pages an hour apart are two pages, two a second apart are one
//! transmission heard twice.

use crate::radio::DecodeRecord;
use common::Value;
use std::time::{Duration, Instant};

/// How close two identical messages have to be to be the same message.
pub const REPEAT: Duration = Duration::from_secs(120);

/// Messages kept. Text is small and there are few of them next to packets,
/// so this is a session's worth rather than a screenful.
const MAX_MESSAGES: usize = 500;

/// One message, with however many times it was heard.
#[derive(Clone, Debug)]
pub struct Message {
    /// The system it came over, taken from the protocol name: `TETRA-SDS`
    /// becomes `TETRA`, so every mode of one system shares a name.
    pub system: String,
    /// Centre of the channel it was heard on, in hertz.
    pub channel_hz: f64,
    /// Whoever sent it, where the system says. A pager network does not.
    pub from: Option<String>,
    /// Who it was addressed to: a talkgroup, a subscriber, a pager's
    /// capcode, or an APRS addressee.
    pub to: Option<String>,
    pub text: String,
    pub first: Instant,
    pub last: Instant,
    /// Times it was heard, which for a pager is usually two.
    pub heard: u64,
}

impl Message {
    pub fn age(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.last)
    }

    /// The header a row shows above the text.
    pub fn title(&self) -> String {
        match (&self.from, &self.to) {
            (Some(f), Some(t)) => format!("{f} > {t}"),
            (Some(f), None) => f.clone(),
            (None, Some(t)) => format!("to {t}"),
            (None, None) => String::new(),
        }
    }
}

#[derive(Default)]
pub struct Messages {
    seen: Vec<Message>,
}

impl Messages {
    /// Newest first, which is the order anybody reads a message list in.
    pub fn recent(&self) -> Vec<&Message> {
        let mut v: Vec<&Message> = self.seen.iter().collect();
        v.sort_by(|a, b| b.last.cmp(&a.last));
        v
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }

    pub fn clear(&mut self) {
        self.seen.clear();
    }

    /// Fold one decode in, if it carried text at all.
    ///
    /// Returns whether it did.
    pub fn update(&mut self, rec: &DecodeRecord, at: Instant) -> bool {
        let Some(msg) = rec.to_message(at) else { return false };

        if let Some(m) = self.seen.iter_mut().find(|m| {
            m.system == msg.system && m.text == msg.text && m.from == msg.from && m.to == msg.to
                && at.saturating_duration_since(m.last) < REPEAT
        }) {
            m.last = at;
            m.heard += 1;
            return true;
        }

        self.seen.push(msg);
        if self.seen.len() > MAX_MESSAGES {
            let drop = self.seen.len() - MAX_MESSAGES;
            self.seen.drain(..drop);
        }
        true
    }
}

impl DecodeRecord {
    /// The message this decode carries, if it carries one.
    ///
    /// This is the one place the convention lives, so anything holding a
    /// record can ask it for a message: the packet log as it appends, a feed,
    /// or a view added later. It reads fields rather than switching on the
    /// protocol, which is what keeps `docs/views.md` true, and a decoder joins
    /// the message view by naming its fields the way everything else does.
    ///
    /// Borrowing rather than consuming, hence `to_` and not `into_`: the
    /// record carries on to the packet log, and the message is a second
    /// reading of it rather than a replacement.
    ///
    /// An empty string is not a message: a link setup frame with an empty
    /// metadata field is a voice transmission, not somebody writing nothing.
    pub fn to_message(&self, at: Instant) -> Option<Message> {
        let body =
            text(self, &["text", "message", "sms"]).filter(|t| !t.trim().is_empty())?;
        Some(Message {
            system: self.model.split('-').next().unwrap_or(&self.model).to_string(),
            channel_hz: self.freq,
            from: text(self, &["from", "src", "source", "radio_id"])
                .filter(|s| !s.is_empty()),
            to: text(
                self,
                &["addressee", "to", "dst", "destination", "talkgroup", "channel", "address"],
            )
            .filter(|s| !s.is_empty()),
            text: body,
            first: at,
            last: at,
            heard: 1,
        })
    }
}

/// The first of these fields the decode carries, as text.
fn text(rec: &DecodeRecord, keys: &[&str]) -> Option<String> {
    for k in keys {
        if let Some((_, v)) = rec.fields.iter().find(|(name, _)| name == k) {
            return Some(match v {
                Value::Text(t) => t.clone(),
                other => other.to_string(),
            });
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rec(model: &str, freq: f64, fields: &[(&str, Value)]) -> DecodeRecord {
        let mut r = DecodeRecord::for_test(freq, model);
        r.channel_hz = 12_500.0;
        r.fields = fields.iter().map(|(k, v)| (k.to_string(), v.clone())).collect();
        r
    }

    fn t(secs: u64) -> Instant {
        Instant::now() + Duration::from_secs(secs)
    }

    /// The mesh protocols reach this view, sender and all.
    ///
    /// The field names here are the ones `nodes::lora_nodes::lora_decoded`
    /// actually emits. MeshCore named its sender `sender` once, which is not
    /// a name this reads, and the messages arrived with nobody on them.
    #[test]
    fn the_mesh_protocols_arrive_with_their_sender_and_recipient() {
        let mut m = Messages::default();

        // Meshtastic: a text message on the default channel.
        let meshtastic = rec(
            "Meshtastic",
            869.495e6,
            &[
                ("source", Value::Text("1de7f958".into())),
                ("destination", Value::Text("broadcast".into())),
                ("channel", Value::Text("LongFast (default key)".into())),
                ("port", Value::Text("text".into())),
                ("text", Value::Text("on my way".into())),
            ],
        );
        assert!(m.update(&meshtastic, t(0)));

        // MeshCore: a message on the public channel.
        let meshcore = rec(
            "MeshCore",
            869.525e6,
            &[
                ("type", Value::Text("group text".into())),
                ("channel", Value::Text("Public (default key)".into())),
                ("from", Value::Text("kieran".into())),
                ("text", Value::Text("on my way".into())),
            ],
        );
        assert!(m.update(&meshcore, t(0)));

        let got = m.recent();
        assert_eq!(got.len(), 2, "two systems, two messages");

        let mt = got.iter().find(|x| x.system == "Meshtastic").expect("meshtastic");
        assert_eq!(mt.from.as_deref(), Some("1de7f958"));
        assert_eq!(mt.to.as_deref(), Some("broadcast"));
        assert_eq!(mt.text, "on my way");

        let mc = got.iter().find(|x| x.system == "MeshCore").expect("meshcore");
        assert_eq!(mc.from.as_deref(), Some("kieran"), "the sender must survive");
        assert_eq!(mc.to.as_deref(), Some("Public (default key)"));
        assert!(!mc.title().is_empty(), "a message with no title is the bug");
    }

    /// The same words on two systems are two messages, not one repeat.
    #[test]
    fn one_systems_words_do_not_swallow_anothers() {
        let mut m = Messages::default();
        let f = [("from", Value::Text("kieran".into())), ("text", Value::Text("hi".into()))];
        assert!(m.update(&rec("MeshCore", 869.5e6, &f), t(0)));
        assert!(m.update(&rec("Meshtastic", 869.5e6, &f), t(1)));
        assert_eq!(m.recent().len(), 2);
    }

    #[test]
    fn a_decode_without_text_is_not_a_message() {
        let mut m = Messages::default();
        assert!(!m.update(&rec("M17-Voice", 433.475e6, &[("from", Value::Text("M0ABC".into()))]), t(0)));
        assert!(!m.update(&rec("M17-Packet", 433.475e6, &[("message", Value::Text("  ".into()))]), t(0)));
        assert!(m.is_empty());
    }

    #[test]
    fn every_system_that_names_its_text_the_same_way_joins_the_view() {
        // The point of the field names: this view is not a switch on protocol.
        let mut m = Messages::default();
        assert!(m.update(
            &rec("TETRA-SDS", 391.1e6, &[("from", Value::Text("2001".into())),
                 ("to", Value::Text("10223295".into())), ("text", Value::Text("on scene".into()))]),
            t(0),
        ));
        assert!(m.update(
            &rec("M17-Packet", 433.475e6, &[("from", Value::Text("M0ABC".into())),
                 ("to", Value::Text("M0XYZ".into())), ("message", Value::Text("hello".into()))]),
            t(1),
        ));
        assert!(m.update(
            &rec("POCSAG-Alpha", 153.35e6, &[("address", Value::Int(1234567)),
                 ("message", Value::Text("CALL CONTROL".into()))]),
            t(2),
        ));
        let list = m.recent();
        assert_eq!(list.len(), 3);
        assert_eq!(list[0].system, "POCSAG", "the newest is first");
        assert_eq!(list[0].to.as_deref(), Some("1234567"), "a capcode is who it was for");
        assert!(list[0].from.is_none(), "a pager network does not say who sent it");
        assert_eq!(list[2].title(), "2001 > 10223295");
    }

    #[test]
    fn the_same_page_sent_twice_is_one_message() {
        // Pagers repeat, TETRA retransmits until acknowledged, and an M17
        // link setup carries its text on every frame of the stream.
        let mut m = Messages::default();
        let page = rec(
            "POCSAG-Alpha",
            153.35e6,
            &[("address", Value::Int(1234567)), ("message", Value::Text("CALL CONTROL".into()))],
        );
        m.update(&page, t(0));
        m.update(&page, t(4));
        assert_eq!(m.recent().len(), 1);
        assert_eq!(m.recent()[0].heard, 2);
        // Far enough apart and it is somebody sending the same words again.
        m.update(&page, t(600));
        assert_eq!(m.recent().len(), 2);
    }

    #[test]
    fn the_same_words_to_a_different_recipient_are_a_different_message() {
        let mut m = Messages::default();
        let to = |who: &str| {
            rec(
                "TETRA-SDS",
                391.1e6,
                &[("to", Value::Text(who.into())), ("text", Value::Text("rtb".into()))],
            )
        };
        m.update(&to("10223295"), t(0));
        m.update(&to("15835885"), t(1));
        assert_eq!(m.recent().len(), 2);
    }
}

