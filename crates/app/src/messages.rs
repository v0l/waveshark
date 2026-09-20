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

use crate::row::Reception;
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
    /// When it was first heard on the clock rather than on the receiver's,
    /// so a message written down today reads as today's when it is loaded
    /// back tomorrow. An `Instant` cannot survive a restart.
    pub at_us: u64,
    /// Times it was heard, which for a pager is usually two.
    pub heard: u64,
    /// Read back from the log rather than heard in this session.
    ///
    /// The list is loaded from the file at start so an overnight watch is
    /// readable in the morning, which means the view holds messages this
    /// receiver never heard on a band it is not pointed at. Unmarked, that
    /// reads as traffic arriving now.
    pub logged: bool,
}

impl Message {
    pub fn age(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.last)
    }

    /// When it was heard, on the clock, as a person reads it.
    ///
    /// The log carries the real timestamp, so this is that and not an
    /// estimate: `14:32` in UTC for a message from today, and the date in
    /// front of it for one that is not.
    pub fn when(&self) -> String {
        let at = crate::segments::when(self.at_us);
        match crate::segments::day_of(self.at_us) == crate::segments::day_of(now_us()) {
            true => at.format("%H:%M").to_string(),
            false => at.format("%Y-%m-%d %H:%M").to_string(),
        }
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

    pub fn len(&self) -> usize {
        self.seen.len()
    }

    pub fn clear(&mut self) {
        self.seen.clear();
    }

    /// Fold one decode in, if it carried text at all.
    ///
    /// Returns whether it did.
    pub fn update(&mut self, rec: &Reception, at: Instant) -> bool {
        let Some(msg) = rec.to_message(at) else {
            return false;
        };
        self.push(msg, at);
        true
    }

    /// Fold a message in, and say whether it was one nobody had heard.
    ///
    /// The repeat rule in one place: a pager sends the same page twice and a
    /// TETRA short data message is retransmitted until it is acknowledged, so
    /// whatever is counting messages, showing them or writing them down has to
    /// agree about what "the same message" is.
    pub fn push(&mut self, msg: Message, at: Instant) -> bool {
        if let Some(m) = self.seen.iter_mut().find(|m| {
            m.system == msg.system
                && m.text == msg.text
                && m.from == msg.from
                && m.to == msg.to
                && at.saturating_duration_since(m.last) < REPEAT
        }) {
            m.last = at;
            m.heard += 1;
            return false;
        }

        self.seen.push(msg);
        if self.seen.len() > MAX_MESSAGES {
            let drop = self.seen.len() - MAX_MESSAGES;
            self.seen.drain(..drop);
        }
        true
    }
}

impl Reception {
    /// The message this decode carries, if it carries one.
    ///
    /// This is the one place the convention lives, so anything holding a
    /// record can ask it for a message: the packet log as it appends, a feed,
    /// or a view added later. It reads fields rather than switching on the
    /// protocol, so a decoder joins the message view by naming its fields the
    /// way everything else does.
    ///
    /// Borrowing rather than consuming, hence `to_` and not `into_`: the
    /// record carries on to the packet log, and the message is a second
    /// reading of it rather than a replacement.
    ///
    /// An empty string is not a message: a link setup frame with an empty
    /// metadata field is a voice transmission, not somebody writing nothing.
    ///
    /// Only a decode that says somebody wrote it. Reading a field called
    /// `message` filled this view with GSM naming its own blocks (`SI3`,
    /// `Paging1`) and a drone naming its message types; reading the media
    /// type instead filled it with an FM station's track listing and an
    /// aircraft's position report, which are text and are not messages.
    /// What belongs here is somebody writing to somebody, and only the
    /// decoder knows: it says so with [`common::packet::Fact::Message`].
    pub fn to_message(&self, at: Instant) -> Option<Message> {
        let (layer, written) = self.packet.facts().find_map(|(l, f)| match f {
            common::packet::Fact::Message(w) => Some((l, w)),
            _ => None,
        })?;
        Message::of(layer, self.freq(), &written.text, at)
    }
}

impl Message {
    /// The message a decode's fields make, for anything holding fields rather
    /// than a record: the log node writes from the bus, the view folds from
    /// the record, and both have to read the same names.
    pub fn of(
        layer: &common::packet::Proto,
        channel_hz: f64,
        text: &str,
        at: Instant,
    ) -> Option<Self> {
        if text.trim().is_empty() {
            return None;
        }
        let who = |p: &Option<common::packet::Party>| {
            p.as_ref().map(|q| q.label().to_string()).filter(|s| !s.is_empty())
        };
        Some(Message {
            system: layer.id.to_string(),
            channel_hz,
            // The sender the protocol named. Where a message carries a name
            // somebody typed as well, the decoder puts the typed name in the
            // text it states, because a group message carries no signature
            // and anyone with the channel key can write any name there.
            from: who(&layer.link.from),
            to: who(&layer.link.to),
            text: text.to_string(),
            first: at,
            last: at,
            at_us: now_us(),
            heard: 1,
            logged: false,
        })
    }
}

/// Now, in microseconds since the epoch.
pub fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::packet::{Carrier, Fact, Link, Packet, Party, Proto};

    /// A message read back off the file is marked as such, and says when it
    /// was heard rather than how long ago.
    ///
    /// The list is loaded at start so an overnight watch is readable in the
    /// morning, which means it holds traffic from a band the receiver is no
    /// longer pointed at. Unmarked, that reads as something arriving now:
    /// mesh nodes from another session appeared to be on the air.
    #[test]
    fn a_message_off_the_log_says_so() {
        let now = Instant::now();
        let heard = Message::of(&Proto::new("meshtastic", "text"), 869_519_700.0, "Hi", now)
            .expect("a message");
        assert!(!heard.logged, "something heard now is not from the log");

        let dir = std::env::temp_dir().join(format!("waveshark-msg-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        crate::messagelog::append(&dir, &heard);
        let back = crate::messagelog::recent(&dir, 2);
        assert_eq!(back.len(), 1, "the message did not come back: {back:?}");
        assert!(back[0].logged, "a message off the file is not marked as read back");
        assert_eq!(back[0].at_us, heard.at_us, "the timestamp did not survive the file");
        assert_eq!(
            back[0].when(),
            crate::segments::when(heard.at_us).format("%H:%M").to_string(),
            "a message from today reads as a time of day"
        );
        // And one from another day says which.
        let yesterday = Message { at_us: heard.at_us - 86_400_000_000, ..heard.clone() };
        assert!(
            yesterday.when().starts_with(&crate::segments::day_of(yesterday.at_us)),
            "{}",
            yesterday.when()
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// A reception carrying one decode, as the bus delivers it.
    fn heard(id: &'static str, kind: &'static str, freq: f64, layer: Proto) -> Reception {
        let carrier =
            Carrier::heard(now_us(), freq as u64, 12_500, -70.0, 12.0, common::SourceId(0));
        let _ = (id, kind);
        Reception::new(Instant::now(), Packet::heard(carrier).decoded(layer))
    }

    /// A decode that says somebody wrote it, which is what this view reads.
    fn wrote(id: &'static str, freq: f64, link: Link, text: &str) -> Reception {
        heard(id, "text", freq, Proto::new(id, "text").between(link).saying(Fact::message(text)))
    }

    fn t(secs: u64) -> Instant {
        Instant::now() + Duration::from_secs(secs)
    }

    /// The mesh protocols reach this view, sender and all.
    ///
    /// Both name their ends on the link, so the view reads them the same way
    /// without knowing what either protocol is.
    #[test]
    fn the_mesh_protocols_arrive_with_their_sender_and_recipient() {
        let mut m = Messages::default();

        let meshtastic =
            wrote("meshtastic", 869.495e6, Link::beacon(Party::unit("1de7f958")), "on my way");
        assert!(m.update(&meshtastic, t(0)));

        // A group message names the channel as its recipient, because that is
        // as far as the protocol says it went.
        let meshcore = wrote(
            "meshcore",
            869.525e6,
            Link::between(Party::unit("kieran"), Party::group("Public")),
            "on my way",
        );
        assert!(m.update(&meshcore, t(0)));

        let got = m.recent();
        assert_eq!(got.len(), 2, "two systems, two messages");

        let mt = got.iter().find(|x| x.system == "meshtastic").expect("meshtastic");
        assert_eq!(mt.from.as_deref(), Some("1de7f958"));
        assert_eq!(mt.to.as_deref(), Some("broadcast"));
        assert_eq!(mt.text, "on my way");

        let mc = got.iter().find(|x| x.system == "meshcore").expect("meshcore");
        assert_eq!(mc.from.as_deref(), Some("kieran"), "the sender must survive");
        assert_eq!(mc.to.as_deref(), Some("Public"));
        assert!(!mc.title().is_empty(), "a message with no title is the bug");
    }

    /// The same words on two systems are two messages, not one repeat.
    #[test]
    fn one_systems_words_do_not_swallow_anothers() {
        let mut m = Messages::default();
        let link = || Link::from(Party::unit("kieran"));
        assert!(m.update(&wrote("meshcore", 869.5e6, link(), "hi"), t(0)));
        assert!(m.update(&wrote("meshtastic", 869.5e6, link(), "hi"), t(1)));
        assert_eq!(m.recent().len(), 2);
    }

    /// Nothing but a stated message reaches this view.
    ///
    /// A voice transmission names its ends and carries no text; a packet
    /// whose text is empty is a link setup, not somebody writing nothing.
    #[test]
    fn a_decode_without_text_is_not_a_message() {
        let mut m = Messages::default();
        let voice = heard(
            "m17",
            "voice",
            433.475e6,
            Proto::new("m17", "voice").between(Link::from(Party::unit("M0ABC"))),
        );
        assert!(!m.update(&voice, t(0)));
        let blank = wrote("m17", 433.475e6, Link::from(Party::unit("M0ABC")), "  ");
        assert!(!m.update(&blank, t(0)));
        assert!(m.is_empty());
    }

    /// Any system that says somebody wrote something joins this view.
    ///
    /// The view is not a switch on protocol and not a search for a field
    /// called `text`: TETRA, M17 and a pager network have nothing in common
    /// but the statement.
    #[test]
    fn every_system_that_states_a_message_joins_the_view() {
        let mut m = Messages::default();
        assert!(m.update(
            &wrote(
                "tetra",
                391.1e6,
                Link::between(Party::unit("2001"), Party::group("10223295")),
                "on scene"
            ),
            t(0),
        ));
        assert!(m.update(
            &wrote(
                "m17",
                433.475e6,
                Link::between(Party::unit("M0ABC"), Party::unit("M0XYZ")),
                "hello"
            ),
            t(1),
        ));
        // A pager network has nothing but an address, and the address is who
        // the page was for rather than who sent it.
        assert!(m.update(
            &wrote(
                "pocsag",
                153.35e6,
                Link { from: None, to: Some(Party::unit("1234567")) },
                "CALL CONTROL"
            ),
            t(2),
        ));
        let list = m.recent();
        assert_eq!(list.len(), 3);
        assert_eq!(list[0].system, "pocsag", "the newest is first");
        assert_eq!(list[0].to.as_deref(), Some("1234567"), "a capcode is who it was for");
        assert!(list[0].from.is_none(), "a pager network does not say who sent it");
        assert_eq!(list[2].title(), "2001 > 10223295");
    }

    #[test]
    fn the_same_page_sent_twice_is_one_message() {
        // Pagers repeat, TETRA retransmits until acknowledged, and an M17
        // link setup carries its text on every frame of the stream.
        let mut m = Messages::default();
        let page = || {
            wrote(
                "pocsag",
                153.35e6,
                Link { from: None, to: Some(Party::unit("1234567")) },
                "CALL CONTROL",
            )
        };
        m.update(&page(), t(0));
        m.update(&page(), t(4));
        assert_eq!(m.recent().len(), 1);
        assert_eq!(m.recent()[0].heard, 2);
        // Far enough apart and it is somebody sending the same words again.
        m.update(&page(), t(600));
        assert_eq!(m.recent().len(), 2);
    }

    #[test]
    fn the_same_words_to_a_different_recipient_are_a_different_message() {
        let mut m = Messages::default();
        let to = |who: &str| {
            wrote("tetra", 391.1e6, Link { from: None, to: Some(Party::group(who)) }, "rtb")
        };
        m.update(&to("10223295"), t(0));
        m.update(&to("15835885"), t(1));
        assert_eq!(m.recent().len(), 2);
    }

    /// A machine is not a correspondent.
    ///
    /// GSM calls one of its fields `message` and puts `SI3` in it, an FM
    /// station's radiotext is its track listing, and an ACARS downlink is an
    /// aeroplane reporting its position. All three are text and none was
    /// written to anybody, so none of them states a message: what a station
    /// is playing is a fact of its own, and a position is a position.
    #[test]
    fn a_machine_talking_is_not_a_message() {
        let mut m = Messages::default();
        let gsm = heard("gsm", "SI3", 947.4e6, Proto::new("gsm", "SI3"));
        assert!(!m.update(&gsm, t(0)));
        let rds = heard(
            "rds",
            "radiotext",
            95.8e6,
            Proto::new("rds", "radiotext").saying(Fact::Playing("NOW PLAYING".into())),
        );
        assert!(!m.update(&rds, t(0)));
        let acars = heard(
            "acars",
            "downlink",
            131.725e6,
            Proto::new("acars", "downlink").between(Link::from(Party::unit("G-EZBF"))),
        );
        assert!(!m.update(&acars, t(0)));
        assert!(m.recent().is_empty(), "{:?}", m.recent());
    }
}
