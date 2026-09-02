//! Who is talking to whom, on which channel.
//!
//! A scanner's central question is not what a packet said but who is on the
//! air: which talkgroup opened up, who keyed the microphone, how long they
//! held it, and where to listen next time. That is the same question on a
//! trunked network as on a single simplex channel, and the answer has the same
//! shape on both, so this keeps one table for all of them.
//!
//! # Fed from the bus, not from a protocol
//!
//! Nothing here knows what M17 or DMR is. A call is assembled from the fields
//! a decode already carries: `from` for whoever is transmitting, `to` for the
//! group or party being called, `seconds` for how long the transmission ran.
//! A decoder added later joins the view by naming its fields the same way,
//! which is the same bargain the map makes with positions.
//!
//! `call_type` is how a system that knows says which kind of call this is,
//! since a trunked network distinguishes a talkgroup from a private call
//! outright. Where it is absent the destination decides, and that is a guess:
//! see [`is_group`].

use crate::radio::DecodeRecord;
use common::Value;
use std::time::{Duration, Instant};

/// How long after the last transmission a call is still counted as live.
///
/// Long enough to hold a conversation together between overs, short enough
/// that a lamp on screen means somebody is talking now. A trunked talkgroup
/// hangs on its channel for a few seconds between transmissions for exactly
/// this reason.
pub const LIVE: Duration = Duration::from_secs(6);

/// How long a call stays in the table after it ends.
const FORGET: Duration = Duration::from_secs(60 * 60);

/// Beyond this many calls the oldest are dropped, so a night on a busy band
/// cannot grow without bound.
const MAX_CALLS: usize = 2048;

/// One conversation: a source, a destination, and the channel it happened on.
#[derive(Clone, Debug)]
pub struct Call {
    /// The system it belongs to, taken from the protocol name: `M17-Voice`
    /// becomes `M17`, so every mode of one system shares a row.
    pub system: String,
    /// Centre of the channel it was heard on, in hertz.
    pub channel_hz: f64,
    /// The talkgroup, reflector or party being called.
    pub to: String,
    /// Whoever is transmitting, when the system says.
    pub from: Option<String>,
    /// A call to a group rather than to one party.
    pub group: bool,
    /// Whether the traffic is enciphered, which decides whether there is any
    /// point listening to it.
    pub encrypted: bool,
    pub first: Instant,
    pub last: Instant,
    /// Separate keyings of the microphone, not packets.
    pub overs: u64,
    /// Airtime in seconds, where the protocol says how long a transmission
    /// ran. Zero where it does not.
    pub seconds: f64,
}

impl Call {
    pub fn age(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.last)
    }

    /// Whether somebody is talking on it now.
    pub fn live(&self, now: Instant) -> bool {
        self.age(now) < LIVE
    }

    /// How long the conversation has been going, which is not its airtime: a
    /// group can be busy for a minute in six seconds of speech.
    pub fn span(&self) -> Duration {
        self.last.saturating_duration_since(self.first)
    }

    /// The label a list shows: the group, with the caller beside it.
    pub fn title(&self) -> String {
        match &self.from {
            Some(f) if self.group => format!("{} < {f}", self.to),
            Some(f) => format!("{f} > {}", self.to),
            None => self.to.clone(),
        }
    }
}

#[derive(Default)]
pub struct Calls {
    seen: Vec<Call>,
}

impl Calls {
    pub fn new() -> Self {
        Self::default()
    }

    /// Calls heard recently, the ones being talked on first and the rest by
    /// how recently they were.
    ///
    /// Sorted by recency rather than by first appearance, which is the
    /// opposite of what the track list does and right for the opposite reason:
    /// a scanner is watched to see who has just come up, and the row that
    /// matters is the one that changed.
    pub fn active(&self, now: Instant) -> Vec<&Call> {
        let mut v: Vec<&Call> = self.seen.iter().filter(|c| c.age(now) < FORGET).collect();
        v.sort_by(|a, b| b.last.cmp(&a.last));
        v
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }

    pub fn clear(&mut self) {
        self.seen.clear();
    }

    /// Fold one decode in, if it is a call at all.
    ///
    /// Returns whether it was. Anything without a destination is somebody
    /// else's business: a sensor reading, a pager message, an aircraft.
    pub fn update(&mut self, rec: &DecodeRecord, at: Instant) -> bool {
        let Some(to) = text(rec, &["to", "dst", "destination", "talkgroup", "group"]) else {
            return false;
        };
        if to.is_empty() {
            return false;
        }
        let from = text(rec, &["from", "src", "source", "radio_id"]).filter(|s| !s.is_empty());
        let system = rec.model.split('-').next().unwrap_or(&rec.model).to_string();
        let group = match text(rec, &["call_type"]) {
            Some(t) => t.eq_ignore_ascii_case("group"),
            None => is_group(&to),
        };
        let encrypted = rec.fields.iter().any(|(k, v)| match (k.as_str(), v) {
            ("encrypted", Value::Bool(b)) => *b,
            ("encryption", Value::Text(t)) => !t.eq_ignore_ascii_case("none"),
            _ => false,
        });
        let seconds = rec
            .fields
            .iter()
            .find(|(k, _)| k == "seconds")
            .and_then(|(_, v)| v.as_f64())
            .unwrap_or(0.0);

        // A channel is matched loosely: the same talkgroup found by two front
        // ends a few hundred hertz apart is one call, not two rows.
        if let Some(c) = self.seen.iter_mut().find(|c| {
            c.system == system
                && c.to == to
                && c.from == from
                && (c.channel_hz - rec.freq).abs() < rec.channel_hz.max(1.0)
        }) {
            // A gap longer than the hang time is a new conversation on the
            // same group, so the old one keeps its duration rather than
            // stretching across the silence.
            if c.age(at) >= LIVE {
                c.first = at;
                c.seconds = 0.0;
                c.overs = 0;
            }
            c.last = at;
            c.overs += 1;
            c.seconds += seconds;
            c.encrypted = encrypted;
            return true;
        }

        self.seen.push(Call {
            system,
            channel_hz: rec.freq,
            to,
            from,
            group,
            encrypted,
            first: at,
            last: at,
            overs: 1,
            seconds,
        });
        if self.seen.len() > MAX_CALLS {
            self.seen.retain(|c| c.age(at) < FORGET);
            if self.seen.len() > MAX_CALLS {
                let drop = self.seen.len() - MAX_CALLS;
                self.seen.drain(..drop);
            }
        }
        true
    }
}

/// Whether a destination names a group rather than one party.
///
/// A guess, and only used where the system did not say. Broadcast names and
/// numeric talkgroups are groups; anything that looks like a callsign is a
/// party. M17 reflectors carry a module letter after a space, which is what
/// the space here is about.
fn is_group(to: &str) -> bool {
    let t = to.trim();
    if t.eq_ignore_ascii_case("all") || t.eq_ignore_ascii_case("broadcast") {
        return true;
    }
    if t.contains(' ') || t.starts_with('#') {
        return true;
    }
    // A destination that is only digits is a talkgroup number on every system
    // that has them.
    t.chars().all(|c| c.is_ascii_digit())
}

/// The first of these fields the record carries, as text.
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

    #[test]
    fn a_transmission_with_no_destination_is_not_a_call() {
        // Most of what a receiver decodes is a sensor or a pager, and a call
        // list full of thermometers is not a call list.
        let mut c = Calls::new();
        assert!(!c.update(&rec("Fineoffset-WHx080", 433.92e6, &[("temperature_c", Value::Float(8.0))]), t(0)));
        assert!(c.is_empty());
    }

    #[test]
    fn overs_on_one_group_stay_one_call() {
        let mut c = Calls::new();
        let over = rec(
            "M17-Voice",
            433.475e6,
            &[
                ("from", Value::Text("M0ABC".into())),
                ("to", Value::Text("M17-M17 C".into())),
                ("seconds", Value::Float(2.0)),
            ],
        );
        assert!(c.update(&over, t(0)));
        assert!(c.update(&over, t(3)));
        let list = c.active(t(3));
        assert_eq!(list.len(), 1, "two overs are one conversation");
        assert_eq!(list[0].overs, 2);
        assert_eq!(list[0].seconds, 4.0, "airtime adds up across overs");
        assert_eq!(list[0].system, "M17", "every mode of one system shares a row");
        assert!(list[0].group, "a reflector is a group");
        assert!(list[0].live(t(3)));
    }

    #[test]
    fn a_gap_longer_than_the_hang_time_starts_a_new_conversation() {
        // Otherwise a group heard once an hour reads as a call that has been
        // running for an hour.
        let mut c = Calls::new();
        let over = rec(
            "M17-Voice",
            433.475e6,
            &[("from", Value::Text("M0ABC".into())), ("to", Value::Text("ALL".into())),
              ("seconds", Value::Float(2.0))],
        );
        c.update(&over, t(0));
        c.update(&over, t(600));
        let list = c.active(t(600));
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].overs, 1, "the count restarted");
        assert_eq!(list[0].seconds, 2.0);
        assert!(list[0].span() < Duration::from_secs(1));
    }

    #[test]
    fn two_parties_on_one_group_are_two_rows() {
        // Who is talking is the point, so a second caller does not overwrite
        // the first.
        let mut c = Calls::new();
        let to = ("to", Value::Text("91".into()));
        c.update(&rec("DMR-Voice", 446.1e6, &[("from", Value::Text("2345001".into())), to.clone()]), t(0));
        c.update(&rec("DMR-Voice", 446.1e6, &[("from", Value::Text("2345002".into())), to.clone()]), t(1));
        let list = c.active(t(1));
        assert_eq!(list.len(), 2);
        assert_eq!(list[0].from.as_deref(), Some("2345002"), "the newest is first");
        assert!(list[0].group, "a numeric destination is a talkgroup");
    }

    #[test]
    fn a_direct_call_is_told_from_a_group_one() {
        let mut c = Calls::new();
        c.update(
            &rec(
                "M17-Voice",
                433.475e6,
                &[("from", Value::Text("M0ABC".into())), ("to", Value::Text("M0XYZ".into()))],
            ),
            t(0),
        );
        let list = c.active(t(0));
        assert!(!list[0].group, "a callsign destination is one party");
        assert_eq!(list[0].title(), "M0ABC > M0XYZ");
    }

    #[test]
    fn a_system_that_says_what_kind_of_call_it_is_is_believed() {
        // The heuristic reads a numeric destination as a talkgroup, which is
        // wrong for a private call to a radio id. A system that knows says so.
        let mut c = Calls::new();
        c.update(
            &rec(
                "DMR-Voice",
                446.1e6,
                &[
                    ("from", Value::Text("2345001".into())),
                    ("to", Value::Text("2345002".into())),
                    ("call_type", Value::Text("private".into())),
                ],
            ),
            t(0),
        );
        assert!(!c.active(t(0))[0].group);
    }

    #[test]
    fn encrypted_traffic_says_so() {
        let mut c = Calls::new();
        c.update(
            &rec(
                "M17-Voice",
                433.475e6,
                &[
                    ("from", Value::Text("M0ABC".into())),
                    ("to", Value::Text("ALL".into())),
                    ("encryption", Value::Text("aes".into())),
                ],
            ),
            t(0),
        );
        assert!(c.active(t(0))[0].encrypted, "there is no point listening to this one");
    }
}
