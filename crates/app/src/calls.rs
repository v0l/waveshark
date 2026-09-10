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
//! Nothing here knows what M17 or DMR is. A call is assembled from what a
//! decode already states in types: [`common::Link`] for who it was between,
//! and [`common::Airtime`] for how long it ran, what protects it and which
//! vocoder it is in. A decoder added later joins the view by filling those
//! in, not by spelling field names this file happens to look for.
//!
//! Whether the party called is a group is the decoder's statement too, since
//! a trunked network distinguishes a talkgroup from a private call outright.
//! Only the audio bus, which hears speech and not link control, has to guess
//! from the destination: see [`is_group`].
//!
//! # Voice only, and the decoder has to say so
//!
//! This table is for people talking. A destination is not enough to earn a
//! row: an APRS frame has one, so does a TETRA short data message and so
//! does a MAC header addressed to a radio that is registering. Each of those
//! put rows on the list that no operator could listen to. So a decode has to
//! assert `Airtime::voice`, and only a decoder that knows the transmission
//! carries speech sets it: M17 from the link setup's data type, TETRA from
//! the circuit mode in the basic service information, or from traffic on a
//! channel a call was granted. Everything else stays in the packet log where
//! it belongs.

use crate::radio::DecodeRecord;
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
    /// How, as the system names it: "AIE-3", "E2E", "privacy". `None` when
    /// the decode said only that it was enciphered.
    pub cipher: Option<String>,
    /// The vocoder the speech is in, as the front end names it: "AMBE+2
    /// 2450", "Codec 2 3200", "ACELP 4.6k". `None` when it did not say.
    pub codec: Option<&'static str>,
    pub first: Instant,
    pub last: Instant,
    /// Separate keyings of the microphone, not packets.
    pub overs: u64,
    /// Airtime in seconds, where the protocol says how long a transmission
    /// ran. Zero where it does not.
    pub seconds: f64,
    /// What was said in the most recent over, when something transcribed it.
    ///
    /// The last one rather than all of them: this is a scanner's list of who
    /// is on the air, and the row is one line. The whole text of every over
    /// stays in the transcript beside the audio it was read from.
    pub transcript: Option<String>,
    /// Seconds of the over in progress the bus has already reported, so the
    /// running total it sends is folded in once rather than summed again on
    /// every block.
    pub heard_s: f64,
    /// Whether the audio bus has reported this call. Once it has, the bus is
    /// what counts overs and airtime: a decoder's packets say the same over
    /// happened, and counting both listed every digital over twice.
    pub by_bus: bool,
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
    #[cfg_attr(not(test), allow(dead_code))]
    pub fn span(&self) -> Duration {
        self.last.saturating_duration_since(self.first)
    }

    /// The conversation this call is, as the bus and the transcriber key it.
    pub fn key(&self) -> common::ConversationKey {
        common::ConversationKey::new(&self.system, self.channel_hz)
            .to((!self.to.is_empty()).then(|| self.to.clone()))
            .from(self.from.clone())
    }

    /// The label a list shows: the group, with the caller beside it.
    #[cfg_attr(not(test), allow(dead_code))]
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

    /// Give every call the text the transcriber has for it.
    ///
    /// The transcriber writes down conversations, keyed by who was talking on
    /// what, and the call list keeps calls. They meet on the key rather than
    /// on a wire between them: neither has to know the other exists, and a
    /// view that wants the whole conversation asks the log for the key.
    pub fn read_transcripts(&mut self, said: &[crate::transcripts::Utterance]) {
        for c in &mut self.seen {
            let key = c.key();
            if let Some(u) = said.iter().rev().find(|u| u.key == key) {
                c.transcript = Some(u.text.clone());
            }
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }

    pub fn len(&self) -> usize {
        self.seen.len()
    }

    pub fn clear(&mut self) {
        self.seen.clear();
    }

    /// Fold in a conversation the audio bus is hearing.
    ///
    /// This is where every analogue call comes from, and where a digital
    /// one is marked live: the bus is the first stop for every demodulator's
    /// audio, so it is the one place that knows who is talking right now.
    /// What only a decoder knows, the cipher, the codec, the kind of call,
    /// arrives by [`Self::update`] from the packet side and lands on the same
    /// row, because the two are keyed the same way.
    pub fn hear(&mut self, c: &crate::audiobus::LiveCall) {
        let key = c.key();
        let same = |k: &Call| k.key().same_conversation(&key);
        let found = self
            .seen
            .iter()
            .enumerate()
            .filter(|(_, k)| same(k))
            .max_by_key(|(_, k)| k.last)
            .map(|(i, _)| i);
        if let Some(k) = found.map(|i| &mut self.seen[i]) {
            if k.from.is_none() {
                k.from = c.from.clone();
            }
            // A gap longer than the hang time is a new conversation on the
            // same group, so the old one keeps its duration rather than
            // stretching across the silence.
            if c.first > k.last && k.age(c.first) >= LIVE {
                k.first = c.first;
                k.seconds = 0.0;
                k.overs = 0;
                k.heard_s = 0.0;
            }
            k.last = c.last;
            // The first word from the bus on a row the packets made: from
            // here on the bus counts, so what the packets counted is let go
            // rather than added to.
            if !k.by_bus {
                k.by_bus = true;
                k.seconds = 0.0;
                k.overs = 0;
                k.heard_s = 0.0;
            }
            // The bus reports the over's running total, so what is added is
            // the part not already counted.
            let more = (c.seconds - k.heard_s).max(0.0);
            k.seconds += more;
            k.heard_s = if c.over { 0.0 } else { c.seconds };
            if c.over {
                k.overs += 1;
            }
            return;
        }
        self.seen.push(Call {
            system: c.system.clone(),
            channel_hz: c.channel_hz,
            to: c.to.clone(),
            from: c.from.clone(),
            group: is_group(&c.to),
            encrypted: false,
            cipher: None,
            codec: None,
            first: c.first,
            last: c.last,
            overs: u64::from(c.over),
            seconds: c.seconds,
            heard_s: if c.over { 0.0 } else { c.seconds },
            by_bus: true,
            transcript: None,
        });
        if self.seen.len() > MAX_CALLS {
            let at = c.last;
            self.seen.retain(|k| k.age(at) < FORGET);
        }
    }

    /// Fold one decode in, if it is a call at all.
    ///
    /// Returns whether it was. Anything the decoder did not say is voice is
    /// somebody else's business: a sensor reading, a pager message, an
    /// aircraft, a data call, a radio registering on a trunked network.
    pub fn update(&mut self, rec: &DecodeRecord, at: Instant) -> bool {
        let Some(airtime) = rec.airtime.as_ref().filter(|a| a.voice) else {
            return false;
        };
        let Some(party) = rec.link.as_ref().and_then(|l| l.to.as_ref()) else {
            return false;
        };
        let to = party.label().to_string();
        if to.is_empty() {
            return false;
        }
        let from = rec
            .link
            .as_ref()
            .and_then(|l| l.from.as_ref())
            .map(|p| p.label().to_string())
            .filter(|s| !s.is_empty());
        let system = rec.system().to_string();
        // The decoder said which kind of party it named, so nothing here has
        // to guess from how the destination is spelled.
        let group = matches!(
            party.kind,
            pipeline::event::PartyKind::Group | pipeline::event::PartyKind::Broadcast
        );
        let cipher = airtime.secrecy.cipher().map(|c| c.to_string());
        let codec = airtime.codec;
        let encrypted = airtime.secrecy.encrypted();
        let seconds = airtime.seconds;
        // A decode that says the transmission is still running is not an
        // over yet; the one that says it ended is.
        let live = airtime.live;

        // A channel is matched loosely: the same talkgroup found by two front
        // ends a few hundred hertz apart is one call, not two rows. And a
        // caller is matched only where both sides name one: on TETRA the
        // grant names who is talking and the traffic that follows does
        // not, and treating those as two callers listed every call twice,
        // once with a name and once without. The key here names nobody
        // talking for that reason; the caller is compared beside it.
        let channel = common::ConversationKey::new(&system, rec.freq).to(Some(to.clone()));
        let same = |c: &Call| c.key().same_conversation(&channel);
        let found = self.seen.iter().position(|c| same(c) && c.from == from).or_else(|| {
            // The one most recently heard, since that is the call the
            // unnamed traffic belongs to.
            self.seen
                .iter()
                .enumerate()
                .filter(|(_, c)| {
                    same(c) && (c.from.is_none() || from.is_none()) && c.age(at) < LIVE
                })
                .max_by_key(|(_, c)| c.last)
                .map(|(i, _)| i)
        });
        if let Some(c) = found.map(|i| &mut self.seen[i]) {
            if c.from.is_none() {
                c.from = from;
            }
            // A gap longer than the hang time is a new conversation on the
            // same group, so the old one keeps its duration rather than
            // stretching across the silence.
            if c.age(at) >= LIVE {
                c.first = at;
                c.seconds = 0.0;
                c.overs = 0;
            }
            c.last = at;
            // The bus counts overs and airtime where it hears the call; the
            // decoder's own count is for a call nothing is playing, such as
            // one enciphered or in a vocoder this build does not have.
            if !c.by_bus {
                if !live {
                    c.overs += 1;
                }
                c.seconds += seconds;
            }
            // Only a decode that says something about the cipher may change
            // this. TETRA names it in the grant and not in the traffic that
            // follows, so a verdict from every record flipped the call back
            // to clear while it still carried the cipher's name, and a
            // traffic burst that says it is enciphered without naming it
            // must not take the name away either.
            if airtime.secrecy != common::Secrecy::Unsaid {
                c.encrypted = encrypted;
            }
            if cipher.is_some() {
                c.cipher = cipher;
            }
            if codec.is_some() {
                c.codec = codec;
            }
            return true;
        }

        self.seen.push(Call {
            system,
            channel_hz: rec.freq,
            to,
            from,
            group,
            encrypted,
            cipher,
            codec,
            first: at,
            last: at,
            overs: u64::from(!live),
            seconds,
            heard_s: 0.0,
            by_bus: false,
            transcript: None,
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
/// A guess, and only for a call heard on the audio bus, where all there is
/// to go on is what the speech was labelled with. Broadcast names and
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

#[cfg(test)]
mod tests {
    use super::*;
    use common::{Airtime, Secrecy, Value};
    use pipeline::event::{Link, Party};

    fn rec(model: &'static str, freq: f64, fields: &[(&str, Value)]) -> DecodeRecord {
        let mut r = DecodeRecord::for_test(freq, model);
        r.channel_hz = 12_500.0;
        r.fields = fields.iter().map(|(k, v)| (k.to_string(), v.clone())).collect();
        r
    }

    /// A decode from a mode that knows it is carrying speech: the parties as
    /// the decoder named them and the airtime as it measured it, which is
    /// all of it the call list reads.
    fn voice(model: &'static str, freq: f64, link: Link, airtime: Airtime) -> DecodeRecord {
        let mut r = DecodeRecord::for_test(freq, model);
        r.channel_hz = 12_500.0;
        r.link = Some(link);
        r.airtime = Some(airtime);
        r
    }

    /// An over that has ended, of `seconds`, with nothing said about a
    /// cipher.
    fn over(seconds: f64) -> Airtime {
        Airtime { seconds, voice: true, live: false, ..Default::default() }
    }

    fn t(secs: u64) -> Instant {
        Instant::now() + Duration::from_secs(secs)
    }

    #[test]
    fn a_transmission_with_no_destination_is_not_a_call() {
        // Most of what a receiver decodes is a sensor or a pager, and a call
        // list full of thermometers is not a call list.
        let mut c = Calls::new();
        assert!(!c.update(
            &rec("Fineoffset-WHx080", 433.92e6, &[("temperature_c", Value::Float(8.0))]),
            t(0)
        ));
        assert!(c.is_empty());
    }

    #[test]
    fn a_destination_alone_is_not_a_call() {
        // An APRS frame is addressed, a TETRA short data message is
        // addressed, and a MAC header naming a radio that is registering is
        // addressed. None of them is somebody talking, and a list of them is
        // not a call list. Only a decoder that knows there is speech says so,
        // and it says it in the airtime rather than in a field.
        let mut c = Calls::new();
        let addressed = [
            ("APRS", 144.8e6, Link::between(Party::unit("M0ABC-9"), Party::group("APRS"))),
            ("TETRA-SDS", 391.1e6, Link::between(Party::unit("70311"), Party::unit("10223295"))),
            ("TETRA-Call", 391.1e6, Link { from: None, to: Some(Party::unit("10223295")) }),
        ];
        for (model, hz, link) in addressed {
            let mut r = DecodeRecord::for_test(hz, model);
            r.link = Some(link);
            assert!(!c.update(&r, t(0)), "{} earned a row", r.protocol());
        }
        assert!(c.is_empty());
    }

    #[test]
    fn overs_on_one_group_stay_one_call() {
        let mut c = Calls::new();
        let call = voice(
            "M17-Voice",
            433.475e6,
            Link::between(Party::unit("M0ABC"), Party::group("M17-M17 C")),
            over(2.0),
        );
        assert!(c.update(&call, t(0)));
        assert!(c.update(&call, t(3)));
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
        let call = voice(
            "M17-Voice",
            433.475e6,
            Link::between(Party::unit("M0ABC"), Party::group("ALL")),
            over(2.0),
        );
        c.update(&call, t(0));
        c.update(&call, t(600));
        let list = c.active(t(600));
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].overs, 1, "the count restarted");
        assert_eq!(list[0].seconds, 2.0);
        assert!(list[0].span() < Duration::from_secs(1));
    }

    #[test]
    fn traffic_that_does_not_name_its_caller_joins_the_grant_that_did() {
        // On TETRA the grant says who is talking and the bursts that follow
        // say nothing, and on a network that never grants by name the bursts
        // are all there is. Either way it is one call, with the caller
        // filled in from whichever row carried it.
        let mut c = Calls::new();
        let unnamed = || Link { from: None, to: Some(Party::group("2001")) };
        let named = |id: &str| Link::between(Party::unit(id), Party::group("2001"));
        c.update(&voice("TETRA-Voice", 391.7e6, unnamed(), over(0.0)), t(0));
        c.update(&voice("TETRA-Call", 391.7e6, named("70311"), over(0.0)), t(1));
        c.update(&voice("TETRA-Voice", 391.7e6, unnamed(), over(0.0)), t(2));
        let list = c.active(t(2));
        assert_eq!(list.len(), 1, "{list:?}");
        assert_eq!(list[0].from.as_deref(), Some("70311"));
        // A different named caller is still a different row.
        c.update(&voice("TETRA-Call", 391.7e6, named("70312"), over(0.0)), t(3));
        assert_eq!(c.active(t(3)).len(), 2);
    }

    #[test]
    fn two_parties_on_one_group_are_two_rows() {
        // Who is talking is the point, so a second caller does not overwrite
        // the first.
        let mut c = Calls::new();
        let call = |from: &str| {
            voice(
                "DMR-Voice",
                446.1e6,
                Link::between(Party::unit(from), Party::group("91")),
                over(0.0),
            )
        };
        c.update(&call("2345001"), t(0));
        c.update(&call("2345002"), t(1));
        let list = c.active(t(1));
        assert_eq!(list.len(), 2);
        assert_eq!(list[0].from.as_deref(), Some("2345002"), "the newest is first");
        assert!(list[0].group, "a talkgroup is a group");
    }

    #[test]
    fn a_direct_call_is_told_from_a_group_one() {
        let mut c = Calls::new();
        c.update(
            &voice(
                "M17-Voice",
                433.475e6,
                Link::between(Party::unit("M0ABC"), Party::unit("M0XYZ")),
                over(0.0),
            ),
            t(0),
        );
        let list = c.active(t(0));
        assert!(!list[0].group, "a call to one station is one party");
        assert_eq!(list[0].title(), "M0ABC > M0XYZ");
    }

    #[test]
    fn a_system_that_says_what_kind_of_call_it_is_is_believed() {
        // Read off how the destination is spelled, a numeric one is a
        // talkgroup, which is wrong for a private call to a radio id. The
        // decoder named the kind of party, so nothing here guesses.
        let mut c = Calls::new();
        c.update(
            &voice(
                "DMR-Voice",
                446.1e6,
                Link::between(Party::unit("2345001"), Party::unit("2345002")),
                over(0.0),
            ),
            t(0),
        );
        assert!(!c.active(t(0))[0].group);
    }

    #[test]
    fn encrypted_traffic_says_so() {
        let mut c = Calls::new();
        c.update(
            &voice(
                "M17-Voice",
                433.475e6,
                Link::between(Party::unit("M0ABC"), Party::group("ALL")),
                Airtime { secrecy: Secrecy::Encrypted(Some("aes".into())), ..over(0.0) },
            ),
            t(0),
        );
        assert!(c.active(t(0))[0].encrypted, "there is no point listening to this one");
    }

    #[test]
    fn traffic_that_says_nothing_leaves_the_cipher_standing() {
        // TETRA names the cipher in the grant; the traffic frames after it
        // say nothing either way, and they must not clear it.
        let mut c = Calls::new();
        let to = || Link { from: None, to: Some(Party::group("marker 56")) };
        let grant = voice(
            "TETRA-Voice",
            393.9e6,
            to(),
            Airtime { secrecy: Secrecy::Encrypted(Some("AIE-3".into())), ..over(0.0) },
        );
        c.update(&grant, t(0));
        let traffic = voice("TETRA-Voice", 393.9e6, to(), over(0.0));
        assert_eq!(traffic.airtime.as_ref().unwrap().secrecy, Secrecy::Unsaid);
        c.update(&traffic, t(1));
        let call = &c.active(t(1))[0];
        assert!(call.encrypted, "the row would have gone from red to blue");
        assert_eq!(call.cipher.as_deref(), Some("AIE-3"));

        // And a burst that says it is enciphered without naming what with,
        // which is all a traffic burst can say, keeps the name the grant
        // gave rather than blanking the column.
        let burst = voice(
            "TETRA-Voice",
            393.9e6,
            to(),
            Airtime { secrecy: Secrecy::Encrypted(None), ..over(0.0) },
        );
        c.update(&burst, t(2));
        let call = &c.active(t(2))[0];
        assert!(call.encrypted);
        assert_eq!(call.cipher.as_deref(), Some("AIE-3"));
    }

    /// A digital over arrives twice: as the decoder's packet, with what only
    /// the decoder knows, and as speech on the audio bus, with when it was
    /// actually heard. One row, one over, and the bus's count is the one
    /// kept, because it is the count of what was played.
    #[test]
    fn a_digital_over_is_one_row_from_both_sides() {
        let mut c = Calls::new();
        let at = Instant::now();
        let packet = voice(
            "M17-Voice",
            433.475e6,
            Link::between(Party::unit("M0ABC"), Party::group("BROADCAST")),
            Airtime { codec: Some("Codec 2 3200"), ..over(3.0) },
        );
        c.update(&packet, at);
        let live = |seconds: f64, over: bool| crate::audiobus::LiveCall {
            system: "M17".into(),
            channel_hz: 433.475e6,
            to: "BROADCAST".into(),
            from: Some("M0ABC".into()),
            first: at,
            last: at + Duration::from_secs_f64(seconds),
            seconds,
            peak: 0.3,
            quiet_s: 0.0,
            over,
        };
        c.hear(&live(1.0, false));
        c.hear(&live(2.9, false));
        c.hear(&live(2.9, true));
        let rows = c.active(at + Duration::from_secs(3));
        assert_eq!(rows.len(), 1, "two sides of one over made two rows");
        assert_eq!(rows[0].overs, 1, "the over was counted from both sides");
        assert!((rows[0].seconds - 2.9).abs() < 1e-6, "airtime {}", rows[0].seconds);
        assert_eq!(rows[0].codec, Some("Codec 2 3200"), "what the decoder said is kept");
        // And the next packet for the same call does not add to what the
        // bus is counting.
        c.update(&packet, at + Duration::from_secs(1));
        assert_eq!(c.active(at + Duration::from_secs(3))[0].overs, 1);
    }

    /// A call and the speech heard on it have to agree on one key, or the
    /// row shows no transcript and the button into it is never offered. The
    /// two ends build it from different things: the call from what a decoder
    /// published, the transcriber from the voice block the front end put on
    /// the bus.
    #[test]
    fn a_call_and_its_speech_are_the_same_conversation() {
        let mut c = Calls::new();
        c.update(
            &voice(
                "DMR-Voice",
                435.0e6,
                Link::between(Party::unit("1234567"), Party::group("9")),
                over(0.0),
            ),
            t(0),
        );
        let call = &c.active(t(0))[0];
        let spoken = common::ConversationKey::of(&common::Voice {
            system: "DMR",
            channel_hz: 435.0e6,
            to: Some("9".into()),
            from: Some("1234567".into()),
            rate: 8_000.0,
            pcm: vec![0.2; 8],
        });
        assert_eq!(call.key(), spoken);
        assert_eq!(call.key().to_string(), "DMR:435000000:9:1234567");

        // And what the transcriber read reaches the row.
        let said = [crate::transcripts::Utterance {
            key: spoken,
            at: Instant::now(),
            seconds: 1.0,
            text: "go ahead".into(),
            settled: true,
            confidence: -0.3,
            credible: true,
        }];
        c.read_transcripts(&said);
        assert_eq!(c.active(t(0))[0].transcript.as_deref(), Some("go ahead"));
    }
}
