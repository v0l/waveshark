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
    /// The coded squelch an analogue channel's traffic is using, as a radio
    /// names it: "141.3" or "D023". For most analogue traffic it is the only
    /// identity there is, since an FM carrier says nothing about who is on
    /// it.
    pub code: Option<String>,
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
    pub heard_s: std::collections::HashMap<common::ConversationKey, f64>,
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
    pub fn hear(&mut self, c: &crate::mix::heard::LiveCall) {
        let key = c.key();
        let same = |k: &Call| k.key().same_conversation(&key);
        let found = self
            .seen
            .iter()
            .enumerate()
            .filter(|(_, k)| same(k))
            .max_by_key(|(_, k)| k.last)
            .map(|(i, _)| i);
        if let Some(i) = found {
            self.said(c, i);
            let k = &mut self.seen[i];
            if k.from.is_none() {
                k.from = c.from.clone();
            }
            // Read off the audio, so it arrives after the row exists, and
            // the newest reading wins: a code changed on the radio has to
            // change on the list.
            if c.code.is_some() {
                k.code = c.code.clone();
            }
            // A gap longer than the hang time is a new conversation on the
            // same group, so the old one keeps its duration rather than
            // stretching across the silence.
            if c.first > k.last && k.age(c.first) >= LIVE {
                k.first = c.first;
                k.seconds = 0.0;
                k.overs = 0;
                k.heard_s.clear();
            }
            k.last = c.last;
            // The first word from the bus on a row the packets made: from
            // here on the bus counts, so what the packets counted is let go
            // rather than added to.
            if !k.by_bus {
                k.by_bus = true;
                k.seconds = 0.0;
                k.overs = 0;
                k.heard_s.clear();
            }
            // The bus reports the over's running total, so what is added is
            // the part not already counted.
            let renamed = c.was.as_ref().and_then(|w| k.heard_s.remove(w));
            let counted = k.heard_s.get(&key).copied().or(renamed).unwrap_or(0.0);
            k.seconds += (c.seconds - counted).max(0.0);
            if c.over {
                k.heard_s.remove(&key);
            } else {
                k.heard_s.insert(key, c.seconds);
            }
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
            encrypted: c.said.as_ref().is_some_and(|o| o.encrypted()),
            cipher: c.said.as_ref().and_then(|o| o.secrecy.cipher()).map(str::to_string),
            codec: c.said.as_ref().and_then(|o| o.codec),
            code: c.code.clone(),
            first: c.first,
            last: c.last,
            overs: u64::from(c.over),
            seconds: c.seconds,
            heard_s: if c.over { Default::default() } else { [(key, c.seconds)].into() },
            by_bus: true,
            transcript: None,
        });
        if self.seen.len() > MAX_CALLS {
            let at = c.last;
            self.seen.retain(|k| k.age(at) < FORGET);
        }
    }

    /// What the system said about the call, from the voice block it was
    /// stated on.
    fn said(&mut self, c: &crate::mix::heard::LiveCall, k: usize) {
        let Some(said) = c.said.as_ref() else { return };
        let row = &mut self.seen[k];
        // A grant names the cipher and the traffic bursts after it say only
        // that they are enciphered, so a name is kept until another replaces
        // it. Clearing on every silent burst took the row from red to blue
        // mid-call.
        if said.codec.is_some() {
            row.codec = said.codec;
        }
        row.encrypted |= said.encrypted();
        if let Some(name) = said.secrecy.cipher() {
            row.cipher = Some(name.to_string());
        }
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
    use crate::mix::heard::LiveCall;
    use common::{Over, Secrecy};

    fn t(secs: u64) -> Instant {
        Instant::now() + Duration::from_secs(secs)
    }

    /// A report from the audio bus, which is where every call comes from.
    ///
    /// The bus hears speech, so what it reports is the channel, whatever
    /// labels the front end put on it and the seconds it has played. What
    /// only a decoder knows arrives on `said`.
    fn bus(system: &str, hz: f64, to: &str, from: Option<&str>, at: Instant) -> LiveCall {
        LiveCall {
            system: system.into(),
            channel_hz: hz,
            to: to.into(),
            from: from.map(str::to_string),
            code: None,
            first: at,
            last: at,
            seconds: 0.0,
            peak: 0.3,
            quiet_s: 0.0,
            over: false,
            said: None,
            was: None,
        }
    }

    /// One over, of `seconds`, reported as it runs and then as it ends.
    fn over(mut c: LiveCall, seconds: f64, calls: &mut Calls) {
        c.seconds = seconds;
        c.last = c.first + Duration::from_secs_f64(seconds);
        calls.hear(&c);
        c.over = true;
        calls.hear(&c);
    }

    #[test]
    fn overs_on_one_group_stay_one_call() {
        let mut c = Calls::new();
        let at = Instant::now();
        over(bus("M17", 433.475e6, "M17-M17 C", Some("M0ABC"), at), 2.0, &mut c);
        over(bus("M17", 433.475e6, "M17-M17 C", Some("M0ABC"), at), 2.0, &mut c);
        let list = c.active(at + Duration::from_secs(2));
        assert_eq!(list.len(), 1, "two overs are one conversation");
        assert_eq!(list[0].overs, 2);
        assert!((list[0].seconds - 4.0).abs() < 1e-6, "airtime {}", list[0].seconds);
        assert_eq!(list[0].system, "M17", "every mode of one system shares a row");
        assert!(list[0].group, "a reflector is a group");
        assert!(list[0].live(at + Duration::from_secs(2)));
    }

    #[test]
    fn a_gap_longer_than_the_hang_time_starts_a_new_conversation() {
        // Otherwise a group heard once an hour reads as a call that has been
        // running for an hour.
        let mut c = Calls::new();
        let at = Instant::now();
        over(bus("M17", 433.475e6, "ALL", Some("M0ABC"), at), 2.0, &mut c);
        let later = at + Duration::from_secs(600);
        over(bus("M17", 433.475e6, "ALL", Some("M0ABC"), later), 2.0, &mut c);
        let list = c.active(later + Duration::from_secs(2));
        assert_eq!(list.len(), 1);
        assert_eq!(list[0].overs, 1, "the count restarted");
        assert!((list[0].seconds - 2.0).abs() < 1e-6);
        assert!(list[0].span() < Duration::from_secs(3));
    }

    /// Analogue identity arrives late, so the first blocks of an over are
    /// heard under the bare channel and the caller fills in afterwards.
    #[test]
    fn traffic_that_does_not_name_its_caller_joins_the_call_that_did() {
        let mut c = Calls::new();
        let at = Instant::now();
        over(bus("TETRA", 391.7e6, "2001", None, at), 1.0, &mut c);
        over(bus("TETRA", 391.7e6, "2001", Some("70311"), at), 1.0, &mut c);
        let list = c.active(at + Duration::from_secs(1));
        assert_eq!(list.len(), 1, "{list:?}");
        assert_eq!(list[0].from.as_deref(), Some("70311"));
    }

    #[test]
    fn two_parties_on_one_group_are_two_rows() {
        // Who is talking is the point, so a second caller does not overwrite
        // the first.
        let mut c = Calls::new();
        let at = Instant::now();
        over(bus("DMR", 446.1e6, "91", Some("2345001"), at), 1.0, &mut c);
        // A second later, so "newest first" has an order to be in.
        over(bus("DMR", 446.1e6, "91", Some("2345002"), at + Duration::from_secs(1)), 1.0, &mut c);
        let list = c.active(at + Duration::from_secs(1));
        assert_eq!(list.len(), 2);
        assert_eq!(list[0].from.as_deref(), Some("2345002"), "the newest is first");
        assert!(list[0].group, "a talkgroup is a group");
    }

    /// All the bus has to go on is how the destination is spelled: a number
    /// or a broadcast name is a group, a callsign is one station.
    #[test]
    fn a_direct_call_is_told_from_a_group_one() {
        let mut c = Calls::new();
        let at = Instant::now();
        over(bus("M17", 433.475e6, "M0XYZ", Some("M0ABC"), at), 1.0, &mut c);
        let list = c.active(at + Duration::from_secs(1));
        assert!(!list[0].group, "a call to one station is one party");
        assert_eq!(list[0].title(), "M0ABC > M0XYZ");
    }

    #[test]
    fn encrypted_traffic_says_so() {
        let mut c = Calls::new();
        let at = Instant::now();
        let mut said = bus("M17", 433.475e6, "ALL", Some("M0ABC"), at);
        said.said = Some(
            Over::new(Some("Codec 2 3200")).protected_by(Secrecy::Encrypted(Some("aes".into()))),
        );
        over(said, 1.0, &mut c);
        let row = &c.active(at + Duration::from_secs(1))[0];
        assert!(row.encrypted, "there is no point listening to this one");
        assert_eq!(row.cipher.as_deref(), Some("aes"));
        assert_eq!(row.codec, Some("Codec 2 3200"));
    }

    #[test]
    fn traffic_that_says_nothing_leaves_the_cipher_standing() {
        // TETRA names the cipher when it grants the channel; the traffic
        // after it says only that it is enciphered, and must not blank the
        // column or take the row from red back to blue.
        let mut c = Calls::new();
        let at = Instant::now();
        let mut grant = bus("TETRA", 393.9e6, "marker 56", None, at);
        grant.said = Some(Over::new(None).protected_by(Secrecy::Encrypted(Some("AIE-3".into()))));
        over(grant, 1.0, &mut c);

        let mut burst = bus("TETRA", 393.9e6, "marker 56", None, at);
        burst.said = Some(Over::new(None).protected_by(Secrecy::Encrypted(None)));
        over(burst, 1.0, &mut c);
        let row = &c.active(at + Duration::from_secs(1))[0];
        assert!(row.encrypted);
        assert_eq!(row.cipher.as_deref(), Some("AIE-3"));

        // And one that says nothing at all about secrecy leaves both alone.
        let mut quiet = bus("TETRA", 393.9e6, "marker 56", None, at);
        quiet.said = Some(Over::new(None));
        over(quiet, 1.0, &mut c);
        let row = &c.active(at + Duration::from_secs(1))[0];
        assert!(row.encrypted);
        assert_eq!(row.cipher.as_deref(), Some("AIE-3"));
    }

    /// The bus counts an over once, however many times it reports it.
    ///
    /// Each report carries the running total for the over, not an increment,
    /// so a row adds the part it has not already counted. Adding each report
    /// whole made a three second over read as eight.
    #[test]
    fn a_running_over_is_counted_once() {
        let mut c = Calls::new();
        let at = Instant::now();
        let mut live = bus("M17", 433.475e6, "BROADCAST", Some("M0ABC"), at);
        live.said = Some(Over::new(Some("Codec 2 3200")));
        for (seconds, ended) in [(1.0, false), (2.9, false), (2.9, true)] {
            live.seconds = seconds;
            live.over = ended;
            live.last = at + Duration::from_secs_f64(seconds);
            c.hear(&live);
        }
        let rows = c.active(at + Duration::from_secs(3));
        assert_eq!(rows.len(), 1, "one over made more than one row");
        assert_eq!(rows[0].overs, 1);
        assert!((rows[0].seconds - 2.9).abs() < 1e-6, "airtime {}", rows[0].seconds);
        assert_eq!(rows[0].codec, Some("Codec 2 3200"), "what the system said is kept");
    }

    #[test]
    fn two_keys_on_one_row_count_only_their_own_airtime() {
        let mut c = Calls::new();
        let at = Instant::now();
        let mut stale = bus("Audio", 405_147_100.0, "CH22", None, at);
        stale.seconds = 100.0;
        let mut live = bus("Audio", 405_147_400.0, "CH22", None, at);
        for frame in 1..=10 {
            c.hear(&stale);
            live.seconds = 0.01 * f64::from(frame);
            live.last = at + Duration::from_millis(10 * frame as u64);
            c.hear(&live);
        }
        let rows = c.active(at + Duration::from_secs(1));
        assert_eq!(rows.len(), 1, "{rows:?}");
        assert!((rows[0].seconds - 100.1).abs() < 1e-9, "airtime {}", rows[0].seconds);
    }

    #[test]
    fn a_call_renamed_mid_over_keeps_what_was_counted() {
        let mut c = Calls::new();
        let at = Instant::now();
        let mut anonymous = bus("Audio", 145.5e6, "CH1", None, at);
        anonymous.seconds = 0.5;
        c.hear(&anonymous);
        let mut named = bus("Audio", 145.5e6, "CH1", Some("123"), at);
        named.was = Some(anonymous.key());
        named.seconds = 0.6;
        c.hear(&named);
        named.was = None;
        named.seconds = 0.8;
        named.over = true;
        c.hear(&named);
        let rows = c.active(at + Duration::from_secs(1));
        assert_eq!(rows.len(), 1, "{rows:?}");
        assert_eq!(rows[0].overs, 1);
        assert!((rows[0].seconds - 0.8).abs() < 1e-9, "airtime {}", rows[0].seconds);
        assert_eq!(rows[0].heard_s.len(), 0, "an ended over left a watermark");
    }

    /// A call and the speech heard on it have to agree on one key, or the
    /// row shows no transcript and the button into it is never offered. The
    /// two ends build it from different things: the call from the bus's
    /// report, the transcriber from the voice block the front end put on it.
    #[test]
    fn a_call_and_its_speech_are_the_same_conversation() {
        let mut c = Calls::new();
        let at = Instant::now();
        over(bus("DMR", 435.0e6, "9", Some("1234567"), at), 1.0, &mut c);
        let call = &c.active(at + Duration::from_secs(1))[0];
        let spoken = common::ConversationKey::of(&common::Voice {
            system: "DMR",
            channel_hz: 435.0e6,
            to: Some("9".into()),
            from: Some("1234567".into()),
            code: None,
            over: None,
            rate: 8_000.0,
            channels: 1,
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
        assert_eq!(
            c.active(at + Duration::from_secs(1))[0].transcript.as_deref(),
            Some("go ahead")
        );
    }
}
