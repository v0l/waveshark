//! Who is talking to whom, on every protocol at once, and what passed between
//! them.
//!
//! Wireshark's conversation list and follow-stream, for radio. A link is a
//! pair of ends on one protocol, and following it is every packet that link
//! carried in the order it arrived, with the payload where the protocol gives
//! one in the clear.
//!
//! # Fed from the fields, not from the protocols
//!
//! Nothing here knows what BLE or DMR is. A link is assembled from `from` and
//! `to`, which every decode that names its parties now carries: a decoder
//! added later joins the directory by naming its fields the same way, which
//! is the bargain the map makes with positions and the call list makes with
//! `seconds`. A transmission that names only one end, which is most
//! telemetry, is a link from that end to nobody, and that is worth a row: a
//! meter, a beacon and an advertiser are all things somebody wants to see the
//! history of.
//!
//! # Not a byte stream, usually
//!
//! Wireshark can concatenate a TCP conversation because TCP is a byte stream.
//! Radio mostly is not: a DMR call is voice bursts, an advertisement is a
//! beacon repeated every two seconds, a meter reading is one frame with a
//! CRC. So following a link gives the packets in order with their own fields,
//! and the payload of each where there is one to show. Where a protocol
//! really does carry bytes, they are on the packet that carried them and can
//! be read straight down the list.
//!
//! # The same directory from live packets or from the log
//!
//! Every row comes from a [`DecodeRecord`], so the directory is the same
//! whether the records arrive from the bus or from `packetlog::read`. That is
//! what lets the pane show the past without the receiver holding a day of
//! packets in memory.

use crate::radio::DecodeRecord;
use common::Value;
use std::time::{Duration, Instant};

/// How long after its last packet a link is still counted as live.
pub const LIVE: Duration = Duration::from_secs(30);

/// How long a link stays in the directory after its last packet.
const FORGET: Duration = Duration::from_secs(6 * 60 * 60);

/// Beyond this many links the least recently heard are dropped. A busy
/// 2.4 GHz band produces a link per advertiser, and a flat has hundreds.
const MAX_LINKS: usize = 4096;

/// Packets kept per link for the follow view.
///
/// Enough to read a conversation back, bounded because a beacon every two
/// seconds is forty thousand packets a day and the directory is a view, not
/// the log. The log has all of them.
const MAX_PACKETS: usize = 512;

/// What a link is between: an end that named itself, or nobody.
///
/// A broadcast is not a party. Keeping it as its own variant rather than as
/// the string "broadcast" means a device that really is called that cannot
/// merge with every beacon on the band.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub enum End {
    Named(String),
    Broadcast,
    Unknown,
}

impl End {
    fn parse(s: Option<String>) -> Self {
        match s.as_deref().map(str::trim) {
            None | Some("") => End::Unknown,
            Some(t)
                if t.eq_ignore_ascii_case("broadcast")
                    || t.eq_ignore_ascii_case("everyone")
                    || t.eq_ignore_ascii_case("all")
                    || t == "^all"
                    || t == "ffffffff" =>
            {
                End::Broadcast
            }
            Some(t) => End::Named(t.to_string()),
        }
    }

    pub fn label(&self) -> &str {
        match self {
            End::Named(s) => s,
            End::Broadcast => "broadcast",
            End::Unknown => "-",
        }
    }

    pub fn is_named(&self) -> bool {
        matches!(self, End::Named(_))
    }
}

/// One packet as the follow view shows it.
#[derive(Clone, Debug)]
pub struct Moment {
    pub at: Instant,
    /// The protocol as the decode named it, which is finer than the link's
    /// system: `DMR-Voice` and `DMR-Header` share a link.
    pub protocol: String,
    pub channel_hz: f64,
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    /// The decode's own summary line, which is what the packet list shows.
    pub detail: String,
    /// What the protocol carried in the clear, when it carried anything.
    pub text: Option<String>,
    pub bytes: usize,
    pub crc: Option<bool>,
}

/// One link: two ends on one system, and everything heard between them.
#[derive(Clone, Debug)]
pub struct Link {
    /// The system, from the protocol name: `BLE-Adv` becomes `BLE`, so every
    /// kind of packet one system sends shares a link.
    pub system: String,
    pub from: End,
    pub to: End,
    /// Where it was last heard, in hertz.
    pub channel_hz: f64,
    pub first: Instant,
    pub last: Instant,
    pub packets: u64,
    /// Payload bytes carried, which is what says whether a link is chatter or
    /// a transfer.
    pub bytes: u64,
    /// Strongest and most recent level, for deciding what is near.
    pub best_rssi_dbfs: f32,
    pub last_rssi_dbfs: f32,
    /// Whether anything on this link failed its integrity check, which a
    /// directory has to show: a link built from bad frames is not a link.
    pub crc_failures: u64,
    /// The recent packets, oldest first.
    pub moments: std::collections::VecDeque<Moment>,
}

impl Link {
    pub fn age(&self, now: Instant) -> Duration {
        now.saturating_duration_since(self.last)
    }

    pub fn live(&self, now: Instant) -> bool {
        self.age(now) < LIVE
    }

    /// The span from the first packet to the last.
    pub fn duration(&self) -> Duration {
        self.last.saturating_duration_since(self.first)
    }

    /// `BLE  E8:31:CD:0A:F5:3A -> broadcast`, which is what a row and a
    /// window title both want.
    pub fn title(&self) -> String {
        format!("{} {} -> {}", self.system, self.from.label(), self.to.label())
    }
}

#[derive(Default)]
pub struct Links {
    seen: Vec<Link>,
}

impl Links {
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold one decode in, and say whether it belonged to a link at all.
    ///
    /// A decode with neither end named is not a link: an unknown burst has a
    /// frequency and a shape and nobody to attribute it to.
    pub fn update(&mut self, rec: &DecodeRecord, at: Instant) -> bool {
        let from = End::parse(text(rec, &["from", "src", "source", "radio_id", "sender"]));
        let to = End::parse(text(rec, &["to", "dst", "destination", "talkgroup", "addressee"]));
        if !from.is_named() && !to.is_named() {
            return false;
        }
        let system = rec.model.split('-').next().unwrap_or(&rec.model).to_string();
        let moment = Moment {
            at,
            protocol: rec.model.clone(),
            channel_hz: rec.freq,
            rssi_dbfs: rec.rssi_dbfs,
            snr_db: rec.snr_db,
            detail: rec.detail.clone(),
            text: text(rec, &["text", "message", "sms", "name"]).filter(|t| !t.trim().is_empty()),
            bytes: rec.bytes.len(),
            crc: rec.crc,
        };
        let found = self
            .seen
            .iter_mut()
            .find(|l| l.system == system && l.from == from && l.to == to);
        match found {
            Some(l) => {
                l.last = at;
                l.channel_hz = rec.freq;
                l.packets += 1;
                l.bytes += rec.bytes.len() as u64;
                l.last_rssi_dbfs = rec.rssi_dbfs;
                if rec.rssi_dbfs > l.best_rssi_dbfs || l.best_rssi_dbfs.is_nan() {
                    l.best_rssi_dbfs = rec.rssi_dbfs;
                }
                if rec.crc == Some(false) {
                    l.crc_failures += 1;
                }
                l.moments.push_back(moment);
                while l.moments.len() > MAX_PACKETS {
                    l.moments.pop_front();
                }
            }
            None => {
                let mut moments = std::collections::VecDeque::with_capacity(8);
                moments.push_back(moment);
                self.seen.push(Link {
                    system,
                    from,
                    to,
                    channel_hz: rec.freq,
                    first: at,
                    last: at,
                    packets: 1,
                    bytes: rec.bytes.len() as u64,
                    best_rssi_dbfs: rec.rssi_dbfs,
                    last_rssi_dbfs: rec.rssi_dbfs,
                    crc_failures: u64::from(rec.crc == Some(false)),
                    moments,
                });
            }
        }
        self.forget(at);
        true
    }

    /// Drop what is too old to be interesting, and cap the rest.
    fn forget(&mut self, now: Instant) {
        self.seen.retain(|l| l.age(now) < FORGET);
        if self.seen.len() > MAX_LINKS {
            self.seen.sort_by(|a, b| b.last.cmp(&a.last));
            self.seen.truncate(MAX_LINKS);
        }
    }

    /// Every link, most recently heard first.
    pub fn active(&self, now: Instant) -> Vec<&Link> {
        let mut v: Vec<&Link> = self.seen.iter().filter(|l| l.age(now) < FORGET).collect();
        v.sort_by(|a, b| b.last.cmp(&a.last));
        v
    }

    /// The links one end takes part in, whichever end it is: what a directory
    /// is for is picking a device and seeing everything it has said.
    pub fn involving<'a>(&'a self, who: &str, now: Instant) -> Vec<&'a Link> {
        self.active(now)
            .into_iter()
            .filter(|l| l.from.label() == who || l.to.label() == who)
            .collect()
    }

    pub fn is_empty(&self) -> bool {
        self.seen.is_empty()
    }

    pub fn len(&self) -> usize {
        self.seen.len()
    }

    /// Take in another directory's links, merging what is already here.
    ///
    /// A link heard live and again in the log is one link: the counts add and
    /// the packets are interleaved by time, so following it after a load
    /// reads in the order things happened.
    pub fn absorb(&mut self, other: Links) {
        for l in other.seen {
            match self
                .seen
                .iter_mut()
                .find(|m| m.system == l.system && m.from == l.from && m.to == l.to)
            {
                Some(m) => {
                    m.first = m.first.min(l.first);
                    m.last = m.last.max(l.last);
                    m.packets += l.packets;
                    m.bytes += l.bytes;
                    m.crc_failures += l.crc_failures;
                    if l.best_rssi_dbfs > m.best_rssi_dbfs || m.best_rssi_dbfs.is_nan() {
                        m.best_rssi_dbfs = l.best_rssi_dbfs;
                    }
                    let mut all: Vec<Moment> =
                        m.moments.iter().cloned().chain(l.moments).collect();
                    all.sort_by_key(|x| x.at);
                    let from = all.len().saturating_sub(MAX_PACKETS);
                    m.moments = all[from..].iter().cloned().collect();
                }
                None => self.seen.push(l),
            }
        }
        self.forget(Instant::now());
    }

    /// Rebuild from a slice of records, which is how a day of the packet log
    /// becomes a directory.
    pub fn from_records<'a>(recs: impl IntoIterator<Item = &'a DecodeRecord>) -> Self {
        let mut links = Self::new();
        for r in recs {
            links.update(r, r.at);
        }
        links
    }
}

/// Build a directory from a packet log file, decoding as the bus does.
///
/// The stamps are the log's own: a link followed an hour after the fact shows
/// what happened then, spaced as it happened, not squashed into the moment it
/// was read back. `Instant` cannot be built from a wall clock, so the epoch
/// microseconds are laid out relative to now, which keeps the spacing and the
/// order and loses only the absolute clock, and nothing in the view uses one.
pub fn from_log(path: &std::path::Path) -> std::io::Result<Links> {
    let packets = crate::packetlog::read(path)?;
    let mut node = nodes::PacketDecodeNode::default();
    let now = Instant::now();
    let last_us = packets.iter().map(|p| p.at_us).max().unwrap_or(0);
    let mut links = Links::new();
    // One packet at a time, because a decode is stamped from the packet that
    // produced it and a batch would collapse a day into one instant.
    for p in &packets {
        node.decode_all(std::slice::from_ref(p));
        let ago = Duration::from_micros(last_us.saturating_sub(p.at_us));
        let at = now.checked_sub(ago).unwrap_or(now);
        for d in node.hits() {
            let rec = crate::chain::record_of(at, d);
            links.update(&rec, at);
        }
    }
    Ok(links)
}

/// Every log segment in a folder, newest last, for a directory that wants
/// more than the file being written.
pub fn segments(dir: &std::path::Path) -> Vec<std::path::PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else { return Vec::new() };
    let mut v: Vec<std::path::PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "wspkt"))
        .collect();
    v.sort();
    v
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

    fn rec(model: &str, hz: f64, fields: &[(&str, Value)]) -> DecodeRecord {
        let mut r = DecodeRecord::for_test(hz, model);
        r.rssi_dbfs = -40.0;
        r.snr_db = 20.0;
        r.bytes = vec![0; 10];
        r.crc = Some(true);
        r.fields = fields.iter().map(|(k, v)| (k.to_string(), v.clone())).collect();
        r
    }

    fn t(secs: u64) -> Instant {
        Instant::now() + Duration::from_secs(secs)
    }

    #[test]
    fn two_ends_on_one_system_are_one_link() {
        let mut l = Links::new();
        let f = [("from", Value::Text("1234567".into())), ("to", Value::Text("9".into()))];
        assert!(l.update(&rec("DMR-Header", 446.1e6, &f), t(0)));
        assert!(l.update(&rec("DMR-Voice", 446.1e6, &f), t(1)));
        let links = l.active(t(2));
        assert_eq!(links.len(), 1, "{:?}", links.iter().map(|x| x.title()).collect::<Vec<_>>());
        assert_eq!(links[0].packets, 2);
        assert_eq!(links[0].title(), "DMR 1234567 -> 9");
        // Both packets are there to follow, in the order they arrived.
        let seen: Vec<&str> = links[0].moments.iter().map(|m| m.protocol.as_str()).collect();
        assert_eq!(seen, ["DMR-Header", "DMR-Voice"]);
    }

    #[test]
    fn a_beacon_is_a_link_from_one_end() {
        // Most telemetry names only itself, and a directory that refused
        // those would have nothing on 433 or 868 MHz at all.
        let mut l = Links::new();
        assert!(l.update(
            &rec("BLE-Adv", 2426e6, &[("from", Value::Text("6C:70:CB:EF:72:4D".into())),
                                      ("to", Value::Text("broadcast".into()))]),
            t(0)
        ));
        let links = l.active(t(1));
        assert_eq!(links[0].title(), "BLE 6C:70:CB:EF:72:4D -> broadcast");
        assert_eq!(links[0].to, End::Broadcast);
    }

    #[test]
    fn a_burst_nobody_owns_is_not_a_link() {
        let mut l = Links::new();
        assert!(!l.update(&rec("unknown", 433.92e6, &[("baud", Value::Float(1500.0))]), t(0)));
        assert!(l.is_empty());
    }

    #[test]
    fn the_same_ends_on_different_systems_are_different_links() {
        let mut l = Links::new();
        let f = [("from", Value::Text("2001".into())), ("to", Value::Text("2002".into()))];
        l.update(&rec("TETRA-SDS", 391.1e6, &f), t(0));
        l.update(&rec("M17-Packet", 433.475e6, &f), t(1));
        assert_eq!(l.active(t(2)).len(), 2);
    }

    #[test]
    fn a_link_carries_what_was_said() {
        let mut l = Links::new();
        l.update(
            &rec(
                "TETRA-SDS",
                391.1e6,
                &[
                    ("from", Value::Text("2001".into())),
                    ("to", Value::Text("2002".into())),
                    ("text", Value::Text("on my way".into())),
                ],
            ),
            t(0),
        );
        let links = l.active(t(1));
        assert_eq!(links[0].moments[0].text.as_deref(), Some("on my way"));
    }

    #[test]
    fn a_loaded_directory_merges_into_the_live_one() {
        // The same advertiser heard live and again from the log is one link
        // with both sets of packets, in the order they happened.
        let f = [("from", Value::Text("aa:bb".into())), ("to", Value::Text("broadcast".into()))];
        let mut live = Links::new();
        live.update(&rec("BLE-Adv", 2426e6, &f), t(10));
        let mut loaded = Links::new();
        loaded.update(&rec("BLE-Adv", 2426e6, &f), t(0));
        loaded.update(&rec("BLE-Adv", 2426e6, &f), t(5));
        live.absorb(loaded);
        let links = live.active(t(11));
        assert_eq!(links.len(), 1);
        assert_eq!(links[0].packets, 3);
        let order: Vec<Instant> = links[0].moments.iter().map(|m| m.at).collect();
        assert!(order.windows(2).all(|w| w[0] <= w[1]), "packets are out of order");
    }

    #[test]
    fn one_end_can_be_looked_up_whichever_side_it_is_on() {
        let mut l = Links::new();
        l.update(
            &rec("M17-Packet", 433.475e6, &[("from", Value::Text("M0ABC".into())),
                                            ("to", Value::Text("M0XYZ".into()))]),
            t(0),
        );
        l.update(
            &rec("M17-Packet", 433.475e6, &[("from", Value::Text("M0XYZ".into())),
                                            ("to", Value::Text("M0ABC".into()))]),
            t(1),
        );
        // Two links, because a direction is worth keeping; both involve M0ABC.
        assert_eq!(l.active(t(2)).len(), 2);
        assert_eq!(l.involving("M0ABC", t(2)).len(), 2);
        assert_eq!(l.involving("M0XYZ", t(2)).len(), 2);
        assert_eq!(l.involving("M0ZZZ", t(2)).len(), 0);
    }
}
