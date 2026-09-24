use crate::model::{Entry, Version};
use nostr_sdk::prelude::{
    Coordinate, Event, EventBuilder, EventDeletionRequest, FinalizeEvent, Keys, Kind, PublicKey,
    Tag, Timestamp,
};

pub const KIND: Kind = Kind::Custom(10_690);

pub const EXPIRES_AFTER_SECS: u64 = 24 * 60 * 60;
pub const ANNOUNCE_EVERY_SECS: u64 = EXPIRES_AFTER_SECS;
pub const TOMBSTONE_SECS: u64 = 60;

#[derive(Debug, thiserror::Error)]
pub enum Refused {
    #[error("kind {0} is not a tuner listing")]
    Kind(u16),
    #[error("the signature does not match")]
    Signature,
    #[error("expired at {0}")]
    Expired(u64),
    #[error("the listing does not read: {0}")]
    Content(String),
}

pub fn announcement(keys: &Keys, entry: &Entry, at: u64) -> Result<Event, crate::Error> {
    EventBuilder::new(KIND, "")
        .tags(crate::tags::encode(entry))
        .tag(Tag::expiration(Timestamp::from_secs(at + EXPIRES_AFTER_SECS)))
        .custom_created_at(Timestamp::from_secs(at))
        .finalize(keys)
        .map_err(|e| crate::Error::Sign(e.to_string()))
}

pub fn tombstone(keys: &Keys, at: u64) -> Result<Event, crate::Error> {
    EventBuilder::new(KIND, "")
        .tag(Tag::expiration(Timestamp::from_secs(at + TOMBSTONE_SECS)))
        .custom_created_at(Timestamp::from_secs(at))
        .finalize(keys)
        .map_err(|e| crate::Error::Sign(e.to_string()))
}

pub fn withdrawal(keys: &Keys) -> Result<Event, crate::Error> {
    EventDeletionRequest::new()
        .coordinate(Coordinate::new(KIND, keys.public_key()))
        .finalize(keys)
        .map_err(|e| crate::Error::Sign(e.to_string()))
}

#[derive(Clone, Debug, PartialEq)]
pub struct Listing {
    pub author: PublicKey,
    pub seen: u64,
    pub entry: Entry,
}

impl Listing {
    pub fn online(&self, now: u64) -> bool {
        now.saturating_sub(self.seen) < EXPIRES_AFTER_SECS
    }
}

pub fn read(event: &Event, now: u64) -> Result<Listing, Refused> {
    if event.kind != KIND {
        return Err(Refused::Kind(event.kind.as_u16()));
    }
    event.verify().map_err(|_| Refused::Signature)?;
    if let Some(at) = event.tags.expiration().filter(|at| at.as_secs() <= now) {
        return Err(Refused::Expired(at.as_secs()));
    }
    let entry = crate::tags::decode(event.tags.iter()).map_err(Refused::Content)?;
    Ok(Listing { author: event.pubkey, seen: event.created_at.as_secs(), entry })
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Query {
    pub hz: Option<u64>,
    pub tunable: bool,
    pub free: bool,
}

impl Query {
    pub fn keeps(&self, l: &Listing, now: u64) -> bool {
        let s = &l.entry.station;
        l.online(now)
            && Version::OURS.speaks_with(&s.version)
            && (!self.free || s.has_slot())
            && s.tuners
                .iter()
                .any(|t| self.hz.is_none_or(|hz| t.hears(hz)) && (!self.tunable || t.tunable()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Station;
    use crate::model::tests::{airband, entry, hf, station};

    const NOW: u64 = 1_750_000_000;

    #[test]
    fn an_announcement_is_a_replaceable_event_that_expires_with_the_listing() {
        let keys = Keys::generate();
        let mut sent = entry("sdr.example.net", vec![airband()]);
        sent.station.location = None;
        let e = announcement(&keys, &sent, NOW).unwrap();
        assert!(e.kind.is_replaceable(), "kind {} is outside 10000..20000", e.kind);
        assert_eq!(e.created_at.as_secs(), NOW);
        assert_eq!(e.tags.expiration().map(|t| t.as_secs()), Some(NOW + EXPIRES_AFTER_SECS));
        assert_eq!(e.content, "");
        let back = read(&e, NOW).unwrap();
        assert_eq!(back.author, keys.public_key());
        assert_eq!(back.seen, NOW);
        assert_eq!(back.entry, sent);
    }

    #[test]
    fn a_listing_is_refused_when_it_is_not_one_or_not_signed_by_its_author() {
        let keys = Keys::generate();
        let good = announcement(&keys, &entry("a.example", vec![hf()]), NOW).unwrap();
        let note = EventBuilder::new(Kind::TextNote, "")
            .tags(good.tags.iter().cloned())
            .finalize(&keys)
            .unwrap();
        assert!(matches!(read(&note, NOW), Err(Refused::Kind(1))));
        let mut forged = good.clone();
        forged.tags = nostr_sdk::prelude::Tags::from_list(crate::tags::encode(&entry(
            "evil.example",
            vec![hf()],
        )));
        assert!(matches!(read(&forged, NOW), Err(Refused::Signature)));
        let junk = EventBuilder::new(KIND, "")
            .tag(Tag::custom("r", ["iqstream://a.example:5557"]))
            .finalize(&keys)
            .unwrap();
        assert!(matches!(read(&junk, NOW), Err(Refused::Content(_))));
    }

    #[test]
    fn a_listing_past_its_expiration_is_refused_even_if_a_relay_kept_it() {
        let keys = Keys::generate();
        let e = announcement(&keys, &entry("a.example", vec![hf()]), NOW).unwrap();
        assert!(read(&e, NOW + EXPIRES_AFTER_SECS - 1).is_ok());
        assert!(matches!(
            read(&e, NOW + EXPIRES_AFTER_SECS),
            Err(Refused::Expired(at)) if at == NOW + EXPIRES_AFTER_SECS
        ));
    }

    #[test]
    fn a_withdrawal_deletes_the_authors_listing_by_its_address() {
        let keys = Keys::generate();
        let w = withdrawal(&keys).unwrap();
        assert_eq!(w.kind, Kind::EventDeletion);
        let a: Vec<&str> =
            w.tags.iter().filter(|t| t.kind() == "a").filter_map(|t| t.content()).collect();
        assert_eq!(a, [format!("10690:{}:", keys.public_key().to_hex())]);
    }

    #[test]
    fn a_tombstone_replaces_the_listing_and_reads_as_nothing() {
        let keys = Keys::generate();
        let t = tombstone(&keys, NOW).unwrap();
        assert!(t.kind.is_replaceable());
        assert_eq!((t.kind, t.created_at.as_secs()), (KIND, NOW));
        assert_eq!(t.tags.expiration().map(|x| x.as_secs()), Some(NOW + TOMBSTONE_SECS));
        assert!(matches!(read(&t, NOW), Err(Refused::Content(_))));
    }

    fn listed(entry: Entry) -> Listing {
        Listing { author: Keys::generate().public_key(), seen: 2_000, entry }
    }

    #[test]
    fn a_query_keeps_servers_announced_recently_that_hear_the_frequency() {
        let now = 2_000 + EXPIRES_AFTER_SECS - 1;
        let listed = [
            listed(entry("a.example", vec![airband()])),
            listed(entry("b.example", vec![airband(), hf()])),
            Listing { seen: 1, ..listed(entry("d.example", vec![hf()])) },
            listed(Entry {
                station: Station { clients: 4, ..station(vec![hf()]) },
                ..entry("e.example", vec![])
            }),
            listed(Entry {
                station: Station { version: Version { major: 2, minor: 0 }, ..station(vec![hf()]) },
                ..entry("f.example", vec![])
            }),
        ];
        let kept = |q: Query| -> Vec<&str> {
            listed.iter().filter(|l| q.keeps(l, now)).map(|l| l.entry.host.as_str()).collect()
        };
        assert_eq!(kept(Query::default()), ["a.example", "b.example", "e.example"]);
        assert_eq!(
            kept(Query { hz: Some(14_074_000), ..Query::default() }),
            ["b.example", "e.example"]
        );
        assert_eq!(kept(Query { tunable: true, free: true, ..Query::default() }), ["b.example"]);
        assert_eq!(
            kept(Query { hz: Some(125_000_000), tunable: true, ..Query::default() }),
            Vec::<&str>::new(),
            "the tuner hearing 125 MHz is not the tunable one"
        );
    }
}
