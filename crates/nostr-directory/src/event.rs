use crate::keys::{Keys, PublicKey};
use crate::tags::{Tag, tag};
use sdr_directory::{Author, Entry, Listing, STALE_AFTER_SECS};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};

pub const KIND: u16 = 10_690;
pub const DELETION: u16 = 5;

pub const EXPIRES_AFTER_SECS: u64 = STALE_AFTER_SECS;
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

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Event {
    pub id: [u8; 32],
    pub pubkey: PublicKey,
    pub created_at: u64,
    pub kind: u16,
    pub tags: Vec<Tag>,
    pub content: String,
    pub sig: [u8; 64],
}

impl Event {
    pub fn sign(keys: &Keys, kind: u16, tags: Vec<Tag>, content: &str, created_at: u64) -> Event {
        let pubkey = keys.public_key();
        let id = digest(&pubkey, created_at, kind, &tags, content);
        let sig = keys.sign(&id);
        Event { id, pubkey, created_at, kind, tags, content: content.to_string(), sig }
    }

    pub fn verify(&self) -> bool {
        digest(&self.pubkey, self.created_at, self.kind, &self.tags, &self.content) == self.id
            && self.pubkey.verify(&self.id, &self.sig)
    }

    pub fn id_hex(&self) -> String {
        hex::encode(self.id)
    }

    pub fn replaceable(&self) -> bool {
        (10_000..20_000).contains(&self.kind)
    }

    pub fn first(&self, key: &str) -> Option<&str> {
        self.tags.iter().find(|t| t.first().is_some_and(|k| k == key))?.get(1).map(String::as_str)
    }

    pub fn expiration(&self) -> Option<u64> {
        self.first("expiration")?.trim().parse().ok()
    }

    pub fn to_json(&self) -> Value {
        json!({
            "id": self.id_hex(),
            "pubkey": self.pubkey.to_hex(),
            "created_at": self.created_at,
            "kind": self.kind,
            "tags": self.tags,
            "content": self.content,
            "sig": hex::encode(self.sig),
        })
    }

    pub fn from_json(v: &Value) -> Option<Event> {
        let mut id = [0u8; 32];
        hex::decode_to_slice(v["id"].as_str()?, &mut id).ok()?;
        let mut sig = [0u8; 64];
        hex::decode_to_slice(v["sig"].as_str()?, &mut sig).ok()?;
        let tags = v["tags"]
            .as_array()?
            .iter()
            .map(|t| t.as_array()?.iter().map(|s| s.as_str().map(str::to_string)).collect())
            .collect::<Option<Vec<Tag>>>()?;
        Some(Event {
            id,
            pubkey: PublicKey::from_hex(v["pubkey"].as_str()?)?,
            created_at: v["created_at"].as_u64()?,
            kind: u16::try_from(v["kind"].as_u64()?).ok()?,
            tags,
            content: v["content"].as_str()?.to_string(),
            sig,
        })
    }
}

fn digest(pubkey: &PublicKey, created_at: u64, kind: u16, tags: &[Tag], content: &str) -> [u8; 32] {
    let canonical = json!([0, pubkey.to_hex(), created_at, kind, tags, content]).to_string();
    Sha256::digest(canonical.as_bytes()).into()
}

pub fn announcement(keys: &Keys, entry: &Entry, at: u64) -> Event {
    let mut tags = crate::tags::encode(entry);
    tags.push(tag("expiration", [(at + EXPIRES_AFTER_SECS).to_string()]));
    Event::sign(keys, KIND, tags, "", at)
}

pub fn tombstone(keys: &Keys, at: u64) -> Event {
    Event::sign(keys, KIND, vec![tag("expiration", [(at + TOMBSTONE_SECS).to_string()])], "", at)
}

pub fn withdrawal(keys: &Keys, at: u64, announced: Option<[u8; 32]>) -> Event {
    let address = format!("{KIND}:{}:", keys.public_key().to_hex());
    let mut tags = vec![tag("a", [address])];
    tags.extend(announced.map(|id| tag("e", [hex::encode(id)])));
    tags.push(tag("k", [KIND.to_string()]));
    Event::sign(keys, DELETION, tags, "", at)
}

pub fn read(event: &Event, now: u64) -> Result<Listing, Refused> {
    if event.kind != KIND {
        return Err(Refused::Kind(event.kind));
    }
    if !event.verify() {
        return Err(Refused::Signature);
    }
    if let Some(at) = event.expiration().filter(|at| *at <= now) {
        return Err(Refused::Expired(at));
    }
    let entry = crate::tags::decode(event.tags.iter()).map_err(Refused::Content)?;
    Ok(Listing { author: Author::from(event.pubkey), seen: event.created_at, entry })
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdr_directory::model::fixtures::{airband, entry, hf};
    use sdr_directory::{Protocol, Version};

    const NOW: u64 = 1_750_000_000;

    const NOSTR_SDK_0_45: &str = r#"{"id":"30710139919ec4513ddd2e828924d02f2f9fa1feebaa2a52cb930684e9d27d44","pubkey":"f9308a019258c31049344f85f89d5229b531c845836f99b08601f113bce036f9","created_at":1750000000,"kind":10690,"tags":[["name","G0ABC"],["description","Loft \"north\"\nReading, é ✓ \\ \t"],["r","iqstream://sdr.example.net:5557"],["version","1.5"],["clients","1"],["max_clients","4"],["g","g"],["g","gc"],["g","gcp"],["g","gcpk"],["g","gcpk9"],["t","sdr"],["t","iqstream"],["t","rtlsdr"],["tuner","id 0","name Airband","type rtlsdr","antenna Discone","center 125000000","rate 2400000","tunable 0"],["expiration","1750086400"]],"content":"","sig":"86314f8c8849a7667cf9d10a3ed1281e3e25d9388b84e9d11eb06fae9c7ef28d40e84c766eeb4f8766f7866c4b2195af0a9b11c090ac789a5c6425f46f63b4f1"}"#;

    #[test]
    fn an_announcement_has_the_id_nostr_sdk_0_45_gave_it_and_its_signature_reads() {
        let keys = Keys::parse("0000000000000000000000000000000000000000000000000000000000000003")
            .unwrap();
        let mut e = entry("sdr.example.net", vec![airband()]);
        e.station.protocol = Protocol::IqStream(Version { major: 1, minor: 5 });
        e.station.description = "Loft \"north\"\nReading, é ✓ \\ \t".into();
        let ours = announcement(&keys, &e, NOW);
        let theirs = Event::from_json(&serde_json::from_str(NOSTR_SDK_0_45).unwrap()).unwrap();
        assert_eq!(
            ours.id_hex(),
            "30710139919ec4513ddd2e828924d02f2f9fa1feebaa2a52cb930684e9d27d44"
        );
        assert_eq!(Event { sig: theirs.sig, ..ours.clone() }, theirs);
        assert!(theirs.verify(), "nostr-sdk 0.45's signature verifies here");
        assert_eq!(Event::from_json(&ours.to_json()), Some(ours.clone()));
        assert_eq!(read(&theirs, NOW).unwrap().entry.addr(), "sdr.example.net:5557");
    }

    #[test]
    fn an_announcement_is_a_replaceable_event_that_expires_with_the_listing() {
        let keys = Keys::generate();
        let mut sent = entry("sdr.example.net", vec![airband()]);
        sent.station.location = None;
        let e = announcement(&keys, &sent, NOW);
        assert!(e.replaceable(), "kind {} is outside 10000..20000", e.kind);
        assert_eq!(e.created_at, NOW);
        assert_eq!(e.expiration(), Some(NOW + EXPIRES_AFTER_SECS));
        assert_eq!(e.content, "");
        let back = read(&e, NOW).unwrap();
        assert_eq!(back.author, Author::from(keys.public_key()));
        assert_eq!(back.seen, NOW);
        assert_eq!(back.entry, sent);
    }

    #[test]
    fn a_listing_is_refused_when_it_is_not_one_or_not_signed_by_its_author() {
        let keys = Keys::generate();
        let good = announcement(&keys, &entry("a.example", vec![hf()]), NOW);
        let note = Event::sign(&keys, 1, good.tags.clone(), "", NOW);
        assert!(matches!(read(&note, NOW), Err(Refused::Kind(1))));
        let mut forged = good.clone();
        forged.tags = crate::tags::encode(&entry("evil.example", vec![hf()]));
        assert!(matches!(read(&forged, NOW), Err(Refused::Signature)));
        let mut resigned = good.clone();
        resigned.sig = Keys::generate().sign(&good.id);
        assert!(matches!(read(&resigned, NOW), Err(Refused::Signature)));
        let junk = Event::sign(&keys, KIND, vec![tag("r", ["iqstream://a.example:5557"])], "", NOW);
        assert!(matches!(read(&junk, NOW), Err(Refused::Content(_))));
    }

    #[test]
    fn a_listing_past_its_expiration_is_refused_even_if_a_relay_kept_it() {
        let keys = Keys::generate();
        let e = announcement(&keys, &entry("a.example", vec![hf()]), NOW);
        assert!(read(&e, NOW + EXPIRES_AFTER_SECS - 1).is_ok());
        assert!(matches!(
            read(&e, NOW + EXPIRES_AFTER_SECS),
            Err(Refused::Expired(at)) if at == NOW + EXPIRES_AFTER_SECS
        ));
    }

    #[test]
    fn a_withdrawal_deletes_the_authors_listing_by_its_address() {
        let keys = Keys::generate();
        let listed = announcement(&keys, &entry("a.example", vec![hf()]), NOW);
        let w = withdrawal(&keys, NOW + 1, Some(listed.id));
        assert_eq!(w.kind, DELETION);
        assert!(w.verify());
        assert_eq!(
            w.tags,
            [
                tag("a", [format!("10690:{}:", keys.public_key().to_hex())]),
                tag("e", [listed.id_hex()]),
                tag("k", ["10690"]),
            ],
            "by its address for relays that read NIP-09 in full, by id for those that do not"
        );
    }

    #[test]
    fn a_tombstone_replaces_the_listing_and_reads_as_nothing() {
        let keys = Keys::generate();
        let t = tombstone(&keys, NOW);
        assert!(t.replaceable());
        assert_eq!((t.kind, t.created_at), (KIND, NOW));
        assert_eq!(t.expiration(), Some(NOW + TOMBSTONE_SECS));
        assert!(matches!(read(&t, NOW), Err(Refused::Content(_))));
    }
}
