use crate::event::{EXPIRES_AFTER_SECS, KIND, Listing, announcement, read, tombstone, withdrawal};
use crate::model::Entry;
use crate::tags::GEOHASH_LADDER;
use nostr_sdk::prelude::{
    Client, Event, EventId, Filter, Keys, PublicKey, SingleLetterTag, Timestamp,
};
use std::collections::HashMap;
use std::time::Duration;

pub const RELAYS: [&str; 4] =
    ["wss://relay.damus.io", "wss://nos.lol", "wss://relay.primal.net", "wss://relay.snort.social"];

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Published {
    pub id: EventId,
    pub accepted: usize,
    pub refused: Vec<(String, String)>,
}

pub struct Directory {
    client: Client,
}

impl Directory {
    pub async fn connect<S: AsRef<str>>(
        relays: &[S],
        wait: Duration,
    ) -> Result<Self, crate::Error> {
        let client = Client::default();
        for url in relays {
            client
                .add_relay(url.as_ref())
                .await
                .map_err(|e| crate::Error::Relay(format!("{}: {e}", url.as_ref())))?;
        }
        client.connect().and_wait(wait).await;
        Ok(Directory { client })
    }

    pub async fn announce(&self, keys: &Keys, entry: &Entry) -> Result<Published, crate::Error> {
        let now = Timestamp::now().as_secs();
        self.send(&announcement(keys, entry, now)?).await
    }

    pub async fn withdraw(&self, keys: &Keys) -> Result<Published, crate::Error> {
        let now = Timestamp::now().as_secs();
        let replaced = self.send(&tombstone(keys, now)?).await;
        let deleted = self.send(&withdrawal(keys)?).await;
        replaced.or(deleted)
    }

    async fn send(&self, event: &Event) -> Result<Published, crate::Error> {
        let out =
            self.client.send_event(event).await.map_err(|e| crate::Error::Relay(e.to_string()))?;
        let refused: Vec<(String, String)> =
            out.failed.iter().map(|(url, why)| (url.to_string(), why.clone())).collect();
        match out.success.len() {
            0 => Err(crate::Error::Refused(refused)),
            accepted => Ok(Published { id: *out.id(), accepted, refused }),
        }
    }

    pub async fn list(&self, wait: Duration) -> Result<Vec<Listing>, crate::Error> {
        self.fetch(Filter::new(), wait).await
    }

    pub async fn list_near(
        &self,
        geohash: &str,
        wait: Duration,
    ) -> Result<Vec<Listing>, crate::Error> {
        let cell = &geohash[..geohash.len().min(GEOHASH_LADDER)];
        let g = SingleLetterTag::from_char('g').expect("g is a single letter tag");
        self.fetch(Filter::new().custom_tag(g, cell), wait).await
    }

    async fn fetch(&self, filter: Filter, wait: Duration) -> Result<Vec<Listing>, crate::Error> {
        let now = Timestamp::now().as_secs();
        let since = Timestamp::from_secs(now.saturating_sub(EXPIRES_AFTER_SECS));
        let events = self
            .client
            .fetch_events(filter.kind(KIND).since(since))
            .timeout(wait)
            .await
            .map_err(|e| crate::Error::Relay(e.to_string()))?;
        Ok(newest_per_author(events.iter().filter_map(|e| read(e, now).ok()), now))
    }

    pub async fn shutdown(self) {
        self.client.shutdown().await;
    }
}

pub fn newest_per_author(listings: impl Iterator<Item = Listing>, now: u64) -> Vec<Listing> {
    let mut by: HashMap<PublicKey, Listing> = HashMap::new();
    for l in listings.filter(|l| l.online(now)) {
        match by.get(&l.author) {
            Some(held) if held.seen >= l.seen => {}
            _ => {
                by.insert(l.author, l);
            }
        }
    }
    let mut out: Vec<Listing> = by.into_values().collect();
    out.sort_by(|a, b| b.seen.cmp(&a.seen).then_with(|| a.entry.addr().cmp(&b.entry.addr())));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::tests::{airband, entry};

    #[test]
    fn of_two_listings_by_one_author_the_later_is_kept_and_a_stale_one_is_dropped() {
        let (a, b) = (Keys::generate().public_key(), Keys::generate().public_key());
        let l = |author, seen, host| Listing { author, seen, entry: entry(host, vec![airband()]) };
        let now = 1_750_000_000;
        let kept = newest_per_author(
            [
                l(a, now - 500, "old.a"),
                l(a, now - 100, "new.a"),
                l(a, now - 300, "mid.a"),
                l(b, now - EXPIRES_AFTER_SECS, "stale.b"),
            ]
            .into_iter(),
            now,
        );
        let hosts: Vec<&str> = kept.iter().map(|l| l.entry.host.as_str()).collect();
        assert_eq!(hosts, ["new.a"]);
    }
}
