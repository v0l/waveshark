//! Gateways: the places a digital voice network can be reached over the
//! internet, whatever mode is spoken there.
//!
//! An M17 reflector, a YSF room, a DMR master and a P25 reflector are the
//! same thing to somebody looking for traffic: an address, a port, and a
//! sub-address within it that decides which conversation you get. Each
//! network publishes that as a host file, and the files disagree about
//! nearly everything else, so a [`HostFile`] is a row here holding where the
//! file lives and how to turn it into [`Gateway`]s. Adding a network is
//! adding a row and a parser; nothing outside this module knows an M17
//! module letter from a YSF room number.
//!
//! What the rows deliberately do not carry is how to connect. That is a
//! node's problem, and a node asks this for an address rather than the other
//! way round.

use crate::cache::{Cache, Error, Source, When};
use std::time::Duration;

/// One published host file: where it lives, and how to read it.
#[derive(Debug)]
pub struct HostFile {
    /// What the mode is called, for a view that groups by it.
    pub name: &'static str,
    /// File name under the cache directory, and the metadata key.
    pub file: &'static str,
    pub url: &'static str,
    /// Who publishes it, for the line under the name in the settings pane.
    pub publisher: &'static str,
    /// Where to connect when the file publishes no port of its own. For the
    /// D-Star networks that is the port of the protocol itself, and a file
    /// listing it per reflector would be the same number 1500 times.
    pub default_port: u16,
    /// How often it is worth asking whether the file changed.
    pub max_age: Duration,
    pub parse: fn(&'static HostFile, &[u8]) -> Result<Vec<Gateway>, Error>,
}

impl HostFile {
    pub fn source(&'static self) -> Source {
        Source::http(self.file, self.url, self.max_age)
    }
}

/// Rows are told apart by their cache file name, which a test holds unique.
/// Deriving this would compare the parse function pointers, and two of those
/// being equal or not says nothing about the rows.
impl PartialEq for HostFile {
    fn eq(&self, other: &Self) -> bool {
        self.file == other.file
    }
}

impl Eq for HostFile {}

/// Rebuilt daily at the far end, so a check a day apart cannot miss a build
/// and a check every launch would be noise.
const DAILY: Duration = Duration::from_secs(24 * 3600);

pub static M17: HostFile = HostFile {
    name: "M17",
    file: "m17-hosts.json",
    url: "https://m17-project.github.io/hostfiles/M17Hosts.json",
    publisher: "m17project.org",
    default_port: 17000,
    max_age: DAILY,
    parse: crate::m17::parse,
};

pub static DMR: HostFile = HostFile {
    name: "DMR",
    file: "pistar-dmr-hosts.txt",
    url: "https://www.pistar.uk/downloads/DMR_Hosts.txt",
    publisher: "pistar.uk",
    default_port: 62030,
    max_age: DAILY,
    parse: crate::pistar::dmr,
};

pub static DPLUS: HostFile = HostFile {
    name: "D-Star (DPlus)",
    file: "pistar-dplus-hosts.txt",
    url: "https://www.pistar.uk/downloads/DPlus_Hosts.txt",
    publisher: "pistar.uk",
    default_port: 20001,
    max_age: DAILY,
    parse: crate::pistar::dstar,
};

pub static DEXTRA: HostFile = HostFile {
    name: "D-Star (DExtra)",
    file: "pistar-dextra-hosts.txt",
    url: "https://www.pistar.uk/downloads/DExtra_Hosts.txt",
    publisher: "pistar.uk",
    default_port: 30001,
    max_age: DAILY,
    parse: crate::pistar::dstar,
};

pub static DCS: HostFile = HostFile {
    name: "D-Star (DCS)",
    file: "pistar-dcs-hosts.txt",
    url: "https://www.pistar.uk/downloads/DCS_Hosts.txt",
    publisher: "pistar.uk",
    default_port: 30051,
    max_age: DAILY,
    parse: crate::pistar::dstar,
};

/// Every host file that is read, in the order a view lists them.
///
/// Only M17 is decodable here today: the rest speak AMBE, which needs a
/// vocoder this receiver does not have. They are listed anyway because the
/// list is reference data about where the networks are, and a view filters
/// on [`Gateway::kind`] rather than this deciding for it.
pub static HOST_FILES: &[&HostFile] = &[&M17, &DMR, &DPLUS, &DEXTRA, &DCS];

pub fn host_file(name: &str) -> Option<&'static HostFile> {
    HOST_FILES.iter().copied().find(|h| h.name.eq_ignore_ascii_case(name))
}

pub fn sources() -> Vec<Source> {
    HOST_FILES.iter().map(|h| h.source()).collect()
}

/// One conversation within a gateway: an M17 or URF module letter, a YSF
/// room, a DMR talkgroup. A string because the networks do not agree on
/// whether it is a letter or a number, and parsing one into the other's
/// shape would lose the difference.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Channel {
    pub id: String,
    /// Whatever the file calls it, empty when it names none.
    pub name: String,
    /// The gateway will pass encrypted traffic here. It decodes to noise, so
    /// a channel that is only ever encrypted is one worth skipping.
    pub encrypted: bool,
}

/// Somewhere to connect, as its network's host file describes it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Gateway {
    /// The row it was read from, which is also what mode it speaks.
    pub kind: &'static HostFile,
    /// `M17-USA`, `URF018`. Unique within a host file, and for the M17
    /// network it is also the key its live document is published under on
    /// Ham-DHT.
    pub designator: String,
    /// The name its owner gave it, empty for the many that have none.
    pub name: String,
    /// Preferred first: a name outlives the address behind it, and half the
    /// M17 file is on dynamic addresses re-resolved daily by the publisher.
    pub dns: Option<String>,
    pub ipv4: Option<String>,
    pub ipv6: Option<String>,
    pub port: u16,
    /// Only the channels that carry this gateway's mode. A bridge carries
    /// several, and connecting to the wrong one succeeds and stays silent.
    pub channels: Vec<Channel>,
    /// The shared password the host file publishes, where it publishes one.
    /// This is the network's, not the operator's: BrandMeister ignores it
    /// and wants a hotspot security password set per DMR ID instead.
    pub password: Option<String>,
    pub sponsor: String,
    /// Two-letter code, as published.
    pub country: String,
}

impl Gateway {
    /// Where to send the first packet, or `None` for a gateway listed
    /// without any address. Kept rather than dropped, so one that loses its
    /// address upstream shows as unreachable instead of silently vanishing.
    pub fn host(&self) -> Option<&str> {
        self.dns.as_deref().or(self.ipv4.as_deref()).or(self.ipv6.as_deref())
    }

    pub fn channel(&self, id: &str) -> Option<&Channel> {
        self.channels.iter().find(|c| c.id.eq_ignore_ascii_case(id))
    }
}

/// Every gateway from every host file that is cached, downloading whichever
/// is not there yet.
pub fn load(cache: &Cache) -> Result<Vec<Gateway>, Error> {
    read(cache)
}

/// Check every host file and reparse if any changed. `None` means what is
/// cached is still current.
pub fn refresh(cache: &Cache, when: When) -> Result<Option<Vec<Gateway>>, Error> {
    let mut changed = false;
    for h in HOST_FILES {
        changed |= cache.refresh(&h.source(), when)?.is_some();
    }
    if !changed {
        return Ok(None);
    }
    read(cache).map(Some)
}

/// Read every host file and concatenate what they hold.
///
/// A publisher being down is not a reason to have no gateways at all, so a
/// file that will not fetch or parse is logged and skipped, and only the
/// last failure with nothing at all to show for it is an error.
fn read(cache: &Cache) -> Result<Vec<Gateway>, Error> {
    let mut out = Vec::new();
    let mut last: Option<Error> = None;
    for h in HOST_FILES.iter().copied() {
        match cache.read(&h.source()).and_then(|raw| (h.parse)(h, &raw)) {
            Ok(mut g) => out.append(&mut g),
            Err(e) => {
                tracing::warn!(host_file = h.name, "gateways unavailable: {e}");
                last = Some(e);
            }
        }
    }
    match last {
        Some(e) if out.is_empty() => Err(e),
        _ => Ok(out),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_host_file_has_its_own_cache_entry() {
        for h in HOST_FILES {
            assert!(!h.file.is_empty(), "{} has no cache file name", h.name);
            let same = HOST_FILES.iter().filter(|o| o.file == h.file).count();
            assert_eq!(same, 1, "{} shares a cache file with another row", h.name);
            assert_eq!(host_file(h.name).map(|f| f.file), Some(h.file));
        }
    }
}
