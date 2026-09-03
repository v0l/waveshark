//! The M17 host file: every M17 reflector, as the M17 project publishes it.
//!
//! One of the [`crate::gateways`] rows. The file holds two kinds of
//! reflector under one array: an `mrefd` entry lists its modules as bare
//! letters and publishes its port, while a `URF` entry lists them as records
//! naming the mode each carries and publishes no port at all. Both come out
//! as [`Gateway`]s, because a client looking for somewhere to listen does
//! not care which server is on the far end.

use crate::cache::Error;
use crate::gateways::{Channel, Gateway, HostFile};

mod wire {
    //! The published shape. Nearly every field is nullable and two of them
    //! are absent entirely on a URF row, so everything is optional and the
    //! difference between the two kinds is read from what is there.

    #[derive(serde::Deserialize)]
    pub struct Doc {
        #[serde(default)]
        pub reflectors: Vec<Reflector>,
    }

    #[derive(serde::Deserialize)]
    pub struct Reflector {
        #[serde(default)]
        pub designator: String,
        #[serde(default)]
        pub name: Option<String>,
        #[serde(default)]
        pub dns: Option<String>,
        #[serde(default)]
        pub ipv4: Option<String>,
        #[serde(default)]
        pub ipv6: Option<String>,
        /// Absent on a URF row, which is how the two kinds are told apart.
        #[serde(default)]
        pub port: Option<u16>,
        #[serde(default)]
        pub modules: Modules,
        /// The modules an mrefd reflector will pass encrypted streams on.
        #[serde(default)]
        pub encrypted: Vec<String>,
        #[serde(default)]
        pub sponsor: Option<String>,
        #[serde(default)]
        pub country: String,
    }

    /// Letters from mrefd, records from URF.
    #[derive(serde::Deserialize)]
    #[serde(untagged)]
    pub enum Modules {
        Letters(Vec<String>),
        Urf(Vec<UrfModule>),
    }

    impl Default for Modules {
        fn default() -> Self {
            Modules::Letters(Vec::new())
        }
    }

    #[derive(serde::Deserialize)]
    pub struct UrfModule {
        #[serde(default)]
        pub module: String,
        /// `M17`, `All`, `D-Star (DCS)`, `YSF`, and so on.
        #[serde(default)]
        pub mode: String,
        #[serde(default)]
        pub description: Option<String>,
    }
}

pub fn parse(kind: &'static HostFile, json: &[u8]) -> Result<Vec<Gateway>, Error> {
    let doc: wire::Doc =
        serde_json::from_slice(json).map_err(|e| Error::Parse(kind.file.into(), e.to_string()))?;
    if doc.reflectors.is_empty() {
        return Err(Error::Parse(kind.file.into(), "no reflectors in the host file".into()));
    }
    Ok(doc.reflectors.into_iter().map(|r| gateway(kind, r)).collect())
}

fn gateway(kind: &'static HostFile, r: wire::Reflector) -> Gateway {
    let encrypted: Vec<char> = r.encrypted.iter().filter_map(|m| letter(m)).collect();
    let mark = |id: char, name: String| Channel {
        id: id.to_string(),
        name,
        encrypted: encrypted.contains(&id),
    };
    let channels = match r.modules {
        wire::Modules::Letters(l) => {
            l.iter().filter_map(|m| letter(m)).map(|id| mark(id, String::new())).collect()
        }
        // A URF module carries one mode, or every mode when it transcodes.
        // Anything else on it is not M17, and connecting there succeeds and
        // then hears nothing, which looks exactly like a dead network.
        wire::Modules::Urf(l) => l
            .iter()
            .filter(|m| m.mode.eq_ignore_ascii_case("M17") || m.mode.eq_ignore_ascii_case("All"))
            .filter_map(|m| Some((letter(&m.module)?, m.description.clone().unwrap_or_default())))
            .map(|(id, name)| mark(id, name))
            .collect(),
    };
    Gateway {
        kind,
        designator: r.designator,
        name: r.name.unwrap_or_default(),
        dns: text(r.dns),
        ipv4: text(r.ipv4),
        ipv6: text(r.ipv6),
        // The host file does not publish a URF reflector's M17 port, and
        // urfd listens on 17000 unless its owner moved it. One that did will
        // simply not answer, which is a better failure than dropping every
        // URF reflector from the list.
        port: r.port.unwrap_or(kind.default_port),
        channels,
        password: None,
        sponsor: r.sponsor.unwrap_or_default(),
        country: r.country,
    }
}

/// A module is one letter A to Z. The file has been seen with lower case and
/// with an empty string where a module was removed.
fn letter(s: &str) -> Option<char> {
    let c = s.trim().chars().next()?.to_ascii_uppercase();
    c.is_ascii_uppercase().then_some(c)
}

fn text(s: Option<String>) -> Option<String> {
    s.filter(|v| !v.trim().is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gateways;

    const HOSTS: &str = r#"{"reflectors":[
        {"designator":"M17-USA","name":"M17 USA Reflector","slug":"m17-usa",
         "dns":"m17.w1wra.net","ipv4":"1.2.3.4","ipv6":null,
         "modules":["A","B","C"],"encrypted":["C"],"port":17000,
         "sponsor":"W1WRA","country":"US"},
        {"designator":"M17-IP6","name":null,"dns":null,"ipv4":null,
         "ipv6":"2401:c080:2000:2c78:5400:4ff:fe51:7afe",
         "modules":["A"],"encrypted":[],"port":17001,"sponsor":null,"country":"AU"},
        {"designator":"URF018","name":"REF018","dns":"ref018.dstar.com.br",
         "ipv4":"189.44.229.62","ipv6":null,"sponsor":"PY2PE","country":"BR",
         "modules":[
            {"module":"A","mode":"D-Star (DCS)","transcode":false},
            {"module":"B","mode":"All","description":"Multimode","transcode":true},
            {"module":"M","mode":"M17","transcode":false}]},
        {"designator":"M17-DED","name":"gone","dns":null,"ipv4":null,"ipv6":null,
         "modules":[],"encrypted":[],"port":17000,"sponsor":null,"country":"XX"}]}"#;

    fn parsed() -> Vec<Gateway> {
        parse(&gateways::M17, HOSTS.as_bytes()).unwrap()
    }

    #[test]
    fn a_gateway_carries_the_address_port_and_channels_needed_to_connect() {
        let g = &parsed()[0];
        assert_eq!(g.designator, "M17-USA");
        assert_eq!(g.kind.name, "M17");
        assert_eq!(g.host(), Some("m17.w1wra.net"), "a name outlives the address behind it");
        assert_eq!(g.port, 17000);
        assert_eq!(g.channels.len(), 3);
        assert_eq!(g.channel("a").map(|c| c.id.as_str()), Some("A"));
    }

    #[test]
    fn a_channel_that_passes_encrypted_traffic_says_so() {
        let g = &parsed()[0];
        assert!(!g.channel("A").unwrap().encrypted);
        assert!(g.channel("C").unwrap().encrypted, "listed in encrypted");
    }

    #[test]
    fn a_gateway_reachable_only_over_ipv6_still_has_a_host() {
        let g = &parsed()[1];
        assert_eq!(g.host(), Some("2401:c080:2000:2c78:5400:4ff:fe51:7afe"));
        assert_eq!(g.port, 17001, "not every reflector is on 17000");
        assert_eq!(g.name, "", "a published null is no name, not a missing row");
    }

    #[test]
    fn only_the_modules_of_a_urf_reflector_that_carry_m17_are_kept() {
        let g = &parsed()[2];
        let ids: Vec<&str> = g.channels.iter().map(|c| c.id.as_str()).collect();
        assert_eq!(ids, ["B", "M"], "All transcodes, M17 is native, DCS is not either");
        assert_eq!(g.channel("B").unwrap().name, "Multimode");
    }

    #[test]
    fn a_urf_reflector_without_a_published_port_is_assumed_to_be_on_the_default() {
        assert_eq!(parsed()[2].port, gateways::M17.default_port);
    }

    #[test]
    fn a_gateway_with_no_address_is_listed_as_unreachable_rather_than_dropped() {
        let all = parsed();
        assert_eq!(all.len(), 4);
        assert_eq!(all[3].host(), None);
        assert!(all[3].channels.is_empty());
    }

    #[test]
    fn an_empty_host_file_is_an_error_rather_than_a_network_with_no_gateways() {
        assert!(parse(&gateways::M17, br#"{"reflectors":[]}"#).is_err());
    }
}
