//! Every capture in the manifests says what it may be used for.
//!
//! The recordings are published separately from the repository, so a reader
//! has nothing but the manifest to go on: see `testdata/LICENSE.md`, which
//! this pins against. A fixture re-hosted on nostr.download under somebody
//! else's terms is the case worth catching, so the check is on the whole
//! manifest rather than on the entries somebody remembered to label.

use std::path::{Path, PathBuf};

/// Licence of a recording, as a manifest may state it
#[derive(Debug, PartialEq, Eq)]
enum Licence {
    /// Recorded or generated for this project
    CcBy4,
    /// Somebody else's, under the licence they state
    Upstream(String),
    /// Somebody else's, with no terms stated anywhere
    Unstated,
}

impl Licence {
    fn parse(s: &str) -> Licence {
        match s {
            "CC-BY-4.0" => Licence::CcBy4,
            "unstated" => Licence::Unstated,
            other => Licence::Upstream(other.to_string()),
        }
    }
}

struct Entry {
    name: String,
    url: String,
    licence: Option<Licence>,
    source: Option<String>,
}

fn testdata() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata")
}

fn parse(text: &str, header: &str) -> Vec<Entry> {
    let mut out: Vec<Entry> = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if line == header {
            out.push(Entry {
                name: String::new(),
                url: String::new(),
                licence: None,
                source: None,
            });
            continue;
        }
        let Some(entry) = out.last_mut() else {
            continue;
        };
        let Some((key, value)) = line.split_once(" = ") else {
            continue;
        };
        let value = value.trim().trim_matches('"').to_string();
        match key {
            "name" => entry.name = value,
            "url" => entry.url = value,
            "license" => entry.licence = Some(Licence::parse(&value)),
            "license_source" => entry.source = Some(value),
            _ => {}
        }
    }
    out
}

fn manifest(name: &str) -> String {
    std::fs::read_to_string(testdata().join(name)).expect("manifest is committed")
}

#[test]
fn every_capture_says_what_it_may_be_used_for() {
    let fixtures = parse(&manifest("fixtures.toml"), "[[capture]]");
    let offair = parse(&manifest("offair.toml"), "[[capture]]");
    let survey = parse(&manifest("survey.toml"), "[[dataset]]");
    assert_eq!(fixtures.len(), 24, "captures in fixtures.toml");
    assert_eq!(offair.len(), 13, "captures in offair.toml");
    assert_eq!(survey.len(), 1, "datasets in survey.toml");

    for entry in fixtures.iter().chain(&offair).chain(&survey) {
        assert!(entry.licence.is_some(), "{} carries no license field", entry.name);
    }

    let ours = |set: &[Entry]| set.iter().filter(|e| e.licence == Some(Licence::CcBy4)).count();
    // Fourteen recordings and five signals generated here from a known picture.
    assert_eq!(ours(&fixtures), 19, "CC BY 4.0 fixtures");
    assert_eq!(ours(&offair), 13, "CC BY 4.0 off-air captures");

    let mut foreign: Vec<(&str, &Licence, &str)> = fixtures
        .iter()
        .filter(|e| e.licence != Some(Licence::CcBy4))
        .map(|e| (e.name.as_str(), e.licence.as_ref().unwrap(), e.source.as_deref().unwrap_or("")))
        .collect();
    foreign.sort_by_key(|(name, _, _)| *name);
    let names: Vec<&str> = foreign.iter().map(|(n, _, _)| *n).collect();
    assert_eq!(
        names,
        [
            "acars_acarsdec_12500.wav",
            "dvbt_hd_429M_9142857.cs8",
            "rs41_herstmonceux_405.80024M_31.25k.cs16",
            "sstv_martin1_44100.wav",
            "vdl2_model_136.975M_1050k.wav",
        ]
    );
    let stated: Vec<&Licence> = foreign.iter().map(|(_, l, _)| *l).collect();
    assert_eq!(
        stated,
        [
            &Licence::Upstream("LGPL-2.0-only".into()),
            &Licence::Unstated,
            &Licence::Unstated,
            &Licence::Upstream("GPL-3.0".into()),
            &Licence::Upstream("GPL-3.0".into()),
        ]
    );
    for (name, _, source) in &foreign {
        assert!(!source.is_empty(), "{name} names no license_source");
    }
}

#[test]
fn nothing_of_somebody_elses_is_re_hosted_without_naming_them() {
    let fixtures = parse(&manifest("fixtures.toml"), "[[capture]]");
    let rehosted: Vec<&Entry> =
        fixtures.iter().filter(|e| e.url.contains("nostr.download")).collect();
    assert_eq!(rehosted.len(), 22, "fixtures re-hosted on nostr.download");
    for entry in &rehosted {
        match entry.licence {
            Some(Licence::CcBy4) => {}
            _ => assert!(
                entry.source.is_some(),
                "{} is re-hosted under somebody else's terms with no source named",
                entry.name
            ),
        }
    }

    // The rtl_433 corpus states no licence, so it is pointed at and never
    // copied: every URL in it is upstream's.
    let corpus = manifest("rtl433.toml");
    let urls = corpus
        .lines()
        .filter(|l| l.trim_start().starts_with("url = ") || l.contains("reference_url = "))
        .count();
    assert_eq!(urls, 184, "URLs in rtl433.toml");
    assert!(!corpus.contains("nostr.download"), "an rtl_433 capture has been re-hosted");
    assert!(
        !manifest("survey.toml").contains("nostr.download"),
        "the survey dataset has been re-hosted"
    );
}
