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

fn published() -> Vec<Entry> {
    let mut all = parse(&manifest("decode.toml"), "[[capture]]");
    all.extend(parse(&manifest("fixture.toml"), "[[capture]]"));
    all
}

fn ours(e: &Entry) -> bool {
    e.licence == Some(Licence::CcBy4) && e.source.is_none()
}

#[test]
fn every_capture_says_what_it_may_be_used_for() {
    let decode = parse(&manifest("decode.toml"), "[[capture]]");
    let fixture = parse(&manifest("fixture.toml"), "[[capture]]");
    assert_eq!(decode.len(), 10, "captures in decode.toml");
    assert_eq!(fixture.len(), 42, "captures in fixture.toml");

    let all = published();
    for entry in &all {
        assert!(entry.licence.is_some(), "{} carries no license field", entry.name);
    }
    assert_eq!(all.iter().filter(|e| ours(e)).count(), 38, "CC BY 4.0 recordings of our own");

    let mut foreign: Vec<(&str, &Licence, &str)> = all
        .iter()
        .filter(|e| !ours(e))
        .map(|e| (e.name.as_str(), e.licence.as_ref().unwrap(), e.source.as_deref().unwrap_or("")))
        .collect();
    foreign.sort_by_key(|(name, _, _)| *name);
    let names: Vec<&str> = foreign.iter().map(|(n, _, _)| *n).collect();
    assert_eq!(
        names,
        [
            "acars_acarsdec_12500.wav",
            "aero_oqpsk_1546M_48k.cs16",
            "dab_melbourne_9a_202.928M_2500k.cs16",
            "drm_b_3.965M_48k.cs16",
            "dvbt_hd_429M_9142857.cs8",
            "eas_tor_kilx_22050.wav",
            "ft8_wsjtx_210703_133430_12000.wav",
            "nxdn48_453M_48k.cs16",
            "nxdn96_453M_48k.cs16",
            "rs41_herstmonceux_405.80024M_31.25k.cs16",
            "sstv_martin1_44100.wav",
            "stdc_egc_1541.45M_48k.cs16",
            "survey/lora_salvora.csv",
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
            &Licence::Unstated,
            &Licence::Unstated,
            &Licence::Unstated,
            &Licence::Upstream("GPL-3.0".into()),
            &Licence::Unstated,
            &Licence::Unstated,
            &Licence::Unstated,
            &Licence::Upstream("GPL-3.0".into()),
            &Licence::Unstated,
            &Licence::CcBy4,
            &Licence::Upstream("GPL-3.0".into()),
        ]
    );
    for (name, _, source) in &foreign {
        assert!(!source.is_empty(), "{name} names no license_source");
    }
}

#[test]
fn nothing_of_somebody_elses_is_re_hosted_without_naming_them() {
    let all = published();
    let rehosted: Vec<&Entry> = all.iter().filter(|e| e.url.contains("nostr.download")).collect();
    assert_eq!(rehosted.len(), 48, "captures re-hosted on nostr.download");
    for entry in &rehosted {
        assert!(
            ours(entry) || entry.source.is_some(),
            "{} is re-hosted under somebody else's terms with no source named",
            entry.name
        );
    }
    let survey = all.iter().find(|e| e.name.starts_with("survey/")).expect("the survey dataset");
    assert!(!survey.url.contains("nostr.download"), "the survey dataset has been re-hosted");

    // The rtl_433 corpus states no licence, so it is pointed at and never
    // copied: every URL in it is upstream's.
    let corpus = manifest("rtl433.toml");
    let urls = corpus
        .lines()
        .filter(|l| l.trim_start().starts_with("url = ") || l.contains("reference_url = "))
        .count();
    assert_eq!(urls, 194, "URLs in rtl433.toml");
    assert!(!corpus.contains("nostr.download"), "an rtl_433 capture has been re-hosted");
}

#[test]
fn a_local_capture_names_no_place_it_was_published() {
    let local = manifest("local.toml");
    let entries = local.lines().filter(|l| l.trim() == "[[capture]]").count();
    assert_eq!(entries, 15, "captures in local.toml");
    let keys: std::collections::BTreeSet<&str> =
        local.lines().filter_map(|l| l.split_once(" = ").map(|(k, _)| k.trim())).collect();
    assert_eq!(keys, ["name", "sha256", "size"].into_iter().collect());
}
