//! Decode rtl_433's own recordings and compare field for field with its output.
//!
//! `testdata/fetch.sh` pulls a selection of captures from the `rtl_433_tests`
//! corpus, each with the reference JSON rtl_433 produces for it. That reference
//! is what makes this worth running: the expected values come from a separate
//! implementation, so agreement is evidence rather than a restatement of our
//! own assumptions, and it is the difference between a decoder that has been
//! exercised and one that has been verified.
//!
//! The fixtures are absent from a fresh clone, so the tests skip when they are
//! missing.

mod corpus;

use corpus::{describe, fixtures};

/// Decodes rtl_433 found that this receiver is known not to find, with the
/// reason for each. The middle field matches against the reference line, so
/// `id:9884` picks out one device in a capture that holds two.
///
/// Checked in both directions: an entry that starts decoding fails the test as
/// loudly as a decode that stops working, because a gap quietly closed and
/// left on the list is how a limitation outlives the code that caused it.
const KNOWN_GAPS: &[(&str, &str, &str)] = &[(
    "acurite_tower_b",
    "id:9884",
    "two sensors transmit inside one burst and a protocol reports the first \
     frame it finds, so the second device in a package is lost",
)];

/// Every decode rtl_433 found must also be found here, with the same values.
#[test]
fn agrees_with_rtl_433() {
    let fixtures = fixtures();
    if fixtures.is_empty() {
        eprintln!("skipping: no rtl_433 fixtures, run testdata/fetch.sh to enable");
        return;
    }

    let mut failures = Vec::new();
    for f in &fixtures {
        let reports = f.decode();
        for want in &f.expected {
            let found = reports.iter().any(|r| want.matches(r));
            let gap = KNOWN_GAPS
                .iter()
                .find(|(file, marker, _)| f.name.contains(file) && want.source.contains(marker));
            match (found, gap) {
                (false, None) => failures.push(format!(
                    "{}: nothing matches {want}\n    decoded: {}",
                    f.name,
                    describe(&reports)
                )),
                (false, Some((_, _, why))) => {
                    eprintln!("known gap in {}: {want}\n    {why}", f.name)
                }
                (true, Some(_)) => failures.push(format!(
                    "{}: {want} now decodes; remove its KNOWN_GAPS entry",
                    f.name
                )),
                (true, None) => {}
            }
        }
    }
    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}

/// Nothing claiming an integrity check may claim a burst rtl_433 read as
/// something else.
///
/// This is the half of verification the reference JSON is usually not used
/// for, and the more valuable half for a receiver meant to identify unknown
/// signals: a decoder that finds the right sensor and three imaginary ones has
/// not really decoded anything. The bar is set at protocols that report a
/// passing integrity check, because those are the ones a user is entitled to
/// believe. The checksum-free fixed-code remotes will claim almost anything by
/// design, which is why they report `crc_valid: None` and why the packet list
/// shows that distinction rather than hiding it.
#[test]
fn invents_nothing() {
    let fixtures = fixtures();
    if fixtures.is_empty() {
        eprintln!("skipping: no rtl_433 fixtures, run testdata/fetch.sh to enable");
        return;
    }

    let mut failures = Vec::new();
    for f in &fixtures {
        for r in f.decode() {
            if r.crc_valid != Some(true) || f.rtl_433_saw(r.model) {
                continue;
            }
            failures.push(format!(
                "{}: claimed {r}, which rtl_433 did not see here (it read {})",
                f.name,
                f.models().join(", ")
            ));
        }
    }
    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}

/// What the corpus covers, printed rather than asserted. Run with
/// `--ignored --nocapture` when adding a protocol or a capture.
#[test]
#[ignore]
fn coverage() {
    for f in fixtures() {
        println!("{}", f.name);
        println!("  rtl_433:  {}", f.models().join(", "));
        println!("  here:     {}", describe(&f.decode()));
        if !f.unsupported.is_empty() {
            println!("  no decoder for: {}", f.unsupported.join(", "));
        }
    }
}
