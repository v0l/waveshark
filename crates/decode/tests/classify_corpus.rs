//! Score the blind modulation classifier against real recordings.
//!
//! The devices in rtl_433's corpus are known, so their modulation is known:
//! `docs/protocols.md` names it per family, taken from rtl_433's own device
//! table. That makes these 52 captures a labelled set for a classifier that is
//! otherwise tuned entirely against signals this project generated itself,
//! which is the same weakness the protocol table calls **synthetic** and the
//! same reason that label exists.
//!
//! What is checked is the *family*, amplitude keyed against frequency keyed,
//! because that is the decision the router actually makes: it picks a front
//! end, and there is one for each. Whether an OOK burst is PWM or PPM is the
//! slicer's business and is not visible here.
//!
//! The fixtures are absent from a fresh clone, so this skips when they are
//! missing.

mod corpus;

use common::C32;
use dsp::{ClassifyConfig, Classifier, Modulation};

/// The captures whose device transmits FSK. Everything else in the corpus is
/// on-off keyed. Both lists come from `docs/protocols.md`, which took them
/// from rtl_433's device definitions.
const FSK_CAPTURES: &[&str] = &[
    "fineoffset_wh51",
    "lacrosse_tx29it",
    "lacrosse_tx35dthit",
    "tpms_toyota",
];

/// Captures the classifier is known to read wrong, with the reason.
///
/// Checked in both directions, like `KNOWN_GAPS` in the decode corpus: an
/// entry that starts working fails the test as loudly as one that stops,
/// because a limitation left on a list after it is fixed is how a stale
/// excuse outlives the code that earned it.
const KNOWN_MISSES: &[(&str, &str)] = &[
    (
        "bresser_3ch_b_433.92M_250k.cu8",
        "the burst is weak enough that the envelope fits one level rather than \
         two, so nothing scores it as keyed at all. The same sensor's other \
         capture reads correctly",
    ),
    (
        "honeywell_5816_a_344.975M_250k.cu8",
        "every burst in this capture is shorter than the classifier's minimum \
         window, so there is nothing to measure. The decoder reads it: pulse \
         timings need far fewer samples than a spectrum does",
    ),
    (
        "lacrosse_tx29it_a_868.2M_250k.cu8",
        "868 MHz FSK in a 250 kHz span, with the deviation smeared into one \
         cluster by the noise: the tone histogram finds a single peak, so no \
         frequency-keyed hypothesis scores. Refused rather than misrouted, \
         which is the intended failure",
    ),
    (
        "lacrosse_tx35dthit_a_868.2M_250k.cu8",
        "as the TX29-IT above: one tone cluster where there are two",
    ),
    (
        "lacrosse_tx29it_b_868.2M_1000k.cu8",
        "recorded at 1 MS/s, where the packet occupies an eighth of the span \
         and its envelope reads as shallow amplitude keying. A channelizer \
         would hand this over in a 125 kHz channel and the question would not \
         arise; the test harness has no bank",
    ),
    (
        "tpms_toyota_b_433.92M_250k.cu8",
        "one strong spike sets the upper envelope level for the whole window, \
         putting the packet below the threshold the level runs are measured \
         against, so its one long run of carrier reads as many short ones. \
         The other two Toyota captures classify correctly",
    ),
];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Family {
    Amplitude,
    Frequency,
    Neither,
}

fn family(m: Modulation) -> Family {
    match m {
        Modulation::Ook | Modulation::Ask => Family::Amplitude,
        Modulation::Fsk2 | Modulation::Fsk4 | Modulation::Msk => Family::Frequency,
        _ => Family::Neither,
    }
}

#[test]
fn the_classifier_agrees_with_the_device_table() {
    let fixtures = corpus::fixtures();
    if fixtures.is_empty() {
        eprintln!("no fixtures in {}; run testdata/fetch.sh", corpus::dir().display());
        return;
    }

    let mut rows: Vec<(String, Family, Modulation, usize, usize)> = Vec::new();
    for f in &fixtures {
        let expected = if FSK_CAPTURES.iter().any(|k| f.name.starts_with(k)) {
            Family::Frequency
        } else {
            Family::Amplitude
        };
        let (verdict, agreed, total) = classify_capture(&f.path);
        rows.push((f.name.clone(), expected, verdict, agreed, total));
    }

    let mut wrong: Vec<String> = Vec::new();
    let mut counts: std::collections::BTreeMap<(String, String), usize> = Default::default();
    for (name, expected, got, agreed, total) in &rows {
        *counts.entry((format!("{expected:?}"), got.label().to_string())).or_default() += 1;
        let ok = family(*got) == *expected;
        eprintln!(
            "{:>32}  want {:?}  got {:?}  ({agreed}/{total} bursts){}",
            name,
            expected,
            got,
            if ok { "" } else { "   MISS" }
        );
        if !ok {
            wrong.push(name.clone());
        }
    }

    eprintln!("\n{} of {} captures classified into the right family", rows.len() - wrong.len(), rows.len());
    for ((expected, got), n) in &counts {
        eprintln!("  {expected} read as {got}: {n}");
    }

    let known: Vec<&str> = KNOWN_MISSES.iter().map(|(n, _)| *n).collect();
    let unexpected: Vec<&String> = wrong.iter().filter(|n| !known.contains(&n.as_str())).collect();
    let fixed: Vec<&&str> = known.iter().filter(|n| !wrong.iter().any(|w| w == *n)).collect();
    assert!(unexpected.is_empty(), "captures read as the wrong family: {unexpected:?}");
    assert!(fixed.is_empty(), "these are on KNOWN_MISSES but now classify correctly: {fixed:?}");
}

/// Classify every burst in a capture and take the majority verdict.
///
/// Each burst is mixed to its own centre first, exactly as the channelizer
/// would place it in a channel: these captures are recorded on the nominal
/// band frequency and the transmitters sit tens of kilohertz off it, which
/// tilts every spectral feature if left alone.
fn classify_capture(path: &std::path::Path) -> (Modulation, usize, usize) {
    let src = sources::FileSource::open(path).expect("open capture");
    let buf = src.read_all().expect("read capture");
    let rate = buf.rate.as_f64();

    let pkgs = corpus::packages(path);
    let windows = corpus::windows(&pkgs, rate, buf.samples.len());

    let cfg = ClassifyConfig { channel_hz: rate as f32, ..Default::default() };
    let mut classifier = Classifier::new(rate, cfg);
    let mut votes: std::collections::BTreeMap<String, (usize, Modulation)> = Default::default();
    let mut total = 0;
    let mut shifted: Vec<C32> = Vec::new();
    for (a, b) in windows {
        let burst = &buf.samples[a..b];
        if burst.len() < 2048 {
            continue;
        }
        let offset = corpus::carrier_offset(burst, rate);
        shifted.clear();
        dsp::Mixer::new(-offset, rate).process(burst, &mut shifted);
        let c = classifier.classify(&shifted);
        total += 1;
        if c.modulation != Modulation::Unknown {
            let e = votes.entry(format!("{:?}", c.modulation)).or_insert((0, c.modulation));
            e.0 += 1;
        }
    }
    match votes.values().max_by_key(|(n, _)| *n) {
        Some((n, m)) => (*m, *n, total),
        None => (Modulation::Unknown, 0, total),
    }
}
