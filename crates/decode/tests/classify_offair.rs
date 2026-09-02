//! Score the blind modulation classifier against captures of the wider world.
//!
//! `classify_corpus` beside this one scores the same classifier against
//! rtl_433's recordings, which are amplitude and frequency keyed ISM devices
//! in narrow channels. That is most of what this receiver hears and none of
//! what else exists: no chirp, no multi-carrier, no spread spectrum, and
//! nothing wider than a few tens of kilohertz.
//!
//! These captures are the other half. A Meshtastic node on the EU868 plan, an
//! LTE downlink, 802.11 frames, Bluetooth advertising, impulsive interference
//! and an empty band, recorded here with a LimeSDR and a HackRF. Their labels
//! come from the transmitters being known rather than from a decode: the
//! Meshtastic node's spreading factor was set by hand, and a beacon interval
//! of 102.4 ms is not something else.
//!
//! Two captures in `offair.toml` are deliberately not scored. One holds three
//! systems at once and is labelled `mixed`, because no single family is true
//! of it. The FM broadcast sits at 7 dB in an antenna cut for 868 MHz, and
//! every measurement made of it says noise, which is arguably the right answer
//! rather than a miss.
//!
//! The fixtures are absent from a fresh clone, so this skips when they are
//! missing.

use common::{SampleFormat, C32};
use dsp::{ClassifyConfig, Classifier, Modulation};
use std::path::{Path, PathBuf};

/// What each capture's family means in terms of the classifier's classes.
///
/// `NoiseLike` is accepted for OFDM and DSSS because the class exists to cover
/// exactly the signals that cannot be told apart without finding a repeat, and
/// scoring it wrong for one of them would mark the classifier down for a
/// distinction it says up front it does not always draw.
fn accepts(m: Modulation, family: &str) -> bool {
    matches!(
        (m, family),
        (Modulation::Ook | Modulation::Ask, "ook_ask")
            | (Modulation::Fsk2, "fsk")
            | (Modulation::Fsk4, "mfsk")
            | (Modulation::Msk, "msk_gmsk")
            | (Modulation::Psk2 | Modulation::Psk4 | Modulation::Dsss, "psk")
            | (Modulation::Chirp, "chirp")
            | (Modulation::Ofdm | Modulation::NoiseLike, "ofdm")
            | (Modulation::NoiseLike | Modulation::Carrier, "noise")
    )
}

/// Captures the classifier does not read, with the reason, checked in both
/// directions as `classify_corpus` does: an entry that starts working fails
/// as loudly as one that stops.
const KNOWN_MISSES: &[(&str, &str)] = &[
    (
        "fm_broadcast_95.8M_2000k.cs16",
        "broadcast FM at 7 dB through an antenna cut for 868 MHz. Every \
         measurement of it says noise, and at that level so would any other: \
         the capture needs replacing more than the classifier does",
    ),
    (
        "ofdm_wifi_frames_2462M_20000k.cs8",
        "51 microsecond 802.11 frames. The cyclic prefix is there, but a dozen \
         symbols is too few for the repeat to stand clear of the median across \
         lags, so the OFDM hypothesis does not fire and the burst is refused",
    ),
    (
        "fsk_sensor_868.3M_2000k.cs16",
        "an 868 MHz sensor keying 54 kHz apart in a 2 MHz capture, which is the \
         same failure as the LaCrosse entries in classify_corpus: at that span \
         the tone histogram finds one cluster where there are two, and no \
         frequency-keyed hypothesis scores. Refused rather than misrouted",
    ),
    (
        "gfsk_ble_2426M_20000k.cs8",
        "reads MSK on five advertising packets of eleven and refuses the rest. \
         The parameters are right at this sample rate, 996 kbaud and h = 0.47, \
         so what is missing is not the measurement: the weaker packets fall \
         under the score floor. Listed at the level it reaches rather than \
         hidden, because the difference between five of eleven and none is the \
         difference between a gap and a bug",
    ),
    (
        "impulsive_noise_474M_12000k.cs16",
        "switching interference in the UHF television band. Refused, which is \
         defensible for something that is not a transmission at all, but it \
         means the classifier cannot currently say `this is not a signal` in a \
         way a scanner could act on",
    ),
    (
        "quiet_ism_869.525M_2000k.cs16",
        "an empty band, refused rather than named, which is correct. It is \
         listed here because refusing everything and naming nothing scores the \
         same as being right, and the difference should be visible",
    ),
];

struct Capture {
    name: String,
    family: String,
    format: SampleFormat,
    rate: f64,
    burst_us: Option<(f64, f64)>,
    occupancy_min: f32,
    bridge_us: f64,
}

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/offair")
}

#[test]
fn the_classifier_reads_what_was_recorded() {
    let Ok(text) = std::fs::read_to_string(dir().join("../offair.toml")) else {
        eprintln!("off-air manifest absent, skipping");
        return;
    };
    let captures = parse(&text);
    if captures.is_empty() {
        eprintln!("no captures listed, skipping");
        return;
    }

    let mut seen = 0usize;
    let mut lines: Vec<String> = Vec::new();
    let mut failures: Vec<String> = Vec::new();
    let mut unexpected_passes: Vec<&str> = Vec::new();

    for cap in &captures {
        if cap.family == "mixed" {
            continue;
        }
        let Ok(raw) = std::fs::read(dir().join(&cap.name)) else { continue };
        let mut iq: Vec<C32> = Vec::new();
        cap.format.convert(&raw, &mut iq);
        if iq.is_empty() {
            continue;
        }
        seen += 1;

        let mut cls = Classifier::new(
            cap.rate,
            ClassifyConfig {
                channel_hz: cap.rate as f32,
                // A capture, not a channel: the signal is a small part of the
                // span and has to be brought to its own bandwidth first.
                zoom_below: 0.25,
                ..Default::default()
            },
        );

        let (mut ok, mut n) = (0usize, 0usize);
        for (a, b) in bursts(&iq, cap) {
            if let Some((lo, hi)) = cap.burst_us {
                let us = (b - a) as f64 / cap.rate * 1e6;
                if us < lo || us > hi {
                    continue;
                }
            }
            let seg = &iq[a..b];
            let class = cls.classify(seg);
            if cap.occupancy_min > 0.0
                && class.features.bandwidth_hz < cap.occupancy_min * cap.rate as f32
            {
                continue;
            }
            n += 1;
            if accepts(class.modulation, &cap.family) {
                ok += 1;
            }
        }
        if n == 0 {
            continue;
        }
        let share = ok as f32 / n as f32;
        let known = KNOWN_MISSES.iter().find(|(f, _)| *f == cap.name);
        lines.push(format!("  {:<44} {:>3}/{:<3} {:.2}", cap.name, ok, n, share));
        match known {
            Some(_) if share >= 0.5 => unexpected_passes.push(&cap.name),
            None if share < 0.5 => {
                failures.push(format!("{} read {ok} of {n} correctly", cap.name))
            }
            _ => {}
        }
    }

    if seen == 0 {
        eprintln!("captures absent, run testdata/fetch.sh, skipping");
        return;
    }
    eprintln!("off-air captures, correct of classified:\n{}", lines.join("\n"));

    assert!(failures.is_empty(), "captures newly misread:\n  {}", failures.join("\n  "));
    assert!(
        unexpected_passes.is_empty(),
        "these are on KNOWN_MISSES and now pass; delete the entry:\n  {}",
        unexpected_passes.join("\n  ")
    );
}

/// Cut a capture the way a detector would, or slice it when nothing is bursty.
fn bursts(iq: &[C32], cap: &Capture) -> Vec<(usize, usize)> {
    const BLOCK: usize = 128;
    let power: Vec<f32> = iq
        .chunks_exact(BLOCK)
        .map(|c| c.iter().map(|s| s.norm_sqr()).sum::<f32>() / BLOCK as f32)
        .collect();
    if power.is_empty() {
        return Vec::new();
    }
    let mut sorted = power.clone();
    sorted.sort_by(f32::total_cmp);
    let floor = sorted[sorted.len() / 10].max(1e-20);

    // Continuously occupied captures have no bursts, only crest. Cutting one
    // at its own power dips gives fragments of a transmission, and a fragment
    // shorter than a symbol cannot hold the structure that identifies it.
    let occupied = power.iter().filter(|&&p| p > floor * 2.0).count() as f32 / power.len() as f32;
    if occupied > 0.8 {
        let slice = (iq.len() / 6).max(1 << 15);
        return (0..6)
            .map(|k| (k * iq.len() / 6, (k * iq.len() / 6 + slice).min(iq.len())))
            .filter(|(a, b)| b > a)
            .collect();
    }

    let threshold = floor * 10f32.powf(0.6);
    let bridge = (cap.rate * cap.bridge_us * 1e-6) as usize;
    let mut out: Vec<(usize, usize)> = Vec::new();
    let mut open: Option<usize> = None;
    let mut quiet = 0usize;
    for (i, &p) in power.iter().enumerate() {
        if p > threshold {
            quiet = 0;
            open.get_or_insert(i * BLOCK);
        } else if let Some(s) = open {
            quiet += 1;
            if quiet > 3 {
                let end = (i - quiet) * BLOCK;
                match out.last_mut() {
                    Some(last) if s.saturating_sub(last.1) < bridge => last.1 = end,
                    _ => out.push((s, end)),
                }
                open = None;
            }
        }
    }
    if let Some(s) = open {
        out.push((s, iq.len()));
    }
    out.retain(|(a, b)| b - a >= 256);
    out
}

/// Enough TOML for this flat manifest, and no dependency for it.
fn parse(text: &str) -> Vec<Capture> {
    let mut out = Vec::new();
    let (mut name, mut family, mut format, mut rate) =
        (String::new(), String::new(), String::new(), 0.0f64);
    let (mut burst_us, mut occupancy_min, mut bridge_us) = (None, 0.0f32, 2000.0f64);
    let value = |l: &str| l.split('=').nth(1).unwrap_or("").trim().trim_matches('"').to_string();
    let mut flush = |name: &mut String,
                     family: &mut String,
                     format: &mut String,
                     rate: f64,
                     burst_us: Option<(f64, f64)>,
                     occupancy_min: f32,
                     bridge_us: f64,
                     out: &mut Vec<Capture>| {
        if name.is_empty() || family.is_empty() {
            return;
        }
        out.push(Capture {
            name: std::mem::take(name),
            family: std::mem::take(family),
            format: match std::mem::take(format).as_str() {
                "cs8" => SampleFormat::Cs8,
                "cu8" => SampleFormat::Cu8,
                "cf32" => SampleFormat::Cf32,
                _ => SampleFormat::Cs16,
            },
            rate,
            burst_us,
            occupancy_min,
            bridge_us,
        });
    };
    for line in text.lines() {
        let l = line.trim();
        if l == "[[capture]]" {
            flush(
                &mut name,
                &mut family,
                &mut format,
                rate,
                burst_us,
                occupancy_min,
                bridge_us,
                &mut out,
            );
            rate = 0.0;
            burst_us = None;
            occupancy_min = 0.0;
            bridge_us = 2000.0;
        } else if l.starts_with("name") {
            name = value(l);
        } else if l.starts_with("family") {
            family = value(l);
        } else if l.starts_with("format") {
            format = value(l);
        } else if l.starts_with("rate_sps") {
            rate = value(l).replace('_', "").parse().unwrap_or(0.0);
        } else if l.starts_with("bridge_us") {
            bridge_us = value(l).parse().unwrap_or(2000.0);
        } else if l.starts_with("occupancy_min") {
            occupancy_min = value(l).parse().unwrap_or(0.0);
        } else if l.starts_with("burst_us") {
            let v = value(l);
            let mut p = v.trim_matches(['[', ']'].as_ref()).split(',');
            if let (Some(a), Some(b)) = (p.next(), p.next()) {
                if let (Ok(a), Ok(b)) = (a.trim().parse(), b.trim().parse()) {
                    burst_us = Some((a, b));
                }
            }
        }
    }
    flush(
        &mut name,
        &mut family,
        &mut format,
        rate,
        burst_us,
        occupancy_min,
        bridge_us,
        &mut out,
    );
    out
}
