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
//! Two captures in `testdata/offair` are deliberately not scored. One holds
//! three systems at once and is left out of `CAPTURES`, because no single
//! family is true of it. The FM broadcast sits at 7 dB in an antenna cut for 868 MHz, and
//! every measurement made of it says noise, which is arguably the right answer
//! rather than a miss.
//!
//! The fixtures are absent from a fresh clone, so this skips when they are
//! missing.

use common::C32;
use dsp::{Classifier, ClassifyConfig, Modulation};
use std::path::{Path, PathBuf};

/// What each capture's family means in terms of the classifier's classes.
///
/// `NoiseLike` is accepted for OFDM and DSSS because the class exists to cover
/// exactly the signals that cannot be told apart without finding a repeat, and
/// scoring it wrong for one of them would mark the classifier down for a
/// distinction it says up front it does not always draw.
fn accepts(m: Modulation, family: Family) -> bool {
    matches!(
        (m, family),
        (Modulation::Fsk2, Family::Fsk)
            | (Modulation::Msk, Family::MskGmsk)
            | (Modulation::Chirp, Family::Chirp)
            | (Modulation::Ofdm | Modulation::NoiseLike, Family::Ofdm)
            | (Modulation::NoiseLike | Modulation::Carrier, Family::Noise)
    )
}

#[derive(Clone, Copy)]
enum Family {
    Fm,
    Fsk,
    MskGmsk,
    Chirp,
    Ofdm,
    Noise,
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
    name: &'static str,
    family: Family,
    burst_us: Option<(f64, f64)>,
    occupancy_min: f32,
    bridge_us: f64,
}

const fn labelled(name: &'static str, family: Family) -> Capture {
    Capture { name, family, burst_us: None, occupancy_min: 0.0, bridge_us: 2000.0 }
}

const CAPTURES: &[Capture] = &[
    labelled("fm_broadcast_95.8M_2000k.cs16", Family::Fm),
    labelled("fsk_sensor_868.3M_2000k.cs16", Family::Fsk),
    Capture {
        burst_us: Some((80.0, 500.0)),
        ..labelled("gfsk_ble_2426M_20000k.cs8", Family::MskGmsk)
    },
    labelled("impulsive_noise_474M_12000k.cs16", Family::Noise),
    labelled("lora_sf11_meshtastic_a_869.525M_2000k.cs16", Family::Chirp),
    labelled("lora_sf11_meshtastic_b_869.525M_2000k.cs16", Family::Chirp),
    Capture {
        burst_us: Some((300_000.0, 700_000.0)),
        ..labelled("lora_sf11_meshtastic_c_869.0M_2400k.cu8", Family::Chirp)
    },
    Capture {
        burst_us: Some((900_000.0, 1_400_000.0)),
        ..labelled("meshcore_advert_868.9M_2048k.cu8", Family::Chirp)
    },
    labelled("ofdm_lte_762M_12000k.cs16", Family::Ofdm),
    Capture {
        burst_us: Some((20.0, 500.0)),
        occupancy_min: 0.25,
        bridge_us: 200.0,
        ..labelled("ofdm_wifi_frames_2462M_20000k.cs8", Family::Ofdm)
    },
    labelled("quiet_ism_869.525M_2000k.cs16", Family::Noise),
    Capture {
        burst_us: Some((3000.0, 12_000.0)),
        bridge_us: 500.0,
        ..labelled("elrs_100hz_2415M_20000k.cs8", Family::Chirp)
    },
];

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/offair")
}

#[test]
fn the_classifier_reads_what_was_recorded() {
    let mut seen = 0usize;
    let mut lines: Vec<String> = Vec::new();
    let mut failures: Vec<String> = Vec::new();
    let mut unexpected_passes: Vec<&str> = Vec::new();

    for cap in CAPTURES {
        let Ok(buf) = sources::FileSource::open(dir().join(cap.name)).and_then(|s| s.read_all())
        else {
            continue;
        };
        let rate = buf.rate.as_f64();
        let iq = buf.samples;
        if iq.is_empty() {
            continue;
        }
        seen += 1;

        let mut cls = Classifier::new(
            rate,
            ClassifyConfig {
                channel_hz: rate as f32,
                // A capture, not a channel: the signal is a small part of the
                // span and has to be brought to its own bandwidth first.
                zoom_below: 0.25,
                ..Default::default()
            },
        );

        let (mut ok, mut n) = (0usize, 0usize);
        for (a, b) in bursts(&iq, cap, rate) {
            if let Some((lo, hi)) = cap.burst_us {
                let us = (b - a) as f64 / rate * 1e6;
                if us < lo || us > hi {
                    continue;
                }
            }
            let seg = &iq[a..b];
            let class = cls.classify(seg);
            if cap.occupancy_min > 0.0
                && class.features.bandwidth_hz < cap.occupancy_min * rate as f32
            {
                continue;
            }
            n += 1;
            if accepts(class.modulation, cap.family) {
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
            Some(_) if share >= 0.5 => unexpected_passes.push(cap.name),
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
fn bursts(iq: &[C32], cap: &Capture, rate: f64) -> Vec<(usize, usize)> {
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
    let bridge = (rate * cap.bridge_us * 1e-6) as usize;
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
