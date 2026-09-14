//! ACARS against acarsdec's own recording.
//!
//! `testdata/acars_acarsdec_12500.wav` is four channels of off-air airband
//! audio shipped with Thierry Leconte's acarsdec. acarsdec 3.7, built from the
//! commit the manifest pins, reads seven messages out of it, and those seven
//! are what is asserted here: the registrations, the flight numbers, the
//! labels and the message numbers all come from that decoder rather than from
//! this one, so agreement is evidence.
//!
//! The reference output, `acarsdec -o 2 -f test.wav`, with the channel each
//! message was heard on:
//!
//! ```text
//! #2  Mode E  Label 5V  Id 4  Nak  PH-BXR  KL1681  S53A
//! #2  Mode E  Label Q0  Id 6  Nak  LN-DYY  DY083J  S47A
//! #4  Mode 2  Label Q0  Id 4  Nak  LN-DYY  DY083J  S46A
//! #1  Mode G  Label H1  Id 3  Nak  F-GTAE  AF7728  D65C  (an engine report)
//! #1  Mode x  Label _d  Id A  Ack 5  LN-DYY
//! #3  Mode 2  Label _d  Id 0  Ack W  G-DBCK  BA031T  S64A
//! #3  Mode E  Label Q0  Id 9  Nak  G-DBCK  BA031T  S63A
//! ```
//!
//! The fixture is absent from a fresh clone, so this skips when it is missing.

use decode::acars::{Framer, Message, parse};
use dsp::msk::{MskConfig, MskDemod};

/// One channel's worth of the recording, at the rate the file was made at.
fn channels() -> Option<(f64, Vec<Vec<f32>>)> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/acars_acarsdec_12500.wav");
    let raw = std::fs::read(path).ok()?;
    let count = u16::from_le_bytes([raw[22], raw[23]]) as usize;
    let rate = u32::from_le_bytes([raw[24], raw[25], raw[26], raw[27]]) as f64;
    // Walk the chunks rather than assuming the header's length: a writer is
    // free to put anything before the samples.
    let mut at = 12;
    let body = loop {
        let len = u32::from_le_bytes([raw[at + 4], raw[at + 5], raw[at + 6], raw[at + 7]]) as usize;
        if &raw[at..at + 4] == b"data" {
            break &raw[at + 8..at + 8 + len];
        }
        at += 8 + len;
    };
    let samples: Vec<f32> =
        body.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0).collect();
    let split =
        (0..count).map(|c| samples.iter().skip(c).step_by(count).copied().collect()).collect();
    Some((rate, split))
}

/// Every message in the recording, with the channel it was heard on.
fn messages() -> Option<Vec<(usize, Message)>> {
    let (rate, channels) = channels()?;
    let mut out = Vec::new();
    for (n, audio) in channels.iter().enumerate() {
        let mut bits = Vec::new();
        MskDemod::new(rate, MskConfig::ACARS).process(audio, &mut bits);
        let mut blocks = Vec::new();
        Framer::new().process(&bits, &mut blocks);
        for b in &blocks {
            if let Some(m) = parse(b) {
                out.push((n + 1, m));
            }
        }
    }
    Some(out)
}

#[test]
fn the_seven_messages_acarsdec_reads_are_read_here_too() {
    let Some(got) = messages() else {
        eprintln!("skipping: acars_acarsdec_12500.wav absent, run testdata/fetch.sh");
        return;
    };
    for (channel, m) in &got {
        eprintln!(
            "#{channel} mode {} {} {} id {} no {:?} flight {:?}",
            m.mode, m.registration, m.label, m.block_id, m.number, m.flight
        );
    }
    // The count is pinned, not a floor: the file is fixed, so anything that
    // reads six or eight of these has changed what the receiver hears.
    assert_eq!(got.len(), 7, "acarsdec reads seven messages in this recording");

    /// Channel, mode, registration, label, block id, message number, flight.
    type Row =
        (usize, char, &'static str, &'static str, char, Option<&'static str>, Option<&'static str>);
    let want: [Row; 7] = [
        (1, 'G', "F-GTAE", "H1", '3', Some("D65C"), Some("AF7728")),
        (1, 'x', "LN-DYY", "_d", 'A', None, None),
        (2, 'E', "PH-BXR", "5V", '4', Some("S53A"), Some("KL1681")),
        (2, 'E', "LN-DYY", "Q0", '6', Some("S47A"), Some("DY083J")),
        (3, '2', "G-DBCK", "_d", '0', Some("S64A"), Some("BA031T")),
        (3, 'E', "G-DBCK", "Q0", '9', Some("S63A"), Some("BA031T")),
        (4, '2', "LN-DYY", "Q0", '4', Some("S46A"), Some("DY083J")),
    ];
    for (channel, mode, reg, label, id, number, flight) in want {
        let found = got.iter().any(|(c, m)| {
            *c == channel
                && m.mode == mode
                && m.registration == reg
                && m.label == label
                && m.block_id == id
                && m.number.as_deref() == number
                && m.flight.as_deref() == flight
        });
        assert!(found, "nothing on channel {channel} reads as {reg} {label} {id}");
    }
}

#[test]
fn the_engine_report_reads_as_the_text_it_carries() {
    let Some(got) = messages() else {
        eprintln!("skipping: acars_acarsdec_12500.wav absent, run testdata/fetch.sh");
        return;
    };
    // acarsdec prints this one's text in full, so it is the message that says
    // the body survived the parity strip and not only the header.
    let (_, m) = got.iter().find(|(_, m)| m.registration == "F-GTAE").expect("the Air France H1");
    assert!(
        m.text.starts_with("#DFB00000/V206,05,124,183,02,00,00000/V3XX"),
        "the engine report reads as {:?}",
        m.text
    );
    assert!(m.text.ends_with("/V8042,083,00061,22222222222111/"), "text ends as {:?}", m.text);
}
