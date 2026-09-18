//! Why a 2.4 GHz front end reads the span instead of being placed on sources.
//!
//! Every other decoder in the receiver is built on a source the detector
//! opened, at the rate that source was cut at. BLE, 802.15.4 and Wi-Fi are
//! not: each mixes and filters the whole span for itself, which on a
//! 61.44 MS/s band is two dozen private channelizers. This says what stops
//! them, so that a later attempt at it starts from the measurement rather
//! than from the guess.
//!
//! The capture is 0.12 s of 2.4 GHz at 20 MS/s holding eight Bluetooth
//! advertisements, of which the front end reads seven
//! (`radio::bluetooth_advertising_is_found_and_read`).
//!
//! Not the frame length, which was the first guess. An advertisement is 128
//! microseconds and a frame here is 819 with a 410 microsecond hop, so it
//! looked as though the detector could not see one at all; it can, because
//! they arrive in runs and one source covers several. Shortening the frame to
//! a 12.8 microsecond hop opens the same three sources.
//!
//! What stops it is the width. A source is the run of bins that stood over
//! the floor, and the twenty decibel extent of a GFSK burst is a few hundred
//! kilohertz where the channel it occupies is two megahertz. `extract::cut`
//! takes the rate from that measurement, so the source arrives at about
//! 1.8 MS/s and a decoder that refuses under four is never a candidate for
//! it.
//!
//! Widening the source to the channel the registry names does make BLE a
//! candidate, and was tried: it reads five distinct advertisements against
//! the span path's seven, each twice over because two overlapping sources on
//! one channel each get a decoder. So the source path has to open one channel
//! rather than one source per burst, and has to keep it open across the gaps,
//! before it can replace the span path.

use common::C32;
use dsp::{SourceConfig, SourceDetector, SourceEvent};

const FIXTURE: &str = "../../testdata/offair/gfsk_ble_2426M_20000k.cs8";

/// What BLE declares, from `nodes::ble_nodes`, which this crate is below and
/// cannot ask.
const BLE_MIN_RATE_HZ: f64 = 4_000_000.0;
const BLE_CHANNEL_HZ: f64 = 2_000_000.0;

fn capture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

/// Sources opened in the capture, as the width each measured.
fn opened(cfg: SourceConfig, samples: &[C32], rate: f64) -> Vec<f64> {
    let mut d = SourceDetector::new(rate, rate, cfg);
    let mut out = Vec::new();
    for block in samples.chunks(131_072) {
        for e in d.process(block) {
            if let SourceEvent::Opened(s) = e {
                // On channel 38, which is the middle of this span.
                if s.center_hz.abs() < 1e6 {
                    out.push(s.bandwidth_hz());
                }
            }
        }
    }
    out
}

#[test]
fn an_advertisement_opens_a_source_too_narrow_to_be_read_as_one() {
    let Some(buf) = capture() else {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh");
        return;
    };
    let rate = buf.rate.as_f64();
    let cfg = SourceConfig { open_db: 15.0, ..Default::default() };
    // 16384 points at 20 MS/s: 1.2 kHz bins and a 410 microsecond hop.
    assert_eq!(cfg.fft_size_at(rate), 16_384);
    // And a frame thirty-two times shorter, which is a 12.8 microsecond hop.
    let short = SourceConfig { fft_size: 512, ..cfg };

    for (name, cfg) in [("the channel frame", cfg), ("a frame for bursts", short)] {
        let widths = opened(cfg, &buf.samples, rate);
        assert_eq!(widths.len(), 3, "{name} opened {} sources on channel 38", widths.len());
        for bw in &widths {
            assert!(*bw < BLE_CHANNEL_HZ, "{name} measured {bw:.0} Hz, the channel after all");
            // The rate `extract::cut` would give it.
            let want = bw * cfg.width_margin * cfg.oversample;
            assert!(
                want < BLE_MIN_RATE_HZ,
                "{name}: a {bw:.0} Hz source is cut at {want:.0} S/s, which BLE would take"
            );
        }
    }
}
