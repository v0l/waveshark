//! Where Lora can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use common::bands::Usage;

/// LoRa as the auto node knows it: placed on the classifier's verdict, not
/// on width, because it is the dearest decoder to run and a chirp is the
/// one thing the classifier names reliably. On a band of hard-keyed sensors
/// most sources measure over 44 kHz from their splatter, and dechirping six
/// spreading factors on each of them was the largest line on a busy span.
pub struct Lora;

impl Signal for Lora {
    fn id(&self) -> &'static str {
        "lora"
    }

    fn label(&self) -> &'static str {
        "lora"
    }

    fn placement(&self) -> Placement {
        Placement::Usage(&[Usage::Ism, Usage::Wlan])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &ALL_BANDWIDTHS_HZ,
            min_rate_hz: 0.0,
            feed_rate_hz: 0.0,
            span_wide: false,
            families: &[dsp::Modulation::Chirp],
        }
    }

    fn default_hz(&self) -> f64 {
        869_525_000.0
    }

    /// Every standard bandwidth the span could hold, the one that read
    /// most kept.
    ///
    /// A recording says nothing about its bandwidth or spreading factor, so
    /// each bandwidth is designed for in turn and the reader searches every
    /// factor within it. This is the dear one: the receiver only builds it
    /// once the classifier has named a chirp, and a file has no classifier
    /// in front of it.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut best = Reading::default();
        for bw in nodes_bandwidths(center_hz, rate_hz.min(rate_hz)) {
            let rows = read_at(bw, iq, rate_hz, center_hz);
            if rows.count() > best.count() {
                best = rows;
            }
        }
        best
    }
}

/// Every bandwidth a LoRa channel is read at, for the registry.
pub const ALL_BANDWIDTHS_HZ: [f64; 5] = [62_500.0, 125_000.0, 250_000.0, 500_000.0, 812_500.0];

/// The bandwidths worth trying on a span, widest first.
///
/// Every standard bandwidth that fits inside the recording: a file is not a
/// source the detector measured, so there is no width to narrow this by.
fn nodes_bandwidths(center_hz: f64, rate_hz: f64) -> Vec<f64> {
    let table: &[f64] = match is_2g4(center_hz) {
        true => &BANDWIDTHS_2G4_HZ,
        false => &BANDWIDTHS_HZ,
    };
    let mut out: Vec<f64> = table.iter().copied().filter(|bw| *bw <= rate_hz / 2.0).collect();
    out.sort_by(|a, b| b.total_cmp(a));
    out
}

/// One bandwidth, every spreading factor it uses.
fn read_at(bw: f64, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
    let mut reader = dsp::lora::ChirpReader::new();
    let sfs: Vec<u8> = dsp::lora::SCANNED_SPREADING_FACTORS.collect();
    if reader.design(rate_hz, center_hz, bw, sfs, is_2g4(center_hz)).is_err() {
        return Reading::default();
    }
    let center = common::Hz(center_hz as u64);
    let mut rows = Vec::new();
    for b in iq.chunks(crate::BLOCK) {
        reader.feed(b);
        while let Some(found) = reader.next() {
            let ldro = dsp::lora::ldro_default(found.packet.sf, bw);
            let Ok(frame) = decode::lora::decode(&found.packet.symbols, found.packet.sf, ldro)
            else {
                continue;
            };
            // A frame whose payload CRC did not check, or that has none to
            // check, stands on its eight-bit header checksum alone, which
            // noise passes one time in 256.
            if frame.crc_ok != Some(true)
                && (frame.header.has_crc
                    || !decode::lora::KNOWN_SYNC.contains(&found.packet.sync_word))
            {
                continue;
            }
            let bytes = frame.to_bytes(found.packet.sf, bw, found.packet.sync_word);
            if let Some(d) = decode::lora::decoded(&bytes, center) {
                rows.push(d);
            }
        }
    }
    rows.into()
}

/// The bandwidths LoRa is used at in practice. The standard defines nine,
/// down to 7.8 kHz, but a receiver that offers all of them has to guess
/// between neighbours a few kilohertz apart on a measurement worth rather
/// less than that. 62.5 kHz is MeshCore's European preset (869.618 MHz,
/// SF8), which arrived at 61 dB and was refused as no channel at all.
pub const BANDWIDTHS_HZ: [f64; 4] = [62_500.0, 125_000.0, 250_000.0, 500_000.0];

/// The bandwidth LoRa is used at on 2.4 GHz, where every transmitter is an
/// SX128x: ExpressLRS's LoRa rates are all 812.5 kHz. The chip also does
/// 1625 kHz, but what is sent that wide there is FLRC, which is not LoRa.
pub const BANDWIDTHS_2G4_HZ: [f64; 1] = [812_500.0];

/// Whether a centre is in the 2.4 GHz ISM band, where LoRa is an SX128x and
/// therefore inverted.
pub fn is_2g4(center_hz: f64) -> bool {
    (2_400e6..=2_500e6).contains(&center_hz)
}
