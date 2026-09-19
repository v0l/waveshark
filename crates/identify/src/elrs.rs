//! Where Elrs can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::elrs::{self, SPREADING_FACTORS};

/// ExpressLRS as the auto node knows it: on the 2.4 GHz band, a chirp
/// 812.5 kHz wide, placed once the classifier has named one.
pub struct Elrs;

impl Signal for Elrs {
    fn id(&self) -> &'static str {
        "elrs"
    }

    fn label(&self) -> &'static str {
        "elrs"
    }

    fn placement(&self) -> Placement {
        Placement::Bands(vec![(2_400_000_000.0, 2_483_500_000.0)])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 0.0,
            feed_rate_hz: 0.0,
            span_wide: false,
            families: &[dsp::Modulation::Chirp],
        }
    }

    fn default_hz(&self) -> f64 {
        // The middle of the hop set.
        2_440_400_000.0
    }

    /// A handset's link off the recording, in two passes.
    ///
    /// The link's identifier is not on the air: a packet is validated
    /// against a UID the transmitter and receiver were bound with. So every
    /// packet is read first, the UID recovered from a sync packet or from a
    /// run of consecutive ones, and only then are the packets rows. A
    /// recording is what makes the two passes possible; the receiver has
    /// only the packets it has heard so far.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut reader = dsp::lora::ChirpReader::new();
        let sfs: Vec<u8> = SPREADING_FACTORS.collect();
        if reader.design(rate_hz, center_hz, CHANNEL_WIDTH_HZ, sfs, true).is_err() {
            return Reading::default();
        }
        let ota = elrs::OTA_VERSION;
        let mut heard: Vec<(u8, Vec<u8>)> = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            reader.feed(b);
            while let Some(found) = reader.next() {
                if let Some(bytes) = elrs::payload(&found.packet.symbols, found.packet.sf) {
                    heard.push((found.packet.sf, bytes));
                }
            }
        }
        let Some(uid) = link_uid(&heard, ota) else {
            return Reading::default();
        };
        let center = common::Hz(center_hz as u64);
        heard
            .iter()
            .filter_map(|(sf, packet)| {
                let bytes = elrs::to_bytes(*sf, CHANNEL_WIDTH_HZ, ota, &uid, None, packet);
                elrs::decoded(&bytes, center)
            })
            .collect::<Vec<_>>()
            .into()
    }
}

/// The link every packet is checked against: named by a sync packet if one
/// was heard, else recovered from a run of consecutive packets.
fn link_uid(heard: &[(u8, Vec<u8>)], ota: u8) -> Option<[u8; 6]> {
    for (_, packet) in heard {
        if let Some(uid) = elrs::uid_from_sync(packet, ota) {
            return Some(uid);
        }
    }
    for window in heard.windows(elrs::RECOVER_FROM) {
        let last: Vec<&[u8]> = window.iter().map(|(_, p)| &p[..]).collect();
        if let Some(r) = elrs::recover_link(&last, ota) {
            return Some([0, 0, 0, 0, r.uid4, r.uid5.unwrap_or(0)]);
        }
    }
    None
}

/// The one bandwidth every ExpressLRS LoRa rate on 2.4 GHz uses.
pub const CHANNEL_WIDTH_HZ: f64 = 812_500.0;
