//! Where Wifi can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::wifi::CHANNEL_WIDTH_HZ;
use dsp::wifi::ofdm;
use dsp::wifi::{WifiConfig, WifiFrame, WifiSpan};

/// 802.11a/g as the auto node and the tables know it: one 20 MHz channel,
/// read off the span because there is no narrower stream an OFDM frame can
/// be cut into.
pub struct Wifi;

impl Signal for Wifi {
    fn id(&self) -> &'static str {
        "wifi"
    }

    fn label(&self) -> &'static str {
        "wifi"
    }

    fn placement(&self) -> Placement {
        Placement::Channels(channels())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: ofdm::RATE_HZ,
            feed_rate_hz: ofdm::RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// Every 20 MHz channel the span covers, and the ones a beacon names as
    /// it reads: a network says which channel it is on, which is how the
    /// receiver finds the channels its starting list did not hold.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let cfg = WifiConfig::default();
        let Some(mut span) = WifiSpan::new(rate_hz, center_hz, &channels(), cfg) else {
            return Reading::default();
        };
        let mut frames: Vec<WifiFrame> = Vec::new();
        let mut block = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            block.clear();
            span.process(b, &mut block);
            for hz in block
                .iter()
                .filter(|f| f.fcs_ok)
                .filter_map(|f| decode::wifi::parse(&f.psdu))
                .filter_map(|m| m.network.and_then(|n| n.channel))
                .filter_map(dsp::wifi::channel_2ghz)
                .collect::<Vec<_>>()
            {
                span.open(hz, cfg);
            }
            frames.append(&mut block);
        }
        frames
            .iter()
            .filter(|f| f.fcs_ok)
            .filter_map(|f| decode::wifi::decoded(&f.psdu, common::Hz(f.center_hz as u64)))
            .collect::<Vec<_>>()
            .into()
    }
}

/// Every 20 MHz channel this can be placed on: the 2.4 GHz band and the
/// 5 GHz one.
pub fn channels() -> Vec<f64> {
    let mut v: Vec<f64> = (1..=13).filter_map(dsp::wifi::channel_2ghz).collect();
    v.push(2_484_000_000.0);
    v.extend(dsp::wifi::channels_5ghz());
    v
}

/// Channel 6, which is where a 2.4 GHz receiver that has to pick one sits.
pub const DEFAULT_HZ: f64 = 2_437_000_000.0;
