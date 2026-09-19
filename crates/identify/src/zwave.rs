//! Where ZWave can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::zwave;

pub struct ZWave;

impl Signal for ZWave {
    fn id(&self) -> &'static str {
        "zwave"
    }

    fn label(&self) -> &'static str {
        "zwave"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["z-wave", "g9959"]
    }

    fn placement(&self) -> Placement {
        Placement::Channels(CHANNELS.to_vec())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: 400_000.0,
            feed_rate_hz: WORK_HZ,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        868_420_000.0
    }

    /// All three bit rates at once on the channel the recording is tuned
    /// to: a controller and its devices need not agree, and nothing on the
    /// air says which rate a frame is until it frames.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(mut chan) =
            crate::Channel::new(rate_hz, center_hz, center_hz, CHANNEL_WIDTH_HZ, WORK_HZ)
        else {
            return Reading::default();
        };
        let mut readers: Vec<zwave::Reader> = zwave::RATES
            .iter()
            .map(|(baud, bw, man)| zwave::Reader::new(chan.rate_hz, *baud, *bw, *man))
            .collect();
        let (mut narrow, mut found) = (Vec::new(), Vec::new());
        let mut rows = Vec::new();
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            found.clear();
            for r in &mut readers {
                r.read(&narrow, &mut found);
            }
            for f in &found {
                if let Some(d) = zwave::decoded(&f.bytes, chan.hz()) {
                    rows.push(d);
                }
            }
        }
        rows.into()
    }
}

/// Where Z-Wave is, by region, from the Z-Wave Alliance's frequency chart:
/// 868 MHz across Europe, 908.4 and 916 MHz under FCC part 15.249, 919.8 and
/// 921.4 in Australia and Brazil, and the three Japanese channels. The 100
/// kbit/s channel is a different one from the slower pair in most regions,
/// which is why they are all here rather than one per region.
pub const CHANNELS: [f64; 12] = [
    865_200_000.0,
    868_400_000.0,
    868_420_000.0,
    869_850_000.0,
    908_400_000.0,
    908_420_000.0,
    916_000_000.0,
    919_800_000.0,
    921_400_000.0,
    922_500_000.0,
    923_900_000.0,
    926_300_000.0,
];

/// The width a transmission occupies. The widest of the three is 100 kbit/s
/// keyed 29 kHz either way, which is 158 kHz by Carson; the slower two are
/// keyed 20 kHz either way and are narrower than their own channel spacing.
pub const CHANNEL_WIDTH_HZ: f64 = 200_000.0;

/// Rate the channel is cut down to before the clocks read it: eight samples
/// a symbol at 100 kbit/s, which leaves the widest of the three a transition
/// band and the narrowest more resolution than it can use.
pub const WORK_HZ: f64 = 800_000.0;
