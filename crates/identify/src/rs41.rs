//! Vaisala RS41 radiosondes in the 400 to 406 MHz meteorological band.
//!
//! The waveform is [`dsp::fsk::BitSync`] at 4800 baud, the framing and the
//! frame are [`decode::rs41`], and the only thing here that the receiver's
//! node does differently is finding the channel: the node is handed one the
//! detector opened, and a recording has to be searched.

use crate::{BLOCK, Placement, Reading, Shape, Signal};
use common::C32;
use decode::rs41;
use dsp::fsk::BitSync;

/// Symbols a second.
pub const BAUD: f64 = 4_800.0;

/// The channel a sonde is tuned to, which is the 10 kHz raster Vaisala
/// steps through the band on. The signal itself is 9.6 kHz, so the channel
/// is barely wider than what is in it and the two neighbours are clear.
pub const CHANNEL_WIDTH_HZ: f64 = 10_000.0;

/// What the signal itself occupies: 4800 baud keyed 2.4 kHz either way is
/// 9.6 kHz by Carson's rule, which is also what Vaisala quotes.
pub const OCCUPIED_HZ: f64 = 9_600.0;

/// The band sondes are launched into (ITU meteorological aids, region 1 and
/// beyond). Vaisala tunes an RS41 anywhere in it in 10 kHz steps.
pub const BAND: (f64, f64) = (400_000_000.0, 406_000_000.0);

/// Raster channels demodulated in one pass over a recording, however wide
/// the span is.
pub const MOST_CHANNELS: usize = 8;

pub struct Rs41;

impl Signal for Rs41 {
    fn id(&self) -> &'static str {
        "rs41"
    }

    fn label(&self) -> &'static str {
        "rs41"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["radiosonde", "sonde", "vaisala"]
    }

    /// The meteorological aids allocation. Placed by band rather than
    /// anywhere: 4800 baud FSK in a 15 kHz channel is a shape a great many
    /// things have, and outside this band none of them is a sonde.
    fn placement(&self) -> Placement {
        Placement::Bands(vec![BAND])
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            // Four samples a symbol is the demodulator's floor.
            min_rate_hz: 4.0 * BAUD,
            // Ten, which leaves the timing loop room to interpolate.
            feed_rate_hz: 48_000.0,
            span_wide: false,
            families: &[],
        }
    }

    /// The raster channels of the span holding power, and the one of those
    /// that read most frames.
    ///
    /// A sonde is nowhere near the middle of a recording in general: in the
    /// corpus capture it sits 9.76 kHz above a centre 31.25 kHz wide, and the
    /// channel filter passes 4.8 kHz either side, so reading the span as
    /// handed finds nothing at all. Vaisala tunes on the 10 kHz raster, so a
    /// span holds as many candidates as it is wide in units of ten kilohertz;
    /// a wide one holds more of them than there is time to demodulate, and
    /// [`crate::strongest`] says which carry a transmitter.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let mut best = Reading::default();
        let raster = raster(rate_hz, center_hz);
        for hz in crate::strongest(iq, rate_hz, center_hz, &raster, OCCUPIED_HZ, MOST_CHANNELS) {
            let rows = self.read_channel(iq, rate_hz, center_hz, hz);
            if rows.count() > best.count() {
                best = rows;
            }
        }
        best
    }
}

impl Rs41 {
    /// Read one raster channel, mixed down to DC first so the bit clock's
    /// own filter is over the signal rather than beside it.
    fn read_channel(&self, iq: &[C32], rate_hz: f64, center_hz: f64, channel_hz: f64) -> Reading {
        let mut sync = BitSync::with_bandwidth(rate_hz, BAUD, OCCUPIED_HZ);
        if !sync.usable() {
            return Reading::default();
        }
        let mut mixer = dsp::Mixer::new(center_hz - channel_hz, rate_hz);
        let mut framer = rs41::Framer::new();
        let mut shifted: Vec<C32> = Vec::with_capacity(BLOCK);
        let mut rows = Vec::new();
        for block in iq.chunks(BLOCK) {
            shifted.clear();
            mixer.process(block, &mut shifted);
            sync.process(&shifted, framer.sink());
            for bytes in framer.take() {
                if let Some(d) = rs41::read(&bytes) {
                    rows.push(d);
                }
            }
            framer.trim();
        }
        // The raster channel it was read on, not the recorder's centre: a
        // sonde is tuned to Vaisala's 10 kHz grid and that is where a chaser
        // points a receiver.
        Reading::from(rows).at(channel_hz)
    }
}

/// The raster channels whose signal fits inside the span.
///
/// Absolute rather than relative to the recording: the raster is a grid on
/// the band, so where the dial happens to sit does not move it. A channel
/// whose 9.6 kHz would hang over the edge of the span is left out, because
/// half a signal reads as nothing and costs a pass over the file.
fn raster(rate_hz: f64, center_hz: f64) -> Vec<f64> {
    let guard = OCCUPIED_HZ / 2.0;
    let lo = (center_hz - rate_hz / 2.0 + guard).max(BAND.0);
    let hi = (center_hz + rate_hz / 2.0 - guard).min(BAND.1);
    let mut out = Vec::new();
    let mut hz = (lo / CHANNEL_WIDTH_HZ).ceil() * CHANNEL_WIDTH_HZ;
    while hz <= hi {
        out.push(hz);
        hz += CHANNEL_WIDTH_HZ;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The corpus capture is 31.25 kHz wide at 405.80024 MHz and the sonde
    /// is 9.76 kHz above that, which is 405.81 MHz on the raster. Three
    /// channels fit, and the sonde's is one of them.
    #[test]
    fn the_raster_holds_the_sonde_and_two_neighbours() {
        let chs = raster(31_250.0, 405_800_240.0);
        assert_eq!(chs, vec![405_790_000.0, 405_800_000.0, 405_810_000.0]);
        // And nothing outside the band, however wide the recording.
        let wide = raster(20_000_000.0, 406_000_000.0);
        assert_eq!(wide.last(), Some(&406_000_000.0));
        assert_eq!(wide.len(), 601);
    }

    /// 2.4 MHz of the raster is 220 channels and eight passes, not 220.
    #[test]
    fn a_wide_span_is_read_on_the_strongest_channels_alone() {
        let rate = 2_400_000.0;
        let chs = raster(rate, 405_000_000.0);
        assert_eq!(chs.len(), 220);
        let iq = vec![C32::new(0.0, 0.0); 1 << 16];
        assert_eq!(
            crate::strongest(&iq, rate, 405_000_000.0, &chs, OCCUPIED_HZ, MOST_CHANNELS).len(),
            MOST_CHANNELS
        );
    }
}
