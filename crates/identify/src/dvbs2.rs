use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::{dvbs2, mpegts};
use dsp::dvbs2::{Config, estimate};

pub struct Dvbs2;

pub const WIDTH_HZ: f64 = 18_000_000.0;
pub const MIN_RATE_HZ: f64 = 2_000_000.0;
pub const DEFAULT_HZ: f64 = 1_550_000_000.0;
pub const ROLLOFF: f64 = 0.25;
const LOOK: usize = 1 << 21;

impl Signal for Dvbs2 {
    fn id(&self) -> &'static str {
        "dvbs2"
    }

    fn label(&self) -> &'static str {
        "dvb-s2"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["dvbs2", "dvb-s2", "s2"]
    }

    fn placement(&self) -> Placement {
        Placement::Anywhere
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[WIDTH_HZ],
            min_rate_hz: MIN_RATE_HZ,
            feed_rate_hz: 0.0,
            span_wide: false,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        let Some(carrier) = estimate(&iq[..iq.len().min(LOOK)], rate_hz) else {
            return Reading::default();
        };
        let mut mixer = dsp::Mixer::new(-carrier.offset_hz, rate_hz);
        let mut rx = dvbs2::Dvbs2Receiver::new(Config {
            rate_hz,
            symbol_rate: carrier.symbol_rate,
            rolloff: ROLLOFF,
            gold: 0,
            within_hz: carrier.symbol_rate / 2.0,
        });
        let mut mux = mpegts::Mux::new();
        let (mut mixed, mut packets) = (Vec::new(), Vec::new());
        for b in iq.chunks(crate::BLOCK) {
            mixed.clear();
            mixer.process(b, &mut mixed);
            packets.clear();
            rx.push(&mixed, &mut packets);
            for p in &packets {
                mux.push(&p.bytes);
            }
        }
        let mut rows = Vec::new();
        if rx.phy().heard().is_some() {
            rows.push(dvbs2::carrier_read());
        }
        let ids: Vec<u16> = mux.services.iter().map(|s| s.id).collect();
        rows.extend(ids.into_iter().filter_map(|id| dvbs2::service_read(&mux, id)));
        Reading::from(rows).at(center_hz + carrier.offset_hz)
    }
}
