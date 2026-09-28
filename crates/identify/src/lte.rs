use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use dsp::lte::{Heard, Receiver, bands};

pub struct Lte;

pub const MIN_RATE_HZ: f64 = dsp::lte::sync::RATE_HZ;

pub const WIDTHS_HZ: [f64; 6] = [10e6, 1.4e6, 3e6, 5e6, 15e6, 20e6];

pub const WIDEST_HZ: f64 = 20e6;

pub const FEED_RATE_HZ: f64 = 30.72e6;

pub const NARROWEST_HZ: f64 = 0.9e6;

pub const DEFAULT_HZ: f64 = 806e6;

impl Signal for Lte {
    fn id(&self) -> &'static str {
        "lte"
    }

    fn label(&self) -> &'static str {
        "lte"
    }

    fn aliases(&self) -> &'static [&'static str] {
        &["4g", "lte-downlink"]
    }

    fn placement(&self) -> Placement {
        Placement::Bands(bands::ranges())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &WIDTHS_HZ,
            min_rate_hz: MIN_RATE_HZ,
            feed_rate_hz: FEED_RATE_HZ,
            span_wide: true,
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz + 1.0 < MIN_RATE_HZ {
            return Reading::default();
        }
        let guess = bands::on_raster(center_hz + dsp::lte::sync::occupied_centre_hz(iq, rate_hz));
        for hz in [guess, guess - bands::RASTER_HZ, guess + bands::RASTER_HZ] {
            let rows = read_at(iq, rate_hz, center_hz, hz);
            if !rows.is_empty() {
                return Reading::from(rows).at(hz);
            }
        }
        Reading::default()
    }
}

pub fn rows(heard: &Heard) -> Vec<common::packet::Proto> {
    decode::lte::read(&bytes_of(heard)).unwrap_or_default()
}

pub fn bytes_of(heard: &Heard) -> Vec<u8> {
    match heard {
        Heard::Mib { pci, mib, .. } => decode::lte::wrap_mib(*pci, mib),
        Heard::SystemInformation { pci, bytes, .. } => {
            decode::lte::wrap_system_information(*pci, bytes)
        }
    }
}

fn read_at(
    iq: &[C32],
    rate_hz: f64,
    center_hz: f64,
    channel_hz: f64,
) -> Vec<common::packet::Proto> {
    let Some(mut rx) = Receiver::new(rate_hz, center_hz, channel_hz) else { return Vec::new() };
    let mut heard = Vec::new();
    for b in iq.chunks(crate::BLOCK) {
        let from = heard.len();
        rx.push(b, &mut heard);
        for h in &heard[from..] {
            follow(&mut rx, h);
        }
    }
    heard.iter().flat_map(rows).collect()
}

pub fn follow(rx: &mut Receiver, heard: &Heard) {
    let Heard::SystemInformation { pci, bytes, .. } = heard else { return };
    let Some(sib1) = decode::lte::parse_sib1(bytes) else { return };
    let periods: Vec<u16> = sib1.schedule.messages.iter().map(|m| m.period_frames).collect();
    rx.schedule(*pci, sib1.schedule.window_ms, &periods);
}
