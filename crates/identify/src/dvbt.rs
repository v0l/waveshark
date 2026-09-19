//! Where Dvbt can be and what stream it reads.

use crate::{Placement, Reading, Shape, Signal};
use common::C32;
use decode::{dvbt, mpegts};
use dsp::resample::Rational;

pub struct Dvbt;

impl Signal for Dvbt {
    fn id(&self) -> &'static str {
        "dvbt"
    }

    fn label(&self) -> &'static str {
        "dvbt"
    }

    fn placement(&self) -> Placement {
        Placement::Bands(bands())
    }

    fn shape(&self) -> Shape {
        Shape {
            widths: &[CHANNEL_WIDTH_HZ],
            min_rate_hz: RATE_HZ,
            feed_rate_hz: RATE_HZ,
            span_wide: false,
            // A multiplex is on the air without stopping, so there is no
            // burst for the classifier to name and nothing to wait for.
            families: &[],
        }
    }

    fn default_hz(&self) -> f64 {
        DEFAULT_HZ
    }

    /// The multiplex the recording is tuned to: its transmission
    /// parameters and every service the description table names.
    ///
    /// The receiver runs this on a thread of its own because a live 9.14
    /// MS/s is more than a block's worth of work; a file has nowhere to be,
    /// so it is read straight through here.
    fn read(&self, iq: &[C32], rate_hz: f64, center_hz: f64) -> Reading {
        if rate_hz < RATE_HZ * (1.0 - 1e-6) {
            return Reading::default();
        }
        let factor = (rate_hz / RATE_HZ).floor().max(1.0) as usize;
        let Some(mut chan) = crate::Channel::new(
            rate_hz,
            center_hz,
            center_hz,
            CHANNEL_WIDTH_HZ,
            rate_hz / factor as f64,
        ) else {
            return Reading::default();
        };
        let mut resample = Rational::approx(chan.rate_hz, RATE_HZ, 4096);
        let mut rx = dvbt::DvbtReceiver::new();
        let mut mux = mpegts::Mux::new();
        let (mut narrow, mut at_rate, mut packets) = (Vec::new(), Vec::new(), Vec::new());
        for b in iq.chunks(crate::BLOCK) {
            chan.process(b, &mut narrow);
            at_rate.clear();
            resample.process(&narrow, &mut at_rate);
            packets.clear();
            rx.push(&at_rate, &mut packets);
            for p in &packets {
                mux.push(&p.bytes);
            }
        }
        let mut rows = Vec::new();
        if let Some(params) = rx.params() {
            rows.push(dvbt::multiplex_decoded(params, rx.snr_db().unwrap_or(0.0), chan.hz(), 0.0));
        }
        let ids: Vec<u16> = mux.services.iter().map(|s| s.id).collect();
        for id in ids {
            if let Some(d) = dvbt::service_decoded(&mux, id, chan.hz(), 0.0) {
                rows.push(d);
            }
        }
        rows.into()
    }
}

/// The UHF television band as Europe allocates it now, and the remains of
/// band III. What is in them is one 8 MHz multiplex per channel.
pub fn bands() -> Vec<(f64, f64)> {
    vec![(174_000_000.0, 230_000_000.0), (470_000_000.0, 694_000_000.0)]
}

/// The channel a multiplex owns.
pub const CHANNEL_WIDTH_HZ: f64 = dsp::dvbt::CHANNEL_WIDTH_HZ;

/// The middle of the first UK multiplex channel, which is as good a place to
/// start as any: channel 21, 474 MHz.
pub const DEFAULT_HZ: f64 = 474_000_000.0;

/// The rate the receiver hands the front end, which is the standard's own.
pub const RATE_HZ: f64 = dsp::dvbt::RATE_HZ;
