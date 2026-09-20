//! What a detector found, before anything places it.

use crate::packet::{Carrier, Frame, Heard, Keying, Packet};

/// A burst a detector pulled out of a stream, and what it measured about it
///
/// Everything a detector can honestly say and nothing more. It has an
/// envelope or a discriminator output, so it knows the symbols, how strong
/// they were and where in the stream they began; it does not know the
/// frequency it is parked on, the time of day, or which front end it belongs
/// to. Those are the stream's ([`Heard`]), and joining the two is the only
/// way a [`Packet`] is made.
///
/// This replaces three overlapping types that each carried part of the same
/// evidence and stamped the rest on afterwards.
#[derive(Clone, Debug, PartialEq)]
pub struct Detection {
    pub keying: Keying,
    /// Received level in dB, referenced to a full scale sample at the
    /// detector's input
    pub rssi_dbfs: f32,
    pub snr_db: f32,
    /// Where the burst began in the stream, so it can be placed in time and
    /// lined up with a waterfall
    pub at_sample: u64,
    /// How long it held the channel
    pub duration_us: u32,
}

impl Detection {
    pub fn new(keying: Keying, rssi_dbfs: f32, snr_db: f32) -> Self {
        Self { keying, rssi_dbfs, snr_db, at_sample: 0, duration_us: 0 }
    }

    pub fn at(mut self, at_sample: u64) -> Self {
        self.at_sample = at_sample;
        self
    }

    pub fn lasting(mut self, duration_us: u32) -> Self {
        self.duration_us = duration_us;
        self
    }

    /// The timings, for a detector that reads widths
    pub fn pulses(&self) -> &[crate::Pulse] {
        self.keying.pulses().unwrap_or(&[])
    }

    /// The decided symbols, for a demodulator that fits levels
    pub fn hard(&self) -> &[u8] {
        match &self.keying.symbols {
            crate::packet::Symbols::Hard(v) => v,
            _ => &[],
        }
    }

    pub fn modulation(&self) -> crate::Modulation {
        self.keying.modulation
    }

    pub fn is_empty(&self) -> bool {
        self.keying.symbols.is_empty()
    }
}

impl Heard {
    /// The reception a detection amounts to, on this stream
    pub fn packet(&self, d: &Detection) -> Packet {
        Packet::heard(self.reported(d.at_sample, d.rssi_dbfs, d.snr_db, d.duration_us))
            .keyed(d.keying.clone())
    }

    /// The reception a frame amounts to, for a demodulator that measured it
    pub fn frame(&self, at_sample: u64, frame: Frame, rssi_dbfs: f32, snr_db: f32) -> Packet {
        Packet::heard(self.reported(at_sample, rssi_dbfs, snr_db, 0)).framed(frame)
    }

    /// A reception of this whole stream, for a front end that reports no
    /// position of its own within it
    pub fn whole(&self, carrier: Carrier) -> Packet {
        Packet::heard(carrier)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::{Knowledge, Symbols};
    use crate::{Modulation, Pulse, SourceId};

    #[test]
    fn a_detection_becomes_a_reception_when_the_stream_places_it() {
        let d = Detection::new(
            Keying::configured(Modulation::Ook)
                .with(Symbols::Pulses(vec![Pulse { mark: 500, gap: 1500 }])),
            -21.0,
            18.0,
        )
        .at(48_000)
        .lasting(2_000);
        let here = Heard::new(48_000.0, 433_920_000, 250_000, SourceId(4)).at(1_000_000, 0);
        let p = here.packet(&d);
        assert_eq!(p.carrier.at_us, 2_000_000, "a second into the stream at 48 kS/s");
        assert_eq!(p.carrier.center_hz, 433_920_000);
        assert_eq!(p.carrier.bandwidth_hz, 250_000);
        assert_eq!(p.carrier.source, SourceId(4));
        assert_eq!(p.carrier.rssi_dbfs, -21.0);
        assert_eq!(p.carrier.duration_us, 2_000);
        assert_eq!(p.keying.as_ref().map(|k| k.symbols.len()), Some(1));
        assert!(matches!(p.keying.as_ref().unwrap().how, Knowledge::Configured));
    }
}
