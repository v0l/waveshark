//! What a frame was heard at, for the front ends that do not measure it
//! themselves.
//!
//! A frame on the packet bus carries a level, a signal to noise ratio and the
//! samples it was read from, because a row in the list that says NaN is a row
//! nobody can act on: it cannot be sorted by strength, a fade cannot be told
//! from a decoder fault, and there is nothing to look at when the bytes are
//! wrong. Demodulators that already measure the burst (Mode S off its
//! preamble, BLE off the burst and the floor either side) keep their own
//! numbers; everything else feeds its channel samples through this.
//!
//! The measurement is deliberately simple: mean power over the blocks since
//! the last frame, against a floor that follows the quietest block seen and
//! climbs a hundredth per block so a long transmission cannot become the
//! floor. On a channel cut out to the width of the signal, which is what
//! every one of these front ends is fed, that is the level of the thing that
//! was demodulated.

use common::{IqBurst, C32};
use std::sync::Arc;

pub struct FrameMeter {
    rate: f64,
    center_hz: u64,
    /// Samples kept behind the demodulator, so a frame can carry what it was
    /// read from. Bounded in seconds because the front ends run at rates
    /// three orders of magnitude apart: 48 kS/s of pager audio and 20 MS/s
    /// of a Bluetooth channel cannot share one sample count.
    keep: usize,
    ring: Vec<C32>,
    /// Absolute index of `ring[0]`, so a demodulator that reports where its
    /// frame started can have those samples back.
    base: u64,
    seen: u64,
    peak_pow: f32,
    floor_pow: f32,
}

impl FrameMeter {
    pub fn new(rate: f64, center_hz: u64, keep_s: f64) -> Self {
        Self {
            rate,
            center_hz,
            keep: (keep_s * rate).max(1.0) as usize,
            ring: Vec::new(),
            base: 0,
            seen: 0,
            peak_pow: 0.0,
            floor_pow: f32::NAN,
        }
    }

    pub fn reset(&mut self) {
        self.ring.clear();
        self.base = 0;
        self.seen = 0;
        self.peak_pow = 0.0;
        self.floor_pow = f32::NAN;
    }

    /// The block the demodulator is about to read.
    pub fn feed(&mut self, iq: &[C32]) {
        if iq.is_empty() {
            return;
        }
        let pow = iq.iter().map(|c| c.norm_sqr()).sum::<f32>() / iq.len() as f32;
        self.peak_pow = self.peak_pow.max(pow);
        self.floor_pow = if self.floor_pow.is_nan() { pow } else { pow.min(self.floor_pow * 1.01) };
        self.seen += iq.len() as u64;
        self.ring.extend_from_slice(iq);
        // Trimmed when it holds twice what is kept, not every block: moving
        // the whole ring down by a block on every block was measured to cost
        // more than the decoding it feeds.
        if self.ring.len() >= 2 * self.keep {
            let drop = self.ring.len() - self.keep;
            self.ring.drain(..drop);
            self.base += drop as u64;
        }
    }

    pub fn rssi_dbfs(&self) -> f32 {
        10.0 * self.peak_pow.max(1e-20).log10()
    }

    pub fn snr_db(&self) -> f32 {
        if !(self.floor_pow > 0.0) {
            return f32::NAN;
        }
        10.0 * (self.peak_pow / self.floor_pow).max(1.0).log10()
    }

    /// Everything read since the last frame was taken, which is the burst
    /// plus whatever silence preceded it.
    pub fn iq_since_last(&mut self) -> Option<Arc<IqBurst>> {
        if self.ring.is_empty() {
            return None;
        }
        let burst = Arc::new(IqBurst {
            rate: self.rate,
            center_hz: self.center_hz,
            samples: std::mem::take(&mut self.ring),
        });
        self.base = self.seen;
        Some(burst)
    }

    /// The samples a demodulator says its frame occupied, when it counts in
    /// the same stream this was fed.
    pub fn iq_at(&self, start_sample: u64, len: usize) -> Option<Arc<IqBurst>> {
        let from = start_sample.checked_sub(self.base)? as usize;
        if from >= self.ring.len() {
            return None;
        }
        let to = (from + len).min(self.ring.len());
        Some(Arc::new(IqBurst {
            rate: self.rate,
            center_hz: self.center_hz,
            samples: self.ring[from..to].to_vec(),
        }))
    }

    /// A frame at what the channel measured, carrying the samples it names
    /// rather than everything since the last one.
    ///
    /// For a front end that says where its frame sat and can produce several
    /// from one block. Taking the samples the other way empties the ring, so
    /// the second frame of a block came out with nothing behind it.
    pub fn frame_at(&mut self, bytes: Vec<u8>, start_sample: u64, len: usize) -> common::Frame {
        let f = common::Frame {
            bytes,
            center_hz: self.center_hz,
            rssi_dbfs: self.rssi_dbfs(),
            snr_db: self.snr_db(),
            iq: self.iq_at(start_sample, len),
        };
        self.peak_pow = 0.0;
        f
    }

    /// A frame at what the channel measured, with the samples behind it.
    ///
    /// The level is reset afterwards, so the next frame measures its own
    /// transmission rather than the loudest one of the session.
    pub fn frame(&mut self, bytes: Vec<u8>) -> common::Frame {
        let f = common::Frame {
            bytes,
            center_hz: self.center_hz,
            rssi_dbfs: self.rssi_dbfs(),
            snr_db: self.snr_db(),
            iq: self.iq_since_last(),
        };
        self.peak_pow = 0.0;
        f
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_carries_the_level_and_the_samples() {
        let mut m = FrameMeter::new(1_000_000.0, 868_000_000, 0.1);
        m.feed(&vec![C32::new(0.01, 0.0); 1000]);
        m.feed(&vec![C32::new(0.5, 0.0); 1000]);
        let f = m.frame(vec![1, 2, 3]);
        // 0.5 of full scale is a quarter of the power: -6 dBFS.
        assert!(f.rssi_dbfs > -7.0 && f.rssi_dbfs < -5.0, "rssi {}", f.rssi_dbfs);
        assert!(f.snr_db > 30.0, "snr {}", f.snr_db);
        assert_eq!(f.iq.as_ref().map(|q| q.samples.len()), Some(2000));
        assert_eq!(f.center_hz, 868_000_000);
        // The next frame measures its own transmission, not this one.
        m.feed(&vec![C32::new(0.05, 0.0); 1000]);
        let g = m.frame(vec![4]);
        assert!(g.rssi_dbfs < f.rssi_dbfs - 10.0, "{} then {}", f.rssi_dbfs, g.rssi_dbfs);
    }
}
