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

use common::packet::{Frame as PacketFrame, Heard, Packet, dbfs, mean_power};
use common::{C32, IqBurst, SourceId};
use std::sync::Arc;

/// The reception a receiver locked to a multiplex is describing.
///
/// A broadcast receiver publishes what it is tuned to rather than a burst
/// somebody sent: an ensemble's name, a service's, the parameters a multiplex
/// is running. That is still a statement made on the strength of what is
/// coming in, so it carries how the block it was read from was heard, and the
/// one number the receiver itself reports is the ratio it locked at.
pub fn locked(
    center_hz: u64,
    bandwidth_hz: u32,
    iq: &[C32],
    snr_db: f32,
) -> common::packet::Carrier {
    common::packet::Carrier::heard(
        common::packet::now_us(),
        center_hz,
        bandwidth_hz,
        dbfs(mean_power(iq)),
        snr_db.max(0.0),
        SourceId(0),
    )
}

/// The reception something read off demodulated audio.
///
/// Past the demodulator there is no measurement of the air left to take, so
/// the level is the audio's own and the ratio is not claimed. A statement
/// made here is about the channel rather than about a burst: a coded squelch,
/// a unit identifier keyed in tones.
pub fn off_audio(center_hz: u64, bandwidth_hz: u32, audio: &[f32]) -> common::packet::Carrier {
    let power = match audio.is_empty() {
        true => 0.0,
        false => audio.iter().map(|s| s * s).sum::<f32>() / audio.len() as f32,
    };
    common::packet::Carrier::heard(
        common::packet::now_us(),
        center_hz,
        bandwidth_hz,
        dbfs(power),
        0.0,
        SourceId(0),
    )
}

/// A reception a front end measured for itself.
///
/// For a demodulator that reads the burst and knows what it stood at: Mode S
/// off its preamble, BLE off the floor either side, a LoRa header off its
/// own equaliser. The channel is the front end's own, which is finer than the
/// port's where a span holds several channels it reads.
pub fn measured(
    center_hz: u64,
    bandwidth_hz: u32,
    bytes: Vec<u8>,
    rssi_dbfs: f32,
    snr_db: f32,
) -> Packet {
    let carrier = common::packet::Carrier::heard(
        common::packet::now_us(),
        center_hz,
        bandwidth_hz,
        rssi_dbfs,
        snr_db,
        SourceId(0),
    );
    Packet::heard(carrier).framed(PacketFrame::of(bytes))
}

pub struct FrameMeter {
    rate: f64,
    center_hz: u64,
    /// The stream this reads, so a packet says which front end heard it. Set
    /// by whoever built the meter, since a node knows its own source and this
    /// does not
    source: SourceId,
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
            source: SourceId(0),
            keep: (keep_s * rate).max(1.0) as usize,
            ring: Vec::new(),
            base: 0,
            seen: 0,
            peak_pow: 0.0,
            floor_pow: f32::NAN,
        }
    }

    /// Say which stream this is measuring
    pub fn from(mut self, source: SourceId) -> Self {
        self.source = source;
        self
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
        let pow = mean_power(iq);
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
        dbfs(self.peak_pow)
    }

    /// Peak against floor, both clamped at the same -200 dBFS [`Self::rssi_dbfs`]
    /// reports for silence. A channel that has been digitally silent since the
    /// node was built has a floor below anything a sample can express, and the
    /// answer there is "as far above nothing as the peak is" rather than NaN:
    /// a level that is not a number cannot be sorted, compared or drawn.
    pub fn snr_db(&self) -> f32 {
        (dbfs(self.peak_pow) - dbfs(self.floor_pow)).max(0.0)
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

    /// Mean power of the samples a frame names, in dBFS.
    ///
    /// The frame's own level rather than the block's: a front end that reads
    /// several bursts out of one block wants each measured where it sat.
    pub fn power_dbfs_at(&self, start_sample: u64, len: usize) -> Option<f32> {
        let from = start_sample.checked_sub(self.base)? as usize;
        if from >= self.ring.len() || len == 0 {
            return None;
        }
        let to = (from + len).min(self.ring.len());
        let s = &self.ring[from..to];
        if s.is_empty() {
            return None;
        }
        Some(dbfs(mean_power(s)))
    }

    /// A frame's own power against the channel's floor, in dB.
    ///
    /// For a front end whose demodulator reports no ratio of its own but does
    /// say where its frame sat. [`Self::snr_db`] answers with the loudest
    /// block since the last frame, which on a front end that reads a frame
    /// every forty milliseconds is usually some other frame.
    pub fn snr_db_at(&self, start_sample: u64, len: usize) -> f32 {
        match self.power_dbfs_at(start_sample, len) {
            Some(p) => (p - dbfs(self.floor_pow)).max(0.0),
            None => self.snr_db(),
        }
    }

    /// The floor this channel has settled at, for whatever measures against it
    pub fn floor_dbfs(&self) -> f32 {
        dbfs(self.floor_pow)
    }

    /// A frame at what its own samples measured, with those samples behind
    /// it, and a signal to noise ratio the demodulator worked out.
    ///
    /// For a front end that says where its frame sat and can produce several
    /// from one block. It used to take the block's peak and then clear it,
    /// so the second frame of a block reported -200 dBFS: a level of no
    /// signal at all, on a burst that had just decoded. And the meter's own
    /// noise floor is the quietest block it has seen, which on a carrier
    /// that never stops transmitting is the carrier, so the ratio was
    /// nought. A demodulator that equalises knows better than this does.
    pub fn packet_measured(
        &mut self,
        bytes: Vec<u8>,
        start_sample: u64,
        len: usize,
        snr_db: f32,
    ) -> Packet {
        let here = Heard::new(self.rate, self.center_hz, self.rate as u32, self.source)
            .at(common::packet::now_us(), self.seen);
        let held = (len as f64 / self.rate * 1e6) as u32;
        let carrier = here.reported(
            start_sample,
            self.power_dbfs_at(start_sample, len).unwrap_or_else(|| self.rssi_dbfs()),
            snr_db,
            held,
        );
        let carrier = match self.iq_at(start_sample, len) {
            Some(q) => carrier.with_iq(q),
            None => carrier,
        };
        Packet::heard(carrier).framed(PacketFrame::of(bytes))
    }

    /// The reception of a frame at what the channel measured, stamped now.
    ///
    /// The level is reset afterwards, so the next frame measures its own
    /// transmission rather than the loudest one of the session.
    pub fn packet_now(&mut self, bytes: Vec<u8>) -> Packet {
        self.packet(bytes, common::packet::now_us())
    }

    /// The reception a frame was read at: the carrier it stood on and the
    /// bytes that came off it.
    ///
    /// The one way a front end that makes bytes puts a packet together, so
    /// nothing downstream has to repair a level, a centre or a set of samples
    /// that never arrived. The width is the channel the front end was fed,
    /// which is the width the frame was heard through.
    pub fn packet(&mut self, bytes: Vec<u8>, at_us: u64) -> Packet {
        let here = Heard::new(self.rate, self.center_hz, self.rate as u32, self.source)
            .at(at_us, self.seen);
        let iq = self.iq_since_last();
        let held = iq.as_ref().map(|q| q.samples.len()).unwrap_or(0);
        let carrier = here.reported(
            self.seen,
            self.rssi_dbfs(),
            self.snr_db(),
            (held as f64 / self.rate * 1e6) as u32,
        );
        let carrier = match iq {
            Some(q) => carrier.with_iq(q),
            None => carrier,
        };
        self.peak_pow = 0.0;
        Packet::heard(carrier).framed(PacketFrame::of(bytes))
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
        let f = m.packet_now(vec![1, 2, 3]);
        // 0.5 of full scale is a quarter of the power: -6 dBFS.
        assert!(
            f.carrier.rssi_dbfs > -7.0 && f.carrier.rssi_dbfs < -5.0,
            "rssi {}",
            f.carrier.rssi_dbfs
        );
        assert!(f.carrier.snr_db > 30.0, "snr {}", f.carrier.snr_db);
        assert_eq!(f.carrier.iq.as_ref().map(|q| q.samples.len()), Some(2000));
        assert_eq!(f.carrier.center_hz, 868_000_000);
        // The next frame measures its own transmission, not this one.
        m.feed(&vec![C32::new(0.05, 0.0); 1000]);
        let g = m.packet_now(vec![4]);
        assert!(
            g.carrier.rssi_dbfs < f.carrier.rssi_dbfs - 10.0,
            "{} then {}",
            f.carrier.rssi_dbfs,
            g.carrier.rssi_dbfs
        );
    }

    /// A packet leaves here complete: nothing downstream may have to fill in
    /// a level, a centre or the samples it was read from.
    #[test]
    fn a_packet_needs_nothing_filling_in_afterwards() {
        let mut m = FrameMeter::new(1_000_000.0, 868_000_000, 0.1).from(SourceId(3));
        m.feed(&vec![C32::new(0.01, 0.0); 1000]);
        m.feed(&vec![C32::new(0.5, 0.0); 1000]);
        let p = m.packet(vec![1, 2, 3], 1_788_177_600_000_000);
        assert!(p.carrier.rssi_dbfs.is_finite() && p.carrier.snr_db.is_finite());
        assert!(p.carrier.rssi_dbfs > -7.0 && p.carrier.rssi_dbfs < -5.0);
        assert_eq!(p.carrier.center_hz, 868_000_000);
        assert_eq!(p.carrier.bandwidth_hz, 1_000_000);
        assert_eq!(p.carrier.source, SourceId(3));
        assert_eq!(p.carrier.duration_us, 2_000, "two thousand samples at 1 MS/s");
        assert_eq!(p.carrier.iq.as_ref().map(|q| q.samples.len()), Some(2000));
        assert_eq!(p.frame.as_ref().map(|f| f.bytes.as_slice()), Some(&[1u8, 2, 3][..]));
        // And nothing was concluded about it here: that is the decoder's job.
        assert!(p.stack.is_empty());
    }
}
