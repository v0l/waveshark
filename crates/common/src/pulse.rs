//! Pulse-train vocabulary.
//!
//! These types live in `common` rather than `dsp` because they are carried on
//! graph edges, and the graph must not depend on the DSP implementation that
//! happens to produce them. The detector lives in `dsp`; the shape of what it
//! emits belongs to everybody.

/// One mark/gap pair, in microseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pulse {
    /// Carrier present, in microseconds.
    pub mark: u32,
    /// Carrier absent, in microseconds. The final gap of a package is the
    /// timeout that ended it and carries no information.
    pub gap: u32,
}

/// A complete burst: the pulses between two long silences.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Package {
    pub pulses: Vec<Pulse>,
    /// Estimated SNR of the burst, in dB.
    pub snr_db: f32,
    /// Received level in dB, referenced to a full scale sample *at the
    /// detector's input*.
    ///
    /// Worth having next to the SNR rather than instead of it: a strong packet
    /// in a noisy channel and a weak one in a quiet channel can share an SNR,
    /// and only the level tells them apart or says the front end is clipping.
    ///
    /// Not calibrated to the antenna, and not quite calibrated to the ADC
    /// either: every filter between the two has gain, so a very strong signal
    /// reads a little above zero rather than pinning at it. Measured on the
    /// Fine Offset capture through a channelizer it comes out at +1 dB. It is
    /// a comparable number between packets on one receiver, which is what it
    /// is for, and not a field strength.
    pub rssi_dbfs: f32,
    /// Sample index where the burst started, for correlating with a waterfall.
    pub start_sample: u64,
    /// Where the burst was received, in Hz.
    ///
    /// Stamped by the detector from the stream it read, which in a channel
    /// bank is the channel's own centre rather than the tuner's. A package
    /// that has been separated from the port it arrived on is otherwise
    /// unplaceable, and a burst without a frequency is not evidence of much.
    pub center_hz: u64,
}

impl Package {
    pub fn len(&self) -> usize {
        self.pulses.len()
    }

    pub fn is_empty(&self) -> bool {
        self.pulses.is_empty()
    }

    /// Total on-air duration in microseconds, excluding the trailing timeout.
    pub fn duration_us(&self) -> u64 {
        self.pulses.iter().map(|p| p.mark as u64 + p.gap as u64).sum::<u64>()
            - self.pulses.last().map(|p| p.gap as u64).unwrap_or(0)
    }

    /// Histogram of mark widths, bucketed to `tol_us`.
    ///
    /// Reading this is how an unknown protocol gets identified by hand: PWM
    /// shows two clear mark clusters, PPM shows one mark cluster and two gap
    /// clusters, Manchester shows clusters at T and 2T in both.
    pub fn mark_histogram(&self, tol_us: u32) -> Vec<(u32, usize)> {
        histogram(self.pulses.iter().map(|p| p.mark), tol_us)
    }

    pub fn gap_histogram(&self, tol_us: u32) -> Vec<(u32, usize)> {
        // The trailing gap is a timeout, not signal, so leave it out.
        let n = self.pulses.len().saturating_sub(1);
        histogram(self.pulses[..n].iter().map(|p| p.gap), tol_us)
    }
}

fn histogram(vals: impl Iterator<Item = u32>, tol_us: u32) -> Vec<(u32, usize)> {
    let mut buckets: Vec<(u32, usize, u64)> = Vec::new();
    for v in vals {
        match buckets.iter_mut().find(|(c, _, _)| v.abs_diff(*c) <= tol_us) {
            Some((c, n, sum)) => {
                *n += 1;
                *sum += v as u64;
                *c = (*sum / *n as u64) as u32;
            }
            None => buckets.push((v, 1, v as u64)),
        }
    }
    buckets.sort_by_key(|(c, _, _)| *c);
    buckets.into_iter().map(|(c, n, _)| (c, n)).collect()
}
