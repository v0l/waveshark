//! Direct sequence spread spectrum.
//!
//! A chip sequence repeats once per symbol, so the burst is periodic in the
//! same way OFDM is. The complex autocorrelation cannot see it: the data keys
//! the sign of every symbol, and over a burst those flips cancel the
//! correlation to nothing. This is not a subtlety, it is the whole reason
//! 802.11b beacons were being read as GMSK, and it is why
//! [`super::cyclo::envelope`] exists. Squaring the envelope discards the sign
//! and the chip period reappears.
//!
//! Against OFDM, the sample statistics settle it. A spread single carrier is
//! still one carrier and stays sub-Gaussian, where a sum of subcarriers tends
//! to Gaussian: measured on 2.4 GHz captures the beacons sit at kurtosis 1.6
//! to 2.1 and 802.11's own OFDM frames at 2.9 to 3.3.

use super::hypothesis::{ramp, Evidence, Hypothesis};
use super::{Features, Modulation};

pub struct Dsss;

impl Hypothesis for Dsss {
    fn modulation(&self) -> Modulation {
        Modulation::Dsss
    }

    fn score(&self, f: &Features, e: &Evidence) -> f32 {
        // Localization is deliberately weak here where OFDM demands it. A
        // spreading code repeats at every harmonic of its symbol period, so
        // its envelope correlates at many lags at once and the median rises
        // with the peak: beacons measure a ratio of only 3.3 against an
        // absolute peak of 0.90. Demanding a sharp single lag, which is right
        // for a cyclic prefix, refuses them.
        e.filled * e.constant_envelope * e.chips * (1.0 - ramp(f.kurtosis, 2.4, 2.8))
            * (1.0 - e.sweeping)
    }
}
