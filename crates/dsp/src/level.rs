//! What a transmission stood at, measured over the part of a window it was on.

use common::C32;
use common::packet::{dbfs, mean_power};

/// How long a slice the window is measured in, in microseconds.
///
/// Short enough that the shortest thing a front end reads here, a Mode S
/// reply at 120 us, still splits into slices, and long enough that a slice
/// holds many samples at every rate the router runs at.
const SLICE_US: f64 = 50.0;

/// Below this the window is one thing throughout, so all of it is the signal.
///
/// A continuous carrier cut for length has no silence in it, and splitting
/// such a window at its midpoint would throw away half a transmission for no
/// reason. Six decibels is well under the gap between a burst and its margin,
/// which is the ratio this is here to find.
const BIMODAL_DB: f32 = 6.0;

/// Mean power of the part of a window the transmitter was on, in dBFS
///
/// The router hands a burst over with a margin either side, so the mean power
/// of the whole window is the transmitter diluted by its own silence: on the
/// 2.4 GHz capture the short bursts read 11 to 27 dB below what the channel
/// was doing. Averaging every slice above the midpoint between the window's
/// quietest and loudest measures the transmission instead, and a window with
/// no silence in it is measured whole.
pub fn active_dbfs(samples: &[C32], rate: f64) -> f32 {
    let n = ((SLICE_US * rate / 1e6) as usize).max(1);
    if samples.len() < 2 * n {
        return dbfs(mean_power(samples));
    }
    let mut slices: Vec<f32> = samples.chunks(n).map(mean_power).collect();
    slices.sort_by(|a, b| a.total_cmp(b));
    // The tenth percentile rather than the minimum: one slice of a fade is
    // not the floor, and the minimum of a long window usually is one.
    let (floor, peak) = (slices[slices.len() / 10], slices[slices.len() - 1]);
    let (floor_db, peak_db) = (dbfs(floor), dbfs(peak));
    if peak_db - floor_db < BIMODAL_DB {
        return dbfs(mean_power(samples));
    }
    let on = (floor_db + peak_db) / 2.0;
    let held: Vec<f32> = slices.into_iter().filter(|p| dbfs(*p) >= on).collect();
    match held.is_empty() {
        true => peak_db,
        false => dbfs(held.iter().sum::<f32>() / held.len() as f32),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn window(rate: f64, on_us: f64, total_us: f64, on_amp: f32, floor_amp: f32) -> Vec<C32> {
        let n = (total_us * rate / 1e6) as usize;
        let on = (on_us * rate / 1e6) as usize;
        let start = (n - on) / 2;
        (0..n)
            .map(|i| match (start..start + on).contains(&i) {
                true => C32::new(on_amp, 0.0),
                false => C32::new(floor_amp, 0.0),
            })
            .collect()
    }

    #[test]
    fn a_short_burst_reads_at_its_own_level_not_the_windows() {
        // A millisecond of carrier in forty milliseconds of margin: the whole
        // window averages 16 dB low, which is what put a Wi-Fi beacon on the
        // list at the level of the silence around it.
        let rate = 4_000_000.0;
        let w = window(rate, 1_000.0, 40_000.0, 0.5, 0.005);
        assert!((active_dbfs(&w, rate) + 6.02).abs() < 0.5, "{}", active_dbfs(&w, rate));
        let whole = dbfs(mean_power(&w));
        assert!(whole < -20.0, "the window mean is {whole}, so there was nothing to fix");
    }

    #[test]
    fn a_window_that_is_carrier_throughout_is_measured_whole() {
        let rate = 1_000_000.0;
        let w = window(rate, 40_000.0, 40_000.0, 0.5, 0.5);
        assert!((active_dbfs(&w, rate) + 6.02).abs() < 0.1);
    }

    #[test]
    fn a_window_too_short_to_split_is_measured_whole() {
        let rate = 1_000_000.0;
        let w = vec![C32::new(0.5, 0.0); 8];
        assert!((active_dbfs(&w, rate) + 6.02).abs() < 0.1);
    }
}
