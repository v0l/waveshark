//! Stage two: naming the mode, from the family and the parameters.
//!
//! Stage one says a burst is four-level frequency keying at 4800 baud in 12.5
//! kHz. That is already most of the way to naming it, because the space of
//! deployed radio systems is not dense: the combination of family, symbol
//! rate, bandwidth, burst length and band is nearly a primary key. DMR, NXDN
//! and P25 phase 2 all key four levels, and their symbol rates and channel
//! widths tell them apart.
//!
//! So this is a table, not a model. Adding a mode is a row. A row that
//! matches nothing costs nothing, a row that is wrong is visible and
//! correctable, and no retraining is involved. That is the property the
//! design asked for when it said the mode list is open-ended forever.
//!
//! Ranges are inclusive and generous: a receiver's estimate of a symbol rate
//! is worth a few percent, and a mode that only matches when the estimate is
//! perfect will never match.

use super::{Features, Modulation};

/// An inclusive range, or no constraint.
type Range = Option<(f64, f64)>;

pub struct Mode {
    pub name: &'static str,
    pub family: Modulation,
    /// Symbol rate in baud.
    pub baud: Range,
    /// Tone separation in Hz, for the keyed families.
    pub tone_sep_hz: Range,
    /// Sweep rate in Hz per second, for chirps.
    pub sweep_hz_per_s: Range,
    /// Repeating-structure period in seconds: a cyclic prefix's symbol, or a
    /// chip sequence's length.
    pub symbol_period_s: Range,
    /// Burst duration in seconds.
    pub duration_s: Range,
    /// Centre frequency in Hz, where the mode is band-specific.
    pub centre_hz: Range,
    /// What else is worth saying when this matches.
    pub note: &'static str,
}

impl Mode {
    /// The name, with the note in brackets where there is one.
    pub fn label(&self) -> String {
        if self.note.is_empty() {
            self.name.to_string()
        } else {
            format!("{} ({})", self.name, self.note)
        }
    }
}

const fn r(lo: f64, hi: f64) -> Range {
    Some((lo, hi))
}

/// Every known mode. Ordered most specific first; the first match wins.
pub static MODES: &[Mode] = &[
    Mode {
        name: "TETRA",
        // pi/4-DQPSK at 18 kbaud on a 25 kHz raster, and a base station's
        // downlink is on all day. What the family and the rate leave open,
        // the band closes: 380 to 400 MHz is the emergency services across
        // Europe, 410 to 430 the commercial networks.
        family: Modulation::Dqpsk,
        baud: r(16_500.0, 19_500.0),
        tone_sep_hz: None,
        sweep_hz_per_s: None,
        symbol_period_s: None,
        duration_s: None,
        centre_hz: r(380e6, 430e6),
        note: "base station downlink",
    },
    Mode {
        name: "LoRa SF11 BW250",
        family: Modulation::Chirp,
        baud: None,
        tone_sep_hz: None,
        // A LoRa sweep rate is BW^2 / 2^SF, which is 30.5 MHz/s for this one.
        sweep_hz_per_s: r(24e6, 38e6),
        symbol_period_s: None,
        duration_s: None,
        centre_hz: r(863e6, 870e6),
        note: "",
    },
    Mode {
        name: "LoRa BW125",
        family: Modulation::Chirp,
        baud: None,
        tone_sep_hz: None,
        sweep_hz_per_s: r(3.8e6, 125e6),
        symbol_period_s: None,
        duration_s: None,
        centre_hz: r(433e6, 928e6),
        note: "spreading factor follows from the sweep rate",
    },
    Mode {
        name: "Bluetooth LE advertising",
        family: Modulation::Msk,
        // No baud or tone constraint: at h = 0.5 the two tones merge into one
        // hump by design, so the tone test declines and reports neither. What
        // identifies it is constant-envelope keying, of that length, in that
        // band.
        baud: None,
        tone_sep_hz: None,
        sweep_hz_per_s: None,
        symbol_period_s: None,
        duration_s: r(80e-6, 500e-6),
        centre_hz: r(2400e6, 2483e6),
        note: "GFSK at h = 0.5, channels 37/38/39 at 2402, 2426 and 2480 MHz",
    },
    Mode {
        name: "802.11b beacon",
        family: Modulation::Dsss,
        baud: None,
        tone_sep_hz: None,
        sweep_hz_per_s: None,
        symbol_period_s: None,
        // Beacons go out at the 1 Mbps basic rate, which makes a long frame.
        duration_s: r(1.5e-3, 6e-3),
        centre_hz: r(2400e6, 2483e6),
        note: "DSSS at the basic rate, one per 102.4 ms beacon interval",
    },
    Mode {
        name: "802.11 OFDM frame",
        family: Modulation::Ofdm,
        baud: None,
        tone_sep_hz: None,
        sweep_hz_per_s: None,
        // 4 us symbol including the 0.8 us guard.
        symbol_period_s: r(2.4e-6, 6e-6),
        duration_s: r(10e-6, 6e-3),
        centre_hz: r(2400e6, 5900e6),
        note: "20 MHz channel, 64 subcarriers",
    },
    Mode {
        name: "LTE downlink",
        family: Modulation::Ofdm,
        baud: None,
        tone_sep_hz: None,
        sweep_hz_per_s: None,
        // 15 kHz subcarriers make a 66.7 us useful symbol.
        symbol_period_s: r(50e-6, 90e-6),
        duration_s: None,
        centre_hz: None,
        note: "15 kHz subcarrier spacing",
    },
    Mode {
        name: "DMR / NXDN / P25 phase 2",
        family: Modulation::Fsk4,
        baud: r(3600.0, 5200.0),
        tone_sep_hz: r(2000.0, 8000.0),
        sweep_hz_per_s: None,
        symbol_period_s: None,
        duration_s: None,
        centre_hz: None,
        note: "four-level keying near 4800 baud; the three differ in framing",
    },
];

fn within(v: f64, r: Range) -> bool {
    match r {
        None => true,
        Some((lo, hi)) => v >= lo && v <= hi,
    }
}

/// Name the mode, if the measured parameters place it.
///
/// `centre_hz` is where the receiver was tuned, which is evidence like any
/// other: BLE only exists in 2.4 GHz, and a chirp at 868 is not a 433 device.
pub fn identify(m: Modulation, f: &Features, centre_hz: f64) -> Option<&'static Mode> {
    MODES.iter().find(|k| {
        k.family == m
            && within(f.baud as f64, k.baud)
            && within(f.separation_hz as f64, k.tone_sep_hz)
            && within(f.chirp_rate as f64, k.sweep_hz_per_s)
            && (k.symbol_period_s.is_none()
                || (f.cyclic_period_s > 0.0
                    && within(f.cyclic_period_s as f64, k.symbol_period_s)))
            && within(f.duration_us * 1e-6, k.duration_s)
            && within(centre_hz, k.centre_hz)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every mode must be reachable: a row whose family no detector emits, or
    /// whose ranges contradict, is a row that will never fire and will not be
    /// noticed for months.
    #[test]
    fn every_mode_has_a_family_a_detector_can_emit() {
        let emitted: Vec<Modulation> = crate::classify::hypotheses()
            .iter()
            .map(|h| h.modulation())
            .collect();
        for m in MODES {
            assert!(
                emitted.contains(&m.family),
                "{} wants a family no hypothesis can emit",
                m.name
            );
            for range in [m.baud, m.tone_sep_hz, m.sweep_hz_per_s, m.symbol_period_s, m.duration_s]
            {
                if let Some((lo, hi)) = range {
                    assert!(lo < hi, "{} has an inverted range", m.name);
                }
            }
        }
    }
}
