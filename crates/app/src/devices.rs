//! Enumerating and opening radios across drivers.
//!
//! Kept in the app rather than `common`, because `common` defines the trait
//! every driver implements and must not depend on any of them.

use common::{Device, DriverKind, Error, Result, Sps};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Entry {
    pub kind: DriverKind,
    /// Position within that driver's own enumeration.
    pub index: usize,
    pub label: String,
    /// Rates the device will accept, so the span list can be built before the
    /// device is opened. Not a constant per driver: a LimeSDR on a USB 2.0
    /// port cannot carry what the same board does on USB 3.0.
    pub rates: std::ops::RangeInclusive<Sps>,
}

/// Rates an RTL-SDR will accept. See `rtlsdr::RtlSdr::open` for why the
/// ceiling is below what the chip claims.
pub const RTL_RATES: std::ops::RangeInclusive<Sps> = Sps(225_000)..=Sps(2_400_000);
pub const HACKRF_RATES: std::ops::RangeInclusive<Sps> = Sps(2_000_000)..=Sps(20_000_000);

/// Every attached radio, RTL-SDR first because they are the common case.
pub fn list() -> Vec<Entry> {
    let mut v = Vec::new();
    for d in rtlsdr::enumerate() {
        let name = if d.product.is_empty() { d.name.clone() } else { d.product.clone() };
        let tail = short(&d.serial);
        v.push(Entry {
            kind: DriverKind::RtlSdr,
            index: d.index as usize,
            label: if tail.is_empty() { name } else { format!("{name} {tail}") },
            rates: RTL_RATES,
        });
    }
    for (i, serial) in hackrf::enumerate().into_iter().enumerate() {
        v.push(Entry {
            kind: DriverKind::HackRf,
            index: i,
            label: format!("HackRF One {}", short(&serial)),
            rates: HACKRF_RATES,
        });
    }
    for e in limesdr::enumerate() {
        v.push(Entry {
            kind: DriverKind::LimeSdr,
            index: e.index,
            label: e.label(),
            rates: Sps(1_000_000)..=e.rate_max(),
        });
    }
    v
}

/// Serial tails identify a unit; the leading zeros do not.
fn short(s: &str) -> String {
    let t = s.trim_start_matches('0');
    if t.len() > 8 { t[t.len() - 8..].to_string() } else { t.to_string() }
}

pub fn open(e: &Entry) -> Result<Box<dyn Device>> {
    match e.kind {
        DriverKind::RtlSdr => Ok(Box::new(rtlsdr::RtlSdr::open(e.index as u32)?)),
        DriverKind::HackRf => Ok(Box::new(hackrf::HackRfDevice::open(e.index)?)),
        DriverKind::LimeSdr => Ok(Box::new(limesdr::LimeSdr::open(e.index)?)),
        other => Err(Error::other(format!("{} cannot be opened live", other.as_str()))),
    }
}

/// Sample rates worth offering for a device, within what it supports.
///
/// The two radios barely overlap: an RTL-SDR tops out around 2.4 MS/s while a
/// HackRF starts at 2, so a single hard-coded list is wrong for both.
/// One entry in the bandwidth list.
///
/// A span is not always a sample rate: below what the hardware will do, it is
/// a rate plus a decimation factor. A HackRF cannot sample below 2 MS/s, and
/// on a 2 MHz span a 12.5 kHz PMR channel is half a pixel wide, so the only
/// way to see one is to narrow the span in software.
#[derive(Clone, Debug, PartialEq)]
pub struct Span {
    pub label: String,
    /// What to ask the radio for.
    pub rate: f64,
    /// How much to decimate afterwards, 1 for not at all.
    pub zoom: usize,
}

impl Span {
    /// The span actually seen, which is what the label says.
    pub fn effective(&self) -> f64 {
        self.rate / self.zoom as f64
    }
}

/// The bandwidth list for a device: its own rates, then decimated ones.
///
/// Stops at 48 kHz because that is the rate the narrowband audio chain runs
/// at, and a span narrower than the demodulator's own IF cannot be listened
/// to.
pub fn spans_with_zoom(range: &std::ops::RangeInclusive<Sps>) -> Vec<Span> {
    let mut out: Vec<Span> = spans_for(range)
        .into_iter()
        .map(|(label, rate)| Span { label, rate, zoom: 1 })
        .collect();
    let Some(base) = out.first().map(|s| s.rate) else { return out };
    let mut zoom = 2;
    while base / zoom as f64 >= 48_000.0 && zoom <= 64 {
        out.insert(0, Span { label: label(base / zoom as f64), rate: base, zoom });
        zoom *= 2;
    }
    out
}

pub fn spans_for(range: &std::ops::RangeInclusive<Sps>) -> Vec<(String, f64)> {
    const CANDIDATES: &[f64] = &[
        250_000.0,
        1_024_000.0,
        2_048_000.0,
        2_304_000.0,
        2_400_000.0,
        4_000_000.0,
        8_000_000.0,
        10_000_000.0,
        12_500_000.0,
        16_000_000.0,
        20_000_000.0,
        // A LimeSDR's clock is 30.72 MHz, so its wide rates are that and its
        // double rather than round decimal numbers.
        30_720_000.0,
        40_000_000.0,
        61_440_000.0,
    ];
    let (lo, hi) = (range.start().0 as f64, range.end().0 as f64);
    CANDIDATES
        .iter()
        .filter(|r| **r >= lo && **r <= hi)
        .map(|r| (label(*r), *r))
        .collect()
}

fn label(hz: f64) -> String {
    if hz >= 1e6 {
        let m = hz / 1e6;
        if (m - m.round()).abs() < 1e-9 {
            format!("{m:.0}M")
        } else {
            format!("{m:.3}M")
        }
    } else {
        format!("{:.0}k", hz / 1e3)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spans_respect_what_the_device_can_do() {
        // An RTL-SDR cannot do 8 MS/s and a HackRF cannot do 250 kS/s.
        let rtl = spans_for(&(Sps(225_000)..=Sps(2_400_000)));
        assert!(rtl.iter().all(|(_, r)| *r <= 2_400_000.0));
        assert!(rtl.iter().any(|(_, r)| (*r - 2_400_000.0).abs() < 1.0));

        let hrf = spans_for(&(Sps(2_000_000)..=Sps(20_000_000)));
        assert!(hrf.iter().all(|(_, r)| *r >= 2_000_000.0));
        assert!(hrf.iter().any(|(_, r)| (*r - 20_000_000.0).abs() < 1.0));
        assert!(!hrf.iter().any(|(_, r)| (*r - 250_000.0).abs() < 1.0));
    }

    #[test]
    fn a_hackrf_can_be_narrowed_to_a_pmr_channel() {
        // 12.5 kHz channels on a 2 MHz span are half a pixel wide. The
        // narrowest span offered has to make one of them readable, which
        // means tens of pixels across a 1000 pixel window.
        let hrf = spans_with_zoom(&(Sps(2_000_000)..=Sps(20_000_000)));
        let narrowest = hrf.iter().map(|s| s.effective()).fold(f64::MAX, f64::min);
        assert!(
            narrowest <= 70_000.0,
            "the narrowest span a HackRF can be given is {narrowest:.0} Hz"
        );
        let px = 12_500.0 / narrowest * 1000.0;
        assert!(px > 100.0, "a PMR channel would be {px:.0} pixels wide");
    }

    #[test]
    fn narrowed_spans_still_ask_the_radio_for_a_rate_it_has() {
        let range = Sps(2_000_000)..=Sps(20_000_000);
        for sp in spans_with_zoom(&range) {
            assert!(
                sp.rate >= range.start().0 as f64 && sp.rate <= range.end().0 as f64,
                "{} asks for {} S/s, which this radio does not do",
                sp.label,
                sp.rate
            );
            assert!(sp.zoom >= 1);
        }
    }

    #[test]
    fn nothing_narrower_than_the_narrowband_audio_chain_is_offered() {
        // A span narrower than the demodulator's IF cannot be listened to,
        // and a bandwidth in the list that silences the receiver is a trap.
        for range in [Sps(225_000)..=Sps(2_400_000), Sps(2_000_000)..=Sps(20_000_000)] {
            for sp in spans_with_zoom(&range) {
                assert!(sp.effective() >= 48_000.0, "{} is below the audio IF", sp.label);
            }
        }
    }

    #[test]
    fn every_device_offers_at_least_one_rate() {
        for r in [Sps(225_000)..=Sps(2_400_000), Sps(2_000_000)..=Sps(20_000_000)] {
            assert!(!spans_for(&r).is_empty(), "no rates offered for {r:?}");
        }
    }

    #[test]
    fn rate_labels_are_readable() {
        assert_eq!(label(2_400_000.0), "2.400M");
        assert_eq!(label(8_000_000.0), "8M");
        assert_eq!(label(250_000.0), "250k");
    }

    #[test]
    fn serials_shorten_to_the_identifying_tail() {
        assert_eq!(short("0000000000000000457863dc3579c1df"), "3579c1df");
        assert_eq!(short("00000001"), "1");
        assert_eq!(short(""), "");
    }

    #[test]
    fn the_same_index_on_two_drivers_is_not_the_same_device() {
        let a =
            Entry { kind: DriverKind::RtlSdr, index: 0, label: "a".into(), rates: RTL_RATES };
        let b =
            Entry { kind: DriverKind::HackRf, index: 0, label: "b".into(), rates: HACKRF_RATES };
        assert_ne!(a, b, "device identity must include the driver");
    }
}
