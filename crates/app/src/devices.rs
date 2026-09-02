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
    /// Where to reach it, for a radio that is not on this machine.
    pub addr: Option<String>,
    /// The one frequency this device delivers, when the tuner is somebody
    /// else's and cannot be moved from here.
    pub pinned: Option<common::Hz>,
}

impl Entry {
    fn local(kind: DriverKind, index: usize, label: String, rates: std::ops::RangeInclusive<Sps>) -> Self {
        Self { kind, index, label, rates, addr: None, pinned: None }
    }
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
        v.push(Entry::local(
            DriverKind::RtlSdr,
            d.index as usize,
            if tail.is_empty() { name } else { format!("{name} {tail}") },
            RTL_RATES,
        ));
    }
    for (i, serial) in hackrf::enumerate().into_iter().enumerate() {
        v.push(Entry::local(
            DriverKind::HackRf,
            i,
            format!("HackRF One {}", short(&serial)),
            HACKRF_RATES,
        ));
    }
    #[cfg(feature = "limesdr")]
    for e in limesdr::enumerate() {
        v.push(Entry::local(
            DriverKind::LimeSdr,
            e.index,
            e.label(),
            Sps(1_000_000)..=e.rate_max(),
        ));
    }
    for (i, r) in streams().into_iter().enumerate() {
        v.push(stream_entry(i, &r));
    }
    v
}

/// iqstream servers to offer alongside whatever is plugged in.
///
/// A network radio cannot be discovered by looking at the bus, so the list is
/// configuration: it comes from the session file and the command line, and the
/// settings pane edits it.
static STREAMS: parking_lot::Mutex<Vec<Remote>> = parking_lot::Mutex::new(Vec::new());

/// One configured server: where it is, and what its operator calls it.
///
/// The name is the point of having one. `radarpi:1234` says which machine and
/// nothing about which receiver, and somebody with an aerial in the loft and
/// one on the mast has to remember which host is which.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Remote {
    pub addr: String,
    /// Empty when it has never been named, in which case the address is shown.
    pub label: String,
}

pub fn streams() -> Vec<Remote> {
    STREAMS.lock().clone()
}

/// Add one server, or rename one already there. Returns the address as it will
/// be listed, which is not always what was typed: a bare host gains a port.
pub fn add_stream(addr: &str, label: &str) -> Option<String> {
    let a = iqnet::parse_addr(addr)?;
    let label = label.trim().to_string();
    let mut v = STREAMS.lock();
    match v.iter_mut().find(|r| r.addr == a) {
        // A name typed the second time is a rename, not a duplicate: the
        // address is the identity.
        Some(r) if !label.is_empty() => r.label = label,
        Some(_) => {}
        None => v.push(Remote { addr: a.clone(), label }),
    }
    Some(a)
}

pub fn remove_stream(addr: &str) {
    STREAMS.lock().retain(|r| r.addr != addr);
}

/// Ask a server what it is streaming so the entry can carry its rate and its
/// frequency, both of which are decided at the far end.
///
/// A server that does not answer is still listed. Dropping it would look like
/// the setting had been lost, when what happened is that a receiver somewhere
/// else is switched off.
fn stream_entry(index: usize, r: &Remote) -> Entry {
    let name = if r.label.is_empty() { r.addr.clone() } else { r.label.clone() };
    match iqnet::probe(&r.addr) {
        Ok(p) => Entry {
            kind: DriverKind::IqStream,
            index,
            label: format!("{name} {:.3} MHz", p.center.as_f64() / 1e6),
            rates: p.rate..=p.rate,
            addr: Some(p.addr),
            pinned: Some(p.center),
        },
        Err(e) => {
            tracing::debug!("iqstream {}: {e}", r.addr);
            Entry {
                kind: DriverKind::IqStream,
                index,
                label: format!("{name} (offline)"),
                rates: RTL_RATES,
                addr: Some(r.addr.clone()),
                pinned: None,
            }
        }
    }
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
        #[cfg(feature = "limesdr")]
        DriverKind::LimeSdr => Ok(Box::new(limesdr::LimeSdr::open(e.index)?)),
        DriverKind::IqStream => {
            let addr = e.addr.as_deref().ok_or(Error::NoDevice)?;
            Ok(Box::new(iqnet::IqNet::open(addr)?))
        }
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
    // A device pinned to one rate is usually not on a round number, so none of
    // the candidates fall inside it. Its own rate is then the only span there
    // is, and offering nothing would leave the receiver unable to start.
    if out.is_empty() {
        let rate = range.end().as_f64();
        out.push(Span { label: label(rate), rate, zoom: 1 });
    }
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
        let a = Entry::local(DriverKind::RtlSdr, 0, "a".into(), RTL_RATES);
        let b = Entry::local(DriverKind::HackRf, 0, "b".into(), HACKRF_RATES);
        assert_ne!(a, b, "device identity must include the driver");
    }

    #[test]
    fn a_network_radio_is_listed_from_configuration_rather_than_the_bus() {
        // The same server written two ways is one server, or a session that
        // saves what it loads grows a duplicate radio at every start.
        assert_eq!(add_stream("radarpi.test", "Loft").as_deref(), Some("radarpi.test:1234"));
        assert_eq!(add_stream("radarpi.test:1234", "Mast").as_deref(), Some("radarpi.test:1234"));
        let mine: Vec<Remote> =
            streams().into_iter().filter(|r| r.addr == "radarpi.test:1234").collect();
        assert_eq!(mine.len(), 1);
        // The second name renames the radio rather than adding another.
        assert_eq!(mine[0].label, "Mast");
        assert!(add_stream("  ", "").is_none());
        remove_stream("radarpi.test:1234");
        assert!(!streams().iter().any(|r| r.addr == "radarpi.test:1234"));
    }

    #[test]
    fn a_span_list_can_be_built_for_a_rate_that_is_not_one_of_the_offered_ones() {
        // A remote tuner runs at whatever the process feeding it chose, and
        // an empty span list would leave the receiver with no bandwidth at all.
        let odd = Sps(1_920_000);
        let spans = spans_with_zoom(&(odd..=odd));
        assert!(spans.iter().any(|s| (s.rate - 1_920_000.0).abs() < 1.0));
        assert!(spans.iter().all(|s| (s.rate - 1_920_000.0).abs() < 1.0));
    }
}
