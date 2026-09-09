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
    /// The capture this entry replays, for a receiver that is a file.
    pub path: Option<std::path::PathBuf>,
    /// The one frequency this device delivers, when the tuner is somebody
    /// else's and cannot be moved from here.
    pub pinned: Option<common::Hz>,
}

impl Entry {
    fn local(
        kind: DriverKind,
        index: usize,
        label: String,
        rates: std::ops::RangeInclusive<Sps>,
    ) -> Self {
        Self { kind, index, label, rates, addr: None, path: None, pinned: None }
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
    // Last, so plugging a radio in does not change which receiver a fresh
    // session opens on. A capture is what somebody reaches for when there is
    // no aerial to hand, not the default receiver.
    for (i, c) in captures().into_iter().enumerate() {
        v.push(c.entry(i));
    }
    v
}

/// A capture on disk, offered as a receiver that plays it back.
///
/// The point is that a recording is a radio: the same graph, the same
/// detector, the same front ends and the same panes, driven at the rate the
/// samples were taken at rather than as fast as the disk allows. A decoder
/// that only ever sees a file read at ten times real time is not being tested
/// against the timing the receiver will meet, and a pane cannot be looked at
/// at all when the whole capture goes past in a tenth of a second.
#[derive(Clone, Debug, PartialEq)]
pub struct Capture {
    pub path: std::path::PathBuf,
    pub rate: Sps,
    pub center: Option<common::Hz>,
    /// How long it plays for, from the file's size and its rate.
    pub seconds: f64,
}

impl Capture {
    pub fn entry(&self, index: usize) -> Entry {
        let name = self.path.file_name().and_then(|s| s.to_str()).unwrap_or("capture");
        Entry {
            kind: DriverKind::File,
            index,
            label: format!("{name} ({:.1}s)", self.seconds),
            rates: self.rate..=self.rate,
            addr: None,
            path: Some(self.path.clone()),
            // A recording was taken at one frequency and cannot be moved off
            // it. Retuning would leave the dial saying one thing while the
            // samples said another.
            pinned: self.center,
        }
    }
}

/// Captures somebody has opened, in the order they were opened.
///
/// Configuration rather than discovery, the way a network radio is: nothing
/// on the bus reveals a file, and a folder scanned for candidates would fill
/// the list with every recording ever made. One is opened at a time, through
/// the file dialog in the receiver list, and stays there until it is dropped.
static CAPTURES: parking_lot::Mutex<Vec<Capture>> = parking_lot::Mutex::new(Vec::new());

/// Offer this capture as a receiver, and say what it will deliver.
///
/// `None` when the file cannot be replayed, which is nearly always a name
/// that does not carry a sample rate and a format. The rate scales every
/// pulse width downstream, so a guess is a receiver that decodes nothing for
/// a reason nobody can see, and `sources::parse_filename` is the one place
/// that convention lives.
pub fn add_capture(path: impl Into<std::path::PathBuf>) -> Option<Capture> {
    let path = path.into();
    let meta = sources::parse_filename(&path);
    let (rate, format) = (meta.rate?, meta.format?);
    let len = std::fs::metadata(&path).ok().filter(|m| m.is_file())?.len();
    let c = Capture {
        path,
        rate,
        center: meta.center,
        seconds: (len / format.bytes_per_sample() as u64) as f64 / rate.as_f64(),
    };
    let mut v = CAPTURES.lock();
    // Opening the same file twice is the same receiver, not a second one.
    if let Some(i) = v.iter().position(|x| x.path == c.path) {
        v[i] = c.clone();
    } else {
        v.push(c.clone());
    }
    Some(c)
}

pub fn remove_capture(path: &std::path::Path) {
    CAPTURES.lock().retain(|c| c.path != path);
}

pub fn captures() -> Vec<Capture> {
    CAPTURES.lock().clone()
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
            path: None,
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
                path: None,
                pinned: None,
            }
        }
    }
}

/// Serial tails identify a unit; the leading zeros do not.
fn short(s: &str) -> String {
    let t = s.trim_start_matches('0');
    if t.len() > 8 {
        t[t.len() - 8..].to_string()
    } else {
        t.to_string()
    }
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
        DriverKind::File => {
            let path = e.path.as_deref().ok_or(Error::NoDevice)?;
            let rate = *e.rates.end();
            // Paced to the recorded rate, or the whole capture arrives in one
            // gulp and the detector sees a band that switched on and off
            // again between two frames. Looped, because a capture is seconds
            // long and a receiver that stops after one pass is a receiver
            // that has to be restarted to look at anything twice.
            //
            // A block is about 20 ms of samples: short enough that the pane
            // moves, long enough that a 20 MS/s capture is not thousands of
            // reads a second.
            let block = ((rate.as_f64() / 50.0) as usize).clamp(4096, 1 << 20);
            Ok(Box::new(
                sources::FileSource::open(path)?.realtime(true).repeating(true).with_block(block),
            ))
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
    let mut out: Vec<Span> =
        spans_for(range).into_iter().map(|(label, rate)| Span { label, rate, zoom: 1 }).collect();
    // A device pinned to one rate is usually not on a round number, so none of
    // the candidates fall inside it. Its own rate is then the only span there
    // is, and offering nothing would leave the receiver unable to start.
    if out.is_empty() {
        let rate = range.end().as_f64();
        out.push(Span { label: label(rate), rate, zoom: 1 });
    }
    let Some(base) = out.first().map(|s| s.rate) else {
        return out;
    };
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
    CANDIDATES.iter().filter(|r| **r >= lo && **r <= hi).map(|r| (label(*r), *r)).collect()
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

    /// A capture is a receiver. What makes it usable is that the filename
    /// carries the rate and the centre, so the entry can say what it will
    /// deliver before anything opens it.
    #[test]
    fn a_capture_opened_by_hand_is_offered_as_a_receiver() {
        let dir = std::env::temp_dir().join("sr_capture_open");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        // A second of 250 kS/s, eight bit complex.
        let path = dir.join("bench_433.92M_250k.cu8");
        std::fs::write(&path, vec![0u8; 500_000]).unwrap();

        let c = add_capture(path.clone()).expect("a capture");
        assert_eq!(c.rate, Sps(250_000));
        assert_eq!(c.center, Some(common::Hz(433_920_000)));
        assert!((c.seconds - 1.0).abs() < 0.01, "{} s", c.seconds);

        let e = c.entry(0);
        assert_eq!(e.kind, DriverKind::File);
        assert_eq!(e.path.as_deref(), Some(path.as_path()));
        assert!(e.label.contains("bench_433.92M_250k.cu8"), "{}", e.label);
        // Pinned: the samples were taken at one frequency and the dial cannot
        // move off it without lying about what is being heard.
        assert_eq!(e.pinned, Some(common::Hz(433_920_000)));
        assert_eq!(e.rates, Sps(250_000)..=Sps(250_000));
        // And a span list can be built from that one rate.
        assert!(spans_with_zoom(&e.rates).iter().any(|s| (s.rate - 250_000.0).abs() < 1.0));

        // Opening the same file again is the same receiver, not a second one.
        add_capture(path.clone()).expect("a capture");
        assert_eq!(captures().iter().filter(|x| x.path == path).count(), 1);

        // A name that carries no sample rate is refused rather than guessed
        // at: a wrong rate rescales every pulse width downstream.
        let mystery = dir.join("mystery.cu8");
        std::fs::write(&mystery, vec![0u8; 1024]).unwrap();
        assert!(add_capture(mystery).is_none());

        remove_capture(&path);
        assert!(!captures().iter().any(|x| x.path == path));
        std::fs::remove_dir_all(&dir).unwrap();
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
