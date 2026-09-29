use super::{Driver, Entry};
use common::{Device, DriverKind, Result, Sps};

pub(super) struct Airspy;

impl Driver for Airspy {
    fn usb_ids(&self) -> &'static [(u16, u16)] {
        ::airspy::USB_IDS
    }

    fn list(&self) -> Vec<Entry> {
        let r2 = ::airspy::enumerate().into_iter().map(|a| entry(&a));
        r2.chain(::airspy::hf::enumerate().into_iter().map(|a| hf_entry(&a))).collect()
    }

    fn open(&self, e: &Entry) -> Option<Result<Box<dyn Device>>> {
        match e.kind {
            DriverKind::Airspy => {
                Some(::airspy::Airspy::open(e.index).map(|d| Box::new(d) as Box<dyn Device>))
            }
            DriverKind::AirspyHf => {
                Some(::airspy::hf::AirspyHf::open(e.index).map(|d| Box::new(d) as Box<dyn Device>))
            }
            _ => None,
        }
    }
}

fn entry(a: &::airspy::Found) -> Entry {
    let mut e = Entry::new(DriverKind::Airspy, a.index, a.label(), ::airspy::rate_range(&a.rates));
    e.steps = a.rates.iter().map(|r| Sps(*r as u64)).collect();
    e.steps.sort();
    e
}

fn hf_entry(a: &::airspy::hf::Found) -> Entry {
    let mut e =
        Entry::new(DriverKind::AirspyHf, a.index, a.label(), ::airspy::hf::rate_range(&a.rates));
    e.steps = a.rates.iter().map(|r| Sps(*r as u64)).collect();
    e.steps.sort();
    e
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::spans_of;

    #[test]
    fn an_airspy_offers_its_firmware_rates_and_the_slowest_narrowed_to_78_khz() {
        let spans = |rates: Vec<u32>, model| {
            let found =
                ::airspy::Found { index: 0, serial: "A74068C82F531693".into(), model, rates };
            let e = entry(&found);
            assert_eq!(e.kind, DriverKind::Airspy);
            spans_of(&e).iter().map(|s| (s.label.clone(), s.rate, s.zoom)).collect::<Vec<_>>()
        };
        let r2 = spans(vec![10_000_000, 2_500_000], ::airspy::Model::R2);
        let want = |base: f64, top: f64, top_label: &str, base_label: &str, zooms: &[&str]| {
            let mut v: Vec<(String, f64, usize)> = zooms
                .iter()
                .rev()
                .enumerate()
                .map(|(i, l)| (l.to_string(), base, 1 << (zooms.len() - i)))
                .collect();
            v.push((base_label.into(), base, 1));
            v.push((top_label.into(), top, 1));
            v
        };
        assert_eq!(
            r2,
            want(2.5e6, 10e6, "10M", "2.500M", &["1.250M", "625k", "312k", "156k", "78k"])
        );
        let mini = spans(vec![6_000_000, 3_000_000], ::airspy::Model::Mini);
        assert_eq!(mini, want(3e6, 6e6, "6M", "3M", &["1.500M", "750k", "375k", "188k", "94k"]));
    }

    #[test]
    fn an_airspy_hf_offers_every_firmware_rate_and_narrows_the_slowest_to_48_khz() {
        let found = ::airspy::hf::Found {
            index: 0,
            serial: "3952C3DA2A3C0B35".into(),
            product: "AIRSPY HF+ Discovery".into(),
            rates: vec![912_000, 768_000, 456_000, 384_000, 256_000, 192_000],
        };
        let e = hf_entry(&found);
        assert_eq!(
            (e.kind, e.label.as_str()),
            (DriverKind::AirspyHf, "Airspy HF+ Discovery 2A3C0B35")
        );
        let spans: Vec<(String, usize)> =
            spans_of(&e).iter().map(|s| (s.label.clone(), s.zoom)).collect();
        let want: Vec<(String, usize)> = [
            ("48k", 4),
            ("96k", 2),
            ("192k", 1),
            ("256k", 1),
            ("384k", 1),
            ("456k", 1),
            ("768k", 1),
            ("912k", 1),
        ]
        .iter()
        .map(|(l, z)| (l.to_string(), *z))
        .collect();
        assert_eq!(spans, want);
    }
}
