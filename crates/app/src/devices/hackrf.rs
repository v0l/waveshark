use super::{Driver, Entry};
use common::{Device, DriverKind, Result, Sps};

pub(super) const RATES: std::ops::RangeInclusive<Sps> = Sps(2_000_000)..=Sps(20_000_000);

pub(super) struct HackRf;

impl Driver for HackRf {
    fn usb_ids(&self) -> &'static [(u16, u16)] {
        ::hackrf::USB_IDS
    }

    fn list(&self) -> Vec<Entry> {
        ::hackrf::enumerate()
            .into_iter()
            .enumerate()
            .map(|(i, serial)| {
                let label = format!("HackRF One {}", common::serial_tail(&serial));
                Entry::new(DriverKind::HackRf, i, label, RATES)
            })
            .collect()
    }

    fn open(&self, e: &Entry) -> Option<Result<Box<dyn Device>>> {
        (e.kind == DriverKind::HackRf)
            .then(|| ::hackrf::HackRfDevice::open(e.index).map(|d| Box::new(d) as Box<dyn Device>))
    }
}
