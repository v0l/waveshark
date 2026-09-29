use super::{Driver, Entry};
use common::{Device, DriverKind, Result, Sps};

pub(super) struct LimeSdr;

impl Driver for LimeSdr {
    fn usb_ids(&self) -> &'static [(u16, u16)] {
        &[]
    }

    fn list(&self) -> Vec<Entry> {
        ::limesdr::enumerate()
            .into_iter()
            .map(|e| {
                Entry::new(DriverKind::LimeSdr, e.index, e.label(), Sps(1_000_000)..=e.rate_max())
            })
            .collect()
    }

    fn open(&self, e: &Entry) -> Option<Result<Box<dyn Device>>> {
        (e.kind == DriverKind::LimeSdr)
            .then(|| ::limesdr::LimeSdr::open(e.index).map(|d| Box::new(d) as Box<dyn Device>))
    }
}
