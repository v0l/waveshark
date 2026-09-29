use super::{Driver, Entry};
use common::{Device, DriverKind, Error, Result};

pub(super) struct Pluto;

impl Driver for Pluto {
    fn usb_ids(&self) -> &'static [(u16, u16)] {
        &[]
    }

    fn list(&self) -> Vec<Entry> {
        remote::pluto::attached()
            .into_iter()
            .enumerate()
            .map(|(i, p)| Entry {
                addr: Some(remote::pluto::USB_ADDR.to_string()),
                ..Entry::new(DriverKind::Pluto, i, p.label(), remote::pluto::RATES)
            })
            .collect()
    }

    fn open(&self, e: &Entry) -> Option<Result<Box<dyn Device>>> {
        (e.kind == DriverKind::Pluto).then(|| {
            let addr = e.addr.as_deref().ok_or(Error::NoDevice)?;
            Ok(Box::new(remote::pluto::Pluto::open(addr, DriverKind::Pluto)?) as Box<dyn Device>)
        })
    }
}
