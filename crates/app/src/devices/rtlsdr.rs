use super::{Driver, Entry, RTL_RATES};
use common::{Device, DriverKind, Result};

pub(super) struct RtlSdr;

impl Driver for RtlSdr {
    fn usb_ids(&self) -> &'static [(u16, u16)] {
        ::rtlsdr::USB_IDS
    }

    fn list(&self) -> Vec<Entry> {
        let found = ::rtlsdr::enumerate();
        found
            .iter()
            .zip(labels(&found))
            .map(|(d, label)| Entry::new(DriverKind::RtlSdr, d.index, label, RTL_RATES))
            .collect()
    }

    fn open(&self, e: &Entry) -> Option<Result<Box<dyn Device>>> {
        (e.kind == DriverKind::RtlSdr)
            .then(|| ::rtlsdr::RtlSdr::open(e.index as u32).map(|d| Box::new(d) as Box<dyn Device>))
    }
}

/// What each dongle is called in the receiver list
///
/// Nearly every dongle ships with the serial `00000001`, so two of a kind
/// read the same and the port each is plugged into is the only thing telling
/// them apart.
fn labels(found: &[::rtlsdr::Enumerated]) -> Vec<String> {
    found
        .iter()
        .map(|d| {
            let name = if d.product.is_empty() { &d.name } else { &d.product };
            let tail = common::serial_tail(&d.serial);
            let label = if tail.is_empty() { name.clone() } else { format!("{name} {tail}") };
            let twin = found
                .iter()
                .any(|o| o.index != d.index && o.product == d.product && o.serial == d.serial);
            match twin {
                true => format!("{label} ({})", d.port),
                false => label,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two dongles of a kind carry the same serial, so the list has to say
    /// which is which or a second receiver cannot be picked at all.
    #[test]
    fn two_dongles_of_a_kind_are_told_apart_by_their_port() {
        let dongle = |index: usize, serial: &str, port: &str| ::rtlsdr::Enumerated {
            index,
            vid: 0x0bda,
            pid: 0x2838,
            name: "RTL2838UHIDIR".into(),
            manufacturer: "Realtek".into(),
            product: "RTL2838UHIDIR".into(),
            serial: serial.into(),
            port: port.into(),
        };
        let twins = [dongle(0, "00000001", "7-4"), dongle(1, "00000001", "7-8")];
        assert_eq!(labels(&twins), ["RTL2838UHIDIR 1 (7-4)", "RTL2838UHIDIR 1 (7-8)"]);

        let pair = [dongle(0, "00000001", "7-4"), dongle(1, "3579c1df", "7-8")];
        assert_eq!(labels(&pair), ["RTL2838UHIDIR 1", "RTL2838UHIDIR 3579c1df"]);
    }
}
