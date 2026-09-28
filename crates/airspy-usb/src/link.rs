use crate::error::{Error, Result};
use nusb::MaybeFuture;
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Recipient};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

const RX_ENDPOINT: u8 = 0x81;
const TRANSFERS: usize = 16;
const RECEIVER_MODE: u8 = 1;
const CTRL_TIMEOUT: Duration = Duration::from_millis(500);
const BULK_TIMEOUT: Duration = Duration::from_millis(1000);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Enumerated {
    pub index: usize,
    pub serial: String,
    pub product: String,
    pub port: String,
}

pub(crate) fn enumerate(vid: u16, pid: u16) -> Vec<Enumerated> {
    let Ok(devices) = nusb::list_devices().wait() else { return Vec::new() };
    devices
        .filter(|d| d.vendor_id() == vid && d.product_id() == pid)
        .enumerate()
        .map(|(index, d)| Enumerated {
            index,
            serial: serial_of(d.serial_number().unwrap_or("")),
            product: d.product_string().unwrap_or("").trim().to_string(),
            port: port_path(&d),
        })
        .collect()
}

pub fn serial_of(descriptor: &str) -> String {
    let upper = descriptor.to_ascii_uppercase();
    let tail = upper.rsplit_once("SN:").map(|(_, s)| s).unwrap_or(&upper);
    tail.trim().to_string()
}

fn port_path(d: &nusb::DeviceInfo) -> String {
    let ports: Vec<String> = d.port_chain().iter().map(|p| p.to_string()).collect();
    match ports.is_empty() {
        true => format!("{}-?", d.bus_id()),
        false => format!("{}-{}", d.bus_id(), ports.join(".")),
    }
}

pub(crate) struct Link {
    iface: nusb::Interface,
    streaming: Arc<AtomicBool>,
}

impl Link {
    pub(crate) fn open(found: &Enumerated, vid: u16, pid: u16) -> Result<Self> {
        let info = nusb::list_devices()
            .wait()
            .map_err(|e| Error::usb("list", e))?
            .find(|d| d.vendor_id() == vid && d.product_id() == pid && port_path(d) == found.port)
            .ok_or(Error::NoDevice)?;
        let device = info.open().wait().map_err(|e| match e.kind() {
            nusb::ErrorKind::PermissionDenied => Error::Permission,
            nusb::ErrorKind::Busy => Error::Busy,
            _ => Error::usb("open", e),
        })?;
        #[cfg(target_os = "linux")]
        let _ = device.detach_kernel_driver(0);
        let iface = device.claim_interface(0).wait().map_err(|e| match e.kind() {
            nusb::ErrorKind::Busy => Error::Busy,
            nusb::ErrorKind::PermissionDenied => Error::Permission,
            _ => Error::usb("claim", e),
        })?;
        let me = Self { iface, streaming: Arc::new(AtomicBool::new(false)) };
        me.set_receiver_mode(false)?;
        Ok(me)
    }

    pub(crate) fn read(&self, request: u8, value: u16, index: u16, length: u16) -> Result<Vec<u8>> {
        self.iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(|e| Error::usb(&format!("request {request}"), e))
    }

    pub(crate) fn write(&self, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
        write(&self.iface, request, value, index, data)
    }

    pub(crate) fn words(&self, request: u8) -> Result<Vec<u32>> {
        let count = words(&self.read(request, 0, 0, 4)?);
        let n = count.first().copied().unwrap_or(0).min(64) as u16;
        if n == 0 {
            return Err(Error::Usb(format!("request {request}: the firmware lists none")));
        }
        Ok(words(&self.read(request, 0, n, n * 4)?))
    }

    pub(crate) fn clear_halt(&self) {
        if !self.streaming.load(Ordering::SeqCst)
            && let Ok(mut ep) = self.iface.endpoint::<Bulk, In>(RX_ENDPOINT)
        {
            let _ = ep.clear_halt().wait();
        }
    }

    fn set_receiver_mode(&self, on: bool) -> Result<()> {
        self.write(RECEIVER_MODE, on as u16, 0, &[])
    }

    pub(crate) fn start(&self, transfer_bytes: usize) -> Result<Reader> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        let started = self.open_stream(transfer_bytes);
        if started.is_err() {
            self.streaming.store(false, Ordering::SeqCst);
        }
        started
    }

    fn open_stream(&self, transfer_bytes: usize) -> Result<Reader> {
        self.set_receiver_mode(false)?;
        let mut ep = self
            .iface
            .endpoint::<Bulk, In>(RX_ENDPOINT)
            .map_err(|e| Error::usb("open endpoint", e))?;
        let _ = ep.clear_halt().wait();
        self.set_receiver_mode(true)?;
        for _ in 0..TRANSFERS {
            ep.submit(Buffer::new(transfer_bytes));
        }
        Ok(Reader {
            ep,
            transfer_bytes,
            stopper: Stopper {
                iface: self.iface.clone(),
                stopped: Arc::new(AtomicBool::new(false)),
            },
            streaming: self.streaming.clone(),
        })
    }
}

impl Drop for Link {
    fn drop(&mut self) {
        let _ = self.set_receiver_mode(false);
    }
}

fn write(iface: &nusb::Interface, request: u8, value: u16, index: u16, data: &[u8]) -> Result<()> {
    iface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Device,
                request,
                value,
                index,
                data,
            },
            CTRL_TIMEOUT,
        )
        .wait()
        .map_err(|e| Error::usb(&format!("request {request}"), e))
}

pub(crate) fn words(bytes: &[u8]) -> Vec<u32> {
    bytes.chunks_exact(4).map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]])).collect()
}

pub(crate) fn text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}

pub struct Reader {
    ep: nusb::Endpoint<Bulk, In>,
    transfer_bytes: usize,
    stopper: Stopper,
    streaming: Arc<AtomicBool>,
}

#[derive(Clone)]
pub struct Stopper {
    iface: nusb::Interface,
    stopped: Arc<AtomicBool>,
}

impl Stopper {
    pub fn stop(&self) {
        if !self.stopped.swap(true, Ordering::SeqCst) {
            let _ = write(&self.iface, RECEIVER_MODE, 0, 0, &[]);
        }
    }

    pub fn is_stopped(&self) -> bool {
        self.stopped.load(Ordering::SeqCst)
    }
}

impl Reader {
    pub fn stopper(&self) -> Stopper {
        self.stopper.clone()
    }

    pub fn read(&mut self) -> Result<Vec<u8>> {
        loop {
            if self.stopper.is_stopped() {
                self.ep.cancel_all();
                return Err(Error::Stopped);
            }
            let Some(done) = self.ep.wait_next_complete(BULK_TIMEOUT) else { continue };
            match done.status {
                Ok(()) => {
                    self.ep.submit(Buffer::new(self.transfer_bytes));
                    return Ok(done.buffer.into_vec());
                }
                Err(_) if self.stopper.is_stopped() => continue,
                Err(e) => {
                    self.stop();
                    return Err(Error::usb("bulk transfer", e));
                }
            }
        }
    }

    pub fn stop(&mut self) {
        self.stopper.stop();
        self.ep.cancel_all();
    }
}

impl Drop for Reader {
    fn drop(&mut self) {
        self.stop();
        while self.ep.pending() > 0 {
            if self.ep.wait_next_complete(Duration::from_millis(100)).is_none() {
                break;
            }
        }
        self.streaming.store(false, Ordering::SeqCst);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_serial_is_the_hex_after_the_prefix_libairspy_strips() {
        assert_eq!(serial_of("AIRSPY SN:A74068C82F531693"), "A74068C82F531693");
        assert_eq!(serial_of("airspy sn:a74068c82f531693"), "A74068C82F531693");
        assert_eq!(serial_of("AIRSPYHF SN:3952C3DA2A3C0B35"), "3952C3DA2A3C0B35");
        assert_eq!(serial_of("0000000000000001"), "0000000000000001");
        assert_eq!(serial_of(""), "");
    }

    #[test]
    fn rates_come_back_as_little_endian_words() {
        let r2 = [0x80, 0x96, 0x98, 0x00, 0xa0, 0x25, 0x26, 0x00];
        assert_eq!(words(&r2), vec![10_000_000, 2_500_000]);
        let mini = [0x80, 0x8d, 0x5b, 0x00, 0xc0, 0xc6, 0x2d, 0x00, 0xff];
        assert_eq!(words(&mini), vec![6_000_000, 3_000_000], "a stray byte is not a rate");
    }

    #[test]
    fn a_string_ends_at_its_terminator() {
        let mut raw = b"AirSpy NOS v1.0.0-rc10-6-g4008185 2020-05-08".to_vec();
        raw.extend([0, 0x41, 0x42]);
        assert_eq!(text(&raw), "AirSpy NOS v1.0.0-rc10-6-g4008185 2020-05-08");
        assert_eq!(text(b"AirSpy MINI"), "AirSpy MINI");
    }
}
