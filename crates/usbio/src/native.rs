use crate::Error;
use crate::transfer::{
    Buffer, BulkOrInterrupt, Completion, ControlIn, ControlOut, EndpointDirection, TransferError,
};
use nusb::MaybeFuture;
use std::time::Duration;

pub fn list_devices() -> Result<std::vec::IntoIter<DeviceInfo>, Error> {
    Ok(nusb::list_devices().wait()?.map(DeviceInfo).collect::<Vec<_>>().into_iter())
}

pub struct DeviceInfo(nusb::DeviceInfo);

impl DeviceInfo {
    pub fn vendor_id(&self) -> u16 {
        self.0.vendor_id()
    }

    pub fn product_id(&self) -> u16 {
        self.0.product_id()
    }

    pub fn device_version(&self) -> u16 {
        self.0.device_version()
    }

    pub fn serial_number(&self) -> Option<&str> {
        self.0.serial_number()
    }

    pub fn product_string(&self) -> Option<&str> {
        self.0.product_string()
    }

    pub fn manufacturer_string(&self) -> Option<&str> {
        self.0.manufacturer_string()
    }

    pub fn bus_id(&self) -> &str {
        self.0.bus_id()
    }

    pub fn port_chain(&self) -> &[u8] {
        self.0.port_chain()
    }

    pub fn open(&self) -> Result<Device, Error> {
        Ok(Device(self.0.open().wait()?))
    }
}

#[derive(Clone)]
pub struct Device(nusb::Device);

impl Device {
    pub fn claim_interface(&self, n: u8) -> Result<Interface, Error> {
        Ok(Interface(self.0.claim_interface(n).wait()?))
    }

    pub fn reset(&self) -> Result<(), Error> {
        Ok(self.0.reset().wait()?)
    }

    #[cfg(target_os = "linux")]
    pub fn detach_kernel_driver(&self, n: u8) -> Result<(), Error> {
        Ok(self.0.detach_kernel_driver(n)?)
    }
}

#[derive(Clone)]
pub struct Interface(nusb::Interface);

impl Interface {
    pub fn control_in(&self, c: ControlIn, timeout: Duration) -> Result<Vec<u8>, TransferError> {
        self.0.control_in(c, timeout).wait()
    }

    pub fn control_out(&self, c: ControlOut, timeout: Duration) -> Result<(), TransferError> {
        self.0.control_out(c, timeout).wait()
    }

    pub fn endpoint<T: BulkOrInterrupt, D: EndpointDirection>(
        &self,
        address: u8,
    ) -> Result<Endpoint<T, D>, Error> {
        Ok(Endpoint(self.0.endpoint::<T, D>(address)?))
    }
}

pub struct Endpoint<T: BulkOrInterrupt, D: EndpointDirection>(nusb::Endpoint<T, D>);

impl<T: BulkOrInterrupt, D: EndpointDirection> Endpoint<T, D> {
    pub fn submit(&mut self, buffer: Buffer) {
        self.0.submit(buffer)
    }

    pub fn wait_next_complete(&mut self, timeout: Duration) -> Option<Completion> {
        self.0.wait_next_complete(timeout)
    }

    pub fn pending(&self) -> usize {
        self.0.pending()
    }

    pub fn cancel_all(&mut self) {
        self.0.cancel_all()
    }

    pub fn transfer_blocking(&mut self, buffer: Buffer, timeout: Duration) -> Completion {
        self.submit(buffer);
        if let Some(c) = self.wait_next_complete(timeout) {
            return c;
        }
        self.cancel_all();
        loop {
            if let Some(c) = self.wait_next_complete(Duration::from_secs(1)) {
                return c;
            }
        }
    }

    pub fn clear_halt(&mut self) -> Result<(), Error> {
        Ok(self.0.clear_halt().wait()?)
    }
}
