//! Register access over USB control transfers.
//!
//! The RTL2832U answers vendor request 0 on endpoint 0, with the address in
//! `wValue` and the block in `wIndex`; bit 4 of the block selects a write. The
//! demodulator is a page rather than a block, and the tuner hangs off an I2C
//! bus the chip bridges through block 6. Every value here is from librtlsdr.

use crate::error::{Error, Result};
use nusb::MaybeFuture;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use std::time::Duration;

const CTRL_TIMEOUT: Duration = Duration::from_millis(300);

/// Register blocks, as `wIndex >> 8`
pub mod block {
    pub const USB: u8 = 1;
    pub const SYS: u8 = 2;
    pub const IIC: u8 = 6;
}

/// USB block registers
pub mod usb_reg {
    pub const SYSCTL: u16 = 0x2000;
    pub const EPA_CTL: u16 = 0x2148;
    pub const EPA_MAXPKT: u16 = 0x2158;
}

/// System block registers
pub mod sys_reg {
    pub const DEMOD_CTL: u16 = 0x3000;
    pub const GPO: u16 = 0x3001;
    pub const GPOE: u16 = 0x3003;
    pub const GPD: u16 = 0x3004;
    pub const DEMOD_CTL_1: u16 = 0x300b;
}

/// The I2C address of the EEPROM holding the dongle's strings and flags
pub const EEPROM_ADDR: u8 = 0xa0;

/// Control endpoint of one open dongle
#[derive(Clone)]
pub struct Transport {
    iface: nusb::Interface,
}

impl Transport {
    pub fn new(iface: nusb::Interface) -> Self {
        Self { iface }
    }

    pub fn interface(&self) -> &nusb::Interface {
        &self.iface
    }

    pub fn read_array(&self, block: u8, addr: u16, len: u16) -> Result<Vec<u8>> {
        self.iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: addr,
                    index: (block as u16) << 8,
                    length: len,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(|e| Error::usb("read", e))
    }

    pub fn write_array(&self, block: u8, addr: u16, data: &[u8]) -> Result<()> {
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: addr,
                    index: ((block as u16) << 8) | 0x10,
                    data,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(|e| Error::usb("write", e))?;
        Ok(())
    }

    pub fn read_reg(&self, block: u8, addr: u16, len: u16) -> Result<u16> {
        let d = self.read_array(block, addr, len)?;
        Ok(match d.len() {
            0 => 0,
            1 => d[0] as u16,
            _ => ((d[1] as u16) << 8) | d[0] as u16,
        })
    }

    pub fn write_reg(&self, block: u8, addr: u16, val: u16, len: u8) -> Result<()> {
        let data: [u8; 2] =
            if len == 1 { [(val & 0xff) as u8, 0] } else { [(val >> 8) as u8, (val & 0xff) as u8] };
        self.write_array(block, addr, &data[..len as usize])
    }

    pub fn demod_read_reg(&self, page: u8, addr: u16, len: u16) -> Result<u16> {
        let d = self
            .iface
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: (addr << 8) | 0x20,
                    index: page as u16,
                    length: len,
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(|e| Error::usb("demod read", e))?;
        Ok(match d.len() {
            0 => 0,
            1 => d[0] as u16,
            _ => ((d[1] as u16) << 8) | d[0] as u16,
        })
    }

    pub fn demod_write_reg(&self, page: u8, addr: u16, val: u16, len: u8) -> Result<()> {
        let data: [u8; 2] =
            if len == 1 { [(val & 0xff) as u8, 0] } else { [(val >> 8) as u8, (val & 0xff) as u8] };
        self.iface
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request: 0,
                    value: (addr << 8) | 0x20,
                    index: 0x10 | page as u16,
                    data: &data[..len as usize],
                },
                CTRL_TIMEOUT,
            )
            .wait()
            .map_err(|e| Error::usb("demod write", e))?;
        // The demodulator needs a read between writes or it keeps the old
        // value; librtlsdr reads this register for the same reason.
        let _ = self.demod_read_reg(0x0a, 0x01, 1);
        Ok(())
    }

    pub fn i2c_write(&self, i2c_addr: u8, data: &[u8]) -> Result<()> {
        self.write_array(block::IIC, i2c_addr as u16, data)
    }

    pub fn i2c_read(&self, i2c_addr: u8, len: u16) -> Result<Vec<u8>> {
        self.read_array(block::IIC, i2c_addr as u16, len)
    }

    /// One register off an I2C device, or zero where it did not answer
    pub fn i2c_read_reg(&self, i2c_addr: u8, reg: u8) -> u8 {
        if self.i2c_write(i2c_addr, &[reg]).is_err() {
            return 0;
        }
        self.i2c_read(i2c_addr, 1).ok().and_then(|d| d.first().copied()).unwrap_or(0)
    }

    /// Opens the bridge between the host's control transfers and the tuner's
    /// I2C bus. Every tuner register access is between an on and an off.
    pub fn set_i2c_repeater(&self, on: bool) -> Result<()> {
        self.demod_write_reg(1, 0x01, if on { 0x18 } else { 0x10 }, 1)
    }

    pub fn set_gpio_output(&self, gpio: u8) -> Result<()> {
        let bit = 1u16 << gpio;
        let r = self.read_reg(block::SYS, sys_reg::GPD, 1)?;
        self.write_reg(block::SYS, sys_reg::GPD, r & !bit, 1)?;
        let r = self.read_reg(block::SYS, sys_reg::GPOE, 1)?;
        self.write_reg(block::SYS, sys_reg::GPOE, r | bit, 1)
    }

    pub fn set_gpio_bit(&self, gpio: u8, val: bool) -> Result<()> {
        let bit = 1u16 << gpio;
        let r = self.read_reg(block::SYS, sys_reg::GPO, 1)?;
        let r = if val { r | bit } else { r & !bit };
        self.write_reg(block::SYS, sys_reg::GPO, r, 1)
    }

    /// The dongle's EEPROM, which holds its strings and the always-on bias tee
    /// flag. Read a byte at a time, as the bridge has no auto-increment.
    pub fn read_eeprom(&self, offset: u8, len: usize) -> Result<Vec<u8>> {
        self.write_array(block::IIC, EEPROM_ADDR as u16, &[offset])?;
        let mut out = Vec::with_capacity(len);
        for _ in 0..len {
            let d = self.i2c_read(EEPROM_ADDR, 1)?;
            out.push(d.first().copied().unwrap_or(0));
        }
        Ok(out)
    }
}
