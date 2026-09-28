use crate::error::{Error, Result};
use crate::link::{self, Enumerated, Link, Reader};

pub const VID: u16 = 0x1d50;
pub const PID: u16 = 0x60a1;
pub const TRANSFER_BYTES: usize = 262_144;
pub const LNA_MAX: u8 = 14;
pub const MIXER_MAX: u8 = 15;
pub const VGA_MAX: u8 = 15;

const BIAS_TEE_PIN: u16 = (1 << 5) | 13;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum Request {
    BoardIdRead = 9,
    VersionStringRead = 10,
    SetSampleRate = 12,
    SetFreq = 13,
    SetLnaGain = 14,
    SetMixerGain = 15,
    SetVgaGain = 16,
    SetLnaAgc = 17,
    SetMixerAgc = 18,
    GpioWrite = 21,
    GetSampleRates = 25,
    SetPacking = 26,
}

pub fn enumerate() -> Vec<Enumerated> {
    link::enumerate(VID, PID)
}

pub struct Airspy {
    link: Link,
    serial: String,
}

impl Airspy {
    pub fn open(index: usize) -> Result<Self> {
        let found = enumerate().into_iter().nth(index).ok_or(Error::NoDevice)?;
        Self::open_enumerated(&found)
    }

    pub fn open_enumerated(found: &Enumerated) -> Result<Self> {
        let me = Self { link: Link::open(found, VID, PID)?, serial: found.serial.clone() };
        me.command(Request::SetPacking, 0)?;
        Ok(me)
    }

    pub fn serial(&self) -> &str {
        &self.serial
    }

    fn command(&self, request: Request, index: u16) -> Result<()> {
        let got = self.link.read(request as u8, 0, index, 1)?;
        match got.len() {
            1 => Ok(()),
            n => Err(Error::Usb(format!("{request:?}: {n} bytes back where one was due"))),
        }
    }

    pub fn board_id(&self) -> Result<u8> {
        let got = self.link.read(Request::BoardIdRead as u8, 0, 0, 1)?;
        got.first().copied().ok_or_else(|| Error::Usb("BoardIdRead: nothing back".into()))
    }

    pub fn version(&self) -> Result<String> {
        Ok(link::text(&self.link.read(Request::VersionStringRead as u8, 0, 0, 127)?))
    }

    pub fn sample_rates(&self) -> Result<Vec<u32>> {
        self.link.words(Request::GetSampleRates as u8)
    }

    pub fn set_sample_rate_index(&self, index: u16) -> Result<()> {
        self.link.clear_halt();
        self.command(Request::SetSampleRate, index)
    }

    pub fn set_frequency(&self, hz: u32) -> Result<()> {
        self.link.write(Request::SetFreq as u8, 0, 0, &hz.to_le_bytes())
    }

    pub fn set_lna_gain(&self, step: u8) -> Result<()> {
        self.command(Request::SetLnaGain, step.min(LNA_MAX) as u16)
    }

    pub fn set_mixer_gain(&self, step: u8) -> Result<()> {
        self.command(Request::SetMixerGain, step.min(MIXER_MAX) as u16)
    }

    pub fn set_vga_gain(&self, step: u8) -> Result<()> {
        self.command(Request::SetVgaGain, step.min(VGA_MAX) as u16)
    }

    pub fn set_lna_agc(&self, on: bool) -> Result<()> {
        self.command(Request::SetLnaAgc, on as u16)
    }

    pub fn set_mixer_agc(&self, on: bool) -> Result<()> {
        self.command(Request::SetMixerAgc, on as u16)
    }

    pub fn set_bias_tee(&self, on: bool) -> Result<()> {
        self.link.write(Request::GpioWrite as u8, on as u16, BIAS_TEE_PIN, &[])
    }

    pub fn start_rx(&self) -> Result<Reader> {
        self.link.start(TRANSFER_BYTES)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_bias_tee_is_port_one_pin_thirteen() {
        assert_eq!(BIAS_TEE_PIN, 45);
    }
}
