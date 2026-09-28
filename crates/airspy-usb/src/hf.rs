use crate::error::{Error, Result};
use crate::link::{self, Enumerated, Link, Reader};

pub const VID: u16 = 0x03eb;
pub const PID: u16 = 0x800c;
pub const TRANSFER_BYTES: usize = 4096 * 4;

const CALIBRATION_MAGIC: u32 = 0xa5ca_71b0;
const CONFIG_BYTES: u16 = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
enum Request {
    SetFreq = 2,
    GetSampleRates = 3,
    SetSampleRate = 4,
    ConfigRead = 5,
    GetVersionString = 9,
    SetAgc = 10,
    SetAgcThreshold = 11,
    SetAtt = 12,
    SetLna = 13,
    GetSampleRateArchitectures = 14,
    GetFilterGain = 15,
    GetFreqDelta = 16,
    SetVctcxoCalibration = 17,
    SetFrontendOptions = 18,
    GetAttSteps = 19,
    GetBiasTeeCount = 20,
    SetBiasTee = 22,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Calibration {
    pub ppb: i32,
    pub vctcxo: u16,
    pub frontend: u32,
}

pub fn enumerate() -> Vec<Enumerated> {
    link::enumerate(VID, PID)
}

pub struct AirspyHf {
    link: Link,
    serial: String,
    product: String,
}

impl AirspyHf {
    pub fn open(index: usize) -> Result<Self> {
        let found = enumerate().into_iter().nth(index).ok_or(Error::NoDevice)?;
        Self::open_enumerated(&found)
    }

    pub fn open_enumerated(found: &Enumerated) -> Result<Self> {
        Ok(Self {
            link: Link::open(found, VID, PID)?,
            serial: found.serial.clone(),
            product: found.product.clone(),
        })
    }

    pub fn serial(&self) -> &str {
        &self.serial
    }

    pub fn product(&self) -> &str {
        &self.product
    }

    fn set(&self, request: Request, value: u16, index: u16) -> Result<()> {
        self.link.write(request as u8, value, index, &[])
    }

    pub fn version(&self) -> Result<String> {
        Ok(link::text(&self.link.read(Request::GetVersionString as u8, 0, 0, 63)?))
    }

    pub fn sample_rates(&self) -> Result<Vec<u32>> {
        self.link.words(Request::GetSampleRates as u8)
    }

    pub fn low_if(&self, rates: usize) -> Result<Vec<bool>> {
        let n = rates as u16;
        let got = self.link.read(Request::GetSampleRateArchitectures as u8, 0, n, n * 4)?;
        Ok(got.iter().take(rates).map(|b| *b != 0).collect())
    }

    pub fn att_steps(&self) -> Result<Vec<f32>> {
        Ok(self.link.words(Request::GetAttSteps as u8)?.into_iter().map(f32::from_bits).collect())
    }

    pub fn set_sample_rate_index(&self, index: u16) -> Result<()> {
        self.link.clear_halt();
        self.set(Request::SetSampleRate, 0, index)
    }

    pub fn filter_gain_db(&self) -> Result<u8> {
        let got = self.link.read(Request::GetFilterGain as u8, 0, 0, 1)?;
        got.first().copied().ok_or_else(|| Error::Usb("GetFilterGain: nothing back".into()))
    }

    pub fn set_frequency_khz(&self, khz: u32) -> Result<()> {
        self.link.write(Request::SetFreq as u8, 0, 0, &khz.to_be_bytes())
    }

    pub fn freq_delta_hz(&self) -> Result<f64> {
        let got = self.link.read(Request::GetFreqDelta as u8, 0, 0, 4)?;
        freq_delta(&got).ok_or_else(|| Error::Usb("GetFreqDelta: short reply".into()))
    }

    pub fn calibration(&self) -> Result<Option<Calibration>> {
        Ok(calibration(&self.link.read(Request::ConfigRead as u8, 0, 0, CONFIG_BYTES)?))
    }

    pub fn set_vctcxo(&self, vc: u16) -> Result<()> {
        self.set(Request::SetVctcxoCalibration, vc, 0)
    }

    pub fn set_frontend_options(&self, flags: u32) -> Result<()> {
        self.set(Request::SetFrontendOptions, flags as u16, (flags >> 16) as u16)
    }

    pub fn set_agc(&self, on: bool) -> Result<()> {
        self.set(Request::SetAgc, on as u16, 0)
    }

    pub fn set_agc_threshold_high(&self, high: bool) -> Result<()> {
        self.set(Request::SetAgcThreshold, high as u16, 0)
    }

    pub fn set_att(&self, index: u16) -> Result<()> {
        self.set(Request::SetAtt, index, 0)
    }

    pub fn set_lna(&self, on: bool) -> Result<()> {
        self.set(Request::SetLna, on as u16, 0)
    }

    pub fn bias_tees(&self) -> u32 {
        self.link
            .read(Request::GetBiasTeeCount as u8, 0, 0, 4)
            .ok()
            .and_then(|b| link::words(&b).first().copied())
            .filter(|n| *n < 8)
            .unwrap_or(0)
    }

    pub fn set_bias_tee(&self, on: bool) -> Result<()> {
        self.set(Request::SetBiasTee, on as u16, 0)
    }

    pub fn start_rx(&self) -> Result<Reader> {
        self.link.start(TRANSFER_BYTES)
    }
}

fn freq_delta(b: &[u8]) -> Option<f64> {
    let [shift, lo, mid, hi] = *b.first_chunk::<4>()?;
    let value = ((hi as i8 as i32) << 16) | ((mid as i32) << 8) | lo as i32;
    Some(value as f64 * 1e3 / (1u64 << shift.min(63)) as f64)
}

fn calibration(b: &[u8]) -> Option<Calibration> {
    let word = |at: usize| b.get(at..at + 4).map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]]));
    if word(0)? != CALIBRATION_MAGIC {
        return None;
    }
    Some(Calibration { ppb: word(4)? as i32, vctcxo: word(8)? as u16, frontend: word(12)? })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_frequency_delta_is_a_signed_24_bit_count_of_hertz_over_a_power_of_two() {
        assert_eq!(freq_delta(&[0, 0, 0, 0]), Some(0.0));
        assert_eq!(freq_delta(&[10, 0x00, 0x04, 0x00]), Some(1000.0));
        assert_eq!(freq_delta(&[12, 0x00, 0xf0, 0xff]), Some(-1000.0));
        assert_eq!(freq_delta(&[20, 0x01, 0x00, 0x00]), Some(1e3 / 1_048_576.0));
        assert_eq!(freq_delta(&[0, 0, 0]), None);
    }

    #[test]
    fn a_calibration_is_read_only_under_its_magic_number() {
        let mut page = vec![0u8; 256];
        page[..4].copy_from_slice(&CALIBRATION_MAGIC.to_le_bytes());
        page[4..8].copy_from_slice(&(-1234i32).to_le_bytes());
        page[8..12].copy_from_slice(&2150i32.to_le_bytes());
        page[12..16].copy_from_slice(&3u32.to_le_bytes());
        assert_eq!(calibration(&page), Some(Calibration { ppb: -1234, vctcxo: 2150, frontend: 3 }));
        page[0] ^= 1;
        assert_eq!(calibration(&page), None, "a blank or foreign page is no calibration");
        assert_eq!(calibration(&[]), None);
    }
}
