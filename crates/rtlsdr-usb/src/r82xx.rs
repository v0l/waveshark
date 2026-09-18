//! Rafael Micro R820T and R828D tuner.
//!
//! A port of librtlsdr's `tuner_r82xx.c`, which is itself derived from the
//! Linux kernel's r820t driver, with the RTL-SDR Blog fork's V4 additions: the
//! 28.8 MHz upconverter below that frequency, the input switching either side
//! of it, and the notch filter bypass inside the broadcast bands. The register
//! numbers and the measured gain steps come from there.

use crate::error::{Error, Result};
use crate::transport::Transport;

pub const R820T_I2C_ADDR: u8 = 0x34;
pub const R828D_I2C_ADDR: u8 = 0x74;
pub const R828D_XTAL_FREQ: u32 = 16_000_000;
pub const CHECK_ADDR: u8 = 0x00;
pub const CHECK_VAL: u8 = 0x69;
/// IF the RTL2832U expects from an R82xx in the 6 MHz DVB-T mode
pub const IF_FREQ: u32 = 3_570_000;

const REG_SHADOW_START: u8 = 5;
const NUM_REGS: usize = 30;
/// Tuner version written to register 0x13
const VER_NUM: u8 = 49;

/// Which board the tuner is on, where that changes how it is driven
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Board {
    /// Any dongle whose tuner is reached the plain way
    Plain,
    /// RTL-SDR Blog V4, an R828D at 28.8 MHz with an HF upconverter
    BlogV4,
    /// RTL-SDR Blog V4 Lite, the same upconverter on an R820T
    BlogV4Lite,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Chip {
    R820t,
    R828d,
}

/// Which input the V4's switch is on. The numbers are librtlsdr's, and are
/// also what register 0x05 and 0x06 are set from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
enum Band {
    None,
    Hf,
    Vhf,
    Uhf,
}

#[allow(dead_code)]
struct Range {
    mhz: u32,
    open_d: u8,
    rf_mux_ploy: u8,
    tf_c: u8,
    cap20p: u8,
    cap10p: u8,
    cap0p: u8,
}

#[rustfmt::skip]
const FREQ_RANGES: &[Range] = &[
    Range { mhz: 0, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xdf, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 50, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0xbe, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 55, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x8b, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 60, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x7b, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 65, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x69, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 70, open_d: 0x08, rf_mux_ploy: 0x02, tf_c: 0x58, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 75, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 80, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x44, cap20p: 0x02, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 90, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, cap20p: 0x01, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 100, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x34, cap20p: 0x01, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 110, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, cap20p: 0x01, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 120, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x24, cap20p: 0x01, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 140, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x14, cap20p: 0x01, cap10p: 0x01, cap0p: 0x00 },
    Range { mhz: 180, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
    Range { mhz: 220, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x13, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
    Range { mhz: 250, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x11, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
    Range { mhz: 280, open_d: 0x00, rf_mux_ploy: 0x02, tf_c: 0x00, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
    Range { mhz: 310, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
    Range { mhz: 450, open_d: 0x00, rf_mux_ploy: 0x41, tf_c: 0x00, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
    Range { mhz: 588, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
    Range { mhz: 650, open_d: 0x00, rf_mux_ploy: 0x40, tf_c: 0x00, cap20p: 0x00, cap10p: 0x00, cap0p: 0x00 },
];

/// Registers 0x05 to 0x1f at reset
#[rustfmt::skip]
const INIT_ARRAY: [u8; 27] = [
    0x83, 0x30, 0x75,
    0xc0, 0x40, 0xd6, 0x6c,
    0xf5, 0x63, 0x75, 0x68,
    0x6c, 0x83, 0x80, 0x00,
    0x0f, 0x00, 0xc0, 0x30,
    0x48, 0xcc, 0x60, 0x00,
    0x54, 0xae, 0x4a, 0xc0,
];

// Measured with a Racal 6103E GSM test set at 928 MHz and -60 dBm, in tenths
// of a dB per step: http://steve-m.de/projects/rtl-sdr/gain_measurement/r820t/
const LNA_GAIN_STEPS: [i32; 16] = [0, 9, 13, 40, 38, 13, 31, 22, 26, 31, 26, 14, 19, 5, 35, 13];
const MIXER_GAIN_STEPS: [i32; 16] = [0, 5, 10, 10, 19, 9, 10, 25, 17, 10, 8, 16, 13, 6, 3, -8];

/// IF low-pass corners the filter can be set to, widest first
const IF_LOW_PASS_BW: [i32; 10] = [
    1_700_000, 1_600_000, 1_550_000, 1_450_000, 1_200_000, 900_000, 700_000, 550_000, 450_000,
    350_000,
];

const FILT_HP_BW1: i32 = 350_000;
const FILT_HP_BW2: i32 = 380_000;

/// The shadow bank is 30 registers wide and the init table covers 27
fn pad_init() -> [u8; 30] {
    let mut regs = [0u8; 30];
    regs[..INIT_ARRAY.len()].copy_from_slice(&INIT_ARRAY);
    regs
}

pub struct R82xx {
    i2c_addr: u8,
    chip: Chip,
    board: Board,
    /// Reference the PLL divides, which the ppm correction moves
    xtal: u32,
    regs: [u8; 30],
    /// IF the tuner is putting out, which the demodulator has to be told
    int_freq: u32,
    fil_cal_code: u8,
    input: Band,
    has_lock: bool,
    init_done: bool,
}

impl R82xx {
    pub fn new(chip: Chip, board: Board, xtal: u32) -> Self {
        Self {
            i2c_addr: match chip {
                Chip::R820t => R820T_I2C_ADDR,
                Chip::R828d => R828D_I2C_ADDR,
            },
            chip,
            board,
            xtal,
            regs: pad_init(),
            int_freq: IF_FREQ,
            fil_cal_code: 0,
            input: Band::None,
            has_lock: false,
            init_done: false,
        }
    }

    pub fn set_xtal(&mut self, xtal: u32) {
        self.xtal = xtal;
    }

    fn shadow_store(&mut self, reg: u8, val: &[u8]) {
        let mut start = reg as i32 - REG_SHADOW_START as i32;
        let mut val = val;
        if start < 0 {
            val = &val[(-start) as usize..];
            start = 0;
        }
        let start = start as usize;
        let len = val.len().min(NUM_REGS.saturating_sub(start));
        if len > 0 {
            self.regs[start..start + len].copy_from_slice(&val[..len]);
        }
    }

    fn write(&mut self, t: &Transport, reg: u8, val: &[u8]) -> Result<()> {
        self.shadow_store(reg, val);
        // The bridge takes at most 8 bytes in one I2C message, header included.
        let mut reg = reg;
        for chunk in val.chunks(7) {
            let mut buf = Vec::with_capacity(chunk.len() + 1);
            buf.push(reg);
            buf.extend_from_slice(chunk);
            t.i2c_write(self.i2c_addr, &buf)?;
            reg += chunk.len() as u8;
        }
        Ok(())
    }

    fn write_reg(&mut self, t: &Transport, reg: u8, val: u8) -> Result<()> {
        self.write(t, reg, &[val])
    }

    fn cached(&self, reg: u8) -> Result<u8> {
        let i = reg as i32 - REG_SHADOW_START as i32;
        if i >= 0 && (i as usize) < NUM_REGS {
            Ok(self.regs[i as usize])
        } else {
            Err(Error::Tuner(format!("register {reg:#04x} is not shadowed")))
        }
    }

    fn write_reg_mask(&mut self, t: &Transport, reg: u8, val: u8, mask: u8) -> Result<()> {
        let old = self.cached(reg)?;
        self.write_reg(t, reg, (old & !mask) | (val & mask))
    }

    /// The tuner answers with its bits reversed, from register zero onwards.
    fn read(&mut self, t: &Transport, reg: u8, len: u16) -> Result<Vec<u8>> {
        t.i2c_write(self.i2c_addr, &[reg])?;
        let d = t.i2c_read(self.i2c_addr, len)?;
        Ok(d.iter().map(|b| b.reverse_bits()).collect())
    }

    fn set_mux(&mut self, t: &Transport, freq: u32) -> Result<()> {
        let mhz = freq / 1_000_000;
        let mut i = 0;
        while i < FREQ_RANGES.len() - 1 && mhz >= FREQ_RANGES[i + 1].mhz {
            i += 1;
        }
        let (open_d, rf_mux_ploy, tf_c, cap0p) = {
            let r = &FREQ_RANGES[i];
            (r.open_d, r.rf_mux_ploy, r.tf_c, r.cap0p)
        };

        self.write_reg_mask(t, 0x17, open_d, 0x08)?;
        self.write_reg_mask(t, 0x1a, rf_mux_ploy, 0xc3)?;
        self.write_reg(t, 0x1b, tf_c)?;
        // The tuner is always opened with the high capacitor selection, so
        // this is the 0 pF column with the drive bit clear.
        self.write_reg_mask(t, 0x10, cap0p, 0x0b)?;
        self.write_reg_mask(t, 0x08, 0x00, 0x3f)?;
        self.write_reg_mask(t, 0x09, 0x00, 0x3f)
    }

    fn set_pll(&mut self, t: &Transport, freq: u32) -> Result<()> {
        let freq_khz = (freq as u64 + 500) / 1000;
        let pll_ref = self.xtal;
        let pll_ref_khz = (self.xtal as u64 + 500) / 1000;

        self.write_reg_mask(t, 0x10, 0x00, 0x10)?;
        self.write_reg_mask(t, 0x1a, 0x00, 0x0c)?;
        // Blog fork: VCO current at maximum, which is what lets the PLL lock
        // at the top of the range.
        self.write_reg_mask(t, 0x12, 0x06, 0xff)?;

        let vco_min: u64 = 1_770_000;
        let vco_max = vco_min * 2;
        let mut mix_div: u32 = 2;
        let mut div_num: i32 = 0;
        while mix_div <= 64 {
            if freq_khz * mix_div as u64 >= vco_min && freq_khz * mix_div as u64 <= vco_max {
                let mut div_buf = mix_div;
                while div_buf > 2 {
                    div_buf >>= 1;
                    div_num += 1;
                }
                break;
            }
            mix_div <<= 1;
        }

        let data = self.read(t, 0x00, 5)?;
        let vco_power_ref: u32 =
            if self.chip == Chip::R828d || self.board == Board::BlogV4Lite { 1 } else { 2 };
        let vco_fine_tune = (data[4] & 0x30) >> 4;
        if vco_fine_tune as u32 > vco_power_ref {
            div_num -= 1;
        } else if (vco_fine_tune as u32) < vco_power_ref {
            div_num += 1;
        }
        self.write_reg_mask(t, 0x10, ((div_num as u8) << 5) & 0xe0, 0xe0)?;

        let vco_freq = freq as u64 * mix_div as u64;
        let nint = (vco_freq / (2 * pll_ref as u64)) as u32;
        let mut vco_fra = (vco_freq - 2 * pll_ref as u64 * nint as u64) / 1000;

        if nint > (128 / vco_power_ref) - 1 {
            return Err(Error::Tuner(format!("no PLL divider for {freq} Hz")));
        }

        let ni = (nint - 13) / 4;
        let si = nint - 4 * ni - 13;
        self.write_reg(t, 0x14, (ni + (si << 6)) as u8)?;
        self.write_reg_mask(t, 0x12, if vco_fra == 0 { 0x08 } else { 0x00 }, 0x08)?;

        let mut sdm: u32 = 0;
        let mut n_sdm: u32 = 2;
        while vco_fra > 1 {
            if vco_fra > 2 * pll_ref_khz / n_sdm as u64 {
                sdm += 32768 / (n_sdm / 2);
                vco_fra -= 2 * pll_ref_khz / n_sdm as u64;
                if n_sdm >= 0x8000 {
                    break;
                }
            }
            n_sdm <<= 1;
        }
        self.write_reg(t, 0x16, (sdm >> 8) as u8)?;
        self.write_reg(t, 0x15, (sdm & 0xff) as u8)?;

        let mut locked = false;
        for i in 0..2 {
            let data = self.read(t, 0x00, 3)?;
            if data[2] & 0x40 != 0 {
                locked = true;
                break;
            }
            if i == 0 {
                self.write_reg_mask(t, 0x12, 0x06, 0xff)?;
            }
        }
        self.has_lock = locked;
        if !locked {
            tracing::debug!(freq, "R82xx PLL did not lock");
            return Ok(());
        }
        // Autotune back down to 8 kHz now the loop is closed.
        self.write_reg_mask(t, 0x1a, 0x08, 0x08)
    }

    fn sysfreq_sel(&mut self, t: &Transport) -> Result<()> {
        // The DVB-T values, which is what librtlsdr uses for every frequency.
        let mixer_top = 0x24;
        let lna_top = 0xe5;
        let lna_vth_l = 0x53;
        let mixer_vth_l = 0x75;
        let cp_cur = 0x38;
        let filter_cur = 0x40;
        let lna_discharge = 14;

        self.write_reg_mask(t, 0x1d, lna_top, 0xc7)?;
        self.write_reg_mask(t, 0x1c, mixer_top, 0xf8)?;
        self.write_reg(t, 0x0d, lna_vth_l)?;
        self.write_reg(t, 0x0e, mixer_vth_l)?;
        self.input = Band::None;
        self.write_reg_mask(t, 0x05, 0x00, 0x60)?;
        self.write_reg_mask(t, 0x06, 0x00, 0x08)?;
        self.write_reg_mask(t, 0x11, cp_cur, 0x38)?;
        // Blog fork: PLL drop out at 2.0 V, which is worth a few dB at L band.
        self.write_reg_mask(t, 0x17, 0xa0, 0x30)?;
        self.write_reg_mask(t, 0x0a, filter_cur, 0x60)?;

        self.write_reg_mask(t, 0x1d, 0x00, 0x38)?;
        self.write_reg_mask(t, 0x1c, 0x00, 0x04)?;
        self.write_reg_mask(t, 0x06, 0x00, 0x40)?;
        self.write_reg_mask(t, 0x1a, 0x30, 0x30)?;
        self.write_reg_mask(t, 0x1d, 0x18, 0x38)?;
        self.write_reg_mask(t, 0x1c, mixer_top, 0x04)?;
        self.write_reg_mask(t, 0x1e, lna_discharge, 0x1f)?;
        self.write_reg_mask(t, 0x1a, 0x20, 0x30)
    }

    /// Calibrate the channel filter and set the fixed DVB-T shape around it.
    fn set_tv_standard(&mut self, t: &Transport) -> Result<()> {
        let hp_cor = 0x6b;
        self.regs = pad_init();

        self.write_reg_mask(t, 0x0c, 0x00, 0x0f)?;
        self.write_reg_mask(t, 0x13, VER_NUM, 0x3f)?;
        self.write_reg_mask(t, 0x1d, 0x00, 0x38)?;
        self.int_freq = 3_570_000;

        // Two attempts, as the first calibration off a cold chip can land on
        // the end stop.
        for _ in 0..2 {
            self.write_reg_mask(t, 0x0b, hp_cor, 0x60)?;
            self.write_reg_mask(t, 0x0f, 0x04, 0x04)?;
            self.write_reg_mask(t, 0x10, 0x00, 0x03)?;
            self.set_pll(t, 56_000_000)?;
            if !self.has_lock {
                return Err(Error::Tuner("filter calibration could not lock the PLL".into()));
            }
            self.write_reg_mask(t, 0x0b, 0x10, 0x10)?;
            self.write_reg_mask(t, 0x0b, 0x00, 0x10)?;
            self.write_reg_mask(t, 0x0f, 0x00, 0x04)?;
            let data = self.read(t, 0x00, 5)?;
            self.fil_cal_code = data[4] & 0x0f;
            if self.fil_cal_code != 0 && self.fil_cal_code != 0x0f {
                break;
            }
        }
        if self.fil_cal_code == 0x0f {
            self.fil_cal_code = 0;
        }

        let filt_q = 0x10;
        self.write_reg_mask(t, 0x0a, filt_q | self.fil_cal_code, 0x1f)?;
        self.write_reg_mask(t, 0x0b, hp_cor, 0xef)?;
        self.write_reg_mask(t, 0x07, 0x00, 0x80)?;
        self.write_reg_mask(t, 0x06, 0x30, 0x30)?;
        self.write_reg_mask(t, 0x1e, 0x60, 0x60)?;
        self.write_reg_mask(t, 0x05, 0x80, 0x80)?;
        self.write_reg_mask(t, 0x1f, 0x00, 0x80)?;
        self.write_reg_mask(t, 0x0f, 0x00, 0x80)?;
        self.write_reg_mask(t, 0x19, 0x60, 0x60)
    }

    pub fn init(&mut self, t: &Transport) -> Result<()> {
        let init = INIT_ARRAY;
        self.write(t, 0x05, &init)?;
        self.set_tv_standard(t)?;
        self.sysfreq_sel(t)?;
        self.init_done = true;
        Ok(())
    }

    pub fn standby(&mut self, t: &Transport) -> Result<()> {
        if !self.init_done {
            return Ok(());
        }
        for (reg, val) in [
            (0x06u8, 0xb1u8),
            (0x05, 0xa0),
            (0x07, 0x3a),
            (0x08, 0x40),
            (0x09, 0xc0),
            (0x0a, 0x36),
            (0x0c, 0x35),
            (0x0f, 0x68),
            (0x11, 0x03),
            (0x17, 0xf4),
            (0x19, 0x0c),
        ] {
            self.write_reg(t, reg, val)?;
        }
        Ok(())
    }

    /// VGA fixed at 16.3 dB, which is what the gain table above was measured
    /// against. The Blog fork varies it by frequency and then disables that
    /// again, so this matches what a release of librtlsdr actually does.
    fn set_vga_gain(&mut self, t: &Transport) -> Result<()> {
        self.write_reg_mask(t, 0x0c, 0x08, 0x9f)
    }

    /// Manual gain in tenths of a dB, walking the LNA and mixer steps the way
    /// the gain table was built, or the tuner's own loop when `manual` is off.
    pub fn set_gain(&mut self, t: &Transport, manual: bool, tenths: i32) -> Result<()> {
        if manual {
            self.write_reg_mask(t, 0x05, 0x10, 0x10)?;
            self.write_reg_mask(t, 0x07, 0x00, 0x10)?;
            let _ = self.read(t, 0x00, 4)?;
            self.set_vga_gain(t)?;

            let (mut lna_index, mut mix_index, mut total) = (0usize, 0usize, 0i32);
            for _ in 0..15 {
                if total >= tenths {
                    break;
                }
                lna_index += 1;
                total += LNA_GAIN_STEPS[lna_index];
                if total >= tenths {
                    break;
                }
                mix_index += 1;
                total += MIXER_GAIN_STEPS[mix_index];
            }
            self.write_reg_mask(t, 0x05, lna_index as u8, 0x0f)?;
            self.write_reg_mask(t, 0x07, mix_index as u8, 0x0f)
        } else {
            self.write_reg_mask(t, 0x05, 0x00, 0x10)?;
            self.write_reg_mask(t, 0x07, 0x10, 0x10)?;
            self.write_reg_mask(t, 0x0c, 0x0b, 0x9f)
        }
    }

    /// Set the IF filter for a channel this wide, and return the IF the
    /// demodulator now has to be tuned to.
    pub fn set_bandwidth(&mut self, t: &Transport, bw: i32) -> Result<u32> {
        let (reg_0a, mut reg_0b);
        if bw > 7_000_000 {
            reg_0a = 0x10;
            reg_0b = 0x0b;
            self.int_freq = 4_570_000;
        } else if bw > 6_000_000 {
            reg_0a = 0x10;
            reg_0b = 0x2a;
            self.int_freq = 4_570_000;
        } else if bw > IF_LOW_PASS_BW[0] + FILT_HP_BW1 + FILT_HP_BW2 {
            reg_0a = 0x10;
            reg_0b = 0x6b;
            self.int_freq = 3_570_000;
        } else {
            reg_0a = 0x00;
            reg_0b = 0x80u8;
            let mut bw = bw;
            let mut real_bw = 0;
            let mut int_freq = 2_300_000i32;

            if bw > IF_LOW_PASS_BW[0] + FILT_HP_BW1 {
                bw -= FILT_HP_BW2;
                int_freq += FILT_HP_BW2;
                real_bw += FILT_HP_BW2;
            } else {
                reg_0b |= 0x20;
            }
            if bw > IF_LOW_PASS_BW[0] {
                bw -= FILT_HP_BW1;
                int_freq += FILT_HP_BW1;
                real_bw += FILT_HP_BW1;
            } else {
                reg_0b |= 0x40;
            }

            let mut i = 0;
            while i < IF_LOW_PASS_BW.len() && bw <= IF_LOW_PASS_BW[i] {
                i += 1;
            }
            let i = i.saturating_sub(1);
            reg_0b |= 15 - i as u8;
            real_bw += IF_LOW_PASS_BW[i];
            self.int_freq = (int_freq - real_bw / 2).max(0) as u32;
        }

        self.write_reg_mask(t, 0x0a, reg_0a, 0x10)?;
        self.write_reg_mask(t, 0x0b, reg_0b, 0xef)?;
        Ok(self.int_freq)
    }

    pub fn set_freq(&mut self, t: &Transport, freq: u32) -> Result<()> {
        let upconverts = matches!(self.board, Board::BlogV4 | Board::BlogV4Lite);
        // A V4 mixes HF up by its own reference rather than asking the user to
        // enter an offset, so what the tuner is set to is not what was asked.
        let upconvert_freq = if upconverts && freq < 28_800_000 { freq + 28_800_000 } else { freq };
        let lo_freq = upconvert_freq + self.int_freq;

        self.set_mux(t, lo_freq)?;
        self.set_vga_gain(t)?;
        self.set_pll(t, lo_freq)?;
        if !self.has_lock {
            return Err(Error::Tuner(format!("PLL did not lock at {freq} Hz")));
        }

        match self.board {
            Board::BlogV4 => {
                // The notches are switched out inside the bands they would
                // otherwise cut: broadcast FM and band III.
                let open_d = if freq <= 2_200_000
                    || (85_000_000..=112_000_000).contains(&freq)
                    || (172_000_000..=242_000_000).contains(&freq)
                {
                    0x00
                } else {
                    0x08
                };
                self.write_reg_mask(t, 0x17, open_d, 0x08)?;

                let band = if freq <= 28_800_000 {
                    Band::Hf
                } else if freq < 250_000_000 {
                    Band::Vhf
                } else {
                    Band::Uhf
                };
                if band == Band::Hf {
                    // The tracking filter only costs signal on the
                    // upconverted path, and set_mux has just re-applied it.
                    self.write_reg_mask(t, 0x1a, 0x40, 0xc3)?;
                    self.write_reg(t, 0x1b, 0x00)?;
                }
                if band != self.input {
                    self.input = band;
                    let cable_2_in = if band == Band::Hf { 0x08 } else { 0x00 };
                    self.write_reg_mask(t, 0x06, cable_2_in, 0x08)?;
                    t.set_gpio_output(5)?;
                    t.set_gpio_bit(5, cable_2_in == 0)?;
                    self.write_reg_mask(
                        t,
                        0x05,
                        if band == Band::Vhf { 0x40 } else { 0x00 },
                        0x40,
                    )?;
                    self.write_reg_mask(
                        t,
                        0x05,
                        if band == Band::Uhf { 0x00 } else { 0x20 },
                        0x20,
                    )?;
                }
            }
            Board::BlogV4Lite => {
                let band = if freq <= 28_800_000 { Band::Hf } else { Band::Uhf };
                if band == Band::Hf {
                    self.write_reg_mask(t, 0x1a, 0x40, 0xc3)?;
                    self.write_reg(t, 0x1b, 0x00)?;
                }
                if band != self.input {
                    self.input = band;
                    let cable_1_in = if band == Band::Hf { 0x40 } else { 0x00 };
                    t.set_gpio_output(5)?;
                    t.set_gpio_bit(5, cable_1_in == 0)?;
                    self.write_reg_mask(t, 0x05, cable_1_in, 0x40)?;
                    self.write_reg_mask(
                        t,
                        0x05,
                        if band == Band::Uhf { 0x00 } else { 0x20 },
                        0x20,
                    )?;
                }
            }
            Board::Plain => {
                // An R828D has two inputs, switched at 345 MHz where the noise
                // floor is the same either side with the same LNA setting.
                if self.chip == Chip::R828d {
                    let band = if freq > 345_000_000 { Band::Uhf } else { Band::Vhf };
                    if band != self.input {
                        self.input = band;
                        let air_cable1_in = if band == Band::Uhf { 0x00 } else { 0x60 };
                        self.write_reg_mask(t, 0x05, air_cable1_in, 0x60)?;
                    }
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The gain table in `crate::gains` is the sum of the same LNA and mixer
    /// steps the tuner is walked through, so a manual gain request has to land
    /// on a value the table offers.
    #[test]
    fn gain_steps_reach_the_published_table() {
        let mut reached = vec![0i32];
        let (mut lna, mut mix, mut total) = (0usize, 0usize, 0i32);
        while lna < 15 && mix < 15 {
            lna += 1;
            total += LNA_GAIN_STEPS[lna];
            reached.push(total);
            mix += 1;
            total += MIXER_GAIN_STEPS[mix];
            reached.push(total);
        }
        assert_eq!(reached.len(), 31);
        assert_eq!(reached[0], 0);
        assert_eq!(reached[1], 9);
        assert_eq!(reached[2], 14);
        // The published table stops at 49.6 dB, which is the last step this
        // walk reaches before the LNA index runs out.
        assert!(reached.contains(&496));
    }

    #[test]
    fn a_range_is_picked_by_its_start_frequency() {
        let pick = |mhz: u32| {
            let mut i = 0;
            while i < FREQ_RANGES.len() - 1 && mhz >= FREQ_RANGES[i + 1].mhz {
                i += 1;
            }
            FREQ_RANGES[i].mhz
        };
        assert_eq!(pick(0), 0);
        assert_eq!(pick(49), 0);
        assert_eq!(pick(50), 50);
        assert_eq!(pick(104), 100);
        assert_eq!(pick(1090), 650);
    }
}
