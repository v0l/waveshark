//! The RTL2832U demodulator and the dongles built on it.
//!
//! A port of librtlsdr's open, init and control path. Everything the tuner
//! needs goes through the I2C repeater; everything else is register writes
//! from the tables below. Sample streaming is a bulk endpoint queue driven on
//! its own thread, and the control side stays open while it runs.

use crate::error::{Error, Result};
use crate::gains;
use crate::r82xx::{self, Board, Chip, R82xx};
use crate::transport::{self, Transport};
use nusb::MaybeFuture;
use nusb::transfer::{Bulk, In};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

pub const RX_ENDPOINT: u8 = 0x81;
/// Librtlsdr's transfer size, about 3.4 ms a buffer at 2.4 MS/s
const XFER_BYTES: usize = 16 * 1024;
const TRANSFERS: usize = 32;
const BULK_TIMEOUT: Duration = Duration::from_millis(1000);

/// 28.8 MHz, the crystal of every RTL2832U
pub const DEF_RTL_XTAL: u32 = 28_800_000;

const FIR_LEN: usize = 16;
/// Default coefficients the Windows driver uses for DAB and FM
#[rustfmt::skip]
const FIR_DEFAULT: [i32; FIR_LEN] = [
    -54, -36, -41, -40, -32, -14, 14, 53,
    101, 156, 215, 273, 327, 372, 404, 421,
];

/// Direct sampling of one ADC input, below the tuner's floor
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DirectSampling {
    #[default]
    Off,
    /// The I branch
    I,
    /// The Q branch
    Q,
}

/// The tuner found behind the demodulator, numbered as librtlsdr does
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tuner {
    Unknown,
    E4000,
    Fc0012,
    Fc0013,
    Fc2580,
    R820t,
    R828d,
}

impl Tuner {
    pub fn code(self) -> u8 {
        match self {
            Self::Unknown => 0,
            Self::E4000 => gains::code::E4K,
            Self::Fc0012 => gains::code::FC0012,
            Self::Fc0013 => gains::code::FC0013,
            Self::Fc2580 => gains::code::FC2580,
            Self::R820t => gains::code::R820T,
            Self::R828d => gains::code::R828D,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::E4000 => "E4000",
            Self::Fc0012 => "FC0012",
            Self::Fc0013 => "FC0013",
            Self::Fc2580 => "FC2580",
            Self::R820t => "R820T",
            Self::R828d => "R828D",
        }
    }

    pub fn gains(self) -> &'static [i32] {
        gains::for_code(self.code())
    }
}

/// One dongle visible on the bus
#[derive(Clone, Debug)]
pub struct Enumerated {
    pub index: usize,
    pub vid: u16,
    pub pid: u16,
    pub name: String,
    pub manufacturer: String,
    pub product: String,
    pub serial: String,
    /// Where the dongle is plugged in, as sysfs names it (`7-4.1`)
    ///
    /// Every dongle of a kind carries the same serial, `00000001`, so the
    /// port is the only thing telling two of them apart.
    pub port: String,
}

const VID_REALTEK: u16 = 0x0bda;

/// Identifies a dongle without opening it. Most RTL2832U dongles carry a
/// Realtek VID, and the Blog V4 carries its own.
fn is_rtl2832(vid: u16, pid: u16) -> bool {
    vid == VID_REALTEK && matches!(pid, 0x2832 | 0x2838)
        // RTL-SDR Blog V4
        || (vid == 0x1d50 && pid == 0x8667)
}

pub fn enumerate() -> Vec<Enumerated> {
    let Ok(devices) = nusb::list_devices().wait() else { return Vec::new() };
    devices
        .filter(|d| is_rtl2832(d.vendor_id(), d.product_id()))
        .enumerate()
        .map(|(index, d)| Enumerated {
            index,
            vid: d.vendor_id(),
            pid: d.product_id(),
            name: d.product_string().unwrap_or("RTL2832U").to_string(),
            manufacturer: d.manufacturer_string().unwrap_or("").to_string(),
            product: d.product_string().unwrap_or("").to_string(),
            serial: d.serial_number().unwrap_or("").to_string(),
            port: port_path(&d),
        })
        .collect()
}

fn port_path(d: &nusb::DeviceInfo) -> String {
    let ports: Vec<String> = d.port_chain().iter().map(|p| p.to_string()).collect();
    match ports.is_empty() {
        true => format!("{}-?", d.bus_id()),
        false => format!("{}-{}", d.bus_id(), ports.join(".")),
    }
}

struct State {
    tuner: Tuner,
    r82xx: Option<R82xx>,
    /// Tuner reference the ppm correction applies to
    tun_xtal: u32,
    rate: u32,
    freq: u32,
    corr: i32,
    direct_sampling: DirectSampling,
    /// What the operator asked for, which governs auto-switching below 24 MHz
    direct_sampling_mode: DirectSampling,
    bias_tee: bool,
    /// The EEPROM flag that makes a dongle's bias tee always on
    force_bt: bool,
    /// Set once the tuner is opened, which the demodulator then expects
    baseband: bool,
}

pub struct RtlSdr {
    pub(crate) t: Transport,
    state: Mutex<State>,
    board: Board,
    manufacturer: String,
    product: String,
    serial: String,
    streaming: AtomicBool,
}

impl RtlSdr {
    pub fn list() -> Vec<Enumerated> {
        enumerate()
    }

    pub fn open(index: usize) -> Result<Self> {
        let found = enumerate().into_iter().nth(index).ok_or(Error::NoDevice)?;
        Self::open_enumerated(found)
    }

    /// Open the dongle at that port or serial, else by index when `id`
    /// parses as a number.
    pub fn open_by_id(id: &str) -> Result<Self> {
        let list = enumerate();
        let found = list
            .iter()
            .find(|e| e.port == id || e.serial == id)
            .cloned()
            .or_else(|| id.parse::<usize>().ok().and_then(|i| list.into_iter().nth(i)))
            .ok_or(Error::NoDevice)?;
        Self::open_enumerated(found)
    }

    fn open_enumerated(found: Enumerated) -> Result<Self> {
        let mut devices = nusb::list_devices().wait().map_err(|e| Error::usb("list", e))?;
        // By port rather than by serial: every dongle of a kind ships with
        // the same one, so a serial match opens the first of them whichever
        // was asked for.
        let info = devices
            .find(|d| {
                d.vendor_id() == found.vid
                    && d.product_id() == found.pid
                    && port_path(d) == found.port
            })
            .ok_or(Error::NoDevice)?;

        let device = info.open().wait().map_err(|e| match e.kind() {
            nusb::ErrorKind::PermissionDenied => Error::Permission,
            _ => Error::usb("open", e),
        })?;

        #[cfg(target_os = "linux")]
        let _ = device.detach_kernel_driver(0);

        let iface = device.claim_interface(0).wait().map_err(|e| match e.kind() {
            nusb::ErrorKind::Busy => Error::Busy,
            _ => Error::usb("claim", e),
        })?;
        let t = Transport::new(iface);

        let board = if is_model(&found, "RTLSDRBlog", "Blog V4") {
            Board::BlogV4
        } else if is_model(&found, "RTLSDRBlog", "Blog V4L") {
            Board::BlogV4Lite
        } else {
            Board::Plain
        };

        let me = Self {
            t,
            state: Mutex::new(State {
                tuner: Tuner::Unknown,
                r82xx: None,
                tun_xtal: DEF_RTL_XTAL,
                rate: 0,
                freq: 0,
                corr: 0,
                direct_sampling: DirectSampling::Off,
                direct_sampling_mode: DirectSampling::Off,
                bias_tee: false,
                force_bt: false,
                baseband: false,
            }),
            board,
            manufacturer: found.manufacturer.clone(),
            product: found.product.clone(),
            serial: found.serial,
            streaming: AtomicBool::new(false),
        };

        // A write that has to work; if it does not, the device is wedged and
        // a reset gives it back.
        if me.t.write_reg(transport::block::USB, transport::usb_reg::SYSCTL, 0x09, 1).is_err() {
            let _ = device.reset().wait();
            return Err(Error::Usb("the dongle needed a reset; try again".into()));
        }

        me.init_baseband()?;

        {
            let mut st = me.state.lock().unwrap();
            st.force_bt =
                me.t.read_eeprom(7, 1)
                    .map(|d| d.first().is_some_and(|b| *b & 0x02 == 0))
                    .unwrap_or(false);

            me.probe_tuner(&mut st)?;
        }

        Ok(me)
    }

    pub fn manufacturer(&self) -> &str {
        &self.manufacturer
    }

    pub fn product(&self) -> &str {
        &self.product
    }

    pub fn serial(&self) -> &str {
        &self.serial
    }

    fn init_baseband(&self) -> Result<()> {
        let t = &self.t;
        t.write_reg(transport::block::USB, transport::usb_reg::SYSCTL, 0x09, 1)?;
        t.write_reg(transport::block::USB, transport::usb_reg::EPA_MAXPKT, 0x0002, 2)?;
        t.write_reg(transport::block::USB, transport::usb_reg::EPA_CTL, 0x1002, 2)?;

        t.write_reg(transport::block::SYS, transport::sys_reg::DEMOD_CTL_1, 0x22, 1)?;
        t.write_reg(transport::block::SYS, transport::sys_reg::DEMOD_CTL, 0xe8, 1)?;

        t.demod_write_reg(1, 0x01, 0x14, 1)?;
        t.demod_write_reg(1, 0x01, 0x10, 1)?;

        t.demod_write_reg(1, 0x15, 0x00, 1)?;
        t.demod_write_reg(1, 0x16, 0x0000, 2)?;
        for i in 0..6 {
            t.demod_write_reg(1, 0x16 + i, 0x00, 1)?;
        }

        self.set_fir()?;

        // SDR mode on, digital AGC off
        t.demod_write_reg(0, 0x19, 0x05, 1)?;

        t.demod_write_reg(1, 0x93, 0xf0, 1)?;
        t.demod_write_reg(1, 0x94, 0x0f, 1)?;
        t.demod_write_reg(1, 0x11, 0x00, 1)?;
        t.demod_write_reg(1, 0x04, 0x00, 1)?;
        t.demod_write_reg(0, 0x61, 0x60, 1)?;
        t.demod_write_reg(0, 0x06, 0x80, 1)?;
        // Zero-IF, DC cancellation and IQ estimation on
        t.demod_write_reg(1, 0xb1, 0x1b, 1)?;
        t.demod_write_reg(0, 0x0d, 0x83, 1)?;

        let mut st = self.state.lock().unwrap();
        st.baseband = true;
        Ok(())
    }

    fn set_fir(&self) -> Result<()> {
        let mut fir = [0u8; 20];
        for (i, v) in FIR_DEFAULT.iter().take(8).enumerate() {
            fir[i] = *v as i8 as u8;
        }
        for i in (0..8).step_by(2) {
            let (v0, v1) = (FIR_DEFAULT[8 + i], FIR_DEFAULT[9 + i]);
            let j = 8 + i * 3 / 2;
            fir[j] = (v0 >> 4) as u8;
            fir[j + 1] = ((v0 << 4) | ((v1 >> 8) & 0x0f)) as u8;
            fir[j + 2] = v1 as u8;
        }
        for (i, v) in fir.iter().enumerate() {
            self.t.demod_write_reg(1, 0x1c + i as u16, *v as u16, 1)?;
        }
        Ok(())
    }

    /// IF frequency register for the offset the demodulator sits at, from the
    /// corrected clock.
    fn set_if_freq(&self, freq: u32, rtl_xtal: u32) -> Result<()> {
        let if_freq = -((freq as i64 * (1 << 22)) / rtl_xtal as i64) as i32;
        let t = &self.t;
        t.demod_write_reg(1, 0x19, ((if_freq >> 16) & 0x3f) as u16, 1)?;
        t.demod_write_reg(1, 0x1a, ((if_freq >> 8) & 0xff) as u16, 1)?;
        t.demod_write_reg(1, 0x1b, (if_freq & 0xff) as u16, 1)
    }

    fn apply_ppm(&self, corr: i32) -> Result<()> {
        let offs = (corr as i64) * -(1 << 24) / 1_000_000;
        let t = &self.t;
        t.demod_write_reg(1, 0x3f, (offs & 0xff) as u16, 1)?;
        t.demod_write_reg(1, 0x3e, ((offs >> 8) & 0x3f) as u16, 1)
    }

    /// The clock, with the ppm correction applied
    fn xtal(&self, st: &State) -> u32 {
        (DEF_RTL_XTAL as f64 * (1.0 + st.corr as f64 / 1e6)) as u32
    }

    /// Reads the check register of each tuner through the repeater until one
    /// answers, then brings it up the way its type needs.
    fn probe_tuner(&self, st: &mut State) -> Result<()> {
        let t = &self.t;

        t.set_i2c_repeater(true)?;
        let tuner = {
            let e4k = t.i2c_read_reg(0x14, 0x02);
            let fc0013 = t.i2c_read_reg(0xc0, 0x00);
            let r820t = t.i2c_read_reg(r82xx::R820T_I2C_ADDR, r82xx::CHECK_ADDR);
            let r828d = t.i2c_read_reg(r82xx::R828D_I2C_ADDR, r82xx::CHECK_ADDR);
            if e4k == 0x40 {
                Tuner::E4000
            } else if fc0013 == 0xa3 {
                Tuner::Fc0013
            } else if r820t == r82xx::CHECK_VAL {
                Tuner::R820t
            } else if r828d == r82xx::CHECK_VAL {
                Tuner::R828d
            } else {
                Tuner::Unknown
            }
        };

        if tuner == Tuner::Unknown {
            // The FC2580 and FC0012 only answer after their reset line is
            // pulsed, which is GPIO 4.
            t.set_gpio_output(4)?;
            t.set_gpio_bit(4, true)?;
            t.set_gpio_bit(4, false)?;
            let fc2580 = t.i2c_read_reg(0x56, 0x01);
            let fc0012 = t.i2c_read_reg(0xc6, 0x00);
            if fc2580 & 0x7f == 0x56 {
                st.tuner = Tuner::Fc2580;
            } else if fc0012 == 0xa1 {
                t.set_gpio_output(6)?;
                st.tuner = Tuner::Fc0012;
            }
        } else {
            st.tuner = tuner;
        }

        match st.tuner {
            Tuner::R820t | Tuner::R828d => {
                let chip = match st.tuner {
                    Tuner::R828d => Chip::R828d,
                    _ => Chip::R820t,
                };
                // The Blog V4 runs its R828D from the dongle's own 28.8 MHz
                // reference; every other R828D board from its usual 16 MHz.
                st.tun_xtal = match chip {
                    Chip::R828d if self.board != Board::BlogV4 => r82xx::R828D_XTAL_FREQ,
                    _ => DEF_RTL_XTAL,
                };

                // Zero-IF off, I branch only.
                t.demod_write_reg(1, 0xb1, 0x1a, 1)?;
                t.demod_write_reg(0, 0x08, 0x4d, 1)?;
                self.set_if_freq(r82xx::IF_FREQ, self.xtal(st))?;
                // Spectrum inversion on
                t.demod_write_reg(1, 0x15, 0x01, 1)?;

                let mut r = R82xx::new(chip, self.board, self.xtal(st));
                r.init(t)?;
                st.r82xx = Some(r);
            }
            _ => {
                t.set_i2c_repeater(false)?;
                return Err(Error::Unsupported(format!(
                    "tuner {} is not driven by this driver; librtlsdr still covers it",
                    st.tuner.name()
                )));
            }
        }

        t.set_i2c_repeater(false)?;

        if st.force_bt {
            self.set_bias_tee_inner(st, true)?;
        }
        Ok(())
    }

    // ─── Controls ─────────────────────────────────────────────────────────

    pub fn tuner(&self) -> Tuner {
        self.state.lock().unwrap().tuner
    }

    pub fn tuner_gains(&self) -> &'static [i32] {
        self.tuner().gains()
    }

    /// Manual gain in tenths of a dB, snapped by the caller to a step in
    /// [`Self::tuner_gains`], or the tuner's own loop.
    pub fn set_tuner_gain(&self, manual: bool, tenths: i32) -> Result<()> {
        let mut st = self.state.lock().unwrap();
        if st.tuner == Tuner::Unknown {
            return Err(Error::Unsupported("no usable tuner".into()));
        }
        self.t.set_i2c_repeater(true)?;
        let r = match &mut st.r82xx {
            Some(r) => r.set_gain(&self.t, manual, tenths),
            None => Ok(()),
        };
        self.t.set_i2c_repeater(false)?;
        r
    }

    pub fn set_frequency(&self, freq: u32) -> Result<()> {
        let mut st = self.state.lock().unwrap();
        if st.tuner == Tuner::Unknown {
            return Err(Error::Unsupported("no usable tuner".into()));
        }

        // Auto direct sampling: an R820T's floor is 24 MHz, and below it the
        // only way in is an ADC branch, unless the board has an upconverter.
        let mut ds = st.direct_sampling_mode;
        if ds == DirectSampling::Off
            && freq < 24_000_000
            && st.tuner == Tuner::R820t
            && self.board != Board::BlogV4Lite
        {
            ds = DirectSampling::Q;
        }

        let last_ds = st.direct_sampling;
        if ds != DirectSampling::Off {
            self.enter_direct_sampling(&mut st, ds)?;
            self.set_if_freq(freq, self.xtal(&st))?;
        } else {
            self.t.set_i2c_repeater(true)?;
            let r = match &mut st.r82xx {
                Some(r) => r.set_freq(&self.t, freq),
                None => Err(Error::Unsupported("no usable tuner".into())),
            };
            self.t.set_i2c_repeater(false)?;
            r?;
        }
        st.freq = freq;
        st.direct_sampling = ds;

        if last_ds != ds {
            return self.apply_direct_sampling(&mut st);
        }
        Ok(())
    }

    pub fn frequency(&self) -> u32 {
        self.state.lock().unwrap().freq
    }

    pub fn set_ppm(&self, ppm: i32) -> Result<()> {
        let mut st = self.state.lock().unwrap();
        if st.corr == ppm {
            return Ok(());
        }
        st.corr = ppm;
        self.apply_ppm(ppm)?;
        let new_xtal = self.xtal(&st);
        if let Some(r) = &mut st.r82xx {
            r.set_xtal(new_xtal);
        }
        if st.freq > 0 {
            let freq = st.freq;
            drop(st);
            return self.set_frequency(freq);
        }
        Ok(())
    }

    pub fn ppm(&self) -> i32 {
        self.state.lock().unwrap().corr
    }

    pub fn set_sample_rate(&self, rate: u32) -> Result<()> {
        if rate <= 225_000 || rate > 3_200_000 || (rate > 300_000 && rate <= 900_000) {
            return Err(Error::Unsupported(format!(
                "{rate} Hz is not a rate the resampler can produce"
            )));
        }
        let mut st = self.state.lock().unwrap();

        let rsamp_ratio = (DEF_RTL_XTAL as u64 * (1 << 22)) / rate as u64;
        let rsamp_ratio = rsamp_ratio & 0x0ffffffc;
        let real_rsamp_ratio = rsamp_ratio | ((rsamp_ratio & 0x0800_0000) << 1);
        let real_rate = ((DEF_RTL_XTAL as u64 * (1 << 22)) / real_rsamp_ratio) as u32;
        st.rate = real_rate;

        // The channel filter tracks the rate, and setting it retunes.
        let xtal = self.xtal(&st);
        let rate = st.rate;
        if let Some(r) = &mut st.r82xx {
            self.t.set_i2c_repeater(true)?;
            let bw = r.set_bandwidth(&self.t, rate as i32);
            self.t.set_i2c_repeater(false)?;
            let int_freq = bw?;
            self.set_if_freq(int_freq, xtal)?;
        }

        let t = &self.t;
        t.demod_write_reg(1, 0x9f, (rsamp_ratio >> 16) as u16, 2)?;
        t.demod_write_reg(1, 0xa1, (rsamp_ratio & 0xffff) as u16, 2)?;
        self.apply_ppm(st.corr)?;
        t.demod_write_reg(1, 0x01, 0x14, 1)?;
        t.demod_write_reg(1, 0x01, 0x10, 1)?;
        Ok(())
    }

    pub fn sample_rate(&self) -> u32 {
        self.state.lock().unwrap().rate
    }

    /// RTL2832U digital AGC, which sits after the tuner and moves the floor
    /// on its own.
    pub fn set_rtl_agc(&self, on: bool) -> Result<()> {
        self.t.demod_write_reg(0, 0x19, if on { 0x25 } else { 0x05 }, 1)
    }

    pub fn set_bias_tee(&self, on: bool) -> Result<()> {
        let mut st = self.state.lock().unwrap();
        self.set_bias_tee_inner(&mut st, on)
    }

    fn set_bias_tee_inner(&self, st: &mut State, on: bool) -> Result<()> {
        let on = if st.force_bt { true } else { on };
        self.t.set_gpio_output(0)?;
        self.t.set_gpio_bit(0, on)?;
        st.bias_tee = on;
        Ok(())
    }

    pub fn bias_tee(&self) -> bool {
        self.state.lock().unwrap().bias_tee
    }

    pub fn direct_sampling(&self) -> DirectSampling {
        self.state.lock().unwrap().direct_sampling
    }

    /// What the operator asked for, which is also the mode: an `Off` here
    /// lets the auto-switch below 24 MHz decide each tune.
    pub fn set_direct_sampling_mode(&self, mode: DirectSampling) -> Result<()> {
        let mut st = self.state.lock().unwrap();
        st.direct_sampling_mode = mode;
        drop(st);
        self.apply_direct_sampling_mode()
    }

    fn apply_direct_sampling_mode(&self) -> Result<()> {
        let st = self.state.lock().unwrap();
        let mode = st.direct_sampling_mode;
        let freq = st.freq;
        drop(st);
        if freq > 0 {
            self.set_frequency(freq)?;
        } else {
            let st = self.state.lock().unwrap();
            if st.direct_sampling != mode {
                let mut st = self.state.lock().unwrap();
                self.enter_direct_sampling(&mut st, mode)?;
            }
        }
        Ok(())
    }

    /// Bring the demodulator into direct sampling on one ADC branch and put
    /// the tuner to sleep.
    fn enter_direct_sampling(&self, st: &mut State, on: DirectSampling) -> Result<()> {
        if on == DirectSampling::Off {
            // Waking the tuner back up
            self.t.set_i2c_repeater(true)?;
            let r = match &mut st.r82xx {
                Some(r) => r.init(&self.t),
                None => Ok(()),
            };
            self.t.set_i2c_repeater(false)?;
            r?;
            if st.tuner == Tuner::R820t || st.tuner == Tuner::R828d {
                self.set_if_freq(r82xx::IF_FREQ, self.xtal(st))?;
                self.t.demod_write_reg(1, 0x15, 0x01, 1)?;
            }
            self.t.demod_write_reg(0, 0x06, 0x80, 1)?;
            st.direct_sampling = DirectSampling::Off;
            return Ok(());
        }

        self.t.set_i2c_repeater(true)?;
        let r = match &mut st.r82xx {
            Some(r) => r.standby(&self.t),
            None => Ok(()),
        };
        self.t.set_i2c_repeater(false)?;
        r?;

        // Zero-IF off, spectrum inversion off, I branch ADC input only.
        self.t.demod_write_reg(1, 0xb1, 0x1a, 1)?;
        self.t.demod_write_reg(1, 0x15, 0x00, 1)?;
        self.t.demod_write_reg(0, 0x08, 0x4d, 1)?;
        // Swapping the ADC inputs picks between the two sockets.
        self.t.demod_write_reg(0, 0x06, if on == DirectSampling::Q { 0x90 } else { 0x80 }, 1)?;
        st.direct_sampling = on;
        Ok(())
    }

    /// The tuner-independent half of a mode change, which is what a control
    /// call lands on when nothing is tuned yet.
    fn apply_direct_sampling(&self, st: &mut State) -> Result<()> {
        let ds = st.direct_sampling;
        let freq = st.freq;
        let _ = freq;
        if ds != DirectSampling::Off {
            self.enter_direct_sampling(st, ds)?;
            if freq > 0 {
                self.set_if_freq(freq, self.xtal(st))?;
            }
        } else {
            self.enter_direct_sampling(st, DirectSampling::Off)?;
        }
        Ok(())
    }

    // ─── Streaming ────────────────────────────────────────────────────────

    /// Flushes the FIFO and opens the bulk endpoint for reading.
    pub fn reset_buffer(&self) -> Result<()> {
        self.t.write_reg(transport::block::USB, transport::usb_reg::EPA_CTL, 0x1002, 2)?;
        self.t.write_reg(transport::block::USB, transport::usb_reg::EPA_CTL, 0x0000, 2)
    }

    /// Start reading samples. Returns a reader over finished transfers; the
    /// reader does not own the device, so controls stay usable while it runs.
    pub fn start_rx(&self) -> Result<Reader> {
        if self.streaming.swap(true, Ordering::SeqCst) {
            return Err(Error::Busy);
        }
        self.reset_buffer()?;
        let mut ep = self.t.interface().endpoint::<Bulk, In>(RX_ENDPOINT).map_err(|e| {
            self.streaming.store(false, Ordering::SeqCst);
            Error::usb("open endpoint", e)
        })?;
        for _ in 0..TRANSFERS {
            ep.submit(nusb::transfer::Buffer::new(XFER_BYTES));
        }
        let dropped = Arc::new(AtomicU64::new(0));
        let streaming = Arc::new(AtomicBool::new(true));
        self.streaming.store(true, Ordering::SeqCst);
        Ok(Reader { ep, streaming, dropped, stop: Arc::new(AtomicBool::new(false)) })
    }
}

fn is_model(e: &Enumerated, manufacturer: &str, product: &str) -> bool {
    e.manufacturer.eq_ignore_ascii_case(manufacturer) && e.product.eq_ignore_ascii_case(product)
}

/// One bulk transfer at a time, off the queue the endpoint keeps full.
pub struct Reader {
    ep: nusb::Endpoint<Bulk, In>,
    streaming: Arc<AtomicBool>,
    pub dropped: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
}

impl Reader {
    /// The next block of offset-binary I/Q bytes. Blocks until one arrives.
    /// A transfer that has not landed keeps being waited on, so a quiet band
    /// is not the end of the stream.
    pub fn read(&mut self) -> Result<Vec<u8>> {
        if self.stop.load(Ordering::SeqCst) {
            return Err(Error::Usb("stopped".into()));
        }
        loop {
            let completion = match self.ep.wait_next_complete(BULK_TIMEOUT) {
                Some(c) => c,
                None => {
                    if self.stop.load(Ordering::SeqCst) {
                        self.streaming.store(false, Ordering::SeqCst);
                        return Err(Error::Usb("stopped".into()));
                    }
                    continue;
                }
            };
            match completion.status {
                Ok(()) => {
                    self.ep.submit(nusb::transfer::Buffer::new(XFER_BYTES));
                    return Ok(completion.buffer.into_vec());
                }
                // A cancelled transfer is what a stop looks like.
                Err(_e) if self.stop.load(Ordering::SeqCst) => {
                    self.streaming.store(false, Ordering::SeqCst);
                    return Err(Error::Usb("stopped".into()));
                }
                Err(e) => {
                    self.streaming.store(false, Ordering::SeqCst);
                    return Err(Error::usb("bulk transfer", e));
                }
            }
        }
    }

    pub fn stop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        self.ep.cancel_all();
        self.streaming.store(false, Ordering::SeqCst);
    }
}

impl Drop for Reader {
    fn drop(&mut self) {
        self.stop();
        // Drain the queue so its completion handler never races the drop.
        while self.ep.pending() > 0 {
            if self.ep.wait_next_complete(Duration::from_millis(100)).is_none() {
                break;
            }
        }
        self.streaming.store(false, Ordering::SeqCst);
    }
}
