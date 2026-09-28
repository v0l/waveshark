use super::*;
use common::{Device, DeviceInfo, DriverKind, GainStage, Toggle, TunerRange};

/// A radio with three stages that quantise, a switch, and a "tuner"
/// handle that distributes a total across the stages. All three
/// behaviours a reopen has to survive, and all three a HackRF has.
struct ThreeStages {
    info: DeviceInfo,
    tuning: common::Tuning,
    amp: bool,
    lna: u32,
    vga: u32,
    bias_tee: bool,
    trim: f64,
}

impl ThreeStages {
    fn new() -> Self {
        let stage = |name: &str, hi: f32, step: f32| GainStage {
            name: name.into(),
            label: name.into(),
            range: 0.0..=hi,
            values: Vec::new(),
            step,
            auto: false,
        };
        Self {
            info: DeviceInfo {
                kind: DriverKind::HackRf,
                id: "stub".into(),
                label: "Stub".into(),
                tuner: "none".into(),
                ranges: vec![TunerRange { label: "rx", range: Hz(1_000_000)..=Hz(6_000_000_000) }],
                rates: Vec::new(),
                rate_range: Sps(2_000_000)..=Sps(20_000_000),
                gain_stages: vec![
                    stage("amp", 14.0, 14.0),
                    stage("lna", 40.0, 8.0),
                    stage("vga", 62.0, 2.0),
                ],
                native_format: common::SampleFormat::Cs8,
                tunable: true,
                centre_spur: true,
                tx: None,
            },
            tuning: common::Tuning::default(),
            amp: false,
            lna: 0,
            vga: 0,
            bias_tee: false,
            trim: 0.0,
        }
    }
}

impl common::Device for ThreeStages {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }
    fn set_center(&mut self, _f: Hz) -> common::Result<()> {
        Ok(())
    }
    fn center(&self) -> Hz {
        Hz(100_000_000)
    }
    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }
    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }
    fn set_rate(&mut self, _r: Sps) -> common::Result<()> {
        Ok(())
    }
    fn rate(&self) -> Sps {
        Sps(2_000_000)
    }
    fn set_gain(&mut self, stage: &str, mode: GainMode) -> common::Result<()> {
        let db = match mode {
            GainMode::Auto => 32.0,
            GainMode::Manual(db) => db,
        };
        match stage {
            "tuner" => {
                self.amp = db > 102.0;
                let rest = (db - if self.amp { 14.0 } else { 0.0 }).max(0.0);
                self.lna = ((rest / 2.0) as u32 / 8 * 8).min(40);
                self.vga = ((rest - self.lna as f32) as u32 / 2 * 2).min(62);
            }
            "amp" => self.amp = db >= 7.0,
            "lna" => self.lna = (db as u32 / 8 * 8).min(40),
            "vga" => self.vga = (db as u32 / 2 * 2).min(62),
            _ => return Err(common::Error::other("no such stage")),
        }
        Ok(())
    }
    fn gains(&self) -> Vec<(String, GainMode)> {
        vec![
            ("amp".into(), GainMode::Manual(if self.amp { 14.0 } else { 0.0 })),
            ("lna".into(), GainMode::Manual(self.lna as f32)),
            ("vga".into(), GainMode::Manual(self.vga as f32)),
        ]
    }
    fn toggles(&self) -> Vec<Toggle> {
        vec![Toggle {
            name: "bias_tee".into(),
            label: "Bias tee".into(),
            help: String::new(),
            on: self.bias_tee,
        }]
    }
    fn set_toggle(&mut self, name: &str, on: bool) -> common::Result<()> {
        match name {
            "bias_tee" => self.bias_tee = on,
            _ => return Err(common::Error::other("no such switch")),
        }
        Ok(())
    }
    fn numbers(&self) -> Vec<common::Number> {
        vec![common::Number {
            name: "trim".into(),
            label: "Trim".into(),
            help: String::new(),
            range: -1_000.0..=1_000.0,
            step: 1.0,
            unit: "Hz".into(),
            value: self.trim,
        }]
    }
    fn set_number(&mut self, name: &str, value: f64) -> common::Result<()> {
        match name {
            "trim" => self.trim = value,
            _ => return Err(common::Error::other("no such number")),
        }
        Ok(())
    }
    fn start_rx(&mut self) -> common::Result<Box<dyn common::RxStream>> {
        Err(common::Error::other("not a real radio"))
    }
}

/// Reopening for a span change puts every stage back, not the total: the
/// HackRF came back with the VGA at zero because only "tuner" was
/// remembered, and a driver that distributes a total does not land on
/// what the operator set stage by stage.
#[test]
fn a_reopen_restores_every_stage_switch_and_number() {
    let mut was = ThreeStages::new();
    was.set_gain("lna", GainMode::Manual(24.0)).unwrap();
    was.set_gain("vga", GainMode::Manual(45.0)).unwrap();
    was.set_gain("amp", GainMode::Manual(14.0)).unwrap();
    was.set_toggle("bias_tee", true).unwrap();
    was.set_number("trim", -310.0).unwrap();
    // What the hardware landed on, which is not quite what was asked for.
    assert_eq!(
        was.gains(),
        vec![
            ("amp".to_string(), GainMode::Manual(14.0)),
            ("lna".to_string(), GainMode::Manual(24.0)),
            ("vga".to_string(), GainMode::Manual(44.0)),
        ]
    );

    let front = FrontEnd::read(&was);
    let mut back = ThreeStages::new();
    assert_eq!(back.gains()[2].1, GainMode::Manual(0.0), "a fresh device is at its defaults");
    front.apply(&mut back);

    assert_eq!(back.gains(), was.gains());
    assert!(back.bias_tee, "the bias tee is a front end setting and goes back too");
    assert_eq!(back.numbers()[0].value, -310.0, "and so does a number");
    assert_eq!(RadioControls::read(&back).numbers[0].value, -310.0);
}

struct Claim(Arc<std::sync::atomic::AtomicBool>);

impl Drop for Claim {
    fn drop(&mut self) {
        self.0.store(false, Ordering::SeqCst);
    }
}

struct OneClaim {
    info: DeviceInfo,
    tuning: common::Tuning,
    claim: Arc<Claim>,
}

impl OneClaim {
    fn open(usb: &Arc<std::sync::atomic::AtomicBool>) -> common::Result<Box<dyn Device>> {
        if usb.swap(true, Ordering::SeqCst) {
            return Err(common::Error::other("Cannot claim interface - Resource busy"));
        }
        Ok(Box::new(Self {
            info: ThreeStages::new().info,
            tuning: common::Tuning::default(),
            claim: Arc::new(Claim(usb.clone())),
        }))
    }
}

struct HeldStream(#[allow(dead_code)] Arc<Claim>);

impl common::RxStream for HeldStream {
    fn read(&mut self) -> common::Result<common::IqBuf> {
        Err(common::Error::Disconnected)
    }
    fn dropped(&self) -> u64 {
        0
    }
    fn stop(&mut self) {}
}

impl Device for OneClaim {
    fn info(&self) -> &DeviceInfo {
        &self.info
    }
    fn set_center(&mut self, _f: Hz) -> common::Result<()> {
        Ok(())
    }
    fn center(&self) -> Hz {
        Hz(1_097_000_000)
    }
    fn tuning(&self) -> &common::Tuning {
        &self.tuning
    }
    fn tuning_mut(&mut self) -> &mut common::Tuning {
        &mut self.tuning
    }
    fn set_rate(&mut self, _r: Sps) -> common::Result<()> {
        Ok(())
    }
    fn rate(&self) -> Sps {
        Sps(40_000_000)
    }
    fn set_gain(&mut self, _stage: &str, _mode: GainMode) -> common::Result<()> {
        Ok(())
    }
    fn start_rx(&mut self) -> common::Result<Box<dyn common::RxStream>> {
        Ok(Box::new(HeldStream(self.claim.clone())))
    }
}

#[test]
fn a_radio_one_program_may_hold_is_let_go_before_it_is_opened_again() {
    let usb = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let mut dev = OneClaim::open(&usb).unwrap();
    dev.set_offset(9_750_000_000.0);
    dev.correct(1.5);
    let mut stream = Some(dev.start_rx().unwrap());
    assert!(OneClaim::open(&usb).is_err(), "a LimeSDR is claimed by one handle at a time");

    restart(
        || OneClaim::open(&usb),
        &mut dev,
        &mut stream,
        Sps(30_720_000),
        Hz(10_847_000_000),
        &FrontEnd::default(),
    )
    .expect("the span changes on a radio this program already held");
    assert!(stream.is_some());
    assert_eq!((dev.offset(), dev.asked_ppm()), (9_750_000_000.0, 1.5));

    drop(stream.take());
    assert!(
        restart(
            || Err(common::Error::other("unplugged")),
            &mut dev,
            &mut stream,
            Sps(30_720_000),
            Hz(10_847_000_000),
            &FrontEnd::default(),
        )
        .is_err()
    );
    assert!(!usb.load(Ordering::SeqCst), "a failed reopen leaves nothing claimed");
    assert_eq!(dev.offset(), 9_750_000_000.0, "and keeps the converter for the next try");
}
