//! Is the LimeSDR's receive front end alive?
//!
//! Three questions in order: does the converter and the path back to the host
//! work at all (test tone), does the noise floor respond to the RF gain the
//! way an analogue front end does, and does disconnecting the antenna port
//! change anything.
use common::{Device, GainMode, Hz, Sps};

fn floor_and_peak(d: &mut limesdr::LimeSdr) -> (f32, f32, f64) {
    let mut s = d.start_rx().unwrap();
    let mut sp = dsp::spectrum::Spectrum::new(4096);
    for _ in 0..15 { let _ = s.read(); }
    for _ in 0..8 { let b = s.read().unwrap(); sp.process(&b.samples); }
    drop(s);
    let db = sp.power_db().to_vec();
    let n = db.len();
    let (mut peak, mut at) = (f32::MIN, 0usize);
    for (i, v) in db.iter().enumerate() {
        if i.abs_diff(n / 2) < 16 { continue; }
        if *v > peak { peak = *v; at = i; }
    }
    let mut sorted = db.clone();
    sorted.sort_by(f32::total_cmp);
    (sorted[n / 2], peak, (at as f64 - n as f64 / 2.0) * 4e6 / n as f64)
}

fn main() {
    let mut d = limesdr::LimeSdr::open(0).unwrap();
    d.set_rate(Sps(4_000_000)).unwrap();
    let chan = std::env::args().nth(1).unwrap_or_else(|| "RX2".into());
    let ant = std::env::args().nth(2).unwrap_or_else(|| "LNAH".into());
    d.set_choice("channel", &chan).unwrap();
    d.set_choice("antenna", &ant).unwrap();
    d.set_center(Hz(95_800_000)).unwrap();
    println!("{chan} / {ant} at 95.8 MHz, 4 MS/s, chip {:.0} C\n", d.chip_temperature().unwrap_or(0.0));

    d.set_gain("gain", GainMode::Manual(40.0)).unwrap();
    d.set_toggle("test_signal", true).unwrap();
    let (f0, p0, off) = floor_and_peak(&mut d);
    println!("test tone   floor {f0:>7.1}  peak {p0:>7.1} dBFS at {:+.0} kHz  ({:.1} dB up)", off / 1e3, p0 - f0);
    d.set_toggle("test_signal", false).unwrap();

    println!("\ngain sweep, floor should climb with it once the front end's own noise wins");
    for g in [0.0f32, 20.0, 30.0, 40.0, 50.0, 60.0, 73.0] {
        d.set_gain("gain", GainMode::Manual(g)).unwrap();
        let (fl, pk, off) = floor_and_peak(&mut d);
        println!("  {g:>4.0} dB   floor {fl:>7.1}  peak {pk:>7.1} at {:+7.0} kHz  ({:>4.1} dB up)", off / 1e3, pk - fl);
    }

    println!("\nsame gain, each port in turn. A live front end differs between them.");
    d.set_gain("gain", GainMode::Manual(60.0)).unwrap();
    for a in ["LNAH", "LNAL", "LNAW"] {
        d.set_choice("antenna", a).unwrap();
        let (fl, pk, off) = floor_and_peak(&mut d);
        println!("  {a:<5}    floor {fl:>7.1}  peak {pk:>7.1} at {:+7.0} kHz  ({:>4.1} dB up)", off / 1e3, pk - fl);
    }
}
