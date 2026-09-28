use airspy_usb::hf::AirspyHf;
use std::time::{Duration, Instant};

fn drain(reader: &mut airspy_usb::Reader, secs: f64) -> (f64, f64, f64) {
    let start = Instant::now();
    let (mut n, mut ii, mut qq) = (0u64, 0f64, 0f64);
    while start.elapsed() < Duration::from_secs_f64(secs) {
        let Ok(bytes) = reader.read() else { break };
        for w in bytes.chunks_exact(4) {
            let q = i16::from_le_bytes([w[0], w[1]]) as f64;
            let i = i16::from_le_bytes([w[2], w[3]]) as f64;
            ii += i * i;
            qq += q * q;
            n += 1;
        }
    }
    let n = n.max(1) as f64;
    (n / start.elapsed().as_secs_f64(), (ii / n).sqrt(), (qq / n).sqrt())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let found = airspy_usb::hf::enumerate();
    println!("found: {found:?}");
    let dev = AirspyHf::open(0)?;
    println!("firmware: {}", dev.version()?);
    let rates = dev.sample_rates()?;
    println!("rates: {rates:?}");
    println!("low IF: {:?}", dev.low_if(rates.len())?);
    println!("attenuator steps: {:?}", dev.att_steps()?);
    println!("bias tees: {}", dev.bias_tees());
    println!("calibration: {:?}", dev.calibration()?);
    dev.set_agc(true)?;
    for (i, rate) in rates.iter().enumerate() {
        dev.set_sample_rate_index(i as u16)?;
        println!("{rate} S/s: filter gain {} dB", dev.filter_gain_db()?);
        dev.set_frequency_khz(7_079)?;
        println!("  7079 kHz: synthesiser off by {:.3} Hz", dev.freq_delta_hz()?);
        println!("  streaming");
        let mut reader = dev.start_rx()?;
        let (per_sec, i_rms, q_rms) = drain(&mut reader, 1.0);
        println!(
            "  {per_sec:.0} samples/s ({:.3} of the rate), rms I {i_rms:.1} Q {q_rms:.1} counts",
            per_sec / *rate as f64
        );
        dev.set_frequency_khz(145_005)?;
        let (_, i_rms, q_rms) = drain(&mut reader, 0.5);
        println!("  retuned to 145005 kHz while streaming: rms I {i_rms:.1} Q {q_rms:.1} counts");
        reader.stop();
    }
    Ok(())
}
