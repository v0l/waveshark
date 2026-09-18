//! Reads two spans at different gains and a quiet one at the noise floor, and
//! retunes mid-stream. A stream that only ever returned silence would pass
//! everything above.

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dev = rtlsdr_usb::RtlSdr::open(0)?;
    println!("tuner: {:?}", dev.tuner());

    dev.set_sample_rate(2_048_000)?;
    dev.set_ppm(0)?;
    dev.set_tuner_gain(true, 496)?; // max

    let mut reader = dev.start_rx()?;
    let mut drain = |secs: u64| -> (u64, f64, f64) {
        let start = std::time::Instant::now();
        let (mut bytes, mut sum, mut sumsq, mut n) = (0u64, 0f64, 0f64, 0u64);
        while start.elapsed() < std::time::Duration::from_secs(secs) {
            if let Ok(b) = reader.read() {
                for &v in &b {
                    let s = (v as f64 - 127.5) / 127.5;
                    sum += s;
                    sumsq += s * s;
                    n += 1;
                }
                bytes += b.len() as u64;
            }
        }
        let mean = sum / n as f64;
        let rms = (sumsq / n as f64 - mean * mean).sqrt();
        (bytes, mean, rms)
    };

    // 433.65 MHz is the ISM band: usually some traffic on a workbench antenna,
    // 49.6 dB of gain ahead of the ADC.
    dev.set_frequency(433_650_000)?;
    let (b1, m1, r1) = drain(1);
    println!("433.65 MHz  gain 49.6: {} bytes, mean {:.3}, rms {:.3}", b1, m1, r1);

    // Retune while streaming: the control path has to survive this.
    dev.set_frequency(1090_000_000)?;
    let (b2, m2, r2) = drain(1);
    println!("1090 MHz    gain 49.6: {} bytes, mean {:.3}, rms {:.3}", b2, m2, r2);

    // Minimum gain, terminated input if nothing is on the antenna, is the
    // noise floor: rms well below 0.5 and no DC bias.
    dev.set_tuner_gain(true, 0)?;
    let (b3, m3, r3) = drain(1);
    println!("433.65 MHz  gain 0.0:  {} bytes, mean {:.3}, rms {:.3}", b3, m3, r3);
    reader.stop();
    Ok(())
}
