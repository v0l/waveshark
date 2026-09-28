use std::time::{Duration, Instant};

fn drain(reader: &mut airspy_usb::Reader, secs: f64) -> (f64, f64, f64) {
    let start = Instant::now();
    let (mut n, mut sum, mut sumsq) = (0u64, 0f64, 0f64);
    while start.elapsed() < Duration::from_secs_f64(secs) {
        let Ok(bytes) = reader.read() else { break };
        for w in bytes.chunks_exact(2) {
            let v = (u16::from_le_bytes([w[0], w[1]]) & 0x0fff) as f64 - 2048.0;
            sum += v;
            sumsq += v * v;
            n += 1;
        }
    }
    let mean = sum / n.max(1) as f64;
    let rms = (sumsq / n.max(1) as f64 - mean * mean).sqrt();
    (n as f64 / start.elapsed().as_secs_f64(), mean, rms)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let found = airspy_usb::enumerate();
    println!("found: {found:?}");
    let dev = airspy_usb::Airspy::open(0)?;
    println!("board id: {}", dev.board_id()?);
    println!("firmware: {}", dev.version()?);
    let rates = dev.sample_rates()?;
    println!("rates: {rates:?}");
    dev.set_lna_agc(false)?;
    dev.set_mixer_agc(false)?;
    dev.set_lna_gain(10)?;
    dev.set_mixer_gain(8)?;
    dev.set_vga_gain(8)?;
    for (i, rate) in rates.iter().enumerate() {
        dev.set_sample_rate_index(i as u16)?;
        println!("streaming at {rate}");
        let mut reader = dev.start_rx()?;
        dev.set_frequency(100_000_000)?;
        let (per_sec, mean, rms) = drain(&mut reader, 1.0);
        println!(
            "{rate} IQ/s: {per_sec:.0} real samples/s ({:.3} of the {} due), mean {mean:.1}, rms {rms:.1} counts",
            per_sec / (2.0 * *rate as f64),
            2 * rate
        );
        dev.set_frequency(1_090_000_000)?;
        let (_, mean, rms) = drain(&mut reader, 0.5);
        println!("  retuned to 1090 MHz while streaming: mean {mean:.1}, rms {rms:.1} counts");
        dev.set_lna_gain(0)?;
        dev.set_mixer_gain(0)?;
        dev.set_vga_gain(0)?;
        let (_, mean, rms) = drain(&mut reader, 0.5);
        println!("  every gain at its lowest: mean {mean:.1}, rms {rms:.1} counts");
        dev.set_lna_gain(10)?;
        dev.set_mixer_gain(8)?;
        dev.set_vga_gain(8)?;
        reader.stop();
    }
    Ok(())
}
