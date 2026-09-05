//! Transmit a tone, to prove the transmit path against real hardware.
//!
//! Usage: `tx_tone [freq_hz] [offset_hz] [seconds] [txvga_db] [amp]`
//!
//! The tone is offset from the tuned centre so the receiver sees it beside
//! the LO leakage rather than under it: a carrier exactly at centre is
//! indistinguishable from the DC spur every direct conversion radio has.
//!
//! This radiates. Use a dummy load or a screened enclosure, keep the gain at
//! the bottom of its range, and check what band you are pointing it at.

use common::{Device, GainMode, Hz, IqBuf, Sps, C32};

fn main() -> common::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let mut args = std::env::args().skip(1);
    let freq: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(868_500_000);
    let offset: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(100_000.0);
    let secs: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(1.0);
    let txvga: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let amp: bool = args.next().is_some_and(|s| s == "1" || s == "amp" || s == "on");

    let rate = Sps(2_000_000);
    let mut dev = hackrf::HackRfDevice::open_first()?;
    let info = dev.info().clone();
    let tx = info.tx.as_ref().expect("a HackRF reports a transmitter");
    println!("{} ({})", info.label, info.tuner);
    println!(
        "transmit stages: {}",
        tx.gain_stages
            .iter()
            .map(|s| s.name.as_str())
            .collect::<Vec<_>>()
            .join(", ")
    );
    assert!(info.covers_tx(Hz(freq)), "{freq} Hz is outside the transmit range");

    dev.set_rate(rate)?;
    dev.set_center(Hz(freq))?;
    dev.set_tx_gain("amp", GainMode::Manual(if amp { 14.0 } else { 0.0 }))?;
    dev.set_tx_gain("txvga", GainMode::Manual(txvga))?;

    // A quarter of full scale: the DAC clips at 1.0 and a clipped tone is
    // spread across the band rather than confined to one bin.
    const AMPLITUDE: f32 = 0.25;
    let block = 32768usize;
    let step = std::f64::consts::TAU * offset / rate.as_f64();
    let mut phase = 0.0f64;

    let mut stream = dev.start_tx()?;
    println!(
        "transmitting {:.1} s at {:.4} MHz ({} Hz tone), TXVGA {txvga} dB, amp {}",
        secs,
        (freq as f64 + offset) / 1e6,
        offset as i64,
        if amp { "on" } else { "off" }
    );

    let total = (secs * rate.as_f64()) as u64;
    let mut sent = 0u64;
    let start = std::time::Instant::now();
    while sent < total {
        let n = block.min((total - sent) as usize);
        let mut samples = Vec::with_capacity(n);
        for _ in 0..n {
            let (s, c) = phase.sin_cos();
            samples.push(C32::new(c as f32 * AMPLITUDE, s as f32 * AMPLITUDE));
            phase += step;
            if phase > std::f64::consts::TAU {
                phase -= std::f64::consts::TAU;
            }
        }
        stream.write(&IqBuf::new(samples, Hz(freq), rate, sent))?;
        sent += n as u64;
    }
    stream.drain(std::time::Duration::from_secs(2));
    stream.stop();

    println!(
        "sent {sent} samples in {:.2} s, {} unfilled transfers",
        start.elapsed().as_secs_f64(),
        stream.underruns()
    );
    Ok(())
}
