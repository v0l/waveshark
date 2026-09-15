//! How fast the radio will take samples, which is what decides the widest
//! signal it can transmit.
//!
//! Usage: `tx_rate [rate_sps] [seconds]`
//!
//! Zeros, at the bottom of the gain range and with the amplifier off, so
//! what goes out is the LO leakage and nothing else. What it measures is the
//! rate the device drains its queue at: a radio that takes 10 MS/s takes an
//! 8 MHz television multiplex, and one that takes three does not.

use common::{C32, Device, GainMode, Hz, IqBuf, Sps};

fn main() -> common::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .init();
    let mut args = std::env::args().skip(1);
    let rate = Sps(args.next().and_then(|s| s.parse().ok()).unwrap_or(10_000_000));
    let secs: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(5.0);

    let mut dev = hackrf::HackRfDevice::open_first()?;
    println!("{} at {:.3} MS/s", dev.info().label, rate.as_f64() / 1e6);
    dev.set_rate(rate)?;
    dev.set_center(Hz(770_000_000))?;
    dev.set_tx_gain("amp", GainMode::Manual(0.0))?;
    dev.set_tx_gain("txvga", GainMode::Manual(0.0))?;

    // The length the receiver's own transmit chain hands over: what the
    // driver gives the graph while a half duplex radio is deaf.
    let block = (rate.as_f64() * 0.02) as usize;
    let samples = vec![C32::default(); block];
    let mut stream = dev.start_tx()?;

    let start = std::time::Instant::now();
    let mut sent = 0u64;
    let mut worst = std::time::Duration::ZERO;
    while start.elapsed().as_secs_f64() < secs {
        let buf = IqBuf::new(samples.clone(), Hz(770_000_000), rate, sent);
        let at = std::time::Instant::now();
        stream.write(&buf)?;
        worst = worst.max(at.elapsed());
        sent += block as u64;
    }
    let took = start.elapsed().as_secs_f64();
    // What is still queued counts as taken otherwise: the writer holds a
    // few hundred milliseconds, which over a short run reads as a radio
    // faster than its own clock.
    println!(
        "{:.3} MS/s taken over {took:.2} s, worst write {:.1} ms, {} idle transfers",
        sent as f64 / took / 1e6,
        worst.as_secs_f64() * 1e3,
        stream.underruns()
    );
    stream.stop();
    Ok(())
}
