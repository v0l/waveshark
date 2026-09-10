//! Hardware smoke test: tune, channelize, and report the strongest channels.
//!
//! Usage: cargo run --release -p rtlsdr --example scan -- [center_mhz] [gain_db]

use common::device::{Device, GainMode};
use common::{Hz, Sps};
use dsp::Channelizer;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let center_mhz: f64 = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(100.0);
    let gain: Option<f32> = args.get(2).and_then(|s| s.parse().ok());

    let devs = rtlsdr::enumerate();
    println!("devices: {devs:#?}");
    let d = devs.first().ok_or("no RTL-SDR found")?;

    let mut sdr = rtlsdr::RtlSdr::open(d.index)?;
    println!("opened: {} / tuner {}", sdr.info().label, sdr.info().tuner);
    println!("gain steps: {:?}", sdr.supported_gains());

    let rate = Sps(2_400_000);
    sdr.set_rate(rate)?;
    sdr.set_center(Hz((center_mhz * 1e6) as u64))?;
    sdr.set_gain(
        "tuner",
        match gain {
            Some(g) => GainMode::Manual(g),
            None => GainMode::Auto,
        },
    )?;

    println!(
        "tuned to {} (actual {}), rate {}",
        sdr.center(),
        sdr.actual_center(),
        sdr.actual_rate()
    );

    const CHANNELS: usize = 64;
    let mut ch = Channelizer::new(CHANNELS, 12, 90.0);
    let mut power = vec![0.0f64; CHANNELS];
    let mut frames = 0u64;

    let mut rx = sdr.start_rx()?;
    let t0 = std::time::Instant::now();
    let mut total = 0u64;
    while t0.elapsed() < std::time::Duration::from_secs(2) {
        let buf = rx.read()?;
        total += buf.len() as u64;
        ch.process(&buf.samples, |f| {
            frames += 1;
            for (p, s) in power.iter_mut().zip(f.samples) {
                *p += s.norm_sqr() as f64;
            }
        });
    }
    rx.stop();

    let secs = t0.elapsed().as_secs_f64();
    println!(
        "\ncaptured {total} samples in {secs:.2}s = {:.3} MS/s (dropped {})",
        total as f64 / secs / 1e6,
        rx.dropped()
    );
    println!(
        "channelizer: {frames} frames, {} channels @ {:.1} kS/s each, {:.1} kHz wide",
        CHANNELS,
        ch.channel_rate(rate.as_f64()) / 1e3,
        ch.channel_bandwidth(rate.as_f64()) / 1e3
    );

    let mut idx: Vec<usize> = (0..CHANNELS).collect();
    idx.sort_by(|&a, &b| power[b].total_cmp(&power[a]));
    let floor = {
        let mut v: Vec<f64> = power.clone();
        v.sort_by(f64::total_cmp);
        v[CHANNELS / 2]
    };

    println!("\ntop 12 channels (median floor = {:.1} dB):", 10.0 * floor.log10());
    for &m in idx.iter().take(12) {
        let f = sdr.center().as_f64() + ch.channel_offset_hz(m, rate.as_f64());
        let snr = 10.0 * (power[m] / floor).log10();
        println!(
            "  ch{m:3}  {:9.4} MHz   {:+7.1} dB over floor  {}",
            f / 1e6,
            snr,
            "#".repeat((snr.max(0.0) / 2.0) as usize).chars().take(40).collect::<String>()
        );
    }
    Ok(())
}
