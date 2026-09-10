//! Look at what is actually on an ISM band, before writing any decoder.
//!
//! Prints pulse trains and their timing histograms. Those histograms are the
//! diagnostic that matters: a PWM protocol shows two distinct mark clusters, a
//! PPM protocol shows one mark cluster and two gap clusters, and Manchester
//! shows clusters at T and 2T in both. Identifying the coding this way takes
//! seconds and saves guessing at a decoder that was never going to work.
//!
//! Usage: cargo run --release -p rtlsdr --example ism -- [mhz] [secs] [gain]

use common::device::{Device, GainMode};
use common::{Hz, Sps};
use dsp::{FirDecim, FskConfig, FskDetector, Mixer, OokDetector, PulseConfig};

const RF_RATE: u64 = 2_400_000;
/// 2.4 MS/s / 8 = 300 kHz, wide enough for ISM channels and a convenient
/// integer divisor.
const DECIM: usize = 8;
const ENV_RATE: f64 = RF_RATE as f64 / DECIM as f64;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    let mhz: f64 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(868.3);
    let secs: f64 = a.get(2).and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let gain: f32 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(49.6);

    let d = rtlsdr::enumerate();
    let d = d.first().ok_or("no RTL-SDR")?;
    let mut sdr = rtlsdr::RtlSdr::open(d.index)?;
    sdr.set_rate(Sps(RF_RATE))?;
    sdr.set_gain("tuner", GainMode::Manual(gain))?;

    // Offset tuning keeps the signal of interest off the DC spur.
    let offset = 300_000.0f64;
    let target = Hz((mhz * 1e6) as u64);
    sdr.set_center(Hz(target.get() - offset as u64))?;
    println!("listening on {target} for {secs}s (gain {gain} dB, envelope {ENV_RATE:.0} Hz)\n");

    let mut mixer = Mixer::new(-offset, RF_RATE as f64);
    let mut dec = FirDecim::design(DECIM, 0.9, 80.0);
    let mut ook = OokDetector::new(ENV_RATE, PulseConfig::default());
    // Run both front ends over the same samples. Which one a device needs is
    // exactly the question this example exists to answer, and guessing wrong
    // produces silence rather than a clue.
    let mut fsk = FskDetector::new(ENV_RATE, FskConfig::default());

    let (mut shifted, mut iq, mut env, mut pkgs) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut total = 0usize;
    let mut fsk_total = 0usize;

    let mut rx = sdr.start_rx()?;
    let t0 = std::time::Instant::now();
    while t0.elapsed().as_secs_f64() < secs {
        let buf = rx.read()?;
        shifted.clear();
        mixer.process(&buf.samples, &mut shifted);
        iq.clear();
        dec.process(&shifted, &mut iq);
        env.clear();
        env.extend(iq.iter().map(|c| c.norm()));
        pkgs.clear();
        ook.process(&env, &mut pkgs);
        let ook_here = pkgs.len();
        fsk.process(&iq, &mut pkgs);
        fsk_total += pkgs.len() - ook_here;

        for (n, p) in pkgs.iter().enumerate() {
            let how = if n < ook_here { "OOK" } else { "FSK" };
            total += 1;
            if total > 12 {
                continue;
            }
            println!(
                "--- package {total} [{how}]: {} pulses, {:.1} ms, SNR {:.1} dB",
                p.pulses.len(),
                p.duration_us() as f64 / 1000.0,
                p.snr_db
            );
            let mh = p.mark_histogram(40);
            let gh = p.gap_histogram(40);
            println!("  mark clusters: {:?}", &mh[..mh.len().min(6)]);
            println!("  gap  clusters: {:?}", &gh[..gh.len().min(6)]);
            let show: Vec<String> =
                p.pulses.iter().take(16).map(|x| format!("{}/{}", x.mark, x.gap)).collect();
            println!("  first pulses:  {}", show.join(" "));
        }
    }
    rx.stop();

    println!(
        "\n{total} packages in {secs}s ({fsk_total} FSK). \
              noise {:.4} signal {:.4} ({:.1} dB)",
        ook.noise_level(),
        ook.signal_level(),
        ook.snr_db()
    );
    if fsk_total > 0 {
        println!("last FSK tone separation: {:.0} Hz", fsk.separation_hz());
    }
    if total == 0 {
        println!(
            "Nothing seen on either front end. Check the frequency and the gain:\n\
                  an FSK burst would have shown up as a flat envelope with a\n\
                  measurable tone separation."
        );
    }
    Ok(())
}
