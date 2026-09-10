//! Wideband FM receiver, used as an end-to-end correctness proof.
//!
//! Scans the FM band with peak-hold to find the strongest station, tunes it,
//! demodulates, and checks for the 19 kHz stereo pilot. That pilot is the
//! useful part: every stereo FM broadcast carries a tone at exactly 19000 Hz,
//! so finding a sharp peak there proves the whole chain (tuning, mixing,
//! decimation, discriminator, and the sample-rate bookkeeping behind them) is
//! correct to within a few hertz. "It sounds like radio" proves much less.
//!
//! Usage: cargo run --release -p rtlsdr --example wfm -- [mhz] [secs]

use common::device::{Device, GainMode};
use common::{Hz, Sps, C32};
use dsp::{Deemphasis, FirDecim, FmDemod, Mixer};
use std::f64::consts::TAU;

const RF_RATE: u64 = 2_400_000;
/// 2.4 MS/s / 8 = 300 kHz, comfortably wider than WFM's ~200 kHz occupancy.
const IF_DECIM: usize = 8;
const IF_RATE: f64 = RF_RATE as f64 / IF_DECIM as f64;
/// 300 kHz / 6 = 50 kHz audio, keeping the 19 kHz pilot inside Nyquist.
const AUDIO_DECIM: usize = 6;

fn goertzel(x: &[f32], rate: f64, target: f64) -> f64 {
    let k = TAU * target / rate;
    let coeff = 2.0 * k.cos();
    let (mut s1, mut s2) = (0.0f64, 0.0f64);
    for &v in x {
        let s0 = v as f64 + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    (s1 * s1 + s2 * s2 - coeff * s1 * s2).sqrt() / x.len() as f64
}

fn write_wav(path: &str, rate: u32, samples: &[f32]) -> std::io::Result<()> {
    use std::io::Write;
    let n = samples.len() as u32;
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    let data_len = n * 2;
    f.write_all(b"RIFF")?;
    f.write_all(&(36 + data_len).to_le_bytes())?;
    f.write_all(b"WAVEfmt ")?;
    f.write_all(&16u32.to_le_bytes())?;
    f.write_all(&1u16.to_le_bytes())?;
    f.write_all(&1u16.to_le_bytes())?;
    f.write_all(&rate.to_le_bytes())?;
    f.write_all(&(rate * 2).to_le_bytes())?;
    f.write_all(&2u16.to_le_bytes())?;
    f.write_all(&16u16.to_le_bytes())?;
    f.write_all(b"data")?;
    f.write_all(&data_len.to_le_bytes())?;
    let peak = samples.iter().fold(1e-9f32, |a, b| a.max(b.abs()));
    for s in samples {
        let v = (s / peak * 30000.0) as i16;
        f.write_all(&v.to_le_bytes())?;
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    let want: Option<f64> = args.get(1).and_then(|s| s.parse().ok());
    let secs: f64 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(4.0);

    let d = rtlsdr::enumerate();
    let d = d.first().ok_or("no RTL-SDR")?;
    let mut sdr = rtlsdr::RtlSdr::open(d.index)?;
    sdr.set_rate(Sps(RF_RATE))?;
    sdr.set_gain("tuner", GainMode::Manual(49.6))?;

    let station = match want {
        Some(m) => Hz((m * 1e6) as u64),
        None => scan_fm(&mut sdr)?,
    };

    // Tune 400 kHz low so the station does not sit on the RTL2832U's DC spur,
    // then mix it down digitally. Tuning a station to exactly 0 Hz puts the
    // strongest part of the signal on top of the worst artefact the hardware
    // has.
    let offset = 400_000.0f64;
    sdr.set_center(Hz(station.get() - offset as u64))?;
    println!("\nreceiving {station} (tuner at {}, {offset:.0} Hz digital offset)", sdr.center());

    let mut mixer = Mixer::new(-offset, RF_RATE as f64);
    let mut if_dec = FirDecim::design(IF_DECIM, 0.9, 80.0);
    let mut demod = FmDemod::new(IF_RATE, 75_000.0);
    // Cut at 15 kHz, not at the audio Nyquist. A wider filter passes the
    // 19 kHz stereo pilot straight into the mono output, where it sits ~43 dB
    // up, eats headroom and aliases the moment anything resamples it. The
    // broadcast mono baseband ends at 15 kHz by definition, so anything above
    // that is pilot and subcarrier, never programme audio.
    const AUDIO_RATE: f64 = IF_RATE / AUDIO_DECIM as f64;
    let mut audio_dec = FirDecim::design(AUDIO_DECIM, 15_000.0 / (AUDIO_RATE / 2.0), 80.0);
    let mut deemph = Deemphasis::eu(IF_RATE / AUDIO_DECIM as f64);

    let (mut shifted, mut iq_if, mut disc) = (Vec::new(), Vec::new(), Vec::new());
    let mut all_disc: Vec<f32> = Vec::new();
    let mut audio: Vec<f32> = Vec::new();

    let mut rx = sdr.start_rx()?;
    let t0 = std::time::Instant::now();
    while t0.elapsed().as_secs_f64() < secs {
        let buf = rx.read()?;
        shifted.clear();
        mixer.process(&buf.samples, &mut shifted);
        iq_if.clear();
        if_dec.process(&shifted, &mut iq_if);
        disc.clear();
        demod.process(&iq_if, &mut disc);
        all_disc.extend_from_slice(&disc);

        // Audio path decimates the real discriminator output. Reuse the
        // complex decimator by treating the real signal as I with Q=0.
        let cplx: Vec<C32> = disc.iter().map(|v| C32::new(*v, 0.0)).collect();
        let mut a = Vec::new();
        audio_dec.process(&cplx, &mut a);
        let mut a: Vec<f32> = a.iter().map(|c| c.re).collect();
        deemph.process(&mut a);
        audio.extend_from_slice(&a);
    }
    rx.stop();

    let audio_rate = IF_RATE / AUDIO_DECIM as f64;
    println!("discriminator: {} samples @ {IF_RATE:.0} Hz", all_disc.len());
    println!("audio:         {} samples @ {audio_rate:.0} Hz", audio.len());

    // The proof. Compare 19 kHz against nearby frequencies that carry no
    // pilot; a real pilot stands well clear of its neighbours.
    let n = all_disc.len().min(600_000);
    let x = &all_disc[..n];
    let pilot = goertzel(x, IF_RATE, 19_000.0);
    let refs: Vec<f64> =
        [15_500.0, 17_000.0, 21_000.0, 23_000.0].iter().map(|f| goertzel(x, IF_RATE, *f)).collect();
    let noise = refs.iter().sum::<f64>() / refs.len() as f64;
    let snr = 20.0 * (pilot / noise.max(1e-30)).log10();

    println!("\n19 kHz pilot: {:.2e}   neighbours: {:.2e}   ratio {snr:+.1} dB", pilot, noise);
    if snr > 10.0 {
        println!("PILOT DETECTED. WFM demodulation chain verified end to end.");
        // Sweep for the exact peak; it should land within a few Hz of 19000.
        const STEP: f64 = 0.25;
        let mut best = (0.0f64, 0.0f64);
        let mut f = 18_950.0;
        while f <= 19_050.0 {
            let m = goertzel(x, IF_RATE, f);
            if m > best.1 {
                best = (f, m);
            }
            f += STEP;
        }
        let err = best.0 - 19_000.0;
        // Only quote a ppm figure when the error exceeds the sweep resolution.
        // Converting a one-step error into ppm gives an alarming 53 ppm from
        // what is really just the search grid, which would send someone
        // chasing a crystal problem that does not exist.
        if err.abs() <= STEP * 1.5 {
            println!("peak at {:.2} Hz (within the {STEP} Hz search resolution of 19000)", best.0);
        } else {
            println!(
                "peak at {:.2} Hz (error {err:+.2} Hz -> {:+.2} ppm tuning offset)",
                best.0,
                err / 19_000.0 * 1e6
            );
        }
    } else {
        println!("No pilot. Either a mono station, or too weak.");
    }

    write_wav("/tmp/wfm.wav", audio_rate as u32, &audio)?;
    println!("\nwrote /tmp/wfm.wav ({:.1}s)", audio.len() as f64 / audio_rate);
    Ok(())
}

/// Peak-hold sweep across the FM band.
fn scan_fm(sdr: &mut rtlsdr::RtlSdr) -> Result<Hz, Box<dyn std::error::Error>> {
    use dsp::Channelizer;
    const CH: usize = 64;
    println!("scanning 87.5-108 MHz with peak hold...");

    let mut best = (Hz(0), f32::NEG_INFINITY);
    let mut found: Vec<(f64, f32)> = Vec::new();
    let mut f = 88_200_000u64;
    while f < 108_000_000 {
        sdr.set_center(Hz(f))?;
        let mut ch = Channelizer::new(CH, 12, 90.0);
        let mut peak = vec![0.0f32; CH];
        let mut rx = sdr.start_rx()?;
        let t = std::time::Instant::now();
        while t.elapsed().as_millis() < 250 {
            let b = rx.read()?;
            ch.process(&b.samples, |fr| {
                for (p, s) in peak.iter_mut().zip(fr.samples) {
                    *p = p.max(s.norm_sqr());
                }
            });
        }
        drop(rx);

        let mut sorted = peak.clone();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted[CH / 2];
        for (m, p) in peak.iter().enumerate() {
            // Skip the DC channel, which is the hardware spur, not a station.
            if m == 0 {
                continue;
            }
            let snr = 10.0 * (p / floor).log10();
            let freq = f as f64 + ch.channel_offset_hz(m, RF_RATE as f64);
            if snr > best.1 {
                best = (Hz(freq as u64), snr);
            }
            if snr > 12.0 {
                found.push((freq, snr));
            }
        }
        f += 2_000_000;
    }

    found.sort_by(|a, b| b.1.total_cmp(&a.1));
    println!("strongest channels found:");
    for (freq, snr) in found.iter().take(8) {
        println!("  {:8.3} MHz  {:+6.1} dB", freq / 1e6, snr);
    }
    // Round to the nearest 100 kHz: broadcast stations sit on that grid, and
    // the peak channel is only accurate to half a channel width.
    let snapped = Hz(((best.0.get() as f64 / 100_000.0).round() * 100_000.0) as u64);
    println!("picking {snapped} (peak {:+.1} dB)", best.1);
    Ok(snapped)
}
