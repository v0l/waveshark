//! Live WFM/NFM receiver with audio playback.
//!
//! Usage: listen <mhz> [wfm|nfm] [gain]

use audio::AudioPlayer;
use common::device::{Device, GainMode};
use common::{Hz, Sps, C32};
use dsp::{Deemphasis, FirDecim, FmDemod, HighBlend, Mixer, NoiseMeter};

/// 2.304 MS/s / 8 / 6 = exactly 48 kHz, so no resampling is needed anywhere.
const RF_RATE: u64 = 2_304_000;
const IF_DECIM: usize = 8;
const AUDIO_DECIM: usize = 6;
const IF_RATE: f64 = RF_RATE as f64 / IF_DECIM as f64;
const AUDIO_RATE: f64 = IF_RATE / AUDIO_DECIM as f64;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<String> = std::env::args().collect();
    let mhz: f64 = a.get(1).and_then(|s| s.parse().ok()).unwrap_or(95.8);
    let narrow = a.get(2).map(|s| s == "nfm").unwrap_or(false);
    let gain: f32 = a.get(3).and_then(|s| s.parse().ok()).unwrap_or(49.6);

    println!("output devices: {:?}", AudioPlayer::devices());
    let (player, mut sink) = AudioPlayer::open(AUDIO_RATE as u32)?;
    println!(
        "playing on {} at {} Hz, {} ch",
        player.device_name(),
        player.rate(),
        player.channels()
    );
    // write_adaptive resamples, so a mismatched device rate is fine.

    let d = rtlsdr::enumerate();
    let d = d.first().ok_or("no RTL-SDR")?;
    let mut sdr = rtlsdr::RtlSdr::open(d.index)?;
    sdr.set_rate(Sps(RF_RATE))?;
    sdr.set_gain("tuner", GainMode::Manual(gain))?;

    // Tune off-centre so the station avoids the RTL2832U DC spur.
    let offset = 400_000.0f64;
    sdr.set_center(Hz((mhz * 1e6) as u64 - offset as u64))?;

    let deviation = if narrow { 5_000.0 } else { 75_000.0 };
    let mut mixer = Mixer::new(-offset, RF_RATE as f64);
    let mut if_dec = FirDecim::design(IF_DECIM, 0.9, 80.0);
    let mut demod = FmDemod::new(IF_RATE, deviation);
    // 15 kHz for WFM (broadcast mono baseband ends there, and a wider filter
    // passes the 19 kHz pilot into the audio); 3.5 kHz for voice.
    let cutoff = if narrow { 3_500.0 } else { 15_000.0 };
    let mut audio_dec = FirDecim::design(AUDIO_DECIM, cutoff / (AUDIO_RATE / 2.0), 80.0);
    let mut deemph = Deemphasis::eu(AUDIO_RATE);
    let mut noise = NoiseMeter::new(IF_RATE);
    let mut blend = HighBlend::new(AUDIO_RATE);
    let blend_on = !a.iter().any(|x| x == "--no-blend");

    println!(
        "\n{:.4} MHz {}  RF {} -> IF {IF_RATE} -> audio {AUDIO_RATE}\nCtrl-C to stop",
        mhz,
        if narrow { "NFM" } else { "WFM" },
        RF_RATE
    );

    let (mut sh, mut iq, mut disc, mut cx, mut au) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let mut rx = sdr.start_rx()?;
    let start = std::time::Instant::now();
    let mut last = start;

    loop {
        let buf = rx.read()?;
        sh.clear();
        mixer.process(&buf.samples, &mut sh);
        iq.clear();
        if_dec.process(&sh, &mut iq);
        disc.clear();
        demod.process(&iq, &mut disc);

        cx.clear();
        cx.extend(disc.iter().map(|v| C32::new(*v, 0.0)));
        au.clear();
        audio_dec.process(&cx, &mut au);
        let n = noise.process(&disc);
        let mut pcm: Vec<f32> = au.iter().map(|c| c.re * 0.5).collect();
        deemph.process(&mut pcm);
        if blend_on {
            blend.process(n, &mut pcm);
        }
        sink.write_adaptive(&pcm, AUDIO_RATE);
        if last.elapsed().as_secs() >= 2 {
            last = std::time::Instant::now();
            let s = sink.stats();
            let level = (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len().max(1) as f32).sqrt();
            println!(
                "{:5.0}s  level {:5.1} dB  rf dropped {}  audio dropped {} underruns {}",
                start.elapsed().as_secs_f64(),
                20.0 * level.max(1e-9).log10(),
                rx.dropped(),
                s.dropped.load(std::sync::atomic::Ordering::Relaxed),
                s.underruns.load(std::sync::atomic::Ordering::Relaxed),
            );
            println!(
                "       backlog {} samples  drift {:+.1} ppm  noise {:.4}  treble cut {:.0} Hz",
                sink.backlog(),
                sink.drift_ppm(),
                noise.level(),
                blend.cutoff()
            );
        }
    }
}
