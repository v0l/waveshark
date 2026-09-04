//! Feed a .cu8 to the dmr front end directly, at the full rate and again
//! through the same cut the auto node would make.
//!     dmr_probe <file.cu8> <rate> <centre_hz> <channel_hz>
use common::C32;
use nodes::{build_chain, registry, NodeSpec};
use pipeline::StreamSpec;

fn run(label: &str, iq: &[C32], rate: f64, center: f64, channel: f64) {
    let spec = StreamSpec::iq(rate, common::Hz(center as u64));
    let mut g = build_chain(spec, &[NodeSpec::new("dmr").f("channel_hz", channel)], &registry()).unwrap();
    let (mut frames, mut voice) = (0usize, 0usize);
    let bs: usize = std::env::var("BLOCK").ok().and_then(|v| v.parse().ok()).unwrap_or(16_384);
    for b in iq.chunks(bs) {
        g.feed_iq(b).unwrap();
        if let Some(p) = g.output().as_packets() {
            for p in p {
                frames += 1;
                voice += p.audio.as_ref().map(|a| a.pcm.len()).unwrap_or(0);
                if let common::PacketBody::Frame(b) = &p.body {
                    if let Some(d) = nodes::dmr_nodes::dmr_decoded(b, common::Hz(p.center_hz)) {
                        eprintln!("  {label}: {:?}", d.detail);
                    }
                }
            }
        }
    }
    eprintln!("{label}: rate {rate} -> {frames} rows, {voice} voice samples");
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let rate: f64 = a[2].parse().unwrap();
    let center: f64 = a[3].parse().unwrap();
    let channel: f64 = a[4].parse().unwrap();
    let bytes = std::fs::read(&a[1]).unwrap();
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new((c[0] as f32 - 127.5) / 127.5, (c[1] as f32 - 127.5) / 127.5))
        .collect();
    run("full", &iq, rate, center, channel);
    // The auto node's cut: mixed to the channel, decimated to the source rate.
    for (out_rate, cutoff) in [(25_000.0, 10_000.0), (36_571.0, 6_000.0), (36_571.0, 7_500.0), (36_571.0, 9_000.0)] {
        let factor = (rate / out_rate).round() as usize;
        let got = rate / factor as f64;
        let mut mixer = dsp::Mixer::new(center - channel, rate);
        let mut decim = dsp::FirDecim::design_hz(rate, factor, cutoff, 60.0);
        let mut mixed = Vec::new();
        mixer.process(&iq, &mut mixed);
        let mut cut = Vec::new();
        decim.process(&mixed, &mut cut);
        run(&format!("cut {got:.0} +-{cutoff:.0}"), &cut, got, channel, channel);
    }
}
