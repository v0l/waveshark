//! The pass and block filters, run as graph stages rather than as arithmetic.

use common::{C32, Hz};
use nodes::{AutoNotchNode, FirFilterNode, IirFilterNode};
use pipeline::{Graph, StreamSpec};

fn tone(rate: f64, hz: f64, n: usize) -> Vec<C32> {
    (0..n)
        .map(|i| {
            let t = i as f64 / rate;
            let p = std::f64::consts::TAU * hz * t;
            C32::new(p.cos() as f32, p.sin() as f32)
        })
        .collect()
}

fn level(x: &[C32]) -> f32 {
    // The back half only: the front of the buffer is the filter filling up.
    let tail = &x[x.len() / 2..];
    (tail.iter().map(|s| s.norm_sqr()).sum::<f32>() / tail.len() as f32).sqrt()
}

fn run(mut g: Graph, input: &[C32]) -> Vec<C32> {
    let buf = g.input_buf();
    buf.clear();
    buf.iq_mut().extend_from_slice(input);
    g.run().expect("the graph should run");
    g.output().as_iq().unwrap_or(&[]).to_vec()
}

/// A filter drawn into a chain has to do to the samples what its own design
/// says it does. Tested through the graph rather than against the taps
/// because that is where a node gets its rate, and a filter designed at the
/// wrong rate is the failure this catches.
#[test]
fn a_lowpass_stage_keeps_the_low_tone_and_loses_the_high_one() {
    let rate = 48_000.0;
    let spec = StreamSpec::iq(rate, Hz(0));
    let build = || {
        pipeline::chain(
            spec,
            vec![Box::new(FirFilterNode::new(dsp::filter::Response::Lowpass, 3_000.0, 0.0, 127))],
        )
        .expect("a filter is a chain of one")
    };
    let low = run(build(), &tone(rate, 500.0, 4096));
    let high = run(build(), &tone(rate, 12_000.0, 4096));
    assert!(level(&low) > 0.9, "the tone below the cutoff should survive");
    assert!(level(&high) < 0.05, "the one above it should not");
}

#[test]
fn a_biquad_notch_stage_removes_the_carrier_it_is_pointed_at() {
    let rate = 48_000.0;
    let spec = StreamSpec::iq(rate, Hz(0));
    let q = dsp::filter::Biquad::band_q(2_000.0, 200.0);
    let build = || {
        pipeline::chain(
            spec,
            vec![Box::new(IirFilterNode::new(dsp::filter::Response::Bandstop, 2_000.0, q))],
        )
        .expect("a filter is a chain of one")
    };
    let on = run(build(), &tone(rate, 2_000.0, 8192));
    let off = run(build(), &tone(rate, 8_000.0, 8192));
    assert!(level(&on) < 0.2, "the tone at the notch should go");
    assert!(level(&off) > 0.8, "and the one away from it stay");
}

fn run_real(mut g: Graph, input: &[f32]) -> Vec<f32> {
    let buf = g.input_buf();
    buf.clear();
    buf.real_mut().extend_from_slice(input);
    g.run().expect("the graph should run");
    g.output().as_real().unwrap_or(&[]).to_vec()
}

fn notch_chain(on: bool, channels: usize) -> Graph {
    let spec = pipeline::StreamSpec {
        kind: pipeline::PortKind::Real,
        rate: 48_000.0,
        ..Default::default()
    }
    .with_channels(channels);
    pipeline::chain(spec, vec![Box::new(AutoNotchNode::new(on))])
        .expect("a notch is a chain of one")
}

fn audio_tone(hz: f64, n: usize) -> Vec<f32> {
    (0..n).map(|i| 0.3 * (std::f64::consts::TAU * hz * i as f64 / 48_000.0).sin() as f32).collect()
}

fn back_half_rms(x: &[f32]) -> f32 {
    let tail = &x[x.len() / 2..];
    (tail.iter().map(|s| s * s).sum::<f32>() / tail.len() as f32).sqrt()
}

#[test]
fn an_auto_notch_stage_switched_on_takes_a_heterodyne_down_44_db_and_off_passes_it_untouched() {
    let input = audio_tone(1_234.0, 96_000);
    let on = run_real(notch_chain(true, 1), &input);
    let off = run_real(notch_chain(false, 1), &input);
    let depth = 20.0 * (back_half_rms(&on) / back_half_rms(&input)).log10();
    assert!((-47.0..=-40.0).contains(&depth), "notch depth {depth:.1} dB, measured -44.6");
    assert_eq!(off, input);
}

#[test]
fn an_auto_notch_stage_follows_each_stereo_channel_on_its_own() {
    let left = audio_tone(900.0, 96_000);
    let right = audio_tone(2_300.0, 96_000);
    let input: Vec<f32> = left.iter().zip(&right).flat_map(|(l, r)| [*l, *r]).collect();
    let out = run_real(notch_chain(true, 2), &input);
    let side = |c: usize| out.iter().skip(c).step_by(2).copied().collect::<Vec<f32>>();
    for (c, name) in [(0, "left"), (1, "right")] {
        let depth = 20.0 * (back_half_rms(&side(c)) / back_half_rms(&left)).log10();
        assert!((-47.0..=-40.0).contains(&depth), "{name} depth {depth:.1} dB, measured -44.6");
    }
}
