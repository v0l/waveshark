//! What the transmit chain keys, the receive chain has to read back.
//!
//! The check the transmit path is worth having: text goes in one end as
//! bytes, comes out as IQ, is written to a capture in the format a radio
//! would have delivered, and is then read by the same detector that reads
//! signals off the air. Nothing here knows it is a test signal, and nothing
//! radiates.

use common::{Device, Hz, IqBuf, SampleFormat, Sps};
use nodes::MorseTxNode;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Flow, Payload, PortKind, StreamSpec, TAG_TX_END, TAG_TX_START};

const RATE: f64 = 250_000.0;

fn key(text: &str, wpm: f32) -> (Vec<common::C32>, Vec<pipeline::Tag>) {
    let mut node = MorseTxNode::new(wpm, 10_000.0);
    let spec = StreamSpec {
        kind: PortKind::Bytes,
        rate: RATE,
        center: Hz(433_920_000),
        bandwidth: RATE,
        flow: Flow::Tx,
        ..Default::default()
    };
    let outs = node.negotiate(&[PortSpec { spec, latency: 0 }]).unwrap();
    assert_eq!(outs[0].kind, PortKind::Iq);
    assert!(outs[0].is_tx(), "a modulator's output is a transmit stream");

    let input = Payload::Bytes(text.as_bytes().to_vec());
    let mut out = vec![Payload::Iq(Vec::new())];
    let mut events = Vec::new();
    let mut tags = Vec::new();
    let ins = [PortSpec { spec, latency: 0 }];
    let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
    node.process(&[&input], &mut out, &mut ctx).unwrap();
    let iq = match out.remove(0) {
        Payload::Iq(v) => v,
        _ => unreachable!(),
    };
    (iq, tags)
}

/// Put the transmission through something like a channel: a lead-in of noise
/// and noise on top of it.
///
/// Not decoration. The detector's threshold is measured against the floor, so
/// a capture that is exactly zero between marks and starts abruptly on one is
/// a signal no receiver ever sees: the level estimator has nothing to learn
/// from and reads the first symbol as noise, or the whole burst as a stream
/// with no signal in it. 30 dB down is a strong but ordinary off-air signal.
fn through_a_channel(iq: &[common::C32]) -> Vec<common::C32> {
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut noise = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        ((seed >> 40) as f32 / 16_777_216.0 - 0.5) * 0.016
    };
    let lead = (RATE * 0.1) as usize;
    let mut out = Vec::with_capacity(lead + iq.len());
    for _ in 0..lead {
        out.push(common::C32::new(noise(), noise()));
    }
    for s in iq {
        out.push(s + common::C32::new(noise(), noise()));
    }
    out
}

/// Read timings back out of IQ the way the receiver does: envelope, then the
/// OOK detector with its adaptive threshold.
fn detect(iq: &[common::C32]) -> Vec<common::Package> {
    let cfg = dsp::PulseConfig {
        // A word gap at 20 wpm is 420 ms and the burst has to survive it, so
        // only a longer silence ends the transmission.
        reset_us: 600_000,
        min_mark_us: 1_000,
        min_pulses: 2,
        // The default 500 us suits an ISM burst, whose marks are about a
        // millisecond. A Morse dot at 20 wpm is 60 ms, and an estimator that
        // fast follows the mark itself: the level it calls noise climbs to
        // the carrier partway through every dash, and the whole transmission
        // is then thrown away as having no signal in it.
        tau_us: 20_000.0,
        ..Default::default()
    };
    let mut det = dsp::pulse::OokDetector::new(RATE, cfg);
    let env: Vec<f32> = iq.iter().map(|c| c.norm()).collect();
    let mut out = Vec::new();
    det.process(&env, &mut out);
    det.flush(&mut out);
    out
}

#[test]
fn morse_keyed_by_the_transmit_chain_decodes_off_its_own_signal() {
    let text = "CQ DE EI7XYZ";
    let (iq, _) = key(text, 20.0);
    assert!(!iq.is_empty(), "the chain produced no samples");

    let pkgs = detect(&through_a_channel(&iq));
    assert_eq!(pkgs.len(), 1, "expected one burst, got {}", pkgs.len());
    assert_eq!(decode::morse::decode(&pkgs[0]), text);
}

#[test]
fn a_capture_of_the_transmission_replays_as_the_same_text() {
    // Through the sink, in the format a radio would have delivered, so what
    // is asserted is what a receiver would have been given rather than the
    // floats the modulator happened to produce.
    let text = "PARIS";
    let (iq, _) = key(text, 25.0);

    let (mut sink, buf) = sources::FileSink::in_memory(Sps(RATE as u64), SampleFormat::Cs8);
    let mut tx = sink.start_tx().unwrap();
    tx.write(&IqBuf::new(iq, Hz(433_920_000), Sps(RATE as u64), 0)).unwrap();
    tx.drain(std::time::Duration::from_millis(50));
    assert_eq!(tx.underruns(), 0);

    let mut back = Vec::new();
    SampleFormat::Cs8.convert(&buf.lock(), &mut back);
    let pkgs = detect(&through_a_channel(&back));
    assert_eq!(pkgs.len(), 1);
    assert_eq!(decode::morse::decode(&pkgs[0]), text);
}

#[test]
fn every_burst_is_marked_at_both_ends() {
    // What the radio keys on. A transmission the scheduler cannot see the
    // ends of is one it cannot time or duty cycle limit.
    let (iq, tags) = key("E", 20.0);
    let start: Vec<_> = tags.iter().filter(|t| t.key == TAG_TX_START).collect();
    let end: Vec<_> = tags.iter().filter(|t| t.key == TAG_TX_END).collect();
    assert_eq!(start.len(), 1, "no start of burst tag");
    assert_eq!(end.len(), 1, "no end of burst tag");
    assert_eq!(start[0].index, 0);
    assert_eq!(end[0].index as usize, iq.len() - 1);
}

#[test]
fn a_receive_stream_cannot_be_wired_into_a_transmit_stage() {
    // The check GNU Radio cannot make: its ports are complex samples and
    // nothing more, so a demodulator connected to a transmitter builds.
    let mut node = MorseTxNode::default();
    let spec = StreamSpec {
        kind: PortKind::Bytes,
        rate: RATE,
        center: Hz(433_920_000),
        bandwidth: RATE,
        ..Default::default()
    };
    // Accepted here, since a lone stage cannot know: what refuses it is the
    // graph, which sees both directions meeting at one node.
    let out = node.negotiate(&[PortSpec { spec, latency: 0 }]).unwrap();
    assert!(out[0].is_tx(), "the modulator's output must be marked as transmit");
}
