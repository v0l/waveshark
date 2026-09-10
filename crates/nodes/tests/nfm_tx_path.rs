//! The whole transmit path, end to end, with a file where the antenna goes.
//!
//! A graph of tone, modulator and radio sink is built the way the receiver
//! builds its own, run block by block, and what the "radio" received is then
//! demodulated by the receive side. Nothing here is a special case for
//! testing: the sink is the same `TxSinkNode` a HackRF gets, driven through
//! the same `TxStream` trait, and the capture it writes is in the format a
//! receiver would have delivered.

use common::{Device, Hz, SampleFormat, Sps};
use nodes::{FmModNode, ToneNode, TxSinkNode, NBFM_DEVIATION_HZ};
use pipeline::port::{Flow, PortKind, StreamSpec};
use pipeline::{chain, Graph};

/// Audio and transmit rate. One rate throughout, because the modulator does
/// not resample: an NFM channel is 12.5 kHz wide and 48 kS/s covers it with
/// room for the deviation either side.
const RATE: f64 = 48_000.0;
const TONE_HZ: f64 = 1_000.0;
const BLOCK: usize = 4_800;

fn tx_graph(stream: Box<dyn common::TxStream>) -> Graph {
    let input = StreamSpec {
        kind: PortKind::Real,
        rate: RATE,
        center: Hz(433_920_000),
        // The audio's own occupied width, which is what `fm_mod` checks its
        // deviation against.
        bandwidth: 6_000.0,
        flow: Flow::Tx,
        ..Default::default()
    };
    chain(
        input,
        vec![
            Box::new(ToneNode::new(TONE_HZ, 0.8)),
            Box::new(FmModNode::narrowband(0.0)),
            Box::new(TxSinkNode::new(stream)),
        ],
    )
    .expect("the transmit chain builds")
}

/// Run the graph for `blocks` blocks of silence, which the tone node fills.
fn transmit(g: &mut Graph, blocks: usize) {
    for _ in 0..blocks {
        let buf = g.input_buf();
        buf.clear();
        buf.real_mut().resize(BLOCK, 0.0);
        g.run().expect("a block transmits");
    }
}

#[test]
fn a_tone_transmitted_as_nfm_comes_back_off_the_capture() {
    let (mut sink, captured) = sources::FileSink::in_memory(Sps(RATE as u64), SampleFormat::Cs8);
    let mut g = tx_graph(sink.start_tx().unwrap());
    transmit(&mut g, 10);

    // What the radio was handed, as a receiver would have delivered it.
    let mut iq = Vec::new();
    SampleFormat::Cs8.convert(&captured.lock(), &mut iq);
    assert_eq!(iq.len(), 10 * BLOCK, "the sink did not get every block");

    let mut demod = dsp::FmDemod::new(RATE, NBFM_DEVIATION_HZ);
    let mut audio = Vec::new();
    demod.process(&iq, &mut audio);

    // Frequency of the recovered audio, by counting zero crossings over a
    // stretch clear of the first sample, which has nothing to differentiate
    // against. Counting crossings rather than fitting a sine keeps this a
    // measurement of what came back rather than of what was expected.
    let seg = &audio[1_000..];
    let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
    let hz = crossings as f64 * RATE / seg.len() as f64;
    assert!(
        (hz - TONE_HZ).abs() < 5.0,
        "recovered a {hz:.0} Hz tone, sent {TONE_HZ}"
    );

    // And at the level it was sent at: 0.8 of full scale into a 2.5 kHz
    // deviation is 2 kHz, which the discriminator scales back to 0.8.
    //
    // Measured as RMS rather than peak. The capture is 8 bit and the carrier
    // sits at a quarter of full scale, so each sample's phase is quantised
    // coarsely enough that the discriminator throws individual samples 7%
    // past the peak, while the tone's power is unaffected.
    let rms = (seg.iter().map(|v| (v * v) as f64).sum::<f64>() / seg.len() as f64).sqrt();
    let want = 0.8 / 2f64.sqrt();
    assert!(
        (rms - want).abs() < 0.02,
        "recovered {rms:.3} rms, sent {want:.3}"
    );
}

#[test]
fn the_chain_reports_what_it_handed_the_radio() {
    let (mut sink, _c) = sources::FileSink::in_memory(Sps(RATE as u64), SampleFormat::Cs8);
    let mut g = tx_graph(sink.start_tx().unwrap());
    transmit(&mut g, 5);

    let id = g.order().last().map(|(id, _)| id).unwrap();
    let node = g.node(id).unwrap();
    let tx = node.as_any().downcast_ref::<TxSinkNode>();
    let tx = tx.expect("the sink can be read back off the graph");
    assert_eq!(tx.written(), 5 * BLOCK as u64);
    assert_eq!(tx.underruns(), 0, "a file is never late");
    assert_eq!(tx.failed_blocks(), 0);
}

#[test]
fn a_receive_stream_cannot_be_handed_to_the_radio() {
    // The whole point of marking direction on the spec. Built by hand rather
    // than through the modulator, which is the only way to get here.
    let (mut sink, _c) = sources::FileSink::in_memory(Sps(RATE as u64), SampleFormat::Cs8);
    let input = StreamSpec::iq(RATE, Hz(433_920_000));
    assert!(!input.is_tx());
    let err = chain(
        input,
        vec![Box::new(TxSinkNode::new(sink.start_tx().unwrap()))],
    )
    .unwrap_err()
    .to_string();
    assert!(err.contains("receive stream"), "unhelpful: {err}");
}

#[test]
fn the_transmission_is_the_shape_the_graph_says_it_is() {
    // The chain view has to be able to draw this: three stages, the last a
    // sink, and the stream between the modulator and the radio marked as
    // going out rather than coming in.
    let (mut sink, _c) = sources::FileSink::in_memory(Sps(RATE as u64), SampleFormat::Cs8);
    let g = tx_graph(sink.start_tx().unwrap());
    let topo = g.topology();
    let names: Vec<&str> = topo.nodes.iter().map(|n| n.label.as_str()).collect();
    assert_eq!(names, ["tone", "fm_mod", "radio_tx"]);
    // Not a sink: what went to the antenna leaves the last stage as well, so
    // the receiver's own spectrum can be shown the over it cannot hear.
    assert!(!topo.nodes.last().unwrap().sink);

    let modulated = g.output_spec();
    assert_eq!(modulated.kind, PortKind::Iq);
    assert!(modulated.is_tx());
    // Carson: 2 x (2.5 kHz deviation + 3 kHz of audio).
    assert!(
        (modulated.bandwidth - 11_000.0).abs() < 1.0,
        "{}",
        modulated.bandwidth
    );
}

#[test]
fn a_block_the_radio_refuses_does_not_stop_the_graph() {
    let (mut sink, _c) = sources::FileSink::in_memory(Sps(RATE as u64), SampleFormat::Cs8);
    let stream = sink.start_tx().unwrap();
    let mut g = tx_graph(stream);
    transmit(&mut g, 2);

    // Stop the radio underneath the running graph, which is what a HackRF
    // being unplugged mid-transmission looks like.
    let id = g.order().last().map(|(id, _)| id).unwrap();
    if let Some(n) = g.node_mut(id) {
        if let Some(tx) = n.as_any_mut().downcast_mut::<TxSinkNode>() {
            tx.finish(std::time::Duration::from_millis(10));
        }
    }
    transmit(&mut g, 2);
}

#[test]
fn what_went_to_the_radio_can_be_read_back_off_the_sink() {
    // The monitor: a half duplex radio hears nothing while it transmits, so
    // the receiver is shown the transmitter's own samples instead. What is
    // asserted here is that they are the samples that went out, not a copy
    // taken somewhere else that could drift from them.
    let (mut sink, captured) = sources::FileSink::in_memory(Sps(RATE as u64), SampleFormat::Cs8);
    let mut g = tx_graph(sink.start_tx().unwrap());
    transmit(&mut g, 3);

    let id = g.order().last().map(|(id, _)| id).unwrap();
    let monitor = g.buf(id.o()).and_then(|b| b.as_iq()).expect("the monitor port");
    assert_eq!(monitor.len(), BLOCK, "the monitor holds the last block");

    let mut went_out: Vec<common::C32> = Vec::new();
    SampleFormat::Cs8.convert(&captured.lock(), &mut went_out);
    let tail = &went_out[went_out.len() - BLOCK..];
    for (a, b) in monitor.iter().zip(tail) {
        // Cs8 quantises at 1/128, so this is exact to within the capture's
        // own resolution.
        assert!((a - b).norm() < 0.01, "{a} was monitored for {b}");
    }
}
