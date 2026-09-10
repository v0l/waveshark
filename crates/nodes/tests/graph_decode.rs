//! Decode the real capture through a graph assembled at runtime from names
//! and settings, with no compile-time knowledge of the chain.
//!
//! `crates/decode/tests/fineoffset_capture.rs` already proves the DSP and the
//! protocol are right by calling them directly. This proves the *graph* adds
//! nothing and loses nothing: same recording, same answer, but routed through
//! the registry the way a user-configured chain would be.

use common::Hz;
use nodes::{build_chain, registry, NodeSpec};
use pipeline::event::Event;
use pipeline::{ParamValue, StreamSpec};
use sources::FileSource;

const FIXTURE: &str = "fineoffset_wh1080_433.92M_250k.cu8";

fn fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(FIXTURE);
    if !p.exists() {
        return None;
    }
    FileSource::open(&p).ok()?.read_all().ok()
}

macro_rules! need_fixture {
    ($e:expr) => {
        match $e {
            Some(v) => v,
            None => {
                eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh");
                return;
            }
        }
    };
}

/// The chain that decodes this capture. The recording is already at baseband
/// and at 250 kS/s, so no shift and no decimation are needed.
fn chain_specs() -> Vec<NodeSpec> {
    // Decimate before the envelope: the signal is ~4 kHz wide in a 250 kHz
    // capture, so detecting on the raw wideband envelope means competing with
    // 60x more noise power than necessary.
    vec![
        NodeSpec::new("decimate").i("factor", 8),
        NodeSpec::new("envelope"),
        NodeSpec::new("pulse_detect").f("reset_us", 10_000.0).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ]
}

/// The unmatchable chain, which produces bursts no protocol claims.
fn specs_with_unknown() -> Vec<NodeSpec> {
    vec![
        NodeSpec::new("decimate").i("factor", 8),
        NodeSpec::new("envelope"),
        NodeSpec::new("real_decimate").i("factor", 20),
        NodeSpec::new("pulse_detect").f("reset_us", 10_000.0).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ]
}

/// One block through a graph, as the events it produced. Which node raised
/// each is asserted where it is the point, and read off the graph there.
fn events_of(g: &mut pipeline::Graph, iq: &[common::C32]) -> Vec<Event> {
    g.feed_iq(iq).expect("run graph").iter().map(|e| e.event.clone()).collect()
}

fn decodes_from(graph_events: &[Event]) -> Vec<String> {
    graph_events
        .iter()
        .filter_map(|e| match e {
            Event::Decoded(d) => d.text.clone(),
            _ => None,
        })
        .collect()
}

#[test]
fn a_runtime_assembled_graph_decodes_the_real_capture() {
    let buf = need_fixture!(fixture());
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    let mut g = build_chain(spec, &chain_specs(), &registry()).expect("build chain");

    let events = events_of(&mut g, &buf.samples);
    let decodes = decodes_from(&events);

    assert_eq!(decodes.len(), 1, "expected one decode, got {decodes:?}");
    let text = &decodes[0];
    // Same ground truth as the direct test: rtl_433 25.02 on this recording.
    assert!(text.contains("Fineoffset-WHx080"), "{text}");
    assert!(text.contains("station_id=196"), "{text}");
    assert!(text.contains("temperature_c=16.2"), "{text}");
    assert!(text.contains("humidity_pct=89"), "{text}");
    assert!(text.contains("rain_total_mm=84.3"), "{text}");
    assert!(text.contains("[CRC ok]"), "{text}");
}

#[test]
fn the_graph_negotiates_rates_and_kinds_correctly() {
    let buf = need_fixture!(fixture());
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    let g = build_chain(spec, &chain_specs(), &registry()).unwrap();

    let names: Vec<&str> = g.order().map(|(_, n)| n).collect();
    assert_eq!(names, vec!["decimate", "envelope", "pulse_detect", "protocol_decode"]);
    assert_eq!(g.output_spec().kind, pipeline::PortKind::Bytes);
}

#[test]
fn a_misordered_chain_fails_at_build_with_an_actionable_message() {
    // Pulse detection before the envelope: the classic mistake.
    let specs = vec![NodeSpec::new("pulse_detect"), NodeSpec::new("envelope")];
    let spec = StreamSpec::iq(250_000.0, Hz::mhz(433));
    let err = build_chain(spec, &specs, &registry()).unwrap_err().to_string();
    assert!(err.contains("pulse_detect"), "{err}");
    assert!(err.contains("envelope"), "error should say how to fix it: {err}");
}

#[test]
fn an_unknown_node_type_lists_what_is_available() {
    let specs = vec![NodeSpec::new("magic_decoder")];
    let spec = StreamSpec::iq(250_000.0, Hz::mhz(433));
    let err = build_chain(spec, &specs, &registry()).unwrap_err().to_string();
    assert!(err.contains("magic_decoder"), "{err}");
    assert!(err.contains("pulse_detect"), "should list known types: {err}");
}

#[test]
fn retuning_a_parameter_at_runtime_changes_behaviour() {
    // The point of the whole exercise: an ambiguous signal is handled by
    // reconfiguring the chain, not recompiling. A reset gap far below Fine
    // Offset's ~1 ms inter-symbol gaps must fragment the packet and stop it
    // decoding; restoring it must bring the decode back.
    let buf = need_fixture!(fixture());
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);

    let bad = vec![
        NodeSpec::new("decimate").i("factor", 8),
        NodeSpec::new("envelope"),
        NodeSpec::new("pulse_detect").f("reset_us", 600.0).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ];
    let mut g = build_chain(spec, &bad, &registry()).unwrap();
    let events = events_of(&mut g, &buf.samples);
    assert!(
        decodes_from(&events).is_empty(),
        "too short a reset gap should have fragmented the packet"
    );

    // Now fix it in place, without rebuilding the graph.
    let id = pipeline::NodeId(2);
    g.node_mut(id)
        .unwrap()
        .set_param("reset_us", ParamValue::Float(10_000.0))
        .expect("set reset_us");
    g.negotiate().expect("renegotiate");
    g.reset();

    let events = events_of(&mut g, &buf.samples);
    assert_eq!(decodes_from(&events).len(), 1, "restoring the reset gap should decode again");
}

#[test]
fn an_unrecognised_burst_is_reported_as_a_packet_of_its_own() {
    // An unknown device is what a scanner should be best at, so a burst no
    // protocol claims still becomes a packet: the coding inferred from its own
    // timings, and the bits that fall out under that reading. Silence would
    // make the receiver useless for exactly this case.
    let buf = need_fixture!(fixture());
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    let specs = vec![
        NodeSpec::new("decimate").i("factor", 8),
        NodeSpec::new("envelope"),
        // Decimating the envelope by 20 scales every pulse width by 20 and
        // makes the frame unmatchable.
        NodeSpec::new("real_decimate").i("factor", 20),
        NodeSpec::new("pulse_detect").f("reset_us", 10_000.0).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ];
    let mut g = build_chain(spec, &specs, &registry()).unwrap();
    let events = events_of(&mut g, &buf.samples);

    let packets: Vec<&pipeline::event::Decoded> = events
        .iter()
        .filter_map(|e| match e {
            Event::Decoded(d) => Some(d),
            _ => None,
        })
        .collect();
    assert!(!packets.is_empty(), "an unknown burst must be reported: {events:?}");
    for d in &packets {
        assert_eq!(d.protocol, "unknown", "nothing should have matched: {d:?}");
        assert_eq!(
            d.modulation,
            Some(common::Modulation::Ook),
            "the modulation belongs in the report"
        );
        let detail = d.detail.as_deref().unwrap_or_default();
        // Enough to start reverse engineering from: a coding with its
        // timings, and bits to compare between receptions.
        assert!(detail.contains("us"), "no timings in {detail:?}");
        assert!(detail.contains("pulses"), "no pulse count in {detail:?}");
    }

    // How strongly it was received is on the burst the decode was made from
    // and not copied onto the conclusion, so this is where a consumer reads
    // it: the detector's own output, node 3 of the chain above.
    let bursts =
        g.buf(pipeline::NodeId(3).o()).and_then(|p| p.as_pulses()).expect("the detector's bursts");
    assert_eq!(
        bursts.len(),
        packets.len(),
        "every burst the detector found should have been reported"
    );
    for pkg in bursts {
        assert!(pkg.snr_db > 0.0, "no SNR on {pkg:?}");
        assert!(pkg.rssi_dbfs.is_finite(), "no level on {pkg:?}");
    }
}

#[test]
fn turning_off_unknown_reporting_silences_them_without_touching_decodes() {
    let buf = need_fixture!(fixture());
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    // The same unmatchable chain as above, which is the only one that
    // produces unknowns to silence in the first place.
    let mut specs = specs_with_unknown();
    specs[4] = NodeSpec::new("protocol_decode").b("report_unknown", false);
    let mut g = build_chain(spec, &specs, &registry()).unwrap();
    let events = events_of(&mut g, &buf.samples);

    let unknown =
        events.iter().filter(|e| matches!(e, Event::Decoded(d) if d.protocol == "unknown")).count();
    assert_eq!(unknown, 0, "unknown reporting was turned off");

    // And with it on, the same chain does report them.
    let mut g = build_chain(spec, &specs_with_unknown(), &registry()).unwrap();
    let events = events_of(&mut g, &buf.samples);
    assert!(
        events.iter().any(|e| matches!(e, Event::Decoded(d) if d.protocol == "unknown")),
        "the same chain must report unknowns when asked to"
    );
}

#[test]
fn the_registry_describes_every_node_for_a_ui() {
    let r = registry();
    let names: Vec<&str> = r.list().map(|d| d.name).collect();
    for want in ["mixer", "decimate", "envelope", "fm_demod", "pulse_detect", "protocol_decode"] {
        assert!(names.contains(&want), "registry is missing {want}: {names:?}");
    }
    // Categories let a UI group the palette without hard-coding node names.
    assert!(r.by_category(pipeline::Category::Decode).count() >= 2);
    assert!(r.by_category(pipeline::Category::Filter).count() >= 3);
    for d in r.list() {
        assert!(!d.summary.is_empty(), "{} has no summary", d.name);
    }
}

#[test]
fn every_node_exposes_its_parameters() {
    let r = registry();
    let spec = StreamSpec::iq(250_000.0, Hz::mhz(433));
    for desc in r.list() {
        let g = build_chain(spec, &[NodeSpec::new(desc.name)], &r);
        // Some nodes reject an IQ input; only check the ones that accept it.
        let Ok(g) = g else { continue };
        let n = g.node(pipeline::NodeId(0)).unwrap();
        for p in n.params() {
            assert!(!p.name.is_empty(), "{} has an unnamed parameter", desc.name);
            assert!(
                !p.display_label().is_empty(),
                "{} parameter {} has no label",
                desc.name,
                p.name
            );
        }
    }
}

#[test]
fn a_mistuned_detector_says_what_it_discarded_and_which_knob_to_turn() {
    // The failure mode that matters most in practice. A reset gap below Fine
    // Offset's ~1 ms inter-symbol spacing fragments one packet into dozens of
    // short ones, all filtered out by min_pulses. Reporting zero events would
    // be indistinguishable from a dead antenna.
    let buf = need_fixture!(fixture());
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    let specs = vec![
        NodeSpec::new("decimate").i("factor", 8),
        NodeSpec::new("envelope"),
        NodeSpec::new("pulse_detect").f("reset_us", 600.0).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ];
    let mut g = build_chain(spec, &specs, &registry()).unwrap();
    let raised = g.feed_iq(&buf.samples).unwrap().to_vec();
    let events: Vec<Event> = raised.iter().map(|e| e.event.clone()).collect();

    assert!(decodes_from(&events).is_empty());
    let msg = raised
        .iter()
        .find_map(|e| match &e.event {
            Event::Warning { message } if g.label(e.node) == Some("pulse_detect") => {
                Some(message.clone())
            }
            _ => None,
        })
        .expect("a mistuned detector must not fail silently");
    assert!(msg.contains("discarded"), "{msg}");
    assert!(msg.contains("min_pulses") || msg.contains("reset_us"), "must name a knob: {msg}");
}

#[test]
fn the_ask_detector_decodes_the_real_capture_too() {
    // Fine Offset is deep OOK, so the shallow-ASK detector is not needed here.
    // That is exactly why it is worth checking: a detector meant as a drop-in
    // replacement must give the same answer on a signal the OOK path already
    // handles, or swapping it in trades one failure for another.
    let buf = need_fixture!(fixture());
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    let specs = vec![
        NodeSpec::new("decimate").i("factor", 8),
        NodeSpec::new("envelope"),
        NodeSpec::new("ask_detect").f("reset_us", 10_000.0).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ];
    let mut g = build_chain(spec, &specs, &registry()).expect("build chain");
    let events = events_of(&mut g, &buf.samples);
    let decodes = decodes_from(&events);

    assert_eq!(decodes.len(), 1, "expected one decode, got {decodes:?}");
    assert!(decodes[0].contains("station_id=196"), "{}", decodes[0]);
    assert!(decodes[0].contains("temperature_c=16.2"), "{}", decodes[0]);
    assert!(decodes[0].contains("[CRC ok]"), "{}", decodes[0]);
}

/// A burst no front end reads must still reach the log.
///
/// Before this it left a tag on a sample index and a count in a warning,
/// neither of which a packet list shows, so a chirp or a multi-carrier burst
/// was indistinguishable from an empty channel. That is the opposite of what
/// this receiver is for: the burst nobody decodes is the one worth seeing.
#[test]
fn an_unreadable_burst_is_still_reported() {
    use common::C32;
    use pipeline::event::Event;
    use pipeline::node::{Node, NodeCtx, PortSpec};
    use pipeline::port::{Payload, PortKind};

    let rate = 250_000.0;
    // A linear sweep: named as a chirp, and there is no chirp front end.
    let mut iq: Vec<C32> = vec![C32::new(0.0, 0.0); 2048];
    let mut phase = 0.0f64;
    for k in 0..24_000 {
        let t = k as f64 / rate;
        let f = -40_000.0 + 80_000.0 * (t / 0.096);
        phase += std::f64::consts::TAU * f / rate;
        iq.push(C32::new(phase.cos() as f32, phase.sin() as f32));
    }
    iq.extend(std::iter::repeat_n(C32::new(0.0, 0.0), 4096));

    let mut node = nodes::decode_nodes::BurstRouteNode::default_ism();
    let spec = pipeline::StreamSpec::iq(rate, common::Hz(433_920_000));
    let port = PortSpec { spec, latency: 0 };
    Node::negotiate(&mut node, std::slice::from_ref(&port)).expect("negotiate");

    let mut events = Vec::new();
    let mut tags = Vec::new();
    let mut out = [Payload::empty_of(PortKind::Pulses), Payload::empty_of(PortKind::Packets)];
    let inputs = [port];
    let mut ctx = NodeCtx::new(0, &inputs, &[], &mut events, &mut tags);
    let mut input = Payload::empty_of(PortKind::Iq);
    input.iq_mut().extend_from_slice(&iq);
    Node::process(&mut node, &[&input], &mut out, &mut ctx).expect("process");

    // The same burst as a packet, with what it was measured to be and the
    // samples it was cut from, so a log has a row to show rather than a line
    // of text.
    let packets = out[1].as_packets().expect("a packets port");
    assert!(!packets.is_empty(), "the burst left no packet");
    assert!(packets[0].measure.is_some(), "the packet carries no measurement");
    assert!(packets[0].iq.is_some(), "the packet carries no samples");

    let reported: Vec<_> = events
        .iter()
        .filter_map(|e| match e {
            Event::Decoded(d) if d.protocol == "unidentified" => Some(d),
            _ => None,
        })
        .collect();
    assert!(!reported.is_empty(), "a burst with no front end produced no log entry: {events:?}");
    let d = reported[0];
    assert!(d.modulation.is_some(), "reported without naming the modulation");
    assert!(d.detail.is_some(), "reported without saying why nothing read it");

    // And the other direction: an entry is a claim somebody reads, so a
    // classifier that is unsure must stay quiet. Raising the bar above what
    // any burst can reach has to silence it entirely, or the threshold is
    // decorative.
    let mut node = nodes::decode_nodes::BurstRouteNode::default_ism();
    node.set_report_confidence(1.01);
    let spec = pipeline::StreamSpec::iq(rate, common::Hz(433_920_000));
    let port = PortSpec { spec, latency: 0 };
    Node::negotiate(&mut node, std::slice::from_ref(&port)).expect("negotiate");
    let mut events = Vec::new();
    let mut tags = Vec::new();
    let mut out = [Payload::empty_of(PortKind::Pulses), Payload::empty_of(PortKind::Packets)];
    let inputs = [port];
    let mut ctx = NodeCtx::new(0, &inputs, &[], &mut events, &mut tags);
    Node::process(&mut node, &[&input], &mut out, &mut ctx).expect("process");
    let still: Vec<_> = events
        .iter()
        .filter(|e| matches!(e, Event::Decoded(d) if d.protocol == "unidentified"))
        .collect();
    assert!(still.is_empty(), "reported despite the confidence bar: {still:?}");
}
