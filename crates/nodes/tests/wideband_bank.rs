//! Decode several simultaneous transmitters from one wideband stream.
//!
//! The signal is real: the recorded Fine Offset transmission is frequency
//! shifted to several channels and summed, producing a stream that genuinely
//! contains four overlapping-in-time transmitters at different frequencies.
//! Synthesising four fake OOK signals would test far less, because real
//! captures carry the carrier offset, the amplitude ramps and the noise that
//! a channelizer actually has to cope with. rtl_433 measured this recording's
//! carrier at about -10.5 kHz from nominal, and that offset rides along into
//! every channel here.

use common::{Hz, C32};
use dsp::Mixer;
use nodes::{registry, ChannelBank, Gating, NodeSpec};
use pipeline::event::Event;
use sources::FileSource;

const FIXTURE: &str = "fineoffset_wh1080_433.92M_250k.cu8";
const RATE: f64 = 250_000.0;
/// 8 channels over 250 kHz: 31.25 kHz spacing, 62.5 kS/s per channel.
///
/// Deliberately not 16. The recording's carrier sits about 10.5 kHz off
/// nominal, and at 15.6 kHz spacing that lands close enough to a channel
/// boundary to split the signal across two channels. 31.25 kHz spacing keeps
/// it comfortably inside one.
const CHANNELS: usize = 8;
/// Channels to place a transmitter on.
const OCCUPIED: [usize; 4] = [0, 2, 3, 6];

fn fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata")
        .join(FIXTURE);
    if !p.exists() {
        return None;
    }
    Some(FileSource::open(&p).ok()?.read_all().ok()?)
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

/// Sum frequency-shifted copies of the capture into one wideband stream.
fn wideband(base: &[C32], bank: &ChannelBank) -> Vec<C32> {
    let mut out = vec![C32::new(0.0, 0.0); base.len()];
    for &c in &OCCUPIED {
        let offset = bank.channel_center(c).as_f64() - bank.channel_center(0).as_f64();
        // channel_center(0) is the bank centre, so this is the channel's
        // baseband offset.
        let mut m = Mixer::new(offset, RATE);
        let mut shifted = Vec::with_capacity(base.len());
        m.process(base, &mut shifted);
        for (o, s) in out.iter_mut().zip(&shifted) {
            *o += *s;
        }
    }
    out
}

fn ook_chain() -> Vec<NodeSpec> {
    vec![
        NodeSpec::new("envelope"),
        NodeSpec::new("pulse_detect").f("reset_us", 10_000.0).i("min_pulses", 20),
        NodeSpec::new("protocol_decode"),
    ]
}

fn make_bank() -> ChannelBank {
    ChannelBank::new(CHANNELS, 12, RATE, Hz::hz(433_920_000))
}

#[test]
fn decodes_four_simultaneous_transmitters_in_one_pass() {
    let buf = need_fixture!(fixture());
    let mut bank = make_bank();
    bank.set_all_chains(&ook_chain(), &registry()).expect("build chains");
    assert_eq!(bank.active_chains(), CHANNELS);

    let wide = wideband(&buf.samples, &bank);
    let events = bank.process(&wide).expect("run bank").to_vec();

    let decodes: Vec<(usize, Hz, String)> = events
        .iter()
        .filter_map(|e| match &e.event {
            Event::Decoded(d) => Some((e.channel, e.center, d.text.clone().unwrap_or_default())),
            _ => None,
        })
        .collect();

    let channels: Vec<usize> = decodes.iter().map(|(c, _, _)| *c).collect();
    assert_eq!(
        channels, OCCUPIED,
        "expected a decode on each occupied channel, got {decodes:#?}"
    );

    for (ch, center, text) in &decodes {
        assert!(text.contains("Fineoffset-WHx080"), "channel {ch}: {text}");
        assert!(text.contains("station_id=196"), "channel {ch}: {text}");
        assert!(text.contains("temperature_c=16.2"), "channel {ch}: {text}");
        assert!(text.contains("[CRC ok]"), "channel {ch}: {text}");
        // The reported frequency must be that channel's, not the bank centre.
        assert_eq!(*center, bank.channel_center(*ch));
    }
}

#[test]
fn empty_channels_stay_silent() {
    // Channel isolation: a transmitter must not leak into its neighbours and
    // produce phantom decodes. With 90 dB of stopband it should not come
    // close.
    let buf = need_fixture!(fixture());
    let mut bank = make_bank();
    bank.set_all_chains(&ook_chain(), &registry()).unwrap();

    let wide = wideband(&buf.samples, &bank);
    let events = bank.process(&wide).unwrap().to_vec();

    for e in &events {
        if let Event::Decoded(d) = &e.event {
            assert!(
                OCCUPIED.contains(&e.channel),
                "phantom decode on empty channel {}: {:?}",
                e.channel,
                d.text
            );
        }
    }
}

#[test]
fn a_single_transmitter_lands_on_the_channel_its_frequency_implies() {
    let buf = need_fixture!(fixture());
    let mut bank = make_bank();
    bank.set_all_chains(&ook_chain(), &registry()).unwrap();

    // Put one copy on channel 5 only.
    let target = 5usize;
    let offset = bank.channel_center(target).as_f64() - bank.channel_center(0).as_f64();
    let mut m = Mixer::new(offset, RATE);
    let mut wide = Vec::new();
    m.process(&buf.samples, &mut wide);

    let events = bank.process(&wide).unwrap().to_vec();
    let decoded: Vec<usize> = events
        .iter()
        .filter(|e| matches!(e.event, Event::Decoded(_)))
        .map(|e| e.channel)
        .collect();
    assert_eq!(decoded, vec![target], "decoded on {decoded:?}, expected [{target}]");

    // And the lookup agrees with where it actually landed.
    assert_eq!(bank.channel_for(bank.channel_center(target)), target);
}

#[test]
fn detection_gating_finds_the_occupied_channels() {
    let buf = need_fixture!(fixture());
    let mut bank = make_bank();
    bank.set_gating(Gating::OnDetection);
    bank.set_all_chains(&ook_chain(), &registry()).unwrap();

    let wide = wideband(&buf.samples, &bank);
    bank.process(&wide).unwrap();

    // The burst detector runs regardless of gating, so it should have opened
    // on exactly the channels carrying a transmitter. Peak-hold rather than
    // mean power is what makes this work on a short burst.
    let mut peaks = Vec::new();
    bank.drain_peak_hold_db(&mut peaks);
    assert_eq!(peaks.len(), CHANNELS);

    let mut ranked: Vec<usize> = (0..CHANNELS).collect();
    ranked.sort_by(|&a, &b| peaks[b].total_cmp(&peaks[a]));
    let top4: std::collections::BTreeSet<usize> = ranked[..4].iter().copied().collect();
    let want: std::collections::BTreeSet<usize> = OCCUPIED.iter().copied().collect();
    assert_eq!(top4, want, "strongest channels {top4:?}, expected {want:?}, peaks {peaks:?}");
}

#[test]
fn results_are_deterministic_despite_parallel_execution() {
    // Rayon schedules channels in whatever order it likes; the output must not
    // depend on that, or logs become undiffable and tests flaky.
    let buf = need_fixture!(fixture());
    let mut a = make_bank();
    a.set_all_chains(&ook_chain(), &registry()).unwrap();
    let wide = wideband(&buf.samples, &a);

    let first: Vec<(usize, String)> = a
        .process(&wide)
        .unwrap()
        .iter()
        .map(|e| (e.channel, format!("{:?}", e.event)))
        .collect();

    for _ in 0..5 {
        let mut b = make_bank();
        b.set_all_chains(&ook_chain(), &registry()).unwrap();
        let again: Vec<(usize, String)> = b
            .process(&wide)
            .unwrap()
            .iter()
            .map(|e| (e.channel, format!("{:?}", e.event)))
            .collect();
        assert_eq!(first, again, "parallel execution produced a different result");
    }
}

#[test]
fn channels_without_a_chain_are_skipped() {
    let buf = need_fixture!(fixture());
    let mut bank = make_bank();
    // Only channel 2 gets a chain, though four channels carry signal.
    bank.set_chain(2, &ook_chain(), &registry()).unwrap();
    assert_eq!(bank.active_chains(), 1);

    let wide = wideband(&buf.samples, &bank);
    let events = bank.process(&wide).unwrap().to_vec();
    for e in &events {
        assert_eq!(e.channel, 2, "an unconfigured channel produced {:?}", e.event);
    }
    assert!(events.iter().any(|e| matches!(e.event, Event::Decoded(_))));
}

/// What the app runs when it decodes everything it can hear: no protocol
/// chosen, no modulation chosen, nothing tuned by hand.
///
/// The bank finds the bursts and the protocols run once, downstream, over
/// what every front end produced. That split is the thing under test: the
/// channels must hand over enough for a decoder that never saw the radio to
/// name the device.
#[test]
fn the_automatic_chain_decodes_without_being_told_the_modulation() {
    use common::{Packet, PacketBody};
    let base = need_fixture!(fixture());
    let mut bank = make_bank();
    // Exactly what the app runs: gated on detection, both modulations, every
    // channel, nothing chosen by hand.
    bank.set_gating(Gating::OnDetection);
    bank.set_detector_config(nodes::ism_detector_config());
    bank.set_all_graphs(nodes::ism_decode_graph).expect("build graphs");
    let wide = wideband(&base.samples, &bank);

    let mut decoder = nodes::PacketDecodeNode::default();
    let mut found: Vec<(u64, String)> = Vec::new();
    let mut unknown = 0;
    for block in wide.chunks(65_536) {
        bank.process(block).expect("run bank");
        let packets: Vec<Packet> = bank
            .packages()
            .iter()
            .map(|p| Packet {
                at_us: 0,
                center_hz: p.center_hz,
                bandwidth_hz: bank.channel_bandwidth() as u32,
                rssi_dbfs: p.rssi_dbfs,
                snr_db: p.snr_db,
                modulation: p.modulation,
                body: PacketBody::Pulses(p.pulses.clone()),
            })
            .collect();
        for d in decode_packets(&mut decoder, packets) {
            // Bursts nothing claims are still reported, and they are counted
            // rather than matched: the point here is the decode.
            //
            // There used to be several on this recording, and there are none
            // now. They were the FSK front end's reading of an on-off keyed
            // transmission, which is a burst the classifier no longer sends
            // it. A phantom reading of a real packet is the one kind of
            // unknown worth losing.
            if d.protocol == "unknown" {
                unknown += 1;
                continue;
            }
            found.push((d.center.0, d.text.clone().unwrap_or_default()));
        }
    }
    assert_eq!(unknown, 0, "the only bursts here are the transmission, and it decodes");

    let mut channels: Vec<usize> = found.iter().map(|(hz, _)| bank.channel_for(Hz(*hz))).collect();
    channels.sort_unstable();
    channels.dedup();
    assert_eq!(channels, OCCUPIED, "wrong channels decoded: {found:?}");
    for (_, text) in &found {
        assert!(text.contains("Fineoffset-WHx080"), "{text}");
        assert!(text.contains("[CRC ok]"), "{text}");
    }
}

/// Run the bus decoder over one block's worth of packets.
fn decode_packets(
    node: &mut nodes::PacketDecodeNode,
    packets: Vec<common::Packet>,
) -> Vec<pipeline::event::Decoded> {
    use pipeline::node::{NodeCtx, PortSpec, Simple};
    use pipeline::port::{Payload, PortKind};
    let mut spec = pipeline::StreamSpec::iq(0.0, Hz(0)).with_kind(PortKind::Packets);
    spec.bandwidth = 31_250.0;
    let ins = [PortSpec { spec, latency: 0 }];
    let mut events = Vec::new();
    let tags = Vec::new();
    let mut new_tags = Vec::new();
    let mut out = Payload::Packets(Vec::new());
    let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
    Simple::process(node, &Payload::Packets(packets), &mut out, &mut ctx).unwrap();
    node.hits().to_vec()
}

/// The channel graph must measure the burst rather than assume it, or the
/// bank is an OOK chain with extra steps.
#[test]
fn the_automatic_chain_measures_the_burst_before_choosing_a_front_end() {
    let bank = make_bank();
    let g = nodes::ism_decode_graph(bank.channel_spec(0)).expect("build graph");
    let t = g.topology();
    let kinds: Vec<&str> = t.nodes.iter().map(|n| n.kind.as_str()).collect();
    assert!(kinds.contains(&"burst_route"), "{kinds:?}");
    // The front ends are inside it, one per burst rather than all of them per
    // sample. Which one runs is `dsp::route`'s business and is tested there.
    assert!(!kinds.contains(&"pulse_detect"), "{kinds:?}");
    assert!(!kinds.contains(&"fsk_detect"), "{kinds:?}");
    // No decoder here: a channel finds bursts, and the protocols run once
    // on the packet bus over everything every front end produced.
    assert!(
        !kinds.iter().any(|k| *k == "protocol_decode"),
        "the protocols moved to the bus: {kinds:?}"
    );
}
