use super::*;

/// Two transmit channels, which is what recalling one from the bank
/// makes: the chain is drawn for the one going on air, not for whichever
/// is first on the strip. Without this the rebuild that puts a key on air
/// replaced the keyed channel's plan with the first channel's, so a
/// channel set to MIC transmitted the other one's test tone.
#[test]
fn the_transmit_chain_follows_the_channel_going_on_air() {
    let mut plan = crate::chain::tests::plan(2_400_000.0, Hz(145_000_000));
    let ch = |id: u64, offset: f64, source: TxSource| ChannelSpec {
        id,
        label: format!("CH{id}"),
        offset_hz: offset,
        mode: ChanMode::Audio(Demod::Nfm),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: None,
        voice: false,
        reads: None,
        agc: true,
        blanker: None,
        denoise: false,
        denoise_db: dsp::denoise::DEFAULT_DEPTH_DB,
        tx: Some(TxSpec { source, ..Default::default() }),
        tone: None,
    };
    plan.channels = vec![ch(1, 25_000.0, TxSource::Tone), ch(2, -50_000.0, TxSource::Mic)];
    let first = derive_tx(&plan, true, None).expect("a chain to hold ready");
    assert_eq!(first.spec.source, TxSource::Tone);
    assert_eq!(first.on_air, Hz(145_025_000));
    let keyed = derive_tx(&plan, true, Some(2)).expect("the keyed channel's chain");
    assert_eq!(keyed.spec.source, TxSource::Mic);
    assert_eq!(keyed.on_air, Hz(144_950_000));
    // A channel that cannot transmit does not take the chain away from
    // one that can.
    plan.channels.push(ChannelSpec { id: 3, tx: None, ..plan.channels[0].clone() });
    assert_eq!(derive_tx(&plan, true, Some(3)), Some(first));
    assert_eq!(derive_tx(&plan, false, Some(2)), None);
}

fn vox_channel(id: u64, offset: f64) -> ChannelSpec {
    let vox = VoxSpec { on: true, threshold: 0.1, tail_ms: 100.0, anti_trip: true };
    ChannelSpec {
        tx: Some(TxSpec { source: TxSource::Mic, vox, ..Default::default() }),
        ..strip_channel(id, offset)
    }
}

fn speech_then_quiet(speaking: usize, quiet: usize) -> Arc<dyn audio::AudioSource> {
    let mut pcm: Vec<f32> = (0..speaking)
        .map(|i| 0.5 * (std::f32::consts::TAU * 400.0 * i as f32 / 48_000.0).sin())
        .collect();
    pcm.extend(std::iter::repeat_n(0.0, quiet));
    Arc::new(audio::Canned::new(pcm, 48_000.0, false))
}

/// A voice keys the radio and a quiet room lets it up, on the thread that
/// holds the device.
///
/// The stage only says the key should be down; retuning a half duplex
/// radio and handing its stream to the transmitter is this thread's half,
/// and no test reached it until the microphone could be handed in.
#[test]
fn a_voice_on_the_microphone_keys_the_radio_and_a_quiet_room_lets_it_up() {
    let center = Hz(446_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device_hearing(
        Box::new(dev),
        center,
        rate,
        1024,
        Some(speech_then_quiet(24_000, 192_000)),
    );
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));
    radio.send(Cmd::Channels(vec![vox_channel(1, 50_000.0)]));

    until("the voice to key the channel", || radio.status.keyed.load(Ordering::Relaxed) == 1);
    assert!(radio.status.vox_open.load(Ordering::Relaxed), "it keyed with the vox shut");
    assert!(watch.keyed(), "the key lit and the device was never asked to transmit");
    until("a tenth of a second on the antenna", || watch.transmitted_len() > 240_000);

    until("the key to come up when the room went quiet", || {
        radio.status.keyed.load(Ordering::Relaxed) == 0
    });
    until("the device to be given back", || !watch.keyed());
    let sent = watch.transmitted_len();
    std::thread::sleep(std::time::Duration::from_millis(100));
    assert_eq!(watch.transmitted_len(), sent, "it went on transmitting after the vox let up");
    assert_eq!(radio.status.error.lock().clone(), None, "the over went out and it complained");
}

/// A key pressed by hand is the operator's, and a vox that hears nothing
/// does not take it off them.
#[test]
fn a_hand_key_on_a_vox_channel_is_not_let_up_by_the_vox() {
    let center = Hz(446_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device_hearing(
        Box::new(dev),
        center,
        rate,
        1024,
        Some(speech_then_quiet(0, 480_000)),
    );
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));
    radio.send(Cmd::Channels(vec![vox_channel(1, 50_000.0)]));
    until("the vox to have measured the quiet room", || {
        !radio.status.vox_open.load(Ordering::Relaxed)
            && f32::from_bits(radio.status.vox_level.load(Ordering::Relaxed)) < 0.1
    });
    assert_eq!(radio.status.keyed.load(Ordering::Relaxed), 0, "silence keyed the radio");

    radio.send(Cmd::Key(Some(1)));
    until("the hand key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
    until("an over on the antenna", || watch.transmitted_len() > 240_000);
    std::thread::sleep(std::time::Duration::from_millis(200));
    assert_eq!(
        radio.status.keyed.load(Ordering::Relaxed),
        1,
        "the vox let up a key the operator is holding"
    );
    assert!(watch.keyed(), "the device came off transmit under a hand key");

    radio.send(Cmd::Key(None));
    until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
    assert_eq!(radio.status.error.lock().clone(), None);
}

/// A channel added and keyed, on a radio, end to end.
///
/// Everything between a command arriving and a sample reaching the
/// antenna: the plan, the derived transmit chain, the graph, the
/// transmitter's thread and the device. This is the join that kept
/// breaking, and it could not be tested until there was a radio to test
/// it on.
#[test]
fn a_channel_added_and_keyed_reaches_the_antenna() {
    let center = Hz(446_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));

    radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));
    radio.send(Cmd::Key(Some(1)));
    until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
    // Enough of an over to read: a tenth of a second at the span's rate.
    until("a tenth of a second on the antenna", || watch.transmitted_len() > 240_000);
    assert!(watch.keyed(), "the device was never asked to transmit");
    assert_eq!(radio.status.error.lock().clone(), None, "it went on air and still complained");

    // What actually reached the antenna, rather than how much of it.
    // The default source is a 1 kHz tone, so the whole pipeline is
    // judged by whether a discriminator reads 1 kHz back off it at the
    // narrow band deviation: the clock, the tone, the modulator, the
    // sink and the executor that ran them.
    let air = watch.transmitted();
    let mut demod = dsp::FmDemod::new(rate.as_f64(), nodes::NBFM_DEVIATION_HZ);
    let mut audio = Vec::new();
    demod.process(&air[4_800..], &mut audio);
    let seg = &audio[1_000..];
    let crossings = seg.windows(2).filter(|w| w[0] <= 0.0 && w[1] > 0.0).count();
    let hz = crossings as f64 * rate.as_f64() / seg.len() as f64;
    assert!((hz - 1_000.0).abs() < 20.0, "the air carried {hz:.0} Hz, not a 1 kHz tone");
    let peak = seg.iter().fold(0.0f32, |m, v| m.max(v.abs()));
    assert!(peak > 0.5, "the tone is there at {peak}, too quiet to be full deviation");
    let level = air[4_800].norm();
    assert!(
        air[4_800..].iter().all(|s| (s.norm() - level).abs() < 0.05),
        "an FM carrier holds its envelope"
    );

    radio.send(Cmd::Key(None));
    until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
    until("the device to be given back", || !watch.keyed());
    let sent = watch.transmitted_len();
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert_eq!(watch.transmitted_len(), sent, "it went on transmitting after the key came up");
}

/// Two channels, worked one after the other, on a radio.
///
/// Recalling a second channel beside the first is what a bank is for, and
/// the second one has to key up as readily as the first. The two are one
/// transmit chain, so nothing between them is rebuilt and the failure is
/// silent: the key lights and nothing goes out.
#[test]
fn a_second_channel_keys_up_as_readily_as_the_first() {
    let center = Hz(145_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));
    radio.send(Cmd::Channels(vec![strip_channel(1, 25_000.0), strip_channel(2, -50_000.0)]));

    let mut was = 0;
    for id in [1, 2, 1, 2] {
        radio.send(Cmd::Key(Some(id)));
        until(&format!("channel {id} to go on air"), || {
            radio.status.keyed.load(Ordering::Relaxed) == id
        });
        until(&format!("channel {id} on the antenna"), || watch.transmitted_len() > was);
        radio.send(Cmd::Key(None));
        until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
        was = watch.transmitted_len();
    }
    assert_eq!(radio.status.error.lock().clone(), None, "every over went out and it complained");
}

#[test]
fn a_receiver_switched_off_still_transmits_and_a_transmitter_switched_off_does_not() {
    let center = Hz(145_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));
    radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));

    let mut edits = crate::patch::Edits::default();
    edits.off.push(crate::patch::builtin::SPAN);
    radio.send(Cmd::Edits(edits.clone()));
    until("the source to be off", || radio.status.chain().is_some_and(|t| t.input_off));
    let rx = radio.status.chain().expect("a chain");
    let running: Vec<&str> = rx
        .nodes
        .iter()
        .filter(|n| !n.idle && !n.inputs.is_empty())
        .filter(|n| !n.outputs.iter().any(|(_, s)| s.is_tx()))
        .map(|n| n.kind.as_str())
        .collect();
    assert_eq!(
        running,
        [
            "call_network",
            "calls",
            "fader",
            "heard",
            "audio_bus",
            "homeassistant",
            "transcribe_live",
            "call_log",
            "speaker"
        ],
        "only what the network feeds, and not the radio"
    );
    while radio.frames.try_recv().is_ok() {}
    std::thread::sleep(std::time::Duration::from_millis(200));
    assert_eq!(radio.frames.try_iter().count(), 0, "the spectrum went on drawing");

    radio.send(Cmd::Key(Some(1)));
    until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
    until("a tenth of a second on the antenna", || watch.transmitted_len() > 240_000);
    radio.send(Cmd::Key(None));
    until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
    until("the device to be given back", || !watch.keyed());

    edits.off.push(crate::chain::derived::TX_RADIO);
    radio.send(Cmd::Edits(edits));
    until("the transmitter to be off", || {
        radio.status.chain().is_some_and(|t| {
            t.nodes.iter().any(|n| n.tag == Some(crate::chain::derived::TX_RADIO) && n.off)
        })
    });
    let sent = watch.transmitted_len();
    radio.send(Cmd::Key(Some(1)));
    until("the key to be refused", || radio.status.error.lock().is_some());
    assert_eq!(radio.status.error.lock().as_deref(), Some(TRANSMITTER_OFF));
    assert_eq!(radio.status.keyed.load(Ordering::Relaxed), 0);
    assert!(!watch.keyed(), "the device was asked to transmit");
    assert_eq!(watch.transmitted_len(), sent);
}

/// A key goes on air even when a stage in the graph refuses the span it
/// is wired to.
///
/// The refusal is answered by building again without that stage, and the
/// radio the key handed in used to go with the attempt that failed: an
/// edited stage saved from a wider span, a screen decoder among them,
/// left every key on the strip dead until the edit was deleted.
#[test]
fn a_stage_that_refuses_the_span_does_not_take_the_key_with_it() {
    let center = Hz(145_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));

    // A screen decoder wants 4 MS/s and this span is 2.4, so it refuses
    // as it is wired.
    let mut edits = crate::patch::Edits::default();
    edits.stages.push(crate::patch::Stage {
        id: 1,
        kind: "tempest".into(),
        settings: pipeline::registry::Settings::new(),
    });
    edits.links.push(crate::patch::Link { from: crate::patch::Source::Span, to: (1, 0) });
    radio.send(Cmd::Edits(edits));
    radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));
    radio.send(Cmd::Key(Some(1)));
    until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
    until("a tenth of a second on the antenna", || watch.transmitted_len() > 240_000);
    assert!(watch.keyed(), "the device was never asked to transmit");
}

#[test]
fn a_stage_left_out_says_why_on_every_rebuild_until_it_builds() {
    let center = Hz(145_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    let mut edits = crate::patch::Edits::default();
    edits.stages.push(crate::patch::Stage {
        id: 1,
        kind: "tempest".into(),
        settings: pipeline::registry::Settings::new(),
    });
    edits.links.push(crate::patch::Link { from: crate::patch::Source::Span, to: (1, 0) });
    radio.send(Cmd::Edits(edits.clone()));
    until("the stage to be left out", || radio.status.refused.lock().is_some());
    let why = radio.status.refused.lock().take().unwrap_or_default();
    assert!(why.contains("left out"), "{why}");
    radio.send(Cmd::Center(Hz(145_100_000)));
    until("a rebuild to say it again", || radio.status.refused.lock().is_some());
    edits.stages.clear();
    edits.links.clear();
    radio.send(Cmd::Edits(edits));
    let rev = radio.status.patch().0;
    until("the edit that builds", || radio.status.patch().0 > rev);
    std::thread::sleep(std::time::Duration::from_millis(100));
    assert_eq!(radio.status.refused.lock().clone(), None);
}

/// A full duplex radio hears the band through its own transmission.
///
/// The half duplex case is the one every test uses, because it is what a
/// HackRF is. On a radio with a synthesiser per direction the receiver
/// must not retune to transmit, must not draw the loopback over the span,
/// and must go on decoding while the key is down.
#[test]
fn a_full_duplex_radio_keeps_receiving_through_an_over() {
    let dev = sources::FileRadio::hearing(
        Hz(145_000_000),
        Sps(2_400_000),
        vec![common::C32::new(0.25, 0.0); 4_096],
    )
    .half_duplex(false)
    .as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device(Box::new(dev), Hz(145_000_000), Sps(2_400_000), 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));
    radio.send(Cmd::Channels(vec![strip_channel(1, 25_000.0)]));
    radio.send(Cmd::Key(Some(1)));
    until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
    until("something on the antenna", || watch.transmitted_len() > 0);

    // The dial has not moved: that is what the second synthesiser is for,
    // and the spectrum is still arriving from where it was.
    let moved = radio.frames.try_iter().any(|f| (f.center - 145_000_000.0).abs() > 1.0);
    assert!(!moved, "a full duplex radio retuned itself to transmit");
    radio.send(Cmd::Key(None));
    until("the key to come up", || radio.status.keyed.load(Ordering::Relaxed) == 0);
    assert!(radio.status.running.load(Ordering::Relaxed));
}

/// A radio unplugged mid-over ends the over, on the radio thread.
///
/// The key has to come up with it. A lit key over a transmitter that
/// stopped is worse than no key: nothing else on the screen would say
/// the transmission had ended.
#[test]
fn a_radio_unplugged_mid_over_brings_the_key_up() {
    let center = Hz(446_000_000);
    let rate = Sps(2_400_000);
    let dev = sources::FileRadio::silent(center, rate).as_fast_as_it_can();
    let watch = dev.watcher();
    let radio = Radio::on_device(Box::new(dev), center, rate, 1024);
    until("the radio to start", || radio.status.running.load(Ordering::Relaxed));
    until("the radio to say it transmits", || radio.status.can_transmit.load(Ordering::Relaxed));
    radio.send(Cmd::Channels(vec![strip_channel(1, 50_000.0)]));
    radio.send(Cmd::Key(Some(1)));
    until("the key to take", || radio.status.keyed.load(Ordering::Relaxed) == 1);
    until("something on the antenna", || watch.transmitted_len() > 0);

    watch.unplug();
    until("the key to come up on its own", || radio.status.keyed.load(Ordering::Relaxed) == 0);
    let said = radio.status.error.lock().clone().unwrap_or_default();
    assert!(said.contains("the radio stopped taking samples"), "it said {said:?} instead");
}

/// Where an over goes out is not part of the chain that makes it.
///
/// Two channels of one mode are the same stages, so keying between them
/// is the radio moving and nothing else: no rebuild, and so no restart of
/// the spectrum's averaging or of whatever the source has open. Comparing
/// whole plans instead made every such key-up a rebuild, and the rebuild
/// then found it had nothing to do.
#[test]
fn a_frequency_is_not_part_of_the_transmit_chain() {
    use crate::chain::TxPlan;
    let here = TxPlan { spec: TxSpec::default(), mode: TxMode::Nfm, on_air: Hz(145_500_000) };
    let there = TxPlan { on_air: Hz(433_500_000), ..here };
    assert!(here.same_chain(&there));
    assert_ne!(here, there, "they are two plans still: the radio and the monitor read it");

    let mic = TxPlan { spec: TxSpec { source: TxSource::Mic, ..here.spec }, ..here };
    assert!(!here.same_chain(&mic), "a microphone is a different source stage");
    let louder = TxPlan { spec: TxSpec { mic_gain: 9.0, ..mic.spec }, ..mic };
    assert!(!mic.same_chain(&louder), "the gain is a setting on the source stage");
    let shifted = TxPlan { spec: TxSpec { shift_hz: -600_000.0, ..here.spec }, ..here };
    assert!(here.same_chain(&shifted), "a repeater shift only moves the radio");
    let toned =
        TxPlan { spec: TxSpec { tone: Some(dsp::squelch::Coded::Tone(8)), ..here.spec }, ..here };
    assert!(!here.same_chain(&toned), "a tone is a stage of its own");
}
