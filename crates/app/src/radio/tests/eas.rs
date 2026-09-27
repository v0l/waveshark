use super::*;

/// An alert relayed on a channel somebody is listening to is read off
/// that channel's audio, through the receiver the radio thread runs.
///
/// The relay is the case: most alerts are not on the weather channels,
/// and a station passing one on its own FM channel is a station a
/// receiver is already tuned to. What this pins is the whole path, the
/// strip's mixer, channel filter, discriminator, squelch and audio
/// decimator included, ending in a row on the packet bus. The stage's
/// own audio test proves the decoder; this proves the wiring.
/// A station relaying the header on its own FM channel: three copies a
/// second apart in the audio, on a carrier `offset` from the tuner.
fn relayed_alert(rate: f64, offset: f64, header: &str) -> Vec<C32> {
    let audio_hz = 48_000.0;
    let mut audio = vec![0.0f32; audio_hz as usize / 2];
    for _ in 0..3 {
        audio.extend(dsp::afsk::modulate(
            &decode::eas::encode_bits(header),
            audio_hz,
            dsp::afsk::SAME,
        ));
        audio.extend(std::iter::repeat_n(0.0, audio_hz as usize));
    }
    audio.extend(std::iter::repeat_n(0.0, audio_hz as usize));

    let step = (rate / audio_hz) as usize;
    let mut phase = 0.0f64;
    let mut iq = Vec::with_capacity(audio.len() * step);
    for &s in &audio {
        for _ in 0..step {
            let f = offset + f64::from(s) * nodes::eas_nodes::DEVIATION_HZ;
            phase += std::f64::consts::TAU * f / rate;
            iq.push(C32::new(phase.cos() as f32 * 0.5, phase.sin() as f32 * 0.5));
        }
    }
    iq
}

/// The National Weather Service's own example header: a tornado warning
/// for two Missouri counties, from the Kansas City office.
const TOR: &str = "ZCZC-WXR-TOR-029095-029183+0030-1250100-KEAX/NWS-";

#[test]
fn an_alert_relayed_on_a_listening_channel_is_read_off_its_audio() {
    let (rate, center, offset) = (480_000.0, Hz(162_400_000), 100_000.0);
    let iq = relayed_alert(rate, offset, TOR);

    let mut plan = crate::chain::tests::plan(rate, center);
    plan.fronts.clear();
    let mut spec = strip_channel(1, offset);
    spec.reads = Some("eas".into());
    plan.channels = vec![spec];
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");
    assert!(rx.refused.is_none(), "{:?}", rx.refused);
    // One mixer and one channel filter: the channel's own. A second
    // front end reading the same channel off the span is what this
    // wiring exists to avoid.
    assert_eq!(
        rx.topology().nodes.iter().filter(|n| n.kind == "mixer").count(),
        1,
        "a second front end was built for the alert"
    );

    let mut rows = Vec::new();
    for block in iq.chunks(16_384) {
        rx.process(block).expect("the graph runs");
        rows.extend(rx.rows(std::time::Instant::now()));
    }
    let alerts = read_by(&rows, "eas");
    assert_eq!(alerts.len(), 1, "{} alerts off one relayed header", alerts.len());
    assert_eq!(alerts[0].kind(), "alert");
    assert_eq!(
        alerts[0].freq(),
        162_500_000.0,
        "the alert is reported where the channel is tuned, not where the tuner is"
    );
    let said = alerts[0].packet.facts().find_map(|(_, f)| match f {
        common::packet::Fact::Alert(a) => Some(a.clone()),
        _ => None,
    });
    let said = said.expect("the alert itself");
    assert_eq!(said.kind, common::packet::AlertKind::Weather);
    assert_eq!(said.severity, common::packet::Severity::Immediate);
    assert!(
        said.text.as_deref().is_some_and(|t| t.contains("Tornado Warning")),
        "the row says {:?}",
        said.text
    );
}

/// A minute of noise on a channel somebody is listening to is not an
/// alert, and the stage that read none of it still reads the one that
/// follows.
#[test]
fn noise_on_a_listening_channel_reads_as_no_alert() {
    let (rate, center, offset) = (480_000.0, Hz(162_400_000), 100_000.0);
    let mut seed = 0x243f_6a88_85a3_08d3u64;
    let mut noise = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 40) as f32 / 8_388_608.0 - 1.0
    };
    let mut plan = crate::chain::tests::plan(rate, center);
    plan.fronts.clear();
    let mut spec = strip_channel(1, offset);
    spec.reads = Some("eas".into());
    spec.squelch_db = Some(-120.0);
    plan.channels = vec![spec];
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");

    // A minute of it, which is sixty times the length of an alert.
    let mut rows = Vec::new();
    for _ in 0..(60.0 * rate / 16_384.0) as usize {
        let block: Vec<C32> = (0..16_384).map(|_| C32::new(noise() * 0.3, noise() * 0.3)).collect();
        rx.process(&block).expect("the graph runs");
        rows.extend(rx.rows(std::time::Instant::now()));
    }
    assert_eq!(read_by(&rows, "eas").len(), 0, "noise became an alert");
    assert_eq!(rows.len(), 0, "noise became {} rows", rows.len());

    // And the stage is still there reading: the same header through the
    // same channel, after the noise.
    for block in relayed_alert(rate, offset, TOR).chunks(16_384) {
        rx.process(block).expect("the graph runs");
        rows.extend(rx.rows(std::time::Instant::now()));
    }
    assert_eq!(read_by(&rows, "eas").len(), 1, "the alert after the noise was lost");
}
