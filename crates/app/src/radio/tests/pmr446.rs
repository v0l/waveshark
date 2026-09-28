use super::*;

/// A PMR446 handheld through the whole receiver, from IQ to words.
///
/// The path this proves is the one that has no test anywhere else: the
/// span is decimated to a channel, the channel is demodulated as narrow
/// FM, its audio goes on the bus as speech because the channel says it
/// is voice, the transcriber on the bus tap collects it, and a line of
/// text comes out with the words that were spoken into the handheld. Any
/// one of those failing shows up here as an empty transcript, which is
/// exactly what it looks like on screen.
///
/// Skipped without a model, since fetching one is not something a test
/// should do to somebody's machine.
#[cfg(feature = "stt")]
#[test]
fn a_handheld_on_pmr446_arrives_as_words() {
    let Some(buf) = pmr446_fixture() else {
        eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
        return;
    };
    let dir = crate::chain::default_model_dir();
    if !dir.join("config.json").exists() {
        eprintln!("skipping: no whisper model in {}", dir.display());
        return;
    }
    // PMR446 channel 1. The capture is tuned 49.1 kHz below it, which is
    // what the dial was set to rather than anything about the signal.
    const CHANNEL_HZ: f64 = 446_049_100.0;
    let mut plan = replay_plan(&buf, false);
    plan.fronts.clear();
    plan.channels = vec![ChannelSpec {
        id: 1,
        label: "PMR1".into(),
        offset_hz: CHANNEL_HZ - buf.center.as_f64(),
        mode: ChanMode::Audio(Demod::Nfm),
        bandwidth_hz: None,
        audio_low_hz: None,
        // Open: the transmission is what the file holds, and a squelch
        // decision is not what this test is about.
        squelch_db: Some(-200.0),
        agc: true,
        blanker: None,
        voice: true,
        reads: None,
        tx: None,
        tone: None,
    }];
    let since = std::time::Instant::now();
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");
    // Transcription is off in the graph the receiver draws, because
    // writing down what people said is not something to start doing
    // because nobody said otherwise. Switching it on is what an operator
    // does, and is what this test is about.
    let id = rx.node_of_stage(crate::chain::derived::TRANSCRIBE).expect("a transcriber");
    rx.set_node_param(id.0, "enabled", pipeline::ParamValue::Bool(true)).expect("the switch");
    // The receiver's own transcript, so what this test reads is what
    // this receiver heard.
    let log = rx.transcript().clone();
    let _ = replay_blocks(&mut rx, &buf);
    // The model runs on its own thread, so the answer arrives after the
    // samples have run out, the way it does in the receiver.
    let silence = vec![C32::default(); 16_384];
    let mut said: Vec<crate::transcripts::Utterance> = Vec::new();
    for _ in 0..600 {
        let _ = rx.process(&silence);
        said = log
            .lock()
            .recent(usize::MAX)
            .into_iter()
            .filter(|u| u.at >= since && u.key.channel_hz == CHANNEL_HZ as u64)
            .cloned()
            .collect();
        if said.iter().any(|u| u.settled) {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    let text = said.iter().map(|u| u.text.as_str()).collect::<Vec<_>>().join(" ");
    let words = text.to_lowercase();
    // One transmission is one line. More than one means the utterance
    // was cut where nobody paused, which is the failure this count is
    // here to catch; none means nothing reached the model at all.
    assert_eq!(said.len(), 1, "read as {text:?}");
    // Spaces removed before looking for the digits: the model writes a
    // spoken "one two three" as "123" or as "1 2 3" depending on what it
    // makes of the pauses, and both are the number that was said.
    let digits = words.replace(char::is_whitespace, "");
    assert!(digits.contains("123"), "read as {text:?}");
    assert!(words.contains("test"), "read as {text:?}");
    // On the channel it was heard on, since the key is what the call
    // list and the transcript view meet on.
    let key = said[0].key.clone();
    assert_eq!(key.channel_hz, CHANNEL_HZ as u64, "read on {key}");
}

/// The handheld on one analogue strip, with the channel's voice mark
/// either way. Returns what the tap heard, the calls it made and how many
/// blocks the speaker was handed nothing on.
fn pmr446_strip(
    buf: &common::IqBuf,
    voice: bool,
) -> (Vec<crate::mix::heard::LiveCall>, crate::calls::Calls, usize, f32) {
    let mut plan = replay_plan(buf, false);
    plan.fronts.clear();
    plan.channels = vec![ChannelSpec {
        id: 1,
        label: "PMR1".into(),
        offset_hz: 446_049_100.0 - buf.center.as_f64(),
        mode: ChanMode::Audio(Demod::Nfm),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: None,
        agc: true,
        blanker: None,
        voice,
        reads: None,
        tx: None,
        tone: None,
    }];
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");
    assert!(
        !rx.topology().nodes.iter().any(|n| n.kind == "packet_bus"),
        "an analogue channel put something on the packet bus"
    );

    let mut calls = crate::calls::Calls::new();
    let mut heard: Vec<crate::mix::heard::LiveCall> = Vec::new();
    let mut pcm: Vec<f32> = Vec::new();
    let mut silent_blocks = 0;
    for block in buf.samples.chunks(16_384) {
        if rx.process(block).is_err() {
            break;
        }
        let out = rx.audio_out().0;
        if out.is_empty() {
            silent_blocks += 1;
        }
        pcm.extend(out.iter().step_by(2));
        assert!(rx.rows(std::time::Instant::now()).is_empty(), "speech is not a packet");
        for c in rx.heard_mut().expect("the tap").take_calls() {
            calls.hear(&c);
            heard.push(c);
        }
    }

    let rms = (pcm.iter().map(|v| v * v).sum::<f32>() / pcm.len() as f32).sqrt();
    (heard, calls, silent_blocks, rms)
}

/// A channel marked as voice is heard through its own fader, and is a row
/// on the call list without anything of it reaching the packet bus.
///
/// An analogue over used to be wrapped in an empty packet so the call
/// list, which read only the packet bus, would see it: that put a row
/// saying nothing into the packet log for every transmission, made the
/// channel inaudible until something subscribed to it, and was wrong in
/// principle, since there is no packet in analogue speech. The tap is
/// what reads it, and the voice mark is the operator saying people talk
/// here, which is the same statement a decoder makes with `Airtime::voice`
/// and the reason the agent's own channel is listed.
#[test]
fn a_voice_channel_is_heard_and_is_a_call() {
    let Some(buf) = pmr446_fixture() else {
        eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
        return;
    };
    let (heard, calls, silent_blocks, rms) = pmr446_strip(&buf, true);

    // Heard, with no subscription to anything: the fader is the strip's.
    assert_eq!(silent_blocks, 0, "the bus handed the speaker nothing on {silent_blocks} blocks");
    assert!(rms > 0.01, "the channel is silent at the speaker: {rms:e} rms");

    assert!(!heard.is_empty(), "a channel marked as voice made no call");
    assert!(heard.iter().all(|c| c.to == "PMR1"), "a call not named for the strip");
    let now = std::time::Instant::now();
    let active = calls.active(now);
    assert_eq!(active.len(), 1, "one channel, one row: {active:?}");
    assert_eq!(active[0].to, "PMR1");
    assert_eq!(active[0].system, crate::mix::fader::ANALOGUE);
    assert_eq!(active[0].channel_hz, 446_049_100.0);
    assert!(active[0].seconds > 1.0, "airtime of {}s", active[0].seconds);
    // The coded squelch this handheld is set to, read off the audio: an
    // FM carrier says nothing about who is on it, and for analogue
    // traffic the tone is the only group there is.
    assert_eq!(active[0].code.as_deref(), Some("141.3"), "the tone was not read");
    assert!(
        heard.iter().any(|c| c.code.as_deref() == Some("141.3")),
        "the tone never reached the tap's call"
    );
}

/// The same channel with the mark off: audible on the strip, written down
/// by the transcriber, and no row. A mode and a frequency do not say
/// whether what is coming out is a conversation, a repeater idling or an
/// airband loop, so nothing here guesses.
#[test]
fn a_channel_not_marked_as_voice_is_heard_and_is_not_a_call() {
    let Some(buf) = pmr446_fixture() else {
        eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
        return;
    };
    let (heard, calls, silent_blocks, rms) = pmr446_strip(&buf, false);
    assert_eq!(silent_blocks, 0, "the bus handed the speaker nothing");
    assert!(rms > 0.01, "the channel is silent at the speaker: {rms:e} rms");
    assert!(heard.is_empty(), "an unmarked channel became a call: {heard:?}");
    assert!(calls.active(std::time::Instant::now()).is_empty());
}

/// The same handheld, kept as audio: the whole path from IQ to a record
/// on the disk, with the file read back and decoded.
///
/// What it proves that the node's own tests cannot: the tap really
/// carries a tuned analogue channel's audio, at the rate the strip runs
/// at, labelled with the channel it was heard on, and the recorder is in
/// the graph the receiver draws rather than only in a test's patch.
#[test]
fn a_handheld_on_pmr446_is_recorded_and_reads_back() {
    let Some(buf) = pmr446_fixture() else {
        eprintln!("skipping: pmr446_test_446.0M_512k.cs8 absent, run testdata/fetch.sh");
        return;
    };
    const CHANNEL_HZ: f64 = 446_049_100.0;
    let dir = std::env::temp_dir().join(format!("sr-calls-replay-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let mut plan = replay_plan(&buf, false);
    plan.fronts.clear();
    plan.channels = vec![ChannelSpec {
        id: 1,
        label: "PMR1".into(),
        offset_hz: CHANNEL_HZ - buf.center.as_f64(),
        mode: ChanMode::Audio(Demod::Nfm),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: Some(-200.0),
        agc: true,
        blanker: None,
        voice: true,
        reads: None,
        tx: None,
        tone: None,
    }];
    let mut rx = crate::chain::Receiver::build(&plan, Default::default()).expect("a receiver");
    let id = rx.node_of_stage(crate::chain::derived::CALL_LOG).expect("a call log");
    rx.set_node_param(id.0, "dir", pipeline::ParamValue::Text(dir.display().to_string()))
        .expect("the folder");
    rx.set_node_param(id.0, "enabled", pipeline::ParamValue::Bool(true)).expect("the switch");
    let _ = replay_blocks(&mut rx, &buf);
    // The over ends with the samples, so the hang has to run out before
    // the record is written: that is what a real channel going quiet
    // does.
    let silence = vec![C32::default(); 16_384];
    for _ in 0..80 {
        let _ = rx.process(&silence);
    }
    let recorded = rx.recorder().expect("the recorder reports");
    assert_eq!(recorded.calls, 1, "one transmission is one record");
    drop(rx);

    let files: Vec<std::path::PathBuf> =
        std::fs::read_dir(&dir).expect("the folder").flatten().map(|e| e.path()).collect();
    assert_eq!(files.len(), 1, "one segment, got {files:?}");
    let calls = crate::calllog::read(&files[0]).expect("the log reads back");
    assert_eq!(calls.len(), 1);
    let c = &calls[0];
    assert_eq!(c.channel_hz, CHANNEL_HZ as u64, "recorded on {} Hz", c.channel_hz);
    assert_eq!(c.system, crate::mix::fader::ANALOGUE);
    // The capture is six seconds with the handheld keyed for about five
    // of them, and the recording holds the speech rather than the file.
    assert!(
        (3.5..=6.0).contains(&c.seconds()),
        "a five second over came back as {:.2} s",
        c.seconds()
    );
    let speech = c.speech().expect("the audio decodes");
    assert_eq!(speech.rate, crate::calllog::RATE);
    let rms = (speech.pcm.iter().map(|v| v * v).sum::<f32>() / speech.pcm.len() as f32).sqrt();
    assert!((0.03..0.2).contains(&rms), "the recording reads {rms:.4} rms");
    // The tap carries this channel at seventeen times full scale, so
    // without the limiter every sample would be a square wave. A handful
    // of samples on the codec's ringing is not clipping.
    let clipped = speech.pcm.iter().filter(|s| s.abs() > 0.99).count();
    assert!(clipped < 50, "{clipped} of {} samples are clipped", speech.pcm.len());
    assert!(c.peak > 10.0, "the tap's level is not being reported: {}", c.peak);
    // 16 kbit/s and nothing between overs: a six second over is 12 kB.
    let bytes = std::fs::metadata(&files[0]).expect("the segment").len();
    assert!((11_000..14_000).contains(&bytes), "six seconds of speech cost {bytes} bytes");
    let _ = std::fs::remove_dir_all(&dir);
}

fn pmr446_fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/pmr446_test_446.0M_512k.cs8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}
