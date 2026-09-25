use common::{C32, Hz};
use nodes::dvbs2_nodes::Dvbs2Node;
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, PortKind, StreamSpec};

const TG4: &str = "dvbs2_tg4_1431.1M_20000k.cs8";
const SPUR: &str = "dvbs2_centre_spur_1431M_20000k.cs8";
const BBC: &str = "dvbs2_bbc_hd_1097M_40000k.cs8";
const TWO: &str = "dvbs2_two_carriers_1083M_61440k.cs8";
const BLOCK: usize = 65_536;

fn samples(name: &str) -> Option<Vec<C32>> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata").join(name);
    let raw = std::fs::read(path).ok()?;
    Some(
        raw.chunks_exact(2)
            .map(|c| C32::new(c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0))
            .collect(),
    )
}

fn alone() -> std::sync::MutexGuard<'static, ()> {
    static ONE_AT_A_TIME: std::sync::Mutex<()> = std::sync::Mutex::new(());
    ONE_AT_A_TIME.lock().unwrap_or_else(|e| e.into_inner())
}

fn skip(name: &str) {
    eprintln!("skipping: {name} absent, run testdata/fetch.sh");
}

struct Read {
    node: Dvbs2Node,
    stream: Vec<u8>,
}

fn through_the_stage(iq: &[C32], rate: f64, centre: f64) -> Read {
    through_a_channel(iq, rate, centre, centre, None)
}

fn through_a_channel(iq: &[C32], rate: f64, centre: f64, channel: f64, width: Option<f64>) -> Read {
    let mut node = Dvbs2Node::new(channel, None);
    if let Some(w) = width {
        node.within(w);
    }
    let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(centre as u64)), latency: 0 };
    node.negotiate(&[spec]).expect("a carrier in the span");
    let (mut stream, mut events) = (Vec::new(), Vec::new());
    for block in iq.chunks(BLOCK) {
        let payload = Payload::Iq(block.to_vec());
        let mut out = [
            Payload::empty_of(PortKind::Bytes),
            Payload::empty_of(PortKind::Video),
            Payload::empty_of(PortKind::Real),
        ];
        let ins = [spec];
        let (tags, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
        stream.extend_from_slice(out[0].as_bytes().unwrap_or(&[]));
    }
    stream.extend_from_slice(&node.flush(&mut Vec::new()).bytes);
    Read { node, stream }
}

fn reading(node: &Dvbs2Node, caption: &str) -> Option<String> {
    Node::readings(node).into_iter().find(|(c, _)| c == caption).map(|(_, v)| v)
}

fn named(node: &Dvbs2Node) -> Vec<String> {
    node.services().iter().filter_map(|s| s.name.clone()).collect()
}

#[test]
fn tg4_off_astra_2g_on_a_hackrf() {
    let _alone = alone();
    let Some(iq) = samples(TG4) else { return skip(TG4) };
    let read = through_the_stage(&iq, 20e6, 1_431_100_000.0);
    let stats = read.node.stats();
    assert_eq!(read.stream.len() % 188, 0);
    assert_eq!(reading(&read.node, "carrier").as_deref(), Some("8PSK 3/4, normal frames, pilots"));
    assert_eq!(reading(&read.node, "symbol rate").as_deref(), Some("14.250 Msym/s"));
    assert_eq!(reading(&read.node, "roll-off").as_deref(), Some("0.25"));
    assert_eq!(named(&read.node), ["TG4 Rasai BEO Lios Tuathail 2026"]);
    assert_eq!(
        (stats.frames, stats.ldpc_failed + stats.bch_failed, stats.crc_failed),
        (962, 6, 0),
        "{stats:?}"
    );
    assert_eq!(read.stream.len() / 188, 30_716);
}

#[test]
fn a_hackrf_centre_spur_seven_db_under_the_carrier_is_cancelled() {
    let _alone = alone();
    let Some(iq) = samples(SPUR) else { return skip(SPUR) };
    let read = through_the_stage(&iq, 20e6, 1_431_000_000.0);
    let stats = read.node.stats();
    let decoded = stats.frames - stats.ldpc_failed - stats.bch_failed;
    assert_eq!(stats.frames, 231, "{stats:?}");
    assert!(
        (180..=231).contains(&decoded),
        "{decoded} of 231 frames, floor 180; with the spur left in none decode at MER 4.4 dB"
    );
}

#[test]
fn bbc_hd_off_astra_2e_on_a_limesdr_with_its_17_mhz_spur() {
    let _alone = alone();
    let Some(iq) = samples(BBC) else { return skip(BBC) };
    let found = dsp::dvbs2::estimate(&iq[..1 << 20], 40e6).expect("a carrier");
    assert!((found.symbol_rate - 23e6).abs() < 1_150.0, "symbol rate {}", found.symbol_rate);
    let read = through_the_stage(&iq, 40e6, 1_097_000_000.0);
    let stats = read.node.stats();
    assert_eq!(reading(&read.node, "symbol rate").as_deref(), Some("22.999 Msym/s"));
    assert_eq!(
        named(&read.node),
        [
            "BBC One Y&L HD",
            "BBC One SE HD",
            "BBC Two HD",
            "BBC One NI HD",
            "ETV4",
            "BBC Three HD",
            "CBBC HD"
        ]
    );
    assert_eq!((stats.frames, stats.ldpc_failed + stats.bch_failed), (828, 0), "{stats:?}");
    assert_eq!(read.stream.len() / 188, 26_605);
}

#[cfg(feature = "ffmpeg")]
#[test]
fn pictures_keep_coming_after_a_frame_of_the_multiplex_is_lost() {
    let _alone = alone();
    let Some(iq) = samples(TG4) else { return skip(TG4) };
    let read = through_the_stage(&iq, 20e6, 1_431_100_000.0);
    let packets: Vec<&[u8]> = read.stream.chunks_exact(188).collect();
    let run = |lose: bool| {
        let mut media = decode::media::Media::new();
        let (mut out, mut before) = (Vec::new(), None);
        for (n, chunk) in packets.chunks(32).enumerate() {
            if lose && n % 40 == 20 {
                before.get_or_insert(out.len());
                continue;
            }
            media.push(&chunk.concat());
            std::thread::sleep(std::time::Duration::from_micros(500));
            media.take(&mut out);
        }
        media.finish(&mut out);
        let pictures = |o: &[decode::media::Out]| {
            o.iter().filter(|o| matches!(o, decode::media::Out::Picture(_))).count()
        };
        let after = pictures(&out[before.unwrap_or(0)..]);
        (pictures(&out), after, media.fault())
    };
    let (whole, _, _) = run(false);
    let (damaged, after, fault) = run(true);
    assert_eq!(fault, None, "the decoder gave up on a damaged packet");
    assert!(whole >= 15, "{whole} pictures from the whole stream, floor 15");
    assert!(after >= 10, "{after} of {damaged} pictures after the first loss, floor 10");
}

#[test]
fn a_symbol_rate_taken_off_a_spur_does_not_run_the_clock_away() {
    let _alone = alone();
    let Some(iq) = samples(BBC) else { return skip(BBC) };
    let cfg = dsp::dvbs2::Config {
        rate_hz: 40e6,
        symbol_rate: 17e6,
        rolloff: 0.25,
        gold: 0,
        within_hz: 20e6,
    };
    let mut phy = dsp::dvbs2::Dvbs2::new(cfg);
    let mut out = Vec::new();
    for block in iq.chunks(BLOCK) {
        phy.push(block, &mut out);
    }
    let drift = phy.symbol_rate() / 17e6 - 1.0;
    assert!(drift.abs() <= 1.0001e-3, "the clock moved {:.0} ppm off 17 Msym/s", drift * 1e6);
}

struct Live {
    frames_after: u64,
    longest: std::time::Duration,
    lost_s: f64,
    states: Vec<(f64, Option<pipeline::Acquisition>)>,
    weak: Option<String>,
}

struct Fade {
    during: std::ops::Range<f64>,
    snr_db: Option<f32>,
}

fn through_a_radio_that_cannot_wait(
    iq: &[C32],
    rate: f64,
    centre: f64,
    fade: Fade,
    seconds: f64,
) -> Live {
    let power = iq.iter().take(1 << 20).map(|x| x.norm_sqr()).sum::<f32>() / (1 << 20) as f32;
    let (keep, scale) = match fade.snr_db {
        Some(db) => (1.0, (6.0 * power / 10f32.powf(db / 10.0)).sqrt()),
        None => (0.0, 0.3),
    };
    let (mut states, mut weak) = (Vec::new(), None);
    let mut node = Dvbs2Node::new(centre, None);
    let spec = PortSpec { spec: StreamSpec::iq(rate, Hz(centre as u64)), latency: 0 };
    node.negotiate(&[spec]).expect("a carrier in the span");
    let mut events = Vec::new();
    let mut noise = 0x5EED_u64;
    let (mut next, mut lost, mut longest, mut before) =
        (0usize, 0usize, std::time::Duration::ZERO, None);
    let start = std::time::Instant::now();
    let end = (seconds * rate) as usize;
    while next < end {
        let due = start + std::time::Duration::from_secs_f64((next + BLOCK) as f64 / rate);
        std::thread::sleep(due.saturating_duration_since(std::time::Instant::now()));
        let heard = (start.elapsed().as_secs_f64() * rate) as usize / BLOCK * BLOCK;
        if heard > next + 4 * BLOCK {
            lost += heard - BLOCK - next;
            next = heard - BLOCK;
        }
        let at = next as f64 / rate;
        if at >= fade.during.end && before.is_none() {
            before = Some(node.stats().frames);
        }
        let block: Vec<C32> = (next..next + BLOCK)
            .map(|k| match fade.during.contains(&(k as f64 / rate)) {
                true => {
                    noise = noise.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1);
                    let (a, b) = ((noise >> 40) as f32, (noise >> 16 & 0xff_ffff) as f32);
                    let hiss = C32::new(a / 16_777_216.0 - 0.5, b / 16_777_216.0 - 0.5) * scale;
                    iq[k % iq.len()] * keep + hiss
                }
                false => iq[k % iq.len()],
            })
            .collect();
        next += BLOCK;
        let payload = Payload::Iq(block);
        let mut out = [
            Payload::empty_of(PortKind::Bytes),
            Payload::empty_of(PortKind::Video),
            Payload::empty_of(PortKind::Real),
        ];
        let ins = [spec];
        let (tags, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags)
            .with_block_seconds(BLOCK as f64 / rate);
        let called = std::time::Instant::now();
        Node::process(&mut node, &[&payload], &mut out, &mut ctx).expect("the stage runs");
        longest = longest.max(called.elapsed());
        states.push((at, Node::acquisition(&node)));
        if fade.during.contains(&at) {
            weak = reading(&node, "too weak").or(weak);
        }
    }
    Live {
        frames_after: node.stats().frames - before.unwrap_or(0),
        longest,
        lost_s: lost as f64 / rate,
        states,
        weak,
    }
}

#[test]
fn a_carrier_lost_to_noise_on_a_radio_that_cannot_wait_is_found_again() {
    let _alone = alone();
    let Some(iq) = samples(BBC) else { return skip(BBC) };
    let fade = Fade { during: 0.3..0.8, snr_db: None };
    let live = through_a_radio_that_cannot_wait(&iq, 40e6, 1_097_000_000.0, fade, 4.0);
    let report = format!(
        "{} frames after the fade, longest call {:?}, {:.2} s of samples lost",
        live.frames_after, live.longest, live.lost_s
    );
    assert!(live.frames_after >= 3_000, "floor 3000 of about 3300: {report}");
    assert!(live.longest < std::time::Duration::from_millis(50), "{report}");
}

#[test]
fn a_carrier_too_weak_to_read_is_not_shown_as_locked() {
    let _alone = alone();
    use pipeline::Acquisition;
    let Some(iq) = samples(BBC) else { return skip(BBC) };
    let fade = Fade { during: 0.3..1.8, snr_db: Some(-7.0) };
    let live = through_a_radio_that_cannot_wait(&iq, 40e6, 1_097_000_000.0, fade, 3.5);
    let state = |at: f64| live.states.iter().find(|(t, _)| *t >= at).and_then(|(_, a)| *a);
    let report = format!(
        "{:?} at 0.25 s, {:?} at 1.5 s, {:?} at 3.4 s, too weak {:?}, {} frames after",
        state(0.25),
        state(1.5),
        state(3.4),
        live.weak,
        live.frames_after
    );
    assert_eq!(state(0.25), Some(Acquisition::Locked), "{report}");
    assert_eq!(state(1.5), Some(Acquisition::Acquiring), "{report}");
    assert_eq!(state(3.4), Some(Acquisition::Locked), "{report}");
    assert!(live.weak.is_some(), "the frames too weak to read are counted: {report}");
    assert!(live.frames_after >= 1_600, "floor 1600 of about 1730: {report}");
}

#[test]
fn either_of_two_carriers_in_one_span_is_read_on_its_own_channel() {
    let _alone = alone();
    let Some(iq) = samples(TWO) else { return skip(TWO) };
    let mut got = Vec::new();
    for (channel, width) in [(1_068e6, 18e6), (1_097e6, 18e6), (1_068e6, 30e6), (1_097e6, 30e6)] {
        let read = through_a_channel(&iq, 61.44e6, 1_083_000_000.0, channel, Some(width));
        let stats = read.node.stats();
        got.push((
            channel / 1e6,
            width / 1e6,
            reading(&read.node, "offset"),
            stats.frames,
            stats.ldpc_failed + stats.bch_failed,
            reading(&read.node, "too weak"),
        ));
    }
    let found = |mhz, width, offset: &str, frames, failed, weak: Option<&str>| {
        (mhz, width, Some(offset.to_string()), frames, failed, weak.map(str::to_string))
    };
    assert_eq!(
        got,
        [
            found(1068.0, 18.0, "-103 kHz", 306, 0, Some("7")),
            found(1097.0, 18.0, "+399 kHz", 310, 1, None),
            found(1068.0, 30.0, "-103 kHz", 306, 0, Some("7")),
            found(1097.0, 30.0, "+399 kHz", 310, 1, None),
        ],
        "channel MHz, width MHz, offset, frames, failed, too weak"
    );
}

fn on_cpu_s() -> Option<f64> {
    let stat = std::fs::read_to_string("/proc/thread-self/schedstat").ok()?;
    Some(stat.split_whitespace().next()?.parse::<f64>().ok()? / 1e9)
}

#[test]
fn twenty_three_msym_at_61_44_msps_is_read_in_well_under_real_time() {
    let _alone = alone();
    if cfg!(debug_assertions) {
        return;
    }
    let Some(iq) = samples(TWO) else { return skip(TWO) };
    let rate = 61.44e6;
    let mut centred = Vec::new();
    dsp::Mixer::new(15e6, rate).process(&iq, &mut centred);
    let cfg = dsp::dvbs2::Config {
        rate_hz: rate,
        symbol_rate: 23e6,
        rolloff: 0.25,
        gold: 0,
        within_hz: 9e6,
    };
    let mut phy = dsp::dvbs2::Dvbs2::new(cfg);
    let mut out = Vec::new();
    let Some(started) = on_cpu_s() else { return };
    centred.chunks(BLOCK).for_each(|b| phy.push(b, &mut out));
    let took = (on_cpu_s().unwrap_or(0.0) - started) / (iq.len() as f64 / rate);
    assert_eq!(out.len(), 307);
    assert!(took < 0.67, "{took:.2} of real time on one thread, ceiling 0.67; 0.83 before");
}
