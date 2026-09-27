use super::*;

#[test]
fn a_walksnail_link_is_read_off_the_span_by_the_receiver() {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/walksnail_avatar_5805M_20000k.cs8");
    if !p.exists() {
        eprintln!("skipping: testdata/offair/walksnail_avatar_5805M_20000k.cs8 is local only");
        return;
    }
    let buf = sources::FileSource::open(&p).unwrap().read_all().unwrap();
    let mut rx = replay_receiver(&buf, None).unwrap();
    let out = replay_blocks(&mut rx, &buf);
    let links = read_by(&out, "walksnail");
    let parsed: Vec<decode::walksnail::Link> =
        links.iter().filter_map(|r| decode::walksnail::parse(r.bytes())).collect();
    assert_eq!(parsed.len(), 1, "one report for the one whole second in 1.2");
    let l = parsed[0];
    assert_eq!(l.frames, 139, "frames in the first second, as the node alone reads them");
    assert_eq!(l.constellation, Some(dsp::artosyn::Constellation::Qam64));
    let mer = l.mer_db.expect("a MER");
    assert!((15.0..16.5).contains(&mer), "MER {mer} dB: the node alone reads 15.8");
}
