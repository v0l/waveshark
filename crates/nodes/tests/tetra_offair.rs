use dsp::tetra::coding::{self, BLK_HALF};
use dsp::tetra::{
    BurstKind, NDB_BB1, NDB_BLK1, NDB_BLK2, TetraConfig, TetraDemod, TetraRx, speech,
};
use dsp::{FirDecim, Mixer};
use pipeline::node::{Node, NodeCtx, PortSpec};
use pipeline::port::{Payload, StreamSpec};

const CARRIER_HZ: f64 = 390_850_000.0;

fn fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/offair/tetra_voice_390.9M_250k.cu8");
    if !p.exists() {
        eprintln!("skipping: testdata/offair/tetra_voice_390.9M_250k.cu8 is local only");
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

fn fnv(frames: &[[u8; speech::FRAME_BITS]]) -> u64 {
    frames
        .iter()
        .flatten()
        .fold(0xcbf2_9ce4_8422_2325, |h, &b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
}

#[derive(Default)]
struct Slot {
    full: Vec<[u8; speech::FRAME_BITS]>,
    full_bad: usize,
    stolen: Vec<[u8; speech::FRAME_BITS]>,
    stolen_bad: usize,
    all_signalling: usize,
}

#[test]
#[ignore]
fn speech_frames_read_as_the_etsi_reference_decoder_reads_them() {
    let Some(buf) = fixture() else { return };
    let rate = buf.rate.as_f64();
    let factor = (rate / 72_000.0).round() as usize;
    let mut mixer = Mixer::new(buf.center.as_f64() - CARRIER_HZ, rate);
    let mut decim = FirDecim::design_hz(rate, factor, 12_150.0, 60.0);
    let mut demod = TetraDemod::new(rate / factor as f64, TetraConfig::default());
    let mut rx = TetraRx::new();
    let mut slots: [Slot; 5] = Default::default();
    let (mut mixed, mut narrow, mut bursts, mut blocks) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for chunk in buf.samples.chunks(16_384) {
        mixed.clear();
        mixer.process(chunk, &mut mixed);
        narrow.clear();
        decim.process(&mixed, &mut narrow);
        bursts.clear();
        demod.process(&narrow, &mut bursts);
        for b in &bursts {
            blocks.clear();
            rx.push(b, &mut blocks);
            let traffic = blocks.iter().any(|blk| {
                matches!(decode::tetra::Event::from_block(blk),
                    Some(decode::tetra::Event::Aach(a)) if a.traffic_marker().is_some())
            });
            let (Some(cell), Some(t), true) = (rx.cell, rx.time_at(b.slot), traffic) else {
                continue;
            };
            let slot = &mut slots[t.tn as usize];
            match b.kind {
                BurstKind::Normal1 => {
                    let mut chan = [0u8; speech::CHAN_BITS];
                    chan[..216].copy_from_slice(&b.bits[NDB_BLK1..NDB_BB1]);
                    chan[216..].copy_from_slice(&b.bits[NDB_BLK2..NDB_BLK2 + 216]);
                    let (frames, ok) = speech::decode(cell.scramb, &chan);
                    slot.full.extend(frames);
                    slot.full_bad += usize::from(!ok) * 2;
                }
                BurstKind::Normal2 => {
                    let mut half = [0u8; speech::HALF_BITS];
                    half.copy_from_slice(&b.bits[NDB_BLK2..NDB_BLK2 + speech::HALF_BITS]);
                    if coding::decode_block(&BLK_HALF, cell.scramb, &half).is_some() {
                        slot.all_signalling += 1;
                        continue;
                    }
                    let (frame, ok) = speech::decode_stolen(cell.scramb, &half);
                    slot.stolen.push(frame);
                    slot.stolen_bad += usize::from(!ok);
                }
                BurstKind::Sync => {}
            }
        }
    }
    let read = |tn: usize| {
        let s = &slots[tn];
        (s.full.len(), s.full_bad, fnv(&s.full), s.stolen.len(), s.stolen_bad, fnv(&s.stolen))
    };
    assert_eq!(
        read(2),
        (134, 0, 0x1093_9835_ad73_8aa3, 5, 0, 0x9327_268b_884d_837a),
        "timeslot 2, against ETSI EN 300 395-2 V1.3.1 cdecoder"
    );
    assert_eq!(
        read(4),
        (88, 0, 0x67a2_a878_a5cc_a168, 1, 0, 0x2299_e58e_b192_52cb),
        "timeslot 4, against ETSI EN 300 395-2 V1.3.1 cdecoder"
    );
    assert_eq!(
        (slots[2].all_signalling, slots[4].all_signalling),
        (11, 8),
        "half slots whose second half is signalling too"
    );
}

struct Heard {
    rows: usize,
    rows_crc_ok: usize,
    parties: std::collections::BTreeMap<(Option<String>, Option<String>), f64>,
    secrecy: Vec<common::Secrecy>,
    peak: f32,
}

fn listen(buf: &common::IqBuf, block: usize) -> Heard {
    let rate = buf.rate.as_f64();
    let mut node = nodes::tetra_nodes::TetraNode::new(CARRIER_HZ);
    let ins = [PortSpec { spec: StreamSpec::iq(rate, buf.center), latency: 0 }];
    node.negotiate(&ins).unwrap();
    let tags = Vec::new();
    let mut heard = Heard {
        rows: 0,
        rows_crc_ok: 0,
        parties: Default::default(),
        secrecy: Vec::new(),
        peak: 0.0,
    };
    for chunk in buf.samples.chunks(block) {
        let input = Payload::Iq(chunk.to_vec());
        let mut outs = [Payload::Packets(Vec::new()), Payload::Voice(Vec::new())];
        let (mut events, mut new_tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &tags, &mut events, &mut new_tags);
        node.process(&[&input], &mut outs, &mut ctx).unwrap();
        let [packets, voice] = outs;
        if let Payload::Packets(ps) = packets {
            for p in ps {
                let b = p.bytes();
                if nodes::tetra_nodes::traffic_burst_layout(b).is_some() {
                    heard.rows += 1;
                    heard.rows_crc_ok +=
                        usize::from(b[3] & nodes::tetra_nodes::TB_FLAG_CRC_OK != 0);
                }
            }
        }
        if let Payload::Voice(vs) = voice {
            for v in vs {
                *heard.parties.entry((v.to.clone(), v.from.clone())).or_default() += v.seconds();
                if let Some(o) = &v.over
                    && !heard.secrecy.contains(&o.secrecy)
                {
                    heard.secrecy.push(o.secrecy.clone());
                }
                heard.peak = v.pcm.iter().fold(heard.peak, |a, s| a.max(s.abs()));
            }
        }
    }
    heard
}

#[test]
#[ignore]
fn a_clear_call_is_spoken_whatever_the_block_size() {
    let Some(buf) = fixture() else { return };
    for block in [4_096, 16_384, 65_536] {
        let h = listen(&buf, block);
        assert_eq!((h.rows, h.rows_crc_ok), (117, 117), "{block} sample blocks");
        let spoken: f64 = h.parties.values().sum();
        assert!((spoken - 117.0 * 0.06).abs() < 1e-6, "{block} sample blocks: {spoken} s");
        let called: std::collections::BTreeSet<_> =
            h.parties.keys().filter_map(|(to, _)| to.as_deref()).collect();
        assert_eq!(called.len(), 3, "{block} sample blocks: groups called");
        let talkers: std::collections::BTreeSet<_> =
            h.parties.keys().filter_map(|(_, from)| from.as_deref()).collect();
        assert_eq!(talkers.len(), 2, "{block} sample blocks: talkers granted");
        assert_eq!(h.secrecy, [common::Secrecy::Clear], "{block} sample blocks");
        assert_eq!(
            h.peak, 1.0,
            "{block} sample blocks: ETSI sdecoder reaches full scale on timeslot 4"
        );
    }
}
