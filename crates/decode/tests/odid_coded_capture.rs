//! An aircraft's whole Open Drone ID, off air, in one packet each.
//!
//! Bluetooth 5 Long Range carries the message pack rather than one message
//! per advertisement, so a single reception says who the aircraft is, where
//! it is, what the operator called it and who the operator is. This is that
//! path end to end: the coded PHY in `dsp::ble_coded`, the extended
//! advertising header in `decode::ble`, and the pack in `decode::odid`.
//!
//! The packets are the auxiliary ones. What the primary channel carries is a
//! pointer to a data channel chosen afresh each time, so these were recorded
//! by parking on a stretch of data channels instead. See
//! `testdata/fixtures.toml` for what else the capture is evidence of.

use common::C32;
use decode::odid::Message;
use dsp::ble_coded::Coding;
use dsp::{BleConfig, BleDetector, BleFrame};

const FIXTURE: &str = "../../testdata/odid_bt5lr_holybro_2474M_20000k.cs8";
const RATE: f64 = 20e6;
const CENTER: f64 = 2.474e9;

fn read() -> Option<Vec<BleFrame>> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(FIXTURE);
    if !p.exists() {
        eprintln!("skipping: {FIXTURE} absent, run testdata/fetch.sh to enable");
        return None;
    }
    let bytes = std::fs::read(&p).ok()?;
    let iq: Vec<C32> = bytes
        .chunks_exact(2)
        .map(|c| C32::new(c[0] as i8 as f32 / 128.0, c[1] as i8 as f32 / 128.0))
        .collect();
    let cfg = BleConfig {
        data_channels: true,
        ..BleConfig::default()
    };
    let mut det = BleDetector::new(RATE, CENTER, cfg);
    let mut out = Vec::new();
    for chunk in iq.chunks(1 << 20) {
        det.process(chunk, &mut out);
    }
    Some(out)
}

#[test]
fn a_message_pack_arrives_whole_on_the_long_range_phy() {
    let Some(frames) = read() else { return };
    assert!(
        frames.iter().all(|f| f.coding == Some(Coding::S8)),
        "the capture holds only long range packets"
    );
    let packs: Vec<Vec<decode::odid::Parsed>> = frames
        .iter()
        .filter_map(|f| decode::ble::parse(&f.pdu))
        .filter_map(|a| {
            let msgs: Vec<_> = a
                .data
                .iter()
                .filter(|s| s.kind == 0x16)
                .filter_map(|s| decode::odid::from_service_data(&s.value))
                .flatten()
                .collect();
            (!msgs.is_empty()).then_some(msgs)
        })
        .collect();
    assert!(
        packs.len() >= 8,
        "read {} message packs, expected 11",
        packs.len()
    );

    // The payload is on the data channels, which is what the pointer on the
    // primary channel was for. A pack read off 37, 38 or 39 would mean the
    // module was not doing extended advertising at all.
    let on_data = frames
        .iter()
        .filter(|f| decode::ble::parse(&f.pdu).is_some_and(|a| {
            a.data.iter().any(|s| {
                s.kind == 0x16 && decode::odid::from_service_data(&s.value).is_some()
            })
        }))
        .all(|f| f.channel <= 36);
    assert!(on_data, "a message pack arrived on an advertising channel");

    for pack in &packs {
        // Four messages in one reception, which is the whole point of the
        // pack: on the legacy transport these arrive as four advertisements
        // spread over seconds and any of them can be lost on its own.
        assert_eq!(pack.len(), 4, "a pack came back short");
        let id = pack.iter().find_map(|p| match &p.message {
            Message::BasicId { id, .. } => Some(id.clone()),
            _ => None,
        });
        assert_eq!(id.as_deref(), Some("123"));
        assert!(
            pack.iter().any(|p| matches!(p.message, Message::Location(_))),
            "no location message in a pack"
        );
        assert!(
            pack.iter().any(|p| matches!(p.message, Message::OperatorId { .. })),
            "no operator id in a pack"
        );
    }
}
