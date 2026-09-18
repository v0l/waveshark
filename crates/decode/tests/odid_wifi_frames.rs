//! Open Drone ID over Wi-Fi, read out of frames somebody else captured.
//!
//! The unit tests in `decode::odid` and in `nodes::wifi_nodes` build the
//! beacon element and the NAN action frame in the same file that reads them,
//! so an assumption shared by both sides passes. These frames are from the
//! sample captures shipped with `opendroneid/wireshark-dissector`
//! (`odid_wifi_bcn_sample.pcap` and `odid_wifi_sample.pcap`, MIT), recorded
//! in monitor mode off a transmitter this project has nothing to do with, and
//! what is asserted below is what that project's own dissector reports for
//! them.
//!
//! The sniffer stripped the frame check sequence: the element chain in every
//! frame ends exactly at the captured length. `decode::wifi::parse` takes the
//! PSDU with the FCS on it and drops the last four bytes, so one is computed
//! back on here. Nothing reads it; the demodulator is what checks an FCS.

use decode::odid::{self, IdType, Message};
use decode::wifi;

/// A beacon carrying a five message pack: basic id, location, self id,
/// system and operator id, from `odid_wifi_bcn_sample.pcap`.
const BEACON_PACK: &str = concat!(
    "80000000ffffffffffff84cca860432484cca8604324000d0000000000000000b80b2104",
    "030106000e4742522d4f502d31323341424344dd85fa0bbc0dd0f0190500004d46473141",
    "3031323334353637383900000000000050f610005c527ebcba251ba88cb4b60000aa0998",
    "08394100000a00300052656372656174696f6e616c00000000000000000000004004a485",
    "251b6edbb3b601003200000000150000000000000050004742522d4f502d313233414243",
    "44000000000000000000",
);

/// A NAN public action frame carrying a one message pack, from
/// `odid_wifi_sample.pcap`.
const NAN_ACTION: &str = concat!(
    "d0000000516f9a01000084cca8604324506f9a01017950060409506f9a13032700886919",
    "9d92090100101d22f0190150004742522d4f502d31323341424344000000000000000000",
    "0e040001000222",
);

/// A beacon from the NAN capture, carrying a single message rather than a
/// pack of five: the same transmitter sends both forms.
const BEACON_SINGLE: &str = concat!(
    "80000000ffffffffffff84cca860432484cca860432460060000000000000000b80b2104",
    "030106000e4742522d4f502d31323341424344dd21fa0bbc0d22f0190150004742522d4f",
    "502d31323341424344000000000000000004",
);

/// A NAN synchronisation beacon from the same capture. It carries the Remote
/// ID service id inside a Wi-Fi Alliance element and no message at all, which
/// is the frame a decoder matching on the service id alone would report an
/// aircraft for.
const NAN_SYNC_BEACON: &str = concat!(
    "80000000ffffffffffff84cca8604324506f9a0101794006000000000000000000022004",
    "dd22506f9a13000200feea010d0084cca8604324eafe00000000000206008869199d9209",
);

fn psdu(hex: &str) -> Vec<u8> {
    let mut v: Vec<u8> = (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
        .collect();
    v.extend(dsp::wifi::crc32(&v).to_le_bytes());
    v
}

/// Everything Open Drone ID in a frame, whichever transport carried it.
fn messages(hex: &str) -> Vec<odid::Parsed> {
    let f = wifi::parse(&psdu(hex)).expect("a MAC frame");
    let mut m: Vec<odid::Parsed> = f
        .vendor
        .iter()
        .filter_map(|v| odid::from_vendor_element(v.oui, v.kind, &v.data))
        .flatten()
        .collect();
    if let Some(a) = &f.action
        && let Some(pack) = odid::from_nan_action(a.category, a.code, &a.body)
    {
        m.extend(pack);
    }
    m
}

#[test]
fn a_captured_beacon_carries_the_five_messages_the_dissector_reports() {
    let m = messages(BEACON_PACK);
    assert_eq!(m.len(), 5);
    assert!(m.iter().all(|p| p.version == 0), "F3411-19, which is what the sample transmits");

    let Message::BasicId { id_type, ua_type, id } = &m[0].message else {
        panic!("not a basic id: {:?}", m[0].message);
    };
    assert_eq!(*id_type, IdType::None);
    assert_eq!(*ua_type, 0);
    assert_eq!(id, "MFG1A0123456789");

    let Message::Location(l) = &m[1].message else { panic!("not a location: {:?}", m[1].message) };
    assert!((l.latitude.unwrap() - 45.545_746_8).abs() < 1e-7, "{:?}", l.latitude);
    assert!((l.longitude.unwrap() + 122.968_149_6).abs() < 1e-7, "{:?}", l.longitude);
    assert_eq!(l.geodetic_alt_m, Some(237.0));
    assert_eq!(l.height_m, Some(100.0));
    assert!(l.height_above_takeoff);
    assert_eq!(l.speed_ms, Some(20.5));
    assert_eq!(l.track_deg, Some(92));
    assert_eq!(l.status, 0);
    // The pressure altitude and the vertical speed are the specification's
    // "unknown", which is what this transmitter sends for both.
    assert_eq!(l.pressure_alt_m, None);
    assert_eq!(l.vertical_speed_ms, None);

    let Message::SelfId { text, description_type } = &m[2].message else {
        panic!("not a self id: {:?}", m[2].message);
    };
    assert_eq!(text, "Recreational");
    assert_eq!(*description_type, 0);

    let Message::System(s) = &m[3].message else { panic!("not a system: {:?}", m[3].message) };
    assert!((s.operator_latitude.unwrap() - 45.544_387_6).abs() < 1e-7);
    assert!((s.operator_longitude.unwrap() + 122.972_686_6).abs() < 1e-7);
    assert_eq!(s.area_count, 1);
    assert_eq!(s.area_radius_m, 500);
    // Category 1, class 5: the EU label, which the flags byte says is there.
    assert_eq!(s.classification, Some((1, 5)));
    assert_eq!(s.operator_alt_m, None);

    let Message::OperatorId { id, .. } = &m[4].message else {
        panic!("not an operator id: {:?}", m[4].message);
    };
    assert_eq!(id, "GBR-OP-123ABCD");
}

#[test]
fn a_captured_nan_action_frame_carries_the_one_message_it_holds() {
    let m = messages(NAN_ACTION);
    assert_eq!(m.len(), 1);
    let Message::OperatorId { id, .. } = &m[0].message else {
        panic!("not an operator id: {:?}", m[0].message);
    };
    assert_eq!(id, "GBR-OP-123ABCD");

    // The frame is a public action frame, so the pack is in its body and not
    // in any element: the beacon path must find nothing in it.
    let f = wifi::parse(&psdu(NAN_ACTION)).expect("a MAC frame");
    assert_eq!(f.vendor.len(), 0);
    let a = f.action.expect("an action frame");
    assert_eq!((a.category, a.code), (0x04, 0x09));
}

#[test]
fn a_captured_beacon_carrying_one_message_reads_as_one() {
    let m = messages(BEACON_SINGLE);
    assert_eq!(m.len(), 1);
    let Message::OperatorId { id, .. } = &m[0].message else {
        panic!("not an operator id: {:?}", m[0].message);
    };
    assert_eq!(id, "GBR-OP-123ABCD");
}

/// The frames that are not an aircraft saying anything, including the one
/// that names the Remote ID service without carrying a message.
#[test]
fn a_nan_synchronisation_beacon_is_not_an_aircraft() {
    let f = wifi::parse(&psdu(NAN_SYNC_BEACON)).expect("a MAC frame");
    assert_eq!(f.vendor.len(), 1, "the Wi-Fi Alliance element and nothing else");
    assert_eq!(f.vendor[0].oui, [0x50, 0x6f, 0x9a]);
    assert!(
        f.vendor[0].data.windows(6).any(|w| w == odid::NAN_SERVICE_ID),
        "the capture's sync beacon does name the service"
    );
    assert_eq!(messages(NAN_SYNC_BEACON).len(), 0);
}

/// A truncated or corrupted frame reports nothing rather than reading fields
/// at the wrong offsets. Every prefix of a real beacon is tried, which is
/// what a frame cut short by a demodulator looks like.
#[test]
fn a_frame_cut_short_reports_no_aircraft() {
    let whole = psdu(BEACON_PACK);
    for n in 0..whole.len() - 1 {
        let cut = &whole[..n];
        let Some(f) = wifi::parse(cut) else { continue };
        let found: usize = f
            .vendor
            .iter()
            .filter_map(|v| odid::from_vendor_element(v.oui, v.kind, &v.data))
            .map(|m| m.len())
            .sum();
        assert_eq!(found, 0, "a {n} byte prefix reported {found} messages");
    }
}

/// A bit flipped anywhere in the pack may change what a message says and may
/// not change how many there are. 128 pack bytes times 8 bits is 1024 frames:
/// 994 still read as five messages, 29 break the pack header badly enough
/// that the single message form is read instead, and one clears a bit of the
/// count and gives four. Nothing gives more than five, which is the thing
/// worth pinning: a count read out of a corrupted byte must not walk off the
/// end of the element into whatever follows it.
#[test]
fn a_flipped_bit_never_makes_a_longer_pack() {
    let whole = psdu(BEACON_PACK);
    // The pack starts after the element header, the OUI, the type and the
    // transmitter's message counter.
    let start = 55 + 2 + 5;
    let mut counts = std::collections::BTreeMap::new();
    for i in start..start + 3 + 5 * odid::MESSAGE_LEN {
        for bit in 0..8 {
            let mut bad = whole.clone();
            bad[i] ^= 1 << bit;
            *counts.entry(messages_of(&bad)).or_insert(0usize) += 1;
        }
    }
    assert_eq!(counts.get(&5).copied(), Some(994), "{counts:?}");
    assert_eq!(counts.get(&1).copied(), Some(29), "{counts:?}");
    assert_eq!(counts.get(&4).copied(), Some(1), "{counts:?}");
    assert!(counts.keys().all(|&n| n <= 5), "{counts:?}");
}

fn messages_of(psdu: &[u8]) -> usize {
    let Some(f) = wifi::parse(psdu) else { return 0 };
    f.vendor
        .iter()
        .filter_map(|v| odid::from_vendor_element(v.oui, v.kind, &v.data))
        .map(|m| m.len())
        .sum()
}
