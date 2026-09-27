use super::*;
use crate::chain::OOK_CHANNEL_HZ;
use crate::row::Reception;

mod band_2g4;
mod band_5g8;
mod banks;
mod captures;
mod eas;
mod front_end;
mod gsm;
mod m17;
mod pmr446;
mod sideband;
mod squelch;
mod tetra;
mod thread;
mod timing;
mod tx;
mod zoom;

/// Rows one protocol read, by the id its decoder publishes.
fn read_by<'a>(rows: &'a [Reception], id: &str) -> Vec<&'a Reception> {
    rows.iter().filter(|r| r.protocol() == id).collect()
}

/// Rows one protocol read of one kind: `("ism", "Fineoffset-WHx080")`.
fn read_as<'a>(rows: &'a [Reception], id: &str, kind: &str) -> Vec<&'a Reception> {
    rows.iter().filter(|r| r.protocol() == id && r.kind() == kind).collect()
}

/// Whether the transmitter's own check stood behind the bytes.
fn checked(r: &Reception) -> bool {
    r.integrity() == common::packet::Integrity::Passed
}

/// A reading the decoder stated, in whatever unit it stated it.
fn sensed(r: &Reception, q: common::packet::Quantity) -> Option<f64> {
    r.packet.facts().find_map(|(_, f)| match f {
        common::packet::Fact::Sensed(x) if x.quantity == q => Some(x.value),
        _ => None,
    })
}

/// The channel a decode says it was on, as the protocol numbers them.
fn channel(r: &Reception) -> Option<u16> {
    r.packet.facts().find_map(|(_, f)| match f {
        common::packet::Fact::Channel(c) => Some(c.claims.unwrap_or(c.heard)),
        _ => None,
    })
}

/// Who was transmitting, as the decoder identified them.
fn who(r: &Reception) -> Option<String> {
    r.packet.subject().map(|e| e.id.to_string())
}

/// The transmit chain the receiver would draw for a plan, on a radio that
/// can transmit with nothing keyed. Named for what it is outside this
/// file: the strip asks what would go out, and this answers.
pub(crate) fn transmit_plan(plan: &Plan) -> Option<crate::chain::TxPlan> {
    derive_tx(plan, true, None)
}

/// Wait for the radio thread, which runs on its own clock. Fails the
/// test rather than hanging.
fn until(what: &str, mut ready: impl FnMut() -> bool) {
    let until = std::time::Instant::now() + std::time::Duration::from_secs(10);
    while !ready() {
        assert!(std::time::Instant::now() < until, "waited ten seconds for {what}");
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
}

fn strip_channel(id: u64, offset: f64) -> ChannelSpec {
    ChannelSpec {
        id,
        label: format!("CH{id}"),
        offset_hz: offset,
        mode: ChanMode::Audio(Demod::Nfm),
        bandwidth_hz: None,
        audio_low_hz: None,
        squelch_db: None,
        agc: true,
        voice: false,
        reads: None,
        // As the strip sends it: nobody has said anything about
        // transmitting, and the mode decides.
        tx: None,
        tone: None,
    }
}

fn block(n: usize) -> Vec<C32> {
    (0..n)
        .map(|i| {
            let p = std::f64::consts::TAU * 0.1 * i as f64;
            C32::new(p.cos() as f32 * 0.5, p.sin() as f32 * 0.5)
        })
        .collect()
}

fn fixture() -> Option<common::IqBuf> {
    let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../testdata/fineoffset_wh1080_433.92M_250k.cu8");
    if !p.exists() {
        return None;
    }
    sources::FileSource::open(&p).ok()?.read_all().ok()
}

/// A silent buffer, for building a receiver whose shape is the point.
fn empty_buf(rate: f64, center: Hz) -> common::IqBuf {
    common::IqBuf { samples: vec![C32::default(); 1024], rate: Sps(rate as u64), center, seq: 0 }
}

fn every_row_carries_its_measurements(rows: &[&Reception]) {
    assert!(!rows.is_empty(), "nothing to check");
    for r in rows {
        assert!(r.rssi_dbfs().is_finite(), "{} has no level: {:?}", r.protocol(), r.rssi_dbfs());
        assert!(r.snr_db().is_finite(), "{} has no SNR: {:?}", r.protocol(), r.snr_db());
        let iq = r
            .packet
            .carrier
            .iq
            .as_ref()
            .unwrap_or_else(|| panic!("{} kept no samples", r.protocol()));
        assert!(!iq.samples.is_empty(), "{} kept an empty burst", r.protocol());
        assert!(iq.rate > 0.0 && iq.center_hz > 0, "{} samples with no stream", r.protocol());
    }
}
