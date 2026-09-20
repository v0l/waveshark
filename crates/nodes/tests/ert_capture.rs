//! An ERT meter through the whole receiver, not merely through the decoder.
//!
//! `decode`'s corpus test reads rtl_433's own ERT recordings, which is the
//! evidence that the frames are right. This is the other half: a burst keyed
//! the way a meter keys one, into noise, through the auto node with nothing
//! said but the span's centre and rate. The meter has to be found as a source,
//! cut out, classified as on-off keying, sliced as Manchester at 32768 chips a
//! second and read. A protocol nothing routes a burst to is one nobody can
//! hear, however well its decoder works.
//!
//! The frame is real: rtl_433 25.02 and rtlamr both read it as meter 54585868
//! from `tests/ert/scm/01/g001_912.6M_2400k.cu8`, a gas meter at 562456.

use common::{C32, Hz};
use nodes::{NodeSpec, build_chain, registry};
use pipeline::StreamSpec;

const RATE: f64 = 2_400_000.0;
/// 32768 chips a second, one 30.5 us half symbol.
const CHIP_HZ: f64 = 32_768.0;
const CENTER: Hz = Hz(912_600_000);
/// Off the middle of the span, so the source has to be found rather than
/// assumed to be at zero.
const OFFSET_HZ: f64 = 180_000.0;

/// The frame as it goes out: 96 bits, each a pair of chips, a one falling and
/// a zero rising.
const SCM: [u8; 12] = [0xf9, 0x53, 0x06, 0xf0, 0x08, 0x95, 0x18, 0x40, 0xea, 0x0c, 0x10, 0x1a];

fn noise(n: usize, seed: &mut u64) -> Vec<C32> {
    let mut next = || {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        (*seed >> 11) as f32 / (1u64 << 53) as f32 - 0.5
    };
    (0..n).map(|_| C32::new(next() * 0.05, next() * 0.05)).collect()
}

/// Noise, then the frame keyed on a carrier, then noise again.
fn transmission(frame: Option<&[u8; 12]>) -> Vec<C32> {
    // The detector measures its floor over the first thirty-two frames, so the
    // burst cannot be at the front of the span.
    let mut seed = 0x51u64;
    let lead = 600_000;
    let mut iq = noise(lead, &mut seed);
    iq.extend(noise(300_000, &mut seed));
    if let Some(frame) = frame {
        let per_chip = RATE / CHIP_HZ;
        let chips: Vec<bool> = frame
            .iter()
            .flat_map(|b| (0..8).map(move |i| b & (0x80 >> i) != 0))
            .flat_map(|bit| [bit, !bit])
            .collect();
        for (i, on) in chips.iter().enumerate() {
            if !on {
                continue;
            }
            let from = lead + (i as f64 * per_chip) as usize;
            for k in 0..per_chip as usize {
                let ph = std::f64::consts::TAU * OFFSET_HZ * (from + k) as f64 / RATE;
                iq[from + k] += C32::new(0.3 * ph.cos() as f32, 0.3 * ph.sin() as f32);
            }
        }
    }
    iq
}

/// Every ERT decode the auto node produced, as `(protocol, id, consumption)`.
fn meters(iq: &[C32]) -> Vec<(String, i64, i64)> {
    let mut g = build_chain(
        StreamSpec::iq(RATE, CENTER),
        &[NodeSpec::new("auto"), NodeSpec::new("protocols")],
        &registry(),
    )
    .expect("build");
    let mut out = Vec::new();
    for block in iq.chunks(16_384) {
        g.feed_iq(block).expect("run");
        for d in g.output().as_packets().unwrap_or(&[]).iter().flat_map(|p| &p.stack) {
            if !d.kind.starts_with("ERT-") {
                continue;
            }
            // The meter's own identifier and what it has counted, which the
            // family's field names state as an identity and a reading.
            let id = d
                .subject
                .as_ref()
                .and_then(|e| e.id.to_string().split('/').next_back().map(str::to_string))
                .and_then(|s| s.parse::<i64>().ok())
                .unwrap_or(-1);
            let used = d
                .facts
                .iter()
                .find_map(|f| match f {
                    common::packet::Fact::Sensed(r)
                        if r.quantity == common::packet::Quantity::Consumption =>
                    {
                        Some(r.value as i64)
                    }
                    _ => None,
                })
                .unwrap_or(-1);
            let row = (d.kind.to_string(), id, used);
            if !out.contains(&row) {
                out.push(row);
            }
        }
    }
    out
}

#[test]
fn a_meter_on_the_span_is_found_and_read() {
    assert_eq!(meters(&transmission(Some(&SCM))), vec![("ERT-SCM".to_string(), 54585868, 562456)]);
}

#[test]
fn noise_alone_reports_no_meter() {
    assert_eq!(meters(&transmission(None)), Vec::new());
}
