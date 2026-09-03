//! Probe the TETRA fixture carrier by carrier, printing what each layer saw.
//!
//! cargo run --release -p nodes --example tetra_probe -- testdata/tetra_downlink_391.5M_2400k.cu8

use dsp::tetra::{TetraConfig, TetraDemod, TetraRx};
use dsp::{FirDecim, Mixer};

fn main() {
    let path = std::env::args().nth(1).expect("path to capture");
    let buf = sources::FileSource::open(std::path::Path::new(&path))
        .unwrap()
        .read_all()
        .unwrap();
    let rate = buf.rate.as_f64();
    let center = buf.center.as_f64();
    for hz in [391_181_000.0f64, 391_704_500.0] {
        for out_rate in [72_000.0, 25_000.0] {
            let factor = (rate / out_rate).round() as usize;
            let mut mixer = Mixer::new(center - hz, rate);
            let mut decim = FirDecim::design_hz(rate, factor, 12_150.0, 60.0);
            let mut demod = TetraDemod::new(rate / factor as f64, TetraConfig::default());
            let mut rx = TetraRx::new();
            let (mut mixed, mut narrow) = (Vec::new(), Vec::new());
            let mut quality = Vec::new();
            let mut blocks = Vec::new();
            let mut kinds = std::collections::BTreeMap::new();
            for chunk in buf.samples.chunks(65_536) {
                mixed.clear();
                mixer.process(chunk, &mut mixed);
                narrow.clear();
                decim.process(&mixed, &mut narrow);
                let mut got = Vec::new();
                demod.process(&narrow, &mut got);
                for b in &got {
                    *kinds.entry(format!("{:?}", b.kind)).or_insert(0u32) += 1;
                    rx.push(b, &mut blocks);
                }
                quality.extend(got.iter().map(|b| b.quality));
            }
            // What the signalling says, with `TETRA_PDUS` set: every call
            // control PDU, and a tally by name.
            if std::env::var_os("TETRA_PDUS").is_some() && out_rate > 50_000.0 {
                let mut tally = std::collections::BTreeMap::new();
                let mut shown = 0;
                let mut aach = std::collections::BTreeMap::new();
                for b in &blocks {
                    if matches!(b.lchan, dsp::tetra::Lchan::Aach) {
                        let hdr = (b.bits[0] << 1) | b.bits[1];
                        let f1: u8 = b.bits[2..8].iter().fold(0, |a, v| a << 1 | v);
                        let f2: u8 = b.bits[8..14].iter().fold(0, |a, v| a << 1 | v);
                        let tn = b.time.map(|t| t.tn).unwrap_or(0);
                        let f18 = b.time.map(|t| t.frame == 18).unwrap_or(false);
                        *aach.entry(format!("tn{tn} f18={f18} hdr {hdr} f1 {f1} f2 {f2}")).or_insert(0u32) += 1;
                        continue;
                    }
                    if matches!(b.lchan, dsp::tetra::Lchan::Bsch) {
                        continue;
                    }
                    let full = std::env::var_os("TETRA_FULL").is_some();
                    let head: String = b.bits.iter().take(if full { 268 } else { 40 }).map(|v| char::from(b'0' + v)).collect();
                    match decode::tetra::Event::from_block(b) {
                        Some(decode::tetra::Event::Call(c)) => {
                            *tally.entry(c.name()).or_insert(0u32) += 1;
                            if shown < 60 {
                                println!("  {:?} {head} {} {:?} aie {} id {:?} from {:?} group {:?} {:?}",
                                    b.lchan, c.name(), c.address, c.aie, c.call_id, c.from, c.group, c.time);
                                shown += 1;
                            }
                        }
                        Some(decode::tetra::Event::Network(n)) => {
                            *tally.entry("D-NWRK BROADCAST").or_insert(0) += 1;
                            if shown < 60 {
                                println!("  {:?} {head} network {:?}", b.lchan, n.neighbours);
                                shown += 1;
                            }
                        }
                        Some(other) => {
                            *tally.entry(match other {
                                decode::tetra::Event::Sync(_) => "SYNC",
                                decode::tetra::Event::Sysinfo(_) => "SYSINFO",
                                _ => "?",
                            }).or_insert(0) += 1;
                        }
                        None => {
                            *tally.entry("unparsed").or_insert(0) += 1;
                            if shown < 60 {
                                println!("  {:?} {head} unparsed", b.lchan);
                                shown += 1;
                            }
                        }
                    }
                }
                println!("  tally {tally:?}");
                for (k, n) in &aach {
                    println!("  aach {k}: {n}");
                }
            }
            let mean_q = quality.iter().sum::<f32>() / quality.len().max(1) as f32;
            println!(
                "{:.4} MHz @ {:.0} S/s: {} bursts (q~{mean_q:.2}) {kinds:?}, stats {:?}, {} blocks ({} failed), cell {:?}",
                hz / 1e6,
                rate / factor as f64,
                quality.len(),
                demod.stats(),
                blocks.len(),
                rx.failed,
                rx.cell
            );
        }
    }
}
