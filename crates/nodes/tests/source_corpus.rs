//! The auto node against the bank tiers, on rtl_433's corpus.
//!
//! The banks are what the receiver ran before the auto node existed, and the
//! one claim that decides whether they can be retired is that the node hears
//! at least as much. So both run here over every capture, through the same
//! burst front end and the same protocol tables, and the count of reference
//! decodes each one recovers is printed side by side and then compared.
//!
//! The captures are not centred and not on any grid: rtl_433's corpus is
//! recorded by tuning to the nominal band frequency, so a transmitter tens of
//! kilohertz off nominal is normal. That is the case the banks needed four
//! tiers for and the case source detection exists to make ordinary.

#[path = "../../decode/tests/corpus/mod.rs"]
mod corpus;

use common::{Package, C32};
use corpus::{fixtures, Fixture};
use decode::protocol::Report;
use decode::Protocols;
use nodes::{build_chain, ism_decode_graph, registry, ChannelBank, Gating, NodeSpec};
use pipeline::StreamSpec;
use sources::FileSource;

/// The tiers the receiver ships with, as `crate::scanners::DEFAULT_WIDTHS`.
const TIER_WIDTHS: [f64; 4] = [12_500.0, 31_250.0, 125_000.0, 500_000.0];

fn load(f: &Fixture) -> common::IqBuf {
    FileSource::open(&f.path).expect("open").read_all().expect("read")
}

/// Distinct reports from a run of packages, the way the corpus harness
/// deduplicates them.
fn reports(pkgs: &[Package]) -> Vec<Report> {
    let protocols = Protocols::all();
    let mut out: Vec<Report> = Vec::new();
    for p in pkgs {
        for r in protocols.decode_all(p) {
            if !out.iter().any(|q| q.model == r.model && q.fields == r.fields) {
                out.push(r);
            }
        }
    }
    out
}

fn through_auto(buf: &common::IqBuf) -> Vec<Package> {
    let spec = StreamSpec::iq(buf.rate.as_f64(), buf.center);
    let mut g = build_chain(spec, &[NodeSpec::new("auto")], &registry()).expect("build");
    let mut out = Vec::new();
    let silence = vec![C32::new(0.0, 0.0); 16_384];
    // Silence at the end lets the last source drain its tail.
    for block in buf.samples.chunks(16_384).chain(std::iter::repeat(&silence[..]).take(4)) {
        g.feed_iq(block).expect("run");
        let pk = g.output().as_packets().unwrap_or(&[]);
        out.extend(pk.iter().filter_map(|p| p.package()));
    }
    out
}

fn through_banks(buf: &common::IqBuf) -> Vec<Package> {
    let rate = buf.rate.as_f64();
    let mut out = Vec::new();
    let mut built: Vec<usize> = Vec::new();
    for width in TIER_WIDTHS {
        let channels = nodes::BankNode::channels_for(rate, width);
        if built.contains(&channels) {
            continue;
        }
        built.push(channels);
        let mut bank = ChannelBank::new(channels, 12, rate, buf.center);
        bank.set_gating(Gating::OnDetection);
        bank.set_detector_config(nodes::ism_detector_config());
        bank.set_all_graphs(ism_decode_graph).expect("bank graphs");
        for block in buf.samples.chunks(16_384) {
            bank.process(block).expect("bank");
            out.extend_from_slice(bank.packages());
        }
        let silence = vec![C32::new(0.0, 0.0); 16_384];
        for _ in 0..4 {
            bank.process(&silence).expect("bank");
            out.extend_from_slice(bank.packages());
        }
    }
    out
}

fn hits(f: &Fixture, reports: &[Report]) -> usize {
    f.expected.iter().filter(|e| reports.iter().any(|r| e.matches(r))).count()
}

#[test]
fn the_auto_node_hears_at_least_what_the_banks_did() {
    let fixtures = fixtures();
    if fixtures.is_empty() {
        eprintln!("skipping: no rtl_433 fixtures, run testdata/fetch.sh to enable");
        return;
    }
    let (mut want, mut bank_total, mut source_total) = (0usize, 0usize, 0usize);
    let mut lost: Vec<String> = Vec::new();
    eprintln!("{:<44} {:>5} {:>5} {:>7}", "capture", "want", "banks", "auto");
    for f in &fixtures {
        let buf = load(f);
        let b = hits(f, &reports(&through_banks(&buf)));
        let s = hits(f, &reports(&through_auto(&buf)));
        let w = f.expected.len();
        eprintln!("{:<44} {:>5} {:>5} {:>7}{}", f.name, w, b, s, if s < b { "  <" } else { "" });
        want += w;
        bank_total += b;
        source_total += s;
        if s < b {
            lost.push(format!("{} ({b} -> {s})", f.name));
        }
    }
    eprintln!("{:<44} {:>5} {:>5} {:>7}", "total", want, bank_total, source_total);
    // Per capture, not in total: a gain on one recording does not excuse a
    // loss on another, since each is a device somebody owns.
    assert!(
        lost.is_empty(),
        "auto recovered {source_total} reference decodes against the banks' {bank_total}, \
         and lost ground on {lost:?}"
    );
}
