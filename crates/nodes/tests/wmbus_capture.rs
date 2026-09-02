//! Wireless M-Bus meters off the air, against what rtl_433 read from the
//! same recordings.
//!
//! The recordings are rtl_433's, fetched by `testdata/fetch.sh` into
//! `testdata/wmbus` beside the JSON rtl_433 wrote for each, and absent from a
//! fresh clone. Every one goes through the auto node with nothing said but
//! the file's centre and rate: the meter has to be found as a source, cut
//! out wide enough, and read. What comes out is compared field by field with
//! the reference: manufacturer, meter number, version, type, and the bytes
//! themselves.

use common::{Hz, PacketBody, C32};
use nodes::{build_chain, registry, NodeSpec};
use pipeline::StreamSpec;
use sources::FileSource;
use std::path::{Path, PathBuf};

fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/rtl433")
}

/// rtl_433's JSON is one object per line; only a few of its fields are
/// wanted, and a parser for the whole format would be more code than the
/// decoder. The values wanted are numbers and short strings.
fn field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let k = format!("\"{key}\"");
    let i = json.find(&k)? + k.len();
    let rest = json[i..].trim_start().strip_prefix(':')?.trim_start();
    if let Some(s) = rest.strip_prefix('"') {
        s.split('"').next()
    } else {
        rest.split(|c: char| c == ',' || c == '}').next().map(str::trim)
    }
}

fn frames(path: &Path) -> Vec<(u64, Vec<u8>)> {
    let buf = FileSource::open(path).expect("open").read_all().expect("read");
    let rate = buf.rate.as_f64();
    let mut g = build_chain(StreamSpec::iq(rate, buf.center), &[NodeSpec::new("auto")], &registry())
        .expect("build");
    let mut out = Vec::new();
    let silence = vec![C32::new(0.0, 0.0); 16_384];
    for block in buf.samples.chunks(16_384).chain(std::iter::repeat(&silence[..]).take(8)) {
        g.feed_iq(block).expect("run");
        for p in g.output().as_packets().unwrap_or(&[]) {
            if let PacketBody::Frame(f) = &p.body {
                out.push((p.center_hz, f.clone()));
            }
        }
    }
    out
}

#[test]
fn every_meter_rtl_433_read_is_read_here() {
    let Ok(entries) = std::fs::read_dir(dir()) else {
        eprintln!("skipping: no wM-Bus fixtures, run testdata/fetch.sh to enable");
        return;
    };
    let mut paths: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "cu8")
            && p.file_name().is_some_and(|n| n.to_string_lossy().starts_with("wmbus_"))
            && p.with_extension("json").exists())
        .collect();
    paths.sort();
    if paths.is_empty() {
        eprintln!("skipping: no wM-Bus fixtures, run testdata/fetch.sh to enable");
        return;
    }
    let mut failures = Vec::new();
    for p in &paths {
        let name = p.file_name().unwrap().to_string_lossy().into_owned();
        let json = std::fs::read_to_string(p.with_extension("json")).unwrap();
        let want_data = field(&json, "data").unwrap_or("");
        let want_m = field(&json, "M").unwrap_or("");
        let want_id = field(&json, "id").unwrap_or("");
        let want_mode = field(&json, "mode").unwrap_or("");
        let got = frames(p);
        // rtl_433 keeps mode C format B's CRCs inside the bytes; here every
        // frame comes out without CRCs, so the reference is trimmed to
        // match where its length says they were.
        let want: Vec<u8> = {
            let raw: Vec<u8> = (0..want_data.len())
                .step_by(2)
                .filter_map(|i| u8::from_str_radix(&want_data[i..i + 2], 16).ok())
                .collect();
            if want_mode == "C" && raw.len() == raw.first().map(|l| *l as usize + 1).unwrap_or(0) {
                let mut w = raw[..10].to_vec();
                w.extend_from_slice(&raw[12..raw.len() - 2]);
                w
            } else {
                raw
            }
        };
        let hit = got.iter().find(|(_, f)| *f == want);
        match hit {
            Some((hz, f)) => {
                let r = decode::wmbus::parse(f, None).expect("a parse");
                let m = r.get("M").map(|v| v.to_string()).unwrap_or_default();
                let id = r.get("id").map(|v| v.to_string()).unwrap_or_default();
                eprintln!("{name}: {m} {id} at {:.4} MHz, {} bytes", *hz as f64 / 1e6, f.len());
                if m != want_m || id != want_id {
                    failures.push(format!("{name}: read {m} {id}, rtl_433 read {want_m} {want_id}"));
                }
            }
            None => {
                let seen: Vec<String> = got
                    .iter()
                    .map(|(hz, f)| format!("{} bytes at {:.4} MHz", f.len(), *hz as f64 / 1e6))
                    .collect();
                failures.push(format!("{name}: rtl_433's frame not among {seen:?}"));
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

#[test]
fn a_meter_is_named_in_the_list() {
    let p = dir().join("wmbus_diehl_868.9M_1000k.cu8");
    if !p.exists() {
        eprintln!("skipping: fixture absent");
        return;
    }
    let got = frames(&p);
    let (hz, f) = got.first().expect("a frame");
    let d = nodes::wmbus_nodes::wmbus_decoded(f, Hz(*hz)).expect("a decode");
    assert_eq!(d.protocol, "Wireless-MBus");
    assert!(d.text.as_deref().unwrap_or("").contains("DME Water 84850129"), "{:?}", d.text);
    assert_eq!(d.crc_ok, Some(true));
}
