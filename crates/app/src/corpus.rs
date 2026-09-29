use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use common::Modulation;
use common::packet::{Fact, Integrity};
use serde::Deserialize;

use crate::row::Reception;

#[derive(Deserialize)]
struct Manifest {
    capture: Vec<Capture>,
}

#[derive(Deserialize)]
struct Capture {
    name: String,
    rows: Option<usize>,
    scanners: Option<String>,
    channels: Option<String>,
    #[serde(default)]
    read: Vec<Read>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Read {
    protocol: String,
    kind: Option<String>,
    on_hz: Option<f64>,
    rows: Option<usize>,
    rows_min: Option<usize>,
    rows_max: Option<usize>,
    checked: Option<bool>,
    modulation: Option<String>,
    hz: Option<f64>,
    #[serde(default)]
    hz_within: f64,
    channel: Option<u16>,
    subjects: Option<Vec<String>>,
    from: Option<Vec<String>>,
    to: Option<Vec<String>>,
    wrote: Option<Vec<String>>,
    spreading: Option<u8>,
    bandwidth_hz: Option<f32>,
    snr_min_db: Option<f32>,
    says: Option<Vec<String>>,
    sensed: Option<BTreeMap<String, f64>>,
}

fn testdata() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata")
}

fn replay(buf: &common::IqBuf, scanners: Option<&str>, channels: Option<&str>) -> Vec<Reception> {
    let table = match scanners {
        Some(added) => {
            crate::scanners::Scanners::parse(&format!("{}\n{added}", crate::scanners::DEFAULT_TEXT))
        }
        None => crate::scanners::Scanners::default(),
    };
    let mut plan = crate::radio::replay_plan(buf, false);
    plan.fronts = table.fronts(crate::scanners::Span::new(buf.center.as_f64(), buf.rate.as_f64()));
    if let Some(bank) = channels {
        let tuned: Vec<_> = crate::memory::Memory::parse(bank)
            .list
            .iter()
            .zip(1..)
            .map(|(saved, id)| crate::ui::recalled(id, saved))
            .collect();
        plan.channels = crate::ui::specs_of(&tuned, buf.center.as_f64());
    }
    let mut rx = crate::chain::Receiver::build(&plan, crate::chain::Sinks::default())
        .expect("a receiver for the capture");
    crate::radio::replay_blocks(&mut rx, buf)
}

fn distinct(it: impl Iterator<Item = Option<String>>) -> Vec<String> {
    it.flatten().collect::<BTreeSet<_>>().into_iter().collect()
}

fn channel(r: &Reception) -> Option<u16> {
    r.packet.facts().find_map(|(_, f)| match f {
        Fact::Channel(c) => Some(c.claims.unwrap_or(c.heard)),
        _ => None,
    })
}

fn sensed(r: &Reception, label: &str) -> Option<f64> {
    r.packet.facts().find_map(|(_, f)| match f {
        Fact::Sensed(x) if x.quantity.label() == label => Some(x.value),
        _ => None,
    })
}

fn check(read: &Read, all: &[Reception]) -> Vec<String> {
    let rows: Vec<&Reception> = all
        .iter()
        .filter(|r| r.protocol() == read.protocol)
        .filter(|r| read.kind.as_deref().is_none_or(|k| r.kind() == k))
        .filter(|r| read.on_hz.is_none_or(|hz| r.freq() == hz))
        .collect();
    let mut bad = Vec::new();
    let n = rows.len();

    if let Some(want) = read.rows
        && n != want
    {
        bad.push(format!("read {n} rows, expected {want}"));
    }
    if let Some(floor) = read.rows_min
        && n < floor
    {
        bad.push(format!("read {n} rows, under the floor of {floor}"));
    }
    if let Some(ceiling) = read.rows_max
        && n > ceiling
    {
        bad.push(format!("read {n} rows, over the ceiling of {ceiling}"));
    }
    if n == 0 {
        return bad;
    }

    if read.checked == Some(true) {
        let passed = rows.iter().filter(|r| r.integrity() == Integrity::Passed).count();
        if passed != n {
            bad.push(format!("{passed} of {n} rows passed their check"));
        }
    }
    if let Some(label) = &read.modulation {
        match Modulation::parse(label) {
            None => bad.push(format!("{label} is not a modulation")),
            Some(m) => {
                let got: BTreeSet<String> =
                    rows.iter().map(|r| r.modulation().to_string()).collect();
                if rows.iter().any(|r| r.modulation() != m) {
                    bad.push(format!("keyed as {got:?}, expected {m}"));
                }
            }
        }
    }
    if let Some(hz) = read.hz {
        let off: BTreeSet<u64> = rows
            .iter()
            .filter(|r| (r.freq() - hz).abs() > read.hz_within)
            .map(|r| r.freq() as u64)
            .collect();
        if !off.is_empty() {
            bad.push(format!("read at {off:?} Hz, expected {hz} within {}", read.hz_within));
        }
    }
    if let Some(want) = read.channel {
        let got: BTreeSet<Option<u16>> = rows.iter().map(|r| channel(r)).collect();
        if got != BTreeSet::from([Some(want)]) {
            bad.push(format!("on channels {got:?}, expected {want}"));
        }
    }

    let party = |r: &Reception, from: bool| {
        let link = &r.packet.innermost()?.link;
        let end = if from { link.from.as_ref() } else { link.to.as_ref() };
        end.map(|p| p.label().to_string())
    };
    let sets = [
        (
            "subjects",
            &read.subjects,
            distinct(rows.iter().map(|r| r.packet.subject().map(|e| e.id.to_string()))),
        ),
        ("from", &read.from, distinct(rows.iter().map(|r| party(r, true)))),
        ("to", &read.to, distinct(rows.iter().map(|r| party(r, false)))),
        (
            "wrote",
            &read.wrote,
            distinct(rows.iter().map(|r| r.packet.innermost()?.wrote().map(str::to_string))),
        ),
    ];
    for (what, want, got) in sets {
        if let Some(want) = want {
            let mut want = want.clone();
            want.sort();
            if got != want {
                bad.push(format!("{what} {got:?}, expected {want:?}"));
            }
        }
    }

    let keyed = |r: &Reception| r.packet.keying.as_ref().map(|k| k.params);
    if let Some(sf) = read.spreading {
        let got: BTreeSet<Option<u8>> =
            rows.iter().map(|r| keyed(r).and_then(|p| p.spreading)).collect();
        if got != BTreeSet::from([Some(sf)]) {
            bad.push(format!("spreading factors {got:?}, expected {sf}"));
        }
    }
    if let Some(bw) = read.bandwidth_hz {
        let got: BTreeSet<u32> =
            rows.iter().map(|r| keyed(r).map_or(0, |p| p.bandwidth_hz as u32)).collect();
        if got != BTreeSet::from([bw as u32]) {
            bad.push(format!("keyed {got:?} Hz wide, expected {bw}"));
        }
    }
    if let Some(floor) = read.snr_min_db {
        let low = rows.iter().map(|r| r.snr_db()).fold(f32::INFINITY, f32::min);
        if low < floor {
            bad.push(format!("a row at {low:.1} dB SNR, under the floor of {floor}"));
        }
    }
    for text in read.says.iter().flatten() {
        let without = rows.iter().filter(|r| !r.detail().contains(text.as_str())).count();
        if without > 0 {
            bad.push(format!("{without} of {n} rows do not say {text:?}"));
        }
    }
    for (label, want) in read.sensed.iter().flatten() {
        let got: Vec<Option<f64>> = rows.iter().map(|r| sensed(r, label)).collect();
        if got.iter().any(|v| v.is_none_or(|v| (v - want).abs() > 1e-6)) {
            bad.push(format!("{label} read as {got:?}, expected {want}"));
        }
    }

    for r in &rows {
        let iq = r.packet.carrier.iq.as_ref().filter(|q| !q.samples.is_empty());
        if !r.rssi_dbfs().is_finite() || !r.snr_db().is_finite() || iq.is_none() {
            bad.push(format!(
                "a row without its measurements: rssi {}, snr {}, samples {}",
                r.rssi_dbfs(),
                r.snr_db(),
                iq.map_or(0, |q| q.samples.len())
            ));
            break;
        }
    }
    bad
}

#[test]
fn every_capture_in_decode_toml_reads_as_it_says() {
    let _installing = decode::script::test_lock();
    if !decode::script::install_fetched() {
        return;
    }
    let text = std::fs::read_to_string(testdata().join("decode.toml")).expect("decode.toml");
    let manifest: Manifest = toml::from_str(&text).expect("decode.toml parses");

    let mut read = 0;
    let mut failures = Vec::new();
    for cap in &manifest.capture {
        let path = testdata().join(&cap.name);
        if !path.exists() {
            eprintln!("skipping: {} absent, run testdata/fetch.sh", cap.name);
            continue;
        }
        let buf = sources::FileSource::open(&path)
            .and_then(|s| s.read_all())
            .unwrap_or_else(|e| panic!("{}: {e}", cap.name));
        let out = replay(&buf, cap.scanners.as_deref(), cap.channels.as_deref());
        read += 1;

        if let Some(want) = cap.rows
            && out.len() != want
        {
            failures.push(format!("{}: {} rows in all, expected {want}", cap.name, out.len()));
        }
        for r in &cap.read {
            let mut what = r.protocol.clone();
            if let Some(k) = &r.kind {
                what += &format!(" {k}");
            }
            if let Some(hz) = r.on_hz {
                what += &format!(" on {hz} Hz");
            }
            for bad in check(r, &out) {
                failures.push(format!("{}: {what}: {bad}", cap.name));
            }
        }
    }
    if read == 0 {
        eprintln!("skipping: no capture in decode.toml present, run testdata/fetch.sh");
    }
    assert!(failures.is_empty(), "\n  {}", failures.join("\n  "));
}
