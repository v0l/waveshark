use crate::file::{FileMeta, parse_filename};
use common::{Error, Hz, Result, SampleFormat, Sps};
use serde_json::{Map, Value, json};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::ops::Range;
use std::path::{Path, PathBuf};

pub const META: &str = "sigmf-meta";
pub const DATA: &str = "sigmf-data";
pub const ARCHIVE: &str = "sigmf";

const VERSION: &str = "1.2.0";
const NAMESPACE: &str = "waveshark";

#[derive(Clone, Debug, PartialEq)]
pub struct Located {
    pub data: PathBuf,
    pub bytes: Range<u64>,
    pub meta: FileMeta,
}

impl Located {
    pub fn samples(&self, format: SampleFormat) -> u64 {
        (self.bytes.end - self.bytes.start) / format.bytes_per_sample() as u64
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Kind {
    Archive,
    Meta,
    Samples,
}

impl Kind {
    fn of(path: &Path) -> Self {
        match path.extension().and_then(|e| e.to_str()) {
            Some(ARCHIVE) => Self::Archive,
            Some(META) => Self::Meta,
            _ => Self::Samples,
        }
    }
}

pub fn locate(path: &Path) -> Result<Located> {
    match Kind::of(path) {
        Kind::Archive => archive(path),
        Kind::Meta => {
            let described = parse(&std::fs::read_to_string(path)?, path)?;
            let data = match &described.dataset {
                Some(name) => path.with_file_name(name),
                None => path.with_extension(DATA),
            };
            let len = std::fs::metadata(&data)?.len();
            Ok(Located { meta: parse_filename(&data).under(described.meta), data, bytes: 0..len })
        }
        Kind::Samples => {
            let len = std::fs::metadata(path)?.len();
            let named = parse_filename(path);
            let beside = path.with_extension(META);
            let meta = match std::fs::read_to_string(&beside) {
                Ok(text) => {
                    let described = parse(&text, &beside)?;
                    let mine = described
                        .dataset
                        .as_deref()
                        .is_none_or(|d| Path::new(d).file_name() == path.file_name());
                    match mine {
                        true => named.under(described.meta),
                        false => named,
                    }
                }
                Err(_) => named,
            };
            Ok(Located { data: path.to_path_buf(), bytes: 0..len, meta })
        }
    }
}

struct Described {
    meta: FileMeta,
    dataset: Option<String>,
}

fn parse(text: &str, path: &Path) -> Result<Described> {
    let v: Value = serde_json::from_str(text)
        .map_err(|e| Error::other(format!("{} is not SigMF metadata: {e}", path.display())))?;
    let global = &v["global"];
    let datatype = global["core:datatype"]
        .as_str()
        .ok_or_else(|| Error::other(format!("{} has no core:datatype", path.display())))?;
    let format = SampleFormat::from_sigmf_datatype(datatype).ok_or_else(|| {
        let readable: Vec<&str> = SampleFormat::ALL.iter().map(|f| f.sigmf_datatype()).collect();
        Error::other(format!(
            "{} holds {datatype} samples, and this receiver reads {}",
            path.display(),
            readable.join(", ")
        ))
    })?;
    let channels = global["core:num_channels"].as_u64().unwrap_or(1);
    if channels != 1 {
        return Err(Error::other(format!(
            "{} interleaves {channels} channels, and this receiver reads one",
            path.display()
        )));
    }
    let rate =
        global["core:sample_rate"].as_f64().filter(|r| *r >= 1.0).map(|r| Sps(r.round() as u64));
    let center = v["captures"][0]["core:frequency"]
        .as_f64()
        .filter(|f| *f > 0.0)
        .map(|f| Hz(f.round() as u64));
    let dataset = global["core:dataset"].as_str().map(str::to_string);
    Ok(Described { meta: FileMeta { center, rate, format: Some(format) }, dataset })
}

fn archive(path: &Path) -> Result<Located> {
    let mut f = File::open(path)?;
    let members = tar_members(&mut f)?;
    let (meta_name, meta_bytes) = members
        .iter()
        .find(|(name, _)| name.ends_with(&format!(".{META}")))
        .ok_or_else(|| Error::other(format!("{} holds no .{META}", path.display())))?;
    let mut text = vec![0u8; (meta_bytes.end - meta_bytes.start) as usize];
    f.seek(SeekFrom::Start(meta_bytes.start))?;
    f.read_exact(&mut text)?;
    let text = String::from_utf8(text)
        .map_err(|_| Error::other(format!("{meta_name} in {} is not text", path.display())))?;
    let described = parse(&text, Path::new(meta_name))?;
    let stem = &meta_name[..meta_name.len() - META.len()];
    let data_name = format!("{stem}{DATA}");
    let (_, bytes) = members
        .iter()
        .find(|(name, _)| *name == data_name)
        .ok_or_else(|| Error::other(format!("{} holds no {data_name}", path.display())))?;
    Ok(Located { data: path.to_path_buf(), bytes: bytes.clone(), meta: described.meta })
}

fn tar_members(f: &mut File) -> Result<Vec<(String, Range<u64>)>> {
    const BLOCK: u64 = 512;
    let end = f.metadata()?.len();
    let mut members = Vec::new();
    let mut pos = 0u64;
    let mut long_name: Option<String> = None;
    let mut header = [0u8; BLOCK as usize];
    while pos + BLOCK <= end {
        f.seek(SeekFrom::Start(pos))?;
        f.read_exact(&mut header)?;
        if header.iter().all(|b| *b == 0) {
            break;
        }
        let size = octal(&header[124..136])
            .ok_or_else(|| Error::other("a tar header with no size".to_string()))?;
        let start = pos + BLOCK;
        let body = start..start + size;
        if body.end > end {
            return Err(Error::other("a tar member runs past the end of the archive".to_string()));
        }
        match header[156] {
            b'x' => {
                let mut records = vec![0u8; size as usize];
                f.read_exact(&mut records)?;
                long_name = pax_path(&records);
            }
            b'L' => {
                let mut name = vec![0u8; size as usize];
                f.read_exact(&mut name)?;
                long_name = Some(cstr(&name));
            }
            b'0' | 0 => {
                let name = long_name.take().unwrap_or_else(|| {
                    let name = cstr(&header[0..100]);
                    match &header[257..262] == b"ustar" && header[345] != 0 {
                        true => format!("{}/{name}", cstr(&header[345..500])),
                        false => name,
                    }
                });
                members.push((name, body));
            }
            _ => long_name = None,
        }
        pos = start + size.div_ceil(BLOCK) * BLOCK;
    }
    Ok(members)
}

fn cstr(b: &[u8]) -> String {
    let n = b.iter().position(|c| *c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..n]).into_owned()
}

fn octal(b: &[u8]) -> Option<u64> {
    let s = cstr(b);
    u64::from_str_radix(s.trim(), 8).ok()
}

fn pax_path(records: &[u8]) -> Option<String> {
    let text = String::from_utf8_lossy(records);
    text.lines().find_map(|line| {
        let (_, record) = line.split_once(' ')?;
        record.strip_prefix("path=").map(str::to_string)
    })
}

#[derive(Clone, Debug)]
pub struct Recording {
    pub format: SampleFormat,
    pub rate: f64,
    pub center: f64,
    pub at_us: Option<u64>,
    pub annotations: Vec<Annotation>,
}

#[derive(Clone, Debug, Default)]
pub struct Annotation {
    pub start: u64,
    pub count: u64,
    pub label: String,
    pub edges: Option<(f64, f64)>,
    pub fields: Map<String, Value>,
}

impl Recording {
    pub fn new(format: SampleFormat, rate: f64, center: f64) -> Self {
        Self { format, rate, center, at_us: None, annotations: Vec::new() }
    }

    pub fn at(mut self, at_us: u64) -> Self {
        self.at_us = Some(at_us);
        self
    }

    pub fn annotated(mut self, a: Annotation) -> Self {
        self.annotations.push(a);
        self
    }

    pub fn to_json(&self, data: &Path) -> Value {
        let mut global = Map::new();
        global.insert("core:datatype".into(), json!(self.format.sigmf_datatype()));
        global.insert("core:sample_rate".into(), json!(self.rate));
        global.insert("core:version".into(), json!(VERSION));
        global.insert(
            "core:recorder".into(),
            json!(format!("waveshark {}", env!("CARGO_PKG_VERSION"))),
        );
        if data.extension().and_then(|e| e.to_str()) != Some(DATA)
            && let Some(name) = data.file_name().and_then(|n| n.to_str())
        {
            global.insert("core:dataset".into(), json!(name));
        }
        if self.annotations.iter().any(|a| !a.fields.is_empty()) {
            global.insert(
                "core:extensions".into(),
                json!([{ "name": NAMESPACE, "version": "1.0.0", "optional": true }]),
            );
        }
        let mut capture = Map::new();
        capture.insert("core:sample_start".into(), json!(0));
        capture.insert("core:frequency".into(), json!(self.center));
        if let Some(at) = self.at_us.and_then(datetime) {
            capture.insert("core:datetime".into(), json!(at));
        }
        let annotations: Vec<Value> = self
            .annotations
            .iter()
            .map(|a| {
                let mut m = Map::new();
                m.insert("core:sample_start".into(), json!(a.start));
                m.insert("core:sample_count".into(), json!(a.count));
                if let Some((lo, hi)) = a.edges {
                    m.insert("core:freq_lower_edge".into(), json!(lo));
                    m.insert("core:freq_upper_edge".into(), json!(hi));
                }
                m.insert("core:label".into(), json!(a.label));
                for (k, v) in &a.fields {
                    m.insert(format!("{NAMESPACE}:{k}"), v.clone());
                }
                Value::Object(m)
            })
            .collect();
        json!({ "global": global, "captures": [capture], "annotations": annotations })
    }

    pub fn write_beside(&self, data: &Path) -> Result<PathBuf> {
        let path = data.with_extension(META);
        let text = serde_json::to_string_pretty(&self.to_json(data))
            .map_err(|e| Error::other(e.to_string()))?;
        std::fs::write(&path, text)?;
        Ok(path)
    }
}

fn datetime(at_us: u64) -> Option<String> {
    let at = chrono::DateTime::from_timestamp(
        (at_us / 1_000_000) as i64,
        (at_us % 1_000_000) as u32 * 1_000,
    )?;
    Some(at.format("%Y-%m-%dT%H:%M:%S%.6fZ").to_string())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FileSource;
    use common::device::Device;

    fn dir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("sr-sigmf-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    const PYTHON_TONE_META: &str = r#"{
    "global": {
        "core:datatype": "ci16_le",
        "core:description": "a tone an eighth of the rate above the centre",
        "core:num_channels": 1,
        "core:offset": 0,
        "core:sample_rate": 2400000,
        "core:sha512": "7d0a495cb433a24515ea8ec2bae0737dc0ecb1dd7fb18967da5be924b3f7ca0da95426596992fe2b5c90f08ba846d3e84ac462d9721d801425f3d18c7447c537",
        "core:version": "1.2.6"
    },
    "captures": [
        {
            "core:datetime": "2026-09-28T12:00:00.000Z",
            "core:frequency": 433920000,
            "core:sample_start": 0
        }
    ],
    "annotations": [
        {
            "core:freq_lower_edge": 434220000,
            "core:freq_upper_edge": 434220000,
            "core:label": "tone",
            "core:sample_count": 32,
            "core:sample_start": 16
        }
    ]
}"#;

    const PYTHON_TONE_ARCHIVE: &[u8] = include_bytes!("../testdata/tone_sigmf-python-1.13.0.sigmf");

    fn python_tone() -> Vec<u8> {
        let mut raw = Vec::new();
        for k in 0..64 {
            let p = std::f64::consts::TAU * 0.125 * k as f64;
            raw.extend_from_slice(&((p.cos() * 16384.0).round() as i16).to_le_bytes());
            raw.extend_from_slice(&((p.sin() * 16384.0).round() as i16).to_le_bytes());
        }
        raw
    }

    fn assert_python_tone(src: &FileSource, how: &str) {
        assert_eq!(src.rate(), Sps(2_400_000), "{how}: sigmf 1.13.0 wrote 2400000");
        assert_eq!(src.center(), Hz(433_920_000), "{how}");
        assert_eq!(src.info().native_format, SampleFormat::Cs16, "{how}");
        let buf = src.read_all().unwrap();
        assert_eq!(buf.len(), 64, "{how}: 64 samples were written");
        let turn = (buf.samples[1] * buf.samples[0].conj()).arg();
        assert!((turn - std::f32::consts::FRAC_PI_4).abs() < 1e-3, "{how}: turned {turn} a sample");
        assert!((buf.samples[0].re - 0.5).abs() < 1e-4, "{how}: first sample {}", buf.samples[0]);
    }

    #[test]
    fn a_recording_from_the_sigmf_python_package_plays_at_its_rate_and_centre() {
        let d = dir("python");
        std::fs::write(d.join("tone.sigmf-data"), python_tone()).unwrap();
        std::fs::write(d.join("tone.sigmf-meta"), PYTHON_TONE_META).unwrap();
        std::fs::write(d.join("tone.sigmf"), PYTHON_TONE_ARCHIVE).unwrap();

        for name in ["tone.sigmf-data", "tone.sigmf-meta", "tone.sigmf"] {
            assert_python_tone(&FileSource::open(d.join(name)).unwrap(), name);
        }

        let mut rx =
            FileSource::open(d.join("tone.sigmf")).unwrap().with_block(20).start_rx().unwrap();
        let lens: Vec<usize> = std::iter::from_fn(|| rx.read().ok()).map(|b| b.len()).collect();
        assert_eq!(lens, [20, 20, 20, 4], "streamed, the archive is its 64 samples");
        let mut rx = FileSource::open(d.join("tone.sigmf"))
            .unwrap()
            .with_block(48)
            .repeating(true)
            .start_rx()
            .unwrap();
        let lens: Vec<usize> = (0..3).map(|_| rx.read().unwrap().len()).collect();
        assert_eq!(lens, [48, 16, 48], "a repeat starts again at the samples");

        let archived = locate(&d.join("tone.sigmf")).unwrap();
        assert_eq!(
            archived.bytes,
            3072..3328,
            "where Python 3 tarfile says tone/tone.sigmf-data is"
        );
        assert_eq!(archived.samples(SampleFormat::Cs16), 64);

        let skip_s = 16.0 / 2.4e6;
        let cut = crate::clip_file(
            &d.join("tone.sigmf"),
            &d.join("cut.cs16"),
            &crate::Cut::Whole { skip_s },
        )
        .unwrap();
        assert_eq!((cut.total, cut.kept), (64, 48), "the archive's samples, not its headers");
        assert_eq!(std::fs::read(d.join("cut.cs16")).unwrap(), python_tone()[16 * 4..]);
        std::fs::remove_dir_all(&d).unwrap();
    }

    #[test]
    fn a_datatype_the_receiver_cannot_read_is_refused_by_name() {
        let d = dir("refused");
        let data = d.join("be.sigmf-data");
        std::fs::write(&data, [0u8; 256]).unwrap();
        for (datatype, why) in [
            (
                "ci16_be",
                "holds ci16_be samples, and this receiver reads cu8, ci8, ci16_le, cf32_le",
            ),
            ("rf32_le", "holds rf32_le samples"),
            ("cf64_le", "holds cf64_le samples"),
        ] {
            let meta = PYTHON_TONE_META.replace("ci16_le", datatype);
            std::fs::write(d.join("be.sigmf-meta"), meta).unwrap();
            let err = FileSource::open(&data).unwrap_err().to_string();
            assert!(err.contains(why), "{datatype}: {err}");
        }
        let two = PYTHON_TONE_META.replace("\"core:num_channels\": 1", "\"core:num_channels\": 2");
        std::fs::write(d.join("be.sigmf-meta"), two).unwrap();
        let err = FileSource::open(&data).unwrap_err().to_string();
        assert!(err.contains("interleaves 2 channels"), "{err}");

        std::fs::remove_file(d.join("be.sigmf-meta")).unwrap();
        let err = FileSource::open(&data).unwrap_err().to_string();
        assert!(err.contains("sample format"), "data with no metadata is not cs16: {err}");
        std::fs::remove_dir_all(&d).unwrap();
    }

    #[test]
    fn every_format_written_is_read_back_as_itself() {
        let d = dir("roundtrip");
        let iq: Vec<common::C32> = (0..500)
            .map(|k| {
                let p = std::f32::consts::TAU * 0.07 * k as f32;
                common::C32::new(p.cos() * 0.5, p.sin() * 0.5)
            })
            .collect();
        for format in SampleFormat::ALL {
            let data = d.join(format!("capture.{}", format.extension()));
            let mut raw = Vec::new();
            format.encode(&iq, &mut raw);
            std::fs::write(&data, &raw).unwrap();
            let meta = Recording::new(format, 1_024_000.0, 868_300_000.0)
                .at(1_790_000_000_123_456)
                .write_beside(&data)
                .unwrap();
            assert_eq!(meta, d.join("capture.sigmf-meta"));
            for opened in [&data, &meta] {
                let src = FileSource::open(opened).unwrap();
                assert_eq!(src.rate(), Sps(1_024_000), "{format:?} through {}", opened.display());
                assert_eq!(src.center(), Hz(868_300_000), "{format:?}");
                assert_eq!(src.info().native_format, format);
                let back = src.read_all().unwrap();
                assert_eq!(back.len(), 500, "{format:?}");
                let worst =
                    iq.iter().zip(&back.samples).map(|(a, b)| (a - b).norm()).fold(0.0, f32::max);
                assert!(worst < 0.01, "{format:?} off by {worst}");
            }
            let v: Value = serde_json::from_str(&std::fs::read_to_string(&meta).unwrap()).unwrap();
            assert_eq!(
                v["global"]["core:dataset"],
                json!(format!("capture.{}", format.extension()))
            );
            assert_eq!(v["global"]["core:datatype"], json!(format.sigmf_datatype()));
            assert_eq!(v["captures"][0]["core:datetime"], json!("2026-09-21T14:13:20.123456Z"));
        }
        std::fs::remove_dir_all(&d).unwrap();
    }

    #[test]
    fn metadata_describing_another_dataset_says_nothing_about_this_one() {
        let d = dir("other");
        let mine = d.join("x_433.92M_250k.cu8");
        std::fs::write(&mine, [128u8; 64]).unwrap();
        std::fs::write(d.join("x_433.92M_250k.cs8"), [0u8; 64]).unwrap();
        Recording::new(SampleFormat::Cs8, 2_000_000.0, 915e6)
            .write_beside(&d.join("x_433.92M_250k.cs8"))
            .unwrap();
        let named = locate(&mine).unwrap();
        assert_eq!(named.meta, parse_filename(&mine), "the metadata names the .cs8");
        std::fs::remove_dir_all(&d).unwrap();
    }

    #[test]
    fn an_annotation_carries_its_fields_under_the_waveshark_namespace() {
        let mut fields = Map::new();
        fields.insert("snr_db".into(), json!(21.5));
        let v = Recording::new(SampleFormat::Cu8, 250e3, 868.3e6)
            .annotated(Annotation {
                start: 0,
                count: 4096,
                label: "fineoffset".into(),
                edges: Some((868.29e6, 868.31e6)),
                fields,
            })
            .to_json(Path::new("g0001.cu8"));
        let a = &v["annotations"][0];
        assert_eq!(a["core:label"], json!("fineoffset"));
        assert_eq!(a["core:sample_count"], json!(4096));
        assert_eq!(a["waveshark:snr_db"], json!(21.5));
        assert_eq!(v["global"]["core:extensions"][0]["name"], json!("waveshark"));
        assert_eq!(v["annotations"].as_array().unwrap().len(), 1);
    }
}
