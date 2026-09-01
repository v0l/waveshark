//! Fixtures from rtl_433's own test corpus, and the mapping between its output
//! and ours.
//!
//! Three parts: finding the captures, turning one into packages the way the
//! scanner would, and translating rtl_433's JSON into something comparable
//! with a [`Report`]. The translation is explicit, per model and per field,
//! rather than a loose "fields with the same name must agree". A loose rule
//! quietly passes when a field is renamed on either side, which is precisely
//! the drift this corpus exists to catch.

#![allow(dead_code)]

use common::C32;
use decode::protocol::{Report, Value};
use decode::Protocols;
use dsp::{FirDecim, FskConfig, FskDetector, Mixer, OokDetector, Package, PulseConfig};
use sources::FileSource;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

pub fn dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../testdata/rtl433")
}

/// Every capture present, paired with the decodes rtl_433 found in it.
///
/// Missing fixtures are not an error: they are fetched by `testdata/fetch.sh`
/// and a fresh clone has none, so the tests skip instead of failing.
pub fn fixtures() -> Vec<Fixture> {
    let Ok(entries) = std::fs::read_dir(dir()) else {
        return Vec::new();
    };
    let mut paths: Vec<PathBuf> = entries
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "cu8"))
        .collect();
    paths.sort();
    paths.iter().filter_map(|p| Fixture::load(p)).collect()
}

pub struct Fixture {
    pub name: String,
    pub path: PathBuf,
    /// One entry per line of rtl_433's reference JSON, in file order, less the
    /// lines naming a model no decoder here implements.
    pub expected: Vec<Expect>,
    /// Models rtl_433 reported that we have no decoder for. Kept so the false
    /// positive check can tell "we do not decode this" from "we invented it".
    pub unsupported: Vec<String>,
    /// Every model named in the reference, in rtl_433's spelling.
    pub reference_models: Vec<String>,
}

impl Fixture {
    fn load(path: &Path) -> Option<Self> {
        let reference = std::fs::read_to_string(path.with_extension("json")).ok()?;
        let mut expected = Vec::new();
        let mut unsupported = Vec::new();
        for line in reference.lines().filter(|l| !l.trim().is_empty()) {
            let obj = parse_object(line)?;
            let model = obj.get("model")?.as_text()?;
            match spec_for(&model) {
                Some(spec) => expected.push(Expect::build(spec, &obj)),
                None if !unsupported.contains(&model) => unsupported.push(model),
                None => {}
            }
        }
        // Identical repeats say nothing extra: a burst is transmitted several
        // times and rtl_433 prints each one.
        expected.dedup_by(|a, b| a.model == b.model && a.fields == b.fields);
        let mut reference_models: Vec<String> = reference
            .lines()
            .filter(|l| !l.trim().is_empty())
            .filter_map(|l| parse_object(l)?.get("model")?.as_text())
            .collect();
        reference_models.sort();
        reference_models.dedup();
        Some(Fixture {
            name: path.file_name()?.to_string_lossy().into_owned(),
            path: path.to_path_buf(),
            expected,
            unsupported,
            reference_models,
        })
    }

    pub fn models(&self) -> &[String] {
        &self.reference_models
    }

    /// Did rtl_433 report this device, under its own name for it?
    pub fn rtl_433_saw(&self, ours: &str) -> bool {
        self.reference_models
            .iter()
            .filter_map(|m| spec_for(m))
            .any(|s| s.ours == ours)
    }

    /// Run the capture through both front ends and every protocol, exactly as
    /// the scanner does: nothing here is told which protocol to expect.
    pub fn decode(&self) -> Vec<Report> {
        let protocols = Protocols::all();
        let mut reports: Vec<Report> = Vec::new();
        for pkg in packages(&self.path) {
            for r in protocols.decode_all(&pkg) {
                if !reports.iter().any(|p| p.model == r.model && p.fields == r.fields) {
                    reports.push(r);
                }
            }
        }
        reports
    }
}

/// Turn a capture into pulse packages, using each front end at each of a few
/// bandwidths and keeping everything they produce.
///
/// Both front ends run because nothing in a recording says whether it is keyed
/// amplitude or keyed frequency, and the protocol layer cannot tell which one
/// fed it either way.
///
/// Several bandwidths run because these captures are not centred. rtl_433's
/// corpus is recorded by tuning to the nominal band frequency, so a
/// transmitter thirty or forty kilohertz off nominal is normal and a filter
/// narrow enough to match the scanner's 31.25 kHz channel deletes it. Filtering
/// still has to be tried, because a narrow channel is what lifts a weak signal
/// out of a noise floor made mostly of empty spectrum. In the live receiver the
/// channelizer resolves this by mixing each burst down to its own centre; here
/// the cheaper answer is to try the span whole as well.
pub fn packages(path: &Path) -> Vec<Package> {
    let src = FileSource::open(path).expect("open capture");
    let buf = src.read_all().expect("read capture");
    let rate = buf.rate.as_f64();

    let mut out = Vec::new();
    detect(&buf.samples, rate, &mut out);

    // Then again, burst by burst, with each one moved to the middle of the
    // span. This is the only way a narrow channel can be used on a capture
    // recorded off frequency, and most of them are: rtl_433's Bresser DM-7511
    // sits 52 kHz off, its EV1527 remote 96 kHz off. It is done per burst
    // rather than once for the file because a capture can hold two
    // transmitters at two offsets, as rtl_433's 592TXR recording does, and a
    // single estimate splits the difference and recovers neither. The whole
    // file is still tried as one, because a burst too weak to detect wideband
    // leaves no window to refine and only the file-wide estimate reaches it.
    // In the live receiver the channelizer is what does all of this.
    let offset = carrier_offset(&buf.samples, rate);
    if offset.abs() > 2_000.0 {
        let mut centred = Vec::with_capacity(buf.samples.len());
        Mixer::new(-offset, rate).process(&buf.samples, &mut centred);
        detect(&centred, rate, &mut out);
    }

    let mut refined = Vec::new();
    for (a, b) in windows(&out, rate, buf.samples.len()) {
        let burst = &buf.samples[a..b];
        let offset = carrier_offset(burst, rate);
        if offset.abs() < 2_000.0 {
            continue;
        }
        let mut centred = Vec::with_capacity(burst.len());
        Mixer::new(-offset, rate).process(burst, &mut centred);
        detect(&centred, rate, &mut refined);
    }
    out.append(&mut refined);

    out
}

/// Sample ranges covering each detected burst, merged where they overlap.
fn windows(pkgs: &[Package], rate: f64, len: usize) -> Vec<(usize, usize)> {
    let mut spans: Vec<(usize, usize)> = pkgs
        .iter()
        .map(|p| {
            let us: u64 = p.pulses.iter().map(|q| q.mark as u64 + q.gap as u64).sum();
            let dur = (us as f64 * 1e-6 * rate) as usize;
            // A margin either side, because the detector triggers partway into
            // the first mark and the estimate wants the whole burst.
            let margin = (rate * 1e-3) as usize;
            let start = (p.start_sample as usize).saturating_sub(margin);
            (start, (start + dur + 2 * margin).min(len))
        })
        .filter(|(a, b)| b > a)
        .collect();
    spans.sort();
    let mut merged: Vec<(usize, usize)> = Vec::new();
    for (a, b) in spans {
        match merged.last_mut() {
            Some(last) if a <= last.1 => last.1 = last.1.max(b),
            _ => merged.push((a, b)),
        }
    }
    merged
}

/// Mean frequency of the samples loud enough to be signal, by the average
/// phase step between consecutive ones.
///
/// An FFT would do as well and cost more; this is the standard single-tone
/// estimator, and during an OOK mark the signal *is* a single tone. On two
/// level FSK it lands midway between the tones, which is exactly where the
/// discriminator wants zero anyway.
fn carrier_offset(iq: &[C32], rate: f64) -> f64 {
    let peak = iq.iter().map(|c| c.norm_sqr()).fold(0.0f32, f32::max);
    if peak <= 0.0 {
        return 0.0;
    }
    // A tenth of the peak power, so the estimate is made of the burst and not
    // of the silence around it, which is most of a capture.
    let floor = peak * 0.1;
    let mut acc = C32::new(0.0, 0.0);
    for w in iq.windows(2) {
        if w[0].norm_sqr() > floor && w[1].norm_sqr() > floor {
            acc += w[1] * w[0].conj();
        }
    }
    if acc.norm_sqr() == 0.0 {
        return 0.0;
    }
    acc.arg() as f64 * rate / std::f64::consts::TAU
}

fn detect(iq: &[C32], rate: f64, out: &mut Vec<Package>) {
    // Two resets, because the gap that ends a transmission is per protocol and
    // nothing knows it yet at this point in the chain. The scanner's default,
    // 4 ms, keeps repeats apart, which is what the checksum-free protocols need
    // to see a frame on its own; Fine Offset's inter-symbol gaps run near 1 ms
    // and its repeats near 8, so it needs the longer one or a transmission
    // arrives in pieces. `min_pulses` is low enough for a 12 bit gate remote,
    // which is 24 pulses.
    let resets = [PulseConfig::default().reset_us, 10_000];

    // Even the undecimated pass is filtered. A factor of one still trims the
    // top tenth of the span, and on these captures that alone is the
    // difference between the EV1527 remotes detecting and not: the RTL2832U's
    // band edges are where its worst noise lives.
    for decim in [1usize, 2, 8] {
        let mut narrow = Vec::new();
        FirDecim::design(decim, 0.9, 80.0).process(iq, &mut narrow);
        let iq = narrow;
        let rate = rate / decim as f64;
        let env: Vec<f32> = iq.iter().map(|c| c.norm()).collect();
        for reset_us in resets {
            let ook = PulseConfig { reset_us, min_pulses: 8, ..Default::default() };
            OokDetector::new(rate, ook).process(&env, out);
            let fsk = FskConfig { reset_us, min_pulses: 8, ..Default::default() };
            FskDetector::new(rate, fsk).process(&iq, out);
        }
    }
}

pub fn describe(reports: &[Report]) -> String {
    if reports.is_empty() {
        return "nothing".into();
    }
    reports.iter().map(|r| r.to_string()).collect::<Vec<_>>().join("; ")
}

/// One decode rtl_433 reported, expressed in our field names and units.
#[derive(Debug)]
pub struct Expect {
    pub model: &'static str,
    pub fields: Vec<(&'static str, Want)>,
    /// The reference line this came from, for a failure message that can be
    /// checked against the corpus by eye.
    pub source: String,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Want {
    Num(f64),
    Bool(bool),
    Text(String),
    /// Our value must appear inside theirs: rtl_433 writes a control as
    /// `Down (4)` and a clock as a full timestamp, where we store the name and
    /// the time of day.
    Within(String),
}

impl Expect {
    fn build(spec: &'static ModelSpec, obj: &BTreeMap<String, Json>) -> Self {
        let mut fields = Vec::new();
        for (from, to, conv) in spec.fields {
            let Some(v) = obj.get(*from) else { continue };
            if let Some(want) = conv.apply(v) {
                fields.push((*to, want));
            }
        }
        let source = format!(
            "{{{}}}",
            obj.iter().map(|(k, v)| format!("{k}:{v}")).collect::<Vec<_>>().join(" ")
        );
        Expect { model: spec.ours, fields, source }
    }

    pub fn matches(&self, r: &Report) -> bool {
        r.model == self.model
            && self.fields.iter().all(|(k, want)| r.get(k).is_some_and(|v| want.agrees(v)))
    }
}

impl Want {
    fn agrees(&self, got: &Value) -> bool {
        match (self, got) {
            // Tenths on both sides, so a tolerance below half a tenth catches
            // a real disagreement and forgives the rounding.
            (Want::Num(a), Value::Float(b)) => (a - b).abs() < 0.04,
            (Want::Num(a), Value::Int(b)) => (a - *b as f64).abs() < 0.04,
            (Want::Bool(a), Value::Bool(b)) => a == b,
            (Want::Text(a), Value::Text(b)) => a == b,
            (Want::Within(a), Value::Text(b)) => a.contains(b.as_str()),
            _ => false,
        }
    }
}

impl std::fmt::Display for Expect {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.model)?;
        for (k, v) in &self.fields {
            match v {
                Want::Num(n) => write!(f, " {k}={n}")?,
                Want::Bool(b) => write!(f, " {k}={b}")?,
                Want::Text(s) | Want::Within(s) => write!(f, " {k}~{s}")?,
            }
        }
        write!(f, "   [rtl_433: {}]", self.source)
    }
}

/// How one rtl_433 field becomes one of ours.
#[derive(Clone, Copy)]
pub enum Conv {
    /// Same quantity, same units.
    Num,
    /// rtl_433 writes booleans as 1 and 0.
    Bool,
    Text,
    /// Our value is a substring of theirs.
    Within,
    /// Fahrenheit on the wire, Celsius in the report.
    FromF,
    /// Wind is quoted in km/h by rtl_433 and in m/s here.
    FromKmh,
}

impl Conv {
    fn apply(&self, v: &Json) -> Option<Want> {
        Some(match self {
            Conv::Num => Want::Num(v.as_num()?),
            Conv::Bool => Want::Bool(v.as_num()? != 0.0),
            Conv::Text => Want::Text(v.as_text()?),
            Conv::Within => Want::Within(v.as_text()?),
            Conv::FromF => Want::Num(((v.as_num()? - 32.0) / 1.8 * 10.0).round() / 10.0),
            Conv::FromKmh => Want::Num((v.as_num()? / 3.6 * 100.0).round() / 100.0),
        })
    }
}

pub struct ModelSpec {
    /// rtl_433's model string.
    pub rtl: &'static str,
    /// Ours, which is usually the same: the names were taken from rtl_433 so
    /// that a packet log could be compared with one.
    pub ours: &'static str,
    /// `(their field, our field, conversion)`. A field absent from the
    /// reference line is skipped, which is how one spec covers a family whose
    /// members report different things.
    pub fields: &'static [(&'static str, &'static str, Conv)],
}

pub fn spec_for(model: &str) -> Option<&'static ModelSpec> {
    SPECS.iter().find(|s| s.rtl == model)
}

/// Every Oregon Scientific thermo-hygro sensor reports the same five fields,
/// and the ones without a humidity element simply omit it on both sides.
static OREGON_TH: &[(&str, &str, Conv)] = &[
    ("id", "id", Num),
    ("channel", "channel", Num),
    ("temperature_C", "temperature_c", Num),
    ("humidity", "humidity_pct", Num),
    ("battery_ok", "battery_ok", Bool),
];

use Conv::{Bool, FromF, FromKmh, Num, Text, Within};

/// Deliberately not exhaustive over rtl_433: a model missing from here means
/// no decoder for it, and the reference lines naming it are ignored rather
/// than failed.
pub static SPECS: &[ModelSpec] = &[
    ModelSpec {
        rtl: "Fineoffset-WHx080",
        ours: "Fineoffset-WHx080",
        fields: &[
            ("id", "station_id", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("wind_dir_deg", "wind_direction_deg", Num),
            ("wind_avg_km_h", "wind_avg_ms", FromKmh),
            ("wind_max_km_h", "wind_gust_ms", FromKmh),
            ("rain_mm", "rain_total_mm", Num),
            ("battery_ok", "battery_ok", Bool),
            // The clock message, which carries a full date where we keep only
            // the time of day the sensor actually sends.
            ("radio_clock", "time", Within),
        ],
    },
    ModelSpec {
        rtl: "Fineoffset-WH51",
        ours: "Fineoffset-WH51",
        fields: &[
            ("id", "id", Text),
            ("moisture", "moisture_pct", Num),
            ("battery_mV", "battery_mv", Num),
            ("boost", "boost", Num),
            ("ad_raw", "ad_raw", Num),
        ],
    },
    ModelSpec {
        rtl: "Acurite-609TXC",
        ours: "Acurite-609TXC",
        fields: &[
            ("id", "id", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "Acurite-Tower",
        ours: "Acurite-Tower",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Text),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "Bresser-3CH",
        ours: "Bresser-3CH",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("temperature_F", "temperature_c", FromF),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "GT-WT02",
        ours: "GT-WT02",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "GT-WT03",
        ours: "GT-WT03",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "LaCrosse-TX141THBv2",
        ours: "LaCrosse-TX141THBv2",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "LaCrosse-TX29IT",
        ours: "LaCrosse-TX29IT",
        fields: &[
            ("id", "id", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
            ("newbattery", "battery_new", Bool),
        ],
    },
    ModelSpec {
        rtl: "LaCrosse-TX35DTHIT",
        ours: "LaCrosse-TX35DTHIT",
        fields: &[
            ("id", "id", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
            ("newbattery", "battery_new", Bool),
        ],
    },
    ModelSpec {
        rtl: "Nexus-TH",
        ours: "Nexus-TH",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("temperature_C", "temperature_c", Num),
            ("humidity", "humidity_pct", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "Rubicson-Temperature",
        ours: "Rubicson-Temperature",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("temperature_C", "temperature_c", Num),
            ("battery_ok", "battery_ok", Bool),
        ],
    },
    ModelSpec {
        rtl: "Oregon-THGR122N",
        ours: "Oregon-THGR122N",
        fields: OREGON_TH,
    },
    ModelSpec { rtl: "Oregon-THN132N", ours: "Oregon-THN132N", fields: OREGON_TH },
    ModelSpec { rtl: "Oregon-RTGN318", ours: "Oregon-RTGN318", fields: OREGON_TH },
    ModelSpec { rtl: "Oregon-RTHN129", ours: "Oregon-RTHN129", fields: OREGON_TH },
    ModelSpec { rtl: "Oregon-THN129", ours: "Oregon-THN129", fields: OREGON_TH },
    ModelSpec {
        rtl: "Oregon-WGR800",
        ours: "Oregon-WGR800",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("battery_ok", "battery_ok", Bool),
            ("wind_max_m_s", "wind_gust_ms", Num),
            ("wind_avg_m_s", "wind_avg_ms", Num),
            ("wind_dir_deg", "wind_direction_deg", Num),
        ],
    },
    ModelSpec {
        rtl: "Honeywell-Security",
        ours: "Honeywell-Security",
        fields: &[
            ("id", "id", Num),
            ("channel", "channel", Num),
            ("event", "event", Num),
            ("state", "state", Text),
            ("contact_open", "contact_open", Bool),
            ("reed_open", "reed_open", Bool),
            ("alarm", "alarm", Bool),
            ("tamper", "tamper", Bool),
            ("battery_ok", "battery_ok", Bool),
            ("heartbeat", "heartbeat", Bool),
        ],
    },
    ModelSpec {
        rtl: "Schrader",
        ours: "Schrader",
        fields: &[
            ("id", "id", Text),
            ("flags", "flags", Text),
            ("pressure_kPa", "pressure_kpa", Num),
            ("temperature_C", "temperature_c", Num),
        ],
    },
    ModelSpec {
        rtl: "Toyota",
        ours: "Toyota",
        fields: &[
            ("id", "id", Text),
            ("status", "status", Num),
            ("pressure_PSI", "pressure_psi", Num),
            ("temperature_C", "temperature_c", Num),
        ],
    },
    ModelSpec {
        rtl: "Somfy-RTS",
        ours: "Somfy-RTS",
        fields: &[
            ("id", "id", Num),
            ("control", "control", Within),
            ("counter", "counter", Num),
        ],
    },
    ModelSpec {
        rtl: "X10-RF",
        ours: "X10-RF",
        fields: &[
            ("id", "unit", Num),
            ("channel", "channel", Text),
            ("state", "state", Text),
        ],
    },
    ModelSpec {
        rtl: "Generic-Remote",
        ours: "Generic-Remote",
        fields: &[("id", "id", Num), ("cmd", "cmd", Num), ("tristate", "tristate", Text)],
    },
];

// ---------------------------------------------------------------------------
// A JSON reader for exactly what rtl_433 emits: one flat object per line, with
// string, number and boolean values. Hand written because the workspace has no
// JSON dependency and this does not justify adding one.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq)]
pub enum Json {
    Text(String),
    Num(f64),
    Bool(bool),
}

impl Json {
    fn as_num(&self) -> Option<f64> {
        match self {
            Json::Num(v) => Some(*v),
            Json::Bool(b) => Some(*b as u8 as f64),
            Json::Text(_) => None,
        }
    }

    fn as_text(&self) -> Option<String> {
        match self {
            Json::Text(s) => Some(s.clone()),
            _ => None,
        }
    }
}

impl std::fmt::Display for Json {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Json::Text(s) => write!(f, "{s}"),
            Json::Num(v) => write!(f, "{v}"),
            Json::Bool(b) => write!(f, "{b}"),
        }
    }
}

pub fn parse_object(line: &str) -> Option<BTreeMap<String, Json>> {
    let mut chars = line.trim().chars().peekable();
    if chars.next()? != '{' {
        return None;
    }
    let mut out = BTreeMap::new();
    loop {
        skip_space(&mut chars);
        match chars.peek()? {
            '}' => return Some(out),
            ',' => {
                chars.next();
                continue;
            }
            '"' => {}
            _ => return None,
        }
        let key = read_string(&mut chars)?;
        skip_space(&mut chars);
        if chars.next()? != ':' {
            return None;
        }
        skip_space(&mut chars);
        let value = match chars.peek()? {
            '"' => Json::Text(read_string(&mut chars)?),
            't' | 'f' => {
                let word = read_word(&mut chars);
                Json::Bool(word == "true")
            }
            _ => Json::Num(read_word(&mut chars).parse().ok()?),
        };
        out.insert(key, value);
    }
}

fn skip_space(chars: &mut std::iter::Peekable<std::str::Chars>) {
    while chars.peek().is_some_and(|c| c.is_whitespace()) {
        chars.next();
    }
}

fn read_string(chars: &mut std::iter::Peekable<std::str::Chars>) -> Option<String> {
    if chars.next()? != '"' {
        return None;
    }
    let mut s = String::new();
    loop {
        match chars.next()? {
            '"' => return Some(s),
            '\\' => s.push(chars.next()?),
            c => s.push(c),
        }
    }
}

fn read_word(chars: &mut std::iter::Peekable<std::str::Chars>) -> String {
    let mut s = String::new();
    while let Some(c) = chars.peek() {
        if *c == ',' || *c == '}' || c.is_whitespace() {
            break;
        }
        s.push(*c);
        chars.next();
    }
    s
}
