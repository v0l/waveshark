//! The Signal Identification Wiki, in both its halves: the identified
//! signals as Artemis packages them, and the unidentified ones the wiki
//! lists for anybody to recognise.
//!
//! sigidwiki.com is the one crowd-sourced catalogue of what waveforms look
//! like. It is a Semantic MediaWiki with no dump, so the identified half
//! comes from Artemis (github.com/AresValley/Artemis-DB), whose crawler
//! publishes a release tar with a SQLite file at the front and 300 MB of
//! waterfalls and audio behind it. [`Artemis`] reads the tar until the
//! database has gone by and closes the connection; the media stays where it
//! is. The unidentified half has no such package and is small enough to ask
//! the wiki's `ask` API for directly.
//!
//! What a match here means: a burst measured at a centre, a width and a
//! keying is compared against entries that say "868 MHz, FSK, 20 kHz". The
//! wiki's numbers are as coarse as its contributors typed them, a frequency
//! list is sometimes a range and sometimes a set of channels, and nothing
//! says which. So [`Db::matches`] ranks rather than decides, and the score is
//! a hint about where to look, not an identification.

use crate::cache::{Cache, Error, Fetch, Seen, Source, When};
use std::io::{Read, Write};
use std::path::Path;
use std::time::Duration;

/// Artemis cuts a release every few weeks; the wiki changes daily but
/// slowly. A week between checks is one API call nobody notices.
const MAX_AGE: Duration = Duration::from_secs(7 * 24 * 3600);

const RELEASES: &str = "https://api.github.com/repos/AresValley/Artemis-DB/releases/latest";

/// The unidentified half, as the wiki's own database page queries it.
const UNID_URL: &str = "https://www.sigidwiki.com/api.php?action=ask&format=json&query=\
%5B%5BCategory%3AUNID%5D%5D%7C%3FFrequencies%23-auto%7C%3FBandwidth%20min%23-auto\
%7C%3FBandwidth%20max%23-auto%7C%3FModulation%7C%3FMode%7C%3FLocation\
%7C%3FSignal%20description%7C%3FPicture%7C%3FSignal%20file%7Climit%3D2000";

pub fn artemis_source() -> Source {
    Source { name: "artemis-sigid.sqlite", from: std::sync::Arc::new(Artemis), max_age: MAX_AGE }
}

pub fn unid_source() -> Source {
    Source::http("sigidwiki-unid.json", UNID_URL, MAX_AGE)
}

const AGENT: &str = concat!("WaveShark/", env!("CARGO_PKG_VERSION"), " (https://github.com/v0l/waveshark)");

/// The latest Artemis release, reduced to the SQLite file inside its tar.
///
/// The release tag is the validator: the API says what the latest tag is,
/// and if it is the one already held nothing else is fetched.
pub struct Artemis;

impl Fetch for Artemis {
    fn origin(&self) -> String {
        "github.com/AresValley/Artemis-DB".into()
    }

    fn fetch(&self, have: &Seen, to: &mut dyn Write) -> Result<Option<Seen>, Error> {
        let agent: ureq::Agent = ureq::Agent::config_builder()
            .user_agent(AGENT)
            .timeout_global(Some(Duration::from_secs(600)))
            .build()
            .into();
        let fail = |e: String| Error::Fetch(RELEASES.into(), e);
        let mut resp = agent.get(RELEASES).call().map_err(|e| fail(e.to_string()))?;
        let body = resp.body_mut().read_to_string().map_err(|e| fail(e.to_string()))?;
        let rel: serde_json::Value = serde_json::from_str(&body).map_err(|e| fail(e.to_string()))?;
        let tag = rel["tag_name"].as_str().ok_or_else(|| fail("no tag_name".into()))?.to_string();
        if have.etag.as_deref() == Some(tag.as_str()) {
            return Ok(None);
        }
        let url = rel["assets"]
            .as_array()
            .and_then(|a| a.iter().find(|a| a["name"].as_str().is_some_and(|n| n.ends_with(".tar"))))
            .and_then(|a| a["browser_download_url"].as_str())
            .ok_or_else(|| fail(format!("{tag}: no .tar asset")))?
            .to_string();
        let published = rel["published_at"].as_str().map(str::to_string);
        let mut resp = agent.get(&url).call().map_err(|e| Error::Fetch(url.clone(), e.to_string()))?;
        let mut body = resp.body_mut().with_config().limit(1 << 30).reader();
        match tar_entry(&mut body, "data.sqlite", to) {
            Ok(true) => Ok(Some(Seen { etag: Some(tag), last_modified: published })),
            Ok(false) => Err(Error::Fetch(url, "no data.sqlite in the release tar".into())),
            Err(e) => Err(Error::Io(url, e)),
        }
    }
}

/// Copy one file out of a tar stream and stop reading. Enough of the format
/// for a tar GNU or Python wrote: 512-byte headers, octal sizes, pax
/// extension headers skipped like any other entry, a name possibly split
/// across the ustar prefix field.
fn tar_entry(from: &mut dyn Read, want: &str, to: &mut dyn Write) -> std::io::Result<bool> {
    let mut header = [0u8; 512];
    loop {
        from.read_exact(&mut header)?;
        if header.iter().all(|b| *b == 0) {
            return Ok(false);
        }
        let field = |a: usize, b: usize| {
            let s = &header[a..b];
            let end = s.iter().position(|b| *b == 0).unwrap_or(s.len());
            String::from_utf8_lossy(&s[..end]).into_owned()
        };
        let size = u64::from_str_radix(field(124, 136).trim(), 8).unwrap_or(0);
        let kind = header[156];
        let mut name = field(0, 100);
        if &header[257..262] == b"ustar" {
            let prefix = field(345, 500);
            if !prefix.is_empty() {
                name = format!("{prefix}/{name}");
            }
        }
        let name = name.trim_start_matches("./");
        let regular = kind == b'0' || kind == 0;
        if regular && name == want {
            std::io::copy(&mut from.take(size), to)?;
            return Ok(true);
        }
        let padded = size.div_ceil(512) * 512;
        std::io::copy(&mut from.take(padded), &mut std::io::sink())?;
    }
}

/// One signal the wiki knows, or one it is asking about.
#[derive(Clone, Debug, PartialEq)]
pub struct Signal {
    pub name: String,
    pub url: String,
    /// Whether the wiki has a name for it or is still asking.
    pub identified: bool,
    pub description: String,
    pub categories: Vec<String>,
    /// As listed. Two values are usually a range and more are usually
    /// channels, but nobody is bound to that.
    pub frequencies_hz: Vec<u64>,
    pub bandwidths_hz: Vec<u64>,
    pub modulations: Vec<String>,
    pub modes: Vec<String>,
    pub locations: Vec<String>,
    /// Autocorrelation periods in milliseconds, where somebody measured one.
    pub acf_ms: Vec<f64>,
    /// A waterfall picture, where the wiki has one.
    pub picture_url: Option<String>,
}

impl Signal {
    /// The first sentence of the description, for a list row. Artemis keeps
    /// the wiki's markdown, headed "### SUMMARY", so headings are skipped
    /// and emphasis marks dropped.
    pub fn blurb(&self) -> String {
        let line = self
            .description
            .lines()
            .map(str::trim)
            .find(|l| !l.is_empty() && !l.starts_with('#'))
            .unwrap_or("");
        let line = line.replace("**", "").replace('`', "");
        match line.find(". ") {
            Some(i) if i < 200 => line[..=i].to_string(),
            _ => line,
        }
    }
}

/// Both halves of the wiki, loaded.
#[derive(Clone, Debug, Default)]
pub struct Db {
    pub signals: Vec<Signal>,
}

impl Db {
    pub fn len(&self) -> usize {
        self.signals.len()
    }

    pub fn is_empty(&self) -> bool {
        self.signals.is_empty()
    }

    pub fn identified(&self) -> usize {
        self.signals.iter().filter(|s| s.identified).count()
    }

    /// The entries that could be the burst described, best first. Empty when
    /// nothing is within a plausible distance in frequency.
    pub fn matches(&self, q: &Query) -> Vec<Match<'_>> {
        let mut out: Vec<Match<'_>> = self.signals.iter().filter_map(|s| score(s, q)).collect();
        out.sort_by(|a, b| b.score.total_cmp(&a.score).then_with(|| a.signal.name.cmp(&b.signal.name)));
        out
    }
}

/// What was measured about a burst, in the terms the wiki lists.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Query {
    pub center_hz: f64,
    /// Occupied width, when measured. Zero or absent otherwise.
    pub bandwidth_hz: Option<f64>,
    /// The classifier's label: "2-FSK", "OOK", "chirp", "OFDM". Absent for
    /// "unknown" and "noise-like", which constrain nothing.
    pub modulation: Option<String>,
    /// Period of a repeating structure in microseconds, when found.
    pub period_us: Option<f64>,
}

/// One candidate, with the score it earned and the line saying why.
#[derive(Clone, Debug, PartialEq)]
pub struct Match<'a> {
    pub signal: &'a Signal,
    /// Roughly 0 to 1.5: frequency up to 1, keying and width on top.
    pub score: f32,
    /// What matched, in the wiki's numbers: "868 MHz, FSK, 20 kHz".
    pub why: String,
}

/// The wiki's tags for one of the classifier's labels. The wiki's are what
/// contributors typed, so a label maps to every spelling that means the
/// same keying and to the family it belongs to.
pub fn wiki_modulations(label: &str) -> &'static [&'static str] {
    match label {
        "OOK" => &["OOK", "ASK", "PWM", "PPM", "Pulse", "CW"],
        "ASK" => &["ASK", "OOK", "PAM", "MP\u{2011}DASK"],
        "2-FSK" | "FSK" | "2FSK" => &["FSK", "GFSK", "2FSK", "AFSK", "CWFSK", "FFSK"],
        "4-FSK" | "4FSK" => &["4FSK", "FSK", "GFSK", "C4FM"],
        "C4FM" => &["C4FM", "4FSK"],
        "MSK" => &["MSK", "GMSK"],
        "GMSK" => &["GMSK", "MSK"],
        "BPSK" => &["BPSK", "PSK", "DPSK", "SDPSK"],
        "QPSK" => &["QPSK", "PSK", "OQPSK", "DQPSK"],
        "pi/4-DQPSK" => &["DQPSK", "QPSK", "PSK"],
        "8PSK" => &["8PSK", "D8PSK", "PSK"],
        "chirp" => &["FMCW", "CSS", "LFM"],
        "OFDM" => &["OFDM", "CP-OFDM", "SC-FDMA"],
        "DSSS" => &["DSSS", "CDMA"],
        "PPM" => &["PPM", "Pulse"],
        "carrier" => &["CW", "AM", "FM"],
        _ => &[],
    }
}

fn score<'a>(s: &'a Signal, q: &Query) -> Option<Match<'a>> {
    let f = q.center_hz;
    if f <= 0.0 || s.frequencies_hz.is_empty() {
        return None;
    }
    // How far off a listed frequency can be and still mean this one. The
    // wiki rounds to what a contributor thought worth typing, "868 MHz" for
    // a device on 868.3, so the tolerance is relative, floored by the
    // burst's own width and by the 100 kHz an HF entry is quoted to.
    let tol = (f * 0.001).max(q.bandwidth_hz.unwrap_or(0.0)).max(100e3);
    let nearest = s.frequencies_hz.iter().map(|&v| (v as f64 - f).abs()).fold(f64::MAX, f64::min);
    let point = if nearest <= tol { 1.0 - 0.5 * (nearest / tol) as f32 } else { 0.0 };
    let lo = *s.frequencies_hz.iter().min().unwrap() as f64;
    let hi = *s.frequencies_hz.iter().max().unwrap() as f64;
    // Inside the span the entry covers. Worth less the wider the span: a
    // cellular entry from 450 MHz to 3.5 GHz brackets everything and says
    // nothing by doing so.
    let range = if s.frequencies_hz.len() >= 2 && lo <= f && f <= hi {
        0.6 / (1.0 + 2.0 * (hi / lo).log10() as f32)
    } else {
        0.0
    };
    let freq = point.max(range);
    if freq <= 0.0 {
        return None;
    }
    let mut total = freq;
    let mut why = vec![if nearest <= tol {
        let v = s.frequencies_hz.iter().min_by_key(|&&v| (v as i64 - f as i64).abs()).unwrap();
        fmt_hz(*v as f64)
    } else {
        format!("{} to {}", fmt_hz(lo), fmt_hz(hi))
    }];

    if let Some(label) = q.modulation.as_deref() {
        let tags = wiki_modulations(label);
        if !tags.is_empty() && !s.modulations.is_empty() {
            let hit = s.modulations.iter().find(|m| tags.iter().any(|t| t.eq_ignore_ascii_case(m)));
            match hit {
                Some(m) => {
                    total += 0.3;
                    why.push(m.clone());
                }
                None => total -= 0.3,
            }
        }
    }

    if let (Some(bw), Some(&wiki)) = (q.bandwidth_hz.filter(|b| *b > 0.0), s.bandwidths_hz.first()) {
        if wiki > 0 {
            let r = (bw / wiki as f64).abs().log10().abs() as f32;
            let w = 0.2 * (1.0 - r.min(1.0));
            if w > 0.0 {
                total += w;
                why.push(format!("{} wide", fmt_hz(wiki as f64)));
            }
        }
    }

    if let (Some(us), false) = (q.period_us.filter(|p| *p > 0.0), s.acf_ms.is_empty()) {
        let ms = us / 1000.0;
        if s.acf_ms.iter().any(|a| *a > 0.0 && (a / ms).log10().abs() < 0.05) {
            total += 0.2;
            why.push(format!("{ms:.2} ms period"));
        }
    }

    Some(Match { signal: s, score: total, why: why.join(", ") })
}

pub fn fmt_hz(hz: f64) -> String {
    if hz >= 1e9 {
        format!("{:.3} GHz", hz / 1e9)
    } else if hz >= 1e6 {
        format!("{:.3} MHz", hz / 1e6)
    } else if hz >= 1e3 {
        format!("{:.1} kHz", hz / 1e3)
    } else {
        format!("{hz:.0} Hz")
    }
}

/// The identified half: Artemis's SQLite, downloaded if missing.
pub fn load_artemis(cache: &Cache) -> Result<Vec<Signal>, Error> {
    read_sqlite(&cache.get(&artemis_source())?)
}

/// The unidentified half: the wiki's own list, downloaded if missing.
pub fn load_unid(cache: &Cache) -> Result<Vec<Signal>, Error> {
    parse_unid(&cache.read(&unid_source())?)
}

/// Check the release and reread if a new one is out.
pub fn refresh_artemis(cache: &Cache, when: When) -> Result<Option<Vec<Signal>>, Error> {
    match cache.refresh(&artemis_source(), when)? {
        Some(p) => read_sqlite(&p).map(Some),
        None => Ok(None),
    }
}

/// Check the wiki list and reparse if it changed.
pub fn refresh_unid(cache: &Cache, when: When) -> Result<Option<Vec<Signal>>, Error> {
    match cache.refresh(&unid_source(), when)? {
        Some(p) => parse_unid(&std::fs::read(&p).map_err(|e| Error::Io(p.display().to_string(), e))?).map(Some),
        None => Ok(None),
    }
}

/// Both halves, downloading whichever is missing. The unidentified list is
/// optional: the wiki being down should not take the catalogue with it.
pub fn load(cache: &Cache) -> Result<Db, Error> {
    let mut signals = load_artemis(cache)?;
    match load_unid(cache) {
        Ok(mut u) => signals.append(&mut u),
        Err(e) => tracing::warn!("sigidwiki unidentified list unavailable: {e}"),
    }
    Ok(Db { signals })
}

fn read_sqlite(path: &Path) -> Result<Vec<Signal>, Error> {
    let name = path.display().to_string();
    let bad = |e: rusqlite::Error| Error::Parse(name.clone(), e.to_string());
    let db = rusqlite::Connection::open_with_flags(path, rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY)
        .map_err(bad)?;
    let mut out: Vec<Signal> = Vec::new();
    let mut ids: Vec<i64> = Vec::new();
    {
        let mut st = db.prepare("SELECT sig_id, name, url, description FROM signals ORDER BY sig_id").map_err(bad)?;
        let rows = st
            .query_map([], |r| {
                Ok((
                    r.get::<_, i64>(0)?,
                    r.get::<_, Option<String>>(1)?.unwrap_or_default(),
                    r.get::<_, Option<String>>(2)?.unwrap_or_default(),
                    r.get::<_, Option<String>>(3)?.unwrap_or_default(),
                ))
            })
            .map_err(bad)?;
        for row in rows {
            let (id, name, url, description) = row.map_err(bad)?;
            ids.push(id);
            out.push(Signal {
                name,
                url,
                identified: true,
                description,
                categories: Vec::new(),
                frequencies_hz: Vec::new(),
                bandwidths_hz: Vec::new(),
                modulations: Vec::new(),
                modes: Vec::new(),
                locations: Vec::new(),
                acf_ms: Vec::new(),
                picture_url: None,
            });
        }
    }
    let at = |id: i64| ids.binary_search(&id).ok();
    let mut texts = |sql: &str, put: &mut dyn FnMut(&mut Signal, String)| -> Result<(), Error> {
        let mut st = db.prepare(sql).map_err(bad)?;
        let rows = st
            .query_map([], |r| Ok((r.get::<_, i64>(0)?, r.get::<_, Option<String>>(1)?)))
            .map_err(bad)?;
        for row in rows {
            let (id, v) = row.map_err(bad)?;
            if let (Some(i), Some(v)) = (at(id), v.filter(|v| !v.trim().is_empty())) {
                put(&mut out[i], v.trim().to_string());
            }
        }
        Ok(())
    };
    texts(
        "SELECT c.sig_id, l.value FROM category c JOIN categorylabel l ON l.clb_id = c.clb_id",
        &mut |s, v| s.categories.push(v),
    )?;
    texts("SELECT sig_id, value FROM modulation", &mut |s, v| s.modulations.push(v))?;
    texts("SELECT sig_id, value FROM mode", &mut |s, v| s.modes.push(v))?;
    texts("SELECT sig_id, value FROM location", &mut |s, v| s.locations.push(v))?;
    texts("SELECT sig_id, CAST(value AS TEXT) FROM frequency WHERE value > 0", &mut |s, v| {
        if let Ok(hz) = v.parse::<u64>() {
            s.frequencies_hz.push(hz);
        }
    })?;
    texts("SELECT sig_id, CAST(value AS TEXT) FROM bandwidth WHERE value > 0", &mut |s, v| {
        if let Ok(hz) = v.parse::<u64>() {
            s.bandwidths_hz.push(hz);
        }
    })?;
    texts("SELECT sig_id, CAST(value AS TEXT) FROM acf WHERE value > 0", &mut |s, v| {
        if let Ok(ms) = v.parse::<f64>() {
            s.acf_ms.push(ms);
        }
    })?;
    for s in &mut out {
        s.frequencies_hz.sort_unstable();
        s.frequencies_hz.dedup();
    }
    Ok(out)
}

/// The wiki's `ask` result for the unidentified category.
fn parse_unid(raw: &[u8]) -> Result<Vec<Signal>, Error> {
    let bad = |e: String| Error::Parse("sigidwiki-unid.json".into(), e);
    let v: serde_json::Value = serde_json::from_slice(raw).map_err(|e| bad(e.to_string()))?;
    let results = v["query"]["results"].as_object().ok_or_else(|| bad("no query.results".into()))?;
    let texts = |p: &serde_json::Value, key: &str| -> Vec<String> {
        let mut out: Vec<String> = p[key]
            .as_array()
            .map(|a| {
                a.iter()
                    .filter_map(|x| x.as_str().map(|s| s.trim().to_string()).or_else(|| x.as_f64().map(|n| n.to_string())))
                    .filter(|s| !s.is_empty())
                    .collect()
            })
            .unwrap_or_default();
        out.dedup();
        out
    };
    let numbers = |p: &serde_json::Value, key: &str| -> Vec<u64> {
        let mut out: Vec<u64> = p[key]
            .as_array()
            .map(|a| a.iter().filter_map(|x| x.as_f64().or_else(|| x.as_str()?.parse().ok())).filter(|n| *n > 0.0).map(|n| n as u64).collect())
            .unwrap_or_default();
        out.sort_unstable();
        out.dedup();
        out
    };
    let mut out = Vec::with_capacity(results.len());
    for (title, r) in results {
        let p = &r["printouts"];
        let mut bandwidths = numbers(p, "Bandwidth min");
        bandwidths.extend(numbers(p, "Bandwidth max"));
        bandwidths.dedup();
        let picture = texts(p, "Picture").into_iter().next().map(|f| {
            format!("https://www.sigidwiki.com/wiki/Special:FilePath/{}", encode(&f.replace(' ', "_")))
        });
        out.push(Signal {
            name: title.clone(),
            url: r["fullurl"].as_str().unwrap_or_default().to_string(),
            identified: false,
            description: texts(p, "Signal description").into_iter().next().unwrap_or_default(),
            categories: vec!["Unidentified".into()],
            frequencies_hz: numbers(p, "Frequencies"),
            bandwidths_hz: bandwidths,
            modulations: texts(p, "Modulation"),
            modes: texts(p, "Mode"),
            locations: texts(p, "Location"),
            acf_ms: Vec::new(),
            picture_url: picture,
        });
    }
    Ok(out)
}

/// What was seen, for handing to the people who keep the catalogue.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Observation {
    pub center_hz: f64,
    pub modulation: Option<String>,
    pub bandwidth_hz: Option<f64>,
    pub baud: Option<f64>,
    pub duration_ms: Option<f64>,
    /// Sync word and preamble length, where the framing was read.
    pub sync_hex: Option<String>,
    pub preamble_bits: Option<u32>,
    pub frame_bytes: Option<usize>,
    /// How often it was heard and where, in the operator's words.
    pub location: Option<String>,
    pub notes: Option<String>,
}

impl Observation {
    /// A new issue against Artemis-DB with the measurements filled in. The
    /// maintainers ask for additions and corrections as issues there.
    pub fn artemis_issue_url(&self) -> String {
        let title = format!(
            "Unidentified {} signal at {}",
            self.modulation.as_deref().unwrap_or("unknown"),
            fmt_hz(self.center_hz)
        );
        format!(
            "https://github.com/AresValley/Artemis-DB/issues/new?title={}&body={}",
            encode(&title),
            encode(&self.markdown())
        )
    }

    /// The wiki's form for a new unidentified signal, with what the form
    /// accepts from a query string filled in. Prefilling this way is a
    /// Semantic Forms convention the wiki's version is believed to honour;
    /// the fields are still there to type if it does not.
    pub fn sigidwiki_form_url(&self) -> String {
        let mut url = String::from(
            "https://www.sigidwiki.com/index.php/Special:FormEdit/Unidentified_Signal?\
             preload=Signal_Identification_Wiki:Signal_form_preload_text",
        );
        let mut field = |k: &str, v: String| {
            url.push_str(&format!("&Unidentified_Signal%5B{}%5D={}", encode(k), encode(&v)));
        };
        field("Frequencies", fmt_hz(self.center_hz));
        if let Some(m) = &self.modulation {
            field("Modulation", wiki_modulations(m).first().unwrap_or(&m.as_str()).to_string());
        }
        if let Some(bw) = self.bandwidth_hz.filter(|b| *b > 0.0) {
            field("Bandwidth", fmt_hz(bw));
        }
        if let Some(l) = &self.location {
            field("Location", l.clone());
        }
        field("Signal description", self.plain());
        url
    }

    /// The observation as a markdown table with a sentence under it.
    pub fn markdown(&self) -> String {
        let mut rows: Vec<(&str, String)> = vec![("Frequency", fmt_hz(self.center_hz))];
        if let Some(m) = &self.modulation {
            rows.push(("Modulation", m.clone()));
        }
        if let Some(bw) = self.bandwidth_hz.filter(|b| *b > 0.0) {
            rows.push(("Bandwidth", fmt_hz(bw)));
        }
        if let Some(b) = self.baud.filter(|b| *b > 0.0) {
            rows.push(("Symbol rate", format!("{b:.0} Bd")));
        }
        if let Some(d) = self.duration_ms.filter(|d| *d > 0.0) {
            rows.push(("Burst length", format!("{d:.1} ms")));
        }
        if let Some(s) = &self.sync_hex {
            rows.push(("Sync word", format!("`{s}`")));
        }
        if let Some(p) = self.preamble_bits {
            rows.push(("Preamble", format!("{p} bits")));
        }
        if let Some(n) = self.frame_bytes {
            rows.push(("Frame", format!("{n} bytes")));
        }
        if let Some(l) = &self.location {
            rows.push(("Location", l.clone()));
        }
        let mut s = String::from("| | |\n|---|---|\n");
        for (k, v) in rows {
            s.push_str(&format!("| {k} | {v} |\n"));
        }
        if let Some(n) = &self.notes {
            s.push('\n');
            s.push_str(n);
            s.push('\n');
        }
        s.push_str("\nMeasured with WaveShark.\n");
        s
    }

    fn plain(&self) -> String {
        let mut parts = vec![format!("{} at {}", self.modulation.as_deref().unwrap_or("unknown"), fmt_hz(self.center_hz))];
        if let Some(bw) = self.bandwidth_hz.filter(|b| *b > 0.0) {
            parts.push(format!("{} wide", fmt_hz(bw)));
        }
        if let Some(b) = self.baud.filter(|b| *b > 0.0) {
            parts.push(format!("{b:.0} Bd"));
        }
        if let Some(d) = self.duration_ms.filter(|d| *d > 0.0) {
            parts.push(format!("bursts of {d:.1} ms"));
        }
        if let Some(s) = &self.sync_hex {
            parts.push(format!("sync {s}"));
        }
        let mut s = parts.join(", ");
        if let Some(n) = &self.notes {
            s.push_str(". ");
            s.push_str(n);
        }
        s
    }
}

/// Percent-encode for a query string.
pub fn encode(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => out.push(b as char),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sig(name: &str, freqs: &[u64], modulations: &[&str], bw: &[u64]) -> Signal {
        Signal {
            name: name.into(),
            url: String::new(),
            identified: true,
            description: String::new(),
            categories: Vec::new(),
            frequencies_hz: freqs.to_vec(),
            bandwidths_hz: bw.to_vec(),
            modulations: modulations.iter().map(|s| s.to_string()).collect(),
            modes: Vec::new(),
            locations: Vec::new(),
            acf_ms: Vec::new(),
            picture_url: None,
        }
    }

    /// The real 868 MHz neighbourhood: a wide cellular entry that brackets
    /// everything must lose to a keyfob typed as "868.3 MHz, FSK".
    #[test]
    fn a_near_point_with_the_right_keying_beats_a_bracketing_range() {
        let db = Db {
            signals: vec![
                sig("LTE", &[450_000_000, 3_500_000_000], &["QPSK", "OFDM"], &[1_400_000]),
                sig("Keyfob", &[868_300_000], &["FSK"], &[250_000]),
                sig("LoRa", &[433_000_000, 863_000_000, 870_000_000, 915_000_000], &["CSS"], &[125_000]),
                sig("POCSAG", &[25_000_000, 932_000_000], &["FSK"], &[9_000]),
                sig("HF thing", &[7_000_000], &["FSK"], &[3_000]),
            ],
        };
        let q = Query { center_hz: 868_097_000.0, bandwidth_hz: Some(28_000.0), modulation: Some("2-FSK".into()), period_us: None };
        let m = db.matches(&q);
        let names: Vec<&str> = m.iter().map(|m| m.signal.name.as_str()).collect();
        assert_eq!(names[0], "Keyfob", "{names:?}");
        assert!(!names.contains(&"HF thing"), "{names:?}");
        assert!(names.iter().position(|n| *n == "LoRa") < names.iter().position(|n| *n == "LTE"), "{names:?}");
        assert!(m[0].why.starts_with("868.300 MHz"), "{}", m[0].why);
    }

    /// A chirp at 868 is LoRa, not the keyfob, however close the keyfob's
    /// frequency is.
    #[test]
    fn a_blurb_skips_the_markdown_heading_artemis_keeps() {
        let mut s = sig("iDEN", &[869_000_000], &["QAM"], &[]);
        s.description = "### SUMMARY\niDEN is a **TDMA** standard. It is trunked.\n### DETAILS\nmore".into();
        assert_eq!(s.blurb(), "iDEN is a TDMA standard.");
    }

    #[test]
    fn keying_moves_the_order() {
        let db = Db {
            signals: vec![
                sig("Keyfob", &[868_300_000], &["FSK"], &[250_000]),
                sig("LoRa", &[433_000_000, 863_000_000, 870_000_000, 915_000_000], &["CSS"], &[125_000]),
            ],
        };
        let q = Query { center_hz: 868_100_000.0, bandwidth_hz: Some(125_000.0), modulation: Some("chirp".into()), period_us: None };
        assert_eq!(db.matches(&q)[0].signal.name, "LoRa");
    }

    #[test]
    fn a_pax_tar_gives_up_only_the_file_asked_for() {
        // A tar as Python writes one: a pax header, a directory, the file,
        // and something after it that must never be read.
        let mut tar = Vec::new();
        let entry = |name: &str, kind: u8, body: &[u8]| {
            let mut h = [0u8; 512];
            h[..name.len()].copy_from_slice(name.as_bytes());
            let size = format!("{:011o}\0", body.len());
            h[124..136].copy_from_slice(size.as_bytes());
            h[156] = kind;
            h[257..263].copy_from_slice(b"ustar\0");
            let mut v = h.to_vec();
            v.extend_from_slice(body);
            v.resize(v.len().div_ceil(512) * 512, 0);
            v
        };
        tar.extend(entry("././@PaxHeader", b'x', b"28 mtime=1784382605.4039571\n"));
        tar.extend(entry("./", b'5', b""));
        tar.extend(entry("./data.sqlite", b'0', b"SQLite format 3"));
        tar.extend(entry("./media/1.png", b'0', b"PNG"));
        struct Counting<'a>(&'a [u8], usize);
        impl Read for Counting<'_> {
            fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
                let n = self.0.read(buf)?;
                self.1 += n;
                Ok(n)
            }
        }
        let mut from = Counting(&tar, 0);
        let mut out = Vec::new();
        assert!(tar_entry(&mut from, "data.sqlite", &mut out).unwrap());
        assert_eq!(out, b"SQLite format 3");
        assert!(from.1 < tar.len(), "read past the file it wanted");
    }

    #[test]
    fn the_unidentified_list_parses_as_the_wiki_returns_it() {
        let raw = br#"{"query":{"results":{"Odd burst":{"printouts":{"Frequencies":[603000000],"Bandwidth min":[3000],"Bandwidth max":[],"Modulation":["FSK"],"Mode":["USB"],"Location":["Bosnia"],"Signal description":["Seen once."],"Picture":["603 MHz.png"],"Signal file":[]},"fullurl":"https://www.sigidwiki.com/wiki/Odd_burst"}}}}"#;
        let v = parse_unid(raw).unwrap();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].frequencies_hz, [603_000_000]);
        assert_eq!(v[0].bandwidths_hz, [3000]);
        assert!(!v[0].identified);
        assert_eq!(v[0].blurb(), "Seen once.");
        assert_eq!(v[0].picture_url.as_deref(), Some("https://www.sigidwiki.com/wiki/Special:FilePath/603_MHz.png"));
    }

    #[test]
    fn a_report_carries_the_numbers_and_opens_the_right_places() {
        let o = Observation {
            center_hz: 868_100_000.0,
            modulation: Some("2-FSK".into()),
            bandwidth_hz: Some(28_000.0),
            baud: Some(19_600.0),
            duration_ms: Some(13.6),
            sync_hex: Some("474f".into()),
            preamble_bits: Some(84),
            frame_bytes: Some(22),
            location: Some("Ireland".into()),
            notes: None,
        };
        let issue = o.artemis_issue_url();
        assert!(issue.starts_with("https://github.com/AresValley/Artemis-DB/issues/new?title="));
        assert!(issue.contains("474f"));
        let form = o.sigidwiki_form_url();
        assert!(form.contains("Unidentified_Signal%5BModulation%5D=FSK"), "{form}");
        assert!(o.markdown().contains("| Symbol rate | 19600 Bd |"));
    }

    /// The real release's database, when a copy has been fetched into the
    /// cache, reads to the count Artemis publishes.
    #[test]
    fn the_cached_release_reads_if_present() {
        let Ok(dir) = Cache::default_dir() else { return };
        let p = dir.join("artemis-sigid.sqlite");
        if !p.exists() {
            eprintln!("skipping: {} not cached", p.display());
            return;
        }
        let v = read_sqlite(&p).unwrap();
        assert!(v.len() > 500, "{}", v.len());
        let lora = v.iter().find(|s| s.name == "LoRa").expect("LoRa");
        assert!(lora.frequencies_hz.contains(&868_000_000) || lora.frequencies_hz.contains(&863_000_000));
        assert_eq!(lora.modulations, ["CSS"]);
    }
}

#[cfg(test)]
mod network {
    use super::*;

    /// Fetches the real release and the real wiki list into the default
    /// cache. Network, so opt in: `cargo test -p datasets -- --ignored`.
    #[test]
    #[ignore]
    fn both_halves_download_and_read() {
        let cache = Cache::at_default_dir().unwrap();
        let db = load(&cache).unwrap();
        eprintln!("{} signals, {} identified", db.len(), db.identified());
        assert!(db.identified() > 500);
        assert!(db.len() - db.identified() > 300);
        let q = Query { center_hz: 868_097_000.0, bandwidth_hz: Some(28_000.0), modulation: Some("2-FSK".into()), period_us: None };
        for m in db.matches(&q).iter().take(8) {
            eprintln!("{:.2} {} [{}] {}", m.score, m.signal.name, m.why, if m.signal.identified { "" } else { "UNID" });
        }
    }
}
