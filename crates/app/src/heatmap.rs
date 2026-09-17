//! The span kept as readings, and written out as a picture.
//!
//! The waterfall is a display: it turns decibels into pixels as they arrive
//! and throws the decibels away, so nothing downstream can re-colour a row,
//! re-scale it against a different floor or say what a point was. An export
//! needs all three, so this keeps the readings instead of the pixels, in a
//! bounded ring, and colours them only when somebody asks for a file.
//!
//! It is a node rather than something the pane does, because it records: a
//! heatmap that only exists while the waterfall is on screen is a heatmap
//! nobody can take at the end of a night's listening.
//!
//! A reading is stored as one byte, [`DB_STEP`] decibels a step from
//! [`DB_BASE`], which covers a converter's whole range and costs a quarter of
//! what an `f32` does: at 2048 bins and two rows a second the default budget
//! of 32 MB holds a little over two hours.

use base64::Engine;
use common::{Hz, Result};
use pipeline::node::{NodeCtx, PortSpec, Simple};
use pipeline::param::{Param, ParamValue};
use pipeline::port::{Payload, PortKind, StreamSpec};
use pipeline::registry::{Category, Settings, SettingsExt, StageDesc};
use std::collections::VecDeque;
use std::path::{Path, PathBuf};

/// The weakest reading a byte can hold, in dBFS.
pub const DB_BASE: f32 = -170.0;
/// Decibels a step. 255 steps from [`DB_BASE`] reach +21 dBFS, which is above
/// anything a converter can deliver, and the 0.375 dB worst-case error is
/// well under a colour step of any ramp drawn over a 60 dB window.
pub const DB_STEP: f32 = 0.75;

/// What the ring may take, in bytes.
pub const DEFAULT_BUDGET: usize = 32 << 20;

pub fn quantise(db: f32) -> u8 {
    let v = (db - DB_BASE) / DB_STEP;
    v.round().clamp(0.0, 255.0) as u8
}

pub fn dequantise(v: u8) -> f32 {
    DB_BASE + v as f32 * DB_STEP
}

/// Where exported heatmaps are written, beside the pictures.
pub fn heatmaps_dir() -> PathBuf {
    crate::picsave::pictures_dir().with_file_name("heatmaps")
}

/// A colour ramp, as data rather than as a painter: an export runs where
/// there is no egui context and no pane.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Ramp {
    /// The panel's own: cold cyan for noise, hot amber for signal.
    #[default]
    Chassis,
    Grey,
    Inferno,
    Viridis,
}

impl Ramp {
    pub const ALL: [Ramp; 4] = [Ramp::Chassis, Ramp::Grey, Ramp::Inferno, Ramp::Viridis];

    pub fn label(self) -> &'static str {
        match self {
            Ramp::Chassis => "chassis",
            Ramp::Grey => "grey",
            Ramp::Inferno => "inferno",
            Ramp::Viridis => "viridis",
        }
    }

    pub fn parse(s: &str) -> Ramp {
        Ramp::ALL.into_iter().find(|r| r.label() == s).unwrap_or_default()
    }

    /// The stops, lowest first. Every ramp brightens monotonically, so a
    /// feature stays readable in greyscale and to a colour-deficient viewer;
    /// the test beside this pins it.
    fn stops(self) -> &'static [(f32, [u8; 3])] {
        match self {
            // Most bins in any span are noise, so the ramp stays dark well
            // past the midpoint: brightening early spends the whole scale on
            // the noise floor and leaves signals nowhere to go.
            Ramp::Chassis => &[
                (0.00, [7, 9, 13]),
                (0.35, [11, 31, 41]),
                (0.60, [19, 95, 123]),
                (0.78, [45, 156, 189]),
                (0.90, [239, 159, 47]),
                (1.00, [255, 246, 223]),
            ],
            Ramp::Grey => &[(0.0, [0, 0, 0]), (1.0, [255, 255, 255])],
            Ramp::Inferno => &[
                (0.00, [0, 0, 4]),
                (0.25, [87, 16, 110]),
                (0.50, [188, 55, 84]),
                (0.75, [249, 142, 9]),
                (1.00, [252, 255, 164]),
            ],
            Ramp::Viridis => &[
                (0.00, [68, 1, 84]),
                (0.25, [59, 82, 139]),
                (0.50, [33, 145, 140]),
                (0.75, [94, 201, 98]),
                (1.00, [253, 231, 37]),
            ],
        }
    }

    /// The colour at `t`, which is clamped to the ends of the ramp.
    pub fn sample(self, t: f32) -> [u8; 3] {
        let stops = self.stops();
        let t = t.clamp(0.0, 1.0);
        let mut i = 0;
        while i + 2 < stops.len() && t > stops[i + 1].0 {
            i += 1;
        }
        let (a, b) = (stops[i], stops[i + 1]);
        let f = ((t - a.0) / (b.0 - a.0)).clamp(0.0, 1.0);
        let c = |x: usize| (a.1[x] as f32 + (b.1[x] as f32 - a.1[x] as f32) * f) as u8;
        [c(0), c(1), c(2)]
    }
}

/// One spectrum row, as it was read.
///
/// A row covers the span the receiver was on when it was taken, which is not
/// always the whole of the axis the heatmap has grown to: `bin0` is where it
/// starts on that axis.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Row {
    /// Unix microseconds the row was finished at.
    pub at_us: u64,
    /// First bin of the heatmap's axis this row holds a reading for.
    pub bin0: usize,
    pub db: Vec<u8>,
}

/// A reading the receiver was not listening to. Zero is reserved for it, so
/// a band walk's picture says where the dial was rather than drawing an
/// empty span at the noise floor.
pub const UNREAD: u8 = 0;

/// The readings, bounded, on a frequency axis that grows.
///
/// The axis is absolute: a bin is a frequency, not an offset from wherever
/// the dial happens to be, so moving the dial widens the picture instead of
/// throwing it away. A row keeps only the bins it covered, and the export
/// fills the rest with [`UNREAD`].
#[derive(Clone, Debug)]
pub struct Heat {
    rows: VecDeque<Row>,
    bins: usize,
    /// Frequency of the low edge of bin zero.
    base_hz: f64,
    /// Width of a bin. Set by the first row and kept: a later span at a
    /// different rate is resampled onto it rather than starting again.
    step_hz: f64,
    /// Where the receiver is now, for placing the next row.
    center: Hz,
    rate: f64,
    budget: usize,
    bytes: usize,
}

/// How wide the axis may grow, in bins. A walk across a whole band is worth
/// keeping; a receiver dragged from 100 kHz to 6 GHz at a 1 kHz bin is six
/// million columns of mostly nothing, which is a picture nobody can open.
const MAX_BINS: usize = 1 << 17;

impl Default for Heat {
    fn default() -> Self {
        Self::new(DEFAULT_BUDGET)
    }
}

impl Heat {
    pub fn new(budget: usize) -> Self {
        Self {
            rows: VecDeque::new(),
            bins: 0,
            base_hz: 0.0,
            step_hz: 0.0,
            center: Hz(0),
            rate: 0.0,
            budget: budget.max(1 << 16),
            bytes: 0,
        }
    }

    pub fn set_budget(&mut self, bytes: usize) {
        self.budget = bytes.max(1 << 16);
        self.trim();
    }

    pub fn budget(&self) -> usize {
        self.budget
    }

    /// Where the readings are being taken now.
    ///
    /// A retune does not throw the history away: the next row lands at its
    /// own frequency on the same axis, and the axis widens to hold it. That
    /// is what makes a heatmap of a band walk, or of an evening spent moving
    /// the dial, a picture of the band rather than of the last step.
    pub fn tuned(&mut self, center: Hz, rate: f64) {
        self.center = center;
        self.rate = rate;
    }

    pub fn clear(&mut self) {
        self.rows.clear();
        self.bytes = 0;
        self.bins = 0;
        self.step_hz = 0.0;
        self.base_hz = 0.0;
    }

    pub fn push(&mut self, at_us: u64, db: &[f32]) {
        if db.is_empty() || self.rate <= 0.0 {
            return;
        }
        let low = self.center.as_f64() - self.rate / 2.0;
        if self.rows.is_empty() {
            self.step_hz = self.rate / db.len() as f64;
            self.base_hz = low;
            self.bins = db.len();
        }
        // The row on this axis: where it starts, and how many bins of it
        // there are at this axis's resolution.
        let start = ((low - self.base_hz) / self.step_hz).round() as i64;
        let width = (self.rate / self.step_hz).round().max(1.0) as i64;
        let Some(shift) = self.widen(start, width) else {
            // Further than the axis may stretch: this is a different watch,
            // not a wider one.
            self.clear();
            self.step_hz = self.rate / db.len() as f64;
            self.base_hz = low;
            self.bins = db.len();
            self.rows.push_back(Row {
                at_us,
                bin0: 0,
                db: db.iter().map(|v| quantise(*v)).collect(),
            });
            self.bytes += db.len();
            self.trim();
            return;
        };
        let start = (start + shift) as usize;
        // Nearest source bin per axis bin, which is a copy where the rate
        // has not changed and a resample where it has.
        let width = width as usize;
        let mut out = Vec::with_capacity(width);
        for i in 0..width {
            let src = (i as f64 + 0.5) * db.len() as f64 / width as f64;
            let v = db[(src as usize).min(db.len() - 1)];
            out.push(quantise(v));
        }
        self.bytes += out.len();
        self.rows.push_back(Row { at_us, bin0: start, db: out });
        self.trim();
    }

    /// Make room on the axis for a row at `start` of `width` bins, moving
    /// the origin down if it starts below bin zero. The shift applied to
    /// every existing row, or `None` if this would take the axis past
    /// [`MAX_BINS`].
    fn widen(&mut self, start: i64, width: i64) -> Option<i64> {
        let below = (-start).max(0);
        let above = (start + width - self.bins as i64).max(0);
        let bins = self.bins as i64 + below + above;
        if bins > MAX_BINS as i64 {
            return None;
        }
        if below > 0 {
            for r in &mut self.rows {
                r.bin0 += below as usize;
            }
            self.base_hz -= below as f64 * self.step_hz;
        }
        self.bins = bins as usize;
        Some(below)
    }

    fn trim(&mut self) {
        while self.bytes > self.budget {
            match self.rows.pop_front() {
                Some(r) => self.bytes -= r.db.len(),
                None => break,
            }
        }
    }

    pub fn rows(&self) -> usize {
        self.rows.len()
    }

    pub fn bins(&self) -> usize {
        self.bins
    }

    pub fn bytes(&self) -> usize {
        self.bytes
    }

    /// The middle of the axis, which is the middle of everything heard
    /// rather than wherever the dial is now.
    pub fn center(&self) -> Hz {
        Hz((self.base_hz + self.bins as f64 * self.step_hz / 2.0).max(0.0) as u64)
    }

    /// What the axis covers, in hertz.
    pub fn rate(&self) -> f64 {
        self.bins as f64 * self.step_hz
    }

    /// Oldest to newest.
    pub fn row(&self, i: usize) -> Option<&Row> {
        self.rows.get(i)
    }

    /// One row across the whole axis, [`UNREAD`] where the receiver was not
    /// listening at the time.
    pub fn dense_row(&self, i: usize) -> Vec<u8> {
        let mut out = vec![UNREAD; self.bins];
        if let Some(r) = self.rows.get(i) {
            let end = (r.bin0 + r.db.len()).min(self.bins);
            if r.bin0 < end {
                out[r.bin0..end].copy_from_slice(&r.db[..end - r.bin0]);
            }
        }
        out
    }

    /// How long the history covers, from the oldest row to the newest.
    pub fn seconds(&self) -> f64 {
        match (self.rows.front(), self.rows.back()) {
            (Some(a), Some(b)) if b.at_us > a.at_us => (b.at_us - a.at_us) as f64 / 1e6,
            _ => 0.0,
        }
    }

    /// The frequency a bin holds, in hertz.
    pub fn bin_hz(&self, bin: usize) -> f64 {
        if self.bins == 0 {
            return self.center.as_f64();
        }
        self.base_hz + (bin as f64 + 0.5) * self.step_hz
    }
}

/// The same picture with the readings beside it, in one file that opens
/// anywhere: the image as a data URI, the quantised decibels as a second
/// one, and enough of the tuning to turn a pixel back into a time and a
/// frequency.
pub fn html(heat: &Heat, ramp: Ramp, floor: f32, ceil: f32) -> Result<String> {
    let (rows, bins) = (heat.rows(), heat.bins());
    if rows == 0 || bins == 0 {
        return Err(common::Error::other("nothing has been recorded yet"));
    }
    let b64 = base64::engine::general_purpose::STANDARD;
    // Newest row first, so a page opens on what was just heard.
    let mut readings = Vec::with_capacity(rows * bins);
    let mut times = Vec::with_capacity(rows);
    for i in (0..rows).rev() {
        let r = heat.row(i).expect("row in range");
        readings.extend_from_slice(&heat.dense_row(i));
        // Unix milliseconds, so the page can say the time and a reader can
        // put a row beside a log line.
        times.push((r.at_us / 1000).to_string());
    }
    // Gzipped, because the readings are mostly a noise floor and unread
    // axis: the 33 MB page that prompted this is a few hundred kilobytes
    // once deflated, and every browser can undo it with DecompressionStream.
    let mut gz = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
    use std::io::Write as _;
    gz.write_all(&readings)
        .and_then(|()| gz.try_finish())
        .map_err(|e| common::Error::other(format!("cannot compress the readings: {e}")))?;
    let data =
        b64.encode(gz.finish().map_err(|e| common::Error::other(format!("cannot compress: {e}")))?);
    let made = now_us();
    // The colours go as stops rather than as pixels: the page paints the
    // readings itself, so the floor and the ceiling can be anything and the
    // ramp is the same one the panel drew.
    let stops: Vec<String> =
        ramp.stops().iter().map(|(t, c)| format!("[{t},[{},{},{}]]", c[0], c[1], c[2])).collect();
    let meta = format!(
        "{{\"waveshark\":\"{}\",\"generated\":{},\"center\":{},\"rate\":{},\"bins\":{},\
         \"rows\":{},\"base\":{},\"step\":{},\"floor\":{},\"ceil\":{},\"ramp\":[{}],\
         \"at\":[{}]}}",
        crate::update::running(),
        made / 1000,
        heat.center().as_f64(),
        heat.rate(),
        bins,
        rows,
        DB_BASE,
        DB_STEP,
        floor,
        ceil,
        stops.join(","),
        times.join(",")
    );
    let newest = heat.row(rows - 1).map(|r| r.at_us).unwrap_or(made);
    let oldest = heat.row(0).map(|r| r.at_us).unwrap_or(made);
    let page = include_str!("heatmap.html")
        .replace("__TITLE__", &format!("{:.4} MHz", heat.center().as_f64() / 1e6))
        .replace("__SPAN__", &format!("{:.3} MHz", heat.rate() / 1e6))
        .replace("__ROWS__", &rows.to_string())
        .replace("__BINS__", &bins.to_string())
        .replace("__FROM__", &format!("{} UTC", stamp(oldest)))
        .replace("__TO__", &format!("{} UTC", stamp(newest)))
        .replace("__MADE__", &format!("{} UTC", stamp(made)))
        .replace("__VERSION__", crate::update::running())
        .replace("__META__", &meta)
        .replace("__DATA__", &data);
    Ok(page)
}

/// A time as the page prints it: the date as well, because a picture
/// outlives the day it was made.
fn stamp(at_us: u64) -> String {
    chrono::DateTime::from_timestamp((at_us / 1_000_000) as i64, 0)
        .map(|t| t.format("%Y-%m-%d %H:%M:%S").to_string())
        .unwrap_or_default()
}

/// A 3x5 font, enough for a frequency and a time. Burned into the picture
/// rather than drawn with a real typeface because an export must not depend
/// on a font being installed, and these are the only characters an axis
/// label uses.
/// What the heatmap holds and where the last export went.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HeatmapStatus {
    pub recording: bool,
    pub rows: usize,
    pub bins: usize,
    pub seconds: f64,
    pub bytes: usize,
    pub budget: usize,
    pub saved: Option<PathBuf>,
    pub error: Option<String>,
}

/// The recorder: the display's transform, kept at its own rate.
///
/// It reads a spectrum port rather than samples, so the bins it records are
/// the bins the waterfall drew, at whatever size the operator set, and the
/// FFT runs once for both. Its own rate still, because a waterfall scrolling
/// at thirty rows a second would fill any budget in minutes and the point of
/// the thing is a heatmap of a night nobody watched.
pub struct HeatmapNode {
    heat: Heat,
    recording: bool,
    rows_per_sec: f32,
    rate: f64,
    center: Hz,
    /// When the last row was taken, for keeping one row per interval out of
    /// however many frames the display produces.
    last_row: u64,
    saved: Option<PathBuf>,
    error: Option<String>,
}

impl Default for HeatmapNode {
    fn default() -> Self {
        Self::new(DEFAULT_BUDGET)
    }
}

impl HeatmapNode {
    pub fn new(budget: usize) -> Self {
        Self {
            heat: Heat::new(budget),
            recording: true,
            rows_per_sec: 2.0,
            rate: 0.0,
            center: Hz(0),
            last_row: 0,
            saved: None,
            error: None,
        }
    }

    pub fn set_recording(&mut self, on: bool) {
        self.recording = on;
    }

    pub fn set_rows_per_sec(&mut self, v: f32) {
        self.rows_per_sec = v.clamp(0.02, 20.0);
    }

    pub fn status(&self) -> HeatmapStatus {
        HeatmapStatus {
            recording: self.recording,
            rows: self.heat.rows(),
            bins: self.heat.bins(),
            seconds: self.heat.seconds(),
            bytes: self.heat.bytes(),
            budget: self.heat.budget(),
            saved: self.saved.clone(),
            error: self.error.clone(),
        }
    }

    /// Write what has been recorded into `dir`, and remember where it went
    /// so the interface can say so without asking for the file back.
    pub fn export(&mut self, dir: &Path, ramp: Ramp, floor: f32, ceil: f32) -> Result<PathBuf> {
        let r = self.write(dir, ramp, floor, ceil);
        match &r {
            Ok(p) => {
                self.saved = Some(p.clone());
                self.error = None;
            }
            Err(e) => {
                self.error = Some(e.to_string());
            }
        }
        r
    }

    fn write(&self, dir: &Path, ramp: Ramp, floor: f32, ceil: f32) -> Result<PathBuf> {
        std::fs::create_dir_all(dir)?;
        let name = format!(
            "heatmap_{}_{:.4}M_{:.0}k.html",
            chrono::Utc::now().format("%Y%m%d-%H%M%S"),
            self.heat.center().as_f64() / 1e6,
            self.heat.rate() / 1e3,
        );
        let path = dir.join(name);
        std::fs::write(&path, html(&self.heat, ramp, floor, ceil)?)?;
        Ok(path)
    }
}

impl Simple for HeatmapNode {
    fn name(&self) -> &str {
        "heatmap"
    }

    fn is_sink(&self) -> bool {
        true
    }

    /// Bins, not samples: the recorder reads the frames the spectrum stage
    /// already computes rather than transforming the same span again, so a
    /// row is the same reading the waterfall drew and the FFT is run once.
    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Spectrum {
            return Err(common::Error::other("heatmap reads a spectrum"));
        }
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let frames = i.as_spectrum().unwrap_or(&[]);
        if !self.recording {
            return Ok(());
        }
        for f in frames {
            // One row per interval, out of however many the display asked
            // for: a waterfall wants thirty a second and a night's watch
            // wants one every ten.
            let wait = (1e6 / self.rows_per_sec.max(0.02) as f64) as u64;
            if self.last_row > 0 && f.at_us.saturating_sub(self.last_row) < wait {
                continue;
            }
            self.last_row = f.at_us;
            self.rate = f.span_hz;
            self.center = Hz(f.center_hz.max(0.0) as u64);
            self.heat.tuned(self.center, self.rate);
            self.heat.push(f.at_us, &f.db);
        }
        Ok(())
    }

    fn params(&self) -> Vec<Param> {
        vec![
            Param::bool(RECORDING, self.recording).label("Keep the span as readings"),
            Param::float(ROWS_PER_SEC, self.rows_per_sec as f64, 0.02..=20.0)
                .unit("rows/s")
                .label("Rows a second")
                .log(),
            Param::float(BUDGET_MB, self.heat.budget() as f64 / (1 << 20) as f64, 1.0..=4096.0)
                .unit("MB")
                .label("Readings kept")
                .log(),
        ]
    }

    fn set_param(&mut self, name: &str, v: ParamValue) -> Result<()> {
        match name {
            RECORDING => {
                self.set_recording(v.as_bool().unwrap_or(true));
                Ok(())
            }
            ROWS_PER_SEC => {
                self.set_rows_per_sec(v.as_f64().unwrap_or(2.0) as f32);
                Ok(())
            }
            BUDGET_MB => {
                let mb = v.as_f64().unwrap_or(32.0).max(1.0);
                self.heat.set_budget((mb * (1 << 20) as f64) as usize);
                Ok(())
            }
            _ => Err(common::Error::other(format!("heatmap: unknown parameter {name:?}"))),
        }
    }
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

const RECORDING: &str = "recording";
const ROWS_PER_SEC: &str = "rows_per_sec";
const BUDGET_MB: &str = "budget_mb";

pub const DESC: StageDesc = StageDesc {
    name: "heatmap",
    summary: "Keep the span as decibels rather than pixels, so it can be \
              exported as a picture with the readings beside it",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let mb = s.f64_or(BUDGET_MB, DEFAULT_BUDGET as f64 / (1 << 20) as f64).max(1.0);
    let mut n = HeatmapNode::new((mb * (1 << 20) as f64) as usize);
    n.set_recording(s.bool_or(RECORDING, true));
    n.set_rows_per_sec(s.f64_or(ROWS_PER_SEC, 2.0) as f32);
    Ok(Box::new(n))
}

#[cfg(test)]
mod tests {
    use super::*;
    use common::C32;
    use pipeline::node::Node;

    fn luma(c: [u8; 3]) -> f32 {
        0.2126 * c[0] as f32 + 0.7152 * c[1] as f32 + 0.0722 * c[2] as f32
    }

    #[test]
    fn every_ramp_brightens_monotonically() {
        for r in Ramp::ALL {
            let mut prev = -1.0;
            for i in 0..=200 {
                let l = luma(r.sample(i as f32 / 200.0));
                assert!(l >= prev - 1.0, "{} dipped at {i}: {l} after {prev}", r.label());
                prev = l;
            }
        }
    }

    #[test]
    fn a_reading_survives_the_round_trip_to_within_half_a_step() {
        let mut worst = 0.0f32;
        let mut db = -160.0f32;
        while db <= 10.0 {
            let back = dequantise(quantise(db));
            worst = worst.max((back - db).abs());
            db += 0.1;
        }
        // Half of DB_STEP, and nothing worse anywhere in the range.
        assert!(worst <= 0.375 + 1e-4, "worst quantisation error was {worst} dB");
    }

    #[test]
    fn the_budget_drops_the_oldest_rows_and_nothing_else() {
        // 64 KB of budget and 1024 bins to a row is exactly 64 rows.
        let mut h = Heat::new(64 << 10);
        h.tuned(Hz(433_000_000), 1_024_000.0);
        for i in 0..200u64 {
            h.push(1_000_000 + i * 500_000, &vec![-80.0f32; 1024]);
        }
        assert_eq!(h.rows(), 64);
        assert_eq!(h.bytes(), 64 << 10);
        assert_eq!(h.row(0).expect("oldest kept").at_us, 1_000_000 + 136 * 500_000);
        assert_eq!(h.row(63).expect("newest kept").at_us, 1_000_000 + 199 * 500_000);
        assert_eq!(h.seconds(), 31.5);
    }

    /// The dial moving widens the axis instead of emptying it: a walk
    /// across a band is one picture of the band, not a picture of the last
    /// step. Every row keeps the bins it covered and nothing else.
    #[test]
    fn a_retune_widens_the_axis_and_keeps_what_was_heard() {
        let mut h = Heat::default();
        h.tuned(Hz(433_000_000), 1_000_000.0);
        for i in 0..10u64 {
            h.push(i * 1000, &vec![-90.0f32; 100]);
        }
        assert_eq!((h.rows(), h.bins()), (10, 100), "one span, its own bins");
        assert_eq!(h.bytes(), 1000, "and only the bins each row covered");

        // A megahertz up: the axis is now two megahertz wide, the old rows
        // sit at the bottom of it and the new ones at the top.
        h.tuned(Hz(434_000_000), 1_000_000.0);
        h.push(20_000, &vec![-40.0f32; 100]);
        assert_eq!((h.rows(), h.bins()), (11, 200));
        assert_eq!(h.bytes(), 1100, "a row is still only the span it covered");
        assert_eq!(h.rate(), 2_000_000.0);
        assert_eq!(h.center(), Hz(433_500_000));
        assert_eq!(h.row(0).expect("the first row").bin0, 0);
        assert_eq!(h.row(10).expect("the new row").bin0, 100);

        // And a megahertz below the first, which moves the origin down and
        // takes every row with it.
        h.tuned(Hz(432_000_000), 1_000_000.0);
        h.push(30_000, &vec![-60.0f32; 100]);
        assert_eq!((h.rows(), h.bins()), (12, 300));
        assert_eq!(h.row(0).expect("the first row").bin0, 100, "shifted up the axis");
        assert_eq!(h.row(11).expect("the newest row").bin0, 0);
        assert_eq!(h.bin_hz(0), 431_500_000.0 + 5_000.0, "bin zero is the new low edge");
    }

    /// What a row does not cover reads as unlistened rather than as a quiet
    /// band, which is the difference between a walk's picture and a lie.
    #[test]
    fn a_row_is_unread_where_the_receiver_was_not_listening() {
        let mut h = Heat::default();
        h.tuned(Hz(433_000_000), 1_000_000.0);
        h.push(0, &vec![-90.0f32; 100]);
        h.tuned(Hz(434_000_000), 1_000_000.0);
        h.push(1000, &vec![-40.0f32; 100]);
        let old = h.dense_row(0);
        let new = h.dense_row(1);
        assert_eq!(old.len(), 200);
        // To the nearest step, which is what a byte a reading costs.
        assert_eq!(dequantise(old[50]), -89.75);
        assert_eq!(old[150], UNREAD, "the second span was not heard on the first row");
        assert_eq!(new[50], UNREAD);
        assert_eq!(dequantise(new[150]), -40.25);
    }

    /// A span at a different rate lands on the axis it finds rather than
    /// starting a new one: the readings are resampled to the bin the
    /// heatmap already has.
    #[test]
    fn a_wider_span_is_resampled_onto_the_axis_already_there() {
        let mut h = Heat::default();
        h.tuned(Hz(433_000_000), 1_000_000.0);
        h.push(0, &vec![-90.0f32; 100]);
        assert_eq!(h.bins(), 100, "10 kHz a bin");
        // Twice the span at the same bin count is twice the bins on this
        // axis, and it covers the first span.
        h.tuned(Hz(433_000_000), 2_000_000.0);
        h.push(1000, &vec![-50.0f32; 100]);
        assert_eq!(h.bins(), 200);
        assert_eq!(h.rate(), 2_000_000.0);
        let new = h.dense_row(1);
        assert_eq!(new.len(), 200);
        assert_eq!(dequantise(new[0]), -50.0);
        assert_eq!(dequantise(new[199]), -50.0);
    }

    #[test]
    fn a_bin_knows_which_frequency_it_holds() {
        let mut h = Heat::default();
        h.tuned(Hz(433_000_000), 2_400_000.0);
        h.push(0, &vec![-90.0f32; 1024]);
        // The middle of the bin, not its edge: a reading is of the whole
        // bin and the axis is 2343.75 Hz a step here.
        assert_eq!(h.bin_hz(0), 433_000_000.0 - 1_200_000.0 + 1171.875);
        assert_eq!(h.bin_hz(512), 433_000_000.0 + 1171.875);
        assert_eq!(h.bin_hz(768), 433_000_000.0 + 600_000.0 + 1171.875);
    }

    fn recorded() -> Heat {
        let mut h = Heat::new(1 << 20);
        h.tuned(Hz(433_000_000), 2_400_000.0);
        for i in 0..20u64 {
            let mut row = vec![-95.0f32; 128];
            row[40] = -30.0;
            h.push(1_700_000_000_000_000 + i * 500_000, &row);
        }
        h
    }

    #[test]
    fn an_empty_heatmap_refuses_to_export() {
        let h = Heat::default();
        assert!(html(&h, Ramp::Grey, -100.0, -20.0).is_err());
    }

    #[test]
    fn the_html_holds_the_picture_and_the_exact_readings() {
        let h = recorded();
        let page = html(&h, Ramp::Viridis, -100.0, -20.0).expect("a page");
        assert!(page.contains("\"center\":433000000"));
        // The page paints the readings itself, so it carries the ramp and
        // the scale rather than a picture of them.
        assert!(page.contains("\"ramp\":[["));
        assert!(page.contains("\"floor\":-100"));
        assert!(!page.contains("data:image/png"), "no picture to go stale against the readings");
        assert!(page.contains("\"bins\":128"));
        assert!(page.contains("\"rows\":20"));
        let data = readings_of(&page);
        assert_eq!(data.len(), 20 * 128);
        // Newest row first, and the loud bin reads what was put in it, to
        // the nearest step: -30 dBFS is not a multiple of DB_STEP above
        // DB_BASE and comes back as -29.75.
        assert_eq!(dequantise(data[40]), -29.75);
        assert_eq!(dequantise(data[41]), -95.0);
        // The times are what the clock read, in Unix milliseconds, newest
        // first: 2023-11-14 22:13:20 UTC, half a second between rows. A
        // page of seconds-ago says nothing about when the file was made.
        let at = page.find("\"at\":[").expect("row times") + 6;
        let end = page[at..].find(']').expect("closing bracket") + at;
        let times: Vec<u64> =
            page[at..end].split(',').map(|v| v.parse().expect("a number")).collect();
        assert_eq!(times.len(), 20);
        assert_eq!(times[0], 1_700_000_009_500);
        assert_eq!(times[19], 1_700_000_000_000);
        // A crosshair follows the pointer, so a reading can be lined up
        // against a frequency at one end and a time at the other.
        assert!(page.contains("function crosshair"), "a crosshair follows the pointer");
        assert!(page.contains("addEventListener('wheel'"), "the wheel zooms");
        assert!(page.contains("function pan("), "and a drag pans");
        // What made it and when, on the page and in its metadata: a file
        // passed on to somebody else has to say where it came from.
        assert!(page.contains(&format!("\"waveshark\":\"{}\"", crate::update::running())));
        assert!(page.contains("\"generated\":"));
        assert!(page.contains("from <b>2023-11-14 22:13:20 UTC</b>"), "when it starts");
        assert!(page.contains("to <b>2023-11-14 22:13:29 UTC</b>"), "when it ends");
        assert!(page.contains(&format!("by WaveShark {}", crate::update::running())));
    }

    /// A walk's axis is tens of thousands of bins wide, and the page holds
    /// every one of them: a reading folded away on the way out is a reading
    /// nobody can get back.
    #[test]
    fn a_very_wide_axis_is_written_a_bin_at_a_time() {
        let mut h = Heat::new(1 << 24);
        // Twenty steps of 2 MHz at 1 kHz a bin: 40,000 bins of axis.
        for step in 0..20u64 {
            h.tuned(Hz(400_000_000 + step * 2_000_000), 2_000_000.0);
            let mut row = vec![-95.0f32; 2000];
            row[1000] = -20.0;
            h.push(1_700_000_000_000_000 + step * 500_000, &row);
        }
        assert_eq!(h.bins(), 40_000);
        let page = html(&h, Ramp::Viridis, -100.0, -20.0).expect("a page");
        assert!(page.contains("\"bins\":40000"));
        assert_eq!(readings_of(&page).len(), 20 * 40_000, "every bin of every row");
    }

    /// The readings a page carries: out of its own tag, base64 off, gzip
    /// off. What the page's own script does, so the test reads what a
    /// browser would.
    fn readings_of(page: &str) -> Vec<u8> {
        use base64::Engine as _;
        use std::io::Read as _;
        let tag = page.find("id=\"readings\">").expect("a readings tag") + 14;
        let end = page[tag..].find("</script>").expect("closed") + tag;
        let raw = base64::engine::general_purpose::STANDARD
            .decode(page[tag..end].trim())
            .expect("valid base64");
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(raw.as_slice()).read_to_end(&mut out).expect("gzip");
        out
    }

    /// One frame of the display's spectrum, as the stage above publishes it.
    fn frame(at_us: u64, bins: usize) -> common::SpectrumFrame {
        common::SpectrumFrame {
            at_us,
            center_hz: 433_000_000.0,
            span_hz: 240_000.0,
            db: std::sync::Arc::new(vec![-95.0f32; bins]),
        }
    }

    fn feed(n: &mut HeatmapNode, frames: Vec<common::SpectrumFrame>) {
        let spec = StreamSpec::iq(240_000.0, Hz(433_000_000)).with_kind(PortKind::Spectrum);
        let ins = [PortSpec { spec, latency: 0 }];
        let mut out = Payload::Spectrum(Vec::new());
        let (mut events, mut tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
        Simple::process(n, &Payload::Spectrum(frames), &mut out, &mut ctx).unwrap();
    }

    fn accept(n: &mut HeatmapNode) {
        let spec = StreamSpec::iq(240_000.0, Hz(433_000_000)).with_kind(PortKind::Spectrum);
        Node::negotiate(n, &[PortSpec { spec, latency: 0 }]).expect("a spectrum is accepted");
    }

    /// The display's frames come at its own rate; the recorder keeps one an
    /// interval and drops the rest, which is what lets a waterfall at thirty
    /// a second and a night's recording at one every ten share a transform.
    #[test]
    fn the_recorder_keeps_one_frame_an_interval_and_drops_the_rest() {
        let mut n = HeatmapNode::new(1 << 20);
        n.set_rows_per_sec(4.0);
        accept(&mut n);
        // Ten seconds of display frames at thirty a second.
        let start = 1_700_000_000_000_000u64;
        let frames: Vec<_> = (0..300).map(|i| frame(start + i * 33_333, 1024)).collect();
        feed(&mut n, frames);
        let s = n.status();
        // Ten seconds of frames 33.3 ms apart, one kept every 250 ms: the
        // first, then one whenever a quarter of a second has passed, which
        // lands on every eighth frame and comes to 38.
        assert_eq!(s.rows, 38);
        assert_eq!(s.bins, 1024, "the display's bins, not a size of its own");
        assert_eq!(s.bytes, 38 * 1024);
    }

    #[test]
    fn a_heatmap_refuses_samples_because_it_reads_bins() {
        let mut n = HeatmapNode::new(1 << 20);
        let iq = [PortSpec { spec: StreamSpec::iq(240_000.0, Hz(433_000_000)), latency: 0 }];
        assert!(Node::negotiate(&mut n, &iq).is_err(), "the transform is the stage above");
    }

    #[test]
    fn nothing_is_kept_while_the_recorder_is_off() {
        let mut n = HeatmapNode::new(1 << 20);
        n.set_recording(false);
        accept(&mut n);
        let start = 1_700_000_000_000_000u64;
        feed(&mut n, (0..300).map(|i| frame(start + i * 33_333, 1024)).collect());
        assert_eq!(n.status().rows, 0);
    }

    #[test]
    fn an_export_writes_a_file_and_says_where_it_went() {
        let dir = std::env::temp_dir().join(format!("sr-heat-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut n = HeatmapNode::new(1 << 20);
        n.heat = recorded();
        let path = n.export(&dir, Ramp::Grey, -100.0, -20.0).expect("a page");
        assert!(path.to_string_lossy().ends_with(".html"));
        assert!(path.to_string_lossy().contains("433.0000M"));
        assert_eq!(n.status().saved, Some(path.clone()));
        assert_eq!(n.status().error, None);
        assert!(std::fs::metadata(&path).expect("written").len() > 100);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
