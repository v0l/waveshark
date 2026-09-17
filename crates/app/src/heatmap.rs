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

/// Which file an export asks for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Export {
    /// Flat pixels with the axes burned in.
    #[default]
    Png,
    /// The same picture with the readings beside it, so a pointer over a
    /// point gives the time, the frequency and the decibels.
    Html,
}

impl Export {
    pub fn label(self) -> &'static str {
        match self {
            Export::Png => "png",
            Export::Html => "html",
        }
    }

    pub fn extension(self) -> &'static str {
        self.label()
    }
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
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Row {
    /// Unix microseconds the row was finished at.
    pub at_us: u64,
    pub db: Vec<u8>,
}

/// The readings, bounded.
#[derive(Clone, Debug)]
pub struct Heat {
    rows: VecDeque<Row>,
    bins: usize,
    center: Hz,
    rate: f64,
    budget: usize,
    bytes: usize,
}

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

    /// Where the readings were taken. A row is only meaningful against the
    /// axis it was read on, so a retune or a change of bin count throws the
    /// history away rather than exporting two different frequency axes as
    /// one picture.
    pub fn tuned(&mut self, center: Hz, rate: f64) {
        if center == self.center && rate == self.rate {
            return;
        }
        self.center = center;
        self.rate = rate;
        self.clear();
    }

    pub fn clear(&mut self) {
        self.rows.clear();
        self.bytes = 0;
    }

    pub fn push(&mut self, at_us: u64, db: &[f32]) {
        if db.is_empty() {
            return;
        }
        if db.len() != self.bins {
            self.bins = db.len();
            self.clear();
        }
        let row = Row { at_us, db: db.iter().map(|v| quantise(*v)).collect() };
        self.bytes += row.db.len();
        self.rows.push_back(row);
        self.trim();
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

    pub fn center(&self) -> Hz {
        self.center
    }

    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Oldest to newest.
    pub fn row(&self, i: usize) -> Option<&Row> {
        self.rows.get(i)
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
        let frac = bin as f64 / self.bins as f64 - 0.5;
        self.center.as_f64() + frac * self.rate
    }

    /// The readings as pixels, newest row first, with no axes.
    fn pixels(&self, ramp: Ramp, floor: f32, ceil: f32) -> Vec<u8> {
        let span = (ceil - floor).max(1.0);
        let mut out = Vec::with_capacity(self.rows.len() * self.bins * 3);
        for row in self.rows.iter().rev() {
            for v in &row.db {
                let t = (dequantise(*v) - floor) / span;
                out.extend_from_slice(&ramp.sample(t));
            }
        }
        out
    }
}

/// How much of the picture the axes take.
const LEFT: usize = 60;
const BOTTOM: usize = 16;
/// Chassis dark, so an export looks like the panel it came from.
const PAPER: [u8; 3] = [7, 9, 13];
const INK: [u8; 3] = [154, 168, 178];

/// The picture, with the time down the left and the frequency along the
/// bottom, as PNG bytes.
pub fn png(heat: &Heat, ramp: Ramp, floor: f32, ceil: f32) -> Result<Vec<u8>> {
    let (rows, bins) = (heat.rows(), heat.bins());
    if rows == 0 || bins == 0 {
        return Err(common::Error::other("nothing has been recorded yet"));
    }
    let (w, h) = (LEFT + bins, rows + BOTTOM);
    let mut img = vec![0u8; w * h * 3];
    for p in img.chunks_exact_mut(3) {
        p.copy_from_slice(&PAPER);
    }
    let heat_px = heat.pixels(ramp, floor, ceil);
    for y in 0..rows {
        let src = y * bins * 3;
        let dst = (y * w + LEFT) * 3;
        img[dst..dst + bins * 3].copy_from_slice(&heat_px[src..src + bins * 3]);
    }

    // Five frequency ticks across the span, labelled in megahertz. Three
    // decimals is a kilohertz, which is as fine as a label this size can be
    // read and finer than a bin at any span the receiver samples.
    let mut inked_to = 0;
    for k in 0..=4 {
        let bin = bins * k / 4;
        let x = (LEFT + bin.min(bins - 1)).min(w - 1);
        for y in rows..(rows + 4).min(h) {
            put(&mut img, w, h, x, y, INK);
        }
        let label = format!("{:.3}", heat.bin_hz(bin) / 1e6);
        let width = text_width(&label);
        let lx = x.saturating_sub(width / 2).min(w.saturating_sub(width));
        // A narrow span is a narrow picture, and two labels drawn over each
        // other are worse than one: the tick is still there to read against.
        if k > 0 && lx < inked_to + 4 {
            continue;
        }
        inked_to = lx + width;
        text(&mut img, w, h, lx, rows + 5, &label, INK);
    }

    // Time down the left, in seconds before the newest row, which is the
    // reading somebody looking at a heatmap actually wants: a wall clock
    // says when the file was written, not how long ago the burst was.
    let newest = heat.row(rows - 1).map(|r| r.at_us).unwrap_or(0);
    let step = (rows / 4).max(1);
    for y in (0..rows).step_by(step) {
        let at = heat.row(rows - 1 - y).map(|r| r.at_us).unwrap_or(newest);
        let ago = (newest.saturating_sub(at)) as f64 / 1e6;
        let label = match ago < 0.5 {
            true => "0s".to_string(),
            false => format!("-{ago:.0}s"),
        };
        for x in LEFT - 4..LEFT {
            put(&mut img, w, h, x, y, INK);
        }
        text(&mut img, w, h, LEFT - 6 - text_width(&label), y.min(h - 6), &label, INK);
    }

    let mut out = Vec::new();
    let enc = image::codecs::png::PngEncoder::new(&mut out);
    image::ImageEncoder::write_image(enc, &img, w as u32, h as u32, image::ExtendedColorType::Rgb8)
        .map_err(|e| common::Error::other(format!("cannot encode the heatmap: {e}")))?;
    Ok(out)
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
    let picture = png(heat, ramp, floor, ceil)?;
    let b64 = base64::engine::general_purpose::STANDARD;
    let img = b64.encode(&picture);
    // Newest row first, to match the picture.
    let mut readings = Vec::with_capacity(rows * bins);
    let mut times = Vec::with_capacity(rows);
    let newest = heat.row(rows - 1).map(|r| r.at_us).unwrap_or(0);
    for i in (0..rows).rev() {
        let r = heat.row(i).expect("row in range");
        readings.extend_from_slice(&r.db);
        times.push(format!("{:.3}", (newest.saturating_sub(r.at_us)) as f64 / 1e6));
    }
    let data = b64.encode(&readings);
    let meta = format!(
        "{{\"center\":{},\"rate\":{},\"bins\":{},\"rows\":{},\"left\":{},\"bottom\":{},\
         \"base\":{},\"step\":{},\"ago\":[{}]}}",
        heat.center().as_f64(),
        heat.rate(),
        bins,
        rows,
        LEFT,
        BOTTOM,
        DB_BASE,
        DB_STEP,
        times.join(",")
    );
    Ok(format!(
        "<!doctype html>\n<meta charset=\"utf-8\">\n<title>WaveShark heatmap {:.4} MHz</title>\n\
         <style>body{{background:#070909;color:#9aa8b2;font:12px monospace;margin:16px}}\
         #p{{position:relative;display:inline-block}}#p img{{display:block;image-rendering:pixelated}}\
         #r{{margin-top:8px;color:#efc02f}}</style>\n\
         <div id=\"p\"><img id=\"i\" src=\"data:image/png;base64,{}\"></div>\n\
         <div id=\"r\">move the pointer over the heatmap</div>\n\
         <script>\nconst m={};\nconst d=Uint8Array.from(atob(\"{}\"),c=>c.charCodeAt(0));\n\
         const i=document.getElementById('i'),r=document.getElementById('r');\n\
         i.addEventListener('mousemove',e=>{{\n\
         const b=i.getBoundingClientRect();\n\
         const x=Math.floor((e.clientX-b.left)*i.naturalWidth/b.width)-m.left;\n\
         const y=Math.floor((e.clientY-b.top)*i.naturalHeight/b.height);\n\
         if(x<0||x>=m.bins||y<0||y>=m.rows){{r.textContent='outside the readings';return}}\n\
         const hz=m.center+(x/m.bins-0.5)*m.rate;\n\
         const db=m.base+d[y*m.bins+x]*m.step;\n\
         const ago=m.ago[y]<0.05?'now':'-'+m.ago[y].toFixed(3)+' s';\n\
         r.textContent=(hz/1e6).toFixed(4)+' MHz  '+ago+'  '+db.toFixed(1)+' dBFS';\n\
         }});\n</script>\n",
        heat.center().as_f64() / 1e6,
        img,
        meta,
        data
    ))
}

fn put(img: &mut [u8], w: usize, h: usize, x: usize, y: usize, c: [u8; 3]) {
    if x >= w || y >= h {
        return;
    }
    let i = (y * w + x) * 3;
    img[i..i + 3].copy_from_slice(&c);
}

/// Three pixels a glyph and one of gap, at double size.
const SCALE: usize = 2;
const GLYPH_W: usize = 3;
const GLYPH_H: usize = 5;

fn text_width(s: &str) -> usize {
    s.chars().count() * (GLYPH_W + 1) * SCALE
}

fn text(img: &mut [u8], w: usize, h: usize, x: usize, y: usize, s: &str, c: [u8; 3]) {
    let mut cx = x;
    for ch in s.chars() {
        let g = glyph(ch);
        for (row, bits) in g.iter().enumerate() {
            for col in 0..GLYPH_W {
                if bits & (1 << (GLYPH_W - 1 - col)) == 0 {
                    continue;
                }
                for dy in 0..SCALE {
                    for dx in 0..SCALE {
                        put(img, w, h, cx + col * SCALE + dx, y + row * SCALE + dy, c);
                    }
                }
            }
        }
        cx += (GLYPH_W + 1) * SCALE;
    }
}

/// A 3x5 font, enough for a frequency and a time. Burned into the picture
/// rather than drawn with a real typeface because an export must not depend
/// on a font being installed, and these are the only characters an axis
/// label uses.
fn glyph(c: char) -> [u8; GLYPH_H] {
    match c {
        '0' => [0b111, 0b101, 0b101, 0b101, 0b111],
        '1' => [0b010, 0b110, 0b010, 0b010, 0b111],
        '2' => [0b111, 0b001, 0b111, 0b100, 0b111],
        '3' => [0b111, 0b001, 0b111, 0b001, 0b111],
        '4' => [0b101, 0b101, 0b111, 0b001, 0b001],
        '5' => [0b111, 0b100, 0b111, 0b001, 0b111],
        '6' => [0b111, 0b100, 0b111, 0b101, 0b111],
        '7' => [0b111, 0b001, 0b010, 0b010, 0b010],
        '8' => [0b111, 0b101, 0b111, 0b101, 0b111],
        '9' => [0b111, 0b101, 0b111, 0b001, 0b111],
        '.' => [0b000, 0b000, 0b000, 0b000, 0b010],
        '-' => [0b000, 0b000, 0b111, 0b000, 0b000],
        ':' => [0b000, 0b010, 0b000, 0b010, 0b000],
        'M' => [0b101, 0b111, 0b111, 0b101, 0b101],
        'H' => [0b101, 0b101, 0b111, 0b101, 0b101],
        'k' => [0b100, 0b101, 0b110, 0b101, 0b101],
        'z' => [0b111, 0b001, 0b010, 0b100, 0b111],
        's' => [0b011, 0b100, 0b010, 0b001, 0b110],
        _ => [0; GLYPH_H],
    }
}

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

/// The recorder: its own transform over the span, at its own rate.
///
/// Its own rather than the display's because the point of the thing is a
/// heatmap of a night nobody watched, and because a waterfall scrolling at
/// twenty rows a second would fill any budget in minutes.
pub struct HeatmapNode {
    spec: dsp::Spectrum,
    heat: Heat,
    recording: bool,
    rows_per_sec: f32,
    rate: f64,
    center: Hz,
    /// Samples still to be discarded before the next row is started, so the
    /// transform runs at the row rate rather than at the block rate.
    debt: f64,
    collecting: bool,
    saved: Option<PathBuf>,
    error: Option<String>,
}

impl Default for HeatmapNode {
    fn default() -> Self {
        Self::new(2048, DEFAULT_BUDGET)
    }
}

impl HeatmapNode {
    pub fn new(size: usize, budget: usize) -> Self {
        Self {
            spec: dsp::Spectrum::new(size.clamp(64, 32_768).next_power_of_two()),
            heat: Heat::new(budget),
            recording: true,
            rows_per_sec: 2.0,
            rate: 0.0,
            center: Hz(0),
            debt: 0.0,
            collecting: true,
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
    pub fn export(
        &mut self,
        dir: &Path,
        what: Export,
        ramp: Ramp,
        floor: f32,
        ceil: f32,
    ) -> Result<PathBuf> {
        let r = self.write(dir, what, ramp, floor, ceil);
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

    fn write(
        &self,
        dir: &Path,
        what: Export,
        ramp: Ramp,
        floor: f32,
        ceil: f32,
    ) -> Result<PathBuf> {
        std::fs::create_dir_all(dir)?;
        let name = format!(
            "heatmap_{}_{:.4}M_{:.0}k.{}",
            chrono::Utc::now().format("%Y%m%d-%H%M%S"),
            self.heat.center().as_f64() / 1e6,
            self.heat.rate() / 1e3,
            what.extension()
        );
        let path = dir.join(name);
        match what {
            Export::Png => std::fs::write(&path, png(&self.heat, ramp, floor, ceil)?)?,
            Export::Html => std::fs::write(&path, html(&self.heat, ramp, floor, ceil)?)?,
        }
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

    fn negotiate(&mut self, i: &PortSpec) -> Result<StreamSpec> {
        if i.spec.kind != PortKind::Iq {
            return Err(common::Error::other("heatmap needs IQ"));
        }
        self.rate = i.spec.rate;
        self.center = i.spec.center;
        self.heat.tuned(i.spec.center, i.spec.rate);
        Ok(i.spec)
    }

    fn process(&mut self, i: &Payload, _o: &mut Payload, _c: &mut NodeCtx<'_>) -> Result<()> {
        let iq = i.as_iq().unwrap_or(&[]);
        if !self.recording || iq.is_empty() {
            return Ok(());
        }
        if !self.collecting {
            self.debt -= iq.len() as f64;
            if self.debt > 0.0 {
                return Ok(());
            }
            self.collecting = true;
        }
        if self.spec.process(iq) {
            let at = now_us();
            self.heat.push(at, self.spec.power_db());
            self.collecting = false;
            self.debt = self.rate / self.rows_per_sec.max(0.02) as f64;
            // Each row is a fresh look rather than an average of the last
            // half hour, which is what an exponential average across a row
            // interval this long would be.
            self.spec.reset();
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
const SIZE: &str = "size";

pub const DESC: StageDesc = StageDesc {
    name: "heatmap",
    summary: "Keep the span as decibels rather than pixels, so it can be \
              exported as a picture with the readings beside it",
    category: Category::Sink,
    feeds_bus: false,
};

pub fn build(s: &Settings) -> Result<Box<dyn pipeline::node::Node>> {
    let size = s.i64_or(SIZE, 2048).clamp(64, 32_768) as usize;
    let mb = s.f64_or(BUDGET_MB, DEFAULT_BUDGET as f64 / (1 << 20) as f64).max(1.0);
    let mut n = HeatmapNode::new(size, (mb * (1 << 20) as f64) as usize);
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
        for i in 0..200u64 {
            h.push(1_000_000 + i * 500_000, &vec![-80.0f32; 1024]);
        }
        assert_eq!(h.rows(), 64);
        assert_eq!(h.bytes(), 64 << 10);
        assert_eq!(h.row(0).expect("oldest kept").at_us, 1_000_000 + 136 * 500_000);
        assert_eq!(h.row(63).expect("newest kept").at_us, 1_000_000 + 199 * 500_000);
        assert_eq!(h.seconds(), 31.5);
    }

    #[test]
    fn a_retune_throws_the_history_away_because_the_axis_moved() {
        let mut h = Heat::default();
        h.tuned(Hz(433_000_000), 2_400_000.0);
        for i in 0..10u64 {
            h.push(i * 1000, &vec![-90.0f32; 256]);
        }
        assert_eq!(h.rows(), 10);
        h.tuned(Hz(433_000_000), 2_400_000.0);
        assert_eq!(h.rows(), 10, "the same tuning is not a change");
        h.tuned(Hz(434_000_000), 2_400_000.0);
        assert_eq!(h.rows(), 0);
    }

    #[test]
    fn a_bin_knows_which_frequency_it_holds() {
        let mut h = Heat::default();
        h.tuned(Hz(433_000_000), 2_400_000.0);
        h.push(0, &vec![-90.0f32; 1024]);
        assert_eq!(h.bin_hz(0), 433_000_000.0 - 1_200_000.0);
        assert_eq!(h.bin_hz(512), 433_000_000.0);
        assert_eq!(h.bin_hz(768), 433_000_000.0 + 600_000.0);
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
    fn the_png_carries_the_readings_and_the_axes() {
        let h = recorded();
        let bytes = png(&h, Ramp::Chassis, -100.0, -20.0).expect("a picture");
        let img = image::load_from_memory(&bytes).expect("valid png").to_rgb8();
        assert_eq!(img.dimensions(), ((LEFT + 128) as u32, (20 + BOTTOM) as u32));
        let hot = img.get_pixel((LEFT + 40) as u32, 0).0;
        let cold = img.get_pixel((LEFT + 41) as u32, 0).0;
        assert!(luma(hot) > luma(cold) + 50.0, "the loud bin is not hotter: {hot:?} {cold:?}");
        // The axis margin is paper, not signal.
        assert_eq!(img.get_pixel(2, 18).0, PAPER);
        // And the labels are drawn in it: count the ink under the heatmap.
        let ink = (0..img.width())
            .flat_map(|x| (20..img.height()).map(move |y| (x, y)))
            .filter(|(x, y)| img.get_pixel(*x, *y).0 == INK)
            .count();
        assert!(ink > 200, "the frequency axis has only {ink} inked pixels");
    }

    #[test]
    fn an_empty_heatmap_refuses_to_export() {
        let h = Heat::default();
        assert!(png(&h, Ramp::Grey, -100.0, -20.0).is_err());
        assert!(html(&h, Ramp::Grey, -100.0, -20.0).is_err());
    }

    #[test]
    fn the_html_holds_the_picture_and_the_exact_readings() {
        let h = recorded();
        let page = html(&h, Ramp::Viridis, -100.0, -20.0).expect("a page");
        assert!(page.contains("data:image/png;base64,"));
        assert!(page.contains("\"center\":433000000"));
        assert!(page.contains("\"bins\":128"));
        assert!(page.contains("\"rows\":20"));
        let b64 = base64::engine::general_purpose::STANDARD;
        let start = page.find("atob(\"").expect("readings") + 6;
        let end = page[start..].find('"').expect("closing quote") + start;
        let data = b64.decode(&page[start..end]).expect("valid base64");
        assert_eq!(data.len(), 20 * 128);
        // Newest row first, and the loud bin reads what was put in it, to
        // the nearest step: -30 dBFS is not a multiple of DB_STEP above
        // DB_BASE and comes back as -29.75.
        assert_eq!(dequantise(data[40]), -29.75);
        assert_eq!(dequantise(data[41]), -95.0);
    }

    fn tone(n: usize, bin_frac: f32) -> Vec<C32> {
        (0..n)
            .map(|k| {
                let p = std::f32::consts::TAU * bin_frac * k as f32;
                C32::new(p.cos() * 0.5, p.sin() * 0.5)
            })
            .collect()
    }

    fn feed(n: &mut HeatmapNode, iq: &[C32], rate: f64) {
        let ins = [PortSpec { spec: StreamSpec::iq(rate, Hz(433_000_000)), latency: 0 }];
        let mut out = Payload::Iq(Vec::new());
        let (mut events, mut tags) = (Vec::new(), Vec::new());
        let mut ctx = NodeCtx::new(0, &ins, &[], &mut events, &mut tags);
        Simple::process(n, &Payload::Iq(iq.to_vec()), &mut out, &mut ctx).unwrap();
    }

    #[test]
    fn the_recorder_keeps_one_row_per_interval_and_no_more() {
        let rate = 240_000.0;
        let mut n = HeatmapNode::new(1024, 1 << 20);
        n.set_rows_per_sec(4.0);
        Node::negotiate(
            &mut n,
            &[PortSpec { spec: StreamSpec::iq(rate, Hz(433_000_000)), latency: 0 }],
        )
        .expect("iq is accepted");
        // Ten seconds of signal in blocks of 12000 samples, which is 50 ms.
        for _ in 0..200 {
            feed(&mut n, &tone(12_000, 0.1), rate);
        }
        let s = n.status();
        // Four a second for ten seconds, less the first interval spent
        // filling the transform.
        assert_eq!(s.rows, 40);
        assert_eq!(s.bins, 1024);
        assert!(!s.recording || s.bytes == 40 * 1024);
    }

    #[test]
    fn nothing_is_kept_while_the_recorder_is_off() {
        let rate = 240_000.0;
        let mut n = HeatmapNode::new(1024, 1 << 20);
        n.set_recording(false);
        Node::negotiate(
            &mut n,
            &[PortSpec { spec: StreamSpec::iq(rate, Hz(433_000_000)), latency: 0 }],
        )
        .expect("iq is accepted");
        for _ in 0..200 {
            feed(&mut n, &tone(12_000, 0.1), rate);
        }
        assert_eq!(n.status().rows, 0);
    }

    #[test]
    fn an_export_writes_a_file_and_says_where_it_went() {
        let dir = std::env::temp_dir().join(format!("sr-heat-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let mut n = HeatmapNode::new(256, 1 << 20);
        n.heat = recorded();
        let png_path = n.export(&dir, Export::Png, Ramp::Chassis, -100.0, -20.0).expect("png");
        let html_path = n.export(&dir, Export::Html, Ramp::Grey, -100.0, -20.0).expect("html");
        assert!(png_path.to_string_lossy().ends_with(".png"));
        assert!(html_path.to_string_lossy().ends_with(".html"));
        assert!(png_path.to_string_lossy().contains("433.0000M"));
        assert_eq!(n.status().saved, Some(html_path.clone()));
        assert_eq!(n.status().error, None);
        assert!(std::fs::metadata(&png_path).expect("written").len() > 100);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
