//! Scrolling waterfall.
//!
//! Rows are written into a ring buffer and the texture is uploaded whole each
//! frame. Scrolling by rewriting one row and moving a cursor keeps the cost
//! independent of history depth.

use egui::{Color32, ColorImage, Pos2, TextureHandle, TextureOptions};

/// Widest texture asked for. wgpu reports 8192 as the limit on the hardware
/// this runs on, and the API refuses wider rather than clamping.
const MAX_WIDTH: usize = 8192;

pub struct Waterfall {
    width: usize,
    /// Spectrum bins per column, above one only when a row is wider than a
    /// texture may be.
    factor: usize,
    height: usize,
    /// Ring of rows, newest at `cursor - 1`.
    pixels: Vec<Color32>,
    cursor: usize,
    filled: usize,
    tex: Option<TextureHandle>,
    /// Whole texture needs re-uploading (resize, clear, or a pan).
    dirty_all: bool,
    /// Only this row changed since the last upload.
    dirty_row: Option<usize>,
    row_buf: Vec<Color32>,
}

impl Waterfall {
    pub fn new(height: usize) -> Self {
        Self {
            width: 0,
            factor: 1,
            height,
            pixels: Vec::new(),
            cursor: 0,
            filled: 0,
            tex: None,
            dirty_all: true,
            dirty_row: None,
            row_buf: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.pixels.fill(Color32::BLACK);
        self.cursor = 0;
        self.filled = 0;
        self.dirty_all = true;
        self.dirty_row = None;
    }

    /// Add one spectrum row, mapping dB through the given display range.
    ///
    /// A row wider than a texture may be is folded down first, several bins
    /// to a column with the loudest kept, since a narrow burst that falls in
    /// one bin has to stay visible. The GPU's limit is 8192 columns on most
    /// hardware, and a 16384-point spectrum asked for a texture that wide
    /// and took the window down with a validation error.
    pub fn push(&mut self, db: &[f32], floor: f32, ceil: f32) {
        if db.is_empty() {
            return;
        }
        let factor = db.len().div_ceil(MAX_WIDTH).max(1);
        let width = db.len().div_ceil(factor);
        if width != self.width || factor != self.factor {
            self.width = width;
            self.factor = factor;
            self.pixels = vec![Color32::BLACK; self.width * self.height];
            self.row_buf = vec![Color32::BLACK; self.width];
            self.cursor = 0;
            self.filled = 0;
            self.dirty_all = true;
            self.tex = None;
        }
        let span = (ceil - floor).max(1.0);
        let row = self.cursor * self.width;
        for (i, chunk) in db.chunks(factor).enumerate() {
            let v = chunk.iter().copied().fold(f32::MIN, f32::max);
            self.pixels[row + i] = colormap(((v - floor) / span).clamp(0.0, 1.0));
        }
        self.dirty_row = Some(self.cursor);
        self.cursor = (self.cursor + 1) % self.height;
        self.filled = (self.filled + 1).min(self.height);
    }

    /// Change how many rows of history are kept.
    pub fn set_height(&mut self, rows: usize) {
        let rows = rows.max(16);
        if rows == self.height {
            return;
        }
        self.height = rows;
        self.pixels = vec![Color32::BLACK; self.width * rows];
        self.cursor = 0;
        self.filled = 0;
        self.tex = None;
        self.dirty_all = true;
        self.dirty_row = None;
    }

    pub fn height(&self) -> usize {
        self.height
    }

    /// Slide the history sideways when the radio is retuned.
    ///
    /// Panning is a shift of the frequency axis, not a new view, so the rows
    /// already captured are still valid where they overlap. Clearing on every
    /// retune wipes the display continuously while the pointer is dragging,
    /// which looks like the waterfall has stopped.
    pub fn shift(&mut self, d: i32) {
        // `d` is in spectrum bins; the columns may be coarser.
        let d = d / self.factor.max(1) as i32;
        if d == 0 || self.width == 0 {
            return;
        }
        let w = self.width;
        if d.unsigned_abs() as usize >= w {
            self.pixels.fill(Color32::BLACK);
            self.dirty_all = true;
            return;
        }
        let n = d.unsigned_abs() as usize;
        for y in 0..self.height {
            let row = &mut self.pixels[y * w..(y + 1) * w];
            if d > 0 {
                // Centre moved up, so content moves left.
                row.copy_within(n.., 0);
                row[w - n..].fill(Color32::BLACK);
            } else {
                row.copy_within(..w - n, n);
                row[..n].fill(Color32::BLACK);
            }
        }
        // Every row moved, so a partial upload cannot express this.
        self.dirty_all = true;
    }

    /// Push pending pixels to the GPU and draw, newest row at the top.
    ///
    /// Rows are stored in ring order and never rotated, so a new row costs one
    /// row of upload instead of rebuilding the whole image. At 2048 bins and
    /// 512 rows a full rebuild is 4 MB, which at 20 rows a second is 84 MB/s
    /// of texture traffic for one changed row.
    pub fn draw(&mut self, ctx: &egui::Context, p: &egui::Painter, rect: egui::Rect) {
        if self.width == 0 || self.height == 0 {
            return;
        }
        if self.tex.is_none() {
            let img = ColorImage::filled([self.width, self.height], Color32::BLACK);
            self.tex = Some(ctx.load_texture("waterfall", img, TextureOptions::LINEAR));
            self.dirty_all = true;
        }
        let tex = self.tex.as_mut().expect("texture just created");

        if self.dirty_all {
            let mut img = ColorImage::filled([self.width, self.height], Color32::BLACK);
            img.pixels.copy_from_slice(&self.pixels);
            tex.set(img, TextureOptions::LINEAR);
            self.dirty_all = false;
            self.dirty_row = None;
        } else if let Some(r) = self.dirty_row.take() {
            self.row_buf.copy_from_slice(&self.pixels[r * self.width..(r + 1) * self.width]);
            let mut img = ColorImage::filled([self.width, 1], Color32::BLACK);
            img.pixels.copy_from_slice(&self.row_buf);
            tex.set_partial([0, r], img, TextureOptions::LINEAR);
        }

        if self.filled == 0 {
            return;
        }
        let id = tex.id();
        let h = self.height as f32;
        let c = self.cursor;
        let row_px = rect.height() / h;

        // Segment from the start of the ring, newest first, so the V range is
        // inverted: screen top maps to the most recently written row.
        let a_rows = c.min(self.filled);
        let mut y = rect.top();
        if a_rows > 0 {
            let bottom = y + a_rows as f32 * row_px;
            p.image(
                id,
                egui::Rect::from_min_max(Pos2::new(rect.left(), y), Pos2::new(rect.right(), bottom)),
                egui::Rect::from_min_max(
                    Pos2::new(0.0, c as f32 / h),
                    Pos2::new(1.0, (c - a_rows) as f32 / h),
                ),
                Color32::WHITE,
            );
            y = bottom;
        }
        // Older rows wrapped around the end of the ring.
        let b_rows = self.filled.saturating_sub(a_rows);
        if b_rows > 0 {
            let bottom = y + b_rows as f32 * row_px;
            p.image(
                id,
                egui::Rect::from_min_max(Pos2::new(rect.left(), y), Pos2::new(rect.right(), bottom)),
                egui::Rect::from_min_max(
                    Pos2::new(0.0, 1.0),
                    Pos2::new(1.0, (self.height - b_rows) as f32 / h),
                ),
                Color32::WHITE,
            );
        }
    }
}

/// Cold cyan for noise, hot amber for signal, white at the top.
///
/// Built from the theme's two accents rather than a stock inferno ramp, so the
/// waterfall says the same thing the rest of the panel does: cyan is what the
/// radio hears, amber is where the energy is. Brightness rises monotonically
/// so features stay readable in greyscale and for colour-deficient viewers.
pub fn colormap(t: f32) -> Color32 {
    const STOPS: [(f32, [f32; 3]); 6] = [
        // Most bins in any span are noise, so the ramp stays dark well past
        // the midpoint. Brightening early spends the whole scale on the noise
        // floor and leaves signals nowhere to go.
        (0.00, [0.031, 0.039, 0.051]),
        (0.35, [0.047, 0.125, 0.161]),
        (0.60, [0.078, 0.376, 0.486]),
        (0.78, [0.180, 0.612, 0.745]),
        (0.90, [0.941, 0.627, 0.188]),
        (1.00, [1.0, 0.965, 0.878]),
    ];
    let t = t.clamp(0.0, 1.0);
    let mut i = 0;
    while i + 2 < STOPS.len() && t > STOPS[i + 1].0 {
        i += 1;
    }
    let (a, b) = (STOPS[i], STOPS[i + 1]);
    let f = ((t - a.0) / (b.0 - a.0)).clamp(0.0, 1.0);
    let c = |x: usize| ((a.1[x] + (b.1[x] - a.1[x]) * f) * 255.0) as u8;
    Color32::from_rgb(c(0), c(1), c(2))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn luma(c: Color32) -> f32 {
        0.2126 * c.r() as f32 + 0.7152 * c.g() as f32 + 0.0722 * c.b() as f32
    }


    #[test]
    fn the_colormap_brightens_monotonically() {
        let mut prev = -1.0;
        for i in 0..=100 {
            let l = luma(colormap(i as f32 / 100.0));
            assert!(l >= prev - 1.0, "luma dipped at t={}: {l} after {prev}", i as f32 / 100.0);
            prev = l;
        }
    }

    #[test]
    fn the_colormap_clamps_out_of_range_input() {
        assert_eq!(colormap(-5.0), colormap(0.0));
        assert_eq!(colormap(5.0), colormap(1.0));
    }

    #[test]
    fn a_resize_does_not_panic_and_resets() {
        let mut w = Waterfall::new(8);
        w.push(&[-50.0; 16], -100.0, 0.0);
        w.push(&[-50.0; 32], -100.0, 0.0);
        assert_eq!(w.width, 32);
        assert_eq!(w.filled, 1);
    }

    #[test]
    fn the_ring_wraps_without_growing() {
        let mut w = Waterfall::new(4);
        for _ in 0..100 {
            w.push(&[-30.0; 8], -100.0, 0.0);
        }
        assert_eq!(w.pixels.len(), 8 * 4);
        assert_eq!(w.filled, 4);
    }

    #[test]
    fn resizing_history_drops_the_texture_so_it_is_rebuilt() {
        let mut w = Waterfall::new(32);
        w.push(&[-50.0; 16], -100.0, 0.0);
        w.set_height(64);
        assert_eq!(w.height(), 64);
        assert_eq!(w.filled, 0, "old rows no longer line up with the new ring");
        assert!(w.dirty_all);
    }

    #[test]
    fn resizing_to_the_same_height_is_a_no_op() {
        let mut w = Waterfall::new(32);
        w.push(&[-50.0; 16], -100.0, 0.0);
        w.set_height(32);
        assert_eq!(w.filled, 1, "history was thrown away needlessly");
    }

    #[test]
    fn history_has_a_floor_so_the_ring_stays_usable() {
        let mut w = Waterfall::new(32);
        w.set_height(1);
        assert!(w.height() >= 16, "height fell to {}", w.height());
    }

    #[test]
    fn the_ring_cursor_wraps_and_tracks_what_is_valid() {
        let mut w = Waterfall::new(4);
        assert_eq!((w.cursor, w.filled), (0, 0));
        w.push(&[-50.0; 8], -100.0, 0.0);
        assert_eq!((w.cursor, w.filled), (1, 1));
        for _ in 0..20 {
            w.push(&[-50.0; 8], -100.0, 0.0);
        }
        assert_eq!(w.filled, 4, "filled must saturate at the ring size");
        assert!(w.cursor < 4, "cursor escaped the ring: {}", w.cursor);
    }

    #[test]
    fn a_new_row_asks_for_a_partial_upload_not_a_full_one() {
        let mut w = Waterfall::new(8);
        w.push(&[-50.0; 16], -100.0, 0.0);
        w.dirty_all = false;
        w.dirty_row = None;
        w.push(&[-50.0; 16], -100.0, 0.0);
        assert_eq!(w.dirty_row, Some(1), "one changed row should be a partial upload");
        assert!(!w.dirty_all, "a single row must not force a full re-upload");
    }

    #[test]
    fn panning_forces_a_full_upload_because_every_row_moved() {
        let mut w = Waterfall::new(8);
        w.push(&[-50.0; 16], -100.0, 0.0);
        w.dirty_all = false;
        w.shift(3);
        assert!(w.dirty_all);
    }

    #[test]
    fn shifting_moves_content_and_blanks_the_vacated_edge() {
        let mut w = Waterfall::new(2);
        let mut db = vec![-100.0f32; 8];
        db[4] = 0.0;
        w.push(&db, -100.0, 0.0);
        let before = w.pixels[4];
        w.shift(2);
        assert_eq!(w.pixels[2], before, "content did not move left");
        assert_eq!(w.pixels[7], Color32::BLACK, "vacated edge not blanked");
    }

    #[test]
    fn shifting_the_other_way_moves_right() {
        let mut w = Waterfall::new(2);
        let mut db = vec![-100.0f32; 8];
        db[2] = 0.0;
        w.push(&db, -100.0, 0.0);
        let before = w.pixels[2];
        w.shift(-2);
        assert_eq!(w.pixels[4], before);
        assert_eq!(w.pixels[0], Color32::BLACK);
    }

    #[test]
    fn a_row_wider_than_a_texture_is_folded_down() {
        let mut w = Waterfall::new(16);
        let mut db = vec![-90.0f32; 16384];
        db[10_000] = -20.0;
        w.push(&db, -100.0, 0.0);
        assert_eq!(w.width, 8192, "two bins to a column");
        // The loud bin survives the fold, in the column that holds it.
        assert_ne!(w.pixels[5000], w.pixels[4999]);
    }

    #[test]
    fn shifting_further_than_the_width_clears_everything() {
        let mut w = Waterfall::new(2);
        w.push(&[0.0f32; 8], -100.0, 0.0);
        w.shift(99);
        assert!(w.pixels.iter().all(|p| *p == Color32::BLACK));
    }

    #[test]
    fn a_zero_shift_is_a_no_op() {
        let mut w = Waterfall::new(2);
        let mut db = vec![-100.0f32; 8];
        db[3] = 0.0;
        w.push(&db, -100.0, 0.0);
        let snapshot = w.pixels.clone();
        w.shift(0);
        assert_eq!(w.pixels, snapshot);
    }

    #[test]
    fn a_strong_bin_is_brighter_than_a_weak_one() {
        let strong = colormap((-10.0f32 + 100.0) / 100.0);
        let weak = colormap((-90.0f32 + 100.0) / 100.0);
        assert!(luma(strong) > luma(weak));
    }
}
