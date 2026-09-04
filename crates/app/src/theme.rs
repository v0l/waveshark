//! Bench-instrument theme: chassis greys, engraved legends, amber readouts,
//! cyan traces.
//!
//! The two accents carry meaning rather than decoration. Amber is everything
//! you set; cyan is everything the radio hears. Keeping that split consistent
//! means a glance tells you whether a number came from you or from the air.

use egui::text::{LayoutJob, TextFormat};
use egui::{Color32, CornerRadius, FontFamily, FontId, RichText, Stroke, TextStyle};

/// Deepest surface, the case itself.
pub const CHASSIS: Color32 = Color32::from_rgb(0x17, 0x19, 0x1D);
/// Raised control panels.
pub const PANEL: Color32 = Color32::from_rgb(0x21, 0x24, 0x2A);
/// Slightly proud of the panel, for active wells.
pub const WELL: Color32 = Color32::from_rgb(0x14, 0x16, 0x19);
/// Engraved rules and borders.
pub const ETCH: Color32 = Color32::from_rgb(0x33, 0x38, 0x41);
/// Silkscreened label text.
pub const LEGEND: Color32 = Color32::from_rgb(0x8B, 0x92, 0x9C);
/// Brighter legend, for values.
pub const VALUE: Color32 = Color32::from_rgb(0xD5, 0xDB, 0xE3);
/// Amber: what you set.
pub const READOUT: Color32 = Color32::from_rgb(0xF5, 0xA6, 0x3B);
/// Dim amber, for inactive digits.
pub const READOUT_DIM: Color32 = Color32::from_rgb(0x67, 0x46, 0x1A);
/// Cyan: what the radio hears.
pub const TRACE: Color32 = Color32::from_rgb(0x5C, 0xD0, 0xE8);
/// Fault state.
pub const FAULT: Color32 = Color32::from_rgb(0xE2, 0x6D, 0x5A);

/// A lamp that means the thing is doing its job. Desaturated to sit beside the
/// amber readout without competing with it: this is the state you stop looking
/// at, and the panel's attention belongs on the frequency.
pub const OK: Color32 = Color32::from_rgb(0x5C, 0xB0, 0x7A);

/// Font roles. `Legend` is condensed and set uppercase with wide tracking, the
/// way a panel legend is silkscreened.
pub const READOUT_FONT: &str = "readout";
pub const LEGEND_FONT: &str = "legend";

fn load(paths: &[&str]) -> Option<Vec<u8>> {
    paths.iter().find_map(|p| std::fs::read(p).ok())
}

/// Bind a named family, preferring the first system font that exists but always
/// falling back to egui's embedded stack. The family must exist unconditionally:
/// egui panics when a `FontFamily::Name` is not bound, and none of these paths
/// exist off Linux.
fn register(
    fonts: &mut egui::FontDefinitions,
    name: &'static str,
    fallback: FontFamily,
    paths: &[&str],
) {
    let mut stack = fonts.families.get(&fallback).cloned().unwrap_or_default();
    if let Some(d) = load(paths) {
        fonts.font_data.insert(name.into(), egui::FontData::from_owned(d).into());
        stack.insert(0, name.into());
    }
    fonts.families.insert(FontFamily::Name(name.into()), stack);
}

pub fn install(ctx: &egui::Context) {
    let mut fonts = egui::FontDefinitions::default();

    // Tabular figures for the dial: the readout must not reflow as digits change.
    register(
        &mut fonts,
        READOUT_FONT,
        FontFamily::Monospace,
        &[
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf",
            "/usr/share/fonts/truetype/noto/NotoSansMono-Bold.ttf",
            "/System/Library/Fonts/Menlo.ttc",
            "C:\\Windows\\Fonts\\consolab.ttf",
            "C:\\Windows\\Fonts\\consola.ttf",
        ],
    );

    register(
        &mut fonts,
        LEGEND_FONT,
        FontFamily::Proportional,
        &[
            "/usr/share/fonts/truetype/liberation/LiberationSansNarrow-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf",
            "/System/Library/Fonts/Supplemental/Arial Narrow Bold.ttf",
            "C:\\Windows\\Fonts\\ARIALNB.TTF",
            "C:\\Windows\\Fonts\\segoeuib.ttf",
        ],
    );

    if let Some(d) = load(&["/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf"]) {
        fonts.font_data.insert("body".into(), egui::FontData::from_owned(d).into());
        if let Some(f) = fonts.families.get_mut(&FontFamily::Proportional) {
            f.insert(0, "body".into());
        }
    }
    ctx.set_fonts(fonts);

    ctx.set_theme(egui::ThemePreference::Dark);
    let mut style = (*ctx.style_of(egui::Theme::Dark)).clone();
    let v = &mut style.visuals;
    v.dark_mode = true;
    v.override_text_color = Some(VALUE);
    v.panel_fill = PANEL;
    v.window_fill = PANEL;
    v.extreme_bg_color = WELL;
    v.faint_bg_color = Color32::from_rgb(0x1B, 0x1E, 0x23);
    v.window_stroke = Stroke::new(1.0, ETCH);

    // Instrument panels are machined, not rounded plastic. A 2 px radius reads
    // as a milled edge; anything more reads as a web app.
    let r = CornerRadius::same(2);
    for w in [
        &mut v.widgets.noninteractive,
        &mut v.widgets.inactive,
        &mut v.widgets.hovered,
        &mut v.widgets.active,
        &mut v.widgets.open,
    ] {
        w.corner_radius = r;
    }
    v.widgets.noninteractive.bg_fill = PANEL;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, ETCH);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, LEGEND);

    v.widgets.inactive.bg_fill = Color32::from_rgb(0x2A, 0x2E, 0x35);
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, ETCH);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, VALUE);

    v.widgets.hovered.bg_fill = Color32::from_rgb(0x35, 0x3A, 0x43);
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, LEGEND);
    v.widgets.hovered.fg_stroke = Stroke::new(1.0, Color32::WHITE);

    v.widgets.active.bg_fill = Color32::from_rgb(0x3E, 0x34, 0x22);
    v.widgets.active.bg_stroke = Stroke::new(1.0, READOUT);
    v.widgets.active.fg_stroke = Stroke::new(1.0, READOUT);

    v.selection.bg_fill = Color32::from_rgb(0x2E, 0x3C, 0x46);
    v.selection.stroke = Stroke::new(1.0, TRACE);

    style.spacing.item_spacing = egui::vec2(8.0, 6.0);
    style.spacing.button_padding = egui::vec2(8.0, 4.0);
    style.spacing.slider_width = 120.0;

    style.text_styles.insert(TextStyle::Body, FontId::proportional(13.0));
    style.text_styles.insert(TextStyle::Button, FontId::proportional(13.0));
    // Explanatory prose under a control is set in Small, and a dialog is made
    // of it. Eleven px is a size for a tick mark on an axis, not for two
    // sentences somebody has to read before choosing.
    style.text_styles.insert(TextStyle::Small, FontId::proportional(12.0));
    style
        .text_styles
        .insert(TextStyle::Monospace, FontId::new(13.0, FontFamily::Monospace));
    ctx.set_style_of(egui::Theme::Dark, style.clone());
    ctx.set_style_of(egui::Theme::Light, style);
}

/// A silkscreened panel label: condensed, uppercase, widely tracked.
///
/// Eleven and a half rather than ten: the face is condensed and set in caps
/// with letter spacing, all of which costs legibility, and these are the words
/// that say what every control on the panel is.
pub fn legend(text: &str) -> RichText {
    RichText::new(text.to_uppercase())
        .font(FontId::new(11.5, FontFamily::Name(LEGEND_FONT.into())))
        .extra_letter_spacing(1.6)
        .color(LEGEND)
}

/// A value shown next to a legend.
pub fn value(text: impl Into<String>) -> RichText {
    RichText::new(text)
        .font(FontId::new(13.0, FontFamily::Name(READOUT_FONT.into())))
        .color(VALUE)
}

/// Sizes the panel is set in. Two, so a line is either a caption or a
/// reading and there is no third size to invent.
pub const LEGEND_SIZE: f32 = 11.5;
pub const VALUE_SIZE: f32 = 13.0;

/// Space between two spans of one line, matching the style's item spacing so
/// a line reads the same whether it was built here or laid out as widgets.
const SPAN_GAP: f32 = 8.0;

/// A line of text in more than one voice, set as a single galley.
///
/// A legend and its value are different faces at different sizes, and putting
/// them in a `ui.horizontal` gives each its own galley, which egui then
/// centres against the others: the two sit on different baselines and the row
/// looks a millimetre out everywhere it appears. One `LayoutJob` lays every
/// span out against the row's own metrics, which puts them on one baseline to
/// within a pixel, and wraps them together.
#[derive(Default)]
pub struct Line {
    job: LayoutJob,
    /// Space before the next span, when it is not the usual one.
    gap: Option<f32>,
}

impl Line {
    pub fn new() -> Self {
        Self::default()
    }

    fn face(name: &'static str, size: f32, colour: Color32) -> TextFormat {
        TextFormat {
            font_id: FontId::new(size, FontFamily::Name(name.into())),
            color: colour,
            ..Default::default()
        }
    }

    fn add(mut self, text: impl Into<String>, format: TextFormat) -> Self {
        let lead = match self.job.sections.is_empty() {
            true => 0.0,
            false => self.gap.take().unwrap_or(SPAN_GAP),
        };
        self.job.append(&text.into(), lead, format);
        self
    }

    /// A silkscreened caption: condensed, uppercase, widely tracked.
    pub fn legend(self, text: &str) -> Self {
        let mut f = Self::face(LEGEND_FONT, LEGEND_SIZE, LEGEND);
        f.extra_letter_spacing = 1.6;
        self.add(text.to_uppercase(), f)
    }

    /// A reading, in tabular figures.
    pub fn value(self, text: impl Into<String>) -> Self {
        self.add(text, Self::face(READOUT_FONT, VALUE_SIZE, VALUE))
    }

    /// A reading of something the operator set.
    pub fn set(self, text: impl Into<String>) -> Self {
        self.add(text, Self::face(READOUT_FONT, VALUE_SIZE, READOUT))
    }

    /// A reading of something the radio heard.
    pub fn heard(self, text: impl Into<String>) -> Self {
        self.add(text, Self::face(READOUT_FONT, VALUE_SIZE, TRACE))
    }

    /// Prose: a sentence somebody reads rather than a field they scan.
    pub fn note(self, text: impl Into<String>) -> Self {
        self.add(
            text,
            TextFormat {
                font_id: FontId::proportional(12.0),
                color: LEGEND,
                ..Default::default()
            },
        )
    }

    /// Words off the air, which are neither a caption nor a number.
    pub fn words(self, text: impl Into<String>) -> Self {
        self.add(text, Self::face(READOUT_FONT, VALUE_SIZE, VALUE))
    }

    /// Space before the next span, for a group that belongs together or one
    /// that wants separating from what came before it.
    pub fn gap(mut self, px: f32) -> Self {
        self.gap = Some(px);
        self
    }

    /// Start the next span `x` points from the left of the line.
    ///
    /// A column of readings lines its values up while each row stays one
    /// galley, which a fixed-width label beside a separate value cannot do:
    /// that is two galleys again, and egui centres them against each other.
    pub fn column(mut self, ui: &egui::Ui, x: f32) -> Self {
        let so_far = ui.ctx().fonts_mut(|f| f.layout_job(self.job.clone()).size().x);
        self.gap = Some((x - so_far).max(SPAN_GAP));
        self
    }

    /// Recolour the span just added.
    pub fn tint(mut self, colour: Color32) -> Self {
        if let Some(s) = self.job.sections.last_mut() {
            s.format.color = colour;
        }
        self
    }

    /// Resize the span just added.
    pub fn size(mut self, px: f32) -> Self {
        if let Some(s) = self.job.sections.last_mut() {
            s.format.font_id.size = px;
        }
        self
    }

    pub fn show(self, ui: &mut egui::Ui) -> egui::Response {
        ui.add(egui::Label::new(self.job))
    }

    /// The same line, allowed to wrap into the width available. For prose and
    /// for anything off the air, whose length nobody here chose.
    pub fn wrapped(mut self, ui: &mut egui::Ui) -> egui::Response {
        self.job.wrap.max_width = ui.available_width();
        ui.add(egui::Label::new(self.job))
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn luma(c: Color32) -> f32 {
        0.2126 * c.r() as f32 + 0.7152 * c.g() as f32 + 0.0722 * c.b() as f32
    }

    /// Contrast ratio per WCAG, on the sRGB values egui actually paints.
    fn contrast(a: Color32, b: Color32) -> f32 {
        let l = |c: Color32| {
            let f = |v: u8| {
                let s = v as f32 / 255.0;
                if s <= 0.03928 {
                    s / 12.92
                } else {
                    ((s + 0.055) / 1.055).powf(2.4)
                }
            };
            0.2126 * f(c.r()) + 0.7152 * f(c.g()) + 0.0722 * f(c.b())
        };
        let (x, y) = (l(a), l(b));
        (x.max(y) + 0.05) / (x.min(y) + 0.05)
    }

    #[test]
    fn body_text_is_readable_on_the_panel() {
        assert!(contrast(VALUE, PANEL) > 7.0, "{:.1}", contrast(VALUE, PANEL));
    }

    #[test]
    fn legends_stay_legible_without_shouting() {
        let c = contrast(LEGEND, PANEL);
        // Above the 4.5 floor for small text, but deliberately quieter than
        // the values they label.
        assert!(c > 4.5, "legend contrast only {c:.1}");
        assert!(c < contrast(VALUE, PANEL), "legend competes with its own value");
    }

    #[test]
    fn both_accents_carry_on_the_chassis() {
        assert!(contrast(READOUT, CHASSIS) > 6.0, "amber {:.1}", contrast(READOUT, CHASSIS));
        assert!(contrast(TRACE, CHASSIS) > 6.0, "cyan {:.1}", contrast(TRACE, CHASSIS));
    }

    #[test]
    fn the_accents_are_distinguishable_from_each_other() {
        // They encode different meanings, so they must not be confusable even
        // for a red-green deficient viewer. Amber and cyan differ strongly in
        // the blue channel, which is the safe axis.
        let db = (READOUT.b() as i32 - TRACE.b() as i32).abs();
        assert!(db > 100, "accents differ by only {db} in blue");
    }

    #[test]
    fn surfaces_step_in_a_consistent_direction() {
        // Well is recessed, panel is raised: the stack must be ordered or
        // depth cues contradict each other.
        assert!(luma(WELL) < luma(CHASSIS));
        assert!(luma(CHASSIS) < luma(PANEL));
        assert!(luma(PANEL) < luma(ETCH));
    }

    #[test]
    fn dim_readout_reads_as_the_same_hue() {
        // The inactive digits must look like the same lamp turned down, not a
        // different colour.
        let hue = |c: Color32| (c.r() as f32 - c.b() as f32) / (c.r() as f32 + c.b() as f32);
        assert!((hue(READOUT) - hue(READOUT_DIM)).abs() < 0.12);
        assert!(luma(READOUT_DIM) < luma(READOUT) * 0.6);
    }

    /// The reason `Line` exists. Two labels in a `ui.horizontal` are two
    /// galleys, and egui centres them against each other, so a legend beside
    /// a value sits a pixel low and every row on the panel is out by a
    /// different amount. One layout job puts them on one baseline.
    #[test]
    fn spans_of_different_sizes_share_a_baseline() {
        let ctx = egui::Context::default();
        install(&ctx);
        // No fonts exist until a frame has been run.
        let _ = ctx.run_ui(Default::default(), |_| {});
        let job = Line::new().legend("squelch").value("-42 dBFS").size(11.0).job;
        let galley = ctx.fonts_mut(|f| f.layout_job(job));
        assert_eq!(galley.rows.len(), 1, "a short line wrapped");
        let baselines: Vec<f32> = galley.rows[0].glyphs.iter().map(|g| g.pos.y).collect();
        let (lo, hi) = (
            baselines.iter().cloned().fold(f32::MAX, f32::min),
            baselines.iter().cloned().fold(f32::MIN, f32::max),
        );
        assert!(hi - lo <= 1.0, "baselines differ by {:.2} px", hi - lo);
    }

    #[test]
    fn fault_is_distinct_from_the_amber_readout() {
        let d = (FAULT.g() as i32 - READOUT.g() as i32).abs();
        assert!(d > 40, "fault and readout differ by only {d} in green");
    }
}

