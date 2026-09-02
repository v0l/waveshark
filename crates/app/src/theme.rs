//! Bench-instrument theme: chassis greys, engraved legends, amber readouts,
//! cyan traces.
//!
//! The two accents carry meaning rather than decoration. Amber is everything
//! you set; cyan is everything the radio hears. Keeping that split consistent
//! means a glance tells you whether a number came from you or from the air.

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

    #[test]
    fn fault_is_distinct_from_the_amber_readout() {
        let d = (FAULT.g() as i32 - READOUT.g() as i32).abs();
        assert!(d > 40, "fault and readout differ by only {d} in green");
    }
}
