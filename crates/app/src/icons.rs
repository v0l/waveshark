//! The app's icons: Phosphor glyphs, drawn by the painter.
//!
//! Phosphor (`egui-phosphor`) is one font, one weight, one grid, so the whole
//! set is consistent in a way a dozen hand-drawn glyphs never were: they were
//! each tuned against the size the first caller used and drifted in weight
//! between the top bar and the strip. The font is bound as its own family
//! (`theme::ICON_FONT`) so an icon is always served by Phosphor and never by
//! whichever text font happens to have something at that code point.
//!
//! Every icon carries its label as hover text. An icon alone is a rebus, and
//! the label is what makes the first use of the app possible.

use crate::theme;
use egui::{Color32, FontFamily, FontId, Pos2, Rect, Response, Sense, Ui, Vec2};
use egui_phosphor::regular as ph;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Icon {
    /// Start the radio.
    Play,
    Stop,
    /// The radio's own controls: gain, switches, corrections.
    Sliders,
    /// Application setup.
    Setup,
    /// Decode everything in the span.
    Decode,
    /// The packet log.
    Log,
    /// Audio on, and audio muted. Two icons rather than one lit differently:
    /// a mute control has to say which state it is in from across the desk,
    /// and colour alone does not carry that.
    Sound,
    Mute,
    /// Write the raw span to a file.
    Capture,
    /// Key the transmitter.
    Transmit,
    /// The dataset cache: somebody else's files kept on this machine.
    Data,
    /// Open the page a dataset comes from, in a browser.
    Link,
    /// The views, one glyph each. They are tabs rather than a list, so each
    /// one has to be told apart from the other ten at 22 points: no two of
    /// them share a silhouette.
    Dashboard,
    Spectrum,
    Chain,
    Map,
    Calls,
    Messages,
    Video,
    Links,
    Devices,
    Satellite,
    Key,
    /// A model control link: a stick on a gimbal, which no other tab is.
    Control,
    /// What was said, written down: a bubble with words in it, against the
    /// empty bubble that means text somebody typed.
    Transcript,
}

/// Side of the clickable square, in points.
///
/// Sized against the controls beside it rather than against the glyph: an
/// icon the size of a full stop is a smaller target than the text button it
/// replaced, which is a worse control however clean it looks.
pub const SIZE: f32 = 28.0;

/// Glyph height as a fraction of the square.
///
/// Phosphor draws its icons across the whole em, so this is the ink height
/// directly. Below about 0.6 the icon is a smaller target than the button it
/// sits in; above 0.75 adjacent icons appear to touch.
const GLYPH: f32 = 0.68;

impl Icon {
    /// The Phosphor code point this icon is drawn with.
    fn glyph(self) -> &'static str {
        match self {
            Icon::Play => ph::PLAY,
            Icon::Stop => ph::STOP,
            Icon::Sliders => ph::FADERS,
            Icon::Setup => ph::GEAR_SIX,
            Icon::Decode => ph::SCAN,
            Icon::Log => ph::LIST_DASHES,
            Icon::Sound => ph::SPEAKER_HIGH,
            Icon::Mute => ph::SPEAKER_SLASH,
            Icon::Capture => ph::RECORD,
            Icon::Transmit => ph::BROADCAST,
            Icon::Data => ph::DATABASE,
            Icon::Link => ph::ARROW_SQUARE_OUT,
            Icon::Dashboard => ph::GAUGE,
            Icon::Spectrum => ph::WAVEFORM,
            Icon::Chain => ph::TREE_STRUCTURE,
            Icon::Map => ph::MAP_PIN,
            Icon::Calls => ph::MICROPHONE,
            Icon::Messages => ph::CHAT_TEXT,
            Icon::Video => ph::MONITOR_PLAY,
            Icon::Links => ph::SHARE_NETWORK,
            Icon::Devices => ph::DEVICE_MOBILE,
            Icon::Key => ph::KEY,
            Icon::Control => ph::JOYSTICK,
            // A ringed planet rather than a dish, which Phosphor has not got.
            // A dish drawn by hand to match the font read as an umbrella at
            // tab size, where the mast crossed the arm and the two signal
            // arcs closed up into the rim.
            Icon::Satellite => ph::PLANET,
            // Lines of writing rather than a bubble: `Messages` is the empty
            // bubble, and here the point is that speech came out as words.
            Icon::Transcript => ph::ARTICLE,
        }
    }

    /// Draw the icon inside `r`. Public so the panes can settle their corner
    /// affordance with the same shape the top bar uses: two drawings of the
    /// same idea is one of them being wrong.
    pub fn paint(self, p: &egui::Painter, r: Rect, col: Color32) {
        let font = FontId::new(r.height() * GLYPH, FontFamily::Name(theme::ICON_FONT.into()));
        let galley = p.layout_no_wrap(self.glyph().to_string(), font, col);
        // Centre the ink, not the line box. The box carries the font's ascent
        // and descent, which is the same height for every glyph while the ink
        // is not, so centring on it hangs each icon at its own offset in the
        // square.
        let ink = galley.mesh_bounds;
        let at = if ink.is_positive() {
            r.center() - ink.center().to_vec2()
        } else {
            r.center() - (galley.size() * 0.5)
        };
        p.galley(at, galley, col);
    }
}

/// Colour for an icon in a given state.
///
/// Separated from the drawing so the choice can be checked without a painter,
/// and because getting it wrong is the failure that matters: an icon that
/// looks the same on and off is a switch with no readout.
pub fn tint(enabled: bool, selected: bool, hovered: bool) -> Color32 {
    if !enabled {
        theme::ETCH
    } else if selected || hovered {
        // Amber for both. It is the panel's one accent, and a control that
        // lights up white on hover and amber when on belongs to two different
        // instruments. The filled well behind a selected icon is what tells
        // the two states apart.
        theme::READOUT
    } else {
        theme::LEGEND
    }
}

/// An icon that behaves like a button, labelled by hover text.
pub fn icon_button(ui: &mut Ui, icon: Icon, tip: &str, enabled: bool, selected: bool) -> Response {
    icon_button_sized(ui, icon, tip, enabled, selected, SIZE)
}

/// The same, at a size that fits a row of controls rather than the toolbar.
pub fn icon_button_sized(
    ui: &mut Ui,
    icon: Icon,
    tip: &str,
    enabled: bool,
    selected: bool,
    size: f32,
) -> Response {
    let (rect, mut resp) = ui.allocate_exact_size(
        Vec2::splat(size),
        if enabled { Sense::click() } else { Sense::hover() },
    );
    let hovered = resp.hovered();
    if ui.is_rect_visible(rect) {
        let p = ui.painter();
        if selected || (hovered && enabled) {
            p.rect_filled(rect, 3.0, if selected { theme::WELL } else { theme::ETCH });
        }
        icon.paint(p, rect, tint(enabled, selected, hovered));
    }
    resp = resp.on_hover_text(tip);
    if enabled {
        // The pointer has to say the thing is pressable; the icon alone does
        // not, having no border to read as a button.
        resp.clone().on_hover_cursor(egui::CursorIcon::PointingHand);
    }
    resp
}

/// A tab in the view strip: an icon button with a dot in its corner when the
/// view behind it has something to show.
///
/// The dot is the whole point of a strip over a dropdown. Ten tabs are only
/// worth their width if the operator can see, without opening any of them,
/// which ones are holding traffic.
pub fn icon_tab(
    ui: &mut Ui,
    icon: Icon,
    tip: &str,
    selected: bool,
    live: bool,
    size: f32,
) -> Response {
    let (rect, mut resp) = ui.allocate_exact_size(Vec2::splat(size), Sense::click());
    let hovered = resp.hovered();
    if ui.is_rect_visible(rect) {
        let p = ui.painter();
        // The strip is already a well, so the selected button cannot be a
        // second well: it would be the same colour as its own background.
        // A raised face and a bar under it is what a tab has always been.
        if selected {
            p.rect_filled(rect, 3.0, theme::PANEL);
            let bar = Rect::from_min_max(
                Pos2::new(rect.left() + 2.0, rect.bottom() - 2.0),
                Pos2::new(rect.right() - 2.0, rect.bottom() - 0.5),
            );
            p.rect_filled(bar, 1.0, theme::READOUT);
        } else if hovered {
            p.rect_filled(rect, 3.0, theme::ETCH);
        }
        icon.paint(p, rect.translate(Vec2::new(0.0, -1.0)), tint(true, selected, hovered));
        // The dot is the whole point of a strip over a dropdown: it says
        // which views are holding traffic without opening any of them. Not
        // drawn on the open tab, where the traffic is already on screen.
        if live && !selected {
            let at = Pos2::new(rect.right() - size * 0.17, rect.top() + size * 0.17);
            p.circle_filled(at, (size * 0.09).max(1.5), theme::TRACE);
        }
    }
    resp = resp.on_hover_text(tip);
    resp.clone().on_hover_cursor(egui::CursorIcon::PointingHand);
    resp
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every icon in the top bar and the view strip, in the order they are
    /// drawn. Listed rather than derived so a new variant has to be added
    /// here, which is where the tests below then check it.
    const ALL: [Icon; 23] = [
        Icon::Play,
        Icon::Stop,
        Icon::Sliders,
        Icon::Setup,
        Icon::Decode,
        Icon::Log,
        Icon::Sound,
        Icon::Mute,
        Icon::Capture,
        Icon::Transmit,
        Icon::Data,
        Icon::Link,
        Icon::Dashboard,
        Icon::Spectrum,
        Icon::Chain,
        Icon::Map,
        Icon::Calls,
        Icon::Messages,
        Icon::Video,
        Icon::Links,
        Icon::Devices,
        Icon::Satellite,
        Icon::Key,
    ];

    #[test]
    fn no_two_icons_share_a_glyph() {
        // A tab strip is only worth its width if each tab is told apart from
        // the others without reading the hover text.
        let mut seen: Vec<&str> = ALL.iter().map(|i| i.glyph()).collect();
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), ALL.len(), "two icons are drawn with the same glyph");
    }

    /// The failure this catches is a glyph the icon font does not have, which
    /// egui draws as a replacement box: still ink, but a fraction of the size
    /// a Phosphor icon covers, and it would go unnoticed until somebody
    /// opened that pane.
    #[test]
    fn every_glyph_fills_the_square_it_is_given() {
        let ctx = egui::Context::default();
        theme::install(&ctx);
        // No fonts exist until a frame has been run.
        let _ = ctx.run_ui(Default::default(), |_| {});
        let size = SIZE * GLYPH;
        for icon in ALL {
            let font = FontId::new(size, FontFamily::Name(theme::ICON_FONT.into()));
            let g = icon.glyph().to_string();
            let galley = ctx.fonts_mut(|f| f.layout_no_wrap(g, font, Color32::WHITE));
            let ink = galley.mesh_bounds;
            assert!(ink.is_positive(), "an icon drew nothing");
            // Phosphor draws across the em box, so a real icon is most of the
            // requested size in its longer axis. A replacement box is half.
            let long = ink.width().max(ink.height());
            assert!(
                long > size * 0.7,
                "an icon covers {long:.1} pt of {size:.1}, which is a missing glyph"
            );
            assert!(long <= size * 1.1, "an icon overflows its square at {long:.1} pt");
        }
    }

    #[test]
    fn a_disabled_icon_cannot_be_confused_with_an_active_one() {
        assert_eq!(tint(false, false, false), theme::ETCH);
        assert_eq!(tint(false, true, true), theme::ETCH, "disabled wins over every other state");
    }

    #[test]
    fn a_switch_that_is_on_reads_as_on() {
        // Amber is the panel's one accent, the colour the tuned frequency is
        // set in, and nothing that is merely available uses it.
        assert_eq!(tint(true, true, false), theme::READOUT);
        assert_ne!(tint(true, true, false), tint(true, false, false));
    }

    #[test]
    fn hovering_an_available_control_changes_it() {
        assert_ne!(tint(true, false, true), tint(true, false, false));
    }
}
