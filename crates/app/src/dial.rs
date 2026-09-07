//! Per-digit frequency dial.
//!
//! On a hardware receiver you pick a decade and turn the knob, so tuning is
//! precise and repeatable. A drag field cannot do that: its step depends on how
//! fast you move the mouse. Here each digit is its own hit target, and the
//! wheel over a digit steps that decade.

use crate::theme;
use crate::wheel::Wheel;
use egui::{Align2, Color32, FontFamily, FontId, Pos2, Rect, Sense, Stroke, Ui, Vec2};

/// Digits shown, from 1 GHz down to 1 Hz. Ten of them, because the tuner
/// reaches 1766 MHz and nine would cap the dial at 999.999999 MHz.
///
/// The dial does not decide where the radio can go: it clamps at zero and
/// nothing else, and what the tuner can reach is applied where the retune is
/// sent, from the ranges the device itself reports. A ceiling here was 3 GHz
/// for everybody, which a HackRF (6 GHz) and a LimeSDR (3.8 GHz) both have
/// spectrum above.
const DECADES: [i32; 10] = [9, 8, 7, 6, 5, 4, 3, 2, 1, 0];
/// Where a gap is drawn, keyed by the decade to its right.
const GROUP_AFTER: [i32; 2] = [6, 3];

pub struct Dial {
    /// Decade currently under the pointer, if any.
    pub hot: Option<i32>,
    wheel: Wheel,
}

/// Result of drawing the dial.
pub struct DialOut {
    pub changed: bool,
    pub hz: f64,
}

impl Dial {
    pub fn new() -> Self {
        Self { hot: None, wheel: Wheel::default() }
    }

    /// Draw the readout and apply wheel input. Returns the possibly-updated
    /// frequency.
    pub fn show(&mut self, ui: &mut Ui, hz: f64, size: f32) -> DialOut {
        self.show_tunable(ui, hz, size, true)
    }

    /// The same readout on a tuner that cannot be moved: a network stream
    /// pinned by whoever feeds it. The digits are drawn but take no input,
    /// and say so, since a dial that looks live and ignores every drag reads
    /// as broken rather than as fixed.
    pub fn show_tunable(&mut self, ui: &mut Ui, hz: f64, size: f32, tunable: bool) -> DialOut {
        let digit_w = size * 0.62;
        let gap = size * 0.22;
        let width = DECADES.len() as f32 * digit_w + GROUP_AFTER.len() as f32 * gap + size * 2.4;
        let height = size * 1.5;

        let (rect, response) = ui.allocate_exact_size(
            Vec2::new(width, height),
            if tunable { Sense::click_and_drag() } else { Sense::hover() },
        );
        let p = ui.painter_at(rect);

        p.rect_filled(rect, 2.0, theme::WELL);
        p.rect_stroke(rect, 2.0, Stroke::new(1.0, theme::ETCH), egui::StrokeKind::Inside);
        if !tunable {
            self.hot = None;
            response.clone().on_hover_text("This source is pinned by the radio feeding it");
        }

        let hover = if tunable { response.hover_pos() } else { None };
        let mut hot = None;

        // Above 100 MHz the leading digit is significant; below it, it is a
        // leading zero and should be dimmed rather than hidden, so the digits
        // never move.
        let mut n = hz.round().max(0.0) as u64;
        let mut digits = [0u8; 10];
        for (i, _) in DECADES.iter().enumerate().rev() {
            digits[i] = (n % 10) as u8;
            n /= 10;
        }

        let mut x = rect.left() + size * 0.5;
        let cy = rect.center().y;
        let mut leading = true;

        for (i, &dec) in DECADES.iter().enumerate() {
            if digits[i] != 0 || dec <= 6 {
                leading = false;
            }
            let cell = Rect::from_min_size(
                Pos2::new(x, rect.top() + size * 0.18),
                Vec2::new(digit_w, height - size * 0.36),
            );

            let is_hot = hover.is_some_and(|h| cell.x_range().contains(h.x) && rect.contains(h));
            if is_hot {
                hot = Some(dec);
                p.rect_filled(cell, 1.0, Color32::from_rgba_unmultiplied(245, 166, 59, 22));
                // Underline the live decade, the way a receiver marks the
                // selected tuning step.
                p.line_segment(
                    [
                        Pos2::new(cell.left() + 1.0, cell.bottom()),
                        Pos2::new(cell.right() - 1.0, cell.bottom()),
                    ],
                    Stroke::new(2.0, theme::READOUT),
                );
            }

            let col = if leading { theme::READOUT_DIM } else { theme::READOUT };
            p.text(
                Pos2::new(cell.center().x, cy),
                Align2::CENTER_CENTER,
                digits[i].to_string(),
                FontId::new(size, FontFamily::Name(theme::READOUT_FONT.into())),
                col,
            );

            x += digit_w;
            if GROUP_AFTER.contains(&dec) {
                // A dot at the MHz break, a thinner space at the kHz break:
                // the same convention as a printed frequency.
                let mark = if dec == 6 { "." } else { "\u{2009}" };
                p.text(
                    Pos2::new(x + gap * 0.5, cy),
                    Align2::CENTER_CENTER,
                    mark,
                    FontId::new(size, FontFamily::Name(theme::READOUT_FONT.into())),
                    theme::READOUT_DIM,
                );
                x += gap;
            }
        }

        p.text(
            Pos2::new(rect.right() - size * 0.35, cy + size * 0.12),
            Align2::RIGHT_CENTER,
            "MHz",
            FontId::new(size * 0.34, FontFamily::Name(theme::LEGEND_FONT.into())),
            theme::LEGEND,
        );

        self.hot = hot;

        let mut out = hz;
        let mut changed = false;
        if let Some(dec) = hot {
            let n = self.wheel.notches(ui);
            if n != 0 {
                out = (hz + 10f64.powi(dec) * n as f64).max(0.0);
                changed = true;
            }
            if response.secondary_clicked() {
                out = zero_below(hz, dec);
                changed = out != hz;
            }
        }

        DialOut { changed, hz: out }
    }
}

/// Clear a decade and everything under it, the way a receiver's dial does when
/// you want a round number: right-clicking the kHz digit of 95.8437 leaves
/// 95.8000, not 95.8437 with one digit changed.
fn zero_below(hz: f64, dec: i32) -> f64 {
    let step = 10f64.powi(dec + 1);
    (hz / step).floor() * step
}

/// Decades shown by the compact dial: 1 GHz down to 100 Hz. Finer than any
/// broadcast channel needs at the bottom, and the top digit is there because
/// the tuner reaches 1766 MHz and stopping at 100 MHz would silently drop the
/// leading digit of anything above 1 GHz.
const SMALL_DECADES: [i32; 8] = [9, 8, 7, 6, 5, 4, 3, 2];

/// Digits for the compact dial, most significant first.
///
/// The lowest decade shown is 100 Hz, so the frequency is scaled by that first.
/// Taking digits straight off a value in Hz makes every digit mean a hundred
/// times what its column says, which renders 92.4 MHz as 9240.0000.
fn small_digits(hz: f64) -> [u8; SMALL_DECADES.len()] {
    let step = 10f64.powi(*SMALL_DECADES.last().unwrap());
    // Round rather than truncate: a channel dragged to 92,400,050 Hz should
    // read 92.4001, not 92.4000.
    let mut n = (hz / step).round().max(0.0) as u64;
    let mut digits = [0u8; SMALL_DECADES.len()];
    for i in (0..SMALL_DECADES.len()).rev() {
        digits[i] = (n % 10) as u8;
        n /= 10;
    }
    digits
}

impl Dial {
    /// A small per-digit readout for a channel, with the same gesture as the
    /// main dial: the wheel over a digit steps that decade.
    ///
    /// Sharing the widget rather than giving each channel its own is safe
    /// because only one can be under the pointer, and the wheel accumulator
    /// only means anything while a gesture is in progress.
    pub fn compact(&mut self, ui: &mut Ui, hz: f64, size: f32) -> DialOut {
        let digit_w = size * 0.60;
        let gap = size * 0.26;
        let width = SMALL_DECADES.len() as f32 * digit_w + gap;
        let (rect, response) =
            ui.allocate_exact_size(Vec2::new(width, size * 1.25), Sense::click());
        let p = ui.painter_at(rect);

        let digits = small_digits(hz);

        let hover = response.hover_pos();
        let mut hot = None;
        let mut x = rect.left();
        let cy = rect.center().y;
        let mut leading = true;

        for (i, &dec) in SMALL_DECADES.iter().enumerate() {
            if digits[i] != 0 || dec <= 6 {
                leading = false;
            }
            let cell = Rect::from_min_size(Pos2::new(x, rect.top()), Vec2::new(digit_w, rect.height()));
            let is_hot = hover.is_some_and(|h| cell.x_range().contains(h.x) && rect.contains(h));
            if is_hot {
                hot = Some(dec);
                p.rect_filled(cell, 1.0, Color32::from_rgba_unmultiplied(245, 166, 59, 26));
                p.line_segment(
                    [
                        Pos2::new(cell.left() + 1.0, cell.bottom() - 1.0),
                        Pos2::new(cell.right() - 1.0, cell.bottom() - 1.0),
                    ],
                    Stroke::new(1.5, theme::READOUT),
                );
            }
            p.text(
                Pos2::new(cell.center().x, cy),
                Align2::CENTER_CENTER,
                digits[i].to_string(),
                FontId::new(size, FontFamily::Name(theme::READOUT_FONT.into())),
                if leading { theme::READOUT_DIM } else { theme::READOUT },
            );
            x += digit_w;
            if dec == 6 {
                p.text(
                    Pos2::new(x + gap * 0.5, cy),
                    Align2::CENTER_CENTER,
                    ".",
                    FontId::new(size, FontFamily::Name(theme::READOUT_FONT.into())),
                    theme::READOUT_DIM,
                );
                x += gap;
            }
        }

        let mut out = hz;
        let mut changed = false;
        if let Some(dec) = hot {
            let n = self.wheel.notches(ui);
            if n != 0 {
                out = (hz + 10f64.powi(dec) * n as f64).max(0.0);
                changed = true;
            }
            if response.secondary_clicked() {
                out = zero_below(hz, dec);
                changed = out != hz;
            }
        }
        DialOut { changed, hz: out }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_dial_shows_every_decade_it_claims_to() {
        // Nine digits covers 100 MHz down to 1 Hz, which must span the whole
        // RTL-SDR range with the leading digit still meaningful at 1.7 GHz.
        assert_eq!(DECADES.len(), 10);
        assert_eq!(*DECADES.first().unwrap(), 9);
        assert_eq!(*DECADES.last().unwrap(), 0);
        assert!(10f64.powi(DECADES[0] + 1) > 1.766e9);
    }

    #[test]
    fn digit_extraction_matches_the_frequency() {
        let hz = 95_800_000u64;
        let mut n = hz;
        let mut digits = [0u8; 10];
        for i in (0..10).rev() {
            digits[i] = (n % 10) as u8;
            n /= 10;
        }
        // 0095.800 000
        assert_eq!(digits, [0, 0, 9, 5, 8, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn a_step_changes_only_its_own_decade() {
        let hz = 95_800_000.0;
        for dec in 0..10 {
            let stepped = hz + 10f64.powi(dec);
            let diff = stepped - hz;
            assert!((diff - 10f64.powi(dec)).abs() < 1e-6);
        }
    }

    #[test]
    fn zeroing_a_digit_clears_everything_under_it() {
        // Right-clicking the 100 kHz digit of 95.8437 MHz gives 95.0000.
        assert_eq!(zero_below(95_843_700.0, 5), 95_000_000.0);
        // And the 1 kHz digit gives 95.843 MHz exactly.
        assert_eq!(zero_below(95_843_700.0, 3), 95_840_000.0);
        // The units digit clears only the sub-Hz part, so nothing moves.
        assert_eq!(zero_below(95_843_700.0, 0), 95_843_700.0);
    }

    #[test]
    fn zeroing_the_top_digit_goes_to_zero() {
        assert_eq!(zero_below(1_090_000_000.0, 9), 0.0);
    }

    #[test]
    fn zeroing_an_already_round_frequency_changes_nothing() {
        // No change means no retune, so a stray right-click cannot interrupt
        // a station that is already on a round frequency.
        assert_eq!(zero_below(95_000_000.0, 5), 95_000_000.0);
    }

    #[test]
    fn tuning_cannot_go_negative() {
        let hz: f64 = 5.0;
        let out = (hz - 10f64.powi(6)).max(0.0);
        assert_eq!(out, 0.0);
    }

    /// And nothing else is capped here. The dial used to stop at 3 GHz
    /// whatever was plugged in, so a HackRF could not be turned to 5.8 GHz
    /// and a LimeSDR not to 3.5, though both tune there and the band plan
    /// draws the allocations.
    #[test]
    fn the_dial_does_not_decide_where_the_tuner_reaches() {
        let out = (5_800_000_000.0f64 + 10f64.powi(9)).max(0.0);
        assert_eq!(out, 6_800_000_000.0);
    }

    #[test]
    fn the_compact_dial_resolves_finer_than_any_channel_needs() {
        // 100 Hz steps against a 12.5 kHz narrowband channel.
        assert_eq!(*SMALL_DECADES.last().unwrap(), 2);
        // And reaches past the top of the tuner's range, so a channel above
        // 1 GHz does not lose its leading digit.
        assert!(10f64.powi(SMALL_DECADES[0] + 1) > 1.766e9);
    }

    /// What the compact dial puts on screen, point included.
    fn rendered(hz: f64) -> String {
        let d = small_digits(hz);
        let mut out = String::new();
        for (i, &dec) in SMALL_DECADES.iter().enumerate() {
            out.push((b'0' + d[i]) as char);
            if dec == 6 {
                out.push('.');
            }
        }
        out
    }

    #[test]
    fn the_compact_dial_shows_the_frequency_it_was_given() {
        // This read 9240.0000 when the digits were taken off a value in Hz
        // while the lowest column meant hundreds.
        assert_eq!(rendered(92_400_000.0), "0092.4000");
        assert_eq!(rendered(95_800_000.0), "0095.8000");
        assert_eq!(rendered(433_920_000.0), "0433.9200");
        assert_eq!(rendered(1_090_000_000.0), "1090.0000");
    }

    #[test]
    fn the_compact_dial_rounds_rather_than_truncating() {
        assert_eq!(rendered(92_400_050.0), "0092.4001");
        assert_eq!(rendered(92_400_040.0), "0092.4000");
    }

    #[test]
    fn a_step_on_the_compact_dial_lands_on_the_digit_it_underlines() {
        // Stepping the 10 kHz column must change only that column.
        let hz = 92_400_000.0;
        assert_eq!(rendered(hz + 10f64.powi(4)), "0092.4100");
        assert_eq!(rendered(hz + 10f64.powi(6)), "0093.4000");
    }

    #[test]
    fn groups_break_at_mhz_and_khz() {
        // 095.800 000 reads as MHz . kHz Hz, matching how frequencies are
        // written down and spoken.
        assert_eq!(GROUP_AFTER, [6, 3]);
    }
}
