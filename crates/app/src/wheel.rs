//! Discrete mouse-wheel notches.
//!
//! `InputState::smooth_scroll_delta` is interpolated across many frames, so
//! reading it to drive a stepped control fires that step once per frame for a
//! single notch: one detent moved the dial ten decades. The raw `MouseWheel`
//! events arrive once each, which is what a stepped control wants.

use egui::{Event, Ui};

#[cfg(not(target_arch = "wasm32"))]
#[path = "wheel/native.rs"]
mod platform;
#[cfg(target_arch = "wasm32")]
#[path = "wheel/web.rs"]
mod platform;

/// Accumulator so partial notches from a trackpad still eventually step,
/// instead of being rounded away every frame.
#[derive(Default, Clone, Copy)]
pub struct Wheel {
    acc: f32,
}

impl Wheel {
    /// Whole notches scrolled since the last call. Positive is scroll up.
    pub fn notches(&mut self, ui: &Ui) -> i32 {
        let raw: f32 = ui.input(|i| {
            i.events
                .iter()
                .map(|e| match e {
                    Event::MouseWheel { unit, delta, .. } => platform::notches(*unit, delta.y),
                    _ => 0.0,
                })
                .sum()
        });
        self.add(raw)
    }

    /// Split out so the accumulator can be tested without an egui context.
    pub fn add(&mut self, raw: f32) -> i32 {
        self.acc += raw;
        let whole = self.acc.trunc();
        self.acc -= whole;
        whole as i32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_notch_is_one_step() {
        let mut w = Wheel::default();
        assert_eq!(w.add(1.0), 1);
        assert_eq!(w.add(-1.0), -1);
    }

    #[test]
    fn nothing_scrolled_is_nothing_stepped() {
        let mut w = Wheel::default();
        for _ in 0..100 {
            assert_eq!(w.add(0.0), 0);
        }
    }

    #[test]
    fn trackpad_fractions_accumulate_instead_of_vanishing() {
        let mut w = Wheel::default();
        let mut total = 0;
        for _ in 0..10 {
            total += w.add(0.25);
        }
        assert_eq!(total, 2, "2.5 notches of scroll should step twice");
    }

    #[test]
    fn fast_scrolling_is_not_thrown_away() {
        let mut w = Wheel::default();
        assert_eq!(w.add(5.0), 5);
    }

    #[test]
    fn direction_changes_do_not_leave_a_stuck_remainder() {
        let mut w = Wheel::default();
        w.add(0.6);
        // Reversing should cancel the pending fraction rather than adding to it.
        assert_eq!(w.add(-0.6), 0);
        assert_eq!(w.add(-1.0), -1);
    }
}
