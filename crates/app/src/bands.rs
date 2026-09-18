//! The allocation ribbon under the spectrum.
//!
//! The table itself is [`common::bands`], because a band plan is knowledge
//! about the world and the decoders need it as much as the drawing does.
//! What is here is the one thing only a screen has: a colour per service.

pub use common::bands::*;
use egui::Color32;

/// The colour a band is drawn in, one per service.
pub fn color(usage: Usage) -> Color32 {
    match usage {
        Usage::Broadcast => Color32::from_rgb(0x4A, 0x6F, 0x8A),
        Usage::Aero => Color32::from_rgb(0x8A, 0x6B, 0x4A),
        Usage::Amateur => Color32::from_rgb(0x53, 0x7A, 0x5C),
        Usage::Utility => Color32::from_rgb(0x6B, 0x5A, 0x7A),
        // Licence-free is one colour on the ribbon whichever side of a
        // gigahertz it is: the split is about what a decoder should be
        // placed on, not about what an operator is looking at.
        Usage::Ism | Usage::Wlan => Color32::from_rgb(0x8A, 0x4A, 0x55),
        Usage::Cellular => Color32::from_rgb(0x7A, 0x4A, 0x6B),
        Usage::Nav => Color32::from_rgb(0x4A, 0x7A, 0x7A),
    }
}
