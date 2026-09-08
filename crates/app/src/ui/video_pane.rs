//! The picture pane: whatever the video bus is publishing, drawn.
//!
//! A view over the video bus the way the call list is a view over the audio
//! one: it reads a field off the status and draws it, and knows nothing about
//! which front end produced it or on what band.
//!
//! # What it has to say and not only show
//!
//! Analogue video carries no integrity check of any kind, so a picture is
//! never right or wrong, only more or less complete. `lines_seen` is the only
//! quality measure there is, and drawing a field assembled from a third of
//! its lines as though it were a picture claims more than the receiver knows.
//! So the pane prints the count and the channel over the image, and it clears
//! itself when fields stop arriving rather than leaving the last one on the
//! screen: a still picture of a transmitter that has gone away is the worst
//! thing this pane could do.

use super::*;
use common::{Pixels, VideoFrame};

/// How long a picture stays on screen after the last field.
///
/// Two fields is 40 ms, which is far too twitchy for a link fading in and
/// out; half a second is long enough to ride a dropout and short enough that
/// nobody mistakes it for a live picture.
const HOLD: std::time::Duration = std::time::Duration::from_millis(500);

#[derive(Default)]
pub(super) struct VideoState {
    /// The texture the last field was uploaded into, kept so a redraw that
    /// gets no new field is free.
    texture: Option<egui::TextureHandle>,
    /// The field that texture holds, for the caption.
    shown: Option<VideoFrame>,
    last: Option<std::time::Instant>,
    /// Which transmission is being watched, by the key the bus keeps it
    /// under, or `None` for whatever is best.
    watching: Option<String>,
}

pub(super) struct VideoPane<'a> {
    pub st: &'a mut VideoState,
    /// The newest field, or `None` when nothing is producing pictures.
    pub frame: Option<VideoFrame>,
    /// Every transmission the bus is seeing, with what it is called and how
    /// complete its last picture was.
    pub inputs: Vec<(String, String, f32)>,
    /// Where the pane puts what it wants the receiver to do.
    pub cmds: &'a mut Vec<Cmd>,
}

impl VideoPane<'_> {
    pub fn show(self, ui: &mut egui::Ui) {
        let st = self.st;
        // Which picture, when there is more than one. One channel needs no
        // chooser, and a row of buttons over an empty pane says nothing.
        if self.inputs.len() > 1 {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new("watching").color(theme::LEGEND).size(11.0));
                let mut want = st.watching.clone();
                if ui.selectable_label(want.is_none(), "best").clicked() {
                    want = None;
                }
                for (key, label, complete) in &self.inputs {
                    let text = format!("{label}  {:.0}%", complete * 100.0);
                    if ui.selectable_label(want.as_deref() == Some(key.as_str()), text).clicked() {
                        want = Some(key.clone());
                    }
                }
                if want != st.watching {
                    st.watching = want.clone();
                    self.cmds.push(Cmd::WatchVideo(match want {
                        Some(k) => vec![crate::videobus::Rule::Channel(k)],
                        None => vec![crate::videobus::Rule::Everything],
                    }));
                }
            });
        }
        if let Some(f) = self.frame {
            let new = st.shown.as_ref().is_none_or(|s| s.sequence != f.sequence);
            if new {
                st.texture = Some(upload(ui.ctx(), &f, st.texture.take()));
                st.last = Some(std::time::Instant::now());
                st.shown = Some(f);
            }
        }
        if st.last.is_some_and(|t| t.elapsed() > HOLD) {
            st.texture = None;
            st.shown = None;
            st.last = None;
        }

        let (Some(tex), Some(f)) = (st.texture.as_ref(), st.shown.as_ref()) else {
            ui.centered_and_justified(|ui| {
                ui.label(egui::RichText::new("no picture").color(theme::LEGEND).size(14.0));
            });
            return;
        };

        // The shape the transmission says, not the shape of the sample grid.
        // Drawn from its own numbers a 640 by 288 field is 10:9, which is a
        // 4:3 picture with the sides pushed in: how many samples a line was
        // cut into is a fact about the receiver's clock, and a field is half
        // a frame.
        let aspect = if f.aspect > 0.0 { f.aspect } else { 4.0 / 3.0 };
        let space = ui.available_size();
        let size = if space.x / space.y > aspect {
            egui::vec2(space.y * aspect, space.y)
        } else {
            egui::vec2(space.x, space.x / aspect)
        };
        ui.centered_and_justified(|ui| {
            // The ratio is told, not taken from the texture: `fit_to_exact_size`
            // still keeps the image's own proportions unless this is off, so a
            // 640 by 288 field was drawn at 20:9 whatever shape was asked for.
            let r =
                ui.add(egui::Image::new(tex).maintain_aspect_ratio(false).fit_to_exact_size(size));
            // What it is, where it is, and what was actually received: the
            // grid it was sampled into, then the lines that arrived out of
            // the lines a field has. A picture assembled from a third of its
            // lines is a picture of a fade, and analogue video has nothing
            // else to judge it by.
            let where_ = match &f.label {
                Some(l) => format!("{l}  {:.3} MHz", f.channel_hz / 1e6),
                None => format!("{:.3} MHz", f.channel_hz / 1e6),
            };
            let caption = format!(
                "{where_}  {}x{}  {} of {} lines",
                f.width, f.height, f.lines_seen, f.height
            );
            // Over the picture rather than beside it, so the image keeps the
            // whole pane and the caption cannot push it about as the text
            // changes width.
            ui.painter().text(
                r.rect.left_bottom() + egui::vec2(6.0, -6.0),
                egui::Align2::LEFT_BOTTOM,
                caption,
                egui::FontId::monospace(12.0),
                // A partial picture is worth flagging: a fade looks like a
                // picture until the count is read.
                if f.completeness() > 0.9 { theme::READOUT } else { theme::FAULT },
            );
        });
    }
}

/// Put a field into a texture, reusing the one already there when the size
/// matches: a field is half a megabyte and this runs fifty times a second.
fn upload(
    ctx: &egui::Context,
    f: &VideoFrame,
    old: Option<egui::TextureHandle>,
) -> egui::TextureHandle {
    let image = match f.pixels {
        Pixels::Rgb8 => egui::ColorImage::from_rgb([f.width, f.height], &f.samples),
        Pixels::Luma8 => {
            let rgb: Vec<u8> = f.samples.iter().flat_map(|&v| [v, v, v]).collect();
            egui::ColorImage::from_rgb([f.width, f.height], &rgb)
        }
    };
    match old {
        Some(mut t) if t.size() == [f.width, f.height] => {
            t.set(image, egui::TextureOptions::LINEAR);
            t
        }
        _ => ctx.load_texture("video", image, egui::TextureOptions::LINEAR),
    }
}
