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
//!
//! An SSTV picture is the opposite case and says so through
//! [`common::Cadence`]: it arrives a line at a time over two minutes and is
//! finished when it stops arriving, so it is kept rather than cleared.

use super::*;
use common::{Pixels, VideoFrame};
use nodes::dvbt_nodes::Want;

/// How long a picture with nothing to say about its cadence stays on screen.
/// What a frame does say is [`common::Cadence::hold_s`], which is half a
/// second for a camera and an hour for a picture that was built and finished.
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
    /// What that channel was called when it was picked, so the chooser still
    /// names it after it has faded out of the live list.
    watching_label: Option<String>,
}

impl VideoState {
    /// What the bus should publish: the one transmission being watched, or
    /// whatever comes.
    pub(super) fn rules(&self) -> Vec<crate::videobus::Rule> {
        match &self.watching {
            Some(k) => vec![crate::videobus::Rule::Channel(k.clone())],
            None => vec![crate::videobus::Rule::Everything],
        }
    }

    /// Watch one transmission, or whatever comes.
    pub(super) fn watch(&mut self, key: Option<String>) {
        self.watching_label = key.clone();
        self.watching = key;
    }

    pub(super) fn watching(&self) -> Option<&str> {
        self.watching.as_deref()
    }
}

pub(super) struct VideoPane<'a> {
    pub st: &'a mut VideoState,
    /// The newest field, or `None` when nothing is producing pictures.
    pub frame: Option<VideoFrame>,
    /// Every transmission the bus is seeing, with what it is called and how
    /// complete its last picture was.
    pub inputs: Vec<crate::chain::VideoInput>,
    /// Pictures written to disk this session, newest last.
    pub saved: Vec<std::path::PathBuf>,
    /// The television multiplexes being decoded, with the services each
    /// carries. A multiplex is many programmes on one frequency, so it needs
    /// a chooser of its own: the one above picks the transmission, this one
    /// picks what inside it is decoded.
    pub muxes: Vec<crate::chain::Multiplex>,
    /// Where the pane puts what it wants the receiver to do.
    pub cmds: &'a mut Vec<Cmd>,
}

impl VideoPane<'_> {
    pub fn show(self, ui: &mut egui::Ui) {
        let st = self.st;
        // The chooser is always there, including with nothing on the air: a
        // pane whose only control appears once two transmitters happen to be
        // up at once looks like a pane with no controls at all.
        ui.horizontal(|ui| {
            ui.add_space(12.0);
            Line::new().legend("watching").show(ui);
            let mut want = st.watching.clone();
            let shown = match &want {
                Some(k) => match self.inputs.iter().find(|i| &i.key == k) {
                    Some(i) => format!("{}  {:.0}%", i.label, i.completeness * 100.0),
                    // Off the air, but still what was asked for.
                    None => format!(
                        "{}  (waiting)",
                        st.watching_label.clone().unwrap_or_else(|| k.clone())
                    ),
                },
                None => "best picture".to_string(),
            };
            egui::ComboBox::from_id_salt("video-channel")
                .selected_text(value(shown))
                .width(240.0)
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut want, None, "best picture");
                    for i in &self.inputs {
                        ui.selectable_value(
                            &mut want,
                            Some(i.key.clone()),
                            format!("{}  {:.0}%", i.label, i.completeness * 100.0),
                        );
                    }
                    if self.inputs.is_empty() {
                        Line::new().note("nothing receiving").size(11.0).show(ui);
                    }
                });
            ui.add_space(12.0);
            let count = match self.inputs.len() {
                0 => "no channels".to_string(),
                1 => "1 channel".to_string(),
                n => format!("{n} channels"),
            };
            Line::new().legend(&count).show(ui);
            // Every finished still is written out without being asked, so
            // the only thing a person needs from the pane is where they went.
            if let Some(last) = self.saved.last() {
                ui.add_space(12.0);
                let n = self.saved.len();
                let what = match n {
                    1 => "1 picture saved".to_string(),
                    n => format!("{n} pictures saved"),
                };
                let dir = last.parent().unwrap_or(last).display().to_string();
                let r = Line::new().legend(&what).show(ui);
                r.on_hover_text(dir.clone());
                if ui.small_button("open").clicked() {
                    ui.ctx().open_url(egui::OpenUrl::new_tab(format!("file://{dir}")));
                }
            }
            for mux in &self.muxes {
                ui.add_space(12.0);
                Line::new().legend("service").show(ui);
                let mut pick = mux.wanted.clone();
                let shown = match &pick {
                    Want::Any => nodes::dvbt_nodes::ANY.to_string(),
                    Want::Named(n) => n.clone(),
                    Want::Id(id) => mux
                        .services
                        .iter()
                        .find(|s| s.id == *id)
                        .map_or_else(|| format!("service {id}"), nodes::dvbt_nodes::service_label),
                };
                egui::ComboBox::from_id_salt(("dvb-service", mux.node))
                    .selected_text(value(shown))
                    .width(220.0)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut pick, Want::Any, nodes::dvbt_nodes::ANY);
                        for s in &mux.services {
                            ui.selectable_value(
                                &mut pick,
                                Want::of(s),
                                nodes::dvbt_nodes::service_label(s),
                            );
                        }
                        if mux.services.is_empty() {
                            Line::new().note("no services described yet").size(11.0).show(ui);
                        }
                    });
                if pick != mux.wanted {
                    // As a name or a number rather than a position, because
                    // this is written into the patch and read back by a
                    // rebuild, which happens before any table has arrived.
                    self.cmds.push(Cmd::NodeParam(
                        mux.node,
                        nodes::dvbt_nodes::SERVICE.into(),
                        pick.setting(),
                    ));
                }
            }
            if want != st.watching {
                st.watching_label = want
                    .as_ref()
                    .and_then(|k| self.inputs.iter().find(|i| &i.key == k))
                    .map(|i| i.label.clone());
                st.watching = want.clone();
                self.cmds.push(Cmd::WatchVideo(match want {
                    Some(k) => vec![crate::videobus::Rule::Channel(k)],
                    None => vec![crate::videobus::Rule::Everything],
                }));
            }
        });
        ui.add_space(4.0);
        if let Some(f) = self.frame {
            if is_new(st.shown.as_ref(), &f) {
                st.texture = Some(upload(ui.ctx(), &f, st.texture.take()));
                st.last = Some(std::time::Instant::now());
                st.shown = Some(f);
            }
        }
        let hold = st
            .shown
            .as_ref()
            .map(|f| std::time::Duration::from_secs_f64(f.cadence.hold_s()))
            .unwrap_or(HOLD);
        if st.last.is_some_and(|t| t.elapsed() > hold) {
            st.texture = None;
            st.shown = None;
            st.last = None;
        }

        let (Some(tex), Some(f)) = (st.texture.as_ref(), st.shown.as_ref()) else {
            ui.centered_and_justified(|ui| {
                Line::new().note("no picture").size(14.0).show(ui);
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

/// Whether this picture is worth uploading over the one on screen.
///
/// Not the sequence number alone. For a camera that counts fields, so every
/// one differs; for a still it names the picture, and every row of an SSTV
/// transmission carries the same number for two minutes. Keying on it alone
/// drew the first line of a picture and then nothing for the rest of the
/// transmission, while the file on disk was complete.
fn is_new(shown: Option<&VideoFrame>, f: &VideoFrame) -> bool {
    shown.is_none_or(|s| {
        s.sequence != f.sequence
            || s.lines_seen != f.lines_seen
            || s.width != f.width
            || s.height != f.height
            || s.channel_hz != f.channel_hz
    })
}

/// Put a field into a texture, reusing the one already there when the size
/// matches: a field is half a megabyte and this runs fifty times a second.
fn upload(
    ctx: &egui::Context,
    f: &VideoFrame,
    old: Option<egui::TextureHandle>,
) -> egui::TextureHandle {
    let image = match f.pixels {
        // Already the shape a texture is, so this is a copy rather than a
        // pass over every pixel. A 1080 line picture is two million of them,
        // fifty times a second, on the thread that draws everything else.
        Pixels::Rgba8 => egui::ColorImage::from_rgba_unmultiplied([f.width, f.height], &f.samples),
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

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(sequence: u64, lines: usize) -> VideoFrame {
        VideoFrame {
            system: "SSTV",
            channel_hz: 144_500_000.0,
            label: Some("Martin 1".into()),
            width: 2,
            height: 4,
            aspect: 4.0 / 3.0,
            pixels: Pixels::Rgb8,
            samples: std::sync::Arc::new(vec![0u8; 2 * 4 * 3]),
            lines_seen: lines,
            sequence,
            update: common::Update::Whole,
            cadence: common::Cadence::Still,
            sent_at_us: None,
        }
    }

    /// A picture filling in is the same picture with more of it, and that has
    /// to reach the screen: this is the bug where an SSTV transmission drew
    /// one line and then sat there for two minutes.
    #[test]
    fn a_still_that_grew_is_drawn_again() {
        let one = frame(1, 1);
        assert!(is_new(None, &one), "the first picture is new");
        assert!(!is_new(Some(&one), &one), "the same picture is not");
        assert!(is_new(Some(&one), &frame(1, 2)), "a line arrived");
        assert!(is_new(Some(&frame(1, 4)), &frame(2, 1)), "and a new transmission");
    }
}
