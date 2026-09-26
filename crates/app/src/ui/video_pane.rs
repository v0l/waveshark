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

/// How long a picture with nothing to say about its cadence stays on screen.
/// What a frame does say is [`common::Cadence::hold_s`], which is half a
/// second for a camera and an hour for a picture that was built and finished.
const HOLD: std::time::Duration = std::time::Duration::from_millis(500);

const OSD_HOLD: std::time::Duration = std::time::Duration::from_secs(4);

const DEFAULT_PICTURE_FRAC: f32 = 0.65;
const PICTURE_FRAC_RANGE: std::ops::RangeInclusive<f32> = 0.2..=0.9;

const COLS: [(&str, f32); 10] = [
    ("name", 200.0),
    ("on now", 220.0),
    ("next (UTC)", 240.0),
    ("provider", 130.0),
    ("access", 80.0),
    ("video", 90.0),
    ("audio", 90.0),
    ("service", 70.0),
    ("system", 70.0),
    ("frequency", 110.0),
];

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
    split: Option<f32>,
    splitting: bool,
    osd_until: Option<std::time::Instant>,
    osd_title: String,
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
    pub muxes: Vec<crate::videobus::Offered>,
    /// Where the pane puts what it wants the receiver to do.
    pub cmds: &'a mut Vec<Cmd>,
}

impl VideoPane<'_> {
    pub fn show(self, ui: &mut egui::Ui) {
        let st = self.st;
        if st.watching.as_deref().is_some_and(|k| !offered(k, &self.inputs, &self.muxes)) {
            st.watch(None);
            self.cmds.push(Cmd::WatchVideo(st.rules()));
        }
        let before = pick_of(st.watching.as_deref(), &self.muxes);
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

        let top = ui.cursor().top();
        let usable = (ui.available_height() - SPLIT_GRIP_H).max(200.0);
        let frac = st
            .split
            .unwrap_or(DEFAULT_PICTURE_FRAC)
            .clamp(*PICTURE_FRAC_RANGE.start(), *PICTURE_FRAC_RANGE.end());
        let on = playing(&before, st.shown.as_ref(), &self.muxes);
        let osd = Osd::of(&on, st, &self.inputs, &self.muxes, &self.saved);
        ui.allocate_ui(Vec2::new(ui.available_width(), usable * frac), |ui| {
            ui.set_min_size(ui.available_size());
            picture(ui, st, &osd);
        });
        st.split = Some(split_divider(
            ui,
            top,
            usable,
            frac,
            &mut st.splitting,
            PICTURE_FRAC_RANGE,
            DEFAULT_PICTURE_FRAC,
        ));
        ui.add_space(4.0);

        let rows = rows(&self.inputs, &self.muxes);
        let mut want = before.clone();
        if let Some(picked) = table(ui, &rows, &before) {
            want = picked;
        }
        if want != before {
            st.watching_label = match &want {
                Pick::Channel(k) => {
                    self.inputs.iter().find(|i| &i.key == k).map(|i| i.label.clone())
                }
                _ => None,
            };
            st.watching = match &want {
                Pick::First => None,
                Pick::Channel(k) | Pick::Programme(k, _, _) => Some(k.clone()),
            };
            self.cmds.push(Cmd::WatchVideo(st.rules()));
        }
        let playing = playing(&want, st.shown.as_ref(), &self.muxes);
        self.cmds.extend(orders(&self.muxes, &playing));
    }
}

fn picture(ui: &mut egui::Ui, st: &mut VideoState, osd: &Osd) {
    let area = ui.available_rect_before_wrap();
    let shown = match (st.texture.as_ref(), st.shown.as_ref()) {
        (Some(tex), Some(f)) => {
            let aspect = if f.aspect > 0.0 { f.aspect } else { 4.0 / 3.0 };
            let size = if area.width() / area.height() > aspect {
                egui::vec2(area.height() * aspect, area.height())
            } else {
                egui::vec2(area.width(), area.width() / aspect)
            };
            let rect = Rect::from_center_size(area.center(), size);
            egui::Image::new(tex).maintain_aspect_ratio(false).paint_at(ui, rect);
            Some(rect)
        }
        _ => {
            let mut note = ui.new_child(
                egui::UiBuilder::new()
                    .max_rect(area)
                    .layout(egui::Layout::centered_and_justified(egui::Direction::TopDown)),
            );
            Line::new().note("no picture").size(14.0).show(&mut note);
            None
        }
    };
    let rect = shown.unwrap_or(area);
    let resp = ui.interact(rect, ui.id().with("osd"), Sense::click());
    ui.allocate_rect(area, Sense::hover());

    let now = std::time::Instant::now();
    if osd.title != st.osd_title {
        st.osd_title = osd.title.clone();
        st.osd_until = Some(now + OSD_HOLD);
    }
    let timed = st.osd_until.is_some_and(|t| t > now);
    if resp.clicked() {
        st.osd_until = if timed { None } else { Some(now + OSD_HOLD) };
    }
    let timed = st.osd_until.is_some_and(|t| t > now);
    if let Some(left) = st.osd_until.and_then(|t| t.checked_duration_since(now)) {
        ui.ctx().request_repaint_after(left);
    }
    let visible = shown.is_none() || resp.hovered() || timed;
    let alpha = ui.ctx().animate_bool_with_time(resp.id, visible, 0.2);
    if alpha > 0.0 {
        osd.show(ui, rect, alpha);
    }
}

/// What the picture is, drawn over it on demand: what a set top box puts up
/// on a change of channel. A guide's now and next are rows on it.
struct Osd {
    title: String,
    number: Option<u16>,
    facts: Vec<(&'static str, String, Color32)>,
    saved: Option<(String, String)>,
}

impl Osd {
    fn of(
        on: &Pick,
        st: &VideoState,
        inputs: &[crate::chain::VideoInput],
        muxes: &[crate::videobus::Offered],
        saved: &[std::path::PathBuf],
    ) -> Self {
        let f = st.shown.as_ref();
        let service = match on {
            Pick::Programme(_, from, setting) => {
                muxes.iter().find(|o| o.from == *from).and_then(|o| {
                    let list = &o.programmes.list;
                    list.iter().find(|p| &p.setting == setting && p.service.is_some()).or_else(
                        || {
                            let id = o.programmes.on?;
                            list.iter().find(|p| p.service.as_ref().is_some_and(|s| s.id == id))
                        },
                    )
                })
            }
            _ => None,
        };
        let input = match on {
            Pick::Channel(k) => inputs.iter().find(|i| &i.key == k),
            _ => None,
        };
        let title = service
            .map(|p| p.label.clone())
            .or_else(|| f.and_then(|f| f.label.clone()))
            .or_else(|| input.map(|i| i.label.clone()))
            .or_else(|| st.watching_label.clone())
            .unwrap_or_default();
        let mut facts = Vec::new();
        if let Some(s) = service.and_then(|p| p.service.as_ref()) {
            if let Some(p) = &s.provider {
                facts.push(("provider", p.clone(), theme::VALUE));
            }
            if s.scrambled {
                facts.push(("access", "scrambled".to_string(), theme::FAULT));
            }
            let codecs: Vec<&str> = [
                s.video.map(|v| v.trim_end_matches(" video")),
                s.audio.map(|a| a.trim_end_matches(" audio")),
            ]
            .into_iter()
            .flatten()
            .collect();
            if !codecs.is_empty() {
                facts.push(("coding", codecs.join(" "), theme::VALUE));
            }
        }
        if let Some(f) = f {
            facts.push(("tuned", format!("{:.3} MHz", f.channel_hz / 1e6), theme::READOUT));
            facts.push(("system", f.system.to_string(), theme::VALUE));
            facts.push(("picture", format!("{}x{}", f.width, f.height), theme::TRACE));
            let lines = format!("{} of {}", f.lines_seen, f.height);
            let tint = if f.completeness() > 0.9 { theme::TRACE } else { theme::FAULT };
            facts.push(("lines", lines, tint));
        }
        let saved = saved.last().map(|last| {
            let what = match saved.len() {
                1 => "1 picture saved".to_string(),
                n => format!("{n} pictures saved"),
            };
            (what, last.parent().unwrap_or(last).display().to_string())
        });
        Self { title, number: service.and_then(|p| p.service.as_ref()).map(|s| s.id), facts, saved }
    }

    fn show(&self, ui: &mut egui::Ui, over: Rect, alpha: f32) {
        let inner = over.shrink2(Vec2::new(14.0, 9.0));
        let mut sizing =
            ui.new_child(egui::UiBuilder::new().max_rect(inner).sizing_pass().invisible());
        self.rows(&mut sizing);
        let h = sizing.min_rect().height();
        let at =
            Rect::from_min_max(Pos2::new(inner.left(), inner.bottom() - h), inner.right_bottom());
        let band = Rect::from_min_max(Pos2::new(over.left(), at.top() - 9.0), over.right_bottom());
        ui.painter().rect_filled(band, 0.0, theme::CHASSIS.gamma_multiply(0.85 * alpha));
        let mut ui = ui.new_child(egui::UiBuilder::new().max_rect(at));
        ui.set_opacity(alpha);
        self.rows(&mut ui);
    }

    fn rows(&self, ui: &mut egui::Ui) {
        if !self.title.is_empty() {
            let mut title = Line::new().value(&self.title).size(20.0);
            if let Some(n) = self.number {
                title = title.gap(16.0).legend("service").value(n.to_string());
            }
            title.show(ui);
        }
        ui.horizontal_wrapped(|ui| {
            for (legend, value, tint) in &self.facts {
                Line::new().legend(legend).value(value).tint(*tint).show(ui);
                ui.add_space(10.0);
            }
        });
        if let Some((what, dir)) = &self.saved {
            ui.horizontal(|ui| {
                Line::new().legend(what).show(ui).on_hover_text(dir.clone());
                if ui.small_button("OPEN").clicked() {
                    ui.ctx().open_url(egui::OpenUrl::new_tab(format!("file://{dir}")));
                }
            });
        }
    }
}

struct Row {
    pick: Pick,
    cells: [(String, Color32); COLS.len()],
    summary: Option<String>,
}

fn blank_cells() -> [(String, Color32); COLS.len()] {
    std::array::from_fn(|_| (String::new(), theme::LEGEND))
}

fn clock(start_utc: Option<i64>) -> String {
    let at = start_utc.and_then(|t| chrono::DateTime::from_timestamp(t, 0));
    at.map_or_else(String::new, |t| t.format("%H:%M ").to_string())
}

fn rows(inputs: &[crate::chain::VideoInput], muxes: &[crate::videobus::Offered]) -> Vec<Row> {
    let mut out = Vec::new();
    for i in inputs.iter().filter(|i| !muxes.iter().any(|o| o.key() == i.key)) {
        let mut cells = blank_cells();
        cells[0] = (i.label.clone(), theme::VALUE);
        cells[5] = (format!("{:.0}%", i.completeness * 100.0), theme::TRACE);
        out.push(Row { pick: Pick::Channel(i.key.clone()), cells, summary: None });
    }
    for o in muxes {
        let system = (o.programmes.system.to_string(), theme::LEGEND);
        let hz = (format!("{:.3} MHz", o.programmes.channel_hz / 1e6), theme::VALUE);
        for p in &o.programmes.list {
            let stream = |s: Option<&'static str>, suffix: &str| match s {
                Some(s) => (s.trim_end_matches(suffix).to_string(), theme::VALUE),
                None => (String::new(), theme::LEGEND),
            };
            let mut cells = blank_cells();
            cells[8] = system.clone();
            cells[9] = hz.clone();
            let mut summary = None;
            match &p.service {
                Some(s) => {
                    cells[0] = (
                        s.name.clone().unwrap_or_else(|| format!("service {}", s.id)),
                        if s.scrambled { theme::LEGEND } else { theme::TRACE },
                    );
                    if let Some(now) = &s.now {
                        cells[1] = (now.title.clone(), theme::TRACE);
                        summary = Some(now.summary.clone()).filter(|t| !t.is_empty());
                    }
                    if let Some(next) = &s.next {
                        cells[2] =
                            (format!("{}{}", clock(next.start_utc), next.title), theme::LEGEND);
                    }
                    cells[3] = (s.provider.clone().unwrap_or_default(), theme::LEGEND);
                    cells[4] = match s.scrambled {
                        true => ("scrambled".to_string(), theme::FAULT),
                        false => ("clear".to_string(), theme::OK),
                    };
                    cells[5] = stream(s.video, " video");
                    cells[6] = stream(s.audio, " audio");
                    cells[7] = (s.id.to_string(), theme::LEGEND);
                }
                None => cells[0] = (p.label.clone(), theme::VALUE),
            }
            out.push(Row {
                pick: Pick::Programme(o.key(), o.from, p.setting.clone()),
                cells,
                summary,
            });
        }
    }
    out
}

fn table(ui: &mut egui::Ui, rows: &[Row], chosen: &Pick) -> Option<Pick> {
    let width: f32 = COLS.iter().map(|(_, w)| w).sum::<f32>() + 24.0;
    let mut picked = None;
    egui::ScrollArea::horizontal().id_salt("video-table").auto_shrink([false, false]).show(
        ui,
        |ui| {
            ui.set_min_width(width);
            let (rect, _) = ui.allocate_exact_size(Vec2::new(width, table::ROW_H), Sense::hover());
            let p = ui.painter_at(rect);
            let mut x = rect.left() + 12.0;
            for (name, w) in COLS {
                table::cell(&p, rect, x, w, name, theme::LEGEND);
                x += w;
            }
            p.line_segment(
                [Pos2::new(rect.left(), rect.bottom()), Pos2::new(rect.right(), rect.bottom())],
                Stroke::new(1.0, theme::ETCH),
            );
            egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
                for (n, row) in rows.iter().enumerate() {
                    let (rect, resp) =
                        ui.allocate_exact_size(Vec2::new(width, table::ROW_H), Sense::click());
                    if !ui.is_rect_visible(rect) {
                        continue;
                    }
                    let p = ui.painter_at(rect);
                    if n % 2 == 1 {
                        p.rect_filled(rect, 0.0, Color32::from_rgb(0x24, 0x27, 0x2D));
                    }
                    if resp.hovered() {
                        p.rect_filled(rect, 0.0, theme::WELL);
                    }
                    if row.pick == *chosen {
                        p.rect_filled(
                            Rect::from_min_max(
                                rect.left_top(),
                                Pos2::new(rect.left() + 3.0, rect.bottom()),
                            ),
                            0.0,
                            theme::READOUT,
                        );
                    }
                    let mut x = rect.left() + 12.0;
                    for ((t, c), (_, w)) in row.cells.iter().zip(COLS) {
                        table::cell(&p, rect, x, w, t, *c);
                        x += w;
                    }
                    let resp = match &row.summary {
                        Some(summary) => resp.on_hover_text(summary),
                        None => resp,
                    };
                    if resp.clicked() {
                        picked = Some(row.pick.clone());
                    }
                }
            });
        },
    );
    picked
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

#[derive(Clone, Debug, PartialEq)]
enum Pick {
    First,
    Channel(String),
    Programme(String, usize, pipeline::ParamValue),
}

fn offered(
    key: &str,
    inputs: &[crate::chain::VideoInput],
    muxes: &[crate::videobus::Offered],
) -> bool {
    inputs.iter().any(|i| i.key == key) || muxes.iter().any(|o| o.key() == key)
}

fn pick_of(watching: Option<&str>, muxes: &[crate::videobus::Offered]) -> Pick {
    let Some(k) = watching else { return Pick::First };
    match muxes.iter().find(|o| o.key() == k) {
        Some(o) => Pick::Programme(k.to_string(), o.from, o.programmes.wanted.clone()),
        None => Pick::Channel(k.to_string()),
    }
}

fn playing(want: &Pick, shown: Option<&VideoFrame>, muxes: &[crate::videobus::Offered]) -> Pick {
    let Some(f) = shown.filter(|_| *want == Pick::First) else { return want.clone() };
    match pick_of(Some(&crate::videobus::key_of(f)), muxes) {
        Pick::Channel(_) => Pick::First,
        on => on,
    }
}

fn orders(muxes: &[crate::videobus::Offered], pick: &Pick) -> Vec<Cmd> {
    let mut out = Vec::new();
    let playing = match pick {
        Pick::First => {
            return muxes
                .iter()
                .filter(|o| o.programmes.is_idle())
                .map(|o| {
                    Cmd::NodeParam(o.from, o.programmes.param.to_string(), o.programmes.any.clone())
                })
                .collect();
        }
        Pick::Programme(_, from, setting) => Some((*from, setting)),
        Pick::Channel(_) => None,
    };
    for o in muxes {
        // As a name or a number rather than a position, because
        // this is written into the patch and read back by a
        // rebuild, which happens before any table has arrived.
        let set = |to: &pipeline::ParamValue| {
            Cmd::NodeParam(o.from, o.programmes.param.to_string(), to.clone())
        };
        match playing {
            Some((from, setting)) if from == o.from => {
                if o.programmes.wanted != *setting {
                    out.push(set(setting));
                }
            }
            _ if !o.programmes.is_idle() => out.push(set(&o.programmes.idle)),
            _ => {}
        }
    }
    out
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

    fn offered(from: usize, hz: f64, wanted: pipeline::ParamValue) -> crate::videobus::Offered {
        let programme = |label: &str, setting| pipeline::Programme {
            label: label.into(),
            setting,
            service: None,
        };
        crate::videobus::Offered {
            from,
            programmes: std::sync::Arc::new(pipeline::Programmes {
                system: "DVB-S2",
                channel_hz: hz,
                param: "service",
                wanted,
                on: None,
                idle: pipeline::ParamValue::Int(-1),
                any: pipeline::ParamValue::Int(0),
                list: vec![programme(
                    "BBC Two HD",
                    pipeline::ParamValue::Text("BBC Two HD".into()),
                )],
            }),
        }
    }

    fn said(cmds: Vec<Cmd>) -> Vec<(usize, String, pipeline::ParamValue)> {
        cmds.into_iter()
            .filter_map(|c| match c {
                Cmd::NodeParam(n, name, v) => Some((n, name, v)),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn the_osd_over_the_first_picture_names_the_service_the_multiplex_is_decoding() {
        let mut mux = offered(3, 1_097e6, pipeline::ParamValue::Int(0));
        let list = &mut std::sync::Arc::get_mut(&mut mux.programmes).expect("one owner").list;
        list[0].service = Some(pipeline::Service {
            id: 6940,
            name: Some("BBC Two HD".into()),
            provider: Some("BSkyB".into()),
            video: Some("H.264 video"),
            audio: Some("MPEG-2 audio"),
            ..Default::default()
        });
        std::sync::Arc::get_mut(&mut mux.programmes).expect("one owner").on = Some(6940);
        let muxes = [mux];
        let mut shown = frame(1, 1080);
        (shown.system, shown.channel_hz, shown.width, shown.height) =
            ("DVB-S2", 1_097e6, 1920, 1080);
        let st = VideoState { shown: Some(shown.clone()), ..Default::default() };
        let on = playing(&Pick::First, Some(&shown), &muxes);
        let osd = Osd::of(&on, &st, &[], &muxes, &[]);
        assert_eq!((osd.title.as_str(), osd.number), ("BBC Two HD", Some(6940)));
        let facts: Vec<(&str, &str)> = osd.facts.iter().map(|(l, v, _)| (*l, v.as_str())).collect();
        assert_eq!(
            facts,
            [
                ("provider", "BSkyB"),
                ("coding", "H.264 MPEG-2"),
                ("tuned", "1097.000 MHz"),
                ("system", "DVB-S2"),
                ("picture", "1920x1080"),
                ("lines", "1080 of 1080"),
            ]
        );

        let idle = Osd::of(&Pick::First, &VideoState::default(), &[], &[], &[]);
        assert_eq!((idle.title.as_str(), idle.facts.len()), ("", 0));
    }

    #[test]
    fn one_programme_plays_and_every_other_multiplex_stops_decoding() {
        use pipeline::ParamValue::{Int, Text};
        let muxes = [offered(3, 1_097e6, Int(0)), offered(7, 1_068e6, Int(0))];
        let bbc_two = Pick::Programme(muxes[0].key(), 3, Text("BBC Two HD".into()));
        assert_eq!(
            said(orders(&muxes, &bbc_two)),
            [(3, "service".into(), Text("BBC Two HD".into())), (7, "service".into(), Int(-1))]
        );
        let camera = Pick::Channel("FPV:5800000".into());
        assert_eq!(
            said(orders(&muxes, &camera)),
            [(3, "service".into(), Int(-1)), (7, "service".into(), Int(-1))],
            "a picture that is not a programme stops every multiplex"
        );
        assert!(said(orders(&muxes, &Pick::First)).is_empty());
        let settled =
            [offered(3, 1_097e6, Text("BBC Two HD".into())), offered(7, 1_068e6, Int(-1))];
        assert!(said(orders(&settled, &bbc_two)).is_empty(), "nothing more to ask once it is so");
        assert_eq!(
            pick_of(Some(&muxes[1].key()), &settled),
            Pick::Programme(muxes[1].key(), 7, Int(-1))
        );
        assert_eq!(
            said(orders(&settled, &Pick::First)),
            [(7, "service".into(), Int(0))],
            "with nothing picked a stopped multiplex goes back to its first picture"
        );
    }

    #[test]
    fn a_service_row_says_what_is_on_now_and_when_the_next_starts() {
        let mut mux = offered(3, 1_097e6, pipeline::ParamValue::Int(0));
        let listing = std::sync::Arc::make_mut(&mut mux.programmes);
        listing.list.push(listing.list[0].clone());
        listing.list[0].service = Some(pipeline::Service {
            id: 6943,
            name: Some("BBC One NI HD".into()),
            now: Some(pipeline::Showing {
                title: "Antiques Road Trip".into(),
                summary: "Two experts".into(),
                start_utc: Some(1_790_350_200),
                duration_s: 2700,
            }),
            next: Some(pipeline::Showing {
                title: "New: Pointless".into(),
                start_utc: Some(1_790_352_900),
                ..Default::default()
            }),
            ..Default::default()
        });
        let listed = rows(&[], &[mux]);
        assert_eq!(listed.len(), 2);
        let bbc = &listed[0];
        let cells: Vec<&str> = bbc.cells.iter().map(|(t, _)| t.as_str()).collect();
        assert_eq!(
            cells,
            [
                "BBC One NI HD",
                "Antiques Road Trip",
                "16:15 New: Pointless",
                "",
                "clear",
                "",
                "",
                "6943",
                "DVB-S2",
                "1097.000 MHz"
            ]
        );
        assert_eq!(bbc.summary.as_deref(), Some("Two experts"));
        assert_eq!(listed[1].summary, None, "a programme with no service says nothing");
    }

    #[test]
    fn the_first_picture_keeps_its_multiplex_and_stops_the_others() {
        use pipeline::ParamValue::Int;
        let muxes = [offered(3, 1_097e6, Int(0)), offered(7, 1_068e6, Int(0))];
        let mut shown = frame(1, 288);
        shown.system = "DVB-S2";
        shown.channel_hz = 1_068e6;
        let pick = playing(&Pick::First, Some(&shown), &muxes);
        assert_eq!(said(orders(&muxes, &pick)), [(3, "service".into(), Int(-1))]);
        let camera = frame(1, 288);
        assert_eq!(playing(&Pick::First, Some(&camera), &muxes), Pick::First);
        assert_eq!(playing(&Pick::First, None, &muxes), Pick::First);
    }
}
