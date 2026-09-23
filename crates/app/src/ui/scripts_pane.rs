//! The scripts panel: the `.sub` files this machine holds, as a tree.
//!
//! Two kinds of root, drawn the same way: a git repository dataset unpacked
//! into the cache, and the directory the packet list writes saved bursts
//! into. Selecting a file parses it, so what the panel says about a file is
//! what the transmitter would key rather than what its name claims; LOAD
//! puts it on the transmit strip and TUNE moves the dial to the frequency
//! the file names.

use super::*;
use crate::radio::SubFile;

/// One place `.sub` files come from.
struct Root {
    /// What the tree is called, which for a repository is the publisher's
    /// name for it and for the saved files is what they are.
    name: String,
    dir: std::path::PathBuf,
    /// Paths under `dir`, relative and sorted.
    files: Vec<String>,
}

/// What the panel holds between frames.
#[derive(Default)]
pub(super) struct ScriptsState {
    roots: Vec<Root>,
    /// Whether the tree has been read off the disk yet. Read once and on
    /// REFRESH: a panel that walked the cache every frame would stat a few
    /// thousand files sixty times a second to draw a list that changes when
    /// a dataset is downloaded.
    scanned: bool,
    /// Directories drawn open, by their path under a root. A directory
    /// starts closed, so a repository of hundreds of files opens as a
    /// handful of folders.
    open: std::collections::HashSet<String>,
    /// Sources drawn closed. The other way round from the folders inside
    /// them: a source nobody can see is a panel that looks empty.
    shut: std::collections::HashSet<String>,
    /// The file the panel is showing, parsed.
    picked: Option<(std::path::PathBuf, Result<SubFile, String>)>,
    /// Only what a text search is narrowed to, when one is typed.
    filter: String,
}

impl ScriptsState {
    /// Read the trees again, for the next frame that draws.
    pub fn rescan(&mut self) {
        self.scanned = false;
    }

    fn scan(&mut self) {
        self.roots.clear();
        if let Some(cache) = crate::data::cache() {
            for repo in datasets::git::REPOS {
                let files = datasets::git::files_with(repo, cache, ".sub");
                if files.is_empty() {
                    continue;
                }
                self.roots.push(Root {
                    name: repo.name.to_string(),
                    dir: repo.cache_dir(cache),
                    files,
                });
            }
        }
        let saved = crate::chain::default_sub_dir();
        let mut files = Vec::new();
        walk(&saved, &saved, &mut files);
        files.sort();
        if !files.is_empty() {
            self.roots.push(Root { name: "Saved here".into(), dir: saved, files });
        }
        self.scanned = true;
    }
}

/// Every `.sub` under `dir`, relative to `root`.
fn walk(root: &std::path::Path, dir: &std::path::Path, out: &mut Vec<String>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for e in entries.flatten() {
        let p = e.path();
        if p.is_dir() {
            walk(root, &p, out);
        } else if p.extension().is_some_and(|x| x.eq_ignore_ascii_case("sub")) {
            if let Ok(rel) = p.strip_prefix(root) {
                out.push(rel.to_string_lossy().replace('\\', "/"));
            }
        }
    }
}

/// What the panel wants done that it cannot do itself.
pub(super) enum Action {
    /// Stop drawing the panel.
    Hide,
    /// Open the datasets settings, which is where a repository is fetched.
    Open(Settings),
    /// Key this file, at the frequency it names, now.
    Transmit(SubFile),
}

/// The panel, over the files it lists.
pub(super) struct Scripts<'a> {
    pub st: &'a mut ScriptsState,
    /// What the radio can transmit, so a file it cannot reach says so rather
    /// than offering a button that fails when it is pressed.
    pub tx_range: Option<(f64, f64)>,
    /// Whether something is keyed already. One transmitter, one key.
    pub keyed: bool,
    pub acts: Vec<Action>,
}

impl<'a> Scripts<'a> {
    pub(super) fn show(mut self, ui: &mut egui::Ui) -> Vec<Action> {
        if !self.st.scanned {
            self.st.scan();
        }
        Panel::left("scripts")
            .default_size(250.0)
            .max_size(360.0)
            .frame(
                egui::Frame::NONE.fill(theme::PANEL).inner_margin(egui::Margin::symmetric(10, 10)),
            )
            .show(ui, |ui| {
                self.header(ui);
                ui.add_space(4.0);
                self.search(ui);
                ui.add_space(4.0);
                if self.st.roots.is_empty() {
                    self.empty(ui);
                    return;
                }
                // The tree takes what the detail card does not: a list that
                // filled the panel would push what a file is off the bottom,
                // which is the one thing the panel is read for.
                let detail_h = 118.0;
                let tree_h = (ui.available_height() - detail_h).max(80.0);
                egui::ScrollArea::vertical()
                    .id_salt("scripts-tree")
                    .max_height(tree_h)
                    .auto_shrink([false, true])
                    .show(ui, |ui| self.tree(ui));
                ui.add_space(6.0);
                self.detail(ui);
            });
        self.acts
    }

    fn header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            Line::new().legend("scripts").show(ui);
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if crate::icons::icon_button_sized(
                    ui,
                    crate::icons::Icon::Hide,
                    "Hide the scripts panel",
                    true,
                    false,
                    18.0,
                )
                .clicked()
                {
                    self.acts.push(Action::Hide);
                }
                if ui
                    .button("REFRESH")
                    .on_hover_text("Read the cache and the saved files again")
                    .clicked()
                {
                    self.st.rescan();
                }
            });
        });
    }

    fn search(&mut self, ui: &mut egui::Ui) {
        form::field(ui, &mut self.st.filter, "name");
    }

    /// What to do when there is nothing to list, which is the state every
    /// receiver starts in: the files come from a dataset nobody has
    /// downloaded yet.
    fn empty(&mut self, ui: &mut egui::Ui) {
        Line::new()
            .note("no .sub files held. Download a script repository in the data settings, or save a burst from the packet list.")
            .wrapped(ui);
        ui.add_space(6.0);
        if ui.button("DATA").clicked() {
            self.acts.push(Action::Open(Settings::Data));
        }
    }

    fn tree(&mut self, ui: &mut egui::Ui) {
        // Short of the scroll area's own width: the bar is drawn over the
        // rows, and a count painted at the right edge sat under it.
        let width = (ui.available_width() - ui.spacing().scroll.bar_width - 2.0).max(60.0);
        let filter = self.st.filter.to_lowercase();
        // Collected first: the rows are drawn from the state and the clicks
        // change it, and a row cannot borrow the set it toggles.
        let mut toggle: Option<String> = None;
        let mut pick: Option<std::path::PathBuf> = None;
        // Directories already on screen: twenty files under one folder draw
        // that folder once.
        let mut seen: std::collections::HashSet<String> = std::collections::HashSet::new();
        for root in &self.st.roots {
            // The top level is where a file came from: a repository somebody
            // publishes, or this receiver's own saves. Two sources with the
            // same folder names in them are two trees, not one merged one.
            let key = format!("{}\u{1}", root.name);
            let shut = self.st.shut.contains(&key) && filter.is_empty();
            let mark = if shut { "\u{25b8} " } else { "\u{25be} " };
            let count = root.files.len();
            if row_with(
                ui,
                width,
                0,
                &format!("{mark}{}", root.name),
                &count.to_string(),
                theme::LEGEND,
                false,
            ) {
                toggle = Some(key);
            }
            if shut {
                continue;
            }
            for rel in &root.files {
                if !filter.is_empty() && !rel.to_lowercase().contains(&filter) {
                    continue;
                }
                // The directories above this file, each drawn once, and the
                // file drawn only while its own directory is open. A filter
                // opens everything it matched: a search that hid its hits
                // behind a fold would be a search that found nothing.
                let parts: Vec<&str> = rel.split('/').collect();
                let mut path = String::new();
                let mut hidden = false;
                for (depth, part) in parts.iter().enumerate() {
                    let last = depth + 1 == parts.len();
                    if !path.is_empty() {
                        path.push('/');
                    }
                    path.push_str(part);
                    let key = format!("{}\u{1}{path}", root.name);
                    if last {
                        if hidden {
                            break;
                        }
                        let full = root.dir.join(rel);
                        let on = self.st.picked.as_ref().is_some_and(|(p, _)| *p == full);
                        let name = part.trim_end_matches(".sub").trim_end_matches(".SUB");
                        if row(ui, width, depth + 2, name, theme::VALUE, on) {
                            pick = Some(full);
                        }
                        break;
                    }
                    let open = !filter.is_empty() || self.st.open.contains(&key);
                    if hidden {
                        continue;
                    }
                    // Drawn once: the first file under a directory draws it,
                    // and the rest see it already on screen.
                    if seen.insert(key.clone()) {
                        let mark = if open { "\u{25be} " } else { "\u{25b8} " };
                        if row(ui, width, depth + 1, &format!("{mark}{part}"), theme::LEGEND, false)
                        {
                            toggle = Some(key.clone());
                        }
                    }
                    if !open {
                        hidden = true;
                    }
                }
            }
        }
        if let Some(k) = toggle {
            // A source's key ends at the separator; anything after it is a
            // folder inside one.
            let set = match k.ends_with('\u{1}') {
                true => &mut self.st.shut,
                false => &mut self.st.open,
            };
            if !set.remove(&k) {
                set.insert(k);
            }
        }
        if let Some(p) = pick {
            let parsed = SubFile::open(&p).map_err(|e| e.to_string());
            self.st.picked = Some((p, parsed));
        }
    }

    /// What the selected file is and what it would key.
    fn detail(&mut self, ui: &mut egui::Ui) {
        let Some((path, parsed)) = &self.st.picked else {
            text::hint(ui, "choose a file to see what it keys");
            return;
        };
        let name = path.file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
        match parsed {
            Err(e) => {
                panel::card(
                    ui,
                    Some(theme::FAULT),
                    |ui| {
                        Line::new().legend(&name).show(ui);
                    },
                    |ui| {
                        Line::new().note(e).wrapped(ui);
                    },
                );
            }
            Ok(f) => {
                let mhz = f.file.frequency as f64 / 1e6;
                let hz = f.file.frequency as f64;
                // What stops it going out, if anything does.
                let stop = match self.tx_range {
                    None => Some("this radio does not transmit".to_string()),
                    Some((lo, hi)) if hz < lo || hz > hi => {
                        Some(format!("{mhz:.4} MHz is outside what this radio transmits"))
                    }
                    _ if self.keyed => Some("something is keyed already".to_string()),
                    _ => None,
                };
                let file = f.clone();
                panel::card(
                    ui,
                    Some(theme::TRACE),
                    |ui| {
                        Line::new().legend(&name).show(ui);
                    },
                    |ui| {
                        Line::new().legend("protocol").value(file.label()).size(11.0).show(ui);
                        Line::new()
                            .legend("at")
                            .value(format!("{mhz:.4} MHz"))
                            .size(11.0)
                            .legend("for")
                            .value(format!("{:.2} s", file.file.duration().as_secs_f64()))
                            .size(11.0)
                            .legend("keying")
                            .value(file.file.preset.label())
                            .size(11.0)
                            .show(ui);
                        // One button, because there is only one thing to do
                        // with a file: send it. The dial, the channel and the
                        // key are all consequences of that, and an operator
                        // who has to arrange them by hand is doing the
                        // receiver's bookkeeping for it.
                        ui.horizontal(|ui| {
                            let go = ui
                                .add_enabled(stop.is_none(), egui::Button::new("TX"))
                                .on_hover_text(format!(
                                    "Key this file at {mhz:.4} MHz for {:.2} s",
                                    file.file.duration().as_secs_f64()
                                ));
                            if go.clicked() {
                                self.acts.push(Action::Transmit(file.clone()));
                            }
                            if let Some(why) = &stop {
                                Line::new().note(why).size(11.0).elided(ui);
                            }
                        });
                    },
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmpdir(name: &str) -> std::path::PathBuf {
        let d =
            std::env::temp_dir().join(format!("waveshark-scripts-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join("garage")).unwrap();
        d
    }

    #[test]
    fn the_tree_lists_every_sub_under_a_root_and_nothing_else() {
        let d = tmpdir("walk");
        std::fs::write(d.join("gate.sub"), "x").unwrap();
        std::fs::write(d.join("notes.txt"), "x").unwrap();
        std::fs::write(d.join("garage/left.sub"), "x").unwrap();
        std::fs::write(d.join("garage/right.SUB"), "x").unwrap();
        let mut out = Vec::new();
        walk(&d, &d, &mut out);
        out.sort();
        assert_eq!(out, vec!["garage/left.sub", "garage/right.SUB", "gate.sub"]);
        let _ = std::fs::remove_dir_all(&d);
    }

    /// What the packet list writes is what this panel reads: the same file,
    /// through the writer and back through the parser the transmitter uses.
    #[test]
    fn a_saved_burst_is_a_file_the_panel_can_load() {
        let d = tmpdir("saved");
        let save = decode::subghz::Save {
            frequency: 433_920_000,
            preset: decode::subghz::Preset::Ook,
            body: decode::subghz::key_of_decode("Princeton", 0xa1_3f_08).unwrap(),
        };
        let path = d.join(format!("{}.sub", save.file_stem("Princeton", std::time::UNIX_EPOCH)));
        std::fs::write(&path, save.text()).unwrap();
        let mut out = Vec::new();
        walk(&d, &d, &mut out);
        assert_eq!(out, vec!["Princeton_433.92MHz_0.sub"]);
        let f = SubFile::open(&path).expect("the panel parses what the packet list wrote");
        assert_eq!(f.file.frequency, 433_920_000);
        assert_eq!(f.label(), "Princeton");
        // Ten repeats of a 24-bit frame, which is what the encoder keys.
        assert_eq!(f.file.bursts[0].len(), 240);
        let _ = std::fs::remove_dir_all(&d);
    }
}

/// One row of the tree: an indent, a name, and the whole width clickable.
fn row(
    ui: &mut egui::Ui,
    width: f32,
    depth: usize,
    text: &str,
    colour: egui::Color32,
    selected: bool,
) -> bool {
    row_with(ui, width, depth, text, "", colour, selected)
}

/// A row with a figure at the right end, which a long name is cut short
/// before rather than pushing off the edge.
#[allow(clippy::too_many_arguments)]
fn row_with(
    ui: &mut egui::Ui,
    width: f32,
    depth: usize,
    text: &str,
    right: &str,
    colour: egui::Color32,
    selected: bool,
) -> bool {
    let h = table::ROW_H.max(18.0);
    let (rect, resp) = ui.allocate_exact_size(Vec2::new(width, h), Sense::click());
    if !ui.is_rect_visible(rect) {
        return false;
    }
    let p = ui.painter_at(rect);
    if selected {
        p.rect_filled(rect, 0.0, theme::WELL);
    } else if resp.hovered() {
        p.rect_filled(rect, 0.0, egui::Color32::from_rgb(0x24, 0x27, 0x2D));
    }
    let x = rect.left() + 4.0 + depth as f32 * 12.0;
    // Wide enough for the figure it is: these run to five digits, and a
    // fixed column cut the count of the largest collection in half.
    let right_w = if right.is_empty() { 0.0 } else { 8.0 * right.chars().count() as f32 + 6.0 };
    table::cell(
        &p,
        rect,
        x,
        rect.right() - x - right_w,
        text,
        if selected { theme::READOUT } else { colour },
    );
    if !right.is_empty() {
        table::cell(&p, rect, rect.right() - right_w, right_w, right, theme::LEGEND);
    }
    resp.clicked()
}
