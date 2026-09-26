//! The signal chain view: the graph the receiver is running, and the editing
//! of it in manual mode.

use super::state::{ChainSide, ChainState};
use super::*;

const ZOOM: std::ops::RangeInclusive<f32> = 0.2..=2.0;
const CANVAS_MAX: f32 = 100_000.0;
const FIT_MARGIN: f32 = 24.0;
const ZOOM_STEPS: f32 = 4.0;

/// The chain view, over the graph the receiver is running and the one the
/// operator has drawn.
pub(super) struct Chain<'a> {
    pub st: &'a mut ChainState,
    pub cmds: &'a mut Vec<Cmd>,
    /// The open-a-file dialog, for a stage whose setting is a path.
    pub files: &'a mut super::state::FilePick,
}

impl Chain<'_> {
    /// The signal chain the listening channel is running.
    pub(super) fn show(mut self, ui: &mut egui::Ui) {
        let Some(full) = self.st.topo.clone() else {
            ui.centered_and_justified(|ui| {
                Line::new().note("The radio is stopped, so no chain is running.").show(ui);
            });
            return;
        };
        // One direction at a time. They are separate chains that meet only at
        // the radio, and drawn together the transmit half is four stages
        // hidden behind forty.
        let topo = one_side(&full, self.st.side);
        // Stages running in the half not drawn, so they are not offered as
        // ghosts in this one.
        let elsewhere: Vec<u64> = full
            .nodes
            .iter()
            .filter(|n| !topo.nodes.iter().any(|t| t.id == n.id))
            .filter_map(|n| n.tag)
            .collect();
        // Node ids are positions in the built graph, so a rebuild can leave
        // the selection pointing at a stage that is no longer there.
        if self.st.sel.is_some_and(|s| {
            s != crate::chainview::SOURCE && !topo.nodes.iter().any(|n| n.id.0 == s)
        }) {
            self.st.sel = None;
        }
        // The inspector takes a column on the right when a stage is selected,
        // rather than floating over the graph: what a stage is set to is read
        // against where it sits in the chain, and a panel covering the chain
        // hides half of that.
        let mut act = crate::chainview::Interaction { selected: self.st.sel, ..Default::default() };
        let mut browse = None;
        let mut off = None;
        let waiting = self
            .st
            .pick
            .filter(|id| self.st.edit.manual && !full.nodes.iter().any(|n| n.tag == Some(*id)))
            .and_then(|id| self.st.patch.stage(id).cloned());
        let mut setting = None;
        if self.st.sel.is_some() || waiting.is_some() {
            Panel::right("chain-inspector")
                .default_size(260.0)
                // Capped, because the panel takes its width from what is in
                // it: one stage with a long file name in a box otherwise
                // pushes the chain itself off the screen.
                .max_size(340.0)
                .frame(
                    egui::Frame::NONE
                        .fill(theme::PANEL)
                        .inner_margin(egui::Margin::symmetric(12, 10)),
                )
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        if let Some(stage) = &waiting {
                            let desc = self.st.waiting.iter().find(|w| w.id == stage.id);
                            setting = crate::chainview::waiting_inspector(ui, stage, desc)
                                .map(|(name, value)| (stage.id, name, value));
                        } else if let Some(sel) = self.st.sel {
                            act.changed =
                                crate::chainview::inspector(ui, &topo, sel, &mut browse, &mut off);
                        }
                    });
                });
        }
        Panel::left("chain-palette")
            .default_size(210.0)
            .frame(
                egui::Frame::NONE.fill(theme::PANEL).inner_margin(egui::Margin::symmetric(10, 10)),
            )
            .show(ui, |ui| self.palette(ui));
        let manual = self.st.edit.manual;
        let mut scene = *self.st.scene.get_or_insert_with(|| {
            egui::Rect::from_min_size(egui::Pos2::ZERO, ui.available_size())
        });
        let drawn = egui::Scene::new()
            .zoom_range(ZOOM)
            .max_inner_size(egui::Vec2::splat(CANVAS_MAX))
            .show(ui, &mut scene, |ui| {
                let layer = ui.layer_id();
                let from = ui.ctx().graphics_mut(|g| g.entry(layer).next_idx().0);
                let act = crate::chainview::draw(
                    ui,
                    &topo,
                    self.st.latency,
                    self.st.sel,
                    &mut self.st.edit,
                    Some(&self.st.patch),
                    self.st.wire,
                    self.st.pick,
                    &elsewhere,
                    &self.st.scopes,
                    &self.st.waiting,
                );
                let zoom = ui.ctx().layer_transform_to_global(layer).map_or(1.0, |t| t.scaling);
                sharpen_text(ui, layer, from, zoom);
                act
            })
            .inner;
        if manual && drawn.pan != egui::Vec2::ZERO {
            scene = scene.translate(-drawn.pan);
        }
        if std::mem::take(&mut self.st.fit)
            && let Some(b) = drawn.bounds
        {
            scene = b.expand(FIT_MARGIN);
        }
        self.st.scene = Some(scene);
        self.st.sel = drawn.selected;
        if let Some((kind, at, attach)) = drawn.dropped.clone() {
            self.st.add_stage(self.cmds, &kind, at, attach);
        }
        let typing = ui.ctx().egui_wants_keyboard_input();
        if manual {
            if drawn.picked.is_some() {
                self.st.pick = drawn.picked;
                self.st.wire = None;
            } else if drawn.blank {
                self.st.pick = None;
                self.st.wire = None;
            }
            if drawn.wire.is_some() {
                self.st.wire = drawn.wire;
            }
            // Delete takes out whichever of the two is selected, which is
            // what the key does in every editor.
            let del = !typing
                && ui.input(|i| {
                    i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace)
                });
            if del {
                if let Some(to) = self.st.wire.take() {
                    self.st.edit(self.cmds, |p| p.disconnect(to));
                } else if let Some(id) = self.st.pick.take() {
                    self.st.edit(self.cmds, |p| p.remove(id));
                    self.st.sel = None;
                }
            }
            if !typing
                && ui.input_mut(|i| {
                    i.consume_key(egui::Modifiers::COMMAND | egui::Modifiers::SHIFT, egui::Key::Z)
                })
            {
                self.st.redo(self.cmds);
            }
            if !typing && ui.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::Z)) {
                self.st.undo(self.cmds);
            }
            // Unwiring first: taking hold of a wire reports both in the same
            // frame when the drag is short, and doing it the other way round
            // would drop the wire that was just drawn.
            if drawn.unlink.is_some() || drawn.unlink_out.is_some() || drawn.link.is_some() {
                let (unlink, unlink_out, link) = (drawn.unlink, drawn.unlink_out, drawn.link);
                self.st.edit(self.cmds, |p| {
                    if let Some(to) = unlink {
                        p.disconnect(to);
                    }
                    if let Some(from) = unlink_out {
                        p.disconnect_from(from);
                    }
                    if let Some((from, to, port)) = link {
                        p.connect(from, (to, port));
                    }
                });
            }
        }
        if let Some((node, param)) = browse {
            self.files.ask(ui.ctx(), node, &param, "Choose a file for this stage");
        }
        if let Some((tag, switched_off)) = off {
            self.st.edit(self.cmds, |p| p.set_off(tag, switched_off));
        }
        if let Some((id, name, value)) = act.changed.or(drawn.changed) {
            self.cmds.push(Cmd::NodeParam(id, name, value));
        }
        if let Some((id, name, value)) = setting {
            self.st.edit(self.cmds, |p| {
                if let Some(s) = p.stage_mut(id) {
                    s.settings.insert(name, value);
                }
            });
        }
        self.st.save_places();
    }

    /// The column beside the graph: what owns its shape, what can be added to
    /// it, and what to do with what is selected.
    ///
    /// A list rather than a menu. Adding a stage is the ordinary thing to do
    /// in here, and a dropdown makes it two clicks and a hidden inventory:
    /// which stages exist at all is worth being able to read.
    fn palette(&mut self, ui: &mut egui::Ui) {
        let mut manual = self.st.edit.manual;
        let help = "Unlocked, stages can be added, moved and wired. Locked, the graph follows \
                    the dial and the scanner table. Edits stay on the graph either way.";
        if egui_bench::form::switch(ui, "graph", &mut manual, "edit", help) {
            self.st.set_manual(manual, self.cmds);
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            // Which direction the pane draws. Transmit is offered only while
            // something is keyed: an empty pane behind a button that did
            // nothing would read as a fault rather than as an idle
            // transmitter.
            let has_tx =
                self.st.topo.as_ref().is_some_and(|t| t.nodes.iter().any(|n| n.kind == "radio_tx"));
            if !has_tx {
                self.st.side = ChainSide::Rx;
            }
            for side in [ChainSide::Rx, ChainSide::Tx] {
                let on = self.st.side == side;
                let enabled = side == ChainSide::Rx || has_tx;
                let w = egui::Button::new(side.label()).fill(if on {
                    theme::READOUT
                } else {
                    theme::WELL
                });
                let r = ui.add_enabled(enabled, w);
                let r = match side {
                    ChainSide::Rx => r.on_hover_text("The chain the receiver is running"),
                    ChainSide::Tx => r.on_hover_text("The chain that is transmitting"),
                };
                if r.clicked() {
                    self.st.side = side;
                    // The selection belongs to the other half of the graph.
                    self.st.sel = None;
                }
            }
        });
        ui.horizontal(|ui| {
            if ui
                .add_enabled(self.st.edit.moved(), egui::Button::new("ARRANGE"))
                .on_hover_text("Lay the stages out again from the graph")
                .clicked()
            {
                self.st.edit.arrange();
            }
            if ui
                .button("FIT")
                .on_hover_text(
                    "Show the whole graph. Ctrl and the wheel zoom, the wheel or a drag pans",
                )
                .clicked()
            {
                self.st.fit = true;
            }
            let picked = self.st.pick.filter(|id| self.st.patch.stage(*id).is_some());
            if ui
                .add_enabled(
                    self.st.edit.manual && (picked.is_some() || self.st.wire.is_some()),
                    egui::Button::new("REMOVE"),
                )
                .on_hover_text("Delete")
                .clicked()
            {
                if let Some(to) = self.st.wire.take() {
                    self.st.edit(self.cmds, |p| p.disconnect(to));
                } else if let Some(id) = picked {
                    self.st.edit(self.cmds, |p| p.remove(id));
                    self.st.pick = None;
                    self.st.sel = None;
                }
            }
        });
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!self.st.undo.is_empty(), egui::Button::new("UNDO"))
                .on_hover_text("Ctrl+Z")
                .clicked()
            {
                self.st.undo(self.cmds);
            }
            if ui
                .add_enabled(!self.st.redo.is_empty(), egui::Button::new("REDO"))
                .on_hover_text("Ctrl+Shift+Z")
                .clicked()
            {
                self.st.redo(self.cmds);
            }
            if ui
                .add_enabled(!self.st.edits.is_empty(), egui::Button::new("CLEAR"))
                .on_hover_text(
                    "Throw away every edit, settings included, and go back to the graph \
                     the receiver draws. UNDO brings them back.",
                )
                .clicked()
            {
                self.st.clear(self.cmds);
            }
        });

        ui.add_space(8.0);
        let hint = if !self.st.edit.manual {
            "locked; adding a stage unlocks it"
        } else if self.st.wire.is_some() {
            "wire selected; DELETE removes it"
        } else {
            "drag a stage onto the graph, a port or a wire"
        };
        text::hint(ui, hint);
        if let Some(why) = &self.st.refusal {
            panel::status(ui, false, why);
        }
        ui.add_space(6.0);
        egui_bench::form::field(ui, &mut self.st.find, "find a stage");
        ui.add_space(6.0);

        // The list comes from the node registry rather than from anything
        // written here, so a decoder added to the build appears in it without
        // this file being touched.
        let reg = crate::chain::registry();
        let find = self.st.find.trim().to_lowercase();
        let mut by_category: Vec<(pipeline::Category, Vec<(&str, &str)>)> = Vec::new();
        for d in reg.list().filter(|d| {
            find.is_empty()
                || d.name.to_lowercase().contains(&find)
                || d.summary.to_lowercase().contains(&find)
        }) {
            match by_category.iter_mut().find(|(c, _)| *c == d.category) {
                Some((_, v)) => v.push((d.name, d.summary)),
                None => by_category.push((d.category, vec![(d.name, d.summary)])),
            }
        }
        let mut add: Option<String> = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            if by_category.is_empty() {
                Line::new().note("No stage matches.").show(ui);
            }
            for (category, stages) in &by_category {
                Line::new().legend(category.label()).show(ui);
                for (name, summary) in stages {
                    let w = egui::Button::new(egui::RichText::new(*name).size(12.0))
                        .fill(theme::WELL)
                        .sense(egui::Sense::click_and_drag())
                        .min_size(egui::Vec2::new(ui.available_width(), 20.0));
                    let r = ui.add(w).on_hover_text(*summary);
                    r.dnd_set_drag_payload(crate::chainview::Carried(name.to_string()));
                    if r.clicked() {
                        add = Some(name.to_string());
                    }
                }
                ui.add_space(6.0);
            }
        });
        if let Some(kind) = add {
            let at = self.st.edit.free_spot();
            self.st.add_stage(self.cmds, &kind, at, crate::chainview::Attach::Nothing);
        }
    }
}

/// One direction of a chain, on its own.
///
/// Which half a stage belongs to is read off what it carries rather than off
/// a list of names kept here, so a modulator or a decoder added later lands
/// on the right side without this being touched: a transmit stream says so
/// on its port, which is what the direction on the spec is for.
///
/// The stage that takes the receiver's clock carries both, and it belongs to
/// the transmit side: it is where that chain begins, and on the receive side
/// it is a stub with nothing after it.
fn one_side(topo: &pipeline::graph::Topology, side: ChainSide) -> pipeline::graph::Topology {
    let mut out = topo.clone();
    out.nodes.retain(|n| {
        let transmits = n.outputs.iter().any(|(_, s)| s.is_tx());
        match side {
            ChainSide::Tx => transmits,
            ChainSide::Rx => !transmits,
        }
    });
    out
}

fn sharpen_text(ui: &egui::Ui, layer: egui::LayerId, from: usize, zoom: f32) {
    let step = (zoom * ZOOM_STEPS).round() / ZOOM_STEPS;
    if step <= 0.0 || step == 1.0 {
        return;
    }
    let texts: Vec<(usize, egui::epaint::TextShape)> = ui.ctx().graphics_mut(|g| {
        let list = g.entry(layer);
        (from..list.next_idx().0)
            .filter_map(|i| {
                let mut found = None;
                list.mutate_shape(egui::layers::ShapeIdx(i), |c| {
                    if let egui::Shape::Text(t) = &c.shape {
                        found = Some(t.clone());
                    }
                });
                found.map(|t| (i, t))
            })
            .collect()
    });
    let painter = ui.painter();
    let sharp: Vec<(usize, egui::Shape)> = texts
        .into_iter()
        .map(|(i, t)| {
            let galley = painter.layout_job(scaled_job(&t.galley.job, step));
            let (pos, underline) = (t.pos, t.underline);
            let mut t = egui::epaint::TextShape { galley, ..t };
            t.transform(egui::emath::TSTransform::from_scaling(1.0 / step));
            t.pos = pos;
            t.underline = underline;
            (i, egui::Shape::Text(t))
        })
        .collect();
    ui.ctx().graphics_mut(|g| {
        let list = g.entry(layer);
        for (i, shape) in sharp {
            list.mutate_shape(egui::layers::ShapeIdx(i), |c| c.shape = shape);
        }
    });
}

fn scaled_job(job: &egui::text::LayoutJob, by: f32) -> egui::text::LayoutJob {
    let mut job = job.clone();
    job.wrap.max_width *= by;
    job.first_row_min_height *= by;
    for s in &mut job.sections {
        s.leading_space *= by;
        s.format.font_id.size *= by;
        s.format.extra_letter_spacing *= by;
        s.format.line_height = s.format.line_height.map(|h| h * by);
        s.format.expand_bg *= by;
    }
    job
}
