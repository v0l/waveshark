//! The signal chain view: the graph the receiver is running, and the editing
//! of it in manual mode.

use super::*;

impl App {
    /// The signal chain the listening channel is running.
    pub(super) fn chain(&mut self, ui: &mut egui::Ui) {
        let Some(topo) = self.chain_topo.clone() else {
            ui.centered_and_justified(|ui| {
                ui.label(
                    egui::RichText::new("The radio is stopped, so no chain is running.")
                        .color(theme::LEGEND),
                );
            });
            return;
        };
        // Node ids are positions in the built graph, so a rebuild can leave
        // the selection pointing at a stage that is no longer there.
        if self.chain_sel.is_some_and(|s| !topo.nodes.iter().any(|n| n.id.0 == s)) {
            self.chain_sel = None;
        }
        // The inspector takes a column on the right when a stage is selected,
        // rather than floating over the graph: what a stage is set to is read
        // against where it sits in the chain, and a panel covering the chain
        // hides half of that.
        let mut act = crate::chainview::Interaction {
            selected: self.chain_sel,
            ..Default::default()
        };
        if self.chain_sel.is_some() {
            Panel::right("chain-inspector")
                .default_size(260.0)
                .frame(
                    egui::Frame::NONE
                        .fill(theme::PANEL)
                        .inner_margin(egui::Margin::symmetric(12, 10)),
                )
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        if let Some(sel) = self.chain_sel {
                            act.changed = crate::chainview::inspector(ui, &topo, sel);
                        }
                    });
                });
        }
        Panel::left("chain-palette")
            .default_size(190.0)
            .frame(
                egui::Frame::NONE
                    .fill(theme::PANEL)
                    .inner_margin(egui::Margin::symmetric(10, 10)),
            )
            .show(ui, |ui| self.chain_palette(ui));
        // Dragged, not only scrolled: the graph is wider and taller than the
        // pane on any real chain, and reaching for a scrollbar to see a branch
        // is not how anyone reads a diagram. In manual mode a drag moves a
        // stage instead, since dragging is how the graph is edited and the
        // two cannot both own the gesture.
        let manual = self.chain_edit.manual;
        let drawn = egui::ScrollArea::both()
            .scroll_source(egui::containers::scroll_area::ScrollSource {
                drag: if manual {
                    egui::containers::scroll_area::DragScroll::Never
                } else {
                    egui::containers::scroll_area::DragScroll::Always
                },
                ..Default::default()
            })
            .show(ui, |ui| {
                crate::chainview::draw(
                    ui,
                    &topo,
                    self.chain_latency,
                    self.chain_sel,
                    &mut self.chain_edit,
                    Some(&self.chain_patch),
                    self.chain_wire,
                )
            })
            .inner;
        self.chain_sel = drawn.selected;
        if manual {
            if drawn.picked.is_some() {
                self.chain_pick = drawn.picked;
                self.chain_wire = None;
            }
            if drawn.wire.is_some() {
                self.chain_wire = drawn.wire;
            }
            // Delete takes out whichever of the two is selected, which is
            // what the key does in every editor.
            let del = ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace));
            if del {
                if let Some(to) = self.chain_wire.take() {
                    self.edit_patch(|p| p.disconnect(to));
                } else if let Some(id) = self.chain_pick.take() {
                    self.edit_patch(|p| p.remove(id));
                    self.chain_sel = None;
                }
            }
            if ui.input_mut(|i| {
                i.consume_key(egui::Modifiers::COMMAND, egui::Key::Z)
            }) {
                self.undo_patch();
            }
            if ui.input_mut(|i| {
                i.consume_key(egui::Modifiers::COMMAND | egui::Modifiers::SHIFT, egui::Key::Z)
            }) {
                self.redo_patch();
            }
            // Unwiring first: taking hold of a wire reports both in the same
            // frame when the drag is short, and doing it the other way round
            // would drop the wire that was just drawn.
            if drawn.unlink.is_some() || drawn.unlink_out.is_some() || drawn.link.is_some() {
                let (unlink, unlink_out, link) = (drawn.unlink, drawn.unlink_out, drawn.link);
                self.edit_patch(|p| {
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
        if let Some((id, name, value)) = act.changed.or(drawn.changed) {
            self.send(Cmd::NodeParam(id, name, value));
        }
        self.save_places();
    }

    /// The column beside the graph: what owns its shape, what can be added to
    /// it, and what to do with what is selected.
    ///
    /// A list rather than a menu. Adding a stage is the ordinary thing to do
    /// in here, and a dropdown makes it two clicks and a hidden inventory:
    /// which stages exist at all is worth being able to read.
    fn chain_palette(&mut self, ui: &mut egui::Ui) {
        let mut manual = self.chain_edit.manual;
        if ui.checkbox(&mut manual, "MANUAL").clicked() {
            self.set_manual_chain(manual);
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(self.chain_edit.moved(), egui::Button::new("ARRANGE"))
                .on_hover_text("Lay the stages out again from the graph")
                .clicked()
            {
                self.chain_edit.arrange();
            }
            let picked = self.chain_pick.filter(|id| self.chain_patch.stage(*id).is_some());
            if ui
                .add_enabled(
                    self.chain_edit.manual && (picked.is_some() || self.chain_wire.is_some()),
                    egui::Button::new("REMOVE"),
                )
                .on_hover_text("Delete")
                .clicked()
            {
                if let Some(to) = self.chain_wire.take() {
                    self.edit_patch(|p| p.disconnect(to));
                } else if let Some(id) = picked {
                    self.edit_patch(|p| p.remove(id));
                    self.chain_pick = None;
                    self.chain_sel = None;
                }
            }
        });
        ui.horizontal(|ui| {
            if ui
                .add_enabled(!self.chain_undo.is_empty(), egui::Button::new("UNDO"))
                .on_hover_text("Ctrl+Z")
                .clicked()
            {
                self.undo_patch();
            }
            if ui
                .add_enabled(!self.chain_redo.is_empty(), egui::Button::new("REDO"))
                .on_hover_text("Ctrl+Shift+Z")
                .clicked()
            {
                self.redo_patch();
            }
        });

        ui.add_space(8.0);
        // Which gestures exist, and the one thing that is not editable: only
        // the stages added here have live ports.
        let hint = if !self.chain_edit.manual {
            "built from the scanner table for this span"
        } else if self.chain_wire.is_some() {
            "wire selected; DELETE removes it"
        } else if self.chain_patch.stages().is_empty() {
            "add a stage: only stages added here can be wired"
        } else {
            "drag a port to wire, drag a wire off an input to move it"
        };
        ui.label(legend(hint));
        ui.add_space(8.0);
        ui.separator();
        ui.add_space(6.0);

        // The list comes from the node registry rather than from anything
        // written here, so a decoder added to the build appears in it without
        // this file being touched.
        let reg = crate::chain::registry();
        let mut by_category: Vec<(&str, Vec<(&str, &str)>)> = Vec::new();
        for d in reg.list() {
            match by_category.iter_mut().find(|(c, _)| *c == d.category) {
                Some((_, v)) => v.push((d.name, d.summary)),
                None => by_category.push((d.category, vec![(d.name, d.summary)])),
            }
        }
        let manual = self.chain_edit.manual;
        let mut add: Option<String> = None;
        egui::ScrollArea::vertical().show(ui, |ui| {
            for (category, stages) in &by_category {
                ui.label(legend(category));
                for (name, summary) in stages {
                    let w = egui::Button::new(egui::RichText::new(*name).size(12.0))
                        .fill(theme::WELL)
                        .min_size(egui::Vec2::new(ui.available_width(), 20.0));
                    if ui.add_enabled(manual, w).on_hover_text(*summary).clicked() {
                        add = Some(name.to_string());
                    }
                }
                ui.add_space(6.0);
            }
        });
        if let Some(kind) = add {
            let mut added = None;
            self.edit_patch(|p| added = Some(p.add(&kind)));
            self.chain_pick = added;
            self.chain_wire = None;
        }
    }

    /// Change the graph, keeping what it was so the change can be taken back.
    fn edit_patch(&mut self, f: impl FnOnce(&mut crate::patch::Patch)) {
        let before = self.chain_patch.clone();
        f(&mut self.chain_patch);
        if self.chain_patch == before {
            return;
        }
        self.chain_undo.push(before);
        // Undoing and then drawing something else abandons what was undone,
        // which is what makes redo mean anything: a branch nobody can reach
        // is a trap rather than a history.
        self.chain_redo.clear();
        // A hundred edits is more than anybody backs out of in one sitting
        // and small enough to keep in hand: a patch is a few dozen stages.
        if self.chain_undo.len() > 100 {
            self.chain_undo.remove(0);
        }
        self.send_patch();
    }

    fn undo_patch(&mut self) {
        if let Some(was) = self.chain_undo.pop() {
            self.chain_redo.push(std::mem::replace(&mut self.chain_patch, was));
            self.chain_wire = None;
            self.send_patch();
        }
    }

    fn redo_patch(&mut self) {
        if let Some(next) = self.chain_redo.pop() {
            self.chain_undo.push(std::mem::replace(&mut self.chain_patch, next));
            self.chain_wire = None;
            self.send_patch();
        }
    }

    /// Hand the patch to the radio thread, remembering what was sent so that
    /// one handed back after a refusal can be told apart from an echo.
    fn send_patch(&mut self) {
        self.chain_drawn = Some(self.chain_patch.clone());
        self.chain_patch_sent = Some(self.chain_patch.clone());
        self.send(Cmd::Patch(self.chain_patch.clone()));
        self.save_patch();
    }

    /// Write the graph out, with where its stages were put.
    fn save_patch(&mut self) {
        self.chain_places =
            self.chain_edit.pos.iter().map(|(k, p)| (*k, (p.x, p.y))).collect();
        self.chain_patch.save(&self.chain_places);
        self.chain_saved_at = Some(std::time::Instant::now());
    }

    /// Write it out again when a stage has been moved and the pointer has
    /// settled. Dragging changes a position on every frame, and a file
    /// written sixty times a second to record where a box ended up is a lot
    /// of writes for one arrangement.
    fn save_places(&mut self) {
        if !self.chain_edit.manual {
            return;
        }
        let now: crate::patch::Places =
            self.chain_edit.pos.iter().map(|(k, p)| (*k, (p.x, p.y))).collect();
        if now == self.chain_places {
            return;
        }
        let due = self.chain_saved_at.is_none_or(|t| t.elapsed().as_secs_f32() >= 2.0);
        if due {
            self.save_patch();
        }
    }

    /// Hand the shape of the graph to the operator, or give it back to the
    /// scanner table.
    pub fn set_manual_chain(&mut self, on: bool) {
        self.chain_edit.manual = on;
        if !on {
            self.chain_edit.arrange();
            self.chain_pick = None;
        }
        match (on, self.chain_drawn.clone()) {
            // Back to the graph that was drawn, which is what a saved
            // drawing is for. Turning manual mode off and on again is not a
            // request to throw it away.
            (true, Some(drawn)) => {
                self.chain_patch = drawn;
                self.send(Cmd::Manual(true));
                self.send_patch();
            }
            // Nothing drawn yet, so the radio thread answers with the graph
            // it is running: taking it over means taking that over.
            _ => {
                self.chain_patch_sent = None;
                self.send(Cmd::Manual(on));
            }
        }
    }
}
