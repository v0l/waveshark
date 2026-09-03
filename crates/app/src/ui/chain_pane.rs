//! The signal chain view: the graph the receiver is running, and the editing
//! of it in manual mode.

use super::state::ChainState;
use super::*;

/// The chain view, over the graph the receiver is running and the one the
/// operator has drawn.
pub(super) struct Chain<'a> {
    pub st: &'a mut ChainState,
    pub cmds: &'a mut Vec<Cmd>,
}

impl Chain<'_> {
    /// The signal chain the listening channel is running.
    pub(super) fn show(mut self, ui: &mut egui::Ui) {
        let Some(topo) = self.st.topo.clone() else {
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
        if self.st.sel.is_some_and(|s| !topo.nodes.iter().any(|n| n.id.0 == s)) {
            self.st.sel = None;
        }
        // The inspector takes a column on the right when a stage is selected,
        // rather than floating over the graph: what a stage is set to is read
        // against where it sits in the chain, and a panel covering the chain
        // hides half of that.
        let mut act = crate::chainview::Interaction {
            selected: self.st.sel,
            ..Default::default()
        };
        if self.st.sel.is_some() {
            Panel::right("chain-inspector")
                .default_size(260.0)
                .frame(
                    egui::Frame::NONE
                        .fill(theme::PANEL)
                        .inner_margin(egui::Margin::symmetric(12, 10)),
                )
                .show(ui, |ui| {
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        if let Some(sel) = self.st.sel {
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
            .show(ui, |ui| self.palette(ui));
        // Dragged, not only scrolled: the graph is wider and taller than the
        // pane on any real chain, and reaching for a scrollbar to see a branch
        // is not how anyone reads a diagram. In manual mode a drag moves a
        // stage instead, since dragging is how the graph is edited and the
        // two cannot both own the gesture.
        let manual = self.st.edit.manual;
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
                    self.st.latency,
                    self.st.sel,
                    &mut self.st.edit,
                    Some(&self.st.patch),
                    self.st.wire,
                )
            })
            .inner;
        self.st.sel = drawn.selected;
        if manual {
            if drawn.picked.is_some() {
                self.st.pick = drawn.picked;
                self.st.wire = None;
            }
            if drawn.wire.is_some() {
                self.st.wire = drawn.wire;
            }
            // Delete takes out whichever of the two is selected, which is
            // what the key does in every editor.
            let del = ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace));
            if del {
                if let Some(to) = self.st.wire.take() {
                    self.st.edit(self.cmds, |p| p.disconnect(to));
                } else if let Some(id) = self.st.pick.take() {
                    self.st.edit(self.cmds, |p| p.remove(id));
                    self.st.sel = None;
                }
            }
            if ui.input_mut(|i| {
                i.consume_key(egui::Modifiers::COMMAND, egui::Key::Z)
            }) {
                self.st.undo(self.cmds);
            }
            if ui.input_mut(|i| {
                i.consume_key(egui::Modifiers::COMMAND | egui::Modifiers::SHIFT, egui::Key::Z)
            }) {
                self.st.redo(self.cmds);
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
        if let Some((id, name, value)) = act.changed.or(drawn.changed) {
            self.cmds.push(Cmd::NodeParam(id, name, value));
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
        if ui.checkbox(&mut manual, "MANUAL").clicked() {
            self.st.set_manual(manual, self.cmds);
        }
        ui.add_space(6.0);
        ui.horizontal(|ui| {
            if ui
                .add_enabled(self.st.edit.moved(), egui::Button::new("ARRANGE"))
                .on_hover_text("Lay the stages out again from the graph")
                .clicked()
            {
                self.st.edit.arrange();
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
        });

        ui.add_space(8.0);
        // Which gestures exist, and the one thing that is not editable: only
        // the stages added here have live ports.
        let hint = if !self.st.edit.manual {
            "locked; what is drawn follows the dial and the scanner table"
        } else if self.st.wire.is_some() {
            "wire selected; DELETE removes it"
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
        let manual = self.st.edit.manual;
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
            self.st.edit(self.cmds, |p| added = Some(p.add(&kind)));
            self.st.pick = added;
            self.st.wire = None;
        }
    }

}
