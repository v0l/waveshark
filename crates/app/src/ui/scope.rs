//! The spectrum and the waterfall: the trace, the grid, the marks the
//! scanners and the detectors put on it, and the channel markers over both.

use super::*;

impl App {
    pub(super) fn scope(&mut self, ui: &mut egui::Ui) {
        let mut full = ui.available_rect_before_wrap();
        // A spectrum stage the operator added gets a strip of its own under
        // everything else. They cover a band rather than the span, so they
        // cannot share the main plot's axis, and stacking them is what makes
        // watching a decimated band and the whole span at once worth the
        // stage.
        if !self.extra_spectra.is_empty() {
            let each = (full.height() * 0.22).clamp(60.0, 140.0);
            let n = self.extra_spectra.len().min(3);
            let strips = Rect::from_min_max(
                Pos2::new(full.left(), full.bottom() - each * n as f32),
                full.max,
            );
            full = Rect::from_min_max(full.min, Pos2::new(full.right(), strips.top()));
            let p = ui.painter_at(strips).to_owned();
            for (i, s) in self.extra_spectra.iter().take(n).enumerate() {
                let r = Rect::from_min_size(
                    Pos2::new(strips.left(), strips.top() + each * i as f32),
                    Vec2::new(strips.width(), each),
                );
                self.extra_plot(&p, &r, s);
            }
        }
        let ribbon_h = 16.0;
        let usable = (full.height() - ribbon_h - SPLIT_GRIP_H).max(1.0);
        let plot_h = usable * self.plot_frac.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
        let plot = Rect::from_min_max(full.min, Pos2::new(full.right(), full.top() + plot_h));
        let ribbon = Rect::from_min_max(
            Pos2::new(full.left(), plot.bottom()),
            Pos2::new(full.right(), plot.bottom() + ribbon_h),
        );
        let grip = Rect::from_min_max(
            Pos2::new(full.left(), ribbon.bottom()),
            Pos2::new(full.right(), ribbon.bottom() + SPLIT_GRIP_H),
        );
        let fall = Rect::from_min_max(Pos2::new(full.left(), grip.bottom()), full.max);

        let resp = ui.allocate_rect(full, Sense::click_and_drag());
        let p = ui.painter_at(full).to_owned();
        let plot_cog = cog_rect(&plot);
        let fall_cog = cog_rect(&fall);
        p.rect_filled(plot, 0.0, theme::WELL);

        self.grid(&p, &plot);
        {
            let _s = tracing::info_span!("trace").entered();
            self.trace(&p, &plot);
        }
        // Over the trace, because the trace is filled to the floor of the
        // plot and anything drawn under it there is washed out to the fill's
        // own colour. Kept to four pixels and a low alpha so it reads as a
        // margin note rather than as a signal.
        self.scan_marks(&p, &plot);
        self.source_marks(&p, &plot);
        self.ribbon(&p, &ribbon);

        p.rect_filled(fall, 0.0, theme::CHASSIS);
        {
            let _wf = tracing::info_span!("wf_texture").entered();
            self.wf.draw(ui.ctx(), &p, fall);
        }

        self.markers(&p, &full);

        let hover = resp.hover_pos();
        let grip_hot = self.splitting || hover.is_some_and(|h| grip.contains(h));
        split_grip(&p, &grip, grip_hot);
        let plot_hot = hover.is_some_and(|h| plot_cog.contains(h));
        let fall_hot = hover.is_some_and(|h| fall_cog.contains(h));

        // Reaching for the cog is not reading the spectrum, so the crosshair
        // and its readout get out of the way rather than sitting under the
        // pointer while it is over a button.
        cog(&p, &plot_cog, plot_hot);
        cog(&p, &fall_cog, fall_hot);

        if !plot_hot && !fall_hot {
            let shift = ui.input(|i| i.modifiers.shift);
            self.cursor(&p, &full, &resp, shift);
        }

        if resp.clicked() && self.drag_ch.is_none() {
            if let Some(pos) = resp.interact_pointer_pos() {
                // Cogs sit inside the pane, so they get first refusal on a
                // click; otherwise opening settings would also drop a channel.
                if plot_cog.contains(pos) {
                    self.open = Some(Settings::Spectrum);
                } else if fall_cog.contains(pos) {
                    self.open = Some(Settings::Waterfall);
                } else if grip.contains(pos) {
                    // Dropping a channel on the divider is never what was
                    // meant; a double click there restores the default split.
                } else {
                    // Hit testing uses the true position; only the frequency a
                    // new channel lands on is snapped, so shift-clicking an
                    // existing channel still selects it.
                    match self.channel_at(&full, pos.x) {
                        Some(i) => self.listen(i),
                        None => self.add_channel(self.hz_at_snapped(&full, pos.x, ui)),
                    }
                }
            }
        }
        // Grabbing a channel marker moves that channel; grabbing anywhere else
        // pans the view. Deciding once when the drag starts means the gesture
        // cannot change meaning halfway through as the pointer moves off the
        // line it grabbed.
        if resp.drag_started() {
            // Test where the button went down, not where the pointer is now.
            // egui only reports a drag once the pointer has passed its drag
            // threshold, which is about the same distance as the grab
            // tolerance, so by this point the pointer has already left the
            // marker it grabbed and every drag looked like a pan.
            let origin = ui
                .input(|i| i.pointer.press_origin())
                .or_else(|| resp.interact_pointer_pos());
            self.splitting = origin.is_some_and(|pos| grip.contains(pos));
            self.drag_ch = origin.and_then(|pos| {
                if plot_cog.contains(pos) || fall_cog.contains(pos) || grip.contains(pos) {
                    return None;
                }
                self.channel_at(&full, pos.x)
            });
        }
        if resp.dragged() && self.splitting {
            if let Some(pos) = resp.interact_pointer_pos() {
                // Follow the pointer rather than accumulating deltas, so the
                // divider cannot drift away from the cursor over a long drag.
                let f = (pos.y - full.top() - SPLIT_GRIP_H / 2.0) / usable;
                self.plot_frac =
                    f.clamp(*PLOT_FRAC_RANGE.start(), *PLOT_FRAC_RANGE.end());
            }
        } else if resp.dragged() {
            match self.drag_ch {
                Some(i) if i < self.channels.len() => {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        // Follow the pointer rather than accumulating deltas,
                        // so the marker cannot drift away from the cursor.
                        self.channels[i].freq = self.hz_at_snapped(&full, pos.x, ui);
                        if self.listening == Some(i) {
                            self.listen(i);
                        }
                    }
                }
                _ => {
                    let dx = resp.drag_delta().x as f64;
                    if dx.abs() > 0.0 {
                        self.retune(self.center - dx * self.rate / full.width() as f64);
                    }
                }
            }
        }
        if resp.drag_stopped() {
            self.drag_ch = None;
            self.splitting = false;
        }

        if resp.double_clicked() && hover.is_some_and(|h| grip.contains(h)) {
            self.plot_frac = DEFAULT_PLOT_FRAC;
        }

        // A marker under the pointer is draggable, so say so.
        if grip_hot {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeVertical);
        } else if self.drag_ch.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
        } else if let Some(h) = hover {
            if !plot_hot && !fall_hot && self.channel_at(&full, h.x).is_some() {
                ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
            }
        }

        // Wheel over the pane scrubs the centre frequency. A notch moves a
        // twentieth of the span, so the gesture means the same thing at every
        // zoom level.
        if resp.hovered() && !plot_hot && !fall_hot {
            let n = self.scrub.notches(ui);
            if n != 0 {
                self.retune(self.center - n as f64 * self.rate / 20.0);
            }
        }
    }

    /// Where the scanner table is listening, along the foot of the spectrum.
    ///
    /// A bank has hundreds of channels and drawing a line per channel is a
    /// grey wash that hides the spectrum it is describing, so the band is a
    /// strip and the channel grid appears as ticks only when the ticks are far
    /// enough apart to be counted. Below that the strip alone is the honest
    /// drawing: the channels are narrower than a pixel.
    /// The transmitters the detector has open right now, drawn over the
    /// trace where they are and as wide as they were measured: the moment a
    /// sensor keys up it appears here, and the moment it stops it is gone.
    fn source_marks(&self, p: &egui::Painter, plot: &Rect) {
        if !self.decode_on {
            return;
        }
        let Some(r) = &self.radio else { return };
        let seen = r.status.sources.lock().clone();
        if seen.is_empty() {
            return;
        }
        let col = theme::READOUT;
        let font = FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into()));
        // Left to right, so a label that would land on the one before it
        // can take the next row down instead. Two sensors a few kilohertz
        // apart are two sources, and printed on one row they were one
        // unreadable smear.
        let mut seen = seen;
        seen.sort_by(|a, b| a.source.center_hz.partial_cmp(&b.source.center_hz).unwrap());
        let now = std::time::Instant::now();
        let mut rows: Vec<f32> = Vec::new();
        for e in &seen {
            let s = &e.source;
            // A source still open is drawn full; one that closed fades over
            // the seconds it lingers, so a burst leaves a mark that can be
            // read and then gets out of the way.
            let age = now.duration_since(e.last_seen).as_secs_f32();
            let fade = if e.live {
                1.0
            } else {
                (1.0 - age / crate::radio::SOURCE_LINGER.as_secs_f32()).clamp(0.0, 1.0)
            };
            let dim = |a: u8| {
                Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), (a as f32 * fade) as u8)
            };
            let x0 = self.x_of(plot, s.center_hz - s.bandwidth_hz / 2.0);
            let x1 = self.x_of(plot, s.center_hz + s.bandwidth_hz / 2.0);
            let (cx0, cx1) = (x0.max(plot.left()), x1.min(plot.right()));
            if cx1 < plot.left() || cx0 > plot.right() {
                continue;
            }
            // At least two pixels, or a narrow sensor on a wide span vanishes.
            let (cx0, cx1) = if cx1 - cx0 < 2.0 { (cx0 - 1.0, cx0 + 1.0) } else { (cx0, cx1) };
            let label = if s.bandwidth_hz >= 1e6 {
                format!("{:.4} MHz  {:.0} kHz  {:.0} dB", s.center_hz / 1e6, s.bandwidth_hz / 1e3, s.snr_db)
            } else {
                format!("{:.4} MHz  {:.1} kHz  {:.0} dB", s.center_hz / 1e6, s.bandwidth_hz / 1e3, s.snr_db)
            };
            let width = label.len() as f32 * 5.6 + 6.0;
            let row = rows.iter().position(|end| *end < cx0).unwrap_or(rows.len());
            if row == rows.len() {
                rows.push(0.0);
            }
            rows[row] = cx0 + width;
            let y0 = plot.top() + 4.0 + row as f32 * 22.0;
            let y1 = y0 + 10.0;
            p.rect_filled(
                Rect::from_min_max(Pos2::new(cx0, y0), Pos2::new(cx1, y1)),
                1.0,
                dim(90),
            );
            p.rect_stroke(
                Rect::from_min_max(Pos2::new(cx0, y0), Pos2::new(cx1, y1)),
                1.0,
                Stroke::new(1.0, dim(200)),
                egui::StrokeKind::Outside,
            );
            p.text(Pos2::new(cx0, y1 + 2.0), Align2::LEFT_TOP, label, font.clone(), dim(220));
        }
    }

    fn scan_marks(&self, p: &egui::Painter, plot: &Rect) {
        if !self.decode_on {
            return;
        }
        let marks = crate::chain::scan_marks(&self.scanners, self.center, self.rate);
        if marks.is_empty() {
            return;
        }
        let col = theme::OK;
        let dim = |a: u8| Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), a);
        // Read against the trace's own fill, which is what is behind this.
        let (fill_a, tick_a, text_a) = (110u8, 190u8, 220u8);
        // A strip at the foot of the plot, clear of the trace's baseline.
        let floor = plot.bottom() - 1.0;
        let font = FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into()));
        // Banks stack upward. Two of them cover the same band at different
        // channel widths, and drawn on one row they were one strip with two
        // labels printed over each other.
        let mut row = 0usize;

        for m in &marks {
            match m {
                crate::chain::ScanMark::Band { lo, hi, origin, spacing, label } => {
                    let y1 = floor - row as f32 * 6.0;
                    let y0 = y1 - 4.0;
                    row += 1;
                    let (x0, x1) = (self.x_of(plot, *lo), self.x_of(plot, *hi));
                    let (cx0, cx1) = (x0.max(plot.left()), x1.min(plot.right()));
                    if cx1 - cx0 < 1.0 {
                        continue;
                    }
                    p.rect_filled(
                        Rect::from_min_max(Pos2::new(cx0, y0), Pos2::new(cx1, y1)),
                        1.0,
                        dim(fill_a),
                    );
                    let step_px = (x1 - x0) * (*spacing as f32) / (*hi - *lo).max(1.0) as f32;
                    if step_px >= 7.0 {
                        // Stepped from a real channel centre rather than from
                        // the band edge, which is where the grid happens to be
                        // cut. Half a channel of error in a drawing of where
                        // the channels are is the whole of what it says.
                        let first =
                            ((lo - origin) / spacing).ceil() * spacing + origin - spacing / 2.0;
                        let mut hz = first;
                        while hz <= *hi + spacing {
                            let x = self.x_of(plot, hz);
                            if plot.x_range().contains(x) && x >= x0 && x <= x1 {
                                p.line_segment(
                                    [Pos2::new(x, y0 - 2.0), Pos2::new(x, y1)],
                                    Stroke::new(1.0, dim(tick_a)),
                                );
                            }
                            hz += spacing;
                        }
                    }
                    // Named at the left edge of its own strip, where it cannot
                    // be mistaken for a label on the band beside it.
                    if cx1 - cx0 > 46.0 {
                        p.text(
                            Pos2::new(cx0 + 3.0, y0 - 3.0),
                            Align2::LEFT_BOTTOM,
                            label,
                            font.clone(),
                            dim(text_a),
                        );
                    }
                }
                crate::chain::ScanMark::Channel { hz, width, label } => {
                    let (y1, y0) = (floor, floor - 4.0);
                    let x = self.x_of(plot, *hz);
                    if !plot.x_range().contains(x) {
                        continue;
                    }
                    let (x0, x1) =
                        (self.x_of(plot, hz - width / 2.0), self.x_of(plot, hz + width / 2.0));
                    if x1 - x0 >= 2.0 {
                        p.rect_filled(
                            Rect::from_min_max(
                                Pos2::new(x0.max(plot.left()), y0),
                                Pos2::new(x1.min(plot.right()), y1),
                            ),
                            1.0,
                            dim(fill_a),
                        );
                    }
                    p.line_segment(
                        [Pos2::new(x, y1 - 9.0), Pos2::new(x, y1)],
                        Stroke::new(1.0, dim(tick_a)),
                    );
                    p.text(
                        Pos2::new(x + 3.0, y1 - 9.0),
                        Align2::LEFT_BOTTOM,
                        label,
                        font.clone(),
                        dim(text_a),
                    );
                }
            }
        }
    }

    fn grid(&self, p: &egui::Painter, plot: &Rect) {
        for i in 0..=10 {
            let x = plot.left() + plot.width() * i as f32 / 10.0;
            p.line_segment(
                [Pos2::new(x, plot.top()), Pos2::new(x, plot.bottom())],
                Stroke::new(1.0, Color32::from_rgb(0x24, 0x28, 0x2E)),
            );
        }
        // Amplitude graticule, labelled in dBFS so the numbers mean something.
        for i in 1..4 {
            let y = plot.top() + plot.height() * i as f32 / 4.0;
            p.line_segment(
                [Pos2::new(plot.left(), y), Pos2::new(plot.right(), y)],
                Stroke::new(1.0, Color32::from_rgb(0x22, 0x26, 0x2B)),
            );
            let db = self.ceil - (self.ceil - self.floor) * i as f32 / 4.0;
            // The unit once, on the top line. Repeating it on every gridline
            // is three times the ink for the same fact.
            let text = if i == 1 { format!("{db:.0} dBFS") } else { format!("{db:.0}") };
            // Set at 11 rather than 9, and in the panel's legend grey rather
            // than a shade above the background. These are the numbers that
            // say what the trace is worth, and they were unreadable.
            let font = FontId::new(11.0, FontFamily::Name(theme::LEGEND_FONT.into()));
            let at = Pos2::new(plot.right() - 5.0, y - 1.0);
            let galley = p.layout_no_wrap(text, font, theme::LEGEND);
            let rect = Align2::RIGHT_BOTTOM.anchor_size(at, galley.size());
            // A backing, because the trace runs behind these and a number
            // crossed by a carrier is not a number any more.
            p.rect_filled(
                rect.expand2(Vec2::new(3.0, 1.0)),
                2.0,
                Color32::from_rgba_unmultiplied(0x14, 0x16, 0x19, 190),
            );
            p.galley(rect.min, galley, theme::LEGEND);
        }
    }

    /// Which bins of the held spectrum belong under screen column `c`.
    ///
    /// `None` where the column falls outside the data, which happens while a
    /// retune is pending and the view has moved past what has been received.
    pub(super) fn column_bins(
        &self,
        plot: &Rect,
        c: usize,
        _cols: usize,
        n: usize,
    ) -> Option<(usize, usize)> {
        let lo = self.db_center - self.rate / 2.0;
        let bin = |f: f64| ((f - lo) / self.rate * n as f64).floor();
        let a = bin(self.hz_at(plot, plot.left() + c as f32));
        let b = bin(self.hz_at(plot, plot.left() + c as f32 + 1.0));
        if a < 0.0 || a >= n as f64 {
            return None;
        }
        let a = a as usize;
        Some((a, (b.clamp(0.0, n as f64) as usize).max(a + 1).min(n)))
    }

    /// One extra spectrum, with its own band under it.
    ///
    /// Drawn from its own centre and rate rather than the dial's: what is
    /// wired into it decides what it covers, and that is the whole point of
    /// having more than one.
    fn extra_plot(&self, p: &egui::Painter, r: &Rect, s: &crate::radio::Spectrum) {
        p.rect_filled(*r, 0.0, theme::WELL);
        p.line_segment(
            [r.left_top(), r.right_top()],
            Stroke::new(1.0, theme::ETCH),
        );
        let plot = Rect::from_min_max(Pos2::new(r.left(), r.top() + 12.0), r.max);
        let span = (self.ceil - self.floor).max(1.0);
        let n = s.db.len();
        if n >= 2 {
            let cols = plot.width().max(1.0) as usize;
            let mut pts = Vec::with_capacity(cols);
            for c in 0..cols {
                let a = c * n / cols.max(1);
                let b = (((c + 1) * n) / cols.max(1)).max(a + 1).min(n);
                let v = s.db[a..b].iter().copied().fold(f32::MIN, f32::max);
                let t = ((v - self.floor) / span).clamp(0.0, 1.0);
                pts.push(Pos2::new(plot.left() + c as f32, plot.bottom() - t * plot.height()));
            }
            p.add(egui::Shape::line(pts, Stroke::new(1.0, theme::TRACE)));
        }
        let name = self
            .chain_patch
            .stage(s.tag)
            .map(|st| st.kind.clone())
            .unwrap_or_else(|| "spectrum".into());
        p.text(
            Pos2::new(r.left() + 6.0, r.top() + 1.0),
            egui::Align2::LEFT_TOP,
            format!(
                "{name}   {:.4} MHz   {:.3} MS/s",
                s.center / 1e6,
                s.rate / 1e6
            ),
            FontId::new(10.0, FontFamily::Name(theme::READOUT_FONT.into())),
            theme::LEGEND,
        );
    }

    fn trace(&self, p: &egui::Painter, plot: &Rect) {
        if self.db.is_empty() {
            return;
        }
        let span = (self.ceil - self.floor).max(1.0);
        let n = self.db.len();
        let cols = plot.width().max(1.0) as usize;
        // Columns are placed by frequency rather than by bin index. While a
        // retune is pending the held spectrum belongs to a different centre,
        // and drawing it stretched across the pane would put every signal at
        // the wrong frequency. Positioning it by its own centre slides it under
        // the drag instead, which is where its data really is.
        let mut pts = Vec::with_capacity(cols);
        for c in 0..cols {
            let Some((a, b)) = self.column_bins(plot, c, cols, n) else { continue };
            // Max, not mean: averaging hides the narrow carriers that matter.
            let v = self.db[a..b].iter().copied().fold(f32::MIN, f32::max);
            let t = ((v - self.floor) / span).clamp(0.0, 1.0);
            pts.push(Pos2::new(plot.left() + c as f32, plot.bottom() - t * plot.height()));
        }
        if pts.len() < 2 {
            return;
        }
        // Fill under the trace so occupied spectrum reads as mass. Built as a
        // quad strip: a spectrum outline is wildly concave, and asking for a
        // convex polygon fill turns it into fan-shaped wedges.
        let mut mesh = egui::Mesh::default();
        let fill = Color32::from_rgba_unmultiplied(0x5C, 0xD0, 0xE8, 26);
        for w in pts.windows(2) {
            let i = mesh.vertices.len() as u32;
            for v in [
                w[0],
                w[1],
                Pos2::new(w[1].x, plot.bottom()),
                Pos2::new(w[0].x, plot.bottom()),
            ] {
                mesh.colored_vertex(v, fill);
            }
            mesh.add_triangle(i, i + 1, i + 2);
            mesh.add_triangle(i, i + 2, i + 3);
        }
        p.add(egui::Shape::mesh(mesh));
        p.add(egui::Shape::line(pts, Stroke::new(1.2, theme::TRACE)));
    }

    fn ribbon(&self, p: &egui::Painter, r: &Rect) {
        p.rect_filled(*r, 0.0, theme::CHASSIS);
        let (lo, hi) = (self.center - self.rate / 2.0, self.center + self.rate / 2.0);
        for b in bands::in_span(lo, hi) {
            let x0 = self.x_of(r, b.lo.max(lo)).max(r.left());
            let x1 = self.x_of(r, b.hi.min(hi)).min(r.right());
            if x1 - x0 < 1.0 {
                continue;
            }
            let cell = Rect::from_min_max(Pos2::new(x0, r.top() + 2.0), Pos2::new(x1, r.bottom() - 2.0));
            p.rect_filled(cell, 1.0, b.color);
            if x1 - x0 > 60.0 {
                p.text(
                    cell.center(),
                    Align2::CENTER_CENTER,
                    b.name,
                    FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into())),
                    Color32::from_rgb(0xE8, 0xEC, 0xF0),
                );
            }
        }
    }

    fn markers(&self, p: &egui::Painter, full: &Rect) {
        let (lo, hi) = (self.center - self.rate / 2.0, self.center + self.rate / 2.0);
        for (i, ch) in self.channels.iter().enumerate() {
            if ch.freq < lo || ch.freq > hi {
                continue;
            }
            let x = self.x_of(full, ch.freq);
            let active = self.listening == Some(i);
            let col = if active { theme::READOUT } else { Color32::from_rgb(0x6E, 0x7A, 0x88) };

            // Show what the demodulator actually takes in, not just where it
            // is centred: an NFM channel and a WFM channel at the same spot
            // are wildly different slices of spectrum.
            let half = ch.demod.bandwidth() / 2.0;
            let (bx0, bx1) = (self.x_of(full, ch.freq - half), self.x_of(full, ch.freq + half));
            if bx1 - bx0 >= 1.0 {
                let band = Rect::from_min_max(
                    Pos2::new(bx0.max(full.left()), full.top()),
                    Pos2::new(bx1.min(full.right()), full.bottom()),
                );
                p.rect_filled(
                    band,
                    0.0,
                    Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), if active { 34 } else { 18 }),
                );
                for ex in [bx0, bx1] {
                    if full.x_range().contains(ex) {
                        p.line_segment(
                            [Pos2::new(ex, full.top()), Pos2::new(ex, full.bottom())],
                            Stroke::new(1.0, Color32::from_rgba_unmultiplied(col.r(), col.g(), col.b(), 120)),
                        );
                    }
                }
            }
            p.line_segment(
                [Pos2::new(x, full.top()), Pos2::new(x, full.bottom())],
                Stroke::new(if active { 1.5 } else { 1.0 }, col),
            );
            // Flag the label off the line so it never sits on the trace.
            let t = p.layout_no_wrap(
                ch.label.clone(),
                FontId::new(10.0, FontFamily::Name(theme::LEGEND_FONT.into())),
                Color32::BLACK,
            );
            let flag = Rect::from_min_size(
                Pos2::new(x + 1.0, full.top() + 2.0),
                Vec2::new(t.size().x + 8.0, t.size().y + 4.0),
            );
            p.rect_filled(flag, 1.0, col);
            p.galley(Pos2::new(flag.left() + 4.0, flag.top() + 2.0), t, Color32::BLACK);
        }
    }

    fn cursor(&self, p: &egui::Painter, full: &Rect, resp: &egui::Response, shift: bool) {
        let Some(pos) = resp.hover_pos() else { return };
        let raw = self.hz_at(full, pos.x);
        let raster = bands::raster_at(raw);
        // With shift held the readout shows where a channel would actually
        // land, not where the pointer is, so the snap is visible before
        // committing to it.
        let hz = if shift { bands::snap(raw) } else { raw };
        p.line_segment(
            [Pos2::new(pos.x, full.top()), Pos2::new(pos.x, full.bottom())],
            Stroke::new(1.0, Color32::from_rgb(0x55, 0x5E, 0x69)),
        );
        if shift {
            // Mark the snapped frequency when it differs from the pointer.
            let sx = self.x_of(full, hz);
            if (sx - pos.x).abs() > 1.0 && full.x_range().contains(sx) {
                p.line_segment(
                    [Pos2::new(sx, full.top()), Pos2::new(sx, full.bottom())],
                    Stroke::new(1.0, theme::READOUT),
                );
            }
        }
        let text = match (shift, raster) {
            (true, Some(r)) => {
                format!("{} {} snap {}", fmt_hz(hz), bands::name_at(hz), fmt_hz(r.step))
            }
            (true, None) => format!("{} {} no channel plan", fmt_hz(hz), bands::name_at(hz)),
            _ => match raster {
                // Advertise the gesture only where it would do something.
                Some(_) => format!("{} {} shift to snap", fmt_hz(hz), bands::name_at(hz)),
                None => format!("{} {}", fmt_hz(hz), bands::name_at(hz)),
            },
        };
        let g = p.layout_no_wrap(
            text,
            FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())),
            theme::VALUE,
        );
        let left = (pos.x + 8.0).min(full.right() - g.size().x - 10.0);
        let box_r = Rect::from_min_size(
            Pos2::new(left - 5.0, full.top() + 5.0),
            g.size() + Vec2::new(10.0, 6.0),
        );
        p.rect(box_r, 2.0, theme::WELL, Stroke::new(1.0, theme::ETCH), StrokeKind::Inside);
        p.galley(Pos2::new(left, full.top() + 8.0), g, theme::VALUE);
    }
}
