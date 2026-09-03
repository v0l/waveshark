//! Drawing one decoded packet: its levels, its envelope and its bytes.
//!
//! Free functions rather than widgets: each draws into the `Ui` it is given
//! and keeps nothing between frames.

use super::*;

/// A level in dB, or blank when the decoder did not measure one. Blank rather
/// than a zero: a missing measurement and a strong signal must not look alike.
pub(super) fn fmt_db(v: f32) -> String {
    if v.is_finite() {
        format!("{v:6.1}")
    } else {
        " -".into()
    }
}

/// Amber when a packet is loud enough to be clipping the front end, which is
/// worth seeing: a decode can fail from too much gain as easily as too little.
pub(super) fn level_color(rssi_dbfs: f32) -> Color32 {
    if rssi_dbfs > -3.0 {
        theme::FAULT
    } else if rssi_dbfs > -12.0 {
        theme::READOUT
    } else {
        theme::LEGEND
    }
}

/// Fixed-width text, so columns of numbers line up and a hex dump reads as one.
pub(super) fn mono(text: &str, col: Color32) -> egui::RichText {
    egui::RichText::new(text)
        .font(FontId::new(11.0, FontFamily::Name(theme::READOUT_FONT.into())))
        .color(col)
}

/// Green for a verified packet, amber for one with no check to verify, red for
/// a failed one, grey for a burst nothing claimed. The same colours are used
/// on the waterfall.
pub(super) fn row_color(rec: &DecodeRecord) -> Color32 {
    if !rec.is_known() {
        return theme::LEGEND;
    }
    match rec.crc {
        Some(true) => CRC_OK,
        Some(false) => theme::FAULT,
        None => theme::READOUT,
    }
}

/// What a selected packet holds: its fields, then its bytes.
///
/// The fields come first because they are the answer; the bytes are there for
/// when the answer is wrong, or when the protocol is unknown and the bytes are
/// all there is. Both are also what a view widget would consume: a map reads
/// the fields, an image pane reads the bytes and the media type.
/// The detail pane under the packet list.
///
/// Returns whether the operator asked to hear the transmission again, which
/// the caller turns into a command: the audio device belongs to the radio
/// thread, and a view does not get to open its own.
pub(super) fn packet_detail(ui: &mut egui::Ui, rec: &DecodeRecord) -> bool {
    // The burst view takes up to half the room the inspector was dragged
    // to, never less than its natural height, so dragging the divider up
    // grows the RF view and the bytes together rather than only the
    // scrollback under them. A packet without samples gets the same area,
    // blank, so the bytes sit where they did for the last packet.
    let h = (ui.available_height() * 0.5).clamp(BURST_VIEW_H, 320.0);
    match &rec.iq {
        Some(iq) => burst_view(ui, iq, h),
        None => {
            ui.label(legend("burst  no samples kept for this packet"));
            let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width().max(200.0), h), Sense::hover());
            ui.painter().rect_filled(rect, 2.0, theme::WELL);
        }
    }
    ui.add_space(4.0);
    // A voice transmission's payload is what was said, so the row offers to
    // say it again. The bytes below are the vocoder's, and nobody reads those.
    let mut play = false;
    if let Some(a) = &rec.audio {
        let (peak, rms) = crate::callbus::levels_db(a);
        ui.horizontal(|ui| {
            play = ui.button("PLAY").clicked();
            ui.label(legend(&format!(
                "{:.1} s of speech   peak {peak:.0} dBFS   rms {rms:.0} dBFS",
                a.seconds()
            )));
        });
        ui.add_space(4.0);
    }
    if !rec.fields.is_empty() {
        ui.horizontal_wrapped(|ui| {
            ui.spacing_mut().item_spacing.x = 14.0;
            for (k, v) in &rec.fields {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 4.0;
                    ui.label(legend(k));
                    ui.label(mono(&v.to_string(), theme::VALUE));
                });
            }
        });
        ui.add_space(4.0);
    }
    hex_dump(ui, &rec.bytes);
    play
}

/// One column of the burst view: the loudest sample in the column, and the
/// mean instantaneous frequency across it, in hertz.
///
/// A burst is thousands of samples and the view a few hundred pixels wide,
/// so each column stands for a run of samples. The envelope keeps the peak
/// of the run, since a mark a few samples long has to stay visible, and the
/// frequency is the mean phase step across it, which for a keyed carrier
/// sits at its offset during a mark and wanders during a gap.
pub(super) fn burst_columns(samples: &[common::C32], rate: f64, cols: usize) -> Vec<(f32, f32)> {
    let cols = cols.max(1);
    let n = samples.len();
    (0..cols)
        .map(|c| {
            let a = c * n / cols;
            let b = ((c + 1) * n / cols).max(a + 1).min(n);
            let env = samples[a..b].iter().map(|x| x.norm()).fold(0.0f32, f32::max);
            let mut acc = common::C32::new(0.0, 0.0);
            for i in a.max(1)..b {
                acc += samples[i] * samples[i - 1].conj();
            }
            let hz = if acc.norm_sqr() > 0.0 {
                acc.arg() / std::f32::consts::TAU * rate as f32
            } else {
                0.0
            };
            (env, hz)
        })
        .collect()
}

/// The burst as the front end saw it: its envelope filled from the floor,
/// and its instantaneous frequency drawn over it, against time. What
/// Universal Radio Hacker shows beside a burst's bits, and the view an
/// unknown device is worked out from: a keyed carrier's marks and gaps, a
/// two-tone signal's frequency stepping between its tones, a chirp's
/// frequency ramping across the width.
pub(super) fn burst_view(ui: &mut egui::Ui, iq: &common::IqBurst, height: f32) {
    let secs = iq.samples.len() as f64 / iq.rate.max(1.0);
    let half_span = iq.rate / 2.0;
    ui.label(legend(&format!(
        "burst  {:.2} ms  {} samples at {:.0} kS/s  {:.4} MHz +/-{:.0} kHz",
        secs * 1e3,
        iq.samples.len(),
        iq.rate / 1e3,
        iq.center_hz as f64 / 1e6,
        half_span / 1e3,
    )));
    let width = ui.available_width().max(200.0);
    // A strip of envelope under the spectrogram: the two together are the
    // amplitude and the frequency of the burst, which between them show what
    // any of the classes looks like.
    let env_h = (height * 0.22).clamp(18.0, 48.0);
    let spec_h = (height - env_h - 2.0).max(24.0);
    let (resp, p) = ui.allocate_painter(Vec2::new(width, spec_h), Sense::hover());
    let rect = resp.rect;
    p.rect_filled(rect, 2.0, theme::WELL);
    let cols = (rect.width() as usize).max(1);
    // A short transform window: 128 samples, so the time axis resolves the
    // keying rather than averaging a spectrum over many symbols, which is
    // what a long window did and what smeared an on-off burst into a solid
    // band. 128 bins over the extraction's span is ample frequency detail
    // for what this shows. The columns overlap, one per pixel, so the time
    // detail is the panel's width rather than the window.
    let rows = 256usize;
    // A window that is a fixed fraction of the burst, so a fast and a slow
    // extraction of the same signal read alike rather than one crisp and one
    // smeared. About three hundred resolvable time cells across the burst.
    let win = (iq.samples.len() / 300).clamp(8, rows);
    let img = dsp::spectrum::spectrogram(&iq.samples, cols, rows, win);
    let n = img.len() / cols;
    // The floor is the median cell; the top is the peak. A fixed range would
    // wash out a weak burst or clip a strong one, and the burst is all there
    // is on screen so its own range is the right one.
    let mut sorted: Vec<f32> = img.iter().copied().filter(|v| v.is_finite()).collect();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let floor = sorted.get(sorted.len() / 2).copied().unwrap_or(-60.0);
    let span = (0.0 - floor).max(6.0);
    let mut pixels = vec![Color32::BLACK; cols * n];
    for r in 0..n {
        // Row zero of the transform is the lowest frequency; the screen has
        // the highest at the top, so the image is filled upside down.
        let dst = (n - 1 - r) * cols;
        for c in 0..cols {
            let v = ((img[r * cols + c] - floor) / span).clamp(0.0, 1.0);
            pixels[dst + c] = crate::waterfall::colormap(v);
        }
    }
    let image = ColorImage {
        size: [cols, n],
        pixels,
        source_size: egui::Vec2::new(cols as f32, n as f32),
    };
    // Linear filtering fills the panel height smoothly from the transform's
    // rows, which reads as a spectrogram rather than a grid of cells.
    let tex = ui.ctx().load_texture("burst_spectrogram", image, TextureOptions::LINEAR);
    p.image(
        tex.id(),
        rect,
        Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)),
        Color32::WHITE,
    );

    let font = FontId::new(9.0, FontFamily::Name(theme::LEGEND_FONT.into()));
    p.text(
        Pos2::new(rect.left() + 4.0, rect.top() + 2.0),
        Align2::LEFT_TOP,
        format!("+{:.0} kHz", half_span / 1e3),
        font.clone(),
        theme::LEGEND,
    );
    p.text(
        Pos2::new(rect.left() + 4.0, rect.bottom() - 2.0),
        Align2::LEFT_BOTTOM,
        format!("-{:.0} kHz", half_span / 1e3),
        font.clone(),
        theme::LEGEND,
    );
    p.text(
        Pos2::new(rect.right() - 4.0, rect.top() + 2.0),
        Align2::RIGHT_TOP,
        format!("{:.2} ms", secs * 1e3),
        font.clone(),
        theme::LEGEND,
    );
    // DC line, so a reader knows where zero frequency sits.
    let mid = rect.center().y;
    p.line_segment(
        [Pos2::new(rect.left(), mid), Pos2::new(rect.right(), mid)],
        Stroke::new(1.0, Color32::from_rgba_unmultiplied(0x8B, 0x92, 0x9C, 40)),
    );

    // The envelope beneath, filled from its own floor.
    ui.add_space(2.0);
    let (eresp, ep) = ui.allocate_painter(Vec2::new(width, env_h), Sense::hover());
    let erect = eresp.rect;
    ep.rect_filled(erect, 2.0, theme::WELL);
    let env = burst_columns(&iq.samples, iq.rate, erect.width() as usize);
    let peak = env.iter().map(|(e, _)| *e).fold(1e-6f32, f32::max);
    for (c, (e, _)) in env.iter().enumerate() {
        let h = (e / peak) * (erect.height() - 3.0);
        let x = erect.left() + c as f32 + 0.5;
        ep.line_segment(
            [Pos2::new(x, erect.bottom() - 1.0), Pos2::new(x, erect.bottom() - 1.0 - h)],
            Stroke::new(
                1.0,
                Color32::from_rgba_unmultiplied(theme::TRACE.r(), theme::TRACE.g(), theme::TRACE.b(), 150),
            ),
        );
    }
    ep.text(
        Pos2::new(erect.left() + 4.0, erect.top() + 1.0),
        Align2::LEFT_TOP,
        "envelope",
        font,
        theme::LEGEND,
    );

    if let Some(pos) = resp.hover_pos() {
        let t = ((pos.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64 * secs;
        let hz = (mid - pos.y) / (rect.height() / 2.0) * half_span as f32;
        resp.on_hover_text(format!("{:.3} ms   {:+.1} kHz", t * 1e3, hz / 1e3));
    }
}

/// Offset, hex, and printable ASCII, sixteen bytes to the line.
///
/// The bytes are what a protocol is worked out from, so they are shown as they
/// are rather than summarised. For an unknown burst these are the bits sliced
/// under a guessed coding, which is a guess about the framing and not about
/// the reception.
pub(super) fn hex_dump(ui: &mut egui::Ui, bytes: &[u8]) {
    if bytes.is_empty() {
        ui.label(legend("no bits could be read from this burst"));
        return;
    }
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .id_salt("hex")
        .show(ui, |ui| {
            for (i, row) in bytes.chunks(16).enumerate() {
                let hex: String = row
                    .iter()
                    .enumerate()
                    .map(|(k, b)| if k == 7 { format!("{b:02x}  ") } else { format!("{b:02x} ") })
                    .collect();
                let ascii: String = row
                    .iter()
                    .map(|b| if b.is_ascii_graphic() || *b == b' ' { *b as char } else { '.' })
                    .collect();
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 8.0;
                    ui.label(mono(&format!("{:04x}", i * 16), theme::LEGEND));
                    ui.label(mono(&format!("{hex:<49}"), theme::VALUE));
                    ui.label(mono(&ascii, theme::TRACE));
                });
            }
        });
}

/// Group a large count so it can be read at a glance rather than counted.
pub(super) fn thousands(n: u64) -> String {
    let s = n.to_string();
    let mut out = String::with_capacity(s.len() + s.len() / 3);
    for (i, c) in s.chars().enumerate() {
        if i > 0 && (s.len() - i).is_multiple_of(3) {
            out.push(' ');
        }
        out.push(c);
    }
    out
}
