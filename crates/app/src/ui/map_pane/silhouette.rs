use datasets::aircraft::Class;
use egui::emath::Rot2;
use egui::{Color32, ColorImage, Mesh, Pos2, Rect, TextureHandle, TextureOptions, Vec2};
use egui_bench::theme;

const PX: usize = 64;
const SUB: usize = 4;
const REACH: f32 = 1.2;
const HALO: f32 = 0.12;

pub(super) fn span(class: Class) -> f32 {
    match class {
        Class::Heavy => 28.0,
        Class::Jet => 23.0,
        Class::Glider => 21.0,
        Class::Twin | Class::Rotorcraft => 19.0,
        Class::Single => 16.0,
        Class::Balloon => 15.0,
    }
}

pub(super) fn draw(p: &egui::Painter, at: Pos2, class: Class, course_deg: f64, col: Color32) {
    let [fill, halo] = textures(p.ctx(), class);
    let rect = Rect::from_center_size(at, Vec2::splat(span(class) * REACH));
    let turn = match class {
        Class::Balloon => Rot2::IDENTITY,
        _ => Rot2::from_angle((course_deg as f32).to_radians()),
    };
    let shade = theme::CHASSIS.gamma_multiply(0.8 * f32::from(col.a()) / 255.0);
    for (tex, colour) in [(halo, shade), (fill, col)] {
        let mut m = Mesh::with_texture(tex.id());
        m.add_rect_with_uv(rect, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), colour);
        m.rotate(turn, at);
        p.add(m);
    }
}

fn textures(ctx: &egui::Context, class: Class) -> [TextureHandle; 2] {
    let id = egui::Id::new(("silhouette", class));
    if let Some(t) = ctx.data(|d| d.get_temp::<[TextureHandle; 2]>(id)) {
        return t;
    }
    let opts = TextureOptions::LINEAR.with_mipmap_mode(Some(egui::TextureFilter::Linear));
    let [fill, halo] = masks(class);
    let t = [
        ctx.load_texture(format!("silhouette-{class:?}"), fill, opts),
        ctx.load_texture(format!("silhouette-{class:?}-halo"), halo, opts),
    ];
    ctx.data_mut(|d| d.insert_temp(id, t.clone()));
    t
}

fn masks(class: Class) -> [ColorImage; 2] {
    let n = PX * SUB;
    let cell = 2.0 * REACH / n as f32;
    let at = |i: usize| -REACH + (i as f32 + 0.5) * cell;
    let fill: Vec<bool> = (0..n * n).map(|i| covers(class, at(i % n), at(i / n))).collect();
    let halo = dilate(&fill, n, (HALO / cell).round() as usize);
    [image(&fill, n), image(&halo, n)]
}

fn dilate(m: &[bool], n: usize, r: usize) -> Vec<bool> {
    let near = |i: usize| i.saturating_sub(r)..=(i + r).min(n - 1);
    let rows: Vec<bool> = (0..n * n).map(|i| near(i % n).any(|x| m[i / n * n + x])).collect();
    (0..n * n).map(|i| near(i / n).any(|y| rows[y * n + i % n])).collect()
}

fn image(m: &[bool], n: usize) -> ColorImage {
    let pixels = (0..PX * PX)
        .map(|i| {
            let (px, py) = (i % PX * SUB, i / PX * SUB);
            let hit = (0..SUB * SUB).filter(|s| m[(py + s / SUB) * n + px + s % SUB]).count();
            Color32::from_white_alpha((hit * 255 / (SUB * SUB)) as u8)
        })
        .collect();
    ColorImage::new([PX, PX], pixels)
}

fn covers(class: Class, x: f32, y: f32) -> bool {
    let ax = x.abs();
    match class {
        Class::Jet => jet(ax, y, 0.1, &[0.4]),
        Class::Heavy => jet(ax, y, 0.12, &[0.33, 0.64]),
        Class::Single => {
            poly(&[(0.0, -0.92), (0.1, -0.88), (0.13, -0.35), (0.05, 0.82), (0.0, 0.82)], ax, y)
                || rect(ax, y, 0.98, -0.5, -0.25)
                || rect(ax, y, 0.36, 0.62, 0.8)
                || rect(ax, y, 0.3, -1.0, -0.93)
        }
        Class::Twin => {
            poly(
                &[(0.0, -0.95), (0.09, -0.88), (0.12, -0.5), (0.12, 0.3), (0.05, 0.9), (0.0, 0.9)],
                ax,
                y,
            ) || poly(&[(0.0, -0.32), (1.0, -0.25), (1.0, -0.1), (0.0, -0.02)], ax, y)
                || ellipse(ax, y, 0.4, -0.25, 0.1, 0.3)
                || poly(&[(0.0, 0.62), (0.4, 0.72), (0.4, 0.84), (0.0, 0.86)], ax, y)
        }
        Class::Rotorcraft => {
            let (dx, dy) = (x, y + 0.25);
            let blade = (dx - dy).abs() < 0.1 || (dx + dy).abs() < 0.1;
            ellipse(x, y, 0.0, -0.2, 0.26, 0.42)
                || rect(ax, y, 0.055, 0.1, 0.92)
                || rect(ax, y, 0.2, 0.8, 0.9)
                || (blade && dx * dx + dy * dy < 0.8 * 0.8)
        }
        Class::Glider => {
            poly(&[(0.0, -0.8), (0.07, -0.72), (0.07, -0.3), (0.025, 0.88), (0.0, 0.88)], ax, y)
                || poly(&[(0.0, -0.34), (1.0, -0.28), (1.0, -0.22), (0.0, -0.16)], ax, y)
                || rect(ax, y, 0.25, 0.76, 0.86)
        }
        Class::Balloon => {
            ellipse(x, y, 0.0, -0.3, 0.62, 0.66)
                || rect(ax, y, 0.14, 0.62, 0.86)
                || poly(&[(0.33, 0.2), (0.4, 0.2), (0.14, 0.62), (0.09, 0.62)], ax, y)
        }
    }
}

fn jet(ax: f32, y: f32, body: f32, engines: &[f32]) -> bool {
    let (root, tip) = ((-0.18, 0.1), (0.28, 0.36));
    let leading = |x: f32| root.0 + (tip.0 - root.0) * x / 0.95;
    poly(&[(0.0, -1.0), (0.06, -0.95), (body, -0.8), (body, 0.5), (0.05, 0.95), (0.0, 0.97)], ax, y)
        || poly(&[(0.0, root.0), (0.95, tip.0), (0.95, tip.1), (0.0, root.1)], ax, y)
        || poly(&[(0.0, 0.6), (0.38, 0.86), (0.38, 0.93), (0.0, 0.84)], ax, y)
        || engines.iter().any(|&e| ellipse(ax, y, e, leading(e) - 0.03, 0.06, 0.13))
}

fn rect(ax: f32, y: f32, half: f32, top: f32, bottom: f32) -> bool {
    ax <= half && (top..=bottom).contains(&y)
}

fn ellipse(x: f32, y: f32, cx: f32, cy: f32, rx: f32, ry: f32) -> bool {
    let (u, v) = ((x - cx) / rx, (y - cy) / ry);
    u * u + v * v <= 1.0
}

fn poly(pts: &[(f32, f32)], x: f32, y: f32) -> bool {
    let mut inside = false;
    let mut j = pts.len() - 1;
    for i in 0..pts.len() {
        let ((xi, yi), (xj, yj)) = (pts[i], pts[j]);
        if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_shape_fits_its_box_and_its_halo_is_wider() {
        for class in [
            Class::Heavy,
            Class::Jet,
            Class::Twin,
            Class::Single,
            Class::Rotorcraft,
            Class::Glider,
            Class::Balloon,
        ] {
            let [fill, halo] = masks(class);
            let alpha = |img: &ColorImage| img.pixels.iter().map(|p| p.a() as u32).sum::<u32>();
            let edge = |img: &ColorImage| {
                (0..PX).any(|i| {
                    [(i, 0), (i, PX - 1), (0, i), (PX - 1, i)]
                        .iter()
                        .any(|&(x, y)| img.pixels[y * PX + x].a() > 0)
                })
            };
            assert!(alpha(&fill) > 0, "{class:?} drew nothing");
            assert!(alpha(&halo) > alpha(&fill), "{class:?} halo is no wider than its fill");
            assert!(!edge(&halo), "{class:?} runs off the edge of its texture");
        }
    }
}
