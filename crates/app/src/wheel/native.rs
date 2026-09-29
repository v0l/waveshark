use egui::MouseWheelUnit;

const POINTS_PER_NOTCH: f32 = 50.0;

pub fn notches(unit: MouseWheelUnit, delta: f32) -> f32 {
    match unit {
        MouseWheelUnit::Line => delta,
        MouseWheelUnit::Point => delta / POINTS_PER_NOTCH,
        MouseWheelUnit::Page => delta * 8.0,
    }
}
