use egui::MouseWheelUnit;

const POINTS_PER_NOTCH: f32 = 100.0;
const LINES_PER_NOTCH: f32 = 3.0;

pub fn notches(unit: MouseWheelUnit, delta: f32) -> f32 {
    let notches = match unit {
        MouseWheelUnit::Line => delta / LINES_PER_NOTCH,
        MouseWheelUnit::Point => delta / POINTS_PER_NOTCH,
        MouseWheelUnit::Page => delta * 8.0,
    };
    match notches.abs() >= 0.5 {
        true => notches.round(),
        false => notches,
    }
}
