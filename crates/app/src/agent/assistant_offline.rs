use super::config::Config;
use crate::ui::App;

pub struct Assistant {
    pub config: Config,
}

impl Default for Assistant {
    fn default() -> Self {
        Self { config: Config::load() }
    }
}

impl Assistant {
    pub fn new(_ctx: &egui::Context) -> Self {
        Self::default()
    }

    pub fn poll(&mut self) {}

    pub fn turns(&self) -> usize {
        0
    }

    pub fn needs_you(&self) -> bool {
        false
    }

    pub fn follow(&mut self) {}
}

pub fn serve(_app: &mut App, _ctx: &egui::Context) {}
