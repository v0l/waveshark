use super::config::Config;
use crate::ui::App;
use agent_ui::{Agent, Handled, Host};
use serde_json::Value;
use std::sync::Arc;

pub struct Assistant {
    pub config: Config,
    pub page: Agent<App>,
    synced: Option<Config>,
}

impl Default for Assistant {
    fn default() -> Self {
        Self::new(&egui::Context::default())
    }
}

impl Assistant {
    pub fn new(ctx: &egui::Context) -> Self {
        let page = Agent::new(ctx, Arc::new(super::backend::Receiver::default()), true);
        Self { config: Config::load(), page, synced: None }
    }

    pub fn poll(&mut self) {
        self.page.poll();
    }

    pub fn turns(&self) -> usize {
        self.page.chat.turns.len()
    }

    pub fn needs_you(&self) -> bool {
        self.page.needs_you()
    }

    pub fn follow(&mut self) {
        if self.synced.as_ref() == Some(&self.config) {
            return;
        }
        let mut shared = self.page.chat.config.clone();
        shared.url = self.config.url.clone();
        shared.model = self.config.model.clone();
        shared.key = self.config.key.clone();
        shared.steps = self.config.steps;
        shared.brief = self.config.brief.clone();
        shared.workspace = self.config.workspace.clone();
        self.page.set_config(shared);
        self.synced = Some(self.config.clone());
    }
}

pub fn serve(app: &mut App, ctx: &egui::Context) {
    agent_ui::serve(app, ctx);
}

pub fn call(app: &mut App, ctx: &egui::Context, name: &str, args: &Value) -> Handled<App> {
    use crate::agent::Action;
    let Some(tool) = super::catalog::find(name) else {
        return Handled::Now(Err(format!("no tool called {name}")));
    };
    let action = match tool.action(args.clone()) {
        Ok(a) => a,
        Err(e) => return Handled::Now(Err(e)),
    };
    let (tx, mut rx) = tokio::sync::oneshot::channel();
    match action {
        Action::Screenshot => app.agent_screenshot(ctx, tx),
        other => app.agent_take(other, tx, ctx),
    }
    match rx.try_recv() {
        Ok(answer) => Handled::Now(answer),
        Err(tokio::sync::oneshot::error::TryRecvError::Closed) => {
            Handled::Now(Err("the interface dropped the request".into()))
        }
        Err(tokio::sync::oneshot::error::TryRecvError::Empty) => {
            Handled::Later(Box::new(move |_| match rx.try_recv() {
                Ok(answer) => Some(answer),
                Err(tokio::sync::oneshot::error::TryRecvError::Empty) => None,
                Err(_) => Some(Err("the interface dropped the request".into())),
            }))
        }
    }
}

impl Host for App {
    fn agent(&mut self) -> &mut Agent<App> {
        &mut self.assistant.page
    }

    fn call(&mut self, ctx: &egui::Context, name: &str, args: &Value) -> Handled<Self> {
        call(self, ctx, name, args)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_shared_agent_follows_the_model_settings_and_leaves_the_voice_to_waveshark() {
        let mut a = Assistant::default();
        a.config.url = "http://somewhere:11434/v1".into();
        a.config.model = "qwen3:8b".into();
        a.config.key = "secret".into();
        a.config.workspace = "/work/radio".into();
        a.config.wake = "shark".into();
        let kept = a.page.chat.config.rest.clone();
        a.follow();
        let shared = &a.page.chat.config;
        assert_eq!(shared.url, "http://somewhere:11434/v1");
        assert_eq!(shared.model, "qwen3:8b");
        assert_eq!(shared.key, "secret");
        assert_eq!(shared.workspace, "/work/radio");
        assert_eq!(shared.rest, kept, "the voice settings are not the shared agent's to change");
    }

    use agent_ui::chat::Turn;
    use egui_kittest::Harness;

    fn pane(a: Assistant) -> Harness<'static, Assistant> {
        let mut setup = egui_kittest::wgpu::default_wgpu_setup();
        if let egui_wgpu::WgpuSetup::CreateNew(new) = &mut setup {
            new.instance_descriptor.flags = egui_wgpu::wgpu::InstanceFlags::empty();
        }
        let mut h = Harness::builder().with_size([900.0, 800.0]).wgpu_setup(setup).build_ui_state(
            |ui, a: &mut Assistant| {
                egui_bench::theme::install(ui.ctx());
                a.page.show(ui);
            },
            a,
        );
        h.run_steps(4);
        h
    }

    fn painted(h: &Harness<'_, Assistant>) -> String {
        fn walk(s: &egui::Shape, out: &mut String) {
            match s {
                egui::Shape::Text(t) => {
                    out.push_str(t.galley.text());
                    out.push('\n');
                }
                egui::Shape::Vec(v) => v.iter().for_each(|s| walk(s, out)),
                _ => {}
            }
        }
        let mut out = String::new();
        for c in &h.output().shapes {
            walk(&c.shape, &mut out);
        }
        out.to_lowercase()
    }

    fn conversation(a: &mut Assistant) {
        a.page.chat.config.model = "qwen3:8b".into();
        let t = &mut a.page.chat.turns;
        t.push(Turn::Thinking("The tower is usually on AM near 118 MHz.".into()));
        t.push(Turn::Did {
            name: "tune".into(),
            args: serde_json::json!({"mhz": 118.3}).to_string(),
            answer: Some(Ok("tuned".into())),
        });
        t.push(Turn::Said("Tuned to **118.3 MHz**, the tower is on AM.".into()));
    }

    #[test]
    fn the_assistant_pane_shows_reasoning_and_a_tool_row_through_the_shared_widgets() {
        let mut a = Assistant::default();
        conversation(&mut a);
        let h = pane(a);
        let text = painted(&h);
        assert!(text.contains("the tower is usually on am"), "the reasoning is drawn: {text}");
        assert!(text.contains("tune"), "the tool row is drawn: {text}");
        assert!(text.contains("118.3 mhz"), "the reply is drawn: {text}");
        if let Ok(path) = std::env::var("ASSISTANT_SHOT") {
            let mut h = h;
            h.render().expect("rendered").save(path).unwrap();
        }
    }

    #[test]
    fn following_twice_does_not_rewrite_what_the_shared_agent_holds() {
        let mut a = Assistant::default();
        a.config.model = "m".into();
        a.follow();
        a.page.chat.config.steps = 3;
        a.follow();
        assert_eq!(a.page.chat.config.steps, 3, "an unchanged setting must not be pushed again");
    }
}
