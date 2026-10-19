use super::config::Config;
use crate::ui::App;
use egui_bench::agent::{Agent, Handled, Host};
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
    egui_bench::agent::serve(app, ctx);
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
