use super::catalog;
use egui_bench::agent::skillset::Skill;
use egui_bench::agent::toolkit::{Tool, Toolkit};
use egui_bench::agent::{Backend, Welcome};
use serde_json::Value;
use std::path::Path;

pub struct Receiver {
    kit: Toolkit,
}

impl Default for Receiver {
    fn default() -> Self {
        let mut kit = Toolkit::new();
        for tool in catalog::all() {
            kit.push(Tool::host_shaped(
                tool.name,
                tool.about,
                tool.schema.clone(),
                without_picture,
            ));
        }
        Self { kit }
    }
}

fn without_picture(mut answer: Value) -> Value {
    if let Some(o) = answer.as_object_mut()
        && o.remove("png_base64").is_some()
    {
        o.insert("note".into(), serde_json::json!("the picture is in the window"));
    }
    answer
}

impl Backend for Receiver {
    fn app(&self) -> &'static str {
        "waveshark"
    }

    fn config_dir(&self) -> Option<std::path::PathBuf> {
        common::platform::config_dir()
    }

    fn user_agent(&self) -> String {
        httpc::USER_AGENT.to_string()
    }

    fn toolkit(&self) -> &Toolkit {
        &self.kit
    }

    fn brief(&self, workspace: &Path, skills: &[Skill]) -> String {
        format!(
            "{}\n\nYou also work in a folder on this computer, {}.\n{}\n\nSkills installed \
             now, for load_skill:\n{}\n",
            catalog::BRIEF,
            workspace.display(),
            egui_bench::agent::tools::GUIDE,
            egui_bench::agent::skillset::listing(skills)
        )
    }

    fn welcome(&self) -> Welcome {
        Welcome {
            kicker: "agent".into(),
            title: "The receiver, and a folder to work in".into(),
            text: "The model has the whole receiver: what it tunes, opens or switches on happens \
                   in this window, and what it reads is what the panes are showing. It can also \
                   read and write files in a workspace folder and run commands in a sandbox, so \
                   it can write down what it hears or script around it."
                .into(),
            placeholder: "What is on 446 MHz?".into(),
        }
    }

    fn summary(&self, name: &str, args: &Value) -> Option<String> {
        self.kit.summary(name, args)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use egui_bench::agent::local::NAMES;

    #[test]
    fn the_catalogue_reaches_the_shared_agent_whole_and_clashes_with_nothing() {
        let backend = Receiver::default();
        let kit = backend.toolkit();
        assert_eq!(kit.len(), catalog::all().len());
        assert!(kit.problems(&NAMES).is_empty(), "{:?}", kit.problems(&NAMES));
    }

    #[test]
    fn the_model_is_offered_the_receiver_and_the_workspace_tools() {
        let offered: Vec<String> = egui_bench::agent::tools::offered(&Receiver::default())
            .into_iter()
            .map(|s| s.name)
            .collect();
        for want in ["status", "tune", "add_channel", "load_skill", "read_file", "request_access"] {
            assert!(offered.iter().any(|n| n == want), "{want} is not offered: {offered:?}");
        }
    }

    #[test]
    fn a_picture_is_left_on_the_screen_and_not_sent_to_the_model() {
        let shaped = without_picture(serde_json::json!({ "png_base64": "AAAA", "width": 10 }));
        assert!(shaped.get("png_base64").is_none());
        assert_eq!(shaped["width"], 10);
        assert_eq!(shaped["note"], "the picture is in the window");
        let other = serde_json::json!({ "center_hz": 1.0 });
        assert_eq!(without_picture(other.clone()), other);
    }

    #[test]
    fn the_brief_carries_the_receivers_and_the_workspace_guidance() {
        let brief = Receiver::default().brief(std::path::Path::new("/work/radio"), &[]);
        assert!(brief.starts_with("WaveShark, a wideband software radio receiver"));
        assert!(brief.contains("/work/radio"));
        assert!(brief.contains("request_access"));
        assert!(brief.contains("No skills are installed."));
    }

    #[test]
    fn the_settings_file_is_the_one_the_chat_always_used() {
        let dir = Receiver::default().config_dir().expect("a config folder");
        assert_eq!(dir.file_name().and_then(|n| n.to_str()), Some("waveshark"));
        assert_eq!(super::super::config::Config::path(), Some(dir.join("agent")));
    }
}
