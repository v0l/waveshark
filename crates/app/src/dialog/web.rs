use std::path::{Path, PathBuf};
use wasm_bindgen::prelude::*;

#[wasm_bindgen(inline_js = r#"
function chosen(exts) {
    return new Promise((resolve) => {
        const input = document.createElement("input");
        input.type = "file";
        if (exts.length) input.accept = exts.map((e) => "." + e).join(",");
        input.onchange = () => resolve(input.files[0] ?? null);
        input.oncancel = () => resolve(null);
        input.click();
    });
}

async function picked(show, options) {
    try {
        return await show(options);
    } catch (e) {
        if (e.name !== "TypeError" || !options.types) throw e;
        return await show({ ...options, types: undefined });
    }
}

export async function pick(kind, title, name, exts) {
    const allowed = exts.filter((e) => /^[A-Za-z0-9+.]{1,15}$/.test(e));
    const types = allowed.length
        ? [{ description: title || "Files", accept: { "application/octet-stream": allowed.map((e) => "." + e) } }]
        : undefined;
    try {
        if (kind === "open") {
            if (window.showOpenFilePicker) {
                const [h] = await picked((o) => window.showOpenFilePicker(o), { types });
                return { handle: h, name: h.name, dir: false };
            }
            const f = await chosen(exts);
            return f && { handle: f, name: f.name, dir: false };
        }
        if (kind === "folder") {
            if (!window.showDirectoryPicker) return null;
            const h = await window.showDirectoryPicker({ mode: "readwrite" });
            return { handle: h, name: h.name, dir: true };
        }
        if (window.showSaveFilePicker) {
            const h = await picked((o) => window.showSaveFilePicker(o), {
                suggestedName: name || undefined,
                types,
            });
            return { handle: h, name: h.name, dir: false };
        }
        return { download: true, name: name || "waveshark", dir: false };
    } catch (e) {
        if (e.name === "AbortError") return null;
        throw e;
    }
}
"#)]
extern "C" {
    #[wasm_bindgen(catch)]
    async fn pick(
        kind: &str,
        title: &str,
        name: &str,
        exts: Vec<String>,
    ) -> Result<JsValue, JsValue>;
}

#[derive(Default)]
pub struct FileDialog {
    title: String,
    name: String,
    exts: Vec<String>,
}

#[derive(Clone, Copy)]
enum Kind {
    Open,
    Folder,
    Save,
}

impl Kind {
    fn name(self) -> &'static str {
        match self {
            Kind::Open => "open",
            Kind::Folder => "folder",
            Kind::Save => "save",
        }
    }
}

impl FileDialog {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_title(mut self, title: impl Into<String>) -> Self {
        self.title = title.into();
        self
    }

    pub fn set_directory(self, _: impl AsRef<Path>) -> Self {
        self
    }

    pub fn set_file_name(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    pub fn add_filter(mut self, _: impl Into<String>, exts: &[impl ToString]) -> Self {
        self.exts.extend(exts.iter().map(|e| e.to_string()).filter(|e| e != "*"));
        self
    }

    pub fn pick_file(self) -> Option<PathBuf> {
        self.ask(Kind::Open)
    }

    pub fn pick_folder(self) -> Option<PathBuf> {
        self.ask(Kind::Folder)
    }

    pub fn save_file(self) -> Option<PathBuf> {
        self.ask(Kind::Save)
    }

    fn ask(self, kind: Kind) -> Option<PathBuf> {
        if common::page::here() {
            tracing::error!("a file picker was asked for on the page's own thread");
            return None;
        }
        let (tx, rx) = std::sync::mpsc::channel();
        let job: common::page::Job = Box::new(move || {
            Box::pin(async move {
                let picked = pick(kind.name(), &self.title, &self.name, self.exts).await;
                let at = match picked {
                    Ok(p) if p.is_null() => None,
                    Ok(p) => common::fs::mount(p)
                        .await
                        .inspect_err(|e| tracing::warn!("the picked file: {e}"))
                        .ok(),
                    Err(e) => {
                        tracing::warn!("file picker: {e:?}");
                        None
                    }
                };
                let _ = tx.send(at);
            })
        });
        common::page::run(job).ok()?;
        rx.recv().ok().flatten()
    }
}
