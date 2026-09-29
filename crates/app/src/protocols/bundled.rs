use std::sync::OnceLock;

static BUNDLED: OnceLock<Vec<(String, String)>> = OnceLock::new();

pub fn files() -> Vec<(String, String)> {
    BUNDLED.get().cloned().unwrap_or_default()
}

pub async fn fetch() {
    let base = js_sys::Reflect::get(&js_sys::global(), &"location".into())
        .and_then(|l| js_sys::Reflect::get(&l, &"href".into()))
        .ok()
        .and_then(|h| h.as_string())
        .unwrap_or_default();
    let base = base.split(['?', '#']).next().unwrap_or_default();
    let base = &base[..base.rfind('/').map_or(0, |i| i + 1)];
    let got = async {
        let index = text(&format!("{base}protocols/index.txt")).await?;
        let mut files = Vec::new();
        for rel in index.lines().map(str::trim).filter(|l| !l.is_empty()) {
            let url = format!("{base}protocols/{rel}");
            match text(&url).await {
                Ok(body) => files.push((format!("protocols/{rel}"), body)),
                Err(e) => tracing::warn!("bundled protocol description {rel}: {e}"),
            }
        }
        Ok::<_, String>(files)
    };
    match got.await {
        Ok(files) => {
            tracing::info!(files = files.len(), "bundled protocol descriptions");
            let _ = BUNDLED.set(files);
        }
        Err(e) => tracing::warn!("no bundled protocol descriptions: {e}"),
    }
}

async fn text(url: &str) -> Result<String, String> {
    let resp = httpc::get(url).send().await.map_err(|e| e.to_string())?;
    let resp = resp.error_for_status().map_err(|e| e.to_string())?;
    resp.text().await.map_err(|e| e.to_string())
}
