use std::collections::BTreeMap;
use std::io::{Error, ErrorKind};
use std::path::{Path, PathBuf};
use std::sync::Mutex;

static FILES: Mutex<BTreeMap<PathBuf, String>> = Mutex::new(BTreeMap::new());

pub async fn preload(root: &Path, keep: fn(&Path) -> bool) {
    let mut dirs = vec![root.to_path_buf()];
    let mut found = BTreeMap::new();
    while let Some(dir) = dirs.pop() {
        for entry in crate::fs::read_dir(&dir).await.unwrap_or_default() {
            if entry.is_dir() {
                dirs.push(entry.path().to_path_buf());
            } else if keep(entry.path())
                && let Ok(text) = crate::fs::read_to_string(entry.path()).await
            {
                found.insert(entry.path().to_path_buf(), text);
            }
        }
    }
    if let Ok(mut files) = FILES.lock() {
        files.extend(found);
    }
}

pub fn read(path: &Path) -> std::io::Result<String> {
    FILES
        .lock()
        .ok()
        .and_then(|files| files.get(path).cloned())
        .ok_or_else(|| Error::new(ErrorKind::NotFound, path.display().to_string()))
}

pub fn write(path: &Path, text: &str) -> std::io::Result<()> {
    if let Ok(mut files) = FILES.lock() {
        files.insert(path.to_path_buf(), text.to_string());
    }
    crate::fs::write_behind(path, text.as_bytes())
}

pub fn list(dir: &Path) -> Vec<PathBuf> {
    FILES
        .lock()
        .map(|files| files.keys().filter(|p| p.parent() == Some(dir)).cloned().collect())
        .unwrap_or_default()
}
