use std::path::Path;

pub fn files() -> Vec<(String, String)> {
    let mut files = Vec::new();
    if let Some(cache) = crate::data::cache() {
        let repo = &datasets::git::PROTOCOLS;
        let root = repo.cache_dir(cache);
        for rel in datasets::git::files_with(repo, cache, "yaml") {
            read_into(&root.join(&rel), &mut files);
        }
    }
    files
}

fn read_into(path: &Path, files: &mut Vec<(String, String)>) {
    match common::fs::blocking::read_to_string(path) {
        Ok(text) => files.push((path.display().to_string(), text)),
        Err(e) => tracing::warn!(path = %path.display(), "unreadable protocol description: {e}"),
    }
}
