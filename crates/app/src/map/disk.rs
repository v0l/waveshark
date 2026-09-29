use std::path::Path;

pub(super) async fn read(path: &Path) -> Option<Vec<u8>> {
    tokio::fs::read(path).await.ok()
}

pub(super) async fn write(path: &Path, bytes: &[u8]) {
    if let Some(parent) = path.parent() {
        let _ = tokio::fs::create_dir_all(parent).await;
    }
    let _ = tokio::fs::write(path, bytes).await;
}
