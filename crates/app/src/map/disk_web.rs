use std::path::Path;

pub(super) async fn read(_: &Path) -> Option<Vec<u8>> {
    None
}

pub(super) async fn write(_: &Path, _: &[u8]) {}
