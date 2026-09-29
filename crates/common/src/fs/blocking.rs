use super::{DirEntry, Metadata, platform};
use std::path::Path;

pub use platform::File;

fn wait<T>(work: impl std::future::Future<Output = std::io::Result<T>>) -> std::io::Result<T> {
    platform::may_block()?;
    crate::thread::wait(work)
}

pub fn read(path: impl AsRef<Path>) -> std::io::Result<Vec<u8>> {
    wait(super::read(path))
}

pub fn read_to_string(path: impl AsRef<Path>) -> std::io::Result<String> {
    wait(super::read_to_string(path))
}

pub fn write(path: impl AsRef<Path>, bytes: impl AsRef<[u8]>) -> std::io::Result<()> {
    wait(super::write(path, bytes))
}

pub fn create_dir_all(path: impl AsRef<Path>) -> std::io::Result<()> {
    wait(super::create_dir_all(path))
}

pub fn remove_file(path: impl AsRef<Path>) -> std::io::Result<()> {
    wait(super::remove_file(path))
}

pub fn remove_dir_all(path: impl AsRef<Path>) -> std::io::Result<()> {
    wait(super::remove_dir_all(path))
}

pub fn rename(from: impl AsRef<Path>, to: impl AsRef<Path>) -> std::io::Result<()> {
    wait(super::rename(from, to))
}

pub fn metadata(path: impl AsRef<Path>) -> std::io::Result<Metadata> {
    wait(super::metadata(path))
}

pub fn read_dir(path: impl AsRef<Path>) -> std::io::Result<Vec<DirEntry>> {
    wait(super::read_dir(path))
}

pub fn exists(path: impl AsRef<Path>) -> bool {
    metadata(path).is_ok()
}
