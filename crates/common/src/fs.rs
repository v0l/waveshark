use crate::time::SystemTime;
use std::path::{Path, PathBuf};

#[cfg(not(target_arch = "wasm32"))]
#[path = "fs/native.rs"]
mod platform;
#[cfg(target_arch = "wasm32")]
#[path = "fs/opfs.rs"]
mod platform;

pub use platform::{
    append, create_dir_all, metadata, read, read_dir, remove_dir_all, remove_file, rename, write,
};
#[cfg(target_arch = "wasm32")]
pub use platform::{fs_serve, mount, start, write_behind};

pub mod blocking;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Metadata {
    len: u64,
    dir: bool,
    modified: Option<SystemTime>,
}

impl Metadata {
    pub fn len(&self) -> u64 {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn is_dir(&self) -> bool {
        self.dir
    }

    pub fn is_file(&self) -> bool {
        !self.dir
    }

    pub fn modified(&self) -> Option<SystemTime> {
        self.modified
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DirEntry {
    path: PathBuf,
    dir: bool,
    len: u64,
}

impl DirEntry {
    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn file_name(&self) -> &std::ffi::OsStr {
        self.path.file_name().unwrap_or_default()
    }

    pub fn is_dir(&self) -> bool {
        self.dir
    }

    pub fn len(&self) -> u64 {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

pub async fn read_to_string(path: impl AsRef<Path>) -> std::io::Result<String> {
    String::from_utf8(read(path).await?)
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))
}

pub async fn exists(path: impl AsRef<Path>) -> bool {
    metadata(path).await.is_ok()
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;

    #[test]
    fn a_file_is_written_appended_listed_renamed_and_removed() {
        let dir = std::env::temp_dir().join(format!("common-fs-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        crate::thread::wait(async {
            create_dir_all(dir.join("a/b")).await.unwrap();
            let f = dir.join("a/b/log.txt");
            write(&f, b"one\n").await.unwrap();
            append(&f, b"two\n").await.unwrap();
            assert_eq!(read_to_string(&f).await.unwrap(), "one\ntwo\n");
            let m = metadata(&f).await.unwrap();
            assert_eq!((m.len(), m.is_file()), (8, true));
            assert!(m.modified().is_some());
            assert!(metadata(dir.join("a")).await.unwrap().is_dir());
            let names: Vec<String> = read_dir(dir.join("a"))
                .await
                .unwrap()
                .iter()
                .map(|e| {
                    format!(
                        "{}{}",
                        e.file_name().to_string_lossy(),
                        if e.is_dir() { "/" } else { "" }
                    )
                })
                .collect();
            assert_eq!(names, ["b/"]);
            rename(&f, dir.join("a/moved.txt")).await.unwrap();
            assert!(!exists(&f).await);
            assert_eq!(read(dir.join("a/moved.txt")).await.unwrap(), b"one\ntwo\n");
            assert_eq!(read(&f).await.unwrap_err().kind(), std::io::ErrorKind::NotFound);
            remove_file(dir.join("a/moved.txt")).await.unwrap();
            remove_dir_all(&dir).await.unwrap();
            assert!(!exists(&dir).await);
        });
    }
}
