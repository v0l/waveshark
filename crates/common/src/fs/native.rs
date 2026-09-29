use super::{DirEntry, Metadata};
use std::io::Write;
use std::path::Path;

pub async fn read(path: impl AsRef<Path>) -> std::io::Result<Vec<u8>> {
    std::fs::read(path)
}

pub async fn write(path: impl AsRef<Path>, bytes: impl AsRef<[u8]>) -> std::io::Result<()> {
    std::fs::write(path, bytes)
}

pub async fn append(path: impl AsRef<Path>, bytes: impl AsRef<[u8]>) -> std::io::Result<()> {
    std::fs::OpenOptions::new().create(true).append(true).open(path)?.write_all(bytes.as_ref())
}

pub async fn create_dir_all(path: impl AsRef<Path>) -> std::io::Result<()> {
    std::fs::create_dir_all(path)
}

pub async fn remove_file(path: impl AsRef<Path>) -> std::io::Result<()> {
    std::fs::remove_file(path)
}

pub async fn remove_dir_all(path: impl AsRef<Path>) -> std::io::Result<()> {
    std::fs::remove_dir_all(path)
}

pub async fn rename(from: impl AsRef<Path>, to: impl AsRef<Path>) -> std::io::Result<()> {
    std::fs::rename(from, to)
}

pub async fn metadata(path: impl AsRef<Path>) -> std::io::Result<Metadata> {
    let m = std::fs::metadata(path)?;
    let modified = m.modified().ok().and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok());
    Ok(Metadata {
        len: m.len(),
        dir: m.is_dir(),
        modified: modified.map(|d| crate::time::UNIX_EPOCH + d),
    })
}

pub async fn read_dir(path: impl AsRef<Path>) -> std::io::Result<Vec<DirEntry>> {
    let mut out: Vec<DirEntry> = std::fs::read_dir(path)?
        .filter_map(Result::ok)
        .map(|e| {
            let meta = e.metadata().ok();
            DirEntry {
                path: e.path(),
                dir: meta.as_ref().is_some_and(|m| m.is_dir()),
                len: meta.map_or(0, |m| m.len()),
            }
        })
        .collect();
    out.sort_by(|a, b| a.path.cmp(&b.path));
    Ok(out)
}

pub fn may_block() -> std::io::Result<()> {
    Ok(())
}

pub struct File(std::fs::File);

impl File {
    pub fn open(path: impl AsRef<Path>) -> std::io::Result<Self> {
        std::fs::File::open(path).map(Self)
    }

    pub fn create(path: impl AsRef<Path>) -> std::io::Result<Self> {
        std::fs::File::create(path).map(Self)
    }

    pub fn append(path: impl AsRef<Path>) -> std::io::Result<Self> {
        std::fs::OpenOptions::new().create(true).append(true).open(path).map(Self)
    }

    pub fn sync_all(&mut self) -> std::io::Result<()> {
        self.0.sync_all()
    }

    pub fn len(&self) -> u64 {
        self.0.metadata().map(|m| m.len()).unwrap_or(0)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl std::io::Read for File {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.0.read(buf)
    }
}

impl std::io::Write for File {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.0.write(buf)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.0.flush()
    }
}

impl std::io::Seek for File {
    fn seek(&mut self, to: std::io::SeekFrom) -> std::io::Result<u64> {
        self.0.seek(to)
    }
}
