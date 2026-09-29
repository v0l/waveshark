use std::path::PathBuf;

pub fn scratch_dir() -> PathBuf {
    PathBuf::from("/tmp")
}

pub fn config_dir() -> Option<PathBuf> {
    Some(PathBuf::from("/config"))
}

pub fn data_dir() -> Option<PathBuf> {
    Some(PathBuf::from("/data"))
}

pub fn cache_dir() -> Option<PathBuf> {
    Some(PathBuf::from("/cache"))
}

pub fn cross_origin_only() -> bool {
    true
}
