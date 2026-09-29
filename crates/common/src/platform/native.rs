use std::path::PathBuf;

pub fn scratch_dir() -> PathBuf {
    std::env::temp_dir()
}

fn under(xdg: &str, home: &str) -> Option<PathBuf> {
    let base = std::env::var_os(xdg)
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(home)))?;
    Some(base.join("waveshark"))
}

pub fn config_dir() -> Option<PathBuf> {
    under("XDG_CONFIG_HOME", ".config")
}

pub fn data_dir() -> Option<PathBuf> {
    under("XDG_DATA_HOME", ".local/share")
}

pub fn cache_dir() -> Option<PathBuf> {
    under("XDG_CACHE_HOME", ".cache")
}

pub fn cross_origin_only() -> bool {
    false
}
