use super::{Asset, INSTALL, Install, Kind};
use common::time::Duration;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};

static FETCHING: AtomicBool = AtomicBool::new(false);

/// Fetch this platform's installer and hand it to the system.
///
/// Nothing is unpacked or overwritten here. The file goes beside the other
/// downloads, under the cache directory, and then to whatever the desktop
/// opens a package with: `msiexec` behind the .msi, Finder behind the .dmg,
/// the distribution's package installer behind the .deb or .rpm.
pub fn install(asset: Asset) {
    if FETCHING.swap(true, Ordering::AcqRel) {
        return;
    }
    *INSTALL.write() = Install::Fetching { got: 0, total: asset.bytes };
    crate::task::spawner().spawn(async move {
        let outcome =
            download(&asset).await.and_then(|path| hand_over(&path, asset.kind).map(|()| path));
        *INSTALL.write() = match outcome {
            Ok(path) => {
                tracing::info!(file = %path.display(), "the installer is open");
                Install::Launched(path)
            }
            Err(e) => {
                tracing::warn!("the installer could not be fetched: {e}");
                Install::Failed(e)
            }
        };
        FETCHING.store(false, Ordering::Release);
    });
}

async fn download(asset: &Asset) -> Result<PathBuf, String> {
    let dir = crate::data::cache_dir().unwrap_or_else(std::env::temp_dir).join("updates");
    std::fs::create_dir_all(&dir).map_err(|e| e.to_string())?;
    let path = dir.join(&asset.name);

    // A release asset is tens of megabytes over a link that may be a phone,
    // so the timeout is long and the file is written as it arrives rather
    // than held whole in memory.
    let http = httpc::client(Duration::from_secs(600)).map_err(|e| e.to_string())?;
    let mut res = http
        .get(&asset.url)
        .send()
        .await
        .map_err(|e| e.to_string())?
        .error_for_status()
        .map_err(|e| e.to_string())?;
    let total = res.content_length().unwrap_or(asset.bytes);
    let part = path.with_extension("part");
    let mut file = tokio::fs::File::create(&part).await.map_err(|e| e.to_string())?;
    let mut got = 0u64;
    while let Some(chunk) = res.chunk().await.map_err(|e| e.to_string())? {
        use tokio::io::AsyncWriteExt;
        file.write_all(&chunk).await.map_err(|e| e.to_string())?;
        got += chunk.len() as u64;
        *INSTALL.write() = Install::Fetching { got, total };
    }
    use tokio::io::AsyncWriteExt;
    file.flush().await.map_err(|e| e.to_string())?;
    drop(file);
    // The size the release published is the only check available without a
    // signature, and a truncated installer is the failure worth catching:
    // a proxy that answers an error page is otherwise run as a package.
    if asset.bytes > 0 && got != asset.bytes {
        let _ = std::fs::remove_file(&part);
        return Err(format!("{got} bytes arrived of {} expected", asset.bytes));
    }
    std::fs::rename(&part, &path).map_err(|e| e.to_string())?;
    Ok(path)
}

/// Open what was downloaded: the package, so the desktop installs it, or the
/// folder the binary landed in, since opening a program file would hand it to
/// a text editor.
fn hand_over(path: &Path, kind: Kind) -> Result<(), String> {
    let path = match kind {
        Kind::Installer => path,
        Kind::Binary => {
            // A downloaded file is not executable, and a receiver nobody can
            // start is not an upgrade.
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                let mut p = std::fs::metadata(path).map_err(|e| e.to_string())?.permissions();
                p.set_mode(0o755);
                std::fs::set_permissions(path, p).map_err(|e| e.to_string())?;
            }
            path.parent().ok_or_else(|| "nowhere to open".to_string())?
        }
    };
    #[cfg(target_os = "windows")]
    let mut cmd = {
        // `start` is a shell builtin rather than a program, and the empty
        // string is the window title `start` reads its first argument as.
        let mut c = std::process::Command::new("cmd");
        c.args(["/C", "start", ""]).arg(path);
        c
    };
    #[cfg(target_os = "macos")]
    let mut cmd = {
        let mut c = std::process::Command::new("open");
        c.arg(path);
        c
    };
    #[cfg(all(unix, not(target_os = "macos")))]
    let mut cmd = {
        let mut c = std::process::Command::new("xdg-open");
        c.arg(path);
        c
    };
    cmd.spawn().map(|_| ()).map_err(|e| format!("{e}: {}", path.display()))
}
