//! What the newest published release is, and whether this build is behind it.
//!
//! The releases the build workflow publishes are the only statement of what
//! the current version is: the crate version in a running binary says what it
//! was built from, not what is available. GitHub answers that in one request
//! (`/releases/latest`, which skips drafts and pre-releases), so the check is
//! a single unauthenticated GET and a version comparison.
//!
//! The check itself replaces nothing. What it can do is fetch the installer
//! this platform uses and hand it to the system: an installer knows how to
//! replace a running program and a program does not know how to replace
//! itself, least of all on Windows, where the file is locked while it runs.

use parking_lot::RwLock;
use serde::Deserialize;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

const LATEST: &str = "https://api.github.com/repos/v0l/waveshark/releases/latest";

/// What this binary was built as.
pub fn running() -> &'static str {
    env!("CARGO_PKG_VERSION")
}

/// A published release, reduced to what an operator has to decide with.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Release {
    /// The tag with any leading `v` removed, so it compares with [`running`].
    pub version: String,
    pub tag: String,
    /// The release page, which is where a person goes.
    pub page: String,
    /// What this platform installs from, when the release carries it. Absent
    /// for a platform the workflow does not build, and for a release whose
    /// assets are still uploading.
    pub asset: Option<Asset>,
    /// ISO 8601, as GitHub gives it. Shown, not parsed.
    pub published: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Asset {
    pub name: String,
    pub url: String,
    pub bytes: u64,
    pub kind: Kind,
}

/// Whether the system can install the asset or the operator has to unpack it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// A .msi, .dmg, .deb or .rpm: handing it to the desktop starts an
    /// install.
    Installer,
    /// The binary itself, or the zip the Windows one is published in with
    /// the libraries it will not start without. Saved, not installed.
    Binary,
}

/// Where the check has got to. `Newer` is the only state that asks anything
/// of the operator.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum State {
    /// Not asked yet.
    #[default]
    Unchecked,
    Checking,
    /// The newest release is this one, or older than it.
    Current(Release),
    Newer(Release),
    Failed(String),
}

static STATE: RwLock<State> = RwLock::new(State::Unchecked);
static BUSY: AtomicBool = AtomicBool::new(false);
static ASKED: AtomicBool = AtomicBool::new(false);

pub fn state() -> State {
    STATE.read().clone()
}

/// Ask once per run. Called at startup; repeated calls do nothing, so a pane
/// may call it while drawing.
pub fn check() {
    if ASKED.swap(true, Ordering::AcqRel) {
        return;
    }
    check_now();
}

/// Ask again whatever the last answer was. This is the button.
pub fn check_now() {
    if BUSY.swap(true, Ordering::AcqRel) {
        return;
    }
    ASKED.store(true, Ordering::Release);
    *STATE.write() = State::Checking;
    let started = std::thread::Builder::new().name("update-check".into()).spawn(|| {
        // A runtime of its own rather than the interface's: the check runs
        // before the window exists, and it is one request a run.
        let outcome = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|e| e.to_string())
            .and_then(|rt| rt.block_on(fetch()));
        *STATE.write() = match outcome {
            Ok(r) if is_newer(&r.version, running()) => {
                tracing::info!(latest = %r.version, running = running(), "a newer release exists");
                State::Newer(r)
            }
            Ok(r) => State::Current(r),
            Err(e) => {
                tracing::warn!("release check failed: {e}");
                State::Failed(e)
            }
        };
        BUSY.store(false, Ordering::Release);
    });
    if started.is_err() {
        BUSY.store(false, Ordering::Release);
        *STATE.write() = State::Failed("could not start the check".into());
    }
}

/// How far the download of an installer has got. Separate from [`State`],
/// which says what the release is: an operator can ask for the installer and
/// then press check again while it comes down.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub enum Install {
    #[default]
    Idle,
    /// Bytes so far, and the size the release said, which is what draws a bar.
    Fetching {
        got: u64,
        total: u64,
    },
    /// Downloaded and handed to the desktop. WaveShark has to close for the
    /// installer to replace it, so the pane that sees this closes the window.
    Launched(PathBuf),
    Failed(String),
}

static INSTALL: RwLock<Install> = RwLock::new(Install::Idle);
static FETCHING: AtomicBool = AtomicBool::new(false);

pub fn install_state() -> Install {
    INSTALL.read().clone()
}

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
    let started = std::thread::Builder::new().name("update-fetch".into()).spawn(move || {
        let outcome = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .map_err(|e| e.to_string())
            .and_then(|rt| rt.block_on(download(&asset)))
            .and_then(|path| hand_over(&path, asset.kind).map(|()| path));
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
    if started.is_err() {
        FETCHING.store(false, Ordering::Release);
        *INSTALL.write() = Install::Failed("could not start the download".into());
    }
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

async fn fetch() -> Result<Release, String> {
    // GitHub rejects a request with no agent, which is one of the reasons
    // every client here is built the same way.
    let http = httpc::client(Duration::from_secs(10)).map_err(|e| e.to_string())?;
    let body = http
        .get(LATEST)
        .header("Accept", "application/vnd.github+json")
        .send()
        .await
        .map_err(|e| e.to_string())?
        .error_for_status()
        .map_err(|e| e.to_string())?
        .text()
        .await
        .map_err(|e| e.to_string())?;
    parse(&body)
}

#[derive(Deserialize)]
struct Json {
    tag_name: String,
    #[serde(default)]
    html_url: String,
    #[serde(default)]
    published_at: String,
    #[serde(default)]
    assets: Vec<JsonAsset>,
}

#[derive(Deserialize)]
struct JsonAsset {
    name: String,
    browser_download_url: String,
    #[serde(default)]
    size: u64,
}

fn parse(body: &str) -> Result<Release, String> {
    let j: Json = serde_json::from_str(body).map_err(|e| e.to_string())?;
    let version = j.tag_name.trim_start_matches('v').to_string();
    if version.is_empty() {
        return Err("the release has no tag".into());
    }
    let asset = pick(&j.assets, &version);
    Ok(Release { version, tag: j.tag_name, page: j.html_url, asset, published: j.published_at })
}

/// The one asset of a release this build installs from.
///
/// Every name is `waveshark-<version>-<platform><suffix>`, the version being
/// the release's own tag without its `v`, and the whole name is matched
/// rather than its start, because `waveshark-0.3.0-linux-x86_64` is a prefix
/// of `waveshark-0.3.0-linux-arm64`'s neighbours in the list and of every
/// package built from it.
///
/// The first suffix this platform can use wins, so an installer is preferred
/// to the bare binary.
fn pick(assets: &[JsonAsset], version: &str) -> Option<Asset> {
    let stem = format!("waveshark-{version}-{}", platform());
    wanted().into_iter().find_map(|(suffix, kind)| {
        let name = format!("{stem}{suffix}");
        let a = assets.iter().find(|a| a.name == name)?;
        Some(Asset {
            name: a.name.clone(),
            url: a.browser_download_url.clone(),
            bytes: a.size,
            kind,
        })
    })
}

/// The suffixes this platform can install from, best first. The last is
/// always the bare binary, which every release carries; on Windows that is
/// the zip, because the `.exe` alone will not start without the two DLLs
/// published inside it.
fn wanted() -> Vec<(&'static str, Kind)> {
    if cfg!(target_os = "windows") {
        vec![(".msi", Kind::Installer), (".zip", Kind::Binary)]
    } else if cfg!(target_os = "macos") {
        vec![(".dmg", Kind::Installer), ("", Kind::Binary)]
    } else {
        let mut v = Vec::new();
        // Which package a Linux machine can install is not a compile-time
        // answer: one binary runs on both families, so ask the machine.
        match family() {
            Family::Debian => v.push((".deb", Kind::Installer)),
            Family::Redhat => v.push((".rpm", Kind::Installer)),
            Family::Other => {}
        }
        v.push(("", Kind::Binary));
        v
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Family {
    Debian,
    Redhat,
    Other,
}

/// Which packaging family this machine uses, read from `/etc/os-release`.
///
/// `ID_LIKE` as well as `ID`, because a derivative names its parent there and
/// there are far more derivatives than parents. Anything else is `Other`,
/// which offers the archive rather than a package that would not install.
fn family() -> Family {
    let Ok(text) = std::fs::read_to_string("/etc/os-release") else {
        return Family::Other;
    };
    let ids = text
        .lines()
        .filter_map(|l| l.split_once('='))
        .filter(|(k, _)| *k == "ID" || *k == "ID_LIKE")
        .flat_map(|(_, v)| v.trim_matches('"').split_whitespace())
        .collect::<Vec<_>>();
    if ids.iter().any(|id| matches!(*id, "debian" | "ubuntu" | "raspbian")) {
        Family::Debian
    } else if ids.iter().any(|id| matches!(*id, "rhel" | "fedora" | "centos" | "suse" | "opensuse"))
    {
        Family::Redhat
    } else {
        Family::Other
    }
}

/// The name the build workflow gives this platform's assets, which is what
/// picks them out of the release.
pub fn platform() -> &'static str {
    const OS: &str = if cfg!(target_os = "windows") {
        "windows"
    } else if cfg!(target_os = "macos") {
        "macos"
    } else {
        "linux"
    };
    const ARCH: &str = if cfg!(target_arch = "aarch64") { "arm64" } else { "x86_64" };
    // Two consts rather than a format!: the answer is fixed at compile time
    // and callers want a &'static str.
    match (OS, ARCH) {
        ("windows", "arm64") => "windows-arm64",
        ("windows", _) => "windows-x86_64",
        ("macos", "arm64") => "macos-arm64",
        ("macos", _) => "macos-x86_64",
        (_, "arm64") => "linux-arm64",
        _ => "linux-x86_64",
    }
}

/// Whether `latest` is a later version than `current`.
///
/// Dotted numbers compared left to right, with a missing component read as
/// zero so `0.2` and `0.2.0` are the same release. A suffix after `-` marks a
/// pre-release, which sorts below the plain version it names: `0.2.0-rc1` is
/// not an upgrade from `0.2.0`. Anything that will not parse is treated as
/// not newer, because offering an upgrade to a version that cannot be read is
/// worse than saying nothing.
fn is_newer(latest: &str, current: &str) -> bool {
    let (l, lpre) = split(latest);
    let (c, cpre) = split(current);
    for i in 0..l.len().max(c.len()) {
        let a = l.get(i).copied().unwrap_or(0);
        let b = c.get(i).copied().unwrap_or(0);
        if a != b {
            return a > b;
        }
    }
    // Same numbers: a release beats the pre-releases that led to it.
    cpre.is_some() && lpre.is_none()
}

fn split(v: &str) -> (Vec<u64>, Option<&str>) {
    let v = v.trim().trim_start_matches('v');
    let (nums, pre) = match v.split_once(['-', '+']) {
        Some((n, p)) => (n, Some(p)),
        None => (v, None),
    };
    let nums = nums.split('.').map(|p| p.trim().parse::<u64>().unwrap_or(0)).collect();
    (nums, pre)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_later_version_is_newer() {
        assert!(is_newer("0.2.0", "0.1.0"));
        assert!(is_newer("1.0.0", "0.9.9"));
        assert!(is_newer("0.1.1", "0.1.0"));
        assert!(is_newer("v0.2.0", "0.1.0"));
    }

    #[test]
    fn the_same_version_is_not_an_upgrade() {
        assert!(!is_newer("0.1.0", "0.1.0"));
        assert!(!is_newer("0.1", "0.1.0"));
        assert!(!is_newer("0.1.0", "0.2.0"));
    }

    #[test]
    fn a_pre_release_sorts_below_the_version_it_names() {
        assert!(!is_newer("0.2.0-rc1", "0.2.0"));
        assert!(is_newer("0.2.0", "0.2.0-rc1"));
        assert!(is_newer("0.2.0-rc1", "0.1.0"));
    }

    #[test]
    fn nonsense_is_not_an_upgrade() {
        // A tag that is not a version must not offer itself as one.
        assert!(!is_newer("nightly", "0.1.0"));
    }

    /// The version every fixture in these tests is named for.
    const V: &str = "0.3.1";

    /// Every asset a release carries, so the test runs the same on whichever
    /// platform and feature set built it.
    fn every_asset() -> Vec<JsonAsset> {
        [
            "waveshark-0.3.1-linux-x86_64",
            "waveshark-0.3.1-linux-x86_64.deb",
            "waveshark-0.3.1-linux-x86_64.rpm",
            "waveshark-0.3.1-linux-arm64",
            "waveshark-0.3.1-windows-x86_64.zip",
            "waveshark-0.3.1-windows-x86_64.msi",
            "waveshark-0.3.1-macos-arm64",
            "waveshark-0.3.1-macos-arm64.dmg",
        ]
        .iter()
        .enumerate()
        .map(|(i, name)| JsonAsset {
            name: (*name).to_string(),
            browser_download_url: format!("https://example.invalid/{name}"),
            size: 100 + i as u64,
        })
        .collect()
    }

    #[test]
    fn a_release_reads_its_tag_page_and_asset() {
        let body = r#"{
            "tag_name": "v0.3.1",
            "html_url": "https://github.com/v0l/waveshark/releases/tag/v0.3.1",
            "published_at": "2025-01-02T03:04:05Z",
            "assets": [
                {"name": "waveshark-0.3.1-linux-x86_64",
                 "browser_download_url": "https://example.invalid/l", "size": 12},
                {"name": "waveshark-0.3.1-windows-x86_64.zip",
                 "browser_download_url": "https://example.invalid/w.zip", "size": 34},
                {"name": "waveshark-0.3.1-macos-arm64",
                 "browser_download_url": "https://example.invalid/m", "size": 56}
            ]
        }"#;
        let r = parse(body).expect("parses");
        assert_eq!(r.version, "0.3.1");
        assert_eq!(r.tag, "v0.3.1");
        assert!(r.page.ends_with("v0.3.1"));
        let a = r.asset.expect("an asset for the platform this test runs on");
        // A release with no installers still offers the binary.
        assert_eq!(a.kind, Kind::Binary, "{}", a.name);
        assert_eq!(a.name, format!("waveshark-{V}-{}{}", platform(), binary_suffix()));
        assert!(a.bytes > 0);
    }

    fn binary_suffix() -> &'static str {
        if cfg!(target_os = "windows") { ".zip" } else { "" }
    }

    /// An asset of the release before this one must not be offered, which is
    /// what a match on the platform alone would do with two releases' files
    /// in one list.
    #[test]
    fn only_this_releases_assets_are_offered() {
        let mut assets = every_asset();
        for a in &mut assets {
            a.name = a.name.replace("0.3.1", "0.3.0");
        }
        assert!(pick(&assets, V).is_none());
    }

    #[test]
    fn an_installer_is_preferred_to_the_binary() {
        let a = pick(&every_asset(), V).expect("an asset for this platform");
        // Every platform the workflow builds an installer for gets it; a
        // Linux machine of neither packaging family gets the binary.
        let installed = cfg!(target_os = "windows")
            || cfg!(target_os = "macos")
            || (cfg!(target_os = "linux") && family() != Family::Other);
        let expected = if installed { Kind::Installer } else { Kind::Binary };
        assert_eq!(a.kind, expected, "{}", a.name);
        assert!(a.name.starts_with(&format!("waveshark-{V}-{}", platform())), "{}", a.name);
    }

    #[test]
    fn another_platforms_asset_is_never_offered() {
        // The names share a prefix, so a match on the start of the name would
        // pick whichever GitHub listed first.
        let chosen = pick(&every_asset(), V).expect("an asset");
        let rest = chosen.name.trim_start_matches(&format!("waveshark-{V}-{}", platform()));
        assert!(rest.is_empty() || rest.starts_with('.'), "{}", chosen.name);
    }

    #[test]
    fn a_release_carrying_only_the_other_platforms_offers_nothing() {
        let assets = vec![JsonAsset {
            name: "waveshark-0.3.1-solaris-sparc".into(),
            browser_download_url: "https://example.invalid/s".into(),
            size: 1,
        }];
        assert!(pick(&assets, V).is_none());
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn the_packaging_family_is_read_from_os_release() {
        // Whatever this machine is, the answer has to be one of the three and
        // has to agree with what is offered.
        let f = family();
        let wants_deb = wanted().iter().any(|(s, _)| *s == ".deb");
        let wants_rpm = wanted().iter().any(|(s, _)| *s == ".rpm");
        match f {
            Family::Debian => assert!(wants_deb && !wants_rpm),
            Family::Redhat => assert!(wants_rpm && !wants_deb),
            Family::Other => assert!(!wants_deb && !wants_rpm),
        }
        // The binary is always the last resort.
        assert_eq!(wanted().last().map(|(s, _)| *s), Some(""));
    }

    #[test]
    fn a_release_without_this_platform_still_reads() {
        let body = r#"{"tag_name": "v9.0.0", "html_url": "", "published_at": "",
                       "assets": [{"name": "waveshark-9.0.0-solaris-sparc",
                                   "browser_download_url": "https://example.invalid/s", "size": 1}]}"#;
        let r = parse(body).expect("parses");
        assert_eq!(r.version, "9.0.0");
        assert!(r.asset.is_none());
    }

    #[test]
    fn a_body_that_is_not_a_release_is_an_error() {
        // A rate-limited API answers 403 with a message object, and that must
        // not read as a version.
        assert!(parse(r#"{"message": "API rate limit exceeded"}"#).is_err());
        assert!(parse("not json").is_err());
    }
}
