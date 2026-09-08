//! What the newest published release is, and whether this build is behind it.
//!
//! The releases the build workflow publishes are the only statement of what
//! the current version is: the crate version in a running binary says what it
//! was built from, not what is available. GitHub answers that in one request
//! (`/releases/latest`, which skips drafts and pre-releases), so the check is
//! a single unauthenticated GET and a version comparison.
//!
//! Nothing here downloads or replaces anything. It reports, and the pane that
//! draws it offers the release page.

use parking_lot::RwLock;
use serde::Deserialize;
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
    /// The archive for this platform, when the release carries one. Absent
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
    let asset = j
        .assets
        .into_iter()
        .find(|a| a.name.starts_with(&format!("waveshark-{}", platform())))
        .map(|a| Asset { name: a.name, url: a.browser_download_url, bytes: a.size });
    Ok(Release { version, tag: j.tag_name, page: j.html_url, asset, published: j.published_at })
}

/// The name the build workflow gives this platform's archive, which is what
/// picks one asset out of the release.
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

    #[test]
    fn a_release_reads_its_tag_page_and_asset() {
        let body = r#"{
            "tag_name": "v0.3.1",
            "html_url": "https://github.com/v0l/waveshark/releases/tag/v0.3.1",
            "published_at": "2025-01-02T03:04:05Z",
            "assets": [
                {"name": "waveshark-linux-x86_64.tar.gz",
                 "browser_download_url": "https://example.invalid/l.tar.gz", "size": 12},
                {"name": "waveshark-windows-x86_64.zip",
                 "browser_download_url": "https://example.invalid/w.zip", "size": 34},
                {"name": "waveshark-macos-arm64.tar.gz",
                 "browser_download_url": "https://example.invalid/m.tar.gz", "size": 56}
            ]
        }"#;
        let r = parse(body).expect("parses");
        assert_eq!(r.version, "0.3.1");
        assert_eq!(r.tag, "v0.3.1");
        assert!(r.page.ends_with("v0.3.1"));
        let a = r.asset.expect("an asset for the platform this test runs on");
        assert!(a.name.starts_with(&format!("waveshark-{}", platform())), "{}", a.name);
        assert!(a.bytes > 0);
    }

    #[test]
    fn a_release_without_this_platform_still_reads() {
        let body = r#"{"tag_name": "v9.0.0", "html_url": "", "published_at": "",
                       "assets": [{"name": "waveshark-solaris-sparc.tar.gz",
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
