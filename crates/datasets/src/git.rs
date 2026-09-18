//! A git repository as a dataset: the working tree of a branch, extracted
//! into the cache and browsable as files.
//!
//! The datasets elsewhere in this crate are one file each. A repository of
//! scripts is a *tree*: what makes it worth fetching is the directory
//! structure, and a browser wants to walk it on disk rather than through an
//! index. So a [`Repo`] here resolves the branch's HEAD commit, downloads
//! the tarball a forge serves of it, and unpacks the regular files under
//! `cache/<name>/` with the commit id beside them for validation.
//!
//! The HEAD commit is the whole freshness story: one API call answers
//! whether the tree moved, and only a move downloads anything. That is the
//! same bargain the entity tags of a single file make, expressed the way a
//! forge can answer it.
//!
//! Unpacking is deliberately narrow. Only regular files and directories are
//! written: a symlink or a device node in a tarball is a route out of the
//! cache directory, and no dataset here needs one. Paths that escape the
//! extraction root by spelling (`..`, an absolute path) are refused rather
//! than interpreted.

use crate::cache::{Error, When};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use httpc::USER_AGENT as AGENT;

/// One repository arrives in one response; a limit well above that guards
/// against a redirect to something else entirely rather than bounding
/// anything real.
const MAX_BYTES: u64 = 1 << 30;

/// How long a repo's HEAD is worth between checks. A repository of scripts
/// moves when somebody adds one, not hourly.
pub const MAX_AGE: Duration = Duration::from_secs(24 * 3600);

/// One repository, as a row the settings pane offers.
///
/// The forge specifics are kept to two URLs: `tarball`, which must serve a
/// gzipped tar of the branch, and `head`, which must answer the branch's
/// current commit id. For GitHub those are a codeload URL and the branches
/// API; the row spells them out rather than the code guessing a forge from
/// a hostname, because a forge's API is a publisher's decision to change
/// and a row is where a publisher is described.
#[derive(Debug)]
pub struct Repo {
    /// What the repository is called, for the row and the browser.
    pub name: &'static str,
    /// Directory name under the cache, and the metadata key.
    pub dir: &'static str,
    /// Where the branch's HEAD commit id is answered. Expected to return
    /// JSON with `commit.sha`.
    pub head: &'static str,
    /// The gzipped tar of the branch, served whole.
    pub tarball: &'static str,
    /// Who publishes it, for the line under the name.
    pub publisher: &'static str,
    /// What the repository holds, for the pane that offers to download it.
    pub about: &'static str,
    /// The page a person can read rather than the tarball.
    pub page: &'static str,
    /// The terms the publisher states for the content.
    pub terms: &'static str,
    pub max_age: Duration,
    /// Extensions worth keeping, without the dot, or empty for the whole
    /// tree. A repository of captures is mostly not captures: the largest
    /// of these is two gigabytes of firmware, photographs and documents
    /// around eleven thousand `.sub` files that come to a few megabytes.
    /// The tarball is served whole either way; this is what lands on disk.
    pub keep: &'static [&'static str],
    /// What the compressed tarball may run to. Sized per repository rather
    /// than once for all of them, because refusing the one collection
    /// everybody has is not a safe default, and a gigabyte is plenty for
    /// the rest.
    pub max_bytes: u64,
}

impl Repo {
    /// The directory the tree is unpacked into, when it has been.
    pub fn cache_dir(&'static self, cache: &crate::cache::Cache) -> PathBuf {
        cache.dir().join("git").join(self.dir)
    }

    /// The metadata file beside the extracted tree.
    fn meta_file(&'static self, cache: &crate::cache::Cache) -> PathBuf {
        cache.dir().join("git").join(format!("{}.meta.json", self.dir))
    }
}

impl PartialEq for Repo {
    fn eq(&self, other: &Self) -> bool {
        self.dir == other.dir
    }
}

impl Eq for Repo {}

/// What is recorded beside an extracted tree. The shape of
/// `cache::Meta` for a directory, with the same meanings.
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
struct Meta {
    /// The HEAD commit the tree was unpacked from, which is what goes back
    /// to the forge to ask whether it moved.
    #[serde(default)]
    commit: Option<String>,
    /// Files written, which is what says the extraction completed rather
    /// than died halfway: an incomplete tree parses to nothing rather than
    /// to a wrong answer, but it would otherwise be revalidated forever.
    #[serde(default)]
    files: u64,
    /// What those files come to on disc. Zero in a tree unpacked by an
    /// older build, which reads it back off the directory once.
    #[serde(default)]
    bytes: u64,
    /// Unix seconds at the last successful check, new bytes or not.
    checked: u64,
    /// Why the last attempt was refused, when it was. The same rule as a
    /// single-file source: a publisher that answers anything but a 200 is
    /// telling this program to stop asking.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    refused: Option<String>,
}

/// What is held for one repository, for the row that reports it.
#[derive(Clone, Debug, Default)]
pub struct Status {
    /// The commit the tree is at, when a tree is held.
    pub commit: Option<String>,
    pub files: Option<u64>,
    /// What the tree comes to on disc.
    pub bytes: Option<u64>,
    /// Unix seconds at the last successful check.
    pub checked: Option<u64>,
}

/// What the repository holds on this machine, when a tree has landed.
pub struct Tree {
    /// Where the files are.
    pub dir: PathBuf,
    /// The commit it was unpacked from.
    pub commit: String,
}

/// The tree, downloading it if it is not held yet. Blocks on the network,
/// so it belongs off the UI thread like every other dataset load.
pub fn get(repo: &'static Repo, cache: &crate::cache::Cache) -> Result<Tree, Error> {
    if let Some(t) = held(repo, cache) {
        return Ok(t);
    }
    match fetch(repo, cache, &None)? {
        Some(t) => Ok(t),
        None => Err(Error::Fetch(repo.tarball.into(), "unchanged, but nothing is held".into())),
    }
}

/// Read what is held, then ask whether it moved.
pub fn refresh(
    repo: &'static Repo,
    cache: &crate::cache::Cache,
    when: When,
) -> Result<Option<Tree>, Error> {
    let meta = read_meta(repo, cache);
    if let Some(m) = &meta {
        if when == When::IfDue {
            if let Some(why) = &m.refused {
                return Err(Error::Fetch(repo.dir.into(), format!("halted: {why}")));
            }
            let due = now().saturating_sub(m.checked) >= repo.max_age.as_secs();
            if !due && m.commit.is_some() {
                return Ok(None);
            }
        }
    }
    let have = meta.and_then(|m| m.commit);
    let got = match fetch(repo, cache, &have) {
        Ok(got) => got,
        Err(e) => {
            remember_refused(repo, cache, Some(e.to_string()));
            return Err(e);
        }
    };
    if got.is_none() {
        touch(repo, cache);
    }
    Ok(got)
}

/// The tree on disk, if a complete one is held.
pub fn held(repo: &'static Repo, cache: &crate::cache::Cache) -> Option<Tree> {
    let m = read_meta(repo, cache)?;
    let dir = repo.cache_dir(cache);
    let commit = m.commit?;
    (m.files > 0).then(|| Tree { dir, commit })
}

/// What is held and when it was checked, for the row that reports it.
pub fn status(repo: &'static Repo, cache: &crate::cache::Cache) -> Status {
    let mut m = read_meta(repo, cache);
    // A tree unpacked before the size was recorded: measure it once and
    // write it down, rather than walking twenty thousand files every time
    // the settings pane draws a frame.
    if let Some(meta) = m.as_mut()
        && meta.files > 0
        && meta.bytes == 0
    {
        meta.bytes = tree_bytes(&repo.cache_dir(cache));
        write_meta(repo, cache, meta);
    }
    Status {
        commit: m.as_ref().and_then(|m| m.commit.clone()),
        files: m.as_ref().map(|m| m.files),
        bytes: m.as_ref().map(|m| m.bytes),
        checked: m.as_ref().map(|m| m.checked),
    }
}

/// What a tree occupies, added up from the files themselves.
fn tree_bytes(dir: &Path) -> u64 {
    let Ok(entries) = std::fs::read_dir(dir) else { return 0 };
    let mut total = 0;
    for e in entries.flatten() {
        let p = e.path();
        match e.metadata() {
            Ok(m) if m.is_dir() => total += tree_bytes(&p),
            Ok(m) => total += m.len(),
            Err(_) => {}
        }
    }
    total
}

/// Files under the tree with this extension, relative to the extraction
/// root, in path order. The question a scripts browser asks; generic here
/// because it is a question about a tree, not about any one kind of script.
///
/// `ext` is taken with or without its dot and matched regardless of case: a
/// repository of captures traded between people has `.sub` and `.SUB` in it,
/// and a caller asking for ".sub" and getting nothing is the kind of empty
/// list nobody reads as a bug.
pub fn files_with(repo: &'static Repo, cache: &crate::cache::Cache, ext: &str) -> Vec<String> {
    let Some(t) = held(repo, cache) else { return Vec::new() };
    let mut out = Vec::new();
    walk(&t.dir, &t.dir, ext.trim_start_matches('.'), &mut out);
    out.sort();
    out
}

fn walk(root: &Path, dir: &Path, ext: &str, out: &mut Vec<String>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for e in entries.flatten() {
        let p = e.path();
        if p.is_dir() {
            walk(root, &p, ext, out);
        } else if p.extension().is_some_and(|x| x.eq_ignore_ascii_case(ext))
            && let Ok(rel) = p.strip_prefix(root)
        {
            out.push(rel.display().to_string());
        }
    }
}

fn fetch(
    repo: &'static Repo,
    cache: &crate::cache::Cache,
    have: &Option<String>,
) -> Result<Option<Tree>, Error> {
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .user_agent(AGENT)
        .http_status_as_error(false)
        .timeout_global(Some(Duration::from_secs(600)))
        .build()
        .into();
    let commit = head_commit(repo, &agent)?;
    if have.as_deref() == Some(commit.as_str()) {
        return Ok(None);
    }
    // Unpack into a temporary beside the target, then swap, so a run killed
    // mid-download leaves the previous tree rather than half of a new one.
    let git = cache.dir().join("git");
    let dir = repo.cache_dir(cache);
    let tmp = git.join(format!("{}.part", repo.dir));
    std::fs::create_dir_all(&git).map_err(|e| Error::Io(git.display().to_string(), e))?;
    let _ = std::fs::remove_dir_all(&tmp);
    let mut resp = agent
        .get(repo.tarball)
        .call()
        .map_err(|e| Error::Fetch(repo.tarball.into(), e.to_string()))?;
    if resp.status().as_u16() != 200 {
        return Err(Error::Status(repo.tarball.into(), resp.status().as_u16()));
    }
    // Counted on the compressed side, where the length the forge declared
    // applies: what comes out of the decoder is several times what came
    // down the wire, and a bar past its end reads as a fault.
    let progress = crate::progress::of(repo.dir);
    progress.start();
    if let Some(n) = resp
        .headers()
        .get("content-length")
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.parse().ok())
    {
        progress.expect(n);
    }
    let body = resp.body_mut().with_config().limit(repo.max_bytes.max(MAX_BYTES)).reader();
    let mut gz = flate2::read::GzDecoder::new(crate::progress::Tapped { inner: body, progress });
    let files = match unpack_kept(&mut gz, &tmp, repo.keep) {
        Ok(files) => {
            progress.stop();
            files
        }
        Err(e) => {
            progress.stop();
            return Err(e);
        }
    };
    if files == 0 {
        let _ = std::fs::remove_dir_all(&tmp);
        return Err(Error::Parse(repo.dir.into(), "the tarball holds no files".into()));
    }
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::rename(&tmp, &dir).map_err(|e| Error::Io(dir.display().to_string(), e))?;
    let meta = Meta {
        commit: Some(commit.clone()),
        files,
        bytes: tree_bytes(&dir),
        checked: now(),
        refused: None,
    };
    write_meta(repo, cache, &meta);
    tracing::info!(repo = repo.dir, files, from = %repo.tarball, "repository downloaded");
    Ok(Some(Tree { dir, commit }))
}

/// The branch's current commit id, from the `head` URL's JSON.
fn head_commit(repo: &'static Repo, agent: &ureq::Agent) -> Result<String, Error> {
    let mut resp =
        agent.get(repo.head).call().map_err(|e| Error::Fetch(repo.head.into(), e.to_string()))?;
    if resp.status().as_u16() != 200 {
        return Err(Error::Status(repo.head.into(), resp.status().as_u16()));
    }
    let body = resp
        .body_mut()
        .read_to_string()
        .map_err(|e| Error::Fetch(repo.head.into(), e.to_string()))?;
    let v: serde_json::Value =
        serde_json::from_str(&body).map_err(|e| Error::Parse(repo.head.into(), e.to_string()))?;
    v["commit"]["sha"]
        .as_str()
        .map(str::to_string)
        .ok_or_else(|| Error::Parse(repo.head.into(), "no commit sha".into()))
}

/// Unpack a tar stream into `to`, writing regular files and directories
/// only. Returns how many files landed.
///
/// The tar header format as GNU and bsdtar write it: 512-byte records, an
/// octal size, typeflags `0` and `5` for files and directories, and a pair
/// of zero records at the end. Anything else in the stream is skipped
/// rather than interpreted, because a link is a route out of the
/// extraction root and no dataset here needs one.
/// Unpack, writing only the files whose extension is in `keep`. An empty
/// `keep` writes everything.
fn unpack_kept(gz: &mut dyn Read, to: &Path, keep: &[&str]) -> Result<u64, Error> {
    std::fs::create_dir_all(to).map_err(|e| Error::Io(to.display().to_string(), e))?;
    let mut header = [0u8; 512];
    let mut buf = Vec::new();
    let mut files = 0u64;
    let mut prefix = String::new();
    loop {
        if read_exact_or_end(gz, &mut header)? == 0 {
            return Ok(files);
        }
        if header.iter().all(|&b| b == 0) {
            continue;
        }
        let name = str_field(&header[0..100]);
        // pax extended headers carry the real name in their payload and a
        // prefix of their own in the next header; the common case here is a
        // repo tree, whose paths fit the ustar fields without either.
        let typeflag = header[156];
        let size = octal(&header[124..136]);
        let payload = size as usize;
        if typeflag == b'x' || typeflag == b'g' {
            // An extended header: read it, remember any path override, and
            // let it apply to the header that follows.
            take(gz, payload, &mut buf)?;
            if let Some(n) = pax_path(&buf) {
                prefix = n;
            }
            continue;
        }
        let dir = match typeflag {
            b'5' => {
                take(gz, payload, &mut buf)?;
                Some(join_under(to, &rel_path(&name, &prefix, &header))?)
            }
            b'0' | 0 => None,
            // Anything else - links, devices, fifos - is skipped along with
            // its payload: none of it belongs in a dataset.
            _ => {
                take(gz, payload, &mut buf)?;
                continue;
            }
        };
        let path = join_under(to, &rel_path(&name, &prefix, &header))?;
        if let Some(d) = dir {
            std::fs::create_dir_all(&d).map_err(|e| Error::Io(d.display().to_string(), e))?;
            prefix.clear();
            continue;
        }
        take(gz, payload, &mut buf)?;
        // Read whatever it is, so the stream stays in step, and write only
        // what was asked for.
        let wanted = keep.is_empty()
            || path.extension().is_some_and(|e| keep.iter().any(|k| e.eq_ignore_ascii_case(k)));
        if !wanted {
            prefix.clear();
            continue;
        }
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .map_err(|e| Error::Io(parent.display().to_string(), e))?;
        }
        std::fs::write(&path, &buf).map_err(|e| Error::Io(path.display().to_string(), e))?;
        files += 1;
        prefix.clear();
    }
}

/// A file's path, from the ustar name field, the pax override if one came,
/// and the ustar prefix field. The first path component is the archive's
/// own root (`repo-branch/`), which is dropped: the extraction root is the
/// cache directory, not the repository's checkout name.
fn rel_path(name: &str, pax: &str, header: &[u8; 512]) -> String {
    let full = if !pax.is_empty() {
        pax.to_string()
    } else {
        let prefix = str_field(&header[345..500]);
        if prefix.is_empty() { name.to_string() } else { format!("{prefix}/{name}") }
    };
    full.trim_start_matches('/')
        .split_once('/')
        .map(|(_, rest)| rest.to_string())
        .unwrap_or_default()
}

/// `to` joined with `rel`, refusing any spelling that leaves `to`.
fn join_under(to: &Path, rel: &str) -> Result<PathBuf, Error> {
    let rel = Path::new(rel);
    if rel.is_absolute() || rel.components().any(|c| c == std::path::Component::ParentDir) {
        return Err(Error::Parse(
            rel.display().to_string(),
            "a path out of the extraction root".into(),
        ));
    }
    Ok(to.join(rel))
}

/// The path a pax extended header declares, if it does.
fn pax_path(payload: &[u8]) -> Option<String> {
    let text = std::str::from_utf8(payload).ok()?;
    for line in text.split('\n') {
        let rest = line.trim_end_matches('\r');
        if let Some((len, field)) = rest.split_once(' ') {
            if len.parse::<usize>().is_ok() && field.starts_with("path=") {
                return Some(field[5..].to_string());
            }
        }
    }
    None
}

fn str_field(f: &[u8]) -> String {
    let end = f.iter().position(|&b| b == 0).unwrap_or(f.len());
    String::from_utf8_lossy(&f[..end]).trim().to_string()
}

fn octal(f: &[u8]) -> u64 {
    let s = str_field(f);
    u64::from_str_radix(&s, 8).unwrap_or(0)
}

fn read_exact_or_end(r: &mut dyn Read, out: &mut [u8]) -> Result<usize, Error> {
    let mut got = 0;
    while got < out.len() {
        match r.read(&mut out[got..]) {
            Ok(0) => return Ok(got),
            Ok(n) => got += n,
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => return Err(Error::Io("tarball".into(), e)),
        }
    }
    Ok(got)
}

fn take(r: &mut dyn Read, n: usize, buf: &mut Vec<u8>) -> Result<(), Error> {
    buf.clear();
    buf.resize(n, 0);
    if n == 0 {
        return Ok(());
    }
    read_exact_or_end(r, buf)?;
    // Padding to the next 512-byte record.
    let pad = (512 - n % 512) % 512;
    let mut skip = vec![0u8; pad];
    read_exact_or_end(r, &mut skip)?;
    Ok(())
}

fn read_meta(repo: &'static Repo, cache: &crate::cache::Cache) -> Option<Meta> {
    let raw = std::fs::read(repo.meta_file(cache)).ok()?;
    serde_json::from_slice(&raw).ok()
}

fn write_meta(repo: &'static Repo, cache: &crate::cache::Cache, meta: &Meta) {
    if let Ok(raw) = serde_json::to_vec(meta) {
        let _ = std::fs::write(repo.meta_file(cache), raw);
    }
}

fn remember_refused(repo: &'static Repo, cache: &crate::cache::Cache, why: Option<String>) {
    let Some(mut m) = read_meta(repo, cache) else { return };
    if m.refused == why {
        return;
    }
    m.refused = why;
    write_meta(repo, cache, &m);
}

fn touch(repo: &'static Repo, cache: &crate::cache::Cache) {
    let Some(mut m) = read_meta(repo, cache) else { return };
    m.checked = now();
    write_meta(repo, cache, &m);
}

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// The repositories published as script collections. A row each, like the
/// gateway host files: each is published, refreshed and credited on its
/// own.
pub static NULLSEC: Repo = Repo {
    name: "NullSec Flipper Suite",
    dir: "nullsec-flipper-suite",
    head: "https://api.github.com/repos/bad-antics/nullsec-flipper-suite/branches/main",
    tarball: "https://codeload.github.com/bad-antics/nullsec-flipper-suite/tar.gz/refs/heads/main",
    publisher: "github.com/bad-antics",
    about: "Sub-GHz captures and key files for gate, garage and access \
     remotes, collected as Flipper .sub files. This is a catalogue of \
     signals to recognise and replay; what each one opens is its owner's \
     question to answer.",
    page: "https://github.com/bad-antics/nullsec-flipper-suite",
    terms: "MIT (the repository); each capture is its transmitter's",
    max_age: MAX_AGE,
    keep: SUB,
    max_bytes: MAX_BYTES,
};

/// What a sub-GHz collection is kept for. The rest of these repositories is
/// infrared, NFC, badge scripts, firmware and photographs, none of which
/// this receiver can key.
const SUB: &[&str] = &["sub"];

pub static UBERGUIDOZ: Repo = Repo {
    name: "UberGuidoZ Flipper",
    dir: "uberguidoz-flipper",
    head: "https://api.github.com/repos/UberGuidoZ/Flipper/branches/main",
    tarball: "https://codeload.github.com/UberGuidoZ/Flipper/tar.gz/refs/heads/main",
    publisher: "github.com/UberGuidoZ",
    about: "The largest collected Flipper playground: some eleven thousand \
     .sub captures for gates, garages, barriers, shutters and remotes, by \
     make and model. A two gigabyte download, of which the captures are a \
     few megabytes; the rest is not kept.",
    page: "https://github.com/UberGuidoZ/Flipper/tree/main/Sub-GHz",
    terms: "GPL-3.0 (the repository); each capture is its transmitter's",
    max_age: MAX_AGE,
    keep: SUB,
    // Two and a bit gigabytes compressed, and growing.
    max_bytes: 4 << 30,
};

pub static ZERO_SPLOIT: Repo = Repo {
    name: "Zero-Sploit SubGHz DB",
    dir: "zero-sploit-subghz-db",
    head: "https://api.github.com/repos/Zero-Sploit/FlipperZero-Subghz-DB/branches/main",
    tarball: "https://codeload.github.com/Zero-Sploit/FlipperZero-Subghz-DB/tar.gz/refs/heads/main",
    publisher: "github.com/Zero-Sploit",
    about: "Thirteen thousand .sub captures filed by manufacturer and \
     model: the widest catalogue of gate and barrier remotes in one tree.",
    page: "https://github.com/Zero-Sploit/FlipperZero-Subghz-DB",
    terms: "no licence stated; each capture is its transmitter's",
    max_age: MAX_AGE,
    keep: SUB,
    max_bytes: MAX_BYTES,
};

pub static ROCKETGOD: Repo = Repo {
    name: "RocketGod Flipper Zero",
    dir: "rocketgod-flipper-zero",
    head: "https://api.github.com/repos/RocketGod-git/Flipper_Zero/branches/main",
    tarball: "https://codeload.github.com/RocketGod-git/Flipper_Zero/tar.gz/refs/heads/main",
    publisher: "github.com/RocketGod-git",
    about: "A mixed Flipper collection whose sub-GHz half holds several \
     thousand captures, including X10 mains switches and a set of test \
     tones. A one and a half gigabyte download; only the captures are \
     kept.",
    page: "https://github.com/RocketGod-git/Flipper_Zero/tree/main/subghz",
    terms: "no licence stated; each capture is its transmitter's",
    max_age: MAX_AGE,
    keep: SUB,
    max_bytes: 3 << 30,
};

pub static MUDDLEDBOX: Repo = Repo {
    name: "MuddledBox Sub-GHz",
    dir: "muddledbox-subghz",
    head: "https://api.github.com/repos/MuddledBox/FlipperZeroSub-GHz/branches/main",
    tarball: "https://codeload.github.com/MuddledBox/FlipperZeroSub-GHz/tar.gz/refs/heads/main",
    publisher: "github.com/MuddledBox",
    about: "A short, checked set: the Tesla charge port opener at both \
     bandwidths, and a handful of vehicle and gate remotes.",
    page: "https://github.com/MuddledBox/FlipperZeroSub-GHz",
    terms: "no licence stated; each capture is its transmitter's",
    max_age: MAX_AGE,
    keep: SUB,
    max_bytes: MAX_BYTES,
};

pub static TOUCHTUNES: Repo = Repo {
    name: "TouchTunes remotes",
    dir: "flipperzero-touchtunes",
    head: "https://api.github.com/repos/jimilinuxguy/flipperzero-touchtunes/branches/master",
    tarball: "https://codeload.github.com/jimilinuxguy/flipperzero-touchtunes/tar.gz/refs/heads/master",
    publisher: "github.com/jimilinuxguy",
    about: "The TouchTunes jukebox remote at 433.92 MHz, and a generated \
     sweep of every address it accepts. Eight thousand files, nearly all \
     of them one key each of that sweep.",
    page: "https://github.com/jimilinuxguy/flipperzero-touchtunes",
    terms: "GPL-3.0 (the repository); each capture is its transmitter's",
    max_age: MAX_AGE,
    keep: SUB,
    max_bytes: MAX_BYTES,
};

pub static EVILPETE: Repo = Repo {
    name: "evilpete Flipper toolbox",
    dir: "evilpete-flipper-toolbox",
    head: "https://api.github.com/repos/evilpete/flipper_toolbox/branches/main",
    tarball: "https://codeload.github.com/evilpete/flipper_toolbox/tar.gz/refs/heads/main",
    publisher: "github.com/evilpete",
    about: "Tools for converting captures into Flipper files, with a short \
     set of X10 mains-switch commands beside them.",
    page: "https://github.com/evilpete/flipper_toolbox",
    terms: "BSD-3-Clause (the repository); each capture is its transmitter's",
    max_age: MAX_AGE,
    keep: SUB,
    max_bytes: MAX_BYTES,
};

pub static FLIPPER_PLAYLIST: Repo = Repo {
    name: "flipper-playlist test set",
    dir: "flipper-playlist",
    head: "https://api.github.com/repos/darmiel/flipper-playlist/branches/feat%2Fplaylist",
    tarball: "https://codeload.github.com/darmiel/flipper-playlist/tar.gz/refs/heads/feat/playlist",
    publisher: "github.com/darmiel",
    about: "The Flipper firmware's own sub-GHz unit tests: one file per \
     protocol, each a known key. Useful for checking a decoder rather than \
     for opening anything.",
    page: "https://github.com/darmiel/flipper-playlist",
    terms: "GPL-3.0 (the repository)",
    max_age: MAX_AGE,
    keep: SUB,
    max_bytes: MAX_BYTES,
};

/// The protocol descriptions the receiver reads sensors and remotes with,
/// published apart from the build so a fixed or added layout reaches a
/// receiver without a release. The same files are built in, so a receiver
/// that never fetches reads what it shipped with.
pub static PROTOCOLS: Repo = Repo {
    name: "Protocol descriptions",
    dir: "waveshark-protocols",
    head: "https://api.github.com/repos/v0l/waveshark-protocols/branches/main",
    tarball: "https://codeload.github.com/v0l/waveshark-protocols/tar.gz/refs/heads/main",
    publisher: "github.com/v0l",
    about: "One YAML file per sensor or remote: its timing, its frame and \
     its fields, read both ways. What the receiver decodes the ISM bands \
     with, kept up to date between releases.",
    page: "https://github.com/v0l/waveshark-protocols",
    terms: "MIT",
    max_age: MAX_AGE,
    keep: &["yaml"],
    max_bytes: MAX_BYTES,
};

pub static REPOS: &[&Repo] = &[
    &PROTOCOLS,
    &NULLSEC,
    &UBERGUIDOZ,
    &ZERO_SPLOIT,
    &ROCKETGOD,
    &MUDDLEDBOX,
    &TOUCHTUNES,
    &EVILPETE,
    &FLIPPER_PLAYLIST,
];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cache::Cache;
    use std::io::Write as _;

    fn tmpdir(name: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("waveshark-git-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        d
    }

    /// A tarball small enough to read at a glance: a root directory, a
    /// subdirectory, two files, a zero-record end.
    fn tarball(root: &str) -> Vec<u8> {
        let mut out = Vec::new();
        let mut entry = |name: &str, ty: u8, data: &[u8], out: &mut Vec<u8>| {
            let mut h = [0u8; 512];
            h[..name.len()].copy_from_slice(name.as_bytes());
            h[100] = b'0';
            h[100..108].copy_from_slice(b"0000644\0");
            h[108..116].copy_from_slice(b"0000000\0");
            h[116..124].copy_from_slice(b"0000000\0");
            h[124..136].copy_from_slice(format!("{:011o}\0", data.len()).as_bytes());
            h[156] = ty;
            h[257..263].copy_from_slice(b"ustar\0");
            out.extend_from_slice(&h);
            out.extend_from_slice(data);
            let pad = (512 - data.len() % 512) % 512;
            out.extend(std::iter::repeat_n(0u8, pad));
        };
        entry(root, b'5', &[], &mut out);
        entry(&format!("{root}/sub"), b'5', &[], &mut out);
        entry(&format!("{root}/sub/a.sub"), b'0', b"a".as_slice(), &mut out);
        entry(&format!("{root}/b.sub"), b'0', b"b".as_slice(), &mut out);
        entry(&format!("{root}/skip.txt"), b'0', b"x".as_slice(), &mut out);
        // A link, which must not be written.
        let mut h = [0u8; 512];
        h[..9].copy_from_slice(b"../escape");
        h[156] = b'2';
        h[257..263].copy_from_slice(b"ustar\0");
        out.extend_from_slice(&h);
        out.extend_from_slice(&[0u8; 512]);
        out
    }

    fn gzipped(root: &str) -> Vec<u8> {
        let raw = tarball(root);
        let mut gz = Vec::new();
        let mut e = flate2::write::GzEncoder::new(&mut gz, flate2::Compression::default());
        e.write_all(&raw).unwrap();
        e.finish().unwrap();
        gz
    }

    fn repo_at(dir: &Path) -> &'static Repo {
        // A repo whose URLs point nowhere the tests can reach; the
        // extraction and the walk are what is under test, fed directly.
        let _ = dir;
        &NULLSEC
    }

    #[test]
    fn a_tarball_unpacks_to_its_files_and_nothing_else() {
        let d = tmpdir("unpack");
        let to = d.join("tree");
        let mut tar = Vec::new();
        flate2::read::GzDecoder::new(gzipped("repo-main").as_slice())
            .read_to_end(&mut tar)
            .unwrap();
        let mut r = tar.as_slice();
        let n = unpack_kept(&mut r, &to, &[]).unwrap();
        assert_eq!(n, 3, "two .sub files and the .txt");
        assert_eq!(std::fs::read(to.join("sub/a.sub")).unwrap(), b"a");
        assert_eq!(std::fs::read(to.join("b.sub")).unwrap(), b"b");
        // The symlink was skipped, and nothing was written through it.
        assert!(!to.join("escape").exists());
        assert!(!to.join("repo-main").exists(), "the archive root is not a directory of the tree");
    }

    /// The whole tarball is served whatever is wanted from it, so what a
    /// repository costs on disk is what the filter lets through: the
    /// largest of these is two gigabytes around a few megabytes of
    /// captures.
    #[test]
    fn only_the_extensions_asked_for_are_written() {
        let d = tmpdir("keep");
        let to = d.join("tree");
        let mut tar = Vec::new();
        flate2::read::GzDecoder::new(gzipped("repo-main").as_slice())
            .read_to_end(&mut tar)
            .unwrap();
        let mut r = tar.as_slice();
        let n = unpack_kept(&mut r, &to, &["sub"]).unwrap();
        assert_eq!(n, 2, "the two .sub files, and not the .txt beside them");
        assert_eq!(std::fs::read(to.join("sub/a.sub")).unwrap(), b"a");
        assert_eq!(std::fs::read(to.join("b.sub")).unwrap(), b"b");
        assert!(!to.join("c.txt").exists());
    }

    /// Every row has to name a branch that exists and a directory of its
    /// own: a repeated `dir` is two repositories unpacking over each other,
    /// and the tarball and head URLs disagreeing about the branch is a
    /// download validated against the wrong commit.
    #[test]
    fn every_repository_row_is_consistent() {
        let mut dirs: Vec<&str> = REPOS.iter().map(|r| r.dir).collect();
        dirs.sort_unstable();
        let before = dirs.len();
        dirs.dedup();
        assert_eq!(dirs.len(), before, "one cache directory each");
        for r in REPOS {
            let branch = r
                .head
                .rsplit_once("/branches/")
                .map(|(_, b)| b.replace("%2F", "/"))
                .unwrap_or_else(|| panic!("{} has no branch in its head URL", r.dir));
            assert!(
                r.tarball.ends_with(&format!("/refs/heads/{branch}")),
                "{}: the tarball and the head URL name different branches",
                r.dir
            );
            assert!(r.max_bytes >= MAX_BYTES, "{}: a cap under the default", r.dir);
            assert!(!r.terms.is_empty() && !r.about.is_empty(), "{}: unattributed", r.dir);
        }
    }

    #[test]
    fn a_path_escaping_the_root_is_refused() {
        let d = tmpdir("escape");
        assert!(join_under(&d, "../out").is_err());
        assert!(join_under(&d, "/etc/passwd").is_err());
        assert!(join_under(&d, "sub/../../out").is_err());
        assert!(join_under(&d, "sub/x").is_ok());
    }

    #[test]
    fn files_with_lists_only_the_extension_asked_for() {
        let d = tmpdir("walk");
        let cache = Cache::new(d.join("cache"));
        std::fs::create_dir_all(cache.dir().join("git").join("nullsec-flipper-suite/sub")).unwrap();
        let tree = cache.dir().join("git").join("nullsec-flipper-suite");
        std::fs::write(tree.join("sub/a.sub"), b"a").unwrap();
        std::fs::write(tree.join("b.sub"), b"b").unwrap();
        std::fs::write(tree.join("skip.txt"), b"x").unwrap();
        std::fs::write(tree.join("SHOUTED.SUB"), b"c").unwrap();
        write_meta(
            repo_at(&d),
            &cache,
            &Meta { commit: Some("abc".into()), files: 3, bytes: 3, checked: 1, refused: None },
        );
        // With the dot or without it, and whatever case the file is in: a
        // traded capture is as likely to be .SUB as .sub.
        let got = files_with(repo_at(&d), &cache, "sub");
        assert_eq!(
            got,
            vec!["SHOUTED.SUB".to_string(), "b.sub".to_string(), "sub/a.sub".to_string()]
        );
        assert_eq!(files_with(repo_at(&d), &cache, ".sub"), got);
    }

    /// A file short of a full header ends the archive rather than being
    /// read as a header of zeros.
    #[test]
    fn a_truncated_tarball_yields_what_landed_whole() {
        let d = tmpdir("short");
        let to = d.join("tree");
        let mut tar = Vec::new();
        flate2::read::GzDecoder::new(gzipped("repo-main").as_slice())
            .read_to_end(&mut tar)
            .unwrap();
        let cut = tar.len() - 40;
        let mut r = &tar[..cut];
        let n = unpack_kept(&mut r, &to, &[]).unwrap();
        assert_eq!(n, 3, "every whole entry before the cut");
    }
}

/// Network, so opt in: `cargo test -p datasets git:: -- --ignored`.
#[cfg(test)]
mod network {
    use super::*;
    use crate::cache::Cache;

    #[test]
    #[ignore = "fetches from GitHub"]
    fn the_real_repository_downloads_and_lists_its_subs() {
        let d = std::env::temp_dir().join(format!("waveshark-git-net-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        let cache = Cache::new(&d);
        // The short one, so the test costs a couple of hundred kilobytes:
        // what is under test is the URLs, the filter and the walk, and a
        // repository of thirteen thousand captures tests none of it harder.
        let t = get(&MUDDLEDBOX, &cache).expect("download");
        assert!(!t.commit.is_empty());
        let subs = files_with(&MUDDLEDBOX, &cache, "sub");
        assert!(subs.len() >= 12, "the set holds its captures: {}", subs.len());
        assert!(
            subs.iter().all(|p| p.to_lowercase().ends_with(".sub")),
            "and nothing else was written"
        );
        let _ = std::fs::remove_dir_all(&d);
    }
}
