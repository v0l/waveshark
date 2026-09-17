//! Download one script repository by its directory name and report what
//! landed. For checking a new row's URLs without a test that downloads
//! gigabytes on every run.
fn main() {
    let want = std::env::args().nth(1).unwrap_or_default();
    let repo = datasets::git::REPOS
        .iter()
        .find(|r| r.dir == want)
        .unwrap_or_else(|| panic!("no repository {want:?}"));
    let dir = std::env::temp_dir().join(format!("waveshark-fetch-{want}"));
    let _ = std::fs::remove_dir_all(&dir);
    let cache = datasets::cache::Cache::new(&dir);
    match datasets::git::get(repo, &cache) {
        Ok(t) => {
            let subs = datasets::git::files_with(repo, &cache, "sub");
            let bytes: u64 = subs
                .iter()
                .filter_map(|p| std::fs::metadata(t.dir.join(p)).ok())
                .map(|m| m.len())
                .sum();
            println!("{want}: {} at {}, {} KiB kept", subs.len(), &t.commit[..8], bytes / 1024);
        }
        Err(e) => println!("{want}: FAILED {e}"),
    }
    let _ = std::fs::remove_dir_all(&dir);
}
