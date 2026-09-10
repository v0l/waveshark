//! The one HTTP client, and the one name this program goes by.
//!
//! Everything the receiver fetches or posts, map tiles, dataset files, a
//! release check, a mesh node's configuration, a WiGLE upload, goes out over
//! a client built here. Not because building a client is hard but because the
//! `User-Agent` is how the far end sees this program: a tile server that
//! blocks an unnamed client blocks the map, an operator whose upload is
//! refused needs the string in somebody's log to say what sent it, and a
//! client per module means four strings that drift apart and one, in
//! `meshnode`, that was never set at all.
//!
//! So: **no `reqwest::Client::builder()` anywhere else.** Take one from here,
//! with the timeout the caller needs.

use std::time::Duration;

/// What this program calls itself to every server it talks to.
///
/// Name, version, and a URL somebody reading a log can go and look at, which
/// is the convention every well-behaved crawler and client follows.
pub const USER_AGENT: &str =
    concat!("WaveShark/", env!("CARGO_PKG_VERSION"), " (https://github.com/v0l/waveshark)");

/// An asynchronous client, for anything running on the interface's runtime.
pub fn client(timeout: Duration) -> Result<reqwest::Client, reqwest::Error> {
    reqwest::Client::builder().user_agent(USER_AGENT).timeout(timeout).build()
}

/// A blocking client, for a worker thread of its own.
///
/// Must not be built or used on a runtime thread: `reqwest`'s blocking client
/// drives its own runtime and panics when it finds itself inside another.
pub fn blocking(timeout: Duration) -> Result<reqwest::blocking::Client, reqwest::Error> {
    reqwest::blocking::Client::builder().user_agent(USER_AGENT).timeout(timeout).build()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The string is what a server operator sees, so it has to name the
    /// program, its version and where to complain.
    #[test]
    fn the_agent_names_the_program_and_where_to_find_it() {
        assert!(USER_AGENT.starts_with("WaveShark/"));
        assert!(USER_AGENT.contains(env!("CARGO_PKG_VERSION")));
        assert!(USER_AGENT.contains("https://"));
    }

    #[test]
    fn a_client_builds() {
        assert!(client(Duration::from_secs(5)).is_ok());
    }
}
