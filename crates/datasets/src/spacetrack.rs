//! Elements from Space-Track, which is the catalogue everything else is
//! derived from.
//!
//! CelesTrak publishes groups: a few hundred objects somebody has decided
//! belong together. Space-Track publishes the catalogue itself, so a query
//! here answers about an object nobody has grouped yet, which is where a
//! cubesat spends its first weeks and exactly when its beacon is worth
//! hearing.
//!
//! # Why this is not an [`crate::cache::Http`] source
//!
//! Every other dataset is a URL anybody may fetch. Space-Track needs an
//! account: an identity and a password are posted to `/ajaxauth/login`,
//! the session comes back as a cookie, and the query only answers while
//! that cookie is held. So the fetch is three requests, the credential is
//! the operator's own, and nothing is downloaded at all until they have
//! given one.
//!
//! The session is logged out at the end of the fetch rather than kept. A
//! receiver refreshes a dataset a few times a day and holding a login open
//! between them is a session on somebody else's server doing nothing.
//!
//! # Their limits, kept
//!
//! Space-Track's API guidance asks for fewer than 30 requests a minute and
//! fewer than 300 an hour, and asks that the same query not be repeated more
//! often than the data behind it changes. A fetch here is three requests,
//! nothing polls, [`MAX_AGE`] is six hours, and a refusal stops the dataset
//! until a person presses refresh, the same way CelesTrak's does.

use crate::cache::{Error, Fetch, Seen};
use httpc::USER_AGENT as AGENT;
use std::io::Write;
use std::sync::RwLock;
use std::time::Duration;

/// How long a copy is treated as current. Their catalogue is rebuilt several
/// times a day; six hours is inside that and is as often as elements are
/// worth refetching for a receiver.
pub const MAX_AGE: Duration = Duration::from_secs(6 * 3600);

/// The login the operator gave, or none.
///
/// Held here rather than passed down from the interface because a
/// [`crate::cache::Source`] is built statically and the credential arrives
/// later, when a session is restored or a person types one.
static ACCOUNT: RwLock<Option<Account>> = RwLock::new(None);

/// A Space-Track login. Not stored by this crate beyond the process.
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct Account {
    pub identity: String,
    pub password: String,
}

impl Account {
    pub fn is_complete(&self) -> bool {
        !self.identity.trim().is_empty() && !self.password.is_empty()
    }
}

/// Keep what has been typed, whether or not it is a whole login yet: an
/// identity and a password are two fields and arrive one at a time, and
/// dropping the first while waiting for the second empties the field under
/// the operator as they type.
pub fn set_account(account: Option<Account>) {
    *ACCOUNT.write().unwrap_or_else(|e| e.into_inner()) =
        account.filter(|a| !a.identity.is_empty() || !a.password.is_empty());
}

pub fn account() -> Option<Account> {
    ACCOUNT.read().unwrap_or_else(|e| e.into_inner()).clone()
}

/// Whether there is a login to fetch with, which is both halves of one.
pub fn has_account() -> bool {
    account().is_some_and(|a| a.is_complete())
}

const BASE: &str = "https://www.space-track.org";

/// One query against the catalogue, fetched under the operator's login.
pub struct Query {
    /// The whole query URL, as their documentation writes one.
    pub url: String,
}

impl Fetch for Query {
    fn origin(&self) -> String {
        self.url.clone()
    }

    fn fetch(&self, _have: &Seen, to: &mut dyn Write) -> Result<Option<Seen>, Error> {
        let fail = |e: String| Error::Fetch(self.url.clone(), e);
        let Some(account) = account().filter(Account::is_complete) else {
            return Err(fail("no Space-Track login has been given".into()));
        };
        let agent: ureq::Agent = ureq::Agent::config_builder()
            .user_agent(AGENT)
            .http_status_as_error(false)
            .timeout_global(Some(Duration::from_secs(600)))
            .build()
            .into();

        let login = agent
            .post(format!("{BASE}/ajaxauth/login"))
            .send_form([("identity", account.identity.trim()), ("password", &account.password)])
            .map_err(|e| fail(e.to_string()))?;
        if login.status().as_u16() != 200 {
            return Err(Error::Status(format!("{BASE}/ajaxauth/login"), login.status().as_u16()));
        }
        // A wrong password is a 200 with a JSON complaint in the body, not a
        // 401, so the body is what says whether we are logged in.
        let cookie = session_cookie(&login);
        let mut body = login;
        let said = body.body_mut().with_config().limit(4096).read_to_string().unwrap_or_default();
        if said.contains("Failed") || said.contains("failed") {
            return Err(fail("Space-Track refused the login".into()));
        }
        let Some(cookie) = cookie else {
            return Err(fail("Space-Track gave no session cookie".into()));
        };

        let mut resp = agent
            .get(&self.url)
            .header("Cookie", &cookie)
            .call()
            .map_err(|e| fail(e.to_string()))?;
        let code = resp.status().as_u16();
        if code != 200 {
            let _ = agent.get(format!("{BASE}/ajaxauth/logout")).header("Cookie", &cookie).call();
            return Err(Error::Status(self.url.clone(), code));
        }
        let copied =
            std::io::copy(&mut resp.body_mut().with_config().limit(MAX_BYTES).reader(), to);
        // Logged out whether or not the copy worked: a failed download is
        // still a session held open on their server.
        let _ = agent.get(format!("{BASE}/ajaxauth/logout")).header("Cookie", &cookie).call();
        copied.map_err(|e| fail(e.to_string()))?;
        // Nothing to revalidate against: the answer is generated per query
        // and carries no entity tag or modification date, so freshness here
        // is `max_age` and nothing else.
        Ok(Some(Seen::default()))
    }
}

/// The whole catalogue is about 30 MB in this form. A limit well above it
/// guards against a redirect to something else entirely.
const MAX_BYTES: u64 = 1 << 30;

/// The session cookie out of a login response.
///
/// Their cookie is called `chocolatechip`, but the name is taken from what
/// was sent rather than assumed: a login that starts setting a second cookie
/// would otherwise silently stop working.
fn session_cookie<T>(resp: &ureq::http::Response<T>) -> Option<String> {
    let mut parts: Vec<String> = Vec::new();
    for v in resp.headers().get_all("set-cookie") {
        let Ok(text) = v.to_str() else { continue };
        let pair = text.split(';').next().unwrap_or_default().trim();
        if !pair.is_empty() && pair.contains('=') {
            parts.push(pair.to_string());
        }
    }
    match parts.is_empty() {
        true => None,
        false => Some(parts.join("; ")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_login_is_only_a_login_when_both_halves_are_there() {
        assert!(!Account::default().is_complete());
        let half = Account { identity: "someone@example.com".into(), password: String::new() };
        assert!(!half.is_complete());
        assert!(Account { password: "hunter2".into(), ..half }.is_complete());
    }

    /// Half a credential is kept but is not a login: the field an operator
    /// is still typing into must not empty itself, and a row must not offer
    /// a refresh that the far end will refuse.
    #[test]
    fn half_a_login_is_held_but_is_not_one() {
        set_account(Some(Account { identity: "someone".into(), password: String::new() }));
        assert!(!has_account());
        assert_eq!(account().map(|a| a.identity), Some("someone".to_string()));
        set_account(Some(Account { identity: "someone".into(), password: "hunter2".into() }));
        assert!(has_account());
        set_account(None);
        assert!(!has_account());
        assert_eq!(account(), None);
    }

    #[test]
    fn the_session_cookie_is_whatever_was_set() {
        let resp = ureq::http::Response::builder()
            .header("set-cookie", "chocolatechip=abc123; Path=/; HttpOnly")
            .header("set-cookie", "spacetrack_csrf_cookie=def; Path=/")
            .body(())
            .unwrap();
        assert_eq!(
            session_cookie(&resp).as_deref(),
            Some("chocolatechip=abc123; spacetrack_csrf_cookie=def")
        );
        let none = ureq::http::Response::builder().body(()).unwrap();
        assert_eq!(session_cookie(&none), None);
    }
}
