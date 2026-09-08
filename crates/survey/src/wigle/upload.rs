//! Posting a WiGLE CSV file to wigle.net.
//!
//! One request: `POST /api/v2/file/upload`, the file as a multipart part
//! named `file`, basic authentication with the API name and token from
//! <https://wigle.net/account>, and `donate` saying whether WiGLE may sell
//! the contents on. The response is JSON carrying a transaction id, which is
//! the only receipt there is: an upload is queued for processing rather than
//! ingested while you wait, so a success here means accepted, not counted.
//!
//! Credentials are the operator's and this never invents them. An upload with
//! no name and token is possible against the same endpoint and lands
//! anonymously, but then it cannot be attributed, retracted or found again,
//! so it is refused here rather than done quietly.

use std::time::Duration;

/// Where a file goes.
const URL: &str = "https://api.wigle.net/api/v2/file/upload";

/// A drive's file over a phone tether is slow, and a request that gives up
/// halfway leaves the spool file to be tried again rather than lost.
const TIMEOUT: Duration = Duration::from_secs(180);

/// Who is uploading, and on what terms.
#[derive(Clone, Default, PartialEq, Eq)]
pub struct Account {
    /// The API name from the account page, which is not the login name.
    pub name: String,
    pub token: String,
    /// Whether WiGLE may licence the data commercially. Off unless the
    /// operator says otherwise: it is their observation to give away.
    pub donate: bool,
}

impl Account {
    pub fn is_complete(&self) -> bool {
        !self.name.trim().is_empty() && !self.token.trim().is_empty()
    }
}

/// A token is a password. Nothing prints one, including a debug format that
/// ends up in a log somebody pastes into an issue.
impl std::fmt::Debug for Account {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "wigle account {} (token hidden)", self.name)
    }
}

/// What came back: the transaction the file was accepted as, and whatever
/// WiGLE said about it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Receipt {
    pub transaction: Option<String>,
    pub message: Option<String>,
}

/// Send one file. Blocking, so it belongs on a thread of its own.
///
/// An error here is a file that has not been accepted and should be tried
/// again: the caller keeps the spool file until this returns `Ok`.
pub fn upload(account: &Account, filename: &str, csv: Vec<u8>) -> Result<Receipt, String> {
    if !account.is_complete() {
        return Err("no WiGLE API name and token set".into());
    }
    let part = reqwest::blocking::multipart::Part::bytes(csv)
        .file_name(filename.to_string())
        .mime_str("text/csv")
        .map_err(|e| e.to_string())?;
    let form = reqwest::blocking::multipart::Form::new()
        .part("file", part)
        .text("donate", if account.donate { "on" } else { "off" });
    let http = httpc::blocking(TIMEOUT).map_err(|e| e.to_string())?;
    let resp = http
        .post(URL)
        .basic_auth(account.name.trim(), Some(account.token.trim()))
        .header("Accept", "application/json")
        .multipart(form)
        .send()
        .map_err(|e| e.to_string())?;
    let status = resp.status();
    let body = resp.text().unwrap_or_default();
    if status == reqwest::StatusCode::UNAUTHORIZED {
        return Err("WiGLE refused the API name and token".into());
    }
    if !status.is_success() {
        return Err(format!("WiGLE answered {}: {}", status.as_u16(), first_line(&body)));
    }
    read_receipt(&body)
}

/// What WiGLE said, as a receipt or as the reason it did not take the file.
///
/// A 200 with `success: false` is a refusal, and reading only the status code
/// would count a rejected file as sent and delete it.
fn read_receipt(body: &str) -> Result<Receipt, String> {
    let Ok(v) = serde_json::from_str::<serde_json::Value>(body) else {
        // Not JSON: an interception portal on the tether, or an error page.
        return Err(format!("WiGLE sent something that is not JSON: {}", first_line(body)));
    };
    let message = v
        .get("message")
        .or_else(|| v.get("error"))
        .or_else(|| v.get("warning"))
        .and_then(|m| m.as_str())
        .map(str::to_string);
    if v.get("success").and_then(serde_json::Value::as_bool) == Some(false) {
        return Err(message.unwrap_or_else(|| "WiGLE rejected the file".into()));
    }
    // The id is `transids` on some responses and `transid` on others, and it
    // is the only thing that lets an operator find the upload again.
    let transaction = v
        .get("transids")
        .and_then(|t| t.as_array())
        .and_then(|a| a.first())
        .and_then(|s| s.as_str())
        .or_else(|| v.get("transid").and_then(|s| s.as_str()))
        .map(str::to_string);
    Ok(Receipt { transaction, message })
}

fn first_line(s: &str) -> String {
    s.lines().next().unwrap_or("").chars().take(160).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_receipt_carries_the_transaction_id() {
        let r = read_receipt(r#"{"success":true,"transids":["20260907-00042"]}"#).unwrap();
        assert_eq!(r.transaction.as_deref(), Some("20260907-00042"));
        let r = read_receipt(r#"{"success":true,"transid":"20260907-00043"}"#).unwrap();
        assert_eq!(r.transaction.as_deref(), Some("20260907-00043"));
    }

    /// A refusal arrives as a 200, so a caller that trusted the status code
    /// would delete a file WiGLE never took.
    #[test]
    fn a_success_false_body_is_a_failure() {
        let e = read_receipt(r#"{"success":false,"message":"too many uploads"}"#).unwrap_err();
        assert_eq!(e, "too many uploads");
    }

    #[test]
    fn a_page_that_is_not_json_is_a_failure_and_not_a_receipt() {
        let e = read_receipt("<html><body>login</body></html>").unwrap_err();
        assert!(e.contains("not JSON"), "{e}");
    }

    #[test]
    fn an_account_without_a_token_uploads_nothing() {
        let a = Account { name: "AID".into(), token: " ".into(), donate: false };
        assert!(!a.is_complete());
        assert!(upload(&a, "x.csv", Vec::new()).is_err());
    }

    /// A token in a log is a token somebody else has.
    #[test]
    fn a_debug_format_does_not_print_the_token() {
        let a = Account { name: "AID00".into(), token: "hunter2".into(), donate: true };
        assert!(!format!("{a:?}").contains("hunter2"), "{a:?}");
    }
}
