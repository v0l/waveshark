use crate::request::{Request, Response, headers_of};
use std::time::Duration;

pub fn client(_: Duration) -> Result<reqwest::Client, reqwest::Error> {
    reqwest::Client::builder().build()
}

pub(crate) fn client_for(_: &Request) -> Result<reqwest::Client, reqwest::Error> {
    reqwest::Client::builder().build()
}

pub(crate) fn limit(out: reqwest::RequestBuilder, req: &Request) -> reqwest::RequestBuilder {
    match req.timeout {
        Some(t) => out.timeout(t),
        None => out,
    }
}

type Got = Result<(u16, String, Vec<(String, String)>, Vec<u8>), String>;

pub(crate) fn wait(req: Request) -> Result<Response, String> {
    let (tx, rx) = std::sync::mpsc::channel::<Got>();
    let job: common::page::Job = Box::new(move || {
        Box::pin(async move {
            let got = async move {
                let resp = req.send().await?;
                let (status, url, headers) =
                    (resp.status().as_u16(), resp.url().to_string(), headers_of(&resp));
                let body = resp.bytes().await.map_err(|e| e.to_string())?.to_vec();
                Ok((status, url, headers, body))
            };
            let _ = tx.send(got.await);
        })
    });
    common::page::run(job)?;
    let (status, url, headers, body) = rx.recv().map_err(|_| "the request was dropped")??;
    Ok(Response { status, url, headers, body: Box::new(std::io::Cursor::new(body)) })
}

pub struct Chunks(Option<reqwest::Response>);

impl Chunks {
    pub fn of(resp: reqwest::Response) -> Self {
        Chunks(Some(resp))
    }

    pub async fn next(&mut self) -> Result<Option<Vec<u8>>, reqwest::Error> {
        match self.0.take() {
            Some(resp) => Ok(Some(resp.bytes().await?.to_vec())),
            None => Ok(None),
        }
    }
}
