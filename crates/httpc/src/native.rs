use crate::request::{Request, Response, headers_of};
use std::sync::LazyLock;
use std::time::Duration;

/// An asynchronous client, for anything running on the interface's runtime.
pub fn client(timeout: Duration) -> Result<reqwest::Client, reqwest::Error> {
    reqwest::Client::builder().user_agent(crate::USER_AGENT).timeout(timeout).build()
}

pub(crate) fn client_for(req: &Request) -> Result<reqwest::Client, reqwest::Error> {
    let mut b = reqwest::Client::builder().user_agent(crate::USER_AGENT);
    if let Some(t) = req.timeout {
        b = b.timeout(t);
    }
    if let Some(t) = req.connect_timeout {
        b = b.connect_timeout(t);
    }
    b.build()
}

pub(crate) fn limit(out: reqwest::RequestBuilder, _: &Request) -> reqwest::RequestBuilder {
    out
}

static RUNTIME: LazyLock<tokio::runtime::Runtime> = LazyLock::new(|| {
    tokio::runtime::Builder::new_multi_thread()
        .worker_threads(1)
        .thread_name("http")
        .enable_all()
        .build()
        .expect("the HTTP runtime")
});

pub(crate) fn wait(req: Request) -> Result<Response, String> {
    let resp = RUNTIME.block_on(req.send())?;
    let (status, url, headers) =
        (resp.status().as_u16(), resp.url().to_string(), headers_of(&resp));
    Ok(Response {
        status,
        url,
        headers,
        body: Box::new(Streamed { resp, held: Vec::new(), at: 0 }),
    })
}

struct Streamed {
    resp: reqwest::Response,
    held: Vec<u8>,
    at: usize,
}

impl std::io::Read for Streamed {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.at == self.held.len() {
            match RUNTIME.block_on(self.resp.chunk()).map_err(std::io::Error::other)? {
                Some(chunk) => {
                    self.held = chunk.to_vec();
                    self.at = 0;
                }
                None => return Ok(0),
            }
        }
        let n = buf.len().min(self.held.len() - self.at);
        buf[..n].copy_from_slice(&self.held[self.at..self.at + n]);
        self.at += n;
        Ok(n)
    }
}

pub struct Chunks(reqwest::Response);

impl Chunks {
    pub fn of(resp: reqwest::Response) -> Self {
        Chunks(resp)
    }

    pub async fn next(&mut self) -> Result<Option<Vec<u8>>, reqwest::Error> {
        Ok(self.0.chunk().await?.map(|b| b.to_vec()))
    }
}
