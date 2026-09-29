use std::io::Read;
use std::time::Duration;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Method {
    Get,
    Post,
}

pub enum Body {
    Empty,
    Bytes(Vec<u8>),
    Form(Vec<(String, String)>),
    Multipart(Vec<Part>),
}

pub struct Part {
    pub name: String,
    pub bytes: Vec<u8>,
    pub file_name: Option<String>,
    pub mime: Option<String>,
}

pub struct Request {
    pub method: Method,
    pub url: String,
    pub headers: Vec<(String, String)>,
    pub query: Vec<(String, String)>,
    pub body: Body,
    pub timeout: Option<Duration>,
    pub connect_timeout: Option<Duration>,
}

pub fn get(url: impl Into<String>) -> Request {
    Request::new(Method::Get, url.into())
}

pub fn post(url: impl Into<String>) -> Request {
    Request::new(Method::Post, url.into())
}

impl Request {
    fn new(method: Method, url: String) -> Self {
        Request {
            method,
            url,
            headers: Vec::new(),
            query: Vec::new(),
            body: Body::Empty,
            timeout: None,
            connect_timeout: None,
        }
    }

    pub fn header(mut self, name: &str, value: &str) -> Self {
        self.headers.push((name.to_string(), value.to_string()));
        self
    }

    pub fn basic_auth(self, user: &str, password: &str) -> Self {
        use base64::Engine;
        let token = base64::engine::general_purpose::STANDARD.encode(format!("{user}:{password}"));
        self.header("Authorization", &format!("Basic {token}"))
    }

    pub fn bearer(self, token: &str) -> Self {
        self.header("Authorization", &format!("Bearer {token}"))
    }

    pub fn query(mut self, pairs: &[(&str, &str)]) -> Self {
        self.query.extend(pairs.iter().map(|(k, v)| (k.to_string(), v.to_string())));
        self
    }

    pub fn form(mut self, pairs: &[(&str, &str)]) -> Self {
        self.body = Body::Form(pairs.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect());
        self
    }

    pub fn body(mut self, bytes: impl Into<Vec<u8>>) -> Self {
        self.body = Body::Bytes(bytes.into());
        self
    }

    pub fn part(mut self, part: Part) -> Self {
        match &mut self.body {
            Body::Multipart(parts) => parts.push(part),
            _ => self.body = Body::Multipart(vec![part]),
        }
        self
    }

    pub fn text_part(self, name: &str, value: &str) -> Self {
        self.part(Part {
            name: name.to_string(),
            bytes: value.as_bytes().to_vec(),
            file_name: None,
            mime: None,
        })
    }

    pub fn file_part(self, name: &str, file_name: &str, mime: &str, bytes: Vec<u8>) -> Self {
        self.part(Part {
            name: name.to_string(),
            bytes,
            file_name: Some(file_name.to_string()),
            mime: Some(mime.to_string()),
        })
    }

    pub fn timeout(mut self, timeout: Duration) -> Self {
        self.timeout = Some(timeout);
        self
    }

    pub fn connect_timeout(mut self, timeout: Duration) -> Self {
        self.connect_timeout = Some(timeout);
        self
    }

    pub async fn send(self) -> Result<reqwest::Response, String> {
        let url = self.url.clone();
        self.build()?.send().await.map_err(|e| format!("{url}: {e}"))
    }

    pub fn wait(self) -> Result<Response, String> {
        crate::platform::wait(self)
    }

    fn build(self) -> Result<reqwest::RequestBuilder, String> {
        let client = crate::platform::client_for(&self).map_err(|e| e.to_string())?;
        let mut out = match self.method {
            Method::Get => client.get(&self.url),
            Method::Post => client.post(&self.url),
        };
        out = crate::platform::limit(out, &self);
        for (k, v) in &self.headers {
            out = out.header(k, v);
        }
        if !self.query.is_empty() {
            out = out.query(&self.query);
        }
        Ok(match self.body {
            Body::Empty => out,
            Body::Bytes(b) => out.body(b),
            Body::Form(pairs) => out.form(&pairs),
            Body::Multipart(parts) => {
                let mut form = reqwest::multipart::Form::new();
                for p in parts {
                    let mut part = reqwest::multipart::Part::bytes(p.bytes);
                    if let Some(f) = p.file_name {
                        part = part.file_name(f);
                    }
                    if let Some(m) = p.mime {
                        part = part.mime_str(&m).map_err(|e| e.to_string())?;
                    }
                    form = form.part(p.name, part);
                }
                out.multipart(form)
            }
        })
    }
}

pub struct Response {
    pub status: u16,
    pub url: String,
    pub headers: Vec<(String, String)>,
    pub body: Box<dyn Read + Send>,
}

pub(crate) fn headers_of(resp: &reqwest::Response) -> Vec<(String, String)> {
    resp.headers()
        .iter()
        .filter_map(|(k, v)| Some((k.as_str().to_string(), v.to_str().ok()?.to_string())))
        .collect()
}

impl Response {
    pub fn header(&self, name: &str) -> Option<&str> {
        self.headers.iter().find(|(k, _)| k.eq_ignore_ascii_case(name)).map(|(_, v)| v.as_str())
    }

    pub fn all<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a str> {
        self.headers
            .iter()
            .filter(move |(k, _)| k.eq_ignore_ascii_case(name))
            .map(|(_, v)| v.as_str())
    }

    pub fn is_success(&self) -> bool {
        (200..300).contains(&self.status)
    }

    pub fn ok(self) -> Result<Self, String> {
        match self.is_success() {
            true => Ok(self),
            false => Err(format!("HTTP {} from {}", self.status, self.url)),
        }
    }

    pub fn text(mut self) -> Result<String, String> {
        let mut out = String::new();
        self.body.read_to_string(&mut out).map_err(|e| e.to_string())?;
        Ok(out)
    }

    pub fn bytes(mut self) -> Result<Vec<u8>, String> {
        let mut out = Vec::new();
        self.body.read_to_end(&mut out).map_err(|e| e.to_string())?;
        Ok(out)
    }
}

impl Read for Response {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        self.body.read(buf)
    }
}
