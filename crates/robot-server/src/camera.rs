//! Streaming proxy: only multipart content type and bytes cross the boundary.
//! No redirects, upstream error bodies, auth headers, URL credentials or cookies.
use crate::{App, Config, Failure};
use axum::{
    body::Body,
    extract::State,
    http::{header, StatusCode},
    response::{IntoResponse, Response},
};
use reqwest::{Client, Url};
use std::{path::PathBuf, time::Duration};
#[derive(Clone)]
pub struct Camera {
    client: Client,
    url: Url,
    netrc: PathBuf,
}
impl Camera {
    pub fn new(c: &Config) -> Result<Self, Failure> {
        let url = Url::parse(&c.camera_url).map_err(|_| "invalid camera URL")?;
        if !matches!(url.scheme(), "http" | "https")
            || url.host_str().is_none()
            || !url.username().is_empty()
            || url.password().is_some()
        {
            return Err("camera URL must be HTTP(S) without credentials".into());
        }
        let client = Client::builder()
            .redirect(reqwest::redirect::Policy::none())
            .connect_timeout(Duration::from_secs(3))
            .no_proxy()
            .build()?;
        Ok(Self {
            client,
            url,
            netrc: c.netrc.clone(),
        })
    }
    async fn credentials(&self) -> Result<Option<(String, String)>, Failure> {
        let meta = match tokio::fs::metadata(&self.netrc).await {
            Ok(m) => m,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
            Err(_) => return Err("camera credentials unavailable".into()),
        };
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            if meta.permissions().mode() & 0o077 != 0 {
                return Err("camera credentials must be private".into());
            }
        }
        if meta.len() > 65536 {
            return Err("camera credentials file too large".into());
        }
        let text = tokio::fs::read_to_string(&self.netrc)
            .await
            .map_err(|_| "camera credentials unavailable")?;
        netrc(&text, self.url.host_str().unwrap())
    }
    pub async fn stream(&self) -> Result<reqwest::Response, Failure> {
        let mut request = self.client.get(self.url.clone());
        if let Some((login, password)) = self.credentials().await? {
            request = request.basic_auth(login, Some(password));
        }
        let upstream = tokio::time::timeout(Duration::from_secs(5), request.send())
            .await
            .map_err(|_| "camera request timed out")?
            .map_err(|_| "camera connection failed")?;
        if upstream.status() != StatusCode::OK {
            return Err("camera returned unsuccessful status".into());
        }
        let _kind = upstream
            .headers()
            .get(header::CONTENT_TYPE)
            .filter(|v| {
                v.to_str().is_ok_and(|s| {
                    s.to_ascii_lowercase()
                        .starts_with("multipart/x-mixed-replace")
                })
            })
            .cloned()
            .ok_or("camera returned non-MJPEG content")?;
        Ok(upstream)
    }
    async fn response(&self) -> Result<Response, Failure> {
        let upstream = self.stream().await?;
        let kind = upstream.headers()[header::CONTENT_TYPE].clone();
        let bytes = futures_util::StreamExt::map(upstream.bytes_stream(), |chunk| {
            chunk.map_err(|_| std::io::Error::other("camera stream interrupted"))
        });
        Ok((
            [
                (header::CONTENT_TYPE, kind),
                (
                    header::CACHE_CONTROL,
                    axum::http::HeaderValue::from_static("no-store"),
                ),
            ],
            Body::from_stream(bytes),
        )
            .into_response())
    }
}
pub async fn proxy(State(app): State<App>) -> Response {
    match app.camera.response().await {
        Ok(r) => r,
        Err(_) => (StatusCode::BAD_GATEWAY, "camera unavailable").into_response(),
    }
}
// netrc tokens support quoted/escaped passwords and comments, without executing
// a shell or using a global/default machine entry for another host.
fn netrc(text: &str, host: &str) -> Result<Option<(String, String)>, Failure> {
    let mut tokens = Vec::new();
    let mut token = String::new();
    let mut quoted = false;
    let mut escaped = false;
    let mut comment = false;
    for c in text.chars() {
        if comment {
            if c == '\n' {
                comment = false;
            }
            continue;
        }
        if escaped {
            token.push(c);
            escaped = false;
            continue;
        }
        if c == '\\' {
            escaped = true;
            continue;
        }
        if c == '"' {
            quoted = !quoted;
            continue;
        }
        if !quoted && c == '#' {
            comment = true;
            continue;
        }
        if !quoted && c.is_whitespace() {
            if !token.is_empty() {
                tokens.push(std::mem::take(&mut token));
            }
        } else {
            token.push(c);
        }
    }
    if quoted || escaped {
        return Err("invalid camera credentials".into());
    }
    if !token.is_empty() {
        tokens.push(token);
    }
    let (mut machine, mut login, mut password) = (None, None, None);
    let mut i = 0;
    while i < tokens.len() {
        let key = tokens[i].as_str();
        i += 1;
        if matches!(key, "machine" | "default") {
            if machine == Some(host) {
                return match (login, password) {
                    (Some(l), Some(p)) => Ok(Some((l, p))),
                    _ => Err("incomplete camera credentials".into()),
                };
            }
            machine = None;
            login = None;
            password = None;
            if key == "machine" {
                machine = tokens.get(i).map(String::as_str);
                i += 1;
            }
            continue;
        }
        if matches!(key, "login" | "password" | "account") {
            let v = tokens
                .get(i)
                .ok_or("incomplete camera credentials")?
                .clone();
            i += 1;
            match key {
                "login" => login = Some(v),
                "password" => password = Some(v),
                _ => {}
            }
        }
    }
    if machine == Some(host) {
        match (login, password) {
            (Some(l), Some(p)) => Ok(Some((l, p))),
            _ => Err("incomplete camera credentials".into()),
        }
    } else {
        Ok(None)
    }
}
