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
use robot_server::session_recording::JpegStream;
use std::{path::PathBuf, time::Duration};
/// Hard wall-clock bound for one snapshot read; the smaller `max_age` hint
/// freshness only and never extends this limit.
const READ_LIMIT: Duration = Duration::from_secs(30);
/// Hard cap on bytes read from a snapshot stream so hostile endpoints cannot
/// keep one busy forever.
const MAX_READ: usize = 8 * 1024 * 1024;
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
    /// One-shot frame capture on a dedicated safe MJPEG stream (issue #59).
    /// Parses with the shared `JpegStream`, checks the SOI/EOI markers and
    /// the caller's size bound, returns the reception time of `Instant::now()`
    /// (exposure time is unknown; the frame is not promised to be fresh).
    /// Errors name only the reason — never the URL or credentials.
    pub async fn snapshot(
        &self,
        max_bytes: usize,
        max_age: Duration,
    ) -> Result<(Vec<u8>, std::time::Instant), String> {
        let mut upstream = self
            .stream()
            .await
            .map_err(|e| format!("camera stream unavailable: {e}"))?;
        // Freshness bound: never wait for a frame longer than the caller's
        // age limit (capped by the hard READ_LIMIT wall clock).
        let deadline = tokio::time::Instant::now() + max_age.min(READ_LIMIT);
        let mut parser = JpegStream::default();
        let mut received = 0usize;
        while received < MAX_READ {
            let chunk = match tokio::time::timeout_at(deadline, upstream.chunk()).await {
                Err(_) => return Err("camera frame wait timed out".into()),
                Ok(Err(_)) => return Err("camera stream interrupted before a frame".into()),
                Ok(Ok(None)) => {
                    return Err("camera stream ended without a valid frame".into());
                }
                Ok(Ok(Some(c))) => c,
            };
            received += chunk.len();
            for frame in parser.feed(&chunk) {
                let valid = frame.starts_with(&[0xff, 0xd8]) && frame.ends_with(&[0xff, 0xd9]);
                if !valid {
                    continue;
                }
                if frame.len() > max_bytes {
                    return Err(format!(
                        "camera frame exceeds the requested size limit: {} bytes",
                        frame.len()
                    ));
                }
                return Ok((frame, std::time::Instant::now()));
            }
        }
        Err("camera stream exceeded the read limit without a valid frame".into())
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
#[cfg(test)]
mod tests {
    use super::*;
    use tokio::{
        io::{AsyncReadExt, AsyncWriteExt},
        net::TcpListener,
    };

    fn jpeg(pixels: usize) -> Vec<u8> {
        let mut frame = vec![
            0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, b'J', b'F', b'I', b'F', 0x00, 0x01,
        ];
        frame.extend(std::iter::repeat(0xab).take(pixels));
        frame.extend_from_slice(&[0xff, 0xd9]);
        frame
    }
    fn multipart(frame: &[u8]) -> Vec<u8> {
        let mut body = format!(
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
            frame.len()
        )
        .into_bytes();
        body.extend_from_slice(frame);
        body.extend_from_slice(b"\r\n--frame\r\n");
        body
    }
    const GARBAGE: &[u8] = b"--frame\r\nContent-Type: text/plain\r\n\r\n\
        this multipart body deliberately contains no JPEG markers at all\r\n--frame\r\n";
    /// Minimal MJPEG upstream: answers one GET with the given body, then
    /// holds the connection open until the client drops it.
    async fn fake_upstream(respond: impl Fn() -> Vec<u8> + Send + 'static) -> String {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        tokio::spawn(async move {
            while let Ok((mut socket, _)) = listener.accept().await {
                let mut req = [0u8; 2048];
                let _ = socket.read(&mut req).await;
                let body = respond();
                let _ = socket
                    .write_all(
                        b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n",
                    )
                    .await;
                let _ = socket.write_all(&body).await;
                let mut sink = [0u8; 64];
                let _ = socket.read(&mut sink).await;
                let _ = socket.shutdown().await;
            }
        });
        format!("http://127.0.0.1:{port}/mjpeg")
    }
    /// Camera built the production way (same client policy) against a fake
    /// upstream. The netrc path is never created, so no credentials apply.
    fn camera_for(url: &str, netrc: PathBuf) -> Camera {
        Camera {
            client: Client::builder()
                .redirect(reqwest::redirect::Policy::none())
                .connect_timeout(Duration::from_secs(3))
                .no_proxy()
                .build()
                .unwrap(),
            url: Url::parse(url).unwrap(),
            netrc,
        }
    }
    fn absent_netrc() -> PathBuf {
        std::env::temp_dir().join(format!(
            "robot-camera-absent-netrc-{}-{:?}",
            std::process::id(),
            std::time::Instant::now()
        ))
    }

    #[tokio::test]
    async fn snapshot_returns_first_valid_frame_with_reception_time() {
        let frame = jpeg(512);
        let body = multipart(&frame);
        let url = fake_upstream(move || body.clone()).await;
        let camera = camera_for(&url, absent_netrc());
        let before = std::time::Instant::now();
        let (bytes, received) = camera
            .snapshot(64 * 1024, Duration::from_secs(5))
            .await
            .expect("first valid frame");
        assert_eq!(bytes, frame);
        assert!(received >= before && received <= std::time::Instant::now());
    }

    #[tokio::test]
    async fn snapshot_rejects_garbage_without_jpeg_markers() {
        let url = fake_upstream(|| GARBAGE.to_vec()).await;
        let camera = camera_for(&url, absent_netrc());
        let started = std::time::Instant::now();
        let error = camera
            .snapshot(64 * 1024, Duration::from_millis(250))
            .await
            .expect_err("markerless multipart body must not yield a frame");
        assert!(error.to_lowercase().contains("frame"), "cause: {error}");
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "garbage wait must honor max_age, took {:?}",
            started.elapsed()
        );
    }

    #[tokio::test]
    async fn snapshot_rejects_frame_larger_than_limit() {
        let frame = jpeg(4096);
        let body = multipart(&frame);
        let url = fake_upstream(move || body.clone()).await;
        let camera = camera_for(&url, absent_netrc());
        let error = camera
            .snapshot(1024, Duration::from_secs(5))
            .await
            .expect_err("oversized frame must be rejected");
        assert!(error.to_lowercase().contains("size"), "cause: {error}");
    }

    #[tokio::test]
    async fn snapshot_errors_never_leak_credentials() {
        let netrc = std::env::temp_dir().join(format!(
            "robot-camera-netrc-{}-{:?}",
            std::process::id(),
            std::time::Instant::now()
        ));
        std::fs::write(&netrc, "machine 127.0.0.1 login user password topsecret\n").unwrap();
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&netrc, std::fs::Permissions::from_mode(0o600)).unwrap();
        let url = fake_upstream(|| GARBAGE.to_vec()).await;
        let camera = camera_for(&url, netrc.clone());
        let started = std::time::Instant::now();
        let error = camera
            .snapshot(64 * 1024, Duration::from_millis(250))
            .await
            .expect_err("garbage stream must fail");
        assert!(
            started.elapsed() < Duration::from_secs(2),
            "garbage wait must honor max_age, took {:?}",
            started.elapsed()
        );
        assert!(
            !error.contains("topsecret") && !error.contains("user"),
            "leak: {error}"
        );
        let _ = std::fs::remove_file(&netrc);
    }

    #[tokio::test]
    async fn snapshot_stops_at_read_limit_on_endless_garbage() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        tokio::spawn(async move {
            while let Ok((mut socket, _)) = listener.accept().await {
                let mut req = [0u8; 2048];
                let _ = socket.read(&mut req).await;
                let _ = socket
                    .write_all(
                        b"HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n",
                    )
                    .await;
                let chunk = vec![b'.'; 64 * 1024];
                // Keep streaming markerless bytes until the client gives up.
                while socket.write_all(&chunk).await.is_ok() {}
            }
        });
        let url = format!("http://127.0.0.1:{port}/mjpeg");
        let camera = camera_for(&url, absent_netrc());
        let error = camera
            .snapshot(1024, Duration::from_secs(2))
            .await
            .expect_err("endless garbage must hit the read limit");
        assert!(error.to_lowercase().contains("read"), "cause: {error}");
    }
}
