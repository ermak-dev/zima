// FEATURE (#83): polygonal floorplan layer with an operator-triggered rebuild.
// SPEC: rebuild runs the offline floorplan tool over the current SLAM grid;
// results stream as versioned `floorplan` WS frames and replay on connect;
// no grid is an explicit error, failures never erase the last layer, rebuilds
// are rate-limited, and a missing/failing tool or timeout is an honest error.
// IMPLEMENTATION: floorplan.rs (server) spawns `python3 -I floorplan.py --json`
// with the grid on stdin; the App carries a watch channel with the latest
// versioned result; the HTTP endpoint is operator-gated like WS commands.
// RESULT: PENDING
// STATUS: IN_PROGRESS
mod harness {
    include!("mcp.rs");
}
use futures_util::StreamExt;
use harness::{packet, start_with, Server};
use serde_json::Value;
use std::{os::unix::fs::PermissionsExt, time::Duration};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    time::timeout,
};
use tokio_tungstenite::{connect_async, tungstenite::Message as WsMessage};

fn arr(v: &[f32]) -> Vec<u8> {
    let mut b = Vec::new();
    for _ in 0..2 {
        b.extend((v.len() as u32).to_be_bytes());
    }
    for f in v {
        b.extend(f.to_be_bytes());
    }
    b
}

fn scan_packet(stamp: f64) -> Vec<u8> {
    let mut p = 10f32.to_be_bytes().to_vec();
    p.extend(arr(&[1., 2., 0., 0.1, f32::NAN, 11.]));
    p.extend(arr(&[0., 0.7, 1., 2., 3., 4.]));
    p.extend(arr(&[]));
    p.extend(1u32.to_be_bytes());
    packet(6, 0, 3, stamp, &p)
}

fn odom() -> Vec<u8> {
    let mut p = Vec::new();
    for v in [0.0f64, 0.0, 0.0, 0.05, 0.0, 0.0] {
        p.extend(v.to_be_bytes());
    }
    p.extend(0u32.to_be_bytes());
    packet(4, 0, 1, 1., &p)
}

/// Shell SLAM fake: one scan yields a room-like grid (occupied border,
/// free interior) so the floorplan tool can produce rooms.
fn slam_fake(dir: &std::path::Path) -> std::path::PathBuf {
    // 84x64 cells at 5cm: a 4.2m x 3.2m room with 1-cell walls.
    let w = 84usize;
    let h = 64usize;
    let mut cells = vec!["-1"; w * h];
    for y in 0..h {
        for x in 0..w {
            let border = x == 0 || y == 0 || x == w - 1 || y == h - 1;
            let interior = x > 1 && y > 1 && x < w - 2 && y < h - 2;
            cells[y * w + x] = if border {
                "100"
            } else if interior {
                "0"
            } else {
                "-1"
            };
        }
    }
    let grid = cells.join(",");
    let path = dir.join("slam-adapter-floorplan");
    std::fs::write(
        &path,
        format!(
            "#!/bin/sh\nn=0\nwhile read line; do\n  case \"$line\" in \
             *start_mapping*) echo '{{\"ok\":true,\"mode\":\"mapping\"}}' ;; \
             *update*) n=$((n+1)); echo '{{\"ok\":true,\"mode\":\"mapping\",\"pose\":null,\"quality\":0.5}}' ;; \
             *map*) echo '{{\"ok\":true,\"grid\":{{\"x_min\":0.0,\"y_min\":0.0,\"res\":0.05,\"w\":{w},\"h\":{h},\"cells\":\"{grid}\"}}}}' ;; \
             *) echo '{{\"ok\":true}}' ;; \
             esac\ndone\n"
        ),
    )
    .unwrap();
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o700)).unwrap();
    path
}

fn floorplan_tool() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/floorplan/floorplan.py")
}

async fn http_post(port: u16, path: &str, body: &str) -> (u16, String) {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).await.unwrap();
    let request = format!(
        "POST {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nContent-Type: application/json\r\n\
         Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    );
    stream.write_all(request.as_bytes()).await.unwrap();
    let mut response = Vec::new();
    timeout(Duration::from_secs(15), stream.read_to_end(&mut response))
        .await
        .expect("http response timeout");
    let text = String::from_utf8_lossy(&response).to_string();
    let status: u16 = text
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let body = text.split("\r\n\r\n").nth(1).unwrap_or("").to_string();
    (status, body)
}

async fn next_floorplan_frame(
    ws: &mut tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>>,
) -> Value {
    loop {
        let m = timeout(Duration::from_secs(10), ws.next())
            .await
            .expect("ws frame timeout")
            .expect("ws closed")
            .unwrap();
        if let WsMessage::Text(t) = m {
            let v: Value = serde_json::from_str(&t).unwrap();
            if v["type"] == "floorplan" {
                return v["data"].clone();
            }
        }
    }
}

async fn ws_connect(
    port: u16,
) -> tokio_tungstenite::WebSocketStream<tokio_tungstenite::MaybeTlsStream<TcpStream>> {
    connect_async(format!("ws://127.0.0.1:{port}/ws"))
        .await
        .unwrap()
        .0
}

async fn start_floorplan_session(tool: std::path::PathBuf) -> Server {
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let dir = std::env::temp_dir().join(format!(
        "robot-floorplan-it-{}-{}",
        std::process::id(),
        SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let fake = slam_fake(&dir);
    std::env::set_var("ROBOT_FLOORPLAN_TOOL", &tool);
    let mut server = start_with(None, Some(&fake)).await;
    // Wait until the HTTP listener is actually serving (the harness only
    // checks the child process is alive).
    timeout(Duration::from_secs(10), async {
        loop {
            if TcpStream::connect(("127.0.0.1", server.1)).await.is_ok() {
                break;
            }
            tokio::time::sleep(Duration::from_millis(20)).await;
            assert!(server.0.try_wait().unwrap().is_none(), "server died");
        }
    })
    .await
    .expect("http listener never came up");
    server.2.send(odom()).await.unwrap();
    server.2.send(scan_packet(1.1)).await.unwrap();
    server
}

#[tokio::test]
async fn rebuild_streams_versioned_floorplan_and_replays_on_connect() {
    let server = start_floorplan_session(floorplan_tool()).await;
    let mut w = ws_connect(server.1).await;
    // No rebuild yet: no floorplan frame on a fresh connection.
    let none = timeout(Duration::from_millis(400), w.next()).await;
    assert!(
        none.is_err()
            || !matches!(&none.unwrap(), Some(Ok(WsMessage::Text(t))) if t.contains("\"floorplan\"")),
        "no floorplan before the first rebuild"
    );
    // The grid lands asynchronously after the first scan; a 409 before it
    // is honest, so retry until the SLAM map watch has a version.
    let (status, body) = timeout(Duration::from_secs(10), async {
        loop {
            let (status, body) = http_post(server.1, "/api/floorplan/rebuild", "{}").await;
            if status != 409 {
                return (status, body);
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("map never appeared");
    assert_eq!(status, 200, "rebuild failed: {body}");
    let data = next_floorplan_frame(&mut w).await;
    assert_eq!(data["version"], 1, "first floorplan version must be 1");
    assert!(
        data["rooms"].as_array().is_some_and(|a| !a.is_empty()),
        "floorplan frame must carry rooms: {data}"
    );
    assert!(data["grid_version"].as_u64().is_some());
    // A fresh viewer gets the cached frame without a new rebuild.
    let mut w2 = ws_connect(server.1).await;
    let again = next_floorplan_frame(&mut w2).await;
    assert_eq!(again["version"], 1, "cached floorplan must replay");
    // Rate limit: an immediate second rebuild is refused.
    let (status, body) = http_post(server.1, "/api/floorplan/rebuild", "{}").await;
    assert_eq!(status, 429, "rate limit: {status} {body}");
}

#[tokio::test]
async fn tool_failure_is_honest_and_never_streams_a_frame() {
    let dir = std::env::temp_dir().join(format!("robot-floorplan-bad-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let bad = dir.join("bad-tool");
    std::fs::write(&bad, "#!/bin/sh\ncat >/dev/null\nexit 3\n").unwrap();
    std::fs::set_permissions(&bad, std::fs::Permissions::from_mode(0o700)).unwrap();
    let server = start_floorplan_session(bad).await;
    let mut w = ws_connect(server.1).await;
    let (status, body) = timeout(Duration::from_secs(10), async {
        loop {
            let (status, body) = http_post(server.1, "/api/floorplan/rebuild", "{}").await;
            if status != 409 {
                return (status, body);
            }
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("map never appeared");
    assert_ne!(status, 200, "failing tool must not report success: {body}");
    assert_eq!(
        status, 500,
        "failing tool is an honest error: {status} {body}"
    );
    let stale = timeout(Duration::from_millis(400), w.next()).await;
    assert!(
        stale.is_err()
            || !matches!(&stale.unwrap(), Some(Ok(WsMessage::Text(t))) if t.contains("\"floorplan\"")),
        "no floorplan frame on failure"
    );
}
