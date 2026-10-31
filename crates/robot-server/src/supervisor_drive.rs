//! Supervisor DRIVE/STOP client for the MCP motion tools (#64): bounded
//! motions must run under the robot-local deadline, not the host lease.
//! One short-lived authenticated connection per command, like COMPASS/AUDIO.
use crate::Failure;
use std::time::Duration;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt, BufReader},
    net::TcpStream,
    time::timeout,
};

/// One bounded drive command on the supervisor. Returns its reply line
/// (DRIVING / DRIVE_BUSY / DRIVE_FAILED / INVALID_DRIVE).
pub async fn drive(
    addr: &str,
    token: &str,
    vx: f64,
    vyaw: f64,
    ms: u64,
) -> Result<String, Failure> {
    let line = command(addr, token, &format!("DRIVE {vx} {vyaw} {ms}")).await?;
    Ok(line)
}

/// Priority stop on the supervisor.
pub async fn stop(addr: &str, token: &str) -> Result<String, Failure> {
    command(addr, token, "STOP").await
}

async fn command(addr: &str, token: &str, request: &str) -> Result<String, Failure> {
    let stream = timeout(Duration::from_secs(5), TcpStream::connect(addr))
        .await
        .map_err(|_| "supervisor drive connection timed out".to_string())?
        .map_err(|_| "supervisor drive connection failed".to_string())?;
    let (rx, mut tx) = stream.into_split();
    let mut reader = BufReader::new(rx);
    if !token.is_empty() {
        timeout(
            Duration::from_secs(5),
            tx.write_all(format!("AUTH {token}\n").as_bytes()),
        )
        .await
        .map_err(|_| "supervisor auth timed out".to_string())??;
        let auth = timeout(Duration::from_secs(5), line(&mut reader)).await;
        match auth {
            Ok(Ok(l)) if l == "AUTHENTICATED" => {}
            Ok(Ok(l)) => return Err(format!("supervisor auth refused: {l}").into()),
            _ => return Err("supervisor auth failed".into()),
        }
    }
    timeout(
        Duration::from_secs(10),
        tx.write_all(format!("{request}\n").as_bytes()),
    )
    .await
    .map_err(|_| "supervisor request timed out".to_string())??;
    // DRIVE waits for the velocity write confirmation; bounded well below the
    // action duration itself.
    timeout(Duration::from_secs(10), line(&mut reader))
        .await
        .map_err(|_| "supervisor drive reply timed out".to_string())?
        .map_err(|e| -> Failure { e.into() })
}

async fn line(reader: &mut BufReader<tokio::net::tcp::OwnedReadHalf>) -> Result<String, Failure> {
    let mut bytes = Vec::new();
    let mut byte = [0u8; 1];
    loop {
        reader.read_exact(&mut byte).await?;
        if byte[0] == b'\n' {
            return Ok(String::from_utf8_lossy(&bytes).trim().to_owned());
        }
        bytes.push(byte[0]);
        if bytes.len() > 128 {
            return Err("supervisor reply too long".into());
        }
    }
}
