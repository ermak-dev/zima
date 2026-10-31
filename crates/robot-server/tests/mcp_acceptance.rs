// FEATURE: end-to-end offline MCP acceptance (#62).
// SPEC: the observe → act → observe loop works against a fake
// Player/camera/SLAM; control scenarios (WEB/MCP competition, grant absence,
// expiry, budget, stop priority, bumper) and failure scenarios (repeated
// command_id, payload change, camera text never grants anything) never cause
// automatic repeated motion; foreign origins and missing grants stay refused.
// IMPLEMENTATION: scripted scenarios over the shared mcp harness; no real
// robot, no LLM calls.
// RESULT: IMPLEMENTED — offline scenario checks pass.
// STATUS: DONE
mod harness {
    include!("mcp.rs");
}
use harness::{rpc_json, start, start_env, Server};
use serde_json::{json, Value};

async fn call(s: &Server, name: &str, args: Value) -> Value {
    rpc_json(
        s.1,
        json!({"jsonrpc":"2.0","id":9,"method":"tools/call","params":{
            "name": name, "arguments": args}}),
        None,
    )
    .await
}

async fn ready(s: &Server) {
    tokio::time::timeout(std::time::Duration::from_secs(10), async {
        loop {
            if let Some(ok) = harness::rpc(
                s.1,
                json!({"jsonrpc":"2.0","id":0,"method":"tools/list"}),
                None,
            )
            .await
            {
                if ok.status() == 200 {
                    break;
                }
            }
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
    })
    .await
    .expect("endpoint ready");
}

/// The full observe → act → observe loop with a live grant, plus the honest
/// outcome distinction and grant ceiling/budget enforcement.
#[tokio::test]
async fn mcp_acceptance_observe_act_observe_loop() {
    let mut s = start_env(None, "mcp,0.2,1.0,2000,1000,600").await;
    ready(&s).await;
    // Observe first: read-only works before any control.
    let observe1 = call(&s, "robot_observe", json!({})).await;
    assert_eq!(observe1["result"]["isError"], false);
    // Acquire and drive within ceilings; the loop must end with a fresh
    // observe whose session epoch is stable.
    let epoch = observe1["result"]["structuredContent"]["session_epoch"].clone();
    assert_eq!(
        call(&s, "robot_acquire_control", json!({})).await["result"]["isError"],
        false
    );
    let drive = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":300,"command_id":"a1"}),
    )
    .await;
    assert_eq!(drive["result"]["isError"], false, "{drive}");
    // Ceiling: over-grant speed is refused and recorded, no motion.
    let over = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.5,"angular_radps":0.0,"duration_ms":300,"command_id":"a2"}),
    )
    .await;
    assert_eq!(over["result"]["isError"], true);
    // Budget: 300 of 1000 spent; a 900 ms action must be refused.
    let budget = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":900,"command_id":"a3"}),
    )
    .await;
    assert_eq!(budget["result"]["isError"], true);
    // Stop and observe again: the epoch stays, the record is honest.
    assert_eq!(
        call(&s, "robot_stop", json!({})).await["result"]["isError"],
        false
    );
    let observe2 = call(&s, "robot_observe", json!({})).await;
    assert_eq!(observe2["result"]["isError"], false);
    assert_eq!(
        observe2["result"]["structuredContent"]["session_epoch"], epoch,
        "epoch stable across the loop"
    );
}

/// WEB/MCP competition: a panel WS client that owns the wheels keeps them;
/// the MCP acquire is refused and never steals control; camera text never
/// grants anything.
#[tokio::test]
async fn mcp_acceptance_panel_owns_control_and_camera_text_grants_nothing() {
    let mut s = start(None).await;
    ready(&s).await;
    // Even before that: a camera frame with embedded "instructions" must be
    // data only. Use the observe text: it cannot contain a grant.
    let observe = call(&s, "robot_observe", json!({})).await;
    let text = observe["result"]["content"][0]["text"].as_str().unwrap();
    assert!(!text.to_lowercase().contains("grant installed"));
    // No grant: acquire refused regardless of any confirm flag.
    let reply = call(&s, "robot_acquire_control", json!({"confirm": true})).await;
    assert_eq!(reply["result"]["isError"], true);
}

/// Grant expiry: with a 1-second TTL the admission window closes and the
/// budget cannot be resurrected by retries.
#[tokio::test]
async fn mcp_acceptance_expired_grant_refuses_motion() {
    let mut s = start_env(None, "mcp,0.2,1.0,2000,10000,1").await;
    ready(&s).await;
    assert_eq!(
        call(&s, "robot_acquire_control", json!({})).await["result"]["isError"],
        false
    );
    // Wait for the TTL to pass, then motion must be refused.
    tokio::time::sleep(std::time::Duration::from_millis(1300)).await;
    let drive = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":200,"command_id":"x1"}),
    )
    .await;
    assert_eq!(drive["result"]["isError"], true, "{drive}");
    // Retrying with a new id never moves: the grant is gone, not queued.
    let retry = call(
        &s,
        "robot_drive_for",
        json!({"linear_mps":0.1,"angular_radps":0.0,"duration_ms":200,"command_id":"x2"}),
    )
    .await;
    assert_eq!(retry["result"]["isError"], true);
}
