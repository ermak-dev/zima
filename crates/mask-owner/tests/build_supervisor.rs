//! FEATURE: production build selects GNU/glibc without changing musl config.
//! SPEC: build only the supervisor, honor --output, preserve existing config.
use std::os::unix::fs::PermissionsExt;
use std::process::Command;

#[test]
fn build_script_selects_single_gnu_binary_and_copies_output() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let dir = std::env::temp_dir().join(format!("supervisor-build-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let fake = dir.join("cargo");
    std::fs::write(&fake, r#"#!/bin/sh
set -eu
printf '%s\n' "$@" > "$BUILD_TEST_LOG"
test -x "$CC_armv7_unknown_linux_gnueabihf"
test -x "$CARGO_TARGET_ARMV7_UNKNOWN_LINUX_GNUEABIHF_LINKER"
mkdir -p "$CARGO_TARGET_DIR/armv7-unknown-linux-gnueabihf/release"
printf 'single ELF fixture' > "$CARGO_TARGET_DIR/armv7-unknown-linux-gnueabihf/release/robot-supervisor"
"#).unwrap();
    std::fs::set_permissions(&fake, std::fs::Permissions::from_mode(0o755)).unwrap();
    let config = root.join("crates/.cargo/config.toml");
    let before = std::fs::read(&config).unwrap();
    let output = dir.join("output/supervisor");
    let result = Command::new("bash")
        .arg(root.join("tools/build-supervisor.sh"))
        .arg("--output")
        .arg(&output)
        .env(
            "PATH",
            format!("{}:{}", dir.display(), std::env::var("PATH").unwrap()),
        )
        .env("CARGO_TARGET_DIR", dir.join("target"))
        .env("BUILD_TEST_LOG", dir.join("args"))
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        std::fs::read_to_string(output).unwrap(),
        "single ELF fixture"
    );
    let args = std::fs::read_to_string(dir.join("args")).unwrap();
    assert!(args.contains("--target\narmv7-unknown-linux-gnueabihf\n"));
    assert!(args.contains("--bin\nrobot-supervisor\n"));
    assert!(!args.contains("audio-testing"));
    assert_eq!(std::fs::read(config).unwrap(), before);
    std::fs::remove_dir_all(dir).unwrap();
}
