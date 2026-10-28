use std::{env, path::PathBuf, process::Command};

fn tool(name: &str, default: &str) -> String {
    let target = env::var("TARGET").unwrap();
    env::var(format!("{name}_{}", target.replace('-', "_")))
        .or_else(|_| env::var(format!("{name}_{target}")))
        .or_else(|_| env::var(format!("TARGET_{name}")))
        .or_else(|_| env::var(name))
        .unwrap_or_else(|_| default.into())
}

fn main() {
    println!("cargo:rustc-check-cfg=cfg(supervisor_audio)");
    println!("cargo:rerun-if-changed=../../bin/audio/robot_audio.c");
    let target = env::var("TARGET").unwrap();
    for key in ["CC", "AR"] {
        for name in [
            key.to_string(),
            format!("TARGET_{key}"),
            format!("{key}_{target}"),
            format!("{key}_{}", target.replace('-', "_")),
        ] {
            println!("cargo:rerun-if-env-changed={name}");
        }
    }
    // Keep the old static mask-owner build independent of glibc and dlopen.
    // Production supervisor audio is built with the GNU target, never musl.
    if env::var("CARGO_CFG_TARGET_ENV").unwrap() == "musl" {
        return;
    }
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let object = out.join("robot_audio.o");
    let mut cc = Command::new(tool("CC", "cc"));
    cc.args([
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-std=c11",
        "-fPIC",
        "-Dmain=robot_audio_main",
    ]);
    if env::var_os("CARGO_FEATURE_AUDIO_TESTING").is_some() {
        assert_eq!(
            env::var("HOST").unwrap(),
            env::var("TARGET").unwrap(),
            "audio-testing is native-only"
        );
        cc.arg("-DROBOT_AUDIO_TESTING");
    }
    assert!(cc
        .arg("-c")
        .arg("../../bin/audio/robot_audio.c")
        .arg("-o")
        .arg(&object)
        .status()
        .expect("C compiler")
        .success());
    assert!(Command::new(tool("AR", "ar"))
        .arg("crs")
        .arg(out.join("librobot_audio.a"))
        .arg(object)
        .status()
        .expect("archiver")
        .success());
    println!("cargo:rustc-cfg=supervisor_audio");
    println!("cargo:rustc-link-search=native={}", out.display());
    println!("cargo:rustc-link-lib=static=robot_audio");
    println!("cargo:rustc-link-lib=dl");
    println!("cargo:rustc-link-lib=m");
}
