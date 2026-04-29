use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR"));
    let motorbridge_dir = env::var("MOTORBRIDGE_SRC_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| manifest_dir.join("third_party").join("motorbridge"));

    println!("cargo:rerun-if-env-changed=MOTORBRIDGE_SRC_DIR");
    println!("cargo:rerun-if-changed={}", motorbridge_dir.join("Cargo.toml").display());
    let fallback_motorbridge_dir = manifest_dir.join("..").join("motorbridge");
    let motorbridge_dir = if motorbridge_dir.exists() { motorbridge_dir } else { fallback_motorbridge_dir };

    let profile = env::var("PROFILE").unwrap_or_else(|_| "debug".to_string());
    let mut cmd = Command::new("cargo");
    cmd.arg("build").arg("-p").arg("motor_abi");
    if profile == "release" {
        cmd.arg("--release");
    }
    cmd.current_dir(&motorbridge_dir);

    let status = cmd.status().expect("failed to invoke cargo for motorbridge/motor_abi");
    if !status.success() {
        panic!("building motor_abi failed in {}", motorbridge_dir.display());
    }

    let lib_name = if cfg!(target_os = "windows") {
        "motor_abi.dll"
    } else if cfg!(target_os = "macos") {
        "libmotor_abi.dylib"
    } else {
        "libmotor_abi.so"
    };

    let from = motorbridge_dir
        .join("target")
        .join(&profile)
        .join(lib_name);
    if !from.exists() {
        panic!("motor_abi artifact not found: {}", from.display());
    }

    let abi_dir = manifest_dir.join("abi");
    fs::create_dir_all(&abi_dir).expect("create abi dir");
    let to = abi_dir.join(lib_name);
    fs::copy(&from, &to).expect("copy motor_abi artifact");

    let rel_default = Path::new("abi").join(lib_name);
    println!(
        "cargo:rustc-env=MOTORBRIDGE_ABI_DEFAULT_PATH={}",
        rel_default.display()
    );
}

