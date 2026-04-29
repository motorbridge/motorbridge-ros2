# motorbridge-ros2

`motorbridge-ros2` is a ROS2/DDS bridge based on:

- `motorbridge` core (`motor_core` + `motor_vendor_damiao`)
- `RustDDS` (ROS2-compatible DDS transport)
- optional `zenoh` source pin (for aligned dependency workspace management)

The project supports two reproducible dependency modes:

1. git submodule mode (default, recommended)
2. custom source path mode (local development)

## 1) Quick start (submodule mode)

```powershell
git clone <this-repo-url> motorbridge-ros2
cd motorbridge-ros2
git submodule update --init --recursive
cargo run --release -- motorbridge_manifest.yaml
```

## 2) Quick start (custom source path mode)

Use local repositories instead of submodules:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\bootstrap.ps1 `
  -MotorbridgeDir C:\src\motorbridge `
  -RustDDSDir C:\src\RustDDS `
  -ZenohDir C:\src\zenoh
```

This creates `third_party/*` junction links and builds the project.

## 3) Build behavior

- `cargo build` / `cargo run` builds this bridge and also builds `motorbridge` package `motor_abi` through `build.rs`.
- ABI artifacts are copied to local `abi/`:
  - Windows: `abi/motor_abi.dll`
  - Linux: `abi/libmotor_abi.so`
  - macOS: `abi/libmotor_abi.dylib`
- ABI files are generated outputs and are ignored by git.

Optional override:

```powershell
$env:MOTORBRIDGE_ABI_PATH = "C:\path\to\motor_abi.dll"
```

## 4) Manifest and motor mapping

Default manifest file:

- `motorbridge_manifest.yaml`

Current default joint example:

- joint: `base_yaw`
- vendor: `damiao`
- bus: `PCAN_USBBUS1@1000000` (Windows explicit channel)
- motor id: `0x06`
- feedback id: `0x16`
- model: `4340P`

Notes:

- On Windows PCAN backend, `can0` maps to `PCAN_USBBUS1`, `can1` maps to `PCAN_USBBUS2`.
- `enable`/`disable` are state commands; visible movement requires position/velocity or MIT control commands.

## 5) ROS2 topics

Per-joint topics:

- `/<joint>/cmd_json` (subscribe, `std_msgs/msg/String`)
- `/<joint>/state_json` (publish, `std_msgs/msg/String`)
- `/<joint>/cmd` / `/<joint>/state` (typed CDR topics)

Diagnostic topic:

- `/easter_counter` (publish every second)

## 6) Common checks

Windows prerequisite check:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_windows_prereq.ps1
```

If `cargo build --release` fails with access denied on `motorbridge_ros2.exe`, stop existing process first.

## 7) Version pinning

Pinned dependency commits are documented in:

- `DEPENDENCIES.lock.md`

When updating submodule revisions, update `DEPENDENCIES.lock.md` in the same commit.
