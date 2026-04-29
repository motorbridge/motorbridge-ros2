# motorbridge-ros2

`motorbridge-ros2` is a ROS2/DDS bridge built on:

- `motorbridge` core (`motor_core` + `motor_vendor_damiao`)
- `RustDDS` (ROS2-compatible DDS transport)
- `zenoh` (pinned in workspace for dependency alignment)

## 1) Dependency modes

This repository supports two reproducible modes.

### Mode A: git submodule (recommended)

```bash
git clone <repo-url> motorbridge-ros2
cd motorbridge-ros2
git submodule update --init --recursive
```

### Mode B: custom local source paths

Use your own local clones of `motorbridge` / `RustDDS` / `zenoh`:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\bootstrap.ps1 `
  -MotorbridgeDir C:\src\motorbridge `
  -RustDDSDir C:\src\RustDDS `
  -ZenohDir C:\src\zenoh
```

## 2) Platform build and run

### Release binary usage (all platforms)

After `cargo build --release`, run the binary directly:

- Windows: `target\release\motorbridge_ros2.exe`
- Linux/macOS: `./target/release/motorbridge_ros2`

Supported CLI:

- `-h`, `--help`: print usage
- `-V`, `--version`: print binary version
- `[manifest.yaml]`: optional manifest path (default: `motorbridge_manifest.yaml`)

### Windows

#### Prerequisites

- Rust toolchain (MSVC target)
- Visual Studio C++ build tools (`link.exe`)
- Npcap SDK (`Packet.lib`) for transitive dependency link
- PEAK PCAN driver/runtime when using real CAN hardware

#### Build

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_windows_prereq.ps1
cargo build --release
```

#### Run

```powershell
cargo run --release -- motorbridge_manifest.yaml
```

or direct binary:

```powershell
.\target\release\motorbridge_ros2.exe motorbridge_manifest.yaml
.\target\release\motorbridge_ros2.exe --help
.\target\release\motorbridge_ros2.exe --version
```

Notes:

- In this project, Windows CAN is normally configured as `PCAN_USBBUS1@1000000`.
- `can0` is also accepted by motorbridge core and maps to `PCAN_USBBUS1`.
- If rebuild fails with `Access is denied` for `motorbridge_ros2.exe`, stop old process first.

### Ubuntu (Linux)

#### Prerequisites

- Rust toolchain
- CAN interface ready (for example SocketCAN `can0`)
- For ROS2 CLI testing: ROS2 environment sourced on test machine

#### Build

```bash
cargo build --release
```

#### Run

```bash
cargo run --release -- motorbridge_manifest.yaml
```

or direct binary:

```bash
./target/release/motorbridge_ros2 motorbridge_manifest.yaml
./target/release/motorbridge_ros2 --help
./target/release/motorbridge_ros2 --version
```

Notes:

- On Linux, `bus_interface` should use real interface names like `can0` / `can1`.
- Bring interface up before run (for example with `ip link`).

### macOS

#### Prerequisites

- Rust toolchain
- Xcode command line tools

#### Build

```bash
cargo build --release
```

#### Run

```bash
cargo run --release -- motorbridge_manifest.yaml
```

or direct binary:

```bash
./target/release/motorbridge_ros2 motorbridge_manifest.yaml
./target/release/motorbridge_ros2 --help
./target/release/motorbridge_ros2 --version
```

Notes:

- Binary can build on macOS.
- Real motor CAN control depends on available backend/hardware compatibility in your setup.

## 3) Generated ABI artifacts

`build.rs` also builds `motorbridge` package `motor_abi` and copies ABI artifacts into `abi/`:

- Windows: `abi/motor_abi.dll`
- Linux: `abi/libmotor_abi.so`
- macOS: `abi/libmotor_abi.dylib`

These are generated files and are ignored by git.

Optional runtime override:

```bash
MOTORBRIDGE_ABI_PATH=/path/to/libmotor_abi.so cargo run --release -- motorbridge_manifest.yaml
```

Windows PowerShell:

```powershell
$env:MOTORBRIDGE_ABI_PATH = "C:\path\to\motor_abi.dll"
cargo run --release -- motorbridge_manifest.yaml
```

See also: `abi/README.md`.

## 4) Manifest and motor mapping

Default manifest:

- `motorbridge_manifest.yaml`

Current default `base_yaw` config:

- vendor: `damiao`
- bus: `PCAN_USBBUS1@1000000`
- motor id: `0x06`
- feedback id: `0x16`
- model: `4340P`

Important:

- `enable` / `disable` only change state.
- Visible motion requires movement commands (for example `pos_vel` or `mit`).

## 5) ROS2 topics

Per joint:

- `/<joint>/cmd_json` (subscribe, `std_msgs/msg/String`)
- `/<joint>/state_json` (publish, `std_msgs/msg/String`)
- `/<joint>/cmd` and `/<joint>/state` (typed CDR topics)

Diagnostic:

- `/easter_counter` (published every second)

## 6) Quick ROS2 command test

Enable:

```bash
ros2 topic pub --once /base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"enable\"}'}"
```

Move to 0.5 rad:

```bash
ros2 topic pub --once /base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"pos_vel\",\"pos\":0.5,\"vlim\":0.5}'}"
```

Disable:

```bash
ros2 topic pub --once /base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"disable\"}'}"
```

## 7) Version pinning

Pinned dependency commits are documented in:

- `DEPENDENCIES.lock.md`

When updating submodule revisions, update `DEPENDENCIES.lock.md` in the same commit.

## 8) CI

GitHub Actions CI is included at:

- `.github/workflows/ci.yml`

Current CI runs `cargo check` on:

- `ubuntu-latest`
- `macos-latest`

Windows CI note:

- This project transitively links `Packet.lib` on Windows.
- Hosted runners usually do not provide Npcap SDK by default.
- For Windows CI, use a self-hosted runner with Npcap SDK installed (or run `scripts/check_windows_prereq.ps1` in your internal pipeline before build).
