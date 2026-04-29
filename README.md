# MotorBridge-ROS2

MotorBridge-ROS2 is a native ROS2/DDS bridge for MotorBridge.

The bridge does not require a local ROS2 installation. It uses RustDDS to expose ROS2-compatible topics, and delegates all motor behavior to the MotorBridge ABI.

## Features

- Native Windows / Linux / macOS build
- ROS2 topic discovery over DDS
- Runtime manifest configuration
- Unified JSON command topics
- MotorBridge ABI backend
- Vendor-neutral motor support

Supported vendors:

- `damiao`
- `robstride`
- `myactuator`
- `hexfellow`
- `hightorque`

Supported transports:

- `socketcan`
- `socketcanfd`
- `dm-serial`

## Clone

Recommended submodule mode:

```bash
git clone <repo-url> motorbridge-ros2
cd motorbridge-ros2
git submodule update --init --recursive
```

Custom local source mode:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\bootstrap.ps1 `
  -MotorbridgeDir C:\src\motorbridge `
  -RustDDSDir C:\src\RustDDS `
  -ZenohDir C:\src\zenoh
```

## Unified Run Command

Use the same command style on every platform:

```bash
motorbridge_ros2 -c motorbridge_manifest.yaml
```

CLI options:

```text
motorbridge_ros2 -c <manifest.yaml>
motorbridge_ros2 --config <manifest.yaml>
motorbridge_ros2 --help
motorbridge_ros2 --version
```

The positional form is still supported for compatibility:

```bash
motorbridge_ros2 motorbridge_manifest.yaml
```

## Windows

Prerequisites:

- Rust MSVC toolchain
- Visual Studio C++ build tools
- Npcap SDK, providing `Packet.lib`
- PEAK PCAN driver/runtime for real PCAN hardware

Check prerequisites and build:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\check_windows_prereq.ps1
cargo build --release
```

Run from the repository root:

```powershell
.\target\release\motorbridge_ros2.exe -c motorbridge_manifest.yaml
```

Development run:

```powershell
cargo run --release -- -c motorbridge_manifest.yaml
```

Windows CAN notes:

- `PCAN_USBBUS1@1000000` is the recommended explicit PCAN channel format.
- `can0` also maps to `PCAN_USBBUS1` inside MotorBridge core.
- `can1` maps to `PCAN_USBBUS2`.

## Ubuntu / Linux

Prerequisites:

- Rust toolchain
- SocketCAN or CAN-FD interface configured
- ROS2 is optional locally; it is only needed on machines that use `ros2 topic` CLI tools

Build:

```bash
cargo build --release
```

Run:

```bash
./target/release/motorbridge_ros2 -c motorbridge_manifest.yaml
```

Development run:

```bash
cargo run --release -- -c motorbridge_manifest.yaml
```

Linux CAN notes:

- Use real interface names such as `can0`, `can1`, or `slcan0`.
- Configure bitrate using Linux networking tools before starting the bridge.
- For CAN-FD devices, set `transport: "socketcanfd"` in the manifest.

## macOS

Prerequisites:

- Rust toolchain
- Xcode command line tools

Build:

```bash
cargo build --release
```

Run:

```bash
./target/release/motorbridge_ros2 -c motorbridge_manifest.yaml
```

Development run:

```bash
cargo run --release -- -c motorbridge_manifest.yaml
```

macOS note:

- The binary can build natively.
- Real motor control depends on the MotorBridge hardware backend available for your adapter.

## Release Package Layout

A binary release should include:

```text
motorbridge_ros2[.exe]
motorbridge_manifest.yaml
abi/
  motor_abi.dll          # Windows
  libmotor_abi.so        # Linux
  libmotor_abi.dylib     # macOS
```

The build script builds `motorbridge` package `motor_abi` and copies ABI artifacts into `abi/` during local builds. ABI binaries are generated files and are ignored by git.

ABI lookup order:

- `MOTORBRIDGE_ABI_PATH`
- build-time default path, usually `abi/<platform-library>`
- `abi/<platform-library>` next to the executable
- `<platform-library>` next to the executable
- `abi/<platform-library>` from the current working directory

Override ABI path when needed:

```bash
MOTORBRIDGE_ABI_PATH=/path/to/libmotor_abi.so ./motorbridge_ros2 -c motorbridge_manifest.yaml
```

PowerShell:

```powershell
$env:MOTORBRIDGE_ABI_PATH = "C:\path\to\motor_abi.dll"
.\motorbridge_ros2.exe -c motorbridge_manifest.yaml
```

## Manifest

Default config:

- `motorbridge_manifest.yaml`

Each joint chooses a vendor and transport:

```yaml
joints:
  - joint_name: "base_yaw"
    vendor: "damiao"
    transport: "socketcan"
    bus_interface: "PCAN_USBBUS1@1000000"
    serial_baud: 921600          # only used by transport: "dm-serial"
    motor_id: 0x06
    feedback_id: 0x16
    model: "4340P"
    default_profile: "high_stiffness"
```

Transport defaults:

- `hexfellow` defaults to `socketcanfd`
- all other vendors default to `socketcan`

Model defaults when `model` is omitted:

- `damiao`: `4340P`
- `robstride`: `rs-00`
- `myactuator`: `X8`
- `hexfellow`: `hexfellow`
- `hightorque`: `hightorque`

Transport aliases accepted by the bridge:

- `socketcan`, `can`, `auto`
- `socketcanfd`, `canfd`
- `dm-serial`, `dm_serial`, `dmserial`, `serial`

Vendor examples:

```yaml
- joint_name: "robstride_joint"
  vendor: "robstride"
  transport: "socketcan"
  bus_interface: "can0"
  motor_id: 0x14
  feedback_id: 0xFD
  model: "rs-00"
  default_profile: "high_stiffness"

- joint_name: "myactuator_joint"
  vendor: "myactuator"
  transport: "socketcan"
  bus_interface: "can0"
  motor_id: 0x01
  feedback_id: 0x241
  model: "X8"
  default_profile: "high_stiffness"

- joint_name: "hexfellow_joint"
  vendor: "hexfellow"
  transport: "socketcanfd"
  bus_interface: "can0"
  motor_id: 0x01
  feedback_id: 0x01
  model: "hexfellow"
  default_profile: "high_stiffness"

- joint_name: "hightorque_joint"
  vendor: "hightorque"
  transport: "socketcan"
  bus_interface: "can0"
  motor_id: 0x01
  feedback_id: 0x01
  model: "hightorque"
  default_profile: "high_stiffness"
```

## ROS2 Topics

Per joint:

- `/<joint>/cmd_json`: subscribe, `std_msgs/msg/String`
- `/<joint>/state_json`: publish, `std_msgs/msg/String`
- `/<joint>/cmd`: typed CDR command topic
- `/<joint>/state`: typed CDR state topic

Diagnostic topic:

- `/easter_counter`

Command examples:

```bash
ros2 topic pub --once /base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"enable\"}'}"
ros2 topic pub --once /base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"pos_vel\",\"pos\":0.5,\"vlim\":0.5}'}"
ros2 topic pub --once /base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"disable\"}'}"
```

Supported command operations:

- `enable`
- `disable`
- `clear_error`
- `set_zero`
- `store_parameters`
- `mit`
- `pos_vel`
- `vel`
- `force_pos`

MotorBridge ABI may report an error when an operation is not supported by a vendor.

State output:

- `pos`: rad
- `vel`: rad/s
- `torque`: Nm when provided by the vendor backend
- `status_code`: vendor-normalized ABI status byte

## CI

GitHub Actions CI is defined in:

- `.github/workflows/ci.yml`

The default CI runs `cargo check` on:

- `ubuntu-latest`
- `macos-latest`

Windows CI needs a runner with Npcap SDK installed because the dependency chain links `Packet.lib`.

## Dependency Pins

Pinned submodule commits are documented in:

- `DEPENDENCIES.lock.md`
