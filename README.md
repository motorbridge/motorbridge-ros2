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
motorbridge_ros2 --check-config -c <manifest.yaml>
motorbridge_ros2 --list-topics -c <manifest.yaml>
motorbridge_ros2 --help
motorbridge_ros2 --version
```

The positional form is still supported for compatibility:

```bash
motorbridge_ros2 motorbridge_manifest.yaml
```

Configuration-only checks do not open DDS or motor hardware:

```bash
motorbridge_ros2 --check-config -c motorbridge_manifest.yaml
motorbridge_ros2 --list-topics -c motorbridge_manifest.yaml
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
- Use `PCAN_USBBUS2@1000000` when the adapter is the second PEAK USB channel.

Windows manifest channel example:

```yaml
joints:
  - joint_name: "base_yaw"
    vendor: "damiao"
    transport: "socketcan"
    bus_interface: "PCAN_USBBUS1@1000000"
    motor_id: 0x06
    feedback_id: 0x16
    model: "4340P"
    default_profile: "high_stiffness"

global_safety:
  heartbeat_timeout_ms: 100
  emergency_stop_topic: "/sys/estop"
  emergency_stop_json_topic: "/sys/estop_json"
  watchdog_strategy: "hold"

ros_bridge_options:
  namespace: "motorbridge"
  qos_profile: "reliable"
  status_topic: "bridge_status_json"
  enable_easter_counter: true
  state_publish_period_ms: 20
  feedback_warn_ms: 2000
```

Windows Damiao serial bridge example:

```yaml
joints:
  - joint_name: "base_yaw"
    vendor: "damiao"
    transport: "dm-serial"
    bus_interface: "COM3"
    serial_baud: 921600
    motor_id: 0x06
    feedback_id: 0x16
    model: "4340P"
    default_profile: "high_stiffness"
```

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
- Do not put bitrate into `bus_interface` on Linux; configure bitrate with `ip link` first.

Linux SocketCAN setup example:

```bash
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Linux manifest channel example:

```yaml
joints:
  - joint_name: "base_yaw"
    vendor: "damiao"
    transport: "socketcan"
    bus_interface: "can0"
    motor_id: 0x06
    feedback_id: 0x16
    model: "4340P"
    default_profile: "high_stiffness"
```

Linux CAN-FD manifest example:

```yaml
joints:
  - joint_name: "ankle"
    vendor: "hexfellow"
    transport: "socketcanfd"
    bus_interface: "can0"
    motor_id: 0x01
    feedback_id: 0x01
    model: "hexfellow"
    default_profile: "high_stiffness"
```

Linux Damiao serial bridge example:

```yaml
joints:
  - joint_name: "base_yaw"
    vendor: "damiao"
    transport: "dm-serial"
    bus_interface: "/dev/ttyACM0"
    serial_baud: 921600
    motor_id: 0x06
    feedback_id: 0x16
    model: "4340P"
    default_profile: "high_stiffness"
```

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
- Use the same manifest fields as other platforms, but set `bus_interface` to the backend endpoint expected by your MotorBridge adapter.

macOS manifest examples:

```yaml
joints:
  - joint_name: "base_yaw"
    vendor: "damiao"
    transport: "socketcan"
    bus_interface: "can0"
    motor_id: 0x06
    feedback_id: 0x16
    model: "4340P"
    default_profile: "high_stiffness"
```

```yaml
joints:
  - joint_name: "base_yaw"
    vendor: "damiao"
    transport: "dm-serial"
    bus_interface: "/dev/tty.usbmodem1101"
    serial_baud: 921600
    motor_id: 0x06
    feedback_id: 0x16
    model: "4340P"
    default_profile: "high_stiffness"
```

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

Each joint chooses a vendor, transport, and platform-specific bus endpoint:

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

`bus_interface` meaning by platform:

- Windows PCAN: `PCAN_USBBUS1@1000000`, `PCAN_USBBUS2@1000000`; `can0` and `can1` are accepted aliases.
- Windows serial bridge: `COM3`, `COM4`, etc. with `transport: "dm-serial"`.
- Linux SocketCAN: `can0`, `can1`, `slcan0`; bitrate is configured outside the manifest.
- Linux serial bridge: `/dev/ttyACM0`, `/dev/ttyUSB0`, etc. with `transport: "dm-serial"`.
- macOS: adapter-specific endpoint, for example `can0` if your backend exposes it, or `/dev/tty.usbmodemXXXX` for serial bridge.

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
- `/<joint>/event_json`: publish, `std_msgs/msg/String`
- `/<joint>/cmd`: typed CDR command topic
- `/<joint>/state`: typed CDR state topic

Global diagnostic and safety topics:

- `/easter_counter`
- `/bridge_status_json`
- `/sys/estop_json`: subscribe, `std_msgs/msg/String`
- `/sys/estop`: typed internal emergency stop topic

If `ros_bridge_options.namespace` is set, relative topics are exposed under that namespace. For example with `namespace: "motorbridge"`, use `/motorbridge/base_yaw/cmd_json`. Absolute topics such as `/sys/estop_json` stay absolute.

Command examples:

```bash
ros2 topic pub --once /motorbridge/base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"enable\",\"request_id\":\"test-001\"}'}"
ros2 topic pub --once /motorbridge/base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"pos_vel\",\"pos\":0.5,\"vlim\":0.5,\"request_id\":\"test-002\"}'}"
ros2 topic pub --once /motorbridge/base_yaw/cmd_json std_msgs/msg/String "{data: '{\"op\":\"disable\",\"request_id\":\"test-003\"}'}"
ros2 topic pub --once /sys/estop_json std_msgs/msg/String "{data: '{\"engaged\":true}'}"
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

Event output on `/<joint>/event_json`:

- `command_applied`: command accepted by MotorBridge ABI
- `command_failed`: command rejected or ABI returned an error
- `parse_error`: JSON command could not be parsed
- `motor_added`: lazy ABI motor handle was created
- `watchdog`: heartbeat timeout action
- `estop`: emergency stop command received
- `no_feedback`: motor handle exists but feedback has not arrived

Bridge status on `/bridge_status_json` publishes once per second and includes:

- app/version/target metadata
- active controller count
- per-joint connected/enabled state
- command/error counters
- last error
- last command age

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
