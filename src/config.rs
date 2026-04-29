#![allow(dead_code)]

use serde::Deserialize;

#[derive(Debug, Deserialize, Clone)]
pub struct Manifest {
    pub target_name: String,
    pub version: String,
    pub control_profiles: Vec<ControlProfile>,
    pub joints: Vec<JointConfig>,
    pub global_safety: GlobalSafety,
    pub ros_bridge_options: RosBridgeOptions,
}

#[derive(Debug, Deserialize, Clone)]
pub struct ControlProfile {
    pub profile_name: String,
    pub mode: String,
    pub kp: Option<f32>,
    pub kd: Option<f32>,
    pub max_torque: Option<f32>,
    pub max_velocity: Option<f32>,
    pub max_acceleration: Option<f32>,
    pub interpolation: Option<String>,
    pub on_loss_of_comm: Option<String>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct JointConfig {
    pub joint_name: String,
    pub vendor: Option<String>,
    pub transport: Option<String>,
    pub bus_interface: String,
    pub serial_baud: Option<u32>,
    pub motor_id: u16,
    pub feedback_id: Option<u16>,
    pub interface_type: Option<String>,
    pub direction: Option<i8>,
    pub pos_offset: Option<f32>,
    pub pos_limit: Option<[f32; 2]>,
    pub default_profile: String,
    pub model: Option<String>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct GlobalSafety {
    pub heartbeat_timeout_ms: u64,
    pub emergency_stop_topic: String,
    #[serde(default = "default_estop_json_topic")]
    pub emergency_stop_json_topic: String,
    pub watchdog_strategy: String,
}

#[derive(Debug, Deserialize, Clone)]
pub struct RosBridgeOptions {
    pub namespace: String,
    pub qos_profile: String,
    #[serde(default = "default_status_topic")]
    pub status_topic: String,
    #[serde(default = "default_enable_easter_counter")]
    pub enable_easter_counter: bool,
    #[serde(default = "default_state_publish_period_ms")]
    pub state_publish_period_ms: u64,
    #[serde(default = "default_feedback_warn_ms")]
    pub feedback_warn_ms: u64,
}

fn default_estop_json_topic() -> String {
    "/sys/estop_json".to_string()
}

fn default_status_topic() -> String {
    "bridge_status_json".to_string()
}

fn default_enable_easter_counter() -> bool {
    true
}

fn default_state_publish_period_ms() -> u64 {
    20
}

fn default_feedback_warn_ms() -> u64 {
    2000
}
