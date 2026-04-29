use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorCommand {
    pub op: String,
    pub pos: Option<f32>,
    pub vel: Option<f32>,
    pub kp: Option<f32>,
    pub kd: Option<f32>,
    pub tau: Option<f32>,
    pub vlim: Option<f32>,
    pub ratio: Option<f32>,
    pub continuous: Option<bool>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorState {
    pub joint_name: String,
    pub enabled: bool,
    pub pos: Option<f32>,
    pub vel: Option<f32>,
    pub torque: Option<f32>,
    pub t_mos: Option<f32>,
    pub t_rotor: Option<f32>,
    pub status_code: Option<u8>,
    pub status_name: Option<String>,
    pub ts_millis: i64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EStopMessage {
    pub engaged: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RosString {
    pub data: String,
}
