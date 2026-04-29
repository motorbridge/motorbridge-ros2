use std::collections::HashMap;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, Result};
use rustdds::{no_key, CDRDeserializerAdapter, CDRSerializerAdapter};

use crate::abi::{normalize_transport, normalize_vendor, AbiController, AbiMotor, MotorAbi};
use crate::cli::{APP_NAME, APP_VERSION};
use crate::config::{self, ControlProfile, Manifest};
use crate::types::{MotorCommand, RosString};

pub struct JointRuntime {
    pub cfg: config::JointConfig,
    pub profile: ControlProfile,
    pub motor: Option<AbiMotor>,
    pub reader: no_key::DataReader<MotorCommand, CDRDeserializerAdapter<MotorCommand>>,
    pub json_reader: no_key::DataReader<RosString, CDRDeserializerAdapter<RosString>>,
    pub writer: no_key::DataWriter<
        crate::types::MotorState,
        CDRSerializerAdapter<crate::types::MotorState>,
    >,
    pub json_state_writer: no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    pub json_event_writer: no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    pub enabled: bool,
    pub active_cmd: Option<MotorCommand>,
    pub last_cmd_at: Instant,
    pub last_state_log_at: Instant,
    pub last_feedback_warn_at: Instant,
    pub last_bus_error_log_at: Instant,
    pub last_state_publish_at: Instant,
    pub command_count: u64,
    pub error_count: u64,
    pub last_error: Option<String>,
}

impl JointRuntime {
    pub fn new(
        cfg: config::JointConfig,
        profile: ControlProfile,
        reader: no_key::DataReader<MotorCommand, CDRDeserializerAdapter<MotorCommand>>,
        json_reader: no_key::DataReader<RosString, CDRDeserializerAdapter<RosString>>,
        writer: no_key::DataWriter<
            crate::types::MotorState,
            CDRSerializerAdapter<crate::types::MotorState>,
        >,
        json_state_writer: no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
        json_event_writer: no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    ) -> Self {
        Self {
            cfg,
            profile,
            motor: None,
            reader,
            json_reader,
            writer,
            json_state_writer,
            json_event_writer,
            enabled: false,
            active_cmd: None,
            last_cmd_at: Instant::now(),
            last_state_log_at: Instant::now(),
            last_feedback_warn_at: Instant::now(),
            last_bus_error_log_at: Instant::now(),
            last_state_publish_at: Instant::now(),
            command_count: 0,
            error_count: 0,
            last_error: None,
        }
    }
}

pub fn drain_json_commands(
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    rt: &mut JointRuntime,
) {
    while let Ok(Some(sample)) = rt.json_reader.take_next_sample() {
        let raw = sample.value().data.clone();
        println!("[cmd] joint={} via=json raw={}", rt.cfg.joint_name, raw);
        match serde_json::from_str::<MotorCommand>(&raw) {
            Ok(cmd) => {
                if let Err(err) = apply_command(abi, controllers, rt, &cmd) {
                    let err_s = err.to_string();
                    eprintln!(
                        "[cmd] joint={} via=json apply_error={}",
                        rt.cfg.joint_name, err_s
                    );
                    rt.error_count += 1;
                    rt.last_error = Some(err_s.clone());
                    publish_command_event(rt, "error", "command_failed", &cmd, false, Some(&err_s));
                } else {
                    println!(
                        "[cmd] joint={} via=json apply_ok op={}",
                        rt.cfg.joint_name, cmd.op
                    );
                    rt.command_count += 1;
                    rt.last_cmd_at = Instant::now();
                    publish_command_event(rt, "info", "command_applied", &cmd, true, None);
                    rt.active_cmd = cmd.continuous.unwrap_or(false).then_some(cmd);
                }
            }
            Err(err) => {
                eprintln!(
                    "[cmd] joint={} via=json parse_error={} raw={}",
                    rt.cfg.joint_name, err, raw
                );
                rt.error_count += 1;
                rt.last_error = Some(format!("json parse error: {err}"));
                publish_json_event(
                    rt,
                    "error",
                    "parse_error",
                    &format!("json parse error: {err}"),
                );
            }
        }
    }
}

pub fn apply_command(
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    rt: &mut JointRuntime,
    cmd: &MotorCommand,
) -> Result<()> {
    ensure_motor(abi, controllers, rt)?;
    let motor = rt.motor.as_ref().expect("ensure_motor sets motor");
    match cmd.op.as_str() {
        "enable" => {
            abi.motor_enable(motor)?;
            rt.enabled = true;
        }
        "disable" => {
            abi.motor_disable(motor)?;
            rt.enabled = false;
            rt.active_cmd = None;
        }
        "clear_error" | "clear-error" => {
            abi.motor_clear_error(motor)?;
        }
        "set_zero" | "set-zero" | "set_zero_position" | "set-zero-position" => {
            abi.motor_set_zero_position(motor)?;
        }
        "store_parameters" | "store-parameters" | "save_parameters" | "save-parameters" => {
            abi.motor_store_parameters(motor)?;
        }
        "mit" => {
            abi.motor_ensure_mode(motor, 1, 200)?;
            abi.motor_send_mit(
                motor,
                mapped_pos(rt, cmd.pos.unwrap_or(0.0))?,
                mapped_axis(rt, cmd.vel.unwrap_or(0.0)),
                cmd.kp.unwrap_or(rt.profile.kp.unwrap_or(0.05)),
                cmd.kd.unwrap_or(rt.profile.kd.unwrap_or(0.005)),
                mapped_axis(rt, cmd.tau.unwrap_or(0.0)),
            )?;
        }
        "pos_vel" | "pos-vel" => {
            abi.motor_ensure_mode(motor, 2, 200)?;
            abi.motor_send_pos_vel(
                motor,
                mapped_pos(rt, cmd.pos.unwrap_or(0.0))?,
                cmd.vlim.unwrap_or(rt.profile.max_velocity.unwrap_or(3.0)),
            )?;
        }
        "vel" => {
            abi.motor_ensure_mode(motor, 3, 200)?;
            abi.motor_send_vel(motor, mapped_axis(rt, cmd.vel.unwrap_or(0.0)))?;
        }
        "force_pos" | "force-pos" => {
            abi.motor_ensure_mode(motor, 4, 200)?;
            abi.motor_send_force_pos(
                motor,
                mapped_pos(rt, cmd.pos.unwrap_or(0.0))?,
                cmd.vlim.unwrap_or(rt.profile.max_velocity.unwrap_or(3.0)),
                cmd.ratio.unwrap_or(0.3),
            )?;
        }
        other => return Err(anyhow!("unsupported op: {other}")),
    }
    Ok(())
}

fn ensure_motor(
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    rt: &mut JointRuntime,
) -> Result<()> {
    if rt.motor.is_some() {
        return Ok(());
    }
    let vendor = normalize_vendor(rt.cfg.vendor.as_deref().unwrap_or("damiao"));
    let transport = effective_transport(rt);
    let controller_key = format!("{}:{}:{}", vendor, transport, rt.cfg.bus_interface);
    if !controllers.contains_key(&controller_key) {
        println!(
            "[motorbridge_ros2] opening motorbridge ABI controller vendor={} transport={} endpoint='{}'",
            vendor, transport, rt.cfg.bus_interface
        );
        let controller =
            abi.new_controller(&transport, &rt.cfg.bus_interface, rt.cfg.serial_baud)?;
        controllers.insert(controller_key.clone(), controller);
    }
    let controller = controllers
        .get(&controller_key)
        .expect("controller inserted above");
    let model = effective_model(rt, &vendor);
    println!(
        "[motorbridge_ros2] add motorbridge motor joint={} vendor={} transport={} bus={} motor_id=0x{:X} feedback_id=0x{:X} model={}",
        rt.cfg.joint_name,
        vendor,
        transport,
        rt.cfg.bus_interface,
        rt.cfg.motor_id,
        rt.cfg.feedback_id.unwrap_or(rt.cfg.motor_id),
        model
    );
    rt.motor = Some(abi.add_motor(
        controller,
        &vendor,
        rt.cfg.motor_id,
        rt.cfg.feedback_id.unwrap_or(rt.cfg.motor_id),
        &model,
    )?);
    publish_json_event(
        rt,
        "info",
        "motor_added",
        "motorbridge ABI motor handle ready",
    );
    Ok(())
}

pub fn mapped_axis(rt: &JointRuntime, v: f32) -> f32 {
    v * f32::from(rt.cfg.direction.unwrap_or(1))
}

pub fn effective_transport(rt: &JointRuntime) -> String {
    if let Some(transport) = &rt.cfg.transport {
        return normalize_transport(transport);
    }
    match normalize_vendor(rt.cfg.vendor.as_deref().unwrap_or("damiao")).as_str() {
        "hexfellow" => "socketcanfd".to_string(),
        _ => "socketcan".to_string(),
    }
}

fn effective_model(rt: &JointRuntime, vendor: &str) -> String {
    if let Some(model) = &rt.cfg.model {
        return model.clone();
    }
    match vendor {
        "robstride" => "rs-00".to_string(),
        "myactuator" => "X8".to_string(),
        "hightorque" => "hightorque".to_string(),
        "hexfellow" => "hexfellow".to_string(),
        _ => "4340P".to_string(),
    }
}

fn mapped_pos(rt: &JointRuntime, pos: f32) -> Result<f32> {
    if let Some([min_p, max_p]) = rt.cfg.pos_limit {
        if pos < min_p || pos > max_p {
            return Err(anyhow!("position out of range [{min_p}, {max_p}]"));
        }
    }
    Ok((pos - rt.cfg.pos_offset.unwrap_or(0.0)) * f32::from(rt.cfg.direction.unwrap_or(1)))
}

pub fn now_millis() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

pub fn publish_json_event(rt: &JointRuntime, level: &str, kind: &str, message: &str) {
    let data = serde_json::json!({
        "level": level,
        "kind": kind,
        "message": message,
        "joint_name": rt.cfg.joint_name.as_str(),
        "motor_id": rt.cfg.motor_id,
        "feedback_id": rt.cfg.feedback_id.unwrap_or(rt.cfg.motor_id),
        "ts_millis": now_millis()
    })
    .to_string();
    let _ = rt.json_event_writer.write(RosString { data }, None);
}

pub fn publish_command_event(
    rt: &JointRuntime,
    level: &str,
    kind: &str,
    cmd: &MotorCommand,
    ok: bool,
    error: Option<&str>,
) {
    let data = serde_json::json!({
        "level": level,
        "kind": kind,
        "joint_name": rt.cfg.joint_name.as_str(),
        "request_id": cmd.request_id.as_deref(),
        "op": cmd.op.as_str(),
        "ok": ok,
        "error": error,
        "enabled": rt.enabled,
        "command_count": rt.command_count,
        "error_count": rt.error_count,
        "ts_millis": now_millis()
    })
    .to_string();
    let _ = rt.json_event_writer.write(RosString { data }, None);
}

pub fn publish_bridge_status(
    writer: &no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    manifest: &Manifest,
    runtimes: &[JointRuntime],
    controller_count: usize,
) {
    let joints: Vec<_> = runtimes
        .iter()
        .map(|rt| {
            serde_json::json!({
                "joint_name": rt.cfg.joint_name.as_str(),
                "vendor": rt.cfg.vendor.as_deref().unwrap_or("damiao"),
                "transport": effective_transport(rt),
                "bus_interface": rt.cfg.bus_interface.as_str(),
                "motor_id": rt.cfg.motor_id,
                "feedback_id": rt.cfg.feedback_id.unwrap_or(rt.cfg.motor_id),
                "enabled": rt.enabled,
                "connected": rt.motor.is_some(),
                "active_command": rt.active_cmd.as_ref().map(|cmd| cmd.op.as_str()),
                "command_count": rt.command_count,
                "error_count": rt.error_count,
                "last_error": rt.last_error.as_deref(),
                "last_cmd_age_ms": rt.last_cmd_at.elapsed().as_millis() as u64,
            })
        })
        .collect();
    let data = serde_json::json!({
        "app": APP_NAME,
        "version": APP_VERSION,
        "target_name": manifest.target_name.as_str(),
        "manifest_version": manifest.version.as_str(),
        "controller_count": controller_count,
        "joints": joints,
        "ts_millis": now_millis()
    })
    .to_string();
    let _ = writer.write(RosString { data }, None);
}

pub fn engage_estop(abi: &MotorAbi, runtimes: &mut [JointRuntime], reason: &str) {
    eprintln!("[estop] engaged reason={reason}");
    for rt in runtimes {
        if let Some(motor) = &rt.motor {
            let _ = abi.motor_disable(motor);
        }
        rt.enabled = false;
        rt.active_cmd = None;
        publish_json_event(rt, "warn", "estop", reason);
    }
}

pub fn status_name(code: u8) -> Option<&'static str> {
    match code {
        0 => Some("ok"),
        _ => None,
    }
}
