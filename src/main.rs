#![allow(deprecated)]

mod abi;
mod config;
mod types;

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use abi::{normalize_transport, normalize_vendor, AbiController, AbiMotor, MotorAbi};
use anyhow::{anyhow, Context, Result};
use config::{ControlProfile, Manifest};
use mio_06::{Events, Poll, PollOpt, Ready, Token};
use rustdds::policy::{Durability, History, Liveliness, Reliability};
use rustdds::ros2::{NodeOptions, RosParticipant};
use rustdds::{no_key, CDRDeserializerAdapter, CDRSerializerAdapter, QosPolicyBuilder, TopicKind};
use types::{EStopMessage, MotorCommand, MotorState, RosString};

const TICK_MS: u64 = 20;
const TOKEN_BASE_JOINT: usize = 10;
const TOKEN_BASE_JSON: usize = 1000;
const TOKEN_ESTOP: Token = Token(1);
const APP_NAME: &str = "motorbridge_ros2";
const APP_VERSION: &str = env!("CARGO_PKG_VERSION");

struct JointRuntime {
    cfg: config::JointConfig,
    profile: ControlProfile,
    motor: Option<AbiMotor>,
    reader: no_key::DataReader<MotorCommand, CDRDeserializerAdapter<MotorCommand>>,
    json_reader: no_key::DataReader<RosString, CDRDeserializerAdapter<RosString>>,
    writer: no_key::DataWriter<MotorState, CDRSerializerAdapter<MotorState>>,
    json_state_writer: no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    json_event_writer: no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    enabled: bool,
    active_cmd: Option<MotorCommand>,
    last_cmd_at: Instant,
    last_state_log_at: Instant,
    last_feedback_warn_at: Instant,
    last_bus_error_log_at: Instant,
}

fn main() -> Result<()> {
    let manifest_path = match parse_cli_args()? {
        Some(path) => path,
        None => return Ok(()),
    };
    let manifest_text = std::fs::read_to_string(&manifest_path)
        .with_context(|| format!("read manifest failed: {manifest_path}"))?;
    let manifest: Manifest = serde_yaml::from_str(&manifest_text).context("parse manifest yaml failed")?;
    println!(
        "motorbridge_ros2 starting: manifest={} target={} version={}",
        manifest_path, manifest.target_name, manifest.version
    );

    let abi = MotorAbi::load_default().context("load motorbridge ABI failed")?;
    let mut controllers: HashMap<String, AbiController> = HashMap::new();
    println!("[boot] motorbridge ABI backend loaded; controllers connect lazily on first command.");

    let qos = if manifest.ros_bridge_options.qos_profile.eq_ignore_ascii_case("reliable") {
        QosPolicyBuilder::new()
            .durability(Durability::Volatile)
            .liveliness(Liveliness::Automatic {
                lease_duration: rustdds::Duration::INFINITE,
            })
            .reliability(Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(50),
            })
            .history(History::KeepLast { depth: 10 })
            .build()
    } else {
        QosPolicyBuilder::new()
            .durability(Durability::Volatile)
            .liveliness(Liveliness::Automatic {
                lease_duration: rustdds::Duration::INFINITE,
            })
            .reliability(Reliability::BestEffort)
            .history(History::KeepLast { depth: 10 })
            .build()
    };

    let mut ros_participant = RosParticipant::new().context("create ROS participant failed")?;
    println!("[boot] ROS participant ready.");
    let mut ros_node = ros_participant
        .new_ros_node("motorbridge_ros2", &format!("/{}", manifest.ros_bridge_options.namespace), NodeOptions::new(false))
        .context("create ROS node failed")?;

    let counter_topic = ros_node.create_ros_topic(
        "easter_counter",
        "std_msgs::msg::dds_::String_".to_string(),
        &qos,
        TopicKind::NoKey,
    )?;
    let counter_writer = ros_node
        .create_ros_no_key_publisher::<RosString, CDRSerializerAdapter<_>>(&counter_topic, None)
        .context("create easter counter publisher failed")?;
    let mut counter_value: u32 = 1;
    let mut counter_last_at = Instant::now();

    let mut runtimes = Vec::new();
    for joint in &manifest.joints {
        let profile = manifest.control_profiles.iter().find(|p| p.profile_name == joint.default_profile)
            .ok_or_else(|| anyhow!("missing profile {}", joint.default_profile))?.clone();

        // Use relative topic names; ROS node namespace already provides global prefix.
        let cmd_topic_name = format!("{}/cmd", joint.joint_name);
        let state_topic_name = format!("{}/state", joint.joint_name);
        let cmd_json_topic_name = format!("{}/cmd_json", joint.joint_name);
        let state_json_topic_name = format!("{}/state_json", joint.joint_name);
        let event_json_topic_name = format!("{}/event_json", joint.joint_name);
        println!(
            "[boot] joint={} vendor={} transport={} bus={} motor_id=0x{:X} feedback_id=0x{:X} model={} subscribe=/{}, /{} publish=/{}, /{}, /{}",
            joint.joint_name,
            joint.vendor.as_deref().unwrap_or("damiao"),
            joint.transport.as_deref().unwrap_or("auto"),
            joint.bus_interface,
            joint.motor_id,
            joint.feedback_id.unwrap_or(joint.motor_id),
            joint.model.as_deref().unwrap_or("4340P"),
            cmd_topic_name,
            cmd_json_topic_name,
            state_topic_name,
            state_json_topic_name,
            event_json_topic_name
        );
        let cmd_topic = ros_node.create_ros_topic(&cmd_topic_name, format!("motorbridge_msgs::msg::dds_::{}_Cmd_", joint.joint_name), &qos, TopicKind::NoKey)?;
        let state_topic = ros_node.create_ros_topic(&state_topic_name, format!("motorbridge_msgs::msg::dds_::{}_State_", joint.joint_name), &qos, TopicKind::NoKey)?;
        let cmd_json_topic = ros_node.create_ros_topic(&cmd_json_topic_name, "std_msgs::msg::dds_::String_".to_string(), &qos, TopicKind::NoKey)?;
        let state_json_topic = ros_node.create_ros_topic(&state_json_topic_name, "std_msgs::msg::dds_::String_".to_string(), &qos, TopicKind::NoKey)?;
        let event_json_topic = ros_node.create_ros_topic(&event_json_topic_name, "std_msgs::msg::dds_::String_".to_string(), &qos, TopicKind::NoKey)?;

        runtimes.push(JointRuntime {
            cfg: joint.clone(),
            profile,
            motor: None,
            reader: ros_node.create_ros_no_key_subscriber::<MotorCommand, CDRDeserializerAdapter<_>>(&cmd_topic, None)?,
            json_reader: ros_node.create_ros_no_key_subscriber::<RosString, CDRDeserializerAdapter<_>>(&cmd_json_topic, None)?,
            writer: ros_node.create_ros_no_key_publisher::<MotorState, CDRSerializerAdapter<_>>(&state_topic, None)?,
            json_state_writer: ros_node.create_ros_no_key_publisher::<RosString, CDRSerializerAdapter<_>>(&state_json_topic, None)?,
            json_event_writer: ros_node.create_ros_no_key_publisher::<RosString, CDRSerializerAdapter<_>>(&event_json_topic, None)?,
            enabled: false,
            active_cmd: None,
            last_cmd_at: Instant::now(),
            last_state_log_at: Instant::now(),
            last_feedback_warn_at: Instant::now(),
            last_bus_error_log_at: Instant::now(),
        });
    }

    let estop_topic = ros_node.create_ros_topic(&manifest.global_safety.emergency_stop_topic, "motorbridge_msgs::msg::dds_::EmergencyStop_".to_string(), &qos, TopicKind::NoKey)?;
    let mut estop_reader = ros_node.create_ros_no_key_subscriber::<EStopMessage, CDRDeserializerAdapter<_>>(&estop_topic, None)?;

    let poll = Poll::new()?;
    poll.register(&estop_reader, TOKEN_ESTOP, Ready::readable(), PollOpt::edge())?;
    for (i, rt) in runtimes.iter_mut().enumerate() {
        poll.register(&rt.reader, Token(TOKEN_BASE_JOINT + i), Ready::readable(), PollOpt::edge())?;
        poll.register(&rt.json_reader, Token(TOKEN_BASE_JSON + i), Ready::readable(), PollOpt::edge())?;
    }

    let stop_flag = Arc::new(std::sync::atomic::AtomicBool::new(false));
    {
        let stop_flag = Arc::clone(&stop_flag);
        ctrlc::set_handler(move || stop_flag.store(true, std::sync::atomic::Ordering::Release))?;
    }

    let hb_timeout = Duration::from_millis(manifest.global_safety.heartbeat_timeout_ms.max(1));
    while !stop_flag.load(std::sync::atomic::Ordering::Acquire) {
        if counter_last_at.elapsed() >= Duration::from_secs(1) {
            let _ = counter_writer.write(
                RosString {
                    data: counter_value.to_string(),
                },
                None,
            );
            counter_value = if counter_value == 60 {
                0
            } else if counter_value == 0 {
                1
            } else {
                counter_value + 1
            };
            counter_last_at = Instant::now();
        }

        let mut events = Events::with_capacity(128);
        poll.poll(&mut events, Some(Duration::from_millis(TICK_MS)))?;

        for event in events.iter() {
            let token = event.token();
            if token == TOKEN_ESTOP {
                while let Ok(Some(sample)) = estop_reader.take_next_sample() {
                    if sample.value().engaged {
                        for rt in &mut runtimes {
                            if let Some(motor) = &rt.motor {
                                let _ = abi.motor_disable(motor);
                            }
                            rt.enabled = false;
                            rt.active_cmd = None;
                        }
                    }
                }
                continue;
            }
            if token.0 >= TOKEN_BASE_JOINT && token.0 < TOKEN_BASE_JOINT + runtimes.len() {
                let rt = &mut runtimes[token.0 - TOKEN_BASE_JOINT];
                while let Ok(Some(sample)) = rt.reader.take_next_sample() {
                    let cmd: MotorCommand = sample.value().clone();
                    println!(
                        "[cmd] joint={} via=typed op={} payload={:?}",
                        rt.cfg.joint_name, cmd.op, cmd
                    );
                    if let Err(err) = apply_command(&abi, &mut controllers, rt, &cmd) {
                        eprintln!(
                            "[cmd] joint={} via=typed apply_error={}",
                            rt.cfg.joint_name, err
                        );
                        publish_json_event(rt, "error", &err.to_string());
                    } else {
                        println!(
                            "[cmd] joint={} via=typed apply_ok op={}",
                            rt.cfg.joint_name, cmd.op
                        );
                        rt.last_cmd_at = Instant::now();
                        rt.active_cmd = cmd.continuous.unwrap_or(false).then_some(cmd);
                    }
                }
                continue;
            }
            if token.0 >= TOKEN_BASE_JSON && token.0 < TOKEN_BASE_JSON + runtimes.len() {
                let rt = &mut runtimes[token.0 - TOKEN_BASE_JSON];
                drain_json_commands(&abi, &mut controllers, rt);
            }
        }

        for rt in &mut runtimes {
            drain_json_commands(&abi, &mut controllers, rt);

            if let Some(cmd) = rt.active_cmd.clone() {
                let _ = apply_command(&abi, &mut controllers, rt, &cmd);
            }

            if rt.last_cmd_at.elapsed() > hb_timeout && rt.enabled && manifest.global_safety.watchdog_strategy != "hold" {
                if let Some(motor) = &rt.motor {
                    let _ = abi.motor_disable(motor);
                }
                rt.enabled = false;
            }

            let Some(motor) = &rt.motor else {
                continue;
            };
            if let Err(err) = abi.motor_request_feedback(motor) {
                if rt.last_bus_error_log_at.elapsed() >= Duration::from_secs(1) {
                    eprintln!(
                        "[bus] joint={} request_feedback_error={}",
                        rt.cfg.joint_name, err
                    );
                    rt.last_bus_error_log_at = Instant::now();
                }
            }
            if let Some(st) = abi.motor_get_state(motor)? {
                let d = f32::from(rt.cfg.direction.unwrap_or(1));
                let pos = st.pos * d + rt.cfg.pos_offset.unwrap_or(0.0);
                if rt.last_state_log_at.elapsed() >= Duration::from_secs(1) {
                    println!(
                        "[state] joint={} motor_id=0x{:X} feedback_id=0x{:X} enabled={} pos={:+.3} vel={:+.3} torq={:+.3} status={}",
                        rt.cfg.joint_name,
                        rt.cfg.motor_id,
                        rt.cfg.feedback_id.unwrap_or(rt.cfg.motor_id),
                        rt.enabled,
                        pos,
                        st.vel * d,
                        st.torq * d,
                        st.status_code
                    );
                    rt.last_state_log_at = Instant::now();
                }
                let state = MotorState {
                    joint_name: rt.cfg.joint_name.clone(),
                    enabled: rt.enabled,
                    pos: Some(pos),
                    vel: Some(st.vel * d),
                    torque: Some(st.torq * d),
                    t_mos: Some(st.t_mos),
                    t_rotor: Some(st.t_rotor),
                    status_code: Some(st.status_code),
                    status_name: None,
                    ts_millis: now_millis(),
                };
                let _ = rt.writer.write(state.clone(), None);
                if let Ok(data) = serde_json::to_string(&state) {
                    let _ = rt.json_state_writer.write(RosString { data }, None);
                }
            } else if rt.last_feedback_warn_at.elapsed() >= Duration::from_secs(2) {
                eprintln!(
                    "[state] joint={} motor_id=0x{:X} feedback_id=0x{:X} no feedback yet; check PCAN channel, bitrate, wiring, termination, power, and motor feedback id",
                    rt.cfg.joint_name,
                    rt.cfg.motor_id,
                    rt.cfg.feedback_id.unwrap_or(rt.cfg.motor_id)
                );
                rt.last_feedback_warn_at = Instant::now();
            }
        }

        for ctrl in controllers.values() {
            if let Err(err) = abi.controller_poll_feedback_once(ctrl) {
                eprintln!("[bus] poll_feedback_error={err}");
            }
        }
    }

    for rt in &mut runtimes {
        if let Some(motor) = rt.motor.take() {
            abi.free_motor(motor);
        }
    }

    for ctrl in controllers.into_values() {
        let _ = abi.controller_shutdown(&ctrl);
        abi.free_controller(ctrl);
    }

    ros_node.clear_node();
    ros_participant.clear();
    Ok(())
}

fn drain_json_commands(
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    rt: &mut JointRuntime,
) {
    while let Ok(Some(sample)) = rt.json_reader.take_next_sample() {
        let raw = sample.value().data.clone();
        println!(
            "[cmd] joint={} via=json raw={}",
            rt.cfg.joint_name, raw
        );
        match serde_json::from_str::<MotorCommand>(&raw) {
            Ok(cmd) => {
                if let Err(err) = apply_command(abi, controllers, rt, &cmd) {
                    eprintln!(
                        "[cmd] joint={} via=json apply_error={}",
                        rt.cfg.joint_name, err
                    );
                    publish_json_event(rt, "error", &err.to_string());
                } else {
                    println!(
                        "[cmd] joint={} via=json apply_ok op={}",
                        rt.cfg.joint_name, cmd.op
                    );
                    rt.last_cmd_at = Instant::now();
                    rt.active_cmd = cmd.continuous.unwrap_or(false).then_some(cmd);
                }
            }
            Err(err) => {
                eprintln!(
                    "[cmd] joint={} via=json parse_error={} raw={}",
                    rt.cfg.joint_name, err, raw
                );
                publish_json_event(rt, "error", &format!("json parse error: {err}"));
            }
        }
    }
}

fn apply_command(
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    rt: &mut JointRuntime,
    cmd: &MotorCommand,
) -> Result<()> {
    ensure_motor(abi, controllers, rt)?;
    let motor = rt.motor.as_ref().expect("ensure_motor sets motor");
    match cmd.op.as_str() {
        "enable" => { abi.motor_enable(motor)?; rt.enabled = true; }
        "disable" => { abi.motor_disable(motor)?; rt.enabled = false; rt.active_cmd = None; }
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
            abi.motor_send_mit(motor, mapped_pos(rt, cmd.pos.unwrap_or(0.0))?, mapped_axis(rt, cmd.vel.unwrap_or(0.0)), cmd.kp.unwrap_or(rt.profile.kp.unwrap_or(0.05)), cmd.kd.unwrap_or(rt.profile.kd.unwrap_or(0.005)), mapped_axis(rt, cmd.tau.unwrap_or(0.0)))?;
        }
        "pos_vel" | "pos-vel" => {
            abi.motor_ensure_mode(motor, 2, 200)?;
            abi.motor_send_pos_vel(motor, mapped_pos(rt, cmd.pos.unwrap_or(0.0))?, cmd.vlim.unwrap_or(rt.profile.max_velocity.unwrap_or(3.0)))?;
        }
        "vel" => {
            abi.motor_ensure_mode(motor, 3, 200)?;
            abi.motor_send_vel(motor, mapped_axis(rt, cmd.vel.unwrap_or(0.0)))?;
        }
        "force_pos" | "force-pos" => {
            abi.motor_ensure_mode(motor, 4, 200)?;
            abi.motor_send_force_pos(motor, mapped_pos(rt, cmd.pos.unwrap_or(0.0))?, cmd.vlim.unwrap_or(rt.profile.max_velocity.unwrap_or(3.0)), cmd.ratio.unwrap_or(0.3))?;
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
        let controller = abi.new_controller(&transport, &rt.cfg.bus_interface, rt.cfg.serial_baud)?;
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
    Ok(())
}

fn mapped_axis(rt: &JointRuntime, v: f32) -> f32 { v * f32::from(rt.cfg.direction.unwrap_or(1)) }

fn effective_transport(rt: &JointRuntime) -> String {
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
        if pos < min_p || pos > max_p { return Err(anyhow!("position out of range [{min_p}, {max_p}]")); }
    }
    Ok((pos - rt.cfg.pos_offset.unwrap_or(0.0)) * f32::from(rt.cfg.direction.unwrap_or(1)))
}
fn now_millis() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis() as i64).unwrap_or(0)
}

fn publish_json_event(rt: &JointRuntime, level: &str, message: &str) {
    let data = serde_json::json!({
        "level": level,
        "message": message,
        "joint_name": rt.cfg.joint_name,
        "ts_millis": now_millis()
    })
    .to_string();
    let _ = rt.json_event_writer.write(RosString { data }, None);
}

fn parse_cli_args() -> Result<Option<String>> {
    let args: Vec<String> = std::env::args().collect();
    let mut manifest_path: Option<String> = None;

    let mut i = 1usize;
    while i < args.len() {
        let a = &args[i];
        match a.as_str() {
            "-h" | "--help" => {
                print_help();
                return Ok(None);
            }
            "-V" | "--version" => {
                println!("{APP_NAME} {APP_VERSION}");
                return Ok(None);
            }
            "-c" | "--config" => {
                let next = args.get(i + 1).ok_or_else(|| {
                    anyhow!("missing value for {a}; expected a manifest yaml path")
                })?;
                if next.starts_with('-') {
                    return Err(anyhow!(
                        "invalid value for {a}: {next}\nexpected a manifest yaml path"
                    ));
                }
                if manifest_path.is_some() {
                    return Err(anyhow!(
                        "manifest path already set; pass only one of positional path or -c/--config"
                    ));
                }
                manifest_path = Some(next.clone());
                i += 1;
            }
            _ if a.starts_with('-') => {
                return Err(anyhow!(
                    "unknown option: {a}\nuse --help to see supported options"
                ));
            }
            _ => {
                if manifest_path.is_some() {
                    return Err(anyhow!(
                        "multiple manifest paths provided; only one is allowed"
                    ));
                }
                manifest_path = Some(a.clone());
            }
        }
        i += 1;
    }

    Ok(Some(
        manifest_path.unwrap_or_else(|| "motorbridge_manifest.yaml".to_string()),
    ))
}

fn print_help() {
    println!(
        "{APP_NAME} {APP_VERSION}\n\
         \n\
         Usage:\n\
           {APP_NAME} [-c manifest.yaml]\n\
           {APP_NAME} [manifest.yaml]\n\
           {APP_NAME} -h | --help\n\
           {APP_NAME} -V | --version\n\
         \n\
         Arguments:\n\
           -c, --config     optional manifest path (default: motorbridge_manifest.yaml)\n\
           manifest.yaml    positional manifest path (backward-compatible)\n\
         \n\
         Examples:\n\
           {APP_NAME}\n\
           {APP_NAME} -c motorbridge_manifest.yaml\n\
           {APP_NAME} motorbridge_manifest.yaml"
    );
}
