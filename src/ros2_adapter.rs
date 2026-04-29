#![allow(deprecated)]

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Context, Result};
use mio_06::{Events, Poll, PollOpt, Ready, Token};
use rustdds::policy::{Durability, History, Liveliness, Reliability};
use rustdds::ros2::{NodeOptions, RosParticipant};
use rustdds::{CDRDeserializerAdapter, CDRSerializerAdapter, QosPolicyBuilder, TopicKind};

use crate::abi::{AbiController, MotorAbi};
use crate::config::{self, Manifest};
use crate::runtime::{
    apply_command, drain_json_commands, engage_estop, now_millis, publish_bridge_status,
    publish_command_event, publish_json_event, status_name, JointRuntime,
};
use crate::types::{EStopMessage, MotorCommand, MotorState, RosString};

const TICK_MS: u64 = 20;
const TOKEN_BASE_JOINT: usize = 10;
const TOKEN_BASE_JSON: usize = 1000;
const TOKEN_ESTOP: Token = Token(1);
const TOKEN_ESTOP_JSON: Token = Token(2);

pub fn run_bridge(manifest_path: &str, manifest: Manifest) -> Result<()> {
    println!(
        "motorbridge_ros2 starting: manifest={} target={} version={}",
        manifest_path, manifest.target_name, manifest.version
    );

    let abi = MotorAbi::load_default().context("load motorbridge ABI failed")?;
    let mut controllers: HashMap<String, AbiController> = HashMap::new();
    println!("[boot] motorbridge ABI backend loaded; controllers connect lazily on first command.");

    let qos = build_qos(&manifest);
    let mut ros_participant = RosParticipant::new().context("create ROS participant failed")?;
    println!("[boot] ROS participant ready.");
    let mut ros_node = ros_participant
        .new_ros_node(
            "motorbridge_ros2",
            &format!("/{}", manifest.ros_bridge_options.namespace),
            NodeOptions::new(false),
        )
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

    let status_topic = ros_node.create_ros_topic(
        &manifest.ros_bridge_options.status_topic,
        "std_msgs::msg::dds_::String_".to_string(),
        &qos,
        TopicKind::NoKey,
    )?;
    let status_writer = ros_node
        .create_ros_no_key_publisher::<RosString, CDRSerializerAdapter<_>>(&status_topic, None)
        .context("create bridge status publisher failed")?;
    let mut status_last_at = Instant::now();

    let mut runtimes = create_joint_runtimes(&mut ros_node, &manifest, &qos)?;
    let (mut estop_reader, mut estop_json_reader) =
        create_estop_readers(&mut ros_node, &manifest, &qos)?;

    let poll = Poll::new()?;
    register_readers(&poll, &estop_reader, &estop_json_reader, &mut runtimes)?;

    let stop_flag = Arc::new(std::sync::atomic::AtomicBool::new(false));
    {
        let stop_flag = Arc::clone(&stop_flag);
        ctrlc::set_handler(move || stop_flag.store(true, std::sync::atomic::Ordering::Release))?;
    }

    let hb_timeout = Duration::from_millis(manifest.global_safety.heartbeat_timeout_ms.max(1));
    let state_period =
        Duration::from_millis(manifest.ros_bridge_options.state_publish_period_ms.max(1));
    let feedback_warn_period =
        Duration::from_millis(manifest.ros_bridge_options.feedback_warn_ms.max(100));

    while !stop_flag.load(std::sync::atomic::Ordering::Acquire) {
        publish_periodic_diagnostics(
            &manifest,
            &counter_writer,
            &mut counter_value,
            &mut counter_last_at,
            &status_writer,
            &mut status_last_at,
            &runtimes,
            controllers.len(),
        );

        let mut events = Events::with_capacity(128);
        poll.poll(&mut events, Some(Duration::from_millis(TICK_MS)))?;
        handle_reader_events(
            &events,
            &abi,
            &mut controllers,
            &mut runtimes,
            &mut estop_reader,
            &mut estop_json_reader,
        );

        tick_runtimes(
            &abi,
            &mut controllers,
            &mut runtimes,
            &manifest,
            hb_timeout,
            state_period,
            feedback_warn_period,
        )?;

        for ctrl in controllers.values() {
            if let Err(err) = abi.controller_poll_feedback_once(ctrl) {
                eprintln!("[bus] poll_feedback_error={err}");
            }
        }
    }

    shutdown_bridge(&abi, &mut runtimes, controllers);
    ros_node.clear_node();
    ros_participant.clear();
    Ok(())
}

fn build_qos(manifest: &Manifest) -> rustdds::QosPolicies {
    let builder = QosPolicyBuilder::new()
        .durability(Durability::Volatile)
        .liveliness(Liveliness::Automatic {
            lease_duration: rustdds::Duration::INFINITE,
        })
        .history(History::KeepLast { depth: 10 });

    if manifest
        .ros_bridge_options
        .qos_profile
        .eq_ignore_ascii_case("reliable")
    {
        builder
            .reliability(Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(50),
            })
            .build()
    } else {
        builder.reliability(Reliability::BestEffort).build()
    }
}

fn create_joint_runtimes(
    ros_node: &mut rustdds::ros2::RosNode,
    manifest: &Manifest,
    qos: &rustdds::QosPolicies,
) -> Result<Vec<JointRuntime>> {
    let mut runtimes = Vec::new();
    for joint in &manifest.joints {
        let profile = manifest
            .control_profiles
            .iter()
            .find(|p| p.profile_name == joint.default_profile)
            .ok_or_else(|| anyhow!("missing profile {}", joint.default_profile))?
            .clone();

        let cmd_topic_name = format!("{}/cmd", joint.joint_name);
        let state_topic_name = format!("{}/state", joint.joint_name);
        let cmd_json_topic_name = format!("{}/cmd_json", joint.joint_name);
        let state_json_topic_name = format!("{}/state_json", joint.joint_name);
        let event_json_topic_name = format!("{}/event_json", joint.joint_name);
        log_joint_boot(
            joint,
            &cmd_topic_name,
            &cmd_json_topic_name,
            &state_topic_name,
            &state_json_topic_name,
            &event_json_topic_name,
        );

        let cmd_topic = ros_node.create_ros_topic(
            &cmd_topic_name,
            format!("motorbridge_msgs::msg::dds_::{}_Cmd_", joint.joint_name),
            qos,
            TopicKind::NoKey,
        )?;
        let state_topic = ros_node.create_ros_topic(
            &state_topic_name,
            format!("motorbridge_msgs::msg::dds_::{}_State_", joint.joint_name),
            qos,
            TopicKind::NoKey,
        )?;
        let cmd_json_topic = ros_node.create_ros_topic(
            &cmd_json_topic_name,
            "std_msgs::msg::dds_::String_".to_string(),
            qos,
            TopicKind::NoKey,
        )?;
        let state_json_topic = ros_node.create_ros_topic(
            &state_json_topic_name,
            "std_msgs::msg::dds_::String_".to_string(),
            qos,
            TopicKind::NoKey,
        )?;
        let event_json_topic = ros_node.create_ros_topic(
            &event_json_topic_name,
            "std_msgs::msg::dds_::String_".to_string(),
            qos,
            TopicKind::NoKey,
        )?;

        runtimes.push(JointRuntime::new(
            joint.clone(),
            profile,
            ros_node.create_ros_no_key_subscriber::<MotorCommand, CDRDeserializerAdapter<_>>(
                &cmd_topic, None,
            )?,
            ros_node.create_ros_no_key_subscriber::<RosString, CDRDeserializerAdapter<_>>(
                &cmd_json_topic,
                None,
            )?,
            ros_node.create_ros_no_key_publisher::<MotorState, CDRSerializerAdapter<_>>(
                &state_topic,
                None,
            )?,
            ros_node.create_ros_no_key_publisher::<RosString, CDRSerializerAdapter<_>>(
                &state_json_topic,
                None,
            )?,
            ros_node.create_ros_no_key_publisher::<RosString, CDRSerializerAdapter<_>>(
                &event_json_topic,
                None,
            )?,
        ));
    }
    Ok(runtimes)
}

fn create_estop_readers(
    ros_node: &mut rustdds::ros2::RosNode,
    manifest: &Manifest,
    qos: &rustdds::QosPolicies,
) -> Result<(
    rustdds::no_key::DataReader<EStopMessage, CDRDeserializerAdapter<EStopMessage>>,
    rustdds::no_key::DataReader<RosString, CDRDeserializerAdapter<RosString>>,
)> {
    let estop_topic = ros_node.create_ros_topic(
        &manifest.global_safety.emergency_stop_topic,
        "motorbridge_msgs::msg::dds_::EmergencyStop_".to_string(),
        qos,
        TopicKind::NoKey,
    )?;
    let estop_reader = ros_node
        .create_ros_no_key_subscriber::<EStopMessage, CDRDeserializerAdapter<_>>(
            &estop_topic,
            None,
        )?;
    let estop_json_topic = ros_node.create_ros_topic(
        &manifest.global_safety.emergency_stop_json_topic,
        "std_msgs::msg::dds_::String_".to_string(),
        qos,
        TopicKind::NoKey,
    )?;
    let estop_json_reader = ros_node
        .create_ros_no_key_subscriber::<RosString, CDRDeserializerAdapter<_>>(
            &estop_json_topic,
            None,
        )?;
    Ok((estop_reader, estop_json_reader))
}

fn register_readers(
    poll: &Poll,
    estop_reader: &rustdds::no_key::DataReader<EStopMessage, CDRDeserializerAdapter<EStopMessage>>,
    estop_json_reader: &rustdds::no_key::DataReader<RosString, CDRDeserializerAdapter<RosString>>,
    runtimes: &mut [JointRuntime],
) -> Result<()> {
    poll.register(
        estop_reader,
        TOKEN_ESTOP,
        Ready::readable(),
        PollOpt::edge(),
    )?;
    poll.register(
        estop_json_reader,
        TOKEN_ESTOP_JSON,
        Ready::readable(),
        PollOpt::edge(),
    )?;
    for (i, rt) in runtimes.iter_mut().enumerate() {
        poll.register(
            &rt.reader,
            Token(TOKEN_BASE_JOINT + i),
            Ready::readable(),
            PollOpt::edge(),
        )?;
        poll.register(
            &rt.json_reader,
            Token(TOKEN_BASE_JSON + i),
            Ready::readable(),
            PollOpt::edge(),
        )?;
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn publish_periodic_diagnostics(
    manifest: &Manifest,
    counter_writer: &rustdds::no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    counter_value: &mut u32,
    counter_last_at: &mut Instant,
    status_writer: &rustdds::no_key::DataWriter<RosString, CDRSerializerAdapter<RosString>>,
    status_last_at: &mut Instant,
    runtimes: &[JointRuntime],
    controller_count: usize,
) {
    if manifest.ros_bridge_options.enable_easter_counter
        && counter_last_at.elapsed() >= Duration::from_secs(1)
    {
        let _ = counter_writer.write(
            RosString {
                data: counter_value.to_string(),
            },
            None,
        );
        *counter_value = if *counter_value == 60 {
            0
        } else if *counter_value == 0 {
            1
        } else {
            *counter_value + 1
        };
        *counter_last_at = Instant::now();
    }

    if status_last_at.elapsed() >= Duration::from_secs(1) {
        publish_bridge_status(status_writer, manifest, runtimes, controller_count);
        *status_last_at = Instant::now();
    }
}

fn handle_reader_events(
    events: &Events,
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    runtimes: &mut [JointRuntime],
    estop_reader: &mut rustdds::no_key::DataReader<
        EStopMessage,
        CDRDeserializerAdapter<EStopMessage>,
    >,
    estop_json_reader: &mut rustdds::no_key::DataReader<
        RosString,
        CDRDeserializerAdapter<RosString>,
    >,
) {
    for event in events.iter() {
        let token = event.token();
        if token == TOKEN_ESTOP {
            drain_typed_estop(abi, runtimes, estop_reader);
            continue;
        }
        if token == TOKEN_ESTOP_JSON {
            drain_json_estop(abi, runtimes, estop_json_reader);
            continue;
        }
        if token.0 >= TOKEN_BASE_JOINT && token.0 < TOKEN_BASE_JOINT + runtimes.len() {
            drain_typed_commands(abi, controllers, &mut runtimes[token.0 - TOKEN_BASE_JOINT]);
            continue;
        }
        if token.0 >= TOKEN_BASE_JSON && token.0 < TOKEN_BASE_JSON + runtimes.len() {
            drain_json_commands(abi, controllers, &mut runtimes[token.0 - TOKEN_BASE_JSON]);
        }
    }
}

fn drain_typed_estop(
    abi: &MotorAbi,
    runtimes: &mut [JointRuntime],
    estop_reader: &mut rustdds::no_key::DataReader<
        EStopMessage,
        CDRDeserializerAdapter<EStopMessage>,
    >,
) {
    while let Ok(Some(sample)) = estop_reader.take_next_sample() {
        if sample.value().engaged {
            engage_estop(abi, runtimes, "typed estop message");
        }
    }
}

fn drain_json_estop(
    abi: &MotorAbi,
    runtimes: &mut [JointRuntime],
    estop_json_reader: &mut rustdds::no_key::DataReader<
        RosString,
        CDRDeserializerAdapter<RosString>,
    >,
) {
    while let Ok(Some(sample)) = estop_json_reader.take_next_sample() {
        let raw = sample.value().data.clone();
        println!("[estop] via=json raw={raw}");
        match serde_json::from_str::<EStopMessage>(&raw) {
            Ok(msg) if msg.engaged => engage_estop(abi, runtimes, "json estop message"),
            Ok(_) => println!("[estop] via=json ignored because engaged=false"),
            Err(err) => eprintln!("[estop] via=json parse_error={err} raw={raw}"),
        }
    }
}

fn drain_typed_commands(
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    rt: &mut JointRuntime,
) {
    while let Ok(Some(sample)) = rt.reader.take_next_sample() {
        let cmd: MotorCommand = sample.value().clone();
        println!(
            "[cmd] joint={} via=typed op={} payload={:?}",
            rt.cfg.joint_name, cmd.op, cmd
        );
        if let Err(err) = apply_command(abi, controllers, rt, &cmd) {
            eprintln!(
                "[cmd] joint={} via=typed apply_error={}",
                rt.cfg.joint_name, err
            );
            rt.error_count += 1;
            rt.last_error = Some(err.to_string());
            publish_json_event(rt, "error", "command_failed", &err.to_string());
        } else {
            println!(
                "[cmd] joint={} via=typed apply_ok op={}",
                rt.cfg.joint_name, cmd.op
            );
            rt.command_count += 1;
            rt.last_cmd_at = Instant::now();
            publish_command_event(rt, "info", "command_applied", &cmd, true, None);
            rt.active_cmd = cmd.continuous.unwrap_or(false).then_some(cmd);
        }
    }
}

fn tick_runtimes(
    abi: &MotorAbi,
    controllers: &mut HashMap<String, AbiController>,
    runtimes: &mut [JointRuntime],
    manifest: &Manifest,
    hb_timeout: Duration,
    state_period: Duration,
    feedback_warn_period: Duration,
) -> Result<()> {
    for rt in runtimes {
        drain_json_commands(abi, controllers, rt);

        if let Some(cmd) = rt.active_cmd.clone() {
            let _ = apply_command(abi, controllers, rt, &cmd);
        }

        enforce_watchdog(abi, rt, manifest, hb_timeout);
        publish_motor_state(abi, rt, state_period, feedback_warn_period)?;
    }
    Ok(())
}

fn enforce_watchdog(
    abi: &MotorAbi,
    rt: &mut JointRuntime,
    manifest: &Manifest,
    hb_timeout: Duration,
) {
    if rt.last_cmd_at.elapsed() > hb_timeout
        && rt.enabled
        && manifest.global_safety.watchdog_strategy != "hold"
    {
        if let Some(motor) = &rt.motor {
            let _ = abi.motor_disable(motor);
        }
        rt.enabled = false;
        rt.active_cmd = None;
        publish_json_event(rt, "warn", "watchdog", "heartbeat timeout; motor disabled");
    }
}

fn publish_motor_state(
    abi: &MotorAbi,
    rt: &mut JointRuntime,
    state_period: Duration,
    feedback_warn_period: Duration,
) -> Result<()> {
    let Some(motor) = &rt.motor else {
        return Ok(());
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
        if rt.last_state_publish_at.elapsed() >= state_period {
            let state = MotorState {
                joint_name: rt.cfg.joint_name.clone(),
                enabled: rt.enabled,
                pos: Some(pos),
                vel: Some(st.vel * d),
                torque: Some(st.torq * d),
                t_mos: Some(st.t_mos),
                t_rotor: Some(st.t_rotor),
                status_code: Some(st.status_code),
                status_name: status_name(st.status_code).map(str::to_string),
                ts_millis: now_millis(),
            };
            let _ = rt.writer.write(state.clone(), None);
            if let Ok(data) = serde_json::to_string(&state) {
                let _ = rt.json_state_writer.write(RosString { data }, None);
            }
            rt.last_state_publish_at = Instant::now();
        }
    } else if rt.last_feedback_warn_at.elapsed() >= feedback_warn_period {
        eprintln!(
            "[state] joint={} motor_id=0x{:X} feedback_id=0x{:X} no feedback yet; check PCAN channel, bitrate, wiring, termination, power, and motor feedback id",
            rt.cfg.joint_name,
            rt.cfg.motor_id,
            rt.cfg.feedback_id.unwrap_or(rt.cfg.motor_id)
        );
        publish_json_event(
            rt,
            "warn",
            "no_feedback",
            "no feedback yet; check channel, bitrate, wiring, termination, power, and feedback id",
        );
        rt.last_feedback_warn_at = Instant::now();
    }
    Ok(())
}

fn shutdown_bridge(
    abi: &MotorAbi,
    runtimes: &mut [JointRuntime],
    controllers: HashMap<String, AbiController>,
) {
    for rt in runtimes {
        if let Some(motor) = rt.motor.take() {
            abi.free_motor(motor);
        }
    }

    for ctrl in controllers.into_values() {
        let _ = abi.controller_shutdown(&ctrl);
        abi.free_controller(ctrl);
    }
}

fn log_joint_boot(
    joint: &config::JointConfig,
    cmd_topic_name: &str,
    cmd_json_topic_name: &str,
    state_topic_name: &str,
    state_json_topic_name: &str,
    event_json_topic_name: &str,
) {
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
}
