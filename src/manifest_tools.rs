use std::collections::HashSet;

use anyhow::{anyhow, Result};

use crate::abi::{normalize_transport, normalize_vendor};
use crate::config::Manifest;

pub fn validate_manifest(manifest: &Manifest) -> Result<()> {
    if manifest.joints.is_empty() {
        return Err(anyhow!("manifest must define at least one joint"));
    }
    let profile_names: HashSet<&str> = manifest
        .control_profiles
        .iter()
        .map(|p| p.profile_name.as_str())
        .collect();
    if profile_names.is_empty() {
        return Err(anyhow!("manifest must define at least one control profile"));
    }

    let mut joint_names = HashSet::new();
    for joint in &manifest.joints {
        if joint.joint_name.trim().is_empty() {
            return Err(anyhow!("joint_name cannot be empty"));
        }
        if !joint_names.insert(joint.joint_name.as_str()) {
            return Err(anyhow!("duplicate joint_name: {}", joint.joint_name));
        }
        if !profile_names.contains(joint.default_profile.as_str()) {
            return Err(anyhow!(
                "joint {} references missing default_profile {}",
                joint.joint_name,
                joint.default_profile
            ));
        }
        let vendor = normalize_vendor(joint.vendor.as_deref().unwrap_or("damiao"));
        if !matches!(
            vendor.as_str(),
            "damiao" | "robstride" | "myactuator" | "hexfellow" | "hightorque"
        ) {
            return Err(anyhow!(
                "joint {} uses unsupported vendor {}",
                joint.joint_name,
                joint.vendor.as_deref().unwrap_or("")
            ));
        }
        if let Some(transport) = &joint.transport {
            let transport = normalize_transport(transport);
            if !matches!(
                transport.as_str(),
                "socketcan" | "can" | "auto" | "socketcanfd" | "canfd" | "dmserial" | "serial"
            ) {
                return Err(anyhow!(
                    "joint {} uses unsupported transport {}",
                    joint.joint_name,
                    joint.transport.as_deref().unwrap_or("")
                ));
            }
        }
        if let Some([min_p, max_p]) = joint.pos_limit {
            if min_p >= max_p {
                return Err(anyhow!(
                    "joint {} has invalid pos_limit [{}, {}]",
                    joint.joint_name,
                    min_p,
                    max_p
                ));
            }
        }
        if !matches!(joint.direction.unwrap_or(1), -1 | 1) {
            return Err(anyhow!(
                "joint {} direction must be 1 or -1",
                joint.joint_name
            ));
        }
    }
    Ok(())
}

pub fn print_topic_plan(manifest: &Manifest) {
    println!(
        "ROS2/DDS topic plan for target={} namespace={}",
        manifest.target_name, manifest.ros_bridge_options.namespace
    );
    println!(
        "  publish: {}",
        topic_path(
            &manifest.ros_bridge_options.namespace,
            &manifest.ros_bridge_options.status_topic
        )
    );
    if manifest.ros_bridge_options.enable_easter_counter {
        println!(
            "  publish: {}",
            topic_path(&manifest.ros_bridge_options.namespace, "easter_counter")
        );
    }
    println!(
        "  subscribe: {}",
        manifest.global_safety.emergency_stop_topic
    );
    println!(
        "  subscribe: {}",
        manifest.global_safety.emergency_stop_json_topic
    );
    for joint in &manifest.joints {
        println!("  joint {}", joint.joint_name);
        println!(
            "    subscribe: {}",
            topic_path(
                &manifest.ros_bridge_options.namespace,
                &format!("{}/cmd_json", joint.joint_name)
            )
        );
        println!(
            "    publish:   {}",
            topic_path(
                &manifest.ros_bridge_options.namespace,
                &format!("{}/state_json", joint.joint_name)
            )
        );
        println!(
            "    publish:   {}",
            topic_path(
                &manifest.ros_bridge_options.namespace,
                &format!("{}/event_json", joint.joint_name)
            )
        );
        println!(
            "    typed sub: {}",
            topic_path(
                &manifest.ros_bridge_options.namespace,
                &format!("{}/cmd", joint.joint_name)
            )
        );
        println!(
            "    typed pub: {}",
            topic_path(
                &manifest.ros_bridge_options.namespace,
                &format!("{}/state", joint.joint_name)
            )
        );
    }
}

fn topic_path(namespace: &str, relative: &str) -> String {
    let ns = namespace.trim_matches('/');
    let rel = relative.trim_matches('/');
    if ns.is_empty() {
        format!("/{rel}")
    } else {
        format!("/{ns}/{rel}")
    }
}
