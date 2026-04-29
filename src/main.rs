mod abi;
mod cli;
mod config;
mod manifest_tools;
mod ros2_adapter;
mod runtime;
mod types;

use anyhow::{Context, Result};
use cli::parse_cli_args;
use config::Manifest;
use manifest_tools::{print_topic_plan, validate_manifest};
use ros2_adapter::run_bridge;

fn main() -> Result<()> {
    let cli = parse_cli_args()?;
    let Some(manifest_path) = cli.manifest_path.clone() else {
        return Ok(());
    };
    let manifest = load_manifest(&manifest_path)?;

    if cli.check_config {
        println!(
            "config ok: manifest={manifest_path} target={} joints={}",
            manifest.target_name,
            manifest.joints.len()
        );
        return Ok(());
    }
    if cli.list_topics {
        print_topic_plan(&manifest);
        return Ok(());
    }

    run_bridge(&manifest_path, manifest)
}

fn load_manifest(path: &str) -> Result<Manifest> {
    let manifest_text =
        std::fs::read_to_string(path).with_context(|| format!("read manifest failed: {path}"))?;
    let manifest: Manifest =
        serde_yaml::from_str(&manifest_text).context("parse manifest yaml failed")?;
    validate_manifest(&manifest)?;
    Ok(manifest)
}
