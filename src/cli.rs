use anyhow::{anyhow, Result};

pub const APP_NAME: &str = "motorbridge_ros2";
pub const APP_VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Default)]
pub struct CliArgs {
    pub manifest_path: Option<String>,
    pub check_config: bool,
    pub list_topics: bool,
}

pub fn parse_cli_args() -> Result<CliArgs> {
    let args: Vec<String> = std::env::args().collect();
    let mut cli = CliArgs::default();

    let mut i = 1usize;
    while i < args.len() {
        let a = &args[i];
        match a.as_str() {
            "-h" | "--help" => {
                print_help();
                return Ok(cli);
            }
            "-V" | "--version" => {
                println!("{APP_NAME} {APP_VERSION}");
                return Ok(cli);
            }
            "--check-config" => {
                cli.check_config = true;
            }
            "--list-topics" => {
                cli.list_topics = true;
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
                if cli.manifest_path.is_some() {
                    return Err(anyhow!(
                        "manifest path already set; pass only one of positional path or -c/--config"
                    ));
                }
                cli.manifest_path = Some(next.clone());
                i += 1;
            }
            _ if a.starts_with('-') => {
                return Err(anyhow!(
                    "unknown option: {a}\nuse --help to see supported options"
                ));
            }
            _ => {
                if cli.manifest_path.is_some() {
                    return Err(anyhow!(
                        "multiple manifest paths provided; only one is allowed"
                    ));
                }
                cli.manifest_path = Some(a.clone());
            }
        }
        i += 1;
    }

    cli.manifest_path = Some(
        cli.manifest_path
            .unwrap_or_else(|| "motorbridge_manifest.yaml".to_string()),
    );
    Ok(cli)
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
           {APP_NAME} --check-config [-c manifest.yaml]\n\
           {APP_NAME} --list-topics [-c manifest.yaml]\n\
         \n\
         Arguments:\n\
           -c, --config     optional manifest path (default: motorbridge_manifest.yaml)\n\
           manifest.yaml    positional manifest path (backward-compatible)\n\
           --check-config   validate manifest and exit without loading DDS or motor ABI\n\
           --list-topics    print ROS2/DDS topic plan and exit\n\
         \n\
         Examples:\n\
           {APP_NAME}\n\
           {APP_NAME} -c motorbridge_manifest.yaml\n\
           {APP_NAME} motorbridge_manifest.yaml\n\
           {APP_NAME} --check-config -c motorbridge_manifest.yaml\n\
           {APP_NAME} --list-topics -c motorbridge_manifest.yaml"
    );
}
