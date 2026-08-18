mod api;
mod calibration_artifacts;

use anyhow::{bail, Context, Result};
use clap::Parser;
use std::net::SocketAddr;
use std::path::PathBuf;
use tower_http::services::{ServeDir, ServeFile};
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;
use velocitypnp_domain::MachineProfile;
use velocitypnp_driver::{MachineDriver, MachineHandle, MarlinDriver, SimulatorDriver};
use velocitypnp_store::Store;

#[derive(Debug, Parser)]
#[command(version, about = "VelocityPnP machine-control daemon")]
struct Args {
    /// Run without physical hardware.
    #[arg(long, default_value_t = true, action = clap::ArgAction::Set)]
    simulate: bool,

    /// Explicitly permit opening the attached controller.
    #[arg(long, default_value_t = false)]
    allow_hardware: bool,

    #[arg(
        long,
        default_value = "/dev/serial/by-id/usb-STMicroelectronics_MARLIN_OPULO_LUMEN_REV4_CDC_in_FS_Mode_3286336D3331-if00"
    )]
    serial_port: PathBuf,

    #[arg(long, default_value_t = 115_200)]
    baud: u32,

    #[arg(long, default_value = "velocitypnp.db")]
    database: PathBuf,

    #[arg(long, default_value = "127.0.0.1:4080")]
    listen: SocketAddr,

    /// Serve a production web build and use it as SPA fallback.
    #[arg(long)]
    web_dir: Option<PathBuf>,
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "velocitypnp=info,tower_http=info".into()),
        )
        .init();
    let args = Args::parse();
    if !args.simulate && !args.allow_hardware {
        bail!("physical hardware is disabled; pass both --simulate=false and --allow-hardware after simulator validation");
    }

    let profile: MachineProfile =
        serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json"))
            .context("decoding built-in machine profile")?;
    let driver: Box<dyn MachineDriver> = if args.simulate {
        Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm))
    } else {
        Box::new(MarlinDriver::new(
            args.serial_port.to_string_lossy(),
            args.baud,
            profile.home_pose,
            profile.homing.clone(),
            profile
                .capabilities
                .get("rev04_integrated_vacuum_sensors")
                .copied()
                .unwrap_or(false),
        ))
    };
    let machine = MachineHandle::spawn(driver, profile.clone());
    let shutdown_machine = machine.clone();
    let store = Store::open(&args.database)?;
    let state = api::AppState::new(machine, profile, store);
    state.spawn_event_journal();

    let mut app = api::router(state);
    if let Some(web_dir) = args.web_dir {
        let index = web_dir.join("index.html");
        app = app.fallback_service(ServeDir::new(web_dir).fallback(ServeFile::new(index)));
    }
    let listener = tokio::net::TcpListener::bind(args.listen).await?;
    info!(address = %args.listen, simulated = args.simulate, "VelocityPnP daemon listening");
    let serve_result = axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal())
        .await;
    if shutdown_machine.status().state != velocitypnp_domain::MachineState::Disconnected {
        if let Err(error) = shutdown_machine.disconnect().await {
            warn!(%error, "machine disconnect reported errors during shutdown");
        }
    }
    serve_result?;
    Ok(())
}

async fn shutdown_signal() {
    let ctrl_c = async {
        tokio::signal::ctrl_c()
            .await
            .expect("failed to install Ctrl+C handler");
    };

    #[cfg(unix)]
    let terminate = async {
        tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("failed to install SIGTERM handler")
            .recv()
            .await;
    };

    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();

    tokio::select! {
        () = ctrl_c => {}
        () = terminate => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physical_mode_requires_an_explicit_false_value() {
        let args = Args::try_parse_from(["velocitypnpd", "--simulate=false", "--allow-hardware"])
            .expect("documented physical-mode arguments must parse");
        assert!(!args.simulate);
        assert!(args.allow_hardware);
    }

    #[test]
    fn simulation_remains_the_default() {
        assert!(Args::try_parse_from(["velocitypnpd"]).unwrap().simulate);
    }
}
