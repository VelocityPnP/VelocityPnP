mod api;
mod calibration_artifacts;

use anyhow::{bail, Context, Result};
use clap::Parser;
use std::path::{Path, PathBuf};
use std::{backtrace::Backtrace, fs, net::SocketAddr, time::Duration};
use tower_http::services::{ServeDir, ServeFile};
use tracing::{error, info, warn};
use tracing_appender::non_blocking::WorkerGuard;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt, EnvFilter};
use velocitypnp_domain::MachineProfile;
use velocitypnp_driver::{MachineDriver, MachineHandle, MarlinDriver, SimulatorDriver};
use velocitypnp_model::{load_active_model_selection, ModelSelectionMode, ModelSelectionStatus};
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

    /// Directory for daily daemon log files.
    #[arg(long, env = "VELOCITYPNP_LOG_DIR", default_value = "logs")]
    log_dir: PathBuf,

    /// Maximum number of daily daemon log files to retain.
    #[arg(long, env = "VELOCITYPNP_LOG_RETENTION_FILES", default_value_t = 14)]
    log_retention_files: usize,

    /// Write logs only to the console.
    #[arg(long, default_value_t = false)]
    disable_file_logging: bool,

    #[arg(long, default_value = "0.0.0.0:4080")]
    listen: SocketAddr,

    /// Directory with model-manifest.json and its source-component ONNX file.
    #[arg(long, env = "VELOCITYPNP_SOURCE_COMPONENT_MODEL_DIR")]
    source_component_model_dir: Option<PathBuf>,

    /// Directory with model-manifest.json and its held-part ONNX file.
    #[arg(long, env = "VELOCITYPNP_HELD_PART_MODEL_DIR")]
    held_part_model_dir: Option<PathBuf>,

    /// Directory with the placement-verification model package.
    #[arg(long, env = "VELOCITYPNP_PLACEMENT_VERIFICATION_MODEL_DIR")]
    placement_verification_model_dir: Option<PathBuf>,

    /// Content-addressed model cache with an optional active runtime selection.
    #[arg(long, env = "VELOCITYPNP_MODEL_CACHE_DIR")]
    model_cache_dir: Option<PathBuf>,

    /// Do not load the active model-cache selection.
    #[arg(long, default_value_t = false)]
    disable_managed_models: bool,

    /// Serve a production web build and use it as SPA fallback.
    #[arg(long, env = "VELOCITYPNP_WEB_DIR")]
    web_dir: Option<PathBuf>,

    /// Serve only the API, without the production web UI.
    #[arg(long, default_value_t = false)]
    disable_web_ui: bool,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let log_guard = match init_logging(&args) {
        Ok(guard) => guard,
        Err(error) => {
            eprintln!("VelocityPnP logging initialization failed: {error:#}");
            std::process::exit(1);
        }
    };
    install_panic_logging();
    if args.disable_file_logging {
        info!("persistent file logging is disabled");
    } else {
        info!(
            directory = %args.log_dir.display(),
            rotation = "daily",
            retained_files = args.log_retention_files,
            "persistent logging enabled"
        );
    }

    let result = run(args).await;
    let exit_code = match result {
        Ok(()) => {
            info!("VelocityPnP daemon stopped");
            0
        }
        Err(error) => {
            error!(error = %format_args!("{error:#}"), "VelocityPnP daemon exited with an error");
            1
        }
    };
    drop(log_guard);
    // Tokio can wait for long-lived blocking driver workers while it drops the
    // runtime. The controller is already disconnected or startup failed, so
    // exit now and let the OS close any remaining process resources.
    std::process::exit(exit_code)
}

async fn run(args: Args) -> Result<()> {
    if !args.simulate && !args.allow_hardware {
        bail!("physical hardware is disabled; pass both --simulate=false and --allow-hardware after simulator validation");
    }

    let model_packages = resolve_model_packages(&args)?;
    match model_packages.status.mode {
        ModelSelectionMode::Managed => info!(
            lock_sha256 = model_packages
                .status
                .lock_sha256
                .as_deref()
                .unwrap_or_default(),
            "verified active model selection"
        ),
        ModelSelectionMode::Explicit => {
            info!("using explicit model package directories");
        }
        ModelSelectionMode::Disabled => {
            info!("no active model selection");
        }
    }

    let profile: MachineProfile =
        serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.2.json"))
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
    let state = api::AppState::new(machine, profile, store)
        .with_model_selection_status(model_packages.status)
        .with_source_component_model_package(model_packages.source_component.as_deref())
        .with_held_part_model_package(model_packages.held_part.as_deref())
        .with_placement_verification_model_package(
            model_packages.placement_verification.as_deref(),
        );
    state.require_managed_models_ready()?;
    state.spawn_event_journal();

    let mut app = api::router(state);
    if let Some(web_dir) = resolve_web_dir(&args)? {
        let index = web_dir.join("index.html");
        info!(path = %web_dir.display(), "serving production web UI");
        app = app.fallback_service(ServeDir::new(web_dir).fallback(ServeFile::new(index)));
    }
    let listener = tokio::net::TcpListener::bind(args.listen).await?;
    info!(address = %args.listen, simulated = args.simulate, "VelocityPnP daemon listening");
    let serve_result = axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal())
        .await;
    if shutdown_machine.status().state != velocitypnp_domain::MachineState::Disconnected {
        match tokio::time::timeout(Duration::from_secs(5), shutdown_machine.disconnect()).await {
            Ok(Ok(())) => {}
            Ok(Err(error)) => {
                warn!(%error, "machine disconnect reported errors during shutdown");
            }
            Err(_) => {
                warn!("machine disconnect timed out during shutdown");
            }
        }
    }
    serve_result?;
    Ok(())
}

fn init_logging(args: &Args) -> Result<Option<WorkerGuard>> {
    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| "velocitypnp=info,tower_http=info".into());
    let console_layer = tracing_subscriber::fmt::layer();
    if args.disable_file_logging {
        tracing_subscriber::registry()
            .with(filter)
            .with(console_layer)
            .try_init()
            .context("installing console logger")?;
        return Ok(None);
    }
    if args.log_retention_files == 0 {
        bail!("log retention must keep at least one file");
    }
    fs::create_dir_all(&args.log_dir)
        .with_context(|| format!("creating daemon log directory {}", args.log_dir.display()))?;
    let log_dir = fs::canonicalize(&args.log_dir)
        .with_context(|| format!("resolving daemon log directory {}", args.log_dir.display()))?;
    let file_appender = tracing_appender::rolling::Builder::new()
        .rotation(tracing_appender::rolling::Rotation::DAILY)
        .filename_prefix("velocitypnpd")
        .filename_suffix("log")
        .latest_symlink("velocitypnpd.current.log")
        .max_log_files(args.log_retention_files)
        .build(&log_dir)
        .with_context(|| format!("opening rotating daemon log in {}", args.log_dir.display()))?;
    let (file_writer, guard) = tracing_appender::non_blocking(file_appender);
    let file_layer = tracing_subscriber::fmt::layer()
        .with_ansi(false)
        .with_writer(file_writer);
    tracing_subscriber::registry()
        .with(filter)
        .with(console_layer)
        .with(file_layer)
        .try_init()
        .context("installing console and file logger")?;
    Ok(Some(guard))
}

fn install_panic_logging() {
    let default_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |panic_info| {
        let backtrace = Backtrace::force_capture();
        error!(
            panic = %panic_info,
            backtrace = %backtrace,
            "VelocityPnP daemon panicked"
        );
        default_hook(panic_info);
    }));
}

struct StartupModelPackages {
    status: ModelSelectionStatus,
    source_component: Option<PathBuf>,
    held_part: Option<PathBuf>,
    placement_verification: Option<PathBuf>,
}

fn resolve_model_packages(args: &Args) -> Result<StartupModelPackages> {
    let uses_explicit_packages = args.source_component_model_dir.is_some()
        || args.held_part_model_dir.is_some()
        || args.placement_verification_model_dir.is_some();
    if uses_explicit_packages {
        return Ok(StartupModelPackages {
            status: ModelSelectionStatus::explicit(),
            source_component: args.source_component_model_dir.clone(),
            held_part: args.held_part_model_dir.clone(),
            placement_verification: args.placement_verification_model_dir.clone(),
        });
    }
    if args.disable_managed_models {
        return Ok(disabled_model_packages());
    }
    let Some(cache_root) = args
        .model_cache_dir
        .clone()
        .or_else(default_model_cache_root)
    else {
        return Ok(disabled_model_packages());
    };
    let Some(status) = load_active_model_selection(&cache_root).with_context(|| {
        format!(
            "cannot load active model selection from {}",
            cache_root.display()
        )
    })?
    else {
        return Ok(disabled_model_packages());
    };
    Ok(StartupModelPackages {
        source_component: selected_package_dir(&status, "source_component")?,
        held_part: selected_package_dir(&status, "held_part")?,
        placement_verification: selected_package_dir(&status, "placement_verification")?,
        status,
    })
}

fn selected_package_dir(status: &ModelSelectionStatus, role: &str) -> Result<Option<PathBuf>> {
    Ok(Some(
        status
            .package_dir(role)
            .with_context(|| format!("active model selection has no {role} package"))?
            .to_owned(),
    ))
}

fn disabled_model_packages() -> StartupModelPackages {
    StartupModelPackages {
        status: ModelSelectionStatus::disabled(),
        source_component: None,
        held_part: None,
        placement_verification: None,
    }
}

fn default_model_cache_root() -> Option<PathBuf> {
    default_model_cache_root_from(
        std::env::var_os("XDG_CACHE_HOME").as_deref().map(Path::new),
        std::env::var_os("HOME").as_deref().map(Path::new),
    )
}

fn default_model_cache_root_from(xdg_cache: Option<&Path>, home: Option<&Path>) -> Option<PathBuf> {
    xdg_cache
        .map(Path::to_owned)
        .or_else(|| home.map(|path| path.join(".cache")))
        .map(|path| path.join("velocitypnp/models"))
}

fn resolve_web_dir(args: &Args) -> Result<Option<PathBuf>> {
    if args.disable_web_ui {
        return Ok(None);
    }
    if let Some(web_dir) = &args.web_dir {
        if !web_dir.join("index.html").is_file() {
            bail!("web UI directory has no index.html: {}", web_dir.display());
        }
        return Ok(Some(web_dir.clone()));
    }
    let candidates = [
        PathBuf::from("web/dist"),
        PathBuf::from("/usr/share/velocitypnp/web"),
    ];
    let web_dir = candidates
        .into_iter()
        .find(|candidate| candidate.join("index.html").is_file())
        .context(
            "web UI is enabled but no production build was found; run `npm run build` in web, pass --web-dir, or pass --disable-web-ui",
        )?;
    Ok(Some(web_dir))
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

    let signal = tokio::select! {
        () = ctrl_c => "SIGINT",
        () = terminate => "SIGTERM",
    };
    info!(signal, "shutdown signal received");
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

    #[test]
    fn all_ipv4_interfaces_are_the_default() {
        let args = Args::try_parse_from(["velocitypnpd"]).unwrap();
        assert_eq!(args.listen, SocketAddr::from(([0, 0, 0, 0], 4080)));
    }

    #[test]
    fn web_ui_is_enabled_by_default() {
        let args = Args::try_parse_from(["velocitypnpd"]).unwrap();
        assert!(!args.disable_web_ui);
        assert!(args.web_dir.is_none());

        let disabled = Args::try_parse_from(["velocitypnpd", "--disable-web-ui"]).unwrap();
        assert!(disabled.disable_web_ui);
        assert!(resolve_web_dir(&disabled).unwrap().is_none());

        let missing = Args::try_parse_from([
            "velocitypnpd",
            "--web-dir",
            "/velocitypnp-test/missing-web-build",
        ])
        .unwrap();
        assert!(resolve_web_dir(&missing)
            .unwrap_err()
            .to_string()
            .contains("has no index.html"));
    }

    #[test]
    fn rotating_file_logging_is_enabled_by_default() {
        let args = Args::try_parse_from(["velocitypnpd"]).unwrap();
        assert_eq!(args.log_dir, PathBuf::from("logs"));
        assert_eq!(args.log_retention_files, 14);
        assert!(!args.disable_file_logging);

        let console_only = Args::try_parse_from([
            "velocitypnpd",
            "--disable-file-logging",
            "--log-retention-files",
            "3",
        ])
        .unwrap();
        assert!(console_only.disable_file_logging);
        assert_eq!(console_only.log_retention_files, 3);
    }

    #[test]
    fn model_cache_uses_xdg_before_home() {
        assert_eq!(
            default_model_cache_root_from(
                Some(Path::new("/var/cache/operator")),
                Some(Path::new("/home/operator")),
            ),
            Some(PathBuf::from("/var/cache/operator/velocitypnp/models"))
        );
        assert_eq!(
            default_model_cache_root_from(None, Some(Path::new("/home/operator"))),
            Some(PathBuf::from("/home/operator/.cache/velocitypnp/models"))
        );
    }

    #[test]
    fn explicit_model_package_disables_managed_selection() {
        let args = Args::try_parse_from([
            "velocitypnpd",
            "--source-component-model-dir",
            "/tmp/source-model",
            "--model-cache-dir",
            "/tmp/model-cache",
        ])
        .unwrap();
        let packages = resolve_model_packages(&args).unwrap();

        assert_eq!(packages.status.mode, ModelSelectionMode::Explicit);
        assert_eq!(
            packages.source_component,
            Some(PathBuf::from("/tmp/source-model"))
        );
        assert!(packages.held_part.is_none());
    }
}
