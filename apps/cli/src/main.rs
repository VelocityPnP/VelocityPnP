use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use reqwest::{multipart, Client, Response};
use serde_json::{json, Value};
use std::path::PathBuf;
use std::time::Duration;
use uuid::Uuid;
use velocitypnp_domain::{JogTarget, Operation, OperationStatus};

#[derive(Debug, Parser)]
#[command(version, about = "VelocityPnP command-line client")]
struct Args {
    #[arg(
        long,
        env = "VELOCITYPNP_API",
        default_value = "http://127.0.0.1:4080/api/v1"
    )]
    api: String,

    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    Status,
    Devices,
    Connect,
    Disconnect,
    Preflight,
    Lease {
        #[arg(default_value = "velocitypnp-cli")]
        client_name: String,
        #[arg(long, default_value_t = 900)]
        seconds: u32,
    },
    RenewLease {
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value_t = 3600)]
        seconds: u32,
    },
    Home {
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        /// Confirm both nozzles are level, the envelope is clear, and power isolation is reachable.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    Jog {
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        x: f64,
        #[arg(long)]
        y: f64,
        #[arg(long)]
        z: f64,
        /// Primary-nozzle rotation. Omit to preserve the reported angle.
        #[arg(long)]
        rotation: Option<f64>,
        #[arg(long, default_value_t = 300.0)]
        feed: f64,
    },
    Actuate {
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        actuator_id: String,
        #[arg(action = clap::ArgAction::Set)]
        enabled: bool,
    },
    ReadSensor {
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        sensor_id: String,
    },
    VisualHome {
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        /// Apply the bounded move and G92 coordinate reset after verification.
        #[arg(long)]
        apply_offset: bool,
        /// Observe at the current known pose instead of moving to the seed.
        #[arg(long)]
        current_pose: bool,
    },
    EmergencyStop,
    ImportBoard {
        pcb: PathBuf,
        #[arg(long)]
        bom: Option<PathBuf>,
        #[arg(long)]
        name: Option<String>,
    },
    ImportOpenpnp {
        machine: PathBuf,
        #[arg(long)]
        parts: Option<PathBuf>,
    },
    Boards,
    DiscoverBoards {
        scan: Uuid,
    },
    DiscoverObjects {
        scan: Uuid,
    },
    SetWorkspaceBaseline {
        scan: Uuid,
    },
    WorkspaceChanges {
        scan: Uuid,
        #[arg(long)]
        baseline: Option<Uuid>,
    },
    RegisterBoard {
        scan: Uuid,
        #[arg(long)]
        candidate: u32,
        #[arg(long)]
        board: Uuid,
        #[arg(long)]
        label: Option<String>,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value_t = 3)]
        samples: u32,
        /// Confirm a fixed PCB, level nozzles, a clear envelope, and reachable
        /// power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    BoardRegistrations,
    BoardRegistration {
        registration: Uuid,
    },
    CalibrationWorkflow,
    CalibrationArtifacts,
    NewCalibration {
        #[arg(long, default_value = "unassigned")]
        machine_serial: String,
    },
    Calibration {
        calibration: Uuid,
    },
    CalibrationRuns {
        calibration: Uuid,
    },
    CalibrationRun {
        run: Uuid,
    },
    CalibrationPlan {
        calibration: Uuid,
        #[arg(long)]
        target: Vec<String>,
    },
    RecomputeDerivedCalibration {
        calibration: Uuid,
        #[arg(long)]
        target: Vec<String>,
    },
    CaptureCalibrationArtifacts {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, required = true)]
        target: Vec<String>,
        /// Confirm level nozzles, a clear envelope, correctly routed nozzle
        /// tubes, and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    Calibrate {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        /// Confirm level nozzles, a clear envelope, correctly routed nozzle
        /// tubes, and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
        #[arg(long, default_value_t = 5)]
        visual_homing_samples: u32,
    },
    CalibrateHeadGeometry {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        /// Confirm level nozzles, a clear envelope, correctly routed nozzle
        /// tubes, and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    VacuumProbe {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        nozzle: String,
        /// Confirm the datum board and sealing target are clean and fixed,
        /// both nozzles are level, the envelope is clear, and power isolation
        /// is reachable.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    FineHome {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value = "N2")]
        nozzle: String,
        /// Confirm the fixed datum and sealing target are clean, both nozzles
        /// are level, the envelope is clear, and power isolation is reachable.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    VacuumCharacterize {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        nozzle: String,
        /// Confirm the datum board and sealing target are clean and fixed,
        /// both nozzles are level, the envelope is clear, and power isolation
        /// is reachable.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    CalibrateXyDirectionResponse {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        /// Confirm level nozzles, a fixed datum board, a clear envelope, and
        /// reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    CalibrateInstalledGeometry {
        calibration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        /// Confirm level nozzles, secure installed hardware, a clear envelope,
        /// correctly routed tubes, and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    ObserveNozzleHeight {
        calibration: Uuid,
        #[arg(long)]
        nozzle: String,
        #[arg(long)]
        nozzle_z_mm: Option<f64>,
        #[arg(long, allow_hyphen_values = true)]
        /// Observed nozzle center minus target center in machine X.
        error_x_mm: f64,
        #[arg(long, allow_hyphen_values = true)]
        /// Observed nozzle center minus target center in machine Y.
        error_y_mm: f64,
        #[arg(long)]
        uncertainty_mm: f64,
        /// Treat the reported error as residual remaining after the last
        /// applied correction; update only this nozzle's working-plane term.
        #[arg(long)]
        post_correction_residual: bool,
    },
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    let client = Client::builder().timeout(Duration::from_secs(90)).build()?;
    let base = args.api.trim_end_matches('/');
    let output = match args.command {
        Command::Status => get(&client, &format!("{base}/machine/status")).await?,
        Command::Devices => get(&client, &format!("{base}/system/devices")).await?,
        Command::Connect => {
            run_operation(
                &client,
                base,
                post(&client, &format!("{base}/machine/connect"), None).await?,
            )
            .await?
        }
        Command::Disconnect => {
            run_operation(
                &client,
                base,
                post(&client, &format!("{base}/machine/disconnect"), None).await?,
            )
            .await?
        }
        Command::Preflight => {
            run_operation(
                &client,
                base,
                post(
                    &client,
                    &format!("{base}/machine/preflight"),
                    Some(json!({"confirmed": true})),
                )
                .await?,
            )
            .await?
        }
        Command::Lease {
            client_name,
            seconds,
        } => {
            post(
                &client,
                &format!("{base}/control/lease"),
                Some(json!({"client_name": client_name, "duration_seconds": seconds})),
            )
            .await?
        }
        Command::RenewLease { lease, seconds } => {
            post(
                &client,
                &format!("{base}/control/lease/renew"),
                Some(json!({"lease_id": lease, "duration_seconds": seconds})),
            )
            .await?
        }
        Command::Home {
            lease,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!("homing requires --confirm-safety-checks after physical inspection");
            }
            run_operation(
                &client,
                base,
                post(
                    &client,
                    &format!("{base}/machine/home"),
                    Some(json!({"lease_id": lease, "confirmed": true})),
                )
                .await?,
            )
            .await?
        }
        Command::Jog {
            lease,
            x,
            y,
            z,
            rotation,
            feed,
        } => {
            let target = JogTarget {
                x_mm: x,
                y_mm: y,
                z_mm: z,
                rotation_deg: rotation,
            };
            run_operation(
                &client,
                base,
                post(
                    &client,
                    &format!("{base}/machine/jog"),
                    Some(json!({"lease_id": lease, "target": target, "feed_mm_per_min": feed})),
                )
                .await?,
            )
            .await?
        }
        Command::Actuate {
            lease,
            actuator_id,
            enabled,
        } => {
            run_operation(
                &client,
                base,
                post(
                    &client,
                    &format!("{base}/machine/actuators/{actuator_id}"),
                    Some(json!({"lease_id": lease, "enabled": enabled})),
                )
                .await?,
            )
            .await?
        }
        Command::ReadSensor { lease, sensor_id } => {
            post(
                &client,
                &format!("{base}/machine/sensors/{sensor_id}/read"),
                Some(json!({"lease_id": lease})),
            )
            .await?
        }
        Command::VisualHome {
            lease,
            apply_offset,
            current_pose,
        } => {
            post(
                &client,
                &format!("{base}/machine/visual-home"),
                Some(json!({
                    "lease_id": lease,
                    "apply_offset": apply_offset,
                    "move_to_seed": !current_pose,
                })),
            )
            .await?
        }
        Command::EmergencyStop => {
            run_operation(
                &client,
                base,
                post(&client, &format!("{base}/machine/emergency-stop"), None).await?,
            )
            .await?
        }
        Command::ImportBoard { pcb, bom, name } => {
            let mut form = multipart::Form::new().part(
                "pcb",
                multipart::Part::bytes(tokio::fs::read(&pcb).await?).file_name(
                    pcb.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                ),
            );
            if let Some(bom) = bom {
                form = form.part(
                    "bom",
                    multipart::Part::bytes(tokio::fs::read(&bom).await?).file_name(
                        bom.file_name()
                            .unwrap_or_default()
                            .to_string_lossy()
                            .into_owned(),
                    ),
                );
            }
            if let Some(name) = name {
                form = form.text("name", name);
            }
            decode(
                client
                    .post(format!("{base}/boards/import"))
                    .multipart(form)
                    .send()
                    .await?,
            )
            .await?
        }
        Command::ImportOpenpnp { machine, parts } => {
            let mut form = multipart::Form::new().part(
                "machine",
                multipart::Part::bytes(tokio::fs::read(&machine).await?).file_name("machine.xml"),
            );
            if let Some(parts) = parts {
                form = form.part(
                    "parts",
                    multipart::Part::bytes(tokio::fs::read(parts).await?).file_name("parts.xml"),
                );
            }
            decode(
                client
                    .post(format!("{base}/import/openpnp"))
                    .multipart(form)
                    .send()
                    .await?,
            )
            .await?
        }
        Command::DiscoverBoards { scan } => {
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/board-discovery"),
                Some(json!({})),
            )
            .await?
        }
        Command::DiscoverObjects { scan } => {
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/object-proposals"),
                Some(json!({})),
            )
            .await?
        }
        Command::SetWorkspaceBaseline { scan } => {
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/inventory-baseline"),
                None,
            )
            .await?
        }
        Command::WorkspaceChanges { scan, baseline } => {
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/change-set"),
                Some(json!({"baseline_id": baseline})),
            )
            .await?
        }
        Command::RegisterBoard {
            scan,
            candidate,
            board,
            label,
            lease,
            samples,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "board registration requires --confirm-safety-checks after inspecting the fixed PCB, level nozzles, travel envelope, and power isolation"
                );
            }
            let started = post(
                &client,
                &format!("{base}/workspace-scans/{scan}/board-registrations"),
                Some(json!({
                    "lease_id": lease,
                    "candidate_index": candidate,
                    "board_id": board,
                    "label": label,
                    "sample_count": samples,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            let registration_id = started["registration"]["id"]
                .as_str()
                .context("board registration response did not contain a registration ID")?;
            run_operation(&client, base, started["operation"].clone()).await?;
            get(
                &client,
                &format!("{base}/board-registrations/{registration_id}"),
            )
            .await?
        }
        Command::BoardRegistrations => get(&client, &format!("{base}/board-registrations")).await?,
        Command::BoardRegistration { registration } => {
            get(
                &client,
                &format!("{base}/board-registrations/{registration}"),
            )
            .await?
        }
        Command::Boards => get(&client, &format!("{base}/boards")).await?,
        Command::CalibrationWorkflow => {
            get(&client, &format!("{base}/calibration/workflow")).await?
        }
        Command::CalibrationArtifacts => {
            get(&client, &format!("{base}/calibration/artifacts")).await?
        }
        Command::NewCalibration { machine_serial } => {
            post(
                &client,
                &format!("{base}/calibrations"),
                Some(json!({"machine_serial": machine_serial})),
            )
            .await?
        }
        Command::Calibration { calibration } => {
            get(&client, &format!("{base}/calibrations/{calibration}")).await?
        }
        Command::CalibrationRuns { calibration } => {
            get(&client, &format!("{base}/calibrations/{calibration}/runs")).await?
        }
        Command::CalibrationRun { run } => {
            get(&client, &format!("{base}/calibration-runs/{run}")).await?
        }
        Command::CalibrationPlan {
            calibration,
            target,
        } => {
            post(
                &client,
                &format!("{base}/calibrations/{calibration}/artifact-plan"),
                Some(json!({"targets": target})),
            )
            .await?
        }
        Command::RecomputeDerivedCalibration {
            calibration,
            target,
        } => {
            post(
                &client,
                &format!("{base}/calibrations/{calibration}/derived-artifacts/recompute"),
                Some(json!({"targets": target})),
            )
            .await?
        }
        Command::CaptureCalibrationArtifacts {
            calibration,
            lease,
            target,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "artifact capture requires --confirm-safety-checks after inspecting the envelope, nozzles, tube routing, and power isolation"
                );
            }
            let started = post(
                &client,
                &format!("{base}/calibrations/{calibration}/artifact-runs"),
                Some(json!({
                    "lease_id": lease,
                    "confirmed_external_safety_checks": true,
                    "targets": target,
                })),
            )
            .await?;
            let run_id = started["run"]["id"]
                .as_str()
                .context("artifact capture response did not contain a run ID")?;
            run_operation(&client, base, started["operation"].clone()).await?;
            get(&client, &format!("{base}/calibration-runs/{run_id}")).await?
        }
        Command::Calibrate {
            calibration,
            lease,
            confirm_safety_checks,
            visual_homing_samples,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "calibration requires --confirm-safety-checks after inspecting the envelope, nozzles, tube routing, and power isolation"
                );
            }
            let started = post(
                &client,
                &format!("{base}/calibrations/{calibration}/runs"),
                Some(json!({
                    "lease_id": lease,
                    "confirmed_external_safety_checks": true,
                    "visual_homing_sample_count": visual_homing_samples,
                })),
            )
            .await?;
            let run_id = started["run"]["id"]
                .as_str()
                .context("calibration response did not contain a run ID")?;
            run_operation(&client, base, started["operation"].clone()).await?;
            get(&client, &format!("{base}/calibration-runs/{run_id}")).await?
        }
        Command::CalibrateHeadGeometry {
            calibration,
            lease,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "head geometry calibration requires --confirm-safety-checks after inspecting the envelope, nozzles, tube routing, and power isolation"
                );
            }
            let started = post(
                &client,
                &format!("{base}/calibrations/{calibration}/head-geometry-runs"),
                Some(json!({
                    "lease_id": lease,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            let run_id = started["run"]["id"]
                .as_str()
                .context("head geometry response did not contain a run ID")?;
            run_operation(&client, base, started["operation"].clone()).await?;
            get(&client, &format!("{base}/calibration-runs/{run_id}")).await?
        }
        Command::VacuumProbe {
            calibration,
            lease,
            nozzle,
            confirm_safety_checks,
        } => {
            if !matches!(nozzle.as_str(), "N1" | "N2") {
                bail!("--nozzle must be N1 or N2");
            }
            if !confirm_safety_checks {
                bail!(
                    "vacuum probing requires --confirm-safety-checks after inspecting the datum board, sealing target, nozzles, envelope, and power isolation"
                );
            }
            let operation = post(
                &client,
                &format!("{base}/calibrations/{calibration}/vacuum-probe"),
                Some(json!({
                    "lease_id": lease,
                    "nozzle_id": nozzle,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            run_operation(&client, base, operation).await?;
            let snapshot = get(&client, &format!("{base}/calibrations/{calibration}")).await?;
            snapshot["values"]["vacuum_probe"][&nozzle].clone()
        }
        Command::FineHome {
            calibration,
            lease,
            nozzle,
            confirm_safety_checks,
        } => {
            if !matches!(nozzle.as_str(), "N1" | "N2") {
                bail!("--nozzle must be N1 or N2");
            }
            if !confirm_safety_checks {
                bail!(
                    "fine homing requires --confirm-safety-checks after inspecting the datum board, sealing target, nozzles, envelope, and power isolation"
                );
            }
            let operation = post(
                &client,
                &format!("{base}/calibrations/{calibration}/fine-home"),
                Some(json!({
                    "lease_id": lease,
                    "nozzle_id": nozzle,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            run_operation(&client, base, operation).await?
        }
        Command::VacuumCharacterize {
            calibration,
            lease,
            nozzle,
            confirm_safety_checks,
        } => {
            if !matches!(nozzle.as_str(), "N1" | "N2") {
                bail!("--nozzle must be N1 or N2");
            }
            if !confirm_safety_checks {
                bail!(
                    "vacuum characterization requires --confirm-safety-checks after inspecting the datum board, sealing target, nozzles, envelope, and power isolation"
                );
            }
            let operation = post(
                &client,
                &format!("{base}/calibrations/{calibration}/vacuum-characterization"),
                Some(json!({
                    "lease_id": lease,
                    "nozzle_id": nozzle,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            run_operation(&client, base, operation).await?;
            let snapshot = get(&client, &format!("{base}/calibrations/{calibration}")).await?;
            snapshot["values"]["vacuum_sensor_characterization"][&nozzle].clone()
        }
        Command::CalibrateXyDirectionResponse {
            calibration,
            lease,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "local XY direction-response calibration requires --confirm-safety-checks after inspecting the level nozzles, fixed datum board, travel envelope, and power isolation"
                );
            }
            let operation = post(
                &client,
                &format!("{base}/calibrations/{calibration}/xy-direction-response"),
                Some(json!({
                    "lease_id": lease,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            run_operation(&client, base, operation).await?;
            let snapshot = get(&client, &format!("{base}/calibrations/{calibration}")).await?;
            snapshot["values"]["local_xy_direction_response"].clone()
        }
        Command::CalibrateInstalledGeometry {
            calibration,
            lease,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "installed geometry calibration requires --confirm-safety-checks after inspecting the level nozzles, installed hardware, tube routing, travel envelope, and power isolation"
                );
            }
            let started = post(
                &client,
                &format!("{base}/calibrations/{calibration}/installed-geometry-runs"),
                Some(json!({
                    "lease_id": lease,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            let run_id = started["run"]["id"]
                .as_str()
                .context("installed geometry response did not contain a run ID")?;
            run_operation(&client, base, started["operation"].clone()).await?;
            get(&client, &format!("{base}/calibration-runs/{run_id}")).await?
        }
        Command::ObserveNozzleHeight {
            calibration,
            nozzle,
            nozzle_z_mm,
            error_x_mm,
            error_y_mm,
            uncertainty_mm,
            post_correction_residual,
        } => {
            if !matches!(nozzle.as_str(), "N1" | "N2") {
                bail!("--nozzle must be N1 or N2");
            }
            post(
                &client,
                &format!("{base}/calibrations/{calibration}/nozzle-height-observation"),
                Some(json!({
                    "nozzle_id": nozzle,
                    "nozzle_z_mm": nozzle_z_mm,
                    "observed_error_x_mm": error_x_mm,
                    "observed_error_y_mm": error_y_mm,
                    "measurement_uncertainty_mm": uncertainty_mm,
                    "post_correction_residual": post_correction_residual,
                })),
            )
            .await?
        }
    };
    println!("{}", serde_json::to_string_pretty(&output)?);
    Ok(())
}

async fn get(client: &Client, url: &str) -> Result<Value> {
    decode(client.get(url).send().await?).await
}

async fn post(client: &Client, url: &str, body: Option<Value>) -> Result<Value> {
    let request = client.post(url);
    decode(match body {
        Some(body) => request.json(&body).send().await?,
        None => request.send().await?,
    })
    .await
}

async fn decode(response: Response) -> Result<Value> {
    let status = response.status();
    let bytes = response.bytes().await?;
    let value: Value = serde_json::from_slice(&bytes).with_context(|| {
        format!(
            "server returned {status}: {}",
            String::from_utf8_lossy(&bytes)
        )
    })?;
    if !status.is_success() {
        bail!(
            "server returned {status}: {}",
            serde_json::to_string(&value)?
        );
    }
    Ok(value)
}

async fn run_operation(client: &Client, base: &str, initial: Value) -> Result<Value> {
    let mut operation: Operation = serde_json::from_value(initial)?;
    loop {
        if !matches!(
            operation.status,
            OperationStatus::Pending | OperationStatus::Running
        ) {
            if operation.status == OperationStatus::Failed {
                bail!(
                    "operation failed: {}",
                    operation
                        .message
                        .unwrap_or_else(|| "unknown failure".into())
                );
            }
            return Ok(serde_json::to_value(operation)?);
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
        operation = serde_json::from_value(
            get(client, &format!("{base}/operations/{}", operation.id)).await?,
        )?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn actuate_accepts_an_explicit_positional_boolean() {
        let lease = Uuid::nil();
        for enabled in [false, true] {
            let args = Args::try_parse_from([
                "velocitypnp",
                "actuate",
                "--lease",
                &lease.to_string(),
                "lights",
                if enabled { "true" } else { "false" },
            ])
            .unwrap();
            assert!(matches!(
                args.command,
                Command::Actuate {
                    actuator_id,
                    enabled: parsed,
                    ..
                } if actuator_id == "lights" && parsed == enabled
            ));
        }
    }

    #[test]
    fn discover_boards_accepts_a_workspace_scan_id() {
        let scan = Uuid::new_v4();
        let args =
            Args::try_parse_from(["velocitypnp", "discover-boards", &scan.to_string()]).unwrap();
        assert!(matches!(
            args.command,
            Command::DiscoverBoards { scan: parsed } if parsed == scan
        ));

        let board = Uuid::new_v4();
        let lease = Uuid::new_v4();
        let args = Args::try_parse_from([
            "velocitypnp",
            "register-board",
            &scan.to_string(),
            "--candidate",
            "2",
            "--board",
            &board.to_string(),
            "--lease",
            &lease.to_string(),
            "--confirm-safety-checks",
        ])
        .unwrap();
        assert!(matches!(
            args.command,
            Command::RegisterBoard {
                scan: parsed_scan,
                candidate: 2,
                board: parsed_board,
                lease: parsed_lease,
                ..
            } if parsed_scan == scan && parsed_board == board && parsed_lease == lease
        ));

        let args =
            Args::try_parse_from(["velocitypnp", "discover-objects", &scan.to_string()]).unwrap();
        assert!(matches!(
            args.command,
            Command::DiscoverObjects { scan: parsed } if parsed == scan
        ));

        let args =
            Args::try_parse_from(["velocitypnp", "set-workspace-baseline", &scan.to_string()])
                .unwrap();
        assert!(matches!(
            args.command,
            Command::SetWorkspaceBaseline { scan: parsed } if parsed == scan
        ));

        let baseline = Uuid::new_v4();
        let args = Args::try_parse_from([
            "velocitypnp",
            "workspace-changes",
            &scan.to_string(),
            "--baseline",
            &baseline.to_string(),
        ])
        .unwrap();
        assert!(matches!(
            args.command,
            Command::WorkspaceChanges {
                scan: parsed_scan,
                baseline: Some(parsed_baseline),
            } if parsed_scan == scan && parsed_baseline == baseline
        ));
    }
}
