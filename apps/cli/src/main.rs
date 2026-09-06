use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand, ValueEnum};
use reqwest::{multipart, Client, Response};
use serde_json::{json, Value};
use std::path::PathBuf;
use std::time::{Duration, Instant};
use uuid::Uuid;
use velocitypnp_domain::{
    JogTarget, MachineProfile, MachineState, MachineStatus, Operation, OperationStatus, Pose,
};

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
    /// Capture repeated cold camera frame series without a warm-up discard.
    CameraSettling {
        role: String,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value_t = 80, value_parser = clap::value_parser!(u32).range(1..=120))]
        samples: u32,
        #[arg(long, default_value_t = 5, value_parser = clap::value_parser!(u32).range(1..=10))]
        repetitions: u32,
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
    /// Measure clear-XY travel time and camera-based return repeatability.
    MotionSweep {
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        /// Ascending XY feed rates in mm/min.
        #[arg(long = "feed", num_args = 1..)]
        feeds: Vec<f64>,
        #[arg(long, default_value_t = 3, value_parser = clap::value_parser!(u32).range(1..=20))]
        cycles: u32,
        /// Axis or axes to exercise.
        #[arg(long, value_enum, default_value = "xy")]
        axis: MotionSweepAxis,
        #[arg(long, default_value_t = 150.0)]
        x_excursion_mm: f64,
        #[arg(long, default_value_t = 100.0)]
        y_excursion_mm: f64,
        #[arg(long, default_value_t = 0.05)]
        return_tolerance_mm: f64,
        /// Confirm level nozzles, a clear XY path at safe Z, and reachable
        /// power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
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
    AnalyzeStripFeeder {
        scan: Uuid,
        #[arg(long)]
        candidate: u32,
    },
    BuildStripFeederInventory {
        scan: Uuid,
        /// Repeated SCAN_UUID:CANDIDATE_INDEX feeder evidence source.
        #[arg(long = "source", required = true)]
        sources: Vec<String>,
    },
    PromoteStripFeederInventory {
        scan: Uuid,
    },
    CaptureStripFeederSources {
        scan: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value_t = 12)]
        occupied: u32,
        #[arg(long, default_value_t = 12)]
        empty: u32,
        /// Inventory lane index. Repeat to restrict the capture batch.
        #[arg(long = "lane")]
        lanes: Vec<u32>,
        /// Confirm fixed tape, level nozzles, a clear envelope, and reachable
        /// power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    StripFeederSourceCapture {
        capture: Uuid,
    },
    SaveStripFeederDraft {
        scan: Uuid,
        #[arg(long)]
        candidate: u32,
        #[arg(long)]
        proposal: u32,
        #[arg(long)]
        name: Option<String>,
        #[arg(long)]
        part_key: Option<String>,
        #[arg(long)]
        part_rotation: Option<f64>,
    },
    StripFeederDrafts,
    LiveStripFeederHoldout {
        registration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        pocket: Option<u32>,
        #[arg(long, default_value_t = 3)]
        samples: u32,
        /// Confirm fixed tape, level nozzles, a clear envelope, and reachable
        /// power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    StripFeederHoldout {
        holdout: Uuid,
    },
    QualifyStripFeederPickup {
        registration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value = "N1")]
        nozzle: String,
        #[arg(long)]
        pocket: Option<u32>,
        /// Repeat one returned part while one camera scope stays open.
        #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u32).range(1..=5))]
        cycles: u32,
        /// Fallback surface X. Supply this with --drop-y.
        #[arg(long, requires = "drop_y")]
        drop_x: Option<f64>,
        /// Fallback surface Y. Supply this with --drop-x.
        #[arg(long, requires = "drop_x")]
        drop_y: Option<f64>,
        /// Capture same-angle bare and held bottom-camera pairs.
        #[arg(
            long = "rotation-capture-offset",
            num_args = 1..=12,
            allow_hyphen_values = true
        )]
        rotation_capture_offsets: Vec<f64>,
        /// Confirm uncovered fixed tape, level nozzles, a clear envelope and
        /// fallback target, and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    QualifyStripFeederSequence {
        registration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value = "N1")]
        nozzle: String,
        #[arg(long, default_value_t = 3, value_parser = clap::value_parser!(u32).range(3..=5))]
        cycles: u32,
        /// Fallback surface X. Supply this with --drop-y.
        #[arg(long, requires = "drop_y")]
        drop_x: Option<f64>,
        /// Fallback surface Y. Supply this with --drop-x.
        #[arg(long, requires = "drop_x")]
        drop_y: Option<f64>,
        /// Confirm uncovered fixed tape, level nozzles, a clear envelope and
        /// fallback target, and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    StripFeederPickupQualification {
        qualification: Uuid,
    },
    /// Recheck retained before/after return images without machine motion.
    ReanalyzeStripFeederReturn {
        qualification: Uuid,
    },
    ExecuteSinglePlacement {
        job: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        calibration: Uuid,
        #[arg(long, default_value_t = 0)]
        step: u32,
        /// Confirm fixed feeders and PCB, level nozzles, a clear envelope,
        /// and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    QualifyPlacementSequence {
        job: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        calibration: Uuid,
        #[arg(long = "step", required = true, num_args = 1..=5)]
        steps: Vec<u32>,
        /// Confirm fixed feeders and PCB, level nozzles, a clear envelope,
        /// and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    ExecuteJob {
        job: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        calibration: Uuid,
        /// Confirm fixed feeders and PCB, level nozzles, a clear envelope,
        /// and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    JobExecutions,
    JobExecution {
        execution: Uuid,
    },
    PauseJobExecution {
        execution: Uuid,
    },
    ResumeJobExecution {
        execution: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    CancelJobExecution {
        execution: Uuid,
    },
    ResolveJobStep {
        execution: Uuid,
        step: u32,
        action: String,
        #[arg(long)]
        note: Option<String>,
    },
    PlacementExecution {
        execution: Uuid,
    },
    PlacementOutcome {
        execution: Uuid,
    },
    ReviewPlacementOutcome {
        execution: Uuid,
        disposition: String,
        #[arg(long)]
        note: Option<String>,
    },
    PlacementProcess {
        part: Uuid,
        #[arg(long, default_value = "N1")]
        nozzle: String,
        #[arg(long, default_value_t = 0.0)]
        surface_offset: f64,
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
    MeasureBoardSurface {
        registration: Uuid,
        #[arg(long, env = "VELOCITYPNP_LEASE")]
        lease: Uuid,
        #[arg(long, default_value = "N1")]
        nozzle: String,
        #[arg(long, default_value_t = 3)]
        samples: u32,
        /// Confirm a fixed PCB, visible clear fiducials, level nozzles, a
        /// clear envelope, and reachable power isolation.
        #[arg(long)]
        confirm_safety_checks: bool,
    },
    BoardSurfaceMeasurement {
        measurement: Uuid,
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
        Command::CameraSettling {
            role,
            lease,
            samples,
            repetitions,
        } => {
            let mut runs = Vec::with_capacity(repetitions as usize);
            for _ in 0..repetitions {
                runs.push(
                    post(
                        &client,
                        &format!("{base}/cameras/{role}/reproducibility"),
                        Some(json!({
                            "lease_id": lease,
                            "sample_count": samples,
                            "settle_frames": 0,
                        })),
                    )
                    .await?,
                );
            }
            json!({
                "role": role,
                "repetitions": repetitions,
                "samples_per_repetition": samples,
                "discarded_settle_frames": 0,
                "runs": runs,
            })
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
        Command::MotionSweep {
            lease,
            feeds,
            cycles,
            axis,
            x_excursion_mm,
            y_excursion_mm,
            return_tolerance_mm,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "motion sweep requires --confirm-safety-checks after inspecting the level nozzles, clear XY path at safe Z, and power isolation"
                );
            }
            run_motion_sweep(
                &client,
                base,
                lease,
                feeds,
                cycles,
                axis,
                x_excursion_mm,
                y_excursion_mm,
                return_tolerance_mm,
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
        Command::AnalyzeStripFeeder { scan, candidate } => {
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/strip-feeder-vision"),
                Some(json!({"candidate_index": candidate})),
            )
            .await?
        }
        Command::BuildStripFeederInventory { scan, sources } => {
            let evidence_sources = sources
                .iter()
                .map(|source| {
                    let (scan_id, candidate_index) = source.split_once(':').with_context(|| {
                        format!(
                            "invalid feeder source {source}; expected SCAN_UUID:CANDIDATE_INDEX"
                        )
                    })?;
                    Ok(json!({
                        "scan_id": scan_id.parse::<Uuid>().with_context(|| {
                            format!("invalid feeder source scan UUID {scan_id}")
                        })?,
                        "candidate_index": candidate_index.parse::<u32>().with_context(|| {
                            format!("invalid feeder candidate index {candidate_index}")
                        })?,
                    }))
                })
                .collect::<Result<Vec<_>>>()?;
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/strip-feeder-inventory"),
                Some(json!({"evidence_sources": evidence_sources})),
            )
            .await?
        }
        Command::PromoteStripFeederInventory { scan } => {
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/strip-feeder-inventory/registrations"),
                Some(json!({})),
            )
            .await?
        }
        Command::CaptureStripFeederSources {
            scan,
            lease,
            occupied,
            empty,
            lanes,
            confirm_safety_checks,
        } => {
            post(
                &client,
                &format!("{base}/workspace-scans/{scan}/strip-feeder-inventory/source-captures"),
                Some(json!({
                    "lease_id": lease,
                    "occupied_count": occupied,
                    "empty_count": empty,
                    "lane_indices": lanes,
                    "confirmed_external_safety_checks": confirm_safety_checks,
                })),
            )
            .await?
        }
        Command::StripFeederSourceCapture { capture } => {
            get(
                &client,
                &format!("{base}/strip-feeder-source-captures/{capture}"),
            )
            .await?
        }
        Command::SaveStripFeederDraft {
            scan,
            candidate,
            proposal,
            name,
            part_key,
            part_rotation,
        } => {
            post(
                &client,
                &format!("{base}/strip-feeder-registrations"),
                Some(json!({
                    "scan_id": scan,
                    "candidate_index": candidate,
                    "proposal_index": proposal,
                    "name": name,
                    "part_key": part_key,
                    "part_rotation_deg": part_rotation,
                })),
            )
            .await?
        }
        Command::StripFeederDrafts => {
            get(&client, &format!("{base}/strip-feeder-registrations")).await?
        }
        Command::LiveStripFeederHoldout {
            registration,
            lease,
            pocket,
            samples,
            confirm_safety_checks,
        } => {
            post(
                &client,
                &format!("{base}/strip-feeder-registrations/{registration}/live-holdouts"),
                Some(json!({
                    "lease_id": lease,
                    "pocket_index": pocket,
                    "sample_count": samples,
                    "confirmed_external_safety_checks": confirm_safety_checks,
                })),
            )
            .await?
        }
        Command::StripFeederHoldout { holdout } => {
            get(
                &client,
                &format!("{base}/strip-feeder-live-holdouts/{holdout}"),
            )
            .await?
        }
        Command::QualifyStripFeederPickup {
            registration,
            lease,
            nozzle,
            pocket,
            cycles,
            drop_x,
            drop_y,
            rotation_capture_offsets,
            confirm_safety_checks,
        } => {
            post(
                &client,
                &format!("{base}/strip-feeder-registrations/{registration}/pickup-qualifications"),
                Some(json!({
                    "lease_id": lease,
                    "nozzle_id": nozzle,
                    "pocket_index": pocket,
                    "cycle_count": cycles,
                    "fallback_drop_surface_machine_mm": drop_x.zip(drop_y).map(|(x, y)| [x, y]),
                    "rotation_capture_offsets_deg": rotation_capture_offsets,
                    "confirmed_external_safety_checks": confirm_safety_checks,
                })),
            )
            .await?
        }
        Command::QualifyStripFeederSequence {
            registration,
            lease,
            nozzle,
            cycles,
            drop_x,
            drop_y,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "pickup sequence requires --confirm-safety-checks after inspecting the uncovered tape, level nozzles, travel envelope, fallback target, and power isolation"
                );
            }
            let draft = get(
                &client,
                &format!("{base}/strip-feeder-registrations/{registration}"),
            )
            .await?;
            let pockets = draft["pockets"]
                .as_array()
                .context("strip feeder draft has no pocket state")?
                .iter()
                .filter(|pocket| {
                    pocket["status"] == "available" && pocket["occupancy"] == "occupied"
                })
                .filter_map(|pocket| pocket["pocket_index"].as_u64())
                .take(cycles as usize)
                .collect::<Vec<_>>();
            if pockets.len() != cycles as usize {
                bail!("pickup sequence does not have {cycles} distinct available pockets");
            }
            let mut qualifications = Vec::new();
            for pocket in pockets {
                let started = post(
                    &client,
                    &format!(
                        "{base}/strip-feeder-registrations/{registration}/pickup-qualifications"
                    ),
                    Some(json!({
                        "lease_id": lease,
                        "nozzle_id": nozzle,
                        "pocket_index": pocket,
                        "fallback_drop_surface_machine_mm": drop_x.zip(drop_y).map(|(x, y)| [x, y]),
                        "confirmed_external_safety_checks": true,
                    })),
                )
                .await?;
                let qualification_id = started["qualification"]["id"]
                    .as_str()
                    .context("pickup response did not contain a qualification ID")?;
                let operation_result =
                    run_operation(&client, base, started["operation"].clone()).await;
                let qualification = get(
                    &client,
                    &format!("{base}/strip-feeder-pickup-qualifications/{qualification_id}"),
                )
                .await?;
                let passed = qualification["report"]["passed"] == true;
                qualifications.push(qualification);
                if operation_result.is_err() || !passed {
                    break;
                }
            }
            json!({
                "registration_id": registration,
                "requested_cycle_count": cycles,
                "completed_cycle_count": qualifications.len(),
                "passed": qualifications.len() == cycles as usize
                    && qualifications.iter().all(|run| run["report"]["passed"] == true),
                "qualifications": qualifications,
            })
        }
        Command::StripFeederPickupQualification { qualification } => {
            get(
                &client,
                &format!("{base}/strip-feeder-pickup-qualifications/{qualification}"),
            )
            .await?
        }
        Command::ReanalyzeStripFeederReturn { qualification } => {
            post(
                &client,
                &format!(
                    "{base}/strip-feeder-pickup-qualifications/{qualification}/reanalyze-return"
                ),
                None,
            )
            .await?
        }
        Command::ExecuteSinglePlacement {
            job,
            lease,
            calibration,
            step,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "single placement requires --confirm-safety-checks after inspecting the fixed feeders and PCB, level nozzles, travel envelope, and power isolation"
                );
            }
            let started = post(
                &client,
                &format!("{base}/jobs/{job}/placement-executions"),
                Some(json!({
                    "lease_id": lease,
                    "calibration_id": calibration,
                    "step_index": step,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            let execution_id = started["execution"]["id"]
                .as_str()
                .context("placement response did not contain an execution ID")?;
            run_operation(&client, base, started["operation"].clone()).await?;
            get(
                &client,
                &format!("{base}/placement-executions/{execution_id}"),
            )
            .await?
        }
        Command::QualifyPlacementSequence {
            job,
            lease,
            calibration,
            steps,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "placement sequence requires --confirm-safety-checks after inspecting the fixed feeders and PCB, level nozzles, travel envelope, and power isolation"
                );
            }
            if !(1..=5).contains(&steps.len()) {
                bail!("placement sequence requires one to five steps");
            }
            let distinct = steps
                .iter()
                .copied()
                .collect::<std::collections::BTreeSet<_>>();
            if distinct.len() != steps.len() {
                bail!("placement sequence steps must be distinct");
            }
            let plan = get(&client, &format!("{base}/jobs/{job}/plan")).await?;
            let plan_steps = plan["plan"]["steps"]
                .as_array()
                .context("job plan has no placement steps")?;
            let selected = steps
                .iter()
                .map(|step| {
                    plan_steps
                        .get(*step as usize)
                        .with_context(|| format!("placement step {step} is outside the job plan"))
                })
                .collect::<Result<Vec<_>>>()?;
            let qualification_key = selected.first().map(|step| {
                (
                    step["part_key"].clone(),
                    step["nozzle_id"].clone(),
                    step["placement_seating_offset_mm"].clone(),
                    step["placement_surface_offset_mm"].clone(),
                )
            });
            if selected.iter().any(|step| {
                Some((
                    step["part_key"].clone(),
                    step["nozzle_id"].clone(),
                    step["placement_seating_offset_mm"].clone(),
                    step["placement_surface_offset_mm"].clone(),
                )) != qualification_key
            }) {
                bail!(
                    "placement sequence steps must use one part, nozzle, seating offset, and surface offset"
                );
            }
            let mut executions = Vec::new();
            let mut outcomes = Vec::new();
            for step in &steps {
                let started = post(
                    &client,
                    &format!("{base}/jobs/{job}/placement-executions"),
                    Some(json!({
                        "lease_id": lease,
                        "calibration_id": calibration,
                        "step_index": step,
                        "confirmed_external_safety_checks": true,
                    })),
                )
                .await?;
                let execution_id = started["execution"]["id"]
                    .as_str()
                    .context("placement response did not contain an execution ID")?;
                let operation_result =
                    run_operation(&client, base, started["operation"].clone()).await;
                let execution = get(
                    &client,
                    &format!("{base}/placement-executions/{execution_id}"),
                )
                .await?;
                let outcome = get(
                    &client,
                    &format!("{base}/placement-executions/{execution_id}/outcome"),
                )
                .await?;
                let passed = execution["report"]["passed"] == true
                    && outcome["automatic_disposition"] == "good";
                executions.push(execution);
                outcomes.push(outcome);
                if operation_result.is_err() || !passed {
                    break;
                }
            }
            json!({
                "job_id": job,
                "requested_steps": steps,
                "completed_count": executions.len(),
                "passed": executions.len() == steps.len()
                    && outcomes.iter().all(|outcome| {
                        outcome["automatic_disposition"] == "good"
                    }),
                "executions": executions,
                "outcomes": outcomes,
            })
        }
        Command::ExecuteJob {
            job,
            lease,
            calibration,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "job execution requires --confirm-safety-checks after inspecting the fixed feeders and PCB, level nozzles, travel envelope, and power isolation"
                );
            }
            post(
                &client,
                &format!("{base}/jobs/{job}/executions"),
                Some(json!({
                    "lease_id": lease,
                    "calibration_id": calibration,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?
        }
        Command::JobExecutions => get(&client, &format!("{base}/job-executions")).await?,
        Command::JobExecution { execution } => {
            get(&client, &format!("{base}/job-executions/{execution}")).await?
        }
        Command::PauseJobExecution { execution } => {
            post(
                &client,
                &format!("{base}/job-executions/{execution}/pause"),
                None,
            )
            .await?
        }
        Command::ResumeJobExecution {
            execution,
            lease,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "job resume requires --confirm-safety-checks after inspecting the fixed feeders and PCB, level nozzles, travel envelope, and power isolation"
                );
            }
            post(
                &client,
                &format!("{base}/job-executions/{execution}/resume"),
                Some(json!({
                    "lease_id": lease,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?
        }
        Command::CancelJobExecution { execution } => {
            post(
                &client,
                &format!("{base}/job-executions/{execution}/cancel"),
                None,
            )
            .await?
        }
        Command::ResolveJobStep {
            execution,
            step,
            action,
            note,
        } => {
            if !matches!(action.as_str(), "retry" | "skip") {
                bail!("job step action must be retry or skip");
            }
            post(
                &client,
                &format!("{base}/job-executions/{execution}/steps/{step}/resolve"),
                Some(json!({ "action": action, "note": note })),
            )
            .await?
        }
        Command::PlacementExecution { execution } => {
            get(&client, &format!("{base}/placement-executions/{execution}")).await?
        }
        Command::PlacementOutcome { execution } => {
            get(
                &client,
                &format!("{base}/placement-executions/{execution}/outcome"),
            )
            .await?
        }
        Command::ReviewPlacementOutcome {
            execution,
            disposition,
            note,
        } => {
            post(
                &client,
                &format!("{base}/placement-executions/{execution}/outcome"),
                Some(json!({
                    "disposition": disposition,
                    "note": note,
                })),
            )
            .await?
        }
        Command::PlacementProcess {
            part,
            nozzle,
            surface_offset,
        } => {
            get(
                &client,
                &format!(
                    "{base}/parts/{part}/placement-process/{nozzle}/surfaces/{surface_offset}"
                ),
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
        Command::MeasureBoardSurface {
            registration,
            lease,
            nozzle,
            samples,
            confirm_safety_checks,
        } => {
            if !confirm_safety_checks {
                bail!(
                    "board surface measurement requires --confirm-safety-checks after inspecting the fixed PCB, visible clear fiducials, level nozzles, travel envelope, and power isolation"
                );
            }
            let started = post(
                &client,
                &format!("{base}/board-registrations/{registration}/surface-measurements"),
                Some(json!({
                    "lease_id": lease,
                    "nozzle_id": nozzle,
                    "sample_count": samples,
                    "confirmed_external_safety_checks": true,
                })),
            )
            .await?;
            let measurement_id = started["measurement"]["id"]
                .as_str()
                .context("board surface response did not contain a measurement ID")?;
            run_operation(&client, base, started["operation"].clone()).await?;
            get(
                &client,
                &format!("{base}/board-surface-measurements/{measurement_id}"),
            )
            .await?
        }
        Command::BoardSurfaceMeasurement { measurement } => {
            get(
                &client,
                &format!("{base}/board-surface-measurements/{measurement}"),
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

const DEFAULT_MOTION_SWEEP_FEEDS: [f64; 5] = [1_200.0, 3_000.0, 6_000.0, 9_000.0, 12_000.0];
const MOTION_SWEEP_BASELINE_FEED_MM_PER_MIN: f64 = 1_200.0;
const MOTION_SWEEP_BASELINE_SAMPLES: usize = 3;

#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
enum MotionSweepAxis {
    Xy,
    X,
    Y,
}

impl MotionSweepAxis {
    fn as_str(self) -> &'static str {
        match self {
            Self::Xy => "xy",
            Self::X => "x",
            Self::Y => "y",
        }
    }
}

struct MotionSweepPlan {
    axis: MotionSweepAxis,
    feeds: Vec<f64>,
    seed: Pose,
    low: Pose,
    high: Pose,
}

#[allow(clippy::too_many_arguments)]
async fn run_motion_sweep(
    client: &Client,
    base: &str,
    lease: Uuid,
    feeds: Vec<f64>,
    cycles: u32,
    axis: MotionSweepAxis,
    x_excursion_mm: f64,
    y_excursion_mm: f64,
    return_tolerance_mm: f64,
) -> Result<Value> {
    let status_value = get(client, &format!("{base}/machine/status")).await?;
    let status: MachineStatus = serde_json::from_value(status_value["machine"].clone())
        .context("machine status response did not contain a valid machine")?;
    let profile: MachineProfile = get(client, &format!("{base}/machine/profile"))
        .await
        .and_then(|value| {
            serde_json::from_value(value).context("machine profile response was invalid")
        })?;
    let plan = motion_sweep_plan(
        &status,
        &profile,
        feeds,
        axis,
        x_excursion_mm,
        y_excursion_mm,
        return_tolerance_mm,
    )?;

    let initial_move = timed_jog(
        client,
        base,
        lease,
        status.pose.unwrap(),
        plan.seed,
        MOTION_SWEEP_BASELINE_FEED_MM_PER_MIN,
    )
    .await
    .context("moving to the motion-sweep seed")?;
    let conditioning_outbound = timed_jog(
        client,
        base,
        lease,
        plan.seed,
        plan.high,
        MOTION_SWEEP_BASELINE_FEED_MM_PER_MIN,
    )
    .await
    .context("conditioning the motion-sweep baseline approach")?;
    let conditioning_return = timed_jog(
        client,
        base,
        lease,
        plan.high,
        plan.seed,
        MOTION_SWEEP_BASELINE_FEED_MM_PER_MIN,
    )
    .await
    .context("returning from the motion-sweep baseline approach")?;
    let mut baseline_observations = Vec::with_capacity(MOTION_SWEEP_BASELINE_SAMPLES);
    let mut baseline_corrections = Vec::with_capacity(MOTION_SWEEP_BASELINE_SAMPLES);
    for sample in 1..=MOTION_SWEEP_BASELINE_SAMPLES {
        let observation = observe_visual_home(client, base, lease)
            .await
            .with_context(|| format!("capturing motion-sweep baseline sample {sample}"))?;
        if !visual_observation_valid(&observation) {
            bail!("motion-sweep baseline sample {sample} exceeded the fiducial bounds");
        }
        baseline_corrections.push(visual_correction(&observation)?);
        baseline_observations.push(observation["initial"].clone());
    }
    let baseline_correction = mean_correction(&baseline_corrections);
    let baseline_max_deviation_mm = baseline_corrections
        .iter()
        .map(|correction| {
            (correction[0] - baseline_correction[0]).hypot(correction[1] - baseline_correction[1])
        })
        .fold(0.0_f64, f64::max);
    let baseline_max_pairwise_error_mm = max_pairwise_correction_error(&baseline_corrections);
    if baseline_max_pairwise_error_mm > return_tolerance_mm {
        bail!(
            "stationary camera spread {baseline_max_pairwise_error_mm:.4}mm exceeds the motion-sweep tolerance of {return_tolerance_mm:.4}mm"
        );
    }

    let mut passed = true;
    let mut stopped_early = false;
    let mut stop_reason: Option<String> = None;
    let mut levels = Vec::with_capacity(plan.feeds.len());
    for feed in &plan.feeds {
        let mut samples = Vec::with_capacity(cycles as usize);
        let mut return_errors = Vec::with_capacity(cycles as usize);
        let mut level_passed = true;
        for cycle in 1..=cycles {
            let mut moves = Vec::with_capacity(3);
            let mut motion_error = None;
            for (stage, from, target) in [
                ("outbound", plan.seed, plan.low),
                ("traverse", plan.low, plan.high),
                ("return", plan.high, plan.seed),
            ] {
                match timed_jog(client, base, lease, from, target, *feed).await {
                    Ok(motion) => moves.push(motion),
                    Err(error) => {
                        motion_error = Some((stage, format!("{error:#}")));
                        break;
                    }
                }
            }
            if let Some((stage, error)) = motion_error {
                passed = false;
                level_passed = false;
                stopped_early = true;
                stop_reason = Some(format!(
                    "feed {feed:.0}, cycle {cycle}: {stage} move failed: {error}"
                ));
                samples.push(json!({
                    "cycle": cycle,
                    "moves": moves,
                    "failed_stage": stage,
                    "motion_error": error,
                    "passed": false,
                }));
                break;
            }
            let observation = match observe_visual_home(client, base, lease).await {
                Ok(observation) => observation,
                Err(error) => {
                    let error = format!("{error:#}");
                    passed = false;
                    level_passed = false;
                    stopped_early = true;
                    stop_reason = Some(format!(
                        "feed {feed:.0}, cycle {cycle}: return observation failed: {error}"
                    ));
                    samples.push(json!({
                        "cycle": cycle,
                        "moves": moves,
                        "failed_stage": "return_observation",
                        "observation_error": error,
                        "observation_valid": false,
                        "passed": false,
                    }));
                    break;
                }
            };
            let correction = match visual_correction(&observation) {
                Ok(correction) => correction,
                Err(error) => {
                    let error = format!("{error:#}");
                    passed = false;
                    level_passed = false;
                    stopped_early = true;
                    stop_reason = Some(format!(
                        "feed {feed:.0}, cycle {cycle}: invalid return result: {error}"
                    ));
                    samples.push(json!({
                        "cycle": cycle,
                        "moves": moves,
                        "failed_stage": "return_result",
                        "observation_error": error,
                        "observation_valid": false,
                        "passed": false,
                    }));
                    break;
                }
            };
            let error_x_mm = correction[0] - baseline_correction[0];
            let error_y_mm = correction[1] - baseline_correction[1];
            let return_error_mm = error_x_mm.hypot(error_y_mm);
            let observation_valid = visual_observation_valid(&observation);
            let sample_passed = observation_valid && return_error_mm <= return_tolerance_mm;
            return_errors.push(return_error_mm);
            samples.push(json!({
                "cycle": cycle,
                "moves": moves,
                "observed_correction_x_mm": correction[0],
                "observed_correction_y_mm": correction[1],
                "return_error_x_mm": error_x_mm,
                "return_error_y_mm": error_y_mm,
                "return_error_mm": return_error_mm,
                "observation_valid": observation_valid,
                "passed": sample_passed,
            }));
            if !sample_passed {
                passed = false;
                level_passed = false;
                stopped_early = true;
                stop_reason = Some(if observation_valid {
                    format!(
                        "feed {feed:.0}, cycle {cycle}: return error {return_error_mm:.4}mm exceeds {return_tolerance_mm:.4}mm"
                    )
                } else {
                    format!(
                        "feed {feed:.0}, cycle {cycle}: fiducial observation exceeded its configured bounds"
                    )
                });
                break;
            }
        }
        let mean_return_error_mm = (!return_errors.is_empty())
            .then(|| return_errors.iter().sum::<f64>() / return_errors.len() as f64);
        let max_return_error_mm = (!return_errors.is_empty())
            .then(|| return_errors.iter().copied().fold(0.0_f64, f64::max));
        levels.push(json!({
            "feed_mm_per_min": feed,
            "passed": level_passed && return_errors.len() == cycles as usize,
            "mean_return_error_mm": mean_return_error_mm,
            "max_return_error_mm": max_return_error_mm,
            "samples": samples,
        }));
        if stopped_early {
            break;
        }
    }

    Ok(json!({
        "kind": "clear_xy_motion_sweep",
        "profile_id": profile.id,
        "axis": plan.axis.as_str(),
        "passed": passed,
        "stopped_early": stopped_early,
        "stop_reason": stop_reason,
        "cycles_per_feed": cycles,
        "baseline_feed_mm_per_min": MOTION_SWEEP_BASELINE_FEED_MM_PER_MIN,
        "return_tolerance_mm": return_tolerance_mm,
        "requested_feeds_mm_per_min": plan.feeds,
        "points": {
            "seed": plan.seed,
            "low": plan.low,
            "high": plan.high,
        },
        "controller": status.controller,
        "initial_move": initial_move,
        "conditioning_moves": [conditioning_outbound, conditioning_return],
        "baseline": {
            "correction_x_mm": baseline_correction[0],
            "correction_y_mm": baseline_correction[1],
            "max_deviation_mm": baseline_max_deviation_mm,
            "max_pairwise_error_mm": baseline_max_pairwise_error_mm,
            "observations": baseline_observations,
        },
        "levels": levels,
    }))
}

fn motion_sweep_plan(
    status: &MachineStatus,
    profile: &MachineProfile,
    feeds: Vec<f64>,
    axis: MotionSweepAxis,
    x_excursion_mm: f64,
    y_excursion_mm: f64,
    return_tolerance_mm: f64,
) -> Result<MotionSweepPlan> {
    if status.simulated {
        bail!("motion sweep requires a physical machine");
    }
    if status.state != MachineState::Ready || !status.homed {
        bail!("motion sweep requires a homed ready machine");
    }
    if !profile.accepts_profile_id(&status.profile_id) {
        bail!("machine status and active profile do not match");
    }
    let selected_excursions = match axis {
        MotionSweepAxis::Xy => [Some(x_excursion_mm), Some(y_excursion_mm)],
        MotionSweepAxis::X => [Some(x_excursion_mm), None],
        MotionSweepAxis::Y => [None, Some(y_excursion_mm)],
    };
    if selected_excursions
        .into_iter()
        .flatten()
        .any(|excursion| !excursion.is_finite() || excursion <= 0.0)
    {
        bail!("selected motion-sweep excursions must be finite and positive");
    }
    if !return_tolerance_mm.is_finite() || return_tolerance_mm <= 0.0 {
        bail!("motion-sweep return tolerance must be finite and positive");
    }
    let feeds = if feeds.is_empty() {
        DEFAULT_MOTION_SWEEP_FEEDS.to_vec()
    } else {
        feeds
    };
    if feeds
        .iter()
        .any(|feed| !feed.is_finite() || *feed <= 0.0 || *feed > profile.maximum_feed_mm_per_min)
    {
        bail!(
            "motion-sweep feeds must be positive and no more than the profile limit of {:.0} mm/min",
            profile.maximum_feed_mm_per_min
        );
    }
    if MOTION_SWEEP_BASELINE_FEED_MM_PER_MIN > profile.maximum_feed_mm_per_min {
        bail!("motion-sweep baseline feed exceeds the active profile limit");
    }
    if feeds.windows(2).any(|pair| pair[0] >= pair[1]) {
        bail!("motion-sweep feeds must be in strictly ascending order");
    }
    let controller = status
        .controller
        .as_ref()
        .context("motion sweep requires recorded controller settings")?;
    for axis_letter in ["X", "Y"] {
        let axis_limit = controller
            .settings
            .max_feed_units_per_second
            .get(axis_letter)
            .with_context(|| {
                format!("controller settings do not include the {axis_letter} feed limit")
            })?;
        let highest_feed = feeds
            .last()
            .copied()
            .unwrap()
            .max(MOTION_SWEEP_BASELINE_FEED_MM_PER_MIN);
        if highest_feed / 60.0 > *axis_limit {
            bail!(
                "requested feed {highest_feed:.0} mm/min exceeds controller {axis_letter} limit of {:.0} mm/min",
                axis_limit * 60.0
            );
        }
    }

    let current = status.pose.context("motion sweep requires a known pose")?;
    let visual = profile
        .visual_homing
        .as_ref()
        .context("active profile does not define visual homing")?;
    visual
        .validate()
        .map_err(anyhow::Error::msg)
        .context("active visual-homing profile is invalid")?;
    let seed = visual
        .seed_pose
        .resolve(current)
        .context("visual-homing seed pose is invalid")?;
    if (seed.z_mm - profile.safe_z_mm).abs() > 1e-6 {
        bail!("motion-sweep seed must be at the configured safe Z");
    }
    let (x_excursion_mm, y_excursion_mm) = match axis {
        MotionSweepAxis::Xy => (x_excursion_mm, y_excursion_mm),
        MotionSweepAxis::X => (x_excursion_mm, 0.0),
        MotionSweepAxis::Y => (0.0, y_excursion_mm),
    };
    let low = Pose {
        x_mm: seed.x_mm - x_excursion_mm,
        y_mm: seed.y_mm - y_excursion_mm,
        ..seed
    };
    let high = Pose {
        x_mm: seed.x_mm + x_excursion_mm,
        y_mm: seed.y_mm + y_excursion_mm,
        ..seed
    };
    if !profile.bounds.contains(low) || !profile.bounds.contains(high) {
        bail!("motion-sweep points are outside the active machine bounds");
    }
    Ok(MotionSweepPlan {
        axis,
        feeds,
        seed,
        low,
        high,
    })
}

async fn timed_jog(
    client: &Client,
    base: &str,
    lease: Uuid,
    from: Pose,
    target: Pose,
    feed_mm_per_min: f64,
) -> Result<Value> {
    let event_marker = newest_event_sequence(client, base).await?;
    let started = Instant::now();
    let operation = post(
        client,
        &format!("{base}/machine/jog"),
        Some(json!({
            "lease_id": lease,
            "target": JogTarget {
                x_mm: target.x_mm,
                y_mm: target.y_mm,
                z_mm: target.z_mm,
                rotation_deg: None,
            },
            "feed_mm_per_min": feed_mm_per_min,
        })),
    )
    .await?;
    run_operation(client, base, operation).await?;
    let client_elapsed_ms = started.elapsed().as_secs_f64() * 1_000.0;
    let telemetry =
        wait_for_motion_telemetry(client, base, event_marker, target, feed_mm_per_min).await?;
    let linear_distance_mm = (target.x_mm - from.x_mm)
        .hypot(target.y_mm - from.y_mm)
        .hypot(target.z_mm - from.z_mm);
    Ok(json!({
        "from": from,
        "target": target,
        "feed_mm_per_min": feed_mm_per_min,
        "linear_distance_mm": linear_distance_mm,
        "ideal_constant_speed_ms": linear_distance_mm / feed_mm_per_min * 60_000.0,
        "client_elapsed_ms": client_elapsed_ms,
        "controller_wait_telemetry": telemetry,
    }))
}

async fn observe_visual_home(client: &Client, base: &str, lease: Uuid) -> Result<Value> {
    post(
        client,
        &format!("{base}/machine/visual-home"),
        Some(json!({
            "lease_id": lease,
            "apply_offset": false,
            "move_to_seed": false,
        })),
    )
    .await
}

fn visual_correction(observation: &Value) -> Result<[f64; 2]> {
    let initial = observation
        .get("initial")
        .context("visual observation did not contain an initial result")?;
    let x = initial["correction_x_mm"]
        .as_f64()
        .context("visual observation did not contain correction_x_mm")?;
    let y = initial["correction_y_mm"]
        .as_f64()
        .context("visual observation did not contain correction_y_mm")?;
    if !x.is_finite() || !y.is_finite() {
        bail!("visual observation returned a non-finite correction");
    }
    Ok([x, y])
}

fn mean_correction(corrections: &[[f64; 2]]) -> [f64; 2] {
    let count = corrections.len() as f64;
    [
        corrections.iter().map(|value| value[0]).sum::<f64>() / count,
        corrections.iter().map(|value| value[1]).sum::<f64>() / count,
    ]
}

fn max_pairwise_correction_error(corrections: &[[f64; 2]]) -> f64 {
    let mut maximum = 0.0_f64;
    for (index, left) in corrections.iter().enumerate() {
        for right in &corrections[index + 1..] {
            maximum = maximum.max((left[0] - right[0]).hypot(left[1] - right[1]));
        }
    }
    maximum
}

fn visual_observation_valid(observation: &Value) -> bool {
    observation["initial"]["within_capture_bound"]
        .as_bool()
        .unwrap_or(false)
        && observation["initial"]["within_correction_bound"]
            .as_bool()
            .unwrap_or(false)
}

async fn newest_event_sequence(client: &Client, base: &str) -> Result<Option<u64>> {
    let recent = get(client, &format!("{base}/events/recent")).await?;
    Ok(newest_persisted_event_sequence(&recent))
}

fn newest_persisted_event_sequence(recent: &Value) -> Option<u64> {
    recent["events"]
        .as_array()
        .and_then(|events| events.last())
        .and_then(|event| event["sequence"].as_u64())
}

async fn wait_for_motion_telemetry(
    client: &Client,
    base: &str,
    event_marker: Option<u64>,
    target: Pose,
    feed_mm_per_min: f64,
) -> Result<Value> {
    for _ in 0..20 {
        let recent = get(client, &format!("{base}/events/recent")).await?;
        if let Some(event) = recent["events"].as_array().and_then(|events| {
            events.iter().find(|event| {
                event["kind"] == "machine.motion_completed"
                    && event["sequence"]
                        .as_u64()
                        .is_some_and(|sequence| event_marker.is_none_or(|marker| sequence > marker))
                    && event["payload"]["feed_mm_per_min"]
                        .as_f64()
                        .is_some_and(|feed| (feed - feed_mm_per_min).abs() < 1e-6)
                    && event["payload"]["pose"]["x_mm"]
                        .as_f64()
                        .is_some_and(|x| (x - target.x_mm).abs() < 1e-6)
                    && event["payload"]["pose"]["y_mm"]
                        .as_f64()
                        .is_some_and(|y| (y - target.y_mm).abs() < 1e-6)
                    && event["payload"]["pose"]["z_mm"]
                        .as_f64()
                        .is_some_and(|z| (z - target.z_mm).abs() < 1e-6)
            })
        }) {
            return Ok(event["payload"].clone());
        }
        tokio::time::sleep(Duration::from_millis(25)).await;
    }
    bail!("motion completed, but its timing event was not persisted");
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

        let registration = Uuid::new_v4();
        let args = Args::try_parse_from([
            "velocitypnp",
            "measure-board-surface",
            &registration.to_string(),
            "--lease",
            &lease.to_string(),
            "--confirm-safety-checks",
        ])
        .unwrap();
        assert!(matches!(
            args.command,
            Command::MeasureBoardSurface {
                registration: parsed_registration,
                lease: parsed_lease,
                ..
            } if parsed_registration == registration && parsed_lease == lease
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

        let evidence = Uuid::new_v4();
        let source = format!("{evidence}:2");
        let args = Args::try_parse_from([
            "velocitypnp",
            "build-strip-feeder-inventory",
            &scan.to_string(),
            "--source",
            &source,
        ])
        .unwrap();
        assert!(matches!(
            args.command,
            Command::BuildStripFeederInventory {
                scan: parsed_scan,
                sources,
            } if parsed_scan == scan && sources == vec![source]
        ));

        let args = Args::try_parse_from([
            "velocitypnp",
            "promote-strip-feeder-inventory",
            &scan.to_string(),
        ])
        .unwrap();
        assert!(matches!(
            args.command,
            Command::PromoteStripFeederInventory { scan: parsed } if parsed == scan
        ));

        let lease = Uuid::new_v4();
        let args = Args::try_parse_from([
            "velocitypnp",
            "capture-strip-feeder-sources",
            &scan.to_string(),
            "--lease",
            &lease.to_string(),
            "--occupied",
            "8",
            "--empty",
            "6",
            "--lane",
            "0",
            "--lane",
            "2",
            "--confirm-safety-checks",
        ])
        .unwrap();
        assert!(matches!(
            args.command,
            Command::CaptureStripFeederSources {
                scan: parsed_scan,
                lease: parsed_lease,
                occupied: 8,
                empty: 6,
                lanes,
                confirm_safety_checks: true,
            } if parsed_scan == scan && parsed_lease == lease && lanes == vec![0, 2]
        ));
    }

    #[test]
    fn motion_sweep_uses_the_bounded_default_escalation() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.2.json")).unwrap();
        let mut status = MachineStatus {
            state: MachineState::Ready,
            simulated: false,
            profile_id: profile.id.clone(),
            pose: Some(profile.home_pose),
            homed: true,
            ..MachineStatus::default()
        };
        let mut controller = velocitypnp_domain::ControllerConnection::default();
        controller
            .settings
            .max_feed_units_per_second
            .insert("X".into(), 500.0);
        controller
            .settings
            .max_feed_units_per_second
            .insert("Y".into(), 500.0);
        status.controller = Some(controller);

        let plan = motion_sweep_plan(
            &status,
            &profile,
            Vec::new(),
            MotionSweepAxis::Xy,
            150.0,
            100.0,
            0.05,
        )
        .expect("default sweep plan");

        assert_eq!(plan.feeds, DEFAULT_MOTION_SWEEP_FEEDS);
        assert!(profile.bounds.contains(plan.low));
        assert!(profile.bounds.contains(plan.high));
        assert_eq!(plan.seed.z_mm, profile.safe_z_mm);
    }

    #[test]
    fn motion_sweep_can_isolate_each_linear_axis() {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.2.json")).unwrap();
        let mut status = MachineStatus {
            state: MachineState::Ready,
            simulated: false,
            profile_id: profile.id.clone(),
            pose: Some(profile.home_pose),
            homed: true,
            ..MachineStatus::default()
        };
        let mut controller = velocitypnp_domain::ControllerConnection::default();
        controller
            .settings
            .max_feed_units_per_second
            .insert("X".into(), 500.0);
        controller
            .settings
            .max_feed_units_per_second
            .insert("Y".into(), 500.0);
        status.controller = Some(controller);

        let x = motion_sweep_plan(
            &status,
            &profile,
            vec![12_000.0],
            MotionSweepAxis::X,
            150.0,
            100.0,
            0.05,
        )
        .expect("X-only sweep plan");
        let y = motion_sweep_plan(
            &status,
            &profile,
            vec![12_000.0],
            MotionSweepAxis::Y,
            150.0,
            100.0,
            0.05,
        )
        .expect("Y-only sweep plan");

        assert_eq!(x.low.y_mm, x.seed.y_mm);
        assert_eq!(x.high.y_mm, x.seed.y_mm);
        assert_eq!(y.low.x_mm, y.seed.x_mm);
        assert_eq!(y.high.x_mm, y.seed.x_mm);
    }

    #[test]
    fn motion_sweep_cli_accepts_multiple_feed_levels() {
        let lease = Uuid::new_v4();
        let args = Args::try_parse_from([
            "velocitypnp",
            "motion-sweep",
            "--lease",
            &lease.to_string(),
            "--feed",
            "1200",
            "6000",
            "12000",
            "--axis",
            "y",
            "--confirm-safety-checks",
        ])
        .unwrap();

        assert!(matches!(
            args.command,
            Command::MotionSweep {
                lease: parsed_lease,
                feeds,
                axis: MotionSweepAxis::Y,
                confirm_safety_checks: true,
                ..
            } if parsed_lease == lease && feeds == vec![1_200.0, 6_000.0, 12_000.0]
        ));
    }

    #[test]
    fn pickup_benchmark_cli_accepts_five_cycles() {
        let lease = Uuid::new_v4();
        let registration = Uuid::new_v4();
        let args = Args::try_parse_from([
            "velocitypnp",
            "qualify-strip-feeder-pickup",
            &registration.to_string(),
            "--lease",
            &lease.to_string(),
            "--pocket",
            "10",
            "--cycles",
            "5",
            "--confirm-safety-checks",
        ])
        .unwrap();

        assert!(matches!(
            args.command,
            Command::QualifyStripFeederPickup {
                registration: parsed_registration,
                lease: parsed_lease,
                pocket: Some(10),
                cycles: 5,
                confirm_safety_checks: true,
                ..
            } if parsed_registration == registration && parsed_lease == lease
        ));
    }

    #[test]
    fn camera_settling_cli_accepts_a_cold_capture_plan() {
        let lease = Uuid::new_v4();
        let args = Args::try_parse_from([
            "velocitypnp",
            "camera-settling",
            "bottom_camera",
            "--lease",
            &lease.to_string(),
            "--samples",
            "80",
            "--repetitions",
            "5",
        ])
        .unwrap();

        assert!(matches!(
            args.command,
            Command::CameraSettling {
                role,
                lease: parsed_lease,
                samples: 80,
                repetitions: 5,
            } if role == "bottom_camera" && parsed_lease == lease
        ));
    }

    #[test]
    fn timing_marker_uses_the_last_persisted_event_after_a_daemon_restart() {
        let recent = json!({
            "events": [
                {"sequence": 740, "kind": "old.session"},
                {"sequence": 7, "kind": "current.session"},
            ]
        });

        assert_eq!(newest_persisted_event_sequence(&recent), Some(7));
    }

    #[test]
    fn baseline_summary_measures_stationary_camera_spread() {
        let corrections = [[0.01, 0.02], [0.03, 0.02], [0.02, 0.02]];

        assert_eq!(mean_correction(&corrections), [0.02, 0.02]);
        assert!((max_pairwise_correction_error(&corrections) - 0.02).abs() < 1e-12);
    }
}
