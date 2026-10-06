use anyhow::{Context, Result};
use chrono::Utc;
use parking_lot::Mutex;
use rusqlite::{params, Connection, OptionalExtension};
use serde::{de::DeserializeOwned, Serialize};
use std::path::Path;
use std::sync::Arc;
use uuid::Uuid;
use velocitypnp_domain::{
    AutomatedCalibrationRun, BoardDesign, BoardPopulationRefreshReport, BoardRegistrationRun,
    BoardSurfaceMeasurementRun, CalibrationSnapshot, CameraProfile, CameraRole, Feeder,
    InspectionReport, Job, JobExecutionRun, MachineEvent, Operation, Part, PlacementExecutionRun,
    PlacementOutcomeRecord, StripFeederLiveHoldoutRun, StripFeederOccupancyRefreshReport,
    StripFeederPickupQualificationRun, StripFeederPocketOutcome, StripFeederPocketScanObservation,
    StripFeederPocketState, StripFeederPocketStatus, StripFeederRegistration,
    StripFeederSourceCaptureRun, StripPocketOccupancy, WorkspaceDisplayPatch,
    WorkspaceDisplayPatchKind, WorkspaceInventoryBaseline, WorkspaceScanReport,
};

#[derive(Clone)]
pub struct Store {
    connection: Arc<Mutex<Connection>>,
}

impl Store {
    pub fn open(path: &Path) -> Result<Self> {
        let connection = Connection::open(path)
            .with_context(|| format!("opening database {}", path.display()))?;
        let store = Self {
            connection: Arc::new(Mutex::new(connection)),
        };
        store.migrate()?;
        Ok(store)
    }

    pub fn memory() -> Result<Self> {
        let store = Self {
            connection: Arc::new(Mutex::new(Connection::open_in_memory()?)),
        };
        store.migrate()?;
        Ok(store)
    }

    fn migrate(&self) -> Result<()> {
        self.connection.lock().execute_batch(
            r#"
            PRAGMA journal_mode = WAL;
            PRAGMA foreign_keys = ON;
            CREATE TABLE IF NOT EXISTS schema_version (
                version INTEGER PRIMARY KEY,
                applied_at TEXT NOT NULL
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (1, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS entities (
                kind TEXT NOT NULL,
                id TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                document TEXT NOT NULL,
                PRIMARY KEY(kind, id)
            );
            CREATE TABLE IF NOT EXISTS event_journal (
                row_id INTEGER PRIMARY KEY AUTOINCREMENT,
                machine_sequence INTEGER NOT NULL,
                occurred_at TEXT NOT NULL,
                kind TEXT NOT NULL,
                payload TEXT NOT NULL
            );
            CREATE INDEX IF NOT EXISTS event_journal_occurred_at
                ON event_journal(occurred_at);
            CREATE TABLE IF NOT EXISTS calibration_artifacts (
                run_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(run_id, name)
            );
            CREATE TABLE IF NOT EXISTS workspace_scan_artifacts (
                scan_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(scan_id, name)
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (2, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS board_registration_artifacts (
                registration_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(registration_id, name)
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (3, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS strip_feeder_holdout_artifacts (
                holdout_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(holdout_id, name)
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS strip_feeder_pickup_artifacts (
                qualification_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(qualification_id, name)
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (5, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS board_surface_measurement_artifacts (
                measurement_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(measurement_id, name)
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (6, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS placement_execution_artifacts (
                execution_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(execution_id, name)
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (7, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            CREATE TABLE IF NOT EXISTS strip_feeder_source_capture_artifacts (
                capture_id TEXT NOT NULL,
                name TEXT NOT NULL,
                media_type TEXT NOT NULL,
                data BLOB NOT NULL,
                created_at TEXT NOT NULL,
                PRIMARY KEY(capture_id, name)
            );
            INSERT OR IGNORE INTO schema_version(version, applied_at)
                VALUES (8, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'));
            "#,
        )?;
        Ok(())
    }

    pub fn put_board(&self, board: &BoardDesign) -> Result<()> {
        self.put("board", board.id, board)
    }

    pub fn list_boards(&self) -> Result<Vec<BoardDesign>> {
        self.list("board")
    }

    pub fn get_board(&self, id: Uuid) -> Result<Option<BoardDesign>> {
        self.get("board", id)
    }

    pub fn put_board_registration(&self, registration: &BoardRegistrationRun) -> Result<()> {
        self.put("board_registration", registration.id, registration)
    }

    pub fn list_board_registrations(&self) -> Result<Vec<BoardRegistrationRun>> {
        self.list("board_registration")
    }

    pub fn get_board_registration(&self, id: Uuid) -> Result<Option<BoardRegistrationRun>> {
        self.get("board_registration", id)
    }

    pub fn put_board_surface_measurement(
        &self,
        measurement: &BoardSurfaceMeasurementRun,
    ) -> Result<()> {
        self.put("board_surface_measurement", measurement.id, measurement)
    }

    pub fn list_board_surface_measurements(&self) -> Result<Vec<BoardSurfaceMeasurementRun>> {
        self.list("board_surface_measurement")
    }

    pub fn get_board_surface_measurement(
        &self,
        id: Uuid,
    ) -> Result<Option<BoardSurfaceMeasurementRun>> {
        self.get("board_surface_measurement", id)
    }

    pub fn put_board_surface_measurement_artifact(
        &self,
        measurement_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(!name.is_empty(), "board surface artifact name is empty");
        anyhow::ensure!(
            !media_type.is_empty(),
            "board surface artifact media type is empty"
        );
        self.connection.lock().execute(
            r#"INSERT INTO board_surface_measurement_artifacts(
                 measurement_id, name, media_type, data, created_at
               ) VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(measurement_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![measurement_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_board_surface_measurement_artifact(
        &self,
        measurement_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM board_surface_measurement_artifacts \
                 WHERE measurement_id = ?1 AND name = ?2",
                params![measurement_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    pub fn put_placement_execution_artifact(
        &self,
        execution_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(!name.is_empty(), "placement artifact name is empty");
        anyhow::ensure!(!media_type.is_empty(), "placement media type is empty");
        self.connection.lock().execute(
            r#"INSERT INTO placement_execution_artifacts(
                 execution_id, name, media_type, data, created_at
               ) VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(execution_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![execution_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_placement_execution_artifact(
        &self,
        execution_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM placement_execution_artifacts \
                 WHERE execution_id = ?1 AND name = ?2",
                params![execution_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    /// Lists retained artifact names and media types without loading blobs.
    pub fn list_placement_execution_artifacts(
        &self,
        execution_id: Uuid,
    ) -> Result<Vec<(String, String)>> {
        self.list_artifact_metadata(
            "placement_execution_artifacts",
            "execution_id",
            execution_id,
        )
    }

    pub fn put_board_registration_artifact(
        &self,
        registration_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(
            !name.is_empty(),
            "board registration artifact name is empty"
        );
        anyhow::ensure!(
            !media_type.is_empty(),
            "board registration artifact media type is empty"
        );
        self.connection.lock().execute(
            r#"INSERT INTO board_registration_artifacts(
                 registration_id, name, media_type, data, created_at
               ) VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(registration_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![registration_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_board_registration_artifact(
        &self,
        registration_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM board_registration_artifacts \
                 WHERE registration_id = ?1 AND name = ?2",
                params![registration_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    pub fn put_operation(&self, operation: &Operation) -> Result<()> {
        self.put("operation", operation.id, operation)
    }

    pub fn get_operation(&self, id: Uuid) -> Result<Option<Operation>> {
        self.get("operation", id)
    }

    pub fn put_calibration(&self, calibration: &CalibrationSnapshot) -> Result<()> {
        self.put("calibration", calibration.id, calibration)
    }

    pub fn list_calibrations(&self) -> Result<Vec<CalibrationSnapshot>> {
        self.list("calibration")
    }

    pub fn get_calibration(&self, id: Uuid) -> Result<Option<CalibrationSnapshot>> {
        self.get("calibration", id)
    }

    pub fn put_calibration_run(&self, run: &AutomatedCalibrationRun) -> Result<()> {
        self.put("calibration_run", run.id, run)
    }

    pub fn list_calibration_runs(&self) -> Result<Vec<AutomatedCalibrationRun>> {
        self.list("calibration_run")
    }

    pub fn get_calibration_run(&self, id: Uuid) -> Result<Option<AutomatedCalibrationRun>> {
        self.get("calibration_run", id)
    }

    pub fn put_calibration_artifact(
        &self,
        run_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(!name.is_empty(), "calibration artifact name is empty");
        anyhow::ensure!(
            !media_type.is_empty(),
            "calibration artifact media type is empty"
        );
        self.connection.lock().execute(
            r#"INSERT INTO calibration_artifacts(run_id, name, media_type, data, created_at)
               VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(run_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![run_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_calibration_artifact(
        &self,
        run_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM calibration_artifacts WHERE run_id = ?1 AND name = ?2",
                params![run_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    pub fn put_workspace_scan(&self, scan: &WorkspaceScanReport) -> Result<()> {
        self.put("workspace_scan", scan.id, scan)
    }

    pub fn list_workspace_scans(&self) -> Result<Vec<WorkspaceScanReport>> {
        self.list("workspace_scan")
    }

    pub fn get_workspace_scan(&self, id: Uuid) -> Result<Option<WorkspaceScanReport>> {
        self.get("workspace_scan", id)
    }

    pub fn put_workspace_display_patch(&self, patch: &WorkspaceDisplayPatch) -> Result<()> {
        self.put("workspace_display_patch", patch.id, patch)
    }

    pub fn list_workspace_display_patches(&self) -> Result<Vec<WorkspaceDisplayPatch>> {
        self.list("workspace_display_patch")
    }

    pub fn put_strip_feeder_occupancy_refresh(
        &self,
        report: &StripFeederOccupancyRefreshReport,
    ) -> Result<()> {
        self.put_named(
            "strip_feeder_occupancy_refresh",
            &format!("{}:{}", report.source_scan_id, report.registration_id),
            report,
        )
    }

    pub fn list_strip_feeder_occupancy_refreshes(
        &self,
    ) -> Result<Vec<StripFeederOccupancyRefreshReport>> {
        self.list("strip_feeder_occupancy_refresh")
    }

    pub fn put_board_population_refresh(
        &self,
        report: &BoardPopulationRefreshReport,
    ) -> Result<()> {
        self.put_named(
            "board_population_refresh",
            &format!(
                "{}:{}:{}",
                report.source_scan_id, report.job_id, report.board_instance_id,
            ),
            report,
        )
    }

    pub fn list_board_population_refreshes(&self) -> Result<Vec<BoardPopulationRefreshReport>> {
        self.list("board_population_refresh")
    }

    pub fn put_workspace_inventory_baseline(
        &self,
        baseline: &WorkspaceInventoryBaseline,
    ) -> Result<()> {
        self.put("workspace_inventory_baseline", baseline.id, baseline)
    }

    pub fn list_workspace_inventory_baselines(&self) -> Result<Vec<WorkspaceInventoryBaseline>> {
        self.list("workspace_inventory_baseline")
    }

    pub fn get_workspace_inventory_baseline(
        &self,
        id: Uuid,
    ) -> Result<Option<WorkspaceInventoryBaseline>> {
        self.get("workspace_inventory_baseline", id)
    }

    pub fn put_workspace_scan_artifact(
        &self,
        scan_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(!name.is_empty(), "workspace scan artifact name is empty");
        anyhow::ensure!(
            !media_type.is_empty(),
            "workspace scan artifact media type is empty"
        );
        self.connection.lock().execute(
            r#"INSERT INTO workspace_scan_artifacts(scan_id, name, media_type, data, created_at)
               VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(scan_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![scan_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_workspace_scan_artifact(
        &self,
        scan_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM workspace_scan_artifacts WHERE scan_id = ?1 AND name = ?2",
                params![scan_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    /// Lists retained workspace artifact names without loading image data.
    pub fn list_workspace_scan_artifacts(&self, scan_id: Uuid) -> Result<Vec<(String, String)>> {
        self.list_artifact_metadata("workspace_scan_artifacts", "scan_id", scan_id)
    }

    pub fn workspace_scan_artifact_bytes_except(
        &self,
        scan_id: Uuid,
        excluded_name: &str,
    ) -> Result<u64> {
        let bytes: i64 = self.connection.lock().query_row(
            "SELECT COALESCE(SUM(length(data)), 0) FROM workspace_scan_artifacts \
             WHERE scan_id = ?1 AND name <> ?2",
            params![scan_id.to_string(), excluded_name],
            |row| row.get(0),
        )?;
        u64::try_from(bytes).context("workspace artifact byte count is negative")
    }

    pub fn put_camera_profile(&self, profile: &CameraProfile) -> Result<()> {
        self.put_named("camera_profile", &profile.storage_id(), profile)
    }

    pub fn get_camera_profile(
        &self,
        machine_profile_id: &str,
        role: CameraRole,
    ) -> Result<Option<CameraProfile>> {
        self.get_named(
            "camera_profile",
            &format!("{machine_profile_id}:{}", role.as_str()),
        )
    }

    pub fn put_feeder(&self, feeder: &Feeder) -> Result<()> {
        self.put("feeder", feeder_id(feeder), feeder)
    }

    pub fn put_part(&self, part: &Part) -> Result<()> {
        self.put("part", part.id, part)
    }

    pub fn list_parts(&self) -> Result<Vec<Part>> {
        self.list("part")
    }

    pub fn get_part(&self, id: Uuid) -> Result<Option<Part>> {
        self.get("part", id)
    }

    pub fn list_feeders(&self) -> Result<Vec<Feeder>> {
        self.list("feeder")
    }

    pub fn delete_feeder(&self, id: Uuid) -> Result<()> {
        self.connection.lock().execute(
            "DELETE FROM entities WHERE kind = ?1 AND id = ?2",
            params!["feeder", id.to_string()],
        )?;
        Ok(())
    }

    pub fn put_strip_feeder_registration(
        &self,
        registration: &StripFeederRegistration,
    ) -> Result<()> {
        self.put("strip_feeder_registration", registration.id, registration)
    }

    pub fn list_strip_feeder_registrations(&self) -> Result<Vec<StripFeederRegistration>> {
        let mut registrations =
            self.list::<StripFeederRegistration>("strip_feeder_registration")?;
        for registration in &mut registrations {
            registration.ensure_pocket_state();
        }
        Ok(registrations)
    }

    pub fn get_strip_feeder_registration(
        &self,
        id: Uuid,
    ) -> Result<Option<StripFeederRegistration>> {
        let mut registration =
            self.get::<StripFeederRegistration>("strip_feeder_registration", id)?;
        if let Some(registration) = &mut registration {
            registration.ensure_pocket_state();
        }
        Ok(registration)
    }

    /// Atomically applies a completed lane rescan to operational pocket state.
    ///
    /// An active reservation blocks the refresh so that a scan cannot change
    /// state while a pickup owns a pocket.
    pub fn apply_strip_feeder_occupancy_refresh(
        &self,
        registration_id: Uuid,
        observations: &[StripFeederPocketScanObservation],
        observed_at: chrono::DateTime<Utc>,
    ) -> Result<StripFeederRegistration> {
        let mut connection = self.connection.lock();
        let transaction = connection.transaction()?;
        let document: String = transaction
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params!["strip_feeder_registration", registration_id.to_string()],
                |row| row.get(0),
            )
            .context("strip feeder registration not found")?;
        let mut registration: StripFeederRegistration =
            serde_json::from_str(&document).context("decoding strip feeder registration")?;
        registration.ensure_pocket_state();
        anyhow::ensure!(
            registration.pockets.iter().all(|pocket| {
                pocket.status != StripFeederPocketStatus::Reserved
                    && pocket.reservation_id.is_none()
            }),
            "strip feeder occupancy cannot refresh while a pocket is reserved"
        );
        for observation in observations {
            let Some(pocket) = registration
                .pockets
                .iter_mut()
                .find(|pocket| pocket.pocket_index == observation.pocket_index)
            else {
                continue;
            };
            pocket.measured_center_machine_mm = Some(observation.center_machine_mm);
            pocket.occupancy = observation.occupancy;
            pocket.occupancy_confidence = observation.confidence;
            pocket.status = if observation.review_required {
                StripFeederPocketStatus::NeedsReview
            } else {
                match observation.occupancy {
                    StripPocketOccupancy::Occupied => StripFeederPocketStatus::Available,
                    StripPocketOccupancy::Empty => StripFeederPocketStatus::Empty,
                    StripPocketOccupancy::Uncertain => StripFeederPocketStatus::NeedsReview,
                }
            };
            pocket.updated_at = observed_at;
        }
        if let Some(selected_index) = registration.selected_pocket_index {
            let selected = registration
                .pockets
                .iter()
                .find(|pocket| pocket.pocket_index == selected_index)
                .filter(|pocket| pocket.status == StripFeederPocketStatus::Available)
                .map(|pocket| {
                    (
                        pocket.pocket_index,
                        pocket
                            .measured_center_machine_mm
                            .unwrap_or(pocket.nominal_center_machine_mm),
                    )
                })
                .or_else(|| {
                    registration.next_available_pocket(None).map(|pocket| {
                        (
                            pocket.pocket_index,
                            pocket
                                .measured_center_machine_mm
                                .unwrap_or(pocket.nominal_center_machine_mm),
                        )
                    })
                });
            registration.selected_pocket_index = selected.map(|item| item.0);
            registration.selected_pocket_machine_mm = selected.map(|item| item.1);
        }
        registration.updated_at = observed_at;
        transaction.execute(
            r#"UPDATE entities
               SET updated_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), document = ?3
               WHERE kind = ?1 AND id = ?2"#,
            params![
                "strip_feeder_registration",
                registration_id.to_string(),
                serde_json::to_string(&registration)?,
            ],
        )?;
        transaction.commit()?;
        Ok(registration)
    }

    /// Atomically promotes one complete feeder-lane revision.
    ///
    /// The active registration, occupancy report, and displayed mosaic patch
    /// change in one transaction. Existing pocket history remains in prior
    /// reports, while the new physical tape load receives new pocket state.
    pub fn promote_strip_feeder_lane_revision(
        &self,
        report: &StripFeederOccupancyRefreshReport,
        patch: &WorkspaceDisplayPatch,
    ) -> Result<StripFeederRegistration> {
        let revision = report
            .lane_revision
            .as_ref()
            .context("feeder lane promotion has no revision")?;
        anyhow::ensure!(
            !report.observations.is_empty(),
            "feeder lane revision has no pocket observations"
        );
        anyhow::ensure!(
            patch.id == revision.display_patch_id
                && patch.base_scan_id == revision.base_scan_id
                && patch.source_scan_id == report.source_scan_id
                && patch.profile_id == revision.profile_id
                && patch.calibration_id == revision.calibration_id
                && patch.object_id == report.registration_id
                && patch.kind == WorkspaceDisplayPatchKind::FeederLane
                && patch.artifact_name == revision.mosaic_artifact_name
                && patch.source_raster_transform == revision.raster_transform
                && patch.source_width_px == revision.mosaic_width_px
                && patch.source_height_px == revision.mosaic_height_px
                && patch.source_to_base == velocitypnp_domain::Transform2d::identity()
                && patch.anchor_count == report.observations.len() as u32
                && patch.alignment_residuals == revision.geometry_residuals
                && patch.accepted,
            "feeder lane revision and display patch do not match"
        );
        anyhow::ensure!(
            report
                .observations
                .iter()
                .enumerate()
                .all(|(index, observation)| u32::try_from(index)
                    .is_ok_and(|index| observation.pocket_index == index)),
            "feeder lane revision pocket indices are not contiguous"
        );
        anyhow::ensure!(
            revision.tape_width_mm.is_finite()
                && revision.tape_width_mm > 0.0
                && revision.tape_pitch_mm.is_finite()
                && revision.tape_pitch_mm > 0.0
                && revision.tape_rotation_deg.is_finite()
                && revision
                    .pick_centerline_machine_mm
                    .iter()
                    .flatten()
                    .all(|value| value.is_finite())
                && report.observations.iter().all(|observation| {
                    observation
                        .center_machine_mm
                        .iter()
                        .all(|value| value.is_finite())
                        && observation.confidence.is_finite()
                        && (0.0..=1.0).contains(&observation.confidence)
                }),
            "feeder lane revision contains invalid geometry or confidence"
        );

        let mut connection = self.connection.lock();
        let transaction = connection.transaction()?;
        let document: String = transaction
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params![
                    "strip_feeder_registration",
                    report.registration_id.to_string()
                ],
                |row| row.get(0),
            )
            .context("strip feeder registration not found")?;
        let mut registration: StripFeederRegistration =
            serde_json::from_str(&document).context("decoding strip feeder registration")?;
        registration.ensure_pocket_state();
        anyhow::ensure!(
            registration.profile_id == revision.profile_id
                && registration.calibration_id == revision.calibration_id,
            "feeder lane revision uses different machine geometry"
        );
        anyhow::ensure!(
            registration.pockets.iter().all(|pocket| {
                pocket.status != StripFeederPocketStatus::Reserved
                    && pocket.reservation_id.is_none()
            }),
            "strip feeder lane cannot change while a pocket is reserved"
        );

        registration.source_scan_id = report.source_scan_id;
        registration.source_candidate_index = report.source_candidate_index;
        registration.source_proposal_index = report.source_proposal_index;
        registration.updated_at = report.created_at;
        registration.tape_width_mm = revision.tape_width_mm;
        registration.tape_pitch_mm = revision.tape_pitch_mm;
        registration.tape_rotation_deg = revision.tape_rotation_deg;
        registration.pick_centerline_machine_mm = revision.pick_centerline_machine_mm;
        registration.pocket_centers_machine_mm = report
            .observations
            .iter()
            .map(|observation| observation.center_machine_mm)
            .collect();
        registration.pockets = report
            .observations
            .iter()
            .map(|observation| StripFeederPocketState {
                pocket_index: observation.pocket_index,
                nominal_center_machine_mm: observation.center_machine_mm,
                measured_center_machine_mm: None,
                occupancy: observation.occupancy,
                occupancy_confidence: observation.confidence,
                status: if observation.review_required {
                    StripFeederPocketStatus::NeedsReview
                } else {
                    match observation.occupancy {
                        StripPocketOccupancy::Occupied => StripFeederPocketStatus::Available,
                        StripPocketOccupancy::Empty => StripFeederPocketStatus::Empty,
                        StripPocketOccupancy::Uncertain => StripFeederPocketStatus::NeedsReview,
                    }
                },
                attempt_count: 0,
                consecutive_failure_count: 0,
                reservation_id: None,
                last_qualification_id: None,
                last_outcome: None,
                updated_at: report.created_at,
            })
            .collect();
        let selected = registration.next_available_pocket(None).map(|pocket| {
            (
                pocket.pocket_index,
                pocket
                    .measured_center_machine_mm
                    .unwrap_or(pocket.nominal_center_machine_mm),
            )
        });
        registration.selected_pocket_index = selected.map(|item| item.0);
        registration.selected_pocket_machine_mm = selected.map(|item| item.1);
        registration.vision_confidence = revision.vision_confidence;
        registration.vision_review_required = report
            .observations
            .iter()
            .any(|observation| observation.review_required);

        let active_feeder = registration
            .active_feeder_id
            .map(|feeder_id| -> Result<Feeder> {
                let document: String = transaction
                    .query_row(
                        "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                        params!["feeder", feeder_id.to_string()],
                        |row| row.get(0),
                    )
                    .context("active strip feeder not found")?;
                let mut feeder: Feeder =
                    serde_json::from_str(&document).context("decoding active strip feeder")?;
                let Feeder::Strip {
                    first_pick,
                    tape_pitch_mm,
                    tape_width_mm,
                    tape_rotation_deg,
                    remaining,
                    ..
                } = &mut feeder
                else {
                    anyhow::bail!("active feeder is not a strip feeder");
                };
                if let Some(center) = registration.selected_pocket_machine_mm {
                    first_pick.x_mm = center[0];
                    first_pick.y_mm = center[1];
                }
                if let Some(pickup_z_mm) = registration.pickup_z_mm {
                    first_pick.z_mm = pickup_z_mm;
                }
                if let Some(part_rotation_deg) = registration.part_rotation_deg {
                    first_pick.rotation_deg = part_rotation_deg;
                }
                *tape_pitch_mm = registration.tape_pitch_mm;
                *tape_width_mm = registration.tape_width_mm;
                *tape_rotation_deg = registration.tape_rotation_deg;
                *remaining = registration
                    .pockets
                    .iter()
                    .filter(|pocket| {
                        pocket.status == StripFeederPocketStatus::Available
                            && pocket.occupancy == StripPocketOccupancy::Occupied
                    })
                    .count()
                    .try_into()
                    .ok();
                Ok(feeder)
            })
            .transpose()?;

        let stale_documents = {
            let mut statement = transaction
                .prepare("SELECT id, document FROM entities WHERE kind = ?1 AND id != ?2")?;
            let rows = statement.query_map(
                params!["workspace_display_patch", patch.id.to_string()],
                |row| Ok((row.get::<_, String>(0)?, row.get::<_, String>(1)?)),
            )?;
            rows.collect::<rusqlite::Result<Vec<_>>>()?
        };
        for (id, document) in stale_documents {
            let Ok(mut stale) = serde_json::from_str::<WorkspaceDisplayPatch>(&document) else {
                continue;
            };
            if stale.kind != WorkspaceDisplayPatchKind::FeederLane
                || stale.object_id != patch.object_id
                || !stale.accepted
            {
                continue;
            }
            stale.accepted = false;
            stale.rejection_reason = Some("Superseded by a newer feeder lane revision".into());
            stale.evaluated_at = report.created_at;
            transaction.execute(
                "UPDATE entities SET updated_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), \
                 document = ?3 WHERE kind = ?1 AND id = ?2",
                params![
                    "workspace_display_patch",
                    id,
                    serde_json::to_string(&stale)?
                ],
            )?;
        }
        for (kind, id, document) in [
            (
                "strip_feeder_registration",
                registration.id.to_string(),
                serde_json::to_string(&registration)?,
            ),
            (
                "strip_feeder_occupancy_refresh",
                format!("{}:{}", report.source_scan_id, report.registration_id),
                serde_json::to_string(report)?,
            ),
            (
                "workspace_display_patch",
                patch.id.to_string(),
                serde_json::to_string(patch)?,
            ),
        ] {
            transaction.execute(
                r#"INSERT INTO entities(kind, id, updated_at, document)
                   VALUES (?1, ?2, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), ?3)
                   ON CONFLICT(kind, id) DO UPDATE SET
                     updated_at = excluded.updated_at,
                     document = excluded.document"#,
                params![kind, id, document],
            )?;
        }
        if let Some(feeder) = active_feeder {
            transaction.execute(
                r#"UPDATE entities
                   SET updated_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), document = ?3
                   WHERE kind = ?1 AND id = ?2"#,
                params![
                    "feeder",
                    feeder_id(&feeder).to_string(),
                    serde_json::to_string(&feeder)?,
                ],
            )?;
        }
        transaction.commit()?;
        Ok(registration)
    }

    /// Atomically reserves one occupied pocket for a pickup operation.
    pub fn reserve_strip_feeder_pocket(
        &self,
        registration_id: Uuid,
        reservation_id: Uuid,
        preferred_index: Option<u32>,
    ) -> Result<StripFeederRegistration> {
        let mut connection = self.connection.lock();
        let transaction = connection.transaction()?;
        let document: String = transaction
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params!["strip_feeder_registration", registration_id.to_string()],
                |row| row.get(0),
            )
            .context("strip feeder registration not found")?;
        let mut registration: StripFeederRegistration =
            serde_json::from_str(&document).context("decoding strip feeder registration")?;
        registration.ensure_pocket_state();
        let pocket_index = registration
            .next_available_pocket(preferred_index)
            .map(|pocket| pocket.pocket_index)
            .context("strip feeder has no available occupied pocket")?;
        let pocket = registration
            .pockets
            .iter_mut()
            .find(|pocket| pocket.pocket_index == pocket_index)
            .context("selected strip feeder pocket is missing")?;
        pocket.status = StripFeederPocketStatus::Reserved;
        pocket.reservation_id = Some(reservation_id);
        pocket.updated_at = Utc::now();
        registration.selected_pocket_index = Some(pocket_index);
        registration.selected_pocket_machine_mm = pocket
            .measured_center_machine_mm
            .or(Some(pocket.nominal_center_machine_mm));
        registration.updated_at = Utc::now();
        transaction.execute(
            r#"UPDATE entities
               SET updated_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), document = ?3
               WHERE kind = ?1 AND id = ?2"#,
            params![
                "strip_feeder_registration",
                registration_id.to_string(),
                serde_json::to_string(&registration)?,
            ],
        )?;
        transaction.commit()?;
        Ok(registration)
    }

    /// Releases a reservation when execution stopped before pocket approach.
    /// This preserves the observed occupancy and does not record an attempt.
    pub fn release_strip_feeder_pocket_reservation(
        &self,
        registration_id: Uuid,
        reservation_id: Uuid,
    ) -> Result<StripFeederRegistration> {
        let mut connection = self.connection.lock();
        let transaction = connection.transaction()?;
        let document: String = transaction
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params!["strip_feeder_registration", registration_id.to_string()],
                |row| row.get(0),
            )
            .context("strip feeder registration not found")?;
        let mut registration: StripFeederRegistration =
            serde_json::from_str(&document).context("decoding strip feeder registration")?;
        registration.ensure_pocket_state();
        let pocket = registration
            .pockets
            .iter_mut()
            .find(|pocket| pocket.reservation_id == Some(reservation_id))
            .context("strip feeder pocket reservation is not current")?;
        anyhow::ensure!(
            pocket.status == StripFeederPocketStatus::Reserved,
            "strip feeder pocket reservation is not active"
        );
        pocket.status = StripFeederPocketStatus::Available;
        pocket.reservation_id = None;
        pocket.updated_at = Utc::now();
        registration.updated_at = Utc::now();
        transaction.execute(
            r#"UPDATE entities
               SET updated_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), document = ?3
               WHERE kind = ?1 AND id = ?2"#,
            params![
                "strip_feeder_registration",
                registration_id.to_string(),
                serde_json::to_string(&registration)?,
            ],
        )?;
        transaction.commit()?;
        Ok(registration)
    }

    /// Atomically settles a reservation. Consumption requires pickup and
    /// bottom-camera evidence from the caller.
    pub fn settle_strip_feeder_pocket(
        &self,
        registration_id: Uuid,
        reservation_id: Uuid,
        outcome: StripFeederPocketOutcome,
        pickup_evidence_passed: bool,
    ) -> Result<StripFeederRegistration> {
        if outcome == StripFeederPocketOutcome::Consumed && !pickup_evidence_passed {
            anyhow::bail!("strip feeder pocket consumption requires pickup evidence");
        }
        let mut connection = self.connection.lock();
        let transaction = connection.transaction()?;
        let document: String = transaction
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params!["strip_feeder_registration", registration_id.to_string()],
                |row| row.get(0),
            )
            .context("strip feeder registration not found")?;
        let mut registration: StripFeederRegistration =
            serde_json::from_str(&document).context("decoding strip feeder registration")?;
        registration.ensure_pocket_state();
        let pocket = registration
            .pockets
            .iter_mut()
            .find(|pocket| pocket.reservation_id == Some(reservation_id))
            .context("strip feeder pocket reservation is not current")?;
        pocket.attempt_count = pocket.attempt_count.saturating_add(1);
        pocket.last_qualification_id = Some(reservation_id);
        pocket.last_outcome = Some(outcome);
        pocket.reservation_id = None;
        pocket.updated_at = Utc::now();
        match outcome {
            StripFeederPocketOutcome::Returned => {
                pocket.consecutive_failure_count = 0;
                pocket.occupancy = StripPocketOccupancy::Occupied;
                pocket.status = StripFeederPocketStatus::Available;
            }
            StripFeederPocketOutcome::Consumed => {
                pocket.consecutive_failure_count = 0;
                pocket.occupancy = StripPocketOccupancy::Empty;
                pocket.status = StripFeederPocketStatus::Consumed;
            }
            StripFeederPocketOutcome::ObservedEmpty => {
                pocket.consecutive_failure_count = 0;
                pocket.occupancy = StripPocketOccupancy::Empty;
                pocket.status = StripFeederPocketStatus::Empty;
            }
            StripFeederPocketOutcome::RetryableFailure
            | StripFeederPocketOutcome::ReleaseUncertain
            | StripFeederPocketOutcome::Interrupted => {
                pocket.consecutive_failure_count =
                    pocket.consecutive_failure_count.saturating_add(1);
                if matches!(
                    outcome,
                    StripFeederPocketOutcome::ReleaseUncertain
                        | StripFeederPocketOutcome::Interrupted
                ) {
                    pocket.occupancy = StripPocketOccupancy::Uncertain;
                }
                pocket.status = if outcome == StripFeederPocketOutcome::RetryableFailure
                    && pocket.consecutive_failure_count < 3
                {
                    StripFeederPocketStatus::Available
                } else {
                    StripFeederPocketStatus::NeedsReview
                };
            }
        }
        if registration.selected_pocket_index == Some(pocket.pocket_index)
            && pocket.status != StripFeederPocketStatus::Available
        {
            let next = registration.next_available_pocket(None).map(|pocket| {
                (
                    pocket.pocket_index,
                    pocket
                        .measured_center_machine_mm
                        .unwrap_or(pocket.nominal_center_machine_mm),
                )
            });
            registration.selected_pocket_index = next.map(|item| item.0);
            registration.selected_pocket_machine_mm = next.map(|item| item.1);
        }
        registration.updated_at = Utc::now();
        transaction.execute(
            r#"UPDATE entities
               SET updated_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), document = ?3
               WHERE kind = ?1 AND id = ?2"#,
            params![
                "strip_feeder_registration",
                registration_id.to_string(),
                serde_json::to_string(&registration)?,
            ],
        )?;
        transaction.commit()?;
        Ok(registration)
    }

    /// Replaces a completed pocket outcome after deterministic evidence
    /// reanalysis. This does not add another physical pickup attempt.
    pub fn reconcile_strip_feeder_pocket_qualification(
        &self,
        registration_id: Uuid,
        qualification_id: Uuid,
        outcome: StripFeederPocketOutcome,
    ) -> Result<StripFeederRegistration> {
        let mut connection = self.connection.lock();
        let transaction = connection.transaction()?;
        let document: String = transaction
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params!["strip_feeder_registration", registration_id.to_string()],
                |row| row.get(0),
            )
            .context("strip feeder registration not found")?;
        let mut registration: StripFeederRegistration =
            serde_json::from_str(&document).context("decoding strip feeder registration")?;
        registration.ensure_pocket_state();
        let pocket = registration
            .pockets
            .iter_mut()
            .find(|pocket| pocket.last_qualification_id == Some(qualification_id))
            .context("strip feeder qualification has no settled pocket")?;
        anyhow::ensure!(
            pocket.reservation_id.is_none(),
            "strip feeder qualification pocket is still reserved"
        );
        let prior_outcome = pocket.last_outcome;
        pocket.last_outcome = Some(outcome);
        pocket.updated_at = Utc::now();
        match outcome {
            StripFeederPocketOutcome::Returned => {
                pocket.consecutive_failure_count = 0;
                pocket.occupancy = StripPocketOccupancy::Occupied;
                pocket.status = StripFeederPocketStatus::Available;
            }
            StripFeederPocketOutcome::Consumed => {
                pocket.consecutive_failure_count = 0;
                pocket.occupancy = StripPocketOccupancy::Empty;
                pocket.status = StripFeederPocketStatus::Consumed;
            }
            StripFeederPocketOutcome::ObservedEmpty => {
                pocket.consecutive_failure_count = 0;
                pocket.occupancy = StripPocketOccupancy::Empty;
                pocket.status = StripFeederPocketStatus::Empty;
            }
            StripFeederPocketOutcome::RetryableFailure
            | StripFeederPocketOutcome::ReleaseUncertain
            | StripFeederPocketOutcome::Interrupted => {
                if prior_outcome != Some(outcome) {
                    pocket.consecutive_failure_count =
                        pocket.consecutive_failure_count.saturating_add(1);
                }
                pocket.consecutive_failure_count = pocket.consecutive_failure_count.max(1);
                pocket.occupancy = StripPocketOccupancy::Uncertain;
                pocket.status = StripFeederPocketStatus::NeedsReview;
            }
        }
        if registration.selected_pocket_index == Some(pocket.pocket_index)
            && pocket.status != StripFeederPocketStatus::Available
        {
            let next = registration.next_available_pocket(None).map(|pocket| {
                (
                    pocket.pocket_index,
                    pocket
                        .measured_center_machine_mm
                        .unwrap_or(pocket.nominal_center_machine_mm),
                )
            });
            registration.selected_pocket_index = next.map(|item| item.0);
            registration.selected_pocket_machine_mm = next.map(|item| item.1);
        }
        registration.updated_at = Utc::now();
        transaction.execute(
            r#"UPDATE entities
               SET updated_at = strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), document = ?3
               WHERE kind = ?1 AND id = ?2"#,
            params![
                "strip_feeder_registration",
                registration_id.to_string(),
                serde_json::to_string(&registration)?,
            ],
        )?;
        transaction.commit()?;
        Ok(registration)
    }

    pub fn put_strip_feeder_live_holdout(&self, run: &StripFeederLiveHoldoutRun) -> Result<()> {
        self.put("strip_feeder_live_holdout", run.id, run)
    }

    pub fn list_strip_feeder_live_holdouts(&self) -> Result<Vec<StripFeederLiveHoldoutRun>> {
        self.list("strip_feeder_live_holdout")
    }

    pub fn get_strip_feeder_live_holdout(
        &self,
        id: Uuid,
    ) -> Result<Option<StripFeederLiveHoldoutRun>> {
        self.get("strip_feeder_live_holdout", id)
    }

    pub fn put_strip_feeder_source_capture(&self, run: &StripFeederSourceCaptureRun) -> Result<()> {
        self.put("strip_feeder_source_capture", run.id, run)
    }

    pub fn list_strip_feeder_source_captures(&self) -> Result<Vec<StripFeederSourceCaptureRun>> {
        self.list("strip_feeder_source_capture")
    }

    pub fn get_strip_feeder_source_capture(
        &self,
        id: Uuid,
    ) -> Result<Option<StripFeederSourceCaptureRun>> {
        self.get("strip_feeder_source_capture", id)
    }

    pub fn put_strip_feeder_source_capture_artifact(
        &self,
        capture_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(!name.is_empty(), "source capture artifact name is empty");
        anyhow::ensure!(
            !media_type.is_empty(),
            "source capture artifact media type is empty"
        );
        self.connection.lock().execute(
            r#"INSERT INTO strip_feeder_source_capture_artifacts(
                 capture_id, name, media_type, data, created_at
               ) VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(capture_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![capture_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_strip_feeder_source_capture_artifact(
        &self,
        capture_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM strip_feeder_source_capture_artifacts \
                 WHERE capture_id = ?1 AND name = ?2",
                params![capture_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    pub fn put_strip_feeder_pickup_qualification(
        &self,
        run: &StripFeederPickupQualificationRun,
    ) -> Result<()> {
        self.put("strip_feeder_pickup_qualification", run.id, run)
    }

    pub fn list_strip_feeder_pickup_qualifications(
        &self,
    ) -> Result<Vec<StripFeederPickupQualificationRun>> {
        self.list("strip_feeder_pickup_qualification")
    }

    pub fn get_strip_feeder_pickup_qualification(
        &self,
        id: Uuid,
    ) -> Result<Option<StripFeederPickupQualificationRun>> {
        self.get("strip_feeder_pickup_qualification", id)
    }

    pub fn put_strip_feeder_holdout_artifact(
        &self,
        holdout_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(
            !name.is_empty(),
            "strip feeder holdout artifact name is empty"
        );
        anyhow::ensure!(
            !media_type.is_empty(),
            "strip feeder holdout artifact media type is empty"
        );
        self.connection.lock().execute(
            r#"INSERT INTO strip_feeder_holdout_artifacts(
                 holdout_id, name, media_type, data, created_at
               ) VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(holdout_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![holdout_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_strip_feeder_holdout_artifact(
        &self,
        holdout_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM strip_feeder_holdout_artifacts \
                 WHERE holdout_id = ?1 AND name = ?2",
                params![holdout_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    pub fn put_strip_feeder_pickup_artifact(
        &self,
        qualification_id: Uuid,
        name: &str,
        media_type: &str,
        data: &[u8],
    ) -> Result<()> {
        anyhow::ensure!(
            !name.is_empty(),
            "strip feeder pickup artifact name is empty"
        );
        anyhow::ensure!(
            !media_type.is_empty(),
            "strip feeder pickup artifact media type is empty"
        );
        self.connection.lock().execute(
            r#"INSERT INTO strip_feeder_pickup_artifacts(
                 qualification_id, name, media_type, data, created_at
               ) VALUES (?1, ?2, ?3, ?4, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
               ON CONFLICT(qualification_id, name) DO UPDATE SET
                 media_type = excluded.media_type,
                 data = excluded.data,
                 created_at = excluded.created_at"#,
            params![qualification_id.to_string(), name, media_type, data],
        )?;
        Ok(())
    }

    pub fn get_strip_feeder_pickup_artifact(
        &self,
        qualification_id: Uuid,
        name: &str,
    ) -> Result<Option<(String, Vec<u8>)>> {
        self.connection
            .lock()
            .query_row(
                "SELECT media_type, data FROM strip_feeder_pickup_artifacts \
                 WHERE qualification_id = ?1 AND name = ?2",
                params![qualification_id.to_string(), name],
                |row| Ok((row.get(0)?, row.get(1)?)),
            )
            .optional()
            .map_err(Into::into)
    }

    /// Lists retained artifact names and media types without loading blobs.
    pub fn list_strip_feeder_pickup_artifacts(
        &self,
        qualification_id: Uuid,
    ) -> Result<Vec<(String, String)>> {
        self.list_artifact_metadata(
            "strip_feeder_pickup_artifacts",
            "qualification_id",
            qualification_id,
        )
    }

    pub fn put_job(&self, job: &Job) -> Result<()> {
        self.put("job", job.id, job)
    }

    pub fn list_jobs(&self) -> Result<Vec<Job>> {
        self.list("job")
    }

    pub fn get_job(&self, id: Uuid) -> Result<Option<Job>> {
        self.get("job", id)
    }

    pub fn put_job_execution(&self, run: &JobExecutionRun) -> Result<()> {
        self.put("job_execution", run.id, run)
    }

    pub fn list_job_executions(&self) -> Result<Vec<JobExecutionRun>> {
        self.list("job_execution")
    }

    pub fn get_job_execution(&self, id: Uuid) -> Result<Option<JobExecutionRun>> {
        self.get("job_execution", id)
    }

    pub fn put_placement_execution(&self, run: &PlacementExecutionRun) -> Result<()> {
        self.put("placement_execution", run.id, run)
    }

    pub fn list_placement_executions(&self) -> Result<Vec<PlacementExecutionRun>> {
        self.list("placement_execution")
    }

    pub fn get_placement_execution(&self, id: Uuid) -> Result<Option<PlacementExecutionRun>> {
        self.get("placement_execution", id)
    }

    pub fn put_placement_outcome(&self, outcome: &PlacementOutcomeRecord) -> Result<()> {
        self.put("placement_outcome", outcome.id, outcome)
    }

    pub fn list_placement_outcomes(&self) -> Result<Vec<PlacementOutcomeRecord>> {
        self.list("placement_outcome")
    }

    pub fn get_placement_outcome(&self, id: Uuid) -> Result<Option<PlacementOutcomeRecord>> {
        self.get("placement_outcome", id)
    }

    pub fn put_inspection(&self, inspection: &InspectionReport) -> Result<()> {
        self.put("inspection", inspection.id, inspection)
    }

    pub fn list_inspections(&self) -> Result<Vec<InspectionReport>> {
        self.list("inspection")
    }

    pub fn get_inspection(&self, id: Uuid) -> Result<Option<InspectionReport>> {
        self.get("inspection", id)
    }

    pub fn append_event(&self, event: &MachineEvent) -> Result<()> {
        self.connection.lock().execute(
            "INSERT INTO event_journal(machine_sequence, occurred_at, kind, payload) VALUES (?1, ?2, ?3, ?4)",
            params![event.sequence, event.at.to_rfc3339(), event.kind, serde_json::to_string(&event.payload)?],
        )?;
        Ok(())
    }

    pub fn recent_events(&self, limit: u32) -> Result<Vec<MachineEvent>> {
        let connection = self.connection.lock();
        let mut statement = connection.prepare(
            "SELECT machine_sequence, occurred_at, kind, payload FROM event_journal ORDER BY row_id DESC LIMIT ?1",
        )?;
        let rows = statement.query_map([limit.clamp(1, 10_000)], |row| {
            let at: String = row.get(1)?;
            let payload: String = row.get(3)?;
            Ok(MachineEvent {
                sequence: row.get(0)?,
                at: chrono::DateTime::parse_from_rfc3339(&at)
                    .map(|value| value.with_timezone(&chrono::Utc))
                    .map_err(|error| {
                        rusqlite::Error::FromSqlConversionFailure(
                            1,
                            rusqlite::types::Type::Text,
                            Box::new(error),
                        )
                    })?,
                kind: row.get(2)?,
                payload: serde_json::from_str(&payload).map_err(|error| {
                    rusqlite::Error::FromSqlConversionFailure(
                        3,
                        rusqlite::types::Type::Text,
                        Box::new(error),
                    )
                })?,
            })
        })?;
        let mut events = rows.collect::<rusqlite::Result<Vec<_>>>()?;
        events.reverse();
        Ok(events)
    }

    fn list_artifact_metadata(
        &self,
        table: &str,
        id_column: &str,
        id: Uuid,
    ) -> Result<Vec<(String, String)>> {
        let query = match (table, id_column) {
            ("placement_execution_artifacts", "execution_id") => {
                "SELECT name, media_type FROM placement_execution_artifacts \
                 WHERE execution_id = ?1 ORDER BY name"
            }
            ("strip_feeder_pickup_artifacts", "qualification_id") => {
                "SELECT name, media_type FROM strip_feeder_pickup_artifacts \
                 WHERE qualification_id = ?1 ORDER BY name"
            }
            ("workspace_scan_artifacts", "scan_id") => {
                "SELECT name, media_type FROM workspace_scan_artifacts \
                 WHERE scan_id = ?1 ORDER BY name"
            }
            _ => unreachable!("artifact table and key are fixed by Store methods"),
        };
        let connection = self.connection.lock();
        let mut statement = connection.prepare(query)?;
        let rows = statement.query_map(params![id.to_string()], |row| {
            Ok((row.get(0)?, row.get(1)?))
        })?;
        rows.collect::<rusqlite::Result<Vec<_>>>()
            .map_err(Into::into)
    }

    fn put<T: Serialize>(&self, kind: &str, id: Uuid, value: &T) -> Result<()> {
        self.put_named(kind, &id.to_string(), value)
    }

    fn put_named<T: Serialize>(&self, kind: &str, id: &str, value: &T) -> Result<()> {
        self.connection.lock().execute(
            r#"INSERT INTO entities(kind, id, updated_at, document)
               VALUES (?1, ?2, strftime('%Y-%m-%dT%H:%M:%fZ', 'now'), ?3)
               ON CONFLICT(kind, id) DO UPDATE SET
                 updated_at = excluded.updated_at,
                 document = excluded.document"#,
            params![kind, id, serde_json::to_string(value)?],
        )?;
        Ok(())
    }

    fn get<T: DeserializeOwned>(&self, kind: &str, id: Uuid) -> Result<Option<T>> {
        self.get_named(kind, &id.to_string())
    }

    fn get_named<T: DeserializeOwned>(&self, kind: &str, id: &str) -> Result<Option<T>> {
        let document: Option<String> = self
            .connection
            .lock()
            .query_row(
                "SELECT document FROM entities WHERE kind = ?1 AND id = ?2",
                params![kind, id],
                |row| row.get(0),
            )
            .optional()?;
        document
            .map(|value| serde_json::from_str(&value).context("decoding stored entity"))
            .transpose()
    }

    fn list<T: DeserializeOwned>(&self, kind: &str) -> Result<Vec<T>> {
        let connection = self.connection.lock();
        let mut statement = connection
            .prepare("SELECT document FROM entities WHERE kind = ?1 ORDER BY updated_at DESC")?;
        let rows = statement.query_map([kind], |row| row.get::<_, String>(0))?;
        rows.map(|row| {
            let document = row?;
            serde_json::from_str(&document).map_err(|error| {
                rusqlite::Error::FromSqlConversionFailure(
                    0,
                    rusqlite::types::Type::Text,
                    Box::new(error),
                )
            })
        })
        .collect::<rusqlite::Result<Vec<_>>>()
        .map_err(Into::into)
    }
}

fn feeder_id(feeder: &Feeder) -> Uuid {
    match feeder {
        Feeder::Strip { id, .. } | Feeder::Drag { id, .. } | Feeder::Photon { id, .. } => *id,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use std::collections::BTreeMap;
    use velocitypnp_domain::{
        AutomatedCalibrationConfig, BoardDesign, CalibrationRunPhase, CalibrationRunStatus,
        OperationStatus, WorkspaceArtifactRetention, WorkspacePixelBounds,
        WorkspaceRasterTransform, WorkspaceScanMode, WorkspaceScanRegion, WorkspaceScanStatus,
        WorkspaceSurfaceModel,
    };

    #[test]
    fn board_round_trip() {
        let store = Store::memory().unwrap();
        let board = BoardDesign {
            id: Uuid::new_v4(),
            name: "fixture".into(),
            source_format: "kicad_pcb".into(),
            source_version: Some(20240108),
            width_mm: Some(10.0),
            height_mm: Some(20.0),
            outline_bounds: None,
            placements: vec![],
            warnings: vec![],
        };
        store.put_board(&board).unwrap();
        assert_eq!(store.get_board(board.id).unwrap().unwrap().name, "fixture");
    }

    #[test]
    fn strip_feeder_occupancy_refresh_round_trip() {
        let store = Store::memory().unwrap();
        let report = StripFeederOccupancyRefreshReport {
            algorithm_version: "strip_feeder_occupancy_refresh.v1".into(),
            source_scan_id: Uuid::new_v4(),
            source_candidate_index: 2,
            source_proposal_index: 1,
            registration_id: Uuid::new_v4(),
            report_artifact_name: "occupancy.json".into(),
            created_at: Utc::now(),
            model_available: true,
            lane_revision: None,
            observations: Vec::new(),
            warnings: Vec::new(),
        };

        store.put_strip_feeder_occupancy_refresh(&report).unwrap();

        assert_eq!(
            store.list_strip_feeder_occupancy_refreshes().unwrap(),
            [report],
        );
    }

    #[test]
    fn board_population_refresh_round_trip() {
        let store = Store::memory().unwrap();
        let report = BoardPopulationRefreshReport {
            algorithm_version: "board_population_refresh.v1".into(),
            source_scan_id: Uuid::new_v4(),
            profile_id: "test".into(),
            calibration_id: Uuid::new_v4(),
            job_id: Uuid::new_v4(),
            board_instance_id: Uuid::new_v4(),
            board_design_id: Uuid::new_v4(),
            design_to_machine: velocitypnp_domain::Transform2d::identity(),
            report_artifact_name: "board-population.json".into(),
            created_at: Utc::now(),
            observations: Vec::new(),
            warnings: Vec::new(),
        };

        store.put_board_population_refresh(&report).unwrap();

        assert_eq!(store.list_board_population_refreshes().unwrap(), [report]);
    }

    #[test]
    fn feeder_rescan_updates_operational_pocket_state() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let registration_id = Uuid::new_v4();
        let pocket = |pocket_index, occupancy, status| velocitypnp_domain::StripFeederPocketState {
            pocket_index,
            nominal_center_machine_mm: [10.0, f64::from(pocket_index) * 4.0],
            measured_center_machine_mm: None,
            occupancy,
            occupancy_confidence: 0.5,
            status,
            attempt_count: 1,
            consecutive_failure_count: 0,
            reservation_id: None,
            last_qualification_id: None,
            last_outcome: Some(StripFeederPocketOutcome::Consumed),
            updated_at: now,
        };
        let registration = StripFeederRegistration {
            id: registration_id,
            profile_id: "test".into(),
            calibration_id: Uuid::new_v4(),
            source_scan_id: Uuid::new_v4(),
            source_candidate_index: 0,
            source_proposal_index: 0,
            created_at: now,
            updated_at: now,
            status: velocitypnp_domain::StripFeederRegistrationStatus::Ready,
            name: "test strip".into(),
            part_key: None,
            part_rotation_deg: None,
            tape_width_mm: 8.0,
            tape_pitch_mm: 4.0,
            tape_rotation_deg: 90.0,
            pick_centerline_machine_mm: [[10.0, 0.0], [10.0, 4.0]],
            pocket_centers_machine_mm: vec![[10.0, 0.0], [10.0, 4.0]],
            pockets: vec![
                pocket(
                    0,
                    StripPocketOccupancy::Empty,
                    StripFeederPocketStatus::Consumed,
                ),
                pocket(
                    1,
                    StripPocketOccupancy::Occupied,
                    StripFeederPocketStatus::Available,
                ),
            ],
            selected_pocket_machine_mm: Some([10.0, 4.0]),
            selected_pocket_index: Some(1),
            pickup_z_mm: None,
            surveyed_part_size_mm: None,
            measured_part_size_mm: None,
            measured_part_rotation_deg: None,
            measured_part_confidence: None,
            active_feeder_id: None,
            latest_live_holdout_id: None,
            latest_pickup_qualification_id: None,
            pickup_sequence_passed: false,
            latest_pickup_sequence_qualification_ids: Vec::new(),
            latest_placement_validation_id: None,
            vision_confidence: 0.9,
            vision_review_required: false,
            findings: Vec::new(),
        };
        store.put_strip_feeder_registration(&registration).unwrap();
        let observed_at = now + chrono::Duration::seconds(1);
        let observation =
            |pocket_index, center, occupancy, review_required| StripFeederPocketScanObservation {
                pocket_index,
                center_machine_mm: center,
                deterministic_occupancy: occupancy,
                deterministic_confidence: 0.95,
                model_prediction: None,
                occupancy,
                confidence: 0.95,
                review_required,
                findings: Vec::new(),
            };

        let refreshed = store
            .apply_strip_feeder_occupancy_refresh(
                registration_id,
                &[
                    observation(0, [10.1, 0.1], StripPocketOccupancy::Occupied, false),
                    observation(1, [10.1, 4.1], StripPocketOccupancy::Empty, false),
                ],
                observed_at,
            )
            .unwrap();

        assert_eq!(
            refreshed.pockets[0].status,
            StripFeederPocketStatus::Available
        );
        assert_eq!(refreshed.pockets[1].status, StripFeederPocketStatus::Empty);
        assert_eq!(refreshed.pockets[0].attempt_count, 1);
        assert_eq!(refreshed.selected_pocket_index, Some(0));
        assert_eq!(refreshed.selected_pocket_machine_mm, Some([10.1, 0.1]));
    }

    #[test]
    fn feeder_lane_revision_promotes_all_active_state_atomically() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let registration_id = Uuid::new_v4();
        let feeder_id = Uuid::new_v4();
        let base_scan_id = Uuid::new_v4();
        let source_scan_id = Uuid::new_v4();
        let old_patch_id = Uuid::new_v4();
        let new_patch_id = Uuid::new_v4();
        let calibration_id = Uuid::new_v4();
        let raster_transform = WorkspaceRasterTransform {
            top_left_x_mm: 0.0,
            top_left_y_mm: 20.0,
            x_mm_per_pixel: 0.1,
            y_mm_per_pixel: 0.1,
        };
        let residuals = velocitypnp_domain::Residuals {
            sample_count: 8,
            rms_mm: 0.08,
            p95_mm: 0.12,
            max_mm: 0.14,
        };
        let pocket = velocitypnp_domain::StripFeederPocketState {
            pocket_index: 7,
            nominal_center_machine_mm: [10.0, 4.0],
            measured_center_machine_mm: Some([10.1, 4.1]),
            occupancy: StripPocketOccupancy::Occupied,
            occupancy_confidence: 0.9,
            status: StripFeederPocketStatus::Consumed,
            attempt_count: 3,
            consecutive_failure_count: 1,
            reservation_id: None,
            last_qualification_id: Some(Uuid::new_v4()),
            last_outcome: Some(StripFeederPocketOutcome::Consumed),
            updated_at: now,
        };
        let registration = StripFeederRegistration {
            id: registration_id,
            profile_id: "test".into(),
            calibration_id,
            source_scan_id: Uuid::new_v4(),
            source_candidate_index: 3,
            source_proposal_index: 2,
            created_at: now,
            updated_at: now,
            status: velocitypnp_domain::StripFeederRegistrationStatus::Ready,
            name: "Feeder Bank 1 - lane 8".into(),
            part_key: Some("R_0603".into()),
            part_rotation_deg: Some(90.0),
            tape_width_mm: 8.0,
            tape_pitch_mm: 4.0,
            tape_rotation_deg: 89.0,
            pick_centerline_machine_mm: [[10.0, 0.0], [10.0, 8.0]],
            pocket_centers_machine_mm: vec![[10.0, 4.0]],
            pockets: vec![pocket],
            selected_pocket_machine_mm: None,
            selected_pocket_index: None,
            pickup_z_mm: Some(12.5),
            surveyed_part_size_mm: Some([1.6, 0.8]),
            measured_part_size_mm: Some([1.58, 0.79]),
            measured_part_rotation_deg: Some(90.0),
            measured_part_confidence: Some(0.95),
            active_feeder_id: Some(feeder_id),
            latest_live_holdout_id: Some(Uuid::new_v4()),
            latest_pickup_qualification_id: Some(Uuid::new_v4()),
            pickup_sequence_passed: true,
            latest_pickup_sequence_qualification_ids: vec![Uuid::new_v4()],
            latest_placement_validation_id: Some(Uuid::new_v4()),
            vision_confidence: 0.9,
            vision_review_required: false,
            findings: Vec::new(),
        };
        store.put_strip_feeder_registration(&registration).unwrap();
        store
            .put_feeder(&Feeder::Strip {
                id: feeder_id,
                name: registration.name.clone(),
                part_key: "R_0603".into(),
                first_pick: velocitypnp_domain::Pose {
                    x_mm: 10.1,
                    y_mm: 4.1,
                    z_mm: 12.5,
                    rotation_deg: 90.0,
                },
                tape_pitch_mm: 4.0,
                tape_width_mm: 8.0,
                tape_rotation_deg: 89.0,
                remaining: Some(0),
            })
            .unwrap();
        let old_patch = WorkspaceDisplayPatch {
            id: old_patch_id,
            algorithm_version: "workspace_display_patch.v1".into(),
            base_scan_id,
            source_scan_id: Uuid::new_v4(),
            profile_id: "test".into(),
            calibration_id,
            kind: WorkspaceDisplayPatchKind::FeederLane,
            object_id: registration_id,
            object_label: registration.name.clone(),
            artifact_name: "old.png".into(),
            source_raster_transform: raster_transform,
            source_width_px: 100,
            source_height_px: 200,
            source_to_base: velocitypnp_domain::Transform2d::identity(),
            clip_polygon_machine_mm: vec![[9.0, 0.0], [11.0, 0.0], [11.0, 8.0]],
            anchor_count: 8,
            alignment_residuals: residuals,
            accepted: true,
            rejection_reason: None,
            findings: Vec::new(),
            source_created_at: now,
            evaluated_at: now,
        };
        store.put_workspace_display_patch(&old_patch).unwrap();
        let observed_at = now + chrono::Duration::seconds(1);
        let observation =
            |pocket_index, center, occupancy, review_required| StripFeederPocketScanObservation {
                pocket_index,
                center_machine_mm: center,
                deterministic_occupancy: occupancy,
                deterministic_confidence: 0.96,
                model_prediction: None,
                occupancy,
                confidence: 0.96,
                review_required,
                findings: Vec::new(),
            };
        let report = StripFeederOccupancyRefreshReport {
            algorithm_version: "strip_feeder_lane_revision.v2".into(),
            source_scan_id,
            source_candidate_index: 1,
            source_proposal_index: 0,
            registration_id,
            report_artifact_name: "lane-revision.json".into(),
            created_at: observed_at,
            model_available: true,
            lane_revision: Some(velocitypnp_domain::StripFeederLaneRevision {
                id: Uuid::new_v4(),
                display_patch_id: new_patch_id,
                base_scan_id,
                profile_id: "test".into(),
                calibration_id,
                mosaic_artifact_name: "mosaic.png".into(),
                raster_transform,
                mosaic_width_px: 120,
                mosaic_height_px: 320,
                tape_width_mm: 12.0,
                tape_pitch_mm: 8.0,
                tape_rotation_deg: 90.0,
                pick_centerline_machine_mm: [[20.0, 2.0], [20.0, 18.0]],
                geometry_residuals: residuals,
                vision_confidence: 0.97,
            }),
            observations: vec![
                observation(0, [20.0, 2.0], StripPocketOccupancy::Empty, false),
                observation(1, [20.0, 10.0], StripPocketOccupancy::Occupied, false),
                observation(2, [20.0, 18.0], StripPocketOccupancy::Uncertain, true),
            ],
            warnings: Vec::new(),
        };
        let new_patch = WorkspaceDisplayPatch {
            id: new_patch_id,
            algorithm_version: "workspace_display_patch.feeder_lane_revision.v2".into(),
            base_scan_id,
            source_scan_id,
            profile_id: "test".into(),
            calibration_id,
            kind: WorkspaceDisplayPatchKind::FeederLane,
            object_id: registration_id,
            object_label: registration.name.clone(),
            artifact_name: "mosaic.png".into(),
            source_raster_transform: raster_transform,
            source_width_px: 120,
            source_height_px: 320,
            source_to_base: velocitypnp_domain::Transform2d::identity(),
            clip_polygon_machine_mm: vec![[18.0, 0.0], [22.0, 0.0], [22.0, 20.0]],
            anchor_count: 3,
            alignment_residuals: residuals,
            accepted: true,
            rejection_reason: None,
            findings: Vec::new(),
            source_created_at: observed_at,
            evaluated_at: observed_at,
        };

        let promoted = store
            .promote_strip_feeder_lane_revision(&report, &new_patch)
            .unwrap();

        assert_eq!(promoted.source_scan_id, source_scan_id);
        assert_eq!(promoted.tape_width_mm, 12.0);
        assert_eq!(promoted.tape_pitch_mm, 8.0);
        assert_eq!(promoted.pockets.len(), 3);
        assert_eq!(promoted.pockets[0].status, StripFeederPocketStatus::Empty);
        assert_eq!(
            promoted.pockets[1].status,
            StripFeederPocketStatus::Available
        );
        assert_eq!(promoted.pockets[1].attempt_count, 0);
        assert_eq!(promoted.pockets[1].last_outcome, None);
        assert_eq!(
            promoted.pockets[2].status,
            StripFeederPocketStatus::NeedsReview
        );
        assert_eq!(promoted.selected_pocket_index, Some(1));
        assert_eq!(promoted.selected_pocket_machine_mm, Some([20.0, 10.0]));
        assert_eq!(
            store.list_strip_feeder_occupancy_refreshes().unwrap(),
            [report]
        );

        let patches = store.list_workspace_display_patches().unwrap();
        let active_patch = patches
            .iter()
            .find(|patch| patch.id == new_patch_id)
            .unwrap();
        let stale_patch = patches
            .iter()
            .find(|patch| patch.id == old_patch_id)
            .unwrap();
        assert!(active_patch.accepted);
        assert!(!stale_patch.accepted);
        assert_eq!(
            stale_patch.rejection_reason.as_deref(),
            Some("Superseded by a newer feeder lane revision")
        );

        let feeders = store.list_feeders().unwrap();
        let Feeder::Strip {
            first_pick,
            tape_pitch_mm,
            tape_width_mm,
            tape_rotation_deg,
            remaining,
            ..
        } = &feeders[0]
        else {
            panic!("active feeder changed type");
        };
        assert_eq!([first_pick.x_mm, first_pick.y_mm], [20.0, 10.0]);
        assert_eq!(first_pick.z_mm, 12.5);
        assert_eq!(first_pick.rotation_deg, 90.0);
        assert_eq!(*tape_pitch_mm, 8.0);
        assert_eq!(*tape_width_mm, 12.0);
        assert_eq!(*tape_rotation_deg, 90.0);
        assert_eq!(*remaining, Some(1));
    }

    #[test]
    fn board_surface_measurement_and_artifact_round_trip() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let measurement = BoardSurfaceMeasurementRun {
            id: Uuid::new_v4(),
            operation_id: Uuid::new_v4(),
            registration_id: Uuid::new_v4(),
            profile_id: "lumenpnp-v3.0.2".into(),
            calibration_id: Uuid::new_v4(),
            nozzle_id: "N1".into(),
            status: OperationStatus::Pending,
            created_at: now,
            updated_at: now,
            sample_count: 3,
            report: None,
            message: None,
        };

        store.put_board_surface_measurement(&measurement).unwrap();
        assert_eq!(
            store.get_board_surface_measurement(measurement.id).unwrap(),
            Some(measurement.clone())
        );
        store
            .put_board_surface_measurement_artifact(
                measurement.id,
                "report.json",
                "application/json",
                b"{}",
            )
            .unwrap();
        assert_eq!(
            store
                .get_board_surface_measurement_artifact(measurement.id, "report.json")
                .unwrap(),
            Some(("application/json".into(), b"{}".to_vec()))
        );
    }

    #[test]
    fn camera_profile_round_trip_uses_machine_and_role_key() {
        let store = Store::memory().unwrap();
        let profile = CameraProfile::initial_lumenpnp("lumenpnp-v3.0.2", CameraRole::BottomCamera);
        store.put_camera_profile(&profile).unwrap();
        assert_eq!(
            store
                .get_camera_profile("lumenpnp-v3.0.2", CameraRole::BottomCamera)
                .unwrap(),
            Some(profile)
        );
    }

    #[test]
    fn strip_pocket_reservation_skips_empty_and_requires_consumption_evidence() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let registration_id = Uuid::new_v4();
        let pocket = |pocket_index, occupancy, status| velocitypnp_domain::StripFeederPocketState {
            pocket_index,
            nominal_center_machine_mm: [10.0, f64::from(pocket_index) * 4.0],
            measured_center_machine_mm: None,
            occupancy,
            occupancy_confidence: 0.9,
            status,
            attempt_count: 0,
            consecutive_failure_count: 0,
            reservation_id: None,
            last_qualification_id: None,
            last_outcome: None,
            updated_at: now,
        };
        let registration = StripFeederRegistration {
            id: registration_id,
            profile_id: "test".into(),
            calibration_id: Uuid::new_v4(),
            source_scan_id: Uuid::new_v4(),
            source_candidate_index: 0,
            source_proposal_index: 0,
            created_at: now,
            updated_at: now,
            status: velocitypnp_domain::StripFeederRegistrationStatus::LiveValidationRequired,
            name: "test strip".into(),
            part_key: None,
            part_rotation_deg: None,
            tape_width_mm: 8.0,
            tape_pitch_mm: 4.0,
            tape_rotation_deg: 90.0,
            pick_centerline_machine_mm: [[10.0, 0.0], [10.0, 8.0]],
            pocket_centers_machine_mm: vec![[10.0, 0.0], [10.0, 4.0], [10.0, 8.0], [10.0, 12.0]],
            pockets: vec![
                pocket(
                    0,
                    StripPocketOccupancy::Empty,
                    StripFeederPocketStatus::Empty,
                ),
                pocket(
                    1,
                    StripPocketOccupancy::Occupied,
                    StripFeederPocketStatus::Available,
                ),
                pocket(
                    2,
                    StripPocketOccupancy::Occupied,
                    StripFeederPocketStatus::Available,
                ),
                pocket(
                    3,
                    StripPocketOccupancy::Occupied,
                    StripFeederPocketStatus::Available,
                ),
            ],
            selected_pocket_machine_mm: None,
            selected_pocket_index: None,
            pickup_z_mm: None,
            surveyed_part_size_mm: Some([1.6, 3.2]),
            measured_part_size_mm: None,
            measured_part_rotation_deg: None,
            measured_part_confidence: None,
            active_feeder_id: None,
            latest_live_holdout_id: None,
            latest_pickup_qualification_id: None,
            pickup_sequence_passed: false,
            latest_pickup_sequence_qualification_ids: Vec::new(),
            latest_placement_validation_id: None,
            vision_confidence: 0.9,
            vision_review_required: false,
            findings: Vec::new(),
        };
        store.put_strip_feeder_registration(&registration).unwrap();

        let first_reservation = Uuid::new_v4();
        let reserved = store
            .reserve_strip_feeder_pocket(registration_id, first_reservation, Some(0))
            .unwrap();
        assert_eq!(reserved.selected_pocket_index, Some(1));
        assert_eq!(
            reserved.pockets[1].status,
            StripFeederPocketStatus::Reserved
        );
        let released = store
            .release_strip_feeder_pocket_reservation(registration_id, first_reservation)
            .unwrap();
        assert_eq!(
            released.pockets[1].status,
            StripFeederPocketStatus::Available
        );
        assert_eq!(
            released.pockets[1].occupancy,
            StripPocketOccupancy::Occupied
        );
        assert_eq!(released.pockets[1].attempt_count, 0);

        let first_reservation = Uuid::new_v4();
        store
            .reserve_strip_feeder_pocket(registration_id, first_reservation, Some(1))
            .unwrap();
        assert!(store
            .settle_strip_feeder_pocket(
                registration_id,
                first_reservation,
                StripFeederPocketOutcome::Consumed,
                false,
            )
            .is_err());

        let consumed = store
            .settle_strip_feeder_pocket(
                registration_id,
                first_reservation,
                StripFeederPocketOutcome::Consumed,
                true,
            )
            .unwrap();
        assert_eq!(
            consumed.pockets[1].status,
            StripFeederPocketStatus::Consumed
        );
        assert_eq!(consumed.selected_pocket_index, Some(2));

        for failure_count in 1..=3 {
            let reservation = Uuid::new_v4();
            store
                .reserve_strip_feeder_pocket(registration_id, reservation, Some(2))
                .unwrap();
            let failed = store
                .settle_strip_feeder_pocket(
                    registration_id,
                    reservation,
                    StripFeederPocketOutcome::RetryableFailure,
                    false,
                )
                .unwrap();
            assert_eq!(failed.pockets[2].consecutive_failure_count, failure_count);
            assert_eq!(
                failed.pockets[2].status,
                if failure_count < 3 {
                    StripFeederPocketStatus::Available
                } else {
                    StripFeederPocketStatus::NeedsReview
                }
            );
        }
        let after_retries = store
            .get_strip_feeder_registration(registration_id)
            .unwrap()
            .unwrap();
        assert_eq!(after_retries.selected_pocket_index, Some(3));

        let interrupted_reservation = Uuid::new_v4();
        store
            .reserve_strip_feeder_pocket(registration_id, interrupted_reservation, None)
            .unwrap();
        let interrupted = store
            .settle_strip_feeder_pocket(
                registration_id,
                interrupted_reservation,
                StripFeederPocketOutcome::Interrupted,
                false,
            )
            .unwrap();
        assert_eq!(
            interrupted.pockets[3].status,
            StripFeederPocketStatus::NeedsReview
        );
        assert_eq!(
            interrupted.pockets[3].occupancy,
            StripPocketOccupancy::Uncertain
        );
        let reconciled = store
            .reconcile_strip_feeder_pocket_qualification(
                registration_id,
                interrupted_reservation,
                StripFeederPocketOutcome::Returned,
            )
            .unwrap();
        assert_eq!(
            reconciled.pockets[3].status,
            StripFeederPocketStatus::Available
        );
        assert_eq!(
            reconciled.pockets[3].occupancy,
            StripPocketOccupancy::Occupied
        );

        let empty_reservation = Uuid::new_v4();
        store
            .reserve_strip_feeder_pocket(registration_id, empty_reservation, Some(3))
            .unwrap();
        let observed_empty = store
            .settle_strip_feeder_pocket(
                registration_id,
                empty_reservation,
                StripFeederPocketOutcome::ObservedEmpty,
                false,
            )
            .unwrap();
        assert_eq!(
            observed_empty.pockets[3].status,
            StripFeederPocketStatus::Empty
        );
        assert_eq!(
            observed_empty.pockets[3].occupancy,
            StripPocketOccupancy::Empty
        );
        assert_eq!(observed_empty.selected_pocket_index, None);
    }

    #[test]
    fn calibration_run_round_trip_retains_progress() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let run = AutomatedCalibrationRun {
            id: Uuid::new_v4(),
            calibration_id: Uuid::new_v4(),
            operation_id: Uuid::new_v4(),
            profile_id: "lumenpnp-v3.0.2".into(),
            created_at: now,
            updated_at: now,
            status: CalibrationRunStatus::Running,
            phase: CalibrationRunPhase::VisualHomingValidation,
            config: AutomatedCalibrationConfig {
                camera_sample_count: 30,
                tooling_sample_count: 5,
                visual_homing_sample_count: 30,
                excursion_x_mm: -10.0,
                excursion_y_mm: -10.0,
                excursion_feed_mm_per_min: 600.0,
                visual_homing_p95_limit_mm: 0.05,
            },
            checks: vec![],
            camera_reports: BTreeMap::new(),
            visual_homing_samples: vec![],
            visual_homing_report: None,
            head_geometry_report: None,
            installed_geometry_report: None,
            placement_transfer_report: None,
            failure: None,
            simulated: true,
        };
        store.put_calibration_run(&run).unwrap();
        let restored = store.get_calibration_run(run.id).unwrap().unwrap();
        assert_eq!(restored.phase, CalibrationRunPhase::VisualHomingValidation);
        assert_eq!(restored.operation_id, run.operation_id);
    }

    #[test]
    fn placement_execution_round_trip_retains_pending_state() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let run = PlacementExecutionRun {
            id: Uuid::new_v4(),
            operation_id: Uuid::new_v4(),
            job_id: Uuid::new_v4(),
            step_index: 2,
            calibration_id: Uuid::new_v4(),
            status: velocitypnp_domain::OperationStatus::Pending,
            created_at: now,
            updated_at: now,
            report: None,
            message: None,
        };
        store.put_placement_execution(&run).unwrap();

        assert_eq!(store.get_placement_execution(run.id).unwrap(), Some(run));
    }

    #[test]
    fn job_execution_round_trip_retains_recovery_state() {
        use velocitypnp_domain::{
            JobExecutionRun, JobExecutionStatus, JobExecutionStep, JobExecutionStepStatus,
        };

        let store = Store::memory().unwrap();
        let now = Utc::now();
        let child_id = Uuid::new_v4();
        let run = JobExecutionRun {
            id: Uuid::new_v4(),
            operation_id: Uuid::new_v4(),
            job_id: Uuid::new_v4(),
            calibration_id: Uuid::new_v4(),
            simulated: false,
            status: JobExecutionStatus::NeedsAttention,
            created_at: now,
            updated_at: now,
            steps: vec![JobExecutionStep {
                step_index: 2,
                placement_reference: "R15".into(),
                status: JobExecutionStepStatus::NeedsAttention,
                execution_id: Some(child_id),
                attempt_execution_ids: vec![child_id],
                outcome_disposition: Some(
                    velocitypnp_domain::PlacementOutcomeDisposition::StuckOnNozzle,
                ),
                board_contacted: true,
                message: Some("part remained on N1".into()),
            }],
            current_step_index: Some(2),
            pause_requested: false,
            cancel_requested: false,
            message: Some("operator action is required".into()),
        };
        store.put_job_execution(&run).unwrap();

        assert_eq!(store.get_job_execution(run.id).unwrap(), Some(run));
    }

    #[test]
    fn placement_outcome_round_trip_retains_operator_review() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let id = Uuid::new_v4();
        let outcome = PlacementOutcomeRecord {
            id,
            execution_id: id,
            job_id: Uuid::new_v4(),
            placement_reference: "R10".into(),
            part_key: "R0603:10k".into(),
            package: "R_0603_1608Metric".into(),
            nozzle_id: "N1".into(),
            placement_surface_offset_mm: 0.0,
            placement_seating_offset_mm: 0.10,
            automatic_algorithm_version: "fixture".into(),
            automatic_disposition: velocitypnp_domain::PlacementOutcomeDisposition::Good,
            automatic_confidence: 0.95,
            operator_disposition: Some(velocitypnp_domain::PlacementOutcomeDisposition::Offset),
            operator_note: Some("slightly left".into()),
            measured_offset_x_mm: Some(-0.04),
            measured_offset_y_mm: Some(0.02),
            measured_rotation_error_deg: Some(0.2),
            created_at: now,
            updated_at: now,
        };
        store.put_placement_outcome(&outcome).unwrap();
        assert_eq!(store.get_placement_outcome(id).unwrap(), Some(outcome));
    }

    #[test]
    fn placement_execution_artifact_round_trip_retains_evidence() {
        let store = Store::memory().unwrap();
        let execution_id = Uuid::new_v4();
        let data = [5, 4, 3, 2, 1];

        store
            .put_placement_execution_artifact(
                execution_id,
                "held-part-inspection.png",
                "image/png",
                &data,
            )
            .unwrap();

        assert_eq!(
            store
                .get_placement_execution_artifact(execution_id, "held-part-inspection.png")
                .unwrap(),
            Some(("image/png".into(), data.to_vec()))
        );
        assert_eq!(
            store
                .list_placement_execution_artifacts(execution_id)
                .unwrap(),
            vec![("held-part-inspection.png".into(), "image/png".into())]
        );
    }

    #[test]
    fn calibration_artifact_round_trip_retains_media_type_and_bytes() {
        let store = Store::memory().unwrap();
        let run_id = Uuid::new_v4();
        let data = [1, 2, 3, 4];

        store
            .put_calibration_artifact(run_id, "view.png", "image/png", &data)
            .unwrap();

        assert_eq!(
            store.get_calibration_artifact(run_id, "view.png").unwrap(),
            Some(("image/png".into(), data.to_vec()))
        );
    }

    #[test]
    fn board_registration_artifact_round_trip_retains_evidence() {
        let store = Store::memory().unwrap();
        let registration_id = Uuid::new_v4();
        let data = [9, 8, 7, 6];
        store
            .put_board_registration_artifact(
                registration_id,
                "hypothesis-0-fid1.png",
                "image/png",
                &data,
            )
            .unwrap();
        assert_eq!(
            store
                .get_board_registration_artifact(registration_id, "hypothesis-0-fid1.png")
                .unwrap(),
            Some(("image/png".into(), data.to_vec()))
        );
    }

    #[test]
    fn strip_feeder_holdout_artifact_round_trip_retains_evidence() {
        let store = Store::memory().unwrap();
        let holdout_id = Uuid::new_v4();
        let data = [6, 5, 4, 3];
        store
            .put_strip_feeder_holdout_artifact(
                holdout_id,
                "live-pocket-holdout.png",
                "image/png",
                &data,
            )
            .unwrap();
        assert_eq!(
            store
                .get_strip_feeder_holdout_artifact(holdout_id, "live-pocket-holdout.png")
                .unwrap(),
            Some(("image/png".into(), data.to_vec()))
        );
    }

    #[test]
    fn strip_feeder_source_capture_artifact_round_trip_retains_evidence() {
        let store = Store::memory().unwrap();
        let capture_id = Uuid::new_v4();
        let data = [1, 3, 5, 7];
        store
            .put_strip_feeder_source_capture_artifact(
                capture_id,
                "lane-000-pocket-001.png",
                "image/png",
                &data,
            )
            .unwrap();
        assert_eq!(
            store
                .get_strip_feeder_source_capture_artifact(capture_id, "lane-000-pocket-001.png",)
                .unwrap(),
            Some(("image/png".into(), data.to_vec()))
        );
    }

    #[test]
    fn strip_feeder_pickup_artifact_round_trip_retains_evidence() {
        let store = Store::memory().unwrap();
        let qualification_id = Uuid::new_v4();
        let data = [2, 4, 6, 8];
        store
            .put_strip_feeder_pickup_artifact(
                qualification_id,
                "held-part-inspection.png",
                "image/png",
                &data,
            )
            .unwrap();
        assert_eq!(
            store
                .get_strip_feeder_pickup_artifact(qualification_id, "held-part-inspection.png",)
                .unwrap(),
            Some(("image/png".into(), data.to_vec()))
        );
        assert_eq!(
            store
                .list_strip_feeder_pickup_artifacts(qualification_id)
                .unwrap(),
            vec![("held-part-inspection.png".into(), "image/png".into())]
        );
    }

    #[test]
    fn workspace_scan_and_artifact_round_trip() {
        let store = Store::memory().unwrap();
        let now = Utc::now();
        let scan = WorkspaceScanReport {
            id: Uuid::new_v4(),
            operation_id: Uuid::new_v4(),
            calibration_id: Uuid::new_v4(),
            profile_id: "lumenpnp-v3.0.2".into(),
            parent_scan_id: None,
            refinement_target_index: None,
            refinement_algorithm_version: None,
            refinement_batch_operation_id: None,
            refinement_priority: None,
            refinement_severity_score: None,
            refinement_priority_algorithm_version: None,
            created_at: now,
            updated_at: now,
            status: WorkspaceScanStatus::Pending,
            mode: WorkspaceScanMode::Precision,
            artifact_retention: WorkspaceArtifactRetention::FullSources,
            surface_model: WorkspaceSurfaceModel::TopCameraWorkingPlane,
            region: WorkspaceScanRegion {
                x_min_mm: 10.0,
                x_max_mm: 20.0,
                y_min_mm: 30.0,
                y_max_mm: 40.0,
            },
            raster_transform: Some(WorkspaceRasterTransform {
                top_left_x_mm: 10.0,
                top_left_y_mm: 40.0,
                x_mm_per_pixel: 0.02,
                y_mm_per_pixel: -0.02,
            }),
            overlap_fraction: 0.35,
            output_mm_per_pixel: 0.02,
            feed_mm_per_min: 600.0,
            capture_settle_frames: 5,
            discarded_capture_frames: 0,
            timestamp_fresh_frames: 0,
            frame_width_px: 1280,
            frame_height_px: 720,
            usable_pixel_bounds: WorkspacePixelBounds {
                left: 300,
                top: 180,
                right: 980,
                bottom: 540,
            },
            rows: 2,
            columns: 2,
            planned_tiles: 4,
            captured_tiles: 0,
            mosaic_width_px: None,
            mosaic_height_px: None,
            covered_fraction: None,
            nominal_fallback_fraction: 0.0,
            estimated_duration_seconds: 30,
            estimated_retained_bytes: 1_000_000,
            artifact_quota_bytes: 128 * 1024 * 1024,
            retained_artifact_bytes: 0,
            persistent_capture: false,
            accepted_registrations: 0,
            attempted_registrations: 0,
            tiles: vec![],
            registrations: vec![],
            mosaic_artifact_name: None,
            mosaic_layers: Vec::new(),
            parallax: None,
            height_map: None,
            seam_health: None,
            refinement_composite: None,
            board_discovery: None,
            strip_feeder_inventory: None,
            change_set: None,
            warnings: vec![],
            failure: None,
        };
        store.put_workspace_scan(&scan).unwrap();
        assert_eq!(
            store.get_workspace_scan(scan.id).unwrap(),
            Some(scan.clone())
        );

        let data = [7, 8, 9];
        store
            .put_workspace_scan_artifact(scan.id, "mosaic.png", "image/png", &data)
            .unwrap();
        assert_eq!(
            store
                .get_workspace_scan_artifact(scan.id, "mosaic.png")
                .unwrap(),
            Some(("image/png".into(), data.to_vec()))
        );

        let baseline = WorkspaceInventoryBaseline {
            id: Uuid::new_v4(),
            scan_id: scan.id,
            profile_id: scan.profile_id.clone(),
            calibration_id: scan.calibration_id,
            proposal_algorithm_version: "workspace_object_proposals.v2".into(),
            activated_at: now,
            supersedes_baseline_id: None,
        };
        store.put_workspace_inventory_baseline(&baseline).unwrap();
        assert_eq!(
            store.get_workspace_inventory_baseline(baseline.id).unwrap(),
            Some(baseline.clone())
        );
        assert_eq!(
            store.list_workspace_inventory_baselines().unwrap(),
            vec![baseline]
        );
    }
}
