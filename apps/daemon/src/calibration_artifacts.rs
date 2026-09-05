use crate::api::AppState;
use chrono::Utc;
use serde::Serialize;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use uuid::Uuid;
use velocitypnp_domain::{
    lumen_calibration_artifacts, BottomCameraMappingReport, CalibrationArtifact,
    CalibrationArtifactDefinition, CalibrationArtifactId, CalibrationArtifactKind,
    CalibrationArtifactPlan, CalibrationPlanAction, CalibrationPlanItem, CalibrationSnapshot,
    CalibrationStepId, CameraProfile, CameraRole, HeadFeatureCenterReport, NozzleHeightModel,
    NozzleRunoutReport, VacuumProbeReport, VacuumSensorCharacterizationReport,
};
use velocitypnp_vision::calibrate_camera_ray_transfer;

const MIGRATION_VERSION: &str = "legacy_snapshot.v1";

pub fn reconcile_snapshot_artifacts(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
) -> Result<bool, String> {
    let before = snapshot.artifacts.clone();
    migrate_legacy_captures(state, snapshot)?;
    recompute_stale_derived(state, snapshot, None)?;
    migrate_legacy_vacuum_contacts(state, snapshot)?;
    Ok(snapshot.artifacts != before)
}

pub fn recompute_artifacts(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    targets: &[CalibrationArtifactId],
) -> Result<Vec<CalibrationArtifactId>, String> {
    migrate_legacy_captures(state, snapshot)?;
    let mut requested =
        (!targets.is_empty()).then(|| targets.iter().copied().collect::<BTreeSet<_>>());
    if let Some(requested) = requested.as_mut() {
        let definitions = definitions_by_id();
        if let Some(target) = requested.iter().find(|target| {
            definitions
                .get(target)
                .is_none_or(|definition| definition.kind != CalibrationArtifactKind::Derived)
        }) {
            return Err(format!("{target:?} is not a derived artifact"));
        }
        loop {
            let before = requested.len();
            for id in requested.clone() {
                if let Some(definition) = definitions.get(&id) {
                    requested.extend(definition.dependencies.iter().filter(|dependency| {
                        definitions
                            .get(dependency)
                            .is_some_and(|dependency_definition| {
                                dependency_definition.kind == CalibrationArtifactKind::Derived
                            })
                    }));
                }
            }
            if requested.len() == before {
                break;
            }
        }
    }
    let recomputed = recompute_stale_derived(state, snapshot, requested)?;
    migrate_legacy_vacuum_contacts(state, snapshot)?;
    Ok(recomputed)
}

pub fn plan_artifacts(
    state: &AppState,
    snapshot: &CalibrationSnapshot,
    targets: &[CalibrationArtifactId],
) -> Result<CalibrationArtifactPlan, String> {
    let definitions = definitions_by_id();
    let targets = if targets.is_empty() {
        vec![
            CalibrationArtifactId::LocalXyDirectionResponse,
            CalibrationArtifactId::VacuumCharacterizationN1,
            CalibrationArtifactId::VacuumCharacterizationN2,
        ]
    } else {
        targets.to_vec()
    };
    let mut actions = BTreeMap::new();
    for target in &targets {
        visit_plan(state, snapshot, *target, &definitions, &mut actions)?;
    }
    let items = actions.into_values().collect::<Vec<_>>();
    let requires_motion = items
        .iter()
        .any(|item| item.requires_motion && !matches!(item.action, CalibrationPlanAction::Reuse));
    let estimated_duration_seconds = items
        .iter()
        .filter(|item| !matches!(item.action, CalibrationPlanAction::Reuse))
        .map(|item| item.estimated_duration_seconds)
        .sum();
    Ok(CalibrationArtifactPlan {
        targets,
        items,
        requires_motion,
        estimated_duration_seconds,
    })
}

pub fn record_vacuum_contact_artifact(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    report: &VacuumProbeReport,
) -> Result<(), String> {
    reconcile_snapshot_artifacts(state, snapshot)?;
    let id = CalibrationArtifactId::vacuum_contact(&report.nozzle_id)
        .ok_or_else(|| "vacuum report names an unknown nozzle".to_string())?;
    let definition = definition(id)?;
    if !definition
        .dependencies
        .iter()
        .all(|dependency| artifact_tree_is_current(state, snapshot, *dependency))
    {
        return Err(format!(
            "{} vacuum contact depends on stale calibration artifacts",
            report.nozzle_id
        ));
    }
    snapshot.invalidate_artifacts_from(&[id]);
    snapshot
        .completed_steps
        .remove(&CalibrationStepId::VacuumSensorCharacterization);
    if let Some(values) = snapshot.values.as_object_mut() {
        let key = CalibrationStepId::VacuumSensorCharacterization.as_str();
        let remove_empty = values
            .get_mut(key)
            .and_then(Value::as_object_mut)
            .is_some_and(|reports| {
                reports.remove(&report.nozzle_id);
                reports.is_empty()
            });
        if remove_empty {
            values.remove(key);
        }
    }
    insert_artifact(
        state,
        snapshot,
        &definition,
        None,
        serde_json::to_value(report).map_err(|error| error.to_string())?,
    )?;
    Ok(())
}

pub fn record_vacuum_characterization_artifact(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    report: &VacuumSensorCharacterizationReport,
) -> Result<(), String> {
    reconcile_snapshot_artifacts(state, snapshot)?;
    let id = CalibrationArtifactId::vacuum_characterization(&report.nozzle_id)
        .ok_or_else(|| "vacuum characterization names an unknown nozzle".to_string())?;
    let definition = definition(id)?;
    if !definition
        .dependencies
        .iter()
        .all(|dependency| artifact_tree_is_current(state, snapshot, *dependency))
    {
        return Err(format!(
            "{} vacuum characterization depends on stale calibration artifacts",
            report.nozzle_id
        ));
    }
    snapshot.invalidate_artifacts_from(&[id]);
    insert_artifact(
        state,
        snapshot,
        &definition,
        None,
        serde_json::to_value(report).map_err(|error| error.to_string())?,
    )?;
    Ok(())
}

pub fn record_captured_artifact(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    id: CalibrationArtifactId,
    source_run_id: Option<Uuid>,
    value: Value,
) -> Result<Vec<CalibrationArtifactId>, String> {
    let definition = definition(id)?;
    if definition.kind == CalibrationArtifactKind::Derived {
        return Err(format!("{id:?} is calculated, not captured"));
    }
    if !definition
        .dependencies
        .iter()
        .all(|dependency| artifact_tree_is_current(state, snapshot, *dependency))
    {
        return Err(format!("{id:?} has a stale or missing dependency"));
    }
    snapshot.invalidate_artifacts_from(&[id]);
    insert_artifact(state, snapshot, &definition, source_run_id, value)?;
    recompute_stale_derived(state, snapshot, None)
}

pub fn artifact_tree_is_current(
    state: &AppState,
    snapshot: &CalibrationSnapshot,
    id: CalibrationArtifactId,
) -> bool {
    let definitions = definitions_by_id();
    artifact_tree_is_current_inner(state, snapshot, id, &definitions, &mut BTreeSet::new())
}

fn artifact_tree_is_current_inner(
    state: &AppState,
    snapshot: &CalibrationSnapshot,
    id: CalibrationArtifactId,
    definitions: &BTreeMap<CalibrationArtifactId, CalibrationArtifactDefinition>,
    visiting: &mut BTreeSet<CalibrationArtifactId>,
) -> bool {
    if !visiting.insert(id) {
        return false;
    }
    let current = definitions.get(&id).is_some_and(|definition| {
        definition.dependencies.iter().all(|dependency| {
            artifact_tree_is_current_inner(state, snapshot, *dependency, definitions, visiting)
        }) && artifact_is_current(state, snapshot, definition)
    });
    visiting.remove(&id);
    current
}

fn migrate_legacy_captures(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
) -> Result<(), String> {
    let reports = snapshot
        .values
        .get(CalibrationStepId::CameraReproducibility.as_str())
        .and_then(|value| value.get("reports"))
        .cloned();
    if let Some(reports) = reports {
        migrate_value(
            state,
            snapshot,
            CalibrationArtifactId::TopCameraReproducibility,
            source_run_id(snapshot, CalibrationStepId::CameraReproducibility),
            reports.get("top_camera").cloned(),
        )?;
        migrate_value(
            state,
            snapshot,
            CalibrationArtifactId::BottomCameraReproducibility,
            source_run_id(snapshot, CalibrationStepId::CameraReproducibility),
            reports.get("bottom_camera").cloned(),
        )?;
        for nozzle_id in ["N1", "N2"] {
            let id = if nozzle_id == "N1" {
                CalibrationArtifactId::ToolingN1
            } else {
                CalibrationArtifactId::ToolingN2
            };
            migrate_value(
                state,
                snapshot,
                id,
                source_run_id(snapshot, CalibrationStepId::CameraReproducibility),
                reports.get(format!("tooling_{nozzle_id}")).cloned(),
            )?;
        }
    }

    let visual = snapshot
        .values
        .get(CalibrationStepId::VisualHoming.as_str())
        .cloned();
    migrate_value(
        state,
        snapshot,
        CalibrationArtifactId::VisualHomingQualification,
        source_run_id(snapshot, CalibrationStepId::VisualHoming),
        visual,
    )?;

    let bottom_geometry = snapshot
        .values
        .get(CalibrationStepId::BottomCameraGeometry.as_str())
        .cloned();
    if let Some(bottom_geometry) = bottom_geometry {
        migrate_value(
            state,
            snapshot,
            CalibrationArtifactId::TopCameraWorkingPlaneMap,
            source_run_id(snapshot, CalibrationStepId::BottomCameraGeometry),
            bottom_geometry
                .get("top_camera_working_plane_mapping")
                .cloned(),
        )?;
        migrate_value(
            state,
            snapshot,
            CalibrationArtifactId::BottomCameraLocalMap,
            source_run_id(snapshot, CalibrationStepId::BottomCameraGeometry),
            bottom_geometry.get("mapping").cloned(),
        )?;
        let top_camera_lens = bottom_geometry.get("reciprocal_observation").cloned();
        let bottom_camera_lens = snapshot
            .values
            .get(CalibrationStepId::NozzleOffsets.as_str())
            .and_then(|value| value.get("top_camera_center"))
            .cloned();
        if let (Some(top_camera_lens), Some(bottom_camera_lens)) =
            (top_camera_lens, bottom_camera_lens)
        {
            migrate_value(
                state,
                snapshot,
                CalibrationArtifactId::ReciprocalCameraGeometry,
                source_run_id(snapshot, CalibrationStepId::BottomCameraGeometry),
                Some(json!({
                    "top_camera_lens": top_camera_lens,
                    "bottom_camera_lens": bottom_camera_lens,
                })),
            )?;
        }
    }

    let runout_step = snapshot
        .values
        .get(CalibrationStepId::NozzleRunout.as_str())
        .cloned();
    if let Some(runout_step) = runout_step {
        let reports = runout_step
            .get("reports")
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        for report in reports {
            let nozzle_id = report.get("nozzle_id").and_then(Value::as_str);
            if let Some(id) = nozzle_id.and_then(CalibrationArtifactId::nozzle_runout) {
                migrate_value(
                    state,
                    snapshot,
                    id,
                    source_run_id(snapshot, CalibrationStepId::NozzleRunout),
                    Some(report),
                )?;
            }
        }
    }

    let height_step = snapshot
        .values
        .get(CalibrationStepId::NozzleHeightModel.as_str())
        .cloned();
    if let Some(height_step) = height_step {
        let models = height_step
            .get("models")
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        for model in models {
            let nozzle_id = model.get("nozzle_id").and_then(Value::as_str);
            if let Some(id) = nozzle_id.and_then(CalibrationArtifactId::nozzle_height) {
                migrate_value(
                    state,
                    snapshot,
                    id,
                    source_run_id(snapshot, CalibrationStepId::NozzleHeightModel),
                    Some(model),
                )?;
            }
        }
    }
    Ok(())
}

fn migrate_legacy_vacuum_contacts(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
) -> Result<(), String> {
    for nozzle_id in ["N1", "N2"] {
        let Some(value) = snapshot
            .values
            .get("vacuum_probe")
            .and_then(|probes| probes.get(nozzle_id))
            .cloned()
        else {
            continue;
        };
        let id = CalibrationArtifactId::vacuum_contact(nozzle_id).unwrap();
        let replace_older_artifact = snapshot.artifacts.get(&id).is_some_and(|artifact| {
            artifact.algorithm_version != algorithm_version(id) && artifact.value == value
        });
        if replace_older_artifact {
            snapshot.artifacts.remove(&id);
        }
        if snapshot.artifacts.contains_key(&id) {
            continue;
        }
        let definition = definition(id)?;
        if definition
            .dependencies
            .iter()
            .all(|dependency| artifact_tree_is_current(state, snapshot, *dependency))
        {
            insert_artifact(state, snapshot, &definition, None, value)?;
        }
    }
    Ok(())
}

fn migrate_value(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    id: CalibrationArtifactId,
    source_run_id: Option<Uuid>,
    value: Option<Value>,
) -> Result<(), String> {
    if snapshot.artifacts.contains_key(&id) || value.is_none() {
        return Ok(());
    }
    let definition = definition(id)?;
    if !definition
        .dependencies
        .iter()
        .all(|dependency| snapshot.artifacts.contains_key(dependency))
    {
        return Ok(());
    }
    let value = json!({
        "migration": MIGRATION_VERSION,
        "result": value.unwrap(),
    });
    insert_artifact(state, snapshot, &definition, source_run_id, value)
}

fn recompute_stale_derived(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    requested: Option<BTreeSet<CalibrationArtifactId>>,
) -> Result<Vec<CalibrationArtifactId>, String> {
    let mut recomputed = Vec::new();
    for definition in lumen_calibration_artifacts()
        .into_iter()
        .filter(|definition| definition.kind == CalibrationArtifactKind::Derived)
    {
        if requested
            .as_ref()
            .is_some_and(|requested| !requested.contains(&definition.id))
        {
            continue;
        }
        if artifact_tree_is_current(state, snapshot, definition.id) {
            continue;
        }
        if !definition
            .dependencies
            .iter()
            .all(|dependency| artifact_tree_is_current(state, snapshot, *dependency))
        {
            if requested.is_some() {
                return Err(format!(
                    "cannot recompute {:?}: a source artifact requires recapture",
                    definition.id
                ));
            }
            continue;
        }
        let value = derive_value(state, snapshot, definition.id)?;
        insert_artifact(state, snapshot, &definition, None, value)?;
        recomputed.push(definition.id);
    }
    Ok(recomputed)
}

fn derive_value(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    id: CalibrationArtifactId,
) -> Result<Value, String> {
    use CalibrationArtifactId::*;
    match id {
        CameraRayTransfer => {
            let mapping: BottomCameraMappingReport =
                artifact_result(snapshot, TopCameraWorkingPlaneMap)?;
            let reciprocal = artifact_result_value(snapshot, ReciprocalCameraGeometry)?;
            let top_camera_lens: HeadFeatureCenterReport = serde_json::from_value(
                reciprocal
                    .get("top_camera_lens")
                    .cloned()
                    .ok_or_else(|| "reciprocal geometry has no top-camera lens".to_string())?,
            )
            .map_err(|error| error.to_string())?;
            let bottom_camera_lens: HeadFeatureCenterReport = serde_json::from_value(
                reciprocal
                    .get("bottom_camera_lens")
                    .cloned()
                    .ok_or_else(|| "reciprocal geometry has no bottom-camera lens".to_string())?,
            )
            .map_err(|error| error.to_string())?;
            let geometry = &state
                .profile
                .bottom_vision
                .as_ref()
                .ok_or_else(|| "active profile has no bottom vision".to_string())?
                .head_geometry;
            let report = calibrate_camera_ray_transfer(
                &top_camera_lens,
                &bottom_camera_lens,
                mapping.pixel_error_to_machine_mm,
                geometry.bottom_camera_reference_diameter_mm,
                geometry.camera_ray_transfer_max_mm,
            )
            .ok_or_else(|| "camera-ray transfer could not be derived".to_string())?;
            if !report.passed {
                return Err("derived camera-ray transfer exceeds profile bounds".into());
            }
            sync_legacy_camera_ray_transfer(snapshot, &report)?;
            serde_json::to_value(report).map_err(|error| error.to_string())
        }
        HeadNozzleOffsets => {
            let n1: NozzleRunoutReport = artifact_result(snapshot, NozzleAxisRunoutN1)?;
            let n2: NozzleRunoutReport = artifact_result(snapshot, NozzleAxisRunoutN2)?;
            Ok(json!({
                "nozzles": [
                    nozzle_offset_value(&n1),
                    nozzle_offset_value(&n2),
                ],
                "n1_to_n2_x_mm": n1.axis_controller_x_mm - n2.axis_controller_x_mm,
                "n1_to_n2_y_mm": n1.axis_controller_y_mm - n2.axis_controller_y_mm,
            }))
        }
        RuntimeTipTransformN1 | RuntimeTipTransformN2 => {
            let (runout, height) = if id == RuntimeTipTransformN1 {
                (NozzleAxisRunoutN1, NozzleHeightN1)
            } else {
                (NozzleAxisRunoutN2, NozzleHeightN2)
            };
            Ok(json!({
                "camera_ray_transfer": artifact_result_value(snapshot, CameraRayTransfer)?,
                "nozzle_runout": artifact_result_value(snapshot, runout)?,
                "nozzle_height": artifact_result_value(snapshot, height)?,
            }))
        }
        _ => Err(format!("{id:?} is not a derived artifact")),
    }
}

fn nozzle_offset_value(report: &NozzleRunoutReport) -> Value {
    json!({
        "nozzle_id": report.nozzle_id,
        "head_offset_x_mm": report.head_offset_x_mm,
        "head_offset_y_mm": report.head_offset_y_mm,
        "axis_controller_x_mm": report.axis_controller_x_mm,
        "axis_controller_y_mm": report.axis_controller_y_mm,
    })
}

fn sync_legacy_camera_ray_transfer(
    snapshot: &mut CalibrationSnapshot,
    report: &velocitypnp_domain::CameraRayTransferReport,
) -> Result<(), String> {
    let height = snapshot
        .values
        .get_mut(CalibrationStepId::NozzleHeightModel.as_str())
        .ok_or_else(|| "legacy nozzle-height step is missing".to_string())?;
    height["camera_ray_transfer"] =
        serde_json::to_value(report).map_err(|error| error.to_string())?;
    if let Some(models) = height.get_mut("models").and_then(Value::as_array_mut) {
        for model in models {
            let mut decoded: NozzleHeightModel =
                serde_json::from_value(model.clone()).map_err(|error| error.to_string())?;
            decoded.reference_transfer_correction_x_mm = report.transfer_correction_x_mm;
            decoded.reference_transfer_correction_y_mm = report.transfer_correction_y_mm;
            *model = serde_json::to_value(decoded).map_err(|error| error.to_string())?;
        }
    }
    Ok(())
}

fn visit_plan(
    state: &AppState,
    snapshot: &CalibrationSnapshot,
    id: CalibrationArtifactId,
    definitions: &BTreeMap<CalibrationArtifactId, CalibrationArtifactDefinition>,
    actions: &mut BTreeMap<CalibrationArtifactId, CalibrationPlanItem>,
) -> Result<(), String> {
    if actions.contains_key(&id) {
        return Ok(());
    }
    let definition = definitions
        .get(&id)
        .ok_or_else(|| format!("unknown calibration artifact {id:?}"))?;
    for dependency in &definition.dependencies {
        visit_plan(state, snapshot, *dependency, definitions, actions)?;
    }
    let dependencies_reusable = definition.dependencies.iter().all(|dependency| {
        actions
            .get(dependency)
            .is_some_and(|item| item.action == CalibrationPlanAction::Reuse)
    });
    let stale_reason = artifact_stale_reason(state, snapshot, definition);
    let current = dependencies_reusable && stale_reason.is_none();
    let (action, reason) = if current {
        (
            CalibrationPlanAction::Reuse,
            "artifact and provenance are current".to_string(),
        )
    } else if definition.kind == CalibrationArtifactKind::Derived {
        (
            CalibrationPlanAction::Recompute,
            stale_reason.unwrap_or_else(|| {
                "a source will change; recalculate without machine motion".into()
            }),
        )
    } else if definition.dependencies.iter().any(|dependency| {
        actions
            .get(dependency)
            .is_some_and(|item| item.action == CalibrationPlanAction::Blocked)
    }) {
        (
            CalibrationPlanAction::Blocked,
            "a required source artifact is unavailable".to_string(),
        )
    } else {
        (
            CalibrationPlanAction::Recapture,
            stale_reason.unwrap_or_else(|| "a dependency requires a new measurement".to_string()),
        )
    };
    actions.insert(
        id,
        CalibrationPlanItem {
            artifact: id,
            action,
            reason,
            requires_motion: definition.requires_motion,
            estimated_duration_seconds: if action == CalibrationPlanAction::Reuse {
                0
            } else {
                definition.estimated_duration_seconds
            },
        },
    );
    Ok(())
}

fn insert_artifact(
    state: &AppState,
    snapshot: &mut CalibrationSnapshot,
    definition: &CalibrationArtifactDefinition,
    source_run_id: Option<Uuid>,
    value: Value,
) -> Result<(), String> {
    let input_fingerprints = expected_inputs(state, &snapshot.profile_id, definition.id)?;
    let dependency_fingerprints = definition
        .dependencies
        .iter()
        .map(|dependency| {
            snapshot
                .artifacts
                .get(dependency)
                .map(|artifact| (*dependency, artifact.fingerprint.clone()))
                .ok_or_else(|| format!("missing dependency {dependency:?}"))
        })
        .collect::<Result<BTreeMap<_, _>, _>>()?;
    let algorithm_version = algorithm_version(definition.id).to_string();
    let fingerprint = artifact_fingerprint(
        definition.id,
        &algorithm_version,
        &input_fingerprints,
        &dependency_fingerprints,
        &value,
    )?;
    let artifact = normalize_artifact(
        definition.id,
        CalibrationArtifact {
            kind: definition.kind,
            recorded_at: Utc::now(),
            source_run_id,
            algorithm_version,
            input_fingerprints,
            dependency_fingerprints,
            fingerprint,
            value,
        },
    )?;
    snapshot.artifacts.insert(definition.id, artifact);
    Ok(())
}

fn normalize_artifact(
    id: CalibrationArtifactId,
    artifact: CalibrationArtifact,
) -> Result<CalibrationArtifact, String> {
    let bytes = serde_json::to_vec(&artifact).map_err(|error| error.to_string())?;
    let mut normalized: CalibrationArtifact =
        serde_json::from_slice(&bytes).map_err(|error| error.to_string())?;
    normalized.fingerprint = artifact_fingerprint(
        id,
        &normalized.algorithm_version,
        &normalized.input_fingerprints,
        &normalized.dependency_fingerprints,
        &normalized.value,
    )?;
    Ok(normalized)
}

fn artifact_is_current(
    state: &AppState,
    snapshot: &CalibrationSnapshot,
    definition: &CalibrationArtifactDefinition,
) -> bool {
    artifact_stale_reason(state, snapshot, definition).is_none()
}

fn artifact_stale_reason(
    state: &AppState,
    snapshot: &CalibrationSnapshot,
    definition: &CalibrationArtifactDefinition,
) -> Option<String> {
    let Some(artifact) = snapshot.artifacts.get(&definition.id) else {
        return Some("artifact is not stored".into());
    };
    if artifact.kind != definition.kind {
        return Some("artifact kind changed".into());
    }
    let algorithm = algorithm_version(definition.id);
    if artifact.algorithm_version != algorithm {
        return Some(format!(
            "algorithm changed from {} to {algorithm}",
            artifact.algorithm_version
        ));
    }
    let Ok(inputs) = expected_inputs(state, &snapshot.profile_id, definition.id) else {
        return Some("current input fingerprints could not be calculated".into());
    };
    if artifact.input_fingerprints != inputs {
        let changed = artifact
            .input_fingerprints
            .keys()
            .chain(inputs.keys())
            .filter(|key| artifact.input_fingerprints.get(*key) != inputs.get(*key))
            .cloned()
            .collect::<BTreeSet<_>>();
        return Some(format!(
            "input fingerprint changed: {}",
            changed.into_iter().collect::<Vec<_>>().join(", ")
        ));
    }
    let dependencies = definition
        .dependencies
        .iter()
        .filter_map(|dependency| {
            snapshot
                .artifacts
                .get(dependency)
                .map(|artifact| (*dependency, artifact.fingerprint.clone()))
        })
        .collect::<BTreeMap<_, _>>();
    if dependencies.len() != definition.dependencies.len() {
        return Some("a dependency artifact is missing".into());
    }
    if artifact.dependency_fingerprints != dependencies {
        return Some("a dependency fingerprint changed".into());
    }
    let Ok(fingerprint) = artifact_fingerprint(
        definition.id,
        algorithm,
        &inputs,
        &dependencies,
        &artifact.value,
    ) else {
        return Some("artifact fingerprint could not be calculated".into());
    };
    (artifact.fingerprint != fingerprint).then(|| {
        format!(
            "stored value fingerprint changed (stored {}, calculated {})",
            &artifact.fingerprint[..artifact.fingerprint.len().min(12)],
            &fingerprint[..fingerprint.len().min(12)]
        )
    })
}

fn expected_inputs(
    state: &AppState,
    profile_id: &str,
    id: CalibrationArtifactId,
) -> Result<BTreeMap<String, String>, String> {
    use CalibrationArtifactId::*;
    let mut inputs = BTreeMap::new();
    let top_profile = || camera_profile(state, profile_id, CameraRole::TopCamera);
    let bottom_profile = || camera_profile(state, profile_id, CameraRole::BottomCamera);
    let visual = || {
        state
            .profile
            .visual_homing
            .as_ref()
            .ok_or_else(|| "active profile has no visual homing".to_string())
    };
    let bottom = || {
        state
            .profile
            .bottom_vision
            .as_ref()
            .ok_or_else(|| "active profile has no bottom vision".to_string())
    };
    let nozzle = |nozzle_id: &str| {
        bottom()?
            .nozzles
            .iter()
            .find(|nozzle| nozzle.nozzle_id == nozzle_id)
            .ok_or_else(|| format!("active profile has no {nozzle_id} tooling"))
    };
    let add =
        |inputs: &mut BTreeMap<String, String>, name: &str, value: &dyn erased::Fingerprint| {
            inputs.insert(name.into(), value.fingerprint())
        };

    match id {
        TopCameraReproducibility => {
            add(&mut inputs, "top_camera_profile", &top_profile()?);
        }
        BottomCameraReproducibility => {
            add(&mut inputs, "bottom_camera_profile", &bottom_profile()?);
        }
        ToolingN1 | NozzleAxisRunoutN1 | NozzleHeightN1 => {
            add(&mut inputs, "bottom_camera_profile", &bottom_profile()?);
            add(&mut inputs, "tooling", &nozzle("N1")?);
        }
        ToolingN2 | NozzleAxisRunoutN2 | NozzleHeightN2 => {
            add(&mut inputs, "bottom_camera_profile", &bottom_profile()?);
            add(&mut inputs, "tooling", &nozzle("N2")?);
        }
        VisualHomingQualification | TopCameraWorkingPlaneMap => {
            add(&mut inputs, "top_camera_profile", &top_profile()?);
            add(&mut inputs, "visual_homing_profile", &visual()?);
        }
        LocalXyDirectionResponse => {
            add(&mut inputs, "top_camera_profile", &top_profile()?);
            add(&mut inputs, "visual_homing_profile", &visual()?);
            add(
                &mut inputs,
                "direction_response_profile",
                &state
                    .profile
                    .local_xy_direction_response
                    .as_ref()
                    .ok_or_else(|| {
                        "active profile has no local XY direction response".to_string()
                    })?,
            );
        }
        TopCameraPlanarModel => {
            add(&mut inputs, "top_camera_profile", &top_profile()?);
            add(&mut inputs, "visual_homing_profile", &visual()?);
            add(
                &mut inputs,
                "installed_geometry_profile",
                &state
                    .profile
                    .installed_geometry_calibration
                    .as_ref()
                    .ok_or_else(|| {
                        "active profile has no installed geometry calibration".to_string()
                    })?
                    .top_camera_sweep,
            );
        }
        BottomCameraLocalMap => {
            add(&mut inputs, "bottom_camera_profile", &bottom_profile()?);
            add(
                &mut inputs,
                "mapping_profile",
                &bottom()?.head_geometry.probe_distance_mm,
            );
        }
        BottomCameraPlanarModel => {
            add(&mut inputs, "bottom_camera_profile", &bottom_profile()?);
            add(&mut inputs, "tooling", &nozzle("N1")?);
            add(
                &mut inputs,
                "installed_geometry_profile",
                &state
                    .profile
                    .installed_geometry_calibration
                    .as_ref()
                    .ok_or_else(|| {
                        "active profile has no installed geometry calibration".to_string()
                    })?
                    .bottom_camera_sweep,
            );
        }
        InstalledPlateGeometry => {
            let mut geometry_profile = state
                .profile
                .installed_geometry_calibration
                .as_ref()
                .ok_or_else(|| "active profile has no installed geometry calibration".to_string())?
                .clone();
            // Plane depth converts an existing optical map into a physical Z
            // value. It does not change how the plate geometry was captured.
            geometry_profile.installed_plate_depth_below_working_plane_mm = None;
            add(&mut inputs, "installed_geometry_profile", &geometry_profile);
        }
        ReciprocalCameraGeometry => {
            add(&mut inputs, "top_camera_profile", &top_profile()?);
            add(&mut inputs, "bottom_camera_profile", &bottom_profile()?);
            add(
                &mut inputs,
                "reciprocal_geometry_profile",
                &bottom()?.head_geometry,
            );
        }
        CameraRayTransfer => {
            add(
                &mut inputs,
                "camera_ray_profile",
                &json!({
                    "bottom_camera_reference_diameter_mm": bottom()?.head_geometry.bottom_camera_reference_diameter_mm,
                    "camera_ray_transfer_max_mm": bottom()?.head_geometry.camera_ray_transfer_max_mm,
                }),
            );
        }
        HeadNozzleOffsets | RuntimeTipTransformN1 | RuntimeTipTransformN2 => {}
        VacuumContactN1 | VacuumContactN2 | VacuumCharacterizationN1 | VacuumCharacterizationN2 => {
            let nozzle_id = if matches!(id, VacuumContactN1 | VacuumCharacterizationN1) {
                "N1"
            } else {
                "N2"
            };
            add(
                &mut inputs,
                "vacuum_profile",
                &state
                    .profile
                    .vacuum_calibration
                    .as_ref()
                    .ok_or_else(|| "active profile has no vacuum calibration".to_string())?,
            );
            add(&mut inputs, "tooling", &nozzle(nozzle_id)?);
            let sensor_id = if nozzle_id == "N1" {
                "vacuum_1"
            } else {
                "vacuum_2"
            };
            let sensor = state
                .profile
                .sensors
                .iter()
                .find(|sensor| sensor.id == sensor_id)
                .ok_or_else(|| format!("active profile has no {sensor_id} sensor"))?;
            add(&mut inputs, "sensor", &sensor);
            if matches!(id, VacuumCharacterizationN1 | VacuumCharacterizationN2) {
                add(
                    &mut inputs,
                    "characterization_profile",
                    &state
                        .profile
                        .vacuum_characterization
                        .as_ref()
                        .ok_or_else(|| {
                            "active profile has no vacuum characterization".to_string()
                        })?,
                );
            }
        }
    }
    if matches!(id, NozzleHeightN1 | NozzleHeightN2) {
        add(
            &mut inputs,
            "z_kinematics",
            &state
                .profile
                .z_kinematics
                .as_ref()
                .ok_or_else(|| "active profile has no Z kinematics".to_string())?,
        );
        add(
            &mut inputs,
            "height_profile",
            &json!({
                "samples": bottom()?.head_geometry.height_sample_nozzle_z_mm,
                "max_slope": bottom()?.head_geometry.height_max_axis_slope_mm_per_mm,
                "max_extrapolation": bottom()?.head_geometry.height_max_extrapolation_mm,
            }),
        );
    }
    if matches!(id, NozzleAxisRunoutN1 | NozzleAxisRunoutN2) {
        add(
            &mut inputs,
            "runout_profile",
            &json!({
                "angles": bottom()?.head_geometry.rotation_angle_offsets_deg,
                "fit_limit": bottom()?.head_geometry.runout_fit_p95_limit_mm,
            }),
        );
    }
    Ok(inputs)
}

mod erased {
    use super::*;

    pub trait Fingerprint {
        fn fingerprint(&self) -> String;
    }

    impl<T: Serialize> Fingerprint for T {
        fn fingerprint(&self) -> String {
            digest(self).expect("serializable calibration input")
        }
    }
}

fn camera_profile(
    state: &AppState,
    profile_id: &str,
    role: CameraRole,
) -> Result<CameraProfile, String> {
    if let Some(profile) = state
        .store
        .get_camera_profile(profile_id, role)
        .map_err(|error| error.to_string())?
    {
        return Ok(profile);
    }
    if profile_id == state.profile.id {
        for legacy_profile_id in &state.profile.legacy_profile_ids {
            if let Some(profile) = state
                .store
                .get_camera_profile(legacy_profile_id, role)
                .map_err(|error| error.to_string())?
            {
                return Ok(profile);
            }
        }
    }
    Ok(CameraProfile::initial_lumenpnp(profile_id, role))
}

fn artifact_result<T: serde::de::DeserializeOwned>(
    snapshot: &CalibrationSnapshot,
    id: CalibrationArtifactId,
) -> Result<T, String> {
    serde_json::from_value(artifact_result_value(snapshot, id)?)
        .map_err(|error| format!("invalid {id:?} artifact: {error}"))
}

pub fn artifact_result_value(
    snapshot: &CalibrationSnapshot,
    id: CalibrationArtifactId,
) -> Result<Value, String> {
    let value = snapshot
        .artifacts
        .get(&id)
        .ok_or_else(|| format!("missing {id:?} artifact"))?
        .value
        .clone();
    Ok(value.get("result").cloned().unwrap_or(value))
}

fn source_run_id(snapshot: &CalibrationSnapshot, step: CalibrationStepId) -> Option<Uuid> {
    snapshot
        .values
        .get(step.as_str())
        .and_then(|value| value.get("run_id"))
        .and_then(Value::as_str)
        .and_then(|value| Uuid::parse_str(value).ok())
}

fn definition(id: CalibrationArtifactId) -> Result<CalibrationArtifactDefinition, String> {
    lumen_calibration_artifacts()
        .into_iter()
        .find(|definition| definition.id == id)
        .ok_or_else(|| format!("missing definition for {id:?}"))
}

fn definitions_by_id() -> BTreeMap<CalibrationArtifactId, CalibrationArtifactDefinition> {
    lumen_calibration_artifacts()
        .into_iter()
        .map(|definition| (definition.id, definition))
        .collect()
}

fn algorithm_version(id: CalibrationArtifactId) -> &'static str {
    use CalibrationArtifactId::*;
    match id {
        TopCameraReproducibility => "camera_reproducibility.v1",
        BottomCameraReproducibility => "camera_reproducibility.v1",
        ToolingN1 | ToolingN2 => "nozzle_tooling.v1",
        VisualHomingQualification => "visual_homing_qualification.v2",
        TopCameraWorkingPlaneMap | BottomCameraLocalMap => "camera_motion_mapping.v1",
        TopCameraPlanarModel | BottomCameraPlanarModel => "planar_camera_model.v1",
        InstalledPlateGeometry => "installed_plate_geometry.v1",
        LocalXyDirectionResponse => "local_xy_direction_response.v2",
        ReciprocalCameraGeometry => "reciprocal_camera_geometry.v2",
        CameraRayTransfer => "camera_ray_transfer.v2",
        NozzleAxisRunoutN1 | NozzleAxisRunoutN2 => "nozzle_axis_runout.v1",
        HeadNozzleOffsets => "head_nozzle_offsets.v2",
        NozzleHeightN1 | NozzleHeightN2 => "nozzle_height_axis.v2",
        RuntimeTipTransformN1 | RuntimeTipTransformN2 => "runtime_tip_transform.v1",
        VacuumContactN1 | VacuumContactN2 => "vacuum_contact_probe.v3",
        VacuumCharacterizationN1 | VacuumCharacterizationN2 => "vacuum_sensor_characterization.v1",
    }
}

fn artifact_fingerprint(
    id: CalibrationArtifactId,
    algorithm_version: &str,
    input_fingerprints: &BTreeMap<String, String>,
    dependency_fingerprints: &BTreeMap<CalibrationArtifactId, String>,
    value: &Value,
) -> Result<String, String> {
    digest(&json!({
        "id": id,
        "algorithm_version": algorithm_version,
        "input_fingerprints": input_fingerprints,
        "dependency_fingerprints": dependency_fingerprints,
        "value": value,
    }))
}

fn digest<T: Serialize + ?Sized>(value: &T) -> Result<String, String> {
    let bytes = serde_json::to_vec(value).map_err(|error| error.to_string())?;
    let digest = Sha256::digest(bytes);
    Ok(digest.iter().map(|byte| format!("{byte:02x}")).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use velocitypnp_domain::{CalibrationArtifactKind, MachineProfile};
    use velocitypnp_driver::{MachineHandle, SimulatorDriver};
    use velocitypnp_store::Store;

    fn state() -> AppState {
        let profile: MachineProfile =
            serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.2.json")).unwrap();
        let machine = MachineHandle::spawn(
            Box::new(SimulatorDriver::new(profile.bounds, profile.safe_z_mm)),
            profile.clone(),
        );
        AppState::new(machine, profile, Store::memory().unwrap())
    }

    #[test]
    fn dependency_graph_keeps_nozzle_branches_independent() {
        let definitions = definitions_by_id();
        let local = &definitions[&CalibrationArtifactId::LocalXyDirectionResponse];
        assert_eq!(
            local.dependencies,
            [CalibrationArtifactId::TopCameraWorkingPlaneMap]
        );
        let n1 = &definitions[&CalibrationArtifactId::NozzleHeightN1];
        let n2 = &definitions[&CalibrationArtifactId::NozzleHeightN2];
        assert!(n1
            .dependencies
            .contains(&CalibrationArtifactId::NozzleAxisRunoutN1));
        assert!(!n1
            .dependencies
            .contains(&CalibrationArtifactId::NozzleAxisRunoutN2));
        assert!(n2
            .dependencies
            .contains(&CalibrationArtifactId::NozzleAxisRunoutN2));
        assert!(!n2
            .dependencies
            .contains(&CalibrationArtifactId::NozzleAxisRunoutN1));
        let vacuum_n1 = &definitions[&CalibrationArtifactId::VacuumCharacterizationN1];
        let vacuum_n2 = &definitions[&CalibrationArtifactId::VacuumCharacterizationN2];
        assert_eq!(
            vacuum_n1.dependencies,
            [CalibrationArtifactId::VacuumContactN1]
        );
        assert_eq!(
            vacuum_n2.dependencies,
            [CalibrationArtifactId::VacuumContactN2]
        );
    }

    #[test]
    fn ray_transfer_is_a_zero_motion_derived_artifact() {
        let definition = definition(CalibrationArtifactId::CameraRayTransfer).unwrap();
        assert_eq!(definition.kind, CalibrationArtifactKind::Derived);
        assert!(!definition.requires_motion);
        assert_eq!(definition.estimated_duration_seconds, 0);
    }

    #[test]
    fn artifact_fingerprint_survives_json_round_trip() {
        let id = CalibrationArtifactId::VacuumContactN1;
        let artifact = normalize_artifact(
            id,
            CalibrationArtifact {
                kind: CalibrationArtifactKind::Captured,
                recorded_at: Utc::now(),
                source_run_id: None,
                algorithm_version: algorithm_version(id).into(),
                input_fingerprints: BTreeMap::from([("sensor".into(), "abc".into())]),
                dependency_fingerprints: BTreeMap::from([(
                    CalibrationArtifactId::RuntimeTipTransformN1,
                    "def".into(),
                )]),
                fingerprint: String::new(),
                value: json!({
                    "contact_nozzle_z_mm": 3.200000000000003,
                    "signal_delta_counts": 10.0,
                    "observed_at": Utc::now(),
                }),
            },
        )
        .unwrap();
        let stored: CalibrationArtifact =
            serde_json::from_str(&serde_json::to_string(&artifact).unwrap()).unwrap();
        let calculated = artifact_fingerprint(
            id,
            &stored.algorithm_version,
            &stored.input_fingerprints,
            &stored.dependency_fingerprints,
            &stored.value,
        )
        .unwrap();
        assert_eq!(stored.fingerprint, calculated);
    }

    #[tokio::test]
    async fn fingerprints_change_with_relevant_profile_input() {
        let state = state();
        let before = expected_inputs(
            &state,
            &state.profile.id,
            CalibrationArtifactId::CameraRayTransfer,
        )
        .unwrap();
        let mut changed = (*state.profile).clone();
        changed
            .bottom_vision
            .as_mut()
            .unwrap()
            .head_geometry
            .bottom_camera_reference_diameter_mm += 1.0;
        let changed_state = AppState::new(state.machine.clone(), changed, state.store.clone());
        let after = expected_inputs(
            &changed_state,
            &changed_state.profile.id,
            CalibrationArtifactId::CameraRayTransfer,
        )
        .unwrap();
        assert_ne!(before, after);
    }

    #[tokio::test]
    async fn legacy_profile_id_keeps_the_existing_camera_fingerprint() {
        let state = state();
        let inputs = expected_inputs(
            &state,
            "lumenpnp-v3.0.4",
            CalibrationArtifactId::TopCameraReproducibility,
        )
        .unwrap();
        assert_eq!(
            inputs.get("top_camera_profile").map(String::as_str),
            Some("1834e6e1608819495cab353bb0c9916dcb081b4f48808a0e56dbdccce8b20831"),
        );
    }

    #[tokio::test]
    async fn plane_depth_does_not_invalidate_installed_plate_geometry() {
        let state = state();
        let before = expected_inputs(
            &state,
            &state.profile.id,
            CalibrationArtifactId::InstalledPlateGeometry,
        )
        .unwrap();
        let mut changed = (*state.profile).clone();
        changed
            .installed_geometry_calibration
            .as_mut()
            .unwrap()
            .installed_plate_depth_below_working_plane_mm = Some(2.0);
        let changed_state = AppState::new(state.machine.clone(), changed, state.store.clone());
        let after = expected_inputs(
            &changed_state,
            &changed_state.profile.id,
            CalibrationArtifactId::InstalledPlateGeometry,
        )
        .unwrap();
        assert_eq!(before, after);
    }
}
