use crate::Residuals;
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationStepId {
    CameraReproducibility,
    TopCameraIntrinsics,
    CoarseHoming,
    VisualHoming,
    MotionGeometry,
    TopCameraScale,
    NozzleOffsets,
    BottomCameraGeometry,
    NozzleRunout,
    ZPlanes,
    VacuumThresholds,
    Validation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationStepDefinition {
    pub id: CalibrationStepId,
    pub title: String,
    pub description: String,
    pub dependencies: Vec<CalibrationStepId>,
    pub requires_motion: bool,
    pub requires_fixture: bool,
}

pub fn lumen_calibration_workflow() -> Vec<CalibrationStepDefinition> {
    use CalibrationStepId::*;
    vec![
        step(CoarseHoming, "Coarse hardware homing", "Establish controller coordinates using the profile-specific homing method.", &[], true, false),
        step(CameraReproducibility, "Camera reproducibility", "After coarse homing, move the top camera over the 1 mm datum fiducial and nozzle N1 over the bottom camera; then lock resolution, exposure, white balance, focus, ring lighting, and frame settling.", &[CoarseHoming], true, false),
        step(VisualHoming, "Visual homing", "Refine the coarse home against the fixed 1 mm datum fiducial at the center of the Opulo logo.", &[CoarseHoming, CameraReproducibility], true, false),
        step(TopCameraIntrinsics, "Top camera intrinsics", "Measure lens distortion and camera matrix from a known target after visual homing.", &[VisualHoming], true, true),
        step(MotionGeometry, "Motion geometry", "Measure scale, skew, backlash, and direction-dependent residuals over the work envelope.", &[VisualHoming, TopCameraIntrinsics], true, true),
        step(TopCameraScale, "Top camera working-plane scale", "Relate pixels to corrected machine coordinates at calibrated Z planes.", &[MotionGeometry], true, true),
        step(NozzleOffsets, "Nozzle offsets", "Measure each nozzle relative to the down-looking camera.", &[TopCameraScale], true, false),
        step(BottomCameraGeometry, "Bottom camera geometry", "Locate and calibrate the up-looking camera.", &[NozzleOffsets], true, true),
        step(NozzleRunout, "Nozzle runout", "Measure nozzle center versus rotation and fit a compensation curve.", &[BottomCameraGeometry], true, false),
        step(ZPlanes, "Z planes", "Capture safe, board, feeder, discard, and camera-clearance planes.", &[NozzleOffsets], true, false),
        step(VacuumThresholds, "Vacuum thresholds", "Learn empty and known-part pressure distributions per nozzle and tip.", &[ZPlanes], true, false),
        step(Validation, "Known-board validation", "Measure homing, geometry, pick, alignment, and placement acceptance metrics.", &[NozzleRunout, VacuumThresholds], true, true),
    ]
}

fn step(
    id: CalibrationStepId,
    title: &str,
    description: &str,
    dependencies: &[CalibrationStepId],
    requires_motion: bool,
    requires_fixture: bool,
) -> CalibrationStepDefinition {
    CalibrationStepDefinition {
        id,
        title: title.into(),
        description: description.into(),
        dependencies: dependencies.to_vec(),
        requires_motion,
        requires_fixture,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationSnapshot {
    pub id: Uuid,
    pub machine_serial: String,
    pub profile_id: String,
    pub created_at: DateTime<Utc>,
    pub completed_steps: BTreeSet<CalibrationStepId>,
    pub residuals: BTreeMap<CalibrationStepId, Residuals>,
    pub values: serde_json::Value,
    pub parent_id: Option<Uuid>,
    pub production_ready: bool,
}

impl CalibrationSnapshot {
    pub fn next_available_steps(&self) -> Vec<CalibrationStepDefinition> {
        lumen_calibration_workflow()
            .into_iter()
            .filter(|step| {
                !self.completed_steps.contains(&step.id)
                    && step
                        .dependencies
                        .iter()
                        .all(|id| self.completed_steps.contains(id))
            })
            .collect()
    }

    pub fn invalidate_from(&mut self, changed: CalibrationStepId) {
        let definitions = lumen_calibration_workflow();
        let mut invalid = BTreeSet::from([changed]);
        loop {
            let before = invalid.len();
            for definition in &definitions {
                if definition
                    .dependencies
                    .iter()
                    .any(|dependency| invalid.contains(dependency))
                {
                    invalid.insert(definition.id);
                }
            }
            if invalid.len() == before {
                break;
            }
        }
        self.completed_steps.retain(|step| !invalid.contains(step));
        self.residuals.retain(|step, _| !invalid.contains(step));
        self.production_ready = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_change_invalidates_downstream_but_not_homing() {
        let mut snapshot = CalibrationSnapshot {
            id: Uuid::new_v4(),
            machine_serial: "test".into(),
            profile_id: "test".into(),
            created_at: Utc::now(),
            completed_steps: lumen_calibration_workflow()
                .into_iter()
                .map(|s| s.id)
                .collect(),
            residuals: BTreeMap::new(),
            values: serde_json::json!({}),
            parent_id: None,
            production_ready: true,
        };
        snapshot.invalidate_from(CalibrationStepId::CameraReproducibility);
        assert!(snapshot
            .completed_steps
            .contains(&CalibrationStepId::CoarseHoming));
        assert!(!snapshot
            .completed_steps
            .contains(&CalibrationStepId::TopCameraIntrinsics));
        assert!(!snapshot.production_ready);
    }

    #[test]
    fn coarse_homing_precedes_working_plane_camera_setup() {
        let workflow = lumen_calibration_workflow();
        assert_eq!(workflow[0].id, CalibrationStepId::CoarseHoming);
        let camera = workflow
            .iter()
            .find(|step| step.id == CalibrationStepId::CameraReproducibility)
            .unwrap();
        assert_eq!(camera.dependencies, [CalibrationStepId::CoarseHoming]);
        assert!(camera.requires_motion);
    }
}
