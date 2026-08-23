//! Deterministic, conservative job preflight and placement planning.

mod motion;

pub use motion::*;

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};
use uuid::Uuid;
use velocitypnp_domain::{BoardDesign, BoardSide, Feeder, Job, MachineProfile, Placement, Pose};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PreflightReport {
    pub ready: bool,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
    pub placement_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlacementPlan {
    pub job_id: Uuid,
    pub steps: Vec<PlanStep>,
    pub report: PreflightReport,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanStep {
    pub board_instance_id: Uuid,
    pub placement_reference: String,
    pub part_key: String,
    pub feeder_id: Uuid,
    pub nozzle_id: String,
    pub feeder_pick: Pose,
    pub board_place: Pose,
    pub safe_z_mm: f64,
}

pub fn plan_job(
    job: &Job,
    boards: &HashMap<Uuid, BoardDesign>,
    feeders: &[Feeder],
    profile: &MachineProfile,
) -> PlacementPlan {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();
    let mut steps = Vec::new();
    let mut feeder_uses = BTreeMap::<Uuid, u32>::new();

    for instance in &job.boards {
        let Some(board) = boards.get(&instance.board_id) else {
            errors.push(format!("{} references an unknown board", instance.label));
            continue;
        };
        if instance.design_to_machine.is_none() {
            errors.push(format!(
                "{} has no fiducial-derived transform",
                instance.label
            ));
        }
        if instance.surface_z_mm.is_none() {
            errors.push(format!(
                "{} has no measured board surface Z",
                instance.label
            ));
        }
        for placement in board.placements.iter().filter(|placement| {
            placement.enabled && !placement.fiducial && placement.side == instance.side
        }) {
            let part_key = placement_part_key(placement);
            let Some(feeder) = feeders.iter().find(|feeder| feeder.part_key() == part_key) else {
                errors.push(format!(
                    "{} has no feeder for {part_key}",
                    placement.reference
                ));
                continue;
            };
            let Some(height_mm) = placement
                .properties
                .get("Height")
                .and_then(|height| height.parse::<f64>().ok())
                .filter(|height| height.is_finite() && *height > 0.0)
            else {
                errors.push(format!("{} has no valid part height", placement.reference));
                continue;
            };
            let Some(transform) = instance.design_to_machine else {
                continue;
            };
            let Some(surface_z_mm) = instance.surface_z_mm else {
                continue;
            };
            let board_place = transformed_pose(placement.pose, transform, surface_z_mm + height_mm);
            if !profile.bounds.contains(Pose {
                z_mm: profile.safe_z_mm,
                ..board_place
            }) {
                errors.push(format!(
                    "{} is outside the configured machine envelope",
                    placement.reference
                ));
                continue;
            }
            let uses = feeder_uses.entry(feeder.id()).or_default();
            let feeder_pick = feeder_pick_pose(feeder, *uses);
            *uses += 1;
            steps.push(PlanStep {
                board_instance_id: instance.id,
                placement_reference: placement.reference.clone(),
                part_key,
                feeder_id: feeder.id(),
                nozzle_id: "N1".into(),
                feeder_pick,
                board_place,
                safe_z_mm: profile.safe_z_mm,
            });
        }
        if instance.side == BoardSide::Bottom {
            warnings.push(format!(
                "{} requires the guided board-flip and fresh fiducial alignment",
                instance.label
            ));
        }
    }
    if steps.is_empty() && errors.is_empty() {
        warnings.push("The job has no enabled placements for the selected board sides".into());
    }
    let report = PreflightReport {
        ready: errors.is_empty() && !steps.is_empty(),
        errors,
        warnings,
        placement_count: steps.len(),
    };
    PlacementPlan {
        job_id: job.id,
        steps,
        report,
    }
}

pub fn placement_part_key(placement: &Placement) -> String {
    placement
        .properties
        .get("MPN")
        .filter(|value| !value.trim().is_empty())
        .cloned()
        .unwrap_or_else(|| format!("{}:{}", placement.package, placement.value))
}

fn transformed_pose(design: Pose, transform: velocitypnp_domain::Transform2d, z_mm: f64) -> Pose {
    let (x_mm, y_mm) = transform.apply(design.x_mm, design.y_mm);
    let angle = design.rotation_deg.to_radians();
    let x_axis = transform.matrix[0] * angle.cos() + transform.matrix[1] * angle.sin();
    let y_axis = transform.matrix[3] * angle.cos() + transform.matrix[4] * angle.sin();
    Pose {
        x_mm,
        y_mm,
        z_mm,
        rotation_deg: y_axis.atan2(x_axis).to_degrees(),
    }
}

fn feeder_pick_pose(feeder: &Feeder, use_index: u32) -> Pose {
    match feeder {
        Feeder::Strip {
            first_pick,
            tape_pitch_mm,
            tape_rotation_deg,
            ..
        } => {
            let radians = tape_rotation_deg.to_radians();
            let distance = f64::from(use_index) * tape_pitch_mm;
            Pose {
                x_mm: first_pick.x_mm + distance * radians.cos(),
                y_mm: first_pick.y_mm + distance * radians.sin(),
                ..*first_pick
            }
        }
        Feeder::Drag { pick, .. } | Feeder::Photon { pick, .. } => *pick,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    use velocitypnp_domain::{BoardInstance, Placement, Transform2d};

    fn profile() -> MachineProfile {
        serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.4.json")).unwrap()
    }

    #[test]
    fn plan_requires_height_feeder_transform_and_surface() {
        let board_id = Uuid::new_v4();
        let board = BoardDesign {
            id: board_id,
            name: "board".into(),
            source_format: "kicad_pcb".into(),
            source_version: None,
            width_mm: Some(20.0),
            height_mm: Some(10.0),
            outline_bounds: None,
            placements: vec![Placement {
                reference: "R1".into(),
                value: "10k".into(),
                package: "R0402".into(),
                side: BoardSide::Top,
                pose: Pose {
                    x_mm: 2.0,
                    y_mm: 3.0,
                    ..Pose::ORIGIN
                },
                enabled: true,
                fiducial: false,
                properties: BTreeMap::new(),
            }],
            warnings: vec![],
        };
        let job = Job {
            id: Uuid::new_v4(),
            name: "job".into(),
            status: "draft".into(),
            boards: vec![BoardInstance {
                id: Uuid::new_v4(),
                board_id,
                label: "A1".into(),
                side: BoardSide::Top,
                design_to_machine: None,
                surface_z_mm: None,
            }],
        };
        let plan = plan_job(&job, &HashMap::from([(board_id, board)]), &[], &profile());
        assert!(!plan.report.ready);
        assert_eq!(plan.report.errors.len(), 3);
    }

    #[test]
    fn strip_pick_advances_deterministically() {
        let feeder = Feeder::Strip {
            id: Uuid::new_v4(),
            name: "strip".into(),
            part_key: "part".into(),
            first_pick: Pose {
                x_mm: 10.0,
                y_mm: 20.0,
                z_mm: 5.0,
                rotation_deg: 0.0,
            },
            tape_pitch_mm: 4.0,
            tape_width_mm: 8.0,
            tape_rotation_deg: 90.0,
            remaining: None,
        };
        let second = feeder_pick_pose(&feeder, 1);
        assert!((second.x_mm - 10.0).abs() < 1e-9);
        assert!((second.y_mm - 24.0).abs() < 1e-9);
    }

    #[test]
    fn transform_rotates_placement_orientation() {
        let transformed = transformed_pose(
            Pose {
                x_mm: 1.0,
                y_mm: 0.0,
                z_mm: 0.0,
                rotation_deg: 45.0,
            },
            Transform2d::rigid(10.0, 20.0, 90.0),
            5.0,
        );
        assert!((transformed.x_mm - 10.0).abs() < 1e-9);
        assert!((transformed.y_mm - 21.0).abs() < 1e-9);
        assert!((transformed.rotation_deg - 135.0).abs() < 1e-9);
    }
}
