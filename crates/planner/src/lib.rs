//! Deterministic, conservative job preflight and placement planning.

mod motion;

pub use motion::*;

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};
use uuid::Uuid;
use velocitypnp_domain::{
    top_side_placement_rotation_deg, BoardDesign, BoardSide, Feeder, Job, MachineProfile, Part,
    PartOrientation, Placement, PlacementReleaseSchedule, Pose,
};

const MAXIMUM_PLACEMENT_RELEASE_ATTEMPTS: usize = 10;

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
    pub part_height_mm: f64,
    pub body_width_mm: f64,
    pub body_length_mm: f64,
    pub part_orientation: PartOrientation,
    pub feeder_part_rotation_deg: f64,
    pub placement_surface_offset_mm: f64,
    pub placement_seating_offset_mm: f64,
    pub placement_release_seating_offsets_mm: Vec<f64>,
    pub feeder_pick: Pose,
    pub board_place: Pose,
    pub safe_z_mm: f64,
}

pub fn plan_job(
    job: &Job,
    boards: &HashMap<Uuid, BoardDesign>,
    feeders: &[Feeder],
    parts: &HashMap<String, Part>,
    profile: &MachineProfile,
) -> PlacementPlan {
    let mut errors = Vec::new();
    let mut warnings = Vec::new();
    let mut steps = Vec::new();
    let mut feeder_uses = BTreeMap::<Uuid, u32>::new();
    let selected_references = job
        .placement_references
        .iter()
        .map(String::as_str)
        .collect::<std::collections::BTreeSet<_>>();

    if selected_references.len() != job.placement_references.len() {
        errors.push("job placement references must be unique".into());
    }
    if job
        .placement_seating_offset_mm
        .is_some_and(|offset| !offset.is_finite() || !(-0.25..=0.5).contains(&offset))
    {
        errors.push("job placement seating offset is invalid".into());
    }
    if job
        .placement_release
        .is_some_and(|schedule| schedule.validate().is_err())
    {
        errors.push("job placement release schedule is invalid".into());
    }

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
        if !instance.placement_surface_offset_mm.is_finite()
            || !(0.0..=5.0).contains(&instance.placement_surface_offset_mm)
        {
            errors.push(format!(
                "{} has an invalid placement surface offset",
                instance.label
            ));
        }
        if !selected_references.is_empty() {
            for reference in &selected_references {
                if !board.placements.iter().any(|placement| {
                    placement.reference == *reference
                        && placement.enabled
                        && !placement.fiducial
                        && placement.side == instance.side
                }) {
                    errors.push(format!(
                        "{} does not have enabled placement {reference} on the selected side",
                        instance.label
                    ));
                }
            }
        }
        for placement in board.placements.iter().filter(|placement| {
            placement.enabled
                && !placement.fiducial
                && placement.side == instance.side
                && (selected_references.is_empty()
                    || selected_references.contains(placement.reference.as_str()))
        }) {
            let part_key = placement_part_key(placement);
            let Some(feeder) = feeders.iter().find(|feeder| feeder.part_key() == part_key) else {
                errors.push(format!(
                    "{} has no feeder for {part_key}",
                    placement.reference
                ));
                continue;
            };
            let Some(part) = parts.get(&part_key) else {
                errors.push(format!(
                    "{} has no part-library definition for {part_key}",
                    placement.reference
                ));
                continue;
            };
            let Some(part_orientation) = part.resolved_orientation() else {
                errors.push(format!(
                    "{} has no orientation-aware vision policy",
                    placement.reference
                ));
                continue;
            };
            if let Err(error) = part_orientation.validate() {
                errors.push(format!("{} {error}", placement.reference));
                continue;
            }
            let height_mm = placement
                .properties
                .get("Height")
                .and_then(|height| height.parse::<f64>().ok())
                .filter(|height| height.is_finite() && *height > 0.0)
                .or(part.height_mm);
            let Some(height_mm) = height_mm.filter(|height| height.is_finite() && *height > 0.0)
            else {
                errors.push(format!("{} has no valid part height", placement.reference));
                continue;
            };
            let body_width_mm = part
                .body_width_mm
                .filter(|width| width.is_finite() && *width > 0.0);
            let body_length_mm = part
                .body_length_mm
                .filter(|length| length.is_finite() && *length > 0.0);
            let (Some(body_width_mm), Some(body_length_mm)) = (body_width_mm, body_length_mm)
            else {
                errors.push(format!(
                    "{} has no valid package body dimensions",
                    placement.reference
                ));
                continue;
            };
            let (body_width_mm, body_length_mm) = if body_width_mm <= body_length_mm {
                (body_width_mm, body_length_mm)
            } else {
                (body_length_mm, body_width_mm)
            };
            let seating_property = placement.properties.get("Placement Seating Offset");
            let reference_seating_offset_mm = if let Some(value) = seating_property {
                let Ok(offset) = value.parse::<f64>() else {
                    errors.push(format!(
                        "{} has an invalid placement seating offset",
                        placement.reference
                    ));
                    continue;
                };
                Some(offset)
            } else {
                None
            };
            let release_schedule = part.placement_release.or(job.placement_release);
            let placement_seating_offset_mm = reference_seating_offset_mm.unwrap_or_else(|| {
                release_schedule.map_or_else(
                    || {
                        job.placement_seating_offset_mm
                            .unwrap_or_else(|| part.placement_seating_offset_mm.unwrap_or(0.0))
                    },
                    |schedule| schedule.start_seating_offset_mm,
                )
            });
            if !placement_seating_offset_mm.is_finite()
                || !(-0.5..=0.5).contains(&placement_seating_offset_mm)
            {
                errors.push(format!(
                    "{} has an invalid placement seating offset",
                    placement.reference
                ));
                continue;
            }
            let placement_release_seating_offsets_mm = match release_schedule {
                Some(mut schedule) => {
                    schedule.start_seating_offset_mm = placement_seating_offset_mm;
                    match expand_placement_release_schedule(schedule) {
                        Ok(offsets) => offsets,
                        Err(error) => {
                            errors.push(format!("{} {error}", placement.reference));
                            continue;
                        }
                    }
                }
                None => vec![placement_seating_offset_mm],
            };
            let nozzle_id = select_compatible_nozzle(part, profile).unwrap_or_else(|| "N1".into());
            let Some(transform) = instance.design_to_machine else {
                continue;
            };
            let Some(surface_z_mm) = instance.surface_z_mm else {
                continue;
            };
            let board_place = transformed_pose(
                placement.pose,
                transform,
                surface_z_mm
                    + instance.placement_surface_offset_mm
                    + height_mm
                    + placement_seating_offset_mm,
                instance.side,
            );
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
            if !feeder_pick.rotation_deg.is_finite() {
                errors.push(format!(
                    "{} feeder part rotation is invalid",
                    placement.reference
                ));
                continue;
            }
            *uses += 1;
            steps.push(PlanStep {
                board_instance_id: instance.id,
                placement_reference: placement.reference.clone(),
                part_key,
                feeder_id: feeder.id(),
                nozzle_id,
                part_height_mm: height_mm,
                body_width_mm,
                body_length_mm,
                part_orientation,
                feeder_part_rotation_deg: feeder_pick.rotation_deg,
                placement_surface_offset_mm: instance.placement_surface_offset_mm,
                placement_seating_offset_mm,
                placement_release_seating_offsets_mm,
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

fn expand_placement_release_schedule(
    schedule: PlacementReleaseSchedule,
) -> Result<Vec<f64>, &'static str> {
    schedule.validate()?;
    let mut offsets = vec![schedule.start_seating_offset_mm];
    let mut current = schedule.start_seating_offset_mm;
    while current - schedule.step_mm > schedule.minimum_seating_offset_mm + 1e-9 {
        current -= schedule.step_mm;
        offsets.push(current);
        if offsets.len() >= MAXIMUM_PLACEMENT_RELEASE_ATTEMPTS {
            return Err("placement release schedule has too many attempts");
        }
    }
    if current > schedule.minimum_seating_offset_mm + 1e-9 {
        offsets.push(schedule.minimum_seating_offset_mm);
    }
    if offsets.len() > MAXIMUM_PLACEMENT_RELEASE_ATTEMPTS {
        return Err("placement release schedule has too many attempts");
    }
    Ok(offsets)
}

fn select_compatible_nozzle(part: &Part, profile: &MachineProfile) -> Option<String> {
    if part.compatible_tips.is_empty() {
        return None;
    }
    profile
        .bottom_vision
        .as_ref()?
        .nozzles
        .iter()
        .find_map(|nozzle| {
            part.compatible_tips
                .iter()
                .any(|tip| tip == &nozzle.expected_tip_id)
                .then(|| nozzle.nozzle_id.clone())
        })
}

pub fn placement_part_key(placement: &Placement) -> String {
    placement
        .properties
        .get("MPN")
        .filter(|value| !value.trim().is_empty())
        .cloned()
        .unwrap_or_else(|| format!("{}:{}", placement.package, placement.value))
}

fn transformed_pose(
    design: Pose,
    transform: velocitypnp_domain::Transform2d,
    z_mm: f64,
    side: BoardSide,
) -> Pose {
    let (x_mm, y_mm) = transform.apply(design.x_mm, design.y_mm);
    let rotation_deg = if side == BoardSide::Top {
        top_side_placement_rotation_deg(transform, design.rotation_deg)
    } else {
        let angle = design.rotation_deg.to_radians();
        let x_axis = transform.matrix[0] * angle.cos() + transform.matrix[1] * angle.sin();
        let y_axis = transform.matrix[3] * angle.cos() + transform.matrix[4] * angle.sin();
        y_axis.atan2(x_axis).to_degrees()
    };
    Pose {
        x_mm,
        y_mm,
        z_mm,
        rotation_deg,
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
        serde_json::from_str(include_str!("../../../profiles/lumenpnp-v3.0.2.json")).unwrap()
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
            placement_references: Vec::new(),
            placement_seating_offset_mm: None,
            placement_release: None,
            boards: vec![BoardInstance {
                id: Uuid::new_v4(),
                board_id,
                label: "A1".into(),
                side: BoardSide::Top,
                design_to_machine: None,
                surface_z_mm: None,
                placement_surface_offset_mm: 0.0,
            }],
        };
        let plan = plan_job(
            &job,
            &HashMap::from([(board_id, board)]),
            &[],
            &HashMap::new(),
            &profile(),
        );
        assert!(!plan.report.ready);
        assert_eq!(plan.report.errors.len(), 3);
    }

    #[test]
    fn plan_can_select_one_named_placement() {
        let board_id = Uuid::new_v4();
        let part_key = "R0603:10k".to_string();
        let placements = ["R1", "R2"].map(|reference| {
            let mut properties = BTreeMap::new();
            if reference == "R2" {
                properties.insert("Placement Seating Offset".into(), "0.07".into());
            }
            Placement {
                reference: reference.into(),
                value: "10k".into(),
                package: "R0603".into(),
                side: BoardSide::Top,
                pose: Pose {
                    x_mm: 1.0,
                    y_mm: 2.0,
                    z_mm: 0.0,
                    rotation_deg: 0.0,
                },
                enabled: true,
                fiducial: false,
                properties,
            }
        });
        let board = BoardDesign {
            id: board_id,
            name: "board".into(),
            source_format: "kicad_pcb".into(),
            source_version: None,
            width_mm: Some(10.0),
            height_mm: Some(10.0),
            outline_bounds: None,
            placements: placements.into(),
            warnings: vec![],
        };
        let job = Job {
            id: Uuid::new_v4(),
            name: "one placement".into(),
            boards: vec![BoardInstance {
                id: Uuid::new_v4(),
                board_id,
                label: "A1".into(),
                side: BoardSide::Top,
                design_to_machine: Some(velocitypnp_domain::Transform2d::rigid(100.0, 100.0, 0.0)),
                surface_z_mm: Some(3.0),
                placement_surface_offset_mm: 0.10,
            }],
            placement_references: vec!["R2".into()],
            placement_seating_offset_mm: Some(0.06),
            placement_release: Some(PlacementReleaseSchedule {
                start_seating_offset_mm: 0.2,
                minimum_seating_offset_mm: 0.0,
                step_mm: 0.1,
            }),
            status: "ready".into(),
        };
        let feeder = Feeder::Strip {
            id: Uuid::new_v4(),
            name: "resistors".into(),
            part_key: part_key.clone(),
            first_pick: Pose {
                x_mm: 10.0,
                y_mm: 20.0,
                z_mm: 4.0,
                rotation_deg: 0.0,
            },
            tape_pitch_mm: 4.0,
            tape_width_mm: 8.0,
            tape_rotation_deg: 0.0,
            remaining: Some(2),
        };
        let parts = HashMap::from([(
            part_key.clone(),
            Part {
                id: Uuid::new_v4(),
                key: part_key,
                manufacturer_part_number: None,
                package: "R0603".into(),
                height_mm: Some(0.55),
                body_width_mm: Some(0.8),
                body_length_mm: Some(1.6),
                placement_seating_offset_mm: Some(0.05),
                placement_release: Some(PlacementReleaseSchedule {
                    start_seating_offset_mm: 0.1,
                    minimum_seating_offset_mm: -0.1,
                    step_mm: 0.1,
                }),
                compatible_tips: vec!["N045".into()],
                vision_profile: None,
                orientation: Some(velocitypnp_domain::PartOrientation::AxisSymmetric),
            },
        )]);

        let plan = plan_job(
            &job,
            &HashMap::from([(board_id, board)]),
            &[feeder],
            &parts,
            &profile(),
        );

        assert!(plan.report.ready, "{:?}", plan.report.errors);
        assert_eq!(plan.steps.len(), 1);
        assert_eq!(plan.steps[0].placement_reference, "R2");
        assert_eq!(plan.steps[0].nozzle_id, "N1");
        assert!((plan.steps[0].board_place.z_mm - 3.72).abs() < 1e-9);
        assert_eq!(plan.steps[0].part_height_mm, 0.55);
        assert_eq!(
            plan.steps[0].part_orientation,
            PartOrientation::AxisSymmetric
        );
        assert_eq!(plan.steps[0].feeder_part_rotation_deg, 0.0);
        assert_eq!(plan.steps[0].placement_surface_offset_mm, 0.10);
        assert_eq!(plan.steps[0].placement_seating_offset_mm, 0.07);
        let offsets = &plan.steps[0].placement_release_seating_offsets_mm;
        assert_eq!(offsets.len(), 3);
        for (actual, expected) in offsets.iter().zip([0.07, -0.03, -0.1]) {
            assert!((*actual - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn release_schedule_expands_shallow_to_deep_and_keeps_the_limit() {
        let offsets = expand_placement_release_schedule(PlacementReleaseSchedule {
            start_seating_offset_mm: -0.2,
            minimum_seating_offset_mm: -0.4,
            step_mm: 0.1,
        })
        .unwrap();

        assert_eq!(offsets.len(), 3);
        for (actual, expected) in offsets.iter().zip([-0.2, -0.3, -0.4]) {
            assert!((*actual - expected).abs() < 1e-9);
        }

        let error = expand_placement_release_schedule(PlacementReleaseSchedule {
            start_seating_offset_mm: 0.5,
            minimum_seating_offset_mm: -0.5,
            step_mm: 0.02,
        })
        .unwrap_err();
        assert_eq!(error, "placement release schedule has too many attempts");
    }

    #[test]
    fn release_schedule_includes_a_nonuniform_final_depth() {
        let offsets = expand_placement_release_schedule(PlacementReleaseSchedule {
            start_seating_offset_mm: -0.15,
            minimum_seating_offset_mm: -0.4,
            step_mm: 0.1,
        })
        .unwrap();

        assert_eq!(offsets.len(), 4);
        for (actual, expected) in offsets.iter().zip([-0.15, -0.25, -0.35, -0.4]) {
            assert!((*actual - expected).abs() < 1e-9);
        }
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
            BoardSide::Top,
        );
        assert!((transformed.x_mm - 10.0).abs() < 1e-9);
        assert!((transformed.y_mm - 21.0).abs() < 1e-9);
        assert!((transformed.rotation_deg - 135.0).abs() < 1e-9);
    }

    #[test]
    fn reflected_top_side_transform_keeps_kicad_rotation_convention() {
        let bounds = velocitypnp_domain::BoardOutlineBounds {
            min_x_mm: 0.0,
            max_x_mm: 20.0,
            min_y_mm: -10.0,
            max_y_mm: 0.0,
        };
        let transform = velocitypnp_domain::top_side_board_transform(bounds, [100.0, 200.0], -2.0);
        let transformed = transformed_pose(
            Pose {
                x_mm: 15.0,
                y_mm: -8.0,
                rotation_deg: 45.0,
                ..Pose::ORIGIN
            },
            transform,
            5.0,
            BoardSide::Top,
        );
        assert!((transformed.rotation_deg - 43.0).abs() < 1e-9);
        let expected = transform.apply(15.0, -8.0);
        assert!((transformed.x_mm - expected.0).abs() < 1e-9);
        assert!((transformed.y_mm - expected.1).abs() < 1e-9);
    }
}
