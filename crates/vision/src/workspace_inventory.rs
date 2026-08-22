use crate::{resize_gray_frame, GrayFrame};
use anyhow::{ensure, Result};
use velocitypnp_domain::{
    WorkspaceBoardCandidate, WorkspaceChangeDetectionMode, WorkspaceChangeSetReport,
    WorkspaceChangeThresholds, WorkspaceInventoryBaseline, WorkspaceInventoryObject,
    WorkspaceInventoryObjectSource, WorkspaceObjectChange, WorkspaceObjectChangeKind,
    WorkspaceObjectClass, WorkspaceRasterTransform, WorkspaceScanRegion,
};

pub const WORKSPACE_CHANGE_SET_ALGORITHM: &str = "workspace_change_set.v3";
pub const WORKSPACE_CHANGE_SET_ARTIFACT: &str = "workspace-change-set.json";
pub const MINIMUM_OBJECT_MATCH_OVERLAP: f64 = 0.25;
pub const UNCHANGED_OBJECT_OVERLAP: f64 = 0.65;
pub const MAXIMUM_UNCHANGED_VISUAL_DIFFERENCE: f64 = 0.10;
pub const MAXIMUM_UNCHANGED_RASTER_DIFFERENCE: f64 = 0.04;
pub const MINIMUM_NEW_RASTER_REGION_AREA_MM2: f64 = 400.0;
pub const CHANGE_REFINEMENT_MARGIN_MM: f64 = 4.0;
const COMPARISON_FRAME_SIZE: u32 = 96;
const MINIMUM_RASTER_REGION_COVERAGE: f64 = 0.15;
const COMPLETED_REFINEMENT_COVERAGE: f64 = 0.90;

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceObservedObject {
    pub candidate: WorkspaceBoardCandidate,
    pub evidence: Option<GrayFrame>,
}

#[derive(Debug, Clone, Copy)]
pub struct WorkspaceRasterEvidence<'a> {
    pub baseline: &'a GrayFrame,
    pub baseline_transform: WorkspaceRasterTransform,
    pub observation: &'a GrayFrame,
    pub observation_transform: WorkspaceRasterTransform,
}

pub fn compare_workspace_observation(
    baseline: &WorkspaceInventoryBaseline,
    observation_scan_id: uuid::Uuid,
    observation_region: WorkspaceScanRegion,
    baseline_objects: &[WorkspaceObservedObject],
    observation_objects: &[WorkspaceObservedObject],
    raster_evidence: Option<WorkspaceRasterEvidence<'_>>,
    completed_refinement_regions: &[WorkspaceScanRegion],
) -> Result<WorkspaceChangeSetReport> {
    observation_region.validate().map_err(anyhow::Error::msg)?;
    if let Some(raster) = raster_evidence {
        validate_raster_transform(raster.baseline_transform)?;
        validate_raster_transform(raster.observation_transform)?;
    }
    let thresholds = WorkspaceChangeThresholds {
        minimum_match_overlap_fraction: MINIMUM_OBJECT_MATCH_OVERLAP,
        unchanged_overlap_fraction: UNCHANGED_OBJECT_OVERLAP,
        maximum_unchanged_visual_difference: MAXIMUM_UNCHANGED_VISUAL_DIFFERENCE,
        refinement_margin_mm: CHANGE_REFINEMENT_MARGIN_MM,
        maximum_unchanged_raster_difference: raster_evidence
            .map_or(0.0, |_| MAXIMUM_UNCHANGED_RASTER_DIFFERENCE),
        minimum_new_raster_region_area_mm2: 0.0,
    };
    let mut baseline_matched = vec![false; baseline_objects.len()];
    let mut changes = Vec::new();
    let mut inventory = Vec::new();
    let mut target_regions = Vec::new();
    let mut missing_evidence_pairs = 0_u32;

    for observation in observation_objects {
        let matched =
            best_baseline_match(&observation.candidate, baseline_objects, &baseline_matched);
        if let Some((baseline_index, overlap_fraction)) = matched {
            baseline_matched[baseline_index] = true;
            let prior = &baseline_objects[baseline_index];
            let visual_difference = match (&prior.evidence, &observation.evidence) {
                (Some(prior), Some(current)) => Some(normalized_visual_difference(prior, current)?),
                _ => {
                    missing_evidence_pairs += 1;
                    None
                }
            };
            let raster_difference = raster_evidence
                .filter(|_| region_contains(observation_region, prior.candidate.region))
                .map(|raster| {
                    let baseline_crop = crop_machine_region(
                        raster.baseline,
                        raster.baseline_transform,
                        prior.candidate.region,
                    )?;
                    let observation_crop = crop_machine_region(
                        raster.observation,
                        raster.observation_transform,
                        prior.candidate.region,
                    )?;
                    normalized_visual_difference(&baseline_crop, &observation_crop)
                })
                .transpose()?;
            let prior_class = candidate_class(&prior.candidate);
            let current_class = candidate_class(&observation.candidate);
            let class_changed = prior_class != current_class;
            let geometry_changed = overlap_fraction < UNCHANGED_OBJECT_OVERLAP;
            let appearance_changed = visual_difference
                .is_some_and(|difference| difference > MAXIMUM_UNCHANGED_VISUAL_DIFFERENCE);
            let raster_changed = raster_difference
                .is_some_and(|difference| difference > MAXIMUM_UNCHANGED_RASTER_DIFFERENCE);
            let modified =
                class_changed || geometry_changed || appearance_changed || raster_changed;
            let kind = if modified {
                WorkspaceObjectChangeKind::Modified
            } else {
                WorkspaceObjectChangeKind::Unchanged
            };
            let region = union_region(prior.candidate.region, observation.candidate.region);
            let refinement_required = modified
                && needs_refinement(&observation.candidate)
                && !has_completed_refinement(
                    observation.candidate.region,
                    completed_refinement_regions,
                );
            if refinement_required {
                target_regions.push(expand_and_clip_region(
                    region,
                    observation_region,
                    CHANGE_REFINEMENT_MARGIN_MM,
                ));
            }
            changes.push(WorkspaceObjectChange {
                change_index: 0,
                kind,
                baseline_candidate_index: Some(prior.candidate.candidate_index),
                observation_candidate_index: Some(observation.candidate.candidate_index),
                class: current_class,
                known_fixture_id: observation.candidate.known_fixture_id.clone(),
                region,
                overlap_fraction: Some(overlap_fraction),
                visual_difference,
                raster_difference,
                refinement_required,
                explanation: change_explanation(
                    modified,
                    class_changed,
                    geometry_changed,
                    appearance_changed,
                    raster_changed,
                    visual_difference.is_none(),
                ),
            });
            inventory.push(inventory_object(
                observation_scan_id,
                &observation.candidate,
                WorkspaceInventoryObjectSource::Observation,
            ));
        } else {
            let class = candidate_class(&observation.candidate);
            let refinement_required = needs_refinement(&observation.candidate)
                && !has_completed_refinement(
                    observation.candidate.region,
                    completed_refinement_regions,
                );
            if refinement_required {
                target_regions.push(expand_and_clip_region(
                    observation.candidate.region,
                    observation_region,
                    CHANGE_REFINEMENT_MARGIN_MM,
                ));
            }
            changes.push(WorkspaceObjectChange {
                change_index: 0,
                kind: WorkspaceObjectChangeKind::Added,
                baseline_candidate_index: None,
                observation_candidate_index: Some(observation.candidate.candidate_index),
                class,
                known_fixture_id: observation.candidate.known_fixture_id.clone(),
                region: observation.candidate.region,
                overlap_fraction: None,
                visual_difference: None,
                raster_difference: None,
                refinement_required,
                explanation: "the observation contains no matching baseline object".into(),
            });
            inventory.push(inventory_object(
                observation_scan_id,
                &observation.candidate,
                WorkspaceInventoryObjectSource::Observation,
            ));
        }
    }

    for (baseline_index, prior) in baseline_objects.iter().enumerate() {
        if baseline_matched[baseline_index] {
            continue;
        }
        let class = candidate_class(&prior.candidate);
        let completely_observed = region_contains(observation_region, prior.candidate.region);
        if completely_observed {
            let refinement_required = needs_refinement(&prior.candidate)
                && !has_completed_refinement(prior.candidate.region, completed_refinement_regions);
            if refinement_required {
                target_regions.push(expand_and_clip_region(
                    prior.candidate.region,
                    observation_region,
                    CHANGE_REFINEMENT_MARGIN_MM,
                ));
            }
            changes.push(WorkspaceObjectChange {
                change_index: 0,
                kind: WorkspaceObjectChangeKind::Removed,
                baseline_candidate_index: Some(prior.candidate.candidate_index),
                observation_candidate_index: None,
                class,
                known_fixture_id: prior.candidate.known_fixture_id.clone(),
                region: prior.candidate.region,
                overlap_fraction: None,
                visual_difference: None,
                raster_difference: None,
                refinement_required,
                explanation: "the baseline object is absent from a fully observed region".into(),
            });
        } else {
            changes.push(WorkspaceObjectChange {
                change_index: 0,
                kind: WorkspaceObjectChangeKind::ReusedBaseline,
                baseline_candidate_index: Some(prior.candidate.candidate_index),
                observation_candidate_index: None,
                class,
                known_fixture_id: prior.candidate.known_fixture_id.clone(),
                region: prior.candidate.region,
                overlap_fraction: None,
                visual_difference: None,
                raster_difference: None,
                refinement_required: false,
                explanation: "the partial observation does not cover this baseline object".into(),
            });
            inventory.push(inventory_object(
                baseline.scan_id,
                &prior.candidate,
                WorkspaceInventoryObjectSource::BaselineReuse,
            ));
        }
    }

    for (index, change) in changes.iter_mut().enumerate() {
        change.change_index = index as u32;
    }
    for (index, object) in inventory.iter_mut().enumerate() {
        object.inventory_index = index as u32;
    }
    let target_regions = merge_regions(target_regions);
    let count = |kind| changes.iter().filter(|change| change.kind == kind).count() as u32;
    let mut warnings = Vec::new();
    if missing_evidence_pairs > 0 {
        warnings.push(format!(
            "{missing_evidence_pairs} matched object pairs lacked normalized image evidence"
        ));
    }
    Ok(WorkspaceChangeSetReport {
        algorithm_version: WORKSPACE_CHANGE_SET_ALGORITHM.into(),
        detection_mode: WorkspaceChangeDetectionMode::SemanticProposals,
        baseline_id: baseline.id,
        baseline_scan_id: baseline.scan_id,
        observation_scan_id,
        thresholds,
        added_count: count(WorkspaceObjectChangeKind::Added),
        removed_count: count(WorkspaceObjectChangeKind::Removed),
        modified_count: count(WorkspaceObjectChangeKind::Modified),
        unchanged_count: count(WorkspaceObjectChangeKind::Unchanged),
        reused_baseline_count: count(WorkspaceObjectChangeKind::ReusedBaseline),
        changes,
        current_inventory: inventory,
        target_regions,
        warnings,
    })
}

#[allow(clippy::too_many_arguments)]
pub fn compare_workspace_reconnaissance(
    baseline: &WorkspaceInventoryBaseline,
    observation_scan_id: uuid::Uuid,
    observation_region: WorkspaceScanRegion,
    baseline_raster: &GrayFrame,
    baseline_transform: WorkspaceRasterTransform,
    observation_raster: &GrayFrame,
    observation_transform: WorkspaceRasterTransform,
    baseline_objects: &[WorkspaceObservedObject],
    observation_regions: &[WorkspaceScanRegion],
) -> Result<WorkspaceChangeSetReport> {
    observation_region.validate().map_err(anyhow::Error::msg)?;
    validate_raster_transform(baseline_transform)?;
    validate_raster_transform(observation_transform)?;
    let thresholds = WorkspaceChangeThresholds {
        minimum_match_overlap_fraction: MINIMUM_OBJECT_MATCH_OVERLAP,
        unchanged_overlap_fraction: UNCHANGED_OBJECT_OVERLAP,
        maximum_unchanged_visual_difference: MAXIMUM_UNCHANGED_VISUAL_DIFFERENCE,
        refinement_margin_mm: CHANGE_REFINEMENT_MARGIN_MM,
        maximum_unchanged_raster_difference: MAXIMUM_UNCHANGED_RASTER_DIFFERENCE,
        minimum_new_raster_region_area_mm2: MINIMUM_NEW_RASTER_REGION_AREA_MM2,
    };
    let mut matched_regions = vec![false; observation_regions.len()];
    let mut changes = Vec::new();
    let mut inventory = Vec::new();
    let mut target_regions = Vec::new();

    for prior in baseline_objects {
        if !region_contains(observation_region, prior.candidate.region) {
            changes.push(WorkspaceObjectChange {
                change_index: 0,
                kind: WorkspaceObjectChangeKind::ReusedBaseline,
                baseline_candidate_index: Some(prior.candidate.candidate_index),
                observation_candidate_index: None,
                class: candidate_class(&prior.candidate),
                known_fixture_id: prior.candidate.known_fixture_id.clone(),
                region: prior.candidate.region,
                overlap_fraction: None,
                visual_difference: None,
                raster_difference: None,
                refinement_required: false,
                explanation: "the reconnaissance scan does not cover this baseline object".into(),
            });
            inventory.push(inventory_object(
                baseline.scan_id,
                &prior.candidate,
                WorkspaceInventoryObjectSource::BaselineReuse,
            ));
            continue;
        }
        let baseline_crop =
            crop_machine_region(baseline_raster, baseline_transform, prior.candidate.region)?;
        let observation_crop = crop_machine_region(
            observation_raster,
            observation_transform,
            prior.candidate.region,
        )?;
        let visual_difference = normalized_visual_difference(&baseline_crop, &observation_crop)?;
        let matched_region = observation_regions
            .iter()
            .enumerate()
            .map(|(index, region)| {
                (
                    index,
                    intersection_area(*region, prior.candidate.region)
                        / region_area(prior.candidate.region),
                )
            })
            .filter(|(_, coverage)| *coverage >= MINIMUM_RASTER_REGION_COVERAGE)
            .max_by(|first, second| first.1.total_cmp(&second.1));
        if let Some((index, _)) = matched_region {
            matched_regions[index] = true;
        }
        let modified = visual_difference > MAXIMUM_UNCHANGED_RASTER_DIFFERENCE;
        let refinement_required = modified && needs_refinement(&prior.candidate);
        if refinement_required {
            let region = matched_region.map_or(prior.candidate.region, |(index, _)| {
                union_region(prior.candidate.region, observation_regions[index])
            });
            target_regions.push(expand_and_clip_region(
                region,
                observation_region,
                CHANGE_REFINEMENT_MARGIN_MM,
            ));
        }
        changes.push(WorkspaceObjectChange {
            change_index: 0,
            kind: if modified {
                WorkspaceObjectChangeKind::Modified
            } else {
                WorkspaceObjectChangeKind::Unchanged
            },
            baseline_candidate_index: Some(prior.candidate.candidate_index),
            observation_candidate_index: matched_region.map(|(index, _)| index as u32),
            class: candidate_class(&prior.candidate),
            known_fixture_id: prior.candidate.known_fixture_id.clone(),
            region: prior.candidate.region,
            overlap_fraction: matched_region.map(|(_, coverage)| coverage),
            visual_difference: None,
            raster_difference: Some(visual_difference),
            refinement_required,
            explanation: if modified {
                "the plate-projected appearance changed within the baseline object region".into()
            } else {
                "the plate-projected appearance is stable within the baseline object region".into()
            },
        });
        inventory.push(inventory_object(
            baseline.scan_id,
            &prior.candidate,
            WorkspaceInventoryObjectSource::BaselineReuse,
        ));
    }

    let mut ignored_small_regions = 0_u32;
    for (index, region) in observation_regions.iter().copied().enumerate() {
        if matched_regions[index] {
            continue;
        }
        let area_mm2 = region_area(region);
        if area_mm2 < MINIMUM_NEW_RASTER_REGION_AREA_MM2 {
            ignored_small_regions += 1;
            continue;
        }
        target_regions.push(expand_and_clip_region(
            region,
            observation_region,
            CHANGE_REFINEMENT_MARGIN_MM,
        ));
        changes.push(WorkspaceObjectChange {
            change_index: 0,
            kind: WorkspaceObjectChangeKind::Added,
            baseline_candidate_index: None,
            observation_candidate_index: Some(index as u32),
            class: WorkspaceObjectClass::Unknown,
            known_fixture_id: None,
            region,
            overlap_fraction: None,
            visual_difference: None,
            raster_difference: None,
            refinement_required: true,
            explanation: "a new reconnaissance seam region exceeds the review area".into(),
        });
        inventory.push(WorkspaceInventoryObject {
            inventory_index: 0,
            source: WorkspaceInventoryObjectSource::Observation,
            source_scan_id: observation_scan_id,
            source_candidate_index: index as u32,
            class: WorkspaceObjectClass::Unknown,
            classification_confidence: 0.0,
            known_fixture_id: None,
            region,
            outline_machine_mm: axis_aligned_outline(region),
        });
    }
    for (index, change) in changes.iter_mut().enumerate() {
        change.change_index = index as u32;
    }
    for (index, object) in inventory.iter_mut().enumerate() {
        object.inventory_index = index as u32;
    }
    let count = |kind| changes.iter().filter(|change| change.kind == kind).count() as u32;
    let mut warnings = Vec::new();
    if ignored_small_regions > 0 {
        warnings.push(format!(
            "{ignored_small_regions} unmatched small seam regions were below the new-object review area"
        ));
    }
    Ok(WorkspaceChangeSetReport {
        algorithm_version: WORKSPACE_CHANGE_SET_ALGORITHM.into(),
        detection_mode: WorkspaceChangeDetectionMode::BaselineGuidedRaster,
        baseline_id: baseline.id,
        baseline_scan_id: baseline.scan_id,
        observation_scan_id,
        thresholds,
        added_count: count(WorkspaceObjectChangeKind::Added),
        removed_count: 0,
        modified_count: count(WorkspaceObjectChangeKind::Modified),
        unchanged_count: count(WorkspaceObjectChangeKind::Unchanged),
        reused_baseline_count: count(WorkspaceObjectChangeKind::ReusedBaseline),
        changes,
        current_inventory: inventory,
        target_regions: merge_regions(target_regions),
        warnings,
    })
}

fn best_baseline_match(
    observation: &WorkspaceBoardCandidate,
    baseline: &[WorkspaceObservedObject],
    matched: &[bool],
) -> Option<(usize, f64)> {
    if let Some(fixture_id) = observation.known_fixture_id.as_deref() {
        return baseline
            .iter()
            .enumerate()
            .find(|(index, prior)| {
                !matched[*index] && prior.candidate.known_fixture_id.as_deref() == Some(fixture_id)
            })
            .map(|(index, prior)| {
                (
                    index,
                    region_iou(observation.region, prior.candidate.region),
                )
            });
    }
    baseline
        .iter()
        .enumerate()
        .filter(|(index, prior)| !matched[*index] && prior.candidate.known_fixture_id.is_none())
        .map(|(index, prior)| {
            (
                index,
                region_iou(observation.region, prior.candidate.region),
            )
        })
        .filter(|(_, overlap)| *overlap >= MINIMUM_OBJECT_MATCH_OVERLAP)
        .max_by(|first, second| first.1.total_cmp(&second.1))
}

fn normalized_visual_difference(first: &GrayFrame, second: &GrayFrame) -> Result<f64> {
    ensure!(
        !first.pixels.is_empty() && !second.pixels.is_empty(),
        "workspace object evidence is empty"
    );
    let first = resize_gray_frame(first, COMPARISON_FRAME_SIZE, COMPARISON_FRAME_SIZE)?;
    let second = resize_gray_frame(second, COMPARISON_FRAME_SIZE, COMPARISON_FRAME_SIZE)?;
    let first_mean = first
        .pixels
        .iter()
        .map(|value| f64::from(*value))
        .sum::<f64>()
        / first.pixels.len() as f64;
    let second_mean = second
        .pixels
        .iter()
        .map(|value| f64::from(*value))
        .sum::<f64>()
        / second.pixels.len() as f64;
    let difference = first
        .pixels
        .iter()
        .zip(&second.pixels)
        .map(|(first, second)| {
            ((f64::from(*first) - first_mean) - (f64::from(*second) - second_mean)).abs()
        })
        .sum::<f64>()
        / first.pixels.len() as f64
        / 255.0;
    Ok(difference.clamp(0.0, 1.0))
}

fn candidate_class(candidate: &WorkspaceBoardCandidate) -> WorkspaceObjectClass {
    candidate
        .classification
        .as_ref()
        .map(|classification| classification.class)
        .unwrap_or_default()
}

fn is_static_fixture(class: WorkspaceObjectClass) -> bool {
    matches!(
        class,
        WorkspaceObjectClass::CalibrationFixture | WorkspaceObjectClass::FixedMachineFeature
    )
}

fn needs_refinement(candidate: &WorkspaceBoardCandidate) -> bool {
    candidate.known_fixture_id.is_none() && !is_static_fixture(candidate_class(candidate))
}

fn has_completed_refinement(
    region: WorkspaceScanRegion,
    completed_regions: &[WorkspaceScanRegion],
) -> bool {
    completed_regions.iter().any(|completed| {
        intersection_area(region, *completed) / region_area(region) >= COMPLETED_REFINEMENT_COVERAGE
    })
}

fn inventory_object(
    scan_id: uuid::Uuid,
    candidate: &WorkspaceBoardCandidate,
    source: WorkspaceInventoryObjectSource,
) -> WorkspaceInventoryObject {
    WorkspaceInventoryObject {
        inventory_index: 0,
        source,
        source_scan_id: scan_id,
        source_candidate_index: candidate.candidate_index,
        class: candidate_class(candidate),
        classification_confidence: candidate
            .classification
            .as_ref()
            .map_or(0.0, |classification| classification.confidence),
        known_fixture_id: candidate.known_fixture_id.clone(),
        region: candidate.region,
        outline_machine_mm: candidate.outline_machine_mm.clone(),
    }
}

fn change_explanation(
    modified: bool,
    class_changed: bool,
    geometry_changed: bool,
    appearance_changed: bool,
    raster_changed: bool,
    missing_evidence: bool,
) -> String {
    if !modified {
        return if missing_evidence {
            "object geometry and class are stable; image evidence is unavailable".into()
        } else {
            "object geometry, class, and normalized appearance are stable".into()
        };
    }
    let mut reasons = Vec::new();
    if class_changed {
        reasons.push("semantic class changed");
    }
    if geometry_changed {
        reasons.push("proposal overlap decreased");
    }
    if appearance_changed {
        reasons.push("normalized appearance changed");
    }
    if raster_changed {
        reasons.push("plate-projected reconnaissance appearance changed");
    }
    reasons.join("; ")
}

fn validate_raster_transform(transform: WorkspaceRasterTransform) -> Result<()> {
    ensure!(
        transform.top_left_x_mm.is_finite()
            && transform.top_left_y_mm.is_finite()
            && transform.x_mm_per_pixel.is_finite()
            && transform.y_mm_per_pixel.is_finite()
            && transform.x_mm_per_pixel > 0.0
            && transform.y_mm_per_pixel < 0.0,
        "workspace raster transform must use the overhead machine convention"
    );
    Ok(())
}

fn crop_machine_region(
    frame: &GrayFrame,
    transform: WorkspaceRasterTransform,
    region: WorkspaceScanRegion,
) -> Result<GrayFrame> {
    region.validate().map_err(anyhow::Error::msg)?;
    ensure!(
        frame.pixels.len() == frame.width as usize * frame.height as usize,
        "workspace raster buffer does not match its dimensions"
    );
    let x_min = ((region.x_min_mm - transform.top_left_x_mm) / transform.x_mm_per_pixel)
        .floor()
        .max(0.0) as u32;
    let x_max = ((region.x_max_mm - transform.top_left_x_mm) / transform.x_mm_per_pixel)
        .ceil()
        .min(f64::from(frame.width)) as u32;
    let y_min = ((region.y_max_mm - transform.top_left_y_mm) / transform.y_mm_per_pixel)
        .floor()
        .max(0.0) as u32;
    let y_max = ((region.y_min_mm - transform.top_left_y_mm) / transform.y_mm_per_pixel)
        .ceil()
        .min(f64::from(frame.height)) as u32;
    ensure!(
        x_max > x_min && y_max > y_min,
        "workspace object region is outside the raster"
    );
    let mut pixels = Vec::with_capacity((x_max - x_min) as usize * (y_max - y_min) as usize);
    for y in y_min..y_max {
        let start = y as usize * frame.width as usize + x_min as usize;
        let end = start + (x_max - x_min) as usize;
        pixels.extend_from_slice(&frame.pixels[start..end]);
    }
    Ok(GrayFrame {
        width: x_max - x_min,
        height: y_max - y_min,
        pixels,
    })
}

fn region_area(region: WorkspaceScanRegion) -> f64 {
    (region.x_max_mm - region.x_min_mm).max(0.0) * (region.y_max_mm - region.y_min_mm).max(0.0)
}

fn intersection_area(first: WorkspaceScanRegion, second: WorkspaceScanRegion) -> f64 {
    (first.x_max_mm.min(second.x_max_mm) - first.x_min_mm.max(second.x_min_mm)).max(0.0)
        * (first.y_max_mm.min(second.y_max_mm) - first.y_min_mm.max(second.y_min_mm)).max(0.0)
}

fn axis_aligned_outline(region: WorkspaceScanRegion) -> Vec<[f64; 2]> {
    vec![
        [region.x_min_mm, region.y_min_mm],
        [region.x_max_mm, region.y_min_mm],
        [region.x_max_mm, region.y_max_mm],
        [region.x_min_mm, region.y_max_mm],
    ]
}

fn region_iou(first: WorkspaceScanRegion, second: WorkspaceScanRegion) -> f64 {
    let intersection = intersection_area(first, second);
    let first_area = region_area(first);
    let second_area = region_area(second);
    let union = first_area + second_area - intersection;
    if union <= 0.0 {
        0.0
    } else {
        (intersection / union).clamp(0.0, 1.0)
    }
}

fn region_contains(outer: WorkspaceScanRegion, inner: WorkspaceScanRegion) -> bool {
    inner.x_min_mm >= outer.x_min_mm
        && inner.x_max_mm <= outer.x_max_mm
        && inner.y_min_mm >= outer.y_min_mm
        && inner.y_max_mm <= outer.y_max_mm
}

fn union_region(first: WorkspaceScanRegion, second: WorkspaceScanRegion) -> WorkspaceScanRegion {
    WorkspaceScanRegion {
        x_min_mm: first.x_min_mm.min(second.x_min_mm),
        x_max_mm: first.x_max_mm.max(second.x_max_mm),
        y_min_mm: first.y_min_mm.min(second.y_min_mm),
        y_max_mm: first.y_max_mm.max(second.y_max_mm),
    }
}

fn expand_and_clip_region(
    region: WorkspaceScanRegion,
    bounds: WorkspaceScanRegion,
    margin_mm: f64,
) -> WorkspaceScanRegion {
    WorkspaceScanRegion {
        x_min_mm: (region.x_min_mm - margin_mm).max(bounds.x_min_mm),
        x_max_mm: (region.x_max_mm + margin_mm).min(bounds.x_max_mm),
        y_min_mm: (region.y_min_mm - margin_mm).max(bounds.y_min_mm),
        y_max_mm: (region.y_max_mm + margin_mm).min(bounds.y_max_mm),
    }
}

fn regions_overlap(first: WorkspaceScanRegion, second: WorkspaceScanRegion) -> bool {
    first.x_min_mm <= second.x_max_mm
        && first.x_max_mm >= second.x_min_mm
        && first.y_min_mm <= second.y_max_mm
        && first.y_max_mm >= second.y_min_mm
}

fn merge_regions(mut regions: Vec<WorkspaceScanRegion>) -> Vec<WorkspaceScanRegion> {
    let mut merged = Vec::<WorkspaceScanRegion>::new();
    while let Some(mut region) = regions.pop() {
        let mut index = 0;
        while index < merged.len() {
            if regions_overlap(region, merged[index]) {
                region = union_region(region, merged.remove(index));
                index = 0;
            } else {
                index += 1;
            }
        }
        merged.push(region);
    }
    merged.sort_by(|first, second| {
        second
            .y_max_mm
            .total_cmp(&first.y_max_mm)
            .then_with(|| first.x_min_mm.total_cmp(&second.x_min_mm))
    });
    merged
}

#[cfg(test)]
mod tests {
    use super::*;
    use uuid::Uuid;
    use velocitypnp_domain::{
        WorkspaceObjectClass, WorkspaceObjectClassification, WorkspaceScanRegion,
    };

    fn candidate(
        index: u32,
        region: WorkspaceScanRegion,
        class: WorkspaceObjectClass,
        fixture: Option<&str>,
        luma: u8,
    ) -> WorkspaceObservedObject {
        WorkspaceObservedObject {
            candidate: WorkspaceBoardCandidate {
                candidate_index: index,
                region,
                outline_machine_mm: vec![
                    [region.x_min_mm, region.y_min_mm],
                    [region.x_max_mm, region.y_min_mm],
                    [region.x_max_mm, region.y_max_mm],
                    [region.x_min_mm, region.y_max_mm],
                ],
                plate_id: Some("plate".into()),
                plate_role: None,
                center_x_mm: (region.x_min_mm + region.x_max_mm) * 0.5,
                center_y_mm: (region.y_min_mm + region.y_max_mm) * 0.5,
                width_mm: region.x_max_mm - region.x_min_mm,
                height_mm: region.y_max_mm - region.y_min_mm,
                rotation_deg: 0.0,
                evidence_cell_count: 10,
                measured_cell_fraction: 1.0,
                mean_surface_fraction: 0.5,
                mean_height_confidence: 1.0,
                fill_fraction: 1.0,
                texture_energy_luma: 10.0,
                board_likelihood: 0.5,
                classification: Some(WorkspaceObjectClassification {
                    class,
                    confidence: 0.9,
                    classifier_id: "test".into(),
                    review_required: false,
                    evidence: Vec::new(),
                }),
                known_fixture_id: fixture.map(str::to_string),
                evidence_artifact_name: Some(format!("object-evidence-{index:03}.png")),
                design_matches: Vec::new(),
            },
            evidence: Some(GrayFrame {
                width: 16,
                height: 16,
                pixels: (0..256)
                    .map(|index| luma.saturating_add((index % 7) as u8))
                    .collect(),
            }),
        }
    }

    fn baseline(scan_id: Uuid) -> WorkspaceInventoryBaseline {
        WorkspaceInventoryBaseline {
            id: Uuid::new_v4(),
            scan_id,
            profile_id: "profile".into(),
            calibration_id: Uuid::new_v4(),
            proposal_algorithm_version: "workspace_object_proposals.v3".into(),
            activated_at: chrono::Utc::now(),
            supersedes_baseline_id: None,
        }
    }

    fn region(x_min: f64, x_max: f64, y_min: f64, y_max: f64) -> WorkspaceScanRegion {
        WorkspaceScanRegion {
            x_min_mm: x_min,
            x_max_mm: x_max,
            y_min_mm: y_min,
            y_max_mm: y_max,
        }
    }

    fn raster(luma_offset: u8) -> GrayFrame {
        GrayFrame {
            width: 100,
            height: 100,
            pixels: (0..10_000)
                .map(|index| luma_offset.saturating_add(((index % 100) + (index / 100)) as u8 / 2))
                .collect(),
        }
    }

    fn raster_transform() -> WorkspaceRasterTransform {
        WorkspaceRasterTransform {
            top_left_x_mm: 0.0,
            top_left_y_mm: 100.0,
            x_mm_per_pixel: 1.0,
            y_mm_per_pixel: -1.0,
        }
    }

    #[test]
    fn identical_observation_is_unchanged() {
        let scan_id = Uuid::new_v4();
        let object = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Unknown,
            None,
            80,
        );
        let report = compare_workspace_observation(
            &baseline(scan_id),
            Uuid::new_v4(),
            region(0.0, 100.0, 0.0, 100.0),
            std::slice::from_ref(&object),
            std::slice::from_ref(&object),
            None,
            &[],
        )
        .unwrap();

        assert_eq!(report.unchanged_count, 1);
        assert_eq!(report.modified_count, 0);
        assert!(report.target_regions.is_empty());
        assert_eq!(report.current_inventory.len(), 1);
    }

    #[test]
    fn global_luma_change_does_not_modify_an_object() {
        let prior = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Feeder,
            None,
            40,
        );
        let current = candidate(
            1,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Feeder,
            None,
            120,
        );
        let report = compare_workspace_observation(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 100.0, 0.0, 100.0),
            &[prior],
            &[current],
            None,
            &[],
        )
        .unwrap();

        assert_eq!(report.unchanged_count, 1);
        assert_eq!(report.modified_count, 0);
    }

    #[test]
    fn semantic_comparison_preserves_reconnaissance_change_evidence() {
        let object = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Board,
            None,
            80,
        );
        let baseline_raster = raster(20);
        let mut observation_raster = raster(20);
        for y in 60..80 {
            for x in 10..30 {
                observation_raster.pixels[y * 100 + x] = if (x + y) % 2 == 0 { 0 } else { 255 };
            }
        }
        let report = compare_workspace_observation(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 100.0, 0.0, 100.0),
            std::slice::from_ref(&object),
            std::slice::from_ref(&object),
            Some(WorkspaceRasterEvidence {
                baseline: &baseline_raster,
                baseline_transform: raster_transform(),
                observation: &observation_raster,
                observation_transform: raster_transform(),
            }),
            &[region(9.0, 31.0, 19.0, 41.0)],
        )
        .unwrap();

        assert_eq!(report.modified_count, 1);
        assert_eq!(report.unchanged_count, 0);
        assert!(!report.changes[0].refinement_required);
        assert!(report.target_regions.is_empty());
        assert!(report.changes[0].visual_difference.unwrap() < 0.10);
        assert!(report.changes[0].raster_difference.unwrap() > 0.04);
        assert!(report.changes[0]
            .explanation
            .contains("reconnaissance appearance changed"));
    }

    #[test]
    fn changed_and_added_objects_create_refinement_targets() {
        let prior = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Unknown,
            None,
            40,
        );
        let mut changed = candidate(
            1,
            region(12.0, 32.0, 20.0, 40.0),
            WorkspaceObjectClass::Unknown,
            None,
            180,
        );
        changed.evidence.as_mut().unwrap().pixels[80..170].fill(0);
        let added = candidate(
            2,
            region(60.0, 75.0, 60.0, 75.0),
            WorkspaceObjectClass::Unknown,
            None,
            100,
        );
        let report = compare_workspace_observation(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 100.0, 0.0, 100.0),
            &[prior],
            &[changed, added],
            None,
            &[],
        )
        .unwrap();

        assert_eq!(report.modified_count, 1);
        assert_eq!(report.added_count, 1);
        assert_eq!(report.target_regions.len(), 2);
    }

    #[test]
    fn partial_observation_reuses_objects_outside_its_region() {
        let prior = candidate(
            0,
            region(70.0, 90.0, 70.0, 90.0),
            WorkspaceObjectClass::Feeder,
            None,
            80,
        );
        let report = compare_workspace_observation(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 50.0, 0.0, 50.0),
            &[prior],
            &[],
            None,
            &[],
        )
        .unwrap();

        assert_eq!(report.reused_baseline_count, 1);
        assert_eq!(report.removed_count, 0);
        assert_eq!(
            report.current_inventory[0].source,
            WorkspaceInventoryObjectSource::BaselineReuse
        );
        assert!(report.target_regions.is_empty());
    }

    #[test]
    fn absent_object_in_covered_region_is_removed() {
        let prior = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Board,
            None,
            80,
        );
        let report = compare_workspace_observation(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 50.0, 0.0, 50.0),
            &[prior],
            &[],
            None,
            &[],
        )
        .unwrap();

        assert_eq!(report.removed_count, 1);
        assert_eq!(report.target_regions.len(), 1);
        assert!(report.current_inventory.is_empty());
    }

    #[test]
    fn known_tooling_fixture_does_not_request_refinement() {
        let prior = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::ToolingStation,
            Some("nozzle_bank"),
            80,
        );
        let report = compare_workspace_observation(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 50.0, 0.0, 50.0),
            &[prior],
            &[],
            None,
            &[],
        )
        .unwrap();

        assert_eq!(report.removed_count, 1);
        assert!(report.target_regions.is_empty());
    }

    #[test]
    fn reconnaissance_reuses_stable_objects_and_filters_small_new_regions() {
        let prior = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Board,
            None,
            80,
        );
        let report = compare_workspace_reconnaissance(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 100.0, 0.0, 100.0),
            &raster(20),
            raster_transform(),
            &raster(60),
            raster_transform(),
            &[prior],
            &[
                region(8.0, 32.0, 18.0, 42.0),
                region(50.0, 75.0, 50.0, 75.0),
                region(80.0, 90.0, 80.0, 90.0),
            ],
        )
        .unwrap();

        assert_eq!(
            report.detection_mode,
            WorkspaceChangeDetectionMode::BaselineGuidedRaster
        );
        assert_eq!(report.unchanged_count, 1);
        assert_eq!(report.added_count, 1);
        assert_eq!(report.target_regions, vec![region(46.0, 79.0, 46.0, 79.0)]);
        assert_eq!(report.warnings.len(), 1);
    }

    #[test]
    fn reconnaissance_detects_appearance_change_in_a_baseline_region() {
        let prior = candidate(
            0,
            region(10.0, 30.0, 20.0, 40.0),
            WorkspaceObjectClass::Feeder,
            None,
            80,
        );
        let baseline_raster = raster(20);
        let mut observation_raster = raster(20);
        for y in 60..80 {
            for x in 10..30 {
                observation_raster.pixels[y * 100 + x] = if (x + y) % 2 == 0 { 0 } else { 255 };
            }
        }
        let report = compare_workspace_reconnaissance(
            &baseline(Uuid::new_v4()),
            Uuid::new_v4(),
            region(0.0, 100.0, 0.0, 100.0),
            &baseline_raster,
            raster_transform(),
            &observation_raster,
            raster_transform(),
            &[prior],
            &[region(8.0, 32.0, 18.0, 42.0)],
        )
        .unwrap();

        assert_eq!(report.modified_count, 1);
        assert_eq!(report.target_regions.len(), 1);
        assert!(report.changes[0].raster_difference.unwrap() > 0.04);
    }
}
