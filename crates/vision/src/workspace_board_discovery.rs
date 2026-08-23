use crate::workspace_features::{detect_workspace_line_features, WorkspaceLineFeatures};
use crate::{
    GrayFrame, RgbFrame, WorkspaceHeightCell, WorkspaceHeightCellSource, WorkspaceHeightMap,
};
use anyhow::{ensure, Result};
use std::collections::VecDeque;
use velocitypnp_domain::{
    BoardDesign, InstalledPlateGeometryReport, InstalledPlateRole, MachineProfile,
    WorkspaceBoardCandidate, WorkspaceBoardDesignMatch, WorkspaceBoardDiscoveryReport,
    WorkspaceBoardHeightSource, WorkspaceBoardRegistrationMethod, WorkspaceCameraApertureSummary,
    WorkspaceClassificationEvidence, WorkspaceFixtureAnchor, WorkspaceObjectClass,
    WorkspaceObjectClassification, WorkspaceRasterTransform, WorkspaceScanRegion,
};

pub const WORKSPACE_BOARD_DISCOVERY_ALGORITHM: &str = "workspace_object_proposals.v4";
pub const WORKSPACE_BOARD_DISCOVERY_REPORT_ARTIFACT: &str = "board-discovery.json";
pub const WORKSPACE_BOARD_DISCOVERY_PREVIEW_ARTIFACT: &str = "mosaic-board-candidates.png";

const INSTALLED_PLATE_FRACTION: f64 = 1.0;
const RAISED_SURFACE_MAX_FRACTION: f64 = 0.875;
const CONNECTION_RADIUS_CELLS: i32 = 2;
const MINIMUM_EVIDENCE_CELLS: usize = 4;
const MINIMUM_CANDIDATE_SPAN_MM: f64 = 6.0;
const MINIMUM_CANDIDATE_AREA_MM2: f64 = 100.0;
const MAXIMUM_DESIGN_MATCHES: usize = 3;
const MAXIMUM_EVIDENCE_CROP_PIXELS_PER_AXIS: u32 = 2_048;
const FIXTURE_MATCH_MINIMUM_FRACTION: f64 = 0.25;

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspacePlateFrame {
    pub plate_id: String,
    pub role: InstalledPlateRole,
    pub origin_machine_mm: [f64; 2],
    pub axis_x: [f64; 2],
    pub axis_y: [f64; 2],
    pub x_min_mm: f64,
    pub x_max_mm: f64,
    pub y_min_mm: f64,
    pub y_max_mm: f64,
}

#[derive(Debug, Clone, PartialEq)]
enum WorkspaceFixtureShape {
    Rectangle(Vec<[f64; 2]>),
    Circle {
        center_machine_mm: [f64; 2],
        radius_mm: f64,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceResolvedFixture {
    pub id: String,
    pub class: WorkspaceObjectClass,
    shape: WorkspaceFixtureShape,
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct WorkspaceDiscoveryContext {
    pub plates: Vec<WorkspacePlateFrame>,
    pub fixtures: Vec<WorkspaceResolvedFixture>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceCandidateEvidenceArtifact {
    pub artifact_name: String,
    pub frame: GrayFrame,
}

pub struct WorkspaceObjectEvidence<'a> {
    pub candidate: &'a WorkspaceBoardCandidate,
    pub normalized_crop: &'a GrayFrame,
    pub height_map: &'a WorkspaceHeightMap,
    pub component_cell_indices: &'a [usize],
    pub known_fixture: Option<(&'a WorkspaceResolvedFixture, f64)>,
}

/// Semantic classification is replaceable without changing region proposal,
/// geometry, storage, or safety policy. A future learned model implements this
/// interface and receives the same normalized evidence as the initial policy.
pub trait WorkspaceObjectClassifier: Send + Sync {
    fn classifier_id(&self) -> &str;
    fn classify(&self, evidence: WorkspaceObjectEvidence<'_>) -> WorkspaceObjectClassification;
}

#[derive(Debug, Default)]
pub struct DeterministicWorkspaceObjectClassifier;

impl WorkspaceObjectClassifier for DeterministicWorkspaceObjectClassifier {
    fn classifier_id(&self) -> &str {
        "deterministic_workspace_evidence.v2"
    }

    fn classify(&self, evidence: WorkspaceObjectEvidence<'_>) -> WorkspaceObjectClassification {
        let candidate = evidence.candidate;
        let mut observations = Vec::new();
        if let Some((fixture, overlap_fraction)) = evidence.known_fixture {
            let confidence = (0.75 + 0.25 * overlap_fraction).clamp(0.0, 1.0);
            observations.push(WorkspaceClassificationEvidence {
                source: format!("known_fixture:{}", fixture.id),
                suggested_class: fixture.class,
                confidence,
                explanation: format!(
                    "{:.1}% of raised height cells are in the calibrated fixture envelope",
                    overlap_fraction * 100.0
                ),
            });
            return WorkspaceObjectClassification {
                class: fixture.class,
                confidence,
                classifier_id: self.classifier_id().into(),
                review_required: false,
                evidence: observations,
            };
        }

        if let Some(role) = candidate.plate_role {
            let (suggested_class, explanation) = match role {
                InstalledPlateRole::Staging => (
                    WorkspaceObjectClass::Feeder,
                    "the region is on the calibrated staging plate",
                ),
                InstalledPlateRole::Build => (
                    WorkspaceObjectClass::Board,
                    "the region is on the calibrated build plate",
                ),
                InstalledPlateRole::Unspecified => (
                    WorkspaceObjectClass::Unknown,
                    "the installed plate has no semantic role",
                ),
            };
            observations.push(WorkspaceClassificationEvidence {
                source: "plate_role_prior".into(),
                suggested_class,
                confidence: if role == InstalledPlateRole::Unspecified {
                    0.0
                } else {
                    0.55
                },
                explanation: explanation.into(),
            });
        }
        observations.push(WorkspaceClassificationEvidence {
            source: "raised_surface_geometry".into(),
            suggested_class: WorkspaceObjectClass::Board,
            confidence: candidate.board_likelihood,
            explanation: "height, coverage, compactness, and texture support a board-like object"
                .into(),
        });
        if let Some(design_match) = candidate.design_matches.first() {
            let (source, explanation) = match design_match.registration_method {
                WorkspaceBoardRegistrationMethod::VisualOutline => (
                    format!("kicad_visual_outline:{}", design_match.board_id),
                    format!(
                        "a visual rectangle fits {} within {:.3} mm with {:.1}% edge support",
                        design_match.board_name,
                        design_match.dimension_error_mm,
                        design_match.edge_support_fraction.unwrap_or_default() * 100.0,
                    ),
                ),
                WorkspaceBoardRegistrationMethod::CandidateDimensions => (
                    format!("kicad_dimensions:{}", design_match.board_id),
                    format!(
                        "plate-aligned dimensions differ from {} by {:.3} mm",
                        design_match.board_name, design_match.dimension_error_mm
                    ),
                ),
            };
            observations.push(WorkspaceClassificationEvidence {
                source,
                suggested_class: WorkspaceObjectClass::Board,
                confidence: design_match.confidence,
                explanation,
            });
        }
        if candidate.plate_role == Some(InstalledPlateRole::Staging) {
            if let Ok(features) = detect_workspace_line_features(evidence.normalized_crop) {
                if let Some(tracks) = features.parallel_tracks() {
                    let confidence = 1.0 - (1.0 - 0.55) * (1.0 - tracks.confidence);
                    observations.push(WorkspaceClassificationEvidence {
                        source: "staging_parallel_feeder_tracks".into(),
                        suggested_class: WorkspaceObjectClass::Feeder,
                        confidence,
                        explanation: format!(
                            "the staging-plate region has {} long parallel tracks covering {:.1}% of their axis and {:.1}% of its width",
                            tracks.track_count,
                            tracks.mean_length_fraction * 100.0,
                            tracks.transverse_span_fraction * 100.0,
                        ),
                    });
                }
            }
        }

        let strongest = observations
            .iter()
            .filter(|item| item.suggested_class != WorkspaceObjectClass::Unknown)
            .max_by(|first, second| first.confidence.total_cmp(&second.confidence));
        let (class, confidence, mut review_required) = strongest
            .filter(|item| item.confidence >= 0.75)
            .map_or((WorkspaceObjectClass::Unknown, 0.5, true), |item| {
                (
                    item.suggested_class,
                    item.confidence,
                    item.confidence < 0.90,
                )
            });
        if class == WorkspaceObjectClass::Board
            && candidate
                .design_matches
                .first()
                .is_some_and(|design_match| {
                    design_match.orientation_ambiguous || candidate.design_matches.len() > 1
                })
        {
            review_required = true;
        }
        WorkspaceObjectClassification {
            class,
            confidence,
            classifier_id: self.classifier_id().into(),
            review_required,
            evidence: observations,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct WorkspaceBoardDiscoveryArtifacts {
    pub report: WorkspaceBoardDiscoveryReport,
    pub preview: RgbFrame,
    pub evidence: Vec<WorkspaceCandidateEvidenceArtifact>,
}

pub fn build_workspace_discovery_context(
    profile: &MachineProfile,
    geometry: Option<&InstalledPlateGeometryReport>,
    camera_aperture: Option<&WorkspaceCameraApertureSummary>,
) -> Result<WorkspaceDiscoveryContext> {
    let mut context = WorkspaceDiscoveryContext::default();
    let Some(installed) = profile.installed_geometry_calibration.as_ref() else {
        return Ok(context);
    };
    let visual = profile
        .visual_homing
        .as_ref()
        .ok_or_else(|| anyhow::anyhow!("installed plates require a visual datum"))?;
    for plate in &installed.plates {
        let model = geometry.and_then(|report| {
            report
                .plates
                .iter()
                .find(|fit| fit.plate_id == plate.id && fit.passed)
                .map(|fit| fit.model)
        });
        let nominal_origin = [
            visual.fiducial_x_mm + plate.origin_offset_x_mm,
            visual.fiducial_y_mm + plate.origin_offset_y_mm,
        ];
        let transform = |point: [f64; 2]| {
            model
                .map(|model| model.apply(point[0], point[1]))
                .unwrap_or(point)
        };
        let origin_machine_mm = transform(nominal_origin);
        let column_end = transform([
            nominal_origin[0] + plate.column_step_x_mm,
            nominal_origin[1] + plate.column_step_y_mm,
        ]);
        let row_end = transform([
            nominal_origin[0] + plate.row_step_x_mm,
            nominal_origin[1] + plate.row_step_y_mm,
        ]);
        let column_vector = subtract(column_end, origin_machine_mm);
        let row_vector = subtract(row_end, origin_machine_mm);
        let axis_x = normalize_vector(column_vector)
            .ok_or_else(|| anyhow::anyhow!("installed plate column axis is invalid"))?;
        let mut axis_y = [-axis_x[1], axis_x[0]];
        if dot(axis_y, row_vector) < 0.0 {
            axis_y = [-axis_y[0], -axis_y[1]];
        }
        let column_pitch = vector_length(column_vector);
        let row_pitch = dot(axis_y, row_vector).abs();
        ensure!(
            column_pitch > 0.0 && row_pitch > 0.0,
            "installed plate pitch is invalid"
        );
        context.plates.push(WorkspacePlateFrame {
            plate_id: plate.id.clone(),
            role: plate.role,
            origin_machine_mm,
            axis_x,
            axis_y,
            x_min_mm: -column_pitch,
            x_max_mm: f64::from(plate.column_count) * column_pitch,
            y_min_mm: -row_pitch,
            y_max_mm: f64::from(plate.row_count) * row_pitch,
        });
    }

    let datum_origin = geometry
        .map(|report| [report.datum_controller_x_mm, report.datum_controller_y_mm])
        .unwrap_or([visual.fiducial_x_mm, visual.fiducial_y_mm]);
    let datum_axes = context
        .plates
        .iter()
        .find(|plate| plate.role == InstalledPlateRole::Staging)
        .map(|plate| (plate.axis_x, plate.axis_y))
        .unwrap_or(([1.0, 0.0], [0.0, 1.0]));
    for fixture in &profile.workspace_fixtures {
        fixture.validate().map_err(anyhow::Error::msg)?;
        let (origin, axis_x, axis_y) = match &fixture.anchor {
            WorkspaceFixtureAnchor::VisualDatum => (datum_origin, datum_axes.0, datum_axes.1),
            WorkspaceFixtureAnchor::Plate { plate_id } => {
                let plate = context
                    .plates
                    .iter()
                    .find(|plate| &plate.plate_id == plate_id)
                    .ok_or_else(|| {
                        anyhow::anyhow!(
                            "workspace fixture {} refers to absent plate {}",
                            fixture.id,
                            plate_id
                        )
                    })?;
                (plate.origin_machine_mm, plate.axis_x, plate.axis_y)
            }
        };
        context.fixtures.push(WorkspaceResolvedFixture {
            id: fixture.id.clone(),
            class: fixture.class,
            shape: WorkspaceFixtureShape::Rectangle(local_rectangle_to_machine(
                origin,
                axis_x,
                axis_y,
                fixture.x_min_mm,
                fixture.x_max_mm,
                fixture.y_min_mm,
                fixture.y_max_mm,
            )),
        });
    }
    if let Some(aperture) = camera_aperture {
        // The aperture fit describes the optical opening. The fixed camera
        // mount, light ring, fasteners, and their parallax boundary extend
        // farther. Keep the full V3 observed assembly out of board scoring.
        let guard_radius_mm = (aperture.radius_mm + 8.0).max(40.0);
        context.fixtures.push(WorkspaceResolvedFixture {
            id: "bottom_camera_guard".into(),
            class: WorkspaceObjectClass::FixedMachineFeature,
            shape: WorkspaceFixtureShape::Circle {
                center_machine_mm: [aperture.center_x_mm, aperture.center_y_mm],
                radius_mm: guard_radius_mm,
            },
        });
    }
    Ok(context)
}

pub fn discover_workspace_boards(
    mosaic: &GrayFrame,
    raster_transform: WorkspaceRasterTransform,
    height_map: &WorkspaceHeightMap,
    board_designs: &[BoardDesign],
    source_mosaic_artifact_name: &str,
    height_sources: &[WorkspaceBoardHeightSource],
) -> Result<WorkspaceBoardDiscoveryArtifacts> {
    discover_workspace_objects(
        mosaic,
        raster_transform,
        height_map,
        board_designs,
        source_mosaic_artifact_name,
        height_sources,
        &WorkspaceDiscoveryContext::default(),
        &DeterministicWorkspaceObjectClassifier,
    )
}

#[allow(clippy::too_many_arguments)]
pub fn discover_workspace_objects(
    mosaic: &GrayFrame,
    raster_transform: WorkspaceRasterTransform,
    height_map: &WorkspaceHeightMap,
    board_designs: &[BoardDesign],
    source_mosaic_artifact_name: &str,
    height_sources: &[WorkspaceBoardHeightSource],
    context: &WorkspaceDiscoveryContext,
    classifier: &dyn WorkspaceObjectClassifier,
) -> Result<WorkspaceBoardDiscoveryArtifacts> {
    ensure!(
        mosaic.width > 0
            && mosaic.height > 0
            && mosaic.pixels.len() == mosaic.width as usize * mosaic.height as usize,
        "board discovery mosaic is invalid"
    );
    ensure!(
        raster_transform.x_mm_per_pixel.is_finite()
            && raster_transform.y_mm_per_pixel.is_finite()
            && raster_transform.x_mm_per_pixel > 0.0
            && raster_transform.y_mm_per_pixel < 0.0,
        "board discovery raster transform is invalid"
    );
    ensure!(
        height_map.cell_size_mm.is_finite() && height_map.cell_size_mm > 0.0,
        "board discovery height-map cell size is invalid"
    );
    ensure!(
        height_map.cells.len() == (height_map.rows * height_map.columns) as usize,
        "board discovery height-map cells do not match its dimensions"
    );

    let raised = height_map
        .cells
        .iter()
        .map(|cell| {
            cell.surface_fraction
                .is_some_and(|fraction| fraction <= RAISED_SURFACE_MAX_FRACTION)
                && cell.source != WorkspaceHeightCellSource::Unknown
        })
        .collect::<Vec<_>>();
    let components = connected_raised_components(&raised, height_map.rows, height_map.columns);
    let mut drafts = components
        .into_iter()
        .filter(|component| component.len() >= MINIMUM_EVIDENCE_CELLS)
        .filter_map(|component| {
            candidate_from_component(
                &component,
                mosaic,
                raster_transform,
                height_map,
                board_designs,
                context,
                classifier,
            )
        })
        .collect::<Vec<_>>();
    drafts.sort_by(|first, second| {
        second
            .candidate
            .board_likelihood
            .total_cmp(&first.candidate.board_likelihood)
            .then_with(|| {
                (second.candidate.width_mm * second.candidate.height_mm)
                    .total_cmp(&(first.candidate.width_mm * first.candidate.height_mm))
            })
    });
    let mut evidence = Vec::with_capacity(drafts.len());
    for (index, draft) in drafts.iter_mut().enumerate() {
        let artifact_name = format!("object-evidence-{index:03}.png");
        draft.candidate.candidate_index = index as u32;
        draft.candidate.evidence_artifact_name = Some(artifact_name.clone());
        evidence.push(WorkspaceCandidateEvidenceArtifact {
            artifact_name,
            frame: draft.crop.clone(),
        });
    }
    let candidates = drafts
        .into_iter()
        .map(|draft| draft.candidate)
        .collect::<Vec<_>>();

    let mut warnings = Vec::new();
    let dimensioned_board_count = board_designs
        .iter()
        .filter(|board| board.width_mm.is_some() && board.height_mm.is_some())
        .count();
    if dimensioned_board_count == 0 {
        warnings.push("no imported board has dimensions available for candidate matching".into());
    } else if !candidates
        .iter()
        .any(|candidate| !candidate.design_matches.is_empty())
    {
        warnings.push("no raised candidate matched an imported board outline".into());
    }
    let unresolved_count = candidates
        .iter()
        .filter(|candidate| {
            candidate
                .classification
                .as_ref()
                .is_none_or(|classification| classification.class == WorkspaceObjectClass::Unknown)
        })
        .count();
    if unresolved_count > 0 {
        warnings.push(format!(
            "{unresolved_count} raised regions remain unknown pending refinement, KiCad matching, or learned classification"
        ));
    }
    let report = WorkspaceBoardDiscoveryReport {
        algorithm_version: WORKSPACE_BOARD_DISCOVERY_ALGORITHM.into(),
        source_mosaic_artifact_name: source_mosaic_artifact_name.into(),
        height_sources: height_sources.to_vec(),
        installed_plate_fraction: INSTALLED_PLATE_FRACTION,
        raised_surface_max_fraction: RAISED_SURFACE_MAX_FRACTION,
        board_design_count: board_designs.len() as u32,
        candidates,
        warnings,
    };
    let preview = board_candidate_preview(mosaic, raster_transform, &report.candidates);
    Ok(WorkspaceBoardDiscoveryArtifacts {
        report,
        preview,
        evidence,
    })
}

pub fn merge_workspace_height_maps(
    region: WorkspaceScanRegion,
    maps: &[WorkspaceHeightMap],
) -> Result<WorkspaceHeightMap> {
    region.validate().map_err(anyhow::Error::msg)?;
    let first = maps
        .first()
        .ok_or_else(|| anyhow::anyhow!("board discovery has no surface-height maps"))?;
    let cell_size_mm = first.cell_size_mm;
    ensure!(
        cell_size_mm.is_finite() && cell_size_mm > 0.0,
        "board discovery height-map cell size is invalid"
    );
    ensure!(
        maps.iter().all(|map| {
            (map.cell_size_mm - cell_size_mm).abs() <= 1e-9
                && (map.candidate_step - first.candidate_step).abs() <= 1e-9
        }),
        "board discovery height maps use different grids"
    );
    let columns = ((region.x_max_mm - region.x_min_mm) / cell_size_mm)
        .ceil()
        .max(1.0) as u32;
    let rows = ((region.y_max_mm - region.y_min_mm) / cell_size_mm)
        .ceil()
        .max(1.0) as u32;
    let mut cells = Vec::with_capacity((rows * columns) as usize);
    for row in 0..rows {
        for column in 0..columns {
            let x_min = region.x_min_mm + f64::from(column) * cell_size_mm;
            let x_max = (x_min + cell_size_mm).min(region.x_max_mm);
            let y_max = region.y_max_mm - f64::from(row) * cell_size_mm;
            let y_min = (y_max - cell_size_mm).max(region.y_min_mm);
            cells.push(crate::WorkspaceHeightCell {
                row,
                column,
                center_x_mm: (x_min + x_max) * 0.5,
                center_y_mm: (y_min + y_max) * 0.5,
                surface_fraction: None,
                confidence: 0.0,
                supporting_observations: 0,
                total_observations: 0,
                residual_p95_mm: None,
                source: WorkspaceHeightCellSource::Unknown,
            });
        }
    }
    for map in maps {
        ensure!(
            map.cells.len() == (map.rows * map.columns) as usize,
            "board discovery height-map cells do not match its dimensions"
        );
        for source in map
            .cells
            .iter()
            .filter(|cell| cell.source != WorkspaceHeightCellSource::Unknown)
        {
            if source.center_x_mm < region.x_min_mm
                || source.center_x_mm > region.x_max_mm
                || source.center_y_mm < region.y_min_mm
                || source.center_y_mm > region.y_max_mm
            {
                continue;
            }
            let column = (((source.center_x_mm - region.x_min_mm) / cell_size_mm).floor() as u32)
                .min(columns - 1);
            let row = (((region.y_max_mm - source.center_y_mm) / cell_size_mm).floor() as u32)
                .min(rows - 1);
            let target = &mut cells[(row * columns + column) as usize];
            let source_priority = height_source_priority(source.source);
            let target_priority = height_source_priority(target.source);
            if source_priority < target_priority
                || (source_priority == target_priority && source.confidence <= target.confidence)
            {
                continue;
            }
            let center_x_mm = target.center_x_mm;
            let center_y_mm = target.center_y_mm;
            *target = source.clone();
            target.row = row;
            target.column = column;
            target.center_x_mm = center_x_mm;
            target.center_y_mm = center_y_mm;
        }
    }
    let measured_cells = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Measured)
        .count() as u32;
    let inferred_cells = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Inferred)
        .count() as u32;
    let unknown_cells = cells.len() as u32 - measured_cells - inferred_cells;
    let mut candidate_fractions = maps
        .iter()
        .flat_map(|map| map.candidate_fractions.iter().copied())
        .collect::<Vec<_>>();
    candidate_fractions.sort_by(f64::total_cmp);
    candidate_fractions.dedup_by(|first, second| (*first - *second).abs() <= 1e-9);
    Ok(WorkspaceHeightMap {
        region,
        cell_size_mm,
        rows,
        columns,
        candidate_step: first.candidate_step,
        minimum_surface_fraction: maps
            .iter()
            .map(|map| map.minimum_surface_fraction)
            .fold(f64::INFINITY, f64::min),
        maximum_surface_fraction: maps
            .iter()
            .map(|map| map.maximum_surface_fraction)
            .fold(f64::NEG_INFINITY, f64::max),
        measured_cells,
        inferred_cells,
        unknown_cells,
        candidate_fractions,
        cells,
    })
}

fn height_source_priority(source: WorkspaceHeightCellSource) -> u8 {
    match source {
        WorkspaceHeightCellSource::Unknown => 0,
        WorkspaceHeightCellSource::Inferred => 1,
        WorkspaceHeightCellSource::Measured => 2,
    }
}

fn connected_raised_components(mask: &[bool], rows: u32, columns: u32) -> Vec<Vec<usize>> {
    let mut visited = vec![false; mask.len()];
    let mut components = Vec::new();
    for seed in 0..mask.len() {
        if !mask[seed] || visited[seed] {
            continue;
        }
        visited[seed] = true;
        let mut queue = VecDeque::from([seed]);
        let mut component = Vec::new();
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let row = index as i32 / columns as i32;
            let column = index as i32 % columns as i32;
            for row_offset in -CONNECTION_RADIUS_CELLS..=CONNECTION_RADIUS_CELLS {
                for column_offset in -CONNECTION_RADIUS_CELLS..=CONNECTION_RADIUS_CELLS {
                    if row_offset == 0 && column_offset == 0 {
                        continue;
                    }
                    let next_row = row + row_offset;
                    let next_column = column + column_offset;
                    if next_row < 0
                        || next_column < 0
                        || next_row >= rows as i32
                        || next_column >= columns as i32
                    {
                        continue;
                    }
                    let next = (next_row as u32 * columns + next_column as u32) as usize;
                    if mask[next] && !visited[next] {
                        visited[next] = true;
                        queue.push_back(next);
                    }
                }
            }
        }
        components.push(component);
    }
    components
}

struct CandidateDraft {
    candidate: WorkspaceBoardCandidate,
    crop: GrayFrame,
}

fn candidate_from_component(
    component: &[usize],
    mosaic: &GrayFrame,
    raster_transform: WorkspaceRasterTransform,
    height_map: &WorkspaceHeightMap,
    board_designs: &[BoardDesign],
    context: &WorkspaceDiscoveryContext,
    classifier: &dyn WorkspaceObjectClassifier,
) -> Option<CandidateDraft> {
    let cells = component
        .iter()
        .map(|index| &height_map.cells[*index])
        .collect::<Vec<_>>();
    let count = cells.len() as f64;
    let mean_x = cells.iter().map(|cell| cell.center_x_mm).sum::<f64>() / count;
    let mean_y = cells.iter().map(|cell| cell.center_y_mm).sum::<f64>() / count;
    let plate = context
        .plates
        .iter()
        .find(|plate| plate_contains(plate, [mean_x, mean_y]));
    let axis_x = plate.map_or([1.0, 0.0], |plate| plate.axis_x);
    let axis_y = plate.map_or([0.0, 1.0], |plate| plate.axis_y);
    let rotation_rad = axis_x[1].atan2(axis_x[0]);
    let half_cell = height_map.cell_size_mm * 0.5;
    let support_x = half_cell * (axis_x[0].abs() + axis_x[1].abs());
    let support_y = half_cell * (axis_y[0].abs() + axis_y[1].abs());
    let mut minimum_x = f64::INFINITY;
    let mut maximum_x = f64::NEG_INFINITY;
    let mut minimum_y = f64::INFINITY;
    let mut maximum_y = f64::NEG_INFINITY;
    for cell in &cells {
        let projected_x = cell.center_x_mm * axis_x[0] + cell.center_y_mm * axis_x[1];
        let projected_y = cell.center_x_mm * axis_y[0] + cell.center_y_mm * axis_y[1];
        minimum_x = minimum_x.min(projected_x - support_x);
        maximum_x = maximum_x.max(projected_x + support_x);
        minimum_y = minimum_y.min(projected_y - support_y);
        maximum_y = maximum_y.max(projected_y + support_y);
    }
    let width_mm = maximum_x - minimum_x;
    let height_mm = maximum_y - minimum_y;
    if width_mm.min(height_mm) < MINIMUM_CANDIDATE_SPAN_MM
        || width_mm * height_mm < MINIMUM_CANDIDATE_AREA_MM2
    {
        return None;
    }
    let outline_machine_mm = [
        [minimum_x, minimum_y],
        [maximum_x, minimum_y],
        [maximum_x, maximum_y],
        [minimum_x, maximum_y],
    ]
    .into_iter()
    .map(|point| {
        [
            point[0] * axis_x[0] + point[1] * axis_y[0],
            point[0] * axis_x[1] + point[1] * axis_y[1],
        ]
    })
    .collect::<Vec<_>>();
    let region = WorkspaceScanRegion {
        x_min_mm: outline_machine_mm
            .iter()
            .map(|point| point[0])
            .fold(f64::INFINITY, f64::min),
        x_max_mm: outline_machine_mm
            .iter()
            .map(|point| point[0])
            .fold(f64::NEG_INFINITY, f64::max),
        y_min_mm: outline_machine_mm
            .iter()
            .map(|point| point[1])
            .fold(f64::INFINITY, f64::min),
        y_max_mm: outline_machine_mm
            .iter()
            .map(|point| point[1])
            .fold(f64::NEG_INFINITY, f64::max),
    };
    let center_x_mm = outline_machine_mm.iter().map(|point| point[0]).sum::<f64>() / 4.0;
    let center_y_mm = outline_machine_mm.iter().map(|point| point[1]).sum::<f64>() / 4.0;
    let measured_cell_fraction = cells
        .iter()
        .filter(|cell| cell.source == WorkspaceHeightCellSource::Measured)
        .count() as f64
        / count;
    let mean_surface_fraction = cells
        .iter()
        .filter_map(|cell| cell.surface_fraction)
        .sum::<f64>()
        / count;
    let mean_height_confidence = cells.iter().map(|cell| cell.confidence).sum::<f64>() / count;
    let fill_fraction =
        (count * height_map.cell_size_mm.powi(2) / (width_mm * height_mm)).clamp(0.0, 1.0);
    let texture_energy_luma = region_texture_energy(mosaic, raster_transform, region);
    let known_fixture = best_fixture_match(&cells, &context.fixtures);
    let crop = normalized_evidence_crop(
        mosaic,
        raster_transform,
        &outline_machine_mm,
        width_mm,
        height_mm,
    );
    let line_features = detect_workspace_line_features(&crop).ok();
    let mut design_matches = if known_fixture.is_some() {
        Vec::new()
    } else {
        board_design_matches(
            board_designs,
            width_mm,
            height_mm,
            rotation_rad.to_degrees(),
            &crop,
            &outline_machine_mm,
            line_features.as_ref(),
        )
    };
    design_matches.truncate(MAXIMUM_DESIGN_MATCHES);
    let evidence_quality =
        (0.7 * mean_height_confidence + 0.3 * measured_cell_fraction).clamp(0.0, 1.0);
    let compactness = (fill_fraction / 0.45).clamp(0.0, 1.0);
    let texture_score = (texture_energy_luma / 18.0).clamp(0.0, 1.0);
    let aspect_ratio = width_mm.max(height_mm) / width_mm.min(height_mm);
    let shape_score = (1.0 - (aspect_ratio - 1.0).max(0.0) / 10.0).clamp(0.0, 1.0);
    let base_likelihood =
        0.40 * evidence_quality + 0.25 * compactness + 0.20 * texture_score + 0.15 * shape_score;
    let design_score = design_matches
        .first()
        .map_or(0.0, |candidate| candidate.confidence);
    let mut board_likelihood = if design_matches.is_empty() {
        base_likelihood * 0.75
    } else {
        0.55 * base_likelihood + 0.45 * design_score
    }
    .clamp(0.0, 1.0);
    if known_fixture.is_some() {
        board_likelihood = 0.0;
    }

    let mut candidate = WorkspaceBoardCandidate {
        candidate_index: 0,
        region,
        outline_machine_mm,
        plate_id: plate.map(|plate| plate.plate_id.clone()),
        plate_role: plate.map(|plate| plate.role),
        center_x_mm,
        center_y_mm,
        width_mm,
        height_mm,
        rotation_deg: normalize_angle(rotation_rad.to_degrees()),
        evidence_cell_count: component.len() as u32,
        measured_cell_fraction,
        mean_surface_fraction,
        mean_height_confidence,
        fill_fraction,
        texture_energy_luma,
        board_likelihood,
        classification: None,
        known_fixture_id: known_fixture.map(|(fixture, _)| fixture.id.clone()),
        evidence_artifact_name: None,
        design_matches,
    };
    candidate.classification = Some(classifier.classify(WorkspaceObjectEvidence {
        candidate: &candidate,
        normalized_crop: &crop,
        height_map,
        component_cell_indices: component,
        known_fixture,
    }));
    Some(CandidateDraft { candidate, crop })
}

fn plate_contains(plate: &WorkspacePlateFrame, point: [f64; 2]) -> bool {
    let offset = subtract(point, plate.origin_machine_mm);
    let x = dot(offset, plate.axis_x);
    let y = dot(offset, plate.axis_y);
    (plate.x_min_mm..=plate.x_max_mm).contains(&x) && (plate.y_min_mm..=plate.y_max_mm).contains(&y)
}

fn best_fixture_match<'a>(
    cells: &[&WorkspaceHeightCell],
    fixtures: &'a [WorkspaceResolvedFixture],
) -> Option<(&'a WorkspaceResolvedFixture, f64)> {
    fixtures
        .iter()
        .map(|fixture| {
            let inside = cells
                .iter()
                .filter(|cell| fixture_contains(fixture, [cell.center_x_mm, cell.center_y_mm]))
                .count();
            (fixture, inside as f64 / cells.len().max(1) as f64)
        })
        .filter(|(_, fraction)| *fraction >= FIXTURE_MATCH_MINIMUM_FRACTION)
        .max_by(|first, second| first.1.total_cmp(&second.1))
}

fn fixture_contains(fixture: &WorkspaceResolvedFixture, point: [f64; 2]) -> bool {
    match &fixture.shape {
        WorkspaceFixtureShape::Rectangle(outline) => point_in_convex_polygon(outline, point),
        WorkspaceFixtureShape::Circle {
            center_machine_mm,
            radius_mm,
        } => vector_length(subtract(point, *center_machine_mm)) <= *radius_mm,
    }
}

fn point_in_convex_polygon(outline: &[[f64; 2]], point: [f64; 2]) -> bool {
    if outline.len() < 3 {
        return false;
    }
    let mut sign = 0.0_f64;
    for index in 0..outline.len() {
        let start = outline[index];
        let end = outline[(index + 1) % outline.len()];
        let edge = subtract(end, start);
        let relative = subtract(point, start);
        let cross = edge[0] * relative[1] - edge[1] * relative[0];
        if cross.abs() <= 1e-9 {
            continue;
        }
        if sign == 0.0 {
            sign = cross.signum();
        } else if sign != cross.signum() {
            return false;
        }
    }
    true
}

fn normalized_evidence_crop(
    mosaic: &GrayFrame,
    transform: WorkspaceRasterTransform,
    outline: &[[f64; 2]],
    width_mm: f64,
    height_mm: f64,
) -> GrayFrame {
    let source_mm_per_pixel = transform
        .x_mm_per_pixel
        .abs()
        .max(transform.y_mm_per_pixel.abs());
    let crop_mm_per_pixel = source_mm_per_pixel
        .max(width_mm / f64::from(MAXIMUM_EVIDENCE_CROP_PIXELS_PER_AXIS))
        .max(height_mm / f64::from(MAXIMUM_EVIDENCE_CROP_PIXELS_PER_AXIS));
    let width = (width_mm / crop_mm_per_pixel).ceil().max(1.0) as u32;
    let height = (height_mm / crop_mm_per_pixel).ceil().max(1.0) as u32;
    let origin = outline[0];
    let x_edge = subtract(outline[1], origin);
    let y_edge = subtract(outline[3], origin);
    let mut pixels = Vec::with_capacity((width * height) as usize);
    for row in 0..height {
        let y_fraction = (f64::from(row) + 0.5) / f64::from(height);
        for column in 0..width {
            let x_fraction = (f64::from(column) + 0.5) / f64::from(width);
            let point = [
                origin[0] + x_fraction * x_edge[0] + y_fraction * y_edge[0],
                origin[1] + x_fraction * x_edge[1] + y_fraction * y_edge[1],
            ];
            let pixel = machine_to_pixel(transform, point[0], point[1]);
            pixels.push(sample_bilinear(mosaic, pixel[0], pixel[1]));
        }
    }
    GrayFrame {
        width,
        height,
        pixels,
    }
}

fn sample_bilinear(frame: &GrayFrame, x: f64, y: f64) -> u8 {
    let x = x.clamp(0.0, f64::from(frame.width.saturating_sub(1)));
    let y = y.clamp(0.0, f64::from(frame.height.saturating_sub(1)));
    let x0 = x.floor() as u32;
    let y0 = y.floor() as u32;
    let x1 = (x0 + 1).min(frame.width - 1);
    let y1 = (y0 + 1).min(frame.height - 1);
    let x_fraction = x - f64::from(x0);
    let y_fraction = y - f64::from(y0);
    let sample =
        |column: u32, row: u32| f64::from(frame.pixels[(row * frame.width + column) as usize]);
    let top = sample(x0, y0) * (1.0 - x_fraction) + sample(x1, y0) * x_fraction;
    let bottom = sample(x0, y1) * (1.0 - x_fraction) + sample(x1, y1) * x_fraction;
    (top * (1.0 - y_fraction) + bottom * y_fraction).round() as u8
}

fn board_design_matches(
    board_designs: &[BoardDesign],
    candidate_width_mm: f64,
    candidate_height_mm: f64,
    candidate_rotation_deg: f64,
    crop: &GrayFrame,
    candidate_outline_machine_mm: &[[f64; 2]],
    line_features: Option<&WorkspaceLineFeatures>,
) -> Vec<WorkspaceBoardDesignMatch> {
    let mut matches = board_designs
        .iter()
        .filter_map(|board| {
            let (design_width_mm, design_height_mm) = (board.width_mm?, board.height_mm?);
            if design_width_mm <= 0.0 || design_height_mm <= 0.0 {
                return None;
            }
            let visual = line_features.and_then(|features| {
                visual_board_design_match(
                    board,
                    features,
                    crop,
                    candidate_outline_machine_mm,
                    candidate_width_mm,
                    candidate_height_mm,
                    candidate_rotation_deg,
                    design_width_mm,
                    design_height_mm,
                )
            });
            let orientations = [
                (design_width_mm, design_height_mm, candidate_rotation_deg),
                (
                    design_height_mm,
                    design_width_mm,
                    candidate_rotation_deg + 90.0,
                ),
            ];
            let (width, height, rotation_deg) =
                orientations.into_iter().min_by(|first, second| {
                    dimension_error(candidate_width_mm, candidate_height_mm, first.0, first.1)
                        .total_cmp(&dimension_error(
                            candidate_width_mm,
                            candidate_height_mm,
                            second.0,
                            second.1,
                        ))
                })?;
            let width_error = (candidate_width_mm - width).abs();
            let height_error = (candidate_height_mm - height).abs();
            let width_error_fraction = width_error / width;
            let height_error_fraction = height_error / height;
            let dimension_error_fraction = width_error_fraction.max(height_error_fraction);
            let width_tolerance = 4.0_f64.max(width * 0.20);
            let height_tolerance = 4.0_f64.max(height * 0.20);
            if width_error > width_tolerance || height_error > height_tolerance {
                return visual;
            }
            let normalized_error =
                0.5 * (width_error / width_tolerance + height_error / height_tolerance);
            let dimension_match = WorkspaceBoardDesignMatch {
                board_id: board.id,
                board_name: board.name.clone(),
                registration_method: WorkspaceBoardRegistrationMethod::CandidateDimensions,
                rotation_deg: normalize_angle(rotation_deg),
                dimension_error_mm: width_error.hypot(height_error),
                dimension_error_fraction,
                confidence: (1.0 - normalized_error).clamp(0.0, 1.0),
                center_x_mm: None,
                center_y_mm: None,
                outline_machine_mm: Vec::new(),
                edge_support_fraction: None,
                orientation_ambiguous: dimensions_are_orientation_ambiguous(
                    design_width_mm,
                    design_height_mm,
                ),
            };
            Some(
                visual
                    .filter(|visual| visual.confidence > dimension_match.confidence)
                    .unwrap_or(dimension_match),
            )
        })
        .collect::<Vec<_>>();
    matches.sort_by(|first, second| second.confidence.total_cmp(&first.confidence));
    matches
}

#[allow(clippy::too_many_arguments)]
fn visual_board_design_match(
    board: &BoardDesign,
    features: &WorkspaceLineFeatures,
    crop: &GrayFrame,
    candidate_outline_machine_mm: &[[f64; 2]],
    candidate_width_mm: f64,
    candidate_height_mm: f64,
    candidate_rotation_deg: f64,
    design_width_mm: f64,
    design_height_mm: f64,
) -> Option<WorkspaceBoardDesignMatch> {
    if candidate_outline_machine_mm.len() != 4 || crop.width == 0 || crop.height == 0 {
        return None;
    }
    let x_mm_per_pixel = candidate_width_mm / f64::from(crop.width);
    let y_mm_per_pixel = candidate_height_mm / f64::from(crop.height);
    let orientations = [
        (design_width_mm, design_height_mm, 0.0),
        (design_height_mm, design_width_mm, 90.0),
    ];
    orientations
        .into_iter()
        .filter_map(|(width_mm, height_mm, rotation_offset_deg)| {
            let fit = features.fit_rectangular_outline(
                width_mm,
                height_mm,
                x_mm_per_pixel,
                y_mm_per_pixel,
            )?;
            let outline_machine_mm = fit
                .corners_px
                .iter()
                .map(|point| crop_pixel_to_machine(*point, crop, candidate_outline_machine_mm))
                .collect::<Vec<_>>();
            let center = crop_pixel_to_machine(fit.center_px, crop, candidate_outline_machine_mm);
            let relative_rotation_deg =
                if dimensions_are_orientation_ambiguous(design_width_mm, design_height_mm) {
                    normalize_quarter_turn(fit.rotation_deg + rotation_offset_deg)
                } else {
                    fit.rotation_deg + rotation_offset_deg
                };
            Some(WorkspaceBoardDesignMatch {
                board_id: board.id,
                board_name: board.name.clone(),
                registration_method: WorkspaceBoardRegistrationMethod::VisualOutline,
                rotation_deg: normalize_angle(candidate_rotation_deg + relative_rotation_deg),
                dimension_error_mm: fit.dimension_error_mm,
                dimension_error_fraction: fit.dimension_error_fraction,
                confidence: fit.confidence,
                center_x_mm: Some(center[0]),
                center_y_mm: Some(center[1]),
                outline_machine_mm,
                edge_support_fraction: Some(fit.edge_support_fraction),
                orientation_ambiguous: dimensions_are_orientation_ambiguous(
                    design_width_mm,
                    design_height_mm,
                ),
            })
        })
        .max_by(|first, second| first.confidence.total_cmp(&second.confidence))
}

fn crop_pixel_to_machine(
    pixel: [f64; 2],
    crop: &GrayFrame,
    candidate_outline_machine_mm: &[[f64; 2]],
) -> [f64; 2] {
    let origin = candidate_outline_machine_mm[0];
    let x_edge = subtract(candidate_outline_machine_mm[1], origin);
    let y_edge = subtract(candidate_outline_machine_mm[3], origin);
    let x_fraction = pixel[0] / f64::from(crop.width);
    let y_fraction = pixel[1] / f64::from(crop.height);
    [
        origin[0] + x_fraction * x_edge[0] + y_fraction * y_edge[0],
        origin[1] + x_fraction * x_edge[1] + y_fraction * y_edge[1],
    ]
}

fn dimensions_are_orientation_ambiguous(width_mm: f64, height_mm: f64) -> bool {
    (width_mm - height_mm).abs() / width_mm.max(height_mm) <= 0.05
}

fn normalize_quarter_turn(mut angle_deg: f64) -> f64 {
    while angle_deg >= 45.0 {
        angle_deg -= 90.0;
    }
    while angle_deg < -45.0 {
        angle_deg += 90.0;
    }
    angle_deg
}

fn dimension_error(
    candidate_width_mm: f64,
    candidate_height_mm: f64,
    design_width_mm: f64,
    design_height_mm: f64,
) -> f64 {
    (candidate_width_mm - design_width_mm).hypot(candidate_height_mm - design_height_mm)
}

fn region_texture_energy(
    mosaic: &GrayFrame,
    transform: WorkspaceRasterTransform,
    region: WorkspaceScanRegion,
) -> f64 {
    let first = machine_to_pixel(transform, region.x_min_mm, region.y_max_mm);
    let second = machine_to_pixel(transform, region.x_max_mm, region.y_min_mm);
    let left = first[0]
        .min(second[0])
        .floor()
        .clamp(0.0, mosaic.width as f64) as u32;
    let right = first[0]
        .max(second[0])
        .ceil()
        .clamp(0.0, mosaic.width as f64) as u32;
    let top = first[1]
        .min(second[1])
        .floor()
        .clamp(0.0, mosaic.height as f64) as u32;
    let bottom = first[1]
        .max(second[1])
        .ceil()
        .clamp(0.0, mosaic.height as f64) as u32;
    if right.saturating_sub(left) < 2 || bottom.saturating_sub(top) < 2 {
        return 0.0;
    }
    let mut total = 0_u64;
    let mut samples = 0_u64;
    for y in top..bottom - 1 {
        for x in left..right - 1 {
            let index = (y * mosaic.width + x) as usize;
            total += u64::from(mosaic.pixels[index].abs_diff(mosaic.pixels[index + 1]));
            total += u64::from(
                mosaic.pixels[index].abs_diff(mosaic.pixels[index + mosaic.width as usize]),
            );
            samples += 2;
        }
    }
    total as f64 / samples.max(1) as f64
}

fn board_candidate_preview(
    mosaic: &GrayFrame,
    transform: WorkspaceRasterTransform,
    candidates: &[WorkspaceBoardCandidate],
) -> RgbFrame {
    let mut pixels = Vec::with_capacity(mosaic.pixels.len() * 3);
    for luma in &mosaic.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut preview = RgbFrame {
        width: mosaic.width,
        height: mosaic.height,
        pixels,
    };
    for candidate in candidates {
        let class = candidate
            .classification
            .as_ref()
            .map(|classification| classification.class)
            .unwrap_or_default();
        let visual_outline = candidate.design_matches.first().filter(|design_match| {
            design_match.registration_method == WorkspaceBoardRegistrationMethod::VisualOutline
                && design_match.outline_machine_mm.len() >= 3
        });
        let color = match class {
            WorkspaceObjectClass::Board => [55, 230, 135],
            WorkspaceObjectClass::Feeder => [45, 190, 255],
            WorkspaceObjectClass::ToolingStation => [190, 95, 255],
            WorkspaceObjectClass::CalibrationFixture => [210, 85, 220],
            WorkspaceObjectClass::FixedMachineFeature => [120, 120, 255],
            WorkspaceObjectClass::Unknown => [255, 174, 45],
        };
        let proposal_color = if visual_outline.is_some() {
            [120, 155, 120]
        } else {
            color
        };
        for index in 0..candidate.outline_machine_mm.len() {
            let first = candidate.outline_machine_mm[index];
            let second =
                candidate.outline_machine_mm[(index + 1) % candidate.outline_machine_mm.len()];
            let first = machine_to_pixel(transform, first[0], first[1]);
            let second = machine_to_pixel(transform, second[0], second[1]);
            draw_line(&mut preview, first, second, proposal_color);
        }
        if let Some(design_match) = visual_outline {
            for index in 0..design_match.outline_machine_mm.len() {
                let first = design_match.outline_machine_mm[index];
                let second = design_match.outline_machine_mm
                    [(index + 1) % design_match.outline_machine_mm.len()];
                let first = machine_to_pixel(transform, first[0], first[1]);
                let second = machine_to_pixel(transform, second[0], second[1]);
                draw_line(&mut preview, first, second, color);
            }
        }
    }
    preview
}

fn machine_to_pixel(transform: WorkspaceRasterTransform, x_mm: f64, y_mm: f64) -> [f64; 2] {
    [
        (x_mm - transform.top_left_x_mm) / transform.x_mm_per_pixel,
        (y_mm - transform.top_left_y_mm) / transform.y_mm_per_pixel,
    ]
}

fn draw_line(frame: &mut RgbFrame, start: [f64; 2], end: [f64; 2], color: [u8; 3]) {
    let steps = (end[0] - start[0])
        .abs()
        .max((end[1] - start[1]).abs())
        .ceil() as u32;
    for step in 0..=steps.max(1) {
        let fraction = f64::from(step) / f64::from(steps.max(1));
        let x = (start[0] + fraction * (end[0] - start[0])).round() as i32;
        let y = (start[1] + fraction * (end[1] - start[1])).round() as i32;
        for offset_y in -1..=1 {
            for offset_x in -1..=1 {
                let target_x = x + offset_x;
                let target_y = y + offset_y;
                if target_x < 0
                    || target_y < 0
                    || target_x >= frame.width as i32
                    || target_y >= frame.height as i32
                {
                    continue;
                }
                let index = (target_y as u32 * frame.width + target_x as u32) as usize * 3;
                frame.pixels[index..index + 3].copy_from_slice(&color);
            }
        }
    }
}

fn local_rectangle_to_machine(
    origin: [f64; 2],
    axis_x: [f64; 2],
    axis_y: [f64; 2],
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
) -> Vec<[f64; 2]> {
    [
        [x_min, y_min],
        [x_max, y_min],
        [x_max, y_max],
        [x_min, y_max],
    ]
    .into_iter()
    .map(|point| {
        [
            origin[0] + point[0] * axis_x[0] + point[1] * axis_y[0],
            origin[1] + point[0] * axis_x[1] + point[1] * axis_y[1],
        ]
    })
    .collect()
}

fn subtract(first: [f64; 2], second: [f64; 2]) -> [f64; 2] {
    [first[0] - second[0], first[1] - second[1]]
}

fn dot(first: [f64; 2], second: [f64; 2]) -> f64 {
    first[0] * second[0] + first[1] * second[1]
}

fn vector_length(vector: [f64; 2]) -> f64 {
    vector[0].hypot(vector[1])
}

fn normalize_vector(vector: [f64; 2]) -> Option<[f64; 2]> {
    let length = vector_length(vector);
    (length > 1e-9).then_some([vector[0] / length, vector[1] / length])
}

fn normalize_angle(mut angle_deg: f64) -> f64 {
    while angle_deg >= 180.0 {
        angle_deg -= 360.0;
    }
    while angle_deg < -180.0 {
        angle_deg += 360.0;
    }
    angle_deg
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WorkspaceHeightCell;
    use uuid::Uuid;
    use velocitypnp_domain::WorkspaceScanRegion;

    fn synthetic_map() -> WorkspaceHeightMap {
        let region = WorkspaceScanRegion {
            x_min_mm: 0.0,
            x_max_mm: 40.0,
            y_min_mm: 0.0,
            y_max_mm: 30.0,
        };
        let rows = 15;
        let columns = 20;
        let cells = (0..rows)
            .flat_map(|row| {
                (0..columns).map(move |column| {
                    let board = (4..10).contains(&row) && (3..13).contains(&column);
                    WorkspaceHeightCell {
                        row,
                        column,
                        center_x_mm: f64::from(column) * 2.0 + 1.0,
                        center_y_mm: 29.0 - f64::from(row) * 2.0,
                        surface_fraction: Some(if board { 0.50 } else { 1.0 }),
                        confidence: if board { 0.9 } else { 1.0 },
                        supporting_observations: 4,
                        total_observations: 4,
                        residual_p95_mm: Some(0.01),
                        source: WorkspaceHeightCellSource::Measured,
                    }
                })
            })
            .collect();
        WorkspaceHeightMap {
            region,
            cell_size_mm: 2.0,
            rows,
            columns,
            candidate_step: 0.125,
            minimum_surface_fraction: -1.0,
            maximum_surface_fraction: 2.0,
            measured_cells: rows * columns,
            inferred_cells: 0,
            unknown_cells: 0,
            candidate_fractions: vec![0.5, 1.0],
            cells,
        }
    }

    fn board(width_mm: f64, height_mm: f64) -> BoardDesign {
        BoardDesign {
            id: Uuid::new_v4(),
            name: "fixture".into(),
            source_format: "kicad_pcb".into(),
            source_version: Some(20240108),
            width_mm: Some(width_mm),
            height_mm: Some(height_mm),
            placements: Vec::new(),
            warnings: Vec::new(),
        }
    }

    #[test]
    fn raised_rectangle_becomes_a_machine_space_board_candidate() {
        let map = synthetic_map();
        let mosaic = GrayFrame {
            width: 200,
            height: 150,
            pixels: (0..30_000).map(|index| (index % 251) as u8).collect(),
        };
        let transform = WorkspaceRasterTransform {
            top_left_x_mm: 0.0,
            top_left_y_mm: 30.0,
            x_mm_per_pixel: 0.2,
            y_mm_per_pixel: -0.2,
        };

        let artifacts = discover_workspace_boards(
            &mosaic,
            transform,
            &map,
            &[board(20.0, 12.0)],
            "mosaic-refined.png",
            &[WorkspaceBoardHeightSource {
                scan_id: Uuid::nil(),
                artifact_name: "surface-height-map.json".into(),
            }],
        )
        .unwrap();

        assert_eq!(artifacts.report.candidates.len(), 1);
        let candidate = &artifacts.report.candidates[0];
        assert!((candidate.width_mm - 20.0).abs() < 1e-6);
        assert!((candidate.height_mm - 12.0).abs() < 1e-6);
        assert_eq!(candidate.region.x_min_mm, 6.0);
        assert_eq!(candidate.region.x_max_mm, 26.0);
        assert_eq!(candidate.region.y_min_mm, 10.0);
        assert_eq!(candidate.region.y_max_mm, 22.0);
        assert_eq!(candidate.design_matches.len(), 1);
        assert_eq!(candidate.design_matches[0].board_name, "fixture");
        assert_eq!(candidate.design_matches[0].confidence, 1.0);
        assert_eq!(
            candidate.classification.as_ref().unwrap().class,
            WorkspaceObjectClass::Board
        );
        assert_eq!(
            candidate.evidence_artifact_name.as_deref(),
            Some("object-evidence-000.png")
        );
        assert_eq!(artifacts.evidence.len(), 1);
        assert!(!artifacts.evidence[0].frame.pixels.is_empty());
        assert_eq!(artifacts.preview.width, mosaic.width);
        assert_eq!(artifacts.preview.height, mosaic.height);
    }

    #[test]
    fn proposal_outline_uses_the_calibrated_plate_axes() {
        let angle = 8.0_f64.to_radians();
        let context = WorkspaceDiscoveryContext {
            plates: vec![WorkspacePlateFrame {
                plate_id: "staging".into(),
                role: InstalledPlateRole::Staging,
                origin_machine_mm: [0.0, 0.0],
                axis_x: [angle.cos(), angle.sin()],
                axis_y: [-angle.sin(), angle.cos()],
                x_min_mm: -100.0,
                x_max_mm: 100.0,
                y_min_mm: -100.0,
                y_max_mm: 100.0,
            }],
            fixtures: Vec::new(),
        };
        let map = synthetic_map();
        let artifacts = discover_workspace_objects(
            &GrayFrame {
                width: 200,
                height: 150,
                pixels: vec![128; 30_000],
            },
            WorkspaceRasterTransform {
                top_left_x_mm: 0.0,
                top_left_y_mm: 30.0,
                x_mm_per_pixel: 0.2,
                y_mm_per_pixel: -0.2,
            },
            &map,
            &[],
            "mosaic.png",
            &[],
            &context,
            &DeterministicWorkspaceObjectClassifier,
        )
        .unwrap();

        let candidate = &artifacts.report.candidates[0];
        let first_edge = subtract(
            candidate.outline_machine_mm[1],
            candidate.outline_machine_mm[0],
        );
        let edge_angle = first_edge[1].atan2(first_edge[0]).to_degrees();
        assert!((edge_angle - 8.0).abs() < 1e-9);
        assert_eq!(candidate.plate_id.as_deref(), Some("staging"));
        assert_eq!(candidate.plate_role, Some(InstalledPlateRole::Staging));
    }

    #[test]
    fn known_fixture_is_inventory_evidence_not_a_board() {
        let context = WorkspaceDiscoveryContext {
            plates: Vec::new(),
            fixtures: vec![WorkspaceResolvedFixture {
                id: "tooling".into(),
                class: WorkspaceObjectClass::ToolingStation,
                shape: WorkspaceFixtureShape::Rectangle(vec![
                    [4.0, 8.0],
                    [28.0, 8.0],
                    [28.0, 24.0],
                    [4.0, 24.0],
                ]),
            }],
        };
        let map = synthetic_map();
        let artifacts = discover_workspace_objects(
            &GrayFrame {
                width: 40,
                height: 30,
                pixels: vec![128; 1_200],
            },
            WorkspaceRasterTransform {
                top_left_x_mm: 0.0,
                top_left_y_mm: 30.0,
                x_mm_per_pixel: 1.0,
                y_mm_per_pixel: -1.0,
            },
            &map,
            &[board(20.0, 12.0)],
            "mosaic.png",
            &[],
            &context,
            &DeterministicWorkspaceObjectClassifier,
        )
        .unwrap();

        let candidate = &artifacts.report.candidates[0];
        assert_eq!(candidate.known_fixture_id.as_deref(), Some("tooling"));
        assert_eq!(candidate.board_likelihood, 0.0);
        assert!(candidate.design_matches.is_empty());
        assert_eq!(
            candidate.classification.as_ref().unwrap().class,
            WorkspaceObjectClass::ToolingStation
        );
        assert!(!candidate.classification.as_ref().unwrap().review_required);
    }

    #[test]
    fn bare_plate_and_isolated_height_noise_are_not_candidates() {
        let mut map = synthetic_map();
        for cell in &mut map.cells {
            cell.surface_fraction = Some(1.0);
        }
        map.cells[0].surface_fraction = Some(0.0);

        let artifacts = discover_workspace_boards(
            &GrayFrame {
                width: 40,
                height: 30,
                pixels: vec![128; 1_200],
            },
            WorkspaceRasterTransform {
                top_left_x_mm: 0.0,
                top_left_y_mm: 30.0,
                x_mm_per_pixel: 1.0,
                y_mm_per_pixel: -1.0,
            },
            &map,
            &[],
            "mosaic.png",
            &[WorkspaceBoardHeightSource {
                scan_id: Uuid::nil(),
                artifact_name: "surface-height-map.json".into(),
            }],
        )
        .unwrap();

        assert!(artifacts.report.candidates.is_empty());
    }

    #[test]
    fn child_height_maps_merge_into_the_parent_machine_grid() {
        let map = synthetic_map();
        let merged = merge_workspace_height_maps(
            WorkspaceScanRegion {
                x_min_mm: -10.0,
                x_max_mm: 50.0,
                y_min_mm: -10.0,
                y_max_mm: 40.0,
            },
            &[map],
        )
        .unwrap();

        assert_eq!(merged.columns, 30);
        assert_eq!(merged.rows, 25);
        assert_eq!(merged.measured_cells, 300);
        assert_eq!(merged.unknown_cells, 450);
        let board_cells = merged
            .cells
            .iter()
            .filter(|cell| cell.surface_fraction == Some(0.5))
            .count();
        assert_eq!(board_cells, 60);
    }
}
