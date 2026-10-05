use crate::opencv_backend::gray_frame_to_mat;
use crate::{GrayFrame, RgbFrame};
use anyhow::{ensure, Result};
use opencv::core::{self, Mat, MatTraitConst, Point, Point2f, Rect, Size, Vector, CV_32S};
use opencv::imgproc;
use std::f64::consts::PI;
use velocitypnp_domain::{
    PlanarCameraModel, Pose, SourceComponentModelPrediction, StripFeederLaneProposal,
    StripFeederLiveHoldoutReport, StripFeederPocketObservation, StripFeederPocketReturnReport,
    StripFeederRegistration, StripFeederRejectedLattice, StripFeederSourcePoseReport,
    StripPocketOccupancy, WorkspaceBoardCandidate, WorkspaceRasterTransform,
};

pub const STRIP_FEEDER_VISION_ALGORITHM: &str = "strip_feeder_vision.v6";
pub const STRIP_FEEDER_LIVE_HOLDOUT_ALGORITHM: &str = "strip_feeder_live_holdout.v3";
pub const STRIP_FEEDER_POCKET_RETURN_ALGORITHM: &str = "strip_feeder_pocket_return.v4";
pub const STRIP_FEEDER_SOURCE_POSE_ALGORITHM: &str = "strip_feeder_source_pose.v3";
pub const STRIP_FEEDER_SOURCE_COMPONENT_DRAFT_ALGORITHM: &str =
    "strip_feeder_source_component_draft.v1";
pub const STRIP_FEEDER_SPROCKET_PITCH_MM: f64 = 4.0;

const LIVE_HOLDOUT_CROSS_MARGIN_MM: f64 = 3.0;
const MINIMUM_POCKET_RETURN_APPEARANCE_SIMILARITY: f64 = 0.55;
const MINIMUM_SOURCE_POSE_APPEARANCE_SIMILARITY: f64 = 0.55;
const POCKET_RETURN_INTERIOR_SCALE: f64 = 0.60;
const MAXIMUM_POCKET_RETURN_APPEARANCE_SHIFT_MM: f64 = 0.16;
const MAXIMUM_POCKET_RETURN_APPEARANCE_ROTATION_DEG: i32 = 15;
const LIVE_HOLDOUT_ALONG_SPAN_MM: f64 = 20.0;
const LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL: f64 = 0.04;
const LIVE_HOLDOUT_TRACK_TOLERANCE_MM: f64 = 1.25;
const LIVE_HOLDOUT_MAX_CORRECTION_MM: f64 = 1.5;
const LIVE_HOLDOUT_MAX_ROTATION_DEG: f64 = 3.0;
const LIVE_HOLDOUT_MAX_PITCH_RESIDUAL_MM: f64 = 0.35;

const SPROCKET_DIAMETER_MIN_MM: f64 = 1.2;
const SPROCKET_DIAMETER_MAX_MM: f64 = 2.2;
const SPROCKET_TRACK_TOLERANCE_MM: f64 = 0.75;
const SPROCKET_LATTICE_TOLERANCE_MM: f64 = 0.45;
const MINIMUM_SPROCKETS_PER_TRACK: usize = 5;
const MINIMUM_LIVE_HOLDOUT_SPROCKETS: usize = 3;
const EIGHT_MM_POCKET_CENTER_OFFSET_MM: f64 = 3.5;
const POCKET_ROW_PAIR_TOLERANCE_MM: f64 = 0.45;
const STANDARD_TAPE_WIDTHS_MM: [f64; 4] = [8.0, 12.0, 16.0, 24.0];
const STANDARD_COMPONENT_PITCHES_MM: [f64; 5] = [2.0, 4.0, 8.0, 12.0, 16.0];
// Sprocket fits can stop slightly more than half a component pitch from a
// visible terminal pocket. The image-patch check below must also support the
// extra slot. The two directions have different crop and lighting residuals.
const NEGATIVE_ENDPOINT_RANGE_PADDING_FRACTION: f64 = 0.5425;
const POSITIVE_ENDPOINT_RANGE_PADDING_FRACTION: f64 = 0.53;
const ENDPOINT_PATCH_RADIUS_MM: f64 = 1.3;
const MINIMUM_ENDPOINT_PATCH_CORRELATION: f64 = 0.43;
const MINIMUM_SECOND_ENDPOINT_PATCH_CORRELATION: f64 = 0.40;
const MINIMUM_COMPONENT_OUTLINE_SCORE: f64 = 0.43;
// EIA-481-C Table 2 dimension F is 3.5, 5.5, 7.5, and 11.5 mm for
// 8, 12, 16, and 24 mm carrier tape.
const POCKET_CENTERLINE_EDGE_REMAINDER_MM: f64 = 0.5;

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederVisionArtifacts {
    pub proposals: Vec<StripFeederLaneProposal>,
    pub rejected_lattices: Vec<StripFeederRejectedLattice>,
    pub preview: RgbFrame,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederLiveHoldoutArtifacts {
    pub report: StripFeederLiveHoldoutReport,
    pub preview: RgbFrame,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederPocketReturnArtifacts {
    pub report: StripFeederPocketReturnReport,
    pub preview: RgbFrame,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederSourcePoseArtifacts {
    pub report: StripFeederSourcePoseReport,
    pub preview: RgbFrame,
}

/// A review-only component label made from one raw source-pocket image.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct StripFeederSourceComponentDraft {
    pub algorithm_version: String,
    pub occupancy: StripPocketOccupancy,
    pub occupancy_confidence: f64,
    pub expected_size_mm: Option<[f64; 2]>,
    pub measured_size_mm: Option<[f64; 2]>,
    pub center_px: Option<[f64; 2]>,
    pub outline_px: Option<[[f64; 2]; 4]>,
    /// Rotation in the rectified crop. The first draft is aligned to the tape
    /// axes and cannot resolve a directed 180-degree package orientation.
    pub rotation_deg: Option<f64>,
    pub review_required: bool,
    pub findings: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct StripFeederSourceComponentDraftArtifacts {
    pub report: StripFeederSourceComponentDraft,
    pub crop: GrayFrame,
}

/// Rectifies one feeder pocket from a machine-coordinate workspace mosaic.
///
/// This supports review-only batch inference after a targeted lane rescan.
pub fn crop_workspace_source_component(
    mosaic: &GrayFrame,
    raster: WorkspaceRasterTransform,
    pocket_machine_mm: [f64; 2],
    tape_rotation_deg: f64,
    tape_width_mm: f64,
    tape_pitch_mm: f64,
) -> Result<GrayFrame> {
    ensure!(
        mosaic.width >= 2
            && mosaic.height >= 2
            && mosaic.pixels.len() == mosaic.width as usize * mosaic.height as usize,
        "workspace source mosaic is invalid"
    );
    ensure!(
        raster.x_mm_per_pixel.is_finite()
            && raster.y_mm_per_pixel.is_finite()
            && raster.x_mm_per_pixel > 0.0
            && raster.y_mm_per_pixel < 0.0,
        "workspace source raster transform is invalid"
    );
    ensure!(
        pocket_machine_mm.iter().all(|value| value.is_finite())
            && tape_rotation_deg.is_finite()
            && tape_width_mm.is_finite()
            && tape_width_mm >= 8.0
            && tape_pitch_mm.is_finite()
            && tape_pitch_mm >= 2.0,
        "workspace source pocket geometry is invalid"
    );
    let radians = tape_rotation_deg.to_radians();
    let along = [radians.cos(), radians.sin()];
    let cross = [-along[1], along[0]];
    let cross_span_mm = tape_width_mm + 2.0 * LIVE_HOLDOUT_CROSS_MARGIN_MM;
    let along_span_mm = (tape_pitch_mm * 3.0).max(12.0);
    let output_mm_per_pixel = raster.x_mm_per_pixel.abs().max(raster.y_mm_per_pixel.abs());
    ensure!(
        output_mm_per_pixel <= 0.08,
        "workspace source mosaic resolution is too low for component inference"
    );
    let width = (cross_span_mm / output_mm_per_pixel).round().max(64.0) as u32;
    let height = (along_span_mm / output_mm_per_pixel).round().max(64.0) as u32;
    let machine_to_pixel = |machine: [f64; 2]| {
        [
            (machine[0] - raster.top_left_x_mm) / raster.x_mm_per_pixel,
            (machine[1] - raster.top_left_y_mm) / raster.y_mm_per_pixel,
        ]
    };
    let corner_pixels = [
        [-0.5 * cross_span_mm, -0.5 * along_span_mm],
        [0.5 * cross_span_mm, -0.5 * along_span_mm],
        [0.5 * cross_span_mm, 0.5 * along_span_mm],
        [-0.5 * cross_span_mm, 0.5 * along_span_mm],
    ]
    .map(|offset| {
        machine_to_pixel(add(
            pocket_machine_mm,
            add(scale(cross, offset[0]), scale(along, offset[1])),
        ))
    });
    ensure!(
        corner_pixels.iter().all(|pixel| {
            pixel[0] >= 0.0
                && pixel[1] >= 0.0
                && pixel[0] < f64::from(mosaic.width - 1)
                && pixel[1] < f64::from(mosaic.height - 1)
        }),
        "workspace source pocket crop is outside the rescan mosaic"
    );

    let mut pixels = Vec::with_capacity((width * height) as usize);
    for y in 0..height {
        let along_mm =
            (f64::from(y) + 0.5) / f64::from(height) * along_span_mm - 0.5 * along_span_mm;
        for x in 0..width {
            let cross_mm =
                (f64::from(x) + 0.5) / f64::from(width) * cross_span_mm - 0.5 * cross_span_mm;
            let machine = add(
                pocket_machine_mm,
                add(scale(cross, cross_mm), scale(along, along_mm)),
            );
            let pixel = machine_to_pixel(machine);
            pixels.push(bilinear_luma(mosaic, pixel[0], pixel[1]));
        }
    }
    Ok(GrayFrame {
        width,
        height,
        pixels,
    })
}

/// Rectifies one source pocket and produces a draft component label.
///
/// This output is weak supervision. It must not become ground truth until an
/// operator accepts or corrects it.
pub fn draft_strip_feeder_source_component(
    frame: &GrayFrame,
    camera_pose: Pose,
    pocket_machine_mm: [f64; 2],
    camera_model: &PlanarCameraModel,
    registration: &StripFeederRegistration,
) -> Result<StripFeederSourceComponentDraftArtifacts> {
    ensure!(
        camera_model.role == velocitypnp_domain::CameraRole::TopCamera,
        "source component drafting requires a top-camera model"
    );
    let along = normalized(subtract(
        registration.pick_centerline_machine_mm[1],
        registration.pick_centerline_machine_mm[0],
    ))
    .ok_or_else(|| anyhow::anyhow!("source component centerline has no direction"))?;
    let cross = [-along[1], along[0]];
    let cross_span_mm = registration.tape_width_mm + 2.0 * LIVE_HOLDOUT_CROSS_MARGIN_MM;
    let along_span_mm = (registration.tape_pitch_mm * 3.0).max(12.0);
    let crop = rectify_live_tape_frames(
        std::slice::from_ref(frame),
        &[camera_pose],
        camera_model,
        pocket_machine_mm,
        cross,
        along,
        cross_span_mm,
        along_span_mm,
    )?;
    let nominal_center_mm = [cross_span_mm * 0.5, along_span_mm * 0.5];
    let pocket_center = measure_pocket_image_center(
        &crop,
        nominal_center_mm[0],
        nominal_center_mm[1],
        cross_span_mm,
        along_span_mm,
        registration.tape_pitch_mm,
        registration.tape_width_mm,
    );
    let expected_size_mm = registration
        .surveyed_part_size_mm
        .or(registration.measured_part_size_mm);
    let package = pocket_center
        .zip(expected_size_mm)
        .and_then(|(center, expected)| {
            measure_source_package(&crop, center, cross_span_mm, along_span_mm, expected)
        });
    let measured_center_mm = package
        .map(|package| package.center_mm)
        .or_else(|| pocket_center.map(|center| center.center_mm));
    let occupancy_confidence = measured_center_mm
        .map(|center| {
            pocket_occupancy_evidence(
                &crop,
                center[0],
                center[1],
                cross_span_mm,
                along_span_mm,
                registration.tape_pitch_mm,
            )
        })
        .unwrap_or_default();
    let occupancy = pocket_occupancy_from_evidence(occupancy_confidence);
    let center_px = package
        .map(|package| local_mm_to_pixel(&crop, package.center_mm, cross_span_mm, along_span_mm));
    let outline_px = package.map(|package| {
        let half_cross = 0.5 * package.size_mm[0];
        let half_along = 0.5 * package.size_mm[1];
        [
            [-half_cross, -half_along],
            [half_cross, -half_along],
            [half_cross, half_along],
            [-half_cross, half_along],
        ]
        .map(|offset| {
            local_mm_to_pixel(
                &crop,
                [
                    package.center_mm[0] + offset[0],
                    package.center_mm[1] + offset[1],
                ],
                cross_span_mm,
                along_span_mm,
            )
        })
    });
    let review_required = !matches!(
        (occupancy, package),
        (StripPocketOccupancy::Occupied, Some(_)) | (StripPocketOccupancy::Empty, None)
    );
    let mut findings = vec![
        "the label is an automatic draft and requires operator review".into(),
        "the crop is rectified to the carrier-tape cross and travel axes".into(),
    ];
    if package.is_some() {
        findings.push(
            "the box rotation uses the tape axis and does not encode package polarity".into(),
        );
    } else {
        findings.push("no bounded component outline was measured".into());
    }
    Ok(StripFeederSourceComponentDraftArtifacts {
        report: StripFeederSourceComponentDraft {
            algorithm_version: STRIP_FEEDER_SOURCE_COMPONENT_DRAFT_ALGORITHM.into(),
            occupancy,
            occupancy_confidence,
            expected_size_mm,
            measured_size_mm: package.map(|package| package.size_mm),
            center_px,
            outline_px,
            rotation_deg: package.map(|_| 0.0),
            review_required,
            findings,
        },
        crop,
    })
}

/// Maps one point in a rectified source-component crop to machine XY.
pub fn source_component_crop_pixel_to_machine(
    crop: &GrayFrame,
    point_px: [f64; 2],
    pocket_machine_mm: [f64; 2],
    registration: &StripFeederRegistration,
) -> Result<[f64; 2]> {
    ensure!(
        crop.width > 0 && crop.height > 0,
        "source component crop is empty"
    );
    ensure!(
        point_px.iter().all(|value| value.is_finite())
            && (0.0..=f64::from(crop.width)).contains(&point_px[0])
            && (0.0..=f64::from(crop.height)).contains(&point_px[1]),
        "source component point is outside its rectified crop"
    );
    let along = normalized(subtract(
        registration.pick_centerline_machine_mm[1],
        registration.pick_centerline_machine_mm[0],
    ))
    .ok_or_else(|| anyhow::anyhow!("source component centerline has no direction"))?;
    let cross = [-along[1], along[0]];
    let cross_span_mm = registration.tape_width_mm + 2.0 * LIVE_HOLDOUT_CROSS_MARGIN_MM;
    let along_span_mm = (registration.tape_pitch_mm * 3.0).max(12.0);
    let cross_offset_mm = point_px[0] / f64::from(crop.width) * cross_span_mm - 0.5 * cross_span_mm;
    let along_offset_mm =
        point_px[1] / f64::from(crop.height) * along_span_mm - 0.5 * along_span_mm;
    Ok(add(
        pocket_machine_mm,
        add(scale(cross, cross_offset_mm), scale(along, along_offset_mm)),
    ))
}

/// Draw deterministic and learned outlines on the exact model input crop.
pub fn render_source_component_shadow_preview(
    crop: &GrayFrame,
    deterministic_outline_px: Option<[[f64; 2]; 4]>,
    prediction: &SourceComponentModelPrediction,
    review_required: bool,
) -> RgbFrame {
    let mut pixels = Vec::with_capacity(crop.pixels.len() * 3);
    for luma in &crop.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut preview = RgbFrame {
        width: crop.width,
        height: crop.height,
        pixels,
    };
    if let Some(outline) = deterministic_outline_px {
        draw_closed_polyline(&mut preview, &outline, [50, 235, 120]);
    }
    if prediction.class.occupied() {
        draw_closed_polyline(&mut preview, &prediction.pose.outline_px, [30, 220, 245]);
        draw_cross(&mut preview, prediction.pose.center_px, 5.0, [30, 220, 245]);
    } else {
        draw_cross(&mut preview, prediction.pose.center_px, 5.0, [245, 80, 70]);
    }
    if review_required && crop.width > 2 && crop.height > 2 {
        draw_closed_polyline(
            &mut preview,
            &[
                [1.0, 1.0],
                [f64::from(crop.width - 2), 1.0],
                [f64::from(crop.width - 2), f64::from(crop.height - 2)],
                [1.0, f64::from(crop.height - 2)],
            ],
            [245, 40, 210],
        );
    }
    preview
}

/// Compares a source package with a prior package that passed pickup.
#[allow(clippy::too_many_arguments)]
pub fn inspect_strip_feeder_source_pose(
    reference: &GrayFrame,
    reference_camera_pose: Pose,
    reference_pocket_machine_mm: [f64; 2],
    reference_qualification_id: uuid::Uuid,
    candidate: &GrayFrame,
    candidate_camera_pose: Pose,
    candidate_pocket_machine_mm: [f64; 2],
    camera_model: &PlanarCameraModel,
    registration: &StripFeederRegistration,
    allow_face_texture_change: bool,
    artifact_name: &str,
) -> Result<StripFeederSourcePoseArtifacts> {
    ensure!(
        camera_model.role == velocitypnp_domain::CameraRole::TopCamera,
        "source pose inspection requires a top-camera model"
    );
    ensure!(
        reference.width == candidate.width && reference.height == candidate.height,
        "source pose frames changed dimensions"
    );
    let along = normalized(subtract(
        registration.pick_centerline_machine_mm[1],
        registration.pick_centerline_machine_mm[0],
    ))
    .ok_or_else(|| anyhow::anyhow!("source pose centerline has no direction"))?;
    let cross = [-along[1], along[0]];
    let cross_span_mm = registration.tape_width_mm + 2.0 * LIVE_HOLDOUT_CROSS_MARGIN_MM;
    let along_span_mm = (registration.tape_pitch_mm * 3.0).max(12.0);
    let rectify = |frame: &GrayFrame, pose: Pose, center: [f64; 2]| {
        rectify_live_tape_frames(
            std::slice::from_ref(frame),
            &[pose],
            camera_model,
            center,
            cross,
            along,
            cross_span_mm,
            along_span_mm,
        )
    };
    let reference_crop = rectify(
        reference,
        reference_camera_pose,
        reference_pocket_machine_mm,
    )?;
    let candidate_crop = rectify(
        candidate,
        candidate_camera_pose,
        candidate_pocket_machine_mm,
    )?;
    let center_cross_mm = cross_span_mm * 0.5;
    let center_along_mm = along_span_mm * 0.5;
    let measure = |crop: &GrayFrame| {
        measure_pocket_image_center(
            crop,
            center_cross_mm,
            center_along_mm,
            cross_span_mm,
            along_span_mm,
            registration.tape_pitch_mm,
            registration.tape_width_mm,
        )
    };
    let reference_center = measure(&reference_crop);
    let candidate_center = measure(&candidate_crop);
    let expected_package_size_mm = registration
        .surveyed_part_size_mm
        .or(registration.measured_part_size_mm);
    let reference_package =
        reference_center
            .zip(expected_package_size_mm)
            .and_then(|(center, expected)| {
                measure_source_package(
                    &reference_crop,
                    center,
                    cross_span_mm,
                    along_span_mm,
                    expected,
                )
            });
    let candidate_package =
        candidate_center
            .zip(expected_package_size_mm)
            .and_then(|(center, expected)| {
                measure_source_package(
                    &candidate_crop,
                    center,
                    cross_span_mm,
                    along_span_mm,
                    expected,
                )
            });
    let local_center = |package: Option<SourcePackageMeasurement>,
                        measurement: Option<PocketImageCenter>| {
        package
            .map(|package| package.center_mm)
            .or_else(|| measurement.map(|measurement| measurement.center_mm))
            .unwrap_or([center_cross_mm, center_along_mm])
    };
    let reference_local_center = local_center(reference_package, reference_center);
    let candidate_local_center = local_center(candidate_package, candidate_center);
    let reference_confidence = pocket_occupancy_evidence(
        &reference_crop,
        reference_local_center[0],
        reference_local_center[1],
        cross_span_mm,
        along_span_mm,
        registration.tape_pitch_mm,
    );
    let candidate_confidence = pocket_occupancy_evidence(
        &candidate_crop,
        candidate_local_center[0],
        candidate_local_center[1],
        cross_span_mm,
        along_span_mm,
        registration.tape_pitch_mm,
    );
    let reference_occupancy = pocket_occupancy_from_evidence(reference_confidence);
    let candidate_occupancy = pocket_occupancy_from_evidence(candidate_confidence);
    let to_machine = |nominal: [f64; 2], local: [f64; 2]| {
        add(
            nominal,
            add(
                scale(cross, local[0] - center_cross_mm),
                scale(along, local[1] - center_along_mm),
            ),
        )
    };
    let reference_machine =
        reference_center.map(|_| to_machine(reference_pocket_machine_mm, reference_local_center));
    let candidate_machine =
        candidate_center.map(|_| to_machine(candidate_pocket_machine_mm, candidate_local_center));
    let relative_center_shift_mm = reference_center.zip(candidate_center).map(|_| {
        (candidate_local_center[0] - reference_local_center[0])
            .hypot(candidate_local_center[1] - reference_local_center[1])
    });
    let appearance_match = reference_center.zip(candidate_center).and_then(|_| {
        let half_span_mm = pocket_return_patch_half_span_mm(registration, cross, along);
        let interior_half_span_mm = [
            half_span_mm[0] * POCKET_RETURN_INTERIOR_SCALE,
            half_span_mm[1] * POCKET_RETURN_INTERIOR_SCALE,
        ];
        centered_pocket_patch_similarity(
            &reference_crop,
            local_mm_to_pixel(
                &reference_crop,
                reference_local_center,
                cross_span_mm,
                along_span_mm,
            ),
            &candidate_crop,
            local_mm_to_pixel(
                &candidate_crop,
                candidate_local_center,
                cross_span_mm,
                along_span_mm,
            ),
            interior_half_span_mm,
        )
    });
    let appearance_similarity = appearance_match.map(|measurement| measurement.similarity);
    let maximum_relative_center_shift_mm = registration.tape_pitch_mm * 0.25;
    let reference_package_size_mm = reference_package.map(|package| package.size_mm);
    let candidate_package_size_mm = candidate_package.map(|package| package.size_mm);
    let component_outline = |nominal: [f64; 2], package: SourcePackageMeasurement| {
        let half_cross = 0.5 * package.size_mm[0];
        let half_along = 0.5 * package.size_mm[1];
        [
            to_machine(
                nominal,
                [
                    package.center_mm[0] - half_cross,
                    package.center_mm[1] - half_along,
                ],
            ),
            to_machine(
                nominal,
                [
                    package.center_mm[0] + half_cross,
                    package.center_mm[1] - half_along,
                ],
            ),
            to_machine(
                nominal,
                [
                    package.center_mm[0] + half_cross,
                    package.center_mm[1] + half_along,
                ],
            ),
            to_machine(
                nominal,
                [
                    package.center_mm[0] - half_cross,
                    package.center_mm[1] + half_along,
                ],
            ),
        ]
    };
    let reference_component_outline_machine_mm =
        reference_package.map(|package| component_outline(reference_pocket_machine_mm, package));
    let candidate_component_outline_machine_mm =
        candidate_package.map(|package| component_outline(candidate_pocket_machine_mm, package));
    let flat_geometry_passed = allow_face_texture_change
        && reference_package_size_mm
            .zip(expected_package_size_mm)
            .is_some_and(|(measured, expected)| source_package_size_matches(measured, expected))
        && candidate_package_size_mm
            .zip(expected_package_size_mm)
            .is_some_and(|(measured, expected)| source_package_size_matches(measured, expected));
    let appearance_passed = appearance_similarity
        .is_some_and(|similarity| similarity >= MINIMUM_SOURCE_POSE_APPEARANCE_SIMILARITY);
    let passed = reference_occupancy == StripPocketOccupancy::Occupied
        && candidate_occupancy == StripPocketOccupancy::Occupied
        && relative_center_shift_mm.is_some_and(|shift| shift <= maximum_relative_center_shift_mm)
        && (appearance_passed || flat_geometry_passed);
    let mut findings = Vec::new();
    if reference_occupancy != StripPocketOccupancy::Occupied {
        findings.push("the successful pickup reference is not visibly occupied".into());
    }
    if candidate_occupancy != StripPocketOccupancy::Occupied {
        findings.push("the requested source pocket is not visibly occupied".into());
    }
    if reference_center.is_none() || candidate_center.is_none() {
        findings.push("the package center was not measured in both source views".into());
    }
    if relative_center_shift_mm.is_some_and(|shift| shift > maximum_relative_center_shift_mm) {
        findings.push(format!(
            "the source package center differs from the reference by more than {maximum_relative_center_shift_mm:.3} mm"
        ));
    }
    if !appearance_passed && !flat_geometry_passed {
        findings.push(format!(
            "the source package does not match the successful pickup pose at \
             {MINIMUM_SOURCE_POSE_APPEARANCE_SIMILARITY:.2} or better",
        ));
    }
    if allow_face_texture_change && !flat_geometry_passed {
        findings.push("the source package does not have the expected flat geometry".into());
    }
    if reference_package.is_none() || candidate_package.is_none() {
        findings.push(
            "an explicit component outline was not found in both source views; the contrast centroid was used as a fallback"
                .into(),
        );
    } else {
        findings.push(
            "cyan marks the nominal pocket center, green marks the component outline, magenta marks the component center, and yellow marks the nozzle target"
                .into(),
        );
    }
    if let Some(measurement) = appearance_match {
        findings.push(format!(
            "source appearance aligned by cross {:+.3} mm, along {:+.3} mm, and {:+.1} degrees",
            measurement.shift_cross_mm, measurement.shift_along_mm, measurement.rotation_deg,
        ));
    }
    if passed {
        findings.push(if appearance_passed {
            "top vision matched a prior package pose that passed pickup".into()
        } else {
            "flat axis-symmetric package geometry passed despite changed face texture".into()
        });
    }
    let preview = render_source_pose_preview(
        &reference_crop,
        &candidate_crop,
        reference_local_center,
        candidate_local_center,
        reference_package,
        candidate_package,
        cross_span_mm,
        along_span_mm,
    );
    Ok(StripFeederSourcePoseArtifacts {
        report: StripFeederSourcePoseReport {
            algorithm_version: STRIP_FEEDER_SOURCE_POSE_ALGORITHM.into(),
            reference_qualification_id,
            reference_nominal_center_machine_mm: reference_pocket_machine_mm,
            reference_camera_pose,
            reference_occupancy,
            reference_confidence,
            reference_measured_center_machine_mm: reference_machine,
            candidate_nominal_center_machine_mm: candidate_pocket_machine_mm,
            candidate_camera_pose,
            candidate_occupancy,
            candidate_confidence,
            candidate_measured_center_machine_mm: candidate_machine,
            relative_center_shift_mm,
            appearance_similarity,
            minimum_appearance_similarity: MINIMUM_SOURCE_POSE_APPEARANCE_SIMILARITY,
            reference_package_size_mm,
            reference_component_outline_machine_mm,
            candidate_package_size_mm,
            candidate_component_outline_machine_mm,
            flat_geometry_passed,
            face_texture_change_allowed: allow_face_texture_change,
            passed,
            artifact_name: artifact_name.into(),
            findings,
        },
        preview,
    })
}

fn measure_source_package(
    crop: &GrayFrame,
    center: PocketImageCenter,
    cross_span_mm: f64,
    along_span_mm: f64,
    expected_size_mm: [f64; 2],
) -> Option<SourcePackageMeasurement> {
    let mut expected = expected_size_mm;
    expected.sort_by(f64::total_cmp);
    if expected[0] < 0.1 || !expected.iter().all(|value| value.is_finite()) {
        return None;
    }
    let mut candidates = detect_rectangular_features(crop, cross_span_mm, along_span_mm, true)
        .ok()?
        .into_iter()
        .filter_map(|feature| {
            let center_error = (feature.cross_mm - center.center_mm[0])
                .hypot(feature.along_mm - center.center_mm[1]);
            let mut size = [feature.cross_size_mm, feature.along_size_mm];
            size.sort_by(f64::total_cmp);
            (center_error <= 1.50).then_some((feature, size, center_error))
        })
        .collect::<Vec<_>>();
    candidates.sort_by(|first, second| {
        let score = |candidate: &(RectangularFeature, [f64; 2], f64)| {
            let (_, size, center_error) = candidate;
            center_error / 0.75
                + (size[0] / expected[0]).ln().abs()
                + (size[1] / expected[1]).ln().abs()
        };
        score(first).total_cmp(&score(second))
    });
    candidates
        .first()
        .map(|(feature, _, _)| SourcePackageMeasurement {
            center_mm: [feature.cross_mm, feature.along_mm],
            size_mm: [feature.cross_size_mm, feature.along_size_mm],
        })
        .or_else(|| {
            measure_dark_source_package(
                crop,
                center,
                cross_span_mm,
                along_span_mm,
                expected_size_mm,
            )
            .ok()
            .flatten()
        })
}

fn measure_dark_source_package(
    crop: &GrayFrame,
    center: PocketImageCenter,
    cross_span_mm: f64,
    along_span_mm: f64,
    expected_size_mm: [f64; 2],
) -> Result<Option<SourcePackageMeasurement>> {
    let x_px_per_mm = f64::from(crop.width) / cross_span_mm;
    let y_px_per_mm = f64::from(crop.height) / along_span_mm;
    let mut expected = expected_size_mm;
    expected.sort_by(f64::total_cmp);
    let half_extent_mm = (1.35 * expected[1]).clamp(1.25, 2.75);
    let center_px = local_mm_to_pixel(crop, center.center_mm, cross_span_mm, along_span_mm);
    let left = (center_px[0] - half_extent_mm * x_px_per_mm)
        .floor()
        .max(0.0) as i32;
    let right = (center_px[0] + half_extent_mm * x_px_per_mm)
        .ceil()
        .min(f64::from(crop.width)) as i32;
    let top = (center_px[1] - half_extent_mm * y_px_per_mm)
        .floor()
        .max(0.0) as i32;
    let bottom = (center_px[1] + half_extent_mm * y_px_per_mm)
        .ceil()
        .min(f64::from(crop.height)) as i32;
    if right - left < 12 || bottom - top < 12 {
        return Ok(None);
    }
    let source = gray_frame_to_mat(crop)?;
    let roi = Mat::roi(&source, Rect::new(left, top, right - left, bottom - top))?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &roi,
        &mut blurred,
        Size::new(3, 3),
        0.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut mask = Mat::default();
    let otsu_threshold = imgproc::threshold(
        &blurred,
        &mut mask,
        0.0,
        255.0,
        imgproc::THRESH_BINARY_INV | imgproc::THRESH_OTSU,
    )?;
    let core_threshold = (otsu_threshold - 40.0).clamp(32.0, 192.0);
    imgproc::threshold(
        &blurred,
        &mut mask,
        core_threshold,
        255.0,
        imgproc::THRESH_BINARY_INV,
    )?;
    let kernel =
        imgproc::get_structuring_element(imgproc::MORPH_RECT, Size::new(3, 3), Point::new(-1, -1))?;
    let mask_source = mask.try_clone()?;
    imgproc::morphology_ex(
        &mask_source,
        &mut mask,
        imgproc::MORPH_OPEN,
        &kernel,
        Point::new(-1, -1),
        1,
        core::BORDER_CONSTANT,
        imgproc::morphology_default_border_value()?,
    )?;
    let mut labels = Mat::default();
    let mut stats = Mat::default();
    let mut centroids = Mat::default();
    let count = imgproc::connected_components_with_stats(
        &mask,
        &mut labels,
        &mut stats,
        &mut centroids,
        8,
        CV_32S,
    )?;
    let expected_area_mm2 = expected[0] * expected[1];
    let mut candidates = Vec::new();
    for label in 1..count {
        let area_px = f64::from(*stats.at_2d::<i32>(label, imgproc::CC_STAT_AREA)?);
        let component_left = *stats.at_2d::<i32>(label, imgproc::CC_STAT_LEFT)?;
        let component_top = *stats.at_2d::<i32>(label, imgproc::CC_STAT_TOP)?;
        let width_px = *stats.at_2d::<i32>(label, imgproc::CC_STAT_WIDTH)?;
        let height_px = *stats.at_2d::<i32>(label, imgproc::CC_STAT_HEIGHT)?;
        let size_mm = [
            f64::from(width_px) / x_px_per_mm,
            f64::from(height_px) / y_px_per_mm,
        ];
        let area_mm2 = area_px / (x_px_per_mm * y_px_per_mm);
        if area_mm2 < 0.08 * expected_area_mm2
            || area_mm2 > 1.60 * expected_area_mm2
            || size_mm[0].max(size_mm[1]) > 1.60 * expected[1]
            || size_mm[0].min(size_mm[1]) < 0.12
        {
            continue;
        }
        let component_center_px = [
            f64::from(left + component_left) + 0.5 * f64::from(width_px),
            f64::from(top + component_top) + 0.5 * f64::from(height_px),
        ];
        let component_center_mm = [
            component_center_px[0] / x_px_per_mm,
            component_center_px[1] / y_px_per_mm,
        ];
        let center_error_mm = (component_center_mm[0] - center.center_mm[0])
            .hypot(component_center_mm[1] - center.center_mm[1]);
        if center_error_mm > 0.80 {
            continue;
        }
        let fill = area_px / f64::from((width_px * height_px).max(1));
        let aspect = size_mm[0].max(size_mm[1]) / size_mm[0].min(size_mm[1]);
        let expected_aspect = expected[1] / expected[0];
        let score = center_error_mm / 0.40
            + (aspect / expected_aspect).ln().abs()
            + 0.35 * (1.0 - fill.clamp(0.0, 1.0));
        candidates.push((
            score,
            SourcePackageMeasurement {
                center_mm: component_center_mm,
                size_mm,
            },
        ));
    }
    candidates.sort_by(|left, right| left.0.total_cmp(&right.0));
    Ok(candidates.first().map(|candidate| candidate.1))
}

fn source_package_size_matches(measured_size_mm: [f64; 2], expected_size_mm: [f64; 2]) -> bool {
    let mut measured = measured_size_mm;
    let mut expected = expected_size_mm;
    measured.sort_by(f64::total_cmp);
    expected.sort_by(f64::total_cmp);
    measured[0] >= 0.45 * expected[0]
        && measured[0] <= 1.60 * expected[0]
        && measured[1] >= 0.55 * expected[1]
        && measured[1] <= 1.60 * expected[1]
}

/// Checks the same pocket before pickup and after a return operation.
#[allow(clippy::too_many_arguments)]
pub fn inspect_strip_feeder_pocket_return(
    before: &GrayFrame,
    before_camera_pose: Pose,
    after: &GrayFrame,
    after_camera_pose: Pose,
    camera_model: &PlanarCameraModel,
    registration: &StripFeederRegistration,
    pocket_machine_mm: [f64; 2],
    artifact_name: &str,
) -> Result<StripFeederPocketReturnArtifacts> {
    ensure!(
        camera_model.role == velocitypnp_domain::CameraRole::TopCamera,
        "pocket return inspection requires a top-camera model"
    );
    ensure!(
        before.width == after.width && before.height == after.height,
        "pocket return frames changed dimensions"
    );
    let along = normalized(subtract(
        registration.pick_centerline_machine_mm[1],
        registration.pick_centerline_machine_mm[0],
    ))
    .ok_or_else(|| anyhow::anyhow!("pocket return centerline has no direction"))?;
    let cross = [-along[1], along[0]];
    let cross_span_mm = registration.tape_width_mm + 2.0 * LIVE_HOLDOUT_CROSS_MARGIN_MM;
    let along_span_mm = (registration.tape_pitch_mm * 3.0).max(12.0);
    let before_crop = rectify_live_tape_frames(
        std::slice::from_ref(before),
        &[before_camera_pose],
        camera_model,
        pocket_machine_mm,
        cross,
        along,
        cross_span_mm,
        along_span_mm,
    )?;
    let after_crop = rectify_live_tape_frames(
        std::slice::from_ref(after),
        &[after_camera_pose],
        camera_model,
        pocket_machine_mm,
        cross,
        along,
        cross_span_mm,
        along_span_mm,
    )?;
    let center_cross_mm = cross_span_mm * 0.5;
    let center_along_mm = along_span_mm * 0.5;
    let before_center = measure_pocket_image_center(
        &before_crop,
        center_cross_mm,
        center_along_mm,
        cross_span_mm,
        along_span_mm,
        registration.tape_pitch_mm,
        registration.tape_width_mm,
    );
    let after_center = measure_pocket_image_center(
        &after_crop,
        center_cross_mm,
        center_along_mm,
        cross_span_mm,
        along_span_mm,
        registration.tape_pitch_mm,
        registration.tape_width_mm,
    );
    let before_local_center = before_center
        .map(|measurement| measurement.center_mm)
        .unwrap_or([center_cross_mm, center_along_mm]);
    let after_local_center = after_center
        .map(|measurement| measurement.center_mm)
        .unwrap_or([center_cross_mm, center_along_mm]);
    let before_confidence = pocket_occupancy_evidence(
        &before_crop,
        before_local_center[0],
        before_local_center[1],
        cross_span_mm,
        along_span_mm,
        registration.tape_pitch_mm,
    );
    let after_confidence = pocket_occupancy_evidence(
        &after_crop,
        after_local_center[0],
        after_local_center[1],
        cross_span_mm,
        along_span_mm,
        registration.tape_pitch_mm,
    );
    let before_occupancy = pocket_occupancy_from_evidence(before_confidence);
    let after_occupancy = pocket_occupancy_from_evidence(after_confidence);
    let to_machine = |local: [f64; 2]| {
        add(
            pocket_machine_mm,
            add(
                scale(cross, local[0] - center_cross_mm),
                scale(along, local[1] - center_along_mm),
            ),
        )
    };
    let before_machine = before_center.map(|measurement| to_machine(measurement.center_mm));
    let after_machine = after_center.map(|measurement| to_machine(measurement.center_mm));
    let offset = |center: Option<[f64; 2]>| {
        center.map(|center| {
            (center[0] - pocket_machine_mm[0]).hypot(center[1] - pocket_machine_mm[1])
        })
    };
    let before_offset = offset(before_machine);
    let after_offset = offset(after_machine);
    let return_center_shift_mm = before_machine
        .zip(after_machine)
        .map(|(before, after)| (after[0] - before[0]).hypot(after[1] - before[1]));
    let appearance_match = before_center.zip(after_center).and_then(|(before, after)| {
        let patch_half_span_mm = pocket_return_patch_half_span_mm(registration, cross, along);
        let interior_half_span_mm = [
            patch_half_span_mm[0] * POCKET_RETURN_INTERIOR_SCALE,
            patch_half_span_mm[1] * POCKET_RETURN_INTERIOR_SCALE,
        ];
        centered_pocket_patch_similarity(
            &before_crop,
            before.center_px,
            &after_crop,
            after.center_px,
            interior_half_span_mm,
        )
    });
    let appearance_similarity = appearance_match.map(|measurement| measurement.similarity);
    let maximum_nominal_offset_mm = registration.tape_pitch_mm * 0.45;
    let maximum_return_shift_mm = registration.tape_pitch_mm * 0.25;
    let centers_passed = before_offset
        .zip(after_offset)
        .zip(return_center_shift_mm)
        .is_some_and(|((before, after), shift)| {
            before <= maximum_nominal_offset_mm
                && after <= maximum_nominal_offset_mm
                && shift <= maximum_return_shift_mm
        });
    let passed = before_occupancy == StripPocketOccupancy::Occupied
        && after_occupancy == StripPocketOccupancy::Occupied
        && centers_passed
        && appearance_similarity
            .is_some_and(|similarity| similarity >= MINIMUM_POCKET_RETURN_APPEARANCE_SIMILARITY);
    let mut findings = Vec::new();
    if before_occupancy != StripPocketOccupancy::Occupied {
        findings.push("the source pocket was not occupied before pickup".into());
    }
    if after_occupancy != StripPocketOccupancy::Occupied {
        findings.push("the source pocket was not occupied after release".into());
    }
    if before_center.is_none() || after_center.is_none() {
        findings.push("the source pocket center was not measured in both views".into());
    }
    if return_center_shift_mm.is_some_and(|shift| shift > maximum_return_shift_mm) {
        findings.push(format!(
            "the returned center shifted more than {maximum_return_shift_mm:.3} mm"
        ));
    }
    if appearance_similarity
        .is_none_or(|similarity| similarity < MINIMUM_POCKET_RETURN_APPEARANCE_SIMILARITY)
    {
        findings.push(format!(
            "the after-release pocket did not match the before-pickup package appearance at \
             {MINIMUM_POCKET_RETURN_APPEARANCE_SIMILARITY:.2} or better",
        ));
    }
    if let Some(measurement) = appearance_match.filter(|measurement| {
        measurement.shift_cross_mm.hypot(measurement.shift_along_mm) > 0.02
            || measurement.rotation_deg.abs() > 0.5
    }) {
        findings.push(format!(
            "package appearance aligned by cross {:+.3} mm, along {:+.3} mm, and {:+.1} degrees",
            measurement.shift_cross_mm, measurement.shift_along_mm, measurement.rotation_deg,
        ));
    }
    if passed {
        findings.push("top vision confirmed the returned part in its source pocket".into());
    }
    let preview = render_pocket_return_preview(
        &before_crop,
        &after_crop,
        before_occupancy,
        after_occupancy,
        appearance_similarity
            .is_some_and(|similarity| similarity >= MINIMUM_POCKET_RETURN_APPEARANCE_SIMILARITY),
        before_center.map(|measurement| measurement.center_px),
        after_center.map(|measurement| measurement.center_px),
    );
    Ok(StripFeederPocketReturnArtifacts {
        report: StripFeederPocketReturnReport {
            algorithm_version: STRIP_FEEDER_POCKET_RETURN_ALGORITHM.into(),
            nominal_center_machine_mm: Some(pocket_machine_mm),
            before_camera_pose: Some(before_camera_pose),
            before_occupancy,
            before_confidence,
            before_measured_center_machine_mm: before_machine,
            before_center_offset_mm: before_offset,
            after_camera_pose: Some(after_camera_pose),
            after_occupancy,
            after_confidence,
            after_measured_center_machine_mm: after_machine,
            after_center_offset_mm: after_offset,
            return_center_shift_mm,
            appearance_similarity,
            passed,
            artifact_name: artifact_name.into(),
            findings,
        },
        preview,
    })
}

#[derive(Debug, Clone, Copy)]
struct CircleFeature {
    center_px: [f64; 2],
    confidence: f64,
}

#[derive(Debug, Clone, Copy)]
struct PocketImageCenter {
    center_px: [f64; 2],
    center_mm: [f64; 2],
    confidence: f64,
}

#[derive(Debug, Clone, Copy)]
struct SourcePackageMeasurement {
    center_mm: [f64; 2],
    size_mm: [f64; 2],
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PocketPatchMatch {
    similarity: f64,
    shift_cross_mm: f64,
    shift_along_mm: f64,
    rotation_deg: f64,
}

#[derive(Debug, Clone)]
struct SprocketTrack {
    cross_mm: f64,
    circles: Vec<CircleFeature>,
    residual_p95_mm: f64,
    confidence: f64,
    paired_pocket_centerline_mm: Option<f64>,
}

#[derive(Debug, Clone)]
struct SuppressedPocketTrack {
    track: SprocketTrack,
    paired_sprocket_cross_mm: f64,
}

#[derive(Debug, Clone, Copy)]
struct RectangularFeature {
    cross_mm: f64,
    along_mm: f64,
    cross_size_mm: f64,
    along_size_mm: f64,
    rectangularity: f64,
    circularity: f64,
    confidence: f64,
}

#[derive(Debug, Clone)]
struct ComponentFeatureCluster {
    along_mm: f64,
    features: Vec<RectangularFeature>,
    confidence: f64,
}

#[derive(Debug, Clone, Copy)]
struct PocketAppearance {
    mean_luma: f64,
    edge_energy: f64,
}

/// Finds repeated carrier-tape sprocket holes in one normalized feeder crop.
/// Candidate edges define the calibrated plate basis, so the result stays
/// parallel or perpendicular to that basis.
pub fn analyze_strip_feeder_candidate(
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
) -> Result<StripFeederVisionArtifacts> {
    ensure!(
        crop.width >= 24
            && crop.height >= 24
            && crop.pixels.len() == crop.width as usize * crop.height as usize,
        "strip feeder evidence crop is invalid"
    );
    ensure!(
        candidate.outline_machine_mm.len() == 4
            && candidate.width_mm.is_finite()
            && candidate.height_mm.is_finite()
            && candidate.width_mm > 0.0
            && candidate.height_mm > 0.0,
        "strip feeder candidate geometry is invalid"
    );

    let circles = detect_sprocket_circles(crop, candidate.width_mm, candidate.height_mm)?;
    let long_is_y = candidate.height_mm >= candidate.width_mm;
    let (cross_length_mm, along_length_mm) = if long_is_y {
        (candidate.width_mm, candidate.height_mm)
    } else {
        (candidate.height_mm, candidate.width_mm)
    };
    let detected_tracks = build_sprocket_tracks(
        &circles,
        crop,
        candidate.width_mm,
        candidate.height_mm,
        long_is_y,
        MINIMUM_SPROCKETS_PER_TRACK,
    );
    let (tracks, suppressed_tracks) = resolve_pocket_row_competitors(detected_tracks);
    let rectangular_features =
        detect_rectangular_features(crop, candidate.width_mm, candidate.height_mm, long_is_y)?;
    let preferred_pocket_side = infer_bank_pocket_side(
        &suppressed_tracks,
        &tracks,
        &rectangular_features,
        crop,
        candidate.width_mm,
        candidate.height_mm,
        long_is_y,
        cross_length_mm,
        along_length_mm,
    );
    let mut proposals = tracks
        .iter()
        .enumerate()
        .map(|(index, track)| {
            lane_proposal(
                index as u32,
                track,
                crop,
                candidate,
                long_is_y,
                cross_length_mm,
                along_length_mm,
                &rectangular_features,
                preferred_pocket_side,
            )
        })
        .collect::<Vec<_>>();
    proposals.sort_by(|first, second| {
        let first_cross = proposal_cross_position(first, crop, candidate, long_is_y);
        let second_cross = proposal_cross_position(second, crop, candidate, long_is_y);
        first_cross.total_cmp(&second_cross)
    });
    for (index, proposal) in proposals.iter_mut().enumerate() {
        proposal.proposal_index = index as u32;
    }

    let mut warnings = Vec::new();
    if circles.is_empty() {
        warnings.push("no sprocket-hole-sized circular features were found".into());
    } else if tracks.is_empty() {
        warnings.push(format!(
            "{} circular features were found, but none formed a 4 mm tape lattice",
            circles.len()
        ));
    }
    if !suppressed_tracks.is_empty() {
        warnings.push(format!(
            "{} 4 mm lattices were rejected as 8 mm tape pocket-center rows",
            suppressed_tracks.len(),
        ));
    }
    let review_count = proposals
        .iter()
        .filter(|proposal| proposal.review_required)
        .count();
    if review_count > 0 {
        warnings.push(format!(
            "{review_count} lane proposals need live camera review before feeder activation"
        ));
    }
    if !proposals.is_empty() {
        warnings.push(
            "pickup Z, part identity, pocket occupancy, and a live holdout are not measured".into(),
        );
    }
    let rejected_lattices = suppressed_tracks
        .iter()
        .map(|suppressed| rejected_lattice(suppressed, candidate, long_is_y, along_length_mm))
        .collect::<Vec<_>>();
    let preview = render_preview(
        crop,
        candidate,
        long_is_y,
        &circles,
        &proposals,
        &suppressed_tracks,
    );
    Ok(StripFeederVisionArtifacts {
        proposals,
        rejected_lattices,
        preview,
        warnings,
    })
}

/// Checks one retained feeder lane in a fresh top-camera observation.
///
/// The image is first rectified into the draft tape basis. The check then
/// measures the sprocket line, pocket phase, and central pocket image support.
pub fn analyze_live_strip_feeder_holdout(
    frames: &[GrayFrame],
    camera_poses: &[Pose],
    camera_model: &PlanarCameraModel,
    registration: &StripFeederRegistration,
    predicted_pocket_machine_mm: [f64; 2],
    artifact_name: &str,
) -> Result<StripFeederLiveHoldoutArtifacts> {
    ensure!(
        !frames.is_empty(),
        "live feeder holdout has no camera frames"
    );
    ensure!(
        frames.len() == camera_poses.len(),
        "live feeder holdout frame poses do not match frames"
    );
    let first = &frames[0];
    ensure!(
        first.width >= 64
            && first.height >= 64
            && first.pixels.len() == first.width as usize * first.height as usize,
        "live feeder holdout camera frame is invalid"
    );
    ensure!(
        frames.iter().all(|frame| {
            frame.width == first.width
                && frame.height == first.height
                && frame.pixels.len() == first.pixels.len()
        }),
        "live feeder holdout camera frames changed dimensions"
    );
    ensure!(
        camera_model.role == velocitypnp_domain::CameraRole::TopCamera,
        "live feeder holdout requires a top-camera planar model"
    );
    ensure!(
        camera_poses.iter().all(|pose| pose.finite()),
        "live feeder holdout camera pose is invalid"
    );
    ensure!(
        registration.tape_width_mm.is_finite()
            && registration.tape_width_mm >= 8.0
            && registration.tape_pitch_mm.is_finite()
            && registration.tape_pitch_mm >= 2.0,
        "live feeder holdout registration geometry is invalid"
    );
    ensure!(
        registration
            .pick_centerline_machine_mm
            .iter()
            .flatten()
            .all(|value| value.is_finite())
            && predicted_pocket_machine_mm
                .iter()
                .all(|value| value.is_finite()),
        "live feeder holdout target is invalid"
    );

    let along = normalized(subtract(
        registration.pick_centerline_machine_mm[1],
        registration.pick_centerline_machine_mm[0],
    ))
    .ok_or_else(|| anyhow::anyhow!("live feeder holdout centerline has no direction"))?;
    let cross = [-along[1], along[0]];
    let cross_span_mm = registration.tape_width_mm + 2.0 * LIVE_HOLDOUT_CROSS_MARGIN_MM;
    let crop = rectify_live_tape_frames(
        frames,
        camera_poses,
        camera_model,
        predicted_pocket_machine_mm,
        cross,
        along,
        cross_span_mm,
        LIVE_HOLDOUT_ALONG_SPAN_MM,
    )?;
    let circles = detect_live_sprocket_circles(&crop, cross_span_mm, LIVE_HOLDOUT_ALONG_SPAN_MM)?;
    let tracks = build_sprocket_tracks(
        &circles,
        &crop,
        cross_span_mm,
        LIVE_HOLDOUT_ALONG_SPAN_MM,
        true,
        MINIMUM_LIVE_HOLDOUT_SPROCKETS,
    );
    let center_cross_mm = cross_span_mm * 0.5;
    let center_along_mm = LIVE_HOLDOUT_ALONG_SPAN_MM * 0.5;
    let sprocket_offset_mm = registration.tape_width_mm * 0.5 - POCKET_CENTERLINE_EDGE_REMAINDER_MM;
    let expected_tracks = [
        center_cross_mm - sprocket_offset_mm,
        center_cross_mm + sprocket_offset_mm,
    ];
    let matched = tracks
        .iter()
        .flat_map(|track| {
            expected_tracks
                .iter()
                .map(move |expected| (track, *expected, (track.cross_mm - expected).abs()))
        })
        .filter(|(_, _, distance)| *distance <= LIVE_HOLDOUT_TRACK_TOLERANCE_MM)
        .min_by(|first, second| {
            first
                .2
                .total_cmp(&second.2)
                .then_with(|| second.0.confidence.total_cmp(&first.0.confidence))
        });

    let mut findings = Vec::new();
    let Some((track, expected_track_cross_mm, _)) = matched else {
        if tracks.is_empty() {
            findings.push(format!(
                "{} sprocket-sized live features did not form the required three-hole 4 mm lattice",
                circles.len(),
            ));
        } else {
            findings.push(format!(
                "no live sprocket line was found within {LIVE_HOLDOUT_TRACK_TOLERANCE_MM:.2} mm of the draft",
            ));
        }
        let preview = render_live_holdout_preview(
            &crop,
            &circles,
            None,
            expected_tracks,
            center_cross_mm,
            &[],
            None,
        );
        return Ok(StripFeederLiveHoldoutArtifacts {
            report: failed_live_holdout_report(
                predicted_pocket_machine_mm,
                artifact_name,
                findings,
            ),
            preview,
        });
    };

    let local_centers = track
        .circles
        .iter()
        .map(|circle| {
            [
                (circle.center_px[0] + 0.5) / f64::from(crop.width) * cross_span_mm,
                (circle.center_px[1] + 0.5) / f64::from(crop.height) * LIVE_HOLDOUT_ALONG_SPAN_MM,
            ]
        })
        .collect::<Vec<_>>();
    let (actual_track_cross_mm, track_slope) =
        fit_cross_over_along(&local_centers).unwrap_or((track.cross_mm, 0.0));
    let rotation_correction_deg = track_slope.atan().to_degrees();
    let cross_delta_mm = actual_track_cross_mm - expected_track_cross_mm;
    let live_centerline_cross_mm = center_cross_mm + cross_delta_mm;
    let (phase_mm, periodicity_confidence) = estimate_known_component_phase(
        &crop,
        live_centerline_cross_mm,
        cross_span_mm,
        LIVE_HOLDOUT_ALONG_SPAN_MM,
        registration.tape_pitch_mm,
    )
    .unwrap_or((center_along_mm, 0.0));
    let along_delta_mm =
        nearest_periodic_delta(phase_mm, registration.tape_pitch_mm, center_along_mm);
    let mut visible_pockets = Vec::new();
    let first_index =
        ((1.0 - center_along_mm - along_delta_mm) / registration.tape_pitch_mm).ceil() as i32;
    let last_index = ((LIVE_HOLDOUT_ALONG_SPAN_MM - 1.0 - center_along_mm - along_delta_mm)
        / registration.tape_pitch_mm)
        .floor() as i32;
    for index in first_index..=last_index {
        let tentative_along_mm =
            center_along_mm + along_delta_mm + f64::from(index) * registration.tape_pitch_mm;
        let geometry_center_mm = [live_centerline_cross_mm, tentative_along_mm];
        let measured_center = measure_pocket_image_center(
            &crop,
            live_centerline_cross_mm,
            tentative_along_mm,
            cross_span_mm,
            LIVE_HOLDOUT_ALONG_SPAN_MM,
            registration.tape_pitch_mm,
            registration.tape_width_mm,
        );
        let measured_center_mm = measured_center
            .map(|measurement| measurement.center_mm)
            .unwrap_or([live_centerline_cross_mm, tentative_along_mm]);
        let evidence = pocket_occupancy_evidence(
            &crop,
            measured_center_mm[0],
            measured_center_mm[1],
            cross_span_mm,
            LIVE_HOLDOUT_ALONG_SPAN_MM,
            registration.tape_pitch_mm,
        );
        visible_pockets.push((
            index,
            geometry_center_mm,
            evidence,
            measured_center.map_or(0.0, |measurement| measurement.confidence),
        ));
    }
    let requested_occupied = visible_pockets.iter().find(|item| {
        item.0 == 0 && pocket_occupancy_from_evidence(item.2) == StripPocketOccupancy::Occupied
    });
    let selected = requested_occupied.or_else(|| {
        visible_pockets.iter().max_by(|first, second| {
            first
                .2
                .total_cmp(&second.2)
                .then_with(|| first.3.total_cmp(&second.3))
        })
    });
    let occupancy_confidence = selected.map_or(0.0, |item| item.2);
    let occupancy = pocket_occupancy_from_evidence(occupancy_confidence);
    let selected_index = selected.map_or(0, |item| item.0);
    let selected_local_center = selected.map_or(
        [live_centerline_cross_mm, center_along_mm + along_delta_mm],
        |item| item.1,
    );
    let selected_predicted = add(
        predicted_pocket_machine_mm,
        scale(
            along,
            f64::from(selected_index) * registration.tape_pitch_mm,
        ),
    );
    let selected_machine = add(
        predicted_pocket_machine_mm,
        add(
            scale(cross, selected_local_center[0] - center_cross_mm),
            scale(along, selected_local_center[1] - center_along_mm),
        ),
    );
    let correction = subtract(selected_machine, selected_predicted);
    let correction_magnitude_mm = correction[0].hypot(correction[1]);

    let geometry_passed = track.circles.len() >= MINIMUM_LIVE_HOLDOUT_SPROCKETS
        && track.residual_p95_mm <= LIVE_HOLDOUT_MAX_PITCH_RESIDUAL_MM
        && correction_magnitude_mm <= LIVE_HOLDOUT_MAX_CORRECTION_MM
        && rotation_correction_deg.abs() <= LIVE_HOLDOUT_MAX_ROTATION_DEG
        && periodicity_confidence >= 0.15;
    if correction_magnitude_mm > LIVE_HOLDOUT_MAX_CORRECTION_MM {
        findings.push(format!(
            "local correction {correction_magnitude_mm:.3} mm exceeds {LIVE_HOLDOUT_MAX_CORRECTION_MM:.3} mm",
        ));
    }
    if rotation_correction_deg.abs() > LIVE_HOLDOUT_MAX_ROTATION_DEG {
        findings.push(format!(
            "local rotation correction {rotation_correction_deg:.3} degrees exceeds {LIVE_HOLDOUT_MAX_ROTATION_DEG:.3} degrees",
        ));
    }
    if track.residual_p95_mm > LIVE_HOLDOUT_MAX_PITCH_RESIDUAL_MM {
        findings.push(format!(
            "sprocket pitch residual {:.3} mm exceeds {:.3} mm",
            track.residual_p95_mm, LIVE_HOLDOUT_MAX_PITCH_RESIDUAL_MM,
        ));
    }
    if periodicity_confidence < 0.15 {
        findings.push("the live pocket phase has weak periodic image support".into());
    }
    if occupancy != StripPocketOccupancy::Occupied {
        findings.push("no pocket has strong occupied-component image support".into());
    }
    let passed = geometry_passed && occupancy == StripPocketOccupancy::Occupied;
    if passed {
        findings
            .push("fresh sprocket geometry and one occupied pocket passed the live holdout".into());
    }
    let correction_score =
        (1.0 - correction_magnitude_mm / LIVE_HOLDOUT_MAX_CORRECTION_MM).clamp(0.0, 1.0);
    let residual_score =
        (1.0 - track.residual_p95_mm / LIVE_HOLDOUT_MAX_PITCH_RESIDUAL_MM).clamp(0.0, 1.0);
    let confidence = (0.30 * track.confidence
        + 0.20 * residual_score
        + 0.20 * periodicity_confidence.clamp(0.0, 1.0)
        + 0.20 * occupancy_confidence
        + 0.10 * correction_score)
        .clamp(0.0, 1.0);
    let preview_pockets = visible_pockets
        .iter()
        .map(|item| item.1)
        .collect::<Vec<_>>();
    let preview = render_live_holdout_preview(
        &crop,
        &circles,
        Some((actual_track_cross_mm, track_slope)),
        expected_tracks,
        center_cross_mm,
        &preview_pockets,
        Some(selected_local_center),
    );
    Ok(StripFeederLiveHoldoutArtifacts {
        report: StripFeederLiveHoldoutReport {
            algorithm_version: STRIP_FEEDER_LIVE_HOLDOUT_ALGORITHM.into(),
            predicted_pocket_machine_mm: selected_predicted,
            measured_pocket_machine_mm: Some(selected_machine),
            correction_x_mm: correction[0],
            correction_y_mm: correction[1],
            correction_magnitude_mm,
            rotation_correction_deg,
            sprocket_center_count: track.circles.len() as u32,
            sprocket_pitch_residual_p95_mm: track.residual_p95_mm,
            occupancy,
            occupancy_confidence,
            confidence,
            passed,
            artifact_name: artifact_name.into(),
            source_frame_artifact_names: Vec::new(),
            source_frame_camera_poses: Vec::new(),
            findings,
        },
        preview,
    })
}

fn failed_live_holdout_report(
    predicted_pocket_machine_mm: [f64; 2],
    artifact_name: &str,
    findings: Vec<String>,
) -> StripFeederLiveHoldoutReport {
    StripFeederLiveHoldoutReport {
        algorithm_version: STRIP_FEEDER_LIVE_HOLDOUT_ALGORITHM.into(),
        predicted_pocket_machine_mm,
        measured_pocket_machine_mm: None,
        correction_x_mm: 0.0,
        correction_y_mm: 0.0,
        correction_magnitude_mm: 0.0,
        rotation_correction_deg: 0.0,
        sprocket_center_count: 0,
        sprocket_pitch_residual_p95_mm: 0.0,
        occupancy: StripPocketOccupancy::Uncertain,
        occupancy_confidence: 0.0,
        confidence: 0.0,
        passed: false,
        artifact_name: artifact_name.into(),
        source_frame_artifact_names: Vec::new(),
        source_frame_camera_poses: Vec::new(),
        findings,
    }
}

#[allow(clippy::too_many_arguments)]
fn rectify_live_tape_frames(
    frames: &[GrayFrame],
    camera_poses: &[Pose],
    model: &PlanarCameraModel,
    center_machine_mm: [f64; 2],
    cross_axis: [f64; 2],
    along_axis: [f64; 2],
    cross_span_mm: f64,
    along_span_mm: f64,
) -> Result<GrayFrame> {
    let width = (cross_span_mm / LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL).round() as u32;
    let height = (along_span_mm / LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL).round() as u32;
    ensure!(
        width >= 64 && height >= 64,
        "live feeder rectified crop is too small"
    );
    let mut pixels = vec![0_u8; (width * height) as usize];
    let mut samples = Vec::with_capacity(frames.len());
    for y in 0..height {
        let along_mm =
            (f64::from(y) + 0.5) / f64::from(height) * along_span_mm - along_span_mm * 0.5;
        for x in 0..width {
            let cross_mm =
                (f64::from(x) + 0.5) / f64::from(width) * cross_span_mm - cross_span_mm * 0.5;
            let machine = add(
                center_machine_mm,
                add(scale(cross_axis, cross_mm), scale(along_axis, along_mm)),
            );
            samples.clear();
            let mut best_center_distance = f64::INFINITY;
            for (frame, camera_pose) in frames.iter().zip(camera_poses) {
                let desired = [machine[0] - camera_pose.x_mm, machine[1] - camera_pose.y_mm];
                let Some(pixel) = invert_camera_correction(model, desired) else {
                    continue;
                };
                if pixel[0] < model.supported_min_x_px
                    || pixel[0] > model.supported_max_x_px
                    || pixel[1] < model.supported_min_y_px
                    || pixel[1] > model.supported_max_y_px
                    || pixel[0] >= f64::from(frame.width.saturating_sub(1))
                    || pixel[1] >= f64::from(frame.height.saturating_sub(1))
                {
                    continue;
                }
                let center_distance =
                    (pixel[0] - model.center_x_px).hypot(pixel[1] - model.center_y_px);
                if center_distance < best_center_distance - 0.5 {
                    best_center_distance = center_distance;
                    samples.clear();
                    samples.push(bilinear_luma(frame, pixel[0], pixel[1]));
                } else if (center_distance - best_center_distance).abs() <= 0.5 {
                    samples.push(bilinear_luma(frame, pixel[0], pixel[1]));
                }
            }
            if !samples.is_empty() {
                samples.sort_unstable();
                pixels[(y * width + x) as usize] = samples[samples.len() / 2];
            }
        }
    }
    Ok(GrayFrame {
        width,
        height,
        pixels,
    })
}

fn invert_camera_correction(model: &PlanarCameraModel, desired: [f64; 2]) -> Option<[f64; 2]> {
    let mut pixel = [model.center_x_px, model.center_y_px];
    for _ in 0..6 {
        let current = model.correction_at(pixel[0], pixel[1])?;
        let error = [desired[0] - current[0], desired[1] - current[1]];
        if error[0].hypot(error[1]) < 0.002 {
            break;
        }
        let x = model.correction_at(pixel[0] + 1.0, pixel[1])?;
        let y = model.correction_at(pixel[0], pixel[1] + 1.0)?;
        let jacobian = [
            [x[0] - current[0], y[0] - current[0]],
            [x[1] - current[1], y[1] - current[1]],
        ];
        let determinant = jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0];
        if determinant.abs() <= 1e-9 {
            return None;
        }
        pixel[0] += (error[0] * jacobian[1][1] - error[1] * jacobian[0][1]) / determinant;
        pixel[1] += (jacobian[0][0] * error[1] - jacobian[1][0] * error[0]) / determinant;
    }
    (pixel[0] >= 0.0 && pixel[1] >= 0.0).then_some(pixel)
}

fn bilinear_luma(frame: &GrayFrame, x: f64, y: f64) -> u8 {
    if x < 0.0
        || y < 0.0
        || x >= f64::from(frame.width.saturating_sub(1))
        || y >= f64::from(frame.height.saturating_sub(1))
    {
        return 0;
    }
    let x0 = x.floor() as u32;
    let y0 = y.floor() as u32;
    let fx = x - f64::from(x0);
    let fy = y - f64::from(y0);
    let sample = |sample_x: u32, sample_y: u32| {
        f64::from(frame.pixels[(sample_y * frame.width + sample_x) as usize])
    };
    let top = sample(x0, y0) * (1.0 - fx) + sample(x0 + 1, y0) * fx;
    let bottom = sample(x0, y0 + 1) * (1.0 - fx) + sample(x0 + 1, y0 + 1) * fx;
    (top * (1.0 - fy) + bottom * fy).round().clamp(0.0, 255.0) as u8
}

fn fit_cross_over_along(points: &[[f64; 2]]) -> Option<(f64, f64)> {
    if points.len() < 2 {
        return None;
    }
    let mean_cross = points.iter().map(|point| point[0]).sum::<f64>() / points.len() as f64;
    let mean_along = points.iter().map(|point| point[1]).sum::<f64>() / points.len() as f64;
    let numerator = points
        .iter()
        .map(|point| (point[1] - mean_along) * (point[0] - mean_cross))
        .sum::<f64>();
    let denominator = points
        .iter()
        .map(|point| (point[1] - mean_along).powi(2))
        .sum::<f64>();
    let slope = (denominator > 1e-9).then_some(numerator / denominator)?;
    let center_cross = mean_cross + slope * (LIVE_HOLDOUT_ALONG_SPAN_MM * 0.5 - mean_along);
    Some((center_cross, slope))
}

fn estimate_known_component_phase(
    crop: &GrayFrame,
    cross_mm: f64,
    cross_span_mm: f64,
    along_span_mm: f64,
    pitch_mm: f64,
) -> Option<(f64, f64)> {
    let center = (cross_mm / cross_span_mm * f64::from(crop.width)).round() as i32;
    let half_band = ((0.9 / cross_span_mm * f64::from(crop.width)).ceil() as i32).max(1);
    let mut profile = Vec::with_capacity(crop.height as usize);
    for y in 0..crop.height as i32 {
        let mut values = Vec::new();
        for x in center - half_band..=center + half_band {
            if x >= 0 && x < crop.width as i32 {
                values.push(f64::from(
                    crop.pixels[(y as u32 * crop.width + x as u32) as usize],
                ));
            }
        }
        let mean = values.iter().sum::<f64>() / values.len().max(1) as f64;
        let variance = values
            .iter()
            .map(|value| (value - mean).powi(2))
            .sum::<f64>()
            / values.len().max(1) as f64;
        profile.push(variance.sqrt());
    }
    let mm_per_pixel = along_span_mm / f64::from(crop.height);
    let lag = (pitch_mm / mm_per_pixel).round() as usize;
    let confidence = periodicity_score(&profile, lag)?.clamp(0.0, 1.0);
    let phase_px = strongest_profile_phase(&profile, lag);
    Some((phase_px as f64 * mm_per_pixel, confidence))
}

fn nearest_periodic_delta(phase_mm: f64, pitch_mm: f64, expected_mm: f64) -> f64 {
    let nearest_index = ((expected_mm - phase_mm) / pitch_mm).round();
    phase_mm + nearest_index * pitch_mm - expected_mm
}

fn pocket_occupancy_evidence(
    crop: &GrayFrame,
    cross_mm: f64,
    along_mm: f64,
    cross_span_mm: f64,
    along_span_mm: f64,
    pitch_mm: f64,
) -> f64 {
    let center_x = cross_mm / cross_span_mm * f64::from(crop.width);
    let center_y = along_mm / along_span_mm * f64::from(crop.height);
    let mm_per_pixel =
        0.5 * (cross_span_mm / f64::from(crop.width) + along_span_mm / f64::from(crop.height));
    let radius_px = (pitch_mm.min(3.0) * 0.38 / mm_per_pixel).max(4.0);
    let ring_radius_px = radius_px * 1.7;
    let mut core = Vec::new();
    let mut ring = Vec::new();
    let mut edge_sum = 0.0;
    let mut edge_count = 0_u32;
    let x_min = (center_x - ring_radius_px).floor().max(1.0) as i32;
    let x_max = (center_x + ring_radius_px)
        .ceil()
        .min(f64::from(crop.width - 2)) as i32;
    let y_min = (center_y - ring_radius_px).floor().max(1.0) as i32;
    let y_max = (center_y + ring_radius_px)
        .ceil()
        .min(f64::from(crop.height - 2)) as i32;
    for y in y_min..=y_max {
        for x in x_min..=x_max {
            let distance = (f64::from(x) - center_x).hypot(f64::from(y) - center_y);
            let value = f64::from(crop.pixels[(y as u32 * crop.width + x as u32) as usize]);
            if distance <= radius_px {
                core.push(value);
                let left = crop.pixels[(y as u32 * crop.width + x as u32 - 1) as usize];
                let right = crop.pixels[(y as u32 * crop.width + x as u32 + 1) as usize];
                let top = crop.pixels[((y as u32 - 1) * crop.width + x as u32) as usize];
                let bottom = crop.pixels[((y as u32 + 1) * crop.width + x as u32) as usize];
                edge_sum +=
                    0.5 * (f64::from(left.abs_diff(right)) + f64::from(top.abs_diff(bottom)));
                edge_count += 1;
            } else if distance <= ring_radius_px {
                ring.push(value);
            }
        }
    }
    if core.is_empty() || ring.is_empty() {
        return 0.0;
    }
    let statistics = |values: &[f64]| {
        let mean = values.iter().sum::<f64>() / values.len() as f64;
        let variance = values
            .iter()
            .map(|value| (value - mean).powi(2))
            .sum::<f64>()
            / values.len() as f64;
        (mean, variance.sqrt())
    };
    let (core_mean, core_deviation) = statistics(&core);
    let (ring_mean, _) = statistics(&ring);
    let contrast = (core_mean - ring_mean).abs();
    let edge = edge_sum / f64::from(edge_count.max(1));
    (0.40 * (core_deviation / 35.0).clamp(0.0, 1.0)
        + 0.35 * (contrast / 30.0).clamp(0.0, 1.0)
        + 0.25 * (edge / 24.0).clamp(0.0, 1.0))
    .clamp(0.0, 1.0)
}

#[allow(clippy::too_many_arguments)]
fn measure_pocket_image_center(
    crop: &GrayFrame,
    nominal_cross_mm: f64,
    nominal_along_mm: f64,
    cross_span_mm: f64,
    along_span_mm: f64,
    pitch_mm: f64,
    tape_width_mm: f64,
) -> Option<PocketImageCenter> {
    let nominal_x = nominal_cross_mm / cross_span_mm * f64::from(crop.width);
    let nominal_y = nominal_along_mm / along_span_mm * f64::from(crop.height);
    let half_cross_mm = (tape_width_mm * 0.30).clamp(1.25, 2.5);
    let half_along_mm = (pitch_mm * 0.45).clamp(0.9, 2.0);
    let half_x = half_cross_mm / cross_span_mm * f64::from(crop.width);
    let half_y = half_along_mm / along_span_mm * f64::from(crop.height);
    let x_min = (nominal_x - half_x).floor().max(0.0) as u32;
    let x_max = (nominal_x + half_x)
        .ceil()
        .min(f64::from(crop.width.saturating_sub(1))) as u32;
    let y_min = (nominal_y - half_y).floor().max(0.0) as u32;
    let y_max = (nominal_y + half_y)
        .ceil()
        .min(f64::from(crop.height.saturating_sub(1))) as u32;
    if x_max <= x_min + 4 || y_max <= y_min + 4 {
        return None;
    }

    let margin_x = ((x_max - x_min) / 6).max(1);
    let margin_y = ((y_max - y_min) / 6).max(1);
    let mut perimeter = Vec::new();
    for y in y_min..=y_max {
        for x in x_min..=x_max {
            if x <= x_min + margin_x
                || x + margin_x >= x_max
                || y <= y_min + margin_y
                || y + margin_y >= y_max
            {
                perimeter.push(f64::from(crop.pixels[(y * crop.width + x) as usize]));
            }
        }
    }
    if perimeter.is_empty() {
        return None;
    }
    let background = median(&perimeter);
    let noise = median(
        &perimeter
            .iter()
            .map(|value| (value - background).abs())
            .collect::<Vec<_>>(),
    );
    let threshold = (noise * 3.0 + 6.0).clamp(10.0, 45.0);
    let mut weight_sum = 0.0;
    let mut weighted_x = 0.0;
    let mut weighted_y = 0.0;
    let mut support = 0_u32;
    for y in y_min..=y_max {
        for x in x_min..=x_max {
            let value = f64::from(crop.pixels[(y * crop.width + x) as usize]);
            let weight = ((value - background).abs() - threshold).max(0.0);
            if weight > 0.0 {
                let center_x = f64::from(x) + 0.5;
                let center_y = f64::from(y) + 0.5;
                weight_sum += weight;
                weighted_x += center_x * weight;
                weighted_y += center_y * weight;
                support += 1;
            }
        }
    }
    let area = f64::from((x_max - x_min + 1) * (y_max - y_min + 1));
    if weight_sum <= 1e-9 || support < 12 {
        return None;
    }
    let center_px = [weighted_x / weight_sum, weighted_y / weight_sum];
    let center_mm = [
        center_px[0] / f64::from(crop.width) * cross_span_mm,
        center_px[1] / f64::from(crop.height) * along_span_mm,
    ];
    let support_fraction = f64::from(support) / area;
    let mean_weight = weight_sum / f64::from(support);
    let confidence =
        ((support_fraction / 0.08).clamp(0.0, 1.0) * (mean_weight / 35.0).clamp(0.0, 1.0)).sqrt();
    Some(PocketImageCenter {
        center_px,
        center_mm,
        confidence,
    })
}

fn centered_pocket_patch_similarity(
    before: &GrayFrame,
    before_center_px: [f64; 2],
    after: &GrayFrame,
    after_center_px: [f64; 2],
    half_span_mm: [f64; 2],
) -> Option<PocketPatchMatch> {
    if before.width != after.width || before.height != after.height {
        return None;
    }
    let half_cross_px = (half_span_mm[0] / LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL).round() as i32;
    let half_along_px = (half_span_mm[1] / LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL).round() as i32;
    if half_cross_px < 5 || half_along_px < 5 {
        return None;
    }
    let patch = |frame: &GrayFrame, center: [f64; 2], shift: [f64; 2], rotation_deg: f64| {
        let angle = rotation_deg.to_radians();
        let (sin, cos) = angle.sin_cos();
        let mut values =
            Vec::with_capacity(((2 * half_cross_px + 1) * (2 * half_along_px + 1)) as usize);
        for y in -half_along_px..=half_along_px {
            for x in -half_cross_px..=half_cross_px {
                let x = f64::from(x);
                let y = f64::from(y);
                let sample_x = center[0] + shift[0] + cos * x - sin * y;
                let sample_y = center[1] + shift[1] + sin * x + cos * y;
                if sample_x < 1.0
                    || sample_y < 1.0
                    || sample_x >= f64::from(frame.width - 1)
                    || sample_y >= f64::from(frame.height - 1)
                {
                    return None;
                }
                values.push(f64::from(bilinear_luma(frame, sample_x, sample_y)));
            }
        }
        Some(values)
    };
    let before = patch(before, before_center_px, [0.0, 0.0], 0.0)?;
    let maximum_shift_px = (MAXIMUM_POCKET_RETURN_APPEARANCE_SHIFT_MM
        / LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL)
        .round() as i32;
    let mut best: Option<PocketPatchMatch> = None;
    for shift_along_px in -maximum_shift_px..=maximum_shift_px {
        for shift_cross_px in -maximum_shift_px..=maximum_shift_px {
            for rotation_deg in -MAXIMUM_POCKET_RETURN_APPEARANCE_ROTATION_DEG
                ..=MAXIMUM_POCKET_RETURN_APPEARANCE_ROTATION_DEG
            {
                let after = patch(
                    after,
                    after_center_px,
                    [f64::from(shift_cross_px), f64::from(shift_along_px)],
                    f64::from(rotation_deg),
                )?;
                let Some(similarity) = normalized_patch_correlation(&before, &after) else {
                    continue;
                };
                let candidate = PocketPatchMatch {
                    similarity,
                    shift_cross_mm: f64::from(shift_cross_px) * LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL,
                    shift_along_mm: f64::from(shift_along_px) * LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL,
                    rotation_deg: f64::from(rotation_deg),
                };
                if best.is_none_or(|best| candidate.similarity > best.similarity) {
                    best = Some(candidate);
                }
            }
        }
    }
    best
}

fn pocket_return_patch_half_span_mm(
    registration: &StripFeederRegistration,
    cross_axis: [f64; 2],
    along_axis: [f64; 2],
) -> [f64; 2] {
    let measured = registration
        .measured_part_size_mm
        .or(registration.surveyed_part_size_mm)
        .zip(registration.measured_part_rotation_deg);
    if let Some(([width_mm, length_mm], rotation_deg)) = measured {
        let radians = rotation_deg.to_radians();
        let length_axis = [radians.cos(), radians.sin()];
        let width_axis = [-length_axis[1], length_axis[0]];
        let projected_half_span = |axis: [f64; 2]| {
            0.5 * (length_mm * dot(length_axis, axis).abs()
                + width_mm * dot(width_axis, axis).abs())
                + 0.15
        };
        return [
            projected_half_span(cross_axis).clamp(0.35, registration.tape_width_mm * 0.35),
            projected_half_span(along_axis).clamp(0.30, registration.tape_pitch_mm * 0.40),
        ];
    }
    [
        (registration.tape_width_mm * 0.18).clamp(0.6, 1.5),
        (registration.tape_pitch_mm * 0.18).clamp(0.4, 1.0),
    ]
}

fn pocket_occupancy_from_evidence(confidence: f64) -> StripPocketOccupancy {
    if confidence >= 0.42 {
        StripPocketOccupancy::Occupied
    } else if confidence <= 0.16 {
        StripPocketOccupancy::Empty
    } else {
        StripPocketOccupancy::Uncertain
    }
}

fn render_pocket_return_preview(
    before: &GrayFrame,
    after: &GrayFrame,
    before_occupancy: StripPocketOccupancy,
    after_occupancy: StripPocketOccupancy,
    appearance_matches: bool,
    before_center_px: Option<[f64; 2]>,
    after_center_px: Option<[f64; 2]>,
) -> RgbFrame {
    let width = before.width + after.width;
    let height = before.height.max(after.height);
    let mut frame = RgbFrame {
        width,
        height,
        pixels: vec![0; (width * height * 3) as usize],
    };
    for (offset_x, source) in [(0, before), (before.width, after)] {
        for y in 0..source.height {
            for x in 0..source.width {
                let luma = source.pixels[(y * source.width + x) as usize];
                let destination = ((y * width + offset_x + x) * 3) as usize;
                frame.pixels[destination..destination + 3].fill(luma);
            }
        }
    }
    let color = |occupancy| match occupancy {
        StripPocketOccupancy::Occupied => [50, 235, 120],
        StripPocketOccupancy::Empty => [245, 70, 65],
        StripPocketOccupancy::Uncertain => [255, 190, 40],
    };
    let radius = f64::from(before.width.min(before.height)) * 0.10;
    draw_cross(
        &mut frame,
        [
            f64::from(before.width) * 0.5,
            f64::from(before.height) * 0.5,
        ],
        8.0,
        [0, 210, 255],
    );
    draw_cross(
        &mut frame,
        [
            f64::from(before.width) + f64::from(after.width) * 0.5,
            f64::from(after.height) * 0.5,
        ],
        8.0,
        [0, 210, 255],
    );
    draw_circle(
        &mut frame,
        before_center_px.unwrap_or([
            f64::from(before.width) * 0.5,
            f64::from(before.height) * 0.5,
        ]),
        radius,
        color(before_occupancy),
    );
    let after_center_px =
        after_center_px.unwrap_or([f64::from(after.width) * 0.5, f64::from(after.height) * 0.5]);
    draw_circle(
        &mut frame,
        [
            f64::from(before.width) + after_center_px[0],
            after_center_px[1],
        ],
        radius,
        if appearance_matches {
            color(after_occupancy)
        } else {
            [245, 70, 65]
        },
    );
    frame
}

fn local_mm_to_pixel(
    frame: &GrayFrame,
    local_mm: [f64; 2],
    cross_span_mm: f64,
    along_span_mm: f64,
) -> [f64; 2] {
    [
        local_mm[0] / cross_span_mm * f64::from(frame.width),
        local_mm[1] / along_span_mm * f64::from(frame.height),
    ]
}

#[allow(clippy::too_many_arguments)]
fn render_source_pose_preview(
    reference: &GrayFrame,
    candidate: &GrayFrame,
    reference_center_mm: [f64; 2],
    candidate_center_mm: [f64; 2],
    reference_package: Option<SourcePackageMeasurement>,
    candidate_package: Option<SourcePackageMeasurement>,
    cross_span_mm: f64,
    along_span_mm: f64,
) -> RgbFrame {
    let width = reference.width + candidate.width;
    let height = reference.height.max(candidate.height);
    let mut frame = RgbFrame {
        width,
        height,
        pixels: vec![0; (width * height * 3) as usize],
    };
    for (offset_x, source) in [(0, reference), (reference.width, candidate)] {
        for y in 0..source.height {
            for x in 0..source.width {
                let luma = source.pixels[(y * source.width + x) as usize];
                let destination = ((y * width + offset_x + x) * 3) as usize;
                frame.pixels[destination..destination + 3].fill(luma);
            }
        }
    }
    for (offset_x, source, center_mm, package) in [
        (0, reference, reference_center_mm, reference_package),
        (
            reference.width,
            candidate,
            candidate_center_mm,
            candidate_package,
        ),
    ] {
        let offset = [f64::from(offset_x), 0.0];
        let nominal = [
            f64::from(offset_x) + f64::from(source.width) * 0.5,
            f64::from(source.height) * 0.5,
        ];
        draw_cross(&mut frame, nominal, 8.0, [0, 210, 255]);
        let center = add(
            local_mm_to_pixel(source, center_mm, cross_span_mm, along_span_mm),
            offset,
        );
        if let Some(package) = package {
            let half_size_px = [
                0.5 * package.size_mm[0] / cross_span_mm * f64::from(source.width),
                0.5 * package.size_mm[1] / along_span_mm * f64::from(source.height),
            ];
            let corners = [
                [center[0] - half_size_px[0], center[1] - half_size_px[1]],
                [center[0] + half_size_px[0], center[1] - half_size_px[1]],
                [center[0] + half_size_px[0], center[1] + half_size_px[1]],
                [center[0] - half_size_px[0], center[1] + half_size_px[1]],
            ];
            for index in 0..corners.len() {
                draw_line(
                    &mut frame,
                    corners[index],
                    corners[(index + 1) % corners.len()],
                    [50, 235, 120],
                );
            }
        }
        draw_circle(&mut frame, center, 3.0, [235, 70, 235]);
        draw_cross(&mut frame, center, 5.0, [255, 215, 45]);
    }
    frame
}

fn render_live_holdout_preview(
    crop: &GrayFrame,
    circles: &[CircleFeature],
    measured_track: Option<(f64, f64)>,
    expected_tracks_mm: [f64; 2],
    expected_centerline_mm: f64,
    pockets_mm: &[[f64; 2]],
    selected_mm: Option<[f64; 2]>,
) -> RgbFrame {
    let mut pixels = Vec::with_capacity(crop.pixels.len() * 3);
    for luma in &crop.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut frame = RgbFrame {
        width: crop.width,
        height: crop.height,
        pixels,
    };
    let to_px = |point: [f64; 2]| {
        [
            point[0] / (f64::from(crop.width) * LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL)
                * f64::from(crop.width),
            point[1] / (f64::from(crop.height) * LIVE_HOLDOUT_OUTPUT_MM_PER_PIXEL)
                * f64::from(crop.height),
        ]
    };
    for cross_mm in expected_tracks_mm {
        draw_line(
            &mut frame,
            to_px([cross_mm, 0.0]),
            to_px([cross_mm, LIVE_HOLDOUT_ALONG_SPAN_MM]),
            [255, 190, 40],
        );
    }
    draw_line(
        &mut frame,
        to_px([expected_centerline_mm, 0.0]),
        to_px([expected_centerline_mm, LIVE_HOLDOUT_ALONG_SPAN_MM]),
        [30, 220, 245],
    );
    if let Some((cross_mm, slope)) = measured_track {
        draw_line(
            &mut frame,
            to_px([cross_mm - slope * LIVE_HOLDOUT_ALONG_SPAN_MM * 0.5, 0.0]),
            to_px([
                cross_mm + slope * LIVE_HOLDOUT_ALONG_SPAN_MM * 0.5,
                LIVE_HOLDOUT_ALONG_SPAN_MM,
            ]),
            [225, 60, 235],
        );
    }
    for circle in circles {
        draw_circle(&mut frame, circle.center_px, 5.0, [255, 215, 45]);
    }
    for pocket in pockets_mm {
        draw_cross(&mut frame, to_px(*pocket), 4.0, [50, 235, 120]);
    }
    if let Some(selected) = selected_mm {
        draw_circle(&mut frame, to_px(selected), 10.0, [50, 235, 120]);
    }
    frame
}

fn normalized(value: [f64; 2]) -> Option<[f64; 2]> {
    let length = value[0].hypot(value[1]);
    (length.is_finite() && length > 1e-9).then(|| scale(value, 1.0 / length))
}

fn detect_sprocket_circles(
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
) -> Result<Vec<CircleFeature>> {
    let source = gray_frame_to_mat(crop)?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &source,
        &mut blurred,
        Size::new(3, 3),
        0.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut edges = Mat::default();
    imgproc::canny(&blurred, &mut edges, 32.0, 96.0, 3, true)?;
    let mut contours = Vector::<Vector<Point>>::new();
    imgproc::find_contours(
        &edges,
        &mut contours,
        imgproc::RETR_LIST,
        imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0),
    )?;

    let x_mm_per_pixel = width_mm / f64::from(crop.width);
    let y_mm_per_pixel = height_mm / f64::from(crop.height);
    let mean_mm_per_pixel = 0.5 * (x_mm_per_pixel + y_mm_per_pixel);
    let mut features = Vec::new();
    for contour in contours {
        if contour.len() < 8 {
            continue;
        }
        let area = imgproc::contour_area(&contour, false)?.abs();
        let perimeter = imgproc::arc_length(&contour, true)?;
        if area <= 1.0 || perimeter <= 1.0 {
            continue;
        }
        let circularity = 4.0 * PI * area / (perimeter * perimeter);
        if circularity < 0.42 {
            continue;
        }
        let mut center = Point2f::default();
        let mut radius_px = 0.0_f32;
        imgproc::min_enclosing_circle(&contour, &mut center, &mut radius_px)?;
        let diameter_mm = 2.0 * f64::from(radius_px) * mean_mm_per_pixel;
        if !(SPROCKET_DIAMETER_MIN_MM..=SPROCKET_DIAMETER_MAX_MM).contains(&diameter_mm) {
            continue;
        }
        let center_px = [f64::from(center.x), f64::from(center.y)];
        if center_px[0] < 1.0
            || center_px[1] < 1.0
            || center_px[0] >= f64::from(crop.width - 1)
            || center_px[1] >= f64::from(crop.height - 1)
        {
            continue;
        }
        let diameter_score = (1.0 - (diameter_mm - 1.5).abs() / 0.7).clamp(0.0, 1.0);
        let confidence =
            (0.55 * circularity.clamp(0.0, 1.0) + 0.45 * diameter_score).clamp(0.0, 1.0);
        features.push(CircleFeature {
            center_px,
            confidence,
        });
    }
    let duplicate_distance_px = 0.65 / mean_mm_per_pixel;
    Ok(deduplicate_circle_features(features, duplicate_distance_px))
}

fn detect_live_sprocket_circles(
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
) -> Result<Vec<CircleFeature>> {
    let mut features = detect_sprocket_circles(crop, width_mm, height_mm)?;
    let source = gray_frame_to_mat(crop)?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &source,
        &mut blurred,
        Size::new(3, 3),
        0.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut dark_mask = Mat::default();
    imgproc::threshold(
        &blurred,
        &mut dark_mask,
        0.0,
        255.0,
        imgproc::THRESH_BINARY_INV | imgproc::THRESH_OTSU,
    )?;
    let mut contours = Vector::<Vector<Point>>::new();
    imgproc::find_contours(
        &dark_mask,
        &mut contours,
        imgproc::RETR_LIST,
        imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0),
    )?;
    let mean_mm_per_pixel =
        0.5 * (width_mm / f64::from(crop.width) + height_mm / f64::from(crop.height));
    for contour in contours {
        if contour.len() < 8 {
            continue;
        }
        let area = imgproc::contour_area(&contour, false)?.abs();
        let perimeter = imgproc::arc_length(&contour, true)?;
        if area <= 1.0 || perimeter <= 1.0 {
            continue;
        }
        let circularity = 4.0 * PI * area / (perimeter * perimeter);
        if circularity < 0.55 {
            continue;
        }
        let mut center = Point2f::default();
        let mut radius_px = 0.0_f32;
        imgproc::min_enclosing_circle(&contour, &mut center, &mut radius_px)?;
        let diameter_mm = 2.0 * f64::from(radius_px) * mean_mm_per_pixel;
        let fill_fraction = area / (PI * f64::from(radius_px).powi(2)).max(1.0);
        if !(SPROCKET_DIAMETER_MIN_MM..=SPROCKET_DIAMETER_MAX_MM).contains(&diameter_mm)
            || fill_fraction < 0.60
        {
            continue;
        }
        let center_px = [f64::from(center.x), f64::from(center.y)];
        if center_px[0] < 1.0
            || center_px[1] < 1.0
            || center_px[0] >= f64::from(crop.width - 1)
            || center_px[1] >= f64::from(crop.height - 1)
        {
            continue;
        }
        let diameter_score = (1.0 - (diameter_mm - 1.5).abs() / 0.7).clamp(0.0, 1.0);
        features.push(CircleFeature {
            center_px,
            confidence: (0.50 * circularity.clamp(0.0, 1.0)
                + 0.25 * fill_fraction.clamp(0.0, 1.0)
                + 0.25 * diameter_score)
                .clamp(0.0, 1.0),
        });
    }
    Ok(deduplicate_circle_features(
        features,
        0.65 / mean_mm_per_pixel,
    ))
}

fn deduplicate_circle_features(
    mut features: Vec<CircleFeature>,
    duplicate_distance_px: f64,
) -> Vec<CircleFeature> {
    features.sort_by(|first, second| second.confidence.total_cmp(&first.confidence));
    let mut deduplicated: Vec<CircleFeature> = Vec::new();
    for feature in features {
        if deduplicated.iter().any(|accepted| {
            (accepted.center_px[0] - feature.center_px[0])
                .hypot(accepted.center_px[1] - feature.center_px[1])
                <= duplicate_distance_px
        }) {
            continue;
        }
        deduplicated.push(feature);
    }
    deduplicated
}

fn build_sprocket_tracks(
    circles: &[CircleFeature],
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
    long_is_y: bool,
    minimum_sprockets: usize,
) -> Vec<SprocketTrack> {
    let local = circles
        .iter()
        .map(|circle| {
            let x_mm = (circle.center_px[0] + 0.5) / f64::from(crop.width) * width_mm;
            let y_mm = (circle.center_px[1] + 0.5) / f64::from(crop.height) * height_mm;
            let (cross_mm, along_mm) = if long_is_y {
                (x_mm, y_mm)
            } else {
                (y_mm, x_mm)
            };
            (cross_mm, along_mm, *circle)
        })
        .collect::<Vec<_>>();
    let mut ordered = local;
    ordered.sort_by(|first, second| first.0.total_cmp(&second.0));
    let mut clusters: Vec<Vec<(f64, f64, CircleFeature)>> = Vec::new();
    for feature in ordered {
        if let Some(cluster) = clusters.last_mut().filter(|cluster| {
            let mean = cluster.iter().map(|item| item.0).sum::<f64>() / cluster.len() as f64;
            (feature.0 - mean).abs() <= SPROCKET_TRACK_TOLERANCE_MM
        }) {
            cluster.push(feature);
        } else {
            clusters.push(vec![feature]);
        }
    }
    clusters
        .into_iter()
        .filter_map(|cluster| fit_sprocket_track(cluster, minimum_sprockets))
        .collect()
}

/// Separates an 8 mm tape's sprocket row from a circular component-pocket row.
///
/// Both rows can repeat at 4 mm. They cannot both be sprocket rows because
/// their 3.5 mm separation is smaller than the narrowest carrier tape. The
/// stronger circular lattice is retained as the sprocket row. The weaker row
/// remains explicit rejected evidence and supplies the pocket centerline.
fn resolve_pocket_row_competitors(
    mut tracks: Vec<SprocketTrack>,
) -> (Vec<SprocketTrack>, Vec<SuppressedPocketTrack>) {
    let mut pairs = Vec::new();
    for first in 0..tracks.len() {
        for second in first + 1..tracks.len() {
            let separation = (tracks[first].cross_mm - tracks[second].cross_mm).abs();
            let residual = (separation - EIGHT_MM_POCKET_CENTER_OFFSET_MM).abs();
            if residual <= POCKET_ROW_PAIR_TOLERANCE_MM {
                pairs.push((
                    first,
                    second,
                    residual,
                    tracks[first].confidence.max(tracks[second].confidence),
                ));
            }
        }
    }
    pairs.sort_by(|first, second| {
        second
            .3
            .total_cmp(&first.3)
            .then_with(|| first.2.total_cmp(&second.2))
    });

    let mut claimed = vec![false; tracks.len()];
    let mut rejected = vec![false; tracks.len()];
    let mut suppressed = Vec::new();
    for (first, second, _, _) in pairs {
        if claimed[first] || claimed[second] {
            continue;
        }
        let (sprocket, pocket) = if compare_track_quality(&tracks[first], &tracks[second]).is_ge() {
            (first, second)
        } else {
            (second, first)
        };
        claimed[sprocket] = true;
        claimed[pocket] = true;
        rejected[pocket] = true;
        tracks[sprocket].paired_pocket_centerline_mm = Some(tracks[pocket].cross_mm);
        suppressed.push(SuppressedPocketTrack {
            track: tracks[pocket].clone(),
            paired_sprocket_cross_mm: tracks[sprocket].cross_mm,
        });
    }

    let mut accepted = tracks
        .into_iter()
        .enumerate()
        .filter_map(|(index, track)| (!rejected[index]).then_some(track))
        .collect::<Vec<_>>();
    accepted.sort_by(|first, second| first.cross_mm.total_cmp(&second.cross_mm));
    suppressed.sort_by(|first, second| first.track.cross_mm.total_cmp(&second.track.cross_mm));
    (accepted, suppressed)
}

#[allow(clippy::too_many_arguments)]
fn infer_bank_pocket_side(
    suppressed: &[SuppressedPocketTrack],
    tracks: &[SprocketTrack],
    features: &[RectangularFeature],
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
    long_is_y: bool,
    cross_length_mm: f64,
    along_length_mm: f64,
) -> Option<f64> {
    if let Some(first) = suppressed
        .first()
        .map(|item| (item.track.cross_mm - item.paired_sprocket_cross_mm).signum())
    {
        if first != 0.0
            && suppressed
                .iter()
                .all(|item| (item.track.cross_mm - item.paired_sprocket_cross_mm).signum() == first)
        {
            return Some(first);
        }
    }

    let score_side = |side| {
        tracks
            .iter()
            .filter_map(|track| {
                let range = track_along_range(track, crop, width_mm, height_mm, long_is_y);
                let geometry = infer_tape_geometry(
                    features,
                    track.cross_mm,
                    range,
                    cross_length_mm,
                    along_length_mm,
                    Some(side),
                );
                geometry.0.map(|_| geometry.4)
            })
            .sum::<f64>()
    };
    let negative = score_side(-1.0);
    let positive = score_side(1.0);
    let strongest = negative.max(positive);
    let weakest = negative.min(positive);
    if strongest < 0.52 || strongest < weakest + 0.12 {
        None
    } else if positive > negative {
        Some(1.0)
    } else {
        Some(-1.0)
    }
}

fn compare_track_quality(first: &SprocketTrack, second: &SprocketTrack) -> std::cmp::Ordering {
    first
        .confidence
        .total_cmp(&second.confidence)
        .then_with(|| first.circles.len().cmp(&second.circles.len()))
        .then_with(|| second.residual_p95_mm.total_cmp(&first.residual_p95_mm))
        .then_with(|| second.cross_mm.total_cmp(&first.cross_mm))
}

fn detect_rectangular_features(
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
    long_is_y: bool,
) -> Result<Vec<RectangularFeature>> {
    let source = gray_frame_to_mat(crop)?;
    let mut blurred = Mat::default();
    imgproc::gaussian_blur(
        &source,
        &mut blurred,
        Size::new(3, 3),
        0.0,
        0.0,
        core::BORDER_DEFAULT,
    )?;
    let mut edges = Mat::default();
    imgproc::canny(&blurred, &mut edges, 24.0, 72.0, 3, true)?;
    let mut contours = Vector::<Vector<Point>>::new();
    imgproc::find_contours(
        &edges,
        &mut contours,
        imgproc::RETR_LIST,
        imgproc::CHAIN_APPROX_SIMPLE,
        Point::new(0, 0),
    )?;

    let x_mm_per_pixel = width_mm / f64::from(crop.width);
    let y_mm_per_pixel = height_mm / f64::from(crop.height);
    let mut features = Vec::new();
    for contour in contours {
        if contour.len() < 4 {
            continue;
        }
        let bounds = imgproc::bounding_rect(&contour)?;
        if bounds.width < 3 || bounds.height < 3 {
            continue;
        }
        let x_size_mm = f64::from(bounds.width) * x_mm_per_pixel;
        let y_size_mm = f64::from(bounds.height) * y_mm_per_pixel;
        let (cross_size_mm, along_size_mm) = if long_is_y {
            (x_size_mm, y_size_mm)
        } else {
            (y_size_mm, x_size_mm)
        };
        if !(0.25..=5.0).contains(&cross_size_mm) || !(0.25..=4.0).contains(&along_size_mm) {
            continue;
        }
        let area = imgproc::contour_area(&contour, false)?.abs();
        let bounds_area = f64::from(bounds.width * bounds.height);
        if area < 2.0 || bounds_area <= 0.0 {
            continue;
        }
        let rectangularity = (area / bounds_area).clamp(0.0, 1.0);
        if rectangularity < 0.08 {
            continue;
        }
        let perimeter = imgproc::arc_length(&contour, true)?;
        let circularity = if perimeter > 1e-9 {
            (4.0 * PI * area / (perimeter * perimeter)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let x_px = f64::from(bounds.x) + 0.5 * f64::from(bounds.width);
        let y_px = f64::from(bounds.y) + 0.5 * f64::from(bounds.height);
        let x_mm = x_px * x_mm_per_pixel;
        let y_mm = y_px * y_mm_per_pixel;
        let (cross_mm, along_mm) = if long_is_y {
            (x_mm, y_mm)
        } else {
            (y_mm, x_mm)
        };
        let dimension_score = (cross_size_mm.min(along_size_mm) / 0.8).clamp(0.0, 1.0);
        let confidence =
            (0.55 * rectangularity + 0.25 * (1.0 - circularity) + 0.20 * dimension_score)
                .clamp(0.0, 1.0);
        features.push(RectangularFeature {
            cross_mm,
            along_mm,
            cross_size_mm,
            along_size_mm,
            rectangularity,
            circularity,
            confidence,
        });
    }
    Ok(features)
}

fn fit_sprocket_track(
    cluster: Vec<(f64, f64, CircleFeature)>,
    minimum_sprockets: usize,
) -> Option<SprocketTrack> {
    if cluster.len() < minimum_sprockets {
        return None;
    }
    let phases = cluster
        .iter()
        .map(|item| item.1.rem_euclid(STRIP_FEEDER_SPROCKET_PITCH_MM))
        .collect::<Vec<_>>();
    let mut best_inliers = Vec::new();
    let mut best_residuals = Vec::new();
    for phase in phases {
        let mut inliers = Vec::new();
        let mut residuals = Vec::new();
        for item in &cluster {
            let residual = lattice_residual_mm(item.1, phase);
            if residual <= SPROCKET_LATTICE_TOLERANCE_MM {
                inliers.push(*item);
                residuals.push(residual);
            }
        }
        if inliers.len() > best_inliers.len()
            || (inliers.len() == best_inliers.len() && median(&residuals) < median(&best_residuals))
        {
            best_inliers = inliers;
            best_residuals = residuals;
        }
    }
    if best_inliers.len() < minimum_sprockets {
        return None;
    }
    best_inliers.sort_by(|first, second| first.1.total_cmp(&second.1));
    if best_inliers.last()?.1 - best_inliers.first()?.1
        < 2.0 * STRIP_FEEDER_SPROCKET_PITCH_MM - SPROCKET_LATTICE_TOLERANCE_MM
    {
        return None;
    }
    best_residuals.sort_by(f64::total_cmp);
    let residual_p95_mm = percentile(&best_residuals, 0.95);
    let cross_mm = median(&best_inliers.iter().map(|item| item.0).collect::<Vec<_>>());
    let detection_confidence = best_inliers
        .iter()
        .map(|item| item.2.confidence)
        .sum::<f64>()
        / best_inliers.len() as f64;
    let inlier_fraction = best_inliers.len() as f64 / cluster.len() as f64;
    let residual_score = (1.0 - residual_p95_mm / SPROCKET_LATTICE_TOLERANCE_MM).clamp(0.0, 1.0);
    Some(SprocketTrack {
        cross_mm,
        circles: best_inliers.iter().map(|item| item.2).collect(),
        residual_p95_mm,
        confidence: (0.45 * detection_confidence + 0.35 * inlier_fraction + 0.20 * residual_score)
            .clamp(0.0, 1.0),
        paired_pocket_centerline_mm: None,
    })
}

#[allow(clippy::too_many_arguments)]
fn lane_proposal(
    proposal_index: u32,
    track: &SprocketTrack,
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    cross_length_mm: f64,
    along_length_mm: f64,
    rectangular_features: &[RectangularFeature],
    preferred_pocket_side: Option<f64>,
) -> StripFeederLaneProposal {
    let mut findings = Vec::new();
    let sprocket_along_range = track_along_range(
        track,
        crop,
        candidate.width_mm,
        candidate.height_mm,
        long_is_y,
    );
    let (tape_width_mm, centerline, component_pitch_mm, phase_mm, periodicity_confidence) =
        if let Some(cross_mm) = track.paired_pocket_centerline_mm {
            let (pitch_mm, phase_mm, confidence) = estimate_component_grid(
                rectangular_features,
                cross_mm,
                sprocket_along_range,
                along_length_mm,
            );
            findings.push(
                "8 mm tape width and pocket side were resolved from the paired 3.5 mm pocket row"
                    .into(),
            );
            (Some(8.0), Some(cross_mm), pitch_mm, phase_mm, confidence)
        } else {
            let geometry = infer_tape_geometry(
                rectangular_features,
                track.cross_mm,
                sprocket_along_range,
                cross_length_mm,
                along_length_mm,
                preferred_pocket_side,
            );
            if geometry.0.is_some() && preferred_pocket_side.is_some() {
                findings
                    .push("pocket side follows the measured orientation of this tape bank".into());
            }
            geometry
        };
    if tape_width_mm.is_none() {
        findings.push(
            "tape width and pocket side are unresolved because no component lattice was found"
                .into(),
        );
    }
    if component_pitch_mm.is_none() && centerline.is_some() {
        findings.push("component pocket pitch is unresolved in the retained image".into());
    }

    let sprocket_centers_machine_mm = track
        .circles
        .iter()
        .map(|circle| crop_pixel_to_machine(circle.center_px, crop, candidate))
        .collect::<Vec<_>>();
    let pick_centerline_machine_mm = centerline.map(|cross_mm| {
        [
            local_axis_to_machine(cross_mm, 0.0, candidate, long_is_y),
            local_axis_to_machine(cross_mm, along_length_mm, candidate, long_is_y),
        ]
    });
    let pocket_observations = match (centerline, component_pitch_mm, phase_mm) {
        (Some(cross_mm), Some(pitch_mm), Some(phase_mm)) => observe_component_pockets(
            crop,
            rectangular_features,
            cross_mm,
            pitch_mm,
            phase_mm,
            sprocket_along_range,
            candidate,
            long_is_y,
            periodicity_confidence,
        ),
        _ => Vec::new(),
    };
    let pocket_centers_machine_mm = pocket_observations
        .iter()
        .map(|observation| observation.center_machine_mm)
        .collect::<Vec<_>>();
    let present_count = pocket_observations
        .iter()
        .filter(|observation| observation.occupancy == StripPocketOccupancy::Occupied)
        .count() as u32;
    let absent_count = pocket_observations
        .iter()
        .filter(|observation| observation.occupancy == StripPocketOccupancy::Empty)
        .count() as u32;
    let uncertain_count = pocket_observations.len() as u32 - present_count - absent_count;
    let component_size_mm = median_component_size(&pocket_observations);
    if !pocket_observations.is_empty() {
        findings.push(format!(
            "{present_count} present, {absent_count} absent, and {uncertain_count} uncertain feed positions were measured"
        ));
    }
    let width_score = if tape_width_mm.is_some() { 1.0 } else { 0.0 };
    let centerline_score = if centerline.is_some() { 1.0 } else { 0.0 };
    let pitch_score = if component_pitch_mm.is_some() {
        periodicity_confidence
    } else {
        0.0
    };
    let confidence = (0.55 * track.confidence
        + 0.15 * width_score
        + 0.15 * centerline_score
        + 0.15 * pitch_score)
        .clamp(0.0, 1.0);
    let review_required = tape_width_mm.is_none()
        || centerline.is_none()
        || component_pitch_mm.is_none()
        || confidence < 0.85;
    if !review_required {
        findings.push(
            "image geometry passed the draft gate; live pocket and occupancy validation remains required"
                .into(),
        );
    }
    let direction = if long_is_y {
        subtract(
            candidate.outline_machine_mm[3],
            candidate.outline_machine_mm[0],
        )
    } else {
        subtract(
            candidate.outline_machine_mm[1],
            candidate.outline_machine_mm[0],
        )
    };
    StripFeederLaneProposal {
        proposal_index,
        tape_rotation_deg: direction[1].atan2(direction[0]).to_degrees(),
        sprocket_pitch_mm: STRIP_FEEDER_SPROCKET_PITCH_MM,
        sprocket_pitch_residual_p95_mm: track.residual_p95_mm,
        sprocket_center_count: sprocket_centers_machine_mm.len() as u32,
        sprocket_centers_machine_mm,
        tape_width_mm,
        component_pitch_mm,
        pick_centerline_machine_mm,
        pocket_centers_machine_mm,
        component_size_mm,
        pocket_observations,
        present_count,
        absent_count,
        uncertain_count,
        confidence,
        review_required,
        findings,
    }
}

fn rejected_lattice(
    suppressed: &SuppressedPocketTrack,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    along_length_mm: f64,
) -> StripFeederRejectedLattice {
    let pocket_cross_mm = suppressed.track.cross_mm;
    let sprocket_cross_mm = suppressed.paired_sprocket_cross_mm;
    let offset_mm = (pocket_cross_mm - sprocket_cross_mm).abs();
    StripFeederRejectedLattice {
        centerline_machine_mm: [
            local_axis_to_machine(pocket_cross_mm, 0.0, candidate, long_is_y),
            local_axis_to_machine(
                pocket_cross_mm,
                along_length_mm,
                candidate,
                long_is_y,
            ),
        ],
        paired_sprocket_centerline_machine_mm: [
            local_axis_to_machine(sprocket_cross_mm, 0.0, candidate, long_is_y),
            local_axis_to_machine(
                sprocket_cross_mm,
                along_length_mm,
                candidate,
                long_is_y,
            ),
        ],
        offset_mm,
        center_count: suppressed.track.circles.len() as u32,
        confidence: suppressed.track.confidence,
        reason: format!(
            "the weaker 4 mm lattice is {offset_mm:.3} mm from a stronger sprocket row and matches an 8 mm tape pocket centerline",
        ),
    }
}

fn infer_tape_geometry(
    features: &[RectangularFeature],
    sprocket_cross_mm: f64,
    along_range: Option<[f64; 2]>,
    cross_length_mm: f64,
    along_length_mm: f64,
    preferred_side: Option<f64>,
) -> (Option<f64>, Option<f64>, Option<f64>, Option<f64>, f64) {
    let sides = preferred_side.map_or_else(|| vec![-1.0, 1.0], |side| vec![side]);
    let mut candidates = Vec::new();
    for width_mm in STANDARD_TAPE_WIDTHS_MM {
        let offset_mm = width_mm * 0.5 - POCKET_CENTERLINE_EDGE_REMAINDER_MM;
        for side in &sides {
            let cross_mm = sprocket_cross_mm + side * offset_mm;
            if cross_mm < 0.5 || cross_mm > cross_length_mm - 0.5 {
                continue;
            }
            let (pitch_mm, phase_mm, confidence) =
                estimate_component_grid(features, cross_mm, along_range, along_length_mm);
            if pitch_mm.is_some() && phase_mm.is_some() {
                candidates.push((width_mm, cross_mm, pitch_mm, phase_mm, confidence));
            }
        }
    }
    candidates.sort_by(|first, second| {
        second
            .4
            .total_cmp(&first.4)
            .then_with(|| first.0.total_cmp(&second.0))
    });
    candidates.first().copied().map_or(
        (None, None, None, None, 0.0),
        |(width, cross, pitch, phase, confidence)| {
            (Some(width), Some(cross), pitch, phase, confidence)
        },
    )
}

fn track_along_range(
    track: &SprocketTrack,
    crop: &GrayFrame,
    width_mm: f64,
    height_mm: f64,
    long_is_y: bool,
) -> Option<[f64; 2]> {
    let values = track.circles.iter().map(|circle| {
        if long_is_y {
            (circle.center_px[1] + 0.5) / f64::from(crop.height) * height_mm
        } else {
            (circle.center_px[0] + 0.5) / f64::from(crop.width) * width_mm
        }
    });
    let mut minimum = f64::INFINITY;
    let mut maximum = f64::NEG_INFINITY;
    for value in values {
        minimum = minimum.min(value);
        maximum = maximum.max(value);
    }
    (minimum.is_finite() && maximum.is_finite() && maximum > minimum).then_some([minimum, maximum])
}

fn estimate_component_grid(
    features: &[RectangularFeature],
    cross_mm: f64,
    along_range: Option<[f64; 2]>,
    along_length_mm: f64,
) -> (Option<f64>, Option<f64>, f64) {
    let range = along_range.unwrap_or([0.0, along_length_mm]);
    let mut ordered = features
        .iter()
        .copied()
        .filter(|feature| {
            let aspect = feature.cross_size_mm.max(feature.along_size_mm)
                / feature.cross_size_mm.min(feature.along_size_mm).max(1e-9);
            (feature.cross_mm - cross_mm).abs() <= 1.8
                && feature.along_mm >= range[0] - 1.0
                && feature.along_mm <= range[1] + 1.0
                && (feature.circularity < 0.72 || aspect >= 1.25)
        })
        .collect::<Vec<_>>();
    ordered.sort_by(|first, second| first.along_mm.total_cmp(&second.along_mm));
    let mut clusters: Vec<ComponentFeatureCluster> = Vec::new();
    for feature in ordered {
        if let Some(cluster) = clusters
            .last_mut()
            .filter(|cluster| (feature.along_mm - cluster.along_mm).abs() <= 0.65)
        {
            cluster.features.push(feature);
            let total = cluster
                .features
                .iter()
                .map(|item| item.confidence)
                .sum::<f64>();
            cluster.along_mm = cluster
                .features
                .iter()
                .map(|item| item.along_mm * item.confidence)
                .sum::<f64>()
                / total.max(1e-9);
            cluster.confidence = cluster.confidence.max(feature.confidence);
        } else {
            clusters.push(ComponentFeatureCluster {
                along_mm: feature.along_mm,
                features: vec![feature],
                confidence: feature.confidence,
            });
        }
    }
    if clusters.len() < 3 {
        return (None, None, 0.0);
    }

    let mut fits = Vec::new();
    for pitch_mm in STANDARD_COMPONENT_PITCHES_MM {
        if range[1] - range[0] < 2.0 * pitch_mm {
            continue;
        }
        for seed in &clusters {
            let phase_mm = seed.along_mm.rem_euclid(pitch_mm);
            let first_slot = ((range[0] - phase_mm) / pitch_mm).ceil() as i32;
            let last_slot = ((range[1] - phase_mm) / pitch_mm).floor() as i32;
            let expected = (last_slot - first_slot + 1).max(0) as usize;
            if expected < 3 {
                continue;
            }
            let mut occupied_slots = std::collections::BTreeSet::new();
            let mut residuals = Vec::new();
            for cluster in &clusters {
                let slot = ((cluster.along_mm - phase_mm) / pitch_mm).round() as i32;
                if slot < first_slot || slot > last_slot {
                    continue;
                }
                let residual = (cluster.along_mm - (phase_mm + f64::from(slot) * pitch_mm)).abs();
                if residual <= 0.55 {
                    occupied_slots.insert(slot);
                    residuals.push(residual);
                }
            }
            let coverage = occupied_slots.len() as f64 / expected as f64;
            let precision = residuals.len() as f64 / clusters.len() as f64;
            let residual = median(&residuals);
            let residual_score = if residual.is_finite() {
                (1.0 - residual / 0.55).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let score = 0.50 * coverage + 0.30 * precision + 0.20 * residual_score;
            fits.push((pitch_mm, phase_mm, score));
        }
    }
    fits.sort_by(|first, second| {
        second
            .2
            .total_cmp(&first.2)
            .then_with(|| first.0.total_cmp(&second.0))
    });
    let best_for_pitch = |pitch: f64| {
        fits.iter()
            .filter(|fit| fit.0 == pitch)
            .max_by(|first, second| first.2.total_cmp(&second.2))
            .copied()
    };
    let expected_two_mm_positions = ((range[1] - range[0]) / 2.0).floor().max(1.0) as usize;
    let dense_two_mm_evidence = clusters.len() * 4 >= expected_two_mm_positions * 3;
    let selected = match (best_for_pitch(2.0), best_for_pitch(4.0)) {
        (Some(two), Some(four))
            if dense_two_mm_evidence && two.2 >= 0.65 && two.2 >= four.2 + 0.10 =>
        {
            two
        }
        (_, Some(four)) if four.2 >= 0.25 => four,
        _ => {
            let Some(best) = fits.first().copied() else {
                return (None, None, 0.0);
            };
            best
        }
    };
    let (pitch_mm, phase_mm, score) = selected;
    if !score.is_finite() {
        return (None, None, 0.0);
    }
    if score < 0.52 {
        (None, None, score)
    } else {
        (Some(pitch_mm), Some(phase_mm), score.clamp(0.0, 1.0))
    }
}

#[allow(clippy::too_many_arguments)]
fn observe_component_pockets(
    crop: &GrayFrame,
    features: &[RectangularFeature],
    cross_mm: f64,
    pitch_mm: f64,
    phase_mm: f64,
    along_range: Option<[f64; 2]>,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    grid_confidence: f64,
) -> Vec<StripFeederPocketObservation> {
    let Some(range) = along_range else {
        return Vec::new();
    };
    let base_first_slot = ((range[0] - phase_mm) / pitch_mm).ceil() as i32;
    let base_last_slot = ((range[1] - phase_mm) / pitch_mm).floor() as i32;
    let (first_slot, last_slot) = extend_visible_pocket_slot_range(
        crop,
        cross_mm,
        pitch_mm,
        phase_mm,
        range,
        base_first_slot,
        base_last_slot,
        candidate,
        long_is_y,
    );
    let slots = (first_slot..=last_slot)
        .map(|slot| phase_mm + f64::from(slot) * pitch_mm)
        .collect::<Vec<_>>();
    let appearance = slots
        .iter()
        .map(|along_mm| {
            component_window_appearance(crop, candidate, long_is_y, cross_mm, *along_mm, pitch_mm)
        })
        .collect::<Vec<_>>();
    let mut observations = slots
        .iter()
        .copied()
        .enumerate()
        .map(|(index, along_mm)| {
            let mut candidates = features
                .iter()
                .copied()
                .filter(|feature| {
                    (feature.cross_mm - cross_mm).abs() <= 1.2
                        && (feature.along_mm - along_mm).abs() <= 0.8
                        && feature.cross_size_mm >= 0.3
                        && feature.along_size_mm >= 0.25
                        && feature.cross_size_mm <= 3.3
                        && feature.along_size_mm <= (0.65 * pitch_mm).min(2.6)
                        && (feature.circularity < 0.72
                            || feature.cross_size_mm.max(feature.along_size_mm)
                                / feature.cross_size_mm.min(feature.along_size_mm)
                                >= 1.25)
                })
                .collect::<Vec<_>>();
            candidates.sort_by(|first, second| {
                let first_score = component_outline_score(*first, cross_mm, along_mm);
                let second_score = component_outline_score(*second, cross_mm, along_mm);
                second_score.total_cmp(&first_score)
            });
            let component = candidates.first().copied().filter(|feature| {
                component_outline_score(*feature, cross_mm, along_mm)
                    >= MINIMUM_COMPONENT_OUTLINE_SCORE
            });
            let occupancy = if component.is_some() {
                StripPocketOccupancy::Occupied
            } else {
                StripPocketOccupancy::Empty
            };
            let confidence = component.map_or(0.70 * grid_confidence, |feature| {
                (0.45 * grid_confidence
                    + 0.55 * component_outline_score(feature, cross_mm, along_mm))
                .clamp(0.0, 1.0)
            });
            let component_size_mm =
                component.map(|feature| [feature.cross_size_mm, feature.along_size_mm]);
            let component_outline_machine_mm = component.map_or_else(Vec::new, |feature| {
                component_outline(
                    cross_mm,
                    along_mm,
                    [feature.cross_size_mm, feature.along_size_mm],
                    candidate,
                    long_is_y,
                )
            });
            StripFeederPocketObservation {
                pocket_index: index as u32,
                center_machine_mm: local_axis_to_machine(cross_mm, along_mm, candidate, long_is_y),
                occupancy,
                confidence,
                component_outline_machine_mm,
                component_size_mm,
            }
        })
        .collect::<Vec<_>>();
    let seed_occupied = observations
        .iter()
        .map(|observation| observation.occupancy == StripPocketOccupancy::Occupied)
        .collect::<Vec<_>>();
    let seed_fraction = seed_occupied.iter().filter(|occupied| **occupied).count() as f64
        / seed_occupied.len().max(1) as f64;

    let luma = appearance
        .iter()
        .map(|appearance| appearance.mean_luma)
        .collect::<Vec<_>>();
    let luma_classification = classify_component_luma(&luma, &observations);
    let mut appearance_applied = false;
    if seed_fraction >= 0.7 {
        if let Some((occupancy, confidence)) = classify_bright_tape_dark_parts(&appearance) {
            apply_occupancy_states(&mut observations, &occupancy, grid_confidence, confidence);
            appearance_applied = true;
        }
    }
    if !appearance_applied {
        if let Some((labels, occupied_label, separation_score, decisive)) = &luma_classification {
            if *decisive {
                apply_appearance_labels(
                    &mut observations,
                    labels,
                    *occupied_label,
                    grid_confidence,
                    *separation_score,
                );
                appearance_applied = true;
            }
        }
    }
    if !appearance_applied {
        if let Some(confidence) = classify_homogeneous_populated_lane(&appearance, seed_fraction) {
            let occupied = vec![true; observations.len()];
            apply_occupied_mask(&mut observations, &occupied, grid_confidence, confidence);
            appearance_applied = true;
        }
    }
    if !appearance_applied && seed_fraction < 0.85 && appearance.len() <= 24 {
        if let Some((occupied, confidence)) =
            classify_component_change_point(&appearance, &seed_occupied)
        {
            apply_occupied_mask(&mut observations, &occupied, grid_confidence, confidence);
            mark_weak_transition_uncertain(&mut observations, confidence);
            appearance_applied = true;
        }
    }
    if !appearance_applied && seed_fraction <= 0.5 {
        if let Some((labels, occupied_label, confidence)) =
            classify_component_edge_energy(&appearance, &seed_occupied)
        {
            apply_appearance_labels(
                &mut observations,
                &labels,
                occupied_label,
                grid_confidence,
                confidence,
            );
            mark_weak_transition_uncertain(&mut observations, confidence);
            appearance_applied = true;
        }
    }
    if !appearance_applied {
        if let Some((labels, occupied_label, separation_score, _)) = luma_classification {
            for (index, observation) in observations.iter_mut().enumerate() {
                if observation.occupancy != StripPocketOccupancy::Occupied
                    && labels[index] == occupied_label
                {
                    observation.occupancy = StripPocketOccupancy::Occupied;
                    observation.confidence =
                        (0.55 * grid_confidence + 0.45 * separation_score).clamp(0.0, 1.0);
                }
            }
        }
    }

    if let Some(size_mm) = median_component_size(&observations) {
        for (index, observation) in observations.iter_mut().enumerate() {
            if observation.occupancy == StripPocketOccupancy::Occupied
                && observation.component_size_mm.is_none()
            {
                observation.component_size_mm = Some(size_mm);
                observation.component_outline_machine_mm =
                    component_outline(cross_mm, slots[index], size_mm, candidate, long_is_y);
            }
        }
    }
    observations
}

#[allow(clippy::too_many_arguments)]
fn extend_visible_pocket_slot_range(
    crop: &GrayFrame,
    cross_mm: f64,
    pitch_mm: f64,
    phase_mm: f64,
    sprocket_along_range: [f64; 2],
    base_first_slot: i32,
    base_last_slot: i32,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
) -> (i32, i32) {
    if base_first_slot > base_last_slot {
        return (base_first_slot, base_last_slot);
    }
    let along_length_mm = if long_is_y {
        candidate.height_mm
    } else {
        candidate.width_mm
    };
    let negative_padding_mm = NEGATIVE_ENDPOINT_RANGE_PADDING_FRACTION * pitch_mm;
    let positive_padding_mm = POSITIVE_ENDPOINT_RANGE_PADDING_FRACTION * pitch_mm;
    let endpoint_reference = |positive_end: bool| {
        let slots: Box<dyn Iterator<Item = i32>> = if positive_end {
            Box::new((base_first_slot.max(base_last_slot - 3)..=base_last_slot).rev())
        } else {
            Box::new(base_first_slot..=base_last_slot.min(base_first_slot + 3))
        };
        slots
            .map(|slot| phase_mm + f64::from(slot) * pitch_mm)
            .collect::<Vec<_>>()
    };
    let slot_correlation = |slot: i32, positive_end: bool| {
        let along_mm = phase_mm + f64::from(slot) * pitch_mm;
        if along_mm < 0.75 * pitch_mm || along_mm > along_length_mm - 0.75 * pitch_mm {
            return None;
        }
        let references = endpoint_reference(positive_end);
        Some(endpoint_patch_correlation(
            crop,
            candidate,
            long_is_y,
            cross_mm,
            along_mm,
            &references,
        ))
    };
    let mut first_slot = base_first_slot;
    let negative_candidate = base_first_slot - 1;
    let negative_along_mm = phase_mm + f64::from(negative_candidate) * pitch_mm;
    let negative_correlation = slot_correlation(negative_candidate, false);
    if sprocket_along_range[0] - negative_along_mm <= negative_padding_mm
        && negative_correlation.is_some_and(|score| score >= MINIMUM_ENDPOINT_PATCH_CORRELATION)
    {
        first_slot = negative_candidate;
    }

    let mut last_slot = base_last_slot;
    let positive_candidate = base_last_slot + 1;
    let positive_along_mm = phase_mm + f64::from(positive_candidate) * pitch_mm;
    let positive_correlation = slot_correlation(positive_candidate, true);
    if positive_along_mm - sprocket_along_range[1] <= positive_padding_mm
        && positive_correlation.is_some_and(|score| score >= MINIMUM_ENDPOINT_PATCH_CORRELATION)
    {
        last_slot = positive_candidate;
        // A feeder body can hide a terminal sprocket while the next component
        // pocket stays visible. Admit one more slot only with direct evidence.
        let second_candidate = positive_candidate + 1;
        if slot_correlation(second_candidate, true)
            .is_some_and(|score| score >= MINIMUM_SECOND_ENDPOINT_PATCH_CORRELATION)
        {
            last_slot = second_candidate;
        }
    }
    (first_slot, last_slot)
}

#[allow(clippy::too_many_arguments)]
fn endpoint_patch_correlation(
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    cross_mm: f64,
    along_mm: f64,
    reference_along_mm: &[f64],
) -> f64 {
    let Some(candidate_patch) = component_patch(
        crop,
        candidate,
        long_is_y,
        cross_mm,
        along_mm,
        ENDPOINT_PATCH_RADIUS_MM,
    ) else {
        return -1.0;
    };
    reference_along_mm
        .iter()
        .filter_map(|reference| {
            let patch = component_patch(
                crop,
                candidate,
                long_is_y,
                cross_mm,
                *reference,
                ENDPOINT_PATCH_RADIUS_MM,
            )?;
            normalized_patch_correlation(&candidate_patch, &patch)
        })
        .max_by(f64::total_cmp)
        .unwrap_or(-1.0)
}

fn component_patch(
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    cross_mm: f64,
    along_mm: f64,
    radius_mm: f64,
) -> Option<Vec<f64>> {
    let cross_length_mm = if long_is_y {
        candidate.width_mm
    } else {
        candidate.height_mm
    };
    let along_length_mm = if long_is_y {
        candidate.height_mm
    } else {
        candidate.width_mm
    };
    let cross_pixels = if long_is_y { crop.width } else { crop.height };
    let along_pixels = if long_is_y { crop.height } else { crop.width };
    let cross_center = cross_mm / cross_length_mm * f64::from(cross_pixels);
    let along_center = along_mm / along_length_mm * f64::from(along_pixels);
    let cross_radius = (radius_mm / cross_length_mm * f64::from(cross_pixels)).ceil() as i32;
    let along_radius = (radius_mm / along_length_mm * f64::from(along_pixels)).ceil() as i32;
    let cross_center = cross_center.round() as i32;
    let along_center = along_center.round() as i32;
    if cross_center - cross_radius < 0
        || along_center - along_radius < 0
        || cross_center + cross_radius >= cross_pixels as i32
        || along_center + along_radius >= along_pixels as i32
    {
        return None;
    }
    let mut patch = Vec::new();
    for along in along_center - along_radius..=along_center + along_radius {
        for cross in cross_center - cross_radius..=cross_center + cross_radius {
            let (x, y) = if long_is_y {
                (cross as u32, along as u32)
            } else {
                (along as u32, cross as u32)
            };
            patch.push(f64::from(crop.pixels[(y * crop.width + x) as usize]));
        }
    }
    Some(patch)
}

fn normalized_patch_correlation(first: &[f64], second: &[f64]) -> Option<f64> {
    if first.len() != second.len() || first.is_empty() {
        return None;
    }
    let first_mean = first.iter().sum::<f64>() / first.len() as f64;
    let second_mean = second.iter().sum::<f64>() / second.len() as f64;
    let numerator = first
        .iter()
        .zip(second)
        .map(|(first, second)| (first - first_mean) * (second - second_mean))
        .sum::<f64>();
    let first_energy = first
        .iter()
        .map(|value| (value - first_mean).powi(2))
        .sum::<f64>();
    let second_energy = second
        .iter()
        .map(|value| (value - second_mean).powi(2))
        .sum::<f64>();
    let denominator = (first_energy * second_energy).sqrt();
    (denominator > 1e-9).then_some(numerator / denominator)
}

fn component_window_appearance(
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    cross_mm: f64,
    along_mm: f64,
    pitch_mm: f64,
) -> PocketAppearance {
    let cross_length_mm = if long_is_y {
        candidate.width_mm
    } else {
        candidate.height_mm
    };
    let along_length_mm = if long_is_y {
        candidate.height_mm
    } else {
        candidate.width_mm
    };
    let cross_pixels = if long_is_y { crop.width } else { crop.height };
    let along_pixels = if long_is_y { crop.height } else { crop.width };
    let cross_center = cross_mm / cross_length_mm * f64::from(cross_pixels);
    let along_center = along_mm / along_length_mm * f64::from(along_pixels);
    let cross_radius = (0.6 / cross_length_mm * f64::from(cross_pixels)).ceil() as i32;
    let along_radius =
        ((0.5_f64.min(0.2 * pitch_mm)) / along_length_mm * f64::from(along_pixels)).ceil() as i32;
    let mut sum = 0_u64;
    let mut count = 0_u64;
    let mut edge_sum = 0_u64;
    let mut edge_count = 0_u64;
    for along in
        along_center.round() as i32 - along_radius..=along_center.round() as i32 + along_radius
    {
        for cross in
            cross_center.round() as i32 - cross_radius..=cross_center.round() as i32 + cross_radius
        {
            if cross < 0
                || along < 0
                || cross >= cross_pixels as i32
                || along >= along_pixels as i32
            {
                continue;
            }
            let (x, y) = if long_is_y {
                (cross as u32, along as u32)
            } else {
                (along as u32, cross as u32)
            };
            sum += u64::from(crop.pixels[(y * crop.width + x) as usize]);
            count += 1;
            if x > 0 && y > 0 && x + 1 < crop.width && y + 1 < crop.height {
                let left = crop.pixels[(y * crop.width + x - 1) as usize];
                let right = crop.pixels[(y * crop.width + x + 1) as usize];
                let top = crop.pixels[((y - 1) * crop.width + x) as usize];
                let bottom = crop.pixels[((y + 1) * crop.width + x) as usize];
                edge_sum += u64::from(left.abs_diff(right)) + u64::from(top.abs_diff(bottom));
                edge_count += 2;
            }
        }
    }
    PocketAppearance {
        mean_luma: sum as f64 / count.max(1) as f64,
        edge_energy: edge_sum as f64 / edge_count.max(1) as f64,
    }
}

fn classify_component_luma(
    values: &[f64],
    observations: &[StripFeederPocketObservation],
) -> Option<(Vec<usize>, usize, f64, bool)> {
    if values.len() < 4 || values.len() != observations.len() {
        return None;
    }
    let mut centers = [
        values.iter().copied().min_by(f64::total_cmp)?,
        values.iter().copied().max_by(f64::total_cmp)?,
    ];
    if centers[1] - centers[0] < 8.0 {
        return None;
    }
    let mut labels = vec![0_usize; values.len()];
    for _ in 0..12 {
        for (index, value) in values.iter().enumerate() {
            labels[index] = usize::from((value - centers[1]).abs() < (value - centers[0]).abs());
        }
        for (label, center) in centers.iter_mut().enumerate() {
            let selected = values
                .iter()
                .zip(&labels)
                .filter(|(_, assigned)| **assigned == label)
                .map(|(value, _)| *value)
                .collect::<Vec<_>>();
            if selected.is_empty() {
                return None;
            }
            *center = selected.iter().sum::<f64>() / selected.len() as f64;
        }
    }
    let counts = [
        labels.iter().filter(|label| **label == 0).count(),
        labels.iter().filter(|label| **label == 1).count(),
    ];
    if counts[0] < 2 || counts[1] < 2 || (centers[1] - centers[0]).abs() < 10.0 {
        return None;
    }
    let mut votes = [0_u32; 2];
    for (index, observation) in observations.iter().enumerate() {
        if observation.occupancy == StripPocketOccupancy::Occupied {
            votes[labels[index]] += 1;
        }
    }
    let occupied_label = if votes[0] == votes[1] {
        usize::from(centers[1] > centers[0])
    } else {
        usize::from(votes[1] > votes[0])
    };
    let center_separation = (centers[1] - centers[0]).abs();
    let separation_score = (center_separation / 48.0).clamp(0.0, 1.0);
    Some((
        labels,
        occupied_label,
        separation_score,
        center_separation >= 20.0,
    ))
}

fn classify_bright_tape_dark_parts(
    appearance: &[PocketAppearance],
) -> Option<(Vec<StripPocketOccupancy>, f64)> {
    if appearance.len() < 8 {
        return None;
    }
    let luma = appearance
        .iter()
        .map(|item| item.mean_luma)
        .collect::<Vec<_>>();
    let edge = appearance
        .iter()
        .map(|item| item.edge_energy)
        .collect::<Vec<_>>();
    let luma_center = median(&luma);
    let edge_center = median(&edge);
    let minimum_luma = luma.iter().copied().min_by(f64::total_cmp)?;
    let maximum_edge = edge.iter().copied().max_by(f64::total_cmp)?;
    if luma_center < 180.0 || luma_center - minimum_luma < 20.0 || maximum_edge - edge_center < 8.0
    {
        return None;
    }
    let strong = appearance
        .iter()
        .map(|item| item.mean_luma <= luma_center - 20.0)
        .collect::<Vec<_>>();
    let weak = appearance
        .iter()
        .map(|item| item.mean_luma <= luma_center + 2.0 && item.edge_energy >= edge_center + 0.1)
        .collect::<Vec<_>>();
    let mut occupancy = strong
        .iter()
        .map(|strong| {
            if *strong {
                StripPocketOccupancy::Occupied
            } else {
                StripPocketOccupancy::Empty
            }
        })
        .collect::<Vec<_>>();
    let mut start = 0;
    while start < weak.len() {
        if !weak[start] {
            start += 1;
            continue;
        }
        let mut end = start + 1;
        while end < weak.len() && weak[end] {
            end += 1;
        }
        if end - start >= 3 {
            if occupancy[start] != StripPocketOccupancy::Occupied {
                occupancy[start] = StripPocketOccupancy::Uncertain;
            }
            occupancy[start + 1..end].fill(StripPocketOccupancy::Occupied);
        }
        start = end;
    }
    let occupied_count = occupancy
        .iter()
        .filter(|occupancy| **occupancy == StripPocketOccupancy::Occupied)
        .count();
    if occupied_count == 0 || occupied_count * 2 > occupancy.len() {
        return None;
    }
    let confidence = (0.5 * ((luma_center - minimum_luma) / 48.0).clamp(0.0, 1.0)
        + 0.5 * ((maximum_edge - edge_center) / 20.0).clamp(0.0, 1.0))
    .clamp(0.0, 1.0);
    Some((occupancy, confidence))
}

fn classify_homogeneous_populated_lane(
    appearance: &[PocketAppearance],
    seed_fraction: f64,
) -> Option<f64> {
    if appearance.len() < 8 || seed_fraction < 0.5 {
        return None;
    }
    let luma = appearance
        .iter()
        .map(|item| item.mean_luma)
        .collect::<Vec<_>>();
    let edge = appearance
        .iter()
        .map(|item| item.edge_energy)
        .collect::<Vec<_>>();
    let luma_span = luma.iter().copied().max_by(f64::total_cmp)?
        - luma.iter().copied().min_by(f64::total_cmp)?;
    let edge_center = median(&edge);
    let edge_high_excursion = edge.iter().copied().max_by(f64::total_cmp)? - edge_center;
    if luma_span > 32.0 || edge_high_excursion > 4.0 {
        return None;
    }
    Some(
        (0.55 * seed_fraction
            + 0.25 * (1.0 - luma_span / 32.0).clamp(0.0, 1.0)
            + 0.20 * (1.0 - edge_high_excursion / 4.0).clamp(0.0, 1.0))
        .clamp(0.0, 1.0),
    )
}

fn classify_component_change_point(
    appearance: &[PocketAppearance],
    seed_occupied: &[bool],
) -> Option<(Vec<bool>, f64)> {
    if appearance.len() < 7 || appearance.len() != seed_occupied.len() {
        return None;
    }
    let luma = appearance
        .iter()
        .map(|item| item.mean_luma)
        .collect::<Vec<_>>();
    let edge = appearance
        .iter()
        .map(|item| item.edge_energy)
        .collect::<Vec<_>>();
    let luma_center = median(&luma);
    let edge_center = median(&edge);
    let luma_scale = median(
        &luma
            .iter()
            .map(|value| (value - luma_center).abs())
            .collect::<Vec<_>>(),
    )
    .max(1.0);
    let edge_scale = median(
        &edge
            .iter()
            .map(|value| (value - edge_center).abs())
            .collect::<Vec<_>>(),
    )
    .max(1.0);
    let normalized = appearance
        .iter()
        .map(|item| {
            [
                (item.mean_luma - luma_center) / luma_scale,
                (item.edge_energy - edge_center) / edge_scale,
            ]
        })
        .collect::<Vec<_>>();
    let base_cost = appearance_segment_cost(&normalized);
    if !base_cost.is_finite() || base_cost <= 1e-9 {
        return None;
    }
    let (best_cost, split) = (3..normalized.len() - 2)
        .map(|split| {
            (
                appearance_segment_cost(&normalized[..split])
                    + appearance_segment_cost(&normalized[split..]),
                split,
            )
        })
        .min_by(|first, second| first.0.total_cmp(&second.0))?;
    let improvement = (1.0 - best_cost / base_cost).clamp(0.0, 1.0);
    if improvement < 0.35 {
        return None;
    }
    let seed_rate = |values: &[bool]| {
        values.iter().filter(|occupied| **occupied).count() as f64 / values.len().max(1) as f64
    };
    let first_seed_rate = seed_rate(&seed_occupied[..split]);
    let second_seed_rate = seed_rate(&seed_occupied[split..]);
    let first_luma = luma[..split].iter().sum::<f64>() / split as f64;
    let second_luma = luma[split..].iter().sum::<f64>() / (luma.len() - split) as f64;
    if (first_seed_rate - second_seed_rate).abs() < 0.15 && (first_luma - second_luma).abs() < 40.0
    {
        return None;
    }
    let second_is_occupied = if (first_seed_rate - second_seed_rate).abs() >= 0.15 {
        second_seed_rate > first_seed_rate
    } else {
        second_luma > first_luma
    };
    let occupied = (0..appearance.len())
        .map(|index| (index >= split) == second_is_occupied)
        .collect();
    Some((occupied, improvement))
}

fn appearance_segment_cost(values: &[[f64; 2]]) -> f64 {
    if values.is_empty() {
        return f64::INFINITY;
    }
    let centers = [
        median(&values.iter().map(|value| value[0]).collect::<Vec<_>>()),
        median(&values.iter().map(|value| value[1]).collect::<Vec<_>>()),
    ];
    values
        .iter()
        .map(|value| (value[0] - centers[0]).abs() + (value[1] - centers[1]).abs())
        .sum()
}

fn classify_component_edge_energy(
    appearance: &[PocketAppearance],
    seed_occupied: &[bool],
) -> Option<(Vec<usize>, usize, f64)> {
    if appearance.len() < 4 || appearance.len() != seed_occupied.len() {
        return None;
    }
    let values = appearance
        .iter()
        .map(|item| item.edge_energy)
        .collect::<Vec<_>>();
    let mut centers = [
        values.iter().copied().min_by(f64::total_cmp)?,
        values.iter().copied().max_by(f64::total_cmp)?,
    ];
    let mut labels = vec![0_usize; values.len()];
    for _ in 0..12 {
        for (index, value) in values.iter().enumerate() {
            labels[index] = usize::from((value - centers[1]).abs() < (value - centers[0]).abs());
        }
        for (label, center) in centers.iter_mut().enumerate() {
            let selected = values
                .iter()
                .zip(&labels)
                .filter(|(_, assigned)| **assigned == label)
                .map(|(value, _)| *value)
                .collect::<Vec<_>>();
            if selected.len() < 2 {
                return None;
            }
            *center = selected.iter().sum::<f64>() / selected.len() as f64;
        }
    }
    let separation = (centers[1] - centers[0]).abs();
    if separation < 4.0 {
        return None;
    }
    let mut votes = [0_u32; 2];
    for (occupied, label) in seed_occupied.iter().zip(&labels) {
        if *occupied {
            votes[*label] += 1;
        }
    }
    if votes[0] == votes[1] {
        return None;
    }
    let occupied_label = usize::from(votes[1] > votes[0]);
    Some((labels, occupied_label, (separation / 24.0).clamp(0.0, 1.0)))
}

fn apply_appearance_labels(
    observations: &mut [StripFeederPocketObservation],
    labels: &[usize],
    occupied_label: usize,
    grid_confidence: f64,
    appearance_confidence: f64,
) {
    let occupied = labels
        .iter()
        .map(|label| *label == occupied_label)
        .collect::<Vec<_>>();
    apply_occupied_mask(
        observations,
        &occupied,
        grid_confidence,
        appearance_confidence,
    );
}

fn apply_occupied_mask(
    observations: &mut [StripFeederPocketObservation],
    occupied: &[bool],
    grid_confidence: f64,
    appearance_confidence: f64,
) {
    for (observation, occupied) in observations.iter_mut().zip(occupied) {
        observation.occupancy = if *occupied {
            StripPocketOccupancy::Occupied
        } else {
            StripPocketOccupancy::Empty
        };
        observation.confidence =
            (0.55 * grid_confidence + 0.45 * appearance_confidence).clamp(0.0, 1.0);
        if !occupied {
            observation.component_outline_machine_mm.clear();
            observation.component_size_mm = None;
        }
    }
}

fn apply_occupancy_states(
    observations: &mut [StripFeederPocketObservation],
    occupancy: &[StripPocketOccupancy],
    grid_confidence: f64,
    appearance_confidence: f64,
) {
    for (observation, occupancy) in observations.iter_mut().zip(occupancy) {
        observation.occupancy = *occupancy;
        observation.confidence =
            (0.55 * grid_confidence + 0.45 * appearance_confidence).clamp(0.0, 1.0);
        if *occupancy != StripPocketOccupancy::Occupied {
            observation.component_outline_machine_mm.clear();
            observation.component_size_mm = None;
        }
    }
}

fn mark_weak_transition_uncertain(
    observations: &mut [StripFeederPocketObservation],
    confidence: f64,
) {
    if confidence >= 0.6 || observations.len() < 2 {
        return;
    }
    let transitions = observations
        .windows(2)
        .enumerate()
        .filter(|(_, pair)| pair[0].occupancy != pair[1].occupancy)
        .map(|(index, _)| index)
        .collect::<Vec<_>>();
    let [index] = transitions.as_slice() else {
        return;
    };
    if observations[*index].occupancy == StripPocketOccupancy::Empty
        && observations[*index + 1].occupancy == StripPocketOccupancy::Occupied
    {
        let observation = &mut observations[*index + 1];
        observation.occupancy = StripPocketOccupancy::Uncertain;
        observation.component_outline_machine_mm.clear();
        observation.component_size_mm = None;
    }
}

fn component_outline(
    cross_mm: f64,
    along_mm: f64,
    size_mm: [f64; 2],
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
) -> Vec<[f64; 2]> {
    let half_cross = 0.5 * size_mm[0];
    let half_along = 0.5 * size_mm[1];
    [
        [cross_mm - half_cross, along_mm - half_along],
        [cross_mm + half_cross, along_mm - half_along],
        [cross_mm + half_cross, along_mm + half_along],
        [cross_mm - half_cross, along_mm + half_along],
    ]
    .into_iter()
    .map(|point| local_axis_to_machine(point[0], point[1], candidate, long_is_y))
    .collect()
}

fn component_outline_score(
    feature: RectangularFeature,
    expected_cross_mm: f64,
    expected_along_mm: f64,
) -> f64 {
    let center_error =
        (feature.cross_mm - expected_cross_mm).hypot(feature.along_mm - expected_along_mm);
    let center_score = (1.0 - center_error / 0.9).clamp(0.0, 1.0);
    let aspect = feature.cross_size_mm.max(feature.along_size_mm)
        / feature.cross_size_mm.min(feature.along_size_mm).max(1e-9);
    let shape_score = if aspect >= 1.25 {
        feature.rectangularity
    } else {
        0.5 * feature.rectangularity * (1.0 - feature.circularity)
    };
    (0.55 * feature.confidence + 0.30 * center_score + 0.15 * shape_score).clamp(0.0, 1.0)
}

fn median_component_size(observations: &[StripFeederPocketObservation]) -> Option<[f64; 2]> {
    let sizes = observations
        .iter()
        .filter_map(|observation| observation.component_size_mm)
        .collect::<Vec<_>>();
    (!sizes.is_empty()).then(|| {
        let cross = sizes.iter().map(|size| size[0]).collect::<Vec<_>>();
        let along = sizes.iter().map(|size| size[1]).collect::<Vec<_>>();
        [median(&cross), median(&along)]
    })
}

fn periodicity_score(profile: &[f64], lag: usize) -> Option<f64> {
    if lag == 0 || profile.len() <= lag + 2 {
        return None;
    }
    let first = &profile[..profile.len() - lag];
    let second = &profile[lag..];
    let first_mean = first.iter().sum::<f64>() / first.len() as f64;
    let second_mean = second.iter().sum::<f64>() / second.len() as f64;
    let numerator = first
        .iter()
        .zip(second)
        .map(|(a, b)| (a - first_mean) * (b - second_mean))
        .sum::<f64>();
    let first_energy = first
        .iter()
        .map(|value| (value - first_mean).powi(2))
        .sum::<f64>();
    let second_energy = second
        .iter()
        .map(|value| (value - second_mean).powi(2))
        .sum::<f64>();
    let denominator = (first_energy * second_energy).sqrt();
    (denominator > 1e-9).then_some(numerator / denominator)
}

fn strongest_profile_phase(profile: &[f64], lag: usize) -> usize {
    (0..lag)
        .map(|phase| {
            let values = profile.iter().skip(phase).step_by(lag);
            let (sum, count) =
                values.fold((0.0, 0_u32), |(sum, count), value| (sum + value, count + 1));
            (phase, sum / f64::from(count.max(1)))
        })
        .max_by(|first, second| first.1.total_cmp(&second.1))
        .map_or(0, |item| item.0)
}

fn local_axis_to_machine(
    cross_mm: f64,
    along_mm: f64,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
) -> [f64; 2] {
    let (x_mm, y_mm) = if long_is_y {
        (cross_mm, along_mm)
    } else {
        (along_mm, cross_mm)
    };
    let origin = candidate.outline_machine_mm[0];
    let x_axis = scale(
        subtract(candidate.outline_machine_mm[1], origin),
        1.0 / candidate.width_mm,
    );
    let y_axis = scale(
        subtract(candidate.outline_machine_mm[3], origin),
        1.0 / candidate.height_mm,
    );
    add(origin, add(scale(x_axis, x_mm), scale(y_axis, y_mm)))
}

fn crop_pixel_to_machine(
    point: [f64; 2],
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
) -> [f64; 2] {
    local_axis_to_machine(
        point[0] / f64::from(crop.width) * candidate.width_mm,
        point[1] / f64::from(crop.height) * candidate.height_mm,
        candidate,
        true,
    )
}

fn proposal_cross_position(
    proposal: &StripFeederLaneProposal,
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
) -> f64 {
    let Some(center) = proposal.sprocket_centers_machine_mm.first() else {
        return f64::INFINITY;
    };
    let origin = candidate.outline_machine_mm[0];
    let point = subtract(*center, origin);
    let axis_edge = if long_is_y {
        subtract(candidate.outline_machine_mm[1], origin)
    } else {
        subtract(candidate.outline_machine_mm[3], origin)
    };
    let length = if long_is_y {
        candidate.width_mm
    } else {
        candidate.height_mm
    };
    let _ = crop;
    dot(point, scale(axis_edge, 1.0 / length))
}

fn render_preview(
    crop: &GrayFrame,
    candidate: &WorkspaceBoardCandidate,
    long_is_y: bool,
    circles: &[CircleFeature],
    proposals: &[StripFeederLaneProposal],
    suppressed_tracks: &[SuppressedPocketTrack],
) -> RgbFrame {
    let mut pixels = Vec::with_capacity(crop.pixels.len() * 3);
    for luma in &crop.pixels {
        pixels.extend_from_slice(&[*luma, *luma, *luma]);
    }
    let mut preview = RgbFrame {
        width: crop.width,
        height: crop.height,
        pixels,
    };
    for circle in circles {
        draw_circle(&mut preview, circle.center_px, 4.0, [255, 215, 45]);
    }
    for suppressed in suppressed_tracks {
        for circle in &suppressed.track.circles {
            draw_circle(&mut preview, circle.center_px, 5.0, [245, 65, 75]);
        }
        let cross_pixels = if long_is_y {
            f64::from(crop.width)
        } else {
            f64::from(crop.height)
        };
        let cross_length_mm = if long_is_y {
            candidate.width_mm
        } else {
            candidate.height_mm
        };
        let cross_px = suppressed.track.cross_mm / cross_length_mm * cross_pixels;
        let line = if long_is_y {
            ([cross_px, 0.0], [cross_px, f64::from(crop.height - 1)])
        } else {
            ([0.0, cross_px], [f64::from(crop.width - 1), cross_px])
        };
        draw_line(&mut preview, line.0, line.1, [245, 65, 75]);
    }
    for proposal in proposals {
        if let Some(line) = proposal.pick_centerline_machine_mm {
            draw_machine_line(&mut preview, candidate, line[0], line[1], [30, 220, 245]);
        }
        for observation in &proposal.pocket_observations {
            let pixel = machine_to_crop_pixel(observation.center_machine_mm, candidate, crop);
            match observation.occupancy {
                StripPocketOccupancy::Occupied => {
                    if observation.component_outline_machine_mm.len() >= 3 {
                        let points = observation
                            .component_outline_machine_mm
                            .iter()
                            .map(|point| machine_to_crop_pixel(*point, candidate, crop))
                            .collect::<Vec<_>>();
                        draw_closed_polyline(&mut preview, &points, [50, 235, 120]);
                    } else {
                        draw_circle(&mut preview, pixel, 3.0, [50, 235, 120]);
                    }
                }
                StripPocketOccupancy::Empty => {
                    draw_circle(&mut preview, pixel, 3.0, [245, 80, 70]);
                    draw_cross(&mut preview, pixel, 2.0, [245, 80, 70]);
                }
                StripPocketOccupancy::Uncertain => {
                    draw_circle(&mut preview, pixel, 3.0, [255, 190, 40]);
                }
            }
        }
    }
    preview
}

fn draw_machine_line(
    frame: &mut RgbFrame,
    candidate: &WorkspaceBoardCandidate,
    start: [f64; 2],
    end: [f64; 2],
    color: [u8; 3],
) {
    let crop = GrayFrame {
        width: frame.width,
        height: frame.height,
        pixels: Vec::new(),
    };
    draw_line(
        frame,
        machine_to_crop_pixel(start, candidate, &crop),
        machine_to_crop_pixel(end, candidate, &crop),
        color,
    );
}

fn machine_to_crop_pixel(
    point: [f64; 2],
    candidate: &WorkspaceBoardCandidate,
    crop: &GrayFrame,
) -> [f64; 2] {
    let origin = candidate.outline_machine_mm[0];
    let relative = subtract(point, origin);
    let x_edge = subtract(candidate.outline_machine_mm[1], origin);
    let y_edge = subtract(candidate.outline_machine_mm[3], origin);
    let x_unit = scale(x_edge, 1.0 / candidate.width_mm);
    let y_unit = scale(y_edge, 1.0 / candidate.height_mm);
    [
        dot(relative, x_unit) / candidate.width_mm * f64::from(crop.width),
        dot(relative, y_unit) / candidate.height_mm * f64::from(crop.height),
    ]
}

fn draw_circle(frame: &mut RgbFrame, center: [f64; 2], radius: f64, color: [u8; 3]) {
    let steps = (2.0 * PI * radius).ceil().max(16.0) as u32;
    for step in 0..steps {
        let angle = 2.0 * PI * f64::from(step) / f64::from(steps);
        set_rgb(
            frame,
            (center[0] + radius * angle.cos()).round() as i32,
            (center[1] + radius * angle.sin()).round() as i32,
            color,
        );
    }
}

fn draw_cross(frame: &mut RgbFrame, center: [f64; 2], radius: f64, color: [u8; 3]) {
    draw_line(
        frame,
        [center[0] - radius, center[1]],
        [center[0] + radius, center[1]],
        color,
    );
    draw_line(
        frame,
        [center[0], center[1] - radius],
        [center[0], center[1] + radius],
        color,
    );
}

fn draw_line(frame: &mut RgbFrame, start: [f64; 2], end: [f64; 2], color: [u8; 3]) {
    let steps = (end[0] - start[0])
        .abs()
        .max((end[1] - start[1]).abs())
        .ceil()
        .max(1.0) as u32;
    for step in 0..=steps {
        let fraction = f64::from(step) / f64::from(steps);
        set_rgb(
            frame,
            (start[0] + fraction * (end[0] - start[0])).round() as i32,
            (start[1] + fraction * (end[1] - start[1])).round() as i32,
            color,
        );
    }
}

fn draw_closed_polyline(frame: &mut RgbFrame, points: &[[f64; 2]], color: [u8; 3]) {
    if points.len() < 2 {
        return;
    }
    for index in 0..points.len() {
        draw_line(
            frame,
            points[index],
            points[(index + 1) % points.len()],
            color,
        );
    }
}

fn set_rgb(frame: &mut RgbFrame, x: i32, y: i32, color: [u8; 3]) {
    if x < 0 || y < 0 || x >= frame.width as i32 || y >= frame.height as i32 {
        return;
    }
    let index = (y as u32 * frame.width + x as u32) as usize * 3;
    frame.pixels[index..index + 3].copy_from_slice(&color);
}

fn lattice_residual_mm(value: f64, phase: f64) -> f64 {
    ((value - phase + 0.5 * STRIP_FEEDER_SPROCKET_PITCH_MM)
        .rem_euclid(STRIP_FEEDER_SPROCKET_PITCH_MM)
        - 0.5 * STRIP_FEEDER_SPROCKET_PITCH_MM)
        .abs()
}

fn percentile(sorted: &[f64], quantile: f64) -> f64 {
    if sorted.is_empty() {
        return f64::INFINITY;
    }
    let index = ((sorted.len() - 1) as f64 * quantile).ceil() as usize;
    sorted[index]
}

fn median(values: &[f64]) -> f64 {
    if values.is_empty() {
        return f64::INFINITY;
    }
    let mut values = values.to_vec();
    values.sort_by(f64::total_cmp);
    if values.len() % 2 == 0 {
        0.5 * (values[values.len() / 2 - 1] + values[values.len() / 2])
    } else {
        values[values.len() / 2]
    }
}

fn add(first: [f64; 2], second: [f64; 2]) -> [f64; 2] {
    [first[0] + second[0], first[1] + second[1]]
}

fn subtract(first: [f64; 2], second: [f64; 2]) -> [f64; 2] {
    [first[0] - second[0], first[1] - second[1]]
}

fn scale(value: [f64; 2], factor: f64) -> [f64; 2] {
    [value[0] * factor, value[1] * factor]
}

fn dot(first: [f64; 2], second: [f64; 2]) -> f64 {
    first[0] * second[0] + first[1] * second[1]
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;
    use uuid::Uuid;
    use velocitypnp_domain::{
        CameraRole, InstalledPlateRole, PlanarCameraModelKind, StripFeederRegistrationStatus,
        WorkspaceObjectClass, WorkspaceObjectClassification, WorkspaceScanRegion,
    };

    fn candidate() -> WorkspaceBoardCandidate {
        WorkspaceBoardCandidate {
            candidate_index: 2,
            region: WorkspaceScanRegion {
                x_min_mm: 100.0,
                x_max_mm: 124.0,
                y_min_mm: 40.0,
                y_max_mm: 104.0,
            },
            outline_machine_mm: vec![[100.0, 104.0], [124.0, 104.0], [124.0, 40.0], [100.0, 40.0]],
            plate_id: Some("staging".into()),
            plate_role: Some(InstalledPlateRole::Staging),
            center_x_mm: 112.0,
            center_y_mm: 72.0,
            width_mm: 24.0,
            height_mm: 64.0,
            rotation_deg: 0.0,
            evidence_cell_count: 64,
            measured_cell_fraction: 1.0,
            mean_surface_fraction: 0.5,
            mean_height_confidence: 0.9,
            fill_fraction: 0.8,
            texture_energy_luma: 30.0,
            board_likelihood: 0.2,
            classification: Some(WorkspaceObjectClassification {
                class: WorkspaceObjectClass::Feeder,
                confidence: 0.95,
                classifier_id: "test".into(),
                review_required: false,
                evidence: Vec::new(),
            }),
            known_fixture_id: None,
            evidence_artifact_name: Some("object-evidence-002.png".into()),
            feeder_lane_slots: Vec::new(),
            design_matches: Vec::new(),
        }
    }

    #[test]
    fn workspace_source_crop_keeps_the_selected_pocket_centered() {
        let width = 500_u32;
        let height = 500_u32;
        let pixels = (0..height)
            .flat_map(|row| (0..width).map(move |column| ((row + column) % 256) as u8))
            .collect::<Vec<_>>();
        let mosaic = GrayFrame {
            width,
            height,
            pixels,
        };
        let raster = WorkspaceRasterTransform {
            top_left_x_mm: 0.0,
            top_left_y_mm: 25.0,
            x_mm_per_pixel: 0.05,
            y_mm_per_pixel: -0.05,
        };

        let crop =
            crop_workspace_source_component(&mosaic, raster, [12.5, 12.5], 90.0, 8.0, 4.0).unwrap();

        assert_eq!(crop.width, 280);
        assert_eq!(crop.height, 240);
        let center = crop.pixels[((crop.height / 2) * crop.width + crop.width / 2) as usize];
        assert!(center.abs_diff(mosaic.pixels[(250 * width + 250) as usize]) <= 2);
    }

    fn synthetic_tape_bank() -> GrayFrame {
        let width = 240_u32;
        let height = 640_u32;
        let mut pixels = vec![105_u8; (width * height) as usize];
        let px_per_mm = 10.0;
        for sprocket_x_mm in [3.0_f64, 11.0, 19.0] {
            let sprocket_x = (sprocket_x_mm * px_per_mm).round() as i32;
            let pocket_x = ((sprocket_x_mm + 3.5) * px_per_mm).round() as i32;
            for y_mm in (2..63).step_by(4) {
                paint_disc(
                    &mut pixels,
                    width,
                    height,
                    sprocket_x,
                    (f64::from(y_mm) * px_per_mm).round() as i32,
                    7,
                    235,
                );
            }
            for y_mm in (1..64).step_by(4) {
                let pocket_y = (f64::from(y_mm) * px_per_mm).round() as i32;
                paint_disc(&mut pixels, width, height, pocket_x, pocket_y, 6, 35);
            }
        }
        GrayFrame {
            width,
            height,
            pixels,
        }
    }

    fn paint_disc(
        pixels: &mut [u8],
        width: u32,
        height: u32,
        center_x: i32,
        center_y: i32,
        radius: i32,
        value: u8,
    ) {
        for y in center_y - radius..=center_y + radius {
            for x in center_x - radius..=center_x + radius {
                if x >= 0
                    && y >= 0
                    && x < width as i32
                    && y < height as i32
                    && (x - center_x).pow(2) + (y - center_y).pow(2) <= radius.pow(2)
                {
                    pixels[(y as u32 * width + x as u32) as usize] = value;
                }
            }
        }
    }

    fn paint_rectangle(frame: &mut GrayFrame, center: [i32; 2], half_size: [i32; 2], value: u8) {
        for y in center[1] - half_size[1]..=center[1] + half_size[1] {
            for x in center[0] - half_size[0]..=center[0] + half_size[0] {
                if x >= 0 && y >= 0 && x < frame.width as i32 && y < frame.height as i32 {
                    frame.pixels[(y as u32 * frame.width + x as u32) as usize] = value;
                }
            }
        }
    }

    #[test]
    fn finds_plate_aligned_tape_lanes_and_pitch() {
        let result = analyze_strip_feeder_candidate(&synthetic_tape_bank(), &candidate()).unwrap();

        assert_eq!(result.proposals.len(), 3);
        for proposal in &result.proposals {
            assert_eq!(proposal.tape_width_mm, Some(8.0));
            assert!(proposal.sprocket_center_count >= 10);
            assert!(proposal.sprocket_pitch_residual_p95_mm <= 0.2);
            assert!((proposal.tape_rotation_deg + 90.0).abs() < 1e-9);
            assert!(proposal.pick_centerline_machine_mm.is_some());
        }
        assert_eq!(result.rejected_lattices.len(), 3);
        assert!(result.rejected_lattices.iter().all(|rejected| {
            (rejected.offset_mm - EIGHT_MM_POCKET_CENTER_OFFSET_MM).abs() <= 0.1
                && rejected.reason.contains("pocket centerline")
        }));
        assert!(result
            .warnings
            .iter()
            .any(|warning| warning.contains("3 4 mm lattices were rejected")));
        assert_eq!(result.preview.width, 240);
        assert_eq!(result.preview.height, 640);
    }

    #[test]
    fn dense_rectangular_features_resolve_two_mm_pitch() {
        let features = (1..64)
            .step_by(2)
            .map(|along_mm| RectangularFeature {
                cross_mm: 6.5,
                along_mm: f64::from(along_mm),
                cross_size_mm: 1.0,
                along_size_mm: 0.6,
                rectangularity: 0.9,
                circularity: 0.2,
                confidence: 0.9,
            })
            .collect::<Vec<_>>();

        let result = estimate_component_grid(&features, 6.5, Some([1.0, 63.0]), 64.0);

        assert_eq!(result.0, Some(2.0));
        assert!((result.1.unwrap() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn recognizes_component_outlines_and_empty_feed_positions() {
        let width = 240_u32;
        let height = 640_u32;
        let mut frame = GrayFrame {
            width,
            height,
            pixels: vec![105; (width * height) as usize],
        };
        for y_mm in (2..63).step_by(4) {
            paint_disc(&mut frame.pixels, width, height, 30, y_mm * 10, 7, 235);
        }
        for y_mm in (2..63).step_by(8) {
            paint_rectangle(&mut frame, [65, y_mm * 10], [10, 5], 25);
        }

        let result = analyze_strip_feeder_candidate(&frame, &candidate()).unwrap();

        assert_eq!(result.proposals.len(), 1);
        let proposal = &result.proposals[0];
        assert_eq!(proposal.tape_width_mm, Some(8.0));
        assert_eq!(proposal.component_pitch_mm, Some(4.0));
        assert!(proposal.present_count >= 6);
        assert!(proposal.absent_count >= 6);
        assert!(proposal.component_size_mm.is_some());
        assert!(proposal.pocket_observations.iter().any(|observation| {
            observation.occupancy == StripPocketOccupancy::Occupied
                && observation.component_outline_machine_mm.len() == 4
        }));
    }

    #[test]
    fn rejects_isolated_round_features() {
        let mut frame = GrayFrame {
            width: 240,
            height: 640,
            pixels: vec![100; 240 * 640],
        };
        paint_disc(&mut frame.pixels, 240, 640, 50, 50, 7, 240);
        paint_disc(&mut frame.pixels, 240, 640, 180, 350, 7, 240);

        let result = analyze_strip_feeder_candidate(&frame, &candidate()).unwrap();

        assert!(result.proposals.is_empty());
        assert!(!result.warnings.is_empty());
    }

    #[test]
    fn rejects_a_three_hole_plate_lattice() {
        let mut frame = GrayFrame {
            width: 240,
            height: 640,
            pixels: vec![100; 240 * 640],
        };
        for center_y in [50, 90, 130] {
            paint_disc(&mut frame.pixels, 240, 640, 50, center_y, 7, 240);
        }

        let result = analyze_strip_feeder_candidate(&frame, &candidate()).unwrap();

        assert!(result.proposals.is_empty());
    }

    #[test]
    fn tape_width_is_not_inferred_from_neighboring_feeder_spacing() {
        assert_eq!(
            infer_tape_geometry(&[], 2.0, Some([0.0, 40.0]), 24.0, 64.0, Some(1.0)),
            (None, None, None, None, 0.0),
        );
    }

    #[test]
    fn bright_tape_classifier_keeps_a_weak_run_boundary_uncertain() {
        let mut appearance = vec![
            PocketAppearance {
                mean_luma: 225.0,
                edge_energy: 10.0,
            };
            18
        ];
        appearance[4] = PocketAppearance {
            mean_luma: 180.0,
            edge_energy: 22.0,
        };
        for item in &mut appearance[13..] {
            *item = PocketAppearance {
                mean_luma: 220.0,
                edge_energy: 11.0,
            };
        }

        let (occupancy, _) = classify_bright_tape_dark_parts(&appearance).unwrap();

        assert_eq!(occupancy[4], StripPocketOccupancy::Occupied);
        assert_eq!(occupancy[13], StripPocketOccupancy::Uncertain);
        assert!(occupancy[14..]
            .iter()
            .all(|item| *item == StripPocketOccupancy::Occupied));
        assert!(occupancy[..4]
            .iter()
            .all(|item| *item == StripPocketOccupancy::Empty));
    }

    #[test]
    fn homogeneous_repeated_parts_support_a_populated_lane() {
        let appearance = (0..12)
            .map(|index| PocketAppearance {
                mean_luma: 200.0 + f64::from(index) * 0.5,
                edge_energy: 20.0 + f64::from(index % 3) * 0.5,
            })
            .collect::<Vec<_>>();

        assert!(classify_homogeneous_populated_lane(&appearance, 0.75).is_some());
        assert!(classify_homogeneous_populated_lane(&appearance, 0.25).is_none());
    }

    #[test]
    fn physical_feeder_ground_truth_has_the_confirmed_totals() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../tests/fixtures/lumenpnp-v3.0.2-strip-feeder-bank-ground-truth.json"
        ))
        .unwrap();
        let lanes = fixture["lanes"].as_array().unwrap();
        let occupancy = lanes
            .iter()
            .map(|lane| lane["occupancy"].as_str().unwrap())
            .collect::<Vec<_>>();

        assert_eq!(lanes.len(), 8);
        assert_eq!(
            occupancy.iter().map(|value| value.len()).sum::<usize>(),
            170
        );
        assert_eq!(
            occupancy
                .iter()
                .map(|value| value.matches('P').count())
                .sum::<usize>(),
            118
        );
        assert_eq!(
            occupancy
                .iter()
                .map(|value| value.matches('E').count())
                .sum::<usize>(),
            52
        );
    }

    #[test]
    fn normalized_patch_correlation_rejects_an_unrelated_edge_patch() {
        let reference = [10.0, 20.0, 30.0, 40.0];
        let matching = [20.0, 40.0, 60.0, 80.0];
        let unrelated = [10.0, 40.0, 20.0, 30.0];

        assert!((normalized_patch_correlation(&reference, &matching).unwrap() - 1.0).abs() < 1e-9);
        assert!(normalized_patch_correlation(&reference, &unrelated).unwrap() < 0.5);
    }

    #[test]
    fn return_similarity_rejects_an_empty_textured_pocket() {
        let width = 160_u32;
        let height = 120_u32;
        let mut empty = GrayFrame {
            width,
            height,
            pixels: (0..width * height)
                .map(|index| 105 + ((index * 17 + index / width * 11) % 31) as u8)
                .collect(),
        };
        for y in 40..80 {
            for x in 38..122 {
                if x == 38 || x == 121 || y == 40 || y == 79 {
                    empty.pixels[(y * width + x) as usize] = 190;
                }
            }
        }
        let mut occupied = empty.clone();
        for y in 49..71 {
            for x in 47..113 {
                occupied.pixels[(y * width + x) as usize] =
                    if !(53..107).contains(&x) { 230 } else { 165 };
            }
        }

        let matching = centered_pocket_patch_similarity(
            &occupied,
            [80.0, 60.0],
            &occupied,
            [80.0, 60.0],
            [1.7, 0.9],
        )
        .unwrap();
        let missing = centered_pocket_patch_similarity(
            &occupied,
            [80.0, 60.0],
            &empty,
            [80.0, 60.0],
            [1.7, 0.9],
        )
        .unwrap();

        assert!(matching.similarity > 0.99);
        assert!(missing.similarity < MINIMUM_POCKET_RETURN_APPEARANCE_SIMILARITY);
    }

    #[test]
    fn return_similarity_aligns_a_small_package_pose_change() {
        let width = 160_u32;
        let height = 120_u32;
        let center = [80.0, 60.0];
        let before = GrayFrame {
            width,
            height,
            pixels: (0..width * height)
                .map(|index| {
                    let x = f64::from(index % width) - center[0];
                    let y = f64::from(index / width) - center[1];
                    if x.abs() <= 24.0 && y.abs() <= 11.0 {
                        if x < -8.0 && y < -3.0 {
                            220
                        } else {
                            75
                        }
                    } else {
                        145
                    }
                })
                .collect(),
        };
        let angle = 8.0_f64.to_radians();
        let (sin, cos) = angle.sin_cos();
        let mut after = GrayFrame {
            width,
            height,
            pixels: vec![145; (width * height) as usize],
        };
        for y in 0..height {
            for x in 0..width {
                let output_x = f64::from(x) - center[0] - 3.0;
                let output_y = f64::from(y) - center[1] + 2.0;
                let source_x = center[0] + cos * output_x + sin * output_y;
                let source_y = center[1] - sin * output_x + cos * output_y;
                after.pixels[(y * width + x) as usize] = bilinear_luma(&before, source_x, source_y);
            }
        }

        let measurement =
            centered_pocket_patch_similarity(&before, center, &after, center, [1.2, 0.7]).unwrap();

        assert!(measurement.similarity > 0.90, "{measurement:?}");
        assert!(measurement.rotation_deg.abs() >= 6.0, "{measurement:?}");
    }

    #[test]
    fn pocket_center_measurement_does_not_select_the_package_edge() {
        let width = 350_u32;
        let height = 500_u32;
        let mut pixels = vec![235_u8; (width * height) as usize];
        for y in 250..290 {
            for x in 135..215 {
                pixels[(y * width + x) as usize] = 105;
            }
        }
        let crop = GrayFrame {
            width,
            height,
            pixels,
        };

        let measured = measure_pocket_image_center(&crop, 7.0, 10.0, 14.0, 20.0, 4.0, 8.0).unwrap();

        assert!((measured.center_mm[0] - 7.0).abs() < 0.03);
        assert!((measured.center_mm[1] - 10.8).abs() < 0.03);
        assert!(measured.confidence > 0.8);
    }

    #[test]
    fn dark_package_core_centers_an_outline_inside_a_larger_pocket() {
        let width = 350_u32;
        let height = 500_u32;
        let mut crop = GrayFrame {
            width,
            height,
            pixels: vec![235_u8; (width * height) as usize],
        };
        paint_rectangle(&mut crop, [175, 250], [28, 17], 190);
        paint_rectangle(&mut crop, [184, 263], [16, 7], 55);
        paint_rectangle(&mut crop, [169, 263], [3, 7], 220);
        paint_rectangle(&mut crop, [199, 263], [3, 7], 220);
        let pocket_center =
            measure_pocket_image_center(&crop, 7.0, 10.0, 14.0, 20.0, 4.0, 8.0).unwrap();

        let package = measure_dark_source_package(&crop, pocket_center, 14.0, 20.0, [1.0, 1.45])
            .unwrap()
            .unwrap();

        assert!((package.center_mm[0] - 7.36).abs() < 0.12, "{package:?}");
        assert!((package.center_mm[1] - 10.52).abs() < 0.12, "{package:?}");
        assert!((package.size_mm[0] - 0.84).abs() < 0.05, "{package:?}");
        assert!((package.size_mm[1] - 0.56).abs() < 0.05, "{package:?}");
    }

    #[test]
    fn live_holdout_rectifies_the_camera_and_selects_an_occupied_pocket() {
        let width = 500_u32;
        let height = 500_u32;
        let mut pixels = vec![105_u8; (width * height) as usize];
        // A live crop can contain only four sprockets. Offline discovery
        // still requires five, but this bounded local holdout requires three.
        for y_mm in (-6_i32..=6).step_by(4) {
            paint_disc(&mut pixels, width, height, 163, 250 + y_mm * 25, 18, 235);
        }
        for y_mm in (-8_i32..=8).step_by(2) {
            let center_x = if y_mm == 0 { 285 } else { 250 };
            paint_disc(
                &mut pixels,
                width,
                height,
                center_x,
                250 + y_mm * 25,
                11,
                if y_mm == 2 {
                    0
                } else if y_mm == 0 {
                    25
                } else {
                    165
                },
            );
        }
        let frame = GrayFrame {
            width,
            height,
            pixels,
        };
        let model = PlanarCameraModel {
            role: CameraRole::TopCamera,
            center_x_px: 250.0,
            center_y_px: 250.0,
            normalization_x_px: 250.0,
            normalization_y_px: 250.0,
            correction_x_coefficients_mm: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            correction_y_coefficients_mm: [0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            supported_min_x_px: 0.0,
            supported_max_x_px: 499.0,
            supported_min_y_px: 0.0,
            supported_max_y_px: 499.0,
            kind: PlanarCameraModelKind::Affine,
        };
        let now = Utc::now();
        let registration = StripFeederRegistration {
            id: Uuid::new_v4(),
            profile_id: "test".into(),
            calibration_id: Uuid::new_v4(),
            source_scan_id: Uuid::new_v4(),
            source_candidate_index: 0,
            source_proposal_index: 0,
            created_at: now,
            updated_at: now,
            status: StripFeederRegistrationStatus::VisionDraft,
            name: "test tape".into(),
            part_key: None,
            part_rotation_deg: None,
            tape_width_mm: 8.0,
            tape_pitch_mm: 2.0,
            tape_rotation_deg: -90.0,
            pick_centerline_machine_mm: [[112.0, 82.0], [112.0, 62.0]],
            pocket_centers_machine_mm: (0..11)
                .map(|index| [112.0, 82.0 - f64::from(index) * 2.0])
                .collect(),
            pockets: Vec::new(),
            selected_pocket_machine_mm: None,
            selected_pocket_index: None,
            pickup_z_mm: None,
            surveyed_part_size_mm: None,
            measured_part_size_mm: Some([0.84, 1.64]),
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
        let pose = Pose {
            x_mm: 112.0,
            y_mm: 72.0,
            z_mm: 20.0,
            rotation_deg: 0.0,
        };
        let result = analyze_live_strip_feeder_holdout(
            &[frame.clone(), frame.clone(), frame.clone()],
            &[pose, pose, pose],
            &model,
            &registration,
            [112.0, 72.0],
            "holdout.png",
        )
        .unwrap();

        assert!(result.report.passed, "{:?}", result.report.findings);
        assert_eq!(result.report.sprocket_center_count, 4);
        assert_eq!(result.report.occupancy, StripPocketOccupancy::Occupied);
        assert_eq!(result.report.predicted_pocket_machine_mm, [112.0, 72.0]);
        // A loose component can move inside its pocket. The sprocket lattice,
        // not the component centroid, defines the pocket correction.
        assert!(result.report.correction_magnitude_mm < 0.15);
        assert_eq!(result.preview.width, 350);
        assert_eq!(result.preview.height, 500);

        let draft =
            draft_strip_feeder_source_component(&frame, pose, [112.0, 72.0], &model, &registration)
                .unwrap();
        assert_eq!(draft.crop.width, 350);
        assert_eq!(draft.crop.height, 300);
        assert_eq!(draft.report.occupancy, StripPocketOccupancy::Occupied);
        assert!(draft.report.outline_px.is_some());
        assert!(!draft.report.review_required);
        let mapped_center = source_component_crop_pixel_to_machine(
            &draft.crop,
            [175.0, 150.0],
            [112.0, 72.0],
            &registration,
        )
        .unwrap();
        assert!((mapped_center[0] - 112.0).abs() < 1e-9);
        assert!((mapped_center[1] - 72.0).abs() < 1e-9);
        let mapped_offset = source_component_crop_pixel_to_machine(
            &draft.crop,
            [200.0, 175.0],
            [112.0, 72.0],
            &registration,
        )
        .unwrap();
        assert!((mapped_offset[0] - 113.0).abs() < 1e-9);
        assert!((mapped_offset[1] - 71.0).abs() < 1e-9);

        let mut returned_frame = frame.clone();
        paint_disc(&mut returned_frame.pixels, width, height, 285, 250, 11, 105);
        paint_disc(&mut returned_frame.pixels, width, height, 250, 250, 11, 25);
        let returned = inspect_strip_feeder_pocket_return(
            &returned_frame,
            pose,
            &returned_frame,
            pose,
            &model,
            &registration,
            [112.0, 72.0],
            "return.png",
        )
        .unwrap();
        assert!(returned.report.passed, "{:?}", returned.report.findings);
        assert_eq!(
            returned.report.after_occupancy,
            StripPocketOccupancy::Occupied
        );
        assert_eq!(returned.preview.width, 700);

        let reference_id = Uuid::new_v4();
        let flat_pose = inspect_strip_feeder_source_pose(
            &returned_frame,
            pose,
            [112.0, 72.0],
            reference_id,
            &returned_frame,
            pose,
            [112.0, 72.0],
            &model,
            &registration,
            false,
            "source-pose.png",
        )
        .unwrap();
        assert!(flat_pose.report.passed, "{:?}", flat_pose.report.findings);
        assert_eq!(flat_pose.report.reference_qualification_id, reference_id);

        let mut marked_face = frame.clone();
        paint_rectangle(&mut marked_face, [250, 250], [20, 10], 25);
        paint_rectangle(&mut marked_face, [250, 250], [7, 4], 225);
        let mut unmarked_face = frame.clone();
        paint_rectangle(&mut unmarked_face, [250, 250], [20, 10], 225);
        let flipped_flat_pose = inspect_strip_feeder_source_pose(
            &marked_face,
            pose,
            [112.0, 72.0],
            reference_id,
            &unmarked_face,
            pose,
            [112.0, 72.0],
            &model,
            &registration,
            true,
            "source-pose.png",
        )
        .unwrap();
        assert!(
            flipped_flat_pose.report.passed,
            "{:?}",
            flipped_flat_pose.report
        );
        assert!(flipped_flat_pose.report.flat_geometry_passed);
        assert!(flipped_flat_pose
            .report
            .appearance_similarity
            .is_some_and(|similarity| similarity < MINIMUM_SOURCE_POSE_APPEARANCE_SIMILARITY));

        let mut edge_pose_frame = returned_frame.clone();
        for y in 230..=270 {
            for x in 230..=270 {
                edge_pose_frame.pixels[(y * width + x) as usize] = 105;
            }
        }
        paint_rectangle(&mut edge_pose_frame, [250, 250], [4, 14], 25);
        paint_rectangle(&mut edge_pose_frame, [250, 250], [1, 11], 225);
        let edge_pose = inspect_strip_feeder_source_pose(
            &returned_frame,
            pose,
            [112.0, 72.0],
            reference_id,
            &edge_pose_frame,
            pose,
            [112.0, 72.0],
            &model,
            &registration,
            true,
            "source-pose.png",
        )
        .unwrap();
        assert!(!edge_pose.report.passed, "{:?}", edge_pose.report);

        let empty = GrayFrame {
            width,
            height,
            pixels: vec![105; (width * height) as usize],
        };
        let missing = inspect_strip_feeder_pocket_return(
            &frame,
            pose,
            &empty,
            pose,
            &model,
            &registration,
            [112.0, 72.0],
            "return.png",
        )
        .unwrap();
        assert!(!missing.report.passed);
        assert_eq!(missing.report.after_occupancy, StripPocketOccupancy::Empty);
    }

    #[test]
    fn live_dark_disc_path_keeps_three_holes_when_one_touches_a_crop_seam() {
        let width = 350_u32;
        let height = 500_u32;
        let mut pixels = vec![220_u8; (width * height) as usize];
        for y in 0..75 {
            for x in 0..width {
                pixels[(y * width + x) as usize] = 0;
            }
        }
        for center_y in [75, 175, 275, 375] {
            paint_disc(&mut pixels, width, height, 90, center_y, 19, 30);
        }
        let crop = GrayFrame {
            width,
            height,
            pixels,
        };

        let circles = detect_live_sprocket_circles(&crop, 14.0, 20.0).unwrap();
        let tracks = build_sprocket_tracks(
            &circles,
            &crop,
            14.0,
            20.0,
            true,
            MINIMUM_LIVE_HOLDOUT_SPROCKETS,
        );

        assert!(circles.len() >= 3);
        assert_eq!(tracks.len(), 1);
        assert!(tracks[0].circles.len() >= 3);
    }
}
