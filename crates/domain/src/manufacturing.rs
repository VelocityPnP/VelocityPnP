use crate::{OperationStatus, Pose, Residuals, Transform2d, VacuumProbeSample};
use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoardSide {
    Top,
    Bottom,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Placement {
    pub reference: String,
    pub value: String,
    pub package: String,
    pub side: BoardSide,
    pub pose: Pose,
    pub enabled: bool,
    pub fiducial: bool,
    pub properties: std::collections::BTreeMap<String, String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BoardDesign {
    pub id: Uuid,
    pub name: String,
    pub source_format: String,
    pub source_version: Option<u32>,
    pub width_mm: Option<f64>,
    pub height_mm: Option<f64>,
    #[serde(default)]
    pub outline_bounds: Option<BoardOutlineBounds>,
    pub placements: Vec<Placement>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BoardOutlineBounds {
    pub min_x_mm: f64,
    pub max_x_mm: f64,
    pub min_y_mm: f64,
    pub max_y_mm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardInstance {
    pub id: Uuid,
    pub board_id: Uuid,
    pub label: String,
    pub side: BoardSide,
    pub design_to_machine: Option<Transform2d>,
    pub surface_z_mm: Option<f64>,
    /// Added height above the measured board datum, such as adhesive or paste.
    #[serde(default)]
    pub placement_surface_offset_mm: f64,
}

/// One measured design fiducial used to register a physical board.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardRegistrationFiducialObservation {
    pub reference: String,
    pub design_x_mm: f64,
    pub design_y_mm: f64,
    pub predicted_x_mm: f64,
    pub predicted_y_mm: f64,
    pub observed_x_px: f64,
    pub observed_y_px: f64,
    pub measured_x_mm: f64,
    pub measured_y_mm: f64,
    pub diameter_px: f64,
    pub confidence: f64,
    pub artifact_name: String,
}

/// Evidence retained for one square-board orientation hypothesis.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardRegistrationHypothesis {
    pub orientation_index: u32,
    pub seed_rotation_deg: f64,
    #[serde(default)]
    pub observations: Vec<BoardRegistrationFiducialObservation>,
    #[serde(default)]
    pub rejection_reason: Option<String>,
}

/// Persistent state and evidence for one automatic board-registration run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardRegistrationRun {
    pub id: Uuid,
    pub operation_id: Uuid,
    pub profile_id: String,
    pub calibration_id: Uuid,
    pub source_scan_id: Uuid,
    pub source_candidate_index: u32,
    pub board_id: Uuid,
    pub label: String,
    pub side: BoardSide,
    pub status: OperationStatus,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub sample_count: u32,
    pub seed_center_x_mm: f64,
    pub seed_center_y_mm: f64,
    pub seed_rotation_deg: f64,
    #[serde(default)]
    pub hypotheses: Vec<BoardRegistrationHypothesis>,
    #[serde(default)]
    pub selected_rotation_deg: Option<f64>,
    #[serde(default)]
    pub fit_residuals: Option<Residuals>,
    #[serde(default)]
    pub design_to_machine: Option<Transform2d>,
    #[serde(default)]
    pub board_instance: Option<BoardInstance>,
    #[serde(default)]
    pub latest_surface_measurement_id: Option<Uuid>,
    #[serde(default)]
    pub passed: bool,
    #[serde(default)]
    pub message: Option<String>,
}

/// One detected board fiducial from one safe-Z camera view.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardSurfaceFiducialView {
    pub reference: String,
    pub view_index: u32,
    pub camera_pose: Pose,
    pub observed_x_px: f64,
    pub observed_y_px: f64,
    pub diameter_px: f64,
    pub confidence: f64,
    pub artifact_name: String,
}

/// A board-height estimate from two views of the same known fiducial.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardSurfaceParallaxObservation {
    pub reference: String,
    pub fixed_view_index: u32,
    pub moving_view_index: u32,
    pub surface_fraction: f64,
    pub residual_mm: f64,
    pub parallax_signal_mm: f64,
}

/// Non-contact board-surface result in one nozzle's working-Z coordinates.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardSurfaceMeasurementReport {
    pub algorithm_version: String,
    pub nozzle_id: String,
    pub reference_contact_nozzle_z_mm: f64,
    pub installed_plate_depth_below_working_plane_mm: f64,
    pub median_surface_fraction: f64,
    pub surface_fraction_p95_deviation: f64,
    pub surface_z_mm: f64,
    pub uncertainty_mm: f64,
    pub fit_residuals: Residuals,
    pub views: Vec<BoardSurfaceFiducialView>,
    pub observations: Vec<BoardSurfaceParallaxObservation>,
    pub passed: bool,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Persistent state for one automatic, non-contact board-surface run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BoardSurfaceMeasurementRun {
    pub id: Uuid,
    pub operation_id: Uuid,
    pub registration_id: Uuid,
    pub profile_id: String,
    pub calibration_id: Uuid,
    pub nozzle_id: String,
    pub status: OperationStatus,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub sample_count: u32,
    #[serde(default)]
    pub report: Option<BoardSurfaceMeasurementReport>,
    #[serde(default)]
    pub message: Option<String>,
}

/// Builds the top-side design transform used by previews, registration, and
/// job planning. KiCad Y increases toward the bottom of the board view, so
/// this transform includes a reflection about the Edge.Cuts center.
pub fn top_side_board_transform(
    bounds: BoardOutlineBounds,
    center_machine_mm: [f64; 2],
    rotation_deg: f64,
) -> Transform2d {
    let center_x = 0.5 * (bounds.min_x_mm + bounds.max_x_mm);
    let center_y = 0.5 * (bounds.min_y_mm + bounds.max_y_mm);
    let angle = rotation_deg.to_radians();
    let (sin, cos) = angle.sin_cos();
    Transform2d {
        matrix: [
            cos,
            sin,
            center_machine_mm[0] - cos * center_x - sin * center_y,
            sin,
            -cos,
            center_machine_mm[1] - sin * center_x + cos * center_y,
            0.0,
            0.0,
            1.0,
        ],
    }
}

/// Reflects a top-side design point into the right-handed registration frame.
pub fn top_side_registration_point(bounds: BoardOutlineBounds, x_mm: f64, y_mm: f64) -> (f64, f64) {
    let center_y = 0.5 * (bounds.min_y_mm + bounds.max_y_mm);
    (x_mm, 2.0 * center_y - y_mm)
}

/// Returns the physical top-side footprint angle for a registered board.
pub fn top_side_placement_rotation_deg(
    design_to_machine: Transform2d,
    placement_rotation_deg: f64,
) -> f64 {
    design_to_machine.matrix[3]
        .atan2(design_to_machine.matrix[0])
        .to_degrees()
        + placement_rotation_deg
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Part {
    pub id: Uuid,
    pub key: String,
    pub manufacturer_part_number: Option<String>,
    pub package: String,
    pub height_mm: Option<f64>,
    pub body_width_mm: Option<f64>,
    pub body_length_mm: Option<f64>,
    /// Signed process adjustment. A positive value stops the nozzle higher.
    #[serde(default)]
    pub placement_seating_offset_mm: Option<f64>,
    pub compatible_tips: Vec<String>,
    pub vision_profile: Option<String>,
    /// Rotation semantics used by bottom vision and placement planning.
    #[serde(default)]
    pub orientation: Option<PartOrientation>,
}

/// How bottom vision resolves a package's directed rotation.
///
/// Axis-symmetric parts have no meaningful distinction between angles that
/// differ by 180 degrees. Feeder-referenced parts retain the full angle that
/// was assigned during feeder registration. End-marked parts measure a bright
/// or dark end directly in the bottom-camera image.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case")]
pub enum PartOrientation {
    AxisSymmetric,
    FeederReferenced,
    BrighterEnd {
        minimum_contrast_luma: f64,
        #[serde(default)]
        rotation_offset_deg: f64,
    },
    DarkerEnd {
        minimum_contrast_luma: f64,
        #[serde(default)]
        rotation_offset_deg: f64,
    },
}

impl Default for PartOrientation {
    fn default() -> Self {
        Self::AxisSymmetric
    }
}

impl PartOrientation {
    pub fn validate(self) -> Result<(), &'static str> {
        match self {
            Self::AxisSymmetric | Self::FeederReferenced => Ok(()),
            Self::BrighterEnd {
                minimum_contrast_luma,
                rotation_offset_deg,
            }
            | Self::DarkerEnd {
                minimum_contrast_luma,
                rotation_offset_deg,
            } if minimum_contrast_luma.is_finite()
                && (2.0..=64.0).contains(&minimum_contrast_luma)
                && rotation_offset_deg.is_finite()
                && (-360.0..=360.0).contains(&rotation_offset_deg) =>
            {
                Ok(())
            }
            Self::BrighterEnd { .. } | Self::DarkerEnd { .. } => {
                Err("part end-mark orientation policy is invalid")
            }
        }
    }

    pub fn is_directed(self) -> bool {
        !matches!(self, Self::AxisSymmetric)
    }

    pub fn minimum_end_contrast_luma(self) -> Option<f64> {
        match self {
            Self::BrighterEnd {
                minimum_contrast_luma,
                ..
            }
            | Self::DarkerEnd {
                minimum_contrast_luma,
                ..
            } => Some(minimum_contrast_luma),
            Self::AxisSymmetric | Self::FeederReferenced => None,
        }
    }

    pub fn rotation_offset_deg(self) -> f64 {
        match self {
            Self::BrighterEnd {
                rotation_offset_deg,
                ..
            }
            | Self::DarkerEnd {
                rotation_offset_deg,
                ..
            } => rotation_offset_deg,
            Self::AxisSymmetric | Self::FeederReferenced => 0.0,
        }
    }
}

impl Part {
    /// Returns explicit orientation semantics or a compatible legacy profile.
    pub fn resolved_orientation(&self) -> Option<PartOrientation> {
        self.orientation.or(match self.vision_profile.as_deref() {
            Some("rectangular_passive") => Some(PartOrientation::AxisSymmetric),
            Some("rectangular_directional") => Some(PartOrientation::FeederReferenced),
            Some("rectangular_polarized_bright_end") => Some(PartOrientation::BrighterEnd {
                minimum_contrast_luma: 6.0,
                rotation_offset_deg: 0.0,
            }),
            Some("rectangular_polarized_dark_end") => Some(PartOrientation::DarkerEnd {
                minimum_contrast_luma: 6.0,
                rotation_offset_deg: 0.0,
            }),
            _ => None,
        })
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Feeder {
    Strip {
        id: Uuid,
        name: String,
        part_key: String,
        first_pick: Pose,
        tape_pitch_mm: f64,
        tape_width_mm: f64,
        tape_rotation_deg: f64,
        remaining: Option<u32>,
    },
    Drag {
        id: Uuid,
        name: String,
        part_key: String,
        pick: Pose,
        feed_distance_mm: f64,
        actuator_id: String,
        remaining: Option<u32>,
    },
    Photon {
        id: Uuid,
        name: String,
        part_key: String,
        hardware_uuid: String,
        slot_address: Option<u8>,
        part_pitch_mm: f64,
        pick: Pose,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StripFeederRegistrationStatus {
    VisionDraft,
    LiveValidationRequired,
    Ready,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StripPocketOccupancy {
    Occupied,
    Empty,
    Uncertain,
}

/// Persistent operational state for one strip-feeder pocket.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StripFeederPocketStatus {
    Available,
    Empty,
    Reserved,
    Consumed,
    NeedsReview,
}

/// Last completed action for one strip-feeder pocket.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StripFeederPocketOutcome {
    Returned,
    Consumed,
    RetryableFailure,
    ReleaseUncertain,
    Interrupted,
}

/// Persistent evidence and reservation state for one feed position.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StripFeederPocketState {
    pub pocket_index: u32,
    pub nominal_center_machine_mm: [f64; 2],
    #[serde(default)]
    pub measured_center_machine_mm: Option<[f64; 2]>,
    pub occupancy: StripPocketOccupancy,
    pub occupancy_confidence: f64,
    pub status: StripFeederPocketStatus,
    #[serde(default)]
    pub attempt_count: u32,
    #[serde(default)]
    pub consecutive_failure_count: u32,
    #[serde(default)]
    pub reservation_id: Option<Uuid>,
    #[serde(default)]
    pub last_qualification_id: Option<Uuid>,
    #[serde(default)]
    pub last_outcome: Option<StripFeederPocketOutcome>,
    pub updated_at: DateTime<Utc>,
}

/// One fresh top-camera check of a retained strip-feeder draft.
///
/// The holdout measures local tape position and selects a visible occupied
/// pocket. It does not measure pickup Z, lower a nozzle, or index the tape.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StripFeederLiveHoldoutReport {
    pub algorithm_version: String,
    pub predicted_pocket_machine_mm: [f64; 2],
    #[serde(default)]
    pub measured_pocket_machine_mm: Option<[f64; 2]>,
    pub correction_x_mm: f64,
    pub correction_y_mm: f64,
    pub correction_magnitude_mm: f64,
    pub rotation_correction_deg: f64,
    pub sprocket_center_count: u32,
    pub sprocket_pitch_residual_p95_mm: f64,
    pub occupancy: StripPocketOccupancy,
    pub occupancy_confidence: f64,
    pub confidence: f64,
    pub passed: bool,
    pub artifact_name: String,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Persistent operation state for one live strip-feeder holdout.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StripFeederLiveHoldoutRun {
    pub id: Uuid,
    pub operation_id: Uuid,
    pub registration_id: Uuid,
    pub profile_id: String,
    pub calibration_id: Uuid,
    pub status: crate::OperationStatus,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub sample_count: u32,
    #[serde(default)]
    pub pocket_index: u32,
    pub predicted_pocket_machine_mm: [f64; 2],
    #[serde(default)]
    pub report: Option<StripFeederLiveHoldoutReport>,
    #[serde(default)]
    pub message: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StripFeederPickupDisposition {
    ReturnedToPocket,
    DroppedAtFallback,
    PlacedOnBoard,
    NotReleased,
}

/// Bottom-camera comparison between the centered bare nozzle and a held part.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeldPartInspectionReport {
    pub algorithm_version: String,
    pub detected: bool,
    pub center_x_px: Option<f64>,
    pub center_y_px: Option<f64>,
    pub centering_correction_x_mm: Option<f64>,
    pub centering_correction_y_mm: Option<f64>,
    pub centering_correction_mm: Option<f64>,
    pub width_mm: Option<f64>,
    pub length_mm: Option<f64>,
    /// Undirected package-axis angle in machine coordinates.
    pub rotation_deg: Option<f64>,
    /// Full-angle package direction when an end marker was recognized.
    #[serde(default)]
    pub orientation_rotation_deg: Option<f64>,
    /// Absolute mean-luma separation between the two package ends.
    #[serde(default)]
    pub orientation_contrast_luma: Option<f64>,
    #[serde(default)]
    pub orientation_confidence: f64,
    pub changed_pixel_count: u32,
    pub mean_difference_luma: f64,
    pub confidence: f64,
    pub passed: bool,
    pub artifact_name: String,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Top-camera evidence for one placed package at its planned board pose.
///
/// This report is separate from held-part inspection. The detector matches a
/// known package body and does not infer the pose from package markings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacedPartInspectionReport {
    pub algorithm_version: String,
    pub detected: bool,
    /// Nominal image point before local footprint registration.
    #[serde(default)]
    pub nominal_center_x_px: f64,
    #[serde(default)]
    pub nominal_center_y_px: f64,
    /// True when the bare footprint supplied a local placement target.
    #[serde(default)]
    pub target_registered: bool,
    #[serde(default)]
    pub target_correction_x_mm: Option<f64>,
    #[serde(default)]
    pub target_correction_y_mm: Option<f64>,
    #[serde(default)]
    pub target_correction_mm: Option<f64>,
    #[serde(default)]
    pub target_registration_confidence: f64,
    #[serde(default)]
    pub expected_center_x_px: f64,
    #[serde(default)]
    pub expected_center_y_px: f64,
    #[serde(default)]
    pub expected_width_mm: f64,
    #[serde(default)]
    pub expected_length_mm: f64,
    #[serde(default)]
    pub expected_rotation_deg: f64,
    pub center_x_px: Option<f64>,
    pub center_y_px: Option<f64>,
    #[serde(default)]
    pub offset_x_mm: Option<f64>,
    #[serde(default)]
    pub offset_y_mm: Option<f64>,
    #[serde(default)]
    pub offset_mm: Option<f64>,
    pub width_mm: Option<f64>,
    pub length_mm: Option<f64>,
    pub rotation_deg: Option<f64>,
    #[serde(default)]
    pub rotation_error_deg: Option<f64>,
    pub changed_pixel_count: u32,
    pub mean_difference_luma: f64,
    #[serde(default)]
    pub body_change_fraction: f64,
    #[serde(default)]
    pub change_containment: f64,
    pub confidence: f64,
    pub passed: bool,
    pub artifact_name: String,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// One retained bottom-camera observation during held-part alignment.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeldPartAlignmentObservation {
    pub iteration: u32,
    pub head_machine_mm: [f64; 2],
    pub nozzle_rotation_offset_deg: f64,
    pub inspection: HeldPartInspectionReport,
}

/// Bounded closed-loop centering and rotation evidence for a held part.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HeldPartAlignmentReport {
    pub algorithm_version: String,
    #[serde(default)]
    pub orientation: PartOrientation,
    #[serde(default)]
    pub source_rotation_deg: f64,
    pub target_rotation_deg: f64,
    pub commanded_x_mm: f64,
    pub commanded_y_mm: f64,
    pub commanded_rotation_deg: f64,
    pub final_centering_residual_mm: Option<f64>,
    pub final_rotation_residual_deg: Option<f64>,
    #[serde(default)]
    pub final_resolved_rotation_deg: Option<f64>,
    pub passed: bool,
    pub observations: Vec<HeldPartAlignmentObservation>,
    #[serde(default)]
    pub artifact_names: Vec<String>,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Bottom-camera evidence that a released component is no longer attached to
/// the nozzle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NozzleReleaseVerificationReport {
    pub algorithm_version: String,
    pub attempt_count: u32,
    pub retry_performed: bool,
    pub nozzle_clear: bool,
    pub passed: bool,
    #[serde(default)]
    pub observations: Vec<HeldPartInspectionReport>,
    #[serde(default)]
    pub artifact_names: Vec<String>,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Top-camera evidence that a returned part is present in its source pocket.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StripFeederPocketReturnReport {
    pub algorithm_version: String,
    #[serde(default)]
    pub nominal_center_machine_mm: Option<[f64; 2]>,
    #[serde(default)]
    pub before_camera_pose: Option<Pose>,
    pub before_occupancy: StripPocketOccupancy,
    pub before_confidence: f64,
    #[serde(default)]
    pub before_measured_center_machine_mm: Option<[f64; 2]>,
    #[serde(default)]
    pub before_center_offset_mm: Option<f64>,
    #[serde(default)]
    pub after_camera_pose: Option<Pose>,
    pub after_occupancy: StripPocketOccupancy,
    pub after_confidence: f64,
    #[serde(default)]
    pub after_measured_center_machine_mm: Option<[f64; 2]>,
    #[serde(default)]
    pub after_center_offset_mm: Option<f64>,
    #[serde(default)]
    pub return_center_shift_mm: Option<f64>,
    #[serde(default)]
    pub appearance_similarity: Option<f64>,
    pub passed: bool,
    pub artifact_name: String,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Evidence from one bounded pickup, bottom inspection, and release cycle.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StripFeederPickupQualificationReport {
    pub algorithm_version: String,
    pub nozzle_id: String,
    pub tip_id: String,
    pub sensor_id: String,
    pub pocket_machine_mm: [f64; 2],
    pub pickup_nozzle_z_mm: Option<f64>,
    pub pickup_controller_z_mm: Option<f64>,
    pub open_nozzle_raw_values: Vec<i64>,
    pub pickup_samples: Vec<VacuumProbeSample>,
    pub held_raw_values: Vec<i64>,
    pub held_median_raw_value: Option<f64>,
    pub vacuum_acquired: bool,
    pub vacuum_held_after_transfer: bool,
    pub inspection: Option<HeldPartInspectionReport>,
    #[serde(default)]
    pub alignment: Option<HeldPartAlignmentReport>,
    pub disposition: StripFeederPickupDisposition,
    pub release_surface_machine_mm: Option<[f64; 2]>,
    pub release_raw_values: Vec<i64>,
    pub released: bool,
    #[serde(default)]
    pub nozzle_release_verification: Option<NozzleReleaseVerificationReport>,
    #[serde(default)]
    pub return_verification: Option<StripFeederPocketReturnReport>,
    pub passed: bool,
    pub artifact_names: Vec<String>,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Persistent operation state for a pickup qualification cycle.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StripFeederPickupQualificationRun {
    pub id: Uuid,
    pub operation_id: Uuid,
    pub registration_id: Uuid,
    pub profile_id: String,
    pub calibration_id: Uuid,
    pub status: OperationStatus,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub nozzle_id: String,
    #[serde(default)]
    pub pocket_index: u32,
    #[serde(default)]
    pub fallback_drop_surface_machine_mm: Option<[f64; 2]>,
    #[serde(default)]
    pub report: Option<StripFeederPickupQualificationReport>,
    #[serde(default)]
    pub message: Option<String>,
}

/// Persistent feeder setup state derived from retained workspace evidence.
///
/// `Ready` requires a current live holdout and three successful pickup-return
/// cycles at distinct pockets with one nozzle. Creating a draft cannot create
/// or enable a production `Feeder`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StripFeederRegistration {
    pub id: Uuid,
    pub profile_id: String,
    pub calibration_id: Uuid,
    pub source_scan_id: Uuid,
    pub source_candidate_index: u32,
    pub source_proposal_index: u32,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    pub status: StripFeederRegistrationStatus,
    pub name: String,
    #[serde(default)]
    pub part_key: Option<String>,
    #[serde(default)]
    pub part_rotation_deg: Option<f64>,
    pub tape_width_mm: f64,
    pub tape_pitch_mm: f64,
    pub tape_rotation_deg: f64,
    pub pick_centerline_machine_mm: [[f64; 2]; 2],
    pub pocket_centers_machine_mm: Vec<[f64; 2]>,
    #[serde(default)]
    pub pockets: Vec<StripFeederPocketState>,
    #[serde(default)]
    pub selected_pocket_machine_mm: Option<[f64; 2]>,
    #[serde(default)]
    pub selected_pocket_index: Option<u32>,
    #[serde(default)]
    pub pickup_z_mm: Option<f64>,
    #[serde(default)]
    pub surveyed_part_size_mm: Option<[f64; 2]>,
    #[serde(default)]
    pub measured_part_size_mm: Option<[f64; 2]>,
    #[serde(default)]
    pub measured_part_rotation_deg: Option<f64>,
    #[serde(default)]
    pub measured_part_confidence: Option<f64>,
    #[serde(default)]
    pub active_feeder_id: Option<Uuid>,
    #[serde(default)]
    pub latest_live_holdout_id: Option<Uuid>,
    #[serde(default)]
    pub latest_pickup_qualification_id: Option<Uuid>,
    #[serde(default)]
    pub pickup_sequence_passed: bool,
    #[serde(default)]
    pub latest_pickup_sequence_qualification_ids: Vec<Uuid>,
    pub vision_confidence: f64,
    pub vision_review_required: bool,
    #[serde(default)]
    pub findings: Vec<String>,
}

impl StripFeederRegistration {
    /// Adds conservative pocket state when reading a registration from an
    /// older database document.
    pub fn ensure_pocket_state(&mut self) {
        if !self.pockets.is_empty() {
            return;
        }
        let now = Utc::now();
        self.pockets = self
            .pocket_centers_machine_mm
            .iter()
            .enumerate()
            .filter_map(|(index, center)| {
                let pocket_index = u32::try_from(index).ok()?;
                let selected = self.selected_pocket_index == Some(pocket_index);
                Some(StripFeederPocketState {
                    pocket_index,
                    nominal_center_machine_mm: *center,
                    measured_center_machine_mm: selected
                        .then_some(self.selected_pocket_machine_mm)
                        .flatten(),
                    occupancy: if selected {
                        StripPocketOccupancy::Occupied
                    } else {
                        StripPocketOccupancy::Uncertain
                    },
                    occupancy_confidence: if selected { 1.0 } else { 0.0 },
                    status: if selected {
                        StripFeederPocketStatus::Available
                    } else {
                        StripFeederPocketStatus::NeedsReview
                    },
                    attempt_count: 0,
                    consecutive_failure_count: 0,
                    reservation_id: None,
                    last_qualification_id: None,
                    last_outcome: None,
                    updated_at: now,
                })
            })
            .collect();
    }

    pub fn next_available_pocket(
        &self,
        preferred_index: Option<u32>,
    ) -> Option<&StripFeederPocketState> {
        preferred_index
            .and_then(|index| {
                self.pockets.iter().find(|pocket| {
                    pocket.pocket_index == index
                        && pocket.status == StripFeederPocketStatus::Available
                        && pocket.occupancy == StripPocketOccupancy::Occupied
                })
            })
            .or_else(|| {
                self.pockets
                    .iter()
                    .filter(|pocket| {
                        pocket.status == StripFeederPocketStatus::Available
                            && pocket.occupancy == StripPocketOccupancy::Occupied
                    })
                    .min_by_key(|pocket| pocket.pocket_index)
            })
    }
}

impl Feeder {
    pub fn id(&self) -> Uuid {
        match self {
            Self::Strip { id, .. } | Self::Drag { id, .. } | Self::Photon { id, .. } => *id,
        }
    }

    pub fn part_key(&self) -> &str {
        match self {
            Self::Strip { part_key, .. }
            | Self::Drag { part_key, .. }
            | Self::Photon { part_key, .. } => part_key,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Job {
    pub id: Uuid,
    pub name: String,
    pub boards: Vec<BoardInstance>,
    /// Limits a job to named placements when it is not empty.
    #[serde(default)]
    pub placement_references: Vec<String>,
    /// Optional job-wide override for the part seating process adjustment.
    #[serde(default)]
    pub placement_seating_offset_mm: Option<f64>,
    pub status: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlacementExecutionStage {
    Preflight,
    PickApproach,
    Pickup,
    BottomInspection,
    PlaceApproach,
    Placement,
    Release,
    TopInspection,
    Complete,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacementExecutionStageReport {
    pub stage: PlacementExecutionStage,
    pub passed: bool,
    pub message: String,
}

/// Evidence from one guarded placement-plan step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacementExecutionReport {
    pub algorithm_version: String,
    pub simulated: bool,
    pub placement_reference: String,
    pub part_key: String,
    pub feeder_id: Uuid,
    pub nozzle_id: String,
    #[serde(default)]
    pub part_orientation: PartOrientation,
    #[serde(default)]
    pub feeder_part_rotation_deg: f64,
    pub feeder_pick: Pose,
    pub board_place: Pose,
    #[serde(default)]
    pub part_height_mm: f64,
    #[serde(default)]
    pub placement_surface_offset_mm: f64,
    #[serde(default)]
    pub placement_seating_offset_mm: f64,
    #[serde(default)]
    pub pocket_index: Option<u32>,
    #[serde(default)]
    pub pickup_report: Option<StripFeederPickupQualificationReport>,
    #[serde(default)]
    pub top_inspection: Option<PlacedPartInspectionReport>,
    #[serde(default)]
    pub artifact_names: Vec<String>,
    pub corrected_move_count: u32,
    pub passed: bool,
    #[serde(default)]
    pub stages: Vec<PlacementExecutionStageReport>,
    #[serde(default)]
    pub findings: Vec<String>,
}

/// Persistent state for one guarded placement-plan step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacementExecutionRun {
    pub id: Uuid,
    pub operation_id: Uuid,
    pub job_id: Uuid,
    pub step_index: u32,
    pub calibration_id: Uuid,
    pub status: OperationStatus,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
    #[serde(default)]
    pub report: Option<PlacementExecutionReport>,
    #[serde(default)]
    pub message: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlacementOutcomeDisposition {
    Good,
    Offset,
    Rotated,
    Tombstoned,
    Missing,
    Uncertain,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacementOutcomeRecord {
    pub id: Uuid,
    pub execution_id: Uuid,
    pub job_id: Uuid,
    pub placement_reference: String,
    pub part_key: String,
    pub package: String,
    pub nozzle_id: String,
    pub placement_surface_offset_mm: f64,
    pub placement_seating_offset_mm: f64,
    pub automatic_disposition: PlacementOutcomeDisposition,
    pub automatic_confidence: f64,
    #[serde(default)]
    pub operator_disposition: Option<PlacementOutcomeDisposition>,
    #[serde(default)]
    pub operator_note: Option<String>,
    #[serde(default)]
    pub measured_offset_x_mm: Option<f64>,
    #[serde(default)]
    pub measured_offset_y_mm: Option<f64>,
    #[serde(default)]
    pub measured_rotation_error_deg: Option<f64>,
    pub created_at: DateTime<Utc>,
    pub updated_at: DateTime<Utc>,
}

impl PlacementOutcomeRecord {
    pub fn effective_disposition(&self) -> PlacementOutcomeDisposition {
        self.operator_disposition
            .unwrap_or(self.automatic_disposition)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacementOffsetSummary {
    pub seating_offset_mm: f64,
    pub observation_count: u32,
    pub conclusive_count: u32,
    pub accepted_count: u32,
    pub rejected_count: u32,
    pub success_rate: Option<f64>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlacementProcessEstimate {
    pub part_id: Uuid,
    pub part_key: String,
    pub package: String,
    pub nozzle_id: String,
    pub placement_surface_offset_mm: f64,
    pub current_seating_offset_mm: Option<f64>,
    pub observation_count: u32,
    pub operator_review_count: u32,
    pub offset_summaries: Vec<PlacementOffsetSummary>,
    pub recommended_seating_offset_mm: Option<f64>,
    pub recommendation_confidence: f64,
    pub ready: bool,
    pub findings: Vec<String>,
}

pub fn classify_placement_outcome(
    report: &PlacementExecutionReport,
) -> (PlacementOutcomeDisposition, f64) {
    let Some(inspection) = report.top_inspection.as_ref() else {
        return (PlacementOutcomeDisposition::Uncertain, 0.0);
    };
    if inspection.passed {
        return (PlacementOutcomeDisposition::Good, inspection.confidence);
    }
    if inspection.detected {
        if inspection
            .rotation_error_deg
            .is_some_and(|error| error.abs() > 4.0)
        {
            return (PlacementOutcomeDisposition::Rotated, inspection.confidence);
        }
        return (PlacementOutcomeDisposition::Offset, inspection.confidence);
    }
    if inspection.target_registered
        && inspection.body_change_fraction >= 0.20
        && inspection.change_containment < 0.70
    {
        return (
            PlacementOutcomeDisposition::Tombstoned,
            inspection.confidence,
        );
    }
    if inspection.target_registered && inspection.confidence >= 0.50 {
        return (PlacementOutcomeDisposition::Missing, inspection.confidence);
    }
    (
        PlacementOutcomeDisposition::Uncertain,
        inspection.confidence,
    )
}

pub fn estimate_placement_process(
    part: &Part,
    nozzle_id: &str,
    placement_surface_offset_mm: f64,
    outcomes: &[PlacementOutcomeRecord],
) -> PlacementProcessEstimate {
    use std::collections::BTreeMap;

    const MINIMUM_CONCLUSIVE_OBSERVATIONS: u32 = 3;
    const MINIMUM_SUCCESS_RATE: f64 = 0.80;
    const MAXIMUM_UPDATE_MM: f64 = 0.05;

    let selected = outcomes
        .iter()
        .filter(|outcome| {
            outcome.part_key == part.key
                && outcome.nozzle_id == nozzle_id
                && (outcome.placement_surface_offset_mm - placement_surface_offset_mm).abs()
                    <= 0.005
        })
        .collect::<Vec<_>>();
    let operator_review_count = selected
        .iter()
        .filter(|outcome| outcome.operator_disposition.is_some())
        .count() as u32;
    let mut buckets = BTreeMap::<i32, Vec<&PlacementOutcomeRecord>>::new();
    for outcome in &selected {
        let key = (100.0 * outcome.placement_seating_offset_mm).round() as i32;
        buckets.entry(key).or_default().push(outcome);
    }
    let offset_summaries = buckets
        .into_iter()
        .map(|(key, records)| {
            let accepted_count = records
                .iter()
                .filter(|record| {
                    record.effective_disposition() == PlacementOutcomeDisposition::Good
                })
                .count() as u32;
            let rejected_count = records
                .iter()
                .filter(|record| {
                    !matches!(
                        record.effective_disposition(),
                        PlacementOutcomeDisposition::Good | PlacementOutcomeDisposition::Uncertain
                    )
                })
                .count() as u32;
            let conclusive_count = accepted_count + rejected_count;
            PlacementOffsetSummary {
                seating_offset_mm: f64::from(key) / 100.0,
                observation_count: records.len() as u32,
                conclusive_count,
                accepted_count,
                rejected_count,
                success_rate: (conclusive_count > 0)
                    .then_some(f64::from(accepted_count) / f64::from(conclusive_count)),
            }
        })
        .collect::<Vec<_>>();
    let best = offset_summaries
        .iter()
        .filter(|summary| summary.conclusive_count >= MINIMUM_CONCLUSIVE_OBSERVATIONS)
        .filter(|summary| summary.success_rate.unwrap_or(0.0) >= MINIMUM_SUCCESS_RATE)
        .max_by(|left, right| {
            left.success_rate
                .partial_cmp(&right.success_rate)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(left.conclusive_count.cmp(&right.conclusive_count))
                .then_with(|| {
                    let current = part.placement_seating_offset_mm.unwrap_or(0.0);
                    let left_distance = (left.seating_offset_mm - current).abs();
                    let right_distance = (right.seating_offset_mm - current).abs();
                    right_distance
                        .partial_cmp(&left_distance)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .then_with(|| {
                    right
                        .seating_offset_mm
                        .partial_cmp(&left.seating_offset_mm)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
        });
    let recommended_seating_offset_mm = best.map(|summary| {
        let target = summary.seating_offset_mm.clamp(-0.25, 0.50);
        part.placement_seating_offset_mm.map_or(target, |current| {
            target.clamp(current - MAXIMUM_UPDATE_MM, current + MAXIMUM_UPDATE_MM)
        })
    });
    let recommendation_confidence = best.map_or(0.0, |summary| {
        summary.success_rate.unwrap_or(0.0)
            * (f64::from(summary.conclusive_count) / 10.0).clamp(0.0, 1.0)
    });
    let ready = recommended_seating_offset_mm.is_some();
    let findings = if let Some(best) = best {
        vec![format!(
            "offset {:+.2} mm passed {}/{} conclusive placements",
            best.seating_offset_mm, best.accepted_count, best.conclusive_count
        )]
    } else {
        vec![format!(
            "need at least {MINIMUM_CONCLUSIVE_OBSERVATIONS} conclusive placements at one offset with an {:.0}% pass rate",
            100.0 * MINIMUM_SUCCESS_RATE,
        )]
    };
    PlacementProcessEstimate {
        part_id: part.id,
        part_key: part.key.clone(),
        package: part.package.clone(),
        nozzle_id: nozzle_id.into(),
        placement_surface_offset_mm,
        current_seating_offset_mm: part.placement_seating_offset_mm,
        observation_count: selected.len() as u32,
        operator_review_count,
        offset_summaries,
        recommended_seating_offset_mm,
        recommendation_confidence,
        ready,
        findings,
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InspectionFinding {
    pub id: Uuid,
    pub board_instance_id: Uuid,
    pub placement_reference: String,
    pub kind: String,
    pub confidence: f64,
    pub measured_offset_mm: Option<f64>,
    pub measured_rotation_deg: Option<f64>,
    pub image_artifact: Option<String>,
    pub disposition: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InspectionStatus {
    AwaitingCapture,
    NeedsReview,
    Passed,
    ReworkRequired,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InspectionReport {
    pub id: Uuid,
    pub job_id: Uuid,
    pub created_at: DateTime<Utc>,
    pub status: InspectionStatus,
    pub findings: Vec<InspectionFinding>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn outcome(
        part: &Part,
        nozzle_id: &str,
        offset_mm: f64,
        disposition: PlacementOutcomeDisposition,
    ) -> PlacementOutcomeRecord {
        let now = Utc::now();
        let id = Uuid::new_v4();
        PlacementOutcomeRecord {
            id,
            execution_id: id,
            job_id: Uuid::new_v4(),
            placement_reference: "R1".into(),
            part_key: part.key.clone(),
            package: part.package.clone(),
            nozzle_id: nozzle_id.into(),
            placement_surface_offset_mm: 0.0,
            placement_seating_offset_mm: offset_mm,
            automatic_disposition: disposition,
            automatic_confidence: 0.9,
            operator_disposition: None,
            operator_note: None,
            measured_offset_x_mm: None,
            measured_offset_y_mm: None,
            measured_rotation_error_deg: None,
            created_at: now,
            updated_at: now,
        }
    }

    fn part() -> Part {
        Part {
            id: Uuid::new_v4(),
            key: "R0603:10k".into(),
            manufacturer_part_number: None,
            package: "R_0603_1608Metric".into(),
            height_mm: Some(0.55),
            body_width_mm: Some(0.8),
            body_length_mm: Some(1.6),
            placement_seating_offset_mm: Some(0.10),
            compatible_tips: vec!["N045".into()],
            vision_profile: Some("rectangular_passive".into()),
            orientation: None,
        }
    }

    #[test]
    fn process_estimate_requires_three_conclusive_results() {
        let part = part();
        let mut outcomes = vec![
            outcome(&part, "N1", 0.10, PlacementOutcomeDisposition::Good),
            outcome(&part, "N1", 0.10, PlacementOutcomeDisposition::Good),
        ];
        let estimate = estimate_placement_process(&part, "N1", 0.0, &outcomes);
        assert!(!estimate.ready);
        assert_eq!(estimate.recommended_seating_offset_mm, None);
        assert!(estimate.findings[0].contains("80% pass rate"));

        outcomes.push(outcome(
            &part,
            "N1",
            0.10,
            PlacementOutcomeDisposition::Good,
        ));
        let estimate = estimate_placement_process(&part, "N1", 0.0, &outcomes);
        assert!(estimate.ready);
        assert_eq!(estimate.recommended_seating_offset_mm, Some(0.10));
    }

    #[test]
    fn legacy_rectangular_passive_profile_resolves_axis_symmetry() {
        let part = part();
        assert_eq!(
            part.resolved_orientation(),
            Some(PartOrientation::AxisSymmetric)
        );
    }

    #[test]
    fn explicit_directional_orientation_overrides_the_legacy_profile() {
        let mut part = part();
        part.orientation = Some(PartOrientation::BrighterEnd {
            minimum_contrast_luma: 8.0,
            rotation_offset_deg: 180.0,
        });
        assert_eq!(part.resolved_orientation(), part.orientation);
        assert!(part.resolved_orientation().unwrap().validate().is_ok());
    }

    #[test]
    fn process_estimate_bounds_a_change_from_the_part_default() {
        let part = part();
        let outcomes = (0..3)
            .map(|_| outcome(&part, "N1", 0.20, PlacementOutcomeDisposition::Good))
            .collect::<Vec<_>>();
        let estimate = estimate_placement_process(&part, "N1", 0.0, &outcomes);
        assert!((estimate.recommended_seating_offset_mm.unwrap() - 0.15).abs() < f64::EPSILON);
    }

    #[test]
    fn process_estimate_keeps_board_surfaces_separate() {
        let part = part();
        let outcomes = (0..3)
            .map(|_| {
                let mut outcome = outcome(&part, "N1", 0.10, PlacementOutcomeDisposition::Good);
                outcome.placement_surface_offset_mm = 0.20;
                outcome
            })
            .collect::<Vec<_>>();
        let estimate = estimate_placement_process(&part, "N1", 0.0, &outcomes);
        assert_eq!(estimate.observation_count, 0);
        assert!(!estimate.ready);
    }

    #[test]
    fn operator_review_overrides_the_automatic_disposition() {
        let part = part();
        let mut reviewed = outcome(&part, "N1", 0.10, PlacementOutcomeDisposition::Good);
        reviewed.operator_disposition = Some(PlacementOutcomeDisposition::Rotated);
        assert_eq!(
            reviewed.effective_disposition(),
            PlacementOutcomeDisposition::Rotated
        );
    }
}
