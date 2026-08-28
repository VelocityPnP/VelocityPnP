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
    pub passed: bool,
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
    pub compatible_tips: Vec<String>,
    pub vision_profile: Option<String>,
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
    pub rotation_deg: Option<f64>,
    pub changed_pixel_count: u32,
    pub mean_difference_luma: f64,
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
    pub target_rotation_deg: f64,
    pub commanded_x_mm: f64,
    pub commanded_y_mm: f64,
    pub commanded_rotation_deg: f64,
    pub final_centering_residual_mm: Option<f64>,
    pub final_rotation_residual_deg: Option<f64>,
    pub passed: bool,
    pub observations: Vec<HeldPartAlignmentObservation>,
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
#[derive(Debug, Clone, Serialize, Deserialize)]
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
/// `Ready` is reserved for a later live camera and pickup-Z workflow. Creating
/// a draft cannot create or enable a production `Feeder`.
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
    pub status: String,
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
