use crate::{OperationStatus, Pose, Residuals, Transform2d};
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
